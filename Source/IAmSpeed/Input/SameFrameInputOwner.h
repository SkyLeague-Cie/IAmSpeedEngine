#pragma once

#include "InputProducerV2.h"
#include <thread>

namespace Speed::Input::V2
{
struct FInputPollCutoff { std::uint64_t Sequence = 0, Generation = 0, LifecycleFence = 0; };
// Freeze retains actual immutable source evidence before Produce, not just a
// counter. Later ordinary arrivals remain for the next poll. Lifecycle changes
// or overflow invalidate the fence. Unadapted production producers are refused.
class IInputProducerPollFence
{
public:
    virtual ~IInputProducerPollFence() = default;
    virtual std::optional<FInputPollCutoff> FreezeForOwner(FFrameNumber N) = 0;
    // Atomic final validation + release under the source gate; false also closes
    // the source lease. A zero cutoff closes an exceptional/failed preparation.
    virtual bool CloseFrozenCutoff(const FInputPollCutoff& Cutoff) noexcept = 0;
};
enum class EOwnerInputStatus : std::uint8_t
{
    Ready, WrongOwner, WrongFrame, Busy, Paused, InvalidBinding,
    ResyncRequired, Quarantined, Exhausted, InvalidToken, ReplayComplete
};
enum class EOwnerInputPhase : std::uint8_t { Idle, Prepared, Installed, Stepping, Paused, Quarantined };
struct FInputProcessingPolicy
{
    // Zero means direct. Nonzero means maximum quantized change in this step.
    std::array<std::uint16_t, ActionCount> Step{};
};
struct FOwnerInputBinding
{
    std::uint64_t AdapterId = 0;
    FProducerIdentity Producer;
    FStreamEpoch Epoch;
    std::shared_ptr<const FInputActionContract> Contract;
    FInputProcessingPolicy Processing;
};
struct FOwnerInputSnapshot
{
    FInputFrame Input;
    FContractFingerprint BindingFingerprint;
    FActionValues Requested{}, Applied{};
    // Input contains all ordered edges, including multiple edges of one action.
    std::uint64_t CutoffSequence = 0;
};
struct FOwnerInputReceipt
{
    std::shared_ptr<const FOwnerInputSnapshot> Snapshot;
    FFrameNumber AppliedBeforeStep = 0, StepFrame = 0;
    std::uint64_t Epoch = 0, ReplayPass = 0;
    // Replay receipts are internal correction evidence, never fresh presentation.
    bool Replay = false;
};

// Portable seam only. No UObject, component writes, worker installation,
// physical effect execution, AI/network adapter or production activation.
// The sole owner takes unique producer ownership and never exposes it.
// Produce's SourceSequence must match the frozen pre-poll cutoff. Source adapters
// must freeze their source evidence atomically before Produce (tested using an
// IAmSpeed-owned fake inbox); concurrent arrivals afterwards belong to a later
// poll. This class never accepts pushed PhysicalInput or pre-applied targets.
class FSameFrameInputOwner final
{
public:
    class FToken
    {
        friend class FSameFrameInputOwner;
        std::shared_ptr<const std::uint8_t> Identity;
        explicit FToken(std::shared_ptr<const std::uint8_t> InIdentity) : Identity(std::move(InIdentity)) {}
    public:
        FToken() = default;
        FToken(const FToken&) = delete;
        FToken& operator=(const FToken&) = delete;
        FToken(FToken&&) noexcept = default;
        FToken& operator=(FToken&&) noexcept = default;
    };
    struct FPreparation
    {
        EOwnerInputStatus Status = EOwnerInputStatus::InvalidBinding;
        std::optional<FToken> Token;
    };
    static std::unique_ptr<FSameFrameInputOwner> Create(std::unique_ptr<IInputProducer> Producer,
        FOwnerInputBinding Binding, FFrameNumber FirstFrame = 0)
    {
        if (FPresentationInputScope::IsActive() || !ValidBinding(Producer.get(), Binding)) return {};
        return std::unique_ptr<FSameFrameInputOwner>(new FSameFrameInputOwner(std::move(Producer), std::move(Binding), FirstFrame));
    }
    FSameFrameInputOwner(const FSameFrameInputOwner&) = delete;
    FSameFrameInputOwner& operator=(const FSameFrameInputOwner&) = delete;

    FPreparation Poll(FFrameNumber N)
    {
        if (!IsOwner()) return {EOwnerInputStatus::WrongOwner, {}};
        if (FPresentationInputScope::IsActive()) return {EOwnerInputStatus::Busy, {}};
        if (CallingSource) { Quarantine(); return {EOwnerInputStatus::Quarantined, {}}; }
        if (Phase == EOwnerInputPhase::Quarantined) return {EOwnerInputStatus::Quarantined, {}};
        if (Phase == EOwnerInputPhase::Paused) return {EOwnerInputStatus::Paused, {}};
        if (Phase != EOwnerInputPhase::Idle) return {EOwnerInputStatus::Busy, {}};
        if (Replay && ReplayFinished) return {EOwnerInputStatus::ReplayComplete, {}};
        if ((!Replay && Exhausted) || PollSerial == UINT64_MAX) return {EOwnerInputStatus::Exhausted, {}};
        if (N != (Replay ? ReplayNext : NextFrame)) return {EOwnerInputStatus::WrongFrame, {}};
        try
        {
            std::optional<FInputFrame> Input;
            if (Replay)
            {
                const auto& Saved = History[N % HistoryCapacity];
                if (!Saved || Saved->Snapshot->Input.GetData().ConsumptionFrame != N)
                    return Reject(EOwnerInputStatus::ResyncRequired);
                Input = Saved->Snapshot->Input;
            }
            else
            {
                CallingSource = true;
                auto* Fence = dynamic_cast<IInputProducerPollFence*>(Source.get());
                std::optional<FInputPollCutoff> Cutoff;
                bool ValidCutoff = false;
                try
                {
                    Cutoff = Fence->FreezeForOwner(N);
                    if (Phase != EOwnerInputPhase::Quarantined && Cutoff && Cutoff->Sequence && Cutoff->Generation && Cutoff->LifecycleFence)
                    {
                        ++PollSerial; Input = Source->Produce(N);
                        ValidCutoff = Input
                            && Input->GetData().SourceSequence == Cutoff->Sequence
                            && Input->GetData().DeviceGeneration.Value == Cutoff->Generation;
                    }
                }
                catch (...) { ValidCutoff = false; }
                const bool ClosedValid = Fence->CloseFrozenCutoff(Cutoff.value_or(FInputPollCutoff{}));
                ValidCutoff = ValidCutoff && ClosedValid;
                CallingSource = false;
                if (Phase == EOwnerInputPhase::Quarantined) return {EOwnerInputStatus::Quarantined, {}};
                if (!ValidCutoff) return Reject(EOwnerInputStatus::ResyncRequired);
            }
            if (!Input || !Input->IsValidFor(*Binding.Contract) || !ValidateAddress(*Input, N))
                return Reject(EOwnerInputStatus::ResyncRequired);
            if (!Replay && !ValidateContinuity(*Input)) return Reject(EOwnerInputStatus::ResyncRequired);
            const auto Before = Replay ? ReplayApplied : Applied;
            const auto Processed = Process(*Input, Before);
            if (Replay && Processed != History[N % HistoryCapacity]->Snapshot->Applied)
                return Reject(EOwnerInputStatus::ResyncRequired);
            auto Snapshot = std::make_shared<const FOwnerInputSnapshot>(FOwnerInputSnapshot{
                *Input, Fingerprint, Input->GetData().Values, Processed, Input->GetData().SourceSequence});
            auto Receipt = std::make_shared<const FOwnerInputReceipt>(FOwnerInputReceipt{
                Snapshot, N, N, Binding.Epoch.Value, Replay ? ReplayPass : 0, Replay});
            auto Entry = std::make_shared<const FHistoryEntry>(FHistoryEntry{Snapshot, Receipt, Before});
            auto Identity = std::make_shared<const std::uint8_t>(0);
            Pending = FPending{std::move(Identity), std::move(Entry)};
            Phase = EOwnerInputPhase::Prepared;
            return {EOwnerInputStatus::Ready, FToken(Pending->Identity)};
        }
        catch (...) { CallingSource = false; return Reject(EOwnerInputStatus::ResyncRequired); }
    }

    // All storage was reserved by Poll. Installed input is immutable, visible
    // only to this owner through the token, never to publication readers.
    bool Install(const FToken& Token) noexcept
    {
        if (!Allowed(Token, EOwnerInputPhase::Prepared)) return false;
        Phase = EOwnerInputPhase::Installed; return true;
    }
    std::shared_ptr<const FOwnerInputSnapshot> ReadInstalled(const FToken& Token) const noexcept
    {
        if (!IsOwner() || CallingSource || FPresentationInputScope::IsActive() || !Matches(Token)
            || (Phase != EOwnerInputPhase::Installed && Phase != EOwnerInputPhase::Stepping)) return {};
        return Pending->Entry->Snapshot;
    }
    bool BeginStep(const FToken& Token, FFrameNumber N) noexcept
    {
        if (!Allowed(Token, EOwnerInputPhase::Installed) || N != Pending->Entry->Snapshot->Input.GetData().ConsumptionFrame) return false;
        Phase = EOwnerInputPhase::Stepping; return true;
    }
    // This portable endpoint is called by the future simulation adapter ONLY
    // after its real step and all participants validate. It does not itself run
    // physics or certify a gameplay effect. No callback/allocation in commit.
    bool CompleteStep(const FToken& Token, FFrameNumber N) noexcept
    {
        if (!Allowed(Token, EOwnerInputPhase::Stepping) || N != Pending->Entry->Snapshot->Input.GetData().ConsumptionFrame) return false;
        if (Replay)
        {
            ReplayApplied = Pending->Entry->Snapshot->Applied;
            ReplayReceipt = Pending->Entry->Receipt;
            if (N == ReplayEnd) ReplayFinished = true; else ++ReplayNext;
        }
        else
        {
            Applied = Pending->Entry->Snapshot->Applied;
            History[N % HistoryCapacity] = Pending->Entry;
            Latest = Pending->Entry->Receipt;
            LastInput = Pending->Entry->Snapshot;
            RequireBaseline = false;
            Exhausted = N == UINT64_MAX;
            if (!Exhausted) NextFrame = N + 1;
        }
        Pending.reset(); Phase = EOwnerInputPhase::Idle; return true;
    }
    // An aborted producer poll cannot be repeated: its source cursor may have
    // advanced. Quarantine the epoch, retain completed evidence only, and require
    // an explicit new boundary binding. Never publish partially applied input.
    bool Abort(const FToken& Token) noexcept
    {
        if (!IsOwner() || FPresentationInputScope::IsActive() || !Matches(Token)) return false;
        Quarantine(); return true;
    }

    bool PauseAtBoundary()
    {
        if (!IsOwner() || FPresentationInputScope::IsActive() || CallingSource || Replay || Phase != EOwnerInputPhase::Idle) return false;
        CallingSource = true;
        ELifecycleResult Result = ELifecycleResult::Rejected;
        try { Result = Source->SetLifecyclePaused(true); } catch (...) {}
        CallingSource = false;
        if (Phase == EOwnerInputPhase::Quarantined || (Result != ELifecycleResult::Applied && Result != ELifecycleResult::Unaffected)) { Quarantine(); return false; }
        Applied = {}; RequireBaseline = true; Latest.reset(); Phase = EOwnerInputPhase::Paused;
        return true;
    }
    bool ResumeAtBoundary()
    {
        if (!IsOwner() || FPresentationInputScope::IsActive() || CallingSource || Phase != EOwnerInputPhase::Paused) return false;
        CallingSource = true;
        ELifecycleResult Result = ELifecycleResult::Rejected;
        try { Result = Source->SetLifecyclePaused(false); } catch (...) {}
        CallingSource = false;
        if (Phase == EOwnerInputPhase::Quarantined || (Result != ELifecycleResult::Applied && Result != ELifecycleResult::Unaffected)) { Quarantine(); return false; }
        Phase = EOwnerInputPhase::Idle; return true;
    }
    // Replacement is a quiescent source/owner boundary, never a per-frame GT
    // selection. A new epoch is mandatory even when adapter/source is unchanged.
    bool RebindAtBoundary(std::unique_ptr<IInputProducer> Producer, FOwnerInputBinding NewBinding, FFrameNumber FirstFrame)
    {
        if (!IsOwner() || FPresentationInputScope::IsActive() || CallingSource || Pending || Replay
            || (Phase != EOwnerInputPhase::Idle && Phase != EOwnerInputPhase::Paused && Phase != EOwnerInputPhase::Quarantined)
            || NewBinding.Epoch.Value <= Binding.Epoch.Value) return false;
        const auto PriorPhase = Phase;
        const auto PriorFault = FaultSerial;
        if (PriorFault == UINT64_MAX) return false;
        CallingSource = true;
        bool Valid = false;
        FContractFingerprint NewFingerprint;
        try { Valid = ValidBinding(Producer.get(), NewBinding); if (Valid) NewFingerprint = FingerprintFor(NewBinding); }
        catch (...) { Valid = false; }
        CallingSource = false;
        if (!Valid || Phase != PriorPhase || FaultSerial != PriorFault || Pending) return false;
        // Retire the old source under the same reentrancy fence; no new binding
        // can become active if its cancellation/destructor reenters this owner.
        CallingSource = true;
        ELifecycleResult Cancelled = ELifecycleResult::Unaffected;
        try { if (Source) Cancelled = Source->CancelLifecycle(); } catch (...) { Cancelled = ELifecycleResult::Rejected; }
        if (Cancelled != ELifecycleResult::Applied && Cancelled != ELifecycleResult::Unaffected) Quarantine();
        if (Phase != PriorPhase || FaultSerial != PriorFault) { CallingSource = false; return false; }
        Source.reset();
        CallingSource = false;
        if (Phase != PriorPhase || FaultSerial != PriorFault) return false;
        Source = std::move(Producer); Binding = std::move(NewBinding); Fingerprint.swap(NewFingerprint);
        History = {}; Applied = {}; LastInput.reset(); Latest.reset(); ReplayReceipt.reset();
        NextFrame = FirstFrame; Exhausted = false; RequireBaseline = true; Phase = EOwnerInputPhase::Idle;
        return true;
    }
    bool BeginReplay(FFrameNumber First, FFrameNumber Last)
    {
        if (!IsOwner() || FPresentationInputScope::IsActive() || CallingSource || Replay || Phase != EOwnerInputPhase::Idle
            || First > Last || Last - First >= HistoryCapacity || ReplayPass == UINT64_MAX) return false;
        for (FFrameNumber N = First;; ++N)
        {
            const auto& E = History[N % HistoryCapacity];
            if (!E || E->Snapshot->Input.GetData().ConsumptionFrame != N || E->Snapshot->BindingFingerprint != Fingerprint) return false;
            if (N == Last) break;
        }
        ReplayApplied = History[First % HistoryCapacity]->Before;
        Replay = true; ReplayFinished = false; ReplayNext = First; ReplayEnd = Last;
        ++ReplayPass; ReplayReceipt.reset(); return true;
    }
    bool EndReplay() noexcept
    {
        if (!IsOwner() || FPresentationInputScope::IsActive() || !Replay || !ReplayFinished || Phase != EOwnerInputPhase::Idle) return false;
        Replay = false; ReplayFinished = false; ReplayReceipt.reset(); return true;
    }
    std::shared_ptr<const FOwnerInputReceipt> ReadLatest() const noexcept
    { return IsOwner() && Phase != EOwnerInputPhase::Quarantined && Phase != EOwnerInputPhase::Paused ? Latest : nullptr; }
    std::shared_ptr<const FOwnerInputReceipt> ReadReplayReceipt() const noexcept
    { return IsOwner() && Replay && Phase != EOwnerInputPhase::Quarantined ? ReplayReceipt : nullptr; }
    std::shared_ptr<const FOwnerInputReceipt> ReadCompleted(FFrameNumber N) const noexcept
    {
        if (!IsOwner()) return {};
        const auto& E = History[N % HistoryCapacity];
        return E && E->Snapshot->Input.GetData().ConsumptionFrame == N ? E->Receipt : nullptr;
    }
    FActionValues ReadBoundaryApplied() const noexcept
    { return IsOwner() && Phase != EOwnerInputPhase::Quarantined ? Applied : FActionValues{}; }
    EOwnerInputPhase GetPhase() const noexcept { return IsOwner() ? Phase : EOwnerInputPhase::Quarantined; }
    FFrameNumber GetNextFrame() const noexcept { return IsOwner() ? NextFrame : UINT64_MAX; }
    std::uint64_t GetPollSerial() const noexcept { return IsOwner() ? PollSerial : 0; }
    FContractFingerprint GetBindingFingerprint() const { return IsOwner() ? Fingerprint : FContractFingerprint{}; }

private:
    struct FHistoryEntry
    {
        std::shared_ptr<const FOwnerInputSnapshot> Snapshot;
        std::shared_ptr<const FOwnerInputReceipt> Receipt;
        FActionValues Before{};
    };
    struct FPending
    {
        std::shared_ptr<const std::uint8_t> Identity;
        std::shared_ptr<const FHistoryEntry> Entry;
    };
    FSameFrameInputOwner(std::unique_ptr<IInputProducer> Producer, FOwnerInputBinding InBinding, FFrameNumber First)
        : Source(std::move(Producer)), Binding(std::move(InBinding)), Fingerprint(FingerprintFor(Binding)),
          Owner(std::this_thread::get_id()), NextFrame(First) {}
    bool IsOwner() const noexcept { return std::this_thread::get_id() == Owner; }
    bool Matches(const FToken& T) const noexcept { return Pending && T.Identity && Pending->Identity == T.Identity; }
    bool Allowed(const FToken& T, EOwnerInputPhase Required) const noexcept
    { return IsOwner() && !FPresentationInputScope::IsActive() && !CallingSource && Phase == Required && Matches(T); }
    void Quarantine() noexcept
    {
        Pending.reset(); Phase = EOwnerInputPhase::Quarantined;
        Latest.reset(); ReplayReceipt.reset(); Applied = {};
        Replay = false; ReplayFinished = false;
        if (FaultSerial != UINT64_MAX) ++FaultSerial;
    }
    FPreparation Reject(EOwnerInputStatus S) noexcept { Quarantine(); return {S, {}}; }
    static bool ValidBinding(const IInputProducer* Source, const FOwnerInputBinding& B)
    {
        // This phase does NOT implement AI/network V2 or enable their routes.
        if (!Source || !dynamic_cast<const IInputProducerPollFence*>(Source) || !B.AdapterId || !B.Producer.Id || B.Producer.Kind != EProducerKind::Device
            || !B.Epoch.Value || !B.Contract) return false;
        try
        {
            const auto C = Source->GetContract();
            if (!C || C->GetFingerprint() != B.Contract->GetFingerprint()) return false;
        }
        catch (...) { return false; }
        for (std::size_t I = 0; I < ActionCount; ++I)
        {
            const auto* A = B.Contract->Find(static_cast<FActionId>(I));
            if (B.Processing.Step[I] && (!A || A->Wiring != EActionWiring::Wired || A->Type != EActionType::Axis1D
                || B.Processing.Step[I] > static_cast<std::uint16_t>(2 * A->Quantization))) return false;
        }
        return true;
    }
    static FContractFingerprint FingerprintFor(const FOwnerInputBinding& B)
    {
        FContractFingerprint F = B.Contract->GetFingerprint();
        const auto Add = [&](std::uint64_t V) { for (unsigned I = 0; I < 8; ++I) F.push_back(static_cast<std::uint8_t>(V >> (8 * I))); };
        Add(1); Add(B.AdapterId); Add(static_cast<std::uint64_t>(B.Producer.Kind)); Add(B.Producer.Id); Add(B.Epoch.Value);
        for (const auto S : B.Processing.Step) Add(S);
        return F;
    }
    bool ValidateAddress(const FInputFrame& F, FFrameNumber N) const noexcept
    {
        const auto& D = F.GetData();
        return D.ConsumptionFrame == N && D.StreamEpoch.Value == Binding.Epoch.Value
            && D.Producer.Kind == Binding.Producer.Kind && D.Producer.Id == Binding.Producer.Id;
    }
    bool ValidateContinuity(const FInputFrame& F) const
    {
        const auto& D = F.GetData();
        if (RequireBaseline && !D.Reset) return false;
        if (!LastInput) return D.Reset;
        const auto& Old = LastInput->Input.GetData();
        if (D.SourceSequence < Old.SourceSequence) return false;
        if (D.Reset) return D.DeviceGeneration.Value >= Old.DeviceGeneration.Value && D.SourceSequence > Old.SourceSequence;
        if (D.DeviceGeneration.Value != Old.DeviceGeneration.Value) return false;
        if (D.SourceSequence == Old.SourceSequence) return D.Values == Old.Values && D.ActiveMask == Old.ActiveMask && D.Transitions.empty();
        auto Active = Old.ActiveMask;
        for (const auto& E : D.Transitions)
        {
            const auto Bit = std::uint32_t{1} << E.Action;
            const bool Started = E.State == ETransition::Started;
            if (E.Order.Sequence <= Old.SourceSequence || (((Active & Bit) != 0) == Started)) return false;
            if (Started) Active |= Bit; else Active &= ~Bit;
        }
        return Active == D.ActiveMask;
    }
    FActionValues Process(const FInputFrame& Input, const FActionValues& Before) const noexcept
    {
        auto Result = Input.GetData().Values;
        if (Input.GetData().Reset) return Result; // first fresh held baseline is immediate
        for (std::size_t I = 0; I < ActionCount; ++I)
        {
            const int Step = Binding.Processing.Step[I];
            if (!Step) continue;
            const int Delta = int(Result[I]) - int(Before[I]);
            Result[I] = static_cast<std::int16_t>(int(Before[I]) + std::clamp(Delta, -Step, Step));
        }
        return Result;
    }
    std::unique_ptr<IInputProducer> Source;
    FOwnerInputBinding Binding;
    FContractFingerprint Fingerprint;
    const std::thread::id Owner;
    EOwnerInputPhase Phase = EOwnerInputPhase::Idle;
    bool CallingSource = false, RequireBaseline = true, Exhausted = false;
    FFrameNumber NextFrame = 0;
    std::uint64_t PollSerial = 0;
    std::uint64_t FaultSerial = 0;
    FActionValues Applied{};
    std::optional<FPending> Pending;
    std::array<std::shared_ptr<const FHistoryEntry>, HistoryCapacity> History{};
    std::shared_ptr<const FOwnerInputSnapshot> LastInput;
    std::shared_ptr<const FOwnerInputReceipt> Latest, ReplayReceipt;
    bool Replay = false, ReplayFinished = false;
    FFrameNumber ReplayNext = 0, ReplayEnd = 0;
    std::uint64_t ReplayPass = 0;
    FActionValues ReplayApplied{};
};
}
