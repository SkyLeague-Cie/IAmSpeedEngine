#pragma once
#include "IAmSpeed/Input/InputProducerV2.h"
#include <mutex>
#include <thread>

namespace Speed::Netcode
{
namespace V2 = Speed::Input::V2;
enum class ERemoteSubmission { Accepted, Duplicate, Conflict, Invalid, Closed, Stale, TooFar };

// Candidate portable V2 producer. It preserves original source identity, never
// relabels itself as AI/scenario/device, and DOES NOT bypass existing ExactRemote
// admission guards. Host must authenticate/validate packets before SubmitFrame.
class FExactRemoteInputProducer final : public V2::IInputProducer, public V2::IInputProducerPollFence
{
public:
    static std::unique_ptr<FExactRemoteInputProducer> Create(
        std::shared_ptr<const V2::FInputActionContract> Contract,
        V2::FStreamEpoch Epoch, Input::FProducerIdentity Identity, Input::FFrameNumber First)
    {
        if (Input::FPresentationInputScope::IsActive() || !Contract || !Epoch.Value || !Identity.Id
            || Identity.Kind > Input::EProducerKind::Network) return {};
        return std::unique_ptr<FExactRemoteInputProducer>(new FExactRemoteInputProducer(std::move(Contract), Epoch, Identity, First));
    }
    V2::IInputProducerPollFence* PollFence() noexcept override { return this; }
    bool SupportsPollFence() const noexcept override { return true; }
    V2::EProducerContract GetProducerContract() const noexcept override { return V2::EProducerContract::ExactRemote; }
    const std::shared_ptr<const V2::FInputActionContract>& GetContract() const override { return Contract; }

    ERemoteSubmission SubmitFrame(const V2::FInputFrame& Frame)
    {
        if (Input::FPresentationInputScope::IsActive() || !Frame.IsValidFor(*Contract) || !Address(Frame)) return ERemoteSubmission::Invalid;
        auto Immutable = std::make_shared<const V2::FInputFrame>(Frame);
        std::lock_guard<std::mutex> Lock(Gate);
        if (Closed || Quarantined) return ERemoteSubmission::Closed;
        const auto N = Frame.GetData().ConsumptionFrame;
        for (const auto* Table : {&Pending, &History}) for (const auto& P : *Table)
            if (P && P->GetData().ConsumptionFrame == N) return Same(*P, Frame) ? ERemoteSubmission::Duplicate : ERemoteSubmission::Conflict;
        if (Exhausted || N < Next) return ERemoteSubmission::Stale;
        if (N - Next >= Capacity) return ERemoteSubmission::TooFar;
        Pending[N % Capacity] = std::move(Immutable);
        return ERemoteSubmission::Accepted;
    }
    // Admission hint before entering the owner's irreversible Poll transaction.
    // A missing frame stays absent. This performs no source consume or polling.
    bool HasExactFrame(Input::FFrameNumber N) const
    {
        std::lock_guard<std::mutex> Lock(Gate);
        return !Closed && !Quarantined && !Paused && !Exhausted && N == Next && Pending[N % Capacity]
            && Pending[N % Capacity]->GetData().ConsumptionFrame == N;
    }
    std::optional<V2::FInputFrame> InspectBaseline(Input::FFrameNumber N) const override
    {
        if (Input::FPresentationInputScope::IsActive()) return {};
        std::lock_guard<std::mutex> Lock(Gate);
        const auto& P = Pending[N % Capacity];
        // Inspection may expose an explicitly received later Reset on the same
        // owner/epoch. It neither invents one nor consumes/clears Last. A host
        // must still authorize its checkpoint/session/physical-state transaction.
        if (Closed || Quarantined || Exhausted || Frozen || N != Next || !P
            || P->GetData().ConsumptionFrame != N || !P->GetData().Reset || !Continuity(*P)) return {};
        return *P;
    }
    std::optional<V2::FInputPollCutoff> FreezeForOwner(Input::FFrameNumber N) override
    {
        if (Input::FPresentationInputScope::IsActive()) return {};
        std::lock_guard<std::mutex> Lock(Gate);
        const auto Caller = std::this_thread::get_id();
        if (Owner && *Owner != Caller) return {};
        if (Closed || Quarantined || Paused || Exhausted || Frozen || N != Next || LeaseSerial == UINT64_MAX) return {};
        const auto& P = Pending[N % Capacity];
        if (!P || P->GetData().ConsumptionFrame != N) return {};
        Owner = Caller;
        if (!Continuity(*P)) { Quarantined = true; return {}; }
        Frozen = P; Delivered = false; ++LeaseSerial;
        Cutoff = V2::FInputPollCutoff{P->GetData().SourceSequence, P->GetData().DeviceGeneration.Value, LeaseSerial};
        return Cutoff;
    }
    std::optional<V2::FInputFrame> Produce(Input::FFrameNumber N) override
    {
        if (Input::FPresentationInputScope::IsActive()) return {};
        std::lock_guard<std::mutex> Lock(Gate);
        if (!IsOwner() || !Frozen || Closed || Quarantined || Delivered || N != Next) return {};
        std::optional<V2::FInputFrame> Result(*Frozen); // all allocation before Delivered
        Delivered = true;
        return Result;
    }
    bool CloseFrozenCutoff(const V2::FInputPollCutoff& C) noexcept override
    {
        std::lock_guard<std::mutex> Lock(Gate);
        if (!IsOwner() || !Frozen) return false; // foreign call cannot release owner's lease
        const bool Good = Delivered && C.Sequence == Cutoff.Sequence && C.Generation == Cutoff.Generation
            && C.LifecycleFence == Cutoff.LifecycleFence && !Closed && !Quarantined;
        if (Good)
        {
            History[Next % Capacity] = Frozen; Last = Frozen; Pending[Next % Capacity].reset();
            Exhausted = Next == UINT64_MAX; if (!Exhausted) ++Next;
        }
        else Quarantined = true;
        Frozen.reset(); Delivered = false; Cutoff = {};
        return Good;
    }
    V2::ELifecycleResult SetLifecyclePaused(bool Value) override
    {
        if (Input::FPresentationInputScope::IsActive()) return V2::ELifecycleResult::Rejected;
        std::lock_guard<std::mutex> Lock(Gate);
        if ((Owner && !IsOwner()) || Frozen || Closed || Quarantined) return V2::ELifecycleResult::Rejected;
        Paused = Value; // exact timeline is not rewritten into a synthetic neutral baseline
        return V2::ELifecycleResult::Applied;
    }
    V2::ELifecycleResult CancelLifecycle() override
    {
        if (Input::FPresentationInputScope::IsActive()) return V2::ELifecycleResult::Rejected;
        std::lock_guard<std::mutex> Lock(Gate);
        if ((Owner && !IsOwner()) || Frozen) return V2::ELifecycleResult::Rejected;
        Closed = true; Pending = {}; History = {}; Last.reset();
        return V2::ELifecycleResult::Applied;
    }
private:
    static constexpr std::size_t Capacity = 64;
    FExactRemoteInputProducer(std::shared_ptr<const V2::FInputActionContract> C,
        V2::FStreamEpoch E, Input::FProducerIdentity I, Input::FFrameNumber N)
        : Contract(std::move(C)), Epoch(E), Identity(I), Next(N) {}
    bool IsOwner() const { return Owner && *Owner == std::this_thread::get_id(); }
    bool Address(const V2::FInputFrame& F) const
    { const auto& D = F.GetData(); return D.StreamEpoch.Value == Epoch.Value && D.Producer.Id == Identity.Id && D.Producer.Kind == Identity.Kind; }
    static bool Same(const V2::FInputFrame& A, const V2::FInputFrame& B)
    {
        const auto& X = A.GetData(); const auto& Y = B.GetData();
        if (X.Version != Y.Version || X.SourceSequence != Y.SourceSequence || X.ConsumptionFrame != Y.ConsumptionFrame
            || X.DeviceGeneration.Value != Y.DeviceGeneration.Value || X.Reset != Y.Reset || X.Values != Y.Values
            || X.ActiveMask != Y.ActiveMask || X.Transitions.size() != Y.Transitions.size()) return false;
        for (std::size_t N = 0; N < X.Transitions.size(); ++N)
        {
            const auto& E = X.Transitions[N]; const auto& F = Y.Transitions[N];
            if (E.Action != F.Action || E.State != F.State || E.ValueAtTransition != F.ValueAtTransition
                || E.Order.Sequence != F.Order.Sequence || E.Order.WithinSequence != F.Order.WithinSequence) return false;
        }
        return true;
    }
    bool Continuity(const V2::FInputFrame& F) const
    {
        const auto& D = F.GetData(); if (!Last) return D.Reset;
        const auto& Before = Last->GetData(); if (D.SourceSequence < Before.SourceSequence) return false;
        if (D.Reset) return D.DeviceGeneration.Value >= Before.DeviceGeneration.Value && D.SourceSequence > Before.SourceSequence;
        if (D.DeviceGeneration.Value != Before.DeviceGeneration.Value) return false;
        if (D.SourceSequence == Before.SourceSequence)
            return D.Values == Before.Values && D.ActiveMask == Before.ActiveMask && D.Transitions.empty();
        auto Active = Before.ActiveMask;
        for (const auto& E : D.Transitions)
        {
            const auto Bit = std::uint32_t{1} << E.Action; const bool Started = E.State == V2::ETransition::Started;
            if (E.Order.Sequence <= Before.SourceSequence || (bool(Active & Bit) == Started)) return false;
            if (Started) Active |= Bit; else Active &= ~Bit;
        }
        return Active == D.ActiveMask;
    }
    const std::shared_ptr<const V2::FInputActionContract> Contract;
    const V2::FStreamEpoch Epoch;
    const Input::FProducerIdentity Identity;
    mutable std::mutex Gate;
    Input::FFrameNumber Next;
    std::optional<std::thread::id> Owner;
    std::array<std::shared_ptr<const V2::FInputFrame>, Capacity> Pending{}, History{};
    std::shared_ptr<const V2::FInputFrame> Last, Frozen;
    V2::FInputPollCutoff Cutoff;
    std::uint64_t LeaseSerial = 0;
    bool Delivered = false, Closed = false, Paused = false, Quarantined = false, Exhausted = false;
};
}
