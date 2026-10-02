#pragma once

#include "RawInput.h"
#include "InputEdgeIdentity.h"
#include <algorithm>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

namespace Speed::Input::V2
{
enum class EDesktopSubsourceKind : std::uint8_t { Keyboard, Mouse };
enum class EIngressCause : std::uint8_t { Hardware, Disconnect, Pause, Reset, Replacement, Resync };
struct FDesktopSubsource
{
    EDesktopSubsourceKind Kind = EDesktopSubsourceKind::Keyboard;
    std::uint64_t DeviceId = 0, Generation = 0;
};
inline bool operator==(const FDesktopSubsource& A, const FDesktopSubsource& B)
{ return A.Kind == B.Kind && A.DeviceId == B.DeviceId && A.Generation == B.Generation; }
inline bool operator<(const FDesktopSubsource& A, const FDesktopSubsource& B)
{
    if (A.Kind != B.Kind) return A.Kind < B.Kind;
    if (A.DeviceId != B.DeviceId) return A.DeviceId < B.DeviceId;
    return A.Generation < B.Generation;
}
struct FDesktopControlIdentity { FDesktopSubsource Source; FRawControl Control; };
inline bool operator==(const FDesktopControlIdentity& A, const FDesktopControlIdentity& B)
{ return A.Source == B.Source && A.Control == B.Control; }
struct FDesktopIngressChange
{
    FDesktopControlIdentity Origin;
    float Value = 0;
    std::uint64_t GlobalSequence = 0, LocalSequence = 0, OsTimestampUs = 0, ObservationGroup = 0;
    std::uint32_t WithinObservation = 0;
    EIngressCause Cause = EIngressCause::Hardware;
};
struct FDesktopContribution
{
    FDesktopControlIdentity Origin;
    float Value = 0;
    std::uint64_t LastGlobalSequence = 0;
};
struct FDesktopBaseline
{
    std::uint64_t ProducerId = 0, StreamEpoch = 0, CompositeGeneration = 0;
    std::uint64_t BaselineRevision = 0, AcceptedThrough = 0;
    std::vector<FDesktopSubsource> Members;
    std::vector<FDesktopContribution> Contributions;
};
struct FRawContributionTerminal
{
    FDesktopControlIdentity Origin;
    std::uint64_t ActivationSequence = 0, TerminalSequence = 0;
    EIngressCause Cause = EIngressCause::Disconnect;
};
// Declaration only: this source does not invent action activation identities or
// canonical Started edges. The action-aware participant is a separate gate.
struct FActionContributionTerminal
{
    FRawContributionTerminal Raw;
    FActionId Action = 0;
    std::uint64_t ActionActivationSequence = 0;
    std::optional<FInputEdgeIdentity> StartedEdge;
};
struct FDesktopObservationReceipt
{
    FDesktopSubsource Source;
    std::uint64_t LocalSequence = 0, OsTimestampUs = 0, ObservationGroup = 0;
    // Every observation has an acceptance marker, including an unchanged read.
    std::uint64_t AcceptanceSequence = 0;
    std::size_t FirstChange = 0, ChangeCount = 0;
};
// Canonical little-endian uint64 words: schema, ordering, structural bounds,
// subsource/raw enum codes, lifecycle cause codes, and observation/marker rule
// versions. Changing one of these values changes the contract fingerprint.
inline constexpr std::uint64_t DesktopContractWords[] = {
    1, 1, 16, 4096, 256, 256, 256, 256,
    static_cast<std::uint64_t>(EDesktopSubsourceKind::Keyboard),
    static_cast<std::uint64_t>(EDesktopSubsourceKind::Mouse),
    static_cast<std::uint64_t>(ERawDeviceKind::Desktop),
    static_cast<std::uint64_t>(ERawControlKind::KeyboardUsage),
    static_cast<std::uint64_t>(ERawControlKind::MouseButton),
    static_cast<std::uint64_t>(EMouseButton::Left),
    static_cast<std::uint64_t>(EMouseButton::Right),
    static_cast<std::uint64_t>(EMouseButton::Middle),
    static_cast<std::uint64_t>(EMouseButton::Count),
    static_cast<std::uint64_t>(EIngressCause::Hardware),
    static_cast<std::uint64_t>(EIngressCause::Disconnect),
    static_cast<std::uint64_t>(EIngressCause::Pause),
    static_cast<std::uint64_t>(EIngressCause::Reset),
    static_cast<std::uint64_t>(EIngressCause::Replacement),
    static_cast<std::uint64_t>(EIngressCause::Resync), 1, 1
};
constexpr std::uint64_t DesktopContractFingerprint() noexcept
{
    std::uint64_t H = 14695981039346656037ULL;
    for (const auto Word : DesktopContractWords)
        for (unsigned Byte = 0; Byte < 8; ++Byte)
        { H ^= (Word >> (Byte * 8)) & 255; H *= 1099511628211ULL; }
    return H;
}
struct FDesktopPublication
{
    static constexpr std::uint32_t SchemaVersion = 1, IngressOrderingVersion = 1;
    // Fingerprint names the fixed DTO/order contract, not a cryptographic seal.
    static constexpr std::uint64_t ContractFingerprint = DesktopContractFingerprint();
    std::uint64_t Fingerprint = ContractFingerprint;
    std::uint32_t Schema = SchemaVersion, Ordering = IngressOrderingVersion;
    FDesktopBaseline Baseline;
    std::vector<FDesktopIngressChange> Changes;
    std::vector<FDesktopObservationReceipt> Observations;
    std::vector<FRawContributionTerminal> RawTerminals;
    std::vector<FRawValue> PreviousUnion;
    FRawInputSample Sample;
    bool RequiresLifecycleEnvelope = false;
    bool OverflowRecovery = false;
    std::uint64_t LifecycleWatermark = 0, RejectedWatermark = 0;
};
struct FDesktopFullObservation
{
    FDesktopSubsource Source;
    std::uint64_t LocalSequence = 0, OsTimestampUs = 0;
    std::vector<FRawValue> State; // Full capability list, sorted, neutral included.
};
struct FDesktopLifecycleNotice
{
    FDesktopSubsource Source;
    EIngressCause Cause = EIngressCause::Disconnect;
};
enum class EDesktopBatchMode : std::uint8_t { Poll, FreshBaseline, Pause, Reset, Retire };
struct FDesktopBatch
{
    std::uint32_t Schema = FDesktopPublication::SchemaVersion;
    std::uint32_t Ordering = FDesktopPublication::IngressOrderingVersion;
    std::uint64_t Fingerprint = FDesktopPublication::ContractFingerprint;
    EDesktopBatchMode Mode = EDesktopBatchMode::Poll;
    // FreshBaseline replaces the entire registry atomically. Other modes retain
    // it; membership mutations may never be smuggled through a Poll.
    std::vector<FDesktopFullObservation> Observations;
    bool RecoverOverflow = false;
    std::size_t RawTerminalCapacity = 4096;
};
enum class EDesktopAdmission : std::uint8_t
{ Ready, WrongOwner, Invalid, Stale, NeedsFreshBaseline, OverflowFence, Capacity, Exhausted, Retired };

// Portable acquisition seam only. No OS polling, mapper, journal publication,
// physical-frame reservation or action-aware terminal ledger lives here.
// In particular this is NOT a production transaction participant. Activation
// stays prohibited until provenance and action-terminal capacity are prepared
// together by the journal/mapper; Sample alone must never cross that gate.
class FDesktopInputSource
{
#if defined(SPEED_DESKTOP_INPUT_PROBE)
    friend struct FDesktopInputSourceTestAccess;
#endif
public:
    static constexpr std::size_t MaxMembers = 16, MaxContributions = 4096;
    static constexpr std::size_t MaxChanges = 256, MaxUnion = 256, MaxNotices = 256;
    static constexpr std::size_t MaxObservations = 256;
    struct FPumpTicket
    {
        std::uint64_t Revision = 0, NoticeWatermark = 0;
        bool Overflow = false;
        std::vector<FDesktopSubsource> Members;
    };
private:
    struct FMember
    {
        FDesktopSubsource Source;
        std::uint64_t LocalSequence = 0;
        std::vector<FDesktopContribution> Contributions;
    };
    struct FState
    {
        std::uint64_t Revision = 0, Sequence = 0, Observation = 0, Generation = 0;
        std::uint64_t MaxMemberGeneration = 0;
        bool RequiresFresh = true, Retired = false;
        std::vector<FMember> Members;
    };
public:
    class FPrepared
    {
        friend class FDesktopInputSource;
        const FDesktopInputSource* Owner = nullptr;
        FPumpTicket Ticket;
        FState Next;
        std::shared_ptr<const FDesktopPublication> Publication;
        FPrepared() = default;
    public:
        const FDesktopPublication* Inspect() const { return Owner ? Publication.get() : nullptr; }
        FPrepared(const FPrepared&) = delete;
        FPrepared& operator=(const FPrepared&) = delete;
    };
    struct FPreparation
    {
        EDesktopAdmission Status = EDesktopAdmission::Invalid;
        std::unique_ptr<FPrepared> Prepared;
    };
    FDesktopInputSource(std::uint64_t Producer, std::uint64_t Epoch)
        : ProducerId(Producer), StreamEpoch(Epoch), Owner(std::this_thread::get_id())
    { Notices.reserve(MaxNotices); }

    static bool CanAdvanceSequence(std::uint64_t Value, std::uint64_t Count = 1) noexcept
    { return Count <= std::numeric_limits<std::uint64_t>::max() - Value; }

    // The only multi-thread entry. Notifications cannot publish or assign raw
    // ingress identities. Saturation remains sticky until explicit recovery.
    bool EnqueueLifecycle(FDesktopLifecycleNotice Notice)
    {
        std::lock_guard<std::mutex> Lock(Gate);
        if (!ValidSource(Notice.Source) || (Notice.Cause != EIngressCause::Disconnect
            && Notice.Cause != EIngressCause::Replacement)) return false;
        MaxNotifiedGeneration = std::max(MaxNotifiedGeneration, Notice.Source.Generation);
        if (NoticeWatermark == UINT64_MAX) { Overflow = true; return false; }
        ++NoticeWatermark;
        // Failed enqueues also invalidate an outstanding recovery ticket.
        if (Notices.size() == MaxNotices) { Overflow = true; return false; }
        Notices.push_back(Notice); return true;
    }
    std::optional<FPumpTicket> CapturePump() const
    {
        if (!IsOwner()) return {};
        std::lock_guard<std::mutex> Lock(Gate);
        FPumpTicket Ticket{State.Revision, NoticeWatermark, Overflow, {}};
        for (const auto& M : State.Members) Ticket.Members.push_back(M.Source);
        return Ticket;
    }
    std::shared_ptr<const FDesktopPublication> ReadPublished() const
    { std::lock_guard<std::mutex> Lock(Gate); return (Overflow || EpochExhausted) ? nullptr : Published; }

    FPreparation Prepare(const FPumpTicket& Ticket, const FDesktopBatch& Batch) const
    {
        if (!IsOwner()) return Fail(EDesktopAdmission::WrongOwner);
        std::lock_guard<std::mutex> Lock(Gate);
        if (!Matches(Ticket)) return Fail(EDesktopAdmission::Stale);
        if (State.Retired) return Fail(EDesktopAdmission::Retired);
        if (EpochExhausted) return Exhaust();
        if (NoticeWatermark == UINT64_MAX) return Exhaust();
        if (!ProducerId || !StreamEpoch || Batch.Schema != FDesktopPublication::SchemaVersion
            || Batch.Ordering != FDesktopPublication::IngressOrderingVersion
            || Batch.Fingerprint != FDesktopPublication::ContractFingerprint
            || Batch.Mode > EDesktopBatchMode::Retire) return Fail(EDesktopAdmission::Invalid);
        const bool Fresh = Batch.Mode == EDesktopBatchMode::FreshBaseline;
        const bool Terminal = Batch.Mode == EDesktopBatchMode::Pause
            || Batch.Mode == EDesktopBatchMode::Reset || Batch.Mode == EDesktopBatchMode::Retire;
        if (Overflow && !(Fresh && Batch.RecoverOverflow)) return Fail(EDesktopAdmission::OverflowFence);
        if (Batch.RecoverOverflow != Overflow) return Fail(EDesktopAdmission::Invalid);
        if (State.RequiresFresh && !Fresh && !Terminal) return Fail(EDesktopAdmission::NeedsFreshBaseline);
        if (Batch.Observations.size() > MaxObservations || (Fresh && Batch.Observations.size() > MaxMembers))
            return Fail(EDesktopAdmission::Capacity);
        if (Terminal && !Batch.Observations.empty()) return Fail(EDesktopAdmission::Invalid);
        if (!Fresh && !Terminal && !Notices.empty()) return Fail(EDesktopAdmission::NeedsFreshBaseline);
        if (!CanAdvanceSequence(State.Revision) || !CanAdvanceSequence(State.Sequence)
            || ((Fresh || Terminal) && State.Generation == UINT64_MAX)) return Exhaust();
        auto Prepared = std::unique_ptr<FPrepared>(new FPrepared);
        Prepared->Owner = this; Prepared->Ticket = Ticket; Prepared->Next = State;
        auto& Next = Prepared->Next;
        // Registry generations are epoch-wide monotone allocations. Even a
        // departure observed before first admission burns that generation.
        Next.MaxMemberGeneration = std::max(Next.MaxMemberGeneration, MaxNotifiedGeneration);
        auto Pub = std::make_shared<FDesktopPublication>();
        Pub->PreviousUnion = Union(State.Members);
        Pub->LifecycleWatermark = Ticket.NoticeWatermark;
        Pub->OverflowRecovery = Overflow; Pub->RejectedWatermark = Overflow ? Ticket.NoticeWatermark : 0;
        Pub->RequiresLifecycleEnvelope = Fresh || Terminal;
        ++Next.Revision;
        // Each publication has a boundary marker, so empty polls still advance
        // the inclusive accepted watermark without manufacturing a control edge.
        ++Next.Sequence;
        if (Fresh || Terminal) ++Next.Generation;
        if (Fresh)
        {
            std::vector<FMember> Replacements;
            std::uint64_t MaximumGeneration = Next.MaxMemberGeneration;
            for (std::size_t I = 0; I < Batch.Observations.size(); ++I)
            {
                const auto& O = Batch.Observations[I];
                if (!ValidObservation(O) || (I && !(Batch.Observations[I - 1].Source < O.Source)))
                    return Fail(EDesktopAdmission::Invalid);
                for (const auto& M : Replacements)
                    if ((M.Source.Kind == O.Source.Kind && M.Source.DeviceId == O.Source.DeviceId)
                        || M.Source.Generation == O.Source.Generation) return Fail(EDesktopAdmission::Invalid);
                const auto* Old = Find(State.Members, O.Source);
                // Lost lifecycle notices cannot prove any old generation live.
                // Recovery enumeration must issue fresh registry generations.
                if (Overflow && O.Source.Generation <= Next.MaxMemberGeneration) return Fail(EDesktopAdmission::Stale);
                if (!Old && O.Source.Generation <= Next.MaxMemberGeneration) return Fail(EDesktopAdmission::Stale);
                if (Old && O.LocalSequence <= Old->LocalSequence) return Fail(EDesktopAdmission::Stale);
                for (const auto& N : Notices) if (Invalidates(N, O.Source)) return Fail(EDesktopAdmission::Stale);
                MaximumGeneration = std::max(MaximumGeneration, O.Source.Generation);
                FMember M; M.Source = O.Source; M.LocalSequence = O.LocalSequence;
                if (!AppendObservation(Next, *Pub, O)) return Exhaust();
                for (const auto& V : O.State)
                {
                    std::uint64_t Activation = Pub->Observations.back().AcceptanceSequence;
                    if (Old && !State.RequiresFresh && !Overflow)
                        for (const auto& C : Old->Contributions)
                            if (C.Origin.Control == V.Control && C.Value == V.Value) Activation = C.LastGlobalSequence;
                    M.Contributions.push_back({{O.Source, V.Control}, V.Value, Activation});
                }
                Replacements.push_back(std::move(M));
            }
            Next.MaxMemberGeneration = MaximumGeneration;
            // Compare old contributions to the complete post-boundary registry.
            // There is never a visible remove-before-install intermediate union.
            for (const auto& M : State.Members) for (const auto& C : M.Contributions) if (C.Value != 0)
            {
                bool Retained = false;
                if (!State.RequiresFresh && !Overflow)
                    if (const auto* New = Find(Replacements, M.Source))
                        for (const auto& V : New->Contributions)
                            if (V.Origin.Control == C.Origin.Control && V.Value == C.Value) Retained = true;
                if (!Retained)
                {
                    EIngressCause Cause = EIngressCause::Resync;
                    for (const auto& N : Notices) if (Invalidates(N, M.Source)) Cause = N.Cause;
                    if (!AddTerminal(Next, *Pub, C, Cause, Batch.RawTerminalCapacity))
                        return Pub->RawTerminals.size() >= Batch.RawTerminalCapacity || Pub->RawTerminals.size() >= MaxContributions
                            ? Fail(EDesktopAdmission::Capacity) : Exhaust();
                }
            }
            Next.Members = std::move(Replacements); Next.RequiresFresh = false;
        }
        else if (Terminal)
        {
            const auto Cause = Batch.Mode == EDesktopBatchMode::Pause ? EIngressCause::Pause
                : Batch.Mode == EDesktopBatchMode::Reset ? EIngressCause::Reset : EIngressCause::Replacement;
            for (auto& M : Next.Members) for (auto& C : M.Contributions) if (C.Value != 0)
            {
                auto EffectiveCause = Cause;
                for (const auto& N : Notices) if (Invalidates(N, M.Source)) EffectiveCause = N.Cause;
                if (!AddTerminal(Next, *Pub, C, EffectiveCause, Batch.RawTerminalCapacity))
                    return Pub->RawTerminals.size() >= Batch.RawTerminalCapacity || Pub->RawTerminals.size() >= MaxContributions
                            ? Fail(EDesktopAdmission::Capacity) : Exhaust();
                C.Value = 0; C.LastGlobalSequence = Next.Sequence;
            }
            // Consuming a disconnect notice at a paused/terminal boundary must
            // also retire its membership. Otherwise a later fresh read could
            // revive that same generation after the notice has been drained.
            Next.Members.erase(std::remove_if(Next.Members.begin(), Next.Members.end(),
                [&](const FMember& M)
                {
                    for (const auto& Notice : Notices) if (Invalidates(Notice, M.Source)) return true;
                    return false;
                }), Next.Members.end());
            Next.RequiresFresh = true; Next.Retired = Batch.Mode == EDesktopBatchMode::Retire;
        }
        else
        {
            for (std::size_t I = 0; I < Batch.Observations.size(); ++I)
            {
                const auto& O = Batch.Observations[I];
                if (!ValidObservation(O) || (I && O.Source < Batch.Observations[I - 1].Source))
                    return Fail(EDesktopAdmission::Invalid);
                auto* M = Find(Next.Members, O.Source);
                if (!M || M->LocalSequence == UINT64_MAX || O.LocalSequence != M->LocalSequence + 1)
                    return Fail(EDesktopAdmission::Stale);
                if (M->Contributions.size() != O.State.size()) return Fail(EDesktopAdmission::Invalid);
                for (std::size_t J = 0; J < O.State.size(); ++J)
                    if (!(M->Contributions[J].Origin.Control == O.State[J].Control)) return Fail(EDesktopAdmission::Invalid);
                const auto Before = Union(Next.Members);
                if (!AppendObservation(Next, *Pub, O)) return Exhaust();
                auto& Receipt = Pub->Observations.back();
                for (std::size_t J = 0; J < O.State.size(); ++J)
                {
                    auto& C = M->Contributions[J];
                    if (C.Value == O.State[J].Value) continue;
                    if (Pub->Changes.size() == MaxChanges) return Fail(EDesktopAdmission::Capacity);
                    if (Next.Sequence == UINT64_MAX) return Exhaust();
                    ++Next.Sequence;
                    C.Value = O.State[J].Value; C.LastGlobalSequence = Next.Sequence;
                    Pub->Changes.push_back({C.Origin, C.Value, Next.Sequence, O.LocalSequence, O.OsTimestampUs,
                        Receipt.ObservationGroup, static_cast<std::uint32_t>(Receipt.ChangeCount++), EIngressCause::Hardware});
                }
                M->LocalSequence = O.LocalSequence;
                const auto After = Union(Next.Members);
                std::uint32_t Within = 0;
                for (std::size_t J = 0; J < After.size(); ++J) if (Before[J].Value != After[J].Value)
                    Pub->Sample.Changes.push_back({After[J], {Next.Sequence, Within++}, Receipt.ObservationGroup});
            }
        }
        if (Next.Members.size() > MaxMembers) return Fail(EDesktopAdmission::Capacity);
        auto& B = Pub->Baseline;
        B.ProducerId = ProducerId; B.StreamEpoch = StreamEpoch; B.CompositeGeneration = Next.Generation;
        B.BaselineRevision = Next.Revision; B.AcceptedThrough = Next.Sequence;
        for (const auto& M : Next.Members)
        {
            B.Members.push_back(M.Source);
            if (M.Contributions.size() > MaxContributions - B.Contributions.size()) return Fail(EDesktopAdmission::Capacity);
            B.Contributions.insert(B.Contributions.end(), M.Contributions.begin(), M.Contributions.end());
        }
        auto& S = Pub->Sample;
        S.DeviceId = ProducerId; S.Generation.Value = Next.Generation; S.Kind = ERawDeviceKind::Desktop;
        S.Sequence = Next.Sequence; S.Status = Pub->RequiresLifecycleEnvelope ? ERawSampleStatus::Resync : ERawSampleStatus::Valid;
        S.FinalState = Union(Next.Members);
        if (S.FinalState.size() > MaxUnion || S.Changes.size() > MaxChanges) return Fail(EDesktopAdmission::Capacity);
        if (S.FinalState.empty() && !Pub->RequiresLifecycleEnvelope) return Fail(EDesktopAdmission::NeedsFreshBaseline);
        // Empty registry is a valid lifecycle envelope, never a valid ordinary
        // raw sample. Downstream must consume that envelope explicitly.
        if (!S.FinalState.empty() && !S.IsValid()) return Fail(EDesktopAdmission::Invalid);
        Prepared->Publication = std::move(Pub);
        return {EDesktopAdmission::Ready, std::move(Prepared)};
    }

    // All allocations and validation happened in Prepare. The only publication
    // mutation is a swap under the notification fence, after the final check.
    EDesktopAdmission Commit(FPrepared& Prepared)
    {
        if (!IsOwner()) return EDesktopAdmission::WrongOwner;
        std::lock_guard<std::mutex> Lock(Gate);
        if (EpochExhausted) return EDesktopAdmission::Exhausted;
        if (Prepared.Owner != this || !Prepared.Publication || !Matches(Prepared.Ticket)) return EDesktopAdmission::Stale;
        using std::swap; swap(State, Prepared.Next); Published.swap(Prepared.Publication);
        Notices.clear(); Overflow = false; Prepared.Owner = nullptr;
        return EDesktopAdmission::Ready;
    }

private:
    FPreparation Exhaust() const
    { EpochExhausted = true; return Fail(EDesktopAdmission::Exhausted); }
    static FPreparation Fail(EDesktopAdmission Status) { return {Status, {}}; }
    bool IsOwner() const { return std::this_thread::get_id() == Owner; }
    static bool ValidSource(const FDesktopSubsource& S)
    { return S.Kind <= EDesktopSubsourceKind::Mouse && S.DeviceId && S.Generation; }
    static bool Invalidates(const FDesktopLifecycleNotice& N, const FDesktopSubsource& S)
    { return N.Source.Kind == S.Kind && N.Source.DeviceId == S.DeviceId && S.Generation <= N.Source.Generation; }
    static bool ValidObservation(const FDesktopFullObservation& O)
    {
        if (!ValidSource(O.Source) || !O.LocalSequence || O.State.empty() || O.State.size() > MaxUnion) return false;
        for (std::size_t I = 0; I < O.State.size(); ++I)
        {
            const auto& V = O.State[I];
            if (!V.Control.Accepts(V.Value) || (I && !(O.State[I - 1].Control < V.Control))
                || (O.Source.Kind == EDesktopSubsourceKind::Keyboard
                    ? V.Control.Kind != ERawControlKind::KeyboardUsage : V.Control.Kind != ERawControlKind::MouseButton)) return false;
        }
        return true;
    }
    template<class T> static auto Find(T& Members, const FDesktopSubsource& Source) -> decltype(&Members.front())
    { for (auto& M : Members) if (M.Source == Source) return &M; return nullptr; }
    static std::vector<FRawValue> Union(const std::vector<FMember>& Members)
    {
        std::vector<FRawValue> Result;
        for (const auto& M : Members) for (const auto& C : M.Contributions)
        {
            auto It = std::lower_bound(Result.begin(), Result.end(), C.Origin.Control,
                [](const FRawValue& V, FRawControl Control) { return V.Control < Control; });
            if (It == Result.end() || !(It->Control == C.Origin.Control)) Result.insert(It, {C.Origin.Control, C.Value});
            else It->Value = std::max(It->Value, C.Value);
        }
        return Result;
    }
    static bool AppendObservation(FState& S, FDesktopPublication& P, const FDesktopFullObservation& O)
    {
        if (S.Observation == UINT64_MAX || S.Sequence == UINT64_MAX) return false;
        ++S.Observation; ++S.Sequence;
        P.Observations.push_back({O.Source, O.LocalSequence, O.OsTimestampUs, S.Observation, S.Sequence, P.Changes.size(), 0});
        return true;
    }
    static bool AddTerminal(FState& S, FDesktopPublication& P, const FDesktopContribution& C,
        EIngressCause Cause, std::size_t Capacity)
    {
        if (P.RawTerminals.size() >= Capacity || P.RawTerminals.size() >= MaxContributions || S.Sequence == UINT64_MAX) return false;
        ++S.Sequence; P.RawTerminals.push_back({C.Origin, C.LastGlobalSequence, S.Sequence, Cause}); return true;
    }
    bool Matches(const FPumpTicket& T) const
    {
        if (T.Revision != State.Revision || T.NoticeWatermark != NoticeWatermark || T.Overflow != Overflow
            || T.Members.size() != State.Members.size()) return false;
        for (std::size_t I = 0; I < T.Members.size(); ++I) if (!(T.Members[I] == State.Members[I].Source)) return false;
        return true;
    }
    const std::uint64_t ProducerId, StreamEpoch;
    const std::thread::id Owner;
    mutable std::mutex Gate;
    FState State;
    std::vector<FDesktopLifecycleNotice> Notices;
    std::uint64_t NoticeWatermark = 0;
    std::uint64_t MaxNotifiedGeneration = 0; // Includes notices lost at overflow.
    bool Overflow = false;
    mutable bool EpochExhausted = false; // Retains evidence, fences all future admission.
    std::shared_ptr<const FDesktopPublication> Published;
};
}
