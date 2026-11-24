#pragma once

#include "ExactRemoteRoundControlWire.h"
#include <algorithm>
#include <chrono>
#include <set>

namespace Speed::Netcode::RoundControl
{
using FClock = std::chrono::steady_clock;
enum class EResult
{
    Accepted, Duplicate, Invalid, WrongPeer, WrongSession, WrongRound, Stale,
    OutOfOrder, Conflict, Rejected, Expired, Closed
};

// A stale, future, foreign or out-of-order message leaves an open gate intact.
// A contradictory message from the bound peer for this attempt closes the
// gate until a new reservation. Exact valid duplicates change no state.

namespace Detail
{
inline EResult MatchKey(const FKey& Expected, const FKey& Received) noexcept
{
    if (Expected == Received) return EResult::Accepted;
    if (Received.Match != Expected.Match || Received.MatchEpoch != Expected.MatchEpoch)
        return EResult::WrongRound;
    if (Received.Generation < Expected.Generation
        || (Received.Generation == Expected.Generation && Received.Reservation < Expected.Reservation))
        return EResult::Stale;
    if (Received.Generation == Expected.Generation && Received.Reservation == Expected.Reservation)
        return EResult::Conflict;
    return EResult::WrongRound;
}

inline bool MatchesBaseline(const FBinding& Binding, const FPacket& Packet) noexcept
{
    return Binding.Matches(Packet) && Packet.Input.ProducerId == Binding.SourceProducer
        && Packet.Input.ProducerKind == Binding.SourceKind;
}
}

// Keep each gate on one control lane. AuthenticatedPeer is the identity from
// the established transport, never an identity asserted by the message.
class FHostRoundControl final
{
public:
    FHostRoundControl(const FHostRoundControl&) = delete;
    FHostRoundControl& operator=(const FHostRoundControl&) = delete;
    FHostRoundControl(FHostRoundControl&&) noexcept = default;
    FHostRoundControl& operator=(FHostRoundControl&&) noexcept = default;

    static std::optional<FHostRoundControl> Create(const FKey& Key, const std::vector<FBinding>& Bindings,
        std::uint32_t BudgetMilliseconds, FClock::time_point Now)
    {
        const auto Budget = std::chrono::milliseconds(BudgetMilliseconds);
        if (!Key.Valid() || Bindings.empty() || Bindings.size() > Netcode::Window
            || !BudgetMilliseconds || BudgetMilliseconds > MaxBudgetMilliseconds
            || Now >= FClock::time_point::max() - Budget) return {};

        FHostRoundControl Gate(Key, BudgetMilliseconds, Now + Budget);
        std::set<U64> Sessions, Actors, Peers;
        Gate.Slots.reserve(Bindings.size());
        for (const auto& Binding : Bindings)
        {
            if (!Binding.Valid() || Binding.ServerPeer != Bindings.front().ServerPeer
                || !Sessions.insert(Binding.Session).second || !Actors.insert(Binding.Actor).second
                || !Peers.insert(Binding.OwnerPeer).second) return {};
            Gate.Slots.push_back({Binding, {}, {}, false});
        }
        return Gate;
    }

    const FKey& Key() const noexcept { return Round; }
    FClock::time_point Deadline() const noexcept { return Expires; }
    void CancelBeforeRelease() noexcept { if (!bReleased) bFailed = true; }

    std::optional<FMessage> ReserveFor(U64 Session, FClock::time_point Now) const
    {
        auto Slot = Find(Session);
        if (Slot == Slots.end() || !Open(Now) || bReleased) return {};
        return Make(*Slot, FReserve{BudgetMilliseconds});
    }

    EResult ReceiveFromClient(U64 AuthenticatedPeer, const FMessage& M, FClock::time_point Now)
    {
        if (!Valid(M)) return EResult::Invalid;
        const auto Tag = Kind(M);
        if (Tag != EKind::ReservationResult && Tag != EKind::Baseline && Tag != EKind::Applied)
            return EResult::Invalid;
        auto Slot = Find(M.Session.Session);
        if (Slot == Slots.end()) return EResult::WrongSession;
        if (Slot->Binding.OwnerPeer != AuthenticatedPeer) return EResult::WrongPeer;
        if (!(M.Session == FSessionAddress::From(Slot->Binding))) return EResult::WrongSession;
        if (Now >= Expires) return EResult::Expired;
        const auto Match = Detail::MatchKey(Round, M.Key);
        if (Match != EResult::Accepted) return Match == EResult::Conflict ? Conflict() : Match;
        if (bFailed) return EResult::Closed;

        if (const auto* Result = std::get_if<FReservationResult>(&M.Payload))
        {
            if (Slot->Reserved) return *Slot->Reserved == Result->Accepted ? EResult::Duplicate : Conflict();
            Slot->Reserved = Result->Accepted;
            if (!Result->Accepted) { bFailed = true; return EResult::Rejected; }
            return EResult::Accepted;
        }
        if (const auto* Baseline = std::get_if<FBaseline>(&M.Payload))
        {
            if (!Slot->Reserved.value_or(false)) return EResult::OutOfOrder;
            if (!Detail::MatchesBaseline(Slot->Binding, Baseline->Packet)) return Conflict();
            if (Slot->Baseline) return Netcode::Same(*Slot->Baseline, Baseline->Packet) ? EResult::Duplicate : Conflict();
            Slot->Baseline = Baseline->Packet;
            return EResult::Accepted;
        }

        if (!Neutral) return EResult::OutOfOrder;
        if (!(*Neutral == std::get<FApplied>(M.Payload).Step)) return Conflict();
        if (Slot->Applied) return EResult::Duplicate;
        Slot->Applied = true;
        return EResult::Accepted;
    }

    bool AllReserved(FClock::time_point Now) const noexcept
    {
        return Open(Now) && std::all_of(Slots.begin(), Slots.end(),
            [](const FSlot& Slot) { return Slot.Reserved.value_or(false); });
    }

    bool ReadyForWorldCommit(FClock::time_point Now) const noexcept
    {
        return AllReserved(Now) && std::all_of(Slots.begin(), Slots.end(),
            [](const FSlot& Slot) { return Slot.Baseline.has_value(); });
    }

    // The game still validates ToV2 against the pinned input contract and joins
    // each captured frame to the physical cutoff before building the plan.
    // The returned view belongs to this gate and its control lane.
    const FPacket* BaselineFor(U64 Session, FClock::time_point Now) const noexcept
    {
        auto Slot = Find(Session);
        return Slot != Slots.end() && ReadyForWorldCommit(Now) ? &*Slot->Baseline : nullptr;
    }

    // The caller supplies the witness from its actual completed neutral step.
    EResult ObserveCompletedNeutral(const FKey& Key, const FPublication& Step, FClock::time_point Now) noexcept
    {
        if (!Key.Valid() || !Step.Valid()) return EResult::Invalid;
        if (Now >= Expires) return EResult::Expired;
        const auto Match = Detail::MatchKey(Round, Key);
        if (Match != EResult::Accepted) return Match == EResult::Conflict ? Conflict() : Match;
        if (bFailed || Slots.empty()) return EResult::Closed;
        if (Neutral) return *Neutral == Step ? EResult::Duplicate : Conflict();
        if (!ReadyForWorldCommit(Now)) return EResult::OutOfOrder;
        Neutral = Step;
        return EResult::Accepted;
    }

    std::optional<FMessage> PublicationFor(U64 Session, FClock::time_point Now) const
    {
        auto Slot = Find(Session);
        if (Slot == Slots.end() || !Open(Now) || !Neutral) return {};
        return Make(*Slot, FPublished{*Neutral});
    }

    EResult AuthorizeRelease(const FKey& Key, const FPublication& Step, FClock::time_point Now) noexcept
    {
        if (!Key.Valid() || !Step.Valid()) return EResult::Invalid;
        if (Now >= Expires) return EResult::Expired;
        const auto Match = Detail::MatchKey(Round, Key);
        if (Match != EResult::Accepted) return Match == EResult::Conflict ? Conflict() : Match;
        if (bFailed || Slots.empty()) return EResult::Closed;
        if (!Neutral) return EResult::OutOfOrder;
        if (!(*Neutral == Step)) return Conflict();
        if (bReleased) return EResult::Duplicate;
        if (!std::all_of(Slots.begin(), Slots.end(), [](const FSlot& Slot) { return Slot.Applied; }))
            return EResult::OutOfOrder;
        bReleased = true;
        return EResult::Accepted;
    }

    // Use this to issue the one-time release before the deadline. The caller
    // owns the resulting gameplay phase and the wait for fresh F+1 input.
    bool GameplayReleased(const FKey& Key, const FPublication& Step, FClock::time_point Now) const noexcept
    { return Open(Now) && bReleased && Round == Key && Neutral && *Neutral == Step; }

    std::optional<FMessage> ReleaseFor(U64 Session, FClock::time_point Now) const
    {
        auto Slot = Find(Session);
        if (Slot == Slots.end() || !Open(Now) || !bReleased) return {};
        return Make(*Slot, FReleased{*Neutral});
    }

private:
    struct FSlot
    {
        FBinding Binding;
        std::optional<bool> Reserved;
        std::optional<FPacket> Baseline;
        bool Applied = false;
    };

    FHostRoundControl(const FKey& Key, std::uint32_t Budget, FClock::time_point Deadline)
        : Round(Key), BudgetMilliseconds(Budget), Expires(Deadline) {}

    std::vector<FSlot>::iterator Find(U64 Session) noexcept
    { return std::find_if(Slots.begin(), Slots.end(), [&](const FSlot& S) { return S.Binding.Session == Session; }); }
    std::vector<FSlot>::const_iterator Find(U64 Session) const noexcept
    { return std::find_if(Slots.begin(), Slots.end(), [&](const FSlot& S) { return S.Binding.Session == Session; }); }
    FMessage Make(const FSlot& Slot, FPayload Payload) const
    { return {Round, FSessionAddress::From(Slot.Binding), std::move(Payload)}; }
    bool Open(FClock::time_point Now) const noexcept { return !Slots.empty() && Now < Expires && !bFailed; }
    // Once released, later anomalies remain errors but cannot revoke F+1.
    EResult Conflict() noexcept { if (!bReleased) bFailed = true; return EResult::Conflict; }

    FKey Round;
    std::uint32_t BudgetMilliseconds;
    FClock::time_point Expires;
    std::vector<FSlot> Slots;
    std::optional<FPublication> Neutral;
    bool bFailed = false, bReleased = false;
};

class FClientRoundControl final
{
public:
    FClientRoundControl(const FClientRoundControl&) = delete;
    FClientRoundControl& operator=(const FClientRoundControl&) = delete;
    FClientRoundControl(FClientRoundControl&&) noexcept = default;
    FClientRoundControl& operator=(FClientRoundControl&&) noexcept = default;

    // Keep the reservation high-water mark with the authenticated match so an
    // old offer cannot reopen a discarded gate. A duplicate cannot extend time.
    static std::optional<FClientRoundControl> OpenOffer(U64 AuthenticatedPeer, const FBinding& Binding,
        U64 Match, U64 MatchEpoch, U64 CurrentGeneration, U64 LastReservation, const FMessage& Offer,
        FClock::time_point Now)
    {
        const auto* Reserve = std::get_if<FReserve>(&Offer.Payload);
        const auto& K = Offer.Key;
        if (!Reserve || !Valid(Offer) || !Binding.Valid() || Binding.ServerPeer != AuthenticatedPeer
            || !(FSessionAddress::From(Binding) == Offer.Session) || K.Match != Match || K.MatchEpoch != MatchEpoch
            || K.PreviousGeneration != CurrentGeneration || K.Reservation <= LastReservation) return {};
        const auto Budget = std::chrono::milliseconds(Reserve->BudgetMilliseconds);
        if (Now >= FClock::time_point::max() - Budget) return {};
        return FClientRoundControl(K, Binding, Reserve->BudgetMilliseconds, Now + Budget);
    }

    const FKey& Key() const noexcept { return Round; }
    FClock::time_point Deadline() const noexcept { return Expires; }
    void CancelBeforeRelease() noexcept { if (!bReleased) bFailed = true; }

    // The game calls this with the result of its real capture reservation at F.
    std::optional<FMessage> ReservationResult(bool Accepted, FClock::time_point Now) noexcept
    {
        if (!Open(Now)) return {};
        if (Reserved && *Reserved != Accepted) { Conflict(); return {}; }
        Reserved = Accepted;
        return Make(FReservationResult{Accepted});
    }

    std::optional<FMessage> PublishBaseline(const FPacket& Packet, FClock::time_point Now)
    {
        if (!Open(Now) || !Reserved.value_or(false)) return {};
        auto M = Make(FBaseline{Packet});
        if (!Valid(M) || !Detail::MatchesBaseline(Binding, Packet)) return {};
        if (Baseline && !Netcode::Same(*Baseline, Packet)) { Conflict(); return {}; }
        if (!Baseline) Baseline = Packet;
        return M;
    }

    EResult ReceiveFromServer(U64 AuthenticatedPeer, const FMessage& M, FClock::time_point Now) noexcept
    {
        if (!Valid(M)) return EResult::Invalid;
        const auto Tag = Kind(M);
        if (Tag != EKind::Reserve && Tag != EKind::Published && Tag != EKind::Released) return EResult::Invalid;
        if (AuthenticatedPeer != Binding.ServerPeer) return EResult::WrongPeer;
        if (!(M.Session == FSessionAddress::From(Binding))) return EResult::WrongSession;
        if (Now >= Expires) return EResult::Expired;
        const auto Match = Detail::MatchKey(Round, M.Key);
        if (Match != EResult::Accepted) return Match == EResult::Conflict ? Conflict() : Match;
        if (bFailed) return EResult::Closed;
        if (const auto* Offer = std::get_if<FReserve>(&M.Payload))
            return Offer->BudgetMilliseconds == BudgetMilliseconds ? EResult::Duplicate : Conflict();
        if (!Baseline) return EResult::OutOfOrder;
        const auto& Stamp = *Publication(M);
        if (Neutral && !(*Neutral == Stamp)) return Conflict();
        if (Tag == EKind::Published)
        {
            if (Neutral) return EResult::Duplicate;
            Neutral = Stamp;
            return EResult::Accepted;
        }
        if (!bApplied) return EResult::OutOfOrder;
        if (bReleased) return EResult::Duplicate;
        bReleased = true;
        return EResult::Accepted;
    }

    // Call only from the actual application/presentation acknowledgement seam.
    std::optional<FMessage> AcknowledgeInstalled(const FPublication& Step, FClock::time_point Now) noexcept
    {
        if (!Open(Now) || !Neutral) return {};
        if (!(*Neutral == Step)) { Conflict(); return {}; }
        bApplied = true;
        return Make(FApplied{Step});
    }

    bool GameplayReleased(FClock::time_point Now) const noexcept { return Open(Now) && bReleased; }

private:
    FClientRoundControl(const FKey& Key, const FBinding& InBinding, std::uint32_t Budget, FClock::time_point Deadline)
        : Round(Key), Binding(InBinding), BudgetMilliseconds(Budget), Expires(Deadline) {}
    FMessage Make(FPayload Payload) const { return {Round, FSessionAddress::From(Binding), std::move(Payload)}; }
    bool Open(FClock::time_point Now) const noexcept { return Now < Expires && !bFailed; }
    EResult Conflict() noexcept { if (!bReleased) bFailed = true; return EResult::Conflict; }

    FKey Round;
    FBinding Binding;
    std::uint32_t BudgetMilliseconds;
    FClock::time_point Expires;
    std::optional<bool> Reserved;
    std::optional<FPacket> Baseline;
    std::optional<FPublication> Neutral;
    bool bFailed = false, bApplied = false, bReleased = false;
};
}
