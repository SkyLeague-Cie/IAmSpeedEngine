#pragma once

#include "IAmSpeed/Netcode/transport/NetInputTransport.h"
#include <variant>

namespace Speed::Netcode::RoundControl
{
using FBytes = std::vector<std::uint8_t>;

struct FKey
{
    U64 Match = 0, MatchEpoch = 0, Reservation = 0, PreviousGeneration = 0, Generation = 0, Frame = 0;

    bool Valid() const noexcept
    {
        return Match && MatchEpoch && Reservation && PreviousGeneration && PreviousGeneration < UINT64_MAX
            && Generation == PreviousGeneration + 1 && Frame && Frame < UINT32_MAX;
    }

    bool operator==(const FKey& B) const noexcept
    {
        return Match == B.Match && MatchEpoch == B.MatchEpoch && Reservation == B.Reservation
            && PreviousGeneration == B.PreviousGeneration && Generation == B.Generation && Frame == B.Frame;
    }
};

struct FSessionAddress
{
    U64 Session = 0, Epoch = 0, Actor = 0, Incarnation = 0;
    std::array<std::uint8_t, 32> Contract{};

    static FSessionAddress From(const FBinding& B) noexcept
    { return {B.Session, B.Epoch, B.Actor, B.Incarnation, B.Contract}; }

    bool Valid() const noexcept
    {
        for (auto Byte : Contract)
            if (Byte) return Session && Epoch && Actor && Incarnation;
        return false;
    }

    bool operator==(const FSessionAddress& B) const noexcept
    {
        return Session == B.Session && Epoch == B.Epoch && Actor == B.Actor
            && Incarnation == B.Incarnation && Contract == B.Contract;
    }
};

// From the actual dedicated neutral step F and its round commit. These are
// authority identities. Each client verifies installation of the corresponding data.
struct FPublication
{
    U64 Serial = 0, StateHash = 0, InputHash = 0, NeutralBaselineHash = 0;

    bool Valid() const noexcept { return Serial && StateHash && InputHash && NeutralBaselineHash; }
    bool operator==(const FPublication& B) const noexcept
    {
        return Serial == B.Serial && StateHash == B.StateHash && InputHash == B.InputHash
            && NeutralBaselineHash == B.NeutralBaselineHash;
    }
};

struct FReserve { std::uint32_t BudgetMilliseconds = 0; };
struct FReservationResult { bool Accepted = false; };
struct FBaseline { FPacket Packet; };
struct FPublished { FPublication Step; };
struct FApplied { FPublication Step; };
struct FReleased { FPublication Step; };

// Variant order is the version-1 wire tag order.
using FPayload = std::variant<FReserve, FReservationResult, FBaseline, FPublished, FApplied, FReleased>;
struct FMessage { FKey Key; FSessionAddress Session; FPayload Payload; };
enum class EKind : std::uint8_t { Reserve = 1, ReservationResult, Baseline, Published, Applied, Released };
inline constexpr std::uint32_t MaxBudgetMilliseconds = 600000;
inline constexpr std::size_t HeaderBytes = 12 + 6 * 8 + 4 * 8 + 32;
inline constexpr std::size_t MaxMessageBytes = HeaderBytes + Netcode::BaseBytes;

inline EKind Kind(const FMessage& M) noexcept { return static_cast<EKind>(M.Payload.index() + 1); }

inline const FPublication* Publication(const FMessage& M) noexcept
{
    if (const auto* Result = std::get_if<FPublished>(&M.Payload)) return &Result->Step;
    if (const auto* Result = std::get_if<FApplied>(&M.Payload)) return &Result->Step;
    if (const auto* Result = std::get_if<FReleased>(&M.Payload)) return &Result->Step;
    return nullptr;
}

inline bool Valid(const FMessage& M) noexcept
{
    if (!M.Key.Valid() || !M.Session.Valid()) return false;
    if (const auto* Reserve = std::get_if<FReserve>(&M.Payload))
        return Reserve->BudgetMilliseconds && Reserve->BudgetMilliseconds <= MaxBudgetMilliseconds;
    if (std::holds_alternative<FReservationResult>(M.Payload)) return true;
    if (const auto* Baseline = std::get_if<FBaseline>(&M.Payload))
    {
        const auto& P = Baseline->Packet;
        return P.Kind == Netcode::EKind::OwnerInput && P.Frame == M.Key.Frame
            && P.Input.Reset && P.Input.Edges.empty() && Netcode::Valid(P)
            && M.Session == FSessionAddress{P.Session, P.Epoch, P.Actor, P.Incarnation, P.Contract};
    }
    const auto* Step = Publication(M);
    return Step && Step->Valid();
}

inline std::optional<FBytes> Encode(const FMessage& M)
{
    if (!Valid(M)) return {};
    FBytes Body;
    if (const auto* Reserve = std::get_if<FReserve>(&M.Payload))
        Wire::Put(Body, Reserve->BudgetMilliseconds, 4);
    else if (const auto* Result = std::get_if<FReservationResult>(&M.Payload))
        Wire::Put(Body, Result->Accepted, 1);
    else if (const auto* Baseline = std::get_if<FBaseline>(&M.Payload))
    {
        auto Packet = Netcode::Encode(Baseline->Packet);
        if (!Packet || Packet->size() != Netcode::BaseBytes) return {};
        Body = std::move(*Packet);
    }
    else
    {
        const auto& P = *Publication(M);
        for (auto V : {P.Serial, P.StateHash, P.InputHash, P.NeutralBaselineHash}) Wire::Put(Body, V, 8);
    }

    FBytes Bytes;
    Bytes.reserve(HeaderBytes + Body.size());
    Wire::Put(Bytes, 0x31524149, 4); // IAR1
    Wire::Put(Bytes, 1, 2);
    Wire::Put(Bytes, static_cast<std::uint8_t>(Kind(M)), 1);
    Wire::Put(Bytes, 0, 1);
    Wire::Put(Bytes, HeaderBytes + Body.size(), 2);
    Wire::Put(Bytes, 0, 2);
    const auto& K = M.Key;
    for (auto V : {K.Match, K.MatchEpoch, K.Reservation, K.PreviousGeneration, K.Generation, K.Frame,
             M.Session.Session, M.Session.Epoch, M.Session.Actor, M.Session.Incarnation}) Wire::Put(Bytes, V, 8);
    Bytes.insert(Bytes.end(), M.Session.Contract.begin(), M.Session.Contract.end());
    Bytes.insert(Bytes.end(), Body.begin(), Body.end());
    return Bytes;
}

inline std::optional<FMessage> Decode(const std::uint8_t* Data, std::size_t Size)
{
    if (!Data || Size < HeaderBytes || Size > MaxMessageBytes) return {};
    Wire::FReader R{Data, Size};
    if (R.Get(4) != 0x31524149 || R.Get(2) != 1) return {};
    const auto Tag = static_cast<EKind>(R.Get(1));
    if (R.Get(1) || R.Get(2) != Size || R.Get(2)) return {};

    FMessage M;
    auto& K = M.Key;
    for (auto* V : {&K.Match, &K.MatchEpoch, &K.Reservation, &K.PreviousGeneration, &K.Generation, &K.Frame,
             &M.Session.Session, &M.Session.Epoch, &M.Session.Actor, &M.Session.Incarnation}) *V = R.Get(8);
    for (auto& V : M.Session.Contract) V = static_cast<std::uint8_t>(R.Get(1));
    if (!K.Valid() || !M.Session.Valid()) return {};

    switch (Tag)
    {
    case EKind::Reserve:
        M.Payload = FReserve{static_cast<std::uint32_t>(R.Get(4))};
        break;
    case EKind::ReservationResult:
    {
        const auto Accepted = R.Get(1);
        if (Accepted > 1) return {};
        M.Payload = FReservationResult{Accepted != 0};
        break;
    }
    case EKind::Baseline:
    {
        if (Size != HeaderBytes + Netcode::BaseBytes) return {};
        auto P = Netcode::Decode(Data + R.At, Netcode::BaseBytes);
        if (!P) return {};
        M.Payload = FBaseline{std::move(*P)};
        R.At = Size;
        break;
    }
    case EKind::Published:
    case EKind::Applied:
    case EKind::Released:
    {
        const FPublication P{R.Get(8), R.Get(8), R.Get(8), R.Get(8)};
        if (Tag == EKind::Published) M.Payload = FPublished{P};
        else if (Tag == EKind::Applied) M.Payload = FApplied{P};
        else M.Payload = FReleased{P};
        break;
    }
    default:
        return {};
    }
    if (!R.Good || R.At != Size || !Valid(M)) return {};
    return M;
}
}
