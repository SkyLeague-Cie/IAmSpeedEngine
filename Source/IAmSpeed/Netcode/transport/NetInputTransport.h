#pragma once

// Private prototype. No UE types, device polling, physics or clock ownership.
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace Speed::Netcode
{
using U64 = std::uint64_t;
constexpr std::size_t ActionCount = 32, MaxEdges = 64, Window = 64;
constexpr std::size_t HeaderBytes = 92, BaseBytes = 190, EdgeBytes = 16;
constexpr std::size_t MaxDatagram = BaseBytes + MaxEdges * EdgeBytes;
enum class EKind : std::uint8_t { OwnerInput = 1, AuthorityInput = 2, Ack = 3 };
enum class EResult { Accepted, Duplicate, Conflict, Stale, TooFar, WrongPeer,
    WrongStream, Invalid, Missing, Exhausted, Full };
struct FEdge
{
    std::uint8_t Action = 0, State = 0; // 0 Started, 1 Completed
    std::int16_t Value = 0;
    U64 SourceSequence = 0;
    std::uint32_t Ordinal = 0; // V2 FSourceOrder::WithinSequence, no narrowing
};
struct FFrame
{
    U64 SourceSequence = 0, DeviceGeneration = 0, ProducerId = 0;
    std::uint8_t ProducerKind = 0; // V2 source identity, unchanged in transit
    std::uint32_t ActiveMask = 0;
    std::array<std::int16_t, ActionCount> Values{};
    std::vector<FEdge> Edges;
    bool Reset = false;
};
struct FPacket
{
    EKind Kind = EKind::OwnerInput;
    U64 Session = 0, Epoch = 0, Actor = 0, Incarnation = 0;
    std::array<std::uint8_t, 32> Contract{}; // host-negotiated schema ID; control plane is separate
    U64 Sequence = 0, Frame = 0;
    FFrame Input;
};
struct FBinding
{
    U64 Session = 0, Epoch = 0, Actor = 0, Incarnation = 0;
    U64 OwnerPeer = 0, ServerPeer = 0, SourceProducer = 0;
    std::uint8_t SourceKind = 0;
    std::array<std::uint8_t, 32> Contract{};
    bool Valid() const
    {
        bool Nonzero = false;
        for (auto B : Contract) Nonzero |= B != 0;
        return Session && Epoch && Actor && Incarnation && OwnerPeer && ServerPeer
            && OwnerPeer != ServerPeer && SourceProducer && SourceKind <= 2 && Nonzero;
    }
    bool Matches(const FPacket& P) const
    {
        return Session == P.Session && Epoch == P.Epoch && Actor == P.Actor
            && Incarnation == P.Incarnation && Contract == P.Contract;
    }
};

inline bool Valid(const FPacket& P)
{
    if (P.Kind < EKind::OwnerInput || P.Kind > EKind::Ack || !P.Session || !P.Epoch
        || !P.Actor || !P.Incarnation || !P.Sequence) return false;
    bool ContractPresent = false;
    for (auto B : P.Contract) ContractPresent |= B != 0;
    if (!ContractPresent) return false;
    if (P.Kind == EKind::Ack) return true;
    const auto& I = P.Input;
    if (!I.SourceSequence || !I.DeviceGeneration || !I.ProducerId || I.ProducerKind > 2
        || I.Edges.size() > MaxEdges || (I.Reset && !I.Edges.empty())
        || I.Values[0] < 0 || I.Values[0] > 255 || I.Values[1] < 0 || I.Values[1] > 255
        || I.Values[2] < -127 || I.Values[2] > 127) return false;
    std::array<int, ActionCount> Last{};
    for (std::size_t N = 0; N < I.Edges.size(); ++N)
    {
        const auto& E = I.Edges[N];
        if (E.Action >= ActionCount || E.State > 1 || !E.SourceSequence
            || E.SourceSequence > I.SourceSequence) return false;
        if (N)
        {
            const auto& Before = I.Edges[N - 1];
            if (E.SourceSequence < Before.SourceSequence || (E.SourceSequence == Before.SourceSequence
                && E.Ordinal <= Before.Ordinal)) return false;
        }
        const int State = E.State == 0 ? 1 : -1;
        if (Last[E.Action] == State) return false;
        Last[E.Action] = State;
    }
    for (std::size_t N = 0; N < ActionCount; ++N)
        if (Last[N] && ((Last[N] == 1) != bool(I.ActiveMask & (std::uint32_t{1} << N)))) return false;
    return true;
}

namespace Wire
{
inline void Put(std::vector<std::uint8_t>& B, U64 V, unsigned Count)
{ for (unsigned N = 0; N < Count; ++N) B.push_back(std::uint8_t(V >> (N * 8))); }
struct FReader
{
    const std::uint8_t* Data; std::size_t Size, At = 0; bool Good = true;
    U64 Get(unsigned Count)
    {
        if (At > Size || Count > Size - At) { Good = false; return 0; }
        U64 V = 0;
        for (unsigned N = 0; N < Count; ++N) V |= U64(Data[At++]) << (N * 8);
        return V;
    }
};
inline std::int16_t Signed(U64 V)
{ return static_cast<std::int16_t>(V < 32768 ? std::int32_t(V) : std::int32_t(V) - 65536); }
}
inline std::optional<std::vector<std::uint8_t>> Encode(const FPacket& P)
{
    if (!Valid(P)) return {};
    std::vector<std::uint8_t> B;
    const auto Size = P.Kind == EKind::Ack ? HeaderBytes : BaseBytes + P.Input.Edges.size() * EdgeBytes;
    B.reserve(Size);
    Wire::Put(B, 0x314E5349, 4); // ISN1, little-endian
    Wire::Put(B, 1, 2); Wire::Put(B, std::uint8_t(P.Kind), 1); Wire::Put(B, 0, 1);
    Wire::Put(B, Size, 2); Wire::Put(B, 0, 2);
    for (auto V : {P.Session, P.Epoch, P.Actor, P.Incarnation, P.Sequence, P.Frame}) Wire::Put(B, V, 8);
    B.insert(B.end(), P.Contract.begin(), P.Contract.end());
    if (P.Kind != EKind::Ack)
    {
        const auto& I = P.Input;
        Wire::Put(B, I.SourceSequence, 8); Wire::Put(B, I.DeviceGeneration, 8);
        Wire::Put(B, I.ProducerId, 8); Wire::Put(B, I.ProducerKind, 1);
        Wire::Put(B, I.Reset ? 1 : 0, 1); Wire::Put(B, I.ActiveMask, 4);
        Wire::Put(B, I.Edges.size(), 2); Wire::Put(B, 0, 2);
        for (auto V : I.Values) Wire::Put(B, static_cast<std::uint16_t>(V), 2);
        for (const auto& E : I.Edges)
        {
            Wire::Put(B, E.Action, 1); Wire::Put(B, E.State, 1);
            Wire::Put(B, static_cast<std::uint16_t>(E.Value), 2);
            Wire::Put(B, E.SourceSequence, 8); Wire::Put(B, E.Ordinal, 4);
        }
    }
    if (B.size() != Size) return {};
    return B;
}
inline std::optional<FPacket> Decode(const std::uint8_t* Data, std::size_t Size)
{
    if (!Data || Size < HeaderBytes || Size > MaxDatagram) return {};
    Wire::FReader R{Data, Size};
    if (R.Get(4) != 0x314E5349 || R.Get(2) != 1) return {};
    FPacket P; P.Kind = static_cast<EKind>(R.Get(1));
    if (R.Get(1) || R.Get(2) != Size || R.Get(2)) return {};
    P.Session = R.Get(8); P.Epoch = R.Get(8); P.Actor = R.Get(8);
    P.Incarnation = R.Get(8); P.Sequence = R.Get(8); P.Frame = R.Get(8);
    for (auto& B : P.Contract) B = std::uint8_t(R.Get(1));
    if (P.Kind != EKind::Ack)
    {
        if (Size < BaseBytes) return {};
        auto& I = P.Input;
        I.SourceSequence = R.Get(8); I.DeviceGeneration = R.Get(8); I.ProducerId = R.Get(8);
        I.ProducerKind = std::uint8_t(R.Get(1));
        const auto Reset = R.Get(1); if (Reset > 1) return {}; I.Reset = Reset != 0;
        I.ActiveMask = std::uint32_t(R.Get(4));
        const auto Count = R.Get(2);
        if (R.Get(2) || Count > MaxEdges || Size != BaseBytes + Count * EdgeBytes) return {};
        for (auto& V : I.Values) V = Wire::Signed(R.Get(2));
        I.Edges.reserve(static_cast<std::size_t>(Count));
        for (U64 N = 0; N < Count; ++N)
        {
            FEdge E; E.Action = std::uint8_t(R.Get(1)); E.State = std::uint8_t(R.Get(1));
            E.Value = Wire::Signed(R.Get(2)); E.SourceSequence = R.Get(8);
            E.Ordinal = std::uint32_t(R.Get(4)); I.Edges.push_back(E);
        }
    }
    if (!R.Good || R.At != Size || !Valid(P)) return {};
    return P;
}
inline bool Same(const FPacket& A, const FPacket& B)
{ return Encode(A) == Encode(B); }
inline FPacket AckFor(FPacket P) { P.Kind = EKind::Ack; P.Input = {}; return P; }

// One endpoint binding per owned actor; install/rekey at a quiescent boundary.
// Peer identity MUST come from the host's authenticated channel, not wire bytes.
// Validator pins game-specific action ranges/contract, without polling a device.
// Consume advances THIS transport cursor only. The UE adapter must coordinate
// prepare/install/world publication/history commit as one owner transaction.
using FValidator = bool (*)(const FFrame&);
class FExactInbox
{
public:
    FExactInbox(FBinding InBinding, EKind InKind, U64 First, FValidator InValidator)
        : Binding(InBinding), Kind(InKind), Next(First), Validator(InValidator) {}
    EResult Receive(U64 Peer, const FPacket& P)
    {
        if (Quarantined || !Binding.Valid() || !Validator || !Valid(P)) return EResult::Invalid;
        if (Peer != (Kind == EKind::OwnerInput ? Binding.OwnerPeer : Binding.ServerPeer)) return EResult::WrongPeer;
        if (P.Kind != Kind || !Binding.Matches(P) || P.Input.ProducerId != Binding.SourceProducer
            || P.Input.ProducerKind != Binding.SourceKind) return EResult::WrongStream;
        if (!Validator(P.Input)) return EResult::Invalid;
        // Never let a second packet replace a pending or completed frame.
        for (const auto* Table : {&Pending, &History}) for (const auto& S : *Table)
            if (S && (S->Frame == P.Frame || S->Sequence == P.Sequence))
                return Same(*S, P) ? EResult::Duplicate : EResult::Conflict;
        if (Exhausted) return EResult::Exhausted;
        if (P.Frame < Next) return EResult::Stale;
        if (Last && P.Sequence <= Last->Sequence) return EResult::Stale;
        if (P.Frame - Next >= Window) return EResult::TooFar;
        Pending[P.Frame % Window] = P;
        return EResult::Accepted;
    }
    struct FConsumption { FPacket Packet; bool FirstConsume; };
    std::optional<FConsumption> Consume(U64 Frame)
    {
        const auto& Old = History[Frame % Window];
        if (Old && Old->Frame == Frame) return FConsumption{*Old, false};
        if (Quarantined || Exhausted || Frame != Next) return {};
        auto& Slot = Pending[Frame % Window];
        if (!Slot || Slot->Frame != Frame) return {}; // absence is not neutral/held
        const auto& I = Slot->Input;
        const auto Reject = [&]() -> std::optional<FConsumption> { Quarantined = true; return {}; };
        if (!Last)
        { if (!I.Reset) return Reject(); }
        else
        {
            const auto& Prev = Last->Input;
            if (Slot->Sequence <= Last->Sequence || I.SourceSequence < Prev.SourceSequence) return Reject();
            if (I.SourceSequence == Prev.SourceSequence && (I.Reset || !I.Edges.empty()
                || I.DeviceGeneration != Prev.DeviceGeneration || I.Values != Prev.Values
                || I.ActiveMask != Prev.ActiveMask)) return Reject();
            if (I.DeviceGeneration != Prev.DeviceGeneration && (!I.Reset
                || I.DeviceGeneration < Prev.DeviceGeneration || I.SourceSequence <= Prev.SourceSequence)) return Reject();
            if (I.Reset && I.SourceSequence <= Prev.SourceSequence) return Reject();
            // A transition may not replay an event from an earlier input frame.
            auto Active = Prev.ActiveMask;
            for (const auto& E : I.Edges)
            {
                const auto Bit = std::uint32_t{1} << E.Action;
                const bool Started = E.State == 0;
                if (E.SourceSequence <= Prev.SourceSequence || (bool(Active & Bit) == Started)) return Reject();
                if (Started) Active |= Bit; else Active &= ~Bit;
            }
            if (!I.Reset && Active != I.ActiveMask) return Reject();
        }
        auto Result = *Slot;
        History[Frame % Window] = Slot; Slot.reset(); Last = Result;
        Exhausted = Frame == std::numeric_limits<U64>::max();
        if (!Exhausted) ++Next;
        return FConsumption{std::move(Result), true};
    }
    U64 NextFrame() const { return Next; }
    bool RequiresResync() const { return Quarantined; }
private:
    FBinding Binding; EKind Kind; U64 Next; FValidator Validator;
    bool Exhausted = false, Quarantined = false;
    std::optional<FPacket> Last;
    std::array<std::optional<FPacket>, Window> Pending{}, History{};
};

// Bounded selective retransmission. Host schedules retries; no private clock.
// ACK = downstream inbox retention, NOT physical publication or gameplay accept.
class FReliableOutbox
{
public:
    FReliableOutbox(FBinding InBinding, U64 InDestination) : Binding(InBinding), Destination(InDestination) {}
    EResult Queue(const FPacket& P)
    {
        if (!Binding.Valid() || !Destination || !Binding.Matches(P) || !Valid(P) || P.Kind == EKind::Ack)
            return EResult::Invalid;
        for (const auto& S : Slots) if (S && (S->Sequence == P.Sequence || S->Frame == P.Frame))
            return Same(*S, P) ? EResult::Duplicate : EResult::Conflict;
        if (LastQueued && (P.Sequence <= LastQueued->Sequence || P.Frame <= LastQueued->Frame)) return EResult::Stale;
        for (auto& S : Slots) if (!S) { S = P; LastQueued = P; return EResult::Accepted; }
        return EResult::Full;
    }
    bool Acknowledge(U64 Peer, const FPacket& Ack)
    {
        if (Peer != Destination || Ack.Kind != EKind::Ack || !Valid(Ack) || !Binding.Matches(Ack)) return false;
        for (auto& S : Slots) if (S && S->Sequence == Ack.Sequence && S->Frame == Ack.Frame)
        { S.reset(); return true; }
        return false;
    }
    template<class FSend> void Retry(FSend&& Send) const
    { for (const auto& S : Slots) if (S) Send(*S); }
    std::size_t Size() const { std::size_t N = 0; for (const auto& S : Slots) N += bool(S); return N; }
private:
    FBinding Binding; U64 Destination;
    std::optional<FPacket> LastQueued;
    std::array<std::optional<FPacket>, Window> Slots{};
};
}
