#pragma once
#include "NetInputTransport.h"
#include <algorithm>

namespace Speed::Netcode
{
// Physical/mechanical/gameplay state bytes are supplied by each canonical
// serializer. Never serialize a UObject, native struct memory, or pose-only
// presentation output and call it a restorable physical state.
constexpr std::size_t MaxStateActors = 128, MaxActorState = 16384;
constexpr std::size_t MaxWorldState = 262144, StateChunkBytes = 1000, StateHeaderBytes = 88;
enum class ELifecycle : std::uint8_t { Alive = 0, Destroyed = 1 };
struct FActorState
{
    U64 Actor = 0, Incarnation = 0;
    std::uint32_t Type = 0, SchemaVersion = 0;
    ELifecycle Lifecycle = ELifecycle::Alive;
    std::vector<std::uint8_t> Payload;
};
struct FWorldState
{
    U64 Sequence = 0, Frame = 0, RegistryGeneration = 0;
    std::vector<std::uint8_t> WorldPayload; // solver/world checkpoints + version, game-owned
    std::vector<FActorState> Actors; // full live roster plus explicit destruction records
};
struct FStateBinding
{
    U64 Session = 0, Epoch = 0, ServerPeer = 0;
    std::array<std::uint8_t, 32> Schema{};
    bool Valid() const
    { bool Nonzero = false; for (auto B : Schema) Nonzero |= B != 0; return Session && Epoch && ServerPeer && Nonzero; }
};
inline bool ValidState(const FWorldState& S)
{
    if (!S.Sequence || !S.RegistryGeneration || S.WorldPayload.empty() || S.WorldPayload.size() > 65536
        || S.Actors.size() > MaxStateActors) return false;
    std::size_t Total = 40 + S.WorldPayload.size();
    for (std::size_t N = 0; N < S.Actors.size(); ++N)
    {
        const auto& A = S.Actors[N];
        if (!A.Actor || !A.Incarnation || !A.Type || !A.SchemaVersion || A.Lifecycle > ELifecycle::Destroyed
            || A.Payload.size() > MaxActorState || (A.Lifecycle == ELifecycle::Alive && A.Payload.empty())
            || (A.Lifecycle == ELifecycle::Destroyed && !A.Payload.empty())) return false;
        if (N)
        {
            const auto& P = S.Actors[N - 1];
            if (A.Actor < P.Actor || (A.Actor == P.Actor && A.Incarnation <= P.Incarnation)) return false;
            if (A.Actor == P.Actor && P.Lifecycle == ELifecycle::Alive) return false;
        }
        Total += 28 + A.Payload.size();
        if (Total > MaxWorldState) return false;
    }
    return true;
}
inline std::optional<std::vector<std::uint8_t>> EncodeState(const FWorldState& S)
{
    if (!ValidState(S)) return {};
    std::vector<std::uint8_t> B;
    Wire::Put(B, 0x31575349, 4); Wire::Put(B, 1, 2); Wire::Put(B, 1, 2); // full roster, no delta
    Wire::Put(B, S.Actors.size(), 2); Wire::Put(B, 0, 2);
    Wire::Put(B, S.Sequence, 8); Wire::Put(B, S.Frame, 8); Wire::Put(B, S.RegistryGeneration, 8);
    Wire::Put(B, S.WorldPayload.size(), 4); B.insert(B.end(), S.WorldPayload.begin(), S.WorldPayload.end());
    for (const auto& A : S.Actors)
    {
        Wire::Put(B, A.Actor, 8); Wire::Put(B, A.Incarnation, 8); Wire::Put(B, A.Type, 4);
        Wire::Put(B, A.SchemaVersion, 4); Wire::Put(B, std::uint8_t(A.Lifecycle), 1); Wire::Put(B, 0, 1);
        Wire::Put(B, A.Payload.size(), 2); B.insert(B.end(), A.Payload.begin(), A.Payload.end());
    }
    return B;
}
inline std::optional<FWorldState> DecodeState(const std::uint8_t* Bytes, std::size_t Size)
{
    if (!Bytes || Size < 40 || Size > MaxWorldState) return {};
    Wire::FReader R{Bytes, Size};
    if (R.Get(4) != 0x31575349 || R.Get(2) != 1 || R.Get(2) != 1) return {};
    const auto Count = R.Get(2); if (R.Get(2) || Count > MaxStateActors) return {};
    FWorldState S; S.Sequence = R.Get(8); S.Frame = R.Get(8); S.RegistryGeneration = R.Get(8);
    const auto WorldSize = R.Get(4);
    if (!WorldSize || WorldSize > 65536 || !R.Good || WorldSize > Size - R.At) return {};
    S.WorldPayload.assign(Bytes + R.At, Bytes + R.At + WorldSize); R.At += std::size_t(WorldSize);
    for (U64 N = 0; N < Count; ++N)
    {
        FActorState A; A.Actor = R.Get(8); A.Incarnation = R.Get(8); A.Type = std::uint32_t(R.Get(4));
        A.SchemaVersion = std::uint32_t(R.Get(4)); A.Lifecycle = static_cast<ELifecycle>(R.Get(1));
        if (R.Get(1)) return {};
        const auto PayloadSize = R.Get(2);
        if (!R.Good || PayloadSize > MaxActorState || PayloadSize > Size - R.At) return {};
        A.Payload.assign(Bytes + R.At, Bytes + R.At + PayloadSize); R.At += std::size_t(PayloadSize);
        S.Actors.push_back(std::move(A));
    }
    if (!R.Good || R.At != Size || !ValidState(S)) return {};
    return S;
}
inline std::uint32_t StateCrc(const std::vector<std::uint8_t>& Bytes)
{
    std::uint32_t Crc = 0xFFFFFFFFu;
    for (auto B : Bytes)
    {
        Crc ^= B;
        for (int Bit = 0; Bit < 8; ++Bit) Crc = (Crc >> 1) ^ (0xEDB88320u & (0u - (Crc & 1u)));
    }
    return ~Crc; // corruption check; no authentication claim
}
struct FStateFragment
{
    U64 Session = 0, Epoch = 0, Sequence = 0, Frame = 0;
    std::uint32_t TotalBytes = 0, Crc = 0;
    std::uint16_t Index = 0, Count = 0;
    std::array<std::uint8_t, 32> Schema{};
    bool Ack = false;
    std::vector<std::uint8_t> Bytes;
};
inline bool ValidFragment(const FStateFragment& F)
{
    bool Nonzero = false; for (auto B : F.Schema) Nonzero |= B != 0;
    if (!F.Session || !F.Epoch || !F.Sequence || !Nonzero || F.TotalBytes < 40 || F.TotalBytes > MaxWorldState
        || !F.Count || F.Count != (F.TotalBytes + StateChunkBytes - 1) / StateChunkBytes) return false;
    if (F.Ack) return F.Index == 0 && F.Bytes.empty();
    if (F.Index >= F.Count) return false;
    const auto Expected = std::min<std::size_t>(StateChunkBytes, F.TotalBytes - std::size_t(F.Index) * StateChunkBytes);
    return F.Bytes.size() == Expected;
}
inline bool SameTransfer(const FStateFragment& A, const FStateFragment& B)
{
    return A.Session == B.Session && A.Epoch == B.Epoch && A.Sequence == B.Sequence && A.Frame == B.Frame
        && A.TotalBytes == B.TotalBytes && A.Crc == B.Crc && A.Count == B.Count && A.Schema == B.Schema;
}
inline std::optional<std::vector<std::uint8_t>> EncodeFragment(const FStateFragment& F)
{
    if (!ValidFragment(F)) return {};
    std::vector<std::uint8_t> B;
    Wire::Put(B, 0x31545349, 4); Wire::Put(B, 1, 2); Wire::Put(B, F.Ack ? 1 : 0, 1); Wire::Put(B, 0, 1);
    Wire::Put(B, StateHeaderBytes + F.Bytes.size(), 2); Wire::Put(B, 0, 2);
    for (auto V : {F.Session, F.Epoch, F.Sequence, F.Frame}) Wire::Put(B, V, 8);
    Wire::Put(B, F.TotalBytes, 4); Wire::Put(B, F.Crc, 4); Wire::Put(B, F.Index, 2); Wire::Put(B, F.Count, 2);
    B.insert(B.end(), F.Schema.begin(), F.Schema.end()); B.insert(B.end(), F.Bytes.begin(), F.Bytes.end());
    return B;
}
inline std::optional<FStateFragment> DecodeFragment(const std::uint8_t* Bytes, std::size_t Size)
{
    if (!Bytes || Size < StateHeaderBytes || Size > StateHeaderBytes + StateChunkBytes) return {};
    Wire::FReader R{Bytes, Size};
    if (R.Get(4) != 0x31545349 || R.Get(2) != 1) return {};
    const auto Ack = R.Get(1); if (Ack > 1 || R.Get(1) || R.Get(2) != Size || R.Get(2)) return {};
    FStateFragment F; F.Ack = Ack != 0; F.Session = R.Get(8); F.Epoch = R.Get(8);
    F.Sequence = R.Get(8); F.Frame = R.Get(8); F.TotalBytes = std::uint32_t(R.Get(4)); F.Crc = std::uint32_t(R.Get(4));
    F.Index = std::uint16_t(R.Get(2)); F.Count = std::uint16_t(R.Get(2));
    for (auto& B : F.Schema) B = std::uint8_t(R.Get(1));
    if (!R.Good) return {};
    F.Bytes.assign(Bytes + R.At, Bytes + Size);
    if (!ValidFragment(F)) return {};
    return F;
}
inline std::optional<std::vector<FStateFragment>> FragmentState(const FWorldState& S, const FStateBinding& B)
{
    if (!B.Valid()) return {};
    auto Bytes = EncodeState(S); if (!Bytes) return {};
    std::vector<FStateFragment> Result;
    const auto Count = (Bytes->size() + StateChunkBytes - 1) / StateChunkBytes;
    const auto Crc = StateCrc(*Bytes);
    for (std::size_t N = 0; N < Count; ++N)
    {
        FStateFragment F; F.Session = B.Session; F.Epoch = B.Epoch; F.Sequence = S.Sequence; F.Frame = S.Frame;
        F.TotalBytes = std::uint32_t(Bytes->size()); F.Crc = Crc; F.Index = std::uint16_t(N); F.Count = std::uint16_t(Count); F.Schema = B.Schema;
        const auto First = N * StateChunkBytes, Last = std::min(First + StateChunkBytes, Bytes->size());
        F.Bytes.assign(Bytes->begin() + First, Bytes->begin() + Last); Result.push_back(std::move(F));
    }
    return Result;
}
inline FStateFragment StateAck(FStateFragment F) { F.Ack = true; F.Index = 0; F.Bytes.clear(); return F; }
class FStateReceiver
{
public:
    FStateReceiver(FStateBinding B, U64 FirstSequence = 1) : Binding(B), Next(FirstSequence) {}
    EResult Receive(U64 Peer, const FStateFragment& F)
    {
        if (!Binding.Valid() || !ValidFragment(F) || F.Ack || Failed) return EResult::Invalid;
        if (Peer != Binding.ServerPeer) return EResult::WrongPeer;
        if (F.Session != Binding.Session || F.Epoch != Binding.Epoch || F.Schema != Binding.Schema) return EResult::WrongStream;
        if (Last && F.Sequence == Last->Sequence) return SameTransfer(*Last, F) ? EResult::Duplicate : EResult::Conflict;
        if (F.Sequence < Next) return EResult::Stale;
        if (F.Sequence != Next) return EResult::TooFar;
        if (!Header) { Header = F; Header->Bytes.clear(); Parts.resize(F.Count); }
        if (!SameTransfer(*Header, F)) return EResult::Conflict;
        auto& Part = Parts[F.Index];
        if (Part) return *Part == F.Bytes ? EResult::Duplicate : EResult::Conflict;
        Part = F.Bytes; ++Received;
        if (Received != F.Count) return EResult::Accepted;
        std::vector<std::uint8_t> Joined; Joined.reserve(F.TotalBytes);
        for (const auto& P : Parts) Joined.insert(Joined.end(), P->begin(), P->end());
        if (Joined.size() != F.TotalBytes || StateCrc(Joined) != F.Crc) { Failed = true; return EResult::Invalid; }
        Ready = DecodeState(Joined.data(), Joined.size());
        if (!Ready || Ready->Sequence != F.Sequence || Ready->Frame != F.Frame) { Failed = true; return EResult::Invalid; }
        return EResult::Accepted;
    }
    // ACK only after the WHOLE snapshot is retained and structurally validated.
    std::optional<FStateFragment> Acknowledgement() const
    { if (Ready && Header) return StateAck(*Header); if (Last) return StateAck(*Last); return {}; }
    std::optional<FWorldState> TakeReady()
    {
        if (!Ready || Failed) return {};
        auto Result = std::move(Ready); Ready.reset(); Last = Header; Header.reset(); Parts.clear(); Received = 0;
        if (Next == std::numeric_limits<U64>::max()) Failed = true; else ++Next;
        return Result;
    }
private:
    FStateBinding Binding; U64 Next; bool Failed = false; std::size_t Received = 0;
    std::optional<FStateFragment> Header, Last;
    std::vector<std::optional<std::vector<std::uint8_t>>> Parts;
    std::optional<FWorldState> Ready;
};
class FStateOutbox
{
public:
    FStateOutbox(FStateBinding B, U64 Peer) : Binding(B), Destination(Peer) {}
    EResult Queue(const FWorldState& S)
    {
        if (!Fragments.empty()) return EResult::Full;
        if (S.Sequence <= LastSequence) return EResult::Stale;
        auto Parts = FragmentState(S, Binding); if (!Parts || !Destination) return EResult::Invalid;
        Fragments = std::move(*Parts); LastSequence = S.Sequence; return EResult::Accepted;
    }
    bool Acknowledge(U64 Peer, const FStateFragment& F)
    {
        if (Peer != Destination || !ValidFragment(F) || !F.Ack || Fragments.empty() || !SameTransfer(Fragments.front(), F)) return false;
        Fragments.clear(); return true;
    }
    template<class FSend> void Retry(FSend&& Send) const { for (const auto& F : Fragments) Send(F); }
    bool Empty() const { return Fragments.empty(); }
private:
    FStateBinding Binding; U64 Destination, LastSequence = 0;
    std::vector<FStateFragment> Fragments;
};
// Portable lifecycle/retention model only, not an Unreal world restore.
class FReplicatedRoster
{
public:
    using FValidateActor = bool (*)(const FActorState&);
    bool Install(const FWorldState& S, FValidateActor Validator)
    {
        if (!Validator || !ValidState(S) || (Published && (S.Sequence <= Published->Sequence
            || S.Frame < Published->Frame || S.RegistryGeneration < Published->RegistryGeneration))) return false;
        auto Next = Known;
        bool RosterChanged = false;
        for (const auto& A : S.Actors)
        {
            if (!Validator(A)) return false;
            auto It = std::find_if(Next.begin(), Next.end(), [&](const auto& K) { return K.Actor == A.Actor; });
            if (It == Next.end())
            {
                if (Next.size() == MaxStateActors || A.Lifecycle != ELifecycle::Alive) return false;
                Next.push_back({A.Actor, A.Incarnation, A.Type, A.SchemaVersion, true}); RosterChanged = true;
            }
            else
            {
                if (A.Incarnation < It->Incarnation || A.Type != It->Type || A.SchemaVersion != It->Schema) return false;
                if (A.Lifecycle == ELifecycle::Destroyed)
                {
                    if (A.Incarnation != It->Incarnation) return false;
                    RosterChanged |= It->Alive; It->Alive = false;
                }
                else if (A.Incarnation == It->Incarnation)
                { if (!It->Alive) return false; }
                else
                {
                    if (It->Alive) return false; // explicit destruction required before ID reuse
                    It->Incarnation = A.Incarnation; It->Alive = true; RosterChanged = true;
                }
            }
        }
        for (const auto& K : Next) if (K.Alive && std::none_of(S.Actors.begin(), S.Actors.end(), [&](const auto& A)
            { return A.Actor == K.Actor && A.Incarnation == K.Incarnation && A.Lifecycle == ELifecycle::Alive; })) return false;
        if (Published && RosterChanged && S.RegistryGeneration <= Published->RegistryGeneration) return false;
        std::optional<FWorldState> Candidate(S); // all allocations before the two noexcept swaps
        Published.swap(Candidate); Known.swap(Next); return true;
    }
    const std::optional<FWorldState>& Read() const { return Published; }
private:
    struct FKnown { U64 Actor, Incarnation; std::uint32_t Type, Schema; bool Alive; };
    std::vector<FKnown> Known; // incarnation high water; bounded for the epoch
    std::optional<FWorldState> Published;
};
}
