#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <vector>

namespace Speed::PreparedWorld
{
using U64 = std::uint64_t;
using U32 = std::uint32_t;
using Byte = std::uint8_t;
// This is the frozen2776 LOCAL schema-2 format, not a new network protocol.
// Its scalar byte order/widths are explicitly the admitted Windows little-endian
// binary64 FVector/FQuat + binary32 contact fields. No padding is serialized.
constexpr U32 Schema = 2;
constexpr std::size_t PairBytes = 198; // 2*u64 + 15*f64 + 7*u32 + 8*f32 + 2*u8
constexpr std::size_t BodyFixedBytes = 165; // id + 19*f64 + frozen + payload size
constexpr U64 CompleteActorCoverage = (U64{1} << 10) - 1;
enum EActorCoverage : U64
{
    Kinematic = U64{1} << 0, Frozen = U64{1} << 1, Mechanics = U64{1} << 2,
    Gameplay = U64{1} << 3, SubBodies = U64{1} << 4, Sleep = U64{1} << 5,
    StaticContact = U64{1} << 6, Quantization = U64{1} << 7,
    Configuration = U64{1} << 8, Lifecycle = U64{1} << 9
};
constexpr U64 CompleteWorldCoverage = 15;
enum EWorldCoverage : U64
{
    WorldGameplay = 1, StaticGeometryAuthority = 2, CompleteRegistry = 4,
    ActorLifecycle = 8
};
struct FAddress
{
    U64 Authority = 0, WorldIncarnation = 0, TopologyVersion = 0, StateVersion = 0;
    U64 Frame = 0, Next = 0, InputHash = 0, StateHash = 0;
    U64 ResumeGeneration = 0, PublicationSerial = 0, SupplementHash = 0;
    bool Valid() const noexcept
    {
        return Authority && WorldIncarnation && TopologyVersion && StateVersion
            && Frame < UINT32_MAX && Next == Frame + 1 && InputHash && StateHash
            && ResumeGeneration && PublicationSerial && SupplementHash;
    }
    bool operator==(const FAddress& B) const noexcept
    {
        return Authority == B.Authority && WorldIncarnation == B.WorldIncarnation
            && TopologyVersion == B.TopologyVersion && StateVersion == B.StateVersion
            && Frame == B.Frame && Next == B.Next && InputHash == B.InputHash
            && StateHash == B.StateHash && ResumeGeneration == B.ResumeGeneration
            && PublicationSerial == B.PublicationSerial && SupplementHash == B.SupplementHash;
    }
};
struct FActorBinding
{
    U64 StableId = 0, Incarnation = 0, ConfigurationHash = 0;
    U64 MechanicalSchema = 0, GameplaySchema = 0, RequiredCoverage = CompleteActorCoverage;
    bool Valid() const noexcept
    {
        return StableId && Incarnation && ConfigurationHash && MechanicalSchema
            && GameplaySchema && RequiredCoverage == CompleteActorCoverage;
    }
    bool operator==(const FActorBinding& B) const noexcept
    {
        return StableId == B.StableId && Incarnation == B.Incarnation
            && ConfigurationHash == B.ConfigurationHash && MechanicalSchema == B.MechanicalSchema
            && GameplaySchema == B.GameplaySchema && RequiredCoverage == B.RequiredCoverage;
    }
};
struct FEndpointBinding
{
    U64 StableId = 0, Incarnation = 0, Actor = 0, ActorIncarnation = 0;
    U32 NativeObjectId = 0; // ONLY local lookup key; never sent as wire identity
};
struct FActorSupplement
{
    FActorBinding Binding;
    std::vector<Byte> MechanicalExtension, GameplayPayload;
};
struct FRestoreEnvelope
{
    // LOCAL preparation envelope. Endpoint NativeObjectId is resolved locally
    // under the lease and MUST NOT be copied into a network snapshot identity.
    FAddress Address;
    std::vector<FActorBinding> Actors; // complete exact roster, sorted stable id
    std::vector<FEndpointBinding> Endpoints; // complete solids, sorted stable id
    std::vector<FActorSupplement> ActorSupplements; // same exact actor order
    U64 WorldSchema = 0, WorldCoverage = CompleteWorldCoverage;
    std::vector<Byte> WorldPayload;
};
inline U64 HashSupplement(const FRestoreEnvelope& E) noexcept
{
    U64 H = 1469598103934665603ull;
    const auto UInt = [&H](U64 V, std::size_t Width) noexcept
    { for (std::size_t I = 0; I < Width; ++I) { H ^= Byte(V & 255); H *= 1099511628211ull; V >>= 8; } };
    const auto Blob = [&UInt](const std::vector<Byte>& B) noexcept
    { UInt(B.size(), 8); for (const auto V : B) UInt(V, 1); };
    const auto Binding = [&UInt](const FActorBinding& B) noexcept
    { UInt(B.StableId, 8); UInt(B.Incarnation, 8); UInt(B.ConfigurationHash, 8);
      UInt(B.MechanicalSchema, 8); UInt(B.GameplaySchema, 8); UInt(B.RequiredCoverage, 8); };
    // Domain + every address field EXCEPT this digest itself (no hash cycle).
    UInt(0x315253574e, 8); UInt(E.Address.Authority, 8); UInt(E.Address.WorldIncarnation, 8);
    UInt(E.Address.TopologyVersion, 8); UInt(E.Address.StateVersion, 8); UInt(E.Address.Frame, 8);
    UInt(E.Address.Next, 8); UInt(E.Address.InputHash, 8); UInt(E.Address.StateHash, 8);
    UInt(E.Address.ResumeGeneration, 8); UInt(E.Address.PublicationSerial, 8);
    UInt(E.Actors.size(), 8); for (const auto& B : E.Actors) Binding(B);
    UInt(E.Endpoints.size(), 8); for (const auto& B : E.Endpoints)
    { UInt(B.StableId, 8); UInt(B.Incarnation, 8); UInt(B.Actor, 8); UInt(B.ActorIncarnation, 8); UInt(B.NativeObjectId, 4); }
    UInt(E.ActorSupplements.size(), 8); for (const auto& S : E.ActorSupplements)
    { Binding(S.Binding); Blob(S.MechanicalExtension); Blob(S.GameplayPayload); }
    UInt(E.WorldSchema, 8); UInt(E.WorldCoverage, 8); Blob(E.WorldPayload);
    return H ? H : 1; // integrity only; not authentication
}
struct FLimits
{
    std::size_t MaxPayload = 16 * 1024 * 1024, MaxBodies = 4096;
    std::size_t MaxMechanicPayload = 1024 * 1024, MaxPairs = 8192, MaxEndpoints = 65536;
};
struct FBody
{
    U64 StableId = 0;
    std::array<double, 19> Kinematic{};
    bool Frozen = false;
    std::vector<Byte> MechanicPayload;
};
struct FPair
{
    U64 A = 0, B = 0;
    std::array<double, 15> Geometry{};
    std::array<U32, 7> History{};
    std::array<float, 8> Activity{};
    bool Rolling = false, Active = false;
};
struct FParsedWorld
{
    U64 Frame = 0, InputHash = 0, StateHash = 0;
    std::vector<FBody> Bodies;
    std::vector<FPair> Dynamic, Pending;
};
inline U64 HashBytes(const Byte* Data, std::size_t Size) noexcept
{
    U64 H = 1469598103934665603ull;
    for (std::size_t I = 0; I < Size; ++I) { H ^= Data[I]; H *= 1099511628211ull; }
    return H ? H : 1;
}
class FReader
{
    const Byte* Data; std::size_t Size, At = 0;
public:
    FReader(const Byte* D, std::size_t N) noexcept : Data(D), Size(N) {}
    std::size_t Remaining() const noexcept { return Size - At; }
    bool UInt(std::size_t Width, U64& Out) noexcept
    {
        if (Width > 8 || Width > Remaining()) return false;
        Out = 0;
        for (std::size_t I = 0; I < Width; ++I) Out |= U64(Data[At++]) << (I * 8);
        return true;
    }
    template<class T> bool Float(T& Out) noexcept
    {
        static_assert(std::numeric_limits<T>::is_iec559 && (sizeof(T) == 4 || sizeof(T) == 8));
        U64 Bits = 0; if (!UInt(sizeof(T), Bits)) return false;
        if constexpr (sizeof(T) == 4) { const U32 Small = static_cast<U32>(Bits); std::memcpy(&Out, &Small, 4); }
        else std::memcpy(&Out, &Bits, 8);
        return std::isfinite(Out); // reject corrupt numeric states; never normalize or clamp
    }
    bool Bytes(std::size_t Count, std::vector<Byte>& Out)
    {
        if (Count > Remaining()) return false;
        Out.assign(Data + At, Data + At + Count); At += Count; return true;
    }
};
inline bool ValidBindings(const std::vector<FActorBinding>& Actors,
    const std::vector<FEndpointBinding>& Endpoints, const FLimits& Limits)
{
    if (Actors.empty() || Actors.size() > Limits.MaxBodies || Endpoints.size() > Limits.MaxEndpoints) return false;
    U64 Previous = 0;
    for (const auto& B : Actors) { if (!B.Valid() || B.StableId <= Previous) return false; Previous = B.StableId; }
    std::set<U32> NativeIds; Previous = 0;
    for (const auto& E : Endpoints)
    {
        if (!E.StableId || !E.Incarnation || !E.Actor || !E.ActorIncarnation || !E.NativeObjectId
            || E.StableId <= Previous || !NativeIds.insert(E.NativeObjectId).second) return false;
        Previous = E.StableId;
        bool Found = false;
        for (const auto& B : Actors)
            if (B.StableId == E.Actor && B.Incarnation == E.ActorIncarnation) { Found = true; break; }
        if (!Found) return false;
    }
    return true;
}
inline bool ValidEnvelope(const FRestoreEnvelope& E, const FLimits& Limits = {})
{
    if (!E.Address.Valid() || !ValidBindings(E.Actors, E.Endpoints, Limits)
        || !E.WorldSchema || E.WorldCoverage != CompleteWorldCoverage
        || E.ActorSupplements.size() != E.Actors.size() || E.WorldPayload.size() > Limits.MaxPayload) return false;
    std::size_t Bytes = E.WorldPayload.size();
    for (std::size_t I = 0; I < E.Actors.size(); ++I)
    {
        const auto& S = E.ActorSupplements[I];
        if (!(S.Binding == E.Actors[I]) || S.MechanicalExtension.size() > Limits.MaxMechanicPayload
            || S.GameplayPayload.size() > Limits.MaxMechanicPayload) return false;
        if (S.MechanicalExtension.size() > Limits.MaxPayload - Bytes) return false;
        Bytes += S.MechanicalExtension.size();
        if (S.GameplayPayload.size() > Limits.MaxPayload - Bytes) return false;
        Bytes += S.GameplayPayload.size();
    }
    return HashSupplement(E) == E.Address.SupplementHash;
}
inline std::optional<FParsedWorld> Parse(const Byte* Data, std::size_t Size,
    const FAddress& Address, const std::vector<FActorBinding>& Bindings, const FLimits& Limits = {})
{
    if (!Address.Valid() || !Data || Size < 32 || Size > Limits.MaxPayload
        || HashBytes(Data, Size) != Address.StateHash || Bindings.empty() || Bindings.size() > Limits.MaxBodies) return {};
    U64 Previous = 0;
    for (const auto& B : Bindings) { if (!B.Valid() || B.StableId <= Previous) return {}; Previous = B.StableId; }
    FReader R(Data, Size); U64 V = 0, Count = 0; FParsedWorld Out; Out.StateHash = Address.StateHash;
    if (!R.UInt(4, V) || V != Schema || !R.UInt(8, Out.Frame) || Out.Frame != Address.Frame
        || !R.UInt(8, Out.InputHash) || Out.InputHash != Address.InputHash
        || !R.UInt(4, Count) || Count != Bindings.size() || Count > R.Remaining() / BodyFixedBytes) return {};
    Out.Bodies.reserve(static_cast<std::size_t>(Count)); std::set<U64> SeenBodies;
    for (U64 I = 0; I < Count; ++I)
    {
        FBody B;
        if (!R.UInt(8, B.StableId) || !SeenBodies.insert(B.StableId).second) return {};
        bool Known = false; for (const auto& E : Bindings) if (E.StableId == B.StableId) { Known = true; break; }
        if (!Known) return {};
        for (auto& X : B.Kinematic) if (!R.Float(X)) return {};
        if (!R.UInt(1, V) || V > 1) return {}; B.Frozen = V != 0;
        if (!R.UInt(4, V) || V > Limits.MaxMechanicPayload || !R.Bytes(static_cast<std::size_t>(V), B.MechanicPayload)) return {};
        Out.Bodies.push_back(std::move(B));
    }
    std::set<std::pair<U64, U64>> SeenPairs;
    for (auto* Collection : {&Out.Dynamic, &Out.Pending})
    {
        if (!R.UInt(4, Count) || Count > Limits.MaxPairs - SeenPairs.size() || Count > R.Remaining() / PairBytes) return {};
        Collection->reserve(static_cast<std::size_t>(Count));
        for (U64 I = 0; I < Count; ++I)
        {
            FPair P;
            if (!R.UInt(8, P.A) || !R.UInt(8, P.B) || !P.A || !P.B || P.A == P.B) return {};
            const auto Key = std::minmax(P.A, P.B);
            if (!SeenPairs.emplace(Key.first, Key.second).second) return {};
            for (auto& X : P.Geometry) if (!R.Float(X)) return {};
            for (auto& X : P.History) { if (!R.UInt(4, V)) return {}; X = static_cast<U32>(V); }
            for (auto& X : P.Activity) if (!R.Float(X)) return {};
            if (!R.UInt(1, V) || V > 1) return {}; P.Rolling = V != 0;
            if (!R.UInt(1, V) || V > 1) return {}; P.Active = V != 0;
            Collection->push_back(std::move(P));
        }
    }
    return R.Remaining() == 0 ? std::optional<FParsedWorld>(std::move(Out)) : std::nullopt;
}
struct FContactIndexes
{
    std::map<U64, std::size_t> Dynamic, Pending;
    std::map<U32, std::set<U64>> DynamicByBody;
    std::vector<U64> DynamicKeys, PendingKeys;
};
inline std::optional<FContactIndexes> BuildContactIndexes(const FParsedWorld& World,
    const std::vector<FActorBinding>& Actors, const std::vector<FEndpointBinding>& Endpoints, const FLimits& Limits = {})
{
    if (!ValidBindings(Actors, Endpoints, Limits) || World.Dynamic.size() > Limits.MaxPairs
        || World.Pending.size() > Limits.MaxPairs - World.Dynamic.size()) return {};
    FContactIndexes Out; std::set<U64> Seen;
    for (const auto Item : {std::make_pair(&World.Dynamic, true), std::make_pair(&World.Pending, false)})
    {
        auto& Index = Item.second ? Out.Dynamic : Out.Pending;
        auto& Keys = Item.second ? Out.DynamicKeys : Out.PendingKeys;
        Keys.reserve(Item.first->size());
        for (const auto& P : *Item.first)
        {
            const FEndpointBinding* A = nullptr; const FEndpointBinding* B = nullptr;
            for (const auto& E : Endpoints) { if (E.StableId == P.A) A = &E; if (E.StableId == P.B) B = &E; }
            if (!A || !B || A == B || A->NativeObjectId == B->NativeObjectId) return {};
            const auto Low = A->NativeObjectId < B->NativeObjectId ? A->NativeObjectId : B->NativeObjectId;
            const auto High = A->NativeObjectId < B->NativeObjectId ? B->NativeObjectId : A->NativeObjectId;
            const U64 Key = (U64(Low) << 32) | High;
            if (!Seen.insert(Key).second) return {};
            Index.emplace(Key, Keys.size()); Keys.push_back(Key);
            if (Item.second) { Out.DynamicByBody[Low].insert(Key); Out.DynamicByBody[High].insert(Key); }
        }
    }
    return Out;
}
}
