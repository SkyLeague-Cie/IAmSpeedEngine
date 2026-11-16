#pragma once
#include "IAmSpeed/World/Simulation/PreparedWorldCore.h"
#include <algorithm>
#include <optional>
#include <vector>

namespace Speed::CommonState
{
using PreparedWorld::Byte; using PreparedWorld::U64; using PreparedWorld::U32;
using PreparedWorld::FActorBinding;
// Source address deliberately excludes SupplementHash: these bytes may become
// an ActorSupplement extension and cannot contain their own outer digest.
struct FSourceAddress
{
    U64 Authority = 0, WorldIncarnation = 0, TopologyVersion = 0, StateVersion = 0;
    U64 Frame = 0, Next = 0, InputHash = 0, LegacyStateHash = 0;
    U64 ResumeGeneration = 0, PublicationSerial = 0;
    bool Valid() const noexcept
    { return Authority && WorldIncarnation && TopologyVersion && StateVersion && Frame < UINT32_MAX
        && Next == Frame + 1 && InputHash && LegacyStateHash && ResumeGeneration && PublicationSerial; }
    bool operator==(const FSourceAddress& B) const noexcept
    { return Authority == B.Authority && WorldIncarnation == B.WorldIncarnation && TopologyVersion == B.TopologyVersion
        && StateVersion == B.StateVersion && Frame == B.Frame && Next == B.Next && InputHash == B.InputHash
        && LegacyStateHash == B.LegacyStateHash && ResumeGeneration == B.ResumeGeneration && PublicationSerial == B.PublicationSerial; }
};
struct FSleepValues
{
    std::array<double, 3> Location{};
    std::array<double, 4> Rotation{0, 0, 0, 1};
    bool HasCompletedPose = false;
};
struct FQuantizationValues
{
    // Location3, Velocity3, AngularVelocity3, Rotation4, PreviousRotation4.
    // Accelerations are intentionally absent from the ACTUAL cache key.
    std::array<double, 17> NoOpInput{};
    bool Valid = false;
};
enum class EReferenceKind : Byte { None, ActorComponent, SubBody, StaticPrimitive, StaticQueryAuthority };
struct FReference
{
    EReferenceKind Kind = EReferenceKind::None;
    U64 Id = 0, Incarnation = 0, Owner = 0, OwnerIncarnation = 0, ConfigurationHash = 0;
    bool Valid() const noexcept
    {
        if (Kind == EReferenceKind::None) return !Id && !Incarnation && !Owner && !OwnerIncarnation && !ConfigurationHash;
        return Kind <= EReferenceKind::StaticQueryAuthority && Id && Incarnation && Owner && OwnerIncarnation && ConfigurationHash;
    }
    bool operator==(const FReference& B) const noexcept
    { return Kind == B.Kind && Id == B.Id && Incarnation == B.Incarnation && Owner == B.Owner
        && OwnerIncarnation == B.OwnerIncarnation && ConfigurationHash == B.ConfigurationHash; }
};
struct FConstraintValues
{
    std::array<double, 3> Normal{}, ContactPoint{};
    FReference OtherComponent, SourceSubBody;
    float PenetrationDepth = 0, TOI = 0;
    Byte FramesSinceSeen = 0;
    bool Persistent = false;
    // No runtime PairKey is sent: local constraint keys use HIGH<<32|LOW,
    // unlike FSimulationWorld's contact-map LOW<<32|HIGH convention.
};
struct FComponentValues
{
    FSourceAddress Address;
    FActorBinding Actor;
    FSleepValues Sleep;
    FQuantizationValues Quantization;
    std::vector<FConstraintValues> Constraints; // exact order, never register/normalize/sort/dedupe
    FReference StaticRestingWorld;
    float StaticSupportFrameHorizon = 0;
};
constexpr std::size_t ReferenceBytes = 41, ConstraintBytes = 140, HeaderBytes = 379;
constexpr std::size_t MaxConstraints = 1024;
template<class Range> inline bool Finite(const Range& R) noexcept
{ for (const auto V : R) if (!std::isfinite(V)) return false; return true; }
inline bool ValidCaches(const FSleepValues& S, const FQuantizationValues& Q) noexcept
{ return Finite(S.Location) && Finite(S.Rotation) && Finite(Q.NoOpInput); }
inline bool Valid(const FComponentValues& V) noexcept
{
    if (!V.Address.Valid() || !V.Actor.Valid() || !ValidCaches(V.Sleep, V.Quantization)
        || V.Constraints.size() > MaxConstraints || !V.StaticRestingWorld.Valid()
        || (V.StaticRestingWorld.Kind != EReferenceKind::None && V.StaticRestingWorld.Kind != EReferenceKind::StaticQueryAuthority)
        || !std::isfinite(V.StaticSupportFrameHorizon)) return false;
    // The source setter maps a missing borrowed authority to horizon=+0.
    // At arbitrary frame phases do not invent this equivalence: preserve the
    // exact stored bits, including -0.0 and a bound authority with horizon0.
    for (const auto& C : V.Constraints)
    {
        if (!Finite(C.Normal) || !Finite(C.ContactPoint) || !C.OtherComponent.Valid() || !C.SourceSubBody.Valid()
            || !std::isfinite(C.PenetrationDepth) || !std::isfinite(C.TOI)
            || C.OtherComponent.Kind == EReferenceKind::StaticQueryAuthority
            || (C.SourceSubBody.Kind != EReferenceKind::None && C.SourceSubBody.Kind != EReferenceKind::SubBody)) return false;
        if (C.SourceSubBody.Kind != EReferenceKind::None
            && (C.SourceSubBody.Owner != V.Actor.StableId || C.SourceSubBody.OwnerIncarnation != V.Actor.Incarnation)) return false;
    }
    return true;
}
inline void Put(std::vector<Byte>& B, U64 V, std::size_t Width)
{ for (std::size_t I = 0; I < Width; ++I) { B.push_back(Byte(V & 255)); V >>= 8; } }
template<class T> inline void PutFloat(std::vector<Byte>& B, T V)
{
    U64 Bits = 0;
    if constexpr (sizeof(T) == 4) { U32 Small = 0; std::memcpy(&Small, &V, 4); Bits = Small; }
    else std::memcpy(&Bits, &V, 8);
    Put(B, Bits, sizeof(T));
}
inline void PutReference(std::vector<Byte>& B, const FReference& R)
{ Put(B, Byte(R.Kind), 1); for (const U64 V : {R.Id, R.Incarnation, R.Owner, R.OwnerIncarnation, R.ConfigurationHash}) Put(B, V, 8); }
inline std::optional<std::vector<Byte>> Encode(const FComponentValues& V)
{
    if (!Valid(V)) return {};
    std::vector<Byte> B; B.reserve(HeaderBytes + V.Constraints.size() * ConstraintBytes);
    Put(B, 0x3143534e, 4); Put(B, 1, 2); Put(B, 7, 2); // BOTH caches + complete ordered constraint collection
    for (const U64 X : {V.Address.Authority, V.Address.WorldIncarnation, V.Address.TopologyVersion, V.Address.StateVersion,
        V.Address.Frame, V.Address.Next, V.Address.InputHash, V.Address.LegacyStateHash, V.Address.ResumeGeneration, V.Address.PublicationSerial}) Put(B, X, 8);
    for (const U64 X : {V.Actor.StableId, V.Actor.Incarnation, V.Actor.ConfigurationHash, V.Actor.MechanicalSchema, V.Actor.GameplaySchema, V.Actor.RequiredCoverage}) Put(B, X, 8);
    for (const auto X : V.Sleep.Location) PutFloat(B, X); for (const auto X : V.Sleep.Rotation) PutFloat(B, X);
    Put(B, V.Sleep.HasCompletedPose ? 1 : 0, 1);
    for (const auto X : V.Quantization.NoOpInput) PutFloat(B, X); Put(B, V.Quantization.Valid ? 1 : 0, 1);
    PutReference(B, V.StaticRestingWorld); PutFloat(B, V.StaticSupportFrameHorizon); Put(B, V.Constraints.size(), 4);
    for (const auto& C : V.Constraints)
    {
        for (const auto X : C.Normal) PutFloat(B, X); for (const auto X : C.ContactPoint) PutFloat(B, X);
        PutReference(B, C.OtherComponent); PutReference(B, C.SourceSubBody);
        PutFloat(B, C.PenetrationDepth); PutFloat(B, C.TOI); Put(B, C.FramesSinceSeen, 1); Put(B, C.Persistent ? 1 : 0, 1);
    }
    return B;
}
inline bool ReadReference(PreparedWorld::FReader& R, FReference& Out)
{
    U64 V = 0; if (!R.UInt(1, V) || V > Byte(EReferenceKind::StaticQueryAuthority)) return false; Out.Kind = EReferenceKind(Byte(V));
    return R.UInt(8, Out.Id) && R.UInt(8, Out.Incarnation) && R.UInt(8, Out.Owner)
        && R.UInt(8, Out.OwnerIncarnation) && R.UInt(8, Out.ConfigurationHash) && Out.Valid();
}
inline std::optional<FComponentValues> Decode(const std::vector<Byte>& B, const FSourceAddress& Expected, const FActorBinding& Actor)
{
    if (B.size() < HeaderBytes || B.size() > HeaderBytes + MaxConstraints * ConstraintBytes || !Expected.Valid() || !Actor.Valid()) return {};
    PreparedWorld::FReader R(B.data(), B.size()); U64 V = 0; FComponentValues O;
    if (!R.UInt(4, V) || V != 0x3143534e || !R.UInt(2, V) || V != 1 || !R.UInt(2, V) || V != 7) return {};
    for (U64* X : {&O.Address.Authority, &O.Address.WorldIncarnation, &O.Address.TopologyVersion, &O.Address.StateVersion,
        &O.Address.Frame, &O.Address.Next, &O.Address.InputHash, &O.Address.LegacyStateHash, &O.Address.ResumeGeneration, &O.Address.PublicationSerial}) if (!R.UInt(8, *X)) return {};
    for (U64* X : {&O.Actor.StableId, &O.Actor.Incarnation, &O.Actor.ConfigurationHash, &O.Actor.MechanicalSchema, &O.Actor.GameplaySchema, &O.Actor.RequiredCoverage}) if (!R.UInt(8, *X)) return {};
    if (!(O.Address == Expected) || !(O.Actor == Actor)) return {};
    for (auto& X : O.Sleep.Location) if (!R.Float(X)) return {}; for (auto& X : O.Sleep.Rotation) if (!R.Float(X)) return {};
    if (!R.UInt(1, V) || V > 1) return {}; O.Sleep.HasCompletedPose = V != 0;
    for (auto& X : O.Quantization.NoOpInput) if (!R.Float(X)) return {};
    if (!R.UInt(1, V) || V > 1) return {}; O.Quantization.Valid = V != 0;
    if (!ReadReference(R, O.StaticRestingWorld) || !R.Float(O.StaticSupportFrameHorizon)
        || !R.UInt(4, V) || V > MaxConstraints || R.Remaining() != V * ConstraintBytes) return {};
    O.Constraints.reserve(static_cast<std::size_t>(V));
    const auto Count = V;
    for (U64 I = 0; I < Count; ++I)
    {
        FConstraintValues C;
        for (auto& X : C.Normal) if (!R.Float(X)) return {}; for (auto& X : C.ContactPoint) if (!R.Float(X)) return {};
        if (!ReadReference(R, C.OtherComponent) || !ReadReference(R, C.SourceSubBody)
            || !R.Float(C.PenetrationDepth) || !R.Float(C.TOI) || !R.UInt(1, V)) return {};
        C.FramesSinceSeen = Byte(V); if (!R.UInt(1, V) || V > 1) return {}; C.Persistent = V != 0;
        O.Constraints.push_back(std::move(C));
    }
    return R.Remaining() == 0 && Valid(O) ? std::optional<FComponentValues>(std::move(O)) : std::nullopt;
}
inline U64 ConstraintPairKey(U32 SourceObjectId, U32 OtherObjectId) noexcept
{ return (U64(std::max(SourceObjectId, OtherObjectId)) << 32) | std::min(SourceObjectId, OtherObjectId); }

// Process-local memory access declaration. No pointer is serialized. Every
// participant must supply its WHOLE write footprints and typed read-only config
// closure, including backing allocations. This validates geometry, not lifetime:
// the real coordinator must retain actual ownership/version/lifetime fences.
struct FSpan { std::uintptr_t Begin = 0; std::size_t Bytes = 0; };
struct FParticipantAccess
{
    U64 Participant = 0;
    std::vector<FSpan> Writes, ReadOnly;
};
inline bool ValidSpan(FSpan S) noexcept
{ return S.Begin && S.Bytes && S.Bytes - 1 <= std::numeric_limits<std::uintptr_t>::max() - S.Begin; }
inline bool Overlap(FSpan A, FSpan B) noexcept
{ return A.Begin <= B.Begin ? B.Begin - A.Begin < A.Bytes : A.Begin - B.Begin < B.Bytes; }
inline bool ValidateGlobalAccess(const std::vector<FParticipantAccess>& Plans) noexcept
{
    for (std::size_t I = 0; I < Plans.size(); ++I)
    {
        const auto& P = Plans[I]; if (!P.Participant || P.Writes.empty()) return false;
        for (std::size_t J = 0; J < I; ++J) if (P.Participant == Plans[J].Participant) return false;
        for (const auto S : P.Writes) if (!ValidSpan(S)) return false;
        for (const auto S : P.ReadOnly) if (!ValidSpan(S)) return false;
    }
    for (std::size_t I = 0; I < Plans.size(); ++I)
        for (std::size_t A = 0; A < Plans[I].Writes.size(); ++A)
        {
            const auto Write = Plans[I].Writes[A];
            for (std::size_t J = 0; J < Plans.size(); ++J)
            {
                for (const auto Read : Plans[J].ReadOnly) if (Overlap(Write, Read)) return false;
                for (std::size_t B = 0; B < Plans[J].Writes.size(); ++B)
                    if ((J > I || (J == I && B > A)) && Overlap(Write, Plans[J].Writes[B])) return false;
            }
        }
    return !Plans.empty(); // read-only/read-only sharing is allowed
}
}
