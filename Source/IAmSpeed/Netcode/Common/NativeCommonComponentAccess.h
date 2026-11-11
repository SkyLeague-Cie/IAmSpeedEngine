#pragma once
#include "NativeCommonCacheAccess.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "IAmSpeed/SubBodies/SSubBody.h"
#include "Components/PrimitiveComponent.h"
#include "IAmSpeed/World/Simulation/PreparedSimulationRestore.h"
#include <utility>

namespace Speed
{
// Source-only partial common-field helper. NOT a complete component restore
// provider and NOT wired into PrepareCompleteSimulationRestore. Native type,
// resolver, actual lifetime fences and TArray exchange remain unqualified.
struct FCommonComponentValueAccess
{
    struct FCapturedReference { CommonState::FReference Identity; uint32 NativeObjectId = 0; };
    static CommonState::FSourceAddress SourceAddress(const PreparedWorld::FAddress& A) noexcept
    { return {A.Authority, A.WorldIncarnation, A.TopologyVersion, A.StateVersion, A.Frame, A.Next,
        A.InputHash, A.StateHash, A.ResumeGeneration, A.PublicationSerial}; }
    static bool CapturedIdentityValid(const FCapturedReference& R) noexcept
    { return R.Identity.Valid() && ((R.Identity.Kind == CommonState::EReferenceKind::None) == (R.NativeObjectId == 0)); }
    // Resolver must distinguish explicit null from stale/invalid weak handles.
    // A stale/unregistered weak handle returns NO RESULT, never a fabricated null.
    // Static/actor component identities require the authoritative complete binding
    // registry plus configuration/geometry/lifetime pins held by the outer lease.
    template<class Resolver> static std::optional<CommonState::FComponentValues> Capture(
        const ISpeedComponent& C, const FSimulationWorld& World, const ISimulationRestoreBoundaryLease& Lease,
        const PreparedWorld::FAddress& At, const PreparedWorld::FActorBinding& Actor, Resolver& Resolve)
    {
        if (!At.Valid() || !Actor.Valid() || C.GetPublishedSimulationStableId() != Actor.StableId
            || !Lease.ProtectsEscapedMutableViews() || !Lease.HoldsExactBoundary(World, At)
            || !Resolve.ValidateComponentBinding(C, Actor)) return {};
        CommonState::FComponentValues O; O.Address = SourceAddress(At); O.Actor = Actor;
        O.Sleep = FCommonCacheValueAccess::Capture(C.SleepState);
        O.Quantization = FCommonCacheValueAccess::Capture(C.KinematicQuantizationCache);
        O.StaticSupportFrameHorizon = C.StaticSupportFrameHorizon;
        auto Static = Resolve.CaptureStaticAuthority(C.StaticRestingWorld);
        if (!Static) return {}; O.StaticRestingWorld = *Static;
        O.Constraints.reserve(static_cast<std::size_t>(C.PhysicalConstraints.Num()));
        for (const FPhysicalContactConstraint& P : C.PhysicalConstraints)
        {
            auto Other = Resolve.CaptureOther(P.OtherComponent); auto Source = Resolve.CaptureSubBody(P.SourceSubBody);
            if (!Other || !Source || !CapturedIdentityValid(*Other) || !CapturedIdentityValid(*Source)
                || P.PairKey != CommonState::ConstraintPairKey(Source->NativeObjectId, Other->NativeObjectId)) return {};
            CommonState::FConstraintValues V;
            V.Normal = {P.Normal.X, P.Normal.Y, P.Normal.Z}; V.ContactPoint = {P.ContactPoint.X, P.ContactPoint.Y, P.ContactPoint.Z};
            V.OtherComponent = Other->Identity; V.SourceSubBody = Source->Identity;
            V.PenetrationDepth = P.PenetrationDepth; V.TOI = P.TOI; V.FramesSinceSeen = P.FramesSinceSeen; V.Persistent = P.bPersistent;
            O.Constraints.push_back(std::move(V));
        }
        return CommonState::Valid(O) ? std::optional<CommonState::FComponentValues>(std::move(O)) : std::nullopt;
    }
    class FPreparedCommonFields
    {
        friend struct FCommonComponentValueAccess;
        FCommonCacheValueAccess::FPreparedCaches Caches;
        TArray<FPhysicalContactConstraint> Constraints;
        const IStaticCollisionWorld* StaticAuthority = nullptr;
        float Horizon = 0;
        uint64 PublishedStableId = 0;
        ISpeedComponent* Target = nullptr;
        CommonState::FParticipantAccess Access;
        bool Finalized = false, Released = false;
        FPreparedCommonFields() = default;
    public:
        FPreparedCommonFields(const FPreparedCommonFields&) = delete;
        FPreparedCommonFields& operator=(const FPreparedCommonFields&) = delete;
        ~FPreparedCommonFields() noexcept { if (Finalized && !Released) std::terminate(); }
        const CommonState::FParticipantAccess& DeclaredAccess() const noexcept { return Access; }
        // Caller MUST have acquired/revalidated all participants and run
        // ValidateGlobalAccess across ALL writes vs ALL read-only config ranges.
        // No local boolean can replace the global retained native lease.
        void FinalizeTo(ISpeedComponent& C) noexcept
        {
            if (Finalized || Released || &C != Target || C.GetPublishedSimulationStableId() != PublishedStableId) std::terminate();
            if constexpr (!std::is_nothrow_swappable_v<TArray<FPhysicalContactConstraint>>) std::terminate();
            else
            {
                using std::swap; swap(C.PhysicalConstraints, Constraints);
                Caches.CopyTo(C.SleepState, C.KinematicQuantizationCache);
                C.StaticRestingWorld = StaticAuthority; C.StaticSupportFrameHorizon = Horizon;
                Finalized = true;
            }
        }
        void ReleaseAfterCommonPublication() noexcept
        { if (!Finalized || Released) std::terminate(); Released = true; }
    };
    // ResolveOther/ResolveSubBody return typed pointers already pinned and exact
    // identity-validated under Lease; nullptr is accepted ONLY for explicit null.
    // ResolveStaticAuthority returns pointer + the entire typed immutable config
    // closure as ReadOnly spans, not one-byte placeholder pointer spans.
    // No resolver implementation is supplied or admitted by this artifact.
    template<class Resolver> static std::unique_ptr<FPreparedCommonFields> Prepare(
        ISpeedComponent& C, const FSimulationWorld& World, const ISimulationRestoreBoundaryLease& Lease,
        const PreparedWorld::FAddress& At, const CommonState::FComponentValues& Values,
        uint64 ParticipantIdentity, Resolver& Resolve)
    {
        if constexpr (!std::is_nothrow_swappable_v<TArray<FPhysicalContactConstraint>>) return {};
        if (!ParticipantIdentity || !At.Valid() || !CommonState::Valid(Values) || !(Values.Address == SourceAddress(At))
            || C.GetPublishedSimulationStableId() != Values.Actor.StableId
            || !Lease.ProtectsEscapedMutableViews() || !Lease.HoldsExactBoundary(World, At)
            || !Resolve.ValidateComponentBinding(C, Values.Actor)) return {};
        auto Caches = FCommonCacheValueAccess::Prepare(Values.Sleep, Values.Quantization);
        if (!Caches) return {};
        auto Plan = std::unique_ptr<FPreparedCommonFields>(new FPreparedCommonFields()); Plan->Caches = *Caches;
        Plan->PublishedStableId = Values.Actor.StableId; Plan->Horizon = Values.StaticSupportFrameHorizon; Plan->Target = &C;
        auto Authority = Resolve.ResolveStaticAuthority(Values.StaticRestingWorld);
        if (!Authority) return {};
        Plan->StaticAuthority = Authority->Pointer;
        if (!(Authority->Identity == Values.StaticRestingWorld)
            || (Values.StaticRestingWorld.Kind == CommonState::EReferenceKind::None) != (Plan->StaticAuthority == nullptr)
            || (Plan->StaticAuthority && Authority->ReadOnly.empty())) return {};
        Plan->Access.Participant = ParticipantIdentity; // unique PLAN identity, not necessarily actor id
        Plan->Access.ReadOnly = std::move(Authority->ReadOnly);
        Plan->Constraints.Reserve(static_cast<int32>(Values.Constraints.size()));
        for (const auto& V : Values.Constraints)
        {
            auto Other = Resolve.ResolveOther(V.OtherComponent); auto Source = Resolve.ResolveSubBody(V.SourceSubBody);
            if (!Other || !Source) return {};
            // Exact pointer/identity match is resolver responsibility under Lease.
            if (!(Other->Identity == V.OtherComponent) || !(Source->Identity == V.SourceSubBody)
                || (V.OtherComponent.Kind == CommonState::EReferenceKind::None) != (Other->Pointer == nullptr)
                || (V.SourceSubBody.Kind == CommonState::EReferenceKind::None) != (Source->Pointer == nullptr)
                || (Other->Pointer && Other->ReadOnly.empty()) || (Source->Pointer && Source->ReadOnly.empty())) return {};
            Plan->Access.ReadOnly.insert(Plan->Access.ReadOnly.end(), Other->ReadOnly.begin(), Other->ReadOnly.end());
            Plan->Access.ReadOnly.insert(Plan->Access.ReadOnly.end(), Source->ReadOnly.begin(), Source->ReadOnly.end());
            FPhysicalContactConstraint P;
            P.Normal = FVector(V.Normal[0], V.Normal[1], V.Normal[2]);
            P.ContactPoint = FVector(V.ContactPoint[0], V.ContactPoint[1], V.ContactPoint[2]);
            P.OtherComponent = Other->Pointer; P.SourceSubBody = Source->Pointer;
            P.PenetrationDepth = V.PenetrationDepth; P.TOI = V.TOI; P.FramesSinceSeen = V.FramesSinceSeen; P.bPersistent = V.Persistent;
            P.PairKey = CommonState::ConstraintPairKey(Source->Pointer ? Source->Pointer->GetUniqueID() : 0,
                Other->Pointer ? Other->Pointer->GetUniqueID() : 0);
            Plan->Constraints.Add(std::move(P));
        }
        const auto Span = [](const void* P, std::size_t N) { return CommonState::FSpan{reinterpret_cast<std::uintptr_t>(P), N}; };
        Plan->Access.Writes = {Span(&C.PhysicalConstraints, sizeof(C.PhysicalConstraints)), Span(&C.SleepState, sizeof(C.SleepState)),
            Span(&C.KinematicQuantizationCache, sizeof(C.KinematicQuantizationCache)), Span(&C.StaticRestingWorld, sizeof(C.StaticRestingWorld)),
            Span(&C.StaticSupportFrameHorizon, sizeof(C.StaticSupportFrameHorizon)), Span(&Plan->Constraints, sizeof(Plan->Constraints)),
            Span(&Plan->Finalized, sizeof(Plan->Finalized)), Span(&Plan->Released, sizeof(Plan->Released))};
        Plan->Access.ReadOnly.push_back(Span(&Plan->Caches, sizeof(Plan->Caches)));
        Plan->Access.ReadOnly.push_back(Span(&Plan->StaticAuthority, sizeof(Plan->StaticAuthority)));
        Plan->Access.ReadOnly.push_back(Span(&Plan->Horizon, sizeof(Plan->Horizon)));
        Plan->Access.ReadOnly.push_back(Span(&Plan->PublishedStableId, sizeof(Plan->PublishedStableId)));
        Plan->Access.ReadOnly.push_back(Span(&Plan->Target, sizeof(Plan->Target)));
        if (!Plan->Constraints.IsEmpty()) Plan->Access.ReadOnly.push_back(
            Span(Plan->Constraints.GetData(), sizeof(FPhysicalContactConstraint) * static_cast<std::size_t>(Plan->Constraints.Num())));
        if (!CommonState::ValidateGlobalAccess({Plan->Access})) return {};
        return Plan; // PARTIAL participant only, never claims complete actor restoration
    }
};
}
