#include "ISolidSweeper.h"
#include "Engine/World.h"
#include "HAL/IConsoleManager.h"
#if WITH_EDITOR && !UE_BUILD_SHIPPING
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/ScopeExit.h"
#include "IAmSpeed/World/Analytic/DirectedLinearSphereContactMaterialization.h"
#include "IAmSpeed/World/Analytic/DirectedLinearSphereCorpusMaterialization.h"
#include "IAmSpeed/World/Analytic/DirectedSphereFacetProposal.h"
#include "IAmSpeed/World/Analytic/C2QualificationOppositeBallProbe.h"
#include "IAmSpeed/World/Collision/PreparedIntervalScene.h"
#include "IAmSpeed/World/Collision/StaticCollisionWorld.h"
#include "IAmSpeed/World/Simulation/FrameFloatingPointContext.h"
#endif
#include "IAmSpeed/World/Collision/PreparedSphereRequest.h"
#include "IAmSpeed/World/Collision/PreparedSphereRsTrajectoryCapture.h"
#include "IAmSpeed/World/Collision/PreparedSweepEvidence.h"
#include "IAmSpeed/World/Simulation/OnePausedCanonicalWork.h"
#include "IAmSpeed/SubBodies/Solid/SolidSubBody.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "IAmSpeed/IAmSpeed.h"
#include "IAmSpeed/World/Analytic/AnalyticWorldQuery.h"
#include "IAmSpeed/World/Analytic/StaticWorldQueryAudit.h"
#include "IAmSpeed/World/Simulation/CcdPoseAudit.h"
#include "IAmSpeed/World/Analytic/AnalyticNumericContext.h"
#include "IAmSpeed/SubBodies/Solid/SphereSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include <stdexcept>

#if WITH_EDITOR && !UE_BUILD_SHIPPING
namespace
{
void ObserveC259DirectedBall(const Speed::IStaticCollisionWorld& Provider, uint64 Frame, uint64 Generation,
    uint64 Body, uint32 OwnerFrame, int32 SubBody, const Speed::Analytic::FWorldQuery& Query, const Speed::Analytic::FWorldHit& OrdinaryHit,
    float Delta, double PhysicalRadius, bool bTryResponse, Speed::Analytic::FWorldHit& Refined)
{
    namespace D = Speed::Analytic::Certified::DirectedLinearSphere;
    namespace C = Speed::Analytic::Certified;
    namespace A = ::AnalyticCertified;
    namespace M = Speed::Analytic::Certified::OrdinaryBallSeam;
    Speed::FScopedFrameFloatingPointContext RestoreLogging(Speed::CaptureFrameFloatingPointContext());
    const TCHAR* Stage = TEXT("query_identity");
    const bool bFused = FParse::Param(FCommandLine::Get(),TEXT("SLC2QualificationFusedPrefix"));
    const bool bDynamic = FParse::Param(FCommandLine::Get(),TEXT("SLC2QualificationDynamicProposal"));
    bool bProposed = false;
    double ProposalMs = 0.;
    uint64 ProposalWork = 0;
    D::FObservation Observation;
    M::FMaterialized Materialized;
    bool bMaterialized = false;
    const TCHAR* MaterializationStage = TEXT("proof");
    uint64 MaterializationWork = 0;
    double MaterializationMs = 0.;
    double LandscapeGap = 0.;
    uint64 LandscapeId = 0, Winner = 0;
    bool bJoined = false;
    {
        // Preserve the owner's complete scalar and SIMD environment; the
        // diagnostic may accrue arithmetic exception flags on its own work.
        Speed::FScopedFrameFloatingPointContext Numeric(Speed::CaptureFrameFloatingPointContext());
        if (Numeric.IsApplied())
        {
            try
            {
                const auto Inspect = [&]
                {
                    const auto Same = [](double AValue, double BValue) { return A::Bits(AValue) == A::Bits(BValue); };
                    const auto SamePoint = [&](const FVector3d& P, const FVector3d& Q)
                    { return Same(P.X,Q.X) && Same(P.Y,Q.Y) && Same(P.Z,Q.Z); };
                    const auto* Opposite = Speed::Analytic::Qualification::ReadOppositeBallProbe();
                    const FVector3d ExpectedStart = Opposite ? Opposite->Start : FVector3d(3600.0417282057524,3870.4679876494242,96.4159845215794);
                    const FVector3d ExpectedEnd = Opposite ? Opposite->End : FVector3d(3600.0417073241642,3870.8816915256616,94.45924000520793);
                    const uint64 ExpectedWinner = Opposite ? Opposite->Winner : 0x82e2ce3273c1c79eull;
                    if (Query.Shape != Speed::Analytic::EQueryShape::Sphere ||
                        !SamePoint(Query.Start, ExpectedStart) ||
                        !SamePoint(Query.End, ExpectedEnd) ||
                        !Same(Query.Radius,93.150100708007812) || !Same(Query.DomainTolerance,1.e-6) ||
                        !Same(Query.InitialOverlapTolerance,.05) ||
                        Query.TraceChannel != 1 || Query.BlockingObjectTypes != (uint64(1) << ECC_WorldStatic) ||
                        Query.bObjectQuery || !Query.bApplyCollisionFilter || !Query.bAuthorityOnly ||
                        !Query.bIncludeCompactPatches || Query.bIncludeTriangles || Query.bExcludeAuthorityEligible ||
                        Query.RequiredSourceId || Query.RequiredSurfaceId || Query.RequiredCanonicalGroupId ||
                        !Query.ReferenceNormal.IsZero() || Query.MinimumReferenceNormalDot != -1.)
                        return;

                    Stage = TEXT("packet");
                    const Speed::Analytic::FBoundedPlane* Landscape = nullptr;
                    const auto* Packet = Provider.TryReadOrdinaryC2DiagnosticScene(Landscape);
                    if (!Packet || !Packet->Nominal || Packet->SceneGeneration != Generation || !Landscape ||
                        !Packet->FinalSourceHash || Landscape->PrimitiveId != Packet->ExternalLandscapePrimitive)
                        return;
                    constexpr uint64 StadiumSource = 0x11af315a281440a9ull;
                    const auto& Nominal = *Packet->Nominal;
                    const auto Owners = Nominal.GetOwners();
                    if (!Nominal.GetUnion() || Owners.Num() != 2992 || Nominal.GetUnion()->Num() != Owners.Num())
                        return;
                    const auto Passes = [&](uint32 ObjectType, uint64 Channels)
                    {
                        return ObjectType < 64 && (Query.BlockingObjectTypes & (uint64(1) << ObjectType)) &&
                            (Channels & (uint64(1) << Query.TraceChannel));
                    };
                    D::FProposal Proposal;
                    for (int32 I = 0; I < Owners.Num(); ++I)
                    {
                        const auto& Owner = Owners[I];
                        if (Owner.SourceId != StadiumSource || !Owner.QueryEnabled ||
                            !Passes(Owner.ObjectType,Owner.BlockingChannels))
                            return;
                        if (Owner.PrimitiveId == ExpectedWinner)
                        {
                            if (Proposal.ProviderIndex != INDEX_NONE || Owner.FromPlane)
                                return;
                            Proposal.ProviderIndex = I;
                        }
                    }
                    if (Proposal.ProviderIndex == INDEX_NONE)
                        return;

                    Stage = TEXT("landscape");
                    LandscapeId = Landscape->PrimitiveId;
                    if (!Landscape->SourceId || Landscape->SourceId == StadiumSource ||
                        !Landscape->bAuthorityEligible || !Landscape->bQueryCollisionEnabled ||
                        Landscape->bRequiresCompactOptIn || !Passes(Landscape->ObjectType,Landscape->BlockingChannels) ||
                        Landscape->Normal != FVector3d::UpVector ||
                        Landscape->AxisU.Z != 0. || Landscape->AxisV.Z != 0. ||
                        !A::Finite(Landscape->Origin.Z))
                        return;
                    // The entire finite plane lies in this horizontal plane.
                    // Prove separation from the stronger infinite half-space
                    // for both endpoints and hence for the entire linear chord.
                    const auto Shell = A::Add(A::Add(A::Interval::Exact(Query.Radius),
                        A::Interval::Exact(Query.DomainTolerance)), A::Interval::Exact(Query.InitialOverlapTolerance));
                    const auto Height = A::Subtract(A::Interval::Exact(FMath::Min(Query.Start.Z, Query.End.Z)),
                        A::Interval::Exact(Landscape->Origin.Z));
                    const auto Gap = A::Subtract(Height, Shell);
                    if (!A::Valid(Gap) || Gap.Lower <= 0.)
                        return;
                    LandscapeGap = Gap.Lower;

                    Stage = TEXT("kernel");
                    Proposal.UV = FVector2d(193815. / 268435456.,67108877. / 134217728.);
                    Proposal.Lower = 67108863. / 134217728.;
                    Proposal.Upper = 134217729. / 268435456.;
                    C::FDistanceBudget Budget{262144};
                    const D::FQuery DirectedQuery{Query.Start,Query.End,Query.Radius,Query.DomainTolerance,Query.InitialOverlapTolerance};
                    if (bDynamic)
                    {
                        Stage = TEXT("dynamic_proposal");
                        const double ProposalStart = FPlatformTime::Seconds();
                        bProposed = C::DirectedSphereFacetProposal::TryPropose(Nominal,DirectedQuery,OrdinaryHit,Budget,Proposal);
                        ProposalMs = (FPlatformTime::Seconds() - ProposalStart) * 1000.;
                        ProposalWork = Budget.Used;
                        if (!bProposed || !Owners.IsValidIndex(Proposal.ProviderIndex) ||
                            Owners[Proposal.ProviderIndex].PrimitiveId != ExpectedWinner)
                            return;
                        Stage = TEXT("kernel");
                    }
                    D::FLimits Limits;
                    Limits.bFuseMonotonePrefix = bFused;
                    Limits.bRetainLeafChart = FParse::Param(FCommandLine::Get(),TEXT("SLC2QualificationRetainedChart"));
                    Limits.bReuseQuinticEnclosureSplitStorage = FParse::Param(FCommandLine::Get(),TEXT("SLC2QualificationReuseQuintic"));
                    Limits.bCacheEnclosureSupports = FParse::Param(FCommandLine::Get(),TEXT("SLC2QualificationCachedEnclosure"));
                    Limits.bTryDirectedInitialShell = FParse::Param(FCommandLine::Get(),TEXT("SLC2QualificationDirectedShell"));
                    Observation = D::Observe(Nominal.GetUnion(),DirectedQuery,Proposal,Limits,Budget);
                    if (Observation.Status == D::EStatus::FirstContactEnclosed &&
                        Owners.IsValidIndex(Observation.ProviderIndex))
                    {
                        Winner = Owners[Observation.ProviderIndex].PrimitiveId;
                        bJoined = Winner != 0;
                        Stage = bJoined ? TEXT("enclosed") : TEXT("kernel_identity");
                        if (bTryResponse && bJoined)
                        {
                            MaterializationStage = TEXT("scalar");
                            const auto Start = FPlatformTime::Seconds();
                            const uint64 WorkBefore = Budget.Used;
                            if (bDynamic)
                            {
                                C::OrdinarySphereCorpus::FMaterialized DynamicMaterialized;
                                bMaterialized = C::OrdinarySphereCorpus::Private::TryMaterialize(Nominal,DirectedQuery,
                                    Proposal,Observation,Delta,PhysicalRadius,nullptr,Budget,DynamicMaterialized,MaterializationStage);
                                if (bMaterialized)
                                {
                                    Materialized.Hit = DynamicMaterialized.Hit;
                                    Materialized.Scalar = DynamicMaterialized.Scalar;
                                    Materialized.TOIBits = DynamicMaterialized.TOIBits;
                                    Materialized.QueryPointErrorCm = DynamicMaterialized.QueryPointErrorCm;
                                }
                            }
                            else
                                bMaterialized = M::TryMaterialize(Nominal,DirectedQuery,Proposal,Observation,Delta,
                                    PhysicalRadius,nullptr,Budget,Materialized,MaterializationStage);
                            MaterializationWork = Budget.Used - WorkBefore;
                            MaterializationMs = (FPlatformTime::Seconds() - Start) * 1000.;
                            if (bMaterialized && (!OrdinaryHit.bHit || OrdinaryHit.bStartPenetrating ||
                                OrdinaryHit.SourceId != Materialized.Hit.SourceId ||
                                OrdinaryHit.SurfaceId != Materialized.Hit.SurfaceId ||
                                OrdinaryHit.MaterialId != Materialized.Hit.MaterialId ||
                                OrdinaryHit.CanonicalGroupId != Materialized.Hit.CanonicalGroupId ||
                                OrdinaryHit.Time >= Observation.Lower))
                            {
                                MaterializationStage = TEXT("ordinary_identity");
                                bMaterialized = false;
                            }
                            if (bMaterialized)
                                MaterializationStage = TEXT("admitted_exact_witness");
                        }
                    }
                };
                Inspect();
            }
            catch (...)
            {
                Stage = TEXT("diagnostic_exception");
                bJoined = false;
            }
        }
        else
            Stage = TEXT("numeric_capture");

        if (!Numeric.Restore())
        {
            Stage = TEXT("numeric_restore");
            bJoined = false;
        }
    }
    if (bTryResponse)
    {
        bMaterialized &= bJoined;
        if (bMaterialized)
            Refined = Materialized.Hit;
        UE_LOG(SpeedPhysicsLog,Display,TEXT("[C2BallSeamMaterialization] Frame=%llu Body=%llu SubBody=%d Generation=%llu "
            "Stage=%s Applied=%d Primitive=%llu TOIBits=%08x Work=%llu Ms=%.9g PhysicalRadius=%.17g "
            "SampleError=%.17g SurfaceError=%.17g CenterError=%.17g ResponseError=%.17g QueryError=%.17g StoredNormalError=%.17g ConsumedNormalError=%.17g"),
            Frame,Body,SubBody,Generation,MaterializationStage,int32(bMaterialized),Materialized.Hit.PrimitiveId,Materialized.TOIBits,
            MaterializationWork,MaterializationMs,PhysicalRadius,Materialized.Scalar.SampleEvaluationErrorCm,
            Materialized.Scalar.SurfaceErrorCm,Materialized.Scalar.LocationErrorCm,Materialized.Scalar.ResponseErrorCm,
            Materialized.QueryPointErrorCm,Materialized.Scalar.StoredNormalErrorL1,Materialized.Scalar.ConsumedNormalErrorL1);
    }
    UE_LOG(SpeedPhysicsLog,Display,TEXT("[C2BallSeamDirectedObservation] Frame=%llu Body=%llu OwnerFrame=%u SubBody=%d Generation=%llu "
        "Stage=%s Joined=%d NoSubstitution=%d KernelStage=%u Landscape=%016llx LandscapeGap=%.17g Winner=%016llx "
        "OrdinaryHit=%d OrdinaryTime=%.17g Lower=%.17g Upper=%.17g Candidates=%d Excluded=%d "
        "Work=%llu PrefixNodes=%llu EnclosureNodes=%llu SelectMs=%.9g ShellMs=%.9g PrefixMs=%.9g EnclosureMs=%.9g "
        "Fused=%d EndpointLeafTests=%llu EndpointLeafWork=%llu EndpointLeafMs=%.9g "
        "DirectedShell=%d DirectedShellClosed=%d ShellNodes=%llu ShellWork=%llu DirectedShellMs=%.9g SupportTests=%llu SupportControls=%llu SupportEarlyRejects=%llu "
        "ReuseSplits=%llu SplitSlots=%llu Dynamic=%d Proposed=%d ProposalMs=%.9g ProposalWork=%llu"),
        Frame,Body,OwnerFrame,SubBody,Generation,Stage,int32(bJoined),int32(!bMaterialized),uint32(Observation.Stage),LandscapeId,LandscapeGap,
        bJoined?Winner:0ull,int32(OrdinaryHit.bHit),OrdinaryHit.Time,Observation.Lower,Observation.Upper,
        Observation.CandidateCount,Observation.ExcludedCount,Observation.Work,Observation.PrefixNodes,
        Observation.EnclosureNodes,Observation.SelectMs,Observation.ShellMs,Observation.PrefixMs,Observation.EnclosureMs,
        int32(bFused),Observation.EndpointLeafTests,Observation.EndpointLeafWork,Observation.EndpointLeafMs,
        int32(Observation.bDirectedShellAttempted),int32(Observation.bDirectedShellClosed),
        Observation.DirectedShellNodes,Observation.DirectedShellWork,Observation.DirectedShellMs,
        Observation.EnclosureSupportTests,Observation.EnclosureSupportControls,Observation.EnclosureSupportEarlyRejects,
        Observation.EnclosureReusableSplits,Observation.EnclosureSplitStorageSlots,int32(bDynamic),int32(bProposed),ProposalMs,ProposalWork);
    if (bJoined)
        for (int32 Axis = 0; Axis < 3; ++Axis)
            UE_LOG(SpeedPhysicsLog,Display,TEXT("[C2BallSeamDirectedAxis] Frame=%llu Body=%llu Generation=%llu Axis=%d "
                "Point=[%.17g,%.17g] Radial=[%.17g,%.17g] OrdinaryPoint=%.17g OrdinaryNormal=%.17g"),
                Frame,Body,Generation,Axis,Observation.Point.Axis[Axis].Lower,Observation.Point.Axis[Axis].Upper,
                Observation.RadialNormal.Axis[Axis].Lower,Observation.RadialNormal.Axis[Axis].Upper,
                OrdinaryHit.Point[Axis],OrdinaryHit.Normal[Axis]);
}

// Private editor qualification on a joined, closed C2 packet. No World query
// is repeated. A failed optional proof leaves the already selected hit intact.
void TrySelectOrdinaryC2Sphere(const Speed::IStaticCollisionWorld& Provider, uint64 Frame, uint64 Generation,
    uint64 Body, uint32 OwnerFrame, const Speed::Analytic::FWorldQuery& Query, float Delta,
    double PhysicalRadius, Speed::Analytic::FWorldHit& Ordinary)
{
    namespace C = Speed::Analytic::Certified;
    namespace D = C::DirectedLinearSphere;
    namespace A = ::AnalyticCertified;
    namespace M = C::OrdinarySphereCorpus;
    const TCHAR* Stage = TEXT("query");
    bool bRefined = false;
    uint64 Work = 0, ProposalWork = 0;
    D::FProposal Proposal;
    const double Began = FPlatformTime::Seconds();
    double LandscapeGap = 0.;
    const double FastTime = Ordinary.Time;
    const uint64 FastSource = Ordinary.SourceId, FastSurface = Ordinary.SurfaceId;
    const uint64 FastFeature = Ordinary.FeatureId, FastPrimitive = Ordinary.PrimitiveId;
    const uint64 FastGroup = Ordinary.CanonicalGroupId;
    D::FObservation Observation;
    M::FMaterialized Result;
    Speed::FScopedFrameFloatingPointContext Numeric(Speed::CaptureFrameFloatingPointContext());
    if (Numeric.IsApplied())
    {
        try
        {
            const auto Try = [&]
            {
                if (Query.Shape != Speed::Analytic::EQueryShape::Sphere || Query.TraceChannel >= 64 ||
                    Query.bObjectQuery || !Query.bApplyCollisionFilter || !Query.bAuthorityOnly ||
                    !Query.bIncludeCompactPatches || Query.bIncludeTriangles || Query.bExcludeAuthorityEligible ||
                    Query.RequiredSourceId || Query.RequiredSurfaceId || Query.RequiredCanonicalGroupId ||
                    Query.bAllowEstablishedFaceContactAtBoundary ||
                    !Query.ReferenceNormal.IsZero() || Query.MinimumReferenceNormalDot != -1. ||
                    !A::Finite(Delta) || Delta <= 0.f || !A::Finite(PhysicalRadius) ||
                    PhysicalRadius <= 0. || !A::Finite(Query.Radius) || Query.Radius < PhysicalRadius)
                    return false;
                Stage = TEXT("miss");
                if (!Ordinary.bHit) return false;
                Stage = TEXT("event_type");
                if (Ordinary.bStartPenetrating || !(0. < FastTime && FastTime < 1.)) return false;
                Stage = TEXT("fast_exact_or_non_tensor");
                if (!Ordinary.bSurfaceNormalMayVary || !A::Finite(Ordinary.GeometricErrorBoundCm) ||
                    Ordinary.GeometricErrorBoundCm <= 0.) return false;

                Stage = TEXT("packet");
                const Speed::Analytic::FBoundedPlane* Landscape = nullptr;
                const auto* Packet = Provider.TryReadOrdinaryC2DiagnosticScene(Landscape);
                if (!Packet || !Packet->Nominal || Packet->SceneGeneration != Generation ||
                    !Packet->FinalSourceHash || !Landscape ||
                    Landscape->PrimitiveId != Packet->ExternalLandscapePrimitive)
                    return false;
                // Only the existing private attachment admits the complete
                // authored C2 union plus one external Landscape and zero tori.
                const auto& Scene = *Packet->Nominal;
                const auto Owners = Scene.GetOwners();
                if (!Scene.GetUnion() || Owners.IsEmpty() || Scene.GetUnion()->Num() != Owners.Num())
                    return false;
                const auto Passes = [&](uint32 Type, uint64 Channels)
                {
                    return Type < 64 && (Query.BlockingObjectTypes & (uint64(1) << Type)) &&
                        (Channels & (uint64(1) << Query.TraceChannel));
                };
                C::FDistanceBudget Budget{262144};
                ON_SCOPE_EXIT { Work = Budget.Used; };
                Stage = TEXT("roster_filter");
                for (const auto& Owner : Owners)
                    if (!Budget.Consume() || !Owner.SourceId || !Owner.QueryEnabled ||
                        !Passes(Owner.ObjectType, Owner.BlockingChannels))
                        return false;

                Stage = TEXT("landscape");
                if (!Landscape->SourceId || !Landscape->bAuthorityEligible ||
                    !Landscape->bQueryCollisionEnabled || Landscape->bRequiresCompactOptIn ||
                    !Passes(Landscape->ObjectType, Landscape->BlockingChannels) ||
                    Landscape->Normal != FVector3d::UpVector ||
                    Landscape->AxisU.Z != 0. || Landscape->AxisV.Z != 0. ||
                    !A::Finite(Landscape->Origin.Z))
                    return false;
                const auto Shell = A::Add(A::Add(A::Interval::Exact(Query.Radius),
                    A::Interval::Exact(Query.DomainTolerance)), A::Interval::Exact(Query.InitialOverlapTolerance));
                const auto Gap = A::Subtract(A::Subtract(A::Interval::Exact(FMath::Min(Query.Start.Z, Query.End.Z)),
                    A::Interval::Exact(Landscape->Origin.Z)), Shell);
                if (!A::Valid(Gap) || Gap.Lower <= 0.)
                    return false;

                LandscapeGap = Gap.Lower;
                Stage = TEXT("proposal_or_boundary");
                const D::FQuery Directed{Query.Start, Query.End, Query.Radius,
                    Query.DomainTolerance, Query.InitialOverlapTolerance};
                const uint64 ProposalBefore = Budget.Used;
                const bool bProposed = C::DirectedSphereFacetProposal::TryPropose(Scene, Directed, Ordinary, Budget, Proposal, 4);
                ProposalWork = Budget.Used - ProposalBefore;
                if (!bProposed ||
                    !Owners.IsValidIndex(Proposal.ProviderIndex))
                    return false;
                const auto& Owner = Owners[Proposal.ProviderIndex];
                if (Owner.FromPlane || Owner.SourceId != Ordinary.SourceId || Owner.SurfaceId != Ordinary.SurfaceId ||
                    Owner.FeatureId != Ordinary.FeatureId || Owner.CanonicalGroupId != Ordinary.CanonicalGroupId ||
                    Owner.MaterialId != Ordinary.MaterialId)
                    return false;

                D::FLimits Limits;
                Limits.bFuseMonotonePrefix = Limits.bRetainLeafChart = true;
                Limits.bReuseQuinticEnclosureSplitStorage = Limits.bCacheEnclosureSupports = true;
                Limits.bTryDirectedInitialShell = Limits.bShareQuinticHalfScales = true;
                Limits.bFuseReachableMonotonePrefix = Limits.bReduceQuinticSplitScores = true;
                Stage = TEXT("observation");
                Observation = D::Observe(Scene.GetUnion(), Directed, Proposal, Limits, Budget);
                if (Observation.Status != D::EStatus::FirstContactEnclosed ||
                    Observation.ProviderIndex != Proposal.ProviderIndex)
                    return false;
                Stage = TEXT("same_interval");
                if (!(FastTime < Observation.Lower || FastTime > Observation.Upper))
                    return false;
                // No product margins are inferred from an editor fixture.
                // Null margins remain limited to this private qualification.
                if (!M::Private::TryMaterialize(Scene, Directed, Proposal, Observation, Delta,
                    PhysicalRadius, nullptr, Budget, Result, Stage))
                    return false;
                Stage = TEXT("materialized_owner");
                const auto& Hit = Result.Hit;
                return Hit.bHit && !Hit.bStartPenetrating && Hit.SourceId == Owner.SourceId &&
                    Hit.SurfaceId == Owner.SurfaceId && Hit.FeatureId == Owner.FeatureId &&
                    Hit.PrimitiveId == Owner.PrimitiveId && Hit.CanonicalGroupId == Owner.CanonicalGroupId &&
                    Hit.MaterialId == Owner.MaterialId;
            };
            bRefined = Try();
        }
        catch (...)
        {
            Stage = TEXT("optional_exception");
        }
    }
    else
        Stage = TEXT("numeric_capture");
    if (!Numeric.Restore())
    {
        bRefined = false;
        Stage = TEXT("numeric_restore");
    }
    if (bRefined)
    {
        Ordinary = Result.Hit;
        Stage = TEXT("certified_same_owner_divergence");
    }
    UE_LOG(SpeedPhysicsLog, Display, TEXT("[C2OrdinaryQualificationSelector] Frame=%llu Body=%llu OwnerFrame=%u Generation=%llu Family=Sphere "
        "Stage=%s Applied=%d FastTime=%.17g Lower=%.17g Upper=%.17g Work=%llu Ms=%.9g KernelStage=%u "
        "FastSource=%016llx FastSurface=%016llx FastFeature=%016llx FastPrimitive=%016llx FastGroup=%016llx "
        "PublishedPrimitive=%016llx TOIBits=%08x LandscapeGap=%.17g ProposalGrid=4 ProposalWork=%llu "
        "ProposalUV=(%.17g,%.17g) ProposalL=%.17g ProposalU=%.17g PrefixNodes=%llu EnclosureNodes=%llu "
        "ReachableAccepted=%llu ReachableAttempts=%llu ReducedSplitScores=%llu ShellMs=%.9g PrefixMs=%.9g EnclosureMs=%.9g"),
        Frame, Body, OwnerFrame, Generation, Stage, int32(bRefined), FastTime, Observation.Lower, Observation.Upper,
        Work, (FPlatformTime::Seconds() - Began) * 1000., uint32(Observation.Stage),
        FastSource, FastSurface, FastFeature, FastPrimitive, FastGroup, Ordinary.PrimitiveId,
        Result.TOIBits, LandscapeGap, ProposalWork, Proposal.UV.X, Proposal.UV.Y, Proposal.Lower, Proposal.Upper,
        Observation.PrefixNodes, Observation.EnclosureNodes, Observation.ReachableMonotoneAccepted,
        Observation.ReachableMonotoneAttempts, Observation.EnclosureReducedSplitScores, Observation.ShellMs, Observation.PrefixMs, Observation.EnclosureMs);
    if (Frame == 0 || bRefined)
        UE_LOG(SpeedPhysicsLog, Display, TEXT("[C2OrdinarySelectorChord] Frame=%llu Body=%llu OwnerFrame=%u Generation=%llu "
            "Radius=%.17g H=%.17g Start=(%.17g,%.17g,%.17g) End=(%.17g,%.17g,%.17g)"),
            Frame, Body, OwnerFrame, Generation, Query.Radius, double(Delta),
            Query.Start.X, Query.Start.Y, Query.Start.Z, Query.End.X, Query.End.Y, Query.End.Z);
    if (bRefined)
        for (int32 Axis = 0; Axis < 3; ++Axis)
            UE_LOG(SpeedPhysicsLog, Display, TEXT("[C2OrdinarySelectorAxis] Frame=%llu Body=%llu Generation=%llu "
                "Axis=%d Point=[%.17g,%.17g] Radial=[%.17g,%.17g]"), Frame, Body, Generation, Axis,
                Observation.Point.Axis[Axis].Lower, Observation.Point.Axis[Axis].Upper,
                Observation.RadialNormal.Axis[Axis].Lower, Observation.RadialNormal.Axis[Axis].Upper);
}
}
#endif

bool ISolidSweeper::InternalSweep(UWorld* World, const FVector& Start, const FVector& End, SHitResult& OutHit, const float& delta)
{
    // Keep missing-World strict calls from bypassing the authority bridge via
    // the old early no-hit return. Legacy/diagnostic absence stays unchanged.
    if(!World && Speed::Analytic::FStaticWorldQueryAudit::IsSurfaceAnalyticBackend())
        throw std::runtime_error("SurfaceAnalytic solid sweep has no World");
    const USolidSubBody* Solid=GetSolidSweepBody();
    const ISpeedComponent* Owner=Solid ? Solid->GetParentComponent() : nullptr;
#if WITH_DEV_AUTOMATION_TESTS
    const bool bCapture=Speed::Testing::IsCcdPrimitiveCaptureEnabledForCurrentThread(Owner);
    if(!World && !bCapture) return false;
#else
    if(!World) return false;
#endif
    // Freeze the same shape used by the query and its observer. Getter has no
    // reason to be called repeatedly during one fixed sweep.
    const FCollisionShape QueryShape=GetWorldStaticCollisionShape();
#if WITH_DEV_AUTOMATION_TESTS
    // Copy before any query callback. This observer never handles a sweep,
    // changes a hit, permits a missing World or grants collision authority.
    if(bCapture)
    {
        using namespace Speed::Testing;
        const auto BodyState=GetKinematicState();const auto OwnerState=Owner->GetKinematicState();
        const auto& Shape=QueryShape;FCcdPrimitiveCapture Sample;
        Sample.OwnerIdentity=Owner;Sample.BodyIdentity=Solid;
        const auto Stamp=NextCcdPrimitiveCaptureStampForCurrentThread();
        Sample.Serial=Stamp.Serial;Sample.RunGeneration=Stamp.RunGeneration;Sample.OwnerFrame=Owner->NumFrame();
        Sample.Owner={OwnerState.Location,OwnerState.Velocity,OwnerState.Acceleration};
        Sample.SubBody={BodyState.Location,BodyState.Velocity,BodyState.Acceleration};
        Sample.LocalBodyOffset=Solid->GetLocalOffset();Sample.LocalCenterOfMass=Owner->GetPhysCenterOfMassLocal();
        Sample.IsSphereSubBody=Cast<USphereSubBody>(Solid)!=nullptr && Shape.IsSphere();
        Sample.IsWheelSubBody=Cast<USWheelSubBody>(Solid)!=nullptr;
        Sample.EffectiveSphereRadius=Shape.IsSphere()?double(Shape.GetSphereRadius()):0.;
        Sample.QueryStart=Start;Sample.QueryEnd=End;
        Sample.Delta=delta;Sample.WorldWasObserved=true;Sample.HasWorld=World!=nullptr;
        const auto Numeric=Speed::Analytic::InspectCurrentAnalyticNumericContext(Speed::Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        Sample.NumericControlFingerprint=Numeric.ControlFingerprint();Sample.NumericContextSupported=Numeric.Status==Speed::Analytic::EAnalyticNumericStatus::Complete;
        ObserveCcdPrimitiveForCurrentThread(ECcdPoseAuditStage::SphereSweepCapture,Sample);
        const auto CurrentOwner=Owner->GetKinematicState();const auto CurrentBody=GetKinematicState();
        const FCcdLinearStateValue OwnerPVA{CurrentOwner.Location,CurrentOwner.Velocity,CurrentOwner.Acceleration},BodyPVA{CurrentBody.Location,CurrentBody.Velocity,CurrentBody.Acceleration};
        const auto CurrentShape=GetWorldStaticCollisionShape();const auto CurrentOffset=Solid->GetLocalOffset(),CurrentCOM=Owner->GetPhysCenterOfMassLocal();
        const auto CurrentNumeric=Speed::Analytic::InspectCurrentAnalyticNumericContext(Speed::Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!CcdLinearValuesHaveEqualBits(Sample.Owner,OwnerPVA)||!CcdLinearValuesHaveEqualBits(Sample.SubBody,BodyPVA)||
            Sample.OwnerFrame!=Owner->NumFrame()||FMemory::Memcmp(&Sample.LocalBodyOffset,&CurrentOffset,sizeof(CurrentOffset))||
            FMemory::Memcmp(&Sample.LocalCenterOfMass,&CurrentCOM,sizeof(CurrentCOM))||
            Shape.IsSphere()!=CurrentShape.IsSphere()||(Shape.IsSphere()&&Shape.GetSphereRadius()!=CurrentShape.GetSphereRadius())||
            Numeric.ControlFingerprint()!=CurrentNumeric.ControlFingerprint()) MarkCcdPrimitiveCaptureViolationForCurrentThread();
    }
#endif
    if (!World) return false;
    if(Speed::PreparedExecution::Requested())
    {
        Speed::OnePausedWork::FCanonicalFrameWindowPtr window;
        if(!Speed::OnePausedWork::ReadExecutionWindow(window)||!window)
            throw std::runtime_error("Prepared solid query has no current canonical window");
        if(window->GetPurpose()==Speed::OnePausedWork::EPurpose::FullFrame)
        {
            const auto trace=BuildTraceParams();
            return Speed::PreparedExecution::FComponentSweepScope::EvaluateOrdinaryNativeGround(
                World,this,Solid,Start,End,QueryShape,trace,delta,OutHit);
        }
        Speed::PreparedExecution::FNativeSphereRequestPtr request;
        if(!Speed::PreparedExecution::CaptureSphereCCD(World,Solid,Start,End,QueryShape,delta,request))
            throw std::runtime_error("Prepared sphere query lacks its exact native context or supported shape");
        if(Speed::PreparedExecution::FSphereMemberSweepScope::IsTerminalRsCapture())
            Speed::PreparedExecution::FSphereMemberSweepScope::CaptureRsAndStop(MoveTemp(request));
        // A complete rounded hull exclusion is recorded in the live member,
        // then sealed by its virtual-margin reread. Any candidate remains a
        // terminal Unknown; no triangle/chord fallback or owner promotion.
        if(!Speed::PreparedExecution::FSphereMemberSweepScope::RecordGroundSweep(MoveTemp(request),OutHit))
            throw std::runtime_error("Prepared ground sweep result unavailable");
        return OutHit.bHit;
    }
    const Speed::FKinematicState& KS = GetKinematicState();
    Speed::Analytic::FTorusSweepTrajectory Trajectory;
    const bool bTorusTrajectory=Owner && Owner->GetStaticCollisionWorldForFrame();
    if (bTorusTrajectory)
    {
        Trajectory.Owner=Owner->GetKinematicState();Trajectory.DeltaSeconds=delta;
        // Current physical offset includes centre of mass and wheel suspension.
        Trajectory.LocalOffset=Trajectory.Owner.Rotation.UnrotateVector(KS.Location-Trajectory.Owner.Location);
        Trajectory.LocalRotation=Trajectory.Owner.Rotation.Inverse()*KS.Rotation;
    }

	FHitResult Hit;
	bool bHit = false;
	Speed::Analytic::FWorldHit AnalyticHit;
	const uint8 AuthorityTraceChannel = static_cast<uint8>(GetCollisionChannel());
	const auto& AuthorityResponse = GetResponseParams();
	const bool bUsedAnalyticAuthority =
		Speed::Analytic::FStaticWorldQueryAudit::TryCompactAuthoritySingle(
		World,
		Start, End, KS.Rotation, QueryShape,
		AuthorityTraceChannel, AuthorityResponse, Hit, bHit,
		&AnalyticHit,0,0,0,FVector::ZeroVector,-1.f,false,bTorusTrajectory ? &Trajectory : nullptr);
	if (!bUsedAnalyticAuthority)
	{
		const FCollisionQueryParams Params = BuildTraceParams();
		Speed::Analytic::FStaticWorldQueryAudit::RecordLegacySweep();
		bHit = World->SweepSingleByChannel(
			Hit, Start, End, KS.Rotation, GetCollisionChannel(),
			QueryShape, Params, GetResponseParams());
	}
	else if (bHit && QueryShape.IsBox() &&
		Speed::Analytic::FStaticWorldQueryAudit::IsAuthorityChaosShadowEnabled())
	{
		// Explicit one-shot diagnostic. This UWorld query never changes authority
		// and is counted as a legacy sweep so strict zero-UWorld evidence cannot
		// accidentally include a run made with this mode enabled.
		static bool bFirstAuthorityBoxHitReplayed = false;
		if (!bFirstAuthorityBoxHitReplayed)
		{
			bFirstAuthorityBoxHitReplayed = true;
			FHitResult ChaosShadowHit;
			const FCollisionQueryParams Params = BuildTraceParams();
			Speed::Analytic::FStaticWorldQueryAudit::RecordLegacySweep();
			const bool bChaosShadowHit = World->SweepSingleByChannel(
				ChaosShadowHit, Start, End, KS.Rotation, GetCollisionChannel(),
				QueryShape, Params, GetResponseParams());
			UE_LOG(SpeedPhysicsLog, Display,
				TEXT("[AnalyticAuthorityChaosBoxShadow] Start=%s End=%s Rotation=%s HalfExtent=%s AnalyticTime=%.17g AnalyticPoint=%s AnalyticNormal=%s AnalyticStartPenetrating=%d AnalyticPenetrationDepth=%.17g AnalyticSurface=%016llX AnalyticFeature=%016llX AnalyticPrimitive=%016llX ChaosHit=%d ChaosTime=%.9g ChaosPoint=%s ChaosNormal=%s ChaosStartPenetrating=%d ChaosPenetrationDepth=%.9g"),
				*Start.ToString(), *End.ToString(), *KS.Rotation.ToString(),
				*QueryShape.GetBox().ToString(),
				AnalyticHit.Time, *AnalyticHit.Point.ToString(), *AnalyticHit.Normal.ToString(),
				AnalyticHit.bStartPenetrating ? 1 : 0, AnalyticHit.PenetrationDepth,
				AnalyticHit.SurfaceId, AnalyticHit.FeatureId, AnalyticHit.PrimitiveId,
				bChaosShadowHit ? 1 : 0, ChaosShadowHit.Time,
				*ChaosShadowHit.ImpactPoint.ToString(), *ChaosShadowHit.ImpactNormal.ToString(),
				ChaosShadowHit.bStartPenetrating ? 1 : 0, ChaosShadowHit.PenetrationDepth);
		}
	}
	Speed::Analytic::FStaticWorldQueryAudit::RecordSingle(
		Speed::Analytic::EStaticQuerySite::SolidSweep,
		Start, End, KS.Rotation, QueryShape,
		static_cast<uint8>(GetCollisionChannel()), GetResponseParams(), bHit, Hit);

#if WITH_EDITOR && !UE_BUILD_SHIPPING
    // Observe the returned status even on a miss. Nothing here calls a query.
    static const bool bObserveBallSeam =
        FParse::Param(FCommandLine::Get(),TEXT("SLC2QualificationBallSeam")) &&
        FParse::Param(FCommandLine::Get(),TEXT("SLFreeplayC2DirectedSceneWitness"));
    static const bool bQualificationSelector =
        FParse::Param(FCommandLine::Get(),TEXT("SLC2QualificationOrdinarySelector")) &&
        FParse::Param(FCommandLine::Get(),TEXT("SLFreeplayC2DirectedSceneWitness"));
    static const bool bPrivateSelector =
        FParse::Param(FCommandLine::Get(),TEXT("SLPrivateC2RefineDivergentContacts")) &&
        FParse::Param(FCommandLine::Get(),TEXT("SLPrivateOrdinaryC2RuntimeIndex"));
    static const bool bPrivateInputTrace = FParse::Param(FCommandLine::Get(),TEXT("SLPrivateC2BallInputReplay"));
    if ((bQualificationSelector || bPrivateSelector) && bUsedAnalyticAuthority && Owner && Solid && QueryShape.IsSphere() &&
        Solid->GetSubBodyType() == USSubBody::ESubBodyType::Sphere &&
        Solid->GetClass()->GetFName() == FName(TEXT("BallSubBody")) &&
        Owner->GetSubBodies().Num() == 1 && Owner->GetSubBodies()[0] == Solid)
    {
        uint64 Frame = 0, Generation = 0;
        if (Speed::Analytic::FStaticWorldQueryAudit::TryObserveStrictFrame(World, Frame, Generation))
        {
            const auto* Provider = Speed::Analytic::FStaticWorldQueryAudit::ReadStrictFrameProviderForObservation(
                World, Frame, Generation);
            if (const auto* Sphere = Cast<USphereSubBody>(Solid); Provider && Sphere)
            {
                const auto Query = Speed::Analytic::FStaticWorldQueryAudit::MakeStrictSingleQuery1075(
                    Start, End, KS.Rotation, QueryShape, AuthorityTraceChannel, AuthorityResponse);
                TrySelectOrdinaryC2Sphere(*Provider, Frame, Generation, Owner->GetPublishedSimulationStableId(),
                    Owner->NumFrame(), Query, delta, double(Sphere->GetRadius()), AnalyticHit);
            }
        }
    }
    uint64 SeamFrame = 0, SeamGeneration = 0;
    if ((bObserveBallSeam || bPrivateInputTrace) && Owner && Solid && QueryShape.IsSphere() &&
        Solid->GetSubBodyType() == USSubBody::ESubBodyType::Sphere &&
        Speed::Analytic::FStaticWorldQueryAudit::TryObserveStrictFrame(World,SeamFrame,SeamGeneration) &&
        SeamFrame == 0)
    {
        UE_LOG(SpeedPhysicsLog,Display,TEXT("[C2BallSeamLiveSweep] Frame=%llu OwnerFrame=%u Body=%llu SubBody=%d SceneGeneration=%llu "
            "Strict=%d Hit=%d Radius=%.17g H=%.17g Start=(%.17g,%.17g,%.17g) End=(%.17g,%.17g,%.17g) "
            "V=(%.17g,%.17g,%.17g) A=(%.17g,%.17g,%.17g) W=(%.17g,%.17g,%.17g)"),
            SeamFrame,Owner->NumFrame(),Owner->GetPublishedSimulationStableId(),Owner->GetSubBodies().IndexOfByKey(Solid),
            SeamGeneration,int32(bUsedAnalyticAuthority),int32(bHit),double(QueryShape.GetSphereRadius()),double(delta),
            Start.X,Start.Y,Start.Z,End.X,End.Y,End.Z,KS.Velocity.X,KS.Velocity.Y,KS.Velocity.Z,
            KS.Acceleration.X,KS.Acceleration.Y,KS.Acceleration.Z,
            KS.AngularVelocity.X,KS.AngularVelocity.Y,KS.AngularVelocity.Z);
        static thread_local uint64 ObservedGeneration = 0;
        if (bObserveBallSeam && !bQualificationSelector && bUsedAnalyticAuthority && SeamGeneration != ObservedGeneration &&
            Owner->NumFrame() == 1 && Owner->GetPublishedSimulationStableId() &&
            Owner->GetSubBodies().Num() == 1 && Owner->GetSubBodies()[0] == Solid &&
            Solid->GetClass()->GetFName() == FName(TEXT("BallSubBody")))
        {
            ObservedGeneration = SeamGeneration;
            const auto* Provider = Speed::Analytic::FStaticWorldQueryAudit::ReadStrictFrameProviderForObservation(
                World,SeamFrame,SeamGeneration);
            if (Provider)
            {
                const auto Query = Speed::Analytic::FStaticWorldQueryAudit::MakeStrictSingleQuery1075(
                    Start,End,KS.Rotation,QueryShape,AuthorityTraceChannel,AuthorityResponse);
                static const bool bTryResponse = FParse::Param(FCommandLine::Get(),TEXT("SLC2QualificationRefinedResponse"));
                const auto* Sphere = Cast<USphereSubBody>(Solid);
                Speed::Analytic::FWorldHit Refined;
                ObserveC259DirectedBall(*Provider,SeamFrame,SeamGeneration,
                    Owner->GetPublishedSimulationStableId(),Owner->NumFrame(),0,Query,AnalyticHit,
                    delta,Sphere ? double(Sphere->GetRadius()) : 0.,bTryResponse && Sphere,Refined);
                if (Refined.bHit)
                    AnalyticHit = Refined;
            }
            else
                UE_LOG(SpeedPhysicsLog,Display,TEXT("[C2BallSeamDirectedObservation] Frame=%llu Body=%llu Generation=%llu Stage=provider Joined=0 NoSubstitution=1"),
                    SeamFrame,Owner->GetPublishedSimulationStableId(),SeamGeneration);
        }
    }
#endif
    OutHit = SHitResult();
    OutHit.bHit = bHit;
    OutHit.bBlockingHit = bHit;
    if (bHit)
    {
		OutHit = bUsedAnalyticAuthority
			? SHitResult::FromAnalyticHit(AnalyticHit, delta, Hit.Component.Get())
			: SHitResult::FromUnrealHit(Hit, delta);
#if !UE_BUILD_SHIPPING
        // Follow the existing one-frame solver trace. This only observes the
        // hit returned to the solver and never repeats a collision query.
        static const auto* SolverTraceFrame = IConsoleManager::Get().FindTConsoleVariableDataInt(
            TEXT("p.IAmSpeed.Collision.SolverEventTraceFrame"));
        const int32 TraceFrame = SolverTraceFrame ? SolverTraceFrame->GetValueOnAnyThread() : -1;
        uint64 CanonicalFrame = 0, SceneGeneration = 0;
        if (Owner && bUsedAnalyticAuthority && QueryShape.IsSphere() &&
            Cast<USphereSubBody>(Solid) &&
            Speed::Analytic::FStaticWorldQueryAudit::TryObserveStrictFrame(World, CanonicalFrame, SceneGeneration) &&
            ((TraceFrame >= 0 && CanonicalFrame == static_cast<uint64>(TraceFrame)) ||
             Speed::Analytic::FStaticWorldQueryAudit::IsQualificationContactTraceFrame(CanonicalFrame)))
        {
            const auto Query = Speed::Analytic::FStaticWorldQueryAudit::MakeStrictSingleQuery1075(
                Start, End, KS.Rotation, QueryShape, AuthorityTraceChannel, AuthorityResponse);
            const uint64 Body = Owner->GetPublishedSimulationStableId();
            const int32 BodyIndex = Owner->GetSubBodies().IndexOfByKey(Solid);
            uint32 DeltaBits = 0, ConsumedBits = 0;
            FMemory::Memcpy(&DeltaBits, &delta, sizeof(DeltaBits));
            FMemory::Memcpy(&ConsumedBits, &OutHit.TOI, sizeof(ConsumedBits));
            UE_LOG(SpeedPhysicsLog, Display,
                TEXT("[OrdinarySphereHitBridge] Frame=%llu OwnerFrame=%u Body=%llu SubBody=%d SceneGeneration=%llu Channel=%u Mask=%016llX Radius=%.17g Domain=%.17g InitialTolerance=%.17g "
                     "Start=(%.17g,%.17g,%.17g) End=(%.17g,%.17g,%.17g) Source=%llu Surface=%llu Feature=%llu Primitive=%llu Group=%llu "
                     "NormalizedTime=%.17g DeltaBits=%08x ConsumedTOIBits=%08x ConsumedTOI=%.9g Penetrating=%d Depth=%.9g "
                     "SpherePoint=(%.17g,%.17g,%.17g) SurfacePoint=(%.17g,%.17g,%.17g) Normal=(%.17g,%.17g,%.17g)"),
                CanonicalFrame, Owner->NumFrame(), Body, BodyIndex, SceneGeneration, static_cast<uint32>(Query.TraceChannel), Query.BlockingObjectTypes, Query.Radius,
                Query.DomainTolerance, Query.InitialOverlapTolerance, Start.X, Start.Y, Start.Z, End.X, End.Y, End.Z,
                OutHit.SourceId, OutHit.SurfaceId, OutHit.FeatureId, OutHit.PrimitiveId, OutHit.CanonicalGroupId,
                AnalyticHit.Time, DeltaBits, ConsumedBits, OutHit.TOI, OutHit.bStartPenetrating ? 1 : 0, OutHit.PenetrationDepth,
                OutHit.ContactPointThis.X, OutHit.ContactPointThis.Y, OutHit.ContactPointThis.Z,
                OutHit.ContactPointOther.X, OutHit.ContactPointOther.Y, OutHit.ContactPointOther.Z,
                OutHit.ImpactNormal.X, OutHit.ImpactNormal.Y, OutHit.ImpactNormal.Z);
        }
#endif
    }
    return bHit;
}
