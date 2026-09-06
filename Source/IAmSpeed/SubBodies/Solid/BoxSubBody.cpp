// Fill out your copyright notice in the Description page of Project Settings.


#include "BoxSubBody.h"
#include "IAmSpeed/World/Collision/BoxRestingSupport.h"
#include "IAmSpeed/World/Collision/PlanarContactImpulse.h"
#include "IAmSpeed/World/Analytic/StaticWorldQueryAudit.h"
#include "IAmSpeed/Base/SpeedConstant.h"
#include "IAmSpeed/Base/SUtils.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "IAmSpeed/World/Collision/StaticCollisionWorld.h"
#include "IAmSpeed/World/Collision/CollisionResponseMask.h"
#include "SphereSubBody.h"
#include "SWheelSubBody.h"
#include "Configs/SubBodyConfig.h"
#include "PhysicsEngine/BoxElem.h"
#include "PhysicsEngine/BodySetup.h"
#include "HAL/IConsoleManager.h"
#include "Math/QuatRotationTranslationMatrix.h"

DEFINE_LOG_CATEGORY(BoxSubBodyLog);

bool UBoxSubBody::ShouldSkipSphereSweep(const USphereSubBody& Sphere) const
{
	UWorld* World = GetWorld();
	const USpeedWorldSubsystem* SpeedWorld = World
		? World->GetSubsystem<USpeedWorldSubsystem>()
		: nullptr;
	return SpeedWorld &&
		SpeedWorld->IsDynamicContactPairOwnedByRollingManifold(*this, Sphere);
}

static TAutoConsoleVariable<int32> CVarIAmSpeedCollisionDebugSphereBoxImpulse(
    TEXT("p.IAmSpeed.Collision.DebugSphereBoxImpulse"),
    0,
    TEXT("Logs the real CCD impulse exchanged by sphere/box contacts before gameplay fake physics."));

static TAutoConsoleVariable<float> CVarIAmSpeedBoxInertiaScale(
    TEXT("p.IAmSpeed.Box.InertiaScale"),
    1.0f,
    TEXT("Scales box inertia at initialization. Runtime changes require subbody reinitialization to affect existing boxes."));

static TAutoConsoleVariable<int32> CVarIAmSpeedBoxSupportedTransitionResponse(
    TEXT("p.IAmSpeed.Box.SupportedTransitionResponse"),
    0,
    TEXT("When non-zero, a fresh principal-body hit on the same static surface as an established subordinate support manifold uses the bounded transition response instead of a free aerial impact."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarIAmSpeedBoxDeferSupportedTransitionVelocityTransport(
    TEXT("p.IAmSpeed.Box.DeferSupportedTransitionVelocityTransport"),
    1,
    TEXT("When non-zero, a principal body already constrained by a compatible subordinate static support does not independently transport COM velocity across that surface. Positional feasibility remains enforced."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarIAmSpeedAutoRecoverContactDebug(
    TEXT("p.IAmSpeed.AutoRecoverContactDebug"),
    0,
    TEXT("Logs hitbox ground-contact persistence data used by Skycar auto-recover."));

static TAutoConsoleVariable<int32> CVarIAmSpeedBoxGroundSupportDebug(
    TEXT("p.IAmSpeed.Box.DebugGroundSupport"),
    0,
    TEXT("Logs cumulative normal and tangential impulses applied by box ground support."));

static TAutoConsoleVariable<float> CVarIAmSpeedRoofSlideSupportMinUpDot(
    TEXT("p.IAmSpeed.RoofSlideSupportMinUpDot"),
    0.35f,
    TEXT("Minimum world-up dot for upside-down roof contacts to be resolved as sliding gutter support."));

static TAutoConsoleVariable<float> CVarIAmSpeedEdgeLatchFaceReleaseDot(
    TEXT("p.IAmSpeed.EdgeLatchFaceReleaseDot"),
    0.995f,
    TEXT("Abs dot between hitbox up and support normal above which a latched edge is released so face support can take over."));

static TAutoConsoleVariable<float> CVarIAmSpeedFaceSupportContactEpsilonCm(
    TEXT("p.IAmSpeed.FaceSupportContactEpsilonCm"),
    3.0f,
    TEXT("Contact manifold tolerance used when the hitbox is nearly face-aligned with a support plane."));

static TAutoConsoleVariable<float> CVarIAmSpeedFaceSupportPenetrationTolCm(
    TEXT("p.IAmSpeed.FaceSupportPenetrationTolCm"),
    3.0f,
    TEXT("Penetration depth tolerated while preserving nearly face-aligned hitbox support contacts, in cm."));

static TAutoConsoleVariable<float> CVarIAmSpeedFaceSupportSeparatingSpeedCmS(
    TEXT("p.IAmSpeed.FaceSupportSeparatingSpeedCmS"),
    12.0f,
    TEXT("Separating normal speed tolerated while preserving nearly face-aligned hitbox support contacts, in cm/s."));

static TAutoConsoleVariable<float> CVarIAmSpeedEdgeRecoverPenetrationTolCm(
    TEXT("p.IAmSpeed.EdgeRecoverPenetrationTolCm"),
    3.0f,
    TEXT("Penetration depth tolerated while preserving a latched two-point hitbox edge for auto-recover, in cm."));

static TAutoConsoleVariable<float> CVarIAmSpeedEdgeRecoverSeparatingSpeedCmS(
    TEXT("p.IAmSpeed.EdgeRecoverSeparatingSpeedCmS"),
    12.0f,
    TEXT("Separating normal speed tolerated while preserving a latched two-point hitbox edge for auto-recover, in cm/s."));

static TAutoConsoleVariable<float> CVarIAmSpeedVertexSupportContactTolCm(
    TEXT("p.IAmSpeed.VertexSupportContactTolCm"),
    0.35f,
    TEXT("Positive support-plane tolerance kept for single-vertex hitbox contacts during auto-recover."));

static TAutoConsoleVariable<float> CVarIAmSpeedVertexSupportSeparatingSpeedCmS(
    TEXT("p.IAmSpeed.VertexSupportSeparatingSpeedCmS"),
    3.0f,
    TEXT("Separating speed tolerated for single-vertex hitbox contacts during auto-recover, in cm/s."));

static TAutoConsoleVariable<float> CVarIAmSpeedSoftWallDepenMaxNormalKillCmS(
    TEXT("p.IAmSpeed.SoftWallDepenMaxNormalKillCmS"),
    350.0f,
    TEXT("Maximum inward normal velocity removed in one penetration frame for soft wall/gutter contacts, in cm/s. <= 0 disables the clamp."));

static TAutoConsoleVariable<float> CVarIAmSpeedGutterDepenMaxUpDeltaCmS(
    TEXT("p.IAmSpeed.GutterDepenMaxUpDeltaCmS"),
    90.0f,
    TEXT("Maximum upward velocity added by one soft gutter depenetration velocity correction, in cm/s. <= 0 disables the clamp."));

static TAutoConsoleVariable<float> CVarIAmSpeedNearGutterVerticalDepenMaxNormalKillCmS(
    TEXT("p.IAmSpeed.NearGutterVerticalDepenMaxNormalKillCmS"),
    800.0f,
    TEXT("Maximum inward normal velocity removed in one penetration frame for near-vertical gutter seam contacts, in cm/s. <= 0 uses the regular soft wall clamp."));

static TAutoConsoleVariable<float> CVarIAmSpeedGutterSlideDamping(
    TEXT("p.IAmSpeed.GutterSlideDamping"),
    3.25f,
    TEXT("Exponential tangential velocity damping applied while the main hitbox slides upside-down in gutters, in 1/s. <= 0 disables it."));

static TAutoConsoleVariable<float> CVarIAmSpeedGutterSlideCrossDamping(
    TEXT("p.IAmSpeed.GutterSlideCrossDamping"),
    25.0f,
    TEXT("Additional damping applied only across the gutter while the roof follows a curved stadium surface, in 1/s."));

static TAutoConsoleVariable<float> CVarIAmSpeedGutterSlideMaxDeltaCmS(
    TEXT("p.IAmSpeed.GutterSlideMaxDeltaCmS"),
    45.0f,
    TEXT("Maximum tangential velocity removed per physics frame by gutter slide damping, in cm/s. <= 0 disables the per-frame cap."));

static TAutoConsoleVariable<float> CVarIAmSpeedGutterSlideMinSpeedCmS(
    TEXT("p.IAmSpeed.GutterSlideMinSpeedCmS"),
    120.0f,
    TEXT("Minimum tangential speed required before gutter slide damping is applied, in cm/s."));

static TAutoConsoleVariable<float> CVarIAmSpeedGutterSlideMaxClimbSpeedCmS(
    TEXT("p.IAmSpeed.GutterSlideMaxClimbSpeedCmS"),
    850.0f,
    TEXT("Maximum uphill speed allowed while the main hitbox slides upside-down along a gutter slope, in cm/s. <= 0 disables the clamp."));

static TAutoConsoleVariable<float> CVarIAmSpeedGutterSlideMaxClimbBrakeDeltaCmS(
    TEXT("p.IAmSpeed.GutterSlideMaxClimbBrakeDeltaCmS"),
    40.0f,
    TEXT("Maximum uphill velocity removed per physics frame by the gutter climb-speed clamp, in cm/s."));

namespace
{
    // Stadium gutter seams can leave the roof unsupported for several 300 Hz frames.
    constexpr int32 RoofSurfaceTraversalGraceFrames = 60;

    FVector IAmSpeedVelocityAtPointFromKS(const SKinematic& KS, const FVector& Point)
    {
        return KS.Velocity + FVector::CrossProduct(KS.AngularVelocity, Point - KS.Location);
    }

    void IAmSpeedApplyGutterSlideDamping(
        ISpeedComponent* ParentComponent,
        const bool bIsMainSubBody,
        const FVector& SupportNormal,
        const float Dt,
        const bool bEnableDamping,
        const TCHAR* Source)
    {
        if (!ParentComponent || !bIsMainSubBody || !bEnableDamping || Dt <= 0.f)
        {
            return;
        }

        const float Damping = CVarIAmSpeedGutterSlideDamping.GetValueOnAnyThread();
        const FVector N = SupportNormal.GetSafeNormal();
        if (N.IsNearlyZero())
        {
            return;
        }

        const FVector V = ParentComponent->GetPhysCOMVelocity();
        const FVector VT = V - FVector::DotProduct(V, N) * N;
        const float TangentSpeed = VT.Size();
        const float MinSpeed = FMath::Max(0.f, CVarIAmSpeedGutterSlideMinSpeedCmS.GetValueOnAnyThread());
        FVector DeltaV = FVector::ZeroVector;

        if (Damping > 0.f && TangentSpeed > MinSpeed)
        {
            const float Alpha = FMath::Clamp(1.f - FMath::Exp(-Damping * Dt), 0.f, 1.f);
            DeltaV = -Alpha * VT;

            const float MaxDelta = CVarIAmSpeedGutterSlideMaxDeltaCmS.GetValueOnAnyThread();
            if (MaxDelta > 0.f)
            {
                DeltaV = DeltaV.GetClampedToMaxSize(MaxDelta);
            }
        }

        FVector ClimbDeltaV = FVector::ZeroVector;
        const float MaxClimbSpeed = CVarIAmSpeedGutterSlideMaxClimbSpeedCmS.GetValueOnAnyThread();
        const float MaxClimbBrakeDelta =
            FMath::Max(0.f, CVarIAmSpeedGutterSlideMaxClimbBrakeDeltaCmS.GetValueOnAnyThread());
        const bool bGutterSlope = N.Z > 0.20f && N.Z < 0.98f;
        float ClimbSpeed = 0.f;
        if (MaxClimbSpeed > 0.f && MaxClimbBrakeDelta > 0.f && bGutterSlope)
        {
            const FVector Uphill = (FVector::UpVector - FVector::DotProduct(FVector::UpVector, N) * N).GetSafeNormal();
            if (!Uphill.IsNearlyZero())
            {
                ClimbSpeed = FVector::DotProduct(V, Uphill);
                if (ClimbSpeed > MaxClimbSpeed)
                {
                    const float BrakeDelta = FMath::Min(ClimbSpeed - MaxClimbSpeed, MaxClimbBrakeDelta);
                    ClimbDeltaV = -BrakeDelta * Uphill;
                    DeltaV += ClimbDeltaV;
                }

                const FVector CrossGutter = FVector::CrossProduct(N, Uphill).GetSafeNormal();
                const float CrossDamping =
                    FMath::Max(0.f, CVarIAmSpeedGutterSlideCrossDamping.GetValueOnAnyThread());
                if (!CrossGutter.IsNearlyZero() && CrossDamping > 0.f)
                {
                    const float CrossSpeed = FVector::DotProduct(V, CrossGutter);
                    const float CrossAlpha =
                        FMath::Clamp(1.f - FMath::Exp(-CrossDamping * Dt), 0.f, 1.f);
                    DeltaV -= CrossAlpha * CrossSpeed * CrossGutter;
                }
            }
        }

        if (DeltaV.IsNearlyZero())
        {
            return;
        }

        ParentComponent->AddPhysVelocity(DeltaV);

#if !UE_BUILD_SHIPPING
        if (CVarIAmSpeedAutoRecoverContactDebug.GetValueOnAnyThread() != 0)
        {
            UE_LOG(
                BoxSubBodyLog,
                Warning,
                TEXT("[CCDBoxGutterSlideDamping] Frame=%d Source=%s N=%s TangentSpeed=%.2f ClimbSpeed=%.2f DeltaV=%s ClimbDeltaV=%s Damping=%.2f MaxDelta=%.2f MaxClimb=%.2f Velocity=%s"),
                ParentComponent->NumFrame(),
                Source,
                *N.ToString(),
                TangentSpeed,
                ClimbSpeed,
                *DeltaV.ToString(),
                *ClimbDeltaV.ToString(),
                Damping,
                CVarIAmSpeedGutterSlideMaxDeltaCmS.GetValueOnAnyThread(),
                MaxClimbSpeed,
                *V.ToString());
        }
#endif
    }

    FVector IAmSpeedFaceLocalNormal(int32 FaceIndex)
    {
        switch (FaceIndex)
        {
        case 0: return FVector(+1, 0, 0);
        case 1: return FVector(-1, 0, 0);
        case 2: return FVector(0, +1, 0);
        case 3: return FVector(0, -1, 0);
        case 4: return FVector(0, 0, +1);
        case 5: return FVector(0, 0, -1);
        default: return FVector::ZeroVector;
        }
    }

    void IAmSpeedFaceLocalVertices(int32 FaceIndex, const FVector& Ext, FVector Out[4])
    {
        const float Ex = Ext.X;
        const float Ey = Ext.Y;
        const float Ez = Ext.Z;

        switch (FaceIndex)
        {
        case 0:
            Out[0] = FVector(+Ex, -Ey, -Ez);
            Out[1] = FVector(+Ex, -Ey, +Ez);
            Out[2] = FVector(+Ex, +Ey, +Ez);
            Out[3] = FVector(+Ex, +Ey, -Ez);
            break;
        case 1:
            Out[0] = FVector(-Ex, -Ey, -Ez);
            Out[1] = FVector(-Ex, +Ey, -Ez);
            Out[2] = FVector(-Ex, +Ey, +Ez);
            Out[3] = FVector(-Ex, -Ey, +Ez);
            break;
        case 2:
            Out[0] = FVector(-Ex, +Ey, -Ez);
            Out[1] = FVector(-Ex, +Ey, +Ez);
            Out[2] = FVector(+Ex, +Ey, +Ez);
            Out[3] = FVector(+Ex, +Ey, -Ez);
            break;
        case 3:
            Out[0] = FVector(-Ex, -Ey, -Ez);
            Out[1] = FVector(+Ex, -Ey, -Ez);
            Out[2] = FVector(+Ex, -Ey, +Ez);
            Out[3] = FVector(-Ex, -Ey, +Ez);
            break;
        case 4:
            Out[0] = FVector(-Ex, -Ey, +Ez);
            Out[1] = FVector(+Ex, -Ey, +Ez);
            Out[2] = FVector(+Ex, +Ey, +Ez);
            Out[3] = FVector(-Ex, +Ey, +Ez);
            break;
        case 5:
            Out[0] = FVector(-Ex, -Ey, -Ez);
            Out[1] = FVector(-Ex, +Ey, -Ez);
            Out[2] = FVector(+Ex, +Ey, -Ez);
            Out[3] = FVector(+Ex, -Ey, -Ez);
            break;
        default:
            Out[0] = Out[1] = Out[2] = Out[3] = FVector::ZeroVector;
            break;
        }
    }

    float IAmSpeedFindBestSupportFaceAlignment(const FVector& N, const FQuat& RotWS, int32* OutFaceIndex = nullptr)
    {
        int32 BestFaceIndex = INDEX_NONE;
        float BestOpposingDot = FLT_MAX;

        for (int32 FaceIndex = 0; FaceIndex < 6; ++FaceIndex)
        {
            const FVector FaceNormalWS = RotWS.RotateVector(IAmSpeedFaceLocalNormal(FaceIndex)).GetSafeNormal();
            const float Dot = FVector::DotProduct(FaceNormalWS, N);
            if (Dot < BestOpposingDot)
            {
                BestOpposingDot = Dot;
                BestFaceIndex = FaceIndex;
            }
        }

        if (OutFaceIndex)
        {
            *OutFaceIndex = BestFaceIndex;
        }

        return BestFaceIndex != INDEX_NONE ? FMath::Abs(BestOpposingDot) : 0.0f;
    }

    bool IAmSpeedBuildSupportFaceManifoldFromNormal(
        const FVector& N,
        const FVector& CenterWS,
        const FQuat& RotWS,
        const FVector& Ext,
        TArray<FVector>& OutPts,
        float MinFaceAlignmentAbs)
    {
        int32 FaceIndex = INDEX_NONE;
        const float FaceAlignmentAbs = IAmSpeedFindBestSupportFaceAlignment(N, RotWS, &FaceIndex);
        if (FaceIndex == INDEX_NONE || FaceAlignmentAbs < MinFaceAlignmentAbs)
        {
            return false;
        }

        FVector FaceVertsLS[4];
        IAmSpeedFaceLocalVertices(FaceIndex, Ext, FaceVertsLS);

        OutPts.Reset(4);
        for (int32 Index = 0; Index < 4; ++Index)
        {
            OutPts.Add(CenterWS + RotWS.RotateVector(FaceVertsLS[Index]));
        }
        return true;
    }

    bool IAmSpeedBuildRoofTransitionEdge(
        const FVector& N,
        const FVector& CenterWS,
        const FQuat& RotWS,
        const FVector& Ext,
        TArray<FVector>& OutPts)
    {
        const FVector RoofNormalWS = RotWS.GetUpVector();
        if (FVector::DotProduct(RoofNormalWS, N) >= -0.45f)
        {
            return false;
        }

        FVector RoofVertsLS[4];
        IAmSpeedFaceLocalVertices(4, Ext, RoofVertsLS);

        FVector RoofVertsWS[4];
        float Projections[4];
        for (int32 Index = 0; Index < 4; ++Index)
        {
            RoofVertsWS[Index] = CenterWS + RotWS.RotateVector(RoofVertsLS[Index]);
            Projections[Index] = FVector::DotProduct(RoofVertsWS[Index], N);
        }

        int32 BestEdgeStart = 0;
        float BestProjectionSum = TNumericLimits<float>::Max();
        for (int32 EdgeStart = 0; EdgeStart < 4; ++EdgeStart)
        {
            const int32 EdgeEnd = (EdgeStart + 1) % 4;
            const float ProjectionSum = Projections[EdgeStart] + Projections[EdgeEnd];
            if (ProjectionSum < BestProjectionSum)
            {
                BestProjectionSum = ProjectionSum;
                BestEdgeStart = EdgeStart;
            }
        }

        OutPts.Reset(2);
        OutPts.Add(RoofVertsWS[BestEdgeStart]);
        OutPts.Add(RoofVertsWS[(BestEdgeStart + 1) % 4]);
        return true;
    }
}

UBoxSubBody::UBoxSubBody(const FObjectInitializer& ObjectInitializer):
	Super(ObjectInitializer)
#if WITH_EDITOR
    , ShowFlags(ESFIM_All0)
#endif // WITH_EDITOR
{
	SubBodyType = ESubBodyType::Hitbox;
    bUseEditorCompositing = true;
    BoxExtent = FVector(32.f, 32.f, 32.f);
}

void UBoxSubBody::Initialize(ISpeedComponent* InParentComponent)
{
	Super::Initialize(InParentComponent);
	if (ParentComponent)
	{
		SubBodyConfig Config = ParentComponent->GetSubBodyConfig(*this);
        if (Config.bValid)
        {
            BoxExtent = Config.BoxExtent;
			InvInertiaLocal = InitInvInertiaTensor();
        }
        else
        {
            InvInertiaLocal = InitInvInertiaTensor();
        }
	}
}

FVector UBoxSubBody::GetStaticRestingReaction(const Speed::IStaticCollisionWorld& World, double* StopAfterSeconds) const
{
    if (!ParentComponent || !HasToApplyRestForce()) return FVector::ZeroVector;
    Speed::FBoxRestingSupport Support;
    if (!EvaluateStaticBoxSupport(World, ParentComponent->GetPhysAcceleration(), Support, true,
        ParentComponent->GetStaticSupportFrameHorizon())) return FVector::ZeroVector;
    if (StopAfterSeconds) *StopAfterSeconds = Support.StopAfterSeconds;
    return Support.ResultantAcceleration;
}

double UBoxSubBody::GetStaticSupportStopTimeCandidate() const
{
    if (!ParentComponent || !HasToApplyRestForce() || GetDynamicFriction() <= 0) return TNumericLimits<double>::Max();
    const SKinematic& State = ParentComponent->GetKinematicState();
    const double Load = State.Acceleration.Size(), Speed = State.Velocity.Size();
    if (Load == 0 || Speed == 0 || !State.AngularVelocity.IsZero() || !State.AngularAcceleration.IsZero() ||
        FVector::DotProduct(State.Velocity, State.Acceleration) != 0) return TNumericLimits<double>::Max();
    return Speed / (GetDynamicFriction() * Load);
}

bool UBoxSubBody::HasExactStaticRestingSupport(const Speed::IStaticCollisionWorld& World, const FVector& Load) const
{
    if (!ParentComponent) return false;
    Speed::FBoxRestingSupport Support;
    return HasToApplyRestForce() && EvaluateStaticBoxSupport(World, Load, Support, true);
}

bool UBoxSubBody::EvaluateStaticRestingSupport(const Speed::IStaticCollisionWorld& World,
    const FVector& ExternalAcceleration, Speed::FBoxRestingSupport& OutSupport) const
{
    return EvaluateStaticBoxSupport(World, ExternalAcceleration, OutSupport, false);
}

Speed::Analytic::FWorldQuery UBoxSubBody::MakeSupportBoxQuery(const SKinematic& State) const
{
    // Read current COM state, not a recorded frame or a previous contact pose.
    SKinematic Origin = State;
    Origin.Location = State.Location - State.Rotation.RotateVector(ParentComponent->GetPhysCenterOfMassLocal());
    const SKinematic Box = GetKinematicsFromOwnerKS(Origin);
    Speed::Analytic::FWorldQuery Query;
    Query.Shape = Speed::Analytic::EQueryShape::Box;
    Query.Start = Query.End = Box.Location;
    Query.Rotation = Box.Rotation;
    Query.HalfExtent = BoxExtent;
    Query.TraceChannel = static_cast<uint8>(GetCollisionChannel());
    Query.BlockingObjectTypes = Speed::GetBlockingResponseMask(GetResponseParams().CollisionResponse);
    Query.bApplyCollisionFilter = true;
    Query.bAuthorityOnly = true;
    Query.bIncludeCompactPatches = true;
    Query.DomainTolerance = Query.InitialOverlapTolerance = 0;
    return Query;
}

namespace
{
// A first contact may not have a persistent seed yet. Resolve only a proven
// double-roundoff crossing of an exact finite plane, never a finite overlap.
// SAT's projected radius and independently transformed corners can differ by
// several ULPs when the box has a local transform and an off-center COM.
FVector FindBoxPlanarRoundoffCorrection(const Speed::IStaticCollisionWorld& World,
    const Speed::Analytic::FWorldQuery& Box, bool bMoving)
{
    FVector Correction = FVector::ZeroVector;
    const double Scale = FMath::Max(1.0, Box.Start.GetAbsMax() + Box.HalfExtent.GetMax());
    const double Roundoff = 64 * DBL_EPSILON * Scale;
    auto Broad = Box;
    Broad.Shape = Speed::Analytic::EQueryShape::Sphere;
    Broad.Radius = Box.HalfExtent.Size() + Roundoff;
    World.VisitPlanarCandidates(Broad, [&](const Speed::Analytic::FWorldHit& Plane)
    {
        if (!Correction.IsZero()) return;
        double Gap = DBL_MAX;
        FVector Lowest = FVector::ZeroVector;
        for (int32 I = 0; I < 8; ++I)
        {
            const FVector P = Box.Start + Box.Rotation.RotateVector(FVector(
                (I & 1) ? Box.HalfExtent.X : -Box.HalfExtent.X,
                (I & 2) ? Box.HalfExtent.Y : -Box.HalfExtent.Y,
                (I & 4) ? Box.HalfExtent.Z : -Box.HalfExtent.Z));
            const double D = FVector::DotProduct(P - Plane.Point, Plane.Normal);
            if (D < Gap) { Gap = D; Lowest = P; }
        }
        const double Exterior = bMoving ? 16 * DBL_EPSILON * Scale : 0;
        if (Gap >= Exterior || Gap < -Roundoff) return;
        auto Probe = Box;
        Probe.Shape = Speed::Analytic::EQueryShape::Ray;
        Probe.RequiredSourceId = Plane.SourceId; Probe.RequiredSurfaceId = Plane.SurfaceId;
        Probe.Start = Lowest + .01 * Plane.Normal; Probe.End = Lowest - .01 * Plane.Normal;
        const auto Witness = World.SweepSingle(Probe);
        if (Witness.bHit && !Witness.bStartPenetrating && !Witness.bSurfaceNormalMayVary &&
            Witness.GeometricErrorBoundCm == 0 && Witness.SurfaceFeatureKind == Speed::EContactFeatureKind::Face &&
            Witness.Normal.Equals(Plane.Normal, 32 * DBL_EPSILON))
            Correction = (-Gap + 16 * DBL_EPSILON * Scale) * Plane.Normal;
    });
    return Correction;
}
}

bool UBoxSubBody::CanApplyQuantizedPose(const Speed::IStaticCollisionWorld& World, const SKinematic& OwnerPose) const
{
    if (!ParentComponent) return true;
    const auto Query = MakeSupportBoxQuery(OwnerPose);
    const auto Hit = World.SweepSingle(Query);
    return !Hit.bStartPenetrating && Hit.PenetrationDepth <= 0 &&
        FindBoxPlanarRoundoffCorrection(World, Query,
            !OwnerPose.Velocity.IsZero() || !OwnerPose.AngularVelocity.IsZero()).IsZero();
}

bool UBoxSubBody::EvaluateStaticBoxSupport(const Speed::IStaticCollisionWorld& World,
    const FVector& ExternalAcceleration, Speed::FBoxRestingSupport& OutSupport,
    bool bIncludeMotion, double HorizonSeconds) const
{
    OutSupport = {};
    if (!ParentComponent) return false;
    const SKinematic& State = ParentComponent->GetKinematicState();
    const auto Query = MakeSupportBoxQuery(State);
    return !bIncludeMotion || State.Velocity.IsZero()
        ? Speed::TryBuildBoxRestingSupport(World, Query, State.Location, ExternalAcceleration, OutSupport)
        : Speed::TryBuildBoxSlidingSupport(World, Query, State.Location, ExternalAcceleration,
            State.Velocity, GetDynamicFriction(), OutSupport, HorizonSeconds);
}

void UBoxSubBody::PostPhysicsUpdate()
{
    Super::PostPhysicsUpdate();
    PublishExactStaticSupport();
}

bool UBoxSubBody::PublishExactStaticSupport()
{
    const Speed::IStaticCollisionWorld* World = ParentComponent ? ParentComponent->GetStaticCollisionWorldForFrame() : nullptr;
    if (!World || !HasToApplyRestForce() || ParentComponent->IsFrozen()) return false;
    const SKinematic& State = ParentComponent->GetKinematicState();
    const FVector Load = ParentComponent->GetNominalGravityAcceleration();
    if (Load.IsZero() || !State.AngularVelocity.IsZero() || FVector::DotProduct(State.Velocity, Load) != 0) return false;
    Speed::FBoxRestingSupport Support;
    if (!EvaluateStaticBoxSupport(*World, Load, Support, true)) return false;
    const USpeedWorldSubsystem* Bridge = GetWorld() ? GetWorld()->GetSubsystem<USpeedWorldSubsystem>() : nullptr;
    if (!Bridge) return false;
    const auto Source = Bridge->FindAnalyticSourceComponent(Support.ProviderWitness.SourceId);
    if (!Source.IsValid()) return false;
    // A stationary query can correctly report no sweep impact at an exterior
    // representable pose. The four finite support witnesses still establish
    // a passive contact. Publish that fact without an impulse or gameplay hit.
    GroundHit = SHitResult::FromAnalyticHit(Support.ProviderWitness, 0, Source.Get(), ParentComponent->NumFrame());
    GroundHit.ContactFeatureThis = Speed::EContactFeatureKind::Face;
    GroundPlaneN = Support.Normal;
    GroundPlanePointWS = Support.Points[0];
    GroundPlaneD = FVector::DotProduct(GroundPlanePointWS, GroundPlaneN);
    GroundComp = Source;
    bGroundPlaneValid = bHasGroundContact = true;
    CurrentGroundContactsWS.Reset(4);
    for (const FVector& P : Support.Points) CurrentGroundContactsWS.Add(P);
    return true;
}

void UBoxSubBody::ResetForFrame(const float& Delta)
{
    Super::ResetForFrame(Delta);

    const bool bVariableNormalSupportReady =
        !GroundHit.bSurfaceNormalMayVary ||
        RefreshVariableNormalGroundSupport(Delta);
    const bool bContinueRoofTraversalSupport =
        bVariableNormalSupportReady &&
        HasRoofSurfaceTraversalSupport() &&
        bGroundPlaneValid &&
        GroundComp.IsValid();
	const bool bNativeAnalyticEstablishedContact =
		bContinueRoofTraversalSupport &&
		GroundHit.bSurfaceNormalMayVary &&
		GroundHit.SourceId != 0 && GroundHit.SurfaceId != 0 &&
		GroundHit.CanonicalGroupId != 0 &&
		Speed::Analytic::FStaticWorldQueryAudit::IsSurfaceAnalyticBackend();
    if (bContinueRoofTraversalSupport)
    {
        UpdatePersistentGroundContact(Delta);
		// The analytical provider is the constraint manifold for a varying
		// surface. Reapplying the legacy cached-plane support correction here
		// double-solves the same contact and can pin a fast body to the tangent
		// plane as its normal rotates. IntegrateKinematics owns the native
		// unilateral projection; the legacy plane remains for fixed-normal or
		// non-authoritative contacts only.
        if (HasPersistentGroundContact() &&
			!bNativeAnalyticEstablishedContact)
        {
            ApplyPersistentSupportConstraint(Delta);
        }
    }

    if (bContinueRoofTraversalSupport && GroundHit.SourceId != 0 &&
        GroundHit.SurfaceId != 0)
    {
        EstablishedSupportSourceId = GroundHit.SourceId;
        EstablishedSupportSurfaceId = GroundHit.SurfaceId;
    }

    PreviousFrameGroundContactsWS.Reset();
    if (bHasGroundContact && CurrentGroundContactsWS.Num() > 0)
    {
        PreviousFrameGroundContactsWS = CurrentGroundContactsWS;
    }

#if !UE_BUILD_SHIPPING
    if (CVarIAmSpeedAutoRecoverContactDebug.GetValueOnAnyThread() != 0 &&
        (bHasGroundContact ||
            CurrentGroundContactsWS.Num() > 0 ||
            PreviousFrameGroundContactsWS.Num() > 0 ||
            PrevGroundContactsLS.Num() > 0 ||
            bEdgeSupportLatched))
    {
        UE_LOG(
            BoxSubBodyLog,
            Log,
            TEXT("[ARContact.Reset] Frame=%d HasCurrent=%d Current=%d CachedPrev=%d PrevLS=%d Latched=%d LatchedLS=%d PlaneValid=%d GroundComp=%s"),
            ParentComponent ? ParentComponent->NumFrame() : INDEX_NONE,
            bHasGroundContact ? 1 : 0,
            CurrentGroundContactsWS.Num(),
            PreviousFrameGroundContactsWS.Num(),
            PrevGroundContactsLS.Num(),
            bEdgeSupportLatched ? 1 : 0,
            LatchedEdgeContactsLS.Num(),
            bGroundPlaneValid ? 1 : 0,
            GroundComp.IsValid() ? *GroundComp->GetName() : TEXT("None"));
    }
#endif

    HitCountThisFrame.Empty();
    CurrentGroundContactsWS.Empty();
    CurrentGroundNormalsWS.Empty();

    bGroundHitFromSweep = false;
    bHasGroundContact =
        bContinueRoofTraversalSupport &&
        bGroundPlaneValid;
    bFreshEdgeRecoverCandidate = false;
}

bool UBoxSubBody::RefreshVariableNormalGroundSupport(const float Delta)
{
    if (!ParentComponent ||
        !GroundHit.bSurfaceNormalMayVary ||
        GroundHit.SourceId == 0 || GroundHit.SurfaceId == 0 ||
        GroundHit.CanonicalGroupId == 0 ||
        !GroundComp.IsValid())
    {
        return false;
    }

    UWorld* World = GetWorld();
    if (!World)
    {
        return false;
    }

    const FVector PreviousNormal = GroundPlaneN.GetSafeNormal();
    if (PreviousNormal.IsNearlyZero())
    {
        return false;
    }

    const SKinematic State = GetKinematicsFromOwner(ParentComponent->NumFrame());
    const FVector Start = State.Location;
    constexpr float InitialOverlapShellCm = 0.05f;
    const float StepSeconds = FMath::Max(0.f, Delta);
    const float BoxVertexRadiusCm = BoxExtent.Size();
    const float LinearTravelBoundCm =
        (State.Velocity.Size() +
            0.5f * State.Acceleration.Size() * StepSeconds) * StepSeconds;
    const float AngularTravelBoundCm =
        (State.AngularVelocity.Size() +
            0.5f * State.AngularAcceleration.Size() * StepSeconds) *
        BoxVertexRadiusCm * StepSeconds;
    // A support probe must cover the greatest motion of any OBB vertex over
    // the canonical step. A fixed contact-band probe can miss a curved
    // provider even though the previous-frame contact remains continuous.
    const float ProbeDistanceCm = FMath::Max(
        InitialOverlapShellCm,
        FMath::Max(0.f, CVarIAmSpeedFaceSupportContactEpsilonCm.GetValueOnAnyThread()) +
            FMath::Max(0.f, GroundHit.GeometricErrorBoundCm) +
            InitialOverlapShellCm + LinearTravelBoundCm + AngularTravelBoundCm);
    const FVector End = Start - PreviousNormal * ProbeDistanceCm;

    FHitResult UnrealAdapterHit;
    bool bHit = false;
    Speed::Analytic::FWorldHit AnalyticHit;
	// At the canonical 300 Hz step, an established smooth provider cannot turn
	// its support normal by more than 25 degrees without first losing contact.
	// This generous bound prevents a wide OBB from reacquiring a remote cell of
	// the same canonical provider at time zero.
	constexpr float MinimumEstablishedNormalDot = 0.9f;
    const bool bUsedAnalyticAuthority =
        Speed::Analytic::FStaticWorldQueryAudit::TryCompactAuthoritySingle(
            World,
            Start, End, State.Rotation, GetCollisionShape(),
			static_cast<uint8>(GetCollisionChannel()), GetResponseParams(),
			UnrealAdapterHit, bHit, &AnalyticHit,
            GroundHit.SourceId, GroundHit.SurfaceId,
            GroundHit.CanonicalGroupId, PreviousNormal,
			MinimumEstablishedNormalDot);
    Speed::Analytic::FStaticWorldQueryAudit::RecordSingle(
        Speed::Analytic::EStaticQuerySite::BoxPersistentSupportProbe,
        Start, End, State.Rotation, GetCollisionShape(),
        static_cast<uint8>(GetCollisionChannel()), GetResponseParams(),
        bHit, UnrealAdapterHit, GroundHit.SourceId, GroundHit.SurfaceId,
        GroundHit.CanonicalGroupId);
    if (!bUsedAnalyticAuthority || !bHit ||
        !AnalyticHit.bSurfaceNormalMayVary)
    {
        return false;
    }

    SHitResult RefreshedHit =
        SHitResult::FromAnalyticHit(AnalyticHit, Delta, GroundComp.Get());
    RefreshedHit.ImpactNormal = Speed::QuantizeUnitNormal(RefreshedHit.ImpactNormal);
    RefreshedHit.TOI = Speed::QuantizeScalar(RefreshedHit.TOI, 1e-5f);
    RefreshedHit.ImpactPoint = Speed::QuantizeVectorCm(RefreshedHit.ImpactPoint, 0.1f);
    RefreshedHit.PenetrationDepth =
        Speed::QuantizeScalar(RefreshedHit.PenetrationDepth, 0.01f);
    if (RefreshedHit.ImpactNormal.IsNearlyZero())
    {
        return false;
    }

    GroundHit = RefreshedHit;
    GroundPlaneN = RefreshedHit.ImpactNormal;
    GroundPlanePointWS = RefreshedHit.ImpactPoint;
    GroundPlaneD = FVector::DotProduct(GroundPlanePointWS, GroundPlaneN);
    bGroundPlaneValid = true;
    return true;
}

bool UBoxSubBody::ProjectEstablishedStaticContact(const float& Delta)
{
    return ProjectEstablishedStaticContactImpl(Delta, true);
}

bool UBoxSubBody::RequiresUnquantizedContactPose() const
{
    return ParentComponent && ParentComponent->GetStaticCollisionWorldForFrame() &&
        LastResolvedGroundHitFrame == static_cast<int32>(ParentComponent->NumFrame()) &&
        GroundComp.IsValid() && GroundHit.SourceId != 0 &&
        !GroundHit.bSurfaceNormalMayVary && GroundHit.GeometricErrorBoundCm == 0 &&
        GroundHit.ContactFeatureOther == Speed::EContactFeatureKind::Face;
}

bool UBoxSubBody::RequiresEstablishedStaticContactTransport() const
{
	const bool bHasEstablishedAnalyticContact = ParentComponent && bHasGroundContact &&
		GroundHit.bSurfaceNormalMayVary && GroundHit.SourceId != 0 &&
		GroundHit.SurfaceId != 0 && GroundHit.CanonicalGroupId != 0 &&
		GroundComp.IsValid() &&
		Speed::Analytic::FStaticWorldQueryAudit::IsSurfaceAnalyticBackend();
	if (!bHasEstablishedAnalyticContact)
	{
		return false;
	}

	return CVarIAmSpeedBoxDeferSupportedTransitionVelocityTransport.GetValueOnAnyThread() == 0 ||
		!ParentComponent->HasCompatibleEstablishedStaticSupport(GroundHit);
}

bool UBoxSubBody::ProjectEstablishedStaticContactImpl(
    const float& Delta, const bool bProjectVelocity)
{
    if (!ParentComponent || Delta < 0.0f ||
        !Speed::Analytic::FStaticWorldQueryAudit::IsSurfaceAnalyticBackend())
    {
        return false;
    }

    // Translating the whole rigid body to satisfy the box alone can lift an
    // established wheel pivot. Let the coupled pose transaction satisfy both
    // manifolds; declining only the impulse after this translation is too late.
    const bool bExactPlanarEnabled = ParentComponent->GetStaticCollisionWorldForFrame() != nullptr &&
        !ParentComponent->HasActivePhysicalConstraintsOtherThan(this);
    const auto IsUsableSeed = [bExactPlanarEnabled](const SHitResult& Hit)
    {
        return Hit.SourceId != 0 && Hit.SurfaceId != 0 &&
            ((Hit.bSurfaceNormalMayVary && Hit.CanonicalGroupId != 0) ||
                (bExactPlanarEnabled && !Hit.bSurfaceNormalMayVary && Hit.GeometricErrorBoundCm == 0 &&
                    Hit.ContactFeatureOther == Speed::EContactFeatureKind::Face));
    };
    const bool bHasPersistentSeed = bHasGroundContact && IsUsableSeed(GroundHit) && GroundComp.IsValid();
    const bool bHasTransientSeed = !bHasPersistentSeed &&
        CurrentHit.bHit && IsUsableSeed(CurrentHit) && CurrentHit.Component.IsValid() &&
        CurrentHit.Component->GetMobility() == EComponentMobility::Static;
    if (!bHasPersistentSeed && !bHasTransientSeed)
    {
        if (bExactPlanarEnabled)
        {
            // A valid face may be passive (no impact seed). Never raise it as
            // tangential travel changes the world-coordinate roundoff scale.
            if (ParentComponent->HasExactStaticFaceSupport()) return false;
            const FVector Correction = FindBoxPlanarRoundoffCorrection(
                *ParentComponent->GetStaticCollisionWorldForFrame(), MakeSupportBoxQuery(ParentComponent->GetKinematicState()),
                !ParentComponent->GetPhysCOMVelocity().IsZero() || !ParentComponent->GetPhysAngularVelocity().IsZero());
            if (!Correction.IsZero())
            {
                ParentComponent->SetPhysCOMLocation(ParentComponent->GetPhysCOMLocation() + Correction);
                ParentComponent->UpdateSubBodiesKinematics();
                return true;
            }
        }
        return false;
    }
    const SHitResult& ProjectionSeed = bHasPersistentSeed
        ? GroundHit
        : CurrentHit;
    const bool bExactPlanarSeed = !ProjectionSeed.bSurfaceNormalMayVary;

    UWorld* World = GetWorld();
    const USpeedWorldSubsystem* SpeedWorld = World
        ? World->GetSubsystem<USpeedWorldSubsystem>() : nullptr;
    const Speed::IStaticCollisionWorld* StaticWorld = SpeedWorld
        ? SpeedWorld->GetStaticCollisionWorld() : nullptr;
    if (!StaticWorld)
    {
        return false;
    }

    const Speed::FKinematicState PredictedState = GetKinematicState();
    Speed::Analytic::FWorldQuery Query;
    Query.Shape = Speed::Analytic::EQueryShape::Box;
    Query.Start = FVector3d(PredictedState.Location);
    Query.End = Query.Start;
    Query.Rotation = FQuat4d(PredictedState.Rotation);
    Query.HalfExtent = FVector3d(BoxExtent);
    Query.TraceChannel = static_cast<uint8>(GetCollisionChannel());
    Query.BlockingObjectTypes = Speed::GetBlockingResponseMask(GetResponseParams().CollisionResponse);

    Query.bApplyCollisionFilter = true;
    Query.bAuthorityOnly = true;
    Query.bIncludeCompactPatches = true;
    Query.bIncludeTriangles = false;
    Query.RequiredSourceId = ProjectionSeed.SourceId;
    Query.RequiredSurfaceId = ProjectionSeed.SurfaceId;
    Query.RequiredCanonicalGroupId = ProjectionSeed.CanonicalGroupId;
    if (bExactPlanarSeed) Query.DomainTolerance = 0;
	Query.ReferenceNormal = FVector3d(
        bHasPersistentSeed
            ? GroundPlaneN.GetSafeNormal()
            : ProjectionSeed.ImpactNormal.GetSafeNormal());
	Query.MinimumReferenceNormalDot = 0.9;

    constexpr uint32 MaxCorrectionIterations = 4;
    constexpr double ResidualToleranceCm = 1.0e-4;
    Speed::FEstablishedStaticContactProjection Projection =
        Speed::ProjectEstablishedStaticContact(
            *StaticWorld, Query, MaxCorrectionIterations,
            bExactPlanarSeed ? 0.0 : ResidualToleranceCm);
    if (bExactPlanarSeed && (!Projection.bContact || Projection.Location == Query.Start))
    {
        // SAT's sum-of-axis-radii and the independently transformed vertices
        // can disagree by a few double ULPs. Certify the actual lowest corner
        // before directed exterior rounding, even when SAT reports zero depth.
        const FVector N = ProjectionSeed.ImpactNormal;
        FVector Lowest = FVector::ZeroVector;
        double Gap = DBL_MAX;
        for (int32 I = 0; I < 8; ++I)
        {
            const FVector P = Query.Start + Query.Rotation.RotateVector(FVector(
                (I & 1) ? BoxExtent.X : -BoxExtent.X, (I & 2) ? BoxExtent.Y : -BoxExtent.Y,
                (I & 4) ? BoxExtent.Z : -BoxExtent.Z));
            const double D = FVector::DotProduct(P - ProjectionSeed.ImpactPoint, N);
            if (D < Gap) { Gap = D; Lowest = P; }
        }
        const double Scale = FMath::Max(1.0, Query.Start.GetAbsMax() + BoxExtent.GetMax());
        const bool bRotating = !ParentComponent->GetPhysAngularVelocity().IsZero();
        // Different inlined scalar/SIMD corner evaluations may fuse different
        // operations. Reserve exterior roundoff while transporting rotation;
        // never move a certified stationary face merely to add this reserve.
        const double ExteriorRoundoff = bRotating ? 16 * DBL_EPSILON * Scale : 0.0;
        if (Gap < ExteriorRoundoff && Gap >= -64 * DBL_EPSILON * Scale)
        {
            auto Probe = Query;
            Probe.Shape = Speed::Analytic::EQueryShape::Ray;
            Probe.Start = Lowest + N * .01; Probe.End = Lowest - N * .01;
            Probe.InitialOverlapTolerance = 0;
            const auto Witness = StaticWorld->SweepSingle(Probe);
            if (Witness.bHit && !Witness.bStartPenetrating && !Witness.bSurfaceNormalMayVary &&
                Witness.GeometricErrorBoundCm == 0 && Witness.SurfaceFeatureKind == Speed::EContactFeatureKind::Face &&
                Witness.Normal.Equals(N, 32 * DBL_EPSILON))
            {
                Projection.bContact = true;
                Projection.Hit = Witness;
                Projection.Location = Query.Start + (ExteriorRoundoff - Gap) * N;
            }
        }
    }
    if (!Projection.bContact)
    {
        // Absence of overlap is not separation: an exact nonpenetrating face
        // can still have four finite support witnesses at the current pose.
        // Revalidate them after any owner/wheel projection before clearing it.
        if (bExactPlanarSeed && PublishExactStaticSupport()) return false;
        if (bHasPersistentSeed)
        {
            bHasGroundContact = false;
            bGroundPlaneValid = false;
        }
        return false;
    }

    FVector Correction = FVector(Projection.Location - Query.Start);
    if (bExactPlanarSeed && !Correction.IsZero())
    {
        const double Scale = FMath::Max(1.0, Query.Start.GetAbsMax() + Query.HalfExtent.GetMax());
        const double RoundingFactor = ParentComponent->GetPhysAngularVelocity().IsZero() ? 2.0 : 16.0;
        Correction += FVector(Projection.Hit.Normal) * (RoundingFactor * DBL_EPSILON * Scale);
    }
    const bool bHasCorrection = bExactPlanarSeed ? !Correction.IsZero() : !Correction.IsNearlyZero();
    if (bHasCorrection)
    {
        ParentComponent->SetPhysCOMLocation(
            ParentComponent->GetPhysCOM() + Correction);
    }

    if (bExactPlanarSeed)
    {
        ParentComponent->UpdateSubBodiesKinematics();
        // Position feasibility is separate from the event impulse. Re-solve
        // normal velocities after constrained transport, without replaying a
        // gameplay impact notification or inventing a new preferred face.
        if (bProjectVelocity)
        {
            // The seed/current CCD event can refer to another nearby plane.
            // Velocity feasibility must use the same fresh witness as the pose
            // projection, without overwriting the pending collision event.
            const SHitResult ProjectedHit = SHitResult::FromAnalyticHit(
                Projection.Hit, 0.0f, ProjectionSeed.Component.Get());
            TGuardValue<SHitResult> ProjectedEvent(CurrentHit, ProjectedHit);
            TryResolveExactPlanarImpact(false);
        }
        return bHasCorrection;
    }

    // A newly resolved analytical impact may not yet satisfy the persistence
    // heuristics. Its pose must still be feasible before PostPhysics observers
    // sample it; do not promote that transient contact into ground state here.
    if (!bHasPersistentSeed)
    {
        if (!Correction.IsNearlyZero())
        {
            const FVector Normal = FVector(Projection.Hit.Normal).GetSafeNormal();
            const FVector ContactPoint = FVector(Projection.Hit.Point);
            const float NormalSpeed = FVector::DotProduct(
                GetVelocityAtPoint(ContactPoint), Normal);
            if (!Normal.IsNearlyZero() && NormalSpeed < 0.0f)
            {
                ParentComponent->AddPhysVelocity(-NormalSpeed * Normal);
            }
        }
        return !Correction.IsNearlyZero();
    }

    SHitResult ProjectedHit = SHitResult::FromAnalyticHit(
        Projection.Hit, Delta, GroundComp.Get());
    ProjectedHit.ImpactNormal =
        Speed::QuantizeUnitNormal(ProjectedHit.ImpactNormal);
    ProjectedHit.ImpactPoint =
        Speed::QuantizeVectorCm(ProjectedHit.ImpactPoint, 0.1f);
    ProjectedHit.PenetrationDepth = Speed::QuantizeScalar(
        static_cast<float>(Projection.ResidualPenetrationDepth), 0.01f);
    const FVector Normal = ProjectedHit.ImpactNormal.GetSafeNormal();
    if (Normal.IsNearlyZero())
    {
        return !Correction.IsNearlyZero();
    }

	const FVector PreviousNormal = GroundPlaneN.GetSafeNormal();
    GroundHit = ProjectedHit;
    GroundPlaneN = Normal;
    GroundPlanePointWS = ProjectedHit.ImpactPoint;
    GroundPlaneD = FVector::DotProduct(GroundPlanePointWS, GroundPlaneN);
    bGroundPlaneValid = true;

    // Enforce non-closing velocity at the native analytical witness. For a
	// varying established surface, transport linear velocity between successive
	// tangent spaces instead of applying a fresh point impulse at every sampled
	// OBB witness. The latter injects witness-dependent torque and drains energy
	// when an edge or vertex becomes deepest during curved motion.
    const FVector ContactPoint = FVector(Projection.Hit.Point);
    const float NormalSpeed = FVector::DotProduct(
        GetVelocityAtPoint(ContactPoint), Normal);
    constexpr float ReleaseNormalSpeedCmS = 1.0f;
    if (Correction.IsNearlyZero() &&
        Projection.ResidualPenetrationDepth <= ResidualToleranceCm &&
        NormalSpeed > ReleaseNormalSpeedCmS)
    {
        bHasGroundContact = false;
        bGroundPlaneValid = false;
        return false;
    }
    bool bVelocityProjected = false;
    const bool bDeferVelocityTransportToSubordinateSupport =
		CVarIAmSpeedBoxDeferSupportedTransitionVelocityTransport.GetValueOnAnyThread() != 0 &&
		ParentComponent->HasCompatibleEstablishedStaticSupport(ProjectedHit);
    if (bProjectVelocity && NormalSpeed < 0.0f &&
		!bDeferVelocityTransportToSubordinateSupport)
    {
		const FVector RotationalPointVelocity =
			GetVelocityAtPoint(ContactPoint) -
			ParentComponent->GetPhysCOMVelocity();
		const FVector TransportedLinearVelocity =
			Speed::TransportEstablishedContactVelocity(
				ParentComponent->GetPhysCOMVelocity(),
				RotationalPointVelocity, PreviousNormal, Normal);
		ParentComponent->SetPhysCOMVelocity(TransportedLinearVelocity);
		bVelocityProjected = true;
    }

    return !Correction.IsNearlyZero() || bVelocityProjected;
}

void UBoxSubBody::AcceptHit()
{
    CurrentHit = FutureHit;
    FutureHit = SHitResult();

    if (!CurrentHit.Component.IsValid())
        return;

    if (Cast<USphereSubBody>(CurrentHit.SubBody.Get()))
    {
        SphereHit = CurrentHit;
        LastResolvedSphereHit = CurrentHit;
        LastResolvedSphereHitFrame = ParentComponent ? ParentComponent->NumFrame() : INDEX_NONE;
    }

    UPrimitiveComponent* Comp = CurrentHit.Component.Get();
    if (CurrentHit.Component.IsValid() && CurrentHit.Component->GetMobility() == EComponentMobility::Static)
    {
        uint32& Count = HitCountThisFrame.FindOrAdd(Comp);
        Count++;

        // Ignore only after several hits on the same component in the same frame
        if (Count >= MaxHitsPerComponentPerFrame)
        {
            IgnoredComponents.AddUnique(Comp);
        }
    }
    else
    {
        IgnoredComponents.AddUnique(Comp);
    }

    if (CurrentHit.SubBody.IsValid())
    {
        // if Other sub-body will hit, accept the hit for it too to ensure both sides are in sync and won't try to resolve the same hit again
        auto OtherSubBody = CurrentHit.SubBody.Get();
        if (OtherSubBody->WillHit())
        {
            OtherSubBody->AcceptHit();
        }
    }
}

bool UBoxSubBody::GatherStaticPenetrationHits(TArray<FHitResult>& OutHits) const
{
    OutHits.Reset();
    UWorld* World = GetWorld();
    if (!World)
    {
        return false;
    }

    FCollisionQueryParams QueryParams(SCENE_QUERY_STAT(IAmSpeedBoxPosePenetration), false);
    QueryParams.bFindInitialOverlaps = true;
    if (const AActor* Owner = GetOwner())
    {
        QueryParams.AddIgnoredActor(Owner);
    }
    FCollisionObjectQueryParams ObjectQuery;
    ObjectQuery.AddObjectTypesToQuery(ECC_WorldStatic);
    const Speed::FKinematicState& State = GetKinematicState();
	if (!Speed::Analytic::FStaticWorldQueryAudit::TryCompactAuthorityMulti(
		World,
		State.Location, State.Location, State.Rotation, GetCollisionShape(),
		1ull << static_cast<uint8>(ECC_WorldStatic), OutHits))
	{
		Speed::Analytic::FStaticWorldQueryAudit::RecordLegacySweep();
		World->SweepMultiByObjectType(OutHits, State.Location, State.Location,
			State.Rotation, ObjectQuery, GetCollisionShape(), QueryParams);
	}
	Speed::Analytic::FStaticWorldQueryAudit::RecordMulti(
		Speed::Analytic::EStaticQuerySite::BoxPenetrationProjection,
		State.Location, State.Location, State.Rotation,
		GetCollisionShape(), 1ull << static_cast<uint8>(ECC_WorldStatic), OutHits);

    OutHits.RemoveAll([](const FHitResult& Hit)
    {
        return !Hit.bStartPenetrating || Hit.PenetrationDepth <= 0.0f;
    });
    OutHits.Sort([](const FHitResult& A, const FHitResult& B)
    {
        const UPrimitiveComponent* CompA = A.Component.Get();
        const UPrimitiveComponent* CompB = B.Component.Get();
        const uint32 IdA = CompA ? static_cast<uint32>(CompA->GetUniqueID()) : 0u;
        const uint32 IdB = CompB ? static_cast<uint32>(CompB->GetUniqueID()) : 0u;
        if (IdA != IdB)
        {
            return IdA < IdB;
        }
        if (A.FaceIndex != B.FaceIndex)
        {
            return A.FaceIndex < B.FaceIndex;
        }
        return A.PenetrationDepth > B.PenetrationDepth;
    });
    return !OutHits.IsEmpty();
}

//======================================================
// =============== Sweep Methods =======================
// =====================================================
bool UBoxSubBody::TryBuildPersistentSphereContact(bool bHasSweepSphereHit, const SHitResult& SweepSphereHit, SHitResult& OutHit, float& OutTOI) const
{
    if (!ParentComponent || !LastResolvedSphereHit.bBlockingHit || !LastResolvedSphereHit.SubBody.IsValid())
    {
        return false;
    }

    const int32 CurrentFrame = ParentComponent->NumFrame();
    if (LastResolvedSphereHitFrame == INDEX_NONE || CurrentFrame - LastResolvedSphereHitFrame != 1)
    {
        return false;
    }

    USphereSubBody* OtherSphere = Cast<USphereSubBody>(LastResolvedSphereHit.SubBody.Get());
    if (!OtherSphere || ComponentHasBeenIgnored(*OtherSphere))
    {
        return false;
    }

    if (bHasSweepSphereHit && SweepSphereHit.SubBody.IsValid() && SweepSphereHit.SubBody.Get() != OtherSphere)
    {
        return false;
    }

    constexpr float PersistentSphereMaxSepCm = 0.35f;
    constexpr float PersistentSphereSeparatingSpeedCmS = 1.0f;
    constexpr float ContactPointQuantizationCm = 0.1f;

    const SSBox ThisBox = MakeBox();
    const SSphere OtherSphereShape = OtherSphere->MakeSphere();

    FVector ContactPoint = FVector::ZeroVector;
    const float Sep0 = ThisBox.SphereOBBSeparation(
        ThisBox.Rot,
        ThisBox.AbsoluteCenter(),
        OtherSphereShape.Center,
        OtherSphereShape.Radius,
        &ContactPoint);

    if (Sep0 > PersistentSphereMaxSepCm)
    {
        return false;
    }

    FVector N = (ContactPoint - OtherSphereShape.Center).GetSafeNormal();
    if (N.IsNearlyZero())
    {
        N = LastResolvedSphereHit.ImpactNormal.GetSafeNormal();
    }
    if (N.IsNearlyZero())
    {
        return false;
    }

    const FVector BoxVp = GetVelocityAtPoint(ContactPoint);
    const FVector SphereVp = OtherSphere->GetVelocityAtPoint(ContactPoint);
    const float RelVN = FVector::DotProduct(BoxVp - SphereVp, N);
    if (RelVN > PersistentSphereSeparatingSpeedCmS)
    {
        return false;
    }

    OutHit = SHitResult(
        true,
        Speed::QuantizeVectorCm(ContactPoint, ContactPointQuantizationCm),
        Speed::QuantizeUnitNormal(N),
        0.0f);
    OutHit.bBlockingHit = true;
    OutHit.Component = OtherSphere;
    OutHit.SubBody = OtherSphere;
    OutTOI = 0.0f;

    /*UE_LOG(BoxSubBodyLog, Log,
        TEXT("[PersistentBoxSphereContact] Frame=%d PrevFrame=%d Sep0=%.4f RelVN=%.2f SweepHit=%d SweepTOI=%.6f OldTOI=%.6f NewTOI=0.000000 P=%s N=%s Sphere=%s"),
        CurrentFrame,
        LastResolvedSphereHitFrame,
        Sep0,
        RelVN,
        bHasSweepSphereHit ? 1 : 0,
        bHasSweepSphereHit ? SweepSphereHit.TOI : -1.0f,
        LastResolvedSphereHit.TOI,
        *OutHit.ImpactPoint.ToString(),
        *OutHit.ImpactNormal.ToString(),
        *OtherSphere->GetName());*/

    return true;
}

bool UBoxSubBody::SweepTOI(const float& RemainingDelta, float& OutTOI)
{
    if (!ParentComponent)
    {
        return false;
    }

    float TOI_ground = RemainingDelta;
    float TOI_Sphere = RemainingDelta;
    float TOI_Box = RemainingDelta;
    float TOI_Wheel = RemainingDelta;
    FutureHit = SHitResult();
    SHitResult OldGroundHit = GroundHit;

    SHitResult GroundHitresult;
    SHitResult BoxHitresult;
    SHitResult SphereHitresult;
	SHitResult WheelHitresult;
    bool bHitGround = SweepVsGround(GetWorld(), GroundHitresult, RemainingDelta, TOI_ground);
    bool bHitBox = SweepVsBoxes(GetWorld(), BoxHitresult, RemainingDelta, TOI_Box);
    bool bHitSphere = SweepVsSpheres(GetWorld(), SphereHitresult, RemainingDelta, TOI_Sphere);
    bool bHitWheel = SweepVsWheels(GetWorld(), WheelHitresult, RemainingDelta, TOI_Wheel);
    bool bPersistentSphereContact = false;

    SHitResult PersistentSphereHit;
    float PersistentSphereTOI = RemainingDelta;
    if (TryBuildPersistentSphereContact(bHitSphere, SphereHitresult, PersistentSphereHit, PersistentSphereTOI))
    {
        bPersistentSphereContact = true;
        bHitSphere = true;
        SphereHitresult = PersistentSphereHit;
        TOI_Sphere = PersistentSphereTOI;
    }

    if (!bHitGround && !bHitBox && !bHitSphere && !bHitWheel)
    {
        // IMPORTANT : ground can exist even without a hit (persistent contact)
        if (HasPersistentGroundContact())
        {
            const FVector N = OldGroundHit.ImpactNormal.GetSafeNormal();
            const FVector vP = GetVelocityAtPoint(OldGroundHit.ImpactPoint); // 
            const float vN = FVector::DotProduct(vP, N);

            // DO NOT recreate contact if separating
            if (vN <= 0.f)
            {
                if (!IgnoredComponents.Contains(OldGroundHit.Component))
                {
                    OutTOI = 0.0f;
                    FutureHit = OldGroundHit; // reuse current ground hit
                    GroundHit = OldGroundHit;
                    FutureHit.TOI = 0.0f;
                    return true;
                }
                else
                {
                    GroundHit = OldGroundHit;
                }
            }
        }
        return false;
    }

    OutTOI = RemainingDelta;

    if (bHitGround && TOI_ground <= OutTOI)
    {
        OutTOI = TOI_ground;
		GroundHit = GroundHitresult;
        FutureHit = GroundHit;
    }

    if (bHitSphere && (TOI_Sphere < OutTOI || (bPersistentSphereContact && FMath::IsNearlyEqual(TOI_Sphere, OutTOI))))
    {
        OutTOI = TOI_Sphere;
		SphereHit = SphereHitresult;
        FutureHit = SphereHit;
    }

    if (bHitBox && TOI_Box < OutTOI)
    {
        OutTOI = TOI_Box;
		BoxHit = BoxHitresult;
        FutureHit = BoxHit;
    }

    if (bHitWheel && TOI_Wheel < OutTOI)
    {
        OutTOI = TOI_Wheel;
		WheelHit = WheelHitresult;
        FutureHit = WheelHit;
    }

    return true;
}


bool UBoxSubBody::SweepVsGround(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI)
{
    OutTOI = Delta;
    // GroundHit = SHitResult();

    const FVector Start = Kinematics.Location;

    const FVector End =
        Start +
        Kinematics.Velocity * Delta +
        0.5f * Kinematics.Acceleration * Delta * Delta;

    SHitResult Hit;
    bGroundHitFromSweep = false;
    // SSBox CarBox(Kinematics.Location, BoxExtent, Kinematics.Rotation, Kinematics.Velocity, Kinematics.Acceleration, Kinematics.AngularVelocity, Kinematics.AngularAcceleration);
    // CarBox.DrawDebug(GetWorld());
    bool bFoundGroundHit = InternalSweep(World, Start, End, Hit, Delta);
    if (!bFoundGroundHit)
    {
        // An established plane still matters when the translational query
        // misses: rotation can bring a different vertex into contact.
        if (ParentComponent->GetStaticCollisionWorldForFrame() && CurrentHit.bBlockingHit &&
            !CurrentHit.bSurfaceNormalMayVary && CurrentHit.SourceId != 0 &&
            LastResolvedGroundHitFrame == static_cast<int32>(ParentComponent->NumFrame()))
        { Hit = CurrentHit; bFoundGroundHit = true; }
    }

    auto AddGroundNormal = [&](const FVector& Nnew)
        {
            const FVector nn = Nnew.GetSafeNormal();
            for (const FVector& Nprev : CurrentGroundNormalsWS)
            {
                if (FVector::DotProduct(nn, Nprev) > 0.995f) // ~5°
                    return; // already have essentially same normal
            }
            CurrentGroundNormalsWS.Add(nn);
        };


    // A straight query returns a fraction of displacement, not of accelerated
    // time. Refine exact planar translation against the actual integrator and
    // keep the representable time on the non-penetrating side of the event.
    bool bExactPlanarContact = bFoundGroundHit && ParentComponent &&
        ParentComponent->GetStaticCollisionWorldForFrame() &&
        !Hit.bSurfaceNormalMayVary && Hit.GeometricErrorBoundCm == 0 &&
        Hit.ContactFeatureOther == Speed::EContactFeatureKind::Face;
    if (bExactPlanarContact && !RefineExactPlanarTOI(Hit, Delta)) bFoundGroundHit = false;
    const Speed::IStaticCollisionWorld* StaticWorld = ParentComponent->GetStaticCollisionWorldForFrame();
    const SKinematic& OwnerState = ParentComponent->GetKinematicState();
    if (StaticWorld && (!OwnerState.AngularVelocity.IsZero() || !OwnerState.AngularAcceleration.IsZero()))
    {
        // A fixed-orientation box sweep cannot discover rotation-only first
        // contact. Visit all nearby planes, not merely the first sphere hit;
        // the circumsphere is broad phase only, never collision authority.
        Speed::Analytic::FWorldQuery Broad;
        Broad.Shape = Speed::Analytic::EQueryShape::Sphere;
        Broad.Start = OwnerState.Location;
        Broad.End = OwnerState.Integrate(Delta).Location;
        Broad.Radius = BoxExtent.Size() + (GetLocalOffset() - ParentComponent->GetPhysCenterOfMassLocal()).Size() +
            OwnerState.Acceleration.Size() * (0.125 * double(Delta) * Delta);
        Broad.bApplyCollisionFilter = true; Broad.bAuthorityOnly = true; Broad.bIncludeCompactPatches = true;
        Broad.TraceChannel = static_cast<uint8>(GetCollisionChannel());
        Broad.BlockingObjectTypes = Speed::GetBlockingResponseMask(GetResponseParams().CollisionResponse);
        const USpeedWorldSubsystem* Bridge = World ? World->GetSubsystem<USpeedWorldSubsystem>() : nullptr;
        if (Bridge) StaticWorld->VisitPlanarCandidates(Broad, [&](const Speed::Analytic::FWorldHit& Candidate)
        {
            const auto Source = Bridge->FindAnalyticSourceComponent(Candidate.SourceId);
            if (!Source.IsValid()) return;
            // FromAnalyticHit intentionally discards misses. Broad-phase
            // descriptors are not hits yet, so carry only their plane identity.
            SHitResult Refined;
            Refined.Component = Source.Get();
            Refined.ImpactNormal = Candidate.Normal; Refined.ImpactPoint = Candidate.Point;
            Refined.SourceId = Candidate.SourceId; Refined.SurfaceId = Candidate.SurfaceId;
            Refined.FeatureId = Candidate.FeatureId; Refined.PrimitiveId = Candidate.PrimitiveId;
            Refined.MaterialId = Candidate.MaterialId;
            Refined.ContactFeatureOther = Candidate.SurfaceFeatureKind;
            if (!RefineExactPlanarTOI(Refined, Delta)) return;
            if (!bFoundGroundHit || Refined.TOI < Hit.TOI ||
                (Refined.TOI == Hit.TOI && Refined.SourceId == Hit.SourceId && Refined.PrimitiveId == Hit.PrimitiveId))
            {
                Refined.bHit = Refined.bBlockingHit = true;
                Hit = Refined; bFoundGroundHit = true; bExactPlanarContact = true;
            }
        });
    }
    if (!bFoundGroundHit) return false;
    OutHit = Hit;
    if (bExactPlanarContact)
    {
        // Keep exact geometry: rounding the witness to millimetres moves a
        // non-grid support plane and changes the physical equilibrium height.
        OutTOI = OutHit.TOI;
        bGroundHitFromSweep = true;
        AddGroundNormal(OutHit.ImpactNormal);
        return true;
    }
    OutHit.ImpactNormal = Speed::QuantizeUnitNormal(OutHit.ImpactNormal);
    OutHit.TOI = Speed::QuantizeScalar(Hit.TOI, 1e-5f);
	OutTOI = OutHit.TOI;
    OutHit.ImpactPoint = Speed::QuantizeVectorCm(Hit.ImpactPoint, 0.1f); // 1 mm
    OutHit.PenetrationDepth = Speed::QuantizeScalar(Hit.PenetrationDepth, 0.01f);
    bGroundHitFromSweep = true;
    AddGroundNormal(OutHit.ImpactNormal);
    return true;
}

bool UBoxSubBody::SweepVsSpheres(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI)
{
	return IBoxSweeper::SweepVsSpheres(World, OutHit, Delta, OutTOI);
}

bool UBoxSubBody::SweepVsWheels(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI)
{
	return IBoxSweeper::SweepVsWheels(World, OutHit, Delta, OutTOI);
}

bool UBoxSubBody::SweepVsBoxes(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI)
{
	return IBoxSweeper::SweepVsBoxes(World, OutHit, Delta, OutTOI);
}

// ============================================================================
// ======= ResolveHit Methods =================================================
// ============================================================================

void UBoxSubBody::ResolveCurrentHitPrv(const float& delta, const float& SimTime)
{
    if (!CurrentHit.bBlockingHit || !ParentComponent)
        return;

    if (TryResolveExactFaceImpact()) return;
    if (TryResolveExactPlanarImpact()) return;

	RegisterCurrentHitAsConstraint();

    USphereSubBody* Sphere = Cast<USphereSubBody>(CurrentHit.Component.Get());
    UBoxSubBody* Box = Cast<UBoxSubBody>(CurrentHit.Component.Get());

    if (Sphere)
    {
        ResolveHitVsSphere(*Sphere, delta);
    }
    else if (Box)
    {
        ResolveHitVsBox(*Box, delta);
    }
    else if (CurrentHit.Component.IsValid() &&
        CurrentHit.Component->GetCollisionObjectType() == ECC_WorldStatic)
    {
        ResolveHitVsGround(delta);
    }
}


// ============================================================================
//  ResolveHitVsGround
//     - Orchestrates: classify -> event solve -> update persistence -> apply support.
//     - Handles concave rule consistently.
// ============================================================================
void UBoxSubBody::ResolveHitVsGround(const float& Dt)
{
    if (!ParentComponent) return;

    const int32 CurrentFrame = ParentComponent->NumFrame();
    LastResolvedGroundHitFrame = CurrentFrame;

    CompositeGroundNormal = BuildCompositeGroundNormal();

    const FVector RawN = CurrentHit.ImpactNormal.GetSafeNormal();
    const float RawUpDot = FVector::DotProduct(RawN, FVector::UpVector);
    const FVector VelBeforeResolve = ParentComponent->GetPhysCOMVelocity();
    const FVector AngBeforeResolve = ParentComponent->GetPhysAngularVelocity();

    SHitResult EffectiveHit = CurrentHit;
    FVector EffectiveN = RawN;
    float EffectiveUpDot = RawUpDot;

    bool bPromotedWallToSupport = false;

    if (RawUpDot < 0.85f)
    {
        SHitResult PromotedHit;
		// Check if we can promote this wall/gutter hit to a support contact by reclassifying
        // it with a more vertical normal (composite of current hit normal and other ground contacts normals)
        if (ShouldPromoteWallHitToSupport(CurrentHit, RawN, RawUpDot, Dt, PromotedHit))
        {
            EffectiveHit = PromotedHit;
            EffectiveN = EffectiveHit.ImpactNormal.GetSafeNormal();
            EffectiveUpDot = FVector::DotProduct(EffectiveN, FVector::UpVector);
            bPromotedWallToSupport = true;
        }
    }

    constexpr float DirectSupportUpDot = 0.97f;
    constexpr float CandidateSupportUpDot = 0.85f;
    const float RoofSlideSupportMinUpDot =
        CVarIAmSpeedRoofSlideSupportMinUpDot.GetValueOnAnyThread();

    const bool bRoofSurfaceTraversalActive = HasRoofSurfaceTraversalSupport();
    const bool bSameRoofSurfaceComponent =
        EffectiveHit.Component.IsValid() &&
        GroundComp.IsValid() &&
        EffectiveHit.Component.Get() == GroundComp.Get();
    if (bRoofSurfaceTraversalActive &&
        bSameRoofSurfaceComponent &&
        (EffectiveUpDot < 0.f ||
            FVector::DotProduct(EffectiveN, GroundPlaneN.GetSafeNormal()) < 0.8f) &&
        bGroundPlaneValid &&
        GroundPlaneN.Z > 0.f)
    {
        EffectiveN = GroundPlaneN.GetSafeNormal();
        EffectiveHit.ImpactNormal = EffectiveN;
        EffectiveUpDot = FVector::DotProduct(EffectiveN, FVector::UpVector);
    }
    const bool bIsDirectSupportNormal = EffectiveUpDot >= DirectSupportUpDot;
    const FVector HitboxUpVector = Kinematics.Rotation.GetUpVector();
    const bool bRoofFacingSurface =
        FVector::DotProduct(HitboxUpVector, EffectiveN) < -0.45f;
    const bool bUpsideDownRoofSlideCandidate =
        (bRoofFacingSurface &&
            EffectiveUpDot >= RoofSlideSupportMinUpDot &&
            EffectiveUpDot < DirectSupportUpDot) ||
        bRoofSurfaceTraversalActive;
    const bool bIsCandidateSupport =
        EffectiveUpDot >= CandidateSupportUpDot ||
        bUpsideDownRoofSlideCandidate;
    const bool bHasLatchedEdgeSupport =
        bEdgeSupportLatched &&
        LatchedEdgeContactsLS.Num() == 2;
    const bool bHasPreviousEdgeSupport = PrevGroundContactsLS.Num() == 2;

    bool bIsSupportingContact = false;
    int32 PreviewSupportContacts = 0;
    TArray<FVector> PreviewPts;

    if (bIsCandidateSupport)
    {
        BuildSupportManifoldFromNormal(
            EffectiveN,
            Kinematics.Location,
            Kinematics.Rotation,
            BoxExtent,
            PreviewPts,
            0.5f
        );

        PreviewSupportContacts = PreviewPts.Num();

        const bool bSupportManifoldForResolution =
            PreviewPts.Num() >= 2 ||
            (PreviewPts.Num() == 1 && (bIsDirectSupportNormal || bUpsideDownRoofSlideCandidate)) ||
            bHasLatchedEdgeSupport ||
            bHasPreviousEdgeSupport;

        bIsSupportingContact =
            bSupportManifoldForResolution &&
            (bIsDirectSupportNormal || EffectiveUpDot >= CandidateSupportUpDot || bUpsideDownRoofSlideCandidate);
    }

    constexpr float VN_EPS = 1.0f;
    const float vN_atImpact =
        FVector::DotProduct(GetVelocityAtPoint(EffectiveHit.ImpactPoint), EffectiveN);

#if !UE_BUILD_SHIPPING
    if (CVarIAmSpeedAutoRecoverContactDebug.GetValueOnAnyThread() != 0 &&
        (bIsCandidateSupport ||
            PreviewSupportContacts > 0 ||
            bHasLatchedEdgeSupport ||
            bHasPreviousEdgeSupport ||
            bGroundPlaneValid ||
            bEdgeSupportLatched))
    {
        UE_LOG(
            BoxSubBodyLog,
            Log,
            TEXT("[ARContact.ResolveGround] Frame=%d RawUpDot=%.3f EffUpDot=%.3f Promoted=%d RoofSlide=%d Candidate=%d Supporting=%d Preview=%d vN=%.2f Sweep=%d Current=%d PrevLS=%d Latched=%d LatchedLS=%d PlaneValid=%d N=%s HitP=%s"),
            ParentComponent->NumFrame(),
            RawUpDot,
            EffectiveUpDot,
            bPromotedWallToSupport ? 1 : 0,
            bUpsideDownRoofSlideCandidate ? 1 : 0,
            bIsCandidateSupport ? 1 : 0,
            bIsSupportingContact ? 1 : 0,
            PreviewSupportContacts,
            vN_atImpact,
            bGroundHitFromSweep ? 1 : 0,
            CurrentGroundContactsWS.Num(),
            PrevGroundContactsLS.Num(),
            bEdgeSupportLatched ? 1 : 0,
            LatchedEdgeContactsLS.Num(),
            bGroundPlaneValid ? 1 : 0,
            *EffectiveN.ToString(),
            *EffectiveHit.ImpactPoint.ToString());
    }
#endif

	// Debug log at start of hit resolution
    /*UE_LOG(BoxSubBodyLog, Log,
        TEXT("[CCDBox] Start of Hit vs Ground for frame = %d, vN_start = %.2f cm/s, bIsSupportingContact = %d, Parent Kinematic = (%s)"),
        ParentComponent->NumFrame(),
        vN_atImpact,
        bIsSupportingContact ? 1 : 0,
        *ParentComponent->GetKinematicState().ToString());*/

// #if !UE_BUILD_SHIPPING
    /*const bool bLogContactDecision =
        bIsSupportingContact ||
        bPromotedWallToSupport ||
        FMath::Abs(vN_atImpact) > 50.f ||
        HasPersistentGroundContact() ||
        bGroundPlaneValid ||
        bEdgeSupportLatched;

    if (bLogContactDecision)
    {
        const int32 FramesSinceWall =
            LastWallOrGutterFrame == INDEX_NONE
            ? INDEX_NONE
            : ParentComponent->NumFrame() - LastWallOrGutterFrame;

        const bool bRecentlyHadWallOrGutter =
            FramesSinceWall != INDEX_NONE && FramesSinceWall <= 8;
        UE_LOG(BoxSubBodyLog, Log,
            TEXT("[CCDBoxDecision] Frame=%d Branch=%s RawUpDot=%.4f EffUpDot=%.4f Promoted=%d PreviewContacts=%d vN=%.2f RawN=%s EffN=%s HitP=%s TOI=%.6f Sweep=%d Persistent=%d PlaneValid=%d PlaneN=%s Contacts=%d Normals=%d PrevLS=%d Latched=%d LatchedLS=%d LatchFrame=%d Comp=%s,"
                "RecentlyHadWallOrGutter=%d FramesSinceWall=%d LastWallOrGutterUpDot=%.4f LastWallOrGutterN=%s"),
            ParentComponent->NumFrame(),
            bIsSupportingContact ? TEXT("Support") : TEXT("Wall"),
            RawUpDot,
            EffectiveUpDot,
            bPromotedWallToSupport ? 1 : 0,
            PreviewSupportContacts,
            vN_atImpact,
            *RawN.ToString(),
            *EffectiveN.ToString(),
            *EffectiveHit.ImpactPoint.ToString(),
            EffectiveHit.TOI,
            bGroundHitFromSweep ? 1 : 0,
            HasPersistentGroundContact() ? 1 : 0,
            bGroundPlaneValid ? 1 : 0,
            *GroundPlaneN.ToString(),
            CurrentGroundContactsWS.Num(),
            CurrentGroundNormalsWS.Num(),
            PrevGroundContactsLS.Num(),
            bEdgeSupportLatched ? 1 : 0,
            LatchedEdgeContactsLS.Num(),
            EdgeSupportLatchFrame,
            EffectiveHit.Component.IsValid() ? *EffectiveHit.Component->GetName() : TEXT("None"),
            bRecentlyHadWallOrGutter ? 1 : 0,
            FramesSinceWall,
            LastWallOrGutterUpDot,
            * LastWallOrGutterN.ToString());
    }*/
// #endif

    bool bDidDirectSupportSolve = false;
    constexpr float RoofTraversalTargetNormalSpeed = -5.f;

    const bool bAnalyticSemanticSurfaceContinuation =
        bRoofSurfaceTraversalActive &&
        bSameRoofSurfaceComponent &&
        EffectiveHit.bSurfaceNormalMayVary &&
        EffectiveHit.SourceId != 0 && EffectiveHit.SurfaceId != 0 &&
        EffectiveHit.SourceId == EstablishedSupportSourceId &&
        EffectiveHit.SurfaceId == EstablishedSupportSurfaceId &&
        FVector::DotProduct(EffectiveN, GroundPlaneN.GetSafeNormal()) > 0.0f;
    if (bAnalyticSemanticSurfaceContinuation)
    {
        // A smooth provider transition on one semantic surface is a
        // continuation of the existing unilateral contact. Solving every OBB
        // witness as a fresh multi-point impact removes tangential energy and
        // makes the result depend on solver iteration count. The remaining
        // substep is instead handled by the native established-contact
        // projection in IntegrateKinematics.
        GroundHit = EffectiveHit;
        GroundPlaneN = EffectiveN;
        GroundPlanePointWS = EffectiveHit.ImpactPoint;
        GroundPlaneD = FVector::DotProduct(GroundPlanePointWS, GroundPlaneN);
        GroundComp = EffectiveHit.Component;
        bGroundPlaneValid = true;
        bHasGroundContact = true;
        CurrentGroundContactsWS = PreviewPts;
        // Do not move the body from inside the impact callback. The remaining
        // CCD interval must first advance on the continued semantic surface;
        // established-contact projection owns subsequent substeps, and the
        // canonical frame-boundary pass handles a contact acquired at the last
        // TOI before observers can sample it.
        bRoofSurfaceTraversalLatched = true;
        LastRoofSurfaceContactFrame = CurrentFrame;
        RoofSurfaceComp = EffectiveHit.Component;
#if !UE_BUILD_SHIPPING
        if (CVarIAmSpeedAutoRecoverContactDebug.GetValueOnAnyThread() != 0)
        {
            UE_LOG(
                BoxSubBodyLog,
                Log,
                TEXT("[AnalyticContactContinuation] Frame=%d Source=%016llX Surface=%016llX Group=%016llX N=%s Velocity=%s"),
                CurrentFrame,
                EffectiveHit.SourceId,
                EffectiveHit.SurfaceId,
                EffectiveHit.CanonicalGroupId,
                *EffectiveN.ToString(),
                *ParentComponent->GetPhysCOMVelocity().ToString());
        }
#endif
        return;
    }

    if (vN_atImpact >= VN_EPS)
    {
        if (bRoofSurfaceTraversalActive)
        {
            const float SeparationSpeed = FVector::DotProduct(ParentComponent->GetPhysCOMVelocity(), EffectiveN);
            if (SeparationSpeed > RoofTraversalTargetNormalSpeed)
            {
                ParentComponent->AddPhysVelocity(
                    (RoofTraversalTargetNormalSpeed - SeparationSpeed) * EffectiveN);
            }
        }
        IAmSpeedApplyGutterSlideDamping(
            ParentComponent,
            IsMainSubBody(),
            EffectiveN,
            Dt,
            bUpsideDownRoofSlideCandidate,
            TEXT("SeparatingRoofSlide"));

        UpdatePersistentGroundContact(Dt);

        if (HasPersistentGroundContact() && bIsSupportingContact && !bGroundHitFromSweep)
        {
            ApplyPersistentSupportConstraint(Dt);
        }

        return;
    }

    if (bIsSupportingContact)
    {
        ResolveDirectGroundSupport(Dt, EffectiveHit);
        bDidDirectSupportSolve = true;
        if (bRoofSurfaceTraversalActive)
        {
            const float SeparationSpeed =
                FVector::DotProduct(ParentComponent->GetPhysCOMVelocity(), EffectiveN);
            if (SeparationSpeed > RoofTraversalTargetNormalSpeed)
            {
                ParentComponent->AddPhysVelocity(
                    (RoofTraversalTargetNormalSpeed - SeparationSpeed) * EffectiveN);
            }
        }
    }
    else
    {
        ResolveWallOrGutter(Dt, EffectiveHit);

        const bool bHasFacePersistentSupport = PrevGroundContactsLS.Num() >= 3;
        const bool bSteepWallOrGutterHit = EffectiveUpDot < 0.5f;
        const bool bPersistentPlaneDisagreesWithHit =
            bGroundPlaneValid &&
            FVector::DotProduct(GroundPlaneN.GetSafeNormal(), EffectiveN) < 0.5f;
        const bool bUprightHitbox =
            FVector::DotProduct(HitboxUpVector, FVector::UpVector) > 0.2f;

        if (bHasFacePersistentSupport &&
            bSteepWallOrGutterHit &&
            bPersistentPlaneDisagreesWithHit &&
            bUprightHitbox)
        {
#if !UE_BUILD_SHIPPING
            if (CVarIAmSpeedAutoRecoverContactDebug.GetValueOnAnyThread() != 0)
            {
                UE_LOG(
                    BoxSubBodyLog,
                    Log,
                    TEXT("[ARContact.PersistentInvalidate] Frame=%d Reason=UprightSteepWallVsFace PrevLS=%d HitUpDot=%.3f PlaneN=%s HitN=%s"),
                    ParentComponent->NumFrame(),
                    PrevGroundContactsLS.Num(),
                    EffectiveUpDot,
                    *GroundPlaneN.ToString(),
                    *EffectiveN.ToString());
            }
#endif

            bHasGroundContact = false;
            bGroundPlaneValid = false;
            bGroundContactStable = false;
            StableTime = 0.f;
            CurrentGroundContactsWS.Reset();
            PrevGroundContactsLS.Reset();
            LatchedEdgeContactsLS.Reset();
            bEdgeSupportLatched = false;
        }
    }

    UpdatePersistentGroundContact(Dt, bDidDirectSupportSolve);

    if (!bDidDirectSupportSolve && HasPersistentGroundContact() && bIsSupportingContact)
    {
        ApplyPersistentSupportConstraint(Dt);
    }

    PrevContactNormal = EffectiveN;
    bHadContactPrevFrame = true;

	// Debug log at end of hit resolution
    /*const FVector vImpactEnd = GetVelocityAtPoint(EffectiveHit.ImpactPoint);
    const float vN_end = FVector::DotProduct(vImpactEnd, EffectiveN);
    const FVector VelAfterResolve = ParentComponent->GetPhysCOMVelocity();
    const FVector AngAfterResolve = ParentComponent->GetPhysAngularVelocity();
    if ((VelAfterResolve - VelBeforeResolve).Size() > 5.f ||
        (AngAfterResolve - AngBeforeResolve).Size() > 0.01f ||
        bIsSupportingContact ||
        bPromotedWallToSupport)
    {
        UE_LOG(BoxSubBodyLog, Log,
            TEXT("[CCDBoxResolveDelta] Frame=%d Branch=%s RawUpDot=%.4f EffUpDot=%.4f Promoted=%d ")
            TEXT("VelBefore=%s VelAfter=%s dV=%s AngBefore=%s AngAfter=%s dW=%s"),
            ParentComponent->NumFrame(),
            bIsSupportingContact ? TEXT("Support") : TEXT("Wall"),
            RawUpDot,
            EffectiveUpDot,
            bPromotedWallToSupport ? 1 : 0,
            *VelBeforeResolve.ToString(),
            *VelAfterResolve.ToString(),
            *(VelAfterResolve - VelBeforeResolve).ToString(),
            *AngBeforeResolve.ToString(),
            *AngAfterResolve.ToString(),
            *(AngAfterResolve - AngBeforeResolve).ToString());
    }*/
}


void UBoxSubBody::ResolveHitVsSphere(USphereSubBody& Sphere, const float& delta)
{
    // --- Box kinematics at TOI ---
    // Kinematics = GetKinematicsFromOwner(ParentComponent->NumFrame());
    const SKinematic& BoxKS = Kinematics; // already correct
    SSBox BBox = MakeBox();

    // --- Sphere kinematics at TOI ---
    const SKinematic& SphereKS0 =
        Sphere.GetKinematicState();
    FFakePhysicsImpactContext FakePhysicsContext;
    FakePhysicsContext.SelfParentKinematics = ParentComponent->GetKinematicState();
    if (const ISpeedComponent* SphereParent = Sphere.GetParentComponent())
    {
        FakePhysicsContext.OtherParentKinematics = SphereParent->GetKinematicState();
    }
    // const SKinematic SphereKSAtTOI = SphereKS0.Integrate(TimePassed);

    // Current frame
    const unsigned frame = ParentComponent->NumFrame();

    // -------- Overlap branch (Box - Sphere) --------
    /*
    {
        const float epsTOI = 1e-6f;

        FVector Cp;
        const float sep = BBox.SphereOBBSeparation(BBox.Rot, BBox.WorldCenter, SphereKS0.Location, Sphere.GetRadius(), &Cp);
        const float pen = FMath::Max(0.f, -sep);

        if (pen > MinSlopCm)
        {
            FVector N = (SphereKS0.Location - Cp); // Box -> Sphere
            if (!N.Normalize())
                N = BBox.UpVector();

            const FVector P = SphereKS0.Location - N * Sphere.GetRadius();
            ISpeedComponent* OtherComp = Sphere.GetParentComponent(); // needs access; ok if you add getter

            SolveOverlap(
                *ParentComponent, GetMass(), ComputeWorldInvInertiaTensor(), BoxKS,
                OtherComp, Sphere.GetMass(), Sphere.ComputeWorldInvInertiaTensor(), SphereKS0,
                P, N,
                pen,
                0.05f, 0.8f,
                GetImpactThreshold()
            );
            if (IsFakePhysicsEnabled())
            {
                ApplyFakePhysicsOn(Sphere, CurrentHit, delta);
            }
            if (Sphere.IsFakePhysicsEnabled())
            {
                Sphere.ApplyFakePhysicsOn(*this, Sphere.GetHit(), delta);
            }
            return;
        }
    }*/

    const FVector BoxLocalContactPoint = BoxKS.Rotation.UnrotateVector(CurrentHit.ImpactPoint - BoxKS.Location);
    const FVector BoxLocalContactNormal = BoxKS.Rotation.UnrotateVector(CurrentHit.ImpactNormal.GetSafeNormal());
    const float MixedRestitution = ResolveSphereBoxRestitution(
        Sphere,
        *this,
        MixRestitution(Sphere.GetRestitution(), GetRestitution(), EMixMode::E_Max));
    const float MixedFriction = ResolveSphereBoxFriction(Sphere, *this,
        MixFriction(Sphere.GetDynamicFriction(), GetDynamicFriction(), EMixMode::E_Max));
	const float RelativeNormalSpeedForSphere = FMath::Abs(FVector::DotProduct(
		IAmSpeedVelocityAtPointFromKS(SphereKS0, CurrentHit.ImpactPoint) -
		IAmSpeedVelocityAtPointFromKS(BoxKS, CurrentHit.ImpactPoint),
		CurrentHit.ImpactNormal.GetSafeNormal()));

    const bool bUsePostNormalFriction =
        UsesPostNormalFrictionImpulse() || Sphere.UsesPostNormalFrictionImpulse();
    const SKinematic& ImpulseBoxKS = bUsePostNormalFriction
        ? FakePhysicsContext.SelfParentKinematics
        : BoxKS;
    const SKinematic& ImpulseSphereKS = bUsePostNormalFriction
        ? FakePhysicsContext.OtherParentKinematics
        : SphereKS0;
    FVector ImpThis, ImpOther;
    bool ok = Speed::SImpulseSolver::ComputeCollisionImpulse(
        CurrentHit.ImpactPoint,
		CurrentHit.ImpactNormal,   // Sphere -> Box

        ImpulseBoxKS, GetMass(), ComputeWorldInvInertiaTensor(),
        ImpulseSphereKS, Sphere.GetMass(), Sphere.ComputeWorldInvInertiaTensor(),

		MixedRestitution,
        MixedFriction,
        ImpThis,
        ImpOther,
        GetImpactThreshold(),
        (UsesCoupledContactImpulse() || Sphere.UsesCoupledContactImpulse()) &&
        Sphere.AllowsCoupledContactImpulseFor(SphereKS0) &&
        AllowsCoupledContactNormal(BoxLocalContactNormal),
        bUsePostNormalFriction
    );

    if (!ok)
        return;

#if !UE_BUILD_SHIPPING
    if (CVarIAmSpeedCollisionDebugSphereBoxImpulse.GetValueOnAnyThread() != 0)
    {
        const FVector N = CurrentHit.ImpactNormal.GetSafeNormal();
        const FVector BoxVp = IAmSpeedVelocityAtPointFromKS(BoxKS, CurrentHit.ImpactPoint);
        const FVector SphereVp = IAmSpeedVelocityAtPointFromKS(SphereKS0, CurrentHit.ImpactPoint);
        const FVector RelVp = BoxVp - SphereVp;
        const float RelNormalSpeed = FVector::DotProduct(RelVp, N);
        const FVector LocalNormal = BoxLocalContactNormal;
        const FVector BoxForward = BoxKS.Rotation.GetForwardVector();
        const FVector BoxRight = BoxKS.Rotation.GetRightVector();
        const FVector BoxUp = BoxKS.Rotation.GetUpVector();

        UE_LOG(
            BoxSubBodyLog,
            Warning,
            TEXT("[SphereBoxImpulse][BoxResolve] Frame=%d Delta=%.6f TOI=%.6f Box=%s Sphere=%s Point=%s LocalPoint=%s Normal(SphereToBox)=%s LocalNormal=%s Restitution=%.3f Friction=%.3f Threshold=%.3f RelN(BoxMinusSphere)=%.3f RelVp=%s BoxImpulse=%s SphereImpulse=%s BoxLoc=%s BoxRot=%s BoxForward=%s BoxRight=%s BoxUp=%s BoxV=%s BoxW=%s SphereLoc=%s SphereV=%s SphereW=%s BoxVp=%s SphereVp=%s BoxMass=%.3f SphereMass=%.3f"),
            ParentComponent ? ParentComponent->NumFrame() : -1,
            delta,
            CurrentHit.TOI,
            GetOwner() ? *GetOwner()->GetName() : TEXT("<NoBoxOwner>"),
            Sphere.GetOwner() ? *Sphere.GetOwner()->GetName() : TEXT("<NoSphereOwner>"),
            *CurrentHit.ImpactPoint.ToString(),
            *BoxLocalContactPoint.ToString(),
            *N.ToString(),
            *LocalNormal.ToString(),
			MixedRestitution,
            MixedFriction,
            GetImpactThreshold(),
            RelNormalSpeed,
            *RelVp.ToString(),
            *ImpThis.ToString(),
            *ImpOther.ToString(),
            *BoxKS.Location.ToString(),
            *BoxKS.Rotation.ToString(),
            *BoxForward.ToString(),
            *BoxRight.ToString(),
            *BoxUp.ToString(),
            *BoxKS.Velocity.ToString(),
            *BoxKS.AngularVelocity.ToString(),
            *SphereKS0.Location.ToString(),
            *SphereKS0.Velocity.ToString(),
            *SphereKS0.AngularVelocity.ToString(),
            *BoxVp.ToString(),
            *SphereVp.ToString(),
            GetMass(),
            Sphere.GetMass());
    }
#endif

    // --- Debug ---
    /*UE_LOG(BoxSubBodyLog, Log, TEXT("[%s(%d)][CCDBox] Resolving Sphere hit for frame = %d, with ImpactPoint = %s, ImpactNormal = %s, at TOI = %fs"), *GetOwner()->GetName(), GetOwnerRole(),
        ParentComponent->NumFrame(), *CurrentHit.ImpactPoint.ToString(), *CurrentHit.ImpactNormal.ToString(), CurrentHit.TOI);
    UE_LOG(BoxSubBodyLog, Log, TEXT("[%s(%d)][CCDBox] Box KS at TOI: %s"), *GetOwner()->GetName(), GetOwnerRole(),
        *BoxKS.ToString());
    UE_LOG(BoxSubBodyLog, Log, TEXT("[%s(%d)][CCDBox] Sphere KS at TOI: %s"), *GetOwner()->GetName(), GetOwnerRole(),
        *SphereKS0.ToString());
    UE_LOG(BoxSubBodyLog, Log, TEXT("[%s(%d)][CCDBox] Box applies impulse %s on Sphere at TOI = %fs"), *GetOwner()->GetName(), GetOwnerRole(),
        *ImpOther.ToString(), CurrentHit.TOI);*/

#if !UE_BUILD_SHIPPING
    const ISpeedComponent* SphereParentForDebug = Sphere.GetParentComponent();
    const FVector BoxCOMVPre = ParentComponent ? ParentComponent->GetPhysCOMVelocity() : FVector::ZeroVector;
    const FVector BoxWPre = ParentComponent ? ParentComponent->GetPhysAngularVelocity() : FVector::ZeroVector;
    const FVector SphereCOMVPre = SphereParentForDebug ? SphereParentForDebug->GetPhysCOMVelocity() : FVector::ZeroVector;
    const FVector SphereWPre = SphereParentForDebug ? SphereParentForDebug->GetPhysAngularVelocity() : FVector::ZeroVector;
#endif
        // --- Apply impulses ---
    ApplyImpulse(ImpThis, CurrentHit.ImpactPoint);
	Sphere.ApplySphereBoxImpulse(
		ImpOther,
		CurrentHit.ImpactPoint,
		CurrentHit.ImpactNormal,
		RelativeNormalSpeedForSphere,
		SphereKS0,
		BoxKS);

#if !UE_BUILD_SHIPPING
    if (CVarIAmSpeedCollisionDebugSphereBoxImpulse.GetValueOnAnyThread() != 0)
    {
        const FVector BoxCOMVPost = ParentComponent ? ParentComponent->GetPhysCOMVelocity() : FVector::ZeroVector;
        const FVector BoxWPost = ParentComponent ? ParentComponent->GetPhysAngularVelocity() : FVector::ZeroVector;
        const FVector SphereCOMVPost = SphereParentForDebug ? SphereParentForDebug->GetPhysCOMVelocity() : FVector::ZeroVector;
        const FVector SphereWPost = SphereParentForDebug ? SphereParentForDebug->GetPhysAngularVelocity() : FVector::ZeroVector;

        UE_LOG(
            BoxSubBodyLog,
            Warning,
            TEXT("[SphereBoxImpulse][BoxResolvePost] Frame=%d Box=%s Sphere=%s Point=%s BoxCOMVPost=%s BoxWPost=%s SphereCOMVPost=%s SphereWPost=%s DeltaBoxCOMV=%s DeltaBoxW=%s DeltaSphereCOMV=%s DeltaSphereW=%s BoxImpulse=%s SphereImpulse=%s"),
            ParentComponent ? ParentComponent->NumFrame() : -1,
            GetOwner() ? *GetOwner()->GetName() : TEXT("<NoBoxOwner>"),
            Sphere.GetOwner() ? *Sphere.GetOwner()->GetName() : TEXT("<NoSphereOwner>"),
            *CurrentHit.ImpactPoint.ToString(),
            *BoxCOMVPost.ToString(),
            *BoxWPost.ToString(),
            *SphereCOMVPost.ToString(),
            *SphereWPost.ToString(),
            *(BoxCOMVPost - BoxCOMVPre).ToString(),
            *(BoxWPost - BoxWPre).ToString(),
            *(SphereCOMVPost - SphereCOMVPre).ToString(),
            *(SphereWPost - SphereWPre).ToString(),
            *ImpThis.ToString(),
            *ImpOther.ToString());
    }
#endif

    if (IsFakePhysicsEnabled())
    {
		// (ParentComponent->NumFrame()); // update kinematics to current frame before applying fake physics
		// Sphere.UpdateKinematicsFromOwner(ParentComponent->NumFrame());
        ApplyFakePhysicsOn(Sphere, CurrentHit, delta, &FakePhysicsContext);
    }
    if (Sphere.IsFakePhysicsEnabled())
    {
        // UpdateKinematicsFromOwner(ParentComponent->NumFrame()); // update kinematics to current frame before applying fake physics
        // Sphere.UpdateKinematicsFromOwner(ParentComponent->NumFrame());
        SHitResult InvertedHit = CurrentHit;
        InvertedHit.ImpactNormal *= -1.f;
        const FFakePhysicsImpactContext InvertedContext = FakePhysicsContext.Inverted();
        Sphere.ApplyFakePhysicsOn(*this, InvertedHit, delta, &InvertedContext);
    }

	// Handle micro-oscillations
	Sphere.HandleMicroOscillation();
	Sphere.ProjectOutOfBox(*this);
	if (Sphere.ShouldMaintainSphereBoxContact(
		RelativeNormalSpeedForSphere,
		CurrentHit.ImpactNormal,
		FakePhysicsContext.OtherParentKinematics,
		FakePhysicsContext.SelfParentKinematics))
	{
		if (UWorld* World = GetWorld())
		{
			if (USpeedWorldSubsystem* SpeedWorld = World->GetSubsystem<USpeedWorldSubsystem>())
			{
				SpeedWorld->RegisterDynamicContactPair(
					Sphere, *this, CurrentHit.ImpactPoint, -CurrentHit.ImpactNormal,
					RelativeNormalSpeedForSphere);
			}
		}
	}

    // --- Gameplay hooks ---
    ParentComponent->RcvImpactOnSubBody(*this, CurrentHit.ImpactPoint);
}

void UBoxSubBody::ResolveHitVsBox(
    UBoxSubBody& OtherBox,
    const float& delta
)
{
    const SKinematic& KA = Kinematics; // current

    // KB courant
    const SKinematic& KB = OtherBox.Kinematics; // si sync (sinon GetKinematicsFromOwner(frame))

    const FVector P = CurrentHit.ImpactPoint;
    const FVector N = CurrentHit.ImpactNormal.GetSafeNormal(); // Other -> This (entrant dans This)

    // -------- Overlap branch (Box - Box) --------
    // EstimateOBBOverlapAlongNormal + SolveOverlap
    {
        SSBox A = MakeBox();
        SSBox B = OtherBox.MakeBox();
        const float pen = SSBox::EstimateOBBOverlapAlongNormal(A, B, N);
        if (pen > MinSlopCm)
        {
            ISpeedComponent* OtherComp = OtherBox.GetParentComponent();
            SolveOverlap(*ParentComponent, GetMass(), ComputeWorldInvInertiaTensor(), KA,
                OtherComp, OtherBox.GetMass(), OtherBox.ComputeWorldInvInertiaTensor(), KB,
                P, N, pen, MinSlopCm, 0.8f, GetImpactThreshold());
            if (IsFakePhysicsEnabled())
            {
                ApplyFakePhysicsOn(OtherBox, CurrentHit, delta);
            }
            if (OtherBox.IsFakePhysicsEnabled())
            {
                OtherBox.ApplyFakePhysicsOn(*this, OtherBox.GetHit(), delta);
            }
            return;
        }
    }

    FVector ImpA, ImpB;
    const float e = MixRestitution(GetRestitution(), OtherBox.GetRestitution(), EMixMode::E_Max);
    const float mu = MixFriction(GetDynamicFriction(), OtherBox.GetDynamicFriction(), EMixMode::E_Max);

    if (!Speed::SImpulseSolver::ComputeCollisionImpulse(
        P, N,
        KA, GetMass(), ComputeWorldInvInertiaTensor(),
        KB, OtherBox.GetMass(), OtherBox.ComputeWorldInvInertiaTensor(),
        e, mu,
        ImpA, ImpB,
        GetImpactThreshold()))
    {
        return;
    }

    ApplyImpulse(ImpA, P);
    OtherBox.ApplyImpulse(ImpB, P);

    // Fake physics both sides
    if (IsFakePhysicsEnabled())
    {
        // UpdateKinematicsFromOwner(ParentComponent->NumFrame()); // update kinematics to current frame before applying fake physics
		// OtherBox.UpdateKinematicsFromOwner(ParentComponent->NumFrame());
        ApplyFakePhysicsOn(OtherBox, CurrentHit, delta);
    }
    if (OtherBox.IsFakePhysicsEnabled())
    {
        SHitResult InverseHit = CurrentHit;
        InverseHit.ImpactNormal *= -1.f; // invert normal for other box's perspective
		// UpdateKinematicsFromOwner(ParentComponent->NumFrame()); // update kinematics to current frame before applying fake physics
		// OtherBox.UpdateKinematicsFromOwner(ParentComponent->NumFrame());
        OtherBox.ApplyFakePhysicsOn(*this, InverseHit, delta);
    }
}

// ============================================================================
// ================= Helper Methods for ResolveHitVsGround ====================
// ============================================================================

// ============================================================================
//  ResolveWallOrGutter
//     - Single contact, no “support” logic, no persistence.
//     - Only prevents penetration along N and applies optional friction.
//     - Uses sequential impulse with minimal policy decisions.
// ============================================================================
void UBoxSubBody::ResolveWallOrGutter(const float& Dt, const SHitResult& Hit)
{
    if (!ParentComponent) return;

    const FVector N = Speed::QuantizeUnitNormal(Hit.ImpactNormal.GetSafeNormal());
    const FVector P = Speed::QuantizeVectorCm(Hit.ImpactPoint, 0.1f); // 1 mm

    const FVector COM = GetCOM();
    const float FaceAlignment = FMath::Max(
        FMath::Abs(FVector::DotProduct(Kinematics.Rotation.GetAxisX(), N)),
        FMath::Max(
            FMath::Abs(FVector::DotProduct(Kinematics.Rotation.GetAxisY(), N)),
            FMath::Abs(FVector::DotProduct(Kinematics.Rotation.GetAxisZ(), N))));
    const bool bFaceOnPlaneContact = FaceAlignment >= 0.98f;

    // A face-on box/plane impact is a contact manifold in every orientation,
    // including a recontact inside numerical slop. Resolve it through the face
    // center so a sampled corner cannot turn most of a wall or ceiling rebound
    // into angular motion.
    const FVector SolveP = bFaceOnPlaneContact
        ? COM + N * FVector::DotProduct(P - COM, N)
        : P;
    const FVector Vp = GetVelocityAtPoint(SolveP);
    const float vN = FVector::DotProduct(Vp, N);

    // If not approaching, do nothing (prevents “sticky” contacts on edges).
    constexpr float VN_EPS = 1.0f; // cm/s
    if (vN >= VN_EPS)
        return;

    const FVector r = SolveP - COM;

    const float InvMass = 1.f / GetMass();
    const FMatrix InvI = ComputeWorldInvInertiaTensor();

    const float denomN = EffectiveMassAlongDir(InvMass, InvI, r, N);
    if (denomN <= KINDA_SMALL_NUMBER)
        return;

    int32 PreviewSupportContacts = 0;
    const float UpDot = FVector::DotProduct(N, FVector::UpVector);
    const int32 CurrentFrame = ParentComponent->NumFrame();
    const int32 FramesSinceLastWallContact =
        LastWallOrGutterFrame == INDEX_NONE
        ? INDEX_NONE
        : CurrentFrame - LastWallOrGutterFrame;
    const bool bContinuesVerticalWallManifold =
        FramesSinceLastWallContact >= 0 &&
        FramesSinceLastWallContact <= 1 &&
        FMath::Abs(LastWallOrGutterUpDot) < 0.10f &&
        FVector::DotProduct(LastWallOrGutterN, N) >= 0.98f;
    if (UpDot >= 0.85f)
    {
        TArray<FVector> PreviewPts;
        BuildSupportManifoldFromNormal(
            N,
            Kinematics.Location,
            Kinematics.Rotation,
            BoxExtent,
            PreviewPts,
            0.5f
        );
        PreviewSupportContacts = PreviewPts.Num();
    }

    const bool bHasValidSupportPlane =
        bGroundPlaneValid &&
        FVector::DotProduct(GroundPlaneN.GetSafeNormal(), FVector::UpVector) >= 0.85f;

    const bool bIsGutterLikeWall =
        UpDot >= 0.10f && UpDot < 0.85f;

    const bool bIsInclinedWall =
        UpDot >= 0.15f && UpDot < 0.85f;

    const bool bIsLowSlopePenetratingWall =
        Hit.bStartPenetrating &&
        Hit.PenetrationDepth > 0.f &&
        UpDot >= 0.0f &&
        UpDot < 0.10f;

    const bool bIsLowSlopeGutterImpact =
        UpDot >= 0.0f &&
        UpDot < 0.10f &&
        bGroundHitFromSweep &&
        FMath::Abs(vN) > 300.f;

    const bool bIsNearGutterVerticalWall =
        UpDot >= -0.10f &&
        UpDot < 0.10f &&
        (bHasValidSupportPlane || (Hit.bStartPenetrating && Hit.PenetrationDepth > 0.f));

    const bool bIsNearHorizontalEdge =
        UpDot >= 0.85f && PreviewSupportContacts >= 2;

    const bool bIsSinglePointSteepSupportLikeWall =
        UpDot >= 0.85f && PreviewSupportContacts < 2;

    const bool bIsPenetratingStaticWall =
        Hit.bStartPenetrating && Hit.PenetrationDepth > 0.f;

    const bool bSupportedStaticTransition =
        CVarIAmSpeedBoxSupportedTransitionResponse.GetValueOnAnyThread() != 0 &&
        ParentComponent->HasCompatibleEstablishedStaticSupport(Hit);

    const bool bSoftWall =
        bIsNearHorizontalEdge ||
        bIsSinglePointSteepSupportLikeWall ||
        bIsInclinedWall ||
        bIsLowSlopePenetratingWall ||
        bIsLowSlopeGutterImpact ||
        bIsNearGutterVerticalWall ||
        (bIsGutterLikeWall && bHasValidSupportPlane) ||
        (bIsGutterLikeWall && bIsPenetratingStaticWall) ||
        bSupportedStaticTransition;

    // A clean aerial impact against a vertical wall is not a gutter contact.
    // It must be allowed to resolve its full restitution impulse in one hit.
    const bool bHardVerticalWallImpact =
        FMath::Abs(UpDot) < 0.10f &&
        !bHasValidSupportPlane &&
        !bSupportedStaticTransition &&
        (!Hit.bStartPenetrating || bContinuesVerticalWallManifold);
    const bool bUseSoftWallResponse = bSoftWall && !bHardVerticalWallImpact;

    LastWallOrGutterFrame = CurrentFrame;
    LastWallOrGutterN = N;
    LastWallOrGutterUpDot = UpDot;

#if !UE_BUILD_SHIPPING
    if (CVarIAmSpeedAutoRecoverContactDebug.GetValueOnAnyThread() != 0)
    {
        UE_LOG(
            BoxSubBodyLog,
            Log,
            TEXT("[CCDBox] ResolveWallOrGutter for frame = %d ImpactPoint = %s, ImpactNormal = %s, Box Kinematic = (%s)"),
            ParentComponent->NumFrame(),
            *P.ToString(),
            *N.ToString(),
            *Kinematics.ToString());
    }
#endif

    // Preserve restitution for clean wall impacts, while gutters and other
    // soft-wall contacts remain non-bouncy to avoid wedging artifacts.
    const bool bImpact = (vN < -GetImpactThreshold());

    const float Rest = bImpact && !bUseSoftWallResponse
        ? FMath::Clamp(GetRestitution(), 0.f, 1.f)
        : 0.f;
    if (bImpact)
    {
        // ParentComponent->SetHadImpactThisFrame(true);
    }

    // Normal impulse to cancel vN (or bounce if enabled)
    float jn = -(1.f + Rest) * vN / denomN;
    const bool bCanResolveFullPlaneImpact =
        !Hit.bStartPenetrating ||
        bHardVerticalWallImpact ||
        bFaceOnPlaneContact;
    const bool bHardImpact = bCanResolveFullPlaneImpact && bImpact;

    const float MaxWallDeltaVCmS =
        bUseSoftWallResponse ? 50.f :
        bHardImpact ? FMath::Max(180.f, FMath::Abs(vN) * (1.f + Rest)) :
        100.f;

    float SoftWallMassScale = 1.0f;
    if (bIsNearHorizontalEdge)
    {
        const float EdgeT = FMath::Clamp((UpDot - 0.85f) / (1.f - 0.85f), 0.f, 1.f);
        SoftWallMassScale = FMath::Lerp(0.25f, 0.35f, EdgeT);
    }
    else if (bIsSinglePointSteepSupportLikeWall)
    {
        SoftWallMassScale = 0.22f;
    }
    else if (bIsGutterLikeWall)
    {
        const float GutterT = FMath::Clamp((UpDot - 0.10f) / (0.85f - 0.10f), 0.f, 1.f);
        SoftWallMassScale = FMath::Lerp(0.55f, 0.25f, GutterT);
    }
    else if (bIsLowSlopePenetratingWall || bIsLowSlopeGutterImpact)
    {
        SoftWallMassScale = 0.25f;
    }
    else if (bIsNearGutterVerticalWall)
    {
        SoftWallMassScale = 0.25f;
    }

    const float jnMaxByMass =
        (bUseSoftWallResponse ? SoftWallMassScale : 5.0f) * GetMass();

    const float jnMaxByDeltaV = MaxWallDeltaVCmS / FMath::Max(denomN, KINDA_SMALL_NUMBER);

    // Guard against absurd impulses in 1-point concave contacts.
    // This is NOT a “wedging clamp”; it’s a numerical safety for edge cases.
    // Tune with care. If you prefer, delete it and rely on correct CCD.
    const float jnMax = bUseSoftWallResponse
        ? FMath::Min(jnMaxByDeltaV, jnMaxByMass)
        : jnMaxByDeltaV;
    jn = FMath::Clamp(jn, 0.f, jnMax);

    FVector J = jn * N;

    // Friction on walls/gutters:
    // - If your gutter is meant to be “smooth”, keep mu small.
    // - If you set mu too big, you’ll reintroduce wedging via tangential coupling.
    const float mu = 0.05f; // conservative; consider exposing as ParentComponent->HitboxFrictionWall

    const FVector vT = Vp - vN * N;
    const float vTmag = vT.Size();
    if (vTmag > KINDA_SMALL_NUMBER && mu > 0.f)
    {
        const FVector T = vT / vTmag;

        const float denomT = EffectiveMassAlongDir(InvMass, InvI, r, T);
        if (denomT > KINDA_SMALL_NUMBER)
        {
            // impulse to reduce tangential speed (simple Coulomb)
            float jt = -vTmag / denomT;

            // Coulomb clamp
            const float jtMax = mu * jn;
            jt = FMath::Clamp(jt, -jtMax, jtMax);

            J += jt * T;
        }
    }

    ApplyImpulse(J, SolveP);

    const bool bApplyWallGutterSlideDamping =
        ParentComponent->IsUpsideDown() &&
        (bIsGutterLikeWall ||
            bIsInclinedWall ||
            bIsLowSlopePenetratingWall ||
            bIsLowSlopeGutterImpact ||
            bIsNearGutterVerticalWall);
    IAmSpeedApplyGutterSlideDamping(
        ParentComponent,
        IsMainSubBody(),
        N,
        Dt,
        bApplyWallGutterSlideDamping,
        TEXT("WallGutter"));

#if !UE_BUILD_SHIPPING
    if (CVarIAmSpeedAutoRecoverContactDebug.GetValueOnAnyThread() != 0 &&
        (FMath::Abs(vN) > 5.f || Hit.bStartPenetrating))
    {
        UE_LOG(BoxSubBodyLog, Warning,
            TEXT("[CCDBoxWallSolve] Frame=%d vN=%.2f N=%s P=%s J=%s jn=%.4f jnMax=%.4f SoftWall=%d SoftMassScale=%.2f PreviewContacts=%d denomN=%.6f TOI=%.6f Pen=%d Depth=%.3f Kinematic=%s"),
            ParentComponent->NumFrame(),
            vN,
            *N.ToString(),
            *P.ToString(),
            *J.ToString(),
            jn,
            jnMax,
            bUseSoftWallResponse ? 1 : 0,
            SoftWallMassScale,
            PreviewSupportContacts,
            denomN,
            Hit.TOI,
            Hit.bStartPenetrating ? 1 : 0,
            Hit.PenetrationDepth,
            *Kinematics.ToString());
    }
#endif

    // Optional positional correction when starting in penetration.
    // Use Hit.ImpactNormal (depenetration direction) when bStartPenetrating.
    if (Hit.bStartPenetrating && Hit.PenetrationDepth > 0.f)
    {
        const FVector DepenDir = Hit.ImpactNormal.GetSafeNormal();
        constexpr float Slop = 0.5f;     // cm
        constexpr float BetaPos = 0.35f; // stronger than ground to escape concave edges

        // A nearly motionless unsupported body can otherwise remain inside the
        // slop forever: CCD keeps returning the same zero-TOI hit and gravity
        // never gets a chance to advance it.
        const bool bStalledUnsupportedShallowContact =
            Hit.PenetrationDepth <= Slop &&
            !HasPersistentGroundContact() &&
            ParentComponent->GetPhysCOMVelocity().SizeSquared() <= FMath::Square(5.f) &&
            vN >= -VN_EPS;
        const float pushOut = bStalledUnsupportedShallowContact
            ? Hit.PenetrationDepth + 0.1f
            : FMath::Max(Hit.PenetrationDepth - Slop, 0.f);
        if (pushOut > 0.f)
        {
            const float PositionScale =
                bStalledUnsupportedShallowContact ? 1.f : BetaPos;
            ParentComponent->SetPhysLocation(
                ParentComponent->GetPhysLocation() + DepenDir * (PositionScale * pushOut));

            // Kill inward velocity along depen dir
            const FVector V = ParentComponent->GetPhysCOMVelocity();
            const float vIn = FVector::DotProduct(V, DepenDir);
            if (vIn < 0.f && !bHardVerticalWallImpact)
            {
                FVector DepenVelocityDelta = -vIn * DepenDir;
                float DepenScale = 1.f;

                if (bUseSoftWallResponse)
                {
                    float MaxNormalKill = CVarIAmSpeedSoftWallDepenMaxNormalKillCmS.GetValueOnAnyThread();
                    const float NearVerticalMaxNormalKill =
                        CVarIAmSpeedNearGutterVerticalDepenMaxNormalKillCmS.GetValueOnAnyThread();
                    if (bIsNearGutterVerticalWall && NearVerticalMaxNormalKill > 0.f)
                    {
                        MaxNormalKill = FMath::Max(MaxNormalKill, NearVerticalMaxNormalKill);
                    }

                    if (MaxNormalKill > 0.f)
                    {
                        DepenScale = FMath::Min(DepenScale, MaxNormalKill / FMath::Max(-vIn, KINDA_SMALL_NUMBER));
                    }

                    const bool bCanInjectUpwardVelocity =
                        bIsGutterLikeWall ||
                        bIsInclinedWall ||
                        bIsLowSlopePenetratingWall ||
                        bIsLowSlopeGutterImpact ||
                        bIsNearGutterVerticalWall ||
                        bIsNearHorizontalEdge ||
                        bIsSinglePointSteepSupportLikeWall;

                    if (bCanInjectUpwardVelocity)
                    {
                        const float UpDelta = FVector::DotProduct(DepenVelocityDelta, FVector::UpVector);
                        const float MaxUpDelta = CVarIAmSpeedGutterDepenMaxUpDeltaCmS.GetValueOnAnyThread();
                        if (MaxUpDelta > 0.f && UpDelta > MaxUpDelta)
                        {
                            DepenScale = FMath::Min(DepenScale, MaxUpDelta / UpDelta);
                        }
                    }

                    DepenVelocityDelta *= FMath::Clamp(DepenScale, 0.f, 1.f);
                }

                ParentComponent->AddPhysVelocity(DepenVelocityDelta);

#if !UE_BUILD_SHIPPING
                if (CVarIAmSpeedAutoRecoverContactDebug.GetValueOnAnyThread() != 0 &&
                    (FMath::Abs(vIn) > 50.f || DepenScale < 0.999f))
                {
                    UE_LOG(BoxSubBodyLog, Warning,
                        TEXT("[CCDBoxWallDepenVelocity] Frame=%d vIn=%.2f DepenDir=%s DeltaV=%s Scale=%.3f SoftWall=%d GutterLike=%d UpDot=%.3f Depth=%.3f Kinematic=%s"),
                        ParentComponent->NumFrame(),
                        vIn,
                        *DepenDir.ToString(),
                        *DepenVelocityDelta.ToString(),
                        DepenScale,
                        bUseSoftWallResponse ? 1 : 0,
                        bIsGutterLikeWall ? 1 : 0,
                        UpDot,
                        Hit.PenetrationDepth,
                        *Kinematics.ToString());
                }
#endif
            }
        }
    }
}

// ============================================================================
//  ResolveDirectGroundSupport
//     - Multi-point support contact resolution (sequential impulses)
//     - Restitution only for “sharp new impacts”
//     - Coulomb friction only for support contacts
//     - Stores plane for persistence when appropriate
// ============================================================================
void UBoxSubBody::ResolveDirectGroundSupport(const float& Dt, const SHitResult& Hit)
{
    if (!ParentComponent) return;

    const FVector Nraw = Hit.ImpactNormal.GetSafeNormal();
    const FVector N = Nraw; // Keep raw; if you want composite, do it in persistence step
    const FVector ImpactP = Hit.ImpactPoint;

    // If separating, don't solve.
    constexpr float VN_EPS = 1.0f; // cm/s
    const float vN_atImpact = FVector::DotProduct(GetVelocityAtPoint(ImpactP), N);
    if (vN_atImpact >= VN_EPS)
        return;

    // Build manifold from current pose (TOI pose).
    // Kinematics = GetKinematicsFromOwner(ParentComponent->NumFrame());
    const FVector CenterWS = Kinematics.Location;
    const FQuat BoxRotWS = Kinematics.Rotation;
    const FVector Ext = BoxExtent;
    const float FaceReleaseDot = FMath::Clamp(CVarIAmSpeedEdgeLatchFaceReleaseDot.GetValueOnAnyThread(), 0.0f, 1.0f);
    const float FaceAlignmentAbs = IAmSpeedFindBestSupportFaceAlignment(N, BoxRotWS);
    const bool bNearlyFaceAligned = FaceAlignmentAbs >= FaceReleaseDot;
    const int32 CurrentFrame = ParentComponent->NumFrame();
    const bool bRoofFacingSurface =
        FVector::DotProduct(BoxRotWS.GetUpVector(), N) < -0.45f;
    const bool bRoofFaceNearlyAligned =
        FVector::DotProduct(BoxRotWS.GetUpVector(), N) <= -FaceReleaseDot;
    const bool bRoofSurfaceTraversalActive = HasRoofSurfaceTraversalSupport();
    // A face that replaces a supporting edge on the same plane is a
    // continuation of that ground contact, not a new bouncing impact.
    const bool bContinuingEdgeSupport =
        bGroundPlaneValid &&
        FVector::DotProduct(GroundPlaneN.GetSafeNormal(), N) >= 0.95f &&
        (PrevGroundContactsLS.Num() == 2 ||
            (bEdgeSupportLatched && LatchedEdgeContactsLS.Num() == 2));

    if (bNearlyFaceAligned && bEdgeSupportLatched)
    {
#if !UE_BUILD_SHIPPING
        if (CVarIAmSpeedAutoRecoverContactDebug.GetValueOnAnyThread() != 0)
        {
            UE_LOG(
                BoxSubBodyLog,
                Log,
                TEXT("[ARContact.EdgeLatchRelease] Frame=%d Source=ResolveDirect FaceDot=%.5f PrevLS=%d LatchedLS=%d"),
                ParentComponent ? ParentComponent->NumFrame() : INDEX_NONE,
                FaceAlignmentAbs,
                PrevGroundContactsLS.Num(),
                LatchedEdgeContactsLS.Num());
        }
#endif
        bEdgeSupportLatched = false;
        LatchedEdgeContactsLS.Reset();
        PrevGroundContactsLS.Reset();
    }

    TArray<FVector> ContactPtsWS;
    const float SupportContactEps = bNearlyFaceAligned
        ? FMath::Max(0.5f, CVarIAmSpeedFaceSupportContactEpsilonCm.GetValueOnAnyThread())
        : 0.5f;
    BuildSupportManifoldFromNormal(N, CenterWS, BoxRotWS, Ext, ContactPtsWS, SupportContactEps);
    if (bRoofSurfaceTraversalActive && !bRoofFaceNearlyAligned)
    {
        IAmSpeedBuildRoofTransitionEdge(N, CenterWS, BoxRotWS, Ext, ContactPtsWS);
    }
    else if (bNearlyFaceAligned)
    {
        IAmSpeedBuildSupportFaceManifoldFromNormal(N, CenterWS, BoxRotWS, Ext, ContactPtsWS, FaceReleaseDot);
    }
    else if (bRoofSurfaceTraversalActive && ContactPtsWS.Num() < 2)
    {
        IAmSpeedBuildRoofTransitionEdge(N, CenterWS, BoxRotWS, Ext, ContactPtsWS);
    }
    /*UE_LOG(BoxSubBodyLog, Log,
        TEXT("[EdgeLatchState] frame=%d latched=%d latchedLS=%d latchFrame=%d"),
        ParentComponent->NumFrame(),
        bEdgeSupportLatched ? 1 : 0,
        LatchedEdgeContactsLS.Num(),
        EdgeSupportLatchFrame);*/

        // ------------------------------------------------------------
        // PERSISTENT EDGE OVERRIDE
        // ------------------------------------------------------------
    const bool bUseLatchedEdge =
        bEdgeSupportLatched &&
        LatchedEdgeContactsLS.Num() == 2;

    if (bUseLatchedEdge && ContactPtsWS.Num() == 1)
    {
        const SKinematic K = GetKinematicsFromOwner(ParentComponent->NumFrame());

        const FVector A = K.Location + K.Rotation.RotateVector(LatchedEdgeContactsLS[0]);
        const FVector B = K.Location + K.Rotation.RotateVector(LatchedEdgeContactsLS[1]);

        ContactPtsWS.Reset();
        ContactPtsWS.Add(A);
        ContactPtsWS.Add(B);

        // IMPORTANT : renew latch so grace window doesn't expire
        if (vN_atImpact < -5.f)
        {
            bEdgeSupportLatched = true;
            EdgeSupportLatchFrame = ParentComponent->NumFrame();
        }

#if !UE_BUILD_SHIPPING
        // UE_LOG(BoxSubBodyLog, Log, TEXT("[PersistentEdgeOverride] Forcing edge support (renew latch)"));
#endif
    }

    const float UpDot = FVector::DotProduct(N, FVector::UpVector);
    const bool bFreshTwoPointEdgeSupport =
        ContactPtsWS.Num() == 2 &&
        !bNearlyFaceAligned &&
        !bRoofSurfaceTraversalActive &&
        !bUseLatchedEdge &&
        !(bEdgeSupportLatched && LatchedEdgeContactsLS.Num() == 2) &&
        PrevGroundContactsLS.Num() != 2;

    if (bFreshTwoPointEdgeSupport)
    {
#if !UE_BUILD_SHIPPING
        if (CVarIAmSpeedAutoRecoverContactDebug.GetValueOnAnyThread() != 0)
        {
            UE_LOG(
                BoxSubBodyLog,
                Warning,
                TEXT("[ARContact.FreshEdge] Frame=%d Contacts=%d UpDot=%.3f UseLatched=%d PrevLS=%d Latched=%d LatchedLS=%d Sweep=%d PlaneValidBefore=%d N=%s HitP=%s TOI=%.6f"),
                ParentComponent ? ParentComponent->NumFrame() : INDEX_NONE,
                ContactPtsWS.Num(),
                UpDot,
                bUseLatchedEdge ? 1 : 0,
                PrevGroundContactsLS.Num(),
                bEdgeSupportLatched ? 1 : 0,
                LatchedEdgeContactsLS.Num(),
                bGroundHitFromSweep ? 1 : 0,
                bGroundPlaneValid ? 1 : 0,
                *N.ToString(),
                *ImpactP.ToString(),
                Hit.TOI);
        }
#endif

        // Do not solve a fresh edge as a stable support, otherwise the box can
        // balance on an edge. Keep the contact manifold/plane though: car-level
        // auto-recover needs those two points to bias the hitbox toward a face.
        CurrentGroundContactsWS = ContactPtsWS;
        bFreshEdgeRecoverCandidate = true;

        if (bGroundHitFromSweep)
        {
            bGroundPlaneValid = true;
            GroundPlaneN = N;
            GroundPlanePointWS = Hit.ImpactPoint;
            GroundPlaneD = FVector::DotProduct(ImpactP, N);
            GroundComp = Hit.Component;
            ParentComponent->RcvImpactOnSubBody(*this, ImpactP);
        }

        /*UE_LOG(BoxSubBodyLog, Warning,
            TEXT("[CCDBoxSupportReject] Frame=%d Reason=FreshEdge UpDot=%.4f Contacts=%d UseLatched=%d PrevLS=%d Latched=%d LatchedLS=%d -> Wall/Gutter N=%s HitP=%s TOI=%.6f"),
            ParentComponent->NumFrame(),
            UpDot,
            ContactPtsWS.Num(),
            bUseLatchedEdge ? 1 : 0,
            PrevGroundContactsLS.Num(),
            bEdgeSupportLatched ? 1 : 0,
            LatchedEdgeContactsLS.Num(),
            *N.ToString(),
            *ImpactP.ToString(),
            Hit.TOI);*/

        ResolveWallOrGutter(Dt, Hit);
        return;
    }

    const float RoofSlideSupportMinUpDot =
        CVarIAmSpeedRoofSlideSupportMinUpDot.GetValueOnAnyThread();
    const bool bUpsideDownRoofSlideSupport =
        bRoofFacingSurface &&
        (bRoofSurfaceTraversalActive ||
            (ContactPtsWS.Num() == 1 &&
                UpDot >= RoofSlideSupportMinUpDot &&
                UpDot < 0.97f));

    if (ContactPtsWS.Num() < 2 && UpDot < 0.97f && !bUpsideDownRoofSlideSupport)
    {
        CurrentGroundContactsWS.Reset();

        /*UE_LOG(BoxSubBodyLog, Warning,
            TEXT("[CCDBoxSupportReject] Frame=%d Reason=InclinedSinglePoint UpDot=%.4f Contacts=%d UseLatched=%d -> Wall/Gutter N=%s HitP=%s TOI=%.6f"),
            ParentComponent->NumFrame(),
            UpDot,
            ContactPtsWS.Num(),
            bUseLatchedEdge ? 1 : 0,
            *N.ToString(),
            *ImpactP.ToString(),
            Hit.TOI);*/

        ResolveWallOrGutter(Dt, Hit);
        return;
    }

    const bool bHasCompatiblePersistentPlane =
        bGroundPlaneValid &&
        FVector::DotProduct(GroundPlaneN.GetSafeNormal(), N) >= 0.95f;
    const bool bFreshSweepSupport =
        bGroundHitFromSweep &&
        !bHasCompatiblePersistentPlane &&
        !bUseLatchedEdge &&
        !bUpsideDownRoofSlideSupport &&
        !bHasGroundContact &&
        PrevGroundContactsLS.Num() == 0 &&
        !(bEdgeSupportLatched && LatchedEdgeContactsLS.Num() == 2);

    CurrentGroundContactsWS = ContactPtsWS;

    const bool bStableRoofFaceSupport =
        IsMainSubBody() && bRoofFaceNearlyAligned &&
        ContactPtsWS.Num() >= 3;
    const bool bInclinedRoofTraversalSupport =
        bRoofFacingSurface &&
        UpDot >= RoofSlideSupportMinUpDot &&
        UpDot < 0.97f &&
        ContactPtsWS.Num() >= 2;
    const FVector TangentialVelocity =
        ParentComponent->GetPhysCOMVelocity() -
        FVector::DotProduct(ParentComponent->GetPhysCOMVelocity(), N) * N;
    constexpr float MinTraversalLatchSpeedCmS = 1000.f;
    const bool bCanStartRoofTraversal =
        !ParentComponent->IsInAutoRecover() &&
        TangentialVelocity.Size() >= MinTraversalLatchSpeedCmS &&
        (bStableRoofFaceSupport ||
            bUpsideDownRoofSlideSupport ||
            bInclinedRoofTraversalSupport);
    if ((bCanStartRoofTraversal || bRoofSurfaceTraversalActive) &&
        Hit.Component.IsValid())
    {
        bRoofSurfaceTraversalLatched = true;
        LastRoofSurfaceContactFrame = CurrentFrame;
        RoofSurfaceComp = Hit.Component;
    }

    IAmSpeedApplyGutterSlideDamping(
        ParentComponent,
        IsMainSubBody(),
        N,
        Dt,
        bUpsideDownRoofSlideSupport || HasRoofSurfaceTraversalSupport(),
        TEXT("DirectRoofSlide"));

// #if !UE_BUILD_SHIPPING
    /*UE_LOG(BoxSubBodyLog, Warning,
        TEXT("[CCDBoxSupportPrep] Frame=%d vN=%.2f Fresh=%d UseLatched=%d Contacts=%d Latched=%d LatchedLS=%d PlaneValidBefore=%d HitN=%s HitP=%s TOI=%.6f Sweep=%d"),
        ParentComponent->NumFrame(),
        vN_atImpact,
        bFreshSweepSupport ? 1 : 0,
        bUseLatchedEdge ? 1 : 0,
        CurrentGroundContactsWS.Num(),
        bEdgeSupportLatched ? 1 : 0,
        LatchedEdgeContactsLS.Num(),
        bGroundPlaneValid ? 1 : 0,
        *N.ToString(),
        *ImpactP.ToString(),
        Hit.TOI,
        bGroundHitFromSweep ? 1 : 0);*/
// #endif

    // if we end up with 2 points, renew too (even without override)
    if (!bNearlyFaceAligned && CurrentGroundContactsWS.Num() == 2 && bEdgeSupportLatched && LatchedEdgeContactsLS.Num() == 2)
    {
        EdgeSupportLatchFrame = ParentComponent->NumFrame();
    }


    // Store plane for persistence (only if the hit came from sweep and is “support-like”).
    if (bGroundHitFromSweep)
    {
        bGroundPlaneValid = true;
        GroundPlaneN = N;
        GroundPlanePointWS = Hit.ImpactPoint;
        GroundPlaneD = FVector::DotProduct(ImpactP, N);
        GroundComp = Hit.Component;
		ParentComponent->RcvImpactOnSubBody(*this, ImpactP);
    }

    // If concave, do not persist (your existing rule).
    if (IsConcaveGroundContact())
    {
        bGroundPlaneValid = false;
        bHasGroundContact = false;
        CurrentGroundContactsWS.Reset();
    }

    // Sequential impulse (warm-started)
    const int32 NumC = CurrentGroundContactsWS.Num();
    if (NumC <= 0) return;

	// Register constraints for this manifold
    if (CurrentHit.Component.IsValid())
    {
        RegisterContactManifoldAsConstraints(
            CurrentGroundContactsWS,
            N,
            CurrentHit.Component.Get(),
            CurrentHit.PenetrationDepth,
            CurrentHit.TOI,
            false,
            4
        );
    }

    if (PrevLambdaN.Num() != NumC)
        PrevLambdaN.Reset();

    const int32 Iter = FMath::Clamp(NumC * 2, 6, 12);
    const bool bRecentlyHadWallOrGutter =
        LastWallOrGutterFrame != INDEX_NONE &&
        CurrentFrame - LastWallOrGutterFrame <= 8;
    if (FreshSupportClampUntilFrame != INDEX_NONE && CurrentFrame > FreshSupportClampUntilFrame)
    {
        FreshSupportClampUntilFrame = INDEX_NONE;
        FreshSupportClampComp.Reset();
        FreshSupportClampN = FVector::UpVector;
    }

    const bool bSameFreshClampComponent =
        Hit.Component.IsValid() &&
        FreshSupportClampComp.IsValid() &&
        Hit.Component.Get() == FreshSupportClampComp.Get();
    const bool bFreshClampNormalCompatible =
        FVector::DotProduct(N, FreshSupportClampN) >= 0.98f;

    bool bInFreshSupportClampWindow =
        FreshSupportClampUntilFrame != INDEX_NONE &&
        CurrentFrame <= FreshSupportClampUntilFrame &&
        bSameFreshClampComponent &&
        bFreshClampNormalCompatible;

    if (bFreshSweepSupport && NumC >= 3 && Hit.Component.IsValid())
    {
        FreshSupportClampUntilFrame = CurrentFrame + 4;
        FreshSupportClampN = N;
        FreshSupportClampComp = Hit.Component;
        bInFreshSupportClampWindow = true;
    }

    float RemainingFreshSupportImpulse = TNumericLimits<float>::Max();
    if ((bFreshSweepSupport || bInFreshSupportClampWindow) && NumC >= 3)
    {
        const float MaxFreshSupportDeltaVCmS = bRecentlyHadWallOrGutter ? 10.f : 25.f;
        const float VnCom = FVector::DotProduct(ParentComponent->GetPhysCOMVelocity(), N);
        RemainingFreshSupportImpulse =
            GetMass() * FMath::Min(FMath::Max(-VnCom, 0.f), MaxFreshSupportDeltaVCmS);

        /*UE_LOG(BoxSubBodyLog, Warning,
            TEXT("[CCDBoxFreshSupportClamp] Frame=%d Fresh=%d Window=%d Until=%d Contacts=%d vNCom=%.2f MaxImpulse=%.3f N=%s HitP=%s RecentlyHadWallOrGutter=%d"),
            CurrentFrame,
            bFreshSweepSupport ? 1 : 0,
            bInFreshSupportClampWindow ? 1 : 0,
            FreshSupportClampUntilFrame,
            NumC,
            VnCom,
            RemainingFreshSupportImpulse,
            *N.ToString(),
            *ImpactP.ToString(),
            bRecentlyHadWallOrGutter ? 1 : 0);*/
    }

    TArray<float> LambdaN;
    LambdaN.SetNum(NumC);
    for (int32 i = 0; i < NumC; ++i)
        LambdaN[i] = (i < PrevLambdaN.Num()) ? PrevLambdaN[i] : 0.f;

    const float InvMass = 1.f / GetMass();
    const FMatrix InvI = ComputeWorldInvInertiaTensor();
    const FVector COMVelocityBeforeSolve = ParentComponent->GetPhysCOMVelocity();
    FVector TotalNormalImpulse = FVector::ZeroVector;
    FVector TotalTangentialImpulse = FVector::ZeroVector;
    // (BoxSubBodyLog, Log, TEXT("[CCDBox] ResolveDirectGroundSupport for frame = %d CurrentGroundContactsWS.Num = %d, ImpactNormal = %s, Kinematic = %s"),
    //     ParentComponent->NumFrame(), CurrentGroundContactsWS.Num(), *N.ToString(), *Kinematics.ToString());

    // Restitution policy (ground):
    // constexpr float ImpactThreshold = 30.f; // cm/s
    for (int32 it = 0; it < Iter; ++it)
    {
        if (NumC == 2)
        {
            SolveEdgeSupportConstraint(
                N, CurrentGroundContactsWS, Dt, /*bDoFriction=*/false, /*Mu=*/0.f, bRoofSurfaceTraversalActive);
            break;
        }
        for (int32 i = 0; i < NumC; ++i)
        {
            const FVector& P = CurrentGroundContactsWS[i];
            const FVector Vp = GetVelocityAtPoint(P);
            const float vN = FVector::DotProduct(Vp, N);
            if (vN >= 0.f)
                continue;

            const FVector COM = GetCOM();
            const FVector r = P - COM;

            const float denomN = EffectiveMassAlongDir(InvMass, InvI, r, N);
            if (denomN <= KINDA_SMALL_NUMBER)
                continue;

            // “Sharp impact” detection (optional, based on your old logic)
            float Rest = 0.f;
            const bool bImpact = (vN < -GetImpactThreshold());

            const float cosAngle = bHadContactPrevFrame ? FVector::DotProduct(N, PrevContactNormal) : 0.f;
            const bool bNewContact =
                !bContinuingEdgeSupport &&
                !bHadGroundContactPrevFrame &&
                bGroundHitFromSweep;
            const bool bSharpImpact = (bNewContact && bImpact && cosAngle > 0.95f);

            if (bSharpImpact)
            {
                Rest = GetRestitution();
                // ParentComponent->SetHadImpactThisFrame(true);
            }

            // normal impulse increment
            const float targetVN = bImpact ? -(Rest * vN) : 0.f; // simple: drive vN to 0, with bounce if enabled
            const float dvN = (targetVN - vN);

            float dLambda = dvN / denomN;

            const float lambdaPrev = LambdaN[i];
            float lambdaNew = FMath::Max(lambdaPrev + dLambda, 0.f);
            float applied = lambdaNew - lambdaPrev;

            if (RemainingFreshSupportImpulse < TNumericLimits<float>::Max())
            {
                const float AppliedClamped = FMath::Min(applied, RemainingFreshSupportImpulse);
                RemainingFreshSupportImpulse -= AppliedClamped;
                applied = AppliedClamped;
                lambdaNew = lambdaPrev + applied;

                if (applied <= KINDA_SMALL_NUMBER)
                {
                    LambdaN[i] = lambdaPrev;
                    continue;
                }
            }

            LambdaN[i] = lambdaNew;

            FVector J = applied * N;
            const FVector NormalImpulse = J;
            const float appliedRaw = applied;
            const float BudgetBefore = RemainingFreshSupportImpulse;

            // friction (Coulomb) on supporting contact
            const float mu = GetDynamicFriction();
            if (mu > 0.f)
            {
                const FVector vT = Vp - vN * N;
                const float vTmag = vT.Size();
                if (vTmag > KINDA_SMALL_NUMBER)
                {
                    const FVector T = vT / vTmag;

                    const float denomT = EffectiveMassAlongDir(InvMass, InvI, r, T);
                    if (denomT > KINDA_SMALL_NUMBER)
                    {
                        float jt = -vTmag / denomT;
                        const float jtMax = mu * applied;
                        jt = FMath::Clamp(jt, -jtMax, jtMax);
                        J += jt * T;
                    }
                }
            }
            const float BudgetAfter = RemainingFreshSupportImpulse;

            ApplyImpulse(J, P);
            TotalNormalImpulse += NormalImpulse;
            TotalTangentialImpulse += J - NormalImpulse;
            /*(BoxSubBodyLog, Log,
                TEXT("[CCDBoxSupportImpulse] Frame=%d It=%d Pt=%d Fresh=%d Window=%d RecentlyWall=%d ")
                TEXT("NumC=%d vN=%.2f denomN=%.6f appliedRaw=%.3f appliedFinal=%.3f BudgetBefore=%.3f BudgetAfter=%.3f ")
                TEXT("P=%s N=%s J=%s Vel=%s AngVel=%s"),
                ParentComponent->NumFrame(),
                it,
                i,
                bFreshSweepSupport ? 1 : 0,
                bInFreshSupportClampWindow ? 1 : 0,
                bRecentlyHadWallOrGutter ? 1 : 0,
                NumC,
                vN,
                denomN,
                appliedRaw,
                applied,
                BudgetBefore,
                RemainingFreshSupportImpulse,
                *P.ToString(),
                *N.ToString(),
                *J.ToString(),
                *ParentComponent->GetPhysCOMVelocity().ToString(),
                *ParentComponent->GetPhysAngularVelocity().ToString());*/
        }
    }

#if !UE_BUILD_SHIPPING
    if (CVarIAmSpeedBoxGroundSupportDebug.GetValueOnAnyThread() != 0)
    {
        const FVector COMVelocityAfterSolve = ParentComponent->GetPhysCOMVelocity();
        UE_LOG(
            BoxSubBodyLog,
            Log,
            TEXT("[BoxGroundSupport] Frame=%d Contacts=%d Iter=%d Mu=%.3f Fresh=%d ContinuingEdge=%d COMVBefore=%s COMVAfter=%s DeltaCOMV=%s NormalImpulse=%s TangentialImpulse=%s"),
            ParentComponent->NumFrame(),
            NumC,
            Iter,
            GetDynamicFriction(),
            bFreshSweepSupport ? 1 : 0,
            bContinuingEdgeSupport ? 1 : 0,
            *COMVelocityBeforeSolve.ToString(),
            *COMVelocityAfterSolve.ToString(),
            *(COMVelocityAfterSolve - COMVelocityBeforeSolve).ToString(),
            *TotalNormalImpulse.ToString(),
            *TotalTangentialImpulse.ToString());
    }
#endif

    PrevLambdaN = LambdaN;

    // Positional correction (Baumgarte / split impulse style)
    if (Hit.bStartPenetrating && Hit.PenetrationDepth > 0.f)
    {
        const FVector DepenDir = Hit.ImpactNormal.GetSafeNormal();
        constexpr float Slop = 0.5f;     // cm
        constexpr float BetaPos = 0.2f;

        const float pushOut = FMath::Max(Hit.PenetrationDepth - Slop, 0.f);
        if (pushOut > 0.f)
        {
            ParentComponent->SetPhysLocation(ParentComponent->GetPhysLocation() + DepenDir * (BetaPos * pushOut));

            const FVector V = ParentComponent->GetPhysCOMVelocity();
            const float vIn = FVector::DotProduct(V, DepenDir);
            if (vIn < 0.f)
                ParentComponent->AddPhysVelocity(-vIn * DepenDir);
        }
    }

    // Track previous contact normal
    PrevContactNormal = N;
    bHadContactPrevFrame = true;
}

// ============================================================================
//  ApplyPersistentSupportConstraint
//     - This is the “keep rear edge down” constraint.
//     - Runs AFTER event resolution (wall/gutter and/or direct ground hit).
//     - Uses the stored ground plane and recomputes a manifold on that plane.
//     - Applies a small corrective impulse to kill normal velocity into the plane,
//       and a small positional correction to stay within tolerance.
// ============================================================================
void UBoxSubBody::ApplyPersistentSupportConstraint(const float& Dt)
{
    if (!ParentComponent) return;

	// if hitbox is in auto-recover and has 2 ground contacts, skip persistence to avoid fighting with edge recovery logic
    if (ParentComponent->IsInAutoRecover() &&
        GetGroundContacts().Num() == 2)
    {
        return;
    }

    // UpdatePersistentGroundContact must have set:
    //  - bHasGroundContact
    //  - GroundHit with ImpactNormal + ImpactPoint on plane
    //  - CurrentGroundContactsWS rebuilt (manifold)
    if (!HasPersistentGroundContact())
        return;

    // IMPORTANT: keep the persistent support constrained to the stored ground plane.
    // Mixing in a wall normal here can create a fake support plane in floor/wall corners
    // and pull the hitbox into the ground.
    const FVector N = GroundPlaneN.GetSafeNormal();
    const bool bRoofSurfaceTraversalActive = HasRoofSurfaceTraversalSupport();

    // If the plane became non-support (e.g. got overwritten), bail.
    if (FVector::DotProduct(N, FVector::UpVector) < 0.5f && !bRoofSurfaceTraversalActive)
        return;

    // Concave rule: never persist there
    if (IsConcaveGroundContact())
        return;

    // Rebuild manifold from current pose *against the ground plane normal*.
    // This is key: it keeps 2+ points, so you don't pivot on the gutter edge.
    const SKinematic K = GetKinematicsFromOwner(ParentComponent->NumFrame());
    TArray<FVector> SupportPts;
    const float FaceReleaseDot = FMath::Clamp(CVarIAmSpeedEdgeLatchFaceReleaseDot.GetValueOnAnyThread(), 0.0f, 1.0f);
    const float FaceAlignmentAbs = IAmSpeedFindBestSupportFaceAlignment(N, K.Rotation);
    const bool bNearlyFaceAligned = FaceAlignmentAbs >= FaceReleaseDot;

    if (bNearlyFaceAligned && bEdgeSupportLatched)
    {
#if !UE_BUILD_SHIPPING
        if (CVarIAmSpeedAutoRecoverContactDebug.GetValueOnAnyThread() != 0)
        {
            UE_LOG(
                BoxSubBodyLog,
                Log,
                TEXT("[ARContact.EdgeLatchRelease] Frame=%d Source=PersistentConstraint FaceDot=%.5f PrevLS=%d LatchedLS=%d"),
                ParentComponent ? ParentComponent->NumFrame() : INDEX_NONE,
                FaceAlignmentAbs,
                PrevGroundContactsLS.Num(),
                LatchedEdgeContactsLS.Num());
        }
#endif
        bEdgeSupportLatched = false;
        LatchedEdgeContactsLS.Reset();
        PrevGroundContactsLS.Reset();
    }

    if (!bNearlyFaceAligned && bEdgeSupportLatched && LatchedEdgeContactsLS.Num() == 2)
    {
        // Force edge support from latched memory
        SupportPts.Add(
            K.Location + K.Rotation.RotateVector(LatchedEdgeContactsLS[0])
        );
        SupportPts.Add(
            K.Location + K.Rotation.RotateVector(LatchedEdgeContactsLS[1])
        );
    }
    else
    {
        const float SupportContactEps = bNearlyFaceAligned
            ? FMath::Max(0.75f, CVarIAmSpeedFaceSupportContactEpsilonCm.GetValueOnAnyThread())
            : 0.75f;
        BuildSupportManifoldFromNormal(
            N, K.Location, K.Rotation, BoxExtent, SupportPts, SupportContactEps
        );
        if (bNearlyFaceAligned)
        {
            IAmSpeedBuildSupportFaceManifoldFromNormal(N, K.Location, K.Rotation, BoxExtent, SupportPts, FaceReleaseDot);
        }
    }

    if (SupportPts.Num() < 2)
    {
        return;
    }

    // Measure “worst” normal velocity into the plane among support points.
    float worstInto = 0.f;
    for (const FVector& P : SupportPts)
    {
        const float vN = FVector::DotProduct(GetVelocityAtPoint(P), N);
        worstInto = FMath::Min(worstInto, vN); // most negative
    }

    // If not moving into plane, we don't need impulses (prevents sticky behavior).
    constexpr float VN_EPS = 0.0f; // cm/s (allow tiny bias)
    if (worstInto > VN_EPS)
    {
        bHadGroundContactPrevFrame = true;
        PrevGroundNormal = N;
        return;
    }
	// Register constraints for this manifold
    if (GroundComp.IsValid())
    {
        RegisterContactManifoldAsConstraints(
            SupportPts,
            N,
            GroundComp.Get(),
            0.f,
            0.f,
            true,
            4
        );
    }

    // Apply a *distributed* correction impulse across manifold points.
    // Physics-style: sequential impulses over points, targeting vN -> 0.
    const float InvMass = 1.f / GetMass();
    const FMatrix InvI = ComputeWorldInvInertiaTensor();

    // Debug
    Kinematics = GetKinematicsFromOwner(ParentComponent->NumFrame());
    // UE_LOG(BoxSubBodyLog, Log, TEXT("[CCDBox] ResolveDirectGroundSupport for frame = %d, ImpactNormal = %s, Kinematic = %s"),
    //     ParentComponent->NumFrame(), *N.ToString(), *Kinematics.ToString());

    constexpr int32 Iter = 6;
    for (int32 it = 0; it < Iter; ++it)
    {
        if (SupportPts.Num() == 2)
        {
            SolveEdgeSupportConstraint(
                N, SupportPts, Dt, /*bDoFriction=*/false, /*Mu=*/0.f, bRoofSurfaceTraversalActive);
            break;
        }
        for (const FVector& P : SupportPts)
        {
            const FVector Vp = GetVelocityAtPoint(P);
            const float vN = FVector::DotProduct(Vp, N);

            // target is ZERO while support is active
            const float dvN = -vN;

            if (FMath::Abs(dvN) < 0.01f)
                continue;

            const FVector COM = GetCOM();
            const FVector r = P - COM;

            const float denomN = EffectiveMassAlongDir(InvMass, InvI, r, N);
            if (denomN <= KINDA_SMALL_NUMBER)
                continue;

            // Cancel normal velocity (no bounce for persistence)
            const float jn = (dvN) / denomN;
            ApplyImpulse(jn * N, P);
        }
    }

    // Positional drift correction toward plane (soft).
    // Keep within a small band so you don't accumulate penetration over many frames.
    TArray<FVector> Verts;
    GetBoxVertices(K.Location, K.Rotation, BoxExtent, Verts);

    float minDist = FLT_MAX;
    const FVector PlaneN = GroundPlaneN.GetSafeNormal();
    for (const FVector& V : Verts)
    {
        const float dist = PlaneSignedDist(V, PlaneN, GroundPlanePointWS);
        minDist = FMath::Min(minDist, dist);
    }

    constexpr float ContactTol = 0.05f; // cm
    constexpr float PenTol = 0.5f;  // cm
    constexpr float BetaPos = 0.1f;  // very gentle

    // If we drift too far into the plane, push out gently.
    if (minDist < -PenTol)
    {
        const float pushOut = (-PenTol - minDist);
        ParentComponent->SetPhysLocation(ParentComponent->GetPhysLocation() + N * (BetaPos * pushOut));

        // Kill inward normal component at COM
        const FVector Vcm = ParentComponent->GetPhysCOMVelocity();
        const float vIn = FVector::DotProduct(Vcm, N);
        if (vIn < 0.f)
            ParentComponent->AddPhysVelocity(-vIn * N);
    }
    else if (minDist > ContactTol)
    {
        constexpr float MaxRoofTraversalDrift = 25.f;
        if (bRoofSurfaceTraversalActive && minDist <= MaxRoofTraversalDrift)
        {
            ParentComponent->SetPhysLocation(
                ParentComponent->GetPhysLocation() - N * (minDist - ContactTol));

            const FVector Vcm = ParentComponent->GetPhysCOMVelocity();
            const float vOut = FVector::DotProduct(Vcm, N);
            if (vOut > 0.f)
            {
                ParentComponent->AddPhysVelocity(-vOut * N);
            }
        }
        else
        {
            // If we’re clearly leaving the plane, stop persisting.
            // (Otherwise you “float” and keep canceling gravity.)
            bGroundPlaneValid = false;
            bHasGroundContact = false;
        }
    }

    // Hard release conditions for latched edge
    if (bEdgeSupportLatched)
    {
        const float upDot = FVector::DotProduct(N, FVector::UpVector);
        if (upDot < 0.5f || minDist > 2.0f)
        {
            bEdgeSupportLatched = false;
            LatchedEdgeContactsLS.Reset();
        }
    }

    bHadGroundContactPrevFrame = true;
    PrevGroundNormal = N;
}

void UBoxSubBody::UpdatePersistentGroundContact(const float& Dt, const bool bDirectSupportSolved)
{
    const SHitResult SupportSourceHit = GroundHit;
    const auto PreserveSupportProvider = [&SupportSourceHit](SHitResult& Hit)
    {
        Hit.GeometricErrorBoundCm = SupportSourceHit.GeometricErrorBoundCm;
        Hit.bSurfaceNormalMayVary = SupportSourceHit.bSurfaceNormalMayVary;
        Hit.SourceId = SupportSourceHit.SourceId;
        Hit.SurfaceId = SupportSourceHit.SurfaceId;
        Hit.FeatureId = SupportSourceHit.FeatureId;
        Hit.PrimitiveId = SupportSourceHit.PrimitiveId;
        Hit.CanonicalGroupId = SupportSourceHit.CanonicalGroupId;
        Hit.MaterialId = SupportSourceHit.MaterialId;
    };
    bHasGroundContact = false;
	const bool bEdgeRecoverActive = ParentComponent->IsInAutoRecover();

    if (!bGroundPlaneValid || !GroundComp.IsValid())
    {
#if !UE_BUILD_SHIPPING
        if (CVarIAmSpeedAutoRecoverContactDebug.GetValueOnAnyThread() != 0 &&
            (CurrentGroundContactsWS.Num() > 0 || PrevGroundContactsLS.Num() > 0 || bFreshEdgeRecoverCandidate))
        {
            UE_LOG(
                BoxSubBodyLog,
                Log,
                TEXT("[ARContact.PersistentReject] Frame=%d Reason=NoPlane PlaneValid=%d GroundComp=%s Current=%d PrevLS=%d FreshEdge=%d"),
                ParentComponent ? ParentComponent->NumFrame() : INDEX_NONE,
                bGroundPlaneValid ? 1 : 0,
                GroundComp.IsValid() ? *GroundComp->GetName() : TEXT("None"),
                CurrentGroundContactsWS.Num(),
                PrevGroundContactsLS.Num(),
                bFreshEdgeRecoverCandidate ? 1 : 0);
        }
#endif
        return;
    }

    const float GroundPlaneUpDot = FVector::DotProduct(GroundPlaneN, FVector::UpVector);
    const int32 CurrentFrame = ParentComponent->NumFrame();
    const bool bRoofSurfaceTraversalActive = HasRoofSurfaceTraversalSupport();
    if (GroundPlaneUpDot < 0.5f && !bRoofSurfaceTraversalActive)
    {
#if !UE_BUILD_SHIPPING
        if (CVarIAmSpeedAutoRecoverContactDebug.GetValueOnAnyThread() != 0)
        {
            UE_LOG(
                BoxSubBodyLog,
                Log,
                TEXT("[ARContact.PersistentReject] Frame=%d Reason=LowUpDot UpDot=%.3f Current=%d PrevLS=%d FreshEdge=%d N=%s"),
                ParentComponent ? ParentComponent->NumFrame() : INDEX_NONE,
                GroundPlaneUpDot,
                CurrentGroundContactsWS.Num(),
                PrevGroundContactsLS.Num(),
                bFreshEdgeRecoverCandidate ? 1 : 0,
                *GroundPlaneN.ToString());
        }
#endif
        return;
    }

    if (IsConcaveGroundContact())
    {
#if !UE_BUILD_SHIPPING
        if (CVarIAmSpeedAutoRecoverContactDebug.GetValueOnAnyThread() != 0)
        {
            UE_LOG(
                BoxSubBodyLog,
                Log,
                TEXT("[ARContact.PersistentReject] Frame=%d Reason=Concave Current=%d Normals=%d PrevLS=%d FreshEdge=%d"),
                ParentComponent ? ParentComponent->NumFrame() : INDEX_NONE,
                CurrentGroundContactsWS.Num(),
                CurrentGroundNormalsWS.Num(),
                PrevGroundContactsLS.Num(),
                bFreshEdgeRecoverCandidate ? 1 : 0);
        }
#endif
        bGroundPlaneValid = false;
        bHasGroundContact = false;
        bGroundContactStable = false;
        StableTime = 0.f;
        CurrentGroundContactsWS.Reset();
        PrevGroundContactsLS.Reset();
        return;
    }

    const FVector N = GroundPlaneN.GetSafeNormal();
    const SKinematic K = GetKinematicsFromOwner(ParentComponent->NumFrame());
    const float FaceReleaseDot = FMath::Clamp(CVarIAmSpeedEdgeLatchFaceReleaseDot.GetValueOnAnyThread(), 0.0f, 1.0f);
    const float FaceAlignmentAbs = IAmSpeedFindBestSupportFaceAlignment(N, K.Rotation);
    const bool bNearlyFaceAligned = FaceAlignmentAbs >= FaceReleaseDot;
    const bool bEdgeRecoverCandidate =
        !bNearlyFaceAligned &&
        (PrevGroundContactsLS.Num() == 2 || bEdgeSupportLatched || bFreshEdgeRecoverCandidate);
    const bool bVertexRecoverCandidate =
        CurrentGroundContactsWS.Num() == 1 &&
        PrevGroundContactsLS.Num() <= 1 &&
        FaceAlignmentAbs < FaceReleaseDot;

    if (bNearlyFaceAligned && bEdgeSupportLatched)
    {
#if !UE_BUILD_SHIPPING
        if (CVarIAmSpeedAutoRecoverContactDebug.GetValueOnAnyThread() != 0)
        {
            UE_LOG(
                BoxSubBodyLog,
                Log,
                TEXT("[ARContact.EdgeLatchRelease] Frame=%d Source=PersistentUpdate FaceDot=%.5f PrevLS=%d LatchedLS=%d"),
                ParentComponent ? ParentComponent->NumFrame() : INDEX_NONE,
                FaceAlignmentAbs,
                PrevGroundContactsLS.Num(),
                LatchedEdgeContactsLS.Num());
        }
#endif
        bEdgeSupportLatched = false;
        LatchedEdgeContactsLS.Reset();
        PrevGroundContactsLS.Reset();
    }

    TArray<FVector> Verts;
    GetBoxVertices(K.Location, K.Rotation, BoxExtent, Verts);

    // ------------------------------------------------------------
    // Compute min distance to plane
    // ------------------------------------------------------------
    float minDist = FLT_MAX;
    for (const FVector& V : Verts)
    {
        const float dist = PlaneSignedDist(V, GroundPlaneN, GroundPlanePointWS);
        minDist = FMath::Min(minDist, dist);
    }

    float worstVN = 0.f;
    for (const FVector& P : CurrentGroundContactsWS)
    {
        const float vnP = FVector::DotProduct(GetVelocityAtPoint(P), N);
        worstVN = FMath::Max(worstVN, vnP);
    }

    constexpr float ContactTol = 0.10f;
    constexpr float PenTolSupport = 1.0f;
    constexpr float PenTolReject = 2.5f;
    constexpr float SepVelTol = 3.0f;
    const float ActiveContactTol = bNearlyFaceAligned || bRoofSurfaceTraversalActive
        ? FMath::Max(ContactTol, CVarIAmSpeedFaceSupportContactEpsilonCm.GetValueOnAnyThread())
        : bVertexRecoverCandidate
        ? FMath::Max(ContactTol, CVarIAmSpeedVertexSupportContactTolCm.GetValueOnAnyThread())
        : ContactTol;
    const float ActivePenTolSupport = bNearlyFaceAligned || bRoofSurfaceTraversalActive
        ? FMath::Max(PenTolSupport, CVarIAmSpeedFaceSupportPenetrationTolCm.GetValueOnAnyThread())
        : bEdgeRecoverCandidate
        ? FMath::Max(PenTolSupport, CVarIAmSpeedEdgeRecoverPenetrationTolCm.GetValueOnAnyThread())
        : PenTolSupport;
    constexpr float RoofTraversalSeparatingSpeedTolerance = 20.f;
    const float ActiveSepVelTol = bRoofSurfaceTraversalActive
        ? FMath::Max(
            RoofTraversalSeparatingSpeedTolerance,
            CVarIAmSpeedFaceSupportSeparatingSpeedCmS.GetValueOnAnyThread())
        : bNearlyFaceAligned
        ? FMath::Max(SepVelTol, CVarIAmSpeedFaceSupportSeparatingSpeedCmS.GetValueOnAnyThread())
        : bEdgeRecoverCandidate
        ? FMath::Max(SepVelTol, CVarIAmSpeedEdgeRecoverSeparatingSpeedCmS.GetValueOnAnyThread())
        : SepVelTol;

    constexpr float MaxRoofTraversalDrift = 25.f;
    if (bRoofSurfaceTraversalActive &&
        minDist > ActiveContactTol &&
        minDist <= MaxRoofTraversalDrift)
    {
        constexpr float TargetRoofTraversalDistance = 0.5f;
        ParentComponent->SetPhysLocation(
            ParentComponent->GetPhysLocation() -
            N * (minDist - TargetRoofTraversalDistance));
        minDist = TargetRoofTraversalDistance;
    }
    else if (bRoofSurfaceTraversalActive &&
        minDist < -ActivePenTolSupport &&
        minDist >= -MaxRoofTraversalDrift)
    {
        constexpr float TargetRoofTraversalPenetration = -0.5f;
        ParentComponent->SetPhysLocation(
            ParentComponent->GetPhysLocation() +
            N * (TargetRoofTraversalPenetration - minDist));
        minDist = TargetRoofTraversalPenetration;
    }

    const bool bWithinSupportBand =
        (minDist <= ActiveContactTol) &&
        (minDist >= -ActivePenTolSupport);

    const bool bNotExploding =
        (minDist >= -PenTolReject);

    const float MeasuredSupportSeparatingSpeed = bRoofSurfaceTraversalActive
        ? FVector::DotProduct(ParentComponent->GetPhysCOMVelocity(), N)
        : worstVN;
    const float SupportSeparatingSpeed =
        bRoofSurfaceTraversalActive && bDirectSupportSolved
        ? FMath::Min(MeasuredSupportSeparatingSpeed, 0.f)
        : MeasuredSupportSeparatingSpeed;
    const bool bStableNormalMotion =
        (SupportSeparatingSpeed <= ActiveSepVelTol);

#if !UE_BUILD_SHIPPING
    if (CVarIAmSpeedAutoRecoverContactDebug.GetValueOnAnyThread() != 0 &&
        (CurrentGroundContactsWS.Num() > 0 || PrevGroundContactsLS.Num() > 0 || bFreshEdgeRecoverCandidate))
    {
        UE_LOG(
            BoxSubBodyLog,
            Log,
            TEXT("[ARContact.PersistentCheck] Frame=%d Current=%d PrevLS=%d FreshEdge=%d EdgeRecoverActive=%d VertexCandidate=%d RoofTraversal=%d FaceDot=%.5f minDist=%.3f ContactTol=%.3f PenTol=%.3f worstVN=%.2f SupportSep=%.2f SepVelTol=%.2f WithinBand=%d StableMotion=%d NotExploding=%d PlaneN=%s"),
            ParentComponent ? ParentComponent->NumFrame() : INDEX_NONE,
            CurrentGroundContactsWS.Num(),
            PrevGroundContactsLS.Num(),
            bFreshEdgeRecoverCandidate ? 1 : 0,
            bEdgeRecoverActive ? 1 : 0,
            bVertexRecoverCandidate ? 1 : 0,
            bRoofSurfaceTraversalActive ? 1 : 0,
            FaceAlignmentAbs,
            minDist,
            ActiveContactTol,
            ActivePenTolSupport,
            worstVN,
            SupportSeparatingSpeed,
            ActiveSepVelTol,
            bWithinSupportBand ? 1 : 0,
            bStableNormalMotion ? 1 : 0,
            bNotExploding ? 1 : 0,
            *GroundPlaneN.ToString());
    }
#endif

    if (!bNearlyFaceAligned && bFreshEdgeRecoverCandidate && CurrentGroundContactsWS.Num() == 2)
    {
        const FVector& P0 = CurrentGroundContactsWS[0];
        const FVector& P1 = CurrentGroundContactsWS[1];
        if ((P1 - P0).SizeSquared() > FMath::Square(1.0f))
        {
#if !UE_BUILD_SHIPPING
            if (CVarIAmSpeedAutoRecoverContactDebug.GetValueOnAnyThread() != 0)
            {
                UE_LOG(
                    BoxSubBodyLog,
                    Log,
                    TEXT("[ARContact.PersistentFreshEdgeAccepted] Frame=%d P0=%s P1=%s PlaneN=%s GroundComp=%s"),
                    ParentComponent ? ParentComponent->NumFrame() : INDEX_NONE,
                    *P0.ToString(),
                    *P1.ToString(),
                    *GroundPlaneN.ToString(),
                    GroundComp.IsValid() ? *GroundComp->GetName() : TEXT("None"));
            }
#endif
            bHasGroundContact = true;
            bEdgeSupportLatched = true;
            EdgeSupportLatchFrame = ParentComponent->NumFrame();

            LatchedEdgeContactsLS.Reset(2);
            LatchedEdgeContactsLS.Add(K.Rotation.UnrotateVector(P0 - K.Location));
            LatchedEdgeContactsLS.Add(K.Rotation.UnrotateVector(P1 - K.Location));

            PrevGroundContactsLS.Reset(2);
            PrevGroundContactsLS.Add(LatchedEdgeContactsLS[0]);
            PrevGroundContactsLS.Add(LatchedEdgeContactsLS[1]);

            GroundHit = SHitResult();
            GroundHit.bBlockingHit = true;
            GroundHit.Component = GroundComp;
            GroundHit.ImpactNormal = GroundPlaneN;
            GroundHit.TOI = 0.f;
            GroundHit.ImpactPoint = 0.5f * (P0 + P1);
            PreserveSupportProvider(GroundHit);
            return;
        }
    }

    if (!(bWithinSupportBand && bStableNormalMotion && bNotExploding))
    {
#if !UE_BUILD_SHIPPING
        if (CVarIAmSpeedAutoRecoverContactDebug.GetValueOnAnyThread() != 0)
        {
            UE_LOG(
                BoxSubBodyLog,
                Log,
                TEXT("[ARContact.PersistentReject] Frame=%d Reason=BandOrMotion Current=%d PrevLS=%d FreshEdge=%d VertexCandidate=%d minDist=%.3f ContactTol=%.3f PenTol=%.3f worstVN=%.2f SepVelTol=%.2f WithinBand=%d StableMotion=%d NotExploding=%d"),
                ParentComponent ? ParentComponent->NumFrame() : INDEX_NONE,
                CurrentGroundContactsWS.Num(),
                PrevGroundContactsLS.Num(),
                bFreshEdgeRecoverCandidate ? 1 : 0,
                bVertexRecoverCandidate ? 1 : 0,
                minDist,
                ActiveContactTol,
                ActivePenTolSupport,
                worstVN,
                ActiveSepVelTol,
                bWithinSupportBand ? 1 : 0,
                bStableNormalMotion ? 1 : 0,
                bNotExploding ? 1 : 0);
        }
#endif
        // An upright bottom-face manifold rejected on a nearly horizontal
        // surface is not a persistent support candidate. Keeping its plane
        // valid lets a following gutter-wall hit rebuild the stale four-point
        // manifold and cancel gravity after all contacts vanish. This covers
        // separation, excessive penetration, and an invalid support band.
        // Keep it narrow: roof traversal and wall landings rely on their own
        // support hysteresis.
        const float PlaneUpDot = FVector::DotProduct(N, FVector::UpVector);
        const float CarUpToPlaneDot =
            FVector::DotProduct(K.Rotation.GetUpVector(), N);
        if (bNearlyFaceAligned &&
            CurrentGroundContactsWS.Num() >= 3 &&
            PlaneUpDot >= 0.95f &&
            CarUpToPlaneDot >= 0.9f)
        {
            bHasGroundContact = false;
            bGroundPlaneValid = false;
            bGroundContactStable = false;
            StableTime = 0.f;
            CurrentGroundContactsWS.Reset();
        }
        bEdgeSupportLatched = false;
        LatchedEdgeContactsLS.Reset();
        PrevGroundContactsLS.Reset();
        return;
    }
    // If clearly separating, drop persistence. Near a face support we keep a
    // wider hysteresis band. Vertex auto-recover also needs a small grace band,
    // otherwise tiny upward drift erases the single point before it can fall to
    // an edge or face.
    const float VertexSeparatingReleaseSpeed =
        FMath::Max(0.5f, CVarIAmSpeedVertexSupportSeparatingSpeedCmS.GetValueOnAnyThread());
    const float SeparatingReleaseSpeed =
        (bNearlyFaceAligned || bRoofSurfaceTraversalActive) ? ActiveSepVelTol :
        bVertexRecoverCandidate ? VertexSeparatingReleaseSpeed :
        bEdgeRecoverCandidate ? ActiveSepVelTol :
        0.5f;
    if (SupportSeparatingSpeed > SeparatingReleaseSpeed)
    {
#if !UE_BUILD_SHIPPING
        if (CVarIAmSpeedAutoRecoverContactDebug.GetValueOnAnyThread() != 0)
        {
            UE_LOG(
                BoxSubBodyLog,
                Log,
                TEXT("[ARContact.PersistentReject] Frame=%d Reason=Separating worstVN=%.2f ReleaseSpeed=%.2f VertexCandidate=%d FaceDot=%.5f Current=%d PrevLS=%d"),
                ParentComponent ? ParentComponent->NumFrame() : INDEX_NONE,
                worstVN,
                SeparatingReleaseSpeed,
                bVertexRecoverCandidate ? 1 : 0,
                FaceAlignmentAbs,
                CurrentGroundContactsWS.Num(),
                PrevGroundContactsLS.Num());
        }
#endif
        bHasGroundContact = false;
        bGroundPlaneValid = false;
        bEdgeSupportLatched = false;
        CurrentGroundContactsWS.Reset();
        LatchedEdgeContactsLS.Reset();
        PrevGroundContactsLS.Reset();
        return;
    }

    // ------------------------------------------------------------
    // Rebuild support manifold from vertices
    // ------------------------------------------------------------
    bHasGroundContact = true;

    if (bRoofSurfaceTraversalActive)
    {
        LastRoofSurfaceContactFrame = CurrentFrame;
    }

    CurrentGroundContactsWS.Reset();
    const float ContactEps = bNearlyFaceAligned || bRoofSurfaceTraversalActive
        ? FMath::Max(0.5f, CVarIAmSpeedFaceSupportContactEpsilonCm.GetValueOnAnyThread())
        : 0.5f;

    for (const FVector& V : Verts)
    {
        const float dist = PlaneSignedDist(V, GroundPlaneN, GroundPlanePointWS);
        if (dist <= minDist + ContactEps)
            CurrentGroundContactsWS.Add(V);
    }
    if (bRoofSurfaceTraversalActive && !bNearlyFaceAligned)
    {
        IAmSpeedBuildRoofTransitionEdge(N, K.Location, K.Rotation, BoxExtent, CurrentGroundContactsWS);
    }
    if (bNearlyFaceAligned)
    {
        IAmSpeedBuildSupportFaceManifoldFromNormal(
            N,
            K.Location,
            K.Rotation,
            BoxExtent,
            CurrentGroundContactsWS,
            FaceReleaseDot);
    }

    // ------------------------------------------------------------
    // EDGE PERSISTENCE HYSTERESIS (local-space robust)
    // ------------------------------------------------------------
    if (!bNearlyFaceAligned && !bEdgeRecoverActive && PrevGroundContactsLS.Num() == 2 && CurrentGroundContactsWS.Num() == 1)
    {
        const FVector& P = CurrentGroundContactsWS[0];

        // Rebuild previous edge in CURRENT pose using LOCAL stored endpoints
        const FVector A = K.Location + K.Rotation.RotateVector(PrevGroundContactsLS[0]);
        const FVector B = K.Location + K.Rotation.RotateVector(PrevGroundContactsLS[1]);

        const FVector Edge = (B - A);
        const float EdgeLen2 = Edge.SizeSquared();

        if (EdgeLen2 > KINDA_SMALL_NUMBER)
        {
            const FVector EdgeDir = Edge / FMath::Sqrt(EdgeLen2);

            // Project P on the edge line
            const float t = FVector::DotProduct(P - A, EdgeDir);
            const FVector Pproj = A + t * EdgeDir;

            // Snap tolerance should beat per-frame drift + tiny noise
            // (minimum 2cm, but also allow translation magnitude per frame)
            const float vMag = K.Velocity.Size();
            const float dynTol = FMath::Max(2.0f, 1.25f * vMag * Dt); // cm
            const float tol2 = dynTol * dynTol;

            if (FVector::DistSquared(P, Pproj) <= tol2)
            {
                CurrentGroundContactsWS.Add(Pproj);

#if !UE_BUILD_SHIPPING
                /*UE_LOG(BoxSubBodyLog, Log,
                    TEXT("[EdgePersistence] Promoted vertex -> edge | dynTol=%.2f P=%s Pproj=%s"),
                    dynTol, *P.ToString(), *Pproj.ToString());*/
#endif
            }
#if !UE_BUILD_SHIPPING
            else
            {
                /*UE_LOG(BoxSubBodyLog, Log,
                    TEXT("[EdgePersistence] Rejected | dynTol=%.2f dist=%.3f P=%s Pproj=%s"),
                    dynTol, FMath::Sqrt(FVector::DistSquared(P, Pproj)),
                    *P.ToString(), *Pproj.ToString());*/
            }
#endif
        }
    }

    // Latch edge contacts if we have exactly 2 contacts this frame
    if (!bNearlyFaceAligned && !bEdgeRecoverActive && CurrentGroundContactsWS.Num() == 2)
    {
        const FVector& P0 = CurrentGroundContactsWS[0];
        const FVector& P1 = CurrentGroundContactsWS[1];

        const float EdgeLenWS = (P1 - P0).Size();

        constexpr float MinLatchEdgeLen = 1.0f; // cm (tunable)

        if (EdgeLenWS > MinLatchEdgeLen)
        {
            LatchedEdgeContactsLS.Reset(2);
            LatchedEdgeContactsLS.Add(
                K.Rotation.UnrotateVector(P0 - K.Location)
            );
            LatchedEdgeContactsLS.Add(
                K.Rotation.UnrotateVector(P1 - K.Location)
            );

            bEdgeSupportLatched = true;
            EdgeSupportLatchFrame = ParentComponent->NumFrame();
        }
#if !UE_BUILD_SHIPPING
        else
        {
            /*UE_LOG(
                BoxSubBodyLog,
                Log,
                TEXT("[EdgeLatch] Rejected degenerate edge (len=%.4f cm)"),
                EdgeLenWS
            );*/
        }
#endif
    }

    // ------------------------------------------------------------
    // Store previous manifold for next frame
    // ------------------------------------------------------------
    PrevGroundContactsLS.Reset();
    PrevGroundContactsLS.Reserve(CurrentGroundContactsWS.Num());

    for (const FVector& Pw : CurrentGroundContactsWS)
    {
        // local around COM frame: (Pw - K.Location) in hitbox local
        PrevGroundContactsLS.Add(K.Rotation.UnrotateVector(Pw - K.Location));
    }
    if (!bNearlyFaceAligned && PrevGroundContactsLS.Num() == 2)
    {
        bEdgeSupportLatched = true;
        EdgeSupportLatchFrame = ParentComponent->NumFrame();
    }

    // ------------------------------------------------------------
    // Rebuild GroundHit coherently
    // ------------------------------------------------------------
    const FVector& Location = K.Location;
    const FVector Pbox =
        CurrentGroundContactsWS.Num() ?
        CurrentGroundContactsWS[0] :
        Location;

    const float dist = PlaneSignedDist(Pbox, GroundPlaneN, GroundPlanePointWS);
    const FVector Pplane = Pbox - dist * GroundPlaneN;

    GroundHit = SHitResult();
    GroundHit.bBlockingHit = true;
    GroundHit.Component = GroundComp;
    GroundHit.ImpactNormal = GroundPlaneN;
    GroundHit.TOI = 0.f;
    // GroundHit.Location = K.Location;
    GroundHit.ImpactPoint = Pplane;
    PreserveSupportProvider(GroundHit);
}

bool UBoxSubBody::CanResolveRepeatedContact(const SHitResult& Hit) const
{
    return ParentComponent && ParentComponent->GetStaticCollisionWorldForFrame() && Hit.TOI > 0 &&
        Hit.SourceId != 0 && !Hit.bSurfaceNormalMayVary && Hit.GeometricErrorBoundCm == 0 &&
        Hit.ContactFeatureThis == Speed::EContactFeatureKind::Vertex && Hit.ContactFeatureIndexThis >= 0 &&
        Hit.ContactFeatureOther == Speed::EContactFeatureKind::Face;
}

bool UBoxSubBody::TryGetStaticContactAcceleration(FVector& Linear, FVector& Angular) const
{
    Linear = Angular = FVector::ZeroVector;
    if (ParentComponent && ParentComponent->HasActivePhysicalConstraintsOtherThan(this)) return false;
    const Speed::IStaticCollisionWorld* World = ParentComponent ? ParentComponent->GetStaticCollisionWorldForFrame() : nullptr;
    if (!World || !HasToApplyRestForce() || !CurrentHit.bBlockingHit ||
        CurrentHit.SourceId == 0 || CurrentHit.bSurfaceNormalMayVary || CurrentHit.GeometricErrorBoundCm != 0 ||
        CurrentHit.ContactFeatureOther != Speed::EContactFeatureKind::Face || !CurrentHit.Component.IsValid() ||
        CurrentHit.Component->GetCollisionObjectType() != ECC_WorldStatic) return false;
    const SKinematic& State = ParentComponent->GetKinematicState();
    const FVector N = CurrentHit.ImpactNormal;
    const FVector Center = State.Location + State.Rotation.RotateVector(GetLocalOffset() - ParentComponent->GetPhysCenterOfMassLocal());
    const FQuat Q = State.Rotation * GetLocalRotation();
    const double Roundoff = 64 * DBL_EPSILON * FMath::Max(1.0, Center.GetAbsMax() + BoxExtent.GetMax());
    const double SpeedRoundoff = 4096 * DBL_EPSILON * FMath::Max(1.0, State.Velocity.Size() + State.AngularVelocity.Size() * BoxExtent.Size());
    FVector Points[4];
    double Centripetal[4];
    int32 Count = 0;
    bool bSticking = true;
    FVector CommonCentripetal = FVector::ZeroVector;
    Speed::Analytic::FWorldQuery Probe;
    Probe.Shape = Speed::Analytic::EQueryShape::Ray;
    Probe.RequiredSourceId = CurrentHit.SourceId; Probe.RequiredSurfaceId = CurrentHit.SurfaceId;
    Probe.bAuthorityOnly = true; Probe.bIncludeCompactPatches = true; Probe.bApplyCollisionFilter = true;
    Probe.TraceChannel = static_cast<uint8>(GetCollisionChannel());
    Probe.BlockingObjectTypes = Speed::GetBlockingResponseMask(GetResponseParams().CollisionResponse);
    Probe.DomainTolerance = 0; Probe.InitialOverlapTolerance = 0;
    for (int32 I = 0; I < 8; ++I)
    {
        const FVector P = Center + Q.RotateVector(FVector((I & 1) ? BoxExtent.X : -BoxExtent.X,
            (I & 2) ? BoxExtent.Y : -BoxExtent.Y, (I & 4) ? BoxExtent.Z : -BoxExtent.Z));
        const double Gap = FVector::DotProduct(P - CurrentHit.ImpactPoint, N);
        if (Gap < -Roundoff) return false;
        if (Gap > Roundoff) continue;
        const FVector R = P - State.Location;
        const FVector PointVelocity = State.Velocity + FVector::CrossProduct(State.AngularVelocity, R);
        const double VN = FVector::DotProduct(PointVelocity, N);
        if (VN < -SpeedRoundoff) return false; // Incoming impact is not a sustained load.
        if (VN > SpeedRoundoff) continue; // A separating vertex cannot pull the body back.
        if (Count == 4) return false;
        Probe.Start = P + N * 0.01; Probe.End = P - N * 0.01;
        const auto Witness = World->SweepSingle(Probe);
        if (!Witness.bHit || Witness.bStartPenetrating || Witness.bSurfaceNormalMayVary ||
            Witness.GeometricErrorBoundCm != 0 || Witness.SurfaceFeatureKind != Speed::EContactFeatureKind::Face ||
            !Witness.Normal.Equals(N, 32 * DBL_EPSILON)) return false;
        const FVector PointCentripetal = FVector::CrossProduct(State.AngularVelocity,
            FVector::CrossProduct(State.AngularVelocity, R));
        if (Count == 0) CommonCentripetal = PointCentripetal;
        bSticking &= (PointVelocity - VN * N).Size() <= SpeedRoundoff &&
            (PointCentripetal - CommonCentripetal).Size() <= 256 * DBL_EPSILON *
                FMath::Max(1.0, State.Acceleration.Size() + PointCentripetal.Size());
        Points[Count] = P;
        Centripetal[Count++] = FVector::DotProduct(PointCentripetal, N);
    }
    if (Count == 0 || ParentComponent->GetPhysMass() <= 0) return false;
    const FMatrix InvI = RotateLocalInverseInertia(Q);
    if (Count == 4)
    {
        // A face can slide and spin around its normal without rocking. Keep
        // its continuous reaction rather than recreating impacts each frame.
        const auto Response = Speed::SolvePlanarSlidingReaction(MakeArrayView(Points, Count), N,
            State.Location, State.Velocity, State.AngularVelocity, State.Acceleration,
            State.AngularAcceleration, 1.0 / ParentComponent->GetPhysMass(), InvI,
            GetDynamicFriction(), ParentComponent->GetStaticSupportFrameHorizon());
        if (!Response.bSolved) return false;
        Linear = Response.DeltaVelocity; Angular = Response.DeltaAngularVelocity;
        return true;
    }
    // The same contact Jacobian maps force to acceleration and impulse to
    // velocity. The curvature term is essential when rotating about an edge.
    // A sticking vertex/edge needs tangential forces as well as its normal
    // reaction. For edge-axis rotation both points have the same centripetal
    // acceleration, so the impulse Jacobian can solve the complete load using
    // acceleration units. Normal-only support would let the edge slip between
    // impacts and repeatedly recreate the very motion friction just removed.
    const auto Response = bSticking && Count <= 2
        ? Speed::SolvePlanarCoulombImpulse(MakeArrayView(Points, Count), N, State.Location,
            State.Acceleration + CommonCentripetal, State.AngularAcceleration,
            1.0 / ParentComponent->GetPhysMass(), InvI, 0, GetDynamicFriction())
        : Speed::SolvePlanarNormalImpulse(MakeArrayView(Points, Count), N, State.Location,
            State.Acceleration, State.AngularAcceleration, 1.0 / ParentComponent->GetPhysMass(), InvI, 0,
            MakeArrayView(Centripetal, Count));
    if (!Response.bSolved) return false;
    Linear = Response.DeltaVelocity;
    Angular = Response.DeltaAngularVelocity;
    return !Linear.IsZero() || !Angular.IsZero();
}

bool UBoxSubBody::RefineExactPlanarTOI(SHitResult& Hit, float Delta) const
{
    const SKinematic Initial = ParentComponent->GetKinematicState();
    const FVector N = Hit.ImpactNormal;
    const FVector LocalCOM = ParentComponent->GetPhysCenterOfMassLocal();
    const FVector LocalCenter = GetLocalOffset();
    const FQuat LocalQ = GetLocalRotation();
    const auto PoseAt = [&](float Time)
    {
        SKinematic Pose = Initial.Integrate(Time);
        Pose.Location -= Pose.Rotation.RotateVector(LocalCOM);
        Pose.Location += Pose.Rotation.RotateVector(LocalCenter);
        Pose.Rotation = Pose.Rotation * LocalQ;
        return Pose;
    };
    FVector Corners[8];
    double InitialGaps[8], InitialNormalSpeeds[8], MinimumInitial = DBL_MAX;
    const SKinematic StartPose = PoseAt(0);
    const double Roundoff = 64 * DBL_EPSILON * FMath::Max(1.0, StartPose.Location.GetAbsMax() + BoxExtent.GetMax());
    const double SpeedRoundoff = 4096 * DBL_EPSILON * FMath::Max(1.0,
        Initial.Velocity.Size() + Initial.AngularVelocity.Size() * BoxExtent.Size());
    uint8 EstablishedMask = 0;
    // Adjacent/overlapping providers may represent the same supporting plane.
    // Primitive identity is not a new physical impact: a duplicate time-zero
    // event would win over the next arriving vertex, then be pair-suppressed
    // by the world and hide that real collision. Only identify exact coplanar
    // geometry here; each arriving corner still needs its finite-domain ray.
    const bool bEstablished = CurrentHit.bBlockingHit &&
        LastResolvedGroundHitFrame == static_cast<int32>(ParentComponent->NumFrame()) &&
        CurrentHit.SourceId != 0 && !CurrentHit.bSurfaceNormalMayVary && CurrentHit.GeometricErrorBoundCm == 0 &&
        CurrentHit.ContactFeatureOther == Speed::EContactFeatureKind::Face &&
        CurrentHit.ImpactNormal.Equals(N, 32 * DBL_EPSILON) &&
        FMath::Abs(FVector::DotProduct(CurrentHit.ImpactPoint - Hit.ImpactPoint, N)) <= Roundoff;
    for (int32 I = 0; I < 8; ++I)
    {
        Corners[I] = FVector((I & 1) ? BoxExtent.X : -BoxExtent.X,
            (I & 2) ? BoxExtent.Y : -BoxExtent.Y, (I & 4) ? BoxExtent.Z : -BoxExtent.Z);
        const FVector Vertex = StartPose.Location + StartPose.Rotation.RotateVector(Corners[I]);
        InitialGaps[I] = FVector::DotProduct(Vertex - Hit.ImpactPoint, N);
        InitialNormalSpeeds[I] = FVector::DotProduct(Initial.Velocity +
            FVector::CrossProduct(Initial.AngularVelocity, Vertex - Initial.Location), N);
        MinimumInitial = FMath::Min(MinimumInitial, InitialGaps[I]);
        // Geometric touching alone is not sustained support: the last impulse
        // can lift half a face. Keep those separating vertices eligible for a
        // later return in this very frame instead of masking their next impact.
        if (bEstablished && InitialGaps[I] <= Roundoff && InitialNormalSpeeds[I] <= SpeedRoundoff)
            EstablishedMask |= 1u << I;
    }
    const auto GapAt = [&](int32 Corner, float Time)
    {
        const SKinematic Pose = PoseAt(Time);
        double SupportGap = 0;
        for (int32 I = 0; I < 8; ++I)
            if (EstablishedMask & (1u << I)) SupportGap = FMath::Min(SupportGap,
                FVector::DotProduct(Pose.Location + Pose.Rotation.RotateVector(Corners[I]) - Hit.ImpactPoint, N));
        return FVector::DotProduct(Pose.Location + Pose.Rotation.RotateVector(Corners[Corner]) - Hit.ImpactPoint, N) - SupportGap;
    };
    int32 FirstCorner = INDEX_NONE;
    float FirstTime = Delta;
    const bool bInitialEvent = !bEstablished && MinimumInitial <= Roundoff;
    if (bInitialEvent)
    {
        for (int32 I = 0; I < 8; ++I)
            if (InitialGaps[I] == MinimumInitial) { FirstCorner = I; break; }
        FirstTime = 0;
    }
    for (int32 Corner = 0; Corner < 8; ++Corner)
    {
        if (bInitialEvent) break;
        if ((EstablishedMask & (1u << Corner)) ||
            (InitialGaps[Corner] <= Roundoff && InitialNormalSpeeds[Corner] <= SpeedRoundoff) ||
            GapAt(Corner, Delta) > 0) continue;
        float Low = 0, High = Delta;
        bool bCertifiedDeparture = InitialGaps[Corner] > Roundoff;
        for (int32 Iteration = 0; Iteration < 32; ++Iteration)
        {
            const float Mid = Low + (High - Low) * 0.5f;
            if (Mid == Low || Mid == High) break;
            const double Gap = GapAt(Corner, Mid);
            bCertifiedDeparture |= Gap > Roundoff;
            if (Gap >= 0) Low = Mid; else High = Mid;
        }
        // A returning vertex must have actually left the plane. Roundoff-only
        // zero gaps otherwise generate repeated positive-time "arrivals" with
        // no resolvable excursion or progress in the canonical frame clock.
        if (!bCertifiedDeparture) continue;
        if (FirstCorner == INDEX_NONE || Low < FirstTime) { FirstCorner = Corner; FirstTime = Low; }
    }
 #if !UE_BUILD_SHIPPING
    if (CVarIAmSpeedAutoRecoverContactDebug.GetValueOnAnyThread() != 0 &&
        ParentComponent->NumFrame() >= 129 && ParentComponent->NumFrame() <= 132)
        UE_LOG(BoxSubBodyLog, Display, TEXT("[ExactPlanarArrival] frame=%u dt=%.17g established=%d mask=%u corner=%d time=%.17g source=%llu primitive=%llu current_primitive=%llu plane_z=%.17g q=%s a=%s alpha=%s gaps=(%.17g,%.17g,%.17g,%.17g;%.17g,%.17g,%.17g,%.17g)"),
            ParentComponent->NumFrame(), double(Delta), bEstablished ? 1 : 0, EstablishedMask, FirstCorner, double(FirstTime),
            Hit.SourceId, Hit.PrimitiveId, CurrentHit.PrimitiveId, double(Hit.ImpactPoint.Z), *Initial.Rotation.ToString(),
            *Initial.Acceleration.ToString(), *Initial.AngularAcceleration.ToString(),
            InitialGaps[4], InitialGaps[5], InitialGaps[6], InitialGaps[7],
            GapAt(4, Delta), GapAt(5, Delta), GapAt(6, Delta), GapAt(7, Delta));
 #endif
    if (FirstCorner == INDEX_NONE) return false;
    const SKinematic Pose = PoseAt(FirstTime);
    const FVector Vertex = Pose.Location + Pose.Rotation.RotateVector(Corners[FirstCorner]);
    const FVector Projected = Vertex - FVector::DotProduct(Vertex - Hit.ImpactPoint, N) * N;
    Speed::Analytic::FWorldQuery Probe;
    Probe.Shape = Speed::Analytic::EQueryShape::Ray;
    Probe.Start = Projected + N * 0.01; Probe.End = Projected - N * 0.01;
    Probe.RequiredSourceId = Hit.SourceId; Probe.RequiredSurfaceId = Hit.SurfaceId;
    Probe.bAuthorityOnly = true; Probe.bIncludeCompactPatches = true;
    Probe.bApplyCollisionFilter = true;
    Probe.TraceChannel = static_cast<uint8>(GetCollisionChannel());
    Probe.BlockingObjectTypes = Speed::GetBlockingResponseMask(GetResponseParams().CollisionResponse);
    Probe.DomainTolerance = 0; Probe.InitialOverlapTolerance = 0;
    const auto Witness = ParentComponent->GetStaticCollisionWorldForFrame()->SweepSingle(Probe);
    if (!Witness.bHit || Witness.bStartPenetrating || Witness.bSurfaceNormalMayVary ||
        Witness.SurfaceFeatureKind != Speed::EContactFeatureKind::Face || Witness.GeometricErrorBoundCm != 0 ||
        !Witness.Normal.Equals(N, 32 * DBL_EPSILON)) return false;
    Hit.TOI = FirstTime;
    Hit.Location = Pose.Location;
    Hit.ImpactPoint = Witness.Point;
    Hit.ContactPointThis = Vertex;
    Hit.ContactPointOther = Witness.Point;
    Hit.ContactFeatureThis = Speed::EContactFeatureKind::Vertex;
    Hit.ContactFeatureIndexThis = static_cast<int8>(FirstCorner);
    return true;
}

void UBoxSubBody::ProjectPlanarEventRoundoff(SKinematic& State, const SHitResult& Hit) const
{
    if (Hit.TOI <= 0 || Hit.ContactFeatureThis != Speed::EContactFeatureKind::Vertex ||
        Hit.ContactFeatureIndexThis < 0 || State.AngularVelocity.IsZero()) return;
    const FVector N = Hit.ImpactNormal;
    const FQuat BoxQ = State.Rotation * GetLocalRotation();
    const FVector Axes[] = { BoxQ.GetAxisX(), BoxQ.GetAxisY(), BoxQ.GetAxisZ() };
    // Constrained transport may already have impulsed the incoming spin before
    // AcceptHit. Its current magnitude is not an upper bound on sweep motion;
    // use the owner's enforced speed ceiling for the float-time error bound.
    // Retain the local impact-time bound for rocking. The unconstrained
    // normal-axis spin also transports the pose through earlier float-time
    // segments: its accumulated roundoff is not bounded by the last tiny TOI.
    const double SpinTransportRoundoff = 4.0 * FLT_EPSILON *
        ParentComponent->GetStaticSupportFrameHorizon() * FMath::Abs(FVector::DotProduct(State.AngularVelocity, N));
    const double AngularRoundoff = 4.0 * FLT_EPSILON * Hit.TOI *
        (ParentComponent->GetPhysMaxAngularSpeed() + State.AngularAcceleration.Size() * Hit.TOI) +
        SpinTransportRoundoff + 64 * DBL_EPSILON;
    FVector From = FVector::ZeroVector, To = FVector::ZeroVector;
    // A face event has priority over its constituent edges. Cross-product
    // error remains informative where acos(dot) would already round to zero.
    for (const FVector& Axis : Axes)
    {
        const double Dot = FVector::DotProduct(Axis, N);
        if (FMath::Abs(Dot) > 0.9 && FVector::CrossProduct(Axis, N).Size() <= AngularRoundoff)
        { From = Dot > 0 ? Axis : -Axis; To = N; break; }
    }
    if (From.IsZero())
    {
        for (const FVector& Axis : Axes)
        {
            const double Dot = FVector::DotProduct(Axis, N);
            if (FMath::Abs(Dot) <= AngularRoundoff)
            { From = Axis; To = (Axis - Dot * N).GetSafeNormal(); break; }
        }
    }
    if (From.IsZero() || To.IsZero()) return;
    const FVector Cross = FVector::CrossProduct(From, To);
    const FQuat Correction(Cross.X, Cross.Y, Cross.Z, 1 + FVector::DotProduct(From, To));
    SKinematic Candidate = State;
    Candidate.Rotation = (Correction.GetNormalized() * State.Rotation).GetNormalized();
    const FVector Center = Candidate.Location - Candidate.Rotation.RotateVector(ParentComponent->GetPhysCenterOfMassLocal()) +
        Candidate.Rotation.RotateVector(GetLocalOffset());
    const FQuat CandidateQ = Candidate.Rotation * GetLocalRotation();
    double Gap = DBL_MAX;
    for (int32 I = 0; I < 8; ++I)
    {
        const FVector P((I & 1) ? BoxExtent.X : -BoxExtent.X,
            (I & 2) ? BoxExtent.Y : -BoxExtent.Y, (I & 4) ? BoxExtent.Z : -BoxExtent.Z);
        Gap = FMath::Min(Gap, FVector::DotProduct(Center + CandidateQ.RotateVector(P) - Hit.ImpactPoint, N));
    }
    const double Scale = FMath::Max(1.0, Center.GetAbsMax() + BoxExtent.GetMax());
    const double PositionRoundoff = AngularRoundoff * BoxExtent.Size() +
        4.0 * FLT_EPSILON * Hit.TOI * State.Velocity.Size() + 128 * DBL_EPSILON * Scale;
    if (FMath::Abs(Gap) > PositionRoundoff) return;
    Candidate.Location += (-Gap + 2.0 * DBL_EPSILON * Scale) * N;
    State = Candidate;
}

bool UBoxSubBody::TryResolveExactPlanarImpact(bool bNotifyImpact)
{
    // This kernel uses free-body inverse inertia. Existing constraints from
    // other sub-bodies belong to the coupled response, not a later correction
    // applied after committing an incompatible independent box impulse.
    if (ParentComponent->HasActivePhysicalConstraintsOtherThan(this)) return false;
    const Speed::IStaticCollisionWorld* World = ParentComponent->GetStaticCollisionWorldForFrame();
    if (!World || !HasToApplyRestForce() || CurrentHit.SourceId == 0 ||
        CurrentHit.bSurfaceNormalMayVary || CurrentHit.GeometricErrorBoundCm != 0 ||
        CurrentHit.ContactFeatureOther != Speed::EContactFeatureKind::Face ||
        !CurrentHit.Component.IsValid() || CurrentHit.Component->GetCollisionObjectType() != ECC_WorldStatic)
        return false;
    SKinematic State = ParentComponent->GetKinematicState();
    if (bNotifyImpact) ProjectPlanarEventRoundoff(State, CurrentHit);
    const FVector BoxCenter = State.Location - State.Rotation.RotateVector(ParentComponent->GetPhysCenterOfMassLocal()) +
        State.Rotation.RotateVector(GetLocalOffset());
    const FQuat BoxQ = State.Rotation * GetLocalRotation();
    const FVector N = CurrentHit.ImpactNormal;
    const double Scale = FMath::Max(1.0, BoxCenter.GetAbsMax() + BoxExtent.GetMax());
    const double Roundoff = 64.0 * DBL_EPSILON * Scale;
    FVector Vertices[8];
    double Gaps[8], MinimumGap = DBL_MAX;
    for (int32 Corner = 0; Corner < 8; ++Corner)
    {
        const FVector P((Corner & 1) ? BoxExtent.X : -BoxExtent.X,
            (Corner & 2) ? BoxExtent.Y : -BoxExtent.Y, (Corner & 4) ? BoxExtent.Z : -BoxExtent.Z);
        Vertices[Corner] = BoxCenter + BoxQ.RotateVector(P);
        Gaps[Corner] = FVector::DotProduct(Vertices[Corner] - CurrentHit.ImpactPoint, N);
        MinimumGap = FMath::Min(MinimumGap, Gaps[Corner]);
    }
    FVector Points[4];
    int32 Count = 0;
    Speed::Analytic::FWorldQuery Probe;
    Probe.Shape = Speed::Analytic::EQueryShape::Ray;
    Probe.RequiredSourceId = CurrentHit.SourceId;
    Probe.RequiredSurfaceId = CurrentHit.SurfaceId;
    Probe.TraceChannel = static_cast<uint8>(GetCollisionChannel());
    Probe.BlockingObjectTypes = Speed::GetBlockingResponseMask(GetResponseParams().CollisionResponse);
    Probe.bApplyCollisionFilter = true;
    Probe.bAuthorityOnly = true;
    Probe.bIncludeCompactPatches = true;
    Probe.DomainTolerance = 0;
    Probe.InitialOverlapTolerance = 0;
    for (int32 Corner = 0; Corner < 8; ++Corner)
    {
        if (Gaps[Corner] - MinimumGap > Roundoff) continue;
        if (Count == 4) return false;
        const double Span = FMath::Abs(Gaps[Corner]) + 0.01;
        Probe.Start = Vertices[Corner] + Span * N;
        Probe.End = Vertices[Corner] - Span * N;
        const auto Witness = World->SweepSingle(Probe);
        if (!Witness.bHit || Witness.bStartPenetrating || Witness.bSurfaceNormalMayVary ||
            Witness.GeometricErrorBoundCm != 0 || Witness.SurfaceFeatureKind != Speed::EContactFeatureKind::Face ||
            !Witness.Normal.Equals(N, 32 * DBL_EPSILON)) return false;
        Points[Count++] = Witness.Point;
    }
    if (Count == 0 || ParentComponent->GetPhysMass() <= 0) return false;
    const double InvMass = 1.0 / ParentComponent->GetPhysMass();
    const FMatrix InvI = RotateLocalInverseInertia(BoxQ);
    const double Closing = FVector::DotProduct(State.Velocity + FVector::CrossProduct(State.AngularVelocity, Points[0] - State.Location), N);
    // Restitution belongs to an actual CCD impact, never a post-transport
    // velocity-feasibility projection (which must not replay a bounce).
    const double ImpactRestitution = bNotifyImpact && Closing < -GetImpactThreshold() ? GetRestitution() : 0;
    const auto Response = Speed::SolvePlanarCoulombImpulse(MakeArrayView(Points, Count), N,
        State.Location, State.Velocity, State.AngularVelocity, InvMass, InvI, ImpactRestitution, GetDynamicFriction());
#if !UE_BUILD_SHIPPING
    if (CVarIAmSpeedAutoRecoverContactDebug.GetValueOnAnyThread() != 0)
        UE_LOG(BoxSubBodyLog, Display, TEXT("[ExactPlanarAcquisition] frame=%u impact=%d solved=%d count=%d gap=%.17g pose=%s q=%s v=%s w=%s a=%s alpha=%s dv=%s dw=%s toi=%.17g speed=%.17g spin=%.17g next_spin=%.17g"),
            ParentComponent->NumFrame(), bNotifyImpact ? 1 : 0, Response.bSolved ? 1 : 0, Count, MinimumGap,
            *State.Location.ToString(), *State.Rotation.ToString(), *State.Velocity.ToString(), *State.AngularVelocity.ToString(),
            *State.Acceleration.ToString(), *State.AngularAcceleration.ToString(),
            *Response.DeltaVelocity.ToString(), *Response.DeltaAngularVelocity.ToString(), double(CurrentHit.TOI),
            State.Velocity.Size(), State.AngularVelocity.Size(), (State.AngularVelocity + Response.DeltaAngularVelocity).Size());
#endif
    if (!Response.bSolved)
    {
#if !UE_BUILD_SHIPPING
        if (CVarIAmSpeedAutoRecoverContactDebug.GetValueOnAnyThread() != 0)
        {
            UE_LOG(BoxSubBodyLog, Display, TEXT("[ExactPlanarSolveRejected] frame=%u count=%d iterations=%u v=%s w=%s n=(%.17g,%.17g,%.17g) q=%s"),
                ParentComponent->NumFrame(), Count, Response.IterationCount, *State.Velocity.ToString(), *State.AngularVelocity.ToString(),
                double(N.X), double(N.Y), double(N.Z), *State.Rotation.ToString());
            UE_LOG(BoxSubBodyLog, Display, TEXT("[ExactPlanarSolveInput] com=(%.17g,%.17g,%.17g) v=(%.17g,%.17g,%.17g) w=(%.17g,%.17g,%.17g) invmass=%.17g mu=%.17g restitution=%.17g invI=(%.17g,%.17g,%.17g;%.17g,%.17g,%.17g;%.17g,%.17g,%.17g)"),
                State.Location.X, State.Location.Y, State.Location.Z, State.Velocity.X, State.Velocity.Y, State.Velocity.Z,
                State.AngularVelocity.X, State.AngularVelocity.Y, State.AngularVelocity.Z, InvMass, double(GetDynamicFriction()), ImpactRestitution,
                InvI.M[0][0], InvI.M[0][1], InvI.M[0][2], InvI.M[1][0], InvI.M[1][1], InvI.M[1][2], InvI.M[2][0], InvI.M[2][1], InvI.M[2][2]);
            for (int32 I = 0; I < Count; ++I)
                UE_LOG(BoxSubBodyLog, Display, TEXT("[ExactPlanarSolvePoint] index=%d p=(%.17g,%.17g,%.17g)"), I, Points[I].X, Points[I].Y, Points[I].Z);
        }
#endif
        return false;
    }
    const double VelocityRoundoff = 4096 * DBL_EPSILON * FMath::Max(1.0,
        State.Velocity.Size() + State.AngularVelocity.Size() * BoxExtent.Size());
    // Only an actual solved four-point impact can close micro-rocking. Never
    // capture from an old contact during ordinary support projection. The
    // certificate uses incoming energy and fresh geometry, not a cached face.
    const bool bCloseMicroRocking = bNotifyImpact && Count == 4 &&
        Speed::CanStabilizeBoxMicroRocking(*World, MakeSupportBoxQuery(State), State,
            InvI, InvMass, ImpactRestitution, GetDynamicFriction());
    State.Velocity += Response.DeltaVelocity;
    State.AngularVelocity += Response.DeltaAngularVelocity;
    if (Count == 4 && ImpactRestitution == 0)
    {
        bool bAllNormalsSolved = true, bAllSlipSolved = true;
        for (int32 I = 0; I < Count; ++I)
        {
            const FVector Vp = State.Velocity + FVector::CrossProduct(State.AngularVelocity, Points[I] - State.Location);
            const double VN = FVector::DotProduct(Vp, N);
            bAllNormalsSolved &= FMath::Abs(VN) <= VelocityRoundoff;
            bAllSlipSolved &= (Vp - VN * N).Size() <= VelocityRoundoff;
        }
        if (bAllNormalsSolved)
        {
            // Four solved normal rows imply these exact zero modes. Remove
            // only linear-algebra roundoff, not a finite tilt/speed or sleep
            // threshold. Separating/bouncing faces cannot enter this branch.
            State.Velocity -= FVector::DotProduct(State.Velocity, N) * N;
            State.AngularVelocity = FVector::DotProduct(State.AngularVelocity, N) * N;
            if (GetDynamicFriction() > 0 && bAllSlipSolved)
            {
                State.Velocity = FVector::ZeroVector;
                State.AngularVelocity = FVector::ZeroVector;
            }
        }
    }
    if (bCloseMicroRocking)
    {
        State.Velocity = FVector::ZeroVector;
        State.AngularVelocity = FVector::ZeroVector;
    }
    ParentComponent->SetPhysCOMLocation(State.Location);
    ParentComponent->SetPhysRotationPreserveCOM(State.Rotation);
    ParentComponent->SetPhysCOMVelocity(State.Velocity);
    ParentComponent->SetPhysAngularVelocity(State.AngularVelocity);
    ParentComponent->UpdateSubBodiesKinematics();
    GroundHit = CurrentHit;
    GroundPlaneN = N;
    GroundPlanePointWS = Points[0];
    GroundPlaneD = FVector::DotProduct(Points[0], N);
    GroundComp = CurrentHit.Component;
    bGroundPlaneValid = true;
    bHasGroundContact = true;
    CurrentGroundContactsWS.Reset(Count);
    for (int32 I = 0; I < Count; ++I) CurrentGroundContactsWS.Add(Points[I]);
    LastResolvedGroundHitFrame = ParentComponent->NumFrame();
    if (bNotifyImpact) ParentComponent->RcvImpactOnSubBody(*this, CurrentHit.ImpactPoint);
    return true;
}

bool UBoxSubBody::TryResolveExactFaceImpact()
{
    if (ParentComponent->HasActivePhysicalConstraintsOtherThan(this)) return false;
    const Speed::IStaticCollisionWorld* World = ParentComponent->GetStaticCollisionWorldForFrame();
    if (!World || !HasToApplyRestForce() || CurrentHit.SourceId == 0 ||
        CurrentHit.bSurfaceNormalMayVary || CurrentHit.GeometricErrorBoundCm != 0 ||
        CurrentHit.ContactFeatureOther != Speed::EContactFeatureKind::Face ||
        !CurrentHit.Component.IsValid() || CurrentHit.Component->GetCollisionObjectType() != ECC_WorldStatic)
        return false;
    const SKinematic State = ParentComponent->GetKinematicState();
    const FVector N = CurrentHit.ImpactNormal;
    const double VN = FVector::DotProduct(State.Velocity, N);
    if (VN >= 0 || !State.AngularVelocity.IsZero() || !State.AngularAcceleration.IsZero() ||
        !(State.Velocity - VN * N).IsZero()) return false;

    SKinematic Origin = State;
    Origin.Location -= State.Rotation.RotateVector(ParentComponent->GetPhysCenterOfMassLocal());
    const SKinematic Box = GetKinematicsFromOwnerKS(Origin);
    double Gap = DBL_MAX;
    for (int32 Corner = 0; Corner < 8; ++Corner)
    {
        const FVector P((Corner & 1) ? BoxExtent.X : -BoxExtent.X,
            (Corner & 2) ? BoxExtent.Y : -BoxExtent.Y, (Corner & 4) ? BoxExtent.Z : -BoxExtent.Z);
        Gap = FMath::Min(Gap, FVector::DotProduct(Box.Location + Box.Rotation.RotateVector(P) - CurrentHit.ImpactPoint, N));
    }
    const double Scale = FMath::Max(1.0, Box.Location.GetAbsMax() + BoxExtent.GetMax());
    const double EventRoundoff = 4.0 * FLT_EPSILON * FMath::Abs(CurrentHit.TOI * VN) + 64.0 * DBL_EPSILON * Scale;
    // This is only the residual of representing the CCD event time as float,
    // not a contact skin or a general-purpose depenetration/acquisition band.
    if (Gap < 0 || Gap > EventRoundoff) return false;
    Speed::Analytic::FWorldQuery Query;
    Query.Shape = Speed::Analytic::EQueryShape::Box;
    // Place the representable event pose on the exterior side of the plane.
    // The dot-radius and transformed-corner formulas differ by a few double
    // ULPs; this directed-rounding allowance remains below certificate roundoff.
    const FVector EventCorrection = (-Gap + 2.0 * DBL_EPSILON * Scale) * N;
    Query.Start = Query.End = Box.Location + EventCorrection;
    Query.Rotation = Box.Rotation;
    Query.HalfExtent = BoxExtent;
    Query.TraceChannel = static_cast<uint8>(GetCollisionChannel());
    Query.BlockingObjectTypes = Speed::GetBlockingResponseMask(GetResponseParams().CollisionResponse);
    Query.bApplyCollisionFilter = true;
    Query.bAuthorityOnly = true;
    Query.bIncludeCompactPatches = true;
    FVector COM = State.Location + EventCorrection;
    Speed::FBoxRestingSupport Support;
    bool bCertified = false;
    for (int32 RoundoffStep = 0; RoundoffStep < 8; ++RoundoffStep)
    {
        bCertified = Speed::TryBuildBoxRestingSupport(*World, Query, COM, -N, Support);
        if (bCertified || Support.Result != Speed::EBoxRestingSupportResult::Penetrating) break;
        const FVector Correction = N * (DBL_EPSILON * Scale);
        Query.Start += Correction; Query.End = Query.Start; COM += Correction;
    }
    if (!bCertified) return false;

    const double ImpactRestitution = -VN > GetImpactThreshold() ? GetRestitution() : 0.0;
    ParentComponent->SetPhysCOMLocation(COM);
    // The certified positive load fractions place the pressure resultant at
    // the COM projection. Its normal impulse has exactly zero net torque.
    ParentComponent->SetPhysCOMVelocity(State.Velocity - (1.0 + ImpactRestitution) * VN * N);
    ParentComponent->UpdateSubBodiesKinematics();
    GroundPlaneN = N;
    GroundPlanePointWS = Support.Points[0];
    GroundPlaneD = FVector::DotProduct(GroundPlanePointWS, N);
    GroundComp = CurrentHit.Component;
    GroundHit = CurrentHit;
    bGroundPlaneValid = true;
    bHasGroundContact = true;
    CurrentGroundContactsWS.Reset(4);
    for (const FVector& P : Support.Points) CurrentGroundContactsWS.Add(P);
    LastResolvedGroundHitFrame = ParentComponent->NumFrame();
    ParentComponent->RcvImpactOnSubBody(*this, CurrentHit.ImpactPoint);
    return true;
}

// ============================================================================
// =========================== Helper Methods =================================
// ============================================================================

SKinematic UBoxSubBody::GetKinematicsFromOwner(const unsigned int& NumFrame) const
{
    if (!ParentComponent)
        return SKinematic();
    return GetKinematicsFromOwnerKS(ParentComponent->GetOriginKinematicStateForFrame(NumFrame));
}
SKinematic UBoxSubBody::GetKinematicsFromOwnerKS(const SKinematic& CarKinematicState) const
{
    auto HitboxRelativeLocation = GetLocalOffset();

    FQuat CarRot(CarKinematicState.Rotation);
    auto HitboxWorldLocation = CarRot.RotateVector(HitboxRelativeLocation);
    auto HitboxWolrdCenter = CarKinematicState.Location + HitboxWorldLocation;

    FQuat HitboxRelativeQuat(GetLocalRotation());
    FVector HitboxExtent = BoxExtent;
    FQuat HitboxRot(CarRot * HitboxRelativeQuat);

    auto CarVelocity = CarKinematicState.Velocity;
    auto CarAcceleration = CarKinematicState.Acceleration;
    auto CarAngularVelocity = CarKinematicState.AngularVelocity;
    auto CarAngularAcceleration = CarKinematicState.AngularAcceleration;

    auto HitboxVelocity = CarVelocity + FVector::CrossProduct(CarAngularVelocity, HitboxWorldLocation);
    auto HitboxAcceleration = CarAcceleration + FVector::CrossProduct(CarAngularAcceleration, HitboxWorldLocation) +
        FVector::CrossProduct(CarAngularVelocity, FVector::CrossProduct(CarAngularVelocity, HitboxWorldLocation));

    SKinematic CrtKinematics;
    CrtKinematics.Location = HitboxWolrdCenter;
    CrtKinematics.Velocity = HitboxVelocity;
    CrtKinematics.Acceleration = HitboxAcceleration;
    CrtKinematics.Rotation = HitboxRot;
    CrtKinematics.AngularVelocity = CarAngularVelocity;
    CrtKinematics.AngularAcceleration = CarAngularAcceleration;
    return CrtKinematics;
}

SSBox UBoxSubBody::MakeBox() const
{
    const auto& CrtKinematics = Kinematics;
    return SSBox(
        CrtKinematics.Location,
        BoxExtent,
        CrtKinematics.Rotation,
        CrtKinematics.Velocity,
        CrtKinematics.Acceleration,
        CrtKinematics.AngularVelocity,
        CrtKinematics.AngularAcceleration
    );
}

SSBox UBoxSubBody::MakeBoxFromKS(const SKinematic& BoxKinematicState) const
{
    auto CrtKinematics = GetKinematicsFromOwnerKS(BoxKinematicState);
    return SSBox(
        CrtKinematics.Location,
        BoxExtent,
        CrtKinematics.Rotation,
        CrtKinematics.Velocity,
        CrtKinematics.Acceleration,
        CrtKinematics.AngularVelocity,
        CrtKinematics.AngularAcceleration
    );
}

void UBoxSubBody::GetBoxVertices(const FVector& Center, const FQuat& Rot, const FVector& Ext, TArray<FVector>& OutVerts)
{
    OutVerts.Reset();
    for (int sx : {-1, 1})
    {
        for (int sy : {-1, 1})
        {
            for (int sz : {-1, 1})
            {
                FVector local = FVector(sx * Ext.X, sy * Ext.Y, sz * Ext.Z);
                OutVerts.Add(Center + Rot.RotateVector(local));
            }
        }
    }
}

FVector UBoxSubBody::ComputeBoxSupportPointWS(const FVector& Center, const FQuat& Rot, const FVector& Ext, const FVector& N)
{
    const FVector N_LS = Rot.UnrotateVector(N);

    const FVector SupportLS(
        (N_LS.X >= 0.f) ? -Ext.X : Ext.X,
        (N_LS.Y >= 0.f) ? -Ext.Y : Ext.Y,
        (N_LS.Z >= 0.f) ? -Ext.Z : Ext.Z
    );

    return Center + Rot.RotateVector(SupportLS);
}

const TArray<FVector>& UBoxSubBody::GetGroundContacts() const
{
    return CurrentGroundContactsWS;
}

const TArray<FVector>& UBoxSubBody::GetPhysicsTickGroundContacts() const
{
    return CurrentGroundContactsWS.Num() > 0 ? CurrentGroundContactsWS : PreviousFrameGroundContactsWS;
}

bool UBoxSubBody::IsConcaveGroundContact() const
{
    if (CurrentGroundNormalsWS.Num() >= 2)
    {
        for (int i = 0; i < CurrentGroundNormalsWS.Num(); ++i)
        {
            for (int j = i + 1; j < CurrentGroundNormalsWS.Num(); ++j)
            {
                if (FVector::DotProduct(
                    CurrentGroundNormalsWS[i],
                    CurrentGroundNormalsWS[j]) < 0.95f)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

FVector UBoxSubBody::GetGroundPlaneNormal() const
{
    return GroundPlaneN;
}

float UBoxSubBody::GetGroundPlaneD() const
{
    return GroundPlaneD;
}

bool UBoxSubBody::GetLatchedEdgeWS(FVector& OutA, FVector& OutB) const
{
    if (!bEdgeSupportLatched || LatchedEdgeContactsLS.Num() != 2)
        return false;

    const SKinematic K = GetKinematicsFromOwner(ParentComponent->NumFrame());

    OutA = K.Location + K.Rotation.RotateVector(LatchedEdgeContactsLS[0]);
    OutB = K.Location + K.Rotation.RotateVector(LatchedEdgeContactsLS[1]);
    return true;
}

bool UBoxSubBody::IsPointSupportedByPersistentContact(const FVector& P, float Margin) const
{
    if (!HasPersistentGroundContact())
        return false;

    // Distance to ground plane
    const float dist = PlaneSignedDist(P, GroundPlaneN, GroundPlanePointWS);

    // If point is above plane -> still supported
    if (dist > Margin)
        return true;

    return false;
}

bool UBoxSubBody::HasPersistentGroundContact() const
{
    return bHasGroundContact;
}

bool UBoxSubBody::HasPhysicsTickGroundContact() const
{
    return bHasGroundContact || PreviousFrameGroundContactsWS.Num() > 0;
}

bool UBoxSubBody::HasRoofSurfaceTraversalSupport() const
{
    if (!ParentComponent ||
        !IsMainSubBody() ||
        !bRoofSurfaceTraversalLatched ||
        LastRoofSurfaceContactFrame == INDEX_NONE ||
        !RoofSurfaceComp.IsValid())
    {
        return false;
    }

    const FVector N = GroundPlaneN.GetSafeNormal();
    if (N.IsNearlyZero())
    {
        return false;
    }

    const SKinematic K = GetKinematicsFromOwner(ParentComponent->NumFrame());
    if (FVector::DotProduct(K.Rotation.GetUpVector(), N) > -0.25f)
    {
        return false;
    }

    const float SeparatingSpeedTolerance = FMath::Max(
        20.0f, CVarIAmSpeedFaceSupportSeparatingSpeedCmS.GetValueOnAnyThread());
    const float NearestPointNormalSpeed = FVector::DotProduct(K.Velocity, N);
    if (NearestPointNormalSpeed > SeparatingSpeedTolerance)
    {
        return false;
    }

    const int32 CurrentFrame = int32(ParentComponent->NumFrame());
    if (CurrentFrame >= LastRoofSurfaceContactFrame &&
        CurrentFrame - LastRoofSurfaceContactFrame <= RoofSurfaceTraversalGraceFrames)
    {
        return true;
    }

    TArray<FVector> Verts;
    GetBoxVertices(K.Location, K.Rotation, BoxExtent, Verts);

    float MinDistance = FLT_MAX;
    FVector NearestPoint = K.Location;
    for (const FVector& Vertex : Verts)
    {
        const float Distance = PlaneSignedDist(Vertex, N, GroundPlanePointWS);
        if (Distance < MinDistance)
        {
            MinDistance = Distance;
            NearestPoint = Vertex;
        }
    }

    const float ContactTolerance = FMath::Max(
        0.5f, CVarIAmSpeedFaceSupportContactEpsilonCm.GetValueOnAnyThread());
    const float PenetrationTolerance = FMath::Max(
        1.0f, CVarIAmSpeedFaceSupportPenetrationTolCm.GetValueOnAnyThread());
    return MinDistance >= -PenetrationTolerance &&
        MinDistance <= ContactTolerance &&
        NearestPointNormalSpeed <= SeparatingSpeedTolerance;
}

bool UBoxSubBody::HasPersistentEdgeSupport() const
{
    return bHasGroundContact && PrevGroundContactsLS.Num() == 2;
}


bool UBoxSubBody::IsInEdgeBalance() const
{
    if (!HasPersistentGroundContact())
        return false;

    const FVector GroundN = GetGroundPlaneNormal().GetSafeNormal();

    // Normal of the hitbox face most aligned with ground normal
    const FQuat Q = GetKinematicsFromOwner(ParentComponent->NumFrame()).Rotation;

    float bestDot = -1.f;
    int bestFace = -1;

    for (int i = 0; i < 6; ++i)
    {
        const FVector FaceN = GetFaceNormalWS(i, Q);
        const float d = FVector::DotProduct(FaceN, GroundN);
        if (d > bestDot)
        {
            bestDot = d;
            bestFace = i;
        }
    }

    // If no face is really aligned -> edge or corner
    constexpr float FaceAlignThreshold = 0.92f;
    return bestDot < FaceAlignThreshold;;
}

int UBoxSubBody::GetBestSupportingFace() const
{
    const FVector GroundN = GetGroundPlaneNormal().GetSafeNormal();
    const FQuat Q = GetKinematicsFromOwner(ParentComponent->NumFrame()).Rotation;

    float bestDot = -FLT_MAX;
    int bestFace = -1;

    for (int i = 0; i < 6; ++i)
    {
        const FVector FaceN = GetFaceNormalWS(i, Q);
        const float d = FVector::DotProduct(FaceN, GroundN);
        if (d > bestDot)
        {
            bestDot = d;
            bestFace = i;
        }
    }
    return bestFace;
}

int UBoxSubBody::GetLowestFaceIndex() const
{
    const FVector GroundN = GetGroundPlaneNormal().GetSafeNormal();
    const float GroundD = GetGroundPlaneD();

    const FQuat Q = GetKinematicsFromOwner(ParentComponent->NumFrame()).Rotation;
    const FVector C = GetKinematicsFromOwner(ParentComponent->NumFrame()).Location;

    float bestMinDist = FLT_MAX;
    int bestFace = -1;

    for (int face = 0; face < 6; ++face)
    {
        TArray<FVector> FaceVerts = GetFaceVerticesWS(face);

        float faceMin = FLT_MAX;
        for (const FVector& V : FaceVerts)
        {
            const float dist = FVector::DotProduct(V, GroundN) - GroundD;
            faceMin = FMath::Min(faceMin, dist);
        }

        if (faceMin < bestMinDist)
        {
            bestMinDist = faceMin;
            bestFace = face;
        }
    }

    return bestFace;
}

FVector UBoxSubBody::GetFaceNormalWS(const int& FaceIndex, const FQuat& HitboxRot) const
{
    const FVector Nlocal = FaceLocalNormal(FaceIndex);
    if (Nlocal.IsNearlyZero())
        return FVector::ZeroVector;

    return HitboxRot.RotateVector(Nlocal).GetSafeNormal();
}

TArray<FVector> UBoxSubBody::GetFaceVerticesWS(const int& FaceIndex) const
{
    TArray<FVector> OutVertsWS;
    if (!ParentComponent)
        return OutVertsWS;

    const SKinematic K = GetKinematicsFromOwner(ParentComponent->NumFrame());
    const FVector CenterWS = K.Location;
    const FQuat Q = K.Rotation;

    FVector Vlocal[4];
    FaceLocalVertices(FaceIndex, BoxExtent, Vlocal);

    OutVertsWS.Reserve(4);
    for (int i = 0; i < 4; ++i)
    {
        const FVector Vws = CenterWS + Q.RotateVector(Vlocal[i]);
        OutVertsWS.Add(Vws);
    }
    return OutVertsWS;
}


bool UBoxSubBody::ComputeEdgeLockAxis(const FVector& N, const TArray<FVector>& SupportPts,
    FVector& OutLockAxis) const
{
    if (SupportPts.Num() < 2)
        return false;

    // stable edge direction: pair of points which are the farest
    float bestD2 = -1.f;
    FVector EdgeDir = FVector::ZeroVector;

    for (int i = 0; i < SupportPts.Num(); ++i)
        for (int j = i + 1; j < SupportPts.Num(); ++j)
        {
            const FVector d = SupportPts[j] - SupportPts[i];
            const float d2 = d.SizeSquared();
            if (d2 > bestD2)
            {
                bestD2 = d2;
                EdgeDir = d;
            }
        }

    EdgeDir = EdgeDir.GetSafeNormal();
    if (EdgeDir.IsNearlyZero())
        return false;

    OutLockAxis = FVector::CrossProduct(N, EdgeDir).GetSafeNormal();
    return !OutLockAxis.IsNearlyZero();
}

void UBoxSubBody::ApplyImpulse(
    const FVector& LinearImpulse,
    const FVector& WorldPoint)
{
    ParentComponent->ApplyImpulse(LinearImpulse, WorldPoint, this);
}

FVector UBoxSubBody::GetCOM() const
{
    if (!ParentComponent)
    {
        return FVector::ZeroVector;
    }
    // The rigid body is integrated around the component's physical COM.
    // Using the box's local offset here was only valid while the COM happened
    // to coincide with the component origin; it produces incorrect lever arms
    // as soon as a vehicle defines an offset CenterOfMass.
    return ParentComponent->GetPhysCOM();
}

FVector UBoxSubBody::GetBoxExtent() const
{
	return BoxExtent;
}

float UBoxSubBody::GetSphereSeparation(
	const FVector& SphereCenter, const float SphereRadius) const
{
	const Speed::SBox BoxShape = MakeBox();
	return BoxShape.SphereOBBSeparation(
		BoxShape.Rot,
		BoxShape.AbsoluteCenter(),
		SphereCenter,
		SphereRadius);
}

void UBoxSubBody::SetBoxExtent(const FVector& InBoxExtent)
{
	BoxExtent = InBoxExtent.ComponentMax(FVector::ZeroVector);
}

FCollisionShape UBoxSubBody::GetCollisionShape(float Inflation) const
{
    return FCollisionShape::MakeBox(BoxExtent + FVector(Inflation));
}

void UBoxSubBody::BuildContactPoints(const FVector& Nworld, const FVector& CenterWS, const FQuat& BoxRotWS, const FVector& Ext,
    TArray<FVector>& OutPointsWS) const
{
    OutPointsWS.Reset();

    const FVector nLocal = BoxRotWS.UnrotateVector(Nworld).GetSafeNormal();
    const FVector absN = nLocal.GetAbs();

    int dominantAxes = 0;
    if (absN.X > 0.01f) dominantAxes++;
    if (absN.Y > 0.01f) dominantAxes++;
    if (absN.Z > 0.01f) dominantAxes++;

    enum class EFeature { Face, Edge, Corner };
    EFeature feature = (dominantAxes == 1) ? EFeature::Face : (dominantAxes == 2) ? EFeature::Edge : EFeature::Corner;

    TArray<FVector> localPts;

    if (feature == EFeature::Face)
    {
        // pick dominant axis
        if (absN.X >= absN.Y && absN.X >= absN.Z)
        {
            float sx = -FMath::Sign(nLocal.X) * Ext.X;
            localPts = { FVector(sx, +Ext.Y, +Ext.Z), FVector(sx, -Ext.Y, +Ext.Z),
                         FVector(sx, +Ext.Y, -Ext.Z), FVector(sx, -Ext.Y, -Ext.Z) };
        }
        else if (absN.Y >= absN.X && absN.Y >= absN.Z)
        {
            float sy = -FMath::Sign(nLocal.Y) * Ext.Y;
            localPts = { FVector(+Ext.X, sy, +Ext.Z), FVector(-Ext.X, sy, +Ext.Z),
                         FVector(+Ext.X, sy, -Ext.Z), FVector(-Ext.X, sy, -Ext.Z) };
        }
        else
        {
            float sz = -FMath::Sign(nLocal.Z) * Ext.Z;
            localPts = { FVector(+Ext.X, +Ext.Y, sz), FVector(-Ext.X, +Ext.Y, sz),
                         FVector(+Ext.X, -Ext.Y, sz), FVector(-Ext.X, -Ext.Y, sz) };
        }
    }
    else if (feature == EFeature::Edge)
    {
        float sx = (absN.X > 0.01f) ? -FMath::Sign(nLocal.X) * Ext.X : 0.f;
        float sy = (absN.Y > 0.01f) ? -FMath::Sign(nLocal.Y) * Ext.Y : 0.f;
        float sz = (absN.Z > 0.01f) ? -FMath::Sign(nLocal.Z) * Ext.Z : 0.f;

        bool useX = absN.X > 0.01f;
        bool useY = absN.Y > 0.01f;

        if (!useX)      localPts = { FVector(+Ext.X, sy, sz), FVector(-Ext.X, sy, sz) };
        else if (!useY) localPts = { FVector(sx, +Ext.Y, sz), FVector(sx, -Ext.Y, sz) };
        else            localPts = { FVector(sx, sy, +Ext.Z), FVector(sx, sy, -Ext.Z) };
    }
    else // Corner
    {
        float sx = -FMath::Sign(nLocal.X) * Ext.X;
        float sy = -FMath::Sign(nLocal.Y) * Ext.Y;
        float sz = -FMath::Sign(nLocal.Z) * Ext.Z;
        localPts = { FVector(sx, sy, sz) };
    }

    for (const FVector& pL : localPts)
    {
        OutPointsWS.Add(CenterWS + BoxRotWS.RotateVector(pL));
    }
}

FVector UBoxSubBody::BuildCompositeGroundNormal() const
{
    if (CurrentGroundNormalsWS.Num() == 0)
        return FVector::UpVector;

    FVector Nsum = FVector::ZeroVector;

    for (const FVector& N : CurrentGroundNormalsWS)
    {
        Nsum += N;
    }

    if (!Nsum.Normalize())
    {
        return CurrentGroundNormalsWS[0];
    }

    return Nsum;
}

bool UBoxSubBody::ShouldPromoteWallHitToSupport(const SHitResult& Hit, const FVector& HitN, float HitUpDot, float Dt, SHitResult& OutSupportHit) const
{
    if (!ParentComponent)
    {
        return false;
    }

    constexpr int32 WallToSupportPromotionCooldownFrames = 8;
    const int32 CurrentFrame = ParentComponent->NumFrame();
    const int32 FramesSinceWallOrGutter =
        LastWallOrGutterFrame == INDEX_NONE
        ? INDEX_NONE
        : CurrentFrame - LastWallOrGutterFrame;

    if (FramesSinceWallOrGutter != INDEX_NONE &&
        FramesSinceWallOrGutter >= 0 &&
        FramesSinceWallOrGutter <= WallToSupportPromotionCooldownFrames)
    {
        /*UE_LOG(BoxSubBodyLog, Log,
            TEXT("[CCDBoxPromoteReject] Frame=%d Reason=RecentWallOrGutter FramesSinceWall=%d RawUpDot=%.4f LastWallUpDot=%.4f HitN=%s LastWallN=%s HitP=%s TOI=%.6f"),
            CurrentFrame,
            FramesSinceWallOrGutter,
            HitUpDot,
            LastWallOrGutterUpDot,
            *HitN.ToString(),
            *LastWallOrGutterN.ToString(),
            *Hit.ImpactPoint.ToString(),
            Hit.TOI);*/

        return false;
    }

    // Already support : not a promotion.
    if (HitUpDot >= 0.85f)
    {
        return false;
    }

    // True vertical wall : do not promote.
    // Here we only target gutter normals with a notable Z component.
    if (HitUpDot < 0.15f)
    {
        return false;
    }

	// If the hit is too far in the future, don't promote: we want to be reactive to new ground contacts.
    if (Hit.TOI > 0.0015f)
    {
        return false;
    }

    if (!bGroundPlaneValid || !GroundComp.IsValid())
    {
        return false;
    }

    const FVector PlaneN = GroundPlaneN.GetSafeNormal();
    const float PlaneUpDot = FVector::DotProduct(PlaneN, FVector::UpVector);

    if (PlaneUpDot < 0.97f)
    {
        return false;
    }

    // Avoid promoting a hit against a component other than the remembered ground/stadium.
    if (Hit.Component.IsValid() && GroundComp.IsValid() && Hit.Component.Get() != GroundComp.Get())
    {
        return false;
    }

    const FVector Vcm = ParentComponent->GetPhysCOMVelocity();
    const float Vz = Vcm.Z;

    // Velocity normal to the support plane.
    const FVector Pbox = ComputeBoxSupportPointWS(
        Kinematics.Location,
        Kinematics.Rotation,
        BoxExtent,
        PlaneN
    );

    const float vPlane = FVector::DotProduct(GetVelocityAtPoint(Pbox), PlaneN);

    // Only promote if the car is falling towards the plane.
    if (Vz > -50.f && vPlane > -50.f)
    {
        return false;
    }

    const float DistToPlane = PlaneSignedDist(Pbox, PlaneN, GroundPlanePointWS);

    // Large band intentionally: the hitbox center/point can be high in the gutters.
    // To tune with your logs. I would start wide, then narrow it.
    constexpr float MaxAbovePlaneCm = 90.f;
    constexpr float MaxBelowPlaneCm = 15.f;

    if (DistToPlane > MaxAbovePlaneCm || DistToPlane < -MaxBelowPlaneCm)
    {
        return false;
    }

    OutSupportHit = Hit;
    OutSupportHit.ImpactNormal = PlaneN;

    // Use a consistent point on the plane rather than the point on the inclined wall.
    OutSupportHit.ImpactPoint = Pbox - DistToPlane * PlaneN;

    // Keep the TOI of the original hit: the event remains at the same moment.
    OutSupportHit.TOI = Hit.TOI;

    return true;
}


bool UBoxSubBody::SolveEdgeSupportConstraint(
    const FVector& SupportN,
    const TArray<FVector>& SupportPts,
    const float Dt,
    const bool bDoFriction /*= false*/,
    const float Mu /*= 0.0f*/,
    const bool bAllowTraversalTorque /*= false*/)
{
    if (!ParentComponent)
        return false;

    if (SupportPts.Num() != 2)
        return false;

    // ---------------------------------------------------------------------
    // Geometry
    // ---------------------------------------------------------------------
    const FVector N = SupportN.GetSafeNormal();
    if (N.IsNearlyZero())
        return false;

    const FVector P1 = SupportPts[0];
    const FVector P2 = SupportPts[1];

    FVector EdgeDir = P2 - P1;

    // Project edge onto the tangent plane (robustness)
    EdgeDir -= FVector::DotProduct(EdgeDir, N) * N;

    const float EdgeLen = EdgeDir.Size();
    if (EdgeLen < KINDA_SMALL_NUMBER)
        return false;

    EdgeDir /= EdgeLen;

    // Reject degenerate case (edge almost parallel to normal)
    if (FMath::Abs(FVector::DotProduct(EdgeDir, N)) > 0.95f)
        return false;

    const FVector COM = ParentComponent->GetPhysCOM();
    const FVector COMVelocity = ParentComponent->GetPhysCOMVelocity();
    const float vN_com = FVector::DotProduct(COMVelocity, N);
    const bool bIsInAutoRecover = ParentComponent->IsInAutoRecover();

    if (!bAllowTraversalTorque)
    {
        // Normal edge support must remain close to its original plane.
        const float distCOM = PlaneSignedDist(COM, N, GroundPlanePointWS);
        constexpr float MaxSupportDist = 1.0f; // cm
        if (distCOM > MaxSupportDist)
        {
            return false;
        }

        // A normal edge cannot pull a separating body back toward the plane.
        constexpr float COM_SEP_EPS = -0.5f; // cm/s
        if (vN_com > COM_SEP_EPS)
        {
            return false;
        }
    }

    // ---------------------------------------------------------------------
    // 1) Linear support constraint (NO TORQUE)
    //    Cancel average velocity INTO the plane at COM
    // ---------------------------------------------------------------------
    const FVector v1 = GetVelocityAtPoint(P1);
    const FVector v2 = GetVelocityAtPoint(P2);

    const float vN = bIsInAutoRecover ? vN_com :
        0.5f * (FVector::DotProduct(v1, N) + FVector::DotProduct(v2, N));

    constexpr float VN_EPS = -0.5f; // cm/s
    if (vN < VN_EPS)
    {
        if (!bIsInAutoRecover)
        {
            if (bAllowTraversalTorque)
            {
                // The roof follows a gutter by rotating around its supporting edge.
                const FVector EdgeMidpoint = 0.5f * (P1 + P2);
                const FVector R = EdgeMidpoint - COM;
                const float Denominator = EffectiveMassAlongDir(
                    1.f / GetMass(), ComputeWorldInvInertiaTensor(), R, N);
                if (Denominator > KINDA_SMALL_NUMBER)
                {
                    ApplyImpulse((-vN / Denominator) * N, EdgeMidpoint);
                }
            }
            else
            {
                const float lambdaN = -vN * GetMass();
                ApplyImpulse(lambdaN * N, COM);
            }
#if !UE_BUILD_SHIPPING
            // UE_LOG(BoxSubBodyLog, Log, TEXT("[EdgeSupport] Linear support @COM vN=%.3f J=%s"), vN, *J.ToString());
#endif
        }
        else
        {
            // Apply impulse at COM to avoid torque which could cancel auto-recover
            ParentComponent->AddPhysVelocity(-vN * N);
#if !UE_BUILD_SHIPPING
            // UE_LOG(BoxSubBodyLog, Log, TEXT("[EdgeSupport] Linear support @COM vN=%.3f"), vN);
#endif
        }
    if (bAllowTraversalTorque && !bIsInAutoRecover)
    {
        const float MaxSeparationSpeed =
            FMath::Max(0.f, CVarIAmSpeedFaceSupportSeparatingSpeedCmS.GetValueOnAnyThread());
        const float SeparationSpeed = FVector::DotProduct(ParentComponent->GetPhysCOMVelocity(), N);
        if (SeparationSpeed > MaxSeparationSpeed)
        {
            ParentComponent->AddPhysVelocity((MaxSeparationSpeed - SeparationSpeed) * N);
        }
    }
    }

    // ---------------------------------------------------------------------
    // Detect if edge is still REALLY supporting (normal force meaningful)
    // ---------------------------------------------------------------------
    constexpr float SupportVNThreshold = -5.0f; // cm/s
    const bool bEdgeStillSupporting =
        vN < SupportVNThreshold &&
        (bAllowTraversalTorque || FVector::DotProduct(N, FVector::UpVector) > 0.7f);

    if (ParentComponent->IsSubBodyInAutoRecoverMode())
    {
        // Do NOT project angular velocity
        return false; // skip edge support
    }

    // ---------------------------------------------------------------------
    // 2) Angular constraint (HARD projection)
    //    Allow rotation ONLY around the edge axis
    // ---------------------------------------------------------------------
    if (bEdgeStillSupporting)
    {
        if (!bIsInAutoRecover && IsMainSubBody())
        {
            const FVector Omega = ParentComponent->GetPhysAngularVelocity();
            const float wEdge = FVector::DotProduct(Omega, EdgeDir);

            // Project angular velocity onto edge axis
            const FVector OmegaProjected = wEdge * EdgeDir;

            ParentComponent->AddPhysAngularVelocity(OmegaProjected - Omega);

#if !UE_BUILD_SHIPPING
            /*UE_LOG(
                BoxSubBodyLog,
                Log,
                TEXT("[EdgeSupport] Angular projection Omega=(%.3f %.3f %.3f) Edge=(%.3f %.3f %.3f)"),
                OmegaProjected.X, OmegaProjected.Y, OmegaProjected.Z,
                EdgeDir.X, EdgeDir.Y, EdgeDir.Z
            );*/
#endif
        }
        else
        {
            // Mode auto-recover :
            // enable rotation only on Fall axis
            const FVector Omega = ParentComponent->GetPhysAngularVelocity();

            const FVector FallAxis =
                FVector::CrossProduct(EdgeDir, N).GetSafeNormal();

            const float wEdge = FVector::DotProduct(Omega, EdgeDir);
            const float wFall = FVector::DotProduct(Omega, FallAxis);

            // Conserver edge + fall, only suppress normal component
            ParentComponent->SetPhysAngularVelocity(
                wEdge * EdgeDir +
                wFall * FallAxis
            );
        }
    }

    return true;
}

FMatrix UBoxSubBody::InitInvInertiaTensor() const
{
    if (!ParentComponent)
    {
        return FMatrix::Identity;
    }
    FVector HitboxRelLoc = GetLocalOffset(); // cm
    FQuat HitboxRelRot = GetLocalRotation();

    // 1. Compute the box-center-to-COM offset in hitbox local space.
    const FVector COMLocal = ParentComponent->GetPhysRotation().UnrotateVector(
        ParentComponent->GetPhysCOM() - ParentComponent->GetPhysLocation()
    );
    const FVector d_localChassis = HitboxRelLoc - COMLocal;
    FVector d_in_box = HitboxRelRot.UnrotateVector(d_localChassis);

	// 2. Compute inverse inertia tensor at COM, expressed in box local frame
    const float InertiaScale = FMath::Max(UE_KINDA_SMALL_NUMBER, CVarIAmSpeedBoxInertiaScale.GetValueOnAnyThread());
    const FVector Idiag =
        SSBox::ComputeInertiaTensor(BoxExtent, GetMass(), InertiaScale);

    const FMatrix I_atCOM =
        SSBox::ConvertBoxInertiaToCOM(Idiag, d_in_box, GetMass());

    return I_atCOM.InverseFast();
}

FMatrix UBoxSubBody::ComputeWorldInvInertiaTensor() const
{
    if (!ParentComponent)
        return FMatrix::Identity;
    const FQuat ChassisQ = ParentComponent->GetPhysRotation();
    const FQuat HitboxRelQ = GetLocalRotation();

    // Hitbox local -> world
    const FQuat Qworld = ChassisQ * HitboxRelQ;

    return RotateLocalInverseInertia(Qworld);
}

FMatrix UBoxSubBody::ComputeChassisLocalInvInertiaTensor() const
{
    return RotateLocalInverseInertia(GetLocalRotation());
}

FMatrix UBoxSubBody::RotateLocalInverseInertia(const FQuat& BoxToFrame) const
{
    // FMatrix transforms row vectors: first unrotate a frame-space torque,
    // apply the local tensor, then rotate its angular response back. Avoid an
    // Euler conversion, which snaps finite near-pole inclinations to +/-90deg.
    const FMatrix R = FQuatRotationMatrix(BoxToFrame);
    return R.GetTransposed() * InvInertiaLocal * R;
}


#if WITH_EDITOR
void UBoxSubBody::SetShowFlags(const FEngineShowFlags& InShowFlags)
{
    ShowFlags = InShowFlags;
    MarkRenderStateDirty();
}
#endif // WITH_EDITOR

FPrimitiveSceneProxy* UBoxSubBody::CreateSceneProxy()
{
    /** Represents a UBoxSubBody to the scene manager. */
    class FBoxSceneProxy final : public FPrimitiveSceneProxy
    {
    public:
        SIZE_T GetTypeHash() const override
        {
            static size_t UniquePointer;
            return reinterpret_cast<size_t>(&UniquePointer);
        }

        FBoxSceneProxy(const UBoxSubBody* InComponent)
            : FPrimitiveSceneProxy(InComponent)
            , bDrawOnlyIfSelected(InComponent->bDrawOnlyIfSelected)
            , BoxExtents(InComponent->BoxExtent)
            , BoxColor(InComponent->ShapeColor)
            , LineThickness(InComponent->LineThickness)
        {
            bWillEverBeLit = false;

#if WITH_EDITOR
            struct FIterSink
            {
                FIterSink(const FEngineShowFlags InSelectedShowFlags)
                    : SelectedShowFlags(InSelectedShowFlags)
                {
                    SelectedShowFlagIndices.SetNum(FEngineShowFlags::SF_FirstCustom, false);
                }

                bool HandleShowFlag(uint32 InIndex, const FString& InName)
                {
                    if (SelectedShowFlags.GetSingleFlag(InIndex) == true)
                    {
                        SelectedShowFlagIndices.PadToNum(InIndex + 1, false);
                        SelectedShowFlagIndices[InIndex] = true;
                    }

                    return true;
                }

                bool OnEngineShowFlag(uint32 InIndex, const FString& InName)
                {
                    return HandleShowFlag(InIndex, InName);
                }

                bool OnCustomShowFlag(uint32 InIndex, const FString& InName)
                {
                    return HandleShowFlag(InIndex, InName);
                }

                const FEngineShowFlags SelectedShowFlags;

                TBitArray<> SelectedShowFlagIndices;
            };

            FIterSink Sink(InComponent->ShowFlags);
            FEngineShowFlags::IterateAllFlags(Sink);
            SelectedShowFlagIndices = MoveTemp(Sink.SelectedShowFlagIndices);
#endif // WITH_EDITOR
        }

        virtual void GetDynamicMeshElements(const TArray<const FSceneView*>& Views, const FSceneViewFamily& ViewFamily, uint32 VisibilityMap, FMeshElementCollector& Collector) const override
        {
            QUICK_SCOPE_CYCLE_COUNTER(STAT_BoxSceneProxy_GetDynamicMeshElements);

            const FMatrix& LocalToWorld = GetLocalToWorld();

            for (int32 ViewIndex = 0; ViewIndex < Views.Num(); ViewIndex++)
            {
                if (VisibilityMap & (1 << ViewIndex))
                {
                    const FSceneView* View = Views[ViewIndex];

                    const FLinearColor DrawColor = GetViewSelectionColor(BoxColor, *View, IsSelected(), IsHovered(), false, IsIndividuallySelected());

                    FPrimitiveDrawInterface* PDI = Collector.GetPDI(ViewIndex);
                    DrawOrientedWireBox(PDI, LocalToWorld.GetOrigin(), LocalToWorld.GetScaledAxis(EAxis::X), LocalToWorld.GetScaledAxis(EAxis::Y), LocalToWorld.GetScaledAxis(EAxis::Z), BoxExtents, DrawColor, SDPG_World, LineThickness);
                }
            }
        }

        virtual FPrimitiveViewRelevance GetViewRelevance(const FSceneView* View) const override
        {
            const bool bProxyVisible = !bDrawOnlyIfSelected || IsSelected();

            // Should we draw this because collision drawing is enabled, and we have collision
            const bool bShowForCollision = View->Family->EngineShowFlags.Collision && IsCollisionEnabled();

            FPrimitiveViewRelevance Result;
            Result.bDrawRelevance = (IsShown(View) && bProxyVisible) || bShowForCollision;
#if WITH_EDITOR
            bool bAreAllSelectedFlagsEnabled = true;
            for (TConstSetBitIterator<> It(SelectedShowFlagIndices); It; ++It)
            {
                bAreAllSelectedFlagsEnabled &= View->Family->EngineShowFlags.GetSingleFlag(It.GetIndex());
            }

            Result.bDrawRelevance &= bAreAllSelectedFlagsEnabled;
#endif // WITH_EDITOR
            Result.bDynamicRelevance = true;
            Result.bShadowRelevance = IsShadowCast(View);
            Result.bEditorPrimitiveRelevance = UseEditorCompositing(View);
            return Result;
        }
        virtual uint32 GetMemoryFootprint(void) const override { return(sizeof(*this) + GetAllocatedSize()); }
        uint32 GetAllocatedSize(void) const { return(FPrimitiveSceneProxy::GetAllocatedSize()); }

    private:
        const uint32	bDrawOnlyIfSelected : 1;
        const FVector	BoxExtents;
        const FColor	BoxColor;
        const float		LineThickness;
#if WITH_EDITOR
        TBitArray<>		SelectedShowFlagIndices;
#endif // WITH_EDITOR
    };

    return new FBoxSceneProxy(this);
}

FBoxSphereBounds UBoxSubBody::CalcBounds(const FTransform& LocalToWorld) const
{
    return FBoxSphereBounds(FBox(-BoxExtent, BoxExtent)).TransformBy(LocalToWorld);
}

template <EShapeBodySetupHelper UpdateBodySetupAction, typename BodySetupType>
bool InvalidateOrUpdateBoxBodySetup(BodySetupType& ShapeBodySetup, bool bUseArchetypeBodySetup, FVector BoxExtent)
{
    check((bUseArchetypeBodySetup && UpdateBodySetupAction == EShapeBodySetupHelper::InvalidateSharingIfStale) || (!bUseArchetypeBodySetup && UpdateBodySetupAction == EShapeBodySetupHelper::UpdateBodySetup));
    check(ShapeBodySetup->AggGeom.BoxElems.Num() == 1);
    FKBoxElem* se = ShapeBodySetup->AggGeom.BoxElems.GetData();

    // @todo do we allow this now?
    // check for malformed values
    if (BoxExtent.X < UE_KINDA_SMALL_NUMBER)
    {
        BoxExtent.X = 1.0f;
    }

    if (BoxExtent.Y < UE_KINDA_SMALL_NUMBER)
    {
        BoxExtent.Y = 1.0f;
    }

    if (BoxExtent.Z < UE_KINDA_SMALL_NUMBER)
    {
        BoxExtent.Z = 1.0f;
    }

    float XExtent = BoxExtent.X * 2.f;
    float YExtent = BoxExtent.Y * 2.f;
    float ZExtent = BoxExtent.Z * 2.f;

    if (UpdateBodySetupAction == EShapeBodySetupHelper::UpdateBodySetup)
    {
        // now set the PhysX data values
        se->SetTransform(FTransform::Identity);
        se->X = XExtent;
        se->Y = YExtent;
        se->Z = ZExtent;
    }
    else if (se->X != XExtent || se->Y != YExtent || se->Z != ZExtent)
    {
        ShapeBodySetup = nullptr;
        bUseArchetypeBodySetup = false;
    }

    return bUseArchetypeBodySetup;
}

void UBoxSubBody::UpdateBodySetup()
{
    if (PrepareSharedBodySetup<UBoxSubBody>())
    {
        bUseArchetypeBodySetup = InvalidateOrUpdateBoxBodySetup<EShapeBodySetupHelper::InvalidateSharingIfStale>(ShapeBodySetup, bUseArchetypeBodySetup, BoxExtent);
    }

    CreateShapeBodySetupIfNeeded<FKBoxElem>();

    if (!bUseArchetypeBodySetup)
    {
        InvalidateOrUpdateBoxBodySetup<EShapeBodySetupHelper::UpdateBodySetup>(ShapeBodySetup, bUseArchetypeBodySetup, BoxExtent);
    }
}
