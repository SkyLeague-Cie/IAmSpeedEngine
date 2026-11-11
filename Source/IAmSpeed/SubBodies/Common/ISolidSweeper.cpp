#include "ISolidSweeper.h"
#include "Engine/World.h"
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
	const bool bUsedAnalyticAuthority =
		Speed::Analytic::FStaticWorldQueryAudit::TryCompactAuthoritySingle(
		World,
		Start, End, KS.Rotation, QueryShape,
		static_cast<uint8>(GetCollisionChannel()), GetResponseParams(), Hit, bHit,
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

    OutHit = SHitResult();
    OutHit.bHit = bHit;
    OutHit.bBlockingHit = bHit;
    if (bHit)
    {
		OutHit = bUsedAnalyticAuthority
			? SHitResult::FromAnalyticHit(AnalyticHit, delta, Hit.Component.Get())
			: SHitResult::FromUnrealHit(Hit, delta);
    }
    return bHit;
}
