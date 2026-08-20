#include "ISolidSweeper.h"
#include "IAmSpeed/IAmSpeed.h"
#include "IAmSpeed/World/Analytic/AnalyticWorldQuery.h"
#include "IAmSpeed/World/Analytic/StaticWorldQueryAudit.h"

bool ISolidSweeper::InternalSweep(UWorld* World, const FVector& Start, const FVector& End, SHitResult& OutHit, const float& delta)
{
    if (!World) return false;

    const Speed::FKinematicState& KS = GetKinematicState();
    FCollisionQueryParams Params = BuildTraceParams();
	FHitResult Hit;
	bool bHit = false;
	Speed::Analytic::FWorldHit AnalyticHit;
	const bool bUsedAnalyticAuthority =
		Speed::Analytic::FStaticWorldQueryAudit::TryCompactAuthoritySingle(
		World,
		Start, End, KS.Rotation, GetCollisionShape(),
		static_cast<uint8>(GetCollisionChannel()), GetResponseParams(), Hit, bHit,
		&AnalyticHit);
	if (!bUsedAnalyticAuthority)
	{
		Speed::Analytic::FStaticWorldQueryAudit::RecordLegacySweep();
		bHit = World->SweepSingleByChannel(
			Hit, Start, End, KS.Rotation, GetCollisionChannel(),
			GetCollisionShape(), Params, GetResponseParams());
	}
	else if (bHit && GetCollisionShape().IsBox() &&
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
			Speed::Analytic::FStaticWorldQueryAudit::RecordLegacySweep();
			const bool bChaosShadowHit = World->SweepSingleByChannel(
				ChaosShadowHit, Start, End, KS.Rotation, GetCollisionChannel(),
				GetCollisionShape(), Params, GetResponseParams());
			UE_LOG(SpeedPhysicsLog, Display,
				TEXT("[AnalyticAuthorityChaosBoxShadow] Start=%s End=%s Rotation=%s HalfExtent=%s AnalyticTime=%.17g AnalyticPoint=%s AnalyticNormal=%s AnalyticStartPenetrating=%d AnalyticPenetrationDepth=%.17g AnalyticSurface=%016llX AnalyticFeature=%016llX AnalyticPrimitive=%016llX ChaosHit=%d ChaosTime=%.9g ChaosPoint=%s ChaosNormal=%s ChaosStartPenetrating=%d ChaosPenetrationDepth=%.9g"),
				*Start.ToString(), *End.ToString(), *KS.Rotation.ToString(),
				*GetCollisionShape().GetBox().ToString(),
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
		Start, End, KS.Rotation, GetCollisionShape(),
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
