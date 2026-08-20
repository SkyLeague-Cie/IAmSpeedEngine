#include "ISensorSweeper.h"
#include "IAmSpeed/World/Analytic/AnalyticWorldQuery.h"
#include "IAmSpeed/World/Analytic/StaticWorldQueryAudit.h"

bool ISensorSweeper::InternalSweep(UWorld* World, const FVector& Start, const FVector& End, SHitResult& OutHit, const float& delta)
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
	Speed::Analytic::FStaticWorldQueryAudit::RecordSingle(
		Speed::Analytic::EStaticQuerySite::SensorSweep,
		Start, End, KS.Rotation, GetCollisionShape(),
		static_cast<uint8>(GetCollisionChannel()), GetResponseParams(), bHit, Hit);

    OutHit = SHitResult();
    OutHit.bHit = bHit;
    OutHit.bBlockingHit = false;
    if (bHit)
    {
		OutHit = bUsedAnalyticAuthority
			? SHitResult::FromAnalyticHit(AnalyticHit, delta, Hit.Component.Get())
			: SHitResult::FromUnrealHit(Hit, delta);
		OutHit.bBlockingHit = false;
    }
    return bHit;
}
