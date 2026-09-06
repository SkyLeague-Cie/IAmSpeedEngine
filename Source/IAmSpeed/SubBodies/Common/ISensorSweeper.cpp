#include "ISensorSweeper.h"
#include "IAmSpeed/World/Analytic/AnalyticWorldQuery.h"
#include "IAmSpeed/World/Analytic/StaticWorldQueryAudit.h"

bool ISensorSweeper::InternalSweep(UWorld* World, const FVector& Start, const FVector& End, SHitResult& OutHit, const float& delta)
{
    if (!World) return false;
	// Only the static-world query is empty; dynamic overlap sweeps still run.
	if (Speed::Analytic::FStaticWorldQueryAudit::TryRecordEmptySingle(
		World, static_cast<uint8>(GetCollisionChannel()), GetResponseParams()))
	{
		OutHit = SHitResult();
		OutHit.bBlockingHit = false;
		return false;
	}

    const Speed::FKinematicState& KS = GetKinematicState();
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
		const FCollisionQueryParams Params = BuildTraceParams();
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
