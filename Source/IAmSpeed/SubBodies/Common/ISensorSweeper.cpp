#include "ISensorSweeper.h"
#include "IAmSpeed/World/Analytic/StaticWorldQueryAudit.h"

bool ISensorSweeper::InternalSweep(UWorld* World, const FVector& Start, const FVector& End, SHitResult& OutHit, const float& delta)
{
    if (!World) return false;

    const Speed::FKinematicState& KS = GetKinematicState();
    FCollisionQueryParams Params = BuildTraceParams();
    FHitResult Hit;
	bool bHit = false;
	if (!Speed::Analytic::FStaticWorldQueryAudit::TryCompactAuthoritySingle(
		Start, End, KS.Rotation, GetCollisionShape(),
		static_cast<uint8>(GetCollisionChannel()), GetResponseParams(), Hit, bHit))
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
        OutHit = SHitResult::FromUnrealHit(Hit, delta);
    }
    return bHit;
}
