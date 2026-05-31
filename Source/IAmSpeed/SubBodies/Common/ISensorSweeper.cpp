#include "ISensorSweeper.h"

bool ISensorSweeper::InternalSweep(UWorld* World, const FVector& Start, const FVector& End, SHitResult& OutHit, const float& delta)
{
    if (!World) return false;

    const Speed::FKinematicState& KS = GetKinematicState();
    FCollisionQueryParams Params = BuildTraceParams();
    FHitResult Hit;
    bool bHit = World->SweepSingleByChannel(
        Hit,
        Start,
        End,
        KS.Rotation,
        GetCollisionChannel(),
        GetCollisionShape(),
        Params,
        GetResponseParams()
    );

    OutHit = SHitResult();
    OutHit.bHit = bHit;
    OutHit.bBlockingHit = false;
    if (bHit)
    {
        OutHit = SHitResult::FromUnrealHit(Hit, delta);
    }
    return bHit;
}
