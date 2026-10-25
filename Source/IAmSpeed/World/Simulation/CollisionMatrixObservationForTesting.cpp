#include "CollisionMatrixObservationForTesting.h"
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "IAmSpeed/SubBodies/SSubBody.h"

namespace Speed::CollisionMatrix042
{
namespace { thread_local FObserver* Active = nullptr; }
FObserver* FObserver::Exchange(FObserver* Observer)
{
    FObserver* Previous = Active;
    Active = Observer;
    return Previous;
}
FResolution FObserver::Before(uint64 Frame, float TimeWithinFrame,
    const USSubBody& Resolver, const SHitResult& Hit)
{
    FResolution R;
    if (!Active) return R;
    R.Frame = Frame;
    R.TimeWithinFrame = TimeWithinFrame;
    R.Hit = Hit;
    R.Owner = Resolver.GetParentComponent();
    if (const USSubBody* Other = Hit.SubBody.Get()) R.Other = Other->GetParentComponent();
    if (R.Owner) R.Before = R.Owner->GetKinematicState();
    if (R.Other) R.OtherBefore = R.Other->GetKinematicState();
    return R;
}
void FObserver::After(FResolution& R)
{
    if (!Active || !R.Owner) return;
    R.After = R.Owner->GetKinematicState();
    if (R.Other) R.OtherAfter = R.Other->GetKinematicState();
    Active->Resolved(R);
}
}
#endif
