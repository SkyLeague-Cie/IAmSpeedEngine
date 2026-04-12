#include "ISphereSweeper.h"
#include "IAmSpeed/Base/SpeedConstant.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SphereSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"

bool ISphereSweeper::SweepVsGround(UWorld* World, SHitResult& OutHit, const float& DeltaTime, float& OutTOI)
{
    OutTOI = DeltaTime;
    OutHit = SHitResult();

    const Speed::FKinematicState& KS = GetKinematicState();
    const FVector Start = KS.Location;
    const FVector End =
        Start +
        KS.Velocity * DeltaTime +
        0.5f * KS.Acceleration * DeltaTime * DeltaTime;

    SHitResult Hit;
    if (!InternalSweep(World, Start, End, Hit, DeltaTime))
        return false;

    OutHit = Hit;
    OutTOI = Hit.TOI;
    return true;
}

bool ISphereSweeper::SweepVsBoxes(UWorld* World, SHitResult& OutHit, const float& DeltaTime, float& OutTOI)
{
    OutTOI = DeltaTime;
    OutHit = SHitResult();

    const Speed::FKinematicState& KS = GetKinematicState();
    const SSphere ThisSphere(
        KS.Location,
        GetRadiusWithMargin(),
        KS.Velocity,
        KS.Acceleration
    );

    const uint8 NbSteps = SpeedConstants::NbCCDSubsteps;

    bool bHit = false;
    float BestTime = DeltaTime + 1.f;
    SHitResult LocalBest(false, FVector::ZeroVector, FVector::ZeroVector, 0);
    TWeakObjectPtr<UBoxSubBody> BestBox = nullptr;
    const TArray<TWeakObjectPtr<UBoxSubBody>>& OtherBoxes = GetExternalBoxSubBodies();

    for (auto& Box : OtherBoxes)
    {
        if (!Box.IsValid()) continue;

        // Ignore if box's hitbox already hit this frame
        if (ComponentHasBeenIgnored(*Box.Get()))
            continue;

        SSBox BoxShape = Box->MakeBox();
        SHitResult Hit = ThisSphere.IntersectNextFrame(BoxShape, DeltaTime, NbSteps);
        if (!Hit.bHit) continue;

        const float t = Hit.TOI;
        if (t < BestTime)
        {
            BestTime = t;
            LocalBest = Hit;
            BestBox = Box;
            bHit = true;
        }
    }

    if (!bHit)
        return false;

    OutTOI = BestTime;
    OutHit = LocalBest;
    OutHit.Component = BestBox;
    OutHit.bBlockingHit = true;
    OutHit.SubBody = BestBox;

    return true;
}

bool ISphereSweeper::SweepVsSpheres(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI)
{
    OutTOI = Delta;
    OutHit = SHitResult();

    const Speed::FKinematicState& KS = GetKinematicState();
    const SSphere ThisSphere(
        KS.Location,
        GetRadiusWithMargin(),
        KS.Velocity,
        KS.Acceleration
    );

    bool bHit = false;
    float BestTOI = Delta;

    SHitResult BestHit;
    TWeakObjectPtr<USphereSubBody> BestSphere = nullptr;
    const TArray<TWeakObjectPtr<USphereSubBody>>& OtherSpheres = GetExternalSphereSubBodies();

    for (auto& OtherSphere : OtherSpheres)
    {
        if (!OtherSphere.IsValid()) continue;

        // Ignore already-hit Spheres this frame
        if (ComponentHasBeenIgnored(*OtherSphere.Get()))
            continue;

        SSphere OSphere = OtherSphere->MakeSphere();

        SHitResult Hit = ThisSphere.IntersectNextFrame(OSphere, Delta);
        if (!Hit.bHit)
            continue;

        if (Hit.TOI < BestTOI)
        {
            BestTOI = Hit.TOI;
            BestHit = Hit;
            BestSphere = OtherSphere;
            bHit = true;
        }
    }

    if (!bHit)
    {
        return false;
    }

    // Fill hit result (DO NOT resolve yet)
    OutHit = BestHit;
    OutHit.bBlockingHit = true;
    OutHit.Component = BestSphere;
    OutHit.SubBody = BestSphere;
    OutTOI = BestTOI;
    return true;
}

bool ISphereSweeper::SweepVsWheels(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI)
{
    OutTOI = Delta;
    OutHit = SHitResult();
    const Speed::FKinematicState& KS = GetKinematicState();
    const SSphere ThisSphere(
        KS.Location,
        GetRadiusWithMargin(),
        KS.Velocity,
        KS.Acceleration
    );

    bool bHit = false;
    float BestTOI = Delta;

    SHitResult BestHit;
    TWeakObjectPtr<USWheelSubBody> BestWheel = nullptr;
    const TArray<TWeakObjectPtr<USWheelSubBody>>& OtherWheels = GetExternalWheelSubBodies();

    for (auto& OtherWheel : OtherWheels)
    {
        if (!OtherWheel.IsValid()) continue;

        // Ignore already-hit Wheels this frame
        if (ComponentHasBeenIgnored(*OtherWheel.Get()))
            continue;

        SSphere OSphere = OtherWheel->MakeSphere();

        SHitResult Hit = ThisSphere.IntersectNextFrame(OSphere, Delta);
        if (!Hit.bHit)
            continue;

        if (Hit.TOI < BestTOI)
        {
            BestTOI = Hit.TOI;
            BestHit = Hit;
            BestWheel = OtherWheel;
            bHit = true;
        }
    }

    if (!bHit)
        return false;

    // Fill hit result (DO NOT resolve yet)
    OutHit = BestHit;
    OutHit.bBlockingHit = true;
    OutHit.Component = BestWheel;
    OutHit.SubBody = BestWheel;
    OutTOI = BestTOI;
    return true;
}

SSphere ISphereSweeper::MakeSphere() const
{
    const SKinematic& KS = GetKinematicState();
    return SSphere(
        KS.Location,
        GetRadiusWithMargin(),
        KS.Velocity,
        KS.Acceleration
    );
}
