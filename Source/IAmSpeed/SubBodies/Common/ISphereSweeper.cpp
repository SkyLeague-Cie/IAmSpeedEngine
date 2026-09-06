#include "ISphereSweeper.h"
#include "IAmSpeed/Base/SpeedConstant.h"
#include "IAmSpeed/Base/SUtils.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SphereSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"

namespace
{
    constexpr float SphereBoxContactPointQuantizationCm = 0.1f; // 1 mm

    void QuantizeSphereBoxContactHit(SHitResult& Hit)
    {
        Hit.ImpactNormal = Speed::QuantizeUnitNormal(Hit.ImpactNormal);
        Hit.ImpactPoint = Speed::QuantizeVectorCm(Hit.ImpactPoint, SphereBoxContactPointQuantizationCm);
    }
}

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
	const TArray<TWeakObjectPtr<UBoxSubBody>>& OtherBoxes = GetExternalBoxSubBodies();
	if (OtherBoxes.IsEmpty()) return false;

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
	SHitResult Hit; // Reused locally; unsuccessful candidates do not write it.

    for (const auto& BoxRef : OtherBoxes)
    {
        // Resolve once for this candidate; no pointer is retained across sweeps.
        UBoxSubBody* Box = BoxRef.Get();
        if (!Box) continue;
		if (ShouldSkipBoxSweep(*Box))
			continue;

        // Ignore if box's hitbox already hit this frame
        if (ComponentHasBeenIgnored(*Box))
            continue;

        SSBox BoxShape = Box->MakeBox();
        if (!ThisSphere.TryIntersectNextFrame(BoxShape, DeltaTime, NbSteps, Hit)) continue;

        const float t = Hit.TOI;
        if (t < BestTime)
        {
            BestTime = t;
            LocalBest = Hit;
            BestBox = BoxRef;
            bHit = true;
        }
    }

    if (!bHit)
        return false;

    OutTOI = BestTime;
    OutHit = LocalBest;
    QuantizeSphereBoxContactHit(OutHit);
    OutHit.Component = BestBox;
    OutHit.bBlockingHit = true;
    OutHit.SubBody = BestBox;

    return true;
}

bool ISphereSweeper::SweepVsSpheres(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI)
{
    OutTOI = Delta;
    OutHit = SHitResult();
	const TArray<TWeakObjectPtr<USphereSubBody>>& OtherSpheres = GetExternalSphereSubBodies();
	if (OtherSpheres.IsEmpty()) return false;

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

    for (const auto& OtherSphereRef : OtherSpheres)
    {
        // Resolve once for this candidate; no pointer is retained across sweeps.
        USphereSubBody* OtherSphere = OtherSphereRef.Get();
        if (!OtherSphere) continue;

        // Ignore already-hit Spheres this frame
        if (ComponentHasBeenIgnored(*OtherSphere))
            continue;

        SSphere OSphere = OtherSphere->MakeSphere();

        SHitResult Hit = ThisSphere.IntersectNextFrame(OSphere, Delta);
        if (!Hit.bHit)
            continue;

        if (Hit.TOI < BestTOI)
        {
            BestTOI = Hit.TOI;
            BestHit = Hit;
            BestSphere = OtherSphereRef;
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
	const TArray<TWeakObjectPtr<USWheelSubBody>>& OtherWheels = GetExternalWheelSubBodies();
	if (OtherWheels.IsEmpty()) return false;
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

    for (const auto& OtherWheelRef : OtherWheels)
    {
        // Resolve once for this candidate; no pointer is retained across sweeps.
        USWheelSubBody* OtherWheel = OtherWheelRef.Get();
        if (!OtherWheel) continue;

        // Ignore already-hit Wheels this frame
        if (ComponentHasBeenIgnored(*OtherWheel))
            continue;

        SSphere OSphere = OtherWheel->MakeSphere();

        SHitResult Hit = ThisSphere.IntersectNextFrame(OSphere, Delta);
        if (!Hit.bHit)
            continue;

        if (Hit.TOI < BestTOI)
        {
            BestTOI = Hit.TOI;
            BestHit = Hit;
            BestWheel = OtherWheelRef;
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
