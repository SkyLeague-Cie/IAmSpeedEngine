#include "IBoxSweeper.h"
#include "IAmSpeed/Base/SpeedConstant.h"
#include "IAmSpeed/Base/SUtils.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SphereSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"

namespace
{
    constexpr float BoxSphereContactPointQuantizationCm = 0.1f; // 1 mm

    void QuantizeBoxSphereContactHit(SHitResult& Hit)
    {
        Hit.ImpactNormal = Speed::QuantizeUnitNormal(Hit.ImpactNormal);
        Hit.ImpactPoint = Speed::QuantizeVectorCm(Hit.ImpactPoint, BoxSphereContactPointQuantizationCm);
    }
}

bool IBoxSweeper::SweepVsGround(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI)
{
    OutTOI = Delta;
    // GroundHit = SHitResult();

	const SKinematic& KS = GetKinematicState();
    const FVector Start = KS.Location;

    const FVector End =
        Start +
        KS.Velocity * Delta +
        0.5f * KS.Acceleration * Delta * Delta;

    SHitResult Hit;
    // SSBox CarBox(KS.Location, BoxExtent, KS.Rotation, KS.Velocity, KS.Acceleration, KS.AngularVelocity, KS.AngularAcceleration);
    // CarBox.DrawDebug(GetWorld());
    if (!InternalSweep(World, Start, End, Hit, Delta))
        return false;

    OutHit = Hit;
    OutTOI = Hit.TOI;
    return true;
}

bool IBoxSweeper::SweepVsSpheres(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI)
{
    OutTOI = Delta;
    OutHit = SHitResult();
	const TArray<TWeakObjectPtr<USphereSubBody>>& OtherSpheres = GetExternalSphereSubBodies();
	if (OtherSpheres.IsEmpty()) return false;
    bool bHit = false;
    float BestTOI = Delta;

    // --- Build hitbox OBB from SubBody kinematics ---
    SSBox ThisBox = MakeBox(); // no need to integrate TimePassed because Box is already at TimePassed

    SHitResult BestHit;
    TWeakObjectPtr<USphereSubBody> BestSphere = nullptr;
	SHitResult Hit; // Misses preserve this scratch; only successful candidates are read.

    for (const auto& OtherSphereRef : OtherSpheres)
    {
        // Resolve once for this candidate; no pointer is retained across sweeps.
        USphereSubBody* OtherSphere = OtherSphereRef.Get();
        if (!OtherSphere) continue;
		if (ShouldSkipSphereSweep(*OtherSphere))
			continue;

        // Ignore already-hit Spheres this frame
        if (ComponentHasBeenIgnored(*OtherSphere))
            continue;

        SSphere OSphere = OtherSphere->MakeSphere(); // useless to pass TimePassed here since every SubBodies are updated to current TimePassed before sweeping

        if (!ThisBox.TryIntersectNextFrame(OSphere, Delta, SpeedConstants::NbCCDSubsteps, Hit))
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
        return false;

    // --- Fill OutHit ---
    OutHit = BestHit;
    QuantizeBoxSphereContactHit(OutHit);
    OutHit.bBlockingHit = true;
    OutHit.Component = BestSphere;
    OutHit.SubBody = BestSphere;
    OutTOI = OutHit.TOI;

    // Debug Log
	// UE_LOG(LogTemp, Log, TEXT("BoxSweeper with kinematics = (%s) might hit SphereSubBody with kinematics = (%s) at ImpactPoint %s and ImpactNormal %s at time %f"), *GetKinematicState().ToString(),
    //    *BestSphere->GetKinematicState().ToString(), *OutHit.ImpactPoint.ToString(), *OutHit.ImpactNormal.ToString(), OutHit.TOI);

    return true;
}

bool IBoxSweeper::SweepVsWheels(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI)
{
    OutTOI = Delta;
    OutHit = SHitResult();
	const TArray<TWeakObjectPtr<USWheelSubBody>>& OtherWheels = GetExternalWheelSubBodies();
	if (OtherWheels.IsEmpty()) return false;
    bool bHit = false;
    float BestTOI = Delta;
    // --- Build hitbox OBB from SubBody kinematics ---
    SSBox ThisBox = MakeBox();
    SHitResult BestHit;
    TWeakObjectPtr<USWheelSubBody> BestWheel = nullptr;
	SHitResult Hit;
    for (const auto& OtherWheelRef : OtherWheels)
    {
        // Resolve once for this candidate; no pointer is retained across sweeps.
        USWheelSubBody* OtherWheel = OtherWheelRef.Get();
        if (!OtherWheel) continue;

        // Ignore already-hit Wheels this frame
        if (ComponentHasBeenIgnored(*OtherWheel))
            continue;

        SSphere OWheel = OtherWheel->MakeSphere(); // useless to pass TimePassed here since every SubBodies are updated to current TimePassed before sweeping

        if (!ThisBox.TryIntersectNextFrame(OWheel, Delta, SpeedConstants::NbCCDSubsteps, Hit))
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

    // --- Fill OutHit ---
    OutHit = BestHit;
    OutHit.bBlockingHit = true;
    OutHit.Component = BestWheel;
    OutHit.SubBody = BestWheel;
    OutTOI = BestHit.TOI;
    return true;
}

SSBox IBoxSweeper::MakeBox() const
{
    const auto& CrtKinematics = GetKinematicState();
    return SSBox(
        CrtKinematics.Location,
        GetBoxExtent(),
        CrtKinematics.Rotation,
        CrtKinematics.Velocity,
        CrtKinematics.Acceleration,
        CrtKinematics.AngularVelocity,
        CrtKinematics.AngularAcceleration
    );
}

bool IBoxSweeper::SweepVsBoxes(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI)
{
    OutTOI = Delta;
	// Preserve the existing miss contract: unlike the other sweeps, this one
	// leaves OutHit untouched when no candidate hits.
	const TArray<TWeakObjectPtr<UBoxSubBody>>& OtherBoxes = GetExternalBoxSubBodies();
	if (OtherBoxes.IsEmpty()) return false;

    bool bHit = false;
    float BestTOI = Delta + 1.f;
    SHitResult BestHit;
    SHitResult LocalBest;

    // This hitbox (already at TimePassed)
    SSBox ThisBox = MakeBox();
    TWeakObjectPtr<UBoxSubBody> BestBox = nullptr;

    for (const auto& BoxRef : OtherBoxes)
    {
        // Resolve once for this candidate; no pointer is retained across sweeps.
        UBoxSubBody* Box = BoxRef.Get();
        if (!Box) continue;

        // Ignore if box's hitbox already hit this frame
        if (ComponentHasBeenIgnored(*Box))
            continue;

        SSBox BoxShape = Box->MakeBox();
        SHitResult Hit = ThisBox.IntersectNextFrame(BoxShape, Delta, SpeedConstants::NbCCDSubsteps);
        if (!Hit.bHit) continue;

        const float t = Hit.TOI;
        if (t < BestTOI)
        {
            BestTOI = t;
            LocalBest = Hit;
            BestBox = BoxRef;
            bHit = true;
        }
    }

    if (!bHit)
        return false;

    // Fill OutHit (future hit result for selection)
    OutHit.bBlockingHit = true;
    OutHit = LocalBest;
    OutHit.Component = BestBox;
    OutHit.bBlockingHit = true;
    OutHit.SubBody = BestBox;

    OutTOI = BestTOI;
    return true;
}
