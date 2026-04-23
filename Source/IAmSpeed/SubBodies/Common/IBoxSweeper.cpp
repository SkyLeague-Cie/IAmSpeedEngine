#include "IBoxSweeper.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SphereSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"

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
    bool bHit = false;
    float BestTOI = Delta;

    // --- Build hitbox OBB from SubBody kinematics ---
    SSBox ThisBox = MakeBox(); // no need to integrate TimePassed because Box is already at TimePassed

    SHitResult BestHit;
    TWeakObjectPtr<USphereSubBody> BestSphere = nullptr;
    const TArray<TWeakObjectPtr<USphereSubBody>>& OtherSpheres = GetExternalSphereSubBodies();

    for (auto& OtherSphere : OtherSpheres)
    {
        if (!OtherSphere.IsValid()) continue;

        // Ignore already-hit Spheres this frame
        if (ComponentHasBeenIgnored(*OtherSphere.Get()))
            continue;

        SSphere OSphere = OtherSphere->MakeSphere(); // useless to pass TimePassed here since every SubBodies are updated to current TimePassed before sweeping

        SHitResult Hit = ThisBox.IntersectNextFrame(OSphere, Delta, SpeedConstants::NbCCDSubsteps);
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
        return false;

    // --- Fill OutHit ---
    OutHit = BestHit;
    OutHit.bBlockingHit = true;
    OutHit.Component = BestSphere;
    OutHit.SubBody = BestSphere;
    OutTOI = BestHit.TOI;
    return true;
}

bool IBoxSweeper::SweepVsWheels(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI)
{
    OutTOI = Delta;
    OutHit = SHitResult();
    bool bHit = false;
    float BestTOI = Delta;
    // --- Build hitbox OBB from SubBody kinematics ---
    SSBox ThisBox = MakeBox();
    SHitResult BestHit;
    TWeakObjectPtr<USWheelSubBody> BestWheel = nullptr;
    const TArray<TWeakObjectPtr<USWheelSubBody>>& OtherWheels = GetExternalWheelSubBodies();
    for (auto& OtherWheel : OtherWheels)
    {
        if (!OtherWheel.IsValid()) continue;

        // Ignore already-hit Wheels this frame
        if (ComponentHasBeenIgnored(*OtherWheel.Get()))
            continue;

        SSphere OWheel = OtherWheel->MakeSphere(); // useless to pass TimePassed here since every SubBodies are updated to current TimePassed before sweeping

        SHitResult Hit = ThisBox.IntersectNextFrame(OWheel, Delta, SpeedConstants::NbCCDSubsteps);
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

    bool bHit = false;
    float BestTOI = Delta + 1.f;
    SHitResult BestHit;
    SHitResult LocalBest;

    // This hitbox (already at TimePassed)
    SSBox ThisBox = MakeBox();
    TWeakObjectPtr<UBoxSubBody> BestBox = nullptr;
    const TArray<TWeakObjectPtr<UBoxSubBody>>& OtherBoxes = GetExternalBoxSubBodies();

    for (auto& Box : OtherBoxes)
    {
        if (!Box.IsValid()) continue;

        // Ignore if box's hitbox already hit this frame
        if (ComponentHasBeenIgnored(*Box.Get()))
            continue;

        SSBox BoxShape = Box->MakeBox();
        SHitResult Hit = ThisBox.IntersectNextFrame(BoxShape, Delta, SpeedConstants::NbCCDSubsteps);
        if (!Hit.bHit) continue;

        const float t = Hit.TOI;
        if (t < BestTOI)
        {
            BestTOI = t;
            LocalBest = Hit;
            BestBox = Box;
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