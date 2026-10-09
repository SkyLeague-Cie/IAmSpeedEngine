#pragma once
#include "IAmSpeed/Components/Netcode/SpeedPhysicsComponent.h"

/** Versioned physical projection, with no future network authority or history.
 * Sleep and quantization caches are derived and rebuilt after import.
 */
struct IAMSPEED_API FIsolatedMovementState
{
    static constexpr uint32 Version = 1;
    SBaseGameState Game;
    FBasePhysicsState Physical;
    uint16 MinimumCountdown = 0;
    int32 SinceCanMove = INDEX_NONE;
    float EngineFPS = 300;
    bool bEnableSimulation = true, bEnableGravity = true;
    float Mass = 100, GravityZ = -980, MaxSpeed = 10000, MaxAngularSpeed = 100, Damping = 0;
    FVector CenterOfMass = FVector::ZeroVector;
    void Serialize(FArchive& Ar)
    {
        Ar << Game.NumFrame << Game.TestVelocity << Game.TestAngularVelocity << Game.bHasTestVelocity;
        Ar << Physical.Kinematic << Physical.bIsFrozen << Physical.nbFramesbeforeCanMove << Physical.bStartCountdown;
        Ar << MinimumCountdown << SinceCanMove << EngineFPS << bEnableSimulation << bEnableGravity;
        Ar << Mass << GravityZ << MaxSpeed << MaxAngularSpeed << Damping << CenterOfMass;
    }
    bool IsValid() const
    {
        const SKinematic& K = Physical.Kinematic;
        return FMath::IsFinite(EngineFPS) && EngineFPS > 0 && FMath::IsFinite(Mass) && Mass > 0 &&
            FMath::IsFinite(GravityZ) && FMath::IsFinite(MaxSpeed) && MaxSpeed >= 0 &&
            FMath::IsFinite(MaxAngularSpeed) && MaxAngularSpeed >= 0 && FMath::IsFinite(Damping) &&
            !CenterOfMass.ContainsNaN() && !Game.TestVelocity.ContainsNaN() &&
            !Game.TestAngularVelocity.ContainsNaN() && !K.Location.ContainsNaN() &&
            !K.Velocity.ContainsNaN() && !K.Acceleration.ContainsNaN() && K.Rotation.IsNormalized() &&
            !K.AngularVelocity.ContainsNaN() && !K.AngularAcceleration.ContainsNaN();
    }
};
