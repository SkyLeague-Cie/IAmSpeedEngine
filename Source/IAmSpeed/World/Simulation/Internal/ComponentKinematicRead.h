#pragma once

// Module-internal live reads. No exported entry point or public issuer.
// This access control is NOT an ownership, lifetime or restoration lease.
class ISpeedComponent;
class ISolidSweeper;
class USpeedWheeledComponent;
class USolidSubBody;
class UBoxSubBody;
class USphereSubBody;
class USWheelSubBody;
class USpeedWorldSubsystem;
namespace Speed { struct FKinematicState; class FSimulationWorld; }

namespace Speed::NativeKinematics
{
struct FCylinderRigidTrajectory;
struct FDynamicBodyWitness;
class FComponentRead final
{
    friend class ::ISolidSweeper;
    friend class ::USpeedWheeledComponent;
    friend class ::USolidSubBody;
    friend class ::UBoxSubBody;
    friend class ::USphereSubBody;
    friend class ::USWheelSubBody;
    friend class ::USpeedWorldSubsystem;
    friend class Speed::FSimulationWorld;
    friend struct FCylinderRigidTrajectory;
    friend struct FDynamicBodyWitness;
    static const Speed::FKinematicState& Current(const ISpeedComponent& Component);
    static const Speed::FKinematicState& ForFrame(const ISpeedComponent& Component, const unsigned int& Frame);
};
}
