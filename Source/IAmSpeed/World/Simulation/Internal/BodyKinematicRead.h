#pragma once
// Module-internal live references. Private call sites only; not a restore lease.
class USSubBody; class ISweeper; class ISolidSweeper; class ISensorSweeper;
class IBoxSweeper; class ISphereSweeper; class ISpeedWheeledComponent;
class UBoxSubBody; class USphereSubBody; class USolidSubBody; class USWheelSubBody;
class UCylinderSubBody; class USensorSubBody; class USpeedWorldSubsystem;
class USpeedBranchBoxSubBody; class USpeedBranchWheelSubBody;
namespace Speed { struct FKinematicState; }
namespace Speed::FrameParallel { class FJoinedActorKinematics1213; }
namespace Speed::Netcode::Prediction { class FPrivateRoadPhysicsWorld; }
namespace Speed::NativeKinematics
{
struct FCylinderRigidTrajectory; struct FCylinderSuspensionQuery; struct FDynamicBodyWitness;
class FBodyRead final
{
    friend class ::ISolidSweeper; friend class ::ISensorSweeper;
    friend class ::IBoxSweeper; friend class ::ISphereSweeper;
    friend class ::ISpeedWheeledComponent; friend class ::UBoxSubBody;
    friend class ::USphereSubBody; friend class ::USolidSubBody;
    friend class ::USWheelSubBody; friend class ::UCylinderSubBody;
    friend class ::USensorSubBody; friend class ::USpeedWorldSubsystem;
    friend class ::USpeedBranchBoxSubBody; friend class ::USpeedBranchWheelSubBody;
    friend class Speed::FrameParallel::FJoinedActorKinematics1213;
    friend class Speed::Netcode::Prediction::FPrivateRoadPhysicsWorld;
    friend struct FCylinderRigidTrajectory; friend struct FCylinderSuspensionQuery;
    friend struct FDynamicBodyWitness;
    static const Speed::FKinematicState& Body(const USSubBody& Body);
    static const Speed::FKinematicState& Sweeper(const ISweeper& Sweeper);
};
}
