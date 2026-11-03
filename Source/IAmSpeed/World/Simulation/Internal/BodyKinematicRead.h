#pragma once
namespace Speed::FrameParallel {class FJoinedActorKinematics1213;}

// Module-internal live references. Private call sites only; not a restore lease.
#include "IAmSpeed/Base/SHitResult.h"
class UPrimitiveComponent;
class ISpeedComponent; class USpeedMovementComponent; class USpeedWheeledComponent;
class USSubBody; class ISweeper; class ISolidSweeper; class ISensorSweeper;
class IBoxSweeper; class ISphereSweeper; class ISpeedWheeledComponent;
class UBoxSubBody; class USphereSubBody; class USolidSubBody; class USWheelSubBody;
class UCylinderSubBody; class USensorSubBody; class USpeedWorldSubsystem;
class USpeedBranchBoxSubBody; class USpeedBranchWheelSubBody;
namespace Speed { struct FKinematicState; }
namespace Speed::FrameParallel { class FJoinedActorKinematics1213; }
namespace Speed::Netcode::Prediction { class FPrivateRoadPhysicsWorld; class FPrivateNativeOperation; }
namespace Speed::NativeKinematics
{
struct FCylinderRigidTrajectory; struct FCylinderSuspensionQuery; struct FDynamicBodyWitness;
class FBodyRead final
{
    friend class ::ISolidSweeper; friend class ::ISensorSweeper;
    friend class ::USSubBody; friend class ::ISpeedComponent;
    friend class ::USpeedMovementComponent; friend class ::USpeedWheeledComponent;
    friend class ::IBoxSweeper; friend class ::ISphereSweeper;
    friend class ::ISpeedWheeledComponent; friend class ::UBoxSubBody;
    friend class ::USphereSubBody; friend class ::USolidSubBody;
    friend class ::USWheelSubBody; friend class ::UCylinderSubBody;
    friend class ::USensorSubBody; friend class ::USpeedWorldSubsystem;
    friend class ::USpeedBranchBoxSubBody; friend class ::USpeedBranchWheelSubBody;
    friend class Speed::FrameParallel::FJoinedActorKinematics1213;
    friend class Speed::Netcode::Prediction::FPrivateRoadPhysicsWorld;
    friend class Speed::FrameParallel::FJoinedActorKinematics1213;
    friend struct FCylinderRigidTrajectory; friend struct FCylinderSuspensionQuery;
    friend struct FDynamicBodyWitness;
    static FMatrix InitialInverseTensor(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody* Body);
    static FMatrix WorldInvInertia(const USolidSubBody& Body);
    static FMatrix WorldInvInertia(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body);
    static FMatrix WorldInvInertia(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody* Body);
    static ISpeedComponent* Parent(const USSubBody& Body);
    static ISpeedComponent* Parent(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USSubBody& Body);
    static const Speed::FKinematicState& Body(const USSubBody& Body);
    static const Speed::FKinematicState& Body(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USSubBody& Body);
    static const Speed::FKinematicState& Sweeper(const ISweeper& Sweeper);
    static const Speed::FKinematicState& Sweeper(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISweeper& Sweeper);
    static const SHitResult& CurrentHit(const USSubBody& Body);
    static const SHitResult& CurrentHit(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USSubBody& Body);
    static const SHitResult& FutureHit(const USSubBody& Body);
    static const SHitResult& FutureHit(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USSubBody& Body);
    static bool HasHit(const USSubBody& Body);
    static bool HasHit(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USSubBody& Body);
    static bool WillHit(const USSubBody& Body);
    static bool WillHit(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USSubBody& Body);
    static bool ComponentIgnored(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USSubBody& Body,const UPrimitiveComponent* Other);
    static float FutureTOI(const USSubBody& Body);
    static float FutureTOI(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USSubBody& Body);
};
}
