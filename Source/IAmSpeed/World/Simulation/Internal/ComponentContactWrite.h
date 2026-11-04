#pragma once
namespace Speed::FrameParallel {class FJoinedActorKinematics1213;}

// Private native mutations only; no exported issuer or source/restore lease.
#include "IAmSpeed/Base/PhysicalContactConstraint.h"
#include "IAmSpeed/Base/Kinematic.h"
class ISpeedComponent;class USpeedWheeledComponent;
class USolidSubBody;
class UBoxSubBody;
class USpeedWorldSubsystem;
namespace Speed {class IStaticCollisionWorld;}
namespace Speed::Netcode::Prediction { class FPrivateRoadPhysicsWorld; class FPrivateNativeOperation; }
namespace Speed::NativeKinematics
{
class FBodyWrite;
class FComponentWrite final
{
    friend class ::ISpeedComponent;
    friend class ::USpeedWheeledComponent;
    friend class FBodyWrite;
    friend class ::USolidSubBody;
    friend class ::UBoxSubBody;
    friend class ::USpeedWorldSubsystem;
    friend class Speed::Netcode::Prediction::FPrivateRoadPhysicsWorld;
    friend class Speed::FrameParallel::FJoinedActorKinematics1213;
    static void Impulse(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,ISpeedComponent* Component,const FVector& Impulse,const FVector& Point,const USolidSubBody* SubBody);
    static void ImpulseBatch(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,ISpeedComponent* Component,const TArray<FVector>& Impulses,const TArray<FVector>& WorldPoints,const USolidSubBody* SubBody);
    static void Force(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,ISpeedComponent* Component,const FVector& Force,const FVector& WorldPoint,const USolidSubBody* SubBody);
    static void ForceBatch(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,ISpeedComponent* Component,const TArray<FVector>& Forces,const TArray<FVector>& WorldPoints,const USolidSubBody* SubBody);
    static void FrameStaticWorld(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,ISpeedComponent* Component,const Speed::IStaticCollisionWorld* World,float FrameHorizon);
    static void UpdateSubBodies(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,ISpeedComponent* Component);
    static void ResetFrame(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,ISpeedComponent* Component,const float& Delta);
    static TArray<FPhysicalContactConstraint>& FrameConstraints(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,ISpeedComponent& Component);
    static void BirthPose(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,ISpeedComponent* Component,const FQuat& Rotation,const FVector& Location);
    static void Kinematic(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,ISpeedComponent& Component,const SKinematic& State);
    static void RegisterConstraint(ISpeedComponent& Component,const FPhysicalContactConstraint& Constraint);
    static void ClearConstraints(ISpeedComponent& Component);
};
}
