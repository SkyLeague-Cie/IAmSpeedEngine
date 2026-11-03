#include "ComponentContactWrite.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateNativeStorageMap.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadNativeObjects.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
namespace Speed::NativeKinematics
{
void FComponentWrite::RegisterConstraint(ISpeedComponent& Component,const FPhysicalContactConstraint& Constraint)
{
    Component.RegisterPhysicalConstraintForPhysics(Constraint);
}
void FComponentWrite::ClearConstraints(ISpeedComponent& Component)
{
    Component.ClearPhysicalConstraintsForPhysics();
}
void FComponentWrite::Kinematic(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,ISpeedComponent& Component,const SKinematic& State)
{
    // Exactly the final native wheeled SetKinematicState assignment. The
    // actual typed backing is obtained before any mutation; no public callback.
    auto& Values=Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<USpeedBranchWheeledComponent::FOwnedComponentValues>(Operation,&Component);
    Values.BasePhysicsState.Kinematic=State;
}

void FComponentWrite::Impulse(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,ISpeedComponent* Component,const FVector& Impulse,const FVector& Point,const USolidSubBody* SubBody)
{
    // Validate raw adjusted component subject BEFORE its first dereference.
    (void)Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<USpeedBranchWheeledComponent::FOwnedComponentValues>(Operation,Component);
    Component->AddPhysImpulseAtPointNative(Operation,Impulse,Point,SubBody);
}

void FComponentWrite::ImpulseBatch(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,ISpeedComponent* Component,const TArray<FVector>& Impulses,const TArray<FVector>& WorldPoints,const USolidSubBody* SubBody)
{
    // Raw adjusted subject is checked before its first dereference.
    (void)Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<USpeedBranchWheeledComponent::FOwnedComponentValues>(Operation,Component);
    Component->AddPhysImpulseBatchAtPointsNative(Operation,Impulses,WorldPoints,SubBody);
}
void FComponentWrite::Force(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,ISpeedComponent* Component,const FVector& Force,const FVector& WorldPoint,const USolidSubBody* SubBody)
{
    // Raw adjusted subject is checked before its first dereference.
    (void)Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<USpeedBranchWheeledComponent::FOwnedComponentValues>(Operation,Component);
    Component->AddPhysForceAtPointNative(Operation,Force,WorldPoint,SubBody);
}
void FComponentWrite::ForceBatch(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,ISpeedComponent* Component,const TArray<FVector>& Forces,const TArray<FVector>& WorldPoints,const USolidSubBody* SubBody)
{
    // Raw adjusted subject is checked before its first dereference.
    (void)Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<USpeedBranchWheeledComponent::FOwnedComponentValues>(Operation,Component);
    Component->AddPhysForceBatchAtPointsNative(Operation,Forces,WorldPoints,SubBody);
}

void FComponentWrite::FrameStaticWorld(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,ISpeedComponent* Component,const Speed::IStaticCollisionWorld* World,float FrameHorizon)
{
    // Original frame-scoped borrowed pointer and horizon only. This creates
    // no ownership or source/lifecycle certificate for the collision world.
    auto& Values=Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<USpeedBranchWheeledComponent::FOwnedComponentValues>(Operation,Component);
    Values.StaticRestingWorld = World;
    Values.StaticSupportFrameHorizon = World ? FrameHorizon : 0;
}

void FComponentWrite::UpdateSubBodies(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,ISpeedComponent* Component)
{
    (void)Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<USpeedBranchWheeledComponent::FOwnedComponentValues>(Operation,Component);
    Component->UpdateSubBodiesKinematicsNative(Operation);
}

}
