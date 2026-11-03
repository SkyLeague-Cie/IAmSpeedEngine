#include "ComponentKinematicRead.h"
#include "IAmSpeed/SubBodies/Configs/WheelSubBodyConfig.h"
#include "IAmSpeed/SubBodies/Configs/SubBodyConfig.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateNativeStorageMap.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadNativeObjects.h"
#include "IAmSpeed/Base/SpeedConstant.h"
#include "OriginKinematicView.h"
#include "IAmSpeed/Components/ISpeedComponent.h"

namespace Speed::NativeKinematics
{
SubBodyConfig FComponentRead::BirthSubBodyConfiguration(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component,const USSubBody& Body)
{
    using namespace Speed::Netcode::Prediction;
    FPrivateNativeStorageAccess::RequireBirth(Operation);
    (void)FPrivateNativeStorageAccess::Part<const USpeedBranchWheeledComponent::FOwnedComponentValues>(Operation,&Component);
    (void)FPrivateNativeStorageAccess::Part<const USpeedWheeledComponent::FOriginalWheeledDomain>(Operation,&Component);
    (void)FPrivateNativeStorageAccess::Part<const FPrivateRoadSubBodyValues>(Operation,&Body);
    // Exact declared value of the sealed original parent GetSubBodyConfig.
    // A missing binding refused above; this is not a zero-state substitute.
    return SubBodyConfig{};
}
WheelSubBodyConfig FComponentRead::BirthWheelConfiguration(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedWheeledComponent& Component,const USWheelSubBody& Body)
{
    using namespace Speed::Netcode::Prediction;
    FPrivateNativeStorageAccess::RequireBirth(Operation);
    (void)FPrivateNativeStorageAccess::Part<const USpeedBranchWheeledComponent::FOwnedComponentValues>(Operation,static_cast<const ISpeedComponent*>(&Component));
    (void)FPrivateNativeStorageAccess::Part<const USpeedWheeledComponent::FOriginalWheeledDomain>(Operation,static_cast<const ISpeedComponent*>(&Component));
    (void)FPrivateNativeStorageAccess::Part<const FPrivateRoadWheelDomain>(Operation,&Body);
    // Exact inherited stock GetWheelSubBodyConfig result, not guessed coverage.
    return WheelSubBodyConfig{};
}
bool FComponentRead::PrescribedMotion(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component)
{
    ValidateCurrentAndSubBodies(Operation,&Component);
    return Component.IsPrescribedMotion(); // original key-free scalar virtual, no Op passed
}
const TArray<FPhysicalContactConstraint>& FComponentRead::Constraints(const ISpeedComponent& Component)
{
    return Component.PhysicalConstraintsStorage();
}
void FComponentRead::ValidateCurrentAndSubBodies(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent* Component)
{
    // Both true raw adjusted subjects before any UObject dereference.
    using namespace Speed::Netcode::Prediction;
    (void)FPrivateNativeStorageAccess::Part<const USpeedBranchWheeledComponent::FOwnedComponentValues>(Operation,Component);
    (void)FPrivateNativeStorageAccess::Part<const USpeedWheeledComponent::FOriginalWheeledDomain>(Operation,Component);
}
const TArray<USSubBody*>& FComponentRead::SubBodies(const ISpeedComponent& Component)
{return Component.GetSubBodies();}
const TArray<USSubBody*>& FComponentRead::SubBodies(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component)
{return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const USpeedWheeledComponent::FOriginalWheeledDomain>(Operation,&Component).SubBodies;}
const Speed::FKinematicState& FComponentRead::Current(const ISpeedComponent& Component)
{
    return Component.ReadPhysicsKinematicState();
}
const Speed::FKinematicState& FComponentRead::ForFrame(const ISpeedComponent& Component, const unsigned int& Frame)
{
    return Component.ReadPhysicsKinematicStateForFrame(Frame);
}
const FQuat& FComponentRead::Rotation(const ISpeedComponent& Component)
{
    return Component.ReadPhysicsKinematicState().Rotation;
}
const FVector& FComponentRead::COMVelocity(const ISpeedComponent& Component)
{
	return Component.ReadPhysicsKinematicState().Velocity;
}
const FVector& FComponentRead::AngularVelocity(const ISpeedComponent& Component)
{
	return Component.ReadPhysicsKinematicState().AngularVelocity;
}
const FVector& FComponentRead::Acceleration(const ISpeedComponent& Component)
{
    return Component.ReadPhysicsKinematicState().Acceleration;
}
const FVector& FComponentRead::AngularAcceleration(const ISpeedComponent& Component)
{
	return Component.ReadPhysicsKinematicState().AngularAcceleration;
}
const FVector& FComponentRead::COM(const ISpeedComponent& Component)
{
    return Component.ReadPhysicsCOM();
}
const FVector& FComponentRead::LocalCOM(const ISpeedComponent& Component)
{
    return Component.ReadPhysicsCenterOfMassLocal();
}
FVector FComponentRead::Location(const ISpeedComponent& Component)
{
	// Keep the historical-frame selection used by the complete origin getter.
	const SKinematic& COMState = Component.ReadPhysicsKinematicStateForFrame(Component.NumFrame());
	return FOriginKinematicView(COMState, FComponentRead::LocalCOM(Component)).Location();
}
FVector FComponentRead::Velocity(const ISpeedComponent& Component)
{
	const SKinematic& COMState = Component.ReadPhysicsKinematicStateForFrame(Component.NumFrame());
	return FOriginKinematicView(COMState, FComponentRead::LocalCOM(Component)).Velocity();
}
FVector FComponentRead::VelocityAtPoint(const ISpeedComponent& Component, const FVector& Point)
{
    // v + w x r
    const FVector R = Point - FComponentRead::COM(Component);
    return FComponentRead::COMVelocity(Component) + FVector::CrossProduct(FComponentRead::AngularVelocity(Component), R);
}
FVector FComponentRead::AccelerationAtPoint(const ISpeedComponent& Component, const FVector& Point)
{
    // a + alpha x r + w x (w x r)
    const FVector R = Point - FComponentRead::COM(Component);
    return FComponentRead::Acceleration(Component) + FVector::CrossProduct(FComponentRead::AngularAcceleration(Component), R) + FVector::CrossProduct(FComponentRead::AngularVelocity(Component), FVector::CrossProduct(FComponentRead::AngularVelocity(Component), R));
}
FVector FComponentRead::Forward(const ISpeedComponent& Component)
{ return FComponentRead::Rotation(Component).GetForwardVector(); }
FVector FComponentRead::Right(const ISpeedComponent& Component)
{ return FComponentRead::Rotation(Component).GetRightVector(); }
FVector FComponentRead::Up(const ISpeedComponent& Component)
{ return FComponentRead::Rotation(Component).GetUpVector(); }
float FComponentRead::ForwardSpeed(const ISpeedComponent& Component)
{ return FVector::DotProduct(FComponentRead::COMVelocity(Component), FComponentRead::Forward(Component)); }
bool FComponentRead::GoingForward(const ISpeedComponent& Component)
{ return FComponentRead::ForwardSpeed(Component) > 0; }
Speed::FKinematicState FComponentRead::Origin(const ISpeedComponent& Component)
{
	return FComponentRead::OriginForFrame(Component, Component.NumFrame());
}
Speed::FKinematicState FComponentRead::OriginForFrame(const ISpeedComponent& Component, const unsigned int& NumFrameToRead)
{
	const SKinematic& COMState = Component.ReadPhysicsKinematicStateForFrame(NumFrameToRead);
	SKinematic OriginState = COMState;
	const FOriginKinematicView Origin(COMState, FComponentRead::LocalCOM(Component));
	OriginState.Location = Origin.Location();
	OriginState.Velocity = Origin.Velocity();
	OriginState.Acceleration = Origin.Acceleration();
	return OriginState;
}

// Explicit native reads require the genuine owner operation. The legacy
// FView above remains unchanged; no new Operation-holding proxy is returned.
const Speed::IStaticCollisionWorld* FComponentRead::StaticWorld(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component)
{
    return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const USpeedBranchWheeledComponent::FOwnedComponentValues>(Operation,&Component).StaticRestingWorld;
}
unsigned int FComponentRead::NativeFrame(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component)
{
    return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const USpeedBranchWheeledComponent::FOwnedComponentValues>(Operation,&Component).BaseGameState.NumFrame;
}
const TArray<FPhysicalContactConstraint>& FComponentRead::Constraints(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component)
{
    return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const USpeedBranchWheeledComponent::FOwnedComponentValues>(Operation,&Component).PhysicalConstraints;
}
const Speed::FKinematicState& FComponentRead::Current(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component)
{
    return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const USpeedBranchWheeledComponent::FOwnedComponentValues>(Operation,&Component).BasePhysicsState.Kinematic;
}
const Speed::FKinematicState& FComponentRead::ForFrame(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component, const unsigned int& Frame)
{
    const auto& Domain=Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const USpeedWheeledComponent::FOriginalWheeledDomain>(Operation,&Component);
    // Unknown/malformed native history refuses; never fabricate a zero state
    // or treat a missing record as measured equality.
    if(Domain.RecordedBaseFrames.Num()!=SpeedConstants::RecordedHistorySize
        ||Domain.RecordedBaseStates.Num()!=SpeedConstants::RecordedHistorySize)
        throw Speed::Netcode::Prediction::FPrivateNativeStorageUnavailable{};
    const int32 Idx=Frame%SpeedConstants::RecordedHistorySize;
    if(Domain.RecordedBaseFrames[Idx]==Frame)return Domain.RecordedBaseStates[Idx].Kinematic;
    return FComponentRead::Current(Operation,Component);
}
const FQuat& FComponentRead::Rotation(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component)
{
    return FComponentRead::Current(Operation,Component).Rotation;
}
const FVector& FComponentRead::COMVelocity(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component)
{
	return FComponentRead::Current(Operation,Component).Velocity;
}
const FVector& FComponentRead::AngularVelocity(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component)
{
	return FComponentRead::Current(Operation,Component).AngularVelocity;
}
const FVector& FComponentRead::Acceleration(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component)
{
    return FComponentRead::Current(Operation,Component).Acceleration;
}
const FVector& FComponentRead::AngularAcceleration(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component)
{
	return FComponentRead::Current(Operation,Component).AngularAcceleration;
}
const FVector& FComponentRead::COM(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component)
{
    return FComponentRead::Current(Operation,Component).Location;
}
const FVector& FComponentRead::LocalCOM(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component)
{
    return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const USpeedWheeledComponent::FOriginalWheeledDomain>(Operation,&Component).CenterOfMass;
}
FVector FComponentRead::Location(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component)
{
	// Keep the historical-frame selection used by the complete origin getter.
	const SKinematic& COMState = FComponentRead::ForFrame(Operation,Component,FComponentRead::NativeFrame(Operation,Component));
	return FOriginKinematicView(COMState, FComponentRead::LocalCOM(Operation,Component)).Location();
}
FVector FComponentRead::Velocity(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component)
{
	const SKinematic& COMState = FComponentRead::ForFrame(Operation,Component,FComponentRead::NativeFrame(Operation,Component));
	return FOriginKinematicView(COMState, FComponentRead::LocalCOM(Operation,Component)).Velocity();
}
FVector FComponentRead::VelocityAtPoint(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component, const FVector& Point)
{
    // v + w x r
    const FVector R = Point - FComponentRead::COM(Operation,Component);
    return FComponentRead::COMVelocity(Operation,Component) + FVector::CrossProduct(FComponentRead::AngularVelocity(Operation,Component), R);
}
FVector FComponentRead::AccelerationAtPoint(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component, const FVector& Point)
{
    // a + alpha x r + w x (w x r)
    const FVector R = Point - FComponentRead::COM(Operation,Component);
    return FComponentRead::Acceleration(Operation,Component) + FVector::CrossProduct(FComponentRead::AngularAcceleration(Operation,Component), R) + FVector::CrossProduct(FComponentRead::AngularVelocity(Operation,Component), FVector::CrossProduct(FComponentRead::AngularVelocity(Operation,Component), R));
}
FVector FComponentRead::Forward(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component)
{ return FComponentRead::Rotation(Operation,Component).GetForwardVector(); }
FVector FComponentRead::Right(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component)
{ return FComponentRead::Rotation(Operation,Component).GetRightVector(); }
FVector FComponentRead::Up(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component)
{ return FComponentRead::Rotation(Operation,Component).GetUpVector(); }
float FComponentRead::ForwardSpeed(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component)
{ return FVector::DotProduct(FComponentRead::COMVelocity(Operation,Component), FComponentRead::Forward(Operation,Component)); }
bool FComponentRead::GoingForward(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component)
{ return FComponentRead::ForwardSpeed(Operation,Component) > 0; }
Speed::FKinematicState FComponentRead::Origin(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component)
{
	return FComponentRead::OriginForFrame(Operation,Component, FComponentRead::NativeFrame(Operation,Component));
}
Speed::FKinematicState FComponentRead::OriginForFrame(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component, const unsigned int& NumFrameToRead)
{
	const SKinematic& COMState = FComponentRead::ForFrame(Operation,Component,NumFrameToRead);
	SKinematic OriginState = COMState;
	const FOriginKinematicView Origin(COMState, FComponentRead::LocalCOM(Operation,Component));
	OriginState.Location = Origin.Location();
	OriginState.Velocity = Origin.Velocity();
	OriginState.Acceleration = Origin.Acceleration();
	return OriginState;
}

float FComponentRead::Mass(const ISpeedComponent& Component)
{return Component.GetPhysMass();}
float FComponentRead::Mass(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component)
{
    return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const USpeedWheeledComponent::FOriginalWheeledDomain>(Operation,&Component).PhysMass;
}

bool FComponentRead::Frozen(const ISpeedComponent& Component)
{return Component.IsFrozen();}
bool FComponentRead::Frozen(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component)
{
    return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const USpeedBranchWheeledComponent::FOwnedComponentValues>(Operation,&Component).BasePhysicsState.bIsFrozen;
}
float FComponentRead::MaxSpeed(const ISpeedComponent& Component)
{return Component.GetPhysMaxSpeed();}
float FComponentRead::MaxSpeed(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component)
{
    return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const USpeedWheeledComponent::FOriginalWheeledDomain>(Operation,&Component).PhysMaxSpeed;
}
float FComponentRead::MaxAngularSpeed(const ISpeedComponent& Component)
{return Component.GetPhysMaxAngularSpeed();}
float FComponentRead::MaxAngularSpeed(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component)
{
    return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const USpeedWheeledComponent::FOriginalWheeledDomain>(Operation,&Component).PhysMaxAngularSpeed;
}


FMatrix FComponentRead::WorldInvInertia(const ISpeedComponent& Component)
{return Component.ComputeWorldInvInertiaTensor();}
FMatrix FComponentRead::WorldInvInertia(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component)
{return USpeedWheeledComponent::ComputeNativeComponentInvInertia(Operation,Component);}

}
