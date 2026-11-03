#include "ComponentKinematicRead.h"
#include "OriginKinematicView.h"
#include "IAmSpeed/Components/ISpeedComponent.h"

namespace Speed::NativeKinematics
{
const TArray<FPhysicalContactConstraint>& FComponentRead::Constraints(const ISpeedComponent& Component)
{
    return Component.PhysicalConstraintsStorage();
}
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
}
