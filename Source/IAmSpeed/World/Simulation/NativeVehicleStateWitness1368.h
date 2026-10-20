#pragma once
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "IAmSpeed/Base/EpisodeShapeFieldsForTesting.h"
#include "IAmSpeed/Components/ISpeedWheeledComponent.h"
#include "IAmSpeed/Components/Netcode/SpeedWheeledPhysicsComponent.h"
#include "SimpleVehicle.h"
#include <type_traits>

namespace Speed::Testing1368
{
using W=FEpisodeStateBytesForTesting;

// This is a REAL derived object with a copied base subobject. It never casts a
// live base object to a derived type and never reads object representation/ABI.
// The pinned base contains values and a non-owning const config pointer only.
class FSuspensionValueCopy final:public Chaos::FSimpleSuspensionSim
{
public:
 explicit FSuspensionValueCopy(const Chaos::FSimpleSuspensionSim& Source)
  :Chaos::FSimpleSuspensionSim(Source){}
 void Append(TArray<uint8>& Out)const
 {
  W::Add(Out,SetupPtr);
  W::Add(Out,DisplacementInput);W::Add(Out,LastDisplacement);W::Add(Out,LocalVelocity);
  W::Add(Out,SuspensionForce);W::Add(Out,LocalOffset);W::Add(Out,SpringDisplacement);W::Add(Out,SpringIndex);
  W::Add(Out,LastSpringLength);W::Add(Out,AveragingLength);W::Add(Out,AveragingCount);W::Add(Out,AveragingNum);
 }
};
static_assert(std::is_copy_constructible_v<Chaos::FSimpleSuspensionSim>);
static_assert(std::is_trivially_destructible_v<Chaos::FSimpleSuspensionSim>);
static_assert(std::is_trivially_destructible_v<FSuspensionValueCopy>);

inline void AppendWheel(TArray<uint8>& Out,const Chaos::FSimpleWheelSim& Wheel)
{
 W::Add(Out,Wheel.SetupPtr);
  W::Add(Out,Wheel.BrakeEnabled);
  W::Add(Out,Wheel.HandbrakeEnabled);
  W::Add(Out,Wheel.SteeringEnabled);
  W::Add(Out,Wheel.EngineEnabled);
  W::Add(Out,Wheel.TractionControlEnabled);
  W::Add(Out,Wheel.ABSEnabled);
  W::Add(Out,Wheel.FrictionMultiplier);
  W::Add(Out,Wheel.LateralSlipGraphMultiplier);
  W::Add(Out,Wheel.CorneringStiffness);
  W::Add(Out,Wheel.MaxSteeringAngle);
  W::Add(Out,Wheel.MaxBrakeTorque);
  W::Add(Out,Wheel.HandbrakeTorque);
  W::Add(Out,Wheel.ExternalTorqueCombineMethod);
  W::Add(Out,Wheel.Re);
  W::Add(Out,Wheel.Omega);
  W::Add(Out,Wheel.Sx);
  W::Add(Out,Wheel.Inertia);
  W::Add(Out,Wheel.DriveTorque);
  W::Add(Out,Wheel.BrakeTorque);
  W::Add(Out,Wheel.ForceIntoSurface);
  W::Add(Out,Wheel.GroundVelocityVector);
  W::Add(Out,Wheel.AngularPosition);
  W::Add(Out,Wheel.SteeringAngle);
  W::Add(Out,Wheel.SurfaceFriction);
  W::Add(Out,Wheel.MaxOmega);
  W::Add(Out,Wheel.ExternalDriveTorque);
  W::Add(Out,Wheel.ExternalBrakeTorque);
  W::Add(Out,Wheel.ForceFromFriction);
  W::Add(Out,Wheel.MassPerWheel);
  W::Add(Out,Wheel.SlipVelocity);
  W::Add(Out,Wheel.SlipAngle);
  W::Add(Out,Wheel.bInContact);
  W::Add(Out,Wheel.WheelIndex);
  W::Add(Out,Wheel.bEngineBraking);
  W::Add(Out,Wheel.AppliedLinearDriveForce);
  W::Add(Out,Wheel.AppliedLinearBrakeForce);
  W::Add(Out,Wheel.LongitudinalAdhesiveLimit);
  W::Add(Out,Wheel.LateralAdhesiveLimit);
  W::Add(Out,Wheel.SideSlipModifier);
  W::Add(Out,Wheel.Spin);
  W::Add(Out,Wheel.AvailableGrip);
  W::Add(Out,Wheel.InputForces);
  W::Add(Out,Wheel.bClipping);
  W::Add(Out,Wheel.bABSActivated);
}

inline bool AppendVehicle(TArray<uint8>& Out,const Chaos::FSimpleWheeledVehicle& Vehicle)
{
 // Explicit fixture scope: no silently omitted live engine/gear/other subsystem.
 if(!Vehicle.Engine.IsEmpty()||!Vehicle.Transmission.IsEmpty()||!Vehicle.Differential.IsEmpty()
  ||!Vehicle.Steering.IsEmpty()||!Vehicle.Aerodynamics.IsEmpty()||!Vehicle.Aerofoils.IsEmpty()
  ||!Vehicle.Thrusters.IsEmpty()||!Vehicle.Axles.IsEmpty()||!Vehicle.TorqueControlSim.IsEmpty()
  ||!Vehicle.TargetRotationControlSim.IsEmpty()||!Vehicle.StabilizeControlSim.IsEmpty()
  ||Vehicle.Wheels.Num()!=4||Vehicle.Suspension.Num()!=4)return false;
 W::Add(Out,Vehicle.bSuspensionEnabled);W::Add(Out,Vehicle.bMechanicalSimEnabled);
 W::Add(Out,Vehicle.bWheelFrictionEnabled);W::Add(Out,Vehicle.NumDrivenWheels);W::Add(Out,Vehicle.bLegacyWheelFrictionPosition);
 W::Add(Out,Vehicle.Wheels.Num());for(const auto& Wheel:Vehicle.Wheels)AppendWheel(Out,Wheel);
 W::Add(Out,Vehicle.Suspension.Num());
 for(const auto& Suspension:Vehicle.Suspension){const FSuspensionValueCopy Copy(Suspension);Copy.Append(Out);}
 return true;
}

inline void AppendWheelContact(TArray<uint8>& Out,const SWheelGroundContact& Contact)
{
  W::Add(Out,Contact.Wheel);
  W::Add(Out,Contact.SurfaceComponent);
  W::Add(Out,Contact.SurfacePoint);
  W::Add(Out,Contact.SurfaceFaceIndex);
  W::Add(Out,Contact.SurfaceSourceId);
  W::Add(Out,Contact.SurfaceId);
  W::Add(Out,Contact.SurfaceFeatureId);
  W::Add(Out,Contact.WorldPos);
  W::Add(Out,Contact.Normal);
  W::Add(Out,Contact.r);
  W::Add(Out,Contact.vN);
  W::Add(Out,Contact.InvMassEff);
  W::Add(Out,Contact.SpringDisplacement);
  W::Add(Out,Contact.bVelocityLocked);
  W::Add(Out,Contact.bNewContact);
  W::Add(Out,Contact.bAtBumpStop);
}
inline void AppendCameraInput(TArray<uint8>& Out,const FSpeedCarCameraPhysicalInput& Input)
{W::Add(Out,Input.Version);W::Add(Out,Input.Flags);W::Add(Out,Input.Yaw);W::Add(Out,Input.Pitch);}
inline void AppendInput(TArray<uint8>& Out,const FWheeledInputState& Input)
{W::Add(Out,Input.bCanMove);W::Add(Out,Input.Throttle);W::Add(Out,Input.Brake);W::Add(Out,Input.Steer);AppendCameraInput(Out,Input.Camera);}
}
#endif
