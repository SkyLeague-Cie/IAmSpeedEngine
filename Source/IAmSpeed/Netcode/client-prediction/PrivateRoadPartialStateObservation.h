#pragma once
#include "PrivateRoadPhysicsWorld.h"
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "IAmSpeed/Base/EpisodeShapeFieldsForTesting.h"
#include "IAmSpeed/Netcode/Common/NativeCommonCacheAccess.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "SimpleVehicle.h"
#include "WheelSystem.h"
#include "SuspensionSystem.h"
#include <cmath>
#include <type_traits>
namespace Speed::Netcode::Prediction::PartialObservation
{
// Field enumeration is deliberately PARTIAL: no subbody hit/support caches,
// complete component/configuration/world registry or lifetime certificate.
struct FCheckedWriter
{
    static inline thread_local bool* Finite=nullptr;
    template<class T>static bool IsFiniteValue(const T& Value)
    {
        if constexpr(std::is_floating_point_v<T>)return std::isfinite(Value);
        else if constexpr(std::is_array_v<T>){for(const auto& Item:Value)if(!IsFiniteValue(Item))return false;return true;}
        else if constexpr(requires{Value.ContainsNaN();})return !Value.ContainsNaN();
        else return true;
    }
    template<class T>static void Add(TArray<uint8>& Out,const T& Value)
    {
        if(Finite&&!IsFiniteValue(Value))*Finite=false;
        FEpisodeStateBytesForTesting::Add(Out,Value);
    }
};
inline void AppendCheckedWheel(TArray<uint8>& Out,const Chaos::FSimpleWheelSim& Wheel)
{
 FCheckedWriter::Add(Out,Wheel.SetupPtr);
  FCheckedWriter::Add(Out,Wheel.BrakeEnabled);
  FCheckedWriter::Add(Out,Wheel.HandbrakeEnabled);
  FCheckedWriter::Add(Out,Wheel.SteeringEnabled);
  FCheckedWriter::Add(Out,Wheel.EngineEnabled);
  FCheckedWriter::Add(Out,Wheel.TractionControlEnabled);
  FCheckedWriter::Add(Out,Wheel.ABSEnabled);
  FCheckedWriter::Add(Out,Wheel.FrictionMultiplier);
  FCheckedWriter::Add(Out,Wheel.LateralSlipGraphMultiplier);
  FCheckedWriter::Add(Out,Wheel.CorneringStiffness);
  FCheckedWriter::Add(Out,Wheel.MaxSteeringAngle);
  FCheckedWriter::Add(Out,Wheel.MaxBrakeTorque);
  FCheckedWriter::Add(Out,Wheel.HandbrakeTorque);
  FCheckedWriter::Add(Out,Wheel.ExternalTorqueCombineMethod);
  FCheckedWriter::Add(Out,Wheel.Re);
  FCheckedWriter::Add(Out,Wheel.Omega);
  FCheckedWriter::Add(Out,Wheel.Sx);
  FCheckedWriter::Add(Out,Wheel.Inertia);
  FCheckedWriter::Add(Out,Wheel.DriveTorque);
  FCheckedWriter::Add(Out,Wheel.BrakeTorque);
  FCheckedWriter::Add(Out,Wheel.ForceIntoSurface);
  FCheckedWriter::Add(Out,Wheel.GroundVelocityVector);
  FCheckedWriter::Add(Out,Wheel.AngularPosition);
  FCheckedWriter::Add(Out,Wheel.SteeringAngle);
  FCheckedWriter::Add(Out,Wheel.SurfaceFriction);
  FCheckedWriter::Add(Out,Wheel.MaxOmega);
  FCheckedWriter::Add(Out,Wheel.ExternalDriveTorque);
  FCheckedWriter::Add(Out,Wheel.ExternalBrakeTorque);
  FCheckedWriter::Add(Out,Wheel.ForceFromFriction);
  FCheckedWriter::Add(Out,Wheel.MassPerWheel);
  FCheckedWriter::Add(Out,Wheel.SlipVelocity);
  FCheckedWriter::Add(Out,Wheel.SlipAngle);
  FCheckedWriter::Add(Out,Wheel.bInContact);
  FCheckedWriter::Add(Out,Wheel.WheelIndex);
  FCheckedWriter::Add(Out,Wheel.bEngineBraking);
  FCheckedWriter::Add(Out,Wheel.AppliedLinearDriveForce);
  FCheckedWriter::Add(Out,Wheel.AppliedLinearBrakeForce);
  FCheckedWriter::Add(Out,Wheel.LongitudinalAdhesiveLimit);
  FCheckedWriter::Add(Out,Wheel.LateralAdhesiveLimit);
  FCheckedWriter::Add(Out,Wheel.SideSlipModifier);
  FCheckedWriter::Add(Out,Wheel.Spin);
  FCheckedWriter::Add(Out,Wheel.AvailableGrip);
  FCheckedWriter::Add(Out,Wheel.InputForces);
  FCheckedWriter::Add(Out,Wheel.bClipping);
  FCheckedWriter::Add(Out,Wheel.bABSActivated);
}
class FCheckedSuspensionCopy final:public Chaos::FSimpleSuspensionSim
{
public:
 explicit FCheckedSuspensionCopy(const Chaos::FSimpleSuspensionSim& Source)
  :Chaos::FSimpleSuspensionSim(Source){}
 void Append(TArray<uint8>& Out)const
 {
  FCheckedWriter::Add(Out,SetupPtr);
  FCheckedWriter::Add(Out,DisplacementInput);FCheckedWriter::Add(Out,LastDisplacement);FCheckedWriter::Add(Out,LocalVelocity);
  FCheckedWriter::Add(Out,SuspensionForce);FCheckedWriter::Add(Out,LocalOffset);FCheckedWriter::Add(Out,SpringDisplacement);FCheckedWriter::Add(Out,SpringIndex);
  FCheckedWriter::Add(Out,LastSpringLength);FCheckedWriter::Add(Out,AveragingLength);FCheckedWriter::Add(Out,AveragingCount);FCheckedWriter::Add(Out,AveragingNum);
 }
};
inline bool AppendCheckedWheelConfig(TArray<uint8>& O,const Chaos::FSimpleWheelConfig& C)
{
 
 FCheckedWriter::Add(O,C.Offset);FCheckedWriter::Add(O,C.WheelMass);FCheckedWriter::Add(O,C.WheelRadius);FCheckedWriter::Add(O,C.WheelWidth);
 FCheckedWriter::Add(O,C.MaxSteeringAngle);FCheckedWriter::Add(O,C.MaxBrakeTorque);FCheckedWriter::Add(O,C.HandbrakeTorque);FCheckedWriter::Add(O,C.ABSEnabled);
 FCheckedWriter::Add(O,C.BrakeEnabled);FCheckedWriter::Add(O,C.HandbrakeEnabled);FCheckedWriter::Add(O,C.SteeringEnabled);FCheckedWriter::Add(O,C.EngineEnabled);FCheckedWriter::Add(O,C.TractionControlEnabled);
 FCheckedWriter::Add(O,C.TorqueRatio);FCheckedWriter::Add(O,C.AxleType);FCheckedWriter::Add(O,C.FrictionCombineMethod);FCheckedWriter::Add(O,C.ExternalTorqueCombineMethod);
 FCheckedWriter::Add(O,C.FrictionMultiplier);FCheckedWriter::Add(O,C.LateralSlipGraphMultiplier);FCheckedWriter::Add(O,C.CorneringStiffness);FCheckedWriter::Add(O,C.SideSlipModifier);
 FCheckedWriter::Add(O,C.SlipThreshold);FCheckedWriter::Add(O,C.SkidThreshold);FCheckedWriter::Add(O,C.MaxSpinRotation);
 const bool Empty=C.LateralSlipGraph.IsEmpty();FCheckedWriter::Add(O,Empty);return Empty;
}
inline void AppendCheckedSuspensionConfig(TArray<uint8>& O,const Chaos::FSimpleSuspensionConfig& C)
{
 
 FCheckedWriter::Add(O,C.SuspensionAxis);FCheckedWriter::Add(O,C.SuspensionForceOffset);FCheckedWriter::Add(O,C.SuspensionMaxRaise);FCheckedWriter::Add(O,C.SuspensionMaxDrop);FCheckedWriter::Add(O,C.MaxLength);
 FCheckedWriter::Add(O,C.SpringRate);FCheckedWriter::Add(O,C.SpringPreload);FCheckedWriter::Add(O,C.CompressionDamping);FCheckedWriter::Add(O,C.ReboundDamping);FCheckedWriter::Add(O,C.RestingForce);
 FCheckedWriter::Add(O,C.Swaybar);FCheckedWriter::Add(O,C.DampingRatio);FCheckedWriter::Add(O,C.WheelLoadRatio);FCheckedWriter::Add(O,C.RaycastSafetyMargin);FCheckedWriter::Add(O,C.SuspensionSmoothing);
}
// Read only the original owned wheel values at the actual native boundary.
// No pointer/configuration/lifetime completeness follows from these values.
template<class Runtime>bool AppendWheelRuntime(TArray<uint8>& Out,const Runtime& V)
{
    bool Finite=true;bool* Previous=FCheckedWriter::Finite;FCheckedWriter::Finite=&Finite;
    struct FReset{bool* Previous;~FReset(){FCheckedWriter::Finite=Previous;}} Reset{Previous};
    FCheckedWriter::Add(Out,V.SuspensionForce);
    FCheckedWriter::Add(Out,V.bIsJumping);
    FCheckedWriter::Add(Out,V.bJumpUnilateralSupport);
    FCheckedWriter::Add(Out,V.ConsecutiveGroundFrames);
    FCheckedWriter::Add(Out,V.SteeringAngle);
    FCheckedWriter::Add(Out,V.Omega);
    FCheckedWriter::Add(Out,V.RollAngle);
    FCheckedWriter::Add(Out,V.bWasOnGroundPrevFrame);
    FCheckedWriter::Add(Out,V.bCountedGroundFrame);
    FCheckedWriter::Add(Out,V.bContactVelocityLocked);
    FCheckedWriter::Add(Out,V.bCrossedIntoCompressionThisFrame);
    FCheckedWriter::Add(Out,V.ForwardAxis);
    FCheckedWriter::Add(Out,V.RightAxis);
    FCheckedWriter::Add(Out,V.UpAxis);
    FCheckedWriter::Add(Out,V.RenderData.RollRotation);
    FCheckedWriter::Add(Out,V.RenderData.SteerRotation);
    FCheckedWriter::Add(Out,V.RenderData.WorldPosition);
    FCheckedWriter::Add(Out,V.RenderData.RightAxis);
    FCheckedWriter::Add(Out,V.RenderData.UpAxis);
    FCheckedWriter::Add(Out,V.RenderData.ForwardAxis);
    FCheckedWriter::Add(Out,V.RenderData.SpringOffset);
    return Finite;
}
template<class Solid>bool AppendSolidValues(TArray<uint8>& Out,const Solid& V)
{
    bool Finite=true;bool* Previous=FCheckedWriter::Finite;FCheckedWriter::Finite=&Finite;
    struct FReset{bool* Previous;~FReset(){FCheckedWriter::Finite=Previous;}} Reset{Previous};
    FCheckedWriter::Add(Out,V.Mass);
    FCheckedWriter::Add(Out,V.bIsMainSubBody);
    FCheckedWriter::Add(Out,V.bApplyRestForce);
    FCheckedWriter::Add(Out,V.EnableFakePhysics);
    FCheckedWriter::Add(Out,V.bEnableConstraintOnSubBodyHit);
    FCheckedWriter::Add(Out,V.Restitution);
    FCheckedWriter::Add(Out,V.SphereBoxRestitutionOverride);
    FCheckedWriter::Add(Out,V.SphereBoxFrictionOverride);
    FCheckedWriter::Add(Out,V.SphereBoxManifoldFrictionOverride);
    FCheckedWriter::Add(Out,V.bUseCoupledContactImpulse);
    FCheckedWriter::Add(Out,V.bUsePostNormalFrictionImpulse);
    FCheckedWriter::Add(Out,V.bUsePersistentBilateralContact);
    FCheckedWriter::Add(Out,V.CoupledContactMaxLinearSpeed);
    FCheckedWriter::Add(Out,V.CoupledContactMaxAngularSpeed);
    FCheckedWriter::Add(Out,V.CoupledContactMaxAbsLocalNormalY);
    FCheckedWriter::Add(Out,V.StaticFriction);
    FCheckedWriter::Add(Out,V.DynamicFriction);
    FCheckedWriter::Add(Out,V.ImpactThreshold);
    FCheckedWriter::Add(Out,V.HitDamping);
    FCheckedWriter::Add(Out,V.InvInertiaLocal);
    FCheckedWriter::Add(Out,V.idx);
    return Finite;
}
template<class Component>bool Capture(Component& C,uint64 Actor,uint64 Incarnation,
    const Chaos::FSimpleWheelConfig& WheelConfig,const Chaos::FSimpleSuspensionConfig& SuspensionConfig,
    FPrivateRoadActorFrame& Out)
{
    Out.NativePartialValuesAvailable=false;
    if(!IsInGameThread()||!Actor||!Incarnation||C.GetWheelSubBodies().Num()!=4)return false;
    const auto Sleep=Out.NativeSleep;
    const auto Quantization=Out.NativeQuantization;
    if(!CommonState::ValidCaches(Sleep,Quantization))return false;
    FFramePointerRegistry Registry;
    FFramePointerKey Key;Key.Domain=72;Key.Owner=Actor;Key.Generation=Incarnation;Key.Role=1;Key.Membership={0,1,2,3};
    if(!Registry.Register(&WheelConfig,Key))return false;
    Key.Role=2;if(!Registry.Register(&SuspensionConfig,Key)||!Registry.Seal())return false;
    bool Finite=true;bool* Previous=FCheckedWriter::Finite;FCheckedWriter::Finite=&Finite;
    struct FReset{bool* Previous;~FReset(){FCheckedWriter::Finite=Previous;}} Reset{Previous};
    TArray<uint8> Raw,Canonical;FFramePointerCaptureScope Pointers(Raw,Registry);
    FCheckedWriter::Add(Raw,uint32(1));
    if(!AppendCheckedWheelConfig(Raw,WheelConfig))return false;
    AppendCheckedSuspensionConfig(Raw,SuspensionConfig);
    for(const auto& Entry:C.GetWheelSubBodies())
    {
        const auto* Body=Entry.Get();
        if(!Body||!Body->GetWheelSim()||!Body->GetSuspensionSim())return false;
        AppendCheckedWheel(Raw,*Body->GetWheelSim());
        const FCheckedSuspensionCopy Copy(*Body->GetSuspensionSim());Copy.Append(Raw);
    }
    if(!Finite||!Pointers.MakeCanonical(Canonical)||Canonical.IsEmpty())return false;
    Out.NativeSleep=Sleep;Out.NativeQuantization=Quantization;
    Out.NativeWheelSuspensionValues.assign(Canonical.GetData(),Canonical.GetData()+Canonical.Num());
    Out.NativePartialValuesAvailable=true;return true;
}
}
#endif
