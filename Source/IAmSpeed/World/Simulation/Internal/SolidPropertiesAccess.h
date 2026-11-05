#pragma once
// Private native solid values only. Access control is not a source/restore lease.
#include "IAmSpeed/Base/Kinematic.h"
class USolidSubBody;class UBoxSubBody;class USphereSubBody;class USWheelSubBody;
class ISpeedWheeledComponent;class USpeedWheeledComponent;class USpeedMovementComponent;class USpeedWorldSubsystem;
namespace Speed::Netcode::Prediction { class FPrivateRoadPhysicsWorld; class FPrivateNativeOperation; }
namespace Speed::NativeKinematics
{
class FSolidRestitutionPolicy final
{
    friend class ::USolidSubBody;
    static float Resolve(const USolidSubBody& Sphere,const USolidSubBody& Box,const float FallbackRestitution);
};
class FSolidPropertiesAccess final
{
    friend class ::USolidSubBody;friend class ::UBoxSubBody;friend class ::USphereSubBody;friend class ::USWheelSubBody;
    friend class ::ISpeedWheeledComponent;friend class ::USpeedWheeledComponent;friend class ::USpeedMovementComponent;friend class ::USpeedWorldSubsystem;
    friend class FSolidRestitutionPolicy;
    friend class Speed::Netcode::Prediction::FPrivateRoadPhysicsWorld;
    static int Idx(const USolidSubBody& Body);
    static int Idx(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body);
    static void SetIdx(USolidSubBody& Body,int InIdx);
    static void SetIdx(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody& Body,int InIdx);
    static bool IsMainSubBody(const USolidSubBody& Body);
    static bool IsMainSubBody(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body);
    static bool IsFakePhysicsEnabled(const USolidSubBody& Body);
    static bool IsFakePhysicsEnabled(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body);
    static float GetRestitution(const USolidSubBody& Body);
    static float GetRestitution(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body);
    static float GetSphereBoxRestitutionOverride(const USolidSubBody& Body);
    static float GetSphereBoxRestitutionOverride(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body);
    static float GetSphereBoxFrictionOverride(const USolidSubBody& Body);
    static float GetSphereBoxFrictionOverride(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body);
    static float GetSphereBoxManifoldFrictionOverride(const USolidSubBody& Body);
    static float GetSphereBoxManifoldFrictionOverride(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body);
    static bool UsesCoupledContactImpulse(const USolidSubBody& Body);
    static bool UsesCoupledContactImpulse(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body);
    static bool UsesPostNormalFrictionImpulse(const USolidSubBody& Body);
    static bool UsesPostNormalFrictionImpulse(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body);
    static bool UsesPersistentBilateralContact(const USolidSubBody& Body);
    static bool UsesPersistentBilateralContact(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body);
    static bool AllowsCoupledContactImpulseFor(const USolidSubBody& Body,const SKinematic& State);
    static bool AllowsCoupledContactImpulseFor(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body,const SKinematic& State);
    static bool AllowsCoupledContactNormal(const USolidSubBody& Body,const FVector& LocalNormal);
    static bool AllowsCoupledContactNormal(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body,const FVector& LocalNormal);
    static float GetStaticFriction(const USolidSubBody& Body);
    static float GetStaticFriction(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body);
    static float GetDynamicFriction(const USolidSubBody& Body);
    static float GetDynamicFriction(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body);
    static float GetImpactThreshold(const USolidSubBody& Body);
    static float GetImpactThreshold(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body);
    static float GetHitDamping(const USolidSubBody& Body);
    static float GetHitDamping(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body);
    static void SetSphereBoxFrictionOverride(USolidSubBody& Body,float InFriction);
    static void SetSphereBoxFrictionOverride(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody& Body,float InFriction);
    static void SetSphereBoxManifoldFrictionOverride(USolidSubBody& Body,float InFriction);
    static void SetSphereBoxManifoldFrictionOverride(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody& Body,float InFriction);
    static void SetUseCoupledContactImpulse(USolidSubBody& Body,bool bInEnabled);
    static void SetUseCoupledContactImpulse(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody& Body,bool bInEnabled);
    static void SetUsePostNormalFrictionImpulse(USolidSubBody& Body,bool bInEnabled);
    static void SetUsePostNormalFrictionImpulse(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody& Body,bool bInEnabled);
    static void SetUsePersistentBilateralContact(USolidSubBody& Body,bool bInEnabled);
    static void SetUsePersistentBilateralContact(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody& Body,bool bInEnabled);
    static void SetCoupledContactKinematicLimits(USolidSubBody& Body,float MaxLinearSpeed,
		float MaxAngularSpeed);
    static void SetCoupledContactKinematicLimits(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody& Body,float MaxLinearSpeed,
		float MaxAngularSpeed);
    static void SetCoupledContactMaxAbsLocalNormalY(USolidSubBody& Body,float MaxAbsLocalNormalY);
    static void SetCoupledContactMaxAbsLocalNormalY(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody& Body,float MaxAbsLocalNormalY);
    static bool HasToApplyRestForce(const USolidSubBody& Body);
    static bool HasToApplyRestForce(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body);
    static void SetSphereBoxRestitutionOverride(USolidSubBody& Body,float InRestitution);
    static void SetSphereBoxRestitutionOverride(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody& Body,float InRestitution);
};
}
