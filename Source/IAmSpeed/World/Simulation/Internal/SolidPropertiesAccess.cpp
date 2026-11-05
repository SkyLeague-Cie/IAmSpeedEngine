#include "SolidPropertiesAccess.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateNativeStorageMap.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadNativeObjects.h"
#include "IAmSpeed/SubBodies/Solid/SolidSubBody.h"
namespace Speed::NativeKinematics
{
int FSolidPropertiesAccess::Idx(const USolidSubBody& Body)
{ return Body.idxSolidStorage(); }
void FSolidPropertiesAccess::SetIdx(USolidSubBody& Body,int InIdx)
{ Body.idxSolidStorage() = InIdx; }
bool FSolidPropertiesAccess::IsMainSubBody(const USolidSubBody& Body)
{ return Body.bIsMainSubBodySolidStorage(); }
bool FSolidPropertiesAccess::IsFakePhysicsEnabled(const USolidSubBody& Body)
{ return Body.EnableFakePhysicsSolidStorage(); }
float FSolidPropertiesAccess::GetRestitution(const USolidSubBody& Body)
{ return Body.RestitutionSolidStorage(); }
float FSolidPropertiesAccess::GetSphereBoxRestitutionOverride(const USolidSubBody& Body)
{ return Body.SphereBoxRestitutionOverrideSolidStorage(); }
float FSolidPropertiesAccess::GetSphereBoxFrictionOverride(const USolidSubBody& Body)
{ return Body.SphereBoxFrictionOverrideSolidStorage(); }
float FSolidPropertiesAccess::GetSphereBoxManifoldFrictionOverride(const USolidSubBody& Body)
{ return Body.SphereBoxManifoldFrictionOverrideSolidStorage(); }
bool FSolidPropertiesAccess::UsesCoupledContactImpulse(const USolidSubBody& Body)
{ return Body.bUseCoupledContactImpulseSolidStorage(); }
bool FSolidPropertiesAccess::UsesPostNormalFrictionImpulse(const USolidSubBody& Body)
{ return Body.bUsePostNormalFrictionImpulseSolidStorage(); }
bool FSolidPropertiesAccess::UsesPersistentBilateralContact(const USolidSubBody& Body)
{ return Body.bUsePersistentBilateralContactSolidStorage(); }
bool FSolidPropertiesAccess::AllowsCoupledContactImpulseFor(const USolidSubBody& Body,const SKinematic& State)
{
		return (Body.CoupledContactMaxLinearSpeedSolidStorage() < 0.0f ||
			State.Velocity.Size() <= Body.CoupledContactMaxLinearSpeedSolidStorage()) &&
			(Body.CoupledContactMaxAngularSpeedSolidStorage() < 0.0f ||
			State.AngularVelocity.Size() <= Body.CoupledContactMaxAngularSpeedSolidStorage());
	}
bool FSolidPropertiesAccess::AllowsCoupledContactNormal(const USolidSubBody& Body,const FVector& LocalNormal)
{
		return Body.CoupledContactMaxAbsLocalNormalYSolidStorage() < 0.0f ||
			FMath::Abs(LocalNormal.Y) <= Body.CoupledContactMaxAbsLocalNormalYSolidStorage();
	}
float FSolidPropertiesAccess::GetStaticFriction(const USolidSubBody& Body)
{ return Body.StaticFrictionSolidStorage(); }
float FSolidPropertiesAccess::GetDynamicFriction(const USolidSubBody& Body)
{ return Body.DynamicFrictionSolidStorage(); }
float FSolidPropertiesAccess::GetImpactThreshold(const USolidSubBody& Body)
{ return Body.ImpactThresholdSolidStorage(); }
float FSolidPropertiesAccess::GetHitDamping(const USolidSubBody& Body)
{ return Body.HitDampingSolidStorage(); }
void FSolidPropertiesAccess::SetSphereBoxFrictionOverride(USolidSubBody& Body,float InFriction)
{
		Body.SphereBoxFrictionOverrideSolidStorage() = InFriction < 0.0f ? -1.0f : InFriction;
	}
void FSolidPropertiesAccess::SetSphereBoxManifoldFrictionOverride(USolidSubBody& Body,float InFriction)
{
		Body.SphereBoxManifoldFrictionOverrideSolidStorage() = InFriction < 0.0f ? -1.0f : InFriction;
	}
void FSolidPropertiesAccess::SetUseCoupledContactImpulse(USolidSubBody& Body,bool bInEnabled)
{
		Body.bUseCoupledContactImpulseSolidStorage() = bInEnabled;
	}
void FSolidPropertiesAccess::SetUsePostNormalFrictionImpulse(USolidSubBody& Body,bool bInEnabled)
{
		Body.bUsePostNormalFrictionImpulseSolidStorage() = bInEnabled;
	}
void FSolidPropertiesAccess::SetUsePersistentBilateralContact(USolidSubBody& Body,bool bInEnabled)
{
		Body.bUsePersistentBilateralContactSolidStorage() = bInEnabled;
	}
void FSolidPropertiesAccess::SetCoupledContactKinematicLimits(USolidSubBody& Body,float MaxLinearSpeed,
		float MaxAngularSpeed)
{
		Body.CoupledContactMaxLinearSpeedSolidStorage() = MaxLinearSpeed;
		Body.CoupledContactMaxAngularSpeedSolidStorage() = MaxAngularSpeed;
	}
void FSolidPropertiesAccess::SetCoupledContactMaxAbsLocalNormalY(USolidSubBody& Body,float MaxAbsLocalNormalY)
{
		Body.CoupledContactMaxAbsLocalNormalYSolidStorage() = MaxAbsLocalNormalY;
	}
bool FSolidPropertiesAccess::HasToApplyRestForce(const USolidSubBody& Body)
{ return Body.bApplyRestForceSolidStorage(); }
void FSolidPropertiesAccess::SetSphereBoxRestitutionOverride(USolidSubBody& Body,float InRestitution)
{
    Body.SphereBoxRestitutionOverrideSolidStorage() = InRestitution < 0.f ? -1.f : FMath::Clamp(InRestitution, 0.f, 1.f);
}

// Actual owner operation overloads. Legacy/default/custom entries above remain unchanged.
int FSolidPropertiesAccess::Idx(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body)
{ return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).idx; }
void FSolidPropertiesAccess::SetIdx(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody& Body,int InIdx)
{ Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).idx = InIdx; }
bool FSolidPropertiesAccess::IsMainSubBody(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body)
{ return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).bIsMainSubBody; }
bool FSolidPropertiesAccess::IsFakePhysicsEnabled(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body)
{ return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).EnableFakePhysics; }
float FSolidPropertiesAccess::GetRestitution(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body)
{ return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).Restitution; }
float FSolidPropertiesAccess::GetSphereBoxRestitutionOverride(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body)
{ return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).SphereBoxRestitutionOverride; }
float FSolidPropertiesAccess::GetSphereBoxFrictionOverride(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body)
{ return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).SphereBoxFrictionOverride; }
float FSolidPropertiesAccess::GetSphereBoxManifoldFrictionOverride(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body)
{ return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).SphereBoxManifoldFrictionOverride; }
bool FSolidPropertiesAccess::UsesCoupledContactImpulse(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body)
{ return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).bUseCoupledContactImpulse; }
bool FSolidPropertiesAccess::UsesPostNormalFrictionImpulse(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body)
{ return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).bUsePostNormalFrictionImpulse; }
bool FSolidPropertiesAccess::UsesPersistentBilateralContact(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body)
{ return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).bUsePersistentBilateralContact; }
bool FSolidPropertiesAccess::AllowsCoupledContactImpulseFor(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body,const SKinematic& State)
{
		return (Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).CoupledContactMaxLinearSpeed < 0.0f ||
			State.Velocity.Size() <= Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).CoupledContactMaxLinearSpeed) &&
			(Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).CoupledContactMaxAngularSpeed < 0.0f ||
			State.AngularVelocity.Size() <= Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).CoupledContactMaxAngularSpeed);
	}
bool FSolidPropertiesAccess::AllowsCoupledContactNormal(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body,const FVector& LocalNormal)
{
		return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).CoupledContactMaxAbsLocalNormalY < 0.0f ||
			FMath::Abs(LocalNormal.Y) <= Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).CoupledContactMaxAbsLocalNormalY;
	}
float FSolidPropertiesAccess::GetStaticFriction(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body)
{ return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).StaticFriction; }
float FSolidPropertiesAccess::GetDynamicFriction(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body)
{ return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).DynamicFriction; }
float FSolidPropertiesAccess::GetImpactThreshold(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body)
{ return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).ImpactThreshold; }
float FSolidPropertiesAccess::GetHitDamping(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body)
{ return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).HitDamping; }
void FSolidPropertiesAccess::SetSphereBoxFrictionOverride(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody& Body,float InFriction)
{
		Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).SphereBoxFrictionOverride = InFriction < 0.0f ? -1.0f : InFriction;
	}
void FSolidPropertiesAccess::SetSphereBoxManifoldFrictionOverride(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody& Body,float InFriction)
{
		Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).SphereBoxManifoldFrictionOverride = InFriction < 0.0f ? -1.0f : InFriction;
	}
void FSolidPropertiesAccess::SetUseCoupledContactImpulse(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody& Body,bool bInEnabled)
{
		Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).bUseCoupledContactImpulse = bInEnabled;
	}
void FSolidPropertiesAccess::SetUsePostNormalFrictionImpulse(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody& Body,bool bInEnabled)
{
		Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).bUsePostNormalFrictionImpulse = bInEnabled;
	}
void FSolidPropertiesAccess::SetUsePersistentBilateralContact(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody& Body,bool bInEnabled)
{
		Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).bUsePersistentBilateralContact = bInEnabled;
	}
void FSolidPropertiesAccess::SetCoupledContactKinematicLimits(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody& Body,float MaxLinearSpeed,
		float MaxAngularSpeed)
{
		Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).CoupledContactMaxLinearSpeed = MaxLinearSpeed;
		Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).CoupledContactMaxAngularSpeed = MaxAngularSpeed;
	}
void FSolidPropertiesAccess::SetCoupledContactMaxAbsLocalNormalY(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody& Body,float MaxAbsLocalNormalY)
{
		Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).CoupledContactMaxAbsLocalNormalY = MaxAbsLocalNormalY;
	}
bool FSolidPropertiesAccess::HasToApplyRestForce(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body)
{ return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).bApplyRestForce; }
void FSolidPropertiesAccess::SetSphereBoxRestitutionOverride(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody& Body,float InRestitution)
{
    Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).SphereBoxRestitutionOverride = InRestitution < 0.f ? -1.f : FMath::Clamp(InRestitution, 0.f, 1.f);
}

}
