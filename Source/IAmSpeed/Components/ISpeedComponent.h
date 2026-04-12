#pragma once

#include "CoreMinimal.h"
#include "IAmSpeed/Base/SHitResult.h"
#include "IAmSpeed/Base/Kinematic.h"

class USSubBody;
class USolidSubBody;
struct SubBodyConfig;

struct SComponentTOI
{
	bool bHit = false;
	float TOI = 0.f;
	TWeakObjectPtr<USSubBody> Resolver;     // SubBody that will resolve the hit at TOI
	SHitResult Hit;                         // Chosen hit result at TOI
	uint64 PairKey = 0;                     // Optional: anti-double resolution
};

/**
 * ISpeedComponent : Interface for components that own USSubBodies. It allows sub-bodies to query info about their parent component
 * Add this interface to any component that owns USSubBodies
 * (e.g. a car body component that owns wheel sub-bodies or hitbox) and implement
 * the GetRadiusFromSubBody function to return the appropriate radius for each sub-body (e.g. wheel radius)
 */
class IAMSPEED_API ISpeedComponent
{
public:
	// overload this function to return the number of frames that have passed since the start of simulation
	virtual unsigned int NumFrame() const = 0;
	// overload this function to return the mass of the component (e.g. for a car body, this would be the mass of the car body without the wheels)
	virtual float GetPhysMass() const = 0;
	// overload this function to return the center of mass of the component in world space (e.g. for a car body, this would be the center of mass of the car body without the wheels)
	virtual FVector GetPhysCOM() const = 0;
	// overload this function to return the array of sub-bodies owned by this component
	virtual const TArray<USSubBody*>& GetSubBodies() const = 0;
	// overload this function to create the sub-bodies owned by this component (e.g. for a car body, this would be creating the wheel sub-bodies and the hitbox sub-body)
	virtual TArray<USSubBody*> CreateSubBodies() = 0;

	// ==================== SubBody-specific info retrieval functions ====================
	virtual SubBodyConfig GetSubBodyConfig(const USSubBody& SubBody) const = 0;
	//===================================================================================

	// overload this function to return the appropriate kinematic state for SubBody (e.g. wheel kinematics or hitbox kinematics from car body)
	virtual const SKinematic& GetKinematicsOfSubBody(const USSubBody& SubBody, const unsigned int& NumFrame) const = 0;
	// overload this function to return the appropriate inertia tensor in world space
	virtual FMatrix ComputeWorldInvInertiaTensor() const = 0;
	// overload this function to return the appropriate inertia tensor in world space for the given sub-body (e.g. for a car body, the inertia tensor of the wheel sub-body would be different from the inertia tensor of the hitbox sub-body)
	virtual FMatrix ComputeWorldInvInertiaTensorOfSubBody(const USSubBody& SubBody) const = 0;

	// =========== Kinematics getters and setters =================
	virtual const SKinematic& GetKinematicState() const = 0;
	virtual const SKinematic& GetKinematicStateForFrame(const unsigned int& NumFrame) const = 0;
	const FVector& GetPhysLocation() const;
	void SetPhysLocation(const FVector& NewLocation);
	const FQuat& GetPhysRotation() const;
	void SetPhysRotation(const FQuat& NewRotation);
	const FVector& GetPhysVelocity() const;
	FVector GetPhysVelocityAtPoint(const FVector& Point) const;
	const FVector& GetPhysAngularVelocity() const;
	virtual float GetPhysMaxSpeed() const = 0;
	virtual float GetPhysMaxAngularSpeed() const = 0;
	void SetPhysVelocity(const FVector& NewVelocity) { SetPhysVelocityRaw(NewVelocity.GetClampedToMaxSize(GetPhysMaxSpeed())); }
	void SetPhysVelocityAtPoint(const FVector& NewVelocity, const FVector& WorldPoint);
	void SetPhysAngularVelocity(const FVector& NewAngularVelocity) { SetPhysAngularVelocityRaw(NewAngularVelocity.GetClampedToMaxSize(GetPhysMaxAngularSpeed())); }
	const FVector& GetPhysAcceleration() const;
	const FVector& GetPhysAngularAcceleration() const;
	void SetPhysAcceleration(const FVector& NewAcceleration);
	void SetPhysAngularAcceleration(const FVector& NewAngularAcceleration);
	FVector GetPhysAccelerationAtPoint(const FVector& Point) const;
	// =============================================================

	// ====== Kinematics Utils ======
	FVector GetPhysForwardVector() const { return GetPhysRotation().GetForwardVector(); }
	FVector GetPhysRightVector() const { return GetPhysRotation().GetRightVector(); }
	FVector GetPhysUpVector() const { return GetPhysRotation().GetUpVector(); }
	float GetPhysForwardSpeed() const { return FVector::DotProduct(GetPhysVelocity(), GetPhysForwardVector()); }
	bool IsPhysGoingForward() const { return GetPhysForwardSpeed() > 0; }
	// ==============================

	// ===== Kinematics modifiers (e.g. for applying impulses) =======
	void AddPhysVelocity(const FVector& DeltaVelocity) { SetPhysVelocity(GetPhysVelocity() + DeltaVelocity); }
	void AddPhysAngularVelocity(const FVector& DeltaAngularVelocity) { SetPhysAngularVelocity(GetPhysAngularVelocity() + DeltaAngularVelocity); }
	void AddPhysImpulseAtPoint(const FVector& Impulse, const FVector& WorldPoint, const USolidSubBody* SubBody = nullptr);
	void AddPhysAcceleration(const FVector& DeltaAcceleration);
	void AddPhysAngularAcceleration(const FVector& DeltaAngularAcceleration);
	void AddPhysAngularAccelerationLocal(const FVector& LocalAngularAccel);
	void AddPhysForceAtPoint(const FVector& Force, const FVector& WorldPoint, const USolidSubBody* SubBody = nullptr);
	// overload this function to apply an impulse to the parent component at the given world point (e.g. for hitboxes to apply impulse to car body)
	virtual void ApplyImpulse(const FVector& LinearImpulse, const FVector& WorldPoint, const USolidSubBody* SubBody = nullptr);
	// ================================================================

	// This is used to advance the state to the time of impact after a hit is detected
	void IntegrateKinematics(const float& SubDelta);
	void UpdateSubBodiesKinematics();

	// Called once per physics frame to reset any cached info in the component or its sub-bodies (e.g. hit info)
	virtual void ResetForFrame(const float& Delta);
	// overload this function to sweep all sub-bodies for the remaining delta time and return the earliest time of impact and the sub-body that should resolve it
	// (e.g. for a car body, if a wheel hits before the hitbox, then the wheel sub-body should resolve first)
	SComponentTOI SweepTOISubBodies(const float& RemainingDelta, const float& LastSubDelta);
	// overload this function for the component to perform any necessary updates after the physics state has been updated
	void PostPhysicsUpdate(const float& delta);
	// overload this function to set whether the component is upside down (e.g. for a car body, this would be whether the car is flipped over)
	virtual void SetIsUpsideDown(bool bUpsideDown) = 0;
	// overload this function to return whether the component is upside down (e.g. for a car body, this would be whether the car is flipped over)
	virtual bool IsUpsideDown() const = 0;
	// overload this function to know whether the main SubBody is in auto-recover mode (e.g. for a car body, this would be whether the car is trying to right itself after being flipped over)
	virtual bool IsInAutoRecover() const = 0;
	// overload this function to return whether the sub-bodies are in auto-recover mode (e.g. for a car body, this would be whether the wheels are trying to right themselves after being flipped over)
	virtual bool IsSubBodyInAutoRecoverMode() const = 0;
	virtual void RcvImpactOnSubBody(const USSubBody& SubBody, const FVector& Location) = 0;

	void AddExternalSubBodies(const TArray<USSubBody*>& ExtSubBodies);
	void RemoveExternalSubBodies(const TArray<USSubBody*>& ExtSubBodies);

	virtual ~ISpeedComponent() = default;
protected:
	// overload this function to advance the physics state of the parent component by `SubDelta` seconds
	// SubDelta is a fraction of delta during which there is NO collision, so the physics state should be advanced by SubDelta seconds without checking for collision.
	// This is used to advance the state to the time of impact after a hit is detected
	void IntegrateKinematicsPrv(const float& SubDelta);
	// overload this function to perform any necessary updates after the physics state has been updated for SubDelta seconds (e.g. for updating the kinematic state of sub-bodies based on the new physics state of the parent component)
	virtual void PostIntegrateKinematics(const float& SubDelta) {};

	// overload this function for the component to perform any necessary updates after the physics state has been updated
	virtual void PostPhysicsUpdatePrv(const float& delta) = 0;

	virtual void SetKinematicState(const SKinematic& NewKinematicState) = 0;
	void SetPhysVelocityRaw(const FVector& NewVelocity);
	void SetPhysAngularVelocityRaw(const FVector& NewAngularVelocity);
};