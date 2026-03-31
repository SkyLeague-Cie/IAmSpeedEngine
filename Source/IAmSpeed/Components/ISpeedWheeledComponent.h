#pragma once

#include "ISpeedComponent.h"

struct WheelSubBodyConfig;
class USWheelSubBody;

struct SWheelGroundContact
{
	USWheelSubBody* Wheel = nullptr;

	FVector WorldPos = FVector::ZeroVector;
	FVector Normal = FVector::ZeroVector;
	FVector r = FVector::ZeroVector;              // WorldPos - COM
	float   vN = 0.f;             // normal velocity at contact
	float   InvMassEff = 0.f;     // inverse effective mass
};

/*
* ISpeedWheeledComponent: interface for speed components that have wheels, such as cars.
* This interface extends ISpeedComponent and adds wheel-specific functionality, such as getting the wheel sub-body config,
* updating the on-ground states of the wheels, and handling wheel-ground contacts.
*/
class IAMSPEED_API ISpeedWheeledComponent : public ISpeedComponent
{
	public:
	// overload this function to return the array of wheel sub-bodies owned by this component
	virtual WheelSubBodyConfig GetWheelSubBodyConfig(const USWheelSubBody& SubBody) const = 0;
	// overload this function to create the wheel sub-bodies owned by this component (e.g. for a car body, this would be creating the 4 wheel sub-bodies)
	virtual TArray<USWheelSubBody*> CreateWheelSubBodies() = 0;

	// overload this function to update the on-ground states of the wheel sub-bodies
	virtual void UpdateWheelOnGroundStates() = 0;

	// overload this function to return a value between -1 and 1 representing the throttle input for the current frame
	virtual float GetPhysThrottleInput() const = 0;
	// overload this function to return a value between -1 and 1 representing the brake input for the current frame
	virtual float GetPhysBrakeInput() const = 0;
	// returns a value between -1 and 1 representing the steering input for the current frame
	virtual float GetPhysSteeringInput() const = 0;

	virtual void RegisterWheelGroundContact(const SWheelGroundContact& Contact) = 0;
	virtual void ResolveGroupedWheelGroundContacts(const float& delta);

	// =========== Wheel functions ===========
	// overload this function to return every wheel sub-bodies owned by this component (e.g. for a car body, this would be the 4 wheel sub-bodies)
	virtual const TArray<TObjectPtr<USWheelSubBody>>& GetWheelSubBodies() const = 0;
	// overload this function to return the normal of the ground under the wheels (e.g. for a car body, this would be the normal of the ground under the wheels)
	virtual FVector GetNormalFromWheels() const;
	// return true iff every wheel sub-body owned by this component is on the ground (e.g. for a car body, this would be true if all 4 wheels are on the ground)
	bool IsOnTheGround() const;
	// return true iff at least one wheel sub-body owned by this component is on the ground (e.g. for a car body, this would be true if at least 1 wheel is on the ground)
	bool OneWheelOnGround() const;
	bool NoWheelOnGround() const;
	// return true iff the wheel sub-body with the given index is on the ground (e.g. for a car body, this would be true if the wheel with the given index is on the ground)
	bool WheelIdxIsOnGround(const int32& WheelIdx) const;

	void PostIntegrateKinematics(const float& delta) override;

	virtual ~ISpeedWheeledComponent() = default;
protected:
	virtual TArray<SWheelGroundContact>& GetPendingWheelContacts() = 0;
	static FVector QuantizeUnitNormal(const FVector& n, float q = 1e-3f);
	// overload this function for the component to perform any necessary updates after the physics state has been updated
	void PostPhysicsUpdatePrv(const float& delta) override;
};