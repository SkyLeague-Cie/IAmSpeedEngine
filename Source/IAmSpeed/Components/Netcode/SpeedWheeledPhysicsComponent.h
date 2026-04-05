// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Physics/NetworkPhysicsComponent.h"
#include "IAmSpeed/Base/SUtils.h"
#include "SpeedWheeledPhysicsComponent.generated.h"


USTRUCT()
struct FWheeledGameState
{
	GENERATED_BODY()

	// whether the car has all its wheels on the ground
	UPROPERTY()
	bool bGroundState = false;
	// true if the car is upside down
	UPROPERTY()
	bool bIsUpsideDown = false;
	/** Angular velocity for each wheel */
	UPROPERTY()
	float WheelsOmega[4] = { 0,0,0,0 };
	/** Angular position for each wheel */
	UPROPERTY()
	float WheelsAngularPosition[4] = { 0,0,0,0 };
	UPROPERTY()
	uint8 NbWheelsOnGround = 0;
	UPROPERTY()
	uint32 NbFrameSinceInAir = 0;
};

USTRUCT()
struct FWheeledPhysicsState
{
	GENERATED_BODY()

	// true iff game countdown has started
	UPROPERTY()
	bool bStartCountdown = false;
	// number of frames before car can move at the beginning of the game
	UPROPERTY()
	uint16 nbFramesbeforeCanMove = 1;
	// number of frames since last impact
	UPROPERTY()
	uint8 FramesSinceLastImpact = 0;

	// allowed side velocity for steering
	UPROPERTY()
	FVector_NetQuantize100 AllowedSideVelocity = FVector::ZeroVector;
	// allowed angular velocity for steering
	UPROPERTY()
	FVector_NetQuantize100 AllowedAngularVelocity = FVector::ZeroVector;

	// Wheel states
	/** Suspension latest displacement to be used while simulating(to compute spring displacement speed)*/
	UPROPERTY()
	float SuspensionLastDisplacement[4] = { 0,0,0,0 };

	UPROPERTY()
	uint8 NbFramesSinceGroundContact = 0; // number of frames since last ground contact

	int16 QuantizeLastSuspensionDisplacement(int wheelIndex) const;
	float DequantizeLastSuspensionDisplacement(int16 q) const;
	
	static FORCEINLINE int16 QuantizeSigned(float v, float scale)
	{
		const int32 q = FMath::RoundToInt(v * scale);
		return (int16)FMath::Clamp(q, -32767, 32767);
	}

	static FORCEINLINE float DequantizeSigned(int16 q, float invScale)
	{
		return (float)q * invScale;
	}
};

USTRUCT()
struct FWheeledInputState
{
	GENERATED_BODY()

	UPROPERTY()
	bool bCanMove = false;
	UPROPERTY()
	uint8 Throttle = 0;
	UPROPERTY()
	uint8 Brake = 0;
	UPROPERTY()
	int8 Steer = 0;
};

/** Speed state data that will be used in the state history to rewind the simulation at some point in time */
USTRUCT()
struct FNetworkWheeledSpeedState : public FNetworkPhysicsData
{
	GENERATED_BODY()

	/** Physical state of the skycar */
	UPROPERTY()
	FWheeledPhysicsState WheeledState;

	// true iff this state is corresponding to an autonomous proxy on this simulation
	bool bIsAutonomousProxy = false;

	/**  Apply the data onto the network physics component */
	virtual void ApplyData(UActorComponent* NetworkComponent) const override;

	/**  Build the data from the network physics component */
	virtual void BuildData(const UActorComponent* NetworkComponent) override;

	/**  Serialize data function that will be used to transfer the struct across the network */
	bool NetSerialize(FArchive& Ar, class UPackageMap* Map, bool& bOutSuccess);

	/** Interpolate the data in between two inputs data */
	virtual void InterpolateData(const FNetworkPhysicsData& MinData, const FNetworkPhysicsData& MaxData) override;

	/** Define how to compare client and server data for the same frame, returning false means the data differ enough to trigger a resimulation.
	* @param PredictedData is data predicted on the client to compare with the current data received from the server.
	* NOTE: To use this function, CVars np2.Resim.CompareStateToTriggerRewind and/or np2.Resim.CompareInputToTriggerRewind needs to be set to true
	* or the equivalent settings overridden on the actor via UNetworkPhysicsSettingsComponent.
	*/
	virtual bool CompareData(const FNetworkPhysicsData& PredictedData);

	/** Merge data into this input */
	virtual void MergeData(const FNetworkPhysicsData& FromData) override;
};

template<>
struct TStructOpsTypeTraits<FNetworkWheeledSpeedState> : public TStructOpsTypeTraitsBase2<FNetworkWheeledSpeedState>
{
	enum
	{
		WithNetSerializer = true,
	};
};

USTRUCT()
struct FNetworkWheeledSpeedInputState : public FNetworkPhysicsData
{
	GENERATED_BODY()

	UPROPERTY()
	FWheeledInputState WheeledInput;

	UPROPERTY()
	uint32 ClientFrame = 0; // NumFrame() client side when the input was sent

	UPROPERTY()
	bool bIsAutonomousProxy = false;

	/**  Apply the data onto the network physics component */
	virtual void ApplyData(UActorComponent* NetworkComponent) const override;

	/**  Build the data from the network physics component */
	virtual void BuildData(const UActorComponent* NetworkComponent) override;

	/** Use to decay desired data during resimulation if data is forward predicted.
	* @param DecayAmount = Total amount of decay as a multiplier. 10% decay = 0.1.
	* NOTE: Decay is not accumulated, the data will be in its original state each time DecayData is called. DecayAmount will increase each time the input is predicted (reused).
	* EXAMPLE: Use to decay steering inputs to make resimulation not predict too much with a high steering value. Use DecayAmount of 0.1 to turn a steering value of 0.5 into 0.45 for example.
	*/
	virtual void DecayData(float DecayAmount);

	/**  Serialize data function that will be used to transfer the struct across the network */
	bool NetSerialize(FArchive& Ar, class UPackageMap* Map, bool& bOutSuccess);

	/** Define how to interpolate between two data points if we have a gap between known data.
	* @param MinData is data from a previous frame.
	* @param MaxData is data from a future frame.
	* EXAMPLE: We have input data for frame 1 and 4 and we need to interpolate data for frame 2 and 3 based on frame 1 as MinData and frame 4 as MaxData.
	*/
	virtual void InterpolateData(const FNetworkPhysicsData& MinData, const FNetworkPhysicsData& MaxData) override;

	/** Define how to compare client and server data for the same frame, returning false means the data differ enough to trigger a resimulation.
	* @param PredictedData is data predicted on the client to compare with the current data received from the server.
	* NOTE: To use this function, CVars np2.Resim.CompareStateToTriggerRewind and/or np2.Resim.CompareInputToTriggerRewind needs to be set to true
	* or the equivalent settings overridden on the actor via UNetworkPhysicsSettingsComponent.
	*/
	virtual bool CompareData(const FNetworkPhysicsData& PredictedData);

	/** Define how to merge data together
	* @param FromData is data from a previous frame that is getting merged into the current data.
	* EXAMPLE: Simulated proxies might receive two inputs at the same time after having used the same input twice, to not miss any important inputs we need to take both inputs into account
	* and to not get behind in simulation we need to apply them both at the same simulation tick meaning we merge the two new inputs to one input.
	*/
	virtual void MergeData(const FNetworkPhysicsData& FromData) override;
};

template<>
struct TStructOpsTypeTraits<FNetworkWheeledSpeedInputState> : public TStructOpsTypeTraitsBase2<FNetworkWheeledSpeedInputState>
{
	enum
	{
		WithNetSerializer = true,
	};
};


struct FPhysicsWheeledTraits
{
	using InputsType = FNetworkWheeledSpeedInputState;
	using StatesType = FNetworkWheeledSpeedState;
};


/**
 * 
 */
UCLASS()
class IAMSPEED_API USpeedWheeledPhysicsComponent : public UNetworkPhysicsComponent
{
	GENERATED_BODY()
};
