// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Physics/NetworkPhysicsComponent.h"
#include "IAmSpeed/Base/SUtils.h"
#include "SpeedPhysicsComponent.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(SpeedNetcodeLog, Log, All);

struct FNetCorrAccum
{
	FVector RemPos = FVector::ZeroVector;     // cm
	FQuat   RemRot = FQuat::Identity;         // delta quat restant (Past^-1 * Target)
	FVector RemVel = FVector::ZeroVector;     // cm/s
	FVector RemAngVel = FVector::ZeroVector;  // rad/s

	bool bActive = false;
	int32 SourceServerFrame = -1;
	int32 SourceLocalFrame = -1;

	void Reset()
	{
		RemPos = FVector::ZeroVector;
		RemRot = FQuat::Identity;
		RemVel = FVector::ZeroVector;
		RemAngVel = FVector::ZeroVector;
		bActive = false;
		SourceServerFrame = -1;
		SourceLocalFrame = -1;
	}

	static float AlphaFromTau(float Dt, float Tau)
	{
		return 1.f - FMath::Exp(-Dt / FMath::Max(Tau, 1e-4f));
	}

	static FVector RejectNormal(const FVector& V, const FVector& N)
	{
		// retire la composante sur N (projection sur le plan tangent)
		return V - FVector::DotProduct(V, N) * N;
	}
};

struct SBaseGameState
{
	unsigned int NumFrame = 0; // number of frames that have passed since the start of simulation
	FVector TestVelocity = FVector::ZeroVector; // Velocity set for testing purpose
};

USTRUCT()
struct FBasePhysicsState
{
	GENERATED_BODY()

	// current kinematic state of the component
	SKinematic Kinematic;
};

USTRUCT()
struct FClientNetSettings
{
	GENERATED_BODY()

	// whether to compare position to trigger resimulation when comparing client and server data
	bool bEnablePositionResimulation = true;
	// whether to compare rotation to trigger resimulation when comparing client and server data
	bool bEnableRotationResimulation = true;
	// whether to compare velocity to trigger resimulation when comparing client and server data
	bool bEnableVelocityResimulation = true;
	// whether to compare angular velocity to trigger resimulation when comparing client and server data
	bool bEnableAngularVelocityResimulation = true;

	// in cm, if the position difference between client and server is above this threshold, it will trigger a resimulation
	float PositionResimulationThreshold = 400.0f;
	// in degrees, if the rotation difference between client and server is above this threshold, it will trigger a resimulation
	float RotationResimulationThreshold = 30.0f;
	// in cm/s, if the velocity difference between client and server is above this threshold, it will trigger a resimulation
	float VelocityResimulationThreshold = 500.0f;
	// in rad/s, if the angular velocity difference between client and server is above this threshold, it will trigger a resimulation
	float AngularVelocityResimulationThreshold = 7.0f;
};

USTRUCT()
struct FSimProxyNetSettings
{
	GENERATED_BODY()

	// whether to compare position to trigger resimulation when comparing client and server data
	bool bEnablePositionResimulation = true;
	// whether to compare rotation to trigger resimulation when comparing client and server data
	bool bEnableRotationResimulation = true;
	// whether to compare velocity to trigger resimulation when comparing client and server data
	bool bEnableVelocityResimulation = true;
	// whether to compare angular velocity to trigger resimulation when comparing client and server data
	bool bEnableAngularVelocityResimulation = true;

	// in cm, if the position difference between client and server is above this threshold, it will trigger a resimulation
	float PositionResimulationThreshold = 100.0f;
	// in degrees, if the rotation difference between client and server is above this threshold, it will trigger a resimulation
	float RotationResimulationThreshold = 30.0f;
	// in cm/s, if the velocity difference between client and server is above this threshold, it will trigger a resimulation
	float VelocityResimulationThreshold = 400.0f;
	// in rad/s, if the angular velocity difference between client and server is above this threshold, it will trigger a resimulation
	float AngularVelocityResimulationThreshold = 7.0f;
};

/** Speed state data that will be used in the state history to rewind the simulation at some point in time */
USTRUCT()
struct IAMSPEED_API FNetworkBaseSpeedState : public FNetworkPhysicsData
{
	GENERATED_BODY()

	/** Physical state of the skycar */
	FBasePhysicsState BaseState;

	// true iff this state is corresponding to an autonomous proxy on this simulation
	bool bIsAutonomousProxy = false;

	// Netcode settings used on the client when this state was recorded
	FClientNetSettings ClientNetSettings;
	// Netcode settings used on the simulation proxy when this state was recorded
	FSimProxyNetSettings SimProxyNetSettings;

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

	bool CompareDataSimProxy(const FNetworkBaseSpeedState& C);

	/** Merge data into this input */
	virtual void MergeData(const FNetworkPhysicsData& FromData) override;
};

template<>
struct TStructOpsTypeTraits<FNetworkBaseSpeedState> : public TStructOpsTypeTraitsBase2<FNetworkBaseSpeedState>
{
	enum
	{
		WithNetSerializer = true,
	};
};

USTRUCT()
struct FNetworkBaseSpeedInputState : public FNetworkPhysicsData
{
	GENERATED_USTRUCT_BODY()

	/**  Serialize data function that will be used to transfer the struct across the network */
	bool NetSerialize(FArchive& Ar, class UPackageMap* Map, bool& bOutSuccess);
};

template<>
struct TStructOpsTypeTraits<FNetworkBaseSpeedInputState> : public TStructOpsTypeTraitsBase2<FNetworkBaseSpeedInputState>
{
	enum
	{
		WithNetSerializer = true,
	};
};


struct FPhysicsSpeedTraits
{
	using InputsType = FNetworkBaseSpeedInputState;
	using StatesType = FNetworkBaseSpeedState;
};

/**
 * 
 */
UCLASS()
class IAMSPEED_API USpeedPhysicsComponent : public UNetworkPhysicsComponent
{
	GENERATED_BODY()
	
};
