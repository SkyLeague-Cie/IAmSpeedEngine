// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/MovementComponent.h"
#include "ISpeedComponent.h"
#include "Netcode/SpeedPhysicsComponent.h"
#include "SpeedMovementComponent.generated.h"

/**
 * 
 */
UCLASS()
class IAMSPEED_API USpeedMovementComponent : public UMovementComponent, public ISpeedComponent
{
	GENERATED_BODY()

	friend struct FNetworkBaseSpeedState;
	
public:
	USpeedMovementComponent(const FObjectInitializer& ObjectInitializer);

	void InitNetwork();
	// Set the owner of this component. Call this at begin play
	virtual void SetOwner(AActor* NewOwner);
	/** Used to create any physics engine information for this component */
	virtual void OnCreatePhysicsState() override;
	/** Used to shut down and physics engine structure for this component */
	virtual void OnDestroyPhysicsState() override;
	// Used to create the sub-bodies owned by this component (e.g. for a car body, this would be creating the wheel sub-bodies and the hitbox sub-body)
	virtual TArray<USSubBody*> CreateSubBodies() override;

	// Number of frames since the start of simulation
	unsigned int NumFrame() const override;
	// Mass to use for physics simulation
	float GetPhysMass() const override;
	// Center of mass to use for physics simulation (in world space)
	FVector GetPhysCOM() const override;
	// Get array of sub-bodies owned by this component
	const TArray<USSubBody*>& GetSubBodies() const;
	// Get SubBody Config by SubBody
	SubBodyConfig GetSubBodyConfig(const USSubBody& SubBody) const;
	
	// Get appropriate kinematic state for SubBody (e.g. wheel kinematics or hitbox kinematics from car body)
	const SKinematic& GetKinematicsOfSubBody(const USSubBody& SubBody, const unsigned int& NumFrame) const;
	// overload this function to return the appropriate inertia tensor in world space
	virtual FMatrix ComputeWorldInvInertiaTensor() const;
	// overload this function to return the appropriate inertia tensor in world space for the given sub-body (e.g. for a car body, the inertia tensor of the wheel sub-body would be different from the inertia tensor of the hitbox sub-body)
	virtual FMatrix ComputeWorldInvInertiaTensorOfSubBody(const USSubBody& SubBody) const;

	virtual const SKinematic& GetKinematicState() const;
	virtual const SKinematic& GetKinematicStateForFrame(const unsigned int& NumFrame) const;
	virtual float GetPhysMaxSpeed() const;
	virtual float GetPhysMaxAngularSpeed() const;

	// overload this function for the component to perform any necessary updates after the physics state has been updated
	void PostPhysicsUpdatePrv(const float& delta) override;
	// overload this function to set whether the component is upside down (e.g. for a car body, this would be whether the car is flipped over)
	virtual void SetIsUpsideDown(bool bUpsideDown);
	// overload this function to return whether the component is upside down (e.g. for a car body, this would be whether the car is flipped over)
	virtual bool IsUpsideDown() const;
	// overload this function to know whether the main SubBody is in auto-recover mode (e.g. for a car body, this would be whether the car is trying to right itself after being flipped over)
	virtual bool IsInAutoRecover() const;
	// overload this function to return whether the sub-bodies are in auto-recover mode (e.g. for a car body, this would be whether the wheels are trying to right themselves after being flipped over)
	virtual bool IsSubBodyInAutoRecoverMode() const;
	virtual void RcvImpactOnSubBody(const USSubBody& SubBody, const FVector& Location);

	virtual void SetKinematicState(const SKinematic& NewKinematicState);

	void RegisterTestVelocity(const FVector& InitialVelocity);
	void ApplyTestVelocity();

	bool IsAffectedByGravity() const;
	void SetIsAffectedByGravity(bool value);
	void EnableGravity();
	bool HasAuthority() const;
	bool IsOwningClient() const;
	bool IsRemoteClient() const;
	FString GetRole() const;

	void BeginPlay() override;
	void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	void StartTestWithVelocity(const FVector& InitialVelocity);
	void StartTestWithVelocityLocal(const FVector& InitialVelocity);
	UFUNCTION(reliable, NetMulticast)
	void StartTestWithVelocityMulti(const FVector& InitialVelocity);
private:
	void SetEngineFPS(const unsigned int& FPS);

	void AsyncPhysicsTickComponent(float DeltaTime, float SimTime) override final;
	void PhysicsTick(const float& DeltaTime, const float& SimTime);

	void UpdateNumFrame(const float& SimTime);

	void HandleGravity();
	void HandleDamping(const float& delta);

	void applyAccelerationConstraint(const float& delta);
	void applyAngularAccelerationConstraint(const float& delta);
protected:
	virtual void RecordPredictedState();
	bool GetPredictedState(const int32& LocalFrame, FBasePhysicsState& OutState) const;
	void SetSubBodies(const TArray<USSubBody*>& NewSubBodies);

	// Apply here Gravity, air/ground drag, resting forces and other forces that shoukd be applied before gameplay
	virtual void PreGameplayTick(const float& DeltaTime, const float& SimTime);
	// Apply here gameplay forces (e.g. from player input or AI)
	virtual void GameplayTick(const float& DeltaTime, const float& SimTime);
	// Apply here any necessary updates after the gameplay forces have been applied
	virtual void PostGameplayTick(const float& DeltaTime, const float& SimTime);

	// Handle rest force
	virtual void HandleRestForce();
	virtual void ApplyAccelKinematicsConstraint(const float& delta);
	virtual void ApplyNetworkCorrection(const float& delta);
	virtual void QuantizePhysicalState();

	virtual void TagStateHistoryProxyRole();
	TObjectPtr<UNetworkPhysicsSettingsDataAsset> NetDataAsset = nullptr;
public:
	
	//=========== Configuration parameters for the movement component ===========
	// Mass of the component in kg
	UPROPERTY(BlueprintReadWrite, Category = Base, EditDefaultsOnly,
		meta = (ClampMin = "0.0", UIMin = "0.0"))
	float PhysMass = 100.0f;
	// whether to apply gravity to the component
	UPROPERTY(BlueprintReadWrite, Category = Base, EditDefaultsOnly)
	bool bEnableGravity = true;
	// gravity acceleration in cm/s^2 (negative value means gravity is pulling down)
	UPROPERTY(BlueprintReadWrite, Category = Base, EditDefaultsOnly)
	float GravityZ = -980.0f;
	// max speed of the component in cm/s
	UPROPERTY(BlueprintReadWrite, Category = Base, EditDefaultsOnly,
		meta = (ClampMin = "0.0", UIMin = "0.0"))
	float PhysMaxSpeed = 1e4;
	// max angular speed of the component in rad/s
	UPROPERTY(BlueprintReadWrite, Category = Base, EditDefaultsOnly,
		meta = (ClampMin = "0.0", UIMin = "0.0"))
	float PhysMaxAngularSpeed = 100.0f;
	// damping factor of the component (0.0 means no damping, 1.0 means full damping)
	UPROPERTY(BlueprintReadWrite, Category = Base, EditDefaultsOnly,
		meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
	float PhysDamping = 0.0f;
	// center of mass of the actor in local space (relative to the actor's origin)
	UPROPERTY(BlueprintReadWrite, Category = Base, EditDefaultsOnly)
	FVector CenterOfMass = FVector::ZeroVector;

	//=========== Netcode parameters for the movement component ===========
	// how much of the position error to correct (0.0 means no correction, 1.0 means full correction)
	UPROPERTY(BlueprintReadWrite, Category = BaseNetcode, EditDefaultsOnly,
		meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
	float PosStabilityMultiplier = 0.4f;
	// how much of the velocity error to correct (0.0 means no correction, 1.0 means full correction)
	UPROPERTY(BlueprintReadWrite, Category = BaseNetcode, EditDefaultsOnly,
		meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
	float VelStabilityMultiplier = 0.8f;
	// how much of the rotation error to correct (0.0 means no correction, 1.0 means full correction)
	UPROPERTY(BlueprintReadWrite, Category = BaseNetcode, EditDefaultsOnly,
		meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
	float RotStabilityMultiplier = 0.2f;
	// how much of the angular velocity error to correct (0.0 means no correction, 1.0 means full correction)
	UPROPERTY(BlueprintReadWrite, Category = BaseNetcode, EditDefaultsOnly,
		meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
	float AngVelStabilityMultiplier = 0.8f;
	
	// Limit of the correction to apply on the position per second (to avoid visible jerk)
	UPROPERTY(BlueprintReadWrite, Category = BaseNetcode, EditDefaultsOnly,
		meta = (ClampMin = "0.0", UIMin = "0.0"))
	float MaxPosCorrectionPerSecond = 600.f;     // cm/s
	// Limit of the correction to apply on the velocity per second (to avoid visible jerk)
	UPROPERTY(BlueprintReadWrite, Category = BaseNetcode, EditDefaultsOnly,
		meta = (ClampMin = "0.0", UIMin = "0.0"))
	float MaxVelCorrectionPerSecond = 3600.f;    // cm/s^2
	// Limit of the correction to apply on the angular velocity per second (to avoid visible jerk)
	UPROPERTY(BlueprintReadWrite, Category = BaseNetcode, EditDefaultsOnly,
		meta = (ClampMin = "0.0", UIMin = "0.0"))
	float MaxAngVelCorrectionPerSecond = 9.f;    // rad/s^2
	// Limit of the correction to apply on the rotation per second (to avoid visible jerk)
	UPROPERTY(BlueprintReadWrite, Category = BaseNetcode, EditDefaultsOnly,
		meta = (ClampMin = "0.0", UIMin = "0.0"))
	float MaxRotCorrectionPerSecond = 180.f;     // deg/s
	
	// Deadzone for the correction to apply on the position in cm (if the error is below this threshold, no correction will be applied)
	UPROPERTY(BlueprintReadWrite, Category = BaseNetcode, EditDefaultsOnly,
		meta = (ClampMin = "0.0", UIMin = "0.0"))
	float PosCorrDeadzone = 0.1f;
	// Deadzone for the correction to apply on the velocity in cm/s (if the error is below this threshold, no correction will be applied)
	UPROPERTY(BlueprintReadWrite, Category = BaseNetcode, EditDefaultsOnly,
		meta = (ClampMin = "0.0", UIMin = "0.0"))
	float VelCorrDeadzone = 0.2f;
	// Deadzone for the correction to apply on the angular velocity in rad/s (if the error is below this threshold, no correction will be applied)
	UPROPERTY(BlueprintReadWrite, Category = BaseNetcode, EditDefaultsOnly,
		meta = (ClampMin = "0.0", UIMin = "0.0"))
	float AngVelCorrDeadzone = 0.01f;
	// Deadzone for the correction to apply on the rotation in deg (if the error is below this threshold, no correction will be applied)
	UPROPERTY(BlueprintReadWrite, Category = BaseNetcode, EditDefaultsOnly,
		meta = (ClampMin = "0.0", UIMin = "0.0"))
	float RotCorrDeadzone = 0.5f;
	/** Network Settings*/
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = Netcode, meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UNetworkPhysicsSettingsComponent> SNetworkSettings = nullptr;
private:
	//=========== Internal state variables for the movement component ===========
	SBaseGameState BaseGameState; // current game state of the component
	FBasePhysicsState BasePhysicsState; // current physics state of the component (replicated on network)

	TArray<int32> PredictedBaseFrames;
	TArray<FBasePhysicsState> PredictedBaseStates; // array of predicted states for network correction

	float EngineFPS = 120.0; // physics simulation FPS (Hz)
	TArray<USSubBody*> SubBodies; // array of sub-bodies owned by this component (e.g. for a car body, this would be the hitbox and the wheels)
	TArray<USolidSubBody*> SolidSubBodies; // array of solid sub-bodes (i.e. sub-bodies that are not purely for hit detection but also have a physical representation in the physics engine, e.g. for a car body, this would be the hitbox and the wheels)

	// ========== Netcode variables ==========
	int32 NetCorr_LastServerFrame = INDEX_NONE;
	int32 NetCorr_LastLocalFrame = INDEX_NONE;
	int32 NetCorr_BaseNumPredictedFrames = 1;
	bool bNetCorrHasTarget = false;
	int32 NetCorrTickCount = 0;
	FNetworkBaseSpeedState NetCorrTarget;
protected:
	UPROPERTY()
	TObjectPtr<UNetworkPhysicsComponent> SNetworkPhysicsComponent = nullptr;
};
