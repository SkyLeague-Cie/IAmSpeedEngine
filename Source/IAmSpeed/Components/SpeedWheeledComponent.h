// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "ISpeedWheeledComponent.h"
#include "Netcode/SpeedPhysicsComponent.h"
#include "Netcode/SpeedWheeledPhysicsComponent.h"
#include "SpeedWheeledComponent.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(WheelNetcodeLog, Log, All);
DECLARE_LOG_CATEGORY_EXTERN(SpeedInputLog, Log, All);
DECLARE_LOG_CATEGORY_EXTERN(SpeedPhysicsLog, Log, All);

class UBoxSubBody;
class USpeedWheeledComponent;
class ASpeedCar;

class IAMSPEED_API USpeedSimulation : public UChaosWheeledVehicleSimulation
{
public:

	USpeedSimulation() {};
	// void ApplySuspensionForces(float DeltaTime, TArray<FWheelTraceParams>& WheelTraceParams) override;
	bool CanSimulate() const override { return UChaosWheeledVehicleSimulation::CanSimulate(); }
	void ProcessSteering(const FControlInputs& ControlInputs) override {};
	void UpdateState(float DeltaTime, const FChaosVehicleAsyncInput& InputData, Chaos::FRigidBodyHandle_Internal* Handle) override {};
	void UpdateSimulation(float DeltaTime, const FChaosVehicleAsyncInput& InputData, Chaos::FRigidBodyHandle_Internal* Handle) override {};
	void ApplyInput(const FControlInputs& ControlInputs, float DeltaTime) override {};

	USpeedWheeledComponent* SpeedWheeledComponent = nullptr;
};

/**
 * 
 */
UCLASS()
class IAMSPEED_API USpeedWheeledComponent : public UChaosWheeledVehicleMovementComponent, public ISpeedWheeledComponent
{
	GENERATED_BODY()
	
	friend struct FNetworkBaseSpeedState;
	friend struct FNetworkWheeledSpeedState;
	friend struct FNetworkWheeledSpeedInputState;

public:
	USpeedWheeledComponent(const FObjectInitializer& ObjectInitializer);

	// Set the owner of this component. Call this at begin play
	virtual void SetOwner(AActor* NewOwner);
	ASpeedCar* GetSpeedCarOwner() const;
	/** Used to create any physics engine information for this component */
	virtual void OnCreatePhysicsState() override;
	/** Used to shut down and physics engine structure for this component */
	virtual void OnDestroyPhysicsState() override;
	// Used to create all sub bodies
	virtual TArray<USSubBody*> CreateSubBodies() override;
	// Used to create wheel sub bodies
	virtual TArray<USWheelSubBody*> CreateWheelSubBodies() override;
	// Used to create hitbox sub body
	virtual UBoxSubBody* CreateHitboxSubBody();
	// Used to create one wheel sub body, with the given wheel index
	virtual USWheelSubBody* CreateWheelSubBody(const int& WheelIndex);

	// Number of frames since the start of simulation
	unsigned int NumFrame() const override;
	// Number of frames before the component can move after spawning
	uint16 GetMinNbFramesBeforeCanMove() const;
	// Mass to use for physics simulation
	float GetPhysMass() const override;
	// Center of mass to use for physics simulation (in world space)
	FVector GetPhysCOM() const override;
	// Get array of sub-bodies owned by this component
	const TArray<USSubBody*>& GetSubBodies() const;
	// Get SubBody Config by SubBody
	SubBodyConfig GetSubBodyConfig(const USSubBody& SubBody) const;
	// overload this function to return the array of wheel sub-bodies owned by this component
	WheelSubBodyConfig GetWheelSubBodyConfig(const USWheelSubBody& SubBody) const override;

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

	// overload this function for the component to perform any necessary updates after the physics state has been updated by the physics engine (e.g apply network corrections, quantize the physics state to reduce network bandwidth, etc.)
	virtual void PostPhysicsUpdatePrv(const float& delta);
	// overload this function to set whether the component is upside down (e.g. for a car body, this would be whether the car is flipped over)
	virtual void SetIsUpsideDown(bool bUpsideDown);
	// overload this function to return whether the component is upside down (e.g. for a car body, this would be whether the car is flipped over)
	virtual bool IsUpsideDown() const;
	// overload this function to know whether the main SubBody is in auto-recover mode (e.g. for a car body, this would be whether the car is trying to right itself after being flipped over)
	virtual bool IsInAutoRecover() const;
	// overload this function to return whether the sub-bodies are in auto-recover mode (e.g. for a car body, this would be whether the wheels are trying to right themselves after being flipped over)
	virtual bool IsSubBodyInAutoRecoverMode() const;
	virtual bool IsInWheelAutoRecover() const;
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

	// overload this function to update the on-ground states of the wheel sub-bodies
	void UpdateWheelOnGroundStates() override;
	// Get throttle input for the current frame (value between -1 and 1)
	float GetPhysThrottleInput() const override;
	// Get brake input for the current frame (value between -1 and 1)
	float GetPhysBrakeInput() const override;
	// Get steering input for the current frame (value between -1 and 1)
	float GetPhysSteeringInput() const override;
	void SetPhysThrottleInput(const float& Throttle);
	void SetPhysBrakeInput(const float& Brake);
	void SetPhysSteeringInput(const float& Steering);

	void RegisterWheelGroundContact(const SWheelGroundContact& Contact) override;

	float GetForwardFriction() const;
	float GetSideFriction() const;
	virtual float GetLocalForwardFriction() const;
	virtual float GetLocalSideFriction() const;

	// =========== Wheel functions ===========
	bool LeftFrontWheelIsOnGround() const;
	bool RightFrontWheelIsOnGround() const;
	bool LeftRearWheelIsOnGround() const;
	bool RightRearWheelIsOnGround() const;
	bool FrontWheelsAreOnGround() const;
	bool RearWheelsAreOnGround() const;
	bool LeftWheelsAreOnGround() const;
	bool RightWheelsAreOnGround() const;
	unsigned char NumWheelsOnGround() const;
	const TArray<TObjectPtr<USWheelSubBody>>& GetWheelSubBodies() const override;
	// update wheel position, angular velocity etc. in Game Thread
	virtual void UpdateWheelVisuals();
	// recover wheel state from network replication
	virtual void RecoverWheelState();
	// register wheel state for network replication
	virtual void RegisterWheelState();
	float GetSuspensionOffset(int WheelIndex) override;

	bool CanMove() const;
	bool CountdownHasStarted() const;
	void StartConfrontationInSec(const unsigned int& TimeSec);
	void StartConfrontationLocal(const unsigned int& TimeSec);
	UFUNCTION(reliable, NetMulticast)
	void StartConfrontationMulti(const unsigned int& TimeSec);
	void StartTestWithVelocity(const FVector& InitialVelocity);
	void StartTestWithVelocityLocal(const FVector& InitialVelocity);
	UFUNCTION(reliable, NetMulticast)
	void StartTestWithVelocityMulti(const FVector& InitialVelocity);
	void SetCannotMove();
	void SetCannotMoveLocal();
	UFUNCTION(reliable, NetMulticast)
	void SetCannotMoveMulti();

	// bump methods
	void DemoedBy(ASpeedCar* otherCar);

	void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
private:
	void AsyncPhysicsTickComponent(float DeltaTime, float SimTime) override final;
	void PhysicsTick(const float& DeltaTime, const float& SimTime);

	void UpdateFrameState(const float& SimTime);
	void UpdateNumFrame(const float& SimTime);
	bool GetPredictedState(const int32& LocalFrame, FBasePhysicsState& OutState) const;
	bool GetPredictedState(const int32& LocalFrame, FWheeledPhysicsState& OutState) const;

	void HandleGravity();
	void HandleDamping(const float& delta);
	void HandleGroundFriction(const float& delta);
	void HandleSuspension(const float& delta);

	void applyAccelerationConstraint(const float& delta);
	void applyAngularAccelerationConstraint(const float& delta);

	// Timer method to handle countdown
	void HandleCountdownTimer();
	// Handle wheel timers
	void HandleWheelTimers();
	// End Timer methods
protected:
	virtual void InitChaosVehicle();
	void InitNetwork();
	void SetSubBodies(const TArray<USSubBody*>& NewSubBodies);

	void SetEngineFPS(const unsigned int& FPS);
	float GetEngineFPS() const;

	// Update Inputs
	virtual void UpdateInputs();
	// Apply here Gravity, air/ground drag, resting forces and other forces that shoukd be applied before gameplay
	virtual void PreGameplayTick(const float& DeltaTime, const float& SimTime);
	// Apply here gameplay forces (e.g. from player input or AI)
	virtual void GameplayTick(const float& DeltaTime, const float& SimTime);
	// Handle inputs and apply corresponding forces (e.g. from player input or AI)
	virtual void HandleInputs(const float& DeltaTime);
	// Apply here any necessary updates after the gameplay forces have been applied
	virtual void PostGameplayTick(const float& DeltaTime, const float& SimTime);

	void HandleGroundAirState();
	virtual void HandleRestForce();
	virtual void UpdateAutoRecoverState();
	virtual void HandleAngularDamping(const float& delta);
	virtual void HandlePhysicsAutoRecover();
	virtual void HandleAcceleration();
	virtual void HandleSteering(const float& delta);
	virtual bool DisableSuspensionThisFrame() const;
	// Handle Timers
	virtual void HandleTimers();

	float ComputeThrottleAccel() const;
	float ComputeBrakeAccel() const;

	virtual void ApplyAccelKinematicsConstraint(const float& delta);
	virtual void TagStateHistoryProxyRole();
	virtual void ApplyNetworkCorrection(const float& delta);
	virtual void QuantizePhysicalState();

	TArray<SWheelGroundContact>& GetPendingWheelContacts() override;

	void SetGroundState();
	void SetAirState();
	virtual void SetGroundStatePrv();
	virtual void SetAirStatePrv();

	// =========== Acceleration and steering functions ===========
	// return the acceleration to apply in cm/s^2 based on the throttle/brake input and whether we want to move forward or backward
	virtual float ComputeAccel(const float& InputValue, bool wantToMoveForward) const;
	// return the maximum speed in cm/s that the component can reach based on steering input with the given input (value between -1 and 1)
	virtual float GetSteeringSpeedCap() const;
	// return the maximum throttle speed in cm/s that the component can reach based on whether it is on the ground or not
	virtual float GetMaxThrottleSpeed() const;

	// return true iff the component is not allowed to steer
	virtual bool NoSteeringAllowed() const;
	virtual float ComputeSteeringRadius(const float& ForwardVelocity, const float& AbsSteeringInput) const;
	FVector ComputeTurningForce(const float& SteeringInput, const float& UpDot,
		const float& ForwardVelocity, const float& DesiredRadius, const FVector& WheelRight) const;
	virtual void DampenAngularVelocity(const float& DampingFactor, const FVector& VelocityToDampen, const float& delta);
	virtual void DampenAirAngularVelocity(const FVector& VelocityToDampen, const float& delta);
	void SetDampenedAirAngularVelocity(const FVector& TargetAngularVelocity, const FVector& VelocityToSetNow, const float& delta);

	const TObjectPtr<UBoxSubBody>& GetHitboxSubBody() const { return HitboxSubBody; }

	/** Read current state for simulation */
	void UpdateState(float DeltaTime) override;

	// Netcode methods
	virtual void RecordPredictedState();
	TObjectPtr<UNetworkPhysicsSettingsDataAsset> NetDataAsset = nullptr;
public:

	//=========== Configuration parameters for the movement component ===========
	// Time in seconds before the component can move at the start of the simulation or after being reset (e.g. after a respawn)
	UPROPERTY(BlueprintReadWrite, Category = Base, EditDefaultsOnly,
		meta = (ClampMin = "0.0", UIMin = "0.0"))
	float TimeBeforeCanMove = 0.2f;
	// Mass of the component in kg
	UPROPERTY(BlueprintReadWrite, Category = Base, EditDefaultsOnly,
		meta = (ClampMin = "0.0", UIMin = "0.0"))
	float PhysMass = 1000.0f;
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
	float PhysMaxAngularSpeed = 4.f;
	// damping factor of the component (0.0 means no damping, 1.0 means full damping)
	UPROPERTY(BlueprintReadWrite, Category = Base, EditDefaultsOnly,
		meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
	float PhysDamping = 0.0f;
	// center of mass of the actor in local space (relative to the actor's origin)
	UPROPERTY(BlueprintReadWrite, Category = Base, EditDefaultsOnly)
	FVector CenterOfMass = FVector::ZeroVector;

	// Maximum throttle speed in cm/s when the component is on the ground
	UPROPERTY(BlueprintReadWrite, Category = Physics, EditDefaultsOnly)
	float MaxThrottleSpeed = 7000.0f;
	// Friction applied (in cm/s^2) when the component is on the ground (Forward, Side, Up)
	UPROPERTY(BlueprintReadWrite, Category = Physics, EditDefaultsOnly)
	FVector GroundFriction = FVector(510.0, 6120.0, 0.0);
	// Ground angular damping coefficient to dampen angular velocity when car is on the ground (in %)
	UPROPERTY(BlueprintReadWrite, Category = Physics, EditDefaultsOnly,
		meta = (ClampMin = "0.0", ClampMax = "100.0", UIMin = "0.0", UIMax = "100.0"))
	float GroundAngularDamping = 5.0f;
	// Roll drag coefficient to dampen angular velocity (in %)
	UPROPERTY(BlueprintReadWrite, Category = Physics, EditDefaultsOnly,
		meta = (ClampMin = "0.0", ClampMax = "100.0", UIMin = "0.0", UIMax = "100.0"))
	float RollDragCoeff = 5.0f;
	// Pitch drag coefficient to dampen angular velocity (in %)
	UPROPERTY(BlueprintReadWrite, Category = Physics, EditDefaultsOnly,
		meta = (ClampMin = "0.0", ClampMax = "100.0", UIMin = "0.0", UIMax = "100.0"))
	float PitchDragCoeff = 3.0f;
	// Yaw drag coefficient to dampen angular velocity (in %)
	UPROPERTY(BlueprintReadWrite, Category = Physics, EditDefaultsOnly,
		meta = (ClampMin = "0.0", ClampMax = "100.0", UIMin = "0.0", UIMax = "100.0"))
	float YawDragCoeff = 2.0f;
	// forward throttle acceleration in cm/s^2 when the component is in the air
	UPROPERTY(BlueprintReadWrite, Category = Acceleration, EditDefaultsOnly)
	float AirForwardThrottle = 0.0f;
	// backward throttleacceleration in cm/s^2 when the component is in the air
	UPROPERTY(BlueprintReadWrite, Category = Acceleration, EditDefaultsOnly)
	float AirBackwardThrottle = 0.0f;
	// Throttle acceleration in cm/s^2 when the component is on the ground
	UPROPERTY(BlueprintReadWrite, Category = Acceleration, EditDefaultsOnly)
	float ThrottleAccel = 2000.0f;
	// Throttle acceleration in cm/s^2 when the component is on the ground
	UPROPERTY(BlueprintReadWrite, Category = Acceleration, EditDefaultsOnly)
	float SubSteeringAccel = 2000.0f;
	// Brake acceleration in cm/s^2 when the component is on the ground
	UPROPERTY(BlueprintReadWrite, Category = Acceleration, EditDefaultsOnly)
	float BrakeAccel = 4000.0f;

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

	/** Hit Box component */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = Collision, meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UBoxSubBody> HitboxSubBody = nullptr; // pointer to the hitbox sub-body (if any) owned by this component
	/** Wheel Sub Bodies*/
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = Collision, meta = (AllowPrivateAccess = "true"))
	TArray< TObjectPtr<USWheelSubBody>> WheelSubBodies; // array of wheel sub-bodies owned by this component (e.g. for a car body, this would be the wheels)
	/** Network Settings*/
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = Netcode, meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UNetworkPhysicsSettingsComponent> SNetworkSettings = nullptr;

	/** Name of the HitboxSubBody */
	static const FName HitboxName;
	/** Names of the Wheel Subodies*/
	static const TArray<FName> WheelNames;
private:
	USpeedSimulation* SkySimulation = nullptr;
	ASpeedCar* SpeedCarOwner = nullptr;
	virtual TUniquePtr<Chaos::FSimpleWheeledVehicle> CreatePhysicsVehicle() override
	{
		// Make the Vehicle Simulation class that will be updated from the physics thread async callback
		VehicleSimulationPT = MakeUnique<USpeedSimulation>();
		VehicleSimulationPT->RigidHandle = nullptr;
		static_cast<USpeedSimulation*>(VehicleSimulationPT.Get())->SpeedWheeledComponent = this;
		SkySimulation = static_cast<USpeedSimulation*>(VehicleSimulationPT.Get());

		return UChaosVehicleMovementComponent::CreatePhysicsVehicle();
	}


	//=========== Internal state variables for the movement component ===========
	SBaseGameState BaseGameState; // current game state of the component
	FBasePhysicsState BasePhysicsState; // current physics state of the component (replicated on network)
	FWheeledGameState WheeledGameState; // current wheeled game state of the component
	FWheeledPhysicsState WheeledPhysicsState; // current wheeled physics state of the component (replicated on network)
	FWheeledInputState WheeledUserInput; // input state given by the user for this component (replicated on network)
	FWheeledInputState WheeledPhysicalInput; // input state used for physics simulation (e.g. after being processed from the user input)
	FVector CarLocalInvI = FVector::ZeroVector; // local inverse inertia tensor of the car body (in local space)

	TArray<int32> PredictedBaseFrames;
	TArray<FBasePhysicsState> PredictedBaseStates; // array of predicted states for network correction
	TArray<FWheeledPhysicsState> PredictedWheeledStates; // array of predicted states for network correction

	float EngineFPS = 120.0; // physics simulation FPS (Hz)
	uint16 MinNbFramesBeforeCanMove = 0; // minimal number of frames before the component can move at the start of the simulation or after being reset (e.g. after a respawn)
	TArray<USSubBody*> SubBodies; // array of sub-bodies owned by this component (e.g. for a car body, this would be the hitbox and the wheels)
	TArray<USSubBody*> ExtSubBodies; // array of external sub-bodies

	TArray<SWheelGroundContact> PendingWheelContacts; // array of pending wheel ground contacts to be registered at the end of the frame

	// ========== Netcode variables ==========
	UPROPERTY()
	TObjectPtr<UNetworkPhysicsComponent> SNetworkPhysicsComponent = nullptr;
	UPROPERTY()
	TObjectPtr<UNetworkPhysicsComponent> WheeledNetworkPhysicsComponent = nullptr;
	int32 NetCorr_LastServerFrame = INDEX_NONE;
	int32 NetCorr_LastLocalFrame = INDEX_NONE;
	int32 NetCorr_BaseNumPredictedFrames = 1;
	bool bNetCorrHasTarget = false;
	int32 NetCorrTickCount = 0;
	FNetworkBaseSpeedState NetCorrTarget;
	FNetCorrAccum NetCorrAccum;
	FVector NetCorr_LastContactN = FVector::UpVector;
	uint8 NetCorr_StableNFrames = 0;
};
