// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "IAmSpeed/Concurrency/AtomicSharedPtr.h"
namespace Speed::Replay { class FFieldWriter; }
#include "ChaosWheeledVehicleMovementComponent.h"
#include "ISpeedWheeledComponent.h"
#include "Netcode/SpeedPhysicsComponent.h"
#include "Netcode/SpeedWheeledPhysicsComponent.h"
#include <atomic>
#include <memory>
#include "IAmSpeed/Input/InputStream.h"
#include "IAmSpeed/Input/InputStreamV2.h"
#include "IAmSpeed/Input/SameFrameInputOwner.h"
#include "IAmSpeed/Input/LegacyRemotePreSlew.h"

#include "IAmSpeed/World/Simulation/NativeEpochStorageRoute.h"
#include "SpeedWheeledComponent.generated.h"
namespace Speed::Netcode::Prediction {class FPrivateRoadPhysicsWorld;class FPrivateNativeOperation;class FPrivateReferenceWheelBindingBirth;class FPrivateReferenceOwnerBirth;}
namespace Speed::NativeKinematics {class FComponentRead;}
class USpeedBranchWheeledComponent;

// Explicit construction policy; never inferred from role, time or a packet.
enum class ESpeedWheeledNetworkConstruction : uint8 { Legacy, IAmSpeedOnly };

DECLARE_LOG_CATEGORY_EXTERN(WheelNetcodeLog, Log, All);
DECLARE_LOG_CATEGORY_EXTERN(SpeedInputLog, Log, All);

class UBoxSubBody;
class USpeedWheeledComponent;
class ASpeedCar;
class ASpeedPhysicsSimulation;

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

#if !UE_BUILD_SHIPPING
namespace Speed { struct FCanonicalForeignStateForTesting; }
#endif
UCLASS()
class IAMSPEED_API USpeedWheeledComponent : public UChaosWheeledVehicleMovementComponent, public ISpeedWheeledComponent
{
	GENERATED_BODY()
    virtual const FVector& PublicPhysCOMView() const { return BasePhysicsStateStorage().Kinematic.Location; }
    virtual const FVector& PublicPhysLocalCOMView() const { return CenterOfMassWheeledDomainStorage(); }
    friend class USpeedBranchWheeledComponent;
    friend class Speed::Netcode::Prediction::FPrivateRoadPhysicsWorld;
    friend class Speed::NativeKinematics::FComponentRead;
#if WITH_DEV_AUTOMATION_TESTS
	friend struct FLocalNormalCameraFixtureAccess;
#endif

#if !UE_BUILD_SHIPPING
 friend struct Speed::FCanonicalForeignStateForTesting;
#endif
	
	friend struct FNetworkBaseSpeedState;
	friend struct FNetworkWheeledSpeedState;
	friend struct FNetworkWheeledSpeedInputState;

public:
#if WITH_DEV_AUTOMATION_TESTS
    // Isolated native fixtures have one IAmSpeed owner; suppress the host tick.
    void DisableAsyncPhysicsTickForTesting() { SetAsyncPhysicsTickEnabled(false); }
#endif
#if !UE_BUILD_SHIPPING
    // Joined owner-lane diagnostics only; returns the actual applied input.
    const FWheeledInputState& GetWheeledPhysicalInputForWitness() const { return WheeledPhysicalInputStorage(); }
#endif

#if !UE_BUILD_SHIPPING
    /** Exact recorded-slot query for diagnostic fixtures; no state mutation. */
    bool TryGetRecordedStatesForTesting(const int32& LocalFrame,
        FBasePhysicsState& OutBase, FWheeledPhysicsState& OutWheeled) const
    {
        return GetBaseState(LocalFrame, OutBase) && GetWheeledState(LocalFrame, OutWheeled);
    }
#endif

	USpeedWheeledComponent(const FObjectInitializer& ObjectInitializer);
	void OnRegister() override;

	// Set the owner of this component. Call this at begin play
	virtual void SetOwner(AActor* NewOwner);
	void BeginPlay() override;
	bool ValidateSimulationBindings(FString& OutReason) const override;
    bool SupportsJoinedActorKinematicCapture1213() const override
    { return GetClass() == USpeedWheeledComponent::StaticClass(); }
	bool RetireInputProcessingOnWorker() override;
	int32 GetPublishedInputTimelineOrigin() const;
	bool ServiceInputRetirementAtBoundary() override;
	// Transitional network input is forward-only. State restore cannot leave
	// its separate filter running against a corrected/replayed component state.
	virtual bool AllowsExternalNetworkStateRestore() const { return !HasProducedInputAuthority() && !HasLegacyRemoteInputAuthority(); }
protected:
	// Shared arbitration for both native wire payloads. No live input mailbox.
	bool ClaimLegacyRemoteInputAuthority();
	bool HasLegacyRemoteInputAuthority() const { return bLegacyRemoteInputAuthorityWheeledDomainStorage().load(std::memory_order_acquire); }
	void RejectCanonicalInputFrame() { bInputFrameRejectedV2WheeledDomainStorage() = true; }
public:
	ASpeedCar* GetSpeedCarOwner() const;
	/** Used to create any physics engine information for this component */
	bool ShouldCreatePhysicsState() const override;
	virtual void OnCreatePhysicsState() override;
	/** Used to shut down and physics engine structure for this component */
	virtual void OnDestroyPhysicsState() override;
	/** Applies the Speed-specific deterministic suspension setup after Chaos creates the vehicle. */
	virtual void SetupVehicle(TUniquePtr<Chaos::FSimpleWheeledVehicle>& PVehicle) override;
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
	// Investigation hook for local deterministic probes that intentionally
	// bypass the network-alignment startup countdown.
	void SetImmediateMovementForTesting();
    /** GT witness only: both legacy network components were excluded before registration. */
    bool IsLegacyNetworkHistoryExcluded() const;
	// Mass to use for physics simulation
	float GetPhysMass() const override;
	// Center-of-mass state and its fixed local offset from the component origin.
	const FVector& GetPhysCOM() const override;
	const FVector& GetPhysCenterOfMassLocal() const override;
	// Configured COM-local hardpoint span; no live support/pitch sampling.
	float GetConfiguredWheelbaseCm() const;
	// Get array of sub-bodies owned by this component
	const TArray<USSubBody*>& GetSubBodies() const;
	// Get SubBody Config by SubBody
	SubBodyConfig GetSubBodyConfig(const USSubBody& SubBody) const;
	// overload this function to return the array of wheel sub-bodies owned by this component
	WheelSubBodyConfig GetWheelSubBodyConfig(const USWheelSubBody& SubBody) const override;

	// Get appropriate kinematic state for SubBody (e.g. wheel kinematics or hitbox kinematics from car body)
	SKinematic GetKinematicsOfSubBody(const USSubBody& SubBody, const unsigned int& NumFrame) const;
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
    USpeedWheeledComponent* AsWheeledPhysicsComponent() override { return this; }
    bool IsCarBumpInCooldown(const USpeedWheeledComponent& Victim) const;
    bool TryBeginCarBump(USpeedWheeledComponent& Victim);
    void ResetCarBumpCooldown();
    void AppendSimulationSnapshot(TArray<uint8>& OutPayload) const override;
    bool CanRestoreSimulationSnapshot(TConstArrayView<uint8> Payload) const override;
    void RestoreSimulationSnapshotPayload(TConstArrayView<uint8> Payload) override;

	void RegisterTestVelocity(const FVector& InitialVelocity);
	void RegisterTestVelocity(const FVector& InitialVelocity, const FVector& InitialAngularVelocity);
	void ApplyTestVelocity();

	bool IsAffectedByGravity() const;
	FVector GetNominalGravityAcceleration() const override { return bEnableGravityWheeledDomainStorage() ? FVector(0, 0, GravityZWheeledDomainStorage()) : FVector::ZeroVector; }
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
	/** GameThread lifecycle boundary. The worker retains only a values producer. */
	void SetFrameInputStream(std::shared_ptr<Speed::Input::FInputStream> Stream);
	/** Caller owns acknowledged physical quiescence; no accepted token may remain. */
	bool SetFrameInputStreamV2(std::shared_ptr<Speed::Input::V2::FInputStream> Stream);
	bool NeutralizeProducedInputAtBoundary();
	/** Caller has joined the actual IAmSpeed owner; never abort/unlock on GT. */
	bool NeutralizeProducedInputAfterOwnerJoined();
	// Opt-in is lifetime scoped: detaching must not silently restore a legacy
	// physical writer on the same component while cancellation is pending.
	void JoinInputWorkerBeforeStorageDestruction();
	bool HasProducedInputAuthority() const { return bProducedInputAuthorityWheeledDomainStorage().load(std::memory_order_acquire); }
	/** Opt-in generic camera input; SL keeps its independent legacy input adapter. */
	void EnableGenericCameraInput(bool bEnabled);
	bool SetHeldCameraBack(bool bBack);
	bool SetHeldCameraYaw(float Value);
	bool SetHeldCameraPitch(float Value);
	void ClearHeldCameraInput();
	void AppendPresentationSnapshot(TArray<uint8>& OutPayload) const override;
    void AppendWheelPresentationSnapshot(TArray<FSimulationWheelVisual>& Out) const override;
    /** Presentation only; false preserves existing clients and canonical bytes. */
    UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category=Presentation)
    bool bPublishWheelPresentation = false;
    /** Optional sealed V2 action slots. -1 preserves the existing camera queue. */
    UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category=Camera, meta=(ClampMin="-1",ClampMax="31"))
    int32 CanonicalCameraYawAction = -1;
    UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category=Camera, meta=(ClampMin="-1",ClampMax="31"))
    int32 CanonicalCameraPitchAction = -1;
	void OnCanonicalFramePublished(uint64 NumFrame) override;
	bool ValidateCanonicalFrameCommit(uint64 Frame) const override;
	bool CommitCanonicalFrame(uint64 Frame) noexcept override;
	void AbortCanonicalFrame(uint64 Frame, ECanonicalFrameAbortReason Reason) noexcept override;

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
    Speed::NativeObjectRead::TView<USWheelSubBody> ReadWheelSubBodiesForPhysics() const override
    {return {WheelSubBodiesWheeledDomainStorage(),UsesOriginalWheeledDomain()};}
    Speed::NativeObjectRead::TRead<UChaosVehicleWheel> ReadWheelConfigurationForPhysics(int32 Index) const
    {return {WheelsWheeledDomainStorage()[Index],UsesOriginalWheeledDomain()};}
    Speed::NativeObjectRead::TRead<UBoxSubBody> ReadHitboxForPhysics() const
    {return {HitboxSubBodyWheeledDomainStorage(),UsesOriginalWheeledDomain()};}

	// update wheel position, angular velocity etc. in Game Thread
	virtual void UpdateWheelVisuals();
	// recover wheel state from network replication
	virtual void RecoverWheelState();
	// register wheel state for network replication
	virtual void RegisterWheelState();
	void ResetWheelTransientStateForCurrentPose(bool bUseGroundContactGeometry = false);
	float GetSuspensionOffset(int WheelIndex) override;

protected:
    // A selected physical network owner can keep startup closed independently
    // of a product countdown. Legacy/offline components retain their own timer.
    virtual bool IsInitialSynchronizationComplete() const { return true; }

    // Native debug output can synchronously call arbitrary registered output
    // devices. A specialized storage owner suspends its routing around that
    // external call, after formatting immutable values from the physical cut.
    virtual void* SuspendStorageRoutingForExternalObservation() const {return nullptr;}
    virtual void ResumeStorageRoutingAfterExternalObservation(void*) const {}
    class FScopedExternalObservationRouting
    {
        const USpeedWheeledComponent& Owner;
        void* Token;
    public:
        explicit FScopedExternalObservationRouting(const USpeedWheeledComponent& In)
            :Owner(In),Token(In.SuspendStorageRoutingForExternalObservation()){}
        ~FScopedExternalObservationRouting(){Owner.ResumeStorageRoutingAfterExternalObservation(Token);}
        FScopedExternalObservationRouting(const FScopedExternalObservationRouting&)=delete;
        FScopedExternalObservationRouting& operator=(const FScopedExternalObservationRouting&)=delete;
    };
    // Legacy owns these fields directly. A private physical subtype may route
    // its own native calls to separate storage; this is not a restore lease.
    virtual SBaseGameState& BaseGameStateStorage() {return BaseGameState;}
    virtual const SBaseGameState& BaseGameStateStorage() const {return BaseGameState;}
    virtual FBasePhysicsState& BasePhysicsStateStorage() {return BasePhysicsState;}
    virtual const FBasePhysicsState& BasePhysicsStateStorage() const {return BasePhysicsState;}
    virtual FWheeledGameState& WheeledGameStateStorage() {return WheeledGameState;}
    virtual const FWheeledGameState& WheeledGameStateStorage() const {return WheeledGameState;}
    virtual FWheeledPhysicsState& WheeledPhysicsStateStorage() {return WheeledPhysicsState;}
    virtual const FWheeledPhysicsState& WheeledPhysicsStateStorage() const {return WheeledPhysicsState;}
    virtual FWheeledInputState& WheeledPhysicalInputStorage() {return WheeledPhysicalInput;}
    virtual const FWheeledInputState& WheeledPhysicalInputStorage() const {return WheeledPhysicalInput;}

public:
	bool CanMove() const;
	void RequestCanonicalSupportStartupInitialization()
	{
		bCanonicalSupportStartupInitializationRequested = true;
	}
	bool CountdownHasStarted() const;
	void StartConfrontationInSec(const float& TimeSec);
	void StartConfrontationLocal(const float& TimeSec);
	UFUNCTION(reliable, NetMulticast)
	void StartConfrontationMulti(const float& TimeSec);
	void StartTestWithVelocity(const FVector& InitialVelocity);
	void StartTestWithVelocity(const FVector& InitialVelocity, const FVector& InitialAngularVelocity);
	// Restores the handbrake phase when a deterministic scenario starts mid-powerslide.
	void InitializeWheelFrameHandbrakeState(float InitialValue, float InitialActiveTime = 0.0f);
	void StartTestWithVelocityLocal(const FVector& InitialVelocity);
	void StartTestWithVelocityLocal(const FVector& InitialVelocity, const FVector& InitialAngularVelocity);
	UFUNCTION(reliable, NetMulticast)
	void StartTestWithVelocityMulti(const FVector& InitialVelocity);
	UFUNCTION(reliable, NetMulticast)
	void StartTestWithVelocityAndAngularVelocityMulti(const FVector& InitialVelocity, const FVector& InitialAngularVelocity);
	void SetCannotMove();
	void SetCannotMoveLocal();
	UFUNCTION(reliable, NetMulticast)
	void SetCannotMoveMulti();

	// bump methods
	void DemoedBy(ASpeedCar* otherCar);

	void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	// Returns true if the movement is currently frozen (e.g. due to the game being paused)
	bool IsFrozen() const override;
	bool NeutralizeCanonicalInputAtBoundary() override;
	bool InstallCanonicalInput(uint64 Frame,
		std::shared_ptr<const Speed::Input::V2::FOwnerInputSnapshot> Snapshot) override;
	const std::shared_ptr<const Speed::Input::V2::FOwnerInputSnapshot>& GetCanonicalInputSnapshot() const { return CanonicalInputSnapshotWheeledDomainStorage(); }
	bool PrepareCanonicalInputs(const FCanonicalFrameContext& Context) override;
	void PrepareCanonicalFrame(const FCanonicalFrameContext& Context) override;
	// Set whether the movement is currently frozen (e.g. due to the game being paused)
	void SetIsFrozen(bool bFrozen) override;
private:
	void AsyncPhysicsTickComponent(float DeltaTime, float SimTime) override final;
	void PhysicsTick(const float& DeltaTime, const float& SimTime);
	void PreparePhysicsFrame(const float& DeltaTime, const float& SimTime);

	void UpdateFrameState();
	void UpdateNumFrame(const float& SimTime);
	bool GetBaseState(const int32& LocalFrame, FBasePhysicsState& OutState) const;
	bool GetWheeledState(const int32& LocalFrame, FWheeledPhysicsState& OutState) const;

	void HandleGravity();
	void HandleDamping(const float& delta);
	void HandleGroundFriction(const float& delta);
	void ApplyWheelFrameLateralFriction(const float& delta);
	void HandleSuspension(const float& delta);

	void applyAccelerationConstraint(const float& delta);
	void applyAngularAccelerationConstraint(const float& delta);

	// Timer method to handle countdown
	void HandleCountdownTimer();
	// Handle wheel timers
	void HandleWheelTimers();
	// End Timer methods
protected:
    USpeedWheeledComponent(const FObjectInitializer& ObjectInitializer,ESpeedWheeledNetworkConstruction Policy);
    const ESpeedWheeledNetworkConstruction NetworkConstruction=ESpeedWheeledNetworkConstruction::Legacy;
	virtual void InitChaosVehicle();
	void InitNetwork();
	void SetSubBodies(const TArray<USSubBody*>& NewSubBodies);

	void SetEngineFPS(const unsigned int& FPS);
	float GetEngineFPS() const;
	int32 GetSinceCanMoveFrame() const;
	bool CanBypassCanonicalSupportContactWarmup() const override;
	bool CanPreserveCanonicalSupportNormalRotation() const override;
	unsigned int NbFramesSinceCanMove() const;
	// Get the current frame number for this component (e.g. to be used for network replication)
	unsigned int GetCurrentFrame() const;
	FVector GetAllowedAngularVelocity() const { return WheeledPhysicsStateStorage().AllowedAngularVelocity; }
	void SetAllowedAngularVelocity(const FVector& NewAllowedAngularVelocity) { WheeledPhysicsStateStorage().AllowedAngularVelocity = NewAllowedAngularVelocity; }

	// Update Inputs
	virtual void UpdateInputs();
	// The physical owner passes the exact immutable frame from its sole Consume.
	// Derived acceptance runs before any completed-input publication. False cancels
	// this source; no live/queued fallback is permitted for that physical frame.
	virtual bool ValidateProducedInputFrame(const Speed::Input::FInputFrame&) const { return true; }
	virtual bool ApplyProducedInputFrame(const Speed::Input::FInputFrame&) { return true; }
	virtual bool ValidateProducedInputFrameV2(const Speed::Input::V2::FInputFrame&) const { return true; }
	virtual bool ApplyProducedInputFrameV2(const Speed::Input::V2::FInputFrame& Frame);
	// Simulation-lane cancellation, never a synthetic gameplay release event.
	virtual void ResetProducedInputState() {}
	virtual bool PublishInputCancellationAtBoundary() { return true; }
	/** Boundary-only inspection, without acquiring a possibly prepared owner lock. */
	virtual bool CanNeutralizeProducedInputStateAtBoundary() const { return true; }
	// Owner-lane only; helpers do not certify an unknown derived adapter.
	void AppendReplayWheeledFields(Speed::Replay::FFieldWriter& Writer) const;
	bool IsProducedInputFrameOwned() const { return bProducedInputFrameOwnedWheeledDomainStorage(); }
	// Vehicle presets may deterministically put an unchanged support manifold to
	// sleep before frame forces are accumulated.
	virtual void UpdateSupportForceSleepState() {}
	virtual bool DisableGravityThisFrame() const { return false; }

public:
	// Legacy fixture admission flag; never writes physical values.
	void SetTestInputOverrideEnabled(bool bEnabled);
	bool IsTestInputOverrideEnabled() const { return bTestInputOverrideEnabledWheeledDomainStorage().load(std::memory_order_acquire); }
	// Configuration access for native deterministic vehicle profiles.
	UBoxSubBody* GetHitboxSubBodyForConfiguration() const { return ReadHitboxForPhysics().Get(); }

protected:
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
	// Vehicle presets may snapshot the rigid/wheel state immediately before the
	// suspension update without replacing the component tick ordering.
	virtual void BeforeSuspensionUpdate(float DeltaTime) {}
	// Optional vehicle policy. False preserves the generic application exactly.
	virtual bool TryApplyWheelSuspensionForce(const USWheelSubBody& Wheel,
        float Load) { return false; }
	virtual void HandlePhysicsAutoRecover();
	virtual void HandleAcceleration();
	virtual void HandleSteering(const float& delta);
	virtual bool DisableSuspensionThisFrame() const;
	// Handle Timers
	virtual void HandleTimers();

	float ComputeThrottleAccel() const;
	float ComputeBrakeAccel() const;
	float ComputeDriveAccel() const;
	void ApplyDriveAcceleration(float Accel);
	virtual bool IsAcceleratingForWheelFriction() const;
	virtual bool HasDriveInputForGroundFriction() const;
	virtual bool IsHandbrakingForWheelFriction() const;
	virtual bool IsHandbrakingForwardForWheelFriction() const;
	virtual bool TryApplyWheelFramePassiveLongitudinalFriction(
		float DeltaTime,
		float MaxTotalDeceleration);
	virtual bool TryApplyCustomWheelFrameLateralFriction(float DeltaTime);
	virtual FVector GetWheelFrictionApplicationPoint(
		const USWheelSubBody& Wheel,
		const FVector& ContactPoint,
		const FVector& SurfaceNormal) const;

	virtual void ApplyAccelKinematicsConstraint(const float& delta);
	virtual void TagStateHistoryProxyRole();
	virtual void ApplyNetworkCorrection(const float& delta);
	virtual void QuantizePhysicalState();

	TArray<SWheelGroundContact>& GetPendingWheelContacts() override;
	const TArray<SWheelGroundContact>& GetPendingWheelContacts() const override;
	float GetWheelContactNormalVelocityTimeConstant() const override;

	void SetGroundState();
	void SetAirState();
	/** bWasInAir is the pre-update air-to-ground edge; default behavior remains empty. */
	virtual void SetGroundStatePrv(bool bWasInAir);
	virtual void SetAirStatePrv();

	// =========== Acceleration and steering functions ===========
	// return the acceleration to apply in cm/s^2 based on the throttle/brake input and whether we want to move forward or backward
	virtual float ComputeAccel(const float& InputValue, bool wantToMoveForward) const;
	virtual float ComputeAccelFromSignedThrottle(float SignedThrottle) const;
	// return the maximum speed in cm/s that the component can reach based on steering input with the given input (value between -1 and 1)
	virtual float GetSteeringSpeedCap() const;
	// return the maximum throttle speed in cm/s that the component can reach based on whether it is on the ground or not
	virtual float GetMaxThrottleSpeed() const;

	// return true iff the component is not allowed to steer
	virtual bool NoSteeringAllowed() const;
	virtual bool MustReduceAllowedVelocityForSteering() const;
	virtual float GetSteeringYawCorrectionTimeConstant() const;
	virtual float ComputeSteeringTargetYawRate(float InSteeringInput, float InForwardVelocity, float InAbsTargetYawRate) const;
	virtual float GetSteeringAllowedYawClamp(float InAbsTargetYawRate) const;
	virtual float ComputeSteeringRadius(const float& ForwardVelocity, const float& AbsSteeringInput) const;
	void UpdateWheelFrameHandbrakeValue(const float& Delta);
	float ComputeWheelFrameSteerAngle(const float& AbsForwardSpeed, const float& TargetRadius, const float& Wheelbase) const;
	float AdaptWheelFrameSteerAngleToReferenceGeometry(float ReferenceSteerAngle) const;
	bool TryComputeWheelFrameNormalSteeringInputFraction(float& OutFraction) const;
	bool TryProjectCanonicalWheelSupportPose() override;
	bool UsesRetainedHandbrakeDriveModel() const;
	float ComputeFullHandbrakeGroundEngineAccel() const;
	FVector ComputeTurningForce(const float& SteeringInput, const float& UpDot,
		const float& ForwardVelocity, const float& DesiredRadius, const FVector& WheelRight) const;
	virtual void DampenAngularVelocity(const float& DampingFactor, const FVector& VelocityToDampen, const float& delta);
	virtual void DampenAirAngularVelocity(const FVector& VelocityToDampen, const float& delta);
	void SetDampenedAirAngularVelocity(const FVector& TargetAngularVelocity, const FVector& VelocityToSetNow, const float& delta);

	const TObjectPtr<UBoxSubBody>& GetHitboxSubBody() const { return HitboxSubBodyWheeledDomainStorage(); }

	/** Read current state for simulation */
	void UpdateState(float DeltaTime) override;

	// Netcode methods
	virtual void RecordPhysicsState();
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
	// Response time, in seconds, used to absorb inward normal velocity on a new
	// wheel contact. Vehicle presets may tune it independently of physics rate.
	UPROPERTY(BlueprintReadWrite, Category = Physics, EditDefaultsOnly,
		meta = (ClampMin = "0.000001", UIMin = "0.000001", Units = "s"))
	float WheelContactNormalVelocityTimeConstant = 0.0075f;
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
	// Speed-to-throttle scale curve. X is absolute forward speed in cm/s, Y is acceleration scale.
	UPROPERTY(BlueprintReadWrite, Category = Acceleration, EditDefaultsOnly)
	TArray<FVector2D> DriveSpeedAccelScaleCurve;
	// When enabled, longitudinal ground friction is handled by the drive/brake model instead of HandleGroundFriction.
	UPROPERTY(BlueprintReadWrite, Category = Acceleration, EditDefaultsOnly)
	bool bUseDriveModelLongitudinalFriction = true;
	// If throttle magnitude is below this value, the car coasts instead of applying engine force.
	UPROPERTY(BlueprintReadWrite, Category = Acceleration, EditDefaultsOnly)
	float ThrottleDeadzone = 0.001f;
	// Coasting brake scale applied when neither throttle nor brake is pressed.
	UPROPERTY(BlueprintReadWrite, Category = Acceleration, EditDefaultsOnly)
	float CoastingBrakeFactor = 0.0f;
	// Below this speed, coasting applies full brake to settle the car.
	UPROPERTY(BlueprintReadWrite, Category = Acceleration, EditDefaultsOnly)
	float StoppingForwardSpeed = 25.0f;
	// Above this speed, applying opposite throttle/brake suppresses engine throttle and applies brake.
	UPROPERTY(BlueprintReadWrite, Category = Acceleration, EditDefaultsOnly)
	float BrakingNoThrottleSpeedThreshold = 0.01f;
	// Speed-to-max-steer-angle curve. X is absolute forward speed in cm/s, Y is wheel steer angle in radians.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	TArray<FVector2D> SteerAngleFromSpeedCurve;
	// Speed-to-max-steer-angle curve blended in when handbraking/powersliding.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	TArray<FVector2D> HandbrakeSteerAngleFromSpeedCurve;
	// When enabled, lateral ground friction is computed in the wheel frame, so steered wheels naturally slow the car in turns.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	bool bUseWheelFrameLateralFriction = true;
	// Legacy steering helper that applies explicit lateral forces in addition to wheel-frame friction.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	bool bUseExplicitSteeringForce = false;
	// Legacy steering helper that drives AllowedSideVelocity/AllowedAngularVelocity directly.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	bool bUseLegacySteeringVelocityState = false;
	// Global multiplier for the wheel-frame lateral friction model.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	float WheelFrameLateralFrictionScale = 1.0f;
	// Optional physical-data normalization for a shared handling reference.
	// Authored per component; one preserves existing tire response exactly.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly, meta = (ClampMin = "0.0"))
	float WheelFrameLateralResponseScale = 1.0f;
	// Optional axle allocation derived from physical tire-response moments.
	// One preserves the shared scalar model; real contact arms/inertia stay intact.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly, meta = (ClampMin = "0.0"))
	float WheelFrameFrontLateralResponseScale = 1.0f;
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly, meta = (ClampMin = "0.0"))
	float WheelFrameRearLateralResponseScale = 1.0f;
	// Physical axle allocation inside the existing handbrake tire blend only.
	// Axles stay physical when the directional front/rear tire factors swap.
	// Identity defaults preserve the common law and normal steering exactly.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly, meta = (ClampMin = "0.0001", ClampMax = "16.0"))
	float WheelFrameHandbrakeFrontLateralResponseScale = 1.0f;
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly, meta = (ClampMin = "0.0001", ClampMax = "16.0"))
	float WheelFrameHandbrakeRearLateralResponseScale = 1.0f;
	// Optional reverse-direction tire response, still indexed by physical axle.
	// Zero inherits the primary response and preserves existing profiles exactly.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly, meta = (ClampMin = "0.0", ClampMax = "16.0"))
	float WheelFrameHandbrakeReverseFrontLateralResponseScale = 0.0f;
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly, meta = (ClampMin = "0.0", ClampMax = "16.0"))
	float WheelFrameHandbrakeReverseRearLateralResponseScale = 0.0f;
	// Response time for the wheel-frame bilateral lateral impulse. This keeps tire response stable across physics tick rates.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	float WheelFrameLateralImpulseTimeConstant = 0.037345f;
	// Lateral friction retained at zero steering input, blended to full authority as steering increases.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	float WheelFrameZeroSteerLateralFrictionScale = 1.0f;
	// Optional tire self-alignment under high lateral slip with no steering.
	// Zero disables it so IAmSpeed retains a neutral generic default.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	float WheelFrameUnsteeredAligningYawMaxRate = 0.0f;
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	float WheelFrameUnsteeredAligningYawFullSlipSpeed = 600.0f;
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	float WheelFrameUnsteeredAligningYawMinSlipSpeed = 0.0f;
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	float WheelFrameUnsteeredAligningYawTimeConstant = 0.025f;
	// Rear lateral friction at full normal steering, blended from one by steering input magnitude.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	float WheelFrameSteeringRearLateralFrictionScale = 1.0f;
	// Optional safety cap for each wheel-frame lateral impulse.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	bool bClampWheelFrameLateralImpulse = false;
	// Additional impulse relaxation applied to wheel-frame lateral friction while handbraking.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	float WheelFrameHandbrakeLateralImpulseScale = 1.0f;
	// Optional absolute handbrake impulse scale by lateral slip ratio. Empty preserves the scalar above.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	TArray<FVector2D> HandbrakeLateralImpulseScaleCurve;
	// Longitudinal scrub generated by lateral tire force while steering.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	float WheelFrameLongitudinalScrubScale = 0.0f;
	// Curvature multiplier for the wheel-frame steering geometry.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	float WheelFrameSteeringCurvatureScale = 1.0f;
	// Shared handling curves describe one reference geometry. Zero preserves
	// the existing per-vehicle curve path; positive uses actual hardpoints.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly, meta = (ClampMin = "0.0"))
	float WheelFrameSteeringReferenceWheelbaseCm = 0.0f;
	// Scales the forward/backward component produced by wheel-frame lateral friction.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	float WheelFrameLateralForwardLossScale = 1.0f;
	// Speed-to-forward-loss scale curve for wheel-frame lateral friction.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	TArray<FVector2D> LateralForwardLossScaleCurve;
	// Slip-to-forward-loss scale curve for wheel-frame lateral friction. X uses the lateral slip ratio.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	TArray<FVector2D> LateralForwardLossSlipScaleCurve;
	// Speed-to-forward-loss scale curve used while steering without handbrake.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	TArray<FVector2D> SteeringForwardLossSpeedScaleCurve;
	// Lateral friction scale by slip ratio. X is lateral slip / (forward slip + lateral slip), Y is friction scale.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	TArray<FVector2D> LateralFrictionScaleCurve;
	// Longitudinal friction scale by slip ratio. X is lateral slip / (forward slip + lateral slip), Y is friction scale.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	TArray<FVector2D> LongitudinalFrictionScaleCurve;
	// Friction scale by contact normal Z when the vehicle is not accelerating.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	TArray<FVector2D> NonAcceleratingFrictionScaleCurve;

	// Lateral friction factor blended in when handbraking/powersliding for front wheels. X is lateral slip ratio, Y is friction factor.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	TArray<FVector2D> HandbrakeFrontLateralFrictionFactorCurve;
	// Lateral friction factor blended in when handbraking/powersliding for rear wheels. X is lateral slip ratio, Y is friction factor.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	TArray<FVector2D> HandbrakeRearLateralFrictionFactorCurve;
	// Lateral friction scale blended in when handbraking/powersliding for front wheels. X is absolute forward speed, Y is scale.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	TArray<FVector2D> HandbrakeFrontLateralFrictionSpeedScaleCurve;
	// Lateral friction scale blended in when handbraking/powersliding for rear wheels. X is absolute forward speed, Y is scale.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	TArray<FVector2D> HandbrakeRearLateralFrictionSpeedScaleCurve;
	// Extra travel-rear grip release at full steering while handbraking. X is planar ground speed, Y is the full-steering scale.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	TArray<FVector2D> HandbrakeSteeringRearLateralFrictionScaleCurve;
	// Scales the additional steering-driven travel-rear grip release while handbraking. X is lateral slip ratio, Y is the release alpha.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	TArray<FVector2D> HandbrakeSteeringRearLateralFrictionSlipScaleCurve;
	// Scales the handbrake/powerslide front-rear friction imbalance by lateral slip ratio.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	TArray<FVector2D> HandbrakeLowSlipYawCorrectionScaleCurve;

	// Longitudinal friction factor blended in when handbraking/powersliding. X is lateral slip ratio, Y is friction factor.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	TArray<FVector2D> HandbrakeLongitudinalFrictionFactorCurve;
	// Duration in seconds where handbrake can temporarily release the travel-rear lateral friction to start a slide.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	float WheelFrameHandbrakeEntryDuration = 0.0f;
	// Minimum absolute forward speed for the handbrake entry release to start blending in.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	float WheelFrameHandbrakeEntryMinSpeed = 0.0f;
	// Absolute forward speed where the handbrake entry release reaches full speed weight.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	float WheelFrameHandbrakeEntryFullSpeed = 1400.0f;
	// Minimum absolute steering input for the handbrake entry release to start blending in.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	float WheelFrameHandbrakeEntryMinSteering = 0.8f;
	// Temporary multiplier applied to the travel-rear lateral friction while handbrake entry release is active.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	float WheelFrameHandbrakeEntryRearReleaseScale = 1.0f;
	// Rate at which the generic wheel-frame handbrake value rises toward 1.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	float WheelFrameHandbrakeRiseRate = 5.0f;
	// Rate at which the generic wheel-frame handbrake value falls toward 0.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	float WheelFrameHandbrakeFallRate = 2.0f;
	// Below this point-speed slip, the lateral friction curve input stays at 0.
	UPROPERTY(BlueprintReadWrite, Category = Steering, EditDefaultsOnly)
	float LateralFrictionSlipThreshold = 5.0f;
	float WheelFrameHandbrakeValue = 0.0f;
	float WheelFrameHandbrakeActiveTime = 0.0f;
	int32 LastWheelFrameHandbrakeUpdateFrame = INDEX_NONE;

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
	float PosCorrDeadzone = 0.02f;
	// Deadzone for the correction to apply on the velocity in cm/s (if the error is below this threshold, no correction will be applied)
	UPROPERTY(BlueprintReadWrite, Category = BaseNetcode, EditDefaultsOnly,
		meta = (ClampMin = "0.0", UIMin = "0.0"))
	float VelCorrDeadzone = 0.02f;
	// Deadzone for the correction to apply on the angular velocity in rad/s (if the error is below this threshold, no correction will be applied)
	UPROPERTY(BlueprintReadWrite, Category = BaseNetcode, EditDefaultsOnly,
		meta = (ClampMin = "0.0", UIMin = "0.0"))
	float AngVelCorrDeadzone = 0.0002f;
	// Deadzone for the correction to apply on the rotation in deg (if the error is below this threshold, no correction will be applied)
	UPROPERTY(BlueprintReadWrite, Category = BaseNetcode, EditDefaultsOnly,
		meta = (ClampMin = "0.0", UIMin = "0.0"))
	float RotCorrDeadzone = 0.01f;

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
		VehicleSimulationPTWheeledDomainStorage() = MakeUnique<USpeedSimulation>();
		VehicleSimulationPT->RigidHandle = nullptr;
		static_cast<USpeedSimulation*>(VehicleSimulationPTWheeledDomainStorage().Get())->SpeedWheeledComponent = this;
		SkySimulationWheeledDomainStorage() = static_cast<USpeedSimulation*>(VehicleSimulationPTWheeledDomainStorage().Get());

		return UChaosVehicleMovementComponent::CreatePhysicsVehicle();
	}

	// Shared fixed-COM authoring rule for initial setup and BeginPlay.
	void ResolveFixedCenterOfMassFromConfiguration();
    // Shared value/body operation only. The public facade retains its original
    // Engine tick/registration path; the private factory supplies real captured
    // actor pose and explicitly registers its original bridge separately.
    class FLegacyOwnerBirthAccess;class FNativeOwnerBirthAccess;class FReferenceOwnerBirthAccess;
    struct FReferenceOwnerBirthObservation
    {
        uint32 BoundaryChecksReturned=0,TwoNullEntriesReturned=0,InitializersReturned=0;
    };
    template<class TAccess> void SetOwnerBody(TAccess& Storage,AActor* NewOwner);
    void InitializeReferenceOwnerBirth(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,
        const Speed::Netcode::Prediction::FPrivateReferenceOwnerBirth& Proof,AActor* ActualActor,
        FReferenceOwnerBirthObservation* Observation=nullptr);
    struct FNativeOwnerBirthObservation
    {
        enum class EMode : uint8 { CountOnly,StoredTensorFixtureAndAbort };
        EMode Mode=EMode::CountOnly; // diagnostic choice only, never admission
        uint32 InitializersReturned=0,InitialNullParents=0,Refusals=0;
        bool AllRefusalsBeforeWrite=false,PositiveEntryReturned=false,PoseAndCOMMatch=false;
        bool ClearKeepsCanonicalConfigurations=false,StoredTensorFixtureInstalled=false;
        bool StoredChassisTensorMatch=false,StoredTensorDistinguished=false,ChildrenMatch=false;
        bool WheelVelocityOrderDistinguished=false; // actual wheel matches its original FP order and differs from base-only oracle
    };
    template<class TAccess> void ResolveFixedCenterOfMassBody(TAccess& Storage);
    template<class TAccess> void InitializeBoundSubBodiesForPhysicalOwnerBody(TAccess& Storage);
    void InitializeBoundSubBodiesForPhysicalOwner();
    void InitializeOriginalOwnerFromCapturedPose(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,
        AActor* OriginalOwner,const FQuat& OriginalRotation,const FVector& OriginalLocation,
        const TWeakObjectPtr<UBoxSubBody>& ActualHitboxBirth,
        TConstArrayView<TWeakObjectPtr<USWheelSubBody>> ActualBodyBirths,
        TConstArrayView<TWeakObjectPtr<UChaosVehicleWheel>> ActualConfigurationBirths,
        FNativeOwnerBirthObservation* Observation=nullptr);
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    friend class FIASLegacyFirstOwnerBirthTraversal;
    bool ProbeNativeFirstOwnerBirthForDiagnostics(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,
        AActor* ActualActor,const FQuat& Rotation,const FVector& Location,
        const TWeakObjectPtr<UBoxSubBody>& HitboxBirth,TConstArrayView<TWeakObjectPtr<USWheelSubBody>> BodyBirths,
        TConstArrayView<TWeakObjectPtr<UChaosVehicleWheel>> ConfigurationBirths,
        TConstArrayView<TWeakObjectPtr<UChaosVehicleWheel>> ActualReferenceBirths,
        USpeedWheeledComponent* ForeignComponent,AActor* ForeignActor,
        const TWeakObjectPtr<UBoxSubBody>& ForeignHitbox,TConstArrayView<TWeakObjectPtr<USWheelSubBody>> ForeignBodyBirths,
        TConstArrayView<TWeakObjectPtr<UChaosVehicleWheel>> ForeignConfigurationBirths,
        USpeedSimulation* ForeignSimulation,const SKinematic& ForeignOriginalKinematic,FNativeOwnerBirthObservation& Out);
    bool ProbeNativeFirstOwnerStepRefusalForDiagnostics(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,
        AActor* ActualActor,const TWeakObjectPtr<UBoxSubBody>& HitboxBirth,
        TConstArrayView<TWeakObjectPtr<USWheelSubBody>> BodyBirths,
        TConstArrayView<TWeakObjectPtr<UChaosVehicleWheel>> ConfigurationBirths);
#endif


	void SetupSpeedSuspension(TUniquePtr<Chaos::FSimpleWheeledVehicle>& PVehicle);
	// Rebind wheel sub-bodies whenever Chaos recreates the vehicle. The
	// suspension objects live inside PVehicle and their addresses are not stable
	// across physics-state teardown/recreation.
	void BindWheelSimulationPointers();
	void BindWheelSimulationPointers(Chaos::FSimpleWheeledVehicle* PVehicle);
    class FLegacyWheelBindingAccess;class FNativeWheelBindingAccess;class FReferenceWheelBindingAccess;
    template<class TAccess> void BindWheelSimulationPointersBody(TAccess& Storage,Chaos::FSimpleWheeledVehicle* PVehicle);
    void BindReferenceWheelSimulationPointersBirth(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,
        const Speed::Netcode::Prediction::FPrivateReferenceWheelBindingBirth& Proof,
        Chaos::FSimpleWheeledVehicle* RequestedVehicle,
        TConstArrayView<TWeakObjectPtr<USWheelSubBody>> RequestedBodies,
        TConstArrayView<TWeakObjectPtr<UChaosVehicleWheel>> RequestedConfigurations);
    void BindNativeWheelSimulationPointersBirth(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,
        Chaos::FSimpleWheeledVehicle* OriginalAllocation,
        TConstArrayView<TWeakObjectPtr<USWheelSubBody>> ActualBodyBirths,
        TConstArrayView<TWeakObjectPtr<UChaosVehicleWheel>> ActualConfigurationBirths);

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    friend class FIASLegacyWheelSimulationBindings;
    struct FNativeWheelBindingDiagnostics
    {
        uint32 Refusals=0,PositiveEntryReturns=0;
        bool AllRefusalsBeforeWrite=false,PositiveValuesMatch=false,OverridePredicatesMatch=false;
    };
    bool ProbeReferenceWheelBindingsBirthForDiagnostics(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,
        const Speed::Netcode::Prediction::FPrivateReferenceWheelBindingBirth& Proof,
        USpeedWheeledComponent& ForeignComponent,USWheelSubBody* ForeignBody,
        Chaos::FSimpleWheeledVehicle* ForeignVehicle,Chaos::FSimpleWheelSim* ForeignWheel,
        Chaos::FSimpleSuspensionSim* ForeignSuspension,UChaosVehicleWheel* ForeignConfiguration,
        FNativeWheelBindingDiagnostics& Out);
    bool ProbeNativeWheelBindingsBirthForDiagnostics(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,
        Chaos::FSimpleWheeledVehicle* OriginalAllocation,
        TConstArrayView<TWeakObjectPtr<USWheelSubBody>> BodyBirths,
        TConstArrayView<TWeakObjectPtr<UChaosVehicleWheel>> ConfigurationBirths,
        USWheelSubBody* ForeignBody,ISpeedComponent* ForeignParent,
        Chaos::FSimpleWheeledVehicle* ForeignVehicle,Chaos::FSimpleWheelSim* ForeignWheel,
        Chaos::FSimpleSuspensionSim* ForeignSuspension,UChaosVehicleWheel* ForeignConfiguration,
        FNativeWheelBindingDiagnostics& Out);
    bool ProbeNativeWheelBindingStepRefusalForDiagnostics(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,
        Chaos::FSimpleWheeledVehicle* OriginalAllocation,
        TConstArrayView<TWeakObjectPtr<USWheelSubBody>> BodyBirths,
        TConstArrayView<TWeakObjectPtr<UChaosVehicleWheel>> ConfigurationBirths);
#endif
	void ClearWheelSimulationPointers();
	/** Latches game-thread driving inputs exactly once at a physics-frame boundary. */
	bool ConsumeProducedWheeledInputs(uint64 CanonicalFrame);
private:
	struct FCompletedWheeledInput
	{
		FWheeledInputState Requested;
		uint32 LocalFrame = 0;
		int32 SinceCanMove = INDEX_NONE;
	};	struct FPendingCameraInputCommand
	{
		int32 ActivationFrame = INDEX_NONE;
		FSpeedCarCameraPhysicalInput Input;
	};	using FLegacyWheeledIngress = Speed::Input::TLegacyRemotePreSlewIngress<FWheeledInputState>;
	using FLegacyWheeledOwner = Speed::Input::TLegacyRemotePreSlewOwner<FWheeledInputState>;
    class FLegacyInertiaAccess;class FNativeInertiaAccess;
    template<class TAccess> static FMatrix ComputeWorldInvInertiaBody(TAccess& Storage);
    static FMatrix ComputeNativeComponentInvInertia(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent& Component);
    struct FOriginalWheeledDomain
    {
        bool bPublishWheelPresentation= false;
        int32 CanonicalCameraYawAction= -1;
        int32 CanonicalCameraPitchAction= -1;
        TObjectPtr<UNetworkPhysicsSettingsDataAsset> NetDataAsset= nullptr;
        float TimeBeforeCanMove= 0.2f;
        float PhysMass= 1000.0f;
        bool bEnableGravity= true;
        float GravityZ= -980.0f;
        float PhysMaxSpeed= 1e4;
        float PhysMaxAngularSpeed= 4.f;
        float PhysDamping= 0.0f;
        FVector CenterOfMass= FVector::ZeroVector;
        float MaxThrottleSpeed= 7000.0f;
        FVector GroundFriction= FVector(510.0, 6120.0, 0.0);
        float WheelContactNormalVelocityTimeConstant= 0.0075f;
        float GroundAngularDamping= 5.0f;
        float RollDragCoeff= 5.0f;
        float PitchDragCoeff= 3.0f;
        float YawDragCoeff= 2.0f;
        float AirForwardThrottle= 0.0f;
        float AirBackwardThrottle= 0.0f;
        float ThrottleAccel= 2000.0f;
        float SubSteeringAccel= 2000.0f;
        float BrakeAccel= 4000.0f;
        TArray<FVector2D> DriveSpeedAccelScaleCurve{};
        bool bUseDriveModelLongitudinalFriction= true;
        float ThrottleDeadzone= 0.001f;
        float CoastingBrakeFactor= 0.0f;
        float StoppingForwardSpeed= 25.0f;
        float BrakingNoThrottleSpeedThreshold= 0.01f;
        TArray<FVector2D> SteerAngleFromSpeedCurve{};
        TArray<FVector2D> HandbrakeSteerAngleFromSpeedCurve{};
        bool bUseWheelFrameLateralFriction= true;
        bool bUseExplicitSteeringForce= false;
        bool bUseLegacySteeringVelocityState= false;
        float WheelFrameLateralFrictionScale= 1.0f;
        float WheelFrameLateralResponseScale= 1.0f;
        float WheelFrameFrontLateralResponseScale= 1.0f;
        float WheelFrameRearLateralResponseScale= 1.0f;
        float WheelFrameHandbrakeFrontLateralResponseScale= 1.0f;
        float WheelFrameHandbrakeRearLateralResponseScale= 1.0f;
        float WheelFrameHandbrakeReverseFrontLateralResponseScale= 0.0f;
        float WheelFrameHandbrakeReverseRearLateralResponseScale= 0.0f;
        float WheelFrameLateralImpulseTimeConstant= 0.037345f;
        float WheelFrameZeroSteerLateralFrictionScale= 1.0f;
        float WheelFrameUnsteeredAligningYawMaxRate= 0.0f;
        float WheelFrameUnsteeredAligningYawFullSlipSpeed= 600.0f;
        float WheelFrameUnsteeredAligningYawMinSlipSpeed= 0.0f;
        float WheelFrameUnsteeredAligningYawTimeConstant= 0.025f;
        float WheelFrameSteeringRearLateralFrictionScale= 1.0f;
        bool bClampWheelFrameLateralImpulse= false;
        float WheelFrameHandbrakeLateralImpulseScale= 1.0f;
        TArray<FVector2D> HandbrakeLateralImpulseScaleCurve{};
        float WheelFrameLongitudinalScrubScale= 0.0f;
        float WheelFrameSteeringCurvatureScale= 1.0f;
        float WheelFrameSteeringReferenceWheelbaseCm= 0.0f;
        float WheelFrameLateralForwardLossScale= 1.0f;
        TArray<FVector2D> LateralForwardLossScaleCurve{};
        TArray<FVector2D> LateralForwardLossSlipScaleCurve{};
        TArray<FVector2D> SteeringForwardLossSpeedScaleCurve{};
        TArray<FVector2D> LateralFrictionScaleCurve{};
        TArray<FVector2D> LongitudinalFrictionScaleCurve{};
        TArray<FVector2D> NonAcceleratingFrictionScaleCurve{};
        TArray<FVector2D> HandbrakeFrontLateralFrictionFactorCurve{};
        TArray<FVector2D> HandbrakeRearLateralFrictionFactorCurve{};
        TArray<FVector2D> HandbrakeFrontLateralFrictionSpeedScaleCurve{};
        TArray<FVector2D> HandbrakeRearLateralFrictionSpeedScaleCurve{};
        TArray<FVector2D> HandbrakeSteeringRearLateralFrictionScaleCurve{};
        TArray<FVector2D> HandbrakeSteeringRearLateralFrictionSlipScaleCurve{};
        TArray<FVector2D> HandbrakeLowSlipYawCorrectionScaleCurve{};
        TArray<FVector2D> HandbrakeLongitudinalFrictionFactorCurve{};
        float WheelFrameHandbrakeEntryDuration= 0.0f;
        float WheelFrameHandbrakeEntryMinSpeed= 0.0f;
        float WheelFrameHandbrakeEntryFullSpeed= 1400.0f;
        float WheelFrameHandbrakeEntryMinSteering= 0.8f;
        float WheelFrameHandbrakeEntryRearReleaseScale= 1.0f;
        float WheelFrameHandbrakeRiseRate= 5.0f;
        float WheelFrameHandbrakeFallRate= 2.0f;
        float LateralFrictionSlipThreshold= 5.0f;
        float WheelFrameHandbrakeValue= 0.0f;
        float WheelFrameHandbrakeActiveTime= 0.0f;
        int32 LastWheelFrameHandbrakeUpdateFrame= INDEX_NONE;
        float PosStabilityMultiplier= 0.4f;
        float VelStabilityMultiplier= 0.8f;
        float RotStabilityMultiplier= 0.2f;
        float AngVelStabilityMultiplier= 0.8f;
        float MaxPosCorrectionPerSecond= 600.f;
        float MaxVelCorrectionPerSecond= 3600.f;
        float MaxAngVelCorrectionPerSecond= 9.f;
        float MaxRotCorrectionPerSecond= 180.f;
        float PosCorrDeadzone= 0.02f;
        float VelCorrDeadzone= 0.02f;
        float AngVelCorrDeadzone= 0.0002f;
        float RotCorrDeadzone= 0.01f;
        TObjectPtr<UBoxSubBody> HitboxSubBody= nullptr;
        TArray< TObjectPtr<USWheelSubBody>> WheelSubBodies{};
        TObjectPtr<UNetworkPhysicsSettingsComponent> SNetworkSettings= nullptr;
        USpeedSimulation* SkySimulation= nullptr;
        ASpeedCar* SpeedCarOwner= nullptr;
        FCriticalSection FrameInputProducerMutex{};
        std::shared_ptr<Speed::Input::FInputStream> FrameInputStream{};
        std::shared_ptr<Speed::Input::FInputStream> ConsumedFrameInputStream{};
        std::shared_ptr<Speed::Input::V2::FInputStream> FrameInputStreamV2{};
        std::shared_ptr<Speed::Input::V2::FInputStream> ConsumedFrameInputStreamV2{};
        std::optional<Speed::Input::V2::FReservationToken> InputReservationV2{};
        std::optional<uint64> ReservedInputFrameV2{};
        bool bInputFrameRejectedV2= false;
        bool bResetProducedWheeledInputs= false;
        bool bProducedInputFrameOwned= false;
        std::atomic<bool> bProducedInputAuthority{false};
        std::atomic<bool> bLegacyRemoteInputAuthority{false};
        FCriticalSection LegacyWheeledIngressMutex{};
        std::shared_ptr<FLegacyWheeledIngress> LegacyWheeledIngress{};
        std::unique_ptr<FLegacyWheeledOwner> LegacyWheeledOwner{};
        bool bLegacyWheeledRetired= false;
        std::atomic<bool> bInputRetirementRequested{false};
        std::atomic<bool> bInputRetirementCompleted{false};
        bool bWheeledEndPlayTeardown= false;
        bool bSpeedWorldAdapterRetiredForTeardown= false;
        bool bResumeAfterPhysicsRecreation= false;
        bool bOwnedRecreationBoundary= false;
        TWeakObjectPtr<ASpeedPhysicsSimulation> RecreationDriver{};
        TOptional<uint64> PreparedCanonicalInputFrame{};
        std::shared_ptr<const Speed::Input::V2::FOwnerInputSnapshot> CanonicalInputSnapshot{};
        bool bCanonicalInputPending= false;
        mutable std::shared_ptr<const FCompletedWheeledInput> PreparedWheeledNetworkInput{};
        Speed::Concurrency::TAtomicSharedPtr<const FCompletedWheeledInput> PublishedWheeledNetworkInput{};
        std::atomic<bool> bGenericCameraInputEnabled{false};
        std::atomic<uint32> PendingCameraInput{1};
        mutable FCriticalSection PendingCameraInputMutex{};
        mutable TArray<FPendingCameraInputCommand> PendingCameraInputCommands{};
        mutable int32 LastCameraCaptureHistoryFrame= INDEX_NONE;
        mutable int32 LastCameraCaptureActivationFrame= INDEX_NONE;
        mutable FSpeedCarCameraPhysicalInput LastCapturedCameraInput{};
#if WITH_DEV_AUTOMATION_TESTS
        mutable uint64 CameraMailboxReadCount= 0;
#endif
#if WITH_DEV_AUTOMATION_TESTS
        uint64 CameraInputApplyCount= 0;
#endif
        FMatrix CarLocalInvI= FMatrix::Identity;
        TArray<int32> RecordedBaseFrames{};
        TArray<FBasePhysicsState> RecordedBaseStates{};
        TArray<FWheeledPhysicsState> RecordedWheeledStates{};
        float EngineFPS= 120.0;
        uint16 MinNbFramesBeforeCanMove= 0;
        TArray<USSubBody*> SubBodies{};
        TArray<USSubBody*> ExtSubBodies{};
        TArray<SWheelGroundContact> PendingWheelContacts{};
        bool bLegacyNetworkHistoryExcluded= false;
        TObjectPtr<UNetworkPhysicsComponent> SNetworkPhysicsComponent= nullptr;
        TObjectPtr<UNetworkPhysicsComponent> WheeledNetworkPhysicsComponent= nullptr;
        int32 NetCorr_LastServerFrame= INDEX_NONE;
        int32 NetCorr_LastLocalFrame= INDEX_NONE;
        int32 NetCorr_BaseNumPredictedFrames= 1;
        bool bNetCorrHasTarget= false;
        int32 NetCorrTickCount= 0;
        FNetworkBaseSpeedState NetCorrTarget{};
        FNetCorrAccum NetCorrAccum{};
        FVector NetCorr_LastContactN= FVector::UpVector;
        uint8 NetCorr_StableNFrames= 0;
        int32 SinceCanMoveFrame= INDEX_NONE;
        bool bCanonicalSupportStartupInitializationRequested= false;
        bool bCanonicalSupportStartupPoseConfirmed= false;
        bool bSimTimelineWasCanMove= false;
        bool bSimTimelineWasGrounded= false;
        bool bSimTimelineHasGroundState= false;
        std::atomic<bool> bTestInputOverrideEnabled{false};
#if !UE_BUILD_SHIPPING
        int TurnStartFrame= INDEX_NONE;
#endif
#if !UE_BUILD_SHIPPING
        int LastTurnSummaryFrame= INDEX_NONE;
#endif
        decltype(UChaosWheeledVehicleMovementComponent::VehicleSimulationPT) VehicleSimulationPT{};
        decltype(UChaosWheeledVehicleMovementComponent::Wheels) Wheels{};
        decltype(UChaosWheeledVehicleMovementComponent::WheelSetups) WheelSetups{};
        decltype(UChaosWheeledVehicleMovementComponent::CenterOfMassOverride) CenterOfMassOverride{};
        decltype(UChaosWheeledVehicleMovementComponent::bEnableCenterOfMassOverride) bEnableCenterOfMassOverride{};
        decltype(UChaosWheeledVehicleMovementComponent::bThrottleAsBrake) bThrottleAsBrake{};
        decltype(UChaosWheeledVehicleMovementComponent::bSuspensionEnabled) bSuspensionEnabled{};
        decltype(UChaosWheeledVehicleMovementComponent::bWheelFrictionEnabled) bWheelFrictionEnabled{};
        decltype(UChaosWheeledVehicleMovementComponent::ChassisHeight) ChassisHeight{};
        decltype(UChaosWheeledVehicleMovementComponent::DragCoefficient) DragCoefficient{};
        decltype(UChaosWheeledVehicleMovementComponent::EngineSetup) EngineSetup{};
        decltype(UChaosWheeledVehicleMovementComponent::ThrottleInputRate) ThrottleInputRate{};
        decltype(UChaosWheeledVehicleMovementComponent::TransmissionSetup) TransmissionSetup{};
        decltype(UChaosWheeledVehicleMovementComponent::DifferentialSetup) DifferentialSetup{};
        decltype(UChaosWheeledVehicleMovementComponent::bMechanicalSimEnabled) bMechanicalSimEnabled{};
        decltype(UChaosWheeledVehicleMovementComponent::SteeringInputRate) SteeringInputRate{};
        decltype(UChaosWheeledVehicleMovementComponent::SteeringSetup) SteeringSetup{};
    };
    std::shared_ptr<FOriginalWheeledDomain> OriginalWheeledDomain;
    std::weak_ptr<const Speed::NativeEpoch::IOriginalStorageRoute> OriginalWheeledRoute;
    bool UsesOriginalWheeledDomain() const
    {
        if(!IsInGameThread())return false;
        const auto Route=OriginalWheeledRoute.lock();
        return OriginalWheeledDomain&&Route&&Route->SelectsOriginalStorage();
    }
    bool BindOriginalWheeledDomain(const std::shared_ptr<const Speed::NativeEpoch::IOriginalStorageRoute>& Route);
protected:
    // Presence of the actual legacy Engine GameMode field, NOT IAS server role.
    // Only an original native owner with a real readonly field binding can
    // supply this. Unknown original profiles refuse in the default method.
    virtual bool ReadOriginalLegacyGameModePresence() const;
    bool& bPublishWheelPresentationWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bPublishWheelPresentation:bPublishWheelPresentation;}
    bool const& bPublishWheelPresentationWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bPublishWheelPresentation:bPublishWheelPresentation;}
    int32& CanonicalCameraYawActionWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->CanonicalCameraYawAction:CanonicalCameraYawAction;}
    int32 const& CanonicalCameraYawActionWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->CanonicalCameraYawAction:CanonicalCameraYawAction;}
    int32& CanonicalCameraPitchActionWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->CanonicalCameraPitchAction:CanonicalCameraPitchAction;}
    int32 const& CanonicalCameraPitchActionWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->CanonicalCameraPitchAction:CanonicalCameraPitchAction;}
    TObjectPtr<UNetworkPhysicsSettingsDataAsset>& NetDataAssetWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->NetDataAsset:NetDataAsset;}
    TObjectPtr<UNetworkPhysicsSettingsDataAsset> const& NetDataAssetWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->NetDataAsset:NetDataAsset;}
    float& TimeBeforeCanMoveWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->TimeBeforeCanMove:TimeBeforeCanMove;}
    float const& TimeBeforeCanMoveWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->TimeBeforeCanMove:TimeBeforeCanMove;}
    float& PhysMassWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PhysMass:PhysMass;}
    float const& PhysMassWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PhysMass:PhysMass;}
    bool& bEnableGravityWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bEnableGravity:bEnableGravity;}
    bool const& bEnableGravityWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bEnableGravity:bEnableGravity;}
    float& GravityZWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->GravityZ:GravityZ;}
    float const& GravityZWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->GravityZ:GravityZ;}
    float& PhysMaxSpeedWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PhysMaxSpeed:PhysMaxSpeed;}
    float const& PhysMaxSpeedWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PhysMaxSpeed:PhysMaxSpeed;}
    float& PhysMaxAngularSpeedWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PhysMaxAngularSpeed:PhysMaxAngularSpeed;}
    float const& PhysMaxAngularSpeedWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PhysMaxAngularSpeed:PhysMaxAngularSpeed;}
    float& PhysDampingWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PhysDamping:PhysDamping;}
    float const& PhysDampingWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PhysDamping:PhysDamping;}
    FVector& CenterOfMassWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->CenterOfMass:CenterOfMass;}
    FVector const& CenterOfMassWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->CenterOfMass:CenterOfMass;}
    float& MaxThrottleSpeedWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->MaxThrottleSpeed:MaxThrottleSpeed;}
    float const& MaxThrottleSpeedWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->MaxThrottleSpeed:MaxThrottleSpeed;}
    FVector& GroundFrictionWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->GroundFriction:GroundFriction;}
    FVector const& GroundFrictionWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->GroundFriction:GroundFriction;}
    float& WheelContactNormalVelocityTimeConstantWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelContactNormalVelocityTimeConstant:WheelContactNormalVelocityTimeConstant;}
    float const& WheelContactNormalVelocityTimeConstantWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelContactNormalVelocityTimeConstant:WheelContactNormalVelocityTimeConstant;}
    float& GroundAngularDampingWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->GroundAngularDamping:GroundAngularDamping;}
    float const& GroundAngularDampingWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->GroundAngularDamping:GroundAngularDamping;}
    float& RollDragCoeffWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->RollDragCoeff:RollDragCoeff;}
    float const& RollDragCoeffWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->RollDragCoeff:RollDragCoeff;}
    float& PitchDragCoeffWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PitchDragCoeff:PitchDragCoeff;}
    float const& PitchDragCoeffWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PitchDragCoeff:PitchDragCoeff;}
    float& YawDragCoeffWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->YawDragCoeff:YawDragCoeff;}
    float const& YawDragCoeffWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->YawDragCoeff:YawDragCoeff;}
    float& AirForwardThrottleWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->AirForwardThrottle:AirForwardThrottle;}
    float const& AirForwardThrottleWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->AirForwardThrottle:AirForwardThrottle;}
    float& AirBackwardThrottleWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->AirBackwardThrottle:AirBackwardThrottle;}
    float const& AirBackwardThrottleWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->AirBackwardThrottle:AirBackwardThrottle;}
    float& ThrottleAccelWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->ThrottleAccel:ThrottleAccel;}
    float const& ThrottleAccelWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->ThrottleAccel:ThrottleAccel;}
    float& SubSteeringAccelWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->SubSteeringAccel:SubSteeringAccel;}
    float const& SubSteeringAccelWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->SubSteeringAccel:SubSteeringAccel;}
    float& BrakeAccelWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->BrakeAccel:BrakeAccel;}
    float const& BrakeAccelWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->BrakeAccel:BrakeAccel;}
    TArray<FVector2D>& DriveSpeedAccelScaleCurveWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->DriveSpeedAccelScaleCurve:DriveSpeedAccelScaleCurve;}
    TArray<FVector2D> const& DriveSpeedAccelScaleCurveWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->DriveSpeedAccelScaleCurve:DriveSpeedAccelScaleCurve;}
    bool& bUseDriveModelLongitudinalFrictionWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bUseDriveModelLongitudinalFriction:bUseDriveModelLongitudinalFriction;}
    bool const& bUseDriveModelLongitudinalFrictionWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bUseDriveModelLongitudinalFriction:bUseDriveModelLongitudinalFriction;}
    float& ThrottleDeadzoneWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->ThrottleDeadzone:ThrottleDeadzone;}
    float const& ThrottleDeadzoneWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->ThrottleDeadzone:ThrottleDeadzone;}
    float& CoastingBrakeFactorWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->CoastingBrakeFactor:CoastingBrakeFactor;}
    float const& CoastingBrakeFactorWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->CoastingBrakeFactor:CoastingBrakeFactor;}
    float& StoppingForwardSpeedWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->StoppingForwardSpeed:StoppingForwardSpeed;}
    float const& StoppingForwardSpeedWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->StoppingForwardSpeed:StoppingForwardSpeed;}
    float& BrakingNoThrottleSpeedThresholdWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->BrakingNoThrottleSpeedThreshold:BrakingNoThrottleSpeedThreshold;}
    float const& BrakingNoThrottleSpeedThresholdWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->BrakingNoThrottleSpeedThreshold:BrakingNoThrottleSpeedThreshold;}
    TArray<FVector2D>& SteerAngleFromSpeedCurveWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->SteerAngleFromSpeedCurve:SteerAngleFromSpeedCurve;}
    TArray<FVector2D> const& SteerAngleFromSpeedCurveWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->SteerAngleFromSpeedCurve:SteerAngleFromSpeedCurve;}
    TArray<FVector2D>& HandbrakeSteerAngleFromSpeedCurveWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->HandbrakeSteerAngleFromSpeedCurve:HandbrakeSteerAngleFromSpeedCurve;}
    TArray<FVector2D> const& HandbrakeSteerAngleFromSpeedCurveWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->HandbrakeSteerAngleFromSpeedCurve:HandbrakeSteerAngleFromSpeedCurve;}
    bool& bUseWheelFrameLateralFrictionWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bUseWheelFrameLateralFriction:bUseWheelFrameLateralFriction;}
    bool const& bUseWheelFrameLateralFrictionWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bUseWheelFrameLateralFriction:bUseWheelFrameLateralFriction;}
    bool& bUseExplicitSteeringForceWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bUseExplicitSteeringForce:bUseExplicitSteeringForce;}
    bool const& bUseExplicitSteeringForceWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bUseExplicitSteeringForce:bUseExplicitSteeringForce;}
    bool& bUseLegacySteeringVelocityStateWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bUseLegacySteeringVelocityState:bUseLegacySteeringVelocityState;}
    bool const& bUseLegacySteeringVelocityStateWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bUseLegacySteeringVelocityState:bUseLegacySteeringVelocityState;}
    float& WheelFrameLateralFrictionScaleWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameLateralFrictionScale:WheelFrameLateralFrictionScale;}
    float const& WheelFrameLateralFrictionScaleWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameLateralFrictionScale:WheelFrameLateralFrictionScale;}
    float& WheelFrameLateralResponseScaleWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameLateralResponseScale:WheelFrameLateralResponseScale;}
    float const& WheelFrameLateralResponseScaleWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameLateralResponseScale:WheelFrameLateralResponseScale;}
    float& WheelFrameFrontLateralResponseScaleWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameFrontLateralResponseScale:WheelFrameFrontLateralResponseScale;}
    float const& WheelFrameFrontLateralResponseScaleWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameFrontLateralResponseScale:WheelFrameFrontLateralResponseScale;}
    float& WheelFrameRearLateralResponseScaleWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameRearLateralResponseScale:WheelFrameRearLateralResponseScale;}
    float const& WheelFrameRearLateralResponseScaleWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameRearLateralResponseScale:WheelFrameRearLateralResponseScale;}
    float& WheelFrameHandbrakeFrontLateralResponseScaleWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeFrontLateralResponseScale:WheelFrameHandbrakeFrontLateralResponseScale;}
    float const& WheelFrameHandbrakeFrontLateralResponseScaleWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeFrontLateralResponseScale:WheelFrameHandbrakeFrontLateralResponseScale;}
    float& WheelFrameHandbrakeRearLateralResponseScaleWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeRearLateralResponseScale:WheelFrameHandbrakeRearLateralResponseScale;}
    float const& WheelFrameHandbrakeRearLateralResponseScaleWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeRearLateralResponseScale:WheelFrameHandbrakeRearLateralResponseScale;}
    float& WheelFrameHandbrakeReverseFrontLateralResponseScaleWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeReverseFrontLateralResponseScale:WheelFrameHandbrakeReverseFrontLateralResponseScale;}
    float const& WheelFrameHandbrakeReverseFrontLateralResponseScaleWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeReverseFrontLateralResponseScale:WheelFrameHandbrakeReverseFrontLateralResponseScale;}
    float& WheelFrameHandbrakeReverseRearLateralResponseScaleWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeReverseRearLateralResponseScale:WheelFrameHandbrakeReverseRearLateralResponseScale;}
    float const& WheelFrameHandbrakeReverseRearLateralResponseScaleWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeReverseRearLateralResponseScale:WheelFrameHandbrakeReverseRearLateralResponseScale;}
    float& WheelFrameLateralImpulseTimeConstantWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameLateralImpulseTimeConstant:WheelFrameLateralImpulseTimeConstant;}
    float const& WheelFrameLateralImpulseTimeConstantWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameLateralImpulseTimeConstant:WheelFrameLateralImpulseTimeConstant;}
    float& WheelFrameZeroSteerLateralFrictionScaleWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameZeroSteerLateralFrictionScale:WheelFrameZeroSteerLateralFrictionScale;}
    float const& WheelFrameZeroSteerLateralFrictionScaleWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameZeroSteerLateralFrictionScale:WheelFrameZeroSteerLateralFrictionScale;}
    float& WheelFrameUnsteeredAligningYawMaxRateWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameUnsteeredAligningYawMaxRate:WheelFrameUnsteeredAligningYawMaxRate;}
    float const& WheelFrameUnsteeredAligningYawMaxRateWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameUnsteeredAligningYawMaxRate:WheelFrameUnsteeredAligningYawMaxRate;}
    float& WheelFrameUnsteeredAligningYawFullSlipSpeedWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameUnsteeredAligningYawFullSlipSpeed:WheelFrameUnsteeredAligningYawFullSlipSpeed;}
    float const& WheelFrameUnsteeredAligningYawFullSlipSpeedWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameUnsteeredAligningYawFullSlipSpeed:WheelFrameUnsteeredAligningYawFullSlipSpeed;}
    float& WheelFrameUnsteeredAligningYawMinSlipSpeedWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameUnsteeredAligningYawMinSlipSpeed:WheelFrameUnsteeredAligningYawMinSlipSpeed;}
    float const& WheelFrameUnsteeredAligningYawMinSlipSpeedWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameUnsteeredAligningYawMinSlipSpeed:WheelFrameUnsteeredAligningYawMinSlipSpeed;}
    float& WheelFrameUnsteeredAligningYawTimeConstantWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameUnsteeredAligningYawTimeConstant:WheelFrameUnsteeredAligningYawTimeConstant;}
    float const& WheelFrameUnsteeredAligningYawTimeConstantWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameUnsteeredAligningYawTimeConstant:WheelFrameUnsteeredAligningYawTimeConstant;}
    float& WheelFrameSteeringRearLateralFrictionScaleWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameSteeringRearLateralFrictionScale:WheelFrameSteeringRearLateralFrictionScale;}
    float const& WheelFrameSteeringRearLateralFrictionScaleWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameSteeringRearLateralFrictionScale:WheelFrameSteeringRearLateralFrictionScale;}
    bool& bClampWheelFrameLateralImpulseWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bClampWheelFrameLateralImpulse:bClampWheelFrameLateralImpulse;}
    bool const& bClampWheelFrameLateralImpulseWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bClampWheelFrameLateralImpulse:bClampWheelFrameLateralImpulse;}
    float& WheelFrameHandbrakeLateralImpulseScaleWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeLateralImpulseScale:WheelFrameHandbrakeLateralImpulseScale;}
    float const& WheelFrameHandbrakeLateralImpulseScaleWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeLateralImpulseScale:WheelFrameHandbrakeLateralImpulseScale;}
    TArray<FVector2D>& HandbrakeLateralImpulseScaleCurveWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->HandbrakeLateralImpulseScaleCurve:HandbrakeLateralImpulseScaleCurve;}
    TArray<FVector2D> const& HandbrakeLateralImpulseScaleCurveWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->HandbrakeLateralImpulseScaleCurve:HandbrakeLateralImpulseScaleCurve;}
    float& WheelFrameLongitudinalScrubScaleWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameLongitudinalScrubScale:WheelFrameLongitudinalScrubScale;}
    float const& WheelFrameLongitudinalScrubScaleWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameLongitudinalScrubScale:WheelFrameLongitudinalScrubScale;}
    float& WheelFrameSteeringCurvatureScaleWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameSteeringCurvatureScale:WheelFrameSteeringCurvatureScale;}
    float const& WheelFrameSteeringCurvatureScaleWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameSteeringCurvatureScale:WheelFrameSteeringCurvatureScale;}
    float& WheelFrameSteeringReferenceWheelbaseCmWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameSteeringReferenceWheelbaseCm:WheelFrameSteeringReferenceWheelbaseCm;}
    float const& WheelFrameSteeringReferenceWheelbaseCmWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameSteeringReferenceWheelbaseCm:WheelFrameSteeringReferenceWheelbaseCm;}
    float& WheelFrameLateralForwardLossScaleWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameLateralForwardLossScale:WheelFrameLateralForwardLossScale;}
    float const& WheelFrameLateralForwardLossScaleWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameLateralForwardLossScale:WheelFrameLateralForwardLossScale;}
    TArray<FVector2D>& LateralForwardLossScaleCurveWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->LateralForwardLossScaleCurve:LateralForwardLossScaleCurve;}
    TArray<FVector2D> const& LateralForwardLossScaleCurveWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->LateralForwardLossScaleCurve:LateralForwardLossScaleCurve;}
    TArray<FVector2D>& LateralForwardLossSlipScaleCurveWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->LateralForwardLossSlipScaleCurve:LateralForwardLossSlipScaleCurve;}
    TArray<FVector2D> const& LateralForwardLossSlipScaleCurveWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->LateralForwardLossSlipScaleCurve:LateralForwardLossSlipScaleCurve;}
    TArray<FVector2D>& SteeringForwardLossSpeedScaleCurveWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->SteeringForwardLossSpeedScaleCurve:SteeringForwardLossSpeedScaleCurve;}
    TArray<FVector2D> const& SteeringForwardLossSpeedScaleCurveWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->SteeringForwardLossSpeedScaleCurve:SteeringForwardLossSpeedScaleCurve;}
    TArray<FVector2D>& LateralFrictionScaleCurveWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->LateralFrictionScaleCurve:LateralFrictionScaleCurve;}
    TArray<FVector2D> const& LateralFrictionScaleCurveWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->LateralFrictionScaleCurve:LateralFrictionScaleCurve;}
    TArray<FVector2D>& LongitudinalFrictionScaleCurveWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->LongitudinalFrictionScaleCurve:LongitudinalFrictionScaleCurve;}
    TArray<FVector2D> const& LongitudinalFrictionScaleCurveWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->LongitudinalFrictionScaleCurve:LongitudinalFrictionScaleCurve;}
    TArray<FVector2D>& NonAcceleratingFrictionScaleCurveWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->NonAcceleratingFrictionScaleCurve:NonAcceleratingFrictionScaleCurve;}
    TArray<FVector2D> const& NonAcceleratingFrictionScaleCurveWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->NonAcceleratingFrictionScaleCurve:NonAcceleratingFrictionScaleCurve;}
    TArray<FVector2D>& HandbrakeFrontLateralFrictionFactorCurveWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->HandbrakeFrontLateralFrictionFactorCurve:HandbrakeFrontLateralFrictionFactorCurve;}
    TArray<FVector2D> const& HandbrakeFrontLateralFrictionFactorCurveWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->HandbrakeFrontLateralFrictionFactorCurve:HandbrakeFrontLateralFrictionFactorCurve;}
    TArray<FVector2D>& HandbrakeRearLateralFrictionFactorCurveWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->HandbrakeRearLateralFrictionFactorCurve:HandbrakeRearLateralFrictionFactorCurve;}
    TArray<FVector2D> const& HandbrakeRearLateralFrictionFactorCurveWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->HandbrakeRearLateralFrictionFactorCurve:HandbrakeRearLateralFrictionFactorCurve;}
    TArray<FVector2D>& HandbrakeFrontLateralFrictionSpeedScaleCurveWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->HandbrakeFrontLateralFrictionSpeedScaleCurve:HandbrakeFrontLateralFrictionSpeedScaleCurve;}
    TArray<FVector2D> const& HandbrakeFrontLateralFrictionSpeedScaleCurveWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->HandbrakeFrontLateralFrictionSpeedScaleCurve:HandbrakeFrontLateralFrictionSpeedScaleCurve;}
    TArray<FVector2D>& HandbrakeRearLateralFrictionSpeedScaleCurveWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->HandbrakeRearLateralFrictionSpeedScaleCurve:HandbrakeRearLateralFrictionSpeedScaleCurve;}
    TArray<FVector2D> const& HandbrakeRearLateralFrictionSpeedScaleCurveWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->HandbrakeRearLateralFrictionSpeedScaleCurve:HandbrakeRearLateralFrictionSpeedScaleCurve;}
    TArray<FVector2D>& HandbrakeSteeringRearLateralFrictionScaleCurveWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->HandbrakeSteeringRearLateralFrictionScaleCurve:HandbrakeSteeringRearLateralFrictionScaleCurve;}
    TArray<FVector2D> const& HandbrakeSteeringRearLateralFrictionScaleCurveWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->HandbrakeSteeringRearLateralFrictionScaleCurve:HandbrakeSteeringRearLateralFrictionScaleCurve;}
    TArray<FVector2D>& HandbrakeSteeringRearLateralFrictionSlipScaleCurveWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->HandbrakeSteeringRearLateralFrictionSlipScaleCurve:HandbrakeSteeringRearLateralFrictionSlipScaleCurve;}
    TArray<FVector2D> const& HandbrakeSteeringRearLateralFrictionSlipScaleCurveWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->HandbrakeSteeringRearLateralFrictionSlipScaleCurve:HandbrakeSteeringRearLateralFrictionSlipScaleCurve;}
    TArray<FVector2D>& HandbrakeLowSlipYawCorrectionScaleCurveWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->HandbrakeLowSlipYawCorrectionScaleCurve:HandbrakeLowSlipYawCorrectionScaleCurve;}
    TArray<FVector2D> const& HandbrakeLowSlipYawCorrectionScaleCurveWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->HandbrakeLowSlipYawCorrectionScaleCurve:HandbrakeLowSlipYawCorrectionScaleCurve;}
    TArray<FVector2D>& HandbrakeLongitudinalFrictionFactorCurveWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->HandbrakeLongitudinalFrictionFactorCurve:HandbrakeLongitudinalFrictionFactorCurve;}
    TArray<FVector2D> const& HandbrakeLongitudinalFrictionFactorCurveWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->HandbrakeLongitudinalFrictionFactorCurve:HandbrakeLongitudinalFrictionFactorCurve;}
    float& WheelFrameHandbrakeEntryDurationWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeEntryDuration:WheelFrameHandbrakeEntryDuration;}
    float const& WheelFrameHandbrakeEntryDurationWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeEntryDuration:WheelFrameHandbrakeEntryDuration;}
    float& WheelFrameHandbrakeEntryMinSpeedWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeEntryMinSpeed:WheelFrameHandbrakeEntryMinSpeed;}
    float const& WheelFrameHandbrakeEntryMinSpeedWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeEntryMinSpeed:WheelFrameHandbrakeEntryMinSpeed;}
    float& WheelFrameHandbrakeEntryFullSpeedWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeEntryFullSpeed:WheelFrameHandbrakeEntryFullSpeed;}
    float const& WheelFrameHandbrakeEntryFullSpeedWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeEntryFullSpeed:WheelFrameHandbrakeEntryFullSpeed;}
    float& WheelFrameHandbrakeEntryMinSteeringWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeEntryMinSteering:WheelFrameHandbrakeEntryMinSteering;}
    float const& WheelFrameHandbrakeEntryMinSteeringWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeEntryMinSteering:WheelFrameHandbrakeEntryMinSteering;}
    float& WheelFrameHandbrakeEntryRearReleaseScaleWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeEntryRearReleaseScale:WheelFrameHandbrakeEntryRearReleaseScale;}
    float const& WheelFrameHandbrakeEntryRearReleaseScaleWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeEntryRearReleaseScale:WheelFrameHandbrakeEntryRearReleaseScale;}
    float& WheelFrameHandbrakeRiseRateWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeRiseRate:WheelFrameHandbrakeRiseRate;}
    float const& WheelFrameHandbrakeRiseRateWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeRiseRate:WheelFrameHandbrakeRiseRate;}
    float& WheelFrameHandbrakeFallRateWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeFallRate:WheelFrameHandbrakeFallRate;}
    float const& WheelFrameHandbrakeFallRateWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeFallRate:WheelFrameHandbrakeFallRate;}
    float& LateralFrictionSlipThresholdWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->LateralFrictionSlipThreshold:LateralFrictionSlipThreshold;}
    float const& LateralFrictionSlipThresholdWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->LateralFrictionSlipThreshold:LateralFrictionSlipThreshold;}
    float& WheelFrameHandbrakeValueWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeValue:WheelFrameHandbrakeValue;}
    float const& WheelFrameHandbrakeValueWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeValue:WheelFrameHandbrakeValue;}
    float& WheelFrameHandbrakeActiveTimeWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeActiveTime:WheelFrameHandbrakeActiveTime;}
    float const& WheelFrameHandbrakeActiveTimeWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelFrameHandbrakeActiveTime:WheelFrameHandbrakeActiveTime;}
    int32& LastWheelFrameHandbrakeUpdateFrameWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->LastWheelFrameHandbrakeUpdateFrame:LastWheelFrameHandbrakeUpdateFrame;}
    int32 const& LastWheelFrameHandbrakeUpdateFrameWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->LastWheelFrameHandbrakeUpdateFrame:LastWheelFrameHandbrakeUpdateFrame;}
    float& PosStabilityMultiplierWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PosStabilityMultiplier:PosStabilityMultiplier;}
    float const& PosStabilityMultiplierWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PosStabilityMultiplier:PosStabilityMultiplier;}
    float& VelStabilityMultiplierWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->VelStabilityMultiplier:VelStabilityMultiplier;}
    float const& VelStabilityMultiplierWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->VelStabilityMultiplier:VelStabilityMultiplier;}
    float& RotStabilityMultiplierWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->RotStabilityMultiplier:RotStabilityMultiplier;}
    float const& RotStabilityMultiplierWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->RotStabilityMultiplier:RotStabilityMultiplier;}
    float& AngVelStabilityMultiplierWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->AngVelStabilityMultiplier:AngVelStabilityMultiplier;}
    float const& AngVelStabilityMultiplierWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->AngVelStabilityMultiplier:AngVelStabilityMultiplier;}
    float& MaxPosCorrectionPerSecondWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->MaxPosCorrectionPerSecond:MaxPosCorrectionPerSecond;}
    float const& MaxPosCorrectionPerSecondWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->MaxPosCorrectionPerSecond:MaxPosCorrectionPerSecond;}
    float& MaxVelCorrectionPerSecondWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->MaxVelCorrectionPerSecond:MaxVelCorrectionPerSecond;}
    float const& MaxVelCorrectionPerSecondWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->MaxVelCorrectionPerSecond:MaxVelCorrectionPerSecond;}
    float& MaxAngVelCorrectionPerSecondWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->MaxAngVelCorrectionPerSecond:MaxAngVelCorrectionPerSecond;}
    float const& MaxAngVelCorrectionPerSecondWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->MaxAngVelCorrectionPerSecond:MaxAngVelCorrectionPerSecond;}
    float& MaxRotCorrectionPerSecondWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->MaxRotCorrectionPerSecond:MaxRotCorrectionPerSecond;}
    float const& MaxRotCorrectionPerSecondWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->MaxRotCorrectionPerSecond:MaxRotCorrectionPerSecond;}
    float& PosCorrDeadzoneWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PosCorrDeadzone:PosCorrDeadzone;}
    float const& PosCorrDeadzoneWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PosCorrDeadzone:PosCorrDeadzone;}
    float& VelCorrDeadzoneWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->VelCorrDeadzone:VelCorrDeadzone;}
    float const& VelCorrDeadzoneWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->VelCorrDeadzone:VelCorrDeadzone;}
    float& AngVelCorrDeadzoneWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->AngVelCorrDeadzone:AngVelCorrDeadzone;}
    float const& AngVelCorrDeadzoneWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->AngVelCorrDeadzone:AngVelCorrDeadzone;}
    float& RotCorrDeadzoneWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->RotCorrDeadzone:RotCorrDeadzone;}
    float const& RotCorrDeadzoneWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->RotCorrDeadzone:RotCorrDeadzone;}
    TObjectPtr<UBoxSubBody>& HitboxSubBodyWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->HitboxSubBody:HitboxSubBody;}
    TObjectPtr<UBoxSubBody> const& HitboxSubBodyWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->HitboxSubBody:HitboxSubBody;}
    TArray< TObjectPtr<USWheelSubBody>>& WheelSubBodiesWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelSubBodies:WheelSubBodies;}
    TArray< TObjectPtr<USWheelSubBody>> const& WheelSubBodiesWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelSubBodies:WheelSubBodies;}
    TObjectPtr<UNetworkPhysicsSettingsComponent>& SNetworkSettingsWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->SNetworkSettings:SNetworkSettings;}
    TObjectPtr<UNetworkPhysicsSettingsComponent> const& SNetworkSettingsWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->SNetworkSettings:SNetworkSettings;}
    USpeedSimulation*& SkySimulationWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->SkySimulation:SkySimulation;}
    USpeedSimulation* const& SkySimulationWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->SkySimulation:SkySimulation;}
    ASpeedCar*& SpeedCarOwnerWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->SpeedCarOwner:SpeedCarOwner;}
    ASpeedCar* const& SpeedCarOwnerWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->SpeedCarOwner:SpeedCarOwner;}
    FCriticalSection& FrameInputProducerMutexWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->FrameInputProducerMutex:FrameInputProducerMutex;}
    FCriticalSection const& FrameInputProducerMutexWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->FrameInputProducerMutex:FrameInputProducerMutex;}
    std::shared_ptr<Speed::Input::FInputStream>& FrameInputStreamWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->FrameInputStream:FrameInputStream;}
    std::shared_ptr<Speed::Input::FInputStream> const& FrameInputStreamWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->FrameInputStream:FrameInputStream;}
    std::shared_ptr<Speed::Input::FInputStream>& ConsumedFrameInputStreamWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->ConsumedFrameInputStream:ConsumedFrameInputStream;}
    std::shared_ptr<Speed::Input::FInputStream> const& ConsumedFrameInputStreamWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->ConsumedFrameInputStream:ConsumedFrameInputStream;}
    std::shared_ptr<Speed::Input::V2::FInputStream>& FrameInputStreamV2WheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->FrameInputStreamV2:FrameInputStreamV2;}
    std::shared_ptr<Speed::Input::V2::FInputStream> const& FrameInputStreamV2WheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->FrameInputStreamV2:FrameInputStreamV2;}
    std::shared_ptr<Speed::Input::V2::FInputStream>& ConsumedFrameInputStreamV2WheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->ConsumedFrameInputStreamV2:ConsumedFrameInputStreamV2;}
    std::shared_ptr<Speed::Input::V2::FInputStream> const& ConsumedFrameInputStreamV2WheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->ConsumedFrameInputStreamV2:ConsumedFrameInputStreamV2;}
    std::optional<Speed::Input::V2::FReservationToken>& InputReservationV2WheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->InputReservationV2:InputReservationV2;}
    std::optional<Speed::Input::V2::FReservationToken> const& InputReservationV2WheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->InputReservationV2:InputReservationV2;}
    std::optional<uint64>& ReservedInputFrameV2WheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->ReservedInputFrameV2:ReservedInputFrameV2;}
    std::optional<uint64> const& ReservedInputFrameV2WheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->ReservedInputFrameV2:ReservedInputFrameV2;}
    bool& bInputFrameRejectedV2WheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bInputFrameRejectedV2:bInputFrameRejectedV2;}
    bool const& bInputFrameRejectedV2WheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bInputFrameRejectedV2:bInputFrameRejectedV2;}
    bool& bResetProducedWheeledInputsWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bResetProducedWheeledInputs:bResetProducedWheeledInputs;}
    bool const& bResetProducedWheeledInputsWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bResetProducedWheeledInputs:bResetProducedWheeledInputs;}
    bool& bProducedInputFrameOwnedWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bProducedInputFrameOwned:bProducedInputFrameOwned;}
    bool const& bProducedInputFrameOwnedWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bProducedInputFrameOwned:bProducedInputFrameOwned;}
    std::atomic<bool>& bProducedInputAuthorityWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bProducedInputAuthority:bProducedInputAuthority;}
    std::atomic<bool> const& bProducedInputAuthorityWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bProducedInputAuthority:bProducedInputAuthority;}
    std::atomic<bool>& bLegacyRemoteInputAuthorityWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bLegacyRemoteInputAuthority:bLegacyRemoteInputAuthority;}
    std::atomic<bool> const& bLegacyRemoteInputAuthorityWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bLegacyRemoteInputAuthority:bLegacyRemoteInputAuthority;}
    FCriticalSection& LegacyWheeledIngressMutexWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->LegacyWheeledIngressMutex:LegacyWheeledIngressMutex;}
    FCriticalSection const& LegacyWheeledIngressMutexWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->LegacyWheeledIngressMutex:LegacyWheeledIngressMutex;}
    std::shared_ptr<FLegacyWheeledIngress>& LegacyWheeledIngressWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->LegacyWheeledIngress:LegacyWheeledIngress;}
    std::shared_ptr<FLegacyWheeledIngress> const& LegacyWheeledIngressWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->LegacyWheeledIngress:LegacyWheeledIngress;}
    std::unique_ptr<FLegacyWheeledOwner>& LegacyWheeledOwnerWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->LegacyWheeledOwner:LegacyWheeledOwner;}
    std::unique_ptr<FLegacyWheeledOwner> const& LegacyWheeledOwnerWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->LegacyWheeledOwner:LegacyWheeledOwner;}
    bool& bLegacyWheeledRetiredWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bLegacyWheeledRetired:bLegacyWheeledRetired;}
    bool const& bLegacyWheeledRetiredWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bLegacyWheeledRetired:bLegacyWheeledRetired;}
    std::atomic<bool>& bInputRetirementRequestedWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bInputRetirementRequested:bInputRetirementRequested;}
    std::atomic<bool> const& bInputRetirementRequestedWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bInputRetirementRequested:bInputRetirementRequested;}
    std::atomic<bool>& bInputRetirementCompletedWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bInputRetirementCompleted:bInputRetirementCompleted;}
    std::atomic<bool> const& bInputRetirementCompletedWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bInputRetirementCompleted:bInputRetirementCompleted;}
    bool& bWheeledEndPlayTeardownWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bWheeledEndPlayTeardown:bWheeledEndPlayTeardown;}
    bool const& bWheeledEndPlayTeardownWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bWheeledEndPlayTeardown:bWheeledEndPlayTeardown;}
    bool& bSpeedWorldAdapterRetiredForTeardownWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bSpeedWorldAdapterRetiredForTeardown:bSpeedWorldAdapterRetiredForTeardown;}
    bool const& bSpeedWorldAdapterRetiredForTeardownWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bSpeedWorldAdapterRetiredForTeardown:bSpeedWorldAdapterRetiredForTeardown;}
    bool& bResumeAfterPhysicsRecreationWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bResumeAfterPhysicsRecreation:bResumeAfterPhysicsRecreation;}
    bool const& bResumeAfterPhysicsRecreationWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bResumeAfterPhysicsRecreation:bResumeAfterPhysicsRecreation;}
    bool& bOwnedRecreationBoundaryWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bOwnedRecreationBoundary:bOwnedRecreationBoundary;}
    bool const& bOwnedRecreationBoundaryWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bOwnedRecreationBoundary:bOwnedRecreationBoundary;}
    TWeakObjectPtr<ASpeedPhysicsSimulation>& RecreationDriverWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->RecreationDriver:RecreationDriver;}
    TWeakObjectPtr<ASpeedPhysicsSimulation> const& RecreationDriverWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->RecreationDriver:RecreationDriver;}
    TOptional<uint64>& PreparedCanonicalInputFrameWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PreparedCanonicalInputFrame:PreparedCanonicalInputFrame;}
    TOptional<uint64> const& PreparedCanonicalInputFrameWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PreparedCanonicalInputFrame:PreparedCanonicalInputFrame;}
    std::shared_ptr<const Speed::Input::V2::FOwnerInputSnapshot>& CanonicalInputSnapshotWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->CanonicalInputSnapshot:CanonicalInputSnapshot;}
    std::shared_ptr<const Speed::Input::V2::FOwnerInputSnapshot> const& CanonicalInputSnapshotWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->CanonicalInputSnapshot:CanonicalInputSnapshot;}
    bool& bCanonicalInputPendingWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bCanonicalInputPending:bCanonicalInputPending;}
    bool const& bCanonicalInputPendingWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bCanonicalInputPending:bCanonicalInputPending;}
    std::shared_ptr<const FCompletedWheeledInput>& PreparedWheeledNetworkInputWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PreparedWheeledNetworkInput:PreparedWheeledNetworkInput;}
    std::shared_ptr<const FCompletedWheeledInput>& PreparedWheeledNetworkInputWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PreparedWheeledNetworkInput:PreparedWheeledNetworkInput;}
    Speed::Concurrency::TAtomicSharedPtr<const FCompletedWheeledInput>& PublishedWheeledNetworkInputWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PublishedWheeledNetworkInput:PublishedWheeledNetworkInput;}
    Speed::Concurrency::TAtomicSharedPtr<const FCompletedWheeledInput> const& PublishedWheeledNetworkInputWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PublishedWheeledNetworkInput:PublishedWheeledNetworkInput;}
    std::atomic<bool>& bGenericCameraInputEnabledWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bGenericCameraInputEnabled:bGenericCameraInputEnabled;}
    std::atomic<bool> const& bGenericCameraInputEnabledWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bGenericCameraInputEnabled:bGenericCameraInputEnabled;}
    std::atomic<uint32>& PendingCameraInputWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PendingCameraInput:PendingCameraInput;}
    std::atomic<uint32> const& PendingCameraInputWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PendingCameraInput:PendingCameraInput;}
    FCriticalSection& PendingCameraInputMutexWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PendingCameraInputMutex:PendingCameraInputMutex;}
    FCriticalSection& PendingCameraInputMutexWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PendingCameraInputMutex:PendingCameraInputMutex;}
    TArray<FPendingCameraInputCommand>& PendingCameraInputCommandsWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PendingCameraInputCommands:PendingCameraInputCommands;}
    TArray<FPendingCameraInputCommand>& PendingCameraInputCommandsWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PendingCameraInputCommands:PendingCameraInputCommands;}
    int32& LastCameraCaptureHistoryFrameWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->LastCameraCaptureHistoryFrame:LastCameraCaptureHistoryFrame;}
    int32& LastCameraCaptureHistoryFrameWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->LastCameraCaptureHistoryFrame:LastCameraCaptureHistoryFrame;}
    int32& LastCameraCaptureActivationFrameWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->LastCameraCaptureActivationFrame:LastCameraCaptureActivationFrame;}
    int32& LastCameraCaptureActivationFrameWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->LastCameraCaptureActivationFrame:LastCameraCaptureActivationFrame;}
    FSpeedCarCameraPhysicalInput& LastCapturedCameraInputWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->LastCapturedCameraInput:LastCapturedCameraInput;}
    FSpeedCarCameraPhysicalInput& LastCapturedCameraInputWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->LastCapturedCameraInput:LastCapturedCameraInput;}
#if WITH_DEV_AUTOMATION_TESTS
    uint64& CameraMailboxReadCountWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->CameraMailboxReadCount:CameraMailboxReadCount;}
    uint64& CameraMailboxReadCountWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->CameraMailboxReadCount:CameraMailboxReadCount;}
#endif
#if WITH_DEV_AUTOMATION_TESTS
    uint64& CameraInputApplyCountWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->CameraInputApplyCount:CameraInputApplyCount;}
    uint64 const& CameraInputApplyCountWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->CameraInputApplyCount:CameraInputApplyCount;}
#endif
    FMatrix& CarLocalInvIWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->CarLocalInvI:CarLocalInvI;}
    FMatrix const& CarLocalInvIWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->CarLocalInvI:CarLocalInvI;}
    TArray<int32>& RecordedBaseFramesWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->RecordedBaseFrames:RecordedBaseFrames;}
    TArray<int32> const& RecordedBaseFramesWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->RecordedBaseFrames:RecordedBaseFrames;}
    TArray<FBasePhysicsState>& RecordedBaseStatesWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->RecordedBaseStates:RecordedBaseStates;}
    TArray<FBasePhysicsState> const& RecordedBaseStatesWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->RecordedBaseStates:RecordedBaseStates;}
    TArray<FWheeledPhysicsState>& RecordedWheeledStatesWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->RecordedWheeledStates:RecordedWheeledStates;}
    TArray<FWheeledPhysicsState> const& RecordedWheeledStatesWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->RecordedWheeledStates:RecordedWheeledStates;}
    float& EngineFPSWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->EngineFPS:EngineFPS;}
    float const& EngineFPSWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->EngineFPS:EngineFPS;}
    uint16& MinNbFramesBeforeCanMoveWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->MinNbFramesBeforeCanMove:MinNbFramesBeforeCanMove;}
    uint16 const& MinNbFramesBeforeCanMoveWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->MinNbFramesBeforeCanMove:MinNbFramesBeforeCanMove;}
    TArray<USSubBody*>& SubBodiesWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->SubBodies:SubBodies;}
    TArray<USSubBody*> const& SubBodiesWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->SubBodies:SubBodies;}
    TArray<USSubBody*>& ExtSubBodiesWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->ExtSubBodies:ExtSubBodies;}
    TArray<USSubBody*> const& ExtSubBodiesWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->ExtSubBodies:ExtSubBodies;}
    TArray<SWheelGroundContact>& PendingWheelContactsWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PendingWheelContacts:PendingWheelContacts;}
    TArray<SWheelGroundContact> const& PendingWheelContactsWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->PendingWheelContacts:PendingWheelContacts;}
    bool& bLegacyNetworkHistoryExcludedWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bLegacyNetworkHistoryExcluded:bLegacyNetworkHistoryExcluded;}
    bool const& bLegacyNetworkHistoryExcludedWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bLegacyNetworkHistoryExcluded:bLegacyNetworkHistoryExcluded;}
    TObjectPtr<UNetworkPhysicsComponent>& SNetworkPhysicsComponentWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->SNetworkPhysicsComponent:SNetworkPhysicsComponent;}
    TObjectPtr<UNetworkPhysicsComponent> const& SNetworkPhysicsComponentWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->SNetworkPhysicsComponent:SNetworkPhysicsComponent;}
    TObjectPtr<UNetworkPhysicsComponent>& WheeledNetworkPhysicsComponentWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheeledNetworkPhysicsComponent:WheeledNetworkPhysicsComponent;}
    TObjectPtr<UNetworkPhysicsComponent> const& WheeledNetworkPhysicsComponentWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheeledNetworkPhysicsComponent:WheeledNetworkPhysicsComponent;}
    int32& NetCorr_LastServerFrameWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->NetCorr_LastServerFrame:NetCorr_LastServerFrame;}
    int32 const& NetCorr_LastServerFrameWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->NetCorr_LastServerFrame:NetCorr_LastServerFrame;}
    int32& NetCorr_LastLocalFrameWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->NetCorr_LastLocalFrame:NetCorr_LastLocalFrame;}
    int32 const& NetCorr_LastLocalFrameWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->NetCorr_LastLocalFrame:NetCorr_LastLocalFrame;}
    int32& NetCorr_BaseNumPredictedFramesWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->NetCorr_BaseNumPredictedFrames:NetCorr_BaseNumPredictedFrames;}
    int32 const& NetCorr_BaseNumPredictedFramesWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->NetCorr_BaseNumPredictedFrames:NetCorr_BaseNumPredictedFrames;}
    bool& bNetCorrHasTargetWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bNetCorrHasTarget:bNetCorrHasTarget;}
    bool const& bNetCorrHasTargetWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bNetCorrHasTarget:bNetCorrHasTarget;}
    int32& NetCorrTickCountWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->NetCorrTickCount:NetCorrTickCount;}
    int32 const& NetCorrTickCountWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->NetCorrTickCount:NetCorrTickCount;}
    FNetworkBaseSpeedState& NetCorrTargetWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->NetCorrTarget:NetCorrTarget;}
    FNetworkBaseSpeedState const& NetCorrTargetWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->NetCorrTarget:NetCorrTarget;}
    FNetCorrAccum& NetCorrAccumWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->NetCorrAccum:NetCorrAccum;}
    FNetCorrAccum const& NetCorrAccumWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->NetCorrAccum:NetCorrAccum;}
    FVector& NetCorr_LastContactNWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->NetCorr_LastContactN:NetCorr_LastContactN;}
    FVector const& NetCorr_LastContactNWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->NetCorr_LastContactN:NetCorr_LastContactN;}
    uint8& NetCorr_StableNFramesWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->NetCorr_StableNFrames:NetCorr_StableNFrames;}
    uint8 const& NetCorr_StableNFramesWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->NetCorr_StableNFrames:NetCorr_StableNFrames;}
    int32& SinceCanMoveFrameWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->SinceCanMoveFrame:SinceCanMoveFrame;}
    int32 const& SinceCanMoveFrameWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->SinceCanMoveFrame:SinceCanMoveFrame;}
    bool& bCanonicalSupportStartupInitializationRequestedWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bCanonicalSupportStartupInitializationRequested:bCanonicalSupportStartupInitializationRequested;}
    bool const& bCanonicalSupportStartupInitializationRequestedWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bCanonicalSupportStartupInitializationRequested:bCanonicalSupportStartupInitializationRequested;}
    bool& bCanonicalSupportStartupPoseConfirmedWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bCanonicalSupportStartupPoseConfirmed:bCanonicalSupportStartupPoseConfirmed;}
    bool const& bCanonicalSupportStartupPoseConfirmedWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bCanonicalSupportStartupPoseConfirmed:bCanonicalSupportStartupPoseConfirmed;}
    bool& bSimTimelineWasCanMoveWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bSimTimelineWasCanMove:bSimTimelineWasCanMove;}
    bool const& bSimTimelineWasCanMoveWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bSimTimelineWasCanMove:bSimTimelineWasCanMove;}
    bool& bSimTimelineWasGroundedWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bSimTimelineWasGrounded:bSimTimelineWasGrounded;}
    bool const& bSimTimelineWasGroundedWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bSimTimelineWasGrounded:bSimTimelineWasGrounded;}
    bool& bSimTimelineHasGroundStateWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bSimTimelineHasGroundState:bSimTimelineHasGroundState;}
    bool const& bSimTimelineHasGroundStateWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bSimTimelineHasGroundState:bSimTimelineHasGroundState;}
    std::atomic<bool>& bTestInputOverrideEnabledWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bTestInputOverrideEnabled:bTestInputOverrideEnabled;}
    std::atomic<bool> const& bTestInputOverrideEnabledWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bTestInputOverrideEnabled:bTestInputOverrideEnabled;}
#if !UE_BUILD_SHIPPING
    int& TurnStartFrameWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->TurnStartFrame:TurnStartFrame;}
    int const& TurnStartFrameWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->TurnStartFrame:TurnStartFrame;}
#endif
#if !UE_BUILD_SHIPPING
    int& LastTurnSummaryFrameWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->LastTurnSummaryFrame:LastTurnSummaryFrame;}
    int const& LastTurnSummaryFrameWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->LastTurnSummaryFrame:LastTurnSummaryFrame;}
#endif
    decltype(UChaosWheeledVehicleMovementComponent::VehicleSimulationPT)& VehicleSimulationPTWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->VehicleSimulationPT:VehicleSimulationPT;}
    decltype(UChaosWheeledVehicleMovementComponent::VehicleSimulationPT) const& VehicleSimulationPTWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->VehicleSimulationPT:VehicleSimulationPT;}
    decltype(UChaosWheeledVehicleMovementComponent::Wheels)& WheelsWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->Wheels:Wheels;}
    decltype(UChaosWheeledVehicleMovementComponent::Wheels) const& WheelsWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->Wheels:Wheels;}
    decltype(UChaosWheeledVehicleMovementComponent::WheelSetups)& WheelSetupsWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelSetups:WheelSetups;}
    decltype(UChaosWheeledVehicleMovementComponent::WheelSetups) const& WheelSetupsWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->WheelSetups:WheelSetups;}
    decltype(UChaosWheeledVehicleMovementComponent::CenterOfMassOverride)& CenterOfMassOverrideWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->CenterOfMassOverride:CenterOfMassOverride;}
    decltype(UChaosWheeledVehicleMovementComponent::CenterOfMassOverride) const& CenterOfMassOverrideWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->CenterOfMassOverride:CenterOfMassOverride;}
    decltype(UChaosWheeledVehicleMovementComponent::bEnableCenterOfMassOverride)& bEnableCenterOfMassOverrideWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bEnableCenterOfMassOverride:bEnableCenterOfMassOverride;}
    decltype(UChaosWheeledVehicleMovementComponent::bEnableCenterOfMassOverride) const& bEnableCenterOfMassOverrideWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bEnableCenterOfMassOverride:bEnableCenterOfMassOverride;}
    uint8 bThrottleAsBrakeWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bThrottleAsBrake:bThrottleAsBrake;}
    void SetOriginalThrottleAsBrake(uint8 Value) {if(UsesOriginalWheeledDomain())OriginalWheeledDomain->bThrottleAsBrake=Value;else bThrottleAsBrake=Value;}

    decltype(UChaosWheeledVehicleMovementComponent::bSuspensionEnabled)& bSuspensionEnabledWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bSuspensionEnabled:bSuspensionEnabled;}
    decltype(UChaosWheeledVehicleMovementComponent::bSuspensionEnabled) const& bSuspensionEnabledWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bSuspensionEnabled:bSuspensionEnabled;}
    decltype(UChaosWheeledVehicleMovementComponent::bWheelFrictionEnabled)& bWheelFrictionEnabledWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bWheelFrictionEnabled:bWheelFrictionEnabled;}
    decltype(UChaosWheeledVehicleMovementComponent::bWheelFrictionEnabled) const& bWheelFrictionEnabledWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bWheelFrictionEnabled:bWheelFrictionEnabled;}
    decltype(UChaosWheeledVehicleMovementComponent::ChassisHeight)& ChassisHeightWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->ChassisHeight:ChassisHeight;}
    decltype(UChaosWheeledVehicleMovementComponent::ChassisHeight) const& ChassisHeightWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->ChassisHeight:ChassisHeight;}
    decltype(UChaosWheeledVehicleMovementComponent::DragCoefficient)& DragCoefficientWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->DragCoefficient:DragCoefficient;}
    decltype(UChaosWheeledVehicleMovementComponent::DragCoefficient) const& DragCoefficientWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->DragCoefficient:DragCoefficient;}
    decltype(UChaosWheeledVehicleMovementComponent::EngineSetup)& EngineSetupWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->EngineSetup:EngineSetup;}
    decltype(UChaosWheeledVehicleMovementComponent::EngineSetup) const& EngineSetupWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->EngineSetup:EngineSetup;}
    decltype(UChaosWheeledVehicleMovementComponent::ThrottleInputRate)& ThrottleInputRateWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->ThrottleInputRate:ThrottleInputRate;}
    decltype(UChaosWheeledVehicleMovementComponent::ThrottleInputRate) const& ThrottleInputRateWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->ThrottleInputRate:ThrottleInputRate;}
    decltype(UChaosWheeledVehicleMovementComponent::TransmissionSetup)& TransmissionSetupWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->TransmissionSetup:TransmissionSetup;}
    decltype(UChaosWheeledVehicleMovementComponent::TransmissionSetup) const& TransmissionSetupWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->TransmissionSetup:TransmissionSetup;}
    decltype(UChaosWheeledVehicleMovementComponent::DifferentialSetup)& DifferentialSetupWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->DifferentialSetup:DifferentialSetup;}
    decltype(UChaosWheeledVehicleMovementComponent::DifferentialSetup) const& DifferentialSetupWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->DifferentialSetup:DifferentialSetup;}
    decltype(UChaosWheeledVehicleMovementComponent::bMechanicalSimEnabled)& bMechanicalSimEnabledWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bMechanicalSimEnabled:bMechanicalSimEnabled;}
    decltype(UChaosWheeledVehicleMovementComponent::bMechanicalSimEnabled) const& bMechanicalSimEnabledWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->bMechanicalSimEnabled:bMechanicalSimEnabled;}
    decltype(UChaosWheeledVehicleMovementComponent::SteeringInputRate)& SteeringInputRateWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->SteeringInputRate:SteeringInputRate;}
    decltype(UChaosWheeledVehicleMovementComponent::SteeringInputRate) const& SteeringInputRateWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->SteeringInputRate:SteeringInputRate;}
    decltype(UChaosWheeledVehicleMovementComponent::SteeringSetup)& SteeringSetupWheeledDomainStorage() {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->SteeringSetup:SteeringSetup;}
    decltype(UChaosWheeledVehicleMovementComponent::SteeringSetup) const& SteeringSetupWheeledDomainStorage() const {return UsesOriginalWheeledDomain()?OriginalWheeledDomain->SteeringSetup:SteeringSetup;}
private:
	FCriticalSection FrameInputProducerMutex;
	std::shared_ptr<Speed::Input::FInputStream> FrameInputStream;
	// Simulation-lane handle latched for this frame, including in-flight detach.
	std::shared_ptr<Speed::Input::FInputStream> ConsumedFrameInputStream;
	std::shared_ptr<Speed::Input::V2::FInputStream> FrameInputStreamV2;
	std::shared_ptr<Speed::Input::V2::FInputStream> ConsumedFrameInputStreamV2;
	std::optional<Speed::Input::V2::FReservationToken> InputReservationV2;
	std::optional<uint64> ReservedInputFrameV2;
	bool bInputFrameRejectedV2 = false;
	bool ConsumeProducedWheeledInputsV2(uint64 Frame, const std::shared_ptr<Speed::Input::V2::FInputStream>& Stream);
	bool bResetProducedWheeledInputs = false; // Protected by FrameInputProducerMutex.
	bool bProducedInputFrameOwned = false; // Simulation lane only, includes detach/reset frame.
	std::atomic<bool> bProducedInputAuthority{false};
	std::atomic<bool> bLegacyRemoteInputAuthority{false};

	FCriticalSection LegacyWheeledIngressMutex;
	std::shared_ptr<FLegacyWheeledIngress> LegacyWheeledIngress;
	std::unique_ptr<FLegacyWheeledOwner> LegacyWheeledOwner;
	bool bLegacyWheeledRetired = false;
	std::atomic<bool> bInputRetirementRequested{false};
	std::atomic<bool> bInputRetirementCompleted{false};
	bool bWheeledEndPlayTeardown = false; // GT-only, excludes physics-state recreation.
	bool bSpeedWorldAdapterRetiredForTeardown = false;
	bool bResumeAfterPhysicsRecreation = false; // GT-only, captured before teardown pauses the worker.
	bool bOwnedRecreationBoundary = false;
	TWeakObjectPtr<ASpeedPhysicsSimulation> RecreationDriver;
	bool RetireSpeedWorldAdapterBeforeTeardown();
	Speed::Input::ELegacyRemoteAdmission SubmitLegacyWheeledInput(uint64 SourceFrame, int32 ActivationFrame, const FWheeledInputState& Wire);
	bool PrepareLegacyWheeledInput(uint64 Frame);
	TOptional<uint64> PreparedCanonicalInputFrame;
	std::shared_ptr<const Speed::Input::V2::FOwnerInputSnapshot> CanonicalInputSnapshot;
	bool bCanonicalInputPending = false;

	mutable std::shared_ptr<const FCompletedWheeledInput> PreparedWheeledNetworkInput;
	Speed::Concurrency::TAtomicSharedPtr<const FCompletedWheeledInput> PublishedWheeledNetworkInput{};
	bool PrepareWheeledNetworkInput(uint64 Frame) const;
	void RestoreWheeledPhysicalInputFromState();
	void SyncWheeledPhysicalInputToState();


	//=========== Internal state variables for the movement component ===========
	SBaseGameState BaseGameState; // current game state of the component
	FBasePhysicsState BasePhysicsState; // current physics state of the component (replicated on network)
	FWheeledGameState WheeledGameState; // current wheeled game state of the component
	FWheeledPhysicsState WheeledPhysicsState; // current wheeled physics state of the component (replicated on network)
	FWheeledInputState WheeledPhysicalInput; // input state used for physics simulation (e.g. after being processed from the user input)
	std::atomic<bool> bGenericCameraInputEnabled{false};
	// One atomic word publishes all version/flags/axis fields coherently.
	std::atomic<uint32> PendingCameraInput{1};
	bool PublishHeldCameraInput(const FSpeedCarCameraPhysicalInput& Input);

	// BuildData is logically const: it captures input without advancing vehicle
	// state. Only this deferred camera mailbox/queue is mutable, never wheel state.
	mutable FCriticalSection PendingCameraInputMutex;
	mutable TArray<FPendingCameraInputCommand> PendingCameraInputCommands;
	mutable int32 LastCameraCaptureHistoryFrame = INDEX_NONE;
	mutable int32 LastCameraCaptureActivationFrame = INDEX_NONE;
	mutable FSpeedCarCameraPhysicalInput LastCapturedCameraInput;
	FSpeedCarCameraPhysicalInput CaptureNetworkCameraInput(int32 HistoryFrame, int32 ActivationFrame) const;
	void QueueCameraInputForFrame(int32 ActivationFrame, const FSpeedCarCameraPhysicalInput& Input) const;
	void QueueCameraInputLocked(int32 ActivationFrame, const FSpeedCarCameraPhysicalInput& Input) const;
	void ConsumeQueuedCameraInputsForFrame(int32 CurrentFrame);
#if WITH_DEV_AUTOMATION_TESTS
	mutable uint64 CameraMailboxReadCount = 0;
	uint64 CameraInputApplyCount = 0;
#endif
	FMatrix CarLocalInvI = FMatrix::Identity; // local inverse inertia tensor of the car body, expressed about the physical COM

	TArray<int32> RecordedBaseFrames;
	TArray<FBasePhysicsState> RecordedBaseStates; // array of recorded states for network correction
	TArray<FWheeledPhysicsState> RecordedWheeledStates; // array of recorded states for network correction

	float EngineFPS = 120.0; // physics simulation FPS (Hz)
	uint16 MinNbFramesBeforeCanMove = 0; // minimal number of frames before the component can move at the start of the simulation or after being reset (e.g. after a respawn)
	TArray<USSubBody*> SubBodies; // array of sub-bodies owned by this component (e.g. for a car body, this would be the hitbox and the wheels)
	TArray<USSubBody*> ExtSubBodies; // array of external sub-bodies

	TArray<SWheelGroundContact> PendingWheelContacts; // array of pending wheel ground contacts to be registered at the end of the frame

	// ========== Netcode variables ==========
    bool bLegacyNetworkHistoryExcluded = false; // GT lifecycle only, default legacy path unchanged
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

	int32 SinceCanMoveFrame = INDEX_NONE;
	bool bCanonicalSupportStartupInitializationRequested = false;
	bool bCanonicalSupportStartupPoseConfirmed = false;
	bool bSimTimelineWasCanMove = false;
	bool bSimTimelineWasGrounded = false;
	bool bSimTimelineHasGroundState = false;

	static constexpr int32 MaxPendingCameraInputs = 256;
#if WITH_DEV_AUTOMATION_TESTS
	friend class FSkycarFlipResetEligibilityTest;
	friend class FIAmSpeedWheeledInputQueueTest;
	friend class FIAmSpeedProducedWheeledInputBoundaryTest;
	friend class FIAmSpeedProducedInputWorkerOrderTest;
	friend class FIAmSpeedNativeCarsCamera1217;
 friend class FIAmSpeedNativeAnalyticContacts1221;
 friend class FIAmSpeedNativeAnalyticGrounded1368;
 friend class FIAmSpeedNativeAnalyticEightCars1534;
	friend class FSkyProducedJumpPowerslideWorkerTest;
	friend class FSkyProducedBooleanV2WorkerTest;
	friend class FSkyCameraWholePhysicalFrameCounterbalancedTest;
	friend class FSkyBumpWholePhysicalFrameDiagnosticTest;
	friend class FSkyCameraPacedPhysicalFrameCounterbalancedTest;
	friend class FSkyWaveDashAirPitchRearmTest;
	friend class FIAmSpeedProducedDeviceLifecycleTest;
	friend class FIAmSpeedControllerInputLifecycleTest;
	friend class FIAmSpeedWheelSimulationAdmissionTest;
	friend class FIAmSpeedCameraInputBoundaryTest;
	friend class FGenericCameraInputLifecycleTest;
	friend class FIAmSpeedWheeledInertiaCovarianceTest;
#endif
	std::atomic<bool> bTestInputOverrideEnabled{false};
	int TurnStartFrame = INDEX_NONE;
	int LastTurnSummaryFrame = INDEX_NONE;
};
