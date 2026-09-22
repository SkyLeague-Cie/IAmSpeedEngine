// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "CanonicalFrameDriver.h"
#include "SimulationFrameJournal.h"
#include "SimulationWorker.h"
#include "IAmSpeed/Input/InputSessionRegistry.h"
#include "IAmSpeed/Input/InputObservationChannel.h"
#include "SpeedSimulation.generated.h"

class USpeedWorldSubsystem;
struct FCanonicalFrameContext;
class ISpeedComponent;

struct FSimulationPresentationBinding
{
	uint64 OwnerStableId = 0;
	uint64 TargetStableId = 0;
	uint64 FirstFrame = 0;
};

/*
* ASpeedSimulation : Actor responsible for ticking the IAmSpeed Engine simulation.
* It is the one that calls Step() on the SpeedWorldSubsystem every physics frame.
*/
UCLASS(Abstract)
class IAMSPEED_API ASpeedSimulation : public AActor
{
	GENERATED_BODY()
#if WITH_DEV_AUTOMATION_TESTS
	friend class FSkyAIInputPauseRegistryTest;
#endif
#if WITH_DEV_AUTOMATION_TESTS
	friend class FIAmSpeedWheelSimulationAdmissionTest;
	friend class FIAmSpeedProducedInputWorkerOrderTest;
	friend class FSkyProducedJumpPowerslideWorkerTest;
	friend class FSkyProducedBooleanV2WorkerTest;
	friend class FIAmSpeedProducedDeviceLifecycleTest;
	friend class FIAmSpeedControllerInputLifecycleTest;
#endif

public:
	// Sets default values for this actor's properties
	ASpeedSimulation();

	void BeginPlay() override;
	void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	void Tick(float DeltaSeconds) override;
	void AsyncPhysicsTickActor(float DeltaTime, float SimTime) override final;
	void Simulate(const float& DeltaTime, const float& SimTime);
	/** Executes one complete canonical frame; false means no frame was committed. */
	bool StepCanonicalFrame(const FCanonicalFrameContext& Context);
	/** Frame-addressed boundary API; adapters seal inputs before FastSimulation. */
	/** Queues an opaque input for one run-stable component id and target frame. */
	bool QueueSimulationInput(uint64 ActivationFrame, uint64 TargetStableId, TConstArrayView<uint8> Payload);
	void SealSimulationInputs();
	const Speed::SimulationBoundary::FInputJournal& GetSimulationInputJournal() const { return InputJournal; }
	bool ReadLatestSimulationSnapshot(FSimulationSnapshot& OutSnapshot) const { return SnapshotBuffer.ReadLatest(OutSnapshot); }
	/** GT-only opt-in view, shared by every actor on this game frame. No live adapter reads. */
	bool ReadPresentationPose(uint64 StableId, FSimulationPoseConsumption& Out);
	bool ReadPresentationOutput(uint64 StableId, uint32 Channel, FSimulationPresentationOutput& Out);
	/** Thread-safe registry: canonical code copies shared handles under a short lock,
	 * then calls values-only producers without holding the registry lock. */
	bool RegisterPresentationProducer(TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe> Producer);
	void UnregisterPresentationProducer(const TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe>& Producer);
	/** GT lifecycle transaction: pause/acknowledge the owned lane, resolve stable
	 * identities and register before FirstFrame, then preserve the prior pause state.
	 * Legacy Unreal-async hosting is rejected because it has no owned-lane join. */
	TSharedPtr<ISimulationPresentationProducer, ESPMode::ThreadSafe> BindPresentationAtFrameBoundary(
		ISpeedComponent& OwnerComponent, ISpeedComponent& TargetComponent,
		TFunctionRef<TSharedPtr<ISimulationPresentationProducer, ESPMode::ThreadSafe>(
			const FSimulationPresentationBinding&)> Factory,
		TSharedPtr<ISimulationPresentationProducer, ESPMode::ThreadSafe> Previous = nullptr);
	/** Same transaction; nullptr explicitly binds only the owner (target id zero).
	 * A non-null target still requires a registered, distinct identity. */
	TSharedPtr<ISimulationPresentationProducer, ESPMode::ThreadSafe> BindPresentationAtFrameBoundary(
		ISpeedComponent& OwnerComponent, ISpeedComponent* TargetComponent,
		TFunctionRef<TSharedPtr<ISimulationPresentationProducer, ESPMode::ThreadSafe>(
			const FSimulationPresentationBinding&)> Factory,
		TSharedPtr<ISimulationPresentationProducer, ESPMode::ThreadSafe> Previous = nullptr);
	/** Reads an exact physics-side camera sample; never interpolates. */
	bool ReadCanonicalCameraSample(uint64 NumFrame, FCameraCanonicalSample& Out) const;
	/** Reads a contiguous exact-frame camera range; never interpolates. */
	bool ReadCanonicalCameraSamples(uint64 FirstFrame, uint64 LastFrame,
		TArray<FCameraCanonicalSample>& Out) const;
	/** Complete per-frame hashes for comparing two drivers after a run. */
	const Speed::SimulationBoundary::FFrameHashJournal& GetFrameHashes() const { return FrameHashes; }
	/**
	 * Queues an authoritative restore followed by deterministic replay through
	 * TargetFrameInclusive. The simulation owner consumes it at a frame boundary.
	 */
	bool RequestRollbackAndResimulation(
		const FSimulationSnapshot& Snapshot,
		uint64 TargetFrameInclusive);
	/** GT submits inert commands; only the worker constructs and owns producers. */
	Speed::Input::V2::ECommandAdmission SubmitInputSessionCommand(
		const Speed::Input::V2::FBoundaryCommandDescriptor& Command,
		std::shared_ptr<Speed::Input::V2::FRawAcquisitionJournal> Journal = {},
		std::shared_ptr<Speed::Input::V2::FInputObservationChannel> Observation = {},
		std::shared_ptr<Speed::Input::V2::FAIInputCommands> AI = {});
	uint64 AllocateInputCommandId() { check(IsInGameThread()); return NextInputCommandId == MAX_uint64 ? 0 : NextInputCommandId++; }
	std::optional<Speed::Input::V2::FBoundaryReceipt> ReadInputSessionReceipt(uint64 Id) const;
	bool AcknowledgeInputSessionReceipt(uint64 Id) { return InputSessionCommands->Acknowledge(Id); }
	std::shared_ptr<const Speed::Input::V2::FInputRegistryView> ReadInputRegistryView() const;
	std::shared_ptr<const Speed::Input::V2::FRegistryFrame> ReadCompletedInputFrame() const;
	uint64 GetInputWorkerGeneration() const { return InputWorkerGeneration; }
	bool InputOwnersRetiredAfterJoin(uint64 Generation) const
	{ return Generation == InputWorkerGeneration && bInputOwnerRetired.Load() && bInputOwnersClosedOnWorker.Load() && !SimulationWorker; }
	/** Pauses the owned execution lane without changing the canonical frame. */
	void PauseOwnedSimulation();
	/** Bounded lifecycle boundary. Timeout retains pause request; never resume
	 * automatically or access physical state after a non-acknowledged result. */
	ESimulationQuiescence TryPauseOwnedSimulation(uint32 TimeoutMilliseconds = 1000);
	bool ReadInputFirstFrameAtPausedBoundary(uint64& OutFrame);
	/** Teardown fallback: join the physical owner before releasing any input/actor lifetime. */
	bool JoinOwnedSimulationForInputTeardown();
	/** Resumes the owned lane from its current canonical frame. */
	void ResumeOwnedSimulation();
	/** GT-only: re-arms a paused controlled run after its actors/inputs were replaced. Keeps the world's canonical frame continuous. */
	void RestartControlledRun();
	/** After old actors/controllers are destroyed, before admitting replacement
	 * scenario inputs. Joins the old lane and renews its command generation. */
	bool PrepareControlledInputRun();
	/** Thread-safe pause witness used by gameplay integration tests and diagnostics. */
	bool IsOwnedSimulationPaused() const { return bOwnedSimulationPaused.Load(); }
	/** True when canonical frames are currently hosted by IAmSpeed's worker. */
	bool IsOwnedWorkerExecutionMode() const;
#if !UE_BUILD_SHIPPING
	/**
	 * Lets a simulation policy audit how many published frames a presentation
	 * actor observed between two game-thread updates. The base policy is silent.
	 */
	virtual void AuditPresentationFrameCadence(
		const AActor& Observer,
		float GameDeltaSeconds,
		uint64& InOutLastObservedFrame) {}
#endif
	bool ShouldTickIfViewportsOnly() const override { return true; }
	static unsigned int GetEngineFPS() { return EngineFPS; }
protected:
	/** Executes one driver pulse on whichever thread currently owns simulation. */
	virtual ESimulationWorkerResult DriveSimulation(float DeltaTime, float SimTime)
		PURE_VIRTUAL(ASpeedSimulation::DriveSimulation, return ESimulationWorkerResult::Failed;);
	/** Executes one interruptible worker pulse; FastSimulation specializes its granularity. */
	virtual ESimulationWorkerResult DriveOwnedWorkerPulse(float DeltaTime, float SimTime)
	{
		return DriveSimulation(DeltaTime, SimTime);
	}
	/** Defines subclass-specific pacing after one committed worker frame. */
	virtual void WaitBetweenFrames(FSimulationWorkerWaitContext& WaitContext)
		PURE_VIRTUAL(ASpeedSimulation::WaitBetweenFrames, );
	/** Lets a pacing policy discard deadlines invalidated by a restored timeline. */
	virtual void OnCanonicalTimelineRestored() {}
	/** Lifecycle hooks for subtype-owned pacing state. */
	virtual void OnOwnedWorkerStarting() {}
	virtual void OnOwnedSimulationPaused() {}
	virtual void OnOwnedSimulationResumed() {}
	virtual bool ShouldMeasureCallback() const { return true; }
	/** Optional pure camera evaluator hook; false leaves the sample unpublished. */
	virtual bool BuildCanonicalCameraSample(const FSimulationSnapshot& Snapshot,
		FCameraCanonicalSample& OutSample) const { return false; }
	bool EnsureSimulationWorldReady();
	/** Gates controlled runs until their scenario and Unreal bridge are ready. */
	ESimulationWorkerResult CheckCanonicalRunReadiness();
	/** FastSimulation requires a sealed controller; gameplay RealTime does not. */
	virtual bool RequiresCanonicalRunController() const { return false; }
	void InitializeCanonicalFrame(float SimTime);
	void ResetCanonicalFrame();
	void UpdateNumFrame(const float& SimTime);
	unsigned int NumFrame() const;
	static USpeedWorldSubsystem* GetSpeedWorldSubsystem(UWorld* World);
	/** Runs consecutive frames and advances the integer authority only on success. */
	bool RunCanonicalFrames(uint32 FrameCount);
	void RecordStepPerformance(double PhysicalFrameMilliseconds);
	float GetWarningFrameFraction() const;
	float RealDeltaTime = 0.0f;
	USpeedWorldSubsystem* SpeedWorldSubsystem = nullptr;

	unsigned int _NumFrame = 0;
	uint64 CanonicalNumFrame = 0;
	bool bCanonicalFrameInitialized = false;
	// Terminal publication failure cannot be cleared by pause/resume or replay.
	TAtomic<bool> bCanonicalPublicationTerminal{false};
	bool bStaticCollisionReadinessErrorReported = false;
	bool bInputConsumptionErrorReported = false;
	uint32 CanonicalReadyDelayPulsesObserved = 0;
	uint64 LastSlowFrameReport = 0;
	uint32 SlowFrameReportCount = 0;
	uint64 LastRecordedStepSerial = 0;
	uint64 PerformanceFrameCount = 0;
	uint64 PerformanceWarningFrameCount = 0;
	uint64 PerformanceIterationLimitCount = 0;
	double PerformanceTotalStepMilliseconds = 0.0;
	double PerformanceMaximumStepMilliseconds = 0.0;
	double LastPhysicalFrameMilliseconds = 0.0;
	static unsigned int EngineFPS; // The FPS at which the IAmSpeed Engine is running
	Speed::SimulationBoundary::FInputJournal InputJournal;
	Speed::SimulationBoundary::FSnapshotBuffer SnapshotBuffer;
	Speed::SimulationBoundary::FCameraSampleBuffer CameraSampleBuffer;
	Speed::SimulationBoundary::FFrameHashJournal FrameHashes;

private:
	bool bPublishPresentation = false;
	bool bPresentationBindingClosed = false;
	FCriticalSection PresentationProducerMutex;
	TArray<TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe>> PresentationProducers;
	Speed::SimulationBoundary::FPresentationFrameLatch PresentationLatch;
	struct FPendingRollbackRequest
	{
		FSimulationSnapshot Snapshot;
		uint64 TargetFrameInclusive = 0;
	};

	void RefreshExecutionMode();
	void TransitionExecutionMode(ESimulationExecutionMode NewMode);
	void StartOwnedWorkerIfReady();
	void StopOwnedWorker();
	ESimulationExecutionMode GetActiveExecutionMode() const;
	float GetCanonicalPulseSimTime() const;
	/** Applies at most one queued restore/replay transaction on the owning lane. */
	bool ProcessPendingRollbackRequest();

	ESimulationBoundaryResult ServiceInputSessionBoundary(bool bPauseRequested);
	void CloseInputSessionsOnWorker();
	std::shared_ptr<Speed::Input::V2::FInputSessionCommands> InputSessionCommands;
	std::unique_ptr<Speed::Input::V2::FInputSessionRegistry> InputSessionRegistry;
	std::shared_ptr<const Speed::Input::V2::FInputRegistryView> PublishedInputRegistry;
	std::shared_ptr<const Speed::Input::V2::FRegistryFrame> PublishedInputFrame;
	FCriticalSection InputSessionAdmissionMutex;
	std::vector<Speed::Input::V2::FInputSessionRegistry::FJournalService> PendingInputJournals;
	std::map<uint64, std::shared_ptr<Speed::Input::V2::FInputObservationChannel>> InputObservations;
	std::vector<std::pair<uint64, std::shared_ptr<Speed::Input::V2::FInputObservationChannel>>> PendingInputObservations;
	uint64 InputWorkerGeneration = 0, NextInputCommandId = 1;
	TAtomic<bool> bInputSessionAdmissionRequested = false;
	TAtomic<bool> bInputOwnersClosedOnWorker = false;
	uint64 NeutralizedInputRegistryVersion = MAX_uint64;
	bool bInputNeutralizedDuringPause = false;
	TUniquePtr<FSimulationWorker> SimulationWorker;
	FCriticalSection RollbackRequestMutex;
	TOptional<FPendingRollbackRequest> PendingRollbackRequest;
	TAtomic<uint8> ActiveExecutionModeValue =
		static_cast<uint8>(ESimulationExecutionMode::UnrealAsyncCallback);
	TAtomic<bool> bOwnedWorkerTerminal = false;
	// Permanent for this driver instance after input-owner teardown.
	TAtomic<bool> bInputOwnerRetired = false;
	/** Gameplay-owned pause, independent from the global automation pause. */
	TAtomic<bool> bOwnedSimulationPaused = false;
	double GameThreadAccumulatorSeconds = 0.0;
	uint8 HostTransitionDelayTicks = 0;
	bool bExecutionModeInitialized = false;
};
