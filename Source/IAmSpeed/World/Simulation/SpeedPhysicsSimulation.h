// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "IAmSpeed/Concurrency/AtomicSharedPtr.h"
#include "GameFramework/Actor.h"
#include "CanonicalFrameDriver.h"
#include "SimulationFrameJournal.h"
#include "SimulationWorker.h"
#include "IAmSpeed/World/Analytic/AnalyticCaseSatDiagnostics.h"
#include "FrameParallelTiming.h"
#include "CaseCallerCoverage276.h"
#include "IAmSpeed/Input/InputSessionRegistry.h"
#include "IAmSpeed/Input/InputObservationChannel.h"
#include <atomic>
#include <memory>
#if !UE_BUILD_SHIPPING
#include "NativeFrameWitnessRecorder.h"
#endif
#include "SpeedSimulation.h"
#include "SpeedPhysicsSimulation.generated.h"

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
* ASpeedPhysicsSimulation : Actor responsible for ticking the IAmSpeed Engine simulation.
* It is the one that calls Step() on the SpeedWorldSubsystem every physics frame.
*/
UCLASS(Abstract)
class IAMSPEED_API ASpeedPhysicsSimulation : public ASpeedSimulation
{
	GENERATED_BODY()
#if WITH_DEV_AUTOMATION_TESTS
	friend struct FLocalNormalCameraFixtureAccess;
#endif
#if WITH_DEV_AUTOMATION_TESTS
	friend class FSkyAIInputPauseRegistryTest;
#endif
#if WITH_DEV_AUTOMATION_TESTS
	friend class FIAmSpeedWheelSimulationAdmissionTest;
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
	friend class FIAmSpeedProducedDeviceLifecycleTest;
	friend class FIAmSpeedControllerInputLifecycleTest;
	friend class FIAmSpeedAdapterRespawnBoundaryTest;
#endif

public:
	// Sets default values for this actor's properties
	ASpeedPhysicsSimulation();

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
	/** GT config only; callbacks and state reads occur on the canonical owner lane.
	 * Retiring/replacing a sink allows an already latched frame to finish safely. */
	void SetReplaySink(TSharedPtr<ISimulationReplaySink, ESPMode::ThreadSafe> Sink);
    // GT ownership compare/exchange: never replace or detach another sink.
    // Old shared references are released outside the capture publication lock.
    bool TryReplaceReplaySink(const ISimulationReplaySink* Expected,
        TSharedPtr<ISimulationReplaySink, ESPMode::ThreadSafe> Replacement);
	/** GT-only opt-in view, shared by every actor on this game frame. No live adapter reads. */
	bool ReadPresentationPose(uint64 StableId, FSimulationPoseConsumption& Out);
	bool ReadPresentationOutput(uint64 StableId, uint32 Channel, FSimulationPresentationOutput& Out);
	/** Thread-safe registry: canonical code copies shared handles under a short lock,
	 * then calls values-only producers without holding the registry lock. */
	bool RegisterPresentationProducer(TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe> Producer);
	void UnregisterPresentationProducer(const TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe>& Producer);
	/** Simulation policy, independent of pacing and rollback/resimulation. */
	virtual bool AllowsCameraPresentation() const { return true; }
	TSharedPtr<const TAtomic<bool>, ESPMode::ThreadSafe> GetPresentationRetirementWitness() const { return PresentationRetiredAfterJoin; }
	/** GT lifecycle removal and adapter release inside the acknowledged join. */
	bool UnbindPresentationAtFrameBoundary(
		const TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe>& Producer,
		TFunctionRef<void()> ReleaseAtBoundary);
	/** GT lifecycle transaction: pause/acknowledge the owned lane, resolve stable
	 * identities and register before FirstFrame, then preserve the prior pause state.
	 * Legacy Unreal-async hosting is rejected because it has no owned-lane join. */
	TSharedPtr<ISimulationPresentationProducer, ESPMode::ThreadSafe> BindPresentationAtFrameBoundary(
		ISpeedComponent& OwnerComponent, ISpeedComponent& TargetComponent,
		TFunctionRef<TSharedPtr<ISimulationPresentationProducer, ESPMode::ThreadSafe>(
			const FSimulationPresentationBinding&)> Factory,
		TSharedPtr<ISimulationPresentationProducer, ESPMode::ThreadSafe> Previous = nullptr,
		ISimulationPresentationAdapterCommit* AdapterCommit = nullptr);
	/** Same transaction; nullptr explicitly binds only the owner (target id zero).
	 * A non-null target still requires a registered, distinct identity. */
	TSharedPtr<ISimulationPresentationProducer, ESPMode::ThreadSafe> BindPresentationAtFrameBoundary(
		ISpeedComponent& OwnerComponent, ISpeedComponent* TargetComponent,
		TFunctionRef<TSharedPtr<ISimulationPresentationProducer, ESPMode::ThreadSafe>(
			const FSimulationPresentationBinding&)> Factory,
		TSharedPtr<ISimulationPresentationProducer, ESPMode::ThreadSafe> Previous = nullptr,
		ISimulationPresentationAdapterCommit* AdapterCommit = nullptr);
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
	/** Transfers exclusive adapter-registry access to the game thread. Unlike
	 * TryPauseOwnedSimulation, this also parks lifecycle command service. */
	ESimulationQuiescence TrySuspendOwnedBoundaryService(uint32 TimeoutMilliseconds = 1000);
	void ResumeOwnedBoundaryService();
	bool IsOwnedBoundaryServiceSuspended() const;
	bool ReadInputFirstFrameAtPausedBoundary(uint64& OutFrame);
	/** Teardown fallback: join the physical owner before releasing any input/actor lifetime. */
	bool JoinOwnedSimulationForInputTeardown();
#if !UE_BUILD_SHIPPING
    /** Observational case identity. GT only after an owned-worker join; never a physics gate. */
    bool ConfigureCaseSatDiagnosticsAfterJoin(int32 NativeCaseId, int32 BatchIndex, uint64 WorldGeneration);
#endif
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
    /** GT-only recording-resource preparation before BeginPlay/first physics
     * frame. Opaque token is scoped to this actor incarnation. Does not change
     * user pause, actor tick enablement, execution-mode CVars or physics state.
     * Caller retains a finite preparation/cancellation policy and releases the
     * matching token explicitly; timeout/unknown never imply a resumed lane.
     * No instance data is added: native subclass layout remains unchanged. */
    uint64 TryAcquireReplayStartupHoldBeforeBeginPlay();
    bool ReleaseReplayStartupHold(uint64 Token);
    bool IsReplayStartupHeld(uint64 Token) const;
#if !UE_BUILD_SHIPPING
	/**
	 * Lets a simulation policy audit how many published frames a presentation
	 * actor observed between two game-thread updates. The base policy is silent.
	 */
	virtual void AuditPresentationFrameCadence(
		const AActor& Observer,
		float GameDeltaSeconds,
		Speed::PublicationCadence::FObserver& Observation) {}
#endif
	bool ShouldTickIfViewportsOnly() const override { return true; }
	static unsigned int GetEngineFPS() { return EngineFPS; }
protected:
#if !UE_BUILD_SHIPPING
    virtual Speed::PublicationCadence::FTiming GetCanonicalPublicationTiming(
        const FCanonicalFrameContext& Context) const { return {}; }
#endif
	/** Executes one driver pulse on whichever thread currently owns simulation. */
	virtual ESimulationWorkerResult DriveSimulation(float DeltaTime, float SimTime)
		PURE_VIRTUAL(ASpeedPhysicsSimulation::DriveSimulation, return ESimulationWorkerResult::Failed;);
	/** Executes one interruptible worker pulse; FastSimulation specializes its granularity. */
	virtual ESimulationWorkerResult DriveOwnedWorkerPulse(float DeltaTime, float SimTime)
	{
		return DriveSimulation(DeltaTime, SimTime);
	}
	/** Defines subclass-specific pacing after one committed worker frame. */
	virtual void WaitBetweenFrames(FSimulationWorkerWaitContext& WaitContext)
		PURE_VIRTUAL(ASpeedPhysicsSimulation::WaitBetweenFrames, );
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
#if !UE_BUILD_SHIPPING
    void BeginIndependentFastCoverage276() noexcept;
    void EndIndependentFastCoverage276(bool bCompleted, const TCHAR* Reason) noexcept;
    void BeginIndependentRealTimeCoverage696() noexcept;
    void EndIndependentRealTimeCoverage696(bool bCompleted, const TCHAR* Reason) noexcept;
#endif
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
#if !UE_BUILD_SHIPPING
    Speed::Analytic::CaseSatDiagnostics::FCaseIdentity CaseSatDiagnosticIdentity;
    Speed::FNativeFrameWitnessCaseIdentity NativeWitnessCase;
    uint64 NativeWitnessEpoch=0;

    uint64 CaseSatDiagnosticEpoch = 0;
    Speed::ParallelTiming::FCaseRecorder ParallelFrameTiming;
    Speed::Diagnostics::CaseCoverage276::FDriverLedger IndependentFastCoverage276;
#endif
	bool bPublishPresentation = false;
	TSharedRef<TAtomic<bool>, ESPMode::ThreadSafe> PresentationRetiredAfterJoin = MakeShared<TAtomic<bool>, ESPMode::ThreadSafe>(false);
	bool bPresentationBindingClosed = false;
	TAtomic<bool> bReplayCaptureRequested{false};
	FCriticalSection ReplaySinkMutex;
	TSharedPtr<ISimulationReplaySink, ESPMode::ThreadSafe> ReplaySink;
	void ServiceReplayCapture(const FSimulationSnapshot& Snapshot) noexcept;
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
	Speed::Concurrency::TAtomicSharedPtr<const Speed::Input::V2::FInputRegistryView> PublishedInputRegistry{};
	Speed::Concurrency::TAtomicSharedPtr<const Speed::Input::V2::FRegistryFrame> PublishedInputFrame{};
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
	// Game-thread lease count. A late worker ACK after timeout is not ownership;
	// only a successful TrySuspend call increments this count.
	uint32 OwnedBoundarySuspendDepth = 0;
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
