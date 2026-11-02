// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "IAmSpeed/Concurrency/AtomicSharedPtr.h"
#include "GameFramework/Actor.h"
#include "CanonicalFrameDriver.h"
#include "SimulationFrameJournal.h"
#include "CompletedSimulationPublication.h"
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
#include <atomic>
#include <memory>
#include "SpeedSimulation.h"
#include "NativeEpochStorageRoute.h"
#include "SpeedPhysicsSimulation.generated.h"
namespace Speed::Netcode::Prediction {class FPrivateRoadPhysicsWorld;}

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
    friend class Speed::Netcode::Prediction::FPrivateRoadPhysicsWorld;
#if WITH_DEV_AUTOMATION_TESTS
	friend struct FLocalNormalCameraFixtureAccess;
	friend struct FLocalFastBatchFixtureAccess;
	friend class FIAmSpeedCollisionMatrix042Test;
#endif
#if WITH_DEV_AUTOMATION_TESTS
	friend class FSkyAIInputPauseRegistryTest;
#endif
#if WITH_DEV_AUTOMATION_TESTS
	friend class FIAmSpeedWheelSimulationAdmissionTest;
	friend class FIAmSpeedProducedInputWorkerOrderTest;
	friend class FIAmSpeedPresentationClientPendingWorldTest;
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
	const Speed::SimulationBoundary::FInputJournal& GetSimulationInputJournal() const { return InputJournalDriverStorage(); }
	bool ReadLatestSimulationSnapshot(FSimulationSnapshot& OutSnapshot) const { return SnapshotBufferDriverStorage().ReadLatest(OutSnapshot); }
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
	TSharedPtr<const TAtomic<bool>, ESPMode::ThreadSafe> GetPresentationRetirementWitness() const { return PresentationRetiredAfterJoinDriverStorage(); }
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
	const Speed::SimulationBoundary::FFrameHashJournal& GetFrameHashes() const { return FrameHashesDriverStorage(); }
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
		std::shared_ptr<Speed::Input::V2::FAIInputCommands> AI = {},
        std::shared_ptr<Speed::Netcode::FExactRemoteInputIngress> Remote = {});
    /** Dedicated authority only, frozen before any worker or input admission. */
    bool ConfigureExactRemoteInputHost(uint32 RequiredSessions);
    /** Explicit policy mode for original Device frames; no default activation or client-selected policy. */
    bool ConfigureOriginalDeviceRemoteInputHost(uint32 RequiredSessions, Speed::Input::V2::EExactRemoteAdmission Admission);
	uint64 AllocateInputCommandId() { check(IsInGameThread()); return NextInputCommandIdDriverStorage() == MAX_uint64 ? 0 : NextInputCommandIdDriverStorage()++; }
	std::optional<Speed::Input::V2::FBoundaryReceipt> ReadInputSessionReceipt(uint64 Id) const;
	bool AcknowledgeInputSessionReceipt(uint64 Id) { return InputSessionCommandsDriverStorage()->Acknowledge(Id); }
	std::shared_ptr<const Speed::Input::V2::FInputRegistryView> ReadInputRegistryView() const;
	std::shared_ptr<const Speed::Input::V2::FRegistryFrame> ReadCompletedInputFrame() const;
	/** Retain one immutable completed world/input pair; no fullstate, branch, current-input or restore capability. */
	std::shared_ptr<const Speed::FCompletedSimulationPublication> ReadCompletedWorldAndInputFrame() const;
	uint64 GetInputWorkerGeneration() const { return InputWorkerGenerationDriverStorage(); }
	bool HasOwnedWorkerForPresentationAdmission() const { check(IsInGameThread()); return SimulationWorkerDriverStorage() != nullptr; }
	bool InputOwnersRetiredAfterJoin(uint64 Generation) const
	{ return Generation == InputWorkerGenerationDriverStorage() && bInputOwnerRetiredDriverStorage().Load() && bInputOwnersClosedOnWorkerDriverStorage().Load() && !SimulationWorkerDriverStorage(); }
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
	bool IsOwnedSimulationPaused() const { return bOwnedSimulationPausedDriverStorage().Load(); }
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
    // Subtypes with independently owned physical/input state never accept the
    // old pose/mechanics-only snapshot rollback, even without a V2 registry.
    virtual bool AllowsLegacySnapshotRollback() const {return true;}
    virtual bool AllowsNativeDriverExecution() const {return true;}
#if !UE_BUILD_SHIPPING
    virtual Speed::PublicationCadence::FTiming GetCanonicalPublicationTiming(
        const FCanonicalFrameContext& Context) const { return {}; }
#endif
	/** Executes one driver pulse on whichever thread currently owns simulation. */
	virtual ESimulationWorkerResult DriveSimulation(float DeltaTime, float SimTime)
		PURE_VIRTUAL(ASpeedPhysicsSimulation::DriveSimulation, return ESimulationWorkerResult::Failed;);
    /** Owner-only completion request at a quiescent boundary, before input admission. */
    virtual bool CompleteOwnedWorkerAtBoundary() { return false; }
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
    /** Rebase wall-clock pacing after input starvation; physical time stays fixed. */
    virtual void OnOwnedInputWaitEnded() {}
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
	/** Production remains 300 Hz. The matrix opts in only after its owner joined. */
	FCanonicalFrameContext MakeCanonicalFrameContext(uint64 Frame) const;
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
	// No public runtime selector or cross-rate rollback admission is introduced.
	uint32 CollisionMatrixTestHz = 0;
	uint64 CollisionMatrixTestFirstFrame = 0;
#endif
#if !UE_BUILD_SHIPPING
    void BeginIndependentFastCoverage276() noexcept;
    void EndIndependentFastCoverage276(bool bCompleted, const TCHAR* Reason) noexcept;
    void BeginIndependentRealTimeCoverage696() noexcept;
    void EndIndependentRealTimeCoverage696(bool bCompleted, const TCHAR* Reason) noexcept;
#endif
	void RecordStepPerformance(double PhysicalFrameMilliseconds);
	float GetWarningFrameFraction() const;
private:
    // A fresh original driver owns its controls, journals, registries, frame
    // counters and publication tail together. Escaped legacy references keep
    // addressing the original legacy fields below, never this allocation.
	struct FPendingRollbackRequest
	{
		FSimulationSnapshot Snapshot;
		uint64 TargetFrameInclusive = 0;
	};
    struct FOriginalDriverStorage
    {
        float RealDeltaTime= 0.0f;
        USpeedWorldSubsystem* SpeedWorldSubsystem= nullptr;
        unsigned int _NumFrame= 0;
        uint64 CanonicalNumFrame= 0;
        bool bCanonicalFrameInitialized= false;
        TAtomic<bool> bCanonicalPublicationTerminal{false};
        bool bStaticCollisionReadinessErrorReported= false;
        bool bInputConsumptionErrorReported= false;
        uint32 CanonicalReadyDelayPulsesObserved= 0;
        uint64 LastSlowFrameReport= 0;
        uint32 SlowFrameReportCount= 0;
        uint64 LastRecordedStepSerial= 0;
        uint64 PerformanceFrameCount= 0;
        uint64 PerformanceWarningFrameCount= 0;
        uint64 PerformanceIterationLimitCount= 0;
        double PerformanceTotalStepMilliseconds= 0.0;
        double PerformanceMaximumStepMilliseconds= 0.0;
        double LastPhysicalFrameMilliseconds= 0.0;
        Speed::SimulationBoundary::FInputJournal InputJournal;
        Speed::SimulationBoundary::FSnapshotBuffer SnapshotBuffer;
        Speed::SimulationBoundary::FCameraSampleBuffer CameraSampleBuffer;
        Speed::SimulationBoundary::FFrameHashJournal FrameHashes;
#if !UE_BUILD_SHIPPING
        Speed::Analytic::CaseSatDiagnostics::FCaseIdentity CaseSatDiagnosticIdentity;
#endif
#if !UE_BUILD_SHIPPING
        Speed::FNativeFrameWitnessCaseIdentity NativeWitnessCase;
#endif
#if !UE_BUILD_SHIPPING
        uint64 NativeWitnessEpoch=0;
#endif
#if !UE_BUILD_SHIPPING
        uint64 CaseSatDiagnosticEpoch= 0;
#endif
#if !UE_BUILD_SHIPPING
        Speed::ParallelTiming::FCaseRecorder ParallelFrameTiming;
#endif
#if !UE_BUILD_SHIPPING
        Speed::Diagnostics::CaseCoverage276::FDriverLedger IndependentFastCoverage276;
#endif
        bool bPublishPresentation= false;
        bool bPublishCompletedWorldInput= false;
        TSharedRef<TAtomic<bool>, ESPMode::ThreadSafe> PresentationRetiredAfterJoin= MakeShared<TAtomic<bool>, ESPMode::ThreadSafe>(false);
        bool bPresentationBindingClosed= false;
        TAtomic<bool> bReplayCaptureRequested{false};
        FCriticalSection ReplaySinkMutex;
        TSharedPtr<ISimulationReplaySink, ESPMode::ThreadSafe> ReplaySink;
        FCriticalSection PresentationProducerMutex;
        TArray<TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe>> PresentationProducers;
        Speed::SimulationBoundary::FPresentationFrameLatch PresentationLatch;
        std::shared_ptr<Speed::Input::V2::FInputSessionCommands> InputSessionCommands;
        std::unique_ptr<Speed::Input::V2::FInputSessionRegistry> InputSessionRegistry;
        Speed::Concurrency::TAtomicSharedPtr<const Speed::Input::V2::FInputRegistryView> PublishedInputRegistry{};
        Speed::Concurrency::TAtomicSharedPtr<const Speed::Input::V2::FRegistryFrame> PublishedInputFrame{};
        Speed::Concurrency::TAtomicSharedPtr<const Speed::FCompletedSimulationPublication> PublishedCompletedWorldInput{};
        FCriticalSection InputSessionAdmissionMutex;
        std::vector<Speed::Input::V2::FInputSessionRegistry::FJournalService> PendingInputJournals;
        std::map<uint64, std::shared_ptr<Speed::Input::V2::FInputObservationChannel>> InputObservations;
        std::vector<std::pair<uint64, std::shared_ptr<Speed::Input::V2::FInputObservationChannel>>> PendingInputObservations;
        uint64 InputWorkerGeneration= 0;
        uint64 NextInputCommandId= 1;
        TAtomic<bool> bInputSessionAdmissionRequested= false;
        uint32 RequiredExactRemoteInputSessions= 0;
        Speed::Input::V2::EExactRemoteAdmission ConfiguredRemoteInputAdmission= Speed::Input::V2::EExactRemoteAdmission::Closed;
        bool bExactRemoteInputWaiting= false;
        bool bInputCommandBoundaryDirty= false;
        TAtomic<bool> bInputOwnersClosedOnWorker= false;
        uint64 NeutralizedInputRegistryVersion= MAX_uint64;
        bool bInputNeutralizedDuringPause= false;
        TUniquePtr<FSimulationWorker> SimulationWorker;
        uint32 OwnedBoundarySuspendDepth= 0;
        FCriticalSection RollbackRequestMutex;
        TOptional<FPendingRollbackRequest> PendingRollbackRequest;
        TAtomic<uint8> ActiveExecutionModeValue=
		static_cast<uint8>(ESimulationExecutionMode::UnrealAsyncCallback);
        TAtomic<bool> bOwnedWorkerTerminal= false;
        TAtomic<bool> bInputOwnerRetired= false;
        TAtomic<bool> bOwnedSimulationPaused= false;
        double GameThreadAccumulatorSeconds= 0.0;
        uint8 HostTransitionDelayTicks= 0;
        bool bExecutionModeInitialized= false;
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
        uint32 CollisionMatrixTestHz= 0;
#endif
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
        uint64 CollisionMatrixTestFirstFrame= 0;
#endif
    };
    std::shared_ptr<FOriginalDriverStorage> OriginalDriverStorage;
    std::weak_ptr<const Speed::NativeEpoch::IOriginalStorageRoute> OriginalDriverRoute;
    bool UsesOriginalDriverStorage() const
    {
        if(!IsInGameThread())return false;
        const auto Route=OriginalDriverRoute.lock();
        return OriginalDriverStorage&&Route&&Route->SelectsOriginalStorage();
    }
    bool BindOriginalDriverStorage(const std::shared_ptr<const Speed::NativeEpoch::IOriginalStorageRoute>& Route);

protected:
    // Selection diagnostic only: exposes neither the private store nor its binder.
    bool HasOriginalDriverExecutionRoute() const {return UsesOriginalDriverStorage();}
    float& RealDeltaTimeDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->RealDeltaTime:RealDeltaTime;}
    float const& RealDeltaTimeDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->RealDeltaTime:RealDeltaTime;}
    USpeedWorldSubsystem*& SpeedWorldSubsystemDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->SpeedWorldSubsystem:SpeedWorldSubsystem;}
    USpeedWorldSubsystem* const& SpeedWorldSubsystemDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->SpeedWorldSubsystem:SpeedWorldSubsystem;}
    unsigned int& _NumFrameDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->_NumFrame:_NumFrame;}
    unsigned int const& _NumFrameDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->_NumFrame:_NumFrame;}
    uint64& CanonicalNumFrameDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->CanonicalNumFrame:CanonicalNumFrame;}
    uint64 const& CanonicalNumFrameDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->CanonicalNumFrame:CanonicalNumFrame;}
    bool& bCanonicalFrameInitializedDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->bCanonicalFrameInitialized:bCanonicalFrameInitialized;}
    bool const& bCanonicalFrameInitializedDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->bCanonicalFrameInitialized:bCanonicalFrameInitialized;}
    TAtomic<bool>& bCanonicalPublicationTerminalDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->bCanonicalPublicationTerminal:bCanonicalPublicationTerminal;}
    TAtomic<bool> const& bCanonicalPublicationTerminalDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->bCanonicalPublicationTerminal:bCanonicalPublicationTerminal;}
    bool& bStaticCollisionReadinessErrorReportedDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->bStaticCollisionReadinessErrorReported:bStaticCollisionReadinessErrorReported;}
    bool const& bStaticCollisionReadinessErrorReportedDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->bStaticCollisionReadinessErrorReported:bStaticCollisionReadinessErrorReported;}
    bool& bInputConsumptionErrorReportedDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->bInputConsumptionErrorReported:bInputConsumptionErrorReported;}
    bool const& bInputConsumptionErrorReportedDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->bInputConsumptionErrorReported:bInputConsumptionErrorReported;}
    uint32& CanonicalReadyDelayPulsesObservedDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->CanonicalReadyDelayPulsesObserved:CanonicalReadyDelayPulsesObserved;}
    uint32 const& CanonicalReadyDelayPulsesObservedDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->CanonicalReadyDelayPulsesObserved:CanonicalReadyDelayPulsesObserved;}
    uint64& LastSlowFrameReportDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->LastSlowFrameReport:LastSlowFrameReport;}
    uint64 const& LastSlowFrameReportDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->LastSlowFrameReport:LastSlowFrameReport;}
    uint32& SlowFrameReportCountDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->SlowFrameReportCount:SlowFrameReportCount;}
    uint32 const& SlowFrameReportCountDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->SlowFrameReportCount:SlowFrameReportCount;}
    uint64& LastRecordedStepSerialDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->LastRecordedStepSerial:LastRecordedStepSerial;}
    uint64 const& LastRecordedStepSerialDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->LastRecordedStepSerial:LastRecordedStepSerial;}
    uint64& PerformanceFrameCountDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->PerformanceFrameCount:PerformanceFrameCount;}
    uint64 const& PerformanceFrameCountDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->PerformanceFrameCount:PerformanceFrameCount;}
    uint64& PerformanceWarningFrameCountDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->PerformanceWarningFrameCount:PerformanceWarningFrameCount;}
    uint64 const& PerformanceWarningFrameCountDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->PerformanceWarningFrameCount:PerformanceWarningFrameCount;}
    uint64& PerformanceIterationLimitCountDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->PerformanceIterationLimitCount:PerformanceIterationLimitCount;}
    uint64 const& PerformanceIterationLimitCountDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->PerformanceIterationLimitCount:PerformanceIterationLimitCount;}
    double& PerformanceTotalStepMillisecondsDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->PerformanceTotalStepMilliseconds:PerformanceTotalStepMilliseconds;}
    double const& PerformanceTotalStepMillisecondsDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->PerformanceTotalStepMilliseconds:PerformanceTotalStepMilliseconds;}
    double& PerformanceMaximumStepMillisecondsDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->PerformanceMaximumStepMilliseconds:PerformanceMaximumStepMilliseconds;}
    double const& PerformanceMaximumStepMillisecondsDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->PerformanceMaximumStepMilliseconds:PerformanceMaximumStepMilliseconds;}
    double& LastPhysicalFrameMillisecondsDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->LastPhysicalFrameMilliseconds:LastPhysicalFrameMilliseconds;}
    double const& LastPhysicalFrameMillisecondsDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->LastPhysicalFrameMilliseconds:LastPhysicalFrameMilliseconds;}
    Speed::SimulationBoundary::FInputJournal& InputJournalDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->InputJournal:InputJournal;}
    Speed::SimulationBoundary::FInputJournal const& InputJournalDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->InputJournal:InputJournal;}
    Speed::SimulationBoundary::FSnapshotBuffer& SnapshotBufferDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->SnapshotBuffer:SnapshotBuffer;}
    Speed::SimulationBoundary::FSnapshotBuffer const& SnapshotBufferDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->SnapshotBuffer:SnapshotBuffer;}
    Speed::SimulationBoundary::FCameraSampleBuffer& CameraSampleBufferDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->CameraSampleBuffer:CameraSampleBuffer;}
    Speed::SimulationBoundary::FCameraSampleBuffer const& CameraSampleBufferDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->CameraSampleBuffer:CameraSampleBuffer;}
    Speed::SimulationBoundary::FFrameHashJournal& FrameHashesDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->FrameHashes:FrameHashes;}
    Speed::SimulationBoundary::FFrameHashJournal const& FrameHashesDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->FrameHashes:FrameHashes;}
#if !UE_BUILD_SHIPPING
    Speed::Analytic::CaseSatDiagnostics::FCaseIdentity& CaseSatDiagnosticIdentityDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->CaseSatDiagnosticIdentity:CaseSatDiagnosticIdentity;}
    Speed::Analytic::CaseSatDiagnostics::FCaseIdentity const& CaseSatDiagnosticIdentityDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->CaseSatDiagnosticIdentity:CaseSatDiagnosticIdentity;}
#endif
#if !UE_BUILD_SHIPPING
    Speed::FNativeFrameWitnessCaseIdentity& NativeWitnessCaseDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->NativeWitnessCase:NativeWitnessCase;}
    Speed::FNativeFrameWitnessCaseIdentity const& NativeWitnessCaseDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->NativeWitnessCase:NativeWitnessCase;}
#endif
#if !UE_BUILD_SHIPPING
    uint64& NativeWitnessEpochDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->NativeWitnessEpoch:NativeWitnessEpoch;}
    uint64 const& NativeWitnessEpochDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->NativeWitnessEpoch:NativeWitnessEpoch;}
#endif
#if !UE_BUILD_SHIPPING
    uint64& CaseSatDiagnosticEpochDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->CaseSatDiagnosticEpoch:CaseSatDiagnosticEpoch;}
    uint64 const& CaseSatDiagnosticEpochDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->CaseSatDiagnosticEpoch:CaseSatDiagnosticEpoch;}
#endif
#if !UE_BUILD_SHIPPING
    Speed::ParallelTiming::FCaseRecorder& ParallelFrameTimingDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->ParallelFrameTiming:ParallelFrameTiming;}
    Speed::ParallelTiming::FCaseRecorder const& ParallelFrameTimingDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->ParallelFrameTiming:ParallelFrameTiming;}
#endif
#if !UE_BUILD_SHIPPING
    Speed::Diagnostics::CaseCoverage276::FDriverLedger& IndependentFastCoverage276DriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->IndependentFastCoverage276:IndependentFastCoverage276;}
    Speed::Diagnostics::CaseCoverage276::FDriverLedger const& IndependentFastCoverage276DriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->IndependentFastCoverage276:IndependentFastCoverage276;}
#endif
    bool& bPublishPresentationDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->bPublishPresentation:bPublishPresentation;}
    bool const& bPublishPresentationDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->bPublishPresentation:bPublishPresentation;}
    bool& bPublishCompletedWorldInputDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->bPublishCompletedWorldInput:bPublishCompletedWorldInput;}
    bool const& bPublishCompletedWorldInputDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->bPublishCompletedWorldInput:bPublishCompletedWorldInput;}
    TSharedRef<TAtomic<bool>, ESPMode::ThreadSafe>& PresentationRetiredAfterJoinDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->PresentationRetiredAfterJoin:PresentationRetiredAfterJoin;}
    TSharedRef<TAtomic<bool>, ESPMode::ThreadSafe> const& PresentationRetiredAfterJoinDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->PresentationRetiredAfterJoin:PresentationRetiredAfterJoin;}
    bool& bPresentationBindingClosedDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->bPresentationBindingClosed:bPresentationBindingClosed;}
    bool const& bPresentationBindingClosedDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->bPresentationBindingClosed:bPresentationBindingClosed;}
    TAtomic<bool>& bReplayCaptureRequestedDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->bReplayCaptureRequested:bReplayCaptureRequested;}
    TAtomic<bool> const& bReplayCaptureRequestedDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->bReplayCaptureRequested:bReplayCaptureRequested;}
    FCriticalSection& ReplaySinkMutexDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->ReplaySinkMutex:ReplaySinkMutex;}
    FCriticalSection const& ReplaySinkMutexDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->ReplaySinkMutex:ReplaySinkMutex;}
    TSharedPtr<ISimulationReplaySink, ESPMode::ThreadSafe>& ReplaySinkDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->ReplaySink:ReplaySink;}
    TSharedPtr<ISimulationReplaySink, ESPMode::ThreadSafe> const& ReplaySinkDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->ReplaySink:ReplaySink;}
    FCriticalSection& PresentationProducerMutexDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->PresentationProducerMutex:PresentationProducerMutex;}
    FCriticalSection const& PresentationProducerMutexDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->PresentationProducerMutex:PresentationProducerMutex;}
    TArray<TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe>>& PresentationProducersDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->PresentationProducers:PresentationProducers;}
    TArray<TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe>> const& PresentationProducersDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->PresentationProducers:PresentationProducers;}
    Speed::SimulationBoundary::FPresentationFrameLatch& PresentationLatchDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->PresentationLatch:PresentationLatch;}
    Speed::SimulationBoundary::FPresentationFrameLatch const& PresentationLatchDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->PresentationLatch:PresentationLatch;}
    std::shared_ptr<Speed::Input::V2::FInputSessionCommands>& InputSessionCommandsDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->InputSessionCommands:InputSessionCommands;}
    std::shared_ptr<Speed::Input::V2::FInputSessionCommands> const& InputSessionCommandsDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->InputSessionCommands:InputSessionCommands;}
    std::unique_ptr<Speed::Input::V2::FInputSessionRegistry>& InputSessionRegistryDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->InputSessionRegistry:InputSessionRegistry;}
    std::unique_ptr<Speed::Input::V2::FInputSessionRegistry> const& InputSessionRegistryDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->InputSessionRegistry:InputSessionRegistry;}
    Speed::Concurrency::TAtomicSharedPtr<const Speed::Input::V2::FInputRegistryView>& PublishedInputRegistryDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->PublishedInputRegistry:PublishedInputRegistry;}
    Speed::Concurrency::TAtomicSharedPtr<const Speed::Input::V2::FInputRegistryView> const& PublishedInputRegistryDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->PublishedInputRegistry:PublishedInputRegistry;}
    Speed::Concurrency::TAtomicSharedPtr<const Speed::Input::V2::FRegistryFrame>& PublishedInputFrameDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->PublishedInputFrame:PublishedInputFrame;}
    Speed::Concurrency::TAtomicSharedPtr<const Speed::Input::V2::FRegistryFrame> const& PublishedInputFrameDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->PublishedInputFrame:PublishedInputFrame;}
    Speed::Concurrency::TAtomicSharedPtr<const Speed::FCompletedSimulationPublication>& PublishedCompletedWorldInputDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->PublishedCompletedWorldInput:PublishedCompletedWorldInput;}
    Speed::Concurrency::TAtomicSharedPtr<const Speed::FCompletedSimulationPublication> const& PublishedCompletedWorldInputDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->PublishedCompletedWorldInput:PublishedCompletedWorldInput;}
    FCriticalSection& InputSessionAdmissionMutexDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->InputSessionAdmissionMutex:InputSessionAdmissionMutex;}
    FCriticalSection const& InputSessionAdmissionMutexDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->InputSessionAdmissionMutex:InputSessionAdmissionMutex;}
    std::vector<Speed::Input::V2::FInputSessionRegistry::FJournalService>& PendingInputJournalsDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->PendingInputJournals:PendingInputJournals;}
    std::vector<Speed::Input::V2::FInputSessionRegistry::FJournalService> const& PendingInputJournalsDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->PendingInputJournals:PendingInputJournals;}
    std::map<uint64, std::shared_ptr<Speed::Input::V2::FInputObservationChannel>>& InputObservationsDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->InputObservations:InputObservations;}
    std::map<uint64, std::shared_ptr<Speed::Input::V2::FInputObservationChannel>> const& InputObservationsDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->InputObservations:InputObservations;}
    std::vector<std::pair<uint64, std::shared_ptr<Speed::Input::V2::FInputObservationChannel>>>& PendingInputObservationsDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->PendingInputObservations:PendingInputObservations;}
    std::vector<std::pair<uint64, std::shared_ptr<Speed::Input::V2::FInputObservationChannel>>> const& PendingInputObservationsDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->PendingInputObservations:PendingInputObservations;}
    uint64& InputWorkerGenerationDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->InputWorkerGeneration:InputWorkerGeneration;}
    uint64 const& InputWorkerGenerationDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->InputWorkerGeneration:InputWorkerGeneration;}
    uint64& NextInputCommandIdDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->NextInputCommandId:NextInputCommandId;}
    uint64 const& NextInputCommandIdDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->NextInputCommandId:NextInputCommandId;}
    TAtomic<bool>& bInputSessionAdmissionRequestedDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->bInputSessionAdmissionRequested:bInputSessionAdmissionRequested;}
    TAtomic<bool> const& bInputSessionAdmissionRequestedDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->bInputSessionAdmissionRequested:bInputSessionAdmissionRequested;}
    uint32& RequiredExactRemoteInputSessionsDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->RequiredExactRemoteInputSessions:RequiredExactRemoteInputSessions;}
    uint32 const& RequiredExactRemoteInputSessionsDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->RequiredExactRemoteInputSessions:RequiredExactRemoteInputSessions;}
    Speed::Input::V2::EExactRemoteAdmission& ConfiguredRemoteInputAdmissionDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->ConfiguredRemoteInputAdmission:ConfiguredRemoteInputAdmission;}
    Speed::Input::V2::EExactRemoteAdmission const& ConfiguredRemoteInputAdmissionDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->ConfiguredRemoteInputAdmission:ConfiguredRemoteInputAdmission;}
    bool& bExactRemoteInputWaitingDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->bExactRemoteInputWaiting:bExactRemoteInputWaiting;}
    bool const& bExactRemoteInputWaitingDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->bExactRemoteInputWaiting:bExactRemoteInputWaiting;}
    bool& bInputCommandBoundaryDirtyDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->bInputCommandBoundaryDirty:bInputCommandBoundaryDirty;}
    bool const& bInputCommandBoundaryDirtyDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->bInputCommandBoundaryDirty:bInputCommandBoundaryDirty;}
    TAtomic<bool>& bInputOwnersClosedOnWorkerDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->bInputOwnersClosedOnWorker:bInputOwnersClosedOnWorker;}
    TAtomic<bool> const& bInputOwnersClosedOnWorkerDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->bInputOwnersClosedOnWorker:bInputOwnersClosedOnWorker;}
    uint64& NeutralizedInputRegistryVersionDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->NeutralizedInputRegistryVersion:NeutralizedInputRegistryVersion;}
    uint64 const& NeutralizedInputRegistryVersionDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->NeutralizedInputRegistryVersion:NeutralizedInputRegistryVersion;}
    bool& bInputNeutralizedDuringPauseDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->bInputNeutralizedDuringPause:bInputNeutralizedDuringPause;}
    bool const& bInputNeutralizedDuringPauseDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->bInputNeutralizedDuringPause:bInputNeutralizedDuringPause;}
    TUniquePtr<FSimulationWorker>& SimulationWorkerDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->SimulationWorker:SimulationWorker;}
    TUniquePtr<FSimulationWorker> const& SimulationWorkerDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->SimulationWorker:SimulationWorker;}
    uint32& OwnedBoundarySuspendDepthDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->OwnedBoundarySuspendDepth:OwnedBoundarySuspendDepth;}
    uint32 const& OwnedBoundarySuspendDepthDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->OwnedBoundarySuspendDepth:OwnedBoundarySuspendDepth;}
    FCriticalSection& RollbackRequestMutexDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->RollbackRequestMutex:RollbackRequestMutex;}
    FCriticalSection const& RollbackRequestMutexDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->RollbackRequestMutex:RollbackRequestMutex;}
    TOptional<FPendingRollbackRequest>& PendingRollbackRequestDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->PendingRollbackRequest:PendingRollbackRequest;}
    TOptional<FPendingRollbackRequest> const& PendingRollbackRequestDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->PendingRollbackRequest:PendingRollbackRequest;}
    TAtomic<uint8>& ActiveExecutionModeValueDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->ActiveExecutionModeValue:ActiveExecutionModeValue;}
    TAtomic<uint8> const& ActiveExecutionModeValueDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->ActiveExecutionModeValue:ActiveExecutionModeValue;}
    TAtomic<bool>& bOwnedWorkerTerminalDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->bOwnedWorkerTerminal:bOwnedWorkerTerminal;}
    TAtomic<bool> const& bOwnedWorkerTerminalDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->bOwnedWorkerTerminal:bOwnedWorkerTerminal;}
    TAtomic<bool>& bInputOwnerRetiredDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->bInputOwnerRetired:bInputOwnerRetired;}
    TAtomic<bool> const& bInputOwnerRetiredDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->bInputOwnerRetired:bInputOwnerRetired;}
    TAtomic<bool>& bOwnedSimulationPausedDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->bOwnedSimulationPaused:bOwnedSimulationPaused;}
    TAtomic<bool> const& bOwnedSimulationPausedDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->bOwnedSimulationPaused:bOwnedSimulationPaused;}
    double& GameThreadAccumulatorSecondsDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->GameThreadAccumulatorSeconds:GameThreadAccumulatorSeconds;}
    double const& GameThreadAccumulatorSecondsDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->GameThreadAccumulatorSeconds:GameThreadAccumulatorSeconds;}
    uint8& HostTransitionDelayTicksDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->HostTransitionDelayTicks:HostTransitionDelayTicks;}
    uint8 const& HostTransitionDelayTicksDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->HostTransitionDelayTicks:HostTransitionDelayTicks;}
    bool& bExecutionModeInitializedDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->bExecutionModeInitialized:bExecutionModeInitialized;}
    bool const& bExecutionModeInitializedDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->bExecutionModeInitialized:bExecutionModeInitialized;}
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    uint32& CollisionMatrixTestHzDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->CollisionMatrixTestHz:CollisionMatrixTestHz;}
    uint32 const& CollisionMatrixTestHzDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->CollisionMatrixTestHz:CollisionMatrixTestHz;}
#endif
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    uint64& CollisionMatrixTestFirstFrameDriverStorage() {return UsesOriginalDriverStorage()?OriginalDriverStorage->CollisionMatrixTestFirstFrame:CollisionMatrixTestFirstFrame;}
    uint64 const& CollisionMatrixTestFirstFrameDriverStorage() const {return UsesOriginalDriverStorage()?OriginalDriverStorage->CollisionMatrixTestFirstFrame:CollisionMatrixTestFirstFrame;}
#endif
private:
protected:
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
	// Immutable after BeginPlay, before worker creation. Native qualification remains separate.
	bool bPublishCompletedWorldInput = false;
	TSharedRef<TAtomic<bool>, ESPMode::ThreadSafe> PresentationRetiredAfterJoin = MakeShared<TAtomic<bool>, ESPMode::ThreadSafe>(false);
	bool bPresentationBindingClosed = false;
	TAtomic<bool> bReplayCaptureRequested{false};
	FCriticalSection ReplaySinkMutex;
	TSharedPtr<ISimulationReplaySink, ESPMode::ThreadSafe> ReplaySink;
	void ServiceReplayCapture(const FSimulationSnapshot& Snapshot) noexcept;
	FCriticalSection PresentationProducerMutex;
	TArray<TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe>> PresentationProducers;
	Speed::SimulationBoundary::FPresentationFrameLatch PresentationLatch;


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
	Speed::Concurrency::TAtomicSharedPtr<const Speed::FCompletedSimulationPublication> PublishedCompletedWorldInput{};
	FCriticalSection InputSessionAdmissionMutex;
	std::vector<Speed::Input::V2::FInputSessionRegistry::FJournalService> PendingInputJournals;
	std::map<uint64, std::shared_ptr<Speed::Input::V2::FInputObservationChannel>> InputObservations;
	std::vector<std::pair<uint64, std::shared_ptr<Speed::Input::V2::FInputObservationChannel>>> PendingInputObservations;
	uint64 InputWorkerGeneration = 0;
	uint64 NextInputCommandId = 1;
	TAtomic<bool> bInputSessionAdmissionRequested = false;
    uint32 RequiredExactRemoteInputSessions = 0; // frozen GT configuration before worker creation
    Speed::Input::V2::EExactRemoteAdmission ConfiguredRemoteInputAdmission = Speed::Input::V2::EExactRemoteAdmission::Closed;
    bool bExactRemoteInputWaiting = false; // worker only
    bool bInputCommandBoundaryDirty = false; // protected by InputSessionAdmissionMutex
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
