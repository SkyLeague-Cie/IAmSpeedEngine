// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "CanonicalFrameDriver.h"
#include "SimulationFrameJournal.h"
#include "SimulationWorker.h"
#include "SpeedSimulation.generated.h"

class USpeedWorldSubsystem;
struct FCanonicalFrameContext;

/*
* ASpeedSimulation : Actor responsible for ticking the IAmSpeed Engine simulation.
* It is the one that calls Step() on the SpeedWorldSubsystem every physics frame.
*/
UCLASS(Abstract)
class IAMSPEED_API ASpeedSimulation : public AActor
{
	GENERATED_BODY()

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
	/** Complete per-frame hashes for comparing two drivers after a run. */
	const Speed::SimulationBoundary::FFrameHashJournal& GetFrameHashes() const { return FrameHashes; }
	/**
	 * Queues an authoritative restore followed by deterministic replay through
	 * TargetFrameInclusive. The simulation owner consumes it at a frame boundary.
	 */
	bool RequestRollbackAndResimulation(
		const FSimulationSnapshot& Snapshot,
		uint64 TargetFrameInclusive);
	/** Pauses the owned execution lane without changing the canonical frame. */
	void PauseOwnedSimulation();
	/** Resumes the owned lane from its current canonical frame. */
	void ResumeOwnedSimulation();
	/** GT-only: re-arms a paused controlled run after its actors/inputs were replaced. Keeps the world's canonical frame continuous. */
	void RestartControlledRun();
	/** Thread-safe pause witness used by gameplay integration tests and diagnostics. */
	bool IsOwnedSimulationPaused() const { return bOwnedSimulationPaused.Load(); }
#if !UE_BUILD_SHIPPING
	/** True when canonical frames are currently hosted by IAmSpeed's worker. */
	bool IsOwnedWorkerExecutionMode() const;
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
	Speed::SimulationBoundary::FFrameHashJournal FrameHashes;

private:
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

	TUniquePtr<FSimulationWorker> SimulationWorker;
	FCriticalSection RollbackRequestMutex;
	TOptional<FPendingRollbackRequest> PendingRollbackRequest;
	TAtomic<uint8> ActiveExecutionModeValue =
		static_cast<uint8>(ESimulationExecutionMode::UnrealAsyncCallback);
	TAtomic<bool> bOwnedWorkerTerminal = false;
	/** Gameplay-owned pause, independent from the global automation pause. */
	TAtomic<bool> bOwnedSimulationPaused = false;
	double GameThreadAccumulatorSeconds = 0.0;
	uint8 HostTransitionDelayTicks = 0;
	bool bExecutionModeInitialized = false;
};
