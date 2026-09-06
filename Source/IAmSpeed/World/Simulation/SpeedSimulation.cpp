// Fill out your copyright notice in the Description page of Project Settings.


#include "SpeedSimulation.h"
#include "CanonicalFrameContext.h"
#include "SimulationActorDiagnostics.h"
#include "CanonicalFrameDriver.h"
#include "IAmSpeed/World/Analytic/StaticWorldQueryAudit.h"
#include "IAmSpeed/Base/SUtils.h"
#include "Net/UnrealNetwork.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "PhysicsEngine/PhysicsSettings.h"
#include "HAL/PlatformTime.h"
#include "HAL/IConsoleManager.h"
#include "Misc/ScopeExit.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

namespace
{
	TAutoConsoleVariable<int32> CVarSimulationPerformanceAudit(
		TEXT("p.IAmSpeed.Simulation.PerformanceAudit"), 0,
		TEXT("Logs the wall-time and solver phase breakdown of every real-time IAmSpeed physical frame."));
	TAutoConsoleVariable<int32> CVarSimulationSlowFrameWarnings(
		TEXT("p.IAmSpeed.Simulation.SlowFrameWarnings"), 1,
		TEXT("Reports real-time IAmSpeed physical frames that exceed the configured wall-time budget."));
	TAutoConsoleVariable<float> CVarSimulationWarningFrameFraction(
		TEXT("p.IAmSpeed.Simulation.WarningFrameFraction"), 0.04f,
		TEXT("Warns when one physical step exceeds this fraction of its simulated delta time."));
}

unsigned int ASpeedSimulation::EngineFPS = 120; // Default value, can be overridden by config or in-game

// Sets default values
ASpeedSimulation::ASpeedSimulation()
{
    bReplicates = true;
    bAlwaysRelevant = true;
    SetNetUpdateFrequency(100.f);
    SetReplicateMovement(false);
	bAsyncPhysicsTickEnabled = true;
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;
	PrimaryActorTick.bTickEvenWhenPaused = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;
	EngineFPS = Speed::SimUtils::ComputePhysicsFPS(UPhysicsSettings::Get()->AsyncFixedTimeStepSize);
	RealDeltaTime = 1.0f / static_cast<float>(EngineFPS);
}

void ASpeedSimulation::BeginPlay()
{
	Super::BeginPlay();
	RefreshExecutionMode();
}

void ASpeedSimulation::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	StopOwnedWorker();
	bAsyncPhysicsTickEnabled = false;
	Super::EndPlay(EndPlayReason);
}

void ASpeedSimulation::Tick(const float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	RefreshExecutionMode();
	if (HostTransitionDelayTicks > 0)
	{
		--HostTransitionDelayTicks;
		return;
	}
	const bool bExecutionPaused = bOwnedSimulationPaused.Load() ||
		Speed::CanonicalFrameDriver::IsOwnedThreadPaused();

	switch (GetActiveExecutionMode())
	{
	case ESimulationExecutionMode::UnrealAsyncCallback:
		return;
	case ESimulationExecutionMode::GameThread:
	{
		if (bExecutionPaused)
		{
			return;
		}
		GameThreadAccumulatorSeconds += FMath::Max(0.0f, DeltaSeconds);
		const double FrameIntervalSeconds =
			static_cast<double>(FCanonicalFrameContext::CanonicalPhysicalDeltaTime);
		while (GameThreadAccumulatorSeconds + UE_DOUBLE_SMALL_NUMBER >= FrameIntervalSeconds)
		{
			const ESimulationWorkerResult Result = DriveSimulation(
				FCanonicalFrameContext::CanonicalPhysicalDeltaTime,
				GetCanonicalPulseSimTime());
			if (Result == ESimulationWorkerResult::Failed ||
				Result == ESimulationWorkerResult::Complete)
			{
				GameThreadAccumulatorSeconds = 0.0;
				return;
			}
			GameThreadAccumulatorSeconds -= FrameIntervalSeconds;
			if (Result == ESimulationWorkerResult::Idle)
			{
				// Readiness delay is wall-clock time, not simulation time. Keep at
				// most one frame of debt and retry on the next game-thread tick.
				GameThreadAccumulatorSeconds = FMath::Min(
					GameThreadAccumulatorSeconds, FrameIntervalSeconds);
				return;
			}
		}
		return;
	}
	case ESimulationExecutionMode::IAmSpeedThread:
		StartOwnedWorkerIfReady();
		if (SimulationWorker)
		{
			if (bExecutionPaused && !SimulationWorker->IsPaused())
			{
				SimulationWorker->Pause();
			}
			else if (!bExecutionPaused && SimulationWorker->IsPaused())
			{
				SimulationWorker->Resume();
			}
		}
		return;
	default:
		return;
	}
}

void ASpeedSimulation::RefreshExecutionMode()
{
	const ESimulationExecutionMode DesiredMode =
		Speed::CanonicalFrameDriver::IsEnabled()
		? Speed::CanonicalFrameDriver::GetExecutionMode()
		: ESimulationExecutionMode::UnrealAsyncCallback;
	if (!bExecutionModeInitialized || DesiredMode != GetActiveExecutionMode())
	{
		TransitionExecutionMode(DesiredMode);
	}
}

void ASpeedSimulation::TransitionExecutionMode(
	const ESimulationExecutionMode NewMode)
{
	StopOwnedWorker();
	bAsyncPhysicsTickEnabled = false;
	ResetCanonicalFrame();
	GameThreadAccumulatorSeconds = 0.0;
	bOwnedWorkerTerminal.Store(false);
	ActiveExecutionModeValue.Store(static_cast<uint8>(NewMode));
	bExecutionModeInitialized = true;
	HostTransitionDelayTicks = NewMode == ESimulationExecutionMode::UnrealAsyncCallback ? 0 : 1;
	if (NewMode == ESimulationExecutionMode::UnrealAsyncCallback)
	{
		bAsyncPhysicsTickEnabled = true;
	}
}

void ASpeedSimulation::StartOwnedWorkerIfReady()
{
	if (SimulationWorker || bOwnedWorkerTerminal.Load() ||
		!EnsureSimulationWorldReady())
	{
		return;
	}
	InitializeCanonicalFrame(0.0f);
	OnOwnedWorkerStarting();
	SimulationWorker = MakeUnique<FSimulationWorker>(
		[this]()
		{
			const ESimulationWorkerResult Result = DriveOwnedWorkerPulse(
				FCanonicalFrameContext::CanonicalPhysicalDeltaTime,
				GetCanonicalPulseSimTime());
			if (Result == ESimulationWorkerResult::Complete ||
				Result == ESimulationWorkerResult::Failed)
			{
				bOwnedWorkerTerminal.Store(true);
			}
			return Result;
		},
		[this](FSimulationWorkerWaitContext& WaitContext)
		{
			WaitBetweenFrames(WaitContext);
		});
	if (!SimulationWorker->Start())
	{
		SimulationWorker.Reset();
		bOwnedWorkerTerminal.Store(true);
		UE_LOG(LogTemp, Error, TEXT("[SimulationWorkerStartFailed]"));
	}
}

void ASpeedSimulation::StopOwnedWorker()
{
	if (SimulationWorker)
	{
		SimulationWorker->StopAndJoin();
		SimulationWorker.Reset();
	}
}

void ASpeedSimulation::RestartControlledRun()
{
	check(IsInGameThread());
	check(IsOwnedSimulationPaused());
	// Join before clearing the terminal latch: the preceding frame can still
	// be finishing its publication/metrics when the GT consumes its result.
	StopOwnedWorker();
	bOwnedWorkerTerminal.Store(false);
	CanonicalReadyDelayPulsesObserved = 0;
	ResumeOwnedSimulation();
}

void ASpeedSimulation::PauseOwnedSimulation()
{
	bOwnedSimulationPaused.Store(true);
	if (SimulationWorker)
	{
		SimulationWorker->Pause();
	}
	OnOwnedSimulationPaused();
}

void ASpeedSimulation::ResumeOwnedSimulation()
{
	bOwnedSimulationPaused.Store(false);
	OnOwnedSimulationResumed();
	if (SimulationWorker &&
		!Speed::CanonicalFrameDriver::IsOwnedThreadPaused())
	{
		SimulationWorker->Resume();
	}
}

ESimulationExecutionMode ASpeedSimulation::GetActiveExecutionMode() const
{
	return static_cast<ESimulationExecutionMode>(ActiveExecutionModeValue.Load());
}

#if !UE_BUILD_SHIPPING
bool ASpeedSimulation::IsOwnedWorkerExecutionMode() const
{
	return GetActiveExecutionMode() == ESimulationExecutionMode::IAmSpeedThread;
}
#endif

float ASpeedSimulation::GetCanonicalPulseSimTime() const
{
	return static_cast<float>(CanonicalNumFrame) *
		FCanonicalFrameContext::CanonicalPhysicalDeltaTime;
}

void ASpeedSimulation::UpdateNumFrame(const float& SimTime)
{
	_NumFrame = Speed::SimUtils::ComputeNumFrameFromSimTime(EngineFPS, SimTime);
}

unsigned int ASpeedSimulation::NumFrame() const
{
    return _NumFrame;
}

USpeedWorldSubsystem* ASpeedSimulation::GetSpeedWorldSubsystem(UWorld* World)
{
    if (!World) return nullptr;
    USpeedWorldSubsystem* SS = World->GetSubsystem<USpeedWorldSubsystem>();
	return SS;
}

void ASpeedSimulation::AsyncPhysicsTickActor(float Dt, float SimTime)
{
	if (GetActiveExecutionMode() != ESimulationExecutionMode::UnrealAsyncCallback)
	{
		return;
	}
	if (bOwnedSimulationPaused.Load() ||
		Speed::CanonicalFrameDriver::IsOwnedThreadPaused())
	{
		return;
	}
	QUICK_SCOPE_CYCLE_COUNTER(STAT_IAmSpeed_AsyncPhysicsTickActor);
	TRACE_CPUPROFILER_EVENT_SCOPE(IAmSpeed_AsyncPhysicsTickActor);
	const double CallbackStartSeconds = FPlatformTime::Seconds();
	const uint64 StepSerialAtEntry = SpeedWorldSubsystem
		? SpeedWorldSubsystem->GetLastStepDiagnostics().Serial : 0;
	ON_SCOPE_EXIT
	{
		if (!SpeedWorldSubsystem || !ShouldMeasureCallback())
		{
			return;
		}

		const FSpeedStepDiagnostics& Diagnostics =
			SpeedWorldSubsystem->GetLastStepDiagnostics();
		if (Diagnostics.Serial == StepSerialAtEntry)
		{
			return;
		}

		const double CallbackMilliseconds =
			(FPlatformTime::Seconds() - CallbackStartSeconds) * 1000.0;
		const double BudgetMilliseconds =
			Diagnostics.PhysicalDeltaTimeMilliseconds * FMath::Max(
				0.0, static_cast<double>(
					CVarSimulationWarningFrameFraction.GetValueOnAnyThread()));
		const bool bSlow = CallbackMilliseconds > BudgetMilliseconds;
		const bool bDetailed =
			CVarSimulationPerformanceAudit.GetValueOnAnyThread() != 0;
		const bool bReportSlow = bSlow &&
			CVarSimulationSlowFrameWarnings.GetValueOnAnyThread() != 0 &&
			(SlowFrameReportCount == 0 ||
				Diagnostics.Frame >= LastSlowFrameReport + 300u);
		if (!bDetailed && !bReportSlow)
		{
			return;
		}

		if (bReportSlow)
		{
			++SlowFrameReportCount;
			LastSlowFrameReport = Diagnostics.Frame;
		}
		const FString Message = FString::Printf(
			TEXT("[SpeedPhysicalFrame] Frame=%u CallbackMs=%.6f BudgetMs=%.6f StepMs=%.6f ResetMs=%.6f SweepMs=%.6f StaticAuthorityMs=%.6f StaticAuthorityMaxMs=%.6f IntegrateMs=%.6f ResolveMs=%.6f ProjectionMs=%.6f PostMs=%.6f Iterations=%d MaxIterations=%d ComponentSweeps=%d StaticQueries=%u LegacySweeps=%u AuthorityAttempts=%u ResolvedEvents=%d IterationLimit=%d Slow=%d"),
			Diagnostics.Frame, CallbackMilliseconds, BudgetMilliseconds,
			Diagnostics.TotalMilliseconds, Diagnostics.ResetMilliseconds,
			Diagnostics.SweepMilliseconds, Diagnostics.StaticAuthorityMilliseconds,
			Diagnostics.MaximumStaticAuthorityMilliseconds,
			Diagnostics.IntegrateMilliseconds,
			Diagnostics.ResolveMilliseconds,
			Diagnostics.ProjectionMilliseconds, Diagnostics.PostMilliseconds,
			Diagnostics.IterationCount, Diagnostics.MaximumIterationCount,
			Diagnostics.ComponentSweepCount, Diagnostics.StaticQueryCount,
			Diagnostics.LegacySweepCount, Diagnostics.AuthorityAttemptCount,
			Diagnostics.ResolvedEventCount,
			Diagnostics.bIterationLimitReached ? 1 : 0, bSlow ? 1 : 0);
		if (bSlow)
		{
			UE_LOG(LogTemp, Warning, TEXT("%s"), *Message);
		}
		else
		{
			UE_LOG(LogTemp, Display, TEXT("%s"), *Message);
		}
	};

	DriveSimulation(Dt, SimTime);
}

bool ASpeedSimulation::EnsureSimulationWorldReady()
{
	if (!SpeedWorldSubsystem)
	{
		SpeedWorldSubsystem = GetSpeedWorldSubsystem(GetWorld());
	}
	if (!SpeedWorldSubsystem) return false;
	if (Speed::Analytic::FStaticWorldQueryAudit::IsSurfaceAnalyticBackend() &&
		!SpeedWorldSubsystem->GetStaticCollisionWorld())
	{
		if (!bStaticCollisionReadinessErrorReported)
		{
			bStaticCollisionReadinessErrorReported = true;
			UE_LOG(LogTemp, Error,
				TEXT("[StaticCollisionAuthorityNotReady] Canonical simulation is waiting for a valid analytical static collision world."));
		}
		return false;
	}
	bStaticCollisionReadinessErrorReported = false;
	return true;
}

ESimulationWorkerResult ASpeedSimulation::CheckCanonicalRunReadiness()
{
	const ECanonicalRunControlState RunState =
		SpeedWorldSubsystem->GetCanonicalRunControlState();
	if (RunState == ECanonicalRunControlState::WaitingForScenario ||
		(RunState == ECanonicalRunControlState::Uncontrolled &&
			RequiresCanonicalRunController()))
	{
		CanonicalReadyDelayPulsesObserved = 0;
		return ESimulationWorkerResult::Idle;
	}
	if (RunState == ECanonicalRunControlState::Complete)
	{
		return ESimulationWorkerResult::Complete;
	}
	if (RunState == ECanonicalRunControlState::Ready &&
		CanonicalReadyDelayPulsesObserved <
			Speed::CanonicalFrameDriver::GetFastReadyDelayCallbacks())
	{
		++CanonicalReadyDelayPulsesObserved;
		return ESimulationWorkerResult::Idle;
	}
	return ESimulationWorkerResult::Advanced;
}

void ASpeedSimulation::InitializeCanonicalFrame(const float SimTime)
{
	if (bCanonicalFrameInitialized) return;
	UpdateNumFrame(SimTime);
	CanonicalNumFrame = NumFrame() > 0 ? uint64(NumFrame() - 1u) : 0u;
	bCanonicalFrameInitialized = true;
}

void ASpeedSimulation::ResetCanonicalFrame()
{
	bCanonicalFrameInitialized = false;
	CanonicalReadyDelayPulsesObserved = 0;
}

bool ASpeedSimulation::RunCanonicalFrames(const uint32 FrameCount)
{
	if (!ProcessPendingRollbackRequest())
	{
		return false;
	}
	for (uint32 FrameIndex = 0; FrameIndex < FrameCount; ++FrameIndex)
	{
		const double PhysicalFrameStartSeconds = FPlatformTime::Seconds();
		if (!StepCanonicalFrame(FCanonicalFrameContext(CanonicalNumFrame)))
		{
			return false;
		}
		RecordStepPerformance(
			(FPlatformTime::Seconds() - PhysicalFrameStartSeconds) * 1000.0);
		++CanonicalNumFrame;
	}
	return true;
}

void ASpeedSimulation::Simulate(const float& DeltaTime, const float& SimTime)
{
    if (!SpeedWorldSubsystem)
    {
		SpeedWorldSubsystem = GetSpeedWorldSubsystem(GetWorld());
		if (!SpeedWorldSubsystem)
		{
			return;
		}
	}

	const double PhysicalFrameStartSeconds = FPlatformTime::Seconds();
	SpeedWorldSubsystem->Step(DeltaTime, SimTime, NumFrame());
	RecordStepPerformance(
		(FPlatformTime::Seconds() - PhysicalFrameStartSeconds) * 1000.0);
}

bool ASpeedSimulation::QueueSimulationInput(const uint64 ActivationFrame,
	const uint64 TargetStableId, const TConstArrayView<uint8> Payload)
{
	return InputJournal.Append(ActivationFrame, TargetStableId, Payload);
}

void ASpeedSimulation::SealSimulationInputs()
{
	InputJournal.Seal();
}

void ASpeedSimulation::RecordStepPerformance(
	const double PhysicalFrameMilliseconds)
{
	if (!SpeedWorldSubsystem)
	{
		return;
	}
	const FSpeedStepDiagnostics& Diagnostics =
		SpeedWorldSubsystem->GetLastStepDiagnostics();
	if (Diagnostics.Serial == 0 || Diagnostics.Serial == LastRecordedStepSerial)
	{
		return;
	}
	LastRecordedStepSerial = Diagnostics.Serial;
	LastPhysicalFrameMilliseconds = PhysicalFrameMilliseconds;
	++PerformanceFrameCount;
	PerformanceTotalStepMilliseconds += PhysicalFrameMilliseconds;
	PerformanceMaximumStepMilliseconds = FMath::Max(
		PerformanceMaximumStepMilliseconds, PhysicalFrameMilliseconds);
	const double WarningThresholdMilliseconds =
		Diagnostics.PhysicalDeltaTimeMilliseconds * FMath::Max(
			0.0, static_cast<double>(
				CVarSimulationWarningFrameFraction.GetValueOnAnyThread()));
	PerformanceWarningFrameCount +=
		PhysicalFrameMilliseconds > WarningThresholdMilliseconds ? 1u : 0u;
	PerformanceIterationLimitCount +=
		Diagnostics.bIterationLimitReached ? 1u : 0u;
}

float ASpeedSimulation::GetWarningFrameFraction() const
{
	return CVarSimulationWarningFrameFraction.GetValueOnAnyThread();
}

bool ASpeedSimulation::StepCanonicalFrame(const FCanonicalFrameContext& Context)
{
	if (!SpeedWorldSubsystem)
	{
		SpeedWorldSubsystem = GetSpeedWorldSubsystem(GetWorld());
		if (!SpeedWorldSubsystem)
		{
			return false;
		}
	}

	IAMSPEED_FRAME_SCOPE(Initialize);
	Speed::Analytic::FStaticWorldQueryAudit::BeginFrame(
		Context.NumFrame, SpeedWorldSubsystem->GetAnalyticWorldData(),
		SpeedWorldSubsystem);
	if (InputJournal.IsSealed() &&
		!SpeedWorldSubsystem->ApplySimulationInputs(Context.NumFrame, InputJournal))
	{
		if (!bInputConsumptionErrorReported)
		{
			bInputConsumptionErrorReported = true;
			UE_LOG(LogTemp, Error,
				TEXT("[SimulationInputRejected] Frame=%llu JournalHash=%016llX"),
				Context.NumFrame, InputJournal.StableHash());
		}
		Speed::Analytic::FStaticWorldQueryAudit::EndFrame();
		return false;
	}
	bInputConsumptionErrorReported = false;
	IAMSPEED_FRAME_PHASE(Prepare);
	SpeedWorldSubsystem->PrepareCanonicalFrame(Context);
	IAMSPEED_FRAME_PHASE(Core);
	SpeedWorldSubsystem->Step(
		Context.PhysicalDeltaTime,
		Context.SimTime,
		static_cast<unsigned int>(Context.NumFrame));
	IAMSPEED_FRAME_PHASE(Snapshot);
	const FSimulationSnapshot Snapshot = SpeedWorldSubsystem->CaptureSimulationSnapshot(
		Context.NumFrame, InputJournal.StableHash());
	IAMSPEED_FRAME_PHASE(Publish);
	SnapshotBuffer.Publish(Snapshot);
	IAMSPEED_FRAME_PHASE(Journal);
	FrameHashes.Append(Context.NumFrame, Snapshot.StateHash);
	IAMSPEED_FRAME_PHASE(Finalize);
	Speed::Analytic::FStaticWorldQueryAudit::EndFrame();
	return true;
}

bool ASpeedSimulation::RequestRollbackAndResimulation(
	const FSimulationSnapshot& Snapshot,
	const uint64 TargetFrameInclusive)
{
	if (!Speed::CanonicalFrameDriver::IsEnabled() || !InputJournal.IsSealed() ||
		Snapshot.InputJournalHash != InputJournal.StableHash() ||
		Snapshot.NumFrame > TargetFrameInclusive ||
		TargetFrameInclusive >= TNumericLimits<unsigned int>::Max())
	{
		return false;
	}
	FScopeLock Lock(&RollbackRequestMutex);
	if (PendingRollbackRequest.IsSet())
	{
		return false;
	}
	PendingRollbackRequest.Emplace(FPendingRollbackRequest{
		Snapshot, TargetFrameInclusive });
	return true;
}

bool ASpeedSimulation::ProcessPendingRollbackRequest()
{
	TOptional<FPendingRollbackRequest> Request;
	{
		FScopeLock Lock(&RollbackRequestMutex);
		if (!PendingRollbackRequest.IsSet())
		{
			return true;
		}
		Request = MoveTemp(PendingRollbackRequest);
		PendingRollbackRequest.Reset();
	}

	if (!SpeedWorldSubsystem ||
		!SpeedWorldSubsystem->RestoreSimulationSnapshot(
			Request->Snapshot, InputJournal.StableHash()))
	{
		UE_LOG(LogTemp, Error,
			TEXT("[SimulationSnapshotRestoreRejected] Frame=%llu TargetFrame=%llu"),
			Request->Snapshot.NumFrame, Request->TargetFrameInclusive);
		return false;
	}

	FrameHashes.RemoveFrom(Request->Snapshot.NumFrame);
	if (!FrameHashes.Append(
			Request->Snapshot.NumFrame, Request->Snapshot.StateHash) ||
		!SnapshotBuffer.Publish(Request->Snapshot))
	{
		UE_LOG(LogTemp, Error,
			TEXT("[SimulationSnapshotRepublishFailed] Frame=%llu"),
			Request->Snapshot.NumFrame);
		return false;
	}

	CanonicalNumFrame = Request->Snapshot.NumFrame + 1u;
	_NumFrame = static_cast<unsigned int>(CanonicalNumFrame);
	const uint64 ReplayFrameCount =
		Request->TargetFrameInclusive - Request->Snapshot.NumFrame;
	for (uint64 ReplayIndex = 0; ReplayIndex < ReplayFrameCount; ++ReplayIndex)
	{
		const double PhysicalFrameStartSeconds = FPlatformTime::Seconds();
		if (!StepCanonicalFrame(FCanonicalFrameContext(CanonicalNumFrame)))
		{
			UE_LOG(LogTemp, Error,
				TEXT("[SimulationResimulationFailed] Frame=%llu TargetFrame=%llu"),
				CanonicalNumFrame, Request->TargetFrameInclusive);
			return false;
		}
		RecordStepPerformance(
			(FPlatformTime::Seconds() - PhysicalFrameStartSeconds) * 1000.0);
		++CanonicalNumFrame;
		_NumFrame = static_cast<unsigned int>(CanonicalNumFrame);
	}
	OnCanonicalTimelineRestored();
	return true;
}
