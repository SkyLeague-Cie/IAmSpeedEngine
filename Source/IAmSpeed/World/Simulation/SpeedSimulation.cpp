// Fill out your copyright notice in the Description page of Project Settings.


#include "SpeedSimulation.h"
#include "CanonicalFrameContext.h"
#include "SimulationActorDiagnostics.h"
#include "CanonicalFrameDriver.h"
#include "IAmSpeed/World/Analytic/StaticWorldQueryAudit.h"
#include "IAmSpeed/Base/SUtils.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "Net/UnrealNetwork.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "PhysicsEngine/PhysicsSettings.h"
#include "HAL/PlatformTime.h"
#include "HAL/IConsoleManager.h"
#include "Misc/ScopeExit.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

namespace
{
	std::atomic<uint64> NextInputWorkerGeneration{1};

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
	InputWorkerGeneration = NextInputWorkerGeneration.fetch_add(1);
	check(InputWorkerGeneration && InputWorkerGeneration != MAX_uint64);
	InputSessionCommands = std::make_shared<Speed::Input::V2::FInputSessionCommands>();
}

void ASpeedSimulation::BeginPlay()
{
	Super::BeginPlay();
	// Fixed before the worker starts; diagnostics never change the canonical payload.
	bPublishPresentation = FParse::Param(FCommandLine::Get(), TEXT("SpeedPublishPresentation"));
	RefreshExecutionMode();
}

bool ASpeedSimulation::ReadPresentationPose(const uint64 StableId, FSimulationPoseConsumption& Out)
{
	check(IsInGameThread());
	if (!bPublishPresentation) { Out = FSimulationPoseConsumption(); return false; }
	return PresentationLatch.ReadBody(GFrameCounter, SnapshotBuffer, StableId, Out);
}

bool ASpeedSimulation::ReadCanonicalCameraSample(const uint64 NumFrame,
	FCameraCanonicalSample& Out) const
{
	check(IsInGameThread());
	return CameraSampleBuffer.ReadFrame(NumFrame, Out);
}

bool ASpeedSimulation::ReadPresentationOutput(const uint64 StableId, const uint32 Channel,
	FSimulationPresentationOutput& Out)
{
	check(IsInGameThread());
	if (!bPublishPresentation) { Out = FSimulationPresentationOutput(); return false; }
	return PresentationLatch.ReadOutput(GFrameCounter, SnapshotBuffer, StableId, Channel, Out);
}

bool ASpeedSimulation::RegisterPresentationProducer(
	TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe> Producer)
{
	check(IsInGameThread());
	FScopeLock Lock(&PresentationProducerMutex);
	if (bPresentationBindingClosed || !Producer->OwnerStableId() || !Producer->Channel() || PresentationProducers.Num() >= 64) return false;
	for (const auto& Existing : PresentationProducers)
		if (Existing->OwnerStableId() == Producer->OwnerStableId() && Existing->Channel() == Producer->Channel()) return false;
	PresentationProducers.Add(Producer);
	return true;
}

void ASpeedSimulation::UnregisterPresentationProducer(
	const TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe>& Producer)
{
	check(IsInGameThread());
	FScopeLock Lock(&PresentationProducerMutex);
	PresentationProducers.Remove(Producer);
}

TSharedPtr<ISimulationPresentationProducer, ESPMode::ThreadSafe> ASpeedSimulation::BindPresentationAtFrameBoundary(
	ISpeedComponent& OwnerComponent, ISpeedComponent& TargetComponent,
	TFunctionRef<TSharedPtr<ISimulationPresentationProducer, ESPMode::ThreadSafe>(
		const FSimulationPresentationBinding&)> Factory,
	TSharedPtr<ISimulationPresentationProducer, ESPMode::ThreadSafe> Previous)
{
	return BindPresentationAtFrameBoundary(OwnerComponent, &TargetComponent, Factory, Previous);
}

TSharedPtr<ISimulationPresentationProducer, ESPMode::ThreadSafe> ASpeedSimulation::BindPresentationAtFrameBoundary(
	ISpeedComponent& OwnerComponent, ISpeedComponent* TargetComponent,
	TFunctionRef<TSharedPtr<ISimulationPresentationProducer, ESPMode::ThreadSafe>(
		const FSimulationPresentationBinding&)> Factory,
	TSharedPtr<ISimulationPresentationProducer, ESPMode::ThreadSafe> Previous)
{
	check(IsInGameThread());
	if (bPresentationBindingClosed || GetActiveExecutionMode() == ESimulationExecutionMode::UnrealAsyncCallback ||
		bOwnedWorkerTerminal.Load() || !EnsureSimulationWorldReady()) return nullptr;
	const bool bWasPaused = IsOwnedSimulationPaused();
	bOwnedSimulationPaused.Store(true);
	if (SimulationWorker && !SimulationWorker->TryPause(1000))
	{
		UE_LOG(LogTemp, Warning, TEXT("[PresentationBindingPauseUnacknowledged] Registration rejected; simulation remains pause-requested."));
		return nullptr;
	}
	if (!bWasPaused) OnOwnedSimulationPaused();
	ON_SCOPE_EXIT { if (!bWasPaused) ResumeOwnedSimulation(); };
	InitializeCanonicalFrame(0);
	FSimulationPresentationBinding Binding;
	Binding.OwnerStableId = SpeedWorldSubsystem->GetSimulationStableId(OwnerComponent);
	Binding.TargetStableId = TargetComponent ? SpeedWorldSubsystem->GetSimulationStableId(*TargetComponent) : 0;
	Binding.FirstFrame = CanonicalNumFrame;
	if (!Binding.OwnerStableId || (TargetComponent && !Binding.TargetStableId) || Binding.OwnerStableId == Binding.TargetStableId)
		return nullptr;
	auto Producer = Factory(Binding);
	if (!Producer || Producer->OwnerStableId() != Binding.OwnerStableId || !Producer->Channel())
		return nullptr;
	if (Previous)
	{
		FScopeLock Lock(&PresentationProducerMutex);
		const int32 Index = PresentationProducers.IndexOfByKey(Previous.ToSharedRef());
		if (Index == INDEX_NONE || Previous->OwnerStableId() != Producer->OwnerStableId() ||
			Previous->Channel() != Producer->Channel()) return nullptr;
		// The acknowledged boundary makes replacement atomic with respect to Produce.
		PresentationProducers[Index] = Producer.ToSharedRef();
	}
	else if (!RegisterPresentationProducer(Producer.ToSharedRef())) return nullptr;
	// Successful binding opts into its body/output sidecars. The owned worker
	// is pause-acknowledged above and resumes only after this boundary returns.
	// Failed factories/registration must leave publication policy unchanged.
	bPublishPresentation = true;
	return Producer;
}

bool ASpeedSimulation::ReadCanonicalCameraSamples(const uint64 FirstFrame,
	const uint64 LastFrame, TArray<FCameraCanonicalSample>& Out) const
{
	check(IsInGameThread());
	return CameraSampleBuffer.ReadRange(FirstFrame, LastFrame, Out);
}

void ASpeedSimulation::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	bPresentationBindingClosed = true;
	StopOwnedWorker();
	{
		FScopeLock Lock(&PresentationProducerMutex);
		PresentationProducers.Reset();
	}
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
		if (bExecutionPaused || bOwnedWorkerTerminal.Load())
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
				if (Result == ESimulationWorkerResult::Failed)
				{
					bOwnedWorkerTerminal.Store(true);
					bOwnedSimulationPaused.Store(true);
				}
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
	if (bInputOwnerRetired.Load()) return;
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
		},
		[this](bool bPauseRequested) { return ServiceInputSessionBoundary(bPauseRequested); },
		[this]() { CloseInputSessionsOnWorker(); });
	if (!SimulationWorker->Start(bOwnedSimulationPaused.Load() || Speed::CanonicalFrameDriver::IsOwnedThreadPaused()))
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

bool ASpeedSimulation::PrepareControlledInputRun()
{
	check(IsInGameThread());
	if (GetActiveExecutionMode() != ESimulationExecutionMode::IAmSpeedThread
		|| bCanonicalPublicationTerminal.Load()) return false;
	const auto Boundary = TryPauseOwnedSimulation();
	if (Boundary != ESimulationQuiescence::BoundaryAcknowledged
		&& Boundary != ESimulationQuiescence::AlreadyStopped) return false;
	StopOwnedWorker(); // CloseInputSessionsOnWorker retires producers on their lane.
	FScopeLock Lock(&InputSessionAdmissionMutex);
	if (InputSessionRegistry
		|| (bInputSessionAdmissionRequested.Load() && !bInputOwnersClosedOnWorker.Load())
		|| !PendingInputJournals.empty() || !PendingInputObservations.empty()
		|| !InputObservations.empty() || bCanonicalPublicationTerminal.Load()) return false;
	try
	{
		auto NextCommands = std::make_shared<Speed::Input::V2::FInputSessionCommands>();
		const auto Generation = NextInputWorkerGeneration.fetch_add(1);
		if (!Generation || Generation == MAX_uint64) return false;
		InputSessionCommands = std::move(NextCommands);
		InputWorkerGeneration = Generation; NextInputCommandId = 1;
	}
	catch (...) { return false; }
	std::atomic_store(&PublishedInputRegistry, std::shared_ptr<const Speed::Input::V2::FInputRegistryView>{});
	std::atomic_store(&PublishedInputFrame, std::shared_ptr<const Speed::Input::V2::FRegistryFrame>{});
	bInputSessionAdmissionRequested.Store(false); bInputOwnersClosedOnWorker.Store(false);
	bInputNeutralizedDuringPause = false; NeutralizedInputRegistryVersion = MAX_uint64;
	bInputOwnerRetired.Store(false); bOwnedWorkerTerminal.Store(false);
	CanonicalReadyDelayPulsesObserved = 0;
	return true; // Remains paused until the new scenario is sealed.
}

void ASpeedSimulation::RestartControlledRun()
{
	check(IsInGameThread());
	if (bInputOwnerRetired.Load() || bOwnedWorkerTerminal.Load()
		|| bCanonicalPublicationTerminal.Load() || !IsOwnedSimulationPaused())
	{
		UE_LOG(LogTemp, Error, TEXT("[ControlledRunNotPrepared] Prepare the new generation before admitting scenario inputs."));
		return;
	}
	// Never join here: replacement descriptors already belong to this generation.
	CanonicalReadyDelayPulsesObserved = 0;
	ResumeOwnedSimulation();
}

ESimulationQuiescence ASpeedSimulation::TryPauseOwnedSimulation(const uint32 TimeoutMilliseconds)
{
	check(IsInGameThread());
	bOwnedSimulationPaused.Store(true);
	if (!SimulationWorker && HasActorBegunPlay()
		&& GetActiveExecutionMode() == ESimulationExecutionMode::UnrealAsyncCallback)
		return ESimulationQuiescence::Failed; // No owned boundary for legacy async callbacks.
	if (!SimulationWorker || !SimulationWorker->IsRunning())
	{
		OnOwnedSimulationPaused();
		return ESimulationQuiescence::AlreadyStopped;
	}
	if (!SimulationWorker->TryPause(TimeoutMilliseconds)) return ESimulationQuiescence::TimedOut;
	OnOwnedSimulationPaused();
	return ESimulationQuiescence::BoundaryAcknowledged;
}

ESimulationQuiescence ASpeedSimulation::TrySuspendOwnedBoundaryService(const uint32 TimeoutMilliseconds)
{
	check(IsInGameThread());
	if (GetActiveExecutionMode() != ESimulationExecutionMode::IAmSpeedThread)
		return ESimulationQuiescence::Failed;
	if (OwnedBoundarySuspendDepth)
	{
		// Joining while a recreation lease is held supersedes the parked
		// boundary. The GT may finish teardown after the worker is gone; the
		// outstanding lease still has to be released exactly once.
		if (!SimulationWorker || !SimulationWorker->IsRunning())
			return ESimulationQuiescence::AlreadyStopped;
		if (!SimulationWorker->IsBoundaryServiceSuspended())
			return ESimulationQuiescence::Failed;
		++OwnedBoundarySuspendDepth;
		return ESimulationQuiescence::BoundaryAcknowledged;
	}
	if (!SimulationWorker || !SimulationWorker->IsRunning())
		return ESimulationQuiescence::AlreadyStopped;
	bOwnedSimulationPaused.Store(true);
	if (!SimulationWorker->TrySuspendBoundaryService(TimeoutMilliseconds))
		return ESimulationQuiescence::TimedOut;
	OwnedBoundarySuspendDepth = 1;
	return ESimulationQuiescence::BoundaryAcknowledged;
}

void ASpeedSimulation::ResumeOwnedBoundaryService()
{
	check(IsInGameThread());
	if (!OwnedBoundarySuspendDepth) return;
	if (--OwnedBoundarySuspendDepth == 0 && SimulationWorker)
		SimulationWorker->ResumeBoundaryService();
}

bool ASpeedSimulation::IsOwnedBoundaryServiceSuspended() const
{
	return OwnedBoundarySuspendDepth && SimulationWorker && SimulationWorker->IsBoundaryServiceSuspended();
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

bool ASpeedSimulation::ReadInputFirstFrameAtPausedBoundary(uint64& OutFrame)
{
	check(IsInGameThread());
	if (GetActiveExecutionMode() != ESimulationExecutionMode::IAmSpeedThread
		|| bInputOwnerRetired.Load() || !bOwnedSimulationPaused.Load() || bCanonicalPublicationTerminal.Load()
		|| (SimulationWorker && SimulationWorker->IsRunning() && !SimulationWorker->IsPaused())) return false;
	InitializeCanonicalFrame(0.0f); OutFrame = CanonicalNumFrame; return true;
}

bool ASpeedSimulation::JoinOwnedSimulationForInputTeardown()
{
	check(IsInGameThread());
	// An async callback (or absent driver) cannot grant an owned-thread fence.
	if (GetActiveExecutionMode() != ESimulationExecutionMode::IAmSpeedThread) return false;
	bInputOwnerRetired.Store(true);
	bOwnedSimulationPaused.Store(true); bOwnedWorkerTerminal.Store(true);
	StopOwnedWorker();
	if (SimulationWorker) return false;
	if (!bInputOwnersClosedOnWorker.Load())
	{
		FScopeLock Lock(&InputSessionAdmissionMutex);
		// No published registry plus no surviving worker-owned registry means
		// only inert, unserviced descriptors may remain. Do not manufacture a
		// worker-owner retirement receipt for a registry that actually ran.
		if (InputSessionRegistry || ReadInputRegistryView() || !InputObservations.empty()
			|| !InputSessionCommands->CancelUnprocessedAfterJoin()) return false;
		for (const auto& Observation : PendingInputObservations) Observation.second->Deactivate();
		PendingInputObservations.clear(); PendingInputJournals.clear();
		bInputOwnersClosedOnWorker.Store(true); // Joined proof: zero owners were constructed.
	}
	return true;
}

void ASpeedSimulation::ResumeOwnedSimulation()
{
	if (bInputOwnerRetired.Load()) return;
	// A timed-out structural request remains fail-closed until a later caller
	// claims its ACK or joins the worker. Do not publish a false resumed state.
	if (SimulationWorker && SimulationWorker->IsBoundaryServiceSuspendRequested()) return;
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

bool ASpeedSimulation::IsOwnedWorkerExecutionMode() const
{
	return GetActiveExecutionMode() == ESimulationExecutionMode::IAmSpeedThread;
}

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
	if (bCanonicalPublicationTerminal.Load()) return false;
	if (!SpeedWorldSubsystem)
	{
		SpeedWorldSubsystem = GetSpeedWorldSubsystem(GetWorld());
		if (!SpeedWorldSubsystem)
		{
			return false;
		}
	}

	FString BindingFailure;
	bool bAdmitted = false;
	try { bAdmitted = SpeedWorldSubsystem->BeginCanonicalFrame(BindingFailure); }
	catch (...)
	{
		bCanonicalPublicationTerminal.Store(true);
		bOwnedWorkerTerminal.Store(true);
		UE_LOG(LogTemp, Error, TEXT("[CanonicalAdmissionExceptionTerminal] Frame=%llu"), Context.NumFrame);
		return false; // BeginCanonicalFrame sets its active flag only after validation returns.
	}
	if (!bAdmitted)
	{
		UE_LOG(LogTemp, Error, TEXT("[SimulationBindingRejected] frame=%llu %s"),
			Context.NumFrame, *BindingFailure);
		// Both drivers convert a rejected frame to Failed; the owned worker
		// exits at this boundary without advancing time or publishing a pose.
		return false;
	}
	bool bAuditStarted = false;
	bool bGlobalPublished = false;
	bool bOutcomeHandled = false;
	ON_SCOPE_EXIT
	{
		if (bAuditStarted) Speed::Analytic::FStaticWorldQueryAudit::EndFrame();
		SpeedWorldSubsystem->EndCanonicalFrame();
	};
	const auto MarkTerminal = [this]()
	{
		if (InputSessionRegistry) InputSessionRegistry->AbortFrame();
		std::atomic_store(&PublishedInputFrame, std::shared_ptr<const Speed::Input::V2::FRegistryFrame>{});
		bCanonicalPublicationTerminal.Store(true);
		bOwnedWorkerTerminal.Store(true);
	};
	try
	{
		IAMSPEED_FRAME_SCOPE(Initialize);
		const auto InputView = InputSessionRegistry ? InputSessionRegistry->ReadRegistry() : nullptr;
		const bool bHasInputSessions = InputView && !InputView->Bindings.empty();
		if (bHasInputSessions)
		{
			if (!SpeedWorldSubsystem->StageCanonicalScenarioInputs(Context, *InputSessionRegistry))
				throw std::runtime_error("canonical scenario authoring rejected");
			if (!InputSessionRegistry->PrepareFrame(Context.NumFrame) || !InputSessionRegistry->InstallAll())
				throw std::runtime_error("canonical input polling rejected");
			const auto Installed = InputSessionRegistry->ReadInstalled();
			if (!Installed || !SpeedWorldSubsystem->InstallCanonicalInputs(*Installed) || !InputSessionRegistry->BeginAll())
				throw std::runtime_error("canonical input installation rejected");
		}
		Speed::Analytic::FStaticWorldQueryAudit::BeginFrame(
			Context.NumFrame, SpeedWorldSubsystem->GetAnalyticWorldData(),
			SpeedWorldSubsystem);
		bAuditStarted = true;
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
			SpeedWorldSubsystem->AbortCanonicalFrame(Context.NumFrame, ECanonicalFrameAbortReason::PreparationFailed);
			bOutcomeHandled = true;
			MarkTerminal();
			return false;
		}
		if (!SpeedWorldSubsystem->PrepareCanonicalInputs(Context))
		{
			SpeedWorldSubsystem->AbortCanonicalFrame(Context.NumFrame, ECanonicalFrameAbortReason::PreparationFailed);
			bOutcomeHandled = true;
			MarkTerminal();
			UE_LOG(LogTemp, Error, TEXT("[CanonicalInputPreparationRejected] Frame=%llu"), Context.NumFrame);
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
		FSimulationSnapshot Snapshot = SpeedWorldSubsystem->CaptureSimulationSnapshot(
			Context.NumFrame, InputJournal.StableHash(), bPublishPresentation);
		TArray<TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe>> Producers;
		{
			FScopeLock Lock(&PresentationProducerMutex);
			Producers = PresentationProducers;
		}
		for (const auto& Producer : Producers)
		{
			FSimulationPresentationOutput Output;
			Producer->Produce(Snapshot, Output);
			// Producer cannot forge the envelope address or publication serial.
			Output.OwnerStableId = Producer->OwnerStableId();
			Output.Channel = Producer->Channel();
			Output.NumFrame = Snapshot.NumFrame;
			Output.PublicationSerial = 0;
			Snapshot.PresentationOutputs.Add(MoveTemp(Output));
		}
		if (bHasInputSessions && !InputSessionRegistry->ValidateComplete())
			throw std::runtime_error("canonical input completion rejected");
		IAMSPEED_FRAME_PHASE(Publish);
		const ECanonicalPublicationResult Publication = SpeedWorldSubsystem->PublishCanonicalFrame(Context.NumFrame, [&]()
		{
			// One simulation owner. Avoid serial wrap before touching the inactive slot.
			return SnapshotBuffer.PublishedSerial() != MAX_uint64 && SnapshotBuffer.Publish(Snapshot);
		});
		bOutcomeHandled = true;
		bGlobalPublished = Publication == ECanonicalPublicationResult::Completed || Publication == ECanonicalPublicationResult::CommitInvariantFailed;
		if (Publication != ECanonicalPublicationResult::Completed)
		{
			MarkTerminal();
			UE_LOG(LogTemp, Error, TEXT("[CanonicalPublicationTerminal] Frame=%llu Result=%u GlobalPublished=%d"),
				Context.NumFrame, static_cast<uint32>(Publication), bGlobalPublished ? 1 : 0);
			return false;
		}
		if (bHasInputSessions)
		{
			if (!InputSessionRegistry->CompleteAll())
			{
				MarkTerminal();
				return false;
			}
			const auto Completed = InputSessionRegistry->ReadLatest();
			std::atomic_store(&PublishedInputFrame, Completed);
			for (const auto& Input : Completed->Inputs)
			{
				const auto Observation = InputObservations.find(Input.Session);
				if (Observation != InputObservations.end())
				{
					try { if (!Observation->second->Publish(Input.Snapshot)) Observation->second->Deactivate(); }
					catch (...) { UE_LOG(LogTemp, Error, TEXT("[InputObservationPublicationFailed] Frame=%llu"), Context.NumFrame); }
				}
			}
		}
		// V2 authoritative commit is finished before any optional presentation work.
		SpeedWorldSubsystem->NotifyCanonicalFramePublished(Context.NumFrame);
		FCameraCanonicalSample CameraSample;
		if (BuildCanonicalCameraSample(Snapshot, CameraSample))
		{
			CameraSample.NumFrame = Snapshot.NumFrame;
			CameraSample.PublicationSerial = SnapshotBuffer.PublishedSerial();
			CameraSample.StateHash = Snapshot.StateHash;
			CameraSample.InputJournalHash = Snapshot.InputJournalHash;
			CameraSampleBuffer.Publish(CameraSample);
		}
		IAMSPEED_FRAME_PHASE(Journal);
		FrameHashes.Append(Context.NumFrame, Snapshot.StateHash);
		IAMSPEED_FRAME_PHASE(Finalize);
		return true;
	}
	catch (...)
	{
		if (!bGlobalPublished && !bOutcomeHandled)
			SpeedWorldSubsystem->AbortCanonicalFrame(Context.NumFrame, ECanonicalFrameAbortReason::PreparationFailed);
		MarkTerminal();
		UE_LOG(LogTemp, Error, TEXT("[CanonicalFrameExceptionTerminal] Frame=%llu GlobalPublished=%d"),
			Context.NumFrame, bGlobalPublished ? 1 : 0);
		return false; // No exception escapes into the UE worker/callback boundary.
	}
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
	// Registry replay is not a world/actor restore transaction yet. Reject
	// before queuing instead of rewinding physics underneath live owners.
	const auto InputView = ReadInputRegistryView();
	if (InputView && !InputView->Bindings.empty()) return false;
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

	// Recheck on the worker: an input binding may have arrived since admission.
	const auto InputView = ReadInputRegistryView();
	if (InputView && !InputView->Bindings.empty()) return false;
	TArray<TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe>> RestoreProducers;
	{
		FScopeLock Lock(&PresentationProducerMutex);
		RestoreProducers = PresentationProducers;
	}
	TArray<FSimulationPresentationOutput> RestoreOutputs;
	if (Request->Snapshot.PresentationOutputs.Num() != RestoreProducers.Num()) return false;
	for (const auto& Producer : RestoreProducers)
	{
		const FSimulationPresentationOutput* Output = Request->Snapshot.PresentationOutputs.FindByPredicate(
			[&](const FSimulationPresentationOutput& Candidate)
			{
				return Candidate.OwnerStableId == Producer->OwnerStableId() && Candidate.Channel == Producer->Channel();
			});
		if (!Output || Output->NumFrame != Request->Snapshot.NumFrame ||
			!Producer->CanRestore(*Output, Request->TargetFrameInclusive)) return false;
		RestoreOutputs.Add(*Output);
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

	Request->Snapshot.PresentationOutputs.Reset();
	for (int32 Index = 0; Index < RestoreProducers.Num(); ++Index)
	{
		FSimulationPresentationOutput Restored;
		RestoreProducers[Index]->RestoreValidated(RestoreOutputs[Index], Request->TargetFrameInclusive, Restored);
		Request->Snapshot.PresentationOutputs.Add(MoveTemp(Restored));
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
