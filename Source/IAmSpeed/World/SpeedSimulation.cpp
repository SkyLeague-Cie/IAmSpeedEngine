// Fill out your copyright notice in the Description page of Project Settings.


#include "SpeedSimulation.h"
#include "CanonicalFrameContext.h"
#include "CanonicalFrameDriver.h"
#include "IAmSpeed/World/Analytic/StaticWorldQueryAudit.h"
#include "IAmSpeed/Base/SUtils.h"
#include "Net/UnrealNetwork.h"
#include "SpeedWorldSubsystem.h"
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
	EngineFPS = Speed::SimUtils::ComputePhysicsFPS(UPhysicsSettings::Get()->AsyncFixedTimeStepSize);
	RealDeltaTime = 1.0f / static_cast<float>(EngineFPS);
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
	QUICK_SCOPE_CYCLE_COUNTER(STAT_IAmSpeed_AsyncPhysicsTickActor);
	TRACE_CPUPROFILER_EVENT_SCOPE(IAmSpeed_AsyncPhysicsTickActor);
	const double CallbackStartSeconds = FPlatformTime::Seconds();
	const uint64 StepSerialAtEntry = SpeedWorldSubsystem
		? SpeedWorldSubsystem->GetLastStepDiagnostics().Serial : 0;
	ON_SCOPE_EXIT
	{
		if (!SpeedWorldSubsystem ||
			Speed::CanonicalFrameDriver::IsFastSimulationEnabled())
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

	if (Speed::CanonicalFrameDriver::IsEnabled())
	{
		if (!SpeedWorldSubsystem)
		{
			SpeedWorldSubsystem = GetSpeedWorldSubsystem(GetWorld());
		}
		if (!SpeedWorldSubsystem)
		{
			return;
		}
		if (Speed::Analytic::FStaticWorldQueryAudit::IsSurfaceAnalyticBackend() &&
			!SpeedWorldSubsystem->GetStaticCollisionWorld())
		{
			if (!bStaticCollisionReadinessErrorReported)
			{
				bStaticCollisionReadinessErrorReported = true;
				UE_LOG(LogTemp, Error,
					TEXT("[StaticCollisionAuthorityNotReady] Canonical simulation is waiting for a valid analytical static collision world."));
			}
			return;
		}
		bStaticCollisionReadinessErrorReported = false;

		if (!bCanonicalFrameInitialized)
		{
			// Preserve startup/toggle alignment once, then let the integer counter
			// become the sole time authority for all following canonical frames.
			UpdateNumFrame(SimTime);
			CanonicalNumFrame = NumFrame() > 0 ? uint64(NumFrame() - 1u) : 0u;
			bCanonicalFrameInitialized = true;
		}

		if (Speed::CanonicalFrameDriver::IsFastSimulationEnabled())
		{
			RealDeltaTime = 0.0f;
			if (bFastSimulationBudgetExceeded)
			{
				return;
			}

			const ECanonicalRunControlState RunState =
				SpeedWorldSubsystem->GetCanonicalRunControlState();
			if (RunState == ECanonicalRunControlState::WaitingForScenario ||
				RunState == ECanonicalRunControlState::Uncontrolled)
			{
				FastReadyDelayCallbacksObserved = 0;
				return;
			}
			if (RunState == ECanonicalRunControlState::Complete)
			{
				return;
			}
			if (FastReadyDelayCallbacksObserved <
				Speed::CanonicalFrameDriver::GetFastReadyDelayCallbacks())
			{
				++FastReadyDelayCallbacksObserved;
				return;
			}

			if (!FastSimulation(
				Speed::CanonicalFrameDriver::GetFastMaxFramesPerRun()))
			{
				bFastSimulationBudgetExceeded = true;
				UE_LOG(LogTemp, Error,
					TEXT("FastSimulation exhausted its frame budget at canonical frame %llu without a terminal result."),
					CanonicalNumFrame);
			}
		}
		else
		{
			RealTimeSimulation();
		}
		return;
	}

	bCanonicalFrameInitialized = false;
	bFastSimulationBudgetExceeded = false;
	FastReadyDelayCallbacksObserved = 0;
	RealDeltaTime = 1.0f / static_cast<float>(EngineFPS);
	UpdateNumFrame(SimTime);
	Simulate(Dt, SimTime);
}

void ASpeedSimulation::RealTimeSimulation()
{
	RealDeltaTime = 1.0f / static_cast<float>(EngineFPS);
	RunCanonicalFrames(1u);
}

bool ASpeedSimulation::FastSimulation(const uint32 MaxFrameCount)
{
	RealDeltaTime = 0.0f;
	const uint64 StartFrame = CanonicalNumFrame;
	const uint64 StartPerformanceFrameCount = PerformanceFrameCount;
	const uint64 StartWarningFrameCount = PerformanceWarningFrameCount;
	const uint64 StartIterationLimitCount = PerformanceIterationLimitCount;
	const double StartTotalStepMilliseconds = PerformanceTotalStepMilliseconds;
	const double StartSeconds = FPlatformTime::Seconds();
	double RunMaximumStepMilliseconds = 0.0;
	double RunCoreStepMilliseconds = 0.0;
	double RunSweepMilliseconds = 0.0;
	double RunProjectionMilliseconds = 0.0;
	double RunPostMilliseconds = 0.0;
	double RunStaticAuthorityMilliseconds = 0.0;
	double RunMaximumStaticAuthorityMilliseconds = 0.0;
	uint64 RunStaticQueryCount = 0;
	uint64 RunComponentSweepCount = 0;
	uint64 RunIterationCount = 0;
	uint32 RunMaximumStaticQueryCount = 0;
	int32 RunMaximumIterationCount = 0;
	for (uint32 FrameIndex = 0; FrameIndex < MaxFrameCount; ++FrameIndex)
	{
		RunCanonicalFrames(1u);
		const FSpeedStepDiagnostics& Diagnostics =
			SpeedWorldSubsystem->GetLastStepDiagnostics();
		RunMaximumStepMilliseconds = FMath::Max(
			RunMaximumStepMilliseconds,
			LastPhysicalFrameMilliseconds);
		RunCoreStepMilliseconds += Diagnostics.TotalMilliseconds;
		RunSweepMilliseconds += Diagnostics.SweepMilliseconds;
		RunProjectionMilliseconds += Diagnostics.ProjectionMilliseconds;
		RunPostMilliseconds += Diagnostics.PostMilliseconds;
		RunStaticAuthorityMilliseconds += Diagnostics.StaticAuthorityMilliseconds;
		RunMaximumStaticAuthorityMilliseconds = FMath::Max(
			RunMaximumStaticAuthorityMilliseconds,
			Diagnostics.MaximumStaticAuthorityMilliseconds);
		RunStaticQueryCount += Diagnostics.StaticQueryCount;
		RunComponentSweepCount += Diagnostics.ComponentSweepCount;
		RunIterationCount += Diagnostics.IterationCount;
		RunMaximumStaticQueryCount = FMath::Max(
			RunMaximumStaticQueryCount, Diagnostics.StaticQueryCount);
		RunMaximumIterationCount = FMath::Max(
			RunMaximumIterationCount, Diagnostics.IterationCount);
		if (SpeedWorldSubsystem->GetCanonicalRunControlState() ==
			ECanonicalRunControlState::Complete)
		{
			const double WallSeconds = FPlatformTime::Seconds() - StartSeconds;
			const uint64 SimulatedFrames = CanonicalNumFrame - StartFrame;
			const double SimulatedSeconds = static_cast<double>(SimulatedFrames) /
				static_cast<double>(FCanonicalFrameContext::PhysicsFramesPerSecond);
			const uint64 MeasuredFrames =
				PerformanceFrameCount - StartPerformanceFrameCount;
			const uint64 WarningFrames =
				PerformanceWarningFrameCount - StartWarningFrameCount;
			const uint64 IterationLimits =
				PerformanceIterationLimitCount - StartIterationLimitCount;
			const double TotalStepMilliseconds =
				PerformanceTotalStepMilliseconds - StartTotalStepMilliseconds;
			UE_LOG(LogTemp, Display,
				TEXT("[FastSimulationRun] StartFrame=%llu EndFrame=%llu Frames=%llu WallSeconds=%.9f SimulatedSeconds=%.9f RealtimeMultiplier=%.3f"),
				StartFrame,
				CanonicalNumFrame > 0 ? CanonicalNumFrame - 1u : 0u,
				SimulatedFrames,
				WallSeconds,
				SimulatedSeconds,
				WallSeconds > 0.0 ? SimulatedSeconds / WallSeconds : 0.0);
			UE_LOG(LogTemp, Display,
				TEXT("[FastSimulationPerformance] StartFrame=%llu EndFrame=%llu Frames=%llu WarningFrames=%llu IterationLimits=%llu AverageStepMs=%.9f MaximumStepMs=%.9f WarningFrameFraction=%.6f WarningThresholdMs=%.9f CoreAverageMs=%.9f SweepAverageMs=%.9f StaticAuthorityAverageMs=%.9f StaticAuthorityMaximumMs=%.9f ProjectionAverageMs=%.9f PostAverageMs=%.9f StaticQueries=%llu StaticQueriesAverage=%.6f StaticQueriesMaximum=%u ComponentSweepsAverage=%.6f IterationsAverage=%.6f IterationsMaximum=%d"),
				StartFrame,
				CanonicalNumFrame > 0 ? CanonicalNumFrame - 1u : 0u,
				MeasuredFrames, WarningFrames, IterationLimits,
				MeasuredFrames > 0 ? TotalStepMilliseconds /
					static_cast<double>(MeasuredFrames) : 0.0,
				RunMaximumStepMilliseconds,
				CVarSimulationWarningFrameFraction.GetValueOnAnyThread(),
				1000.0 / static_cast<double>(
					FCanonicalFrameContext::PhysicsFramesPerSecond) *
					CVarSimulationWarningFrameFraction.GetValueOnAnyThread(),
				MeasuredFrames > 0 ? RunCoreStepMilliseconds /
					static_cast<double>(MeasuredFrames) : 0.0,
				MeasuredFrames > 0 ? RunSweepMilliseconds /
					static_cast<double>(MeasuredFrames) : 0.0,
				MeasuredFrames > 0 ? RunStaticAuthorityMilliseconds /
					static_cast<double>(MeasuredFrames) : 0.0,
				RunMaximumStaticAuthorityMilliseconds,
				MeasuredFrames > 0 ? RunProjectionMilliseconds /
					static_cast<double>(MeasuredFrames) : 0.0,
				MeasuredFrames > 0 ? RunPostMilliseconds /
					static_cast<double>(MeasuredFrames) : 0.0,
				RunStaticQueryCount,
				MeasuredFrames > 0 ? static_cast<double>(RunStaticQueryCount) /
					static_cast<double>(MeasuredFrames) : 0.0,
				RunMaximumStaticQueryCount,
				MeasuredFrames > 0 ? static_cast<double>(RunComponentSweepCount) /
					static_cast<double>(MeasuredFrames) : 0.0,
				MeasuredFrames > 0 ? static_cast<double>(RunIterationCount) /
					static_cast<double>(MeasuredFrames) : 0.0,
				RunMaximumIterationCount);
			return true;
		}
	}
	return false;
}

void ASpeedSimulation::RunCanonicalFrames(const uint32 FrameCount)
{
	for (uint32 FrameIndex = 0; FrameIndex < FrameCount; ++FrameIndex)
	{
		const double PhysicalFrameStartSeconds = FPlatformTime::Seconds();
		StepCanonicalFrame(FCanonicalFrameContext(CanonicalNumFrame));
		RecordStepPerformance(
			(FPlatformTime::Seconds() - PhysicalFrameStartSeconds) * 1000.0);
		++CanonicalNumFrame;
	}
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

void ASpeedSimulation::StepCanonicalFrame(const FCanonicalFrameContext& Context)
{
	if (!SpeedWorldSubsystem)
	{
		SpeedWorldSubsystem = GetSpeedWorldSubsystem(GetWorld());
		if (!SpeedWorldSubsystem)
		{
			return;
		}
	}

	Speed::Analytic::FStaticWorldQueryAudit::BeginFrame(
		Context.NumFrame, SpeedWorldSubsystem->GetAnalyticWorldData(),
		SpeedWorldSubsystem);
	SpeedWorldSubsystem->PrepareCanonicalFrame(Context);
	SpeedWorldSubsystem->Step(
		Context.PhysicalDeltaTime,
		Context.SimTime,
		static_cast<unsigned int>(Context.NumFrame));
	Speed::Analytic::FStaticWorldQueryAudit::EndFrame();
}
