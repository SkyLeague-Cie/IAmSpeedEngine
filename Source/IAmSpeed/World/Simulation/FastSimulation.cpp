#include "FastSimulation.h"
#include "SimulationSleepState.h"
#include "SimulationActorDiagnostics.h"

#include "CanonicalFrameContext.h"
#include "CanonicalFrameDriver.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "HAL/PlatformTime.h"
#include "HAL/IConsoleManager.h"

#if !UE_BUILD_SHIPPING
static TAutoConsoleVariable<int32> CVarFastWindowTiming(
	TEXT("p.IAmSpeed.FastSimulation.WindowTiming"), 0,
	TEXT("Opt-in enclosing 30 committed-frame windows, including owned pulse overhead. Forward-run diagnostic, not rollback equivalence proof."));
#endif

ESimulationWorkerResult AFastSimulation::DriveSimulation(
	const float DeltaTime, const float SimTime)
{
	return DriveFastSimulation(SimTime, false);
}

ESimulationWorkerResult AFastSimulation::DriveOwnedWorkerPulse(
	const float DeltaTime, const float SimTime)
{
	return DriveFastSimulation(SimTime, true);
}

ESimulationWorkerResult AFastSimulation::DriveFastSimulation(
	const float SimTime, const bool bSingleFramePulse)
{
	RealDeltaTime = 0.0f;
	if (!Speed::CanonicalFrameDriver::IsEnabled() || bFrameBudgetExceeded)
	{
		return ESimulationWorkerResult::Failed;
	}
	if (!EnsureSimulationWorldReady()) return ESimulationWorkerResult::Idle;
	InitializeCanonicalFrame(SimTime);
	const ESimulationWorkerResult Readiness = CheckCanonicalRunReadiness();
	if (Readiness == ESimulationWorkerResult::Idle)
	{
		bRunMetricsActive = false;
		return Readiness;
	}
	if (Readiness == ESimulationWorkerResult::Complete ||
		Readiness == ESimulationWorkerResult::Failed)
	{
		return Readiness;
	}
	const uint32 MaxFrameCount = Speed::CanonicalFrameDriver::GetFastMaxFramesPerRun();
	if (bSingleFramePulse)
	{
		return AdvanceSealedSimulation(MaxFrameCount);
	}

	for (;;)
	{
		const ESimulationWorkerResult Result = AdvanceSealedSimulation(MaxFrameCount);
		if (Result != ESimulationWorkerResult::Advanced)
		{
			return Result;
		}
	}
}

void AFastSimulation::BeginRunMetrics()
{
#if !UE_BUILD_SHIPPING
    BeginIndependentFastCoverage276();
#endif
#if !UE_BUILD_SHIPPING
	Speed::FSimulationSleepState::ThreadIntegrationSkips() = 0;
	Speed::ActorDiagnostics::BeginRun();
#endif
	bRunMetricsActive = true;
	RunStartFrame = CanonicalNumFrame;
	RunStartPerformanceFrameCount = PerformanceFrameCount;
	RunStartWarningFrameCount = PerformanceWarningFrameCount;
	RunStartIterationLimitCount = PerformanceIterationLimitCount;
	RunStartTotalStepMilliseconds = PerformanceTotalStepMilliseconds;
#if !UE_BUILD_SHIPPING
	bMeasureWindows = CVarFastWindowTiming.GetValueOnAnyThread() != 0;
	bWindowTimingInvalid = false;
	WindowMilliseconds.Reset();
	if (bMeasureWindows) WindowMilliseconds.Reserve(65536);
#endif
	RunStartSeconds = FPlatformTime::Seconds();
#if !UE_BUILD_SHIPPING
	WindowStartFrame = RunStartFrame;
	WindowStartSeconds = RunStartSeconds;
#endif
	RunMaximumStepMilliseconds = 0.0;
	RunCoreStepMilliseconds = 0.0;
	RunSweepMilliseconds = 0.0;
	RunProjectionMilliseconds = 0.0;
	RunPostMilliseconds = 0.0;
	RunStaticAuthorityMilliseconds = 0.0;
	RunMaximumStaticAuthorityMilliseconds = 0.0;
	RunStaticQueryCount = 0;
	RunComponentSweepCount = 0;
	RunIterationCount = 0;
	RunMaximumStaticQueryCount = 0;
	RunMaximumIterationCount = 0;
}

void AFastSimulation::AccumulateRunFrameMetrics()
{
	const FSpeedStepDiagnostics& Diagnostics = SpeedWorldSubsystem->GetLastStepDiagnostics();
	RunMaximumStepMilliseconds = FMath::Max(RunMaximumStepMilliseconds, LastPhysicalFrameMilliseconds);
	RunCoreStepMilliseconds += Diagnostics.TotalMilliseconds;
	RunSweepMilliseconds += Diagnostics.SweepMilliseconds;
	RunProjectionMilliseconds += Diagnostics.ProjectionMilliseconds;
	RunPostMilliseconds += Diagnostics.PostMilliseconds;
	RunStaticAuthorityMilliseconds += Diagnostics.StaticAuthorityMilliseconds;
	RunMaximumStaticAuthorityMilliseconds = FMath::Max(
		RunMaximumStaticAuthorityMilliseconds, Diagnostics.MaximumStaticAuthorityMilliseconds);
	RunStaticQueryCount += Diagnostics.StaticQueryCount;
	RunComponentSweepCount += Diagnostics.ComponentSweepCount;
	RunIterationCount += Diagnostics.IterationCount;
	RunMaximumStaticQueryCount = FMath::Max(RunMaximumStaticQueryCount, Diagnostics.StaticQueryCount);
	RunMaximumIterationCount = FMath::Max(RunMaximumIterationCount, Diagnostics.IterationCount);
}

ESimulationWorkerResult AFastSimulation::AdvanceSealedSimulation(
	const uint32 MaxFrameCount)
{
	if (!bRunMetricsActive)
	{
		BeginRunMetrics();
	}
	if (CanonicalNumFrame - RunStartFrame >= MaxFrameCount)
	{
		bFrameBudgetExceeded = true;
		UE_LOG(LogTemp, Error,
			TEXT("FastSimulation exhausted its frame budget at canonical frame %llu without a terminal result."),
			CanonicalNumFrame);
		return ESimulationWorkerResult::Failed;
	}
	if (!RunCanonicalFrames(1u))
	{
		return ESimulationWorkerResult::Failed;
	}
	AccumulateRunFrameMetrics();
#if !UE_BUILD_SHIPPING
	if (bMeasureWindows) RecordWindowTiming();
#endif
	if (SpeedWorldSubsystem->GetCanonicalRunControlState() != ECanonicalRunControlState::Complete)
	{
		return ESimulationWorkerResult::Advanced;
	}
	ReportCompletedRun();
	return ESimulationWorkerResult::Complete;
}

void AFastSimulation::ReportCompletedRun()
{
#if !UE_BUILD_SHIPPING
    EndIndependentFastCoverage276(true,TEXT("Completed"));
#endif
#if !UE_BUILD_SHIPPING
	Speed::ActorDiagnostics::EndRun();
	UE_LOG(LogTemp, Display, TEXT("[SimulationSleep] IntegrationSkips=%llu"),
		Speed::FSimulationSleepState::ThreadIntegrationSkips());
#endif
	const double WallSeconds = FPlatformTime::Seconds() - RunStartSeconds;
#if !UE_BUILD_SHIPPING
	if (bMeasureWindows) ReportWindowTimings();
#endif
	const uint64 SimulatedFrames = CanonicalNumFrame - RunStartFrame;
	const double FrameDuration = MakeCanonicalFrameContext(RunStartFrame).PhysicalDeltaTime;
	const double SimulatedSeconds = static_cast<double>(SimulatedFrames) * FrameDuration;
	const uint64 MeasuredFrames = PerformanceFrameCount - RunStartPerformanceFrameCount;
	const uint64 WarningFrames = PerformanceWarningFrameCount - RunStartWarningFrameCount;
	const uint64 IterationLimits = PerformanceIterationLimitCount - RunStartIterationLimitCount;
	const double TotalStepMilliseconds = PerformanceTotalStepMilliseconds - RunStartTotalStepMilliseconds;
	const float WarningFrameFraction = GetWarningFrameFraction();
	UE_LOG(LogTemp, Display,
		TEXT("[FastSimulationRun] StartFrame=%llu EndFrame=%llu Frames=%llu WallSeconds=%.9f SimulatedSeconds=%.9f RealtimeMultiplier=%.3f"),
		RunStartFrame, CanonicalNumFrame > 0 ? CanonicalNumFrame - 1u : 0u, SimulatedFrames,
		WallSeconds, SimulatedSeconds, WallSeconds > 0.0 ? SimulatedSeconds / WallSeconds : 0.0);
	UE_LOG(LogTemp, Display,
		TEXT("[FastSimulationPerformance] StartFrame=%llu EndFrame=%llu Frames=%llu WarningFrames=%llu IterationLimits=%llu AverageStepMs=%.9f MaximumStepMs=%.9f WarningFrameFraction=%.6f WarningThresholdMs=%.9f CoreAverageMs=%.9f SweepAverageMs=%.9f StaticAuthorityAverageMs=%.9f StaticAuthorityMaximumMs=%.9f ProjectionAverageMs=%.9f PostAverageMs=%.9f StaticQueries=%llu StaticQueriesAverage=%.6f StaticQueriesMaximum=%u ComponentSweepsAverage=%.6f IterationsAverage=%.6f IterationsMaximum=%d"),
		RunStartFrame, CanonicalNumFrame > 0 ? CanonicalNumFrame - 1u : 0u,
		MeasuredFrames, WarningFrames, IterationLimits,
		MeasuredFrames > 0 ? TotalStepMilliseconds / static_cast<double>(MeasuredFrames) : 0.0,
		RunMaximumStepMilliseconds, WarningFrameFraction,
		1000.0 * FrameDuration * WarningFrameFraction,
		MeasuredFrames > 0 ? RunCoreStepMilliseconds / static_cast<double>(MeasuredFrames) : 0.0,
		MeasuredFrames > 0 ? RunSweepMilliseconds / static_cast<double>(MeasuredFrames) : 0.0,
		MeasuredFrames > 0 ? RunStaticAuthorityMilliseconds / static_cast<double>(MeasuredFrames) : 0.0,
		RunMaximumStaticAuthorityMilliseconds,
		MeasuredFrames > 0 ? RunProjectionMilliseconds / static_cast<double>(MeasuredFrames) : 0.0,
		MeasuredFrames > 0 ? RunPostMilliseconds / static_cast<double>(MeasuredFrames) : 0.0,
		RunStaticQueryCount,
		MeasuredFrames > 0 ? static_cast<double>(RunStaticQueryCount) / static_cast<double>(MeasuredFrames) : 0.0,
		RunMaximumStaticQueryCount,
		MeasuredFrames > 0 ? static_cast<double>(RunComponentSweepCount) / static_cast<double>(MeasuredFrames) : 0.0,
		MeasuredFrames > 0 ? static_cast<double>(RunIterationCount) / static_cast<double>(MeasuredFrames) : 0.0,
		RunMaximumIterationCount);
	bRunMetricsActive = false;
}

#if !UE_BUILD_SHIPPING
void AFastSimulation::RecordWindowTiming()
{
	if (bWindowTimingInvalid) return;
	if (CanonicalNumFrame < WindowStartFrame || CanonicalNumFrame - WindowStartFrame > 30u)
	{
		bWindowTimingInvalid = true; // A discontinuity cannot be relabelled a contiguous window.
		return;
	}
	if (CanonicalNumFrame - WindowStartFrame != 30u) return;
	if (WindowMilliseconds.Num() == 65536)
	{
		bWindowTimingInvalid = true; // Bound memory; the reader must reject this report.
		return;
	}
	const double EndSeconds = FPlatformTime::Seconds();
	WindowMilliseconds.Add((EndSeconds - WindowStartSeconds) * 1000.0);
	WindowStartFrame = CanonicalNumFrame;
	WindowStartSeconds = EndSeconds;
}

void AFastSimulation::ReportWindowTimings() const
{
	constexpr double DeadlineMs = 1000.0 / 300.0;
	uint64 Misses = 0;
	for (int32 Index = 0; Index < WindowMilliseconds.Num(); ++Index)
	{
		const double Milliseconds = WindowMilliseconds[Index];
		if (Milliseconds > DeadlineMs) ++Misses;
		const uint64 First = RunStartFrame + uint64(Index) * 30u;
		UE_LOG(LogTemp, Display, TEXT("[FastSimulationWindow] FirstFrame=%llu LastFrame=%llu Frames=30 WallMs=%.9f"),
			First, First + 29u, Milliseconds);
	}
	TArray<double> Sorted = WindowMilliseconds;
	Sorted.Sort();
	const auto Percentile = [&Sorted](double Fraction)
	{
		return Sorted.IsEmpty() ? 0.0 : Sorted[FMath::Clamp(FMath::CeilToInt(Fraction * Sorted.Num()) - 1, 0, Sorted.Num() - 1)];
	};
	UE_LOG(LogTemp, Display, TEXT("[FastSimulationWindowSummary] Valid=%d Windows=%d FramesPerWindow=30 TailFrames=%llu DeadlineMs=%.9f Misses=%llu P50Ms=%.9f P95Ms=%.9f P99Ms=%.9f MaxMs=%.9f Scope=ForwardCommittedFrames CPUCostMeasured=0 RollbackRestoreMeasured=0"),
		bWindowTimingInvalid ? 0 : 1, Sorted.Num(), CanonicalNumFrame - WindowStartFrame, DeadlineMs, Misses,
		Percentile(0.50), Percentile(0.95), Percentile(0.99), Sorted.IsEmpty() ? 0.0 : Sorted.Last());
}
#endif
