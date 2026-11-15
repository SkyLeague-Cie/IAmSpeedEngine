#include "FastSimulation.h"
#include "IAmSpeed/World/Analytic/AnalyticNumericContext.h"
#include "SimulationSleepState.h"
#include "SimulationActorDiagnostics.h"

#include "CanonicalFrameContext.h"
#include "IAmSpeed/World/Collision/PreparedPhysicalExecution.h"
#include "CanonicalFrameDriver.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "HAL/PlatformTime.h"
#include "HAL/IConsoleManager.h"

#if !UE_BUILD_SHIPPING
static TAutoConsoleVariable<int32> CVarFastPeakPhases(
    TEXT("p.IAmSpeed.FastSimulation.PeakPhases"),0,
    TEXT("Default OFF. Retain the four hottest committed Fast frames and their already measured Step phases."));
static TAutoConsoleVariable<int32> CVarFastWindowTiming(
	TEXT("p.IAmSpeed.FastSimulation.WindowTiming"), 0,
	TEXT("Opt-in enclosing 30 committed-frame windows, including owned pulse overhead. Forward-run diagnostic, not rollback equivalence proof."));
#endif


// Diagnostic only: already acquired successful-frame counters. No authority,
// physics nonce, context application, query, live World or missing-start repair.
namespace
{
#if !UE_BUILD_SHIPPING
struct FFastPeakFrame
{
    uint64 Frame=0,PhysicalBits=0;
    FSpeedStepDiagnostics Step;
};
#endif
struct FExtraFastRunPhases
{
    const void* Owner=nullptr;
    uint64 Start=0,PerformanceStart=0,SecondsBits=0,Frames=0,Events=0,Attempts=0,Legacy=0;
    double Integrate=0,Resolve=0,Reset=0;
    bool Known=false;
#if !UE_BUILD_SHIPPING
    bool TrackPeaks=false,PeaksValid=true;
    uint32 PeakCount=0;
    FFastPeakFrame Peaks[4];
#endif
};
thread_local FExtraFastRunPhases ExtraFastPhases;
uint64 FastPhaseBits(double value)
{uint64 bits=0;FMemory::Memcpy(&bits,&value,sizeof(bits));return bits;}
#if !UE_BUILD_SHIPPING
bool NonnegativeFastPhaseBits(uint64 bits)
{return bits<0x7ff0000000000000ull||bits==0x8000000000000000ull;}
void RecordFastPeak(FExtraFastRunPhases& run,uint64 frame,double physical,const FSpeedStepDiagnostics& step)
{
    if(!run.TrackPeaks||!run.PeaksValid)return;
    uint64 physicalBits=FastPhaseBits(physical);
    if(!NonnegativeFastPhaseBits(physicalBits)){run.PeaksValid=false;return;}
    physicalBits&=0x7fffffffffffffffull;
    uint32 rank=0;
    while(rank<run.PeakCount&&physicalBits<=run.Peaks[rank].PhysicalBits)++rank;
    if(rank>=UE_ARRAY_COUNT(run.Peaks))return;
    for(double phase:{step.TotalMilliseconds,step.ResetMilliseconds,step.IntegrateMilliseconds,
        step.SweepMilliseconds,step.ResolveMilliseconds,step.ProjectionMilliseconds,step.PostMilliseconds,
        step.StaticAuthorityMilliseconds,step.MaximumStaticAuthorityMilliseconds})
        if(!NonnegativeFastPhaseBits(FastPhaseBits(phase))){run.PeaksValid=false;return;}
    if(run.PeakCount<UE_ARRAY_COUNT(run.Peaks))++run.PeakCount;
    for(uint32 i=run.PeakCount-1;i>rank;--i)run.Peaks[i]=run.Peaks[i-1];
    run.Peaks[rank]={frame,physicalBits,step};
}
void EmitFastPeaks(const FExtraFastRunPhases& run,bool known,bool complete)
{
    if(!run.TrackPeaks)return;
    const bool valid=known&&run.PeaksValid&&run.PeakCount>0;
    if(!valid)
    {
        UE_LOG(LogTemp,Display,TEXT("[FastSimulationPeakPhases] Known=0 Complete=%d StartFrame=%llu"),int32(complete),run.Start);
        return;
    }
    for(uint32 rank=0;rank<run.PeakCount;++rank)
    {
        const auto& peak=run.Peaks[rank];const auto& step=peak.Step;
        double physical=0.;FMemory::Memcpy(&physical,&peak.PhysicalBits,sizeof(physical));
        UE_LOG(LogTemp,Display,TEXT("[FastSimulationPeakPhases] Known=1 Complete=%d StartFrame=%llu Frame=%llu StepSerial=%llu Rank=%u PhysicalMs=%.9f CoreMs=%.9f ResetMs=%.9f IntegrateMs=%.9f SweepMs=%.9f ResolveMs=%.9f ProjectionMs=%.9f PostMs=%.9f StaticAuthorityMs=%.9f MaxStaticQueryMs=%.9f StaticQueries=%u AuthorityAttempts=%u Legacy=%u ComponentSweeps=%d Events=%d Iterations=%d IterationLimit=%d InclusivePhasesNonAdditive=1"),
            int32(complete),run.Start,peak.Frame,step.Serial,rank+1,physical,step.TotalMilliseconds,
            step.ResetMilliseconds,step.IntegrateMilliseconds,step.SweepMilliseconds,step.ResolveMilliseconds,
            step.ProjectionMilliseconds,step.PostMilliseconds,step.StaticAuthorityMilliseconds,
            step.MaximumStaticAuthorityMilliseconds,step.StaticQueryCount,step.AuthorityAttemptCount,
            step.LegacySweepCount,step.ComponentSweepCount,step.ResolvedEventCount,step.IterationCount,
            int32(step.bIterationLimitReached));
    }
}
#endif
bool FastPhaseKey(const void* owner,uint64 start,uint64 performance,double seconds)
{
    const auto& e=ExtraFastPhases;
    return e.Owner==owner&&e.Start==start&&e.PerformanceStart==performance&&e.SecondsBits==FastPhaseBits(seconds);
}
bool FastPhaseAdd(uint64& target,uint64 value)
{if(target>MAX_uint64-value)return false;target+=value;return true;}
void EmitExtraFastPhases(const void* owner,uint64 start,uint64 performance,double seconds,
    uint64 measured,uint64 successful,bool complete,bool countersCoherent)
{
    const auto& e=ExtraFastPhases;const bool key=FastPhaseKey(owner,start,performance,seconds);
    const bool known=countersCoherent&&e.Known&&key&&e.Frames==measured&&measured==successful;
#if !UE_BUILD_SHIPPING
    if(key)EmitFastPeaks(e,known,complete);
#endif
    if(!known)
    {
        UE_LOG(LogTemp,Display,TEXT("[FastSimulationDetailedPhases] Known=0 Complete=%d NoPASS=1 SuccessfulFramesOnly=1 InclusivePhasesNonAdditive=1 RunKeyMatches=%d CapturedFrames=%llu MeasuredFrames=%llu SuccessfulFrames=%llu"),
            int32(complete),int32(key),e.Frames,measured,successful);
        return;
    }
    UE_LOG(LogTemp,Display,TEXT("[FastSimulationDetailedPhases] Known=1 Complete=%d NoPASS=1 SuccessfulFramesOnly=1 InclusivePhasesNonAdditive=1 StartFrame=%llu Frames=%llu IntegrateTotalMs=%.9f ResolveTotalMs=%.9f ResetTotalMs=%.9f ResolvedCCDEvents=%llu AuthorityAttempts=%llu LegacySweeps=%llu ResolveScope=WorldCCDBatchNotAllSupportCorrections"),
        int32(complete),start,e.Frames,e.Integrate,e.Resolve,e.Reset,e.Events,e.Attempts,e.Legacy);
}
}

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
	if (Readiness == ESimulationWorkerResult::Complete)
	{
        // Native completion is observed by the guarded readiness aggregation
        // in a NEW window without another committed frame or World effects.
        if(Speed::PreparedExecution::Requested()&&bRunMetricsActive)ReportCompletedRun();
		return Readiness;
	}
    if(Readiness==ESimulationWorkerResult::Failed)return Readiness;
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
    // Only a true BeginRun initializes this thread-local observation.
    ExtraFastPhases={};ExtraFastPhases.Owner=this;ExtraFastPhases.Start=RunStartFrame;
    ExtraFastPhases.PerformanceStart=RunStartPerformanceFrameCount;
    ExtraFastPhases.SecondsBits=FastPhaseBits(RunStartSeconds);ExtraFastPhases.Known=true;
#if !UE_BUILD_SHIPPING
    ExtraFastPhases.TrackPeaks=CVarFastPeakPhases.GetValueOnAnyThread()==1;
#endif
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
    auto& extra=ExtraFastPhases;
    const bool monotone=CanonicalNumFrame>=RunStartFrame&&PerformanceFrameCount>=RunStartPerformanceFrameCount;
    const uint64 committed=monotone?CanonicalNumFrame-RunStartFrame:0;
    const uint64 measured=monotone?PerformanceFrameCount-RunStartPerformanceFrameCount:0;
    if(!extra.Known||!FastPhaseKey(this,RunStartFrame,RunStartPerformanceFrameCount,RunStartSeconds)||
        !monotone||CanonicalNumFrame==0||Diagnostics.Frame!=uint32(CanonicalNumFrame-1u)||extra.Frames==MAX_uint64||extra.Frames+1!=committed||committed!=measured)
        extra.Known=false;
    else
    {
        ++extra.Frames;
#if !UE_BUILD_SHIPPING
        RecordFastPeak(extra,CanonicalNumFrame-1u,LastPhysicalFrameMilliseconds,Diagnostics);
#endif
        if(Diagnostics.ResolvedEventCount<0||
            !FMath::IsFinite(Diagnostics.IntegrateMilliseconds)||Diagnostics.IntegrateMilliseconds<0||
            !FMath::IsFinite(Diagnostics.ResolveMilliseconds)||Diagnostics.ResolveMilliseconds<0||
            !FMath::IsFinite(Diagnostics.ResetMilliseconds)||Diagnostics.ResetMilliseconds<0)
            extra.Known=false;
        else
        {
            extra.Known=FastPhaseAdd(extra.Events,uint64(Diagnostics.ResolvedEventCount))&&
                FastPhaseAdd(extra.Attempts,uint64(Diagnostics.AuthorityAttemptCount))&&
                FastPhaseAdd(extra.Legacy,uint64(Diagnostics.LegacySweepCount));
            extra.Integrate+=Diagnostics.IntegrateMilliseconds;extra.Resolve+=Diagnostics.ResolveMilliseconds;
            extra.Reset+=Diagnostics.ResetMilliseconds;
            extra.Known=extra.Known&&FMath::IsFinite(extra.Integrate)&&FMath::IsFinite(extra.Resolve)&&FMath::IsFinite(extra.Reset);
        }
    }
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
#if !UE_BUILD_SHIPPING
        // Integer-only inspection: failed numeric context is NEVER repaired,
        // timed or formatted. The original failure remains the sole evidence.
        const auto Numeric=Speed::Analytic::InspectCurrentAnalyticNumericContext(Speed::Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(Numeric.Status==Speed::Analytic::EAnalyticNumericStatus::Complete)
        {
        // Failure-only readback of already accumulated members. No live World,
        // pose, authority query or success/completion observer is consulted.
        const double ObservedSeconds=FPlatformTime::Seconds(); // Before logging.
        const bool CountersCoherent=CanonicalNumFrame>=RunStartFrame&&PerformanceFrameCount>=RunStartPerformanceFrameCount;
        const uint64 SuccessfulFrames=CountersCoherent?CanonicalNumFrame-RunStartFrame:0;
        const uint64 MeasuredFrames=CountersCoherent?PerformanceFrameCount-RunStartPerformanceFrameCount:0;
        const float NativeDelta=FCanonicalFrameContext::CanonicalPhysicalDeltaTime;
        uint32 NativeDeltaBits=0;static_assert(sizeof(NativeDelta)==sizeof(NativeDeltaBits));
        FMemory::Memcpy(&NativeDeltaBits,&NativeDelta,sizeof(NativeDeltaBits));
        UE_LOG(LogTemp, Display,
            TEXT("[FastSimulationFailedPrefix] Complete=0 ScenarioAccepted=0 StartFrame=%llu NextFrame=%llu SuccessfulFrames=%llu MeasuredFrames=%llu CountersMonotone=%d CountsMatch=%d Hz=%u NativeDeltaBits=%08X NativeSimulatedSeconds=%.9f PartialRunWallIncludingFailedAttempt=%.9f CompletedStepWallMs=%.9f WorkerWallNotCPU=1 IncludesFailedAttemptWall=1 FirstSuccessfulFrameIncluded=1"),
            RunStartFrame,CanonicalNumFrame,SuccessfulFrames,MeasuredFrames,int32(CountersCoherent),int32(CountersCoherent&&SuccessfulFrames==MeasuredFrames),FCanonicalFrameContext::PhysicsFramesPerSecond,NativeDeltaBits,
            double(SuccessfulFrames)*double(NativeDelta),ObservedSeconds-RunStartSeconds,PerformanceTotalStepMilliseconds-RunStartTotalStepMilliseconds);
        UE_LOG(LogTemp, Display,
            TEXT("[FastSimulationFailedPrefixPhases] Complete=0 ScenarioAccepted=0 MeasuredFrames=%llu SuccessfulFramesOnly=1 InclusivePhasesNotAdditive=1 MaximumStepMs=%.9f CoreTotalMs=%.9f SweepTotalMs=%.9f ProjectionTotalMs=%.9f PostTotalMs=%.9f StaticAuthorityTotalMs=%.9f StaticAuthorityMaximumMs=%.9f StaticQueries=%llu ComponentSweeps=%llu Iterations=%llu"),
            MeasuredFrames,RunMaximumStepMilliseconds,RunCoreStepMilliseconds,RunSweepMilliseconds,RunProjectionMilliseconds,RunPostMilliseconds,
            RunStaticAuthorityMilliseconds,RunMaximumStaticAuthorityMilliseconds,RunStaticQueryCount,RunComponentSweepCount,RunIterationCount);
        Speed::ActorDiagnostics::DumpFailedPrefix();
        EmitExtraFastPhases(this,RunStartFrame,RunStartPerformanceFrameCount,RunStartSeconds,MeasuredFrames,SuccessfulFrames,false,CountersCoherent);
        }
#endif
		return ESimulationWorkerResult::Failed;
	}
	AccumulateRunFrameMetrics();
#if !UE_BUILD_SHIPPING
	if (bMeasureWindows) RecordWindowTiming();
#endif
    // One real committed native frame is Advanced, even when its Post marked
    // the scenario complete. The next pulse closes Complete without effects;
    // never call the live controller again outside nativeReady's two checks.
    if(Speed::PreparedExecution::Requested())return ESimulationWorkerResult::Advanced;
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
    EmitExtraFastPhases(this,RunStartFrame,RunStartPerformanceFrameCount,RunStartSeconds,MeasuredFrames,SimulatedFrames,true,CanonicalNumFrame>=RunStartFrame&&PerformanceFrameCount>=RunStartPerformanceFrameCount);
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
