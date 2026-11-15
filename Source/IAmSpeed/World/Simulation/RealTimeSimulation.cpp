#include "RealTimeSimulation.h"

#include "CanonicalFrameDriver.h"
#include "CanonicalFrameContext.h"
#if !UE_BUILD_SHIPPING
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "IAmSpeed/World/Analytic/ReplayTypedClock918.h"
#endif
#if !UE_BUILD_SHIPPING
#include "IAmSpeed/IAmSpeed.h"
#endif
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTime.h"
#include "Misc/ScopeExit.h"

namespace
{
	constexpr double MaximumRecoveryDebtFrames = 2.0;
	constexpr double MaximumIntervalRecoveryFraction = 0.25;

#if !UE_BUILD_SHIPPING
    TAutoConsoleVariable<int32> CVarRealTimeLatePulseTiming(
        TEXT("p.IAmSpeed.Simulation.RealTimeLatePulseTiming"),0,TEXT("Attribute real-time starts to the preceding owned pulse. 1: >25 ms. 2: >4 ms plus sparse samples."));
	TAutoConsoleVariable<int32> CVarRealTimeCadenceWarnings(
		TEXT("p.IAmSpeed.Simulation.RealTimeCadenceWarnings"), 1,
		TEXT("Reports worker-start and game-thread snapshot cadence anomalies."));
	TAutoConsoleVariable<float> CVarRealTimeCadenceToleranceMilliseconds(
		TEXT("p.IAmSpeed.Simulation.RealTimeCadenceToleranceMs"), 0.25f,
		TEXT("Allowed deviation of a real-time physical-frame start interval, in milliseconds."));
	TAutoConsoleVariable<float> CVarRealTimeCadenceWarningCooldownSeconds(
		TEXT("p.IAmSpeed.Simulation.RealTimeCadenceWarningCooldownSeconds"), 0.5f,
		TEXT("Minimum delay between repeated cadence warnings of the same kind."));
#endif
	TAutoConsoleVariable<float> CVarRealTimePrecisionWindowMilliseconds(
		TEXT("p.IAmSpeed.Simulation.RealTimePrecisionWindowMs"), 0.10f,
		TEXT("Final wait window reserved for thread yields instead of millisecond event waits."));
	TAutoConsoleVariable<float> CVarRealTimeRecoveryWindowMilliseconds(
		TEXT("p.IAmSpeed.Simulation.RealTimeRecoveryWindowMs"), 10.0f,
		TEXT("Wall-clock window used to smooth recoverable real-time cadence debt; zero disables recovery."));
}

FRealTimeFramePacer::FFrameStartSample FRealTimeFramePacer::BeginFrame(
	const double StartSeconds,
	const double FrameIntervalSeconds,
	const double RecoveryWindowSeconds)
{
	FFrameStartSample Sample;
	const double SafeFrameIntervalSeconds = FMath::Max(0.0, FrameIntervalSeconds);
	const double PreviousDeadlineSeconds = NextDeadlineSeconds;
#if !UE_BUILD_SHIPPING
	Sample.bHasPreviousStart = bHasPreviousStart;
	if (bHasPreviousStart)
	{
		Sample.IntervalSeconds = StartSeconds - PreviousStartSeconds;
		Sample.TargetIntervalSeconds = FMath::Max(
			0.0, PreviousDeadlineSeconds - PreviousStartSeconds);
		Sample.DeadlineErrorSeconds = StartSeconds - PreviousDeadlineSeconds;
	}
	PreviousStartSeconds = StartSeconds;
	bHasPreviousStart = true;
#endif

	double AppliedRecoverySeconds = 0.0;
	bool bRecoveryLimited = false;
	if (PreviousDeadlineSeconds > 0.0 && SafeFrameIntervalSeconds > 0.0 &&
		RecoveryWindowSeconds > 0.0)
	{
		const double UnexpectedLatenessSeconds = FMath::Max(
			0.0, StartSeconds - PreviousDeadlineSeconds);
		const double MaximumRecoveryDebtSeconds =
			SafeFrameIntervalSeconds * MaximumRecoveryDebtFrames;
		const double UnclampedRecoveryDebtSeconds =
			RecoveryDebtSeconds + UnexpectedLatenessSeconds;
		RecoveryDebtSeconds = FMath::Min(
			UnclampedRecoveryDebtSeconds, MaximumRecoveryDebtSeconds);
		bRecoveryLimited = UnclampedRecoveryDebtSeconds >
			MaximumRecoveryDebtSeconds + UE_DOUBLE_SMALL_NUMBER;

		if (UnexpectedLatenessSeconds > UE_DOUBLE_SMALL_NUMBER ||
			RecoveryIntervalsRemaining <= 0)
		{
			RecoveryIntervalsRemaining = FMath::Max(1, FMath::CeilToInt(
				RecoveryWindowSeconds / SafeFrameIntervalSeconds));
		}

		if (RecoveryDebtSeconds > UE_DOUBLE_SMALL_NUMBER)
		{
			const double DesiredRecoverySeconds = RecoveryDebtSeconds /
				static_cast<double>(RecoveryIntervalsRemaining);
			const double MaximumRecoverySeconds = SafeFrameIntervalSeconds *
				MaximumIntervalRecoveryFraction;
			AppliedRecoverySeconds = FMath::Min(
				DesiredRecoverySeconds, MaximumRecoverySeconds);
			bRecoveryLimited |= DesiredRecoverySeconds >
				MaximumRecoverySeconds + UE_DOUBLE_SMALL_NUMBER;
			RecoveryDebtSeconds = FMath::Max(
				0.0, RecoveryDebtSeconds - AppliedRecoverySeconds);
			--RecoveryIntervalsRemaining;
		}
	}
	else
	{
		RecoveryDebtSeconds = 0.0;
		RecoveryIntervalsRemaining = 0;
	}

	NextDeadlineSeconds = StartSeconds + FMath::Max(
		0.0, SafeFrameIntervalSeconds - AppliedRecoverySeconds);
	Sample.NextDeadlineSeconds = NextDeadlineSeconds;
#if !UE_BUILD_SHIPPING
	Sample.RecoveryDebtSeconds = RecoveryDebtSeconds;
	Sample.bRecoveryLimited = bRecoveryLimited;
#endif
	return Sample;
}

void FRealTimeFramePacer::Reset()
{
#if !UE_BUILD_SHIPPING
	bHasPreviousStart = false;
	PreviousStartSeconds = 0.0;
#endif
	NextDeadlineSeconds = 0.0;
	RecoveryDebtSeconds = 0.0;
	RecoveryIntervalsRemaining = 0;
}

#if !UE_BUILD_SHIPPING
FRealTimeFramePacer::FFrameCountExpectation
FRealTimeFramePacer::BuildFrameCountExpectation(
	const double ObservationSeconds,
	const double FrameIntervalSeconds)
{
	FFrameCountExpectation Expectation;
	Expectation.ExactFrames = FMath::Max(0.0, ObservationSeconds) /
		FMath::Max(FrameIntervalSeconds, UE_DOUBLE_SMALL_NUMBER);
	const double NearestWholeFrames = FMath::RoundToDouble(Expectation.ExactFrames);
	const double StableExactFrames = FMath::IsNearlyEqual(
		Expectation.ExactFrames, NearestWholeFrames, 0.0001)
		? NearestWholeFrames
		: Expectation.ExactFrames;
	Expectation.MinimumFrames = static_cast<uint64>(FMath::FloorToDouble(
		StableExactFrames));
	Expectation.MaximumFrames = Expectation.MinimumFrames + 1u;
	return Expectation;
}
#endif

ESimulationWorkerResult ARealTimeSimulation::DriveSimulation(
	const float DeltaTime, const float SimTime)
{
	RealDeltaTime = 1.0f / static_cast<float>(EngineFPS);
	if (!Speed::CanonicalFrameDriver::IsEnabled())
	{
		ResetCanonicalFrame();
		UpdateNumFrame(SimTime);
		Simulate(DeltaTime, SimTime);
		return ESimulationWorkerResult::Advanced;
	}
	if (!EnsureSimulationWorldReady()) return ESimulationWorkerResult::Idle;
	InitializeCanonicalFrame(SimTime);
	const ESimulationWorkerResult Readiness = CheckCanonicalRunReadiness();
	if (Readiness != ESimulationWorkerResult::Advanced)
	{
#if !UE_BUILD_SHIPPING
        if (Readiness == ESimulationWorkerResult::Complete)
            EndIndependentRealTimeCoverage696(true,TEXT("Completed"));
        else if (Readiness == ESimulationWorkerResult::Failed)
            EndIndependentRealTimeCoverage696(false,TEXT("ReadinessFailed"));
#endif
		return Readiness;
	}
#if !UE_BUILD_SHIPPING
    BeginIndependentRealTimeCoverage696();
#endif
    const bool bAdvanced = RunCanonicalFrames(1u);
#if !UE_BUILD_SHIPPING
    if (!bAdvanced) EndIndependentRealTimeCoverage696(false,TEXT("CanonicalFrameFailed"));
    else if (SpeedWorldSubsystem->InspectAdmittedCanonicalRunControlForTesting() ==
        ECanonicalRunControlState::Complete)
    {
        // RunCanonicalFrames has returned: publication, all observer scopes,
        // frame timing and the integer-frame advance have finished. The GT may
        // request a pause/exit immediately, before another readiness pulse.
        // This is an observer close only; the driver still returns Advanced.
        EndIndependentRealTimeCoverage696(true,TEXT("CompletedAfterCommittedFrame"));
    }
#endif
	return bAdvanced ? ESimulationWorkerResult::Advanced : ESimulationWorkerResult::Failed;
}

ESimulationWorkerResult ARealTimeSimulation::DriveOwnedWorkerPulse(
	const float DeltaTime, const float SimTime)
{
	const double FrameStartSeconds = FPlatformTime::Seconds();
#if !UE_BUILD_SHIPPING
    ActivePublicationTiming = {PacingEpoch.load(std::memory_order_relaxed), FrameStartSeconds, true};
    ON_SCOPE_EXIT { ActivePublicationTiming = {}; };
    using namespace Speed::Analytic::Private;
    const int32 timingMode=CVarRealTimeLatePulseTiming.GetValueOnAnyThread();
    const bool timing=timingMode==1||timingMode==2;
    const auto cpuStart=timing?ReadReplayClock918(EReplayClock918::ThreadCycleAccounting):FReplayClockRead918{};
    struct FPreviousPulse
    {
        const ARealTimeSimulation* Owner=nullptr;
        uint64 Epoch=0,CanonicalAfter=0,Cycles=0;
        double Elapsed=0.;
        bool CpuValid=false;
        FSpeedStepDiagnostics Step;
    };
    static thread_local FPreviousPulse previous;
#endif
	const ESimulationWorkerResult Result = DriveSimulation(DeltaTime, SimTime);
#if !UE_BUILD_SHIPPING
    const double pulseElapsed=timing?FPlatformTime::Seconds()-FrameStartSeconds:0.;
    const auto cpuEnd=timing?ReadReplayClock918(EReplayClock918::ThreadCycleAccounting):FReplayClockRead918{};
    if(!timing||Result!=ESimulationWorkerResult::Advanced)previous={};
#endif
	if (Result == ESimulationWorkerResult::Advanced)
	{
		const double RecoveryWindowSeconds = FMath::Max(0.0,
			static_cast<double>(CVarRealTimeRecoveryWindowMilliseconds.GetValueOnAnyThread()) /
			1000.0);
#if !UE_BUILD_SHIPPING
		const FRealTimeFramePacer::FFrameStartSample Sample = FramePacer.BeginFrame(
			FrameStartSeconds, static_cast<double>(RealDeltaTime), RecoveryWindowSeconds);
        if(timing)
        {
            const uint64 epoch=PacingEpoch.load(std::memory_order_relaxed);
            const bool report=timingMode==1?Sample.IntervalSeconds>.025:
                (Sample.IntervalSeconds>.004||(CanonicalNumFrame%64u)==0);
            if(previous.Owner==this&&previous.Epoch==epoch&&CanonicalNumFrame>0&&
                previous.CanonicalAfter==CanonicalNumFrame-1u&&Sample.bHasPreviousStart&&report)
            {
                const auto& step=previous.Step;
                UE_LOG(SpeedPhysicsLog,Display,TEXT("[RealTimeLatePulseTiming] FrameAfter=%llu IntervalMs=%.6f PreviousPulseMs=%.6f OutsidePulseMs=%.6f TargetMs=%.6f PulsePastTargetMs=%.6f OutsidePastTargetMs=%.6f PreviousThreadCycles=%llu CpuValid=%d StepKnown=%d StepMs=%.6f SweepMs=%.6f ResolveMs=%.6f IntegrateMs=%.6f ProjectionMs=%.6f PostMs=%.6f StadiumMs=%.6f StaticQueries=%u"),
                    CanonicalNumFrame,Sample.IntervalSeconds*1000.,previous.Elapsed*1000.,
                    (Sample.IntervalSeconds-previous.Elapsed)*1000.,Sample.TargetIntervalSeconds*1000.,
                    FMath::Max(0.,previous.Elapsed-Sample.TargetIntervalSeconds)*1000.,
                    FMath::Max(0.,Sample.IntervalSeconds-FMath::Max(previous.Elapsed,Sample.TargetIntervalSeconds))*1000.,
                    previous.Cycles,int32(previous.CpuValid),
                    int32(step.Serial!=0),step.TotalMilliseconds,step.SweepMilliseconds,step.ResolveMilliseconds,
                    step.IntegrateMilliseconds,step.ProjectionMilliseconds,step.PostMilliseconds,step.StaticAuthorityMilliseconds,step.StaticQueryCount);
            }
            const auto cycles=DifferenceReplayClock918(cpuStart,cpuEnd);
            previous={this,epoch,CanonicalNumFrame,cycles.Raw,pulseElapsed,cycles.bValid,{}};
            if(SpeedWorldSubsystem&&CanonicalNumFrame>0&&CanonicalNumFrame-1u<=MAX_uint32)
            {
                const auto& step=SpeedWorldSubsystem->GetLastStepDiagnostics();
                if(step.Serial&&step.Frame==CanonicalNumFrame-1u)previous.Step=step;
            }
        }
		AuditPhysicalFrameStart(Sample);
        if (Speed::CanonicalFrameDriver::IsEnabled())
        {
            Speed::PublicationCadence::FStartOutcome Outcome;
            Outcome.ActualIntervalSeconds = Sample.IntervalSeconds;
            Outcome.TargetIntervalSeconds = Sample.TargetIntervalSeconds;
            Outcome.BaseIntervalSeconds = static_cast<double>(RealDeltaTime);
            Outcome.RecoveryDebtSeconds = Sample.RecoveryDebtSeconds;
            Outcome.HasPreviousStart = Sample.bHasPreviousStart;
            Outcome.RecoveryLimited = Sample.bRecoveryLimited;
            if (!SnapshotBuffer.CompleteCadenceTiming(CanonicalNumFrame - 1u, FrameStartSeconds, Outcome))
                UE_LOG(SpeedPhysicsLog, Error, TEXT("[RealTimePublicationCadenceIncomplete] Frame=%llu"), CanonicalNumFrame - 1u);
        }
#else
		FramePacer.BeginFrame(
			FrameStartSeconds, static_cast<double>(RealDeltaTime), RecoveryWindowSeconds);
#endif
	}
	return Result;
}

void ARealTimeSimulation::WaitBetweenFrames(
	FSimulationWorkerWaitContext& WaitContext)
{
	double DeadlineSeconds = FramePacer.GetNextDeadlineSeconds();
	if (DeadlineSeconds <= 0.0)
	{
		// A restore can reset pacing inside the frame body. Starting a fresh
		// interval after that work is conservative and cannot create a burst.
		DeadlineSeconds = FPlatformTime::Seconds() +
			static_cast<double>(RealDeltaTime);
	}
	const double PrecisionWindowSeconds = FMath::Max(0.0,
		static_cast<double>(CVarRealTimePrecisionWindowMilliseconds.GetValueOnAnyThread()) /
		1000.0);
	if (WaitContext.WaitUntil(DeadlineSeconds, PrecisionWindowSeconds) ==
		ESimulationWaitResult::Interrupted)
	{
		ResetFramePacing();
	}
}

#if !UE_BUILD_SHIPPING
void ARealTimeSimulation::AuditPhysicalFrameStart(
	const FRealTimeFramePacer::FFrameStartSample& Sample)
{
	if (!Sample.bHasPreviousStart ||
		CVarRealTimeCadenceWarnings.GetValueOnAnyThread() == 0)
	{
		return;
	}

	const double ExpectedSeconds = Sample.TargetIntervalSeconds;
	const double ToleranceSeconds = FMath::Max(0.0,
		static_cast<double>(CVarRealTimeCadenceToleranceMilliseconds.GetValueOnAnyThread()) /
		1000.0);
	const double NowSeconds = FPlatformTime::Seconds();
	const double CooldownSeconds = FMath::Max(0.0,
		static_cast<double>(CVarRealTimeCadenceWarningCooldownSeconds.GetValueOnAnyThread()));
	if (Sample.IntervalSeconds + ToleranceSeconds < ExpectedSeconds &&
		NowSeconds - LastPhysicalFastWarningSeconds >= CooldownSeconds)
	{
		LastPhysicalFastWarningSeconds = NowSeconds;
		UE_LOG(SpeedPhysicsLog, Warning,
			TEXT("[RealTimeSimulationCadence] Physics Thread is too fast: Frame=%llu ActualStartIntervalMs=%.6f TargetStartIntervalMs=%.6f BaseStartIntervalMs=%.6f PhaseErrorMs=%.6f RecoveryDebtMs=%.6f"),
			CanonicalNumFrame, Sample.IntervalSeconds * 1000.0,
			ExpectedSeconds * 1000.0, static_cast<double>(RealDeltaTime) * 1000.0,
			Sample.DeadlineErrorSeconds * 1000.0,
			Sample.RecoveryDebtSeconds * 1000.0);
	}
	else if (Sample.bRecoveryLimited &&
		Sample.IntervalSeconds > ExpectedSeconds + ToleranceSeconds &&
		NowSeconds - LastPhysicalSlowWarningSeconds >= CooldownSeconds)
	{
		LastPhysicalSlowWarningSeconds = NowSeconds;
		UE_LOG(SpeedPhysicsLog, Warning,
			TEXT("[RealTimeSimulationCadence] Physics Thread is too slow: Frame=%llu ActualStartIntervalMs=%.6f TargetStartIntervalMs=%.6f BaseStartIntervalMs=%.6f PhaseErrorMs=%.6f RecoveryDebtMs=%.6f"),
			CanonicalNumFrame, Sample.IntervalSeconds * 1000.0,
			ExpectedSeconds * 1000.0, static_cast<double>(RealDeltaTime) * 1000.0,
			Sample.DeadlineErrorSeconds * 1000.0,
			Sample.RecoveryDebtSeconds * 1000.0);
	}
}
#endif

#if !UE_BUILD_SHIPPING
Speed::PublicationCadence::FTiming ARealTimeSimulation::GetCanonicalPublicationTiming(
    const FCanonicalFrameContext& Context) const
{
    if (!ActivePublicationTiming.Present || Context.bResimulationForTesting) return {};
    auto Timing = ActivePublicationTiming;
    Timing.Epoch = PacingEpoch.load(std::memory_order_relaxed);
    return Timing;
}

void ARealTimeSimulation::AuditPresentationFrameCadence(
    const AActor& Observer, const float GameDeltaSeconds,
    Speed::PublicationCadence::FObserver& Observation)
{
    if (!IsOwnedWorkerExecutionMode() || GameDeltaSeconds <= 0.0f ||
        IsOwnedSimulationPaused() || Speed::CanonicalFrameDriver::IsOwnedThreadPaused())
    {
        Observation = {};
        return;
    }
    const double ToleranceSeconds = FMath::Max(0.0,
        static_cast<double>(CVarRealTimeCadenceToleranceMilliseconds.GetValueOnAnyThread()) / 1000.0);
    // Availability at this read, not the packet previously frozen by an actor latch.
    // The unchanged pacer supplies actual starts/deadlines; visibility has its own
    // monotone clock. GameDelta cannot define the interval between these reads.
    const auto Result = SnapshotBuffer.ReadCadenceObservation(Observation, ToleranceSeconds);
    if (!Result.Available) return;
    // A missing receipt/clock/history is a diagnostic failure, never a green gap.
    if (!Result.HistoryComplete || Result.FrameAccountingMismatch)
    {
        UE_LOG(SpeedPhysicsLog, Error,
            TEXT("[RealTimePresentationCadence] Incomplete publication observation: Observer=%s HistoryComplete=%u FrameAccountingMismatch=%u Frame=%llu Serial=%llu"),
            *Observer.GetName(), uint32(Result.HistoryComplete), uint32(Result.FrameAccountingMismatch),
            uint64(Result.Latest.Frame), uint64(Result.Latest.Serial));
        return;
    }
    const double NowSeconds = Observation.ObservationSeconds;
    if (NowSeconds < SuppressPresentationWarningsUntilSeconds ||
        CVarRealTimeCadenceWarnings.GetValueOnGameThread() == 0) return;
    const double CooldownSeconds = FMath::Max(0.0,
        static_cast<double>(CVarRealTimeCadenceWarningCooldownSeconds.GetValueOnGameThread()));
    if (Result.Rebased) return;
    const auto& Early = Result.FirstEarlyStart;
    const auto& Late = Result.FirstLateStart;
    if (Result.EarlyStart && NowSeconds - LastPresentationFastWarningSeconds >= CooldownSeconds)
    {
        LastPresentationFastWarningSeconds = NowSeconds;
        UE_LOG(SpeedPhysicsLog, Warning,
            TEXT("[RealTimePresentationCadence] Physics Thread is too fast: Observer=%s Frame=%llu ActualStartIntervalMs=%.6f TargetStartIntervalMs=%.6f Publications=%llu ObservationIntervalMs=%.6f"),
            *Observer.GetName(), uint64(Early.Frame), Early.Start.ActualIntervalSeconds * 1000.0,
            Early.Start.TargetIntervalSeconds * 1000.0, uint64(Result.Publications),
            Result.ObservationIntervalSeconds * 1000.0);
    }
    if (Result.LateStart && NowSeconds - LastPresentationSlowWarningSeconds >= CooldownSeconds)
    {
        LastPresentationSlowWarningSeconds = NowSeconds;
        UE_LOG(SpeedPhysicsLog, Warning,
            TEXT("[RealTimePresentationCadence] Physics Thread is too slow: Observer=%s Frame=%llu ActualStartIntervalMs=%.6f TargetStartIntervalMs=%.6f Publications=%llu ObservationIntervalMs=%.6f"),
            *Observer.GetName(), uint64(Late.Frame), Late.Start.ActualIntervalSeconds * 1000.0,
            Late.Start.TargetIntervalSeconds * 1000.0, uint64(Result.Publications),
            Result.ObservationIntervalSeconds * 1000.0);
    }
}

#endif

void ARealTimeSimulation::ResetFramePacing()
{
	FramePacer.Reset();
#if !UE_BUILD_SHIPPING
    PacingEpoch.fetch_add(1, std::memory_order_relaxed);
#endif
}

void ARealTimeSimulation::OnCanonicalTimelineRestored()
{
	ResetFramePacing();
}

void ARealTimeSimulation::OnOwnedWorkerStarting()
{
	ResetFramePacing();
#if !UE_BUILD_SHIPPING
	SuppressPresentationWarningsUntilSeconds = FPlatformTime::Seconds() +
		FMath::Max(0.05, static_cast<double>(RealDeltaTime) * 2.0);
#endif
}

void ARealTimeSimulation::OnOwnedSimulationPaused()
{
	ResetFramePacing();
}

void ARealTimeSimulation::OnOwnedSimulationResumed()
{
	ResetFramePacing();
#if !UE_BUILD_SHIPPING
	SuppressPresentationWarningsUntilSeconds = FPlatformTime::Seconds() +
		FMath::Max(0.05, static_cast<double>(RealDeltaTime) * 2.0);
#endif
}
