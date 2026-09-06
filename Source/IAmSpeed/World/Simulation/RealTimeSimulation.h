#pragma once

#include "SpeedSimulation.h"
#include "RealTimeSimulation.generated.h"

/** Pure wall-clock state used to keep frame starts phase-locked without bursts. */
class IAMSPEED_API FRealTimeFramePacer final
{
public:
	struct FFrameStartSample
	{
#if !UE_BUILD_SHIPPING
		bool bHasPreviousStart = false;
		double IntervalSeconds = 0.0;
		double TargetIntervalSeconds = 0.0;
		double DeadlineErrorSeconds = 0.0;
		double RecoveryDebtSeconds = 0.0;
		bool bRecoveryLimited = false;
#endif
		double NextDeadlineSeconds = 0.0;
	};
#if !UE_BUILD_SHIPPING
	struct FFrameCountExpectation
	{
		double ExactFrames = 0.0;
		uint64 MinimumFrames = 0;
		uint64 MaximumFrames = 0;
	};
#endif

	/**
	 * Records an actual frame start and spreads recoverable lateness over the
	 * requested wall-clock window. The simulated delta itself never changes.
	 */
	FFrameStartSample BeginFrame(
		double StartSeconds,
		double FrameIntervalSeconds,
		double RecoveryWindowSeconds = 0.010);
#if !UE_BUILD_SHIPPING
	/** Builds the valid published-frame count for two asynchronously phased clocks. */
	static FFrameCountExpectation BuildFrameCountExpectation(
		double ObservationSeconds,
		double FrameIntervalSeconds);
#endif
	/** Clears timing history after a pause, restore, or interrupted wait. */
	void Reset();
	double GetNextDeadlineSeconds() const { return NextDeadlineSeconds; }

private:
#if !UE_BUILD_SHIPPING
	bool bHasPreviousStart = false;
	double PreviousStartSeconds = 0.0;
#endif
	double NextDeadlineSeconds = 0.0;
	double RecoveryDebtSeconds = 0.0;
	int32 RecoveryIntervalsRemaining = 0;
};

/** Paced real-time policy; the selected execution host advances canonical frames. */
UCLASS()
class IAMSPEED_API ARealTimeSimulation final : public ASpeedSimulation
{
	GENERATED_BODY()

public:
#if !UE_BUILD_SHIPPING
	/** Compares game-thread snapshot consumption with this real-time policy. */
	void AuditPresentationFrameCadence(
		const AActor& Observer,
		float GameDeltaSeconds,
		uint64& InOutLastObservedFrame) override;
#endif

protected:
	ESimulationWorkerResult DriveSimulation(float DeltaTime, float SimTime) override;
	/** Records the actual worker-frame start before entering the common frame body. */
	ESimulationWorkerResult DriveOwnedWorkerPulse(float DeltaTime, float SimTime) override;
	/** Waits for the phase-smoothed deadline selected by the real-time policy. */
	void WaitBetweenFrames(FSimulationWorkerWaitContext& WaitContext) override;
	/** Discards wall-clock history that no longer belongs to the active timeline. */
	void OnCanonicalTimelineRestored() override;
	void OnOwnedWorkerStarting() override;
	void OnOwnedSimulationPaused() override;
	void OnOwnedSimulationResumed() override;

private:
#if !UE_BUILD_SHIPPING
	void AuditPhysicalFrameStart(const FRealTimeFramePacer::FFrameStartSample& Sample);
#endif
	void ResetFramePacing();

	FRealTimeFramePacer FramePacer;
#if !UE_BUILD_SHIPPING
	double LastPhysicalSlowWarningSeconds = -DBL_MAX;
	double LastPhysicalFastWarningSeconds = -DBL_MAX;
	double LastPresentationSlowWarningSeconds = -DBL_MAX;
	double LastPresentationFastWarningSeconds = -DBL_MAX;
	double SuppressPresentationWarningsUntilSeconds = 0.0;
#endif
};
