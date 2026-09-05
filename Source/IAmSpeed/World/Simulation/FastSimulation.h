#pragma once

#include "SpeedSimulation.h"
#include "FastSimulation.generated.h"

/** Unpaced driver for a complete, previously sealed frame-addressed run. */
UCLASS()
class IAMSPEED_API AFastSimulation final : public ASpeedSimulation
{
	GENERATED_BODY()

protected:
	ESimulationWorkerResult DriveSimulation(float DeltaTime, float SimTime) override;
	ESimulationWorkerResult DriveOwnedWorkerPulse(float DeltaTime, float SimTime) override;
	/** FastSimulation intentionally performs no wait between committed frames. */
	void WaitBetweenFrames(FSimulationWorkerWaitContext& WaitContext) override {}
	bool ShouldMeasureCallback() const override { return false; }
	bool RequiresCanonicalRunController() const override { return true; }

private:
	/** Chooses a complete legacy pulse or one interruptible owned-worker frame. */
	ESimulationWorkerResult DriveFastSimulation(float SimTime, bool bSingleFramePulse);
	/** Advances one frame and updates aggregate metrics for the current sealed run. */
	ESimulationWorkerResult AdvanceSealedSimulation(uint32 MaxFrameCount);
	void BeginRunMetrics();
	void AccumulateRunFrameMetrics();
	void ReportCompletedRun();

	bool bFrameBudgetExceeded = false;
	bool bRunMetricsActive = false;
	uint64 RunStartFrame = 0;
	uint64 RunStartPerformanceFrameCount = 0;
	uint64 RunStartWarningFrameCount = 0;
	uint64 RunStartIterationLimitCount = 0;
	double RunStartTotalStepMilliseconds = 0.0;
	double RunStartSeconds = 0.0;
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
};
