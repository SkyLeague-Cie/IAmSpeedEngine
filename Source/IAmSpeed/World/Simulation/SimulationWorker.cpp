#include "SimulationWorker.h"

#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "HAL/RunnableThread.h"

FSimulationWorker::FSimulationWorker(
	FWork&& InWork, FWaitBetweenFrames&& InWaitBetweenFrames)
	: Work(MoveTemp(InWork))
	, WaitBetweenFrames(MoveTemp(InWaitBetweenFrames))
	, WakeEvent(FPlatformProcess::GetSynchEventFromPool(false))
	, PauseAcknowledgedEvent(FPlatformProcess::GetSynchEventFromPool(true))
{
}

ESimulationWaitResult FSimulationWorkerWaitContext::WaitUntil(
	const double DeadlineSeconds,
	const double PrecisionWindowSeconds) const
{
	const double ClampedPrecisionWindowSeconds = FMath::Clamp(
		PrecisionWindowSeconds, 0.0, 0.005);
	while (!StopRequested.Load() && !Paused.Load())
	{
		const double RemainingSeconds = DeadlineSeconds - FPlatformTime::Seconds();
		if (RemainingSeconds <= 0.0)
		{
			return ESimulationWaitResult::DeadlineReached;
		}
		const double CoarseWaitSeconds =
			RemainingSeconds - ClampedPrecisionWindowSeconds;
		const int32 CoarseWaitMilliseconds = FMath::FloorToInt(
			CoarseWaitSeconds * 1000.0);
		if (CoarseWaitMilliseconds > 0)
		{
			WakeEvent.Wait(static_cast<uint32>(FMath::Min(
				CoarseWaitMilliseconds, 100)));
		}
		else if (ClampedPrecisionWindowSeconds > 0.0)
		{
			// Yielding keeps the short precision tail interruptible and avoids the
			// systematic oversleep caused by rounding a sub-millisecond tail up.
			FPlatformProcess::YieldThread();
		}
		else
		{
			WakeEvent.Wait(1);
		}
	}
	return ESimulationWaitResult::Interrupted;
}

FSimulationWorker::~FSimulationWorker()
{
	StopAndJoin();
	if (WakeEvent)
	{
		FPlatformProcess::ReturnSynchEventToPool(WakeEvent);
		WakeEvent = nullptr;
	}
	if (PauseAcknowledgedEvent)
	{
		FPlatformProcess::ReturnSynchEventToPool(PauseAcknowledgedEvent);
		PauseAcknowledgedEvent = nullptr;
	}
}

bool FSimulationWorker::Start()
{
	if (Thread || !Work || !WaitBetweenFrames || !WakeEvent ||
		!PauseAcknowledgedEvent)
	{
		return false;
	}
	bStopRequested.Store(false);
	bPaused.Store(false);
	PauseAcknowledgedEvent->Reset();
	bRunning.Store(true);
	Thread = FRunnableThread::Create(
		this, TEXT("IAmSpeedSimulation"), 0, TPri_AboveNormal);
	if (!Thread)
	{
		bRunning.Store(false);
	}
	return Thread != nullptr;
}

void FSimulationWorker::Pause()
{
	if (!bPaused.Load() && PauseAcknowledgedEvent)
	{
		PauseAcknowledgedEvent->Reset();
	}
	bPaused.Store(true);
	if (WakeEvent)
	{
		WakeEvent->Trigger();
	}

	// Pause is a boundary operation: callers may safely freeze presentation as
	// soon as this method returns because no canonical frame is still running.
	while (bRunning.Load() && PauseAcknowledgedEvent &&
		!PauseAcknowledgedEvent->Wait(100))
	{
	}
}

void FSimulationWorker::Resume()
{
	bPaused.Store(false);
	if (WakeEvent)
	{
		WakeEvent->Trigger();
	}
}

void FSimulationWorker::Stop()
{
	bStopRequested.Store(true);
	if (WakeEvent)
	{
		WakeEvent->Trigger();
	}
}

void FSimulationWorker::StopAndJoin()
{
	Stop();
	if (Thread)
	{
		Thread->WaitForCompletion();
		delete Thread;
		Thread = nullptr;
	}
}

uint32 FSimulationWorker::Run()
{
	FSimulationWorkerWaitContext WaitContext(
		*WakeEvent, bStopRequested, bPaused);
	while (!bStopRequested.Load())
	{
		if (bPaused.Load())
		{
			PauseAcknowledgedEvent->Trigger();
			while (bPaused.Load() && !bStopRequested.Load())
			{
				WakeEvent->Wait(100);
				PauseAcknowledgedEvent->Trigger();
			}
			PauseAcknowledgedEvent->Reset();
			continue;
		}

		const ESimulationWorkerResult Result = Work();
		if (Result == ESimulationWorkerResult::Complete ||
			Result == ESimulationWorkerResult::Failed)
		{
			break;
		}
		if (Result == ESimulationWorkerResult::Idle)
		{
			WakeEvent->Wait(1);
			continue;
		}
		WaitBetweenFrames(WaitContext);
	}
	PauseAcknowledgedEvent->Trigger();
	bRunning.Store(false);
	return 0;
}
