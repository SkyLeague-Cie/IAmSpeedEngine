#include "SimulationWorker.h"

#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "HAL/RunnableThread.h"

FSimulationWorker::FSimulationWorker(
	FWork&& InWork, FWaitBetweenFrames&& InWaitBetweenFrames,
	FServiceBoundary InServiceBoundary, FCloseOnWorker InCloseOnWorker)
	: Work(MoveTemp(InWork))
	, WaitBetweenFrames(MoveTemp(InWaitBetweenFrames))
	, ServiceBoundary(MoveTemp(InServiceBoundary))
	, CloseOnWorker(MoveTemp(InCloseOnWorker))
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

bool FSimulationWorker::Start(bool bStartPaused)
{
	if (Thread || !Work || !WaitBetweenFrames || !WakeEvent ||
		!PauseAcknowledgedEvent)
	{
		return false;
	}
	bStopRequested.Store(false);
	bPaused.Store(bStartPaused);
	PauseRequestSerial.Store(bStartPaused ? 1 : 0);
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
	(void)TryPause(MAX_uint32);
}

void FSimulationWorker::RequestPause()
{
	if (!bPaused.Load() && PauseAcknowledgedEvent)
	{
		PauseAcknowledgedEvent->Reset();
		++PauseRequestSerial;
	}
	bPaused.Store(true);
	if (WakeEvent)
	{
		WakeEvent->Trigger();
	}

}

bool FSimulationWorker::TryPause(const uint32 TimeoutMilliseconds)
{
	RequestPause();
	const uint64 RequestedSerial = PauseRequestSerial.Load();
	// Only a true return proves the canonical boundary. A timeout leaves the
	// request pending and never authorizes access to live simulation state.
	const double Deadline = FPlatformTime::Seconds() + double(TimeoutMilliseconds) / 1000.0;
	while (bRunning.Load() && PauseAcknowledgedEvent)
	{
		if (PauseAckSerial.Load() >= RequestedSerial && !bStopRequested.Load()) return true;
		const double Remaining = Deadline - FPlatformTime::Seconds();
		if (Remaining <= 0) return false;
		PauseAcknowledgedEvent->Wait(uint32(FMath::Max(1,
			FMath::CeilToInt(FMath::Min(Remaining * 1000.0, 100.0)))));
	}
	return false;
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
	uint32 ExitCode = 0;
	try
	{
		while (!bStopRequested.Load())
		{
			// Capture the request before servicing it. A newer pause cannot borrow
			// an acknowledgment from work performed for an earlier boundary.
			const uint64 Request = PauseRequestSerial.Load();
			const bool bPauseAtBoundary = bPaused.Load();
			const ESimulationBoundaryResult Boundary = ServiceBoundary
				? ServiceBoundary(bPauseAtBoundary) : ESimulationBoundaryResult::Ready;
			if (Boundary == ESimulationBoundaryResult::Failed)
			{
				ExitCode = 1;
				break;
			}
			if (bStopRequested.Load()) break;
			if (Boundary == ESimulationBoundaryResult::Waiting)
			{
				WakeEvent->Wait(1);
				continue;
			}
			if (bPaused.Load())
			{
				if (bPauseAtBoundary && PauseRequestSerial.Load() == Request)
				{
					PauseAckSerial.Store(Request);
					PauseAcknowledgedEvent->Trigger();
				}
				// Continue servicing commands while paused; no producer polling or
				// physical frame runs from this branch.
				WakeEvent->Wait(1);
				continue;
			}
			PauseAcknowledgedEvent->Reset();
			const ESimulationWorkerResult Result = Work();
			if (Result == ESimulationWorkerResult::Complete || Result == ESimulationWorkerResult::Failed)
			{
				ExitCode = Result == ESimulationWorkerResult::Failed ? 1 : 0;
				break;
			}
			if (Result == ESimulationWorkerResult::Idle)
			{
				WakeEvent->Wait(1);
				continue;
			}
			WaitBetweenFrames(WaitContext);
		}
	}
	catch (...) { ExitCode = 1; }
	// The callback retires and destroys thread-affine session owners before
	// join can let the GT release their storage, on every exit path.
	try { if (CloseOnWorker) CloseOnWorker(); }
	catch (...) { ExitCode = 1; }
	bStopRequested.Store(true);
	bRunning.Store(false);
	PauseAcknowledgedEvent->Trigger();
	return ExitCode;
}
