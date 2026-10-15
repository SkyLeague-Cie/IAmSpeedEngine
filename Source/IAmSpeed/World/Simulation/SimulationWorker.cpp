#include "SimulationWorker.h"
#include "OwnedFrameAudit.h"

#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "HAL/RunnableThread.h"

#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Testing
{
namespace
{
thread_local FSimulationWaitAuditObserver WaitAuditObserver = nullptr;
thread_local void* WaitAuditContext = nullptr;
}
bool SetSimulationWaitAuditObserverForCurrentThread(
    FSimulationWaitAuditObserver Observer, void* Context) noexcept
{
    if (!Observer || !Context || WaitAuditObserver) return false;
    WaitAuditObserver = Observer; WaitAuditContext = Context; return true;
}
bool ClearSimulationWaitAuditObserverForCurrentThread(
    FSimulationWaitAuditObserver Observer, void* Context) noexcept
{
    if (WaitAuditObserver != Observer || WaitAuditContext != Context) return false;
    WaitAuditObserver = nullptr; WaitAuditContext = nullptr; return true;
}
static void ObserveSimulationWait(ESimulationWaitAuditEvent Event,
    double Deadline, double Precision, double Remaining,
    uint32 RequestedMilliseconds = 0, int32 Outcome = -1) noexcept
{
    auto* const OwnedAtEntry = Speed::OwnedFrameAudit::ActiveForCurrentThread();
    if (!WaitAuditObserver && !OwnedAtEntry) return;
    FSimulationWaitAuditSample Sample;
    Sample.Event = Event;
    Sample.WallTicks = FPlatformTime::Cycles64();
    FMemory::Memcpy(&Sample.DeadlineBits, &Deadline, sizeof(Deadline));
    FMemory::Memcpy(&Sample.PrecisionBits, &Precision, sizeof(Precision));
    FMemory::Memcpy(&Sample.RemainingBits, &Remaining, sizeof(Remaining));
    Sample.RequestedMilliseconds = RequestedMilliseconds; Sample.Outcome = Outcome;
    if (WaitAuditObserver) WaitAuditObserver(WaitAuditContext, Sample);
    Speed::OwnedFrameAudit::RecordWaitSample(OwnedAtEntry, Sample);
}
}
#endif

FSimulationWorker::FSimulationWorker(
	FWork&& InWork, FWaitBetweenFrames&& InWaitBetweenFrames,
	FServiceBoundary InServiceBoundary, FCloseOnWorker InCloseOnWorker)
	: Work(MoveTemp(InWork))
	, WaitBetweenFrames(MoveTemp(InWaitBetweenFrames))
	, ServiceBoundary(MoveTemp(InServiceBoundary))
	, CloseOnWorker(MoveTemp(InCloseOnWorker))
	, WakeEvent(FPlatformProcess::GetSynchEventFromPool(false))
	, PauseAcknowledgedEvent(FPlatformProcess::GetSynchEventFromPool(true))
	, BoundarySuspendedEvent(FPlatformProcess::GetSynchEventFromPool(true))
{
}

ESimulationWaitResult FSimulationWorkerWaitContext::WaitUntil(
	const double DeadlineSeconds,
	const double PrecisionWindowSeconds) const
{
	const double ClampedPrecisionWindowSeconds = FMath::Clamp(
		PrecisionWindowSeconds, 0.0, 0.005);
#if WITH_DEV_AUTOMATION_TESTS
        Speed::Testing::ObserveSimulationWait(Speed::Testing::ESimulationWaitAuditEvent::Entry,
            DeadlineSeconds, ClampedPrecisionWindowSeconds, 0.0, 0, -1);
#endif
	while (!StopRequested.Load() && !Paused.Load())
	{
		const double RemainingSeconds = DeadlineSeconds - FPlatformTime::Seconds();
#if WITH_DEV_AUTOMATION_TESTS
        Speed::Testing::ObserveSimulationWait(Speed::Testing::ESimulationWaitAuditEvent::RemainingClockSample,
            DeadlineSeconds, ClampedPrecisionWindowSeconds, RemainingSeconds, 0, -1);
#endif
		if (RemainingSeconds <= 0.0)
		{
#if WITH_DEV_AUTOMATION_TESTS
        Speed::Testing::ObserveSimulationWait(Speed::Testing::ESimulationWaitAuditEvent::Return,
            DeadlineSeconds, ClampedPrecisionWindowSeconds, RemainingSeconds, 0, int32(ESimulationWaitResult::DeadlineReached));
#endif
			return ESimulationWaitResult::DeadlineReached;
		}
		const double CoarseWaitSeconds =
			RemainingSeconds - ClampedPrecisionWindowSeconds;
		const int32 CoarseWaitMilliseconds = FMath::FloorToInt(
			CoarseWaitSeconds * 1000.0);
		if (CoarseWaitMilliseconds > 0)
		{
#if WITH_DEV_AUTOMATION_TESTS
            const uint32 RequestedMilliseconds = static_cast<uint32>(FMath::Min(CoarseWaitMilliseconds, 100));
        Speed::Testing::ObserveSimulationWait(Speed::Testing::ESimulationWaitAuditEvent::EventWaitEntry,
            DeadlineSeconds, ClampedPrecisionWindowSeconds, RemainingSeconds, RequestedMilliseconds, -1);
            const bool SignalObserved = WakeEvent.Wait(RequestedMilliseconds);
        Speed::Testing::ObserveSimulationWait(Speed::Testing::ESimulationWaitAuditEvent::EventWaitExit,
            DeadlineSeconds, ClampedPrecisionWindowSeconds, RemainingSeconds, RequestedMilliseconds, int32(SignalObserved));
#else
			WakeEvent.Wait(static_cast<uint32>(FMath::Min(
				CoarseWaitMilliseconds, 100)));
#endif
		}
		else if (ClampedPrecisionWindowSeconds > 0.0)
		{
			// Yielding keeps the short precision tail interruptible and avoids the
			// systematic oversleep caused by rounding a sub-millisecond tail up.
#if WITH_DEV_AUTOMATION_TESTS
        Speed::Testing::ObserveSimulationWait(Speed::Testing::ESimulationWaitAuditEvent::YieldEntry,
            DeadlineSeconds, ClampedPrecisionWindowSeconds, RemainingSeconds, 0, -1);
#endif
			// Processor pause is confined to the existing precision window.
			// The sub-millisecond coarse remainder still yields the thread.
			if (RemainingSeconds <= ClampedPrecisionWindowSeconds)
			{
				FPlatformProcess::Yield();
			}
			else
			{
				FPlatformProcess::YieldThread();
			}
#if WITH_DEV_AUTOMATION_TESTS
        Speed::Testing::ObserveSimulationWait(Speed::Testing::ESimulationWaitAuditEvent::YieldExit,
            DeadlineSeconds, ClampedPrecisionWindowSeconds, RemainingSeconds, 0, -1);
#endif

		}
		else
		{
#if WITH_DEV_AUTOMATION_TESTS
        Speed::Testing::ObserveSimulationWait(Speed::Testing::ESimulationWaitAuditEvent::EventWaitEntry,
            DeadlineSeconds, ClampedPrecisionWindowSeconds, RemainingSeconds, 1, -1);
            const bool SignalObserved = WakeEvent.Wait(1);
        Speed::Testing::ObserveSimulationWait(Speed::Testing::ESimulationWaitAuditEvent::EventWaitExit,
            DeadlineSeconds, ClampedPrecisionWindowSeconds, RemainingSeconds, 1, int32(SignalObserved));
#else
			WakeEvent.Wait(1);
#endif
		}
	}
#if WITH_DEV_AUTOMATION_TESTS
        Speed::Testing::ObserveSimulationWait(Speed::Testing::ESimulationWaitAuditEvent::Return,
            DeadlineSeconds, ClampedPrecisionWindowSeconds, 0.0, 0, int32(ESimulationWaitResult::Interrupted));
#endif
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
	if (BoundarySuspendedEvent)
	{
		FPlatformProcess::ReturnSynchEventToPool(BoundarySuspendedEvent);
		BoundarySuspendedEvent = nullptr;
	}
}

bool FSimulationWorker::Start(bool bStartPaused)
{
	if (Thread || !Work || !WaitBetweenFrames || !WakeEvent ||
		!PauseAcknowledgedEvent || !BoundarySuspendedEvent)
	{
		return false;
	}
	bStopRequested.Store(false);
	bPaused.Store(bStartPaused);
	PauseRequestSerial.Store(bStartPaused ? 1 : 0);
	bBoundaryServiceSuspended.Store(false);
	BoundarySuspendRequestSerial.Store(0);
	BoundarySuspendAckSerial.Store(0);
	PauseAcknowledgedEvent->Reset();
	BoundarySuspendedEvent->Reset();
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

bool FSimulationWorker::TrySuspendBoundaryService(const uint32 TimeoutMilliseconds)
{
	// A paused physical lane still services commands. This second barrier parks
	// that callback as well. It can acknowledge even when a lifecycle command is
	// waiting for baseline, because the entire callback has returned and no
	// physical work can run before the next loop's suspension check.
	if (!bRunning.Load() || bStopRequested.Load()) return false;
	RequestPause();
	if (!bBoundaryServiceSuspended.Load())
	{
		BoundarySuspendedEvent->Reset();
		++BoundarySuspendRequestSerial;
		bBoundaryServiceSuspended.Store(true);
		WakeEvent->Trigger();
	}
	const uint64 RequestedSerial = BoundarySuspendRequestSerial.Load();
	const double Deadline = FPlatformTime::Seconds() + double(TimeoutMilliseconds) / 1000.0;
	while (bRunning.Load() && !bStopRequested.Load())
	{
		if (BoundarySuspendAckSerial.Load() >= RequestedSerial) return true;
		const double Remaining = Deadline - FPlatformTime::Seconds();
		if (Remaining <= 0) return false;
		BoundarySuspendedEvent->Wait(uint32(FMath::Max(1,
			FMath::CeilToInt(FMath::Min(Remaining * 1000.0, 100.0)))));
	}
	return false;
}

void FSimulationWorker::ResumeBoundaryService()
{
	bBoundaryServiceSuspended.Store(false);
	if (WakeEvent) WakeEvent->Trigger();
}

void FSimulationWorker::Resume()
{
	// Structural access must be explicitly released before physical work resumes.
	if (bBoundaryServiceSuspended.Load()) return;
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
			if (bBoundaryServiceSuspended.Load())
			{
				const uint64 Request = BoundarySuspendRequestSerial.Load();
				BoundarySuspendAckSerial.Store(Request);
				BoundarySuspendedEvent->Trigger();
				// Exclusive suspension is stronger than an ordinary pause: no
				// callback or physical work can run. A nested pause request may
				// therefore acknowledge the same parked boundary.
				if (bPaused.Load())
				{
					PauseAckSerial.Store(PauseRequestSerial.Load());
					PauseAcknowledgedEvent->Trigger();
				}
#if WITH_DEV_AUTOMATION_TESTS
				Speed::OwnedFrameAudit::ParkWait(*WakeEvent, 1);
#else
				WakeEvent->Wait(1);
#endif
				continue;
			}
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
#if WITH_DEV_AUTOMATION_TESTS
				Speed::OwnedFrameAudit::ParkWait(*WakeEvent, 1);
#else
				WakeEvent->Wait(1);
#endif
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
#if WITH_DEV_AUTOMATION_TESTS
				Speed::OwnedFrameAudit::ParkWait(*WakeEvent, 1);
#else
				WakeEvent->Wait(1);
#endif
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
#if WITH_DEV_AUTOMATION_TESTS
				Speed::OwnedFrameAudit::ParkWait(*WakeEvent, 1);
#else
				WakeEvent->Wait(1);
#endif
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
	BoundarySuspendedEvent->Trigger();
	return ExitCode;
}
