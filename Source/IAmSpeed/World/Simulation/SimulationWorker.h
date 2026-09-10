#pragma once

#include "CoreMinimal.h"
#include "HAL/Runnable.h"

enum class ESimulationWorkerResult : uint8
{
	Advanced,
	Idle,
	Complete,
	Failed,
};

enum class ESimulationWaitResult : uint8
{
	DeadlineReached,
	Interrupted,
};

/** Restricted wait service exposed to a simulation pacing policy. */
class IAMSPEED_API FSimulationWorkerWaitContext final
{
public:
	/**
	 * Waits until an absolute monotonic deadline or a pause/stop request.
	 * PrecisionWindowSeconds reserves the final part of the wait for yielding
	 * instead of rounding the whole duration up to an OS-event millisecond.
	 */
	ESimulationWaitResult WaitUntil(
		double DeadlineSeconds,
		double PrecisionWindowSeconds = 0.0) const;

private:
	friend class FSimulationWorker;
	FSimulationWorkerWaitContext(
		FEvent& InWakeEvent,
		const TAtomic<bool>& InStopRequested,
		const TAtomic<bool>& InPaused)
		: WakeEvent(InWakeEvent)
		, StopRequested(InStopRequested)
		, Paused(InPaused)
	{
	}

	FEvent& WakeEvent;
	const TAtomic<bool>& StopRequested;
	const TAtomic<bool>& Paused;
};

/**
 * Single owned execution lane for canonical simulation frames. The worker
 * delegates the between-frame pacing policy to the simulation subclass.
 */
class IAMSPEED_API FSimulationWorker final : private FRunnable
{
public:
	using FWork = TFunction<ESimulationWorkerResult()>;
	using FWaitBetweenFrames = TFunction<void(FSimulationWorkerWaitContext&)>;

	FSimulationWorker(FWork&& InWork, FWaitBetweenFrames&& InWaitBetweenFrames);
	~FSimulationWorker();

	/** Starts the one worker thread; subsequent calls are rejected. */
	bool Start();
	/** Prevents new work and waits until the worker reaches a frame boundary. */
	void Pause();
	/** Bounded boundary acknowledgment. Failure leaves the pause requested and
	 * must not authorize mutation or automatic resume of a partially running frame. */
	bool TryPause(uint32 TimeoutMilliseconds);
	/** Resumes work and resets the real-time deadline to now. */
	void Resume();
	/** Requests termination and blocks until the owned thread has joined. */
	void StopAndJoin();
	bool IsRunning() const { return bRunning.Load(); }
	bool IsPaused() const { return bPaused.Load(); }

private:
	uint32 Run() override;
	void Stop() override;

	FWork Work;
	FWaitBetweenFrames WaitBetweenFrames;
	TAtomic<bool> bStopRequested = false;
	TAtomic<bool> bPaused = false;
	TAtomic<bool> bRunning = false;
	TAtomic<uint64> PauseRequestSerial = 0;
	TAtomic<uint64> PauseAckSerial = 0;
	FEvent* WakeEvent = nullptr;
	FEvent* PauseAcknowledgedEvent = nullptr;
	FRunnableThread* Thread = nullptr;
};
