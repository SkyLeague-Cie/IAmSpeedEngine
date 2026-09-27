#pragma once

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

namespace Speed::Input::V2
{
enum class EAcquisitionPumpResult { Installed, NoChange, Neutralized, Rejected, Closed };
class IInputAcquisition
{
public:
	virtual ~IInputAcquisition() = default;
	virtual EAcquisitionPumpResult Pump() = 0;
	// Must invalidate consumer-visible cached data before attempting platform
	// teardown. False may retain callback resources for retry, never live input.
	virtual bool Close() = 0;
};

// Platform-independent IAmSpeed acquisition lifetime. It has no access to the
// simulation worker, physical frame counter, controller, or game callbacks.
// Cadence is explicit host configuration; no Unreal delta-time enters here.
class FInputAcquisitionWorker final
{
public:
	explicit FInputAcquisitionWorker(std::shared_ptr<IInputAcquisition> InSource) : Source(std::move(InSource)) {}
	~FInputAcquisitionWorker() { if (!Stop()) std::terminate(); }
	FInputAcquisitionWorker(const FInputAcquisitionWorker&) = delete;
	FInputAcquisitionWorker& operator=(const FInputAcquisitionWorker&) = delete;
	bool Start(std::chrono::microseconds Cadence)
	{
		std::lock_guard<std::mutex> Lifecycle(LifecycleGate);
		std::lock_guard<std::mutex> Lock(Gate);
		if (!Source || Started || Stopping || Cadence.count() <= 0 || Cadence > std::chrono::seconds(1)) return false;
		Period = Cadence; Started = true;
		try { Thread = std::thread([this] { Run(); }); }
		catch (...) { Started = false; return false; }
		return true;
	}
	// Startup gate only. Host pause/replacement also needs physical quiescence
	// and the journal's explicit fresh-resume acknowledgment.
	bool WaitForFirstPublication(std::chrono::milliseconds Timeout)
	{
		std::unique_lock<std::mutex> Lock(Gate);
		if (!Started || Timeout.count() < 0) return false;
		Changed.wait_for(Lock, Timeout, [&] { return Ready || Finished || Stopping; });
		return Ready && !Stopping && !Finished;
	}
	std::optional<EAcquisitionPumpResult> LastResult() const
	{ std::lock_guard<std::mutex> Lock(Gate); return Result; }
	bool HasFinished() const
	{ std::lock_guard<std::mutex> Lock(Gate); return Finished; }
	bool IsStopRequested() const
	{ std::lock_guard<std::mutex> Lock(Gate); return Stopping; }
	bool Stop()
	{
		{
			std::lock_guard<std::mutex> Lock(Gate);
			// Reject BEFORE LifecycleGate: another Stop may hold it while joining
			// this owner. Reading our protected ID never races std::thread::join.
			if (!Finished && WorkerId == std::this_thread::get_id()) return false;
		}
		std::lock_guard<std::mutex> Lifecycle(LifecycleGate);
		{
			std::lock_guard<std::mutex> Lock(Gate);
			Stopping = true; Changed.notify_all();
		}
		if (Thread.joinable()) Thread.join(); // Full acquisition lifetime fence.
		if (Source && !SourceClosed)
		{
			try { SourceClosed = Source->Close(); }
			catch (...) { return false; }
		}
		return !Source || SourceClosed;
	}
private:
	void Run() noexcept
	{
		{ std::lock_guard<std::mutex> Lock(Gate); WorkerId = std::this_thread::get_id(); }
		auto Deadline = std::chrono::steady_clock::now();
		for (;;)
		{
			{
				std::lock_guard<std::mutex> Lock(Gate);
				if (Stopping) break;
			}
			EAcquisitionPumpResult Current = EAcquisitionPumpResult::Rejected;
			try { Current = Source->Pump(); } catch (...) {}
			std::unique_lock<std::mutex> Lock(Gate);
			Result = Current;
			if (Current == EAcquisitionPumpResult::Installed || Current == EAcquisitionPumpResult::Neutralized) Ready = true;
			Changed.notify_all();
			if (Current == EAcquisitionPumpResult::Closed || Current == EAcquisitionPumpResult::Rejected) break;
			Deadline += Period;
			const auto Now = std::chrono::steady_clock::now();
			// No catch-up burst after a slow OS call. SDK history handles readings
			// between acquisitions; its overflow is an explicit source failure.
			if (Deadline <= Now) Deadline = Now + Period;
			if (Changed.wait_until(Lock, Deadline, [&] { return Stopping; })) break;
		}
		bool ClosedSource = false;
		try { ClosedSource = Source->Close(); } catch (...) {}
		std::lock_guard<std::mutex> Lock(Gate);
		SourceClosed = ClosedSource; Finished = true; Changed.notify_all();
	}
	const std::shared_ptr<IInputAcquisition> Source;
	mutable std::mutex Gate;
	std::mutex LifecycleGate;
	std::condition_variable Changed;
	std::thread Thread;
	std::thread::id WorkerId;
	std::chrono::microseconds Period{0};
	std::optional<EAcquisitionPumpResult> Result;
	bool Started = false, Stopping = false, Finished = false, Ready = false, SourceClosed = false;
};
}
