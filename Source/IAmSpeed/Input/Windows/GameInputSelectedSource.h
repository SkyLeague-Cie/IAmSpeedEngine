#pragma once

#include "GameInputDiscovery.h"

namespace Speed::Input::Windows
{
/** Explicit-selection composition. One discovery-owned session/journal, one raw
 * cursor. Automatic last-active arbitration is deliberately a separate gate.
 */
class FGameInputSelectedSource final : public IInputProducer
{
	using FApi = GameInput::v3::IGameInput;
	using FLease = FGameInputDiscovery::FLease;
	using FRequest = std::optional<std::pair<FDeviceId, EDeviceKind>>;
public:
	static std::unique_ptr<FGameInputSelectedSource> Create(FApi* Api, std::uint64_t ProducerId,
		const std::array<bool, ActionCount>& Digital, FGameInputMapper Keyboard, FGameInputMapper Gamepad)
	{
		if (!Keyboard || !Gamepad) return nullptr;
		auto Discovery = FGameInputDiscovery::Create(Api, ProducerId, Digital);
		if (!Discovery) return nullptr;
		return std::unique_ptr<FGameInputSelectedSource>(new FGameInputSelectedSource(
			Api, std::move(Discovery), std::move(Keyboard), std::move(Gamepad)));
	}
	~FGameInputSelectedSource() override { if (!Shutdown()) std::terminate(); }
	bool Shutdown()
	{
		std::lock_guard<std::mutex> Lock(Gate);
		Stopping = true; Cursor.Reset(); Active.reset();
		return Discovery->Shutdown(); // False retains discovery and its callback context.
	}
	// Latest explicit request applies only on a forward Produce/Skip boundary.
	bool RequestSelection(FRequest Request)
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Gate);
		if (Stopping || FAILED(Discovery->GetLastError())) return false;
		if (Request && Request->second != EDeviceKind::Keyboard && Request->second != EDeviceKind::Gamepad) return false;
		Pending = Request; HasPending = true;
		return true;
	}
	bool SetPaused(bool Paused)
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Gate);
		if (Stopping || !Discovery->SetPaused(Paused)) return false;
		bPaused = Paused; Cursor.Reset(); Active.reset(); Sequence = 0;
		return true;
	}
	std::vector<FDiscoveredDevice> Snapshot() const { return Discovery->Snapshot(); }
	std::optional<FInputFrame> Produce(FFrameNumber Frame) override
	{
		if (FPresentationInputScope::IsActive()) return std::nullopt;
		std::lock_guard<std::mutex> Lock(Gate);
		if (Stopping) return std::nullopt;
		if (LastFrame && Frame <= *LastFrame) return Discovery->Produce(Frame);
		if (!IsNext(Frame)) return std::nullopt;
		PollLocked();
		auto Result = Discovery->Produce(Frame); // Final latch serializes with hotplug.
		if (Result) LastFrame = Frame;
		return Result;
	}
	bool Skip(FFrameNumber Frame) override
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Gate);
		if (Stopping) return false;
		if (LastFrame && Frame <= *LastFrame) return Discovery->Skip(Frame);
		if (!IsNext(Frame)) return false;
		PollLocked();
		if (!Discovery->Skip(Frame)) return false;
		LastFrame = Frame;
		return true;
	}
	EPollStatus GetLastPollStatus() const
	{
		std::lock_guard<std::mutex> Lock(Gate);
		return FAILED(Discovery->GetLastError()) ? EPollStatus::Failed : Status;
	}
	HRESULT GetLastError() const
	{
		std::lock_guard<std::mutex> Lock(Gate);
		const auto Fatal = Discovery->GetLastError();
		return FAILED(Fatal) ? Fatal : Error;
	}
private:
	FGameInputSelectedSource(FApi* InApi, std::unique_ptr<FGameInputDiscovery> InDiscovery,
		FGameInputMapper InKeyboard, FGameInputMapper InGamepad)
		: Api(InApi), Discovery(std::move(InDiscovery)), Keyboard(std::move(InKeyboard)), Gamepad(std::move(InGamepad)) {}
	bool IsNext(FFrameNumber Frame) const
	{
		return !LastFrame || (*LastFrame != std::numeric_limits<FFrameNumber>::max() && Frame == *LastFrame + 1);
	}
	static bool Same(const FLease& A, const FLease& B)
	{
		return A.Ticket.Generation == B.Ticket.Generation && A.Device.Get() == B.Device.Get()
			&& A.Ticket.Device.Id == B.Ticket.Device.Id && A.Ticket.Device.Revision == B.Ticket.Device.Revision
			&& A.Ticket.Kind == B.Ticket.Kind;
	}
	void ResetReadingLocked(const FLease& Lease)
	{
		// A changed ticket already means the discovery callback purged old state.
		Discovery->Resynchronize(Lease);
		Cursor.Reset(); Active.reset(); Sequence = 0;
		Status = EPollStatus::Resynchronized;
	}
	void PollLocked()
	{
		using namespace GameInput::v3;
		if (FAILED(Discovery->GetLastError())) { Status = EPollStatus::Failed; return; }
		if (HasPending)
		{
			if (!Discovery->Select(Pending)) { Discovery->FailAcquisition(E_FAIL); Status = EPollStatus::Failed; return; }
			HasPending = false;
		}
		if (bPaused) { Status = EPollStatus::Paused; return; }
		const auto Lease = Discovery->AcquireSelected();
		if (!Lease) { Cursor.Reset(); Active.reset(); Status = EPollStatus::Disconnected; return; }
		if (Disconnected && Disconnected->first == Lease->Ticket.Device.Id && Disconnected->second == Lease->Ticket.Device.Revision)
		{
			Status = EPollStatus::Disconnected; return; // SDK read-side removal waits for a lifecycle revision.
		}
		Disconnected.reset();
		if (!Active || !Same(*Active, *Lease)) { Cursor.Reset(); Sequence = 0; Active = Lease; }
		const auto Kind = Lease->Ticket.Kind == EDeviceKind::Keyboard ? GameInputKindKeyboard : GameInputKindGamepad;
		const auto& Mapper = Lease->Ticket.Kind == EDeviceKind::Keyboard ? Keyboard : Gamepad;
		const auto Result = Cursor.Poll(*Api.Get(), Lease->Device.Get(), Kind, Mapper,
			[&](const FActionValues& Values)
			{
				if (Sequence == std::numeric_limits<std::uint64_t>::max() || !Discovery->Submit(*Lease, Sequence + 1, Values)) return false;
				++Sequence; return true;
			});
		Error = Result.Error;
		if (Result.Status == EReadBatchStatus::Error)
		{
			if (Error == GAMEINPUT_E_REFERENCE_READING_TOO_OLD) ResetReadingLocked(*Lease);
			else if (Error == GAMEINPUT_E_DEVICE_DISCONNECTED)
			{
				ResetReadingLocked(*Lease);
				Disconnected = std::make_pair(Lease->Ticket.Device.Id, Lease->Ticket.Device.Revision);
				Status = EPollStatus::Disconnected;
			}
			else { Discovery->FailAcquisition(Error); Cursor.Reset(); Status = EPollStatus::Failed; }
			return;
		}
		if (Result.Status == EReadBatchStatus::Resynchronize) { ResetReadingLocked(*Lease); return; }
		Status = Result.Status == EReadBatchStatus::Updated ? EPollStatus::Updated : EPollStatus::NoChange;
	}
	Microsoft::WRL::ComPtr<FApi> Api;
	std::unique_ptr<FGameInputDiscovery> Discovery;
	const FGameInputMapper Keyboard, Gamepad; // Pure/bounded/nonthrowing; no reentrant source calls.
	mutable std::mutex Gate;
	FGameInputReadCursor Cursor;
	std::optional<FLease> Active;
	std::optional<std::pair<FDeviceId, std::uint64_t>> Disconnected;
	std::optional<FFrameNumber> LastFrame;
	std::uint64_t Sequence = 0;
	FRequest Pending;
	bool HasPending = false, bPaused = false, Stopping = false;
	EPollStatus Status = EPollStatus::NoChange;
	HRESULT Error = S_OK;
};
}
