#pragma once

#include "GameInputDiscovery.h"
#include "../DeviceActivityPolicy.h"

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
	struct FPollObservation
	{
		FFrameNumber Frame = 0;
		std::optional<FDeviceSelection> AcquisitionTicket;
		FReadObservations Readings;
		std::uint64_t SubmitAccepted = 0, SubmitRejected = 0;
	};
	// Diagnostic-only opt-in before first polling. A ticket labels acquisition,
	// not the final frame generation: hotplug can invalidate it before latching.
	bool EnableObservations()
	{
		std::lock_guard<std::mutex> Lock(Gate);
		if (Stopping || PollAttempted) return false;
		Observations = std::make_unique<FPollObservation>(); return true;
	}
	std::optional<FPollObservation> ReadObservation() const
	{
		std::lock_guard<std::mutex> Lock(Gate);
		return Observations ? std::optional<FPollObservation>(*Observations) : std::nullopt;
	}
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
	static std::unique_ptr<FGameInputSelectedSource> CreateAutomatic(FApi* Api, std::uint64_t ProducerId,
		const std::array<bool, ActionCount>& Digital, FGameInputMapper Keyboard, FGameInputMapper Gamepad,
		FActivityConfig Config)
	{
		if (!FDeviceActivityPolicy::ValidConfig(Config)) return nullptr;
		auto Result = Create(Api, ProducerId, Digital, std::move(Keyboard), std::move(Gamepad));
		if (Result) Result->Activity = std::make_unique<FDeviceActivityPolicy>(Config);
		return Result;
	}
	bool RequestLock(std::optional<FDeviceId> Id)
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Gate);
		if (Stopping || !Activity || FAILED(Discovery->GetLastError())) return false;
		PendingLock = Id; HasPendingLock = true; return true;
	}
	bool Shutdown()
	{
		std::lock_guard<std::mutex> Lock(Gate);
		Stopping = true; Cursor.Reset(); Active.reset(); ActivityCursors.clear();
		return Discovery->Shutdown(); // False retains discovery and its callback context.
	}
	// Latest explicit request applies only on a forward Produce/Skip boundary.
	bool RequestSelection(FRequest Request)
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Gate);
		if (Stopping || Activity || FAILED(Discovery->GetLastError())) return false;
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
		if (Activity)
		{
			Activity->SetPaused(Paused);
			for (auto& Item : ActivityCursors) Item.second.Cursor.Reset(); // Retain read-side disconnect quarantine.
		}
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
		PollLocked(Frame);
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
		PollLocked(Frame);
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
	bool PollActivityLocked(FFrameNumber Frame)
	{
		using namespace GameInput::v3;
		if (HasPendingLock) { Activity->SetLock(PendingLock); HasPendingLock = false; }
		const auto Devices = Discovery->Snapshot();
		if (!Activity->Sync(Devices)) { Discovery->FailAcquisition(E_OUTOFMEMORY); return false; }
		std::map<FDeviceId, FActivityCursor> Retained;
		if (!bPaused) for (const auto& D : Devices)
		{
			const auto Kind = Activity->Kind(D.Id);
			if (!Kind) continue;
			const auto Lease = Discovery->AcquireDevice(D.Id, *Kind);
			if (!Lease || Lease->Ticket.Device.Revision != D.Revision) continue;
			auto Old = ActivityCursors.find(D.Id);
			FActivityCursor Reading;
			if (Old != ActivityCursors.end() && Old->second.Revision == D.Revision) Reading = std::move(Old->second);
			Reading.Revision = D.Revision;
			if (!Reading.Disconnected)
			{
				const auto Result = Reading.Cursor.Poll(*Api.Get(), Lease->Device.Get(),
					*Kind == EDeviceKind::Keyboard ? GameInputKindKeyboard : GameInputKindGamepad,
					[&](const FDeviceState& Raw, FActionValues&)
					{
						FActivityState Sample; Sample.Axes = Raw.Axes;
						if (*Kind == EDeviceKind::Keyboard) Sample.Buttons = Raw.VirtualKeys;
						else for (std::size_t B = 0; B < 32; ++B) Sample.Buttons[B] = (Raw.GamepadButtons & (std::uint32_t(1) << B)) != 0;
						return Activity->Observe(D.Id, D.Revision, Raw.TimestampMicroseconds, Sample);
					}, [](const FActionValues&) { return true; });
				if (Result.Status == EReadBatchStatus::Resynchronize || Result.Status == EReadBatchStatus::Error)
				{
					Activity->ResetDevice(D.Id); Reading.Cursor.Reset();
					if (Result.Error == GAMEINPUT_E_DEVICE_DISCONNECTED) Reading.Disconnected = true;
					else if (Result.Status == EReadBatchStatus::Error && Result.Error != GAMEINPUT_E_REFERENCE_READING_TOO_OLD)
					{ Discovery->FailAcquisition(Result.Error); return false; }
				}
			}
			Retained.emplace(D.Id, std::move(Reading));
		}
		if (!bPaused) ActivityCursors = std::move(Retained);
		// Remove activity recorded from any lifecycle revision invalidated mid-poll.
		auto Latest = Discovery->Snapshot();
		for (auto& D : Latest)
		{
			const auto It = ActivityCursors.find(D.Id);
			if (It != ActivityCursors.end() && It->second.Revision == D.Revision && It->second.Disconnected)
				D.Connected = false;
		}
		if (!Activity->Sync(Latest)) { Discovery->FailAcquisition(E_FAIL); return false; }
		const auto Choice = Activity->Decide(Frame);
		if (Activity->IsFailed()) { Discovery->FailAcquisition(E_FAIL); return false; }
		Pending.reset();
		if (Choice) Pending = std::make_pair(Choice->Id, Choice->Kind);
		HasPending = true; return true;
	}
	void PollLocked(FFrameNumber Frame)
	{
		using namespace GameInput::v3;
		PollAttempted = true;
		if (Observations) { *Observations = {}; Observations->Frame = Frame; }
		if (FAILED(Discovery->GetLastError())) { Status = EPollStatus::Failed; return; }
		if (Activity && !PollActivityLocked(Frame)) { Status = EPollStatus::Failed; return; }
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
		if (Observations) Observations->AcquisitionTicket = Lease->Ticket;
		const auto& Mapper = Lease->Ticket.Kind == EDeviceKind::Keyboard ? Keyboard : Gamepad;
		const auto Result = Cursor.Poll(*Api.Get(), Lease->Device.Get(), Kind, Mapper,
			[&](const FActionValues& Values)
			{
				if (Sequence == std::numeric_limits<std::uint64_t>::max() || !Discovery->Submit(*Lease, Sequence + 1, Values)) {
					if (Observations) ++Observations->SubmitRejected;
					return false;
				}
				if (Observations) ++Observations->SubmitAccepted;
				++Sequence; return true;
			}, Observations ? &Observations->Readings : nullptr);
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
	struct FActivityCursor { FGameInputReadCursor Cursor; std::uint64_t Revision = 0; bool Disconnected = false; };
	std::unique_ptr<FDeviceActivityPolicy> Activity;
	std::map<FDeviceId, FActivityCursor> ActivityCursors;
	std::optional<FDeviceId> PendingLock;
	bool HasPendingLock = false;
	std::unique_ptr<FGameInputDiscovery> Discovery;
	const FGameInputMapper Keyboard, Gamepad; // Pure/bounded/nonthrowing; no reentrant source calls.
	mutable std::mutex Gate;
	FGameInputReadCursor Cursor;
	std::unique_ptr<FPollObservation> Observations;
	bool PollAttempted = false;
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
