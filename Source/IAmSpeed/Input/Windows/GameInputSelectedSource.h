#pragma once

#include "GameInputDiscovery.h"
#include "GameInputCallbackReadings.h"
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
	enum class ERawPollReject : std::uint8_t
	{
		None, AdmissionGate, Activity, Selection,
		ReadBatch, CommitGate, Sink
	};
	struct FSelectedRawBatch
	{
		std::optional<FDeviceSelection> Ticket;
		FRawDeviceReadBatch Readings;
		EPollStatus Status = EPollStatus::Disconnected;
		std::uint64_t AcquisitionTick = 0;
	};
	struct FSelectedRawDeviceDiagnostic
	{
		FDeviceId Id{};
		EDeviceKind Kind = EDeviceKind::Keyboard;
		std::uint16_t VendorId = 0, ProductId = 0;
	};
	struct FRawReadGapDiagnostic
	{
		FDeviceId SelectedId{};
		EDeviceKind Kind = EDeviceKind::Keyboard;
		std::uint64_t Revision = 0, SelectionGeneration = 0;
		std::uint64_t AcquisitionTick = 0, LastSuccessfulReadTick = 0;
		FTooOldReadDiagnostic Reading;
	};
	std::optional<FRawReadGapDiagnostic> TakeRawReadGapDiagnostic()
	{
		std::lock_guard<std::mutex> Lock(Gate);
		auto Result = LastRawReadGap;
		LastRawReadGap.reset(); return Result;
	}
	// Read-only provenance for a supervised hardware diagnostic. This is not a
	// second input path and cannot affect selection or action delivery.
	std::optional<FSelectedRawDeviceDiagnostic> ReadSelectedRawDeviceDiagnostic() const
	{
		if (FPresentationInputScope::IsActive()) return {};
		std::lock_guard<std::mutex> Lock(Gate);
		if (!RawMode || Stopping || !Active) return {};
		const GameInput::v3::GameInputDeviceInfo* Info = nullptr;
		if (FAILED(Active->Device->GetDeviceInfo(&Info)) || !Info) return {};
		return FSelectedRawDeviceDiagnostic{Active->Ticket.Device.Id, Active->Ticket.Kind,
			Info->vendorId, Info->productId};
	}
	// Raw and legacy action ownership cannot be mixed on one cursor/session.
	static std::unique_ptr<FGameInputSelectedSource> CreateRaw(FApi* Api, std::uint64_t ProducerId,
		std::optional<FActivityConfig> Config = std::nullopt, bool CaptureCallbacks = false)
	{
		if (Config && !FDeviceActivityPolicy::ValidConfig(*Config)) return {};
		auto Discovery = FGameInputDiscovery::Create(Api, ProducerId, {});
		if (!Discovery) return {};
		auto Result = std::unique_ptr<FGameInputSelectedSource>(new FGameInputSelectedSource(Api,
			std::move(Discovery), {}, {}));
		Result->RawMode = true;
		if (CaptureCallbacks) Result->CallbackCursor = std::make_unique<FGameInputCallbackReadCursor>();
		if (Config) Result->Activity = std::make_unique<FDeviceActivityPolicy>(*Config);
		return Result;
	}
	// AcquisitionTick is the acquisition owner's clock, never a physics frame.
	// Sink installs a bounded value batch under the hotplug fence, without
	// reentering this object. False means nothing was installed: discard all data.
	template<class TSink> bool PollRaw(std::uint64_t AcquisitionTick, TSink&& Sink)
	{
		static_assert(std::is_nothrow_invocable_r_v<bool, TSink, const FSelectedRawBatch&>, "raw sink must return acceptance without throwing");
		// Fail before entering Gate: a forbidden presentation callback must never
		// wait for or reenter an in-progress acquisition. This path is not logged.
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Gate);
		LastRawPollReject = ERawPollReject::None;
		LastRawReadGap.reset();
		if (!RawMode || Stopping || bPaused || !AcquisitionTick
			|| AcquisitionTick <= LastAcquisitionTick || FAILED(Discovery->GetLastError()))
		{ LastRawPollReject = ERawPollReject::AdmissionGate; return false; }
		LastAcquisitionTick = AcquisitionTick; PollAttempted = true;
		if (Activity && !PollActivityLocked(AcquisitionTick))
		{ LastRawPollReject = ERawPollReject::Activity; return false; }
		if (HasPending)
		{
			if (!Discovery->Select(Pending))
			{ Discovery->FailAcquisition(E_FAIL); Status = EPollStatus::Failed; LastRawPollReject = ERawPollReject::Selection; return false; }
			HasPending = false;
		}
		const auto Lease = Discovery->AcquireSelected();
		FSelectedRawBatch Batch; Batch.AcquisitionTick = AcquisitionTick;
		if (!Lease) { if (!ResetCursorsLocked()) return false; Active.reset(); LastSuccessfulRawReadTick = 0; }
		else
		{
			Batch.Ticket = Lease->Ticket;
			if (!Disconnected || Disconnected->first != Lease->Ticket.Device.Id
				|| Disconnected->second != Lease->Ticket.Device.Revision)
			{
				Disconnected.reset();
				if (!Active || !Same(*Active, *Lease))
				{ if (!ResetCursorsLocked()) return false; Active = Lease; LastSuccessfulRawReadTick = 0; }
				FTooOldReadDiagnostic TooOld;
				const auto Kind = Lease->Ticket.Kind == EDeviceKind::Keyboard
					? GameInput::v3::GameInputKindKeyboard : GameInput::v3::GameInputKindGamepad;
				Batch.Readings = CallbackCursor
					? CallbackCursor->PollRaw(*Api.Get(), Lease->Device.Get(), Kind)
					: Cursor.PollRaw(*Api.Get(), Lease->Device.Get(), Kind, &TooOld);
				Error = Batch.Readings.Result.Error;
				if (Batch.Readings.Result.Status == EReadBatchStatus::Error
					|| Batch.Readings.Result.Status == EReadBatchStatus::Resynchronize)
				{
					if (TooOld.Observed)
						LastRawReadGap = FRawReadGapDiagnostic{Lease->Ticket.Device.Id,
							Lease->Ticket.Kind, Lease->Ticket.Device.Revision,
							Lease->Ticket.Generation, AcquisitionTick, LastSuccessfulRawReadTick, TooOld};
					if (FAILED(Error) && Error != GameInput::v3::GAMEINPUT_E_REFERENCE_READING_TOO_OLD && Error != GameInput::v3::GAMEINPUT_E_DEVICE_DISCONNECTED)
					{ Discovery->FailAcquisition(Error); Status = EPollStatus::Failed; LastRawPollReject = ERawPollReject::ReadBatch; return false; }
					ResetReadingLocked(*Lease);
					if (Error == GameInput::v3::GAMEINPUT_E_DEVICE_DISCONNECTED)
						Disconnected = std::make_pair(Lease->Ticket.Device.Id, Lease->Ticket.Device.Revision);
					// Resynchronization changes generation. Never relabel an old batch.
					Status = Error == GameInput::v3::GAMEINPUT_E_DEVICE_DISCONNECTED ? EPollStatus::Disconnected : EPollStatus::Resynchronized;
					LastRawPollReject = ERawPollReject::ReadBatch;
					return false;
				}
				Batch.Status = Batch.Readings.Result.Status == EReadBatchStatus::Updated ? EPollStatus::Updated : EPollStatus::NoChange;
			}
		}
		bool SinkCalled = false;
		const bool Accepted = Discovery->CommitRaw(Lease, [&]() noexcept
		{
			SinkCalled = true;
			return Sink(Batch);
		});
		if (!Accepted)
		{
			LastRawPollReject = SinkCalled ? ERawPollReject::Sink : ERawPollReject::CommitGate;
			if (Lease) ResetReadingLocked(*Lease);
			else { if (!ResetCursorsLocked()) return false; Active.reset(); LastSuccessfulRawReadTick = 0; }
			Status = EPollStatus::Resynchronized; return false;
		}
		// A lock acknowledgement requires a committed real reading from that
		// exact selected ID, not merely an activity-policy decision.
		if (Activity && PendingLock && Batch.Ticket && Batch.Readings.Count
			&& Batch.Ticket->Device.Id == *PendingLock)
			AppliedLock = PendingLock;
		if (Batch.Readings.Count) LastSuccessfulRawReadTick = AcquisitionTick;
		Status = Batch.Status; return true;
	}
	ERawPollReject GetLastRawPollReject() const
	{
		std::lock_guard<std::mutex> Lock(Gate);
		return LastRawPollReject;
	}
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
	// A supervised hardware probe must not start its gesture window until a
	// queued lock has crossed a real acquisition boundary.
	bool HasAppliedRawLockDiagnostic(const FDeviceId& Id) const
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Gate);
		return RawMode && !Stopping && Activity && !HasPendingLock
			&& AppliedLock && *AppliedLock == Id && Active
			&& Active->Ticket.Device.Id == Id;
	}
	bool Shutdown()
	{
		std::lock_guard<std::mutex> Lock(Gate);
		Stopping = true; if (!ResetCursorsLocked()) return false; Active.reset(); ActivityCursors.clear();
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
		bPaused = Paused; if (!ResetCursorsLocked()) return false; Active.reset(); Sequence = 0;
		if (Activity)
		{
			Activity->SetPaused(Paused);
			for (auto& Item : ActivityCursors) Item.second.Cursor.Reset(); // Retain read-side disconnect quarantine.
		}
		return true;
	}
	// Acquisition owner only. Physical pause must not call SetPaused on this
	// source: it continues supplying control requests while the worker is idle.
	bool RequestFreshRawReading()
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Gate);
		if (!RawMode || Stopping || bPaused || FAILED(Discovery->GetLastError())) return false;
		if (!ResetCursorsLocked()) return false;
		Active.reset(); AppliedLock.reset(); LastSuccessfulRawReadTick = 0; return true;
	}
	std::vector<FDiscoveredDevice> Snapshot() const { return Discovery->Snapshot(); }
	std::optional<FInputFrame> Produce(FFrameNumber Frame) override
	{
		if (FPresentationInputScope::IsActive()) return std::nullopt;
		std::lock_guard<std::mutex> Lock(Gate);
		if (Stopping || RawMode) return std::nullopt;
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
		if (Stopping || RawMode) return false;
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
	bool ResetCursorsLocked()
	{
		Cursor.Reset();
		if (CallbackCursor && !CallbackCursor->Stop())
		{ Discovery->FailAcquisition(E_FAIL); Status = EPollStatus::Failed; return false; }
		return true;
	}
	void ResetReadingLocked(const FLease& Lease)
	{
		// A changed ticket already means the discovery callback purged old state.
		Discovery->Resynchronize(Lease);
		ResetCursorsLocked(); Active.reset(); Sequence = 0; LastSuccessfulRawReadTick = 0;
		Status = EPollStatus::Resynchronized;
	}
	bool PollActivityLocked(FFrameNumber Frame)
	{
		using namespace GameInput::v3;
		if (HasPendingLock)
		{
			Activity->SetLock(PendingLock);
			AppliedLock.reset(); HasPendingLock = false;
		}
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
	std::optional<FDeviceId> AppliedLock;
	std::optional<FRawReadGapDiagnostic> LastRawReadGap;
	std::uint64_t LastSuccessfulRawReadTick = 0;
	bool HasPendingLock = false;
	std::unique_ptr<FGameInputDiscovery> Discovery;
	const FGameInputMapper Keyboard, Gamepad; // Pure/bounded/nonthrowing; no reentrant source calls.
	mutable std::mutex Gate;
	FGameInputReadCursor Cursor;
	std::unique_ptr<FGameInputCallbackReadCursor> CallbackCursor;
	std::unique_ptr<FPollObservation> Observations;
	bool PollAttempted = false;
	bool RawMode = false;
	std::uint64_t LastAcquisitionTick = 0;
	std::optional<FLease> Active;
	std::optional<std::pair<FDeviceId, std::uint64_t>> Disconnected;
	std::optional<FFrameNumber> LastFrame;
	std::uint64_t Sequence = 0;
	FRequest Pending;
	bool HasPending = false, bPaused = false, Stopping = false;
	EPollStatus Status = EPollStatus::NoChange;
	ERawPollReject LastRawPollReject = ERawPollReject::None;
	HRESULT Error = S_OK;
};
}
