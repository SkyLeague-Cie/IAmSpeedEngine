#pragma once

#include "../DeviceInputSession.h"
#include "GameInputReadings.h"

namespace Speed::Input::Windows
{
enum class EPollStatus { NoChange, Updated, Disconnected, Paused, Resynchronized, Failed };

/** One explicitly selected device/kind, no implicit keyboard/gamepad arbitration.
 * The host creates GameInput and selects a device during lifecycle setup.
 * Produce polls SDK history on the IAmSpeed caller; replay never touches the OS.
 * Connection callbacks only update a tiny mailbox. No UObject or UE dispatch.
 */
class FGameInputAcquisition final : public IInputProducer
{
	using FApi = GameInput::v3::IGameInput;
	using FDevice = GameInput::v3::IGameInputDevice;
	using FReading = GameInput::v3::IGameInputReading;
	using FKind = GameInput::v3::GameInputKind;
	template<class T> using TComPtr = Microsoft::WRL::ComPtr<T>;
public:
	using FMapper = std::function<bool(const FDeviceState&, FActionValues&)>;
	static std::unique_ptr<FGameInputAcquisition> Create(FApi* Api, FDevice* Device,
		FKind Kind, std::uint64_t ProducerId, const std::array<bool, ActionCount>& Digital,
		FMapper Mapper)
	{
		using namespace GameInput::v3;
		if (FPresentationInputScope::IsActive() || !Api || !Device || !ProducerId || !Mapper
			|| (Kind != GameInputKindKeyboard && Kind != GameInputKindGamepad)) return nullptr;
		const GameInputDeviceInfo* Info = nullptr;
		if (FAILED(Device->GetDeviceInfo(&Info)) || !Info || !(Info->supportedInput & Kind)) return nullptr;
		auto Result = std::unique_ptr<FGameInputAcquisition>(
			new FGameInputAcquisition(Api, Device, Kind, ProducerId, Digital, std::move(Mapper)));
		// Blocking enumeration + notifications avoids a query/register hotplug gap.
		if (FAILED(Api->RegisterDeviceCallback(Device, Kind, GameInputDeviceConnected,
			GameInputBlockingEnumeration, Result.get(), &OnConnection, &Result->CallbackToken))) return nullptr;
		Result->Registered = true;
		return Result;
	}
	~FGameInputAcquisition() override
	{
		// Host must resolve a failed explicit Shutdown before releasing ownership.
		// Fail-fast is the last resort: never free a possibly live callback context.
		if (!Shutdown()) std::terminate();
	}
	bool Shutdown()
	{
		// Never invoke from OnConnection: unregister waits for that callback.
		// Holding Gate is safe because OnConnection only takes MailboxMutex.
		std::lock_guard<std::mutex> Lock(Gate);
		ShutdownStarted = true;
		Cursor.Reset();
		// No mailbox lock: successful v3 unregister waits for callbacks to finish.
		// false keeps Registered and this object's resources alive for a retry.
		if (Registered && !Api->UnregisterCallback(CallbackToken))
		{
			LastStatus = EPollStatus::Failed; LastError = E_FAIL;
			return false;
		}
		Registered = false;
		return true;
	}
	bool SetPaused(bool Paused)
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Gate);
		if (ShutdownStarted || PermanentFailure) return false;
		const auto Next = Session.SetPaused(Paused);
		if (!Next) return false;
		Generation = *Next; bPaused = Paused; Cursor.Reset(); ReadingSequence = 0;
		return true;
	}
	std::optional<FInputFrame> Produce(FFrameNumber Frame) override
	{
		if (FPresentationInputScope::IsActive()) return std::nullopt;
		std::lock_guard<std::mutex> Lock(Gate);
		if (ShutdownStarted) return std::nullopt;
		if (LastFrame && Frame <= *LastFrame) return Session.Produce(Frame);
		if (LastFrame && (*LastFrame == std::numeric_limits<FFrameNumber>::max() || Frame != *LastFrame + 1))
			return std::nullopt;
		LastStatus = PollLocked();
		if (LastStatus == EPollStatus::Failed && (!PermanentFailure || !FatalResetReady)) return std::nullopt;
		std::lock_guard<std::mutex> ConnectionLock(MailboxMutex);
		if (!PrepareLatchLocked()) return std::nullopt;
		auto Result = Session.Produce(Frame);
		if (Result) LastFrame = Frame;
		return Result;
	}
	bool Skip(FFrameNumber Frame) override
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Gate);
		if (ShutdownStarted) return false;
		if (LastFrame && Frame <= *LastFrame) return Session.Skip(Frame);
		if (LastFrame && (*LastFrame == std::numeric_limits<FFrameNumber>::max() || Frame != *LastFrame + 1)) return false;
		LastStatus = PollLocked();
		if (LastStatus == EPollStatus::Failed && (!PermanentFailure || !FatalResetReady)) return false;
		std::lock_guard<std::mutex> ConnectionLock(MailboxMutex);
		if (!PrepareLatchLocked() || !Session.Skip(Frame)) return false;
		LastFrame = Frame;
		return true;
	}
	EPollStatus GetLastPollStatus() const
	{
		std::lock_guard<std::mutex> Lock(Gate);
		return LastStatus;
	}
	HRESULT GetLastError() const
	{
		std::lock_guard<std::mutex> Lock(Gate);
		return LastError;
	}

private:
	FGameInputAcquisition(FApi* InApi, FDevice* InDevice, FKind InKind, std::uint64_t Id,
		const std::array<bool, ActionCount>& Digital, FMapper InMapper)
		: Api(InApi), Device(InDevice), Kind(InKind), Session(Id, Digital), Mapper(std::move(InMapper)) {}

	static void CALLBACK OnConnection(GameInput::v3::GameInputCallbackToken, void* Context,
		FDevice*, std::uint64_t, GameInput::v3::GameInputDeviceStatus Current,
		GameInput::v3::GameInputDeviceStatus)
	{
		auto& Self = *static_cast<FGameInputAcquisition*>(Context);
		std::lock_guard<std::mutex> Lock(Self.MailboxMutex);
		Self.Connected = (Current & GameInput::v3::GameInputDeviceConnected) != 0;
		if (Self.ConnectionEpoch == std::numeric_limits<std::uint64_t>::max()) Self.EpochExhausted = true;
		else ++Self.ConnectionEpoch;
	}
	EPollStatus ResynchronizeLocked()
	{
		Cursor.Reset(); ReadingSequence = 0;
		const auto Next = Session.Resynchronize();
		if (!Next) return EPollStatus::Failed;
		Generation = *Next;
		return EPollStatus::Resynchronized;
	}
	EPollStatus FailLocked(HRESULT Error)
	{
		LastError = Error; PermanentFailure = true;
		Cursor.Reset();
		FatalResetReady = bool(Session.Resynchronize());
		return EPollStatus::Failed;
	}
	bool PrepareLatchLocked()
	{
		// Permanent OS failure still produces canonical neutral frames. Do not
		// process later connection events or overwrite its diagnostic until rebuild.
		if (PermanentFailure) return FatalResetReady;
		if (RefreshConnectionLocked()) return true;
		return PermanentFailure && FatalResetReady;
	}
	EPollStatus HandleReadErrorLocked(HRESULT Error)
	{
		using namespace GameInput::v3;
		LastError = Error;
		if (Error == GAMEINPUT_E_REFERENCE_READING_TOO_OLD) return ResynchronizeLocked();
		if (Error == GAMEINPUT_E_DEVICE_DISCONNECTED)
		{
			std::lock_guard<std::mutex> Lock(MailboxMutex);
			// Keep the real callback mailbox untouched. Only a subsequent SDK
			// connection epoch may clear this authoritative read-side disconnect.
			DisconnectedAtEpoch = ConnectionEpoch;
			SeenEpoch = ConnectionEpoch;
			Cursor.Reset(); ReadingSequence = 0;
			const auto Next = Session.SetConnected(false);
			if (!Next) return FailLocked(Error);
			Generation = *Next;
			return EPollStatus::Disconnected;
		}
		// DEVICE_NOT_FOUND, OBJECT_NO_LONGER_EXISTS, INPUT_KIND_NOT_PRESENT,
		// invalid arguments and unknown failures require explicit reconstruction.
		return FailLocked(Error);
	}
	// Caller owns Gate + MailboxMutex. Also used at the final physical latch so
	// a disconnect arriving after the last mapped reading still neutralizes it.
	bool RefreshConnectionLocked()
	{
		if (EpochExhausted) { LastStatus = FailLocked(E_UNEXPECTED); return false; }
		if (!SeenEpoch || *SeenEpoch != ConnectionEpoch)
		{
			DisconnectedAtEpoch.reset();
			const auto Next = Session.SetConnected(Connected);
			if (!Next) { LastStatus = FailLocked(E_FAIL); return false; }
			Generation = *Next; SeenEpoch = ConnectionEpoch; Cursor.Reset(); ReadingSequence = 0;
		}
		if (!Connected || DisconnectedAtEpoch) LastStatus = EPollStatus::Disconnected;
		return true;
	}
	EPollStatus PollLocked()
	{
		using namespace GameInput::v3;
		if (PermanentFailure) return EPollStatus::Failed;
		std::uint64_t Epoch;
		{
			std::lock_guard<std::mutex> Lock(MailboxMutex);
			if (!RefreshConnectionLocked()) return EPollStatus::Failed;
			Epoch = ConnectionEpoch;
			if (!Connected || DisconnectedAtEpoch) return EPollStatus::Disconnected;
		}
		if (bPaused) return EPollStatus::Paused;
		const auto Result = Cursor.Poll(*Api.Get(), Device.Get(), Kind, Mapper,
			[&](const FActionValues& Values)
			{
				std::lock_guard<std::mutex> Lock(MailboxMutex);
				if (EpochExhausted || ConnectionEpoch != Epoch || !Connected
					|| ReadingSequence == std::numeric_limits<std::uint64_t>::max()
					|| !Session.Submit(Generation, ReadingSequence + 1, Values)) return false;
				++ReadingSequence;
				return true;
			});
		LastError = Result.Error;
		if (Result.Status == EReadBatchStatus::Error) return HandleReadErrorLocked(Result.Error);
		if (Result.Status == EReadBatchStatus::Resynchronize) return ResynchronizeLocked();
		return Result.Status == EReadBatchStatus::Updated ? EPollStatus::Updated : EPollStatus::NoChange;
	}

	TComPtr<FApi> Api;
	TComPtr<FDevice> Device;
	const FKind Kind;
	FDeviceInputSession Session;
	const FMapper Mapper; // Pure, bounded, nonthrowing; no reentrant/lifecycle calls.
	mutable std::mutex Gate;
	std::mutex MailboxMutex;
	GameInput::v3::GameInputCallbackToken CallbackToken = 0;
	bool Registered = false;
	bool ShutdownStarted = false;
	bool PermanentFailure = false;
	bool FatalResetReady = false;
	HRESULT LastError = S_OK;
	bool Connected = false;
	bool EpochExhausted = false;
	std::uint64_t ConnectionEpoch = 0;
	std::optional<std::uint64_t> SeenEpoch;
	std::optional<std::uint64_t> DisconnectedAtEpoch;
	std::uint64_t Generation = 0;
	std::uint64_t ReadingSequence = 0;
	FGameInputReadCursor Cursor;
	std::optional<FFrameNumber> LastFrame;
	bool bPaused = false;
	EPollStatus LastStatus = EPollStatus::NoChange;
};
}
