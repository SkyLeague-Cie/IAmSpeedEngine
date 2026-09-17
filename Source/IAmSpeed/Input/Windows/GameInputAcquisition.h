#pragma once

// Optional Windows leaf. No portable header includes this file. The host must
// supply Microsoft GameInput v3 include/link dependencies before enabling it.
// Source-only checkpoint: not included by the Unreal module yet.
#include "../DeviceInputSession.h"
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include <GameInput.h>
#include <wrl/client.h>
#include <functional>
#include <memory>

#if GAMEINPUT_API_VERSION != 3
#error This adapter targets the inspected GameInput v3 API.
#endif

namespace Speed::Input::Windows
{
// Hardware mapping belongs to this platform leaf/host, never the portable core.
struct FDeviceState
{
	std::array<bool, 256> VirtualKeys{};
	std::uint32_t GamepadButtons = 0;
	std::array<float, 6> Axes{}; // LT, RT, LX, LY, RX, RY (GameInput ranges).
	std::uint64_t TimestampMicroseconds = 0;
};

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
		// v3 UnregisterCallback waits for a running callback. Never hold MailboxMutex
		// here and never destroy this object from inside OnConnection.
		if (Registered) Api->UnregisterCallback(CallbackToken);
	}
	bool SetPaused(bool Paused)
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Gate);
		const auto Next = Session.SetPaused(Paused);
		if (!Next) return false;
		Generation = *Next; bPaused = Paused; Cursor.Reset(); ReadingSequence = 0;
		return true;
	}
	std::optional<FInputFrame> Produce(FFrameNumber Frame) override
	{
		if (FPresentationInputScope::IsActive()) return std::nullopt;
		std::lock_guard<std::mutex> Lock(Gate);
		if (LastFrame && Frame <= *LastFrame) return Session.Produce(Frame);
		if (LastFrame && (*LastFrame == std::numeric_limits<FFrameNumber>::max() || Frame != *LastFrame + 1))
			return std::nullopt;
		LastStatus = PollLocked();
		if (LastStatus == EPollStatus::Failed) return std::nullopt;
		std::lock_guard<std::mutex> ConnectionLock(MailboxMutex);
		if (!RefreshConnectionLocked()) return std::nullopt;
		auto Result = Session.Produce(Frame);
		if (Result) LastFrame = Frame;
		return Result;
	}
	bool Skip(FFrameNumber Frame) override
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Gate);
		if (LastFrame && Frame <= *LastFrame) return Session.Skip(Frame);
		if (LastFrame && (*LastFrame == std::numeric_limits<FFrameNumber>::max() || Frame != *LastFrame + 1)) return false;
		LastStatus = PollLocked();
		if (LastStatus == EPollStatus::Failed) return false;
		std::lock_guard<std::mutex> ConnectionLock(MailboxMutex);
		if (!RefreshConnectionLocked() || !Session.Skip(Frame)) return false;
		LastFrame = Frame;
		return true;
	}
	EPollStatus GetLastPollStatus() const
	{
		std::lock_guard<std::mutex> Lock(Gate);
		return LastStatus;
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
	// Caller owns Gate + MailboxMutex. Also used at the final physical latch so
	// a disconnect arriving after the last mapped reading still neutralizes it.
	bool RefreshConnectionLocked()
	{
		if (EpochExhausted) return false;
		if (!SeenEpoch || *SeenEpoch != ConnectionEpoch)
		{
			const auto Next = Session.SetConnected(Connected);
			if (!Next) return false;
			Generation = *Next; SeenEpoch = ConnectionEpoch; Cursor.Reset(); ReadingSequence = 0;
		}
		if (!Connected) LastStatus = EPollStatus::Disconnected;
		return true;
	}
	bool MapReading(FReading& Reading, FActionValues& Values)
	{
		using namespace GameInput::v3;
		FDeviceState State;
		State.TimestampMicroseconds = Reading.GetTimestamp();
		if (Kind == GameInputKindKeyboard)
		{
			std::array<GameInputKeyState, 256> Keys{};
			const auto Count = Reading.GetKeyCount();
			if (Count > Keys.size() || Reading.GetKeyState(static_cast<std::uint32_t>(Keys.size()), Keys.data()) != Count)
				return false;
			for (std::uint32_t I = 0; I < Count; ++I) State.VirtualKeys[Keys[I].virtualKey] = true;
		}
		else
		{
			GameInputGamepadState Pad{};
			if (!Reading.GetGamepadState(&Pad)) return false;
			State.GamepadButtons = static_cast<std::uint32_t>(Pad.buttons);
			State.Axes = {Pad.leftTrigger, Pad.rightTrigger, Pad.leftThumbstickX,
				Pad.leftThumbstickY, Pad.rightThumbstickX, Pad.rightThumbstickY};
		}
		return Mapper(State, Values);
	}
	EPollStatus PollLocked()
	{
		using namespace GameInput::v3;
		std::uint64_t Epoch;
		{
			std::lock_guard<std::mutex> Lock(MailboxMutex);
			if (!RefreshConnectionLocked()) return EPollStatus::Failed;
			Epoch = ConnectionEpoch;
			if (!Connected) return EPollStatus::Disconnected;
		}
		if (bPaused) return EPollStatus::Paused;
		bool Changed = false;
		constexpr std::size_t MaxReadingsPerPoll = 64;
		for (std::size_t I = 0; I <= MaxReadingsPerPoll; ++I)
		{
			TComPtr<FReading> Next;
			const HRESULT Status = Cursor
				? Api->GetNextReading(Cursor.Get(), Kind, Device.Get(), Next.GetAddressOf())
				: Api->GetCurrentReading(Kind, Device.Get(), Next.GetAddressOf());
			if (Status == GAMEINPUT_E_READING_NOT_FOUND) return Changed ? EPollStatus::Updated : EPollStatus::NoChange;
			if (FAILED(Status) || !Next || I == MaxReadingsPerPoll) return ResynchronizeLocked();
			if (Cursor && Next->GetTimestamp() < Cursor->GetTimestamp()) return ResynchronizeLocked();
			FActionValues Values{};
			if (!MapReading(*Next.Get(), Values)) return ResynchronizeLocked();
			// The callback may run while the SDK or mapper is reading. Recheck under
			// the mailbox lock and retain it through commit, not just callback entry.
			{
				std::lock_guard<std::mutex> Lock(MailboxMutex);
				if (EpochExhausted) return EPollStatus::Failed;
				if (ConnectionEpoch != Epoch || !Connected) return ResynchronizeLocked();
				if (ReadingSequence == std::numeric_limits<std::uint64_t>::max()
					|| !Session.Submit(Generation, ReadingSequence + 1, Values)) return ResynchronizeLocked();
				++ReadingSequence;
			}
			Cursor = std::move(Next); Changed = true;
		}
		return ResynchronizeLocked();
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
	bool Connected = false;
	bool EpochExhausted = false;
	std::uint64_t ConnectionEpoch = 0;
	std::optional<std::uint64_t> SeenEpoch;
	std::uint64_t Generation = 0;
	std::uint64_t ReadingSequence = 0;
	TComPtr<FReading> Cursor;
	std::optional<FFrameNumber> LastFrame;
	bool bPaused = false;
	EPollStatus LastStatus = EPollStatus::NoChange;
};
}
