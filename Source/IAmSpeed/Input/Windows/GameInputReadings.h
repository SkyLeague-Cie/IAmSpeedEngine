#pragma once

// Optional Windows leaf. No portable header includes this file. The host must
// supply Microsoft GameInput v3 include/link dependencies before enabling it.
// Source-only checkpoint: not included by the Unreal module yet.
#include "../InputFrame.h"
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include <GameInput.h>
#include <wrl/client.h>
#include <functional>
#include <memory>
#include <exception>
#include <cstring>

#if GAMEINPUT_API_VERSION != 3
#error This adapter targets the inspected GameInput v3 API.
#endif

namespace Speed::Input::Windows
{
// Hardware mapping belongs to this platform leaf/host, never the portable core.
struct FDeviceState
{
	std::array<bool, 256> VirtualKeys{};
	// Preserve physical scan codes as well: virtual keys alone lose layout,
	// left/right modifiers and keypad distinctions needed by canonical HID.
	std::array<std::uint32_t, 256> ScanCodes{};
	std::size_t KeyCount = 0;
	std::uint32_t GamepadButtons = 0;
	std::array<float, 6> Axes{}; // LT, RT, LX, LY, RX, RY (GameInput ranges).
	std::uint64_t TimestampMicroseconds = 0;
};

using FGameInputMapper = std::function<bool(const FDeviceState&, FActionValues&)>;
enum class EReadBatchStatus { NoChange, Updated, Resynchronize, Error };
struct FReadBatchResult { EReadBatchStatus Status; HRESULT Error; };
struct FRawDeviceReadBatch
{
	FReadBatchResult Result{EReadBatchStatus::NoChange, S_OK};
	std::array<FDeviceState, 64> States{};
	std::size_t Count = 0;
	bool FreshBaseline = false;
};
// Optional bounded diagnostic values. COM identities never cross this boundary.
struct FReadingObservation
{
	bool Current = false, HasReading = false, HasDeviceId = false;
	HRESULT Result = S_OK;
	bool IdentityValid = false, SamePrevious = false;
	std::uint64_t Timestamp = 0;
	std::array<std::uint8_t, 32> DeviceId{};
};
struct FReadObservations
{
	std::array<FReadingObservation, 65> Calls{};
	std::size_t Count = 0;
};

// Raw traversal only: no session, journal, selection or lifecycle authority.
// The owner resets this cursor whenever its acquisition generation changes.
class FGameInputReadCursor
{
public:
	void Reset() { Cursor.Reset(); }
	// Raw traversal seam for the V2 adapter. No action frame is converted: the
	// existing device decoder supplies complete hardware state before mapping.
	// A failed traversal returns no partial batch and requires a new baseline.
	FRawDeviceReadBatch PollRaw(GameInput::v3::IGameInput& Api,
		GameInput::v3::IGameInputDevice* Device, GameInput::v3::GameInputKind Kind)
	{
		FRawDeviceReadBatch Batch;
		Batch.FreshBaseline = !Cursor;
		FDeviceState Staged;
		Batch.Result = Poll(Api, Device, Kind,
			[&](const FDeviceState& State, FActionValues&) { Staged = State; return true; },
			[&](const FActionValues&)
			{
				if (Batch.Count == Batch.States.size()) return false;
				Batch.States[Batch.Count++] = Staged;
				return true;
			});
		if (Batch.Result.Status != EReadBatchStatus::Updated && Batch.Result.Status != EReadBatchStatus::NoChange)
		{
			Batch.Count = 0;
			Batch.States = {};
			Cursor.Reset();
		}
		return Batch;
	}
	FReadBatchResult Poll(GameInput::v3::IGameInput& Api, GameInput::v3::IGameInputDevice* Device,
		GameInput::v3::GameInputKind Kind, const FGameInputMapper& Mapper,
		const std::function<bool(const FActionValues&)>& Commit, FReadObservations* Observations = nullptr)
	{
		using namespace GameInput::v3;
		if (Observations) *Observations = {};
		if (!Device || !Mapper || !Commit || (Kind != GameInputKindKeyboard && Kind != GameInputKindGamepad))
			return {EReadBatchStatus::Error, E_INVALIDARG};
		bool Changed = false;
		constexpr std::size_t MaxReadings = 64;
		for (std::size_t I = 0; I <= MaxReadings; ++I)
		{
			Microsoft::WRL::ComPtr<IGameInputReading> Next;
			const auto Status = Cursor
				? Api.GetNextReading(Cursor.Get(), Kind, Device, Next.GetAddressOf())
				: Api.GetCurrentReading(Kind, Device, Next.GetAddressOf());
			if (Observations)
			{
				auto& O = Observations->Calls[Observations->Count++];
				O.Current = !Cursor; O.Result = Status; O.HasReading = !!Next;
				if (Next) {
					Microsoft::WRL::ComPtr<IUnknown> Identity, PreviousIdentity;
					O.IdentityValid = SUCCEEDED(Next.As(&Identity)) && Identity;
					if (Cursor) {
						O.IdentityValid = O.IdentityValid && SUCCEEDED(Cursor.As(&PreviousIdentity)) && PreviousIdentity;
						O.SamePrevious = O.IdentityValid && Identity.Get() == PreviousIdentity.Get();
					}
					O.Timestamp = Next->GetTimestamp();
					Microsoft::WRL::ComPtr<IGameInputDevice> Observed;
					Next->GetDevice(Observed.GetAddressOf());
					const GameInputDeviceInfo* Info = nullptr;
					if (Observed && SUCCEEDED(Observed->GetDeviceInfo(&Info)) && Info) {
						static_assert(sizeof(Info->deviceId) == sizeof(O.DeviceId), "diagnostic ID size");
						std::memcpy(O.DeviceId.data(), &Info->deviceId, O.DeviceId.size()); O.HasDeviceId = true;
					}
				}
			}
			if (Status == GAMEINPUT_E_READING_NOT_FOUND)
				return {Changed ? EReadBatchStatus::Updated : EReadBatchStatus::NoChange, S_OK};
			if (FAILED(Status)) return {EReadBatchStatus::Error, Status};
			if (!Next) return {EReadBatchStatus::Error, E_UNEXPECTED};
			if (I == MaxReadings || (Cursor && Next->GetTimestamp() < Cursor->GetTimestamp()))
				return {EReadBatchStatus::Resynchronize, S_OK};
			FDeviceState State{};
			State.TimestampMicroseconds = Next->GetTimestamp();
			if (Kind == GameInputKindKeyboard)
			{
				std::array<GameInputKeyState, 256> Keys{};
				const auto Count = Next->GetKeyCount();
				if (Count > Keys.size() || Next->GetKeyState(static_cast<std::uint32_t>(Keys.size()), Keys.data()) != Count)
					return {EReadBatchStatus::Resynchronize, S_OK};
				State.KeyCount = Count;
				for (std::uint32_t K = 0; K < Count; ++K)
				{
					State.VirtualKeys[Keys[K].virtualKey] = true;
					State.ScanCodes[K] = Keys[K].scanCode;
				}
			}
			else
			{
				GameInputGamepadState Pad{};
				if (!Next->GetGamepadState(&Pad)) return {EReadBatchStatus::Resynchronize, S_OK};
				State.GamepadButtons = static_cast<std::uint32_t>(Pad.buttons);
				State.Axes = {Pad.leftTrigger, Pad.rightTrigger, Pad.leftThumbstickX,
					Pad.leftThumbstickY, Pad.rightThumbstickX, Pad.rightThumbstickY};
			}
			FActionValues Values{};
			if (!Mapper(State, Values) || !Commit(Values)) return {EReadBatchStatus::Resynchronize, S_OK};
			Cursor = std::move(Next); Changed = true;
		}
		return {EReadBatchStatus::Resynchronize, S_OK};
	}
private:
	Microsoft::WRL::ComPtr<GameInput::v3::IGameInputReading> Cursor;
};
}
