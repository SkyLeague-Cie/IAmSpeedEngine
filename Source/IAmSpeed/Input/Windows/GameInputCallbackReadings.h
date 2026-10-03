#pragma once

#include "GameInputReadings.h"
#include <deque>
#include <mutex>

namespace Speed::Input::Windows
{
// Retains OS-delivered readings until the acquisition worker's next poll.
// The callback only takes a bounded COM reference; decoding and publication
// remain on the acquisition worker, never on GameInput's callback thread.
class FGameInputCallbackReadCursor final
{
	using FApi = GameInput::v3::IGameInput;
	using FDevice = GameInput::v3::IGameInputDevice;
	using FReading = GameInput::v3::IGameInputReading;
	using FKind = GameInput::v3::GameInputKind;
	using FRef = Microsoft::WRL::ComPtr<FReading>;
	static constexpr std::size_t QueueCapacity = 256;
	static constexpr std::size_t SeenCapacity = 512;
public:
	~FGameInputCallbackReadCursor() { if (!Stop()) std::terminate(); }
	FGameInputCallbackReadCursor(const FGameInputCallbackReadCursor&) = delete;
	FGameInputCallbackReadCursor& operator=(const FGameInputCallbackReadCursor&) = delete;
	FGameInputCallbackReadCursor() = default;

	bool Stop()
	{
		{
			std::lock_guard<std::mutex> Lock(Gate);
			Stopping = true;
		}
		// GameInput fences executing callbacks here. Never hold Gate while waiting.
		if (Registered && !Api->UnregisterCallback(Token)) return false;
		Registered = false;
		std::lock_guard<std::mutex> Lock(Gate);
		Pending.clear(); Seen.clear(); Overflow = false; LastTimestamp = 0;
		Api.Reset(); Device.Reset(); Started = false; Stopping = false;
		return true;
	}

	FRawDeviceReadBatch PollRaw(FApi& InApi, FDevice* InDevice, FKind InKind)
	{
		using namespace GameInput::v3;
		FRawDeviceReadBatch Batch;
		if (!InDevice || (InKind != GameInputKindKeyboard && InKind != GameInputKindGamepad))
		{ Batch.Result = {EReadBatchStatus::Error, E_INVALIDARG}; return Batch; }
		if (Registered && (Api.Get() != &InApi || Device.Get() != InDevice || Kind != InKind))
		{ Batch.Result = {EReadBatchStatus::Error, E_INVALIDARG}; return Batch; }
		if (!Started)
		{
			FRef Baseline;
			const auto Current = InApi.GetCurrentReading(InKind, InDevice, Baseline.GetAddressOf());
			if (Current == GAMEINPUT_E_READING_NOT_FOUND) return Batch;
			if (FAILED(Current) || !Baseline)
			{ Batch.Result = {EReadBatchStatus::Error, FAILED(Current) ? Current : E_UNEXPECTED}; return Batch; }
			Api = &InApi; Device = InDevice; Kind = InKind;
			const auto Registration = Api->RegisterReadingCallback(Device.Get(), Kind, this, &OnReading, &Token);
			if (FAILED(Registration))
			{
				Api.Reset(); Device.Reset();
				Batch.Result = {EReadBatchStatus::Error, Registration}; return Batch;
			}
			Registered = Started = true;
			Batch.FreshBaseline = true;
			if (!Append(Baseline.Get(), Batch)) return Gap(Batch);
			// The baseline precedes registration. Bridge that race through GameInput
			// history, then discard matching callback references by identity.
			FRef Previous = Baseline;
			for (std::size_t I = 1; I <= Batch.States.size(); ++I)
			{
				FRef Next;
				const auto Status = Api->GetNextReading(Previous.Get(), Kind, Device.Get(), Next.GetAddressOf());
				if (Status == GAMEINPUT_E_READING_NOT_FOUND) return Batch;
				if (FAILED(Status) || !Next || I == Batch.States.size()) return Gap(Batch);
				if (!Append(Next.Get(), Batch)) return Gap(Batch);
				Previous = std::move(Next);
			}
			return Gap(Batch);
		}
		std::deque<FRef> Drained;
		bool Lost = false;
		{
			std::lock_guard<std::mutex> Lock(Gate);
			Lost = Overflow;
			while (!Lost && !Pending.empty() && Drained.size() < Batch.States.size())
			{ Drained.push_back(std::move(Pending.front())); Pending.pop_front(); }
		}
		if (Lost) return Gap(Batch);
		for (const auto& Reading : Drained)
			if (!Append(Reading.Get(), Batch)) return Gap(Batch);
		if (Batch.Count) Batch.Result.Status = EReadBatchStatus::Updated;
		return Batch;
	}
private:
	static void CALLBACK OnReading(GameInput::v3::GameInputCallbackToken, void* Context, FReading* Reading)
	{
		auto& Self = *static_cast<FGameInputCallbackReadCursor*>(Context);
		std::lock_guard<std::mutex> Lock(Self.Gate);
		if (Self.Stopping || Self.Overflow) return;
		if (!Reading || Self.Pending.size() == QueueCapacity)
		{ Self.Overflow = true; return; }
		try { Self.Pending.emplace_back(Reading); }
		catch (...) { Self.Overflow = true; }
	}
	FRawDeviceReadBatch Gap(FRawDeviceReadBatch& Batch)
	{
		Batch = {};
		Batch.Result = {EReadBatchStatus::Resynchronize, S_OK};
		if (!Stop()) Batch.Result = {EReadBatchStatus::Error, E_FAIL};
		return Batch;
	}
	bool Append(FReading* Reading, FRawDeviceReadBatch& Batch)
	{
		if (!Reading) return false;
		Microsoft::WRL::ComPtr<IUnknown> Identity;
		if (FAILED(Reading->QueryInterface(__uuidof(IUnknown),
			reinterpret_cast<void**>(Identity.GetAddressOf()))) || !Identity) return false;
		for (const auto& Item : Seen) if (Item.Get() == Identity.Get()) return true;
		const auto Timestamp = Reading->GetTimestamp();
		if (Timestamp < LastTimestamp || Batch.Count == Batch.States.size()) return false;
		FDeviceState State{}; State.TimestampMicroseconds = Timestamp;
		if (Kind == GameInput::v3::GameInputKindKeyboard)
		{
			std::array<GameInput::v3::GameInputKeyState, 256> Keys{};
			const auto Count = Reading->GetKeyCount();
			if (Count > Keys.size() || Reading->GetKeyState(static_cast<std::uint32_t>(Keys.size()), Keys.data()) != Count)
				return false;
			State.KeyCount = Count;
			for (std::uint32_t I = 0; I < Count; ++I)
			{ State.VirtualKeys[Keys[I].virtualKey] = true; State.ScanCodes[I] = Keys[I].scanCode; }
		}
		else
		{
			GameInput::v3::GameInputGamepadState Pad{};
			if (!Reading->GetGamepadState(&Pad)) return false;
			State.GamepadButtons = static_cast<std::uint32_t>(Pad.buttons);
			State.Axes = {Pad.leftTrigger, Pad.rightTrigger, Pad.leftThumbstickX,
				Pad.leftThumbstickY, Pad.rightThumbstickX, Pad.rightThumbstickY};
		}
		Batch.States[Batch.Count++] = State;
		Seen.emplace_back(std::move(Identity));
		if (Seen.size() > SeenCapacity) Seen.pop_front();
		LastTimestamp = Timestamp;
		Batch.Result.Status = EReadBatchStatus::Updated;
		return true;
	}
	mutable std::mutex Gate;
	Microsoft::WRL::ComPtr<FApi> Api;
	Microsoft::WRL::ComPtr<FDevice> Device;
	FKind Kind{};
	GameInput::v3::GameInputCallbackToken Token = 0;
	std::deque<FRef> Pending;
	std::deque<Microsoft::WRL::ComPtr<IUnknown>> Seen;
	std::uint64_t LastTimestamp = 0;
	bool Registered = false, Started = false, Stopping = false, Overflow = false;
};
}
