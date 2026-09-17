#pragma once
#include "IAmSpeed/Input/Windows/GameInputAcquisition.h"
#include <atomic>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <thread>
#include <vector>

using namespace GameInput::v3;
using namespace Speed::Input;
using Speed::Input::Windows::FGameInputAcquisition;
using Speed::Input::Windows::FDeviceState;
using Speed::Input::Windows::EPollStatus;
template<class T> using ComPtr = Microsoft::WRL::ComPtr<T>;
static unsigned Checks = 0;
static void Check(bool Value, const char* Message)
{
	++Checks;
	if (!Value) { std::cerr << "FAIL " << Message << '\n'; std::exit(1); }
}
template<class P> static void Await(P Predicate)
{
	const auto Deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
	while (!Predicate())
	{
		if (std::chrono::steady_clock::now() > Deadline) { Check(false, "bounded synchronization timeout"); }
		std::this_thread::yield();
	}
}
template<class Interface> class FakeCom : public Interface
{
public:
	HRESULT STDMETHODCALLTYPE QueryInterface(REFIID, void** Out) override { *Out = nullptr; return E_NOINTERFACE; }
	ULONG STDMETHODCALLTYPE AddRef() override { return ++References; }
	ULONG STDMETHODCALLTYPE Release() override { auto N = --References; if (!N) delete this; return N; }
	ULONG RefCount() const { return References.load(); }
protected:
	virtual ~FakeCom() = default;
private:
	std::atomic<ULONG> References{1};
};

class FakeDevice final : public FakeCom<IGameInputDevice>
{
public:
	GameInputDeviceInfo Info{};
	FakeDevice() { Info.supportedInput = GameInputKindKeyboard | GameInputKindGamepad; }
	HRESULT STDMETHODCALLTYPE GetDeviceInfo(const GameInputDeviceInfo** Out) override { *Out = &Info; return S_OK; }
	HRESULT STDMETHODCALLTYPE GetHapticInfo(GameInputHapticInfo*) override { return E_NOTIMPL; }
	GameInputDeviceStatus STDMETHODCALLTYPE GetDeviceStatus() override { return GameInputDeviceConnected; }
	HRESULT STDMETHODCALLTYPE CreateForceFeedbackEffect(uint32_t, const GameInputForceFeedbackParams*, IGameInputForceFeedbackEffect**) override { return E_NOTIMPL; }
	bool STDMETHODCALLTYPE IsForceFeedbackMotorPoweredOn(uint32_t) override { return false; }
	void STDMETHODCALLTYPE SetForceFeedbackMotorGain(uint32_t, float) override {}
	void STDMETHODCALLTYPE SetRumbleState(const GameInputRumbleParams*) override {}
	HRESULT STDMETHODCALLTYPE DirectInputEscape(uint32_t, const void*, uint32_t, void*, uint32_t, uint32_t*) override { return E_NOTIMPL; }
	HRESULT STDMETHODCALLTYPE CreateInputMapper(IGameInputMapper**) override { return E_NOTIMPL; }
	HRESULT STDMETHODCALLTYPE GetExtraAxisCount(GameInputKind, uint32_t*) override { return E_NOTIMPL; }
	HRESULT STDMETHODCALLTYPE GetExtraButtonCount(GameInputKind, uint32_t*) override { return E_NOTIMPL; }
	HRESULT STDMETHODCALLTYPE GetExtraAxisIndexes(GameInputKind, uint32_t, uint8_t*) override { return E_NOTIMPL; }
	HRESULT STDMETHODCALLTYPE GetExtraButtonIndexes(GameInputKind, uint32_t, uint8_t*) override { return E_NOTIMPL; }
	HRESULT STDMETHODCALLTYPE CreateRawDeviceReport(uint32_t, GameInputRawDeviceReportKind, IGameInputRawDeviceReport**) override { return E_NOTIMPL; }
	HRESULT STDMETHODCALLTYPE SendRawDeviceOutput(IGameInputRawDeviceReport*) override { return E_NOTIMPL; }
};

class FakeReading final : public FakeCom<IGameInputReading>
{
public:
	GameInputGamepadState Pad{};
	std::vector<GameInputKeyState> Keys;
	uint64_t Stamp = 100;
	bool ValidPad = true;
	GameInputKind STDMETHODCALLTYPE GetInputKind() override { return GameInputKindGamepad | GameInputKindKeyboard; }
	uint64_t STDMETHODCALLTYPE GetTimestamp() override { return Stamp; }
	void STDMETHODCALLTYPE GetDevice(IGameInputDevice** Out) override { *Out = nullptr; }
	uint32_t STDMETHODCALLTYPE GetControllerAxisCount() override { return 0; }
	uint32_t STDMETHODCALLTYPE GetControllerAxisState(uint32_t, float*) override { return 0; }
	uint32_t STDMETHODCALLTYPE GetControllerButtonCount() override { return 0; }
	uint32_t STDMETHODCALLTYPE GetControllerButtonState(uint32_t, bool*) override { return 0; }
	uint32_t STDMETHODCALLTYPE GetControllerSwitchCount() override { return 0; }
	uint32_t STDMETHODCALLTYPE GetControllerSwitchState(uint32_t, GameInputSwitchPosition*) override { return 0; }
	uint32_t STDMETHODCALLTYPE GetKeyCount() override { return static_cast<uint32_t>(Keys.size()); }
	uint32_t STDMETHODCALLTYPE GetKeyState(uint32_t Count, GameInputKeyState* Out) override
	{
		const auto N = (std::min)(Count, static_cast<uint32_t>(Keys.size()));
		for (uint32_t I = 0; I < N; ++I) Out[I] = Keys[I];
		return N;
	}
	bool STDMETHODCALLTYPE GetMouseState(GameInputMouseState*) override { return false; }
	bool STDMETHODCALLTYPE GetSensorsState(GameInputSensorsState*) override { return false; }
	bool STDMETHODCALLTYPE GetArcadeStickState(GameInputArcadeStickState*) override { return false; }
	bool STDMETHODCALLTYPE GetFlightStickState(GameInputFlightStickState*) override { return false; }
	bool STDMETHODCALLTYPE GetGamepadState(GameInputGamepadState* Out) override { *Out = Pad; return ValidPad; }
	bool STDMETHODCALLTYPE GetRacingWheelState(GameInputRacingWheelState*) override { return false; }
	bool STDMETHODCALLTYPE GetRawReport(IGameInputRawDeviceReport** Out) override { *Out = nullptr; return false; }
};

class FakeApi : public FakeCom<IGameInput>
{
public:
	struct Response { HRESULT Status; ComPtr<FakeReading> Reading; };
	std::deque<Response> Responses;
	std::atomic<unsigned> ReadCalls{0}, CurrentCalls{0}, NextCalls{0}, UnregisterCalls{0};
	bool FailUnregister = false;
	std::mutex CallbackGate;
	std::condition_variable CallbackFinished;
	GameInputDeviceCallback Callback = nullptr;
	void* Context = nullptr;
	unsigned Running = 0;
	std::atomic<bool> WaitingForCallback{false};
	std::function<void()> BeforeReturn;
	void Error(HRESULT H) { Responses.push_back({H, {}}); }
	void Reading(float ThrottleValue = 1, bool Held = true, uint64_t Stamp = 100)
	{
		ComPtr<FakeReading> R; R.Attach(new FakeReading);
		R->Pad.rightTrigger = ThrottleValue; R->Pad.buttons = Held ? GameInputGamepadA : GameInputGamepadNone;
		R->Stamp = Stamp; Responses.push_back({S_OK, R});
	}
	void Fire(bool Connected)
	{
		GameInputDeviceCallback Function; void* Data;
		{ std::lock_guard<std::mutex> Lock(CallbackGate); if (!Callback) return; ++Running; Function = Callback; Data = Context; }
		Function(1, Data, nullptr, 0, Connected ? GameInputDeviceConnected : static_cast<GameInputDeviceStatus>(0), static_cast<GameInputDeviceStatus>(0));
		{ std::lock_guard<std::mutex> Lock(CallbackGate); --Running; CallbackFinished.notify_all(); }
	}
	HRESULT Pop(IGameInputReading** Out)
	{
		++ReadCalls; *Out = nullptr;
		if (BeforeReturn) { auto Hook = std::move(BeforeReturn); BeforeReturn = {}; Hook(); }
		if (Responses.empty()) return GAMEINPUT_E_READING_NOT_FOUND;
		auto R = std::move(Responses.front()); Responses.pop_front();
		if (R.Reading) { *Out = R.Reading.Get(); (*Out)->AddRef(); }
		return R.Status;
	}
	uint64_t STDMETHODCALLTYPE GetCurrentTimestamp() override { return 100; }
	HRESULT STDMETHODCALLTYPE GetCurrentReading(GameInputKind, IGameInputDevice*, IGameInputReading** Out) override { ++CurrentCalls; return Pop(Out); }
	HRESULT STDMETHODCALLTYPE GetNextReading(IGameInputReading*, GameInputKind, IGameInputDevice*, IGameInputReading** Out) override { ++NextCalls; return Pop(Out); }
	HRESULT STDMETHODCALLTYPE GetPreviousReading(IGameInputReading*, GameInputKind, IGameInputDevice*, IGameInputReading**) override { return E_NOTIMPL; }
	HRESULT STDMETHODCALLTYPE RegisterReadingCallback(IGameInputDevice*, GameInputKind, void*, GameInputReadingCallback, GameInputCallbackToken*) override { return E_NOTIMPL; }
	HRESULT STDMETHODCALLTYPE RegisterDeviceCallback(IGameInputDevice*, GameInputKind, GameInputDeviceStatus, GameInputEnumerationKind, void* Data, GameInputDeviceCallback Function, GameInputCallbackToken* Token) override
	{
		Callback = Function; Context = Data; *Token = 1; Fire(true); return S_OK;
	}
	HRESULT STDMETHODCALLTYPE RegisterSystemButtonCallback(IGameInputDevice*, GameInputSystemButtons, void*, GameInputSystemButtonCallback, GameInputCallbackToken*) override { return E_NOTIMPL; }
	HRESULT STDMETHODCALLTYPE RegisterKeyboardLayoutCallback(IGameInputDevice*, void*, GameInputKeyboardLayoutCallback, GameInputCallbackToken*) override { return E_NOTIMPL; }
	void STDMETHODCALLTYPE StopCallback(GameInputCallbackToken) override {}
	bool STDMETHODCALLTYPE UnregisterCallback(GameInputCallbackToken) override
	{
		++UnregisterCalls;
		if (FailUnregister) return false;
		std::unique_lock<std::mutex> Lock(CallbackGate);
		if (Running) WaitingForCallback.store(true);
		CallbackFinished.wait(Lock, [&] { return Running == 0; });
		Callback = nullptr; Context = nullptr; return true;
	}
	HRESULT STDMETHODCALLTYPE CreateDispatcher(IGameInputDispatcher**) override { return E_NOTIMPL; }
	HRESULT STDMETHODCALLTYPE FindDeviceFromId(const APP_LOCAL_DEVICE_ID*, IGameInputDevice**) override { return E_NOTIMPL; }
	HRESULT STDMETHODCALLTYPE FindDeviceFromPlatformString(LPCWSTR, IGameInputDevice**) override { return E_NOTIMPL; }
	void STDMETHODCALLTYPE SetFocusPolicy(GameInputFocusPolicy) override {}
	HRESULT STDMETHODCALLTYPE CreateAggregateDevice(GameInputKind, APP_LOCAL_DEVICE_ID*) override { return E_NOTIMPL; }
	HRESULT STDMETHODCALLTYPE DisableAggregateDevice(const APP_LOCAL_DEVICE_ID*) override { return E_NOTIMPL; }
};
