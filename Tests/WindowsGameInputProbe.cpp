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

class FakeApi final : public FakeCom<IGameInput>
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

struct Fixture
{
	ComPtr<FakeApi> Api;
	ComPtr<FakeDevice> Device;
	std::unique_ptr<FGameInputAcquisition> Source;
	Fixture(FGameInputAcquisition::FMapper Mapper = {}, GameInputKind Kind = GameInputKindGamepad)
	{
		Api.Attach(new FakeApi); Device.Attach(new FakeDevice);
		if (!Mapper) Mapper = [](const FDeviceState& S, FActionValues& V)
		{
			const auto Axis = QuantizeAxis(S.Axes[1], false); if (!Axis) return false;
			V[Throttle] = *Axis; V[3] = (S.GamepadButtons & GameInputGamepadA) ? 1 : 0; return true;
		};
		std::array<bool, ActionCount> Digital{}; Digital[3] = true;
		Source = FGameInputAcquisition::Create(Api.Get(), Device.Get(), Kind, 301, Digital, std::move(Mapper));
		Check(bool(Source), "factory");
	}
};
static bool Same(const FInputFrame& A, const FInputFrame& B)
{
	if (A.GetSourceFrame() != B.GetSourceFrame() || A.GetConsumptionFrame() != B.GetConsumptionFrame()
		|| A.GetProducer().Kind != B.GetProducer().Kind || A.GetProducer().Id != B.GetProducer().Id
		|| A.GetActions() != B.GetActions() || A.RequiresReset() != B.RequiresReset() || A.GetEdgeCount() != B.GetEdgeCount()) return false;
	for (size_t I = 0; I < A.GetEdgeCount(); ++I)
		if (A.GetEdges()[I].Action != B.GetEdges()[I].Action || A.GetEdges()[I].Kind != B.GetEdges()[I].Kind
			|| A.GetEdges()[I].SourceFrame != B.GetEdges()[I].SourceFrame) return false;
	return true;
}
static void ErrorPolicies()
{
	for (HRESULT Error : {GAMEINPUT_E_DEVICE_NOT_FOUND, GAMEINPUT_E_OBJECT_NO_LONGER_EXISTS,
		GAMEINPUT_E_INPUT_KIND_NOT_PRESENT, GAMEINPUT_E_FEEDBACK_NOT_SUPPORTED,
		GAMEINPUT_E_CALLBACK_NOT_FOUND, GAMEINPUT_E_HAPTIC_INFO_NOT_FOUND,
		GAMEINPUT_E_AGGREGATE_OPERATION_NOT_SUPPORTED, E_INVALIDARG, E_FAIL})
	{
		Fixture F; F.Api->Reading(); const auto Old = F.Source->Produce(0);
		Check(Old && Old->GetActions()[Throttle] == 255, "pre-fatal held input");
		F.Api->Error(Error); const auto Neutral = F.Source->Produce(1);
		Check(Neutral && Neutral->RequiresReset() && Neutral->GetActions() == FActionValues{}, "fatal delivers reset");
		Check(F.Source->GetLastPollStatus() == EPollStatus::Failed && F.Source->GetLastError() == Error, "fatal exact diagnostics");
		const auto Calls = F.Api->ReadCalls.load(); F.Api->Fire(false); F.Api->Fire(true);
		Check(F.Source->Skip(2) && F.Source->Skip(3), "fatal neutral cadence permits skips");
		const auto Later = F.Source->Produce(4);
		Check(Later && !Later->RequiresReset() && Later->GetActions() == FActionValues{}, "post-fatal stays neutral");
		Check(F.Api->ReadCalls == Calls && F.Source->GetLastError() == Error && F.Source->GetLastPollStatus() == EPollStatus::Failed, "no OS after fatal even with callbacks");
		Check(Same(*Old, *F.Source->Produce(0)) && F.Api->ReadCalls == Calls, "immutable replay no OS");
	}
	{
		Fixture F; F.Api->Reading(); Check(bool(F.Source->Produce(0)), "override seed");
		F.Api->Error(E_FAIL); Check(F.Source->Skip(1) && F.Source->Skip(2) && F.Source->Skip(3), "fatal in override advances");
		const auto Calls = F.Api->ReadCalls.load(); const auto Resume = F.Source->Produce(4);
		Check(Resume && Resume->RequiresReset() && Resume->GetActions() == FActionValues{} && F.Api->ReadCalls == Calls, "fatal marker survives multi-skip");
	}
	{
		Fixture F; F.Api->Error(GAMEINPUT_E_DEVICE_DISCONNECTED); const auto N = F.Source->Produce(0);
		Check(N && N->RequiresReset() && F.Source->GetLastPollStatus() == EPollStatus::Disconnected, "SDK disconnect immediately neutral");
		const auto Calls = F.Api->ReadCalls.load(); F.Source->Produce(1);
		Check(F.Api->ReadCalls == Calls, "SDK disconnect waits for epoch");
		F.Api->Fire(true); F.Api->Reading(); const auto R = F.Source->Produce(2);
		Check(R && R->GetActions()[Throttle] == 255 && R->GetEdgeCount() == 0, "epoch reconnect fresh baseline");
	}
	{
		Fixture F; F.Api->Reading(); F.Source->Produce(0); F.Api->Error(GAMEINPUT_E_REFERENCE_READING_TOO_OLD);
		const auto N = F.Source->Produce(1);
		Check(N && N->RequiresReset() && F.Source->GetLastPollStatus() == EPollStatus::Resynchronized
			&& F.Source->GetLastError() == GAMEINPUT_E_REFERENCE_READING_TOO_OLD, "old reference resync");
		F.Api->Reading(); Check(F.Source->Produce(2)->GetActions()[Throttle] == 255, "history loss recovers");
		F.Source->Produce(3); Check(F.Source->GetLastPollStatus() == EPollStatus::NoChange && F.Source->GetLastError() == S_OK, "reading not found normal end");
	}
	{
		Fixture F; F.Api->Error(S_OK); const auto N = F.Source->Produce(0);
		Check(N && N->RequiresReset() && F.Source->GetLastError() == E_UNEXPECTED, "null success is fatal contract failure");
	}
}
static void CadenceAndLifecycle()
{
	for (unsigned Count : {64u, 65u})
	{
		Fixture F; for (unsigned I = 0; I < Count; ++I) F.Api->Reading(0.5f, false, 100);
		const auto R = F.Source->Produce(0);
		Check(R && F.Api->ReadCalls == 65, "64/65 bounded API calls");
		Check(F.Source->GetLastPollStatus() == (Count == 64 ? EPollStatus::Updated : EPollStatus::Resynchronized), "64/65 boundary status");
		Check(R->GetActions()[Throttle] == (Count == 64 ? 128 : 0), "65th sample rejects whole pending batch");
	}
	{
		Fixture F; F.Api->Reading(1, true, 100); F.Api->Reading(1, false, 100); F.Api->Reading(1, true, 100);
		const auto R = F.Source->Produce(0);
		Check(R && R->GetEdgeCount() == 2 && R->GetEdges()[0].Kind == EEdgeKind::Stop && R->GetEdges()[1].Kind == EEdgeKind::Start, "equal timestamps retain actual edge order after baseline");
		const auto Calls = F.Api->ReadCalls.load();
		Check(!F.Source->Produce(2) && F.Api->ReadCalls == Calls, "gap rejected without OS");
		Check(F.Source->SetPaused(true), "pause"); const auto Paused = F.Source->Produce(1);
		Check(Paused && Paused->RequiresReset() && Paused->GetActions() == FActionValues{} && F.Api->ReadCalls == Calls, "pause no OS neutral");
		Check(F.Source->Skip(2) && F.Source->SetPaused(false), "paused override then resume");
		F.Api->Reading(); const auto Resumed = F.Source->Produce(3);
		Check(Resumed && Resumed->RequiresReset() && Resumed->GetActions()[3] == 1 && Resumed->GetEdgeCount() == 0, "resume held no fake jump");
	}
	{
		Fixture F; F.Api->Reading(1, false, 200); F.Source->Produce(0); F.Api->Reading(1, true, 199);
		Check(F.Source->Produce(1)->RequiresReset() && F.Source->GetLastPollStatus() == EPollStatus::Resynchronized, "backwards timestamp resets");
	}
	{
		Fixture F; F.Api->Reading(std::numeric_limits<float>::quiet_NaN());
		Check(F.Source->Produce(0)->GetActions() == FActionValues{} && F.Source->GetLastPollStatus() == EPollStatus::Resynchronized, "mapping invalid input resets");
	}
	{
		FakeApi* Api = nullptr;
		Fixture F([&](const FDeviceState&, FActionValues& V) { V[Throttle] = 255; Api->Fire(false); return true; });
		Api = F.Api.Get(); Api->Reading(); const auto R = F.Source->Produce(0);
		Check(R && R->RequiresReset() && R->GetActions() == FActionValues{}
			&& F.Source->GetLastPollStatus() == EPollStatus::Disconnected, "disconnect during mapping cannot commit stale reading");
	}
	{
		Fixture F; F.Api->Reading(); F.Source->Produce(0);
		F.Api->Fire(false); F.Api->Fire(true); F.Api->Reading(); const auto R = F.Source->Produce(1);
		Check(R && R->RequiresReset() && R->GetEdgeCount() == 0, "disconnect reconnect between polls resets baseline");
	}
	{
		FakeApi* Api = nullptr;
		Fixture F([&](const FDeviceState&, FActionValues& V)
		{
			V[Throttle] = 255; Api->BeforeReturn = [&] { Api->Fire(false); }; return true;
		});
		Api = F.Api.Get(); Api->Reading(); const auto R = F.Source->Produce(0);
		Check(R && R->RequiresReset() && R->GetActions() == FActionValues{}
			&& F.Source->GetLastPollStatus() == EPollStatus::Disconnected, "disconnect after commit checked at physical latch");
	}
	{
		Fixture F([](const FDeviceState& S, FActionValues& V) { V[Throttle] = S.VirtualKeys['W'] ? 255 : 0; return true; }, GameInputKindKeyboard);
		ComPtr<FakeReading> R; R.Attach(new FakeReading); GameInputKeyState Key{}; Key.virtualKey = 'W'; R->Keys.push_back(Key);
		F.Api->Responses.push_back({S_OK, R}); Check(F.Source->Produce(0)->GetActions()[Throttle] == 255, "keyboard mapping");
	}
}
static void ShutdownCases()
{
	{
		Fixture F; const auto ApiRefs = F.Api->RefCount(), DeviceRefs = F.Device->RefCount();
		F.Api->FailUnregister = true;
		Check(!F.Source->Shutdown() && F.Api->RefCount() == ApiRefs && F.Device->RefCount() == DeviceRefs, "failed unregister retains ownership");
		F.Api->Fire(false);
		Check(!F.Source->Produce(0) && !F.Source->Skip(0) && !F.Source->SetPaused(false) && F.Api->ReadCalls == 0, "failed shutdown rejects work with live callback context");
		F.Api->FailUnregister = false;
		Check(F.Source->Shutdown() && F.Source->Shutdown() && F.Api->UnregisterCalls == 2, "retry succeeds and shutdown idempotent");
		F.Source.reset(); Check(F.Api->RefCount() == 1 && F.Device->RefCount() == 1, "successful destruction releases COM ownership");
	}
	{
		Fixture F;
		// The fake dispatcher has accepted a callback but has not completed it.
		{ std::lock_guard<std::mutex> Lock(F.Api->CallbackGate); ++F.Api->Running; }
		std::atomic<bool> Finished{false}; bool Result = false;
		std::thread Shutdown([&] { Result = F.Source->Shutdown(); Finished.store(true); });
		Await([&] { return F.Api->WaitingForCallback.load(); });
		Check(!Finished.load(), "shutdown waits for callback completion");
		F.Api->Callback(1, F.Api->Context, nullptr, 0, GameInputDeviceConnected, static_cast<GameInputDeviceStatus>(0));
		{ std::lock_guard<std::mutex> Lock(F.Api->CallbackGate); --F.Api->Running; F.Api->CallbackFinished.notify_all(); }
		Shutdown.join(); Check(Result && Finished.load(), "running callback completes without Gate deadlock");
	}
	wchar_t Path[32768]{}; Check(GetModuleFileNameW(nullptr, Path, 32768) != 0, "death test executable path");
	std::wstring Command = L"\"" + std::wstring(Path) + L"\" --death";
	STARTUPINFOW Startup{}; Startup.cb = sizeof(Startup); PROCESS_INFORMATION Process{};
	Check(CreateProcessW(Path, Command.data(), nullptr, nullptr, FALSE, CREATE_NO_WINDOW, nullptr, nullptr, &Startup, &Process) != 0, "isolated death subprocess");
	Check(WaitForSingleObject(Process.hProcess, 10000) == WAIT_OBJECT_0, "bounded death subprocess completion");
	DWORD Code = 0; Check(GetExitCodeProcess(Process.hProcess, &Code) && Code == 73, "unresolved destruction invokes terminate handler");
	CloseHandle(Process.hThread); CloseHandle(Process.hProcess);
}
int main(int Argc, char**)
{
	if (Argc > 1)
	{
		std::set_terminate([] { std::_Exit(73); });
		Fixture F; F.Api->FailUnregister = true; F.Source.reset(); return 2;
	}
	ErrorPolicies(); CadenceAndLifecycle(); ShutdownCases();
	std::cout << "PASS WindowsGameInputProbe checks=" << Checks << " hardware=none sdk=v3\n";
}
