#include "WindowsGameInputFakes.h"

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
