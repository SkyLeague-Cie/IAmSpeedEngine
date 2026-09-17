#include "WindowsDiscoveryFakes.h"
#include "IAmSpeed/Input/Windows/GameInputSelectedSource.h"
using Speed::Input::Windows::FGameInputSelectedSource;
class SelectedApi final : public DiscoveryApi
{
public:
	std::map<IGameInputDevice*, std::deque<Response>> DeviceResponses;
	std::vector<IGameInputDevice*> Polled;
	std::vector<bool> WasCurrent;
	void Pad(FakeDevice* Device, float ThrottleValue, bool Held, uint64_t Stamp)
	{
		ComPtr<FakeReading> R; R.Attach(new FakeReading);
		R->Pad.rightTrigger = ThrottleValue; R->Pad.buttons = Held ? GameInputGamepadA : GameInputGamepadNone;
		R->Stamp = Stamp; DeviceResponses[Device].push_back({S_OK, R});
	}
	void KeyHeld(FakeDevice* Device)
	{
		ComPtr<FakeReading> R; R.Attach(new FakeReading); GameInputKeyState K{}; K.virtualKey = 'W'; R->Keys.push_back(K);
		DeviceResponses[Device].push_back({S_OK, R});
	}
	HRESULT Read(IGameInputDevice* Device, bool Current, IGameInputReading** Out)
	{
		Polled.push_back(Device); WasCurrent.push_back(Current);
		Responses.swap(DeviceResponses[Device]); const auto Result = Pop(Out); Responses.swap(DeviceResponses[Device]);
		return Result;
	}
	HRESULT STDMETHODCALLTYPE GetCurrentReading(GameInputKind, IGameInputDevice* Device, IGameInputReading** Out) override
	{ ++CurrentCalls; return Read(Device, true, Out); }
	HRESULT STDMETHODCALLTYPE GetNextReading(IGameInputReading*, GameInputKind, IGameInputDevice* Device, IGameInputReading** Out) override
	{ ++NextCalls; return Read(Device, false, Out); }
};
struct SelectedFixture
{
	ComPtr<SelectedApi> Api;
	ComPtr<FakeDevice> Pad, Keyboard;
	std::unique_ptr<FGameInputSelectedSource> Source;
	SelectedFixture(FGameInputAcquisition::FMapper PadMapper = {})
	{
		Api.Attach(new SelectedApi); Pad = MakeDevice(1); Keyboard = MakeDevice(2, GameInputKindKeyboard);
		Api->Initial = {{Pad, 1, true}, {Keyboard, 1, true}};
		std::array<bool, ActionCount> Digital{}; Digital[3] = true;
		if (!PadMapper) PadMapper = [](const FDeviceState& S, FActionValues& V)
		{
			const auto Axis = QuantizeAxis(S.Axes[1], false); if (!Axis) return false;
			V[Throttle] = *Axis; V[3] = (S.GamepadButtons & GameInputGamepadA) ? 1 : 0; return true;
		};
		Source = FGameInputSelectedSource::Create(Api.Get(), 501, Digital,
			[](const FDeviceState& S, FActionValues& V) { V[Brake] = S.VirtualKeys['W'] ? 255 : 0; return true; }, PadMapper);
		Check(bool(Source), "selected factory");
	}
	void SelectPad() { Check(Source->RequestSelection(std::make_pair(Key(1), EDeviceKind::Gamepad)), "queue pad selection"); }
};
int main()
{
	{
		SelectedFixture F;
		Check(F.Source->Produce(0)->GetActions() == FActionValues{} && F.Api->ReadCalls == 0, "no implicit source");
		F.SelectPad(); F.Api->Pad(F.Pad.Get(), 1, true, 100);
		Check(F.Api->ReadCalls == 0 && F.Source->Produce(0)->GetActions() == FActionValues{}, "request and replay do not poll or switch");
		const auto A = *F.Source->Produce(1);
		Check(A.RequiresReset() && A.GetActions()[Throttle] == 255 && A.GetActions()[3] == 1 && A.GetEdgeCount() == 0, "selected fresh baseline");
		F.Api->Pad(F.Pad.Get(), 1, false, 101); F.Api->Pad(F.Pad.Get(), 1, true, 102);
		const auto B = *F.Source->Produce(2);
		Check(B.GetEdgeCount() == 2 && B.GetEdges()[0].Kind == EEdgeKind::Stop && B.GetEdges()[1].Kind == EEdgeKind::Start, "normal traversal preserves edges");
		const auto Calls = F.Api->ReadCalls.load();
		F.Source->RequestSelection(std::make_pair(Key(2), EDeviceKind::Keyboard));
		Check(!F.Source->Produce(4) && F.Api->ReadCalls == Calls && F.Source->Produce(1)->GetActions() == A.GetActions(), "gap and replay do not apply pending request");
		F.Api->Pad(F.Pad.Get(), 0, false, 103); F.Api->KeyHeld(F.Keyboard.Get());
		const auto C = *F.Source->Produce(3);
		Check(C.RequiresReset() && C.GetActions()[Throttle] == 0 && C.GetActions()[Brake] == 255 && C.GetEdgeCount() == 0, "switch publishes exactly one source fresh values");
		Check(F.Api->DeviceResponses[F.Pad.Get()].size() == 1 && F.Api->Polled[Calls] == F.Keyboard.Get() && F.Api->WasCurrent[Calls], "switch does not drain old device and resets raw cursor");
		Check(F.Source->SetPaused(true), "pause"); const auto BeforePause = F.Api->ReadCalls.load();
		Check(F.Source->Produce(4)->GetActions() == FActionValues{} && F.Api->ReadCalls == BeforePause, "pause neutral no OS");
		F.Source->SetPaused(false); F.Api->KeyHeld(F.Keyboard.Get());
		Check(F.Source->Produce(5)->GetActions()[Brake] == 255, "resume fresh held");
		F.Source->RequestSelection(std::nullopt); Check(F.Source->Skip(6), "deselect via override");
		Check(F.Source->Produce(7)->RequiresReset() && F.Source->Produce(7)->GetActions() == FActionValues{}, "deselection reset survives skip");
	}
	{
		SelectedApi* Api = nullptr; FakeDevice* Pad = nullptr;
		SelectedFixture F([&](const FDeviceState&, FActionValues& V) { V[Throttle] = 255; Api->FireDevice(Pad, 2, false); return true; });
		Api = F.Api.Get(); Pad = F.Pad.Get(); F.SelectPad(); Api->Pad(Pad, 1, true, 100);
		Check(F.Source->Produce(0)->GetActions() == FActionValues{}, "disconnect during mapping cannot commit stale data");
	}
	{
		SelectedFixture F; F.SelectPad(); F.Api->Pad(F.Pad.Get(), 1, true, 100); F.Source->Produce(0);
		F.Api->BeforeReturn = [&] { F.Api->FireDevice(F.Pad.Get(), 2, false); };
		Check(F.Source->Produce(1)->GetActions() == FActionValues{}, "disconnect at traversal end neutralizes final latch");
		F.Api->FireDevice(F.Pad.Get(), 3, true); F.Api->Pad(F.Pad.Get(), 1, true, 200);
		const auto R = *F.Source->Produce(2); Check(R.RequiresReset() && R.GetActions()[3] == 1 && R.GetEdgeCount() == 0, "reconnect no synthetic edge");
		auto Replacement = MakeDevice(1); F.Api->FireDevice(Replacement.Get(), 4, true); F.Api->Pad(Replacement.Get(), .5f, true, 201);
		Check(F.Source->Produce(3)->GetActions()[Throttle] == 128, "same ID replacement polls new retained object");
	}
	for (HRESULT Error : {GAMEINPUT_E_REFERENCE_READING_TOO_OLD, GAMEINPUT_E_DEVICE_DISCONNECTED, E_FAIL})
	{
		SelectedFixture F; F.SelectPad(); F.Api->Pad(F.Pad.Get(), 1, true, 100); F.Source->Produce(0);
		F.Api->DeviceResponses[F.Pad.Get()].push_back({Error, {}});
		Check(F.Source->Skip(1) && F.Source->Skip(2), "error through suppressed frames");
		const auto N = *F.Source->Produce(3); Check(N.RequiresReset() && N.GetActions() == FActionValues{}, "error neutral reset reaches real consumption");
		const auto Calls = F.Api->ReadCalls.load(); F.Source->Produce(4);
		if (Error != GAMEINPUT_E_REFERENCE_READING_TOO_OLD)
			Check(F.Api->ReadCalls == Calls && F.Source->GetLastError() == Error, "disconnect/fatal stop OS and preserve diagnostics");
		if (Error == GAMEINPUT_E_DEVICE_DISCONNECTED)
		{
			F.Api->FireDevice(F.Pad.Get(), 2, true); F.Api->Pad(F.Pad.Get(), 1, true, 200);
			Check(F.Source->Produce(5)->GetActions()[Throttle] == 255, "read disconnect waits for new lifecycle revision");
		}
	}
	for (unsigned Count : {64u, 65u})
	{
		SelectedFixture F; F.SelectPad(); for (unsigned I = 0; I < Count; ++I) F.Api->Pad(F.Pad.Get(), 1, false, 100);
		const auto R = *F.Source->Produce(0);
		Check(F.Api->ReadCalls == 65 && R.GetActions()[Throttle] == (Count == 64 ? 255 : 0), "shared traversal64/65 budget");
	}
	{
		SelectedFixture F; F.Api->FailUnregister = true;
		Check(!F.Source->Shutdown() && !F.Source->Produce(0) && !F.Source->RequestSelection(std::nullopt), "failed shutdown retains closed source");
		F.Api->FailUnregister = false; Check(F.Source->Shutdown() && F.Source->Shutdown(), "shutdown retry idempotent");
	}
	std::cout << "PASS WindowsSelectedSourceProbe checks=" << Checks << " hardware=none sdk=v3\n";
}
