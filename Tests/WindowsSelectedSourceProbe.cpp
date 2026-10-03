#include "WindowsDiscoveryFakes.h"
#include "IAmSpeed/Input/Windows/GameInputSelectedSource.h"
#include "IAmSpeed/Input/Windows/GameInputCanonicalControls.h"
#include "IAmSpeed/Input/Windows/GameInputRawAcquisition.h"
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
	SelectedFixture(FGameInputAcquisition::FMapper PadMapper = {}, bool Raw = false)
	{
		Api.Attach(new SelectedApi); Pad = MakeDevice(1); Keyboard = MakeDevice(2, GameInputKindKeyboard);
		Api->Initial = {{Pad, 1, true}, {Keyboard, 1, true}};
		std::array<bool, ActionCount> Digital{}; Digital[3] = true;
		if (Raw) { Source = FGameInputSelectedSource::CreateRaw(Api.Get(), 501); Check(bool(Source), "raw factory"); return; }
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
		using namespace Speed::Input::Windows;
		SelectedFixture F({}, true); // No selected device is a rejected sink, not an SDK read failure.
		auto Hub = std::make_shared<Speed::Input::V2::FRawAcquisitionJournal>(9);
		auto* Selection = F.Source.get();
		FGameInputRawAcquisition Owner(std::move(F.Source), Hub);
		Check(Owner.Pump() == ERawPumpResult::Installed, "unselected startup publishes one neutral baseline");
		const auto Initial = Hub->ReadControlsSince({9, 0});
		Check(Initial.Status == Speed::Input::V2::EAcquisitionRead::Batch && Initial.Next.Serial == 1
			&& Initial.ControlBarrier == 0 && !Owner.TakeNeutralizeDiagnostic(),
			"initial no-ticket is a valid neutral baseline without an invalidation barrier");
		Check(Owner.Pump() == ERawPumpResult::NoChange
			&& Hub->ReadControlsSince({9, 1}).Status == Speed::Input::V2::EAcquisitionRead::NoChange,
			"idle no-ticket does not repeat a neutral baseline or invent control requests");
		Check(Hub->SetLifecyclePaused(true) == Speed::Input::V2::ELifecycleResult::Applied
			&& Hub->RequestFreshResume(), "fresh resume requested while no device is selected");
		Check(Owner.Pump() == ERawPumpResult::NoChange && !Hub->IsResumeReady()
			&& Hub->NeedsFreshResume(), "no-ticket neutral cannot masquerade as fresh resumed device reading");
		Check(Selection->RequestSelection(std::make_pair(Key(1), EDeviceKind::Gamepad)),
			"select device after neutral pause");
		F.Api->Pad(F.Pad.Get(), 1, true, 10);
		Check(Owner.Pump() == ERawPumpResult::Installed && Hub->IsResumeReady()
			&& Hub->SetLifecyclePaused(false) == Speed::Input::V2::ELifecycleResult::Applied,
			"fresh held reading releases pause on first device acquisition");
		const auto Resumed = Hub->Poll(0);
		Check(Resumed && Resumed->FinalState[0].Value == 1,
			"held input applies on first resumed physical poll");
	}
	{
		using namespace Speed::Input::Windows;
		SelectedFixture F({}, true);
		auto* Selection = F.Source.get();
		auto Hub = std::make_shared<Speed::Input::V2::FRawAcquisitionJournal>(7);
		FGameInputRawAcquisition Owner(std::move(F.Source), Hub);
		Check(Hub->SetLifecyclePaused(true) == Speed::Input::V2::ELifecycleResult::Applied
			&& Hub->RequestFreshResume(), "resume requested before first acquisition");
		Check(Owner.Pump() == ERawPumpResult::NoChange && !Hub->IsResumeReady()
			&& Hub->NeedsFreshResume()
			&& Hub->ReadControlsSince({7, 0}).Status == Speed::Input::V2::EAcquisitionRead::NoChange,
			"pre-first-poll no-ticket never fakes a fresh resumed reading");
		Check(Selection->RequestSelection(std::make_pair(Key(1), EDeviceKind::Gamepad)),
			"select device after pre-first-poll pause");
		F.Api->Pad(F.Pad.Get(), 1, true, 11);
		Check(Owner.Pump() == ERawPumpResult::Installed && Hub->IsResumeReady()
			&& Hub->SetLifecyclePaused(false) == Speed::Input::V2::ELifecycleResult::Applied,
			"only actual held device reading makes pre-first-poll resume ready");
		const auto Resumed = Hub->Poll(0);
		Check(Resumed && Resumed->FinalState[0].Value == 1,
			"first physical poll after pre-first-poll pause receives held value");
	}
	{
		using namespace Speed::Input::Windows;
		SelectedFixture F({}, true); F.SelectPad();
		auto* Selection = F.Source.get();
		auto Hub = std::make_shared<Speed::Input::V2::FRawAcquisitionJournal>(8);
		FGameInputRawAcquisition Owner(std::move(F.Source), Hub);
		F.Api->Pad(F.Pad.Get(), 1, true, 1);
		Check(Owner.Pump() == ERawPumpResult::Installed, "selected held device publishes an active baseline");
		const auto Active = Hub->Poll(0);
		Check(Active && Active->FinalState[0].Value == 1, "active physical snapshot contains held input");
		Check(Selection->RequestSelection(std::nullopt), "explicitly drop selected device");
		Check(Owner.Pump() == ERawPumpResult::Neutralized, "active ticket loss publishes neutral baseline");
		const auto Lost = Owner.TakeNeutralizeDiagnostic();
		const auto History = Hub->ReadControlsSince({8, 1});
		Check(Lost && Lost->RawPollReject == FGameInputSelectedSource::ERawPollReject::Sink
			&& Lost->SinkReject == FGameInputRawAcquisition::ESinkReject::NoTicket
			&& History.Status == Speed::Input::V2::EAcquisitionRead::Gap
			&& History.ControlBarrier == 2,
			"real selected-device loss keeps the invalidation barrier");
		const auto Neutral = Hub->Poll(1);
		Check(Neutral && Neutral->FinalState[0].Value == 0 && Neutral->Changes.empty(),
			"ticket loss clears held physical input without synthesizing an edge");
		Check(Owner.Pump() == ERawPumpResult::NoChange
			&& Hub->ReadControlsSince({8, 2}).Status == Speed::Input::V2::EAcquisitionRead::NoChange,
			"continued absence after neutralization adds no repeated barrier");
	}
	{
		using namespace Speed::Input::Windows;
		using Speed::Input::V2::FRawAcquisitionJournal;
		using Speed::Input::V2::ELifecycleResult;
		SelectedFixture F({}, true); F.SelectPad();
		auto Hub = std::make_shared<FRawAcquisitionJournal>(10);
		FGameInputRawAcquisition Owner(std::move(F.Source), Hub);
		F.Api->Pad(F.Pad.Get(), 0, false, 1); F.Api->Pad(F.Pad.Get(), 1, true, 2); F.Api->Pad(F.Pad.Get(), 0, false, 3);
		Check(Owner.Pump() == ERawPumpResult::Installed, "real selected cursor through canonical acquisition hub");
		const auto First = Hub->Poll(0); const auto Next = Hub->Poll(1);
		Check(First && First->Status == Speed::Input::V2::ERawSampleStatus::Resync
			&& Next && Next->Changes.size() == 4 && Next->FinalState[0].Value == 0, "OS batch short button and trigger retained across baseline");
		const auto Calls = F.Api->ReadCalls.load();
		ERawPumpResult OtherResult{}; std::thread Other([&] { OtherResult = Owner.Pump(); }); Other.join();
		Check(OtherResult == ERawPumpResult::Rejected && F.Api->ReadCalls == Calls, "second polling owner refused without OS access");
		Check(Hub->SetLifecyclePaused(true) == ELifecycleResult::Applied, "physical delivery paused independently");
		F.Api->Pad(F.Pad.Get(), 1, true, 4); F.Api->Pad(F.Pad.Get(), 0, false, 5);
		Check(Owner.Pump() == ERawPumpResult::Installed && !Hub->Poll(2)
			&& Hub->ReadControlsSince({10, 3}).Readings.size() == 2, "pause leaves acquisition and controls active");
		Check(Hub->RequestFreshResume(), "resume request");
		const auto CurrentCalls = F.Api->CurrentCalls.load();
		F.Api->Pad(F.Pad.Get(), 1, true, 6);
		Check(Owner.Pump() == ERawPumpResult::Installed && F.Api->CurrentCalls == CurrentCalls + 1
			&& Hub->IsResumeReady(), "resume actually calls current reading");
		Check(Hub->SetLifecyclePaused(false) == ELifecycleResult::Applied, "fresh resume gate");
		const auto Resume = Hub->Poll(2);
		Check(Resume && Resume->Changes.empty() && Resume->FinalState[0].Value == 1, "resume held without replay");
		F.Api->BeforeReturn = [&] { F.Api->FireDevice(F.Pad.Get(), 2, false); };
		Check(Owner.Pump() == ERawPumpResult::Neutralized, "hotplug invalidation reaches hub");
		const auto HotplugDiagnostic = Owner.TakeNeutralizeDiagnostic();
		Check(HotplugDiagnostic && HotplugDiagnostic->RawPollReject == FGameInputSelectedSource::ERawPollReject::CommitGate
			&& HotplugDiagnostic->SinkReject == FGameInputRawAcquisition::ESinkReject::None,
			"hotplug admission fence is distinguished from SDK reading and sink rejection");
		const auto Neutral = Hub->Poll(3);
		Check(Neutral && Neutral->Changes.empty() && Neutral->FinalState[0].Value == 0, "disconnect delivers neutral reset");
		F.Api->FireDevice(F.Pad.Get(), 3, true); F.Api->Pad(F.Pad.Get(), 1, true, 7);
		Check(Owner.Pump() == ERawPumpResult::Installed && Hub->Poll(4)->FinalState[0].Value == 1, "reconnect fresh held");
		Check(Owner.Close() && Owner.Close() && Owner.Pump() == ERawPumpResult::Closed, "acquisition close prevents future reads");
	}
	{
		using namespace Speed::Input::Windows;
		SelectedFixture F({}, true); F.SelectPad();
		FGameInputSelectedSource::FSelectedRawBatch Captured;
		unsigned Installs = 0;
		auto Sink = [&](const auto& Batch) noexcept { Captured = Batch; ++Installs; return true; };
		F.Api->Pad(F.Pad.Get(), 1, true, 100);
		Check(F.Source->PollRaw(1, Sink) && Captured.Readings.Count == 1 && Captured.Readings.FreshBaseline, "raw admitted fresh ticket");
		const auto Generation = Captured.Ticket->Generation;
		const auto Calls = F.Api->ReadCalls.load();
		Check(!F.Source->Produce(0) && !F.Source->Skip(0) && !F.Source->PollRaw(1, Sink)
			&& F.Api->ReadCalls == Calls && Installs == 1, "raw ownership rejects legacy and repeated clock");
		F.Api->Pad(F.Pad.Get(), 0, false, 101); F.Api->Pad(F.Pad.Get(), 1, true, 102);
		Check(F.Source->PollRaw(2, Sink) && Captured.Readings.Count == 2 && !Captured.Readings.FreshBaseline, "raw repeated transitions retained");
		F.Api->BeforeReturn = [&] { F.Api->FireDevice(F.Pad.Get(), 2, false); };
		F.Api->Pad(F.Pad.Get(), 1, false, 103);
		Check(!F.Source->PollRaw(3, Sink) && Installs == 2, "hotplug mid-read rejects complete raw batch");
		Check(F.Source->PollRaw(4, Sink) && !Captured.Ticket && Captured.Readings.Count == 0, "disconnected admission has no stale payload");
		F.Api->FireDevice(F.Pad.Get(), 3, true); F.Api->Pad(F.Pad.Get(), .5f, true, 200);
		Check(F.Source->PollRaw(5, Sink) && Captured.Ticket->Generation > Generation && Captured.Readings.FreshBaseline, "reconnect renews raw ticket and baseline");
		FCanonicalDeviceState Canonical;
		Check(Canonicalize(Captured.Readings.States[0], Speed::Input::V2::ERawDeviceKind::Gamepad, Canonical)
			&& Canonical.Count == 20 && Canonical.Values[0].Value == 1 && Canonical.Values[19].Value == .5f, "standard gamepad canonical mapping");
		const unsigned BeforeOverflow = Installs;
		for (unsigned I = 0; I < 65; ++I) F.Api->Pad(F.Pad.Get(), 1, I % 2 == 0, 300 + I);
		Check(!F.Source->PollRaw(6, Sink) && Installs == BeforeOverflow
			&& F.Source->GetLastRawPollReject() == FGameInputSelectedSource::ERawPollReject::ReadBatch,
			"SDK read overflow is distinguished from sink/commit rejection");
		F.Api->Pad(F.Pad.Get(), 0, false, 400);
		Check(F.Source->PollRaw(7, Sink) && Captured.Readings.Count == 1 && Captured.Readings.FreshBaseline, "overflow recovers only from fresh reading");
		Check(F.Source->SetPaused(true) && !F.Source->PollRaw(8, Sink) && F.Source->SetPaused(false), "acquisition lifetime pause fences calls");
		F.Api->Pad(F.Pad.Get(), 1, true, 500);
		Check(F.Source->PollRaw(9, Sink) && Captured.Readings.FreshBaseline, "acquisition resume fresh state");
		F.Api->Pad(F.Pad.Get(), 0, false, 501);
		Check(!F.Source->PollRaw(10, [](const auto&) noexcept { return false; }), "sink refusal is propagated");
		F.Api->Pad(F.Pad.Get(), 1, true, 502);
		Check(F.Source->PollRaw(11, Sink) && Captured.Readings.FreshBaseline, "sink refusal forces fresh generation");
		FDeviceState Keys; Keys.KeyCount = 5; Keys.ScanCodes = {0x11, 0x43, 0xe01d, 0x1d, 0xe036};
		Check(KeyboardUsage(0x45) == 0x48 && KeyboardUsage(0xe045) == 0x53
			&& KeyboardUsage(0xe11d45) == 0x48 && KeyboardUsage(0xe046) == 0x48
			&& KeyboardUsage(0xe11d) == 0, "Pause versus NumLock and unsupported prefix are distinct");
		Check(Canonicalize(Keys, Speed::Input::V2::ERawDeviceKind::Keyboard, Canonical), "keyboard canonical conversion");
		for (const auto Usage : {0x1a, 0x42, 0xe4, 0xe0, 0xe5})
		{
			bool Found = false;
			for (std::size_t I = 0; I < Canonical.Count; ++I)
				if (Canonical.Values[I].Control.Code == Usage) Found = Canonical.Values[I].Value == 1;
			Check(Found, "physical W/F9 and independent modifiers map to HID");
		}
		for (std::size_t I = 0; I < Canonical.Count; ++I)
			Check(Canonical.Values[I].Control.Accepts(Canonical.Values[I].Value)
				&& (!I || Canonical.Values[I - 1].Control < Canonical.Values[I].Control), "canonical keyboard capabilities sorted unique valid");
		FDeviceState Invalid; Invalid.Axes[4] = std::numeric_limits<float>::quiet_NaN();
		Check(!Canonicalize(Invalid, Speed::Input::V2::ERawDeviceKind::Gamepad, Canonical) && !Canonical.Count, "invalid analog state clears complete output");
	}
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
