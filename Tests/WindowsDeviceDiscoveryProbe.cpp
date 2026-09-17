#include "WindowsGameInputFakes.h"
#include "IAmSpeed/Input/Windows/GameInputDiscovery.h"
using Speed::Input::Windows::FGameInputDiscovery;
class DiscoveryApi final : public FakeApi
{
public:
	struct Event { ComPtr<FakeDevice> Device; uint64_t Stamp; bool Connected; };
	std::vector<Event> Initial;
	bool AllKinds = false;
	void FireDevice(FakeDevice* Device, uint64_t Stamp, bool Connected)
	{
		GameInputDeviceCallback Function; void* Data;
		{ std::lock_guard<std::mutex> Lock(CallbackGate); if (!Callback) return; ++Running; Function = Callback; Data = Context; }
		Function(1, Data, Device, Stamp, Connected ? GameInputDeviceConnected : static_cast<GameInputDeviceStatus>(0), static_cast<GameInputDeviceStatus>(0));
		{ std::lock_guard<std::mutex> Lock(CallbackGate); --Running; CallbackFinished.notify_all(); }
	}
	HRESULT STDMETHODCALLTYPE RegisterDeviceCallback(IGameInputDevice* Device, GameInputKind Kind, GameInputDeviceStatus Status,
		GameInputEnumerationKind Enumeration, void* Data, GameInputDeviceCallback Function, GameInputCallbackToken* Token) override
	{
		AllKinds = !Device && (Kind & GameInputKindKeyboard) && (Kind & GameInputKindGamepad)
			&& (Kind & GameInputKindMouse) && (Kind & GameInputKindRawDeviceReport)
			&& Status == GameInputDeviceConnected && Enumeration == GameInputBlockingEnumeration;
		Callback = Function; Context = Data; *Token = 1;
		for (const auto& E : Initial) FireDevice(E.Device.Get(), E.Stamp, E.Connected);
		return S_OK;
	}
};
static ComPtr<FakeDevice> MakeDevice(std::uint8_t Id, GameInputKind Kind = GameInputKindGamepad)
{
	ComPtr<FakeDevice> D; D.Attach(new FakeDevice); D->Info.supportedInput = Kind;
	std::memset(&D->Info.deviceId, Id, sizeof(D->Info.deviceId)); return D;
}
static FDeviceId Key(std::uint8_t N) { FDeviceId R{}; R.fill(N); return R; }
int main()
{
	std::array<bool, ActionCount> Digital{}; Digital[3] = true;
	for (bool Reverse : {false, true})
	{
		ComPtr<DiscoveryApi> Api; Api.Attach(new DiscoveryApi);
		auto A = MakeDevice(1, GameInputKindKeyboard); auto B = MakeDevice(2); auto Mouse = MakeDevice(3, GameInputKindMouse);
		Api->Initial = {{A, 10, true}, {B, 10, true}, {Mouse, 10, true}};
		if (Reverse) std::reverse(Api->Initial.begin(), Api->Initial.end());
		auto D = FGameInputDiscovery::Create(Api.Get(), 40, Digital);
		Check(D && Api->AllKinds && D->Snapshot().size() == 3, "blocking all-device enumeration");
		Check(D->Snapshot()[0].Id == Key(1) && D->Snapshot()[2].SupportedKinds == 0 && !D->AcquireSelected(), "sorted catalogue includes unsupported device, no default");
		Check(D->Select(std::make_pair(Key(2), EDeviceKind::Gamepad)), "explicit gamepad selection");
		const auto Old = *D->AcquireSelected(); FActionValues Held{}; Held[Throttle] = 255; Held[3] = 1;
		Check(Old.Device.Get() == B.Get() && D->Submit(Old, 1, Held), "retained selected handle and baseline");
		const auto Frame = *D->Produce(0); Check(Frame.GetEdgeCount() == 0 && Frame.GetActions() == Held, "first held baseline");
		D->Submit(Old, 2, {}); D->Submit(Old, 3, Held);
		auto Replacement = MakeDevice(2); Api->FireDevice(Replacement.Get(), 20, true);
		const auto New = *D->AcquireSelected();
		Check(New.Device.Get() == Replacement.Get() && New.Ticket.Generation != Old.Ticket.Generation && !D->Submit(Old, 4, Held), "replacement retains new object rejects old ticket");
		Api->FireDevice(B.Get(), 21, false); Api->FireDevice(B.Get(), 9, true);
		Check(D->AcquireSelected()->Ticket.Generation == New.Ticket.Generation, "old object disconnect and stale connect cannot replace current object");
		Check(D->Submit(New, 1, Held) && D->Skip(1), "new baseline survives override");
		const auto Switched = *D->Produce(2);
		Check(Switched.RequiresReset() && Switched.GetEdgeCount() == 0, "replacement purges old pending edges");
		Api->FireDevice(Replacement.Get(), 22, false);
		Check(!D->AcquireSelected() && !D->Submit(New, 2, Held) && D->Produce(3)->GetActions() == FActionValues{}, "removal neutralizes immediately");
		Api->FireDevice(Replacement.Get(), 23, true); const auto Reconnected = *D->AcquireSelected();
		Check(D->Submit(Reconnected, 1, Held) && D->Produce(4)->GetEdgeCount() == 0, "reconnection fresh held");
		Check(D->Produce(0)->GetActions() == Frame.GetActions(), "old immutable frame preserved");
		Check(Api->ReadCalls == 0, "discovery never polls hardware");
		Api->FailUnregister = true; const auto Refs = Replacement->RefCount();
		Check(!D->Shutdown() && Replacement->RefCount() == Refs && !D->AcquireSelected(), "unregister failure retains references, closes selection");
		Api->FireDevice(Replacement.Get(), 24, false);
		Check(!D->Submit(Reconnected, 2, Held) && !D->Produce(5), "no operations after shutdown begins");
		Api->FailUnregister = false; Check(D->Shutdown() && D->Shutdown() && Api->UnregisterCalls == 2, "retry/idempotent shutdown");
	}
	{
		ComPtr<DiscoveryApi> Api; Api.Attach(new DiscoveryApi); auto A = MakeDevice(1);
		Api->Initial = {{A, 1, true}}; auto D = FGameInputDiscovery::Create(Api.Get(), 41, Digital);
		D->Select(std::make_pair(Key(1), EDeviceKind::Gamepad));
		Api->FireDevice(A.Get(), 1, false);
		Check(D->GetLastError() == E_UNEXPECTED && !D->AcquireSelected(), "timestamp ambiguity fails closed");
		Check(D->Produce(0)->RequiresReset() && D->Produce(1)->GetActions() == FActionValues{}, "discovery failure neutral cadence");
	}
	{
		ComPtr<DiscoveryApi> Api; Api.Attach(new DiscoveryApi); auto A = MakeDevice(1);
		auto D = FGameInputDiscovery::Create(Api.Get(), 42, Digital);
		{ std::lock_guard<std::mutex> Lock(Api->CallbackGate); ++Api->Running; }
		std::atomic<bool> Finished{false}; bool Result = false;
		std::thread Stop([&] { Result = D->Shutdown(); Finished.store(true); });
		Await([&] { return Api->WaitingForCallback.load(); });
		Check(!Finished.load(), "shutdown waits for in-flight callback");
		Api->Callback(1, Api->Context, A.Get(), 1, GameInputDeviceConnected, static_cast<GameInputDeviceStatus>(0));
		{ std::lock_guard<std::mutex> Lock(Api->CallbackGate); --Api->Running; Api->CallbackFinished.notify_all(); }
		Stop.join(); Check(Result && Finished.load(), "callback exits without shutdown deadlock");
	}
	std::cout << "PASS WindowsDeviceDiscoveryProbe checks=" << Checks << " hardware=none sdk=v3\n";
}
