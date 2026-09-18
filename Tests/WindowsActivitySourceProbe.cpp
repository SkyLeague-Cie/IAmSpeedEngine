#include "WindowsDiscoveryFakes.h"
#include "IAmSpeed/Input/Windows/GameInputSelectedSource.h"
using Speed::Input::Windows::FGameInputSelectedSource;
class HistoryApi final : public DiscoveryApi
{
public:
	std::map<IGameInputDevice*, std::vector<ComPtr<FakeReading>>> History;
	std::map<IGameInputDevice*, bool> ReadDisconnected;
	std::map<IGameInputDevice*, unsigned> DeviceCalls;
	void Sample(FakeDevice* D, std::uint64_t Stamp, bool Held, float Stick = 0)
	{
		ComPtr<FakeReading> R; R.Attach(new FakeReading); R->Stamp = Stamp;
		if (D->Info.supportedInput & GameInputKindKeyboard)
		{ if (Held) { GameInputKeyState K{}; K.virtualKey = 'W'; R->Keys.push_back(K); } }
		else { R->Pad.buttons = Held ? GameInputGamepadA : GameInputGamepadNone; R->Pad.leftThumbstickX = Stick; R->Pad.rightTrigger = Held ? 1.f : 0.f; }
		History[D].push_back(R);
	}
	HRESULT STDMETHODCALLTYPE GetCurrentReading(GameInputKind, IGameInputDevice* D, IGameInputReading** Out) override
	{
		++ReadCalls; ++CurrentCalls; *Out = nullptr;
		++DeviceCalls[D]; if (ReadDisconnected[D]) return GAMEINPUT_E_DEVICE_DISCONNECTED;
		const auto& H = History[D]; if (H.empty()) return GAMEINPUT_E_READING_NOT_FOUND;
		*Out = H.back().Get(); (*Out)->AddRef(); return S_OK;
	}
	HRESULT STDMETHODCALLTYPE GetNextReading(IGameInputReading* Previous, GameInputKind, IGameInputDevice* D, IGameInputReading** Out) override
	{
		++ReadCalls; ++NextCalls; *Out = nullptr;
		++DeviceCalls[D]; if (ReadDisconnected[D]) return GAMEINPUT_E_DEVICE_DISCONNECTED;
		const auto& H = History[D];
		for (std::size_t I = 0; I < H.size(); ++I) if (H[I].Get() == Previous)
		{
			if (I + 1 == H.size()) return GAMEINPUT_E_READING_NOT_FOUND;
			*Out = H[I+1].Get(); (*Out)->AddRef(); return S_OK;
		}
		return GAMEINPUT_E_REFERENCE_READING_TOO_OLD;
	}
};
static FActivityConfig Config() { return {{.20f,.15f,.10f},{.08f,.05f,.04f},0,8,EDeviceKind::Gamepad}; }
static std::unique_ptr<FGameInputSelectedSource> Source(HistoryApi* Api)
{
	std::array<bool,ActionCount> Digital{}; Digital[3]=true;
	return FGameInputSelectedSource::CreateAutomatic(Api,601,Digital,
		[](const FDeviceState& S,FActionValues& V) { V[Brake]=S.VirtualKeys['W']?255:0; V[3]=S.VirtualKeys['W']?1:0; return true; },
		[](const FDeviceState& S,FActionValues& V) { V[Throttle]=(S.GamepadButtons&GameInputGamepadA)?255:0; V[3]=(S.GamepadButtons&GameInputGamepadA)?1:0; return true; },Config());
}
int main()
{
	for (bool Reverse : {false,true})
	{
		ComPtr<HistoryApi> Api; Api.Attach(new HistoryApi);
		auto Pad=MakeDevice(1); auto Keyboard=MakeDevice(2,GameInputKindKeyboard);
		Api->Initial={{Pad,1,true},{Keyboard,1,true}}; if(Reverse) std::reverse(Api->Initial.begin(),Api->Initial.end());
		Api->Sample(Pad.Get(),1,false); Api->Sample(Keyboard.Get(),1,false);
		auto S=Source(Api.Get()); Check(bool(S),"auto factory explicit configuration");
		Check(S->Produce(0)->GetActions()==FActionValues{},"initial neutral baselines do not choose source");
		Api->Sample(Pad.Get(),2,true); Api->Sample(Keyboard.Get(),2,true);
		const auto A=*S->Produce(1);
		Check(A.GetActions()[Throttle]==255 && A.GetActions()[Brake]==0 && A.GetEdgeCount()==0 && A.RequiresReset(),"simultaneous first activity stable ID, exclusive fresh baseline");
		Api->Sample(Pad.Get(),3,false); Api->Sample(Pad.Get(),4,true);
		Check(S->Produce(2)->GetEdgeCount()==2,"selected traversal retains subsequent true stop/start edges");
		Api->Sample(Keyboard.Get(),5,false); Check(S->Produce(3)->GetActions()[Throttle]==255,"release cannot steal");
		Api->Sample(Keyboard.Get(),6,true); const auto B=*S->Produce(4);
		Check(B.RequiresReset() && B.GetActions()[Brake]==255 && B.GetActions()[Throttle]==0 && B.GetEdgeCount()==0,"keyboard activity switches only fresh held with old edges purged");
		const auto Calls=Api->ReadCalls.load();
		S->RequestLock(Key(1)); Check(S->Produce(1)->GetActions()==A.GetActions() && Api->ReadCalls==Calls,"replay does not poll or apply queued lock");
		const auto Locked=*S->Produce(5); Check(Locked.RequiresReset() && Locked.GetActions()[Throttle]==255 && Locked.GetEdgeCount()==0,"device ID lock applies at forward frame");
		Api->Sample(Keyboard.Get(),7,false); Api->Sample(Keyboard.Get(),8,true);
		Check(S->Produce(6)->GetActions()[Throttle]==255,"locked device ignores newer keyboard activity");
		Api->FireDevice(Pad.Get(),9,false); Check(S->Produce(7)->GetActions()==FActionValues{},"locked disconnect fails neutral without fallback");
		Api->FireDevice(Pad.Get(),10,true); Api->Sample(Pad.Get(),10,true);
		const auto Reconnected=*S->Produce(8); Check(Reconnected.RequiresReset() && Reconnected.GetActions()[Throttle]==255 && Reconnected.GetEdgeCount()==0,"locked reconnect immediate fresh held");
		Check(S->SetPaused(true) && S->Produce(9)->GetActions()==FActionValues{},"pause neutral");
		const auto PausedCalls=Api->ReadCalls.load(); S->Produce(10); Check(Api->ReadCalls==PausedCalls,"paused polling stops");
		S->SetPaused(false); Api->Sample(Pad.Get(),11,true); const auto Resume=*S->Produce(11);
		Check(Resume.GetActions()[Throttle]==255 && Resume.RequiresReset() && Resume.GetEdgeCount()==0,"resume held no replay edges");
		S->RequestLock(std::nullopt); Check(S->Produce(12)->GetActions()[Throttle]==255,"unlock drops suppressed historical activity");
		Api->Sample(Keyboard.Get(),12,false); Api->Sample(Keyboard.Get(),13,true);
		Check(S->Skip(13),"automatic switch during override");
		const auto New=*S->Produce(14); Check(New.RequiresReset() && New.GetActions()[Brake]==255 && New.GetEdgeCount()==0,"auto switch reset survives override");
		Api->Sample(Pad.Get(),14,false,.01f); Api->Sample(Pad.Get(),15,false,.14f); Api->Sample(Pad.Get(),16,false,.19f);
		Check(S->Produce(15)->GetActions()[Brake]==255,"raw stick drift and neutral reports cannot reclaim");
		Api->Sample(Pad.Get(),17,false,.4f); Check(S->Produce(16)->GetActions()==FActionValues{} && S->Produce(16)->RequiresReset(),"meaningful raw analog movement selects pad even when mapped values neutral");
		Check(!S->RequestSelection(std::make_pair(Key(2),EDeviceKind::Keyboard)),"automatic source does not mix explicit selection control");
		Check(S->Shutdown(),"auto shutdown");
	}
	{
		ComPtr<HistoryApi> Api; Api.Attach(new HistoryApi); auto Pad=MakeDevice(1);
		Api->Initial={{Pad,1,true}}; Api->Sample(Pad.Get(),1,true); auto S=Source(Api.Get());
		S->RequestLock(Key(1)); Check(S->Produce(0)->GetActions()[Throttle]==255,"locked startup fresh held");
		Api->ReadDisconnected[Pad.Get()]=true;
		Check(S->Produce(1)->GetActions()==FActionValues{},"activity read-side disconnect neutralizes without callback");
		const auto Calls=Api->DeviceCalls[Pad.Get()]; S->SetPaused(true); S->Produce(2); S->SetPaused(false); S->Produce(3);
		Check(Api->DeviceCalls[Pad.Get()]==Calls,"read-side disconnect quarantine survives pause/resume");
		Api->ReadDisconnected[Pad.Get()]=false; Api->FireDevice(Pad.Get(),2,true); Api->Sample(Pad.Get(),2,true);
		Check(S->Produce(4)->GetActions()[Throttle]==255,"new lifecycle revision clears quarantine");
	}
	std::cout<<"PASS WindowsActivitySourceProbe checks="<<Checks<<" hardware=none sdk=v3\n";
}
