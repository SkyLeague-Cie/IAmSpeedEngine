#include "WindowsDiscoveryFakes.h"
#include "IAmSpeed/Input/Windows/GameInputWarmContext.h"
#include "IAmSpeed/Input/Windows/GameInputSelectedSource.h"
#include "IAmSpeed/Input/Windows/GameInputRawAcquisition.h"
using Speed::Input::Windows::FGameInputSelectedSource;
class HistoryApi final : public DiscoveryApi
{
public:
	std::map<IGameInputDevice*, std::vector<ComPtr<FakeReading>>> History;
	std::map<IGameInputDevice*, bool> ReadDisconnected;
	std::map<IGameInputDevice*, unsigned> DeviceCalls;
	void Sample(FakeDevice* D, std::uint64_t Stamp, bool Held, float Stick = 0)
	{
		ComPtr<FakeReading> R; R.Attach(new FakeReading); R->Stamp = Stamp; R->ObservedDevice = D;
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
// Two independent backend catalogues see the same two fake devices. These
// are authored API fakes: this proves routing, never two real physical pads.
static void RunRoutes(bool UseWarm)
{
 using namespace Speed::Input::Windows;
 using namespace Speed::Input::V2;
 ComPtr<HistoryApi> ApiA,ApiB;ApiA.Attach(new HistoryApi);ApiB.Attach(new HistoryApi);
 auto Pad1=MakeDevice(1),Pad2=MakeDevice(2),Other=MakeDevice(3);
 ApiA->Initial={{Pad1,1,true},{Pad2,1,true},{Other,1,true}};ApiB->Initial=ApiA->Initial;
 FGameInputWarmContext WarmA,WarmB;
 auto A=UseWarm?FGameInputSelectedSource::CreateRaw(WarmA.GetOrCreate(ApiA.Get()),1001):FGameInputSelectedSource::CreateRaw(ApiA.Get(),1001);
 auto B=UseWarm?FGameInputSelectedSource::CreateRaw(WarmB.GetOrCreate(ApiB.Get()),1002):FGameInputSelectedSource::CreateRaw(ApiB.Get(),1002);
 Check(bool(A)&&bool(B),"two independent raw sources constructed without Activity arbiter");
 Check(A->RequestSelection(std::make_pair(Key(1),EDeviceKind::Gamepad)),"client A exact request accepted");
 Check(B->RequestSelection(std::make_pair(Key(2),EDeviceKind::Gamepad)),"client B exact request accepted");
 auto JA=std::make_shared<FRawAcquisitionJournal>(1001),JB=std::make_shared<FRawAcquisitionJournal>(1002);
 FGameInputRawAcquisition OwnerA(std::move(A),JA),OwnerB(std::move(B),JB);
 for(auto* Api:{ApiA.Get(),ApiB.Get()})
 {Api->Sample(Pad1.Get(),10,true);Api->Sample(Pad2.Get(),10,false);Api->Sample(Other.Get(),10,true);}
 Check(OwnerA.Pump()==ERawPumpResult::Installed&&OwnerB.Pump()==ERawPumpResult::Installed,"both exact device baselines installed");
 auto FA=JA->Poll(0),FB=JB->Poll(0);
 Check(FA&&FB&&FA->FinalState.size()>19&&FB->FinalState.size()>19&&FA->FinalState[19].Value==1&&FB->FinalState[19].Value==0,"pad1 throttle affects A only");
 Check(OwnerA.ReadSelectedRawDeviceDiagnostic()->Id==Key(1)&&OwnerB.ReadSelectedRawDeviceDiagnostic()->Id==Key(2),"backend IDs remain distinct from sessions");
 Check(ApiA->DeviceCalls[Pad2.Get()]==0&&ApiA->DeviceCalls[Other.Get()]==0&&ApiB->DeviceCalls[Pad1.Get()]==0&&ApiB->DeviceCalls[Other.Get()]==0,"explicit sources never poll unassigned devices for activity");
 for(auto* Api:{ApiA.Get(),ApiB.Get()})
 {Api->Sample(Pad1.Get(),11,false);Api->Sample(Pad2.Get(),11,true);Api->Sample(Other.Get(),11,true);}
 Check(OwnerA.Pump()==ERawPumpResult::Installed&&OwnerB.Pump()==ERawPumpResult::Installed,"opposite commands installed");
 FA=JA->Poll(1);FB=JB->Poll(1);
 Check(FA&&FB&&FA->FinalState.size()>19&&FB->FinalState.size()>19&&FA->FinalState[19].Value==0&&FB->FinalState[19].Value==1,"pad2 throttle affects B only");
 const auto Before=FA->Generation.Value;
 ApiA->FireDevice(Pad1.Get(),20,false);ApiB->FireDevice(Pad1.Get(),20,false);
 OwnerA.Pump();OwnerB.Pump();FA=JA->Poll(2);FB=JB->Poll(2);
 Check(FA&&FB&&!FA->FinalState.empty()&&FB->FinalState.size()>19&&std::all_of(FA->FinalState.begin(),FA->FinalState.end(),[](const auto& V){return V.Value==0;})&&FB->FinalState[19].Value==1,"removed A becomes neutral while B retains its device");
 Check(!OwnerA.ReadSelectedRawDeviceDiagnostic()&&OwnerB.ReadSelectedRawDeviceDiagnostic()->Id==Key(2),"another active pad never steals disconnected request");
 auto Replacement=MakeDevice(1);
 for(auto* Api:{ApiA.Get(),ApiB.Get()})
 {Api->Sample(Replacement.Get(),30,false);Api->FireDevice(Replacement.Get(),30,true);Api->FireDevice(Pad1.Get(),31,false);}
 OwnerA.Pump();FA=JA->Poll(3);
 Check(FA&&FA->FinalState.size()>19&&FA->Generation.Value>Before&&FA->FinalState[19].Value==0,"exact ID reconnect creates new generation and fresh neutral baseline");
 Check(OwnerA.ReadSelectedRawDeviceDiagnostic()->Id==Key(1),"obsolete object disconnect cannot retire replacement");
 Check(OwnerA.Close()&&OwnerB.Close(),"both acquisition owners close without worker remnants");
 Check(WarmA.Close()&&WarmB.Close(),"warm catalogues close after exact route owners retire");
}
int main()
{
 RunRoutes(false);RunRoutes(true);
 std::cout<<"PASS WindowsExplicitDeviceRoutingProbe checks="<<Checks<<" hardware=none cold_and_warm=1\n";
 return 0;
}
