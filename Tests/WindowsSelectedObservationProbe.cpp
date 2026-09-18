#include "WindowsDiscoveryFakes.h"
#include "IAmSpeed/Input/Windows/GameInputSelectedSource.h"
using Speed::Input::Windows::FGameInputSelectedSource;
int main()
{
    ComPtr<DiscoveryApi> Api; Api.Attach(new DiscoveryApi);
    auto Device=MakeDevice(1,GameInputKindKeyboard); auto Other=MakeDevice(2,GameInputKindKeyboard);
    Api->Initial={{Device,1,true}};
    std::array<bool,ActionCount> Digital{}; Digital[3]=true;
    auto Mapper=[](const FDeviceState& S,FActionValues& V) {V[3]=S.VirtualKeys[VK_F9]?1:0;return true;};
    auto Source=FGameInputSelectedSource::Create(Api.Get(),902,Digital,Mapper,Mapper);
    Check(Source && !Source->ReadObservation(),"observations opt-in");
    Check(Source->EnableObservations() && Source->RequestSelection(std::make_pair(Key(1),EDeviceKind::Keyboard)),"diagnostic setup");
    auto Sample=[&](IGameInputDevice* D,bool Held) {
        ComPtr<FakeReading> R; R.Attach(new FakeReading); R->ObservedDevice=D;
        if(Held) {GameInputKeyState K{}; K.virtualKey=VK_F9; R->Keys.push_back(K);}
        Api->Responses.push_back({S_OK,R}); return R;
    };
    Sample(Device.Get(),false); Sample(Device.Get(),true);
    Check(Source->Produce(0).has_value(),"initial observed poll");
    auto O=Source->ReadObservation();
    Check(O && O->Frame==0 && O->AcquisitionTicket && O->AcquisitionTicket->Device.Id==Key(1)
        && O->AcquisitionTicket->Device.Revision==1 && O->AcquisitionTicket->Generation>0,"actual lease ticket");
    Check(O->SubmitAccepted==2 && O->SubmitRejected==0 && O->Readings.Count==3,"successful submissions and terminal not-found");
    Check(O->Readings.Calls[0].Current && !O->Readings.Calls[1].Current && !O->Readings.Calls[2].HasReading,"actual current/next branches");
    Check(O->Readings.Calls[0].IdentityValid && O->Readings.Calls[1].IdentityValid
        && !O->Readings.Calls[1].SamePrevious && O->Readings.Calls[0].Timestamp==O->Readings.Calls[1].Timestamp,"COM identity independent of timestamp");
    Check(O->Readings.Calls[0].HasDeviceId && O->Readings.Calls[0].DeviceId==Key(1),"observed reading identity");
    const auto Calls=Api->ReadCalls.load(); Source->ReadObservation();
    Check(Api->ReadCalls==Calls && !Source->EnableObservations(),"copy no SDK read; cannot reset instrumentation after polling");
    Sample(Other.Get(),false); Check(Source->Produce(1).has_value(),"mismatch fixture");
    Check(Source->ReadObservation()->Readings.Calls[0].DeviceId==Key(2),"observed ID not copied from filter");
    Check(O->Readings.Calls[0].DeviceId==Key(1),"old observation immutable");
    Check(Source->SetPaused(true) && Source->Produce(2).has_value(),"pause fixture");
    Check(!Source->ReadObservation()->AcquisitionTicket && Source->ReadObservation()->Readings.Count==0,"no stale ticket when paused");
    Source->SetPaused(false); Sample(Device.Get(),true);
    Api->BeforeReturn=[&] {Api->FireDevice(Device.Get(),2,false);};
    Check(Source->Produce(3).has_value(),"invalidation fixture");
    const auto Lost=Source->ReadObservation();
    Check(Lost->SubmitAccepted==0 && Lost->SubmitRejected==1,"attempted ticket not committed on hotplug");
    Check(Source->Shutdown(),"shutdown");
    std::cout<<"PASS WindowsSelectedObservationProbe checks="<<Checks<<" hardware=none sdk=v3\n";
}
