#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Netcode/client-prediction/BranchInputFilter.h"
#include "IAmSpeed/Driving/DrivingInput.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASBranchInputFilter,
    "IAmSpeed.Netcode.Prediction.NativeBranchInputFilter",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASBranchInputFilter::RunTest(const FString&)
{
    namespace P=Speed::Netcode::Prediction;namespace V=Speed::Input::V2;
    using namespace Speed::Netcode;
    auto Contract=Speed::Driving::MakeContract(Speed::Driving::FBindings{});if(!Contract)return false;
    V::FInputProcessingPolicy Policy;Policy.Step[0]=17;Policy.Step[1]=11;Policy.Step[2]=9;
    FBinding B;B.Session=1;B.Epoch=2;B.Actor=101;B.Incarnation=3;B.OwnerPeer=4;B.ServerPeer=5;
    B.SourceProducer=6;B.SourceKind=0;B.Contract[0]=7;
    const Speed::Input::FProducerIdentity Local{Speed::Input::EProducerKind::Device,42};
    auto Make=[&](P::EBranchFilterRole Role){return P::FBranchInputFilter::Create(Role,Contract,Policy,Local,{3},B,10,{10,{ },0});};
    auto A=Make(P::EBranchFilterRole::LocalDevice),R=Make(P::EBranchFilterRole::RemoteRequests);
    if(!TestTrue(TEXT("distinct local and remote processing roles created"),bool(A)&&bool(R)))return false;
    V::FInputFrameData D;D.SourceSequence=55;D.ConsumptionFrame=10;D.Producer=Local;D.StreamEpoch={3};D.DeviceGeneration={2};
    D.Values[0]=255;D.ActiveMask=1;D.Transitions.push_back({0,V::ETransition::Started,255,{55,0}});
    auto Retained=std::make_shared<const V::FInputFrame>(Contract,D);
    auto L=A->PrepareLocal(Retained);if(!L)return false;
    auto Observed=L->ReadProcessed();
    TestEqual(TEXT("local request processed exactly once"),Observed->Applied[0],int16(17));
    TestTrue(TEXT("original immutable local frame retained without remapping"),std::get<std::shared_ptr<const V::FInputFrame>>(Observed->Evidence)==Retained);
    TestTrue(TEXT("preparation changes no filter state"),A->Capture()->Next==10&&A->Capture()->Applied[0]==0);
    TestTrue(TEXT("local filter commits"),A->Commit(*L));TestFalse(TEXT("local frame cannot commit twice"),A->Commit(*L));
    TestFalse(TEXT("remote role cannot consume local Device receipt"),bool(R->PrepareLocal(Retained)));
    auto Forecast=[](U64 Frame,U64 Evidence,int16 Value){P::FPrediction F;F.Status=P::EPredict::Predicted;F.Request={Frame,Evidence,{ },Value?1u:0u};F.Request.Requested[0]=Value;return F;};
    auto F10=Forecast(10,9,255);TestFalse(TEXT("local role cannot consume opponent forecasts"),bool(A->PrepareRemote(F10)));
    const auto Baseline=*R->Capture();auto T10=R->PrepareRemote(F10);if(!T10)return false;
    auto RemoteObserved=T10->ReadProcessed();
    TestTrue(TEXT("speculation retains its own type with no Device metadata"),std::holds_alternative<P::FSpeculativeRequest>(RemoteObserved->Evidence));
    TestTrue(TEXT("remote request commits"),R->Commit(*T10));
    auto T11=R->PrepareRemote(Forecast(11,9,255));if(!T11)return false;
    TestEqual(TEXT("held REQUEST advances slew again, not held APPLIED"),T11->ReadProcessed()->Applied[0],int16(34));
    TestTrue(TEXT("second remote request commits"),R->Commit(*T11));const auto AfterTwo=*R->Capture();
    auto Release=R->PrepareRemote(Forecast(12,11,0));if(!Release)return false;
    TestEqual(TEXT("release applies one bounded decrement"),Release->ReadProcessed()->Applied[0],int16(17));
    TestTrue(TEXT("release commits"),R->Commit(*Release));
    auto Rewind=R->PrepareRestore(Baseline);if(!Rewind)return false;TestTrue(TEXT("filter baseline restores"),R->Commit(*Rewind));
    TestFalse(TEXT("pre-rewind token cannot mutate new revision"),R->Commit(*Release));
    auto Again10=R->PrepareRemote(F10);if(!Again10||!R->Commit(*Again10))return false;
    auto Again11=R->PrepareRemote(Forecast(11,9,255));if(!Again11||!R->Commit(*Again11))return false;
    TestTrue(TEXT("replay from exact baseline reproduces full filter state"),*R->Capture()==AfterTwo);
    TestEqual(TEXT("replay did not invent or advance local raw source sequence"),Retained->GetData().SourceSequence,uint64_t(55));
    TestFalse(TEXT("same-frame opponent evidence refused"),bool(R->PrepareRemote(Forecast(12,12,255))));
    TestFalse(TEXT("prediction horizon exceeded refused"),bool(R->PrepareRemote(Forecast(12,1,255))));
    P::FPrediction Missing;TestFalse(TEXT("no earlier authority is not implicit neutral input"),bool(R->PrepareRemote(Missing)));
    FPacket Packet;Packet.Kind=EKind::AuthorityInput;Packet.Session=B.Session;Packet.Epoch=B.Epoch;Packet.Actor=B.Actor;
    Packet.Incarnation=B.Incarnation;Packet.Contract=B.Contract;Packet.Sequence=20;Packet.Frame=12;
    Packet.Input.SourceSequence=99;Packet.Input.DeviceGeneration=4;Packet.Input.ProducerId=B.SourceProducer;Packet.Input.ProducerKind=B.SourceKind;
    Packet.Input.Reset=true;Packet.Input.Values[0]=255;Packet.Input.ActiveMask=1;
    P::FPrediction Confirmed;Confirmed.Status=P::EPredict::Confirmed;Confirmed.ConfirmedPacket=Packet;
    auto Exact=R->PrepareRemote(Confirmed);if(!Exact)return false;
    TestEqual(TEXT("actual confirmed reset preserves original policy semantics"),Exact->ReadProcessed()->Applied[0],int16(255));
    TestTrue(TEXT("confirmed evidence remains original received packet type"),std::holds_alternative<std::shared_ptr<const FPacket>>(Exact->ReadProcessed()->Evidence));
    Confirmed.ConfirmedPacket->Input.Reset=false;Confirmed.ConfirmedPacket->Input.Values[31]=1;
    TestFalse(TEXT("unwired request refused"),bool(R->PrepareRemote(Confirmed)));
    auto Other=Make(P::EBranchFilterRole::RemoteRequests);TestFalse(TEXT("another native filter lifetime rejects plan"),Other->Commit(*Exact));
    bool ForeignPrepared=true;std::thread Foreign([&]{ForeignPrepared=bool(R->PrepareRemote(Forecast(12,11,255)));});Foreign.join();
    TestFalse(TEXT("wrong physical owner thread cannot prepare"),ForeignPrepared);
    auto BadPolicy=Policy;BadPolicy.Step[Speed::Driving::PauseAction]=1;
    TestFalse(TEXT("boolean action cannot receive analog slew"),bool(P::FBranchInputFilter::Create(P::EBranchFilterRole::RemoteRequests,Contract,BadPolicy,Local,{3},B,10,Baseline)));
    TestTrue(TEXT("retained processed values survive staging token retirement"),Observed->Applied[0]==17&&RemoteObserved->Applied[0]==17);
    std::array<Consumed::FActionPolicy,ActionCount> Actions{};
    Actions[0]={true,true,false,255};Actions[1]={true,true,false,255};Actions[2]={true,true,true,127};
    auto PacketAt=[&](U64 N,int16 Value)
    {
        auto Q=Packet;Q.Frame=N;Q.Sequence=N+1;Q.Input.SourceSequence=N+1;Q.Input.Reset=false;
        Q.Input.Values={};Q.Input.Values[0]=Value;Q.Input.ActiveMask=Value?1u:0u;return Q;
    };
    P::FRemoteRequestHistory Mixed(B,Actions,10,16);
    TestTrue(TEXT("mixed branch history anchors accepted"),Mixed.Confirm(5,PacketAt(9,255)).Status==EResult::Accepted&&Mixed.Confirm(5,PacketAt(11,0)).Status==EResult::Accepted);
    TestTrue(TEXT("branch consumes speculation then actual confirmation then speculation"),Mixed.Predict(10).Status==P::EPredict::Predicted&&Mixed.Predict(11).Status==P::EPredict::Confirmed&&Mixed.Predict(12).Status==P::EPredict::Predicted);
    Mixed.DiscardSpeculationFrom(12);auto Replayed=Mixed.Predict(12);
    TestTrue(TEXT("rewind restores actual first-replay cursor including consumed confirmation"),Replayed.Status==P::EPredict::Predicted&&Replayed.Request.EvidenceFrame==11&&Replayed.Request.Requested[0]==0);
    Mixed.DiscardSpeculationFrom(0);
    TestTrue(TEXT("exact zero-frame cursor cannot skip ahead"),Mixed.Predict(1).Status==P::EPredict::OutOfOrder);
    TestTrue(TEXT("confirmed frame zero can replay after exact rewind"),Mixed.Confirm(5,PacketAt(0,0)).Status==EResult::Accepted&&Mixed.Predict(0).Status==P::EPredict::Confirmed);
    AddInfo(TEXT("Input processing domain only, using original pure authoritative policy. Unit-authored input evidence; no hardware, native physical world, transport ACK, native fullstate restore or connected prediction qualification."));
    return !HasAnyErrors();
}
#endif
