#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Netcode/client-prediction/BranchInputFilter.h"
#include "IAmSpeed/Driving/DrivingInput.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASInputHistoryFilterReplay,
    "IAmSpeed.Netcode.Prediction.InputHistoryFilterReplay",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASInputHistoryFilterReplay::RunTest(const FString&)
{
    namespace P=Speed::Netcode::Prediction;namespace V=Speed::Input::V2;
    using namespace Speed::Netcode;
    auto C=Speed::Driving::MakeContract(Speed::Driving::FBindings{});if(!C)return false;
    FBinding B;B.Session=1;B.Epoch=2;B.Actor=101;B.Incarnation=3;B.OwnerPeer=4;B.ServerPeer=5;
    B.SourceProducer=6;B.SourceKind=0;B.Contract[0]=7;
    std::array<Consumed::FActionPolicy,ActionCount> Actions{};
    Actions[0]={true,true,false,255};Actions[1]={true,true,false,255};Actions[2]={true,true,true,127};
    V::FInputProcessingPolicy Policy;Policy.Step[0]=17;
    P::FBranchFilterState Baseline;Baseline.Next=10;Baseline.Applied[0]=64;Baseline.ActiveMask=1;
    const Speed::Input::FProducerIdentity Local{Speed::Input::EProducerKind::Device,42};
    auto Client=P::FBranchInputFilter::Create(P::EBranchFilterRole::RemoteRequests,C,Policy,Local,{3},B,10,Baseline);
    auto Reference=P::FBranchInputFilter::Create(P::EBranchFilterRole::RemoteRequests,C,Policy,Local,{3},B,10,Baseline);
    if(!Client||!Reference)return false;
    auto Packet=[&](U64 N,int16 Value)
    {
        FPacket Q;Q.Kind=EKind::AuthorityInput;Q.Session=B.Session;Q.Epoch=B.Epoch;Q.Actor=B.Actor;Q.Incarnation=B.Incarnation;
        Q.Contract=B.Contract;Q.Frame=N;Q.Sequence=N+1;Q.Input.SourceSequence=N+1;Q.Input.DeviceGeneration=1;
        Q.Input.ProducerId=B.SourceProducer;Q.Input.ProducerKind=B.SourceKind;Q.Input.Values[0]=Value;Q.Input.ActiveMask=Value?1u:0u;return Q;
    };
    P::FRemoteRequestHistory History(B,Actions,10,32),Authority(B,Actions,10,32);
    TestTrue(TEXT("client receives only original anchor and later confirmed release"),History.Confirm(5,Packet(9,255)).Status==EResult::Accepted&&History.Confirm(5,Packet(11,0)).Status==EResult::Accepted);
    for(const auto& Q:{Packet(9,255),Packet(10,128),Packet(11,0),Packet(12,0)})
        if(Authority.Confirm(5,Q).Status!=EResult::Accepted)return false;
    std::shared_ptr<const P::FBranchProcessedInput> Original10;
    P::FBranchFilterState After11;
    const int16 Expected[]={81,64,47};
    for(U64 N=10;N<=12;++N)
    {
        const auto Forecast=History.Predict(N);auto Plan=Client->PrepareRemote(Forecast);if(!Plan)return false;
        if(N==10)Original10=Plan->ReadProcessed();
        TestEqual(TEXT("one bounded policy step per consumed frame"),Plan->ReadProcessed()->Applied[0],Expected[N-10]);
        if(!Client->Commit(*Plan))return false;
        if(N==11)After11=*Client->Capture();
        auto Known=Reference->PrepareRemote(Authority.Predict(N));if(!Known||!Reference->Commit(*Known))return false;
    }
    const auto BeforeCorrection=*Client->Capture();
    // The surviving spec10 must NOT move the cursor behind the consumed,
    // confirmed frame11 when the actual first replay frame is12.
    auto Restore=Client->PrepareRestore(After11);if(!Restore||!Client->Commit(*Restore))return false;
    History.DiscardSpeculationFrom(12);
    auto Replayed12=Client->PrepareRemote(History.Predict(12));
    if(!TestTrue(TEXT("history cursor and filter resume the same exact frame after mixed prefix"),bool(Replayed12)))return false;
    if(!Client->Commit(*Replayed12))return false;
    TestTrue(TEXT("replayed prefix reproduces full filter state"),*Client->Capture()==BeforeCorrection);
    const auto Late=History.Confirm(5,Packet(10,128));
    TestTrue(TEXT("late changed REQUEST invalidates history even when applied value coincides"),Late.Status==EResult::Accepted&&Late.EarliestInvalidatedFrame==10);
    Restore=Client->PrepareRestore(Baseline);if(!Restore||!Client->Commit(*Restore))return false;
    History.DiscardSpeculationFrom(10);
    std::shared_ptr<const P::FBranchProcessedInput> Corrected10;
    for(U64 N=10;N<=12;++N)
    {
        auto Plan=Client->PrepareRemote(History.Predict(N));if(!Plan)return false;
        if(N==10)Corrected10=Plan->ReadProcessed();
        if(!Client->Commit(*Plan))return false;
    }
    TestTrue(TEXT("original forecast retains only previously received request"),Original10&&Original10->Requested[0]==255&&Original10->Origin==P::EBranchRequestOrigin::SpeculativeHoldRequest);
    TestTrue(TEXT("replay substitutes actual later receipt without counterfeit raw input"),Corrected10&&Corrected10->Requested[0]==128&&Corrected10->Origin==P::EBranchRequestOrigin::ReceivedAuthorityRequest);
    TestEqual(TEXT("same applied result never erases request/provenance divergence"),Original10->Applied[0],Corrected10->Applied[0]);
    TestTrue(TEXT("full processing state converges to independently consumed confirmed input"),*Client->Capture()==*Reference->Capture());
    TestTrue(TEXT("received authority history retained across rewind"),History.Confirm(5,Packet(11,0)).Status==EResult::Duplicate);
    AddInfo(TEXT("Scripted input-domain integration only: real history and filter implementation, exact mixed cut and late receipt replay. No sockets, hardware, physics world, native lifetime lease or connected client rollback qualification."));
    return !HasAnyErrors();
}
#endif
