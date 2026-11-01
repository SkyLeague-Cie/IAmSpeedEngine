#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadPhysicsWorld.h"
#include "IAmSpeed/Driving/DrivingInput.h"
#include "Misc/AutomationTest.h"
#include "UObject/GarbageCollection.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASPrivateRoadNativeWorld,
    "IAmSpeed.Netcode.Prediction.PrivateRoadNativeWorld",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASPrivateRoadNativeWorld::RunTest(const FString&)
{
    namespace P=Speed::Netcode::Prediction;namespace V=Speed::Input::V2;using namespace Speed::Netcode;
    if(!TestTrue(TEXT("actual native private graph uses the GT"),IsInGameThread()))return false;
    auto Contract=Speed::Driving::MakeContract(Speed::Driving::FBindings{});if(!Contract)return false;
    std::vector<P::FPrivateRoadCarSeed> Seeds;
    for(U64 A=0;A<2;++A)
    {
        P::FPrivateRoadCarSeed S;S.Actor=101+A;S.Incarnation=3;S.Contract=Contract;
        S.Local={Speed::Input::EProducerKind::Device,42+A};S.LocalEpoch={3};
        S.Remote.Session=1;S.Remote.Epoch=2;S.Remote.Actor=S.Actor;S.Remote.Incarnation=3;
        S.Remote.OwnerPeer=4;S.Remote.ServerPeer=5;S.Remote.SourceProducer=6+A;S.Remote.SourceKind=0;S.Remote.Contract[0]=7;
        S.Initial.Location=FVector(0,A?1000:-1000,45);S.Initial.Velocity=FVector(500,0,0);
        Seeds.push_back(S);
    }
    FString Why;
    auto A=P::FPrivateRoadPhysicsWorld::Create(Seeds,Why);
    if(!TestTrue(*FString::Printf(TEXT("first real native world created: %s"),*Why),bool(A)))return false;
    auto B=P::FPrivateRoadPhysicsWorld::Create(Seeds,Why);
    if(!TestTrue(*FString::Printf(TEXT("second independent native world created: %s"),*Why),bool(B)))return false;
    std::array<Consumed::FActionPolicy,ActionCount> Actions{};
    Actions[0]={true,true,false,255};Actions[1]={true,true,false,255};Actions[2]={true,true,true,127};
    P::FRemoteRequestHistory HA(Seeds[0].Remote,Actions,256,512),HB(Seeds[1].Remote,Actions,256,512);
    const auto Packet=[&](size_t Actor,U64 Frame,int16 Throttle,int16 Brake,int16 Steering)
    {
        const auto& Bind=Seeds[Actor].Remote;FPacket Q;Q.Kind=EKind::AuthorityInput;
        Q.Session=Bind.Session;Q.Epoch=Bind.Epoch;Q.Actor=Bind.Actor;Q.Incarnation=Bind.Incarnation;
        Q.Contract=Bind.Contract;Q.Frame=Frame;Q.Sequence=Frame+1;
        Q.Input.SourceSequence=Frame+1;Q.Input.DeviceGeneration=1;Q.Input.Reset=Frame==0;
        Q.Input.ProducerId=Bind.SourceProducer;Q.Input.ProducerKind=Bind.SourceKind;
        Q.Input.Values[0]=Throttle;Q.Input.Values[1]=Brake;Q.Input.Values[2]=Steering;
        Q.Input.ActiveMask=(Throttle?1u:0u)|(Brake?2u:0u)|(Steering?4u:0u);return Q;
    };
    if(HA.Confirm(5,Packet(0,0,255,0,0)).Status!=EResult::Accepted
        ||HB.Confirm(5,Packet(1,0,0,255,0)).Status!=EResult::Accepted)return false;
    std::vector<std::vector<P::FPrivateRoadRequest>> Retained;
    std::vector<std::shared_ptr<const P::FPrivateRoadPublication>> Expected;
    const auto Equivalent=[&](const P::FPrivateRoadPublication& L,const P::FPrivateRoadPublication& R)
    {
        if(L.Frame!=R.Frame||L.Actors.size()!=2||R.Actors.size()!=2||L.Physics.NumFrame!=R.Physics.NumFrame
            ||L.Physics.PresentationBodies.Num()!=2||R.Physics.PresentationBodies.Num()!=2)return false;
        for(size_t C=0;C<2;++C)
        {
            if(L.Actors[C].Actor!=R.Actors[C].Actor||!(L.Actors[C].Filter==R.Actors[C].Filter)
                ||L.Actors[C].Input->Applied!=R.Actors[C].Input->Applied)return false;
            const auto* LP=L.Physics.PresentationBodies.FindByPredicate([&](const auto& Body){return Body.StableId==L.Actors[C].NativeStableId;});
            const auto* RP=R.Physics.PresentationBodies.FindByPredicate([&](const auto& Body){return Body.StableId==R.Actors[C].NativeStableId;});
            if(!LP||!RP||LP->WheelVisuals.Num()!=4||RP->WheelVisuals.Num()!=4)return false;
            const auto& X=LP->COMState;const auto& Y=RP->COMState;
            if(X.Location!=Y.Location||X.Velocity!=Y.Velocity||X.Rotation!=Y.Rotation||X.AngularVelocity!=Y.AngularVelocity)return false;
        }
        return true;
    };
    for(U64 N=0;N<181;++N)
    {
        if(N==60&&HA.Confirm(5,Packet(0,N,255,0,64)).Status!=EResult::Accepted)return false;
        if(N==120&&HA.Confirm(5,Packet(0,N,255,0,-64)).Status!=EResult::Accepted)return false;
        std::vector<P::FPrivateRoadRequest> Requests={{101,3,HA.Predict(N)},{102,3,HB.Predict(N)}};
        if(!TestTrue(*FString::Printf(TEXT("actual native step A frame %llu: %s"),N,*Why),A->Step(Requests,false,Why))
            ||!TestTrue(*FString::Printf(TEXT("actual native step B frame %llu: %s"),N,*Why),B->Step(Requests,false,Why)))return false;
        auto X=A->ReadLatest(),Y=B->ReadLatest();
        if(!TestTrue(TEXT("same commands produce exact independent native trajectories"),X&&Y&&Equivalent(*X,*Y)))return false;
        if(!TestTrue(TEXT("separate real birth lifetimes"),X->LocalKernelLifetime!=Y->LocalKernelLifetime))return false;
        Retained.push_back(Requests);Expected.push_back(X);
        if(N==90)CollectGarbage(RF_NoFlags,true); // actual graph/config pins, no BeginPlay/context root assumption.
    }
    const auto Last=Expected.back();
    const auto Body=[&](size_t C)->const FSimulationPresentationBody*
    {return Last->Physics.PresentationBodies.FindByPredicate([&](const auto& P){return P.StableId==Last->Actors[C].NativeStableId;});};
    if(!TestTrue(TEXT("throttle and brake drive different real trajectories"),Body(0)&&Body(1)
        &&Body(0)->COMState.Location.X>Body(1)->COMState.Location.X+1))return false;
    // Fresh-origin reconstruction and replay exercises actual native storage,
    // not a forged full-state restore. Authenticated historical correction is
    // a later, separate gate; this test cannot qualify connected rollback.
    auto Replay=P::FPrivateRoadPhysicsWorld::Create(Seeds,Why);
    if(!TestTrue(TEXT("fresh native replay world"),bool(Replay)))return false;
    for(size_t N=0;N<Retained.size();++N)
    {
        if(!TestTrue(*FString::Printf(TEXT("actual native replay frame %llu: %s"),U64(N),*Why),Replay->Step(Retained[N],true,Why)))return false;
        const auto X=Replay->ReadLatest();
        if(!TestTrue(TEXT("fresh-origin resimulation matches actual native forward trajectory"),X&&Equivalent(*Expected[N],*X)))return false;
    }
    TestTrue(TEXT("all real graphs retire on owning GT"),A->Close()&&B->Close()&&Replay->Close());
    TestFalse(TEXT("closed kernel has no current publication"),bool(A->ReadLatest()));
    TestFalse(TEXT("closed graph cannot consume another frame"),A->Step(Retained.back(),false,Why));
    CollectGarbage(RF_NoFlags,true);
    TestTrue(TEXT("retained immutable values survive graph retirement and GC"),Last->Frame==180&&Last->Actors[0].Filter.Next==181&&Last->Actors[0].Input->Requested[0]==255);
    AddInfo(TEXT("Three actual private native worlds, two cars each, 181 frames each, independent reference and fresh-origin replay. No sockets, source authentication, arbitrary snapshot restore, lifecycle lease certificate or connected client prediction qualification."));
    return !HasAnyErrors();
}
#endif
