#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadPhysicsWorld.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadKinematicDiagnostics.h"
#include "IAmSpeed/Netcode/transport/AuthorityInputJournal.h"
#include "IAmSpeed/Driving/DrivingInput.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASRetainedNativeBoundary,
    "IAmSpeed.Netcode.Prediction.RetainedNativeBoundaryExperiment",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASRetainedNativeBoundary::RunTest(const FString&)
{
    namespace P=Speed::Netcode::Prediction;using namespace Speed::Netcode;
    constexpr U64 Cut=30,Change=45,Last=180;
    auto Contract=Speed::Driving::MakeContract(Speed::Driving::FBindings{});if(!Contract)return false;
    std::vector<P::FPrivateRoadCarSeed> Seeds;
    std::vector<FBinding> Bindings;
    for(U64 A=0;A<2;++A)
    {
        P::FPrivateRoadCarSeed S;S.StartPolicy=P::EPrivateRoadStartPolicy::ExplicitUnitRecipe;
        S.Actor=101+A;S.Incarnation=3;S.Contract=Contract;S.Role=P::EBranchFilterRole::RemoteRequests;
        S.Local={Speed::Input::EProducerKind::Device,42+A};S.LocalEpoch={3};
        auto& B=S.Remote;B.Session=11;B.Epoch=3;B.Actor=S.Actor;B.Incarnation=3;
        B.OwnerPeer=A?7:4;B.ServerPeer=5;B.SourceProducer=42+A;B.SourceKind=0;B.Contract[0]=7;
        S.Initial.Location=FVector(0,A?1000:-1000,45);S.Initial.Velocity=FVector(500,0,0);
        S.Processing.Step[0]=17;S.Processing.Step[1]=17;S.Processing.Step[2]=9;
        Bindings.push_back(B);Seeds.push_back(S);
    }
    std::array<Consumed::FActionPolicy,ActionCount> Actions{};
    Actions[0]={true,true,false,255};Actions[1]={true,true,false,255};Actions[2]={true,true,true,127};
    P::FRemoteRequestHistory RA(Bindings[0],Actions,256,512),RB(Bindings[1],Actions,256,512);
    P::FRemoteRequestHistory LA(Bindings[0],Actions,256,512),LB(Bindings[1],Actions,256,512);
    P::FRemoteRequestHistory CA(Bindings[0],Actions,256,512),CB(Bindings[1],Actions,256,512);
    const auto Packet=[&](unsigned A,U64 N)
    {
        const auto& B=Bindings[A];FPacket Q;Q.Kind=EKind::AuthorityInput;
        Q.Session=B.Session;Q.Epoch=B.Epoch;Q.Actor=B.Actor;Q.Incarnation=B.Incarnation;Q.Contract=B.Contract;
        Q.Frame=N;Q.Sequence=N+1;Q.Input.SourceSequence=N+1;Q.Input.DeviceGeneration=1;
        Q.Input.ProducerId=B.SourceProducer;Q.Input.ProducerKind=B.SourceKind;Q.Input.Reset=N==0;
        if(A==0&&N>=Change){Q.Input.Values[0]=255;Q.Input.ActiveMask=1;}
        if(A==0&&N==Change)Q.Input.Edges.push_back({0,0,255,N+1,0});
        return Q;
    };
    FString Why;
    auto Reference=P::FPrivateRoadPhysicsWorld::Create(Seeds,Why);
    auto Live=P::FPrivateRoadPhysicsWorld::Create(Seeds,Why);
    auto Retained=P::FPrivateRoadPhysicsWorld::Create(Seeds,Why);
    auto Spectator=P::FPrivateRoadPhysicsWorld::Create({Seeds[1]},Why);
    if(!TestTrue(*FString::Printf(TEXT("four actual independent native graphs: %s"),*Why),Reference&&Live&&Retained&&Spectator))return false;
    auto Journal=FAuthorityInputJournal::Create(Bindings,Last+1);if(!Journal)return false;
    std::shared_ptr<const P::FPrivateRoadPublication> OriginalCut;
    const auto SameObservable=[&](const P::FPrivateRoadPublication& A,const P::FPrivateRoadPublication& B)
    {
        if(A.Frame!=B.Frame||A.Actors.size()!=B.Actors.size()||A.NativeNetworkComponents||B.NativeNetworkComponents||A.NativeDebugDelegateBindings||B.NativeDebugDelegateBindings
            ||A.RenderScenePresent||B.RenderScenePresent||A.PhysicsScenePresent||B.PhysicsScenePresent)return false;
        for(size_t N=0;N<A.Actors.size();++N)
        {
            const auto& X=A.Actors[N];const auto& Y=B.Actors[N];
            if(X.Actor!=Y.Actor||X.Incarnation!=Y.Incarnation||!(X.Filter==Y.Filter)||!X.Input||!Y.Input
                ||X.Input->Requested!=Y.Input->Requested||X.Input->Applied!=Y.Input->Applied
                ||X.CanMove!=Y.CanMove||X.Frozen!=Y.Frozen||X.CountdownFrames!=Y.CountdownFrames)return false;
            const auto* P0=A.Physics.PresentationBodies.FindByPredicate([&](const auto& V){return V.StableId==X.NativeStableId;});
            const auto* P1=B.Physics.PresentationBodies.FindByPredicate([&](const auto& V){return V.StableId==Y.NativeStableId;});
            if(!P0||!P1)return false;const auto D=P::MeasurePrivateRoadKinematics(P0->COMState,P1->COMState);
            if(!D.Valid||D.Different)return false;
        }
        return true;
    };
    if(LA.Confirm(5,Packet(0,0)).Status!=EResult::Accepted||LB.Confirm(5,Packet(1,0)).Status!=EResult::Accepted)return false;
    for(U64 N=0;N<=Last;++N)
    {
        const auto A=Packet(0,N),B=Packet(1,N);
        if(RA.Confirm(5,A).Status!=EResult::Accepted||RB.Confirm(5,B).Status!=EResult::Accepted)return false;
        const std::vector<P::FPrivateRoadRequest> Requests={{101,3,RA.Predict(N)},{102,3,RB.Predict(N)}};
        if(!Reference->Step(Requests,false,Why))return false;
        // Original records are retained only after the actual native reference
        // consumed them. This test's author is a UNIT author, not network auth.
        const auto Actual=Reference->ReadLatest();if(!Actual||Actual->Frame!=N||Actual->Actors.size()!=2)return false;
        for(unsigned Actor=0;Actor<2;++Actor)
        {
            const auto& Input=Actual->Actors[Actor].Input;if(!Input||Input->Frame!=N)return false;
            const auto* Original=std::get_if<std::shared_ptr<const FPacket>>(&Input->Evidence);
            if(!Original||!*Original||Encode(**Original)!=Encode(Actor?B:A))return false;
        }
        if(Journal->Retain(5,A)!=EResult::Accepted||Journal->Retain(5,B)!=EResult::Accepted)return false;
        if(!Live->Step({{101,3,LA.Predict(N)},{102,3,LB.Predict(N)}},false,Why))return false;
        if(!Spectator->Step({{102,3,Requests[1].Evidence}},false,Why))return false;
        if(N<=Cut)
        {
            if(CA.Confirm(5,A).Status!=EResult::Accepted||CB.Confirm(5,B).Status!=EResult::Accepted
                ||!Retained->Step({{101,3,CA.Predict(N)},{102,3,CB.Predict(N)}},false,Why))return false;
            if(N==Cut)
            {
                OriginalCut=Retained->ReadLatest();
                if(!TestTrue(TEXT("actual retained graph reached the confirmed observable cut"),OriginalCut&&SameObservable(*Actual,*OriginalCut)))return false;
            }
        }
        else if(!TestTrue(TEXT("retained original graph does not advance with speculative live tail"),Retained->ReadLatest()==OriginalCut))return false;
    }
    const auto Expected=Reference->ReadLatest(),Before=Live->ReadLatest(),SpectatorBefore=Spectator->ReadLatest();
    if(!Expected||!Before)return false;double BeforePositionError=0;
    for(size_t Actor=0;Actor<Expected->Actors.size();++Actor)
    {
        const auto* X=Expected->Physics.PresentationBodies.FindByPredicate([&](const auto& V){return V.StableId==Expected->Actors[Actor].NativeStableId;});
        const auto* Y=Before->Physics.PresentationBodies.FindByPredicate([&](const auto& V){return V.StableId==Before->Actors[Actor].NativeStableId;});
        if(!X||!Y)return false;const auto D=P::MeasurePrivateRoadKinematics(X->COMState,Y->COMState);if(!D.Valid)return false;
        BeforePositionError=FMath::Max(BeforePositionError,D.Position);
    }
    if(!TestTrue(TEXT("deliberately withheld request causes actual native position divergence"),BeforePositionError>1))return false;
    if(!TestTrue(TEXT("all actor authority input records form a complete finite prefix"),Journal->CommonContiguousCount()==Last+1))return false;
    const U64 OriginalLifetime=OriginalCut->LocalKernelLifetime;U64 Replayed=0;
    for(U64 N=Cut+1;N<=Last;++N)
    {
        const auto A=Journal->Read(101,N),B=Journal->Read(102,N);if(!A||!B)return false;
        if(CA.Confirm(5,*A).Status!=EResult::Accepted||CB.Confirm(5,*B).Status!=EResult::Accepted)return false;
        if(!Retained->Step({{101,3,CA.Predict(N)},{102,3,CB.Predict(N)}},true,Why))return false;
        ++Replayed;
    }
    auto ConsumedGraph=std::move(Retained);const auto Corrected=ConsumedGraph->ReadLatest();
    TestTrue(TEXT("consume the actual retained graph once, without copy/retag/reconstruction"),!Retained&&Corrected&&Corrected->LocalKernelLifetime==OriginalLifetime);
    TestTrue(TEXT("native replay starts strictly after real held cut, not from birth"),Replayed==Last-Cut&&Replayed<Last+1);
    TestTrue(TEXT("all six COM fields, processing and startup values match reference"),Corrected&&SameObservable(*Expected,*Corrected));
    TestTrue(TEXT("separate spectator kernel received no fast-simulation transition"),Spectator->ReadLatest()==SpectatorBefore&&SpectatorBefore->Frame==Last);
    TestTrue(TEXT("original immutable cut survives actual graph advancement"),OriginalCut->Frame==Cut&&OriginalCut->Actors[0].Filter.Next==Cut+1);
    TestTrue(TEXT("all original native graphs retire through their actual owning API"),Reference->Close()&&Live->Close()&&ConsumedGraph->Close()&&Spectator->Close());
    if(HasAnyErrors())return false;
    AddInfo(FString::Printf(TEXT("[RetainedNativeGraphReplay] actualNative=1 heldFrame=%llu throughFrame=%llu replayedFrames=%llu fromBirthFrames=%llu beforePositionErrorCm=%.17g retainedLifetimePreserved=1 spectatorFastSteps=0 sixCOMExact=1 filterExact=1 completeStateCertificate=0 sameWorldLocalization=0 network=0"),Cut,Last,Replayed,Last+1,BeforePositionError));
    AddInfo(TEXT("Controlled retained native-graph experiment only. All actors of the correction world participate; the spectator is in a DIFFERENT world. No same-world island exclusion, source/lifetime/config certificate, complete restore lease, runtime checkpoint adoption or two-client game admission is claimed."));
    return !HasAnyErrors();
}
#endif
