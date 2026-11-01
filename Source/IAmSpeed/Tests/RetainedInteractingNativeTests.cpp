#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadPhysicsWorld.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadNativeObjects.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadKinematicDiagnostics.h"
#include "IAmSpeed/Driving/DrivingInput.h"
#include "Misc/AutomationTest.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "UObject/ObjectKey.h"
#include "Components/PrimitiveComponent.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASRetainedInteractingNative,
    "IAmSpeed.Netcode.Prediction.RetainedInteractingNativeExperiment",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASRetainedInteractingNative::RunTest(const FString&)
{
    namespace P=Speed::Netcode::Prediction;using namespace Speed::Netcode;
    constexpr U64 Cut=10,Change=20,Last=900; // 3s observation, never startup warmup
    auto Contract=Speed::Driving::MakeContract(Speed::Driving::FBindings{});if(!Contract)return false;
    std::vector<P::FPrivateRoadCarSeed> Seeds;
    for(U64 A=0;A<3;++A)
    {
        P::FPrivateRoadCarSeed S;S.Actor=101+A;S.Incarnation=3;S.Contract=Contract;
        S.Local={Speed::Input::EProducerKind::Device,42+A};S.LocalEpoch={3};S.PredictionHorizon=1024;
        auto& B=S.Remote;B.Session=11;B.Epoch=3;B.Actor=S.Actor;B.Incarnation=3;
        B.OwnerPeer=4+A;B.ServerPeer=9;B.SourceProducer=42+A;B.SourceKind=0;B.Contract[0]=7;
        S.Initial.Location=FVector(A==0?-70:70,A==2?5000:0,45);
        // FreezeMovement zeroes velocity by design. Start stationary and drive
        // only with genuine inputs AFTER the synchronized publication0.
        S.Initial.Velocity=FVector::ZeroVector;
        if(A==1)S.Initial.Rotation=FQuat(FVector::UpVector,PI);
        Seeds.push_back(S); // default synchronization gate, no warmup timer
    }
    std::array<Consumed::FActionPolicy,ActionCount> Actions{};
    Actions[0]={true,true,false,255};Actions[1]={true,true,false,255};Actions[2]={true,true,true,127};
    using H=std::vector<std::unique_ptr<P::FRemoteRequestHistory>>;
    const auto Histories=[&]{H Out;for(const auto& S:Seeds)Out.push_back(std::make_unique<P::FRemoteRequestHistory>(S.Remote,Actions,1024,2048));return Out;};
    auto RH=Histories(),LH=Histories(),KH=Histories();
    const auto Value=[](U64 A,U64 N){return A==0&&N>=Change&&((N-Change)/10)%2==0?255:0;};
    const auto Packet=[&](U64 A,U64 N)
    {
        const auto& B=Seeds[A].Remote;FPacket Q;Q.Kind=EKind::AuthorityInput;
        Q.Session=B.Session;Q.Epoch=B.Epoch;Q.Actor=B.Actor;Q.Incarnation=B.Incarnation;Q.Contract=B.Contract;
        Q.Frame=N;Q.Sequence=N+1;Q.Input.SourceSequence=N+1;Q.Input.DeviceGeneration=1;
        Q.Input.ProducerId=B.SourceProducer;Q.Input.ProducerKind=B.SourceKind;Q.Input.Reset=N==0;
        const int T=N?(A==2?127:255):0,V=Value(A,N);
        Q.Input.Values[0]=std::int16_t(T);Q.Input.Values[1]=std::int16_t(V);
        Q.Input.ActiveMask=(T?1:0)|(V?2:0);
        if(N==1)Q.Input.Edges.push_back({0,0,std::int16_t(T),N+1,0});
        if(N&&V!=Value(A,N-1))Q.Input.Edges.push_back({1,std::uint8_t(V?0:1),std::int16_t(V),N+1,std::uint32_t(Q.Input.Edges.size())});
        return Q;
    };
    FString Why;
    auto Reference=P::FPrivateRoadPhysicsWorld::Create(Seeds,Why);
    auto Live=P::FPrivateRoadPhysicsWorld::Create(Seeds,Why),Retained=P::FPrivateRoadPhysicsWorld::Create(Seeds,Why);
    if(!TestTrue(*Why,Reference&&Live&&Retained))return false;
    const auto Same=[&](const auto& X,const auto& Y)
    {
        if(!X||!Y||X->Frame!=Y->Frame||X->Actors.size()!=3||Y->Actors.size()!=3)return false;
        for(size_t A=0;A<3;++A)
        {
            const auto& L=X->Actors[A];const auto& R=Y->Actors[A];
            if(L.Actor!=R.Actor||L.Incarnation!=R.Incarnation||!(L.Filter==R.Filter)||!L.Input||!R.Input
                ||L.Input->Requested!=R.Input->Requested||L.Input->Applied!=R.Input->Applied
                ||L.NativeHitboxHalfExtent!=R.NativeHitboxHalfExtent||L.NativeHitboxCenter!=R.NativeHitboxCenter
                ||L.CanMove!=R.CanMove||L.Frozen!=R.Frozen||L.CountdownFrames!=R.CountdownFrames)return false;
            const auto* B=X->Physics.PresentationBodies.FindByPredicate([&](const auto& V){return V.StableId==L.NativeStableId;});
            const auto* C=Y->Physics.PresentationBodies.FindByPredicate([&](const auto& V){return V.StableId==R.NativeStableId;});
            if(!B||!C)return false;const auto D=P::MeasurePrivateRoadKinematics(B->COMState,C->COMState);
            if(!D.Valid||D.Different)return false;
        }
        return true;
    };
    const auto Requests=[&](H& Hist,U64 N)
    {std::vector<P::FPrivateRoadRequest> Out;for(size_t A=0;A<3;++A)Out.push_back({Seeds[A].Actor,3,Hist[A]->Predict(N)});return Out;};
    uint64 DynamicContactFrames=0,FirstContact=0;
    double NearestCOM=MAX_dbl,MaximumTravel=0;
    FVector StartCOM=FVector::ZeroVector;
    std::shared_ptr<const P::FPrivateRoadPublication> OriginalCut;
    std::vector<std::vector<P::FPrivateRoadContactDispatchPair>> ActualReferenceContactDispatches;
    for(U64 N=0;N<=Last;++N)
    {
        for(size_t A=0;A<3;++A)
        {
            const auto Q=Packet(A,N);
            if(RH[A]->Confirm(9,Q).Status!=EResult::Accepted||KH[A]->Confirm(9,Q).Status!=EResult::Accepted)return false;
            if((A!=0||N<Change)&&LH[A]->Confirm(9,Q).Status!=EResult::Accepted)return false;
        }
        if(!Reference->Step(Requests(RH,N),false,Why)||!Live->Step(Requests(LH,N),false,Why))return false;
        if(N<=Cut&&!Retained->Step(Requests(KH,N),false,Why))return false;
        if(N==0)
        {
            const auto R=Reference->ReadLatest(),L=Live->ReadLatest(),K=Retained->ReadLatest();
            if(!TestTrue(TEXT("actual synchronized frozen native baseline"),Same(R,L)&&Same(R,K)))return false;
            if(!Reference->ReleaseInitialPublication(R,Why)||!Live->ReleaseInitialPublication(L,Why)||!Retained->ReleaseInitialPublication(K,Why))return false;
        }
        if(N==Cut){OriginalCut=Retained->ReadLatest();if(!Same(Reference->ReadLatest(),OriginalCut))return false;}
        const auto Actual=Reference->ReadLatest();
        if(!TestTrue(TEXT("actual native contact dispatch observation remains complete"),Actual&&Actual->ContactDispatchObservationComplete))return false;
        ActualReferenceContactDispatches.push_back(Actual->ContactDispatchPairs);
        const auto* BA=Actual->Physics.PresentationBodies.FindByPredicate([&](const auto& V){return V.StableId==Actual->Actors[0].NativeStableId;});
        const auto* BB=Actual->Physics.PresentationBodies.FindByPredicate([&](const auto& V){return V.StableId==Actual->Actors[1].NativeStableId;});
        if(!BA||!BB)return false;
        if(N==0)StartCOM=BA->COMState.Location;
        NearestCOM=FMath::Min(NearestCOM,FVector::Distance(BA->COMState.Location,BB->COMState.Location));
        MaximumTravel=FMath::Max(MaximumTravel,FVector::Distance(StartCOM,BA->COMState.Location));
        if(N==0||N==10||N==20||N==60||N==120||N==180||N==360||N==600||N==Last)
            for(size_t A=0;A<Actual->Actors.size();++A)
            {
                const auto& F=Actual->Actors[A];
                const auto* Body=Actual->Physics.PresentationBodies.FindByPredicate([&](const auto& V){return V.StableId==F.NativeStableId;});
                if(!Body)return false;const auto& K=Body->COMState;
                AddInfo(FString::Printf(TEXT("[IASInteractionTrajectory] frame=%llu actor=%llu x=%.17g y=%.17g z=%.17g vx=%.17g vy=%.17g vz=%.17g hitbox_x=%.17g hitbox_y=%.17g hitbox_z=%.17g half_x=%.17g half_y=%.17g half_z=%.17g can_move=%d frozen=%d"),N,F.Actor,K.Location.X,K.Location.Y,K.Location.Z,K.Velocity.X,K.Velocity.Y,K.Velocity.Z,F.NativeHitboxCenter.X,F.NativeHitboxCenter.Y,F.NativeHitboxCenter.Z,F.NativeHitboxHalfExtent.X,F.NativeHitboxHalfExtent.Y,F.NativeHitboxHalfExtent.Z,int(F.CanMove),int(F.Frozen)));
            }
        bool Contact=false;
        for(const auto& Pair:Actual->ContactDispatchPairs)
            if(Pair.ActorA==101&&Pair.ActorB==102&&Pair.Dispatches>0)Contact=true;
        if(Contact){if(!DynamicContactFrames)FirstContact=N;++DynamicContactFrames;}
    }
    AddInfo(FString::Printf(TEXT("[IASInteractingFixtureMotion] native=1 default_sync_gate=1 throttle_after_release=1 nearest_com_cm=%.17g max_actor0_travel_cm=%.17g actual_box_contact_dispatch_frames=%llu"),NearestCOM,MaximumTravel,DynamicContactFrames));
    if(!TestTrue(TEXT("real post-release throttle moves native car"),MaximumTravel>1))return false;
    const auto BeforeCorrection=Live->ReadLatest(),Truth=Reference->ReadLatest();
    double MaximumBefore=0;
    for(size_t A=0;A<3;++A)
    {
        const auto* B=BeforeCorrection->Physics.PresentationBodies.FindByPredicate([&](const auto& V){return V.StableId==BeforeCorrection->Actors[A].NativeStableId;});
        const auto* C=Truth->Physics.PresentationBodies.FindByPredicate([&](const auto& V){return V.StableId==Truth->Actors[A].NativeStableId;});
        if(!B||!C)return false;MaximumBefore=FMath::Max(MaximumBefore,P::MeasurePrivateRoadKinematics(B->COMState,C->COMState).Position);
    }
    AddInfo(FString::Printf(TEXT("[IASInteractionBeforeCorrection] frame=%llu max_position_cm=%.17g"),Last,MaximumBefore));
    if(!TestTrue(TEXT("actual native blocking box/box dispatch between different cars, not optional constraints or floor contact"),DynamicContactFrames>0))return false;
    if(!TestTrue(TEXT("withheld oscillating input caused measurable native divergence"),MaximumBefore>0))return false;
    if(!TestTrue(TEXT("actual retained graph stayed at original K without stepping"),Retained->ReadLatest()==OriginalCut))return false;
    for(U64 N=Cut+1;N<=Last;++N)
    {
        if(!Retained->Step(Requests(KH,N),true,Why))return false;
        const auto Actual=Retained->ReadLatest();
        if(!TestTrue(TEXT("actual corrected native blocking contact dispatch sequence matches independent reference"),Actual&&Actual->ContactDispatchObservationComplete&&Actual->ContactDispatchPairs==ActualReferenceContactDispatches[size_t(N)]))return false;
    }
    if(!TestTrue(TEXT("same original native lifetime, three actors exact after K+1..N"),Retained->ReadLatest()->LocalKernelLifetime==OriginalCut->LocalKernelLifetime&&Same(Truth,Retained->ReadLatest())))return false;
    // ALL THREE actors stepped during this experiment, including the distant
    // same-world spectator. This must NOT be reported as localized replay.
    AddInfo(FString::Printf(TEXT("[IASRetainedInteracting] native=1 actors=3 held_frame=%llu through_frame=%llu replayed_frames=%llu actual_box_contact_dispatch_frames=%llu first_actual_box_contact_dispatch=%llu before_max_position_cm=%.17g exact_six_com_and_filters=1 spectator_replay_frames=%llu same_world_localized=0 full_state_certificate=0 network=0"),Cut,Last,Last-Cut,DynamicContactFrames,FirstContact,MaximumBefore,Last-Cut));
    return !HasAnyErrors();
}
#endif
