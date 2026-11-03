#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadPhysicsWorld.h"
#include "IAmSpeed/Driving/DrivingInput.h"
#include "Misc/AutomationTest.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASPrivateRoadShapeHitAcceptance,
    "IAmSpeed.Netcode.Prediction.PrivateRoadShapeHitAcceptance",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASPrivateRoadShapeHitAcceptance::RunTest(const FString&)
{
    namespace P=Speed::Netcode::Prediction;using namespace Speed::Netcode;
    if(!TestTrue(TEXT("native shape controls execute on GT"),IsInGameThread()))return false;
    auto Contract=Speed::Driving::MakeContract(Speed::Driving::FBindings{});if(!Contract)return false;
    std::vector<P::FPrivateRoadCarSeed> Seeds;
    for(U64 Index=0;Index<2;++Index)
    {
        P::FPrivateRoadCarSeed Seed;Seed.Actor=101+Index;Seed.Incarnation=3;Seed.Contract=Contract;
        Seed.StartPolicy=P::EPrivateRoadStartPolicy::ExplicitUnitRecipe;
        Seed.Local={Speed::Input::EProducerKind::Device,42+Index};Seed.LocalEpoch={3};
        Seed.Remote.Session=1;Seed.Remote.Epoch=2;Seed.Remote.Actor=Seed.Actor;Seed.Remote.Incarnation=3;
        Seed.Remote.OwnerPeer=4;Seed.Remote.ServerPeer=5;Seed.Remote.SourceProducer=6+Index;Seed.Remote.SourceKind=0;Seed.Remote.Contract[0]=7;
        Seed.Initial.Location=FVector(0,Index?1000:-1000,45);Seeds.push_back(Seed);
    }
    FString Why;auto Own=P::FPrivateRoadPhysicsWorld::Create(Seeds,Why);
    auto Foreign=P::FPrivateRoadPhysicsWorld::Create(Seeds,Why);auto Reference=P::FPrivateRoadPhysicsWorld::Create(Seeds,Why);
    if(!TestTrue(*FString::Printf(TEXT("three genuine private graphs: %s"),*Why),Own&&Foreign&&Reference))return false;
    std::array<Consumed::FActionPolicy,ActionCount> Actions{};
    Actions[0]={true,true,false,255};Actions[1]={true,true,false,255};Actions[2]={true,true,true,127};
    P::FRemoteRequestHistory HA(Seeds[0].Remote,Actions,256,512),HB(Seeds[1].Remote,Actions,256,512);
    const auto Packet=[&](size_t Actor)
    {
        const auto& Bind=Seeds[Actor].Remote;FPacket Q;Q.Kind=EKind::AuthorityInput;
        Q.Session=Bind.Session;Q.Epoch=Bind.Epoch;Q.Actor=Bind.Actor;Q.Incarnation=Bind.Incarnation;Q.Contract=Bind.Contract;
        Q.Frame=0;Q.Sequence=1;Q.Input.SourceSequence=1;Q.Input.DeviceGeneration=1;Q.Input.Reset=true;
        Q.Input.ProducerId=Bind.SourceProducer;Q.Input.ProducerKind=Bind.SourceKind;return Q;
    };
    if(HA.Confirm(5,Packet(0)).Status!=EResult::Accepted||HB.Confirm(5,Packet(1)).Status!=EResult::Accepted)return false;
    std::vector<P::FPrivateRoadRequest> Requests={{101,3,HA.Predict(0)},{102,3,HB.Predict(0)}};
    P::FPrivateRoadHitAcceptanceProbe Probe;
    if(!TestTrue(*FString::Printf(TEXT("controls inside real Step, followed by canonical advance: %s"),*Why),
        Own->StepWithHitAcceptanceProbeForDiagnostics(Requests,false,*Foreign,Probe,Why)))return false;
    TestTrue(TEXT("actual Step operation, not a minted diagnostic permission"),Probe.ActualStepOperation);
    TestTrue(TEXT("Birth-only initialization refuses actual Step before write"),Probe.BirthInitializationRefusedDuringStep);
    TestEqual(TEXT("eight shape-specific cases"),Probe.Cases.size(),size_t(8));
    TestEqual(TEXT("eight actual legacy dispatches"),Probe.LegacyCalls,uint32(8));
    TestEqual(TEXT("eight actual private native dispatches"),Probe.NativeCalls,uint32(8));
    for(const auto& Case:Probe.Cases)
    {
        TestTrue(*(Case.Name+TEXT(" legacy expected behavior")),Case.LegacyExpected);
        TestTrue(*(Case.Name+TEXT(" native expected behavior")),Case.NativeExpected);
        TestTrue(*(Case.Name+TEXT(" actual named body values equal")),Case.LegacyNativeEqual);
    }
    TestTrue(TEXT("unchanged base differs from Box and Wheel"),Probe.BaseBehaviorDistinguished);
    TestEqual(TEXT("foreign subject, peer and parent rejected"),Probe.Refusals,uint32(3));
    TestTrue(TEXT("each rejection precedes that entry's first write"),Probe.AllRefusalsBeforeWrite);
    if(!TestTrue(TEXT("foreign graph remains usable"),Foreign->Step(Requests,false,Why))
        ||!TestTrue(TEXT("independent unchanged reference advances"),Reference->Step(Requests,false,Why)))return false;
    auto O=Own->ReadLatest(),F=Foreign->ReadLatest(),R=Reference->ReadLatest();
    const auto Same=[](const auto& L,const auto& RR)
    {
        if(!L||!RR||L->Frame!=RR->Frame||L->Physics.PresentationBodies.Num()!=RR->Physics.PresentationBodies.Num())return false;
        for(int32 Index=0;Index<L->Physics.PresentationBodies.Num();++Index)
        {
            const auto& X=L->Physics.PresentationBodies[Index].COMState;const auto& Y=RR->Physics.PresentationBodies[Index].COMState;
            if(X.Location!=Y.Location||X.Velocity!=Y.Velocity||X.Acceleration!=Y.Acceleration
                ||X.Rotation!=Y.Rotation||X.AngularVelocity!=Y.AngularVelocity||X.AngularAcceleration!=Y.AngularAcceleration)return false;
        }
        return true;
    };
    TestTrue(TEXT("fixture cleanup leaves same-frame COM trajectory equal"),Same(O,R)&&Same(F,R));
    TestTrue(TEXT("all three original owners close"),Own->Close()&&Foreign->Close()&&Reference->Close());
    AddInfo(TEXT("[PrivateRoadShapeHitAcceptance] cases=8 legacyCalls=8 nativeCalls=8 foreignRefusals=3 baseDistinct=1 actualStep=1. Authored contact-cache fixtures, not acquired collision/source coverage, complete state, restoration lease or connected rollback."));
    return !HasAnyErrors();
}
#endif
