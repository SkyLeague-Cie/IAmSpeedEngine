#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadPhysicsWorld.h"
#include "IAmSpeed/Driving/DrivingInput.h"
#include "Misc/AutomationTest.h"
#include "Engine/World.h"
#include "UObject/UObjectIterator.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASPrivateRoadContractAdmission,
    "IAmSpeed.Netcode.Prediction.PrivateRoadContractAdmission",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASPrivateRoadContractAdmission::RunTest(const FString&)
{
    namespace P=Speed::Netcode::Prediction;namespace V=Speed::Input::V2;
    const auto Stock=Speed::Driving::MakeContract(Speed::Driving::FBindings{});if(!Stock)return false;
    const auto WorldCount=[](){int32 N=0;for(TObjectIterator<UWorld> It;It;++It)if(!It->IsTemplate())++N;return N;};
    const auto Seeds=[&](std::shared_ptr<const V::FInputActionContract> Contract)
    {
        std::vector<P::FPrivateRoadCarSeed> Out;
        for(Speed::Netcode::U64 A=0;A<2;++A)
        {
            P::FPrivateRoadCarSeed S;S.Actor=101+A;S.Incarnation=3;S.Contract=Contract;
            S.Local={Speed::Input::EProducerKind::Device,42+A};S.LocalEpoch={3};
            S.Remote.Session=1;S.Remote.Epoch=2;S.Remote.Actor=S.Actor;S.Remote.Incarnation=3;
            S.Remote.OwnerPeer=4;S.Remote.ServerPeer=5;S.Remote.SourceProducer=6+A;S.Remote.SourceKind=0;S.Remote.Contract[0]=7;
            S.Initial.Location=FVector(0,A?1000:-1000,45);Out.push_back(S);
        }
        return Out;
    };
    for(int32 Case=0;Case<5;++Case)
    {
        auto D=Stock->GetDescription();
        switch(Case)
        {
        case 0:std::swap(D.Physical[0].Action,D.Physical[1].Action);break;
        case 1:D.Actions[5].Quantization=1024;break;
        case 2:D.Actions[6].Quantization=1024;break;
        case 3:D.Actions[5].Name="CameraZoom";break;
        case 4:D.Actions[6].Signed=false;D.Actions[6].Quantization=255;break;
        }
        const auto Other=V::FInputActionContract::Create(std::move(D));
        if(!TestTrue(TEXT("counterexample remains a valid generic input contract"),bool(Other)))return false;
        if(Case==1)
        {
            V::FInputFrameData F;F.ConsumptionFrame=0;F.SourceSequence=1;F.Producer={Speed::Input::EProducerKind::Device,42};
            F.StreamEpoch={3};F.DeviceGeneration={1};F.Reset=true;F.Values[5]=256;F.ActiveMask=1u<<5;
            TestTrue(TEXT("wide camera value is valid raw evidence in its declared contract"),V::FInputFrame(Other,F).IsValidFor(*Other));
        }
        const auto Before=WorldCount();FString Why;
        auto World=P::FPrivateRoadPhysicsWorld::Create(Seeds(Other),Why);
        TestFalse(TEXT("fixed native road consumer refuses unsupported physical/camera interpretation"),bool(World));
        TestEqual(TEXT("refusal occurs before native world allocation"),WorldCount(),Before);
        TestTrue(TEXT("explicit contract admission diagnostic"),Why.Contains(TEXT("unsupported private road action contract")));
    }
    auto Rebound=Stock->GetDescription();++Rebound.Revision.Value;
    Rebound.Mapping[0].Control.Code=0x05; // legal raw rebinding, same consumed semantic contract
    const auto ReboundContract=V::FInputActionContract::Create(std::move(Rebound));
    if(!TestTrue(TEXT("raw control customization remains valid"),bool(ReboundContract)))return false;
    FString Why;auto World=P::FPrivateRoadPhysicsWorld::Create(Seeds(ReboundContract),Why);
    if(!TestTrue(*FString::Printf(TEXT("native semantic recipe permits raw control rebinding: %s"),*Why),bool(World)))return false;
    TestTrue(TEXT("actual accepted graph closes on owning thread"),World->Close());
    AddInfo(TEXT("Valid generic contracts with swapped throttle/brake destinations, wide camera axes and changed camera semantics are refused before native allocation. No silent clamp or coefficient change; legal raw rebinding remains supported."));
    return !HasAnyErrors();
}
#endif
