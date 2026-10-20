#include "FrameInputWitness.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFrameInputWitnessTests,
    "IAmSpeed.Simulation.FrameWitness.InputEndpointBytes",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FFrameInputWitnessTests::RunTest(const FString&)
{
    using namespace Speed;using namespace Speed::Input;using namespace Speed::Input::V2;
    FInputActionContractDescription Description;
    Description.Revision={1};Description.Actions=FInputActionContract::BaseActions();
    Description.Physical={{Throttle,EPhysicalDestination::Throttle},{Brake,EPhysicalDestination::Brake},{Steering,EPhysicalDestination::Steering}};
    const auto Contract=FInputActionContract::Create(Description);
    if(!TestTrue(TEXT("real input contract"),bool(Contract)))return false;
    FInputFrameData Data;Data.ConsumptionFrame=7;Data.SourceSequence=8;
    Data.Producer={EProducerKind::Device,1};Data.StreamEpoch={1};Data.DeviceGeneration={1};
    Data.Reset=true;Data.Values[Throttle]=123;Data.ActiveMask=1;
    FOwnerInputSnapshot Snapshot{Speed::Input::V2::FInputFrame(Contract,Data)};
    Snapshot.BindingFingerprint=Contract->GetFingerprint();Snapshot.Requested=Data.Values;
    Snapshot.Applied=Data.Values;Snapshot.CutoffSequence=8;
    auto Registry=std::make_shared<FInputRegistryView>();Registry->Version=1;Registry->WorkerGeneration=1;
    FRegistryFrame A;A.Frame=7;A.Registry=Registry;
    FInstalledSessionInput Input;Input.Session=1;Input.Epoch=1;Input.Actors={{1,1}};
    Input.Snapshot=std::make_shared<FOwnerInputSnapshot>(Snapshot);A.Inputs.push_back(Input);
    FRegistryFrame B=A;B.Inputs[0].Snapshot=std::make_shared<FOwnerInputSnapshot>(Snapshot);
    TArray<uint8> Left,Right;
    TestTrue(TEXT("capture real installed values"),AppendFrameInputWitness(A,EFrameInputWitnessStage::Installed,Left));
    TestTrue(TEXT("capture independently allocated identical snapshot"),AppendFrameInputWitness(B,EFrameInputWitnessStage::Installed,Right));
    TestTrue(TEXT("address-independent exact bytes"),Left==Right);
    auto Changed=Snapshot;Changed.Applied[Throttle]=122;B.Inputs[0].Snapshot=std::make_shared<FOwnerInputSnapshot>(Changed);
    TestTrue(TEXT("changed physically applied values still capture"),AppendFrameInputWitness(B,EFrameInputWitnessStage::Installed,Right));
    TestTrue(TEXT("applied change is not hidden by request equality"),Left!=Right);
    TestTrue(TEXT("completed endpoint capture"),AppendFrameInputWitness(A,EFrameInputWitnessStage::RegistryCompleted,Right));
    TestTrue(TEXT("installed is not mislabeled completed"),Left!=Right);
    B.Inputs[0].Snapshot.reset();Right={99};
    TestFalse(TEXT("missing real endpoint is UNKNOWN"),AppendFrameInputWitness(B,EFrameInputWitnessStage::Installed,Right));
    TestEqual(TEXT("failure retains caller forensic output"),Right[0],uint8(99));
    return true;
}
#endif
