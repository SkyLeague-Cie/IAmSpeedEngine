#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadPhysicsWorld.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadNativeObjects.h"
#include "IAmSpeed/Driving/DrivingInput.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASPrivateRoadRegistryBoundary,"IAmSpeed.Netcode.Prediction.PrivateRoadRegistryBoundary",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASPrivateRoadRegistryBoundary::RunTest(const FString&)
{
    namespace P=Speed::Netcode::Prediction;using namespace Speed::Netcode;
    TSet<UWorld*> Before;for(TObjectIterator<UWorld> It;It;++It)if(!It->IsTemplate())Before.Add(*It);
    P::FPrivateRoadCarSeed Seed;Seed.Actor=101;Seed.Incarnation=3;
    Seed.Contract=Speed::Driving::MakeContract(Speed::Driving::FBindings{});
    Seed.Local={Speed::Input::EProducerKind::Device,42};Seed.LocalEpoch={3};
    auto& B=Seed.Remote;B.Session=11;B.Epoch=3;B.Actor=101;B.Incarnation=3;B.OwnerPeer=4;B.ServerPeer=5;B.SourceProducer=42;B.SourceKind=0;B.Contract[0]=7;
    Seed.Initial.Location=FVector(0,0,45);
    FString Why;auto Owner=P::FPrivateRoadPhysicsWorld::Create({Seed},Why);
    if(!TestTrue(*FString::Printf(TEXT("original private native graph: %s"),*Why),bool(Owner)))return false;
    // Use the REAL global enumeration path, not a private owner test accessor.
    // Finding it is evidence that this negative test does NOT close that path.
    UWorld* EscapedWorld=nullptr;
    for(TObjectIterator<UWorld> It;It;++It)if(!It->IsTemplate()&&!Before.Contains(*It))
    {if(!TestTrue(TEXT("one actual new native world"),!EscapedWorld))return false;EscapedWorld=*It;}
    if(!TestTrue(TEXT("actual world remains globally discoverable"),bool(EscapedWorld)))return false;
    ASpeedBranchCar* Car=nullptr;for(TActorIterator<ASpeedBranchCar> It(EscapedWorld);It;++It)Car=*It;
    auto* Component=Car?Cast<USpeedBranchWheeledComponent>(Car->GetVehicleMovement()):nullptr;
    auto* Bridge=EscapedWorld->GetSubsystem<USpeedWorldSubsystem>();
    if(!TestTrue(TEXT("actual component and registry escaped via supported Engine enumeration"),Component&&Bridge))return false;
    std::array<Consumed::FActionPolicy,ActionCount> Actions{};Actions[0]={true,true,false,255};Actions[1]={true,true,false,255};Actions[2]={true,true,true,127};
    P::FRemoteRequestHistory History(B,Actions,8,8);
    FPacket Packet;Packet.Kind=EKind::AuthorityInput;Packet.Session=B.Session;Packet.Epoch=B.Epoch;Packet.Actor=B.Actor;Packet.Incarnation=B.Incarnation;Packet.Contract=B.Contract;Packet.Sequence=1;
    Packet.Input.SourceSequence=1;Packet.Input.DeviceGeneration=1;Packet.Input.ProducerId=42;Packet.Input.ProducerKind=0;Packet.Input.Reset=true;
    if(!TestTrue(TEXT("actual baseline authority request retained"),History.Confirm(5,Packet).Status==EResult::Accepted))return false;
    const std::vector<P::FPrivateRoadRequest> First{{101,3,History.Predict(0)}};
    if(!TestTrue(TEXT("actual frozen native publication0 completed"),Owner->Step(First,false,Why)))return false;
    const auto Baseline=Owner->ReadLatest();if(!Baseline)return false;
    Bridge->UnregisterSpeedComponent(Component); // real pending mutation
    TestFalse(TEXT("pending native unregister blocks initial CanMove release"),Owner->ReleaseInitialPublication(Baseline,Why));
    TestTrue(TEXT("pending mutation explicitly identified"),Why.Contains(TEXT("pending mutation")));
    TestFalse(TEXT("pending unregister refused before duplicate physical step/input preparation"),Owner->Step(First,false,Why));
    TestTrue(TEXT("read-only refusal preserves exact original immutable publication"),Owner->ReadLatest()==Baseline&&!Baseline->Actors[0].CanMove&&Baseline->Frame==0);
    Bridge->ApplyPendingOps(); // actual original registry removal, no fake flag
    TestFalse(TEXT("materialized native unregister also blocks release"),Owner->ReleaseInitialPublication(Baseline,Why));
    TestTrue(TEXT("actual changed registry diagnosed"),Why.Contains(TEXT("registered roster changed")));
    TestFalse(TEXT("materialized roster drift refused before native advance"),Owner->Step(First,false,Why));
    TestTrue(TEXT("original frozen publication retained after both actual failures"),Owner->ReadLatest()==Baseline&&!Baseline->Actors[0].CanMove);
    TestTrue(TEXT("actual owning graph retires on original GT"),Owner->Close());
    AddInfo(TEXT("[IASPrivateRegistryBoundary] actual_pending_unregister_refused=1 actual_registered_drift_refused=1 unchanged_publication=1 global_engine_enumeration_still_open=1 complete_lease=0"));
    return !HasAnyErrors();
}
#endif
