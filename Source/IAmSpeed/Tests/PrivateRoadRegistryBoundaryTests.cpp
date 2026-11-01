#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadPhysicsWorld.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadNativeObjects.h"
#include "IAmSpeed/Driving/DrivingInput.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include "UObject/ObjectKey.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "WheelSystem.h"
#include "IAmSpeed/Base/CarBumpSnapshot.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadKinematicDiagnostics.h"

namespace
{
struct FForbiddenPrivatePhaseObserver final : ISpeedPhysicsPhaseObserver
{
    int32 Calls=0;
    void Observe(const ISpeedComponent&,ESpeedPhysicsObservationPhase,float) override {++Calls;}
};
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASPrivateRoadRegistryBoundary,"IAmSpeed.Netcode.Prediction.PrivateRoadRegistryBoundary",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASPrivateRoadRegistryBoundary::RunTest(const FString&)
{
    namespace P=Speed::Netcode::Prediction;using namespace Speed::Netcode;
    TSet<FObjectKey> Before;for(TObjectIterator<UWorld> It;It;++It)if(!It->IsTemplate())Before.Add(FObjectKey(*It));
    UWorld* CallbackWorld=nullptr;USpeedBranchWheeledComponent* CallbackComponent=nullptr;
    int32 ActualCleanupCallbacks=0,ActualCleanupBoundWheels=0;bool ActualCleanupRead=false;
    const Chaos::FSimpleWheelSim* OriginalWheelPointers[4]{};
    const Chaos::FSimpleSuspensionSim* OriginalSuspensionPointers[4]{};
    TWeakObjectPtr<USWheelSubBody> OriginalWheelObjects[4];
    const auto CreationHandle=FWorldDelegates::OnPostWorldCreation.AddLambda([&](UWorld* Created)
    {
        // Real callback registered BEFORE private native birth. Do not inspect
        // partially initialized world internals or run on a foreign thread.
        if(IsInGameThread()&&Created&&!Before.Contains(FObjectKey(Created)))CallbackWorld=Created;
    });
    const auto CleanupHandle=FWorldDelegates::OnWorldCleanup.AddLambda([&](UWorld* Retiring,bool,bool)
    {
        if(!IsInGameThread()||Retiring!=CallbackWorld)return;
        ++ActualCleanupCallbacks;
        if(IsValid(CallbackComponent))
        {
            // A real supported native read through the retained original
            // component, DURING actual owner retirement. This is a remaining
            // external reader, never an invented exclusion/drain receipt.
            const SKinematic Copy=CallbackComponent->GetKinematicState();
            ActualCleanupRead=!Copy.Location.ContainsNaN()&&!Copy.Velocity.ContainsNaN();
            const auto& Wheels=CallbackComponent->GetWheelSubBodies();
            for(int32 W=0;W<Wheels.Num()&&W<4;++W)
                if(Wheels[W]&&Wheels[W]->GetWheelSim()==OriginalWheelPointers[W]
                    &&Wheels[W]->GetSuspensionSim()==OriginalSuspensionPointers[W]
                    &&OriginalWheelPointers[W]&&OriginalSuspensionPointers[W]
                    &&FMath::IsFinite(OriginalWheelPointers[W]->GetAngularVelocity()))++ActualCleanupBoundWheels;
        }
    });
    ON_SCOPE_EXIT{FWorldDelegates::OnPostWorldCreation.Remove(CreationHandle);FWorldDelegates::OnWorldCleanup.Remove(CleanupHandle);};
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
    for(TObjectIterator<UWorld> It;It;++It)if(!It->IsTemplate()&&!Before.Contains(FObjectKey(*It)))
    {if(!TestTrue(TEXT("one actual new native world"),!EscapedWorld))return false;EscapedWorld=*It;}
    if(!TestTrue(TEXT("actual world remains globally discoverable"),bool(EscapedWorld)))return false;
    if(!TestTrue(TEXT("actual global birth callback retained the SAME original world"),CallbackWorld==EscapedWorld))return false;
    ASpeedBranchCar* Car=nullptr;for(TActorIterator<ASpeedBranchCar> It(EscapedWorld);It;++It)Car=*It;
    auto* Component=Car?Cast<USpeedBranchWheeledComponent>(Car->GetVehicleMovement()):nullptr;
    auto* Bridge=EscapedWorld->GetSubsystem<USpeedWorldSubsystem>();
    if(!TestTrue(TEXT("actual component and registry escaped via supported Engine enumeration"),Component&&Bridge))return false;
    CallbackComponent=Component;
    // Exercise the actual BASE entry, including its inline/native dispatch;
    // hiding a same-named method in the subtype would not protect this call.
    ISpeedComponent& Adapter=*Component;FForbiddenPrivatePhaseObserver ForbiddenObserver;
    TestFalse(TEXT("actual private base entry refuses installing an external physical-phase callback"),Adapter.TryInstallPhysicsPhaseObserver(ForbiddenObserver));
    TestFalse(TEXT("actual base observer storage remains empty"),Adapter.ISpeedComponent::IsPhysicsPhaseObservationEnabled());
    const SKinematic BeforeLegacy=Adapter.GetKinematicState();const bool FrozenBeforeLegacy=Adapter.IsFrozen();
    SKinematic WrongLegacy=BeforeLegacy;WrongLegacy.Location.X+=999;
    TArray<uint8> ActualMechanicPayload;Adapter.AppendSimulationSnapshot(ActualMechanicPayload);
    std::uint16_t ActualCooldown=0;std::uint64_t ActualVictim=0;
    TestTrue(TEXT("real native captured mechanic payload decodes, not an invalid empty envelope"),
        Speed::Collision::DecodeCarBumpSnapshot(ActualMechanicPayload.GetData(),ActualMechanicPayload.Num(),ActualCooldown,ActualVictim));
    TestFalse(TEXT("private component refuses even its genuine captured legacy snapshot payload"),Adapter.CanRestoreSimulationSnapshot(ActualMechanicPayload));
    Adapter.RestoreSimulationSnapshot(WrongLegacy,!FrozenBeforeLegacy,ActualMechanicPayload);
    const auto Refused=P::MeasurePrivateRoadKinematics(BeforeLegacy,Adapter.GetKinematicState());
    TestTrue(TEXT("real direct legacy restore is refused before native kinematic/frozen writes"),Refused.Valid&&!Refused.Different&&Adapter.IsFrozen()==FrozenBeforeLegacy);
    const auto& InitialWheels=Component->GetWheelSubBodies();
    if(!TestTrue(TEXT("four genuine originally bound native wheels"),InitialWheels.Num()==4))return false;
    for(int32 W=0;W<4;++W)
    {
        OriginalWheelObjects[W]=InitialWheels[W];OriginalWheelPointers[W]=InitialWheels[W]->GetWheelSim();OriginalSuspensionPointers[W]=InitialWheels[W]->GetSuspensionSim();
        if(!TestTrue(TEXT("actual original wheel/suspension storage, not a copied pointer tag"),OriginalWheelPointers[W]&&OriginalSuspensionPointers[W]))return false;
    }
    std::array<Consumed::FActionPolicy,ActionCount> Actions{};Actions[0]={true,true,false,255};Actions[1]={true,true,false,255};Actions[2]={true,true,true,127};
    P::FRemoteRequestHistory History(B,Actions,8,8);
    FPacket Packet;Packet.Kind=EKind::AuthorityInput;Packet.Session=B.Session;Packet.Epoch=B.Epoch;Packet.Actor=B.Actor;Packet.Incarnation=B.Incarnation;Packet.Contract=B.Contract;Packet.Sequence=1;
    Packet.Input.SourceSequence=1;Packet.Input.DeviceGeneration=1;Packet.Input.ProducerId=42;Packet.Input.ProducerKind=0;Packet.Input.Reset=true;
    if(!TestTrue(TEXT("actual baseline authority request retained"),History.Confirm(5,Packet).Status==EResult::Accepted))return false;
    const std::vector<P::FPrivateRoadRequest> First{{101,3,History.Predict(0)}};
    if(!TestTrue(TEXT("actual frozen native publication0 completed"),Owner->Step(First,false,Why)))return false;
    const auto Baseline=Owner->ReadLatest();if(!Baseline)return false;
    TestEqual(TEXT("refused observer was never invoked by the real native step"),ForbiddenObserver.Calls,0);
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
    TestTrue(TEXT("original owner retirement actually invoked retained global world callback"),ActualCleanupCallbacks>0);
    TestTrue(TEXT("actual original native wheel storage stays readable through the synchronous cleanup callback"),ActualCleanupBoundWheels==4);
    for(const auto& Weak:OriginalWheelObjects)if(const auto* Wheel=Weak.Get())
        TestTrue(TEXT("surviving native UObject has wheel/suspension pointers cleared AFTER callback returned"),!Wheel->GetWheelSim()&&!Wheel->GetSuspensionSim());
    // Never dereference OriginalWheelPointers after Close: storage has retired.
    // The unknown future borrowers remain unclosed; complete_lease stays0.
    AddInfo(FString::Printf(TEXT("[IASPrivateRegistryBoundary] actual_direct_legacy_restore_refused=1 actual_external_phase_observer_refused=1 actual_pending_unregister_refused=1 actual_registered_drift_refused=1 unchanged_publication=1 actual_global_birth_callback=1 actual_cleanup_callbacks=%d actual_cleanup_native_read=%d actual_cleanup_original_wheels=%d global_engine_enumeration_still_open=1 complete_lease=0"),ActualCleanupCallbacks,int(ActualCleanupRead),ActualCleanupBoundWheels));
    return !HasAnyErrors();
}
#endif
