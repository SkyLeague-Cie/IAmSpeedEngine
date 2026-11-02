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
#include "Misc/OutputDevice.h"
#include "Misc/OutputDeviceRedirector.h"
#include "HAL/IConsoleManager.h"
#include "IAmSpeed/IAmSpeed.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadPartialStateComparison.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadKinematicDiagnostics.h"

namespace
{
// Actual GLog output device, registered through the supported Engine entry.
// The existing physical UpdateFrameState debug log is the triggering call;
// no new native callback/hook or private-field access is invented for the test.
struct FActualPhysicalLogFacadeAttack final : FOutputDevice
{
    USpeedBranchWheeledComponent* Component=nullptr;
    bool WithinOriginalStepCall=false;
    int32 Calls=0;
    const SKinematic* RetainedView=nullptr;
    double WrittenX=0;
    bool CanBeUsedOnAnyThread() const override {return true;}
    bool CanBeUsedOnMultipleThreads() const override {return true;}
    void Serialize(const TCHAR* Text,ELogVerbosity::Type,const FName&) override
    {
        // Unbuffered Engine dispatch can arrive concurrently. Only the GT
        // ever reads/writes the fixture fields; foreign calls return first.
        if(!IsInGameThread())return;
        if(Calls||!Component||!WithinOriginalStepCall
            ||!FCString::Strstr(Text,TEXT("[UpdateFrameState]")))return;
        ++Calls;RetainedView=&Component->GetKinematicState();
        SKinematic Intrusion=*RetainedView;Intrusion.Location.X+=864200;WrittenX=Intrusion.Location.X;
        Component->SetKinematicState(Intrusion);
        Component->SetPhysCOMVelocity(FVector(1000000,2000000,3000000));
    }
};
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
    // The earlier callbacks measured the genuine original retirement. Remove
    // them before constructing this independent storage-isolation scenario.
    FWorldDelegates::OnPostWorldCreation.Remove(CreationHandle);
    FWorldDelegates::OnWorldCleanup.Remove(CleanupHandle);
    TSet<FObjectKey> ExistingWorlds;
    for(TObjectIterator<UWorld> It;It;++It)if(!It->IsTemplate())ExistingWorlds.Add(FObjectKey(*It));
    auto Isolated=P::FPrivateRoadPhysicsWorld::Create({Seed},Why);
    if(!TestTrue(TEXT("separate original component-storage graph created"),bool(Isolated)))return false;
    USpeedBranchWheeledComponent* EscapedFacade=nullptr;
    for(TObjectIterator<USpeedBranchWheeledComponent> It;It;++It)
        if(!It->IsTemplate()&&It->GetWorld()&&!ExistingWorlds.Contains(FObjectKey(It->GetWorld())))
        {if(EscapedFacade)return false;EscapedFacade=*It;}
    if(!TestTrue(TEXT("real native enumeration obtains the isolated facade"),bool(EscapedFacade)))return false;
    auto Control=P::FPrivateRoadPhysicsWorld::Create({Seed},Why);
    if(!Control||!Isolated->Step(First,false,Why)||!Control->Step(First,false,Why))return false;
    const auto OriginalZero=Isolated->ReadLatest(),ControlZero=Control->ReadLatest();
    if(!OriginalZero||!ControlZero)return false;
    ISpeedComponent& FacadeBase=*EscapedFacade;
    const SKinematic& RetainedFacadeView=FacadeBase.GetKinematicState();
    SKinematic Intrusion=RetainedFacadeView;Intrusion.Location.X+=123456;
    static_cast<USpeedWheeledComponent&>(*EscapedFacade).SetKinematicState(Intrusion);
    // The legacy live-input setters intentionally ensure/refuse. Do not call
    // or weaken them; physical inputs still come only from retained evidence.
    FacadeBase.UnFreezeMovement();
    FacadeBase.SetPhysCOMVelocity(FVector(1000000,0,0));
    TestTrue(TEXT("supported public velocity mutation actually changed facade values"),FacadeBase.GetPhysCOMVelocity().Size()>1000);
    TestEqual(TEXT("escaped base setter writes only its facade view"),RetainedFacadeView.Location.X,Intrusion.Location.X);
    if(!TestTrue(TEXT("actual synchronization release reads original physical storage"),
        Isolated->ReleaseInitialPublication(OriginalZero,Why)&&Control->ReleaseInitialPublication(ControlZero,Why)))return false;
    FPacket Next=Packet;Next.Frame=1;Next.Sequence=2;Next.Input.SourceSequence=2;Next.Input.Reset=false;
    if(History.Confirm(5,Next).Status!=EResult::Accepted)return false;
    const std::vector<P::FPrivateRoadRequest> Second{{101,3,History.Predict(1)}};
    auto* DebugKinematics=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.DebugKinematics"));
    if(!DebugKinematics||!GLog)return false;
    const int32 OriginalDebug=DebugKinematics->GetInt();
    const ELogVerbosity::Type OriginalLogVerbosity=SpeedPhysicsLog.GetVerbosity();
    FActualPhysicalLogFacadeAttack LogAttack;LogAttack.Component=EscapedFacade;
    GLog->AddOutputDevice(&LogAttack);
    ON_SCOPE_EXIT{GLog->RemoveOutputDevice(&LogAttack);DebugKinematics->Set(OriginalDebug,ECVF_SetByCode);SpeedPhysicsLog.SetVerbosity(OriginalLogVerbosity);};
    SpeedPhysicsLog.SetVerbosity(ELogVerbosity::Log); // enable existing diagnostic, including all higher severities
    DebugKinematics->Set(1,ECVF_SetByCode);
    if(DebugKinematics->GetInt()!=1)return false;
    LogAttack.WithinOriginalStepCall=true;
    const bool IsolatedAdvanced=Isolated->Step(Second,false,Why);
    LogAttack.WithinOriginalStepCall=false;
    if(!TestTrue(TEXT("genuine next physical step executes its real debug output callback"),IsolatedAdvanced&&Control->Step(Second,false,Why)))return false;
    const auto ActualLogCut=Isolated->ReadLatest();
    AddInfo(FString::Printf(TEXT("[IASActualPhysicalLogContext] frame=%llu actualCanMove=%d debugCVar=%d categoryVerbosity=%d synchronousCalls=%d"),
        ActualLogCut?ActualLogCut->Frame:MAX_uint64,ActualLogCut&&!ActualLogCut->Actors.empty()?int(ActualLogCut->Actors[0].CanMove):-1,
        DebugKinematics->GetInt(),int(SpeedPhysicsLog.GetVerbosity()),LogAttack.Calls));
    TestEqual(TEXT("actual synchronous output callback reached inside original physical Step"),LogAttack.Calls,1);
    if(!LogAttack.RetainedView)return false;

    const auto Actual=Isolated->ReadLatest(),Reference=Control->ReadLatest();
    if(!Actual||!Reference||Actual->Frame!=1||Reference->Frame!=1)return false;
    const auto* Body=Actual->Physics.PresentationBodies.FindByPredicate([&](const auto& V){return V.StableId==Actual->Actors[0].NativeStableId;});
    const auto* Ref=Reference->Physics.PresentationBodies.FindByPredicate([&](const auto& V){return V.StableId==Reference->Actors[0].NativeStableId;});
    if(!Body||!Ref)return false;
    const auto PhysicalDifference=P::MeasurePrivateRoadKinematics(Body->COMState,Ref->COMState);
    if(!TestTrue(TEXT("actual forward native kinematics stay exact versus unattacked control"),PhysicalDifference.Valid&&!PhysicalDifference.Different))return false;
    if(!TestTrue(TEXT("declared native cache/wheel subset also exact after the real callback attack"),
        P::ComparePrivateRoadPartialValues(Actual->Actors[0],Reference->Actors[0])==P::EPrivateRoadPartialComparison::Equal))return false;
    TestEqual(TEXT("reference borrowed inside actual callback aliases facade only"),LogAttack.RetainedView->Location.X,LogAttack.WrittenX);
    TestEqual(TEXT("previous facade view observes only the callback facade write"),RetainedFacadeView.Location.X,LogAttack.WrittenX);
    Next.Frame=2;Next.Sequence=3;Next.Input.SourceSequence=3;
    if(History.Confirm(5,Next).Status!=EResult::Accepted)return false;
    const std::vector<P::FPrivateRoadRequest> Third{{101,3,History.Predict(2)}};
    if(!Isolated->Step(Third,false,Why)||!Control->Step(Third,false,Why))return false;
    const auto Later=Isolated->ReadLatest(),LaterRef=Control->ReadLatest();
    const auto* LaterBody=Later?Later->Physics.PresentationBodies.FindByPredicate([&](const auto& V){return V.StableId==Later->Actors[0].NativeStableId;}):nullptr;
    const auto* LaterControl=LaterRef?LaterRef->Physics.PresentationBodies.FindByPredicate([&](const auto& V){return V.StableId==LaterRef->Actors[0].NativeStableId;}):nullptr;
    if(!LaterBody||!LaterControl)return false;
    const auto LaterDifference=P::MeasurePrivateRoadKinematics(LaterBody->COMState,LaterControl->COMState);
    if(!TestTrue(TEXT("next real physical frame remains exact with callback view retained"),LaterDifference.Valid&&!LaterDifference.Different))return false;
    TestEqual(TEXT("retained callback view still never aliases native advance"),LogAttack.RetainedView->Location.X,LogAttack.WrittenX);
    DebugKinematics->Set(OriginalDebug,ECVF_SetByCode);

    TestTrue(TEXT("both actual original graphs retire on their owning GT"),Isolated->Close()&&Control->Close());
    AddInfo(TEXT("[IASPrivateComponentStorageIsolation] actual_native_forward_steps=2 actual_output_device_callback=1 retained_callback_view=1 facade_pose_and_velocity_attack=1 exact_six_com=1 component_value_blocks=5 common_value_blocks=5 complete_lease=0 localized=0"));
    // Never dereference OriginalWheelPointers after Close: storage has retired.
    // The unknown future borrowers remain unclosed; complete_lease stays0.
    AddInfo(FString::Printf(TEXT("[IASPrivateRegistryBoundary] actual_direct_legacy_restore_refused=1 actual_external_phase_observer_refused=1 actual_pending_unregister_refused=1 actual_registered_drift_refused=1 unchanged_publication=1 actual_global_birth_callback=1 actual_cleanup_callbacks=%d actual_cleanup_native_read=%d actual_cleanup_original_wheels=%d global_engine_enumeration_still_open=1 complete_lease=0"),ActualCleanupCallbacks,int(ActualCleanupRead),ActualCleanupBoundWheels));
    return !HasAnyErrors();
}
#endif
