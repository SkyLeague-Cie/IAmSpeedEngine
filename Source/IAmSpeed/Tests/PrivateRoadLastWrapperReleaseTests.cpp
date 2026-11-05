#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadPhysicsWorld.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadNativeObjects.h"
#include "IAmSpeed/Driving/DrivingInput.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "IAmSpeed/World/Simulation/NativeEpochDiagnostics.h"
#include "IAmSpeed/IAmSpeed.h"
#include "EngineUtils.h"
#include "Engine/World.h"
#include "HAL/IConsoleManager.h"
#include "Misc/AutomationTest.h"
#include "Misc/OutputDevice.h"
#include "Misc/OutputDeviceRedirector.h"
#include "Misc/ScopeExit.h"
#include "UObject/GarbageCollection.h"
#include "UObject/ObjectKey.h"
#include "WheelSystem.h"
#include <type_traits>

namespace
{
namespace P=Speed::Netcode::Prediction;
static_assert(!std::is_copy_constructible_v<P::FPrivateRoadPhysicsWorld>);
static_assert(!std::is_copy_assignable_v<P::FPrivateRoadPhysicsWorld>);
static_assert(!std::is_move_constructible_v<P::FPrivateRoadPhysicsWorld>);
static_assert(!std::is_move_assignable_v<P::FPrivateRoadPhysicsWorld>);

using FTraces=std::vector<std::shared_ptr<const P::FNativeConfigurationLifetimeTrace>>;
bool ActualNativeConfigurationIsLive(const FTraces& Traces)
{
    if(Traces.size()!=2)return false;
    for(const auto& Trace:Traces)
    {
        if(!Trace)return false;
        const auto V=Trace->Read();
        if(V.NativeConstructionReturns!=1||V.NativeBaseDestructionReturns!=0
            ||V.ConfigurationDestructionEntries!=0||V.ConfigurationDestructionBeforeNativeReturn)return false;
    }
    return true;
}
bool ActualNativeConfigurationWasRetiredOnce(const FTraces& Traces)
{
    if(Traces.size()!=2)return false;
    for(const auto& Trace:Traces)
    {
        if(!Trace)return false;
        const auto V=Trace->Read();
        if(V.NativeConstructionReturns!=1||V.NativeBaseDestructionReturns!=1
            ||V.ConfigurationDestructionEntries!=1||V.ConfigurationDestructionBeforeNativeReturn)return false;
    }
    return true;
}
struct FActualOwnerReleaseException final {};
struct FLastWrapperReleaseFromActualLog final : FOutputDevice
{
    std::unique_ptr<P::FPrivateRoadPhysicsWorld>* ActualOwner=nullptr;
    FTraces Traces;
    P::FPrivateRoadPhysicsWorld* OtherGraph=nullptr;
    const std::vector<P::FPrivateRoadRequest>* ActualRequests=nullptr;
    std::shared_ptr<const P::FPrivateRoadPublication> ActualBaseline;
    bool StepRefusedBeforeMutex=false,ActivationRefusedBeforeMutex=false;
    bool CounterReadRefused=false,CounterObserverRefused=false,OriginalWorldProbeRefused=false;
    bool OtherGraphAdvanced=false;uint64 OtherGraphActualFrame=MAX_uint64;
    std::array<const Chaos::FSimpleWheelSim*,4> ReferenceFacadeWheels{};
    bool Armed=false,ThrowAfterRelease=false,CloseWasRefused=false;
    bool OutsidePhysicalOutbox=false,NativeLiveAfterRelease=false,NativeLiveAfterGC=false;
    int32 Calls=0,ActualGCReturns=0,ActualReferenceWheelReads=0;
    bool CanBeUsedOnAnyThread() const override {return true;}
    bool CanBeUsedOnMultipleThreads() const override {return true;}
    void Serialize(const TCHAR* Text,ELogVerbosity::Type,const FName&) override
    {
        if(!IsInGameThread()||!Armed||Calls||!ActualOwner||!*ActualOwner
            ||!FCString::Strstr(Text,TEXT("[UpdateFrameState]")))return;
        ++Calls;
        OutsidePhysicalOutbox=!Speed::NativeEpoch::HasOriginalDiagnosticOutbox();
        // Real owner call while the outer Step holds its gate, not a flag
        // claiming a drain. Close must refuse and retain the actual graph.
        CloseWasRefused=!(*ActualOwner)->Close();
        FString ReentrantWhy;
        if(ActualRequests)
            StepRefusedBeforeMutex=!(*ActualOwner)->Step(*ActualRequests,false,ReentrantWhy)
                &&ReentrantWhy.Contains(TEXT("Step refused before mutex"));
        ReentrantWhy.Empty();
        ActivationRefusedBeforeMutex=!(*ActualOwner)->ReleaseInitialPublication(ActualBaseline,ReentrantWhy)
            &&ReentrantWhy.Contains(TEXT("activation refused before mutex"));
        CounterReadRefused=(*ActualOwner)->ReadNativeConfigurationLifetimeForDiagnostics().empty();
        CounterObserverRefused=(*ActualOwner)->CaptureNativeConfigurationLifetimeObserversForDiagnostics().empty();
        OriginalWorldProbeRefused=!(*ActualOwner)->ObserveOriginalWorldReadForDiagnostics().Admitted;
        if(OtherGraph&&ActualRequests)
        {
            FString OtherWhy;
            OtherGraphAdvanced=OtherGraph->Step(*ActualRequests,false,OtherWhy);
            const auto ActualOther=OtherGraph->ReadLatest();
            if(ActualOther)OtherGraphActualFrame=ActualOther->Frame;
        }
        ActualOwner->reset(); // genuine destruction of the LAST wrapper owner
        NativeLiveAfterRelease=ActualNativeConfigurationIsLive(Traces);
        CollectGarbage(RF_NoFlags);++ActualGCReturns;
        NativeLiveAfterGC=ActualNativeConfigurationIsLive(Traces);
        // These public pointers were obtained OUTSIDE the native route and
        // therefore belong to the real reference/facade simulation. They do
        // not prove post-GC reads of the original physical wheels.
        for(const auto* Wheel:ReferenceFacadeWheels)
            if(Wheel&&FMath::IsFinite(Wheel->GetAngularVelocity()))++ActualReferenceWheelReads;
        if(ThrowAfterRelease)throw FActualOwnerReleaseException{};
    }
};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASPrivateRoadLastWrapperRelease,
    "IAmSpeed.Netcode.Prediction.PrivateRoadLastWrapperRelease",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASPrivateRoadLastWrapperRelease::RunTest(const FString&)
{
    using namespace Speed::Netcode;
    auto* Debug=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.DebugKinematics"));
    if(!TestTrue(TEXT("actual diagnostic output channel available"),Debug&&GLog))return false;
    const int32 OriginalDebug=Debug->GetInt();
    const auto OriginalVerbosity=SpeedPhysicsLog.GetVerbosity();
    ON_SCOPE_EXIT{Debug->Set(OriginalDebug,ECVF_SetByCode);SpeedPhysicsLog.SetVerbosity(OriginalVerbosity);};
    Debug->Set(1,ECVF_SetByCode);SpeedPhysicsLog.SetVerbosity(ELogVerbosity::Log);
    if(!TestEqual(TEXT("real physical diagnostic CVar enabled"),Debug->GetInt(),1))return false;
    for(const bool ThrowFromActualLog:{false,true})
    {
        P::FPrivateRoadCarSeed Seed;Seed.Actor=101;Seed.Incarnation=3;
        Seed.Contract=Speed::Driving::MakeContract(Speed::Driving::FBindings{});
        Seed.Local={Speed::Input::EProducerKind::Device,42};Seed.LocalEpoch={3};
        auto& B=Seed.Remote;B.Session=11;B.Epoch=3;B.Actor=101;B.Incarnation=3;
        B.OwnerPeer=4;B.ServerPeer=5;B.SourceProducer=42;B.SourceKind=0;B.Contract[0]=7;
        Seed.Initial.Location=FVector(0,0,45);
        TSet<FObjectKey> Before;
        for(TObjectIterator<UWorld> It;It;++It)if(!It->IsTemplate())Before.Add(FObjectKey(*It));
        FString Why;auto Owner=P::FPrivateRoadPhysicsWorld::Create({Seed},Why);
        if(!TestTrue(*FString::Printf(TEXT("actual owner created: %s"),*Why),bool(Owner)))return false;
        UWorld* TargetWorld=nullptr;USpeedBranchWheeledComponent* Component=nullptr;
        for(TObjectIterator<UWorld> It;It;++It)if(!It->IsTemplate()&&!Before.Contains(FObjectKey(*It)))
        {
            if(!TestTrue(TEXT("exactly one actual new world"),!TargetWorld))return false;
            TargetWorld=*It;
        }
        if(!TestTrue(TEXT("actual new world observed via Engine enumeration"),TargetWorld!=nullptr))return false;
        for(TActorIterator<ASpeedBranchCar> It(TargetWorld);It;++It)
            Component=Cast<USpeedBranchWheeledComponent>(It->GetVehicleMovement());
        if(!TestTrue(TEXT("actual native component enumerated"),Component!=nullptr))return false;
        const auto Traces=Owner->CaptureNativeConfigurationLifetimeObserversForDiagnostics();
        if(!TestTrue(TEXT("original and reference actual native constructors returned"),ActualNativeConfigurationIsLive(Traces)))return false;
        const auto RetirementObserver=Owner->CaptureRetirementBindingObserverForDiagnostics();
        const auto& ActualWheelsForCleanup=Component->GetWheelSubBodies();
        if(!TestEqual(TEXT("four actual wheel subjects before cleanup"),ActualWheelsForCleanup.Num(),4))return false;
        TWeakObjectPtr<USWheelSubBody> WheelToInvalidate(ActualWheelsForCleanup[0].Get());
        int32 CleanupCalls=0;bool NativeLiveAtCleanup=false;int32 ActualInvalidationCalls=0;
        const auto Cleanup=FWorldDelegates::OnWorldCleanup.AddLambda([&](UWorld* World,bool,bool)
        {
            if(!IsInGameThread()||World!=TargetWorld)return;
            ++CleanupCalls;NativeLiveAtCleanup=ActualNativeConfigurationIsLive(Traces);
            if(auto* Wheel=WheelToInvalidate.Get()) {
                // Real target/callback, never a forced diagnostic count. These
                // APIs may resolve into different actual skip populations.
                if(ThrowFromActualLog)Wheel->ConditionalBeginDestroy();else Wheel->MarkAsGarbage();
                ++ActualInvalidationCalls;
            }
        });
        ON_SCOPE_EXIT{FWorldDelegates::OnWorldCleanup.Remove(Cleanup);};
        std::array<Consumed::FActionPolicy,ActionCount> Actions{};
        Actions[0]={true,true,false,255};Actions[1]={true,true,false,255};Actions[2]={true,true,true,127};
        P::FRemoteRequestHistory History(B,Actions,256,256);
        FPacket Packet;Packet.Kind=EKind::AuthorityInput;Packet.Session=B.Session;Packet.Epoch=B.Epoch;
        Packet.Actor=B.Actor;Packet.Incarnation=B.Incarnation;Packet.Contract=B.Contract;Packet.Sequence=1;
        Packet.Input.SourceSequence=1;Packet.Input.DeviceGeneration=1;Packet.Input.ProducerId=42;
        Packet.Input.ProducerKind=0;Packet.Input.Reset=true;
        if(History.Confirm(5,Packet).Status!=EResult::Accepted)return false;
        const std::vector<P::FPrivateRoadRequest> FirstRequests{{101,3,History.Predict(0)}};
        if(!Owner->Step(FirstRequests,false,Why))return false;
        const auto Baseline=Owner->ReadLatest();
        if(!Baseline||!Owner->ReleaseInitialPublication(Baseline,Why))return false;
        // A second ACTUALLY born independent native graph is advanced from
        // the first graph's real GLog callback. This is a nested-owner control,
        // never a claim of two playable/network clients or complete source state.
        auto OtherOwner=P::FPrivateRoadPhysicsWorld::Create({Seed},Why);
        if(!TestTrue(TEXT("second actual native graph created"),bool(OtherOwner)))return false;
        if(!OtherOwner->Step(FirstRequests,false,Why))return false;
        const auto OtherBaseline=OtherOwner->ReadLatest();
        if(!OtherBaseline||!OtherOwner->ReleaseInitialPublication(OtherBaseline,Why))return false;
        Packet.Frame=1;Packet.Sequence=2;Packet.Input.SourceSequence=2;Packet.Input.Reset=false;
        Packet.Input.Values[0]=255;Packet.Input.ActiveMask=1;Packet.Input.Edges.push_back({0,0,255,2,0});
        if(History.Confirm(5,Packet).Status!=EResult::Accepted)return false;
        const std::vector<P::FPrivateRoadRequest> Requests{{101,3,History.Predict(1)}};
        FLastWrapperReleaseFromActualLog Device;Device.ActualOwner=&Owner;Device.Traces=Traces;
        Device.ThrowAfterRelease=ThrowFromActualLog;
        Device.OtherGraph=OtherOwner.get();Device.ActualRequests=&Requests;Device.ActualBaseline=Baseline;
        const auto& Wheels=Component->GetWheelSubBodies();
        if(!TestEqual(TEXT("actual four facade wheel UObjects available"),Wheels.Num(),4))return false;
        for(int32 I=0;I<4;++I)
        {
            Device.ReferenceFacadeWheels[I]=Wheels[I]?Wheels[I]->GetWheelSim():nullptr;
            if(!TestTrue(TEXT("genuine reference/facade WheelSim backing obtained through public entry"),Device.ReferenceFacadeWheels[I]!=nullptr))return false;
        }
        GLog->AddOutputDevice(&Device);
        ON_SCOPE_EXIT{GLog->RemoveOutputDevice(&Device);};
        // Raw pointer used ONCE to start the public operation. Never touch it
        // after a real callback may have deleted the public wrapper.
        auto* Entry=Owner.get();Device.Armed=true;
        const bool Returned=Entry->Step(Requests,false,Why);
        Device.Armed=false;
        // A skipped garbage/BeginDestroyed UObject can legitimately retain its
        // original C++ aliases/configuration until real GC. Observe that GC
        // return separately; never invent destruction during the earlier Close.
        CollectGarbage(RF_NoFlags);
        AddInfo(TEXT("[PrivateRoadRetirementPostCloseGC] actualReturns=1"));
        TestTrue(TEXT("Step returned the actual expected result after callback"),Returned!=ThrowFromActualLog);
        TestEqual(TEXT("actual supported GLog callback executed once"),Device.Calls,1);
        TestTrue(TEXT("real callback deleted last wrapper"),!Owner);
        TestTrue(TEXT("reentrant actual Close refused while Step remained active"),Device.CloseWasRefused);
        TestTrue(TEXT("same-graph reentrant Step refused BEFORE nonrecursive mutex acquisition"),Device.StepRefusedBeforeMutex);
        TestTrue(TEXT("same-graph reentrant activation refused BEFORE nonrecursive mutex acquisition"),Device.ActivationRefusedBeforeMutex);
        TestTrue(TEXT("same-graph counter query refused while its actual operation remains active"),Device.CounterReadRefused);
        TestTrue(TEXT("same-graph observer query refused while its actual operation remains active"),Device.CounterObserverRefused);
        TestTrue(TEXT("same-graph original-world probe refused before mutex acquisition"),Device.OriginalWorldProbeRefused);
        TestTrue(TEXT("different real graph advanced during first graph's actual log callback"),Device.OtherGraphAdvanced);
        TestEqual(TEXT("second graph actually published requested frame1"),Device.OtherGraphActualFrame,uint64(1));
        TestTrue(TEXT("real output callback was outside physical outbox/GC scope"),Device.OutsidePhysicalOutbox);
        TestTrue(TEXT("actual original/reference simulations remained live after wrapper deletion"),Device.NativeLiveAfterRelease);
        TestEqual(TEXT("actual GC returned after wrapper deletion"),Device.ActualGCReturns,1);
        TestTrue(TEXT("actual native configurations survived that real GC"),Device.NativeLiveAfterGC);
        TestEqual(TEXT("all four actual reference/facade WheelSim reads succeeded AFTER wrapper deletion and GC"),Device.ActualReferenceWheelReads,4);
        TestEqual(TEXT("real native world cleanup callback executed exactly once"),CleanupCalls,1);
        TestTrue(TEXT("actual configurations still live during original world cleanup callback"),NativeLiveAtCleanup);
        TestEqual(TEXT("one actual cleanup callback invalidation returned"),ActualInvalidationCalls,1);
        if(!TestTrue(TEXT("retained counter values survive last wrapper without retaining graph"),bool(RetirementObserver)))return false;
        const auto& RT=*RetirementObserver;
        TestTrue(TEXT("real deferred Close and world callback returned"),RT.CloseReturns==1&&RT.WorldDestroyReturns==1);
        TestTrue(TEXT("actual skipped subject is measured; branch identity is not invented"),RT.WeakUnresolved+RT.GarbageFlagSkips>0);
        TestTrue(TEXT("remaining real live subjects actually cleared"),RT.LiveResolved>0&&RT.FacadeClearReturns==RT.LiveResolved);
        TestTrue(TEXT("last-wrapper positive has actual facade pointer presence before clear"),RT.AnyFacadeBindingPresentBefore>0);
        TestTrue(TEXT("last-wrapper actual three facade cells null after returned clears"),RT.FacadeWheelNullAfter==RT.FacadeClearReturns&&RT.FacadeSuspensionNullAfter==RT.FacadeClearReturns&&RT.FacadeConfigurationNullAfter==RT.FacadeClearReturns);
        TestTrue(TEXT("last-wrapper observed field transitions remain distinct"),RT.FacadeWheelChangedFromPresent==RT.FacadeWheelPresentBefore&&RT.FacadeSuspensionChangedFromPresent==RT.FacadeSuspensionPresentBefore&&RT.FacadeConfigurationChangedFromPresent==RT.FacadeConfigurationPresentBefore);
        AddInfo(FString::Printf(TEXT("[PrivateRoadRetirementLastWrapperEffects] throw=%d anyBefore=%u wheelBefore=%u suspensionBefore=%u configurationBefore=%u wheelNullAfter=%u suspensionNullAfter=%u configurationNullAfter=%u wheelChanged=%u suspensionChanged=%u configurationChanged=%u"),int(ThrowFromActualLog),RT.AnyFacadeBindingPresentBefore,RT.FacadeWheelPresentBefore,RT.FacadeSuspensionPresentBefore,RT.FacadeConfigurationPresentBefore,RT.FacadeWheelNullAfter,RT.FacadeSuspensionNullAfter,RT.FacadeConfigurationNullAfter,RT.FacadeWheelChangedFromPresent,RT.FacadeSuspensionChangedFromPresent,RT.FacadeConfigurationChangedFromPresent));
        TestEqual(TEXT("actual last-wrapper weak population partitions"),RT.WeakVisited,RT.WeakUnresolved+RT.GarbageFlagSkips+RT.LiveResolved);
        TestTrue(TEXT("clearing facade preserved actual original pointer triples"),RT.OriginalPointerTriplesCompared==4&&RT.FacadeClearsKeptOriginalPointers);
        TestEqual(TEXT("separate original retained wheel disconnection returns"),RT.OriginalWheelDisconnectReturns,uint32(4));
        AddInfo(FString::Printf(TEXT("[PrivateRoadRetirementLastWrapper] throw=%d visited=%u unresolved=%u flagged=%u live=%u cleared=%u originalCompared=%u originalKept=%d disconnected=%u worldReturned=%u closeReturned=%u invalidationCalls=%d"),int(ThrowFromActualLog),RT.WeakVisited,RT.WeakUnresolved,RT.GarbageFlagSkips,RT.LiveResolved,RT.FacadeClearReturns,RT.OriginalPointerTriplesCompared,int(RT.FacadeClearsKeptOriginalPointers),RT.OriginalWheelDisconnectReturns,RT.WorldDestroyReturns,RT.CloseReturns,ActualInvalidationCalls));
        TestTrue(TEXT("both actual native destructors returned before actual config destructors, exactly once"),ActualNativeConfigurationWasRetiredOnce(Traces));
        TestTrue(TEXT("retained immutable startup value survives independently"),Baseline->Frame==0&&!Baseline->Actors.empty()&&!Baseline->Actors[0].CanMove);
        if(ThrowFromActualLog)TestTrue(TEXT("real callback exception was retained as a failed physical operation"),Why.Contains(TEXT("diagnostic callback exception")));
        if(OtherOwner)TestTrue(TEXT("second actual owner closes after the original callback returned"),OtherOwner->Close());
        AddInfo(FString::Printf(TEXT("[IASActualGraphReentry] same_step_refused_before_mutex=%d same_activation_refused_before_mutex=%d counter_refused=%d observer_refused=%d original_world_refused=%d other_graph_advanced=%d other_actual_frame=%llu"),
            int(Device.StepRefusedBeforeMutex),int(Device.ActivationRefusedBeforeMutex),int(Device.CounterReadRefused),
            int(Device.CounterObserverRefused),int(Device.OriginalWorldProbeRefused),int(Device.OtherGraphAdvanced),Device.OtherGraphActualFrame));
        AddInfo(FString::Printf(TEXT("[IASLastWrapperRelease] throw=%d actual_log_calls=%d actual_gc_returns=%d actual_reference_wheel_reads=%d actual_cleanup_calls=%d native_dtor_order=%d operation_result=%d"),
            int(ThrowFromActualLog),Device.Calls,Device.ActualGCReturns,Device.ActualReferenceWheelReads,CleanupCalls,
            int(ActualNativeConfigurationWasRetiredOnce(Traces)),int(Returned)));
    }
    return true;
}
#endif
