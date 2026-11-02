#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadPhysicsWorld.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadNativeObjects.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadKinematicDiagnostics.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadPartialStateComparison.h"
#include "IAmSpeed/Driving/DrivingInput.h"
#include "Misc/AutomationTest.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "UObject/ObjectKey.h"
#include "Components/PrimitiveComponent.h"
#include "Components/BoxComponent.h"
#include "IAmSpeed/SubBodies/SSubBody.h"
#include "IAmSpeed/World/Simulation/RoadSettings/NativeRoadSettingsRead.h"
#include "IAmSpeed/World/Simulation/RoadSettings/DeclaredRoadConformanceProfile.h"
#include "UObject/ObjectHandleTracking.h"
#include "UObject/UObjectHash.h"
#include "Misc/ScopeExit.h"
#include <array>
#if PLATFORM_WINDOWS
#include "Windows/WindowsHWrapper.h"
#endif

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASRetainedVehicleInteractionNative,
    "IAmSpeed.Netcode.Prediction.RetainedVehicleInteractionNativeExperiment",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASRetainedVehicleInteractionNative::RunTest(const FString&)
{
    namespace P=Speed::Netcode::Prediction;using namespace Speed::Netcode;
    constexpr U64 Cut=10,Change=20,Last=900; // 3s observation, never startup warmup
    auto Contract=Speed::Driving::MakeContract(Speed::Driving::FBindings{});if(!Contract)return false;
    const auto DeclaredSettings=Speed::RoadSettings::MakeDeclaredRoadConformanceProfile();
    if(!DeclaredSettings)return false;
    std::vector<P::FPrivateRoadCarSeed> Seeds;
    for(U64 A=0;A<3;++A)
    {
        P::FPrivateRoadCarSeed S;S.DeclaredSettings=DeclaredSettings;S.Actor=101+A;S.Incarnation=3;S.Contract=Contract;
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
    auto RH=Histories(),LH=Histories(),KH=Histories(),UH=Histories();
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
    std::unique_ptr<P::FPrivateRoadPhysicsWorld> Reference,Live,Retained,Uncoupled;
    TArray<TWeakObjectPtr<UWorld>> ActualWorlds;
    {
        // Save real birth identities outside all original recipe/math scopes.
        const auto CreationHandle=FWorldDelegates::OnPostWorldCreation.AddLambda(
            [&](UWorld* World){if(IsInGameThread())ActualWorlds.Add(World);});
        ON_SCOPE_EXIT{FWorldDelegates::OnPostWorldCreation.Remove(CreationHandle);};
        Reference=P::FPrivateRoadPhysicsWorld::Create(Seeds,Why);
        Live=P::FPrivateRoadPhysicsWorld::Create(Seeds,Why);
        Retained=P::FPrivateRoadPhysicsWorld::Create(Seeds,Why);
        // Only the neighbor's proximity differs in this real native control.
        auto UncoupledSeeds=Seeds;UncoupledSeeds[1].Initial.Location.Y=10000;
        Uncoupled=P::FPrivateRoadPhysicsWorld::Create(UncoupledSeeds,Why);
    }
    if(!TestTrue(*Why,Reference&&Live&&Retained&&Uncoupled))return false;
    struct FReadTarget {TWeakObjectPtr<UPrimitiveComponent> Original;bool Floor=false;uint64 Scoped=0,Outside=0;};
    std::vector<FReadTarget> ReadTargets;
    uint64 AllScopedObjectReads=0,ScopedOtherObjectReads=0,ScopedNullObjectReads=0;
    if(!TestEqual(TEXT("four actual native fixture worlds created"),ActualWorlds.Num(),4))return false;
    for(const auto& WeakWorld:ActualWorlds)
    {
        auto* World=WeakWorld.Get();if(!World)return false;
        ForEachObjectWithOuter(World,[&](UObject* Object)
        {
            if(auto* Body=Cast<USSubBody>(Object))ReadTargets.push_back({Body,false});
            else if(auto* Floor=Cast<UBoxComponent>(Object))ReadTargets.push_back({Floor,true});
        },EGetObjectsFlags::IncludeNestedObjects);
    }
    if(!TestEqual(TEXT("sixty real subbodies and four actual floor components captured before observer"),ReadTargets.size(),size_t(64)))return false;
#if UE_WITH_OBJECT_HANDLE_TRACKING && PLATFORM_WINDOWS
    // Resolve/pin the actual system capture function BEFORE registering the
    // observer. No UE stack walker, symbols, names, logging or heap allocation
    // is called by the sampler below. These are diagnostic PCs, not a lease.
    using FRawStackCapture=uint16 (WINAPI*)(uint32,uint32,void**,uint32*);
    const auto NativeLibrary=::GetModuleHandleW(L"ntdll.dll");
    const auto RawCapture=NativeLibrary?reinterpret_cast<FRawStackCapture>(
        ::GetProcAddress(NativeLibrary,"RtlCaptureStackBackTrace")):nullptr;
    HMODULE PinnedCaptureModule=nullptr;
    const bool CapturePinned=RawCapture&&::GetModuleHandleExW(
        GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_PIN,
        reinterpret_cast<const wchar_t*>(RawCapture),&PinnedCaptureModule);
    if(!TestTrue(TEXT("actual system PC capture function resolved and pinned before observer"),CapturePinned))return false;
    struct FReadStack
    {
        bool Floor=false;uint16 Depth=0;uint64 Events=0;
        std::array<uint64,24> PCs{};
    };
    std::array<FReadStack,64> ReadStacks{};
    uint32 UsedStacks=0,TraceDepth=0,MaxTraceDepth=0;
    uint64 CaptureFailures=0,OverflowStackEvents=0,ReentrantSamplesSkipped=0;
#endif
#if UE_WITH_OBJECT_HANDLE_TRACKING
    const auto ReadHandle=UE::CoreUObject::AddObjectHandleReadCallback(
        [&](TArrayView<const UObject* const> Objects)
        {
            if(!IsInGameThread())return;
            if(Speed::RoadSettings::SelectedOriginalRoadSettings())for(const auto* Object:Objects)
            {
                ++AllScopedObjectReads;
                if(!Object){++ScopedNullObjectReads;continue;}
                bool TargetFound=false;
                for(const auto& Target:ReadTargets)if(Object==Target.Original.Get()){TargetFound=true;break;}
                if(!TargetFound)++ScopedOtherObjectReads;
            }
            for(const auto* Object:Objects)for(auto& Target:ReadTargets)
                if(Object&&Object==Target.Original.Get())
                {
                    if(Speed::RoadSettings::SelectedOriginalRoadSettings())
                    {
                        ++Target.Scoped; // ALL actual target events remain counted.
#if PLATFORM_WINDOWS
                        ++TraceDepth;MaxTraceDepth=FMath::Max(MaxTraceDepth,TraceDepth);
                        ON_SCOPE_EXIT{--TraceDepth;};
                        // A nested observer event is counted above, never
                        // suppressed. Avoid recursively instrumenting a stack
                        // sampler; its skipped sample remains explicitly open.
                        if(TraceDepth>1){++ReentrantSamplesSkipped;continue;}
                        std::array<void*,24> Captured{};
                        const uint16 Depth=RawCapture(0,uint32(Captured.size()),Captured.data(),nullptr);
                        if(!Depth){++CaptureFailures;continue;}
                        uint32 Index=0;
                        for(;Index<UsedStacks;++Index)
                        {
                            const auto& Stack=ReadStacks[Index];
                            if(Stack.Floor!=Target.Floor||Stack.Depth!=Depth)continue;
                            bool Same=true;
                            for(uint16 Frame=0;Frame<Depth;++Frame)
                                if(Stack.PCs[Frame]!=reinterpret_cast<uint64>(Captured[Frame])){Same=false;break;}
                            if(Same)break;
                        }
                        if(Index==UsedStacks)
                        {
                            if(UsedStacks==ReadStacks.size()){++OverflowStackEvents;continue;}
                            auto& Stack=ReadStacks[UsedStacks++];Stack.Floor=Target.Floor;Stack.Depth=Depth;
                            for(uint16 Frame=0;Frame<Depth;++Frame)Stack.PCs[Frame]=reinterpret_cast<uint64>(Captured[Frame]);
                        }
                        ++ReadStacks[Index].Events;
#endif
                    }
                    else ++Target.Outside;
                }
        });
    ON_SCOPE_EXIT{UE::CoreUObject::RemoveObjectHandleReadCallback(ReadHandle);};
#endif
    const auto Same=[&](const auto& X,const auto& Y)
    {
        if(!X||!Y||X->Frame!=Y->Frame||X->Actors.size()!=3||Y->Actors.size()!=3)return false;
        for(size_t A=0;A<3;++A)
        {
            const auto& L=X->Actors[A];const auto& R=Y->Actors[A];
            if(P::ComparePrivateRoadPartialValues(L,R)!=P::EPrivateRoadPartialComparison::Equal
                ||L.Actor!=R.Actor||L.Incarnation!=R.Incarnation||!(L.Filter==R.Filter)||!L.Input||!R.Input
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
    uint64 DynamicContactFrames=0,FirstContact=0,BoxContactFrames=0,WheelRegistrationFrames=0,BoxWheelFrames=0;
    double MaximumControlDifference=0;
    double NearestCOM=MAX_dbl,MaximumTravel=0;
    FVector StartCOM=FVector::ZeroVector;
    std::shared_ptr<const P::FPrivateRoadPublication> OriginalCut;
    std::vector<std::vector<P::FPrivateRoadContactDispatchPair>> ActualReferenceContactDispatches;
    for(U64 N=0;N<=Last;++N)
    {
        for(size_t A=0;A<3;++A)
        {
            const auto Q=Packet(A,N);
            if(RH[A]->Confirm(9,Q).Status!=EResult::Accepted||KH[A]->Confirm(9,Q).Status!=EResult::Accepted||UH[A]->Confirm(9,Q).Status!=EResult::Accepted)return false;
            if((A!=0||N<Change)&&LH[A]->Confirm(9,Q).Status!=EResult::Accepted)return false;
        }
        if(!Reference->Step(Requests(RH,N),false,Why)||!Live->Step(Requests(LH,N),false,Why)||!Uncoupled->Step(Requests(UH,N),false,Why))return false;
        if(N<=Cut&&!Retained->Step(Requests(KH,N),false,Why))return false;
        if(N==0)
        {
            const auto R=Reference->ReadLatest(),L=Live->ReadLatest(),K=Retained->ReadLatest(),U=Uncoupled->ReadLatest();
            if(!TestTrue(TEXT("actual synchronized frozen native baseline"),Same(R,L)&&Same(R,K)))return false;
            if(!Reference->ReleaseInitialPublication(R,Why)||!Live->ReleaseInitialPublication(L,Why)||!Retained->ReleaseInitialPublication(K,Why)||!Uncoupled->ReleaseInitialPublication(U,Why))return false;
        }
        if(N==Cut){OriginalCut=Retained->ReadLatest();if(!Same(Reference->ReadLatest(),OriginalCut))return false;}
        const auto Actual=Reference->ReadLatest();
        if(!TestTrue(TEXT("actual native contact dispatch observation remains complete"),Actual&&Actual->ContactDispatchObservationComplete))return false;
        ActualReferenceContactDispatches.push_back(Actual->ContactDispatchPairs);
        const auto* BA=Actual->Physics.PresentationBodies.FindByPredicate([&](const auto& V){return V.StableId==Actual->Actors[0].NativeStableId;});
        const auto* BB=Actual->Physics.PresentationBodies.FindByPredicate([&](const auto& V){return V.StableId==Actual->Actors[1].NativeStableId;});
        const auto U=Uncoupled->ReadLatest();
        const auto* UB=U->Physics.PresentationBodies.FindByPredicate([&](const auto& V){return V.StableId==U->Actors[0].NativeStableId;});
        if(!BA||!BB||!UB)return false;
        const auto Control=P::MeasurePrivateRoadKinematics(BA->COMState,UB->COMState);
        if(!Control.Valid)return false;
        if(N==0&&!TestTrue(TEXT("controlled actor has exact synchronized baseline before neighboring motion"),!Control.Different))return false;
        MaximumControlDifference=FMath::Max(MaximumControlDifference,Control.Position);
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
        bool Contact=false,Box=false,BoxWheel=false,Wheel=false;
        for(const auto& Pair:Actual->ContactDispatchPairs)
            if(Pair.ActorA==101&&Pair.ActorB==102)
            {Box=Pair.Dispatches>0;BoxWheel=Pair.BoxWheelDispatches>0;Wheel=Pair.WheelRegistrations>0;Contact=Box||BoxWheel||Wheel;}
        if(Box)++BoxContactFrames;if(BoxWheel)++BoxWheelFrames;if(Wheel)++WheelRegistrationFrames;
        if(Contact){if(!DynamicContactFrames)FirstContact=N;++DynamicContactFrames;}
    }
    AddInfo(FString::Printf(TEXT("[IASInteractingFixtureMotion] native=1 default_sync_gate=1 throttle_after_release=1 nearest_com_cm=%.17g max_actor0_travel_cm=%.17g actual_vehicle_event_frames=%llu"),NearestCOM,MaximumTravel,DynamicContactFrames));
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
    AddInfo(FString::Printf(TEXT("[IASInteractionCausalControl] frame=%llu actual_box_contact_dispatch_frames=%llu actual_box_wheel_dispatch_frames=%llu actual_wheel_registration_frames=%llu actual_vehicle_event_frames=%llu first_vehicle_event=%llu uncoupled_actor0_max_position_cm=%.17g box_only_failure35_preserved=1 complete_contact_coverage=0"),Last,BoxContactFrames,BoxWheelFrames,WheelRegistrationFrames,DynamicContactFrames,FirstContact,MaximumControlDifference));
    if(!TestTrue(TEXT("real different-car box/wheel event, not optional constraints or floor contact"),DynamicContactFrames>0))return false;
    if(!TestTrue(TEXT("same actor/inputs show physical coupling versus native distant-neighbor control"),MaximumControlDifference>1))return false;
    if(!TestTrue(TEXT("withheld oscillating input caused measurable native divergence"),MaximumBefore>0))return false;
    if(!TestTrue(TEXT("actual retained graph stayed at original K without stepping"),Retained->ReadLatest()==OriginalCut))return false;
    const double ReplayStarted=FPlatformTime::Seconds();
    for(U64 N=Cut+1;N<=Last;++N)
    {
        if(!Retained->Step(Requests(KH,N),true,Why))return false;
        const auto Actual=Retained->ReadLatest();
        if(!TestTrue(TEXT("actual corrected native blocking contact dispatch sequence matches independent reference"),Actual&&Actual->ContactDispatchObservationComplete&&Actual->ContactDispatchPairs==ActualReferenceContactDispatches[size_t(N)]))return false;
    }
    if(!TestTrue(TEXT("same original native lifetime, three actors exact after K+1..N"),Retained->ReadLatest()->LocalKernelLifetime==OriginalCut->LocalKernelLifetime&&Same(Truth,Retained->ReadLatest())))return false;
    const double ReplayWallSeconds=FPlatformTime::Seconds()-ReplayStarted;
    if(!TestTrue(TEXT("actual native replay wall time is finite and positive"),FMath::IsFinite(ReplayWallSeconds)&&ReplayWallSeconds>0))return false;
    AddInfo(FString::Printf(TEXT("[IASNativeFastReplayTiming] frames=%llu physical_seconds=%.17g wall_seconds=%.17g nominal_to_wall_ratio=%.17g wait_policy_bypassed=1 same_world_localized=0"),Last-Cut,double(Last-Cut)/300.,ReplayWallSeconds,(double(Last-Cut)/300.)/ReplayWallSeconds));
    uint64 ScopedBodyReads=0,ScopedFloorReads=0,PositiveReads=0;
    for(auto& Target:ReadTargets)
    {
        auto* Original=Target.Original.Get();
        if(!TestTrue(TEXT("observed physical target retains real original identity"),Original!=nullptr))return false;
        const uint64 Before=Target.Outside;
        TObjectPtr<UPrimitiveComponent> Positive=Original;
        if(!TestTrue(TEXT("ordinary positive Get returns this actual target"),Positive.Get()==Original))return false;
#if UE_WITH_OBJECT_HANDLE_TRACKING
        if(!TestTrue(TEXT("ordinary positive Get invokes the hook for this specific target"),Target.Outside>Before))return false;
#endif
        PositiveReads+=Target.Outside-Before;
        if(Target.Floor)ScopedFloorReads+=Target.Scoped;else ScopedBodyReads+=Target.Scoped;
    }
    if(!TestEqual(TEXT("actual ALL scoped events partition exactly into target bodies, floors, other objects and nulls"),
        AllScopedObjectReads,ScopedBodyReads+ScopedFloorReads+ScopedOtherObjectReads+ScopedNullObjectReads))return false;
    AddInfo(FString::Printf(TEXT("[ActualInteractingAllObjectReads] trackingCompiled=%d allScopedObjectReads=%llu scopedOtherObjectReads=%llu scopedNullObjectReads=%llu completeCallbackClosure=0 completeRestoreLease=0"),
        int(UE_WITH_OBJECT_HANDLE_TRACKING),AllScopedObjectReads,ScopedOtherObjectReads,ScopedNullObjectReads));
    AddInfo(FString::Printf(TEXT("[ActualInteractingObjectReadWitness] trackingCompiled=%d targets=%u actualWorlds=4 carsPerWorld=3 observedVehicleEventFrames=%llu scopedBodyReads=%llu scopedFloorReads=%llu ordinaryPositiveReads=%llu birthObserved=0 unknownOtherObjects=1 completeCallbackClosure=0 completeRestoreLease=0"),
        int(UE_WITH_OBJECT_HANDLE_TRACKING),uint32(ReadTargets.size()),DynamicContactFrames,ScopedBodyReads,ScopedFloorReads,PositiveReads));
#if UE_WITH_OBJECT_HANDLE_TRACKING && PLATFORM_WINDOWS
    uint64 RecordedStackEvents=0,DepthCappedEvents=0;
    for(uint32 Index=0;Index<UsedStacks;++Index)
    {
        RecordedStackEvents+=ReadStacks[Index].Events;
        if(ReadStacks[Index].Depth==24)DepthCappedEvents+=ReadStacks[Index].Events;
    }
    AddInfo(FString::Printf(TEXT("[ActualInteractingReadStacks] unique=%u recordedEvents=%llu captureFailures=%llu overflowEvents=%llu reentrantSamplesSkipped=%llu maxObserverDepth=%u depthCappedEvents=%llu frameCap=24 stackCap=64 capturedPrefixOnly=1 addressesOnly=1 timingProfileInstrumented=1 completeCallbackClosure=0"),
        UsedStacks,RecordedStackEvents,CaptureFailures,OverflowStackEvents,ReentrantSamplesSkipped,MaxTraceDepth,DepthCappedEvents));
    if(!TestEqual(TEXT("every scoped target event is represented or explicitly incomplete"),
        RecordedStackEvents+CaptureFailures+OverflowStackEvents+ReentrantSamplesSkipped,ScopedBodyReads+ScopedFloorReads))return false;
    for(uint32 Index=0;Index<UsedStacks;++Index)
    {
        const auto& Stack=ReadStacks[Index];
        AddInfo(FString::Printf(TEXT("[ActualInteractingReadStack] ordinal=%u floor=%d events=%llu depth=%u suffixUnknown=%d"),Index,int(Stack.Floor),Stack.Events,uint32(Stack.Depth),int(Stack.Depth==24)));
        for(uint16 Frame=0;Frame<Stack.Depth;++Frame)
        {
            const uint64 PC=Stack.PCs[Frame];HMODULE Module=nullptr;
            wchar_t Path[4096]{};uint32 Length=0;uint64 Base=0;
            // Actual address-to-module mapping is performed after ALL math,
            // outside the observer. No PDB/network/symbol-server work here.
            const bool Mapped=::GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS,
                reinterpret_cast<const wchar_t*>(PC),&Module)!=0;
            if(Mapped)
            {
                Base=reinterpret_cast<uint64>(Module);
                Length=::GetModuleFileNameW(Module,Path,uint32(UE_ARRAY_COUNT(Path)));
                ::FreeLibrary(Module); // balance only this ordinary temporary reference
            }
            const bool PathComplete=Length>0&&Length<UE_ARRAY_COUNT(Path);
            AddInfo(FString::Printf(TEXT("[ActualInteractingReadStackFrame] stack=%u frame=%u pc=%016llX moduleBase=%016llX rva=%016llX mapped=%d pathComplete=%d module=%s"),
                Index,uint32(Frame),PC,Base,Mapped?PC-Base:0,int(Mapped),int(PathComplete),PathComplete?Path:TEXT("<unknown>")));
        }
    }
#endif
    // ALL THREE actors stepped during this experiment, including the distant
    // same-world spectator. This must NOT be reported as localized replay.
    AddInfo(FString::Printf(TEXT("[IASRetainedInteracting] native=1 actors=3 held_frame=%llu through_frame=%llu replayed_frames=%llu actual_vehicle_event_frames=%llu first_actual_vehicle_event=%llu before_max_position_cm=%.17g exact_six_com_and_filters=1 spectator_replay_frames=%llu same_world_localized=0 full_state_certificate=0 network=0"),Cut,Last,Last-Cut,DynamicContactFrames,FirstContact,MaximumBefore,Last-Cut));
    return !HasAnyErrors();
}
#endif
