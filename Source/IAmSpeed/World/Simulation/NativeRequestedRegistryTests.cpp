#include "Misc/Build.h"
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "IAmSpeed/World/Simulation/SpeedPhysicsSimulation.h"
#include "IAmSpeed/World/Simulation/RealTimeSimulation.h"
#include "IAmSpeed/World/Simulation/CanonicalFrameContext.h"
#include "IAmSpeed/World/Simulation/NativeExactInputFixture1304.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "IAmSpeed/Actors/SpeedCar.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/Netcode/transport/NetworkRequestedInputSource.h"
#include "ChaosVehicleWheel.h"
#include "SimpleVehicle.h"
#include "SuspensionSystem.h"
#include "WheelSystem.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTLS.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include <atomic>
#include <type_traits>
static_assert(!std::is_default_constructible_v<Speed::FNativeRequestedInputObservation>);
static_assert(!std::is_copy_constructible_v<Speed::FNativeRequestedInputObservation>);

namespace
{
struct FRequestedPresentationStats
{
    std::atomic<uint32> GetterThread{0},ProduceThread{0},InvalidateThread{0},DestroyedThread{0},Calls{0},Invalidates{0};
};
class FRequestedPresentationProbe final:public ISimulationPresentationProducer
{
public:
    FRequestedPresentationProbe(uint64 Id,uint32 Channel,std::shared_ptr<FRequestedPresentationStats> S,uint8 Mode=0)
        :Owner(Id),WireChannel(Channel),Stats(std::move(S)),OutputMode(Mode){}
    ~FRequestedPresentationProbe(){Stats->DestroyedThread=FPlatformTLS::GetCurrentThreadId();}
    uint64 OwnerStableId()const override{Stats->GetterThread=FPlatformTLS::GetCurrentThreadId();return Owner;}
    uint32 Channel()const override{Stats->GetterThread=FPlatformTLS::GetCurrentThreadId();return WireChannel;}
    void Produce(const FSimulationSnapshot& S,FSimulationPresentationOutput& Out)override
    {
        Stats->ProduceThread=FPlatformTLS::GetCurrentThreadId();++Stats->Calls;
        if(OutputMode==1)throw std::runtime_error("intentional observed producer failure");
        // Test values only. Common publisher must supply address/frame/serial.
        for(unsigned I=0;I<8;++I)Out.Payload.Add(uint8(S.NumFrame>>(8*I)));
        if(OutputMode==2)Out.Payload.SetNumUninitialized(17); // admission cap is16
        if(OutputMode==3)Out.StatePayload.Add(1); // causal payload forbidden here
    }
    void InvalidateTimeline()override{Stats->InvalidateThread=FPlatformTLS::GetCurrentThreadId();++Stats->Invalidates;}
private:
    uint64 Owner;uint32 WireChannel;std::shared_ptr<FRequestedPresentationStats> Stats;uint8 OutputMode;
};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedNativeRequestedRegistry,
    "IAmSpeed.Netcode.RequestedRegistry.NativeOwnerCompletedPair",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedNativeRequestedRegistry::RunTest(const FString&)
{
    using namespace Speed;
    using namespace Speed::Input;
    using namespace Speed::Input::V2;
    using namespace Speed::Netcode;
    constexpr uint32 Frames=4;
    auto* Backend=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.StaticCollision.Backend"));
    if(!TestNotNull(TEXT("actual Analytic selector"),Backend))return false;
    const int32 PreviousBackend=Backend->GetInt();Backend->Set(2,ECVF_SetByCode);
    ON_SCOPE_EXIT{Backend->Set(PreviousBackend,ECVF_SetByCode);};
    const auto Options=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(false)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false).SetTransactional(false);
    UWorld* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Options);
    if(!TestNotNull(TEXT("isolated actual native world"),World))return false;
    GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
    ON_SCOPE_EXIT{World->DestroyWorld(false);GEngine->DestroyWorldContext(World);};
    auto* Driver=World->SpawnActor<ARealTimeSimulation>();auto* Bridge=World->GetSubsystem<USpeedWorldSubsystem>();
    if(!TestNotNull(TEXT("actual driver"),Driver)||!TestNotNull(TEXT("actual native registry"),Bridge))return false;
    Driver->SpeedWorldSubsystem=Bridge;
    Driver->ActiveExecutionModeValue.Store(static_cast<uint8>(ESimulationExecutionMode::IAmSpeedThread));
    Driver->bPublishCompletedWorldInput=true;
    // The source identity is minted by the original lifecycle API, never the fixture.
    if(!TestTrue(TEXT("renew real input generation while stopped"),Driver->PrepareControlledInputRun()))return false;
    auto Data=MakeShared<Speed::Analytic::FAnalyticWorldData>();
    if(!TestTrue(TEXT("explicit empty Analytic authority"),Data->FinalizeAndValidate()))return false;
    Bridge->AnalyticWorldData=Data;
    Bridge->SimulationWorld.SetStaticCollisionWorld(MakeUnique<FAnalyticStaticCollisionWorld>(*Data,1,1,1));
    Chaos::FSimpleWheelConfig WheelConfig;Chaos::FSimpleSuspensionConfig SuspensionConfig;
    SuspensionConfig.SetSuspensionMaxRaise(10);SuspensionConfig.SetSuspensionMaxDrop(25);
    TArray<USpeedWheeledComponent*> Cars;
    ON_SCOPE_EXIT
    {
        Driver->JoinOwnedSimulationForInputTeardown();
        for(auto* C:Cars)
        {
            C->ClearWheelSimulationPointers();C->SkySimulation=nullptr;C->VehicleSimulationPT.Reset();
            Bridge->UnregisterSpeedComponent(C);
        }
        Bridge->ApplyPendingOps();
    };
    AddExpectedMessagePlain(TEXT("[WheelSuspensionBinding] subbodies="),ELogVerbosity::Display,EAutomationExpectedMessageFlags::Contains,2);
    for(int32 A=0;A<2;++A)
    {
        auto* Car=World->SpawnActor<ASpeedCar>();auto* C=Car?Cast<USpeedWheeledComponent>(Car->GetVehicleMovement()):nullptr;
        if(!TestNotNull(TEXT("real wheeled component"),C))return false;
        C->SetOwner(Car);C->DisableAsyncPhysicsTickForTesting();Cars.Add(C);
        C->CanonicalCameraYawAction=-1;C->CanonicalCameraPitchAction=-1;
        auto Vehicle=MakeUnique<Chaos::FSimpleWheeledVehicle>();
        for(int32 I=0;I<4;++I)
        {Vehicle->Wheels.Emplace(&WheelConfig);Vehicle->Suspension.Emplace(&SuspensionConfig);C->Wheels.Add(NewObject<UChaosVehicleWheel>(C));}
        C->VehicleSimulationPT=MakeUnique<USpeedSimulation>();
        C->SkySimulation=static_cast<USpeedSimulation*>(C->VehicleSimulationPT.Get());C->SkySimulation->Init(Vehicle);C->BindWheelSimulationPointers();
        FString Reason;if(!TestTrue(TEXT("production wheel storage admitted"),C->ValidateSimulationBindings(Reason)))return false;
        C->SetEngineFPS(300);C->SetImmediateMovementForTesting();
        SKinematic K;K.Location=FVector(2500*A,0,10000);K.Velocity=FVector(300,0,0);C->SetKinematicState(K);
    }
    Bridge->ApplyPendingOps();
    if(!TestTrue(TEXT("real world and bodies ready"),Driver->EnsureSimulationWorldReady()))return false;
    Driver->InitializeCanonicalFrame(0);
    FInputActionContractDescription Description;Description.Revision={1};Description.Actions=FInputActionContract::BaseActions();
    Description.Physical={{Throttle,EPhysicalDestination::Throttle},{Brake,EPhysicalDestination::Brake},{Steering,EPhysicalDestination::Steering}};
    const auto Contract=FInputActionContract::Create(Description);
    if(!TestTrue(TEXT("actual driving contract"),bool(Contract)))return false;
    std::shared_ptr<FNetworkRequestedInputService> Services[2];
    std::unique_ptr<TNetworkInputWorkerSession<>::FIngress> Ingresses[2];
    FBoundaryCommandDescriptor Commands[2];
    for(int32 A=0;A<2;++A)
    {
        FNetworkInputWorkerConfiguration C;C.Consumer=ENetworkInputConsumer::DedicatedServer;
        C.Binding={501,502,uint64(200+A),1,{}};C.Binding.ContractId[0]=1;
        C.AuthenticatedPeer=17+A;C.Original.Producer=18+A;C.Original.StreamEpoch=19;C.Original.DeviceGeneration=20;
        C.AllowLocalFallback=true;C.PredictionFrames=1;C.Prediction={FNetworkInputPredictionPolicy::Hold};
        Services[A]=FNetworkRequestedInputService::Create(C,Contract);
        if(!TestTrue(TEXT("inert service configured"),bool(Services[A])))return false;
        Ingresses[A]=Services[A]->BindIngressOnCurrentThread();
        FInputFrameData Original;Original.SourceSequence=100;Original.ConsumptionFrame=0;
        Original.Producer={EProducerKind::Device,C.Original.Producer};Original.StreamEpoch={19};Original.DeviceGeneration={20};
        Original.Reset=true;Original.Values[Throttle]=A?60:200;Original.ActiveMask=uint32(1)<<Throttle;
        if(!TestTrue(TEXT("true original baseline queued without native capability"),Ingresses[A]
            &&Ingresses[A]->TrySubmitOriginal(C.AuthenticatedPeer,Speed::Input::V2::FInputFrame(Contract,Original))==ENetworkInputAdmission::Queued))return false;
        auto& Cmd=Commands[A];Cmd.Id=Driver->AllocateInputCommandId();Cmd.WorkerGeneration=Driver->GetInputWorkerGeneration();Cmd.RegistryVersion=A;
        auto& D=Cmd.Binding;D.Id=1000+A;D.Epoch=1;D.Contract=Contract;D.Kind=EProducerContract::ExactRemote;
        D.Processing.Step[Throttle]=16;
        const auto LocalId=Bridge->GetSimulationStableId(*Cars[A]);D.Actors={{LocalId,LocalId}};
        FRequestedNetworkSessionDescriptor N;N.Service=5000+A;N.SourceGeneration=100+A;N.WorldGeneration=Cmd.WorkerGeneration;
        N.Session=C.Binding.Session;N.Epoch=C.Binding.Epoch;N.Actor=C.Binding.Actor;N.Incarnation=1;N.Component=1;
        N.LocalComponent=N.LocalComponentGeneration=LocalId;N.LocalBodyGeneration=Cars[A]->GetCanonicalRequestedBodyGeneration();
        N.WireContract=C.Binding.ContractId;N.DigitalPolicy=ERequestedDigitalPolicy::HeldStateAndCancelInactive;D.Network=N;
    }
    std::shared_ptr<const Speed::FCompletedSimulationPublication> Retained[Frames];
    std::shared_ptr<const Speed::FNativeRequestedInputObservation> BindingObservation,StaleObservation;
    constexpr uint32 PresentationChannel=0x49415301,FailureChannel=0x49415302,LegacyChannel=0x49415308;
    auto PresentationA=std::make_shared<FRequestedPresentationStats>();
    auto PresentationB=std::make_shared<FRequestedPresentationStats>();
    auto PresentationFailure=std::make_shared<FRequestedPresentationStats>();
    auto PresentationStale=std::make_shared<FRequestedPresentationStats>();
    auto PresentationDuplicate=std::make_shared<FRequestedPresentationStats>();
    auto PresentationWrongOwner=std::make_shared<FRequestedPresentationStats>();
    auto PresentationPaused=std::make_shared<FRequestedPresentationStats>();
    auto PresentationOversize=std::make_shared<FRequestedPresentationStats>();
    auto PresentationState=std::make_shared<FRequestedPresentationStats>();
    auto PresentationLegacy=std::make_shared<FRequestedPresentationStats>();
    auto PresentationLegacyCollision=std::make_shared<FRequestedPresentationStats>();
    auto PresentationLegacyReplacement=std::make_shared<FRequestedPresentationStats>();
    uint64 RegistrationA=0,RegistrationB=0,RegistrationFailure=0,RegistrationOversize=0,RegistrationState=0,RegistrationLegacyReplacement=0,RemovalA=0;
    TestFalse(TEXT("no native observation before actual binding"),bool(Driver->ReadNativeRequestedInputObservation()));
    uint32 Steps=0,OwnerThread=0,Decisions=0;int32 FailureLine=0;
    std::atomic<bool> AwaitingLateIngress{false},LateIngressPublished{false},AwaitingLifecycle{false};
    Driver->BeforeRollbackDecisionForTesting=[&](uint64 N)
    {
        ++Decisions;
        if(!Driver->InputSessionRegistry||!Driver->InputSessionRegistry->RequestedSourcesFrozenFor(N))
        {FailureLine=__LINE__;return false;}
        if(N==1)
        {
            // Test-only barrier, bounded at two seconds. No production wait is
            // added: the actual ingress owner queues authority after the cutoff.
            AwaitingLateIngress=true;const double End=FPlatformTime::Seconds()+2;
            while(!LateIngressPublished&&FPlatformTime::Seconds()<End)FPlatformProcess::SleepNoStats(.001f);
            if(!LateIngressPublished){FailureLine=__LINE__;return false;}
        }
        if(N==2)
        {
            const auto Confirmed=Driver->InputSessionRegistry->ReadConfirmedRequestedInput(1000,1,1);
            const auto Prior=Driver->ReadCompletedInputFrame();
            if(!Confirmed||!Confirmed->GetOriginal()||Confirmed->GetOriginal()->GetData().SourceSequence!=101
                ||Confirmed->GetState().Values[Throttle]!=240||!Prior||Prior->Frame!=1)
            {FailureLine=__LINE__;return false;}
        }
        if(N==3)
        {
            Retained[2]=Driver->ReadCompletedWorldAndInputFrame();
            if(!Retained[2]||Retained[2]->World().NumFrame!=2
                ||Cars[0]->WheeledPhysicalInput.Throttle!=216||Cars[1]->WheeledPhysicalInput.Throttle!=44)
            {FailureLine=__LINE__;return false;}
        }
        return true;
    };
    std::atomic<bool> Failed{false};
    Driver->SimulationWorker=MakeUnique<FSimulationWorker>([&]()
    {
        if(Steps==Frames)
        {
            AwaitingLifecycle=true;
            return ESimulationWorkerResult::Idle;
        }
        OwnerThread=FPlatformTLS::GetCurrentThreadId();const uint32 First=Steps,Count=First==2?2:1;
        const auto Refuse=[&](int32 Line){if(!FailureLine)FailureLine=Line;Failed=true;return ESimulationWorkerResult::Failed;};
        const bool Advanced=Count==2?Driver->RunCanonicalFrames(Count):
            static_cast<ASpeedPhysicsSimulation*>(Driver)->DriveOwnedWorkerPulse(1.f/300,float(First)/300)==ESimulationWorkerResult::Advanced;
        if(!Advanced||Driver->CanonicalNumFrame!=First+Count)return Refuse(__LINE__);
        for(uint32 N=First;N<First+Count;++N)
        {
        auto Pair=N==2?Retained[2]:Driver->ReadCompletedWorldAndInputFrame();
        const auto Completed=Pair?Pair->ConsumedInputs():nullptr;
        if(!Pair||Pair->World().NumFrame!=N||Pair->World().PublicationSerial!=N+1||Pair->World().Payload.IsEmpty()
            ||!Completed||Pair->ConsumedInputs()!=Completed||Completed->Inputs.size()!=2||!Pair->RegistryAtFrame()
            ||Pair->RegistryAtFrame()!=Completed->Registry||Pair->RegistryAtFrame()->Version!=2)return Refuse(__LINE__);
        // Native managed output is attached to this same completed pair by
        // the common publisher; no GT pose copy or guessed serial is accepted.
        if(Pair->World().PresentationBodies.Num()!=2)return Refuse(__LINE__);
        for(int32 A=0;A<2;++A)
        {
            const uint64 Owner=Commands[A].Binding.Network->LocalComponent;
            const auto* Output=Pair->World().PresentationOutputs.FindByPredicate([&](const auto& X){return X.OwnerStableId==Owner&&X.Channel==PresentationChannel;});
            if((A==0&&N>=2)?Output!=nullptr:Output==nullptr)return Refuse(__LINE__);
            if(Output)
            {
                if(Output->NumFrame!=N||Output->PublicationSerial!=Pair->World().PublicationSerial
                    ||Output->Payload.Num()!=8||Output->StatePayload.Num())return Refuse(__LINE__);
                for(unsigned I=0;I<8;++I)if(Output->Payload[int32(I)]!=uint8(uint64(N)>>(8*I)))return Refuse(__LINE__);
            }
        }
        for(const uint32 Channel:{FailureChannel,FailureChannel+1,FailureChannel+2})
        {
            const auto* FailedOutput=Pair->World().PresentationOutputs.FindByPredicate([&](const auto& X){return X.Channel==Channel;});
            if(N==0?(!FailedOutput||!FailedOutput->Payload.IsEmpty()||!FailedOutput->StatePayload.IsEmpty()):FailedOutput!=nullptr)return Refuse(__LINE__);
        }
        const auto* LegacyOutput=Pair->World().PresentationOutputs.FindByPredicate([&](const auto& X){return X.Channel==LegacyChannel;});
        if(N==1?LegacyOutput!=nullptr:(!LegacyOutput||LegacyOutput->Payload.Num()!=8||LegacyOutput->NumFrame!=N||LegacyOutput->PublicationSerial!=Pair->World().PublicationSerial))return Refuse(__LINE__);
        const auto Observed=Driver->ReadNativeRequestedInputObservation();
        if(!Observed||Observed!=BindingObservation||Observed->Source()!=Pair->Source()
            ||Observed->RegistryAtBoundary()!=Pair->RegistryAtFrame()||Observed->BoundaryFrame()!=0
            ||Observed->PhysicalDeltaTime()!=FCanonicalFrameContext::CanonicalPhysicalDeltaTime
            ||Observed->Bindings().size()!=2)return Refuse(__LINE__);
        for(int32 A=0;A<2;++A)
        {
            const auto& Captured=Observed->Bindings()[A];
            const auto& Config=Captured.Configuration;
            if(Captured.Descriptor.Id!=Commands[A].Binding.Id||Captured.Descriptor.Epoch!=1
                ||!Captured.Descriptor.Network||!(*Captured.Descriptor.Network==*Commands[A].Binding.Network)
                ||Captured.Descriptor.Processing.Step[Throttle]!=16||Config.AuthenticatedPeer!=uint64(17+A)
                ||Config.Binding.Session!=501||Config.Binding.Epoch!=502||Config.Binding.Actor!=uint64(200+A)
                ||Config.FirstFrame!=0||Config.Consumer!=ENetworkInputConsumer::DedicatedServer
                ||Config.PredictionFrames!=1||!Config.AllowLocalFallback)return Refuse(__LINE__);
            const auto Found=std::find_if(Completed->Inputs.begin(),Completed->Inputs.end(),[&](const auto& I){return I.Session==Commands[A].Binding.Id;});
            if(Found==Completed->Inputs.end()||Found->Snapshot||!Found->Requested||!Found->RequestedMapping
                ||!(*Found->RequestedMapping==*Commands[A].Binding.Network))return Refuse(__LINE__);
            const auto& R=*Found->Requested;const auto& Request=*R.Request;
            if(Captured.OwnerBindingFingerprint!=R.BindingFingerprint)return Refuse(__LINE__);
            const auto ExpectedOrigin=N==0?ERequestedInputOrigin::ConfirmedNetwork:(N==1||(!A&&N==2))?ERequestedInputOrigin::LocalPrediction:ERequestedInputOrigin::LocalFallback;
            const int16 Initial=A?60:200;const int16 Expected=A?(N<2?Initial:int16(Initial-16*(N-1))):(N<2?200:N==2?216:200);
            if(Request.GetState().Frame!=N||Request.GetOrigin()!=ExpectedOrigin||bool(Request.GetOriginal())!=(N==0)
                ||R.Cutoff.Ordinal.Value!=N+1||R.Applied[Throttle]!=Expected||R.CancelDigitalMask
                ||(N==First+Count-1&&Cars[A]->WheeledPhysicalInput.Throttle!=Expected)
                ||Cars[A]->WheeledPhysicalInput.bCanMove||Cars[A]->CanMove())return Refuse(__LINE__);
            // Authority and actually consumed predictions remain separate histories.
            if(N==0&&!Driver->InputSessionRegistry->ReadConfirmedRequestedInput(1000+A,1,0))return Refuse(__LINE__);
            if(N>0&&Driver->InputSessionRegistry->ReadConfirmedRequestedInput(1000+A,1,N))return Refuse(__LINE__);
        }
        Retained[N]=std::move(Pair);++Steps;
        }
        return ESimulationWorkerResult::Advanced;
    },[&](FSimulationWorkerWaitContext& Wait){static_cast<ASpeedPhysicsSimulation*>(Driver)->WaitBetweenFrames(Wait);},
    [&](bool Paused){return Driver->ServiceInputSessionBoundary(Paused);},[&]{Driver->CloseInputSessionsOnWorker();});
    auto* Worker=Driver->SimulationWorker.Get();
    ON_SCOPE_EXIT{Driver->JoinOwnedSimulationForInputTeardown();};
    if(!TestTrue(TEXT("actual owned worker starts paused"),Worker->Start(true)))return false;
    for(int32 A=0;A<2;++A)
    {
        if(!TestTrue(TEXT("inert descriptor queued through native host"),Driver->SubmitRequestedNetworkInputSession(Commands[A],Services[A])==ECommandAdmission::Enqueued))return false;
        const double Deadline=FPlatformTime::Seconds()+5;
        std::optional<FBoundaryReceipt> Receipt;
        do{Receipt=Driver->ReadInputSessionReceipt(Commands[A].Id);
            if(Receipt&&Receipt->Status!=EBoundaryStatus::Pending&&Receipt->Status!=EBoundaryStatus::WaitingForBaseline)break;
            FPlatformProcess::SleepNoStats(.001f);}
        while(Worker->IsRunning()&&FPlatformTime::Seconds()<Deadline);
        if(!TestTrue(TEXT("real boundary admits one canonical actor into its distinct local binding"),Receipt&&Receipt->Status==EBoundaryStatus::Applied))return false;
        if(A==0)
        {
            const double End=FPlatformTime::Seconds()+2;
            do{StaleObservation=Driver->ReadNativeRequestedInputObservation();if(StaleObservation&&StaleObservation->Bindings().size()==1)break;FPlatformProcess::SleepNoStats(.001f);}
            while(Worker->IsRunning()&&FPlatformTime::Seconds()<End);
            if(!TestTrue(TEXT("retain genuine old registry observation"),StaleObservation&&StaleObservation->RegistryAtBoundary()->Version==1))return false;
        }
    }
    const double ObservationDeadline=FPlatformTime::Seconds()+2;
    do
    {
        BindingObservation=Driver->ReadNativeRequestedInputObservation();
        if(BindingObservation&&BindingObservation->Bindings().size()==2)break;
        FPlatformProcess::SleepNoStats(.001f);
    }while(Worker->IsRunning()&&FPlatformTime::Seconds()<ObservationDeadline);
    if(!TestTrue(TEXT("real source/configuration captured before first physics"),BindingObservation
        &&BindingObservation->Source()&&BindingObservation->RegistryAtBoundary()
        &&BindingObservation->RegistryAtBoundary()->Version==2&&BindingObservation->Bindings().size()==2))return false;
    const auto WaitPresentation=[&](uint64 Id)
    {
        const double End=FPlatformTime::Seconds()+2;
        do
        {
            const auto R=Driver->ReadRequestedInputPresentationReceipt(Id);
            if(R&&R->Status!=Speed::ERequestedPresentationStatus::Pending)return R;
            FPlatformProcess::SleepNoStats(.001f);
        }while(Worker->IsRunning()&&FPlatformTime::Seconds()<End);
        return std::optional<Speed::FRequestedPresentationReceipt>{};
    };
    const auto SubmitPresentation=[&](auto Observation,uint64 Session,uint64 Owner,uint32 Channel,
        std::shared_ptr<FRequestedPresentationStats> Stats,uint8 Mode=0)
    {
        Speed::FRequestedPresentationDescriptor D;D.Id=Driver->AllocateInputCommandId();D.WorkerGeneration=Driver->GetInputWorkerGeneration();
        D.Session=Session;D.Epoch=1;D.Channel=Channel;D.MaxPayloadBytes=16;D.Observation=std::move(Observation);
        const auto Id=D.Id;
        TestTrue(TEXT("GT queues exclusive presentation values without producer callbacks"),Driver->SubmitRequestedInputPresentation(std::move(D),
            std::make_unique<FRequestedPresentationProbe>(Owner,Channel,std::move(Stats),Mode))==Speed::ERequestedPresentationAdmission::Enqueued);
        return Id;
    };
    const uint64 OwnerA=Commands[0].Binding.Network->LocalComponent,OwnerB=Commands[1].Binding.Network->LocalComponent;
    const auto StaleId=SubmitPresentation(StaleObservation,1000,OwnerA,PresentationChannel,PresentationStale);
    const auto StaleReceipt=WaitPresentation(StaleId);
    if(!TestTrue(TEXT("old exact source but stale registry rejected before producer callbacks"),StaleReceipt
        &&StaleReceipt->Status==Speed::ERequestedPresentationStatus::Rejected
        &&StaleReceipt->Reason==Speed::ERequestedPresentationReason::NotCurrentObservation&&PresentationStale->GetterThread==0))return false;
    const auto LegacyProducer=MakeShared<FRequestedPresentationProbe,ESPMode::ThreadSafe>(OwnerB,LegacyChannel,PresentationLegacy);
    TestTrue(TEXT("legacy presentation registration remains available"),Driver->RegisterPresentationProducer(LegacyProducer));
    RegistrationA=SubmitPresentation(BindingObservation,1000,OwnerA,PresentationChannel,PresentationA);
    RegistrationB=SubmitPresentation(BindingObservation,1001,OwnerB,PresentationChannel,PresentationB);
    RegistrationFailure=SubmitPresentation(BindingObservation,1000,OwnerA,FailureChannel,PresentationFailure,1);
    RegistrationOversize=SubmitPresentation(BindingObservation,1000,OwnerA,FailureChannel+1,PresentationOversize,2);
    RegistrationState=SubmitPresentation(BindingObservation,1000,OwnerA,FailureChannel+2,PresentationState,3);
    for(const auto Id:{RegistrationA,RegistrationB,RegistrationFailure,RegistrationOversize,RegistrationState})
    {const auto Receipt=WaitPresentation(Id);if(!TestTrue(TEXT("actual owned boundary applied managed presentation"),Receipt&&Receipt->Status==Speed::ERequestedPresentationStatus::Applied))return false;}
    const auto LegacyCollisionId=SubmitPresentation(BindingObservation,1001,OwnerB,LegacyChannel,PresentationLegacyCollision);
    const auto LegacyCollisionReceipt=WaitPresentation(LegacyCollisionId);
    TestTrue(TEXT("managed admission cannot collide with a legacy output address"),LegacyCollisionReceipt&&LegacyCollisionReceipt->Status==Speed::ERequestedPresentationStatus::Rejected&&LegacyCollisionReceipt->Reason==Speed::ERequestedPresentationReason::DuplicateAddress);
    const auto DuplicateId=SubmitPresentation(BindingObservation,1000,OwnerA,PresentationChannel,PresentationDuplicate);
    const auto DuplicateReceipt=WaitPresentation(DuplicateId);
    TestTrue(TEXT("duplicate bound address rejected"),DuplicateReceipt&&DuplicateReceipt->Status==Speed::ERequestedPresentationStatus::Rejected&&DuplicateReceipt->Reason==Speed::ERequestedPresentationReason::DuplicateAddress);
    const auto WrongId=SubmitPresentation(BindingObservation,1000,OwnerA+1000,PresentationChannel+1,PresentationWrongOwner);
    const auto WrongReceipt=WaitPresentation(WrongId);
    TestTrue(TEXT("producer owner must match actual mapped body"),WrongReceipt&&WrongReceipt->Status==Speed::ERequestedPresentationStatus::Rejected&&WrongReceipt->Reason==Speed::ERequestedPresentationReason::InvalidProducer);
    TestFalse(TEXT("binding observation is not a completed physical pair"),bool(Driver->ReadCompletedWorldAndInputFrame()));
    TestFalse(TEXT("GT cannot borrow the owned worker to freeze a frame"),Driver->RunCanonicalFrames(1));
    const uint32 GT=FPlatformTLS::GetCurrentThreadId();Driver->ResumeOwnedSimulation();
    const double Deadline=FPlatformTime::Seconds()+10;
    bool SubmittedLate=false,LifecycleHandled=false;
    while(Worker->IsRunning()&&FPlatformTime::Seconds()<Deadline)
    {
        if(AwaitingLateIngress&&!SubmittedLate)
        {
            FInputFrameData Late;Late.SourceSequence=101;Late.ConsumptionFrame=1;
            Late.Producer={EProducerKind::Device,18};Late.StreamEpoch={19};Late.DeviceGeneration={20};
            Late.Values[Throttle]=240;Late.ActiveMask=uint32(1)<<Throttle;
            check(Contract->Find(Throttle)->Accepts(Late.Values[Throttle]));
            TestTrue(TEXT("actual ingress queues late authority after N1 cutoff"),Ingresses[0]->TrySubmitOriginal(17,
                Speed::Input::V2::FInputFrame(Contract,Late))==ENetworkInputAdmission::Queued);
            Speed::FRequestedPresentationDescriptor Remove;Remove.Id=Driver->AllocateInputCommandId();RemovalA=Remove.Id;
            Remove.WorkerGeneration=Driver->GetInputWorkerGeneration();Remove.Operation=Speed::ERequestedPresentationOperation::Remove;Remove.TargetRegistration=RegistrationA;
            TestTrue(TEXT("removal queued during N1 without an in-frame mutation"),Driver->SubmitRequestedInputPresentation(std::move(Remove))==Speed::ERequestedPresentationAdmission::Enqueued);
            // Preserve the existing GT-only legacy removal behavior. The new
            // managed route must then be able to claim its freed address at N2.
            Driver->UnregisterPresentationProducer(LegacyProducer);
            RegistrationLegacyReplacement=SubmitPresentation(BindingObservation,1001,OwnerB,LegacyChannel,PresentationLegacyReplacement);
            SubmittedLate=true;LateIngressPublished=true;
        }
        if(AwaitingLifecycle&&!LifecycleHandled)
        {
            const auto WaitReceipt=[&](uint64 Id)
            {
                const double End=FPlatformTime::Seconds()+2;
                do
                {
                    const auto R=Driver->ReadInputSessionReceipt(Id);
                    if(R&&R->Status!=EBoundaryStatus::Pending&&R->Status!=EBoundaryStatus::WaitingForBaseline)
                        return R->Status==EBoundaryStatus::Applied;
                    FPlatformProcess::SleepNoStats(.001f);
                }while(Worker->IsRunning()&&FPlatformTime::Seconds()<End);
                return false;
            };
            FBoundaryCommandDescriptor Pause;Pause.Id=Driver->AllocateInputCommandId();
            Pause.WorkerGeneration=Driver->GetInputWorkerGeneration();Pause.RegistryVersion=2;Pause.Operation=EBoundaryOperation::PauseAll;
            const bool PauseQueued=Driver->SubmitInputSessionCommand(Pause)==ECommandAdmission::Enqueued;
            TestTrue(TEXT("real paused boundary retires requested sources"),PauseQueued&&WaitReceipt(Pause.Id));
            const double End=FPlatformTime::Seconds()+2;
            while(Driver->ReadNativeRequestedInputObservation()&&Worker->IsRunning()&&FPlatformTime::Seconds()<End)
                FPlatformProcess::SleepNoStats(.001f);
            TestFalse(TEXT("pause clears published binding observation without mutating retained history"),bool(Driver->ReadNativeRequestedInputObservation()));
            std::optional<Speed::FRequestedPresentationReceipt> BReceipt;
            const double RetireEnd=FPlatformTime::Seconds()+2;
            do
            {
                BReceipt=Driver->ReadRequestedInputPresentationReceipt(RegistrationB);
                if(BReceipt&&BReceipt->Status==Speed::ERequestedPresentationStatus::Retired)break;
                FPlatformProcess::SleepNoStats(.001f);
            }while(Worker->IsRunning()&&FPlatformTime::Seconds()<RetireEnd);
            TestTrue(TEXT("PauseAll retires managed producer automatically"),BReceipt&&BReceipt->Status==Speed::ERequestedPresentationStatus::Retired&&PresentationB->Invalidates==1);
            const auto PausedId=SubmitPresentation(BindingObservation,1000,OwnerA,PresentationChannel+3,PresentationPaused);
            const auto PausedReceipt=WaitPresentation(PausedId);
            TestTrue(TEXT("retained observation after PauseAll cannot register producer"),PausedReceipt&&PausedReceipt->Status==Speed::ERequestedPresentationStatus::Rejected
                &&PausedReceipt->Reason==Speed::ERequestedPresentationReason::Inactive&&PresentationPaused->GetterThread==0);
            FBoundaryCommandDescriptor Detach;Detach.Id=Driver->AllocateInputCommandId();
            Detach.WorkerGeneration=Driver->GetInputWorkerGeneration();Detach.RegistryVersion=2;
            Detach.Operation=EBoundaryOperation::Detach;Detach.Session=1000;Detach.Epoch=1;
            const bool DetachQueued=Driver->SubmitInputSessionCommand(Detach)==ECommandAdmission::Enqueued;
            TestTrue(TEXT("real detach completes on the same owned boundary"),DetachQueued&&WaitReceipt(Detach.Id));
            TestTrue(TEXT("retained registry and configuration preserve the pre-detach observation"),BindingObservation
                &&BindingObservation->RegistryAtBoundary()->Version==2&&BindingObservation->Bindings().size()==2
                &&BindingObservation->Bindings()[0].Descriptor.Id==1000);
            Driver->ResumeOwnedSimulation();
            TestTrue(TEXT("closed requested source is correctly kept paused until a new admission"),Worker->IsPaused());
            // The public resume guard must refuse these closed sources. Join
            // deliberately below; no physical N4 and no bypass of that guard.
            LifecycleHandled=true;break;
        }
        FPlatformProcess::SleepNoStats(.001f);
    }
    const bool Finished=LifecycleHandled||!Worker->IsRunning();
    if(!TestTrue(TEXT("original native owner joined"),Driver->JoinOwnedSimulationForInputTeardown()))return false;
    if(Failed)AddInfo(FString::Printf(TEXT("[RequestedNativeFirstFailure] Steps=%u Line=%d"),Steps,FailureLine));
    TestTrue(TEXT("all four actual physical frames completed"),Finished&&!Failed&&Steps==Frames);
    TestTrue(TEXT("one frozen cutoff precedes each decision entry"),Decisions==Frames);
    TestTrue(TEXT("real pause/detach lifecycle was exercised after the four physics frames"),LifecycleHandled);
    const auto AReceipt=Driver->ReadRequestedInputPresentationReceipt(RegistrationA);
    const auto RemovalReceipt=Driver->ReadRequestedInputPresentationReceipt(RemovalA);
    const auto FailedReceipt=Driver->ReadRequestedInputPresentationReceipt(RegistrationFailure);
    TestTrue(TEXT("explicit removal completed at next boundary"),AReceipt&&AReceipt->Status==Speed::ERequestedPresentationStatus::Retired&&AReceipt->Reason==Speed::ERequestedPresentationReason::Removed
        &&PresentationA->Calls==2&&PresentationA->Invalidates==1&&RemovalReceipt&&RemovalReceipt->Status==Speed::ERequestedPresentationStatus::Applied);
    TestTrue(TEXT("producer exception recorded without blocking four physical frames"),FailedReceipt&&FailedReceipt->Status==Speed::ERequestedPresentationStatus::Failed
        &&FailedReceipt->Reason==Speed::ERequestedPresentationReason::ProducerFailure&&PresentationFailure->Calls==1&&PresentationFailure->Invalidates==1);
    TestTrue(TEXT("remaining managed producer emitted four frames"),PresentationB->Calls==4);
    for(const auto Id:{RegistrationOversize,RegistrationState})
    {
        const auto Receipt=Driver->ReadRequestedInputPresentationReceipt(Id);
        TestTrue(TEXT("oversize and state payload failures are explicit, without physical interruption"),Receipt&&Receipt->Status==Speed::ERequestedPresentationStatus::Failed&&Receipt->Reason==Speed::ERequestedPresentationReason::InvalidPayload);
    }
    TestTrue(TEXT("invalid payload producers ran once then retired"),PresentationOversize->Calls==1&&PresentationOversize->Invalidates==1&&PresentationState->Calls==1&&PresentationState->Invalidates==1);
    TestTrue(TEXT("legacy removal preserves its prior invalidate semantics"),PresentationLegacy->Calls==1&&PresentationLegacy->Invalidates==0);
    const auto ReplacementReceipt=Driver->ReadRequestedInputPresentationReceipt(RegistrationLegacyReplacement);
    TestTrue(TEXT("managed replacement of freed legacy address retired on PauseAll"),ReplacementReceipt&&ReplacementReceipt->Status==Speed::ERequestedPresentationStatus::Retired&&PresentationLegacyReplacement->Calls==2&&PresentationLegacyReplacement->Invalidates==1);
    for(const auto& Stats:{PresentationA,PresentationB,PresentationFailure,PresentationOversize,PresentationState,PresentationLegacyReplacement})
        TestTrue(TEXT("produce invalidation destruction stay on actual canonical owner"),Stats->GetterThread==OwnerThread&&Stats->ProduceThread==OwnerThread
            &&Stats->InvalidateThread==OwnerThread&&Stats->DestroyedThread==OwnerThread&&OwnerThread!=GT);
    TestTrue(TEXT("late ingress was exercised, without rewriting N1 consumption"),SubmittedLate&&Retained[1]
        &&Retained[1]->ConsumedInputs()->Inputs[0].Requested->Applied[Throttle]==200);
    TestTrue(TEXT("physical owner independent of GT"),OwnerThread&&OwnerThread!=GT);
    TestFalse(TEXT("retired source cannot present current readiness"),bool(Driver->ReadCompletedWorldAndInputFrame()));
    TestFalse(TEXT("retired driver publishes no new binding observation"),bool(Driver->ReadNativeRequestedInputObservation()));
    TestTrue(TEXT("retained binding/source observation remains historical after join"),BindingObservation&&Retained[0]
        &&BindingObservation->Source()==Retained[0]->Source()
        &&BindingObservation->RegistryAtBoundary()==Retained[0]->RegistryAtFrame()
        &&BindingObservation->Bindings()[0].Configuration.PredictionFrames==1);
    for(uint32 N=0;N<Frames;++N)TestTrue(TEXT("immutable completed historical pair remains readable after join"),Retained[N]&&Retained[N]->World().NumFrame==N);
    AddInfo(TEXT("[RequestedNativeScope] managed observational producers registered on actual owner; exact-source/current-registry/lifecycle revalidation; stale/duplicate/wrong-owner rejection; real same-pair outputs and serials; producer failure isolated; owner-only retirement; real immutable Source/registry/config binding observation before first frame and after join; no host approval; two real bodies; cutoff before decision; post-cutoff authority deferred to N2; N1 immutable; Original/Prediction/Fallback; exact completed endpoint; CanMove closed; no socket, corrective replay, collision or controller qualification"));
    return !HasAnyErrors();
}
#endif
