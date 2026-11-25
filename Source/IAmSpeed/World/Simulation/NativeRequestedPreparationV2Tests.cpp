#include "Misc/Build.h"
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "NativeRequestedPreparationFixture.h"
#include "SpeedPhysicsSimulation.h"
#include "RealTimeSimulation.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "IAmSpeed/Netcode/agreement/HostAgreementHost.h"
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedNativeRequestedPreparationV2Test,
    "IAmSpeed.Netcode.RequestedRegistry.NativePreparationObservedHostComparedOnly",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)

bool FIAmSpeedNativeRequestedPreparationV2Test::RunTest(const FString&)
{
    using namespace Speed;using namespace Speed::Input::V2;
    using namespace Speed::Netcode;using namespace Speed::Netcode::Agreement;
    auto* Backend=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.StaticCollision.Backend"));
    if(!TestNotNull(TEXT("actual analytic selector"),Backend))return false;
    const int32 OldBackend=Backend->GetInt();Backend->Set(2,ECVF_SetByCode);
    ON_SCOPE_EXIT{Backend->Set(OldBackend,ECVF_SetByCode);};
    const auto Options=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(false)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false).SetTransactional(false);
    auto* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Options);
    if(!TestNotNull(TEXT("actual isolated native world"),World))return false;
    GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
    ON_SCOPE_EXIT{World->DestroyWorld(false);GEngine->DestroyWorldContext(World);};
    auto* Driver=World->SpawnActor<ARealTimeSimulation>();auto* Bridge=World->GetSubsystem<USpeedWorldSubsystem>();
    if(!TestNotNull(TEXT("actual driver"),Driver)||!TestNotNull(TEXT("actual bridge"),Bridge))return false;
    Driver->SpeedWorldSubsystem=Bridge;Driver->ActiveExecutionModeValue.Store(uint8(ESimulationExecutionMode::IAmSpeedThread));
    Driver->bPublishCompletedWorldInput=true;
    if(!TestTrue(TEXT("real generation minted while stopped"),Driver->PrepareControlledInputRun()))return false;
    auto Data=MakeShared<Speed::Analytic::FAnalyticWorldData>();
    if(!TestTrue(TEXT("empty analytic world admitted"),Data->FinalizeAndValidate()))return false;
    Bridge->AnalyticWorldData=Data;Bridge->SimulationWorld.SetStaticCollisionWorld(MakeUnique<FAnalyticStaticCollisionWorld>(*Data,1,1,1));
    auto* Car=World->SpawnActor<ANativeRequestedPreparationCar>();
    auto* Component=Car?Cast<UNativeRequestedPreparationProbe>(Car->GetVehicleMovement()):nullptr;
    if(!TestNotNull(TEXT("real derived native component"),Component))return false;
    Chaos::FSimpleWheelConfig Wheel;Chaos::FSimpleSuspensionConfig Suspension;
    Component->SetOwner(Car);Component->DisableAsyncPhysicsTickForTesting();
    ON_SCOPE_EXIT
    {
        Driver->JoinOwnedSimulationForInputTeardown();Component->ClearWheelSimulationPointers();Component->SkySimulation=nullptr;
        Component->VehicleSimulationPT.Reset();Bridge->UnregisterSpeedComponent(Component);Bridge->ApplyPendingOps();
    };
    Component->CanonicalCameraYawAction=-1;Component->CanonicalCameraPitchAction=-1;
    Suspension.SetSuspensionMaxRaise(10);Suspension.SetSuspensionMaxDrop(25);
    auto Vehicle=MakeUnique<Chaos::FSimpleWheeledVehicle>();
    for(int32 I=0;I<4;++I){Vehicle->Wheels.Emplace(&Wheel);Vehicle->Suspension.Emplace(&Suspension);Component->Wheels.Add(NewObject<UChaosVehicleWheel>(Component));}
    Component->VehicleSimulationPT=MakeUnique<USpeedSimulation>();Component->SkySimulation=static_cast<USpeedSimulation*>(Component->VehicleSimulationPT.Get());
    Component->SkySimulation->Init(Vehicle);Component->BindWheelSimulationPointers();Component->SetEngineFPS(300);
    Component->SetCannotMoveLocal();TestFalse(TEXT("initial fixture movement gate closed"),Component->CanMove());SKinematic K;K.Location=FVector(0,0,10000);Component->SetKinematicState(K);
    FString Reason;if(!TestTrue(TEXT("real wheel storage ready"),Component->ValidateSimulationBindings(Reason)))return false;
    Bridge->ApplyPendingOps();if(!TestTrue(TEXT("world genuinely ready"),Driver->EnsureSimulationWorldReady()))return false;
    Driver->InitializeCanonicalFrame(0);
    FInputActionContractDescription Description;Description.Revision={1};Description.Actions=FInputActionContract::BaseActions();
    Description.Physical={{Speed::Input::Throttle,EPhysicalDestination::Throttle},{Speed::Input::Brake,EPhysicalDestination::Brake},{Speed::Input::Steering,EPhysicalDestination::Steering}};
    const auto Contract=FInputActionContract::Create(Description);if(!TestTrue(TEXT("actual contract"),bool(Contract)))return false;
    FOffer Offer;Offer.GameCodec=0x43545801;Offer.GameCodecVersion=1;Offer.Match=501;Offer.MatchEpoch=502;
    Offer.Actor=200;Offer.Incarnation=Offer.Component=1;Offer.OfferSequence=1;Offer.StreamEpoch=19;
    Offer.LogicalRateNumerator=300;Offer.LogicalRateDenominator=1;Offer.OriginalProducerKind=1;Offer.OriginalProducer=18;
    Offer.OriginalStreamEpoch=19;Offer.OriginalDeviceGeneration=20;Offer.WireContract={0x43,0x54,0x58,1}; // opaque comparison-fixture ID, not a fingerprint hash
    Offer.Profiles={{EProfileRoute::ClientInputsAtServer,{'C','T','X',1}}};
    FHostLimits Limits;Limits.Lifetime=std::chrono::seconds(15);auto Host=FHostAgreementHost::Create(501,502,Limits);
    std::unique_ptr<FTcpAgreementSession> Client;bool NetworkJoined=false;
    const auto CloseNetwork=[&]()
    {
        if(NetworkJoined)return true;if(Client)Client->RequestStop();if(Host)Host->RequestStop();
        const double End=FPlatformTime::Seconds()+3;
        do
        {
            const bool A=!Client||Client->TryJoin();const bool B=!Host||Host->TryJoin();
            if(A&&B){NetworkJoined=true;return true;}FPlatformProcess::SleepNoStats(.001f);
        }while(FPlatformTime::Seconds()<End);
        return false;
    };
    ON_SCOPE_EXIT{CloseNetwork();};
    if(!TestTrue(TEXT("real private TCP host started"),Host&&Host->Start()))return false;
    if(!TestTrue(TEXT("real host identity configured before native worker"),Driver->ConfigureRequestedNetworkPreparationV2(Host->Identity())))return false;
    FRosterRequest Reserve;Reserve.Offer=Offer;Reserve.Session.Handshake=std::chrono::seconds(3);Reserve.Session.Lifetime=std::chrono::seconds(12);
    if(!TestTrue(TEXT("real host reserve accepted"),Host->SubmitReserve(1,Reserve)==EHostAdmission::Queued))return false;
    std::optional<FHostReceipt> H;double End=FPlatformTime::Seconds()+4;
    do{H=Host->ReadReceipt(1);if(H&&H->Permit)break;FPlatformProcess::SleepNoStats(.001f);}while(FPlatformTime::Seconds()<End);
    if(!TestTrue(TEXT("genuine join permit published by real host"),H&&H->Permit))return false;
    Client=FHostAgreementHost::Join(*H->Permit,Reserve.Session);
    if(!TestTrue(TEXT("real enrolled TCP client started"),Client&&Client->Start()))return false;
    End=FPlatformTime::Seconds()+4;
    do{H=Host->ReadReceipt(1);if(H&&H->Agreement)break;FPlatformProcess::SleepNoStats(.001f);}while(FPlatformTime::Seconds()<End);
    if(!TestTrue(TEXT("real accept observed at server"),H&&H->Agreement&&H->Agreement->Received()
        &&H->Agreement->Received()->Stage()==EObservedStage::AcceptReceivedAtServer))return false;
    FNetworkInputWorkerConfiguration Config;Config.Consumer=ENetworkInputConsumer::DedicatedServer;
    Config.Binding={501,502,200,1,Offer.WireContract};Config.AuthenticatedPeer=H->Agreement->Received()->Connection()->Instance();
    Config.Original.Producer=18;Config.Original.StreamEpoch=19;Config.Original.DeviceGeneration=20;Config.AllowLocalFallback=true;
    Config.PredictionFrames=1;Config.Prediction={FNetworkInputPredictionPolicy::Hold};
    auto Service=FNetworkRequestedInputService::Create(Config,Contract);if(!TestTrue(TEXT("real inert requested service"),bool(Service)))return false;
    FRequestedNetworkPreparationRequestV2 Request;Request.Service=Service;Request.Agreement=H->Agreement;
    auto& Cmd=Request.Command;Cmd.WorkerGeneration=Driver->GetInputWorkerGeneration();Cmd.RegistryVersion=0;
    auto& D=Cmd.Binding;D.Id=1000;D.Epoch=1;D.Contract=Contract;D.Kind=EProducerContract::ExactRemote;
    const auto Local=Bridge->GetSimulationStableId(*Component);D.Actors={{Local,Local}};
    FRequestedNetworkSessionDescriptor N;N.Version=2;N.Service=5000;N.SourceGeneration=100;N.WorldGeneration=Cmd.WorkerGeneration;
    N.Session=501;N.Epoch=502;N.Actor=200;N.Incarnation=N.Component=1;N.LocalComponent=N.LocalComponentGeneration=Local;
    N.LocalBodyGeneration=Component->GetCanonicalRequestedBodyGeneration();N.WireContract=Offer.WireContract;N.DigitalPolicy=ERequestedDigitalPolicy::HeldStateAndCancelInactive;
    FRequestedNetworkAgreementLocator L;L.HostGeneration=Host->Identity()->Generation();L.Slot=H->Agreement->SlotId();L.Connection=Config.AuthenticatedPeer;
    L.OfferSequence=1;L.StreamEpoch=19;L.GameCodec=Offer.GameCodec;L.GameCodecVersion=1;L.ProfileRoute=1;L.LeaseMilliseconds=5000;N.Agreement=L;D.Network=N;
    if(!TestTrue(TEXT("complete inert V2 locator"),N.IsValid()))return false;
    // Genuine external worker boundary, as used by controlled physics tests.
    // It is intentionally NOT installed as Driver->SimulationWorker. Empty
    // native-V2 services must not terminate unrelated input execution.
    const auto OnExternalBoundary=[&](TFunction<void()> Check)
    {
        std::atomic<bool> Entered{false},Returned{false},Threw{false};
        FSimulationWorker External([](){return ESimulationWorkerResult::Complete;},
            [](FSimulationWorkerWaitContext&){},[&](bool)
            {
                Entered.store(true,std::memory_order_release);
                try{Check();Returned.store(true,std::memory_order_release);}catch(...){Threw.store(true,std::memory_order_release);}
                return ESimulationBoundaryResult::Complete;
            });
        if(!External.Start(false))return false;
        const double Deadline=FPlatformTime::Seconds()+2;
        while(External.IsRunning()&&FPlatformTime::Seconds()<Deadline)FPlatformProcess::SleepNoStats(.001f);
        const bool Finished=!External.IsRunning();External.StopAndJoin();
        return Finished&&Entered.load(std::memory_order_acquire)
            &&Returned.load(std::memory_order_acquire)&&!Threw.load(std::memory_order_acquire);
    };
    if(!TestTrue(TEXT("empty V2 queues tolerate genuine external boundary without a driver worker"),OnExternalBoundary([&]
    {
        Driver->ServiceRequestedNetworkBindsV2AtBoundary(0);
        Driver->ServiceRequestedNetworkPreparationV2AtBoundary();
    })))return false;
    auto PendingRequest=Request;PendingRequest.Command.Id=Driver->AllocateInputCommandId();
    const auto PendingId=PendingRequest.Command.Id;
    if(!TestTrue(TEXT("real agreement comparison request enqueued"),Driver->SubmitRequestedNetworkPreparationV2(std::move(PendingRequest))==ERequestedPreparationAdmissionV2::Enqueued))return false;
    std::atomic<bool> PendingRefused{false};
    if(!TestTrue(TEXT("external worker observes pending guard without inventing native V2 ownership"),OnExternalBoundary([&]
    {
        try{Driver->ServiceRequestedNetworkPreparationV2AtBoundary();}
        catch(const std::runtime_error&){PendingRefused.store(true,std::memory_order_release);}
    })))return false;
    const auto BeforeOwned=Driver->ReadRequestedNetworkPreparationReceiptV2(Cmd.WorkerGeneration,PendingId);
    TestTrue(TEXT("nonempty V2 queue still refuses absent native owner before capture"),PendingRefused.load(std::memory_order_acquire)
        &&BeforeOwned&&BeforeOwned->Status==ERequestedPreparationStatusV2::Pending
        &&BeforeOwned->EnqueueOrdinal>0&&BeforeOwned->BoundaryCutoff==0&&!BeforeOwned->Context);
    std::atomic<uint64> ObserveRequestId{0}, ObservedRequestId{0};
    std::atomic<uint32> RegistryObservationThread{0};
    std::atomic<bool> NoRequestedBinding{false};
    Driver->SimulationWorker=MakeUnique<FSimulationWorker>([](){return ESimulationWorkerResult::Idle;},[](FSimulationWorkerWaitContext&){},
        [&](bool Paused)
        {
            const auto Result=Driver->ServiceInputSessionBoundary(Paused);
            const auto Id=ObserveRequestId.load(std::memory_order_acquire);
            const auto Receipt=Id&&Driver->RequestedPreparationV2?Driver->RequestedPreparationV2->Read(Cmd.WorkerGeneration,Id):std::nullopt;
            if(Receipt&&Receipt->Status!=ERequestedPreparationStatusV2::Pending&&Driver->InputSessionRegistry)
            {
                // ReadRegistry is owner-only. Publish only a test observation,
                // after the corresponding actual receipt has completed.
                const auto V=Driver->InputSessionRegistry->ReadRegistry();
                NoRequestedBinding.store(V&&!V->Terminal&&V->Version==0&&V->Bindings.empty(),std::memory_order_relaxed);
                RegistryObservationThread.store(FPlatformTLS::GetCurrentThreadId(),std::memory_order_relaxed);
                ObservedRequestId.store(Id,std::memory_order_release);
            }
            return Result;
        },[&]{Driver->CloseInputSessionsOnWorker();});
    auto* Worker=Driver->SimulationWorker.Get();ON_SCOPE_EXIT{Driver->JoinOwnedSimulationForInputTeardown();};
    if(!TestTrue(TEXT("native owner starts paused"),Worker->Start(true)))return false;
    const auto SubmitAndWait=[&](FRequestedNetworkPreparationRequestV2 R)
    {
        R.Command.Id=Driver->AllocateInputCommandId();const auto Id=R.Command.Id;
        ObserveRequestId.store(Id,std::memory_order_release);
        if(Driver->SubmitRequestedNetworkPreparationV2(std::move(R))!=ERequestedPreparationAdmissionV2::Enqueued)return std::optional<FRequestedPreparationReceiptV2>{};
        const double Deadline=FPlatformTime::Seconds()+3;
        do
        {
            auto Receipt=Driver->ReadRequestedNetworkPreparationReceiptV2(Cmd.WorkerGeneration,Id);
            if(Receipt&&Receipt->Status!=ERequestedPreparationStatusV2::Pending&&ObservedRequestId.load(std::memory_order_acquire)==Id)return Receipt;
            FPlatformProcess::SleepNoStats(.001f);
        }while(Worker->IsRunning()&&FPlatformTime::Seconds()<Deadline);
        return std::optional<FRequestedPreparationReceiptV2>{};
    };
    // The same queued request now completes on the actual installed owner.
    const double PendingDeadline=FPlatformTime::Seconds()+3;
    std::optional<FRequestedPreparationReceiptV2> AfterOwned;
    do
    {
        AfterOwned=Driver->ReadRequestedNetworkPreparationReceiptV2(Cmd.WorkerGeneration,PendingId);
        if(AfterOwned&&AfterOwned->Status!=ERequestedPreparationStatusV2::Pending)break;
        FPlatformProcess::SleepNoStats(.001f);
    }while(Worker->IsRunning()&&FPlatformTime::Seconds()<PendingDeadline);
    TestTrue(TEXT("same pending ordinal completes only on actual installed owner"),BeforeOwned&&AfterOwned
        &&AfterOwned->Id==BeforeOwned->Id&&AfterOwned->EnqueueOrdinal==BeforeOwned->EnqueueOrdinal
        &&AfterOwned->BoundaryCutoff>=AfterOwned->EnqueueOrdinal
        &&AfterOwned->Status==ERequestedPreparationStatusV2::ComparedOnly&&AfterOwned->Context);
    auto Compared=SubmitAndWait(Request);
    if(!TestTrue(TEXT("actual native context compared only"),Compared&&Compared->Status==ERequestedPreparationStatusV2::ComparedOnly
        &&Compared->Reason==ERequestedPreparationReasonV2::None&&Compared->Context))return false;
    TestTrue(TEXT("actual native callback independent of GT and socket thread"),Component->ComparisonCalls==2
        &&Component->ComparisonThread!=FPlatformTLS::GetCurrentThreadId()&&Compared->Context->PreparationThread()!=std::this_thread::get_id()
        &&Compared->Context->PreparationThread()!=H->Agreement->Received()->ReceivingThread());
    TestTrue(TEXT("game support remains explicitly absent"),!Component->SupportsCanonicalRequestedInputsV2(*Compared->Context));
    Cmd.Id=Driver->AllocateInputCommandId();
    TestTrue(TEXT("compared receipt cannot enable runtime V2 admission"),Driver->SubmitRequestedNetworkInputSessionV2(Request)==ECommandAdmission::Rejected);
    Component->RejectComparison=true;auto Rejected=SubmitAndWait(Request);Component->RejectComparison=false;
    TestTrue(TEXT("game rejection is explicit and historical context grants nothing"),Rejected&&Rejected->Status==ERequestedPreparationStatusV2::Rejected
        &&Rejected->Reason==ERequestedPreparationReasonV2::GameRejected&&Rejected->Context);
    Component->ThrowComparison=true;auto Thrown=SubmitAndWait(Request);Component->ThrowComparison=false;
    TestTrue(TEXT("game exception stays scoped to comparison"),Thrown&&Thrown->Status==ERequestedPreparationStatusV2::Rejected
        &&Thrown->Reason==ERequestedPreparationReasonV2::GameException&&Thrown->Context);
    auto Stale=Request;++Stale.Command.Binding.Network->LocalBodyGeneration;const uint32 BeforeStale=Component->ComparisonCalls.load();
    auto StaleResult=SubmitAndWait(std::move(Stale));
    TestTrue(TEXT("stale actual body mapping rejected before game callback"),StaleResult&&StaleResult->Status==ERequestedPreparationStatusV2::Rejected
        &&StaleResult->Reason==ERequestedPreparationReasonV2::Mapping&&!StaleResult->Context&&Component->ComparisonCalls==BeforeStale);
    TestTrue(TEXT("real TCP owners stopped and joined"),CloseNetwork());auto Closed=SubmitAndWait(Request);
    TestTrue(TEXT("retained real agreement after host closure refused"),Closed&&Closed->Status==ERequestedPreparationStatusV2::Rejected
        &&Closed->Reason==ERequestedPreparationReasonV2::Closed&&!Closed->Context);
    TestTrue(TEXT("preparation never binds a requested source, observed on owner after the closed receipt"),Closed
        &&ObservedRequestId.load(std::memory_order_acquire)==Closed->Id&&NoRequestedBinding.load(std::memory_order_relaxed)
        &&RegistryObservationThread==Component->ComparisonThread&&RegistryObservationThread!=FPlatformTLS::GetCurrentThreadId());
    TestTrue(TEXT("actual native owner joined"),Driver->JoinOwnedSimulationForInputTeardown());
    TestTrue(TEXT("no frame or completed physical pair from comparison"),Driver->CanonicalNumFrame==0&&!Driver->ReadCompletedWorldAndInputFrame());
    TestFalse(TEXT("qualification barrier remains hard closed"),NativeRequestedV2RuntimeQualified());
    TestFalse(TEXT("CanMove remains closed"),Component->CanMove());
    AddInfo(TEXT("[EmptyV2BoundaryScope] actual external worker: empty bind/preparation no-op; nonempty real preparation refused without consumption; same ordinal completed on actual installed owner; runtime V2 remains hard false"));
    AddInfo(TEXT("[RequestedPreparationNativeScope] genuine host/client accept, real native owner/mapping/context and game comparison; reject/exception/stale-body/closed-host paths; all network/native owners joined; ComparedOnly is not V2 admission, F0F1, CanMove, restore or physics replay; opaque fixture profile is not a complete game policy"));
    return true;
}
#endif
