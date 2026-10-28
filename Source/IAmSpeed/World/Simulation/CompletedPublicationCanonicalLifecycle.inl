// New independently named integration fixture, included only in the existing
// friend test method. Reuses its actual controller/raw acquisition/registry/
// wheeled component/analytical-world/canonical-worker lifecycle unchanged.
// Numeric raw reports are authored; hardware/contact/gameplay/match coverage is
// not supplied. This source is NOT_COMPILED and NOT_RUN.
// Included only inside the existing friend fixture's RunTest. No new friend,
// runtime setter, fake registry ACK or private state injection is introduced.
using namespace Speed::Input::V2;
if (!TestTrue(TEXT("V2 device fixture is GT"),IsInGameThread())) return false;
const auto Options=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(false)
    .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false).SetTransactional(false);
auto* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Options);
if (!TestNotNull(TEXT("V2 world"),World)) return false;
GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
ON_SCOPE_EXIT { World->DestroyWorld(false); GEngine->DestroyWorldContext(World); };
World->SetGameInstance(NewObject<UGameInstance>(GEngine));
World->GetWorldSettings()->DefaultGameMode=ASpeedGameMode::StaticClass();
if (!TestTrue(TEXT("V2 authoritative mode"),World->SetGameMode(FURL()))) return false;
auto* Mode=Cast<ASpeedGameMode>(World->GetAuthGameMode());
auto* Controller=World->SpawnActor<ASpeedController>();
auto* Car=World->SpawnActor<ASpeedCar>();
auto* Driver=World->SpawnActor<ARealTimeSimulation>();
auto* Bridge=World->GetSubsystem<USpeedWorldSubsystem>();
if (!TestNotNull(TEXT("V2 mode"),Mode)||!TestNotNull(TEXT("V2 controller"),Controller)
    ||!TestNotNull(TEXT("V2 car"),Car)||!TestNotNull(TEXT("V2 driver"),Driver)||!TestNotNull(TEXT("V2 bridge"),Bridge)) return false;
if (!Controller->PlayerState) Controller->SetPlayerState(World->SpawnActor<APlayerState>());
if (!TestNotNull(TEXT("V2 actual pause owner"),Controller->PlayerState.Get())) return false;
auto* Component=Cast<USpeedWheeledComponent>(Car->GetVehicleMovement());
if (!TestNotNull(TEXT("V2 native wheeled component"),Component)) return false;
Driver->SpeedWorldSubsystem=Bridge; Mode->SpeedSimActor=Driver;
auto Data=MakeShared<Speed::Analytic::FAnalyticWorldData>();
if (!TestTrue(TEXT("V2 immutable empty Analytic world"),Data->FinalizeAndValidate())) return false;
Bridge->AnalyticWorldData=Data;
Bridge->SimulationWorld.SetStaticCollisionWorld(MakeUnique<Speed::FAnalyticStaticCollisionWorld>(*Data));
Component->SetOwner(Car); Component->SetAsyncPhysicsTickEnabled(false); Bridge->ApplyPendingOps();
ON_SCOPE_EXIT { Bridge->UnregisterSpeedComponent(Component); Bridge->ApplyPendingOps(); };
if (!TestEqual(TEXT("V2 production wheel count"),Component->WheelSubBodies.Num(),4)) return false;
Chaos::FSimpleWheelConfig WheelConfig; Chaos::FSimpleSuspensionConfig SuspensionConfig;
SuspensionConfig.SetSuspensionMaxRaise(10); SuspensionConfig.SetSuspensionMaxDrop(25);
auto Vehicle=MakeUnique<Chaos::FSimpleWheeledVehicle>();
for (int32 I=0;I<4;++I)
{
    Vehicle->Wheels.Emplace(&WheelConfig); Vehicle->Suspension.Emplace(&SuspensionConfig);
    Component->Wheels.Add(NewObject<UChaosVehicleWheel>(Component));
}
Component->VehicleSimulationPT=MakeUnique<USpeedSimulation>();
Component->SkySimulation=static_cast<USpeedSimulation*>(Component->VehicleSimulationPT.Get());
Component->SkySimulation->Init(Vehicle);
AddExpectedMessagePlain(TEXT("[WheelSuspensionBinding] subbodies="),ELogVerbosity::Warning,EAutomationExpectedMessageFlags::Contains,1);
Component->BindWheelSimulationPointers();
ON_SCOPE_EXIT { Component->ClearWheelSimulationPointers(); Component->SkySimulation=nullptr; Component->VehicleSimulationPT.Reset(); };
FString BindingReason;
if (!TestTrue(TEXT("V2 real mechanics admitted"),Component->ValidateSimulationBindings(BindingReason))
    ||!TestTrue(TEXT("V2 actual driver admission"),Driver->EnsureSimulationWorldReady())) return false;
Driver->TransitionExecutionMode(ESimulationExecutionMode::IAmSpeedThread);
// Test-only friend access selects the SAME opt-in field normally fixed by
// BeginPlay's command-line parse. BeginPlay/bootstrap coverage is separate.
if (!TestFalse(TEXT("common publication remains default off"),Driver->bPublishCompletedWorldInput)) return false;
Driver->bPublishCompletedWorldInput=true;
Driver->InitializeCanonicalFrame(0);

std::atomic<uint32> Steps{0},Budget{0},BoundaryCalls{0},PhysicalThread{0};
std::atomic<bool> Failed{false}; std::atomic<uint64> Watch{0};
std::mutex ReceiptsGate; std::vector<FBoundaryReceipt> Receipts;
struct FRow2443 { uint64 N=0,Epoch=0,Session=0,ComponentNext=0; int32 Requested=0,Applied=0,Physical=0; bool Reset=false; };
TArray<FRow2443> Rows; Rows.Reserve(16);
struct FCompletedPairWitness
{
    std::shared_ptr<const Speed::FCompletedSimulationPublication> Pair;
    std::shared_ptr<const FRegistryFrame> Endpoint;
    FSimulationSnapshot WorldAtCompletion;
};
TArray<FCompletedPairWitness> CompletedPairs; CompletedPairs.Reserve(16);
auto ObserveReceipt=[&]
{
    const auto Id=Watch.load(); const auto Receipt=Id?Driver->ReadInputSessionReceipt(Id):std::nullopt;
    if (!Receipt) return;
    std::lock_guard<std::mutex> Lock(ReceiptsGate);
    if (Receipts.empty() || Receipts.back().Id!=Receipt->Id || Receipts.back().Status!=Receipt->Status)
        Receipts.push_back(*Receipt);
};
Driver->SimulationWorker=MakeUnique<FSimulationWorker>([&]
{
    PhysicalThread.store(FPlatformTLS::GetCurrentThreadId());
    if (Steps.load()>=Budget.load()) return ESimulationWorkerResult::Idle;
    if (Rows.Num()>=16 || !Driver->RunCanonicalFrames(1)) { Failed=true; return ESimulationWorkerResult::Failed; }
    const auto Out=Driver->ReadCompletedInputFrame();
    if (!Out || Out->Inputs.size()!=1 || !Out->Inputs[0].Snapshot) { Failed=true; return ESimulationWorkerResult::Failed; }
    // This is the actual ASpeedSimulation::RunCanonicalFrames/StepCanonicalFrame
    // path on the physical owner, not a handcrafted registry completion record.
    const auto Pair=Driver->ReadCompletedWorldAndInputFrame();
    FSimulationSnapshot WorldAtCompletion;
    if (!Pair || Pair->ConsumedInputs()!=Out || Pair->RegistryAtFrame()!=Out->Registry
        || Pair->World().NumFrame!=Out->Frame || Pair->World().Payload.IsEmpty()
        || !Pair->World().PublicationSerial || !Driver->ReadLatestSimulationSnapshot(WorldAtCompletion)
        || Pair->World().NumFrame!=WorldAtCompletion.NumFrame
        || Pair->World().StateHash!=WorldAtCompletion.StateHash
        || Pair->World().InputJournalHash!=WorldAtCompletion.InputJournalHash
        || Pair->World().PublicationSerial!=WorldAtCompletion.PublicationSerial
        || Pair->World().Payload!=WorldAtCompletion.Payload
        || (CompletedPairs.Num() && Pair->World().PublicationSerial!=CompletedPairs.Last().Pair->World().PublicationSerial+1))
    { Failed=true; return ESimulationWorkerResult::Failed; }
    CompletedPairs.Add(FCompletedPairWitness{Pair,Out,MoveTemp(WorldAtCompletion)});
    const auto& Input=Out->Inputs[0]; const auto& Snapshot=*Input.Snapshot;
    if (Snapshot.Input.GetData().ConsumptionFrame!=Out->Frame) { Failed=true; return ESimulationWorkerResult::Failed; }
    Rows.Add({Out->Frame,Input.Epoch,Input.Session,uint64(Component->NumFrame()),
        Snapshot.Requested[Throttle],Snapshot.Applied[Throttle],Component->GetWheeledPhysicalInputForWitness().Throttle,
        Snapshot.Input.GetData().Reset});
    Steps.fetch_add(1); return ESimulationWorkerResult::Advanced;
},[](FSimulationWorkerWaitContext& Wait){Wait.WaitUntil(FPlatformTime::Seconds()+.001);},
[&](bool Paused)
{
    const auto Result=Driver->ServiceInputSessionBoundary(Paused);
    // Before the worker acknowledges this request to SetPause/UnPossess, copy
    // its genuine receipt. GT receipt acknowledgement cannot be synthesized here.
    ObserveReceipt(); BoundaryCalls.fetch_add(1); return Result;
},[&]{Driver->CloseInputSessionsOnWorker();});
TArray<std::shared_ptr<FInputHostSession>> Sessions;
TArray<std::shared_ptr<FControllerRawAcquisition2443>> Sources;
ON_SCOPE_EXIT
{
    if (Controller->InputSessionV2) Controller->UnPossess();
    Driver->StopOwnedWorker();
    for (const auto& S:Sessions) if (S->Acquisition) S->Acquisition->Stop();
};
if (!TestTrue(TEXT("V2 worker starts paused with actual boundary service"),Driver->SimulationWorker->Start(true))) return false;
auto Await=[&](auto&& Predicate)
{
    const double Deadline=FPlatformTime::Seconds()+3.;
    while (!Predicate()) { if (Failed.load() || FPlatformTime::Seconds()>=Deadline) return false; FPlatformProcess::SleepNoStats(.001f); }
    return true;
};
auto Captured=[&](uint64 Id,EBoundaryStatus Status)
{
    std::lock_guard<std::mutex> Lock(ReceiptsGate);
    return std::any_of(Receipts.begin(),Receipts.end(),[&](const auto& R){return R.Id==Id && R.Status==Status && R.WorkerGeneration==Driver->GetInputWorkerGeneration();});
};
auto StableWaiting=[&](uint64 Id,uint32 Before)
{
    const auto Calls=BoundaryCalls.load();
    return Await([&]{return BoundaryCalls.load()>=Calls+4;})
        && Steps.load()==Before && Driver->ReadInputSessionReceipt(Id)
        && Driver->ReadInputSessionReceipt(Id)->Status==EBoundaryStatus::WaitingForBaseline;
};
auto Service=[&]{return Controller->ServiceInputSessionV2() && !Controller->bInputLifecycleFault;};
auto RunOne=[&](uint64 Epoch,int32 Expected,bool Reset)
{
    const auto Before=Steps.load(); Budget.store(Before+1); Driver->ResumeOwnedSimulation();
    if (!Await([&]{return Steps.load()==Before+1;})) return false;
    if (Driver->TryPauseOwnedSimulation()!=ESimulationQuiescence::BoundaryAcknowledged) return false;
    // The real acknowledged pause must close BOTH current access and the stored
    // common endpoint, while readers holding a prior immutable pair keep it.
    if (Driver->ReadCompletedWorldAndInputFrame()
        || Driver->PublishedCompletedWorldInput.load(std::memory_order_seq_cst)
        || CompletedPairs.Num()!=int32(Before+1)) return false;
    const auto& Kept=CompletedPairs[int32(Before)];
    if (Kept.Pair->ConsumedInputs()!=Kept.Endpoint
        || Kept.Pair->RegistryAtFrame()!=Kept.Endpoint->Registry
        || Kept.Pair->World().NumFrame!=Kept.WorldAtCompletion.NumFrame
        || Kept.Pair->World().PublicationSerial!=Kept.WorldAtCompletion.PublicationSerial
        || Kept.Pair->World().Payload!=Kept.WorldAtCompletion.Payload) return false;
    const auto& R=Rows[int32(Before)];
    UE_LOG(LogTemp,Display,TEXT("[DeviceV2Lifecycle2443Frame] N=%llu Epoch=%llu Session=%llu ComponentNext=%llu Requested=%d Applied=%d Physical=%d Reset=%u"),R.N,R.Epoch,R.Session,R.ComponentNext,R.Requested,R.Applied,R.Physical,unsigned(R.Reset));
    return R.N==Before && R.ComponentNext==Before+1 && R.Epoch==Epoch && R.Applied==Expected
        && R.Requested==Expected && R.Physical==Expected && R.Reset==Reset;
};
for (uint32 Replacement=0;Replacement<2;++Replacement)
{
    const uint64 Epoch=2443001+Replacement;
    FInputActionContractDescription Description; Description.Revision={1}; Description.Actions=FInputActionContract::BaseActions();
    Description.Physical={{0,EPhysicalDestination::Throttle},{1,EPhysicalDestination::Brake},{2,EPhysicalDestination::Steering}};
    Description.Mapping={{{ERawControlKind::KeyboardUsage,4},Throttle,1}};
    FSessionDescriptor Binding; Binding.Id=2443; Binding.Controller=2443; Binding.Epoch=Epoch;
    Binding.Producer=24430+Replacement; Binding.Journal=24440+Replacement; Binding.Kind=EProducerContract::Device;
    Binding.First=Steps.load(); Binding.Contract=FInputActionContract::Create(Description);
    auto Session=FInputHostSession::CreateDescriptor(Binding);
    if (!TestTrue(TEXT("V2 real descriptor session"),bool(Session))) return false;
    Session->Journal=std::make_shared<FRawAcquisitionJournal>(Epoch,Binding.First);
    auto Source=std::make_shared<FControllerRawAcquisition2443>(Session->Journal);
    Session->Acquisition=std::make_unique<FInputAcquisitionWorker>(Source);
    Session->AcquisitionStartupTimeout=std::chrono::milliseconds(2000);
    Sessions.Add(Session); Sources.Add(Source);
    if (!TestTrue(TEXT("V2 real acquisition starts"),Session->Acquisition->Start(std::chrono::microseconds(1000)))
        ||!TestTrue(TEXT("V2 initial report from acquisition owner"),Source->Submit(false,true))
        ||!TestTrue(TEXT("V2 genuine startup publication"),Session->Acquisition->WaitForFirstPublication(std::chrono::milliseconds(1000)))) return false;
    const auto OldFence=Session->Journal->BeginAcquisition();
    if (!TestTrue(TEXT("V2 public configure before possession"),Controller->ConfigureInputSessionV2(Session))) return false;
    const auto BindId=Driver->NextInputCommandId; Watch.store(BindId); Controller->Possess(Car);
    if (!TestTrue(TEXT("V2 true Bind applied"),Await([&]{return Captured(BindId,EBoundaryStatus::Applied);}))
        ||!TestTrue(TEXT("V2 controller handles actual Bind"),Service())
        ||!TestTrue(TEXT("V2 registry bound by receipt"),Session->RegistryBound)) return false;
    const auto ResumeId=Session->PendingCommand; Watch.store(ResumeId);
    if (!TestTrue(TEXT("V2 initial Resume really waits"),ResumeId && Await([&]{return Captured(ResumeId,EBoundaryStatus::WaitingForBaseline);}))
        ||!TestTrue(TEXT("V2 old ticket control reading installs"),Source->Submit(true,true,OldFence))
        ||!TestTrue(TEXT("V2 stale fence cannot supply physical resume baseline"),!Session->Journal->IsResumeReady() && StableWaiting(ResumeId,Steps.load()))
        ||!TestTrue(TEXT("V2 post-fence neutral actual report"),Source->Submit(false,true))
        ||!TestTrue(TEXT("V2 initial Resume truly applied"),Await([&]{return Captured(ResumeId,EBoundaryStatus::Applied);}))
        ||!TestTrue(TEXT("V2 actual Resume handled"),Service())
        ||!TestTrue(TEXT("V2 initial neutral physical frame"),RunOne(Epoch,0,true))
        ||!TestTrue(TEXT("V2 actual held reading"),Source->Submit(true,false))
        ||!TestTrue(TEXT("V2 actual held physical frame"),RunOne(Epoch,255,false))) return false;
    for (uint32 Cycle=0;Cycle<3;++Cycle)
    {
        const auto Prior=Session->Journal->BeginAcquisition(); const auto Before=Steps.load();
        const auto PauseId=Driver->NextInputCommandId; Watch.store(PauseId);
        const auto QueueWorldPause=[&]
        {
            if (Driver->TrySuspendOwnedBoundaryService()!=ESimulationQuiescence::BoundaryAcknowledged) return false;
            ON_SCOPE_EXIT { Driver->ResumeOwnedBoundaryService(); };
            return Controller->SetPause(true); // Real command waits behind the real structural fence.
        };
        if (!TestTrue(TEXT("V2 actual controller world pause"),QueueWorldPause())
            ||!TestTrue(TEXT("V2 real PauseApplied receipt observed"),Await([&]{return Captured(PauseId,EBoundaryStatus::Applied);}))
            ||!TestTrue(TEXT("V2 real pause state service"),Service())
            ||!TestTrue(TEXT("V2 paused neutral owner"),Controller->IsPaused() && Session->RegistryPaused && Driver->IsOwnedSimulationPaused())) return false;
        const auto Resume=Driver->NextInputCommandId; Watch.store(Resume);
        if (!TestTrue(TEXT("V2 actual world resume requested"),Controller->SetPause(false))
            ||!TestTrue(TEXT("V2 Resume receipt actually waits"),Await([&]{return Captured(Resume,EBoundaryStatus::WaitingForBaseline);}))
            ||!TestTrue(TEXT("V2 no frame during missing baseline"),StableWaiting(Resume,Before))
            ||!TestTrue(TEXT("V2 stale ticket retained for control lane only"),Source->Submit(true,true,Prior))
            ||!TestTrue(TEXT("V2 stale ticket cannot wake physics"),!Session->Journal->IsResumeReady() && StableWaiting(Resume,Before))
            ||!TestTrue(TEXT("V2 genuinely fresh neutral report"),Source->Submit(false,true))
            ||!TestTrue(TEXT("V2 ResumeApplied from physical owner"),Await([&]{return Captured(Resume,EBoundaryStatus::Applied);}))
            ||!TestTrue(TEXT("V2 resumed controller consumes actual receipt"),Service())
            ||!TestTrue(TEXT("V2 first resumed frame neutral/reset"),RunOne(Epoch,0,true))
            ||!TestTrue(TEXT("V2 new held report"),Source->Submit(true,false))
            ||!TestTrue(TEXT("V2 resumed held frame"),RunOne(Epoch,255,false))) return false;
    }
    if (Replacement)
    {
        FBoundaryCommandDescriptor Stale; Stale.Id=Driver->AllocateInputCommandId(); Stale.WorkerGeneration=Driver->GetInputWorkerGeneration();
        Stale.RegistryVersion=Driver->ReadInputRegistryView()->Version; Stale.Operation=EBoundaryOperation::Resume;
        Stale.Session=Session->Descriptor->Id; Stale.Epoch=Sessions[0]->Descriptor->Epoch; Stale.ResumeGeneration=99;
        const auto BeforeView=Driver->ReadInputRegistryView(); const auto Before=Steps.load(); Watch.store(Stale.Id);
        if (!TestTrue(TEXT("V2 stale epoch descriptor submitted without mutation"),Driver->SubmitInputSessionCommand(Stale)==ECommandAdmission::Enqueued)
            ||!TestTrue(TEXT("V2 owner rejects true stale epoch"),Await([&]{return Captured(Stale.Id,EBoundaryStatus::Rejected);}))
            ||!TestTrue(TEXT("V2 stale epoch preserves active view/frame"),Driver->ReadInputRegistryView()->Version==BeforeView->Version && Steps.load()==Before)
            ||!TestTrue(TEXT("V2 explicit stale receipt acknowledged"),Driver->AcknowledgeInputSessionReceipt(Stale.Id))) return false;
    }
    const auto DetachId=Driver->NextInputCommandId; Watch.store(DetachId);
    if (!TestTrue(TEXT("V2 controller queues real Detach"),Controller->QueueRegistryInputCommand(EBoundaryOperation::Detach))
        ||!TestTrue(TEXT("V2 real Detach applied before release"),Await([&]{return Captured(DetachId,EBoundaryStatus::Applied);}))) return false;
    Controller->UnPossess(); // Consumes and acknowledges the genuine completed Detach.
    if (!TestTrue(TEXT("V2 real Detach applied before owners released"),Captured(DetachId,EBoundaryStatus::Applied))
        ||!TestTrue(TEXT("V2 controller released session"),!Controller->InputSessionV2 && !Controller->GetPawn())
        ||!TestTrue(TEXT("V2 acquisition joined and journal closed"),Session->Closed && Session->Acquisition->HasFinished() && Source->IsClosed())
        ||!TestTrue(TEXT("V2 retained closed session cannot be configured"),!Controller->ConfigureInputSessionV2(Session))) return false;
    FRawAcquisitionBatch Late; Late.Count=1;
    if (!TestFalse(TEXT("V2 old epoch journal cannot publish after detach"),Session->Journal->Publish(OldFence,Late))) return false;
    if (!TestTrue(TEXT("V2 actual acquisition owner distinct"),Source->Thread.load()!=0 && Source->Thread.load()!=GGameThreadId
        && Source->Thread.load()!=PhysicalThread.load())) return false;
}
if (!TestTrue(TEXT("V2 actual teardown joins owner"),Driver->JoinOwnedSimulationForInputTeardown())
    ||!TestTrue(TEXT("V2 actual owner closed at join"),Driver->InputOwnersRetiredAfterJoin(Driver->GetInputWorkerGeneration()))
    ||!TestEqual(TEXT("V2 sixteen real canonical component frames"),Steps.load(),uint32(16))
    ||!TestFalse(TEXT("V2 worker did not fail"),Failed.load())
    ||!TestTrue(TEXT("V2 physical owner distinct from GT"),PhysicalThread.load()!=0 && PhysicalThread.load()!=GGameThreadId)) return false;
if (!TestEqual(TEXT("sixteen actual common canonical publications"),CompletedPairs.Num(),16)
    || !TestFalse(TEXT("owner retirement inhibits current common access"),bool(Driver->ReadCompletedWorldAndInputFrame()))
    || !TestFalse(TEXT("owner retirement cleared stored common endpoint"),bool(Driver->PublishedCompletedWorldInput.load(std::memory_order_seq_cst)))) return false;
for (int32 I=0;I<CompletedPairs.Num();++I)
{
    const auto& W=CompletedPairs[I];
    if (!TestTrue(TEXT("retained common endpoint remains the exact completed input object"),W.Pair->ConsumedInputs()==W.Endpoint)
        || !TestTrue(TEXT("retained registry remains the exact captured registry"),W.Pair->RegistryAtFrame()==W.Endpoint->Registry)
        || !TestEqual(TEXT("retained world is at the same real historical canonical frame"),W.Pair->World().NumFrame,uint64(I))
        || !TestEqual(TEXT("retained input is at the same real historical canonical frame"),uint64(W.Endpoint->Frame),uint64(I))
        || !TestTrue(TEXT("retained legacy physical bytes unchanged after subsequent frames/lifecycle"),W.Pair->World().Payload==W.WorldAtCompletion.Payload)
        || !TestEqual(TEXT("retained state hash unchanged"),W.Pair->World().StateHash,W.WorldAtCompletion.StateHash)) return false;
}
UE_LOG(LogTemp,Display,TEXT("[NetcodeCompletedCanonicalLifecycle] ActualCanonicalFrames=16 ExactInputEndpoint=1 ExactRegistry=1 WorldBytesMatched=1 PauseCurrentInvalidation=1 RetainedAfterRetirement=1 Hardware=NONE Contacts=EMPTY Fullstate=NOT_QUALIFIED Rollback=NOT_RUN Client1v1=NOT_RUN"));
for (const auto& Receipt:Receipts)
    UE_LOG(LogTemp,Display,TEXT("[DeviceV2Lifecycle2443Receipt] Id=%llu WorkerGeneration=%llu RegistryVersion=%llu ResumeGeneration=%llu Status=%u Outcomes=%llu"),uint64(Receipt.Id),uint64(Receipt.WorkerGeneration),uint64(Receipt.RegistryVersion),uint64(Receipt.ResumeGeneration),unsigned(Receipt.Status),uint64(Receipt.Sessions.size()));
UE_LOG(LogTemp,Display,TEXT("[DeviceV2Lifecycle2443] Sessions=2 PauseResumeCycles=6 CanonicalFrames=16 Hardware=NONE Focus=NOT_RUN WorldReset=NOT_RUN SkyLeagueGame=NOT_RUN"));
return true;
