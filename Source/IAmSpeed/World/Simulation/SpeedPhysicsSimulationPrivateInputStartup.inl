// Included in SpeedPhysicsSimulation.cpp after its GT startup-hold registry.
// Diagnostics own only this fixed sidecar, never actor/input/worker state.
void ASpeedPhysicsSimulation::ObservePrivateInputStartup(EPrivateInputStartupObservation Observation,
    uint64 Command, uint64 Session, uint64 Epoch) const
{
    if (!CanRecordPrivateInputStartup() || HasAnyFlags(RF_ClassDefaultObject) || !GetWorld()
        || !GetWorld()->IsGameWorld() || GetWorld()->bIsTearingDown) return;

    // Use the completed snapshot channel. CanonicalNumFrame and other live
    // worker-owned frame counters are deliberately never read on this GT path.
    if (SnapshotBuffer.PublishedSerial()) return; // Avoid copying an already-running world's complete snapshot every tick.
    FSimulationSnapshot Published;
    const bool HasPublished = ReadLatestSimulationSnapshot(Published);
    if (HasPublished && Published.PublicationSerial) return;

    struct FSeen
    {
        TWeakObjectPtr<ASpeedPhysicsSimulation> Driver;
        uint64 Command = 0, Session = 0, Epoch = 0, Generation = 0, Flags = 0, RegistryVersion = 0;
        double EnqueuedAt = 0., FirstTickAt = 0.;
        EPrivateInputStartupObservation Gate = EPrivateInputStartupObservation::BindEnqueued;
        bool Used = false, FirstTick = false, OneSecond = false, HasState = false;
    };
    static FSeen Seen[8]; // GT only; weak identities never retain a traveled world.
    FSeen* Entry = nullptr;
    for (auto& Candidate : Seen)
        if (Candidate.Used && Candidate.Driver.Get() == this) { Entry = &Candidate; break; }
    const bool Bind = Observation == EPrivateInputStartupObservation::BindEnqueued;
    if (!Entry && Bind)
    {
        for (auto& Candidate : Seen)
            if (!Candidate.Used || !Candidate.Driver.IsValid())
            {
                Candidate = FSeen{};
                Candidate.Used = true;
                Candidate.Driver = const_cast<ASpeedPhysicsSimulation*>(this);
                Entry = &Candidate; break;
            }
    }
    if (!Entry || (!Bind && !Entry->Command)) return;
    const double Now = FPlatformTime::Seconds();
    if (Bind)
    {
        if (!Command || (Entry->Command == Command && Entry->Generation == GetInputWorkerGeneration())) return;
        Entry->Command = Command; Entry->Session = Session; Entry->Epoch = Epoch;
        Entry->EnqueuedAt = Now; Entry->FirstTick = Entry->OneSecond = Entry->HasState = false;
        Entry->Gate = EPrivateInputStartupObservation::BindEnqueued;
    }
    const bool Hold = HasReplayStartupHoldOnGameThread(this);
    const bool Tick = Observation == EPrivateInputStartupObservation::Tick;
    if (Tick && Hold) Entry->Gate = EPrivateInputStartupObservation::ReplayHold;
    else if (!Bind && !Tick) Entry->Gate = Observation;

    const bool TickEnabled = IsActorTickEnabled();
    const bool TickRegistered = PrimaryActorTick.IsTickFunctionRegistered();
    const bool WorkerPresent = bool(SimulationWorker);
    const bool WorkerRunning = SimulationWorker && SimulationWorker->IsRunning();
    const bool WorkerPaused = SimulationWorker && SimulationWorker->IsPaused();
    const bool ServiceSuspended = SimulationWorker && SimulationWorker->IsBoundaryServiceSuspendRequested();
    const bool WorkerTerminal = bOwnedWorkerTerminal.Load(), Retired = bInputOwnerRetired.Load();
    const bool PublicationTerminal = bCanonicalPublicationTerminal.Load();
    const bool OwnedPaused = bOwnedSimulationPaused.Load();
    const bool GlobalPaused = Speed::CanonicalFrameDriver::IsOwnedThreadPaused();
    const bool Admission = bInputSessionAdmissionRequested.Load();
    const bool Subsystem = SpeedWorldSubsystem != nullptr;
    const bool Collision = Subsystem && SpeedWorldSubsystem->GetStaticCollisionWorld() != nullptr;
    const auto View = ReadInputRegistryView();
    const auto Receipt = ReadInputSessionReceipt(Entry->Command);
    const int32 ReceiptStatus = Receipt ? int32(Receipt->Status) : -1;
    const uint64 Generation = GetInputWorkerGeneration(), RegistryVersion = View ? View->Version : 0;
    const auto Bit = [](bool Value, uint32 Index) { return Value ? uint64(1) << Index : uint64(0); };
    const uint64 Flags = Bit(TickEnabled,0) | Bit(TickRegistered,1) | Bit(Hold,2) | Bit(WorkerPresent,3)
        | Bit(WorkerRunning,4) | Bit(WorkerPaused,5) | Bit(ServiceSuspended,6) | Bit(WorkerTerminal,7)
        | Bit(Retired,8) | Bit(PublicationTerminal,9) | Bit(OwnedPaused,10) | Bit(GlobalPaused,11)
        | Bit(Admission,12) | Bit(Subsystem,13) | Bit(Collision,14) | Bit(bool(View),15)
        | Bit(View && View->Terminal,16) | (uint64(ReceiptStatus + 1) << 24)
        | (uint64(HostTransitionDelayTicks) << 32) | (uint64(GetActiveExecutionMode()) << 40)
        | (uint64(Entry->Gate) << 48);
    const bool FirstTick = Tick && !Entry->FirstTick;
    if (FirstTick) Entry->FirstTickAt = Now;
    const bool OneSecond = Tick && Entry->FirstTick && !Entry->OneSecond && Now - Entry->FirstTickAt >= 1.;
    const bool Changed = Entry->HasState && (Entry->Flags != Flags || Entry->Generation != Generation
        || Entry->RegistryVersion != RegistryVersion);
    if (Tick) Entry->FirstTick = true;
    if (!Bind && !FirstTick && !OneSecond && !Changed) return;
    if (OneSecond) Entry->OneSecond = true;
    Entry->Flags = Flags; Entry->Generation = Generation;
    Entry->RegistryVersion = RegistryVersion; Entry->HasState = true;
    const auto GateName = [](EPrivateInputStartupObservation Gate) -> const TCHAR*
    {
        switch (Gate)
        {
        case EPrivateInputStartupObservation::ReplayHold: return TEXT("replay_hold");
        case EPrivateInputStartupObservation::WorkerPresent: return TEXT("worker_present");
        case EPrivateInputStartupObservation::WorkerTerminal: return TEXT("worker_terminal");
        case EPrivateInputStartupObservation::WorldNotReady: return TEXT("world_not_ready");
        case EPrivateInputStartupObservation::StartFailed: return TEXT("start_failed");
        case EPrivateInputStartupObservation::Started: return TEXT("started");
        default: return TEXT("not_attempted");
        }
    };
    const TCHAR* Site = Bind ? TEXT("Bind.Enqueued") : FirstTick ? TEXT("Tick.FirstWaiting")
        : OneSecond ? TEXT("Tick.OneSecondWaiting") : Tick ? TEXT("Tick.StateChanged") : TEXT("Start.GateChanged");
    const FString Evidence = FString::Printf(
        TEXT("map=%s driver_uid=%u generation=%llu command=%llu session=%llu epoch=%llu elapsed_ms=%lld tick_enabled=%d tick_registered=%d startup_hold=%d mode=%u worker_present=%d worker_running=%d worker_paused=%d boundary_suspend=%d worker_terminal=%d retired=%d publication_terminal=%d owned_pause=%d global_pause=%d admission_requested=%d subsystem=%d static_collision=%d registry_present=%d registry_terminal=%d registry_version=%llu receipt_present=%d receipt_status=%d receipt_reason=%d receipt_generation=%llu latest_start_gate=%s transition_delay=%u snapshot_present=%d published_serial=%llu published_frame=%lld"),
        *GetWorld()->GetMapName().Left(128), GetUniqueID(), Generation, Entry->Command, Entry->Session, Entry->Epoch,
        static_cast<long long>((Now - Entry->EnqueuedAt) * 1000.), int32(TickEnabled), int32(TickRegistered), int32(Hold),
        uint32(GetActiveExecutionMode()), int32(WorkerPresent), int32(WorkerRunning), WorkerPresent ? int32(WorkerPaused) : -1,
        int32(ServiceSuspended), int32(WorkerTerminal), int32(Retired), int32(PublicationTerminal), int32(OwnedPaused), int32(GlobalPaused),
        int32(Admission), int32(Subsystem), int32(Collision), int32(bool(View)), int32(View && View->Terminal), RegistryVersion,
        int32(bool(Receipt)), ReceiptStatus, Receipt ? int32(Receipt->RejectionReason) : -1,
        Receipt ? Receipt->WorkerGeneration : 0, GateName(Entry->Gate), uint32(HostTransitionDelayTicks), int32(HasPublished),
        HasPublished ? Published.PublicationSerial : 0, HasPublished ? static_cast<long long>(Published.NumFrame) : -1LL);
    RecordPrivateInputStartup(Site, Evidence);
}
