static void BoundaryReservationPinsAndBlocks()
{
    FFixture F(1); ReadyRegistryForReplay(F);
    const auto C=F.Registry.CaptureCompletedBoundary(2);CHECK(C);
    const auto Replay=ReplayRequest(F);const auto Resume=F.Request(2,3);
    auto AI=std::make_shared<V::FAIInputCommands>(Contract());
    V::FInputSessionRegistry::FJournalService Service{91,{},AI};
    const auto Append=MakeFrame(100,4);const auto Poll=F.Registry.PollCount(1);
    auto T=F.Registry.ReserveCompletedBoundary(C);CHECK(T);
    CHECK(F.Registry.ReadReservedCompletedBoundary(*T)==C && F.Registry.ReadRegistry()==C->CurrentBoundary());
    CHECK(!F.Registry.ValidateCompletedBoundaryCapture(*C) && !F.Registry.CaptureCompletedBoundary(2));
    CHECK(!F.Registry.ReserveCompletedBoundary(C) && !F.Registry.CloseWorkerLedger());
    CHECK(!F.Registry.ServiceBoundary(3) && !F.Registry.PrepareFrame(3));
    CHECK(!F.Registry.InstallAll() && !F.Registry.BeginAll() && !F.Registry.CloseInitialBoundary());
    CHECK(!F.Registry.ValidateComplete() && !F.Registry.CompleteAll());
    CHECK(!F.Registry.RegisterJournalAtBoundary(Service) && !F.Registry.AppendExactScenarioFrame(1,7,200,Append));
    CHECK(!F.Registry.PrepareGroupResumeAtBoundary(Resume) && !F.Registry.PrepareGroupRetainedReplayStart(Replay));
    CHECK(!F.Registry.PrepareGroupReplayFrame(1) && !F.Registry.EndGroupRetainedReplay());
    CHECK(!F.Registry.BeginRetainedReplay(1,2,Replay.Actors) && !F.Registry.NextRetainedReplay() && !F.Registry.EndRetainedReplay());
    F.Registry.AbortFrame();CHECK(!F.Registry.IsTerminal());
    CHECK(F.Registry.ReadHistorical(2)==C->Completed() && F.Registry.PollCount(1)==Poll);
    FFixture Other(1);CHECK(!Other.Registry.ReadReservedCompletedBoundary(*T) && !Other.Registry.AbortCompletedBoundaryReservation(*T));
    auto Foreign=std::async(std::launch::async,[&] {
        return !F.Registry.ReserveCompletedBoundary(C) && !F.Registry.ReadReservedCompletedBoundary(*T)
            && !F.Registry.AbortCompletedBoundaryReservation(*T) && !F.Registry.ServiceBoundary();
    });CHECK(Foreign.get());
    { I::FPresentationInputScope Scope;CHECK(!F.Registry.ReadReservedCompletedBoundary(*T) && !F.Registry.AbortCompletedBoundaryReservation(*T)); }
    std::shared_ptr<const V::FInputSessionRegistry::FCompletedBoundaryCapture> CommonInputMetadata;
    Allocations=0;Budget=0;
    const auto Read=F.Registry.ReadReservedCompletedBoundary(*T);
    const auto View=F.Registry.ReadRegistry();
    CommonInputMetadata=Read; // INPUT-only marker; no native/world publication claimed.
    F.Registry.ReleaseCompletedBoundaryReservation(*T);
    Budget=-1;
    CHECK(Allocations==0 && Read==C && View==C->CurrentBoundary() && CommonInputMetadata==C);
    CHECK(!F.Registry.ReadReservedCompletedBoundary(*T) && !F.Registry.AbortCompletedBoundaryReservation(*T));
    CHECK(F.Registry.ValidateCompletedBoundaryCapture(*C) && F.Registry.PollCount(1)==Poll);
    CHECK(F.Registry.RegisterJournalAtBoundary(Service) && F.Registry.AppendExactScenarioFrame(1,7,200,Append));
    F.Complete(3);CHECK(F.Registry.ReadHistorical(2)==C->Completed() && !F.Registry.ValidateCompletedBoundaryCapture(*C));
}
static void BoundaryReservationCommandFence()
{
    FFixture F(1);const auto C=F.Registry.CaptureCompletedBoundary(0);CHECK(C);
    auto T=F.Registry.ReserveCompletedBoundary(C);CHECK(T);
    std::promise<void> Started;auto Begin=Started.get_future();
    auto Stop=std::async(std::launch::async,[&]{Started.set_value();F.Commands->RequestStop();});
    Begin.wait();CHECK(Stop.wait_for(std::chrono::milliseconds(5))==std::future_status::timeout);
    CHECK(F.Registry.ReadReservedCompletedBoundary(*T)==C && !F.Registry.ServiceBoundary());
    CHECK(F.Registry.AbortCompletedBoundaryReservation(*T));
    CHECK(Stop.wait_for(std::chrono::seconds(1))==std::future_status::ready);Stop.get();
    CHECK(!F.Registry.ValidateCompletedBoundaryCapture(*C) && !F.Registry.ReserveCompletedBoundary(C));
    CHECK(F.Registry.ServiceBoundary() && F.Registry.IsTerminal());
}
static V::FInputSessionRegistry* ReservationHookRegistry=nullptr;
static std::shared_ptr<const V::FInputSessionRegistry::FCompletedBoundaryCapture> ReservationHookCapture;
static int ReservationHookCalls=0;
static bool ReservationHookViolation=false;
static void TryReenterReservationAllocation()
{
    ++ReservationHookCalls;
    auto& R=*ReservationHookRegistry;
    if(R.ServiceBoundary() || R.CloseWorkerLedger() || R.PrepareFrame(1)
        || R.BeginRetainedReplay(0,0,{}) || R.CaptureCompletedBoundary(0)
        || R.ReserveCompletedBoundary(ReservationHookCapture)) ReservationHookViolation=true;
    R.AbortFrame();
    if(R.IsTerminal())ReservationHookViolation=true;
}
static void BoundaryReservationFailuresAndTeardown()
{
    FFixture F(1);const auto C=F.Registry.CaptureCompletedBoundary(0);CHECK(C);
    Budget=10000;Allocations=0;auto Probe=F.Registry.ReserveCompletedBoundary(C);const int Count=Allocations;Budget=-1;
    CHECK(Probe && Count>0 && F.Registry.AbortCompletedBoundaryReservation(*Probe));
    for(int At=0;At<Count;++At)
    {
        Budget=At;Allocations=0;auto Failed=F.Registry.ReserveCompletedBoundary(C);Budget=-1;
        CHECK(!Failed && F.Registry.ValidateCompletedBoundaryCapture(*C) && F.Registry.PollCount(1)==1);
        auto Retry=F.Registry.ReserveCompletedBoundary(C);CHECK(Retry && F.Registry.AbortCompletedBoundaryReservation(*Retry));
    }
    ReservationHookRegistry=&F.Registry;ReservationHookCapture=C;ReservationHookCalls=0;ReservationHookViolation=false;
    AllocationHook=TryReenterReservationAllocation;
    auto Reentrant=F.Registry.ReserveCompletedBoundary(C);
    AllocationHook=nullptr;ReservationHookRegistry=nullptr;ReservationHookCapture.reset();
    CHECK(Reentrant && ReservationHookCalls==Count && !ReservationHookViolation);
    CHECK(F.Registry.ReadReservedCompletedBoundary(*Reentrant)==C && F.Registry.AbortCompletedBoundaryReservation(*Reentrant));
    std::cout<<"BOUNDARY_RESERVATION_ALLOCATIONS "<<Count<<"; each failure/reentrant allocation preserves consumed input\n";
    auto Owned=std::make_unique<FFixture>(1);const auto Retained=Owned->Registry.CaptureCompletedBoundary(0);CHECK(Retained);
    auto Token=Owned->Registry.ReserveCompletedBoundary(Retained);CHECK(Token);
    auto Commands=Owned->Commands;
    std::promise<void> Started;auto Begin=Started.get_future();
    auto Stop=std::async(std::launch::async,[&]{Started.set_value();Commands->RequestStop();});
    Begin.wait();CHECK(Stop.wait_for(std::chrono::milliseconds(5))==std::future_status::timeout);
    CHECK(!Owned->Registry.CloseWorkerLedger());
    CHECK(Owned->Registry.AbortCompletedBoundaryReservation(*Token)); // Explicit release precedes endpoint destruction.
    Owned.reset();
    CHECK(Stop.wait_for(std::chrono::seconds(1))==std::future_status::ready);Stop.get();
    CHECK(Retained->Completed()->Inputs.size()==1 && Retained->CurrentBoundary()->Bindings.size()==1);
    CHECK(!F.Registry.ReadReservedCompletedBoundary(*Token) && !F.Registry.AbortCompletedBoundaryReservation(*Token));
}
static void BoundaryReservationRejectsStale()
{
    FFixture F(1);CHECK(!F.Registry.ReserveCompletedBoundary({}));
    const auto C=F.Registry.CaptureCompletedBoundary(0);CHECK(C);
    const auto Command=F.Command(V::EBoundaryOperation::Detach);
    auto Detach=Command;Detach.Session=1;Detach.Epoch=7;
    CHECK(F.Commands->Submit(Detach)==V::ECommandAdmission::Enqueued);
    CHECK(!F.Registry.ReserveCompletedBoundary(C));CHECK(F.Registry.ServiceBoundary(1));
    CHECK(!F.Registry.ReserveCompletedBoundary(C));
    const auto EmptyCurrent=F.Registry.CaptureCompletedBoundary(0);CHECK(EmptyCurrent);
    auto T=F.Registry.ReserveCompletedBoundary(EmptyCurrent);CHECK(T);
    CHECK(F.Registry.ReadReservedCompletedBoundary(*T)->Completed()->Inputs.size()==1);
    CHECK(F.Registry.ReadRegistry()->Bindings.empty());
    CHECK(F.Registry.AbortCompletedBoundaryReservation(*T));
    CHECK(F.Registry.ValidateCompletedBoundaryCapture(*EmptyCurrent));
    // This is old completed INPUT frame0 plus empty current registry, never an
    // empty-input world-frame1 completion or automatic actor resurrection.
}
static int DestroyWhileBoundaryReservationIsLive()
{
    std::set_terminate([] { std::_Exit(86); });
    auto F=std::make_unique<FFixture>(1);
    const auto C=F->Registry.CaptureCompletedBoundary(0);if(!C)return 87;
    const auto T=F->Registry.ReserveCompletedBoundary(C);if(!T)return 88;
    std::cout<<"LIVE_INPUT_REGISTRY_RESERVATION_DESTRUCTION_PROBE\n"<<std::flush;
    F.reset(); // A live reservation must prohibit destruction of its mutable registry.
    return 0; // Continuing here would let an external tail use a destroyed registry.
}
