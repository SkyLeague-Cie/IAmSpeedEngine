static void CompletedBoundaryViewsSurviveDetach()
{
    FFixture F;const auto Historical=F.Registry.ReadHistorical(0);CHECK(Historical && Historical->Frame==0);
    const auto Capture=F.Registry.CaptureCompletedBoundary(0);CHECK(Capture);
    CHECK(Capture->Completed()==Historical && Capture->FrameFollowingCapturedInput()==1);
    CHECK(Capture->Completed()->Registry->Version==Capture->CurrentBoundary()->Version); // Pause changes phases without changing this version.
    CHECK(Capture->Completed()->Inputs.size()==6 && Capture->CurrentBoundary()->Bindings.size()==6);
    for(const auto& Phase:Capture->CurrentBoundary()->Phases)CHECK(Phase.Phase==V::ESessionPhase::Paused);
    CHECK(F.Registry.ValidateCompletedBoundaryCapture(*Capture));
    const auto InitialPolls=F.Registry.PollCount(2);
    auto Detach=F.Command(V::EBoundaryOperation::Detach);Detach.Session=1;Detach.Epoch=7;
    CHECK(F.Commands->Submit(Detach)==V::ECommandAdmission::Enqueued);
    CHECK(!F.Registry.ValidateCompletedBoundaryCapture(*Capture));CHECK(!F.Registry.CaptureCompletedBoundary(0));
    CHECK(F.Registry.ServiceBoundary(1));CHECK(F.Commands->Read(Detach.Id)->Status==V::EBoundaryStatus::Applied);
    CHECK(!F.Registry.ReadLatest() && F.Registry.ReadHistorical(0)==Historical && !F.Registry.ValidateCompletedBoundaryCapture(*Capture));
    const auto After=F.Registry.CaptureCompletedBoundary(0);CHECK(After && F.Registry.ValidateCompletedBoundaryCapture(*After));
    CHECK(After->Completed()==Historical && After->Completed()->Inputs.size()==6);
    CHECK(After->CurrentBoundary()->Bindings.size()==5 && After->CurrentBoundary()->Phases.size()==5);
    CHECK(After->Completed()->Registry->Version<After->CurrentBoundary()->Version);
    CHECK(After->Completed()->Inputs.front().Session==1 && After->Completed()->Inputs.front().Epoch==7);
    CHECK(After->Completed()->Inputs.front().Actors.size()==1 && After->Completed()->Inputs.front().Actors[0].Id==1001);
    CHECK(F.Registry.PollCount(2)==InitialPolls);
    for(std::uint64_t Id=2;Id<=6;++Id)
    {
        auto Remove=F.Command(V::EBoundaryOperation::Detach);Remove.Session=Id;Remove.Epoch=7;
        CHECK(F.Commands->Submit(Remove)==V::ECommandAdmission::Enqueued);CHECK(F.Registry.ServiceBoundary(1));
        CHECK(F.Commands->Read(Remove.Id)->Status==V::EBoundaryStatus::Applied);
    }
    CHECK(!F.Registry.ValidateCompletedBoundaryCapture(*After));
    const auto Empty=F.Registry.CaptureCompletedBoundary(0);CHECK(Empty && Empty->Completed()==Historical);
    CHECK(Empty->Completed()->Inputs.size()==6 && Empty->CurrentBoundary()->Bindings.empty() && Empty->CurrentBoundary()->Phases.empty());
    CHECK(!F.Registry.BeginRetainedReplay(0,0,{})); // Archived metadata does not restore the unavailable actors.
    CHECK(F.Registry.ValidateCompletedBoundaryCapture(*Empty));
    FFixture Other;CHECK(!Other.Registry.ValidateCompletedBoundaryCapture(*After)); // Same worker/generation, different registry origin.
    auto Foreign=std::async(std::launch::async,[&]{return !F.Registry.CaptureCompletedBoundary(0) && !F.Registry.ValidateCompletedBoundaryCapture(*After);});
    CHECK(Foreign.get());
    CHECK(!F.Registry.CaptureCompletedBoundary(1));
    CHECK(!F.Registry.CaptureCompletedBoundary(std::numeric_limits<I::FFrameNumber>::max()));
}
static void BoundaryCapturePendingGroupsAndHistory()
{
    FFixture F(1);const auto Capture=F.Registry.CaptureCompletedBoundary(0);CHECK(Capture);
    auto Resume=F.Registry.PrepareGroupResumeAtBoundary(F.Request());CHECK(Resume);
    CHECK(!F.Registry.CaptureCompletedBoundary(0) && !F.Registry.ValidateCompletedBoundaryCapture(*Capture));
    CHECK(F.Registry.AbortGroupResume(*Resume));
    Resume=F.Registry.PrepareGroupResumeAtBoundary(F.Request());CHECK(Resume && F.Registry.PrepareGroupResumeFinalization(*Resume));
    CHECK(!F.Registry.CaptureCompletedBoundary(0) && !F.Registry.ValidateCompletedBoundaryCapture(*Capture)); // Retained mutex, no deadlock.
    F.Registry.FinalizePreparedGroupResume(*Resume);F.Registry.PublishPreparedGroupResume(*Resume);F.Registry.ReleasePreparedGroupResume(*Resume);
    CHECK(!F.Registry.ValidateCompletedBoundaryCapture(*Capture));
    const auto Active=F.Registry.CaptureCompletedBoundary(0);CHECK(Active);
    CHECK(F.Registry.PrepareFrame(1));CHECK(!F.Registry.CaptureCompletedBoundary(0));
    CHECK(F.Registry.InstallAll());CHECK(!F.Registry.CaptureCompletedBoundary(0));
    CHECK(F.Registry.BeginAll());CHECK(!F.Registry.CaptureCompletedBoundary(0));
    CHECK(F.Registry.ValidateComplete() && F.Registry.CompleteAll());
    CHECK(!F.Registry.ValidateCompletedBoundaryCapture(*Active));
    const auto New=F.Registry.CaptureCompletedBoundary(1);CHECK(New && New->FrameFollowingCapturedInput()==2);
    CHECK(Active->Completed()->Frame==0 && Active->Completed()->Inputs.front().Snapshot->Input.GetData().ConsumptionFrame==0);
    CHECK(New->Completed()->Inputs.front().Snapshot->Input.GetData().ConsumptionFrame==1);
    const auto ReplayRequest=V::FInputSessionRegistry::FGroupReplayRequest{23,F.Registry.ReadRegistry()->Version,0,1,2,F.Registry.ReadRegistry()->Phases,{{1001,5}}};
    auto Replay=F.Registry.PrepareGroupRetainedReplayStart(ReplayRequest);CHECK(Replay);
    CHECK(!F.Registry.CaptureCompletedBoundary(1) && !F.Registry.ValidateCompletedBoundaryCapture(*New));
    CHECK(F.Registry.AbortGroupReplay(*Replay));
}
static void BoundaryCaptureFailureAndLifetimes()
{
    std::shared_ptr<const V::FInputSessionRegistry::FCompletedBoundaryCapture> Retained;
    {
        FFixture F(1);Retained=F.Registry.CaptureCompletedBoundary(0);CHECK(Retained);
        const auto Frame=F.Registry.ReadLatest();const auto ViewVersion=F.Registry.ReadRegistry()->Version;const auto Poll=F.Registry.PollCount(1);
        Budget=10000;Allocations=0;const auto Probe=F.Registry.CaptureCompletedBoundary(0);const auto Count=Allocations;Budget=-1;CHECK(Probe && Count>0);
        for(int At=0;At<Count;++At)
        {
            Budget=At;Allocations=0;const auto Failed=F.Registry.CaptureCompletedBoundary(0);Budget=-1;
            CHECK(!Failed && F.Registry.ReadLatest()==Frame && F.Registry.ReadRegistry()->Version==ViewVersion && F.Registry.PollCount(1)==Poll);
            CHECK(F.Registry.ValidateCompletedBoundaryCapture(*Retained));
        }
        std::cout<<"BOUNDARY_CAPTURE_ALLOCATIONS "<<Count<<"; source/registry unchanged at each failure\n";
        F.Commands->RequestStop();CHECK(!F.Registry.CaptureCompletedBoundary(0));CHECK(!F.Registry.ValidateCompletedBoundaryCapture(*Retained));
    }
    CHECK(Retained && Retained->Completed()->Inputs.front().Snapshot->Input.GetData().ConsumptionFrame==0);
    CHECK(Retained->CurrentBoundary()->Bindings.size()==1); // Immutable metadata survives registry/producers' safe teardown.
}
static void InitialBoundaryIsNotCompletedConsumption()
{
    auto Commands=std::make_shared<V::FInputSessionCommands>();V::FInputSessionRegistry Registry(Commands,23);
    CHECK(!Registry.CaptureCompletedBoundary(0));
    V::FSessionDescriptor D;D.Id=1;D.Epoch=7;D.Controller=200;D.Producer=100;D.Contract=Contract();
    D.Kind=V::EProducerContract::ExactScenario;D.Actors={{1001,5}};D.Scenario={MakeFrame(100,0),MakeFrame(100,1)};
    V::FBoundaryCommandDescriptor Bind;Bind.Id=1;Bind.WorkerGeneration=23;Bind.RegistryVersion=0;Bind.Binding=D;
    CHECK(Commands->Submit(Bind)==V::ECommandAdmission::Enqueued);CHECK(Registry.ServiceBoundary(0));
    CHECK(Registry.PrepareFrame(0) && Registry.InstallAll() && Registry.CloseInitialBoundary());
    CHECK(Registry.ReadInitial() && !Registry.ReadLatest() && !Registry.CaptureCompletedBoundary(0));
    CHECK(Registry.PrepareFrame(0) && Registry.InstallAll() && Registry.BeginAll() && Registry.ValidateComplete() && Registry.CompleteAll());
    const auto Closed=Registry.CaptureCompletedBoundary(0);CHECK(Closed && Closed->Completed()->Frame==0);
    CHECK(Closed->Completed()->Inputs[0].AlreadyAppliedAtInitialBoundary);
}
