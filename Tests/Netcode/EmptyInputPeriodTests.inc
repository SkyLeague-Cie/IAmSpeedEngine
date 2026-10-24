static auto EmptyPeriodReserve(V::FInputSessionRegistry& R, I::FFrameNumber N)
{
    auto C=R.CaptureCompletedBoundary(N);CHECK(C);
    auto T=R.ReserveCompletedBoundary(C);CHECK(T);
    return T;
}
static void EmptyPeriodActualHistoryAndReservation()
{
    FEmptyFixture F;
    auto A=F.Complete(10);auto B=F.Complete(11);auto C=F.Complete(12);
    auto T=EmptyPeriodReserve(F.Registry,12);
    auto E=F.Registry.CaptureReservedEmptyInputPeriod(*T,10,12);CHECK(E);
    CHECK(E->FirstFrame()==10 && E->LastFrame()==12 && E->HistoricalFrames().size()==3);
    CHECK(E->HistoricalFrames()[0]==A && E->HistoricalFrames()[1]==B && E->HistoricalFrames()[2]==C);
    CHECK(E->ObservedInputBoundary()->Completed()==C && E->ObservedInputBoundary()->CurrentBoundary()->Bindings.empty());
    Allocations=0;Budget=0;
    const bool Valid=F.Registry.ValidateReservedEmptyInputPeriod(*T,*E);
    Budget=-1;CHECK(Valid && Allocations==0);
    CHECK(!F.Registry.CaptureReservedEmptyInputPeriod(*T,12,11));
    CHECK(!F.Registry.CaptureReservedEmptyInputPeriod(*T,10,13));
    CHECK(!F.Registry.CaptureReservedEmptyInputPeriod(*T,0,UINT64_MAX));
    CHECK(!F.Registry.CaptureReservedEmptyInputPeriod(*T,UINT64_MAX,UINT64_MAX));
    CHECK(!F.Registry.CaptureReservedEmptyInputPeriod(*T,0,4096));
    V::FInputSessionRegistry::FCompletedBoundaryReservation Default;
    CHECK(!F.Registry.CaptureReservedEmptyInputPeriod(Default,10,12) && !F.Registry.ValidateReservedEmptyInputPeriod(Default,*E));
    auto Foreign=std::async(std::launch::async,[&]{return !F.Registry.CaptureReservedEmptyInputPeriod(*T,10,12)
        && !F.Registry.ValidateReservedEmptyInputPeriod(*T,*E);});CHECK(Foreign.get());
    {I::FPresentationInputScope Scope;CHECK(!F.Registry.CaptureReservedEmptyInputPeriod(*T,10,12) && !F.Registry.ValidateReservedEmptyInputPeriod(*T,*E));}
    FEmptyFixture Other;Other.Complete(10);auto OtherT=EmptyPeriodReserve(Other.Registry,10);
    CHECK(!Other.Registry.ValidateReservedEmptyInputPeriod(*OtherT,*E));
    Other.Registry.ReleaseCompletedBoundaryReservation(*OtherT);
    const auto Boundary=E->ObservedInputBoundary();
    F.Registry.ReleaseCompletedBoundaryReservation(*T);
    CHECK(!F.Registry.ValidateReservedEmptyInputPeriod(*T,*E));
    auto Renewed=F.Registry.ReserveCompletedBoundary(Boundary);CHECK(Renewed);
    CHECK(!F.Registry.ValidateReservedEmptyInputPeriod(*Renewed,*E)); // Same data, DIFFERENT reservation nonce.
    auto NewE=F.Registry.CaptureReservedEmptyInputPeriod(*Renewed,10,12);CHECK(NewE);
    CHECK(F.Registry.ValidateReservedEmptyInputPeriod(*Renewed,*NewE));
    F.Registry.ReleaseCompletedBoundaryReservation(*Renewed);
    CHECK(E->HistoricalFrames()[0]==A && E->HistoricalFrames()[2]==C); // Historical data remains readable.
}
static void EmptyPeriodRejectsMissingAndVersionTransition()
{
    FEmptyFixture F;F.Complete(10);F.Complete(12);
    auto T=EmptyPeriodReserve(F.Registry,12);
    CHECK(!F.Registry.CaptureReservedEmptyInputPeriod(*T,10,12)); // Real hole at11, never backfilled.
    CHECK(F.Registry.CaptureReservedEmptyInputPeriod(*T,10,10));
    F.Registry.ReleaseCompletedBoundaryReservation(*T);
    V::FBoundaryCommandDescriptor Pause;Pause.Id=1;Pause.WorkerGeneration=23;Pause.RegistryVersion=0;Pause.Operation=V::EBoundaryOperation::PauseAll;
    CHECK(F.Commands->Submit(Pause)==V::ECommandAdmission::Enqueued && F.Registry.ServiceBoundary(13));
    CHECK(F.Registry.ReadRegistry()->Version==0); // Pause does not create a topology version.
    auto N=F.Complete(13);CHECK(N->Registry->Version==0);
    auto Same=EmptyPeriodReserve(F.Registry,13);
    CHECK(F.Registry.CaptureReservedEmptyInputPeriod(*Same,12,13));
    F.Registry.ReleaseCompletedBoundaryReservation(*Same);
    auto Bind=EmptyFutureBind(0);Bind.Id=2;
    CHECK(F.Commands->Submit(Bind)==V::ECommandAdmission::Enqueued && F.Registry.ServiceBoundary(14));
    CHECK(F.Commands->Read(Bind.Id)->Status==V::EBoundaryStatus::Applied && F.Registry.ReadRegistry()->Version==1);
    V::FBoundaryCommandDescriptor Detach;Detach.Id=3;Detach.WorkerGeneration=23;Detach.RegistryVersion=1;
    Detach.Operation=V::EBoundaryOperation::Detach;Detach.Session=1;Detach.Epoch=7;
    CHECK(F.Commands->Submit(Detach)==V::ECommandAdmission::Enqueued && F.Registry.ServiceBoundary(14));
    CHECK(F.Registry.ReadRegistry()->Version==2 && F.Registry.ReadRegistry()->Bindings.empty());
    F.Complete(14);
    auto Current=EmptyPeriodReserve(F.Registry,14);
    CHECK(!F.Registry.CaptureReservedEmptyInputPeriod(*Current,13,14)); // Actual transition is deliberately unsupported.
    auto Older=F.Registry.CaptureReservedEmptyInputPeriod(*Current,12,13);CHECK(Older);
    CHECK(Older->HistoricalFrames()[0]->Registry->Version==0 && Older->ObservedInputBoundary()->CurrentBoundary()->Version==2);
    CHECK(F.Registry.CaptureReservedEmptyInputPeriod(*Current,14,14));
    F.Registry.ReleaseCompletedBoundaryReservation(*Current);
}
static void EmptyPeriodCurrentEmptyCannotHidePriorNonempty()
{
    FFixture F(1);const auto Nonempty=F.Registry.ReadHistorical(0);
    const auto Paused=F.Registry.CaptureCompletedBoundary(0);CHECK(Paused);
    auto Held=F.Registry.ReserveCompletedBoundary(Paused);CHECK(Held);
    CHECK(!F.Registry.CaptureReservedEmptyInputPeriod(*Held,0,0)); // Paused session is not absent.
    F.Registry.ReleaseCompletedBoundaryReservation(*Held);
    auto D=F.Command(V::EBoundaryOperation::Detach);D.Session=1;D.Epoch=7;
    CHECK(F.Commands->Submit(D)==V::ECommandAdmission::Enqueued && F.Registry.ServiceBoundary(1));
    auto Detached=EmptyPeriodReserve(F.Registry,0);
    CHECK(!F.Registry.CaptureReservedEmptyInputPeriod(*Detached,0,0));
    CHECK(!F.Registry.CaptureReservedEmptyInputPeriod(*Detached,1,1)); // No completed step1 exists yet.
    F.Registry.ReleaseCompletedBoundaryReservation(*Detached);
    CHECK(V::BeginEmptyCanonicalInputFrameReceipt(F.Registry,1) && V::CompleteCanonicalInputFrameReceipt(F.Registry,1));
    CHECK(V::BeginEmptyCanonicalInputFrameReceipt(F.Registry,2) && V::CompleteCanonicalInputFrameReceipt(F.Registry,2));
    auto T=EmptyPeriodReserve(F.Registry,2);
    CHECK(!F.Registry.CaptureReservedEmptyInputPeriod(*T,0,2));
    auto E=F.Registry.CaptureReservedEmptyInputPeriod(*T,1,2);CHECK(E && E->HistoricalFrames().size()==2);
    CHECK(F.Registry.ReadHistorical(0)==Nonempty && Nonempty->Inputs.size()==1 && Nonempty->Registry->Bindings.size()==1);
    F.Registry.ReleaseCompletedBoundaryReservation(*T);
}
static void EmptyPeriodRetainedAfterRingRecyclingAndDestruction()
{
    std::shared_ptr<const V::FInputSessionRegistry::FEmptyInputPeriodEvidence> Retained;
    {
        V::FRegistryLimits Limits;Limits.Frames=4;
        auto Commands=std::make_shared<V::FInputSessionCommands>(Limits);V::FInputSessionRegistry R(Commands,23);
        const auto Step=[&](I::FFrameNumber N){CHECK(V::BeginEmptyCanonicalInputFrameReceipt(R,N));CHECK(V::CompleteCanonicalInputFrameReceipt(R,N));};
        Step(10);Step(11);auto T=EmptyPeriodReserve(R,11);
        Retained=R.CaptureReservedEmptyInputPeriod(*T,10,11);CHECK(Retained);
        R.ReleaseCompletedBoundaryReservation(*T);
        Step(12);Step(13);Step(14);Step(15);
        CHECK(!R.ReadHistorical(10) && !R.ReadHistorical(11));
        CHECK(Retained->HistoricalFrames()[0]->Frame==10 && Retained->HistoricalFrames()[1]->Frame==11);
        auto New=EmptyPeriodReserve(R,15);
        CHECK(!R.CaptureReservedEmptyInputPeriod(*New,10,11) && !R.ValidateReservedEmptyInputPeriod(*New,*Retained));
        CHECK(!R.CaptureReservedEmptyInputPeriod(*New,11,15)); // Inclusive cardinal5 exceeds ring4.
        R.ReleaseCompletedBoundaryReservation(*New);
    }
    CHECK(Retained->HistoricalFrames()[0]->Frame==10 && Retained->HistoricalFrames()[1]->Inputs.empty());
    // No call to a destroyed registry: only independently retained immutable data.
}
static V::FInputSessionRegistry* PeriodHookRegistry=nullptr;
static V::FInputSessionRegistry::FCompletedBoundaryReservation* PeriodHookToken=nullptr;
static std::shared_ptr<const V::FInputSessionRegistry::FCompletedBoundaryCapture> PeriodHookBoundary;
static bool PeriodHookRenew=false,PeriodHookDone=false,PeriodHookWorked=false;
static void ReenterEmptyPeriodAllocation()
{
    if(PeriodHookDone)return;PeriodHookDone=true;
    PeriodHookWorked=PeriodHookRegistry->AbortCompletedBoundaryReservation(*PeriodHookToken);
    if(PeriodHookWorked && PeriodHookRenew)
    {
        auto T=PeriodHookRegistry->ReserveCompletedBoundary(PeriodHookBoundary);PeriodHookWorked=bool(T);
        if(T)*PeriodHookToken=std::move(*T);
    }
}
static void EmptyPeriodAllocationAndReentrantRenewal()
{
    FEmptyFixture F;F.Complete(10);F.Complete(11);F.Complete(12);
    auto T=EmptyPeriodReserve(F.Registry,12);const auto C=F.Registry.ReadReservedCompletedBoundary(*T);
    unsigned Failures=0;bool Success=false;
    for(int N=0;N<16;++N)
    {
        Allocations=0;Budget=N;auto E=F.Registry.CaptureReservedEmptyInputPeriod(*T,10,12);Budget=-1;
        CHECK(F.Registry.ReadReservedCompletedBoundary(*T)==C && F.Registry.ReadLatest()==C->Completed());
        if(E){CHECK(F.Registry.ValidateReservedEmptyInputPeriod(*T,*E));Success=true;break;}
        ++Failures;
    }
    CHECK(Success && Failures==3);
    for(bool Renew:{false,true})
    {
        if(!F.Registry.ReadReservedCompletedBoundary(*T)){T=F.Registry.ReserveCompletedBoundary(C);CHECK(T);}
        PeriodHookRegistry=&F.Registry;PeriodHookToken=&*T;PeriodHookBoundary=C;PeriodHookRenew=Renew;PeriodHookDone=PeriodHookWorked=false;
        AllocationHook=ReenterEmptyPeriodAllocation;
        auto E=F.Registry.CaptureReservedEmptyInputPeriod(*T,10,12);
        AllocationHook=nullptr;PeriodHookRegistry=nullptr;PeriodHookToken=nullptr;PeriodHookBoundary.reset();
        CHECK(PeriodHookWorked && !E && !F.Registry.IsTerminal());
        CHECK(F.Registry.ReadHistorical(10)->Frame==10 && F.Registry.ReadLatest()==C->Completed());
    }
    CHECK(F.Registry.ReadReservedCompletedBoundary(*T)==C);
    auto Final=F.Registry.CaptureReservedEmptyInputPeriod(*T,10,12);CHECK(Final);
    F.Registry.ReleaseCompletedBoundaryReservation(*T);
    std::cout << "EMPTY_PERIOD_ALLOCATIONS " << Failures << "; reentrant abort and same-boundary token renewal refused\n";
}
static void TestActualEmptyInputPeriodEvidence()
{
    EmptyPeriodActualHistoryAndReservation();EmptyPeriodRejectsMissingAndVersionTransition();
    EmptyPeriodCurrentEmptyCannotHidePriorNonempty();EmptyPeriodRetainedAfterRingRecyclingAndDestruction();EmptyPeriodAllocationAndReentrantRenewal();
}
