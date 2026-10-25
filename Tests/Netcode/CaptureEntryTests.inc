static std::shared_ptr<V::FInputSessionCommands> CaptureHookCommands;
static const V::FBoundaryCommandDescriptor* CaptureHookCommand=nullptr;
static bool CaptureHookCalled=false;
static V::ECommandAdmission CaptureHookAdmission=V::ECommandAdmission::Rejected;
static int CaptureHookLockError=0;
static bool CaptureHookStop=false;
static bool CaptureHookVerbose=true;
static int CaptureHookAt=0;
static void SubmitCommandInsideCaptureAllocation()
{
    if(CaptureHookCalled)return;
    if(CaptureHookAt-- > 0)return;
    CaptureHookCalled=true;
    if(CaptureHookVerbose)std::cerr << "CAPTURE_ALLOCATION_CALLBACK_ENTER\n" << std::flush;
    try
    {
        if(CaptureHookStop)CaptureHookCommands->RequestStop();
        else CaptureHookAdmission=CaptureHookCommands->Submit(*CaptureHookCommand);
    }
    catch(const std::system_error& E) { CaptureHookLockError=E.code().value(); }
}
static int CaptureEntryCommandSubmissionProbe(bool Stop=false,int At=0,bool Verbose=true)
{
    FFixture F(1);
    const auto C=F.Registry.CaptureCompletedBoundary(0);CHECK(C);
    auto Command=F.Command(V::EBoundaryOperation::PauseAll);
    CaptureHookCommands=F.Commands;CaptureHookCommand=&Command;CaptureHookCalled=false;CaptureHookLockError=0;CaptureHookStop=Stop;
    CaptureHookAt=At;CaptureHookVerbose=Verbose;
    AllocationHook=SubmitCommandInsideCaptureAllocation;
    const auto Captured=F.Registry.CaptureCompletedBoundary(0);
    AllocationHook=nullptr;CaptureHookCommands.reset();CaptureHookCommand=nullptr;
    if(Verbose)std::cerr << "CAPTURE_CALLBACK_EXIT lock_error=" << CaptureHookLockError << " captured=" << bool(Captured) << '\n';
    if(Stop)
    {
        CHECK(CaptureHookCalled && !CaptureHookLockError && !Captured);
        CHECK(F.Registry.ServiceBoundary() && F.Registry.IsTerminal());
        CHECK(F.Registry.ReadHistorical(0)==C->Completed());
        return 0;
    }
    CHECK(CaptureHookCalled && CaptureHookAdmission==V::ECommandAdmission::Enqueued);
    CHECK(!Captured); // The newly accepted command invalidates the optimistic capture.
    CHECK(F.Commands->Read(Command.Id)->Status==V::EBoundaryStatus::Pending);
    CHECK(!F.Registry.ValidateCompletedBoundaryCapture(*C));
    CHECK(F.Registry.ServiceBoundary(1));
    CHECK(F.Commands->Read(Command.Id)->Status==V::EBoundaryStatus::Applied);
    CHECK(F.Registry.ReadHistorical(0)==C->Completed() && F.Registry.PollCount(1)==1);
    CHECK(!F.Registry.IsTerminal());
    return 0;
}
static V::FInputSessionRegistry* CaptureWorkerRegistry=nullptr;
static std::shared_ptr<const V::FInputSessionRegistry::FCompletedBoundaryCapture> CaptureWorkerBoundary;
static bool CaptureWorkerCalled=false,CaptureWorkerRefused=false;
static void ReenterCaptureWorkerOperations()
{
    if(CaptureWorkerCalled || CaptureHookAt-- > 0)return;
    CaptureWorkerCalled=true;
    auto& R=*CaptureWorkerRegistry;
    CaptureWorkerRefused=!R.CaptureCompletedBoundary(0)
        && !R.ValidateCompletedBoundaryCapture(*CaptureWorkerBoundary)
        && !R.ReserveCompletedBoundary(CaptureWorkerBoundary)
        && !R.ServiceBoundary(1) && !R.CloseWorkerLedger() && !R.PrepareFrame(1);
    R.AbortFrame(); // Must not retire this live, unmodified source during capture.
    CaptureWorkerRefused &= !R.IsTerminal();
}
static void TestCaptureEntryReentrancy()
{
    FFixture F(1);
    const auto Original=F.Registry.CaptureCompletedBoundary(0);CHECK(Original);
    Budget=10000;Allocations=0;const auto Measured=F.Registry.CaptureCompletedBoundary(0);
    const auto Count=Allocations;Budget=-1;CHECK(Measured && Count>0 && Count<64);
    for(int At=0;At<Count;++At)
    {
        CaptureEntryCommandSubmissionProbe(false,At,false);
        CaptureEntryCommandSubmissionProbe(true,At,false);
        CaptureWorkerRegistry=&F.Registry;CaptureWorkerBoundary=Original;CaptureHookAt=At;
        CaptureWorkerCalled=CaptureWorkerRefused=false;AllocationHook=ReenterCaptureWorkerOperations;
        const auto C=F.Registry.CaptureCompletedBoundary(0);
        AllocationHook=nullptr;CaptureWorkerRegistry=nullptr;CaptureWorkerBoundary.reset();
        CHECK(CaptureWorkerCalled && CaptureWorkerRefused && C);
        CHECK(F.Registry.ValidateCompletedBoundaryCapture(*C));
        CHECK(F.Registry.ReadHistorical(0)==Original->Completed() && F.Registry.PollCount(1)==1);
    }
    CHECK(F.Registry.ValidateCompletedBoundaryCapture(*Original));
    std::cout << "CAPTURE_ENTRY_ALLOCATIONS " << Count
        << "; command/stop preserved and recursive worker mutations denied at every allocation\n";
}
