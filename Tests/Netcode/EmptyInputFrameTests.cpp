// Compile the actual registry and the ordinary adapter proposed for the native
// driver. The driver/UE world itself is NOT compiled or simulated by this test.
#define main RebasedRegistryRegressionMain
#include "RegistryReplayTests.cpp"
#undef main
#include "IAmSpeed/Input/CanonicalInputFrameReceipt.h"

struct FEmptyFixture
{
    std::shared_ptr<V::FInputSessionCommands> Commands = std::make_shared<V::FInputSessionCommands>();
    V::FInputSessionRegistry Registry{Commands, 23};
    std::shared_ptr<const V::FRegistryFrame> Complete(I::FFrameNumber N)
    {
        CHECK(V::BeginEmptyCanonicalInputFrameReceipt(Registry, N));
        CHECK(V::ValidateCanonicalInputFrameReceipt(Registry, N));
        CHECK(!V::ValidateCanonicalInputFrameReceipt(Registry, N + 1));
        const auto Result = V::CompleteCanonicalInputFrameReceipt(Registry, N);
        CHECK(Result && Result->Frame == N && Result->Inputs.empty() && Result->Registry->Bindings.empty());
        return Result;
    }
};

static void EmptyRequiresExplicitFullInputLifecycle()
{
    FEmptyFixture F;
    CHECK(!F.Registry.PrepareFrame(10)); // Existing empty admission is unchanged.
    CHECK(!F.Registry.CaptureCompletedBoundary(10));
    CHECK(!F.Registry.ReadLatest() && !F.Registry.ReadInitial() && !F.Registry.ReadHistoricalInitial());
    CHECK(F.Registry.PrepareEmptyFrameReceipt(10));
    CHECK(!F.Registry.ServiceBoundary(11) && !F.Registry.PrepareEmptyFrameReceipt(11));
    CHECK(!F.Registry.ReadInstalled() && !F.Registry.ReadHistorical(10));
    CHECK(!F.Registry.ValidateComplete() && !F.Registry.CompleteAll());
    CHECK(!F.Registry.CloseInitialBoundary());
    CHECK(F.Registry.InstallAll());
    const auto Installed = F.Registry.ReadInstalled();
    CHECK(Installed && Installed->Frame == 10 && Installed->Inputs.empty());
    CHECK(Installed->Registry->WorkerGeneration == 23 && Installed->Registry->Version == 0);
    CHECK(Installed->Registry->Bindings.empty() && Installed->Registry->Phases.empty());
    CHECK(!F.Registry.CloseInitialBoundary()); // Never a fabricated Reset/baseline.
    CHECK(!F.Registry.ReadInitial() && !F.Registry.ReadHistoricalInitial());
    CHECK(!F.Registry.CompleteAll() && F.Registry.BeginAll());
    CHECK(!V::CompleteCanonicalInputFrameReceipt(F.Registry, 11)); // Wrong frame cannot consume it.
    CHECK(V::ValidateCanonicalInputFrameReceipt(F.Registry, 10));
    Allocations = 0; Budget = 0;
    const auto Completed = V::CompleteCanonicalInputFrameReceipt(F.Registry, 10);
    Budget = -1;
    CHECK(Completed == Installed && Allocations == 0);
    CHECK(!V::CompleteCanonicalInputFrameReceipt(F.Registry, 10));
    CHECK(F.Registry.ReadHistorical(10) == Completed && F.Registry.ReadLatest() == Completed);
    CHECK(!F.Registry.ReadInitial() && !F.Registry.ReadHistoricalInitial());
    CHECK(F.Registry.PollCount(1) == 0 && F.Registry.ConstructionCount() == 0 && F.Registry.DestructionCount() == 0);
    const auto Capture = F.Registry.CaptureCompletedBoundary(10);
    CHECK(Capture && Capture->Completed() == Completed && Capture->CurrentBoundary()->Bindings.empty());
    auto Token = F.Registry.ReserveCompletedBoundary(Capture);
    CHECK(Token && !F.Registry.PrepareEmptyFrameReceipt(11));
    CHECK(!M::PrepareRegistryMetadata(F.Registry, *Token)); // IRB1 v1 is NOT silently upgraded.
    F.Registry.ReleaseCompletedBoundaryReservation(*Token);
    CHECK(!F.Registry.PrepareEmptyFrameReceipt(10) && !F.Registry.PrepareEmptyFrameReceipt(9));
    CHECK(!F.Registry.PrepareEmptyFrameReceipt(UINT64_MAX));
    F.Complete(12); // Exact supplied frame; gap11 is not invented or backfilled.
    CHECK(!F.Registry.ReadHistorical(11) && F.Registry.ReadHistorical(10) == Completed);
    CHECK(!F.Registry.ValidateCompletedBoundaryCapture(*Capture));
    CHECK(F.Registry.BeginRetainedReplay(10, 10, {})); // Evidence cursor only.
    CHECK(!F.Registry.PrepareEmptyFrameReceipt(13));
    CHECK(F.Registry.NextRetainedReplay() == Completed && F.Registry.EndRetainedReplay());
    V::FInputSessionRegistry::FGroupReplayRequest Replay{23, 0, 10, 10, 13, {}, {}};
    CHECK(!F.Registry.PrepareGroupRetainedReplayStart(Replay)); // Empty executable replay stays closed.
    CHECK(!F.Registry.IsTerminal());
}

static void DetachThenEmptyDoesNotRelabelPriorConsumption()
{
    FFixture F(2); // Actual nonempty frame0 is complete, both sessions paused.
    const auto Original = F.Registry.ReadHistorical(0);
    CHECK(Original && Original->Inputs.size() == 2);
    CHECK(!F.Registry.PrepareEmptyFrameReceipt(1)); // Paused is not absent.
    for (std::uint64_t Id = 1; Id <= 2; ++Id)
    {
        auto C = F.Command(V::EBoundaryOperation::Detach); C.Session = Id; C.Epoch = 7;
        CHECK(F.Commands->Submit(C) == V::ECommandAdmission::Enqueued);
        CHECK(F.Registry.ServiceBoundary(1) && F.Commands->Read(C.Id)->Status == V::EBoundaryStatus::Applied);
    }
    const auto BeforeEmptyStep = F.Registry.CaptureCompletedBoundary(0);
    CHECK(BeforeEmptyStep && BeforeEmptyStep->Completed() == Original);
    CHECK(BeforeEmptyStep->CurrentBoundary()->Bindings.empty() && Original->Registry->Bindings.size() == 2);
    auto Held = F.Registry.ReserveCompletedBoundary(BeforeEmptyStep);
    CHECK(Held && !F.Registry.PrepareEmptyFrameReceipt(1));
    CHECK(F.Registry.AbortCompletedBoundaryReservation(*Held));
    const auto Version = F.Registry.ReadRegistry()->Version;
    CHECK(V::BeginEmptyCanonicalInputFrameReceipt(F.Registry, 1));
    CHECK(!F.Registry.CaptureCompletedBoundary(0) && !F.Registry.ReadHistorical(1));
    const auto New = V::CompleteCanonicalInputFrameReceipt(F.Registry, 1);
    CHECK(New && New->Frame == 1 && New->Inputs.empty() && New->Registry->Bindings.empty());
    CHECK(New->Registry->Version == Version && F.Registry.ReadRegistry()->Version == Version);
    CHECK(F.Registry.ReadHistorical(0) == Original && Original->Frame == 0 && Original->Inputs.size() == 2);
    CHECK(!F.Registry.ValidateCompletedBoundaryCapture(*BeforeEmptyStep));
    const auto Current = F.Registry.CaptureCompletedBoundary(1);
    CHECK(Current && Current->Completed() == New && Current->CurrentBoundary()->Bindings.empty());
}

static V::FBoundaryCommandDescriptor EmptyFutureBind(std::uint64_t Version)
{
    V::FBoundaryCommandDescriptor C; C.Id = 1; C.WorkerGeneration = 23; C.RegistryVersion = Version;
    auto& D = C.Binding; D.Id = 1; D.Epoch = 7; D.Controller = 200; D.Producer = 100; D.First = 14;
    D.Contract = Contract(); D.Kind = V::EProducerContract::ExactScenario; D.Actors = {{1001,5}};
    D.Scenario = {MakeFrame(100,14,true), MakeFrame(100,15,false)};
    return C;
}

static void PendingEmptyPreservesNextBoundaryBind()
{
    FEmptyFixture F;
    const auto Empty10 = F.Complete(10);
    CHECK(V::BeginEmptyCanonicalInputFrameReceipt(F.Registry, 12));
    auto C = EmptyFutureBind(0);
    CHECK(F.Commands->Submit(C) == V::ECommandAdmission::Enqueued);
    CHECK(!F.Registry.ServiceBoundary(14));
    const auto Empty12 = V::CompleteCanonicalInputFrameReceipt(F.Registry, 12);
    CHECK(Empty12 && Empty12->Registry->Version == 0 && Empty12->Registry->Bindings.empty());
    CHECK(!F.Registry.CaptureCompletedBoundary(12)); // Queued command must be serviced first.
    CHECK(F.Registry.ServiceBoundary(14) && F.Commands->Read(C.Id)->Status == V::EBoundaryStatus::Applied);
    const auto Boundary = F.Registry.CaptureCompletedBoundary(12);
    CHECK(Boundary && Boundary->Completed() == Empty12 && Boundary->CurrentBoundary()->Bindings.size() == 1);
    CHECK(Boundary->CurrentBoundary()->Version == 1 && Empty12->Registry->Version == 0);
    CHECK(!F.Registry.PrepareEmptyFrameReceipt(14));
    CHECK(F.Registry.PrepareFrame(14) && F.Registry.InstallAll() && F.Registry.BeginAll());
    const auto Nonempty = V::CompleteCanonicalInputFrameReceipt(F.Registry, 14);
    CHECK(Nonempty && Nonempty->Inputs.size() == 1 && Nonempty->Frame == 14);
    CHECK(Nonempty->Inputs[0].Epoch == 7 && Nonempty->Inputs[0].Snapshot->Input.GetData().Reset);
    CHECK(F.Registry.PollCount(1) == 1 && F.Registry.ReadHistorical(10) == Empty10 && F.Registry.ReadHistorical(12) == Empty12);
}

static V::FInputSessionRegistry* EmptyHookRegistry = nullptr;
static bool EmptyHookSawRefusals = false;
static void ReenterEmptyPreparation()
{
    auto& R = *EmptyHookRegistry;
    EmptyHookSawRefusals = !R.PrepareEmptyFrameReceipt(21) && !R.PrepareFrame(21)
        && !R.ServiceBoundary(21) && !R.CloseWorkerLedger() && !R.InstallAll() && !R.BeginAll()
        && !R.CompleteAll() && !R.CloseInitialBoundary() && !R.CaptureCompletedBoundary(20);
    R.AbortFrame(); // Must not terminalize the registry during allocator reentrance.
    EmptyHookSawRefusals = EmptyHookSawRefusals && !R.IsTerminal();
}

static void EmptyFailuresNoCompletedReceipt()
{
    FEmptyFixture F;
    const auto Before = F.Complete(10);
    Allocations = 0; Budget = 0;
    const bool Prepared = F.Registry.PrepareEmptyFrameReceipt(20);
    Budget = -1;
    CHECK(!Prepared && Allocations == 1 && !F.Registry.ReadInstalled() && !F.Registry.IsTerminal());
    CHECK(F.Registry.ReadLatest() == Before && !F.Registry.ReadHistorical(20));
    EmptyHookRegistry = &F.Registry; EmptyHookSawRefusals = false; AllocationHook = ReenterEmptyPreparation;
    const bool AfterHook = F.Registry.PrepareEmptyFrameReceipt(20);
    AllocationHook = nullptr; EmptyHookRegistry = nullptr;
    CHECK(AfterHook && EmptyHookSawRefusals && !F.Registry.IsTerminal());
    CHECK(F.Registry.InstallAll() && F.Registry.BeginAll());
    F.Registry.AbortFrame(); // Models the existing abort path on failed step/publication.
    CHECK(F.Registry.IsTerminal() && !F.Registry.ReadLatest() && !F.Registry.ReadHistorical(20));
    CHECK(F.Registry.ReadHistorical(10) == Before && Before->Frame == 10);
    CHECK(!F.Registry.CaptureCompletedBoundary(20) && !F.Registry.PrepareEmptyFrameReceipt(21));
    CHECK(!V::CompleteCanonicalInputFrameReceipt(F.Registry,20));
}

static void EmptyWorkerPresentationAndStopGates()
{
    FEmptyFixture F;
    auto Foreign = std::async(std::launch::async, [&] { return F.Registry.PrepareEmptyFrameReceipt(1); });
    CHECK(!Foreign.get());
    { I::FPresentationInputScope Scope; CHECK(!F.Registry.PrepareEmptyFrameReceipt(1)); }
    CHECK(V::BeginEmptyCanonicalInputFrameReceipt(F.Registry,1));
    auto ForeignInitial = std::async(std::launch::async, [&] { return F.Registry.CloseInitialBoundary(); });
    CHECK(!ForeignInitial.get());
    F.Commands->RequestStop(); // Existing asynchronous stop, not an invented missing-input policy.
    CHECK(!F.Registry.ServiceBoundary(2));
    CHECK(V::CompleteCanonicalInputFrameReceipt(F.Registry,1));
    CHECK(!F.Registry.CaptureCompletedBoundary(1));
    CHECK(F.Registry.ServiceBoundary(2) && F.Registry.IsTerminal());
    CHECK(F.Registry.ReadHistorical(1) && !F.Registry.PrepareEmptyFrameReceipt(2));
}

#include "EmptyInputPeriodTests.inc"
#include "ReservationObservationTests.inc"

int main(int Argc, char* Argv[])
{
    if (RebasedRegistryRegressionMain(Argc,Argv)) return 1;
    const auto Before = Checks;
    try
    {
        EmptyRequiresExplicitFullInputLifecycle(); DetachThenEmptyDoesNotRelabelPriorConsumption();
        PendingEmptyPreservesNextBoundaryBind(); EmptyFailuresNoCompletedReceipt(); EmptyWorkerPresentationAndStopGates();
        std::cout << "EMPTY_COHORT " << Checks-Before << " new checks; combined " << Checks
            << "; actual registry+driver adapter INPUT only; UE driver/world publication NOT_COMPILED/RUN\n";
        const auto BeforePeriod = Checks;
        TestActualEmptyInputPeriodEvidence();
        std::cout << "EMPTY_PERIOD " << Checks-BeforePeriod << " new checks; combined " << Checks << "; genuine INPUT history only, no world/branch/restore admission\n";
        const auto BeforeObservation=Checks;
        TestReservationObservation();
        std::cout << "RESERVATION_OBSERVATION " << Checks-BeforeObservation << " new checks; combined " << Checks
            << "; exact live INPUT reservation only, no world/branch/restore authority\n";
        return 0;
    }
    catch (const std::exception& E)
    { Budget = -1; AllocationHook = nullptr; std::cerr << "EMPTY_FAIL " << Checks << " " << E.what() << '\n'; return 1; }
}
