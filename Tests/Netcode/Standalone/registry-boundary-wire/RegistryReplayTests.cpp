#include "IAmSpeed/Input/InputSessionRegistry.h"
#include "RegistryBoundaryWireAdapter.h"
#include <cstdlib>
#include <future>
#include <fstream>
#include <iostream>
#include <new>
#include <stdexcept>
#include <string>

static thread_local int Budget = -1, Allocations = 0;
static thread_local void (*AllocationHook)() = nullptr;
static thread_local bool InAllocationHook = false;
void* operator new(std::size_t N)
{ if (AllocationHook && !InAllocationHook) { InAllocationHook = true; AllocationHook(); InAllocationHook = false; } if (Budget >= 0) { ++Allocations; if (Budget-- == 0) throw std::bad_alloc(); } if (void* P = std::malloc(N ? N : 1)) return P; throw std::bad_alloc(); }
void operator delete(void* P) noexcept { std::free(P); }
void operator delete(void* P, std::size_t) noexcept { std::free(P); }
void* operator new[](std::size_t N) { return ::operator new(N); }
void operator delete[](void* P) noexcept { ::operator delete(P); }
void operator delete[](void* P, std::size_t) noexcept { ::operator delete(P); }
namespace I = Speed::Input;
namespace V = Speed::Input::V2;
static int Checks = 0;
#define CHECK(E) do { ++Checks; if (!(E)) throw std::runtime_error(std::string(__func__) + ":" + std::to_string(__LINE__) + " " #E); } while (false)
static std::string BoundaryWireRunDirectory;
static void SaveBoundaryWireSample(const char* Name,const std::vector<std::uint8_t>& Bytes)
{
    if(BoundaryWireRunDirectory.empty())return;
    std::ofstream File(BoundaryWireRunDirectory+"/"+Name,std::ios::binary);CHECK(File.good());
    File.write(reinterpret_cast<const char*>(Bytes.data()),std::streamsize(Bytes.size()));File.close();CHECK(!File.fail());
}
static auto Contract()
{
    static const auto C = [] { V::FInputActionContractDescription D; D.Revision.Value = 1;
        D.Actions = V::FInputActionContract::BaseActions();
        D.Physical = {{0, V::EPhysicalDestination::Throttle}, {1, V::EPhysicalDestination::Brake}, {2, V::EPhysicalDestination::Steering}};
        return V::FInputActionContract::Create(std::move(D)); }(); return C;
}
static V::FInputFrame MakeFrame(std::uint64_t Producer, I::FFrameNumber N, bool Reset = true)
{
    V::FInputFrameData D; D.ConsumptionFrame = N; D.SourceSequence = N + 1; D.DeviceGeneration = {1};
    D.Producer = {I::EProducerKind::Device, Producer}; D.StreamEpoch = {7}; D.Reset = Reset;
    return V::FInputFrame(Contract(), std::move(D));
}
struct FFixture
{
    std::shared_ptr<V::FInputSessionCommands> Commands = std::make_shared<V::FInputSessionCommands>();
    V::FInputSessionRegistry Registry{Commands, 23};
    std::uint64_t CommandId = 0;
    std::size_t Count;
    explicit FFixture(std::size_t InCount = 6, std::uint64_t MissingReset = 0) : Count(InCount)
    {
        for (std::uint64_t Id = 1; Id <= Count; ++Id)
        {
            V::FSessionDescriptor D; D.Id = Id; D.Epoch = 7; D.Controller = 199 + Id; D.Producer = 99 + Id;
            D.Contract = Contract(); D.Kind = V::EProducerContract::ExactScenario; D.Actors = {{1000 + Id, 5}};
            D.AllowScenarioAppend = true;
            for (I::FFrameNumber N = 0; N < 4; ++N) D.Scenario.push_back(MakeFrame(D.Producer, N, !(Id == MissingReset && N == 1)));
            V::FBoundaryCommandDescriptor C; C.Id = ++CommandId; C.WorkerGeneration = 23;
            C.RegistryVersion = Registry.ReadRegistry()->Version; C.Binding = D;
            CHECK(Commands->Submit(C) == V::ECommandAdmission::Enqueued);
            CHECK(Registry.ServiceBoundary()); CHECK(Commands->Read(C.Id)->Status == V::EBoundaryStatus::Applied);
        }
        Complete(0); Pause();
    }
    void Complete(I::FFrameNumber N)
    {
        CHECK(Registry.PrepareFrame(N)); CHECK(Registry.InstallAll());
        const auto Frame = Registry.ReadInstalled(); CHECK(Frame && Frame->Frame == N && Frame->Inputs.size() == Count);
        for (const auto& S : Frame->Inputs)
        {
            CHECK(S.Snapshot && S.Snapshot->Input.GetData().ConsumptionFrame == N);
            CHECK(S.Snapshot->Input.GetData().SourceSequence == N + 1);
        }
        CHECK(Registry.BeginAll()); CHECK(Registry.ValidateComplete()); CHECK(Registry.CompleteAll());
    }
    V::FBoundaryCommandDescriptor Command(V::EBoundaryOperation Operation)
    {
        V::FBoundaryCommandDescriptor C; C.Id = ++CommandId; C.WorkerGeneration = 23;
        C.RegistryVersion = Registry.ReadRegistry()->Version; C.Operation = Operation; return C;
    }
    void Pause()
    {
        const auto C = Command(V::EBoundaryOperation::PauseAll);
        CHECK(Commands->Submit(C) == V::ECommandAdmission::Enqueued); CHECK(Registry.ServiceBoundary());
        CHECK(Commands->Read(C.Id)->Status == V::EBoundaryStatus::Applied);
    }
    V::FInputSessionRegistry::FGroupResumeRequest Request(std::uint64_t Generation = 1, I::FFrameNumber Next = 1)
    {
        const auto View = Registry.ReadRegistry();
        return {23, View->Version, Generation, Next, View->Phases};
    }
    void PausedUnconsumed(std::uint64_t Poll = 1)
    {
        const auto View = Registry.ReadRegistry(); CHECK(!View->Terminal && View->Phases.size() == Count);
        for (const auto& S : View->Phases) { CHECK(S.Phase == V::ESessionPhase::Paused); CHECK(Registry.PollCount(S.Session) == Poll); }
    }
};
static void GroupPublishAndNextRealFrame()
{
    FFixture F;
    const auto Initial = F.Registry.ReadHistorical(0); CHECK(Initial && Initial->Frame == 0);
    auto Token = F.Registry.PrepareGroupResumeAtBoundary(F.Request()); CHECK(Token);
    auto Prepared = F.Registry.ReadPreparedGroupResume(*Token); CHECK(Prepared && Prepared->Baselines.size() == 6);
    CHECK(Prepared->Next == 1 && Prepared->ResumeGeneration == 1 && Prepared->Registry->Version == 6);
    F.PausedUnconsumed();
    for (std::uint64_t Id = 1; Id <= 6; ++Id)
    {
        const auto& B = Prepared->Baselines[Id - 1];
        CHECK(B.Session == Id && B.Epoch == 7 && B.Input && B.Input->GetData().Reset);
        CHECK(B.Input->GetData().ConsumptionFrame == 1 && B.Input->GetData().Producer.Id == 99 + Id);
    }
    CHECK(!F.Registry.ServiceBoundary()); CHECK(!F.Registry.PrepareFrame(1));
    CHECK(!F.Registry.AppendExactScenarioFrame(1, 7, 200, MakeFrame(100, 4)));
    CHECK(!F.Registry.BeginRetainedReplay(0, 0, {{1001, 5}}));
    CHECK(!F.Registry.PrepareGroupResumeAtBoundary(F.Request()));
    F.Registry.AbortFrame(); F.PausedUnconsumed(); // no frame exists; must not invalidate the group
    CHECK(F.Registry.PrepareGroupResumeFinalization(*Token)); CHECK(F.Registry.PrepareGroupResumeFinalization(*Token));
    std::shared_ptr<const V::FInputSessionRegistry::FGroupResumePublication> CommonInputPublication;
    Allocations = 0; Budget = 0;
    F.Registry.FinalizePreparedGroupResume(*Token);
    auto StillPaused = F.Registry.ReadRegistry();
    F.Registry.PublishPreparedGroupResume(*Token);
    auto ActiveButHeld = F.Registry.ReadRegistry();
    CommonInputPublication = Prepared; // INPUT-only fixture marker, no world restore claim
    const bool PollStillBlocked = !F.Registry.PrepareFrame(1);
    F.Registry.ReleasePreparedGroupResume(*Token);
    Budget = -1;
    CHECK(Allocations == 0 && PollStillBlocked);
    CHECK(StillPaused->Phases[0].Phase == V::ESessionPhase::Paused);
    CHECK(ActiveButHeld == Prepared->Registry && CommonInputPublication == Prepared);
    CHECK(!F.Registry.AbortGroupResume(*Token) && !F.Registry.ReadPreparedGroupResume(*Token));
    F.Complete(1); // the exact Reset reserved above is STILL the next real input
    const auto Frame = F.Registry.ReadLatest(); CHECK(Frame && Frame->Frame == 1);
    for (const auto& S : Frame->Inputs) { CHECK(S.Snapshot->Input.GetData().Reset); CHECK(F.Registry.PollCount(S.Session) == 2); }
    CHECK(F.Registry.ReadHistorical(0) == Initial && Initial->Frame == 0);
    F.Pause();
    CHECK(!F.Registry.PrepareGroupResumeAtBoundary(F.Request(1, 2))); // committed generation survives
    auto Next = F.Registry.PrepareGroupResumeAtBoundary(F.Request(2, 2)); CHECK(Next);
    CHECK(F.Registry.AbortGroupResume(*Next));
}
static void AbortAndExactRoster()
{
    FFixture F(2);
    auto Request = F.Request(); auto Bad = Request;
    Bad.WorkerGeneration++; CHECK(!F.Registry.PrepareGroupResumeAtBoundary(Bad));
    Bad = Request; Bad.RegistryVersion--; CHECK(!F.Registry.PrepareGroupResumeAtBoundary(Bad));
    Bad = Request; Bad.Next++; CHECK(!F.Registry.PrepareGroupResumeAtBoundary(Bad));
    Bad = Request; Bad.ResumeGeneration = 0; CHECK(!F.Registry.PrepareGroupResumeAtBoundary(Bad));
    Bad = Request; Bad.Sessions.pop_back(); CHECK(!F.Registry.PrepareGroupResumeAtBoundary(Bad));
    Bad = Request; Bad.Sessions[1] = Bad.Sessions[0]; CHECK(!F.Registry.PrepareGroupResumeAtBoundary(Bad));
    Bad = Request; std::swap(Bad.Sessions[0], Bad.Sessions[1]); CHECK(!F.Registry.PrepareGroupResumeAtBoundary(Bad));
    Bad = Request; Bad.Sessions[1].Epoch++; CHECK(!F.Registry.PrepareGroupResumeAtBoundary(Bad));
    Bad = Request; Bad.Sessions[1].Phase = V::ESessionPhase::Active; CHECK(!F.Registry.PrepareGroupResumeAtBoundary(Bad));
    auto Token = F.Registry.PrepareGroupResumeAtBoundary(Request); CHECK(Token);
    { I::FPresentationInputScope Scope; CHECK(!F.Registry.AbortGroupResume(*Token)); CHECK(!F.Registry.PrepareGroupResumeFinalization(*Token)); }
    auto Foreign = std::async(std::launch::async, [&] { return F.Registry.PrepareGroupResumeFinalization(*Token) || F.Registry.AbortGroupResume(*Token); });
    CHECK(!Foreign.get()); CHECK(F.Registry.PrepareGroupResumeFinalization(*Token));
    CHECK(F.Registry.AbortGroupResume(*Token)); F.PausedUnconsumed();
    auto Again = F.Registry.PrepareGroupResumeAtBoundary(Request); CHECK(Again);
    CHECK(!F.Registry.AbortGroupResume(*Token)); CHECK(F.Registry.AbortGroupResume(*Again));
    // Dropping the opaque token cannot silently release owner reservations.
    { auto Dropped = F.Registry.PrepareGroupResumeAtBoundary(Request); CHECK(Dropped); }
    CHECK(!F.Registry.PrepareFrame(1)); CHECK(F.Registry.CloseWorkerLedger());
}
static void PartialFailureAndAllocationFailures()
{
    {
        FFixture F(3, 3); // owners 1/2 prepare; owner3 has no explicitly supplied Reset
        CHECK(!F.Registry.PrepareGroupResumeAtBoundary(F.Request())); F.PausedUnconsumed();
        CHECK(!F.Registry.PrepareGroupResumeAtBoundary(F.Request())); F.PausedUnconsumed();
        CHECK(F.Registry.CloseWorkerLedger()); // no leaked owner/source reservation
    }
    int Failed = 0, Succeeded = 0;
    for (int Limit = 0; Limit != 70; ++Limit)
    {
        FFixture F(2);
        const auto Request = F.Request();
        Allocations = 0; Budget = Limit;
        auto Token = F.Registry.PrepareGroupResumeAtBoundary(Request);
        Budget = -1;
        if (Token) { ++Succeeded; CHECK(F.Registry.AbortGroupResume(*Token)); }
        else ++Failed;
        F.PausedUnconsumed();
        auto Retry = F.Registry.PrepareGroupResumeAtBoundary(Request); CHECK(Retry);
        CHECK(F.Registry.PrepareGroupResumeFinalization(*Retry)); CHECK(F.Registry.AbortGroupResume(*Retry));
    }
    CHECK(Failed > 10 && Succeeded > 0);
}
static void CommandsAndStopFence()
{
    {
        FFixture F(2); auto Token = F.Registry.PrepareGroupResumeAtBoundary(F.Request()); CHECK(Token);
        const auto Pause = F.Command(V::EBoundaryOperation::PauseAll);
        CHECK(F.Commands->Submit(Pause) == V::ECommandAdmission::Enqueued);
        CHECK(!F.Registry.PrepareGroupResumeFinalization(*Token)); F.PausedUnconsumed();
        CHECK(!F.Registry.AbortGroupResume(*Token)); CHECK(F.Registry.ServiceBoundary());
        CHECK(F.Commands->Read(Pause.Id)->Status == V::EBoundaryStatus::Applied);
        auto Retry = F.Registry.PrepareGroupResumeAtBoundary(F.Request()); CHECK(Retry); CHECK(F.Registry.AbortGroupResume(*Retry));
    }
    {
        FFixture F(2); auto Token = F.Registry.PrepareGroupResumeAtBoundary(F.Request()); CHECK(Token);
        F.Commands->RequestStop(); CHECK(!F.Registry.PrepareGroupResumeFinalization(*Token));
        CHECK(F.Registry.ServiceBoundary()); CHECK(F.Registry.ReadRegistry()->Terminal);
    }
    {
        FFixture F(2); auto Token = F.Registry.PrepareGroupResumeAtBoundary(F.Request()); CHECK(Token);
        CHECK(F.Registry.PrepareGroupResumeFinalization(*Token));
        std::promise<void> Started;
        auto Stop = std::async(std::launch::async, [&] { Started.set_value(); F.Commands->RequestStop(); return true; });
        Started.get_future().wait(); CHECK(Stop.wait_for(std::chrono::milliseconds(10)) == std::future_status::timeout);
        F.Registry.FinalizePreparedGroupResume(*Token);
        CHECK(!F.Registry.CloseWorkerLedger()); // cannot undo/release a finalized unpublished group
        F.Registry.PublishPreparedGroupResume(*Token);
        CHECK(Stop.wait_for(std::chrono::milliseconds(10)) == std::future_status::timeout);
        F.Registry.ReleasePreparedGroupResume(*Token);
        CHECK(Stop.get()); CHECK(F.Registry.ServiceBoundary());
        CHECK(F.Registry.ReadRegistry()->Terminal); // stop linearizes AFTER the committed input publication
    }
}
static void RemoteRegistryGuardStaysClosed()
{
    FFixture F(1);
    auto C = F.Command(V::EBoundaryOperation::Bind);
    C.Binding = F.Registry.ReadRegistry()->Bindings[0];
    C.Binding.Id = 20; C.Binding.Producer = 120; C.Binding.Controller = 220; C.Binding.Actors = {{1020, 1}};
    C.Binding.Kind = V::EProducerContract::ExactRemote; C.Binding.AllowScenarioAppend = false; C.Binding.Scenario.clear();
    CHECK(F.Commands->Submit(C) == V::ECommandAdmission::Enqueued); CHECK(F.Registry.ServiceBoundary());
    CHECK(F.Commands->Read(C.Id)->Status == V::EBoundaryStatus::Rejected);
    CHECK(F.Registry.ReadRegistry()->Bindings.size() == 1);
}
int ResumeRegressionMain()
{
    try
    {
        GroupPublishAndNextRealFrame(); AbortAndExactRoster(); PartialFailureAndAllocationFailures();
        CommandsAndStopFence(); RemoteRegistryGuardStaysClosed();
        std::cout << "PASS " << Checks << " checks; real registry/owner derivative, sealed local scenarios; INPUT publication only; native guards closed\n";
        return 0;
    }
    catch (const std::exception& E) { Budget = -1; std::cerr << "FAIL " << Checks << " " << E.what() << '\n'; return 1; }
}

#include "GroupReplayTests.inc"
