#include "ExactRemoteInputProducer.h"
#include "ExactRemoteInputIngress.h"
#include <atomic>
#include "IAmSpeed/Input/SameFrameInputOwner.h"
#include "IAmSpeed/Input/InputSessionRegistry.h"
#include "../transport/TransportFixture.h"
#include <iostream>
#include <stdexcept>
#include <string>
using namespace Speed::Netcode;
static int Checks = 0;
#define CHECK(E) do { ++Checks; if (!(E)) throw std::runtime_error(std::string(__func__) + ":" + std::to_string(__LINE__) + " " #E); } while (false)
static std::unique_ptr<FExactRemoteInputProducer> Producer()
{ return FExactRemoteInputProducer::Create(Fixture::Contract(), {7}, {Speed::Input::EProducerKind::Device, 99}, 0); }
static V2::FInputFrame Frame(std::uint64_t N)
{ auto F = ToV2(Fixture::Packet(N), Fixture::Contract()); CHECK(F); return *F; }
static void Consume(FExactRemoteInputProducer& P, std::uint64_t N)
{
    auto C = P.FreezeForOwner(N); CHECK(C); auto F = P.Produce(N); CHECK(F);
    CHECK(F->GetData().ConsumptionFrame == N && F->GetData().SourceSequence == C->Sequence);
    CHECK(!P.Produce(N)); CHECK(P.CloseFrozenCutoff(*C)); CHECK(!P.CloseFrozenCutoff(*C));
}
static void ExactFramesAndImmutableCutoff()
{
    auto P = Producer(); CHECK(P && P->GetProducerContract() == V2::EProducerContract::ExactRemote && P->SupportsPollFence());
    CHECK(!P->HasExactFrame(0) && !P->FreezeForOwner(0) && !P->Produce(0));
    CHECK(P->SubmitFrame(Frame(1)) == ERemoteSubmission::Accepted);
    CHECK(P->SubmitFrame(Frame(0)) == ERemoteSubmission::Accepted);
    CHECK(P->InspectBaseline(0)); CHECK(!P->InspectBaseline(1));
    auto Cutoff = P->FreezeForOwner(0); CHECK(Cutoff);
    // Future arrivals can proceed while the immutable cutoff is leased.
    CHECK(P->SubmitFrame(Frame(2)) == ERemoteSubmission::Accepted);
    CHECK(P->SubmitFrame(Frame(0)) == ERemoteSubmission::Duplicate);
    auto Different = Fixture::Packet(0); Different.Input.Values[0] = 42;
    CHECK(P->SubmitFrame(*ToV2(Different, Fixture::Contract())) == ERemoteSubmission::Conflict);
    auto Frozen = P->Produce(0); CHECK(Frozen && Frozen->GetData().Values[0] == 0);
    CHECK(P->CloseFrozenCutoff(*Cutoff)); CHECK(!P->HasExactFrame(0));
    Consume(*P, 1); Consume(*P, 2);
    CHECK(P->SubmitFrame(Frame(0)) == ERemoteSubmission::Duplicate);
    CHECK(P->SubmitFrame(Frame(67)) == ERemoteSubmission::TooFar);
    CHECK(!P->HasExactFrame(3));
}
static void ForeignCallsCannotReleaseLease()
{
    auto P = Producer(); CHECK(P->SubmitFrame(Frame(0)) == ERemoteSubmission::Accepted);
    auto C = P->FreezeForOwner(0); CHECK(C);
    bool ForeignClose = true, ForeignProduce = true, ForeignFreeze = true;
    V2::ELifecycleResult ForeignCancel{}, ForeignPause{};
    std::thread Foreign([&]
    {
        ForeignClose = P->CloseFrozenCutoff(*C); ForeignProduce = bool(P->Produce(0));
        ForeignFreeze = bool(P->FreezeForOwner(0)); ForeignCancel = P->CancelLifecycle();
        ForeignPause = P->SetLifecyclePaused(true);
    }); Foreign.join();
    CHECK(!ForeignClose && !ForeignProduce && !ForeignFreeze);
    CHECK(ForeignCancel == V2::ELifecycleResult::Rejected && ForeignPause == V2::ELifecycleResult::Rejected);
    CHECK(P->Produce(0)); CHECK(P->CloseFrozenCutoff(*C));
}
static void FailedCloseQuarantinesAndPauseDoesNotForge()
{
    auto Bad = Producer(); CHECK(Bad->SubmitFrame(Frame(0)) == ERemoteSubmission::Accepted);
    auto C = Bad->FreezeForOwner(0); CHECK(C); CHECK(Bad->Produce(0));
    CHECK(!Bad->CloseFrozenCutoff({})); CHECK(!Bad->FreezeForOwner(0));
    CHECK(Bad->SubmitFrame(Frame(1)) == ERemoteSubmission::Closed);
    auto P = Producer(); CHECK(P->SetLifecyclePaused(true) == V2::ELifecycleResult::Applied);
    CHECK(P->SubmitFrame(Frame(0)) == ERemoteSubmission::Accepted); CHECK(!P->FreezeForOwner(0));
    CHECK(P->SetLifecyclePaused(false) == V2::ELifecycleResult::Applied); Consume(*P, 0);
    CHECK(P->SetLifecyclePaused(true) == V2::ELifecycleResult::Applied);
    auto Held = Fixture::Packet(1); Held.Input.SourceSequence = 1; Held.Input.ActiveMask = 0;
    Held.Input.Values = {}; Held.Input.Edges.clear();
    CHECK(P->SubmitFrame(*ToV2(Held, Fixture::Contract())) == ERemoteSubmission::Accepted);
    CHECK(P->SetLifecyclePaused(false) == V2::ELifecycleResult::Applied);
    C = P->FreezeForOwner(1); CHECK(C); auto F = P->Produce(1);
    CHECK(F && !F->GetData().Reset && F->GetData().Transitions.empty()); CHECK(P->CloseFrozenCutoff(*C));
    CHECK(P->CancelLifecycle() == V2::ELifecycleResult::Applied);
    CHECK(P->SubmitFrame(Frame(2)) == ERemoteSubmission::Closed);
}
static void SourceContinuityAndAddress()
{
    auto P = Producer(); CHECK(P->SubmitFrame(Frame(0)) == ERemoteSubmission::Accepted); Consume(*P, 0);
    auto Wrong = Fixture::Packet(1); ++Wrong.Epoch;
    CHECK(P->SubmitFrame(*ToV2(Wrong, Fixture::Contract())) == ERemoteSubmission::Invalid);
    Wrong = Fixture::Packet(1); Wrong.Input.Edges.clear();
    CHECK(P->SubmitFrame(*ToV2(Wrong, Fixture::Contract())) == ERemoteSubmission::Accepted);
    CHECK(!P->FreezeForOwner(1)); CHECK(P->SubmitFrame(Frame(1)) == ERemoteSubmission::Closed);
    auto MissingBaseline = Producer(); auto P0 = Fixture::Packet(0); P0.Input.Reset = false;
    CHECK(MissingBaseline->SubmitFrame(*ToV2(P0, Fixture::Contract())) == ERemoteSubmission::Accepted);
    CHECK(!MissingBaseline->FreezeForOwner(0));
    auto Presentation = Producer();
    {
        Speed::Input::FPresentationInputScope Scope;
        CHECK(Presentation->SubmitFrame(Frame(0)) == ERemoteSubmission::Invalid);
        CHECK(!Presentation->FreezeForOwner(0));
    }
}
static void CurrentOwnerAdmissionStillRefuses()
{
    for (auto Kind : {V2::EProducerContract::ExactRemote, V2::EProducerContract::Device,
        V2::EProducerContract::AI, V2::EProducerContract::ExactScenario})
    {
        V2::FOwnerInputBinding B; B.AdapterId = 1; B.Contract = Fixture::Contract();
        B.Epoch = {7}; B.Producer = {Speed::Input::EProducerKind::Device, 99}; B.ProducerContract = Kind;
        CHECK(!V2::FSameFrameInputOwner::Create(Producer(), B));
    }
}
static void FullEdgesAndStaleLeaseSerial()
{
    auto P = Producer();
    for (std::uint64_t N = 0; N <= 5; ++N)
    {
        CHECK(P->SubmitFrame(Frame(N)) == ERemoteSubmission::Accepted);
        auto C = P->FreezeForOwner(N); CHECK(C); auto F = P->Produce(N); CHECK(F);
        if (N == 5)
        {
            const auto& E = F->GetData().Transitions;
            CHECK(E.size() == 2 && E[0].Action == 3 && E[1].Action == 3);
            CHECK(E[0].State == V2::ETransition::Started && E[1].State == V2::ETransition::Completed);
            CHECK(E[0].Order < E[1].Order && F->GetData().Values[3] == 0);
        }
        CHECK(P->CloseFrozenCutoff(*C));
    }
    auto SameSource = Producer(); CHECK(SameSource->SubmitFrame(Frame(0)) == ERemoteSubmission::Accepted);
    auto C0 = SameSource->FreezeForOwner(0); CHECK(C0); CHECK(SameSource->Produce(0)); CHECK(SameSource->CloseFrozenCutoff(*C0));
    auto Held = Fixture::Packet(1); Held.Input.SourceSequence = 1; Held.Input.ActiveMask = 0;
    Held.Input.Values = {}; Held.Input.Edges.clear();
    CHECK(SameSource->SubmitFrame(*ToV2(Held, Fixture::Contract())) == ERemoteSubmission::Accepted);
    auto C1 = SameSource->FreezeForOwner(1); CHECK(C1 && C1->Sequence == C0->Sequence && C1->LifecycleFence != C0->LifecycleFence);
    CHECK(SameSource->Produce(1));
    CHECK(!SameSource->CloseFrozenCutoff(*C0)); // old token cannot confirm a new lease
    CHECK(!SameSource->CloseFrozenCutoff(*C1));
    CHECK(!SameSource->FreezeForOwner(1));
}
static void GenerationChangeRequiresFreshReset()
{
    auto Bad = Producer(); CHECK(Bad->SubmitFrame(Frame(0)) == ERemoteSubmission::Accepted); Consume(*Bad, 0);
    auto Changed = Fixture::Packet(1); Changed.Input.DeviceGeneration = 2;
    CHECK(Bad->SubmitFrame(*ToV2(Changed, Fixture::Contract())) == ERemoteSubmission::Accepted);
    CHECK(!Bad->FreezeForOwner(1));
    auto Good = Producer(); CHECK(Good->SubmitFrame(Frame(0)) == ERemoteSubmission::Accepted); Consume(*Good, 0);
    Changed.Input.Reset = true; Changed.Input.Edges.clear();
    CHECK(Good->SubmitFrame(*ToV2(Changed, Fixture::Contract())) == ERemoteSubmission::Accepted); Consume(*Good, 1);
}
static void ExplicitLaterBaselinePreservesSameEpochAndHistory()
{
    auto P = Producer(); CHECK(P->SubmitFrame(Frame(0)) == ERemoteSubmission::Accepted); Consume(*P, 0);
    CHECK(P->SetLifecyclePaused(true) == V2::ELifecycleResult::Applied);
    CHECK(!P->InspectBaseline(1)); // no transport frame => no invented baseline
    auto Reset = Fixture::Packet(1); Reset.Input.Reset = true; Reset.Input.Edges.clear();
    CHECK(P->SubmitFrame(*ToV2(Reset, Fixture::Contract())) == ERemoteSubmission::Accepted);
    const auto First = P->InspectBaseline(1), Again = P->InspectBaseline(1);
    CHECK(First && Again && First->GetData().Reset && Again->GetData().Reset);
    CHECK(First->GetData().StreamEpoch.Value == 7 && First->GetData().Producer.Id == 99);
    CHECK(First->GetData().Transitions.empty() && First->GetData().SourceSequence == 2);
    CHECK(!P->HasExactFrame(1) && !P->FreezeForOwner(1)); // inspection does not unpause
    CHECK(!P->InspectBaseline(0) && !P->InspectBaseline(2));
    CHECK(P->SubmitFrame(Frame(0)) == ERemoteSubmission::Duplicate); // Last/history preserved
    CHECK(P->SetLifecyclePaused(false) == V2::ELifecycleResult::Applied);
    auto Lease = P->FreezeForOwner(1); CHECK(Lease);
    CHECK(!P->InspectBaseline(1)); // cannot inspect a currently leased transaction
    auto Polled = P->Produce(1); CHECK(Polled && Polled->GetData().Reset);
    CHECK(Polled->GetData().Values == First->GetData().Values);
    CHECK(P->CloseFrozenCutoff(*Lease));
    CHECK(!P->InspectBaseline(1));
    CHECK(P->SubmitFrame(Frame(2)) == ERemoteSubmission::Accepted); Consume(*P, 2);
    // Source inspection supplies no current owner admission; the existing guard
    // tests still reject ExactRemote even with valid source-provided Reset data.
}
static void BaselineInspectionRejectsStaleResetWithoutMutation()
{
    for (int Case = 0; Case < 4; ++Case)
    {
        auto P = Producer();
        auto Initial = Fixture::Packet(0); Initial.Input.DeviceGeneration = 2;
        CHECK(P->SubmitFrame(*ToV2(Initial, Fixture::Contract())) == ERemoteSubmission::Accepted); Consume(*P, 0);
        CHECK(P->SetLifecyclePaused(true) == V2::ELifecycleResult::Applied);
        auto Next = Fixture::Packet(1); Next.Input.Reset = true; Next.Input.Edges.clear();
        Next.Input.DeviceGeneration = 2;
        if (Case == 0) Next.Input.SourceSequence = 1; // Reset must be source-fresh
        if (Case == 1) Next.Input.DeviceGeneration = 1; // generation cannot regress
        if (Case == 2)
        {
            Next.Input.Reset = false; Next.Input.Values = {}; Next.Input.ActiveMask = 0;
        } // a valid held frame still cannot be advertised as a Reset baseline
        CHECK(P->SubmitFrame(*ToV2(Next, Fixture::Contract())) == ERemoteSubmission::Accepted);
        const bool Expected = Case == 3;
        CHECK(bool(P->InspectBaseline(1)) == Expected);
        CHECK(bool(P->InspectBaseline(1)) == Expected); // no source consume/quarantine
        CHECK(P->SetLifecyclePaused(false) == V2::ELifecycleResult::Applied);
        auto Lease = P->FreezeForOwner(1);
        if (Case < 2) CHECK(!Lease); // bad continuity is quarantined only by Poll
        else { CHECK(Lease); CHECK(P->Produce(1)); CHECK(P->CloseFrozenCutoff(*Lease)); }
    }
}
static void ReadinessDoesNotConsumeOrForge()
{
    using R = V2::EInputFrameReadiness;
    auto P = Producer();
    for (int N = 0; N < 4; ++N) CHECK(P->QueryFrameReadiness(0) == R::Waiting);
    CHECK(P->QueryFrameReadiness(1) == R::Rejected);
    CHECK(P->SubmitFrame(Frame(1)) == ERemoteSubmission::Accepted);
    CHECK(P->QueryFrameReadiness(0) == R::Waiting);
    CHECK(P->SubmitFrame(Frame(0)) == ERemoteSubmission::Accepted);
    for (int N = 0; N < 4; ++N) CHECK(P->QueryFrameReadiness(0) == R::Ready);
    auto Cut = P->FreezeForOwner(0); CHECK(Cut);
    CHECK(P->QueryFrameReadiness(0) == R::Rejected);
    CHECK(P->Produce(0)); CHECK(P->CloseFrozenCutoff(*Cut));
    CHECK(P->QueryFrameReadiness(1) == R::Ready);
    CHECK(P->SetLifecyclePaused(true) == V2::ELifecycleResult::Applied);
    CHECK(P->QueryFrameReadiness(1) == R::Rejected);
    CHECK(P->SetLifecyclePaused(false) == V2::ELifecycleResult::Applied);
    CHECK(P->QueryFrameReadiness(1) == R::Ready); Consume(*P, 1);
    CHECK(P->QueryFrameReadiness(2) == R::Waiting);
    CHECK(P->CancelLifecycle() == V2::ELifecycleResult::Applied);
    CHECK(P->QueryFrameReadiness(2) == R::Rejected);
    auto Bad = Producer(); auto Packet = Fixture::Packet(0); Packet.Input.Reset = false;
    CHECK(Bad->SubmitFrame(*ToV2(Packet, Fixture::Contract())) == ERemoteSubmission::Accepted);
    CHECK(Bad->QueryFrameReadiness(0) == R::Rejected);
    CHECK(Bad->QueryFrameReadiness(0) == R::Rejected);
    // Inspection has not quarantined/consumed the received record.
    CHECK(Bad->SubmitFrame(*ToV2(Packet, Fixture::Contract())) == ERemoteSubmission::Duplicate);
}
static void IngressOwnsNoPhysicalSourceBeforeWorkerBind()
{
    std::shared_ptr<FExactRemoteInputIngress> Ingress;
    std::thread Acquirer([&] { Ingress = FExactRemoteInputIngress::Create(Fixture::Contract(), {7},
        {Speed::Input::EProducerKind::Device, 99}, 0); }); Acquirer.join();
    CHECK(Ingress && Ingress->GetState() == ERemoteIngressState::Unbound);
    CHECK(Ingress->SubmitFrame(Frame(0)) == ERemoteSubmission::Closed);
    CHECK(Ingress->MatchesBinding(*Fixture::Contract(), {7}, {Speed::Input::EProducerKind::Device, 99}, 0));
    CHECK(!Ingress->MatchesBinding(*Fixture::Contract(), {8}, {Speed::Input::EProducerKind::Device, 99}, 0));
    CHECK(!Ingress->MatchesBinding(*Fixture::Contract(), {7}, {Speed::Input::EProducerKind::AI, 99}, 0));
    CHECK(!Ingress->MatchesBinding(*Fixture::Contract(), {7}, {Speed::Input::EProducerKind::Device, 100}, 0));
    CHECK(!Ingress->MatchesBinding(*Fixture::Contract(), {7}, {Speed::Input::EProducerKind::Device, 99}, 1));
    auto Source = Ingress->CreateProducerForCurrentOwner(); CHECK(Source);
    CHECK(Ingress->GetState() == ERemoteIngressState::Bound && !Ingress->CreateProducerForCurrentOwner());
    CHECK(Source->GetProducerContract() == V2::EProducerContract::ExactRemote);
    CHECK(Source->QueryFrameReadiness(0) == V2::EInputFrameReadiness::Waiting);
    auto Received = Frame(0); ERemoteSubmission Result = ERemoteSubmission::Invalid;
    std::thread Transport([&] { Result = Ingress->SubmitFrame(Received); }); Transport.join();
    CHECK(Result == ERemoteSubmission::Accepted);
    CHECK(Source->QueryFrameReadiness(0) == V2::EInputFrameReadiness::Ready);
    bool ForeignRejected = false;
    std::thread Foreign([&] {
        ForeignRejected = Source->QueryFrameReadiness(0) == V2::EInputFrameReadiness::Rejected
            && !Source->InspectBaseline(0) && !Source->PollFence()->FreezeForOwner(0) && !Source->Produce(0)
            && Source->SetLifecyclePaused(true) == V2::ELifecycleResult::Rejected
            && Source->CancelLifecycle() == V2::ELifecycleResult::Rejected;
    }); Foreign.join(); CHECK(ForeignRejected);
    auto Cut = Source->PollFence()->FreezeForOwner(0); CHECK(Cut);
    CHECK(!Source->AppendExactScenarioFrame(Received));
    CHECK(Ingress->SubmitFrame(Frame(1)) == ERemoteSubmission::Accepted);
    CHECK(Ingress->SubmitFrame(Received) == ERemoteSubmission::Duplicate);
    auto F = Source->Produce(0); CHECK(F && F->GetData().Producer.Kind == Speed::Input::EProducerKind::Device);
    CHECK(F->GetData().Producer.Id == 99 && F->GetData().StreamEpoch.Value == 7);
    CHECK(Source->PollFence()->CloseFrozenCutoff(*Cut));
    CHECK(Source->QueryFrameReadiness(1) == V2::EInputFrameReadiness::Ready);
    Source.reset();
    CHECK(Ingress->GetState() == ERemoteIngressState::Retired);
    CHECK(Ingress->SubmitFrame(Frame(1)) == ERemoteSubmission::Closed);
    CHECK(!Ingress->CreateProducerForCurrentOwner()); // same epoch/service never resurrects
}
static void IngressSubmissionSerializesWithOwnerRetirement()
{
    auto Ingress = FExactRemoteInputIngress::Create(Fixture::Contract(), {7},
        {Speed::Input::EProducerKind::Device, 99}, 0);
    auto Source = Ingress->CreateProducerForCurrentOwner(); CHECK(Source);
    const auto Received = Frame(0);
    std::atomic<bool> Started{false}, Bad{false};
    std::thread Transport([&] {
        Started.store(true);
        for (int N = 0; N < 2048; ++N)
        {
            const auto Result = Ingress->SubmitFrame(Received);
            if (Result != ERemoteSubmission::Accepted && Result != ERemoteSubmission::Duplicate
                && Result != ERemoteSubmission::Closed) Bad.store(true);
        }
    });
    while (!Started.load()) std::this_thread::yield();
    Source.reset(); Transport.join();
    CHECK(!Bad.load() && Ingress->GetState() == ERemoteIngressState::Retired);
    CHECK(Ingress->SubmitFrame(Received) == ERemoteSubmission::Closed);
}
static V2::FSessionDescriptor RemoteDescriptor(std::uint64_t Id)
{
    V2::FSessionDescriptor D; D.Id = Id; D.Epoch = 7; D.Controller = Id + 100;
    D.Producer = Id + 98; D.Journal = Id + 200; D.First = 0;
    D.Kind = V2::EProducerContract::ExactRemote; D.Contract = Fixture::Contract(); D.Actors = {{Id * 10, 1}};
    return D;
}
static V2::FInputFrame RemoteFrame(std::uint64_t Id, std::uint64_t N)
{
    auto Data = Frame(N).GetData(); Data.Producer.Id = Id + 98;
    return V2::FInputFrame(Fixture::Contract(), std::move(Data));
}
static std::shared_ptr<FExactRemoteInputIngress> RemoteService(const V2::FSessionDescriptor& D)
{ return FExactRemoteInputIngress::Create(D.Contract, {D.Epoch}, {Speed::Input::EProducerKind::Device,D.Producer}, D.First); }
static V2::FBoundaryCommandDescriptor RemoteBind(const V2::FSessionDescriptor& D, std::uint64_t Id, std::uint64_t Version)
{
    V2::FBoundaryCommandDescriptor C; C.Id = Id; C.WorkerGeneration = 7;
    C.RegistryVersion = Version; C.Operation = V2::EBoundaryOperation::Bind; C.Binding = D; return C;
}
static void RegistryRemoteOptInAndCompleteCohort()
{
    using R = V2::EInputFrameReadiness;
    const auto D1 = RemoteDescriptor(1), D2 = RemoteDescriptor(2);
    auto A = RemoteService(D1), B = RemoteService(D2);
    auto Commands = std::make_shared<V2::FInputSessionCommands>();
    V2::FInputSessionRegistry Registry(Commands, 7, {}, V2::EExactRemoteAdmission::OriginalDeviceExact);
    CHECK(Registry.QueryFrameReadiness(0, 2) == R::Waiting);
    CHECK(Registry.RegisterJournalAtBoundary({D1.Journal, {}, {}, A}));
    CHECK(Registry.RegisterJournalAtBoundary({D1.Journal, {}, {}, A})); // idempotent exact service
    CHECK(!Registry.RegisterJournalAtBoundary({D1.Journal + 1, {}, {}, A}));
    CHECK(!Registry.RegisterJournalAtBoundary({D1.Journal, {}, {}, B}));
    CHECK(Commands->Submit(RemoteBind(D1, 1, 0)) == V2::ECommandAdmission::Enqueued);
    CHECK(Registry.ServiceBoundary(0) && Commands->Read(1)->Status == V2::EBoundaryStatus::Applied);
    CHECK(A->GetState() == ERemoteIngressState::Bound && Registry.ConstructionCount() == 1);
    CHECK(A->SubmitFrame(RemoteFrame(1, 0)) == ERemoteSubmission::Accepted);
    CHECK(Registry.QueryFrameReadiness(0, 2) == R::Waiting && Registry.PollCount(1) == 0);
    CHECK(Registry.RegisterJournalAtBoundary({D2.Journal, {}, {}, B}));
    CHECK(Commands->Submit(RemoteBind(D2, 2, 1)) == V2::ECommandAdmission::Enqueued);
    CHECK(Registry.ServiceBoundary(0) && Commands->Read(2)->Status == V2::EBoundaryStatus::Applied);
    for (int N = 0; N < 5; ++N)
    {
        CHECK(Registry.QueryFrameReadiness(0, 2) == R::Waiting);
        CHECK(!Registry.PrepareFrame(0));
        CHECK(Registry.ReadLastFrameFailure().Reason == V2::ERegistryFrameFailure::InputUnavailable);
        CHECK(Registry.PollCount(1) == 0 && Registry.PollCount(2) == 0 && !Registry.IsTerminal());
    }
    CHECK(B->SubmitFrame(RemoteFrame(2, 0)) == ERemoteSubmission::Accepted);
    for (std::uint64_t N = 0; N < 2; ++N)
    {
        CHECK(Registry.QueryFrameReadiness(N, 2) == R::Ready);
        CHECK(Registry.PrepareFrame(N) && Registry.InstallAll() && Registry.BeginAll());
        const auto Installed = Registry.ReadInstalled(); CHECK(Installed && Installed->Inputs.size() == 2);
        for (const auto& S : Installed->Inputs)
        {
            CHECK(S.Snapshot && S.Snapshot->Input.GetData().ConsumptionFrame == N);
            CHECK(S.Snapshot->Input.GetData().Producer.Kind == Speed::Input::EProducerKind::Device);
            CHECK(S.Snapshot->Input.GetData().Producer.Id == S.Session + 98);
            CHECK(S.Snapshot->Applied == S.Snapshot->Input.GetData().Values);
        }
        CHECK(Registry.ValidateComplete() && Registry.CompleteAll());
        CHECK(Registry.ReadLatest() == Installed && Registry.PollCount(1) == N + 1 && Registry.PollCount(2) == N + 1);
        if (N == 0)
        {
            CHECK(A->SubmitFrame(RemoteFrame(1, 1)) == ERemoteSubmission::Accepted);
            CHECK(Registry.QueryFrameReadiness(1, 2) == R::Waiting && !Registry.PrepareFrame(1));
            CHECK(Registry.PollCount(1) == 1 && Registry.PollCount(2) == 1);
            CHECK(B->SubmitFrame(RemoteFrame(2, 1)) == ERemoteSubmission::Accepted);
        }
    }
    const auto Before = Registry.ReadLatest();
    CHECK(A->SubmitFrame(RemoteFrame(1, 2)) == ERemoteSubmission::Accepted);
    auto Data = RemoteFrame(2, 2).GetData(); Data.DeviceGeneration.Value = 2; // no Reset => invalid continuity
    CHECK(B->SubmitFrame(V2::FInputFrame(D2.Contract, Data)) == ERemoteSubmission::Accepted);
    CHECK(Registry.QueryFrameReadiness(2, 2) == R::Rejected && !Registry.PrepareFrame(2));
    CHECK(Registry.PollCount(1) == 2 && Registry.PollCount(2) == 2 && Registry.ReadLatest() == Before);
    CHECK(Registry.CloseWorkerLedger());
    CHECK(A->GetState() == ERemoteIngressState::Retired && B->GetState() == ERemoteIngressState::Retired);
    CHECK(Registry.ConstructionCount() == 2 && Registry.DestructionCount() == 2);
}
static void RegistryRemoteDefaultsAndMismatchStayClosed()
{
    for (int Case = 0; Case < 8; ++Case)
    {
        auto D = RemoteDescriptor(1); auto Ingress = RemoteService(D);
        auto Commands = std::make_shared<V2::FInputSessionCommands>();
        V2::FInputSessionRegistry Registry(Commands, 7, {}, Case == 0 ? V2::EExactRemoteAdmission::Closed : V2::EExactRemoteAdmission::OriginalDeviceExact);
        const bool Registered = Registry.RegisterJournalAtBoundary({D.Journal, {}, {}, Ingress});
        CHECK(Registered == (Case != 0));
        if (Case == 1) D.Processing.Step[0] = 1;
        if (Case == 2) D.AllowScenarioAppend = true;
        if (Case == 3) D.Scenario.push_back(RemoteFrame(1, 0));
        if (Case == 4) ++D.Producer;
        if (Case == 5) ++D.Epoch;
        if (Case == 6) ++D.First;
        if (Case == 7) ++D.Journal;
        CHECK(Commands->Submit(RemoteBind(D, 1, 0)) == V2::ECommandAdmission::Enqueued);
        CHECK(Registry.ServiceBoundary(0));
        CHECK(Commands->Read(1)->Status == V2::EBoundaryStatus::Rejected);
        CHECK(Registry.ConstructionCount() == 0 && Registry.PollCount(1) == 0);
        CHECK(Ingress->GetState() == ERemoteIngressState::Unbound && !Registry.IsTerminal());
    }
}
int main()
{
    try
    {
        for (auto Test : {RegistryRemoteOptInAndCompleteCohort, RegistryRemoteDefaultsAndMismatchStayClosed,
            ReadinessDoesNotConsumeOrForge, IngressOwnsNoPhysicalSourceBeforeWorkerBind,
            IngressSubmissionSerializesWithOwnerRetirement, ExactFramesAndImmutableCutoff, ForeignCallsCannotReleaseLease,
            FailedCloseQuarantinesAndPauseDoesNotForge, SourceContinuityAndAddress, CurrentOwnerAdmissionStillRefuses,
            FullEdgesAndStaleLeaseSerial, GenerationChangeRequiresFreshReset,
            ExplicitLaterBaselinePreservesSameEpochAndHistory, BaselineInspectionRejectsStaleResetWithoutMutation}) Test();
        std::cout << "PASS checks=" << Checks << " scope=portable_optin_remote_registry_default_closed_no_UE_no_physics\n"; return 0;
    }
    catch (const std::exception& E) { std::cerr << "FAIL " << E.what() << '\n'; return 1; }
}
