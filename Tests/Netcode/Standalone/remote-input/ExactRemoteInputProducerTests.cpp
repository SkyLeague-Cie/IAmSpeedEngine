#include "ExactRemoteInputProducer.h"
#include "IAmSpeed/Input/SameFrameInputOwner.h"
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
int main()
{
    try
    {
        for (auto Test : {ExactFramesAndImmutableCutoff, ForeignCallsCannotReleaseLease,
            FailedCloseQuarantinesAndPauseDoesNotForge, SourceContinuityAndAddress, CurrentOwnerAdmissionStillRefuses,
            FullEdgesAndStaleLeaseSerial, GenerationChangeRequiresFreshReset,
            ExplicitLaterBaselinePreservesSameEpochAndHistory, BaselineInspectionRejectsStaleResetWithoutMutation}) Test();
        std::cout << "PASS checks=" << Checks << " scope=portable_V2_remote_producer_fence_no_owner_admission_no_UE_no_physics\n"; return 0;
    }
    catch (const std::exception& E) { std::cerr << "FAIL " << E.what() << '\n'; return 1; }
}
