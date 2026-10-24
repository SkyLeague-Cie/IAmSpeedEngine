#include "AuthoritativeDecisionOwner.h"
namespace A = Speed::Netcode::Authoritative;
struct FAnchorEvidenceFixture final : A::IReceiverRestoreEvidence
{
    bool Confirms(const A::FReceiverBinding&, const A::FRestoreAnchor&) const noexcept override { return true; }
}; // Synthetic restore evidence ONLY, native complete-world provider absent.
struct FLocalPublicationFixture final : A::ILocalWorldPublication
{
    bool Published = false;
    N::U64 Receiver = 500, Frame = 1, Revision = 2, Serial = 101;
    A::FPublishedDecisionOwner* Reenter = nullptr;
    const A::FPublishedDecisionOwner::FToken* Token = nullptr;
    mutable bool Reentered = false;
    bool Confirms(N::U64 R, N::U64 N, N::U64 V, N::U64 S, const C::FPreparedAuthoritativeInput& E) const noexcept override
    {
        if (Reenter && Token) Reentered = !Reenter->AbortBeforeStep(*Token) && !Reenter->FaultAfterStep(*Token);
        return Published && R == Receiver && N == Frame && V == Revision && S == Serial && E.AcceptedDecision && E.LocalSnapshot;
    }
};
static N::FPacket PacketFromSnapshot(const N::FPacket& Template, const V::FOwnerInputSnapshot& S)
{
    auto P = Template; const auto& D = S.Input.GetData(); P.Frame = D.ConsumptionFrame; P.Sequence = D.ConsumptionFrame + 1;
    P.Input.Values = D.Values; P.Input.ActiveMask = D.ActiveMask; P.Input.Reset = D.Reset;
    P.Input.SourceSequence = D.SourceSequence; P.Input.DeviceGeneration = D.DeviceGeneration.Value; P.Input.ProducerId = D.Producer.Id;
    P.Input.ProducerKind = std::uint8_t(D.Producer.Kind); P.Input.Edges.clear();
    for (const auto& E : D.Transitions) P.Input.Edges.push_back({E.Action, std::uint8_t(E.State), E.ValueAtTransition, E.Order.Sequence, E.Order.WithinSequence});
    return P;
}
struct FReceiverFixture
{
    FDecisionFixture Origin;
    A::FReceiverBinding Binding;
    A::FRestoreAnchor Anchor;
    FAnchorEvidenceFixture Evidence;
    std::unique_ptr<A::FPublishedDecisionOwner> Receiver;
    explicit FReceiverFixture(A::ERole Role = A::ERole::OwnerClient)
    {
        Binding.ReceiverId = 500; Binding.ReceiverPeer = Role == A::ERole::OwnerClient ? 10 : 30;
        Binding.WorkerGeneration = 6; Binding.LocalEpoch = 100; Binding.Role = Role;
        Binding.Origin = Origin.Context; Binding.Contract = Origin.Contract; Binding.DestinationFingerprint = {9, 8, 7};
        Anchor.World = Origin.Context.World; Anchor.AuthorityTimelineRevision = Origin.Context.AuthorityTimelineRevision; Anchor.AuthorityBeforeWorldPublication = Origin.Context.BeforeWorldPublication;
        Anchor.Next = 1; Anchor.TimelineRevision = 2; Anchor.LocalWorldPublication = 100; Anchor.LocalWorldHash = 444;
        Anchor.AppliedBefore = Origin.Context.Processing.Before;
        Anchor.PreviousOriginal = PacketFromSnapshot(Origin.Decision.Original, *Origin.Owner->ReadCompleted(0)->Snapshot);
        Receiver = A::FPublishedDecisionOwner::Create(Binding, Anchor, Evidence); CHECK(Receiver);
    }
    auto Prepare(N::U64 Serial = 101) { return Receiver->PrepareAuthoritativeAtBoundary(20, Origin.Decision, Origin.Context, Origin.Ledger, Serial); }
};
static void RealDecisionEndpointAppliesAndCompletesLocally()
{
    for (auto Role : {A::ERole::OwnerClient, A::ERole::RemoteClient})
    {
        FReceiverFixture F(Role); auto T = F.Prepare(); CHECK(T && !F.Receiver->ReadLatest());
        CHECK(F.Receiver->GetPhase() == A::EPhase::Prepared && !F.Prepare());
        CHECK(F.Receiver->Install(*T)); auto Entry = F.Receiver->ReadInstalled(*T); CHECK(Entry);
        CHECK(Entry->AcceptedDecision->Applied[0] == 74 && Entry->LocalSnapshot->Applied[0] == 74 && Entry->LocalSnapshot->Requested[0] == 255);
        CHECK(Entry->AcceptedDecision->Original.Input.ProducerId == 99 && Entry->AcceptedDecision->Original.Epoch == 7);
        CHECK(!F.Receiver->ReadCompleted(1)); CHECK(F.Receiver->BeginStep(*T, 1)); CHECK(!F.Receiver->AbortBeforeStep(*T));
        A::ILocalWorldPublication Missing; CHECK(!F.Receiver->CompleteStepAfterWorldPublication(*T, Missing));
        FLocalPublicationFixture Local; CHECK(!F.Receiver->CompleteStepAfterWorldPublication(*T, Local));
        CHECK(!F.Receiver->ReadLatest() && F.Receiver->GetNext() == 1);
        Local.Published = true; Local.Reenter = F.Receiver.get(); Local.Token = &*T;
        Allocations = 0; Budget = 0; const bool Done = F.Receiver->CompleteStepAfterWorldPublication(*T, Local); Budget = -1;
        CHECK(Done && Local.Reentered && !Allocations && F.Receiver->GetNext() == 2);
        auto Receipt = F.Receiver->ReadCompleted(1); CHECK(Receipt && Receipt == F.Receiver->ReadLatest());
        CHECK(Receipt->AuthoritativeEntry == Entry && Receipt->TimelineRevision == 2 && Receipt->Serial == 1 && Receipt->LocalWorldPublication == 101);
        CHECK(Receipt->AuthoritativeEntry->AcceptedDecision->Publication.Serial == 17 && Receipt->LocalEpoch == 100);
        CHECK(!F.Receiver->CompleteStepAfterWorldPublication(*T, Local) && !F.Receiver->AbortBeforeStep(*T));
        CHECK(!F.Prepare(102)); // old origin frame cannot be physically applied twice
        Complete(*F.Origin.Owner, 2); const auto Source2 = F.Origin.Owner->ReadCompleted(2)->Snapshot;
        auto Context2 = F.Origin.Context; Context2.Frame = 2; Context2.BeforeWorldPublication = F.Origin.Decision.Publication.Serial; Context2.Processing.Before = F.Origin.Decision.Applied;
        auto D2 = C::Propose(PacketFromSnapshot(F.Origin.Decision.Original, *Source2), Context2); CHECK(D2 && D2->Applied[0] == 84);
        D2->Stage = C::EStage::Published; D2->Publication = {20, 3, 18, 12346}; FWorldLedgerFixture Ledger2; Ledger2.Exact = *C::EncodeDecision(*D2);
        T = F.Receiver->PrepareAuthoritativeAtBoundary(20, *D2, Context2, Ledger2, 102); CHECK(T);
        CHECK(F.Receiver->Install(*T) && F.Receiver->BeginStep(*T, 2)); Local.Frame = 2; Local.Serial = 102;
        CHECK(F.Receiver->CompleteStepAfterWorldPublication(*T, Local));
        CHECK(F.Receiver->ReadCompleted(1) == Receipt && F.Receiver->ReadLatest()->AuthoritativeEntry->LocalSnapshot->Applied[0] == 84);
        CHECK(F.Origin.Raw->Next == 3 && F.Origin.Raw->Polls == 3 && F.Origin.Owner->GetPollSerial() == 3);
    }
}
static void EndpointRefusalsAbortAndNewRevision()
{
    FReceiverFixture F; A::IReceiverRestoreEvidence Missing;
    CHECK(!A::FPublishedDecisionOwner::Create(F.Binding, F.Anchor, Missing));
    auto B = F.Binding; B.Role = A::ERole::RemoteClient; CHECK(!A::FPublishedDecisionOwner::Create(B, F.Anchor, F.Evidence));
    B = F.Binding; B.ReceiverPeer = 20; CHECK(!A::FPublishedDecisionOwner::Create(B, F.Anchor, F.Evidence));
    auto Anchor = F.Anchor; Anchor.AppliedBefore->at(0) = 40; CHECK(!A::FPublishedDecisionOwner::Create(F.Binding, Anchor, F.Evidence));
    B = F.Binding; ++B.Origin.Frame; CHECK(!A::FPublishedDecisionOwner::Create(B, F.Anchor, F.Evidence));
    B = F.Binding; B.WorkerGeneration = 0; CHECK(!A::FPublishedDecisionOwner::Create(B, F.Anchor, F.Evidence));
}
static void EndpointAllocationAndContinuityFailures()
{
    FReceiverFixture F; auto Latest = F.Receiver->ReadLatest();
    for (int Limit = 0; Limit < 48; ++Limit)
    {
        Budget = Limit; auto T = F.Prepare(); Budget = -1;
        if (T) CHECK(F.Receiver->AbortBeforeStep(*T));
        CHECK(F.Receiver->GetNext() == 1 && F.Receiver->ReadLatest() == Latest && F.Receiver->GetPhase() == A::EPhase::Idle);
    }
    auto T = F.Prepare(); CHECK(T && F.Receiver->Install(*T)); CHECK(F.Receiver->AbortBeforeStep(*T));
    CHECK(!F.Receiver->Install(*T)); T = F.Prepare(); CHECK(T);
    bool Foreign = true; std::thread Other([&] { Foreign = F.Receiver->AbortBeforeStep(*T); }); Other.join(); CHECK(!Foreign);
    { I::FPresentationInputScope Scope; CHECK(!F.Receiver->Install(*T) && !F.Receiver->AbortBeforeStep(*T)); }
    CHECK(F.Receiver->AbortBeforeStep(*T));
    const auto RejectOrigin = [&](N::FPacket P)
    {
        auto D = C::Propose(P, F.Origin.Context); CHECK(D);
        D->Stage = C::EStage::Published; D->Publication = F.Origin.Decision.Publication;
        FWorldLedgerFixture Ledger; Ledger.Exact = *C::EncodeDecision(*D);
        CHECK(!F.Receiver->PrepareAuthoritativeAtBoundary(20, *D, F.Origin.Context, Ledger, 101));
    };
    auto P = F.Origin.Decision.Original; ++P.Input.DeviceGeneration; RejectOrigin(P);
    P = F.Origin.Decision.Original; P.Sequence = F.Anchor.PreviousOriginal->Sequence; RejectOrigin(P);
    P = F.Origin.Decision.Original; P.Input.SourceSequence = 1; P.Input.Edges.clear(); RejectOrigin(P);
    P = F.Origin.Decision.Original; P.Input.Edges = {{0, 0, 255, P.Input.SourceSequence, 0}}; P.Input.ActiveMask |= 1; RejectOrigin(P);
    CHECK(!F.Prepare(100) && !F.Prepare(99));
    for (int Limit = 0; Limit < 40; ++Limit)
    {
        Budget = Limit; auto Candidate = A::FPublishedDecisionOwner::Create(F.Binding, F.Anchor, F.Evidence); Budget = -1;
        if (Candidate) CHECK(Candidate->GetNext() == 1 && !Candidate->ReadLatest());
        CHECK(F.Receiver->GetNext() == 1 && !F.Receiver->ReadLatest());
    }
    auto Context = F.Origin.Context; Context.Processing.Before->at(0) = 40; Context.AuthorityTimelineRevision = 8; Context.BeforeWorldPublication = 18;
    auto D = C::Propose(F.Origin.Decision.Original, Context); CHECK(D && D->Applied[0] == 50);
    D->Stage = C::EStage::Published; D->Publication = {20, 3, 19, 56789}; FWorldLedgerFixture Ledger; Ledger.Exact = *C::EncodeDecision(*D);
    CHECK(!F.Receiver->PrepareAuthoritativeAtBoundary(20, *D, Context, Ledger, 101)); // needs explicit new complete restore anchor
    auto B = F.Binding; B.Origin = Context; auto Anchor = F.Anchor; Anchor.AppliedBefore = Context.Processing.Before;
    Anchor.AuthorityTimelineRevision = Context.AuthorityTimelineRevision; Anchor.AuthorityBeforeWorldPublication = Context.BeforeWorldPublication;
    Anchor.TimelineRevision = 3; Anchor.LocalWorldPublication = 200; Anchor.PreviousOriginal->Input.Values[0] = 40;
    auto Corrected = A::FPublishedDecisionOwner::Create(B, Anchor, F.Evidence); CHECK(Corrected);
    T = Corrected->PrepareAuthoritativeAtBoundary(20, *D, Context, Ledger, 201); CHECK(T && Corrected->Install(*T));
    CHECK(Corrected->ReadInstalled(*T)->LocalSnapshot->Applied[0] == 50 && F.Origin.Owner->ReadCompleted(1)->Snapshot->Applied[0] == 74);
    CHECK(Corrected->BeginStep(*T, 1)); CHECK(!Corrected->AbortBeforeStep(*T)); CHECK(Corrected->FaultAfterStep(*T));
    CHECK(Corrected->GetPhase() == A::EPhase::Faulted && !Corrected->ReadLatest() && !Corrected->ReadCompleted(1));
    CHECK(F.Origin.Raw->Next == 2 && F.Origin.Raw->Closes == 2);
}
int OwnerV2RegressionMain()
{
    if (ConsumedRegressionMain()) return 1;
    try
    {
        RealDecisionEndpointAppliesAndCompletesLocally(); EndpointRefusalsAbortAndNewRevision(); EndpointAllocationAndContinuityFailures();
        std::cout << "PASS " << Checks << " checks; explicit authoritative decision endpoint; synthetic local world/anchor providers; no native integration\n"; return 0;
    }
    catch (const std::exception& E) { Budget = -1; std::cerr << "FAIL " << Checks << " " << E.what() << '\n'; return 1; }
}

#include "BranchTests.inc"
