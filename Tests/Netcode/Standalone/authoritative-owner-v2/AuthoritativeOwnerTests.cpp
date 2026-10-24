#define main RetainedOwnerRegressionMain
#include "../retained-replay/ReplayStartTests.cpp"
#undef main
#include "../consumed-command-v2/OwnerSnapshotBridge.h"
namespace N = Speed::Netcode;
namespace C = Speed::Netcode::Consumed;
struct FWorldLedgerFixture final : C::IPublishedWorldEvidence
{
    std::vector<std::uint8_t> Exact;
    mutable unsigned Consultations = 0;
    bool Confirms(const C::FDecision&, const std::vector<std::uint8_t>& B) const noexcept override
    { ++Consultations; return B == Exact; }
}; // Synthetic complete-world publication ledger ONLY, not a native provider.
struct FDecisionFixture
{
    std::shared_ptr<const V::FInputActionContract> Contract;
    std::shared_ptr<FRawReplayFixture> Raw = std::make_shared<FRawReplayFixture>();
    std::unique_ptr<V::FSameFrameInputOwner> Owner;
    C::FTrustedContext Context;
    C::FDecision Decision;
    FWorldLedgerFixture Ledger;
    FDecisionFixture()
    {
        V::FInputActionContractDescription D; D.Revision.Value = 23; D.Actions = V::FInputActionContract::BaseActions();
        D.Mapping.push_back({{V::ERawControlKind::PadAxis, std::uint16_t(V::EPadAxis::LeftTrigger)}, 0, 1, {}});
        D.Physical = {{0, V::EPhysicalDestination::Throttle}, {1, V::EPhysicalDestination::Brake}, {2, V::EPhysicalDestination::Steering}};
        Contract = V::FInputActionContract::Create(std::move(D)); CHECK(Contract);
        auto Device = V::FDeviceInputProducer::Create(Raw, Contract, {7}, {I::EProducerKind::Device, 99}, 0); CHECK(Device);
        auto B = Binding(); B.Contract = Contract; B.ProducerContract = V::EProducerContract::Device; B.Processing.Step[0] = 10;
        Owner = V::FSameFrameInputOwner::Create(std::move(Device), B, 0); CHECK(Owner);
        Complete(*Owner, 0); const auto Before = Owner->ReadBoundaryApplied();
        Complete(*Owner, 1); const auto S = Owner->ReadCompleted(1)->Snapshot;
        N::FBinding NB; NB.Session = B.AdapterId; NB.Epoch = 7; NB.Actor = 100; NB.Incarnation = 2;
        NB.OwnerPeer = 10; NB.ServerPeer = 20; NB.SourceProducer = 99; NB.SourceKind = 0; NB.Contract[0] = 23;
        Context.Binding = NB; Context.Frame = 1; Context.RegistryVersion = 9; Context.AuthorityEpoch = 3; Context.AuthorityTimelineRevision = 7; Context.BeforeWorldPublication = 16; Context.World = {700, 4};
        Context.Targets = {{100, 2}, {200, 3}}; Context.Actions = C::Policies(*Contract);
        Context.Processing.Origin = C::EOrigin::Device; Context.Processing.Step[0] = 10; Context.Processing.Before = Before;
        Context.Processing.BindingFingerprint = S->BindingFingerprint;
        CHECK(Context.Processing.BindingFingerprint == C::OriginBindingFingerprintFor(*Contract, Context));
        N::FPacket P; P.Kind = N::EKind::OwnerInput; P.Session = NB.Session; P.Epoch = NB.Epoch; P.Actor = NB.Actor; P.Incarnation = NB.Incarnation;
        P.Contract = NB.Contract; P.Sequence = 2; P.Frame = 1;
        const auto& O = S->Input.GetData(); P.Input.SourceSequence = O.SourceSequence; P.Input.DeviceGeneration = O.DeviceGeneration.Value;
        P.Input.ProducerId = O.Producer.Id; P.Input.ProducerKind = std::uint8_t(O.Producer.Kind); P.Input.Values = O.Values;
        P.Input.ActiveMask = O.ActiveMask; P.Input.Reset = O.Reset;
        for (const auto& E : O.Transitions) P.Input.Edges.push_back({E.Action, std::uint8_t(E.State), E.ValueAtTransition, E.Order.Sequence, E.Order.WithinSequence});
        auto Proposal = C::Propose(P, Context); CHECK(Proposal); Decision = *Proposal;
        CHECK(Decision.Applied == S->Applied && Decision.Applied[0] == 74 && Decision.Original.Input.Values[0] == 255);
        CHECK(Decision.Stage == C::EStage::Proposed && Decision.Publication.Empty());
        Decision.Stage = C::EStage::Published; Decision.Publication = {20, 3, 17, 12345};
        auto Encoded = C::EncodeDecision(Decision); CHECK(Encoded); Ledger.Exact = *Encoded;
    }
};
static void CompleteDecisionPreservesRequestedAndApplied()
{
    FDecisionFixture F; const auto History = F.Owner->ReadCompleted(1); const auto Poll = F.Owner->GetPollSerial();
    auto Bytes = C::EncodeDecision(F.Decision); CHECK(Bytes); auto D = C::DecodeDecision(*Bytes); CHECK(D);
    CHECK(C::EncodeDecision(*D) == Bytes && C::MatchesTrusted(*D, F.Context));
    C::IPublishedWorldEvidence Default; CHECK(!C::AcceptPublished(20, *D, F.Context, Default));
    auto Accepted = C::AcceptPublished(20, *D, F.Context, F.Ledger); CHECK(Accepted);
    const V::FContractFingerprint LocalBinding{1, 2, 3};
    auto Entry = C::PrepareLocalDecisionEntry(20, *D, F.Context, F.Ledger, F.Contract, LocalBinding); CHECK(Entry); auto S = Entry->LocalSnapshot; CHECK(S); CHECK(Entry->AcceptedDecision && Entry->OriginContext);
    CHECK(S->Requested[0] == 255 && S->Applied[0] == 74 && S->Input.GetData().Values[0] == 255);
    CHECK(S->Input.GetData().ActiveMask == History->Snapshot->Input.GetData().ActiveMask);
    CHECK(S->Input.GetData().Transitions.size() == History->Snapshot->Input.GetData().Transitions.size());
    for (std::size_t I = 0; I < S->Input.GetData().Transitions.size(); ++I)
    {
        const auto& A = S->Input.GetData().Transitions[I]; const auto& B = History->Snapshot->Input.GetData().Transitions[I];
        CHECK(A.Action == B.Action && A.State == B.State && A.ValueAtTransition == B.ValueAtTransition
            && A.Order.Sequence == B.Order.Sequence && A.Order.WithinSequence == B.Order.WithinSequence);
    }
    CHECK(S->CutoffSequence == History->Snapshot->CutoffSequence && S->BindingFingerprint == LocalBinding);
    CHECK(F.Raw->Next == 2 && F.Raw->Freezes == 2 && F.Raw->Polls == 2 && F.Raw->Closes == 2);
    CHECK(F.Owner->GetPollSerial() == Poll && F.Owner->ReadCompleted(1) == History);
    Complete(*F.Owner, 2); CHECK(F.Owner->ReadCompleted(2)->Snapshot->Applied[0] == 84);
    // No installation, historical native replay or local completed receipt is minted here.
    CHECK(!C::NativeEndpointQualified);
}
static void AdversarialContextsAndPublicationStages()
{
    FDecisionFixture F;
    const auto Reject = [&](const C::FDecision& D, const C::FTrustedContext& Context)
    { CHECK(!C::AcceptPublished(20, D, Context, F.Ledger)); };
    auto D = F.Decision; D.Stage = C::EStage::Proposed; D.Publication = {}; Reject(D, F.Context);
    CHECK(C::EncodeDecision(D)); D.Publication.Serial = 1; CHECK(!C::EncodeDecision(D));
    D = F.Decision; D.Publication.WorldStateHash = 0; CHECK(!C::EncodeDecision(D)); Reject(D, F.Context);
    D = F.Decision; ++D.Publication.Serial; Reject(D, F.Context); // structurally valid but absent ledger
    D = F.Decision; ++D.Publication.AuthorityEpoch; Reject(D, F.Context);
    D = F.Decision; ++D.Publication.AuthorityPeer; Reject(D, F.Context);
    CHECK(!C::AcceptPublished(10, F.Decision, F.Context, F.Ledger));
    D = F.Decision; ++D.Applied[0]; Reject(D, F.Context);
    D = F.Decision; ++D.Original.Input.Values[0]; Reject(D, F.Context);
    D = F.Decision; ++D.Original.Input.DeviceGeneration; Reject(D, F.Context); // exact ledger catches metadata drift
    D = F.Decision; ++D.Original.Input.SourceSequence; Reject(D, F.Context);
    D = F.Decision; ++D.Original.Epoch; Reject(D, F.Context);
    D = F.Decision; ++D.Original.Incarnation; Reject(D, F.Context);
    D = F.Decision; ++D.RegistryVersion; Reject(D, F.Context);
    D = F.Decision; D.Targets.pop_back(); Reject(D, F.Context);
    D = F.Decision; std::reverse(D.Targets.begin(), D.Targets.end()); CHECK(!C::EncodeDecision(D));
    D = F.Decision; D.Targets[1].Actor = D.Targets[0].Actor; CHECK(!C::EncodeDecision(D));
    D = F.Decision; ++(*D.Processing.Before)[0]; Reject(D, F.Context);
    D = F.Decision; ++D.Processing.Step[0]; Reject(D, F.Context);
    D = F.Decision; D.Processing.BindingFingerprint[0] ^= 1; Reject(D, F.Context);
    auto T = F.Context; T.Processing.Before.reset(); CHECK(!C::Propose(F.Decision.Original, T));
    T = F.Context; T.Processing.Step[0] = 511; CHECK(!C::Propose(F.Decision.Original, T));
    T = F.Context; T.Processing.Step[3] = 1; CHECK(!C::Propose(F.Decision.Original, T));
    T = F.Context; T.Actions[0].Axis = false; CHECK(!C::Propose(F.Decision.Original, T));
    T = F.Context; T.Processing.Before->at(0) = -1; CHECK(!C::Propose(F.Decision.Original, T));
    T = F.Context; T.AuthorityEpoch = 0; Reject(F.Decision, T);
    T = F.Context; ++T.RegistryVersion; Reject(F.Decision, T);
    T = F.Context; T.Actions[0].Quantization = 127;
    CHECK(!C::PrepareLocalDecisionEntry(20, F.Decision, T, F.Ledger, F.Contract, {1}));
    CHECK(!C::PrepareLocalDecisionEntry(20, F.Decision, F.Context, F.Ledger, F.Contract, {}));
    auto P = F.Decision.Original; P.Input.ActiveMask |= (1u << 31); CHECK(!C::Propose(P, F.Context));
    P = F.Decision.Original; P.Input.Edges = {{31, 0, 1, P.Input.SourceSequence, 0}}; P.Input.ActiveMask |= 1u << 31; CHECK(!C::Propose(P, F.Context));
    P = F.Decision.Original; P.Input.Edges = {{0, 0, -1, P.Input.SourceSequence, 0}}; P.Input.ActiveMask |= 1u; CHECK(!C::Propose(P, F.Context));
}
static void ExactResetAndAuthoritativeBefore()
{
    FDecisionFixture F; auto P = F.Decision.Original;
    P.Input.Reset = true; P.Input.Edges.clear();
    auto D = C::Propose(P, F.Context); CHECK(D && D->Applied == P.Input.Values && D->Applied[0] == 255);
    auto T = F.Context; T.Processing.Before->at(0) = 100;
    D = C::Propose(F.Decision.Original, T); CHECK(D && D->Applied[0] == 110);
    CHECK(!C::MatchesTrusted(*D, F.Context)); // untrusted client Before is never installed
    for (auto Origin : {C::EOrigin::ExactScenario, C::EOrigin::AI})
    {
        T = F.Context; T.Processing.Origin = Origin; T.Processing.Before.reset(); T.Processing.Step.fill(0);
        T.Binding.SourceKind = Origin == C::EOrigin::AI ? 1 : 0;
        P = F.Decision.Original; P.Input.ProducerKind = T.Binding.SourceKind;
        D = C::Propose(P, T); CHECK(D && D->Applied == P.Input.Values);
        T.Processing.Step[0] = 1; CHECK(!C::Propose(P, T));
    }
    T = F.Context; T.Processing.Before->at(2) = -120; T.Processing.Step[2] = 10;
    P = F.Decision.Original; P.Input.Values[2] = 127;
    D = C::Propose(P, T); CHECK(D && D->Applied[2] == -110);
    T.Processing.Before->at(2) = 120; P.Input.Values[2] = -127;
    D = C::Propose(P, T); CHECK(D && D->Applied[2] == 110);
}
static void BoundedParserAndAllocationFailures()
{
    FDecisionFixture F; const auto Bytes = *C::EncodeDecision(F.Decision);
    for (std::size_t N = 0; N < Bytes.size(); ++N)
        CHECK(!C::DecodeDecision(std::vector<std::uint8_t>(Bytes.begin(), Bytes.begin() + N)));
    auto B = Bytes; B.push_back(0); CHECK(!C::DecodeDecision(B));
    for (std::size_t I : {0u, 4u, 6u, 7u}) { B = Bytes; B[I] = 0xff; CHECK(!C::DecodeDecision(B)); }
    B = Bytes; for (unsigned I = 48; I < 52; ++I) B[I] = 0xff; CHECK(!C::DecodeDecision(B));
    B.assign(C::MaxPayload + 1, 0); CHECK(!C::DecodeDecision(B));
    auto D = F.Decision; D.Targets.resize(C::MaxTargets + 1); CHECK(!C::EncodeDecision(D));
    D = F.Decision; D.Processing.BindingFingerprint.resize(C::MaxFingerprint + 1); CHECK(!C::EncodeDecision(D));
    const auto Latest = F.Owner->ReadLatest(); const auto Poll = F.Owner->GetPollSerial();
    for (int Limit = 0; Limit < 36; ++Limit)
    {
        Budget = Limit;
        try { (void)C::PrepareLocalDecisionEntry(20, F.Decision, F.Context, F.Ledger, F.Contract, {1}); }
        catch (const std::bad_alloc&) {}
        Budget = -1;
        CHECK(F.Owner->ReadLatest() == Latest && F.Owner->GetPollSerial() == Poll && F.Raw->Next == 2);
    }
}
struct FReentrantEvidence final : C::IPublishedWorldEvidence
{
    C::FDecision* CallerDecision = nullptr;
    C::FTrustedContext* CallerContext = nullptr;
    V::FContractFingerprint* CallerDestination = nullptr;
    std::vector<std::uint8_t> Exact;
    bool Confirms(const C::FDecision&, const std::vector<std::uint8_t>& Bytes) const noexcept override
    {
        const bool WasExact = Bytes == Exact;
        if (CallerDecision) { CallerDecision->Applied[0] = 255; CallerDecision->Original.Input.Values[0] = 1; }
        if (CallerContext) { CallerContext->Targets.clear(); CallerContext->Processing.Before.reset(); }
        if (CallerDestination) CallerDestination->clear();
        return WasExact;
    }
};
static void ImmutablePinsSurviveReentrantEvidence()
{
    FDecisionFixture F; auto D = F.Decision; auto T = F.Context; const auto OriginalContext = T;
    V::FContractFingerprint Destination{7, 8, 9};
    FReentrantEvidence E; E.CallerDecision = &D; E.CallerContext = &T; E.CallerDestination = &Destination; E.Exact = F.Ledger.Exact;
    auto Accepted = C::AcceptPublished(20, D, T, E); CHECK(Accepted);
    CHECK(Accepted->Applied[0] == 74 && Accepted->Original.Input.Values[0] == 255 && C::MatchesTrusted(*Accepted, OriginalContext));
    CHECK(D.Applied[0] == 255 && D.Original.Input.Values[0] == 1 && T.Targets.empty() && Destination.empty());
    D = F.Decision; T = F.Context; Destination = {7, 8, 9};
    auto Entry = C::PrepareLocalDecisionEntry(20, D, T, E, F.Contract, Destination); CHECK(Entry);
    CHECK(Entry->AcceptedDecision->Applied[0] == 74 && Entry->AcceptedDecision->Original.Input.Values[0] == 255);
    CHECK(Entry->LocalSnapshot->Requested[0] == 255 && Entry->LocalSnapshot->Applied[0] == 74);
    CHECK(Entry->LocalSnapshot->BindingFingerprint == V::FContractFingerprint({7, 8, 9}));
    CHECK(Entry->OriginContext->Targets == OriginalContext.Targets && Entry->OriginContext->Processing == OriginalContext.Processing);
    CHECK(C::MatchesTrusted(*Entry->AcceptedDecision, *Entry->OriginContext));
    CHECK(!Entry->AcceptedDecision->Publication.Empty() && !Entry->LocalSnapshot->Input.GetData().Reset);
}
int ConsumedRegressionMain()
{
    if (RetainedOwnerRegressionMain()) return 1;
    try
    {
        CompleteDecisionPreservesRequestedAndApplied(); AdversarialContextsAndPublicationStages();
        ExactResetAndAuthoritativeBefore(); BoundedParserAndAllocationFailures(); ImmutablePinsSurviveReentrantEvidence();
        std::cout << "PASS " << Checks << " checks; versioned requested/applied decision; real Device source; synthetic world ledger; no native endpoint\n"; return 0;
    }
    catch (const std::exception& E) { Budget = -1; std::cerr << "FAIL " << Checks << " " << E.what() << '\n'; return 1; }
}


#include "AuthoritativeOwnerTests.inc"
