#include "AuthoritativeBatch.h"
static void ExplicitAnchorBranchAndParentPublication()
{
    FReceiverFixture F; auto Context = F.Origin.Context; Context.AuthorityTimelineRevision = 8;
    auto D = C::Propose(F.Origin.Decision.Original, Context); CHECK(D);
    D->Stage = C::EStage::Published; D->Publication = F.Origin.Decision.Publication;
    FWorldLedgerFixture Ledger; Ledger.Exact = *C::EncodeDecision(*D);
    CHECK(!F.Receiver->PrepareAuthoritativeAtBoundary(20, *D, Context, Ledger, 101));
    auto Anchor = F.Anchor; Anchor.AuthorityTimelineRevision = 8; CHECK(!A::FPublishedDecisionOwner::Create(F.Binding, Anchor, F.Evidence));
    Anchor = F.Anchor; Anchor.AuthorityBeforeWorldPublication = 15; CHECK(!A::FPublishedDecisionOwner::Create(F.Binding, Anchor, F.Evidence));
    Anchor = F.Anchor; ++Anchor.World.Id; CHECK(!A::FPublishedDecisionOwner::Create(F.Binding, Anchor, F.Evidence));
    Anchor = F.Anchor; ++Anchor.World.Incarnation; CHECK(!A::FPublishedDecisionOwner::Create(F.Binding, Anchor, F.Evidence));
    Context = F.Origin.Context; Context.BeforeWorldPublication = 15;
    D = C::Propose(F.Origin.Decision.Original, Context); CHECK(D); D->Stage = C::EStage::Published; D->Publication = F.Origin.Decision.Publication;
    Ledger.Exact = *C::EncodeDecision(*D); CHECK(!F.Receiver->PrepareAuthoritativeAtBoundary(20, *D, Context, Ledger, 101));
    Context = F.Origin.Context; D = F.Origin.Decision; D->Publication.Serial = Context.BeforeWorldPublication;
    Ledger.Exact = *C::EncodeDecision(*D); CHECK(!F.Receiver->PrepareAuthoritativeAtBoundary(20, *D, Context, Ledger, 101));
    // A local replay pass may change while origin authoritative branch stays7.
    Anchor = F.Anchor; Anchor.TimelineRevision = 9; Anchor.LocalWorldPublication = 200;
    auto LocalReplay = A::FPublishedDecisionOwner::Create(F.Binding, Anchor, F.Evidence); CHECK(LocalReplay);
    auto T = LocalReplay->PrepareAuthoritativeAtBoundary(20, F.Origin.Decision, F.Origin.Context, F.Origin.Ledger, 201); CHECK(T);
    CHECK(LocalReplay->Install(*T) && LocalReplay->BeginStep(*T, 1));
    FLocalPublicationFixture Local; Local.Published = true; Local.Revision = 9; Local.Serial = 201;
    CHECK(LocalReplay->CompleteStepAfterWorldPublication(*T, Local));
    auto R = LocalReplay->ReadLatest(); CHECK(R && R->TimelineRevision == 9 && R->AuthoritativeEntry->AcceptedDecision->AuthorityTimelineRevision == 7);
}
static void MixedSourceBranchesRefusedBeforeGroupInstall()
{
    FDecisionFixture F;
    auto E1 = C::PrepareLocalDecisionEntry(20, F.Decision, F.Context, F.Ledger, F.Contract, {1}); CHECK(E1);
    auto T2 = F.Context; T2.Binding.Session = 2;
    T2.Processing.BindingFingerprint = C::OriginBindingFingerprintFor(*F.Contract, T2);
    auto P2 = F.Decision.Original; P2.Session = 2;
    auto D2 = C::Propose(P2, T2); CHECK(D2); D2->Stage = C::EStage::Published; D2->Publication = F.Decision.Publication;
    FWorldLedgerFixture L2; L2.Exact = *C::EncodeDecision(*D2);
    auto E2 = C::PrepareLocalDecisionEntry(20, *D2, T2, L2, F.Contract, {2}); CHECK(E2);
    A::FAuthorityBatchAddress Address{20, 3, 7, 1, 9, 16, 17, 12345, {700, 4}};
    CHECK(A::ValidateAuthoritativeInputBatch(Address, {1,2}, {E1,E2}));
    auto WrongOrigin = std::make_shared<C::FPreparedAuthoritativeInput>(*E1);
    auto WrongContext = std::make_shared<C::FTrustedContext>(*E1->OriginContext); ++WrongContext->AuthorityEpoch;
    WrongOrigin->OriginContext = WrongContext; CHECK(!A::ValidateAuthoritativeInputBatch(Address, {1,2}, {WrongOrigin,E2}));
    WrongContext->AuthorityEpoch = E1->OriginContext->AuthorityEpoch; ++WrongContext->Binding.ServerPeer;
    CHECK(!A::ValidateAuthoritativeInputBatch(Address, {1,2}, {WrongOrigin,E2}));
    auto Forged = std::make_shared<C::FPreparedAuthoritativeInput>(*E1);
    auto WrongSnapshot = std::make_shared<V::FOwnerInputSnapshot>(*E1->LocalSnapshot); WrongSnapshot->Applied[0] = 255;
    Forged->LocalSnapshot = WrongSnapshot; CHECK(!A::ValidateAuthoritativeInputBatch(Address, {1,2}, {Forged,E2}));
    WrongSnapshot->Applied = E1->LocalSnapshot->Applied; WrongSnapshot->BindingFingerprint.clear();
    CHECK(!A::ValidateAuthoritativeInputBatch(Address, {1,2}, {Forged,E2}));
    T2.AuthorityTimelineRevision = 8; D2->AuthorityTimelineRevision = 8; L2.Exact = *C::EncodeDecision(*D2);
    E2 = C::PrepareLocalDecisionEntry(20, *D2, T2, L2, F.Contract, {2}); CHECK(E2);
    CHECK(!A::ValidateAuthoritativeInputBatch(Address, {1,2}, {E1,E2})); // individually valid, wrong common world branch
    CHECK(!A::ValidateAuthoritativeInputBatch(Address, {1}, {E1,E2}));
    CHECK(!A::ValidateAuthoritativeInputBatch(Address, {2,1}, {E2,E1}));
    CHECK(!A::ValidateAuthoritativeInputBatch(Address, {1,1}, {E1,E1}));
    CHECK(!A::ValidateAuthoritativeInputBatch(Address, {1,2}, {E1,{}}));
    Address.AuthorityTimelineRevision = 8; CHECK(!A::ValidateAuthoritativeInputBatch(Address, {1,2}, {E1,E2}));
    CHECK(F.Raw->Next == 2 && F.Raw->Polls == 2);
}
int main()
{
    if (OwnerV2RegressionMain()) return 1;
    try { ExplicitAnchorBranchAndParentPublication(); MixedSourceBranchesRefusedBeforeGroupInstall(); std::cout << "PASS " << Checks << " checks; authoritative wire v2/anchor/batch branch identity; local revision distinct; native closed\n"; return 0; }
    catch (const std::exception& E) { Budget = -1; std::cerr << "FAIL " << Checks << " " << E.what() << '\n'; return 1; }
}
