static void ExplicitAuthorityBranchAndPrecedingWorldPublication()
{
    FDecisionFixture F; auto Bytes = C::EncodeDecision(F.Decision); CHECK(Bytes && Bytes->at(4) == 2);
    auto D = C::DecodeDecision(*Bytes); CHECK(D && D->AuthorityTimelineRevision == 7 && D->BeforeWorldPublication == 16);
    auto Context = F.Context; Context.AuthorityTimelineRevision = 8;
    CHECK(!C::MatchesTrusted(*D, Context) && !C::AcceptPublished(20, *D, Context, F.Ledger));
    auto Other = *D; Other.AuthorityTimelineRevision = 8;
    CHECK(!C::MatchesTrusted(Other, F.Context)); // new serial/hash alone cannot select a branch
    FWorldLedgerFixture OtherLedger; OtherLedger.Exact = *C::EncodeDecision(Other);
    CHECK(C::AcceptPublished(20, Other, Context, OtherLedger)); // individually valid on another branch
    CHECK(!C::AcceptPublished(20, Other, F.Context, OtherLedger));
    Other = *D; Other.BeforeWorldPublication = 15; CHECK(!C::MatchesTrusted(Other, F.Context));
    Other.BeforeWorldPublication = 0; CHECK(!C::EncodeDecision(Other));
    Other = *D; Other.AuthorityTimelineRevision = 0; CHECK(!C::EncodeDecision(Other));
    Context = F.Context; Context.AuthorityTimelineRevision = 0; CHECK(!C::Propose(F.Decision.Original, Context));
    Context = F.Context; Context.BeforeWorldPublication = 0; CHECK(!C::Propose(F.Decision.Original, Context));
    auto OldVersion = *Bytes; OldVersion[4] = 1; CHECK(!C::DecodeDecision(OldVersion));
    // Exact historical v1 bytes: remove newly added branch/parent fields and
    // set version1. There is no implicit upgrade to current revision.
    auto V1 = *Bytes; V1.erase(V1.begin()+16, V1.begin()+48); V1[4] = 1; CHECK(!C::DecodeDecision(V1));
    CHECK(F.Decision.Publication.WorldStateHash == Other.Publication.WorldStateHash);
    CHECK(C::PrepareLocalDecisionEntry(20, F.Decision, F.Context, F.Ledger, F.Contract, {1})->AcceptedDecision->AuthorityTimelineRevision == 7);
    Context = F.Context; ++Context.World.Id; CHECK(!C::AcceptPublished(20, F.Decision, Context, F.Ledger));
    Context = F.Context; ++Context.World.Incarnation; CHECK(!C::AcceptPublished(20, F.Decision, Context, F.Ledger));
    Other = F.Decision; Other.World.Id = 0; CHECK(!C::EncodeDecision(Other));
}
int main()
{
    if (Version2RegressionMain()) return 1;
    try { ExplicitAuthorityBranchAndPrecedingWorldPublication(); std::cout << "PASS " << Checks << " checks; wire v2 explicit authority timeline and prior world publication; no v1 upgrade; native closed\n"; return 0; }
    catch (const std::exception& E) { Budget = -1; std::cerr << "FAIL " << Checks << " " << E.what() << '\n'; return 1; }
}
