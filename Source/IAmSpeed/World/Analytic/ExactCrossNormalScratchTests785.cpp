#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "ExactCrossNormalCache784.h"
#include "Misc/AutomationTest.h"
#include <limits>

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedExactCrossNormalScratch785Test,
    "IAmSpeed.AnalyticWorld.ExactCrossNormalScratch785",
    (EAutomationTestFlags::EditorContext | EAutomationTestFlags::ClientContext |
     EAutomationTestFlags::ServerContext | EAutomationTestFlags::CommandletContext) | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedExactCrossNormalScratch785Test::RunTest(const FString& Parameters)
{
    using namespace Speed::Analytic;
    using namespace Speed::Analytic::Private;
    FWorldQuery Query; Query.Shape = EQueryShape::Box;
    FBoxSweepContext Context(Query); Context.bLazyExtrudedSatAxes = true;
    FExactCrossNormalScope Reuse(Query, Context, false);
    FExactCrossNormalScope Probe(Query, Context, true);
    FWorldQuery OtherQuery = Query;
    FBoxSweepContext OtherContext(Query); OtherContext.bLazyExtrudedSatAxes = true;
    if (!Reuse.IsBoundTo(Query, Context) || Reuse.IsBoundTo(OtherQuery, Context) ||
        Reuse.IsBoundTo(Query, OtherContext))
    { AddError(TEXT("785 scope identity differs")); return false; }
    const FVector3d Cross(0.0, 3.0, 4.0);
    const double Squared = Cross.SquaredLength();
    bool Used = true;
    const FVector3d First = Reuse.Normalize(Cross, Squared, 0, &Used);
    if (Used || !FExactCrossNormalScope::SameBits(First, Cross / FMath::Sqrt(Squared)))
    { AddError(TEXT("785 empty-cache/original arithmetic")); return false; }
    const FVector3d Second = Reuse.Normalize(Cross, Squared, 0, &Used);
    if (!Used || !FExactCrossNormalScope::SameBits(First, Second))
    { AddError(TEXT("785 exact prepared hit missing")); return false; }
    FVector3d NegativeZero = Cross; NegativeZero.X = -0.0;
    const FVector3d Signed = Reuse.Normalize(NegativeZero, Squared, 0, &Used);
    if (Used || !FExactCrossNormalScope::SameBits(Signed, NegativeZero / FMath::Sqrt(Squared)))
    { AddError(TEXT("785 signed zero must remain a distinct bit key")); return false; }
    Reuse.Normalize(NegativeZero, Squared, 0, &Used);
    if (!Used) { AddError(TEXT("785 negative-zero repeated key")); return false; }
    Reuse.Normalize(Cross, Squared, 1, &Used);
    if (Used) { AddError(TEXT("785 axis groups must remain separate")); return false; }
    Reuse.Normalize(Cross, Squared + 1.0, 0, &Used);
    if (Used) { AddError(TEXT("785 squared-length bits must belong to key")); return false; }
    for (int32 I = 0; I < 128; ++I)
    {
        const FVector3d V(0.125 * (I % 7 + 1), -0.25 * (I % 5 + 1), 0.5 * (I % 11 + 1));
        const double L = V.SquaredLength();
        const FVector3d Original = V / FMath::Sqrt(L);
        const FVector3d A = Reuse.Normalize(V, L, I % 3, &Used);
        const FVector3d B = Reuse.Normalize(V, L, I % 3, &Used);
        if (!Used || !FExactCrossNormalScope::SameBits(A, Original) || !FExactCrossNormalScope::SameBits(B, Original))
        { AddError(FString::Printf(TEXT("FIRST785 eviction/key divergence I=%d"), I)); return false; }
    }
    for (int32 I = 0; I < 2; ++I)
    {
        const FVector3d Actual = Probe.Normalize(Cross, Squared, 2, &Used);
        if (Used || !FExactCrossNormalScope::SameBits(Actual, Cross / FMath::Sqrt(Squared)))
        { AddError(TEXT("785 observation must recompute original arithmetic")); return false; }
    }
    const auto& Stats = Probe.GetObservationStats();
    if (Stats.Eligible != 2 || Stats.Hits != 1 || Stats.Misses != 1 || Stats.RecomputedBitMismatches != 0)
    { AddError(TEXT("785 exact observation counters")); return false; }
    for (const double L : { 1.0e-20, 0.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN() })
    {
        if (FExactCrossNormalScope::CanMemoizeKey(Cross, L, 0))
        { AddError(TEXT("785 invalid/nonfinite squared length must bypass cache")); return false; }
    }
    Reuse.Normalize(Cross, Squared, -1, &Used);
    if (Used) { AddError(TEXT("785 invalid axis bypass")); return false; }
    FExactCrossNormalScope Fresh(Query, Context, false);
    Fresh.Normalize(Cross, Squared, 0, &Used);
    if (Used) { AddError(TEXT("785 no cross-scope lifetime reuse")); return false; }
    AddInfo(TEXT("[ExactCrossNormalScratch785] Status=EXACT_SCRATCH_PASS SignedZeroDistinct=1 SquaredLengthKey=1 AxisGroupsSeparate=1 FiniteFallback=1 ObserveReturnsOriginal=1 ScopeLifetime=1 RuntimeProviderCoverage=UNKNOWN Timing=UNKNOWN"));
    return true;
}
#endif
