#include "AnalyticCausalStats1075.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "HAL/IConsoleManager.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedCausalGrainBudget1185,
    "IAmSpeed.Parallel.Analytic1185.CausalGrainBudget",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FIAmSpeedCausalGrainBudget1185::RunTest(const FString&)
{
    using namespace Speed::Analytic;
    auto* Enabled = IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.Parallel.AnalyticCausal1075"));
    auto* Limit = IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.Parallel.AnalyticCausalMaxSamples1075"));
    if (!Enabled || !Limit) { AddError(TEXT("causal controls unavailable")); return false; }
    const int32 OldEnabled = Enabled->GetInt(), OldLimit = Limit->GetInt();
    ON_SCOPE_EXIT { Enabled->Set(OldEnabled, ECVF_SetByCode); Limit->Set(OldLimit, ECVF_SetByCode); ResetAnalyticCausalStats1075(); };
    Enabled->Set(0, ECVF_SetByCode); Limit->Set(2, ECVF_SetByCode); ResetAnalyticCausalStats1075();
    if (!TestEqual(TEXT("observer-off value admitted"), Enabled->GetInt(), 0)) return false;
    { FScopedAnalyticCausalObservation1075 Off(EAnalyticCausalKind1075::TensorApproximation); Off.SetItems(9); }
    auto S = SnapshotAnalyticCausalStats1075();
    TestEqual(TEXT("observer-off creates no seen sample"), S.Kinds[int32(EAnalyticCausalKind1075::TensorApproximation)].Seen, uint64(0));

    Enabled->Set(1, ECVF_SetByCode); Limit->Set(0, ECVF_SetByCode); ResetAnalyticCausalStats1075();
    if (!TestEqual(TEXT("observer-on value admitted"), Enabled->GetInt(), 1)) return false;
    { FScopedAnalyticCausalObservation1075 Skipped(EAnalyticCausalKind1075::TensorApproximation); Skipped.SetItems(64); }
    S = SnapshotAnalyticCausalStats1075();
    TestEqual(TEXT("zero budget retains original call count"), S.Kinds[int32(EAnalyticCausalKind1075::TensorApproximation)].Seen, uint64(1));
    TestEqual(TEXT("zero budget records skipped work explicitly"), S.Kinds[int32(EAnalyticCausalKind1075::TensorApproximation)].Skipped, uint64(1));
    TestEqual(TEXT("zero budget contributes no timed sample"), S.Kinds[int32(EAnalyticCausalKind1075::TensorApproximation)].Sampled, uint64(0));

    Limit->Set(262144, ECVF_SetByCode); ResetAnalyticCausalStats1075();
    TestEqual(TEXT("measured long-case budget is not silently clamped to65536"), SnapshotAnalyticCausalStats1075().SampleLimitPerKind, 262144);
    Limit->Set(2, ECVF_SetByCode); ResetAnalyticCausalStats1075();
    { FScopedAnalyticCausalObservation1075 Outer(EAnalyticCausalKind1075::WorldSweep);
      { FScopedAnalyticCausalObservation1075 Empty(EAnalyticCausalKind1075::TensorApproximation); Empty.SetItems(0); }
      { FScopedAnalyticCausalObservation1075 Many(EAnalyticCausalKind1075::TensorApproximation); Many.SetItems(64); Many.MarkPrepared();
        FScopedAnalyticCycles1075 Numeric(Many.NumericTarget(0)); }
      { FScopedAnalyticCausalObservation1075 Overflow(EAnalyticCausalKind1075::TensorApproximation); Overflow.SetItems(8192); }
    }
    S = SnapshotAnalyticCausalStats1075();
    const auto& T = S.Kinds[int32(EAnalyticCausalKind1075::TensorApproximation)];
    TestEqual(TEXT("nested kind keeps its own exact seen count"), T.Seen, uint64(3));
    TestEqual(TEXT("kind budget counts only two complete samples"), T.Sampled, uint64(2));
    TestEqual(TEXT("kind overflow remains visibly skipped"), T.Skipped, uint64(1));
    TestEqual(TEXT("empty scope belongs to zero-item bucket"), T.ItemCountBuckets[0], uint64(1));
    TestEqual(TEXT("64 visited items belong to64..127 bucket"), T.ItemCountBuckets[7], uint64(1));
    TestEqual(TEXT("skipped large scope cannot manufacture a timed large bucket"), T.ItemCountBuckets[13], uint64(0));
    uint64 Count = 0, Items = 0, Elapsed = 0, Numeric = 0;
    for (int32 B = 0; B < 14; ++B)
    {
        Count += T.ItemCountBuckets[B]; Items += T.ItemBucketItems1185[B]; Elapsed += T.ItemBucketElapsedCycles1185[B]; Numeric += T.ItemBucketNumericCycles1185[B];
        TestTrue(TEXT("bucket maximum cannot exceed its own elapsed sum"), T.ItemBucketMaximumElapsedCycles1185[B] <= T.ItemBucketElapsedCycles1185[B]);
    }
    TestEqual(TEXT("bucket population partitions sampled scopes exactly"), Count, T.Sampled);
    TestEqual(TEXT("bucket visited-item totals partition original item count"), Items, T.Items);
    TestEqual(TEXT("bucket elapsed partitions this one kind only"), Elapsed, T.ElapsedCycles);
    TestEqual(TEXT("bucket numeric partitions original lane-sum counter"), Numeric, T.NumericCycles);
    TestEqual(TEXT("parent kind remains one enclosing sample"), S.Kinds[int32(EAnalyticCausalKind1075::WorldSweep)].Sampled, uint64(1));
    return !HasAnyErrors();
}
#endif
