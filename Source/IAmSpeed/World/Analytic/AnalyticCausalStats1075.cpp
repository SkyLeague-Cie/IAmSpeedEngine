#include "AnalyticCausalStats1075.h"
#include "HAL/IConsoleManager.h"
#include <atomic>
#include <exception>
#include <mutex>

namespace Speed::Analytic
{
namespace
{
TAutoConsoleVariable<int32> CVarAnalyticCausal1075(
    TEXT("p.IAmSpeed.Parallel.AnalyticCausal1075"), 0,
    TEXT("Diagnostic only. Bounded elapsed causal observations; disable for final performance runs."));
TAutoConsoleVariable<int32> CVarAnalyticCausalMaxSamples1075(
    TEXT("p.IAmSpeed.Parallel.AnalyticCausalMaxSamples1075"), 4096,
    TEXT("Maximum timed samples per kind between quiescent resets (0..1048576). Default4096; skipped calls stay explicit."));
std::atomic<uint64> GAnalyticCausalSeen1075[AnalyticCausalKindCount1075]{};
std::mutex GAnalyticCausalTotalsMutex1075;
FAnalyticCausalSnapshot1075 GAnalyticCausalTotals1075;
int32 AnalyticCausalSampleLimit1075() { return FMath::Clamp(CVarAnalyticCausalMaxSamples1075.GetValueOnAnyThread(), 0, 1048576); }
int32 AnalyticCausalItemBucket1075(int32 Items)
{
    if (Items <= 0) return 0;
    int32 Bucket = 1;
    while (Items > 1 && Bucket < 13) { Items >>= 1; ++Bucket; }
    return Bucket;
}
}

FScopedAnalyticCausalObservation1075::FScopedAnalyticCausalObservation1075(
    EAnalyticCausalKind1075 InKind, const FIndependentQueryBatchStats* InDispatch)
    : Kind(InKind), Dispatch(InDispatch)
{
    if (CVarAnalyticCausal1075.GetValueOnAnyThread() == 0) return;
    const uint64 Ordinal = GAnalyticCausalSeen1075[int32(Kind)].fetch_add(1, std::memory_order_relaxed);
    if (Ordinal >= uint64(AnalyticCausalSampleLimit1075())) return;
    State.Emplace();
    State.GetValue().UncaughtAtStart = std::uncaught_exceptions();
    State.GetValue().Start = FPlatformTime::Cycles64();
}
FScopedAnalyticCausalObservation1075::~FScopedAnalyticCausalObservation1075()
{
    if (!State.IsSet()) return;
    const uint64 Elapsed = FPlatformTime::Cycles64() - State.GetValue().Start;
    uint64 Numeric = 0, MaximumNumeric = 0, Callback = 0;
    for (int32 Lane = 0; Lane < FFrameParallelPolicy::MaximumLanes; ++Lane)
    {
        Numeric += State.GetValue().NumericCycles[Lane];
        MaximumNumeric = FMath::Max(MaximumNumeric, State.GetValue().NumericCycles[Lane]);
        Callback += State.GetValue().DispatchObservation.LaneCallbackCycles[Lane];
    }
    const int32 Lanes = Dispatch ? FMath::Clamp(Dispatch->DispatchedLogicalLanes,
        0, FFrameParallelPolicy::MaximumLanes) : 1;
    const int32 Fallback = Dispatch ? int32(Dispatch->Fallback) : 0;
    std::lock_guard<std::mutex> Lock(GAnalyticCausalTotalsMutex1075);
    auto& T = GAnalyticCausalTotals1075.Kinds[int32(Kind)];
    ++T.Sampled;
    T.FailedSamples += std::uncaught_exceptions() > State.GetValue().UncaughtAtStart;
    T.Items += State.GetValue().Items;
    T.ElapsedCycles += Elapsed;
    T.PreparationCycles += State.GetValue().PreparationCycles;
    T.NumericCycles += Numeric;
    T.MaximumLaneNumericCycles += MaximumNumeric;
    T.FoldCycles += State.GetValue().FoldCycles;
    T.AdmissionCycles += State.GetValue().DispatchObservation.AdmissionCycles;
    T.CallerExecutionCycles += State.GetValue().DispatchObservation.CallerExecutionCycles;
    T.JoinWaitCycles += State.GetValue().DispatchObservation.JoinWaitCycles;
    T.CallbackCycles += Callback;
    ++T.LogicalLaneCounts[Lanes];
    ++T.FallbackCounts[FMath::Clamp(Fallback, 0, AnalyticCausalFallbackCount1075 - 1)];
    const int32 ItemBucket1185 = AnalyticCausalItemBucket1075(State.GetValue().Items);
    ++T.ItemCountBuckets[ItemBucket1185];
    T.ItemBucketItems1185[ItemBucket1185] += State.GetValue().Items;
    T.ItemBucketElapsedCycles1185[ItemBucket1185] += Elapsed;
    T.ItemBucketNumericCycles1185[ItemBucket1185] += Numeric;
    T.ItemBucketMaximumElapsedCycles1185[ItemBucket1185] = FMath::Max(
        T.ItemBucketMaximumElapsedCycles1185[ItemBucket1185], Elapsed);
    if (State.GetValue().bDispatchAttempted && State.GetValue().DispatchObservation.bAdmissionCompleted)
    {
        T.PoolCalls += State.GetValue().DispatchObservation.Fallback == EFrameDispatchFallback1075::None;
        T.NestedExecutorFallbacks += State.GetValue().DispatchObservation.Fallback == EFrameDispatchFallback1075::Nested;
        T.QueueCapacityFallbacks += State.GetValue().DispatchObservation.Fallback == EFrameDispatchFallback1075::QueueCapacity;
    }
}
void ResetAnalyticCausalStats1075()
{
    std::lock_guard<std::mutex> Lock(GAnalyticCausalTotalsMutex1075);
    GAnalyticCausalTotals1075 = FAnalyticCausalSnapshot1075{};
    for (auto& Count : GAnalyticCausalSeen1075) Count.store(0, std::memory_order_relaxed);
}
FAnalyticCausalSnapshot1075 SnapshotAnalyticCausalStats1075()
{
    std::lock_guard<std::mutex> Lock(GAnalyticCausalTotalsMutex1075);
    FAnalyticCausalSnapshot1075 Copy = GAnalyticCausalTotals1075;
    Copy.bEnabled = CVarAnalyticCausal1075.GetValueOnAnyThread() != 0;
    Copy.SampleLimitPerKind = AnalyticCausalSampleLimit1075();
    for (int32 Kind = 0; Kind < AnalyticCausalKindCount1075; ++Kind)
    {
        Copy.Kinds[Kind].Seen = GAnalyticCausalSeen1075[Kind].load(std::memory_order_relaxed);
        Copy.Kinds[Kind].Skipped = Copy.Kinds[Kind].Seen >= Copy.Kinds[Kind].Sampled
            ? Copy.Kinds[Kind].Seen - Copy.Kinds[Kind].Sampled : 0;
    }
    return Copy;
}
void LogAnalyticCausalStats1075(const TCHAR* Context)
{
    const auto S = SnapshotAnalyticCausalStats1075();
    for (int32 Kind = 0; Kind < AnalyticCausalKindCount1075; ++Kind)
    {
        const auto& T = S.Kinds[Kind];
        UE_LOG(LogTemp, Display, TEXT("[IAmSpeedAnalyticCausal1075] Context=%s Kind=%d Enabled=%d Limit=%d Seen=%llu Sampled=%llu Skipped=%llu Failed=%llu Items=%llu Elapsed=%llu Prepare=%llu NumericSum=%llu NumericLaneMaxSum=%llu Fold=%llu Admission=%llu Caller=%llu JoinWait=%llu CallbackSum=%llu Pool=%llu NestedExecutorFallback=%llu QueueFallback=%llu SerialPolicy=%llu TooFew=%llu Diagnostics=%llu NestedPolicy=%llu Capacity=%llu NumericUnavailable=%llu SecondsPerCycle=%.17g Unit=PlatformElapsedCycles Overlap=1"),
            Context ? Context : TEXT(""), Kind, S.bEnabled ? 1 : 0, S.SampleLimitPerKind,
            T.Seen, T.Sampled, T.Skipped, T.FailedSamples, T.Items, T.ElapsedCycles,
            T.PreparationCycles, T.NumericCycles, T.MaximumLaneNumericCycles, T.FoldCycles,
            T.AdmissionCycles, T.CallerExecutionCycles, T.JoinWaitCycles, T.CallbackCycles,
            T.PoolCalls, T.NestedExecutorFallbacks, T.QueueCapacityFallbacks,
            T.FallbackCounts[int32(EIndependentQueryFallback::SerialPolicy)],
            T.FallbackCounts[int32(EIndependentQueryFallback::TooFewQueryItems)],
            T.FallbackCounts[int32(EIndependentQueryFallback::ThreadOwnedDiagnostics)],
            T.FallbackCounts[int32(EIndependentQueryFallback::NestedTask)],
            T.FallbackCounts[int32(EIndependentQueryFallback::BatchCapacityExceeded)],
            T.FallbackCounts[int32(EIndependentQueryFallback::NumericEnvironmentUnavailable)],
            FPlatformTime::GetSecondsPerCycle64());
        // Separate additive buckets within ONE kind only. Numeric and elapsed
        // overlap; child kinds and parallel lanes must never be summed as CPU.
        for (int32 Bucket1185 = 0; Bucket1185 < 14; ++Bucket1185)
        {
            UE_LOG(LogTemp, Display, TEXT("[IAmSpeedAnalyticCausalBuckets1185] Context=%s Kind=%d Bucket=%d Count=%llu Items=%llu Elapsed=%llu NumericSum=%llu MaximumElapsed=%llu Seen=%llu Sampled=%llu Skipped=%llu Failed=%llu Limit=%d CoverageComplete=%d SecondsPerCycle=%.17g Scope=SampledInclusiveScopesByVisitedItems Unit=PlatformElapsedCycles NumericAndKindsNotAdditive=1"),
                Context ? Context : TEXT(""), Kind, Bucket1185,
                T.ItemCountBuckets[Bucket1185], T.ItemBucketItems1185[Bucket1185], T.ItemBucketElapsedCycles1185[Bucket1185],
                T.ItemBucketNumericCycles1185[Bucket1185], T.ItemBucketMaximumElapsedCycles1185[Bucket1185],
                T.Seen, T.Sampled, T.Skipped, T.FailedSamples, S.SampleLimitPerKind,
                T.Skipped == 0 && T.FailedSamples == 0 ? 1 : 0,
                FPlatformTime::GetSecondsPerCycle64());
        }
    }
}
}
