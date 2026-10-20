#pragma once

#include "CoreMinimal.h"
#include "HAL/PlatformTime.h"
#include "IAmSpeed/World/Collision/StaticQueryBatch.h"
#include "IAmSpeed/World/Simulation/FrameDispatchObservation1075.h"

namespace Speed::Analytic
{
enum class EAnalyticCausalKind1075 : uint8
{
    WorldSweep, ExtrudedProvider, TensorProvider, PiecewiseProvider,
    IndependentSweepBatch, TensorApproximation, DetailedWorldQuery, Count
};
constexpr int32 AnalyticCausalKindCount1075 = int32(EAnalyticCausalKind1075::Count);
constexpr int32 AnalyticCausalFallbackCount1075 =
    int32(EIndependentQueryFallback::NumericEnvironmentUnavailable) + 1;

// All timing fields use FPlatformTime elapsed cycles. Nested groups and lane
// callbacks overlap: neither kind totals nor callback sums are additive CPU
// costs. Samples beyond the explicit per-kind cap are counted, never timed.
struct FAnalyticCausalKindStats1075
{
    uint64 Seen = 0, Sampled = 0, Skipped = 0, FailedSamples = 0, Items = 0;
    uint64 ElapsedCycles = 0, PreparationCycles = 0, NumericCycles = 0, FoldCycles = 0;
    uint64 AdmissionCycles = 0, CallerExecutionCycles = 0, JoinWaitCycles = 0;
    uint64 CallbackCycles = 0, MaximumLaneNumericCycles = 0;
    uint64 PoolCalls = 0, NestedExecutorFallbacks = 0, QueueCapacityFallbacks = 0;
    uint64 FallbackCounts[AnalyticCausalFallbackCount1075]{};
    uint64 LogicalLaneCounts[FFrameParallelPolicy::MaximumLanes + 1]{};
    // 0=zero items, 1=one, 2=2..3, 3=4..7, ..., 13=4096 or more.
    uint64 ItemCountBuckets[14]{};
    // Inclusive elapsed scopes grouped by their actual visited-item count.
    // Only sampled scopes contribute. Skipped scopes stay UNKNOWN, not zero.
    uint64 ItemBucketItems1185[14]{};
    uint64 ItemBucketElapsedCycles1185[14]{};
    uint64 ItemBucketNumericCycles1185[14]{};
    uint64 ItemBucketMaximumElapsedCycles1185[14]{};
};
struct FAnalyticCausalSnapshot1075
{
    FAnalyticCausalKindStats1075 Kinds[AnalyticCausalKindCount1075];
    int32 SampleLimitPerKind = 0;
    bool bEnabled = false;
};

// Call these only at a quiescent case/frame boundary, after every joined task.
// Reset restarts the bounded sample budget; it does not affect physical state.
IAMSPEED_API void ResetAnalyticCausalStats1075();
IAMSPEED_API FAnalyticCausalSnapshot1075 SnapshotAnalyticCausalStats1075();
IAMSPEED_API void LogAnalyticCausalStats1075(const TCHAR* Context);

class FScopedAnalyticCycles1075
{
public:
    explicit FScopedAnalyticCycles1075(uint64* InTarget)
        : Target(InTarget), Start(Target ? FPlatformTime::Cycles64() : 0) {}
    ~FScopedAnalyticCycles1075()
    {
        if (Target) *Target += FPlatformTime::Cycles64() - Start;
    }
    FScopedAnalyticCycles1075(const FScopedAnalyticCycles1075&) = delete;
    FScopedAnalyticCycles1075& operator=(const FScopedAnalyticCycles1075&) = delete;
private:
    uint64* Target;
    uint64 Start;
};

class IAMSPEED_API FScopedAnalyticCausalObservation1075
{
public:
    explicit FScopedAnalyticCausalObservation1075(EAnalyticCausalKind1075 InKind,
        const FIndependentQueryBatchStats* InDispatch = nullptr);
    ~FScopedAnalyticCausalObservation1075();
    bool IsEnabled() const { return State.IsSet(); }
    void SetItems(int32 Items) { if (State.IsSet()) State.GetValue().Items = FMath::Max(0, Items); }
    void AddItem() { if (State.IsSet()) ++State.GetValue().Items; }
    void MarkPrepared()
    {
        if (State.IsSet() && !State.GetValue().bPrepared)
        {
            State.GetValue().PreparationCycles = FPlatformTime::Cycles64() - State.GetValue().Start;
            State.GetValue().bPrepared = true;
        }
    }
    uint64* NumericTarget(int32 Lane)
    {
        return State.IsSet() ? &State.GetValue().NumericCycles[Lane] : nullptr;
    }
    uint64* FoldTarget() { return State.IsSet() ? &State.GetValue().FoldCycles : nullptr; }
    FFrameDispatchObservation1075* DispatchObservation()
    {
        if (!State.IsSet()) return nullptr;
        State.GetValue().bDispatchAttempted = true;
        return &State.GetValue().DispatchObservation;
    }
    FScopedAnalyticCausalObservation1075(const FScopedAnalyticCausalObservation1075&) = delete;
    FScopedAnalyticCausalObservation1075& operator=(const FScopedAnalyticCausalObservation1075&) = delete;
private:
    struct FMeasuredState
    {
        uint64 Start = 0, PreparationCycles = 0, FoldCycles = 0;
        uint64 NumericCycles[FFrameParallelPolicy::MaximumLanes]{};
        FFrameDispatchObservation1075 DispatchObservation;
        int32 Items = 0, UncaughtAtStart = 0;
        bool bPrepared = false, bDispatchAttempted = false;
    };
    EAnalyticCausalKind1075 Kind;
    const FIndependentQueryBatchStats* Dispatch;
    // Observer OFF constructs no measured storage and calls no clock.
    TOptional<FMeasuredState> State;
};
}
