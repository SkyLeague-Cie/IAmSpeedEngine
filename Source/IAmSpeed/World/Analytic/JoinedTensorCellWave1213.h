#pragma once
#include "AnalyticWorldQuery.h"
#include <exception>
#include "IAmSpeed/World/Simulation/FrameFloatingPointContext.h"

namespace Speed::Analytic
{
struct FTensorCellWaveIdentity1213
{
    const void* World = nullptr;
    const void* Index = nullptr;
    // Joined invocation-local lifetime: records never escape this stack frame.
    // A recycled payload/index address cannot survive the owning invocation.
    const void* Invocation = nullptr;
    uint8 Selection = 0;
};
// Preparation metadata only; it never participates in physics or key validation.
enum class ETensorCellWindowReason1534 : uint8 { Legacy, First, Advance, BoundChanged };
enum class ETensorCellWaveFallback1213 : uint8
{ None, SerialPolicy, TooFew, Nested, Diagnostics, Capacity, InvalidBound, Numeric, ExecutorAdmission, Count };
struct FTensorCellWaveStats1213
{
    uint64 Attempts=0, Waves=0, Candidates=0, Eligible=0, Skipped=0;
    uint64 TimedAttempts=0, TimingSkippedAttempts=0;
    uint64 Computed=0, Reused=0, Recomputed=0, Wasted=0;
    uint64 RejectBound=0, RejectQuery=0, RejectContext=0, RejectNumeric=0;
    uint64 WindowAttempts1534=0, WindowWaves1534=0, WindowAdvances1534=0, WindowBoundRebases1534=0;
    int32 WindowCandidatesMax1534=0;
    uint64 Fallbacks[int32(ETensorCellWaveFallback1213::Count)]={};
    uint64 PreparationCycles=0, AdmissionCycles=0, CallerCycles=0, JoinWaitCycles=0;
    uint64 PreparationMeasuredAttempts1213=0, DispatchAttempts1213=0, DispatchMeasuredAttempts1213=0;
    uint64 OwnerEligibilitySeen1213=0, OwnerEligibilityTimed1213=0, OwnerEligibilitySkipped1213=0;
    uint64 OwnerEligibilityFailed1213=0, OwnerEligibilityCycles1213=0;
    // Lane elapsed scope includes FP application/capture, Evaluate, exceptions and loop overhead.
    uint64 KernelCycles=0, MaxLaneKernelCycles=0, ValidationCycles=0;
    // Owner FP publication is a separate sampled elapsed scope inside World.
    // It starts after validation; never add it to its enclosing frame or World.
    uint64 PublicationCycles1442=0, PublicationCalls1442=0;
    uint64 PublicationTimedCalls1442=0, PublicationFailedCalls1442=0;
    // UsefulLaneMax counts logical callbacks with completed outcomes, not native threads.
    int32 RequestedMax=0, LogicalMax=0, NativeMax=0, UsefulLaneMax=0, UsefulNativeMax=0;
    // Reused counts validated canonical consumption, not merely completed tasks.
    int32 ReusedLaneMax1213=0, ReusedNativeMax1213=0;
    uint64 ComputedKernelCycles1213=0, ReusedKernelCycles1213=0, WastedKernelCycles1213=0;
    // Group by the ACTUAL lane count. Offline exact rational means need no new
    // clock calls or floating-point arithmetic on the canonical owner.
    uint64 BalanceWaves1241[13]={}, BalanceLaneSum1241[13]={};
    uint64 BalanceLaneMin1241[13]={}, BalanceLaneMax1241[13]={};
    uint64 BalanceCellsMin1241[13]={}, BalanceCellsMax1241[13]={};
};
IAMSPEED_API bool IsTensorCellWave1213Enabled();
IAMSPEED_API void ResetTensorCellWaveStats1213();
IAMSPEED_API FTensorCellWaveStats1213 SnapshotTensorCellWaveStats1213();
IAMSPEED_API void LogTensorCellWaveStats1213(const TCHAR* Context);

// Owner-only BVH/SAT admission probe. Separate bounded budget; no clock,
// allocation or counter update when observation is OFF. Scope ends before
// constructing/dispatching the wave, and remains inside the parent World step.
class IAMSPEED_API FScopedTensorCellEligibilityObservation1213
{
public:
    FScopedTensorCellEligibilityObservation1213();
    ~FScopedTensorCellEligibilityObservation1213();
    FScopedTensorCellEligibilityObservation1213(const FScopedTensorCellEligibilityObservation1213&)=delete;
    FScopedTensorCellEligibilityObservation1213& operator=(const FScopedTensorCellEligibilityObservation1213&)=delete;
private:
    uint64 Start1213=0;
    int32 UncaughtAtEntry1213=0;
    bool bObserve1213=false, bTime1213=false;
};

// Same immutable query, Piecewise-cell generation/lifetime, selection and bound.
// Each complete Tensor approximation owns its local Best, BVH stack and box context.
// The original owner loop alone decides whether a result is consumed and folds it.
class IAMSPEED_API FJoinedTensorCellWave1213
{
public:
    static constexpr int32 MaximumProviders=4096;
    static int32 MinimumCandidateItems();
    // 0 preserves the one-shot legacy wave. A positive cap is opt-in and uses
    // the existing minimum/grain/lane admission, including its serial fallback.
    static int32 WindowCandidateItems1534();
    static int32 ResolveLanes(int32 Candidates,int32 RequestedLanes);
    FJoinedTensorCellWave1213(const FWorldQuery& Query, const FBox3d& Bounds,
        const FTensorCellWaveIdentity1213& Identity,
        TConstArrayView<int32> ProviderIndices, int32 TotalCandidates, int32 Lanes,
        TFunctionRef<double()> CaptureBound,
        TFunctionRef<FWorldHit(int32,double)> Evaluate,
        ETensorCellWindowReason1534 WindowReason1534=ETensorCellWindowReason1534::Legacy);
    ~FJoinedTensorCellWave1213();
    FJoinedTensorCellWave1213(const FJoinedTensorCellWave1213&)=delete;
    FJoinedTensorCellWave1213& operator=(const FJoinedTensorCellWave1213&)=delete;
    bool TryConsume(int32 ProviderIndex, const FWorldQuery& CurrentQuery,
        const FBox3d& CurrentBounds, const FTensorCellWaveIdentity1213& CurrentIdentity,
        double CurrentMaximumSearchTime, FWorldHit& OutHit);
    bool IsReady() const { return bReady; }
    // Scheduling hint only. TryConsume retains every original exact key/FP
    // check, single-use rule, output-state publication and deferred exception.
    bool ContainsProvider1534(int32 ProviderIndex) const;
    // Scheduling hint only. Any added bound arithmetic is isolated in a
    // private FP guard and restored before the original canonical boundary.
    bool HasSameBound1534(TFunctionRef<double()> CaptureBound) const;
private:
    struct FRecord { int32 Index=INDEX_NONE; FWorldHit Hit; std::exception_ptr Failure; FFrameFloatingPointContext NumericAfter; bool Used=false; bool Reused1213=false; int32 Lane1213=0; uint32 Native1213=0; uint64 KernelCycles1213=0; };
    FWorldQuery CapturedQuery;
    FBox3d CapturedBounds;
    FTensorCellWaveIdentity1213 CapturedIdentity;
    FFrameFloatingPointContext NumericContext;
    TArray<FRecord> Records;
    struct FLookup1213 { int32 Index=0; int32 Record=0; };
    // Sorting this private integer lookup never changes physical visitation.
    TArray<FLookup1213> RecordLookup1213;
    FTensorCellWaveStats1213 Stats;
    double MaximumSearchTime=1;
    bool bObserve=false, bTime=false, bReady=false;
};
}
