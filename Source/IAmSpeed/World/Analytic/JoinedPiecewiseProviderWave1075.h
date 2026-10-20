#pragma once
#include "AnalyticWorldQuery.h"
#include <exception>
#include "IAmSpeed/World/Simulation/FrameFloatingPointContext.h"

namespace Speed::Analytic
{
struct FPiecewiseProviderWaveIdentity1075
{
    const void* World = nullptr;
    const void* Index = nullptr;
    // Joined invocation-local lifetime: records never escape this stack frame.
    // A recycled payload/index address cannot survive the owning invocation.
    const void* Invocation = nullptr;
    uint8 Selection = 0;
};
enum class EProviderWaveFallback1075 : uint8
{ None, SerialPolicy, TooFew, Nested, Diagnostics, Capacity, InvalidBound, Numeric, ExecutorAdmission, Count };
struct FPiecewiseProviderWaveStats1075
{
    uint64 Attempts=0, Waves=0, Candidates=0, Eligible=0, Skipped=0;
    uint64 TimedAttempts=0, TimingSkippedAttempts=0;
    uint64 Computed=0, Reused=0, Recomputed=0, Wasted=0;
    uint64 RejectBound=0, RejectQuery=0, RejectContext=0, RejectNumeric=0;
    uint64 Fallbacks[int32(EProviderWaveFallback1075::Count)]={};
    uint64 PreparationCycles=0, AdmissionCycles=0, CallerCycles=0, JoinWaitCycles=0;
    // Lane elapsed scope includes FP application/capture, Evaluate, exceptions and loop overhead.
    uint64 KernelCycles=0, MaxLaneKernelCycles=0, ValidationCycles=0;
    // UsefulLaneMax counts logical callbacks with completed outcomes, not native threads.
    int32 LogicalMax=0, NativeMax=0, UsefulLaneMax=0, UsefulNativeMax=0;
};
IAMSPEED_API bool IsPiecewiseProviderWave1075Enabled();
IAMSPEED_API void ResetPiecewiseProviderWaveStats1075();
IAMSPEED_API FPiecewiseProviderWaveStats1075 SnapshotPiecewiseProviderWaveStats1075();
IAMSPEED_API void LogPiecewiseProviderWaveStats1075(const TCHAR* Context);

// Same immutable query, provider generation/lifetime, selection and bound.
// Each complete provider still owns its local Best, BVH stack and box context.
// The original owner loop alone decides whether a result is consumed and folds it.
class IAMSPEED_API FJoinedPiecewiseProviderWave1075
{
public:
    static constexpr int32 MaximumProviders=256;
    FJoinedPiecewiseProviderWave1075(const FWorldQuery& Query, const FBox3d& Bounds,
        const FPiecewiseProviderWaveIdentity1075& Identity,
        TConstArrayView<int32> ProviderIndices, int32 TotalCandidates, int32 Lanes,
        TFunctionRef<double()> CaptureBound,
        TFunctionRef<FWorldHit(int32,double)> Evaluate);
    ~FJoinedPiecewiseProviderWave1075();
    FJoinedPiecewiseProviderWave1075(const FJoinedPiecewiseProviderWave1075&)=delete;
    FJoinedPiecewiseProviderWave1075& operator=(const FJoinedPiecewiseProviderWave1075&)=delete;
    bool TryConsume(int32 ProviderIndex, const FWorldQuery& CurrentQuery,
        const FBox3d& CurrentBounds, const FPiecewiseProviderWaveIdentity1075& CurrentIdentity,
        double CurrentMaximumSearchTime, FWorldHit& OutHit);
    bool IsReady() const { return bReady; }
private:
    struct FRecord { int32 Index=INDEX_NONE; FWorldHit Hit; std::exception_ptr Failure; FFrameFloatingPointContext NumericAfter; bool Used=false; };
    FWorldQuery CapturedQuery;
    FBox3d CapturedBounds;
    FPiecewiseProviderWaveIdentity1075 CapturedIdentity;
    FFrameFloatingPointContext NumericContext;
    TArray<FRecord> Records;
    FPiecewiseProviderWaveStats1075 Stats;
    double MaximumSearchTime=1;
    bool bObserve=false, bTime=false, bReady=false;
};
}
