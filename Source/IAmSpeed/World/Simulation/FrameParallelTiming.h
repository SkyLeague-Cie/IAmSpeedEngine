#pragma once
#include "CoreMinimal.h"
#if !UE_BUILD_SHIPPING
namespace Speed::ParallelTiming
{
struct FFrameWorkEvidence
{
    int32 Requested = 0, PolicyLanes = 0, LogicalMax = 0, NativeMax = 0;
    // Observed owner-joined work only. Counts denote actual logical dispatches;
    // per-stage native maxima distinguish those from multi-thread participation.
    uint64 FacetProviders = 0, ParallelFacetProviders = 0, EvaluatedFacets = 0;
    uint64 ParallelPrepareCalls = 0, ParallelQueryCalls = 0, ParallelCameraCalls = 0;
    int32 FacetNativeMax = 0, PrepareNativeMax = 0, QueryNativeMax = 0, CameraNativeMax = 0;
    uint32 FallbackMask = 0;
    bool bKnown = false;
};
// Exclusive top-level elapsed wall intervals on the canonical owner. Analytic
// child intervals and simultaneous worker callback costs must not be added here.
enum class EFrameCostPhase : uint8
{ Admission, Input, Prepare, WorldStep, Snapshot, Presentation, Publish, PostCommit, Finalize, Count };
inline constexpr int32 FrameCostPhaseCount = static_cast<int32>(EFrameCostPhase::Count);
struct FFrameCostPhases
{
    uint64 Ticks[FrameCostPhaseCount] = {};
    uint32 SeenMask = 0;
    bool bTracked = false, bValid = false;
};
struct FFrameSample
{
    uint64 Frame = 0, Begin = 0, End = 0, StateHash = 0, InputHash = 0;
    FFrameWorkEvidence Work;
    FFrameCostPhases Phases;
    bool bCommitted = false, bClockValid = false;
};
// Case identity comes from the already-joined SLTest boundary. This recorder
// never borrows a CaseSat identity and never changes a physical admission.
class IAMSPEED_API FCaseRecorder final
{
public:
    void ConfigureAfterJoin(uint64 DriverKey, int32 Case, int32 Batch, uint64 WorldGeneration);
    void BeginDriver(bool bFast, uint64 CanonicalBefore, bool bOwnedLane) noexcept;
    void Close(bool bCompleted, const TCHAR* Reason) noexcept;
    bool IsActive() const { return bOpen; }
    const FString& JoinedCaseContextForLogging() const { return JoinedCaseContext; }
    bool ClaimJoinedCaseLog() noexcept { if(bJoinedCaseLogClosed)return false; bJoinedCaseLogClosed=true; return !JoinedCaseContext.IsEmpty(); }
    bool IsPhaseTimingEnabled() const { return bPhaseTiming; }
    void Record(const FFrameSample& Sample) noexcept;
private:
    FString Run;
    // Joined-case observer identity remains available when frame timing is off.
    FString JoinedCaseContext;
    bool bJoinedCaseLogClosed = true;
    uint64 Driver = 0, Epoch = 0, NextEpoch = 0, Generation = 0, CanonicalBefore = 0;
    int32 NativeCase = INDEX_NONE, BatchIndex = INDEX_NONE, Mode = 0;
    uint32 OwnerThread = 0;
    uint64 Frequency = 0, Attempts = 0, Dropped = 0, RawRowsWritten = 0;
    bool bConfigured = false, bOpen = false, bFast = false, bInvalid = false, bPhaseTiming = false;
    TArray<FFrameSample> Samples;
};
class IAMSPEED_API FFrameScope final
{
public:
    FFrameScope(FCaseRecorder& Owner, uint64 Frame, float PhysicalDeltaTime) noexcept;
    ~FFrameScope() noexcept { Finish(Sample.bCommitted, Sample.StateHash, Sample.InputHash, Sample.Work); }
    void NextPhase(EFrameCostPhase Phase) noexcept;
    void MarkCommitted(uint64 StateHash, uint64 InputHash, const FFrameWorkEvidence& Work = {}) noexcept;
    void Finish(bool bCommitted, uint64 StateHash = 0, uint64 InputHash = 0,
        const FFrameWorkEvidence& Work = {}) noexcept;
private:
    FCaseRecorder& Recorder;
    FFrameSample Sample;
    bool bTracked = false, bFinished = false;
    EFrameCostPhase Phase = EFrameCostPhase::Admission;
    uint64 LastPhaseTick = 0;
    void AccumulatePhase(uint64 Now) noexcept;
};
}
#endif
