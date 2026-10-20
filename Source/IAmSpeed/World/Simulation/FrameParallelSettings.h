#pragma once

#include "CoreMinimal.h"
#include "IAmSpeed/World/Simulation/FrameParallelPolicy.h"
#include "IAmSpeed/World/Simulation/FrameFloatingPointContext.h"

namespace Speed
{
/** A simulation-owner capture. Lane counts include the calling simulation lane. */
struct FFrameParallelSettings
{
    FFrameParallelDecision Decision;
    int32 RequestedLanes = 1;
    int32 LaneLimit = 1;
    int32 MinQueryItems = 8;
    int32 MinFacetItems = 32;
    int32 MinPrepareItems = 8;
    bool bParallelCamera = false;
};

/** Capture on the owner before dispatch, never read mutable CVars inside a task.
 * Host topology is sampled once per process. A world retains its query pool
 * configuration until that world is rebuilt. Tests can inject explicit policy
 * values without changing these process settings. */
IAMSPEED_API FFrameParallelSettings CaptureFrameParallelSettings();

/** A task already using this frame's bounded lanes must not start a sub-pool. */
IAMSPEED_API bool IsInsideFrameParallelTask();

enum class EFrameParallelStage : uint8 { Camera, Prepare, QueryBatch };
enum class EFrameParallelFallback : uint8
{
    None, SerialPolicy, TooFewItems, NestedTask, UnsupportedWork,
    ThreadOwnedDiagnostics, BatchCapacityExceeded, NumericEnvironmentUnavailable
};
/** Owner-thread observations, read/reset only at a joined frame barrier. */
struct FFrameParallelStageTotals
{
    uint64 Invocations = 0, ParallelInvocations = 0, Items = 0;
    uint64 NumericFallbacks = 0, FallbackMask = 0;
    int32 MaximumLogicalLanes = 0, MaximumNativeThreads = 0;
};
IAMSPEED_API FFrameParallelStageTotals GetFrameParallelStageTotals(EFrameParallelStage Stage);
IAMSPEED_API void ResetFrameParallelStageTotals(EFrameParallelStage Stage);
IAMSPEED_API void RecordFrameParallelStage(EFrameParallelStage Stage, int32 Items,
    int32 LogicalLanes, int32 NativeThreads, EFrameParallelFallback Fallback);

class IAMSPEED_API FScopedFrameParallelTask
{
public:
    FScopedFrameParallelTask();
    explicit FScopedFrameParallelTask(const FFrameFloatingPointContext& Context);
    bool IsNumericContextApplied() const noexcept;
    bool RestoreNumericContext() noexcept;
    ~FScopedFrameParallelTask();
    FScopedFrameParallelTask(const FScopedFrameParallelTask&) = delete;
    FScopedFrameParallelTask& operator=(const FScopedFrameParallelTask&) = delete;
private:
    bool Previous;
    FScopedFrameFloatingPointContext NumericContext;
    bool bNumericContextActive = true;
};
}
