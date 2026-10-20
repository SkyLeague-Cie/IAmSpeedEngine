#pragma once

#include "CoreMinimal.h"
#include "FrameParallelPolicy.h"

namespace Speed
{
enum class EFrameDispatchFallback1075 : uint8 { None, Nested, QueueCapacity };

// Optional caller-owned storage. Each lane writes only its own element; the
// owner reads it after the existing joined barrier. Units are platform elapsed
// cycles (FPlatformTime), not CPU-thread cycles. Callback sums overlap in time.
struct FFrameDispatchObservation1075
{
    uint64 AdmissionCycles = 0; // Lease, group allocation, queue admission/wake.
    uint64 CallerExecutionCycles = 0; // Lane 0, or all lanes on serial fallback.
    uint64 JoinWaitCycles = 0; // Caller wait after its execution has completed.
    uint64 LaneCallbackCycles[FFrameParallelPolicy::MaximumLanes]{};
    bool bAdmissionCompleted = false;
    EFrameDispatchFallback1075 Fallback = EFrameDispatchFallback1075::None;
};
}
