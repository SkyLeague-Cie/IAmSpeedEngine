#pragma once

#include "FrameParallelPolicy.h"

namespace Speed
{
enum class EFrameParallelQuotaObservation : uint8
{
    Unknown,
    OutsideJobUnbounded,
    ImmediateJobOnly,
    QualifiedAdmission,
    LinuxAffinityQuotaUnobserved
};

struct FFrameParallelRuntimeTopology
{
    FFrameParallelTopology Limits;
    int32 TaskGraphWorkers = 0, PrivateFrameWorkers = 0, RequestedBackend = 0;
    bool bPrivateExecutor = false, bExecutorReady = true;
    EFrameParallelQuotaObservation QuotaObservation = EFrameParallelQuotaObservation::Unknown;
    uint64 ProcessAffinityMask = 0;
    uint32 ImmediateJobControlFlags = 0;
    uint32 ImmediateJobRate10000 = 0;
    uint32 LastQueryError = 0;
    uint16 ProcessorGroup = 0;
    bool bProcessInJobKnown = false;
    bool bProcessInJob = false;
    bool bSingleProcessorGroupKnown = false;
};

// Call at configuration/world creation, never from a physical frame hot path.
// QualifiedEffectiveCpuRate10000 may only come from an owner which proves the
// complete enclosing Job hierarchy; it is not inferred from a child Job query.
IAMSPEED_API FFrameParallelRuntimeTopology CollectFrameParallelTopology(
    int32 QualifiedEffectiveCpuRate10000 = 0);
}
