#pragma once

#include "CoreMinimal.h"

namespace Speed
{
// Values captured by the owner outside the frame. This policy does not query the
// OS, sample clocks, change deterministic state, or construct worker threads.
struct FFrameParallelTopology
{
    int32 PhysicalCores = 0;
    int32 LogicalCores = 0;
    int32 AvailableLogicalCores = 0;
    int32 AvailableTaskWorkers = 0;
    // Effective rate across the complete enclosing Job chain, in 1/10000 of
    // machine CPU capacity. An immediate child Job rate alone is insufficient.
    int32 EffectiveCpuRate10000 = 0;
    bool bEffectiveCpuRateKnown = false;
    int32 ReservedPhysicalCores = 2;
    int32 MaxLanes = 12;
};

struct FFrameParallelDecision
{
    int32 LaneLimit = 1;
    int32 RequestedLanes = 0;
    bool bTopologyKnown = false;
    bool bEffectiveQuotaKnown = false;
    bool bInvalidRequest = false;
    bool bRequestLimited = false;
};

// A lane is total concurrent execution, INCLUDING the simulation caller.
// The defaults are candidate caps, not a measured optimum or release default.
class FFrameParallelPolicy
{
public:
    static constexpr int32 MaximumLanes = 12;

    static FFrameParallelDecision ResolveDecision(
        const FFrameParallelTopology& Topology, const int32 RequestedLanes)
    {
        FFrameParallelDecision Result;
        Result.RequestedLanes = RequestedLanes;
        Result.bInvalidRequest = RequestedLanes != 0 && RequestedLanes != 1
            && RequestedLanes != 2 && RequestedLanes != 4
            && RequestedLanes != 8 && RequestedLanes != 12;
        Result.bTopologyKnown = Topology.PhysicalCores > 0
            && Topology.LogicalCores >= Topology.PhysicalCores
            && Topology.AvailableLogicalCores > 0
            && Topology.AvailableLogicalCores <= Topology.LogicalCores
            && Topology.AvailableTaskWorkers >= 0;
        Result.bEffectiveQuotaKnown = Topology.bEffectiveCpuRateKnown
            && Topology.EffectiveCpuRate10000 > 0 && Topology.EffectiveCpuRate10000 <= 10000;
        if (Result.bInvalidRequest || !Result.bTopologyKnown || RequestedLanes == 1
            || (RequestedLanes == 0 && !Result.bEffectiveQuotaKnown))
        {
            Result.bRequestLimited = RequestedLanes > 1;
            return Result;
        }

        // Affinity cannot create physical capacity. Physical topology scoped to
        // the effective process mask is preferred; a narrower logical mask is
        // additionally bounded here. SMT does not double the physical budget.
        const int32 EffectiveCoreUpperBound = FMath::Min(
            Topology.PhysicalCores, Topology.AvailableLogicalCores);
        const int32 Reserved = FMath::Max(0, Topology.ReservedPhysicalCores);
        const int32 CoreBudget = FMath::Max(1, EffectiveCoreUpperBound - Reserved);
        const int32 PoolBudget = Topology.AvailableTaskWorkers < MAX_int32
            ? Topology.AvailableTaskWorkers + 1 : MAX_int32;
        // An explicit supported request is diagnostic when the enclosing quota is
        // unknown. It never qualifies automatic production selection. The OS
        // still enforces its actual Job limits; the caller reports UNKNOWN.
        const int32 QuotaBudget = Result.bEffectiveQuotaKnown ? FMath::Max(1, int32(
            int64(Topology.LogicalCores) * Topology.EffectiveCpuRate10000 / 10000)) : MAX_int32;
        const int32 HardwareLimit = FMath::Min(FMath::Clamp(Topology.MaxLanes, 1, MaximumLanes),
            FMath::Min(CoreBudget, FMath::Min(PoolBudget, QuotaBudget)));
        const int32 RequestedLimit = RequestedLanes == 0
            ? HardwareLimit : FMath::Min(RequestedLanes, HardwareLimit);
        Result.LaneLimit = RequestedLimit >= 12 ? 12 : RequestedLimit >= 8 ? 8
            : RequestedLimit >= 4 ? 4 : RequestedLimit >= 2 ? 2 : 1;
        Result.bRequestLimited = RequestedLanes > Result.LaneLimit;
        return Result;
    }

    static int32 ResolveLaneLimit(const FFrameParallelTopology& Topology,
        const int32 RequestedLanes)
    {
        return ResolveDecision(Topology, RequestedLanes).LaneLimit;
    }

    // MinimumParallelWorkItems is fixed from paired measurements for a phase.
    // Do not select it from an in-frame elapsed timer or assume actor count is
    // equivalent to query count. The serial path uses the original call order.
    static int32 ResolveDispatchLanes(const int32 LaneLimit,
        const int32 WorkItemCount, const int32 MinimumParallelWorkItems)
    {
        if (LaneLimit <= 1 || WorkItemCount < FMath::Max(2, MinimumParallelWorkItems))
        {
            return 1;
        }
        return FMath::Min(FMath::Clamp(LaneLimit, 1, MaximumLanes), WorkItemCount);
    }
};
}
