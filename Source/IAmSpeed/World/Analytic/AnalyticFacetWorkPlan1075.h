#pragma once

#include "CoreMinimal.h"
#include "IAmSpeed/World/Simulation/FrameParallelPolicy.h"

namespace Speed::Analytic
{
// Scheduling only: every admitted index still produces one private hit, followed
// by the original canonical fold. Zero preserves the 1074 strided assignment.
struct FAnalyticFacetWorkPlan1075
{
    int32 Items = 0;
    int32 Lanes = 1;
    bool bContiguous = false;

    static FAnalyticFacetWorkPlan1075 Resolve(
        int32 InItems, int32 RequestedLanes, int32 MinimumBatchItems,
        int32 MinimumItemsPerLane)
    {
        FAnalyticFacetWorkPlan1075 Plan;
        Plan.Items = FMath::Max(0, InItems);
        Plan.bContiguous = MinimumItemsPerLane > 0;
        if (Plan.Items < FMath::Max(2, MinimumBatchItems)) return Plan;
        const int32 Budget = FMath::Clamp(RequestedLanes, 1,
            FFrameParallelPolicy::MaximumLanes);
        const int32 UsefulLanes = MinimumItemsPerLane > 0
            ? Plan.Items / MinimumItemsPerLane : Plan.Items;
        Plan.Lanes = FMath::Clamp(UsefulLanes, 1, Budget);
        return Plan;
    }

    int32 First(int32 Lane) const
    {
        return bContiguous ? int32(int64(Items) * Lane / Lanes) : Lane;
    }
    int32 End(int32 Lane) const
    {
        return bContiguous ? int32(int64(Items) * (Lane + 1) / Lanes) : Items;
    }
    int32 Stride() const { return bContiguous ? 1 : Lanes; }
};
}
