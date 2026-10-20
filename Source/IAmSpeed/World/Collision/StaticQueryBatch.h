#pragma once
#include "CoreMinimal.h"
#include "IAmSpeed/World/Simulation/FrameParallelPolicy.h"
namespace Speed
{
// Admission policy only: no query, recorder, UObject access, or dispatch.
IAMSPEED_API bool CanSpeculateIndependentAnalyticGeometry1075();
// Identity of one immutable payload and its constructed query/index generation.
// Generation is observational; it never participates in physical arbitration.
struct FImmutableStaticQueryIdentity1075
{
    const void* Data = nullptr;
    const void* Index = nullptr;
    uint64 Generation = 0;
    bool IsValid() const { return Data && Index && Generation != 0; }
    bool operator==(const FImmutableStaticQueryIdentity1075& Other) const
    { return Data==Other.Data && Index==Other.Index && Generation==Other.Generation; }
};
enum class EIndependentQueryFallback : uint8
{
    None, SerialPolicy, TooFewQueryItems, ThreadOwnedDiagnostics, Busy, InvalidOutputExtent,
    NestedTask, BatchCapacityExceeded, NumericEnvironmentUnavailable
};
// Observation only; never serialized into physical state. Logical lanes and
// actual native participation are distinct. Completion order has no authority.
struct FIndependentQueryBatchStats
{
    int32 RequestedLanes = 1;
    int32 DispatchedLogicalLanes = 0;
    int32 DistinctNativeThreads = 0;
    // Whole-query completion counts distinguish useful lanes from empty chunks.
    // Facet/legacy callers that do not populate this field leave the flag false.
    bool bWorkCountsKnown1075 = false;
    int32 ExecutedItems1075[FFrameParallelPolicy::MaximumLanes] = {};
    int32 NonEmptyLanes1075 = 0;
    uint32 NativeThreadIds[FFrameParallelPolicy::MaximumLanes] = {};
    EIndependentQueryFallback Fallback = EIndependentQueryFallback::None;
};
}
