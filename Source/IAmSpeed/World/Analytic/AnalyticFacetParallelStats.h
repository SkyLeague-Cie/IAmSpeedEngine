#pragma once
#include "CoreMinimal.h"
#include "IAmSpeed/World/Collision/StaticQueryBatch.h"
namespace Speed::Analytic
{
// Owner-lane observation only. Read/reset at a joined frame boundary. It is
// never authoritative physics, snapshot content, or an elapsed-time estimate.
struct FFacetParallelTotals
{
    uint64 Providers = 0, ParallelProviders = 0, EvaluatedFacets = 0;
    uint64 InstrumentationFallbackProviders = 0, NestedFallbackProviders = 0;
    uint64 CapacityFallbackProviders = 0, NumericFallbackProviders = 0;
    int32 MaximumLogicalLanes = 0, MaximumNativeThreads = 0;
    void Accumulate(const FFacetParallelTotals& Other)
    {
        Providers += Other.Providers; ParallelProviders += Other.ParallelProviders;
        EvaluatedFacets += Other.EvaluatedFacets;
        InstrumentationFallbackProviders += Other.InstrumentationFallbackProviders;
        NestedFallbackProviders += Other.NestedFallbackProviders;
        CapacityFallbackProviders += Other.CapacityFallbackProviders;
        NumericFallbackProviders += Other.NumericFallbackProviders;
        MaximumLogicalLanes = FMath::Max(MaximumLogicalLanes, Other.MaximumLogicalLanes);
        MaximumNativeThreads = FMath::Max(MaximumNativeThreads, Other.MaximumNativeThreads);
    }
    void Add(const Speed::FIndependentQueryBatchStats& Stats, int32 Facets)
    {
        ++Providers;
        EvaluatedFacets += Facets;
        if (Stats.DispatchedLogicalLanes > 1) ++ParallelProviders;
        InstrumentationFallbackProviders += Stats.Fallback == Speed::EIndependentQueryFallback::ThreadOwnedDiagnostics;
        NestedFallbackProviders += Stats.Fallback == Speed::EIndependentQueryFallback::NestedTask;
        CapacityFallbackProviders += Stats.Fallback == Speed::EIndependentQueryFallback::BatchCapacityExceeded;
        NumericFallbackProviders += Stats.Fallback == Speed::EIndependentQueryFallback::NumericEnvironmentUnavailable;
        MaximumLogicalLanes = FMath::Max(MaximumLogicalLanes, Stats.DispatchedLogicalLanes);
        MaximumNativeThreads = FMath::Max(MaximumNativeThreads, Stats.DistinctNativeThreads);
    }
};
}
