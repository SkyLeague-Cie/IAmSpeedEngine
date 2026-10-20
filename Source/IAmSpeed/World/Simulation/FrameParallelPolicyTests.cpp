#include "FrameParallelPolicy.h"

#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFrameParallelPolicyTests,
    "IAmSpeed.Simulation.ParallelFrame.WorkerPolicy",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FFrameParallelPolicyTests::RunTest(const FString& Parameters)
{
    using namespace Speed;
    FFrameParallelTopology Topology;
    TestEqual(TEXT("Unknown topology is serial"),
        FFrameParallelPolicy::ResolveLaneLimit(Topology, 0), 1);
    Topology.PhysicalCores = 24;
    Topology.LogicalCores = 32;
    Topology.AvailableLogicalCores = 32;
    Topology.AvailableTaskWorkers = 31;
    Topology.EffectiveCpuRate10000 = 10000;
    Topology.bEffectiveCpuRateKnown = true;
    for (const int32 Requested : {1, 2, 4, 8, 12})
    {
        TestEqual(FString::Printf(TEXT("Supported explicit %d"), Requested),
            FFrameParallelPolicy::ResolveLaneLimit(Topology, Requested), Requested);
    }
    TestEqual(TEXT("High-core auto is bounded"),
        FFrameParallelPolicy::ResolveLaneLimit(Topology, 0), 12);
    Topology.bEffectiveCpuRateKnown = false;
    TestEqual(TEXT("Unknown enclosing Job quota keeps auto serial"),
        FFrameParallelPolicy::ResolveLaneLimit(Topology, 0), 1);
    const FFrameParallelDecision UnknownDiagnostic =
        FFrameParallelPolicy::ResolveDecision(Topology, 4);
    TestEqual(TEXT("Explicit bounded diagnostic can run with unknown quota"),
        UnknownDiagnostic.LaneLimit, 4);
    TestFalse(TEXT("Unknown diagnostic quota stays visibly unqualified"),
        UnknownDiagnostic.bEffectiveQuotaKnown);
    Topology.bEffectiveCpuRateKnown = true;
    Topology.EffectiveCpuRate10000 = 800;
    TestEqual(TEXT("Machine quota limits total lanes"),
        FFrameParallelPolicy::ResolveLaneLimit(Topology, 4), 2);
    Topology.EffectiveCpuRate10000 = 10000;
    Topology.MaxLanes = 3;
    TestEqual(TEXT("Auto selects only measured candidate lane counts"),
        FFrameParallelPolicy::ResolveLaneLimit(Topology, 0), 2);
    Topology.MaxLanes = FFrameParallelPolicy::MaximumLanes;
    for (const int32 Invalid : {-1, 3, 5, 7, 9, 13, MAX_int32})
    {
        const FFrameParallelDecision Decision =
            FFrameParallelPolicy::ResolveDecision(Topology, Invalid);
        TestEqual(TEXT("Invalid request is serial"), Decision.LaneLimit, 1);
        TestTrue(TEXT("Invalid request is visible"), Decision.bInvalidRequest);
    }
    Topology.MaxLanes = 11;
    TestEqual(TEXT("cap below12 selects8"), FFrameParallelPolicy::ResolveLaneLimit(Topology, 12), 8);
    Topology.MaxLanes = FFrameParallelPolicy::MaximumLanes;
    Topology.AvailableTaskWorkers = 7;
    TestEqual(TEXT("pool of7 workers plus caller admits8"), FFrameParallelPolicy::ResolveLaneLimit(Topology, 12), 8);
    Topology.AvailableTaskWorkers = 31;
    Topology.EffectiveCpuRate10000 = 2000;
    TestEqual(TEXT("20percent of32 logical processors cannot admit8 or12"), FFrameParallelPolicy::ResolveLaneLimit(Topology, 12), 4);
    Topology.EffectiveCpuRate10000 = 4000;
    TestEqual(TEXT("40percent of32 logical processors admits12"), FFrameParallelPolicy::ResolveLaneLimit(Topology, 12), 12);
    Topology.EffectiveCpuRate10000 = 10000;
    TestEqual(TEXT("twelve lanes clamp to available work"), FFrameParallelPolicy::ResolveDispatchLanes(12, 9, 2), 9);
    Topology.PhysicalCores = 4;
    Topology.LogicalCores = 8;
    Topology.AvailableLogicalCores = 8;
    TestEqual(TEXT("Four physical cores reserve two"),
        FFrameParallelPolicy::ResolveLaneLimit(Topology, 0), 2);
    const FFrameParallelDecision Limited = FFrameParallelPolicy::ResolveDecision(Topology, 4);
    TestEqual(TEXT("Explicit request cannot bypass topology cap"), Limited.LaneLimit, 2);
    TestTrue(TEXT("Limited request is observable"), Limited.bRequestLimited);
    Topology.AvailableTaskWorkers = 0;
    TestEqual(TEXT("No pool worker is serial"),
        FFrameParallelPolicy::ResolveLaneLimit(Topology, 0), 1);
    Topology.AvailableTaskWorkers = 7;
    Topology.AvailableLogicalCores = 2;
    TestEqual(TEXT("Restricted effective CPU mask is serial"),
        FFrameParallelPolicy::ResolveLaneLimit(Topology, 0), 1);
    Topology.PhysicalCores = 2;
    Topology.LogicalCores = 4;
    Topology.AvailableLogicalCores = 4;
    TestEqual(TEXT("Two physical cores retain original serial path"),
        FFrameParallelPolicy::ResolveLaneLimit(Topology, 0), 1);
    Topology.PhysicalCores = 8;
    TestEqual(TEXT("Inconsistent topology is serial"),
        FFrameParallelPolicy::ResolveLaneLimit(Topology, 0), 1);
    TestEqual(TEXT("Empty dispatch is serial"),
        FFrameParallelPolicy::ResolveDispatchLanes(4, 0, 8), 1);
    TestEqual(TEXT("Small work retains serial"),
        FFrameParallelPolicy::ResolveDispatchLanes(4, 7, 8), 1);
    TestEqual(TEXT("Threshold boundary permits bounded dispatch"),
        FFrameParallelPolicy::ResolveDispatchLanes(4, 8, 8), 4);
    TestEqual(TEXT("Cannot dispatch more lanes than items"),
        FFrameParallelPolicy::ResolveDispatchLanes(4, 2, 2), 2);
    return true;
}
#endif
