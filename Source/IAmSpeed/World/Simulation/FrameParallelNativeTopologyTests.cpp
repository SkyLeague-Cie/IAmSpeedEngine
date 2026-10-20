#include "FrameParallelTopology.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedNativeTopologyCollectionTest,
    "IAmSpeed.Parallel.Topology.NativeCollection",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FIAmSpeedNativeTopologyCollectionTest::RunTest(const FString&)
{
    const auto Host = Speed::CollectFrameParallelTopology();
    TestEqual(TEXT("Hardware discovery does not claim Unreal task workers"), Host.TaskGraphWorkers, 0);
    TestEqual(TEXT("An explicit serial request stays serial"),
        Speed::FFrameParallelPolicy::ResolveLaneLimit(Host.Limits, 1), 1);
#if PLATFORM_LINUX
    TestTrue(TEXT("Linux physical cores were observed in the actual affinity"), Host.Limits.PhysicalCores > 0);
    TestTrue(TEXT("Linux logical capacity bounds the observed physical topology"),
        Host.Limits.LogicalCores >= Host.Limits.PhysicalCores && Host.Limits.AvailableLogicalCores > 0 &&
        Host.Limits.AvailableLogicalCores <= Host.Limits.LogicalCores);
    TestFalse(TEXT("Visible affinity never certifies enclosing cgroup quota"), Host.Limits.bEffectiveCpuRateKnown);
    TestEqual(TEXT("Unqualified automatic selection remains serial"),
        Speed::FFrameParallelPolicy::ResolveLaneLimit(Host.Limits, 0), 1);
#endif
    AddInfo(FString::Printf(TEXT("[NativeTopology1617] Physical=%d Logical=%d AvailableLogical=%d EffectiveQuotaKnown=%d QuotaObservation=%u Scope=ReadOnlyHardwareNotEffectiveCgroupOrParallelExecution"),
        Host.Limits.PhysicalCores, Host.Limits.LogicalCores, Host.Limits.AvailableLogicalCores,
        int(Host.Limits.bEffectiveCpuRateKnown), uint32(Host.QuotaObservation)));
    return !HasAnyErrors();
}
#endif
