#include "IAmSpeed/World/Simulation/FrameParallelSettings.h"

#include "HAL/IConsoleManager.h"
#include "IAmSpeed/World/Simulation/FrameParallelTopology.h"
#include "IAmSpeed/World/Simulation/FrameParallelExecutor.h"
#include "IAmSpeed/World/Simulation/FrameSpeculationSafety1213.h"

namespace
{
thread_local bool GInsideFrameParallelTask = false;
thread_local Speed::FFrameParallelStageTotals GFrameParallelStageTotals[3];
TAutoConsoleVariable<int32> CVarFrameParallelLanes(
    TEXT("p.IAmSpeed.Parallel.Lanes"), 1,
    TEXT("Total intra-frame lanes including the simulation caller. 1=serial baseline, "
         "0=host policy, positive values=bounded diagnostic override. "
         "A request is not proof that that many threads executed concurrently."));
TAutoConsoleVariable<int32> CVarFrameParallelMaxLanes(
    TEXT("p.IAmSpeed.Parallel.MaxLanes"), Speed::FFrameParallelPolicy::MaximumLanes,
    TEXT("Maximum total intra-frame lanes, further bounded by host topology. "
         "Capacity12; serial default remains until representative/full-matrix measurements."));
TAutoConsoleVariable<int32> CVarFrameParallelReservedCores(
    TEXT("p.IAmSpeed.Parallel.ReservedPhysicalCores"), 2,
    TEXT("Physical cores reserved by the bounded host policy outside intra-frame work."));
TAutoConsoleVariable<int32> CVarFrameParallelMinQueryItems(
    TEXT("p.IAmSpeed.Parallel.MinQueryItems"), 8,
    TEXT("Minimum independent query count before task dispatch. "
         "Candidate threshold; measure dispatch/join and tails before adopting."));
TAutoConsoleVariable<int32> CVarFrameParallelMinPrepareItems(
    TEXT("p.IAmSpeed.Parallel.MinPrepareItems"), 8,
    TEXT("Minimum independent values-only preparation items before task dispatch."));
TAutoConsoleVariable<int32> CVarFrameParallelMinFacetItems(
    TEXT("p.IAmSpeed.Parallel.MinFacetItems"), 32,
    TEXT("Minimum independent analytic facets before bounded task dispatch. "
         "Candidates merge in their original serial order after the barrier."));
TAutoConsoleVariable<int32> CVarFrameParallelCamera(
    TEXT("p.IAmSpeed.Parallel.Camera"), 1,
    TEXT("Allow a certified independent presentation producer after snapshotN. "
         "Requires more than1 selected lane and a join before publicationN."));

const Speed::FFrameParallelRuntimeTopology& HostTopology()
{
    // No launcher-supplied quota assertion: an unknown effective Job quota
    // stays unknown in the decision. Explicit diagnostic runs retain that fact.
    static const Speed::FFrameParallelRuntimeTopology Host = []
    {
        Speed::FFrameParallelRuntimeTopology Captured = Speed::CollectFrameParallelTopology();
        const int32 Max = FMath::Clamp(CVarFrameParallelMaxLanes.GetValueOnAnyThread(), 1, Speed::FFrameParallelPolicy::MaximumLanes);
        const int32 Reserved = FMath::Max(0, CVarFrameParallelReservedCores.GetValueOnAnyThread());
        int32 HardwareCapacity = Captured.Limits.PhysicalCores > 0
            ? FMath::Min(Max,FMath::Max(1,FMath::Min(Captured.Limits.PhysicalCores,Captured.Limits.AvailableLogicalCores)-Reserved)) : 1;
        if(Captured.Limits.bEffectiveCpuRateKnown)
            HardwareCapacity=FMath::Min(HardwareCapacity,FMath::Max(1,int32(int64(Captured.Limits.LogicalCores)*Captured.Limits.EffectiveCpuRate10000/10000)));
        const auto Executor = Speed::ConfigureFrameParallelExecutor(HardwareCapacity,CVarFrameParallelLanes.GetValueOnAnyThread());
        Captured.TaskGraphWorkers=Executor.TaskGraphWorkers;Captured.PrivateFrameWorkers=Executor.PrivateWorkers;
        Captured.RequestedBackend=Speed::RequestedFrameExecutorBackend();Captured.bPrivateExecutor=Executor.bPrivateSelected;Captured.bExecutorReady=Executor.bReady;
        Captured.Limits.AvailableTaskWorkers=Executor.bReady?Executor.SelectedWorkers:0;
#if !UE_BUILD_SHIPPING
        const int32 FirstReservedCores = FMath::Max(0,
            CVarFrameParallelReservedCores.GetValueOnAnyThread());
        const int32 FirstMaxLanes = FMath::Clamp(
            CVarFrameParallelMaxLanes.GetValueOnAnyThread(), 1, Speed::FFrameParallelPolicy::MaximumLanes);
        // Function-local static initialization is thread safe and executes once
        // per module/process lifetime. These are observed runtime fields, not
        // CPU-OS guesses for the Unreal worker pool or enclosing Job hierarchy.
        UE_LOG(LogTemp, Display, TEXT("[FrameParallelTopologyV1] PhysicalCores=%d LogicalCores=%d AvailableLogicalCores=%d AvailableTaskWorkers=%d ReservedPhysicalCores=%d MaxLanes=%d EffectiveCpuRate10000=%d EffectiveCpuRateKnown=%d SingleProcessorGroupKnown=%d ProcessorGroup=%u AffinityMask=%016llX ProcessInJobKnown=%d ProcessInJob=%d QuotaObservation=%u ImmediateJobControlFlags=%08x ImmediateJobRate10000=%u LastQueryError=%u TaskGraphWorkers=%d PrivateFrameWorkers=%d WorkerBackend=%d PrivateSelected=%d ExecutorReady=%d WorkerCapacityScope=SelectedIAmSpeedBackend ConfigScope=FirstSettingsCapture TopologyScope=ProcessFirstCapture"),
            Captured.Limits.PhysicalCores, Captured.Limits.LogicalCores,
            Captured.Limits.AvailableLogicalCores, Captured.Limits.AvailableTaskWorkers,
            FirstReservedCores, FirstMaxLanes, Captured.Limits.EffectiveCpuRate10000,
            Captured.Limits.bEffectiveCpuRateKnown ? 1 : 0,
            Captured.bSingleProcessorGroupKnown ? 1 : 0, uint32(Captured.ProcessorGroup),
            Captured.ProcessAffinityMask, Captured.bProcessInJobKnown ? 1 : 0,
            Captured.bProcessInJob ? 1 : 0, uint32(Captured.QuotaObservation),
            Captured.ImmediateJobControlFlags, Captured.ImmediateJobRate10000,
            Captured.LastQueryError,Captured.TaskGraphWorkers,Captured.PrivateFrameWorkers,Captured.RequestedBackend,int32(Captured.bPrivateExecutor),int32(Captured.bExecutorReady));
#endif
        return Captured;
    }();
    return Host;
}
}

namespace Speed
{
FFrameParallelSettings CaptureFrameParallelSettings()
{
    FFrameParallelTopology Limits = HostTopology().Limits;
    Limits.MaxLanes = FMath::Clamp(CVarFrameParallelMaxLanes.GetValueOnAnyThread(), 1, Speed::FFrameParallelPolicy::MaximumLanes);
    Limits.ReservedPhysicalCores = FMath::Max(0,
        CVarFrameParallelReservedCores.GetValueOnAnyThread());

    FFrameParallelSettings Settings;
    Settings.RequestedLanes = CVarFrameParallelLanes.GetValueOnAnyThread();
    Settings.Decision = FFrameParallelPolicy::ResolveDecision(Limits, Settings.RequestedLanes);
    Settings.LaneLimit = Settings.Decision.LaneLimit;
    // Decision retains the host/request cap. Actual admission must also keep
    // every displaced calculation serial under trapping/unknown FP masks.
    if (Settings.LaneLimit > 1 && !AreCurrentFrameFloatingPointExceptionsMasked1213())
        Settings.LaneLimit = 1;
    Settings.MinQueryItems = FMath::Max(1,
        CVarFrameParallelMinQueryItems.GetValueOnAnyThread());
    Settings.MinPrepareItems = FMath::Max(1,
        CVarFrameParallelMinPrepareItems.GetValueOnAnyThread());
    Settings.MinFacetItems = FMath::Max(1,
        CVarFrameParallelMinFacetItems.GetValueOnAnyThread());
    Settings.bParallelCamera = Settings.LaneLimit > 1 &&
        CVarFrameParallelCamera.GetValueOnAnyThread() != 0;
    return Settings;
}

bool IsInsideFrameParallelTask()
{
    return GInsideFrameParallelTask;
}

FScopedFrameParallelTask::FScopedFrameParallelTask()
    : FScopedFrameParallelTask(CaptureFrameFloatingPointContext())
{
}

FScopedFrameParallelTask::FScopedFrameParallelTask(const FFrameFloatingPointContext& Context)
    : Previous(GInsideFrameParallelTask), NumericContext(Context)
{
    GInsideFrameParallelTask = true;
}

bool FScopedFrameParallelTask::IsNumericContextApplied() const noexcept
{
    return bNumericContextActive && NumericContext.IsApplied();
}

bool FScopedFrameParallelTask::RestoreNumericContext() noexcept
{
    const bool bRestored = NumericContext.Restore();
    if (bRestored) bNumericContextActive = false;
    return bRestored;
}

FScopedFrameParallelTask::~FScopedFrameParallelTask()
{
    const bool bRestored = RestoreNumericContext();
    ensureAlwaysMsgf(bRestored, TEXT("IAmSpeed frame task numeric restoration failed; infrastructure failure"));
    GInsideFrameParallelTask = Previous;
}

FFrameParallelStageTotals GetFrameParallelStageTotals(EFrameParallelStage Stage)
{
    return GFrameParallelStageTotals[static_cast<uint8>(Stage)];
}

void ResetFrameParallelStageTotals(EFrameParallelStage Stage)
{
    GFrameParallelStageTotals[static_cast<uint8>(Stage)] = {};
}

void RecordFrameParallelStage(EFrameParallelStage Stage, int32 Items,
    int32 LogicalLanes, int32 NativeThreads, EFrameParallelFallback Fallback)
{
    auto& Totals = GFrameParallelStageTotals[static_cast<uint8>(Stage)];
    ++Totals.Invocations;
    Totals.Items += FMath::Max(0, Items);
    Totals.ParallelInvocations += LogicalLanes > 1;
    Totals.MaximumLogicalLanes = FMath::Max(Totals.MaximumLogicalLanes, LogicalLanes);
    Totals.MaximumNativeThreads = FMath::Max(Totals.MaximumNativeThreads, NativeThreads);
    Totals.NumericFallbacks += Fallback == EFrameParallelFallback::NumericEnvironmentUnavailable;
    if (Fallback != EFrameParallelFallback::None)
        Totals.FallbackMask |= uint64(1) << static_cast<uint8>(Fallback);
}
}
