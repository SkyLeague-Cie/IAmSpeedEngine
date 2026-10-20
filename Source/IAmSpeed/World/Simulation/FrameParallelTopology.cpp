#include "FrameParallelTopology.h"

#if PLATFORM_WINDOWS
#include "Windows/WindowsHWrapper.h"
#endif

namespace Speed
{
FFrameParallelRuntimeTopology CollectFrameParallelTopology(
    const int32 QualifiedEffectiveCpuRate10000)
{
    FFrameParallelRuntimeTopology Result;
    // Filled from IAmSpeed's executor by the owner settings capture.
    // Hardware discovery has no dependency on an Unreal task scheduler.
    Result.Limits.AvailableTaskWorkers = 0;
#if PLATFORM_WINDOWS
    const HANDLE Process = ::GetCurrentProcess();
    Result.Limits.LogicalCores = int32(::GetActiveProcessorCount(ALL_PROCESSOR_GROUPS));
    USHORT GroupCount = 64;
    USHORT Groups[64] = {};
    DWORD_PTR ProcessMask = 0, SystemMask = 0;
    if (::GetProcessGroupAffinity(Process, &GroupCount, Groups) && GroupCount == 1
        && ::GetProcessAffinityMask(Process, &ProcessMask, &SystemMask) && ProcessMask != 0)
    {
        Result.bSingleProcessorGroupKnown = true;
        Result.ProcessorGroup = Groups[0];
        Result.ProcessAffinityMask = uint64(ProcessMask);
        for (DWORD_PTR Bits = ProcessMask; Bits; Bits &= Bits - 1)
        {
            ++Result.Limits.AvailableLogicalCores;
        }
        DWORD RequiredBytes = 0;
        ::GetLogicalProcessorInformationEx(RelationProcessorCore, nullptr, &RequiredBytes);
        if (::GetLastError() == ERROR_INSUFFICIENT_BUFFER && RequiredBytes > 0
            && RequiredBytes <= 128 * 1024)
        {
            TArray<uint8> Storage;
            Storage.SetNumUninitialized(int32(RequiredBytes));
            if (::GetLogicalProcessorInformationEx(RelationProcessorCore,
                reinterpret_cast<PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX>(Storage.GetData()),
                &RequiredBytes))
            {
                bool bValid = true;
                for (uint32 Offset = 0; Offset < RequiredBytes; )
                {
                    if (RequiredBytes - Offset < 2 * sizeof(DWORD)) { bValid = false; break; }
                    const auto* Entry = reinterpret_cast<const SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX*>(
                        Storage.GetData() + Offset);
                    if (Entry->Size < 2 * sizeof(DWORD) || Entry->Size > RequiredBytes - Offset)
                    { bValid = false; break; }
                    if (Entry->Relationship == RelationProcessorCore)
                    {
                        const uint64 Prefix = offsetof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX, Processor)
                            + offsetof(PROCESSOR_RELATIONSHIP, GroupMask);
                        if (Entry->Size < Prefix || Entry->Processor.GroupCount == 0
                            || Prefix + uint64(Entry->Processor.GroupCount) * sizeof(GROUP_AFFINITY) > Entry->Size)
                        { bValid = false; break; }
                        for (USHORT Index = 0; Index < Entry->Processor.GroupCount; ++Index)
                        {
                            const GROUP_AFFINITY& Affinity = Entry->Processor.GroupMask[Index];
                            if (Affinity.Group == Groups[0] && (Affinity.Mask & ProcessMask) != 0)
                            { ++Result.Limits.PhysicalCores; break; }
                        }
                    }
                    Offset += Entry->Size;
                }
                if (!bValid) { Result.Limits.PhysicalCores = 0; }
            }
            else { Result.LastQueryError = ::GetLastError(); }
        }
        else { Result.LastQueryError = ::GetLastError(); }
    }
    else { Result.LastQueryError = ::GetLastError(); }

    BOOL InJob = 0;
    if (::IsProcessInJob(Process, nullptr, &InJob))
    {
        Result.bProcessInJobKnown = true;
        Result.bProcessInJob = InJob != 0;
        if (!InJob)
        {
            Result.Limits.EffectiveCpuRate10000 = 10000;
            Result.Limits.bEffectiveCpuRateKnown = true;
            Result.QuotaObservation = EFrameParallelQuotaObservation::OutsideJobUnbounded;
        }
        else
        {
            JOBOBJECT_CPU_RATE_CONTROL_INFORMATION Cpu = {};
            if (::QueryInformationJobObject(nullptr, JobObjectCpuRateControlInformation,
                &Cpu, sizeof(Cpu), nullptr))
            {
                Result.ImmediateJobControlFlags = Cpu.ControlFlags;
                if (Cpu.ControlFlags & JOB_OBJECT_CPU_RATE_CONTROL_HARD_CAP)
                    Result.ImmediateJobRate10000 = Cpu.CpuRate;
                else if (Cpu.ControlFlags & JOB_OBJECT_CPU_RATE_CONTROL_MIN_MAX_RATE)
                    Result.ImmediateJobRate10000 = Cpu.MaxRate;
                Result.QuotaObservation = EFrameParallelQuotaObservation::ImmediateJobOnly;
                // Windows exposes the immediate Job here; an unobserved parent
                // may enforce a stricter quota, even if this Job has no limit.
            }
            else { Result.LastQueryError = ::GetLastError(); }
        }
    }
    else { Result.LastQueryError = ::GetLastError(); }
#endif
    if (QualifiedEffectiveCpuRate10000 > 0 && QualifiedEffectiveCpuRate10000 <= 10000)
    {
        Result.Limits.EffectiveCpuRate10000 = QualifiedEffectiveCpuRate10000;
        Result.Limits.bEffectiveCpuRateKnown = true;
        Result.QuotaObservation = EFrameParallelQuotaObservation::QualifiedAdmission;
    }
    return Result;
}
}
