#pragma once
// Source-only diagnostic building block. No call sites or default activation.
#include "CoreMinimal.h"
#if !UE_BUILD_SHIPPING
#if PLATFORM_WINDOWS
#include "Windows/WindowsHWrapper.h"
#include <realtimeapiset.h>
#endif
namespace Speed::Analytic::Private
{
enum class EReplayClock918 : uint8 { QpcElapsed, ThreadCycleAccounting, ThreadUser100ns, ThreadKernel100ns };
struct FReplayClockRead918
{
    uint64 Count=0;
    uint64 QpcFrequency=0; // Must stay0 for all non-QPC kinds.
    uint32 Thread=0;
    EReplayClock918 Kind=EReplayClock918::QpcElapsed;
    bool bValid=false;
};
inline FReplayClockRead918 ReadReplayClock918(EReplayClock918 Kind)
{
    FReplayClockRead918 Out; Out.Kind=Kind;
#if PLATFORM_WINDOWS
    Out.Thread=::GetCurrentThreadId();
    if(Kind==EReplayClock918::QpcElapsed)
    {
        LARGE_INTEGER Count={}, Frequency={};
        if(::QueryPerformanceFrequency(&Frequency) && Frequency.QuadPart>0 &&
           ::QueryPerformanceCounter(&Count) && Count.QuadPart>=0)
        { Out.Count=uint64(Count.QuadPart); Out.QpcFrequency=uint64(Frequency.QuadPart); Out.bValid=true; }
    }
    else if(Kind==EReplayClock918::ThreadCycleAccounting)
    {
        ULONG64 Count=0;
        if(::QueryThreadCycleTime(::GetCurrentThread(),&Count))
        { Out.Count=uint64(Count); Out.bValid=true; }
    }
    else if(Kind==EReplayClock918::ThreadUser100ns || Kind==EReplayClock918::ThreadKernel100ns)
    {
        FILETIME Creation={}, Exit={}, Kernel={}, User={};
        if(::GetThreadTimes(::GetCurrentThread(),&Creation,&Exit,&Kernel,&User))
        {
            const FILETIME& T=Kind==EReplayClock918::ThreadUser100ns?User:Kernel;
            Out.Count=(uint64(T.dwHighDateTime)<<32)|uint64(T.dwLowDateTime);Out.bValid=true;
        }
    }
#endif
    return Out; // Unsupported/failure is explicitly invalid, never valid zero work.
}
struct FReplayClockDelta918
{
    uint64 Raw=0;
    uint64 QpcFrequency=0;
    bool bValid=false;
};
inline FReplayClockDelta918 DifferenceReplayClock918(const FReplayClockRead918& A,const FReplayClockRead918& B)
{
    FReplayClockDelta918 Out;
    if(!A.bValid || !B.bValid || A.Thread==0 || A.Thread!=B.Thread || A.Kind!=B.Kind || B.Count<A.Count) return Out;
    if(A.Kind==EReplayClock918::QpcElapsed)
    {
        if(A.QpcFrequency==0 || A.QpcFrequency!=B.QpcFrequency) return Out;
        Out.QpcFrequency=A.QpcFrequency;
    }
    else if(A.QpcFrequency!=0 || B.QpcFrequency!=0) return Out;
    Out.Raw=B.Count-A.Count;Out.bValid=true;
    return Out; // No seconds conversion, overhead subtraction, or CPU-resolution assertion.
}
}
#endif
