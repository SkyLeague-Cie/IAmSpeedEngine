#include "AnalyticCaseSatDiagnostics.h"
#include "ReplayProbe925.h"

#if !UE_BUILD_SHIPPING
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTLS.h"
#include "HAL/PlatformTime.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include <atomic>
#if PLATFORM_WINDOWS
#include "Windows/WindowsHWrapper.h"
#include <realtimeapiset.h>
#endif

namespace Speed::Analytic::CaseSatDiagnostics
{
namespace
{
TAutoConsoleVariable<int32> CVarCaseSatDiagnostics(
    TEXT("p.IAmSpeed.AnalyticWorld.CaseSatDiagnostics"), 0,
    TEXT("Diagnostic-only owned case.0off(default),1CPUonly(requiresPhaseTiming0),2CPU+SATcounts(requiresPhaseTiming2); -SLCaseSatRunUUID required. Never use detailed CPU as real-cost benchmark."));

TAutoConsoleVariable<int32> CVarCpuPhase400(
    TEXT("p.IAmSpeed.AnalyticWorld.CaseCpuPhase400"), 0,
    TEXT("Observational only, owned Mode1/PhaseTiming0.0off(default),1SubsystemStep,2SweepDetailed,3Extruded,4BoxTriangle,5BoxPlaneAllProviders,6SweptBoundsAllProviders,7PlaneTriangulationAllProviders. One inclusive phase per run; nested phases must never be added. CPU cycles are not seconds."));
TAutoConsoleVariable<int32> CVarCpuSampleStride627(
    TEXT("p.IAmSpeed.AnalyticWorld.CaseCpuSampleStride627"), 1,
    TEXT("Diagnostic only. Power-of-two stride1..1024;1 preserves full timing. Sampled cycles are not an unbiased full-cost estimate."));
TAutoConsoleVariable<int32> CVarCpuSampleOffset627(
    TEXT("p.IAmSpeed.AnalyticWorld.CaseCpuSampleOffset627"), 0,
    TEXT("Diagnostic only. Case-wide zero-based outermost-call offset in [0,stride). No per-frame reset. Multiple offsets require separate controls."));
struct FPhaseCycles400
{
    uint64 Eligible627=0; // Every outermost selected-phase call, including untimed calls.
    uint64 Calls=0, Valid=0, Zero=0, Unavailable=0, Overflow=0;
    uint64 Cycles=0, Minimum=MAX_uint64, Maximum=0;
};
void AddPhase400(FPhaseCycles400& D, const FPhaseCycles400& S, bool& Invalid) noexcept
{
    if(S.Eligible627>MAX_uint64-D.Eligible627 ||
       S.Calls>MAX_uint64-D.Calls || S.Valid>MAX_uint64-D.Valid ||
       S.Zero>MAX_uint64-D.Zero || S.Unavailable>MAX_uint64-D.Unavailable ||
       S.Overflow>MAX_uint64-D.Overflow || S.Cycles>MAX_uint64-D.Cycles)
    { Invalid=true; return; }
    D.Eligible627+=S.Eligible627;
    D.Calls+=S.Calls; D.Valid+=S.Valid; D.Zero+=S.Zero;
    D.Unavailable+=S.Unavailable; D.Overflow+=S.Overflow; D.Cycles+=S.Cycles;
    if(S.Valid) { D.Minimum=FMath::Min(D.Minimum,S.Minimum); D.Maximum=FMath::Max(D.Maximum,S.Maximum); }
}
struct FCpuAndFrameTotals
{
    uint64 Frames = 0, CpuValidFrames = 0, CpuZeroFrames = 0, CpuUnavailableFrames = 0;
    uint64 User100ns = 0, Kernel100ns = 0;
    uint64 CycleValidFrames = 0, CycleZeroFrames = 0, CycleUnavailableFrames = 0;
    uint64 CycleOverflowFrames = 0, Cycles = 0, MinimumCycles = MAX_uint64, MaximumCycles = 0;
    double WallSeconds = 0.0;
};
struct FOwnerState
{
    FCaseIdentity Case;
    uint32 ThreadId = 0;
    bool bOpen = false, bFrameActive = false, bExtruded = false;
    bool bInvalid = false;
    uint64 FirstFrame = MAX_uint64, LastFrame = 0, NonContiguousFrames = 0;
    // status0=committed, status1=partial/failed. Then provider0=other,1=extruded;
    // then lazy0/1. Partial work is never labelled committed.
    FSatCounters Totals[2][2][2];
    FSatCounters CurrentFrame[2][2];
    FCpuAndFrameTotals FrameTotals[2];
    // Diagnostic TLS only: no public scope/context layout or physical state grows.
    uint64 FrameStartCycles = 0;
    bool bFrameStartCyclesValid = false;
    int32 PhaseSelector400=0;
    int32 SampleStride627=1, SampleOffset627=0;
    bool bSamplingValid627=true;
    uint64 EligibleSequence627=0; // Includes partial frames; reset only with the case owner.
    uint32 PhaseDepth400=0;
    uint64 PhaseFrame400=0;
    FPhaseCycles400 CurrentPhase400, PhaseTotals400[2];
};
thread_local FOwnerState Owner;
std::atomic<uint64> NextFlushSequence{1}; // Outside measured scopes; unique across case/TID reuse in this process.

bool SameCase(const FCaseIdentity& A, const FCaseIdentity& B)
{
    return A.DriverKey==B.DriverKey && A.Epoch==B.Epoch &&
        A.NativeCaseId==B.NativeCaseId && A.BatchIndex==B.BatchIndex &&
        A.WorldGeneration==B.WorldGeneration && A.RunUUID==B.RunUUID && A.Mode==B.Mode;
}
bool ReadCurrentThreadCpu(uint64& User100ns, uint64& Kernel100ns) noexcept
{
    User100ns=0; Kernel100ns=0;
#if PLATFORM_WINDOWS
    FILETIME Creation={}, Exit={}, Kernel={}, User={};
    if (!::GetThreadTimes(::GetCurrentThread(), &Creation, &Exit, &Kernel, &User)) return false;
    User100ns=(uint64(User.dwHighDateTime)<<32) | uint64(User.dwLowDateTime);
    Kernel100ns=(uint64(Kernel.dwHighDateTime)<<32) | uint64(Kernel.dwLowDateTime);
    return true;
#else
    return false; // Portable CPU support is UNKNOWN, never fake zero seconds.
#endif
}
bool ReadCurrentThreadCycles(uint64& Cycles) noexcept
{
    Cycles=0;
#if PLATFORM_WINDOWS
    ULONG64 NativeCycles=0;
    if (!::QueryThreadCycleTime(::GetCurrentThread(), &NativeCycles)) return false;
    Cycles=static_cast<uint64>(NativeCycles);
    return true;
#else
    return false; // Unsupported is UNKNOWN, never valid zero CPU work.
#endif
}
void AddCounters(FSatCounters& D, const FSatCounters& S) noexcept
{
    D.Calls+=S.Calls; D.CrossProducts+=S.CrossProducts;
    D.NormalizedCrossAxes+=S.NormalizedCrossAxes; D.VisitedAxes+=S.VisitedAxes;
    D.ReachedFirstValidCross+=S.ReachedFirstValidCross; D.CompletedAllAxes+=S.CompletedAllAxes;
    D.PostSatShellRejected+=S.PostSatShellRejected; D.Contacts+=S.Contacts;
    for(int32 I=0; I<13; ++I) { D.StationaryReject[I]+=S.StationaryReject[I]; D.IntervalReject[I]+=S.IntervalReject[I]; }
    for(int32 I=0; I<9; ++I) D.DegenerateCrossSlots[I]+=S.DegenerateCrossSlots[I];
}
template<int32 N> FString Counts(const uint64 (&Values)[N])
{
    FString Result;
    for(int32 I=0; I<N; ++I) { if(I) Result+=TEXT(","); Result+=FString::Printf(TEXT("%llu"),Values[I]); }
    return Result;
}
}

bool TryCopyOwnedForwardIdentity917(FOwnedForwardIdentity917& Out) noexcept
{
    Out=FOwnedForwardIdentity917{};
    if(!Owner.bOpen || !Owner.bFrameActive || Owner.bInvalid || !Owner.Case.bEnabled ||
       Owner.Case.Mode!=1 || Owner.ThreadId==0 || Owner.ThreadId!=FPlatformTLS::GetCurrentThreadId()) return false;
    const FCaseIdentity& C=Owner.Case;
    const int32 Length=C.RunUUID.Len();
    if(Length<=0 || Length>64 || C.DriverKey==0 || C.Epoch==0 || C.NativeCaseId<0 || C.BatchIndex<0 || C.WorldGeneration==0) return false;
    // PhaseFrame400 is assigned unconditionally from CanonicalFrame in FScopedFrame,
    // including selector0. No new lifecycle state or renamed frame numbering.
    Out.DriverKey=C.DriverKey;Out.Epoch=C.Epoch;Out.NativeCaseId=C.NativeCaseId;
    Out.BatchIndex=C.BatchIndex;Out.WorldGeneration=C.WorldGeneration;
    Out.CanonicalFrame=Owner.PhaseFrame400;Out.Mode=C.Mode;Out.ThreadId=Owner.ThreadId;
    for(int32 I=0;I<Length;++I) Out.RunUUID[I]=C.RunUUID[I];
    Out.RunUUID[Length]=TCHAR(0);
    return true;
}

bool IsEnabled() { return CVarCaseSatDiagnostics.GetValueOnAnyThread()!=0; }

bool BuildCaseIdentity(FCaseIdentity& Out, uint64 DriverKey, uint64 Epoch,
    int32 CaseId, int32 BatchIndex, uint64 WorldGeneration)
{
    Out=FCaseIdentity{};
    if(!IsEnabled()) return true;
    FString UUID;
    const int32 Mode=CVarCaseSatDiagnostics.GetValueOnAnyThread();
    const IConsoleVariable* Phase=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"));
    if(!FParse::Value(FCommandLine::Get(),TEXT("SLCaseSatRunUUID="),UUID) || UUID.IsEmpty() || UUID.Len()>64 ||
        UUID.Contains(TEXT(" ")) || UUID.Contains(TEXT("\n")) || UUID.Contains(TEXT("\r")) ||
        !Phase || (Mode!=1 && Mode!=2) || Phase->GetInt()!=(Mode==1?0:2) || CaseId<0 || DriverKey==0 || Epoch==0)
    {
        UE_LOG(LogTemp, Warning, TEXT("[CaseSatUnavailable] Status=INVALID_CONFIG Reason=MissingIdentityOrPhaseModeMismatch DiagnosticOnly=1"));
        return false;
    }
    for(const TCHAR C : UUID) if(!FChar::IsAlnum(C) && C!=TCHAR('-'))
    {
        UE_LOG(LogTemp, Warning, TEXT("[CaseSatUnavailable] Status=INVALID_CONFIG Reason=UnsafeRunUUID DiagnosticOnly=1"));
        return false;
    }
    Out.RunUUID=MoveTemp(UUID); Out.DriverKey=DriverKey; Out.Epoch=Epoch;
    Out.NativeCaseId=CaseId; Out.BatchIndex=BatchIndex; Out.WorldGeneration=WorldGeneration;
    Out.Mode=Mode; Out.bEnabled=true;
    return true;
}

FScopedFrame::FScopedFrame(const FCaseIdentity& Case, uint64 CanonicalFrame) noexcept
{
    if(!Case.bEnabled) return;
    try
    {
        if(Owner.bFrameActive) { Owner.bInvalid=true; return; } // Nested scopes are unqualified.
        if(Owner.bOpen && !SameCase(Owner.Case,Case))
        { Owner.bInvalid=true; FlushOnOwnerThread(TEXT("UnexpectedCaseSwitch")); }
        if(!Owner.bOpen)
        {
            Owner=FOwnerState{};
            Owner.Case=Case;
            Owner.ThreadId=FPlatformTLS::GetCurrentThreadId();
            Owner.bOpen=true;
            Owner.PhaseSelector400=CVarCpuPhase400.GetValueOnAnyThread();
            Owner.SampleStride627=CVarCpuSampleStride627.GetValueOnAnyThread();
            Owner.SampleOffset627=CVarCpuSampleOffset627.GetValueOnAnyThread();
            const int32 Stride=Owner.SampleStride627;
            Owner.bSamplingValid627=Stride>=1 && Stride<=1024 && (Stride&(Stride-1))==0 &&
                Owner.SampleOffset627>=0 && Owner.SampleOffset627<Stride;
            if(!Owner.bSamplingValid627) Owner.bInvalid=true; // UNKNOWN, never block physics.
            if(Owner.PhaseSelector400<0 || Owner.PhaseSelector400>7 ||
               (Owner.PhaseSelector400!=0 && Case.Mode!=1)) Owner.bInvalid=true;
        }
        if(Owner.ThreadId!=FPlatformTLS::GetCurrentThreadId()) { Owner.bInvalid=true; return; }
        const IConsoleVariable* Phase=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"));
        if(CVarCaseSatDiagnostics.GetValueOnAnyThread()!=Case.Mode || !Phase || Phase->GetInt()!=(Case.Mode==1?0:2))
            Owner.bInvalid=true; // Report UNKNOWN; never rewrite configuration or block physics.
        for(int32 P=0; P<2; ++P) for(int32 L=0; L<2; ++L) Owner.CurrentFrame[P][L]=FSatCounters{};
        Owner.bExtruded=false; Owner.bFrameActive=true;
        Frame=CanonicalFrame; StartThreadId=Owner.ThreadId;
        if(Owner.PhaseSelector400!=CVarCpuPhase400.GetValueOnAnyThread() || Owner.PhaseDepth400!=0 ||
           Owner.SampleStride627!=CVarCpuSampleStride627.GetValueOnAnyThread() ||
           Owner.SampleOffset627!=CVarCpuSampleOffset627.GetValueOnAnyThread())
            Owner.bInvalid=true;
        Owner.PhaseFrame400=CanonicalFrame;
        Owner.CurrentPhase400=FPhaseCycles400{};
        bCpuStartValid=ReadCurrentThreadCpu(CpuStartUser100ns,CpuStartKernel100ns);
        WallStart=FPlatformTime::Seconds();
        Owner.bFrameStartCyclesValid=ReadCurrentThreadCycles(Owner.FrameStartCycles);
        bActive=true;
    }
    catch(...) { Owner.bInvalid=true; Owner.bFrameActive=false; }
}
FScopedFrame::~FScopedFrame() noexcept { Finish(false); }
void FScopedFrame::Finish(bool bCommitted) noexcept
{
    if(!bActive || bFinished) return;
    bFinished=true;
    uint64 CyclesEnd=0;
    const bool bCyclesEndValid=ReadCurrentThreadCycles(CyclesEnd);
    uint64 UserEnd=0, KernelEnd=0;
    const bool bCpuEndValid=ReadCurrentThreadCpu(UserEnd,KernelEnd);
    const double WallEnd=FPlatformTime::Seconds();
    const bool bSameThread=StartThreadId==FPlatformTLS::GetCurrentThreadId();
    if(!bSameThread) return; // Do not mutate a different thread's owner/TLS. Original open scope cannot close => UNKNOWN.
    Owner.bFrameActive=false; Owner.bExtruded=false;
    const int32 Status=bCommitted?0:1;
    FCpuAndFrameTotals& T=Owner.FrameTotals[Status]; ++T.Frames;
    if(Owner.PhaseDepth400!=0) Owner.bInvalid=true;
    AddPhase400(Owner.PhaseTotals400[Status],Owner.CurrentPhase400,Owner.bInvalid);
    if(bSameThread && bCpuStartValid && bCpuEndValid &&
        UserEnd>=CpuStartUser100ns && KernelEnd>=CpuStartKernel100ns)
    {
        const uint64 U=UserEnd-CpuStartUser100ns, K=KernelEnd-CpuStartKernel100ns;
        ++T.CpuValidFrames; T.CpuZeroFrames+=(U==0 && K==0)?1:0;
        T.User100ns+=U; T.Kernel100ns+=K;
    }
    else { ++T.CpuUnavailableFrames; }
    if(Owner.bFrameStartCyclesValid && bCyclesEndValid && CyclesEnd>=Owner.FrameStartCycles)
    {
        const uint64 Delta=CyclesEnd-Owner.FrameStartCycles;
        if(Delta<=MAX_uint64-T.Cycles)
        {
            ++T.CycleValidFrames; T.CycleZeroFrames+=Delta==0?1:0;
            T.Cycles+=Delta;
            T.MinimumCycles=FMath::Min(T.MinimumCycles,Delta);
            T.MaximumCycles=FMath::Max(T.MaximumCycles,Delta);
        }
        else { ++T.CycleUnavailableFrames; ++T.CycleOverflowFrames; }
    }
    else { ++T.CycleUnavailableFrames; }
    T.WallSeconds+=FMath::Max(0.0,WallEnd-WallStart);
    if(!bSameThread) Owner.bInvalid=true;
    if(Owner.FirstFrame==MAX_uint64) Owner.FirstFrame=Frame;
    else if(Frame!=Owner.LastFrame+1) { ++Owner.NonContiguousFrames; Owner.bInvalid=true; }
    Owner.LastFrame=Frame;
    for(int32 P=0; P<2; ++P) for(int32 L=0; L<2; ++L) AddCounters(Owner.Totals[Status][P][L],Owner.CurrentFrame[P][L]);
}

FScopedCpuPhase400::FScopedCpuPhase400(ECpuPhase400 Phase) noexcept
{
    if(!Owner.bOpen || !Owner.bFrameActive || Owner.Case.Mode!=1 ||
       Owner.PhaseSelector400!=static_cast<int32>(Phase)) return;
    ThreadId=FPlatformTLS::GetCurrentThreadId();
    if(ThreadId!=Owner.ThreadId || Owner.PhaseDepth400==MAX_uint32)
    { Owner.bInvalid=true; return; }
    Frame=Owner.PhaseFrame400; bEntered=true;
    bOuter=(Owner.PhaseDepth400++==0);
    if(bOuter)
    {
        FPhaseCycles400& T=Owner.CurrentPhase400;
        if(T.Eligible627==MAX_uint64 || Owner.EligibleSequence627==MAX_uint64)
        { Owner.bInvalid=true; bOuter=false; return; }
        const uint64 Index=Owner.EligibleSequence627++;
        ++T.Eligible627;
        // Keep depth entered even for untimed outer scopes: recursion must stay inclusive.
        bOuter=Owner.bSamplingValid627 &&
            (Index & static_cast<uint64>(Owner.SampleStride627-1))==static_cast<uint64>(Owner.SampleOffset627);
        if(bOuter) bStartValid=ReadCurrentThreadCycles(Start);
    }
}
FScopedCpuPhase400::~FScopedCpuPhase400() noexcept
{
    if(!bEntered || ThreadId!=FPlatformTLS::GetCurrentThreadId()) return;
    if(!Owner.bOpen || !Owner.bFrameActive || Frame!=Owner.PhaseFrame400 || Owner.PhaseDepth400==0)
    { Owner.bInvalid=true; return; }
    --Owner.PhaseDepth400;
    if(!bOuter) return; // Outermost inclusive span only; recursion is not summed.
    if(Owner.PhaseDepth400!=0) { Owner.bInvalid=true; return; }
    uint64 End=0; const bool bEndValid=ReadCurrentThreadCycles(End);
    FPhaseCycles400& T=Owner.CurrentPhase400;
    if(T.Calls==MAX_uint64) { Owner.bInvalid=true; return; }
    ++T.Calls;
    if(!bStartValid || !bEndValid || End<Start) { ++T.Unavailable; return; }
    const uint64 Delta=End-Start;
    if(Delta>MAX_uint64-T.Cycles) { ++T.Unavailable; ++T.Overflow; return; }
    ++T.Valid; T.Zero+=Delta==0?1:0; T.Cycles+=Delta;
    T.Minimum=FMath::Min(T.Minimum,Delta); T.Maximum=FMath::Max(T.Maximum,Delta);
}

FScopedExtrudedProvider::FScopedExtrudedProvider() noexcept : bPrevious(Owner.bExtruded)
{ if(Owner.bFrameActive) Owner.bExtruded=true; }
FScopedExtrudedProvider::~FScopedExtrudedProvider() noexcept { Owner.bExtruded=bPrevious; }
FSatCounters* GetActiveSatCounters(bool bLazy) noexcept
{
    return Owner.bOpen && Owner.bFrameActive && Owner.Case.Mode==2 ? &Owner.CurrentFrame[Owner.bExtruded?1:0][bLazy?1:0] : nullptr;
}

void FlushOnOwnerThread(const TCHAR* Reason) noexcept
{
    if(!Owner.bOpen) return; // Idempotent: a later close cannot duplicate a case.
    if(Owner.bFrameActive) { Owner.bInvalid=true; return; } // Missing final marker => UNKNOWN.
    Owner.bOpen=false; // Invalidate attribution BEFORE output or owner teardown.
    const uint64 Sequence=NextFlushSequence.fetch_add(1,std::memory_order_relaxed);
    if(Sequence==0 || Sequence==MAX_uint64) Owner.bInvalid=true;
    try
    {
        const FCaseIdentity& C=Owner.Case;
        const bool bSameThread=Owner.ThreadId==FPlatformTLS::GetCurrentThreadId();
        UE_LOG(LogTemp, Display, TEXT("[CaseSatFlushBegin] Run=%s Driver=%llu Epoch=%llu Case=%d Batch=%d WorldGeneration=%llu Thread=%u Flush=%llu FirstFrame=%llu LastFrame=%llu NonContiguous=%llu Invalid=%d SameThread=%d Reason=%s Rows=8 ExpectedFrames=UNKNOWN Scope=OwnedStepOnly OutsideOrChildQueries=UNKNOWN RollbackCPU=UNKNOWN Mode=%d SATCoverage=%s"),
            *C.RunUUID,C.DriverKey,C.Epoch,C.NativeCaseId,C.BatchIndex,C.WorldGeneration,Owner.ThreadId,Sequence,
            Owner.FirstFrame,Owner.LastFrame,Owner.NonContiguousFrames,Owner.bInvalid?1:0,bSameThread?1:0,Reason,C.Mode,C.Mode==2?TEXT("OwnedStepOnly"):TEXT("DISABLED_CPU_ONLY"));
        for(int32 S=0; S<2; ++S)
        {
            const FCpuAndFrameTotals& T=Owner.FrameTotals[S];
            UE_LOG(LogTemp, Display, TEXT("[CaseSatThreadCPU] Run=%s Driver=%llu Epoch=%llu Case=%d Batch=%d WorldGeneration=%llu Thread=%u Flush=%llu Status=%s Frames=%llu CPUValidFrames=%llu CPUZeroFrames=%llu CPUUnavailableFrames=%llu User100ns=%llu Kernel100ns=%llu WallSeconds=%.9f CPUResolution=UNKNOWN Mode=%d CPUMeaning=%s"),
                *C.RunUUID,C.DriverKey,C.Epoch,C.NativeCaseId,C.BatchIndex,C.WorldGeneration,Owner.ThreadId,Sequence,
                S?TEXT("Partial"):TEXT("Committed"),T.Frames,T.CpuValidFrames,T.CpuZeroFrames,T.CpuUnavailableFrames,T.User100ns,T.Kernel100ns,T.WallSeconds,C.Mode,C.Mode==2?TEXT("DETAILED_INSTRUMENTED_STEP"):TEXT("CPU_ONLY_STEP"));
            // Separate ancillary row keeps the exact231 CPU/work/flush protocol intact.
            // CPU cycles are hardware/OS counters, not seconds; no frequency conversion.
            UE_LOG(LogTemp, Display, TEXT("[CaseSatThreadCycles365] Run=%s Driver=%llu Epoch=%llu Case=%d Batch=%d WorldGeneration=%llu Thread=%u Flush=%llu Status=%s Frames=%llu CycleValidFrames=%llu CycleZeroFrames=%llu CycleUnavailableFrames=%llu CycleOverflowFrames=%llu Cycles=%llu MinCycles=%llu MaxCycles=%llu Clock=QueryThreadCycleTime ClockResolution=UNKNOWN ElapsedSeconds=UNKNOWN Mode=%d Scope=OwnedStepCallerPlusClockGuards CycleRows=2"),
                *C.RunUUID,C.DriverKey,C.Epoch,C.NativeCaseId,C.BatchIndex,C.WorldGeneration,Owner.ThreadId,Sequence,
                S?TEXT("Partial"):TEXT("Committed"),T.Frames,T.CycleValidFrames,T.CycleZeroFrames,T.CycleUnavailableFrames,T.CycleOverflowFrames,T.Cycles,
                T.CycleValidFrames?T.MinimumCycles:0,T.CycleValidFrames?T.MaximumCycles:0,C.Mode);
            if(Owner.PhaseSelector400!=0)
            {
                const FPhaseCycles400& Q=Owner.PhaseTotals400[S];
                UE_LOG(LogTemp, Display, TEXT("[CaseCpuPhase400] Run=%s Driver=%llu Epoch=%llu Case=%d Batch=%d WorldGeneration=%llu Thread=%u Flush=%llu Status=%s Frames=%llu Selector=%d Calls=%llu ValidCalls=%llu ZeroCalls=%llu UnavailableCalls=%llu OverflowCalls=%llu Cycles=%llu MinCycles=%llu MaxCycles=%llu Mode=%d Clock=QueryThreadCycleTime ElapsedSeconds=UNKNOWN Scope=OwnedForwardInclusive SelectedPhases=1 TimerOverhead=UNKNOWN OutsideForward=UNKNOWN PhaseRows=2"),
                    *C.RunUUID,C.DriverKey,C.Epoch,C.NativeCaseId,C.BatchIndex,C.WorldGeneration,Owner.ThreadId,Sequence,
                    S?TEXT("Partial"):TEXT("Committed"),T.Frames,Owner.PhaseSelector400,Q.Calls,Q.Valid,Q.Zero,Q.Unavailable,Q.Overflow,Q.Cycles,Q.Valid?Q.Minimum:0,Q.Valid?Q.Maximum:0,C.Mode);
                UE_LOG(LogTemp, Display, TEXT("[CaseCpuSampling627] Run=%s Driver=%llu Epoch=%llu Case=%d Batch=%d WorldGeneration=%llu Thread=%u Flush=%llu Status=%s Frames=%llu Selector=%d Stride=%d Offset=%d ConfigValid=%d EligibleCalls=%llu SampledCalls=%llu CaseEligibleCalls=%llu Sequence=CaseWideZeroBased IncludePartial=1 Scope=OutermostSelectedPhase CallsMeaning=Sampled TimedCyclesMeaning=SampleOnly FullCost=UNKNOWN SamplingBias=UNKNOWN SamplingRows=2"),
                    *C.RunUUID,C.DriverKey,C.Epoch,C.NativeCaseId,C.BatchIndex,C.WorldGeneration,Owner.ThreadId,Sequence,
                    S?TEXT("Partial"):TEXT("Committed"),T.Frames,Owner.PhaseSelector400,Owner.SampleStride627,Owner.SampleOffset627,
                    Owner.bSamplingValid627?1:0,Q.Eligible627,Q.Calls,Owner.EligibleSequence627);
            }
            for(int32 P=0; P<2; ++P) for(int32 L=0; L<2; ++L)
            {
                const FSatCounters& Tally=Owner.Totals[S][P][L];
                UE_LOG(LogTemp, Display, TEXT("[CaseSatWork] Run=%s Driver=%llu Epoch=%llu Case=%d Batch=%d WorldGeneration=%llu Thread=%u Flush=%llu Status=%s Provider=%s Lazy=%d Calls=%llu PotentialCrossProducts=%llu CrossProducts=%llu NormalizedCrossAxes=%llu VisitedAxes=%llu ReachedFirstValidCross=%llu CompletedAllAxes=%llu PostSatShellRejected=%llu Contacts=%llu StationaryReject=%s IntervalReject=%s DegenerateCrossSlots=%s"),
                    *C.RunUUID,C.DriverKey,C.Epoch,C.NativeCaseId,C.BatchIndex,C.WorldGeneration,Owner.ThreadId,Sequence,
                    S?TEXT("Partial"):TEXT("Committed"),P?TEXT("Extruded"):TEXT("Other"),L,
                    Tally.Calls,9*Tally.Calls,Tally.CrossProducts,Tally.NormalizedCrossAxes,Tally.VisitedAxes,
                    Tally.ReachedFirstValidCross,Tally.CompletedAllAxes,Tally.PostSatShellRejected,Tally.Contacts,
                    *Counts(Tally.StationaryReject),*Counts(Tally.IntervalReject),*Counts(Tally.DegenerateCrossSlots));
            }
        }
        Private::FReplayProbe925::Flush(C);
        UE_LOG(LogTemp, Display, TEXT("[CaseSatFlushEnd] Run=%s Driver=%llu Epoch=%llu Case=%d Batch=%d WorldGeneration=%llu Thread=%u Flush=%llu Rows=8 CPURows=2 Complete=%d"),
            *C.RunUUID,C.DriverKey,C.Epoch,C.NativeCaseId,C.BatchIndex,C.WorldGeneration,Owner.ThreadId,Sequence,
            bSameThread && !Owner.bInvalid ? 1:0);
    }
    catch(...) { /* Diagnostic output failure must never skip physical input-owner close. Missing marker => UNKNOWN. */ }
    Owner.bFrameActive=false; Owner.bExtruded=false;
}
}
#endif
