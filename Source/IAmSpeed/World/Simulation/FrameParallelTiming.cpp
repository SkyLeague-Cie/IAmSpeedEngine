#include "FrameParallelTiming.h"
#if !UE_BUILD_SHIPPING
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTLS.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#if PLATFORM_WINDOWS
#include "Windows/WindowsHWrapper.h"
#endif
namespace Speed::ParallelTiming
{
namespace
{
TAutoConsoleVariable<int32> CVarFrameTiming(
    TEXT("p.IAmSpeed.Parallel.FrameTiming"), 0,
    TEXT("Owned QPC canonical-frame observation, independent of CaseSat:0off,1bounded raw per-case,2case aggregates. Requires SLParallelTimingRunUUID. Never enables task workers."));
TAutoConsoleVariable<int32> CVarFrameCostPhases(
    TEXT("p.IAmSpeed.Parallel.FrameCostPhases"), 0,
    TEXT("Opt-in exclusive complete canonical-frame phase QPC intervals. Requires FrameTiming1; no ActorDiagnostics/CaseSat dependency or worker-policy change.0off1on."));
bool Counter(uint64& Out) noexcept
{
    Out=0;
#if PLATFORM_WINDOWS
    LARGE_INTEGER V={};if(::QueryPerformanceCounter(&V)&&V.QuadPart>=0){Out=uint64(V.QuadPart);return true;}
#endif
    return false;
}
uint64 QpcFrequency() noexcept
{
#if PLATFORM_WINDOWS
    LARGE_INTEGER V={};if(::QueryPerformanceFrequency(&V)&&V.QuadPart>0)return uint64(V.QuadPart);
#endif
    return 0;
}
bool ValidRun(const FString& V)
{
    if(V.IsEmpty()||V.Len()>64)return false;
    for(TCHAR C:V)if(!FChar::IsAlnum(C)&&C!=TEXT('-')&&C!=TEXT('_')&&C!=TEXT('.'))return false;
    return true;
}
uint64 Quantile(const TArray<uint64>& Sorted,int32 Percent)
{
    if(Sorted.IsEmpty())return 0;
    return Sorted[FMath::Clamp((Percent*Sorted.Num()+99)/100-1,0,Sorted.Num()-1)];
}
}
void FCaseRecorder::ConfigureAfterJoin(uint64 InDriver,int32 InCase,int32 InBatch,uint64 WorldGeneration)
{
    // Called only after the existing joined-owner fence. No new pause/join,
    // owner retirement, canonical reset, input or physical state is performed.
    if(bOpen){bInvalid=true;return;}
    bJoinedCaseLogClosed=false;
    JoinedCaseContext = FString::Printf(TEXT("d%llu-c%d-b%d-g%llu"),InDriver,InCase,InBatch,WorldGeneration);
    Mode=CVarFrameTiming.GetValueOnAnyThread();bConfigured=false;
    bPhaseTiming=Mode==1 && CVarFrameCostPhases.GetValueOnAnyThread()==1;
    if(Mode==0)return;
    FString Candidate;
    FParse::Value(FCommandLine::Get(),TEXT("SLParallelTimingRunUUID="),Candidate);
    if((Mode!=1&&Mode!=2)||!ValidRun(Candidate)||InDriver==0||InCase<0||InBatch<0||WorldGeneration==0||NextEpoch==MAX_uint64)
    {
        UE_LOG(LogTemp,Warning,TEXT("[IAmSpeedParallelTimingUnavailableV1] Case=%d Reason=MissingOwnedIdentity Scope=TimingOnly"),InCase);
        return;
    }
    Run=MoveTemp(Candidate);Driver=InDriver;Epoch=++NextEpoch;NativeCase=InCase;BatchIndex=InBatch;Generation=WorldGeneration;
    Attempts=Dropped=0;OwnerThread=0;Frequency=QpcFrequency();bInvalid=Frequency==0;
    Samples.Reset();Samples.Reserve(4096);bConfigured=true;
}
void FCaseRecorder::BeginDriver(bool InFast,uint64 Before,bool bOwnedLane) noexcept
{
    if(!bConfigured)return;
    if(bOpen){if(bFast!=InFast||OwnerThread!=FPlatformTLS::GetCurrentThreadId())bInvalid=true;return;}
    if(!bOwnedLane){bInvalid=true;return;}
    bOpen=true;bFast=InFast;OwnerThread=FPlatformTLS::GetCurrentThreadId();CanonicalBefore=Before;
    UE_LOG(LogTemp,Display,TEXT("[IAmSpeedParallelCaseBeginV1] Run=%s Driver=%llu Epoch=%llu Case=%d Batch=%d WorldGeneration=%llu Thread=%u ExecutionMode=%s Mode=%d CanonicalBefore=%llu QpcFrequency=%llu Clock=WindowsQPC IdentityKnown=1"),
        *Run,Driver,Epoch,NativeCase,BatchIndex,Generation,OwnerThread,bFast?TEXT("Fast"):TEXT("RealTime"),Mode,CanonicalBefore,Frequency);
}
void FCaseRecorder::Record(const FFrameSample& S) noexcept
{
    if(!bOpen)return;
    ++Attempts;
    if(OwnerThread!=FPlatformTLS::GetCurrentThreadId()||!S.bCommitted||!S.bClockValid||S.End<S.Begin)bInvalid=true;
    if(!Samples.IsEmpty()&&S.Frame!=Samples.Last().Frame+1)bInvalid=true;
    if(Samples.Num()==4096){++Dropped;bInvalid=true;return;}
    Samples.Add(S);
}
void FCaseRecorder::Close(bool bCompleted,const TCHAR* Reason) noexcept
{
    if(!bOpen)return;
    if(OwnerThread!=FPlatformTLS::GetCurrentThreadId()||!bCompleted||Attempts==0)bInvalid=true;
    TArray<uint64> Sorted;Sorted.Reserve(Samples.Num());
    uint64 Sum=0,Committed=0,UnknownWork=0;int32 LogicalMax=0,NativeMax=0;uint32 FallbackMask=0;
    uint64 FacetProviders=0,ParallelFacetProviders=0,EvaluatedFacets=0,ParallelPrepareCalls=0,ParallelQueryCalls=0,ParallelCameraCalls=0;
    int32 FacetNativeMax=0,PrepareNativeMax=0,QueryNativeMax=0,CameraNativeMax=0;
    for(const FFrameSample& S:Samples)
    {
        const uint64 Duration=S.bClockValid&&S.End>=S.Begin?S.End-S.Begin:0;
        if(Duration>MAX_uint64-Sum)bInvalid=true;else Sum+=Duration;
        Sorted.Add(Duration);Committed+=S.bCommitted;UnknownWork+=!S.Work.bKnown;
        LogicalMax=FMath::Max(LogicalMax,S.Work.LogicalMax);NativeMax=FMath::Max(NativeMax,S.Work.NativeMax);FallbackMask|=S.Work.FallbackMask;
        if(S.Work.FacetProviders>MAX_uint64-FacetProviders)bInvalid=true;else FacetProviders+=S.Work.FacetProviders;
        if(S.Work.ParallelFacetProviders>MAX_uint64-ParallelFacetProviders)bInvalid=true;else ParallelFacetProviders+=S.Work.ParallelFacetProviders;
        if(S.Work.EvaluatedFacets>MAX_uint64-EvaluatedFacets)bInvalid=true;else EvaluatedFacets+=S.Work.EvaluatedFacets;
        if(S.Work.ParallelPrepareCalls>MAX_uint64-ParallelPrepareCalls)bInvalid=true;else ParallelPrepareCalls+=S.Work.ParallelPrepareCalls;
        if(S.Work.ParallelQueryCalls>MAX_uint64-ParallelQueryCalls)bInvalid=true;else ParallelQueryCalls+=S.Work.ParallelQueryCalls;
        if(S.Work.ParallelCameraCalls>MAX_uint64-ParallelCameraCalls)bInvalid=true;else ParallelCameraCalls+=S.Work.ParallelCameraCalls;
        FacetNativeMax=FMath::Max(FacetNativeMax,S.Work.FacetNativeMax);
        PrepareNativeMax=FMath::Max(PrepareNativeMax,S.Work.PrepareNativeMax);
        QueryNativeMax=FMath::Max(QueryNativeMax,S.Work.QueryNativeMax);
        CameraNativeMax=FMath::Max(CameraNativeMax,S.Work.CameraNativeMax);
        if(Mode==1)
        {
            if(RawRowsWritten==65536){++Dropped;bInvalid=true;continue;}
            ++RawRowsWritten;
            UE_LOG(LogTemp,Display,TEXT("[IAmSpeedParallelFrameV1] Run=%s Driver=%llu Epoch=%llu Case=%d Frame=%llu Committed=%d Hz=300 QpcBegin=%llu QpcEnd=%llu QpcFrequency=%llu Requested=%d PolicyLanes=%d LogicalMax=%d NativeMax=%d FallbackMask=%u WorkKnown=%d FacetProviders=%llu ParallelFacetProviders=%llu EvaluatedFacets=%llu ParallelPrepareCalls=%llu ParallelQueryCalls=%llu ParallelCameraCalls=%llu FacetNativeMax=%d PrepareNativeMax=%d QueryNativeMax=%d CameraNativeMax=%d StateHash=%016llX InputHash=%016llX"),
                *Run,Driver,Epoch,NativeCase,S.Frame,S.bCommitted?1:0,S.Begin,S.End,Frequency,
                S.Work.Requested,S.Work.PolicyLanes,S.Work.LogicalMax,S.Work.NativeMax,S.Work.FallbackMask,S.Work.bKnown?1:0,S.Work.FacetProviders,S.Work.ParallelFacetProviders,S.Work.EvaluatedFacets,S.Work.ParallelPrepareCalls,S.Work.ParallelQueryCalls,S.Work.ParallelCameraCalls,S.Work.FacetNativeMax,S.Work.PrepareNativeMax,S.Work.QueryNativeMax,S.Work.CameraNativeMax,S.StateHash,S.InputHash);
            if (bPhaseTiming)
            {
                uint64 PhaseSum=0;bool Known=S.Phases.bTracked&&S.Phases.bValid;
                for(uint64 Tick:S.Phases.Ticks)
                {if(Tick>MAX_uint64-PhaseSum)Known=false;else PhaseSum+=Tick;}
                Known=Known&&S.Phases.SeenMask==((1u<<FrameCostPhaseCount)-1u)&&PhaseSum==Duration;
                UE_LOG(LogTemp,Display,TEXT("[IAmSpeedParallelPhaseFrameV1] Run=%s Driver=%llu Epoch=%llu Case=%d Frame=%llu ExecutionMode=%s Committed=%d Hz=300 QpcBegin=%llu QpcEnd=%llu QpcFrequency=%llu PhaseKnown=%d SeenMask=%u AdmissionTicks=%llu InputTicks=%llu PrepareTicks=%llu WorldStepTicks=%llu SnapshotTicks=%llu PresentationTicks=%llu PublishTicks=%llu PostCommitTicks=%llu FinalizeTicks=%llu PhaseSumTicks=%llu Scope=CompleteCanonicalFrameBody ExclusiveTopLevel=1 PacingExcluded=1 ObserverOverheadIncluded=1"),
                    *Run,Driver,Epoch,NativeCase,S.Frame,bFast?TEXT("Fast"):TEXT("RealTime"),S.bCommitted?1:0,S.Begin,S.End,Frequency,Known?1:0,S.Phases.SeenMask,
                    S.Phases.Ticks[0],S.Phases.Ticks[1],S.Phases.Ticks[2],S.Phases.Ticks[3],S.Phases.Ticks[4],S.Phases.Ticks[5],S.Phases.Ticks[6],S.Phases.Ticks[7],S.Phases.Ticks[8],PhaseSum);
            }
        }
    }
    Sorted.Sort();
    UE_LOG(LogTemp,Display,TEXT("[IAmSpeedParallelCaseEndV1] Run=%s Driver=%llu Epoch=%llu Case=%d Batch=%d WorldGeneration=%llu Thread=%u ExecutionMode=%s Mode=%d Complete=%d Valid=%d Attempts=%llu Committed=%llu Samples=%d Dropped=%llu First=%llu Last=%llu QpcFrequency=%llu SumTicks=%llu P50Ticks=%llu P95Ticks=%llu P99Ticks=%llu MaxTicks=%llu LogicalMax=%d NativeMax=%d FallbackMask=%u UnknownWork=%llu FacetProviders=%llu ParallelFacetProviders=%llu EvaluatedFacets=%llu ParallelPrepareCalls=%llu ParallelQueryCalls=%llu ParallelCameraCalls=%llu FacetNativeMax=%d PrepareNativeMax=%d QueryNativeMax=%d CameraNativeMax=%d EndReason=%s Quantile=NearestRank Scope=CanonicalFrameBody PacingExcluded=1"),
        *Run,Driver,Epoch,NativeCase,BatchIndex,Generation,OwnerThread,bFast?TEXT("Fast"):TEXT("RealTime"),Mode,bCompleted?1:0,bInvalid?0:1,
        Attempts,Committed,Samples.Num(),Dropped,Samples.IsEmpty()?0:Samples[0].Frame,Samples.IsEmpty()?0:Samples.Last().Frame,Frequency,Sum,
        Quantile(Sorted,50),Quantile(Sorted,95),Quantile(Sorted,99),Sorted.IsEmpty()?0:Sorted.Last(),LogicalMax,NativeMax,FallbackMask,UnknownWork,FacetProviders,ParallelFacetProviders,EvaluatedFacets,ParallelPrepareCalls,ParallelQueryCalls,ParallelCameraCalls,FacetNativeMax,PrepareNativeMax,QueryNativeMax,CameraNativeMax,Reason?Reason:TEXT("Unknown"));
    bOpen=false;bConfigured=false;
}
FFrameScope::FFrameScope(FCaseRecorder& Owner,uint64 Frame,float Delta) noexcept:Recorder(Owner)
{
    bTracked=Recorder.IsActive();if(!bTracked)return;
    Sample.Frame=Frame;Sample.bClockValid=Delta==1.0f/300.0f&&Counter(Sample.Begin);
    Sample.Phases.bTracked=Recorder.IsPhaseTimingEnabled();
    Sample.Phases.bValid=Sample.bClockValid;
    Sample.Phases.SeenMask=Sample.Phases.bTracked?1u:0u;
    LastPhaseTick=Sample.Begin;
}
void FFrameScope::AccumulatePhase(uint64 Now) noexcept
{
    if(!Sample.Phases.bTracked)return;
    const int32 Index=static_cast<int32>(Phase);
    if(!Sample.bClockValid||Now<LastPhaseTick||Index<0||Index>=FrameCostPhaseCount)
    {Sample.Phases.bValid=false;return;}
    const uint64 Elapsed=Now-LastPhaseTick;
    if(Elapsed>MAX_uint64-Sample.Phases.Ticks[Index])Sample.Phases.bValid=false;
    else Sample.Phases.Ticks[Index]+=Elapsed;
    LastPhaseTick=Now;
}
void FFrameScope::NextPhase(EFrameCostPhase Next) noexcept
{
    if(!bTracked||!Sample.Phases.bTracked)return;
    const int32 Index=static_cast<int32>(Next);uint64 Now=0;
    if(Index<0||Index>=FrameCostPhaseCount||!Counter(Now))
    {Sample.Phases.bValid=false;return;}
    AccumulatePhase(Now);Phase=Next;Sample.Phases.SeenMask|=1u<<Index;
}
void FFrameScope::MarkCommitted(uint64 StateHash,uint64 InputHash,const FFrameWorkEvidence& Work) noexcept
{
    Sample.bCommitted=true;Sample.StateHash=StateHash;Sample.InputHash=InputHash;Sample.Work=Work;
    // QPC closes in the first-constructed function boundary's destructor, after
    // later scope-exit work has completed. Publication alone is not frame end.
}
void FFrameScope::Finish(bool bCommitted,uint64 StateHash,uint64 InputHash,const FFrameWorkEvidence& Work) noexcept
{
    if(!bTracked||bFinished)return;bFinished=true;
    Sample.bClockValid=Counter(Sample.End)&&Sample.bClockValid;
    AccumulatePhase(Sample.End);
    Sample.bCommitted=bCommitted;Sample.StateHash=StateHash;Sample.InputHash=InputHash;Sample.Work=Work;
    Recorder.Record(Sample);
}
}
#endif
