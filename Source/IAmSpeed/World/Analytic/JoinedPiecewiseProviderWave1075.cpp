#include "JoinedPiecewiseProviderWave1075.h"
#include "IAmSpeed/World/Collision/StaticQueryBatch.h"
#include "IAmSpeed/World/Simulation/FrameParallelExecutor.h"
#include "IAmSpeed/World/Simulation/FrameParallelSettings.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTLS.h"
#include <cmath>
#include <atomic>
#include <exception>
#include <mutex>
#include <stdexcept>

namespace Speed::Analytic
{
namespace
{
TAutoConsoleVariable<int32> CVarPiecewiseProviderWave1075(
    TEXT("p.IAmSpeed.Parallel.PiecewiseProviderWave1075"),0,
    TEXT("Experimental whole Piecewise-provider waves, joined before the original ordered fold. Exact current bound/query/context/FP required; default off."));
TAutoConsoleVariable<int32> CVarPiecewiseProviderObserve1075(
    TEXT("p.IAmSpeed.Parallel.PiecewiseProviderObserve1075"),0,
    TEXT("Observe bounded provider-wave counters and elapsed cycles. Default off; keep off for final performance."));
TAutoConsoleVariable<int32> CVarPiecewiseProviderMaxSamples1075(
    TEXT("p.IAmSpeed.Parallel.PiecewiseProviderMaxSamples1075"),4096,
    TEXT("Bounded timing samples per joined reset, clamped 0..65536. All observed counts remain exact; skipped timing samples are explicit."));
std::atomic<uint64> TimingAdmissions{0};
std::mutex GProviderWaveStatsMutex1075;
FPiecewiseProviderWaveStats1075 GProviderWaveTotals1075;
template<typename T> bool SameBitsProviderWave1075(const T& A,const T& B)
{return FMemory::Memcmp(&A,&B,sizeof(T))==0;}
bool SameVectorProviderWave1075(const FVector3d& A,const FVector3d& B)
{return SameBitsProviderWave1075(A.X,B.X)&&SameBitsProviderWave1075(A.Y,B.Y)&&SameBitsProviderWave1075(A.Z,B.Z);}
bool SameQueryProviderWave1075(const FWorldQuery& A,const FWorldQuery& B)
{
    return FWorldQueryService::QueriesHaveSameCacheKey1075(A,B) &&
        SameVectorProviderWave1075(A.Start,B.Start)&&SameVectorProviderWave1075(A.End,B.End)&&
        SameBitsProviderWave1075(A.Rotation.X,B.Rotation.X)&&SameBitsProviderWave1075(A.Rotation.Y,B.Rotation.Y)&&
        SameBitsProviderWave1075(A.Rotation.Z,B.Rotation.Z)&&SameBitsProviderWave1075(A.Rotation.W,B.Rotation.W)&&
        SameBitsProviderWave1075(A.Radius,B.Radius)&&SameVectorProviderWave1075(A.HalfExtent,B.HalfExtent)&&
        SameBitsProviderWave1075(A.DomainTolerance,B.DomainTolerance)&&SameBitsProviderWave1075(A.InitialOverlapTolerance,B.InitialOverlapTolerance)&&
        SameVectorProviderWave1075(A.ReferenceNormal,B.ReferenceNormal)&&SameBitsProviderWave1075(A.MinimumReferenceNormalDot,B.MinimumReferenceNormalDot);
}
bool SameNumericProviderWave1075(const FFrameFloatingPointContext& A,const FFrameFloatingPointContext& B)
{
    return A.bValid&&B.bValid&&A.VectorControl==B.VectorControl&&
        FMemory::Memcmp(&A.Environment,&B.Environment,sizeof(std::fenv_t))==0;
}
void AccumulateProviderWave1075(const FPiecewiseProviderWaveStats1075& S)
{
    std::lock_guard<std::mutex> Lock(GProviderWaveStatsMutex1075);
#define SUM(Name) GProviderWaveTotals1075.Name+=S.Name
    SUM(Attempts);SUM(Waves);SUM(Candidates);SUM(Eligible);SUM(Skipped);SUM(TimedAttempts);SUM(TimingSkippedAttempts);
    SUM(Computed);SUM(Reused);SUM(Recomputed);SUM(Wasted);
    SUM(RejectBound);SUM(RejectQuery);SUM(RejectContext);SUM(RejectNumeric);
    SUM(PreparationCycles);SUM(AdmissionCycles);SUM(CallerCycles);SUM(JoinWaitCycles);
    SUM(KernelCycles);SUM(MaxLaneKernelCycles);SUM(ValidationCycles);
#undef SUM
    for(int32 I=0;I<int32(EProviderWaveFallback1075::Count);++I)GProviderWaveTotals1075.Fallbacks[I]+=S.Fallbacks[I];
    GProviderWaveTotals1075.LogicalMax=FMath::Max(GProviderWaveTotals1075.LogicalMax,S.LogicalMax);
    GProviderWaveTotals1075.NativeMax=FMath::Max(GProviderWaveTotals1075.NativeMax,S.NativeMax);
    GProviderWaveTotals1075.UsefulLaneMax=FMath::Max(GProviderWaveTotals1075.UsefulLaneMax,S.UsefulLaneMax);
    GProviderWaveTotals1075.UsefulNativeMax=FMath::Max(GProviderWaveTotals1075.UsefulNativeMax,S.UsefulNativeMax);
}
}
bool IsPiecewiseProviderWave1075Enabled(){return CVarPiecewiseProviderWave1075.GetValueOnAnyThread()!=0;}
void ResetPiecewiseProviderWaveStats1075(){std::lock_guard<std::mutex> Lock(GProviderWaveStatsMutex1075);GProviderWaveTotals1075={};TimingAdmissions.store(0,std::memory_order_relaxed);}
FPiecewiseProviderWaveStats1075 SnapshotPiecewiseProviderWaveStats1075()
{std::lock_guard<std::mutex> Lock(GProviderWaveStatsMutex1075);return GProviderWaveTotals1075;}
void LogPiecewiseProviderWaveStats1075(const TCHAR* Context)
{
    const auto S=SnapshotPiecewiseProviderWaveStats1075();
    UE_LOG(LogTemp,Display,TEXT("[IAmSpeedPiecewiseProviderWave1075] Context=%s Attempts=%llu Waves=%llu TimedAttempts=%llu TimingSkippedAttempts=%llu Candidates=%llu Eligible=%llu Skipped=%llu Computed=%llu Reused=%llu Recomputed=%llu Wasted=%llu RejectBound=%llu RejectQuery=%llu RejectContext=%llu RejectNumeric=%llu LogicalMax=%d NativeMax=%d UsefulLaneMax=%d UsefulNativeMax=%d PreparationCycles=%llu AdmissionCycles=%llu CallerCycles=%llu JoinWaitCycles=%llu KernelCycles=%llu MaxLaneKernelCycles=%llu ValidationCycles=%llu SecondsPerCycle=%.17g FallbackSerial=%llu FallbackFew=%llu FallbackNested=%llu FallbackDiagnostics=%llu FallbackCapacity=%llu FallbackBound=%llu FallbackNumeric=%llu FallbackExecutor=%llu Scope=WholeProvidersConsumptionOriginalOrder NestedTimingsNotAdditive=1 Clock=ElapsedFPlatformTimeCycles64"),
        Context?Context:TEXT(""),S.Attempts,S.Waves,S.TimedAttempts,S.TimingSkippedAttempts,S.Candidates,S.Eligible,S.Skipped,S.Computed,S.Reused,S.Recomputed,S.Wasted,
        S.RejectBound,S.RejectQuery,S.RejectContext,S.RejectNumeric,S.LogicalMax,S.NativeMax,S.UsefulLaneMax,S.UsefulNativeMax,
        S.PreparationCycles,S.AdmissionCycles,S.CallerCycles,S.JoinWaitCycles,S.KernelCycles,S.MaxLaneKernelCycles,S.ValidationCycles,FPlatformTime::GetSecondsPerCycle64(),
        S.Fallbacks[1],S.Fallbacks[2],S.Fallbacks[3],S.Fallbacks[4],S.Fallbacks[5],S.Fallbacks[6],S.Fallbacks[7],S.Fallbacks[8]);
}
FJoinedPiecewiseProviderWave1075::FJoinedPiecewiseProviderWave1075(
    const FWorldQuery& Query,const FBox3d& Bounds,const FPiecewiseProviderWaveIdentity1075& Identity,
    TConstArrayView<int32> ProviderIndices,int32 TotalCandidates,int32 Lanes,
    TFunctionRef<double()> CaptureBound,TFunctionRef<FWorldHit(int32,double)> Evaluate)
{
    if(!IsPiecewiseProviderWave1075Enabled())return;
    bObserve=CVarPiecewiseProviderObserve1075.GetValueOnAnyThread()!=0;
    Stats.Attempts=1;Stats.Candidates=TotalCandidates;Stats.Eligible=ProviderIndices.Num();
    Stats.Skipped=FMath::Max(0,TotalCandidates-ProviderIndices.Num());
    const auto Refuse=[&](EProviderWaveFallback1075 Why){++Stats.Fallbacks[int32(Why)];};
    if(Lanes<2){Refuse(EProviderWaveFallback1075::SerialPolicy);return;}
    if(IsInsideFrameParallelTask()){Refuse(EProviderWaveFallback1075::Nested);return;}
    if(!CanSpeculateIndependentAnalyticGeometry1075()){Refuse(EProviderWaveFallback1075::Diagnostics);return;}
    if(ProviderIndices.Num()>MaximumProviders||TotalCandidates>MaximumProviders){Refuse(EProviderWaveFallback1075::Capacity);return;}
    if(ProviderIndices.Num()<2){Refuse(EProviderWaveFallback1075::TooFew);return;}
    if(!Identity.World||!Identity.Index||!Identity.Invocation){Refuse(EProviderWaveFallback1075::Numeric);return;}
    if(bObserve)
    {
        const uint64 Slot=TimingAdmissions.fetch_add(1,std::memory_order_relaxed);
        bTime=Slot<uint64(FMath::Clamp(CVarPiecewiseProviderMaxSamples1075.GetValueOnAnyThread(),0,65536));
        if(bTime)Stats.TimedAttempts=1;else Stats.TimingSkippedAttempts=1;
    }
    const uint64 PreparationStart=bTime?FPlatformTime::Cycles64():0;
    const auto PreviousNumeric=CaptureFrameFloatingPointContext();
    if(!PreviousNumeric.bValid){Refuse(EProviderWaveFallback1075::Numeric);return;}
    FScopedFrameFloatingPointContext OwnerNumeric(PreviousNumeric);
    if(!OwnerNumeric.IsApplied()){Refuse(EProviderWaveFallback1075::Numeric);
        if(!OwnerNumeric.Restore())throw std::runtime_error("IAmSpeed provider owner numeric restoration failed");return;}
    // Bound evaluation is inside the restored owner guard. It is repeated by
    // the original owner loop at its original point before each kernel call.
    MaximumSearchTime=CaptureBound();
    if(!std::isfinite(MaximumSearchTime)||MaximumSearchTime<0||MaximumSearchTime>1)
    {Refuse(EProviderWaveFallback1075::InvalidBound);
        if(!OwnerNumeric.Restore())throw std::runtime_error("IAmSpeed provider owner numeric restoration failed");return;}
    NumericContext=CaptureFrameFloatingPointContext();
    if(!NumericContext.bValid){Refuse(EProviderWaveFallback1075::Numeric);
        if(!OwnerNumeric.Restore())throw std::runtime_error("IAmSpeed provider owner numeric restoration failed");return;}
    CapturedQuery=Query;CapturedBounds=Bounds;CapturedIdentity=Identity;
    Records.SetNum(ProviderIndices.Num());
    for(int32 I=0;I<ProviderIndices.Num();++I)Records[I].Index=ProviderIndices[I];
    const int32 UsedLanes=FMath::Clamp(Lanes,2,FMath::Min(12,ProviderIndices.Num()));
    uint32 ThreadIds[12]={};uint64 NumericCycles[12]={};int32 Completed[12]={};
    std::exception_ptr Failures[12];
    FFrameDispatchObservation1075 Dispatch;
    const auto Work=[&](int32 Lane)
    {
        FScopedFrameParallelTask Task(NumericContext);
        try
        {
            if(!Task.IsNumericContextApplied())throw std::runtime_error("IAmSpeed provider wave numeric context unavailable");
            if(bObserve)ThreadIds[Lane]=FPlatformTLS::GetCurrentThreadId();
            const uint64 Start=bTime?FPlatformTime::Cycles64():0;
            for(int32 I=Lane;I<Records.Num();I+=UsedLanes)
            {
                // Every provider starts with the exact captured environment,
                // replacing flags left by a previous provider on the same lane.
                if(!ApplyFrameFloatingPointContext(NumericContext))
                    throw std::runtime_error("IAmSpeed provider entry numeric context unavailable");
                try {Records[I].Hit=Evaluate(Records[I].Index,MaximumSearchTime);}
                catch(...){Records[I].Failure=std::current_exception();} // Deferred until canonical consumption.
                Records[I].NumericAfter=CaptureFrameFloatingPointContext();
                if(!Records[I].NumericAfter.bValid)
                    throw std::runtime_error("IAmSpeed provider output numeric context unavailable");
                ++Completed[Lane];
            }
            if(bTime)NumericCycles[Lane]=FPlatformTime::Cycles64()-Start;
        }
        catch(...){Failures[Lane]=std::current_exception();}
        if(!Task.RestoreNumericContext())Failures[Lane]=std::make_exception_ptr(std::runtime_error("IAmSpeed provider wave numeric restoration failed"));
    };
    if(bTime)Stats.PreparationCycles=FPlatformTime::Cycles64()-PreparationStart;
    const bool Admitted=TryDispatchPrivateFrameParallelLanes1075(UsedLanes,Work,bTime?&Dispatch:nullptr);
    if(!Admitted){Refuse(EProviderWaveFallback1075::ExecutorAdmission);
        if(!OwnerNumeric.Restore())throw std::runtime_error("IAmSpeed provider owner numeric restoration failed");return;}
    // No result is consumable until every lane restored and joined.
    std::exception_ptr FirstFailure;
    for(int32 Lane=0;Lane<UsedLanes;++Lane)
    {
        if(Failures[Lane]&&!FirstFailure)FirstFailure=Failures[Lane];
        Stats.Computed+=Completed[Lane];
        if(bObserve)
        {
            Stats.KernelCycles+=NumericCycles[Lane];Stats.MaxLaneKernelCycles=FMath::Max(Stats.MaxLaneKernelCycles,NumericCycles[Lane]);
            if(Completed[Lane]>0)++Stats.UsefulLaneMax;
            bool Seen=false;for(int32 Earlier=0;Earlier<Lane;++Earlier)Seen|=ThreadIds[Earlier]==ThreadIds[Lane];
            if(!Seen&&ThreadIds[Lane]!=0)++Stats.NativeMax;
            bool SeenUseful=false;
            for(int32 Earlier=0;Earlier<Lane;++Earlier)
                SeenUseful|=Completed[Earlier]>0&&ThreadIds[Earlier]==ThreadIds[Lane];
            if(Completed[Lane]>0&&ThreadIds[Lane]!=0&&!SeenUseful)++Stats.UsefulNativeMax;
        }
    }
    if(!OwnerNumeric.Restore())throw std::runtime_error("IAmSpeed provider owner numeric restoration failed");
    if(FirstFailure)std::rethrow_exception(FirstFailure); // Numeric infrastructure takes precedence.
    if(Stats.Computed!=uint64(Records.Num()))throw std::runtime_error("IAmSpeed provider wave incomplete joined results");
    Stats.Waves=1;Stats.LogicalMax=UsedLanes;
    if(bTime){Stats.AdmissionCycles=Dispatch.AdmissionCycles;Stats.CallerCycles=Dispatch.CallerExecutionCycles;Stats.JoinWaitCycles=Dispatch.JoinWaitCycles;}
    bReady=true;
}
FJoinedPiecewiseProviderWave1075::~FJoinedPiecewiseProviderWave1075()
{
    if(!bObserve)return;
    Stats.Wasted=Stats.Computed-Stats.Reused;
    AccumulateProviderWave1075(Stats);
}
bool FJoinedPiecewiseProviderWave1075::TryConsume(int32 ProviderIndex,const FWorldQuery& Query,
    const FBox3d& Bounds,const FPiecewiseProviderWaveIdentity1075& Identity,double Bound,FWorldHit& OutHit)
{
    if(!bReady)return false;
    for(FRecord& R:Records)
    {
        if(R.Index!=ProviderIndex||R.Used)continue;
        R.Used=true; // A rejected result is never retried at a later physical boundary.
        const uint64 Start=bTime?FPlatformTime::Cycles64():0;
        const auto CurrentNumeric=CaptureFrameFloatingPointContext();
        bool Match=true;
        if(Identity.World!=CapturedIdentity.World||Identity.Index!=CapturedIdentity.Index||
            Identity.Invocation!=CapturedIdentity.Invocation||Identity.Selection!=CapturedIdentity.Selection)
        {++Stats.RejectContext;Match=false;}
        else if(!SameBitsProviderWave1075(Bound,MaximumSearchTime)){++Stats.RejectBound;Match=false;}
        else if(!SameQueryProviderWave1075(Query,CapturedQuery)||!SameVectorProviderWave1075(Bounds.Min,CapturedBounds.Min)||
            !SameVectorProviderWave1075(Bounds.Max,CapturedBounds.Max)||Bounds.IsValid!=CapturedBounds.IsValid)
        {++Stats.RejectQuery;Match=false;}
        else if(!SameNumericProviderWave1075(CurrentNumeric,NumericContext)){++Stats.RejectNumeric;Match=false;}
        else if(!CanSpeculateIndependentAnalyticGeometry1075()){++Stats.RejectContext;Match=false;}
        if(bTime)Stats.ValidationCycles+=FPlatformTime::Cycles64()-Start;
        if(!Match){++Stats.Recomputed;return false;}
        // The serial kernel would have left this numeric status/control state.
        // Speculative worker state is replayed only for a validated consumption.
        if(!ApplyFrameFloatingPointContext(R.NumericAfter))
            throw std::runtime_error("IAmSpeed provider output numeric publication failed");
        ++Stats.Reused;
        // A speculative exception is authoritative only at this original
        // provider boundary, after the exact key/bound/FP checks and output FP.
        if(R.Failure)std::rethrow_exception(R.Failure);
        OutHit=R.Hit;return true;
    }
    return false;
}
}
