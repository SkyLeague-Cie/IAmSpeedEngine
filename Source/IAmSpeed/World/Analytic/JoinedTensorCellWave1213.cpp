#include "JoinedTensorCellWave1213.h"
#include "CompleteCellChunks1353.h"
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
TAutoConsoleVariable<int32> CVarTensorCellWave1213(
    TEXT("p.IAmSpeed.Parallel.TensorCellWave1213"),0,
    TEXT("Experimental Piecewise CELL waves. Each cell retains its complete Tensor approximation. Joined before the original provider-cell fold; exact bound/query/context/FP. Default off."));
TAutoConsoleVariable<int32> CVarTensorCellWindowItems1534(
    TEXT("p.IAmSpeed.Parallel.TensorCellWindowItems1534"),0,
    TEXT("Opt-in complete-cell window cap. 0 retains the legacy one-shot wave; positive values clamp to 2..4096. Canonical TOI changes retire the wave and require this many stable serial cells before a rebase. Existing exact key/FP validation and serial fallback remain. No measured gain adopted."));
TAutoConsoleVariable<int32> CVarTensorCellObserve1213(
    TEXT("p.IAmSpeed.Parallel.TensorCellObserve1213"),0,
    TEXT("Observe bounded Tensor-cell wave counts and elapsed cycles. Default off; nested elapsed scopes are not additive."));
TAutoConsoleVariable<int32> CVarTensorCellMaxSamples1213(
    TEXT("p.IAmSpeed.Parallel.TensorCellMaxSamples1213"),4096,
    TEXT("Bounded timing samples per joined reset, clamped 0..65536. All observed counts remain exact; skipped timing samples are explicit."));
TAutoConsoleVariable<int32> CVarTensorCellMinimum1213(
    TEXT("p.IAmSpeed.Parallel.TensorCellMinItems1213"),32,
    TEXT("Minimum SAT-admitted Piecewise cells before an inner wave, clamped 2..4096. Default32 is experimental, not a measured optimum."));
TAutoConsoleVariable<int32> CVarTensorCellGrain1213(
    TEXT("p.IAmSpeed.Parallel.TensorCellMinItemsPerLane1213"),8,
    TEXT("Minimum complete Tensor cells per lane, clamped 1..4096; lane count is reduced rather than launching empty work."));
TAutoConsoleVariable<int32> CVarTensorCellChunk1213(
    TEXT("p.IAmSpeed.Parallel.TensorCellChunkItems1213"),4,
    TEXT("Contiguous complete-cell chunk size; chunks are assigned cyclically to the fixed private lanes. 1..4096."));
TAutoConsoleVariable<int32> CVarTensorCellDynamic1353(
    TEXT("p.IAmSpeed.Parallel.TensorCellDynamic1353"),0,
    TEXT("Experimental bounded complete-cell queue after one reserved chunk per lane. Original canonical fold and numeric contexts retained. Default off; no measured gain adopted."));
std::atomic<uint64> TensorCellTimingAdmissions1213{0};
std::atomic<uint64> TensorCellEligibilityTimingAdmissions1213{0};
std::mutex GTensorCellWaveStatsMutex1213;
FTensorCellWaveStats1213 GTensorCellWaveTotals1213;
template<typename T> bool SameBitsTensorCellWave1213(const T& A,const T& B)
{return FMemory::Memcmp(&A,&B,sizeof(T))==0;}
bool SameVectorTensorCellWave1213(const FVector3d& A,const FVector3d& B)
{return SameBitsTensorCellWave1213(A.X,B.X)&&SameBitsTensorCellWave1213(A.Y,B.Y)&&SameBitsTensorCellWave1213(A.Z,B.Z);}
bool SameQueryTensorCellWave1213(const FWorldQuery& A,const FWorldQuery& B)
{
    return FWorldQueryService::QueriesHaveSameCacheKey1075(A,B) &&
        SameVectorTensorCellWave1213(A.Start,B.Start)&&SameVectorTensorCellWave1213(A.End,B.End)&&
        SameBitsTensorCellWave1213(A.Rotation.X,B.Rotation.X)&&SameBitsTensorCellWave1213(A.Rotation.Y,B.Rotation.Y)&&
        SameBitsTensorCellWave1213(A.Rotation.Z,B.Rotation.Z)&&SameBitsTensorCellWave1213(A.Rotation.W,B.Rotation.W)&&
        SameBitsTensorCellWave1213(A.Radius,B.Radius)&&SameVectorTensorCellWave1213(A.HalfExtent,B.HalfExtent)&&
        SameBitsTensorCellWave1213(A.DomainTolerance,B.DomainTolerance)&&SameBitsTensorCellWave1213(A.InitialOverlapTolerance,B.InitialOverlapTolerance)&&
        SameVectorTensorCellWave1213(A.ReferenceNormal,B.ReferenceNormal)&&SameBitsTensorCellWave1213(A.MinimumReferenceNormalDot,B.MinimumReferenceNormalDot);
}
bool SameNumericTensorCellWave1213(const FFrameFloatingPointContext& A,const FFrameFloatingPointContext& B)
{
    return A.bValid&&B.bValid&&A.VectorControl==B.VectorControl&&
        FMemory::Memcmp(&A.Environment,&B.Environment,sizeof(std::fenv_t))==0;
}
void AccumulateTensorCellWave1213(const FTensorCellWaveStats1213& S)
{
    std::lock_guard<std::mutex> Lock(GTensorCellWaveStatsMutex1213);
#define SUM(Name) GTensorCellWaveTotals1213.Name+=S.Name
    SUM(Attempts);SUM(Waves);SUM(Candidates);SUM(Eligible);SUM(Skipped);SUM(TimedAttempts);SUM(TimingSkippedAttempts);
    SUM(Computed);SUM(Reused);SUM(Recomputed);SUM(Wasted);
    SUM(RejectBound);SUM(RejectQuery);SUM(RejectContext);SUM(RejectNumeric);
    SUM(WindowAttempts1534);SUM(WindowWaves1534);SUM(WindowAdvances1534);SUM(WindowBoundRebases1534);
    SUM(PreparationCycles);SUM(AdmissionCycles);SUM(CallerCycles);SUM(JoinWaitCycles);
    SUM(PreparationMeasuredAttempts1213);SUM(DispatchAttempts1213);SUM(DispatchMeasuredAttempts1213);
    SUM(KernelCycles);SUM(MaxLaneKernelCycles);SUM(ValidationCycles);
    SUM(PublicationCycles1442);SUM(PublicationCalls1442);
    SUM(PublicationTimedCalls1442);SUM(PublicationFailedCalls1442);
    SUM(ComputedKernelCycles1213);SUM(ReusedKernelCycles1213);SUM(WastedKernelCycles1213);
#undef SUM
    for(int32 L=2;L<=12;++L)
    {
        GTensorCellWaveTotals1213.BalanceWaves1241[L]+=S.BalanceWaves1241[L];
        GTensorCellWaveTotals1213.BalanceLaneSum1241[L]+=S.BalanceLaneSum1241[L];
        GTensorCellWaveTotals1213.BalanceLaneMin1241[L]+=S.BalanceLaneMin1241[L];
        GTensorCellWaveTotals1213.BalanceLaneMax1241[L]+=S.BalanceLaneMax1241[L];
        GTensorCellWaveTotals1213.BalanceCellsMin1241[L]+=S.BalanceCellsMin1241[L];
        GTensorCellWaveTotals1213.BalanceCellsMax1241[L]+=S.BalanceCellsMax1241[L];
    }
    for(int32 I=0;I<int32(ETensorCellWaveFallback1213::Count);++I)GTensorCellWaveTotals1213.Fallbacks[I]+=S.Fallbacks[I];
    GTensorCellWaveTotals1213.WindowCandidatesMax1534=FMath::Max(GTensorCellWaveTotals1213.WindowCandidatesMax1534,S.WindowCandidatesMax1534);
    GTensorCellWaveTotals1213.RequestedMax=FMath::Max(GTensorCellWaveTotals1213.RequestedMax,S.RequestedMax);
    GTensorCellWaveTotals1213.ReusedLaneMax1213=FMath::Max(GTensorCellWaveTotals1213.ReusedLaneMax1213,S.ReusedLaneMax1213);
    GTensorCellWaveTotals1213.ReusedNativeMax1213=FMath::Max(GTensorCellWaveTotals1213.ReusedNativeMax1213,S.ReusedNativeMax1213);
    GTensorCellWaveTotals1213.LogicalMax=FMath::Max(GTensorCellWaveTotals1213.LogicalMax,S.LogicalMax);
    GTensorCellWaveTotals1213.NativeMax=FMath::Max(GTensorCellWaveTotals1213.NativeMax,S.NativeMax);
    GTensorCellWaveTotals1213.UsefulLaneMax=FMath::Max(GTensorCellWaveTotals1213.UsefulLaneMax,S.UsefulLaneMax);
    GTensorCellWaveTotals1213.UsefulNativeMax=FMath::Max(GTensorCellWaveTotals1213.UsefulNativeMax,S.UsefulNativeMax);
}
}
bool IsTensorCellWave1213Enabled(){return CVarTensorCellWave1213.GetValueOnAnyThread()!=0;}
int32 FJoinedTensorCellWave1213::WindowCandidateItems1534()
{
    const int32 Requested1534=CVarTensorCellWindowItems1534.GetValueOnAnyThread();
    return Requested1534<=0?0:FMath::Clamp(Requested1534,2,MaximumProviders);
}
int32 FJoinedTensorCellWave1213::MinimumCandidateItems()
{return FMath::Clamp(CVarTensorCellMinimum1213.GetValueOnAnyThread(),2,MaximumProviders);}
int32 FJoinedTensorCellWave1213::ResolveLanes(int32 Candidates,int32 RequestedLanes)
{
    const int32 Minimum=MinimumCandidateItems();
    const int32 Grain=FMath::Clamp(CVarTensorCellGrain1213.GetValueOnAnyThread(),1,MaximumProviders);
    if(Candidates<Minimum || Candidates>MaximumProviders || RequestedLanes<2)return 1;
    const int32 Chunk=FMath::Clamp(CVarTensorCellChunk1213.GetValueOnAnyThread(),1,MaximumProviders);
    const int32 Chunks=FMath::DivideAndRoundUp(Candidates,Chunk);
    for(int32 Lanes=FMath::Min(FMath::Min(Chunks,Candidates/Grain),FMath::Min(12,RequestedLanes));Lanes>=2;--Lanes)
    {
        int32 Smallest=MAX_int32;
        for(int32 Lane=0;Lane<Lanes;++Lane)
        {
            const int32 LaneChunks=Lane<Chunks?1+(Chunks-1-Lane)/Lanes:0;
            int32 Items=LaneChunks*Chunk;
            if(LaneChunks>0 && (Chunks-1)%Lanes==Lane)
                Items-=Chunk-(Candidates-(Chunks-1)*Chunk);
            Smallest=FMath::Min(Smallest,Items);
        }
        if(Smallest>=Grain)return Lanes;
    }
    return 1;
}
void ResetTensorCellWaveStats1213(){std::lock_guard<std::mutex> Lock(GTensorCellWaveStatsMutex1213);GTensorCellWaveTotals1213={};TensorCellTimingAdmissions1213.store(0,std::memory_order_relaxed);TensorCellEligibilityTimingAdmissions1213.store(0,std::memory_order_relaxed);}
FTensorCellWaveStats1213 SnapshotTensorCellWaveStats1213()
{std::lock_guard<std::mutex> Lock(GTensorCellWaveStatsMutex1213);return GTensorCellWaveTotals1213;}
FScopedTensorCellEligibilityObservation1213::FScopedTensorCellEligibilityObservation1213()
{
    bObserve1213=CVarTensorCellObserve1213.GetValueOnAnyThread()!=0;
    if(!bObserve1213)return;
    UncaughtAtEntry1213=std::uncaught_exceptions();
    const uint64 Slot1213=TensorCellEligibilityTimingAdmissions1213.fetch_add(1,std::memory_order_relaxed);
    bTime1213=Slot1213<uint64(FMath::Clamp(CVarTensorCellMaxSamples1213.GetValueOnAnyThread(),0,65536));
    if(bTime1213)Start1213=FPlatformTime::Cycles64();
}
FScopedTensorCellEligibilityObservation1213::~FScopedTensorCellEligibilityObservation1213()
{
    if(!bObserve1213)return;
    const uint64 Cycles1213=bTime1213?FPlatformTime::Cycles64()-Start1213:0;
    const bool Failed1213=std::uncaught_exceptions()>UncaughtAtEntry1213;
    std::lock_guard<std::mutex> Lock(GTensorCellWaveStatsMutex1213);
    ++GTensorCellWaveTotals1213.OwnerEligibilitySeen1213;
    if(bTime1213){++GTensorCellWaveTotals1213.OwnerEligibilityTimed1213;GTensorCellWaveTotals1213.OwnerEligibilityCycles1213+=Cycles1213;}
    else ++GTensorCellWaveTotals1213.OwnerEligibilitySkipped1213;
    if(Failed1213)++GTensorCellWaveTotals1213.OwnerEligibilityFailed1213;
}

void LogTensorCellWaveStats1213(const TCHAR* Context)
{
    const auto S=SnapshotTensorCellWaveStats1213();
    UE_LOG(LogTemp,Display,TEXT("[IAmSpeedTensorCellWindow1534] Context=%s ConfiguredItems=%d Attempts=%llu Waves=%llu Advances=%llu BoundRebases=%llu CandidatesMax=%d Refused=%llu Scope=BoundedCompleteCellSuffixOnly Admission=CountsAndGeometry BoundCheckCost=UNKNOWN SettlingCost=ParentWorldOnly DefaultOff=1 GainAdopted=0"),
        Context?Context:TEXT(""),FJoinedTensorCellWave1213::WindowCandidateItems1534(),S.WindowAttempts1534,S.WindowWaves1534,S.WindowAdvances1534,S.WindowBoundRebases1534,S.WindowCandidatesMax1534,S.WindowAttempts1534-S.WindowWaves1534);
    UE_LOG(LogTemp,Display,TEXT("[IAmSpeedTensorCellWave1213] Context=%s Attempts=%llu Waves=%llu TimedAttempts=%llu TimingSkippedAttempts=%llu Candidates=%llu Eligible=%llu Skipped=%llu Computed=%llu Reused=%llu Recomputed=%llu Wasted=%llu RejectBound=%llu RejectQuery=%llu RejectContext=%llu RejectNumeric=%llu LogicalMax=%d NativeMax=%d UsefulLaneMax=%d UsefulNativeMax=%d PreparationCycles=%llu AdmissionCycles=%llu CallerCycles=%llu JoinWaitCycles=%llu KernelCycles=%llu MaxLaneKernelCycles=%llu ValidationCycles=%llu SecondsPerCycle=%.17g FallbackSerial=%llu FallbackFew=%llu FallbackNested=%llu FallbackDiagnostics=%llu FallbackCapacity=%llu FallbackBound=%llu FallbackNumeric=%llu FallbackExecutor=%llu Scope=WholeTensorCellsConsumptionOriginalOrder NestedTimingsNotAdditive=1 Clock=ElapsedFPlatformTimeCycles64"),
        Context?Context:TEXT(""),S.Attempts,S.Waves,S.TimedAttempts,S.TimingSkippedAttempts,S.Candidates,S.Eligible,S.Skipped,S.Computed,S.Reused,S.Recomputed,S.Wasted,
        S.RejectBound,S.RejectQuery,S.RejectContext,S.RejectNumeric,S.LogicalMax,S.NativeMax,S.UsefulLaneMax,S.UsefulNativeMax,
        S.PreparationCycles,S.AdmissionCycles,S.CallerCycles,S.JoinWaitCycles,S.KernelCycles,S.MaxLaneKernelCycles,S.ValidationCycles,FPlatformTime::GetSecondsPerCycle64(),
        S.Fallbacks[1],S.Fallbacks[2],S.Fallbacks[3],S.Fallbacks[4],S.Fallbacks[5],S.Fallbacks[6],S.Fallbacks[7],S.Fallbacks[8]);
    UE_LOG(LogTemp,Display,TEXT("[IAmSpeedTensorCellConsumption1213] Context=%s RequestedMax=%d AdmittedMax=%d CompletedNativeMax=%d ReusedLaneMax=%d ReusedNativeMax=%d ComputedKernelCycles=%llu ReusedKernelCycles=%llu WastedKernelCycles=%llu TimedAttempts=%llu TimingSkippedAttempts=%llu SecondsPerCycle=%.17g Scope=TimedCompleteTensorCellEvaluateElapsed NestedTimingsNotAdditive=1 UntimedCostsUnknown=1"),
        Context?Context:TEXT(""),S.RequestedMax,S.LogicalMax,S.UsefulNativeMax,S.ReusedLaneMax1213,S.ReusedNativeMax1213,
        S.ComputedKernelCycles1213,S.ReusedKernelCycles1213,S.WastedKernelCycles1213,S.TimedAttempts,S.TimingSkippedAttempts,FPlatformTime::GetSecondsPerCycle64());
    UE_LOG(LogTemp,Display,TEXT("[IAmSpeedTensorCellPublication1442] Context=%s Calls=%llu TimedCalls=%llu FailedCalls=%llu Cycles=%llu SecondsPerCycle=%.17g Scope=OwnerOutputFPApplyOnly AfterValidation=1 NestedTimingsNotAdditive=1 UntimedCostsUnknown=1"),
        Context?Context:TEXT(""),S.PublicationCalls1442,S.PublicationTimedCalls1442,S.PublicationFailedCalls1442,S.PublicationCycles1442,FPlatformTime::GetSecondsPerCycle64());
    const uint64 Classified1213=FMath::Min(S.Attempts,S.TimedAttempts+S.TimingSkippedAttempts);
    const uint64 Unclassified1213=S.Attempts-Classified1213;
    const uint64 PreparationUnknown1213=S.Attempts-FMath::Min(S.Attempts,S.PreparationMeasuredAttempts1213);
    const uint64 DispatchUnknown1213=S.DispatchAttempts1213-FMath::Min(S.DispatchAttempts1213,S.DispatchMeasuredAttempts1213);
    UE_LOG(LogTemp,Display,TEXT("[IAmSpeedTensorCellCoverage1213] Context=%s ObserverEnabled=%d OwnerEligibilitySeen=%llu OwnerEligibilityTimed=%llu OwnerEligibilitySkipped=%llu OwnerEligibilityFailed=%llu OwnerEligibilityCycles=%llu OwnerEligibilityCostKnown=%d ConstructorAttempts=%llu ConstructorTimingUnclassifiedAttempts=%llu ConstructorPreparationMeasuredAttempts=%llu ConstructorPreparationUnknownAttempts=%llu DispatchAttempts=%llu DispatchMeasuredAttempts=%llu DispatchUnknownAttempts=%llu SecondsPerCycle=%.17g EligibilityScope=OwnerPrivateBVHSATProbeIncludingScratchAndFPGuard EligibilityExcludesWaveConstructor=1 ConstructorPreparationScope=AfterPolicyRefusalsBeforeDispatch ConstructorPolicyAndRefusalCosts=UNKNOWN UnmeasuredCosts=UNKNOWN FailedScopeCosts=PARTIAL ParentWorldIncludesAllScopes=1 NestedTimingsNotAdditive=1"),
        Context?Context:TEXT(""),CVarTensorCellObserve1213.GetValueOnAnyThread()!=0,
        S.OwnerEligibilitySeen1213,S.OwnerEligibilityTimed1213,S.OwnerEligibilitySkipped1213,S.OwnerEligibilityFailed1213,S.OwnerEligibilityCycles1213,
        S.OwnerEligibilitySeen1213>0&&S.OwnerEligibilitySkipped1213==0&&S.OwnerEligibilityFailed1213==0,
        S.Attempts,Unclassified1213,S.PreparationMeasuredAttempts1213,PreparationUnknown1213,S.DispatchAttempts1213,S.DispatchMeasuredAttempts1213,DispatchUnknown1213,FPlatformTime::GetSecondsPerCycle64());
    for(int32 L=2;L<=12;++L)
    {
        UE_LOG(LogTemp,Display,TEXT("[IAmSpeedTensorCellBalance1241] Context=%s Lanes=%d TimedWaves=%llu LaneElapsedSum=%llu LaneMinimumSum=%llu LaneMaximumSum=%llu CompletedCellsMinimumSum=%llu CompletedCellsMaximumSum=%llu SecondsPerCycle=%.17g Scope=PerWaveExistingLaneElapsedGroupedByActualLaneCount Clock=ElapsedFPlatformTimeCycles64 NestedTimingsNotAdditive=1 UntimedWavesUnknown=1"),
            Context?Context:TEXT(""),L,S.BalanceWaves1241[L],S.BalanceLaneSum1241[L],S.BalanceLaneMin1241[L],S.BalanceLaneMax1241[L],S.BalanceCellsMin1241[L],S.BalanceCellsMax1241[L],FPlatformTime::GetSecondsPerCycle64());
    }
}
FJoinedTensorCellWave1213::FJoinedTensorCellWave1213(
    const FWorldQuery& Query,const FBox3d& Bounds,const FTensorCellWaveIdentity1213& Identity,
    TConstArrayView<int32> ProviderIndices,int32 TotalCandidates,int32 Lanes,
    TFunctionRef<double()> CaptureBound,TFunctionRef<FWorldHit(int32,double)> Evaluate,
    ETensorCellWindowReason1534 WindowReason1534)
{
    if(!IsTensorCellWave1213Enabled())return;
    bObserve=CVarTensorCellObserve1213.GetValueOnAnyThread()!=0;
    Stats.Attempts=1;Stats.RequestedMax=Lanes;Stats.Candidates=TotalCandidates;Stats.Eligible=ProviderIndices.Num();
    Stats.Skipped=FMath::Max(0,TotalCandidates-ProviderIndices.Num());
    if(WindowReason1534!=ETensorCellWindowReason1534::Legacy)
    {
        Stats.WindowAttempts1534=1;Stats.WindowCandidatesMax1534=ProviderIndices.Num();
        Stats.WindowAdvances1534=WindowReason1534==ETensorCellWindowReason1534::Advance;
        Stats.WindowBoundRebases1534=WindowReason1534==ETensorCellWindowReason1534::BoundChanged;
    }
    const auto Refuse=[&](ETensorCellWaveFallback1213 Why){++Stats.Fallbacks[int32(Why)];};
    if(Lanes<2){Refuse(ETensorCellWaveFallback1213::SerialPolicy);return;}
    Lanes=ResolveLanes(ProviderIndices.Num(),Lanes);
    if(Lanes<2){Refuse(ETensorCellWaveFallback1213::TooFew);return;}
    if(IsInsideFrameParallelTask()){Refuse(ETensorCellWaveFallback1213::Nested);return;}
    if(!CanSpeculateIndependentAnalyticGeometry1075()){Refuse(ETensorCellWaveFallback1213::Diagnostics);return;}
    if(ProviderIndices.Num()>MaximumProviders||TotalCandidates>MaximumProviders){Refuse(ETensorCellWaveFallback1213::Capacity);return;}
    if(ProviderIndices.Num()<2){Refuse(ETensorCellWaveFallback1213::TooFew);return;}
    if(!Identity.World||!Identity.Index||!Identity.Invocation){Refuse(ETensorCellWaveFallback1213::Numeric);return;}
    if(bObserve)
    {
        const uint64 Slot=TensorCellTimingAdmissions1213.fetch_add(1,std::memory_order_relaxed);
        bTime=Slot<uint64(FMath::Clamp(CVarTensorCellMaxSamples1213.GetValueOnAnyThread(),0,65536));
        if(bTime)Stats.TimedAttempts=1;else Stats.TimingSkippedAttempts=1;
    }
    const uint64 PreparationStart=bTime?FPlatformTime::Cycles64():0;
    const auto PreviousNumeric=CaptureFrameFloatingPointContext();
    if(!PreviousNumeric.bValid){Refuse(ETensorCellWaveFallback1213::Numeric);return;}
    FScopedFrameFloatingPointContext OwnerNumeric(PreviousNumeric);
    if(!OwnerNumeric.IsApplied()){Refuse(ETensorCellWaveFallback1213::Numeric);
        if(!OwnerNumeric.Restore())throw std::runtime_error("IAmSpeed Tensor cell owner numeric restoration failed");return;}
    // Bound evaluation is inside the restored owner guard. It is repeated by
    // the original owner loop at its original point before each kernel call.
    MaximumSearchTime=CaptureBound();
    if(!std::isfinite(MaximumSearchTime)||MaximumSearchTime<0||MaximumSearchTime>1)
    {Refuse(ETensorCellWaveFallback1213::InvalidBound);
        if(!OwnerNumeric.Restore())throw std::runtime_error("IAmSpeed Tensor cell owner numeric restoration failed");return;}
    NumericContext=CaptureFrameFloatingPointContext();
    if(!NumericContext.bValid){Refuse(ETensorCellWaveFallback1213::Numeric);
        if(!OwnerNumeric.Restore())throw std::runtime_error("IAmSpeed Tensor cell owner numeric restoration failed");return;}
    CapturedQuery=Query;CapturedBounds=Bounds;CapturedIdentity=Identity;
    Records.SetNum(ProviderIndices.Num());
    for(int32 I=0;I<ProviderIndices.Num();++I)Records[I].Index=ProviderIndices[I];
    RecordLookup1213.SetNum(Records.Num());
    for(int32 I=0;I<Records.Num();++I)RecordLookup1213[I]={Records[I].Index,I};
    RecordLookup1213.Sort([](const FLookup1213& A,const FLookup1213& B){return A.Index<B.Index;});
    for(int32 I=1;I<RecordLookup1213.Num();++I)
    {
        if(RecordLookup1213[I-1].Index!=RecordLookup1213[I].Index)continue;
        Refuse(ETensorCellWaveFallback1213::Capacity);
        if(!OwnerNumeric.Restore())throw std::runtime_error("IAmSpeed Tensor cell owner numeric restoration failed");
        return; // Duplicate cell keys cannot publish an ambiguous result.
    }
    const int32 UsedLanes=FMath::Clamp(Lanes,2,FMath::Min(12,ProviderIndices.Num()));
    const int32 ChunkItems=FMath::Clamp(CVarTensorCellChunk1213.GetValueOnAnyThread(),1,MaximumProviders);
    const bool Dynamic1353=CVarTensorCellDynamic1353.GetValueOnAnyThread()==1;
    FCompleteCellChunks1353 Claims1353(static_cast<std::uint32_t>(Records.Num()),
        static_cast<std::uint32_t>(UsedLanes),static_cast<std::uint32_t>(ChunkItems));
    uint32 ThreadIds[12]={};uint64 NumericCycles[12]={};int32 Completed[12]={};
    std::exception_ptr Failures[12];
    FFrameDispatchObservation1075 Dispatch;
    const auto Work=[&](int32 Lane)
    {
        FScopedFrameParallelTask Task(NumericContext);
        try
        {
            if(!Task.IsNumericContextApplied())throw std::runtime_error("IAmSpeed Tensor cell wave numeric context unavailable");
            if(bObserve)ThreadIds[Lane]=FPlatformTLS::GetCurrentThreadId();
            const uint64 Start=bTime?FPlatformTime::Cycles64():0;
            const auto EvaluateRange1353=[&](int32 Begin1353,int32 End1353)
            {
            for(int32 I=Begin1353;I<End1353;++I)
            {
                // Every provider starts with the exact captured environment,
                // replacing flags left by a previous provider on the same lane.
                if(!ApplyFrameFloatingPointContext(NumericContext))
                    throw std::runtime_error("IAmSpeed Tensor cell entry numeric context unavailable");
                Records[I].Lane1213=Lane;
                if(bObserve)Records[I].Native1213=ThreadIds[Lane];
                const uint64 CellKernelStart1213=bTime?FPlatformTime::Cycles64():0;
                try {Records[I].Hit=Evaluate(Records[I].Index,MaximumSearchTime);}
                catch(...){Records[I].Failure=std::current_exception();}
                if(bTime)Records[I].KernelCycles1213=FPlatformTime::Cycles64()-CellKernelStart1213; // Deferred until canonical consumption.
                Records[I].NumericAfter=CaptureFrameFloatingPointContext();
                if(!Records[I].NumericAfter.bValid)
                    throw std::runtime_error("IAmSpeed Tensor cell output numeric context unavailable");
                ++Completed[Lane];
            }
            };
            if(Dynamic1353)
            {
                if(!Claims1353.IsValid())throw std::runtime_error("IAmSpeed invalid complete-cell assignment");
                FCompleteCellChunks1353::FRange Range1353;
                if(Claims1353.InitialRangeForLane(static_cast<std::uint32_t>(Lane),Range1353))
                    EvaluateRange1353(static_cast<int32>(Range1353.Begin),static_cast<int32>(Range1353.End));
                while(Claims1353.TakeNext(Range1353))
                    EvaluateRange1353(static_cast<int32>(Range1353.Begin),static_cast<int32>(Range1353.End));
            }
            else
            {
                for(int32 Chunk=Lane*ChunkItems;Chunk<Records.Num();Chunk+=UsedLanes*ChunkItems)
                    EvaluateRange1353(Chunk,FMath::Min(Chunk+ChunkItems,Records.Num()));
            }
            if(bTime)NumericCycles[Lane]=FPlatformTime::Cycles64()-Start;
        }
        catch(...){Failures[Lane]=std::current_exception();}
        if(!Task.RestoreNumericContext())Failures[Lane]=std::make_exception_ptr(std::runtime_error("IAmSpeed Tensor cell wave numeric restoration failed"));
    };
    if(bTime){Stats.PreparationCycles=FPlatformTime::Cycles64()-PreparationStart;Stats.PreparationMeasuredAttempts1213=1;}
    Stats.DispatchAttempts1213=1;
    const bool Admitted=TryDispatchPrivateFrameParallelLanes1075(UsedLanes,Work,bTime?&Dispatch:nullptr);
    if(bTime)
    {
        Stats.DispatchMeasuredAttempts1213=1;
        Stats.AdmissionCycles=Dispatch.AdmissionCycles;
        Stats.CallerCycles=Dispatch.CallerExecutionCycles;Stats.JoinWaitCycles=Dispatch.JoinWaitCycles;
    }
    if(!Admitted){Refuse(ETensorCellWaveFallback1213::ExecutorAdmission);
        if(!OwnerNumeric.Restore())throw std::runtime_error("IAmSpeed Tensor cell owner numeric restoration failed");return;}
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
    if(!OwnerNumeric.Restore())throw std::runtime_error("IAmSpeed Tensor cell owner numeric restoration failed");
    if(FirstFailure)std::rethrow_exception(FirstFailure); // Numeric infrastructure takes precedence.
    if(Stats.Computed!=uint64(Records.Num()))throw std::runtime_error("IAmSpeed Tensor cell wave incomplete joined results");
    Stats.Waves=1;Stats.LogicalMax=UsedLanes;
    Stats.WindowWaves1534=Stats.WindowAttempts1534;
    if(bTime)
    {
        // Every lane has joined and numeric contexts have been restored above.
        // These are integer reductions over the original elapsed samples only.
        uint64 Sum1241=0,Min1241=NumericCycles[0],Max1241=NumericCycles[0];
        int32 CellsMin1241=Completed[0],CellsMax1241=Completed[0];
        for(int32 L=0;L<UsedLanes;++L)
        {
            Sum1241+=NumericCycles[L];Min1241=FMath::Min(Min1241,NumericCycles[L]);Max1241=FMath::Max(Max1241,NumericCycles[L]);
            CellsMin1241=FMath::Min(CellsMin1241,Completed[L]);CellsMax1241=FMath::Max(CellsMax1241,Completed[L]);
        }
        Stats.BalanceWaves1241[UsedLanes]=1;Stats.BalanceLaneSum1241[UsedLanes]=Sum1241;
        Stats.BalanceLaneMin1241[UsedLanes]=Min1241;Stats.BalanceLaneMax1241[UsedLanes]=Max1241;
        Stats.BalanceCellsMin1241[UsedLanes]=uint64(CellsMin1241);Stats.BalanceCellsMax1241[UsedLanes]=uint64(CellsMax1241);
    }
    if(bTime){Stats.AdmissionCycles=Dispatch.AdmissionCycles;Stats.CallerCycles=Dispatch.CallerExecutionCycles;Stats.JoinWaitCycles=Dispatch.JoinWaitCycles;}
    bReady=true;
}
FJoinedTensorCellWave1213::~FJoinedTensorCellWave1213()
{
    if(!bObserve)return;
    Stats.Wasted=Stats.Computed-Stats.Reused;
    bool ReusedLanes1213[12]={};uint32 ReusedNatives1213[12]={};
    for(const FRecord& Record1213:Records)
    {
        Stats.ComputedKernelCycles1213+=Record1213.KernelCycles1213;
        if(!Record1213.Reused1213)continue;
        Stats.ReusedKernelCycles1213+=Record1213.KernelCycles1213;
        ReusedLanes1213[Record1213.Lane1213]=true;
        bool Seen1213=false;
        for(int32 I1213=0;I1213<Stats.ReusedNativeMax1213;++I1213)
            Seen1213|=ReusedNatives1213[I1213]==Record1213.Native1213;
        if(!Seen1213&&Record1213.Native1213!=0)
            ReusedNatives1213[Stats.ReusedNativeMax1213++]=Record1213.Native1213;
    }
    for(const bool Reused1213:ReusedLanes1213)Stats.ReusedLaneMax1213+=Reused1213;
    Stats.WastedKernelCycles1213=Stats.ComputedKernelCycles1213-Stats.ReusedKernelCycles1213;
    AccumulateTensorCellWave1213(Stats);
}
bool FJoinedTensorCellWave1213::HasSameBound1534(TFunctionRef<double()> CaptureBound) const
{
    if(!bReady)return false;
    const auto Entry1534=CaptureFrameFloatingPointContext();
    if(!Entry1534.bValid)return false;
    FScopedFrameFloatingPointContext BoundNumeric1534(Entry1534);
    bool Same1534=false;
    if(BoundNumeric1534.IsApplied())
        Same1534=SameBitsTensorCellWave1213(CaptureBound(),MaximumSearchTime);
    if(!BoundNumeric1534.Restore())
        throw std::runtime_error("IAmSpeed Tensor cell window bound numeric restoration failed");
    return Same1534;
}
bool FJoinedTensorCellWave1213::ContainsProvider1534(int32 ProviderIndex) const
{
    if(!bReady)return false;
    int32 Low1534=0,High1534=RecordLookup1213.Num();
    while(Low1534<High1534)
    {
        const int32 Middle1534=Low1534+(High1534-Low1534)/2;
        if(RecordLookup1213[Middle1534].Index<ProviderIndex)Low1534=Middle1534+1;
        else High1534=Middle1534;
    }
    return Low1534<RecordLookup1213.Num()&&RecordLookup1213[Low1534].Index==ProviderIndex;
}
bool FJoinedTensorCellWave1213::TryConsume(int32 ProviderIndex,const FWorldQuery& Query,
    const FBox3d& Bounds,const FTensorCellWaveIdentity1213& Identity,double Bound,FWorldHit& OutHit)
{
    if(!bReady)return false;
    int32 Low1213=0,High1213=RecordLookup1213.Num();
    while(Low1213<High1213)
    {
        const int32 Middle1213=Low1213+(High1213-Low1213)/2;
        if(RecordLookup1213[Middle1213].Index<ProviderIndex)Low1213=Middle1213+1;
        else High1213=Middle1213;
    }
    if(Low1213>=RecordLookup1213.Num()||RecordLookup1213[Low1213].Index!=ProviderIndex)return false;
    FRecord& R=Records[RecordLookup1213[Low1213].Record];
    if(R.Used)return false;
    {
        R.Used=true; // A rejected result is never retried at a later physical boundary.
        const uint64 Start=bTime?FPlatformTime::Cycles64():0;
        const auto CurrentNumeric=CaptureFrameFloatingPointContext();
        bool Match=true;
        if(Identity.World!=CapturedIdentity.World||Identity.Index!=CapturedIdentity.Index||
            Identity.Invocation!=CapturedIdentity.Invocation||Identity.Selection!=CapturedIdentity.Selection)
        {++Stats.RejectContext;Match=false;}
        else if(!SameBitsTensorCellWave1213(Bound,MaximumSearchTime)){++Stats.RejectBound;Match=false;}
        else if(!SameQueryTensorCellWave1213(Query,CapturedQuery)||!SameVectorTensorCellWave1213(Bounds.Min,CapturedBounds.Min)||
            !SameVectorTensorCellWave1213(Bounds.Max,CapturedBounds.Max)||Bounds.IsValid!=CapturedBounds.IsValid)
        {++Stats.RejectQuery;Match=false;}
        else if(!SameNumericTensorCellWave1213(CurrentNumeric,NumericContext)){++Stats.RejectNumeric;Match=false;}
        else if(!CanSpeculateIndependentAnalyticGeometry1075()){++Stats.RejectContext;Match=false;}
        if(bTime)Stats.ValidationCycles+=FPlatformTime::Cycles64()-Start;
        if(!Match){++Stats.Recomputed;return false;}
        // The serial kernel would have left this numeric status/control state.
        // Speculative worker state is replayed only for a validated consumption.
        if(bObserve)++Stats.PublicationCalls1442;
        const uint64 PublicationStart1442=bTime?FPlatformTime::Cycles64():0;
        const bool Published1442=ApplyFrameFloatingPointContext(R.NumericAfter);
        if(bTime)
        {
            Stats.PublicationCycles1442+=FPlatformTime::Cycles64()-PublicationStart1442;
            ++Stats.PublicationTimedCalls1442;
        }
        if(!Published1442)
        {
            if(bObserve)++Stats.PublicationFailedCalls1442;
            throw std::runtime_error("IAmSpeed Tensor cell output numeric publication failed");
        }
        ++Stats.Reused;R.Reused1213=true;
        // A speculative exception is authoritative only at this original
        // provider boundary, after the exact key/bound/FP checks and output FP.
        if(R.Failure)std::rethrow_exception(R.Failure);
        OutHit=R.Hit;return true;
    }
}
}
