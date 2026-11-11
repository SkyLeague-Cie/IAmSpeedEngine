#include "CanonicalFixedSphereWheelProbe.h"
#include "BoundedApproxFixedWheelProbe.h"
#include "IAmSpeed/World/Analytic/AnalyticWorldData.h"
#include "Math/VectorRegister.h"
#include "ReferenceAffineContact.h"
#include "IAmSpeed/World/Analytic/StaticWorldQueryAudit.h"
#include <atomic>
#include <exception>
#include "Misc/CoreDelegates.h"
#include "HAL/PlatformTime.h"
#include "HAL/IConsoleManager.h"
#include "PreparedNativeOwnerPoseMutation.h"
#include "PreparedCanonicalStaticQueryService.h"
#include "IAmSpeed/Base/SHitResult.h"
#include "CollisionQueryParams.h"
#include "Misc/ScopeExit.h"
#include "HAL/CriticalSection.h"
#include "Misc/ScopeLock.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "Misc/ScopedCVar.h"
#include "Components/BoxComponent.h"
#include "UObject/StrongObjectPtr.h"
#include "IAmSpeed/World/Analytic/AnalyticWorldQuery.h"
#include "IAmSpeed/World/Simulation/FrameFloatingPointContext.h"
#include <cfenv>
#endif

namespace Speed::Analytic::Certified
{
IAMSPEED_API bool ProveRegularFaceLeafWithImmutablePlanarRank(FIndexedIntervalUnionPtr,
    const FTensorSphereContactLeaf&,FDistanceBudget&,FIntervalPoint&);
}

// Private profiling accompanies the exact Contact geometry memo optimization.

// Versioned opt-in historical complete-query cache. Statistics are observations,
// never authority or a replacement for this producer's fresh live guards.


namespace CurveStationaryContact98Profile
{
TAutoConsoleVariable<int32> CVarEnabled(TEXT("p.IAmSpeed.AnalyticWorld.CurveStationaryContact98"),0,
    TEXT("Private exact 1D curve contact solver98; default OFF. No geometry/threshold change."),ECVF_Default);
void Publish()
{
    const auto& o=Speed::Analytic::Certified::CurveStationaryContact98::Observation();
    UE_LOG(LogTemp,Display,TEXT("[CurveStationaryContact98] Schema=CurveStationaryContact98/v1 Scope=IAmSpeedModuleProgramPrefixAtPreExit PerCase=0 TerminalQueryJoin=0 NoPASS=1 DefaultOff=1 Enabled=%d Attempts=%llu Proved=%llu Excluded=%llu SkippedSiblings=%llu WorkFailures=%llu"),
        int32(CVarEnabled.GetValueOnAnyThread()==1),o.Attempts.load(),o.Proved.load(),o.Excluded.load(),o.SkippedSiblings.load(),o.WorkFailures.load());
}
}

namespace DirectPlaneProposal95Profile
{
TAutoConsoleVariable<int32> CVarEnabled(TEXT("p.IAmSpeed.AnalyticWorld.DirectPlaneUVProposal95"),0,
    TEXT("Private fixed-UV proposal95; original-net verification only, default OFF."),ECVF_Default);
void Publish()
{
    namespace Q=Speed::Analytic::Certified::PlaneProposal95;
    auto& o=Q::Observation();uint64 values[Q::Count]{},again[Q::Count]{};
    const uint64 finished=o.Finished.load();bool coherent=o.Integrity.load()&&o.Active.load()==0;
    for(uint32 i=0;i<Q::Count;++i)values[i]=o.Values[i].load();
    for(uint32 i=0;i<Q::Count;++i)again[i]=o.Values[i].load();
    for(uint32 i=0;i<Q::Count;++i)coherent=coherent&&values[i]==again[i];
    coherent=coherent&&o.Active.load()==0&&o.Finished.load()==finished;
    UE_LOG(LogTemp,Display,TEXT("[DirectPlaneUVProposal95] Schema=DirectPlaneUVProposal95/v1 Scope=IAmSpeedModuleProgramPrefixAtPreExit PerCase=0 TerminalQueryJoin=0 NoPASS=1 Inclusive=1 NotAdditive=1 DefaultOff=1 Enabled=%d ObservedEnabled=%d SnapshotCoherent=%d Finished=%llu Calls=%llu Admissions=%llu SkippedInverse=%llu SkippedLocal=%llu Evaluated=%llu Improved=%llu ThresholdDecided=%llu GapClosed=%llu WorkFail=%llu NumericFail=%llu WorkUnits=%llu Ticks=%llu SecondsPerCycle=%.17g NoSavedCellsClaim=1"),
        int32(CVarEnabled.GetValueOnAnyThread()==1),int32(o.ObservedEnabled.load()),int32(coherent),finished,
        values[Q::Calls],values[Q::Admissions],values[Q::SkippedInverse],values[Q::SkippedLocal],
        values[Q::Evaluated],values[Q::Improved],values[Q::ThresholdDecided],values[Q::GapClosed],
        values[Q::WorkFail],values[Q::NumericFail],values[Q::WorkUnits],values[Q::Ticks],FPlatformTime::GetSecondsPerCycle64());
}
}

namespace HistoricalQueryCache94
{
TAutoConsoleVariable<int32> CVarCompactPartition(TEXT("p.IAmSpeed.AnalyticWorld.CandidatePartitionCompact"),0,
    TEXT("Private exact complete candidate complement and cache94 compact key; default OFF."),ECVF_Default);
bool CompactPartitionEnabled(){return CVarCompactPartition.GetValueOnAnyThread()==1;}
TAutoConsoleVariable<int32> CVarEnabled(TEXT("p.IAmSpeed.AnalyticWorld.HistoricalWholeQueryCache94"),0,
    TEXT("Private exact whole-query geometry cache94; default OFF, explicit scoped admission only."),ECVF_Default);
TAutoConsoleVariable<int32> CVarEarlyEnabled(TEXT("p.IAmSpeed.AnalyticWorld.EarlyHistoricalWholeQueryCache94"),0,
    TEXT("Private early lookup in exact cache94 after fresh inherited guards; default OFF."),ECVF_Default);
TAutoConsoleVariable<int32> CVarBeforeCoverage(TEXT("p.IAmSpeed.AnalyticWorld.HistoricalCacheBeforeCoverage"),0,
    TEXT("Default OFF. Route a previous successful wheel fallback to the unchanged exact producer before repeated coverage; no cached authority or Unknown result."),ECVF_Default);
std::atomic<uint64> CoveragePeeks{0},CoveragePreferred{0},CoverageRecorded{0};
std::atomic<uint64> EarlyLookups{0},EarlyHits{0},EarlyMisses{0},EarlyNotApplicable{0};
bool EarlyEnabled(){return CVarEarlyEnabled.GetValueOnAnyThread()==1;}
std::atomic<uint64> Lookups{0},Hits{0},Misses{0},NotApplicable{0},NotStored{0},Inserts{0},Evictions{0},SnapshotClears{0},Materialized{0},KeyWork{0},CopyWork{0},LiveBytes{0},PeakBytes{0},InsertedBytes{0},Active{0};
std::atomic<bool> Integrity{true};
void Add(std::atomic<uint64>& value,uint64 n=1){if(value.fetch_add(n)>MAX_uint64-n)Integrity.store(false);}
struct FActive{FActive(){Add(Active);}~FActive(){if(!Active.fetch_sub(1))Integrity.store(false);}};
bool Enabled(){return CVarEnabled.GetValueOnAnyThread()==1;}
void AddBytes(uint64 bytes)
{
    Add(LiveBytes,bytes);Add(InsertedBytes,bytes);const uint64 live=LiveBytes.load();uint64 peak=PeakBytes.load();
    while(live>peak&&!PeakBytes.compare_exchange_weak(peak,live)){}
}
void RemoveBytes(uint64 bytes){if(LiveBytes.fetch_sub(bytes)<bytes)Integrity.store(false);}
void Publish()
{
    UE_LOG(LogTemp,Display,TEXT("[HistoricalCacheBeforeCoverage] DefaultOFF=1 RoutingOnly=1 Enabled=%d Peeks=%llu Preferred=%llu Recorded=%llu"),
        int32(CVarBeforeCoverage.GetValueOnAnyThread()==1),CoveragePeeks.load(),CoveragePreferred.load(),CoverageRecorded.load());
    const uint64 values[]={Lookups.load(),Hits.load(),Misses.load(),NotApplicable.load(),NotStored.load(),Inserts.load(),Evictions.load(),SnapshotClears.load(),Materialized.load(),KeyWork.load(),CopyWork.load(),LiveBytes.load(),PeakBytes.load(),InsertedBytes.load()};
    const uint64 checkValues[]={Lookups.load(),Hits.load(),Misses.load(),NotApplicable.load(),NotStored.load(),Inserts.load(),Evictions.load(),SnapshotClears.load(),Materialized.load(),KeyWork.load(),CopyWork.load(),LiveBytes.load(),PeakBytes.load(),InsertedBytes.load()};
    bool coherent=Integrity.load()&&Active.load()==0;
    for(int32 i=0;i<int32(UE_ARRAY_COUNT(values));++i)coherent=coherent&&values[i]==checkValues[i];
    const TCHAR* bytesScope=EarlyEnabled()||EarlyLookups.load()!=0?
        TEXT("AccountedEntryAndRetainedCandidateAllocatedBytesUpperBoundExcludesSharedWorld"):
        TEXT("OwnedEntryPayloadArraysExcludesSharedWorld");
    UE_LOG(LogTemp,Display,TEXT("[HistoricalQueryCache94] Schema=HistoricalQueryCache94/v1 Scope=ProgramPrefixAtPreExit PerCase=0 TerminalQueryJoin=0 NoPASS=1 NotAdditive=1 DefaultOff=1 Capacity=64 Ways=4 Enabled=%d SnapshotCoherent=%d Lookups=%llu Hits=%llu Misses=%llu NotApplicable=%llu NotStored=%llu Inserts=%llu Evictions=%llu SnapshotClears=%llu Materialized=%llu KeyWork=%llu CopyWork=%llu LiveBytes=%llu PeakBytes=%llu InsertedBytes=%llu BytesScope=%s"),
        int32(Enabled()),int32(coherent),values[0],values[1],values[2],values[3],values[4],values[5],values[6],values[7],values[8],values[9],values[10],values[11],values[12],values[13],bytesScope);
    if(EarlyEnabled()||EarlyLookups.load()!=0||EarlyNotApplicable.load()!=0)
        UE_LOG(LogTemp,Display,TEXT("[EarlyHistoricalQueryCache94] Scope=ProgramPrefixAtPreExit NoPASS=1 DefaultOff=1 Enabled=%d Lookups=%llu Hits=%llu Misses=%llu NotApplicable=%llu RetainedCandidateAccounting=AllocatedObjectPathIndicesAndNetsPerEntryUpperBoundMayShare"),
            int32(EarlyEnabled()),EarlyLookups.load(),EarlyHits.load(),EarlyMisses.load(),EarlyNotApplicable.load());
}
}

// Private observation only. No field below is a query/identity/result cache,
// authority, budget, continuation token or collision decision.


// Passive per-candidate attribution. No measurement is a collision input.
namespace ParityContactCost
{
struct FRow
{
    uint64 Calls=0,Ticks=0,TimeWork=0,Cells=0,MaxTicks=0;
    FVector3d Start{},End{};double Radius=0;
};
FCriticalSection Mutex;
TMap<FString,FRow> Rows;
void Record(bool affine,const Speed::Analytic::Prepared::FReferenceTransportPtr& reference,
    const Speed::Analytic::Certified::FReferenceCandidateSetPtr& candidates,
    const Speed::Analytic::FWorldQuery& query,
    const Speed::Analytic::Certified::FTensorSphereContactEnclosure& contact,uint64 ticks)
{
    if(!reference||!candidates)return;
    FString key=affine?TEXT("Affine"):TEXT("Generic");
    for(int32 index:candidates->GetCandidateIndices())
    {
        uint32 axes=0,straight=0;
        for(const auto& fact:reference->GetExactAffineExtrusions(index))
        {
            if(fact.Product.FreeWorldAxis<0||fact.Product.FreeWorldAxis>2)continue;
            const uint32 bit=1u<<fact.Product.FreeWorldAxis;axes|=bit;
            int32 nonzero=0;
            for(int32 d=0;d<3;++d)
            {
                const auto& v=fact.Product.ExactCurveDerivativeHull.Axis[d];
                nonzero+=!v.Known||((AnalyticCertified::Bits(v.Lower)|AnalyticCertified::Bits(v.Upper))&AnalyticCertified::Magnitude)!=0;
            }
            if(nonzero==1)straight|=bit;
        }
        key+=FString::Printf(TEXT("_%d:a%u:s%u"),index,axes,straight);
    }
    FScopeLock lock(&Mutex);
    auto& row=Rows.FindOrAdd(key);++row.Calls;row.Ticks+=ticks;
    row.TimeWork+=contact.TimeWorkUsed;row.Cells+=contact.SpatialVisitedCells;
    if(ticks>row.MaxTicks){row.MaxTicks=ticks;row.Start=query.Start;row.End=query.End;row.Radius=query.Radius;}
}
void Publish()
{
    FScopeLock lock(&Mutex);TArray<FString> keys;Rows.GetKeys(keys);keys.Sort();
    for(const auto& key:keys)
    {
        const auto& r=Rows[key];
        UE_LOG(LogTemp,Display,TEXT("[ParityContactCost] Key=%s Calls=%llu Ticks=%llu TimeWork=%llu Cells=%llu MaxTicks=%llu SecondsPerCycle=%.17g Start=%.17g,%.17g,%.17g End=%.17g,%.17g,%.17g Radius=%.17g"),
            *key,r.Calls,r.Ticks,r.TimeWork,r.Cells,r.MaxTicks,FPlatformTime::GetSecondsPerCycle64(),
            r.Start.X,r.Start.Y,r.Start.Z,r.End.X,r.End.Y,r.End.Z,r.Radius);
    }
}
}

namespace StadiumSphereProfile70
{
enum ERow:uint32 { Producer,RadiusSelection,Contact,ContactConsumer,RowCount };
struct FRow {std::atomic<uint64> Calls{0},Finished{0},InFlight{0},Ticks{0},Unwinding{0},BudgetUsed{0},Candidates{0},Excluded{0},ContactAccounting{0},TimeWork{0},SpatialCells{0},MemoRequests{0},MemoHits{0},MemoChildrenHits{0},MemoCellsCreated{0},MemoEvictions{0},MidpointRequests{0},MidpointHits{0},MidpointBuilds{0},MidpointEvictions{0},RetainedMiddleRequests{0},RetainedMiddleHits{0},RetainedMiddlePublications{0},OrdinaryBoxRecomputedCP{0},OrdinaryBoxAvoidedCP{0},KernelTimeCalls{0},KernelTimeTicks{0},KernelSpatialCalls{0},KernelSpatialTicks{0},KernelSetupCalls{0},KernelSetupTicks{0},KernelUnwinding{0},AffineConsidered{0},AffineSelected{0},AffineGeneric{0},AffineUnknown{0},AffineBranchCalls{0},GenericBranchCalls{0},AffineBranchTicks{0},GenericBranchTicks{0},AffineSpatialCells{0},GenericSpatialCells{0},AffineTimeWork{0},GenericTimeWork{0},DominatedProviders{0};std::atomic<bool> Integrity{true};std::atomic<uint64> HashCalls84[5]{},HashFinished84[5]{},HashBoundaryFlight84[5]{},HashTicks84[5]{},HashUnwinding84[5]{};
 std::atomic<uint64> HashSamples84{0},HashCaptureFlight84{0};std::atomic<bool> HashIntegrity84{true};};
FRow Rows[RowCount];std::atomic<bool> ObservedEnabled{false};
// New diagnostic family only; no counter is a physical/admission input.
struct FSelectionObservation {std::atomic<uint64> Counts[15]{};std::atomic<bool> Integrity{true};};
FSelectionObservation SelectionObservation;
void ObserveSelection(const Speed::Analytic::Certified::FReferenceSelectionWork& w,bool complete)
{
 const uint64 values[]={1,uint64(complete),uint64(!complete),uint64(w.UsedBvh),w.NodesVisited,w.ProviderBoundsTested,w.RadiusPredicates,
  w.CandidatesBeforeSegment,w.CandidatesAfterSegment,w.SegmentTests,w.SegmentExcluded,w.SegmentUncertain,w.SegmentAxes,w.SegmentPairs,w.SegmentInapplicable};
 for(int32 i=0;i<15;++i)if(SelectionObservation.Counts[i].fetch_add(values[i])>MAX_uint64-values[i])SelectionObservation.Integrity.store(false);
}

// Independent Source86 build/preproof costs. BuildTicks INCLUDES SubsetProofTicks.
struct FAffineObservation86 {std::atomic<uint64> Counts[18]{},InFlight{0};std::atomic<bool> Integrity{true};};
FAffineObservation86 AffineObservation86;
void ObserveAffineBuild86(const Speed::Analytic::Prepared::FReferenceAffineBuildObservation86& w,
    Speed::Analytic::Certified::AffineReferenceDetail::EOutcome status,uint64 start)
{
 using E=Speed::Analytic::Certified::AffineReferenceDetail::EOutcome;const uint64 end=FPlatformTime::Cycles64();
 if(!w.Integrity||end<start)AffineObservation86.Integrity.store(false);
 const uint64 values[]={1,uint64(status==E::Complete),uint64(status==E::NotApplicable),uint64(status!=E::Complete&&status!=E::NotApplicable),
  w.InitialNoCommonAxis,w.CoverageNotApplicable,w.ProjectionNotApplicable,w.SubsetAttempts,w.BlockerTests,w.BlockerSeparated,w.BlockerUnresolved,
  w.SubsetUnknown,w.SubsetPlans,w.SubsetRetainedProviders,w.SubsetSeparatedProviders,w.SubsetProofWork,w.SubsetProofTicks,end>=start?end-start:0};
 static_assert(UE_ARRAY_COUNT(values)==18,"Affine86 diagnostic field count");
 for(int32 i=0;i<18;++i)if(AffineObservation86.Counts[i].fetch_add(values[i])>MAX_uint64-values[i])AffineObservation86.Integrity.store(false);
}

bool Enabled()
{static const IConsoleVariable* cvar=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"));return cvar&&cvar->GetInt()!=0;}
void Add(FRow& row,std::atomic<uint64>& value,uint64 delta)
{if(value.fetch_add(delta)>MAX_uint64-delta)row.Integrity.store(false);}
struct FScope
{
 FRow& Row;bool Active;uint64 Start=0;int Uncaught=0;
 explicit FScope(ERow id):Row(Rows[id]),Active(Enabled())
 {if(Active){ObservedEnabled.store(true);Add(Row,Row.Calls,1);Add(Row,Row.InFlight,1);Uncaught=std::uncaught_exceptions();Start=FPlatformTime::Cycles64();}}
 ~FScope() noexcept
 {if(!Active)return;const uint64 end=FPlatformTime::Cycles64();if(end<Start)Row.Integrity.store(false);else Add(Row,Row.Ticks,end-Start);if(std::uncaught_exceptions()>Uncaught)Add(Row,Row.Unwinding,1);Add(Row,Row.Finished,1);if(Row.InFlight.fetch_sub(1)==0)Row.Integrity.store(false);}
 void Budget(uint64 before,uint64 after)
 {if(!Active)return;if(after<before)Row.Integrity.store(false);else Add(Row,Row.BudgetUsed,after-before);}
 void Partition(int32 candidates,int32 excluded)
 {if(!Active)return;if(candidates<0||excluded<0){Row.Integrity.store(false);return;}Add(Row,Row.Candidates,uint64(candidates));Add(Row,Row.Excluded,uint64(excluded));}
};
// Source89 observation family; nested Contact costs are not additive.
struct FGenericRow89 {std::atomic<uint64> C[14]{},InFlight{0};std::atomic<bool> Integrity{true};};
FGenericRow89 Generic89[25];
int32 Bucket89(Speed::PreparedExecution::FFixedSphereProbeDiagnostic89 d)
{
    using S=Speed::Analytic::EStaticQuerySite;
    const int32 site=d.Site==uint8(S::WheelSuspensionProbe)?0:d.Site==uint8(S::WheelEstablishedSupportProbe)?1:-1;
    return site<0||d.Phase>=6||d.Attempt<1||d.Attempt>2?0:1+site*12+int32(d.Phase)*2+int32(d.Attempt)-1;
}
struct FGenericScope89
{
    FGenericRow89& R;bool Active;bool& Kernel;uint64 Start=0;int Uncaught=0;
    Speed::Analytic::Certified::ETensorSphereContactStatus Result=Speed::Analytic::Certified::ETensorSphereContactStatus::Unknown;
    FGenericScope89(Speed::PreparedExecution::FFixedSphereProbeDiagnostic89 d,bool active,bool& kernel):R(Generic89[Bucket89(d)]),Active(active),Kernel(kernel)
    {if(Active){Add(0,1);R.InFlight.fetch_add(1);Start=FPlatformTime::Cycles64();Uncaught=std::uncaught_exceptions();}}
    void Add(int32 k,uint64 v){if(R.C[k].fetch_add(v)>MAX_uint64-v)R.Integrity.store(false);}
    void Proof(Speed::Analytic::Certified::EObliqueCapsule89 status,uint64 tested,uint64 before,uint64 after,uint64 start)
    {
        if(!Active)return;const uint64 end=FPlatformTime::Cycles64();if(after<before||end<start){R.Integrity.store(false);return;}
        using E=Speed::Analytic::Certified::EObliqueCapsule89;
        Add(status==E::Separated?2:status==E::Near?3:4,1);Add(5,tested);Add(6,after-before);Add(7,end-start);
    }
    ~FGenericScope89() noexcept
    {
        if(!Active)return;using E=Speed::Analytic::Certified::ETensorSphereContactStatus;
        Add(1,uint64(Kernel));Add(Result==E::FirstContactEnclosed?8:Result==E::NoContactWithProvidedTensors?9:10,1);
        const uint64 end=FPlatformTime::Cycles64();if(end<Start)R.Integrity.store(false);else Add(12,end-Start);
        Add(13,uint64(std::uncaught_exceptions()>Uncaught));Add(11,1);if(!R.InFlight.fetch_sub(1))R.Integrity.store(false);
    }
};
void PublishGeneric89(const TCHAR* scope)
{
    static const TCHAR* fields[]={TEXT("GenericEntryCalls"),TEXT("ActualGenericKernelCalls"),TEXT("WholeQueryEarlySeparated"),TEXT("Near"),TEXT("Unknown"),TEXT("CandidatesTested"),TEXT("Work"),TEXT("ProofTicks"),TEXT("GenericHit"),TEXT("GenericMiss"),TEXT("GenericOther"),TEXT("Finished"),TEXT("Ticks"),TEXT("Unwinding")};
    for(int32 bucket=0;bucket<25;++bucket)
    {
        auto& row=Generic89[bucket];uint64 v[14]{};for(int32 k=0;k<14;++k)v[k]=row.C[k].load();if(!v[0])continue;
        bool coherent=row.Integrity.load()&&row.InFlight.load()==0&&v[0]==v[11]&&v[13]==0;
        for(int32 k=0;k<14;++k)coherent=coherent&&v[k]==row.C[k].load();coherent=coherent&&row.InFlight.load()==0;
        const int32 site=bucket?(bucket-1)/12:-1,phase=bucket?((bucket-1)%12)/2:-1,attempt=bucket?((bucket-1)%2)+1:0;
        FString line=FString::Printf(TEXT("[ObliqueCapsule89] Schema=ObliqueCapsule89/v1 Scope=%s Row=Contact PerCase=0 TerminalQueryJoin=0 NoPASS=1 Inclusive=1 NotAdditive=1 PreproofActive=0 SnapshotCoherent=%d Site=%s Phase=%d Attempt=%d SecondsPerCycle=%.17g"),scope,int32(coherent),site==0?TEXT("WheelSuspensionProbe"):site==1?TEXT("WheelEstablishedSupportProbe"):TEXT("UNKNOWN"),phase,attempt,FPlatformTime::GetSecondsPerCycle64());
        for(int32 k=0;k<14;++k)line+=FString::Printf(TEXT(" %s=%llu"),fields[k],v[k]);UE_LOG(LogTemp,Display,TEXT("%s"),*line);
    }
}

void PublishSnapshot(const TCHAR* scope)
{
 static constexpr const TCHAR* names[]={TEXT("Producer"),TEXT("RadiusSelection"),TEXT("Contact"),TEXT("ContactConsumer")};
 FString text=FString::Printf(TEXT("[CanonicalSphereStageProfile70] Scope=%s PerCase=0 TerminalQueryJoin=0 NoPASS=1 NoCachePermission=1 InclusiveRowsNotAdditive=1 Enabled=%d SecondsPerCycle=%.17g"),scope,int32(ObservedEnabled.load()),FPlatformTime::GetSecondsPerCycle64());
 for(uint32 i=0;i<RowCount;++i)
 {
  auto& r=Rows[i];const uint64 calls=r.Calls.load(),flight=r.InFlight.load(),finished=r.Finished.load(),ticks=r.Ticks.load(),unwinding=r.Unwinding.load(),budget=r.BudgetUsed.load(),candidates=r.Candidates.load(),excluded=r.Excluded.load(),accounting=r.ContactAccounting.load(),time=r.TimeWork.load(),cells=r.SpatialCells.load(),memoRequests=r.MemoRequests.load(),memoHits=r.MemoHits.load(),memoChildren=r.MemoChildrenHits.load(),memoCells=r.MemoCellsCreated.load(),memoEvictions=r.MemoEvictions.load(),midpointRequests=r.MidpointRequests.load(),midpointHits=r.MidpointHits.load(),midpointBuilds=r.MidpointBuilds.load(),midpointEvictions=r.MidpointEvictions.load(),retainedMiddleRequests=r.RetainedMiddleRequests.load(),retainedMiddleHits=r.RetainedMiddleHits.load(),retainedMiddlePublications=r.RetainedMiddlePublications.load(),ordinaryBoxRecomputedCP=r.OrdinaryBoxRecomputedCP.load(),ordinaryBoxAvoidedCP=r.OrdinaryBoxAvoidedCP.load(),kernelTimeCalls=r.KernelTimeCalls.load(),kernelTimeTicks=r.KernelTimeTicks.load(),kernelSpatialCalls=r.KernelSpatialCalls.load(),kernelSpatialTicks=r.KernelSpatialTicks.load(),kernelSetupCalls=r.KernelSetupCalls.load(),kernelSetupTicks=r.KernelSetupTicks.load(),kernelUnwinding=r.KernelUnwinding.load(),affineConsidered=r.AffineConsidered.load(),affineSelected=r.AffineSelected.load(),affineGeneric=r.AffineGeneric.load(),affineUnknown=r.AffineUnknown.load(),affineBranchCalls=r.AffineBranchCalls.load(),genericBranchCalls=r.GenericBranchCalls.load(),affineBranchTicks=r.AffineBranchTicks.load(),genericBranchTicks=r.GenericBranchTicks.load(),affineSpatialCells=r.AffineSpatialCells.load(),genericSpatialCells=r.GenericSpatialCells.load(),affineTimeWork=r.AffineTimeWork.load(),genericTimeWork=r.GenericTimeWork.load(),dominatedProviders=r.DominatedProviders.load();
  const bool coherent=r.Integrity.load()&&flight==0&&r.InFlight.load()==0&&calls==r.Calls.load()&&calls==finished;
  text+=FString::Printf(TEXT(" | Row=%s SnapshotCoherent=%d Calls=%llu Finished=%llu InFlight=%llu Ticks=%llu Unwinding=%llu BudgetUsed=%llu Candidates=%llu Excluded=%llu ContactAccounting=%llu TimeWork=%llu SpatialCells=%llu MemoRequests=%llu MemoHits=%llu MemoChildrenHits=%llu MemoCellsCreated=%llu MemoEvictions=%llu MidpointRequests=%llu MidpointHits=%llu MidpointBuilds=%llu MidpointEvictions=%llu RetainedMiddleRequests=%llu RetainedMiddleHits=%llu RetainedMiddlePublications=%llu OrdinaryBoxRecomputedCP=%llu OrdinaryBoxAvoidedCP=%llu KernelTimeCalls=%llu KernelTimeTicks=%llu KernelSpatialCalls=%llu KernelSpatialTicks=%llu KernelSetupCalls=%llu KernelSetupTicks=%llu KernelUnwinding=%llu AffineConsidered=%llu AffineSelected=%llu AffineGeneric=%llu AffineUnknown=%llu AffineBranchCalls=%llu GenericBranchCalls=%llu AffineBranchTicks=%llu GenericBranchTicks=%llu AffineSpatialCells=%llu GenericSpatialCells=%llu AffineTimeWork=%llu GenericTimeWork=%llu DominatedProviders=%llu"),names[i],int32(coherent),calls,finished,flight,ticks,unwinding,budget,candidates,excluded,accounting,time,cells,memoRequests,memoHits,memoChildren,memoCells,memoEvictions,midpointRequests,midpointHits,midpointBuilds,midpointEvictions,retainedMiddleRequests,retainedMiddleHits,retainedMiddlePublications,ordinaryBoxRecomputedCP,ordinaryBoxAvoidedCP,kernelTimeCalls,kernelTimeTicks,kernelSpatialCalls,kernelSpatialTicks,kernelSetupCalls,kernelSetupTicks,kernelUnwinding,affineConsidered,affineSelected,affineGeneric,affineUnknown,affineBranchCalls,genericBranchCalls,affineBranchTicks,genericBranchTicks,affineSpatialCells,genericSpatialCells,affineTimeWork,genericTimeWork,dominatedProviders);
 }
 UE_LOG(LogTemp,Display,TEXT("%s"),*text);
 // Independent observation row; every original StageProfile70 field/log stays intact.
 namespace H84=Speed::Analytic::Certified::Detail::MinimumMidpointProfile84;
 static_assert(H84::StageCount==5,"Profile84 field cardinality");
 static constexpr const TCHAR* hashNames84[]={TEXT("RootHash"),TEXT("CellHash"),TEXT("SameRoot"),TEXT("SameCell"),TEXT("CallMidpoint")};
 auto& h84=Rows[Contact];const uint64 samples84=h84.HashSamples84.load();
 const uint64 contactCalls84=h84.Calls.load(),contactFinished84=h84.Finished.load();
 uint64 hc84[5]{},hf84[5]{},hi84[5]{},ht84[5]{},hu84[5]{};
 bool coherent84=h84.HashIntegrity84.load()&&h84.HashCaptureFlight84.load()==0&&contactCalls84==contactFinished84&&h84.InFlight.load()==0;
 for(uint32 stage=0;stage<5;++stage)
 {
  hc84[stage]=h84.HashCalls84[stage].load();hf84[stage]=h84.HashFinished84[stage].load();hi84[stage]=h84.HashBoundaryFlight84[stage].load();
  ht84[stage]=h84.HashTicks84[stage].load();hu84[stage]=h84.HashUnwinding84[stage].load();
  coherent84=coherent84&&hc84[stage]==hf84[stage]&&hi84[stage]==0&&hc84[stage]==h84.HashCalls84[stage].load()&&
   hf84[stage]==h84.HashFinished84[stage].load()&&ht84[stage]==h84.HashTicks84[stage].load()&&hu84[stage]==h84.HashUnwinding84[stage].load();
 }
 coherent84=coherent84&&samples84==h84.HashSamples84.load()&&h84.HashCaptureFlight84.load()==0&&
  contactCalls84==h84.Calls.load()&&contactFinished84==h84.Finished.load()&&h84.InFlight.load()==0;
 FString hashText84=FString::Printf(TEXT("[MinimumHashProfile84] Schema=MinimumHashProfile84/v1 Scope=%s Row=Contact PerCase=0 TerminalQueryJoin=0 NoPASS=1 Inclusive=1 NotAdditive=1 TimingIncludesInstrumentationOverhead=1 SnapshotCoherent=%d Samples=%llu Enabled=%d SecondsPerCycle=%.17g"),scope,int32(coherent84),samples84,int32(ObservedEnabled.load()),FPlatformTime::GetSecondsPerCycle64());
 for(uint32 stage=0;stage<5;++stage)hashText84+=FString::Printf(TEXT(" %sCalls=%llu %sFinished=%llu %sInFlightAtBoundary=%llu %sTicks=%llu %sUnwinding=%llu"),hashNames84[stage],hc84[stage],hashNames84[stage],hf84[stage],hashNames84[stage],hi84[stage],hashNames84[stage],ht84[stage],hashNames84[stage],hu84[stage]);
 UE_LOG(LogTemp,Display,TEXT("%s"),*hashText84);
 static constexpr const TCHAR* selectionNames[]={TEXT("AttemptedQueries"),TEXT("CompletedQueries"),TEXT("FailedQueries"),TEXT("BvhQueries"),
  TEXT("NodesVisited"),TEXT("ProviderBoundsTested"),TEXT("RadiusPredicates"),TEXT("CandidatesBeforeSegment"),TEXT("CandidatesAfterSegment"),
  TEXT("SegmentTests"),TEXT("SegmentExcluded"),TEXT("SegmentUncertain"),TEXT("SegmentAxes"),TEXT("SegmentPairs"),TEXT("SegmentInapplicable")};
 uint64 selectionValues[15]{};bool selectionCoherent=SelectionObservation.Integrity.load();
 for(int32 i=0;i<15;++i)selectionValues[i]=SelectionObservation.Counts[i].load();
 const auto& selectionRow=Rows[RadiusSelection];const uint64 selectionCalls=selectionRow.Calls.load();
 selectionCoherent=selectionCoherent&&selectionRow.InFlight.load()==0&&selectionCalls==selectionRow.Finished.load()&&selectionRow.Unwinding.load()==0;
 for(int32 i=0;i<15;++i)selectionCoherent=selectionCoherent&&selectionValues[i]==SelectionObservation.Counts[i].load();
 selectionCoherent=selectionCoherent&&selectionCalls==selectionRow.Calls.load()&&selectionRow.InFlight.load()==0;
 FString selectionText=FString::Printf(TEXT("[CertifiedSegmentSelection] Schema=CertifiedSegmentSelection/v1 Scope=%s PerCase=0 NoPASS=1 SnapshotCoherent=%d"),scope,int32(selectionCoherent));
 for(int32 i=0;i<15;++i)selectionText+=FString::Printf(TEXT(" %s=%llu"),selectionNames[i],selectionValues[i]);
 UE_LOG(LogTemp,Display,TEXT("%s"),*selectionText);
 static constexpr const TCHAR* affineNames86[]={TEXT("BuildCalls"),TEXT("BuildComplete"),TEXT("BuildNotApplicable"),TEXT("BuildUnknown"),
  TEXT("InitialNoCommonAxis"),TEXT("CoverageNotApplicable"),TEXT("ProjectionNotApplicable"),TEXT("SubsetAttempts"),TEXT("BlockerTests"),TEXT("BlockerSeparated"),TEXT("BlockerUnresolved"),
  TEXT("SubsetUnknown"),TEXT("SubsetPlans"),TEXT("SubsetRetainedProviders"),TEXT("SubsetSeparatedProviders"),TEXT("SubsetProofWork"),TEXT("SubsetProofTicks"),TEXT("BuildTicks")};
 static_assert(UE_ARRAY_COUNT(affineNames86)==18,"Affine86 diagnostic names");
 uint64 affineValues86[18]{};bool affineCoherent86=AffineObservation86.Integrity.load()&&AffineObservation86.InFlight.load()==0;
 for(int32 i=0;i<18;++i)affineValues86[i]=AffineObservation86.Counts[i].load();
 for(int32 i=0;i<18;++i)affineCoherent86=affineCoherent86&&affineValues86[i]==AffineObservation86.Counts[i].load();
 affineCoherent86=affineCoherent86&&AffineObservation86.InFlight.load()==0;
 FString affineText86=FString::Printf(TEXT("[CertifiedAffineSubset86] Schema=CertifiedAffineSubset86/v1 Scope=%s PerCase=0 TerminalQueryJoin=0 NoPASS=1 Inclusive=1 NotAdditive=1 SnapshotCoherent=%d SecondsPerCycle=%.17g"),scope,int32(affineCoherent86),FPlatformTime::GetSecondsPerCycle64());
 for(int32 i=0;i<18;++i)affineText86+=FString::Printf(TEXT(" %s=%llu"),affineNames86[i],affineValues86[i]);
 UE_LOG(LogTemp,Display,TEXT("%s"),*affineText86);
 PublishGeneric89(scope);
}

void Publish()
{
 PublishSnapshot(TEXT("ProgramPrefixAtPreExit"));
 ::ParityContactCost::Publish();
 ::HistoricalQueryCache94::Publish();
 ::Speed::Analytic::Certified::PublishPlanarFamilyShortcutObservation();
 ::DirectPlaneProposal95Profile::Publish();
 ::CurveStationaryContact98Profile::Publish();

}
// Same-DLL static registration only, during module load before any worker.
// The callback runs before module shutdown; it reads diagnostics only.
struct FRegister {FRegister(){FCoreDelegates::OnPreExit.AddStatic(&Publish);}};
FRegister Register;
}

// Source91 diagnostics are values-only, nested and non-additive. No result,
// policy value or inherited cap is changed by the observer.
namespace InheritedReachProfile91
{
using R=Speed::Analytic::Certified::EInheritedReachRoute91;
struct FRow
{
    std::atomic<uint64> Calls{0},Eligible{0},Invalid{0},Outside{0},Near{0},Cleared{0},Unknown{0},
        ContinueOriginal{0},Work{0},Ticks{0},Finished{0},InFlight{0},Unwinding{0};
    std::atomic<bool> Integrity{true};
};
FRow Rows[uint8(R::Count)];std::atomic<uint64> InitCaptured{0},InitMissing{0};
std::atomic<bool> InitIntegrity{true};
void Add(FRow& row,std::atomic<uint64>& field,uint64 value)
{if(field.fetch_add(value)>MAX_uint64-value)row.Integrity.store(false);}
void Publish()
{
    const TCHAR* names[]={TEXT("SolidSphere"),TEXT("SolidBox"),TEXT("WheelSuspension"),TEXT("WheelSupport"),TEXT("UNKNOWN")};
    for(int32 i=0;i<int32(R::Count);++i)
    {
        auto& r=Rows[i];const uint64 c=r.Calls.load(),e=r.Eligible.load(),invalid=r.Invalid.load(),outside=r.Outside.load(),
            near=r.Near.load(),clear=r.Cleared.load(),unknown=r.Unknown.load(),full=r.ContinueOriginal.load(),
            work=r.Work.load(),ticks=r.Ticks.load(),finished=r.Finished.load(),flight=r.InFlight.load(),unwind=r.Unwinding.load();
        const uint64 init=InitCaptured.load(),missing=InitMissing.load();
        bool coherent=r.Integrity.load()&&InitIntegrity.load()&&flight==0&&unwind==0&&c==finished&&
            c==invalid+outside+near+clear+unknown&&e==near+clear&&full==invalid+outside+near;
        coherent=coherent&&c==r.Calls.load()&&finished==r.Finished.load()&&work==r.Work.load()&&
            ticks==r.Ticks.load()&&r.InFlight.load()==0;
        UE_LOG(LogTemp,Display,TEXT("[InheritedReach91] Schema=InheritedReach91/v1 Scope=ProgramPrefixAtPreExit PerCase=0 TerminalQueryJoin=0 NoPASS=1 Inclusive=1 NotAdditive=1 TimingIncludesInstrumentationOverhead=1 Route=%s SnapshotCoherent=%d InitCaptured=%llu InitMissing=%llu Calls=%llu Eligible=%llu Invalid=%llu Outside=%llu Near=%llu Cleared=%llu Unknown=%llu ContinueOriginal=%llu ProofWork=%llu ProofTicks=%llu Finished=%llu InFlight=%llu Unwinding=%llu SecondsPerCycle=%.17g"),
            names[i],int32(coherent),init,missing,c,e,invalid,outside,near,clear,unknown,full,work,ticks,finished,flight,unwind,FPlatformTime::GetSecondsPerCycle64());
    }
}
struct FRegister{FRegister(){FCoreDelegates::OnPreExit.AddStatic(&Publish);}};FRegister Register;
}
namespace Speed
{
void ObserveInheritedMotionInitialization91(bool captured)
{
    auto& field=captured?InheritedReachProfile91::InitCaptured:InheritedReachProfile91::InitMissing;
    if(field.fetch_add(1)==MAX_uint64)InheritedReachProfile91::InitIntegrity.store(false);
}
}
namespace Speed::Analytic::Certified
{
FInheritedReachObservation91::FInheritedReachObservation91(EInheritedReachRoute91 route,FDistanceBudget& budget)
    :Route(route),Budget(budget),Before(budget.Used),Active(::StadiumSphereProfile70::Enabled())
{
    if(!Active)return;auto& r=::InheritedReachProfile91::Rows[uint8(Route)];
    ::InheritedReachProfile91::Add(r,r.Calls,1);::InheritedReachProfile91::Add(r,r.InFlight,1);
    Start=FPlatformTime::Cycles64();Uncaught=std::uncaught_exceptions();
}
FInheritedReachObservation91::~FInheritedReachObservation91() noexcept
{
    if(!Active)return;using namespace ::InheritedReachProfile91;auto& r=Rows[uint8(Route)];
    const bool unwinding=std::uncaught_exceptions()>Uncaught;
    const auto status=unwinding?EInheritedReach91::Unknown:Result;
    switch(status)
    {
    case EInheritedReach91::Invalid:Add(r,r.Invalid,1);break;
    case EInheritedReach91::Outside:Add(r,r.Outside,1);break;
    case EInheritedReach91::Near:Add(r,r.Near,1);Add(r,r.Eligible,1);break;
    case EInheritedReach91::Cleared:Add(r,r.Cleared,1);Add(r,r.Eligible,1);break;
    default:Add(r,r.Unknown,1);break;
    }
    if(status==EInheritedReach91::Invalid||status==EInheritedReach91::Outside||status==EInheritedReach91::Near)Add(r,r.ContinueOriginal,1);
    const uint64 end=FPlatformTime::Cycles64();
    if(end<Start||Budget.Used<Before)r.Integrity.store(false);
    else {Add(r,r.Work,Budget.Used-Before);Add(r,r.Ticks,end-Start);}
    Add(r,r.Unwinding,uint64(unwinding));Add(r,r.Finished,1);if(!r.InFlight.fetch_sub(1))r.Integrity.store(false);
}
}

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
// Actual pure helper draft; original FDistanceBudget remains the only account.
namespace Speed::PreparedExecution::BoundedLegacyWheelDetail
{
enum class EOptionalAdmission : uint8 {Unavailable,Ready,InvalidAccount};
inline EOptionalAdmission CanBeginOptional(const Analytic::Certified::FDistanceBudget& budget,
    uint64 minimumWork, uint64 localCap)
{
    if(budget.ConsumptionFloor<0 || budget.Remaining<budget.ConsumptionFloor)
        return EOptionalAdmission::InvalidAccount;
    if(!minimumWork || minimumWork>localCap ||
        minimumWork>uint64(int64(budget.Remaining)-int64(budget.ConsumptionFloor)) ||
        minimumWork>MAX_uint64-budget.Used)
        return EOptionalAdmission::Unavailable;
    return EOptionalAdmission::Ready;
}
// Sticky partial failure: this local view neither owns nor restores the budget.
struct FOptionalWork final
{
    Analytic::Certified::FDistanceBudget& Budget;
    uint64 InitialUsed;
    uint64 LocalCap;
    bool Failed=false;
    FOptionalWork(Analytic::Certified::FDistanceBudget& budget,uint64 localCap)
        :Budget(budget),InitialUsed(budget.Used),LocalCap(localCap){}
    bool Pay()
    {
        if(Failed || Budget.Used<InitialUsed || Budget.Used-InitialUsed>=LocalCap || !Budget.Consume())
        {Failed=true;return false;}
        return true;
    }
};
}

// Private APPROXIMATE local-normal recovery only. The radial belongs to the
// authenticated original point at the proposed consumed time; it is neither
// the first-contact normal nor a proof of global nearest distance.
namespace Speed::PreparedExecution::BoundedLegacyRadialNormal
{
namespace A=::AnalyticCertified;
namespace C=::Speed::Analytic::Certified;
namespace M=C::FixedWheelProbeMath;
using I=A::Interval;
TAutoConsoleVariable<int32> Enabled(TEXT("p.SkyLeague.Stadium.BoundedLegacyWheelOriginalRadialNormal"),0,
    TEXT("PRIVATE OFF: after a disjoint Legacy normal cell, propose the original local radial midpoint and preserve the same strict quantizer. No exact TOI/first-contact normal."));
int32 Policy(){return Enabled.GetValueOnAnyThread()==1?1:0;}
std::atomic<uint64> FirstFailures[uint8(M::EQuantizedNormalFailure::NumericContext)+1]{};
std::atomic<uint64> Attempts{0},Recovered{0},RetryRefused{0},MaxQuantizedChangeL1Bits{0};
void Publish()
{
    UE_LOG(LogTemp,Display,TEXT("[BoundedLegacyWheelOriginalRadialNormal] Scope=ProgramPrefixAtPreExit NoPASS=1 PreClosureSamples=1 DefaultOFF=1 LocalApproximateNormalOnly=1 Enabled=%d Attempts=%llu Recovered=%llu RetryRefused=%llu MaxQuantizedChangeL1=%.17g"),
        Policy(),Attempts.load(),Recovered.load(),RetryRefused.load(),A::FromBits(MaxQuantizedChangeL1Bits.load()));
    const TCHAR* names[]={TEXT("None"),TEXT("Work"),TEXT("Input"),TEXT("RadialBounds"),TEXT("NormalizeBounds"),TEXT("ScalarNormalize"),TEXT("CellArithmetic"),TEXT("AmbiguousCell"),TEXT("DisjointCell"),TEXT("QuantizedMismatch"),TEXT("ConsumedNormalize"),TEXT("NumericContext")};
    static_assert(UE_ARRAY_COUNT(names)==UE_ARRAY_COUNT(FirstFailures));
    for(int32 index=1;index<UE_ARRAY_COUNT(names);++index)if(FirstFailures[index].load())
        UE_LOG(LogTemp,Display,TEXT("[BoundedLegacyWheelOriginalNormalRefused] ProfileOnly=1 Failure=%s Count=%llu"),names[index],FirstFailures[index].load());
}
struct FRegister {FRegister(){FCoreDelegates::OnPreExit.AddStatic(&Publish);}};FRegister Register;
// Seven consumes for the unchanged first test. A retry adds one proposed
// midpoint/normalization, seven unchanged normal-test consumes and one L1
// bound. The existing local 32/64 caps and total query cap are unchanged.
static bool Try(const C::FIntervalPoint& radial,FVector legacyStored,int32 policy,C::FDistanceBudget& budget,
    M::FUniqueQuantizedNormal& output,FVector& stored,I& change)
{
    output={};stored=FVector::ZeroVector;change={};
    if(policy!=0&&policy!=1)return false;
    const bool observe=::StadiumSphereProfile70::Enabled();
    M::FUniqueQuantizedNormal original;M::FQuantizedNormalDiagnostic diagnostic;
    if(M::TryUniqueQuantizedNormal(radial,legacyStored,budget,original,&diagnostic))
    {output=original;stored=legacyStored;change=I::Exact(0.);return true;}
    if(observe)++FirstFailures[uint8(diagnostic.Failure)];
    // Tightening or selecting a different scalar cannot cure an enclosure
    // genuinely crossing a strict boundary. No retry for arithmetic or work.
    if(policy!=1||diagnostic.Failure!=M::EQuantizedNormalFailure::DisjointCell)return false;
    if(observe)++Attempts;
    const auto refused=[&](){if(observe)++RetryRefused;return false;};
    if(!budget.Consume())return refused();
    FVector proposed=FVector::ZeroVector;
    for(int32 axis=0;axis<3;++axis)
    {
        if(!A::Valid(radial.Axis[axis]))return refused();
        proposed[axis]=.5*radial.Axis[axis].Lower+.5*radial.Axis[axis].Upper;
    }
    if(!M::ValidVector(proposed)||!proposed.Normalize()||!M::ValidVector(proposed))return refused();
    M::FUniqueQuantizedNormal replacement;
    if(!M::TryUniqueQuantizedNormal(radial,proposed,budget,replacement))return refused();
    if(!budget.Consume())return refused();
    // Bound the actual two discrete normals that native delivery would use.
    // This is observed change, not a new admission tolerance or TOI bound.
    const FVector oldQuantized=Speed::QuantizeUnitNormal(legacyStored,1.e-3f);
    if(!M::ValidVector(oldQuantized))return refused();
    I difference=I::Exact(0.);
    for(int32 axis=0;axis<3;++axis)
    {
        const I delta=A::Subtract(I::Exact(oldQuantized[axis]),I::Exact(replacement.Quantized[axis]));
        if(!A::Valid(delta))return refused();
        const double lo=A::FromBits(A::Bits(delta.Lower)&A::Magnitude),hi=A::FromBits(A::Bits(delta.Upper)&A::Magnitude);
        difference=A::Add(difference,I::Exact(FMath::Max(lo,hi)));
    }
    if(!A::Valid(difference))return refused();
    output=replacement;stored=proposed;change=difference;
    if(observe)
    {
        ++Recovered;const uint64 value=A::Bits(difference.Upper);uint64 current=MaxQuantizedChangeL1Bits.load();
        while(current<value&&!MaxQuantizedChangeL1Bits.compare_exchange_weak(current,value)){}
    }
    return true;
}
}

// Conditional LOCAL sample validation. This is NOT a TOI/contact certificate.
// Caller must authenticate product as the same fixed source, the complete
// inflated partition, current policy and the actual stored rectangle atlas.
// Requires ReferenceAffineExtrusionMath.h, ContinuousCenterPath.h,
// CanonicalFixedSphereWheelProbeMath.h and budget-helper.hpp.inc.
namespace Speed::PreparedExecution::BoundedLegacyWheelDetail
{
namespace A=::AnalyticCertified;
namespace C=Speed::Analytic::Certified;
namespace M=C::FixedWheelProbeMath;
using I=A::Interval;
enum class ELocalSample : uint8 {Unavailable,BoundedLocalSample,Work,Numeric};
enum class ELocalRefusal:uint8 {Input,Partition,Owner,OriginalFact,OriginalUV,OriginalEvaluate,Regularity,NativeTime,Approach,PositionSlack,NormalCell,ReferenceNormal,Binding,Conversion,Count};
inline void Reason(ELocalRefusal* reason,ELocalRefusal value){if(reason)*reason=value;}
struct FLocalSample
{
    C::FIntervalPoint OriginalPoint,NativeSampleCenter,RadialAtSample;
    I PositionError;
    M::FUniqueQuantizedNormal Normal;
    float NativeSeconds=0;
    uint64 NumericControl=0;
    FVector StoredNormal=FVector::ZeroVector;
    int32 NormalPolicy=0;
    I QuantizedNormalChange=I::Exact(0.);
};
inline I L1Difference(const C::FIntervalPoint& first,const C::FIntervalPoint& second)
{
    if(!C::Detail::ValidPoint(first)||!C::Detail::ValidPoint(second))return {};
    I sum=I::Exact(0.);
    for(int32 axis=0;axis<3;++axis)
    {
        const auto difference=A::Subtract(first.Axis[axis],second.Axis[axis]);
        if(!A::Valid(difference))return {};
        const double lo=A::FromBits(A::Bits(difference.Lower)&A::Magnitude);
        const double hi=A::FromBits(A::Bits(difference.Upper)&A::Magnitude);
        sum=A::Add(sum,I::Exact(FMath::Max(lo,hi)));
    }
    return sum;
}
inline ELocalSample InspectFixedPointSample(const C::FIntervalPoint& sourcePoint,const Analytic::FWorldQuery& query,const Analytic::FWorldHit& hit,
    double actualDelta,double physicalRadius,C::FDistanceBudget& budget,FLocalSample& output,ELocalRefusal* reason=nullptr,int32 normalPolicy=0)
{
    Reason(reason,ELocalRefusal::Input);output={};const auto before=Analytic::InspectCurrentAnalyticNumericContext(
        Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(before.Status!=Analytic::EAnalyticNumericStatus::Complete)return ELocalSample::Numeric;
    const auto admission=CanBeginOptional(budget,32,32);
    if(admission==EOptionalAdmission::InvalidAccount)return ELocalSample::Work;
    if(admission!=EOptionalAdmission::Ready)return ELocalSample::Unavailable;
    FOptionalWork work(budget,32);
    C::FScopedDistanceBudgetFloor floor(budget,FMath::Max(budget.ConsumptionFloor,budget.Remaining-32));
    if(!floor.IsValid()||!work.Pay())return ELocalSample::Work;
    if(!hit.bHit||hit.bStartPenetrating||hit.PenetrationDepth!=0||
        hit.SurfaceFeatureKind!=EContactFeatureKind::Face||
        !M::ValidVector(hit.Location)||!M::ValidVector(hit.Point)||!M::ValidVector(hit.QueryPoint)||
        !M::ValidVector(hit.Normal)||!M::NormalOrZero(hit.Time)||!(hit.Time>0&&hit.Time<1)||
        !M::NormalOrZero(actualDelta)||!(actualDelta>0)||
        !M::NormalOrZero(physicalRadius)||!(physicalRadius>0)||
        !M::NormalOrZero(query.Radius)||!(query.Radius>physicalRadius)||
        !C::Detail::ValidPoint(sourcePoint))
        return ELocalSample::Unavailable;
    const auto slack=A::Subtract(I::Exact(query.Radius),I::Exact(physicalRadius));
    if(!A::Valid(slack)||!(slack.Lower>0))return ELocalSample::Unavailable;
    FLocalSample value;value.OriginalPoint=sourcePoint;
    if(!work.Pay())return ELocalSample::Work;
    // Exactly the existing wheel seconds conversion, never a fabricated
    // FBoundedFixedProbeSample tied to a made-up first-contact enclosure.
    Reason(reason,ELocalRefusal::NativeTime);
    value.NativeSeconds=static_cast<float>(hit.Time*actualDelta);
    uint32 secondsBits=0;FMemory::Memcpy(&secondsBits,&value.NativeSeconds,4);
    if((secondsBits&0x80000000u)||secondsBits>=0x7f800000u||
        (secondsBits&&secondsBits<0x00800000u))return ELocalSample::Unavailable;
    const auto parameter=M::DividePositive(I::Exact(double(value.NativeSeconds)),I::Exact(actualDelta));
    C::FIntervalBezierCenterPath path;
    if(!A::Valid(parameter)||parameter.Lower<0||parameter.Upper>1||
        !C::CenterDetail::Linear(query.Start,query.End,path))return ELocalSample::Unavailable;
    value.NativeSampleCenter=C::CenterDetail::Evaluate(path,parameter);
    if(!C::Detail::ValidPoint(value.NativeSampleCenter))return ELocalSample::Unavailable;
    if(!work.Pay())return ELocalSample::Work;
    C::FIntervalPoint radial,direction;
    for(int32 axis=0;axis<3;++axis)
    {
        radial.Axis[axis]=A::Subtract(value.NativeSampleCenter.Axis[axis],value.OriginalPoint.Axis[axis]);
        direction.Axis[axis]=A::Subtract(I::Exact(query.End[axis]),I::Exact(query.Start[axis]));
    }
    if(!work.Pay())return ELocalSample::Work;
    Reason(reason,ELocalRefusal::Approach);
    if(!M::BoundFirstNormalize(radial,value.RadialAtSample))return ELocalSample::Unavailable;
    const auto approach=C::ContactDetail::Dot(direction,value.RadialAtSample);
    if(!A::Valid(approach)||!(approach.Upper<0))return ELocalSample::Unavailable;
    Reason(reason,ELocalRefusal::PositionSlack);
    const auto pointError=L1Difference(value.OriginalPoint,C::Detail::Point(hit.Point));
    const auto queryPointError=L1Difference(value.OriginalPoint,C::Detail::Point(hit.QueryPoint));
    const auto centerError=L1Difference(value.NativeSampleCenter,C::Detail::Point(hit.Location));
    const auto radialLength=M::SqrtPositive(C::ContactDetail::Dot(radial,radial));
    const auto residual=A::Subtract(radialLength,I::Exact(query.Radius));
    if(!A::Valid(pointError)||!A::Valid(queryPointError)||!A::Valid(centerError)||!A::Valid(residual))
        return ELocalSample::Unavailable;
    const double residualUpper=FMath::Max(A::FromBits(A::Bits(residual.Lower)&A::Magnitude),
        A::FromBits(A::Bits(residual.Upper)&A::Magnitude));
    value.PositionError=A::Add(A::Add(I::Exact(FMath::Max(pointError.Upper,queryPointError.Upper)),centerError),I::Exact(residualUpper));
    if(!A::Valid(value.PositionError)||value.PositionError.Upper>slack.Lower)return ELocalSample::Unavailable;
    // Existing quantization and strict cell test, on this LOCAL radial sample.
    Reason(reason,ELocalRefusal::NormalCell);
    const uint64 normalBegan=budget.Used;
    if(!BoundedLegacyRadialNormal::Try(value.RadialAtSample,hit.Normal,normalPolicy,budget,value.Normal,value.StoredNormal,value.QuantizedNormalChange))
        return budget.Remaining<=budget.ConsumptionFloor||budget.Used==MAX_uint64?ELocalSample::Work:ELocalSample::Unavailable;
    if(budget.Used<normalBegan||budget.Used-work.InitialUsed>work.LocalCap)return ELocalSample::Work;
    if(query.MinimumReferenceNormalDot>-1)
    {
        Reason(reason,ELocalRefusal::ReferenceNormal);
        if(!work.Pay())return ELocalSample::Work;
        const FVector3d reference=query.ReferenceNormal.GetSafeNormal();
        const auto alignment=C::ContactDetail::Dot(C::Detail::Point(reference),value.RadialAtSample);
        if(reference.IsZero()||!A::Valid(alignment)||alignment.Lower<query.MinimumReferenceNormalDot||
            FVector3d::DotProduct(reference,value.StoredNormal)<query.MinimumReferenceNormalDot)return ELocalSample::Unavailable;
    }
    const auto after=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(after.Status!=Analytic::EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=before.ControlFingerprint())
        return ELocalSample::Numeric;
    value.NumericControl=before.ControlFingerprint();value.NormalPolicy=normalPolicy;output=MoveTemp(value);
    return ELocalSample::BoundedLocalSample;
}

inline ELocalSample InspectFixedProductSample(const C::AffineReferenceDetail::FProduct& product,
    FVector2d proposedUV,const Analytic::FWorldQuery& query,const Analytic::FWorldHit& hit,
    double actualDelta,double physicalRadius,C::FDistanceBudget& budget,FLocalSample& output,ELocalRefusal* reason=nullptr,int32 normalPolicy=0)
{
    Reason(reason,ELocalRefusal::OriginalUV);output={};const auto admission=CanBeginOptional(budget,64,64);
    if(admission==EOptionalAdmission::InvalidAccount)return ELocalSample::Work;
    if(admission!=EOptionalAdmission::Ready)return ELocalSample::Unavailable;
    FOptionalWork work(budget,64);
    C::FScopedDistanceBudgetFloor floor(budget,FMath::Max(budget.ConsumptionFloor,budget.Remaining-64));
    if(!floor.IsValid()||!work.Pay())return ELocalSample::Work;
    if(!M::NormalOrZero(proposedUV.X)||!M::NormalOrZero(proposedUV.Y)||
        !(proposedUV.X>0&&proposedUV.X<1&&proposedUV.Y>0&&proposedUV.Y<1)||
        product.FreeWorldAxis<0||product.FreeWorldAxis>2||product.Curve.DegreeU!=0||
        product.Curve.DegreeV!=5||product.Curve.ControlPoints.Num()!=6)return ELocalSample::Unavailable;
    for(int32 cp=0;cp<6;++cp)if(!work.Pay())return ELocalSample::Work;
    C::FIntervalPoint point;
    Reason(reason,ELocalRefusal::OriginalEvaluate);
    if(!C::AffineReferenceDetail::EvaluateFixedProductUV(product,proposedUV,point))return ELocalSample::Unavailable;
    if(!work.Pay())return ELocalSample::Work;
    // Differentiate the ORIGINAL curve, then evaluate at the proposed global
    // curve parameter. A broad global derivative hull can contain zero in
    // every component on a regular arc and is deliberately not used here.
    Reason(reason,ELocalRefusal::Regularity);
    const auto slope=A::Subtract(product.End,product.Start);
    if(!A::Valid(slope)||!(slope.Lower>0||slope.Upper<0))return ELocalSample::Unavailable;
    C::FIntervalTensorSurface derivative;derivative.DegreeU=0;derivative.DegreeV=4;
    for(int32 cp=0;cp<5;++cp)
    {
        if(!work.Pay())return ELocalSample::Work;
        C::FIntervalPoint value;
        for(int32 axis=0;axis<3;++axis)value.Axis[axis]=A::Multiply(I::Exact(5.),
            A::Subtract(product.Curve.ControlPoints[cp+1].Axis[axis],product.Curve.ControlPoints[cp].Axis[axis]));
        if(!C::Detail::ValidPoint(value))return ELocalSample::Unavailable;
        derivative.ControlPoints.Add(value);
    }
    if(!work.Pay())return ELocalSample::Work;
    C::FIntervalPoint localDerivative;
    if(!C::ProjectedDistanceDetail::EvaluateFixedUV(derivative,FVector2d(.5,product.FreeAlongU?proposedUV.Y:proposedUV.X),localDerivative))
        return ELocalSample::Unavailable;
    auto freeDerivative=C::Detail::Point(FVector3d::ZeroVector);
    freeDerivative.Axis[product.FreeWorldAxis]=slope;bool regular=false;
    for(int32 axis=0;axis<3;++axis)
    {
        if(!work.Pay())return ELocalSample::Work;
        const auto cross=A::Subtract(
            A::Multiply(freeDerivative.Axis[(axis+1)%3],localDerivative.Axis[(axis+2)%3]),
            A::Multiply(freeDerivative.Axis[(axis+2)%3],localDerivative.Axis[(axis+1)%3]));
        if(!A::Valid(cross))return ELocalSample::Unavailable;
        regular|=cross.Lower>0||cross.Upper<0;
    }
    if(!regular)return ELocalSample::Unavailable;
    return InspectFixedPointSample(point,query,hit,actualDelta,physicalRadius,budget,output,reason,normalPolicy);
}
}

#endif
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
namespace Speed::PreparedExecution
{
namespace NativeSolidDetail
{bool SameQueryBits(const Analytic::FWorldQuery&,const Analytic::FWorldQuery&) noexcept;}
namespace BoundedLegacyWheelProfile
{
TAutoConsoleVariable<int32> Enabled(TEXT("p.SkyLeague.Stadium.BoundedLegacyWheelProbe"),0,
    TEXT("PRIVATE:0 OFF,1 StrictAllCandidateHits,2 LegacyFirstHitApprox. Bounded spatial proposals, never certified exact first TOI. Global miss/ambiguity use exact."));
std::atomic<uint64> Attempts{0},Accepted{0},Fallback{0},BudgetPreflightRefused{0},WorkFailures{0},NumericFailures{0},LocalRefused{0},WorkUsed{0},Ticks{0};
std::atomic<uint64> LocalReasons[2][uint8(BoundedLegacyWheelDetail::ELocalRefusal::Count)]{};
void Publish()
{
    UE_LOG(LogTemp,Display,TEXT("[BoundedLegacyWheel] Schema=BoundedLegacyWheel/v1 DefaultOFF=1 ExactTOI=0 SpatiallyBoundedOnly=1 Enabled=%d Attempts=%llu Accepted=%llu ExactFallback=%llu BudgetPreflightRefused=%llu WorkFailures=%llu NumericFailures=%llu LocalRefused=%llu WorkUsed=%llu Seconds=%.9f"),
        Enabled.GetValueOnAnyThread(),Attempts.load(),Accepted.load(),Fallback.load(),BudgetPreflightRefused.load(),WorkFailures.load(),NumericFailures.load(),LocalRefused.load(),WorkUsed.load(),FPlatformTime::ToSeconds64(Ticks.load()));
    const TCHAR* names[]={TEXT("Input"),TEXT("Partition"),TEXT("Owner"),TEXT("OriginalFact"),TEXT("OriginalUV"),TEXT("OriginalEvaluate"),TEXT("Regularity"),TEXT("NativeTime"),TEXT("Approach"),TEXT("PositionSlack"),TEXT("NormalCell"),TEXT("ReferenceNormal"),TEXT("Binding"),TEXT("Conversion")};
    static_assert(UE_ARRAY_COUNT(names)==uint8(BoundedLegacyWheelDetail::ELocalRefusal::Count));
    for(int32 kind=0;kind<2;++kind)for(int32 reason=0;reason<UE_ARRAY_COUNT(names);++reason)
        if(LocalReasons[kind][reason].load())UE_LOG(LogTemp,Display,TEXT("[BoundedLegacyWheelLocalRefused] ProfileOnly=1 Kind=%s Reason=%s Count=%llu"),kind?TEXT("Curve"):TEXT("Plane"),names[reason],LocalReasons[kind][reason].load());
}
}
bool BoundedLegacyWheelRuntimeEnabled()
{
    static const bool registered=[](){FCoreDelegates::OnPreExit.AddStatic(&BoundedLegacyWheelProfile::Publish);return true;}();
    (void)registered;return BoundedLegacyWheelRuntimeMode()!=0;
}
int32 BoundedLegacyWheelRuntimeMode()
{const int32 mode=BoundedLegacyWheelProfile::Enabled.GetValueOnAnyThread();return mode==1||mode==2?mode:0;}
bool FBoundedApproxFixedWheelProbeValues::Matches(const FPreparedWorldSnapshotPtr& snapshot,
    const Analytic::FWorldQuery& query,const FNativeSolidTraceValues& trace,double delta,double radius,
    uint64 serial,const FPreparedCanonicalStaticQueryService* producer)const
{
    const auto numeric=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    return snapshot&&snapshot==Snapshot&&Producer&&Producer.Get()==producer&&Producer->MatchesPreparedSnapshot(snapshot)&&
        (bool(Partition)!=bool(Inventory))&&
        (Partition?Partition->GetScene()==snapshot->GetGeometrySelection()->GetSelectedUnion():
            Inventory->GetScene()==snapshot->GetGeometrySelection()->GetSelectedUnion()&&Inventory->HasCurrentNumericControls())&&
        numeric.Status==Analytic::EAnalyticNumericStatus::Complete&&numeric.ControlFingerprint()==NumericControl&&
        NativeSolidDetail::SameQueryBits(Query,query)&&Trace.Same(trace)&&
        AnalyticCertified::Bits(delta)==AnalyticCertified::Bits(Delta)&&
        AnalyticCertified::Bits(radius)==AnalyticCertified::Bits(PhysicalRadius)&&serial==QuerySerial&&
        Hit.bHit&&!Hit.bStartPenetrating&&AnalyticCertified::Valid(LocalPositionError)&&
        OriginalProvider>=0&&BoundedLegacyWheelRuntimeMode()==ApproximationMode&&ApproximationMode!=0&&
        BoundedLegacyRadialNormal::Policy()==NormalPolicy&&
        AnalyticCertified::Finite(RepresentationErrorCm)&&RepresentationErrorCm>=0&&RepresentationErrorCm<=.001;
}
bool FBoundedApproxFixedWheelProbeValues::MatchesConsumedContact(const SHitResult& actual)const
{
    const auto numeric=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    return BoundedLegacyWheelRuntimeMode()==ApproximationMode&&ApproximationMode!=0&&
        BoundedLegacyRadialNormal::Policy()==NormalPolicy&&
        numeric.Status==Analytic::EAnalyticNumericStatus::Complete&&numeric.ControlFingerprint()==NumericControl&&
        SameNativeAnalyticContactValues(ExpectedNativeValues,actual);
}
}
#endif

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
namespace Speed::PreparedExecution
{
namespace InitCertifiedLegacyWheelControl
{
TAutoConsoleVariable<int32> Enabled(TEXT("p.SkyLeague.Stadium.InitCertifiedLegacyWheelResponse"),0,
    TEXT("PRIVATE OFF: Legacy curve hit on init-certified .001cm representation; no local original point/normal/TOI certificate. Requires Bound2, Frontend0. Misses and unsupported queries use the existing fallback."));
std::atomic<uint64> Attempts{0},Accepted{0},Refused{0},MemoHits{0},Ticks{0};
enum class EConversion:uint8 {Unavailable,Ready,Work,Numeric};
EConversion Convert(Analytic::FWorldHit& hit,double representationError,double delta,
    const FPreparedSourceContactBinding& binding,Analytic::Certified::FDistanceBudget& budget,SHitResult& expected)
{
    namespace A=::AnalyticCertified;namespace M=Analytic::Certified::FixedWheelProbeMath;expected={};
    const auto before=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(before.Status!=Analytic::EAnalyticNumericStatus::Complete)return EConversion::Numeric;
    if(!budget.Consume())return EConversion::Work;
    if(!hit.bHit||hit.bStartPenetrating||hit.PenetrationDepth!=0||hit.SurfaceFeatureKind!=EContactFeatureKind::Face||
        !M::NormalOrZero(hit.Time)||!(hit.Time>0&&hit.Time<1)||!M::NormalOrZero(delta)||!(delta>0)||
        !A::Finite(representationError)||representationError<0||representationError>.001)return EConversion::Unavailable;
    const float seconds=static_cast<float>(hit.Time*delta);const uint32 bits=M::FloatBits(seconds);
    if((bits&0x80000000u)||bits>=0x7f800000u||(bits&&bits<0x00800000u))return EConversion::Unavailable;
    auto proposed=hit;
    proposed.GeometricErrorBoundCm=FMath::Max(proposed.GeometricErrorBoundCm,representationError);proposed.bSurfaceNormalMayVary=true;
    SHitResult converted;
    if(!BuildNativeAnalyticContactValues(proposed,seconds,0,binding,converted))return EConversion::Unavailable;
    converted.FaceIndex=int32(proposed.PrimitiveId&0x7fffffffull);converted.ImpactNormal=QuantizeUnitNormal(converted.ImpactNormal);
    if(!M::ValidVector(converted.ImpactNormal)||converted.ImpactNormal.IsZero())return EConversion::Unavailable;
    const auto after=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(after.Status!=Analytic::EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=before.ControlFingerprint())return EConversion::Numeric;
    if(!budget.Consume())return EConversion::Work;
    hit=MoveTemp(proposed);expected=MoveTemp(converted);return EConversion::Ready;
}
void Publish()
{
    UE_LOG(LogTemp,Display,TEXT("[InitCertifiedLegacyWheelResponse] Scope=ProgramPrefixAtPreExit NoPASS=1 RepresentationOnly=1 ExactTOI=0 OriginalNormalBound=0 Enabled=%d Attempts=%llu Accepted=%llu Refused=%llu MemoHits=%llu Ticks=%llu SecondsPerCycle=%.17g"),
        int32(InitCertifiedLegacyWheelRuntimeEnabled()),Attempts.load(),Accepted.load(),Refused.load(),MemoHits.load(),Ticks.load(),FPlatformTime::GetSecondsPerCycle64());
}
}
bool InitCertifiedLegacyWheelRuntimeEnabled()
{
    static const bool registered=[](){FCoreDelegates::OnPreExit.AddStatic(&InitCertifiedLegacyWheelControl::Publish);return true;}();(void)registered;
    return InitCertifiedLegacyWheelControl::Enabled.GetValueOnAnyThread()==1&&BoundedLegacyWheelRuntimeMode()==2&&!BoundedLegacyWheelFrontendEnabled();
}
bool FInitCertifiedLegacyRepresentation::MatchesOwnedGeometry(int32 provider,
    const Analytic::FWorldHit& hit,int32 facet)const
{
    return Geometry&&Provider==provider&&provider>=0&&
        AnalyticCertified::Finite(ErrorCm)&&ErrorCm>=0&&ErrorCm<=.001&&Geometry->SectionParameters.Num()>=2&&
        facet>=0&&facet<Geometry->SectionParameters.Num()-1&&
        hit.SourceId==Geometry->SourceId&&hit.SurfaceId==Geometry->SurfaceId&&hit.FeatureId==Geometry->FeatureId&&
        hit.CanonicalGroupId==Geometry->CanonicalGroupId&&hit.MaterialId==Geometry->MaterialId&&
        hit.PrimitiveId==Analytic::CombineStableIds(Geometry->PrimitiveId,uint64(facet+1));
}
bool FInitCertifiedLegacyRepresentation::Matches(const FPreparedWorldSnapshotPtr& snapshot,int32 provider,
    const Analytic::FWorldHit& hit,int32 facet)const
{
    const auto numeric=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    return snapshot&&snapshot==Snapshot&&numeric.Status==Analytic::EAnalyticNumericStatus::Complete&&
        numeric.ControlFingerprint()==NumericControl&&MatchesOwnedGeometry(provider,hit,facet);
}
bool FInitCertifiedLegacyWheelProbeValues::Matches(const FPreparedWorldSnapshotPtr& snapshot,
    const Analytic::FWorldQuery& query,const FNativeSolidTraceValues& trace,double delta,double radius,
    uint64 serial,const FPreparedCanonicalStaticQueryService* producer)const
{
    const auto numeric=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    return InitCertifiedLegacyWheelRuntimeEnabled()&&snapshot&&snapshot==Snapshot&&Producer&&Producer.Get()==producer&&
        Producer->MatchesPreparedSnapshot(snapshot)&&snapshot->GetGeometrySelection()&&Partition&&
        Partition->GetScene()==snapshot->GetGeometrySelection()->GetSelectedUnion()&&Representation&&
        Representation->Matches(snapshot,OriginalProvider,Hit,OriginalFacet)&&serial&&serial==QuerySerial&&
        numeric.Status==Analytic::EAnalyticNumericStatus::Complete&&numeric.ControlFingerprint()==NumericControl&&
        NativeSolidDetail::SameQueryBits(Query,query)&&Trace.Same(trace)&&
        AnalyticCertified::Bits(delta)==AnalyticCertified::Bits(Delta)&&AnalyticCertified::Bits(radius)==AnalyticCertified::Bits(PhysicalRadius)&&
        Hit.bHit&&!Hit.bStartPenetrating&&Hit.PenetrationDepth==0;
}
bool FInitCertifiedLegacyWheelProbeValues::MatchesConsumedContact(const SHitResult& actual)const
{
    const auto numeric=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    return InitCertifiedLegacyWheelRuntimeEnabled()&&numeric.Status==Analytic::EAnalyticNumericStatus::Complete&&
        numeric.ControlFingerprint()==NumericControl&&SameNativeAnalyticContactValues(ExpectedNativeValues,actual);
}
}

#endif

namespace Speed::PreparedExecution
{
// Existing IAS-local implementation; no unexported function crosses the DLL.

namespace
{
namespace C=::Speed::Analytic::Certified;
namespace A=::AnalyticCertified;
namespace M=C::FixedWheelProbeMath;
using I=A::Interval;
// The actual fixed trace is a linear polynomial, including a stationary
// segment. Construct its degree and coefficients together through the same
// checked API used by the retained-reference interval operations.
bool SelectFixedSphereTraceCandidates(C::FIndexedIntervalUnionPtr scene,
    const FVector3d& start,const FVector3d& end,double radius,
    C::FDistanceBudget& budget,C::FReferenceCandidateSetPtr& output)
{
    ::StadiumSphereProfile70::FScope selection70(::StadiumSphereProfile70::RadiusSelection);
    const uint64 selectionUsed70=budget.Used;
    ON_SCOPE_EXIT{selection70.Budget(selectionUsed70,budget.Used);};
    output.Reset();C::FIntervalBezierCenterPath path;C::FReferenceSelectionWork selectionWork;
    const bool selected70=C::CenterDetail::Linear(start,end,path)&&
        C::FReferenceCandidateSet::Select(scene,path,radius,budget,output,selection70.Active?&selectionWork:nullptr,
            C::PlanarFamilyTimeShortcutControl::Enabled(),::HistoricalQueryCache94::CompactPartitionEnabled());
    if(selection70.Active)::StadiumSphereProfile70::ObserveSelection(selectionWork,selected70);
    if(selected70&&output)selection70.Partition(output->GetCandidateIndices().Num(),output->GetExcludedIndices().Num());
    return selected70;
}
thread_local int32 GFixedProbeCoverageInspection=0;
bool SameDouble(double a,double b){return A::Bits(a)==A::Bits(b);}
bool Policy(const Analytic::FWorldQuery& q,const Analytic::Prepared::FOwner& owner)
{
    if((q.RequiredSourceId&&q.RequiredSourceId!=owner.SourceId)||
        (q.RequiredSurfaceId&&q.RequiredSurfaceId!=owner.SurfaceId)||
        (q.RequiredCanonicalGroupId&&q.RequiredCanonicalGroupId!=owner.CanonicalGroupId)||
        !owner.QueryEnabled||(owner.CompactOptIn&&!q.bIncludeCompactPatches)||owner.ObjectType>=64)return false;
    return q.TraceChannel<64&&(owner.BlockingChannels&(uint64(1)<<q.TraceChannel))&&
        (q.BlockingObjectTypes&(uint64(1)<<owner.ObjectType));
}
bool TracePolicy(const FPreparedWorldSnapshotPtr& snapshot,const Analytic::FWorldQuery& q,
    const FNativeSolidTraceValues& trace,C::FDistanceBudget& budget)
{
    if(q.bObjectQuery||trace.Responses.Num()!=64||trace.TraceChannel!=q.TraceChannel)return false;
    FCollisionQueryParams expected(NAME_None,false);
    expected.bReturnFaceIndex=true;expected.bReturnPhysicalMaterial=true;
    if(trace.TraceComplex!=expected.bTraceComplex||trace.FindInitialOverlaps!=expected.bFindInitialOverlaps||
        trace.ReturnFaceIndex!=expected.bReturnFaceIndex||trace.ReturnPhysicalMaterial!=expected.bReturnPhysicalMaterial||
        trace.IgnoreBlocks!=expected.bIgnoreBlocks||trace.IgnoreTouches!=expected.bIgnoreTouches||
        trace.SkipNarrowPhase!=expected.bSkipNarrowPhase||trace.TraceIntoSubComponents!=expected.bTraceIntoSubComponents||
        trace.ReplaceHitWithSubComponents!=expected.bReplaceHitWithSubComponents||
        trace.Mobility!=uint8(expected.MobilityType)||trace.IgnoreMask!=uint64(expected.IgnoreMask))return false;
    uint64 blocking=0;
    for(int32 i=0;i<64;++i)
    {
        if(!budget.Consume()||trace.Responses[i]>uint8(ECR_Block))return false;
        if(trace.Responses[i]==uint8(ECR_Block))blocking|=uint64(1)<<i;
    }
    if(blocking!=q.BlockingObjectTypes)return false;
    if(C::PlanarFamilyTimeShortcutControl::RuntimeEnabled())
        return snapshot->AllowsNativeIgnoredIds(trace.IgnoredSourceObjects,trace.IgnoredComponents,budget);
    for(const auto& binding:snapshot->GetAllContactBindings())
    {
        if(!budget.Consume()||!binding.SourceObjectId||!binding.SourceOwnerObjectId||
            !binding.ContactObjectId||!binding.ContactOwnerObjectId)return false;
        for(uint32 id:{binding.SourceObjectId,binding.SourceOwnerObjectId,binding.ContactObjectId,binding.ContactOwnerObjectId})
            if(trace.IgnoredSourceObjects.Contains(id)||trace.IgnoredComponents.Contains(id))return false;
    }
    if(snapshot->GetExternalLandscape())
        for(const auto& plane:snapshot->GetExternalLandscape()->GetPlanes())for(const auto& member:plane.Members)
            if(trace.IgnoredSourceObjects.Contains(member.ObjectId)||trace.IgnoredSourceObjects.Contains(member.OwnerObjectId)||
                trace.IgnoredComponents.Contains(member.ObjectId)||trace.IgnoredComponents.Contains(member.OwnerObjectId))return false;
    return true;
}

// Enclose every position of the original straight probe up to the certified
// upper time, including the full radius and existing finite-domain shell.
bool FixedSpherePrefixHull(const Analytic::FWorldQuery& q,double upper,C::FIntervalPoint& out)
{
    if(!A::Finite(upper)||upper<0||upper>1)return false;
    const I time{0,upper,true};
    const auto extent=A::Add(I::Exact(q.Radius),I::Exact(q.DomainTolerance));
    if(!A::Valid(extent)||extent.Lower<0)return false;
    const I shell{-extent.Upper,extent.Upper,true};
    for(int32 axis=0;axis<3;++axis)
    {
        const auto start=I::Exact(q.Start[axis]);
        const auto displacement=A::Subtract(I::Exact(q.End[axis]),start);
        out.Axis[axis]=A::Add(A::Add(start,A::Multiply(displacement,time)),shell);
    }
    return C::Detail::ValidPoint(out);
}

bool ReadFixedSphereLiveInputs(const FPreparedWorldSnapshotPtr& snapshot,const Analytic::FWorldQuery& q,
    const FNativeSolidTraceValues& trace,double delta,double physicalRadius,uint64 serial,
    C::FDistanceBudget& budget,I& slack,C::FIntervalPoint& hull,bool& externalSeparated,FString& reason)
{
    slack={};hull={};externalSeparated=false;const auto fail=[&](const TCHAR* text){reason=text;return false;};
    if(!snapshot||!snapshot->GetGeometrySelection()||!snapshot->GetGeometrySelection()->HasValidAliases()||
        snapshot->GetGeometrySelection()->GetModel()!=EPreparedGeometryModel::RetainedReference||
        !snapshot->GetFilters()||!serial||!M::NormalOrZero(delta)||(A::Bits(delta)&A::Sign)||
        !M::NormalOrZero(physicalRadius)||!(physicalRadius>0)||
        q.Shape!=Analytic::EQueryShape::Sphere||!M::NormalOrZero(q.Radius)||!(q.Radius>0)||
        !q.bAuthorityOnly||!q.bApplyCollisionFilter||q.bExcludeAuthorityEligible||q.bObjectQuery||q.TraceChannel>=64||
        !FMath::IsFinite(q.DomainTolerance)||q.DomainTolerance<0||
        !FMath::IsFinite(q.InitialOverlapTolerance)||q.InitialOverlapTolerance<0||
        q.ReferenceNormal.ContainsNaN()||!FMath::IsFinite(q.MinimumReferenceNormalDot)||
        q.MinimumReferenceNormalDot < -1||q.MinimumReferenceNormalDot>1)return fail(TEXT("fixed_sphere_input"));
    bool filter=false;
    for(const auto& row:snapshot->GetFilters()->GetRecords())
    {
        if(!budget.Consume())return fail(TEXT("fixed_sphere_filter_budget"));
        filter|=row.TraceChannel==q.TraceChannel&&(q.BlockingObjectTypes&~row.BlockingObjectTypes)==0;
    }
    if(!filter||!TracePolicy(snapshot,q,trace,budget))return fail(TEXT("fixed_sphere_actual_trace_filter"));
    slack=A::Subtract(I::Exact(q.Radius),I::Exact(physicalRadius));
    if(!A::Valid(slack)||!(slack.Lower>0))return fail(TEXT("fixed_sphere_captured_radius_slack"));
    if(!C::BuildFixedWorldQueryVolumeBounds(q,hull))return fail(TEXT("fixed_sphere_full_trace_envelope"));
    for(int32 d=0;d<3;++d)hull.Axis[d]=A::Add(hull.Axis[d],I{-q.DomainTolerance,q.DomainTolerance,true});
    if(!C::Detail::ValidPoint(hull))return fail(TEXT("fixed_sphere_full_trace_envelope"));
    const auto& landscape=snapshot->GetExternalLandscape();
    externalSeparated=!landscape||landscape->StrictlySeparated(q,hull);
    return true;
}

// The entry has mathematical values only. No Trace, QuerySerial, owner/body,
// GT/native ticket, one-use affine plan, budget pointer, or launch/PASS state.
struct FCompleteGeometryProof94
{
    C::FTensorSphereContactEnclosure Contact;
    TArray<C::FIntervalPoint> RegularNormals;
    M::FBoundedFixedProbeSample Time;
    C::FScalarSphereContact Scalar;
    M::FUniqueQuantizedNormal Normal;
    I PositionError;
    Analytic::FWorldHit Hit;
};
void SanitizeContactAccounting94(C::FTensorSphereContactEnclosure& c)
{
    c.Failure=C::ETensorSphereContactFailure::None;c.TimeFailure=0;
    c.HasCallAccounting=false;c.CallInitialUsed=0;c.TimeWorkUsed=0;c.SpatialVisitedCells=0;
    c.SpatialSplitsU=0;c.SpatialSplitsV=0;c.TemporalWidthRefinements=0;c.ConsumerWidthRefinements=0;
    c.TimeDyadicSite=0;c.TimeDyadicDepth=-1;c.TimeAnchorRefinements=0;c.TimeDyadicA=0;c.TimeDyadicB=0;
    c.HasWidthDiagnostic=false;c.WidthRefinement=-1;c.WidthSurface=INDEX_NONE;c.WidthDepthU=0;c.WidthDepthV=0;
    c.WidthMinU=c.WidthMaxU=c.WidthMinV=c.WidthMaxV=0;c.WidthTimeLower=c.WidthTimeUpper=0;
    c.WidthPointTarget=c.WidthNormalTarget=0;
    c.WidthPointUnion={};c.WidthNormalUnion={};c.WidthLastDu={};c.WidthLastDv={};c.WidthLastGu={};c.WidthLastGv={};
    c.WorkSite=TEXT("historical_query_cache94_materialization");
}
struct FCompleteQueryView94
{
    const FPreparedWorldSnapshotPtr& Snapshot;
    const Analytic::FWorldQuery& Query;
    uint64 Numeric=0,DeltaBits=0,PhysicalRadiusBits=0;
    TConstArrayView<int32> CandidateIndices;
    C::FExcludedProviderView ExcludedIndices;
};
struct FCompleteQueryKey94
{
    FPreparedWorldSnapshotPtr Snapshot;
    Analytic::FWorldQuery Query;
    uint64 Numeric=0,DeltaBits=0,PhysicalRadiusBits=0;
    TArray<int32> CandidateIndices,ExcludedIndices;
    // Only a checked producer's compact partition may set this identity. The
    // complete complement is then uniquely determined by Scene/N/candidates.
    C::FIndexedIntervalUnionPtr ComplementScene;
    int32 ExcludedCount()const{return ComplementScene?ComplementScene->Num()-CandidateIndices.Num():ExcludedIndices.Num();}
    int32 ExcludedAt(int32 ordinal)const
    {
        if(!ComplementScene)return ExcludedIndices[ordinal];
        int32 provider=ordinal;for(int32 i:CandidateIndices){if(i>provider)break;++provider;}return provider;
    }
};
enum class ECompleteCache94:uint8{NotApplicable,Miss,Hit,Failure};
// This routing predicate never turns a failed or cleared inherited proof into a
// cached contact. The producer calls it only AFTER the original guard decision.
constexpr bool AllowsEarlyCache94(C::EInheritedReach91 inherited)
{
    return inherited==C::EInheritedReach91::Invalid||inherited==C::EInheritedReach91::Outside||
        inherited==C::EInheritedReach91::Near;
}
class FHistoricalQueryCache94
{
    struct FEntry
    {
        FCompleteQueryKey94 Key;TSharedPtr<const FCompleteGeometryProof94,ESPMode::ThreadSafe> Geometry;
        // The producer's actual immutable partition/CPs, never a reconstruction
        // from historical index arrays. Extends its lifetime without copying N.
        C::FReferenceCandidateSetPtr Candidates;uint64 Bytes=0;
        // Retains only a successful completed wheel-fallback route domain.
        // Direct producer calls and Unknown coverage values never set this.
        TSharedPtr<const void,ESPMode::ThreadSafe> CompletedWheelFallbackDomain;
    };
    FEntry Entries[64];uint8 Next[16]{};FPreparedWorldSnapshotPtr World;
    static bool Space(const C::FDistanceBudget& b,int64 work){return work>=0&&int64(b.Remaining)-int64(b.ConsumptionFloor)>=work;}
    static bool Spend(C::FDistanceBudget& b,int32 count)
    {
        if(count<0)return false;
        for(int32 i=0;i<count;++i)if(!b.Consume())return false;
        return true;
    }
    static bool SameEarly(const FCompleteQueryView94& now,const FCompleteQueryKey94& old)
    {
        return now.Snapshot==old.Snapshot&&now.Numeric==old.Numeric&&now.DeltaBits==old.DeltaBits&&
            now.PhysicalRadiusBits==old.PhysicalRadiusBits&&NativeSolidDetail::SameQueryBits(now.Query,old.Query);
    }
    static bool Same(const FCompleteQueryView94& now,const FCompleteQueryKey94& old,C::FDistanceBudget& budget,bool& same)
    {
        same=false;
        if(now.Snapshot!=old.Snapshot||now.Numeric!=old.Numeric||now.DeltaBits!=old.DeltaBits||now.PhysicalRadiusBits!=old.PhysicalRadiusBits||
            !NativeSolidDetail::SameQueryBits(now.Query,old.Query)||now.CandidateIndices.Num()!=old.CandidateIndices.Num()||now.ExcludedIndices.Num()!=old.ExcludedCount())return true;
        const auto* compactOwner=now.ExcludedIndices.GetCompactOwner();
        if(compactOwner&&old.ComplementScene&&compactOwner->GetScene()!=old.ComplementScene)return true;
        for(int32 i=0;i<now.CandidateIndices.Num();++i)
        {if(!budget.Consume())return false;::HistoricalQueryCache94::Add(::HistoricalQueryCache94::KeyWork);if(now.CandidateIndices[i]!=old.CandidateIndices[i])return true;}
        if(compactOwner&&old.ComplementScene)
        {
            const uint64 before=budget.Used;
            const bool paid=C::CandidateDetail::ConsumeRepeated(budget,uint64(now.ExcludedIndices.Num()));
            ::HistoricalQueryCache94::Add(::HistoricalQueryCache94::KeyWork,budget.Used-before);
            if(!paid)return false;
        }
        else for(int32 i=0;i<now.ExcludedIndices.Num();++i)
        {if(!budget.Consume())return false;::HistoricalQueryCache94::Add(::HistoricalQueryCache94::KeyWork);if(now.ExcludedIndices[i]!=old.ExcludedAt(i))return true;}
        same=true;return true;
    }
    static uint32 Bucket(const FCompleteQueryView94& v)
    {
        // Historical placement only. Complete equality above decides reuse.
        return HashCombineFast(GetTypeHash(v.Query.Start),GetTypeHash(v.Query.End))&15u;
    }
    void ClearEntry(FEntry& entry)
    {if(entry.Geometry){::HistoricalQueryCache94::RemoveBytes(entry.Bytes);entry=FEntry{};}}
public:
    ~FHistoricalQueryCache94(){for(auto& e:Entries)ClearEntry(e);}
    void BindSnapshot(const FPreparedWorldSnapshotPtr& snapshot)
    {
        if(World==snapshot)return;
        for(auto& e:Entries)ClearEntry(e);for(auto& way:Next)way=0;
        World=snapshot;::HistoricalQueryCache94::Add(::HistoricalQueryCache94::SnapshotClears);
    }
    static bool SameHitBits(const Analytic::FWorldHit& a,const Analytic::FWorldHit& b)
    {
        return a.bHit==b.bHit&&a.bStartPenetrating==b.bStartPenetrating&&a.bSurfaceNormalMayVary==b.bSurfaceNormalMayVary&&
            SameDouble(a.Time,b.Time)&&SameDouble(a.PenetrationDepth,b.PenetrationDepth)&&
            SameDouble(a.GeometricErrorBoundCm,b.GeometricErrorBoundCm)&&
            SameDouble(a.AdditionalResidualAgreementAllowanceCm,b.AdditionalResidualAgreementAllowanceCm)&&
            M::SameBits(a.Location,b.Location)&&M::SameBits(a.Point,b.Point)&&M::SameBits(a.QueryPoint,b.QueryPoint)&&M::SameBits(a.Normal,b.Normal)&&
            a.QueryFeatureKind==b.QueryFeatureKind&&a.SurfaceFeatureKind==b.SurfaceFeatureKind&&
            a.QueryFeatureIndex==b.QueryFeatureIndex&&a.SurfaceFeatureIndex==b.SurfaceFeatureIndex&&
            a.SourceId==b.SourceId&&a.SurfaceId==b.SurfaceId&&a.FeatureId==b.FeatureId&&
            a.PrimitiveId==b.PrimitiveId&&a.CanonicalGroupId==b.CanonicalGroupId&&a.MaterialId==b.MaterialId;
    }
    bool PeekCompletedWheelFallback(const FCompleteQueryView94& v,const TSharedPtr<const void,ESPMode::ThreadSafe>& domain) const
    {
        if(!domain||World!=v.Snapshot)return false;
        const uint32 first=Bucket(v)*4;
        for(uint32 way=0;way<4;++way)
        {
            const auto& e=Entries[first+way];
            if(e.Geometry&&e.Candidates&&e.CompletedWheelFallbackDomain==domain&&SameEarly(v,e.Key))return true;
        }
        return false;
    }
    bool RecordCompletedWheelFallback(const FCompleteQueryView94& v,const TSharedPtr<const void,ESPMode::ThreadSafe>& domain,
        const Analytic::FWorldHit& delivered)
    {
        if(!domain||World!=v.Snapshot||!delivered.bHit)return false;
        const uint32 first=Bucket(v)*4;
        for(uint32 way=0;way<4;++way)
        {
            auto& e=Entries[first+way];
            if(e.Geometry&&e.Candidates&&SameEarly(v,e.Key)&&SameHitBits(e.Geometry->Hit,delivered))
            {e.CompletedWheelFallbackDomain=domain;return true;}
        }
        return false;
    }
    static bool Bounds(const FCompleteQueryView94& v)
    {return v.CandidateIndices.Num()>0&&v.CandidateIndices.Num()<=16&&v.ExcludedIndices.Num()>=0&&int64(v.CandidateIndices.Num())+v.ExcludedIndices.Num()<=4096&&
        (!v.ExcludedIndices.IsCompact()||v.ExcludedIndices.SharesIncludedStorage(v.CandidateIndices));}
    static constexpr uint64 MaximumRetainedCandidateBytes=512ull*1024;
    static bool CanRetainCandidates(const FCompleteQueryView94& v,const C::FReferenceCandidateSetPtr& p)
    {
        if(!p||!Bounds(v)||!p->GetScene()||v.Query.Shape!=Analytic::EQueryShape::Sphere||
            p->GetCandidateIndices().Num()!=v.CandidateIndices.Num()||
            p->GetExcludedIndices().Num()!=v.ExcludedIndices.Num()||
            p->GetCandidateIndices().GetData()!=v.CandidateIndices.GetData()||
            !p->GetExcludedIndices().SameStorage(v.ExcludedIndices)||
            !SameDouble(p->GetRadiusForFixedSource(),v.Query.Radius))return false;
        // Null snapshot is usable by the existing pure-table tests only. The
        // real producer already rejected it before any cache or input work.
        if(v.Snapshot&&(!v.Snapshot->GetGeometrySelection()||
            p->GetScene()!=v.Snapshot->GetGeometrySelection()->GetSelectedUnion()))return false;
        const auto& path=p->GetPathForFixedSource();
        if(path.Degree!=1||path.ControlPoints.Num()!=2)return false;
        for(int32 k=0;k<2;++k)for(int32 d=0;d<3;++d)
        {
            const auto& a=path.ControlPoints[k].Axis[d];const double x=k?v.Query.End[d]:v.Query.Start[d];
            if(!a.Known||!SameDouble(a.Lower,x)||!SameDouble(a.Upper,x))return false;
        }
        return true;
    }
    static ECompleteCache94 PrepareEarlyRetention(const FCompleteQueryView94& v,
        const C::FReferenceCandidateSetPtr& candidates,C::FDistanceBudget& budget,uint64& bytes,
        uint64 maximumBytes=MaximumRetainedCandidateBytes)
    {
        bytes=0;
        if(!candidates||!Space(budget,3ll+candidates->GetSurfacesForFixedSource().Num()))return ECompleteCache94::NotApplicable;
        if(!budget.Consume())return ECompleteCache94::Failure;
        if(!CanRetainCandidates(v,candidates))return ECompleteCache94::NotApplicable;
        // Count actual allocated capacities, including path and copied CP nets,
        // before the entry can retain them. This is an attributed upper bound:
        // another result/entry may share the same const candidate object.
        uint64 measured=0;
        if(!candidates->AccountOwnedQueryStorageBytes(budget,measured))return ECompleteCache94::Failure;
        if(measured>maximumBytes)return ECompleteCache94::NotApplicable;
        if(!budget.Consume())return ECompleteCache94::Failure; // future pointer publication
        bytes=measured;return ECompleteCache94::Hit;
    }
    ECompleteCache94 FindEarly(const FCompleteQueryView94& v,C::FDistanceBudget& budget,
        TSharedPtr<const FCompleteGeometryProof94,ESPMode::ThreadSafe>& out,C::FReferenceCandidateSetPtr& candidates)
    {
        out.Reset();candidates.Reset();
        // Bounded setup + all four fixed-size key compares + fresh candidate
        // policy/accuracy + maximal existing materialization. No prior debit is
        // restored. Known inapplicability is decided before the first Consume.
        if(!Space(budget,1+4+16+2+129))
        {::HistoricalQueryCache94::Add(::HistoricalQueryCache94::EarlyNotApplicable);return ECompleteCache94::NotApplicable;}
        ::HistoricalQueryCache94::FActive active;
        if(!budget.Consume())return ECompleteCache94::Failure;
        ::HistoricalQueryCache94::Add(::HistoricalQueryCache94::EarlyLookups);
        ::HistoricalQueryCache94::Add(::HistoricalQueryCache94::KeyWork);
        const uint32 first=Bucket(v)*4;
        for(uint32 way=0;way<4;++way)
        {
            if(!budget.Consume())return ECompleteCache94::Failure;
            ::HistoricalQueryCache94::Add(::HistoricalQueryCache94::KeyWork);
            const auto& e=Entries[first+way];
            if(!e.Geometry||!e.Candidates||!SameEarly(v,e.Key))continue;
            out=e.Geometry;candidates=e.Candidates;
            ::HistoricalQueryCache94::Add(::HistoricalQueryCache94::EarlyHits);return ECompleteCache94::Hit;
        }
        ::HistoricalQueryCache94::Add(::HistoricalQueryCache94::EarlyMisses);return ECompleteCache94::Miss;
    }
    ECompleteCache94 Find(const FCompleteQueryView94& v,C::FDistanceBudget& budget,TSharedPtr<const FCompleteGeometryProof94,ESPMode::ThreadSafe>& out)
    {
        out.Reset();const int64 parts=int64(v.CandidateIndices.Num())+v.ExcludedIndices.Num();
        // Known not-applicable is decided BEFORE any Consume. Reserve the
        // bounded four-way compare plus maximal geometry materialization.
        if(!Bounds(v)||!Space(budget,1+4*parts+129))
        {::HistoricalQueryCache94::Add(::HistoricalQueryCache94::NotApplicable);return ECompleteCache94::NotApplicable;}
        ::HistoricalQueryCache94::FActive active;
        if(!budget.Consume())return ECompleteCache94::Failure;
        ::HistoricalQueryCache94::Add(::HistoricalQueryCache94::Lookups);::HistoricalQueryCache94::Add(::HistoricalQueryCache94::KeyWork);
        const uint32 first=Bucket(v)*4;
        for(uint32 way=0;way<4;++way)
        {
            const auto& e=Entries[first+way];if(!e.Geometry)continue;bool same=false;
            if(!Same(v,e.Key,budget,same))return ECompleteCache94::Failure;
            if(same){out=e.Geometry;::HistoricalQueryCache94::Add(::HistoricalQueryCache94::Hits);return ECompleteCache94::Hit;}
        }
        ::HistoricalQueryCache94::Add(::HistoricalQueryCache94::Misses);return ECompleteCache94::Miss;
    }
    static bool ChargeMaterialization(const FCompleteGeometryProof94& p,C::FDistanceBudget& budget)
    {
        const int64 count=1ll+p.Contact.Leaves.Num()+p.RegularNormals.Num();
        if(count<1||count>129||!Spend(budget,int32(count)))return false;
        ::HistoricalQueryCache94::Add(::HistoricalQueryCache94::CopyWork,uint64(count));return true;
    }
    static ECompleteCache94 PrepareStore(const FCompleteQueryView94& v,const C::FTensorSphereContactEnclosure& contact,
        TConstArrayView<C::FIntervalPoint> normals,C::FDistanceBudget& budget)
    {
        const int64 count=1ll+v.CandidateIndices.Num()+v.ExcludedIndices.Num()+contact.Leaves.Num()+normals.Num();
        if(!Bounds(v)||contact.Status!=C::ETensorSphereContactStatus::FirstContactEnclosed||contact.Leaves.IsEmpty()||
            contact.Leaves.Num()>64||normals.Num()!=contact.Leaves.Num()||!Space(budget,count))
        {::HistoricalQueryCache94::Add(::HistoricalQueryCache94::NotStored);return ECompleteCache94::NotApplicable;}
        if(count>MAX_int32||!(v.ExcludedIndices.IsCompact()?C::CandidateDetail::ConsumeRepeated(budget,uint64(count)):
            Spend(budget,int32(count))))return ECompleteCache94::Failure;
        ::HistoricalQueryCache94::Add(::HistoricalQueryCache94::CopyWork,uint64(count));return ECompleteCache94::Hit;
    }
    void StoreReserved(const FCompleteQueryView94& v,TSharedPtr<const FCompleteGeometryProof94,ESPMode::ThreadSafe> proof,
        const C::FReferenceCandidateSetPtr& candidates={},uint64 retainedCandidateBytes=0)
    {
        ::HistoricalQueryCache94::FActive active;check(proof);
        const uint32 bucket=Bucket(v),slot=bucket*4+Next[bucket];auto& e=Entries[slot];
        if(e.Geometry)::HistoricalQueryCache94::Add(::HistoricalQueryCache94::Evictions);
        ClearEntry(e);e.Key.Snapshot=v.Snapshot;e.Key.Query=v.Query;e.Key.Numeric=v.Numeric;e.Key.DeltaBits=v.DeltaBits;e.Key.PhysicalRadiusBits=v.PhysicalRadiusBits;
        e.Key.CandidateIndices.Append(v.CandidateIndices.GetData(),v.CandidateIndices.Num());
        if(const auto* compactOwner=v.ExcludedIndices.GetCompactOwner())
        {
            check(v.ExcludedIndices.SharesIncludedStorage(v.CandidateIndices));
            e.Key.ComplementScene=compactOwner->GetScene();
        }
        else
        {
            const auto explicitView=v.ExcludedIndices.GetExplicitView();
            e.Key.ExcludedIndices.Append(explicitView.GetData(),explicitView.Num());
        }
        e.Geometry=MoveTemp(proof);e.Bytes=sizeof(FEntry)+sizeof(FCompleteGeometryProof94)+e.Key.CandidateIndices.GetAllocatedSize()+e.Key.ExcludedIndices.GetAllocatedSize()+e.Geometry->Contact.Leaves.GetAllocatedSize()+e.Geometry->RegularNormals.GetAllocatedSize();
        if(retainedCandidateBytes>=sizeof(C::FReferenceCandidateSet)&&retainedCandidateBytes<=MaximumRetainedCandidateBytes&&
            retainedCandidateBytes<=MAX_uint64-e.Bytes&&CanRetainCandidates(v,candidates))
        {e.Candidates=candidates;e.Bytes+=retainedCandidateBytes;}
        ::HistoricalQueryCache94::AddBytes(e.Bytes);::HistoricalQueryCache94::Add(::HistoricalQueryCache94::Inserts);Next[bucket]=(Next[bucket]+1)&3u;
    }
};
FHistoricalQueryCache94& ThreadHistoricalQueryCache94(){static thread_local FHistoricalQueryCache94 cache;return cache;}

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
// Independent cache of explicitly APPROXIMATE geometry. No exact94 entry,
// trace, serial, native result, scope/window, account or callback is retained.
namespace BoundedWheelMemo
{
TAutoConsoleVariable<int32> Enabled(TEXT("p.SkyLeague.Stadium.BoundedLegacyWheelMemo"),0,
    TEXT("PRIVATE OFF: retain bounded approximate proposals/local samples only; repeat all live guards and consumed-value construction."));
std::atomic<uint64> Lookups{0},Hits{0},Misses{0},Stores{0},Evictions{0},NotStored{0},
    Materialized{0},Peeks{0},Preferred{0},Recorded{0},ReplayedWork{0},LiveBytes{0},PeakBytes{0};
bool Active(){return Enabled.GetValueOnAnyThread()==1&&BoundedLegacyWheelRuntimeMode()!=0;}
void Publish()
{
    UE_LOG(LogTemp,Display,TEXT("[BoundedLegacyWheelMemo] DefaultOFF=1 ApproximateOnly=1 Enabled=%d Capacity=64 Ways=4 Lookups=%llu Hits=%llu Misses=%llu Stores=%llu Evictions=%llu NotStored=%llu Materialized=%llu Peeks=%llu Preferred=%llu Recorded=%llu ReplayedWork=%llu LiveBytes=%llu PeakBytes=%llu"),
        Enabled.GetValueOnAnyThread(),Lookups.load(),Hits.load(),Misses.load(),Stores.load(),Evictions.load(),NotStored.load(),Materialized.load(),Peeks.load(),Preferred.load(),Recorded.load(),ReplayedWork.load(),LiveBytes.load(),PeakBytes.load());
}
struct FGeometry
{
    FBoundedApproxWheelProposal Proposal;
    BoundedLegacyWheelDetail::FLocalSample Sample;
    C::FReferenceCandidateSetPtr OriginalCandidates;
    uint64 SelectionWork=0,ProposalWork=0,SampleWork=0;
    bool InventoryOnly=false;
    // A distinct cache variant carries no FLocalSample authority.
    bool InitOnly=false;int32 InitPolicy=0;
};
using FGeometryPtr=TSharedPtr<const FGeometry,ESPMode::ThreadSafe>;
class FCache
{
    struct FEntry
    {
        FCompleteQueryKey94 Key;
        TSharedPtr<const void,ESPMode::ThreadSafe> Producer,CompletedDomain;
        // Identity handshake ONLY: never read a historic probe value or reuse
        // its authority. An eviction/reinsertion clears this weak identity.
        TWeakPtr<const void,ESPMode::ThreadSafe> LastMaterialization;
        FGeometryPtr Geometry;uint64 Bytes=0;int32 Mode=0;
    };
    FEntry Entries[64];uint8 Next[16]{};FPreparedWorldSnapshotPtr World;
    TSharedPtr<const void,ESPMode::ThreadSafe> WorldProducer;
    static uint32 Bucket(const FCompleteQueryView94& v)
    {return HashCombineFast(GetTypeHash(v.Query.Start),GetTypeHash(v.Query.End))&15u;}
    static bool Same(const FCompleteQueryView94& v,const TSharedPtr<const void,ESPMode::ThreadSafe>& producer,int32 mode,const FEntry& e)
    {
        return e.Geometry&&e.Producer==producer&&mode==e.Mode&&mode!=0&&
            e.Geometry->InitPolicy==int32(InitCertifiedLegacyWheelRuntimeEnabled())&&
            (e.Geometry->InitOnly||e.Geometry->Sample.NormalPolicy==BoundedLegacyRadialNormal::Policy())&&v.Snapshot==e.Key.Snapshot&&
            v.Numeric==e.Key.Numeric&&v.DeltaBits==e.Key.DeltaBits&&v.PhysicalRadiusBits==e.Key.PhysicalRadiusBits&&
            NativeSolidDetail::SameQueryBits(v.Query,e.Key.Query);
    }
    void Clear(FEntry& e)
    {if(e.Geometry){LiveBytes.fetch_sub(e.Bytes);e=FEntry{};}}
public:
    static constexpr uint64 MaximumOwnedBytes=512ull*1024;
    static bool Space(const C::FDistanceBudget& budget,uint64 work)
    {
        return budget.ConsumptionFloor>=0&&budget.Remaining>=budget.ConsumptionFloor&&
            uint64(int64(budget.Remaining)-budget.ConsumptionFloor)>=work&&MAX_uint64-budget.Used>=work;
    }
    ~FCache(){for(auto& e:Entries)Clear(e);}
    void Bind(const FPreparedWorldSnapshotPtr& snapshot,const TSharedPtr<const void,ESPMode::ThreadSafe>& producer)
    {
        if(World!=snapshot||WorldProducer!=producer)
        {for(auto& e:Entries)Clear(e);for(auto& n:Next)n=0;World=snapshot;WorldProducer=producer;}
    }
    ECompleteCache94 Find(const FCompleteQueryView94& v,const TSharedPtr<const void,ESPMode::ThreadSafe>& producer,int32 mode,
        C::FDistanceBudget& budget,FGeometryPtr& result,bool inventoryOnly=false)
    {
        result.Reset();if(!producer||mode<1||mode>2||!Space(budget,5))return ECompleteCache94::NotApplicable;
        Bind(v.Snapshot,producer);++Lookups;if(!budget.Consume())return ECompleteCache94::Failure;
        const uint32 first=Bucket(v)*4;
        for(uint32 way=0;way<4;++way)
        {
            if(!budget.Consume())return ECompleteCache94::Failure;
            const auto& e=Entries[first+way];if(!Same(v,producer,mode,e)||e.Geometry->InventoryOnly!=inventoryOnly)continue;
            // Replay the original selector's logical debit before sharing its
            // immutable partition. Partial failure retains exactly that debit.
            const uint64 before=budget.Used;
            const bool paid=C::CandidateDetail::ConsumeRepeated(budget,e.Geometry->SelectionWork);
            ReplayedWork.fetch_add(budget.Used-before);
            if(!paid)return ECompleteCache94::Failure;
            result=e.Geometry;++Hits;return ECompleteCache94::Hit;
        }
        ++Misses;return ECompleteCache94::Miss;
    }
    bool Store(const FCompleteQueryView94& v,const TSharedPtr<const void,ESPMode::ThreadSafe>& producer,int32 mode,
        const FGeometry& geometry,C::FDistanceBudget& budget,FGeometryPtr& stored,uint64 maximumBytes=MaximumOwnedBytes)
    {
        stored.Reset();const auto& original=geometry.OriginalCandidates;
        const auto& inflated=geometry.Proposal.CompleteInflatedPartition;
        const auto& inventory=geometry.Proposal.CompleteInflatedInventory;
        if(!producer||mode<1||mode>2||
            (geometry.InventoryOnly?bool(original)||bool(inflated)||!inventory||geometry.SelectionWork!=0:!original||!inflated||bool(inventory))||
            !geometry.Proposal.Hit.bHit||
            geometry.Proposal.Hit.bStartPenetrating||(geometry.InitPolicy!=int32(InitCertifiedLegacyWheelRuntimeEnabled()))||
            (geometry.InitOnly?(!geometry.Proposal.InitRepresentation||geometry.Proposal.Plane||geometry.InventoryOnly||geometry.SampleWork!=0):
                (geometry.Sample.NumericControl!=v.Numeric||geometry.Sample.NormalPolicy!=BoundedLegacyRadialNormal::Policy()||!A::Valid(geometry.Sample.PositionError)))||geometry.SelectionWork>MAX_int32||
            geometry.ProposalWork>200000||geometry.SampleWork>64||
            geometry.ProposalWork+geometry.SampleWork>200000)
        {++NotStored;return true;}
        uint64 originalBytes=0,inflatedBytes=0;
        if(geometry.InventoryOnly)
        {
            const auto& path=inventory->GetPathForFixedSource();
            const auto inflatedRadius=A::Add(I::Exact(v.Query.Radius),I::Exact(.001));
            if(!inventory->GetScene()||!inventory->HasCurrentNumericControls()||
                !A::Valid(inflatedRadius)||!SameDouble(inventory->GetRadiusForFixedSource(),inflatedRadius.Upper)||
                inventory->GetCandidateIndices().IsEmpty()||inventory->GetCandidateIndices().Num()>16||
                inventory->GetScene()->Num()>4096||path.Degree!=1||path.ControlPoints.Num()!=2||
                (v.Snapshot&&(!v.Snapshot->GetGeometrySelection()||inventory->GetScene()!=v.Snapshot->GetGeometrySelection()->GetSelectedUnion())))
            {++NotStored;return true;}
            for(int32 endpoint=0;endpoint<2;++endpoint)for(int32 axis=0;axis<3;++axis)
            {
                const auto& a=path.ControlPoints[endpoint].Axis[axis];const double x=endpoint?v.Query.End[axis]:v.Query.Start[axis];
                if(!a.Known||!SameDouble(a.Lower,x)||!SameDouble(a.Upper,x)){++NotStored;return true;}
            }
            if(!Space(budget,2)){++NotStored;return true;}
            if(!inventory->AccountOwnedQueryStorageBytes(budget,inflatedBytes))return false;
        }
        else
        {
        const FCompleteQueryView94 originalView{v.Snapshot,v.Query,v.Numeric,v.DeltaBits,v.PhysicalRadiusBits,
            original->GetCandidateIndices(),original->GetExcludedIndices()};
        auto inflatedQuery=v.Query;inflatedQuery.Radius=inflated->GetRadiusForFixedSource();
        const FCompleteQueryView94 inflatedView{v.Snapshot,inflatedQuery,v.Numeric,v.DeltaBits,v.PhysicalRadiusBits,
            inflated->GetCandidateIndices(),inflated->GetExcludedIndices()};
        // Actual producer partitions only; full source/path/radius and finite
        // retained-storage bounds are checked before their first retention.
        if(!FHistoricalQueryCache94::CanRetainCandidates(originalView,original)||
            !FHistoricalQueryCache94::CanRetainCandidates(inflatedView,inflated)||original->GetScene()!=inflated->GetScene())
        {++NotStored;return true;}
        const uint64 work=3ull+original->GetSurfacesForFixedSource().Num()+inflated->GetSurfacesForFixedSource().Num();
        if(!Space(budget,work)){++NotStored;return true;}
        if(!original->AccountOwnedQueryStorageBytes(budget,originalBytes)||
            !inflated->AccountOwnedQueryStorageBytes(budget,inflatedBytes))return false;
        }
        const uint64 representationBytes=geometry.Proposal.InitRepresentation?geometry.Proposal.InitRepresentation->GetOwnedGeometryBytes():0;
        if((geometry.InitOnly&&representationBytes==0)||representationBytes>maximumBytes){++NotStored;return true;}
        const uint64 fixed=sizeof(FEntry)+sizeof(FGeometry)+representationBytes;
        if(originalBytes>maximumBytes||inflatedBytes>maximumBytes-originalBytes||
            fixed>maximumBytes-originalBytes-inflatedBytes){++NotStored;return true;}
        if(!budget.Consume())return false;
        Bind(v.Snapshot,producer);const uint32 bucket=Bucket(v),slot=bucket*4+Next[bucket];auto& e=Entries[slot];
        if(e.Geometry)++Evictions;Clear(e);
        e.Key.Snapshot=v.Snapshot;e.Key.Query=v.Query;e.Key.Numeric=v.Numeric;
        e.Key.DeltaBits=v.DeltaBits;e.Key.PhysicalRadiusBits=v.PhysicalRadiusBits;e.Mode=mode;e.Producer=producer;
        e.Geometry=MakeShared<FGeometry,ESPMode::ThreadSafe>(geometry);e.Bytes=fixed+originalBytes+inflatedBytes;
        const uint64 live=LiveBytes.fetch_add(e.Bytes)+e.Bytes;uint64 peak=PeakBytes.load();
        while(live>peak&&!PeakBytes.compare_exchange_weak(peak,live)){}
        Next[bucket]=(Next[bucket]+1)&3u;stored=e.Geometry;++Stores;return true;
    }
    bool MarkMaterialized(const FCompleteQueryView94& v,const TSharedPtr<const void,ESPMode::ThreadSafe>& producer,int32 mode,
        const FGeometryPtr& geometry,const TSharedPtr<const void,ESPMode::ThreadSafe>& identity)
    {
        if(!identity||!geometry||World!=v.Snapshot)return false;
        const uint32 first=Bucket(v)*4;
        for(uint32 way=0;way<4;++way)
        {auto& e=Entries[first+way];if(Same(v,producer,mode,e)&&e.Geometry==geometry){e.LastMaterialization=identity;++Materialized;return true;}}
        return false;
    }
    bool Record(const FCompleteQueryView94& v,const TSharedPtr<const void,ESPMode::ThreadSafe>& producer,int32 mode,
        const TSharedPtr<const void,ESPMode::ThreadSafe>& domain,const TSharedPtr<const void,ESPMode::ThreadSafe>& identity,
        const Analytic::FWorldHit& delivered)
    {
        if(!identity||!domain||World!=v.Snapshot)return false;
        const uint32 first=Bucket(v)*4;
        for(uint32 way=0;way<4;++way)
        {auto& e=Entries[first+way];if(Same(v,producer,mode,e)&&e.LastMaterialization.Pin()==identity&&
            FHistoricalQueryCache94::SameHitBits(e.Geometry->Proposal.Hit,delivered))
            {e.CompletedDomain=domain;++Recorded;return true;}}
        return false;
    }
    bool Peek(const FCompleteQueryView94& v,const TSharedPtr<const void,ESPMode::ThreadSafe>& producer,int32 mode,
        const TSharedPtr<const void,ESPMode::ThreadSafe>& domain)const
    {
        if(!domain||World!=v.Snapshot)return false;
        const uint32 first=Bucket(v)*4;
        for(uint32 way=0;way<4;++way)
        {const auto& e=Entries[first+way];if(Same(v,producer,mode,e)&&!e.Geometry->InventoryOnly&&e.CompletedDomain==domain)return true;}
        return false;
    }
};
FCache& ThreadCache()
{
    static const bool registered=[](){FCoreDelegates::OnPreExit.AddStatic(&Publish);return true;}();
    (void)registered;static thread_local FCache cache;return cache;
}
}
#endif


bool SameOwner(const Analytic::Prepared::FOwner& a,const Analytic::Prepared::FOwner& b)
{
    return a.SourceId==b.SourceId&&a.SurfaceId==b.SurfaceId&&a.FeatureId==b.FeatureId&&
        a.PrimitiveId==b.PrimitiveId&&a.CanonicalGroupId==b.CanonicalGroupId&&a.MaterialId==b.MaterialId;
}
}

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
// One closure for both source inventories. A proposal remains unsealed until
// all live policy/local-original geometry/native conversion checks succeed.
struct FBoundedWheelValueFactory
{
    enum class ESource:uint8 {Unavailable,Ready,Work,Numeric};
    static ESource Source(const FBoundedApproxWheelProposal& proposal,C::FIndexedIntervalUnionPtr scene,
        const Analytic::FWorldQuery& q,C::FDistanceBudget& budget,TConstArrayView<int32>& indices)
    {
        indices={};
        if(bool(proposal.CompleteInflatedPartition)==bool(proposal.CompleteInflatedInventory))return ESource::Unavailable;
            if(!budget.Consume())return ESource::Work;
            const auto inflatedRadius=A::Add(I::Exact(q.Radius),I::Exact(.001));
            const bool inventoryOnly=bool(proposal.CompleteInflatedInventory);
            if(inventoryOnly&&!proposal.CompleteInflatedInventory->HasCurrentNumericControls())return ESource::Numeric;
            const auto& selectedScene=inventoryOnly?proposal.CompleteInflatedInventory->GetScene():proposal.CompleteInflatedPartition->GetScene();
            const auto& selectedPath=inventoryOnly?proposal.CompleteInflatedInventory->GetPathForFixedSource():proposal.CompleteInflatedPartition->GetPathForFixedSource();
            const double selectedRadius=inventoryOnly?proposal.CompleteInflatedInventory->GetRadiusForFixedSource():proposal.CompleteInflatedPartition->GetRadiusForFixedSource();
            const auto selectedIndices=inventoryOnly?proposal.CompleteInflatedInventory->GetCandidateIndices():proposal.CompleteInflatedPartition->GetCandidateIndices();
            if(selectedScene!=scene||!A::Valid(inflatedRadius)||A::Bits(selectedRadius)!=A::Bits(inflatedRadius.Upper)||
                selectedPath.Degree!=1||selectedPath.ControlPoints.Num()!=2)return ESource::Unavailable;
            for(int32 endpoint=0;endpoint<2;++endpoint)for(int32 axis=0;axis<3;++axis)
            {
                const auto expected=I::Exact(endpoint?q.End[axis]:q.Start[axis]);
                const auto& actual=selectedPath.ControlPoints[endpoint].Axis[axis];
                if(actual.Known!=expected.Known||A::Bits(actual.Lower)!=A::Bits(expected.Lower)||A::Bits(actual.Upper)!=A::Bits(expected.Upper))
                    return ESource::Unavailable;
            }
            indices=selectedIndices;return ESource::Ready;
    }

    static BoundedLegacyWheelDetail::ELocalSample CloseInit(const FPreparedWorldSnapshotPtr& snapshot,
        const Analytic::FWorldQuery& q,const FNativeSolidTraceValues& trace,double delta,double physicalRadius,uint64 serial,
        uint64 numericControl,const TSharedPtr<const FPreparedCanonicalStaticQueryService,ESPMode::ThreadSafe>& service,
        FBoundedApproxWheelProposal& proposal,C::FDistanceBudget& budget,FInitCertifiedLegacyWheelProbeValuesPtr& output)
    {
        namespace B=BoundedLegacyWheelDetail;output.Reset();
        if(!InitCertifiedLegacyWheelRuntimeEnabled()||!snapshot||!snapshot->GetGeometrySelection()||!snapshot->GetNominalScene()||
            !service||!service->MatchesPreparedSnapshot(snapshot)||proposal.Plane||!proposal.InitRepresentation||
            proposal.CompleteInflatedInventory||!proposal.CompleteInflatedPartition||
            !proposal.InitRepresentation->Matches(snapshot,proposal.OriginalProvider,proposal.Hit,proposal.OriginalFacet))return B::ELocalSample::Unavailable;
        const auto scene=snapshot->GetGeometrySelection()->GetSelectedUnion();
        TConstArrayView<int32> indices;const auto source=Source(proposal,scene,q,budget,indices);
        if(source==ESource::Work)return B::ELocalSample::Work;
        if(source==ESource::Numeric)return B::ELocalSample::Numeric;
        if(source!=ESource::Ready)return B::ELocalSample::Unavailable;
        const auto owners=snapshot->GetNominalScene()->GetOwners();bool found=false;
        for(int32 provider:indices)
        {
            if(!budget.Consume())return B::ELocalSample::Work;
            if(!owners.IsValidIndex(provider)||!Policy(q,owners[provider]))return B::ELocalSample::Unavailable;
            found|=provider==proposal.OriginalProvider;
        }
        if(!found||!owners.IsValidIndex(proposal.OriginalProvider))return B::ELocalSample::Unavailable;
        const auto& owner=owners[proposal.OriginalProvider];const auto& h=proposal.Hit;
        if(owner.FromPlane||owner.SourceId!=h.SourceId||owner.SurfaceId!=h.SurfaceId||owner.FeatureId!=h.FeatureId||
            owner.CanonicalGroupId!=h.CanonicalGroupId||owner.MaterialId!=h.MaterialId||
            !h.bHit||h.bStartPenetrating||h.PenetrationDepth!=0||h.SurfaceFeatureKind!=EContactFeatureKind::Face||
            !M::ValidVector(h.Location)||!M::ValidVector(h.Point)||!M::ValidVector(h.QueryPoint)||!M::ValidVector(h.Normal)||
            !M::NormalOrZero(h.Time)||!(h.Time>0&&h.Time<1)||!M::NormalOrZero(delta)||!(delta>0))return B::ELocalSample::Unavailable;
        bool ambiguous=false;const auto* binding=snapshot->FindUniqueContactBinding(h.SourceId,ambiguous);
        if(!binding||ambiguous)return B::ELocalSample::Unavailable;
        SHitResult expected;using K=InitCertifiedLegacyWheelControl::EConversion;
        const auto converted=InitCertifiedLegacyWheelControl::Convert(proposal.Hit,proposal.InitRepresentation->GetRepresentationErrorCm(),delta,*binding,budget,expected);
        if(converted==K::Work)return B::ELocalSample::Work;
        if(converted==K::Numeric)return B::ELocalSample::Numeric;
        if(converted!=K::Ready)return B::ELocalSample::Unavailable;
        TSharedPtr<FInitCertifiedLegacyWheelProbeValues,ESPMode::ThreadSafe> value(new FInitCertifiedLegacyWheelProbeValues);
        value->Snapshot=snapshot;value->Producer=service;value->Partition=proposal.CompleteInflatedPartition;
        value->Representation=proposal.InitRepresentation;value->Query=q;value->Trace=trace;value->Delta=delta;value->PhysicalRadius=physicalRadius;
        value->QuerySerial=serial;value->NumericControl=numericControl;value->OriginalProvider=proposal.OriginalProvider;value->OriginalFacet=proposal.OriginalFacet;
        value->Hit=proposal.Hit;value->ExpectedNativeValues=MoveTemp(expected);
        if(!value->Matches(snapshot,q,trace,delta,physicalRadius,serial,service.Get()))return B::ELocalSample::Numeric;
        output=MoveTemp(value);return B::ELocalSample::BoundedLocalSample; // Internal control enum only; output type has no local-sample claim.
    }

    static BoundedLegacyWheelDetail::ELocalSample Close(const FPreparedWorldSnapshotPtr& snapshot,
        const Analytic::FWorldQuery& q,const FNativeSolidTraceValues& trace,double delta,double physicalRadius,uint64 serial,
        uint64 numericControl,int32 approximationMode,const TSharedPtr<const FPreparedCanonicalStaticQueryService,ESPMode::ThreadSafe>& approximateService,
        FBoundedApproxWheelProposal& proposal,C::FDistanceBudget& budget,
        const BoundedLegacyWheelDetail::FLocalSample* cachedSample,uint64 cachedSampleWork,
        BoundedLegacyWheelDetail::FLocalSample& retainedSample,uint64& retainedSampleWork,
        FBoundedApproxFixedWheelProbeValuesPtr& output,BoundedLegacyWheelDetail::ELocalRefusal& localRefusal)
    {
        namespace B=BoundedLegacyWheelDetail;output.Reset();retainedSample={};retainedSampleWork=0;
        const int32 normalPolicy=BoundedLegacyRadialNormal::Policy();
        if(!snapshot||!snapshot->GetGeometrySelection()||!snapshot->GetNominalScene()||!approximateService||
            !approximateService->MatchesPreparedSnapshot(snapshot))return B::ELocalSample::Unavailable;
        const auto scene=snapshot->GetGeometrySelection()->GetSelectedUnion();
        const auto owners=snapshot->GetNominalScene()->GetOwners();
            if(!proposal.Hit.bHit||proposal.Hit.bStartPenetrating||!owners.IsValidIndex(proposal.OriginalProvider)||
                !A::Finite(proposal.RepresentationErrorCm)||proposal.RepresentationErrorCm<0||proposal.RepresentationErrorCm>.001)
                return B::ELocalSample::Unavailable;
            localRefusal=B::ELocalRefusal::Partition;
            TConstArrayView<int32> selectedIndices;const auto source=Source(proposal,scene,q,budget,selectedIndices);
            if(source==ESource::Numeric)return B::ELocalSample::Numeric;
            if(source==ESource::Work)return B::ELocalSample::Work;
            if(source!=ESource::Ready)return B::ELocalSample::Unavailable;
            bool foundProvider=false;
            for(int32 provider:selectedIndices)
            {
                if(!budget.Consume())return B::ELocalSample::Work;
                if(!owners.IsValidIndex(provider)||!Policy(q,owners[provider]))return B::ELocalSample::Unavailable;
                foundProvider|=provider==proposal.OriginalProvider;
            }
            localRefusal=B::ELocalRefusal::Owner;
            const auto& sourceOwner=owners[proposal.OriginalProvider];
            if(!foundProvider||sourceOwner.SourceId!=proposal.Hit.SourceId||sourceOwner.SurfaceId!=proposal.Hit.SurfaceId||
                sourceOwner.FeatureId!=proposal.Hit.FeatureId||sourceOwner.CanonicalGroupId!=proposal.Hit.CanonicalGroupId||
                sourceOwner.MaterialId!=proposal.Hit.MaterialId)return B::ELocalSample::Unavailable;
            B::FLocalSample sample;B::ELocalSample status=B::ELocalSample::Unavailable;
            const uint64 sampleBegan=budget.Used;
            if(cachedSample)
            {
                const uint64 replayBegan=budget.Used;
                const bool paid=C::CandidateDetail::ConsumeRepeated(budget,cachedSampleWork);
                BoundedWheelMemo::ReplayedWork.fetch_add(budget.Used-replayBegan);
                if(!paid)return B::ELocalSample::Work;
                if(cachedSample->NormalPolicy!=normalPolicy)return B::ELocalSample::Unavailable;
                sample=*cachedSample;status=B::ELocalSample::BoundedLocalSample;
            }
            else if(proposal.Plane)
            {
                if(!sourceOwner.FromPlane||proposal.PlaneNormalAxis<0||proposal.PlaneNormalAxis>2||!A::Finite(proposal.PlaneHeight)||
                    proposal.RepresentationErrorCm!=0||sourceOwner.PrimitiveId!=proposal.Hit.PrimitiveId)return B::ELocalSample::Unavailable;
                // The service's prepared plane factory proved complete source
                // membership and revalidated this actual foot. This reconstructs
                // the same mathematical point on that exact axial plane.
                auto originalPoint=C::Detail::Point(proposal.Hit.Point);
                originalPoint.Axis[proposal.PlaneNormalAxis]=I::Exact(proposal.PlaneHeight);
                status=B::InspectFixedPointSample(originalPoint,q,proposal.Hit,delta,physicalRadius,budget,sample,&localRefusal,normalPolicy);
            }
            else
            {
                localRefusal=B::ELocalRefusal::OriginalFact;
                if(sourceOwner.FromPlane||!snapshot->GetReference())return B::ELocalSample::Unavailable;
                const Analytic::Prepared::FReferenceAffineExtrusionDescription* selected=nullptr;
                for(const auto& fact:snapshot->GetReference()->GetExactAffineExtrusions(proposal.OriginalProvider))
                {
                    if(!budget.Consume())return B::ELocalSample::Work;
                    if(fact.Provider==proposal.OriginalProvider&&fact.CorrelationProofIdentity==snapshot->GetReference()->GetExactAffineProofIdentity()&&
                        fact.Product.FreeWorldAxis==proposal.ProductFreeWorldAxis&&fact.Product.FreeAlongU==proposal.ProductFreeAlongU)
                    {if(selected)return B::ELocalSample::Unavailable;selected=&fact;}
                }
                if(!selected)return B::ELocalSample::Unavailable;
                status=B::InspectFixedProductSample(selected->Product,proposal.ProposedOriginalUV,q,proposal.Hit,delta,physicalRadius,budget,sample,&localRefusal,normalPolicy);
            }
            if(status!=B::ELocalSample::BoundedLocalSample)return status;
            if(sample.NumericControl!=numericControl||sample.NormalPolicy!=normalPolicy||
                BoundedLegacyRadialNormal::Policy()!=normalPolicy)return B::ELocalSample::Numeric;
            // Only the separate approximate value changes. Time, point and source
            // IDs stay the original Legacy proposal; this is no first-contact normal.
            proposal.Hit.Normal=sample.StoredNormal;
            retainedSample=sample;retainedSampleWork=budget.Used-sampleBegan;
            if(!budget.Consume())return B::ELocalSample::Work;
            localRefusal=B::ELocalRefusal::Binding;
            bool ambiguous=false;const auto* binding=snapshot->FindUniqueContactBinding(proposal.Hit.SourceId,ambiguous);
            if(!binding||ambiguous)return B::ELocalSample::Unavailable;
            // This field bounds provider representation, NOT first-TOI error.
            proposal.Hit.GeometricErrorBoundCm=FMath::Max(proposal.Hit.GeometricErrorBoundCm,proposal.RepresentationErrorCm);
            proposal.Hit.bSurfaceNormalMayVary=true;
            localRefusal=B::ELocalRefusal::Conversion;
            SHitResult expected;
            if(!BuildNativeAnalyticContactValues(proposal.Hit,sample.NativeSeconds,0,*binding,expected))return B::ELocalSample::Unavailable;
            expected.FaceIndex=int32(proposal.Hit.PrimitiveId&0x7fffffffull);
            expected.ImpactNormal=QuantizeUnitNormal(expected.ImpactNormal);
            TSharedPtr<FBoundedApproxFixedWheelProbeValues,ESPMode::ThreadSafe> value(new FBoundedApproxFixedWheelProbeValues);
            value->Snapshot=snapshot;value->Producer=approximateService;value->Partition=proposal.CompleteInflatedPartition;
            value->Inventory=proposal.CompleteInflatedInventory;
            value->Query=q;value->Trace=trace;value->Delta=delta;value->PhysicalRadius=physicalRadius;
            value->QuerySerial=serial;value->NumericControl=sample.NumericControl;value->Hit=proposal.Hit;
            value->ExpectedNativeValues=MoveTemp(expected);value->RepresentationErrorCm=proposal.RepresentationErrorCm;
            value->LocalPositionError=sample.PositionError;value->OriginalProvider=proposal.OriginalProvider;
            value->OriginalFacet=proposal.OriginalFacet;value->OriginalSampleUV=proposal.ProposedOriginalUV;
            value->ApproximationMode=approximationMode;value->NormalPolicy=normalPolicy;
            if(!value->Matches(snapshot,q,trace,delta,physicalRadius,serial,approximateService.Get()))return B::ELocalSample::Numeric;
            output=MoveTemp(value);return B::ELocalSample::BoundedLocalSample;
    }
};
#endif

// IAS-local declarations are repeated in PreparedNativeWheelProbe.inl only;
// this optimization adds no exported producer ABI and no producer argument.
bool HistoricalCacheBeforeCoverageEnabled()
{return ::HistoricalQueryCache94::CVarBeforeCoverage.GetValueOnAnyThread()==1&&::HistoricalQueryCache94::EarlyEnabled();}
static bool PeekCompletedWheelFallbackChecked(FHistoricalQueryCache94& cache,const FCompleteQueryView94& view,
    const TSharedPtr<const void,ESPMode::ThreadSafe>& domain)
{
    const auto numeric=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=Analytic::EAnalyticNumericStatus::Complete)return false;
    const FCompleteQueryView94 current{view.Snapshot,view.Query,numeric.ControlFingerprint(),view.DeltaBits,view.PhysicalRadiusBits,{}, {}};
    return cache.PeekCompletedWheelFallback(current,domain);
}
bool PreferHistoricalCacheBeforeCoverage(const FPreparedWorldSnapshotPtr& snapshot,const Analytic::FWorldQuery& query,
    double delta,double physicalRadius,const TSharedPtr<const void,ESPMode::ThreadSafe>& domain)
{
    if(!HistoricalCacheBeforeCoverageEnabled()||!snapshot||!domain)return false;
    const bool observe=::StadiumSphereProfile70::Enabled();
    if(observe)::HistoricalQueryCache94::Add(::HistoricalQueryCache94::CoveragePeeks);
    const FCompleteQueryView94 view{snapshot,query,0,A::Bits(delta),A::Bits(physicalRadius),{}, {}};
    const bool found=PeekCompletedWheelFallbackChecked(ThreadHistoricalQueryCache94(),view,domain);
    if(found&&observe)::HistoricalQueryCache94::Add(::HistoricalQueryCache94::CoveragePreferred);
    return found;
}
void RecordHistoricalCompletedWheelFallback(const FPreparedWorldSnapshotPtr& snapshot,const Analytic::FWorldQuery& query,
    double delta,double physicalRadius,const TSharedPtr<const void,ESPMode::ThreadSafe>& domain,const Analytic::FWorldHit& hit)
{
    if(!HistoricalCacheBeforeCoverageEnabled()||!snapshot||!domain)return;
    const auto numeric=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=Analytic::EAnalyticNumericStatus::Complete)return;
    const FCompleteQueryView94 view{snapshot,query,numeric.ControlFingerprint(),A::Bits(delta),A::Bits(physicalRadius),{}, {}};
    if(ThreadHistoricalQueryCache94().RecordCompletedWheelFallback(view,domain,hit)&&::StadiumSphereProfile70::Enabled())
        ::HistoricalQueryCache94::Add(::HistoricalQueryCache94::CoverageRecorded);
}

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
bool BoundedLegacyWheelMemoEnabled(){return BoundedWheelMemo::Active();}
bool PreferBoundedLegacyWheelBeforeCoverage(const FPreparedWorldSnapshotPtr& snapshot,const Analytic::FWorldQuery& query,
    double delta,double radius,const TSharedPtr<const void,ESPMode::ThreadSafe>& domain,
    const TSharedPtr<const FPreparedCanonicalStaticQueryService,ESPMode::ThreadSafe>& producer)
{
    if(!BoundedWheelMemo::Active()||!snapshot||!domain||!producer||!producer->MatchesPreparedSnapshot(snapshot))return false;
    const auto numeric=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=Analytic::EAnalyticNumericStatus::Complete)return false;
    ++BoundedWheelMemo::Peeks;
    const FCompleteQueryView94 view{snapshot,query,numeric.ControlFingerprint(),A::Bits(delta),A::Bits(radius),{}, {}};
    const bool found=BoundedWheelMemo::ThreadCache().Peek(view,producer,BoundedLegacyWheelRuntimeMode(),domain);
    if(found)++BoundedWheelMemo::Preferred;return found;
}
void RecordBoundedLegacyWheelCompleted(const FPreparedWorldSnapshotPtr& snapshot,const Analytic::FWorldQuery& query,
    double delta,double radius,const TSharedPtr<const void,ESPMode::ThreadSafe>& domain,
    const TSharedPtr<const FPreparedCanonicalStaticQueryService,ESPMode::ThreadSafe>& producer,
    const FBoundedApproxFixedWheelProbeValuesPtr& value)
{
    if(!BoundedWheelMemo::Active()||!snapshot||!domain||!producer||!value||!producer->MatchesPreparedSnapshot(snapshot))return;
    const auto numeric=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=Analytic::EAnalyticNumericStatus::Complete)return;
    const FCompleteQueryView94 view{snapshot,query,numeric.ControlFingerprint(),A::Bits(delta),A::Bits(radius),{}, {}};
    BoundedWheelMemo::ThreadCache().Record(view,producer,BoundedLegacyWheelRuntimeMode(),domain,value,value->GetHit());
}
void RecordInitCertifiedLegacyWheelCompleted(const FPreparedWorldSnapshotPtr& snapshot,const Analytic::FWorldQuery& query,
    double delta,double radius,const TSharedPtr<const void,ESPMode::ThreadSafe>& domain,
    const TSharedPtr<const FPreparedCanonicalStaticQueryService,ESPMode::ThreadSafe>& producer,
    const FInitCertifiedLegacyWheelProbeValuesPtr& value)
{
    if(!BoundedWheelMemo::Active()||!snapshot||!domain||!producer||!value||!producer->MatchesPreparedSnapshot(snapshot))return;
    const auto numeric=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=Analytic::EAnalyticNumericStatus::Complete)return;
    const FCompleteQueryView94 view{snapshot,query,numeric.ControlFingerprint(),A::Bits(delta),A::Bits(radius),{}, {}};
    BoundedWheelMemo::ThreadCache().Record(view,producer,BoundedLegacyWheelRuntimeMode(),domain,value,value->GetHit());
}
#endif

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
namespace BoundedWheelFrontend
{
TAutoConsoleVariable<int32> Enabled(TEXT("p.SkyLeague.Stadium.BoundedLegacyWheelFrontend"),0,
    TEXT("PRIVATE OFF: try the bounded approximate Legacy frontend before canonical coverage/partitions. Unknown/global miss use the existing exact path on the same remaining account."));
std::atomic<uint64> Calls{0},Accepted{0},Unavailable{0},PreflightRefused{0},LocalRefused{0},
    WorkFailure{0},NumericFailure{0},MemoHits{0},Used{0},Ticks{0};
void Publish()
{
    UE_LOG(LogTemp,Display,TEXT("[BoundedLegacyWheelFrontend] Scope=ProgramPrefixAtPreExit NoPASS=1 DefaultOFF=1 ExactTOI=0 InventoryOnly=1 Enabled=%d Calls=%llu Accepted=%llu Unavailable=%llu PreflightRefused=%llu LocalRefused=%llu WorkFailure=%llu NumericFailure=%llu MemoHits=%llu Used=%llu Seconds=%.9f"),
        Enabled.GetValueOnAnyThread(),Calls.load(),Accepted.load(),Unavailable.load(),PreflightRefused.load(),LocalRefused.load(),WorkFailure.load(),NumericFailure.load(),MemoHits.load(),Used.load(),FPlatformTime::ToSeconds64(Ticks.load()));
}
}
bool BoundedLegacyWheelFrontendEnabled()
{
    static const bool registered=[](){FCoreDelegates::OnPreExit.AddStatic(&BoundedWheelFrontend::Publish);return true;}();
    (void)registered;return BoundedWheelFrontend::Enabled.GetValueOnAnyThread()==1&&BoundedLegacyWheelRuntimeMode()!=0;
}
EBoundedApproxWheelAttempt EvaluateBoundedLegacyWheelFrontend(FPreparedWorldSnapshotPtr snapshot,
    const Analytic::FWorldQuery& q,const FNativeSolidTraceValues& trace,double delta,double physicalRadius,uint64 serial,
    C::FDistanceBudget& budget,const TSharedPtr<const FPreparedCanonicalStaticQueryService,ESPMode::ThreadSafe>& service,
    FBoundedApproxFixedWheelProbeValuesPtr& output,FString& reason)
{
    namespace F=BoundedWheelFrontend;namespace B=BoundedLegacyWheelDetail;
    using E=EBoundedApproxWheelAttempt;output.Reset();reason.Reset();
    if(!BoundedLegacyWheelFrontendEnabled())return E::NotApplicable;
    ++F::Calls;const uint64 began=FPlatformTime::Cycles64(),initialUsed=budget.Used;
    ON_SCOPE_EXIT{F::Ticks.fetch_add(FPlatformTime::Cycles64()-began);if(budget.Used>=initialUsed)F::Used.fetch_add(budget.Used-initialUsed);};
    const auto decline=[&](const TCHAR* why){reason=why;++F::Unavailable;return E::NotApplicable;};
    const auto workFailure=[&](const TCHAR* why){reason=why;++F::WorkFailure;return E::WorkFailure;};
    const auto numericFailure=[&](const TCHAR* why){reason=why;++F::NumericFailure;return E::NumericFailure;};
    if(!snapshot||!snapshot->GetGeometrySelection()||!snapshot->GetGeometrySelection()->GetSelectedUnion()||
        !snapshot->GetFilters()||!service)return decline(TEXT("bounded_frontend_source_unavailable"));
    if(!service->MatchesPreparedSnapshot(snapshot))return numericFailure(TEXT("bounded_frontend_service_changed"));
    const auto numeric=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=Analytic::EAnalyticNumericStatus::Complete)return numericFailure(TEXT("bounded_frontend_numeric"));
    const auto scene=snapshot->GetGeometrySelection()->GetSelectedUnion();
    // Worst fixed live inventory plus the candidate/sample suffix. This is an
    // optional admission only; the original full query budget is never raised.
    const uint64 minimum=2ull*uint64(scene->Num())+uint64(snapshot->GetAllContactBindings().Num())+
        uint64(snapshot->GetFilters()->GetRecords().Num())+256;
    const auto admission=B::CanBeginOptional(budget,minimum,200000);
    if(admission==B::EOptionalAdmission::InvalidAccount)return workFailure(TEXT("bounded_frontend_account"));
    if(admission!=B::EOptionalAdmission::Ready){++F::PreflightRefused;return decline(TEXT("bounded_frontend_preflight"));}
    const uint64 available=FMath::Min(uint64(int64(budget.Remaining)-budget.ConsumptionFloor),MAX_uint64-budget.Used);
    const int32 allowance=int32(FMath::Min(uint64(200000),available));
    C::FScopedDistanceBudgetFloor cap(budget,budget.Remaining-allowance);
    if(!cap.IsValid())return workFailure(TEXT("bounded_frontend_cap"));
    const int32 capFloor=budget.ConsumptionFloor,initialRemaining=budget.Remaining;
    const uint64 scopedUsed=budget.Used;const int32 mode=BoundedLegacyWheelRuntimeMode();
    I slack;C::FIntervalPoint hull;bool externalSeparated=false;
    if(!ReadFixedSphereLiveInputs(snapshot,q,trace,delta,physicalRadius,serial,budget,slack,hull,externalSeparated,reason))
    {
        if(budget.Remaining<=budget.ConsumptionFloor||budget.Used==MAX_uint64){++F::WorkFailure;return E::WorkFailure;}
        ++F::Unavailable;return E::NotApplicable;
    }
    if(!externalSeparated){++F::Unavailable;return E::NotApplicable;}
    const FCompleteQueryView94 key{snapshot,q,numeric.ControlFingerprint(),A::Bits(delta),A::Bits(physicalRadius),{}, {}};
    BoundedWheelMemo::FGeometryPtr cached;
    if(BoundedWheelMemo::Active())
    {
        const auto status=BoundedWheelMemo::ThreadCache().Find(key,service,mode,budget,cached,true);
        if(status==ECompleteCache94::Failure)return workFailure(TEXT("bounded_frontend_memo_work"));
        if(cached)++F::MemoHits;
    }
    FBoundedApproxWheelProposal proposal;E attempt;
    if(B::CanBeginOptional(budget,uint64(scene->Num())+192,200000)!=B::EOptionalAdmission::Ready)
        return decline(TEXT("bounded_frontend_suffix_reserve"));
    const uint64 proposalBegan=budget.Used;
    {
        C::FScopedDistanceBudgetFloor sampleReserve(budget,capFloor+scene->Num()+128);
        if(!sampleReserve.IsValid())return workFailure(TEXT("bounded_frontend_sample_reserve"));
        if(cached)
        {
            const uint64 replayBegan=budget.Used;
            const bool paid=C::CandidateDetail::ConsumeRepeated(budget,cached->ProposalWork);
            BoundedWheelMemo::ReplayedWork.fetch_add(budget.Used-replayBegan);
            if(!paid)return workFailure(TEXT("bounded_frontend_memo_proposal_work"));
            proposal=cached->Proposal;attempt=E::BoundedApproxSpatialHit;
        }
        else attempt=service->TryBoundedLegacyWheelInventoryProposal(q,budget,mode,proposal);
    }
    const uint64 proposalWork=budget.Used-proposalBegan;
    if(budget.Used<scopedUsed||budget.Remaining>initialRemaining||budget.ConsumptionFloor!=capFloor||
        budget.Used-scopedUsed!=uint64(int64(initialRemaining)-budget.Remaining))return workFailure(TEXT("bounded_frontend_account_changed"));
    const auto after=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(after.Status!=Analytic::EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=numeric.ControlFingerprint()||
        !BoundedLegacyWheelFrontendEnabled()||BoundedLegacyWheelRuntimeMode()!=mode||!service->MatchesPreparedSnapshot(snapshot))
        return numericFailure(TEXT("bounded_frontend_context_changed"));
    if(attempt==E::WorkFailure)return workFailure(TEXT("bounded_frontend_proposal_work"));
    if(attempt==E::NumericFailure)return numericFailure(TEXT("bounded_frontend_proposal_numeric"));
    if(attempt!=E::BoundedApproxSpatialHit)return decline(TEXT("bounded_frontend_no_proposal"));
    if(proposal.CompleteInflatedPartition||!proposal.CompleteInflatedInventory)return decline(TEXT("bounded_frontend_wrong_source_kind"));
    B::FLocalSample retained;uint64 sampleWork=0;B::ELocalRefusal localReason=B::ELocalRefusal::Input;
    FBoundedApproxFixedWheelProbeValuesPtr value;
    const auto status=FBoundedWheelValueFactory::Close(snapshot,q,trace,delta,physicalRadius,serial,numeric.ControlFingerprint(),mode,service,
        proposal,budget,cached?&cached->Sample:nullptr,cached?cached->SampleWork:0,retained,sampleWork,value,localReason);
    if(status==B::ELocalSample::Work)return workFailure(TEXT("bounded_frontend_local_work"));
    if(status==B::ELocalSample::Numeric)return numericFailure(TEXT("bounded_frontend_local_numeric"));
    if(status!=B::ELocalSample::BoundedLocalSample)
    {
        ++F::LocalRefused;
        if(::StadiumSphereProfile70::Enabled())++BoundedLegacyWheelProfile::LocalReasons[proposal.Plane?0:1][uint8(localReason)];
        return decline(TEXT("bounded_frontend_local_unavailable"));
    }
    if(!BoundedLegacyWheelFrontendEnabled()||BoundedLegacyWheelRuntimeMode()!=mode)
        return numericFailure(TEXT("bounded_frontend_final_mode_changed"));
    if(BoundedWheelMemo::Active())
    {
        if(!cached)
        {
            const BoundedWheelMemo::FGeometry geometry{proposal,retained,{},0,proposalWork,sampleWork,true};
            if(!BoundedWheelMemo::ThreadCache().Store(key,service,mode,geometry,budget,cached))return workFailure(TEXT("bounded_frontend_memo_store"));
        }
        if(cached)BoundedWheelMemo::ThreadCache().MarkMaterialized(key,service,mode,cached,value);
    }
    output=MoveTemp(value);++F::Accepted;return E::BoundedApproxSpatialHit;
}
#endif

bool IsCanonicalFixedProbeCoverageObservation(){return GFixedProbeCoverageInspection>0;}
bool HasCanonicalFixedProbeCoverageForInspection(const FPreparedCanonicalStaticQueryService& service,
    const Analytic::FWorldQuery& q)
{
    ++GFixedProbeCoverageInspection;
    ON_SCOPE_EXIT{--GFixedProbeCoverageInspection;};
    // Ordinary Unknown is false. Numeric/malformed-result exceptions propagate.
    return service.HasAuthorityCoverage(q);
}
} // namespace Speed::PreparedExecution
namespace Speed
{
bool FPreparedCanonicalStaticQueryService::TrySweepNativeFixedProbeWithCoverage(
    const Analytic::FWorldQuery& query,const PreparedExecution::FNativeSolidTraceValues& trace,
    Analytic::FWorldHit& out) const
{
    PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
    out={};Analytic::FWorldHit coveredHit,nativeHit;
    EPreparedStaticQueryResult coveredStatus=EPreparedStaticQueryResult::Unknown;
    Analytic::FAnalyticNumericEvidence beforeCoverage,afterCoverage;
    {
        ++PreparedExecution::GFixedProbeCoverageInspection;
        ON_SCOPE_EXIT{--PreparedExecution::GFixedProbeCoverageInspection;};
        beforeCoverage=Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        coveredStatus=EvaluateSweep(query,coveredHit);
        afterCoverage=Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        if(coveredStatus==EPreparedStaticQueryResult::Unknown)return false;
        // Preserve the old coverage validation BEFORE any trace-policy work.
        coveredHit=ReadResult(coveredStatus,MoveTemp(coveredHit));
    }
    // The diagnostic suppression has ended. A native trace/policy Unknown
    // must still throw through ReadResult, never open the Sphere fallback.
    const auto beforeNative=Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
    const bool sameControl=beforeCoverage.ControlFingerprint()==afterCoverage.ControlFingerprint()&&
        afterCoverage.ControlFingerprint()==beforeNative.ControlFingerprint();
    const auto nativeStatus=sameControl?
        EvaluateNativeFixedProbeFromCovered(query,trace,coveredStatus,coveredHit,nativeHit):
        EvaluateNativeFixedProbe(query,trace,nativeHit); // Old path if allowed FP mode changed.
    Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
    out=ReadResult(nativeStatus,MoveTemp(nativeHit));return true;
}
} // namespace Speed
namespace Speed::PreparedExecution
{

bool FCanonicalFixedSphereWheelProbeValues::Matches(const FPreparedWorldSnapshotPtr& snapshot,
    const Analytic::FWorldQuery& q,const FNativeSolidTraceValues& trace,double delta,double radius,uint64 serial) const
{
    const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(
        Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    return Snapshot&&Snapshot==snapshot&&Snapshot->GetGeometrySelection()&&Snapshot->GetGeometrySelection()->HasValidAliases()&&
        Result!=ECanonicalFixedSphereWheelProbeResult::Unknown&&
        (Result!=ECanonicalFixedSphereWheelProbeResult::CanonicalContact||
            (!Contact.Leaves.IsEmpty()&&RegularLeafChartNormals.Num()==Contact.Leaves.Num()&&M::MatchesBoundedFixedProbeSample(Contact,delta,Time)))&&
        NativeSolidDetail::SameQueryBits(Query,q)&&Trace.Same(trace)&&
        SameDouble(Delta,delta)&&SameDouble(PhysicalRadius,radius)&&QuerySerial==serial&&serial&&
        numeric.ControlFingerprint()==NumericControl&&Candidates&&
        Candidates->GetScene()==snapshot->GetGeometrySelection()->GetSelectedUnion();
}
bool FCanonicalFixedSphereWheelProbeValues::MatchesConsumedContact(const SHitResult& value) const
{
    const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(
        Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(Result!=ECanonicalFixedSphereWheelProbeResult::CanonicalContact||!value.bHit||!value.bBlockingHit||
        value.bStartPenetrating||value.PenetrationDepth!=0||value.FrameTag!=0||
        numeric.ControlFingerprint()!=NumericControl||M::FloatBits(value.TOI)!=Time.NativeSecondsBits||
        !M::SameBits(value.Location,Hit.Location)||!M::SameBits(value.ImpactPoint,Hit.Point)||
        !M::SameBits(value.ContactPointThis,Hit.QueryPoint)||!M::SameBits(value.ImpactNormal,Normal.Quantized)||
        !M::SameBits(value.ImpactNormal.GetSafeNormal(),Normal.ReNormalized)||
        value.SourceId!=Hit.SourceId||value.SurfaceId!=Hit.SurfaceId||value.FeatureId!=Hit.FeatureId||
        value.PrimitiveId!=Hit.PrimitiveId||value.CanonicalGroupId!=Hit.CanonicalGroupId||value.MaterialId!=Hit.MaterialId||
        value.FaceIndex!=int32(Hit.PrimitiveId&0x7fffffffull)||!value.bSurfaceNormalMayVary||
        !FMath::IsFinite(value.GeometricErrorBoundCm)||value.GeometricErrorBoundCm<Scalar.SampleEvaluationErrorCm)return false;
    return true;
}

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
bool EvaluateFixedSphereWheelProbeRoute(FPreparedWorldSnapshotPtr snapshot,const Analytic::FWorldQuery& query,
    const FNativeSolidTraceValues& trace,double delta,double physicalRadius,uint64 serial,
    C::FDistanceBudget& budget,FCanonicalFixedSphereWheelProbeValuesPtr& out,FString& reason,FFixedSphereProbeDiagnostic89 diagnostic,
    const C::FInheritedReachInput91& inheritedReach,
    const TSharedPtr<const FPreparedCanonicalStaticQueryService,ESPMode::ThreadSafe>& approximateService,
    FBoundedApproxFixedWheelProbeValuesPtr& approximateOut,FInitCertifiedLegacyWheelProbeValuesPtr* initialOut)
#else
bool EvaluateCanonicalFixedSphereWheelProbeValues(FPreparedWorldSnapshotPtr snapshot,const Analytic::FWorldQuery& query,
    const FNativeSolidTraceValues& trace,double delta,double physicalRadius,uint64 serial,
    C::FDistanceBudget& budget,FCanonicalFixedSphereWheelProbeValuesPtr& out,FString& reason,FFixedSphereProbeDiagnostic89 diagnostic,
    const C::FInheritedReachInput91& inheritedReach)
#endif
{
    ::StadiumSphereProfile70::FScope producer70(::StadiumSphereProfile70::Producer);
    const uint64 producerUsed70=budget.Used;
    ON_SCOPE_EXIT{producer70.Budget(producerUsed70,budget.Used);};
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    approximateOut.Reset();if(initialOut)initialOut->Reset();
#endif
    out.Reset();reason.Reset();const auto fail=[&](const TCHAR* text){reason=text;return false;};
    const Analytic::FWorldQuery q=query;
    const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(
        Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    I slack;C::FIntervalPoint hull;bool externalSeparated=false;
    if(!ReadFixedSphereLiveInputs(snapshot,q,trace,delta,physicalRadius,serial,budget,slack,hull,externalSeparated,reason))return false;
    const auto geometry=snapshot->GetGeometrySelection();const auto scene=geometry->GetSelectedUnion();
    C::FReferenceCandidateSetPtr candidates;
    bool inheritedClear91=false;C::EInheritedReach91 inheritedStatus91=C::EInheritedReach91::Invalid;
    {
        using E=C::EInheritedReach91;using S=Analytic::EStaticQuerySite;
        const auto route=diagnostic.Site==uint8(S::WheelSuspensionProbe)?C::EInheritedReachRoute91::WheelSuspension:
            diagnostic.Site==uint8(S::WheelEstablishedSupportProbe)?C::EInheritedReachRoute91::WheelSupport:C::EInheritedReachRoute91::Other;
        C::FInheritedReachObservation91 observed91(route,budget);
        auto physical=q;physical.Radius=physicalRadius;C::FIntervalPoint reach;
        auto status=C::BuildInheritedMotionReach91(inheritedReach,q,physical,delta,0.,hull,budget,reach);
        if(status==E::Near)
        {
            C::FIntervalBezierCenterPath actualPath;
            if(!C::CenterDetail::Linear(q.Start,q.End,actualPath))status=E::Unknown;
            else
            {
                const auto empty=C::FReferenceCandidateSet::SelectInheritedEmpty91(scene,actualPath,q.Radius,reach,budget,candidates);
                status=empty==C::EEmptyBounds91::Clear?E::Cleared:empty==C::EEmptyBounds91::Near?E::Near:E::Unknown;
            }
        }
        observed91.Result=status;
        if(status==E::Unknown)return fail(TEXT("inherited_reach91_unknown"));
        inheritedClear91=status==E::Cleared;inheritedStatus91=status;
    }
    // No early lookup can precede the live trace/external/inherited guards.
    // Unknown already returned above; Cleared keeps its original empty route.
    FHistoricalQueryCache94* cache94=nullptr;bool cacheMiss94=false;
    TSharedPtr<const FCompleteGeometryProof94,ESPMode::ThreadSafe> geometry94;
    const bool earlyEnabled94=externalSeparated&&::HistoricalQueryCache94::EarlyEnabled();
    if(earlyEnabled94&&AllowsEarlyCache94(inheritedStatus91))
    {
        cache94=&ThreadHistoricalQueryCache94();cache94->BindSnapshot(snapshot);
        const FCompleteQueryView94 earlyKey94{snapshot,q,numeric.ControlFingerprint(),A::Bits(delta),A::Bits(physicalRadius),{}, {}};
        const auto earlyStatus94=cache94->FindEarly(earlyKey94,budget,geometry94,candidates);
        if(earlyStatus94==ECompleteCache94::Failure)return fail(TEXT("early_historical_query_cache94_lookup_work"));
        if(earlyStatus94==ECompleteCache94::Hit&&(!geometry94||!candidates))return fail(TEXT("early_historical_query_cache94_payload"));
        cacheMiss94=earlyStatus94==ECompleteCache94::Miss;
    }

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    BoundedWheelMemo::FGeometryPtr boundedGeometry;
    uint64 boundedSelectionWork=0;int32 boundedGeometryMode=0;
    if(externalSeparated&&!geometry94&&approximateService&&BoundedWheelMemo::Active()&&AllowsEarlyCache94(inheritedStatus91))
    {
        if(!approximateService->MatchesPreparedSnapshot(snapshot))return fail(TEXT("bounded_wheel_memo_service"));
        const FCompleteQueryView94 memoKey{snapshot,q,numeric.ControlFingerprint(),A::Bits(delta),A::Bits(physicalRadius),{}, {}};
        boundedGeometryMode=BoundedLegacyWheelRuntimeMode();
        const auto memoStatus=BoundedWheelMemo::ThreadCache().Find(memoKey,approximateService,boundedGeometryMode,budget,boundedGeometry);
        if(memoStatus==ECompleteCache94::Failure)return fail(TEXT("bounded_wheel_memo_lookup_work"));
        if(memoStatus==ECompleteCache94::Hit)
        {candidates=boundedGeometry->OriginalCandidates;boundedSelectionWork=boundedGeometry->SelectionWork;}
    }
    const uint64 beforeOriginalSelection=budget.Used;
#endif
    // A miss/inapplicability retains the ORIGINAL selector and budget. A hit
    // shares the producer's immutable candidate object, then repeats all fresh
    // candidate policy/accuracy/serial/trace/materialization checks below.
    if(!inheritedClear91&&!geometry94
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
        &&!boundedGeometry
#endif
        &&(!SelectFixedSphereTraceCandidates(scene,q.Start,q.End,q.Radius,budget,candidates)||!candidates))
        return fail(TEXT("fixed_sphere_candidates"));
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    if(!boundedGeometry)boundedSelectionWork=budget.Used-beforeOriginalSelection;
#endif
    const auto owners=snapshot->GetNominalScene()->GetOwners();
    for(int32 index:candidates->GetCandidateIndices())
        if(!budget.Consume()||!owners.IsValidIndex(index)||!Policy(q,owners[index]))return fail(TEXT("fixed_sphere_all_candidate_policy"));
    TSharedPtr<FCanonicalFixedSphereWheelProbeValues,ESPMode::ThreadSafe> result;
    const auto initializeExactResult=[&]()
    {
        result=TSharedPtr<FCanonicalFixedSphereWheelProbeValues,ESPMode::ThreadSafe>(new FCanonicalFixedSphereWheelProbeValues);
        result->Snapshot=snapshot;result->Query=q;result->Trace=trace;result->Delta=delta;result->PhysicalRadius=physicalRadius;
        result->QuerySerial=serial;result->NumericControl=numeric.ControlFingerprint();result->Envelope=hull;
        result->Candidates=candidates;result->CapturedProbeRadiusSlack=slack;
    };
    if(inheritedClear91)
    {
        if(!externalSeparated)return fail(TEXT("fixed_sphere_external_trace"));
        // Complete empty original-order partition, not a manufactured hit or
        // readiness ticket. Existing Matches/suffix/external guards still run.
        initializeExactResult();
        result->Result=ECanonicalFixedSphereWheelProbeResult::CanonicalSeparated;
        out=result;return true;
    }
    // Search accuracy is technical, derived from captured probe slack and the
    // existing normal resolution; it grants no new acceptance margin.
    double timeGap=0;
    const double pointWidth=slack.Lower*.125,normalWidth=double(1.e-3f)*.125;
    if(!FMath::IsFinite(pointWidth)||!(pointWidth>0)||
        !M::TryFixedProbeSpatialTimeGap(q.Start,q.End,pointWidth,normalWidth,q.Radius,timeGap))return fail(TEXT("fixed_sphere_accuracy"));
    if(!budget.Consume())return fail(TEXT("fixed_sphere_work_exhausted"));
    const int32 perAttempt=FMath::Min(8192,budget.Remaining);
    if(perAttempt<=0)return fail(TEXT("fixed_sphere_work_exhausted"));

    // All live guards above remain fresh. Both lookup positions use exactly
    // this one existing geometry materialization/Matches block.
    const FCompleteQueryView94 key94{snapshot,q,numeric.ControlFingerprint(),A::Bits(delta),A::Bits(physicalRadius),candidates->GetCandidateIndices(),candidates->GetExcludedIndices()};
    if(externalSeparated&&!geometry94&&::HistoricalQueryCache94::Enabled())
    {
        if(!cache94){cache94=&ThreadHistoricalQueryCache94();cache94->BindSnapshot(snapshot);}
        const auto status94=cache94->Find(key94,budget,geometry94);
        if(status94==ECompleteCache94::Failure)return fail(TEXT("historical_query_cache94_lookup_work"));
        cacheMiss94=cacheMiss94||status94==ECompleteCache94::Miss;
    }
    if(geometry94)
    {
        if(geometry94->Contact.Status!=C::ETensorSphereContactStatus::FirstContactEnclosed||
            !A::Valid(geometry94->PositionError)||geometry94->PositionError.Upper>slack.Lower||
            !FHistoricalQueryCache94::ChargeMaterialization(*geometry94,budget))return fail(TEXT("historical_query_cache94_materialization"));
        // Charge every bounded dynamic copy BEFORE allocation. Live binding
        // fields Snapshot/Trace/serial/envelope/candidates/slack stay NEW.
        initializeExactResult();
        result->Contact=geometry94->Contact;SanitizeContactAccounting94(result->Contact);
        result->RegularLeafChartNormals=geometry94->RegularNormals;result->Time=geometry94->Time;
        result->Scalar=geometry94->Scalar;result->Normal=geometry94->Normal;result->ProbePositionError=geometry94->PositionError;result->Hit=geometry94->Hit;
        result->Result=ECanonicalFixedSphereWheelProbeResult::CanonicalContact;
        if(!result->Matches(snapshot,q,trace,delta,physicalRadius,serial))return fail(TEXT("historical_query_cache94_fresh_identity"));
        ::HistoricalQueryCache94::Add(::HistoricalQueryCache94::Materialized);out=MoveTemp(result);return true;
    }

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    if(externalSeparated&&approximateService&&BoundedLegacyWheelRuntimeEnabled()&&
        BoundedLegacyWheelDetail::CanBeginOptional(budget,2ull*uint64(scene->Num())+256,200000)!=BoundedLegacyWheelDetail::EOptionalAdmission::Ready)
    {++BoundedLegacyWheelProfile::BudgetPreflightRefused;++BoundedLegacyWheelProfile::Fallback;}
    if(externalSeparated&&approximateService&&BoundedLegacyWheelRuntimeEnabled()&&
        BoundedLegacyWheelDetail::CanBeginOptional(budget,2ull*uint64(scene->Num())+256,200000)==BoundedLegacyWheelDetail::EOptionalAdmission::Ready)
    {
        namespace B=BoundedLegacyWheelDetail;namespace P=BoundedLegacyWheelProfile;
        ++P::Attempts;const auto optionalStart=FPlatformTime::Cycles64();const uint64 optionalUsed=budget.Used;
        const int32 optionalRemaining=budget.Remaining;
        const uint64 availableOptional=FMath::Min(uint64(int64(budget.Remaining)-budget.ConsumptionFloor),MAX_uint64-budget.Used);
        const int32 optionalAllowance=int32(FMath::Min(uint64(200000),availableOptional));
        C::FScopedDistanceBudgetFloor optionalCap(budget,budget.Remaining-optionalAllowance);
        if(!optionalCap.IsValid())return fail(TEXT("bounded_wheel_local_cap"));
        const int32 optionalFloor=budget.ConsumptionFloor;
        ON_SCOPE_EXIT{P::Ticks.fetch_add(FPlatformTime::Cycles64()-optionalStart);if(budget.Used>=optionalUsed)P::WorkUsed.fetch_add(budget.Used-optionalUsed);};
        if(!approximateService->MatchesPreparedSnapshot(snapshot))return fail(TEXT("bounded_wheel_service_changed"));
        const int32 approximationMode=BoundedLegacyWheelRuntimeMode();
        FBoundedApproxWheelProposal proposal;
        if(boundedGeometry&&approximationMode!=boundedGeometryMode)return fail(TEXT("bounded_wheel_memo_mode_changed"));
        B::FLocalSample retainedSample;uint64 retainedSampleWork=0;
        const uint64 proposalBegan=budget.Used;
        EBoundedApproxWheelAttempt attempt;
        {
            // Same 200k cap includes publication and the original local sample.
            // Reserve their finite maximum before spending on Legacy kernels.
            C::FScopedDistanceBudgetFloor localSampleReserve(budget,optionalFloor+scene->Num()+128);
            if(!localSampleReserve.IsValid())return fail(TEXT("bounded_wheel_sample_reserve"));
            if(boundedGeometry)
            {
                const uint64 replayBegan=budget.Used;
                const bool paid=C::CandidateDetail::ConsumeRepeated(budget,boundedGeometry->ProposalWork);
                BoundedWheelMemo::ReplayedWork.fetch_add(budget.Used-replayBegan);
                if(!paid)return fail(TEXT("bounded_wheel_memo_proposal_work"));
                proposal=boundedGeometry->Proposal;attempt=EBoundedApproxWheelAttempt::BoundedApproxSpatialHit;
            }
            else attempt=approximateService->TryBoundedLegacyWheelProposal(q,budget,approximationMode,proposal);
        }
        const uint64 proposalWork=budget.Used-proposalBegan;
        if(budget.Used<optionalUsed||budget.Remaining>optionalRemaining||budget.ConsumptionFloor!=optionalFloor||
            budget.Used-optionalUsed!=uint64(int64(optionalRemaining)-int64(budget.Remaining)))
            return fail(TEXT("bounded_wheel_account_changed"));
        if(attempt==EBoundedApproxWheelAttempt::WorkFailure){++P::WorkFailures;return fail(TEXT("bounded_wheel_work"));}
        if(attempt==EBoundedApproxWheelAttempt::NumericFailure){++P::NumericFailures;return fail(TEXT("bounded_wheel_numeric"));}
        const auto afterAttempt=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(afterAttempt.Status!=Analytic::EAnalyticNumericStatus::Complete||afterAttempt.ControlFingerprint()!=numeric.ControlFingerprint()||
            !approximateService->MatchesPreparedSnapshot(snapshot)||BoundedLegacyWheelRuntimeMode()!=approximationMode)return fail(TEXT("bounded_wheel_query_context_changed"));
        B::ELocalRefusal localRefusal=B::ELocalRefusal::Input;
        const auto tryClose=[&]()->B::ELocalSample
        {
            if(attempt!=EBoundedApproxWheelAttempt::BoundedApproxSpatialHit)return B::ELocalSample::Unavailable;
            if(initialOut&&InitCertifiedLegacyWheelRuntimeEnabled()&&!proposal.Plane&&proposal.InitRepresentation)
            {
                namespace L=InitCertifiedLegacyWheelControl;++L::Attempts;const uint64 began=FPlatformTime::Cycles64();
                ON_SCOPE_EXIT{L::Ticks.fetch_add(FPlatformTime::Cycles64()-began);};
                if(boundedGeometry)++L::MemoHits;
                FInitCertifiedLegacyWheelProbeValuesPtr value;
                const auto status=FBoundedWheelValueFactory::CloseInit(snapshot,q,trace,delta,physicalRadius,serial,numeric.ControlFingerprint(),
                    approximateService,proposal,budget,value);
                if(status!=B::ELocalSample::BoundedLocalSample){++L::Refused;return status;}
                if(BoundedWheelMemo::Active())
                {
                    if(!boundedGeometry)
                    {
                        BoundedWheelMemo::FGeometry geometry;geometry.Proposal=proposal;geometry.OriginalCandidates=candidates;
                        geometry.SelectionWork=boundedSelectionWork;geometry.ProposalWork=proposalWork;geometry.InitOnly=true;geometry.InitPolicy=1;
                        if(!BoundedWheelMemo::ThreadCache().Store(key94,approximateService,approximationMode,geometry,budget,boundedGeometry))return B::ELocalSample::Work;
                    }
                    if(boundedGeometry)BoundedWheelMemo::ThreadCache().MarkMaterialized(key94,approximateService,approximationMode,boundedGeometry,value);
                }
                *initialOut=MoveTemp(value);++L::Accepted;return B::ELocalSample::BoundedLocalSample;
            }
            if(boundedGeometry&&boundedGeometry->InitOnly)return B::ELocalSample::Unavailable;
            FBoundedApproxFixedWheelProbeValuesPtr value;
            const auto status=FBoundedWheelValueFactory::Close(snapshot,q,trace,delta,physicalRadius,serial,numeric.ControlFingerprint(),
                approximationMode,approximateService,proposal,budget,boundedGeometry?&boundedGeometry->Sample:nullptr,
                boundedGeometry?boundedGeometry->SampleWork:0,retainedSample,retainedSampleWork,value,localRefusal);
            if(status!=B::ELocalSample::BoundedLocalSample)return status;
            if(BoundedWheelMemo::Active())
            {
                if(!boundedGeometry)
                {
                    BoundedWheelMemo::FGeometry geometry{proposal,retainedSample,candidates,boundedSelectionWork,proposalWork,retainedSampleWork};
                    geometry.InitPolicy=int32(InitCertifiedLegacyWheelRuntimeEnabled());
                    if(!BoundedWheelMemo::ThreadCache().Store(key94,approximateService,approximationMode,geometry,budget,boundedGeometry))
                        return B::ELocalSample::Work;
                }
                if(boundedGeometry)BoundedWheelMemo::ThreadCache().MarkMaterialized(key94,approximateService,approximationMode,boundedGeometry,value);
            }
            approximateOut=MoveTemp(value);return B::ELocalSample::BoundedLocalSample;
        };
        const auto closed=tryClose();
        if(closed==B::ELocalSample::Work){++P::WorkFailures;return fail(TEXT("bounded_wheel_local_work"));}
        if(closed==B::ELocalSample::Numeric){++P::NumericFailures;return fail(TEXT("bounded_wheel_local_numeric"));}
        if(closed==B::ELocalSample::BoundedLocalSample){++P::Accepted;out.Reset();return true;}
        if(attempt==EBoundedApproxWheelAttempt::BoundedApproxSpatialHit)
        {++P::LocalRefused;if(::StadiumSphereProfile70::Enabled())++P::LocalReasons[proposal.Plane?0:1][uint8(localRefusal)];}
        ++P::Fallback;
        // All optional debit is retained. A later exact exhaustion remains
        // Unknown; no credit, enlarged cap or miss is manufactured here.
    }
#endif

    initializeExactResult();
    Analytic::Prepared::FReferenceAffineContactPlanPtr affinePlan;
    const auto reference=snapshot->GetReference();
    if(reference&&!reference->GetExactAffineProofIdentity().IsEmpty()&&!candidates->GetCandidateIndices().IsEmpty())
    {
        Analytic::Prepared::FReferenceAffineBuildObservation86 build86;
        const auto choice=[&]()
        {
            auto status=C::AffineReferenceDetail::EOutcome::Unknown;
            const uint64 start=producer70.Active?FPlatformTime::Cycles64():0;
            if(producer70.Active)::StadiumSphereProfile70::AffineObservation86.InFlight.fetch_add(1);
            ON_SCOPE_EXIT
            {
                if(producer70.Active)
                {
                    ::StadiumSphereProfile70::ObserveAffineBuild86(build86,status,start);
                    if(!::StadiumSphereProfile70::AffineObservation86.InFlight.fetch_sub(1))::StadiumSphereProfile70::AffineObservation86.Integrity.store(false);
                }
            };
            status=Analytic::Prepared::FReferenceAffineContactPlan::Build(reference,candidates,budget,affinePlan,producer70.Active?&build86:nullptr,perAttempt,false);
            return status;
        }();
        if(producer70.Active)
        {
            auto& row=::StadiumSphereProfile70::Rows[::StadiumSphereProfile70::Contact];
            ::StadiumSphereProfile70::Add(row,row.AffineConsidered,1);
            if(choice==C::AffineReferenceDetail::EOutcome::Complete)
            {
                ::StadiumSphereProfile70::Add(row,row.AffineSelected,1);
                if(affinePlan)::StadiumSphereProfile70::Add(row,row.DominatedProviders,uint64(affinePlan->GetDominatedProviderCountForInspection()));
            }
            else if(choice==C::AffineReferenceDetail::EOutcome::NotApplicable)::StadiumSphereProfile70::Add(row,row.AffineGeneric,1);
            else ::StadiumSphereProfile70::Add(row,row.AffineUnknown,1);
        }
        if(choice==C::AffineReferenceDetail::EOutcome::Unknown||
            (choice==C::AffineReferenceDetail::EOutcome::Complete&&!affinePlan)||
            (choice==C::AffineReferenceDetail::EOutcome::NotApplicable&&affinePlan))
            return fail(TEXT("fixed_sphere_exact_source_product_context"));
    }
    M::FBoundedFixedProbeSample time;
    C::FScalarSphereContact scalar;
    M::FUniqueQuantizedNormal normal;
    M::FQuantizedNormalDiagnostic normalDiagnostic;
    I error;
    const auto checkContact=[&](const C::FTensorSphereContactEnclosure& contact)
    {
        ::StadiumSphereProfile70::FScope consumer70(::StadiumSphereProfile70::ContactConsumer);
        reason.Reset();
        const auto refuse=[&](const TCHAR* text){reason=text;return C::EContactEnclosureDecision::Refuse;};
        if(!M::TryBoundedFixedProbeSample(contact,delta,time))return refuse(TEXT("fixed_sphere_sample_seconds_unsupported"));
        const auto& first=contact.Leaves[0];
        if(!owners.IsValidIndex(first.SurfaceIndex))return refuse(TEXT("fixed_sphere_first_identity"));
        const auto& id=owners[first.SurfaceIndex];
        if(!id.SourceId||!id.SurfaceId||!id.PrimitiveId)return refuse(TEXT("fixed_sphere_empty_identity"));
        result->RegularLeafChartNormals.Reset();
        for(const auto& leaf:contact.Leaves)
        {
            if(!budget.Consume()||!owners.IsValidIndex(leaf.SurfaceIndex)||!SameOwner(id,owners[leaf.SurfaceIndex])||
                !(leaf.MinU>0&&leaf.MaxU<1&&leaf.MinV>0&&leaf.MaxV<1))return refuse(TEXT("fixed_sphere_nonunique_or_boundary_feature"));
            C::FIntervalPoint regularChart;
            const bool regular=affinePlan?affinePlan->ProveFixedMasterLeafRank(leaf,budget,regularChart):
                C::ProveRegularFaceLeafWithImmutablePlanarRank(scene,leaf,budget,regularChart);
            if(!regular)return refuse(TEXT("fixed_sphere_singular_or_unproved_face_chart"));
            result->RegularLeafChartNormals.Add(MoveTemp(regularChart));
        }
        C::FIntervalPoint direction;
        for(int32 d=0;d<3;++d)direction.Axis[d]=A::Subtract(I::Exact(q.End[d]),I::Exact(q.Start[d]));
        const I approach=C::ContactDetail::Dot(direction,contact.RadialUnitNormal);
        if(!A::Valid(approach)||!(approach.Upper<0))return refuse(TEXT("fixed_sphere_nonapproaching"));
        FVector3d point;
        for(int32 d=0;d<3;++d)point[d]=first.FixedParameterPoint.Axis[d].Lower;
        const FVector3d center=q.Start+(q.End-q.Start)*time.NormalizedRepresentative;
        if(C::ConvertSphereContact(contact,0,point,center,q.Radius,physicalRadius,scalar)!=C::EScalarSphereContactStatus::Bounded)
            return refuse(TEXT("fixed_sphere_scalar_values"));
        error=A::Add(I::Exact(scalar.LocationErrorCm),I::Exact(scalar.SurfaceErrorCm));
        if(!A::Valid(error)||error.Upper>slack.Lower)return refuse(TEXT("fixed_sphere_probe_position_quality"));
        if(!M::TryUniqueQuantizedNormal(contact.RadialUnitNormal,scalar.NormalStored,budget,normal,&normalDiagnostic))
        {
            reason=FString::Printf(TEXT("fixed_sphere_normal_cell_unknown Failure=%u Axis=%d Radial=[%.17g,%.17g] FirstNormalized=[%.17g,%.17g] Shifted=[%.17g,%.17g] ActualFirst=%.17g Integer=%.17g Next=%.17g RawLower=%.17g RawUpper=%.17g WidthRefinements=%d ConsumerRefinements=%d Remaining=%d Used=%llu Delta=%.17g PhysicalRadius=%.17g CollisionRadius=%.17g"),
                uint32(normalDiagnostic.Failure),normalDiagnostic.Axis,normalDiagnostic.Radial.Lower,normalDiagnostic.Radial.Upper,
                normalDiagnostic.FirstNormalized.Lower,normalDiagnostic.FirstNormalized.Upper,normalDiagnostic.Shifted.Lower,normalDiagnostic.Shifted.Upper,
                normalDiagnostic.ActualFirst,normalDiagnostic.Integer,normalDiagnostic.Next,contact.LowerTime,contact.UpperTime,
                contact.TemporalWidthRefinements,contact.ConsumerWidthRefinements,budget.Remaining,budget.Used,delta,physicalRadius,q.Radius);
            return normalDiagnostic.Failure==M::EQuantizedNormalFailure::AmbiguousCell?
                C::EContactEnclosureDecision::Refine:C::EContactEnclosureDecision::Refuse;
        }
        if(q.MinimumReferenceNormalDot>-1)
        {
            const FVector3d ref=q.ReferenceNormal.GetSafeNormal();if(ref.IsZero())return refuse(TEXT("fixed_sphere_reference_normal"));
            const I alignment=C::ContactDetail::Dot(C::Detail::Point(ref),contact.RadialUnitNormal);
            if(!A::Valid(alignment)||alignment.Lower<q.MinimumReferenceNormalDot||
                FVector3d::DotProduct(ref,scalar.NormalStored)<q.MinimumReferenceNormalDot)return refuse(TEXT("fixed_sphere_reference_normal"));
        }
        return C::EContactEnclosureDecision::Accept;
    };
    const C::FContactEnclosureFilter contactFilter(checkContact);
    const uint64 parityStart=producer70.Active?FPlatformTime::Cycles64():0;
    auto contact=[&]()->C::FTensorSphereContactEnclosure
    {
        ::StadiumSphereProfile70::FScope contact70(::StadiumSphereProfile70::Contact);const uint64 used70=budget.Used;
        C::ContactKernelProfile72::FEnabledScope kernelEnabled72(contact70.Active);
        C::CurveStationaryContact98::FEnabledScope curve98(::CurveStationaryContact98Profile::CVarEnabled.GetValueOnAnyThread()==1);
        // Source84 profiling-only capture. Future correction trial may disable ONLY this new scope.
        C::Detail::MinimumMidpointProfile84::FEnabledScope hashEnabled84(contact70.Active);
        const auto hashBefore84=C::Detail::MinimumMidpointProfile84::Counters();
        const auto kernelBefore72=C::ContactKernelProfile72::Counters();
        const auto memoBefore=C::ContactDetail::ThreadContactCellGeometryMemo().GetCountersForInspection();
        const auto midpointBefore=C::Detail::ThreadMinimumMidpointMemo().GetCountersForInspection();
        ON_SCOPE_EXIT
        {
            contact70.Budget(used70,budget.Used);
            if(contact70.Active)
            {
                const auto memoAfter=C::ContactDetail::ThreadContactCellGeometryMemo().GetCountersForInspection();auto& row=contact70.Row;
                const uint64 branchEnd=FPlatformTime::Cycles64();
                if(branchEnd<contact70.Start)row.Integrity.store(false);
                else ::StadiumSphereProfile70::Add(row,affinePlan?row.AffineBranchTicks:row.GenericBranchTicks,branchEnd-contact70.Start);
                ::StadiumSphereProfile70::Add(row,affinePlan?row.AffineBranchCalls:row.GenericBranchCalls,1);
                const auto delta=[&](std::atomic<uint64>& destination,uint64 before,uint64 after)
                {if(after<before)row.Integrity.store(false);else ::StadiumSphereProfile70::Add(row,destination,after-before);};
                delta(row.MemoRequests,memoBefore.Requests,memoAfter.Requests);delta(row.MemoHits,memoBefore.Hits,memoAfter.Hits);
                delta(row.MemoChildrenHits,memoBefore.ChildrenHits,memoAfter.ChildrenHits);delta(row.MemoCellsCreated,memoBefore.CellsCreated,memoAfter.CellsCreated);
                delta(row.MemoEvictions,memoBefore.GeometryEvictions,memoAfter.GeometryEvictions);
                const auto midpointAfter=C::Detail::ThreadMinimumMidpointMemo().GetCountersForInspection();
                delta(row.MidpointRequests,midpointBefore.Requests,midpointAfter.Requests);delta(row.MidpointHits,midpointBefore.Hits,midpointAfter.Hits);
                delta(row.MidpointBuilds,midpointBefore.Builds,midpointAfter.Builds);delta(row.MidpointEvictions,midpointBefore.Evictions,midpointAfter.Evictions);
                delta(row.RetainedMiddleRequests,midpointBefore.RetainedMiddleRequests,midpointAfter.RetainedMiddleRequests);delta(row.RetainedMiddleHits,midpointBefore.RetainedMiddleHits,midpointAfter.RetainedMiddleHits);
                delta(row.RetainedMiddlePublications,midpointBefore.RetainedMiddlePublications,midpointAfter.RetainedMiddlePublications);
                delta(row.OrdinaryBoxRecomputedCP,midpointBefore.OrdinaryBoxRecomputedCP,midpointAfter.OrdinaryBoxRecomputedCP);
                delta(row.OrdinaryBoxAvoidedCP,midpointBefore.OrdinaryBoxAvoidedCP,midpointAfter.OrdinaryBoxAvoidedCP);
                const auto kernelAfter72=C::ContactKernelProfile72::Counters();
                if(!kernelBefore72.Integrity||!kernelAfter72.Integrity)row.Integrity.store(false);
                delta(row.KernelTimeCalls,kernelBefore72.Calls[0],kernelAfter72.Calls[0]);delta(row.KernelTimeTicks,kernelBefore72.Ticks[0],kernelAfter72.Ticks[0]);
                delta(row.KernelSpatialCalls,kernelBefore72.Calls[1],kernelAfter72.Calls[1]);delta(row.KernelSpatialTicks,kernelBefore72.Ticks[1],kernelAfter72.Ticks[1]);
                delta(row.KernelSetupCalls,kernelBefore72.Calls[2],kernelAfter72.Calls[2]);delta(row.KernelSetupTicks,kernelBefore72.Ticks[2],kernelAfter72.Ticks[2]);
                for(int32 stage=0;stage<3;++stage)delta(row.KernelUnwinding,kernelBefore72.Unwinding[stage],kernelAfter72.Unwinding[stage]);
                const auto hashAfter84=C::Detail::MinimumMidpointProfile84::Counters();
                row.HashCaptureFlight84.fetch_add(1);
                if(!hashBefore84.Integrity||!hashAfter84.Integrity)row.HashIntegrity84.store(false);
                const auto hashAdd84=[&](std::atomic<uint64>& target,uint64 value)
                {if(target.fetch_add(value)>MAX_uint64-value)row.HashIntegrity84.store(false);};
                const auto hashDelta84=[&](std::atomic<uint64>& target,uint64 before,uint64 after)
                {if(after<before)row.HashIntegrity84.store(false);else hashAdd84(target,after-before);};
                for(uint32 stage=0;stage<5;++stage)
                {
                    hashDelta84(row.HashCalls84[stage],hashBefore84.Calls[stage],hashAfter84.Calls[stage]);
                    hashDelta84(row.HashFinished84[stage],hashBefore84.Finished[stage],hashAfter84.Finished[stage]);
                    hashDelta84(row.HashTicks84[stage],hashBefore84.Ticks[stage],hashAfter84.Ticks[stage]);
                    hashDelta84(row.HashUnwinding84[stage],hashBefore84.Unwinding[stage],hashAfter84.Unwinding[stage]);
                    if(hashBefore84.InFlight[stage]>MAX_uint64-hashAfter84.InFlight[stage])row.HashIntegrity84.store(false);
                    else hashAdd84(row.HashBoundaryFlight84[stage],hashBefore84.InFlight[stage]+hashAfter84.InFlight[stage]);
                }
                hashAdd84(row.HashSamples84,1);if(!row.HashCaptureFlight84.fetch_sub(1))row.HashIntegrity84.store(false);
            }
        };
        if(affinePlan)
        {
            // Explicit fixed-source extraction. This private native value
            // producer already authenticated the snapshot's retained master;
            // no independent-original-coefficient-family claim is made here.
            const auto fixed=affinePlan->Contact(budget,timeGap,0,perAttempt,pointWidth,normalWidth,contactFilter);
            const auto* bounds=fixed?fixed->InspectForSameFixedSource(reference,candidates):nullptr;
            if(!bounds)return {};
            return *bounds;
        }
        bool actualKernel89=false;::StadiumSphereProfile70::FGenericScope89 observation89(diagnostic,contact70.Active,actualKernel89);
        const auto finish89=[&](C::FTensorSphereContactEnclosure value){observation89.Result=value.Status;return value;};
        if(!candidates->GetCandidateIndices().IsEmpty())
        {
            const uint64 before89=budget.Used,start89=contact70.Active?FPlatformTime::Cycles64():0;uint64 tested89=0;
            const auto gate89=candidates->SeparateAllOblique89(budget,tested89);observation89.Proof(gate89,tested89,before89,budget.Used,start89);
            if(gate89==C::EObliqueCapsule89::Unknown)return finish89({});
            if(gate89==C::EObliqueCapsule89::Separated)
            {C::FTensorSphereContactEnclosure clear;clear.Status=C::ETensorSphereContactStatus::NoContactWithProvidedTensors;clear.LowerTime=clear.UpperTime=1;return finish89(MoveTemp(clear));}
        }
        return finish89(candidates->Contact(budget,timeGap,0,perAttempt,pointWidth,normalWidth,24,&contactFilter,&actualKernel89));
    }();
    if(producer70.Active)::ParityContactCost::Record(bool(affinePlan),reference,candidates,q,contact,FPlatformTime::Cycles64()-parityStart);
    if(producer70.Active&&contact.HasCallAccounting)
    {auto& row70=::StadiumSphereProfile70::Rows[::StadiumSphereProfile70::Contact];::StadiumSphereProfile70::Add(row70,row70.ContactAccounting,1);::StadiumSphereProfile70::Add(row70,row70.TimeWork,contact.TimeWorkUsed);::StadiumSphereProfile70::Add(row70,row70.SpatialCells,contact.SpatialVisitedCells);
        ::StadiumSphereProfile70::Add(row70,affinePlan?row70.AffineSpatialCells:row70.GenericSpatialCells,contact.SpatialVisitedCells);
        ::StadiumSphereProfile70::Add(row70,affinePlan?row70.AffineTimeWork:row70.GenericTimeWork,contact.TimeWorkUsed);}
    if(contact.Status==C::ETensorSphereContactStatus::NoContactWithProvidedTensors)
    {
        if(!externalSeparated)return fail(TEXT("fixed_sphere_external_trace"));
        result->Result=ECanonicalFixedSphereWheelProbeResult::CanonicalSeparated;result->Contact=MoveTemp(contact);
        if(!result->Matches(snapshot,q,trace,delta,physicalRadius,serial))return fail(TEXT("fixed_sphere_separation_identity"));
        out=MoveTemp(result);return true;
    }
    if(contact.Status!=C::ETensorSphereContactStatus::FirstContactEnclosed||contact.Leaves.IsEmpty()||contact.CollisionRadiusBits!=A::Bits(q.Radius))
    {
        // Non-Time includes setup, splits, callbacks and wrapper remapping;
        // it is NOT an isolated UV/physics-CPU measurement. Default wrapper
        // return{} deliberately means unavailable, never total-World blame.
        const bool accounting=contact.HasCallAccounting&&contact.CallInitialUsed<=budget.Used&&contact.TimeWorkUsed<=budget.Used-contact.CallInitialUsed;
        const uint64 nonTimeWork=accounting?budget.Used-contact.CallInitialUsed-contact.TimeWorkUsed:0;
        const int32 localWidth=contact.WidthSurface;
        const int32 globalWidth=affinePlan?affinePlan->MapRetainedProviderForInspection(localWidth):
            (localWidth>=0&&localWidth<candidates->GetCandidateIndices().Num()?candidates->GetCandidateIndices()[localWidth]:INDEX_NONE);
        UE_LOG(LogTemp,Warning,TEXT("[CanonicalFixedSphereContactWork] AccountingKnown=%d CallBase=%llu TimeWork=%llu NonTimeWork=%llu Cells=%llu SplitU=%llu SplitV=%llu Site=%s WidthDiag=%d WidthLocal=%d WidthProvider=%d WidthDepth=%d,%d WidthUV=[%.17g,%.17g]x[%.17g,%.17g] WidthTime=[%.17g,%.17g] Targets=%.17g,%.17g"),
            int32(accounting),contact.CallInitialUsed,contact.TimeWorkUsed,nonTimeWork,contact.SpatialVisitedCells,contact.SpatialSplitsU,contact.SpatialSplitsV,contact.WorkSite,int32(contact.HasWidthDiagnostic),localWidth,globalWidth,contact.WidthDepthU,contact.WidthDepthV,contact.WidthMinU,contact.WidthMaxU,contact.WidthMinV,contact.WidthMaxV,contact.WidthTimeLower,contact.WidthTimeUpper,contact.WidthPointTarget,contact.WidthNormalTarget);
        if(contact.TimeDyadicSite)
            UE_LOG(LogTemp,Warning,TEXT("[CanonicalFixedSphereTimeRefusal] Site=%u Depth=%d A=%.17g B=%.17g AnchorRefinements=%d"),
                uint32(contact.TimeDyadicSite),contact.TimeDyadicDepth,contact.TimeDyadicA,contact.TimeDyadicB,contact.TimeAnchorRefinements);
        const FString consumerReason=reason;
        return fail(*FString::Printf(TEXT("fixed_sphere_first_contact_unknown Status=%u Failure=%u TimeFailure=%u WidthRefinements=%d ConsumerRefinements=%d Leaves=%d RadiusBits=%016llX ExpectedRadiusBits=%016llX Candidates=%d Excluded=%d Remaining=%d Used=%llu PerAttempt=%d RawLower=%.17g RawUpper=%.17g TimeGap=%.17g PointWidth=%.17g NormalWidth=%.17g Consumer=%s"),
            uint32(contact.Status),uint32(contact.Failure),uint32(contact.TimeFailure),contact.TemporalWidthRefinements,contact.ConsumerWidthRefinements,
            contact.Leaves.Num(),contact.CollisionRadiusBits,A::Bits(q.Radius),candidates->GetCandidateIndices().Num(),candidates->GetExcludedIndices().Num(),
            budget.Remaining,budget.Used,perAttempt,contact.LowerTime,contact.UpperTime,timeGap,pointWidth,normalWidth,*consumerReason));
    }
    if(!externalSeparated)
    {
        C::FIntervalPoint prefix;
        if(!budget.Consume()||!FixedSpherePrefixHull(q,contact.UpperTime,prefix)||
            !snapshot->GetExternalLandscape()->StrictlySeparated(q,prefix))
            return fail(TEXT("fixed_sphere_external_before_stadium_contact"));
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
        static std::atomic<bool> reported=false;
        if(!reported.exchange(true,std::memory_order_relaxed))
            UE_LOG(LogTemp,Display,TEXT("[FixedSphereLandscapeAfterStadium] ClearPrefixUpper=%.17g"),contact.UpperTime);
#endif
    }
    const auto& id=owners[contact.Leaves[0].SurfaceIndex];
        Analytic::FWorldHit hit;hit.bHit=true;hit.Time=time.NormalizedRepresentative;
        hit.Location=scalar.Location;hit.Point=scalar.ImpactPoint;hit.Normal=scalar.NormalStored;
        hit.QueryPoint=scalar.Location-q.Radius*scalar.NormalStored;
        hit.bSurfaceNormalMayVary=true;hit.SurfaceFeatureKind=EContactFeatureKind::Face;
        hit.GeometricErrorBoundCm=scalar.SampleEvaluationErrorCm; // provider evaluation only
        hit.SourceId=id.SourceId;hit.SurfaceId=id.SurfaceId;hit.FeatureId=id.FeatureId;hit.PrimitiveId=id.PrimitiveId;
        hit.CanonicalGroupId=id.CanonicalGroupId;hit.MaterialId=id.MaterialId;
        result->Result=ECanonicalFixedSphereWheelProbeResult::CanonicalContact;result->Contact=MoveTemp(contact);
        result->Time=time;result->Scalar=scalar;result->Normal=normal;result->ProbePositionError=error;result->Hit=MoveTemp(hit);
        if(!result->Matches(snapshot,q,trace,delta,physicalRadius,serial))return fail(TEXT("fixed_sphere_contact_identity"));

        if(cache94&&cacheMiss94)
        {
            const auto store94=FHistoricalQueryCache94::PrepareStore(key94,result->Contact,MakeArrayView(result->RegularLeafChartNormals),budget);
            if(store94==ECompleteCache94::Failure)return fail(TEXT("historical_query_cache94_store_work"));
            if(store94==ECompleteCache94::Hit)
            {
                uint64 retainedBytes94=0;
                if(earlyEnabled94)
                {
                    const auto retainStatus94=FHistoricalQueryCache94::PrepareEarlyRetention(key94,candidates,budget,retainedBytes94);
                    if(retainStatus94==ECompleteCache94::Failure)return fail(TEXT("early_historical_query_cache94_retention_work"));
                    // NotApplicable limits early retention only. The original
                    // late geometry/key store below is still executed unchanged.
                }
                TSharedPtr<FCompleteGeometryProof94,ESPMode::ThreadSafe> pure94(new FCompleteGeometryProof94);
                pure94->Contact=result->Contact;SanitizeContactAccounting94(pure94->Contact);
                pure94->RegularNormals=result->RegularLeafChartNormals;pure94->Time=result->Time;pure94->Scalar=result->Scalar;
                pure94->Normal=result->Normal;pure94->PositionError=result->ProbePositionError;pure94->Hit=result->Hit;
                cache94->StoreReserved(key94,MoveTemp(pure94),retainedBytes94?candidates:C::FReferenceCandidateSetPtr{},retainedBytes94);
            }
        }

        out=MoveTemp(result);return true;

}

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
bool EvaluateCanonicalFixedSphereWheelProbeValues(FPreparedWorldSnapshotPtr snapshot,const Analytic::FWorldQuery& query,
    const FNativeSolidTraceValues& trace,double delta,double physicalRadius,uint64 serial,
    C::FDistanceBudget& budget,FCanonicalFixedSphereWheelProbeValuesPtr& out,FString& reason,FFixedSphereProbeDiagnostic89 diagnostic,
    const C::FInheritedReachInput91& inheritedReach)
{
    FBoundedApproxFixedWheelProbeValuesPtr unused;
    return EvaluateFixedSphereWheelProbeRoute(MoveTemp(snapshot),query,trace,delta,physicalRadius,serial,budget,out,reason,diagnostic,inheritedReach,{},unused);
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
namespace
{
bool ObliqueFixture89(const TArray<FVector3d>& points,double radius,C::FReferenceCandidateSetPtr& out)
{
    TArray<C::FIntervalTensorSurface> surfaces;for(const auto& p:points){C::FIntervalTensorSurface v;v.ControlPoints.Add(C::Detail::Point(p));surfaces.Add(v);}
    C::FDistanceBudget build{2048};C::FIndexedIntervalUnionPtr scene;if(!C::FIndexedIntervalUnion::Build(MakeArrayView(surfaces),build,scene))return false;
    C::FIntervalBezierCenterPath path;path.Degree=1;path.ControlPoints={C::Detail::Point(FVector3d::ZeroVector),C::Detail::Point(FVector3d(1,1,0))};
    C::FDistanceBudget select{2048};return C::FReferenceCandidateSet::Select(scene,path,radius,select,out);
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FObliqueCapsuleSeparation89Test,"IAmSpeed.AnalyticWorld.CompleteUnion.ObliqueCapsuleSeparation",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FObliqueCapsuleSeparation89Test::RunTest(const FString&)
{
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 controls=VectorGetControlRegister();for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        C::FReferenceCandidateSetPtr p;if(!TestTrue(TEXT("AABB retains"),ObliqueFixture89({FVector3d(0,1,0)},.6,p)&&p->GetCandidateIndices().Num()==1))return false;
        C::FIntervalBezierCenterPath path;path.Degree=1;path.ControlPoints={C::Detail::Point(FVector3d::ZeroVector),C::Detail::Point(FVector3d(1,1,0))};
        C::FReferenceCandidateSetPtr old85;C::FDistanceBudget oldWork{2048};
        TestTrue(TEXT("Old85 expanded box keeps corner"),C::FReferenceCandidateSet::SelectFixedLinearSphere(p->GetScene(),path,.6,oldWork,old85)&&old85->GetCandidateIndices().Num()==1);
        C::FDistanceBudget work{128};uint64 tested=0;
        TestTrue(TEXT("Oblique excludes full capsule"),p->SeparateAllOblique89(work,tested)==C::EObliqueCapsule89::Separated&&tested==1);
        TestTrue(TEXT("Exact debit"),work.Used==59&&work.Remaining==69);
        TestTrue(TEXT("Partition unchanged"),p->GetCandidateIndices().Num()==1&&p->GetExcludedIndices().IsEmpty());
    }
    TestTrue(TEXT("Restored"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FObliqueCapsuleRetention89Test,"IAmSpeed.AnalyticWorld.CompleteUnion.ObliqueCapsuleRetentionAndSamples",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FObliqueCapsuleRetention89Test::RunTest(const FString&)
{
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 controls=VectorGetControlRegister();for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        C::FIntervalBezierCenterPath path;path.Degree=1;path.ControlPoints={C::Detail::Point(FVector3d(-1,0,0)),C::Detail::Point(FVector3d(1,0,0))};
        const auto direction=C::Detail::Point(FVector3d(2,0,0)),tangent=C::Detail::Point(FVector3d(0,1,0));
        TestTrue(TEXT("Tangency retained"),C::CandidateDetail::ObliqueFixedSegmentBox89(path,tangent,A::Interval::Exact(1),direction)==C::EObliqueCapsule89::Near);
        TestTrue(TEXT("Zero axis retained"),C::CandidateDetail::ObliqueFixedSegmentBox89(path,tangent,A::Interval::Exact(1),C::Detail::Point(FVector3d::ZeroVector))==C::EObliqueCapsule89::Near);
        auto invalid=tangent;invalid.Axis[0].Known=false;TestTrue(TEXT("Invalid never clears"),C::CandidateDetail::ObliqueFixedSegmentBox89(path,invalid,A::Interval::Exact(1),direction)==C::EObliqueCapsule89::Unknown);
        C::FReferenceCandidateSetPtr mixed;TestTrue(TEXT("Mixed fixture"),ObliqueFixture89({FVector3d(0,1,0),FVector3d(.5,.5,0)},.6,mixed));
        if(mixed){C::FDistanceBudget work{256};uint64 tested=0;TestTrue(TEXT("One near blocks miss"),mixed->SeparateAllOblique89(work,tested)==C::EObliqueCapsule89::Near&&tested==2);}
        for(int32 j=0;j<24;++j){const FVector3d point(double(j-12)/8.,double((j*7)%19-9)/7.,double(j%5)/9.);
            if(C::CandidateDetail::ObliqueFixedSegmentBox89(path,C::Detail::Point(point),A::Interval::Exact(.36),direction)==C::EObliqueCapsule89::Separated)
                for(int32 k=0;k<=32;++k)TestTrue(TEXT("No independent sampled collision"),(FVector3d(-1.+2.*double(k)/32.,0,0)-point).SizeSquared()>.36);}
    }
    TestTrue(TEXT("Restored"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FObliqueCapsuleBudget89Test,"IAmSpeed.AnalyticWorld.CompleteUnion.ObliqueCapsuleBudgetAndDiagnostic",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FObliqueCapsuleBudget89Test::RunTest(const FString&)
{
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 controls=VectorGetControlRegister();for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        C::FReferenceCandidateSetPtr p;if(!ObliqueFixture89({FVector3d(0,1,0)},.6,p))return false;uint64 tested=0;C::FDistanceBudget shortWork{58};
        TestTrue(TEXT("No partial provider debit"),p->SeparateAllOblique89(shortWork,tested)==C::EObliqueCapsule89::Unknown&&shortWork.Used==5&&tested==0);
        C::FDistanceBudget floorWork{66};floorWork.ConsumptionFloor=7;
        TestTrue(TEXT("Floor intact"),p->SeparateAllOblique89(floorWork,tested)==C::EObliqueCapsule89::Separated&&floorWork.Used==59&&floorWork.Remaining==7);
        C::FDistanceBudget emptyWork{0};TestTrue(TEXT("No budget no miss"),p->SeparateAllOblique89(emptyWork,tested)==C::EObliqueCapsule89::Unknown&&emptyWork.Used==0);
        using D=FFixedSphereProbeDiagnostic89;using S=Analytic::EStaticQuerySite;
        TestTrue(TEXT("Default UNKNOWN"),StadiumSphereProfile70::Bucket89(D{})==0);
        TestTrue(TEXT("Actual roles split"),StadiumSphereProfile70::Bucket89(D{uint8(S::WheelSuspensionProbe),5,2})==12&&StadiumSphereProfile70::Bucket89(D{uint8(S::WheelEstablishedSupportProbe),0,1})==13);
        TestTrue(TEXT("Bad diagnostic is not admission"),StadiumSphereProfile70::Bucket89(D{uint8(S::SolidSweep),0,1})==0&&StadiumSphereProfile70::Bucket89(D{uint8(S::WheelSuspensionProbe),6,1})==0&&StadiumSphereProfile70::Bucket89(D{uint8(S::WheelSuspensionProbe),0,0})==0);
    }
    TestTrue(TEXT("Restored"),restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCanonicalFixedSphereWheelLinearPathTest,
    "IAmSpeed.AnalyticWorld.NativeSolid.CanonicalFixedSphereWheelLinearPath",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCanonicalFixedSphereWheelLinearPathTest::RunTest(const FString&)
{
    const auto original=Speed::CaptureFrameFloatingPointContext();
    Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE numeric fixture"),original.bValid&&restore.IsApplied()&&
        std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        C::FIntervalTensorSurface plane;plane.DegreeU=plane.DegreeV=1;
        for(const FVector3d p:{FVector3d(-10,-10,0),FVector3d(10,-10,0),FVector3d(-10,10,0),FVector3d(10,10,0)})
            plane.ControlPoints.Add(C::Detail::Point(p));
        C::FDistanceBudget build{128};C::FIndexedIntervalUnionPtr scene;
        if(!TestTrue(TEXT("complete immutable arithmetic plane"),C::FIndexedIntervalUnion::Build(MakeArrayView(&plane,1),build,scene)))return false;
        // Actual failed Fast7 query coefficients; not a gameplay spawn/profile.
        const FVector3d start(35.96143,7472.2,24.882188841165025),end(35.96143,7472.2,.87218884138854413);
        const double radius=12.510000228881836;
        C::FReferenceCandidateSetPtr proof;C::FDistanceBudget work{256};
        if(!TestTrue(TEXT("actual two-endpoint wheel trace reaches candidate selection"),
            SelectFixedSphereTraceCandidates(scene,start,end,radius,work,proof)))return false;
        TestTrue(TEXT("all retained providers accounted for in separated trace"),proof&&proof->GetScene()==scene&&
            proof->GetCandidateIndices().IsEmpty()&&proof->GetExcludedIndices().Num()==1&&proof->GetExcludedIndices()[0]==0);
        TestTrue(TEXT("typed complete-union contact search reports no provided contact"),
            proof->Contact(work,1./64.,0,16,.01,.05).Status==C::ETensorSphereContactStatus::NoContactWithProvidedTensors);
        // The degree-zero/two-coefficient construction was the real failure.
        C::FIntervalBezierCenterPath malformed;malformed.ControlPoints={C::Detail::Point(start),C::Detail::Point(end)};
        work={256};TestFalse(TEXT("malformed degree-zero segment is still refused"),C::FReferenceCandidateSet::Select(scene,malformed,radius,work,proof));
        TestFalse(TEXT("refused malformed curve clears earlier proof"),proof.IsValid());
        work={256};if(!TestTrue(TEXT("stationary tangent remains a valid linear trace"),
            SelectFixedSphereTraceCandidates(scene,FVector3d(0,0,radius),FVector3d(0,0,radius),radius,work,proof)))return false;
        TestTrue(TEXT("strict selection retains exact tangency"),proof->GetCandidateIndices().Num()==1&&proof->GetCandidateIndices()[0]==0&&proof->GetExcludedIndices().IsEmpty());
        // Optional planar culling must reserve the exact old provider/CP
        // cost. Exercise both sides of the 512-unit optional-proof boundary.
        C::FIntervalBezierCenterPath tangentPath;
        if(!TestTrue(TEXT("tangent path for reserve test"),C::CenterDetail::Linear(FVector3d(0,0,radius),FVector3d(0,0,radius),tangentPath)))return false;
        for(int32 amount:{5,6,255,256,511,512,516,517,518})
        {
            C::FDistanceBudget limited{amount};C::FReferenceCandidateSetPtr selected;
            TestTrue(TEXT("optional edge proof never steals baseline work"),
                C::FReferenceCandidateSet::Select(scene,tangentPath,radius,limited,selected,nullptr,true)&&selected&&
                selected->GetCandidateIndices().Num()==1&&selected->GetExcludedIndices().IsEmpty());
            TestTrue(TEXT("fixed optional work charged only when fully reserved"),limited.Used==uint64(amount>=517?517:5));
        }
        C::FDistanceBudget protectedWork{520};protectedWork.ConsumptionFloor=10;C::FReferenceCandidateSetPtr protectedProof;
        TestTrue(TEXT("optional edge cull preserves caller floor"),
            C::FReferenceCandidateSet::Select(scene,tangentPath,radius,protectedWork,protectedProof,nullptr,true)&&
            protectedProof&&protectedWork.Used==5&&protectedWork.Remaining==515);
        work={0};TestFalse(TEXT("empty work budget never publishes a trace proof"),SelectFixedSphereTraceCandidates(scene,start,end,radius,work,proof));
        TestFalse(TEXT("budget refusal clears prior tangent proof"),proof.IsValid());

        // A fan triangle can span a huge AABB while being far from the query.
        C::FIntervalTensorSurface triangle;triangle.DegreeU=triangle.DegreeV=1;
        for(const FVector3d p:{FVector3d(-100,100,0),FVector3d(100,100,0),FVector3d(100,90,0),FVector3d(100,90,0)})
            triangle.ControlPoints.Add(C::Detail::Point(p));
        const auto cull=[&](const C::FIntervalTensorSurface& s,const C::FIntervalPoint& center,double r,bool expected)
        {
            C::FDistanceBudget b{2048};bool separated=false;
            return C::Detail::SeparatePlanarEdgeHull(s,center,A::Square(I::Exact(r)),b,separated)&&separated==expected;
        };
        TestTrue(TEXT("fan false AABB is rejected by supporting edge"),cull(triangle,C::Detail::Point(FVector3d(-90,91,0)),1,true));
        TestTrue(TEXT("exact edge tangency is kept"),cull(triangle,C::Detail::Point(FVector3d(0,101,0)),1,false));
        TestTrue(TEXT("near triangle seam is kept"),cull(triangle,C::Detail::Point(FVector3d(0,99.5,0)),1,false));
        for(int32 i:{2,3})for(int32 d=0;d<2;++d)
        {double lo=0,hi=0;A::Outward(triangle.ControlPoints[i].Axis[d].Lower,false,lo);A::Outward(triangle.ControlPoints[i].Axis[d].Upper,true,hi);triangle.ControlPoints[i].Axis[d]={lo,hi,true};}
        TestTrue(TEXT("independent interval apex boxes can be excluded without correlation"),cull(triangle,C::Detail::Point(FVector3d(-90,91,0)),1,true));
        auto curved=triangle;curved.ControlPoints[3].Axis[2]=I::Exact(1);
        TestTrue(TEXT("nonplanar surface stays generic"),cull(curved,C::Detail::Point(FVector3d(-90,91,0)),1,false));
        auto crossing=C::Detail::Point(FVector3d(-90,91,0));crossing.Axis[0]={-90,90,true};crossing.Axis[1]={91,100,true};
        TestTrue(TEXT("whole crossing hull cannot be excluded by distant endpoints"),cull(triangle,crossing,1,false));

        C::ContactDetail::FPlanarProjectionDomains domains;
        C::FDistanceBudget projectionBudget{10000};
        const auto foot=C::Detail::Point(FVector3d(50,97,2));
        TestTrue(TEXT("legacy independent-box API keeps its strict contract"),
            C::ContactDetail::ProjectedPlanarDomains(MakeArrayView(&triangle,1),foot,projectionBudget,domains)==C::ContactDetail::EPlanarProjectionProof::NotApplicable);
        TestTrue(TEXT("homotopy proves actual bilinear coverage with independent apex intervals"),
            C::ContactDetail::ProjectedPlanarDomains(MakeArrayView(&triangle,1),foot,projectionBudget,domains,true)==C::ContactDetail::EPlanarProjectionProof::Proved&&domains.Providers[0].Bounded);
        auto open=triangle;open.ControlPoints[3]=C::Detail::Point(FVector3d(-100,80,0));
        TestTrue(TEXT("large independent apex perturbation cannot manufacture coverage"),
            C::ContactDetail::ProjectedPlanarDomains(MakeArrayView(&open,1),foot,projectionBudget,domains,true)==C::ContactDetail::EPlanarProjectionProof::NotApplicable);


    }
    TestTrue(TEXT("original numeric context restored"),restore.Restore());
    return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCanonicalContactRetainedWidthTest,
    "IAmSpeed.AnalyticWorld.NativeSolid.CanonicalContactRetainedWidth",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCanonicalContactRetainedWidthTest::RunTest(const FString&)
{
    const auto original=Speed::CaptureFrameFloatingPointContext();
    Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE mathematical fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        C::FIntervalTensorSurface plane;plane.DegreeU=plane.DegreeV=1;
        for(const FVector3d p:{FVector3d(-128,-8,0),FVector3d(-128,8,0),FVector3d(128,-8,0),FVector3d(128,8,0)})plane.ControlPoints.Add(C::Detail::Point(p));
        C::FIntervalBezierCenterPath path;
        if(!TestTrue(TEXT("fixed arithmetic curve"),C::CenterDetail::Linear(FVector3d(0,0,3),FVector3d(100,0,-3),path)))return false;
        C::FDistanceBudget oneAttempt{1048576};
        const auto wide=C::BoundTensorSphereFirstContact(MakeArrayView(&plane,1),path,1.,oneAttempt,1./1024.,0.,2048,.001,1.,1);
        TestTrue(TEXT("original wide bracket needs explicit continuation"),wide.Status==C::ETensorSphereContactStatus::Unknown&&wide.Failure==C::ETensorSphereContactFailure::WidthRefinementLimit&&wide.Leaves.IsEmpty());
        C::FDistanceBudget work{1048576};
        const auto bound=C::BoundTensorSphereFirstContact(MakeArrayView(&plane,1),path,1.,work,1./1024.,0.,2048,.001,1.);
        TestTrue(TEXT("same contact operation narrows an overly wide union"),bound.Status==C::ETensorSphereContactStatus::FirstContactEnclosed&&bound.TemporalWidthRefinements>0&&!bound.Leaves.IsEmpty());
        TestTrue(TEXT("caller point and normal widths are unchanged"),C::ContactDetail::Fits(bound.SurfacePoint,.001)&&C::ContactDetail::Fits(bound.RadialUnitNormal,1.));
        TestTrue(TEXT("retained first root stays enclosed"),bound.LowerTime<=1./3.&&bound.UpperTime>=1./3.);
        TestTrue(TEXT("one shared budget consumed without refill"),work.Used>0&&uint64(work.Remaining)+work.Used==1048576);
        C::FDistanceBudget empty{0};
        TestTrue(TEXT("zero work cannot publish earlier contact leaves"),C::BoundTensorSphereFirstContact(MakeArrayView(&plane,1),path,1.,empty,1./1024.,0.,2048,.001,1.).Status==C::ETensorSphereContactStatus::Unknown);
        C::TimeDetail::FRetainedSphereTimeSearch search(MakeArrayView(&plane,1),path,1.,work,0.,2048);
        const auto first=search.Advance(1./1024.);const auto second=search.Advance(1./4096.);
        TestTrue(TEXT("continued frontier yields nested first-contact brackets"),first.Status==C::ETensorSphereTimeStatus::FirstContactBracket&&second.Status==C::ETensorSphereTimeStatus::FirstContactBracket&&second.LowerTime>=first.LowerTime&&second.UpperTime<=first.UpperTime&&second.InitialSeparationProved);
        TestTrue(TEXT("same or larger accuracy gap is refused"),search.Advance(1./4096.).Status==C::ETensorSphereTimeStatus::Unknown);
    }
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCanonicalFixedProbeBoundedSampleTest,
    "IAmSpeed.AnalyticWorld.NativeSolid.CanonicalFixedProbeBoundedSample",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCanonicalFixedProbeBoundedSampleTest::RunTest(const FString&)
{
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE arithmetic fixtures"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        // Pure conversion enclosure of a plane contact at t=.5, not a World
        // request/spawn or a replacement for the actual Contact producer.
        C::FTensorSphereContactEnclosure c;c.Status=C::ETensorSphereContactStatus::FirstContactEnclosed;
        c.LowerTime=.25;c.UpperTime=.75;c.CollisionRadiusBits=A::Bits(1.01);
        c.SurfacePoint=C::Detail::Point(FVector3d::ZeroVector);c.RadialUnitNormal=C::Detail::Point(FVector3d(0,0,1));
        C::FTensorSphereContactLeaf leaf;leaf.SurfaceIndex=0;leaf.MinU=leaf.MinV=.25;leaf.MaxU=leaf.MaxV=.75;
        leaf.FixedParameterPoint=leaf.PointHull=c.SurfacePoint;leaf.RadialNormalHull=c.RadialUnitNormal;leaf.FixedParameter=FVector2d(.5,.5);c.Leaves.Add(leaf);
        M::FUniqueFloatSeconds unique;TestFalse(TEXT("old unique-seconds theorem still refuses multiple bins"),M::TryUniqueFloatSeconds(c,1.,unique));
        M::FBoundedFixedProbeSample sample;
        if(!TestTrue(TEXT("distinct probe sample captures its own conversion"),M::TryBoundedFixedProbeSample(c,1.,sample)&&M::MatchesBoundedFixedProbeSample(c,1.,sample)))return false;
        TestTrue(TEXT("same old lower double chosen without a scalar-root claim"),sample.NormalizedRepresentative==c.LowerTime&&sample.SampleProduct==.25&&sample.NativeSecondsBits==M::FloatBits(.25f));
        const FVector3d start(0,0,1.01005),end(0,0,1.00995),center=start+(end-start)*sample.NormalizedRepresentative;
        C::FScalarSphereContact scalar;
        if(!TestTrue(TEXT("actual sampled center is independently bounded"),C::ConvertSphereContact(c,0,FVector3d::ZeroVector,center,1.01,1.,scalar)==C::EScalarSphereContactStatus::Bounded))return false;
        const I slack=A::Subtract(I::Exact(1.01),I::Exact(1.));
        const I error=A::Add(I::Exact(scalar.LocationErrorCm),I::Exact(scalar.SurfaceErrorCm));
        TestTrue(TEXT("sample pays the same captured physical slack"),A::Valid(error)&&error.Upper<=slack.Lower);
        C::FScalarSphereContact distant;const FVector3d farStart(0,0,3.01),farEnd(0,0,-.99),far=farStart+(farEnd-farStart)*sample.NormalizedRepresentative;
        if(!TestTrue(TEXT("large segment error is measured"),C::ConvertSphereContact(c,0,FVector3d::ZeroVector,far,1.01,1.,distant)==C::EScalarSphereContactStatus::Bounded))return false;
        const I tooFar=A::Add(I::Exact(distant.LocationErrorCm),I::Exact(distant.SurfaceErrorCm));
        TestTrue(TEXT("unchanged production quality comparison rejects large sample displacement"),A::Valid(tooFar)&&tooFar.Upper>slack.Lower);
        auto changed=sample;changed.NativeSecondsBits^=1;TestFalse(TEXT("changed seconds bits refused"),M::MatchesBoundedFixedProbeSample(c,1.,changed));
        changed=sample;changed.NormalizedRepresentative=A::FromBits(A::Bits(sample.NormalizedRepresentative)+1);TestFalse(TEXT("changed sample bit refused"),M::MatchesBoundedFixedProbeSample(c,1.,changed));
        TestFalse(TEXT("changed actual Delta refused"),M::MatchesBoundedFixedProbeSample(c,A::FromBits(A::Bits(1.)+1),sample));
        FCanonicalFixedSphereWheelProbeValues v;v.Result=ECanonicalFixedSphereWheelProbeResult::CanonicalContact;v.NumericControl=sample.NumericControl;v.Time=sample;v.Scalar=scalar;
        v.Normal.Quantized=FVector(0,0,1);v.Normal.ReNormalized=FVector(0,0,1);
        v.Hit.SourceId=1;v.Hit.SurfaceId=2;v.Hit.FeatureId=3;v.Hit.PrimitiveId=4;v.Hit.CanonicalGroupId=5;v.Hit.MaterialId=6;
        v.Hit.Location=scalar.Location;v.Hit.Point=scalar.ImpactPoint;v.Hit.QueryPoint=scalar.Location-1.01*FVector(0,0,1);
        v.Hit.bHit=true;v.Hit.Time=sample.NormalizedRepresentative;v.Hit.Normal=scalar.NormalStored;
        v.Hit.SurfaceFeatureKind=EContactFeatureKind::Face;v.Hit.bSurfaceNormalMayVary=true;v.Hit.GeometricErrorBoundCm=scalar.SampleEvaluationErrorCm;
        auto actual=SHitResult::FromAnalyticHitAtSeconds(v.Hit,sample.NativeSeconds,0);
        actual.FaceIndex=static_cast<int32>(v.Hit.PrimitiveId&0x7fffffffull);actual.ImpactNormal=QuantizeUnitNormal(actual.ImpactNormal);
        TestTrue(TEXT("actual consumed sample bits match"),v.MatchesConsumedContact(actual));
        auto bad=actual;bad.TOI=0.f;TestFalse(TEXT("mutated consumed TOI refused"),v.MatchesConsumedContact(bad));
        bad=actual;bad.Location.X=A::FromBits(A::Bits(1.)+1);TestFalse(TEXT("mutated consumed Location refused"),v.MatchesConsumedContact(bad));
        bad=actual;bad.ImpactNormal.X=1.e-3;TestFalse(TEXT("mutated consumed normal refused"),v.MatchesConsumedContact(bad));
        double gap=0;TestTrue(TEXT("quality-derived search gap is bounded"),M::TryFixedProbeSpatialTimeGap(FVector(0,0,3),FVector(0,0,-3),.001,1.,1.01,gap)&&gap>0&&gap*6.<.001*.125);
        M::FBoundedFixedProbeSample zero;TestTrue(TEXT("positive zero Delta supports a real nonzero segment"),M::TryBoundedFixedProbeSample(c,0.,zero)&&zero.NativeSecondsBits==0);
        for(double invalid:{-0.,A::FromBits(1),std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()})TestFalse(TEXT("invalid actual Delta refused"),M::TryBoundedFixedProbeSample(c,invalid,zero));
        auto invalid=c;invalid.Leaves.Reset();TestFalse(TEXT("missing contact leaves refused"),M::TryBoundedFixedProbeSample(invalid,1.,zero));
        invalid=c;invalid.UpperTime=std::numeric_limits<double>::quiet_NaN();TestFalse(TEXT("nonfinite contact bracket refused"),M::TryBoundedFixedProbeSample(invalid,1.,zero));
        invalid=c;invalid.LowerTime=invalid.UpperTime=1.;TestFalse(TEXT("native float seconds overflow refused"),M::TryBoundedFixedProbeSample(invalid,std::numeric_limits<double>::max(),zero));
    }
    TestTrue(TEXT("numeric controls restored"),restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCanonicalEndpointWitnessTest,
    "IAmSpeed.AnalyticWorld.NativeSolid.CanonicalEndpointWitness",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCanonicalEndpointWitnessTest::RunTest(const FString&)
{
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE proof fixtures"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        C::FIntervalTensorSurface plane;plane.DegreeU=plane.DegreeV=1;
        for(const FVector3d p:{FVector3d(-2,-2,0),FVector3d(-2,2,0),FVector3d(2,-2,0),FVector3d(2,2,0)})plane.ControlPoints.Add(C::Detail::Point(p));
        auto far=plane;for(auto& point:far.ControlPoints)point.Axis[2]=I::Exact(10.);
        TArray<C::FIntervalTensorSurface> two{plane,far};const I r2=A::Square(I::Exact(.6));
        C::FDistanceBudget validated{128};const auto initial=C::TimeDetail::Evaluate(MakeArrayView(two),C::Detail::Point(FVector3d(0,0,2)),validated,r2,0.,2048);
        TestTrue(TEXT("initial separation checks the full valid union"),initial.Known&&initial.AllClear);
        C::FDistanceBudget tight{2};const auto inside=C::TimeDetail::Evaluate(MakeArrayView(two),C::Detail::Point(FVector3d::ZeroVector),tight,r2,0.,2048,true);
        TestTrue(TEXT("post-validation inside witness closes with its exact bounded work"),inside.Known&&inside.AnyInside&&!inside.AllClear&&inside.SurfaceIndex==0&&tight.Remaining==0);
        auto invalid=far;invalid.DegreeU=99;TArray<C::FIntervalTensorSurface> malformed{plane,invalid};C::FDistanceBudget bad{128};
        TestFalse(TEXT("default initialization never hides an invalid tail after inside"),C::TimeDetail::Evaluate(MakeArrayView(malformed),C::Detail::Point(FVector3d::ZeroVector),bad,r2,0.,2048).Known);
        C::FDistanceBudget empty{0};TestFalse(TEXT("coarse budget Unknown is not a recoverable miss"),C::TimeDetail::Evaluate(MakeArrayView(two),C::Detail::Point(FVector3d::ZeroVector),empty,r2,0.,1).Known);
        C::FIntervalBezierCenterPath path;if(!TestTrue(TEXT("fixed transverse polynomial"),C::CenterDetail::Linear(FVector3d(0,0,2),FVector3d(0,0,0),path)))return false;
        C::FDistanceBudget work{512};C::TimeDetail::FRetainedSphereTimeSearch search(MakeArrayView(&plane,1),path,.6,work,0.,2048);
        const auto first=search.Advance(1./64.),second=search.Advance(1./256.);
        TestTrue(TEXT("endpoint witness avoids an impossible whole-hull separation attempt"),first.Status==C::ETensorSphereTimeStatus::FirstContactBracket&&first.LowerTime<=.7&&first.UpperTime>=.7);
        TestTrue(TEXT("same retained frontier narrows inside its prior bracket"),second.Status==C::ETensorSphereTimeStatus::FirstContactBracket&&second.LowerTime>=first.LowerTime&&second.UpperTime<=first.UpperTime&&second.InitialSeparationProved);
        TestTrue(TEXT("single budget never refilled"),uint64(work.Remaining)+work.Used==512);
        // Both endpoints are outside. A quadratic path enters and leaves the
        // tube: the fallback whole-hull proof must not publish a false miss.
        C::FIntervalBezierCenterPath loop;loop.Degree=2;loop.ControlPoints={C::Detail::Point(FVector3d(0,0,2)),C::Detail::Point(FVector3d(0,0,-2)),C::Detail::Point(FVector3d(0,0,2))};
        C::FDistanceBudget curved{262144};const auto crossing=C::BoundTensorSphereSweep(MakeArrayView(&plane,1),loop,.6,curved,1./64.,0.,2048);
        TestTrue(TEXT("clear endpoints never discharge an interior crossing"),crossing.Status==C::ETensorSphereTimeStatus::FirstContactBracket&&crossing.LowerTime<.5&&crossing.InitialSeparationProved);
    }
    TestTrue(TEXT("numeric controls restored"),restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCanonicalNormalCellDiagnosticTest,
    "IAmSpeed.AnalyticWorld.NativeSolid.CanonicalNormalCellDiagnostics",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCanonicalNormalCellDiagnosticTest::RunTest(const FString&)
{
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE value fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        const auto up=C::Detail::Point(FVector3d(0,0,1));
        M::FUniqueQuantizedNormal value;M::FQuantizedNormalDiagnostic diagnostic;C::FDistanceBudget work{64};
        if(!TestTrue(TEXT("strict cell still agrees with actual native quantizer"),M::TryUniqueQuantizedNormal(up,FVector(.00001,0,1),work,value,&diagnostic)))return false;
        const FVector actual=Speed::QuantizeUnitNormal(FVector(.00001,0,1));
        TestTrue(TEXT("scalar outside continuous cone shares exact consumed discrete bits"),diagnostic.Failure==M::EQuantizedNormalFailure::None&&M::SameBits(actual,value.Quantized)&&M::SameBits(actual.GetSafeNormal(),value.ReNormalized));
        const double step=double(1.e-3f);auto ambiguous=up;
        ambiguous.Axis[0]={step*.49,step*.51,true};ambiguous.Axis[2]={.999,1.001,true};work={64};
        TestFalse(TEXT("crossing cell remains unqualified"),M::TryUniqueQuantizedNormal(ambiguous,FVector(step*.5,0,1),work,value,&diagnostic));
        TestTrue(TEXT("only valid open overlap is refinable"),diagnostic.Failure==M::EQuantizedNormalFailure::AmbiguousCell&&diagnostic.Axis==0&&diagnostic.Shifted.Upper>diagnostic.Integer&&diagnostic.Shifted.Lower<diagnostic.Next&&value.Quantized.IsZero()&&value.ReNormalized.IsZero());
        work={64};TestFalse(TEXT("scalar in another cell still refused"),M::TryUniqueQuantizedNormal(up,FVector(1,0,0),work,value,&diagnostic));
        TestTrue(TEXT("disjoint scalar cell is terminal"),diagnostic.Failure==M::EQuantizedNormalFailure::DisjointCell);
        for(int32 allowance=0;allowance<7;++allowance)
        {
            // Nonzero prior debit and a reserved floor must never be refilled.
            work={allowance+11,17,11};
            TestFalse(TEXT("every insufficient normal budget remains terminal"),M::TryUniqueQuantizedNormal(up,FVector(0,0,1),work,value,&diagnostic));
            TestTrue(TEXT("floor and previous debit preserved"),diagnostic.Failure==M::EQuantizedNormalFailure::Work&&work.Remaining==11&&work.Used==uint64(17+allowance)&&work.ConsumptionFloor==11&&value.Quantized.IsZero()&&value.ReNormalized.IsZero());
        }
        work={18,17,11};TestTrue(TEXT("seven original consumes suffice"),M::TryUniqueQuantizedNormal(up,FVector(0,0,1),work,value,&diagnostic)&&work.Remaining==11&&work.Used==24);
        auto invalid=up;invalid.Axis[1].Known=false;work={64};
        TestFalse(TEXT("unknown normal coordinate cannot request refinement"),M::TryUniqueQuantizedNormal(invalid,FVector(0,0,1),work,value,&diagnostic));
        TestTrue(TEXT("invalid bound classified terminal"),diagnostic.Failure==M::EQuantizedNormalFailure::RadialBounds);
        invalid=up;invalid.Axis[0]={A::FromBits(1),A::FromBits(1),true};work={64};
        TestFalse(TEXT("forged subnormal cannot become zero normal bound"),M::TryUniqueQuantizedNormal(invalid,FVector(0,0,1),work,value,&diagnostic));
        TestTrue(TEXT("raw subnormal terminal under every FTZ mode"),diagnostic.Failure==M::EQuantizedNormalFailure::RadialBounds);
        work={64};TestFalse(TEXT("zero scalar cannot become an up contact"),M::TryUniqueQuantizedNormal(up,FVector::ZeroVector,work,value,&diagnostic));
        TestTrue(TEXT("scalar normalization terminal"),diagnostic.Failure==M::EQuantizedNormalFailure::ScalarNormalize&&value.Quantized.IsZero());
        work={64};TestFalse(TEXT("nonfinite scalar terminal"),M::TryUniqueQuantizedNormal(up,FVector(std::numeric_limits<double>::quiet_NaN(),0,1),work,value,&diagnostic));
        TestTrue(TEXT("nonfinite input classified"),diagnostic.Failure==M::EQuantizedNormalFailure::Input);
    }
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCanonicalContactConsumerContinuationTest,
    "IAmSpeed.AnalyticWorld.NativeSolid.CanonicalContactConsumerContinuation",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCanonicalContactConsumerContinuationTest::RunTest(const FString&)
{
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE union fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        const uint32 currentControls=(controls&~uint32(0x8040))|flush;VectorSetControlRegister(currentControls);
        C::FIntervalTensorSurface plane;plane.DegreeU=plane.DegreeV=1;
        for(const FVector3d p:{FVector3d(-2,-2,0),FVector3d(-2,2,0),FVector3d(2,-2,0),FVector3d(2,2,0)})plane.ControlPoints.Add(C::Detail::Point(p));
        auto far=plane;for(auto& point:far.ControlPoints)point.Axis[2]=I::Exact(10.);
        const TArray<C::FIntervalTensorSurface> surfaces{far,plane,plane};C::FDistanceBudget build{256};C::FIndexedIntervalUnionPtr scene;
        if(!TestTrue(TEXT("complete union with excluded provider before two retained providers"),C::FIndexedIntervalUnion::Build(MakeArrayView(surfaces),build,scene)))return false;
        C::FReferenceCandidateSetPtr candidates;C::FDistanceBudget work{1048576,17,11};
        if(!TestTrue(TEXT("immutable full-world candidate partition"),SelectFixedSphereTraceCandidates(scene,FVector3d(0,0,2),FVector3d::ZeroVector,.6,work,candidates)&&candidates))return false;
        if(!TestTrue(TEXT("nonidentity local to global mapping"),candidates->GetCandidateIndices().Num()==2&&candidates->GetCandidateIndices()[0]==1&&candidates->GetCandidateIndices()[1]==2&&candidates->GetExcludedIndices().Num()==1))return false;
        int32 calls=0;double lower=0,upper=1;
        const auto requestOnce=[&](const C::FTensorSphereContactEnclosure& provisional)
        {
            ++calls;bool first=false,second=false;
            TestTrue(TEXT("provisional enclosure complete only within callback"),provisional.Status==C::ETensorSphereContactStatus::FirstContactEnclosed&&!provisional.Leaves.IsEmpty());
            TestTrue(TEXT("retained time brackets nested"),provisional.LowerTime>=lower&&provisional.UpperTime<=upper);
            TestTrue(TEXT("same first root remains enclosed"),provisional.LowerTime<=.7&&provisional.UpperTime>=.7);
            lower=provisional.LowerTime;upper=provisional.UpperTime;
            for(const auto& leaf:provisional.Leaves){first|=leaf.SurfaceIndex==1;second|=leaf.SurfaceIndex==2;TestTrue(TEXT("global provider identity retained"),leaf.SurfaceIndex==1||leaf.SurfaceIndex==2);}
            TestTrue(TEXT("no earlier closest provider discarded"),first&&second);
            return calls==1?C::EContactEnclosureDecision::Refine:C::EContactEnclosureDecision::Accept;
        };
        const C::FContactEnclosureFilter refineOnce(requestOnce);
        const auto contact=candidates->Contact(work,1./64.,0.,2048,.25,.25,24,&refineOnce);
        TestTrue(TEXT("same contact call continues to one accepted enclosure"),calls==2&&contact.Status==C::ETensorSphereContactStatus::FirstContactEnclosed&&contact.ConsumerWidthRefinements==1&&contact.TemporalWidthRefinements>=1&&!contact.Leaves.IsEmpty());
        TestTrue(TEXT("one budget includes candidate mapping and both callbacks"),uint64(work.Remaining)+work.Used==1048576+17&&work.Remaining>=11&&work.ConsumptionFloor==11);
        calls=0;
        const auto alwaysRefine=[&](const C::FTensorSphereContactEnclosure&){++calls;return C::EContactEnclosureDecision::Refine;};
        const C::FContactEnclosureFilter capped(alwaysRefine);work={1048576};
        const auto limit=candidates->Contact(work,1./64.,0.,2048,.25,.25,3,&capped);
        TestTrue(TEXT("one global limit covers consumer and width refinements"),calls<=3&&calls>0&&limit.Status==C::ETensorSphereContactStatus::Unknown&&limit.Failure==C::ETensorSphereContactFailure::WidthRefinementLimit&&limit.Leaves.IsEmpty()&&limit.TemporalWidthRefinements<3&&limit.ConsumerWidthRefinements<=limit.TemporalWidthRefinements);
        calls=0;const auto reject=[&](const C::FTensorSphereContactEnclosure&){++calls;return C::EContactEnclosureDecision::Refuse;};
        const C::FContactEnclosureFilter terminal(reject);work={1048576};
        const auto refused=candidates->Contact(work,1./64.,0.,2048,.25,.25,24,&terminal);
        TestTrue(TEXT("refusal is terminal after one provisional enclosure"),calls==1&&refused.Status==C::ETensorSphereContactStatus::Unknown&&refused.Failure==C::ETensorSphereContactFailure::ConsumerRefusal&&refused.Leaves.IsEmpty());
        calls=0;work={1048576,17,11};
        const auto consumeRemaining=[&](const C::FTensorSphereContactEnclosure&){++calls;while(work.Consume()){}return C::EContactEnclosureDecision::Refine;};
        const C::FContactEnclosureFilter exhausted(consumeRemaining);
        const auto noWork=candidates->Contact(work,1./64.,0.,2048,.25,.25,24,&exhausted);
        TestTrue(TEXT("failed continuation cannot publish stale provisional leaves"),calls==1&&noWork.Status==C::ETensorSphereContactStatus::Unknown&&noWork.Failure==C::ETensorSphereContactFailure::Work&&noWork.Leaves.IsEmpty()&&work.Remaining==11&&work.Used==1048576+17-11);
        calls=0;work={1048576};
        const auto changeControl=[&](const C::FTensorSphereContactEnclosure&){++calls;VectorSetControlRegister(currentControls^uint32(0x40));return C::EContactEnclosureDecision::Accept;};
        const C::FContactEnclosureFilter numeric(changeControl);
        const auto changed=candidates->Contact(work,1./64.,0.,2048,.25,.25,24,&numeric);VectorSetControlRegister(currentControls);
        TestTrue(TEXT("changed numeric fingerprint cannot commit contact"),calls==1&&changed.Status==C::ETensorSphereContactStatus::Unknown&&changed.Failure==C::ETensorSphereContactFailure::NumericContext&&changed.Leaves.IsEmpty());
    }
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}
namespace
{
// Actual failed790 query's two retained providers ONLY (118,1118), in original
// order. This subset fixture never grants complete2992world or physical PASS.
const uint64 CapturedNormal790CP[2][216]={
    {
        0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AE000000000000ULL,0x40AE000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,
        0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AE28365EED39E1ULL,0x40AE28365EED39E2ULL,0x0000000000000000ULL,0x0000000000000000ULL,
        0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AE506CBDDA73C3ULL,0x40AE506CBDDA73C4ULL,0x0000000000000000ULL,0x0000000000000000ULL,
        0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AE781ECECD30B4ULL,0x40AE781ECECD30B5ULL,0x4017B0042A803897ULL,0x4017B0042A803898ULL,
        0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AE9EC8530920ACULL,0x40AE9EC8530920ADULL,0x402795C6D9C027BCULL,0x402795C6D9C027BDULL,
        0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AEC3EF1535754BULL,0x40AEC3EF1535754CULL,0x40337CA1866B95CEULL,0x40337CA1866B95CFULL,
        0xC0A67FDD7D3ED05FULL,0xC0A67FDD7D3ED05EULL,0x40AE000000000000ULL,0x40AE000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,
        0xC0A67FDD7D3ED05FULL,0xC0A67FDD7D3ED05EULL,0x40AE28365EED39E1ULL,0x40AE28365EED39E2ULL,0x0000000000000000ULL,0x0000000000000000ULL,
        0xC0A67FDD7D3ED05FULL,0xC0A67FDD7D3ED05EULL,0x40AE506CBDDA73C3ULL,0x40AE506CBDDA73C4ULL,0x0000000000000000ULL,0x0000000000000000ULL,
        0xC0A67FDD7D3ED05FULL,0xC0A67FDD7D3ED05EULL,0x40AE781ECECD30B4ULL,0x40AE781ECECD30B5ULL,0x4017B0042A803897ULL,0x4017B0042A803898ULL,
        0xC0A67FDD7D3ED05FULL,0xC0A67FDD7D3ED05EULL,0x40AE9EC8530920ACULL,0x40AE9EC8530920ADULL,0x402795C6D9C027BCULL,0x402795C6D9C027BDULL,
        0xC0A67FDD7D3ED05FULL,0xC0A67FDD7D3ED05EULL,0x40AEC3EF1535754BULL,0x40AEC3EF1535754CULL,0x40337CA1866B95CEULL,0x40337CA1866B95CFULL,
        0xC0A0DFE61DEF1C47ULL,0xC0A0DFE61DEF1C46ULL,0x40AE000000000000ULL,0x40AE000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,
        0xC0A0DFE61DEF1C47ULL,0xC0A0DFE61DEF1C46ULL,0x40AE28365EED39E1ULL,0x40AE28365EED39E2ULL,0x0000000000000000ULL,0x0000000000000000ULL,
        0xC0A0DFE61DEF1C47ULL,0xC0A0DFE61DEF1C46ULL,0x40AE506CBDDA73C3ULL,0x40AE506CBDDA73C4ULL,0x0000000000000000ULL,0x0000000000000000ULL,
        0xC0A0DFE61DEF1C47ULL,0xC0A0DFE61DEF1C46ULL,0x40AE781ECECD30B4ULL,0x40AE781ECECD30B5ULL,0x4017B0042A803897ULL,0x4017B0042A803898ULL,
        0xC0A0DFE61DEF1C47ULL,0xC0A0DFE61DEF1C46ULL,0x40AE9EC8530920ACULL,0x40AE9EC8530920ADULL,0x402795C6D9C027BCULL,0x402795C6D9C027BDULL,
        0xC0A0DFE61DEF1C47ULL,0xC0A0DFE61DEF1C46ULL,0x40AEC3EF1535754BULL,0x40AEC3EF1535754CULL,0x40337CA1866B95CEULL,0x40337CA1866B95CFULL,
        0xC0967FDD7D3ED05FULL,0xC0967FDD7D3ED05EULL,0x40AE000000000000ULL,0x40AE000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,
        0xC0967FDD7D3ED05FULL,0xC0967FDD7D3ED05EULL,0x40AE28365EED39E1ULL,0x40AE28365EED39E2ULL,0x0000000000000000ULL,0x0000000000000000ULL,
        0xC0967FDD7D3ED05FULL,0xC0967FDD7D3ED05EULL,0x40AE506CBDDA73C3ULL,0x40AE506CBDDA73C4ULL,0x0000000000000000ULL,0x0000000000000000ULL,
        0xC0967FDD7D3ED05FULL,0xC0967FDD7D3ED05EULL,0x40AE781ECECD30B4ULL,0x40AE781ECECD30B5ULL,0x4017B0042A803897ULL,0x4017B0042A803898ULL,
        0xC0967FDD7D3ED05FULL,0xC0967FDD7D3ED05EULL,0x40AE9EC8530920ACULL,0x40AE9EC8530920ADULL,0x402795C6D9C027BCULL,0x402795C6D9C027BDULL,
        0xC0967FDD7D3ED05FULL,0xC0967FDD7D3ED05EULL,0x40AEC3EF1535754BULL,0x40AEC3EF1535754CULL,0x40337CA1866B95CEULL,0x40337CA1866B95CFULL,
        0xC0867FDD7D3ED05FULL,0xC0867FDD7D3ED05EULL,0x40AE000000000000ULL,0x40AE000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,
        0xC0867FDD7D3ED05FULL,0xC0867FDD7D3ED05EULL,0x40AE28365EED39E1ULL,0x40AE28365EED39E2ULL,0x0000000000000000ULL,0x0000000000000000ULL,
        0xC0867FDD7D3ED05FULL,0xC0867FDD7D3ED05EULL,0x40AE506CBDDA73C3ULL,0x40AE506CBDDA73C4ULL,0x0000000000000000ULL,0x0000000000000000ULL,
        0xC0867FDD7D3ED05FULL,0xC0867FDD7D3ED05EULL,0x40AE781ECECD30B4ULL,0x40AE781ECECD30B5ULL,0x4017B0042A803897ULL,0x4017B0042A803898ULL,
        0xC0867FDD7D3ED05FULL,0xC0867FDD7D3ED05EULL,0x40AE9EC8530920ACULL,0x40AE9EC8530920ADULL,0x402795C6D9C027BCULL,0x402795C6D9C027BDULL,
        0xC0867FDD7D3ED05FULL,0xC0867FDD7D3ED05EULL,0x40AEC3EF1535754BULL,0x40AEC3EF1535754CULL,0x40337CA1866B95CEULL,0x40337CA1866B95CFULL,
        0x0000000000000000ULL,0x0000000000000000ULL,0x40AE000000000000ULL,0x40AE000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,
        0x0000000000000000ULL,0x0000000000000000ULL,0x40AE28365EED39E1ULL,0x40AE28365EED39E2ULL,0x0000000000000000ULL,0x0000000000000000ULL,
        0x0000000000000000ULL,0x0000000000000000ULL,0x40AE506CBDDA73C3ULL,0x40AE506CBDDA73C4ULL,0x0000000000000000ULL,0x0000000000000000ULL,
        0x0000000000000000ULL,0x0000000000000000ULL,0x40AE781ECECD30B4ULL,0x40AE781ECECD30B5ULL,0x4017B0042A803897ULL,0x4017B0042A803898ULL,
        0x0000000000000000ULL,0x0000000000000000ULL,0x40AE9EC8530920ACULL,0x40AE9EC8530920ADULL,0x402795C6D9C027BCULL,0x402795C6D9C027BDULL,
        0x0000000000000000ULL,0x0000000000000000ULL,0x40AEC3EF1535754BULL,0x40AEC3EF1535754CULL,0x40337CA1866B95CEULL,0x40337CA1866B95CFULL
    },
    {
        0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AEC3EF1535754BULL,0x40AEC3EF1535754CULL,0x40337CA1866B95CEULL,0x40337CA1866B95CFULL,
        0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AEE915D761C9E9ULL,0x40AEE915D761C9EAULL,0x403B2E5F9FF717BFULL,0x403B2E5F9FF717C0ULL,
        0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AF0CB9D77E832FULL,0x40AF0CB9D77E8330ULL,0x4042597CB8670AF3ULL,0x4042597CB8670AF4ULL,
        0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AF2E60D35523D4ULL,0x40AF2E60D35523D5ULL,0x4047F892FFBB3959ULL,0x4047F892FFBB395AULL,
        0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AF4D9AAE3CF7EBULL,0x40AF4D9AAE3CF7ECULL,0x404E61B85B441C72ULL,0x404E61B85B441C73ULL,
        0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AF6A09E667F3BCULL,0x40AF6A09E667F3BDULL,0x4052BEC333018866ULL,0x4052BEC333018867ULL,
        0xC0A67FDD7D3ED05FULL,0xC0A67FDD7D3ED05EULL,0x40AEC3EF1535754BULL,0x40AEC3EF1535754CULL,0x40337CA1866B95CEULL,0x40337CA1866B95CFULL,
        0xC0A67FDD7D3ED05FULL,0xC0A67FDD7D3ED05EULL,0x40AEE915D761C9E9ULL,0x40AEE915D761C9EAULL,0x403B2E5F9FF717BFULL,0x403B2E5F9FF717C0ULL,
        0xC0A67FDD7D3ED05FULL,0xC0A67FDD7D3ED05EULL,0x40AF0CB9D77E832FULL,0x40AF0CB9D77E8330ULL,0x4042597CB8670AF3ULL,0x4042597CB8670AF4ULL,
        0xC0A67FDD7D3ED05FULL,0xC0A67FDD7D3ED05EULL,0x40AF2E60D35523D4ULL,0x40AF2E60D35523D5ULL,0x4047F892FFBB3959ULL,0x4047F892FFBB395AULL,
        0xC0A67FDD7D3ED05FULL,0xC0A67FDD7D3ED05EULL,0x40AF4D9AAE3CF7EBULL,0x40AF4D9AAE3CF7ECULL,0x404E61B85B441C72ULL,0x404E61B85B441C73ULL,
        0xC0A67FDD7D3ED05FULL,0xC0A67FDD7D3ED05EULL,0x40AF6A09E667F3BCULL,0x40AF6A09E667F3BDULL,0x4052BEC333018866ULL,0x4052BEC333018867ULL,
        0xC0A0DFE61DEF1C47ULL,0xC0A0DFE61DEF1C46ULL,0x40AEC3EF1535754BULL,0x40AEC3EF1535754CULL,0x40337CA1866B95CEULL,0x40337CA1866B95CFULL,
        0xC0A0DFE61DEF1C47ULL,0xC0A0DFE61DEF1C46ULL,0x40AEE915D761C9E9ULL,0x40AEE915D761C9EAULL,0x403B2E5F9FF717BFULL,0x403B2E5F9FF717C0ULL,
        0xC0A0DFE61DEF1C47ULL,0xC0A0DFE61DEF1C46ULL,0x40AF0CB9D77E832FULL,0x40AF0CB9D77E8330ULL,0x4042597CB8670AF3ULL,0x4042597CB8670AF4ULL,
        0xC0A0DFE61DEF1C47ULL,0xC0A0DFE61DEF1C46ULL,0x40AF2E60D35523D4ULL,0x40AF2E60D35523D5ULL,0x4047F892FFBB3959ULL,0x4047F892FFBB395AULL,
        0xC0A0DFE61DEF1C47ULL,0xC0A0DFE61DEF1C46ULL,0x40AF4D9AAE3CF7EBULL,0x40AF4D9AAE3CF7ECULL,0x404E61B85B441C72ULL,0x404E61B85B441C73ULL,
        0xC0A0DFE61DEF1C47ULL,0xC0A0DFE61DEF1C46ULL,0x40AF6A09E667F3BCULL,0x40AF6A09E667F3BDULL,0x4052BEC333018866ULL,0x4052BEC333018867ULL,
        0xC0967FDD7D3ED05FULL,0xC0967FDD7D3ED05EULL,0x40AEC3EF1535754BULL,0x40AEC3EF1535754CULL,0x40337CA1866B95CEULL,0x40337CA1866B95CFULL,
        0xC0967FDD7D3ED05FULL,0xC0967FDD7D3ED05EULL,0x40AEE915D761C9E9ULL,0x40AEE915D761C9EAULL,0x403B2E5F9FF717BFULL,0x403B2E5F9FF717C0ULL,
        0xC0967FDD7D3ED05FULL,0xC0967FDD7D3ED05EULL,0x40AF0CB9D77E832FULL,0x40AF0CB9D77E8330ULL,0x4042597CB8670AF3ULL,0x4042597CB8670AF4ULL,
        0xC0967FDD7D3ED05FULL,0xC0967FDD7D3ED05EULL,0x40AF2E60D35523D4ULL,0x40AF2E60D35523D5ULL,0x4047F892FFBB3959ULL,0x4047F892FFBB395AULL,
        0xC0967FDD7D3ED05FULL,0xC0967FDD7D3ED05EULL,0x40AF4D9AAE3CF7EBULL,0x40AF4D9AAE3CF7ECULL,0x404E61B85B441C72ULL,0x404E61B85B441C73ULL,
        0xC0967FDD7D3ED05FULL,0xC0967FDD7D3ED05EULL,0x40AF6A09E667F3BCULL,0x40AF6A09E667F3BDULL,0x4052BEC333018866ULL,0x4052BEC333018867ULL,
        0xC0867FDD7D3ED05FULL,0xC0867FDD7D3ED05EULL,0x40AEC3EF1535754BULL,0x40AEC3EF1535754CULL,0x40337CA1866B95CEULL,0x40337CA1866B95CFULL,
        0xC0867FDD7D3ED05FULL,0xC0867FDD7D3ED05EULL,0x40AEE915D761C9E9ULL,0x40AEE915D761C9EAULL,0x403B2E5F9FF717BFULL,0x403B2E5F9FF717C0ULL,
        0xC0867FDD7D3ED05FULL,0xC0867FDD7D3ED05EULL,0x40AF0CB9D77E832FULL,0x40AF0CB9D77E8330ULL,0x4042597CB8670AF3ULL,0x4042597CB8670AF4ULL,
        0xC0867FDD7D3ED05FULL,0xC0867FDD7D3ED05EULL,0x40AF2E60D35523D4ULL,0x40AF2E60D35523D5ULL,0x4047F892FFBB3959ULL,0x4047F892FFBB395AULL,
        0xC0867FDD7D3ED05FULL,0xC0867FDD7D3ED05EULL,0x40AF4D9AAE3CF7EBULL,0x40AF4D9AAE3CF7ECULL,0x404E61B85B441C72ULL,0x404E61B85B441C73ULL,
        0xC0867FDD7D3ED05FULL,0xC0867FDD7D3ED05EULL,0x40AF6A09E667F3BCULL,0x40AF6A09E667F3BDULL,0x4052BEC333018866ULL,0x4052BEC333018867ULL,
        0x0000000000000000ULL,0x0000000000000000ULL,0x40AEC3EF1535754BULL,0x40AEC3EF1535754CULL,0x40337CA1866B95CEULL,0x40337CA1866B95CFULL,
        0x0000000000000000ULL,0x0000000000000000ULL,0x40AEE915D761C9E9ULL,0x40AEE915D761C9EAULL,0x403B2E5F9FF717BFULL,0x403B2E5F9FF717C0ULL,
        0x0000000000000000ULL,0x0000000000000000ULL,0x40AF0CB9D77E832FULL,0x40AF0CB9D77E8330ULL,0x4042597CB8670AF3ULL,0x4042597CB8670AF4ULL,
        0x0000000000000000ULL,0x0000000000000000ULL,0x40AF2E60D35523D4ULL,0x40AF2E60D35523D5ULL,0x4047F892FFBB3959ULL,0x4047F892FFBB395AULL,
        0x0000000000000000ULL,0x0000000000000000ULL,0x40AF4D9AAE3CF7EBULL,0x40AF4D9AAE3CF7ECULL,0x404E61B85B441C72ULL,0x404E61B85B441C73ULL,
        0x0000000000000000ULL,0x0000000000000000ULL,0x40AF6A09E667F3BCULL,0x40AF6A09E667F3BDULL,0x4052BEC333018866ULL,0x4052BEC333018867ULL
    }
};
TArray<C::FIntervalTensorSurface> CapturedNormal790Surfaces()
{
    TArray<C::FIntervalTensorSurface> surfaces;
    for(int32 provider=0;provider<2;++provider)
    {
        C::FIntervalTensorSurface surface;surface.DegreeU=surface.DegreeV=5;
        for(int32 cp=0;cp<36;++cp)
        {
            C::FIntervalPoint point;
            for(int32 d=0;d<3;++d)point.Axis[d]={A::FromBits(CapturedNormal790CP[provider][cp*6+d*2]),A::FromBits(CapturedNormal790CP[provider][cp*6+d*2+1]),true};
            surface.ControlPoints.Add(point);
        }
        surfaces.Add(MoveTemp(surface));
    }
    return surfaces;
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCanonicalCapturedNormal790Test,
    "IAmSpeed.AnalyticWorld.NativeSolid.CanonicalCapturedGutterNormal790",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCanonicalCapturedNormal790Test::RunTest(const FString&)
{
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE captured arithmetic fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        // Diagnostic unit stages only. Integer-only inspection comes before
        // logging; Unknown is neither formatted nor repaired. No clock, work
        // allowance, search result or production callback is changed here.
        const auto stage=[&](const TCHAR* name,int32 pass,uint64 gapBits,
            const C::FDistanceBudget& account,int32 anchors,uint32 status=0,uint32 failure=0)
        {
            if(Speed::Analytic::InspectCurrentAnalyticNumericContext(Speed::Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).Status!=Speed::Analytic::EAnalyticNumericStatus::Complete)return;
            UE_LOG(LogTemp,Display,TEXT("[Canonical790MathStage] Stage=%s FTZ=%u Pass=%d GapBits=%016llX Used=%llu Remaining=%d AnchorRefinements=%d Status=%u Failure=%u"),
                name,flush,pass,gapBits,account.Used,account.Remaining,anchors,status,failure);
        };
        auto surfaces=CapturedNormal790Surfaces();C::FDistanceBudget build{4096};C::FIndexedIntervalUnionPtr scene;
        if(!TestTrue(TEXT("exact captured two-provider CP subset"),C::FIndexedIntervalUnion::Build(MakeArrayView(surfaces),build,scene)))return false;
        const FVector3d start(-27.785273718823138,3934.6565340488623,38.635603602631974),end(-27.788858917536604,3938.4765034164861,14.931428669301077);
        const double radius=12.510000228881836,physical=12.5,delta=.0033333334140479565;
        const I slack=A::Subtract(I::Exact(radius),I::Exact(physical));const double pointWidth=slack.Lower*.125,normalWidth=double(1.e-3f)*.125;
        double timeGap=0;if(!TestTrue(TEXT("same captured accuracy law"),M::TryFixedProbeSpatialTimeGap(start,end,pointWidth,normalWidth,radius,timeGap)))return false;
        C::FDistanceBudget work{1048576};C::FReferenceCandidateSetPtr candidates;
        if(!TestTrue(TEXT("both original candidates remain in this subset"),SelectFixedSphereTraceCandidates(scene,start,end,radius,work,candidates)&&candidates&&candidates->GetCandidateIndices().Num()==2))return false;
        M::FUniqueQuantizedNormal normal;C::FScalarSphereContact scalar;int32 callbacks=0;
        const auto check=[&](const C::FTensorSphereContactEnclosure& contact)
        {
            ++callbacks;stage(TEXT("CapturedConsumerBefore"),callbacks,A::Bits(timeGap),work,0);
            M::FBoundedFixedProbeSample sample;
            if(contact.Leaves.IsEmpty()||!M::TryBoundedFixedProbeSample(contact,delta,sample))return C::EContactEnclosureDecision::Refuse;
            for(const auto& leaf:contact.Leaves)
            {
                C::FIntervalPoint chart;
                if(leaf.SurfaceIndex!=1||!M::ProveRegularFaceLeaf(scene,leaf,work,chart))return C::EContactEnclosureDecision::Refuse;
            }
            C::FIntervalPoint direction;for(int32 d=0;d<3;++d)direction.Axis[d]=A::Subtract(I::Exact(end[d]),I::Exact(start[d]));
            const I approach=C::ContactDetail::Dot(direction,contact.RadialUnitNormal);
            if(!A::Valid(approach)||!(approach.Upper<0))return C::EContactEnclosureDecision::Refuse;
            FVector3d point;for(int32 d=0;d<3;++d)point[d]=contact.Leaves[0].FixedParameterPoint.Axis[d].Lower;
            if(C::ConvertSphereContact(contact,0,point,start+(end-start)*sample.NormalizedRepresentative,radius,physical,scalar)!=C::EScalarSphereContactStatus::Bounded)return C::EContactEnclosureDecision::Refuse;
            const I error=A::Add(I::Exact(scalar.LocationErrorCm),I::Exact(scalar.SurfaceErrorCm));
            if(!A::Valid(error)||error.Upper>slack.Lower)return C::EContactEnclosureDecision::Refuse;
            M::FQuantizedNormalDiagnostic diagnostic;
            const bool unique=M::TryUniqueQuantizedNormal(contact.RadialUnitNormal,scalar.NormalStored,work,normal,&diagnostic);
            stage(TEXT("CapturedNormalAfter"),callbacks,A::Bits(timeGap),work,0,unique?1u:0u,uint32(diagnostic.Failure));
            if(unique)return C::EContactEnclosureDecision::Accept;
            return diagnostic.Failure==M::EQuantizedNormalFailure::AmbiguousCell?C::EContactEnclosureDecision::Refine:C::EContactEnclosureDecision::Refuse;
        };
        const C::FContactEnclosureFilter filter(check);
        stage(TEXT("CapturedContactBefore"),-1,A::Bits(timeGap),work,0);
        const auto contact=candidates->Contact(work,timeGap,0.,8192,pointWidth,normalWidth,24,&filter);
        stage(TEXT("CapturedContactAfter"),-1,A::Bits(timeGap),work,0,uint32(contact.Status),uint32(contact.Failure));
        AddInfo(FString::Printf(TEXT("Captured790 subset only FTZ=%u Status=%u Failure=%u TimeFailure=%u Temporal=%d Consumer=%d Calls=%d Used=%llu Remaining=%d"),flush,uint32(contact.Status),uint32(contact.Failure),uint32(contact.TimeFailure),contact.TemporalWidthRefinements,contact.ConsumerWidthRefinements,callbacks,work.Used,work.Remaining));
        AddInfo(FString::Printf(TEXT("Captured790 time exit FTZ=%u DyadicSite=%u Depth=%d A=%.17g B=%.17g ActualAnchorRefinements=%d LastWidth=%u Pass=%d Surface=%d DepthU=%d DepthV=%d U=[%.17g,%.17g] V=[%.17g,%.17g] T=[%.17g,%.17g] PointTarget=%.17g NormalTarget=%.17g Gu=[%.17g,%.17g] Gv=[%.17g,%.17g]"),
            flush,uint32(contact.TimeDyadicSite),contact.TimeDyadicDepth,contact.TimeDyadicA,contact.TimeDyadicB,contact.TimeAnchorRefinements,uint32(contact.HasWidthDiagnostic),contact.WidthRefinement,contact.WidthSurface,contact.WidthDepthU,contact.WidthDepthV,contact.WidthMinU,contact.WidthMaxU,contact.WidthMinV,contact.WidthMaxV,contact.WidthTimeLower,contact.WidthTimeUpper,contact.WidthPointTarget,contact.WidthNormalTarget,contact.WidthLastGu.Lower,contact.WidthLastGu.Upper,contact.WidthLastGv.Lower,contact.WidthLastGv.Upper));
        if(contact.HasWidthDiagnostic)for(int32 d=0;d<3;++d)
            AddInfo(FString::Printf(TEXT("Captured790 last width FTZ=%u Axis=%d Point=[%.17g,%.17g] Normal=[%.17g,%.17g] Du=[%.17g,%.17g] Dv=[%.17g,%.17g]"),flush,d,contact.WidthPointUnion.Axis[d].Lower,contact.WidthPointUnion.Axis[d].Upper,contact.WidthNormalUnion.Axis[d].Lower,contact.WidthNormalUnion.Axis[d].Upper,contact.WidthLastDu.Axis[d].Lower,contact.WidthLastDu.Axis[d].Upper,contact.WidthLastDv.Axis[d].Lower,contact.WidthLastDv.Axis[d].Upper));
        TestTrue(TEXT("captured normal closes within unchanged global work and cells"),contact.Status==C::ETensorSphereContactStatus::FirstContactEnclosed&&!contact.Leaves.IsEmpty()&&callbacks>0&&!normal.Quantized.IsZero()&&M::SameBits(normal.Quantized,Speed::QuantizeUnitNormal(scalar.NormalStored)));
        TestTrue(TEXT("shared work never refilled"),uint64(work.Remaining)+work.Used==1048576);
    }
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCanonicalAnchorFrontierTest,
    "IAmSpeed.AnalyticWorld.NativeSolid.CanonicalAnchorFrontierRefinement",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCanonicalAnchorFrontierTest::RunTest(const FString&)
{
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE retained-anchor fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        // Diagnostic unit stages only. Integer-only inspection comes before
        // logging; Unknown is neither formatted nor repaired. No clock, work
        // allowance, search result or production callback is changed here.
        const auto stage=[&](const TCHAR* name,int32 pass,uint64 gapBits,
            const C::FDistanceBudget& account,int32 anchors,uint32 status=0,uint32 failure=0)
        {
            if(Speed::Analytic::InspectCurrentAnalyticNumericContext(Speed::Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).Status!=Speed::Analytic::EAnalyticNumericStatus::Complete)return;
            UE_LOG(LogTemp,Display,TEXT("[Canonical790MathStage] Stage=%s FTZ=%u Pass=%d GapBits=%016llX Used=%llu Remaining=%d AnchorRefinements=%d Status=%u Failure=%u"),
                name,flush,pass,gapBits,account.Used,account.Remaining,anchors,status,failure);
        };
        auto surfaces=CapturedNormal790Surfaces();C::FIntervalBezierCenterPath path;
        const FVector3d start(-27.785273718823138,3934.6565340488623,38.635603602631974),end(-27.788858917536604,3938.4765034164861,14.931428669301077);
        if(!TestTrue(TEXT("same fixed captured reference trace"),C::CenterDetail::Linear(start,end,path)))return false;
        C::FDistanceBudget work{1048576};C::TimeDetail::FRetainedSphereTimeSearch search(MakeArrayView(surfaces),path,12.510000228881836,work,0.,8192);
        double gap=2.8381123165383108e-06,lower=0,upper=1;
        int32 provedBrackets=0;
        for(int32 refinement=0;refinement<4;++refinement)
        {
            stage(TEXT("AnchorTimeBefore"),refinement,A::Bits(gap),work,search.GetAnchorRefinementsForInspection());
            const auto time=search.Advance(gap);
            stage(TEXT("AnchorTimeAfter"),refinement,A::Bits(gap),work,search.GetAnchorRefinementsForInspection(),uint32(time.Status),uint32(search.GetFailureForInspection()));
            if(Speed::Analytic::InspectCurrentAnalyticNumericContext(Speed::Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).Status==Speed::Analytic::EAnalyticNumericStatus::Complete)
            {
                const auto& counts=search.GetSpatialCountersForInspection();
                UE_LOG(LogTemp,Display,TEXT("[Canonical790SpatialCounters] FTZ=%u Pass=%d NewRoots=%llu ReusedLeaves=%llu DirtyBounds=%llu KKTExcluded=%llu UVSplits=%llu LocalStops=%llu PeakHeapLeaves=%d"),
                    flush,refinement,counts.NewRoots,counts.ReusedLeaves,counts.DirtyBounds,counts.KKTExcluded,counts.UVSplits,counts.LocalStops,counts.PeakHeapLeaves);
            }
            AddInfo(FString::Printf(TEXT("Captured790 time only FTZ=%u Pass=%d Status=%u Failure=%u AnchorRefinements=%d Used=%llu Remaining=%d"),flush,refinement,uint32(time.Status),uint32(search.GetFailureForInspection()),search.GetAnchorRefinementsForInspection(),work.Used,work.Remaining));
            if(!TestTrue(TEXT("all four requested brackets close without dropping an unresolved prefix"),time.Status==C::ETensorSphereTimeStatus::FirstContactBracket&&time.InitialSeparationProved&&time.LowerTime>=lower&&time.UpperTime<=upper&&A::Valid(A::Subtract(I::Exact(time.UpperTime),I::Exact(time.LowerTime)))&&A::Subtract(I::Exact(time.UpperTime),I::Exact(time.LowerTime)).Upper<=gap))break;
            ++provedBrackets;
            lower=time.LowerTime;upper=time.UpperTime;gap*=.25;
        }
        // A stronger universal lower bound can discharge the positive prefix
        // directly. The exact tangent fixture below separately requires actual
        // anchor revisits and refuses skipping an earlier unresolved contact.
        TestEqual(TEXT("all four requested captured brackets were proved"),provedBrackets,4);
        TestTrue(TEXT("same time work account"),uint64(work.Remaining)+work.Used==1048576);
        C::FIntervalTensorSurface plane;plane.DegreeU=plane.DegreeV=1;
        for(const FVector3d p:{FVector3d(-10,-10,0),FVector3d(-10,10,0),FVector3d(10,-10,0),FVector3d(10,10,0)})plane.ControlPoints.Add(C::Detail::Point(p));
        // Valid quadratic fixed curve x=4t,z=3+4(t-1/4)^2: EARLIER
        // tangency to z=0 at1/4, then entry into the x=6 plane tube after3/4.
        // Exact dyadic CPs; a synthetic mathematical union, no actor pose.
        C::FIntervalTensorSurface wall;wall.DegreeU=wall.DegreeV=1;
        for(const FVector3d p:{FVector3d(6,-10,-10),FVector3d(6,-10,10),FVector3d(6,10,-10),FVector3d(6,10,10)})wall.ControlPoints.Add(C::Detail::Point(p));
        const TArray<C::FIntervalTensorSurface> twoPlanes{plane,wall};
        C::FIntervalBezierCenterPath tangent;tangent.Degree=2;
        for(const FVector3d p:{FVector3d(0,0,3.25),FVector3d(2,0,2.25),FVector3d(4,0,5.25)})tangent.ControlPoints.Add(C::Detail::Point(p));
        if(!TestTrue(TEXT("tangent fixture is an admitted degree2 curve"),C::CenterDetail::Valid(tangent)))return false;
        C::FDistanceBudget bounded{65536};C::TimeDetail::FRetainedSphereTimeSearch ambiguous(MakeArrayView(twoPlanes),tangent,3.,bounded,0.,256);
        stage(TEXT("EarlierTangentBefore"),-1,A::Bits(1./128.),bounded,ambiguous.GetAnchorRefinementsForInspection());
        const auto refused=ambiguous.Advance(1./128.);
        stage(TEXT("EarlierTangentAfter"),-1,A::Bits(1./128.),bounded,ambiguous.GetAnchorRefinementsForInspection(),uint32(refused.Status),uint32(ambiguous.GetFailureForInspection()));
        TestTrue(TEXT("earlier tangency is not skipped for a later inside witness"),refused.Status==C::ETensorSphereTimeStatus::Unknown&&ambiguous.GetAnchorRefinementsForInspection()>0);
        TestTrue(TEXT("ambiguous prefix remains bounded work"),uint64(bounded.Remaining)+bounded.Used==65536);
    }
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}

namespace
{
// Synthetic mathematical plane only: no native pose/profile/contact admission.
C::FIntervalTensorSurface MakeRetainedMinimumTestPlane47()
{
    C::FIntervalTensorSurface surface;surface.DegreeU=surface.DegreeV=1;
    for(const FVector3d point:{FVector3d(-4,-4,0),FVector3d(-4,4,0),FVector3d(4,-4,0),FVector3d(4,4,0)})
        surface.ControlPoints.Add(C::Detail::Point(point));
    return surface;
}
bool SameRetainedMinimumBounds47(const C::FTensorDistanceBounds& a,const C::FTensorDistanceBounds& b)
{
    if(a.Status!=b.Status||A::Bits(a.LowerSquared)!=A::Bits(b.LowerSquared)||A::Bits(a.UpperSquared)!=A::Bits(b.UpperSquared)||
        A::Bits(a.SampleUV.X)!=A::Bits(b.SampleUV.X)||A::Bits(a.SampleUV.Y)!=A::Bits(b.SampleUV.Y))return false;
    for(int32 d=0;d<3;++d)
        if(a.SamplePoint.Axis[d].Known!=b.SamplePoint.Axis[d].Known||
            A::Bits(a.SamplePoint.Axis[d].Lower)!=A::Bits(b.SamplePoint.Axis[d].Lower)||
            A::Bits(a.SamplePoint.Axis[d].Upper)!=A::Bits(b.SamplePoint.Axis[d].Upper))return false;
    return true;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRetainedMinimumParity47Test,
    "IAmSpeed.AnalyticWorld.TensorDistance.RetainedMinimumParity",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FRetainedMinimumParity47Test::RunTest(const FString&)
{
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE retained spatial test"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);

        TArray<C::FIntervalTensorSurface> surfaces;surfaces.Add(MakeRetainedMinimumTestPlane47());
        const auto captured=CapturedNormal790Surfaces();surfaces.Add(captured[1]);
        for(int32 index=0;index<surfaces.Num();++index)for(int32 cap:{1,64,512})
        {
            const auto center=C::Detail::Point(index?FVector3d(-27.785273718823138,3934.6565340488623,38.635603602631974):FVector3d(.37,.29,3.5));
            C::FDistanceBudget legacyWork{65536},retainedWork{65536};
            const auto legacy=C::BoundTensorDistanceSquaredIntervalCenter(surfaces[index],center,legacyWork,0.,cap);
            C::FRetainedTensorMinimum retained(surfaces[index],center,retainedWork,0.,cap);
            const auto actual=retained.Advance();
            TestTrue(TEXT("fresh retained operation preserves legacy bounds, UV, witness and work"),
                legacy.Status!=C::EDistanceBoundStatus::Unknown&&SameRetainedMinimumBounds47(legacy,actual)&&
                legacyWork.Used==retainedWork.Used&&legacyWork.Remaining==retainedWork.Remaining&&
                retained.GetLocalSpentForInspection()==retainedWork.Used&&retainedWork.Used<=uint64(cap));
        }

    }
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRetainedMinimumContainedFork47Test,
    "IAmSpeed.AnalyticWorld.TensorDistance.RetainedMinimumContainedFork",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FRetainedMinimumContainedFork47Test::RunTest(const FString&)
{
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE retained spatial test"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);

        const auto surface=MakeRetainedMinimumTestPlane47();auto box=C::Detail::Point(FVector3d(0,0,3.5));
        box.Axis[0]={-1,1,true};box.Axis[1]={-1,1,true};box.Axis[2]={3,4,true};
        const I threshold=I::Exact(9.);C::FDistanceBudget work{16384};
        C::FRetainedTensorMinimum parent(surface,box,work,0.,256,&threshold);
        const auto parentBound=parent.Advance();
        if(!TestTrue(TEXT("parent scope remains valid without choosing one center"),parentBound.Status!=C::EDistanceBoundStatus::Unknown&&
            parentBound.LowerSquared<=9.&&parentBound.UpperSquared>=16.))return false;
        const uint64 parentSpent=parent.GetLocalSpentForInspection();
        const auto& frozen=parent;
        C::ERetainedTensorMinimumFailure why=C::ERetainedTensorMinimumFailure::None;
        const uint64 beforeSnapshot=work.Used;
        TestTrue(TEXT("known state can be frozen without any new work"),frozen.IsKnownSnapshot()&&work.Used==beforeSnapshot);
        auto immutableChild=frozen.ForkContainedImmutable(C::Detail::Point(FVector3d(0,0,3.25)),why);
        if(!TestTrue(TEXT("immutable snapshot materializes a paid independent child"),immutableChild&&why==C::ERetainedTensorMinimumFailure::None&&work.Used>beforeSnapshot))return false;
        const auto immutableBound=immutableChild->Advance();
        TestTrue(TEXT("immutable child covers exact distance without changing parent"),immutableBound.Status!=C::EDistanceBoundStatus::Unknown&&
            immutableBound.LowerSquared<=10.5625&&immutableBound.UpperSquared>=10.5625&&frozen.IsKnownSnapshot()&&
            frozen.GetLocalSpentForInspection()==parentSpent&&frozen.GetFailureForInspection()==C::ERetainedTensorMinimumFailure::None);
        auto widened=box;widened.Axis[2]={2,5,true};const uint64 beforeBad=work.Used;
        TestTrue(TEXT("failed widened materialization never mutates frozen snapshot or pays"),
            !frozen.ForkContainedImmutable(widened,why)&&why==C::ERetainedTensorMinimumFailure::Scope&&
            work.Used==beforeBad&&frozen.IsKnownSnapshot()&&frozen.GetLocalSpentForInspection()==parentSpent);
        auto first=parent.ForkContained(C::Detail::Point(FVector3d(0,0,3.5)));
        if(!TestTrue(TEXT("strict child inherits its own paid metadata state"),first.IsValid()))return false;
        const auto a=first->Advance();const auto ca=first->GetCountersForInspection();
        TestTrue(TEXT("child encloses exact plane distance and proves strict radius separation"),
            a.Status!=C::EDistanceBoundStatus::Unknown&&a.LowerSquared<=12.25&&a.UpperSquared>=12.25&&a.LowerSquared>9.&&
            ca.NewRoots==0&&ca.ReusedLeaves>0&&ca.DirtyBounds>0&&first->GetLocalSpentForInspection()<=256);
        auto second=parent.ForkContained(C::Detail::Point(FVector3d(.25,-.25,3.75)));
        if(!TestTrue(TEXT("sibling has independent metadata and a contained scope"),second.IsValid()))return false;
        const auto b=second->Advance();
        TestTrue(TEXT("sibling encloses its own exact plane distance"),b.Status!=C::EDistanceBoundStatus::Unknown&&
            b.LowerSquared<=14.0625&&b.UpperSquared>=14.0625&&b.LowerSquared>9.);
        TestTrue(TEXT("children never change the parent's local debit"),parent.GetLocalSpentForInspection()==parentSpent&&parentSpent<=256);
        const uint64 used=work.Used;const auto again=parent.Advance();
        TestTrue(TEXT("parent retains its bounds and only its genuine remaining local allowance"),
            SameRetainedMinimumBounds47(parentBound,again)&&work.Used>=used&&work.Used-used<=256-parentSpent&&
            parent.GetLocalSpentForInspection()==parentSpent+(work.Used-used)&&uint64(work.Remaining)+work.Used==16384);

    }
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRetainedMinimumBudgetAndScope47Test,
    "IAmSpeed.AnalyticWorld.TensorDistance.RetainedMinimumBudgetAndScope",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FRetainedMinimumBudgetAndScope47Test::RunTest(const FString&)
{
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE retained spatial test"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);

        const auto surface=MakeRetainedMinimumTestPlane47();const auto center=C::Detail::Point(FVector3d(.37,.29,3.5));
        C::FDistanceBudget work{4096};C::FRetainedTensorMinimum minimum(surface,center,work,0.,1);
        const auto initial=minimum.Advance();const uint64 used=work.Used;
        TestTrue(TEXT("one-unit valid bound remains bounded"),initial.Status==C::EDistanceBoundStatus::Bounded&&used==1);
        TestTrue(TEXT("same scope never refills its local allowance"),SameRetainedMinimumBounds47(initial,minimum.Advance())&&work.Used==used);
        C::ERetainedTensorMinimumFailure why=C::ERetainedTensorMinimumFailure::None;
        const C::FRetainedTensorMinimum& frozen=minimum;
        TestTrue(TEXT("equal frozen scope cannot regain its exhausted local allowance"),!frozen.ForkContainedImmutable(center,why)&&
            why==C::ERetainedTensorMinimumFailure::Work&&work.Used==used&&frozen.IsKnownSnapshot()&&frozen.GetLocalSpentForInspection()==1);
        const auto equal=minimum.ForkContained(center);
        TestTrue(TEXT("unfunded equal-scope metadata is refused before global consumption"),!equal&&work.Used==used&&
            minimum.GetFailureForInspection()==C::ERetainedTensorMinimumFailure::Work);
        TestTrue(TEXT("no old result is recovered after actual refusal"),minimum.Advance().Status==C::EDistanceBoundStatus::Unknown&&work.Used==used);
        C::FDistanceBudget other{4096};C::FRetainedTensorMinimum scope(surface,center,other,0.,64);
        if(!TestTrue(TEXT("valid separate scope"),scope.Advance().Status!=C::EDistanceBoundStatus::Unknown))return false;
        auto enlarged=center;enlarged.Axis[0]={-1,1,true};const uint64 prior=other.Used;
        TestTrue(TEXT("enlargement does not inherit excluded cells"),!scope.ForkContained(enlarged)&&other.Used==prior&&
            scope.GetFailureForInspection()==C::ERetainedTensorMinimumFailure::Scope&&scope.Advance().Status==C::EDistanceBoundStatus::Unknown);
        C::FDistanceBudget blocked{8};C::FRetainedTensorMinimum floor(surface,center,blocked,0.,64);
        {
            C::FScopedDistanceBudgetFloor reservation(blocked,7);
            if(!TestTrue(TEXT("existing technical floor is valid"),reservation.IsValid()))return false;
            TestTrue(TEXT("global floor remains an actual Work refusal"),floor.Advance().Status==C::EDistanceBoundStatus::Unknown&&
                floor.GetFailureForInspection()==C::ERetainedTensorMinimumFailure::Work&&blocked.Used==1&&blocked.Remaining==7);
        }
        TestTrue(TEXT("floor release never revives a failed search or refunds work"),floor.Advance().Status==C::EDistanceBoundStatus::Unknown&&blocked.Used==1&&blocked.Remaining==7);

    }
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRetainedMinimumNumericTerminal47Test,
    "IAmSpeed.AnalyticWorld.TensorDistance.RetainedMinimumNumericTerminal",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FRetainedMinimumNumericTerminal47Test::RunTest(const FString&)
{
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE retained spatial test"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);

        const auto surface=MakeRetainedMinimumTestPlane47();const auto center=C::Detail::Point(FVector3d(.37,.29,3.5));
        C::FDistanceBudget work{4096};C::FRetainedTensorMinimum minimum(surface,center,work,0.,1);
        if(!TestTrue(TEXT("valid fresh scope before numeric change"),minimum.Advance().Status!=C::EDistanceBoundStatus::Unknown))return false;
        const uint64 used=work.Used;const uint32 active=VectorGetControlRegister();
        VectorSetControlRegister(active^uint32(0x8000));const auto refused=minimum.Advance();VectorSetControlRegister(active);
        TestTrue(TEXT("changed fingerprint refuses without new arithmetic or budget"),refused.Status==C::EDistanceBoundStatus::Unknown&&
            minimum.GetFailureForInspection()==C::ERetainedTensorMinimumFailure::Numeric&&work.Used==used);
        TestTrue(TEXT("restoring controls never replays a failed retained operation"),minimum.Advance().Status==C::EDistanceBoundStatus::Unknown&&work.Used==used);

    }
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}


IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDirectionalControlHullLowerFutureTest,
    "IAmSpeed.AnalyticWorld.TensorDistance.DirectionalControlHullLower",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDirectionalControlHullLowerFutureTest::RunTest(const FString&)
{
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE projected hull test"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        using E=C::Detail::EDirectionalHullLower;
        C::Detail::FNode diagonal;diagonal.DegreeU=diagonal.DegreeV=1;
        for(const FVector3d& p:{FVector3d(-1,3,-1),FVector3d(-1,3,1),FVector3d(3,-1,-1),FVector3d(3,-1,1)})
            diagonal.Net.Add(C::Detail::Point(p));
        const auto center=C::Detail::Point(FVector3d::ZeroVector);
        for(const FVector3d& d:{FVector3d(1,1,0),FVector3d(-1,-1,0),FVector3d(2,2,0)})
        {
            double lower=99;
            TestTrue(TEXT("full diagonal hull excludes center despite overlapping AABB; nonunit and reversed directions are safe"),
                C::Detail::BoundDirectionalHullLower(diagonal,center,d,lower)==E::Known&&lower>0&&lower<=2.);
        }
        auto centerBox=center;centerBox.Axis[0]=centerBox.Axis[1]={0,.25,true};double lower=99;
        TestTrue(TEXT("projection covers entire center box, not its midpoint"),
            C::Detail::BoundDirectionalHullLower(diagonal,centerBox,FVector3d(1,1,0),lower)==E::Known&&lower>0&&lower<=1.125);
        C::Detail::FNode point;point.Net.Add(C::Detail::Point(FVector3d(1,0,0)));
        TestTrue(TEXT("nonunit vector cannot certify numerator as squared distance"),
            C::Detail::BoundDirectionalHullLower(point,center,FVector3d(2,0,0),lower)==E::Known&&lower>0&&lower<=1.);
        C::Detail::FNode negative;negative.DegreeU=1;negative.Net.Add(C::Detail::Point(FVector3d(-3,0,0)));negative.Net.Add(C::Detail::Point(FVector3d(-2,0,0)));
        TestTrue(TEXT("negative slab uses its nearest upper endpoint"),
            C::Detail::BoundDirectionalHullLower(negative,center,FVector3d(1,0,0),lower)==E::Known&&lower>0&&lower<=4.);
        auto crossing=negative;crossing.Net[0]=C::Detail::Point(FVector3d(-1,0,0));crossing.Net[1]=C::Detail::Point(FVector3d(1,0,0));
        TestTrue(TEXT("one separated witness does not exclude a crossing coefficient hull"),
            C::Detail::BoundDirectionalHullLower(crossing,center,FVector3d(1,0,0),lower)==E::Known&&lower==0.);
        crossing.Net[0]=center;
        TestTrue(TEXT("touching slab is not strict separation"),
            C::Detail::BoundDirectionalHullLower(crossing,center,FVector3d(1,0,0),lower)==E::Known&&lower==0.);
        auto independent=point;independent.Net[0].Axis[0]={2,3,true};
        TestTrue(TEXT("coefficient boxes cover every fixed represented polynomial"),
            C::Detail::BoundDirectionalHullLower(independent,center,FVector3d(1,0,0),lower)==E::Known&&lower>0&&lower<=4.);
        TestTrue(TEXT("zero direction is explicitly inapplicable and clears stale output"),
            C::Detail::BoundDirectionalHullLower(point,center,FVector3d(0,-0.,0),lower)==E::NotApplicable&&lower==0.);
        const double invalid=AnalyticCertified::FromBits(AnalyticCertified::Infinity|1);
        TestTrue(TEXT("nonfinite direction cannot mint a lower bound"),
            C::Detail::BoundDirectionalHullLower(point,center,FVector3d(invalid,0,0),lower)==E::Unknown&&lower==0.);
        TestTrue(TEXT("norm overflow is an arithmetic Unknown with no old bound"),
            C::Detail::BoundDirectionalHullLower(point,center,FVector3d(1.e308,0,0),lower)==E::Unknown&&lower==0.);
        const double tiny=AnalyticCertified::FromBits(1);
        const auto tinyStatus=C::Detail::BoundDirectionalHullLower(point,center,FVector3d(tiny,0,0),lower);
        TestTrue(TEXT("subnormal direction cannot yield a false positive under any flush mode"),
            (tinyStatus==E::Known&&lower>=0&&lower<=1.)||(tinyStatus==E::Unknown&&lower==0.));
        auto ordinary=diagonal;
        if(!TestTrue(TEXT("directional ordinary node bounds are valid"),C::Detail::Bounds(ordinary,center)))return false;
        C::RetainedMinimumDetail::FLeafBounds retained;C::FIntervalTensorSurface placeholder;
        TestTrue(TEXT("ordinary and retained bounds use identical directional arithmetic"),
            C::RetainedMinimumDetail::Bounds(placeholder,diagonal,center,retained)&&
            AnalyticCertified::Bits(retained.LowerSquared)==AnalyticCertified::Bits(ordinary.LowerSquared)&&
            AnalyticCertified::Bits(retained.UpperSquared)==AnalyticCertified::Bits(ordinary.UpperSquared));
    }
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}


IMPLEMENT_SIMPLE_AUTOMATION_TEST(FOriginalContactDifferentialSubdivisionTest,
    "IAmSpeed.AnalyticWorld.TensorContact.OriginalDerivativeSubdivision",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FOriginalContactDifferentialSubdivisionTest::RunTest(const FString&)
{
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE original differential test"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        // Exact affine polynomial with a large translation. Analytic original
        // derivatives are (4,0,0) and (0,3,0) on EVERY restricted domain.
        C::Detail::FNode position;position.DegreeU=position.DegreeV=1;
        for(int32 u=0;u<2;++u)for(int32 v=0;v<2;++v)
            position.Net.Add(C::Detail::Point(FVector3d(1099511627776.+4.*u,3.*v,0.)));
        C::Detail::FNode du,dv;
        if(!TestTrue(TEXT("original derivative nets built from the full root"),
            C::ContactDetail::OriginalDerivativeNet(position,true,du)&&C::ContactDetail::OriginalDerivativeNet(position,false,dv)))return false;
        for(int32 level=0;level<26;++level)for(bool alongU:{true,false})
        {
            C::Detail::FNode pl,pr,ul,ur,vl,vr;
            if(!TestTrue(TEXT("all three nets follow the same dyadic restriction"),
                C::Detail::Split(position,alongU,pl,pr)&&C::Detail::Split(du,alongU,ul,ur)&&C::Detail::Split(dv,alongU,vl,vr)))return false;
            const bool right=bool(level&1);position=right?MoveTemp(pr):MoveTemp(pl);du=right?MoveTemp(ur):MoveTemp(ul);dv=right?MoveTemp(vr):MoveTemp(vl);
        }
        C::FIntervalPoint uHull,vHull;
        if(!TestTrue(TEXT("restricted original derivative hulls are valid"),C::ContactDetail::NetHull(du,uHull)&&C::ContactDetail::NetHull(dv,vHull)))return false;
        const double exactU[3]={4.,0.,0.},exactV[3]={0.,3.,0.};
        for(int32 d=0;d<3;++d)
        {
            TestTrue(TEXT("original derivative values survive translation and deep restriction"),
                uHull.Axis[d].Lower<=exactU[d]&&uHull.Axis[d].Upper>=exactU[d]&&vHull.Axis[d].Lower<=exactV[d]&&vHull.Axis[d].Upper>=exactV[d]);
        }
        TestTrue(TEXT("nonzero analytic derivative signs remain proved without local rescaling"),uHull.Axis[0].Lower>0&&vHull.Axis[1].Lower>0);
        C::Detail::FNode refused;
        TestFalse(TEXT("restricted position coefficients cannot be relabelled original coefficients"),C::ContactDetail::OriginalDerivativeNet(position,true,refused));
        C::Detail::FNode constant;constant.Net.Add(C::Detail::Point(FVector3d(2,3,4)));
        TestTrue(TEXT("degree-zero derivative is a valid zero polynomial"),C::ContactDetail::OriginalDerivativeNet(constant,true,refused)&&
            C::ContactDetail::NetHull(refused,uHull)&&uHull.Axis[0].Lower==0&&uHull.Axis[0].Upper==0);
    }
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}

#include "ReferenceAffineExtrusionTests.inl"


IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHistoricalWholeQueryGeometryCache4494Test,
    "IAmSpeed.AnalyticWorld.NativeSolid.HistoricalWholeQueryGeometryCache4494",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FHistoricalWholeQueryGeometryCache4494Test::RunTest(const FString&)
{
    FPreparedWorldSnapshotPtr noNativeSnapshot;Analytic::FWorldQuery q;q.Shape=Analytic::EQueryShape::Sphere;q.Start=FVector3d(0,0,2);q.End=FVector3d(0,0,0);q.Radius=.5;
    TArray<int32> candidates{2},excluded{0,1,3};C::FDistanceBudget budget{65536};
    const FCompleteQueryView94 key{noNativeSnapshot,q,17,A::Bits(.01),A::Bits(.49),MakeArrayView(candidates),MakeArrayView(excluded)};
    FHistoricalQueryCache94 cache;TSharedPtr<const FCompleteGeometryProof94,ESPMode::ThreadSafe> found;
    TestTrue(TEXT("pure table starts miss"),cache.Find(key,budget,found)==ECompleteCache94::Miss&&!found);
    TSharedPtr<FCompleteGeometryProof94,ESPMode::ThreadSafe> proof(new FCompleteGeometryProof94);
    proof->Contact.Status=C::ETensorSphereContactStatus::FirstContactEnclosed;proof->Contact.Leaves.AddDefaulted();proof->RegularNormals.AddDefaulted();
    proof->Contact.HasCallAccounting=true;proof->Contact.CallInitialUsed=91;proof->Contact.TimeWorkUsed=92;proof->Contact.SpatialVisitedCells=93;proof->Contact.TemporalWidthRefinements=2;proof->Contact.ConsumerWidthRefinements=1;proof->Contact.SpatialSplitsU=3;proof->Contact.SpatialSplitsV=4;proof->Contact.WorkSite=TEXT("old_work");
    SanitizeContactAccounting94(proof->Contact);
    TestTrue(TEXT("historical work is not a new budget credit"),!proof->Contact.HasCallAccounting&&proof->Contact.CallInitialUsed==0&&proof->Contact.TimeWorkUsed==0&&proof->Contact.SpatialVisitedCells==0&&proof->Contact.TemporalWidthRefinements==0&&proof->Contact.ConsumerWidthRefinements==0&&proof->Contact.SpatialSplitsU==0&&proof->Contact.SpatialSplitsV==0&&FCString::Strcmp(proof->Contact.WorkSite,TEXT("historical_query_cache94_materialization"))==0);
    TestTrue(TEXT("bounded insertion pays before copies"),FHistoricalQueryCache94::PrepareStore(key,proof->Contact,MakeArrayView(proof->RegularNormals),budget)==ECompleteCache94::Hit);
    proof->Contact.LowerTime=.25;proof->Contact.UpperTime=.5;proof->Contact.CollisionRadiusBits=A::Bits(q.Radius);proof->Hit.Time=.25;proof->Hit.SourceId=123;
    cache.StoreReserved(key,proof);TestTrue(TEXT("identical complete geometry key hits"),cache.Find(key,budget,found)==ECompleteCache94::Hit&&found==proof);
    C::FDistanceBudget noCopyBudget{0};TestFalse(TEXT("materialization consume failure is false, never a cached success"),FHistoricalQueryCache94::ChargeMaterialization(*found,noCopyBudget));
    C::FDistanceBudget copyBudget{128};if(!TestTrue(TEXT("actual clone allocation paid first"),FHistoricalQueryCache94::ChargeMaterialization(*found,copyBudget)))return false;
    FCompleteGeometryProof94 clone=*found;SanitizeContactAccounting94(clone.Contact);clone.Contact.Leaves[0].SurfaceIndex=77;
    TestTrue(TEXT("clone preserves all representative math bits and owns its arrays"),A::Bits(clone.Contact.LowerTime)==A::Bits(.25)&&A::Bits(clone.Contact.UpperTime)==A::Bits(.5)&&clone.Contact.CollisionRadiusBits==A::Bits(q.Radius)&&A::Bits(clone.Hit.Time)==A::Bits(.25)&&clone.Hit.SourceId==123&&found->Contact.Leaves[0].SurfaceIndex==INDEX_NONE);
    C::FDistanceBudget sticky{65536};sticky.Used=MAX_uint64-1;
    TestTrue(TEXT("actual consume failure after lookup is terminal, never a miss"),cache.Find(key,sticky,found)==ECompleteCache94::Failure&&sticky.Used==MAX_uint64);
    auto changed=q;changed.RequiredSourceId=1;const FCompleteQueryView94 differentFilter{noNativeSnapshot,changed,17,A::Bits(.01),A::Bits(.49),MakeArrayView(candidates),MakeArrayView(excluded)};
    TestTrue(TEXT("same bucket different full filter misses"),cache.Find(differentFilter,budget,found)==ECompleteCache94::Miss&&!found);
    const FCompleteQueryView94 differentDelta{noNativeSnapshot,q,17,A::Bits(.02),A::Bits(.49),MakeArrayView(candidates),MakeArrayView(excluded)};
    TestTrue(TEXT("delta bits participate"),cache.Find(differentDelta,budget,found)==ECompleteCache94::Miss);
    const FCompleteQueryView94 differentNumeric{noNativeSnapshot,q,18,A::Bits(.01),A::Bits(.49),MakeArrayView(candidates),MakeArrayView(excluded)};
    TestTrue(TEXT("numeric control participates"),cache.Find(differentNumeric,budget,found)==ECompleteCache94::Miss);
    TArray<int32> reverseExcluded{3,1,0};const FCompleteQueryView94 differentPartition{noNativeSnapshot,q,17,A::Bits(.01),A::Bits(.49),MakeArrayView(candidates),MakeArrayView(reverseExcluded)};
    TestTrue(TEXT("complete excluded order participates"),cache.Find(differentPartition,budget,found)==ECompleteCache94::Miss);
    TArray<int32> otherCandidate{1};const FCompleteQueryView94 differentCandidate{noNativeSnapshot,q,17,A::Bits(.01),A::Bits(.49),MakeArrayView(otherCandidate),MakeArrayView(excluded)};
    TestTrue(TEXT("candidate identity participates"),cache.Find(differentCandidate,budget,found)==ECompleteCache94::Miss);
    const FCompleteQueryView94 differentPhysical{noNativeSnapshot,q,17,A::Bits(.01),A::Bits(.48),MakeArrayView(candidates),MakeArrayView(excluded)};
    TestTrue(TEXT("physical radius bits participate"),cache.Find(differentPhysical,budget,found)==ECompleteCache94::Miss);
    C::FDistanceBudget empty{0};TestTrue(TEXT("known no-budget is not applicable before consume"),cache.Find(key,empty,found)==ECompleteCache94::NotApplicable&&empty.Used==0);
    auto bad=proof->Contact;bad.Status=C::ETensorSphereContactStatus::Unknown;const auto beforeUsed=budget.Used;
    TestTrue(TEXT("Unknown is never stored"),FHistoricalQueryCache94::PrepareStore(key,bad,MakeArrayView(proof->RegularNormals),budget)==ECompleteCache94::NotApplicable&&budget.Used==beforeUsed);
    bad=proof->Contact;bad.Leaves.SetNum(65);TArray<C::FIntervalPoint> manyNormals;manyNormals.SetNum(65);
    TestTrue(TEXT("oversize is refused before copying or Consume"),FHistoricalQueryCache94::PrepareStore(key,bad,MakeArrayView(manyNormals),budget)==ECompleteCache94::NotApplicable&&budget.Used==beforeUsed);
    for(uint64 i=1;i<=5;++i)
    {changed=q;changed.RequiredSourceId=i;const FCompleteQueryView94 collide{noNativeSnapshot,changed,17,A::Bits(.01),A::Bits(.49),MakeArrayView(candidates),MakeArrayView(excluded)};
        if(FHistoricalQueryCache94::PrepareStore(collide,proof->Contact,MakeArrayView(proof->RegularNormals),budget)!=ECompleteCache94::Hit)return false;cache.StoreReserved(collide,proof);}
    TestTrue(TEXT("four ways evict and recompute rather than return a wrong hit"),cache.Find(key,budget,found)==ECompleteCache94::Miss&&!found);
    return true;
}

namespace
{
// Canonical packet coefficients for the two neighboring LowerArc charts in
// each frame 811 capture. X has already been eliminated by the same fixed
// extrusion reduction as the wheel's affine path.
static constexpr uint64 Frame811CurveBits[2][2][24]={
    {
        {
            0x40AF6A09E667F3BCULL,0x40AF6A09E667F3BDULL,0x4052BEC333018866ULL,0x4052BEC333018867ULL,
            0x40AF86791E92EF8EULL,0x40AF86791E92EF8FULL,0x40564CAA38610294ULL,0x40564CAA38610295ULL,
            0x40AFA01DB401131AULL,0x40AFA01DB401131BULL,0x405A33E5955B8563ULL,0x405A33E5955B8564ULL,
            0x40AFB69A0D1E63D4ULL,0x40AFB69A0D1E63D5ULL,0x405E68C5102F9A13ULL,0x405E68C5102F9A14ULL,
            0x40AFC9A340C011D0ULL,0x40AFC9A340C011D1ULL,0x40616EA289E36161ULL,0x40616EA289E36162ULL,
            0x40AFD906BCF328D4ULL,0x40AFD906BCF328D5ULL,0x4063C10EACA8AB4EULL,0x4063C10EACA8AB4FULL
        },
        {
            0x40AEC3EF1535754BULL,0x40AEC3EF1535754CULL,0x40337CA1866B95CEULL,0x40337CA1866B95CFULL,
            0x40AEE915D761C9E9ULL,0x40AEE915D761C9EAULL,0x403B2E5F9FF717BFULL,0x403B2E5F9FF717C0ULL,
            0x40AF0CB9D77E832FULL,0x40AF0CB9D77E8330ULL,0x4042597CB8670AF3ULL,0x4042597CB8670AF4ULL,
            0x40AF2E60D35523D4ULL,0x40AF2E60D35523D5ULL,0x4047F892FFBB3959ULL,0x4047F892FFBB395AULL,
            0x40AF4D9AAE3CF7EBULL,0x40AF4D9AAE3CF7ECULL,0x404E61B85B441C72ULL,0x404E61B85B441C73ULL,
            0x40AF6A09E667F3BCULL,0x40AF6A09E667F3BDULL,0x4052BEC333018866ULL,0x4052BEC333018867ULL
        }
    },
    {
        {
            0xC0AF6A09E667F3BDULL,0xC0AF6A09E667F3BCULL,0x4052BEC333018866ULL,0x4052BEC333018867ULL,
            0xC0AF86791E92EF8FULL,0xC0AF86791E92EF8EULL,0x40564CAA38610294ULL,0x40564CAA38610295ULL,
            0xC0AFA01DB401131BULL,0xC0AFA01DB401131AULL,0x405A33E5955B8563ULL,0x405A33E5955B8564ULL,
            0xC0AFB69A0D1E63D5ULL,0xC0AFB69A0D1E63D4ULL,0x405E68C5102F9A13ULL,0x405E68C5102F9A14ULL,
            0xC0AFC9A340C011D1ULL,0xC0AFC9A340C011D0ULL,0x40616EA289E36161ULL,0x40616EA289E36162ULL,
            0xC0AFD906BCF328D5ULL,0xC0AFD906BCF328D4ULL,0x4063C10EACA8AB4EULL,0x4063C10EACA8AB4FULL
        },
        {
            0xC0AEC3EF1535754CULL,0xC0AEC3EF1535754BULL,0x40337CA1866B95CEULL,0x40337CA1866B95CFULL,
            0xC0AEE915D761C9EAULL,0xC0AEE915D761C9E9ULL,0x403B2E5F9FF717BFULL,0x403B2E5F9FF717C0ULL,
            0xC0AF0CB9D77E8330ULL,0xC0AF0CB9D77E832FULL,0x4042597CB8670AF3ULL,0x4042597CB8670AF4ULL,
            0xC0AF2E60D35523D5ULL,0xC0AF2E60D35523D4ULL,0x4047F892FFBB3959ULL,0x4047F892FFBB395AULL,
            0xC0AF4D9AAE3CF7ECULL,0xC0AF4D9AAE3CF7EBULL,0x404E61B85B441C72ULL,0x404E61B85B441C73ULL,
            0xC0AF6A09E667F3BDULL,0xC0AF6A09E667F3BCULL,0x4052BEC333018866ULL,0x4052BEC333018867ULL
        }
    }
};
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCanonicalCapturedWheelBoundary811Test,
    "IAmSpeed.AnalyticWorld.NativeSolid.CanonicalCapturedWheelBoundary811",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCanonicalCapturedWheelBoundary811Test::RunTest(const FString&)
{
    const auto original=Speed::CaptureFrameFloatingPointContext();
    Speed::FScopedFrameFloatingPointContext restore(original);
    if(!TestTrue(TEXT("captured numeric context"),original.bValid&&restore.IsApplied()))return false;
    const FVector3d starts[]={
        FVector3d(0,4003.672790419213,92.282395643219957),
        FVector3d(0,-4003.6727888957225,92.282394093714331)
    };
    const FVector3d ends[]={
        FVector3d(0,4020.6433531676903,75.311832894742821),
        FVector3d(0,-4020.6433516441998,75.311831345237195)
    };
    constexpr double radius=12.510000228881836;
    const double pointWidth=A::Subtract(I::Exact(radius),I::Exact(12.5)).Lower*.125;
    const double normalWidth=double(1.e-3f)*.125;
    for(int32 scenario=0;scenario<2;++scenario)
    {
        TArray<C::FIntervalTensorSurface> curves;
        for(const auto& bits:Frame811CurveBits[scenario])
        {
            C::FIntervalTensorSurface curve;curve.DegreeV=5;
            for(int32 k=0;k<6;++k)
            {
                C::FIntervalPoint cp;cp.Axis[0]=I::Exact(0);
                for(int32 d=0;d<2;++d)
                    cp.Axis[d+1]={A::FromBits(bits[k*4+d*2]),A::FromBits(bits[k*4+d*2+1]),true};
                curve.ControlPoints.Add(cp);
            }
            curves.Add(MoveTemp(curve));
        }
        double timeGap=0;
        if(!TestTrue(TEXT("captured point and normal width law"),
            M::TryFixedProbeSpatialTimeGap(starts[scenario],ends[scenario],pointWidth,normalWidth,radius,timeGap)))return false;
        C::FDistanceBudget work{1048576};
        const auto contact=C::BoundTensorSphereFirstContact(MakeArrayView(curves),starts[scenario],ends[scenario],
            radius,work,timeGap,0,8192,pointWidth,normalWidth,24);
        if(!TestTrue(TEXT("first contact enclosed near the common edge"),
            contact.Status==C::ETensorSphereContactStatus::FirstContactEnclosed&&!contact.Leaves.IsEmpty()))return false;
        for(const auto& leaf:contact.Leaves)
            TestTrue(TEXT("only the interior LowerArc1 provider owns the contact"),
                leaf.SurfaceIndex==1&&leaf.MinV>0&&leaf.MaxV<1);
    }
    return !HasAnyErrors();
}

#include "CanonicalFixedSphereEarlyQueryCacheTests.inl"

#endif
}

#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::PreparedExecution
{
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHistoricalBeforeCoverageProvenanceTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.EarlyHistoricalQueryCache94.BeforeCoverageSuccessfulProvenance",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FHistoricalBeforeCoverageProvenanceTest::RunTest(const FString&)
{
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE pure table fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        FEarlyCache94Fixture f;if(!f.Build(true))return false;
        FHistoricalQueryCache94 plain,marked;
        if(!f.Store(plain,f.Query,true)||!f.Store(marked,f.Query,true))return false;
        const TSharedPtr<const void,ESPMode::ThreadSafe> domain=MakeShared<uint8,ESPMode::ThreadSafe>(1),
            differentDomain=MakeShared<uint8,ESPMode::ThreadSafe>(1);
        const auto key=f.EarlyKey(f.Query);
        if(!TestTrue(TEXT("a successful DIRECT producer entry alone never changes coverage priority"),
            !marked.PeekCompletedWheelFallback(key,domain)&&!marked.RecordCompletedWheelFallback(key,{},f.Proof->Hit)))return false;
        TArray<Analytic::FWorldHit> changed;
        const auto add=[&](auto mutation){auto hit=f.Proof->Hit;mutation(hit);changed.Add(hit);};
        add([](auto& h){h.bHit=false;});add([](auto& h){h.bStartPenetrating=true;});
        add([](auto& h){h.Time=.5;});add([](auto& h){h.PenetrationDepth=1;});
        add([](auto& h){h.GeometricErrorBoundCm=1;});add([](auto& h){h.AdditionalResidualAgreementAllowanceCm=1;});
        add([](auto& h){h.bSurfaceNormalMayVary=true;});add([](auto& h){h.Location.X=-0.;});
        add([](auto& h){h.Point.Y=1;});add([](auto& h){h.QueryPoint.Z=1;});add([](auto& h){h.Normal.X=1;});
        add([](auto& h){h.QueryFeatureKind=Speed::EContactFeatureKind::Face;});
        add([](auto& h){h.SurfaceFeatureKind=Speed::EContactFeatureKind::Face;});
        add([](auto& h){h.QueryFeatureIndex=1;});add([](auto& h){h.SurfaceFeatureIndex=1;});
        add([](auto& h){++h.SourceId;});add([](auto& h){++h.SurfaceId;});add([](auto& h){++h.FeatureId;});
        add([](auto& h){++h.PrimitiveId;});add([](auto& h){++h.CanonicalGroupId;});add([](auto& h){++h.MaterialId;});
        for(const auto& hit:changed)
            if(!TestTrue(TEXT("every delivered hit field must match the retained successful geometry exactly"),
                !marked.RecordCompletedWheelFallback(key,domain,hit)&&!marked.PeekCompletedWheelFallback(key,domain)))return false;
        if(!TestTrue(TEXT("only matching completed wheel fallback supplies routing provenance"),
            marked.RecordCompletedWheelFallback(key,domain,f.Proof->Hit)&&marked.PeekCompletedWheelFallback(key,domain)&&
            !marked.PeekCompletedWheelFallback(key,differentDomain)&&!marked.PeekCompletedWheelFallback(key,{})))return false;
        auto q=f.Query;q.Start.X=-0.;
        if(!TestTrue(TEXT("query bits and all existing physical/numeric key fields remain mandatory"),
            !marked.PeekCompletedWheelFallback(f.EarlyKey(q),domain)&&
            !marked.PeekCompletedWheelFallback(f.EarlyKey(f.Query,18),domain)&&
            !marked.PeekCompletedWheelFallback(f.EarlyKey(f.Query,17,A::Bits(.02)),domain)&&
            !marked.PeekCompletedWheelFallback(f.EarlyKey(f.Query,17,0,A::Bits(.48)),domain)))return false;
        for(int32 cap=0;cap<180;++cap)
        {
            C::FDistanceBudget a{cap,3,1},b=a;TSharedPtr<const FCompleteGeometryProof94,ESPMode::ThreadSafe> x,y;
            C::FReferenceCandidateSetPtr px,py;
            const auto ra=plain.FindEarly(key,a,x,px);
            if(!marked.PeekCompletedWheelFallback(key,domain))return false;
            const auto rb=marked.FindEarly(key,b,y,py);
            if(!TestTrue(TEXT("peek supplies no result or budget: original lookup/floor/materialization still decide identically"),
                ra==rb&&a.Used==b.Used&&a.Remaining==b.Remaining&&a.ConsumptionFloor==b.ConsumptionFloor&&x==y&&px==py))return false;
            if(x)
            {
                C::FDistanceBudget ca{128,7,3},cb=ca;
                if(!TestTrue(TEXT("existing materialization work and values remain identical after preferred routing"),
                    FHistoricalQueryCache94::ChargeMaterialization(*x,ca)==FHistoricalQueryCache94::ChargeMaterialization(*y,cb)&&
                    ca.Used==cb.Used&&ca.Remaining==cb.Remaining&&FHistoricalQueryCache94::SameHitBits(x->Hit,y->Hit)))return false;
            }
        }
        // New entries never inherit the successful-route marker of an evicted
        // slot. Same four-way collision policy and caps as cache94.
        for(uint64 id=1;id<=5;++id){q=f.Query;q.RequiredSourceId=id;if(!f.Store(marked,q,true))return false;}
        if(!TestTrue(TEXT("eviction clears route provenance and replacement remains a direct-only entry"),
            !marked.PeekCompletedWheelFallback(key,domain)&&!marked.PeekCompletedWheelFallback(f.EarlyKey(q),domain)))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHistoricalBeforeCoverageNumericTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.EarlyHistoricalQueryCache94.BeforeCoverageFreshNumeric",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FHistoricalBeforeCoverageNumericTest::RunTest(const FString&)
{
    FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.HistoricalCacheBeforeCoverage"),1,FName(TEXT("BeforeCoverageFreshNumeric")));
    FScopedCVar<int32> early(TEXT("p.IAmSpeed.AnalyticWorld.EarlyHistoricalWholeQueryCache94"),1,FName(TEXT("BeforeCoverageEarlyRequired")));
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE pure table fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        FEarlyCache94Fixture f;if(!f.Build())return false;
        const auto numeric=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(numeric.Status!=Analytic::EAnalyticNumericStatus::Complete)return false;
        const auto key=f.Key(f.Query,numeric.ControlFingerprint());
        FHistoricalQueryCache94 cache;uint64 bytes=0;
        if(FHistoricalQueryCache94::PrepareStore(key,f.Proof->Contact,MakeArrayView(f.Proof->RegularNormals),f.Budget)!=ECompleteCache94::Hit||
            FHistoricalQueryCache94::PrepareEarlyRetention(key,f.Candidates,f.Budget,bytes)!=ECompleteCache94::Hit)return false;
        cache.StoreReserved(key,f.Proof,f.Candidates,bytes);
        const TSharedPtr<const void,ESPMode::ThreadSafe> domain=MakeShared<uint8,ESPMode::ThreadSafe>(0);
        if(!cache.RecordCompletedWheelFallback(key,domain,f.Proof->Hit)||!PeekCompletedWheelFallbackChecked(cache,key,domain))return false;
        const uint32 before=VectorGetControlRegister();VectorSetControlRegister(before^0x40u);
        if(!TestFalse(TEXT("fresh DAZ change cannot use a previously complete route"),PeekCompletedWheelFallbackChecked(cache,key,domain)))return false;
        VectorSetControlRegister((before&~uint32(0x6000))|0x2000u);
        if(!TestFalse(TEXT("nonconforming rounding cannot be hidden by a positive exact key"),PeekCompletedWheelFallbackChecked(cache,key,domain)))return false;
        VectorSetControlRegister(before);
        if(!TestTrue(TEXT("restoring the real numeric mode restores only routing eligibility"),PeekCompletedWheelFallbackChecked(cache,key,domain)))return false;
        // Null native identity is permitted only by the mathematical table
        // fixture, never by the actual public-to-TU routing function.
        if(!TestFalse(TEXT("pure table data cannot authorize a native wheel route"),
            PreferHistoricalCacheBeforeCoverage({},f.Query,.01,.49,domain)))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
}
#endif

// R-only unit draft. Include Misc/AutomationTest.h explicitly in the final TU.
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBoundedLegacyWheelOptionalWorkTest,
 "IAmSpeed.Analytic.BoundedLegacyWheel.OptionalWorkNoRefund",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FBoundedLegacyWheelOptionalWorkTest::RunTest(const FString&)
{
    using namespace Speed::PreparedExecution::BoundedLegacyWheelDetail;
    using Speed::Analytic::Certified::FDistanceBudget;
    FDistanceBudget account{10};account.ConsumptionFloor=3;
    TestTrue(TEXT("known minimum cannot begin above floor"),
        CanBeginOptional(account,8,20)==EOptionalAdmission::Unavailable&&account.Used==0&&account.Remaining==10);
    TestTrue(TEXT("local cap is an admission bound"),CanBeginOptional(account,7,6)==EOptionalAdmission::Unavailable);
    TestTrue(TEXT("exact available amount admitted"),CanBeginOptional(account,7,7)==EOptionalAdmission::Ready);
    {
        FOptionalWork work(account,2);
        TestTrue(TEXT("actual two charges"),work.Pay()&&work.Pay());
        TestFalse(TEXT("local exhaustion sticky"),work.Pay());
        TestFalse(TEXT("no later retry through same local work"),work.Pay());
    }
    TestTrue(TEXT("scope exit refunds nothing"),account.Used==2&&account.Remaining==8&&account.ConsumptionFloor==3);
    for(int32 step=0;step<5;++step)TestTrue(TEXT("exact continuation uses real remainder"),account.Consume());
    TestFalse(TEXT("eventual exact exhaustion is not a miss"),account.Consume());
    TestTrue(TEXT("no budget reset"),account.Used==7&&account.Remaining==3);
    account.ConsumptionFloor=-1;
    TestTrue(TEXT("invalid account distinct"),CanBeginOptional(account,1,1)==EOptionalAdmission::InvalidAccount);
    account={2};account.Used=MAX_uint64;
    TestTrue(TEXT("used overflow does not begin"),CanBeginOptional(account,1,1)==EOptionalAdmission::Unavailable);
    return true;
}
#endif

// Include Misc/AutomationTest.h, Misc/ScopeExit.h and Math/VectorRegister.h
// explicitly in the final TU. No production fenv writes are added.
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBoundedLegacyWheelLocalSampleTest,
 "IAmSpeed.Analytic.BoundedLegacyWheel.OriginalLocalSample",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FBoundedLegacyWheelLocalSampleTest::RunTest(const FString&)
{
    namespace B=Speed::PreparedExecution::BoundedLegacyWheelDetail;
    namespace C=Speed::Analytic::Certified;
    namespace A=::AnalyticCertified;
    using I=A::Interval;
    const uint32 saved=VectorGetControlRegister();ON_SCOPE_EXIT{VectorSetControlRegister(saved);};
    for(uint32 mode=0;mode<4;++mode)
    {
        VectorSetControlRegister((saved&~0x8040u)|((mode&1)?0x0040u:0)|((mode&2)?0x8000u:0));
        for(int32 freeAxis=0;freeAxis<3;++freeAxis)for(bool alongU:{false,true})
        {
            const int32 tangent=(freeAxis+1)%3,normalAxis=(freeAxis+2)%3;
            C::AffineReferenceDetail::FProduct product;
            product.FreeWorldAxis=freeAxis;product.FreeAlongU=alongU;
            product.Start=I::Exact(-10.);product.End=I::Exact(10.);
            product.Curve.DegreeU=0;product.Curve.DegreeV=5;
            for(int32 index=0;index<6;++index)
            {
                auto cp=C::Detail::Point(FVector3d::ZeroVector);cp.Axis[tangent]=I::Exact(2.*index);
                product.Curve.ControlPoints.Add(cp);
            }
            product.ExactCurveDerivativeHull=C::Detail::Point(FVector3d::ZeroVector);
            product.ExactCurveDerivativeHull.Axis[tangent]=I::Exact(10.);
            FVector3d point=FVector3d::ZeroVector;point[tangent]=5.;
            FVector3d unit=FVector3d::ZeroVector;unit[normalAxis]=1.;
            Speed::Analytic::FWorldQuery query;
            query.Start=point+unit*2.;query.End=point;query.Radius=1.;
            Speed::Analytic::FWorldHit hit;
            hit.bHit=true;hit.Time=.5;hit.Location=point+unit;hit.Point=point;hit.QueryPoint=point;
            hit.Normal=unit;hit.SurfaceFeatureKind=Speed::EContactFeatureKind::Face;
            B::FLocalSample output;C::FDistanceBudget budget{100};
            const auto status=B::InspectFixedProductSample(product,FVector2d(.5,.5),query,hit,1.,.99,budget,output);
            TestTrue(TEXT("authentic local sample retains existing slack and normal cell"),
                status==B::ELocalSample::BoundedLocalSample&&A::Valid(output.PositionError)&&
                output.PositionError.Upper<.01&&output.NativeSeconds==.5f&&budget.Used>0&&budget.Used<=64);
            // The fixture has a true root, but the helper's output type makes
            // no such theorem: it is just a local fixed-parameter statement.
            auto displaced=hit;displaced.Point[normalAxis]=.1;
            C::FDistanceBudget displacedBudget{100};
            TestTrue(TEXT("point outside existing slack falls back and clears stale sample"),
                B::InspectFixedProductSample(product,FVector2d(.5,.5),query,displaced,1.,.99,displacedBudget,output)==B::ELocalSample::Unavailable&&
                !A::Valid(output.PositionError)&&displacedBudget.Used>0);
            auto reversed=hit;reversed.Normal=-unit;C::FDistanceBudget reverseBudget{100};
            TestTrue(TEXT("different quantized original sample normal falls back"),
                B::InspectFixedProductSample(product,FVector2d(.5,.5),query,reversed,1.,.99,reverseBudget,output)==B::ELocalSample::Unavailable);
            auto initial=hit;initial.bStartPenetrating=true;C::FDistanceBudget initialBudget{100};
            TestTrue(TEXT("initial overlap never becomes approximate clearance"),
                B::InspectFixedProductSample(product,FVector2d(.5,.5),query,initial,1.,.99,initialBudget,output)==B::ELocalSample::Unavailable);
            auto missed=hit;missed.bHit=false;C::FDistanceBudget missBudget{100};
            TestTrue(TEXT("miss has no approximate result"),
                B::InspectFixedProductSample(product,FVector2d(.5,.5),query,missed,1.,.99,missBudget,output)==B::ELocalSample::Unavailable);
            C::FDistanceBudget boundaryBudget{100};
            TestTrue(TEXT("finite boundary sample remains exact fallback"),
                B::InspectFixedProductSample(product,FVector2d(0.,.5),query,hit,1.,.99,boundaryBudget,output)==B::ELocalSample::Unavailable);
            C::FDistanceBudget minimumBudget{80};minimumBudget.ConsumptionFloor=17;
            TestTrue(TEXT("minimum inventory refuses before doing optional work"),
                B::InspectFixedProductSample(product,FVector2d(.5,.5),query,hit,1.,.99,minimumBudget,output)==B::ELocalSample::Unavailable&&minimumBudget.Used==0&&minimumBudget.Remaining==80);
            // Regular quarter turn: BOTH global derivative component hulls
            // touch zero, but the derivative at v=.5 is (-20,+20).
            auto arc=product;const double falling[6]={20,12,6,2,0,0},rising[6]={0,0,2,6,12,20};
            for(int32 index=0;index<6;++index)
            {arc.Curve.ControlPoints[index].Axis[tangent]=I::Exact(falling[index]);arc.Curve.ControlPoints[index].Axis[normalAxis]=I::Exact(rising[index]);}
            arc.ExactCurveDerivativeHull.Axis[tangent]={-40.,0.,true};
            arc.ExactCurveDerivativeHull.Axis[normalAxis]={0.,40.,true};
            FVector3d arcPoint=FVector3d::ZeroVector,arcNormal=FVector3d::ZeroVector;
            arcPoint[tangent]=arcPoint[normalAxis]=5.;arcNormal[tangent]=arcNormal[normalAxis]=1./FMath::Sqrt(2.);
            auto arcQuery=query;arcQuery.Start=arcPoint+arcNormal*2.;arcQuery.End=arcPoint;
            auto arcHit=hit;arcHit.Location=arcPoint+arcNormal;arcHit.Point=arcHit.QueryPoint=arcPoint;arcHit.Normal=arcNormal;
            C::FDistanceBudget arcBudget{100};
            TestTrue(TEXT("local derivative admits a regular arc despite global component zero"),
                B::InspectFixedProductSample(arc,FVector2d(.5,.5),arcQuery,arcHit,1.,.99,arcBudget,output)==B::ELocalSample::BoundedLocalSample);
        }
    }
    return true;
}
#endif

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
namespace Speed::PreparedExecution
{
namespace
{
struct FBoundedMemoFixture
{
    FPreparedWorldSnapshotPtr NoNativeSnapshot;
    Analytic::FWorldQuery Query;
    BoundedWheelMemo::FGeometry Geometry;
    TSharedPtr<const void,ESPMode::ThreadSafe> Producer=MakeShared<uint8,ESPMode::ThreadSafe>(1);
    C::FDistanceBudget Work{65536};
    FCompleteQueryView94 Key(const Analytic::FWorldQuery& query,uint64 control=0,uint64 delta=0,uint64 physical=0)const
    {return {NoNativeSnapshot,query,control?control:Geometry.Sample.NumericControl,delta?delta:A::Bits(1.),physical?physical:A::Bits(.99),{}, {}};}
    bool Build()
    {
        Query.Shape=Analytic::EQueryShape::Sphere;Query.Start=FVector3d(0,0,2);Query.End=FVector3d(0,0,0);Query.Radius=1.;
        C::FIntervalTensorSurface plane;plane.DegreeU=plane.DegreeV=1;
        for(double x:{-10.,10.})for(double y:{-10.,10.})plane.ControlPoints.Add(C::Detail::Point(FVector3d(x,y,0)));
        C::FIndexedIntervalUnionPtr scene;
        if(!C::FIndexedIntervalUnion::Build(MakeArrayView(&plane,1),Work,scene))return false;
        C::FIntervalBezierCenterPath path;if(!C::CenterDetail::Linear(Query.Start,Query.End,path))return false;
        uint64 began=Work.Used;
        if(!C::FReferenceCandidateSet::Select(scene,path,Query.Radius,Work,Geometry.OriginalCandidates,nullptr,false,true))return false;
        Geometry.SelectionWork=Work.Used-began;
        const auto inflated=A::Add(I::Exact(Query.Radius),I::Exact(.001));began=Work.Used;
        if(!A::Valid(inflated)||!C::FReferenceCandidateSet::Select(scene,path,inflated.Upper,Work,Geometry.Proposal.CompleteInflatedPartition,nullptr,false,true))return false;
        Geometry.ProposalWork=Work.Used-began;
        auto& proposal=Geometry.Proposal;proposal.OriginalProvider=0;proposal.Plane=true;proposal.PlaneNormalAxis=2;
        auto& hit=proposal.Hit;hit.bHit=true;hit.Time=.5;hit.Location=FVector3d(0,0,1);
        hit.Point=hit.QueryPoint=FVector3d::ZeroVector;hit.Normal=FVector3d(0,0,1);
        hit.SurfaceFeatureKind=Speed::EContactFeatureKind::Face;hit.SourceId=123;hit.PrimitiveId=456;
        began=Work.Used;
        if(BoundedLegacyWheelDetail::InspectFixedPointSample(C::Detail::Point(FVector3d::ZeroVector),Query,hit,1.,.99,Work,Geometry.Sample,nullptr,BoundedLegacyRadialNormal::Policy())!=
            BoundedLegacyWheelDetail::ELocalSample::BoundedLocalSample)return false;
        Geometry.SampleWork=Work.Used-began;return true;
    }
};
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBoundedLegacyMemoIdentityTest,
 "IAmSpeed.Analytic.BoundedLegacyWheel.MemoIdentityAndBudget",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FBoundedLegacyMemoIdentityTest::RunTest(const FString&)
{
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE memo fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(int32 mode:{1,2})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        FBoundedMemoFixture f;if(!TestTrue(TEXT("real immutable partitions and original local sample"),f.Build()))return false;
        BoundedWheelMemo::FCache cache;BoundedWheelMemo::FGeometryPtr stored,found;const auto key=f.Key(f.Query);
        if(!TestTrue(TEXT("success stores only bounded math under paid capacity"),cache.Store(key,f.Producer,mode,f.Geometry,f.Work,stored)&&stored))return false;
        const uint64 before=f.Work.Used;
        if(!TestTrue(TEXT("same full identity reuses original and inflated partition with selector debit"),
            cache.Find(key,f.Producer,mode,f.Work,found)==ECompleteCache94::Hit&&found==stored&&
            found->OriginalCandidates==f.Geometry.OriginalCandidates&&
            found->Proposal.CompleteInflatedPartition==f.Geometry.Proposal.CompleteInflatedPartition&&
            FHistoricalQueryCache94::SameHitBits(found->Proposal.Hit,f.Geometry.Proposal.Hit)&&
            A::Bits(found->Sample.PositionError.Upper)==A::Bits(f.Geometry.Sample.PositionError.Upper)&&
            found->Sample.NativeSeconds==f.Geometry.Sample.NativeSeconds&&f.Work.Used-before==2+f.Geometry.SelectionWork))return false;
        auto changed=f.Query;changed.RequiredSourceId=1;
        TestTrue(TEXT("all policy bits remain key material"),cache.Find(f.Key(changed),f.Producer,mode,f.Work,found)==ECompleteCache94::Miss&&!found);
        changed=f.Query;changed.Start.X=-0.;
        TestTrue(TEXT("signed zero query bits remain distinct"),cache.Find(f.Key(changed),f.Producer,mode,f.Work,found)==ECompleteCache94::Miss&&!found);
        TestTrue(TEXT("approximation modes cannot reuse one another"),cache.Find(key,f.Producer,3-mode,f.Work,found)==ECompleteCache94::Miss);
        TestTrue(TEXT("physical radius/delta/control are complete identity"),
            cache.Find(f.Key(f.Query,0,0,A::Bits(.98)),f.Producer,mode,f.Work,found)==ECompleteCache94::Miss&&
            cache.Find(f.Key(f.Query,0,A::Bits(.5)),f.Producer,mode,f.Work,found)==ECompleteCache94::Miss&&
            cache.Find(f.Key(f.Query,key.Numeric^uint64(1)),f.Producer,mode,f.Work,found)==ECompleteCache94::Miss);
        for(int32 cap=0;cap<int32(f.Geometry.SelectionWork)+8;++cap)
        {
            C::FDistanceBudget budget{cap+7,19,7};const auto status=cache.Find(key,f.Producer,mode,budget,found);
            const uint64 expected=cap<5?0:FMath::Min(uint64(cap),2+f.Geometry.SelectionWork);
            TestTrue(TEXT("floor/partial replay never credits an old operation"),budget.Used==19+expected&&budget.Remaining==cap+7-int32(expected)&&budget.ConsumptionFloor==7&&
                (cap<5?status==ECompleteCache94::NotApplicable:cap<int32(2+f.Geometry.SelectionWork)?status==ECompleteCache94::Failure:status==ECompleteCache94::Hit));
        }
        C::FDistanceBudget overflow{100,MAX_uint64-4,0};
        TestTrue(TEXT("known cumulative Used insufficiency does no optional work"),cache.Find(key,f.Producer,mode,overflow,found)==ECompleteCache94::NotApplicable&&overflow.Used==MAX_uint64-4);
        overflow={100,MAX_uint64-5,0};
        TestTrue(TEXT("actual replay cumulative overflow fails with paid prefix"),cache.Find(key,f.Producer,mode,overflow,found)==ECompleteCache94::Failure&&overflow.Used==MAX_uint64);
        auto invalid=f.Geometry;invalid.Proposal.Hit.bHit=false;BoundedWheelMemo::FGeometryPtr absent;
        const uint64 invalidBefore=f.Work.Used;
        TestTrue(TEXT("miss never becomes an approximate cache value"),cache.Store(key,f.Producer,mode,invalid,f.Work,absent)&&!absent&&f.Work.Used==invalidBefore);
        TestTrue(TEXT("allocated capacities are counted before retention"),cache.Store(key,f.Producer,mode,f.Geometry,f.Work,absent,1)&&!absent&&f.Work.Used>invalidBefore);
        const TSharedPtr<const void,ESPMode::ThreadSafe> differentService=MakeShared<uint8,ESPMode::ThreadSafe>(1);
        TestTrue(TEXT("new service flushes all old service retention even on same snapshot"),cache.Find(key,differentService,mode,f.Work,found)==ECompleteCache94::Miss&&
            cache.Find(key,f.Producer,mode,f.Work,found)==ECompleteCache94::Miss);
    }
    return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFixedWheelLandscapePrefixTest,
    "IAmSpeed.AnalyticWorld.NativeSolid.FixedWheelLandscapePrefix",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FFixedWheelLandscapePrefixTest::RunTest(const FString&)
{
    Analytic::FWorldQuery query;
    query.Start=FVector(509.69661701672572,-3752.6639909872556,22.466157664928303);
    query.End=FVector(509.65442540661593,-3752.4860247165998,-1.5431459248356241);
    query.Radius=12.510000228881836;query.DomainTolerance=1.e-6;
    C::FIntervalPoint before,full;
    TestTrue(TEXT("Closed prefix is above the real Landscape at -5 cm"),
        FixedSpherePrefixHull(query,.6,before)&&before.Axis[2].Lower>-5);
    TestTrue(TEXT("The same full swept volume spans that Landscape"),
        FixedSpherePrefixHull(query,1,full)&&full.Axis[2].Lower<-5&&full.Axis[2].Upper>-5);
    TestFalse(TEXT("A time past the original trace cannot certify ordering"),FixedSpherePrefixHull(query,1.1,full));
    return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBoundedLegacyMemoProvenanceTest,
 "IAmSpeed.Analytic.BoundedLegacyWheel.MemoCompletedRoute",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FBoundedLegacyMemoProvenanceTest::RunTest(const FString&)
{
    FBoundedMemoFixture f;if(!TestTrue(TEXT("valid local fixture"),f.Build()))return false;
    BoundedWheelMemo::FCache cache;BoundedWheelMemo::FGeometryPtr stored;const auto key=f.Key(f.Query);
    const TSharedPtr<const void,ESPMode::ThreadSafe> domain=MakeShared<uint8,ESPMode::ThreadSafe>(1),otherDomain=MakeShared<uint8,ESPMode::ThreadSafe>(1),
        materialized=MakeShared<uint8,ESPMode::ThreadSafe>(1),oldMaterialized=MakeShared<uint8,ESPMode::ThreadSafe>(1);
    if(!cache.Store(key,f.Producer,2,f.Geometry,f.Work,stored)||!stored)return false;
    TestTrue(TEXT("direct mathematical cache entry cannot prefer a route"),!cache.Peek(key,f.Producer,2,domain)&&
        !cache.Record(key,f.Producer,2,domain,materialized,f.Geometry.Proposal.Hit));
    if(!cache.MarkMaterialized(key,f.Producer,2,stored,materialized))return false;
    auto differentHit=f.Geometry.Proposal.Hit;differentHit.Point.X=1.;
    TestTrue(TEXT("fresh delivered identity AND exact delivered fields are required"),
        !cache.Record(key,f.Producer,2,domain,oldMaterialized,f.Geometry.Proposal.Hit)&&
        !cache.Record(key,f.Producer,2,domain,materialized,differentHit)&&!cache.Peek(key,f.Producer,2,domain));
    TestTrue(TEXT("completed suffix marks only this service/mode/coverage domain"),
        cache.Record(key,f.Producer,2,domain,materialized,f.Geometry.Proposal.Hit)&&cache.Peek(key,f.Producer,2,domain)&&
        !cache.Peek(key,f.Producer,1,domain)&&!cache.Peek(key,f.Producer,2,otherDomain));
    const uint64 before=f.Work.Used;
    TestTrue(TEXT("peek does not consume, materialize or authorize"),cache.Peek(key,f.Producer,2,domain)&&f.Work.Used==before);
    for(uint64 index=1;index<=4;++index)
    {
        auto changed=f.Query;changed.RequiredSourceId=index;BoundedWheelMemo::FGeometryPtr replacement;
        if(!cache.Store(f.Key(changed),f.Producer,2,f.Geometry,f.Work,replacement)||!replacement)return false;
    }
    TestTrue(TEXT("eviction cannot transfer weak materialization or completed route"),!cache.Peek(key,f.Producer,2,domain)&&
        !cache.Record(key,f.Producer,2,domain,materialized,f.Geometry.Proposal.Hit));
    BoundedWheelMemo::FGeometryPtr reinserted;
    if(!cache.Store(key,f.Producer,2,f.Geometry,f.Work,reinserted)||!reinserted)return false;
    TestTrue(TEXT("same key reinserted is a new uncompleted mathematical value"),reinserted!=stored&&!cache.Peek(key,f.Producer,2,domain)&&
        !cache.Record(key,f.Producer,2,domain,materialized,f.Geometry.Proposal.Hit));
    return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBoundedLegacyLocalReasonTest,
 "IAmSpeed.Analytic.BoundedLegacyWheel.LocalRefusalAttribution",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FBoundedLegacyLocalReasonTest::RunTest(const FString&)
{
    namespace B=BoundedLegacyWheelDetail;
    FBoundedMemoFixture f;if(!f.Build())return false;
    auto hit=f.Geometry.Proposal.Hit;hit.Point.X=1.;B::FLocalSample sample;B::ELocalRefusal reason=B::ELocalRefusal::Input;
    C::FDistanceBudget a{100},b=a;
    const auto withReason=B::InspectFixedPointSample(C::Detail::Point(FVector3d::ZeroVector),f.Query,hit,1.,.99,a,sample,&reason);
    const auto plain=B::InspectFixedPointSample(C::Detail::Point(FVector3d::ZeroVector),f.Query,hit,1.,.99,b,sample);
    TestTrue(TEXT("passive position attribution changes neither status nor debit"),withReason==B::ELocalSample::Unavailable&&withReason==plain&&
        reason==B::ELocalRefusal::PositionSlack&&a.Used==b.Used&&a.Remaining==b.Remaining);
    hit=f.Geometry.Proposal.Hit;hit.Normal=FVector3d(1,0,0);a={100};
    TestTrue(TEXT("normal cell refusal is distinct from geometric slack"),
        B::InspectFixedPointSample(C::Detail::Point(FVector3d::ZeroVector),f.Query,hit,1.,.99,a,sample,&reason)==B::ELocalSample::Unavailable&&reason==B::ELocalRefusal::NormalCell);
    auto away=f.Query;away.Start=FVector3d::ZeroVector;away.End=FVector3d(0,0,2);hit=f.Geometry.Proposal.Hit;a={100};
    TestTrue(TEXT("outgoing local approach remains refused with separate attribution"),
        B::InspectFixedPointSample(C::Detail::Point(FVector3d::ZeroVector),away,hit,1.,.99,a,sample,&reason)==B::ELocalSample::Unavailable&&reason==B::ELocalRefusal::Approach);
    return !HasAnyErrors();
}
}
#endif

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
namespace Speed::PreparedExecution
{
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBoundedFrontendInventoryMemoTest,
 "IAmSpeed.Analytic.BoundedLegacyWheel.FrontendInventoryMemo",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FBoundedFrontendInventoryMemoTest::RunTest(const FString&)
{
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE inventory cache"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(int32 mode:{1,2})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        FBoundedMemoFixture f;if(!f.Build())return false;
        auto geometry=f.Geometry;const auto& partition=f.Geometry.Proposal.CompleteInflatedPartition;
        C::FReferenceCandidateInventoryPtr inventory;
        if(!TestTrue(TEXT("complete original CP inventory without surface copies"),C::FReferenceCandidateInventory::Select(
            partition->GetScene(),partition->GetPathForFixedSource(),partition->GetRadiusForFixedSource(),f.Work,inventory)&&inventory))return false;
        geometry.OriginalCandidates.Reset();geometry.Proposal.CompleteInflatedPartition.Reset();geometry.Proposal.CompleteInflatedInventory=inventory;
        geometry.InventoryOnly=true;geometry.SelectionWork=0;
        BoundedWheelMemo::FCache cache;BoundedWheelMemo::FGeometryPtr stored,found;const auto key=f.Key(f.Query);
        if(!TestTrue(TEXT("inventory math retained only in explicit frontend variant"),cache.Store(key,f.Producer,mode,geometry,f.Work,stored)&&stored))return false;
        TestTrue(TEXT("partition producer cannot receive inventory-only value"),cache.Find(key,f.Producer,mode,f.Work,found)==ECompleteCache94::Miss&&!found);
        const uint64 before=f.Work.Used;
        if(!TestTrue(TEXT("frontend lookup retains original local sample and inventory identity"),
            cache.Find(key,f.Producer,mode,f.Work,found,true)==ECompleteCache94::Hit&&found==stored&&found->InventoryOnly&&
            !found->OriginalCandidates&&!found->Proposal.CompleteInflatedPartition&&found->Proposal.CompleteInflatedInventory==inventory&&
            FHistoricalQueryCache94::SameHitBits(found->Proposal.Hit,geometry.Proposal.Hit)&&
            A::Bits(found->Sample.PositionError.Upper)==A::Bits(geometry.Sample.PositionError.Upper)&&f.Work.Used==before+2))return false;
        const TSharedPtr<const void,ESPMode::ThreadSafe> domain=MakeShared<uint8,ESPMode::ThreadSafe>(1),identity=MakeShared<uint8,ESPMode::ThreadSafe>(1);
        TestTrue(TEXT("inventory completion never creates old partition-route preference"),
            cache.MarkMaterialized(key,f.Producer,mode,stored,identity)&&cache.Record(key,f.Producer,mode,domain,identity,geometry.Proposal.Hit)&&
            !cache.Peek(key,f.Producer,mode,domain));
        BoundedWheelMemo::FGeometryPtr originalStored;
        if(!cache.Store(key,f.Producer,mode,f.Geometry,f.Work,originalStored)||!originalStored)return false;
        TestTrue(TEXT("same query may hold both distinct payload kinds without confusion"),
            cache.Find(key,f.Producer,mode,f.Work,found)==ECompleteCache94::Hit&&found==originalStored&&
            cache.Find(key,f.Producer,mode,f.Work,found,true)==ECompleteCache94::Hit&&found==stored);
        auto mixed=geometry;mixed.OriginalCandidates=f.Geometry.OriginalCandidates;
        const uint64 beforeMixed=f.Work.Used;
        TestTrue(TEXT("mixed payload refused before retention or debit"),cache.Store(key,f.Producer,mode,mixed,f.Work,found)&&!found&&f.Work.Used==beforeMixed);
        C::FDistanceBudget tiny{4,8,0};
        TestTrue(TEXT("bounded setup cannot borrow from a later exact path"),cache.Find(key,f.Producer,mode,tiny,found,true)==ECompleteCache94::NotApplicable&&tiny.Used==8&&!found);
        auto changed=f.Query;changed.End.Z=A::FromBits(A::Bits(changed.End.Z)+1);
        TestTrue(TEXT("different original path cannot reuse inventory math"),cache.Find(f.Key(changed),f.Producer,mode,f.Work,found,true)==ECompleteCache94::Miss);
    }
    return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBoundedFrontendSourceIdentityTest,
 "IAmSpeed.Analytic.BoundedLegacyWheel.FrontendSourceIdentity",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FBoundedFrontendSourceIdentityTest::RunTest(const FString&)
{
    using S=FBoundedWheelValueFactory::ESource;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE source closure"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        FBoundedMemoFixture f;if(!f.Build())return false;
        const auto& partition=f.Geometry.Proposal.CompleteInflatedPartition;const auto scene=partition->GetScene();
        C::FReferenceCandidateInventoryPtr inventory;
        if(!C::FReferenceCandidateInventory::Select(scene,partition->GetPathForFixedSource(),partition->GetRadiusForFixedSource(),f.Work,inventory))return false;
        auto proposal=f.Geometry.Proposal;TConstArrayView<int32> indices;
        C::FDistanceBudget originalWork{100},inventoryWork{100};
        if(!TestTrue(TEXT("original partition path remains admitted"),FBoundedWheelValueFactory::Source(proposal,scene,f.Query,originalWork,indices)==S::Ready&&indices.Num()==1))return false;
        proposal.CompleteInflatedPartition.Reset();proposal.CompleteInflatedInventory=inventory;
        if(!TestTrue(TEXT("same source family admitted through the inventory without changed debit"),
            FBoundedWheelValueFactory::Source(proposal,scene,f.Query,inventoryWork,indices)==S::Ready&&indices.Num()==1&&indices[0]==0&&inventoryWork.Used==originalWork.Used))return false;
        auto wrongQuery=f.Query;wrongQuery.Start.X=.125;
        TestTrue(TEXT("wrong path bits refuse without leaking candidate view"),FBoundedWheelValueFactory::Source(proposal,scene,wrongQuery,f.Work,indices)==S::Unavailable&&indices.IsEmpty());
        wrongQuery=f.Query;wrongQuery.Radius=2.;
        TestTrue(TEXT("wrong inflated radius cannot qualify source coverage"),FBoundedWheelValueFactory::Source(proposal,scene,wrongQuery,f.Work,indices)==S::Unavailable&&indices.IsEmpty());
        auto mixed=proposal;mixed.CompleteInflatedPartition=partition;const uint64 before=f.Work.Used;
        TestTrue(TEXT("mixed partition/inventory is structurally unavailable before debit"),FBoundedWheelValueFactory::Source(mixed,scene,f.Query,f.Work,indices)==S::Unavailable&&indices.IsEmpty()&&f.Work.Used==before);
        TestTrue(TEXT("a complete inventory from another scene cannot be authorized"),FBoundedWheelValueFactory::Source(proposal,{},f.Query,f.Work,indices)==S::Unavailable&&indices.IsEmpty());
        C::FDistanceBudget noWork{7,31,7};
        TestTrue(TEXT("source closure preserves real floor failure"),FBoundedWheelValueFactory::Source(proposal,scene,f.Query,noWork,indices)==S::Work&&indices.IsEmpty()&&noWork.Used==31&&noWork.Remaining==7);
        const uint32 current=VectorGetControlRegister();VectorSetControlRegister(current^0x40u);
        TestTrue(TEXT("captured numeric control mismatch remains terminal"),FBoundedWheelValueFactory::Source(proposal,scene,f.Query,f.Work,indices)==S::Numeric&&indices.IsEmpty());
        VectorSetControlRegister(current);
        VectorSetControlRegister((current&~0x3fu)|0x3fu);
        TestTrue(TEXT("foreign sticky history is not a numeric-control mismatch"),FBoundedWheelValueFactory::Source(proposal,scene,f.Query,f.Work,indices)==S::Ready);
        VectorSetControlRegister(current);
    }
    return !HasAnyErrors();
}
}
#endif

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
namespace Speed::PreparedExecution
{
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBoundedOriginalRadialNormalCellsTest,
 "IAmSpeed.Analytic.BoundedLegacyWheel.OriginalRadialNormalCells",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FBoundedOriginalRadialNormalCellsTest::RunTest(const FString&)
{
    namespace R=BoundedLegacyRadialNormal;namespace B=BoundedLegacyWheelDetail;
    const auto savedContext=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(savedContext);std::fenv_t held;
    if(!TestTrue(TEXT("RNE radial fixture"),savedContext.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        const auto up=C::Detail::Point(FVector3d(0,0,1));
        const FVector legacy=FVector(.002,0,1).GetSafeNormal();
        M::FUniqueQuantizedNormal normal;M::FQuantizedNormalDiagnostic diagnostic;FVector stored;I change;
        C::FDistanceBudget oracle{100},off{100},on{100};
        if(!TestTrue(TEXT("disjoint cell is the real unchanged oracle failure"),!M::TryUniqueQuantizedNormal(up,legacy,oracle,normal,&diagnostic)&&
            diagnostic.Failure==M::EQuantizedNormalFailure::DisjointCell))return false;
        TestTrue(TEXT("OFF preserves refusal and exact debit"),!R::Try(up,legacy,0,off,normal,stored,change)&&off.Used==oracle.Used&&stored.IsZero()&&!A::Valid(change));
        if(!TestTrue(TEXT("original radial recovers a different strict discrete cell"),R::Try(up,legacy,1,on,normal,stored,change)&&
            M::SameBits(stored,FVector(0,0,1))&&M::SameBits(normal.Quantized,Speed::QuantizeUnitNormal(stored))&&A::Valid(change)&&change.Upper>0&&change.Upper<.003))return false;
        const uint64 retryWork=on.Used;
        const FVector difference=Speed::QuantizeUnitNormal(legacy)-normal.Quantized;
        TestTrue(TEXT("published delta bounds actual quantized native-vector change"),FMath::Abs(difference.X)+FMath::Abs(difference.Y)+FMath::Abs(difference.Z)<=change.Upper);
        const FVector closeLegacy(.00001,0,1);C::FDistanceBudget originalAccepted{100},kept{100};M::FUniqueQuantizedNormal oldNormal;
        if(!M::TryUniqueQuantizedNormal(up,closeLegacy,originalAccepted,oldNormal))return false;
        TestTrue(TEXT("already accepted raw normal and consumed bits never change"),R::Try(up,closeLegacy,1,kept,normal,stored,change)&&
            M::SameBits(stored,closeLegacy)&&M::SameBits(normal.Quantized,oldNormal.Quantized)&&M::SameBits(normal.ReNormalized,oldNormal.ReNormalized)&&
            kept.Used==originalAccepted.Used&&A::Bits(change.Lower)==A::Bits(0.)&&A::Bits(change.Upper)==A::Bits(0.));
        const double step=double(1.e-3f);auto ambiguous=up;
        ambiguous.Axis[0]={step*.49,step*.51,true};ambiguous.Axis[2]={.999,1.001,true};oracle={100};on={100};
        if(!TestTrue(TEXT("strict boundary is genuinely ambiguous"),!M::TryUniqueQuantizedNormal(ambiguous,FVector(step*.5,0,1),oracle,oldNormal,&diagnostic)&&
            diagnostic.Failure==M::EQuantizedNormalFailure::AmbiguousCell))return false;
        TestTrue(TEXT("no midpoint can authorize a crossing enclosure"),!R::Try(ambiguous,FVector(step*.5,0,1),1,on,normal,stored,change)&&
            on.Used==oracle.Used&&stored.IsZero()&&!A::Valid(change));
        for(int32 available=0;available<=int32(retryWork)+1;++available)
        {
            C::FDistanceBudget limited{available+7,19,7};
            const bool ready=R::Try(up,legacy,1,limited,normal,stored,change);
            TestTrue(TEXT("one account, floor and paid failure remain intact"),ready==(uint64(available)>=retryWork)&&limited.Used>=19&&limited.Used<=uint64(19+available)&&
                limited.Remaining==available+7-int32(limited.Used-19)&&limited.ConsumptionFloor==7&&
                (ready||(stored.IsZero()&&!A::Valid(change)&&normal.Quantized.IsZero())));
        }
        C::FDistanceBudget overflow{100,MAX_uint64-3,0};
        TestTrue(TEXT("cumulative overflow cannot publish a replacement"),!R::Try(up,legacy,1,overflow,normal,stored,change)&&overflow.Used==MAX_uint64&&stored.IsZero());
        FBoundedMemoFixture f;if(!f.Build())return false;
        auto hit=f.Geometry.Proposal.Hit;hit.Normal=legacy;B::FLocalSample sample;B::ELocalRefusal reason;
        C::FDistanceBudget localOff{100},localOn{100};
        TestTrue(TEXT("same original geometry remains rejected under old local policy"),B::InspectFixedPointSample(C::Detail::Point(FVector3d::ZeroVector),f.Query,hit,1.,.99,localOff,sample,&reason,0)==B::ELocalSample::Unavailable&&reason==B::ELocalRefusal::NormalCell);
        TestTrue(TEXT("full local validator publishes stored replacement within old caps"),B::InspectFixedPointSample(C::Detail::Point(FVector3d::ZeroVector),f.Query,hit,1.,.99,localOn,sample,&reason,1)==B::ELocalSample::BoundedLocalSample&&
            sample.NormalPolicy==1&&M::SameBits(sample.StoredNormal,FVector(0,0,1))&&sample.PositionError.Upper<=.01&&localOn.Used<=32);
        auto restricted=f.Query;restricted.ReferenceNormal=legacy;restricted.MinimumReferenceNormalDot=.9999999;C::FDistanceBudget referenceWork{100};
        TestTrue(TEXT("reference-normal threshold is still independently enforced"),B::InspectFixedPointSample(C::Detail::Point(FVector3d::ZeroVector),restricted,hit,1.,.99,referenceWork,sample,&reason,1)==B::ELocalSample::Unavailable&&reason==B::ELocalRefusal::ReferenceNormal);
        C::AffineReferenceDetail::FProduct curved;curved.FreeWorldAxis=0;curved.FreeAlongU=true;
        curved.Start=I::Exact(-10.);curved.End=I::Exact(10.);curved.Curve.DegreeU=0;curved.Curve.DegreeV=5;
        for(int32 index=0;index<6;++index)curved.Curve.ControlPoints.Add(C::Detail::Point(FVector3d(0,4.*index,double(index*(index-1)))));
        const FVector curvePoint(0,10,5),curveNormal=FVector(0,-1,1).GetSafeNormal();auto curveQuery=f.Query;
        curveQuery.Start=curvePoint+curveNormal*2.;curveQuery.End=curvePoint;
        auto curveHit=f.Geometry.Proposal.Hit;curveHit.Point=curveHit.QueryPoint=curvePoint;curveHit.Location=curvePoint+curveNormal;
        curveHit.Normal=(curveNormal+FVector(.002,0,0)).GetSafeNormal();C::FDistanceBudget curveOld{100},curveNew{100};
        TestTrue(TEXT("actual original product and derivative route forwards the isolated normal policy"),
            B::InspectFixedProductSample(curved,FVector2d(.5,.5),curveQuery,curveHit,1.,.99,curveOld,sample,&reason,0)==B::ELocalSample::Unavailable&&reason==B::ELocalRefusal::NormalCell&&
            B::InspectFixedProductSample(curved,FVector2d(.5,.5),curveQuery,curveHit,1.,.99,curveNew,sample,&reason,1)==B::ELocalSample::BoundedLocalSample&&
            sample.NormalPolicy==1&&M::SameBits(sample.Normal.Quantized,Speed::QuantizeUnitNormal(curveNormal))&&curveNew.Used<=64);
    }
    return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBoundedOriginalRadialNormalCachePolicyTest,
 "IAmSpeed.Analytic.BoundedLegacyWheel.OriginalRadialNormalCachePolicy",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FBoundedOriginalRadialNormalCachePolicyTest::RunTest(const FString&)
{
    namespace R=BoundedLegacyRadialNormal;namespace B=BoundedLegacyWheelDetail;
    FScopedCVar<int32> outer(TEXT("p.SkyLeague.Stadium.BoundedLegacyWheelOriginalRadialNormal"),0,FName(TEXT("RadialPolicyOuterOFF")));
    const auto savedContext=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(savedContext);std::fenv_t held;
    if(!TestTrue(TEXT("RNE cache-policy fixture"),savedContext.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(int32 mode:{1,2})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        FBoundedMemoFixture f;if(!f.Build())return false;
        BoundedWheelMemo::FCache cache;BoundedWheelMemo::FGeometryPtr initial,found,replacement;const auto key=f.Key(f.Query);
        const TSharedPtr<const void,ESPMode::ThreadSafe> domain=MakeShared<uint8,ESPMode::ThreadSafe>(1),identity=MakeShared<uint8,ESPMode::ThreadSafe>(1);
        if(!cache.Store(key,f.Producer,mode,f.Geometry,f.Work,initial)||!initial)return false;
        if(!cache.MarkMaterialized(key,f.Producer,mode,initial,identity)||!cache.Record(key,f.Producer,mode,domain,identity,f.Geometry.Proposal.Hit))return false;
        {
            FScopedCVar<int32> inner(TEXT("p.SkyLeague.Stadium.BoundedLegacyWheelOriginalRadialNormal"),1,FName(TEXT("RadialPolicyInnerON")));
            if(!TestTrue(TEXT("real opt-in applied"),R::Policy()==1))return false;
            TestTrue(TEXT("all old lookups and early route hints exclude other normal policy"),cache.Find(key,f.Producer,mode,f.Work,found)==ECompleteCache94::Miss&&!found&&!cache.Peek(key,f.Producer,mode,domain));
            const uint64 before=f.Work.Used;
            TestTrue(TEXT("old sample cannot be republished under the new policy"),cache.Store(key,f.Producer,mode,f.Geometry,f.Work,found)&&!found&&f.Work.Used==before);
            auto geometry=f.Geometry;geometry.Proposal.Hit.Normal=FVector(.002,0,1).GetSafeNormal();const uint64 sampleBefore=f.Work.Used;
            if(B::InspectFixedPointSample(C::Detail::Point(FVector3d::ZeroVector),f.Query,geometry.Proposal.Hit,1.,.99,f.Work,geometry.Sample,nullptr,R::Policy())!=B::ELocalSample::BoundedLocalSample)return false;
            geometry.SampleWork=f.Work.Used-sampleBefore;geometry.Proposal.Hit.Normal=geometry.Sample.StoredNormal;
            if(!cache.Store(key,f.Producer,mode,geometry,f.Work,replacement)||!replacement)return false;
            TestTrue(TEXT("new variant retains its actual produced normal and paid sample"),cache.Find(key,f.Producer,mode,f.Work,found)==ECompleteCache94::Hit&&found==replacement&&found->Sample.NormalPolicy==1&&M::SameBits(found->Proposal.Hit.Normal,found->Sample.StoredNormal));
            TestTrue(TEXT("materialization cannot borrow the other policy's provenance"),!cache.MarkMaterialized(key,f.Producer,mode,initial,identity)&&!cache.Record(key,f.Producer,mode,domain,identity,f.Geometry.Proposal.Hit));
        }
        if(!TestTrue(TEXT("unique CVar tags restore the real outer policy"),R::Policy()==0))return false;
        TestTrue(TEXT("returning OFF restores only old geometry and old completed hint"),cache.Find(key,f.Producer,mode,f.Work,found)==ECompleteCache94::Hit&&found==initial&&cache.Peek(key,f.Producer,mode,domain)&&!cache.MarkMaterialized(key,f.Producer,mode,replacement,identity));
    }
    return !HasAnyErrors();
}
}
#endif

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
namespace Speed::PreparedExecution
{
// Synthetic tokens intentionally have NO prepared snapshot: they exercise
// owned geometry/cache/consumer helpers, and cannot authorize a native query.
struct FInitCertifiedLegacyWheelTests
{
    static bool Patch(Analytic::FExtrudedQuinticPatch& p)
    {
        p={};p.SourceId=1;p.SurfaceId=2;p.FeatureId=3;p.PrimitiveId=4;p.CanonicalGroupId=5;p.MaterialId=6;
        p.bQueryCollisionEnabled=true;p.BlockingChannels=MAX_uint64;p.ExtrusionAxis=FVector3d(1,0,0);
        p.MinimumExtrusionCoordinate=-10;p.MaximumExtrusionCoordinate=10;
        for(int32 j=0;j<6;++j)p.SectionControlPoints[j]=FVector3d(0,4.*j,0);
        return p.BuildQueryApproximationBounded(.00099,1024);
    }
    static FInitCertifiedLegacyRepresentationPtr Copy(const Analytic::FExtrudedQuinticPatch& p)
    {
        TSharedPtr<FInitCertifiedLegacyRepresentation,ESPMode::ThreadSafe> t(new FInitCertifiedLegacyRepresentation);
        t->Geometry=MakeShared<Analytic::FExtrudedQuinticPatch,ESPMode::ThreadSafe>(p);t->Provider=0;t->ErrorCm=.001;
        const auto& owned=*t->Geometry;
        t->RetainedBytes=sizeof(FInitCertifiedLegacyRepresentation)+sizeof(Analytic::FExtrudedQuinticPatch)+192+
            owned.SectionPolyline.GetAllocatedSize()+owned.SectionParameters.GetAllocatedSize()+owned.SectionSegmentBounds.GetAllocatedSize()+
            owned.SectionFacetGeometry.GetAllocatedSize()+owned.SectionSegmentBvhNodes.GetAllocatedSize()+owned.SectionSegmentBvhIndices.GetAllocatedSize();
        t->NumericControl=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
        return t;
    }
    static bool GeometryMatches(const FInitCertifiedLegacyRepresentationPtr& t,int32 provider,const Analytic::FWorldHit& h,int32 facet)
    {return t&&t->MatchesOwnedGeometry(provider,h,facet);}
    static Analytic::FWorldHit Sweep(const FInitCertifiedLegacyRepresentationPtr& t,const Analytic::FWorldQuery& q)
    {return Analytic::FWorldQueryService::TestRetainedExtrudedSweep938(q,*t->Geometry,false,1,false);}
};
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FInitLegacyOwnedGeometryTest,
 "IAmSpeed.Analytic.BoundedLegacyWheel.InitCertifiedOwnedGeometry",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FInitLegacyOwnedGeometryTest::RunTest(const FString&)
{
    using T=FInitCertifiedLegacyWheelTests;
    const auto saved=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(saved);std::fenv_t held;
    if(!TestTrue(TEXT("RNE test context"),saved.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 control=VectorGetControlRegister();int32 hits=0;
    for(uint32 mode:{0u,0x40u,0x8000u,0x8040u})for(uint32 sticky:{0u,0x3fu})
    {
        VectorSetControlRegister((control&~uint32(0x807f))|mode|sticky);
        Analytic::FExtrudedQuinticPatch p;if(!TestTrue(TEXT("real legacy line representation"),T::Patch(p)))return false;
        const auto token=T::Copy(p);Analytic::FWorldQuery q;q.Shape=Analytic::EQueryShape::Sphere;
        q.Start=FVector3d(0,10,30);q.End=FVector3d(0,10,-10);q.Radius=1.;q.bAuthorityOnly=true;q.bUseFiniteContactDomain=true;
        const auto old=Analytic::FWorldQueryService::TestRetainedExtrudedSweep938(q,p,false,1,false),owned=T::Sweep(token,q);
        if(!TestTrue(TEXT("same actual legacy kernel on deeply owned patch"),old.bHit&&owned.bHit&&FHistoricalQueryCache94::SameHitBits(old,owned)))return false;
        ++hits;
        TestTrue(TEXT("facet belongs to this exact owned geometry"),T::GeometryMatches(token,0,owned,0));
        TestFalse(TEXT("overflow-sized facet refuses before addition"),T::GeometryMatches(token,0,owned,MAX_int32));
        TestFalse(TEXT("foreign provider refuses"),T::GeometryMatches(token,1,owned,0));
        auto foreign=owned;foreign.MaterialId++;
        TestFalse(TEXT("foreign material refuses"),T::GeometryMatches(token,0,foreign,0));
        foreign=owned;foreign.PrimitiveId++;
        TestFalse(TEXT("foreign full facet identity refuses"),T::GeometryMatches(token,0,foreign,0));
        p.SectionPolyline.Reset();p.SectionFacetGeometry.Reset();p.SectionParameters.Reset();p.SourceId=77;
        TestTrue(TEXT("source mutation and release cannot alter owned geometry"),FHistoricalQueryCache94::SameHitBits(owned,T::Sweep(token,q)));
        TestFalse(TEXT("no fabricated snapshot can authorize this synthetic token"),token->Matches({},0,owned,0));
        auto miss=q;miss.Start.X=miss.End.X=100;
        TestFalse(TEXT("legacy miss stays a miss of the prepared representation"),T::Sweep(token,miss).bHit);
    }
    TestEqual(TEXT("eight nonvacuous kernel comparisons"),hits,8);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FInitLegacyNativeConversionTest,
 "IAmSpeed.Analytic.BoundedLegacyWheel.InitCertifiedNativeConversion",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FInitLegacyNativeConversionTest::RunTest(const FString&)
{
    using K=InitCertifiedLegacyWheelControl::EConversion;
    if(!TestTrue(TEXT("native token created on game thread"),IsInGameThread()))return false;
    const TStrongObjectPtr<UBoxComponent> component(NewObject<UBoxComponent>());
    FPreparedSourceContactBinding binding;binding.SourceId=1;binding.SourceIdentity=component.Get();binding.ContactIdentity=component.Get();binding.ContactToken=component.Get();
    const auto saved=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(saved);std::fenv_t held;
    if(!TestTrue(TEXT("RNE conversion context"),saved.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 control=VectorGetControlRegister();int32 known=0;
    for(uint32 mode:{0u,0x40u,0x8000u,0x8040u})for(double delta:{1.,.01666666753590107})
    {
        VectorSetControlRegister((control&~uint32(0x8040))|mode);
        Analytic::FWorldHit hit;hit.bHit=true;hit.Time=.375;hit.SourceId=1;hit.SurfaceId=2;hit.PrimitiveId=3;hit.FeatureId=4;
        hit.Location=FVector3d(0,0,1);hit.Point=hit.QueryPoint=FVector3d::ZeroVector;hit.Normal=FVector3d(.12,.34,.94);
        hit.SurfaceFeatureKind=EContactFeatureKind::Face;
        auto original=hit,oracleInput=hit;oracleInput.GeometricErrorBoundCm=.001;oracleInput.bSurfaceNormalMayVary=true;
        SHitResult oracle;if(!BuildNativeAnalyticContactValues(oracleInput,static_cast<float>(hit.Time*delta),0,binding,oracle))return false;
        oracle.FaceIndex=int32(hit.PrimitiveId&0x7fffffffull);oracle.ImpactNormal=QuantizeUnitNormal(oracle.ImpactNormal);
        C::FDistanceBudget work{9};work.ConsumptionFloor=7;work.Used=13;SHitResult actual;
        if(!TestTrue(TEXT("real native conversion and quantization unchanged"),InitCertifiedLegacyWheelControl::Convert(hit,.001,delta,binding,work,actual)==K::Ready&&
            SameNativeAnalyticContactValues(actual,oracle)&&work.Remaining==7&&work.Used==15&&M::SameBits(hit.Normal,original.Normal)))return false;
        ++known;
        for(int32 cap=0;cap<2;++cap)
        {
            auto refused=original;C::FDistanceBudget shortWork{7+cap};shortWork.ConsumptionFloor=7;shortWork.Used=13;SHitResult absent=actual;
            TestTrue(TEXT("paid conversion refusal never publishes a partial value"),InitCertifiedLegacyWheelControl::Convert(refused,.001,delta,binding,shortWork,absent)==K::Work&&
                !absent.bHit&&shortWork.Remaining==7&&shortWork.Used==uint64(13+cap)&&FHistoricalQueryCache94::SameHitBits(refused,original));
        }
        for(int32 kind=0;kind<4;++kind)
        {
            auto invalid=original;auto badBinding=binding;double error=.001;
            if(kind==0)invalid.bHit=false;if(kind==1)invalid.bStartPenetrating=true;
            if(kind==2)badBinding.ContactToken.Reset();if(kind==3)error=.002;
            C::FDistanceBudget denied{8};SHitResult absent=actual;
            TestTrue(TEXT("miss/overlap/absent native contact/uncertified error cannot materialize"),InitCertifiedLegacyWheelControl::Convert(invalid,error,delta,badBinding,denied,absent)==K::Unavailable&&!absent.bHit);
        }
        C::FDistanceBudget exhausted{10};exhausted.Used=MAX_uint64;SHitResult absent=actual;auto copy=original;
        TestTrue(TEXT("Used overflow preserves account and output"),InitCertifiedLegacyWheelControl::Convert(copy,.001,delta,binding,exhausted,absent)==K::Work&&!absent.bHit&&exhausted.Used==MAX_uint64);
    }
    TestEqual(TEXT("eight successful real conversions"),known,8);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FInitLegacyMemoVariantTest,
 "IAmSpeed.Analytic.BoundedLegacyWheel.InitCertifiedMemoVariant",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FInitLegacyMemoVariantTest::RunTest(const FString&)
{
    FScopedCVar<int32> bound(TEXT("p.SkyLeague.Stadium.BoundedLegacyWheelProbe"),2,FName(TEXT("InitMemoBound2")));
    FScopedCVar<int32> frontend(TEXT("p.SkyLeague.Stadium.BoundedLegacyWheelFrontend"),0,FName(TEXT("InitMemoFrontOFF")));
    FScopedCVar<int32> init(TEXT("p.SkyLeague.Stadium.InitCertifiedLegacyWheelResponse"),1,FName(TEXT("InitMemoON")));
    const auto saved=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(saved);std::fenv_t held;
    if(!TestTrue(TEXT("RNE memo context"),saved.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 control=VectorGetControlRegister();
    for(uint32 mode:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((control&~uint32(0x8040))|mode);
        FBoundedMemoFixture f;if(!f.Build())return false;const auto key=f.Key(f.Query);
        auto geometry=f.Geometry;Analytic::FExtrudedQuinticPatch patch;if(!FInitCertifiedLegacyWheelTests::Patch(patch))return false;
        geometry.Proposal.Plane=false;geometry.Proposal.InitRepresentation=FInitCertifiedLegacyWheelTests::Copy(patch);
        geometry.Sample={};geometry.SampleWork=0;geometry.InitOnly=true;geometry.InitPolicy=1;
        BoundedWheelMemo::FCache cache;BoundedWheelMemo::FGeometryPtr stored,found;
        if(!TestTrue(TEXT("distinct init variant stored without inventing a local sample"),cache.Store(key,f.Producer,2,geometry,f.Work,stored)&&stored&&
            cache.Find(key,f.Producer,2,f.Work,found)==ECompleteCache94::Hit&&found==stored&&!A::Valid(found->Sample.PositionError)))return false;
        const TSharedPtr<const void,ESPMode::ThreadSafe> domain=MakeShared<uint8,ESPMode::ThreadSafe>(1),identity=MakeShared<uint8,ESPMode::ThreadSafe>(2);
        TestTrue(TEXT("same completion handshake works for distinct typed materialization"),cache.MarkMaterialized(key,f.Producer,2,stored,identity)&&
            cache.Record(key,f.Producer,2,domain,identity,geometry.Proposal.Hit)&&cache.Peek(key,f.Producer,2,domain));
        {
            FScopedCVar<int32> off(TEXT("p.SkyLeague.Stadium.InitCertifiedLegacyWheelResponse"),0,FName(TEXT("InitMemoInnerOFF")));
            TestTrue(TEXT("local-original policy cannot use init geometry or completion peek"),cache.Find(key,f.Producer,2,f.Work,found)==ECompleteCache94::Miss&&!cache.Peek(key,f.Producer,2,domain));
        }
        auto changed=f.Query;changed.End.X=1.;
        TestTrue(TEXT("new path cannot inherit the proposal"),cache.Find(f.Key(changed),f.Producer,2,f.Work,found)==ECompleteCache94::Miss);
        TestTrue(TEXT("new radius/producer/control cannot inherit it"),cache.Find(f.Key(f.Query,0,0,A::Bits(.98)),f.Producer,2,f.Work,found)==ECompleteCache94::Miss&&
            cache.Find(key,MakeShared<uint8,ESPMode::ThreadSafe>(3),2,f.Work,found)==ECompleteCache94::Miss);
        auto invalid=geometry;invalid.Proposal.InitRepresentation.Reset();BoundedWheelMemo::FGeometryPtr absent;
        TestTrue(TEXT("no certificate cannot be stored as init variant"),cache.Store(key,f.Producer,2,invalid,f.Work,absent)&&!absent);
    }
    return true;
}
}
#endif
