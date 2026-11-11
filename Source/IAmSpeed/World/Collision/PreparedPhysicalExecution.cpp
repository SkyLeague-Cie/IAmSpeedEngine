#include "PreparedPhysicalExecution.h"
#include <type_traits>
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "Misc/ScopedCVar.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#endif
#include <atomic>
#include <exception>
#include "Misc/CoreDelegates.h"
#include "HAL/PlatformTime.h"
#include "HAL/IConsoleManager.h"
#include "PreparedNativeFrameConsumer.h"
#include "PreparedNativeWheelProbe.h"
#include "PreparedNativeOwnerPoseMutation.h"
#include "CanonicalFixedSphereWheelProbe.h"
#include "PreparedSleepingSphereSupport.h"
#include "HAL/PlatformTime.h"
#include "PreparedFirstRsResponseData.h"
#include "IAmSpeed/World/Simulation/SubBodyKinematicsTasks.h"
#include "IAmSpeed/World/Simulation/JoinedActorKinematics1213.h"
#include "NativeInitialHitCache.h"
#include "PreparedFirstArrivalRsResponseQuality.h"
#include "NativeGeneralSphereContactValues.h"
#include "NativeRepeatedSphereFloorExclusion.h"
#include "IAmSpeed/World/Simulation/CcdPoseAudit.h"
#include "NativePreparationPrefix.h"
#include "IAmSpeed/World/Simulation/OnePausedCanonicalWork.h"
#include "Engine/World.h"
#include "PreparedSphereRequest.h"
#include "PreparedNativeSolidRequest.h"
#include "EarliestCollisionBatch.h"
#include "ResolvedPairSet.h"
#include "NativeFloorSphereContactGeometry.h"
#include "PreparedOriginPlaneContact.h"
#include "NativeFloorBoxPlaneValues.h"
#include "PreparedCanonicalStaticQueryService.h"
#include "IAmSpeed/Base/SUtils.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/SubBodies/Common/ISolidSweeper.h"
#include "IAmSpeed/World/Analytic/StaticWorldQueryAudit.h"
#include "PreparedSphereRsTrajectoryCapture.h"
#include "PreparedSpherePostSyncRsCapture.h"
#include <exception>
#include "Misc/ScopeExit.h"
#include "PreparedSphereResponseQuality.h"
#include "PreparedSphereResponseBinding.h"
#include "PreparedSphereHitValues.h"
#include "PreparedSphereConsumedState.h"
#include "IAmSpeed/Base/OriginKinematicStateValues.h"
#include "PreparedSweepEvidence.h"
#include "PreparedNativeAnalyticContactValues.h"
#include "PreparedIntervalScene.h"
#include "PreparedWorldSnapshot.h"
#include "StaticCollisionWorld.h"
#include "PreparedPhysicalQueryDomain.h"
#include "IAmSpeed/World/Simulation/SpeedPhysicsSimulation.h"
#include "IAmSpeed/World/Simulation/SimulationWorker.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "IAmSpeed/World/Simulation/SimulationWorld.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "IAmSpeed/SubBodies/Solid/SphereSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "IAmSpeed/World/Analytic/AnalyticNumericContext.h"
#include "IAmSpeed/World/Analytic/ReferenceSphereConsumer.h"
#include "HAL/PlatformTLS.h"
#include "IAmSpeed/Base/SpeedConstant.h"
#include "Templates/UnrealTemplate.h"
#include "Misc/ScopeLock.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include <stdexcept>

// Private observation only. No field below is a query/identity/result cache,
// authority, budget, continuation token or collision decision.
namespace StadiumSphereProfile70 { void PublishSnapshot(const TCHAR* scope); }

namespace StadiumProbeProfile70
{
enum ERow:uint32 { OwnerCheck,WheelCheck,Census,WholeSweep,WholeEvaluate,Coverage,SphereFallback,Suffix,Conversion,RowCount };
struct FRow {std::atomic<uint64> Calls{0},Finished{0},InFlight{0},Ticks{0},Unwinding{0},BudgetUsed{0},Candidates{0},Excluded{0},ContactAccounting{0},TimeWork{0},SpatialCells{0};std::atomic<bool> Integrity{true};};
FRow Rows[RowCount];std::atomic<bool> ObservedEnabled{false};
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
void PublishSnapshot(const TCHAR* scope)
{
 static constexpr const TCHAR* names[]={TEXT("OwnerCheck"),TEXT("WheelCheck"),TEXT("Census"),TEXT("WholeSweep"),TEXT("WholeEvaluate"),TEXT("Coverage"),TEXT("SphereFallback"),TEXT("Suffix"),TEXT("Conversion")};
 FString text=FString::Printf(TEXT("[PreparedWheelStageProfile70] Scope=%s PerCase=0 TerminalQueryJoin=0 NoPASS=1 NoCachePermission=1 InclusiveRowsNotAdditive=1 Enabled=%d SecondsPerCycle=%.17g"),scope,int32(ObservedEnabled.load()),FPlatformTime::GetSecondsPerCycle64());
 for(uint32 i=0;i<RowCount;++i)
 {
  auto& r=Rows[i];const uint64 calls=r.Calls.load(),flight=r.InFlight.load(),finished=r.Finished.load(),ticks=r.Ticks.load(),unwinding=r.Unwinding.load(),budget=r.BudgetUsed.load(),candidates=r.Candidates.load(),excluded=r.Excluded.load(),accounting=r.ContactAccounting.load(),time=r.TimeWork.load(),cells=r.SpatialCells.load();
  const bool coherent=r.Integrity.load()&&flight==0&&r.InFlight.load()==0&&calls==r.Calls.load()&&calls==finished;
  text+=FString::Printf(TEXT(" | Row=%s SnapshotCoherent=%d Calls=%llu Finished=%llu InFlight=%llu Ticks=%llu Unwinding=%llu BudgetUsed=%llu Candidates=%llu Excluded=%llu ContactAccounting=%llu TimeWork=%llu SpatialCells=%llu"),names[i],int32(coherent),calls,finished,flight,ticks,unwinding,budget,candidates,excluded,accounting,time,cells);
 }
 UE_LOG(LogTemp,Display,TEXT("%s"),*text);
}
void Publish(){PublishSnapshot(TEXT("ProgramPrefixAtPreExit"));}
// Same-DLL static registration only, during module load before any worker.
// The callback runs before module shutdown; it reads diagnostics only.
struct FRegister {FRegister(){FCoreDelegates::OnPreExit.AddStatic(&Publish);}};
FRegister Register;
}

namespace PreparedFrameworkProfile1
{
// Observational spans only: self time subtracts only nested spans named here,
// not arbitrary engine work. No admission, mutation or allocation is added.
enum ERow:uint32 {FrameStateRead,IntegrationContext,ResponseContext,FinalContext,PrimitiveClose,GeneralEndpoint,Configuration,RowCount};
struct FRow
{
    std::atomic<uint64> Calls{0},Finished{0},InFlight{0},InclusiveTicks{0},SelfTicks{0},Unwinding{0};
    std::atomic<uint64> OwnersCopied{0},BodiesCopied{0},ConstraintsCopied{0},IgnoredCopied{0},CapacityGrowths{0},CapacityGrowthBytes{0};
    std::atomic<bool> Integrity{true};
};
static FRow Rows[RowCount];static std::atomic<bool> Observed{false};
struct FScope;
static thread_local FScope* Current=nullptr;
static void Add(FRow& row,std::atomic<uint64>& counter,uint64 value)
{if(counter.fetch_add(value)>MAX_uint64-value)row.Integrity.store(false);}
struct FScope
{
    FRow& Row;bool Active=false;FScope* Parent=nullptr;uint64 Start=0,Children=0;int Uncaught=0;
    uint64 Owners=0,Bodies=0,Constraints=0,Ignored=0,Growths=0,GrowthBytes=0;
    explicit FScope(ERow id):Row(Rows[id]),Active(::StadiumProbeProfile70::Enabled())
    {
        if(!Active)return;Observed.store(true);Add(Row,Row.Calls,1);Add(Row,Row.InFlight,1);
        Uncaught=std::uncaught_exceptions();Parent=Current;Current=this;Start=FPlatformTime::Cycles64();
    }
    template<class TArrayType> void Capacity(int32 previous,const TArrayType& values)
    {
        if(!Active||values.Max()<=previous)return;
        ++Growths;GrowthBytes+=uint64(values.Max()-previous)*sizeof(typename TArrayType::ElementType);
    }
    ~FScope() noexcept
    {
        if(!Active)return;const uint64 end=FPlatformTime::Cycles64();
        if(Current!=this||end<Start){Row.Integrity.store(false);if(Parent)Parent->Row.Integrity.store(false);}
        else
        {
            const uint64 elapsed=end-Start;Add(Row,Row.InclusiveTicks,elapsed);
            if(Children>elapsed)Row.Integrity.store(false);else Add(Row,Row.SelfTicks,elapsed-Children);
            if(Parent){if(Parent->Children>MAX_uint64-elapsed)Parent->Row.Integrity.store(false);else Parent->Children+=elapsed;}
        }
        Current=Parent;
        Add(Row,Row.OwnersCopied,Owners);Add(Row,Row.BodiesCopied,Bodies);Add(Row,Row.ConstraintsCopied,Constraints);Add(Row,Row.IgnoredCopied,Ignored);
        Add(Row,Row.CapacityGrowths,Growths);Add(Row,Row.CapacityGrowthBytes,GrowthBytes);
        if(std::uncaught_exceptions()>Uncaught)Add(Row,Row.Unwinding,1);
        Add(Row,Row.Finished,1);if(Row.InFlight.fetch_sub(1)==0)Row.Integrity.store(false);
    }
    FScope(const FScope&)=delete;FScope& operator=(const FScope&)=delete;
};
static void Publish()
{
    static constexpr const TCHAR* names[]={TEXT("FrameStateRead"),TEXT("IntegrationContext"),TEXT("ResponseContext"),
        TEXT("FinalContext"),TEXT("PrimitiveClose"),TEXT("GeneralEndpoint"),TEXT("Configuration")};
    FString report=FString::Printf(TEXT("[PreparedFrameworkProfile1] Scope=ProgramPrefixAtPreExit ObservationOnly=1 Enabled=%d SelfExcludesOnlyNamedNestedSpans=1 CapacityGrowthNotSystemMalloc=1 SecondsPerCycle=%.17g"),
        int32(Observed.load()),FPlatformTime::GetSecondsPerCycle64());
    for(uint32 i=0;i<RowCount;++i)
    {
        auto& row=Rows[i];const auto calls=row.Calls.load(),finished=row.Finished.load(),flight=row.InFlight.load();
        const bool coherent=row.Integrity.load()&&flight==0&&calls==finished&&calls==row.Calls.load()&&row.InFlight.load()==0;
        report+=FString::Printf(TEXT(" | Row=%s Coherent=%d Calls=%llu Finished=%llu InFlight=%llu InclusiveTicks=%llu SelfTicks=%llu Unwinding=%llu OwnersCopied=%llu BodiesCopied=%llu ConstraintsCopied=%llu IgnoredCopied=%llu CapacityGrowths=%llu CapacityGrowthBytes=%llu"),
            names[i],int32(coherent),calls,finished,flight,row.InclusiveTicks.load(),row.SelfTicks.load(),row.Unwinding.load(),
            row.OwnersCopied.load(),row.BodiesCopied.load(),row.ConstraintsCopied.load(),row.IgnoredCopied.load(),
            row.CapacityGrowths.load(),row.CapacityGrowthBytes.load());
    }
    UE_LOG(LogTemp,Display,TEXT("%s"),*report);
}
struct FRegister {FRegister(){FCoreDelegates::OnPreExit.AddStatic(&Publish);}};
static FRegister Register;
}

#if !UE_BUILD_SHIPPING
namespace Speed::Analytic::Certified::Case205RemainderObservation
{
void Drain(const Speed::PreparedExecution::FStepIdentity&,bool terminal) noexcept;
}
#endif

namespace Speed::PreparedExecution
{
namespace
{
thread_local const FSimulationWorker* CurrentWorker=nullptr;
thread_local const FHostCallScope* HostScope=nullptr;
thread_local FStepScope* StepScope=nullptr;
thread_local FCanonicalFramePreflight* NativeFramePreflight=nullptr;
thread_local FSphereMemberSweepScope* MemberSweepScope=nullptr;
thread_local uint64 MemberSweepSerial=0;
thread_local FComponentSweepScope* ComponentSweepScope=nullptr;
thread_local FOrdinaryOwnerProbePhaseScope* OwnerProbePhase=nullptr;
thread_local FNativeWheelProbeScope* WheelProbeScope=nullptr;
thread_local uint64 ComponentSweepSerial=0;
thread_local bool Poisoned=false;
thread_local uint64 PoisonDiagnosticFrame=0,PoisonDiagnosticStep=0;
bool SameFloat(float a,float b) noexcept {uint32 x=0,y=0;FMemory::Memcpy(&x,&a,4);FMemory::Memcpy(&y,&b,4);return x==y;}
bool SameDouble(double a,double b) noexcept {uint64 x=0,y=0;FMemory::Memcpy(&x,&a,8);FMemory::Memcpy(&y,&b,8);return x==y;}
bool SameVector(const FVector& a,const FVector& b) noexcept {return SameDouble(a.X,b.X)&&SameDouble(a.Y,b.Y)&&SameDouble(a.Z,b.Z);}
bool SameQuat(const FQuat& a,const FQuat& b) noexcept {return SameDouble(a.X,b.X)&&SameDouble(a.Y,b.Y)&&SameDouble(a.Z,b.Z)&&SameDouble(a.W,b.W);}
bool FiniteState(const FKinematicState& s) noexcept
{return !s.Location.ContainsNaN()&&!s.Velocity.ContainsNaN()&&!s.Acceleration.ContainsNaN()&&!s.Rotation.ContainsNaN()&&!s.AngularVelocity.ContainsNaN()&&!s.AngularAcceleration.ContainsNaN();}
void RecordPreparedPoison(const char* site,const char* detail=nullptr,uint64 ordinal=0,uint64 serial=0,
    int32 wheel=INDEX_NONE,uint8 phase=0xff,bool queryOpen=false,bool used=false,int32 expectedUncaught=0) noexcept
{
    const bool first=!Poisoned;Poisoned=true; // Same sticky poison; never reset or bypassed.
    if(first)try
    {
        UE_LOG(LogTemp,Display,TEXT("[PreparedContextFirstPoison] Site=%hs Detail=%hs CapturedFrame=%llu CapturedStep=%llu Thread=%u Worker=%p Host=%p Step=%p Member=%p OwnerProbe=%p WheelProbe=%p CapturedOrdinal=%llu CapturedScopeSerial=%llu Wheel=%d CapturedPhase=%u QueryOpen=%d Used=%d ExpectedUncaught=%d Uncaught=%d"),
            site,detail?detail:"none",PoisonDiagnosticFrame,StepScope?PoisonDiagnosticStep:0,FPlatformTLS::GetCurrentThreadId(),
            CurrentWorker,HostScope,StepScope,MemberSweepScope,OwnerProbePhase,WheelProbeScope,ordinal,serial,wheel,uint32(phase),queryOpen,used,expectedUncaught,std::uncaught_exceptions());
    }catch(...){} // Diagnostics cannot replace the original failure/cleanup.
}
void Refuse(const char* why)
{
    try{UE_LOG(LogTemp,Display,TEXT("[PreparedContextRefusal] Reason=%hs CapturedFrame=%llu CapturedStep=%llu Thread=%u Poisoned=%d"),
        why,PoisonDiagnosticFrame,StepScope?PoisonDiagnosticStep:0,FPlatformTLS::GetCurrentThreadId(),Poisoned);}catch(...){}
    throw std::runtime_error(why);
}
}
bool Requested(){static const bool result=FParse::Param(FCommandLine::Get(),TEXT("SpeedPreparedIntervalWorld"));return result;}
bool RejectPreparedPendingMutation(bool active,bool pending) noexcept {return active&&pending;}
bool RejectPreparedOrderMutation(bool active,bool dirty) noexcept {return active&&dirty;}
ECanonicalCensusRefusal InspectCanonicalCensusForRefusal(TConstArrayView<FPreparedNativeOwnerBinding> rows,
    TConstArrayView<ISpeedComponent*> adapters,TConstArrayView<ISpeedComponent*> ordered,bool pending,bool dirty)
{
    if(pending)return ECanonicalCensusRefusal::PendingRegistry;
    if(dirty)return ECanonicalCensusRefusal::DirtyOrder;
    if(rows.IsEmpty()||rows.Num()!=adapters.Num()||ordered.Num()!=adapters.Num())return ECanonicalCensusRefusal::IncompleteInventory;
    // Same keys/hash/predicates; inline storage avoids repeated heap traffic
    // for the common census. The allocator still grows on heap beyond 64.
    TSet<const ISpeedComponent*,DefaultKeyFuncs<const ISpeedComponent*>,TInlineSetAllocator<64>> actual,sorted,declared;
    TSet<const USSubBody*,DefaultKeyFuncs<const USSubBody*>,TInlineSetAllocator<64>> children;
    TSet<uint32,DefaultKeyFuncs<uint32>,TInlineSetAllocator<64>> childObjectIds;
    for(const auto* owner:adapters)if(!owner||actual.Contains(owner))return ECanonicalCensusRefusal::DuplicateOwner;else actual.Add(owner);
    for(const auto* owner:ordered)if(!owner||!actual.Contains(owner)||sorted.Contains(owner))return ECanonicalCensusRefusal::IncompleteInventory;else sorted.Add(owner);
    for(const auto& row:rows)
    {
        if(!row.Adapter||!actual.Contains(row.Adapter)||declared.Contains(row.Adapter))return ECanonicalCensusRefusal::DuplicateOwner;
        declared.Add(row.Adapter);
        // Pure pointer/class metadata only. NO WeakObject.Get, Cast, virtual
        // adapter/body getter, kinematics or COM call may precede qualification.
        if(!row.NativeObject||!row.Class||!row.BodyCensusCaptured)return ECanonicalCensusRefusal::MissingNativeMetadata;
        for(int32 i=0;i<row.Bodies.Num();++i)
        {
            const auto& body=row.Bodies[i];
            if(body.Index!=i||!body.Body||!body.Class||!body.ObjectIdCaptured||children.Contains(body.Body)||childObjectIds.Contains(body.ObjectId))return ECanonicalCensusRefusal::InvalidChildCensus;
            children.Add(body.Body);childObjectIds.Add(body.ObjectId);
        }
    }
    // No current compiled implementation covers ALL frame phases. Inherited
    // free motion or sphere sync is not countdown/input/resolve/Post coverage;
    // zero-body runner and frozen car therefore remain unqualified as well.
    return ECanonicalCensusRefusal::NoCompiledPhaseCoverage;
}
FCanonicalFramePreflight::~FCanonicalFramePreflight() noexcept
{
    if(!NativeConsumer)return;
    if(NativeFramePreflight!=this||(!PreparedOwners.IsEmpty()&&(!NativeStepClosed||!SnapshotClosureConsumed)))RecordPreparedPoison("PreparedPhysicalExecution.cpp:119");
    if(NativeFramePreflight==this)NativeFramePreflight=nullptr;
}

namespace
{
TAutoConsoleVariable<int32> CVarRawCensusValueMemo(TEXT("p.IAmSpeed.AnalyticWorld.RawCensusValueMemo"),0,
    TEXT("Reuse the pure immutable raw-census value for unchanged actual adapter arrays. All live gates remain fresh."),ECVF_Default);
// Rows are privately owned by a const FPreparedNativeOwnerBindings. Retaining
// this metadata object prevents pointer reuse; it does not retain UObject
// ownership or cache a frame/window, numeric control, query or permission.
struct FRawCensusValueMemo
{
    FPreparedNativeOwnerBindingsPtr OwnerPin;
    const void* Identity=nullptr;
    TArray<ISpeedComponent*,TInlineAllocator<16>> Adapters,Ordered;
    ECanonicalCensusRefusal Result=ECanonicalCensusRefusal::IncompleteInventory;
    bool Valid=false;
    static bool Same(TConstArrayView<ISpeedComponent*> a,TConstArrayView<ISpeedComponent*> b)
    {
        if(a.Num()!=b.Num())return false;
        // A differing representation is a cache miss and reruns the original census.
        return a.IsEmpty()||FMemory::Memcmp(a.GetData(),b.GetData(),SIZE_T(a.Num())*sizeof(ISpeedComponent*))==0;
    }
    ECanonicalCensusRefusal Evaluate(const void* identity,TConstArrayView<FPreparedNativeOwnerBinding> rows,
        TConstArrayView<ISpeedComponent*> adapters,TConstArrayView<ISpeedComponent*> ordered,
        bool pending,bool dirty,bool& reused)
    {
        reused=false;
        // Fresh dynamic revocation retains the original refusal order.
        if(pending)return ECanonicalCensusRefusal::PendingRegistry;
        if(dirty)return ECanonicalCensusRefusal::DirtyOrder;
        if(identity&&Valid&&Identity==identity&&Same(Adapters,adapters)&&Same(Ordered,ordered))
        {reused=true;return Result;}
        const auto result=InspectCanonicalCensusForRefusal(rows,adapters,ordered,false,false);
        if(identity&&adapters.Num()<=256&&ordered.Num()<=256)
        {
            Adapters.Reset();Adapters.Append(adapters.GetData(),adapters.Num());
            Ordered.Reset();Ordered.Append(ordered.GetData(),ordered.Num());
            Identity=identity;Result=result;Valid=true;
        }
        else {Valid=false;Identity=nullptr;Adapters.Reset();Ordered.Reset();}
        return result;
    }
};
}

bool FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(
    const USpeedWorldSubsystem& bridge, const FPreparedWorldSnapshot& snapshot, uint64 expectedStep)
{
    ::StadiumProbeProfile70::FScope queryProfile70(::StadiumProbeProfile70::Census);
    if (bridge.StepSerial!=expectedStep || !snapshot.GetNativeOwners()) return false;
    bool pending=false; {FScopeLock lock(&bridge.PendingCS);pending=!bridge.PendingOps.IsEmpty();}
    if(CVarRawCensusValueMemo.GetValueOnAnyThread()!=0)
    {
        thread_local FRawCensusValueMemo memo;
        const auto& owners=snapshot.GetNativeOwners();
        if(memo.OwnerPin!=owners){memo.Valid=false;memo.OwnerPin=owners;}
        bool reused=false;
        return memo.Evaluate(owners.Get(),owners->GetRecordsForInspection(),
            bridge.SimulationWorld.GetAdapters(),bridge.SimulationWorld.GetOrderedAdapters(),
            pending,bridge.SimulationWorld.bOrderDirty,reused)==ECanonicalCensusRefusal::NoCompiledPhaseCoverage;
    }
    // This enum remains a refusal for FullFrame. Here it proves ONLY the raw
    // census shape; a closed preparation source/value contract is also required.
    return InspectCanonicalCensusForRefusal(snapshot.GetNativeOwners()->GetRecordsForInspection(),
        bridge.SimulationWorld.GetAdapters(),bridge.SimulationWorld.GetOrderedAdapters(),pending,
        bridge.SimulationWorld.bOrderDirty)==ECanonicalCensusRefusal::NoCompiledPhaseCoverage;
}
bool FCanonicalFramePreflight::HasCurrentNativeRunControlContext(const ASpeedPhysicsSimulation& host,
    const USpeedWorldSubsystem& bridge,const FSimulationWorker* worker,uint64 generation,uint64 frame,float delta,float time)
{
    OnePausedWork::FCanonicalFrameWindowPtr window;
    const auto* provider=bridge.GetStaticCollisionWorld();
    if(Poisoned||HostScope||StepScope||NativeFramePreflight||!IsCurrentWorker(worker)||!provider||
        !OnePausedWork::ReadExecutionWindow(window)||!window||window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame||
        !window->GetNonce()||!window->Matches(&host,&bridge,provider,worker,generation,frame,delta,time)||
        host.GetInputWorkerGeneration()!=generation||!host.GetWorld()||host.GetWorld()!=bridge.GetWorld()||
        host.GetWorld()->bIsTearingDown||!provider->IsReadyForPhysicsStep())return false;
    const auto snapshot=provider->GetPreparedSnapshot();const auto consumer=provider->GetPreparedNativeFrameConsumer();
    if(!snapshot||window->GetSnapshot()!=snapshot||!consumer||!consumer->Matches(snapshot,provider->GetPreparedQueryService()))return false;
    const auto& origin=snapshot->GetOrigin();
    if(origin.Host!=&host||origin.Bridge!=&bridge||origin.World!=host.GetWorld()||origin.Generation!=generation||
        !snapshot->GetNativeOwners()||snapshot->GetNativeOwners()->GetGeneration()!=generation||
        !HasUnchangedRawPreparationCensus(bridge,*snapshot,window->GetStepBefore()))return false;
    Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    return true;
}
FCanonicalFramePreflight::FCanonicalFramePreflight(bool required,const ASpeedPhysicsSimulation& host,
    const USpeedWorldSubsystem& bridge,const FSimulationWorker* worker,uint64 generation,uint64 frame,float delta,float time)
{
    if(!required)return;
    // Revocation/window identity is checked before native/UObject reads.
    if(Poisoned||!IsCurrentWorker(worker)||!OnePausedWork::ReadExecutionWindow(Window)||!Window||
        !Window->Matches(&host,&bridge,bridge.GetStaticCollisionWorld(),worker,generation,frame,delta,time)||!Window->GetNonce())
        Refuse("Prepared canonical preflight window/worker identity unavailable");
    if(!HostScope||!HostScope->Active||HostScope->Host!=&host||HostScope->Bridge!=&bridge||
        HostScope->Worker!=worker||HostScope->World!=Window->GetWorldIdentity()||HostScope->Generation!=generation||
        HostScope->Frame!=frame||HostScope->ThreadId!=FPlatformTLS::GetCurrentThreadId()||
        !SameFloat(HostScope->Delta,delta)||!SameFloat(HostScope->SimTime,time))
        Refuse("Prepared canonical preflight is not inside the same actual native Host call");
    Snapshot=Window->GetSnapshot();
    if(!Snapshot||!Snapshot->GetNativeOwners()||!Snapshot->GetGeometrySelection()||!Snapshot->GetGeometrySelection()->HasValidAliases()||
        host.GetInputWorkerGeneration()!=generation||!host.GetWorld()||host.GetWorld()!=bridge.GetWorld()||host.GetWorld()->bIsTearingDown||
        Window->GetWorldIdentity()!=host.GetWorld()||bridge.GetStaticCollisionWorld()->GetPreparedSnapshot()!=Snapshot)
        Refuse("Prepared canonical preflight snapshot/World identity differs");
    const auto& origin=Snapshot->GetOrigin();
    if(origin.World!=host.GetWorld()||origin.Host!=&host||origin.Bridge!=&bridge||origin.Generation!=generation||
        Snapshot->GetNativeOwners()->GetGeneration()!=generation||bridge.StepSerial!=Window->GetStepBefore())
        Refuse("Prepared canonical preflight generation/origin/StepBefore differs");
    Host=&host;Bridge=&bridge;Worker=worker;Generation=generation;Frame=frame;StepBefore=Window->GetStepBefore();
    WindowSerial=Window->GetSerial();Nonce=Window->GetNonce();ThreadId=FPlatformTLS::GetCurrentThreadId();
    FMemory::Memcpy(&DeltaBits,&delta,4);FMemory::Memcpy(&TimeBits,&time,4);
    bool pending=false;
    {FScopeLock lock(&bridge.PendingCS);pending=!bridge.PendingOps.IsEmpty();}
    const auto status=InspectCanonicalCensusForRefusal(Snapshot->GetNativeOwners()->GetRecordsForInspection(),
        bridge.SimulationWorld.GetAdapters(),bridge.SimulationWorld.GetOrderedAdapters(),pending,bridge.SimulationWorld.bOrderDirty);
    if ((Window->GetPurpose()==OnePausedWork::EPurpose::PreparationPrefixStop||
        OnePausedWork::HasInitialPosition(Window->GetPurpose())) &&
        status==ECanonicalCensusRefusal::NoCompiledPhaseCoverage)
    {
        // A separate source-bound prefix NEVER satisfies full-frame readiness.
        PreparationCursor.Reset(new PreparationPrefix::FCursor(host,bridge,worker,generation,frame,delta,time));
        return;
    }
    if(status==ECanonicalCensusRefusal::NoCompiledPhaseCoverage&&Window->GetPurpose()==OnePausedWork::EPurpose::FullFrame)
    {
        const auto* provider=bridge.GetStaticCollisionWorld();
        const auto consumer=provider->GetPreparedNativeFrameConsumer();
        if(consumer&&consumer->Matches(Snapshot,provider->GetPreparedQueryService()))
        {
            if(NativeFramePreflight)Refuse("Native FullFrame preflight reentrance");
            NativeConsumer=consumer;NativeFramePreflight=this;return;
        }
    }
    // This first guard supplies no positive contract, phase transition, ready
    // promotion or ability to publish/mutate. Future admission must additionally
    // bind every real phase/ordinal/epoch and recheck pending/order at effects.
    if(status==ECanonicalCensusRefusal::NoCompiledPhaseCoverage)Refuse("Prepared native owner ALL-phase contracts remain unqualified");
    Refuse("Prepared canonical raw census/pending/order refused before any adapter callback");
}
bool IsCurrentWorker(const FSimulationWorker* expected) noexcept {return !Poisoned&&expected&&CurrentWorker==expected;}
FWorkerCallScope::FWorkerCallScope(const FSimulationWorker& worker):Worker(&worker),ThreadId(FPlatformTLS::GetCurrentThreadId())
{if(Poisoned||CurrentWorker||HostScope||StepScope||MemberSweepScope||OwnerProbePhase||WheelProbeScope)Refuse("Prepared native worker call reentrance");CurrentWorker=Worker;}
FWorkerCallScope::~FWorkerCallScope() noexcept
{
    if(CurrentWorker!=Worker||ThreadId!=FPlatformTLS::GetCurrentThreadId()||HostScope||StepScope||MemberSweepScope||OwnerProbePhase||WheelProbeScope)RecordPreparedPoison("PreparedPhysicalExecution.cpp:210");
    CurrentWorker=nullptr;HostScope=nullptr;StepScope=nullptr;MemberSweepScope=nullptr;OwnerProbePhase=nullptr;WheelProbeScope=nullptr;
}
FHostCallScope::FHostCallScope(bool required,const ASpeedPhysicsSimulation& host,const USpeedWorldSubsystem& bridge,
    const FSimulationWorker* worker,uint64 generation,uint64 frame,float delta,float time,bool available)
{
    if(!required)return;
    if(Poisoned||HostScope||StepScope||MemberSweepScope||OwnerProbePhase||WheelProbeScope||!available||!IsCurrentWorker(worker)||!generation||frame>MAX_uint32||
        !FMath::IsFinite(delta)||delta<=0||!FMath::IsFinite(time)||host.GetInputWorkerGeneration()!=generation||
        !host.GetWorld()||host.GetWorld()!=bridge.GetWorld()||host.GetWorld()->bIsTearingDown)
        Refuse("Prepared native host/worker/World/input/frame context unavailable");
    Host=&host;Bridge=&bridge;World=host.GetWorld();Worker=worker;Generation=generation;Frame=frame;
    ThreadId=FPlatformTLS::GetCurrentThreadId();Delta=delta;SimTime=time;Active=true;HostScope=this;PoisonDiagnosticFrame=Frame;
}
FHostCallScope::~FHostCallScope() noexcept
{
    if(!Active)return;
    if(HostScope!=this||StepScope||MemberSweepScope||OwnerProbePhase||WheelProbeScope||ThreadId!=FPlatformTLS::GetCurrentThreadId())RecordPreparedPoison("PreparedPhysicalExecution.cpp:227");
    HostScope=nullptr;StepScope=nullptr;MemberSweepScope=nullptr;OwnerProbePhase=nullptr;WheelProbeScope=nullptr;
}
bool SameKinematicBits(const FKinematicState&,const FKinematicState&) noexcept;
#include "PreparedOrdinarySweepReturns.inl"
#include "PreparedOrdinaryWorldSelection.inl"
#include "PreparedOrdinaryResponseState.inl"
#include "PreparedOrdinaryIntegrationState.inl"
#include "PreparedNativeFrameState.inl"

FStepScope::FStepScope(bool required,const USpeedWorldSubsystem& bridge,const FSimulationWorld& registry,
    const IStaticCollisionWorld* provider,uint64 serial,uint32 frame,float delta,float time,
    TSharedPtr<const Analytic::Prepared::FScene,ESPMode::ThreadSafe> scene,
    TSharedPtr<const FPreparedPhysicalQueryDomain,ESPMode::ThreadSafe> filters)
{
    if(!required)return;
    const auto* host=HostScope;
    if(Poisoned||StepScope||MemberSweepScope||!host||!host->Active||host->Bridge!=&bridge||host->Frame!=uint64(frame)||
        !SameFloat(host->Delta,delta)||!SameFloat(host->SimTime,time)||host->ThreadId!=FPlatformTLS::GetCurrentThreadId()||
        !IsCurrentWorker(host->Worker)||!serial||!provider||bridge.GetStaticCollisionWorld()!=provider||
        registry.GetStaticCollisionWorld()!=provider||!scene||!filters||filters->GetGeneration()!=host->Generation)
        Refuse("Prepared native Step is not bound to its actual host call and input");
    const auto snapshot=provider->GetPreparedSnapshot();
    if(!snapshot||!snapshot->GetGeometrySelection()||!snapshot->GetGeometrySelection()->HasValidAliases()||snapshot->GetNominalScene()!=scene||snapshot->GetFilters()!=filters)
        Refuse("Prepared native Step snapshot/scene/filter aliases differ");
    if(snapshot->GetReference())
    {
        if(!OnePausedWork::ReadExecutionWindow(Window)||Window->GetSnapshot()!=snapshot||Window->GetExpectedStep()!=serial)
            Refuse("Prepared reference Step execution window absent or mixed");
        const auto& origin=snapshot->GetOrigin();
        if(origin.World!=host->World||origin.Host!=host->Host||origin.Bridge!=&bridge||
            origin.Generation!=host->Generation||snapshot->GetReference()->GetNominalScene()!=scene)
            Refuse("Prepared native canonical snapshot origin differs");
    }
    if(registry.GetAdapters().Num()!=filters->GetAdapterCount())Refuse("Prepared native adapter count changed");
    TArray<const void*> adapters;TArray<FPreparedPhysicalFilterRecord> records;
    adapters.Reserve(filters->GetAdapterCount());records.Reserve(filters->GetRecords().Num());
    for(auto* owner:registry.GetAdapters())
    {
        if(!owner)Refuse("Prepared native adapter missing");adapters.Add(owner);
        if(!registry.FindStableId(*owner)||registry.FindStableId(*owner)!=owner->GetPublishedSimulationStableId())Refuse("Prepared native owner registration identity unavailable");
        if(owner->GetSubBodies().Num()>filters->GetRecords().Num()-records.Num())Refuse("Prepared native body count grew");
        for(auto* body:owner->GetSubBodies())
        {
            if(!body||body->GetParentComponent()!=owner||body->GetWorld()!=host->World)Refuse("Prepared native body ownership changed");
            uint64 mask=0;for(int32 c=0;c<UE_ARRAY_COUNT(FCollisionResponseContainer::EnumArray);++c)
                if(body->GetResponseParams().CollisionResponse.GetResponse(ECollisionChannel(c))==ECR_Block)mask|=uint64(1)<<c;
            const FPreparedPhysicalFilterRecord* frozen=nullptr;
            for(const auto& entry:filters->GetRecords())if(entry.Owner==owner&&entry.Body==body){frozen=&entry;break;}
            if(!frozen)Refuse("Prepared native body is absent from frozen input");
            records.Add({owner,body,frozen->BodyPath,uint8(body->GetCollisionChannel()),mask});
        }
    }
    if(!filters->Matches(MoveTemp(records),adapters))Refuse("Prepared native body/filter domain changed since publication");
    const auto fp=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    Identity.Host=host->Host;Identity.Bridge=&bridge;Identity.World=host->World;Identity.Worker=host->Worker;
    Identity.Provider=provider;Identity.Lease=this;Identity.InputGeneration=host->Generation;Identity.Frame=frame;Identity.StepSerial=serial;Identity.QueryEpoch=1;
    if(Window){Identity.WindowSerial=Window->GetSerial();Identity.WindowNonce=Window->GetNonce();}
    Identity.NumericControl=fp.ControlFingerprint();Identity.ThreadId=host->ThreadId;Identity.Delta=delta;Identity.SimTime=time;
    if(Window&&Window->GetPurpose()==OnePausedWork::EPurpose::FullFrame)
    {
        if(!NativeFramePreflight||NativeFramePreflight->Snapshot!=snapshot||NativeFramePreflight->Window!=Window||
            NativeFramePreflight->PreparedOwners.Num()!=registry.GetOrderedAdapters().Num()||
            !NativeFramePreflight->NativeConsumer||
            !NativeFramePreflight->NativeConsumer->Matches(snapshot,provider->GetPreparedQueryService()))
            Refuse("Native FullFrame Step lacks returned preparation for its complete actual inventory");
        for(const auto* owner:registry.GetOrderedAdapters())if(!NativeFramePreflight->PreparedOwners.Contains(owner))
            Refuse("Native FullFrame Step preparation omitted an owner");
        NativeConsumer=NativeFramePreflight->NativeConsumer;
    }
    Registry=&registry;Snapshot=snapshot;Scene=MoveTemp(scene);Filters=MoveTemp(filters);Active=true;StepScope=this;PoisonDiagnosticStep=Identity.StepSerial;
}
FStepScope::~FStepScope() noexcept
{
    FNativePlanarResponseCommitScope::CloseStep(*this);
    if(!Active)return;
    if(Window&&OnePausedWork::HasFirstRsResponsePrefix(Window->GetPurpose()))
    {
        const auto& contract=Window->GetPreparationContract();
        if(contract&&contract->FirstRsResponse.Started&&!contract->FirstRsResponse.Complete)
        {contract->FirstRsResponse.Failed=true;RecordPreparedPoison("PreparedPhysicalExecution.cpp:306");}
    }
    if(Window&&OnePausedWork::HasFirstRsResponseSyncPrefix(Window->GetPurpose()))
    {
        const auto& contract=Window->GetPreparationContract();
        if(contract&&contract->FirstRsResponseSync.Attempted&&!contract->FirstRsResponseSync.Complete)
        {contract->FirstRsResponseSync.Failed=true;RecordPreparedPoison("PreparedPhysicalExecution.cpp:312");}
    }
    if(OrdinaryReturns&&(OrdinaryReturns->Pending||HasOpenOrdinaryWorldSelection()))RecordPreparedPoison("PreparedPhysicalExecution.cpp:314");
    if(OrdinaryResponse&&!OrdinaryResponse->Closed)RecordPreparedPoison("PreparedPhysicalExecution.cpp:315");
    if(OrdinaryIntegration&&!OrdinaryIntegration->Closed)RecordPreparedPoison("PreparedPhysicalExecution.cpp:316");
    if(NativeConsumer&&(!OrdinaryFinal||!OrdinaryFinal->Closed||(NativeAdvance&&!NativeAdvance->Consumed)))RecordPreparedPoison("PreparedPhysicalExecution.cpp:317");
    if(PostSyncRs&&PostSyncRs->Started&&(!PostSyncRs->Capture||!PostSyncRs->Capture->IsClosed()||PostSyncRs->OwnerActive))RecordPreparedPoison("PreparedPhysicalExecution.cpp:318");
    if(StepScope!=this||MemberSweepScope||OwnerProbePhase||WheelProbeScope||RsOwnerActive||(RsCapture&&!RsCapture->IsClosed())||Identity.ThreadId!=FPlatformTLS::GetCurrentThreadId())RecordPreparedPoison("PreparedPhysicalExecution.cpp:319");
#if !UE_BUILD_SHIPPING
    const bool terminal205=Poisoned&&std::uncaught_exceptions()>0&&StepScope==this&&
        !MemberSweepScope&&!OwnerProbePhase&&!WheelProbeScope;
#endif
    StepScope=nullptr;MemberSweepScope=nullptr;
#if !UE_BUILD_SHIPPING
    Analytic::Certified::Case205RemainderObservation::Drain(Identity,terminal205);
#endif
}
void FStepScope::BeginSweepIteration(int32 iteration,float remaining,float timePassed)
{
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
    if(!Active)return;
    const bool terminal=IsTerminalRsPurpose();
    if(Window&&Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame&&!terminal)
        Refuse("Terminal native Step sync grants no CCD iteration");
    const bool post=PostSyncRs&&PostSyncRs->Started;
    if(post&&(!HasPostSyncRsIdentity()||iteration!=2||Identity.Iteration!=1||Identity.QueryEpoch!=3||Identity.QuerySerial!=1||
        PostSyncRs->SupportCount!=3||PostSyncRs->OwnerActive||PostSyncRs->Capture||
        !SameFloat(timePassed,PostSyncRs->TimePassed)||!SameFloat(remaining,PostSyncRs->Horizon)||
        !PreparationPrefix::FCursor::IsCurrentPostSyncRsPrelude(*Identity.Bridge)))Refuse("Post-sync Rs actual iteration2 differs");
    if(!post&&terminal&&(iteration!=1||Identity.Iteration||RsSupportCount!=3||RsCapture||RsOwnerActive||
        !SameFloat(timePassed,0.f)||!SameFloat(remaining,Identity.Delta)||
        !PreparationPrefix::FCursor::IsCurrentRsPrelude(*Identity.Bridge)))
        Refuse("Terminal Rs purpose permits only its one exact initial CCD iteration");
    if(StepScope!=this||MemberSweepScope||iteration<=0||iteration<=Identity.Iteration||!FMath::IsFinite(remaining)||remaining<=0||
        !FMath::IsFinite(timePassed)||timePassed<Identity.TimePassed||timePassed>Identity.Delta||remaining>Identity.Delta||Identity.QueryEpoch==MAX_uint64)
        Refuse("Prepared native CCD iteration context unavailable");
    if(OrdinaryResponse&&!OrdinaryResponse->Closed)Refuse("Ordinary response batch remained open before next sweep");
#if !UE_BUILD_SHIPPING
    CaptureFixedNoPoseSweepBaseline(iteration,remaining,timePassed);
#endif
    OrdinaryResponse.Reset();
    if(OrdinaryIntegration&&!OrdinaryIntegration->Closed)Refuse("Ordinary integration remained open before next sweep");
    OrdinaryIntegration.Reset();
    if(NativeAdvance&&!NativeAdvance->Consumed)Refuse("Unused native advance ticket before next sweep");
    NativeAdvance.Reset();
    ResetOrdinaryOwnerSweepReturns();
    ++Identity.QueryEpoch;SweepPhase=true;Identity.Iteration=iteration;Identity.Remaining=remaining;Identity.TimePassed=timePassed;
}
void FStepScope::InvalidateQueries()
{
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
    if(!Active)return;
    if(StepScope!=this||MemberSweepScope||Identity.QueryEpoch==MAX_uint64)Refuse("Prepared native query epoch unavailable");
    if(OrdinaryReturns&&OrdinaryReturns->Pending)Refuse("Ordinary owner sweep is still active at query invalidation");
    ++Identity.QueryEpoch;SweepPhase=false;
}
bool ReadCurrentStep(FStepIdentity& out) noexcept
{
    out={};const auto* s=StepScope;
    if(Poisoned)return false;
    // Revocation is checked before the following native/UObject reads.
    if(s&&s->Window)
    {
        if(!s->Window->IsOpen())return false;
        if(s->Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame&&
            !(OnePausedWork::HasSphereRsCapturePrefix(s->Window->GetPurpose())&&
              (PreparationPrefix::FCursor::IsCurrentRsNativeWindow(*s->Identity.Bridge)||
               (s->IsPostSyncRsPurpose()&&s->HasPostSyncRsIdentity()))))return false;
        OnePausedWork::FCanonicalFrameWindowPtr current;
        if(!OnePausedWork::ReadExecutionWindow(current)||current!=s->Window||s->Identity.WindowSerial!=current->GetSerial()||s->Identity.WindowNonce!=current->GetNonce())return false;
    }
    if(Poisoned||!s||!s->Active||!HostScope||!IsCurrentWorker(s->Identity.Worker)||
        s->Identity.ThreadId!=FPlatformTLS::GetCurrentThreadId()||s->Identity.Host->GetInputWorkerGeneration()!=s->Identity.InputGeneration||
        s->Identity.Host->GetWorld()!=s->Identity.World||s->Identity.Bridge->GetWorld()!=s->Identity.World||
        s->Identity.Bridge->GetStaticCollisionWorld()!=s->Identity.Provider||s->Registry->GetStaticCollisionWorld()!=s->Identity.Provider||
        !s->Snapshot||!s->Snapshot->GetGeometrySelection()||!s->Snapshot->GetGeometrySelection()->HasValidAliases()||s->Identity.Provider->GetPreparedSnapshot()!=s->Snapshot||
        s->Snapshot->GetNominalScene()!=s->Scene||s->Snapshot->GetFilters()!=s->Filters)return false;
    const auto fp=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(fp.Status!=Analytic::EAnalyticNumericStatus::Complete||fp.ControlFingerprint()!=s->Identity.NumericControl)return false;
    out=s->Identity;return true;
}
bool NextSphereQuery(FStepIdentity& out) noexcept
{
    if(FNativePlanarResponseCommitScope::HasUnclosedBodySync()){out={};RecordPreparedPoison("planar_query_serial_before_actual_sync");return false;}
    out={};FStepIdentity current;if(!ReadCurrentStep(current)||!StepScope->SweepPhase||current.Iteration<=0||current.QuerySerial>=MAX_uint64-1)return false;
    const bool post=StepScope->PostSyncRs&&StepScope->PostSyncRs->Started;
    if(post)
    {
        const auto& slot=*StepScope->PostSyncRs;
        if(!StepScope->HasPostSyncRsIdentity()||current.QuerySerial!=1||current.Iteration!=2||current.QueryEpoch!=4||
            !slot.OwnerActive||slot.SweepOwners!=1||!slot.Owner||
            !PreparationPrefix::FCursor::IsCurrentPostSyncRsSweepSite(*slot.Owner,1))return false;
    }
    else if(StepScope->IsTerminalRsPurpose()&&(current.QuerySerial||current.Iteration!=1||
        !StepScope->RsOwnerActive||StepScope->RsSweepOwners!=1||!StepScope->RsOwner||
        !PreparationPrefix::FCursor::IsCurrentRsSweepSite(*StepScope->RsOwner,1)))return false;
    ++StepScope->Identity.QuerySerial;out=StepScope->Identity;return true;
}
bool ReadCurrentSnapshot(TSharedPtr<const FPreparedWorldSnapshot,ESPMode::ThreadSafe>& out) noexcept
{
    out.Reset();FStepIdentity current;
    if(!ReadCurrentStep(current)||!StepScope->Snapshot)return false;
    out=StepScope->Snapshot;return true;
}
bool ReadCurrentInputs(TSharedPtr<const Analytic::Prepared::FScene,ESPMode::ThreadSafe>& scene,
    TSharedPtr<const FPreparedPhysicalQueryDomain,ESPMode::ThreadSafe>& filters) noexcept
{
    scene.Reset();filters.Reset();FStepIdentity current;if(!ReadCurrentStep(current))return false;
    scene=StepScope->Scene;filters=StepScope->Filters;return true;
}
uint64 FStepScope::ReadOwnerStableId(const void* owner) noexcept
{
    FStepIdentity current;if(!ReadCurrentStep(current)||!owner)return 0;
    const auto* component=static_cast<const ISpeedComponent*>(owner);
    return StepScope->Registry->FindStableId(*component);
}
bool IsSupportedSphereResponseMargin(float margin) noexcept
{
    uint32 bits=0;FMemory::Memcpy(&bits,&margin,4);const uint32 magnitude=bits&0x7fffffffu;
    return magnitude<0x7f800000u&&(!magnitude||(!(bits&0x80000000u)&&magnitude>=0x00800000u));
}
bool FSphereMemberSweepScope::ReadBody(const USolidSubBody* raw,const USphereSubBody*& body,const void*& owner) noexcept
{
    body=nullptr;owner=nullptr;const auto* scope=MemberSweepScope;uint64 serial=0;float margin=0;
    // Upcast only the member-owned live pointer, never Cast/dereference raw.
    if(!scope||!scope->Active||scope->Completed||scope->Interrupted||!raw||
        static_cast<const USolidSubBody*>(scope->Body)!=raw||
        !ReadSphereMemberSweep(scope->Body,scope->Owner,serial,margin))return false;
    body=scope->Body;owner=scope->Owner;return true;
}
FSphereMemberSweepScope::FSphereMemberSweepScope(const USphereSubBody* body,const void* owner,float margin,const FNativeRsTraceValues* trace)
{
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
    FStepIdentity step;
    if(!Requested()||Poisoned||MemberSweepScope||!body||!owner||!ReadCurrentStep(step)||
        !StepScope->SweepPhase||step.Iteration<=0||MemberSweepSerial==MAX_uint64||!IsSupportedSphereResponseMargin(margin))
        Refuse("Prepared sphere member sweep has no authentic Step/body/margin scope");
    const bool terminal=StepScope->IsTerminalRsPurpose();
    const bool post=StepScope->PostSyncRs&&StepScope->PostSyncRs->Started;
    const bool validTerminal=post?(StepScope->PostSyncRs->OwnerActive&&StepScope->PostSyncRs->Owner==owner&&
        StepScope->PostSyncRs->SweepOwners==1&&StepScope->PostSyncRs->ReactionObserved&&StepScope->HasPostSyncRsIdentity()):
        (StepScope->RsOwnerActive&&StepScope->RsOwner==owner&&StepScope->RsSweepOwners==1&&StepScope->RsReactionObserved);
    if(terminal!=bool(trace)||(terminal&&!validTerminal))
        Refuse("Terminal Rs member requires its actual protected trace-policy capture");
    if(trace)Trace=MakeShared<FNativeRsTraceValues,ESPMode::ThreadSafe>(*trace);
    Body=body;Owner=owner;Step=step;Margin=margin;Serial=++MemberSweepSerial;Active=true;MemberSweepScope=this;
}
void FSphereMemberSweepScope::Finish(float actualMargin,bool deliveredHit,const SHitResult& delivered,float deliveredSeconds)
{
    uint64 serial=0;float captured=0;
    if(!Active||Completed||Interrupted||!ReadSphereMemberSweep(Body,Owner,serial,captured)||serial!=Serial||
        !IsSupportedSphereResponseMargin(actualMargin)||!SameFloat(actualMargin,Margin))
        Refuse("Prepared sphere virtual response margin or member context changed during sweep");
    if(PendingSleepingSupport){FinishSleepingSupport(deliveredHit,delivered,deliveredSeconds);Completed=true;return;}
    if(PendingOrdinaryGround)
    {
        FinishOrdinaryGround(deliveredHit,delivered,deliveredSeconds);Completed=true;return;
    }
    if(!PendingGround||!PendingGround->Request||!PendingGround->Geometry||
        PendingGround->Geometry!=PendingGround->Request->GetGeometrySelection()||!RevalidateSphereCCD(*PendingGround->Request)||!ComponentSweepScope)
        Refuse("Prepared sphere member has no complete pending ground result");
    if(PendingGround->Kind==FGroundSweepEvidence::EKind::WholeHullExclusion)
    {
        if(!PendingGround->Partition||PendingGround->Partition->GetScene()!=PendingGround->Geometry->GetSelectedUnion()||
            !PendingGround->Partition->GetCandidateIndices().IsEmpty()||deliveredHit||delivered.bHit||delivered.PreparedGroundEvidence||
            !SamePreparedHitValues(delivered,SHitResult{})||!SameFloat(deliveredSeconds,PendingGround->Request->GetValues().Horizon))Refuse("Prepared empty ground delivery changed");
    }
    else if(PendingGround->Kind==FGroundSweepEvidence::EKind::FirstContact)
    {
        if(!PendingGround->Quality||!deliveredHit||!delivered.bHit||delivered.PreparedGroundEvidence.Get()!=PendingGround.Get()||
            !SameFloat(deliveredSeconds,PendingGround->PendingHit.TOI)||!SamePreparedHitValues(delivered,PendingGround->PendingHit))
            Refuse("Prepared first contact delivered bits changed before native member seal");
    }
    else Refuse("Prepared ground result kind unavailable");
    // Copy a derived state seed before revoking the member request. The seed
    // intentionally has no Ground/Request backpointer and no advance purpose.
    TSharedPtr<FSealedSphereSweepState,ESPMode::ThreadSafe> state(new FSealedSphereSweepState);
    state->Step=PendingGround->Request->GetStep();state->Values=PendingGround->Request->GetValues();
    state->Geometry=PendingGround->Geometry;state->Snapshot=PendingGround->Request->GetSnapshot();
    state->Reference=PendingGround->Request->GetReference();state->Quality=PendingGround->Quality;
    const auto* actualOwner=static_cast<const ISpeedComponent*>(Owner);
    FCanonicalPreparedFrameClock actualClock;
    if(!JoinCanonicalPreparedFrameClock(state->Step.Frame,actualOwner->NumFrame(),actualClock)||
        !(actualClock==state->Values.FrameClock))Refuse("Prepared sweep seed owner-local/canonical frame changed");
    state->OwnerHistoryAtFrame=actualOwner->GetKinematicStateForFrame(actualClock.OwnerLocalFrame);
    state->NativeLocalInputSupported=FrameParallel::CaptureNativeSubBodyKinematicInput(*Body,state->NativeLocalInput);
    // The game producer binds the exact derived child implementation to a
    // genuine GT object/interface association. No broad IsA promotion, GT
    // lookup on worker, advance purpose or lifetime extension follows here.
    if(!state->NativeLocalInputSupported&&state->Snapshot->GetNativeOwners()&&
        state->Snapshot->GetNativeOwners()->GetGeneration()==state->Step.InputGeneration)
        state->NativeLocalInputSupported=state->Snapshot->GetNativeOwners()->CaptureSphereInput(
            static_cast<const ISpeedComponent*>(Owner),*Body,state->NativeLocalInput);
    state->NamedGround=delivered;state->NamedGround.PreparedGroundEvidence.Reset(); // no shared-pointer cycle
    state->GroundHit=deliveredHit;state->GroundSeconds=deliveredSeconds;
    state->WholeHullExclusion=PendingGround->Kind==FGroundSweepEvidence::EKind::WholeHullExclusion;
    if(!SameKinematicBits(state->Values.OwnerState,static_cast<const ISpeedComponent*>(Owner)->GetKinematicState())||
        !SameKinematicBits(state->Values.BodyState,Body->GetKinematicState())||!RevalidateSphereCCD(*PendingGround->Request))
        Refuse("Prepared derived sweep-state seed changed during capture");
    PendingGround->StateSeed=MoveTemp(state);
    // No seed makes RevalidateSphereCCD valid after Completed below.
    PendingGround->Sealed=true;
    ComponentSweepScope->SealGround(PendingGround);
    PendingGround.Reset();
    Completed=true;
}
FSphereMemberSweepScope::~FSphereMemberSweepScope() noexcept
{
    if(!Active)return;
    const bool closedInterrupt=HasClosedInterruptedPostSyncRsMember()||(Interrupted&&!Completed&&StepScope&&StepScope->IsTerminalRsPurpose()&&
        StepScope->RsCapture&&StepScope->RsCapture->Stage==FNativeSphereRsTrajectoryCapture::EStage::MemberClosed&&
        StepScope->RsCapture->Request&&StepScope->RsCapture->Request->GetValues().Body==Body&&
        StepScope->RsCapture->Request->GetValues().ResponseScopeSerial==Serial);
    if(MemberSweepScope!=this||(!Completed&&!closedInterrupt)||Step.ThreadId!=FPlatformTLS::GetCurrentThreadId())RecordPreparedPoison("PreparedPhysicalExecution.cpp:511");
    MemberSweepScope=nullptr;Active=false;
}
bool ReadSphereMemberSweep(const USphereSubBody* body,const void* owner,uint64& serial,float& margin) noexcept
{
    serial=0;margin=0;const auto* scope=MemberSweepScope;FStepIdentity now;
    // Only pointer equality is used before authentic Step/epoch/lane binding.
    // Never dereference a stale retained body merely to discover its owner.
    if(Poisoned||!scope||!scope->Active||scope->Completed||scope->Interrupted||!body||!owner||scope->Body!=body||scope->Owner!=owner||
        !scope->Serial||!ReadCurrentStep(now))return false;
    const auto& saved=scope->Step;
    if(now.Lease!=saved.Lease||now.Host!=saved.Host||now.Bridge!=saved.Bridge||now.World!=saved.World||now.Worker!=saved.Worker||
        now.Provider!=saved.Provider||now.InputGeneration!=saved.InputGeneration||now.Frame!=saved.Frame||
        now.WindowSerial!=saved.WindowSerial||now.WindowNonce!=saved.WindowNonce||now.StepSerial!=saved.StepSerial||now.QueryEpoch!=saved.QueryEpoch||now.Iteration!=saved.Iteration||
        now.NumericControl!=saved.NumericControl||now.ThreadId!=saved.ThreadId||
        !SameFloat(now.Delta,saved.Delta)||!SameFloat(now.SimTime,saved.SimTime)||
        !SameFloat(now.Remaining,saved.Remaining)||!SameFloat(now.TimePassed,saved.TimePassed)||
        !IsSupportedSphereResponseMargin(scope->Margin))return false;
    serial=scope->Serial;margin=scope->Margin;return true;
}
bool SameKinematicBits(const FKinematicState& a,const FKinematicState& b) noexcept
{
    return SameVector(a.Location,b.Location)&&SameVector(a.Velocity,b.Velocity)&&SameVector(a.Acceleration,b.Acceleration)&&
        SameQuat(a.Rotation,b.Rotation)&&SameVector(a.AngularVelocity,b.AngularVelocity)&&SameVector(a.AngularAcceleration,b.AngularAcceleration);
}
#include "PreparedNativeSolidRequest.inl"
#include "PreparedNativeWheelProbe.inl"
#include "NativeFloorSphereContactGeometry.inl"
#include "NativeFloorBoxPlaneValues.inl"
#include "PreparedOrdinaryNativeGround.inl"
#include "PreparedBoxGroundReuse.inl"
#include "PreparedOrdinaryFrameStateValues.inl"
#include "PreparedOrdinaryPrimitiveValues.inl"
namespace GeneralBoxTransportDetail
{
bool Pose(const FKinematicState& a,const FKinematicState& b)
{return SameVector(a.Location,b.Location)&&SameQuat(a.Rotation,b.Rotation);}
bool Configuration(const FNativeSolidStaticValues& source)
{
    ::PreparedFrameworkProfile1::FScope observedFramework(::PreparedFrameworkProfile1::Configuration);
    const auto* owner=source.Owner;const auto* body=source.Body;const auto* box=Cast<UBoxSubBody>(body);
    if(!owner||!body||!box||!source.Sweeper||source.Frozen||source.Prescribed||owner->IsFrozen()||owner->IsPrescribedMotion()||
        body->GetClass()!=source.BodyClass||body->GetParentComponent()!=owner||
        !owner->GetSubBodies().IsValidIndex(source.BodyIndex)||owner->GetSubBodies()[source.BodyIndex]!=body||
        !SameVector(owner->GetPhysCenterOfMassLocal(),source.LocalCOM)||
        !SameVector(body->GetLocalOffset(),source.LocalOffset)||
        !SameQuat(body->GetLocalRotation(),source.LocalRotation)||
        !SameVector(box->GetBoxExtent(),source.PhysicalHalfExtent))return false;
    const auto shape=source.Sweeper->GetWorldStaticCollisionShape();
    if(!shape.IsBox()||!SameVector(FVector(shape.GetBox()),source.QueryHalfExtent))return false;
    FNativeSolidTraceValues trace;
    if(!CaptureNativeSolidTraceValues(*source.Sweeper,source.Sweeper->BuildTraceParams(),trace)||!trace.Same(source.Trace))return false;
    FNativeSolidResponseConfigValues config;
    return NativeSolidDetail::ReadResponseConfig(*body,*owner,config)&&SameNativeSolidResponseConfigValues(config,source.ResponseConfig);
}
bool Within(const C::FIntervalPoint& outer,const C::FIntervalPoint& inner)
{
    if(!C::Detail::ValidPoint(outer)||!C::Detail::ValidPoint(inner))return false;
    for(int32 d=0;d<3;++d)if(inner.Axis[d].Lower<outer.Axis[d].Lower||inner.Axis[d].Upper>outer.Axis[d].Upper)return false;
    return true;
}
bool Endpoint(const FNativeSolidStaticValues& source,const C::FIntervalPoint& envelope)
{
    ::PreparedFrameworkProfile1::FScope observedFramework(::PreparedFrameworkProfile1::GeneralEndpoint);
    if(!Configuration(source)||!C::Detail::ValidPoint(envelope))return false;
    const auto actual=source.Body->GetKinematicState();auto pose=source.Query;
    pose.Start=pose.End=actual.Location;pose.Rotation=actual.Rotation;
    for(const auto extent:{source.PhysicalHalfExtent,source.QueryHalfExtent})
    {
        pose.HalfExtent=extent;C::FIntervalPoint volume;
        if(!C::BuildFixedWorldQueryVolumeBounds(pose,volume)||!Within(envelope,volume))return false;
    }
    return true;
}
bool Endpoint(const FNativeSolidStaticValues& source,const FNativeBoxPrimitiveGeometryValuesPtr& proof)
{
    ::PreparedFrameworkProfile1::FScope observedFramework(::PreparedFrameworkProfile1::GeneralEndpoint);
    if(!proof||!Configuration(source))return false;
    const auto actual=source.Body->GetKinematicState();auto pose=source.Query;
    pose.Start=pose.End=actual.Location;pose.Rotation=actual.Rotation;
    return proof->ContainsEndpoint(pose);
}
bool Endpoint(const FPreparedWorldSnapshotPtr& snapshot,const FOrdinaryIntegrationOwnerValues& row,int32 index)
{
    ::PreparedFrameworkProfile1::FScope observedFramework(::PreparedFrameworkProfile1::GeneralEndpoint);
    if(!row.GeneralBindings.IsValidIndex(index)||!row.GeneralProofs.IsValidIndex(index))return false;
    if(!row.GeneralBindings[index].Request)return false;
    const auto& source=row.GeneralBindings[index].Request->GetValues();
    const auto& binding=row.GeneralBindings[index];
    if(binding.FloorMotion)
    {
        if(!row.LatestPoseCorrections.IsEmpty()||!binding.HasExactlyOneKind()||!Configuration(source))return false;
        const auto actual=source.Body->GetKinematicState();auto pose=source.Query;
        pose.Start=pose.End=actual.Location;pose.Rotation=actual.Rotation;
        return row.GeneralProofs[index]&&row.GeneralProofs[index]->ContainsFloorMotionEndpoint(pose,binding.FloorMotionWorldSeconds,
            binding.FloorMotion->GetGeneralFloorMotionDescriptorForInspection());
    }
    if(row.LatestPoseCorrections.IsEmpty())return Endpoint(source,row.GeneralProofs[index]);
    if(row.LatestPoseCorrections.Num()!=row.GeneralBindings.Num()||!row.LatestPoseCorrections[index]||
        !row.LatestPoseCorrections[index]->MatchesEndpoint(snapshot,source,row.GeneralExpected))return false;
    if(!Configuration(source))return false;
    const auto actual=source.Body->GetKinematicState();auto pose=source.Query;
    pose.Start=pose.End=actual.Location;pose.Rotation=actual.Rotation;
    return row.LatestPoseCorrections[index]->ContainsEndpoint(pose);
}
void ClosedSetters(const FOrdinaryIntegrationOwnerValues& row,bool requireSync)
{
    if(row.OpenPoseSetter||row.PoseAssignmentsBegun!=row.PoseAssignmentsCompleted||
        (requireSync&&row.PoseVersion!=row.SyncVersion))Refuse("General Box pose assignment or subsequent real child sync remains open");
}
const FOrdinaryConsumedOwnerValues* Find(const FOrdinaryConsumedStateValues& values,const ISpeedComponent* owner)
{for(const auto& row:values.Owners)if(row.Owner==owner)return &row;return nullptr;}
bool SamePoseAndBoxConfiguration(const FOrdinaryConsumedOwnerValues& a,const FOrdinaryConsumedOwnerValues& b)
{
    if(a.Owner!=b.Owner||a.StableId!=b.StableId||a.Frozen!=b.Frozen||a.Prescribed!=b.Prescribed||!Pose(a.State,b.State)||
        !SameVector(a.LocalCOM,b.LocalCOM)||!SameQuat(a.PhysRotation,b.PhysRotation)||
        a.Bodies.Num()!=b.Bodies.Num())return false;
    for(int32 i=0;i<a.Bodies.Num();++i)
    {
        const auto& x=a.Bodies[i];const auto& y=b.Bodies[i];
        if(x.BoxShapeCaptured!=y.BoxShapeCaptured)return false;if(!x.BoxShapeCaptured)continue;
        if(x.Body!=y.Body||x.Index!=y.Index||x.Class!=y.Class||x.GTObjectId!=y.GTObjectId||!Pose(x.State,y.State)||
            !SameVector(x.LocalOffset,y.LocalOffset)||!SameQuat(x.LocalRotation,y.LocalRotation)||
            !SameVector(x.BoxExtent,y.BoxExtent)||!x.SolidConfigCaptured||!y.SolidConfigCaptured||
            !SameNativeSolidResponseConfigValues(x.Config,y.Config))return false;
    }
    return true;
}
}

#include "PreparedNativeOwnerPoseMutation.inl"

void FStepScope::RequireGeneralBoxUnchanged(const FOrdinaryConsumedStateValues& before,const FOrdinaryConsumedStateValues& after,
    const ISpeedComponent* except) const
{
    if(GeneralBoxOwners.IsEmpty())return;
    if(!before.Captured||!after.Captured)Refuse("General Box operation has no complete actual before/after values");
    for(const auto* owner:GeneralBoxOwners)if(owner!=except)
    {
        const auto* a=GeneralBoxTransportDetail::Find(before,owner);const auto* b=GeneralBoxTransportDetail::Find(after,owner);
        if(!a||!b||!GeneralBoxTransportDetail::SamePoseAndBoxConfiguration(*a,*b))
            Refuse("General Box final/other-owner operation changed an uncovered pose or shape");
    }
}
void FStepScope::ObserveGeneralBoxStage(FOrdinaryIntegrationOwnerValues& row,const FOrdinaryIntegrationStageValues& stage)
{
    if(row.GeneralBindings.IsEmpty())return;
    RequireOrdinaryIntegrationContext();auto& tx=*OrdinaryIntegration;
    GeneralBoxTransportDetail::ClosedSetters(row,stage.Stage!=ENativeIntegrationStage::PrimitiveEnter&&stage.Stage!=ENativeIntegrationStage::PrimitiveReturn);
    if(!NativeAdvance||!NativeAdvance->Consumed||!tx.SelectorJoined||row.Owner!=tx.Owners[tx.ActiveIndex].Owner||
        tx.Restoring!=(stage.Stage==ENativeIntegrationStage::ReactionRestored))Refuse("General Box stage lost consumed selector/owner");
    for(const auto& binding:row.GeneralBindings)
        if(!binding.Request||binding.Request->GetSnapshot()!=Snapshot||!binding.HasExactlyOneKind()||!GeneralBoxTransportDetail::Configuration(binding.Request->GetValues())||
            Registry->FindStableId(*row.Owner)!=binding.Request->GetValues().OwnerStableId)
            Refuse("General Box primitive intrinsic binding changed");
    if(stage.Stage==ENativeIntegrationStage::PrimitiveEnter)
    {
        if(row.GeneralOpen||row.GeneralNext!=stage.Index||stage.Count<1||stage.Count>4||stage.Index>=stage.Count||
            !FMath::IsFinite(stage.Delta)||stage.Delta<=0||!SameFloat(stage.Delta,tx.Seconds/static_cast<float>(stage.Count))||
            (row.GeneralCount&&row.GeneralCount!=stage.Count)||row.Stages.Num()!=3*stage.Index+1)
            Refuse("General Box actual primitive sequence is incomplete or repeated");
        const auto* entry=GeneralBoxTransportDetail::Find(row.Before,row.Owner);
        if(!entry||!GeneralBoxTransportDetail::Pose(stage.OwnerState,row.GeneralNext?row.GeneralExpected:entry->State))
            Refuse("General Box pose changed between covered primitive segments");
        row.GeneralProofs.Reset();row.LatestPoseCorrections.Reset();const auto service=Identity.Provider->GetPreparedQueryService();
        if(!service||service->GetPreparedSnapshot()!=Snapshot)Refuse("General Box primitive lacks current retained service");
        for(const auto& binding:row.GeneralBindings)
        {
            FNativeBoxPrimitiveGeometryValuesPtr proof;
            if(binding.FloorMotion&&!SameFloat(binding.FloorMotionWorldSeconds,tx.Seconds))
                Refuse("General FLOOR primitive world time differs from the consumed actual selector");
            const bool proved=binding.FloorMotion?
                service->DescribeNativeGeneralBoxFloorMotionPrimitiveGeometryValues(binding.Request->GetValues(),stage.OwnerState,stage.Delta,
                    tx.Seconds,binding.FloorMotion->GetGeneralFloorMotionDescriptorForInspection(),proof):binding.Released?
                service->DescribeNativeReleasedBoxFloorPrimitiveGeometryValues(binding.Request->GetValues(),stage.OwnerState,stage.Delta,
                    binding.Released->GetReleasedFloorDescriptorForInspection(),proof):
                service->DescribeNativeBoxPrimitiveGeometryValues(binding.Request->GetValues(),stage.OwnerState,stage.Delta,proof,binding.Prediction);
            if(!proved||!proof)Refuse("General Box actual primitive lacks its distinct complete geometry proof");
            row.GeneralProofs.Add(MoveTemp(proof));
        }
        RequireOrdinaryIntegrationContext();row.GeneralExpected=stage.OwnerState.Integrate(stage.Delta);
        if(!FiniteState(row.GeneralExpected))Refuse("General Box primitive expected actual values are nonfinite");
        row.GeneralCount=stage.Count;row.GeneralOpen=true;row.GeneralReturned=false;row.GeneralSyncs=0;
    }
    else if(stage.Stage==ENativeIntegrationStage::PrimitiveReturn)
    {
        if(!row.GeneralOpen||row.GeneralReturned||stage.Index!=row.GeneralNext||stage.Count!=row.GeneralCount||
            !SameFloat(stage.Delta,tx.Seconds/static_cast<float>(row.GeneralCount))||row.Stages.Num()!=3*stage.Index+2||
            !SameKinematicBits(stage.OwnerState,row.GeneralExpected))Refuse("General Box actual primitive differs from covered native integration");
        row.GeneralReturned=true;
    }
    else
    {
        if(!GeneralBoxTransportDetail::Pose(stage.OwnerState,row.GeneralExpected)||row.GeneralProofs.Num()!=row.GeneralBindings.Num())
            Refuse("General Box post/projection/restoration changed its covered endpoint pose");
        for(int32 i=0;i<row.GeneralBindings.Num();++i)
            if(!GeneralBoxTransportDetail::Endpoint(Snapshot,row,i))
                Refuse("General Box actual synchronized endpoint escaped its primitive bounds");
        if(stage.Stage==ENativeIntegrationStage::ProjectionReturn)
        {
            bool allReleased=true;for(const auto& binding:row.GeneralBindings)allReleased&=binding.Released.IsValid()||binding.FloorMotion.IsValid();
            if(!row.GeneralOpen||!row.GeneralReturned||!row.GeneralSyncs||(stage.Projected&&!allReleased)||stage.Index!=row.GeneralNext||
                stage.Count!=row.GeneralCount||!SameFloat(stage.Delta,tx.Seconds/static_cast<float>(row.GeneralCount))||
                row.Stages.Num()!=3*stage.Index+3)Refuse("General Box actual projection is uncovered or child sync missing");
            row.GeneralOpen=false;++row.GeneralNext;
        }
        else if((stage.Stage!=ENativeIntegrationStage::PostReturn&&stage.Stage!=ENativeIntegrationStage::ReactionRestored)||
            row.GeneralOpen||row.GeneralNext!=row.GeneralCount||!row.GeneralCount||!SameFloat(stage.Delta,tx.Seconds)||
            stage.Index!=INDEX_NONE||stage.Count)Refuse("General Box nontransport or incomplete actual path");
    }
}
void FStepScope::ObserveGeneralBoxChildSync(FOrdinaryIntegrationOwnerValues& row)
{
    if(row.GeneralBindings.IsEmpty())return;
    GeneralBoxTransportDetail::ClosedSetters(row,false);
    // Same four predicates in the same short-circuit order. Their failure
    // messages distinguish absent coverage, changed pose, and actual ordering.
    const auto refuse=[&](const char* why)
    {
        const auto* sync=row.ChildSync.IsEmpty()?nullptr:&row.ChildSync.Last();
        const int32 last=row.Stages.IsEmpty()?-1:int32(row.Stages.Last().Stage);
        Refuse(TCHAR_TO_UTF8(*FString::Printf(TEXT("%hs: proofs=%d bindings=%d count=%d next=%d open=%d returned=%d stages=%d lastStage=%d post=%d projection=%d restores=%d/%d"),
            why,row.GeneralProofs.Num(),row.GeneralBindings.Num(),row.GeneralCount,row.GeneralNext,int32(row.GeneralOpen),int32(row.GeneralReturned),
            row.Stages.Num(),last,int32(sync&&sync->InExactPostProbe),int32(sync&&sync->InExactIntegrationProjection),row.RestorationAttempts,row.RestorationReturns)));
    };
    if(row.ChildSync.IsEmpty()||!row.ChildSync.Last().Captured)refuse("General Box child sync has no complete actual capture");
    if(row.GeneralProofs.Num()!=row.GeneralBindings.Num())refuse("General Box child sync lacks its actual primitive geometry");
    if(!GeneralBoxTransportDetail::Pose(row.ChildSync.Last().OwnerState,row.GeneralExpected))
    {
        const auto& actual=row.ChildSync.Last().OwnerState;
        try{UE_LOG(LogTemp,Display,TEXT("[GeneralBoxChildSyncPoseDifference] ActualP=(%.17g,%.17g,%.17g) ExpectedP=(%.17g,%.17g,%.17g) ActualQ=(%.17g,%.17g,%.17g,%.17g) ExpectedQ=(%.17g,%.17g,%.17g,%.17g)"),
            actual.Location.X,actual.Location.Y,actual.Location.Z,row.GeneralExpected.Location.X,row.GeneralExpected.Location.Y,row.GeneralExpected.Location.Z,
            actual.Rotation.X,actual.Rotation.Y,actual.Rotation.Z,actual.Rotation.W,row.GeneralExpected.Rotation.X,row.GeneralExpected.Rotation.Y,row.GeneralExpected.Rotation.Z,row.GeneralExpected.Rotation.W);}catch(...){}
        refuse("General Box child sync changed its covered owner pose");
    }
    if(row.GeneralOpen&&!row.GeneralReturned)refuse("General Box child sync occurred before its real primitive return");
    // No geometry evaluation or additional native call in a noexcept cleanup.
    // The safe projection/Post/owner-return boundary checks the copied values
    // and the actual endpoints; any failed capture poisons before it can close.
    ++row.GeneralSyncs;row.SyncVersion=row.PoseVersion;
}

namespace OrdinarySetFutureDetail
{
bool ExactZeroDelta(float) noexcept;
bool Unchanged(const FOrdinaryConsumedStateValues&,const FOrdinaryConsumedStateValues&);
}
#include "PreparedOrdinaryIntegration.inl"
#include "PreparedOrdinarySetFutureHitValues.inl"
#include "PreparedOrdinaryResponseConsume.inl"

#if !UE_BUILD_SHIPPING
void FStepScope::CaptureFixedNoPoseSweepBaseline(int32 iteration,float remaining,float timePassed)
{
    FixedNoPoseSweepBaseline.Reset();
    if(CVarNativeRoofFixedCachedFloorArrival.GetValueOnAnyThread()!=1||!OrdinaryResponse||
        !OrdinaryIntegration||!OrdinaryIntegration->Closed||!Window||
        Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame||SweepPhase||OwnerProbePhase||
        Identity.Frame!=3||Identity.StepSerial!=4||Identity.Iteration!=2||iteration!=3)return;
    const auto& tx=*OrdinaryResponse;FStepIdentity now;
    if(!tx.Closed||tx.Snapshot!=Snapshot||tx.Entries.Num()!=1||tx.Selected.Num()!=1||tx.NextIndex!=1||
        tx.ActiveIndex!=INDEX_NONE||tx.ActiveCall!=INDEX_NONE||!SameFloat(tx.ActualAfter,timePassed)||
        !ReadCurrentStep(now)||!OrdinaryWorldDetail::Epoch(tx.EntryStep,now)||
        now.QuerySerial!=tx.EntryStep.QuerySerial||now.QueryEpoch==MAX_uint64)return;
    const auto& row=tx.Entries[0];
    if(!row.Closed||!row.Eligible||!row.Allowed||row.Ignored||!row.PairBefore||!row.PairAfter||
        row.GenerationBefore!=1||row.GenerationAfter!=2||row.Selected.PairKey!=6880767498340045620ull||
        !row.RawResolver||row.Calls.Num()!=3||row.Calls[2].PlanarEffect||!row.Entry.Captured||!row.After.Captured||
        C::KinematicCenterDetail::Bits(row.Selected.Hit.TOI)!=0x34a69593u||
        !SameFloat(tx.Seconds,row.Selected.Hit.TOI))return;
    for(int32 i=0;i<3;++i)
        if(row.Calls[i].Call!=i||!row.Calls[i].Closed||!row.Calls[i].Returned)return;
    const auto& inventory=row.Selected.PreparedEvidence;
    if(!inventory||!inventory->Sealed||!inventory->Bodies.IsValidIndex(inventory->SelectedBodyIndex))return;
    const auto& body=inventory->Bodies[inventory->SelectedBodyIndex];const auto& ground=body.OrdinaryGround;
    if(body.RawBodyIdentity!=row.RawResolver||!ground||!ground->IsFirstGeneralBoxFloorContactValues()||
        !ground->HasMatchingFirstGeneralFloorGeometry()||
        !SameNativeAnalyticContactValues(ground->GetGroundValuesForInspection(),row.Selected.Hit))return;
    const auto& request=ground->GetRequestForInspection();
    const auto geometry=ground->GetGeneralFloorMotionDescriptorForInspection();
    if(!request||request->GetSnapshot()!=Snapshot||!geometry||geometry->GetRequest()!=request||
        geometry->GetValues().Status!=ENativeSolidGeometryStatus::GeneralBoxFloorCachedArrivalDescriptor)return;
    const auto& proof=geometry->GetValues().GeneralBoxCachedFloorArrivalDescriptor.CachedPrefix;
    const auto& source=request->GetValues();const auto& step=request->GetStep();
    const auto* resolver=Cast<UBoxSubBody>(row.RawResolver);
    if(!proof||!proof->IsFixedCardinal()||!resolver||source.Body!=resolver||source.Owner!=inventory->Owner||
        source.BodyIndex!=0||body.Index!=0||source.OwnerStableId!=3||source.FrameClock.OwnerLocalFrame!=4||
        step.Frame!=3||step.StepSerial!=4||step.Iteration!=2||step.QuerySerial!=4||step.QueryEpoch!=4)return;
    const auto* before=GeneralBoxTransportDetail::Find(row.Entry,source.Owner);
    const auto* after=GeneralBoxTransportDetail::Find(row.After,source.Owner);
    if(!before||!after||!after->Bodies.IsValidIndex(source.BodyIndex)||after->Bodies[source.BodyIndex].Body!=resolver||
        !GeneralBoxTransportDetail::SamePoseAndBoxConfiguration(*before,*after))return;
    for(int32 i=0;i<after->Bodies.Num();++i)
    {
        const auto& a=before->Bodies[i];const auto& b=after->Bodies[i];
        if(!PlanarBodyPoseDetail::SameShape(a,b)||!GeneralBoxTransportDetail::Pose(a.State,b.State))return;
    }
    auto baseline=MakeShared<FFixedNoPoseSweepBaseline,ESPMode::ThreadSafe>();
    baseline->Snapshot=Snapshot;baseline->ResponseStep=now;baseline->SweepStep=now;
    ++baseline->SweepStep.QueryEpoch;baseline->SweepStep.Iteration=iteration;
    baseline->SweepStep.Remaining=remaining;baseline->SweepStep.TimePassed=timePassed;
    baseline->Resolver=resolver;baseline->After=*after;baseline->Ground=ground;
    FixedNoPoseSweepBaseline=MoveTemp(baseline);
    UE_LOG(LogTemp,Display,TEXT("[NativeRoofFixedNoPoseSweepBaseline] Frame=%llu Serial=%llu ResponseEpoch=%llu SweepEpoch=%llu Iteration=%d Pair=%llu GenerationBefore=%u GenerationAfter=%u BodyCount=%d"),
        now.Frame,now.QuerySerial,now.QueryEpoch,FixedNoPoseSweepBaseline->SweepStep.QueryEpoch,iteration,
        row.Selected.PairKey,uint32(row.GenerationBefore),uint32(row.GenerationAfter),after->Bodies.Num());
}
#endif

#include "PreparedNativeFrameExecution.inl"
#include "PreparedSleepingSphereSupport.inl"

namespace
{
bool ReadSphereValues(const UWorld* world,const USolidSubBody* solid,FVector start,FVector end,const FCollisionShape& shape,float horizon,
    const FStepIdentity& step,const FPreparedPhysicalQueryDomain& filters,FSphereValues& out)
{
    out={};const USphereSubBody* body=nullptr;const void* boundOwner=nullptr;
    if(!FSphereMemberSweepScope::ReadBody(solid,body,boundOwner))return false;
    const auto* owner=body->GetParentComponent();
    if(!body||Cast<USWheelSubBody>(solid)||!owner||owner!=boundOwner||world!=step.World||body->GetWorld()!=world||!shape.IsSphere()||
        !SameFloat(horizon,step.Remaining))return false;
    FCanonicalPreparedFrameClock frameClock;
    if(!JoinCanonicalPreparedFrameClock(step.Frame,owner->NumFrame(),frameClock))return false;
    const int32 index=owner->GetSubBodies().IndexOfByKey(const_cast<USolidSubBody*>(solid));
    const uint64 stable=FStepScope::ReadOwnerStableId(owner);
    if(index==INDEX_NONE||!stable||stable!=owner->GetPublishedSimulationStableId())return false;
    FSphereValues v;v.Owner=owner;v.Body=body;v.BodyClass=body->GetClass();v.OwnerStableId=stable;v.BodyIndex=index;v.FrameClock=frameClock;
    v.OwnerState=owner->GetKinematicState();v.BodyState=body->GetKinematicState();
    v.LocalOffset=body->GetLocalOffset();v.LocalRotation=body->GetLocalRotation();v.LocalCOM=owner->GetPhysCenterOfMassLocal();
    if(!ReadSphereMemberSweep(body,owner,v.ResponseScopeSerial,v.ResponseMargin))return false;
    v.ResponseMarginCaptured=true;
    v.Frozen=owner->IsFrozen();v.Prescribed=owner->IsPrescribedMotion();v.Start=start;v.End=end;v.Horizon=horizon;
    const auto* sweeper=static_cast<const ISolidSweeper*>(body);
    const auto actualShape=sweeper->GetWorldStaticCollisionShape();v.CollisionRadius=shape.GetSphereRadius();v.PhysicalRadius=body->GetRadius();v.RadiusInflation=double(v.CollisionRadius)-double(v.PhysicalRadius);
    v.StaticContactOffsetCm=body->GetWorldStaticContactOffsetCm();v.StaticSupportRadius=body->GetWorldStaticSupportRadius();
    v.StaticImpactFriction=body->GetWorldStaticImpactFriction();
    if(v.Frozen||v.Prescribed||!actualShape.IsSphere()||!SameFloat(actualShape.GetSphereRadius(),v.CollisionRadius)||
        !FMath::IsFinite(v.PhysicalRadius)||v.PhysicalRadius<=0||!FMath::IsFinite(v.CollisionRadius)||v.CollisionRadius<v.PhysicalRadius||
        !FMath::IsFinite(v.RadiusInflation)||v.RadiusInflation<0||
        !FMath::IsFinite(v.StaticContactOffsetCm)||v.StaticContactOffsetCm<0||!FMath::IsFinite(v.StaticSupportRadius)||v.StaticSupportRadius<=0||
        !SameDouble(v.StaticSupportRadius,double(v.PhysicalRadius)+double(v.StaticContactOffsetCm))||
        !FMath::IsFinite(v.StaticImpactFriction)||v.StaticImpactFriction<0||!FiniteState(v.OwnerState)||!FiniteState(v.BodyState)||
        !SameVector(v.OwnerState.Location,v.BodyState.Location)||!SameVector(v.OwnerState.Velocity,v.BodyState.Velocity)||
        !SameVector(v.OwnerState.Acceleration,v.BodyState.Acceleration)||!SameVector(start,v.BodyState.Location))return false;
    const FVector expectedEnd=start+v.BodyState.Velocity*horizon+0.5f*v.BodyState.Acceleration*horizon*horizon;
    if(!SameVector(end,expectedEnd))return false; // retained call DAG, not the integration/reference DAG
    v.Filter.bApplyCollisionFilter=true;v.Filter.bAuthorityOnly=true;v.Filter.TraceChannel=uint8(sweeper->GetCollisionChannel());v.Filter.BlockingObjectTypes=0;
    for(int32 c=0;c<UE_ARRAY_COUNT(FCollisionResponseContainer::EnumArray);++c)
        if(sweeper->GetResponseParams().CollisionResponse.GetResponse(ECollisionChannel(c))==ECR_Block)v.Filter.BlockingObjectTypes|=uint64(1)<<c;
    if(!filters.CoversPhysicalFilter(true,step.InputGeneration,owner,body,v.Filter))return false;
    out=MoveTemp(v);return true;
}
bool SameValues(const FSphereValues& a,const FSphereValues& b) noexcept
{
    return a.Owner==b.Owner&&a.Body==b.Body&&a.BodyClass==b.BodyClass&&a.OwnerStableId==b.OwnerStableId&&a.BodyIndex==b.BodyIndex&&a.FrameClock==b.FrameClock&&
        SameKinematicBits(a.OwnerState,b.OwnerState)&&SameKinematicBits(a.BodyState,b.BodyState)&&SameVector(a.LocalOffset,b.LocalOffset)&&
        SameQuat(a.LocalRotation,b.LocalRotation)&&SameVector(a.LocalCOM,b.LocalCOM)&&SameVector(a.Start,b.Start)&&SameVector(a.End,b.End)&&
        SameFloat(a.Horizon,b.Horizon)&&SameFloat(a.CollisionRadius,b.CollisionRadius)&&SameFloat(a.PhysicalRadius,b.PhysicalRadius)&&
        SameDouble(a.RadiusInflation,b.RadiusInflation)&&SameDouble(a.StaticSupportRadius,b.StaticSupportRadius)&&
        SameFloat(a.StaticContactOffsetCm,b.StaticContactOffsetCm)&&SameFloat(a.StaticImpactFriction,b.StaticImpactFriction)&&a.ResponseMarginCaptured==b.ResponseMarginCaptured&&SameFloat(a.ResponseMargin,b.ResponseMargin)&&a.ResponseScopeSerial==b.ResponseScopeSerial&&a.Frozen==b.Frozen&&a.Prescribed==b.Prescribed&&
        a.Filter.TraceChannel==b.Filter.TraceChannel&&a.Filter.BlockingObjectTypes==b.Filter.BlockingObjectTypes;
}
}
bool CaptureSphereCCD(const UWorld* world,const USolidSubBody* solid,FVector start,FVector end,const FCollisionShape& shape,float horizon,FNativeSphereRequestPtr& out)
{
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
    out.Reset();FStepIdentity step;if(!NextSphereQuery(step))return false;
    TSharedPtr<const Analytic::Prepared::FScene,ESPMode::ThreadSafe> scene;TSharedPtr<const FPreparedPhysicalQueryDomain,ESPMode::ThreadSafe> filters;
    TSharedPtr<const FPreparedWorldSnapshot,ESPMode::ThreadSafe> snapshot;
    if(!ReadCurrentSnapshot(snapshot)||!ReadCurrentInputs(scene,filters)||snapshot->GetNominalScene()!=scene||
        snapshot->GetFilters()!=filters||!snapshot->GetGeometrySelection()||!snapshot->GetGeometrySelection()->HasValidAliases())return false;FSphereValues values;
    if(!ReadSphereValues(world,solid,start,end,shape,horizon,step,*filters,values))return false;
    TSharedPtr<FNativeSphereRequest,ESPMode::ThreadSafe> value(new FNativeSphereRequest);value->Step=step;value->Values=values;value->Scene=scene;value->Filters=filters;value->Snapshot=snapshot;value->Geometry=snapshot->GetGeometrySelection();
    if(!Analytic::Certified::BuildZeroOffsetKinematicCenterReference(values.BodyState.Location,values.BodyState.Velocity,values.BodyState.Acceleration,
        horizon,values.LocalOffset,values.LocalCOM,value->Reference)||!RevalidateSphereCCD(*value))return false;
    out=value;return true;
}
bool RevalidateSphereCCD(const FNativeSphereRequest& request) noexcept
{
    // Snapshot identity/epochs are checked before any retained Body access.
    TSharedPtr<const FPreparedWorldSnapshot,ESPMode::ThreadSafe> currentSnapshot;
    if(!ReadCurrentSnapshot(currentSnapshot)||!request.Snapshot||currentSnapshot!=request.Snapshot||
        currentSnapshot->GetNominalScene()!=request.Scene||currentSnapshot->GetFilters()!=request.Filters||
        !request.Geometry||currentSnapshot->GetGeometrySelection()!=request.Geometry||!request.Geometry->HasValidAliases()||
        request.Geometry->GetNominalScene()!=request.Scene||request.Geometry->GetReference()!=currentSnapshot->GetReference())return false;

    uint64 memberSerial=0;float memberMargin=0;
    if(!ReadSphereMemberSweep(request.Values.Body,request.Values.Owner,memberSerial,memberMargin)||
        !request.Values.ResponseMarginCaptured||memberSerial!=request.Values.ResponseScopeSerial||!SameFloat(memberMargin,request.Values.ResponseMargin))return false;
    FStepIdentity now;
    if(!ReadCurrentStep(now)||now.Lease!=request.Step.Lease||now.Host!=request.Step.Host||now.Bridge!=request.Step.Bridge||now.World!=request.Step.World||
        now.InputGeneration!=request.Step.InputGeneration||now.Frame!=request.Step.Frame||now.WindowSerial!=request.Step.WindowSerial||now.WindowNonce!=request.Step.WindowNonce||now.StepSerial!=request.Step.StepSerial||now.Iteration!=request.Step.Iteration||now.QueryEpoch!=request.Step.QueryEpoch||
        now.NumericControl!=request.Step.NumericControl||now.Provider!=request.Step.Provider||now.ThreadId!=request.Step.ThreadId||
        !request.Step.QuerySerial||request.Step.QuerySerial>now.QuerySerial||!SameFloat(now.Remaining,request.Step.Remaining)||!SameFloat(now.TimePassed,request.Step.TimePassed))return false;
    const auto shape=FCollisionShape::MakeSphere(request.Values.CollisionRadius);FSphereValues current;
    return request.Filters&&ReadSphereValues(now.World,request.Values.Body,request.Values.Start,request.Values.End,shape,request.Values.Horizon,now,*request.Filters,current)&&SameValues(current,request.Values);
}

C::FReferenceCandidateSetPtr SelectWholeConsumedSphereHull(C::FIndexedIntervalUnionPtr scene,
    const C::FZeroOffsetKinematicCenterReference& model,float horizon,double radius,C::FDistanceBudget& budget)
{
    if(!scene||!model.IsKnown()||!budget.Consume())return {};
    const auto hull=C::BoundConsumedCenterBetweenFloatSeconds(model,0.f,horizon);
    if(!C::Detail::ValidPoint(hull))return {};
    C::FIntervalBezierCenterPath path;path.ControlPoints.Add(hull);
    C::FReferenceCandidateSetPtr result;
    if(!C::FReferenceCandidateSet::Select(scene,path,radius,budget,result))return {};
    return result;
}
bool FSphereMemberSweepScope::RecordGroundSweep(FNativeSphereRequestPtr request,SHitResult& outHit)
{
    if(StepScope&&StepScope->IsTerminalRsPurpose())Refuse("Terminal Rs capture cannot deliver an ordinary ground hit or miss");
    outHit={};auto* scope=MemberSweepScope;
    if(!scope||scope->PendingGround||!request||!RevalidateSphereCCD(*request))Refuse("Prepared ground producer lacks authentic member request");
    const auto geometry=request->GetGeometrySelection();const auto& v=request->GetValues();
    if(!geometry||!geometry->HasValidAliases())Refuse("Prepared ground selected geometry unavailable");
    // Four bounded full-provider/CP passes, shared with contact refinement and
    // payload work. This is numerical work, not a collision/response tolerance.
    const int64 work=int64(geometry->GetSelectedUnion()->Num())*(1+17*17)*4+1;
    if(work<=0||work>MAX_int32)Refuse("Prepared ground query work bound unavailable");
    C::FDistanceBudget budget{int32(work)};
    const auto selected=SelectSelectedWholeConsumedSphereHull(geometry,request->GetReference(),v.Horizon,double(v.CollisionRadius),budget);
    if(!selected.UsesGeometry(geometry)||!RevalidateSphereCCD(*request))Refuse("Prepared complete ground partition unavailable");
    TSharedPtr<FGroundSweepEvidence,ESPMode::ThreadSafe> result(new FGroundSweepEvidence);
    result->Geometry=geometry;result->Request=request;result->Partition=selected.Partition;
    if(selected.Partition->GetCandidateIndices().IsEmpty())result->Kind=FGroundSweepEvidence::EKind::WholeHullExclusion;
    else
    {
        FSphereResponseQueryValues q;q.Position=v.BodyState.Location;q.Velocity=v.BodyState.Velocity;q.Acceleration=v.BodyState.Acceleration;
        q.Horizon=v.Horizon;q.CollisionRadius=v.CollisionRadius;q.PhysicalRadius=v.PhysicalRadius;q.ResponseMargin=v.ResponseMargin;q.StaticContactOffsetCm=v.StaticContactOffsetCm;q.Filter=v.Filter;
        C::FReferenceSphereOptions options;if(!BuildMarginDrivenSphereReferenceOptions(q,budget.Remaining,options))Refuse("Native captured margin cannot select supported numerical goals");
        FString reason;const auto evaluated=EvaluateNativeSphereResponseQuality(request,options,budget,reason);
        if(evaluated.Request!=request||evaluated.Geometry!=geometry||!evaluated.Quality||
            evaluated.Quality->GetStatus()!=ESphereResponseQualityStatus::BoundedWithinInputMargin||request->GetStep().Frame>MAX_uint32)
            Refuse("Prepared first contact quality/captured policy remains unknown");
        FSelectedSphereHitValues payload;
        if(!BuildSelectedSphereHitValues(geometry,evaluated.Quality,uint32(request->GetStep().Frame),budget,payload,reason)||!RevalidateSphereCCD(*request))
            Refuse("Prepared first contact payload or native request changed");
        result->Kind=FGroundSweepEvidence::EKind::FirstContact;result->Quality=evaluated.Quality;result->PendingHit=payload.Hit;
        outHit=payload.Hit;outHit.PreparedGroundEvidence=result;
    }
    scope->PendingGround=MoveTemp(result);return true; // Pending delivery only, never advance/response permission.
}

FComponentSweepScope::FComponentSweepScope(const ISpeedComponent& owner)
{
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
    if(!Requested())return;
    FStepIdentity step;
    if(Poisoned||ComponentSweepScope||MemberSweepScope||ComponentSweepSerial==MAX_uint64||
        !ReadCurrentStep(step)||!FStepScope::ReadOwnerStableId(&owner))
        Refuse("Prepared component sweep has no authentic inventory scope");
    Pending=TSharedPtr<FComponentSweepEvidence,ESPMode::ThreadSafe>(new FComponentSweepEvidence);
    Pending->Step=step;Pending->Owner=&owner;Pending->OwnerStableId=FStepScope::ReadOwnerStableId(&owner);
    Pending->ScopeSerial=++ComponentSweepSerial;Pending->SweepOwnerState=owner.GetKinematicState();
    const auto& bodies=owner.GetSubBodies();
    for(int32 i=0;i<bodies.Num();++i){auto& slot=Pending->Bodies.AddDefaulted_GetRef();slot.Body=bodies[i];slot.RawBodyIdentity=bodies[i];slot.Index=i;}
    Active=true;ComponentSweepScope=this;
}
void FComponentSweepScope::BeginBody(int32 index,USSubBody* body)
{
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
    if(!Active)return;
    FStepIdentity now;
    if(ComponentSweepScope!=this||Completed||MemberSweepScope||CurrentIndex!=INDEX_NONE||
        index!=NextIndex||!Pending->Bodies.IsValidIndex(index)||Pending->Bodies[index].Body.Get()!=body||
        !ReadCurrentStep(now)||now.QueryEpoch!=Pending->Step.QueryEpoch||now.WindowSerial!=Pending->Step.WindowSerial||now.WindowNonce!=Pending->Step.WindowNonce||now.StepSerial!=Pending->Step.StepSerial||
        !SameKinematicBits(Pending->SweepOwnerState,Pending->Owner->GetKinematicState()))
        Refuse("Prepared subbody sweep order, state or identity changed");
    CurrentIndex=index;
    if(StepScope&&StepScope->IsTerminalRsPurpose())TerminalRsRawBody=body;
}
void FComponentSweepScope::ObserveBody(USSubBody* body,bool hit,float toi)
{
    if(!Active)return;
    if(ComponentSweepScope!=this||Completed||MemberSweepScope||CurrentIndex!=NextIndex||
        !Pending->Bodies.IsValidIndex(CurrentIndex)||Pending->Bodies[CurrentIndex].Body.Get()!=body||
        !FMath::IsFinite(toi)||toi<0||toi>Pending->Step.Remaining||
        !SameKinematicBits(Pending->SweepOwnerState,Pending->Owner->GetKinematicState()))
        Refuse("Prepared subbody result is missing, reordered or mutated");
    auto& slot=Pending->Bodies[CurrentIndex];if(slot.Returned)Refuse("Prepared subbody result duplicated");
    if(slot.Ground)
    {
        const auto seed=slot.Ground->StateSeed;FStepIdentity now;
        if(!seed||!ReadCurrentStep(now)||seed->Step.Lease!=now.Lease||seed->Step.QueryEpoch!=now.QueryEpoch||
            seed->Step.WindowSerial!=now.WindowSerial||seed->Step.WindowNonce!=now.WindowNonce||seed->Step.StepSerial!=now.StepSerial||seed->Step.QuerySerial!=now.QuerySerial||
            seed->Snapshot!=StepScope->Snapshot||seed->Geometry!=seed->Snapshot->GetGeometrySelection()||
            seed->Values.Body!=body||seed->Values.Owner!=Pending->Owner||seed->Values.BodyIndex!=CurrentIndex||
            !SameKinematicBits(seed->Values.OwnerState,Pending->Owner->GetKinematicState())||
            !SameKinematicBits(seed->Values.BodyState,body->GetKinematicState()))
            Refuse("Prepared final sphere callback state or snapshot changed");
        const auto& finalHit=body->GetFutureHit();
        if(hit!=seed->GroundHit||!SameFloat(toi,seed->GroundSeconds)||!SamePreparedHitValues(finalHit,seed->NamedGround)||
            (hit?finalHit.PreparedGroundEvidence.Get()!=slot.Ground.Get():bool(finalHit.PreparedGroundEvidence)))
            Refuse("Prepared final sphere callback/result differs from sealed ground state");
        slot.FinalSphereState=seed;
    }
    if(slot.OrdinaryGround)
    {
        // The member is closed now. Do NOT revalidate its request via the old
        // live-member reader, and do NOT identify the final dynamic/cache
        // winner with the separately recorded World candidate.
        const auto& ground=slot.OrdinaryGround;FStepIdentity now;
        if(!ground->IsClosedQueryValues()||!ReadCurrentStep(now)||!StepScope||
            ground->Request->GetSnapshot()!=StepScope->Snapshot||
            ground->Request->GetValues().Body!=body||ground->Request->GetValues().Owner!=Pending->Owner||
            ground->Request->GetValues().BodyIndex!=CurrentIndex||
            ground->Request->GetStep().Lease!=now.Lease||ground->Request->GetStep().QueryEpoch!=now.QueryEpoch||
            ground->Request->GetStep().StepSerial!=now.StepSerial||ground->Request->GetStep().WindowSerial!=now.WindowSerial||
            ground->Request->GetStep().WindowNonce!=now.WindowNonce||ground->Request->GetStep().QuerySerial>now.QuerySerial)
            Refuse("Ordinary World candidate closure does not join its final body callback");
    }
    if(slot.SleepingSupportBypass)
    {
        const auto& observed=slot.SleepingSupportBypass;FStepIdentity now;
        if(!observed->IsClosedObservation()||!ReadCurrentStep(now)||observed->Snapshot!=StepScope->Snapshot||
            observed->Intrinsic.Body!=body||observed->Intrinsic.Owner!=Pending->Owner||observed->Intrinsic.BodyIndex!=CurrentIndex||
            observed->Step.Lease!=now.Lease||observed->Step.QueryEpoch!=now.QueryEpoch||observed->Step.StepSerial!=now.StepSerial||
            observed->Step.WindowSerial!=now.WindowSerial||observed->Step.WindowNonce!=now.WindowNonce||observed->Step.QuerySerial>now.QuerySerial)
            Refuse("Sleeping support bypass observation lost its actual owner/body callback");
        // No World query was executed. Final dynamic/cache results below stay
        // independent; this branch cannot become a World-clear certificate.
    }
    if(slot.BoxGroundReuse)
    {
        CloseNativeBoxOuterReturn(slot.BoxGroundReuse->Body,hit,toi);
        if(!slot.BoxGroundReuse->IsClosedObservation())Refuse("Box outer callback observation remained open");
    }
    if(body){slot.ReturnedFutureHit=body->GetFutureHit();slot.FutureHitObserved=true;}
    slot.Returned=true;slot.ReportedHit=hit;slot.ReportedTOI=toi;
    CurrentIndex=INDEX_NONE;++NextIndex;
}
void FComponentSweepScope::SealGround(FGroundSweepEvidencePtr ground)
{
    if(!Active||Completed||ComponentSweepScope!=this||!ground||(!ground->IsSealedWholeHullExclusion()&&!ground->IsSealedFirstContact())||
        !Pending->Bodies.IsValidIndex(CurrentIndex)||Pending->Bodies[CurrentIndex].Ground)
        Refuse("Prepared ground result has no unique current inventory slot");
    const auto& values=ground->GetCaptureForInspection()->GetValues();
    if(Pending->Bodies[CurrentIndex].Body.Get()!=values.Body||Pending->Owner!=values.Owner||
        Pending->OwnerStableId!=values.OwnerStableId||CurrentIndex!=values.BodyIndex||
        !SameKinematicBits(Pending->SweepOwnerState,values.OwnerState))
        Refuse("Prepared ground result does not belong to current component slot");
    Pending->Bodies[CurrentIndex].Ground=MoveTemp(ground);
}
void FComponentSweepScope::ObserveWinner(int32 index,const SHitResult& raw,const SHitResult& oriented,
    const TWeakObjectPtr<USSubBody>& resolver,const USSubBody* rawResolver,uint64 pairKey)
{
    if(!Active)return;
    FStepIdentity now;
    if(ComponentSweepScope!=this||Completed||CurrentIndex!=INDEX_NONE||MemberSweepScope||
        !Pending->Bodies.IsValidIndex(index)||!ReadCurrentStep(now)||now.Lease!=Pending->Step.Lease||
        now.StepSerial!=Pending->Step.StepSerial||now.QueryEpoch!=Pending->Step.QueryEpoch||
        now.WindowSerial!=Pending->Step.WindowSerial||now.WindowNonce!=Pending->Step.WindowNonce||
        !SameKinematicBits(Pending->SweepOwnerState,Pending->Owner->GetKinematicState()))
        Refuse("Prepared component winner is outside its actual owner/body sweep");
    const auto& body=Pending->Bodies[index];
    if(!body.Returned||!body.ReportedHit||!body.FutureHitObserved||
        !SameNativeAnalyticContactValues(body.ReturnedFutureHit,raw)||
        (Pending->SelectedHit&&body.ReportedTOI>=Pending->SelectedSeconds))
        Refuse("Prepared component winner differs from actual final body values or minimum order");
    // Join the ACTUAL PickResolver raw result to the complete retained GT
    // census. A dynamic resolver can belong to another owner. No Weak.Get,
    // GetUniqueID or operation qualification is inferred here on the worker.
    const auto bindings=StepScope->Snapshot?StepScope->Snapshot->GetNativeOwners():FPreparedNativeOwnerBindingsPtr{};
    const FPreparedNativeBodyBinding* identity=nullptr;const ISpeedComponent* resolverOwner=nullptr;
    if(!rawResolver||!bindings||bindings->GetGeneration()!=now.InputGeneration)
        Refuse("Prepared selected resolver has no current GT identity census");
    for(const auto& owner:bindings->GetRecordsForInspection())for(const auto& child:owner.Bodies)
        if(child.Body==rawResolver)
        {
            if(identity||!owner.Adapter||!owner.BodyCensusCaptured||!child.ObjectIdCaptured||!child.Class||
                !static_cast<FWeakObjectPtr>(resolver).HasSameIndexAndSerialNumber(child.Object))
                Refuse("Prepared selected resolver raw/weak GT identity is absent or ambiguous");
            identity=&child;resolverOwner=owner.Adapter;
        }
    if(!identity)Refuse("Prepared actual resolver is outside retained GT body census");
    Pending->SelectedRawResolver=rawResolver;Pending->SelectedResolverGTObjectId=identity->ObjectId;
    Pending->SelectedResolverOwner=resolverOwner;Pending->SelectedResolverBodyIndex=identity->Index;
    Pending->SelectionObserved=true;Pending->SelectedHit=true;Pending->SelectedBodyIndex=index;
    Pending->SelectedSeconds=body.ReportedTOI;Pending->SelectedPairKey=pairKey;
    Pending->SelectedResolver=resolver;Pending->SelectedRawHit=raw;Pending->SelectedOrientedHit=oriented;
}
FComponentSweepEvidencePtr FComponentSweepScope::Finish(const ::SComponentTOI& actual)
{
    if(!Active)return {};
    FStepIdentity now;
    if(ComponentSweepScope!=this||Completed||MemberSweepScope||CurrentIndex!=INDEX_NONE||
        NextIndex!=Pending->Bodies.Num()||!ReadCurrentStep(now)||now.Lease!=Pending->Step.Lease||
        now.QueryEpoch!=Pending->Step.QueryEpoch||now.WindowSerial!=Pending->Step.WindowSerial||now.WindowNonce!=Pending->Step.WindowNonce||now.StepSerial!=Pending->Step.StepSerial||
        !SameKinematicBits(Pending->SweepOwnerState,Pending->Owner->GetKinematicState())||
        Pending->Owner->GetSubBodies().Num()!=Pending->Bodies.Num())
        Refuse("Prepared complete component sweep inventory unavailable");
    for(int32 i=0;i<Pending->Bodies.Num();++i)
        if(!Pending->Bodies[i].Returned||Pending->Bodies[i].Body.Get()!=Pending->Owner->GetSubBodies()[i])
            Refuse("Prepared complete component sweep inventory changed");
    if(actual.PreparedEvidence||actual.bHit!=Pending->SelectedHit||!FMath::IsFinite(actual.TOI)||actual.TOI<0||actual.TOI>now.Remaining)
        Refuse("Prepared component return does not match its actual recorded minimum");
    if(actual.bHit)
    {
        if(!Pending->SelectionObserved||!SameFloat(actual.TOI,Pending->SelectedSeconds)||
            actual.PairKey!=Pending->SelectedPairKey||!actual.Resolver.HasSameIndexAndSerialNumber(Pending->SelectedResolver)||
            !SameNativeAnalyticContactValues(actual.Hit,Pending->SelectedOrientedHit))
            Refuse("Prepared returned component winner or oriented values changed");
    }
    else
    {
        if(!SameFloat(actual.TOI,now.Remaining))Refuse("Prepared no-hit component return has a different horizon");
        Pending->SelectionObserved=true;Pending->SelectedSeconds=actual.TOI;
    }
    Pending->CompletionStep=now;Pending->Sealed=true;Completed=true;return Pending;
}
FComponentSweepScope::~FComponentSweepScope() noexcept
{
    if(!Active)return;
    if(!Completed&&!Interrupted&&std::uncaught_exceptions()>0)CloseInterruptedTerminalRsCapture();
    if(ComponentSweepScope!=this||(!Completed&&!Interrupted)||MemberSweepScope)RecordPreparedPoison("PreparedPhysicalExecution.cpp:998");
    ComponentSweepScope=nullptr;Active=false;
}
void FStepScope::RequireAdvanceEvidence(TConstArrayView<FComponentSweepEvidencePtr> bundles,float seconds,
    bool hasSelection,TConstArrayView<SComponentTOI> selectedBatch,float sweepHorizon)
{
    if(!Active)return;
    if(Window&&Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame)
        Refuse("Terminal native Step sync grants no integration or response");
    FStepIdentity now;
    if(ComponentSweepScope||MemberSweepScope||!ReadCurrentStep(now)||!SweepPhase||
        !FMath::IsFinite(seconds)||seconds<0||seconds>Identity.Remaining||
        bundles.Num()!=Registry->GetOrderedAdapters().Num())
        Refuse("Prepared advance has no complete current owner sweep inventory");
    for(int32 i=0;i<bundles.Num();++i)
    {
        const auto& bundle=bundles[i];const auto* owner=Registry->GetOrderedAdapters()[i];
        if(!owner||!bundle||!bundle->Sealed||bundle->Owner!=owner||!bundle->ScopeSerial||
            bundle->Step.Lease!=now.Lease||bundle->Step.QueryEpoch!=now.QueryEpoch||
            bundle->Step.WindowSerial!=now.WindowSerial||bundle->Step.WindowNonce!=now.WindowNonce||bundle->Step.StepSerial!=now.StepSerial||bundle->Step.InputGeneration!=now.InputGeneration||
            bundle->Step.Provider!=now.Provider||bundle->Step.ThreadId!=now.ThreadId||
            bundle->OwnerStableId!=Registry->FindStableId(*owner))
            Refuse("Prepared advance inventory is stale, incomplete or belongs to another owner");
    }
    CheckOrdinaryOwnerSweepReturns(bundles);
    CheckOrdinaryWorldSelection(bundles,seconds,hasSelection,selectedBatch,sweepHorizon);
    if(NativeConsumer)
    {
        RequireNativeFrameContext();
        if(!hasSelection||!SameFloat(sweepHorizon,now.Remaining)||
            (NativeAdvance&&!NativeAdvance->Consumed))Refuse("Native advance requires a fresh complete actual selector");
        for(const auto& bundle:bundles)for(const auto& child:bundle->Bodies)
            if(!child.Returned)Refuse("Native advance omitted a real body sweep return");
        NativeAdvance=MakeUnique<FOrdinaryNativeAdvanceTicket>();NativeAdvance->Queried=now;
        NativeAdvance->Seconds=seconds;NativeAdvance->Horizon=sweepHorizon;NativeAdvance->Before=now.TimePassed;
        NativeAdvance->HasSelection=hasSelection;NativeAdvance->Selected.Append(selectedBatch.GetData(),selectedBatch.Num());
        for(const auto& bundle:bundles)for(const auto& child:bundle->Bodies)
        {
            const auto ground=child.OrdinaryGround;
            if(!ground)continue;
            const bool released=ground->IsReleasedBoxFloorMotionValues();
            const bool floorMotion=ground->IsFirstGeneralBoxFloorContactValues()||ground->IsWholeGeneralBoxFloorClearValues();
            const bool curved=ground->IsCurvedBoxArrivalValues();
            if(curved&&seconds>ground->GetGroundValuesForInspection().TOI)Refuse("Curved Box advance exceeds its original proved prefix");
            const auto prediction=(curved||ground->IsCanonicalReferenceBoxSweepPathClearValues()||
                ground->IsCurvedBoxActualMotionClearValues())?ground->GetCanonicalBoxClearanceForInspection():FNativeCanonicalBoxSweepClearanceValuesPtr{};
            if(!released&&!floorMotion&&(!prediction||!prediction->RequiresGeneralBoxTransport()))continue;
            if(int32(released)+int32(floorMotion)+int32(prediction.IsValid())!=1)Refuse("General Box advance has ambiguous geometry kinds");
            const auto request=ground->GetRequestForInspection();
            if(!request||request->GetSnapshot()!=Snapshot||request->GetValues().Owner!=bundle->Owner||
                request->GetValues().BodyIndex!=child.Index||request->GetValues().Body!=child.RawBodyIdentity)
                Refuse("General Box advance prediction has a different actual body binding");
            auto& binding=NativeAdvance->GeneralBindings.AddDefaulted_GetRef();binding.Request=request;binding.Prediction=prediction;
            if(released)binding.Released=ground;
            if(floorMotion){binding.FloorMotion=ground;binding.FloorMotionWorldSeconds=seconds;}
            if(!binding.HasExactlyOneKind())Refuse("General Box advance binding is not exactly one distinct kind");
            GeneralBoxOwners.AddUnique(bundle->Owner);
        }
        return;
    }
    // A canonical-reference prediction is NOT a legacy World-kernel miss or
    // a proof of the effective constrained integration/one-time effects.
    for(const auto& b:bundles)for(const auto& child:b->Bodies)
        if(child.OrdinaryGround&&(child.OrdinaryGround->IsCanonicalReferenceBoxSweepPathClearValues()||
            child.OrdinaryGround->IsCurvedBoxActualMotionClearValues()||child.OrdinaryGround->IsCanonicalAxialBoxPlaneContactValues()||child.OrdinaryGround->IsCurvedBoxArrivalValues()||
            child.OrdinaryGround->IsReleasedBoxFloorMotionValues()||
            child.OrdinaryGround->IsFirstGeneralBoxFloorContactValues()||child.OrdinaryGround->IsWholeGeneralBoxFloorClearValues()))
            Refuse("Canonical Box prediction/contact values need their distinct qualified all-owner advance consumer");
    // Preserve the ACTUAL full batch; the old single-Sphere admission below is
    // deliberately still closed until generic geometry/selection/advance effects
    // exist. Multiple events are no longer replaced by a null pointer upstream.
    const int32 selectedCount=selectedBatch.Num();
    const SComponentTOI* selected=selectedCount==1?selectedBatch.GetData():nullptr;
    if(!hasSelection||!SameFloat(sweepHorizon,now.Remaining)||bundles.Num()!=1||selectedCount>1)
        Refuse("Prepared generic all-owner temporal selection consumer is not installed");
    const auto& bundle=bundles[0];auto* owner=Registry->GetOrderedAdapters()[0];
    if(bundle->Bodies.Num()!=1||owner->GetSubBodies().Num()!=1)Refuse("Prepared initial advance requires exactly one native sphere");
    const auto& row=bundle->Bodies[0];const auto seed=row.FinalSphereState;auto* body=owner->GetSubBodies()[0];
    FCanonicalPreparedFrameClock actualClock;
    if(!JoinCanonicalPreparedFrameClock(now.Frame,owner->NumFrame(),actualClock))
        Refuse("Prepared advance owner-local/canonical frame unavailable");
    if(!row.Returned||row.Index!=0||row.Body.Get()!=body||!seed||!row.Ground||row.Ground->StateSeed!=seed||
        seed->Snapshot!=Snapshot||seed->Geometry!=Snapshot->GetGeometrySelection()||seed->Step.QuerySerial!=now.QuerySerial||
        seed->Step.QueryEpoch!=now.QueryEpoch||seed->Step.Lease!=now.Lease||seed->Values.Owner!=owner||seed->Values.Body!=body||
        !SameKinematicBits(seed->Values.OwnerState,owner->GetKinematicState())||
        !(seed->Values.FrameClock==actualClock)||
        !SameKinematicBits(seed->OwnerHistoryAtFrame,owner->GetKinematicStateForFrame(actualClock.OwnerLocalFrame))||
        !SameKinematicBits(seed->Values.BodyState,body->GetKinematicState()))Refuse("Prepared final sphere state seed is stale or incomplete");
    if(selectedCount==0)
    {
        if(row.ReportedHit||seed->GroundHit||!seed->WholeHullExclusion||!row.Ground->IsSealedWholeHullExclusion()||
            !SameFloat(seconds,sweepHorizon)||!SameFloat(row.ReportedTOI,sweepHorizon))Refuse("Prepared actual no-hit time selection changed");
    }
    else if(!selected->bHit||selected->PreparedEvidence!=bundle||selected->Resolver.Get()!=body||
        !row.ReportedHit||!seed->GroundHit||!row.Ground->IsSealedFirstContact()||
        selected->Hit.PreparedGroundEvidence!=row.Ground||!SamePreparedHitValues(selected->Hit,seed->NamedGround)||
        !SameFloat(selected->TOI,seed->GroundSeconds)||!SameFloat(seconds,seed->GroundSeconds)||
        !SameFloat(row.ReportedTOI,seed->GroundSeconds))Refuse("Prepared selected first-contact bits or consumed seconds changed");
    if(!seed->NativeLocalInputSupported||owner->IsFrozen()||owner->IsPrescribedMotion()||owner->IsPhysicsSleeping()||
        !owner->GetPhysicalConstraints().IsEmpty()||body->RequiresEstablishedStaticContactTransport()||
        !SameKinematicBits(seed->OwnerHistoryAtFrame,seed->Values.OwnerState))Refuse("Prepared free-state routing or native class remains unsupported");
    FSphereFreeAdvanceValues values;values.OwnerBefore=seed->Values.OwnerState;values.BodyBefore=seed->Values.BodyState;
    values.LocalCOM=seed->Values.LocalCOM;values.Local=seed->NativeLocalInput;values.Seconds=seconds;
    values.MaximumSpeed=owner->GetPhysMaxSpeed();values.MaximumAngularSpeed=owner->GetPhysMaxAngularSpeed();
    FSphereFreeAdvanceExpectation expected;FString reason;
    if(!BuildSphereFreeAdvanceExpectation(values,expected,reason))Refuse("Prepared free-state primitive expectation unavailable");
    // Exact values and selection are still not proof of virtual setters,
    // reactions, callback routing, projection or a real integration result.
    // A future private AdvanceSerial/epoch-purpose transition must close those
    // effects; retaining a sweep seed never revives the member request.
    Refuse("Prepared effective integration hooks, post-state and response consumer are not installed");
}
}

namespace Speed::Analytic::Certified
{
FReferenceSphereEvaluation EvaluateReferenceSphereValues(
    FIndexedIntervalUnionPtr scene,FVector3d position,FVector3d velocity,FVector3d acceleration,
    float horizon,double collisionRadius,double physicalRadius,const FReferenceSphereOptions& options,
    FDistanceBudget& budget,TFunctionRef<bool(int32)> acceptsCandidate)
{
    return EvaluateReferenceSphere(MoveTemp(scene),position,velocity,acceleration,horizon,
        collisionRadius,physicalRadius,options,budget,acceptsCandidate);
}
}

namespace Speed::PreparedExecution
{
namespace ResponseQualityDetail
{
uint32 FloatBits(float value){uint32 bits;FMemory::Memcpy(&bits,&value,sizeof(bits));return bits;}
bool SamePoint(const FVector3d& a,const FVector3d& b)
{
    for(int32 d=0;d<3;++d)if(AnalyticCertified::Bits(a[d])!=AnalyticCertified::Bits(b[d]))return false;
    return true;
}
bool SameOptions(const Analytic::Certified::FReferenceSphereOptions& a,const Analytic::Certified::FReferenceSphereOptions& b)
{
    return AnalyticCertified::Bits(a.TimeGap)==AnalyticCertified::Bits(b.TimeGap)&&AnalyticCertified::Bits(a.SpatialGap)==AnalyticCertified::Bits(b.SpatialGap)&&
        AnalyticCertified::Bits(a.PointWidth)==AnalyticCertified::Bits(b.PointWidth)&&AnalyticCertified::Bits(a.NormalWidth)==AnalyticCertified::Bits(b.NormalWidth)&&
        a.SpatialNodesPerAttempt==b.SpatialNodesPerAttempt&&a.PrefixRangeAttempts==b.PrefixRangeAttempts;
}
}
bool FSelectedSphereResponseQuality::MatchesValues(const FPreparedGeometrySelectionPtr& geometry,const FSphereResponseQualityValues& v) const
{
    using namespace ResponseQualityDetail;
    if(!geometry||!geometry->HasValidAliases()||Geometry!=geometry||Reference.Status!=Analytic::Certified::EReferenceSphereStatus::ReferenceContactBounded||
        !Reference.Candidates||Reference.Candidates->GetScene()!=geometry->GetSelectedUnion()||
        !SamePoint(Values.Position,v.Position)||!SamePoint(Values.Velocity,v.Velocity)||!SamePoint(Values.Acceleration,v.Acceleration)||
        FloatBits(Values.Horizon)!=FloatBits(v.Horizon)||FloatBits(Values.CollisionRadius)!=FloatBits(v.CollisionRadius)||
        FloatBits(Values.PhysicalRadius)!=FloatBits(v.PhysicalRadius)||FloatBits(Values.ResponseMargin)!=FloatBits(v.ResponseMargin)||
        FloatBits(Values.StaticContactOffsetCm)!=FloatBits(v.StaticContactOffsetCm)||
        !SameOptions(Values.Options,v.Options)||Values.Branches.Num()!=v.Branches.Num()||Values.AcceptedProviders!=v.AcceptedProviders)return false;
    for(int32 i=0;i<v.Branches.Num();++i)
        if(Values.Branches[i].SurfaceId!=v.Branches[i].SurfaceId||Values.Branches[i].NormalMayVary!=v.Branches[i].NormalMayVary||
            Values.Branches[i].StartPenetrating!=v.Branches[i].StartPenetrating)return false;
    return true;
}
bool FSelectedSphereResponseQuality::MatchesConsumedContact(float seconds,float physicalRadius,FVector3d location,
    FVector3d impactPoint,FVector3d normalStored,FVector3d responsePoint) const
{
    using namespace ResponseQualityDetail;
    return FloatBits(seconds)==FloatBits(Reference.ConsumedSeconds)&&FloatBits(physicalRadius)==FloatBits(Values.PhysicalRadius)&&
        SamePoint(location,ReboundedScalar.Location)&&SamePoint(impactPoint,ReboundedScalar.ImpactPoint)&&
        SamePoint(normalStored,ReboundedScalar.NormalStored)&&SamePoint(responsePoint,ResponsePoint);
}
ESphereResponseQualityStatus EvaluateSelectedSphereResponseQuality(
    FPreparedGeometrySelectionPtr geometry,const FSphereResponseQualityValues& values,
    Analytic::Certified::FDistanceBudget& budget,
    FSelectedSphereResponseQualityPtr& output,FString& reason)
{
    using namespace Analytic::Certified;using namespace ResponseQualityDetail;
    const FSelectedSphereResponseQualityPtr keepPrevious=output;output.Reset();reason.Reset();
    Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    auto refuse=[&](const TCHAR* why){reason=why;return ESphereResponseQualityStatus::Unknown;};
    if(!geometry||!geometry->HasValidAliases()||!IsSupportedSphereResponseMargin(values.ResponseMargin)||!FMath::IsFinite(values.StaticContactOffsetCm)||values.StaticContactOffsetCm<0)
        return refuse(TEXT("Missing selected geometry or unsupported caller margin"));
    const auto owners=geometry->GetNominalScene()->GetOwners();
    if(values.Branches.Num()!=owners.Num()||values.AcceptedProviders.Num()!=owners.Num())return refuse(TEXT("Response branch values do not cover every selected provider"));
    for(int32 i=0;i<owners.Num();++i)
        if(!budget.Consume()||values.Branches[i].SurfaceId!=owners[i].SurfaceId||values.AcceptedProviders[i]>1)
            return refuse(TEXT("Caller response branches and immutable owner mapping differ"));
    auto selected=EvaluateSelectedSphereReference(geometry,values.Position,values.Velocity,values.Acceleration,
        values.Horizon,double(values.CollisionRadius),double(values.PhysicalRadius),values.Options,budget,
        [&](int32 index){return values.AcceptedProviders.IsValidIndex(index)&&values.AcceptedProviders[index]==1;});
    if(!selected.UsesGeometry(geometry)||selected.Reference.Status!=EReferenceSphereStatus::ReferenceContactBounded||
        !selected.Reference.ConsumedPrefixClear||selected.Reference.Contact.Leaves.IsEmpty())
        return refuse(TEXT("No bounded first contact and complete consumed prefix"));
    bool radial=false,haveBranch=false;
    for(const auto& leaf:selected.Reference.Contact.Leaves)
    {
        if(!budget.Consume()||!values.Branches.IsValidIndex(leaf.SurfaceIndex))return refuse(TEXT("Contact branch mapping unavailable"));
        const auto& b=values.Branches[leaf.SurfaceIndex];const bool next=SphereStaticResponse::UsesRadialPoint(b.SurfaceId,b.NormalMayVary,b.StartPenetrating,values.StaticContactOffsetCm);
        if(haveBranch&&next!=radial)return refuse(TEXT("First-contact leaves require incompatible response-point branches"));
        radial=next;haveBranch=true;
    }
    // Recompute the exact consumed SBox seconds and re-bound the ACTUAL bits.
    // No normalized Hit.Time*Delta round trip and no Rc-Rp response margin.
    const auto center=SBox::AdvancePosition(values.Position,values.Velocity,values.Acceleration,selected.Reference.ConsumedSeconds);
    FScalarSphereContact scalar;
    if(ConvertSphereContact(selected.Reference.Contact,0,selected.Reference.Scalar.ImpactPoint,center,
        double(values.CollisionRadius),double(values.PhysicalRadius),scalar)!=EScalarSphereContactStatus::Bounded||
        !SamePoint(center,selected.Reference.Scalar.Location))return refuse(TEXT("Consumed center or scalar re-bound unavailable"));
    const auto& branch=values.Branches[selected.Reference.Contact.Leaves[0].SurfaceIndex];
    const auto point=SphereStaticResponse::Point(center,values.PhysicalRadius,scalar.ImpactPoint,scalar.NormalStored,
        branch.SurfaceId,branch.NormalMayVary,branch.StartPenetrating,values.StaticContactOffsetCm);
    if(!SamePoint(point,radial?scalar.RadialResponsePoint:scalar.ImpactPoint))return refuse(TEXT("Actual response-point arithmetic and bounded bits differ"));
    const double error=radial?scalar.ResponseErrorCm:scalar.SurfaceErrorCm;
    if(!TimeDetail::NonnegativeFinite(error))return refuse(TEXT("Response position error unavailable"));
    TSharedPtr<FSelectedSphereResponseQuality,ESPMode::ThreadSafe> result(new FSelectedSphereResponseQuality);
    result->Geometry=MoveTemp(geometry);result->Values=values;result->Reference=MoveTemp(selected.Reference);
    result->ReboundedScalar=MoveTemp(scalar);result->ResponsePoint=point;result->PositionErrorCm=error;result->Radial=radial;
    result->Status=error<=double(result->Values.ResponseMargin)?ESphereResponseQualityStatus::BoundedWithinInputMargin:ESphereResponseQualityStatus::ErrorBoundExceedsInputMargin;
    Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);output=result;return result->Status;
}
}

namespace Speed::PreparedExecution
{
bool PrepareSelectedSphereResponseQualityValues(FPreparedGeometrySelectionPtr geometry,const FSphereResponseQueryValues& query,
    Analytic::Certified::FDistanceBudget& budget,FSphereResponseQualityValues& output,FString& reason)
{
    output={};reason.Reset();auto refuse=[&](const TCHAR* why){reason=why;return false;};
    if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
        !geometry->GetReference()||!IsSupportedSphereResponseMargin(query.ResponseMargin)||!FMath::IsFinite(query.StaticContactOffsetCm)||query.StaticContactOffsetCm<0)return refuse(TEXT("Retained response model or supported input margin unavailable"));
    const auto owners=geometry->GetNominalScene()->GetOwners();const auto reference=geometry->GetReference();
    const auto contracts=reference->GetSphereResponseContracts();const auto mapping=reference->GetProviderFaceIndices();
    if(contracts.Num()!=owners.Num()||mapping.Num()!=owners.Num()||owners.Num()>budget.Remaining||uint64(owners.Num())>MAX_uint64-budget.Used)
        return refuse(TEXT("Complete same-model response contract/work budget unavailable"));
    // Separate accounted metadata traversal: native bitmap N + metadata N +
    // quality validation N + contact leaves L =3N+L, beyond query work.
    for(int32 i=0;i<owners.Num();++i)if(!budget.Consume())return refuse(TEXT("Response contract traversal budget exhausted"));
    for(int32 i=0;i<owners.Num();++i)
        if(!contracts[i].StructurallyValid(mapping[i])||!contracts[i].IsRadialReacquisitionV1())
            return refuse(TEXT("Unknown or unsupported producer sphere response contract"));
    FSphereResponseQualityValues value;
    if(!BuildPreparedSphereAcceptanceBitmap(owners,query.Filter,budget,value.AcceptedProviders))return refuse(TEXT("Complete native filter bitmap budget unavailable"));
    value.Position=query.Position;value.Velocity=query.Velocity;value.Acceleration=query.Acceleration;value.Horizon=query.Horizon;
    value.CollisionRadius=query.CollisionRadius;value.PhysicalRadius=query.PhysicalRadius;value.ResponseMargin=query.ResponseMargin;value.StaticContactOffsetCm=query.StaticContactOffsetCm;value.Options=query.Options;
    value.Branches.Reserve(owners.Num());
    // False is an INPUT to the first-contact-only quality adapter, whose fresh
    // evaluation must establish FirstContactEnclosed AND complete prefix. It
    // never turns InitialInsideTube into a penetration classification/MTD.
    for(const auto& owner:owners)value.Branches.Add({owner.SurfaceId,true,false});
    output=MoveTemp(value);return true;
}
FNativeSphereResponseQualityEvaluation EvaluateNativeSphereResponseQuality(FNativeSphereRequestPtr request,
    const Analytic::Certified::FReferenceSphereOptions& options,Analytic::Certified::FDistanceBudget& budget,FString& reason)
{
    reason.Reset();if(!request||!RevalidateSphereCCD(*request)){reason=TEXT("Native sphere request/member unavailable");return {};}
    const auto geometry=request->GetGeometrySelection();const auto& v=request->GetValues();
    if(!geometry||!geometry->HasValidAliases()||geometry->GetNominalScene()!=request->GetScene()||!v.ResponseMarginCaptured||!v.ResponseScopeSerial)
        {reason=TEXT("Native response selection or virtual margin capture unavailable");return {};}
    FSphereResponseQueryValues query;query.Position=v.BodyState.Location;query.Velocity=v.BodyState.Velocity;query.Acceleration=v.BodyState.Acceleration;
    query.Horizon=v.Horizon;query.CollisionRadius=v.CollisionRadius;query.PhysicalRadius=v.PhysicalRadius;query.ResponseMargin=v.ResponseMargin;query.StaticContactOffsetCm=v.StaticContactOffsetCm;query.Options=options;query.Filter=v.Filter;
    FSphereResponseQualityValues prepared;if(!PrepareSelectedSphereResponseQualityValues(geometry,query,budget,prepared,reason))return {};
    FSelectedSphereResponseQualityPtr quality;
    const auto status=EvaluateSelectedSphereResponseQuality(geometry,prepared,budget,quality,reason);
    if(status==ESphereResponseQualityStatus::Unknown||!quality||!quality->MatchesValues(geometry,prepared)||!RevalidateSphereCCD(*request))return {};
    // ErrorBoundExceedsInputMargin may retain mathematical information for
    // refinement; it is never a response permission or proof of actual error.
    return {MoveTemp(request),geometry,MoveTemp(quality)};
}
}

namespace Speed::PreparedExecution
{
bool FSelectedSphereResponseQuality::MatchesConsumedContactAndNormal(float seconds,float physicalRadius,FVector3d location,
    FVector3d impactPoint,FVector3d normalStored,FVector3d normalConsumed,FVector3d responsePoint) const
{
    return MatchesConsumedContact(seconds,physicalRadius,location,impactPoint,normalStored,responsePoint)&&
        ResponseQualityDetail::SamePoint(normalConsumed,ReboundedScalar.NormalConsumed);
}
bool BuildMarginDrivenSphereReferenceOptions(const FSphereResponseQueryValues& q,int32 remaining,C::FReferenceSphereOptions& out)
{
    out={};Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!IsSupportedSphereResponseMargin(q.ResponseMargin)||q.ResponseMargin==0||remaining<=0||!FMath::IsFinite(q.Horizon)||q.Horizon<=0||
        !FMath::IsFinite(q.PhysicalRadius)||q.PhysicalRadius<=0||!C::ScalarContactDetail::Finite(q.Velocity)||!C::ScalarContactDetail::Finite(q.Acceleration))return false;
    const double goal=double(q.ResponseMargin)/16.;const double h=double(q.Horizon);
    const double travel=(FMath::Abs(q.Velocity.X)+FMath::Abs(q.Velocity.Y)+FMath::Abs(q.Velocity.Z))*h+
        .5*(FMath::Abs(q.Acceleration.X)+FMath::Abs(q.Acceleration.Y)+FMath::Abs(q.Acceleration.Z))*h*h;
    const double time=travel>0?FMath::Min(1./16.,goal/travel):1./16.;
    C::FReferenceSphereOptions v;v.TimeGap=time;v.SpatialGap=goal*goal;v.PointWidth=goal;
    v.NormalWidth=FMath::Min(.5,goal/double(q.PhysicalRadius));v.SpatialNodesPerAttempt=remaining;v.PrefixRangeAttempts=remaining;
    if(!AnalyticCertified::Finite(travel)||!C::ReferenceSphereDetail::OptionsValid(v)||v.TimeGap<=0||v.SpatialGap<=0||v.PointWidth<=0||v.NormalWidth<=0)return false;
    out=v;return true; // Heuristic precision goals; never an acceptance theorem.
}
bool BuildSelectedSphereHitValues(FPreparedGeometrySelectionPtr geometry,FSelectedSphereResponseQualityPtr quality,uint32 frame,
    C::FDistanceBudget& budget,FSelectedSphereHitValues& output,FString& reason)
{
    output={};reason.Reset();auto refuse=[&](const TCHAR* why){reason=why;return false;};
    if(!geometry||!geometry->HasValidAliases()||!quality||quality->GetGeometry()!=geometry||
        quality->GetStatus()!=ESphereResponseQualityStatus::BoundedWithinInputMargin||!quality->MatchesValues(geometry,quality->GetValues()))
        return refuse(TEXT("Selected bounded sphere quality unavailable"));
    const auto& r=quality->GetReference();const auto& scalar=quality->GetReboundedScalar();const auto owners=geometry->GetNominalScene()->GetOwners();
    if(r.Status!=C::EReferenceSphereStatus::ReferenceContactBounded||!r.ConsumedPrefixClear||r.Contact.Leaves.IsEmpty()||
        !quality->UsesRadialPoint()||!FMath::IsFinite(r.ConsumedSeconds)||r.ConsumedSeconds<0||r.ConsumedSeconds>=quality->GetValues().Horizon||!geometry->GetReference())
        return refuse(TEXT("Unsupported first-contact/prefix/seconds/response branch"));
    const auto contracts=geometry->GetReference()->GetSphereResponseContracts();const auto mapping=geometry->GetReference()->GetProviderFaceIndices();
    const auto& first=r.Contact.Leaves[0];if(!owners.IsValidIndex(first.SurfaceIndex))return refuse(TEXT("First-contact owner unavailable"));
    const auto& id=owners[first.SurfaceIndex];
    for(const auto& leaf:r.Contact.Leaves)
    {
        if(!budget.Consume()||!owners.IsValidIndex(leaf.SurfaceIndex)||!contracts.IsValidIndex(leaf.SurfaceIndex)||!mapping.IsValidIndex(leaf.SurfaceIndex))return refuse(TEXT("ALL-leaf owner/contract budget unavailable"));
        const auto& o=owners[leaf.SurfaceIndex];const auto& c=contracts[leaf.SurfaceIndex];
        if(!c.StructurallyValid(mapping[leaf.SurfaceIndex])||!c.IsRadialReacquisitionV1()||o.ObjectType!=ECC_WorldStatic||
            o.SourceId!=id.SourceId||o.SurfaceId!=id.SurfaceId||o.FeatureId!=id.FeatureId||o.PrimitiveId!=id.PrimitiveId||
            o.CanonicalGroupId!=id.CanonicalGroupId||o.MaterialId!=id.MaterialId)return refuse(TEXT("First-contact ALL-leaf identity/material contract is not unique-compatible"));
    }
    Analytic::FWorldHit h;h.bHit=true;h.Location=scalar.Location;h.Point=scalar.ImpactPoint;h.Normal=scalar.NormalStored;
    h.QueryPoint=quality->GetResponsePoint();h.bSurfaceNormalMayVary=true;h.GeometricErrorBoundCm=scalar.SurfaceErrorCm;
    h.SourceId=id.SourceId;h.SurfaceId=id.SurfaceId;h.FeatureId=id.FeatureId;h.PrimitiveId=id.PrimitiveId;h.CanonicalGroupId=id.CanonicalGroupId;h.MaterialId=id.MaterialId;
    const auto hit=SHitResult::FromAnalyticHitAtSeconds(h,r.ConsumedSeconds,frame);
    const auto actualN=hit.ImpactNormal.GetSafeNormal();
    const auto actualP=SphereStaticResponse::Point(hit.Location,quality->GetValues().PhysicalRadius,hit.ImpactPoint,
        hit.ImpactNormal,hit.SurfaceId,hit.bSurfaceNormalMayVary,hit.bStartPenetrating,quality->GetValues().StaticContactOffsetCm);
    if(!FMath::IsFinite(hit.GeometricErrorBoundCm)||hit.GeometricErrorBoundCm<0||double(hit.GeometricErrorBoundCm)<scalar.SurfaceErrorCm||
        !quality->MatchesConsumedContactAndNormal(hit.TOI,quality->GetValues().PhysicalRadius,hit.Location,hit.ImpactPoint,hit.ImpactNormal,actualN,actualP)||
        !ResponseQualityDetail::SamePoint(actualP,hit.ContactPointThis)||!SphereStaticResponse::UsesRadialPoint(hit.SurfaceId,hit.bSurfaceNormalMayVary,hit.bStartPenetrating,quality->GetValues().StaticContactOffsetCm))
        return refuse(TEXT("Final named hit bits or outward witness bound unavailable"));
    output={MoveTemp(geometry),MoveTemp(quality),hit};return true;
}
bool SamePreparedHitValues(const SHitResult& a,const SHitResult& b) noexcept
{
    using namespace ResponseQualityDetail;
    return a.bHit==b.bHit&&a.bBlockingHit==b.bBlockingHit&&SamePoint(a.Location,b.Location)&&SamePoint(a.ImpactPoint,b.ImpactPoint)&&SamePoint(a.ImpactNormal,b.ImpactNormal)&&
        FloatBits(a.TOI)==FloatBits(b.TOI)&&a.FaceIndex==b.FaceIndex&&a.Component.IsExplicitlyNull()&&b.Component.IsExplicitlyNull()&&
        a.SubBody.IsExplicitlyNull()&&b.SubBody.IsExplicitlyNull()&&a.bStartPenetrating==b.bStartPenetrating&&
        FloatBits(a.PenetrationDepth)==FloatBits(b.PenetrationDepth)&&FloatBits(a.GeometricErrorBoundCm)==FloatBits(b.GeometricErrorBoundCm)&&a.bSurfaceNormalMayVary==b.bSurfaceNormalMayVary&&
        SamePoint(a.ContactPointThis,b.ContactPointThis)&&SamePoint(a.ContactPointOther,b.ContactPointOther)&&a.ContactFeatureThis==b.ContactFeatureThis&&a.ContactFeatureOther==b.ContactFeatureOther&&
        a.ContactFeatureIndexThis==b.ContactFeatureIndexThis&&a.ContactFeatureIndexOther==b.ContactFeatureIndexOther&&a.SourceId==b.SourceId&&a.SurfaceId==b.SurfaceId&&a.FeatureId==b.FeatureId&&
        a.PrimitiveId==b.PrimitiveId&&a.CanonicalGroupId==b.CanonicalGroupId&&a.MaterialId==b.MaterialId&&a.FrameTag==b.FrameTag&&a.FirstRsResponseData==b.FirstRsResponseData;
}
}

#if !UE_BUILD_SHIPPING
namespace Speed::PreparedExecution
{
void AppendPreparedGroundEvidenceState(TArray<uint8>& out,const FGroundSweepEvidencePtr& e)
{
    if(!e)return;
    auto add=[&out](const auto& value){out.Append(reinterpret_cast<const uint8*>(&value),sizeof(value));};
    const uint32 magic=0x33534750;add(magic);add(e->Kind);add(e->Sealed);
    const bool captured=bool(e->Request);add(captured);if(!captured)return;
    const auto& step=e->Request->GetStep();const auto& v=e->Request->GetValues();
    add(step.InputGeneration);add(step.Frame);add(step.StepSerial);add(step.QueryEpoch);add(step.QuerySerial);
    add(step.NumericControl);add(step.Iteration);add(step.Delta);add(step.SimTime);add(step.Remaining);add(step.TimePassed);
    add(v.OwnerStableId);add(v.BodyIndex);add(v.ResponseScopeSerial);add(v.ResponseMarginCaptured);add(v.ResponseMargin);
    add(v.Horizon);add(v.CollisionRadius);add(v.PhysicalRadius);add(v.RadiusInflation);add(v.StaticSupportRadius);add(v.StaticContactOffsetCm);add(v.StaticImpactFriction);add(v.Frozen);add(v.Prescribed);
    auto point=[&add](const auto& p){add(p.X);add(p.Y);add(p.Z);};
    auto state=[&](const FKinematicState& k){point(k.Location);point(k.Velocity);point(k.Acceleration);
        add(k.Rotation.X);add(k.Rotation.Y);add(k.Rotation.Z);add(k.Rotation.W);point(k.AngularVelocity);point(k.AngularAcceleration);};
    state(v.OwnerState);state(v.BodyState);point(v.LocalOffset);point(v.LocalCOM);
    add(v.LocalRotation.X);add(v.LocalRotation.Y);add(v.LocalRotation.Z);add(v.LocalRotation.W);point(v.Start);point(v.End);
    const bool selected=bool(e->Geometry);add(selected);if(!selected)return;
    const auto model=e->Geometry->GetModel();add(model);const bool reference=bool(e->Geometry->GetReference());add(reference);
    if(reference){const auto& bytes=e->Geometry->GetReference()->GetPacketByteIdentity().GetBytes();out.Append(bytes,sizeof(bytes));}
    const bool seeded=bool(e->StateSeed);add(seeded);
    if(seeded)
    {
        const auto& seed=*e->StateSeed;state(seed.OwnerHistoryAtFrame);add(seed.GroundHit);add(seed.GroundSeconds);
        add(seed.WholeHullExclusion);add(seed.NativeLocalInputSupported);point(seed.NativeLocalInput.RelativeLocation);
        add(seed.NativeLocalInput.RelativeRotation.X);add(seed.NativeLocalInput.RelativeRotation.Y);
        add(seed.NativeLocalInput.RelativeRotation.Z);add(seed.NativeLocalInput.RelativeRotation.W);
        add(seed.NativeLocalInput.bWheel);add(seed.NativeLocalInput.WheelSpringDisplacement);
    }
    // Diagnostic state only. These semantic bytes cannot authenticate a live
    // lease, request, first contact, event identity or physical authorization.
}
}
#endif

namespace Speed::PreparedExecution
{
bool SameSphereFreeAdvanceValues(const FSphereFreeAdvanceValues& a,const FSphereFreeAdvanceValues& b) noexcept
{
    return SameKinematicBits(a.OwnerBefore,b.OwnerBefore)&&SameKinematicBits(a.BodyBefore,b.BodyBefore)&&
        SameVector(a.LocalCOM,b.LocalCOM)&&SameVector(a.Local.RelativeLocation,b.Local.RelativeLocation)&&
        SameQuat(a.Local.RelativeRotation,b.Local.RelativeRotation)&&a.Local.bWheel==b.Local.bWheel&&
        SameFloat(a.Local.WheelSpringDisplacement,b.Local.WheelSpringDisplacement)&&SameFloat(a.Seconds,b.Seconds)&&
        SameFloat(a.MaximumSpeed,b.MaximumSpeed)&&SameFloat(a.MaximumAngularSpeed,b.MaximumAngularSpeed);
}
bool BuildSphereFreeAdvanceExpectation(const FSphereFreeAdvanceValues& v,FSphereFreeAdvanceExpectation& out,FString& reason)
{
    out={};reason.Reset();Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    auto fail=[&](const TCHAR* text){reason=text;return false;};uint32 timeBits=0;FMemory::Memcpy(&timeBits,&v.Seconds,4);
    if((timeBits&0x80000000u)||(timeBits!=0&&timeBits<0x00800000u)||!FMath::IsFinite(v.Seconds)||!FiniteState(v.OwnerBefore)||!FiniteState(v.BodyBefore)||
        !C::ScalarContactDetail::Finite(v.LocalCOM)||!C::ScalarContactDetail::Finite(v.Local.RelativeLocation)||
        v.Local.bWheel||!FMath::IsFinite(v.MaximumSpeed)||v.MaximumSpeed<=0||
        !FMath::IsFinite(v.MaximumAngularSpeed)||v.MaximumAngularSpeed<=0)
        return fail(TEXT("Unsupported finite free-advance values"));
    FSphereFreeAdvanceExpectation value;value.Seconds=v.Seconds;value.OwnerAfter=v.OwnerBefore;value.BodyAfter=v.BodyBefore;
    if(timeBits==0){value.Known=true;out=value;return true;} // actual IntegrateKinematics returns before ALL arithmetic/sync
    if(!FMath::IsFinite(v.Local.RelativeRotation.X)||!FMath::IsFinite(v.Local.RelativeRotation.Y)||
        !FMath::IsFinite(v.Local.RelativeRotation.Z)||!FMath::IsFinite(v.Local.RelativeRotation.W))return fail(TEXT("Local rotation unavailable"));
    const auto originBefore=CalculateOriginKinematicStateValues(v.OwnerBefore,v.LocalCOM);
    if(!SameKinematicBits(FrameParallel::CalculateSubBodyKinematics(originBefore,v.Local),v.BodyBefore))
        return fail(TEXT("Pure body/origin input routing differs"));
    value.OwnerAfter=v.OwnerBefore.Integrate(v.Seconds);if(!FiniteState(value.OwnerAfter))return fail(TEXT("Native primitive overflow"));
    if(!SameVector(value.OwnerAfter.Velocity,value.OwnerAfter.Velocity.GetClampedToMaxSize(v.MaximumSpeed))||
        !SameVector(value.OwnerAfter.AngularVelocity,value.OwnerAfter.AngularVelocity.GetClampedToMaxSize(v.MaximumAngularSpeed)))
        return fail(TEXT("Effective velocity cap needs its own motion contract"));
    value.OriginAfter=CalculateOriginKinematicStateValues(value.OwnerAfter,v.LocalCOM);
    value.BodyAfter=FrameParallel::CalculateSubBodyKinematics(value.OriginAfter,v.Local);if(!FiniteState(value.BodyAfter))return fail(TEXT("Native child synchronization overflow"));
    value.Known=true;value.PerformsIntegration=true;out=value;return true;
}
#include "PreparedSphereRsTrajectoryCapture.inl"
#include "PreparedSpherePostSyncRsCapture.inl"

}

namespace Speed::PreparedExecution
{
#include "PreparedConsumedPrefixAdvance.inl"
#include "PreparedFirstRsResponse.inl"
#include "PreparedFirstRsResponseSync.inl"
}

#include "PreparedFirstArrivalRsResponseQuality.inl"
#include "PreparedFirstArrivalRsHitValues.inl"
#include "PreparedGeneralFirstArrivalRsResponseQuality.inl"
#include "PreparedGeneralFirstArrivalRsHitValues.inl"
#include "NativeGeneralSphereContactValues.inl"
#include "NativeCanonicalBoxSweepClearanceValues.inl"
#include "NativeBoxPoseCorrectionGeometryValues.inl"
#include "NativeCanonicalAxialBoxPlaneValues.inl"
#include "PreparedFirstRsNonImpactResponse.inl"

#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
namespace Speed::PreparedExecution
{
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRawCensusValueMemoTest,"IAmSpeed.AnalyticWorld.Prepared.RawCensusValueMemo",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FRawCensusValueMemoTest::RunTest(const FString&)
{
    // Pure metadata sentinels: no UObject, adapter or body method is called.
    ISpeedComponent* a=reinterpret_cast<ISpeedComponent*>(UPTRINT(0x1000));
    ISpeedComponent* b=reinterpret_cast<ISpeedComponent*>(UPTRINT(0x2000));
    TArray<FPreparedNativeOwnerBinding> rows;
    for(auto* owner:{a,b})
    {
        FPreparedNativeOwnerBinding row;row.Adapter=owner;row.NativeObject=reinterpret_cast<UActorComponent*>(UPTRINT(owner));
        row.Class=reinterpret_cast<void*>(UPTRINT(0x3000));row.BodyCensusCaptured=true;rows.Add(MoveTemp(row));
    }
    TArray<ISpeedComponent*> adapters{a,b},ordered{b,a};
    FRawCensusValueMemo memo;int identityA=0,identityB=0;bool reused=false;
    auto verifyCensus=[&](const void* identity,bool pending,bool dirty)
    {
        const auto expected=InspectCanonicalCensusForRefusal(rows,adapters,ordered,pending,dirty);
        const auto actual=memo.Evaluate(identity,rows,adapters,ordered,pending,dirty,reused);
        TestTrue(TEXT("same complete refusal classifier"),actual==expected);
    };
    verifyCensus(&identityA,false,false);TestFalse(TEXT("first rows inspection"),reused);
    verifyCensus(&identityA,false,false);TestTrue(TEXT("immutable rows and exact actual arrays reused"),reused);
    verifyCensus(&identityA,true,false);TestFalse(TEXT("pending mutation still fresh"),reused);
    verifyCensus(&identityA,false,true);TestFalse(TEXT("dirty order still fresh"),reused);
    verifyCensus(&identityA,false,false);TestTrue(TEXT("revocation does not mutate pure cached value"),reused);
    ordered[0]=a;verifyCensus(&identityA,false,false);TestFalse(TEXT("ordered duplicate invalidates"),reused);
    ordered[0]=b;verifyCensus(&identityA,false,false);TestFalse(TEXT("restored arrays recomputed"),reused);
    adapters[1]=a;verifyCensus(&identityA,false,false);TestFalse(TEXT("adapter duplicate invalidates"),reused);
    adapters[1]=b;verifyCensus(&identityA,false,false);
    verifyCensus(&identityB,false,false);TestFalse(TEXT("different immutable owner collection invalidates"),reused);
    rows[0].Class=nullptr;
    verifyCensus(nullptr,false,false);TestFalse(TEXT("no immutable identity never memoized"),reused);
    rows[0].Class=reinterpret_cast<void*>(UPTRINT(0x3000));
    adapters.Add(a);verifyCensus(&identityA,false,false);TestFalse(TEXT("inventory length invalidates"),reused);
    return true;
}
}
#endif
