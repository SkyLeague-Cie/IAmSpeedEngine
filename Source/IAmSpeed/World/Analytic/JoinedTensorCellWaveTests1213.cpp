#if WITH_DEV_AUTOMATION_TESTS
#include "JoinedTensorCellWave1213.h"
#include "CompleteCellChunks1353.h"
#include "IAmSpeed/World/Collision/StaticCollisionWorld.h"
#include "IAmSpeed/World/Simulation/FrameParallelExecutor.h"
#include "IAmSpeed/World/Simulation/FrameParallelSettings.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTLS.h"
#include "Misc/AutomationTest.h"
#include <atomic>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <cfenv>
#include <limits>

namespace Speed::Analytic::ParallelTests1075
{bool SameHit(const FWorldHit&,const FWorldHit&);}
namespace Speed::Analytic::TensorCellTests1213
{
struct FScopedInt
{
    IConsoleVariable* Variable;int32 Previous;
    FScopedInt(const TCHAR* Name,int32 Value):Variable(IConsoleManager::Get().FindConsoleVariable(Name)),Previous(Variable?Variable->GetInt():0)
    {if(Variable)Variable->Set(Value,ECVF_SetByCode);}
    ~FScopedInt(){if(Variable)Variable->Set(Previous,ECVF_SetByCode);}
    bool Is(int32 Value)const{return Variable&&Variable->GetInt()==Value;}
};
bool World(FAnalyticWorldData& Data,FString& Reason)
{
    // A single provider with64 independent authored cells. The two providers
    // below also exercise outerON/innerON without a nested private pool.
    for(int32 Provider=0;Provider<2;++Provider)
    {
        FPiecewiseTensorBezierPatch P;
        P.SourceId=Provider+1;P.SurfaceId=11+Provider;P.PrimitiveId=21+Provider;P.CanonicalGroupId=31+Provider;
        P.ObjectType=1;P.BlockingChannels=MAX_uint64;P.bQueryCollisionEnabled=true;
        for(int32 Cell=0;Cell<64;++Cell)
        {
            auto& C=P.Cells.AddDefaulted_GetRef();
            C.FeatureId=1000+Cell;C.PrimitiveId=2000+Provider*100+Cell;
            C.MinimumU=double(Cell)/64;C.MaximumU=double(Cell+1)/64;
            C.Surface.DegreeU=1;C.Surface.DegreeV=1;
            const double X0=-320+10*Cell,X1=X0+10;
            C.Surface.ControlPoints={FVector3d(X0,-100,0),FVector3d(X0,100,0),FVector3d(X1,-100,0),FVector3d(X1,100,0)};
        }
        Data.PiecewiseTensorBezierPatches.Add(MoveTemp(P));
    }
    if(!Data.FinalizeAndValidate(&Reason))return false;
    for(auto& P:Data.PiecewiseTensorBezierPatches)P.bAuthorityEligible=true;
    return true;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedTensorCellGrainPlan1213,
    "IAmSpeed.Parallel.Analytic1213.TensorCellGrainPlan",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedTensorCellGrainPlan1213::RunTest(const FString& Parameters)
{
    using namespace Speed::Analytic;using namespace TensorCellTests1213;
    FScopedInt Minimum(TEXT("p.IAmSpeed.Parallel.TensorCellMinItems1213"),32);
    FScopedInt Grain(TEXT("p.IAmSpeed.Parallel.TensorCellMinItemsPerLane1213"),8);
    FScopedInt Chunk(TEXT("p.IAmSpeed.Parallel.TensorCellChunkItems1213"),4);
    if(!Minimum.Is(32)||!Grain.Is(8)||!Chunk.Is(4))return false;
    for(int32 Requested:{1,2,4,8,12})
    for(int32 N:{0,1,2,8,31,32,33,63,64,95,96,1024,4096,4097})
    {
        const int32 L=FJoinedTensorCellWave1213::ResolveLanes(N,Requested);
        if(!TestTrue(TEXT("admitted lanes bounded"),L>=1&&L<=Requested))return false;
        if(N<32||N>4096)TestEqual(TEXT("small or unbounded work stays serial"),L,1);
        if(L>1)
        {
            TArray<int32> Visits;Visits.Init(0,N);
            for(int32 Lane=0;Lane<L;++Lane)
                for(int32 Begin=Lane*4;Begin<N;Begin+=L*4)
                    for(int32 I=Begin;I<FMath::Min(Begin+4,N);++I)++Visits[I];
            for(const int32 Count:Visits)if(!TestEqual(TEXT("chunk partition covers every cell once"),Count,1))return false;
            TestTrue(TEXT("admission has enough independent complete cells"),N/L>=8);
        }
    }
    for(int32 Count:{0,1,2,3,31,32,33,63,64,65,4095,4096})
    for(int32 Lanes:{1,2,4,8,12})for(int32 ChunkSize:{1,2,4,8,32,4096})
    {
        FCompleteCellChunks1353 Claims(Count,Lanes,ChunkSize);
        if(!TestTrue(TEXT("bounded complete-cell assignment admitted"),Claims.IsValid()))return false;
        TArray<int32> Seen;Seen.Init(0,Count);
        const auto Visit=[&](const FCompleteCellChunks1353::FRange& R)
        {
            if(R.Begin>=R.End||R.End>uint32(Count)||R.End-R.Begin>uint32(ChunkSize))return false;
            for(uint32 K=R.Begin;K<R.End;++K)++Seen[int32(K)];return true;
        };
        FCompleteCellChunks1353::FRange R;
        for(int32 Lane=Lanes-1;Lane>=0;--Lane)
            if(Claims.InitialRangeForLane(Lane,R)&&!Visit(R))return false;
        while(Claims.TakeNext(R))if(!Visit(R))return false;
        bool Once=true;for(int32 V:Seen)Once&=V==1;
        if(!TestTrue(TEXT("every complete cell claimed exactly once including partial tails"),Once))return false;
        R={123,456};for(int32 N=0;N<32;++N)
            if(!TestTrue(TEXT("exhausted queue stays saturated without changing the output"),!Claims.TakeNext(R)&&R.Begin==123&&R.End==456))return false;
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedTensorCellJoinedKeysAndConcurrency1213,
    "IAmSpeed.Parallel.Analytic1213.TensorCellJoinedKeysAndConcurrency",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedTensorCellJoinedKeysAndConcurrency1213::RunTest(const FString& Parameters)
{
    using namespace Speed;using namespace Analytic;using namespace TensorCellTests1213;
    FScopedInt Enabled(TEXT("p.IAmSpeed.Parallel.TensorCellWave1213"),1),Observe(TEXT("p.IAmSpeed.Parallel.TensorCellObserve1213"),1);
    FScopedInt Minimum(TEXT("p.IAmSpeed.Parallel.TensorCellMinItems1213"),2),Grain(TEXT("p.IAmSpeed.Parallel.TensorCellMinItemsPerLane1213"),1),Chunk(TEXT("p.IAmSpeed.Parallel.TensorCellChunkItems1213"),1);
    FScopedInt CausalOff(TEXT("p.IAmSpeed.Parallel.AnalyticCausal1075"),0);
    if(!Enabled.Is(1)||!Observe.Is(1)||!Minimum.Is(2)||!Grain.Is(1)||!Chunk.Is(1)||!CausalOff.Is(0))return false;
    FScopedInt Distribution(TEXT("p.IAmSpeed.Parallel.TensorCellDynamic1353"),1);
    if(!Distribution.Is(1))return false;
    EnsurePrivateFrameWorkers(11);ResetTensorCellWaveStats1213();
    FWorldQuery Q;Q.Shape=EQueryShape::Box;Q.Start=FVector3d(0,0,4);Q.End=FVector3d(0,0,-4);Q.HalfExtent=FVector3d(2,2,2);
    const FBox3d B(FVector3d(-8,-8,-8),FVector3d(8,8,8));
    int32 W=1,I=2,V=3;const FTensorCellWaveIdentity1213 Key{&W,&I,&V,0};const TArray<int32> Indices{0,1,2,3};
    std::atomic<int32> Active{0},Peak{0},Arrived{0},Calls{0};std::atomic<bool> Timeout{false};
    const auto Evaluate=[&](int32 Index,double Bound)
    {
        ++Calls;const int32 Current=++Active;int32 Expected=Peak.load();while(Expected<Current&&!Peak.compare_exchange_weak(Expected,Current)){}
        ++Arrived;const auto Deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
        while(Arrived.load()<4&&std::chrono::steady_clock::now()<Deadline)std::this_thread::yield();
        if(Arrived.load()<4)Timeout=true;
        FWorldHit H;H.bHit=true;H.Time=Bound;H.SourceId=Index+1;--Active;return H;
    };
    {
        FJoinedTensorCellWave1213 Wave(Q,B,Key,Indices,4,4,[](){return 1.0;},Evaluate);
        if(!TestTrue(TEXT("four private lanes joined"),Wave.IsReady()))return false;
        TestFalse(TEXT("concurrency barrier completed within bound"),Timeout.load());
        TestTrue(TEXT("useful callbacks really overlapped"),Peak.load()>1);
        TestEqual(TEXT("every immutable cell evaluated once"),Calls.load(),4);
        FWorldHit H;
        TestTrue(TEXT("canonical first result is consumed"),Wave.TryConsume(0,Q,B,Key,1.0,H));
        TestEqual(TEXT("canonical output identity retained"),H.SourceId,uint64(1));
        TestFalse(TEXT("no result consumed twice"),Wave.TryConsume(0,Q,B,Key,1.0,H));
        TestFalse(TEXT("tightened current bound rejects speculation"),Wave.TryConsume(1,Q,B,Key,0.5,H));
        auto Changed=Q;Changed.InitialOverlapTolerance=0.123;
        TestFalse(TEXT("changed query rejects speculation"),Wave.TryConsume(2,Changed,B,Key,1.0,H));
        auto ChangedKey=Key;ChangedKey.Selection=1;
        TestFalse(TEXT("changed operation policy rejects speculation"),Wave.TryConsume(3,Q,B,ChangedKey,1.0,H));
    }
    const auto S=SnapshotTensorCellWaveStats1213();
    TestTrue(TEXT("actual multiple native participants were observed"),S.NativeMax>1&&S.UsefulNativeMax>1&&S.LogicalMax==4);
    TestEqual(TEXT("reused plus discarded is exact"),S.Computed,S.Reused+S.Wasted);
    TestTrue(TEXT("three key failures were measured"),S.RejectBound==1&&S.RejectQuery==1&&S.RejectContext==1);
    TestEqual(TEXT("consumed logical lanes differ from merely completed callbacks"),S.ReusedLaneMax1213,1);
    TestEqual(TEXT("consumed native participants differ from completed participants"),S.ReusedNativeMax1213,1);
    TestEqual(TEXT("timed completed kernels split into consumed and discarded"),S.ComputedKernelCycles1213,S.ReusedKernelCycles1213+S.WastedKernelCycles1213);
    // Deferred failure belongs to its canonical cell, not the dispatch point.
    const auto Throwing=[](int32 Index,double Bound)->FWorldHit
    {if(Index==1)throw std::runtime_error("tensor-cell-fixture");FWorldHit H;H.Time=Bound;return H;};
    {
        FJoinedTensorCellWave1213 Wave(Q,B,Key,Indices,4,4,[](){return 1.0;},Throwing);
        FWorldHit H;TestTrue(TEXT("earlier canonical success remains consumable"),Wave.TryConsume(0,Q,B,Key,1.0,H));
        bool Threw=false;try{Wave.TryConsume(1,Q,B,Key,1.0,H);}catch(const std::runtime_error&){Threw=true;}
        TestTrue(TEXT("cell exception deferred until canonical consumption"),Threw);
    }
    std::atomic<int32> NestedCalls{0};
    {
        FScopedFrameParallelTask Outer;
        FJoinedTensorCellWave1213 Wave(Q,B,Key,Indices,4,4,[](){return 1.0;},[&](int32,double){++NestedCalls;return FWorldHit();});
        TestFalse(TEXT("existing private task refuses a nested inner pool"),Wave.IsReady());
    }
    TestEqual(TEXT("nested refusal executes no hidden serial work"),NestedCalls.load(),0);
    ResetTensorCellWaveStats1213();return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedTensorCellWholeKernelOracle1213,
    "IAmSpeed.Parallel.Analytic1213.TensorCellWholeKernelOracle",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedTensorCellWholeKernelOracle1213::RunTest(const FString& Parameters)
{
    using namespace Speed;using namespace Analytic;using namespace TensorCellTests1213;
    FScopedInt Inner(TEXT("p.IAmSpeed.Parallel.TensorCellWave1213"),0),Observe(TEXT("p.IAmSpeed.Parallel.TensorCellObserve1213"),1),Outer(TEXT("p.IAmSpeed.Parallel.PiecewiseProviderWave1075"),0);
    FScopedInt Minimum(TEXT("p.IAmSpeed.Parallel.TensorCellMinItems1213"),2),Grain(TEXT("p.IAmSpeed.Parallel.TensorCellMinItemsPerLane1213"),1),Chunk(TEXT("p.IAmSpeed.Parallel.TensorCellChunkItems1213"),1);
    FScopedInt CausalOff(TEXT("p.IAmSpeed.Parallel.AnalyticCausal1075"),0);
    if(!Inner.Is(0)||!Outer.Is(0)||!Observe.Is(1)||!Minimum.Is(2)||!Grain.Is(1)||!Chunk.Is(1)||!CausalOff.Is(0))return false;
    FAnalyticWorldData Data;FString Reason;if(!World(Data,Reason)){AddError(Reason);return false;}
    const auto NumericEntry1213=CaptureFrameFloatingPointContext();
    FScopedFrameFloatingPointContext WholeTestNumeric1213(NumericEntry1213);
    if(!WholeTestNumeric1213.IsApplied())return false;
    TArray<FWorldQuery> Queries;
    for(int32 Index=0;Index<18;++Index)
    {
        FWorldQuery Q;Q.Shape=EQueryShape(Index%3);Q.Radius=3;Q.HalfExtent=FVector3d(340,110,4);
        Q.Start=FVector3d((Index%3-1)*5,0,20);Q.End=FVector3d(Q.Start.X,0,-20);Q.DomainTolerance=0;Q.InitialOverlapTolerance=0;
        Q.bAuthorityOnly=true;Q.bIncludeCompactPatches=true;
        if(Index%4==0)Q.Start=Q.End=FVector3d(0,0,1);
        if(Index%5==0){Q.ReferenceNormal=FVector3d::DownVector;Q.MinimumReferenceNormalDot=0.9;}
        if(Index%7==0)Q.Start.X=Q.End.X=-0.0;
        Queries.Add(Q);
    }
    FAnalyticStaticCollisionWorld Serial(Data,1,2,2);TArray<FWorldHit> Expected;
    TArray<FFrameFloatingPointContext> ExpectedNumeric1213;
    for(const auto& Q:Queries)
    {
        if(!ApplyFrameFloatingPointContext(NumericEntry1213))return false;
        Expected.Add(Serial.SweepSingle(Q));ExpectedNumeric1213.Add(CaptureFrameFloatingPointContext());
    }
    TArray<FWorldQuery> Poses;TArray<FWorldHit> DeepestExpected;TArray<bool> HasDeepestExpected;
    FWorldQueryService SerialService1213(Data,1,2);
    for(int32 Index=0;Index<Queries.Num();++Index)
    {
        auto Pose=Queries[Index];Pose.End=Pose.Start;Pose.Start.Z=Pose.End.Z=1;
        Poses.Add(Pose);FWorldHit H;
        HasDeepestExpected.Add(SerialService1213.TryFindDeepestOverlap(Pose,H));DeepestExpected.Add(H);
    }
    FScopedInt Distribution(TEXT("p.IAmSpeed.Parallel.TensorCellDynamic1353"),0);
    if(!Distribution.Is(0))return false;
    uint64 RealCellWaves=0,RealComputed=0;
    for(int32 DistributionValue:{0,1})
    for(int32 OuterValue:{0,1})
    for(int32 Lanes:{1,2,4,8,12})
    {
        Distribution.Variable->Set(DistributionValue,ECVF_SetByCode);
        Inner.Variable->Set(1,ECVF_SetByCode);Outer.Variable->Set(OuterValue,ECVF_SetByCode);
        ResetTensorCellWaveStats1213();FAnalyticStaticCollisionWorld Actual(Data,Lanes,2,2);
        for(int32 Index=0;Index<Queries.Num();++Index)
        {
            if(!ApplyFrameFloatingPointContext(NumericEntry1213))return false;
            const auto H=Actual.SweepSingle(Queries[Index]);const auto NumericAfter1213=CaptureFrameFloatingPointContext();
            if(!TestTrue(TEXT("whole Tensor cell retains exact output FP environment"),
                NumericAfter1213.bValid&&NumericAfter1213.VectorControl==ExpectedNumeric1213[Index].VectorControl&&
                FMemory::Memcmp(&NumericAfter1213.Environment,&ExpectedNumeric1213[Index].Environment,sizeof(std::fenv_t))==0))return false;
            if(!ParallelTests1075::SameHit(H,Expected[Index]))
            {AddError(FString::Printf(TEXT("whole Tensor cell divergence outer=%d lanes=%d query=%d"),OuterValue,Lanes,Index));return false;}
        }
        FWorldQueryService ActualService1213(Data,Lanes,2);
        for(int32 Index=0;Index<Poses.Num();++Index)
        {
            FWorldHit H;const bool Has=ActualService1213.TryFindDeepestOverlap(Poses[Index],H);
            if(Has!=HasDeepestExpected[Index] || !ParallelTests1075::SameHit(H,DeepestExpected[Index]))
            {AddError(FString::Printf(TEXT("deepest Tensor cell divergence outer=%d lanes=%d pose=%d"),OuterValue,Lanes,Index));return false;}
        }
        const auto S=SnapshotTensorCellWaveStats1213();RealCellWaves+=S.Waves;RealComputed+=S.Computed;
        if(OuterValue==0&&Lanes>1)TestTrue(TEXT("real geometry entered an inner cell wave"),S.Waves>0&&S.Computed>0&&S.NativeMax>1);
        // A rejected outer result can re-enter the inner strategy on the owner
        // AFTER its previous outer wave joined. That is legal, not nesting.
        TestTrue(TEXT("inner strategy never claims more than requested lanes"),S.LogicalMax<=Lanes);
    }
    TestTrue(TEXT("oracle includes joined real Tensor geometry"),RealCellWaves>0&&RealComputed>0);
    ResetTensorCellWaveStats1213();return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedTensorCellNumericAndRefusals1213,
    "IAmSpeed.Parallel.Analytic1213.TensorCellNumericAndRefusals",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedTensorCellNumericAndRefusals1213::RunTest(const FString& Parameters)
{
    using namespace Speed;using namespace Analytic;using namespace TensorCellTests1213;
    FScopedInt Enabled(TEXT("p.IAmSpeed.Parallel.TensorCellWave1213"),1),Observe(TEXT("p.IAmSpeed.Parallel.TensorCellObserve1213"),1);
    FScopedInt Minimum(TEXT("p.IAmSpeed.Parallel.TensorCellMinItems1213"),2),Grain(TEXT("p.IAmSpeed.Parallel.TensorCellMinItemsPerLane1213"),1),Chunk(TEXT("p.IAmSpeed.Parallel.TensorCellChunkItems1213"),1);
    if(!Enabled.Is(1)||!Observe.Is(1)||!Minimum.Is(2)||!Grain.Is(1)||!Chunk.Is(1))return false;
    const auto Entry=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext Numeric(Entry);
    if(!Numeric.IsApplied())return false;
    FWorldQuery Q;const FBox3d B(FVector3d(-1,-1,-1),FVector3d(1,1,1));
    int32 W=1,I=2,V=3;const FTensorCellWaveIdentity1213 Key{&W,&I,&V,0};const TArray<int32> Indices{0,1};
    std::atomic<int32> Calls{0};const auto Work=[&](int32 Index,double Bound){++Calls;FWorldHit H;H.Time=Bound;H.SourceId=Index+1;return H;};
    {
        FJoinedTensorCellWave1213 Wave(Q,B,Key,Indices,2,2,[](){return 1.0;},Work);
        if(!TestTrue(TEXT("numeric fixture dispatched"),Wave.IsReady()))return false;
        const auto BeforeChange=CaptureFrameFloatingPointContext();
        if(std::fesetround(FE_DOWNWARD)!=0){AddError(TEXT("native rounding change unavailable"));return false;}
        const auto Changed=CaptureFrameFloatingPointContext();FWorldHit H;H.SourceId=999;
        const bool Consumed=Wave.TryConsume(0,Q,B,Key,1.0,H);const auto After=CaptureFrameFloatingPointContext();
        const bool Preserved=Changed.bValid&&After.bValid&&Changed.VectorControl==After.VectorControl&&
            FMemory::Memcmp(&Changed.Environment,&After.Environment,sizeof(std::fenv_t))==0;
        if(!ApplyFrameFloatingPointContext(BeforeChange))return false;
        TestFalse(TEXT("changed numeric input forces exact serial recompute"),Consumed);
        TestTrue(TEXT("numeric refusal leaves caller environment untouched"),Preserved);
        TestEqual(TEXT("numeric refusal leaves result untouched"),H.SourceId,uint64(999));
    }
    {
        FJoinedTensorCellWave1213 Wave(Q,B,Key,Indices,2,2,[](){return -0.0;},Work);
        FWorldHit H;TestFalse(TEXT("signed-zero bound key is exact"),Wave.TryConsume(0,Q,B,Key,+0.0,H));
    }
    const int32 BeforeRefusals=Calls.load();
    {
        FJoinedTensorCellWave1213 Invalid(Q,B,Key,Indices,2,2,[](){return std::numeric_limits<double>::quiet_NaN();},Work);
        TestFalse(TEXT("nonfinite entry bound refuses native admission"),Invalid.IsReady());
    }
    {
        const TArray<int32> Duplicate{0,0};
        FJoinedTensorCellWave1213 Invalid(Q,B,Key,Duplicate,2,2,[](){return 1.0;},Work);
        TestFalse(TEXT("duplicate result keys refuse admission"),Invalid.IsReady());
    }
    {
        FJoinedTensorCellWave1213 Invalid(Q,B,Key,Indices,4097,2,[](){return 1.0;},Work);
        TestFalse(TEXT("capacity refusal is bounded"),Invalid.IsReady());
    }
    TestEqual(TEXT("refusals execute no hidden serial or partial native work"),Calls.load(),BeforeRefusals);
    ResetTensorCellWaveStats1213();
    {
        FScopedInt ObserveOff(TEXT("p.IAmSpeed.Parallel.TensorCellObserve1213"),0);
        FScopedTensorCellEligibilityObservation1213 Off;
    }
    auto ScopeStats1213=SnapshotTensorCellWaveStats1213();
    TestEqual(TEXT("observation OFF creates no measured eligibility sample"),ScopeStats1213.OwnerEligibilitySeen1213,uint64(0));
    {
        FScopedInt NoTiming(TEXT("p.IAmSpeed.Parallel.TensorCellMaxSamples1213"),0);
        FScopedTensorCellEligibilityObservation1213 Skipped;
    }
    ScopeStats1213=SnapshotTensorCellWaveStats1213();
    TestTrue(TEXT("zero budget records UNKNOWN eligibility cost rather than a timed zero"),
        ScopeStats1213.OwnerEligibilitySeen1213==1&&ScopeStats1213.OwnerEligibilityTimed1213==0&&
        ScopeStats1213.OwnerEligibilitySkipped1213==1&&ScopeStats1213.OwnerEligibilityCycles1213==0);
    ResetTensorCellWaveStats1213();
    {
        FScopedInt OneTiming(TEXT("p.IAmSpeed.Parallel.TensorCellMaxSamples1213"),1);
        {FScopedTensorCellEligibilityObservation1213 Complete;}
        try {FScopedTensorCellEligibilityObservation1213 Partial;throw std::runtime_error("eligibility-partial-fixture");}
        catch(const std::runtime_error&){}
    }
    ScopeStats1213=SnapshotTensorCellWaveStats1213();
    TestTrue(TEXT("eligibility cap and partial failure remain explicit"),
        ScopeStats1213.OwnerEligibilitySeen1213==2&&ScopeStats1213.OwnerEligibilityTimed1213==1&&
        ScopeStats1213.OwnerEligibilitySkipped1213==1&&ScopeStats1213.OwnerEligibilityFailed1213==1);
    ResetTensorCellWaveStats1213();
    {
        FScopedInt TooFew(TEXT("p.IAmSpeed.Parallel.TensorCellMinItems1213"),32);
        FJoinedTensorCellWave1213 Refused(Q,B,Key,Indices,2,2,[](){return 1.0;},Work);
        TestFalse(TEXT("small constructor refuses without timing classification"),Refused.IsReady());
    }
    ScopeStats1213=SnapshotTensorCellWaveStats1213();
    TestTrue(TEXT("pre-timing refusal is distinguishable from complete timing coverage"),
        ScopeStats1213.Attempts==1&&ScopeStats1213.TimedAttempts==0&&ScopeStats1213.TimingSkippedAttempts==0&&
        ScopeStats1213.PreparationMeasuredAttempts1213==0&&ScopeStats1213.DispatchMeasuredAttempts1213==0);
    ResetTensorCellWaveStats1213();return true;
}
namespace Speed::Analytic::TensorCellTests1213
{
bool SameNumeric1534(const FFrameFloatingPointContext& A,const FFrameFloatingPointContext& B)
{
    return A.bValid&&B.bValid&&A.VectorControl==B.VectorControl&&
        FMemory::Memcmp(&A.Environment,&B.Environment,sizeof(std::fenv_t))==0;
}
bool ChangingWorld1534(FAnalyticWorldData& Data,FString& Reason)
{
    FPiecewiseTensorBezierPatch P;
    P.SourceId=1;P.SurfaceId=11;P.PrimitiveId=21;P.CanonicalGroupId=31;
    P.ObjectType=1;P.BlockingChannels=MAX_uint64;P.bQueryCollisionEnabled=true;
    for(int32 Cell=0;Cell<64;++Cell)
    {
        auto& C=P.Cells.AddDefaulted_GetRef();
        C.FeatureId=1000+Cell;C.PrimitiveId=2000+Cell;
        C.MinimumU=double(Cell)/64;C.MaximumU=double(Cell+1)/64;
        C.Surface.DegreeU=1;C.Surface.DegreeV=1;
        const double X0=-320+10*Cell,X1=X0+10;
        // Connected ascending cells, then a long flat tail. Canonical cells
        // initially improve TOI, then provide enough stable serial cells for
        // a fresh bounded wave. No assumption about the BVH builder's order.
        const double Z0=FMath::Min(Cell,7),Z1=FMath::Min(Cell+1,7);
        C.Surface.ControlPoints={FVector3d(X0,-100,Z0),FVector3d(X0,100,Z0),
            FVector3d(X1,-100,Z1),FVector3d(X1,100,Z1)};
    }
    Data.PiecewiseTensorBezierPatches.Add(MoveTemp(P));
    if(!Data.FinalizeAndValidate(&Reason))return false;
    auto& Patch=Data.PiecewiseTensorBezierPatches[0];Patch.bAuthorityEligible=true;
    if(Patch.CellBvhNodes.IsEmpty())return false;
    const auto RootBounds=Patch.CellBvhNodes[0].Bounds;
    Patch.CellBvhNodes.SetNum(1);auto& Root=Patch.CellBvhNodes[0];
    Root.Bounds=RootBounds;Root.LeftChild=INDEX_NONE;Root.RightChild=INDEX_NONE;
    Root.FirstIndex=0;Root.IndexCount=Patch.Cells.Num();
    Patch.CellBvhIndices.Reset();
    for(int32 Cell=0;Cell<Patch.Cells.Num();++Cell)Patch.CellBvhIndices.Add(Cell);
    return true;
}
FWorldQuery WideBox1534()
{
    FWorldQuery Q;Q.Shape=EQueryShape::Box;Q.HalfExtent=FVector3d(340,110,4);
    Q.Start=FVector3d(0,0,20);Q.End=FVector3d(0,0,-20);
    Q.DomainTolerance=0;Q.InitialOverlapTolerance=0;
    Q.bAuthorityOnly=true;Q.bIncludeCompactPatches=true;return Q;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedTensorCellWindowKeys1534,
    "IAmSpeed.Parallel.Analytic1534.TensorCellWindowKeys",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedTensorCellWindowKeys1534::RunTest(const FString& Parameters)
{
    using namespace Speed;using namespace Analytic;using namespace TensorCellTests1213;
    FScopedInt Window(TEXT("p.IAmSpeed.Parallel.TensorCellWindowItems1534"),0);
    if(!Window.Is(0))return false;
    TestEqual(TEXT("zero preserves the one-shot legacy strategy"),FJoinedTensorCellWave1213::WindowCandidateItems1534(),0);
    for(int32 Requested:{-1,1,2,8,4096,4097})
    {
        Window.Variable->Set(Requested,ECVF_SetByCode);
        const int32 Expected=Requested<=0?0:FMath::Clamp(Requested,2,4096);
        TestEqual(TEXT("window admission is an integer bounded policy"),FJoinedTensorCellWave1213::WindowCandidateItems1534(),Expected);
    }
    Window.Variable->Set(8,ECVF_SetByCode);
    FScopedInt Enabled(TEXT("p.IAmSpeed.Parallel.TensorCellWave1213"),1),Observe(TEXT("p.IAmSpeed.Parallel.TensorCellObserve1213"),1);
    FScopedInt Minimum(TEXT("p.IAmSpeed.Parallel.TensorCellMinItems1213"),2),Grain(TEXT("p.IAmSpeed.Parallel.TensorCellMinItemsPerLane1213"),1),Chunk(TEXT("p.IAmSpeed.Parallel.TensorCellChunkItems1213"),1);
    FScopedInt CausalOff(TEXT("p.IAmSpeed.Parallel.AnalyticCausal1075"),0);
    if(!Enabled.Is(1)||!Observe.Is(1)||!Minimum.Is(2)||!Grain.Is(1)||!Chunk.Is(1)||!CausalOff.Is(0))return false;
    EnsurePrivateFrameWorkers(7);ResetTensorCellWaveStats1213();
    const auto CallerEntry=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext Numeric(CallerEntry);
    if(!Numeric.IsApplied())return false;
    if(std::feclearexcept(FE_ALL_EXCEPT)!=0)return false;
    const auto Entry=CaptureFrameFloatingPointContext();
    FWorldQuery Q;const FBox3d B(FVector3d(-1,-1,-1),FVector3d(1,1,1));
    int32 W=1,I=2,V=3;const FTensorCellWaveIdentity1213 Key{&W,&I,&V,0};const TArray<int32> Indices{0,1};
    const auto Evaluate=[](int32 Index,double Bound)->FWorldHit
    {
        if(Index==1)
        {
            std::feraiseexcept(FE_INEXACT);
            if(Bound>0.75)throw std::runtime_error("stale-tensor-window-exception1534");
        }
        FWorldHit H;H.bHit=true;H.Time=Bound;H.SourceId=Index+1;return H;
    };
    {
        FJoinedTensorCellWave1213 Wave(Q,B,Key,Indices,2,2,[](){return 1.0;},Evaluate,ETensorCellWindowReason1534::First);
        if(!TestTrue(TEXT("initial window fully joined"),Wave.IsReady()))return false;
        TestTrue(TEXT("membership finds recorded cells only"),Wave.ContainsProvider1534(0)&&Wave.ContainsProvider1534(1)&&!Wave.ContainsProvider1534(2));
        if(!ApplyFrameFloatingPointContext(Entry))return false;
        const auto BeforeProbe=CaptureFrameFloatingPointContext();
        const bool SameBound=Wave.HasSameBound1534([](){std::feraiseexcept(FE_INEXACT);return 1.0;});
        const auto AfterSameProbe=CaptureFrameFloatingPointContext();
        const bool ChangedBound=Wave.HasSameBound1534([](){std::feraiseexcept(FE_INEXACT);return 0.5;});
        const auto AfterChangedProbe=CaptureFrameFloatingPointContext();
        TestTrue(TEXT("exact clamped bound can retain a live window"),SameBound);
        TestFalse(TEXT("changed bound retires a live window"),ChangedBound);
        TestTrue(TEXT("extra bound probes restore the exact caller FP environment"),
            SameNumeric1534(BeforeProbe,AfterSameProbe)&&SameNumeric1534(BeforeProbe,AfterChangedProbe));
        const auto Before=CaptureFrameFloatingPointContext();FWorldHit H;H.SourceId=999;
        bool Threw=false,Consumed=false;
        try{Consumed=Wave.TryConsume(1,Q,B,Key,0.5,H);}catch(...){Threw=true;}
        const auto AfterRejected=CaptureFrameFloatingPointContext();
        TestFalse(TEXT("stale-bound deferred exception is never authoritative"),Threw);
        TestFalse(TEXT("membership does not authorize reuse across a tighter bound"),Consumed);
        TestEqual(TEXT("rejected window leaves output untouched"),H.SourceId,uint64(999));
        TestTrue(TEXT("rejected window leaves exact sticky/control state untouched"),SameNumeric1534(Before,AfterRejected));
    }
    if(!ApplyFrameFloatingPointContext(Entry))return false;
    const auto Expected=Evaluate(1,0.5);const auto ExpectedNumeric=CaptureFrameFloatingPointContext();
    TestTrue(TEXT("sticky-state fixture raises a nontrivial inexact flag"),std::fetestexcept(FE_INEXACT)!=0);
    if(!ApplyFrameFloatingPointContext(Entry))return false;
    {
        FJoinedTensorCellWave1213 Wave(Q,B,Key,Indices,2,2,[](){return 0.5;},Evaluate,ETensorCellWindowReason1534::BoundChanged);
        FWorldHit H;
        const bool Consumed=Wave.TryConsume(1,Q,B,Key,0.5,H);
        const auto ActualNumeric=CaptureFrameFloatingPointContext();
        if(!TestTrue(TEXT("fresh rebased key is consumable"),Consumed))return false;
        TestTrue(TEXT("fresh complete-cell output remains exact"),ParallelTests1075::SameHit(H,Expected));
        TestTrue(TEXT("fresh complete-cell output publishes exact sticky/control state"),SameNumeric1534(ExpectedNumeric,ActualNumeric));
        TestFalse(TEXT("fresh rebased result remains single use"),Wave.TryConsume(1,Q,B,Key,0.5,H));
    }
    const auto S=SnapshotTensorCellWaveStats1213();
    TestTrue(TEXT("window counters classify attempts without changing existing refusal counters"),
        S.WindowAttempts1534==2&&S.WindowWaves1534==2&&S.WindowBoundRebases1534==1&&S.WindowCandidatesMax1534==2&&S.RejectBound==1);
    TestEqual(TEXT("window computed work partitions into reused and wasted"),S.Computed,S.Reused+S.Wasted);
    ResetTensorCellWaveStats1213();return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedTensorCellWindowWholeKernelOracle1534,
    "IAmSpeed.Parallel.Analytic1534.TensorCellWindowWholeKernelOracle",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedTensorCellWindowWholeKernelOracle1534::RunTest(const FString& Parameters)
{
    using namespace Speed;using namespace Analytic;using namespace TensorCellTests1213;
    FScopedInt Inner(TEXT("p.IAmSpeed.Parallel.TensorCellWave1213"),0),Window(TEXT("p.IAmSpeed.Parallel.TensorCellWindowItems1534"),0);
    FScopedInt Observe(TEXT("p.IAmSpeed.Parallel.TensorCellObserve1213"),1),Outer(TEXT("p.IAmSpeed.Parallel.PiecewiseProviderWave1075"),0),CausalOff(TEXT("p.IAmSpeed.Parallel.AnalyticCausal1075"),0);
    FScopedInt Minimum(TEXT("p.IAmSpeed.Parallel.TensorCellMinItems1213"),2),Grain(TEXT("p.IAmSpeed.Parallel.TensorCellMinItemsPerLane1213"),1),Chunk(TEXT("p.IAmSpeed.Parallel.TensorCellChunkItems1213"),1);
    FScopedInt Distribution(TEXT("p.IAmSpeed.Parallel.TensorCellDynamic1353"),0);
    if(!Inner.Is(0)||!Window.Is(0)||!Observe.Is(1)||!Outer.Is(0)||!CausalOff.Is(0)||!Minimum.Is(2)||!Grain.Is(1)||!Chunk.Is(1)||!Distribution.Is(0))return false;
    FAnalyticWorldData Data;FString Reason;if(!World(Data,Reason)){AddError(Reason);return false;}
    const auto Entry=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext Numeric(Entry);
    if(!Numeric.IsApplied())return false;
    const auto Q=WideBox1534();FAnalyticStaticCollisionWorld Serial(Data,1,2,2);
    if(!ApplyFrameFloatingPointContext(Entry))return false;
    const auto Expected=Serial.SweepSingle(Q);const auto ExpectedNumeric=CaptureFrameFloatingPointContext();
    auto Pose=Q;Pose.Start.Z=Pose.End.Z=1;
    FWorldQueryService SerialService(Data,1,2);FWorldHit ExpectedOverlap;
    if(!ApplyFrameFloatingPointContext(Entry))return false;
    const bool ExpectedHasOverlap=SerialService.TryFindDeepestOverlap(Pose,ExpectedOverlap);
    const auto ExpectedOverlapNumeric=CaptureFrameFloatingPointContext();
    EnsurePrivateFrameWorkers(7);Inner.Variable->Set(1,ECVF_SetByCode);
    for(int32 DistributionValue:{0,1})for(int32 Lanes:{1,8})for(int32 WindowItems:{0,2,8,32,64,128,256})
    {
        Distribution.Variable->Set(DistributionValue,ECVF_SetByCode);Window.Variable->Set(WindowItems,ECVF_SetByCode);
        ResetTensorCellWaveStats1213();FAnalyticStaticCollisionWorld Actual(Data,Lanes,2,2);
        if(!ApplyFrameFloatingPointContext(Entry))return false;
        const auto H=Actual.SweepSingle(Q);
        const auto ActualNumeric=CaptureFrameFloatingPointContext();
        if(!TestTrue(TEXT("bounded suffix traversal retains every hit field and exact FP state"),
            ParallelTests1075::SameHit(H,Expected)&&SameNumeric1534(ExpectedNumeric,ActualNumeric)))return false;
        FWorldQueryService Service(Data,Lanes,2);FWorldHit Overlap;
        if(!ApplyFrameFloatingPointContext(Entry))return false;
        const bool HasOverlap=Service.TryFindDeepestOverlap(Pose,Overlap);
        const auto ActualOverlapNumeric=CaptureFrameFloatingPointContext();
        if(!TestTrue(TEXT("bounded suffix traversal retains deepest selection and exact FP state"),
            HasOverlap==ExpectedHasOverlap&&ParallelTests1075::SameHit(Overlap,ExpectedOverlap)&&SameNumeric1534(ExpectedOverlapNumeric,ActualOverlapNumeric)))return false;
        const auto S=SnapshotTensorCellWaveStats1213();
        if(WindowItems==0||Lanes==1)TestEqual(TEXT("legacy and serial controls construct no bounded windows"),S.WindowAttempts1534,uint64(0));
        else
        {
            TestTrue(TEXT("real complete-cell windows dispatch and remain inside the configured cap"),
                S.WindowWaves1534>0&&S.WindowCandidatesMax1534<=WindowItems&&S.LogicalMax<=Lanes);
            if(WindowItems<32)TestTrue(TEXT("real BVH suffix advances across multiple joined windows"),S.WindowAdvances1534>0);
            if(WindowItems==2)TestTrue(TEXT("each provider invocation refuses its one-cell tail only once"),
                S.WindowAttempts1534>S.WindowWaves1534&&S.WindowAttempts1534-S.WindowWaves1534<=4);
        }
        TestEqual(TEXT("all observed speculative work is accounted for"),S.Computed,S.Reused+S.Wasted);
    }
    ResetTensorCellWaveStats1213();return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedTensorCellWindowRebaseOracle1534,
    "IAmSpeed.Parallel.Analytic1534.TensorCellWindowRebaseOracle",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedTensorCellWindowRebaseOracle1534::RunTest(const FString& Parameters)
{
    using namespace Speed;using namespace Analytic;using namespace TensorCellTests1213;
    FScopedInt Inner(TEXT("p.IAmSpeed.Parallel.TensorCellWave1213"),0),Window(TEXT("p.IAmSpeed.Parallel.TensorCellWindowItems1534"),0);
    FScopedInt Observe(TEXT("p.IAmSpeed.Parallel.TensorCellObserve1213"),1),Outer(TEXT("p.IAmSpeed.Parallel.PiecewiseProviderWave1075"),0),CausalOff(TEXT("p.IAmSpeed.Parallel.AnalyticCausal1075"),0);
    FScopedInt Minimum(TEXT("p.IAmSpeed.Parallel.TensorCellMinItems1213"),2),Grain(TEXT("p.IAmSpeed.Parallel.TensorCellMinItemsPerLane1213"),1),Chunk(TEXT("p.IAmSpeed.Parallel.TensorCellChunkItems1213"),1);
    FScopedInt Distribution(TEXT("p.IAmSpeed.Parallel.TensorCellDynamic1353"),0);
    if(!Inner.Is(0)||!Window.Is(0)||!Observe.Is(1)||!Outer.Is(0)||!CausalOff.Is(0)||!Minimum.Is(2)||!Grain.Is(1)||!Chunk.Is(1)||!Distribution.Is(0))return false;
    FAnalyticWorldData Data;FString Reason;if(!ChangingWorld1534(Data,Reason)){AddError(Reason);return false;}
    const auto Entry=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext Numeric(Entry);
    if(!Numeric.IsApplied())return false;
    const auto Q=WideBox1534();FAnalyticStaticCollisionWorld Serial(Data,1,2,2);
    if(!ApplyFrameFloatingPointContext(Entry))return false;
    const auto Expected=Serial.SweepSingle(Q);const auto ExpectedNumeric=CaptureFrameFloatingPointContext();
    EnsurePrivateFrameWorkers(7);Inner.Variable->Set(1,ECVF_SetByCode);
    for(int32 DistributionValue:{0,1})
    {
        Distribution.Variable->Set(DistributionValue,ECVF_SetByCode);Window.Variable->Set(0,ECVF_SetByCode);
        ResetTensorCellWaveStats1213();FAnalyticStaticCollisionWorld Legacy(Data,8,2,2);
        if(!ApplyFrameFloatingPointContext(Entry))return false;
        const auto LegacyHit=Legacy.SweepSingle(Q);const auto LegacyNumeric=CaptureFrameFloatingPointContext();
        const auto LegacyStats=SnapshotTensorCellWaveStats1213();
        if(!TestTrue(TEXT("changing-TOI fixture first proves the legacy exact oracle"),
            ParallelTests1075::SameHit(LegacyHit,Expected)&&SameNumeric1534(ExpectedNumeric,LegacyNumeric)))return false;
        TestTrue(TEXT("changing-TOI fixture exercises stale-bound rejection"),LegacyStats.RejectBound>0);
        Window.Variable->Set(8,ECVF_SetByCode);ResetTensorCellWaveStats1213();FAnalyticStaticCollisionWorld Actual(Data,8,2,2);
        if(!ApplyFrameFloatingPointContext(Entry))return false;
        const auto H=Actual.SweepSingle(Q);const auto ActualNumeric=CaptureFrameFloatingPointContext();
        const auto S=SnapshotTensorCellWaveStats1213();
        if(!TestTrue(TEXT("retired windows and stable serial prefix retain every hit field and exact FP state"),
            ParallelTests1075::SameHit(H,Expected)&&SameNumeric1534(ExpectedNumeric,ActualNumeric)))return false;
        TestTrue(TEXT("changing TOI eventually rebases a fresh fully joined bounded window"),
            S.WindowBoundRebases1534==1&&S.WindowWaves1534>1&&S.WindowCandidatesMax1534<=8);
        TestTrue(TEXT("settling prefix bounds repeated compute and refused tail probes"),
            S.Computed<LegacyStats.Computed&&S.Wasted<=8&&S.Wasted<LegacyStats.Wasted&&
            S.WindowAttempts1534<=9&&S.WindowAttempts1534-S.WindowWaves1534<=1);
        TestEqual(TEXT("discarded joined windows remain exactly accounted"),S.Computed,S.Reused+S.Wasted);
    }
    ResetTensorCellWaveStats1213();return true;
}

#endif
