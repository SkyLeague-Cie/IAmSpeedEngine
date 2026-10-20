#if WITH_DEV_AUTOMATION_TESTS
#include "AnalyticWorldQuery.h"
#include "AnalyticFacetWorkPlan1075.h"
#include "AnalyticCausalStats1075.h"
#include "IAmSpeed/World/Collision/StaticCollisionWorld.h"
#include "IAmSpeed/World/Simulation/FrameParallelExecutor.h"
#include "IAmSpeed/World/Simulation/FrameParallelSettings.h"
#include "HAL/IConsoleManager.h"
#include "Misc/AutomationTest.h"
#include "Algo/Reverse.h"
#include <atomic>

namespace Speed::Analytic::ParallelTests1075
{
struct FScopedIntCVar
{
    IConsoleVariable* Variable;
    int32 Previous;
    FScopedIntCVar(const TCHAR* Name, int32 Value)
        : Variable(IConsoleManager::Get().FindConsoleVariable(Name)),
          Previous(Variable ? Variable->GetInt() : 0)
    { if (Variable) Variable->Set(Value, ECVF_SetByCode); }
    ~FScopedIntCVar() { if (Variable) Variable->Set(Previous, ECVF_SetByCode); }
    bool Is(int32 Value) const { return Variable && Variable->GetInt() == Value; }
};
bool SameHit(const FWorldHit& A, const FWorldHit& B)
{
    const auto Bits = [](const auto& X, const auto& Y)
    { return FMemory::Memcmp(&X, &Y, sizeof(X)) == 0; };
    const auto Vec = [&](const FVector3d& X, const FVector3d& Y)
    { return Bits(X.X,Y.X) && Bits(X.Y,Y.Y) && Bits(X.Z,Y.Z); };
    return A.bHit==B.bHit && A.bStartPenetrating==B.bStartPenetrating &&
        Bits(A.Time,B.Time) && Bits(A.PenetrationDepth,B.PenetrationDepth) &&
        Bits(A.GeometricErrorBoundCm,B.GeometricErrorBoundCm) &&
        Bits(A.AdditionalResidualAgreementAllowanceCm,B.AdditionalResidualAgreementAllowanceCm) &&
        A.bSurfaceNormalMayVary==B.bSurfaceNormalMayVary && Vec(A.Location,B.Location) &&
        Vec(A.Point,B.Point) && Vec(A.QueryPoint,B.QueryPoint) && Vec(A.Normal,B.Normal) &&
        A.QueryFeatureKind==B.QueryFeatureKind && A.SurfaceFeatureKind==B.SurfaceFeatureKind &&
        A.QueryFeatureIndex==B.QueryFeatureIndex && A.SurfaceFeatureIndex==B.SurfaceFeatureIndex &&
        A.SourceId==B.SourceId && A.SurfaceId==B.SurfaceId && A.FeatureId==B.FeatureId &&
        A.PrimitiveId==B.PrimitiveId && A.CanonicalGroupId==B.CanonicalGroupId && A.MaterialId==B.MaterialId;
}
bool PiecewiseWorld(FAnalyticWorldData& Data, FString& Reason)
{
    // Two incident finite patches share the seam x=0. An overlapping copy with
    // a different identity makes geometric ties visible to every hit field.
    for (int32 Provider = 0; Provider < 2; ++Provider)
    {
        FPiecewiseTensorBezierPatch Patch;
        Patch.SourceId=1+Provider; Patch.SurfaceId=11+Provider;
        Patch.PrimitiveId=21+Provider; Patch.CanonicalGroupId=31+Provider;
        Patch.ObjectType=1; Patch.BlockingChannels=MAX_uint64;
        Patch.bQueryCollisionEnabled=true;
        for (int32 CellIndex = 0; CellIndex < 2; ++CellIndex)
        {
            FPiecewiseTensorBezierCell Cell;
            Cell.FeatureId=101+CellIndex; Cell.PrimitiveId=201+10*Provider+CellIndex;
            Cell.MinimumU=0.5*CellIndex; Cell.MaximumU=0.5*(CellIndex+1);
            Cell.Surface.DegreeU=1; Cell.Surface.DegreeV=1;
            const double X0=-100.0+100.0*CellIndex, X1=X0+100.0;
            Cell.Surface.ControlPoints={FVector3d(X0,-100,0),FVector3d(X0,100,0),
                FVector3d(X1,-100,0),FVector3d(X1,100,0)};
            Patch.Cells.Add(MoveTemp(Cell));
        }
        Data.PiecewiseTensorBezierPatches.Add(MoveTemp(Patch));
    }
    if (!Data.FinalizeAndValidate(&Reason)) return false;
    for (auto& Patch : Data.PiecewiseTensorBezierPatches) Patch.bAuthorityEligible=true;
    return true;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedFacetGrainPlan1075,
    "IAmSpeed.Parallel.Analytic1075.FacetGrainPlan",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedFacetGrainPlan1075::RunTest(const FString& Parameters)
{
    using namespace Speed::Analytic;
    for (int32 Count : {0,1,2,3,31,32,33,63,64,65,127,128,129,383,384,385,4095,4096})
    for (int32 Budget : {1,2,4,8,12})
    for (int32 Grain : {0,1,4,16,32,64,4096})
    {
        const auto Plan=FAnalyticFacetWorkPlan1075::Resolve(Count,Budget,2,Grain);
        TArray<int32> Visits; Visits.Init(0,Count);
        int32 Smallest=MAX_int32;
        for (int32 Lane=0; Lane<Plan.Lanes; ++Lane)
        {
            int32 Items=0;
            for (int32 Index=Plan.First(Lane); Index<Plan.End(Lane); Index+=Plan.Stride())
            {
                if (!TestTrue(TEXT("partition stays within result extent"),Visits.IsValidIndex(Index))) return false;
                ++Visits[Index]; ++Items;
            }
            Smallest=FMath::Min(Smallest,Items);
        }
        for (int32 VisitsToItem : Visits)
            if (!TestEqual(TEXT("every canonical index is evaluated once"),VisitsToItem,1)) return false;
        if (Grain>0 && Plan.Lanes>1)
            if (!TestTrue(TEXT("each admitted lane meets minimum grain"),Smallest>=Grain)) return false;
        if (!TestTrue(TEXT("bounded lane count"),Plan.Lanes>=1 && Plan.Lanes<=Budget)) return false;
        if (Grain==0 && Count>=2)
            if (!TestEqual(TEXT("legacy zero retains lane count"),Plan.Lanes,FMath::Min(Budget,Count))) return false;
    }
    TestEqual(TEXT("64 facets at grain32 admit two lanes"),FAnalyticFacetWorkPlan1075::Resolve(64,12,32,32).Lanes,2);
    TestEqual(TEXT("63 facets at grain32 remain serial"),FAnalyticFacetWorkPlan1075::Resolve(63,12,32,32).Lanes,1);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedPiecewiseWholeSweepChunks1075,
    "IAmSpeed.Parallel.Analytic1075.PiecewiseWholeSweepChunks",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedPiecewiseWholeSweepChunks1075::RunTest(const FString& Parameters)
{
    using namespace Speed;
    using namespace Analytic;
    using namespace ParallelTests1075;
    FScopedIntCVar CausalOff(TEXT("p.IAmSpeed.Parallel.AnalyticCausal1075"),0);
    FAnalyticWorldData Data; FString Reason;
    if (!TestTrue(TEXT("piecewise seam/tie fixture validates"),PiecewiseWorld(Data,Reason)))
    { AddError(Reason); return false; }
    FAnalyticStaticCollisionWorld Serial(Data);
    TArray<FWorldQuery> Queries; TArray<FWorldHit> Expected;
    uint64 PositiveHits=0, Dispatches=0;
    for (int32 Index=0; Index<96; ++Index)
    {
        FWorldQuery Q; Q.Shape=EQueryShape(Index%3);
        Q.Start=FVector3d((Index%9-4)*24, (Index%5-2)*30, 30);
        Q.End=FVector3d(Q.Start.X,Q.Start.Y,-30);
        Q.Radius=3; Q.HalfExtent=FVector3d(5,7,4);
        Q.bAuthorityOnly=true; Q.bIncludeCompactPatches=true;
        Q.DomainTolerance=0; Q.InitialOverlapTolerance=0;
        if (Index%7==0) Q.Start=Q.End=FVector3d(0,0,1);
        if (Index%11==0) Q.Start.X=Q.End.X=-0.0;
        if (Index%13==0) Q.RequiredSourceId=2;
        if (Index%17==0) Q.Rotation=FRotator3d(13,27,11).Quaternion();
        Queries.Add(Q); Expected.Add(Serial.SweepSingle(Q));
        PositiveHits+=Expected.Last().bHit;
    }
    TestTrue(TEXT("fixture includes finite contacts"),PositiveHits>0);
    for (int32 Chunk : {0,1,2,4,16,256})
    {
        FScopedIntCVar Mode(TEXT("p.IAmSpeed.Parallel.QueryChunkItems1075"),Chunk);
        if (!TestTrue(TEXT("chunk override is active"),Mode.Is(Chunk))) return false;
        for (int32 Lanes : {1,2,4,8,12})
        {
            FAnalyticStaticCollisionWorld World(Data,Lanes,2,2);
            TArray<FWorldHit> Actual; Actual.SetNum(Queries.Num());
            for (int32 Repeat=0; Repeat<3; ++Repeat)
            {
                FIndependentQueryBatchStats Stats;
                if (!TestTrue(TEXT("whole sweeps join"),World.SweepIndependentBatch(Queries,Actual,&Stats))) return false;
                int32 Completed=0;
                for(int32 Lane=0;Lane<Stats.DispatchedLogicalLanes;++Lane) Completed+=Stats.ExecutedItems1075[Lane];
                if(!TestTrue(TEXT("actual completed-query accounting is known"),Stats.bWorkCountsKnown1075))return false;
                if(!TestEqual(TEXT("all queries completed once across private lanes"),Completed,Queries.Num()))return false;
                if(Chunk==256 && Lanes>1)
                    if(!TestEqual(TEXT("one large chunk has one useful lane despite admitted callbacks"),Stats.NonEmptyLanes1075,1))return false;
                if(Chunk==0)
                    if(!TestEqual(TEXT("strided assignment gives every lane useful work"),Stats.NonEmptyLanes1075,Stats.DispatchedLogicalLanes))return false;
                if (Lanes>1)
                {
                    if (!TestTrue(TEXT("positive owned dispatch is required"),Stats.Fallback==EIndependentQueryFallback::None)) return false;
                    ++Dispatches;
                }
                for (int32 Index=0; Index<Queries.Num(); ++Index)
                    if (!SameHit(Expected[Index],Actual[Index]))
                    { AddError(FString::Printf(TEXT("piecewise divergence chunk=%d lanes=%d repeat=%d query=%d"),Chunk,Lanes,Repeat,Index)); return false; }
                Algo::Reverse(Queries); Algo::Reverse(Expected);
            }
            {
                FScopedFrameParallelTask Nested;
                FIndependentQueryBatchStats Stats;
                if (!World.SweepIndependentBatch(Queries,Actual,&Stats)) return false;
                if (Lanes>1 && !TestTrue(TEXT("nested whole-sweep batches stay serial"),Stats.Fallback==EIndependentQueryFallback::NestedTask)) return false;
                for (int32 Index=0; Index<Queries.Num(); ++Index)
                    if (!SameHit(Expected[Index],Actual[Index])) return false;
            }
        }
    }
    TestTrue(TEXT("oracle exercised owned parallel work"),Dispatches>0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedAnalyticCausalBoundAndOff1075,
    "IAmSpeed.Parallel.Analytic1075.CausalBoundAndOff",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedAnalyticCausalBoundAndOff1075::RunTest(const FString& Parameters)
{
    using namespace Speed::Analytic;
    using namespace ParallelTests1075;
    FScopedIntCVar Enabled(TEXT("p.IAmSpeed.Parallel.AnalyticCausal1075"),1);
    FScopedIntCVar Limit(TEXT("p.IAmSpeed.Parallel.AnalyticCausalMaxSamples1075"),2);
    if (!Enabled.Is(1) || !Limit.Is(2)) return false;
    ResetAnalyticCausalStats1075();
    for (int32 Index=0; Index<3; ++Index)
    {
        FScopedAnalyticCausalObservation1075 Scope(EAnalyticCausalKind1075::ExtrudedProvider);
        Scope.SetItems(4);
        if (uint64* Numeric=Scope.NumericTarget(0)) *Numeric=7;
    }
    const auto On=SnapshotAnalyticCausalStats1075().Kinds[int32(EAnalyticCausalKind1075::ExtrudedProvider)];
    TestEqual(TEXT("all observer admissions counted"),On.Seen,uint64(3));
    TestEqual(TEXT("bounded samples"),On.Sampled,uint64(2));
    TestEqual(TEXT("skips are explicit"),On.Skipped,uint64(1));
    TestEqual(TEXT("private numeric accumulators fold after scopes"),On.NumericCycles,uint64(14));
    ResetAnalyticCausalStats1075();
    {
        FScopedIntCVar Off(TEXT("p.IAmSpeed.Parallel.AnalyticCausal1075"),0);
        FScopedAnalyticCausalObservation1075 Scope(EAnalyticCausalKind1075::ExtrudedProvider);
        TestFalse(TEXT("disabled observer does not admit storage/timers"),Scope.IsEnabled());
        TestTrue(TEXT("disabled numeric target is null"),Scope.NumericTarget(0)==nullptr);
    }
    TestEqual(TEXT("disabled observer leaves counts untouched"),SnapshotAnalyticCausalStats1075().Kinds[int32(EAnalyticCausalKind1075::ExtrudedProvider)].Seen,uint64(0));
    ResetAnalyticCausalStats1075();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedExecutorObservationAndNested1075,
    "IAmSpeed.Parallel.Analytic1075.ExecutorObservationAndNested",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedExecutorObservationAndNested1075::RunTest(const FString& Parameters)
{
    using namespace Speed;
    std::atomic<int32> Visits{0};
    FFrameDispatchObservation1075 Outer, Inner[2];
    DispatchPrivateFrameParallelLanes(2,[&](int32 Lane)
    {
        DispatchPrivateFrameParallelLanes(2,[&](int32)
        { Visits.fetch_add(1,std::memory_order_relaxed); },&Inner[Lane]);
    },&Outer);
    TestEqual(TEXT("every nested callback executes exactly once"),Visits.load(),4);
    for (const auto& Probe : Inner)
        TestTrue(TEXT("nested dispatch fallback is distinguished"),Probe.Fallback==EFrameDispatchFallback1075::Nested);
    TestTrue(TEXT("outer execution is admitted to the owned pool"),Outer.Fallback==EFrameDispatchFallback1075::None);
    return true;
}
#endif
