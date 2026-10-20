#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "StaticCollisionWorld.h"
#include "IAmSpeed/World/Simulation/FrameParallelSettings.h"
#include "HAL/IConsoleManager.h"
#include "Misc/AutomationTest.h"
namespace Speed::StaticQueryDiagnosticsTests
{
struct FConsoleLease
{
    IConsoleVariable* Variable;
    int32 Value;
    uint32 Flags;
    explicit FConsoleLease(IConsoleVariable* V):Variable(V),Value(V->GetInt()),Flags(V->GetFlags()){}
    bool Set(int32 V)
    {Variable->ClearFlags(ECVF_SetByMask);Variable->Set(V,ECVF_SetByCode);return Variable->GetInt()==V;}
    ~FConsoleLease()
    {
        Variable->ClearFlags(ECVF_SetByMask);Variable->Set(Value,ECVF_SetByCode);
        Variable->ClearFlags(static_cast<EConsoleVariableFlags>(MAX_uint32));
        Variable->SetFlags(static_cast<EConsoleVariableFlags>(Flags));
    }
};
// Preserve exact textual values: tolerance controls are floats and must never
// be truncated by the integer diagnostic lease used above.
struct FConfigurationConsoleLease
{
    IConsoleVariable* Variable;
    FString Value;
    uint32 Flags;
    explicit FConfigurationConsoleLease(IConsoleVariable* V)
        : Variable(V), Value(V->GetString()), Flags(V->GetFlags()) {}
    bool SetOne()
    {
        Variable->ClearFlags(ECVF_SetByMask); Variable->Set(TEXT("1"), ECVF_SetByCode);
        return Variable->GetFloat() == 1.0f;
    }
    ~FConfigurationConsoleLease()
    {
        Variable->ClearFlags(ECVF_SetByMask); Variable->Set(*Value, ECVF_SetByCode);
        Variable->ClearFlags(static_cast<EConsoleVariableFlags>(MAX_uint32));
        Variable->SetFlags(static_cast<EConsoleVariableFlags>(Flags));
    }
};
static bool SameHit(const Analytic::FWorldHit& A,const Analytic::FWorldHit& B)
{
    const auto Bits=[](const auto& X,const auto& Y){return FMemory::Memcmp(&X,&Y,sizeof(X))==0;};
    const auto Vec=[&](const FVector3d& X,const FVector3d& Y){return Bits(X.X,Y.X)&&Bits(X.Y,Y.Y)&&Bits(X.Z,Y.Z);};
    return A.bHit==B.bHit&&A.bStartPenetrating==B.bStartPenetrating&&Bits(A.Time,B.Time)&&
        Bits(A.PenetrationDepth,B.PenetrationDepth)&&Bits(A.GeometricErrorBoundCm,B.GeometricErrorBoundCm)&&
        Bits(A.AdditionalResidualAgreementAllowanceCm,B.AdditionalResidualAgreementAllowanceCm)&&
        A.bSurfaceNormalMayVary==B.bSurfaceNormalMayVary&&Vec(A.Location,B.Location)&&Vec(A.Point,B.Point)&&
        Vec(A.QueryPoint,B.QueryPoint)&&Vec(A.Normal,B.Normal)&&A.QueryFeatureKind==B.QueryFeatureKind&&
        A.SurfaceFeatureKind==B.SurfaceFeatureKind&&A.QueryFeatureIndex==B.QueryFeatureIndex&&
        A.SurfaceFeatureIndex==B.SurfaceFeatureIndex&&A.SourceId==B.SourceId&&A.SurfaceId==B.SurfaceId&&
        A.FeatureId==B.FeatureId&&A.PrimitiveId==B.PrimitiveId&&A.CanonicalGroupId==B.CanonicalGroupId&&A.MaterialId==B.MaterialId;
}
static bool Fixture(Analytic::FAnalyticWorldData& Data,TArray<Analytic::FWorldQuery>& Queries)
{
    Analytic::FBoundedPlane Plane;Plane.SourceId=1;Plane.SurfaceId=2;Plane.FeatureId=3;Plane.PrimitiveId=4;
    Plane.HalfExtents=FVector2d(50,50);Plane.BlockingChannels=MAX_uint64;Data.Planes.Add(Plane);
    if(!Data.FinalizeAndValidate())return false;
    Data.Planes[0].bAuthorityEligible=true;
    for(int32 I=0;I<8;++I)
    {
        Analytic::FWorldQuery Q;Q.Start=FVector3d(I,0,5);Q.End=FVector3d(I,0,-5);
        Q.bAuthorityOnly=true;Queries.Add(Q);
    }
    return true;
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedStaticQueryDiagnosticsFallbackTest,
    "IAmSpeed.Parallel.StaticQueryBatch.DiagnosticsFallback",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedStaticQueryDiagnosticsFallbackTest::RunTest(const FString& Parameters)
{
    using namespace Speed;
    using namespace StaticQueryDiagnosticsTests;
    if(!IsInGameThread()){AddError(TEXT("console lease requires the controlled test thread"));return false;}
    TArray<TUniquePtr<FConsoleLease>> Leases;
    for(const TCHAR* Name:{TEXT("p.IAmSpeed.AnalyticWorld.CaseSatDiagnostics"),TEXT("p.IAmSpeed.AnalyticWorld.CaseCpuPhase400"),
        TEXT("p.IAmSpeed.Simulation.ActorProfile"),TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"),
        TEXT("p.IAmSpeed.AnalyticWorld.ExactCrossCostProbe844"),TEXT("p.IAmSpeed.AnalyticWorld.ReplayDecomposition925"),
        TEXT("p.IAmSpeed.AnalyticWorld.CacheProfile"),TEXT("p.IAmSpeed.StaticWorldQuery.Audit"),
        TEXT("p.IAmSpeed.AnalyticWorld.ExactCrossNormalCache784"),TEXT("p.IAmSpeed.AnalyticWorld.Shadow"),
        TEXT("p.IAmSpeed.AnalyticWorld.MeshShadow"),TEXT("p.IAmSpeed.AnalyticWorld.CompactShadow"),
        TEXT("p.IAmSpeed.AnalyticWorld.AuthorityChaosShadow"),
        TEXT("p.IAmSpeed.AnalyticWorld.ResidualSelectionAudit"),TEXT("p.IAmSpeed.AnalyticWorld.StrictQueryDetail"),
        TEXT("p.IAmSpeed.AnalyticWorld.ExtrudedBoxPrimarySatScopeCache")})
        if(IConsoleVariable* Var=IConsoleManager::Get().FindConsoleVariable(Name))
        {auto& Lease=Leases.Add_GetRef(MakeUnique<FConsoleLease>(Var));if(!Lease->Set(0))return false;}
    Analytic::FAnalyticWorldData Data;TArray<Analytic::FWorldQuery> Queries;
    if(!TestTrue(TEXT("diagnostic fixture validates"),Fixture(Data,Queries)))return false;
    FAnalyticStaticCollisionWorld World(Data,4,4),Serial(Data);
    TArray<Analytic::FWorldHit> Actual,Expected;Actual.SetNum(Queries.Num());Expected.SetNum(Queries.Num());
    Serial.SweepIndependentBatch(Queries,Expected);
    for(const auto& Lease:Leases)
    {
        if(!TestTrue(TEXT("instrumentation control set"),Lease->Set(1)))return false;
        FIndependentQueryBatchStats Stats;
        if(!TestTrue(TEXT("fallback joins"),World.SweepIndependentBatch(Queries,Actual,&Stats)))return false;
        TestTrue(TEXT("thread-owned instrumentation is explicitly serial"),Stats.Fallback==EIndependentQueryFallback::ThreadOwnedDiagnostics);
        TestEqual(TEXT("requested remains4"),Stats.RequestedLanes,4);
        TestEqual(TEXT("actual logical lane1"),Stats.DispatchedLogicalLanes,1);
        TestEqual(TEXT("actual native thread1"),Stats.DistinctNativeThreads,1);
        for(int32 I=0;I<Queries.Num();++I)
            if(!SameHit(Expected[I],Actual[I])){AddError(TEXT("diagnostic fallback changed hit bits"));return false;}
        if(!Lease->Set(0))return false;
    }
    for (const TCHAR* Name : {TEXT("p.IAmSpeed.AnalyticWorld.CompactShadow.Detail"),
        TEXT("p.IAmSpeed.AnalyticWorld.Shadow.PointToleranceCm"),
        TEXT("p.IAmSpeed.AnalyticWorld.Shadow.PenetrationToleranceCm")})
    {
        IConsoleVariable* Variable = IConsoleManager::Get().FindConsoleVariable(Name);
        if (!Variable) continue;
        FConfigurationConsoleLease Configuration(Variable);
        if (!TestTrue(TEXT("configuration fixture admitted"), Configuration.SetOne())) return false;
        FIndependentQueryBatchStats Stats;
        if (!World.SweepIndependentBatch(Queries, Actual, &Stats)) return false;
        TestTrue(TEXT("inactive shadow options do not disable dispatch"), Stats.Fallback == EIndependentQueryFallback::None);
        TestEqual(TEXT("configuration-only control keeps4 logical lanes"), Stats.DispatchedLogicalLanes, 4);
        for (int32 I=0; I<Queries.Num(); ++I)
            if (!SameHit(Expected[I],Actual[I])) { AddError(TEXT("configuration-only control changed hit bits")); return false; }
    }
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedStaticQueryNestedGuardTest,
    "IAmSpeed.Parallel.StaticQueryBatch.NestedGuard",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedStaticQueryNestedGuardTest::RunTest(const FString& Parameters)
{
    using namespace Speed;
    using namespace StaticQueryDiagnosticsTests;
    Analytic::FAnalyticWorldData Data;TArray<Analytic::FWorldQuery> Queries;
    if(!Fixture(Data,Queries))return false;
    FAnalyticStaticCollisionWorld World(Data,4,4),Serial(Data);
    TArray<Analytic::FWorldHit> Actual,Expected;Actual.SetNum(Queries.Num());Expected.SetNum(Queries.Num());
    Serial.SweepIndependentBatch(Queries,Expected);
    FIndependentQueryBatchStats Stats;
    {
        FScopedFrameParallelTask Task;
        if(!World.SweepIndependentBatch(Queries,Actual,&Stats))return false;
    }
    TestTrue(TEXT("nested query batch explicitly serial"),Stats.Fallback==EIndependentQueryFallback::NestedTask);
    TestEqual(TEXT("no nested worker budget expansion"),Stats.DispatchedLogicalLanes,1);
    for(int32 I=0;I<Queries.Num();++I)
        if(!SameHit(Expected[I],Actual[I])){AddError(TEXT("nested guard changed hit bits"));return false;}
    TestFalse(TEXT("task context restored"),IsInsideFrameParallelTask());
    return true;
}
#endif
