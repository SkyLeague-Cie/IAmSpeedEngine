#if WITH_DEV_AUTOMATION_TESTS
#include "AnalyticWorldQuery.h"
#include "IAmSpeed/World/Simulation/FrameParallelSettings.h"
#include "HAL/IConsoleManager.h"
#include "Misc/AutomationTest.h"
#include <limits>
namespace Speed::Analytic::FacetParallelTests
{
static bool SameHit(const FWorldHit& A, const FWorldHit& B)
{
    const auto Bits=[](const auto& X,const auto& Y) { return FMemory::Memcmp(&X,&Y,sizeof(X))==0; };
    const auto Vec=[&](const FVector3d& X,const FVector3d& Y) { return Bits(X.X,Y.X)&&Bits(X.Y,Y.Y)&&Bits(X.Z,Y.Z); };
    return A.bHit==B.bHit && A.bStartPenetrating==B.bStartPenetrating &&
        Bits(A.Time,B.Time)&&Bits(A.PenetrationDepth,B.PenetrationDepth)&&
        Bits(A.GeometricErrorBoundCm,B.GeometricErrorBoundCm)&&
        Bits(A.AdditionalResidualAgreementAllowanceCm,B.AdditionalResidualAgreementAllowanceCm)&&
        A.bSurfaceNormalMayVary==B.bSurfaceNormalMayVary&&Vec(A.Location,B.Location)&&
        Vec(A.Point,B.Point)&&Vec(A.QueryPoint,B.QueryPoint)&&Vec(A.Normal,B.Normal)&&
        A.QueryFeatureKind==B.QueryFeatureKind&&A.SurfaceFeatureKind==B.SurfaceFeatureKind&&
        A.QueryFeatureIndex==B.QueryFeatureIndex&&A.SurfaceFeatureIndex==B.SurfaceFeatureIndex&&
        A.SourceId==B.SourceId&&A.SurfaceId==B.SurfaceId&&A.FeatureId==B.FeatureId&&
        A.PrimitiveId==B.PrimitiveId&&A.CanonicalGroupId==B.CanonicalGroupId&&A.MaterialId==B.MaterialId;
}
static bool BuildWorld(FAnalyticWorldData& Data, int32 Kind, FString& Reason)
{
    FExtrudedQuinticPatch Patch;
    Patch.SourceId=30;Patch.SurfaceId=31;Patch.FeatureId=32;Patch.PrimitiveId=33;
    const double Bend=Kind==0?0.0:Kind==1?3.0:-3.0;
    for (int32 I=0;I<6;++I) Patch.SectionControlPoints[I]=FVector3d(-50+20*I,0,Bend*I*(5-I));
    Patch.ExtrusionAxis=FVector3d::RightVector;
    Patch.MinimumExtrusionCoordinate=-25;Patch.MaximumExtrusionCoordinate=25;
    Patch.bQueryCollisionEnabled=true;Patch.ObjectType=1;Patch.BlockingChannels=MAX_uint64;
    for (const FVector3d& C:Patch.SectionControlPoints)
    { Patch.Bounds+=C-25*Patch.ExtrusionAxis;Patch.Bounds+=C+25*Patch.ExtrusionAxis; }
    Data.ExtrudedQuinticPatches.Add(Patch);
    if (!Data.FinalizeAndValidate(&Reason))return false;
    Data.ExtrudedQuinticPatches[0].bAuthorityEligible=true;
    return true;
}
static FWorldQuery WideQuery()
{
    FWorldQuery Q;Q.Shape=EQueryShape::Box;Q.bAuthorityOnly=true;Q.bIncludeCompactPatches=true;
    Q.bUseFiniteContactDomain=true;Q.HalfExtent=FVector3d(60,35,40);
    Q.Start=FVector3d(0,0,20);Q.End=FVector3d(0,0,-20);
    Q.DomainTolerance=0;Q.InitialOverlapTolerance=0;
    return Q;
}
static bool DiagnosticsDisabled(FAutomationTestBase& Test)
{
    for (const TCHAR* Name:{TEXT("p.IAmSpeed.AnalyticWorld.CaseSatDiagnostics"),
        TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"),TEXT("p.IAmSpeed.Simulation.ActorProfile"),
        TEXT("p.IAmSpeed.AnalyticWorld.CacheProfile"),TEXT("p.IAmSpeed.AnalyticWorld.ExactCrossCostProbe844"),
        TEXT("p.IAmSpeed.AnalyticWorld.ReplayDecomposition925"),TEXT("p.IAmSpeed.StaticWorldQuery.Audit"),
        TEXT("p.IAmSpeed.AnalyticWorld.ExactCrossNormalCache784"),TEXT("p.IAmSpeed.AnalyticWorld.ExtrudedBoxPrimarySatScopeCache")})
        if (const IConsoleVariable* V=IConsoleManager::Get().FindConsoleVariable(Name))
            if(V->GetInt()!=0) {Test.AddError(FString::Printf(TEXT("parallel coverage requires %s=0"),Name));return false;}
    return true;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedExtrudedFacetBatchBitsTest,
    "IAmSpeed.Parallel.ExtrudedFacetBatch.AllHitFields",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedExtrudedFacetBatchBitsTest::RunTest(const FString& Parameters)
{
    using namespace Speed::Analytic;
    using namespace Speed::Analytic::FacetParallelTests;
    if(!DiagnosticsDisabled(*this))return false;
    uint64 Compared=0,TotalParallelProviders=0;
    for(int32 Kind:{0,1,2})
    {
        FAnalyticWorldData Data;FString Reason;
        if(!TestTrue(TEXT("provider fixture validates"),BuildWorld(Data,Kind,Reason))) {AddError(Reason);return false;}
        if(Kind!=0&&!TestTrue(TEXT("curved fixture contains multiple ordered facets"),Data.ExtrudedQuinticPatches[0].SectionPolyline.Num()>4))return false;
        for(int32 Lanes:{1,2,4,8,12})
        {
            FWorldQueryService Serial(Data,1,2),Parallel(Data,Lanes,2);
            for(int32 Case=0;Case<57;++Case)
            {
                FWorldQuery Q=WideQuery();
                if(Case>0)
                {
                    Q.Shape=EQueryShape(Case%3);Q.Radius=2;Q.HalfExtent=FVector3d(6,1.5,2);
                    Q.Start=FVector3d(-48+2*(Case%24),-20+2*(Case%20),60);
                    Q.End=FVector3d(48-2*(Case%24),20-2*(Case%20),-60);
                    if(Case%5==0)Q.Start=Q.End=FVector3d(0,0,1);
                    if(Case%7==0)Q.Rotation=FRotator3d(13,27,11).Quaternion();
                    if(Case%11==0)Q.Rotation=FQuat4d(FVector3d::UpVector,1.e-10);
                    if(Case%13==0)Q.RequiredSourceId=999;
                }
                if(Case==49){Q.Start.X=Q.End.X=-0.0;Q.HalfExtent.X=0.0;}
                if(Case==50){Q.Start.X=Q.End.X=0.0;Q.HalfExtent.X=-0.0;}
                if(Case==51){Q.Start.X=Q.End.X=1.e100;}
                if(Case==52){Q.Start.X=Q.End.X=-1.e100;}
                if(Case==53){Q.DomainTolerance=std::numeric_limits<double>::denorm_min();}
                if(Case==54){Q.InitialOverlapTolerance=std::numeric_limits<double>::min();}
                if(Case==55){Q.Rotation=FQuat4d(FVector3d::UpVector,2.e-11);}
                if(Case==56){Q.Rotation=FQuat4d(FVector3d::UpVector,1.e-10);}
                for(int32 Warm=0;Warm<2;++Warm)
                {
                    const FWorldHit Expected=Serial.Sweep(Q),Actual=Parallel.Sweep(Q);++Compared;
                    if(!SameHit(Expected,Actual))
                    {AddError(FString::Printf(TEXT("first divergence kind=%d lanes=%d case=%d warm=%d"),Kind,Lanes,Case,Warm));return false;}
                }
            }
            const FFacetParallelTotals Totals=Parallel.GetFacetParallelTotals();
            if(Lanes==1)TestEqual(TEXT("serial reference never dispatches parallel"),Totals.ParallelProviders,uint64(0));
            if(Kind!=0&&Lanes>1)
            {
                TestTrue(TEXT("real extruded kernel parallel branch was reached"),Totals.ParallelProviders>0);
                TestEqual(TEXT("requested logical lanes reached"),Totals.MaximumLogicalLanes,Lanes);
                TestTrue(TEXT("actual native thread participation recorded independently"),Totals.MaximumNativeThreads>=1&&Totals.MaximumNativeThreads<=Lanes);
                TotalParallelProviders+=Totals.ParallelProviders;
            }
            AddInfo(FString::Printf(TEXT("[IAmSpeedFacetParallelUnit] Kind=%d Requested=%d Providers=%llu ParallelProviders=%llu Facets=%llu MaxLogical=%d MaxNative=%d"),
                Kind,Lanes,Totals.Providers,Totals.ParallelProviders,Totals.EvaluatedFacets,Totals.MaximumLogicalLanes,Totals.MaximumNativeThreads));
        }
    }
    TestTrue(TEXT("parallel oracle includes positive native dispatch"),TotalParallelProviders>0);
    AddInfo(FString::Printf(TEXT("[IAmSpeedFacetParallelOracle] Compared=%llu ParallelProviders=%llu"),Compared,TotalParallelProviders));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedExtrudedFacetNestedGuardTest,
    "IAmSpeed.Parallel.ExtrudedFacetBatch.NestedGuard",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedExtrudedFacetNestedGuardTest::RunTest(const FString& Parameters)
{
    using namespace Speed::Analytic;
    using namespace Speed::Analytic::FacetParallelTests;
    if(!DiagnosticsDisabled(*this))return false;
    FAnalyticWorldData Data;FString Reason;
    if(!BuildWorld(Data,1,Reason)){AddError(Reason);return false;}
    FWorldQueryService Serial(Data,1,2),Nested(Data,4,2);
    const FWorldQuery Q=WideQuery();const FWorldHit Expected=Serial.Sweep(Q);
    {
        Speed::FScopedFrameParallelTask Scope;
        if(!TestTrue(TEXT("nested kernel preserves every hit bit"),SameHit(Expected,Nested.Sweep(Q))))return false;
    }
    const FFacetParallelTotals Totals=Nested.GetFacetParallelTotals();
    TestTrue(TEXT("nested task has explicit fallback evidence"),Totals.NestedFallbackProviders>0);
    TestEqual(TEXT("no nested pool dispatch"),Totals.ParallelProviders,uint64(0));
    TestFalse(TEXT("thread guard restored after join scope"),Speed::IsInsideFrameParallelTask());
    return true;
}
#endif
