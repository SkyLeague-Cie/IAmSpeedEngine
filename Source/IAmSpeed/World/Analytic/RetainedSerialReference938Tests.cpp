#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "AnalyticWorldQuery.h"
#include "HAL/IConsoleManager.h"
#include "Misc/AutomationTest.h"
#include <limits>
namespace Speed::Analytic::Retained938Tests
{
static const TCHAR* Difference(const FWorldHit& A,const FWorldHit& B)
{
    const auto Bits=[](const auto& X,const auto& Y){return FMemory::Memcmp(&X,&Y,sizeof(X))==0;};
#define SCALAR(N) if(A.N!=B.N)return TEXT(#N)
#define REAL(N) if(!Bits(A.N,B.N))return TEXT(#N)
#define VECTOR(N) REAL(N.X);REAL(N.Y);REAL(N.Z)
    SCALAR(bHit);SCALAR(bStartPenetrating);REAL(Time);REAL(PenetrationDepth);
    REAL(GeometricErrorBoundCm);REAL(AdditionalResidualAgreementAllowanceCm);SCALAR(bSurfaceNormalMayVary);
    VECTOR(Location);VECTOR(Point);VECTOR(QueryPoint);VECTOR(Normal);
    SCALAR(QueryFeatureKind);SCALAR(SurfaceFeatureKind);SCALAR(QueryFeatureIndex);SCALAR(SurfaceFeatureIndex);
    SCALAR(SourceId);SCALAR(SurfaceId);SCALAR(FeatureId);SCALAR(PrimitiveId);SCALAR(CanonicalGroupId);SCALAR(MaterialId);
#undef SCALAR
#undef REAL
#undef VECTOR
    return nullptr;
}
static bool Fixture(FAnalyticWorldData& Data,int32 Kind,FString& Error)
{
    FExtrudedQuinticPatch P;P.SourceId=30;P.SurfaceId=31;P.FeatureId=32;P.PrimitiveId=33;
    const double Bend=Kind==1?3.0:Kind==2?-3.0:0.0;
    for(int32 I=0;I<6;++I)P.SectionControlPoints[I]=FVector3d(-50+20*I,0,Bend*I*(5-I));
    P.ExtrusionAxis=Kind==3?FVector3d(.6,.8,0):FVector3d::RightVector;
    P.MinimumExtrusionCoordinate=-25;P.MaximumExtrusionCoordinate=25;
    P.bQueryCollisionEnabled=true;P.ObjectType=1;P.BlockingChannels=MAX_uint64;
    P.Bounds=FBox3d(EForceInit::ForceInit);
    for(const auto& C:P.SectionControlPoints){P.Bounds+=C-25*P.ExtrusionAxis;P.Bounds+=C+25*P.ExtrusionAxis;}
    Data.ExtrudedQuinticPatches.Add(P);
    if(!Data.FinalizeAndValidate(&Error))return false;
    Data.ExtrudedQuinticPatches[0].bAuthorityEligible=true;
    return true;
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedRetainedSerialReference938Test,
    "IAmSpeed.Parallel.ExtrudedFacetBatch.RetainedSerialReference938",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedRetainedSerialReference938Test::RunTest(const FString& Parameters)
{
    using namespace Speed::Analytic;
    using namespace Speed::Analytic::Retained938Tests;
    for(const TCHAR* Name:{TEXT("p.IAmSpeed.AnalyticWorld.CaseSatDiagnostics"),TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"),
        TEXT("p.IAmSpeed.AnalyticWorld.CaseCpuPhase400"),TEXT("p.IAmSpeed.AnalyticWorld.ExactCrossCostProbe844"),
        TEXT("p.IAmSpeed.AnalyticWorld.ReplayDecomposition925"),TEXT("p.IAmSpeed.AnalyticWorld.ExactCrossNormalCache784"),
        TEXT("p.IAmSpeed.Simulation.ActorProfile"),TEXT("p.IAmSpeed.AnalyticWorld.CacheProfile"),
        TEXT("p.IAmSpeed.AnalyticWorld.ExtrudedBoxPrimarySatScopeCache")})
        if(const IConsoleVariable* V=IConsoleManager::Get().FindConsoleVariable(Name))
            if(V->GetInt()!=0){AddError(FString::Printf(TEXT("retained parallel coverage requires %s=0"),Name));return false;}
    uint64 Compared=0,Parallel=0;
    for(int32 Kind:{0,1,2,3})
    {
        FAnalyticWorldData Data;FString Error;
        if(!TestTrue(TEXT("retained reference fixture validates"),Fixture(Data,Kind,Error))){AddError(Error);return false;}
        const FExtrudedQuinticPatch& Patch=Data.ExtrudedQuinticPatches[0];
        for(int32 Case=0;Case<64;++Case)
        for(bool Deepest:{false,true})
        {
            FWorldQuery Q;Q.Shape=EQueryShape(Case%3);Q.Radius=2;Q.HalfExtent=FVector3d(6,1.5,2);
            Q.Start=FVector3d(-48+2*(Case%24),-20+2*(Case%20),60);
            Q.End=FVector3d(48-2*(Case%24),20-2*(Case%20),-60);
            Q.bAuthorityOnly=true;Q.bIncludeCompactPatches=true;Q.bUseFiniteContactDomain=true;
            Q.DomainTolerance=0;Q.InitialOverlapTolerance=0;
            if(Case==0){Q.Shape=EQueryShape::Box;Q.HalfExtent=FVector3d(60,35,40);Q.Start=FVector3d(0,0,20);Q.End=FVector3d(0,0,-20);}
            if(Case%7==0&&Case!=0)Q.Rotation=FRotator3d(13,27,11).Quaternion();
            if(Case%11==0&&Case!=0)Q.Rotation=FQuat4d(FVector3d::UpVector,1.e-10);
            if(Case%13==0&&Case!=0)Q.RequiredSourceId=999;
            if(Case==49){Q.Start.X=Q.End.X=-0.0;Q.HalfExtent.X=0.0;}
            if(Case==50){Q.Start.X=Q.End.X=0.0;Q.HalfExtent.X=-0.0;}
            if(Case==51)Q.Start.X=Q.End.X=1.e100;
            if(Case==52)Q.Start.X=Q.End.X=-1.e100;
            if(Case==53)Q.DomainTolerance=std::numeric_limits<double>::denorm_min();
            if(Case==54)Q.InitialOverlapTolerance=std::numeric_limits<double>::min();
            if(Case==55)Q.Rotation=FQuat4d(FVector3d::UpVector,2.e-11);
            if(Case==56)Q.Rotation=FQuat4d(FVector3d::UpVector,1.e-10);
            if(Deepest)Q.End=Q.Start;
            const FWorldHit Expected=FWorldQueryService::TestRetainedExtrudedSweep938(Q,Patch,true,1,Deepest);
            for(int32 Lanes:{1,2,4,8,12})
            {
                Speed::FIndependentQueryBatchStats Stats;
                const FWorldHit Actual=FWorldQueryService::TestRetainedExtrudedSweep938(Q,Patch,false,Lanes,Deepest,&Stats);
                ++Compared;if(Stats.DispatchedLogicalLanes>1)++Parallel;
                if(const TCHAR* Field=Difference(Expected,Actual))
                {AddError(FString::Printf(TEXT("first retained938 divergence field=%s kind=%d case=%d deepest=%d lanes=%d"),Field,Kind,Case,Deepest?1:0,Lanes));return false;}
                if(Case==0&&(Kind==1||Kind==2)&&Lanes>1)
                {
                    TestEqual(TEXT("retained oracle reaches real requested logical lanes"),Stats.DispatchedLogicalLanes,Lanes);
                    TestTrue(TEXT("actual native participation separately witnessed"),Stats.DistinctNativeThreads>=1&&Stats.DistinctNativeThreads<=Lanes);
                }
            }
        }
    }
    TestTrue(TEXT("retained reference compared real parallel kernels"),Parallel>0);
    AddInfo(FString::Printf(TEXT("[IAmSpeedRetainedSerialReference938] Compared=%llu ParallelProviders=%llu AllFieldsExact=1 Scope=ProviderOnly FPEnvironment=ObservedDefault"),Compared,Parallel));
    return true;
}
#endif
