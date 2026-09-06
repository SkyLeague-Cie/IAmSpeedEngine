#if WITH_DEV_AUTOMATION_TESTS
#include "AnalyticWorldQuery.h"
#include "IAmSpeed/World/Collision/StaticCollisionWorld.h"
#include "Misc/AutomationTest.h"

namespace DeepestOverlapTestPrivate
{
using namespace Speed::Analytic;

// RED187 used SweepSingle here and exposed the shallow-contact winner. The
// explicit operation must now pass without changing the CCD control assertions.
bool QueryOverlap(const Speed::IStaticCollisionWorld& World, const FWorldQuery& Q, FWorldHit& Hit)
{
    return World.TryFindDeepestOverlap(Q, Hit);
}

void AddPlane(FAnalyticWorldData& World, uint64 Id, double Height)
{
    FBoundedPlane P;
    P.SourceId=Id; P.SurfaceId=Id+10; P.FeatureId=Id+20; P.PrimitiveId=Id+30;
    P.Origin=FVector3d(0,0,Height); P.HalfExtents=FVector2d(50,50);
    P.ObjectType=static_cast<uint32>(Id);
    World.Planes.Add(P);
}

FWorldQuery SpherePose()
{
    FWorldQuery Q;
    Q.Shape=EQueryShape::Sphere; Q.Radius=5;
    Q.Start=Q.End=FVector3d(3,2,5);
    Q.bIncludeCompactPatches=true; Q.bAuthorityOnly=true;
    return Q;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedDeepestOverlapPlanesTest,
    "IAmSpeed.AnalyticWorld.DeepestOverlapPlanes",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedDeepestOverlapPlanesTest::RunTest(const FString& Parameters)
{
    using namespace DeepestOverlapTestPrivate;
    for (const bool Reverse : {false,true})
    {
        FAnalyticWorldData Data;
        AddPlane(Data,Reverse ? 2 : 1,Reverse ? 1 : 0);
        AddPlane(Data,Reverse ? 1 : 2,Reverse ? 0 : 1);
        FString Reason;
        if (!TestTrue(TEXT("exact two-plane fixture validates"),Data.FinalizeAndValidate(&Reason)))
        { AddError(Reason); return false; }
        for (auto& P : Data.Planes) P.bAuthorityEligible=true;
        const Speed::FAnalyticStaticCollisionWorld World(Data);
        for (const EQueryShape Shape : {EQueryShape::Sphere,EQueryShape::Box})
        for (const double Inflation : {0.,.015})
        {
            FWorldQuery Q=SpherePose(); Q.Shape=Shape;
            Q.Radius+=Inflation; Q.HalfExtent=FVector3d(2,3,5+Inflation);
            Q.InitialOverlapTolerance=0;
            const FWorldHit Before=World.SweepSingle(Q);
            TestTrue(TEXT("unchanged first sweep chooses the shallow floor"),
                Before.bHit && FMath::IsNearlyEqual(Before.PenetrationDepth,Inflation,1.e-9));
            for (int32 Repeat=0; Repeat<3; ++Repeat)
            {
                FWorldHit H;
                TestTrue(TEXT("stationary operation is available"),QueryOverlap(World,Q,H));
                TestTrue(TEXT("deepest overlap cannot be hidden by shallow floor"),
                    H.bHit && H.bStartPenetrating && FMath::IsNearlyEqual(H.PenetrationDepth,1+Inflation,1.e-9));
                TestEqual(TEXT("deepest provider identity"),H.SourceId,uint64(2));
                TestTrue(TEXT("overlap witnesses agree with depth"),
                    (H.Point-H.QueryPoint).Equals(H.PenetrationDepth*H.Normal,1.e-8));
                const FWorldHit After=World.SweepSingle(Q);
                TestTrue(TEXT("overlap cache cannot change ordinary CCD"),
                    Before.PrimitiveId==After.PrimitiveId && Before.PenetrationDepth==After.PenetrationDepth &&
                    Before.Normal==After.Normal && Before.Point==After.Point && Before.QueryPoint==After.QueryPoint);
            }
            Q.RequiredSourceId=1;
            Q.Radius=5; Q.HalfExtent.Z=5;
            FWorldHit H;
            TestTrue(TEXT("filtered pose query remains available"),QueryOverlap(World,Q,H));
            TestFalse(TEXT("pure tangency is not a positive overlap"),H.bHit);
            Q.End.Z+=1;
            TestFalse(TEXT("moving query cannot silently become pose certification"),QueryOverlap(World,Q,H));
            TestFalse(TEXT("refused query clears the previous result"),H.bHit);
        }
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedDeepestOverlapPiecewiseTest,
    "IAmSpeed.AnalyticWorld.DeepestOverlapPiecewise",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedDeepestOverlapPiecewiseTest::RunTest(const FString& Parameters)
{
    using namespace DeepestOverlapTestPrivate;
    for (const double Height : {1.,.02})
    {
        FAnalyticWorldData Data;
        FPiecewiseTensorBezierPatch P;
        P.SourceId=3; P.SurfaceId=13; P.PrimitiveId=33; P.CanonicalGroupId=43;
        for (int32 Side=0; Side<2; ++Side)
        {
            auto& C=P.Cells.AddDefaulted_GetRef();
            C.PrimitiveId=100+Side; C.FeatureId=200+Side;
            C.MinimumU=.5*Side; C.MaximumU=.5*(Side+1);
            // Degree two explicitly carries the second derivatives used by the
            // existing C2 adjacency certificate (a bilinear cell did not).
            C.Surface.DegreeU=C.Surface.DegreeV=2;
            for (int32 U=0; U<3; ++U)
            for (int32 V=0; V<3; ++V)
                C.Surface.ControlPoints.Add(FVector3d(-50+50*Side+25*U,-50+50*V,Height));
        }
        Data.PiecewiseTensorBezierPatches.Add(MoveTemp(P));
        FString Reason;
        if (!TestTrue(TEXT("exact connected piecewise plane validates"),Data.FinalizeAndValidate(&Reason)))
        { AddError(Reason); return false; }
        Data.PiecewiseTensorBezierPatches[0].bAuthorityEligible=true;
        TestEqual(TEXT("actual smooth join is recognized in both directions"),
            Data.PiecewiseTensorBezierPatches[0].Adjacencies.Num(),2);
        const Speed::FAnalyticStaticCollisionWorld World(Data);
        FWorldQuery Q=SpherePose();
        FWorldHit H;
        TestTrue(TEXT("piecewise overlap available"),QueryOverlap(World,Q,H));
        TestTrue(TEXT("internal tangent/shell cannot mask the positive face overlap"),
            H.bHit && FMath::IsNearlyEqual(H.PenetrationDepth,Height,1.e-9));
        Q.Start=Q.End=FVector3d(60,2,5);
        TestTrue(TEXT("finite miss query available"),QueryOverlap(World,Q,H));
        TestFalse(TEXT("finite plane must not extend outside its actual domain"),H.bHit);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedDeepestOverlapFiltersTest,
    "IAmSpeed.AnalyticWorld.DeepestOverlapFiltersAndCache",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedDeepestOverlapFiltersTest::RunTest(const FString& Parameters)
{
    using namespace DeepestOverlapTestPrivate;
    for (const bool Reverse : {false,true})
    {
        FAnalyticWorldData Data;
        AddPlane(Data,1,0);
        for (int32 I=0; I<2; ++I)
        {
            const uint64 Id=Reverse ? 3-I : 2+I;
            AddPlane(Data,Id,0);
            auto& Wall=Data.Planes.Last();
            Wall.Origin=FVector3d(4,0,5);
            Wall.Normal=FVector3d::ForwardVector;
            Wall.AxisU=FVector3d::RightVector; Wall.AxisV=FVector3d::UpVector;
            Wall.BlockingChannels=0; // Object eligibility is independent of trace channels.
        }
        FString Reason;
        if (!TestTrue(TEXT("finite floor/wall fixture validates"),Data.FinalizeAndValidate(&Reason)))
        { AddError(Reason); return false; }
        for (auto& P : Data.Planes) P.bAuthorityEligible=true;
        const Speed::FAnalyticStaticCollisionWorld World(Data);
        FWorldQueryService Service(Data);
        FWorldQuery Q=SpherePose(); Q.Start=Q.End=FVector3d(0,0,5);
        Q.bObjectQuery=Q.bApplyCollisionFilter=true;
        Q.ObjectTypes=(1ull<<1)|(1ull<<2)|(1ull<<3);
        Q.InitialOverlapTolerance=0;
        FWorldHit HybridBefore,HybridAfter,H;
        const bool CoveredBefore=Service.TrySweepAuthority(Q,HybridBefore);
        TestTrue(TEXT("orthogonal overlap available"),QueryOverlap(World,Q,H));
        TestTrue(TEXT("real wall wins over floor tangency with exact depth tie by ID"),
            H.bHit && H.SourceId==2 && H.PenetrationDepth==1 && H.Normal==FVector3d(-1,0,0));
        const FWorldHit Expected=H;
        for (int32 I=0; I<80; ++I)
        {
            auto Other=Q; Other.Start.Y=Other.End.Y=-40+I;
            QueryOverlap(World,Other,H);
        }
        QueryOverlap(World,Q,H);
        TestTrue(TEXT("cache eviction does not alter overlap"),
            H.PrimitiveId==Expected.PrimitiveId && H.PenetrationDepth==Expected.PenetrationDepth &&
            H.Normal==Expected.Normal && H.Point==Expected.Point && H.QueryPoint==Expected.QueryPoint);
        Service.TryFindDeepestOverlap(Q,H);
        const bool CoveredAfter=Service.TrySweepAuthority(Q,HybridAfter);
        TestTrue(TEXT("Hybrid coverage and winner unchanged across overlap operation"),
            CoveredBefore==CoveredAfter && HybridBefore.bHit==HybridAfter.bHit &&
            HybridBefore.PrimitiveId==HybridAfter.PrimitiveId && HybridBefore.Time==HybridAfter.Time &&
            HybridBefore.Point==HybridAfter.Point && HybridBefore.QueryPoint==HybridAfter.QueryPoint &&
            HybridBefore.PenetrationDepth==HybridAfter.PenetrationDepth && HybridBefore.Normal==HybridAfter.Normal);
        Q.RequiredSourceId=3; QueryOverlap(World,Q,H);
        TestEqual(TEXT("source restriction retained"),H.SourceId,uint64(3));
        Q.RequiredSourceId=0; Q.ObjectTypes=1ull<<1; QueryOverlap(World,Q,H);
        TestFalse(TEXT("excluded wall cannot enter the overlap set"),H.bHit);
        Q.ObjectTypes=0; QueryOverlap(World,Q,H);
        TestFalse(TEXT("empty object filter is a clear query"),H.bHit);
        Q.ObjectTypes=MAX_uint64; Q.ReferenceNormal=FVector3d::UpVector;
        Q.MinimumReferenceNormalDot=.9; QueryOverlap(World,Q,H);
        TestFalse(TEXT("reference-normal restriction retained"),H.bHit);
        Q.MinimumReferenceNormalDot=-1; Q.Start.Z=Q.End.Z=100;
        QueryOverlap(World,Q,H); TestFalse(TEXT("finite world miss"),H.bHit);
        Q.Radius=-1;
        TestFalse(TEXT("invalid radius is unavailable, not certified empty"),QueryOverlap(World,Q,H));
        TestFalse(TEXT("invalid shape clears output"),H.bHit);
        Q.Radius=5; Q.Shape=EQueryShape::Ray;
        TestFalse(TEXT("ray is not a volume overlap"),QueryOverlap(World,Q,H));
    }
    return true;
}
#endif
