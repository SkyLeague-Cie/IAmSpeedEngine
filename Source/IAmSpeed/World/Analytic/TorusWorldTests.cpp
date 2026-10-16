#include "TorusWorldQuery.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "StaticWorldQueryAudit.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTorusWorldProviderTest,"IAmSpeed.Collision.Torus.WorldProvider",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FTorusWorldProviderTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;
    FTorusSurface Torus;Torus.SourceId=Torus.SurfaceId=Torus.FeatureId=Torus.PrimitiveId=17;
    Torus.Geometry={100.,10.};FString Error;
    TestTrue(TEXT("Bounded immutable torus admitted"),Torus.Finalize(&Error));
    FAnalyticWorldData World;World.Tori.Add(Torus);
    TestTrue(TEXT("Torus-only world finalizes"),World.FinalizeAndValidate(&Error));
    TestTrue(TEXT("Torus-only strict authority available"),World.IsAuthorityEligible());
    FWorldQueryService Service(World);
    FWorldQuery Query;Query.Shape=EQueryShape::Sphere;Query.Radius=5.;Query.bAuthorityOnly=true;
    Query.Start=FVector(-30.,100.,0.);Query.End=FVector(30.,100.,0.);
    auto Hit=Service.Sweep(Query);
    TestTrue(TEXT("World sphere hits tube"),Hit.bHit && FMath::IsNearlyEqual(Hit.Time,.25,1.e-6));
    TestEqual(TEXT("Stable authored provider retained"),Hit.SourceId,uint64(17));
    TestTrue(TEXT("Curved provider requires normal reacquisition"),Hit.bSurfaceNormalMayVary);
    Query.Shape=EQueryShape::Box;Query.HalfExtent=FVector(5.);
    Hit=Service.Sweep(Query);
    TestTrue(TEXT("World OBB hits true tube"),Hit.bHit && FMath::Abs(Hit.Time-.25)*60.<=2.e-4);
    Query.Start=FVector(-50.,0.,0.);Query.End=FVector(50.,0.,0.);
    TestFalse(TEXT("World box sees empty ring hole"),Service.Sweep(Query).bHit);
    auto Overlap=Query;Overlap.Start=Overlap.End=FVector(-14.995,100.,0.);
    TestTrue(TEXT("Initial OBB pose is supported"),Service.TryFindDeepestOverlap(Overlap,Hit));
    TestTrue(TEXT("Quantization-sized penetration has a certified bounded MTD"),Hit.bHit && Hit.bStartPenetrating &&
        Hit.PenetrationDepth>=.005-1.e-6 && Hit.PenetrationDepth<=.005+2.01e-4);
    TestTrue(TEXT("Penetration witnesses encode the certified correction"),
        (Hit.Point-Hit.QueryPoint).Equals(Hit.Normal*Hit.PenetrationDepth,1.e-6));
    Query.Shape=EQueryShape::Ray;Query.Start=FVector(-30.,100.,0.);Query.End=FVector(30.,100.,0.);
    Hit=Service.Sweep(Query);TestTrue(TEXT("Wheel ray sees curved ring"),Hit.bHit && FMath::IsNearlyEqual(Hit.Time,1./3.,1.e-6));
    Query.RequiredSourceId=18;TestFalse(TEXT("Provider restriction filters torus"),Service.Sweep(Query).bHit);
    Query.RequiredSourceId=0;Query.bApplyCollisionFilter=true;Query.BlockingObjectTypes=0;
    TestFalse(TEXT("Collision response filter preserved"),Service.Sweep(Query).bHit);
    Query.bApplyCollisionFilter=false;
    FTorusSweepTrajectory WheelMotion;
    WheelMotion.DeltaSeconds=1.f;WheelMotion.Owner.AngularVelocity=FVector(0.,0.,PI*.5);
    WheelMotion.LocalOffset=FVector(110.,0.,0.);
    Query.Shape=EQueryShape::Sphere;Query.Radius=5.;
    Query.Start=WheelMotion.PoseAt(0.).GetTranslation();Query.End=WheelMotion.PoseAt(1.).GetTranslation();
    auto Arc=[&](double T){return WheelMotion.PoseAt(T);};
    Hit=SweepTorusTrajectory(Query,Torus,Arc,110.*PI*.5);
    TestTrue(TEXT("Captured owner rotation carries wheel into true tube"),Hit.bHit && Hit.Time>0. && Hit.Time<1.);
    Torus.Pose=FTransform(FQuat(FVector::UpVector,PI*.5),FVector(200.,300.,400.));
    TestTrue(TEXT("Rotated provider admitted"),Torus.Finalize(&Error));
    Query.Shape=EQueryShape::Sphere;Query.Radius=5.;
    Query.Start=Torus.Pose.TransformPosition(FVector(-30.,100.,0.));
    Query.End=Torus.Pose.TransformPosition(FVector(30.,100.,0.));
    Hit=SweepTorusSurface(Query,Torus);
    TestTrue(TEXT("Rotated torus preserves analytical TOI"),Hit.bHit && FMath::IsNearlyEqual(Hit.Time,.25,1.e-6));
    TestTrue(TEXT("Normal transformed to world"),Hit.Normal.Equals(Torus.Pose.TransformVectorNoScale(FVector(-1.,0.,0.)),1.e-6));
    auto Other=Torus;Other.PrimitiveId=Other.SourceId=Other.SurfaceId=Other.FeatureId=18;
    FAnalyticWorldData A,B;A.Tori={Torus,Other};B.Tori={Other,Torus};
    TestTrue(TEXT("First ordering finalizes"),A.FinalizeAndValidate(&Error));
    TestTrue(TEXT("Second ordering finalizes"),B.FinalizeAndValidate(&Error));
    TestEqual(TEXT("Authoring order cannot change canonical hash"),A.StableHash(),B.StableHash());
    Other.Geometry.TubeRadius+=1.;B.Tori={Torus,Other};B.FinalizeAndValidate(&Error);
    TestNotEqual(TEXT("Physical radius changes canonical hash"),A.StableHash(),B.StableHash());
    B.Tori={Torus,Torus};TestFalse(TEXT("Duplicate primitive identity rejected"),B.FinalizeAndValidate(&Error));
    return !HasAnyErrors();
}
#endif
