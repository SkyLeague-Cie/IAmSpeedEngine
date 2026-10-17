#include "CylinderGeometry.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
using namespace Speed::Geometry;
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCylinderSphereGeometryTest,"IAmSpeed.Collision.Cylinder.ExactSphereFeatures",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCylinderSphereGeometryTest::RunTest(const FString&)
{
    const FCylinderGeometry C{10.,20.};FCylinderContact Hit;
    const auto Check=[&](FVector3d P,double R,double Expected,FVector3d N)
    {
        TestTrue(TEXT("Finite sphere query"),TryCylinderSphereContact(C,P,R,Hit));
        TestTrue(TEXT("Exact feature separation"),FMath::Abs(Hit.UpperSeparation-Expected)<1.e-10);
        TestTrue(TEXT("Exact feature normal"),Hit.Normal.Equals(N,1.e-10));
        TestTrue(TEXT("Witness difference agrees"),(Hit.ShapePoint-Hit.CylinderPoint).Equals(Hit.Normal*Expected,1.e-10));
    };
    Check({15.,0.,0.},2.,3.,{1.,0.,0.});Check({0.,0.,27.},2.,5.,{0.,0.,1.});
    Check({0.,0.,-27.},2.,5.,{0.,0.,-1.});
    const double D=FMath::Sqrt(34.);Check({13.,0.,25.},2.,D-2.,FVector3d(3.,0.,5.)/D);
    Check({9.,0.,0.},2.,-3.,{1.,0.,0.});Check({0.,0.,19.},2.,-3.,{0.,0.,1.});
    TestFalse(TEXT("Invalid cylinder rejected"),TryCylinderSphereContact({0.,20.},{0.,0.,0.},1.,Hit));
    TestFalse(TEXT("Negative sphere rejected"),TryCylinderSphereContact(C,{0.,0.,0.},-1.,Hit));
    return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCylinderBoxGeometryTest,"IAmSpeed.Collision.Cylinder.ConvexBoxWitnesses",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCylinderBoxGeometryTest::RunTest(const FString&)
{
    const FCylinderGeometry C{10.,20.};FCylinderContact H;
    const auto Check=[&](FVector3d P,FVector3d E,double Expected)
    {
        TestTrue(TEXT("Box admitted"),TryCylinderBoxContact(C,P,FQuat4d::Identity,E,H));
        if(!TestTrue(*FString::Printf(TEXT("Bounded convex oracle converged at %s, iterations=%d"),*P.ToString(),H.Iterations),H.bConverged))return;
        TestTrue(TEXT("Analytical separation bracket"),H.LowerSeparation<=Expected+2.e-6&&H.UpperSeparation>=Expected-2.e-6);
        TestTrue(TEXT("Tight separation bracket"),H.UpperSeparation-H.LowerSeparation<=1.01e-6);
        TestTrue(TEXT("Finite unit normal"),FMath::Abs(H.Normal.SizeSquared()-1.)<1.e-8);
        TestTrue(TEXT("Two witnesses agree with signed normal"),FMath::Abs(FVector3d::DotProduct(H.ShapePoint-H.CylinderPoint,H.Normal)-H.UpperSeparation)<4.e-6);
    };
    Check({15.,0.,0.},{2.,3.,2.},3.);Check({0.,0.,27.},{2.,3.,2.},5.);
    Check({15.,0.,27.},{2.,3.,2.},FMath::Sqrt(34.));
    Check({9.,0.,0.},{2.,3.,2.},-3.);Check({0.,0.,21.},{2.,3.,2.},-1.);
    Check({0.,0.,0.},{2.,2.,2.},-12.);Check({12.,0.,0.},{2.,3.,2.},0.);
    // Inclined box tangent to the circular side: the selected supporting
    // vertex has Y=0, Z inside the finite cylinder and exact X=Radius.
    for(int32 I=1;I<=12;++I)
    {
        const FVector3d E(2.,3.,2.);
        const FQuat4d Q=FRotator3d(I*3.,I*7.,I*2.).Quaternion();
        const FVector3d L=Q.UnrotateVector(FVector3d(-1.,0.,0.));
        const FVector3d V=Q.RotateVector(FVector3d(L.X>0.?E.X:-E.X,L.Y>0.?E.Y:-E.Y,L.Z>0.?E.Z:-E.Z));
        const FVector3d P=FVector3d(C.Radius,0.,0.)-V;
        TryCylinderBoxContact(C,P,Q,E,H);
        TestTrue(TEXT("Inclined exact tangent converges without fabricated penetration"),H.bConverged&&H.LowerSeparation<=2.e-6&&H.UpperSeparation>=-2.e-6);
        TestTrue(TEXT("Inclined tangent bracket stays tight"),H.UpperSeparation-H.LowerSeparation<=1.01e-6);
    }
    FRandomStream Random(294);
    for(int32 I=0;I<48;++I)
    {
        const FVector3d P(Random.FRandRange(-35.,35.),Random.FRandRange(-35.,35.),Random.FRandRange(-35.,35.));
        const FVector3d E(Random.FRandRange(1.,5.),Random.FRandRange(1.,5.),Random.FRandRange(1.,5.));
        const FQuat4d Q=FRotator3d(Random.FRandRange(-180.,180.),Random.FRandRange(-180.,180.),Random.FRandRange(-180.,180.)).Quaternion();
        TryCylinderBoxContact(C,P,Q,E,H);
        if(!TestTrue(TEXT("Rotated holdout converges"),H.bConverged))continue;
        const FVector3d Local=Q.UnrotateVector(H.ShapePoint-P);
        TestTrue(TEXT("Shape witness lies in OBB"),FMath::Abs(Local.X)<=E.X+1.e-5&&FMath::Abs(Local.Y)<=E.Y+1.e-5&&FMath::Abs(Local.Z)<=E.Z+1.e-5);
        const FVector3d Normal=H.Normal;const auto L=Q.UnrotateVector(-Normal);
        const auto BoxSupport=P+Q.RotateVector(FVector3d(L.X>=0.?E.X:-E.X,L.Y>=0.?E.Y:-E.Y,L.Z>=0.?E.Z:-E.Z));
        const double Gap=FVector3d::DotProduct(BoxSupport-C.Support(Normal),Normal);
        TestTrue(TEXT("Independent support plane certifies signed clearance"),FMath::Abs(Gap-H.LowerSeparation)<5.e-6);
        if(H.UpperSeparation<-.01)
        {
            FCylinderContact Released;TryCylinderBoxContact(C,P+Normal*(-H.LowerSeparation+.001),Q,E,Released);
            TestTrue(TEXT("MTD actually separates solids"),Released.bConverged&&Released.LowerSeparation>0.);
        }
    }
    return !HasAnyErrors();
}
#endif
