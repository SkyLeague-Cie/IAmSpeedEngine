#include "TorusBoxGeometry.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTorusBoxDistanceTest,"IAmSpeed.Collision.Torus.BoxDistanceBounds",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FTorusBoxDistanceTest::RunTest(const FString&)
{
    using namespace Speed::Geometry;
    const FTorusGeometry Torus{100.,10.}; FTorusBoxDistance Result;
    const auto Check=[this,&Torus,&Result](const FVector3d& Centre,const FVector3d& Extent,double Expected)
    {
        TestTrue(TEXT("Valid query"),TryTorusBoxDistance(Torus,Centre,FQuat4d::Identity,Extent,1.e-5,8192,Result));
        TestTrue(TEXT("Bounded solver converged"),Result.bConverged);
        TestTrue(TEXT("Analytical oracle enclosed"),Result.LowerSeparation<=Expected+1.e-9 && Result.UpperSeparation>=Expected-1.e-9);
        TestTrue(TEXT("Certified distance width"),Result.UpperSeparation-Result.LowerSeparation<=1.0001e-5);
    };
    Check(FVector3d(0.,125.,0.),FVector3d(5.),10.);
    Check(FVector3d(0.,115.,0.),FVector3d(5.),0.);
    Check(FVector3d::ZeroVector,FVector3d(10.),90.-FMath::Sqrt(200.));
    Check(FVector3d(20.,100.,0.),FVector3d(5.),5.);
    TestTrue(TEXT("Finite budget query accepted"),TryTorusBoxDistance(Torus,FVector3d::ZeroVector,FQuat4d::Identity,FVector3d(10.),1.e-9,1,Result));
    TestFalse(TEXT("Insufficient budget never claims convergence"),Result.bConverged);
    TestFalse(TEXT("Invalid OBB rejected"),TryTorusBoxDistance(Torus,FVector3d::ZeroVector,FQuat4d::Identity,FVector3d(-1.),1.e-5,32,Result));
    return !HasAnyErrors();
}
#endif
