#include "TorusSphereSweep.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTorusSphereSweepTest,"IAmSpeed.Collision.Torus.SphereCCD",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FTorusSphereSweepTest::RunTest(const FString&)
{
    using namespace Speed::Geometry;
    const FTorusGeometry Torus{100.,10.};
    auto Hit=SweepTorusSphere(Torus,FVector3d(-30.,100.,0.),FVector3d(30.,100.,0.),5.);
    TestTrue(TEXT("Axial rim impact"),Hit.Result==ETorusSweepResult::Hit);
    TestTrue(TEXT("Analytical axial TOI"),FMath::Abs(Hit.Time-.25)<1.e-9);
    TestTrue(TEXT("Axial normal"),Hit.Contact.Normal.Equals(FVector3d(-1.,0.,0.),1.e-8));
    Hit=SweepTorusSphere(Torus,FVector3d(-1000000.,100.,0.),FVector3d(1000000.,100.,0.),5.);
    TestTrue(TEXT("High-speed impact cannot tunnel"),Hit.Result==ETorusSweepResult::Hit);
    TestTrue(TEXT("High-speed analytical TOI"),FMath::Abs(Hit.Time-.4999925)<1.e-10);
    Hit=SweepTorusSphere(Torus,FVector3d(-50.,0.,0.),FVector3d(50.,0.,0.),5.);
    TestTrue(TEXT("Hole remains empty"),Hit.Result==ETorusSweepResult::Miss);
    Hit=SweepTorusSphere(Torus,FVector3d(0.,-200.,0.),FVector3d(0.,200.,0.),5.);
    TestTrue(TEXT("First of four crossings wins"),Hit.Result==ETorusSweepResult::Hit);
    TestTrue(TEXT("Radial analytical TOI"),FMath::Abs(Hit.Time-.2125)<1.e-8);
    Hit=SweepTorusSphere(Torus,FVector3d(0.,-85.,0.),FVector3d(0.,200.,0.),5.);
    TestTrue(TEXT("Separating inner touch does not hide far-side impact"),Hit.Result==ETorusSweepResult::Hit && Hit.Time>0.);
    Hit=SweepTorusSphere(Torus,FVector3d(0.,105.,0.),FVector3d(0.,120.,0.),5.);
    TestTrue(TEXT("Initial penetration reported"),Hit.Result==ETorusSweepResult::Hit && Hit.bInitialOverlap);
    return !HasAnyErrors();
}
#endif
