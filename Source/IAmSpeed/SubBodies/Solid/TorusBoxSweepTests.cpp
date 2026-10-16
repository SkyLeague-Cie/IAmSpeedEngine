#include "TorusBoxSweep.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTorusBoxSweepTest,"IAmSpeed.Collision.Torus.BoxTrajectoryCCD",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FTorusBoxSweepTest::RunTest(const FString&)
{
    using namespace Speed::Geometry;
    const FTorusGeometry Torus{100.,10.};const FVector3d Half(5.);
    auto Axial=[](double T){return FTransform(FVector(-30.+60.*T,100.,0.));};
    auto Hit=SweepTorusBoxTrajectory(Torus,Half,Axial,60.);
    TestTrue(TEXT("Axial box impact"),Hit.Result==ETorusSweepResult::Hit);
    TestTrue(TEXT("Axial analytical TOI within spatial bound"),FMath::Abs(Hit.Time-.25)*60.<=2.e-4);
    TestTrue(TEXT("Curved surface witness"),Hit.Normal.Equals(FVector(-1.,0.,0.),1.e-5));
    auto Hole=[](double T){return FTransform(FVector(-50.+100.*T,0.,0.));};
    Hit=SweepTorusBoxTrajectory(Torus,FVector3d(10.),Hole,100.);
    TestTrue(TEXT("Box flight through central hole"),Hit.Result==ETorusSweepResult::Miss);
    const FVector3d LongHalf(2.,5.,20.);
    auto Rotate=[](double T){return FTransform(FQuat(FVector::RightVector,PI*.5*T),FVector(25.,100.,0.));};
    Hit=SweepTorusBoxTrajectory(Torus,LongHalf,Rotate,PI*.5*LongHalf.Size());
    TestTrue(TEXT("Rotation-only impact detected"),Hit.Result==ETorusSweepResult::Hit && Hit.Time>0. && Hit.Time<1.);
    TestTrue(TEXT("Witness separation bounded"),FMath::Abs(Hit.Separation)<=Hit.ErrorBoundCm+1.e-5);
    // A wheel follows a rotating owner offset, while its spherical shape itself
    // has no orientation. This is the same values-only query used by the bridge.
    auto WheelArc=[](double T){return FTransform(FVector(-30.+60.*T,100.,0.));};
    Hit=SweepTorusShapeTrajectory(Torus,true,5.,FVector3d::ZeroVector,WheelArc,60.);
    TestTrue(TEXT("Round wheel trajectory impact"),Hit.Result==ETorusSweepResult::Hit);
    TestTrue(TEXT("Round wheel analytical TOI"),FMath::Abs(Hit.Time-.25)*60.<=2.e-4);
    // Persistent initial contact must not visit thousands of temporal leaves.
    // Use the production-scale torus and an intentionally loose valid bound.
    const FTorusGeometry Large{641.,64.};
    auto Resting=[](double){return FTransform(FVector(-68.995,641.,0.));};
    Hit=SweepTorusBoxTrajectory(Large,Half,Resting,10000.);
    TestTrue(TEXT("Initial quantized overlap resolves at t0 within unchanged budget"),
        Hit.Result==ETorusSweepResult::Hit && Hit.Time==0. && Hit.Separation<0.);
    return !HasAnyErrors();
}
#endif
