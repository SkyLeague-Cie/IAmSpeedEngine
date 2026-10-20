#include "CylinderSweep.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
using namespace Speed::Geometry;
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCylinderSweepGeometryTest,"IAmSpeed.Collision.Cylinder.MovingCCD",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCylinderSweepGeometryTest::RunTest(const FString&)
{
    const FCylinderGeometry C{10.,20.};
    const auto Fixed=[](double){return FTransform::Identity;};
    const auto Sphere=[](double T){return FTransform(FQuat::Identity,FVector(20.-40.*T,0.,0.));};
    const auto H=SweepCylinderShapeTrajectory(C,true,2.,FVector3d::ZeroVector,Fixed,Sphere,40.,0.);
    TestTrue(TEXT("Fast crossing, clear endpoints, earliest side contact"),H.Result==ECylinderSweepResult::Hit&&FMath::Abs(H.Time-.2)<1.e-5);
    const auto Moving=[](double T){return FTransform(FQuat::Identity,FVector(40.*T,0.,0.));};
    const auto Stationary=[](double){return FTransform(FQuat::Identity,FVector(20.,0.,0.));};
    const auto M=SweepCylinderShapeTrajectory(C,true,2.,FVector3d::ZeroVector,Moving,Stationary,40.,0.);
    TestTrue(TEXT("Cylinder movement participates in CCD"),M.Result==ECylinderSweepResult::Hit&&FMath::Abs(M.Time-.2)<1.e-5);
    const auto Box=[](double T){return FTransform(FQuat::Identity,FVector(20.-40.*T,0.,0.));};
    const auto B=SweepCylinderShapeTrajectory(C,false,0.,{2.,3.,2.},Fixed,Box,40.,0.);
    TestTrue(TEXT("Box enters exact curved solid"),B.Result==ECylinderSweepResult::Hit&&FMath::Abs(B.Time-.2)<1.e-5);
    const auto Separating=[](double T){return FTransform(FQuat::Identity,FVector(12.+T,0.,0.));};
    const auto S=SweepCylinderShapeTrajectory(C,false,0.,{2.,3.,2.},Fixed,Separating,1.,0.);
    TestTrue(TEXT("Initial tangent separating is released"),S.Result==ECylinderSweepResult::Miss&&S.PoseEvaluations<=3);
    const auto Sliding=[](double T){return FTransform(FQuat::Identity,FVector(12.,0.,T));};
    const auto L=SweepCylinderShapeTrajectory(C,false,0.,{2.,3.,2.},Fixed,Sliding,1.,0.);
    TestTrue(TEXT("Axial slide has a certified nonpenetrating support plane"),L.Result==ECylinderSweepResult::Miss&&L.PoseEvaluations<=3);
    const auto Shallow=[](double T){return FTransform(FQuat::Identity,FVector(11.99999+T,0.,0.));};
    const auto R=SweepCylinderShapeTrajectory(C,true,2.,FVector3d::ZeroVector,Fixed,Shallow,1.,0.);
    TestTrue(TEXT("Sub-slop separating contact cannot loop at time zero"),R.Result==ECylinderSweepResult::Miss);
    const auto Cap=[](double T){return FTransform(FQuat::Identity,FVector(0.,0.,30.-40.*T));};
    const auto K=SweepCylinderShapeTrajectory(C,true,2.,FVector3d::ZeroVector,Fixed,Cap,40.,0.);
    TestTrue(TEXT("Finite cap is solid"),K.Result==ECylinderSweepResult::Hit&&FMath::Abs(K.Time-.2)<1.e-5);
    const auto Overlap=[](double){return FTransform(FQuat::Identity,FVector(9.,0.,0.));};
    const auto O=SweepCylinderShapeTrajectory(C,true,2.,FVector3d::ZeroVector,Fixed,Overlap,0.,0.);
    TestTrue(TEXT("Deep initial overlap remains explicit"),O.Result==ECylinderSweepResult::Hit&&O.Time==0.&&FMath::Abs(O.Contact.UpperSeparation+3.)<1.e-12);
    const auto Miss=[](double T){return FTransform(FQuat::Identity,FVector(12.1,10.*T,0.));};
    const auto N=SweepCylinderShapeTrajectory(C,true,2.,FVector3d::ZeroVector,Fixed,Miss,10.,0.);
    TestTrue(TEXT("Near miss not inflated into hit"),N.Result==ECylinderSweepResult::Miss);
    return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCylinderProjectedSlideTest,"IAmSpeed.Collision.Cylinder.ProjectedSideSlide",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCylinderProjectedSlideTest::RunTest(const FString&)
{
    constexpr double Dt=1./300.;const FCylinderGeometry C{10.,20.};
    const auto Fixed=[](double){return FTransform::Identity;};
    const auto Slide=[](double T){const float S=float(T*Dt);return FTransform(FQuat::Identity,FVector(12.,1000.*S,-325.*double(S*S)));};
    FCylinderProjectedMotionBounds P;P.TranslationVelocity={0.,1000.*Dt,0.};P.TranslationAcceleration={0.,0.,-650.*Dt*Dt};P.TransformRoundoff=1.e-9;
    const auto H=SweepCylinderShapeTrajectory(C,false,0.,{2.,6.,2.},Fixed,Slide,3.35,.0073,1.e-5,&P);
    TestTrue(TEXT("Long exact box-side tangent with parallel gravity is released within declared spatial error"),H.Result==ECylinderSweepResult::Miss);
    TestTrue(TEXT("Certified projected bound avoids budget exhaustion"),H.PoseEvaluations<=4);
    const auto Closing=[](double T){const float S=float(T*Dt);return FTransform(FQuat::Identity,FVector(12.-100.*S,1000.*S,0.));};
    P.TranslationVelocity.X=-100.*Dt;P.TranslationAcceleration=FVector3d::ZeroVector;
    const auto Hit=SweepCylinderShapeTrajectory(C,false,0.,{2.,6.,2.},Fixed,Closing,3.7,0.,1.e-5,&P);
    TestTrue(TEXT("Projection cannot hide an inward approach"),Hit.Result==ECylinderSweepResult::Hit&&Hit.Time<1.e-4);
    const auto CoCylinder=[](double T){return FTransform(FVector(50.+100.*float(T*Dt),0.,0.));};
    const auto CoSphere=[](double T){return FTransform(FVector(60.99+100.*float(T*Dt),0.,0.));};
    FCylinderProjectedMotionBounds Co;Co.TransformRoundoff=1.e-9;
    const auto Held=SweepCylinderShapeTrajectory(C,true,1.,FVector3d::ZeroVector,CoCylinder,CoSphere,1.,0.,4.e-6,&Co);
    TestTrue(TEXT("Equal physical velocities release a tolerated initial overlap with float-time bounds"),Held.Result==ECylinderSweepResult::Miss&&Held.PoseEvaluations<=4);
    const auto FloatSlop=[](double T){return FTransform(FVector(61.-double(.05f)-1.e-10+100.*float(T*Dt),0.,0.));};
    const auto Slop=SweepCylinderShapeTrajectory(C,true,1.,FVector3d::ZeroVector,CoCylinder,FloatSlop,1.,0.,4.e-6,&Co);
    TestTrue(TEXT("Float response slop bin cannot produce an infinite zero-correction CCD loop"),Slop.Result==ECylinderSweepResult::Miss&&Slop.PoseEvaluations<=4);
    const auto Inward=[](double T){return FTransform(FVector(60.99+90.*float(T*Dt),0.,0.));};
    Co.TranslationVelocity={-10.*Dt,0.,0.};
    const auto Again=SweepCylinderShapeTrajectory(C,true,1.,FVector3d::ZeroVector,CoCylinder,Inward,1.,0.,4.e-6,&Co);
    TestTrue(TEXT("Tolerated overlap still catches a genuinely inward relative velocity"),Again.Result==ECylinderSweepResult::Hit);
    return !HasAnyErrors();
}
#endif
