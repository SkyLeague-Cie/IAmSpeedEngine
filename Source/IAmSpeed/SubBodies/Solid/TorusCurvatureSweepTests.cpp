#include "TorusBoxSweep.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "HAL/PlatformTime.h"
#include <limits>
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTorusCurvatureSweepTest,"IAmSpeed.Collision.Torus.SmoothCurvatureBounds238",
 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FTorusCurvatureSweepTest::RunTest(const FString&)
{
    using namespace Speed::Geometry;
    const FTorusGeometry Torus{641.294797713158,64.1294797713158};const FVector3d Half(78.79,52.3,19.14);
    // Smooth endpoint reconstructions, not a claim of original physical replay.
    {
        const FTransform A=FTransform(FRotator(35.399082,27.371064,8.652544),FVector(-113.792475,-548.941392,432.03267)),B=FTransform(FRotator(36.225542,27.986111,8.598812),FVector(-112.00862,-549.029741,433.529337));
        const auto PoseAt=[&](double T){return FTransform(FQuat::Slerp(A.GetRotation(),B.GetRotation(),T),FMath::Lerp(A.GetTranslation(),B.GetTranslation(),T));};
        const double Angle=A.GetRotation().AngularDistance(B.GetRotation());
        const double L=(B.GetTranslation()-A.GetTranslation()).Size()+Angle*Half.Size();
        const double Acceleration=Angle*Angle*Half.Size();
        int32 Evaluations=0;const double Start=FPlatformTime::Seconds();
        const auto Hit=SweepTorusShapeBoundedTrajectory(Torus,false,0.,Half,PoseAt,L,.05,Acceleration,8.*double(std::numeric_limits<float>::epsilon())*(1.+L),&Evaluations);
        AddInfo(FString::Printf(TEXT("Smooth recorded endpoints0 result=%d evaluations=%d duration_ms=%.3f"),int32(Hit.Result),Evaluations,1000.*(FPlatformTime::Seconds()-Start)));
        TestTrue(TEXT("Smooth grazing resolves"),Hit.Result==ETorusSweepResult::Hit || Hit.Result==ETorusSweepResult::Miss);
        TestTrue(TEXT("Second-derivative certificate avoids thousands of pose queries"),Evaluations<128);
        if(Hit.Result==ETorusSweepResult::Hit && Hit.Time>0.) TestTrue(TEXT("Unchanged spatial bound"),FMath::Abs(Hit.Separation)<=Hit.ErrorBoundCm+1.e-5);
    }
    {
        const FTransform A=FTransform(FRotator(50.216996,-82.947267,-92.150387),FVector(-41.782547,-686.967412,-332.367077)),B=FTransform(FRotator(49.790896,-83.098,-91.31925),FVector(-41.573551,-687.133151,-334.264974));
        const auto PoseAt=[&](double T){return FTransform(FQuat::Slerp(A.GetRotation(),B.GetRotation(),T),FMath::Lerp(A.GetTranslation(),B.GetTranslation(),T));};
        const double Angle=A.GetRotation().AngularDistance(B.GetRotation());
        const double L=(B.GetTranslation()-A.GetTranslation()).Size()+Angle*Half.Size();
        const double Acceleration=Angle*Angle*Half.Size();
        int32 Evaluations=0;const double Start=FPlatformTime::Seconds();
        const auto Hit=SweepTorusShapeBoundedTrajectory(Torus,false,0.,Half,PoseAt,L,.05,Acceleration,8.*double(std::numeric_limits<float>::epsilon())*(1.+L),&Evaluations);
        AddInfo(FString::Printf(TEXT("Smooth recorded endpoints1 result=%d evaluations=%d duration_ms=%.3f"),int32(Hit.Result),Evaluations,1000.*(FPlatformTime::Seconds()-Start)));
        TestTrue(TEXT("Smooth grazing resolves"),Hit.Result==ETorusSweepResult::Hit || Hit.Result==ETorusSweepResult::Miss);
        TestTrue(TEXT("Second-derivative certificate avoids thousands of pose queries"),Evaluations<128);
        if(Hit.Result==ETorusSweepResult::Hit && Hit.Time>0.) TestTrue(TEXT("Unchanged spatial bound"),FMath::Abs(Hit.Separation)<=Hit.ErrorBoundCm+1.e-5);
    }
    {
        const FTransform A=FTransform(FRotator(40.097207,-53.180842,-18.028367),FVector(-124.782508,-539.610558,-257.062631)),B=FTransform(FRotator(40.727478,-53.337373,-18.961594),FVector(-123.550483,-539.930925,-257.085593));
        const auto PoseAt=[&](double T){return FTransform(FQuat::Slerp(A.GetRotation(),B.GetRotation(),T),FMath::Lerp(A.GetTranslation(),B.GetTranslation(),T));};
        const double Angle=A.GetRotation().AngularDistance(B.GetRotation());
        const double L=(B.GetTranslation()-A.GetTranslation()).Size()+Angle*Half.Size();
        const double Acceleration=Angle*Angle*Half.Size();
        int32 Evaluations=0;const double Start=FPlatformTime::Seconds();
        const auto Hit=SweepTorusShapeBoundedTrajectory(Torus,false,0.,Half,PoseAt,L,.05,Acceleration,8.*double(std::numeric_limits<float>::epsilon())*(1.+L),&Evaluations);
        AddInfo(FString::Printf(TEXT("Smooth recorded endpoints2 result=%d evaluations=%d duration_ms=%.3f"),int32(Hit.Result),Evaluations,1000.*(FPlatformTime::Seconds()-Start)));
        TestTrue(TEXT("Smooth grazing resolves"),Hit.Result==ETorusSweepResult::Hit || Hit.Result==ETorusSweepResult::Miss);
        TestTrue(TEXT("Second-derivative certificate avoids thousands of pose queries"),Evaluations<128);
        if(Hit.Result==ETorusSweepResult::Hit && Hit.Time>0.) TestTrue(TEXT("Unchanged spatial bound"),FMath::Abs(Hit.Separation)<=Hit.ErrorBoundCm+1.e-5);
    }
    {
        const FTransform A=FTransform(FRotator(52.213666,4.675259,-3.83507),FVector(-115.932226,523.962152,-261.095795)),B=FTransform(FRotator(52.976712,4.549333,-4.652949),FVector(-114.639037,523.912198,-261.769715));
        const auto PoseAt=[&](double T){return FTransform(FQuat::Slerp(A.GetRotation(),B.GetRotation(),T),FMath::Lerp(A.GetTranslation(),B.GetTranslation(),T));};
        const double Angle=A.GetRotation().AngularDistance(B.GetRotation());
        const double L=(B.GetTranslation()-A.GetTranslation()).Size()+Angle*Half.Size();
        const double Acceleration=Angle*Angle*Half.Size();
        int32 Evaluations=0;const double Start=FPlatformTime::Seconds();
        const auto Hit=SweepTorusShapeBoundedTrajectory(Torus,false,0.,Half,PoseAt,L,.05,Acceleration,8.*double(std::numeric_limits<float>::epsilon())*(1.+L),&Evaluations);
        AddInfo(FString::Printf(TEXT("Smooth recorded endpoints3 result=%d evaluations=%d duration_ms=%.3f"),int32(Hit.Result),Evaluations,1000.*(FPlatformTime::Seconds()-Start)));
        TestTrue(TEXT("Smooth grazing resolves"),Hit.Result==ETorusSweepResult::Hit || Hit.Result==ETorusSweepResult::Miss);
        TestTrue(TEXT("Second-derivative certificate avoids thousands of pose queries"),Evaluations<128);
        if(Hit.Result==ETorusSweepResult::Hit && Hit.Time>0.) TestTrue(TEXT("Unchanged spatial bound"),FMath::Abs(Hit.Separation)<=Hit.ErrorBoundCm+1.e-5);
    }

    const FTorusGeometry Small{100.,10.};
    const auto Axial=[](double T){return FTransform(FVector(-30.+60.*T,100.,0.));};
    const auto Hit=SweepTorusShapeBoundedTrajectory(Small,true,5.,FVector3d::ZeroVector,Axial,60.,.05,0.);
    TestTrue(TEXT("Analytical sphere first-entry bound unchanged"),Hit.Result==ETorusSweepResult::Hit && FMath::Abs(Hit.Time-.25)*60.<=2.e-4);
    const auto Uncertain=SweepTorusShapeBoundedTrajectory(Small,true,5.,FVector3d::ZeroVector,Axial,60.,.05,0.,.002);
    TestTrue(TEXT("Over-uncertain samples must not manufacture a collision/miss"),Uncertain.Result==ETorusSweepResult::Unresolved);
    const auto Hole=[](double T){return FTransform(FVector(-50.+100.*T,0.,0.));};
    const auto Miss=SweepTorusShapeBoundedTrajectory(Small,false,0.,FVector3d(10.),Hole,100.,.05,0.);
    TestTrue(TEXT("Axis-singular region falls back safely and preserves empty hole"),Miss.Result==ETorusSweepResult::Miss);
    return !HasAnyErrors();
}
#endif
