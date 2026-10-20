#include "TorusBoxSweep.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "HAL/PlatformTime.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTorusRecordedTrajectoryTest,"IAmSpeed.Collision.Torus.RecordedGrazing227",
 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FTorusRecordedTrajectoryTest::RunTest(const FString&)
{
    using namespace Speed::Geometry;
    // Five rounded diagnostic poses per human failure, interpolated with
    // piecewise translation/slerp. These reproduce grazing geometry, not the
    // original unpublished physical trajectory or its complete input history.
    const FTorusGeometry Torus{641.294797713158,64.1294797713158};const FVector3d Half(78.79,52.3,19.14);
    {
        const FTransform Samples[]={FTransform(FRotator(35.399082,27.371064,8.652544),FVector(-113.792475,-548.941392,432.03267)),
            FTransform(FRotator(35.603245,27.525908,8.635816),FVector(-113.346004,-548.964044,432.406884)),
            FTransform(FRotator(35.809047,27.68002,8.621277),FVector(-112.89987,-548.98632,432.78107)),
            FTransform(FRotator(36.016482,27.833415,8.608939),FVector(-112.454075,-549.00822,433.155223)),
            FTransform(FRotator(36.225542,27.986111,8.598812),FVector(-112.00862,-549.029741,433.529337))};
        const auto PoseAt=[&](double T)
        {
            const double U=FMath::Clamp(T*4.,0.,4.);const int32 I=FMath::Min(3,int32(U));
            return FTransform(FQuat::Slerp(Samples[I].GetRotation(),Samples[I+1].GetRotation(),U-I),
                FMath::Lerp(Samples[I].GetTranslation(),Samples[I+1].GetTranslation(),U-I));
        };
        double Bound=4.7161230136723837;
        for(int32 I=0;I<4;++I) Bound=FMath::Max(Bound,4.*((Samples[I+1].GetTranslation()-Samples[I].GetTranslation()).Size()+
            Half.Size()*Samples[I].GetRotation().AngularDistance(Samples[I+1].GetRotation())));
        const double Start=FPlatformTime::Seconds();
        const auto Hit=SweepTorusBoxTrajectory(Torus,Half,PoseAt,Bound);
        AddInfo(FString::Printf(TEXT("Recorded case0 result=%d duration_ms=%.3f bound=%.9g"),int32(Hit.Result),1000.*(FPlatformTime::Seconds()-Start),Bound));
        TestTrue(TEXT("Recorded case0 resolves within the unchanged CCD budget"),Hit.Result==ETorusSweepResult::Hit || Hit.Result==ETorusSweepResult::Miss);
        if(Hit.Result==ETorusSweepResult::Hit && Hit.Time>0.) TestTrue(TEXT("Unchanged spatial contact bound"),FMath::Abs(Hit.Separation)<=Hit.ErrorBoundCm+1.e-5);
    }
    {
        const FTransform Samples[]={FTransform(FRotator(50.216996,-82.947267,-92.150387),FVector(-41.782547,-686.967412,-332.367077)),
            FTransform(FRotator(50.110086,-82.984311,-91.940688),FVector(-41.730612,-687.008591,-332.840802)),
            FTransform(FRotator(50.003433,-83.021786,-91.732271),FVector(-41.678466,-687.04994,-333.315026)),
            FTransform(FRotator(49.897036,-83.059686,-91.525127),FVector(-41.626112,-687.09146,-333.789749)),
            FTransform(FRotator(49.790896,-83.098,-91.31925),FVector(-41.573551,-687.133151,-334.264974))};
        const auto PoseAt=[&](double T)
        {
            const double U=FMath::Clamp(T*4.,0.,4.);const int32 I=FMath::Min(3,int32(U));
            return FTransform(FQuat::Slerp(Samples[I].GetRotation(),Samples[I+1].GetRotation(),U-I),
                FMath::Lerp(Samples[I].GetTranslation(),Samples[I+1].GetTranslation(),U-I));
        };
        double Bound=4.1541243957866802;
        for(int32 I=0;I<4;++I) Bound=FMath::Max(Bound,4.*((Samples[I+1].GetTranslation()-Samples[I].GetTranslation()).Size()+
            Half.Size()*Samples[I].GetRotation().AngularDistance(Samples[I+1].GetRotation())));
        const double Start=FPlatformTime::Seconds();
        const auto Hit=SweepTorusBoxTrajectory(Torus,Half,PoseAt,Bound);
        AddInfo(FString::Printf(TEXT("Recorded case1 result=%d duration_ms=%.3f bound=%.9g"),int32(Hit.Result),1000.*(FPlatformTime::Seconds()-Start),Bound));
        TestTrue(TEXT("Recorded case1 resolves within the unchanged CCD budget"),Hit.Result==ETorusSweepResult::Hit || Hit.Result==ETorusSweepResult::Miss);
        TestTrue(TEXT("Recorded positive grazing clearance is not a collision"),Hit.Result==ETorusSweepResult::Miss);
        if(Hit.Result==ETorusSweepResult::Hit && Hit.Time>0.) TestTrue(TEXT("Unchanged spatial contact bound"),FMath::Abs(Hit.Separation)<=Hit.ErrorBoundCm+1.e-5);
    }
    {
        const FTransform Samples[]={FTransform(FRotator(40.097207,-53.180842,-18.028367),FVector(-124.782508,-539.610558,-257.062631)),
            FTransform(FRotator(40.253576,-53.216945,-18.260813),FVector(-124.474148,-539.690331,-257.067048)),
            FTransform(FRotator(40.410749,-53.255056,-18.493819),FVector(-124.166024,-539.770317,-257.072346)),
            FTransform(FRotator(40.568719,-53.295192,-18.727407),FVector(-123.858136,-539.850516,-257.078527)),
            FTransform(FRotator(40.727478,-53.337373,-18.961594),FVector(-123.550483,-539.930925,-257.085593))};
        const auto PoseAt=[&](double T)
        {
            const double U=FMath::Clamp(T*4.,0.,4.);const int32 I=FMath::Min(3,int32(U));
            return FTransform(FQuat::Slerp(Samples[I].GetRotation(),Samples[I+1].GetRotation(),U-I),
                FMath::Lerp(Samples[I].GetTranslation(),Samples[I+1].GetTranslation(),U-I));
        };
        double Bound=3.6964447217136853;
        for(int32 I=0;I<4;++I) Bound=FMath::Max(Bound,4.*((Samples[I+1].GetTranslation()-Samples[I].GetTranslation()).Size()+
            Half.Size()*Samples[I].GetRotation().AngularDistance(Samples[I+1].GetRotation())));
        const double Start=FPlatformTime::Seconds();
        const auto Hit=SweepTorusBoxTrajectory(Torus,Half,PoseAt,Bound);
        AddInfo(FString::Printf(TEXT("Recorded case2 result=%d duration_ms=%.3f bound=%.9g"),int32(Hit.Result),1000.*(FPlatformTime::Seconds()-Start),Bound));
        TestTrue(TEXT("Recorded case2 resolves within the unchanged CCD budget"),Hit.Result==ETorusSweepResult::Hit || Hit.Result==ETorusSweepResult::Miss);
        TestTrue(TEXT("Recorded inward grazing contact is retained"),Hit.Result==ETorusSweepResult::Hit);
        if(Hit.Result==ETorusSweepResult::Hit && Hit.Time>0.) TestTrue(TEXT("Unchanged spatial contact bound"),FMath::Abs(Hit.Separation)<=Hit.ErrorBoundCm+1.e-5);
    }
    {
        const FTransform Samples[]={FTransform(FRotator(52.213666,4.675259,-3.83507),FVector(-115.932226,523.962152,-261.095795)),
            FTransform(FRotator(52.40446,4.645124,-4.038415),FVector(-115.609422,523.949627,-261.263147)),
            FTransform(FRotator(52.595233,4.614099,-4.242501),FVector(-115.28629,523.937126,-261.431252)),
            FTransform(FRotator(52.785984,4.582173,-4.447342),FVector(-114.962829,523.924649,-261.600108)),
            FTransform(FRotator(52.976712,4.549333,-4.652949),FVector(-114.639037,523.912198,-261.769715))};
        const auto PoseAt=[&](double T)
        {
            const double U=FMath::Clamp(T*4.,0.,4.);const int32 I=FMath::Min(3,int32(U));
            return FTransform(FQuat::Slerp(Samples[I].GetRotation(),Samples[I+1].GetRotation(),U-I),
                FMath::Lerp(Samples[I].GetTranslation(),Samples[I+1].GetTranslation(),U-I));
        };
        double Bound=3.7348322300644834;
        for(int32 I=0;I<4;++I) Bound=FMath::Max(Bound,4.*((Samples[I+1].GetTranslation()-Samples[I].GetTranslation()).Size()+
            Half.Size()*Samples[I].GetRotation().AngularDistance(Samples[I+1].GetRotation())));
        const double Start=FPlatformTime::Seconds();
        const auto Hit=SweepTorusBoxTrajectory(Torus,Half,PoseAt,Bound);
        AddInfo(FString::Printf(TEXT("Recorded case3 result=%d duration_ms=%.3f bound=%.9g"),int32(Hit.Result),1000.*(FPlatformTime::Seconds()-Start),Bound));
        TestTrue(TEXT("Recorded case3 resolves within the unchanged CCD budget"),Hit.Result==ETorusSweepResult::Hit || Hit.Result==ETorusSweepResult::Miss);
        if(Hit.Result==ETorusSweepResult::Hit && Hit.Time>0.) TestTrue(TEXT("Unchanged spatial contact bound"),FMath::Abs(Hit.Separation)<=Hit.ErrorBoundCm+1.e-5);
    }
    return !HasAnyErrors();
}
#endif
