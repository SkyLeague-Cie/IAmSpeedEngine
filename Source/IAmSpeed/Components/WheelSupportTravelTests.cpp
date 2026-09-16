#if WITH_DEV_AUTOMATION_TESTS
#include "WheelSupportTravel.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedWheelSupportTravelTest,
    "IAmSpeed.PhysicalLaws.WheelSupport.TravelBeforeRigidProjection",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedWheelSupportTravelTest::RunTest(const FString& Parameters)
{
    // The support point is built independently from the requested sphere
    // center; no stale spring displacement participates in rigid feasibility.
    for (const FQuat& Rotation : { FQuat::Identity, FRotator(70,35,12).Quaternion(),
                                  FRotator(180,0,0).Quaternion() })
    {
        const FVector Origin(23,-47,81);
        const FVector Up=Rotation.RotateVector(FVector::UpVector);
        for (const double Alignment : { 1., .5, 0., -.5, -1. })
        {
            const FVector Normal=Rotation.RotateVector(FVector(FMath::Sqrt(1-Alignment*Alignment),0,Alignment));
            const double Limit=Alignment>=0 ? 15. : -8.;
            for (const double TravelBeyondLimit : { -3., 0., 3. })
            {
                const double Compression=Limit+(Alignment>=0 ? 1 : -1)*TravelBeyondLimit;
                const FVector Point=Origin+Compression*Up-12.5*Normal;
                const double Actual=Speed::WheelSupportTravelPenetration(Origin,Up,15,8,Point,Normal,12.5);
                const double Expected=TravelBeyondLimit*FMath::Abs(Alignment);
                TestTrue(TEXT("true excess travel remains a signed normal violation"),
                    FMath::IsNearlyEqual(Actual,Expected,1.e-10));
            }
        }
        for (const double Compression : { 4.999,5.001,5.29294,7.224,14.999 })
        {
            const FVector Point=Origin+Compression*Up-12.5*Up;
            TestTrue(TEXT("crossing force bump-stop onset within available travel cannot expel chassis"),
                Speed::WheelSupportTravelPenetration(Origin,Up,15,8,Point,Up,12.5)<0);
            const double OldDepth=FVector::DotProduct(Point+12.5*Up-(Origin-8*Up),Up);
            TestTrue(TEXT("stale deployed center would incorrectly penetrate"),OldDepth>12.9);
        }
    }
    return true;
}
#endif
