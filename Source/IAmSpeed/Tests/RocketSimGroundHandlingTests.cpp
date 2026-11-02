#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "UObject/UObjectGlobals.h"

struct FRocketSimGroundHandlingFixtureAccess
{
    static float Angle(const USpeedWheeledComponent& V, float Speed)
    { return V.ComputeWheelFrameSteerAngle(Speed, 123.f, 456.f); }
};

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRocketSimGroundDataTest,
    "IAmSpeed.Handling.RocketSimGround.PinnedDefaults",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRocketSimGroundDataTest::RunTest(const FString&)
{
    USpeedWheeledComponent* V = NewObject<USpeedWheeledComponent>();
    if (!TestNotNull(TEXT("Unregistered component"), V)) return false;
    TestFalse(TEXT("Base IAS has no invented boost/fuel state"), V->HasBoostThrottleForRocketSimGroundHandling());
    const FVector OriginalCOM = V->CenterOfMass;
    const float OriginalMass = V->PhysMass, OriginalGravity = V->GravityZ;
    V->WheelFrameLateralForwardLossScale = 3.5f;
    V->WheelFrameLongitudinalScrubScale = .15f;
    V->ConfigureRocketSimGroundHandlingDefaults();
    TestEqual(TEXT("Explicit per-instance version"), V->RocketSimGroundHandlingVersion, 1);
    TestEqual(TEXT("Natural forward projection retained"), V->WheelFrameLateralForwardLossScale, 1.f);
    TestEqual(TEXT("No added scrub"), V->WheelFrameLongitudinalScrubScale, 0.f);
    TestEqual(TEXT("No normal rear gain"), V->WheelFrameSteeringRearLateralFrictionScale, 1.f);
    TestEqual(TEXT("All front/rear response one"), V->WheelFrameFrontLateralResponseScale * V->WheelFrameRearLateralResponseScale, 1.f);
    TestEqual(TEXT("Six source normal knots"), V->SteerAngleFromSpeedCurve.Num(), 6);
    TestEqual(TEXT("Two source handbrake knots"), V->HandbrakeSteerAngleFromSpeedCurve.Num(), 2);
    TestEqual(TEXT("No speed loss compensation"), V->LateralForwardLossScaleCurve.Num() + V->LateralForwardLossSlipScaleCurve.Num() + V->SteeringForwardLossSpeedScaleCurve.Num(), 0);
    TestEqual(TEXT("Handbrake source rise"), V->WheelFrameHandbrakeRiseRate, 5.f);
    TestEqual(TEXT("Handbrake source fall"), V->WheelFrameHandbrakeFallRate, 2.f);
    TestEqual(TEXT("No geometry normalization"), V->WheelFrameSteeringReferenceWheelbaseCm, 0.f);
    TestTrue(TEXT("COM preserved"), V->CenterOfMass == OriginalCOM);
    TestEqual(TEXT("Mass preserved"), V->PhysMass, OriginalMass);
    TestEqual(TEXT("Gravity preserved"), V->GravityZ, OriginalGravity);
    V->ConfigureRocketSimGroundHandlingDefaults();
    TestTrue(TEXT("Reapplication remains exact"), V->SteerAngleFromSpeedCurve[2] == FVector2D(1000.f, .18203f));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRocketSimGroundAngleTest,
    "IAmSpeed.Handling.RocketSimGround.SourceAngleBlend",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRocketSimGroundAngleTest::RunTest(const FString&)
{
    USpeedWheeledComponent* V = NewObject<USpeedWheeledComponent>();
    if (!TestNotNull(TEXT("Unregistered component"), V)) return false;
    V->ConfigureRocketSimGroundHandlingDefaults();
    V->WheelFrameHandbrakeValue = 0.f;
    TestTrue(TEXT("Native speed knot, not target radius"), FMath::IsNearlyEqual(FRocketSimGroundHandlingFixtureAccess::Angle(*V, 1000.f), .18203f, 1.e-6f));
    V->WheelFrameHandbrakeValue = 1.f;
    const float PowerslideAt1000 = .39235f + (.12610f - .39235f) * (1000.f / 2500.f);
    TestTrue(TEXT("Source full handbrake curve"), FMath::IsNearlyEqual(FRocketSimGroundHandlingFixtureAccess::Angle(*V, 1000.f), PowerslideAt1000, 1.e-6f));
    V->WheelFrameHandbrakeValue = .25f;
    const float Expected = .18203f + (PowerslideAt1000 - .18203f) * .25f;
    TestTrue(TEXT("Source analog handbrake blend"), FMath::IsNearlyEqual(FRocketSimGroundHandlingFixtureAccess::Angle(*V, 1000.f), Expected, 1.e-6f));
    V->WheelFrameHandbrakeValue = 0.f;
    TestTrue(TEXT("Source low speed endpoint"), FMath::IsNearlyEqual(FRocketSimGroundHandlingFixtureAccess::Angle(*V, 0.f), .53356f, 1.e-6f));
    TestTrue(TEXT("Source upper endpoint"), FMath::IsNearlyEqual(FRocketSimGroundHandlingFixtureAccess::Angle(*V, 3000.f), .03454f, 1.e-6f));
    return true;
}
#endif
