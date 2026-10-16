#if WITH_DEV_AUTOMATION_TESTS
#include "SpeedWheeledSteeringMath.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedUnsteeredAligningYawAuthorityTest,
	"IAmSpeed.PhysicalLaws.Steering.UnsteeredAligningYawAuthority",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedUnsteeredAligningYawAuthorityTest::RunTest(const FString& Parameters)
{
	constexpr float TargetYawRate = 1.5f;
	constexpr float CurrentYawRate = -2.0f;
	constexpr float TimeConstant = 0.006f;
	const float FullCorrection = (TargetYawRate - CurrentYawRate) / TimeConstant;

	TestEqual(TEXT("zero slip authority applies no yaw correction"),
		IAmSpeedSteering::ComputeUnsteeredAligningYawAcceleration(
			TargetYawRate, CurrentYawRate, TimeConstant, 0.0f),
		0.0f);
	TestEqual(TEXT("negative slip authority cannot reintroduce yaw braking"),
		IAmSpeedSteering::ComputeUnsteeredAligningYawAcceleration(
			TargetYawRate, CurrentYawRate, TimeConstant, -0.1f),
		0.0f);

	constexpr float LowAuthority = 0.0001f;
	const float LowCorrection =
		IAmSpeedSteering::ComputeUnsteeredAligningYawAcceleration(
			TargetYawRate, CurrentYawRate, TimeConstant, LowAuthority);
	TestTrue(TEXT("low authority remains finite"), FMath::IsFinite(LowCorrection));
	TestTrue(TEXT("low authority scales the entire correction continuously"),
		FMath::IsNearlyEqual(LowCorrection, FullCorrection * LowAuthority, 1.e-5f));

	TestTrue(TEXT("full authority preserves the published correction"),
		FMath::IsNearlyEqual(
			IAmSpeedSteering::ComputeUnsteeredAligningYawAcceleration(
				TargetYawRate, CurrentYawRate, TimeConstant, 1.0f),
			FullCorrection, 1.e-4f));
	TestTrue(TEXT("authority above one remains bounded to full correction"),
		FMath::IsNearlyEqual(
			IAmSpeedSteering::ComputeUnsteeredAligningYawAcceleration(
				TargetYawRate, CurrentYawRate, TimeConstant, 1.1f),
			FullCorrection, 1.e-4f));

	return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedResidualYawDampingTest,
    "IAmSpeed.PhysicalLaws.Steering.UnsteeredAligningYawResidualDamping",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedResidualYawDampingTest::RunTest(const FString& Parameters)
{
    for (const float Sign : {-1.0f, 1.0f})
    {
        const float Current = Sign * 1.0f;
        const float Delta = 1.0f / 300.0f;
        const float Alpha = IAmSpeedSteering::ComputeUnsteeredAligningYawAcceleration(
            0.0f, Current, 0.006f, 0.0f, true);
        const float Next = Current + Alpha * Delta;
        TestTrue(TEXT("zero-slip residual decays without reversing sign"),
            Next * Current > 0.0f && FMath::Abs(Next) < FMath::Abs(Current));
        TestEqual(TEXT("existing default keeps its zero-authority behavior"),
            IAmSpeedSteering::ComputeUnsteeredAligningYawAcceleration(
                0.0f, Current, 0.006f, 0.0f), 0.0f);
    }
    for (const float Authority : {0.0f, 0.25f, 0.5f, 1.0f})
    {
        const float Target = 1.5f * Authority;
        for (const float Delta : {1.0f / 300.0f, 1.0f / 120.0f})
        {
            const float Time = FMath::Max(Delta, 0.006f);
            const float Alpha = IAmSpeedSteering::ComputeUnsteeredAligningYawAcceleration(
                Target, -2.0f, Time, Authority, true);
            const float Mirror = IAmSpeedSteering::ComputeUnsteeredAligningYawAcceleration(
                -Target, 2.0f, Time, Authority, true);
            TestTrue(TEXT("mirrored yaw acceleration preserves sign symmetry"),
                FMath::IsNearlyEqual(Alpha, -Mirror, 1.e-4f));
            const float Next = -2.0f + Alpha * Delta;
            TestTrue(TEXT("bounded integration approaches the slip-scaled target without overshoot"),
                Next >= -2.0f && Next <= Target + 1.e-5f);
        }
    }
    TestEqual(TEXT("full authority is identical in both modes"),
        IAmSpeedSteering::ComputeUnsteeredAligningYawAcceleration(1.5f, -2.0f, 0.006f, 1.0f),
        IAmSpeedSteering::ComputeUnsteeredAligningYawAcceleration(1.5f, -2.0f, 0.006f, 1.0f, true));
    return !HasAnyErrors();
}
#endif
