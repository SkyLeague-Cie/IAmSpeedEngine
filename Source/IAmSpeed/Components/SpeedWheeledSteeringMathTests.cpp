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
#endif
