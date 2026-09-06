#if WITH_DEV_AUTOMATION_TESTS
#include "SUtils.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedSmallRotationIntegrationTest,
	"IAmSpeed.PhysicalLaws.Integration.SmallRotationContinuity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedSmallRotationIntegrationTest::RunTest(const FString& Parameters)
{
	const FVector Axis = FVector(1, -2, 3).GetSafeNormal();
	for (double Angle : { 1.e-3, 1.e-7, 1.1e-8, 9.e-9, 1.e-10, 1.e-14 })
	{
		const FVector W = Axis * Angle;
		const FQuat Q = Speed::SBox::IntegrateRotation(FQuat::Identity, W, FVector::ZeroVector, 1.f);
		const FQuat Expected(Axis, Angle);
		TestTrue(TEXT("every representable finite rotation advances the orientation"), Q.X != 0 && Q.Y != 0 && Q.Z != 0);
		TestTrue(TEXT("exponential map remains accurate on both sides of the former cutoff"),
			FMath::Abs(Q.X - Expected.X) <= 16 * DBL_EPSILON * Angle &&
			FMath::Abs(Q.Y - Expected.Y) <= 16 * DBL_EPSILON * Angle &&
			FMath::Abs(Q.Z - Expected.Z) <= 16 * DBL_EPSILON * Angle);
		const FQuat Half = Speed::SBox::IntegrateRotation(FQuat::Identity, W, FVector::ZeroVector, .5f);
		const FQuat Split = Speed::SBox::IntegrateRotation(Half, W, FVector::ZeroVector, .5f);
		TestTrue(TEXT("splitting a tiny constant spin does not suppress its motion"), Split.Equals(Q, 16 * DBL_EPSILON));
	}
	const FQuat Initial = FRotator(20, 40, 80).Quaternion();
	TestTrue(TEXT("exact zero motion preserves the original quaternion"),
		Speed::SBox::IntegrateRotation(Initial, FVector::ZeroVector, FVector::ZeroVector, 1.f) == Initial);
	return true;
}
#endif
