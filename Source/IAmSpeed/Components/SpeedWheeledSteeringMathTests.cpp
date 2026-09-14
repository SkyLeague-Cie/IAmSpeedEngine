#if WITH_DEV_AUTOMATION_TESTS
#include "SpeedWheeledSteeringMath.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedUnsteeredAligningYawAuthorityTest,
	"IAmSpeed.PhysicalLaws.Steering.UnsteeredAligningYawAuthority",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedUnsteeredAligningYawAuthorityTest::RunTest(const FString& Parameters)
{
	using namespace IAmSpeedSteering;
	constexpr float DeltaTime = 1.0f / 300.0f;
	constexpr float TimeConstant = 0.006f;
	constexpr float MinSlipSpeed = 300.0f;
	constexpr float FullSlipSpeed = 600.0f;
	constexpr float MaxRate = 1.5f;

	for (const float CurrentYawRate : {-2.0f, 2.0f})
	{
		const FUnsteeredAligningYawResponse Response =
			ComputeUnsteeredAligningYawResponse(0.0f, MinSlipSpeed,
				FullSlipSpeed, MaxRate, CurrentYawRate, TimeConstant, DeltaTime);
		TestEqual(TEXT("zero slip has zero target authority"), Response.SlipAuthority, 0.0f);
		TestTrue(TEXT("zero-authority settling opposes residual yaw"),
			Response.AngularAcceleration * CurrentYawRate < 0.0f);
		const float NextYawRate = CurrentYawRate + Response.AngularAcceleration * DeltaTime;
		TestTrue(TEXT("bounded settling reduces yaw magnitude"),
			FMath::Abs(NextYawRate) < FMath::Abs(CurrentYawRate));
		TestTrue(TEXT("bounded settling cannot reverse yaw sign"),
			NextYawRate * CurrentYawRate >= 0.0f);
	}

	const FUnsteeredAligningYawResponse Rest =
		ComputeUnsteeredAligningYawResponse(0.0f, MinSlipSpeed,
			FullSlipSpeed, MaxRate, 0.0f, TimeConstant, DeltaTime);
	TestEqual(TEXT("zero yaw remains zero"), Rest.AngularAcceleration, 0.0f);

	const float Epsilon = 0.01f;
	const FUnsteeredAligningYawResponse Below =
		ComputeUnsteeredAligningYawResponse(MinSlipSpeed - Epsilon,
			MinSlipSpeed, FullSlipSpeed, MaxRate, 0.4f, TimeConstant, DeltaTime);
	const FUnsteeredAligningYawResponse At =
		ComputeUnsteeredAligningYawResponse(MinSlipSpeed,
			MinSlipSpeed, FullSlipSpeed, MaxRate, 0.4f, TimeConstant, DeltaTime);
	const FUnsteeredAligningYawResponse Above =
		ComputeUnsteeredAligningYawResponse(MinSlipSpeed + Epsilon,
			MinSlipSpeed, FullSlipSpeed, MaxRate, 0.4f, TimeConstant, DeltaTime);
	TestEqual(TEXT("authority is zero below the threshold"), Below.SlipAuthority, 0.0f);
	TestEqual(TEXT("authority is zero at the threshold"), At.SlipAuthority, 0.0f);
	TestTrue(TEXT("authority begins continuously above the threshold"),
		Above.SlipAuthority > 0.0f && Above.SlipAuthority < 0.001f);
	TestTrue(TEXT("acceleration is continuous at the threshold"),
		FMath::Abs(Above.AngularAcceleration - At.AngularAcceleration) < 0.01f);

	const FUnsteeredAligningYawResponse HighSlip =
		ComputeUnsteeredAligningYawResponse(FullSlipSpeed + 100.0f,
			MinSlipSpeed, FullSlipSpeed, MaxRate, 0.0f, TimeConstant, DeltaTime);
	TestEqual(TEXT("high slip retains full authority"), HighSlip.SlipAuthority, 1.0f);
	TestEqual(TEXT("positive side slip retains the signed high-slip target"),
		HighSlip.TargetYawRate, -MaxRate);

	const FUnsteeredAligningYawResponse Bounded =
		ComputeUnsteeredAligningYawResponse(0.0f, MinSlipSpeed,
			FullSlipSpeed, MaxRate, 1.0f, 0.0f, DeltaTime);
	TestEqual(TEXT("time constant is bounded by delta"),
		Bounded.AngularAcceleration, -1.0f / DeltaTime);
	TestEqual(TEXT("delta-bounded step reaches zero without reversing"),
		1.0f + Bounded.AngularAcceleration * DeltaTime, 0.0f);

	const FUnsteeredAligningYawResponse ZeroTime =
		ComputeUnsteeredAligningYawResponse(0.0f, MinSlipSpeed,
			FullSlipSpeed, MaxRate, 1.0f, 0.0f, 0.0f);
	TestTrue(TEXT("zero delta and time constant retain finite authority"),
		FMath::IsFinite(ZeroTime.SlipAuthority));
	TestTrue(TEXT("zero delta and time constant retain finite target"),
		FMath::IsFinite(ZeroTime.TargetYawRate));
	TestTrue(TEXT("zero delta and time constant retain finite acceleration"),
		FMath::IsFinite(ZeroTime.AngularAcceleration));

	const FUnsteeredAligningYawResponse InvalidThresholds =
		ComputeUnsteeredAligningYawResponse(2.0f, 10.0f,
			0.0f, MaxRate, 0.4f, TimeConstant, DeltaTime);
	TestTrue(TEXT("invalid thresholds normalize to finite authority"),
		FMath::IsFinite(InvalidThresholds.SlipAuthority));
	TestTrue(TEXT("invalid thresholds normalize to finite target"),
		FMath::IsFinite(InvalidThresholds.TargetYawRate));
	TestTrue(TEXT("invalid thresholds normalize to finite acceleration"),
		FMath::IsFinite(InvalidThresholds.AngularAcceleration));
	TestTrue(TEXT("invalid thresholds retain bounded authority"),
		InvalidThresholds.SlipAuthority >= 0.0f &&
		InvalidThresholds.SlipAuthority <= 1.0f);

	return !HasAnyErrors();
}
#endif
