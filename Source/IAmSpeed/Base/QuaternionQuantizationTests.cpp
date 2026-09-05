#if WITH_DEV_AUTOMATION_TESTS
#include "QuaternionQuantizationCache.h"
#include "Misc/AutomationTest.h"
#include "Math/RandomStream.h"

namespace
{
	// Frozen pre-cache implementation: never use the cached production function as its own oracle.
	FQuat QuantizeUncached(const FQuat& Quat, const FQuat& Reference)
	{
		FRotator R = Quat.Rotator();
		R.Normalize();
		R.Pitch = FRotator::NormalizeAxis(R.Pitch);
		R.Yaw = FRotator::NormalizeAxis(R.Yaw);
		R.Roll = FRotator::NormalizeAxis(R.Roll);
		const uint16 SP = FRotator::CompressAxisToShort(R.Pitch);
		const uint16 SY = FRotator::CompressAxisToShort(R.Yaw);
		const uint16 SR = FRotator::CompressAxisToShort(R.Roll);
		const float Pitch = FRotator::DecompressAxisFromShort(SP);
		const float Yaw = FRotator::DecompressAxisFromShort(SY);
		const float Roll = FRotator::DecompressAxisFromShort(SR);
		FQuat Out = FRotator(Pitch, Yaw, Roll).Quaternion();
		Out.Normalize();
		if (Speed::QuatDot(Out, Reference) < 0.f)
		{
			Out.X *= -1; Out.Y *= -1; Out.Z *= -1; Out.W *= -1;
		}
		return Out;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedQuaternionQuantizationTest,
	"IAmSpeed.Simulation.QuaternionQuantizationCache",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedQuaternionQuantizationTest::RunTest(const FString& Parameters)
{
	Speed::FQuaternionQuantizationCache Cache;
	const auto Check = [this, &Cache](const FQuat& Input, const FQuat& Reference)
	{
		const FQuat Expected = QuantizeUncached(Input, Reference);
		const FQuat Actual = Cache.Get(Input, Reference);
		TestTrue(TEXT("quantization remains bit-exact, including signed zero"),
			FMemory::Memcmp(&Expected, &Actual, sizeof(FQuat)) == 0);
		const FQuat Production = Speed::RoundQuatToNetQuantize(Input, Reference);
		TestTrue(TEXT("production conversion also matches the uncached oracle"),
			FMemory::Memcmp(&Expected, &Production, sizeof(FQuat)) == 0);
	};
	const auto CheckHemisphere = [&Check](const FQuat& Input)
	{
		Check(Input, FQuat::Identity);
		Check(Input, FQuat(0, 0, 0, -1));
		Check(Input, Input);
		Check(Input, FQuat(-Input.X, -Input.Y, -Input.Z, -Input.W));
	};
	CheckHemisphere(FQuat::Identity);
	CheckHemisphere(FQuat(-0.0, 0.0, -0.0, 1.0));
	CheckHemisphere(FQuat(0.0, -0.0, 0.0, -1.0));
	for (double Pitch : { -90.001, -90.0, -89.999, 0.0, 89.999, 90.0, 90.001 })
	{
		for (double Yaw : { -180.001, -180.0, -179.999, 0.0, 179.999, 180.0, 180.001 })
		{
			CheckHemisphere(FRotator(Pitch, Yaw, 37.123).Quaternion());
		}
	}
	FRandomStream Random(0x51eed);
	for (int32 Index = 0; Index < 1000; ++Index)
	{
		const FQuat Input = FRotator(Random.FRandRange(-180.f, 180.f),
			Random.FRandRange(-180.f, 180.f), Random.FRandRange(-180.f, 180.f)).Quaternion();
		CheckHemisphere(Input);
		// Same value after more unique inputs than slots must be recomputed without changing output.
		for (int32 Eviction = 0; Eviction < 5; ++Eviction)
			CheckHemisphere(FRotator(13.0 * Eviction, Index * .17, 73.0 * Eviction).Quaternion());
		CheckHemisphere(Input);
		FQuat Perturbed = Input;
		Perturbed.X += 1.e-12;
		CheckHemisphere(Perturbed);
	}
	return true;
}
#endif
