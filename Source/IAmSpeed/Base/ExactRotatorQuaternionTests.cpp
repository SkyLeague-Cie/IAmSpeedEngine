#if WITH_DEV_AUTOMATION_TESTS
#include "ExactRotatorQuaternionCache.h"
#include "Misc/AutomationTest.h"
#include "Math/RandomStream.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedExactRotatorQuaternionTest,
	"IAmSpeed.Simulation.ExactRotatorQuaternionCache",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedExactRotatorQuaternionTest::RunTest(const FString& Parameters)
{
	Speed::FExactRotatorQuaternionCache Cache;
	const auto Check = [this, &Cache](const FRotator& Rotation)
	{
		const FQuat Expected = Rotation.Quaternion();
		for (int32 Repeat = 0; Repeat < 3; ++Repeat)
		{
			const FQuat Actual = Cache.Get(Rotation);
			TestTrue(TEXT("cached Euler conversion preserves every quaternion bit"),
				FMemory::Memcmp(&Expected, &Actual, sizeof(FQuat)) == 0);
		}
	};
	Check(FRotator(0.0, 0.0, 0.0));
	Check(FRotator(-0.0, 0.0, -0.0));
	Check(FRotator(0.0, -0.0, 0.0));
	for (double Angle : { -1000000.0, -720.0, -180.001, -180.0, -90.0, 89.999, 90.0, 180.0, 360.0, 1000000.0 })
	{
		Check(FRotator(Angle, 13.0, 27.0));
		Check(FRotator(13.0, Angle, 27.0));
		Check(FRotator(13.0, 27.0, Angle));
	}
	FRandomStream Random(0x5a71c);
	for (int32 Index = 0; Index < 1000; ++Index)
	{
		const FRotator Rotation(Random.FRandRange(-720.f, 720.f),
			Random.FRandRange(-720.f, 720.f), Random.FRandRange(-720.f, 720.f));
		Check(Rotation);
		Check(FRotator(Rotation.Pitch + 1.e-10, Rotation.Yaw, Rotation.Roll));
		for (int32 Eviction = 0; Eviction < 5; ++Eviction)
			Check(FRotator(17.0 * Eviction, 97.0 * Eviction, Index * .17));
		Check(Rotation);
	}
	return true;
}
#endif
