#if WITH_DEV_AUTOMATION_TESTS
#include "KinematicQuantizationCache.h"
#include "Misc/AutomationTest.h"
#include "Math/RandomStream.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedKinematicQuantizationTest,
	"IAmSpeed.Simulation.IdentityKinematicQuantizationCache",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedKinematicQuantizationTest::RunTest(const FString& Parameters)
{
	Speed::FIdentityKinematicQuantizationCache Cache;
	const auto Check = [this, &Cache](Speed::FKinematicState State, const FQuat& PreviousRotation)
	{
		Speed::FKinematicState Expected = State;
		Expected.Quantize(PreviousRotation); // Uncached outer operation; independent of this no-op cache.
		const bool bReused = Cache.Quantize(State, PreviousRotation);
		const auto EqualVector = [](const FVector& A, const FVector& B)
		{
			return FMemory::Memcmp(&A, &B, sizeof(FVector)) == 0;
		};
		TestTrue(TEXT("all fields match the canonical quantizer bit for bit"),
			EqualVector(Expected.Location, State.Location) && EqualVector(Expected.Velocity, State.Velocity) &&
			EqualVector(Expected.AngularVelocity, State.AngularVelocity) &&
			EqualVector(Expected.Acceleration, State.Acceleration) &&
			EqualVector(Expected.AngularAcceleration, State.AngularAcceleration) &&
			FMemory::Memcmp(&Expected.Rotation, &State.Rotation, sizeof(FQuat)) == 0);
		return bReused;
	};
	Speed::FKinematicState State;
	// Build an exact point-fix rather than assuming the identity quaternion has the quantizer's signed zeros.
	State.Quantize(FQuat::Identity);
	const FQuat Previous = State.Rotation;
	State.Quantize(Previous);
	TestFalse(TEXT("first encounter is evaluated"), Check(State, Previous));
	TestTrue(TEXT("proven no-op is reused"), Check(State, Previous));
	State.Acceleration = FVector(1.25, -4.75, 650.0);
	State.AngularAcceleration = FVector(-1.0, 2.0, .125);
	TestTrue(TEXT("unquantized accelerations remain live on reuse"), Check(State, Previous));
	Cache.Reset();
	TestFalse(TEXT("restore invalidation forces reevaluation"), Check(State, Previous));
	Check(State, FQuat(-Previous.X, -Previous.Y, -Previous.Z, -Previous.W));
	State.Location.X = -0.0;
	Check(State, Previous);
	for (double Boundary : { -.005000001, -.005, -.004999999, .004999999, .005, .005000001 })
	{
		State.Location.X = Boundary;
		State.Velocity.Z = Boundary;
		State.AngularVelocity.Y = Boundary * .01;
		Check(State, Previous);
	}
	FRandomStream Random(0x1de17);
	// Missing history may return the live state itself, so the reference can alias Rotation.
	Speed::FKinematicState Aliased;
	Aliased.Rotation = FRotator(13.7, -162.3, 8.1).Quaternion();
	for (int32 Repeat = 0; Repeat < 10; ++Repeat)
	{
		Speed::FKinematicState Expected = Aliased;
		Expected.Quantize(Expected.Rotation);
		Cache.Quantize(Aliased, Aliased.Rotation);
		TestTrue(TEXT("live rotation reference alias is preserved"),
			FMemory::Memcmp(&Expected.Rotation, &Aliased.Rotation, sizeof(FQuat)) == 0);
	}
	for (int32 Index = 0; Index < 1000; ++Index)
	{
		State.Location = Random.VRand() * 7000.0;
		State.Velocity = Random.VRand() * 2000.0;
		State.AngularVelocity = Random.VRand() * 5.5;
		State.Rotation = FRotator(Random.FRandRange(-180.f, 180.f),
			Random.FRandRange(-180.f, 180.f), Random.FRandRange(-180.f, 180.f)).Quaternion();
		const FQuat Reference = State.Rotation;
		for (int32 Repeat = 0; Repeat < 4; ++Repeat)
		{
			Check(State, Reference);
			State.Quantize(Reference);
		}
	}
	return true;
}
#endif
