#if WITH_DEV_AUTOMATION_TESTS
#include "SpeedMovementComponent.h"
#include "Misc/AutomationTest.h"
#include "Math/RandomStream.h"
#include "HAL/PlatformTime.h"
#include "UObject/StrongObjectPtr.h"

namespace
{
	SKinematic OriginalOrigin(const SKinematic& COM, const FVector& LocalCOM)
	{
		SKinematic Origin = COM;
		const FVector Offset = COM.Rotation.RotateVector(LocalCOM);
		Origin.Location = COM.Location - Offset;
		Origin.Velocity = COM.Velocity - FVector::CrossProduct(COM.AngularVelocity, Offset);
		Origin.Acceleration = COM.Acceleration - FVector::CrossProduct(COM.AngularAcceleration, Offset)
			- FVector::CrossProduct(COM.AngularVelocity, FVector::CrossProduct(COM.AngularVelocity, Offset));
		return Origin;
	}
	bool SameVectorBits(const FVector& A, const FVector& B)
	{
		return FMemory::Memcmp(&A, &B, sizeof(FVector)) == 0;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedOriginGetterTest,
	"IAmSpeed.Simulation.OriginGetterBits",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedOriginGetterTest::RunTest(const FString& Parameters)
{
	TStrongObjectPtr<USpeedMovementComponent> Component(NewObject<USpeedMovementComponent>());
	FRandomStream Random(150940);
	for (int32 Sample = 0; Sample < 1024; ++Sample)
	{
		const auto Vector = [&]()
		{
			return Sample % 13 == 0 ? FVector(-0.0, 0.0, -0.0) : FVector(Random.FRandRange(-10000.f, 10000.f),
				Random.FRandRange(-10000.f, 10000.f), Random.FRandRange(-10000.f, 10000.f));
		};
		SKinematic State;
		State.Location = Vector(); State.Velocity = Vector(); State.Acceleration = Vector();
		State.AngularVelocity = Vector(); State.AngularAcceleration = Vector();
		State.Rotation = FRotator(Random.FRandRange(-180.f, 180.f), Random.FRandRange(-180.f, 180.f),
			Random.FRandRange(-180.f, 180.f)).Quaternion();
		if (Sample % 11 == 0) State.Rotation *= .8;
		Component->CenterOfMass = Vector();
		Component->BaseGameState.NumFrame = 0;
		Component->RecordedBaseFrames[0] = INDEX_NONE;
		Component->SetKinematicState(State);
		const SKinematic Expected = OriginalOrigin(State, Component->CenterOfMass);
		for (int32 History = 0; History < 2; ++History)
		{
			if (History)
			{
				Component->RecordPhysicsState();
				SKinematic Poison; Poison.Location = FVector(1.e9); Poison.Velocity = FVector(-1.e9);
				Component->SetKinematicState(Poison);
			}
			const SKinematic Actual = Component->GetOriginKinematicState();
			if (!TestTrue(TEXT("complete reconstruction retains the original fields and operation order"),
				SameVectorBits(Expected.Location, Actual.Location) && SameVectorBits(Expected.Velocity, Actual.Velocity) &&
				SameVectorBits(Expected.Acceleration, Actual.Acceleration) &&
				SameVectorBits(Expected.AngularVelocity, Actual.AngularVelocity) &&
				SameVectorBits(Expected.AngularAcceleration, Actual.AngularAcceleration) &&
				FMemory::Memcmp(&Expected.Rotation, &Actual.Rotation, sizeof(FQuat)) == 0)) return false;
			if (!TestTrue(TEXT("partial getters preserve bits and prefer recorded current-frame history"),
				SameVectorBits(Expected.Location, Component->GetPhysLocation()) &&
				SameVectorBits(Expected.Velocity, Component->GetPhysVelocity()))) return false;
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedOriginGetterCostTest,
	"IAmSpeed.Simulation.OriginGetterCost",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedOriginGetterCostTest::RunTest(const FString& Parameters)
{
	TStrongObjectPtr<USpeedMovementComponent> Component(NewObject<USpeedMovementComponent>());
	Component->CenterOfMass = FVector(-16, 0, 22);
	SKinematic State;
	State.Location = FVector(123, 456, 789); State.Velocity = FVector(2000, -1000, 500);
	State.Acceleration = FVector(30, -50, 70); State.Rotation = FRotator(15, 30, -60).Quaternion();
	State.AngularVelocity = FVector(1, 2, 3); State.AngularAcceleration = FVector(-5, -6, 2);
	Component->SetKinematicState(State);
	constexpr int32 Calls = 131072;
	double Sums[2] = {};
	const auto Measure = [&](bool bFull)
	{
		double Sum = 0;
		const double Start = FPlatformTime::Seconds();
		if (bFull)
			for (int32 Index = 0; Index < Calls; ++Index) Sum += Component->GetOriginKinematicState().Location.X;
		else
			for (int32 Index = 0; Index < Calls; ++Index) Sum += Component->GetPhysLocation().X;
		const double Elapsed = FPlatformTime::Seconds() - Start;
		Sums[bFull ? 1 : 0] = Sum;
		return 1.e9 * Elapsed / Calls;
	};
	Measure(false); Measure(true);
	for (int32 Pair = 0; Pair < 5; ++Pair)
	{
		double Full, Partial;
		if (Pair % 2 == 0) { Full = Measure(true); Partial = Measure(false); }
		else { Partial = Measure(false); Full = Measure(true); }
		TestEqual(TEXT("paired getter checksums match"), Sums[0], Sums[1]);
		AddInfo(FString::Printf(TEXT("[OriginGetterMicro] Pair=%d Calls=%d FullNs=%.6f LocationOnlyNs=%.6f"),
			Pair, Calls, Full, Partial));
	}
	return true;
}
#endif
