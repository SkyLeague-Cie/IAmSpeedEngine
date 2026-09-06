#if WITH_DEV_AUTOMATION_TESTS
#include "SUtils.h"
#include "Misc/AutomationTest.h"
#include "Math/RandomStream.h"
#include "HAL/PlatformTime.h"

namespace
{
	// Original complete signed-distance/witness calculation, independent of the
	// production optional-output path. Keep the face order and float conversions.
	float OriginalSeparation(const Speed::SBox& Box, const FQuat& Q, const FVector& X,
		const FVector& CS, float R, FVector& Contact)
	{
		const FVector Local = Q.UnrotateVector(CS - X);
		FVector Closest(FMath::Clamp(Local.X, Box.Min.X, Box.Max.X),
			FMath::Clamp(Local.Y, Box.Min.Y, Box.Max.Y), FMath::Clamp(Local.Z, Box.Min.Z, Box.Max.Z));
		const bool Inside = Local.X >= Box.Min.X && Local.X <= Box.Max.X &&
			Local.Y >= Box.Min.Y && Local.Y <= Box.Max.Y &&
			Local.Z >= Box.Min.Z && Local.Z <= Box.Max.Z;
		float Distance = 0.f;
		if (Inside)
		{
			const float Faces[6] = { Local.X - Box.Min.X, Box.Max.X - Local.X,
				Local.Y - Box.Min.Y, Box.Max.Y - Local.Y, Local.Z - Box.Min.Z, Box.Max.Z - Local.Z };
			int32 Nearest = 0;
			for (int32 Face = 1; Face < 6; ++Face)
				if (Faces[Face] < Faces[Nearest]) Nearest = Face;
			switch (Nearest)
			{
			case 0: Closest.X = Box.Min.X; break;
			case 1: Closest.X = Box.Max.X; break;
			case 2: Closest.Y = Box.Min.Y; break;
			case 3: Closest.Y = Box.Max.Y; break;
			case 4: Closest.Z = Box.Min.Z; break;
			default: Closest.Z = Box.Max.Z; break;
			}
			Distance = -Faces[Nearest];
		}
		else Distance = FVector::Distance(Local, Closest);
		Contact = Q.RotateVector(Closest) + X;
		return Distance - R;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedSeparationWitnessTest,
	"IAmSpeed.Geometry.Solid.SeparationWitnessBits",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedSeparationWitnessTest::RunTest(const FString& Parameters)
{
	FRandomStream Random(150938);
	for (int32 Sample = 0; Sample < 1024; ++Sample)
	{
		const FVector Extent(Random.FRandRange(.01f, 200.f), Random.FRandRange(.01f, 200.f),
			Random.FRandRange(.01f, 200.f));
		const FVector Origin(Random.FRandRange(-10000.f, 10000.f), Random.FRandRange(-10000.f, 10000.f),
			Random.FRandRange(-10000.f, 10000.f));
		FQuat Rotation = FRotator(Random.FRandRange(-180.f, 180.f), Random.FRandRange(-180.f, 180.f),
			Random.FRandRange(-180.f, 180.f)).Quaternion();
		if (Sample % 7 == 0) Rotation *= .7; // optional output must not assume a unit quaternion
		const Speed::SBox Box(Origin, Extent, Rotation, FVector::ZeroVector, FVector::ZeroVector,
			FVector::ZeroVector, FVector::ZeroVector);
		for (const FVector& Local : { FVector(-0.0, 0.0, -0.0), Extent, -Extent,
			Extent * .5, Extent * 2.0, FVector(Extent.X, 0, 0), FVector(0, -Extent.Y, 0),
			FVector(0, 0, Extent.Z), FVector(Random.FRandRange(-500.f, 500.f),
				Random.FRandRange(-500.f, 500.f), Random.FRandRange(-500.f, 500.f)) })
		{
			const FVector Center = Rotation.RotateVector(Local) + Origin;
			const float Radius = Sample % 5 == 0 ? -0.f : Random.FRandRange(.001f, 100.f);
			FVector ExpectedPoint, ActualPoint;
			const float Expected = OriginalSeparation(Box, Rotation, Origin, Center, Radius, ExpectedPoint);
			const float WithWitness = Box.SphereOBBSeparation(Rotation, Origin, Center, Radius, &ActualPoint);
			const float DistanceOnly = Box.SphereOBBSeparation(Rotation, Origin, Center, Radius);
			if (!TestTrue(TEXT("optional witness retains original separation bits"),
				FMemory::Memcmp(&Expected, &WithWitness, sizeof(float)) == 0 &&
				FMemory::Memcmp(&Expected, &DistanceOnly, sizeof(float)) == 0)) return false;
			if (!TestTrue(TEXT("requested witness retains every original coordinate bit"),
				FMemory::Memcmp(&ExpectedPoint, &ActualPoint, sizeof(FVector)) == 0)) return false;
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedSeparationWitnessCostTest,
	"IAmSpeed.Geometry.Solid.SeparationWitnessCost",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedSeparationWitnessCostTest::RunTest(const FString& Parameters)
{
	// Paired diagnostic, not a timing pass/fail assertion. Both paths use the
	// same production function: only the now-unneeded optional world witness differs.
	const Speed::SBox Box(FVector(12, -34, 56), FVector(60, 45, 20), FRotator(17, 31, -9).Quaternion(),
		FVector::ZeroVector, FVector::ZeroVector, FVector::ZeroVector, FVector::ZeroVector);
	TArray<FVector> Centers;
	FRandomStream Random(150939);
	for (int32 Index = 0; Index < 1024; ++Index)
		Centers.Emplace(Random.FRandRange(-10000.f, 10000.f), Random.FRandRange(-10000.f, 10000.f),
			Random.FRandRange(-500.f, 500.f));
	constexpr int32 Calls = 262144;
	float Sums[2] = {};
	const auto Measure = [&](bool bWitness)
	{
		FVector Point;
		FVector* Output = bWitness ? &Point : nullptr;
		float Sum = 0.f;
		const double Start = FPlatformTime::Seconds();
		for (int32 Index = 0; Index < Calls; ++Index)
			Sum += Box.SphereOBBSeparation(Box.Rot, Box.WorldCenter, Centers[Index & 1023], 15.f, Output);
		const double Elapsed = FPlatformTime::Seconds() - Start;
		Sums[bWitness ? 1 : 0] = Sum;
		return 1.e9 * Elapsed / Calls;
	};
	Measure(false); Measure(true); // warm both paths outside reported samples
	for (int32 Pair = 0; Pair < 5; ++Pair)
	{
		double WithWitness, DistanceOnly;
		if (Pair % 2 == 0) { WithWitness = Measure(true); DistanceOnly = Measure(false); }
		else { DistanceOnly = Measure(false); WithWitness = Measure(true); }
		TestEqual(TEXT("paired distance checksums remain identical"), Sums[0], Sums[1]);
		AddInfo(FString::Printf(TEXT("[SeparationWitnessMicro] Pair=%d Calls=%d WithWitnessNs=%.6f DistanceOnlyNs=%.6f"),
			Pair, Calls, WithWitness, DistanceOnly));
	}
	return true;
}
#endif
