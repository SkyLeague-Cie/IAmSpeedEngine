#if WITH_DEV_AUTOMATION_TESTS
#include "AnalyticBoxSweepContext.h"
#include "Misc/AutomationTest.h"
#include "Math/RandomStream.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedBoundsProjectionCacheTest,
	"IAmSpeed.AnalyticWorld.BoundsProjectionCache",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedBoundsProjectionCacheTest::RunTest(const FString& Parameters)
{
	using namespace Speed::Analytic;
	Private::FBoxBoundsSweepContext Context;
	FRandomStream Random(51832);
	const FVector3d WorldAxes[] = { FVector3d::ForwardVector, FVector3d::RightVector, FVector3d::UpVector };
	for (int32 Sample = 0; Sample < 1200; ++Sample)
	{
		FWorldQuery Query;
		Query.Shape = EQueryShape::Box;
		Query.Start = FVector3d(Random.FRandRange(-10000, 10000), Random.FRandRange(-10000, 10000), Random.FRandRange(-10000, 10000));
		Query.End = Query.Start + FVector3d(Random.FRandRange(-1000, 1000), Random.FRandRange(-1000, 1000), Random.FRandRange(-1000, 1000));
		Query.HalfExtent = FVector3d(Random.FRandRange(-1, 100), Random.FRandRange(-1, 100), Random.FRandRange(-1, 100));
		Query.DomainTolerance = Sample % 3 == 0 ? 0 : 0.001;
		Query.Rotation = FRotator3d(Random.FRandRange(-180, 180), Random.FRandRange(-180, 180), Random.FRandRange(-180, 180)).Quaternion();
		if (Sample % 4 == 0) Query.Rotation = FQuat4d::Identity;
		if (Sample % 4 == 1) Query.Rotation = FQuat4d(FVector3d::UpVector, 1.0e-13);
		if (Sample % 5 == 0) Query.End = Query.Start;
		const FVector3d CachedAxes[] = { Query.Rotation.RotateVector(WorldAxes[0]).GetSafeNormal(),
			Query.Rotation.RotateVector(WorldAxes[1]).GetSafeNormal(), Query.Rotation.RotateVector(WorldAxes[2]).GetSafeNormal() };
		Context.Initialize(Query, Sample % 2 == 0 ? CachedAxes : nullptr);
		for (int32 Repeat = 0; Repeat < 3; ++Repeat)
		{
			for (int32 Visit = 0; Visit < 9; ++Visit)
			{
				const int32 Index = Repeat % 2 == 0 ? Visit : 8 - Visit;
				const int32 BoxAxis = Index / 3;
				const int32 WorldAxis = Index % 3;
				const auto& Actual = Context.GetCrossAxis(BoxAxis, WorldAxis);
				// Original per-node scalar expressions, independent of the cached values.
				const FVector3d Axis = FVector3d::CrossProduct(Context.Axis[BoxAxis], WorldAxes[WorldAxis]);
				const double AxisLengthSquared = Axis.SquaredLength();
				const bool bDegenerate = AxisLengthSquared <= 1.0e-24;
				if (!TestTrue(TEXT("cross axis and degeneracy preserve original bits"),
					FMemory::Memcmp(&Axis, &Actual.Axis, sizeof(Axis)) == 0 && bDegenerate == Actual.bDegenerate)) return false;
				if (bDegenerate) continue;
				const double BoxRadius =
					Context.HalfExtent.X * FMath::Abs(FVector3d::DotProduct(Context.Axis[0], Axis)) +
					Context.HalfExtent.Y * FMath::Abs(FVector3d::DotProduct(Context.Axis[1], Axis)) +
					Context.HalfExtent.Z * FMath::Abs(FVector3d::DotProduct(Context.Axis[2], Axis));
				const double MotionDistance = FVector3d::DotProduct(Context.Motion, Axis);
				const double ToleranceRadius = Context.DomainTolerance * FMath::Sqrt(AxisLengthSquared);
				if (!TestTrue(TEXT("fresh, repeated and reinitialized projections preserve every bit"),
					FMemory::Memcmp(&BoxRadius, &Actual.BoxRadius, sizeof(double)) == 0 &&
					FMemory::Memcmp(&MotionDistance, &Actual.MotionDistance, sizeof(double)) == 0 &&
					FMemory::Memcmp(&ToleranceRadius, &Actual.ToleranceRadius, sizeof(double)) == 0)) return false;
			}
		}
	}
	return true;
}
#endif
