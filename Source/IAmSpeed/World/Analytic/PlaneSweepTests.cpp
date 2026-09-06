#if WITH_DEV_AUTOMATION_TESTS
#include "AnalyticWorldQuery.h"
#include "Misc/AutomationTest.h"
#include "Math/RandomStream.h"
#include "HAL/PlatformTime.h"

namespace
{
	using namespace Speed::Analytic;

	// Independent copy of the previous ray/sphere control flow. This fixture
	// has no coplanar union; union arbitration remains covered by world tests.
	FORCENOINLINE FWorldHit OriginalRoundPlane(const FWorldQuery& Query, const FBoundedPlane& Plane)
	{
		FWorldHit Hit;
		FVector3d Normal = Plane.Normal;
		double RawStartDistance = Plane.SignedDistance(Query.Start);
		if (RawStartDistance < 0.0)
		{
			Normal = -Normal;
			RawStartDistance = -RawStartDistance;
		}
		const double Support = Query.Shape == EQueryShape::Sphere ? FMath::Max(0.0, Query.Radius) : 0.0;
		const double StartDistance = RawStartDistance - Support;
		const double EndDistance = FVector3d::DotProduct(Query.End - Plane.Origin, Normal) - Support;
		double Time = 1.0;
		if (StartDistance <= 0.0)
		{
			Time = 0.0;
			Hit.bStartPenetrating = StartDistance < 0.0;
			Hit.PenetrationDepth = FMath::Max(0.0, -StartDistance);
		}
		else
		{
			const double Denominator = StartDistance - EndDistance;
			if (Denominator <= 0.0 || EndDistance > 0.0) return Hit;
			Time = StartDistance / Denominator;
			if (Time < 0.0 || Time > 1.0) return Hit;
		}
		const FVector3d Center = FMath::Lerp(Query.Start, Query.End, Time);
		const FVector3d ContactPoint = Center - Support * Normal;
		if (!Plane.ContainsProjectedPoint(ContactPoint, Query.DomainTolerance)) return FWorldHit();
		if (Query.bAuthorityOnly && !Query.bAllowEstablishedFaceContactAtBoundary)
		{
			double RequiredInteriorMargin = 0.0;
			if (Query.Shape == EQueryShape::Sphere) RequiredInteriorMargin = FMath::Max(0.0, Query.Radius);
			const double DistanceToAuthorityBoundary = Plane.DistanceToDomainBoundary(ContactPoint);
			if (DistanceToAuthorityBoundary + Query.DomainTolerance < RequiredInteriorMargin) return FWorldHit();
		}
		Hit.bHit = true;
		Hit.Time = Time;
		Hit.Location = Center;
		Hit.Point = Plane.ClosestPoint(ContactPoint);
		Hit.QueryPoint = ContactPoint;
		Hit.Normal = Normal;
		Hit.QueryFeatureKind = Query.Shape == EQueryShape::Sphere ? EContactFeatureKind::Vertex : EContactFeatureKind::Unknown;
		Hit.SurfaceFeatureKind = EContactFeatureKind::Face;
		Hit.SourceId = Plane.SourceId; Hit.SurfaceId = Plane.SurfaceId; Hit.FeatureId = Plane.FeatureId;
		Hit.PrimitiveId = Plane.PrimitiveId; Hit.MaterialId = Plane.MaterialId;
		return Hit;
	}

	bool SameHitBits(const FWorldHit& A, const FWorldHit& B)
	{
		const auto Bits = [](const auto& X, const auto& Y) { return FMemory::Memcmp(&X, &Y, sizeof(X)) == 0; };
		return A.bHit == B.bHit && A.bStartPenetrating == B.bStartPenetrating &&
			Bits(A.Time, B.Time) && Bits(A.PenetrationDepth, B.PenetrationDepth) &&
			Bits(A.GeometricErrorBoundCm, B.GeometricErrorBoundCm) &&
			Bits(A.AdditionalResidualAgreementAllowanceCm, B.AdditionalResidualAgreementAllowanceCm) &&
			A.bSurfaceNormalMayVary == B.bSurfaceNormalMayVary &&
			Bits(A.Location, B.Location) && Bits(A.Point, B.Point) && Bits(A.QueryPoint, B.QueryPoint) && Bits(A.Normal, B.Normal) &&
			A.QueryFeatureKind == B.QueryFeatureKind && A.SurfaceFeatureKind == B.SurfaceFeatureKind &&
			A.QueryFeatureIndex == B.QueryFeatureIndex && A.SurfaceFeatureIndex == B.SurfaceFeatureIndex &&
			A.SourceId == B.SourceId && A.SurfaceId == B.SurfaceId && A.FeatureId == B.FeatureId &&
			A.PrimitiveId == B.PrimitiveId && A.CanonicalGroupId == B.CanonicalGroupId && A.MaterialId == B.MaterialId;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedPlaneSweepTest,
	"IAmSpeed.AnalyticWorld.RoundPlaneMiss", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedPlaneSweepTest::RunTest(const FString& Parameters)
{
	using namespace Speed::Analytic;
	FRandomStream Random(150953);
	int32 Hits = 0, Misses = 0;
	FWorldHit Scratch;
	for (int32 Sample = 0; Sample < 4096; ++Sample)
	{
		FBoundedPlane Plane;
		const FQuat4d Rotation = FRotator3d(Random.FRandRange(-180.f, 180.f),
			Random.FRandRange(-180.f, 180.f), Random.FRandRange(-180.f, 180.f)).Quaternion();
		Plane.Origin = FVector3d(Random.FRandRange(-100.f, 100.f), Random.FRandRange(-100.f, 100.f), Random.FRandRange(-100.f, 100.f));
		Plane.AxisU = Rotation.RotateVector(FVector3d::ForwardVector);
		Plane.AxisV = Rotation.RotateVector(FVector3d::RightVector);
		Plane.Normal = Rotation.RotateVector(FVector3d::UpVector);
		Plane.HalfExtents = FVector2d(20.0, 10.0);
		Plane.SourceId = 11; Plane.SurfaceId = 12; Plane.FeatureId = 13; Plane.PrimitiveId = 14; Plane.MaterialId = 15;
		if (Sample % 3 == 0)
			Plane.DomainVertices = { FVector2d(-20, -10), FVector2d(20, -10), FVector2d(20, 10), FVector2d(-20, 10) };
		FWorldQuery Query;
		Query.Shape = Sample % 2 ? EQueryShape::Sphere : EQueryShape::Ray;
		Query.Radius = Random.FRandRange(-1.f, 6.f);
		Query.DomainTolerance = Sample % 4 ? 1.e-6 : 0.0;
		Query.bAuthorityOnly = Sample % 5 == 0;
		Query.bAllowEstablishedFaceContactAtBoundary = Sample % 7 == 0;
		const FVector3d Target = Plane.Origin + Random.FRandRange(-30.f, 30.f) * Plane.AxisU + Random.FRandRange(-20.f, 20.f) * Plane.AxisV;
		Query.Start = Target + Random.FRandRange(-15.f, 15.f) * Plane.Normal;
		Query.End = Target + Random.FRandRange(-15.f, 15.f) * Plane.Normal;
		if (Sample % 11 == 0) Query.Start = Query.End = Target;
		if (Sample % 19 == 0) Query.Start = Query.End = FVector3d(-0.0, 0.0, -0.0);
		const FWorldHit Expected = OriginalRoundPlane(Query, Plane);
		// Emulate the caller's previous decorated witness. Defaults must replace
		// these on a hit; a miss must not touch any byte, including padding.
		Scratch.bHit = true; Scratch.bStartPenetrating = true; Scratch.bSurfaceNormalMayVary = true;
		Scratch.Time = 123; Scratch.PenetrationDepth = 456; Scratch.GeometricErrorBoundCm = 789;
		Scratch.AdditionalResidualAgreementAllowanceCm = 321; Scratch.CanonicalGroupId = 654;
		Scratch.QueryFeatureIndex = 1; Scratch.SurfaceFeatureIndex = 2;
		uint8 Before[sizeof(FWorldHit)];
		FMemory::Memcpy(Before, &Scratch, sizeof(Scratch));
		const bool Found = FWorldQueryService::TrySweepRoundPlane(Query, Plane, Scratch);
		if (!TestEqual(TEXT("plane kernel retains every hit/miss"), Found, Expected.bHit)) return false;
		if (Found)
		{
			++Hits;
			if (!TestTrue(TEXT("plane hit replaces every field with the original bits"), SameHitBits(Scratch, Expected))) return false;
		}
		else
		{
			++Misses;
			if (!TestTrue(TEXT("plane miss leaves all scratch bytes untouched"), FMemory::Memcmp(Before, &Scratch, sizeof(Scratch)) == 0)) return false;
		}
		if (!TestTrue(TEXT("legacy wrapper preserves its complete hit/default miss"),
			SameHitBits(FWorldQueryService::SweepPlane(Query, Plane), Expected))) return false;
	}
	TestTrue(TEXT("oracle covers hits and misses"), Hits > 100 && Misses > 100);

	FBoundedPlane Plane;
	Plane.HalfExtents = FVector2d(20.0, 10.0);
	TArray<FWorldQuery> Queries;
	for (int32 Sample = 0; Sample < 4096; ++Sample)
	{
		auto& Query = Queries.AddDefaulted_GetRef();
		Query.Shape = Sample % 2 ? EQueryShape::Sphere : EQueryShape::Ray;
		Query.Radius = 2.0;
		Query.Start = FVector3d(0, 0, Random.FRandRange(10.f, 100.f));
		Query.End = FVector3d(0, 0, Random.FRandRange(-10.f, 100.f));
	}
	const auto Measure = [&](bool Original, double& Checksum)
	{
		double Sum = 0;
		const double Start = FPlatformTime::Seconds();
		if (Original)
		{
			for (int32 Repeat = 0; Repeat < 16; ++Repeat)
				for (const auto& Query : Queries)
				{
					const FWorldHit Hit = OriginalRoundPlane(Query, Plane);
					Sum += Hit.bHit ? Hit.Time + 2 : 1;
				}
		}
		else
		{
			FWorldHit Hit;
			for (int32 Repeat = 0; Repeat < 16; ++Repeat)
				for (const auto& Query : Queries)
					Sum += FWorldQueryService::TrySweepRoundPlane(Query, Plane, Hit) ? Hit.Time + 2 : 1;
		}
		const double Elapsed = FPlatformTime::Seconds() - Start;
		Checksum = Sum;
		return Elapsed * 1.e9 / (16 * Queries.Num());
	};
	double OldSum = 0, NewSum = 0;
	Measure(true, OldSum); Measure(false, NewSum);
	for (int32 Pair = 0; Pair < 5; ++Pair)
	{
		double OldNs, NewNs;
		if (Pair % 2 == 0) { OldNs = Measure(true, OldSum); NewNs = Measure(false, NewSum); }
		else { NewNs = Measure(false, NewSum); OldNs = Measure(true, OldSum); }
		TestEqual(TEXT("plane microbenchmark checksums"), NewSum, OldSum);
		AddInfo(FString::Printf(TEXT("[RoundPlaneMicro] Pair=%d OriginalNs=%.6f OptimizedNs=%.6f"), Pair, OldNs, NewNs));
	}
	return true;
}
#endif
