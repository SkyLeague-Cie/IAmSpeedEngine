#include "SUtils.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FIAmSpeedSolidGeometryPenetrationTest,
	"IAmSpeed.Geometry.Solid.PenetrationMetadata",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedSolidGeometryPenetrationTest::RunTest(const FString& Parameters)
{
	using namespace Speed;

	const SSphere SphereA(
		FVector::ZeroVector, 2.0f, FVector::ZeroVector, FVector::ZeroVector);
	const SSphere SphereB(
		FVector(3.5f, 0.0f, 0.0f), 2.0f,
		FVector::ZeroVector, FVector::ZeroVector);
	const SHitResult SphereHit = SphereA.IntersectNextFrame(SphereB, 1.0f);
	TestTrue(TEXT("Initial sphere overlap is reported"), SphereHit.bHit);
	TestTrue(TEXT("Initial sphere overlap is distinguished from exact TOI"),
		SphereHit.bStartPenetrating);
	TestTrue(TEXT("Initial sphere overlap depth is geometric"),
		FMath::IsNearlyEqual(SphereHit.PenetrationDepth, 0.5f, KINDA_SMALL_NUMBER));

	const Speed::SBox Box(
		FVector::ZeroVector,
		FVector(2.0f, 3.0f, 4.0f),
		FQuat::Identity,
		FVector::ZeroVector,
		FVector::ZeroVector,
		FVector::ZeroVector,
		FVector::ZeroVector);
	FVector ContactPoint;
	const float ExteriorSeparation = Box.SphereOBBSeparation(
		FQuat::Identity, FVector::ZeroVector, FVector(2.25f, 0.0f, 0.0f),
		0.5f, &ContactPoint);
	TestTrue(TEXT("Exterior sphere/box overlap depth is signed"),
		FMath::IsNearlyEqual(ExteriorSeparation, -0.25f, KINDA_SMALL_NUMBER));
	TestTrue(TEXT("Exterior sphere/box contact lies on the box face"),
		ContactPoint.Equals(FVector(2.0f, 0.0f, 0.0f), KINDA_SMALL_NUMBER));

	const float InteriorSeparation = Box.SphereOBBSeparation(
		FQuat::Identity, FVector::ZeroVector, FVector(1.5f, 0.0f, 0.0f),
		0.5f, &ContactPoint);
	TestTrue(TEXT("Contained sphere uses distance to the nearest exit face"),
		FMath::IsNearlyEqual(InteriorSeparation, -1.0f, KINDA_SMALL_NUMBER));
	TestTrue(TEXT("Contained sphere receives a stable exit contact"),
		ContactPoint.Equals(FVector(2.0f, 0.0f, 0.0f), KINDA_SMALL_NUMBER));

	const SSphere OverlappingBoxSphere(
		FVector(2.25f, 0.0f, 0.0f), 0.5f,
		FVector::ZeroVector, FVector::ZeroVector);
	const SHitResult BoxHit = Box.IntersectNextFrame(OverlappingBoxSphere, 1.0f, 8);
	TestTrue(TEXT("Initial sphere/box overlap is reported"), BoxHit.bHit);
	TestTrue(TEXT("Initial sphere/box overlap is distinguished from exact TOI"),
		BoxHit.bStartPenetrating);
	TestTrue(TEXT("Initial sphere/box overlap depth is geometric"),
		FMath::IsNearlyEqual(BoxHit.PenetrationDepth, 0.25f, KINDA_SMALL_NUMBER));
	TestTrue(TEXT("Initial sphere/box hit preserves the sphere center"),
		BoxHit.Location.Equals(OverlappingBoxSphere.Center, KINDA_SMALL_NUMBER));

	const SSphere MovingBoxSphere(
		FVector(4.0f, 0.0f, 0.0f), 0.5f,
		FVector(-4.0f, 0.0f, 0.0f), FVector::ZeroVector);
	const SHitResult MovingSphereHit =
		MovingBoxSphere.IntersectNextFrame(Box, 1.0f, 16);
	TestTrue(TEXT("Moving sphere/box contact is reported"), MovingSphereHit.bHit);
	TestFalse(TEXT("Moving sphere/box contact does not start penetrating"),
		MovingSphereHit.bStartPenetrating);
	TestTrue(TEXT("Moving sphere/box TOI center remains unquantized and tangent"),
		MovingSphereHit.Location.Equals(FVector(2.5f, 0.0f, 0.0f), 1.0e-3f));
	TestTrue(TEXT("Sphere-side contact point belongs to the sphere"),
		MovingSphereHit.ContactPointThis.Equals(
			FVector(2.0f, 0.0f, 0.0f), 1.0e-3f));

	return true;
}

#endif
