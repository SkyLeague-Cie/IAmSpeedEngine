#include "SUtils.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FIAmSpeedRayGeometryTest,
	"IAmSpeed.Geometry.Ray.Intersections",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedRayGeometryTest::RunTest(const FString& Parameters)
{
	using namespace Speed;

	const SRay Ray;
	const SSphere Sphere(
		FVector::ZeroVector, 1.0f, FVector::ZeroVector, FVector::ZeroVector);
	const SHitResult SphereHit = Ray.IntersectDuringMovement(
		Sphere, FVector(-2.0f, 0.0f, 0.0f), FVector(2.0f, 0.0f, 0.0f), 1.0f);
	TestTrue(TEXT("Ray intersects sphere"), SphereHit.bHit);
	TestTrue(TEXT("Sphere ray TOI"),
		FMath::IsNearlyEqual(SphereHit.TOI, 0.25f, KINDA_SMALL_NUMBER));
	TestTrue(TEXT("Sphere ray impact point"),
		SphereHit.ImpactPoint.Equals(FVector(-1.0f, 0.0f, 0.0f), KINDA_SMALL_NUMBER));
	TestTrue(TEXT("Sphere ray impact normal"),
		SphereHit.ImpactNormal.Equals(FVector(-1.0f, 0.0f, 0.0f), KINDA_SMALL_NUMBER));

	const Speed::SBox Box(
		FVector::ZeroVector,
		FVector::OneVector,
		FQuat::Identity,
		FVector::ZeroVector,
		FVector::ZeroVector,
		FVector::ZeroVector,
		FVector::ZeroVector);
	const SHitResult BoxHit = Ray.IntersectDuringMovement(
		Box, FVector(0.0f, 0.0f, 2.0f), FVector(0.0f, 0.0f, -2.0f), 1.0f);
	TestTrue(TEXT("Ray intersects box"), BoxHit.bHit);
	TestTrue(TEXT("Box ray TOI"),
		FMath::IsNearlyEqual(BoxHit.TOI, 0.25f, KINDA_SMALL_NUMBER));
	TestTrue(TEXT("Box ray impact point"),
		BoxHit.ImpactPoint.Equals(FVector(0.0f, 0.0f, 1.0f), KINDA_SMALL_NUMBER));
	TestTrue(TEXT("Box ray impact normal"),
		BoxHit.ImpactNormal.Equals(FVector::UpVector, KINDA_SMALL_NUMBER));

	return true;
}

#endif
