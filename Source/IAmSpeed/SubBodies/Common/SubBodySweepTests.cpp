#if WITH_DEV_AUTOMATION_TESTS
#include "ISphereSweeper.h"
#include "IBoxSweeper.h"
#include "Misc/AutomationTest.h"

namespace
{
	template <typename TSweeper>
	class TEmptyCandidateSweeper final : public TSweeper
	{
	public:
		using TSweeper::SweepVsBoxes;
		using TSweeper::SweepVsSpheres;
		using TSweeper::SweepVsWheels;
		mutable int32 KinematicReads = 0;
		mutable int32 ShapeReads = 0;
		SKinematic Kinematic;
		FCollisionResponseParams Responses;
		TArray<TWeakObjectPtr<UBoxSubBody>> Boxes;
		TArray<TWeakObjectPtr<USphereSubBody>> Spheres;
		TArray<TWeakObjectPtr<USWheelSubBody>> Wheels;
		bool InternalSweep(UWorld*, const FVector&, const FVector&, SHitResult&, const float&) override { return false; }
		const SKinematic& GetKinematicState() const override { ++KinematicReads; return Kinematic; }
		FCollisionShape GetCollisionShape(float = 0) const override { ++ShapeReads; return FCollisionShape::MakeSphere(1); }
		ECollisionChannel GetCollisionChannel() const override { return ECC_WorldStatic; }
		const FCollisionResponseParams& GetResponseParams() const override { return Responses; }
		FCollisionQueryParams BuildTraceParams() const override { return FCollisionQueryParams(); }
		const TArray<TWeakObjectPtr<UBoxSubBody>>& GetExternalBoxSubBodies() const override { return Boxes; }
		const TArray<TWeakObjectPtr<USphereSubBody>>& GetExternalSphereSubBodies() const override { return Spheres; }
		const TArray<TWeakObjectPtr<USWheelSubBody>>& GetExternalWheelSubBodies() const override { return Wheels; }
		bool ComponentHasBeenIgnored(const UPrimitiveComponent&) const override { return false; }
		float GetRadiusWithMargin() const { ++ShapeReads; return 1; }
		FVector GetBoxExtent() const { ++ShapeReads; return FVector(1); }
	};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedEmptySweepCandidatesTest,
	"IAmSpeed.ContactDetection.EmptySweepCandidates",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedEmptySweepCandidatesTest::RunTest(const FString& Parameters)
{
	TEmptyCandidateSweeper<ISphereSweeper> Sphere;
	TEmptyCandidateSweeper<IBoxSweeper> Box;
	const float Delta = 1.0f / 300.0f;
	float TOI = -1;
	SHitResult Hit;
	const auto CheckResetMiss = [&](auto& Sweeper, auto Sweep)
	{
		Hit.bHit = true;
		TOI = -1;
		TestFalse(TEXT("empty candidates miss"), (Sweeper.*Sweep)(nullptr, Hit, Delta, TOI));
		TestFalse(TEXT("reset-on-miss output remains reset"), Hit.bHit);
		TestEqual(TEXT("miss TOI is the supplied interval"), TOI, Delta);
	};
	CheckResetMiss(Sphere, &TEmptyCandidateSweeper<ISphereSweeper>::SweepVsBoxes);
	CheckResetMiss(Sphere, &TEmptyCandidateSweeper<ISphereSweeper>::SweepVsSpheres);
	CheckResetMiss(Sphere, &TEmptyCandidateSweeper<ISphereSweeper>::SweepVsWheels);
	CheckResetMiss(Box, &TEmptyCandidateSweeper<IBoxSweeper>::SweepVsSpheres);
	CheckResetMiss(Box, &TEmptyCandidateSweeper<IBoxSweeper>::SweepVsWheels);
	Hit.bHit = true;
	Hit.ImpactPoint = FVector(1, 2, 3);
	TOI = -1;
	TestFalse(TEXT("box-box empty candidates miss"), Box.SweepVsBoxes(nullptr, Hit, Delta, TOI));
	TestTrue(TEXT("box-box miss preserves caller output"), Hit.bHit && Hit.ImpactPoint == FVector(1, 2, 3));
	TestEqual(TEXT("box-box miss TOI is the supplied interval"), TOI, Delta);
	TestEqual(TEXT("empty sphere sweeps do not read kinematics"), Sphere.KinematicReads, 0);
	TestEqual(TEXT("empty box sweeps do not read kinematics"), Box.KinematicReads, 0);
	TestEqual(TEXT("empty sphere sweeps do not build geometry"), Sphere.ShapeReads, 0);
	TestEqual(TEXT("empty box sweeps do not build geometry"), Box.ShapeReads, 0);
	return true;
}
#endif
