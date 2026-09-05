#if WITH_DEV_AUTOMATION_TESTS
#include "IBoxSweeper.h"
#include "ISphereSweeper.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SphereSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "Misc/AutomationTest.h"
#include "UObject/StrongObjectPtr.h"

namespace
{
	// Plain C++ fixtures exercise the real sweeper implementations without a
	// world, solver or UObject subclass. Only partner selection is supplied here.
	template <typename TSweeper>
	class TCandidateFixture final : public TSweeper
	{
	public:
		SKinematic State;
		TArray<TWeakObjectPtr<UBoxSubBody>> Boxes;
		TArray<TWeakObjectPtr<USphereSubBody>> Spheres;
		TArray<TWeakObjectPtr<USWheelSubBody>> Wheels;
		TSet<const UPrimitiveComponent*> Ignored;
		const UPrimitiveComponent* Skipped = nullptr;
		FCollisionResponseParams Responses;

		bool Run(int32 Kind, SHitResult& Hit, float& TOI)
		{
			constexpr float Delta = 1.0f / 300.0f;
			if (Kind == 0) return this->SweepVsBoxes(nullptr, Hit, Delta, TOI);
			if (Kind == 1) return this->SweepVsSpheres(nullptr, Hit, Delta, TOI);
			return this->SweepVsWheels(nullptr, Hit, Delta, TOI);
		}
		bool InternalSweep(UWorld*, const FVector&, const FVector&, SHitResult&, const float&) override { return false; }
		const SKinematic& GetKinematicState() const override { return State; }
		FCollisionShape GetCollisionShape(float = 0.0f) const override { return FCollisionShape(); }
		ECollisionChannel GetCollisionChannel() const override { return ECC_WorldDynamic; }
		const FCollisionResponseParams& GetResponseParams() const override { return Responses; }
		FCollisionQueryParams BuildTraceParams() const override { return FCollisionQueryParams(); }
		const TArray<TWeakObjectPtr<UBoxSubBody>>& GetExternalBoxSubBodies() const override { return Boxes; }
		const TArray<TWeakObjectPtr<USphereSubBody>>& GetExternalSphereSubBodies() const override { return Spheres; }
		const TArray<TWeakObjectPtr<USWheelSubBody>>& GetExternalWheelSubBodies() const override { return Wheels; }
		bool ComponentHasBeenIgnored(const UPrimitiveComponent& Other) const override { return Ignored.Contains(&Other); }
		// Each fixture overrides the relevant shape's hooks; no runtime type check.
		float GetRadiusWithMargin() const { return 10.0f; }
		FVector GetBoxExtent() const { return FVector(10.0); }
		bool ShouldSkipBoxSweep(const UBoxSubBody& Other) const { return Skipped == &Other; }
		bool ShouldSkipSphereSweep(const USphereSubBody& Other) const { return Skipped == &Other; }
	};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedSweeperCandidatesTest,
	"IAmSpeed.ContactDetection.SweeperCandidates",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedSweeperCandidatesTest::RunTest(const FString& Parameters)
{
	TStrongObjectPtr<UBoxSubBody> BoxA(NewObject<UBoxSubBody>()), BoxB(NewObject<UBoxSubBody>());
	TStrongObjectPtr<USphereSubBody> SphereA(NewObject<USphereSubBody>()), SphereB(NewObject<USphereSubBody>());
	TStrongObjectPtr<USWheelSubBody> WheelA(NewObject<USWheelSubBody>()), WheelB(NewObject<USWheelSubBody>());
	SKinematic PartnerState;
	PartnerState.Location = FVector(15.0, 0.0, 0.0);
	for (USSubBody* Body : { static_cast<USSubBody*>(BoxA.Get()), static_cast<USSubBody*>(BoxB.Get()),
		static_cast<USSubBody*>(SphereA.Get()), static_cast<USSubBody*>(SphereB.Get()),
		static_cast<USSubBody*>(WheelA.Get()), static_cast<USSubBody*>(WheelB.Get()) })
	{
		Body->SetKinematicState(PartnerState);
	}
	TCandidateFixture<ISphereSweeper> SphereFixture;
	TCandidateFixture<IBoxSweeper> BoxFixture;
	const auto Check = [this](auto& Fixture, auto& List, auto* A, auto* B,
		int32 Kind, bool bPreservesMiss, bool bHasSkipHook)
	{
		SHitResult Hit;
		float TOI = -1;
		TestFalse(TEXT("empty partner list misses"), Fixture.Run(Kind, Hit, TOI));
		List.AddDefaulted(); // Invalid weak entry before two equal-time candidates.
		List.Add(A);
		List.Add(B);
		TestTrue(TEXT("valid candidates still hit after invalid entry"), Fixture.Run(Kind, Hit, TOI));
		TestTrue(TEXT("first equal-time partner wins"), Hit.Component.Get() == A && Hit.SubBody.Get() == A);
		TestEqual(TEXT("initial overlap remains at time zero"), TOI, 0.0f);
		List.Swap(1, 2);
		TestTrue(TEXT("reordered candidates still hit"), Fixture.Run(Kind, Hit, TOI));
		TestTrue(TEXT("tie arbitration follows caller order"), Hit.Component.Get() == B);
		Fixture.Ignored.Add(B);
		TestTrue(TEXT("ignored first candidate leaves second eligible"), Fixture.Run(Kind, Hit, TOI));
		TestTrue(TEXT("ignored candidate cannot win"), Hit.Component.Get() == A);
		Fixture.Ignored.Add(A);
		Hit.FrameTag = 99;
		TestFalse(TEXT("all ignored partners miss"), Fixture.Run(Kind, Hit, TOI));
		TestEqual(TEXT("each sweep retains its existing miss-output contract"), Hit.FrameTag, bPreservesMiss ? 99u : 0u);
		Fixture.Ignored.Reset();
		if (bHasSkipHook)
		{
			Fixture.Skipped = B;
			TestTrue(TEXT("virtual skip hook leaves second eligible"), Fixture.Run(Kind, Hit, TOI));
			TestTrue(TEXT("virtual skip hook excludes first partner"), Hit.Component.Get() == A);
			Fixture.Skipped = nullptr;
		}
		// Lifetime state is checked afresh on the next sweep, never memoized.
		B->MarkAsGarbage();
		TestTrue(TEXT("garbage partner is skipped on a later sweep"), Fixture.Run(Kind, Hit, TOI));
		TestTrue(TEXT("live partner wins after lifetime invalidation"), Hit.Component.Get() == A);
		B->ClearGarbage();
		List.Reset();
	};
	Check(SphereFixture, SphereFixture.Boxes, BoxA.Get(), BoxB.Get(), 0, false, true);
	Check(SphereFixture, SphereFixture.Spheres, SphereA.Get(), SphereB.Get(), 1, false, false);
	Check(SphereFixture, SphereFixture.Wheels, WheelA.Get(), WheelB.Get(), 2, false, false);
	Check(BoxFixture, BoxFixture.Boxes, BoxA.Get(), BoxB.Get(), 0, true, false);
	Check(BoxFixture, BoxFixture.Spheres, SphereA.Get(), SphereB.Get(), 1, false, true);
	Check(BoxFixture, BoxFixture.Wheels, WheelA.Get(), WheelB.Get(), 2, false, false);
	return true;
}
#endif
