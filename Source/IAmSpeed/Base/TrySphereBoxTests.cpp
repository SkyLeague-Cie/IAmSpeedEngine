#if WITH_DEV_AUTOMATION_TESTS
#include "SUtils.h"
#include "Misc/AutomationTest.h"
#include "Math/RandomStream.h"
#include "HAL/PlatformTime.h"

namespace
{
using namespace Speed;

// Independent pre-Try CCD control flow. Do not call the production Try kernel:
// this retained copy protects all hit fields and the early-return branches.
struct FOriginalBox : Speed::SBox
{
    explicit FOriginalBox(const Speed::SBox& Box) : Speed::SBox(Box) {}
SHitResult IntersectOriginal(const SSphere& Sphere, const float deltaTime, const uint8 NbSubsteps) const
{
	if (deltaTime <= 0.f || NbSubsteps == 0) return SHitResult();
	if (Extent() == FVector::ZeroVector || Sphere.Radius == 0.0f)
	{
		return SHitResult();
	}

	// Initial state at t=0
	const FVector Xb0 = AbsoluteCenter();
	const FQuat Qb0 = Rot;
	const FVector Vb0 = Vel;
	const FVector Ab0 = Accel;
	const FVector Wb0 = AngVel;
	const FVector Alphab0 = AngAccel;

	const FVector Xs0 = Sphere.Center;
	const FVector Vs0 = Sphere.Vel;
	const FVector As0 = Sphere.Accel;
	const float Rs = Sphere.Radius;

	// -------- - EARLY OUT -------------
	// Distant-pair rejection needs only distance, not a transformed world witness.
	const float Sep0 = SphereOBBSeparation(Qb0, Xb0, Xs0, Rs);
	if (Sep0 > 0.f)
	{
		const float MaxLinearClosing = SEarlyOut::MaxRelativeTravel(Vb0, Ab0, Vs0, As0, deltaTime);
		const float BoxAngularRadius = MaxAngularSweepRadius(Extent());
		const float MaxRotClosing = SEarlyOut::MaxAngularTravel(Wb0, Alphab0, BoxAngularRadius, deltaTime);
		const float MaxPossibleClosing = MaxLinearClosing + MaxRotClosing;
		if (Sep0 > MaxPossibleClosing)
			return SHitResult(); // no hit possible; keep the caller's scratch untouched
	}
	// ---------------------------------

	auto BoxX = [&](float t) { return AdvancePosition(Xb0, Vb0, Ab0, t); }; // Absolute box center at time t
	auto BoxQ = [&](float t) { return IntegrateRotation(Qb0, Wb0, Alphab0, t); }; // Absolute box rotation at time t
	auto SphereC = [&](float t) { return AdvancePosition(Xs0, Vs0, As0, t); }; // Absolute sphere center at time t

	auto EvalSeparation = [&](float t, FVector& OutClosestPoint, FVector& OutNormal) -> float
		{
			const FVector BoxPos = BoxX(t);
			const FQuat BoxRot = BoxQ(t);
			const FVector SpherePos = SphereC(t);

			const float Sep = SphereOBBSeparation(BoxRot, BoxPos, SpherePos, Rs, &OutClosestPoint);
			OutNormal = (OutClosestPoint - SpherePos).GetSafeNormal();
			if (OutNormal.IsNearlyZero())
			{
				OutNormal = (BoxPos - SpherePos).GetSafeNormal();
			}
			return Sep;
		};

	constexpr int32 MaxRootIterations = 16;
	constexpr float SeparationToleranceCm = 1e-4f;
	const float dtStep = deltaTime / NbSubsteps;

	FVector PrevContactPoint;
	FVector PrevNormal;
	float PrevT = 0.f;
	float PrevSep = EvalSeparation(PrevT, PrevContactPoint, PrevNormal);

	if (PrevSep <= 0.f)
	{
		SHitResult Hit(true, PrevContactPoint, PrevNormal, 0.f);
		Hit.Location = Xs0;
		Hit.bStartPenetrating = PrevSep < 0.f;
		Hit.PenetrationDepth = FMath::Max(0.f, -PrevSep);
		Hit.ContactPointThis = PrevContactPoint;
		Hit.ContactPointOther = Xs0 + PrevNormal * Rs;
		return Hit;
	}

	for (int32 i = 1; i <= static_cast<int32>(NbSubsteps); ++i)
	{
		const float T = FMath::Min(static_cast<float>(i) * dtStep, deltaTime);

		FVector ContactPoint;
		FVector Normal;
		const float Sep = EvalSeparation(T, ContactPoint, Normal);

		if (Sep <= 0.f)
		{
			float LowT = PrevT;
			float HighT = T;
			FVector BestContactPoint = ContactPoint;
			FVector BestNormal = Normal;

			for (int32 Iter = 0; Iter < MaxRootIterations; ++Iter)
			{
				const float MidT = 0.5f * (LowT + HighT);
				FVector MidContactPoint;
				FVector MidNormal;
				const float MidSep = EvalSeparation(MidT, MidContactPoint, MidNormal);

				if (FMath::Abs(MidSep) <= SeparationToleranceCm)
				{
					BestContactPoint = MidContactPoint;
					BestNormal = MidNormal;
					HighT = MidT;
					break;
				}

				if (MidSep > 0.f)
				{
					LowT = MidT;
				}
				else
				{
					HighT = MidT;
					BestContactPoint = MidContactPoint;
					BestNormal = MidNormal;
				}
			}

			FVector FinalContactPoint;
			FVector FinalNormal;
			const float FinalSeparation = EvalSeparation(HighT, FinalContactPoint, FinalNormal);
			if (!FinalNormal.IsNearlyZero())
			{
				BestContactPoint = FinalContactPoint;
				BestNormal = FinalNormal;
			}

			SHitResult Hit(true, BestContactPoint, BestNormal, HighT);
			Hit.Location = SphereC(HighT);
			Hit.PenetrationDepth = FMath::Max(0.f, -FinalSeparation);
			Hit.ContactPointThis = FinalContactPoint;
			Hit.ContactPointOther = Hit.Location + FinalNormal * Rs;
			return Hit;
		}

		PrevT = T;
	}

	return SHitResult();
}



};

bool SameHit(const SHitResult& A, const SHitResult& B)
{
#define SAME_HIT_FIELD(Field) if (FMemory::Memcmp(&A.Field, &B.Field, sizeof(A.Field)) != 0) return false
    SAME_HIT_FIELD(bHit); SAME_HIT_FIELD(bBlockingHit);
    SAME_HIT_FIELD(Location); SAME_HIT_FIELD(ImpactPoint); SAME_HIT_FIELD(ImpactNormal);
    SAME_HIT_FIELD(TOI); SAME_HIT_FIELD(FaceIndex);
    SAME_HIT_FIELD(bStartPenetrating); SAME_HIT_FIELD(PenetrationDepth);
    SAME_HIT_FIELD(GeometricErrorBoundCm); SAME_HIT_FIELD(bSurfaceNormalMayVary);
    SAME_HIT_FIELD(ContactPointThis); SAME_HIT_FIELD(ContactPointOther);
    SAME_HIT_FIELD(ContactFeatureThis); SAME_HIT_FIELD(ContactFeatureOther);
    SAME_HIT_FIELD(ContactFeatureIndexThis); SAME_HIT_FIELD(ContactFeatureIndexOther);
    SAME_HIT_FIELD(SourceId); SAME_HIT_FIELD(SurfaceId); SAME_HIT_FIELD(FeatureId);
    SAME_HIT_FIELD(PrimitiveId); SAME_HIT_FIELD(CanonicalGroupId);
    SAME_HIT_FIELD(MaterialId); SAME_HIT_FIELD(FrameTag);
#undef SAME_HIT_FIELD
    return A.Component == B.Component && A.SubBody == B.SubBody;
}

template<bool bTry>
FORCENOINLINE double Measure(const FOriginalBox& Box, const TArray<SSphere>& Spheres, double& Checksum)
{
    constexpr int32 Calls = 32768;
    SHitResult Scratch;
    double Sum = 0;
    const double Start = FPlatformTime::Seconds();
    for (int32 Index = 0; Index < Calls; ++Index)
    {
        const SSphere& Sphere = Spheres[Index % Spheres.Num()];
        if constexpr (bTry)
        {
            if (Box.TryIntersectNextFrame(Sphere, 1.f / 300.f, 8, Scratch))
                Sum += 1.0 + Scratch.TOI;
        }
        else
        {
            const SHitResult Hit = Box.IntersectOriginal(Sphere, 1.f / 300.f, 8);
            if (Hit.bHit) Sum += 1.0 + Hit.TOI;
        }
    }
    const double Elapsed = FPlatformTime::Seconds() - Start;
    Checksum = Sum;
    return 1.e9 * Elapsed / Calls;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedTrySphereBoxTest,
    "IAmSpeed.Geometry.Solid.TrySphereBoxBits",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedTrySphereBoxTest::RunTest(const FString& Parameters)
{
    FRandomStream Random(150942);
    const auto Vector = [&]() { return FVector(Random.FRandRange(-500.f, 500.f),
        Random.FRandRange(-500.f, 500.f), Random.FRandRange(-500.f, 500.f)); };
    SHitResult Poison(true, FVector(1, -2, 3), FVector(-4, 5, -6), .75f);
    Poison.SourceId = 918; Poison.SurfaceId = 912; Poison.FrameTag = 713;
    Poison.Location = FVector(-17, 18, -19); Poison.bStartPenetrating = true;
    Poison.PenetrationDepth = 1.25f;
    for (int32 Sample = 0; Sample < 2048; ++Sample)
    {
        const FVector Center = Vector();
        const FVector Extent = Sample % 29 == 0 ? FVector::ZeroVector : Vector().GetAbs() + FVector(.1);
        FQuat Rotation = FRotator(Random.FRandRange(-180.f, 180.f),
            Random.FRandRange(-180.f, 180.f), Random.FRandRange(-180.f, 180.f)).Quaternion();
        if (Sample % 7 == 0) Rotation *= .7;
        const FOriginalBox Box(Speed::SBox(Center, Extent, Rotation, Vector(), Vector(),
            Vector() * .01, Vector() * .01));
        const FVector SphereCenter = Sample % 3 == 0 ? Center : Center + Vector() * (Sample % 3 == 1 ? .5 : 20);
        const SSphere Sphere(SphereCenter, Sample % 31 == 0 ? 0.f : Random.FRandRange(.1f, 200.f),
            Vector(), Vector());
        const float Dt = Sample % 37 == 0 ? -0.0f : Random.FRandRange(.00001f, .1f);
        const uint8 Steps = Sample % 41 == 0 ? 0 : 8;
        const SHitResult Expected = Box.IntersectOriginal(Sphere, Dt, Steps);
        SHitResult Actual = Poison;
        const bool bHit = Box.TryIntersectNextFrame(Sphere, Dt, Steps, Actual);
        if (!TestEqual(TEXT("box Try retains the original decision"), bHit, Expected.bHit) ||
            !TestTrue(TEXT("box Try preserves all output fields or leaves miss scratch untouched"),
                SameHit(Actual, bHit ? Expected : Poison)) ||
            !TestTrue(TEXT("legacy box API keeps its full miss/hit contract"),
                SameHit(Box.IntersectNextFrame(Sphere, Dt, Steps), Expected))) return false;
        SHitResult SphereExpected = Expected;
        if (SphereExpected.bHit)
        {
            SphereExpected.ImpactNormal *= -1.0f;
            Swap(SphereExpected.ContactPointThis, SphereExpected.ContactPointOther);
        }
        Actual = Poison;
        const bool bSphereHit = Sphere.TryIntersectNextFrame(Box, Dt, Steps, Actual);
        if (!TestEqual(TEXT("sphere Try retains the original decision"), bSphereHit, Expected.bHit) ||
            !TestTrue(TEXT("sphere Try flips only a complete hit"),
                SameHit(Actual, bSphereHit ? SphereExpected : Poison)) ||
            !TestTrue(TEXT("legacy sphere API keeps its full miss/hit contract"),
                SameHit(Sphere.IntersectNextFrame(Box, Dt, Steps), SphereExpected))) return false;
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedTrySphereBoxCostTest,
    "IAmSpeed.Geometry.Solid.TrySphereBoxCost",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedTrySphereBoxCostTest::RunTest(const FString& Parameters)
{
    const FOriginalBox Box(Speed::SBox(FVector(50, -10, 20), FVector(60, 40, 20),
        FRotator(5, 20, -10).Quaternion(), FVector(1000, 0, 0), FVector(0, 0, -650),
        FVector(.1, -.2, .3), FVector::ZeroVector));
    for (const bool bNear : { false, true })
    {
        TArray<SSphere> Spheres;
        for (int32 Index = 0; Index < 256; ++Index)
            Spheres.Emplace(FVector(bNear ? 50.0 : 2000.0 + Index, -10, 20), 32.f,
                FVector::ZeroVector, FVector::ZeroVector);
        double OriginalSum = 0, TrySum = 0;
        Measure<false>(Box, Spheres, OriginalSum); Measure<true>(Box, Spheres, TrySum);
        for (int32 Pair = 0; Pair < 5; ++Pair)
        {
            double Original, Try;
            if (Pair % 2 == 0)
            {
                Original = Measure<false>(Box, Spheres, OriginalSum);
                Try = Measure<true>(Box, Spheres, TrySum);
            }
            else
            {
                Try = Measure<true>(Box, Spheres, TrySum);
                Original = Measure<false>(Box, Spheres, OriginalSum);
            }
            TestEqual(TEXT("paired hit checksums match"), OriginalSum, TrySum);
            AddInfo(FString::Printf(TEXT("[TrySphereBoxMicro] Near=%d Pair=%d OriginalNs=%.6f TryNs=%.6f"),
                bNear, Pair, Original, Try));
        }
    }
    return true;
}
#endif
