#if WITH_DEV_AUTOMATION_TESTS
#include "ResolvedPairSet.h"
#include "Misc/AutomationTest.h"
#include "Math/RandomStream.h"
#include "HAL/PlatformTime.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedResolvedPairSetTest,
	"IAmSpeed.Simulation.ResolvedPairSet",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedResolvedPairSetTest::RunTest(const FString& Parameters)
{
	{
		Speed::Collision::FResolvedPairSet Generations;
		TestTrue(TEXT("new exact identity may resolve initially"), Generations.CanResolve(91, true));
		Generations.RecordEligibleResolution(91);
		TestEqual(TEXT("initial resolution records generation one"), Generations.GetGeneration(91), uint8(1));
		TestTrue(TEXT("one exact reacquisition remains eligible"), Generations.CanResolve(91, true));
		Generations.RecordEligibleResolution(91);
		TestEqual(TEXT("reacquisition saturates at generation two"), Generations.GetGeneration(91), uint8(2));
		TestFalse(TEXT("second repeat is rejected"), Generations.CanResolve(91, true));
		Generations.RecordEligibleResolution(91);
		TestEqual(TEXT("generation stays saturated"), Generations.GetGeneration(91), uint8(2));
		TestTrue(TEXT("another vertex runtime key is independent"), Generations.CanResolve(92, true));
		TestTrue(TEXT("another provider runtime key is independent"), Generations.CanResolve(93, true));
		TestTrue(TEXT("another pair runtime key is independent"), Generations.CanResolve(94, true));

		Speed::Collision::FResolvedPairSet Excluded;
		Excluded.Add(101);
		TestEqual(TEXT("ordinary or ignored add consumes no generation"), Excluded.GetGeneration(101), uint8(0));
		TestFalse(TEXT("ordinary resolved pair cannot reacquire"), Excluded.CanResolve(101, false));
		TestFalse(TEXT("unrecorded eligible generation cannot bypass ordinary membership"), Excluded.CanResolve(101, true));

		SHitResult Eligible;
		Eligible.TOI = .001f;
		Eligible.SourceId = 11; Eligible.SurfaceId = 12;
		Eligible.ContactFeatureThis = Speed::EContactFeatureKind::Vertex;
		Eligible.ContactFeatureIndexThis = 7;
		Eligible.ContactFeatureOther = Speed::EContactFeatureKind::Face;
		TestTrue(TEXT("exact positive vertex-face event is structurally eligible"),
			Speed::Collision::IsEligibleExactStaticRepeatHit(Eligible));
		for (const float InvalidTOI : { 0.f, -.001f })
		{
			SHitResult Invalid = Eligible;
			Invalid.TOI = InvalidTOI;
			TestFalse(TEXT("nonpositive TOI is excluded"),
				Speed::Collision::IsEligibleExactStaticRepeatHit(Invalid));
		}
		SHitResult Penetrating = Eligible;
		Penetrating.bStartPenetrating = true;
		Penetrating.PenetrationDepth = .001f;
		TestFalse(TEXT("penetrating hit is excluded"),
			Speed::Collision::IsEligibleExactStaticRepeatHit(Penetrating));
		SHitResult Curved = Eligible;
		Curved.bSurfaceNormalMayVary = true;
		TestFalse(TEXT("varying normal is excluded"),
			Speed::Collision::IsEligibleExactStaticRepeatHit(Curved));
		SHitResult Uncertain = Eligible;
		Uncertain.GeometricErrorBoundCm = .001;
		TestFalse(TEXT("uncertain geometry is excluded"),
			Speed::Collision::IsEligibleExactStaticRepeatHit(Uncertain));
		SHitResult DifferentFeature = Eligible;
		DifferentFeature.ContactFeatureThis = Speed::EContactFeatureKind::Face;
		TestFalse(TEXT("non vertex-face contact is excluded"),
			Speed::Collision::IsEligibleExactStaticRepeatHit(DifferentFeature));
	}
	FRandomStream Random(150941);
	for (int32 Frame = 0; Frame < 16; ++Frame)
	{
		Speed::Collision::FResolvedPairSet Actual;
		const TSet<uint64> Empty;
		TSet<uint64> Expected;
		Expected.Reserve(128);
		// TSet's accounting includes its small inline hash even before allocation.
		TestEqual(TEXT("empty frame has only default set storage"), Actual.Pairs.GetAllocatedSize(), Empty.GetAllocatedSize());
		TestTrue(TEXT("empty frame avoids the 128-entry reserve"), Actual.Pairs.GetAllocatedSize() < Expected.GetAllocatedSize());
		TestFalse(TEXT("new frame remembers no old pair"), Actual.Contains(0));
		for (int32 Index = 0; Index < 4096; ++Index)
		{
			const uint64 Key = Index % 7 == 0 ? 0 :
				(uint64(Random.GetUnsignedInt() % 300) << 32) | (Random.GetUnsignedInt() % 3);
			TestEqual(TEXT("membership before insertion"), Actual.Contains(Key), Expected.Contains(Key));
			Actual.Add(Key); Expected.Add(Key);
			TestTrue(TEXT("inserted pair can be found"), Actual.Contains(Key));
			TestEqual(TEXT("duplicates and growth preserve cardinality"), Actual.Pairs.Num(), Expected.Num());
			TestEqual(TEXT("nonempty frame has original capacity"), Actual.Pairs.GetAllocatedSize(), Expected.GetAllocatedSize());
		}
		for (const uint64 Key : Expected)
		{
			if (!TestTrue(TEXT("all old pairs survive growth"), Actual.Contains(Key))) return false;
		}
	}
	return true;
}

namespace
{
	struct FOriginalResolvedPairSet : TSet<uint64>
	{
		FOriginalResolvedPairSet() { Reserve(128); }
	};

	template<typename SetType>
	FORCENOINLINE double MeasureResolvedPairs(const int32 PairCount, uint64& Checksum)
	{
		constexpr int32 Frames = 32768;
		uint64 Sum = 0;
		const double Start = FPlatformTime::Seconds();
		for (int32 Frame = 0; Frame < Frames; ++Frame)
		{
			SetType Pairs;
			for (int32 Index = 0; Index < PairCount; ++Index)
			{
				const uint64 Key = (uint64(Frame) << 32) | uint64(Index);
				Sum += Pairs.Contains(Key);
				Pairs.Add(Key);
				Sum += Pairs.Contains(Key);
			}
			Sum += Pairs.Contains(uint64(Frame));
		}
		const double Elapsed = FPlatformTime::Seconds() - Start;
		Checksum = Sum;
		return 1.e9 * Elapsed / Frames;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedResolvedPairSetCostTest,
	"IAmSpeed.Simulation.ResolvedPairSetCost",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedResolvedPairSetCostTest::RunTest(const FString& Parameters)
{
	for (const int32 Count : { 0, 1, 8, 128 })
	{
		uint64 OriginalSum = 0, LazySum = 0;
		MeasureResolvedPairs<FOriginalResolvedPairSet>(Count, OriginalSum);
		MeasureResolvedPairs<Speed::Collision::FResolvedPairSet>(Count, LazySum);
		for (int32 Pair = 0; Pair < 5; ++Pair)
		{
			double Original, Lazy;
			if (Pair % 2 == 0)
			{
				Original = MeasureResolvedPairs<FOriginalResolvedPairSet>(Count, OriginalSum);
				Lazy = MeasureResolvedPairs<Speed::Collision::FResolvedPairSet>(Count, LazySum);
			}
			else
			{
				Lazy = MeasureResolvedPairs<Speed::Collision::FResolvedPairSet>(Count, LazySum);
				Original = MeasureResolvedPairs<FOriginalResolvedPairSet>(Count, OriginalSum);
			}
			TestEqual(TEXT("paired set checksums match"), OriginalSum, LazySum);
			AddInfo(FString::Printf(TEXT("[ResolvedPairSetMicro] Count=%d Pair=%d OriginalNs=%.6f LazyNs=%.6f"),
				Count, Pair, Original, Lazy));
		}
	}
	return true;
}
#endif
