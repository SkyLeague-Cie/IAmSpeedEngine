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
