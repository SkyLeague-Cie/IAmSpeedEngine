#if WITH_DEV_AUTOMATION_TESTS
#include "OrderedBoundsIndex.h"
#include "Misc/AutomationTest.h"
#include "Math/RandomStream.h"
#include "HAL/PlatformTime.h"
#include <cmath>
#include <limits>

using namespace Speed::Collision;

namespace
{
	TArray<FBox3d> MakeBounds(const int32 Count, const int32 Permutation, const double Scale)
	{
		TArray<FBox3d> Bounds;
		for (int32 I = 0; I < Count; ++I)
		{
			FVector3d Center(0.0), Extent(0.0);
			Center[Permutation % 3] = (I % 16) * 100.0 * Scale;
			Center[(Permutation + 1) % 3] = (I / 16) * 100.0 * Scale;
			Extent[Permutation % 3] = 20.0 * Scale;
			Extent[(Permutation + 1) % 3] = 20.0 * Scale;
			Extent[(Permutation + 2) % 3] = Scale;
			Bounds.Emplace(Center - Extent, Center + Extent);
		}
		return Bounds;
	}

	// Independent exhaustive oracle: the index may return false positives, never
	// omit an intersecting box or reorder two providers that could both hit.
	bool VerifyCandidates(FAutomationTestBase& Test, TConstArrayView<FBox3d> Bounds,
		const FOrderedBoundsIndex& Index, const FBox3d& Query, FOrderedBoundsScratch& Scratch)
	{
		const auto Candidates = Index.FindCandidates(Query, Scratch);
		TBitArray<> Found(false, Bounds.Num());
		int32 Previous = INDEX_NONE;
		for (const int32 Candidate : Candidates)
		{
			if (!Test.TestTrue(TEXT("canonical unique valid indices"), Candidate > Previous && Candidate < Bounds.Num())) return false;
			Found[Candidate] = true;
			Previous = Candidate;
		}
		for (int32 I = 0; I < Bounds.Num(); ++I)
			if (Bounds[I].Intersect(Query) && !Test.TestTrue(TEXT("every possible contact survives"), Found[I])) return false;
		return true;
	}

	template<bool Indexed>
	FORCENOINLINE double MeasureBounds(TConstArrayView<FBox3d> Bounds, const FOrderedBoundsIndex& Index,
		TConstArrayView<FBox3d> Queries, uint64& Checksum)
	{
		constexpr int32 Repeats = 64;
		uint64 Sum = 0;
		const double Start = FPlatformTime::Seconds();
		for (int32 Repeat = 0; Repeat < Repeats; ++Repeat)
		{
			for (const FBox3d& Query : Queries)
			{
				// Match the intended service integration: scratch is local to a query.
				if constexpr (Indexed)
				{
					FOrderedBoundsScratch Scratch;
					for (const int32 I : Index.FindCandidates(Query, Scratch))
						if (Bounds[I].Intersect(Query)) Sum = Sum * 33 + uint64(I + 1);
				}
				else
				{
					for (int32 I = 0; I < Bounds.Num(); ++I)
						if (Bounds[I].Intersect(Query)) Sum = Sum * 33 + uint64(I + 1);
				}
			}
		}
		const double Elapsed = FPlatformTime::Seconds() - Start;
		Checksum = Sum;
		return Elapsed * 1.e9 / (Repeats * Queries.Num());
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedOrderedBoundsIndexTest,
	"IAmSpeed.AnalyticWorld.OrderedBoundsIndex", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedOrderedBoundsIndexTest::RunTest(const FString& Parameters)
{
	FRandomStream Random(150950);
	FOrderedBoundsScratch Scratch;
	for (const int32 Count : { 0, 1, 31, 32, 107, 512 })
	{
		for (int32 Permutation = 0; Permutation < 3; ++Permutation)
		{
			for (const double Scale : { 1.e-150, 1.0, 1.e150 })
			{
				const auto Bounds = MakeBounds(Count, Permutation, Scale);
				const auto Index = FOrderedBoundsIndex::Create(Bounds);
				for (int32 Q = 0; Q < 128; ++Q)
				{
					FVector3d Center(0.0), Extent(0.0);
					Center[Permutation % 3] = Random.FRandRange(-200.f, 1800.f) * Scale;
					Center[(Permutation + 1) % 3] = Random.FRandRange(-200.f, 3500.f) * Scale;
					Extent[Permutation % 3] = Random.FRandRange(0.f, Q % 5 ? 50.f : 2000.f) * Scale;
					Extent[(Permutation + 1) % 3] = Random.FRandRange(0.f, Q % 5 ? 50.f : 3500.f) * Scale;
					Extent[(Permutation + 2) % 3] = 2.0 * Scale;
					if (!VerifyCandidates(*this, Bounds, *Index, FBox3d(Center - Extent, Center + Extent), Scratch)) return false;
				}
				// Provider endpoints and their adjacent doubles include tangencies on
				// either side of bucket boundaries without an arbitrary epsilon.
				for (const FBox3d& Bound : Bounds)
				{
					for (const FVector3d Point : { Bound.Min, Bound.Max })
					{
						for (const double Direction : { -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity() })
						{
							const FVector3d Adjacent(std::nextafter(Point.X, Direction), std::nextafter(Point.Y, Direction), std::nextafter(Point.Z, Direction));
							if (!VerifyCandidates(*this, Bounds, *Index, FBox3d(Adjacent, Adjacent), Scratch)) return false;
						}
						if (!VerifyCandidates(*this, Bounds, *Index, FBox3d(Point, Point), Scratch)) return false;
					}
				}
			}
		}
	}
	const auto LocalBounds = MakeBounds(256, 0, 1.0);
	const auto LocalIndex = FOrderedBoundsIndex::Create(LocalBounds);
	TestTrue(TEXT("localized query prunes a large world"), LocalIndex->FindCandidates(LocalBounds[0], Scratch).Num() < LocalBounds.Num() / 4);
	for (const FBox3d Invalid : { FBox3d(EForceInit::ForceInit),
		FBox3d(FVector3d(-1.0), FVector3d(std::numeric_limits<double>::infinity())),
		FBox3d(FVector3d(std::numeric_limits<double>::quiet_NaN()), FVector3d(1.0)),
		FBox3d(FVector3d(1.0), FVector3d(-1.0)) })
	{
		TestEqual(TEXT("invalid query cannot cause a rejection"), LocalIndex->FindCandidates(Invalid, Scratch).Num(), LocalBounds.Num());
		auto InvalidWorld = LocalBounds;
		InvalidWorld[0] = Invalid;
		const auto InvalidIndex = FOrderedBoundsIndex::Create(InvalidWorld);
		TestEqual(TEXT("invalid provider selects conservative policy"), InvalidIndex->FindCandidates(LocalBounds[0], Scratch).Num(), LocalBounds.Num());
	}
	TArray<FBox3d> Saturated;
	// Set a 0..160 domain, then exceed the 16384-membership budget with
	// eight-by-four-cell providers; include all-world overflow providers too.
	Saturated.Emplace(FVector3d(0.0), FVector3d(160.0));
	for (int32 I = 0; I < 2048; ++I)
		Saturated.Emplace(FVector3d(1.0, 1.0, 1.0), FVector3d(79.0, 39.0, 2.0));
	const auto SaturatedIndex = FOrderedBoundsIndex::Create(Saturated);
	TestTrue(TEXT("storage bounded independently of membership multiplication"), SaturatedIndex->GetAllocatedSize() < 128 * 1024 + SIZE_T(Saturated.Num()) * 8);
	for (const FBox3d Query : { Saturated[1], FBox3d(FVector3d(160.0), FVector3d(160.0)), FBox3d(FVector3d(-1000.0), FVector3d(1000.0)) })
		if (!VerifyCandidates(*this, Saturated, *SaturatedIndex, Query, Scratch)) return false;
	TArray<FBox3d> Degenerate;
	Degenerate.Init(FBox3d(FVector3d(0.0), FVector3d(1.0, 0.0, 0.0)), 64);
	const auto DegenerateIndex = FOrderedBoundsIndex::Create(Degenerate);
	TestEqual(TEXT("one-dimensional world uses original sequence"), DegenerateIndex->FindCandidates(Degenerate[0], Scratch).Num(), 64);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedOrderedBoundsIndexCostTest,
	"IAmSpeed.AnalyticWorld.OrderedBoundsIndexCost", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedOrderedBoundsIndexCostTest::RunTest(const FString& Parameters)
{
	for (const int32 Count : { 107, 512 })
	{
		const auto Bounds = MakeBounds(Count, 0, 1.0);
		const auto Index = FOrderedBoundsIndex::Create(Bounds);
		uint64 LinearSum = 0, GridSum = 0;
		MeasureBounds<false>(Bounds, *Index, Bounds, LinearSum);
		MeasureBounds<true>(Bounds, *Index, Bounds, GridSum);
		for (int32 Pair = 0; Pair < 5; ++Pair)
		{
			double Linear, Grid;
			if (Pair % 2 == 0)
			{
				Linear = MeasureBounds<false>(Bounds, *Index, Bounds, LinearSum);
				Grid = MeasureBounds<true>(Bounds, *Index, Bounds, GridSum);
			}
			else
			{
				Grid = MeasureBounds<true>(Bounds, *Index, Bounds, GridSum);
				Linear = MeasureBounds<false>(Bounds, *Index, Bounds, LinearSum);
			}
			TestEqual(TEXT("indexed and exhaustive ordered checksum"), GridSum, LinearSum);
			AddInfo(FString::Printf(TEXT("[OrderedBoundsMicro] Count=%d Pair=%d LinearNs=%.6f GridNs=%.6f Bytes=%llu"), Count, Pair, Linear, Grid, uint64(Index->GetAllocatedSize())));
		}
	}
	return true;
}
#endif
