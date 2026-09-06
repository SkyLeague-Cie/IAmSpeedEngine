#include "OrderedBoundsIndex.h"

namespace Speed::Collision
{
namespace
{
	bool IsFiniteBounds(const FBox3d& Bounds)
	{
		if (!Bounds.IsValid) return false;
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			if (!FMath::IsFinite(Bounds.Min[Axis]) || !FMath::IsFinite(Bounds.Max[Axis]) ||
				Bounds.Min[Axis] > Bounds.Max[Axis]) return false;
		}
		return true;
	}

	class FLinearBoundsIndex final : public FOrderedBoundsIndex
	{
	public:
		explicit FLinearBoundsIndex(int32 Count) : FOrderedBoundsIndex(Count) {}
		TConstArrayView<int32> FindCandidates(const FBox3d&, FOrderedBoundsScratch&) const override
		{
			return AllIndices;
		}
	};

	// Index the two widest axes; omitting the third can add false positives, never misses.
	class FGridBoundsIndex final : public FOrderedBoundsIndex
	{
		static constexpr int32 Divisions = 16;
		static constexpr int32 MaximumMemberships = 16384;
		static constexpr int32 MaximumProviderCells = 32;
		static constexpr int32 MaximumQueryCells = 16;
		struct FCell { int32 First = 0; int32 Count = 0; };
		struct FRange { int32 Min[2]; int32 Max[2]; int32 Count() const
			{ return (Max[0] - Min[0] + 1) * (Max[1] - Min[1] + 1); } };
		FBox3d WorldBounds;
		FVector3d Span;
		int32 Axes[2] = { 0, 1 };
		FCell Cells[Divisions * Divisions];
		TArray<int32> Memberships;
		TArray<int32> Overflow;

		FRange Range(const FBox3d& Bounds) const
		{
			FRange Result;
			for (int32 Dimension = 0; Dimension < 2; ++Dimension)
			{
				const int32 Axis = Axes[Dimension];
				const auto Coordinate = [&](double Value)
				{
					// Positive normalized values make truncation a floor. Inclusive max
					// endpoints share a bucket; clamp before converting distant values.
					const double Normalized = FMath::Clamp((Value - WorldBounds.Min[Axis]) / Span[Axis], 0.0, 1.0);
					return FMath::Min(static_cast<int32>(Normalized * Divisions), Divisions - 1);
				};
				Result.Min[Dimension] = Coordinate(Bounds.Min[Axis]);
				Result.Max[Dimension] = Coordinate(Bounds.Max[Axis]);
			}
			return Result;
		}

	public:
		FGridBoundsIndex(TConstArrayView<FBox3d> Bounds, const FBox3d& InWorldBounds, const int32 Axis0, const int32 Axis1)
			: FOrderedBoundsIndex(Bounds.Num()), WorldBounds(InWorldBounds), Span(InWorldBounds.GetSize())
		{
			Axes[0] = Axis0; Axes[1] = Axis1;
			TArray<int32> Buckets[Divisions * Divisions];
			int32 TotalMemberships = 0;
			for (int32 Index = 0; Index < Bounds.Num(); ++Index)
			{
				const FRange Provider = Range(Bounds[Index]);
				if (Provider.Count() > MaximumProviderCells || TotalMemberships + Provider.Count() > MaximumMemberships)
				{
					// Large bounds and a saturated index use one global entry, not an
					// unbounded duplication into every grid bucket.
					Overflow.Add(Index);
					continue;
				}
				TotalMemberships += Provider.Count();
				for (int32 Y = Provider.Min[1]; Y <= Provider.Max[1]; ++Y)
					for (int32 X = Provider.Min[0]; X <= Provider.Max[0]; ++X)
						Buckets[Y * Divisions + X].Add(Index);
			}
			Memberships.Reserve(TotalMemberships);
			for (int32 Cell = 0; Cell < UE_ARRAY_COUNT(Cells); ++Cell)
			{
				Cells[Cell].First = Memberships.Num();
				Cells[Cell].Count = Buckets[Cell].Num();
				Memberships.Append(Buckets[Cell]);
			}
		}

		TConstArrayView<int32> FindCandidates(const FBox3d& Bounds, FOrderedBoundsScratch& Scratch) const override
		{
			if (!IsFiniteBounds(Bounds)) return AllIndices;
			if (!WorldBounds.Intersect(Bounds)) return {};
			const FRange Query = Range(Bounds);
			if (Query.Count() > MaximumQueryCells) return AllIndices;
			Scratch.Selected.Init(false, AllIndices.Num());
			Scratch.Ordered.Reset();
			for (const int32 Index : Overflow) Scratch.Selected[Index] = true;
			for (int32 Y = Query.Min[1]; Y <= Query.Max[1]; ++Y)
			{
				for (int32 X = Query.Min[0]; X <= Query.Max[0]; ++X)
				{
					const FCell& Cell = Cells[Y * Divisions + X];
					for (int32 Offset = 0; Offset < Cell.Count; ++Offset)
						Scratch.Selected[Memberships[Cell.First + Offset]] = true;
				}
			}
			for (TConstSetBitIterator<TInlineAllocator<8>> It(Scratch.Selected); It; ++It)
				Scratch.Ordered.Add(It.GetIndex());
			return Scratch.Ordered;
		}

		SIZE_T GetAllocatedSize() const override
		{
			return sizeof(*this) + AllIndices.GetAllocatedSize() +
				Memberships.GetAllocatedSize() + Overflow.GetAllocatedSize();
		}
	};
}

FOrderedBoundsIndex::FOrderedBoundsIndex(const int32 Count)
{
	AllIndices.SetNumUninitialized(Count);
	for (int32 Index = 0; Index < Count; ++Index) AllIndices[Index] = Index;
}

TSharedRef<const FOrderedBoundsIndex> FOrderedBoundsIndex::Create(TConstArrayView<FBox3d> Bounds)
{
	if (Bounds.Num() < 32) return MakeShared<FLinearBoundsIndex>(Bounds.Num());
	FBox3d WorldBounds(EForceInit::ForceInit);
	for (const FBox3d& Bound : Bounds)
	{
		if (!IsFiniteBounds(Bound)) return MakeShared<FLinearBoundsIndex>(Bounds.Num());
		WorldBounds += Bound;
	}
	const FVector3d Span = WorldBounds.GetSize();
	int32 Axes[3] = { 0, 1, 2 };
	for (int32 A = 0; A < 2; ++A)
		for (int32 B = A + 1; B < 3; ++B)
			if (Span[Axes[B]] > Span[Axes[A]]) Swap(Axes[A], Axes[B]);
	if (!FMath::IsFinite(Span[Axes[0]]) || !FMath::IsFinite(Span[Axes[1]]) || Span[Axes[1]] <= 0.0)
		return MakeShared<FLinearBoundsIndex>(Bounds.Num());
	return MakeShared<FGridBoundsIndex>(Bounds, WorldBounds, Axes[0], Axes[1]);
}
}
