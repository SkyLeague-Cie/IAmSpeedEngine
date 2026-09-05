#pragma once

#include "CoreMinimal.h"

namespace Speed::Collision
{

/** Caller-owned scratch: indexes are immutable and can be queried by independent lanes. */
struct FOrderedBoundsScratch
{
	TBitArray<TInlineAllocator<8>> Selected;
	TArray<int32, TInlineAllocator<32>> Ordered;
};

/** Conservative candidate selection; output always follows the original provider order. */
class IAMSPEED_API FOrderedBoundsIndex
{
public:
	virtual ~FOrderedBoundsIndex() = default;
	/** Returned view borrows this index or Scratch; no hash/spatial iteration sets physical order. */
	virtual TConstArrayView<int32> FindCandidates(const FBox3d& Bounds, FOrderedBoundsScratch& Scratch) const = 0;
	virtual SIZE_T GetAllocatedSize() const { return sizeof(*this) + AllIndices.GetAllocatedSize(); }
	/** Choose the lookup policy once from immutable bounds; invalid/small worlds keep linear lookup. */
	static TSharedRef<const FOrderedBoundsIndex> Create(TConstArrayView<FBox3d> Bounds);

protected:
	explicit FOrderedBoundsIndex(int32 Count);
	TArray<int32> AllIndices;
};

}
