#pragma once

#include "Containers/Set.h"

#if WITH_DEV_AUTOMATION_TESTS
class FIAmSpeedResolvedPairSetTest;
#endif

namespace Speed::Collision
{

/** Frame-local membership only; canonical collision order never comes from this set. */
class FResolvedPairSet
{
public:
	bool Contains(const uint64 PairKey) const { return Pairs.Contains(PairKey); }

	/** Reserve the ordinary contact capacity only when the frame actually records a pair. */
	void Add(const uint64 PairKey)
	{
		if (Pairs.IsEmpty())
		{
			Pairs.Reserve(128);
		}
		Pairs.Add(PairKey);
	}

private:
	// There is deliberately no removal/reset API: emptiness identifies the first Add.
	TSet<uint64> Pairs;
#if WITH_DEV_AUTOMATION_TESTS
	friend class ::FIAmSpeedResolvedPairSetTest;
#endif
};

}
