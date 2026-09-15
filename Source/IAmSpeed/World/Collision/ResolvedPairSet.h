#pragma once

#include "Containers/Map.h"
#include "Containers/Set.h"
#include "IAmSpeed/Base/SHitResult.h"

#if WITH_DEV_AUTOMATION_TESTS
class FIAmSpeedResolvedPairSetTest;
#endif

namespace Speed::Collision
{

/** Threshold-free structural eligibility; generation is tracked separately. */
inline bool IsEligibleExactStaticRepeatHit(const SHitResult& Hit)
{
	return Hit.TOI > 0 && Hit.SourceId != 0 && Hit.SurfaceId != 0 &&
		!Hit.bSurfaceNormalMayVary && Hit.GeometricErrorBoundCm == 0 &&
		Hit.ContactFeatureThis == EContactFeatureKind::Vertex &&
		Hit.ContactFeatureIndexThis >= 0 && Hit.ContactFeatureIndexThis < 8 &&
		Hit.ContactFeatureOther == EContactFeatureKind::Face &&
		!Hit.bStartPenetrating && Hit.PenetrationDepth <= 0;
}

/** Frame-local membership only; canonical collision order never comes from this set. */
class FResolvedPairSet
{
public:
	bool Contains(const uint64 PairKey) const { return Pairs.Contains(PairKey); }
	uint8 GetGeneration(const uint64 PairKey) const
	{
		const uint8* Generation = Generations.Find(PairKey);
		return Generation ? *Generation : 0;
	}

	/** Ordinary pairs resolve once. Eligible exact identities may reacquire once. */
	bool CanResolve(const uint64 PairKey, const bool bEligibleExactReacquisition) const
	{
		if (!Contains(PairKey)) return true;
		return bEligibleExactReacquisition && GetGeneration(PairKey) == 1;
	}

	/** Reserve the ordinary contact capacity only when the frame actually records a pair. */
	void Add(const uint64 PairKey)
	{
		if (Pairs.IsEmpty())
		{
			Pairs.Reserve(128);
		}
		Pairs.Add(PairKey);
	}

	/** Call only after an eligible, positive-TOI, non-ignored resolution executed. */
	void RecordEligibleResolution(const uint64 PairKey)
	{
		Add(PairKey);
		uint8& Generation = Generations.FindOrAdd(PairKey);
		Generation = Generation < 2 ? Generation + 1 : 2;
	}

private:
	// There is deliberately no removal/reset API: emptiness identifies the first Add.
	TSet<uint64> Pairs;
	TMap<uint64, uint8> Generations;
#if WITH_DEV_AUTOMATION_TESTS
	friend class ::FIAmSpeedResolvedPairSetTest;
#endif
};

}
