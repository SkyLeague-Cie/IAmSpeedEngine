#pragma once

#include "Containers/Map.h"
#include "Containers/Set.h"
#include "Math/UnrealMathUtility.h"
#include "IAmSpeed/Base/SHitResult.h"

#if WITH_DEV_AUTOMATION_TESTS
class FIAmSpeedResolvedPairSetTest;
#endif

namespace Speed::Collision
{

/** Shared exact identity checks; no temporal admissibility is implied here. */
inline bool IsExactStaticVertexFaceIdentity(const SHitResult& Hit)
{
	return Hit.SourceId != 0 && Hit.SurfaceId != 0 &&
		!Hit.bSurfaceNormalMayVary && Hit.GeometricErrorBoundCm == 0 &&
		Hit.ContactFeatureThis == EContactFeatureKind::Vertex &&
		Hit.ContactFeatureIndexThis >= 0 && Hit.ContactFeatureIndexThis < 8 &&
		Hit.ContactFeatureOther == EContactFeatureKind::Face &&
		!Hit.bStartPenetrating && Hit.PenetrationDepth <= 0;
}

/** Repeat events still require strictly positive physical time. */
inline bool IsEligibleExactStaticRepeatHit(const SHitResult& Hit)
{
	return Hit.TOI > 0 && IsExactStaticVertexFaceIdentity(Hit);
}

/** Seed after a real nonignored initial resolution, never for a repeat query. */
inline bool IsEligibleExactStaticInitialHit(const SHitResult& Hit)
{
	return Hit.TOI == 0 && IsExactStaticVertexFaceIdentity(Hit);
}

/** Frame-local membership only; canonical collision order never comes from this set. */
class FResolvedPairSet
{
public:
	bool Contains(const uint64 PairKey) const { return Pairs.Contains(PairKey); }
	uint8 GetGeneration(const uint64 PairKey) const
	{
		const FGenerationState* State = Generations.Find(PairKey);
		return State ? State->Generation : 0;
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
		if (FGenerationState* State = Generations.Find(PairKey)) State->bTimed = false;
	}

	/** Call only after a nonignored positive repeat or qualified exact initial resolution actually executed. */
	void RecordEligibleResolution(const uint64 PairKey)
	{
		Add(PairKey);
		FGenerationState& State = Generations.FindOrAdd(PairKey);
		State.Generation = State.Generation < 2 ? State.Generation + 1 : 2;
	}

    /** Separate temporal certificate; the legacy two-generation contract above is unchanged. */
    bool CanResolveExactProgress(const uint64 PairKey, const float OriginalTOI,
        const float Before, const float Arrival, const float Dt) const
    {
        const FGenerationState* State = Generations.Find(PairKey);
        return State && State->bTimed && FMath::IsFinite(OriginalTOI) && OriginalTOI > 0 &&
            FMath::IsFinite(Before) && FMath::IsFinite(Arrival) && FMath::IsFinite(Dt) &&
            0 <= Before && State->LastResolvedTime <= Before && Before < Arrival && Arrival <= Dt && Arrival > State->LastResolvedTime;
    }

    /** Guard the float addition before predicting arrival; never clamp an invalid repeat TOI. */
    bool CanSelectExactProgress(const uint64 PairKey, const float OriginalTOI,
        const float Before, const float Dt) const
    {
        if (!FMath::IsFinite(OriginalTOI) || !FMath::IsFinite(Before) || !FMath::IsFinite(Dt) ||
            OriginalTOI <= 0 || Before < 0 || Before > Dt || OriginalTOI > Dt - Before) return false;
        const float Arrival = Before + OriginalTOI;
        return CanResolveExactProgress(PairKey, OriginalTOI, Before, Arrival, Dt);
    }

    /** Only after an actually executed, nonignored, qualified exact resolution.
     * A previously ordinary/ignored member cannot acquire a temporal certificate.
     * Refusal invalidates temporal eligibility without inventing a legacy resolution. */
    bool RecordExactProgressResolution(const uint64 PairKey, const float ActualTime, const float Dt)
    {
        const bool bWasMember = Contains(PairKey);
        const FGenerationState* Previous = Generations.Find(PairKey);
        const bool bMayRecord = FMath::IsFinite(ActualTime) && FMath::IsFinite(Dt) &&
            0 <= ActualTime && ActualTime <= Dt &&
            (!bWasMember || (Previous && Previous->bTimed && ActualTime > Previous->LastResolvedTime));
        if (!bMayRecord)
        {
            Add(PairKey);
            return false;
        }
        RecordEligibleResolution(PairKey);
        FGenerationState& State = Generations.FindChecked(PairKey);
        State.bTimed = true;
        State.LastResolvedTime = ActualTime;
        return true;
    }

private:
    struct FGenerationState
    {
        uint8 Generation = 0;
        bool bTimed = false;
        float LastResolvedTime = 0;
    };
	// There is deliberately no removal/reset API: emptiness identifies the first Add.
	TSet<uint64> Pairs;
	TMap<uint64, FGenerationState> Generations;
#if WITH_DEV_AUTOMATION_TESTS
	friend class ::FIAmSpeedResolvedPairSetTest;
#endif
};

}
