#pragma once

#include "AnalyticBoxSweepContext.h"
#include "ExactCrossNormalCapture844.h"
#include "HAL/UnrealMemory.h"
#include <limits>

// Experimental private scratch. No query/world state, TLS ownership, tolerance
// changes or persistent cache. Only original normalized Cross values are stored.
namespace Speed::Analytic::Private
{
#if !UE_BUILD_SHIPPING
struct FExactCrossNormalScope final
{
    struct FStats
    {
        uint64 Eligible = 0, Hits = 0, Misses = 0, Evictions = 0;
        uint64 Bypass = 0, RecomputedBitMismatches = 0;
    };

    FExactCrossNormalScope(const FWorldQuery& InQuery,
        const FBoxSweepContext& InContext, const bool InObserveOnly,
        FExactCrossNormalCapture844* InCapture = nullptr)
        : Query(InQuery), Context(InContext), bObserveOnly(InObserveOnly), Capture(InCapture) {}
    FExactCrossNormalScope(const FExactCrossNormalScope&) = delete;
    FExactCrossNormalScope& operator=(const FExactCrossNormalScope&) = delete;

    bool IsBoundTo(const FWorldQuery& InQuery, const FBoxSweepContext& InContext) const
    {
        return &InQuery == &Query && &InContext == &Context &&
            Context.bLazyExtrudedSatAxes;
    }

    // Caller still computes the original Cross and squared length, at the
    // original logical slot, and applies the unchanged >1e-20 degeneracy test.
    FVector3d Normalize(const FVector3d& Cross, const double LengthSquared,
        const int32 BoxAxisIndex, bool* OutReused = nullptr) const
    {
        if (Capture) Capture->Record(Cross, LengthSquared, BoxAxisIndex);
        if (OutReused) *OutReused = false;
        if (!CanMemoizeKey(Cross, LengthSquared, BoxAxisIndex))
        {
            if (bObserveOnly) ++Stats.Bypass;
            return Cross / FMath::Sqrt(LengthSquared);
        }
        const uint64 Key[4] = { Bits(Cross.X), Bits(Cross.Y), Bits(Cross.Z), Bits(LengthSquared) };
        if (bObserveOnly) ++Stats.Eligible;
        for (int32 Way = 0; Way < 2; ++Way)
        {
            const uint8 Bit = uint8(1u << Way);
            if ((Prepared[BoxAxisIndex] & Bit) == 0) continue;
            const FEntry& Entry = Entries[BoxAxisIndex][Way];
            if (FMemory::Memcmp(Key, Entry.Key, sizeof(Key)) != 0) continue;
            if (!bObserveOnly)
            {
                if (OutReused) *OutReused = true;
                return Entry.Unit;
            }
            ++Stats.Hits;
            // Observation mode ALWAYS executes and returns original arithmetic.
            const FVector3d Original = Cross / FMath::Sqrt(LengthSquared);
            if (!SameBits(Original, Entry.Unit)) ++Stats.RecomputedBitMismatches;
            return Original;
        }
        if (bObserveOnly) ++Stats.Misses;
        const FVector3d Original = Cross / FMath::Sqrt(LengthSquared);
        if (Finite(Original.X) && Finite(Original.Y) && Finite(Original.Z))
        {
            const uint8 Mask = Prepared[BoxAxisIndex];
            const int32 Way = (Mask & 1u) == 0 ? 0 : (Mask & 2u) == 0 ? 1 : Next[BoxAxisIndex];
            if (bObserveOnly && Mask == 3) ++Stats.Evictions;
            FEntry& Entry = Entries[BoxAxisIndex][Way];
            FMemory::Memcpy(Entry.Key, Key, sizeof(Key));
            Entry.Unit = Original;
            Prepared[BoxAxisIndex] |= uint8(1u << Way);
            Next[BoxAxisIndex] = uint8(1 - Way);
        }
        return Original;
    }

    const FStats& GetObservationStats() const { return Stats; }
    static bool CanMemoizeKey(const FVector3d& Cross, const double LengthSquared, const int32 BoxAxisIndex)
    {
        return BoxAxisIndex >= 0 && BoxAxisIndex < 3 && LengthSquared > 1.0e-20 &&
            Finite(Cross.X) && Finite(Cross.Y) && Finite(Cross.Z) && Finite(LengthSquared);
    }
    static uint64 Bits(const double Value)
    {
        static_assert(sizeof(double) == sizeof(uint64) && std::numeric_limits<double>::is_iec559,
            "Exact normal reuse requires IEEE binary64");
        uint64 Result;
        FMemory::Memcpy(&Result, &Value, sizeof(Result));
        return Result;
    }
    static bool SameBits(const FVector3d& A, const FVector3d& B)
    {
        return Bits(A.X) == Bits(B.X) && Bits(A.Y) == Bits(B.Y) && Bits(A.Z) == Bits(B.Z);
    }

private:
    static bool Finite(const double Value)
    {
        return (Bits(Value) & 0x7ff0000000000000ull) != 0x7ff0000000000000ull;
    }
    struct FEntry
    {
        uint64 Key[4];
        FVector3d Unit;
        FEntry() {} // Prepared bits guard every read; no padding comparisons.
    };
    const FWorldQuery& Query;
    const FBoxSweepContext& Context;
    const bool bObserveOnly;
    FExactCrossNormalCapture844* const Capture;
    mutable FEntry Entries[3][2];
    mutable uint8 Prepared[3] = {};
    mutable uint8 Next[3] = {};
    mutable FStats Stats;
};
static_assert(sizeof(FExactCrossNormalScope) <= 512, "Bound scratch independently of provider/facet count");
#endif
}
