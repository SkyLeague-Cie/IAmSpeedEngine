#pragma once
// SOURCE-ONLY draft. Not installed, compiled, or called by a physics path.
// Include beside the unchanged production784 helper in a future opt-in probe.
#include "ExactCrossNormalCache784.h"
namespace Speed::Analytic::Private
{
#if !UE_BUILD_SHIPPING
struct FExactCrossReplayKernels913 final
{
    struct FPreparedRow
    {
        FVector3d Cross;
        double Squared;
        int32 Axis;
        FVector3d OriginalUnit; // Original math precomputed outside measured blocks.
    };
    enum class EKernel : uint8 { Payload, Normalization, LookupShadow, ProductionReuse };
    // A diagnostic surrogate, not a claim about the machine code of production784.
    // Keys, axis partition, two ways, replacement, and finite guards match its source.
    struct FLookupShadow
    {
        struct FEntry { uint64 Key[4]; FVector3d Unit; };
        FEntry Entries[3][2];
        uint8 Prepared[3] = {}, Next[3] = {};
        static bool Finite(double V)
        { return (FExactCrossNormalScope::Bits(V) & 0x7ff0000000000000ull) != 0x7ff0000000000000ull; }
        FORCENOINLINE FVector3d Lookup(const FPreparedRow& R)
        {
            if (!FExactCrossNormalScope::CanMemoizeKey(R.Cross, R.Squared, R.Axis)) return R.OriginalUnit;
            const uint64 Key[4] = {FExactCrossNormalScope::Bits(R.Cross.X), FExactCrossNormalScope::Bits(R.Cross.Y), FExactCrossNormalScope::Bits(R.Cross.Z), FExactCrossNormalScope::Bits(R.Squared)};
            for (int32 Way=0; Way<2; ++Way)
            {
                if ((Prepared[R.Axis] & uint8(1u<<Way)) == 0) continue;
                const FEntry& E=Entries[R.Axis][Way];
                if (FMemory::Memcmp(Key,E.Key,sizeof(Key)) == 0) return E.Unit;
            }
            if (Finite(R.OriginalUnit.X) && Finite(R.OriginalUnit.Y) && Finite(R.OriginalUnit.Z))
            {
                const uint8 Mask=Prepared[R.Axis];
                const int32 Way=(Mask&1u)==0 ? 0 : (Mask&2u)==0 ? 1 : Next[R.Axis];
                FEntry& E=Entries[R.Axis][Way];
                FMemory::Memcpy(E.Key,Key,sizeof(Key));E.Unit=R.OriginalUnit;
                Prepared[R.Axis] |= uint8(1u<<Way);Next[R.Axis]=uint8(1-Way);
            }
            return R.OriginalUnit;
        }
    };
    template<EKernel K> static FORCENOINLINE FVector3d One(const FPreparedRow& R,
        FLookupShadow& Shadow, const FExactCrossNormalScope& Production)
    {
        if constexpr (K==EKernel::Payload) return R.OriginalUnit;
        if constexpr (K==EKernel::Normalization) return R.Cross / FMath::Sqrt(R.Squared);
        if constexpr (K==EKernel::LookupShadow) return Shadow.Lookup(R);
        if constexpr (K==EKernel::ProductionReuse) return Production.Normalize(R.Cross,R.Squared,R.Axis);
    }
    // Clock wrapper belongs OUTSIDE this function. All labels include loop, wrappers,
    // scratch construction and volatile sink. Never subtract Payload from other blocks.
    template<EKernel K> static FORCENOINLINE uint64 Batch(const TArray<FPreparedRow>& Rows,
        const FWorldQuery& Q,const FBoxSweepContext& X,uint32 Repeats)
    {
        volatile uint64 Sink=0;
        for(uint32 I=0;I<Repeats;++I)
        {
            FLookupShadow Shadow;
            FExactCrossNormalScope Production(Q,X,false);
            for(const FPreparedRow& R:Rows)
            {
                const FVector3d U=One<K>(R,Shadow,Production);
                Sink=Sink ^ FExactCrossNormalScope::Bits(U.X) ^ FExactCrossNormalScope::Bits(U.Y) ^ FExactCrossNormalScope::Bits(U.Z);
            }
        }
        return Sink; // Compiler/layout equivalence across kernels remains UNKNOWN.
    }
};
#endif
}
