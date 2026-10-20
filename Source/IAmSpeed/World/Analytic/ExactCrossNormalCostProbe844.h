#pragma once
#include "ExactCrossNormalCache784.h"
#include "HAL/PlatformTime.h"
#include "HAL/PlatformTLS.h"
namespace Speed::Analytic::Private
{
#if !UE_BUILD_SHIPPING
// Artificial same-input throughput probe, NOT live frame cost or thread CPU.
// Complete provider sequences only. Cache is reconstructed at each replay.
struct FExactCrossCostProbe844 final
{
    static constexpr int32 Repeats = 32;
    static FORCENOINLINE FVector3d Original(const FExactCrossNormalCapture844::FRow& R)
    { return R.Cross / FMath::Sqrt(R.Squared); }
    static FORCENOINLINE FVector3d Reuse(const FExactCrossNormalCapture844::FRow& R,
        const FExactCrossNormalScope& Scope)
    { return Scope.Normalize(R.Cross, R.Squared, R.Axis); }
    static uint64 Batch(const FExactCrossNormalCapture844& C, const FWorldQuery& Q,
        const FBoxSweepContext& X, bool bReuse)
    {
        volatile uint64 Sink = 0;
        const uint64 Begin = FPlatformTime::Cycles64();
        for (int32 Repeat = 0; Repeat < Repeats; ++Repeat)
        {
            FExactCrossNormalScope Scope(Q, X, false);
            for (const auto& R : C.Rows)
            {
                const FVector3d Unit = bReuse ? Reuse(R, Scope) : Original(R);
                Sink = Sink ^ FExactCrossNormalScope::Bits(Unit.X) ^
                    FExactCrossNormalScope::Bits(Unit.Y) ^ FExactCrossNormalScope::Bits(Unit.Z);
            }
        }
        const uint64 End = FPlatformTime::Cycles64();
        (void)Sink;
        return End - Begin;
    }
    static void Report(const FExactCrossNormalCapture844& C, const FWorldQuery& Q,
        const FBoxSweepContext& X, uint64 ProviderOrdinal)
    {
        uint64 Mismatches = 0;
        FExactCrossNormalScope Check(Q, X, false);
        for (const auto& R : C.Rows)
            if (!FExactCrossNormalScope::SameBits(Original(R), Reuse(R, Check))) ++Mismatches;
        uint64 ClockMin = MAX_uint64, ClockMax = 0;
        for (int32 I = 0; I < 64; ++I)
        {
            const uint64 A = FPlatformTime::Cycles64(), B = FPlatformTime::Cycles64();
            ClockMin = FMath::Min(ClockMin, B - A); ClockMax = FMath::Max(ClockMax, B - A);
        }
        // ABBA then BAAB; all raw blocks retained, no filtering or clock subtraction.
        uint64 O[4] = {}, R[4] = {};
        if (C.IsComplete() && C.Rows.Num() > 0 && Mismatches == 0)
        {
            O[0] = Batch(C,Q,X,false); R[0] = Batch(C,Q,X,true);
            R[1] = Batch(C,Q,X,true); O[1] = Batch(C,Q,X,false);
            R[2] = Batch(C,Q,X,true); O[2] = Batch(C,Q,X,false);
            O[3] = Batch(C,Q,X,false); R[3] = Batch(C,Q,X,true);
        }
        UE_LOG(LogTemp, Display, TEXT("[ExactCrossCostProbe844] Thread=%u Provider=%llu Rows=%d Dropped=%llu Complete=%u Mismatches=%llu Repeats=%d Original0=%llu Reuse0=%llu Original1=%llu Reuse1=%llu Original2=%llu Reuse2=%llu Original3=%llu Reuse3=%llu ClockMin=%llu ClockMax=%llu Scope=RecordedProviderThroughput NotThreadCPU=1 FullCoverage=UNKNOWN"),
            FPlatformTLS::GetCurrentThreadId(), ProviderOrdinal, C.Rows.Num(), C.Dropped,
            uint32(C.IsComplete()), Mismatches, Repeats, O[0],R[0],O[1],R[1],O[2],R[2],O[3],R[3],ClockMin,ClockMax);
    }
};
#endif
}
