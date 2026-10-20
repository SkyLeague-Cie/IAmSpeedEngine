#pragma once
#include "CoreMinimal.h"
#include <atomic>
#if !UE_BUILD_SHIPPING
namespace Speed::Analytic::Private
{
// Row operations, including calibration and every measured kernel/clock pass.
// No reset, config override, waiting or larger allocation on exhaustion.
class FReplayWorkBudget921 final
{
public:
    static constexpr uint64 Limit=uint64(1)<<24;
    bool TryReserve(uint64 Rows,uint32 Repeats) noexcept
    {
        if(Rows==0 || Rows>512 || Repeats==0 || Repeats>65536) return false;
        return TryReserveWork(Rows*uint64(Repeats));
    }
    bool TryReserveWork(uint64 Cost) noexcept
    {
        if(Cost==0 || Cost>Limit) return false;
        uint64 Current=Remaining.load(std::memory_order_relaxed);
        while(Current>=Cost)
            if(Remaining.compare_exchange_weak(Current,Current-Cost,std::memory_order_relaxed)) return true;
        return false;
    }
    uint64 ReadRemaining() const noexcept { return Remaining.load(std::memory_order_relaxed); }
private:
    std::atomic<uint64> Remaining{Limit};
};
// Single plugin process lifetime, shared by all diagnostic callers in this module.
inline FReplayWorkBudget921& ProcessReplayWorkBudget921()
{ static FReplayWorkBudget921 Budget;return Budget; }
}
#endif
