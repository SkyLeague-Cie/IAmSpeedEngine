#pragma once
#include <atomic>
#include <cstdint>

namespace Speed::Analytic
{
// Integer work assignment only. Results are still folded by the original
// owner in canonical cell order after every worker has joined.
class FCompleteCellChunks1353 final
{
public:
    struct FRange { std::uint32_t Begin=0, End=0; };
    static constexpr std::uint32_t MaximumItems=4096, MaximumLanes=12;
    FCompleteCellChunks1353(std::uint32_t Items,std::uint32_t Lanes,std::uint32_t Chunk) noexcept
        : Count(Items), LaneCount(Lanes), ChunkSize(Chunk),
          Valid(Items<=MaximumItems && Lanes>=1 && Lanes<=MaximumLanes && Chunk>=1 && Chunk<=MaximumItems),
          Cursor(Valid ? (Lanes*Chunk<Items ? Lanes*Chunk : Items) : 0) {}
    bool IsValid() const noexcept { return Valid; }
    // One seed range is reserved for each lane. Even a very fast caller cannot
    // consume a worker's seed before that worker starts. Call once per lane.
    bool InitialRangeForLane(std::uint32_t Lane,FRange& Out) const noexcept
    {
        if(!Valid || Lane>=LaneCount)return false;
        const auto Begin=Lane*ChunkSize;
        if(Begin>=Count)return false;
        const auto Proposed=Begin+ChunkSize;
        Out={Begin,Proposed<Count ? Proposed : Count};return true;
    }
    // Saturation prevents wraparound even after repeated exhausted calls.
    bool TakeNext(FRange& Out) noexcept
    {
        if(!Valid)return false;
        auto Begin=Cursor.load(std::memory_order_relaxed);
        while(Begin<Count)
        {
            const auto Proposed=Begin+ChunkSize;
            const auto End=Proposed<Count ? Proposed : Count;
            if(Cursor.compare_exchange_weak(Begin,End,std::memory_order_relaxed,std::memory_order_relaxed))
            {Out={Begin,End};return true;}
        }
        return false;
    }
private:
    const std::uint32_t Count,LaneCount,ChunkSize;
    const bool Valid;
    std::atomic<std::uint32_t> Cursor;
};
}
