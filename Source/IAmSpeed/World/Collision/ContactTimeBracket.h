#pragma once

namespace Speed::Collision
{
struct FContactTimeBracket
{
    float SeparatedTime;
    float OverlapTime;
};
// The caller has proved separation at Low and overlap at High. Keep those
// endpoints as invariants; resolving at Low avoids stepping into the overlap.
// This refines a detected event, and does not certify the outer scan's misses.
template<class IsOverlapping>
FContactTimeBracket RefineFirstOverlap(float Low, float High, IsOverlapping&& Overlaps)
{
    for(int Iteration=0;Iteration<32;++Iteration)
    {
        const float Mid=Low+(High-Low)*0.5f;
        if(!(Mid>Low && Mid<High)) break;
        if(Overlaps(Mid)) High=Mid;
        else Low=Mid;
    }
    return {Low,High};
}
}
