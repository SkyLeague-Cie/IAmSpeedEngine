#pragma once
#include "Containers/Array.h"
#include "Math/UnrealMathUtility.h"

namespace Speed::Collision
{
// Caller supplies an eligible candidate and its already-clamped native TOI.
// Tolerance groups simultaneous events; it must never delay integration past
// a strictly earlier arrival. Retain caller order among surviving entries.
template<typename TEntry,typename TAllocator>
inline void AppendEarliestCollision(const TEntry& Candidate,float Time,float Tolerance,
    TArray<TEntry,TAllocator>& Batch,float& Earliest)
{
    if(Batch.IsEmpty()||Time<Earliest)
    {
        Earliest=Time;
        // A chain of near-equal arrivals must not keep entries outside the
        // band around the new minimum. RemoveAll preserves surviving order.
        Batch.RemoveAll([Time,Tolerance](const TEntry& Entry)
            {return !FMath::IsNearlyEqual(Entry.TOI,Time,Tolerance);});
    }
    else if(!FMath::IsNearlyEqual(Time,Earliest,Tolerance))return;
    TEntry& Selected=Batch.Add_GetRef(Candidate);
    Selected.TOI=Time; // Hit's own TOI/payload is not rewritten.
}
}
