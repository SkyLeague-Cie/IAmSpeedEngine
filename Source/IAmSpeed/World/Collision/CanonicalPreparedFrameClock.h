#pragma once
#include "CoreMinimal.h"
namespace Speed::PreparedExecution
{
// Value relationship only. This cannot authorize callbacks, Step, input
// consumption or a sphere query. The actual owner is read only AFTER native
// member/Step ownership has already been authenticated by the caller.
struct FCanonicalPreparedFrameClock
{
    uint64 CanonicalFrame=0;uint32 OwnerLocalFrame=0;
    bool operator==(const FCanonicalPreparedFrameClock& x) const
    {return CanonicalFrame==x.CanonicalFrame&&OwnerLocalFrame==x.OwnerLocalFrame;}
};
inline bool JoinCanonicalPreparedFrameClock(uint64 canonical,uint32 actualOwnerLocal,
    FCanonicalPreparedFrameClock& out) noexcept
{
    out={};
    // Native owners store uint32 local frames. Prepare canonical f writes f+1;
    // the canonical Step identity remains f. Do not wrap/narrow before checking.
    if(canonical>=uint64(MAX_uint32)||uint64(actualOwnerLocal)!=canonical+1)return false;
    out.CanonicalFrame=canonical;out.OwnerLocalFrame=actualOwnerLocal;return true;
}
}
