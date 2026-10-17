#pragma once
#include "CoreTypes.h"
#include "Serialization/Archive.h"

namespace Speed::Input
{
// Private input wire envelope v1. A missing committed projection is NOT a
// neutral command. Both peers must use this codec; legacy packets are not
// claimed compatible. Mechanical-state and bump snapshot layouts are unchanged.
inline bool SerializeInputProjectionAvailability(FArchive& Ar, bool& bHasProjection)
{
    uint8 Version=1;
    uint8 Presence=bHasProjection?1:0;
    Ar << Version << Presence;
    if(Ar.IsError() || Version!=1 || Presence>1)
    {
        Ar.SetError();
        return false;
    }
    if(Ar.IsLoading()) bHasProjection=Presence!=0;
    return true;
}
}
