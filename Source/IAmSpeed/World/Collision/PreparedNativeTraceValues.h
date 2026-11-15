#pragma once
#include "CoreMinimal.h"
namespace Speed::PreparedExecution
{
// Values copied by the actual protected BuildTraceParams caller. These are
// observations, not query permission and not a replacement filter domain.
struct FNativeRsTraceValues
{
    bool TraceComplex=false,FindInitialOverlaps=false,ReturnFaceIndex=false,ReturnPhysicalMaterial=false;
    bool IgnoreBlocks=false,IgnoreTouches=false,SkipNarrowPhase=false,TraceIntoSubComponents=false;
    bool ReplaceHitWithSubComponents=false;
    uint64 IgnoreMask=0;
    uint8 TraceChannel=0,Mobility=0;
    TArray<uint8> Responses;
    TArray<uint32> IgnoredSourceObjects,IgnoredComponents;
    bool Same(const FNativeRsTraceValues& x) const
    {
        return TraceComplex==x.TraceComplex&&FindInitialOverlaps==x.FindInitialOverlaps&&
            ReturnFaceIndex==x.ReturnFaceIndex&&ReturnPhysicalMaterial==x.ReturnPhysicalMaterial&&
            IgnoreBlocks==x.IgnoreBlocks&&IgnoreTouches==x.IgnoreTouches&&SkipNarrowPhase==x.SkipNarrowPhase&&
            TraceIntoSubComponents==x.TraceIntoSubComponents&&ReplaceHitWithSubComponents==x.ReplaceHitWithSubComponents&&
            IgnoreMask==x.IgnoreMask&&TraceChannel==x.TraceChannel&&Mobility==x.Mobility&&Responses==x.Responses&&
            IgnoredSourceObjects==x.IgnoredSourceObjects&&IgnoredComponents==x.IgnoredComponents;
    }
};
// Preserve the old type/ABI while sharing its shape-independent trace values.
using FNativeSolidTraceValues=FNativeRsTraceValues;
}
