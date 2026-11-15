#pragma once
#if WITH_EDITOR && !UE_BUILD_SHIPPING
#include "CoreMinimal.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

namespace Speed::Analytic::Qualification
{
struct FOppositeBallProbe { FVector3d Start, End; uint64 Winner; };

inline const FOppositeBallProbe& ReadTangentialBallProbe()
{
    static const FOppositeBallProbe C{FVector3d(3600.2220889606056,3872.1720389758334,96.78974383713013),FVector3d(3600.972093543983,3872.6149085076395,94.83939522657089),0x82e2ce3273c1c79eull};
    return C;
}

inline const FOppositeBallProbe* ReadOppositeBallProbe()
{
    static const bool bA = FParse::Param(FCommandLine::Get(),TEXT("SLC2QualificationOppositeA"));
    static const bool bB = FParse::Param(FCommandLine::Get(),TEXT("SLC2QualificationOppositeB"));
    static const bool bC = FParse::Param(FCommandLine::Get(),TEXT("SLC2QualificationTangentialC"));
    if (int32(bA) + int32(bB) + int32(bC) != 1)
        return nullptr;
    static const FOppositeBallProbe A{FVector3d(3599.7907437886233,3870.4679824356604,96.4159834214402),FVector3d(3599.7907437886233,3870.8816864229602,94.45923892843864),0xbea91209a592227eull};
    static const FOppositeBallProbe B{FVector3d(3600.041717867045,3870.467982420408,96.41598342135424),FVector3d(3600.041718013276,3870.8816864077166,94.4592389283545),0x82e2ce3273c1c79eull};
    return bC ? &ReadTangentialBallProbe() : (bA ? &A : &B);
}
}
#endif
