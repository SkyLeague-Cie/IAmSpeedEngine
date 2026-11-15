#pragma once
#include "CoreMinimal.h"
namespace Speed::SphereStaticResponse
{
// One generic branch/arithmetic contract shared with the real sphere response.
// A static skin changes separation, never the physical torque arm.
// This function supplies no hit, owner, margin or physical admission.
inline bool UsesRadialPoint(uint64 surfaceId,bool normalMayVary,bool startPenetrating,float staticContactOffsetCm=0)
{return staticContactOffsetCm>0||(surfaceId!=0&&normalMayVary&&!startPenetrating);}
inline FVector3d Point(FVector3d center,float radius,FVector3d impactPoint,FVector3d normalStored,
    uint64 surfaceId,bool normalMayVary,bool startPenetrating,float staticContactOffsetCm=0)
{
    if(UsesRadialPoint(surfaceId,normalMayVary,startPenetrating,staticContactOffsetCm))
        return center-radius*normalStored.GetSafeNormal();
    return impactPoint;
}
}
