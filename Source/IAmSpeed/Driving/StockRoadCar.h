#pragma once
#include "IAmSpeed/Components/SpeedWheeledComponent.h"

namespace Speed::Driving
{
/** Construction-time road profile. A game's specialized movement class is
 * deliberately excluded; it may call this explicitly in its own constructor.
 * SI-ish accelerations are cm/s^2; the existing angular drag law uses 1/s.
 * These are a sports-road starting point, not a measured vehicle homologation.
 */
inline void ApplyStockRoadCar(USpeedWheeledComponent& M)
{
    M.MaxThrottleSpeed=8000;M.PhysMaxSpeed=8000;
    M.ThrottleAccel=750;M.BrakeAccel=1100;
    M.PhysDamping=0; // Never introduce the legacy per-frame linear damping.
    M.RollDragCoeff=.10f;M.PitchDragCoeff=.12f;M.YawDragCoeff=.10f;
    M.AirForwardThrottle=0;M.AirBackwardThrottle=0;
    M.DriveSpeedAccelScaleCurve={{0,1},{3000,.9f},{6000,.7f},{8000,.6f},{8100,0}};
    M.SteerAngleFromSpeedCurve={{0,.593412f},{1000,.488692f},{2000,.314159f},{3000,.174533f},{5000,.05236f},{8000,.013963f}};
    M.CanonicalCameraYawAction=5;M.CanonicalCameraPitchAction=6;
}
inline bool ApplyStockRoadCarToGenericMovement(USpeedWheeledComponent& M)
{
    if(M.GetClass()!=USpeedWheeledComponent::StaticClass())return false;
    ApplyStockRoadCar(M);return true;
}
}
