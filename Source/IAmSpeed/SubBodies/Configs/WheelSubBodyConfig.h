#pragma once

#include "SubBodyConfig.h"
#include "CoreMinimal.h"

class UChaosVehicleWheel;
namespace Chaos
{
    class FSimpleWheelSim;
    class FSimpleSuspensionSim;
}

struct WheelSubBodyConfig : public SubBodyConfig
{
    uint8 WheelIndex = 0;
    float CollisionMargin = 0.0f; // extra margin added to the wheel radius for collision detection
    FVector LocalOffset = FVector::ZeroVector; // local offset of the suspension rest position from the car origin
    UChaosVehicleWheel* ChaosWheel = nullptr;
    Chaos::FSimpleWheelSim* PWheel = nullptr;
    Chaos::FSimpleSuspensionSim* PSuspension = nullptr;
    FCollisionResponseParams ResponseParams;
};