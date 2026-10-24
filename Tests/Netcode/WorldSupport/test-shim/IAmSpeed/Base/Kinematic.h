#pragma once
#include "CoreMinimal.h"
struct SKinematic
{
    FVector Location, Velocity, Acceleration;
    FQuat Rotation;
    FVector AngularVelocity, AngularAcceleration;
};
