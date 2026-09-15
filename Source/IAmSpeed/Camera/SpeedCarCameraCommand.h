#pragma once
#include "CoreMinimal.h"

enum class ESpeedCarCameraCommandKind : uint8
{
    Toggle, Height, Distance, FieldOfView, Angle, Stiffness, SwivelSpeed,
    TransitionSpeed, InvertSwivel
};

struct IAMSPEED_API FSpeedCarCameraCommand
{
    uint64 Frame = 0;
    uint64 Serial = 0;
    ESpeedCarCameraCommandKind Kind = ESpeedCarCameraCommandKind::Height;
    float Value = 0;
};
