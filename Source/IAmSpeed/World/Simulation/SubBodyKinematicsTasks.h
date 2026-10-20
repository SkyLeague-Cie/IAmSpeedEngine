#pragma once

#include "CoreMinimal.h"
#include "IAmSpeed/Base/Kinematic.h"

class ISpeedComponent;
class USSubBody;

namespace Speed::FrameParallel
{
/** Immutable values captured on the canonical owner, never a UObject handle. */
struct FSubBodyKinematicInput
{
    FVector RelativeLocation = FVector::ZeroVector;
    FQuat RelativeRotation = FQuat::Identity;
    float WheelSpringDisplacement = 0;
    bool bWheel = false;
};

/** Exact native classes only; arbitrary virtual Update overrides retain serial execution. */
IAMSPEED_API bool CaptureNativeSubBodyKinematicInput(const USSubBody& Body, FSubBodyKinematicInput& Out);

/** Same operation order as USSubBody and USWheelSubBody::UpdateKinematicsFromOwner. */
IAMSPEED_API SKinematic CalculateSubBodyKinematics(const SKinematic& Parent, const FSubBodyKinematicInput& Input);

/** Values-only, synchronous, bounded batch; output storage must already exist. */
IAMSPEED_API bool CalculateSubBodyKinematicsBatch(const SKinematic& Parent,
    TConstArrayView<FSubBodyKinematicInput> Inputs, TArrayView<SKinematic> Outputs, int32 Lanes);

/**
 * At the original UpdateSubBodiesKinematics call site, capture one actor's
 * immutable inputs, compute private outputs, join and install in original order.
 * False leaves all bodies untouched; caller executes the unchanged legacy method.
 * No ResetForFrame, callbacks, support queries or collision solve run on workers.
 */
IAMSPEED_API bool TryUpdateSubBodyKinematicsInParallel(ISpeedComponent& Component,
    int32 LaneLimit, int32 MinItemsPerLane);
}
