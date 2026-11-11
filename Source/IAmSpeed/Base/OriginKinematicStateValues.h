#pragma once
#include "Kinematic.h"
namespace Speed
{
// Same COM-to-origin operation order as the actual component getter. Values
// only: this cannot choose which historical/live owner state a callback reads.
IAMSPEED_API FKinematicState CalculateOriginKinematicStateValues(const FKinematicState& COM,FVector LocalCOM);
}
