#include "ComponentKinematicRead.h"
#include "IAmSpeed/Components/ISpeedComponent.h"

namespace Speed::NativeKinematics
{
const Speed::FKinematicState& FComponentRead::Current(const ISpeedComponent& Component)
{
    return Component.ReadPhysicsKinematicState();
}
const Speed::FKinematicState& FComponentRead::ForFrame(const ISpeedComponent& Component, const unsigned int& Frame)
{
    return Component.ReadPhysicsKinematicStateForFrame(Frame);
}
}
