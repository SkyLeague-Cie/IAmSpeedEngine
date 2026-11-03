#include "BodyKinematicRead.h"
#include "IAmSpeed/SubBodies/SSubBody.h"
#include "IAmSpeed/SubBodies/Common/ISweeper.h"
namespace Speed::NativeKinematics
{
const Speed::FKinematicState& FBodyRead::Body(const USSubBody& Body)
{
    return Body.KinematicsStorage();
}
const Speed::FKinematicState& FBodyRead::Sweeper(const ISweeper& Sweeper)
{
    return Sweeper.ReadPhysicsSweeperKinematics();
}
}
