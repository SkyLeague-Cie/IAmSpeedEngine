#include "BodyKinematicRead.h"
#include "IAmSpeed/SubBodies/SSubBody.h"
#include "IAmSpeed/SubBodies/Common/ISweeper.h"
namespace Speed::NativeKinematics
{
const SHitResult& FBodyRead::CurrentHit(const USSubBody& Body) { return Body.CurrentHitStorage(); }
const SHitResult& FBodyRead::FutureHit(const USSubBody& Body) { return Body.FutureHitStorage(); }
bool FBodyRead::HasHit(const USSubBody& Body) { return Body.CurrentHitStorage().bBlockingHit; }
bool FBodyRead::WillHit(const USSubBody& Body) { return Body.FutureHitStorage().bBlockingHit; }
float FBodyRead::FutureTOI(const USSubBody& Body) { return Body.FutureHitStorage().TOI; }
const Speed::FKinematicState& FBodyRead::Body(const USSubBody& Body)
{
    return Body.KinematicsStorage();
}
const Speed::FKinematicState& FBodyRead::Sweeper(const ISweeper& Sweeper)
{
    return Sweeper.ReadPhysicsSweeperKinematics();
}
}
