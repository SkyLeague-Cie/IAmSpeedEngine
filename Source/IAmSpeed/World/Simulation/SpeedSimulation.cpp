#include "SpeedSimulation.h"

ASpeedSimulation::ASpeedSimulation()
{
    bAsyncPhysicsTickEnabled = false;
    PrimaryActorTick.bCanEverTick = false;
    PrimaryActorTick.bStartWithTickEnabled = false;
}
