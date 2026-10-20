#include "IAmSpeed.h"
#include "IAmSpeed/World/Simulation/FrameParallelExecutor.h"

DEFINE_LOG_CATEGORY(SpeedPhysicsLog);

void IAmSpeed::StartupModule()
{
}

void IAmSpeed::ShutdownModule()
{
    Speed::ShutdownFrameParallelExecutor();
}

IMPLEMENT_MODULE(IAmSpeed, IAmSpeed)
