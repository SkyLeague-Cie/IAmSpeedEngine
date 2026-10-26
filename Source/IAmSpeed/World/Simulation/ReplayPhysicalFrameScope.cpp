#include "ReplayPhysicalFrameScope.h"
namespace Speed::Replay
{
namespace {thread_local const ASpeedPhysicsSimulation* CurrentDriver=nullptr;thread_local uint64 CurrentFrame=0;thread_local bool Active=false;}
FPhysicalFrameScope::FPhysicalFrameScope(const ASpeedPhysicsSimulation& Driver,uint64 Frame)
    :PreviousDriver(CurrentDriver),PreviousFrame(CurrentFrame),PreviousActive(Active)
{CurrentDriver=&Driver;CurrentFrame=Frame;Active=!PreviousActive;}
FPhysicalFrameScope::~FPhysicalFrameScope()
{CurrentDriver=PreviousDriver;CurrentFrame=PreviousFrame;Active=PreviousActive;}
bool ReadPhysicalOwnerFrame(const ASpeedPhysicsSimulation* Driver,uint64& Frame) noexcept
{if(!Active || !Driver || CurrentDriver!=Driver)return false;Frame=CurrentFrame;return true;}
}
