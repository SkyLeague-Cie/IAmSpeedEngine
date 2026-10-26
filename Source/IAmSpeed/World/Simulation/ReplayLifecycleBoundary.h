#pragma once
#include "CoreMinimal.h"
class ASpeedPhysicsSimulation;
namespace Speed::ReplayLifecycle
{
using Token=uint64;
// GT-owned, finite native physical fence, distinct from before-frame0 startup.
// No UObject layout change; neither token nor proof is accepted from an archive.
IAMSPEED_API Token Acquire(ASpeedPhysicsSimulation&,FString& Failure);
IAMSPEED_API bool Validate(ASpeedPhysicsSimulation&,Token,FString& Failure);
IAMSPEED_API bool SuspendService(ASpeedPhysicsSimulation&,Token,FString& Failure);
IAMSPEED_API bool ResumeService(ASpeedPhysicsSimulation&,Token,FString& Failure);
// Only after the actual old input generation has joined; renews under fence.
IAMSPEED_API bool RenewJoinedInputs(ASpeedPhysicsSimulation&,Token,FString& Failure);
IAMSPEED_API bool Release(ASpeedPhysicsSimulation&,Token,FString& Failure);
}
