#pragma once
#include "IAmSpeed/Base/Kinematic.h"
namespace Speed::PreparedExecution
{
// Actual inherited native branch/locals only. Not a mode prediction, effect
// qualification, trajectory model or permission to integrate.
enum class ENativeIntegrationPath:uint8 {Frozen,NonpositiveDelta,Prescribed,CoulombStop,Sleeping,Transport};
enum class ENativeIntegrationStage:uint8 {PrimitiveEnter,PrimitiveReturn,ProjectionReturn,PostReturn,ReactionRestored};
struct FNativeIntegrationPathValues
{
    ENativeIntegrationPath Path=ENativeIntegrationPath::Frozen;float Delta=0;
    bool ReactionsComputed=false,ContactReactionCalled=false,SleepingComputed=false,SleepingResult=false;
    bool TransportComputed=false,RequiresTransport=false;
    FVector RestingReaction=FVector::ZeroVector,AngularReaction=FVector::ZeroVector;
    double StopAfterSeconds=0;int32 TransportSteps=0;float TransportDelta=0;
};
}
