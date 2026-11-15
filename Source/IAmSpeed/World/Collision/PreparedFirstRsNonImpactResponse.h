#pragma once
#include "PreparedFirstArrivalRsHitValues.h"
#include "IAmSpeed/Base/Kinematic.h"

namespace Speed::PreparedExecution
{
struct FFirstRsNonImpactResponseInput
{
    FKinematicState OwnerBefore,BodyBefore;
    FVector ActualCOM=FVector::ZeroVector;
    float PhysicalRadius=0.0f,StaticContactOffsetCm=0.0f,ImpactThreshold=0.0f;
};
struct FFirstRsNonImpactResponseExpectation
{
    FFirstRsNonImpactResponseInput Input;
    SHitResult Hit;
    FKinematicState OwnerAfter;
    FVector Normal=FVector::ZeroVector,Point=FVector::ZeroVector,VelocityAtPoint=FVector::ZeroVector;
    float IncomingNormalSpeed=0.0f,PostNormalSpeed=0.0f,NormalAcceleration=0.0f;
    bool AddsAcceleration=false;
};
// Pure native-arithmetic prediction, never a dispatch ticket. The native Source
// must bind these inputs to actual identities/COM/threshold/empty constraints.
// A real impact is refused before any hit-cache or owner mutation.
IAMSPEED_API bool BuildFirstRsNonImpactResponseExpectation(FFirstArrivalRsHitValues Values,
    FFirstRsNonImpactResponseInput Input,FFirstRsNonImpactResponseExpectation& Out,FString& Reason);
}
