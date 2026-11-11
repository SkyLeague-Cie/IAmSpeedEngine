#pragma once
#include "IAmSpeed/Base/Kinematic.h"
namespace Speed::PreparedExecution
{
// Copies of the decisions evaluated by UBox::SweepTOI, never a replay of
// HasPersistentGroundContact/GetVelocityAtPoint/Contains or a contact proof.
struct FNativeBoxGroundReuseDecisionValues
{
    bool Persistent=false,VelocityComputed=false,NormalComparison=false;
    bool IgnoreComputed=false,Ignored=false;
    FVector Normal=FVector::ZeroVector,PointVelocity=FVector::ZeroVector;
    float NormalSpeed=0; // the actual native narrowing to FLOAT
};
enum class ENativeBoxGroundReuseOutcome:uint8
{Incomplete,NotEntered,NoPersistent,NormalComparisonFalse,Ignored,Reused};
inline ENativeBoxGroundReuseOutcome ClassifyNativeBoxGroundReuseDecision(const FNativeBoxGroundReuseDecisionValues& v)
{
    using E=ENativeBoxGroundReuseOutcome;
    if(!v.Persistent)return !v.VelocityComputed&&!v.NormalComparison&&!v.IgnoreComputed&&!v.Ignored?E::NoPersistent:E::Incomplete;
    if(!v.VelocityComputed)return E::Incomplete;
    // False also includes a native unordered comparison (NaN). This is NOT
    // called separation, nor is a finite normal/speed inferred from the bool.
    if(!v.NormalComparison)return !v.IgnoreComputed&&!v.Ignored?E::NormalComparisonFalse:E::Incomplete;
    if(!v.IgnoreComputed)return E::Incomplete;
    return v.Ignored?E::Ignored:E::Reused;
}
}
