#pragma once
#include "TorusGeometry.h"

namespace Speed::Geometry
{
struct IAMSPEED_API FTorusBoxDistance
{
    // Bounds on distance(circle, solid OBB) minus tube radius, in centimetres.
    // These are NOT minimum-translation penetration depths for interpenetrating solids.
    double LowerSeparation=0.;
    double UpperSeparation=0.;
    FVector3d CentrelinePoint=FVector3d::ZeroVector;
    FVector3d BoxPoint=FVector3d::ZeroVector;
    bool bConverged=false;
    int32 Evaluations=0;
};

/** True circular centreline versus OBB. Finite branch budget; unresolved is explicit.
 * Box pose is expressed in the torus local frame. No faceting or sampling-only proof.
 * Returns false only for invalid input. Callers must check bConverged before admission.
 */
IAMSPEED_API bool TryTorusBoxDistance(const FTorusGeometry& Torus,
    const FVector3d& BoxCentre,const FQuat4d& BoxRotation,const FVector3d& HalfExtent,
    double ToleranceCm,int32 MaximumEvaluations,FTorusBoxDistance& Out);
}
