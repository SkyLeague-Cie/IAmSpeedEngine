#pragma once
#include "TorusGeometry.h"

namespace Speed::Geometry
{
enum class ETorusSweepResult : uint8 { Miss, Hit, Invalid, Unresolved };
struct IAMSPEED_API FTorusSphereSweep
{
    ETorusSweepResult Result=ETorusSweepResult::Invalid;
    double Time=1.;
    bool bInitialOverlap=false;
    FTorusContact Contact;
};

/** Exact circular torus and linear sphere trajectory, in torus local space.
 * Quartic candidates are isolated by derivative intervals, then verified against
 * the actual circle-distance surface (spindle roots are not physical contacts).
 * No sampling of the rim. Times are fractions of the supplied segment.
 */
IAMSPEED_API FTorusSphereSweep SweepTorusSphere(const FTorusGeometry& Torus,
    const FVector3d& Start,const FVector3d& End,double Radius,double InitialOverlapTolerance=0.05);
}
