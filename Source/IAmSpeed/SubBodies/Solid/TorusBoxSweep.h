#pragma once
#include "TorusSphereSweep.h"
#include "TorusBoxGeometry.h"

namespace Speed::Geometry
{
struct IAMSPEED_API FTorusBoxSweep
{
    ETorusSweepResult Result=ETorusSweepResult::Invalid;
    double Time=1.;
    double Separation=0.;
    double ErrorBoundCm=0.;
    FVector3d Normal=FVector3d::ZeroVector;
    FVector3d SurfacePoint=FVector3d::ZeroVector;
    FVector3d BoxPoint=FVector3d::ZeroVector;
};

/** Pose and all witnesses are in torus-local coordinates. MotionBoundCm bounds
 * the total motion of ANY box point over normalized time [0,1], including
 * rotation/COM offset/acceleration. The callback reads only captured values.
 * Exhausted budgets and undefined deep-overlap normals are explicit Unresolved.
 */
IAMSPEED_API FTorusBoxSweep SweepTorusBoxTrajectory(const FTorusGeometry& Torus,
    const FVector3d& HalfExtent,TFunctionRef<FTransform(double)> PoseAt,
    double MotionBoundCm,double InitialOverlapTolerance=0.05);
IAMSPEED_API FTorusBoxSweep SweepTorusShapeTrajectory(const FTorusGeometry& Torus,
    bool bSphere,double Radius,const FVector3d& HalfExtent,TFunctionRef<FTransform(double)> PoseAt,
    double MotionBoundCm,double InitialOverlapTolerance=0.05);
}
