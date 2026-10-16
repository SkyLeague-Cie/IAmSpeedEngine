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
/** Optional smooth-trajectory certificate. Acceleration bounds the second
 * derivative of EVERY shape point in normalized time. A negative bound keeps
 * the generic Lipschitz path. PoseError bounds sampling vs the smooth curve.
 * These bounds prune certified empty intervals only; contact/MTD tolerances
 * and budgets are unchanged. Existing exported entry points retain their ABI.
 */
IAMSPEED_API FTorusBoxSweep SweepTorusShapeBoundedTrajectory(const FTorusGeometry& Torus,
    bool bSphere,double Radius,const FVector3d& HalfExtent,TFunctionRef<FTransform(double)> PoseAt,
    double MotionBoundCm,double InitialOverlapTolerance,double AccelerationBoundCm,
    double PoseErrorCm=0.,int32* PoseEvaluations=nullptr);
}
