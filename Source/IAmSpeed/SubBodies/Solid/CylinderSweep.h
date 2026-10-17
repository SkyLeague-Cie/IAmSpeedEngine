#pragma once
#include "CylinderGeometry.h"
namespace Speed::Geometry
{
enum class ECylinderSweepResult : uint8 { Invalid, Miss, Hit, Unresolved };
struct IAMSPEED_API FCylinderSweep
{
    ECylinderSweepResult Result=ECylinderSweepResult::Invalid;
    double Time=1.;
    FCylinderContact Contact;
    int32 PoseEvaluations=0;
};
/** Both rigid poses are sampled in the same world frame. Bound is the sum of
 * maximum point displacements of both solids per normalized unit time. The
 * optional acceleration bound covers every pair of material points. */
/** Point-motion enclosure from the actual rigid owners, in normalized time.
 * Allows a fixed support plane to discard tangential gravity without loosening
 * any collision tolerance. Never populated from course or user data. */
struct FCylinderProjectedMotionBounds
{
    FVector3d TranslationVelocity=FVector3d::ZeroVector,TranslationAcceleration=FVector3d::ZeroVector;
    double RotationalVelocity=0.,RotationalAcceleration=0.,TransformRoundoff=0.;
};
IAMSPEED_API FCylinderSweep SweepCylinderShapeTrajectory(const FCylinderGeometry& Cylinder,
    bool bSphere,double Radius,const FVector3d& HalfExtent,
    TFunctionRef<FTransform(double)> CylinderPoseAt,TFunctionRef<FTransform(double)> ShapePoseAt,
    double MotionBoundCm,double AccelerationBoundCm=-1.,double PoseErrorCm=0.,
    const FCylinderProjectedMotionBounds* ProjectedBounds=nullptr);
}
