#pragma once
#include "CoreMinimal.h"
namespace Speed::Geometry
{
/** Finite solid circular cylinder, local Z axis. Exact curved side and planar caps. */
struct IAMSPEED_API FCylinderGeometry
{
    double Radius=1.,HalfHeight=1.;
    bool IsValid() const;
    FVector3d Support(const FVector3d& Direction) const;
};
struct IAMSPEED_API FCylinderContact
{
    double LowerSeparation=0.,UpperSeparation=0.;
    FVector3d Normal=FVector3d::ZeroVector;
    FVector3d CylinderPoint=FVector3d::ZeroVector,ShapePoint=FVector3d::ZeroVector;
    int32 Iterations=0;
    bool bConverged=false;
};
IAMSPEED_API bool TryCylinderSphereContact(const FCylinderGeometry& Cylinder,
    const FVector3d& Centre,double Radius,FCylinderContact& Out);
/** Deterministic bounded GJK distance and EPA penetration; no polygonal cylinder.
 * Both witnesses and the OBB pose are in cylinder-local coordinates. Failure to
 * converge remains explicit; callers must never promote it to a MISS. */
IAMSPEED_API bool TryCylinderBoxContact(const FCylinderGeometry& Cylinder,
    const FVector3d& Centre,const FQuat4d& Rotation,const FVector3d& HalfExtent,
    FCylinderContact& Out,double ToleranceCm=1.e-6,int32 MaximumIterations=96);
}
