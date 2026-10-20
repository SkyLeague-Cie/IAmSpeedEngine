#pragma once

#include "CoreMinimal.h"

namespace Speed::Geometry
{
/** Local X is the axis; the centreline circle lies in YZ. No UObject access. */
struct IAMSPEED_API FTorusGeometry
{
    double MajorRadius = 0.;
    double TubeRadius = 0.;
    bool IsValid() const;
};

struct IAMSPEED_API FTorusContact
{
    FVector3d SurfacePoint = FVector3d::ZeroVector;
    FVector3d Normal = FVector3d::ZeroVector;
    double Separation = 0.;
};

/** Signed separation from a point; deterministic at nonunique closest points. */
IAMSPEED_API bool TryTorusPointContact(const FTorusGeometry& Torus,
    const FVector3d& Point, FTorusContact& Out);
/** Static local-space sphere contact. This is not a CCD/TOI query. */
IAMSPEED_API bool TryTorusSphereContact(const FTorusGeometry& Torus,
    const FVector3d& Centre, double Radius, FTorusContact& Out);
}
