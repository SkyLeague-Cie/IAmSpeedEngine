#include "TorusGeometry.h"
#include <cmath>

namespace Speed::Geometry
{
bool FTorusGeometry::IsValid() const
{
    return FMath::IsFinite(MajorRadius) && FMath::IsFinite(TubeRadius) &&
        TubeRadius > 0. && MajorRadius > TubeRadius;
}

bool TryTorusPointContact(const FTorusGeometry& Torus,
    const FVector3d& Point, FTorusContact& Out)
{
    Out = {};
    if (!Torus.IsValid() || !FMath::IsFinite(Point.X) ||
        !FMath::IsFinite(Point.Y) || !FMath::IsFinite(Point.Z)) return false;
    const double RadialDistance = std::hypot(Point.Y, Point.Z);
    if (!FMath::IsFinite(RadialDistance)) return false;
    // On the axis every centreline point is equally close; select local +Y.
    const FVector3d Radial = RadialDistance > 0. ?
        FVector3d(0., Point.Y / RadialDistance, Point.Z / RadialDistance) :
        FVector3d(0., 1., 0.);
    const FVector3d Centreline = Radial * Torus.MajorRadius;
    const FVector3d Offset = Point - Centreline;
    const double Distance = std::hypot(Offset.X, std::hypot(Offset.Y, Offset.Z));
    if (!FMath::IsFinite(Distance)) return false;
    // On the centreline all tube normals are equally valid; select local +X.
    const FVector3d Normal = Distance > 0. ? Offset / Distance : FVector3d(1., 0., 0.);
    const FVector3d Surface = Centreline + Normal * Torus.TubeRadius;
    if (!FMath::IsFinite(Surface.X) || !FMath::IsFinite(Surface.Y) || !FMath::IsFinite(Surface.Z)) return false;
    Out.SurfacePoint = Surface;
    Out.Normal = Normal;
    Out.Separation = Distance - Torus.TubeRadius;
    return true;
}

bool TryTorusSphereContact(const FTorusGeometry& Torus,
    const FVector3d& Centre, double Radius, FTorusContact& Out)
{
    Out = {};
    if (!FMath::IsFinite(Radius) || Radius < 0.) return false;
    FTorusContact PointContact;
    if (!TryTorusPointContact(Torus, Centre, PointContact)) return false;
    const double Separation = PointContact.Separation - Radius;
    if (!FMath::IsFinite(Separation)) return false;
    Out = PointContact;
    Out.Separation = Separation;
    return true;
}
}
