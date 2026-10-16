#include "TorusSurface.h"

namespace Speed::Analytic
{
bool FTorusSurface::Finalize(FString* Reason)
{
    if (!SourceId || !SurfaceId || !FeatureId || !PrimitiveId || ObjectType>=64 ||
        !Geometry.IsValid() || Geometry.MajorRadius>200000. || Geometry.TubeRadius>20000. ||
        Pose.ContainsNaN() || !Pose.GetRotation().IsNormalized() ||
        !Pose.GetScale3D().Equals(FVector::OneVector,1.e-9))
    {
        if (Reason) *Reason=TEXT("Torus requires finite bounded radii, rigid pose and stable identities.");
        return false;
    }
    const FVector Extent(Geometry.TubeRadius,Geometry.MajorRadius+Geometry.TubeRadius,Geometry.MajorRadius+Geometry.TubeRadius);
    Bounds=FBox3d(FBox(-Extent,Extent).TransformBy(Pose));
    return Bounds.IsValid!=0;
}
}
