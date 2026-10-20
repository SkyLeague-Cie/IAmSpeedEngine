#pragma once
#include "CoreMinimal.h"
#include "IAmSpeed/SubBodies/Solid/TorusGeometry.h"

namespace Speed::Analytic
{
/** Immutable authored torus. Local X axis; rigid Pose, dimensions already scaled. */
struct IAMSPEED_API FTorusSurface
{
    uint64 SourceId=0,SurfaceId=0,FeatureId=0,PrimitiveId=0;
    uint32 MaterialId=0,ObjectType=0;
    uint64 BlockingChannels=MAX_uint64;
    FTransform Pose=FTransform::Identity;
    Geometry::FTorusGeometry Geometry;
    FBox3d Bounds=FBox3d(EForceInit::ForceInit);
    bool bQueryCollisionEnabled=true;
    bool bAuthorityEligible=true;
    bool Finalize(FString* Reason=nullptr);
};
}
