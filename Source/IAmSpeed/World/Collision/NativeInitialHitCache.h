#pragma once
#include "IAmSpeed/Base/SHitResult.h"
namespace Speed
{
// Values-only predicate. It never resolves weak handles or grants permission
// to mutate. DAZ cannot turn a residual subnormal into an empty cache. Signed
// zero is accepted explicitly; object padding is deliberately not compared.
inline bool IsEmptyNativeInitialHit(const SHitResult& h)
{
    const auto f=[](float x){uint32 b;FMemory::Memcpy(&b,&x,4);return (b&0x7fffffffu)==0;};
    const auto d=[](double x){uint64 b;FMemory::Memcpy(&b,&x,8);return (b&0x7fffffffffffffffull)==0;};
    const auto v=[&](const FVector& x){return d(x.X)&&d(x.Y)&&d(x.Z);};
    return !h.bHit&&!h.bBlockingHit&&!h.bStartPenetrating&&!h.bSurfaceNormalMayVary&&
        h.Component.IsExplicitlyNull()&&h.SubBody.IsExplicitlyNull()&&!h.PreparedGroundEvidence&&!h.FirstRsResponseData&&
        v(h.Location)&&v(h.ImpactPoint)&&v(h.ImpactNormal)&&f(h.TOI)&&h.FaceIndex==INDEX_NONE&&
        f(h.PenetrationDepth)&&f(h.GeometricErrorBoundCm)&&v(h.ContactPointThis)&&v(h.ContactPointOther)&&
        h.ContactFeatureThis==EContactFeatureKind::Unknown&&h.ContactFeatureOther==EContactFeatureKind::Unknown&&
        h.ContactFeatureIndexThis==INDEX_NONE&&h.ContactFeatureIndexOther==INDEX_NONE&&
        !h.SourceId&&!h.SurfaceId&&!h.FeatureId&&!h.PrimitiveId&&!h.CanonicalGroupId&&!h.MaterialId&&!h.FrameTag;
}
}
