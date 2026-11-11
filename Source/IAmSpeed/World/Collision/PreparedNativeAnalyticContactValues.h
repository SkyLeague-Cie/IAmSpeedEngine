#pragma once
#include "PreparedWorldSnapshot.h"
#include "IAmSpeed/Base/SHitResult.h"
#include "IAmSpeed/World/Analytic/AnalyticWorldQuery.h"
#include "IAmSpeed/World/Analytic/AnalyticIntervalBounds.h"
#include "IAmSpeed/World/Analytic/AnalyticNumericContext.h"
namespace Speed::PreparedExecution
{
// VALUES ONLY. The caller binds complete query/filter/geometry coverage and
// exact live request serial. Native delivery/consume additionally requires the
// actual response/effect/selection contracts; this utility mints none of them.
// The token is COPIED from the GT binding; no worker weak lookup or raw UObject
// assignment takes place. No FaceIndex is fabricated from an analytic ID.
inline bool BuildNativeAnalyticContactValues(const Analytic::FWorldHit& input,float seconds,uint32 frame,
    const FPreparedSourceContactBinding& binding,SHitResult& output)
{
    output={};Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    uint32 timeBits=0;FMemory::Memcpy(&timeBits,&seconds,4);
    auto nonnegative=[](double v){const auto bits=AnalyticCertified::Bits(v);return (bits&AnalyticCertified::Magnitude)<AnalyticCertified::Infinity&&
        (!(bits&AnalyticCertified::Sign)||!(bits&AnalyticCertified::Magnitude));};
    if((timeBits&0x80000000u)||timeBits>=0x7f800000u||(timeBits&&timeBits<0x00800000u)||
        !input.bHit||!input.SourceId||input.SourceId!=binding.SourceId||!input.SurfaceId||!input.PrimitiveId||
        !binding.SourceIdentity||!binding.ContactIdentity||binding.ContactToken.IsExplicitlyNull()||
        input.Location.ContainsNaN()||input.Point.ContainsNaN()||input.QueryPoint.ContainsNaN()||input.Normal.ContainsNaN()||
        input.Normal.IsZero()||!nonnegative(input.PenetrationDepth)||!nonnegative(input.GeometricErrorBoundCm))return false;
    auto hit=SHitResult::FromAnalyticHitAtSeconds(input,seconds,frame);
    if(!FMath::IsFinite(hit.PenetrationDepth)||!FMath::IsFinite(hit.GeometricErrorBoundCm)||
        double(hit.GeometricErrorBoundCm)<input.GeometricErrorBoundCm)return false;
    // Keep the real nearest float conversion. A positive geometric depth
    // cannot silently become native zero/subnormal under FTZ/DAZ; refuse it
    // rather than inflating a support depth or fabricating physical clearance.
    uint32 nativeDepthBits=0;FMemory::Memcpy(&nativeDepthBits,&hit.PenetrationDepth,4);
    if((AnalyticCertified::Bits(input.PenetrationDepth)&AnalyticCertified::Magnitude)&&
        (nativeDepthBits<0x00800000u||nativeDepthBits>=0x7f800000u))return false;
    hit.Component=binding.ContactToken;
    hit.ContactFeatureThis=input.QueryFeatureKind;hit.ContactFeatureOther=input.SurfaceFeatureKind;
    hit.ContactFeatureIndexThis=input.QueryFeatureIndex;hit.ContactFeatureIndexOther=input.SurfaceFeatureIndex;
    output=MoveTemp(hit);return true;
}
// Separate from SamePreparedHitValues: that historical terminal comparator
// intentionally requires null Component/SubBody. All native values, typed weak
// identities (including stale index/serial) and attached evidence identities are
// compared here without resolving either token.
inline bool SameNativeAnalyticContactValues(const SHitResult& a,const SHitResult& b) noexcept
{
    auto sameFloat=[](float x,float y){uint32 u,v;FMemory::Memcpy(&u,&x,4);FMemory::Memcpy(&v,&y,4);return u==v;};
    auto samePoint=[](const FVector& x,const FVector& y){for(int32 d=0;d<3;++d)if(AnalyticCertified::Bits(x[d])!=AnalyticCertified::Bits(y[d]))return false;return true;};
    return a.bHit==b.bHit&&a.bBlockingHit==b.bBlockingHit&&samePoint(a.Location,b.Location)&&samePoint(a.ImpactPoint,b.ImpactPoint)&&
        samePoint(a.ImpactNormal,b.ImpactNormal)&&sameFloat(a.TOI,b.TOI)&&a.FaceIndex==b.FaceIndex&&
        a.Component.HasSameIndexAndSerialNumber(b.Component)&&a.SubBody.HasSameIndexAndSerialNumber(b.SubBody)&&
        a.bStartPenetrating==b.bStartPenetrating&&sameFloat(a.PenetrationDepth,b.PenetrationDepth)&&
        sameFloat(a.GeometricErrorBoundCm,b.GeometricErrorBoundCm)&&a.bSurfaceNormalMayVary==b.bSurfaceNormalMayVary&&
        samePoint(a.ContactPointThis,b.ContactPointThis)&&samePoint(a.ContactPointOther,b.ContactPointOther)&&
        a.ContactFeatureThis==b.ContactFeatureThis&&a.ContactFeatureOther==b.ContactFeatureOther&&
        a.ContactFeatureIndexThis==b.ContactFeatureIndexThis&&a.ContactFeatureIndexOther==b.ContactFeatureIndexOther&&
        a.SourceId==b.SourceId&&a.SurfaceId==b.SurfaceId&&a.FeatureId==b.FeatureId&&a.PrimitiveId==b.PrimitiveId&&
        a.CanonicalGroupId==b.CanonicalGroupId&&a.MaterialId==b.MaterialId&&a.FrameTag==b.FrameTag&&
        a.PreparedGroundEvidence==b.PreparedGroundEvidence&&a.FirstRsResponseData==b.FirstRsResponseData;
}
}
