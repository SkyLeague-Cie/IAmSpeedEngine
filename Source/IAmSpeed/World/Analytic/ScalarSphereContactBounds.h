#pragma once
#include "TensorSphereContactBounds.h"
namespace Speed::Analytic::Certified
{
enum class EScalarSphereContactStatus : uint8 { Unknown, Bounded };
struct FScalarSphereContact
{
    int32 RepresentativeSurfaceIndex=INDEX_NONE;
    FVector2d FixedUV=FVector2d::ZeroVector;
    FVector3d ImpactPoint=FVector3d::ZeroVector,Location=FVector3d::ZeroVector;
    FVector3d NormalStored=FVector3d::ZeroVector,NormalConsumed=FVector3d::ZeroVector,RadialResponsePoint=FVector3d::ZeroVector;
    double SampleEvaluationErrorCm=0,SurfaceErrorCm=0,LocationErrorCm=0,ResponseErrorCm=0,StoredNormalErrorL1=0,ConsumedNormalErrorL1=0;
};
namespace ScalarContactDetail
{
inline bool Finite(const FVector3d& v)
{ return AnalyticCertified::Finite(v.X)&&AnalyticCertified::Finite(v.Y)&&AnalyticCertified::Finite(v.Z); }
inline bool Nonzero(const FVector3d& v)
{ return (AnalyticCertified::Bits(v.X)&AnalyticCertified::Magnitude)||(AnalyticCertified::Bits(v.Y)&AnalyticCertified::Magnitude)||(AnalyticCertified::Bits(v.Z)&AnalyticCertified::Magnitude); }
// L1 upper bound also bounds Euclidean error. Difference endpoints are normal
// or zero by the existing FTZ/DAZ-safe interval kernel, not forged subnormals.
inline bool ErrorToEveryPoint(const FVector3d& value,const FIntervalPoint& box,double& out)
{
    if(!Finite(value)||!Detail::ValidPoint(box))return false;
    auto sum=Interval::Exact(0);
    for(int32 d=0;d<3;++d)
    {
        const auto delta=AnalyticCertified::Subtract(Interval::Exact(value[d]),box.Axis[d]);
        if(!AnalyticCertified::Valid(delta))return false;
        const double a=AnalyticCertified::FromBits(AnalyticCertified::Bits(delta.Lower)&AnalyticCertified::Magnitude);
        const double b=AnalyticCertified::FromBits(AnalyticCertified::Bits(delta.Upper)&AnalyticCertified::Magnitude);
        sum=AnalyticCertified::Add(sum,Interval::Exact(FMath::Max(a,b)));
        if(!AnalyticCertified::Valid(sum))return false;
    }
    out=sum.Upper;return TimeDetail::NonnegativeFinite(out);
}
inline FIntervalPoint AddScaled(const FIntervalPoint& point,const FIntervalPoint& normal,Interval scale)
{
    FIntervalPoint out;
    for(int32 d=0;d<3;++d)out.Axis[d]=AnalyticCertified::Add(point.Axis[d],AnalyticCertified::Multiply(scale,normal.Axis[d]));
    return out;
}
}
// Pure scalar conversion, NOT a World hit, FT0/prefix certificate, owner/policy
// selection, wheel/MTD certificate or admitted response. ALL first contacts stay
// in enclosure; a later inside witness is never substituted as the true owner.
// Point is a scalar approximation at the retained fixed UV, with explicit error
// against its own evaluation enclosure; no exact on-surface/root claim. Vectors
// enter by value before output reset, so re-bounding the output itself is safe.
// Caller must consume these exact bits or re-bound the actually recomputed bits,
// then compare the errors of EACH actual consumer with its own virtual margin.
inline EScalarSphereContactStatus ConvertSphereContact(
    const FTensorSphereContactEnclosure& enclosure,int32 leafIndex,
    FVector3d evaluatedPointAtFixedUV,FVector3d consumedCenterAtFT0,
    double capturedCollisionRadius,double physicalRadius,FScalarSphereContact& output)
{
    output={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(enclosure.Status!=ETensorSphereContactStatus::FirstContactEnclosed||!enclosure.Leaves.IsValidIndex(leafIndex)||
       !Detail::ValidPoint(enclosure.SurfacePoint)||!Detail::ValidPoint(enclosure.RadialUnitNormal)||
       !ScalarContactDetail::Finite(evaluatedPointAtFixedUV)||!ScalarContactDetail::Finite(consumedCenterAtFT0)||
       !AnalyticCertified::Finite(capturedCollisionRadius)||!AnalyticCertified::Finite(physicalRadius)||
       enclosure.CollisionRadiusBits!=AnalyticCertified::Bits(capturedCollisionRadius)||
       capturedCollisionRadius<physicalRadius||physicalRadius<=0)return EScalarSphereContactStatus::Unknown;
    const auto& leaf=enclosure.Leaves[leafIndex];
    if(leaf.SurfaceIndex<0||!Detail::ValidPoint(leaf.FixedParameterPoint)||
       !AnalyticCertified::Finite(leaf.FixedParameter.X)||!AnalyticCertified::Finite(leaf.FixedParameter.Y))return EScalarSphereContactStatus::Unknown;
    FScalarSphereContact value;value.RepresentativeSurfaceIndex=leaf.SurfaceIndex;value.FixedUV=leaf.FixedParameter;
    value.ImpactPoint=evaluatedPointAtFixedUV;value.Location=consumedCenterAtFT0;
    // These are the actual scalar normalization operations of the selected
    // response route. Their output bits, including all normalization error,
    // are bounded against the global enclosure below; no exact InvSqrt claim.
    value.NormalStored=(consumedCenterAtFT0-evaluatedPointAtFixedUV).GetSafeNormal();
    value.NormalConsumed=value.NormalStored.GetSafeNormal();
    value.RadialResponsePoint=consumedCenterAtFT0-physicalRadius*value.NormalConsumed;
    const auto center=ScalarContactDetail::AddScaled(enclosure.SurfacePoint,enclosure.RadialUnitNormal,Interval::Exact(capturedCollisionRadius));
    const auto response=ScalarContactDetail::AddScaled(enclosure.SurfacePoint,enclosure.RadialUnitNormal,
        AnalyticCertified::Subtract(Interval::Exact(capturedCollisionRadius),Interval::Exact(physicalRadius)));
    if(!ScalarContactDetail::Finite(value.NormalStored)||!ScalarContactDetail::Finite(value.NormalConsumed)||
       !ScalarContactDetail::Nonzero(value.NormalStored)||!ScalarContactDetail::Nonzero(value.NormalConsumed)||
       !ScalarContactDetail::ErrorToEveryPoint(value.ImpactPoint,leaf.FixedParameterPoint,value.SampleEvaluationErrorCm)||
       !ScalarContactDetail::ErrorToEveryPoint(value.ImpactPoint,enclosure.SurfacePoint,value.SurfaceErrorCm)||
       !ScalarContactDetail::ErrorToEveryPoint(value.Location,center,value.LocationErrorCm)||
       !ScalarContactDetail::ErrorToEveryPoint(value.RadialResponsePoint,response,value.ResponseErrorCm)||
       !ScalarContactDetail::ErrorToEveryPoint(value.NormalStored,enclosure.RadialUnitNormal,value.StoredNormalErrorL1)||
       !ScalarContactDetail::ErrorToEveryPoint(value.NormalConsumed,enclosure.RadialUnitNormal,value.ConsumedNormalErrorL1)||
       value.StoredNormalErrorL1>=1||value.ConsumedNormalErrorL1>=1)return EScalarSphereContactStatus::Unknown;
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    output=MoveTemp(value);return EScalarSphereContactStatus::Bounded;
}
}
