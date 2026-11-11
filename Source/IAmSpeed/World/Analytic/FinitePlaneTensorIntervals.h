#pragma once
#include "TensorDistanceBounds.h"

namespace Speed::Analytic::Certified
{
struct FPlaneTriangleIntervalNet
{
    FIntervalTensorSurface Surface;
    uint64 SourceId=0,SurfaceId=0,FeatureId=0,PrimitiveId=0;
    uint32 MaterialId=0,ObjectType=0;
    uint64 BlockingChannels=0;
    bool bQueryCollisionEnabled=false,bRequiresCompactOptIn=false,bAuthorityEligible=false;
    FIntVector OriginalDomainIndices=FIntVector::ZeroValue;
};
// Construct ONE triangle from the plane's actual ordered polygon. The caller
// must separately bind a complete certified partition of that polygon. This
// function does NOT prove coverage, rank, World transforms or eligibility.
// FBoundedPlane fields are already in their declared runtime frame: never apply
// an Actor transform a second time. The exact affine sums are enclosed before
// becoming coefficients; rounded FVector3d vertices are not used as authority.
inline bool BuildPlaneTriangleIntervalNet(const FBoundedPlane& plane,const FIntVector& indices,
    FPlaneTriangleIntervalNet& output)
{
    output={}; // Clear prior output even if the mandatory numeric guard throws.
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(plane.DomainVertices.Num()<3||indices.X<0||indices.Y<0||indices.Z<0||
        indices.X>=plane.DomainVertices.Num()||indices.Y>=plane.DomainVertices.Num()||indices.Z>=plane.DomainVertices.Num()||
        indices.X==indices.Y||indices.X==indices.Z||indices.Y==indices.Z)return false;
    const auto origin=Detail::Point(plane.Origin),axisU=Detail::Point(plane.AxisU),axisV=Detail::Point(plane.AxisV);
    if(!Detail::ValidPoint(origin)||!Detail::ValidPoint(axisU)||!Detail::ValidPoint(axisV))return false;
    FIntervalPoint vertices[3];const int32 selected[3]={indices.X,indices.Y,indices.Z};
    for(int32 i=0;i<3;++i)
    {
        const auto& uv=plane.DomainVertices[selected[i]];
        const auto u=Interval::Exact(uv.X),v=Interval::Exact(uv.Y);
        if(!AnalyticCertified::Valid(u)||!AnalyticCertified::Valid(v))return false;
        for(int32 d=0;d<3;++d)
            vertices[i].Axis[d]=AnalyticCertified::Add(AnalyticCertified::Add(origin.Axis[d],AnalyticCertified::Multiply(axisU.Axis[d],u)),AnalyticCertified::Multiply(axisV.Axis[d],v));
        if(!Detail::ValidPoint(vertices[i]))return false;
    }
    FPlaneTriangleIntervalNet converted;
    converted.SourceId=plane.SourceId;converted.SurfaceId=plane.SurfaceId;converted.FeatureId=plane.FeatureId;converted.PrimitiveId=plane.PrimitiveId;
    converted.MaterialId=plane.MaterialId;converted.ObjectType=plane.ObjectType;converted.BlockingChannels=plane.BlockingChannels;
    converted.bQueryCollisionEnabled=plane.bQueryCollisionEnabled;converted.bRequiresCompactOptIn=plane.bRequiresCompactOptIn;converted.bAuthorityEligible=plane.bAuthorityEligible;
    converted.OriginalDomainIndices=indices;converted.Surface.DegreeU=converted.Surface.DegreeV=1;
    // U-major {a,c,b,b}: P=(1-u)(1-v)a+(1-u)vc+ub covers exactly
    // the CLOSED triangle for the fixed true affine vertices. u=1 collapses to b.
    // Interval copies may lose the repeated-b correlation, conservatively only.
    converted.Surface.ControlPoints={vertices[0],vertices[2],vertices[1],vertices[1]};
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    output=MoveTemp(converted);return true;
}
}
