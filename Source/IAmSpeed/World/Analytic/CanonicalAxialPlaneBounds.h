#pragma once
#include "FixedShapeQueryBounds.h"
#include "IAmSpeed/World/Collision/PreparedIntervalScene.h"
namespace Speed::Analytic::Certified
{
// A finite interior/exclusion description of ONE exact axial native plane
// and ONE actual retained canonical triangle. Never an infinite plane, a
// residual-facet miss, a Contact, physical readiness or a support release.
struct FCanonicalAxialPlaneBounds
{
    FBoundedPlane Plane;FIntervalTensorSurface Triangle;FIntervalPoint Envelope;
    FReferenceBoundsCandidateSetPtr Partition;int32 NormalAxis=INDEX_NONE,UAxis=INDEX_NONE,VAxis=INDEX_NONE;
    int32 ProviderIndex=INDEX_NONE;uint64 CanonicalGroupId=0;bool Complete=false;
};
namespace CanonicalPlaneStaticMemo
{
IAMSPEED_API bool RuntimeEnabled();
IAMSPEED_API void PublishObservation();
IAMSPEED_API bool NativeInterior(const FCanonicalAxialPlaneBounds& value,const FIntervalPoint& hull,FDistanceBudget& budget);
}
namespace CanonicalAxialPlaneDetail
{
inline int32 Axis(const FVector3d& v)
{
    int32 axis=INDEX_NONE;
    for(int32 d=0;d<3;++d)if(AnalyticCertified::Bits(v[d])&AnalyticCertified::Magnitude)
    {
        if(axis!=INDEX_NONE||(AnalyticCertified::Bits(v[d])!=AnalyticCertified::Bits(1.)&&
            AnalyticCertified::Bits(v[d])!=AnalyticCertified::Bits(-1.)))return INDEX_NONE;
        axis=d;
    }
    return axis;
}
inline bool Policy(const FWorldQuery& q,const Prepared::FOwner& o)
{
    if((q.RequiredSourceId&&q.RequiredSourceId!=o.SourceId)||(q.RequiredSurfaceId&&q.RequiredSurfaceId!=o.SurfaceId)||
        (q.RequiredCanonicalGroupId&&q.RequiredCanonicalGroupId!=o.CanonicalGroupId)||!o.QueryEnabled||
        (o.CompactOptIn&&!q.bIncludeCompactPatches)||o.ObjectType>=64)return false;
    return q.bObjectQuery?(q.ObjectTypes&(uint64(1)<<o.ObjectType))!=0:
        q.TraceChannel<64&&(o.BlockingChannels&(uint64(1)<<q.TraceChannel))&&(q.BlockingObjectTypes&(uint64(1)<<o.ObjectType));
}
inline Interval Cross(Interval ax,Interval ay,Interval bx,Interval by)
{return AnalyticCertified::Subtract(AnalyticCertified::Multiply(ax,by),AnalyticCertified::Multiply(ay,bx));}
inline Interval Orient(const FIntervalPoint& a,const FIntervalPoint& b,const FIntervalPoint& p,int32 u,int32 v)
{
    return Cross(AnalyticCertified::Subtract(b.Axis[u],a.Axis[u]),AnalyticCertified::Subtract(b.Axis[v],a.Axis[v]),
        AnalyticCertified::Subtract(p.Axis[u],a.Axis[u]),AnalyticCertified::Subtract(p.Axis[v],a.Axis[v]));
}
inline bool Positive(Interval v,int32 sign)
{return AnalyticCertified::Valid(v)&&(sign>0?v.Lower>0:v.Upper<0);}
inline bool SamePoint(const FIntervalPoint& a,const FIntervalPoint& b)
{
    for(int32 d=0;d<3;++d)if(a.Axis[d].Known!=b.Axis[d].Known||
        AnalyticCertified::Bits(a.Axis[d].Lower)!=AnalyticCertified::Bits(b.Axis[d].Lower)||
        AnalyticCertified::Bits(a.Axis[d].Upper)!=AnalyticCertified::Bits(b.Axis[d].Upper))return false;
    return true;
}
inline bool ExactPlaneTriangle(const FCanonicalAxialPlaneBounds& value,const FIntervalTensorSurface& triangle)
{
    if(value.NormalAxis<0||value.NormalAxis>2||triangle.DegreeU!=1||triangle.DegreeV!=1||
        triangle.ControlPoints.Num()!=4||!SamePoint(triangle.ControlPoints[2],triangle.ControlPoints[3]))return false;
    for(const auto& cp:triangle.ControlPoints)
    {
        const auto& c=cp.Axis[value.NormalAxis];
        if(!Detail::ValidPoint(cp)||!AnalyticCertified::Valid(c)||
            AnalyticCertified::Bits(c.Lower)!=AnalyticCertified::Bits(value.Plane.Origin[value.NormalAxis])||
            AnalyticCertified::Bits(c.Upper)!=AnalyticCertified::Bits(value.Plane.Origin[value.NormalAxis]))return false;
    }
    return true;
}
inline bool TriangleInterior(const FCanonicalAxialPlaneBounds& value,const FIntervalPoint& hull)
{
    if(!Detail::ValidPoint(hull)||value.NormalAxis<0||value.UAxis<0||value.VAxis<0||
        value.Triangle.ControlPoints.Num()!=4)return false;
    const auto& cp=value.Triangle.ControlPoints;const int32 indices[]={0,2,1};
    const auto area=Orient(cp[0],cp[2],cp[1],value.UAxis,value.VAxis);
    const int32 sign=Positive(area,1)?1:Positive(area,-1)?-1:0;if(!sign)return false;
    for(int32 k=0;k<3;++k)if(!Positive(Orient(cp[indices[k]],cp[indices[(k+1)%3]],hull,value.UAxis,value.VAxis),sign))return false;
    return true;
}
// Equal fixed native coordinate literals, never equality of an interval
// family. Integer bits avoid DAZ inventing equality of unequal subnormals.
inline bool SameFiniteNativeCoordinate(double a,double b)
{
    namespace A=::AnalyticCertified;
    if(!A::Finite(a)||!A::Finite(b))return false;
    uint64 x=A::Bits(a),y=A::Bits(b);
    if(!(x&A::Magnitude))x=0;if(!(y&A::Magnitude))y=0;
    return x==y;
}
inline bool ProveFixedAxisCollinearity(const FVector2d& a,const FVector2d& b,const FVector2d& p)
{
    namespace A=::AnalyticCertified;
    if(!A::Finite(a.X)||!A::Finite(a.Y)||!A::Finite(b.X)||!A::Finite(b.Y)||!A::Finite(p.X)||!A::Finite(p.Y))return false;
    const bool sameX=SameFiniteNativeCoordinate(a.X,b.X),sameY=SameFiniteNativeCoordinate(a.Y,b.Y);
    if(sameX&&sameY)return false; // No zero edge can support a strict halfspace.
    return (sameX&&SameFiniteNativeCoordinate(a.X,p.X))||(sameY&&SameFiniteNativeCoordinate(a.Y,p.Y));
}
inline bool NativeInterior(const FCanonicalAxialPlaneBounds& value,const FIntervalPoint& hull,FDistanceBudget& budget,
    bool enableStaticMemo=CanonicalPlaneStaticMemo::RuntimeEnabled())
{
    if(enableStaticMemo)return CanonicalPlaneStaticMemo::NativeInterior(value,hull,budget);
    const auto& p=value.Plane;const auto n=p.DomainVertices.Num();
    if(n<3||!Detail::ValidPoint(hull)||value.UAxis<0||value.VAxis<0)return false;
    FIntervalPoint local;local.Axis[2]=Interval::Exact(0);
    local.Axis[0]=AnalyticCertified::Multiply(AnalyticCertified::Subtract(hull.Axis[value.UAxis],
        Interval::Exact(p.Origin[value.UAxis])),Interval::Exact(p.AxisU[value.UAxis]));
    local.Axis[1]=AnalyticCertified::Multiply(AnalyticCertified::Subtract(hull.Axis[value.VAxis],
        Interval::Exact(p.Origin[value.VAxis])),Interval::Exact(p.AxisV[value.VAxis]));
    if(!Detail::ValidPoint(local)||local.Axis[0].Lower<=-p.HalfExtents.X||local.Axis[0].Upper>=p.HalfExtents.X||
        local.Axis[1].Lower<=-p.HalfExtents.Y||local.Axis[1].Upper>=p.HalfExtents.Y)return false;
    const auto vertex=[&](int32 i){return Detail::Point(FVector3d(p.DomainVertices[i].X,p.DomainVertices[i].Y,0));};
    int32 orientation=0;
    for(int32 k=0;k<n;++k)
    {
        const int32 next=(k+1)%n;const auto a=vertex(k),b=vertex(next);
        if(SameFiniteNativeCoordinate(p.DomainVertices[k].X,p.DomainVertices[next].X)&&
            SameFiniteNativeCoordinate(p.DomainVertices[k].Y,p.DomainVertices[next].Y))return false;
        // Convexity means EVERY other vertex is on the same side of EACH
        // edge. Proved redundant axis-collinear native literals may be skipped;
        // concavity and every unproved zero-crossing turn remain refused.
        for(int32 j=0;j<n;++j)if(j!=k&&j!=next)
        {
            if(!budget.Consume())return false;
            if(ProveFixedAxisCollinearity(p.DomainVertices[k],p.DomainVertices[next],p.DomainVertices[j]))continue;
            const auto turn=Orient(a,b,vertex(j),0,1);
            const int32 sign=Positive(turn,1)?1:Positive(turn,-1)?-1:0;
            if(!sign||(orientation&&sign!=orientation))return false;orientation=sign;
        }
        if(!budget.Consume()||!orientation||!Positive(Orient(a,b,local,0,1),orientation))return false;
    }
    return true;
}
inline bool OwnerIsPlane(const Prepared::FOwner& o,const FBoundedPlane& p,uint64 group)
{
    return o.FromPlane&&o.SourceId==p.SourceId&&o.SurfaceId==p.SurfaceId&&o.FeatureId==p.FeatureId&&
        o.PrimitiveId==p.PrimitiveId&&o.MaterialId==p.MaterialId&&o.CanonicalGroupId==group&&
        o.ObjectType==p.ObjectType&&o.BlockingChannels==p.BlockingChannels&&o.QueryEnabled==p.bQueryCollisionEnabled&&
        o.CompactOptIn==p.bRequiresCompactOptIn&&o.AuthorityEligible==p.bAuthorityEligible;
}
}
inline bool CanonicalAxialPlaneContains(const FCanonicalAxialPlaneBounds& value,const FVector3d& point,FDistanceBudget& budget)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!value.Complete||value.NormalAxis<0||value.NormalAxis>2||point.ContainsNaN()||AnalyticCertified::Bits(point[value.NormalAxis])!=
        AnalyticCertified::Bits(value.Plane.Origin[value.NormalAxis]))return false;
    const auto hull=Detail::Point(point);
    return CanonicalAxialPlaneDetail::ExactPlaneTriangle(value,value.Triangle)&&
        CanonicalAxialPlaneDetail::TriangleInterior(value,hull)&&CanonicalAxialPlaneDetail::NativeInterior(value,hull,budget)&&
        value.Plane.ContainsProjectedPoint(point,0);
}
inline bool CloseCanonicalAxialPlaneBounds(const FIndexedIntervalUnionPtr& scene,const Prepared::FScenePtr& nominal,
    const FWorldQuery& q,const FIntervalPoint& hull,FDistanceBudget& budget,FCanonicalAxialPlaneBounds& out,
    bool exclusiveSourceSurface=false)
{
    out={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    using namespace CanonicalAxialPlaneDetail;
    if(!scene||!nominal||scene->Num()!=nominal->GetOwners().Num()||!Detail::ValidPoint(hull)||
        !q.bAuthorityOnly||!q.bApplyCollisionFilter||q.bExcludeAuthorityEligible||
        !FMath::IsFinite(q.DomainTolerance)||q.DomainTolerance<0||
        !FMath::IsFinite(q.InitialOverlapTolerance)||q.InitialOverlapTolerance<0||
        (exclusiveSourceSurface&&(!q.RequiredSourceId||!q.RequiredSurfaceId||q.DomainTolerance!=0)))return false;
    FCanonicalAxialPlaneBounds value;value.Envelope=hull;
    if(!FReferenceBoundsCandidateSet::Select(scene,hull,budget,value.Partition))return false;
    const auto owners=nominal->GetOwners();int32 first=INDEX_NONE;
    for(int32 index:value.Partition->GetCandidateIndices())
    {
        if(!budget.Consume())return false;
        if(Policy(q,owners[index])){if(!owners[index].FromPlane)return false;first=index;break;}
    }
    if(first==INDEX_NONE)return false;
    const auto& owner=owners[first];int32 planes=0;
    for(const auto& p:nominal->GetPlanes())if(p.PrimitiveId==owner.PrimitiveId)
    {if(!budget.Consume()||++planes!=1||!p.IsValid())return false;value.Plane=p;}
    if(planes!=1)return false;
    value.NormalAxis=Axis(value.Plane.Normal);value.UAxis=Axis(value.Plane.AxisU);value.VAxis=Axis(value.Plane.AxisV);
    value.CanonicalGroupId=owner.CanonicalGroupId;
    if(value.NormalAxis<0||value.UAxis<0||value.VAxis<0||value.NormalAxis==value.UAxis||
        value.NormalAxis==value.VAxis||value.UAxis==value.VAxis||!NativeInterior(value,hull,budget))return false;
    for(int32 index:value.Partition->GetCandidateIndices())
        if(!budget.Consume()||(Policy(q,owners[index])&&!OwnerIsPlane(owners[index],value.Plane,value.CanonicalGroupId)))return false;
    bool interior=false;
    for(int32 index=0;index<owners.Num();++index)
    {
        if(!budget.Consume())return false;const auto& o=owners[index];
        // The actual native refiner retains only Source/Surface restrictions.
        // For its ray the pair must name this unique native plane EXHAUSTIVELY,
        // not merely among master-AABB candidates or their rounded facettes.
        if(exclusiveSourceSurface&&Policy(q,o)&&!OwnerIsPlane(o,value.Plane,value.CanonicalGroupId))return false;
        if(o.PrimitiveId!=value.Plane.PrimitiveId)continue;
        if(!OwnerIsPlane(o,value.Plane,value.CanonicalGroupId)||!Policy(q,o))return false;
        FIntervalTensorSurface triangle;
        if(!scene->CopySurfaceForQuery(index,budget,triangle)||!ExactPlaneTriangle(value,triangle))return false;
        auto candidate=value;candidate.Triangle=MoveTemp(triangle);
        if(!interior&&TriangleInterior(candidate,hull))
        {value.Triangle=MoveTemp(candidate.Triangle);value.ProviderIndex=index;interior=true;}
    }
    if(!interior)return false;value.Complete=true;out=MoveTemp(value);return true;
}
}
