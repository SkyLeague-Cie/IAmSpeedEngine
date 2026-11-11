#pragma once
#include "ReferenceBoundsStrictSeparation.h"
namespace Speed::Analytic::Certified
{
// Stationary QUERY Box, including its actual rounded dimensions/orientation
// and both declared tolerance shells. This is not a bound on a physical Box,
// an integration path, a legacy facet kernel, a contact or a projection MTD.
inline bool BuildCanonicalFixedBoxOverlapEnvelope(const FWorldQuery& q,FIntervalPoint& out)
{
    out={};
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(q.Shape!=EQueryShape::Box||!q.bAuthorityOnly||!q.bApplyCollisionFilter||q.bExcludeAuthorityEligible||
        !AnalyticCertified::Finite(q.DomainTolerance)||q.DomainTolerance<0||
        !AnalyticCertified::Finite(q.InitialOverlapTolerance)||q.InitialOverlapTolerance<0||
        q.ReferenceNormal.ContainsNaN()||!AnalyticCertified::Finite(q.MinimumReferenceNormalDot)||
        q.MinimumReferenceNormalDot < -1||q.MinimumReferenceNormalDot>1)return false;
    for(int32 d=0;d<3;++d)
        if(AnalyticCertified::Bits(q.Start[d])!=AnalyticCertified::Bits(q.End[d]))return false;
    FIntervalPoint hull;if(!BuildFixedWorldQueryVolumeBounds(q,hull))return false;
    const auto shell=AnalyticCertified::Add(Interval{-q.DomainTolerance,q.DomainTolerance,true},
        Interval{-q.InitialOverlapTolerance,q.InitialOverlapTolerance,true});
    for(int32 d=0;d<3;++d)hull.Axis[d]=AnalyticCertified::Add(hull.Axis[d],shell);
    if(!Detail::ValidPoint(hull))return false;
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    out=hull;return true;
}
// Pure query-volume geometry only. No hit/miss of a nominal facet kernel,
// Contact, support reaction, actor identity or readiness follows from this.
inline bool BuildCanonicalFixedBoxSupportProbeEnvelope(const FWorldQuery& box,
    TConstArrayView<FWorldQuery> probes,FIntervalPoint& out)
{
    out={};
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(probes.Num()!=4)return false;
    for(double tolerance:{box.DomainTolerance,box.InitialOverlapTolerance})
    {
        const uint64 bits=AnalyticCertified::Bits(tolerance);
        if((bits&AnalyticCertified::Magnitude)>=AnalyticCertified::Infinity||
            ((bits&AnalyticCertified::Sign)&&(bits&AnalyticCertified::Magnitude)))return false;
    }
    FIntervalPoint hull;if(!BuildCanonicalFixedBoxOverlapEnvelope(box,hull))return false;
    const auto same=[](double a,double b){return AnalyticCertified::Bits(a)==AnalyticCertified::Bits(b);};
    const auto shell=AnalyticCertified::Add(Interval{-box.DomainTolerance,box.DomainTolerance,true},
        Interval{-box.InitialOverlapTolerance,box.InitialOverlapTolerance,true});
    for(const auto& ray:probes)
    {
        if(ray.Shape!=EQueryShape::Ray||!ray.HalfExtent.IsZero()||
            !same(ray.Radius,box.Radius)||!same(ray.DomainTolerance,box.DomainTolerance)||
            !same(ray.InitialOverlapTolerance,box.InitialOverlapTolerance)||
            !same(ray.MinimumReferenceNormalDot,box.MinimumReferenceNormalDot)||
            ray.TraceChannel!=box.TraceChannel||ray.ObjectTypes!=box.ObjectTypes||
            ray.BlockingObjectTypes!=box.BlockingObjectTypes||ray.RequiredSourceId!=box.RequiredSourceId||
            ray.RequiredSurfaceId!=box.RequiredSurfaceId||ray.RequiredCanonicalGroupId!=box.RequiredCanonicalGroupId||
            ray.bAllowEstablishedFaceContactAtBoundary!=box.bAllowEstablishedFaceContactAtBoundary||
            ray.bUseFiniteContactDomain!=box.bUseFiniteContactDomain||ray.bObjectQuery!=box.bObjectQuery||
            ray.bApplyCollisionFilter!=box.bApplyCollisionFilter||ray.bAuthorityOnly!=box.bAuthorityOnly||
            ray.bExcludeAuthorityEligible!=box.bExcludeAuthorityEligible||ray.bExcludeTori!=box.bExcludeTori||
            ray.bIncludeCompactPatches!=box.bIncludeCompactPatches||ray.bIncludeTriangles!=box.bIncludeTriangles)return false;
        for(int32 d=0;d<3;++d)if(!same(ray.ReferenceNormal[d],box.ReferenceNormal[d]))return false;
        if(!same(ray.Rotation.X,box.Rotation.X)||!same(ray.Rotation.Y,box.Rotation.Y)||
            !same(ray.Rotation.Z,box.Rotation.Z)||!same(ray.Rotation.W,box.Rotation.W))return false;
        FIntervalPoint volume;if(!BuildFixedWorldQueryVolumeBounds(ray,volume))return false;
        for(int32 d=0;d<3;++d)volume.Axis[d]=AnalyticCertified::Add(volume.Axis[d],shell);
        if(!Detail::ValidPoint(volume))return false;
        hull=TimeDetail::Hull(hull,volume);
    }
    if(!Detail::ValidPoint(hull))return false;
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    out=hull;return true;
}
}
