#pragma once
#include "FloorNativeSolidSweepBounds.h"
namespace Speed::Analytic::Certified
{
// Retains BOTH motion models, the captured pose, the actual rounded World-call
// shape/chord and the physical/support shape. This volume is geometric only;
// a partition still needs policies, finite-domain and live-request binding.
inline bool BuildNativeSolidSweepVolume(const PreparedExecution::FNativeSolidStaticValues& values,
    FFloorNativeSolidModel& physical,FIntervalPoint& output)
{
    output={};physical={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!BuildFloorNativeSolidModel(values,physical))return false;
    auto hull=physical.BoundSupportHull();const auto reference=physical.BoundSupportHull(false);
    if(!Detail::ValidPoint(hull)||!Detail::ValidPoint(reference))return false;
    hull=TimeDetail::Hull(hull,reference);
    auto join=[&](const FIntervalPoint& point)
    {if(!Detail::ValidPoint(point))return false;hull=TimeDetail::Hull(hull,point);return Detail::ValidPoint(hull);};
    FIntervalPoint chord;if(!BuildFixedWorldQueryVolumeBounds(values.Query,chord)||!join(chord))return false;
    auto captured=values.Query;captured.Start=values.BodyState.Location;captured.End=captured.Start;
    captured.Rotation=values.BodyState.Rotation;
    using PreparedExecution::ENativeSolidQueryShape;
    auto rounded=values;
    if(values.Shape==ENativeSolidQueryShape::Sphere)
    {
        if(values.Query.Shape!=EQueryShape::Sphere||values.Query.Radius!=double(values.QueryRadius))return false;
        captured.Radius=values.SupportRadius;rounded.SupportRadius=double(values.QueryRadius);
    }
    else if(values.Shape==ENativeSolidQueryShape::Box)
    {
        if(values.Query.Shape!=EQueryShape::Box||values.Query.HalfExtent!=values.QueryHalfExtent)return false;
        captured.HalfExtent=values.PhysicalHalfExtent;rounded.PhysicalHalfExtent=values.QueryHalfExtent;
    }
    else return false;
    FIntervalPoint pose;if(!BuildFixedWorldQueryVolumeBounds(captured,pose)||!join(pose))return false;
    FFloorNativeSolidModel queryModel;
    if(!BuildFloorNativeSolidModel(rounded,queryModel)||!join(queryModel.BoundSupportHull())||
        !join(queryModel.BoundSupportHull(false)))return false;
    // Fixed query footprint retains BOTH normalized SAT axes and raw rotated
    // corners, also over accelerated centers. Never infer that a chord AABB
    // already encloses the same footprint on an interior motion excursion.
    auto footprintQuery=values.Query;footprintQuery.Start=FVector3d::ZeroVector;
    footprintQuery.End=FVector3d::ZeroVector;footprintQuery.Rotation=values.BodyState.Rotation;
    FIntervalPoint footprint;if(!BuildFixedWorldQueryVolumeBounds(footprintQuery,footprint))return false;
    for(bool consumed:{false,true})
    {
        auto movingFootprint=physical.BoundCenter(0.f,values.Horizon,consumed);
        for(int32 d=0;d<3;++d)movingFootprint.Axis[d]=AnalyticCertified::Add(movingFootprint.Axis[d],footprint.Axis[d]);
        if(!join(movingFootprint))return false;
    }
    // These are existing query shells / the captured Sphere budget. Adding
    // them conservatively to a volume creates no Box margin or +0 contact.
    const double shells[]={values.Query.DomainTolerance,values.Query.InitialOverlapTolerance,
        values.Shape==ENativeSolidQueryShape::Sphere?double(values.SphereMargin):0.};
    for(double shell:shells)
    {
        const uint64 bits=AnalyticCertified::Bits(shell);
        if((bits&AnalyticCertified::Magnitude)>=AnalyticCertified::Infinity||
            ((bits&AnalyticCertified::Sign)&&(bits&AnalyticCertified::Magnitude)))return false;
        for(int32 d=0;d<3;++d)hull.Axis[d]=AnalyticCertified::Add(hull.Axis[d],Interval{-shell,shell,true});
    }
    if(!Detail::ValidPoint(hull))return false;output=hull;return true;
}
}
