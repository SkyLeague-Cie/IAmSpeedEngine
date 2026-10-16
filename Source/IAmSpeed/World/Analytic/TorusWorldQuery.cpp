#include "TorusWorldQuery.h"
#include "IAmSpeed/SubBodies/Solid/TorusSphereSweep.h"
#include "IAmSpeed/SubBodies/Solid/TorusBoxSweep.h"
#include "IAmSpeed/SubBodies/Solid/TorusBoxGeometry.h"
#include <stdexcept>

namespace Speed::Analytic
{
bool TorusPassesFilter(const FWorldQuery& Query,const FTorusSurface& Torus)
{
    if (Query.bExcludeTori || !Torus.bQueryCollisionEnabled ||
        (Query.bAuthorityOnly && !Torus.bAuthorityEligible) ||
        (Query.bExcludeAuthorityEligible && Torus.bAuthorityEligible) ||
        (Query.RequiredSourceId && Query.RequiredSourceId!=Torus.SourceId) ||
        (Query.RequiredSurfaceId && Query.RequiredSurfaceId!=Torus.SurfaceId) ||
        (Query.RequiredCanonicalGroupId && Query.RequiredCanonicalGroupId!=Torus.SurfaceId)) return false;
    if (!Query.bApplyCollisionFilter) return true;
    if (Query.bObjectQuery) return Torus.ObjectType<64 && (Query.ObjectTypes&(uint64(1)<<Torus.ObjectType));
    return Query.TraceChannel<64 && Torus.ObjectType<64 &&
        (Torus.BlockingChannels&(uint64(1)<<Query.TraceChannel)) &&
        (Query.BlockingObjectTypes&(uint64(1)<<Torus.ObjectType));
}
namespace
{
FWorldHit MakeHit(const FWorldQuery& Query,const FTorusSurface& Torus,double Time,double Separation,
    const FVector3d& Normal,const FVector3d& Point,const FVector3d& QueryPoint,double Error)
{
    FWorldHit Hit;Hit.bHit=true;Hit.Time=Time;Hit.bStartPenetrating=Time==0. && Separation<0.;
    Hit.PenetrationDepth=Hit.bStartPenetrating ? -Separation : 0.;Hit.GeometricErrorBoundCm=Error;
    Hit.Location=FMath::Lerp(Query.Start,Query.End,Time);
    Hit.Normal=Torus.Pose.TransformVectorNoScale(Normal);
    Hit.Point=Torus.Pose.TransformPosition(Point);Hit.QueryPoint=Torus.Pose.TransformPosition(QueryPoint);
    Hit.SourceId=Torus.SourceId;Hit.SurfaceId=Torus.SurfaceId;Hit.FeatureId=Torus.FeatureId;
    Hit.PrimitiveId=Torus.PrimitiveId;Hit.CanonicalGroupId=Torus.SurfaceId;Hit.MaterialId=Torus.MaterialId;
    Hit.bSurfaceNormalMayVary=true;Hit.SurfaceFeatureKind=Speed::EContactFeatureKind::Face;
    Hit.QueryFeatureKind=Query.Shape==EQueryShape::Box ? Speed::EContactFeatureKind::Unknown : Speed::EContactFeatureKind::Face;
    return Hit;
}
}
FWorldHit SweepTorusTrajectory(const FWorldQuery& Query,const FTorusSurface& Torus,
    TFunctionRef<FTransform(double)> WorldPoseAt,double MotionBoundCm)
{
    if (!TorusPassesFilter(Query,Torus)) return {};
    const auto LocalPoseAt=[&](double Time)
    {
        const FTransform Pose=WorldPoseAt(Time);
        return FTransform(Torus.Pose.GetRotation().Inverse()*Pose.GetRotation(),
            Torus.Pose.InverseTransformPosition(Pose.GetTranslation()));
    };
    const auto Contact=Geometry::SweepTorusShapeTrajectory(Torus.Geometry,Query.Shape!=EQueryShape::Box,
        Query.Shape==EQueryShape::Ray ? 0. : Query.Radius,Query.HalfExtent,
        LocalPoseAt,MotionBoundCm,Query.InitialOverlapTolerance);
    if (Contact.Result==Geometry::ETorusSweepResult::Miss) return {};
    if (Contact.Result!=Geometry::ETorusSweepResult::Hit)
    {
        UE_LOG(LogTemp,Error,TEXT("[TorusTrajectoryUnresolved] Shape=%d R=%.17g r=%.17g Radius=%.17g Half=%s Bound=%.17g OverlapTolerance=%.17g Surface=%llu TorusPose=%s P0=%s P25=%s P50=%s P75=%s P1=%s"),
            int32(Query.Shape),Torus.Geometry.MajorRadius,Torus.Geometry.TubeRadius,Query.Radius,*Query.HalfExtent.ToString(),
            MotionBoundCm,Query.InitialOverlapTolerance,Torus.SurfaceId,*Torus.Pose.ToString(),
            *LocalPoseAt(0.).ToString(),*LocalPoseAt(.25).ToString(),*LocalPoseAt(.5).ToString(),*LocalPoseAt(.75).ToString(),*LocalPoseAt(1.).ToString());
        throw std::runtime_error("Torus trajectory query invalid or unresolved; refusing collision miss");
    }
    double Separation=Contact.Separation;
    FVector3d SurfacePoint=Contact.SurfacePoint;
    if (Query.Shape==EQueryShape::Box)
    {
        Separation=0.;
        if (Contact.Time==0. && Contact.Separation<0.)
        {
            // The torus contains the radius-r sphere at the witness centreline
            // point, so r-distance(point,OBB) is a necessary translation bound.
            // Independently certify that translating along this normal clears
            // the ENTIRE torus. Only then is the upper vector an MTD within the
            // existing spatial error bound; a negative distance alone is not.
            const double LowerDepth=-Contact.Separation;
            const double UpperDepth=LowerDepth+Contact.ErrorBoundCm;
            FTransform Shifted=LocalPoseAt(0.);
            Shifted.AddToTranslation(Contact.Normal*UpperDepth);
            Geometry::FTorusBoxDistance Certificate;
            if (!Geometry::TryTorusBoxDistance(Torus.Geometry,Shifted.GetTranslation(),Shifted.GetRotation(),
                    Query.HalfExtent,1.e-5,8192,Certificate) || !Certificate.bConverged || Certificate.LowerSeparation<0.)
                throw std::runtime_error("Initial torus/box MTD not certified within spatial bound");
            Separation=-UpperDepth;
            // Static witness has at most ErrorBoundCm outward displacement;
            // the box witness stays exact and encodes the certified correction.
            SurfacePoint=Contact.BoxPoint+Contact.Normal*UpperDepth;
        }
    }
    FWorldHit Hit=MakeHit(Query,Torus,Contact.Time,Separation,Contact.Normal,
        SurfacePoint,Contact.BoxPoint,Contact.ErrorBoundCm);
    Hit.Location=WorldPoseAt(Contact.Time).GetTranslation();
    return Hit;
}
FWorldHit SweepTorusSurface(const FWorldQuery& Query,const FTorusSurface& Torus)
{
    if (!TorusPassesFilter(Query,Torus)) return {};
    if (Query.Shape==EQueryShape::Box)
    {
        const auto PoseAt=[&](double Time){return FTransform(Query.Rotation,FMath::Lerp(Query.Start,Query.End,Time));};
        return SweepTorusTrajectory(Query,Torus,PoseAt,(Query.End-Query.Start).Size());
    }
    const FVector3d Start=Torus.Pose.InverseTransformPosition(Query.Start),End=Torus.Pose.InverseTransformPosition(Query.End);
    const double Radius=Query.Shape==EQueryShape::Ray ? 0. : Query.Radius;
    const auto Contact=Geometry::SweepTorusSphere(Torus.Geometry,Start,End,Radius,Query.InitialOverlapTolerance);
    if (Contact.Result==Geometry::ETorusSweepResult::Miss) return {};
    if (Contact.Result!=Geometry::ETorusSweepResult::Hit)
        throw std::runtime_error("Torus sphere query invalid or unresolved; refusing collision miss");
    const FVector3d Centre=FMath::Lerp(Start,End,Contact.Time);
    return MakeHit(Query,Torus,Contact.Time,Contact.Contact.Separation,Contact.Contact.Normal,
        Contact.Contact.SurfacePoint,Centre-Contact.Contact.Normal*Radius,1.e-5);
}
}
