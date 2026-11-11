#include "PreparedOriginPlaneContact.h"
#include "IAmSpeed/Base/SUtils.h"
#include "IAmSpeed/World/Analytic/ScalarSphereContactBounds.h"
namespace Speed::Analytic::Prepared
{
FOriginPlaneContact BoundOriginPlaneInitialContact(const FScenePtr& input,
    const FVector3d& center,double radius,C::FDistanceBudget& budget)
{
    Speed::Analytic::RequireCurrentAnalyticNumericContext(Speed::Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    FOriginPlaneContact result;
    if(!input||!input->GetUnion()||!C::Detail::ValidPoint(C::Detail::Point(center))||!C::CandidateDetail::PositiveRadius(radius)||
        (AnalyticCertified::Bits(radius)&AnalyticCertified::Magnitude)<0x0020000000000000ull)return result;
    const auto owners=input->GetOwners();
    for(const auto& plane:input->GetPlanes())
    {
        if(!budget.Consume())return {};
        bool originZero=true;for(int32 d=0;d<3;++d)originZero&=(AnalyticCertified::Bits(plane.Origin[d])&AnalyticCertified::Magnitude)==0;
        int32 n,u,v;double sn,su,sv;
        if(!originZero||!plane.bQueryCollisionEnabled||!OriginPlaneDetail::SignedAxis(plane.Normal,n,sn)||
            !OriginPlaneDetail::SignedAxis(plane.AxisU,u,su)||!OriginPlaneDetail::SignedAxis(plane.AxisV,v,sv)||n==u||n==v||u==v)continue;
        const double height=sn*center[n];
        // Sterbenz: subtraction radius-height is exact in this normal range.
        // Refuse deep or opposite-side overlaps rather than invent their MTD.
        double depth=0;if(!OriginPlaneDetail::ShallowDepth(height,radius,depth))continue;
        const FVector2d uv(su*center[u],sv*center[v]);
        if(!OriginPlaneDetail::StrictInside(plane.DomainVertices,uv,budget))continue;
        FVector3d target=center;target[n]=sn*radius;
        C::FIntervalPoint correctionHull;
        for(int32 d=0;d<3;++d)correctionHull.Axis[d]={FMath::Min(center[d],target[d]),FMath::Max(center[d],target[d]),true};
        C::FIntervalBezierCenterPath path;path.ControlPoints.Add(correctionHull);C::FReferenceCandidateSetPtr selected;
        if(!C::FReferenceCandidateSet::Select(input->GetUnion(),path,radius,budget,selected))return {};
        const auto candidates=selected->GetCandidateIndices();bool onlyThisPlane=!candidates.IsEmpty();
        for(int32 index:candidates)
        {
            if(!budget.Consume()||(index<0||index>=owners.Num()))return {};
            const auto& owner=owners[index];
            if(!owner.FromPlane||owner.SourceId!=plane.SourceId||owner.SurfaceId!=plane.SurfaceId||owner.FeatureId!=plane.FeatureId||
                owner.PrimitiveId!=plane.PrimitiveId||!owner.QueryEnabled){onlyThisPlane=false;break;}
        }
        if(!onlyThisPlane)continue;
        result.Status=depth>0?EOriginPlaneContactStatus::InitialFaceOverlap:EOriginPlaneContactStatus::InitialFaceTangency;
        auto& hit=result.Hit;hit.bHit=true;hit.bStartPenetrating=depth>0;hit.Time=0;hit.PenetrationDepth=depth;hit.Location=center;hit.Normal=plane.Normal;
        hit.Point=center;hit.Point[n]=0;hit.QueryPoint=center;hit.QueryPoint[n]=sn*(height-radius);
        hit.QueryFeatureKind=Speed::EContactFeatureKind::Vertex;hit.SurfaceFeatureKind=Speed::EContactFeatureKind::Face;
        hit.SourceId=plane.SourceId;hit.SurfaceId=plane.SurfaceId;hit.FeatureId=plane.FeatureId;hit.PrimitiveId=plane.PrimitiveId;hit.MaterialId=plane.MaterialId;
        result.TubeTangencyCenter=target;result.SamePlaneCandidates=candidates.Num();result.OtherProvidersExcluded=selected->GetExcludedIndices().Num();
        Speed::Analytic::RequireCurrentAnalyticNumericContext(Speed::Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);return result;
    }
    return result;
}

FOriginPlaneTangencyRange BoundOriginPlaneTangencyRange(const FScenePtr& scene,
    FVector3d position,FVector3d velocity,FVector3d acceleration,float horizon,double collisionRadius,double physicalRadius,
    float lastSeconds,C::FDistanceBudget& budget)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!scene||!C::CandidateDetail::PositiveRadius(collisionRadius)||!C::CandidateDetail::PositiveRadius(physicalRadius)||
        collisionRadius<physicalRadius||!budget.Consume())return {};
    C::FZeroOffsetKinematicCenterReference model;
    if(!C::BuildZeroOffsetKinematicCenterReference(position,velocity,acceleration,horizon,FVector3d::ZeroVector,FVector3d::ZeroVector,model))return {};
    const auto initial=BoundOriginPlaneInitialContact(scene,position,physicalRadius,budget);
    if(initial.Status!=EOriginPlaneContactStatus::InitialFaceTangency)return {};
    const auto& id=initial.Hit;const FBoundedPlane* plane=nullptr;
    for(const auto& frame:scene->GetPlanes())
    {
        if(!budget.Consume())return {};
        if(frame.SourceId==id.SourceId&&frame.SurfaceId==id.SurfaceId&&frame.FeatureId==id.FeatureId&&frame.PrimitiveId==id.PrimitiveId)
        {if(plane)return {};plane=&frame;}
    }
    int32 n,u,v;double sn,su,sv;
    if(!plane||!OriginPlaneDetail::SignedAxis(plane->Normal,n,sn)||!OriginPlaneDetail::SignedAxis(plane->AxisU,u,su)||
        !OriginPlaneDetail::SignedAxis(plane->AxisV,v,sv)||n==u||n==v||u==v||
        sn*position[n]!=physicalRadius||(AnalyticCertified::Bits(velocity[n])&AnalyticCertified::Magnitude)||
        (AnalyticCertified::Bits(acceleration[n])&AnalyticCertified::Magnitude))return {};
    auto hull=C::BoundConsumedCenterBetweenFloatSeconds(model,0.f,lastSeconds);
    if(!C::Detail::ValidPoint(hull))return {}; // includes actual float t*t overflow/NaN refusal
    // Exact normal-zero PVA and finite t*t make the actual SBox normal DAG
    // identically +/-Rp+zero+zero for EVERY second. No runtime pose is changed.
    // Check the actual compiled routine at both ends as an additional guard.
    for(float seconds:{0.f,lastSeconds})
    {
        const auto actual=Speed::SBox::AdvancePosition(position,velocity,acceleration,seconds);
        if(!C::ScalarContactDetail::Finite(actual)||AnalyticCertified::Bits(actual[n])!=AnalyticCertified::Bits(position[n]))return {};
        for(int32 d=0;d<3;++d)if(actual[d]<hull.Axis[d].Lower||actual[d]>hull.Axis[d].Upper)return {};
    }
    hull.Axis[n]=C::Interval::Exact(position[n]);
    auto signedInterval=[](C::Interval value,double sign)
    {return sign>0?value:C::Interval{-value.Upper,-value.Lower,value.Known};};
    if(!OriginPlaneDetail::StrictRectangleInside(plane->DomainVertices,signedInterval(hull.Axis[u],su),signedInterval(hull.Axis[v],sv),budget))return {};
    C::FIntervalBezierCenterPath path;path.ControlPoints.Add(hull);C::FReferenceCandidateSetPtr selected;
    if(!C::FReferenceCandidateSet::Select(scene->GetUnion(),path,collisionRadius,budget,selected)||selected->GetCandidateIndices().IsEmpty())return {};
    const auto owners=scene->GetOwners();
    for(int32 index:selected->GetCandidateIndices())
    {
        if(!budget.Consume()||index<0||index>=owners.Num())return {};const auto& owner=owners[index];
        if(!owner.FromPlane||owner.SourceId!=plane->SourceId||owner.SurfaceId!=plane->SurfaceId||owner.FeatureId!=plane->FeatureId||
            owner.PrimitiveId!=plane->PrimitiveId||!owner.QueryEnabled||owner.MaterialId!=plane->MaterialId||
            owner.ObjectType!=plane->ObjectType||owner.BlockingChannels!=plane->BlockingChannels||
            owner.CompactOptIn!=plane->bRequiresCompactOptIn||owner.AuthorityEligible!=plane->bAuthorityEligible)return {};
    }
    FOriginPlaneTangencyRange result;result.Status=EOriginPlaneTangencyRangeStatus::PhysicalTangencyOverConsumedRange;
    result.Scene=scene;result.Reference=MoveTemp(model);result.Selected=MoveTemp(selected);result.InitialPhysicalWitness=initial;
    result.Position=position;result.Velocity=velocity;result.Acceleration=acceleration;
    result.CollisionRadiusBits=AnalyticCertified::Bits(collisionRadius);result.PhysicalRadiusBits=AnalyticCertified::Bits(physicalRadius);
    result.Horizon=horizon;result.LastSeconds=lastSeconds;
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);return result;
}
}
