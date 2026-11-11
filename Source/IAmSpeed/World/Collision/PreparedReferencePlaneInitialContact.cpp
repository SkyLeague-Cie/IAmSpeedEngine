#include "PreparedReferencePlaneInitialContact.h"
namespace Speed::Analytic::Prepared
{
bool FReferencePlaneInitialContact::TryBound(FPreparedGeometrySelectionPtr geometry,const FVector3d& center,
    double radius,double coverageRadius,C::FDistanceBudget& budget,FReferencePlaneInitialContactPtr& out,FString& reason)
{
    out.Reset();reason.Reset();RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    auto refuse=[&](const TCHAR* text){reason=text;return false;};
    if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
        !C::Detail::ValidPoint(C::Detail::Point(center))||!C::CandidateDetail::PositiveRadius(coverageRadius)||coverageRadius<radius)
        return refuse(TEXT("Retained reference model or finite radius coverage unavailable"));
    for(int32 axis=0;axis<3;++axis)
    {
        if(!budget.Consume())return refuse(TEXT("Initial plane budget exhausted"));
        bool projectionIsOrigin=true;
        for(int32 d=0;d<3;++d)if(d!=axis)projectionIsOrigin&=(AnalyticCertified::Bits(center[d])&AnalyticCertified::Magnitude)==0;
        const double sign=(AnalyticCertified::Bits(center[axis])&AnalyticCertified::Sign)?-1.:1.;
        const double height=sign*center[axis];double depth=0;
        if(!projectionIsOrigin||!OriginPlaneDetail::ShallowDepth(height,radius,depth))continue;
        FVector3d target=center;target[axis]=sign*radius;
        C::FIntervalPoint hull;
        for(int32 d=0;d<3;++d)hull.Axis[d]={FMath::Min(center[d],target[d]),FMath::Max(center[d],target[d]),true};
        C::FIntervalBezierCenterPath path;path.ControlPoints.Add(hull);
        C::FReferenceCandidateSetPtr selected;
        if(!C::FReferenceCandidateSet::Select(geometry->GetSelectedUnion(),path,coverageRadius,budget,selected))
            return refuse(TEXT("Whole correction segment partition unavailable"));
        C::FReferenceCandidateSet::FClosedPlaneOriginWitness witness;
        if(!selected||selected->GetScene()!=geometry->GetSelectedUnion()||!selected->ProveCoordinatePlaneOrigin(axis,budget,witness))continue;
        TSharedPtr<FReferencePlaneInitialContact,ESPMode::ThreadSafe> result(new FReferencePlaneInitialContact);
        result->Geometry=MoveTemp(geometry);result->Selected=MoveTemp(selected);result->Witness=witness;
        result->Center=center;result->Target=target;result->Normal[axis]=sign;
        result->Radius=radius;result->CoverageRadius=coverageRadius;result->Depth=depth;
        result->Kind=depth>0?EReferencePlaneInitialKind::Overlap:EReferencePlaneInitialKind::Tangency;
        // Every selected point is in the plane, whose distance is the height;
        // the origin is actually on a closed provider domain. The other nets
        // are strictly excluded along the WHOLE segment at coverageRadius>=R.
        // Thus target is tangent and no shorter displacement can remove the
        // overlap with that actual origin point (triangle inequality).
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        out=result;return true;
    }
    return refuse(TEXT("No exact coordinate-plane group/origin witness covers this initial correction"));
}
}
