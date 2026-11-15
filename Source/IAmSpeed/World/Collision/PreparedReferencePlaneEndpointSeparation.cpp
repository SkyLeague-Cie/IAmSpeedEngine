#include "PreparedReferencePlaneEndpointSeparation.h"
namespace Speed::Analytic::Prepared
{
bool FReferencePlaneEndpointSeparation::TryBound(FReferencePlaneInitialContactPtr initial,const FVector3d& target,
    C::FDistanceBudget& budget,FReferencePlaneEndpointSeparationPtr& out,FString& reason)
{
    out.Reset();reason.Reset();RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    auto refuse=[&](const TCHAR* why){reason=why;return false;};
    if(!initial||!budget.Consume())return refuse(TEXT("Initial plane proof or endpoint budget unavailable"));
    const auto geometry=initial->GetGeometry();const double radius=initial->GetRadius(),coverage=initial->GetCoverageRadius();
    int32 axis=INDEX_NONE;double sign=0;
    if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
        !C::CandidateDetail::PositiveRadius(radius)||!C::CandidateDetail::PositiveRadius(coverage)||
        !C::Detail::ValidPoint(C::Detail::Point(target))||!OriginPlaneDetail::SignedAxis(initial->GetGeometricNormal(),axis,sign))
        return refuse(TEXT("Retained exact coordinate-plane initial proof or finite supplied endpoint unavailable"));
    const uint64 r=AnalyticCertified::Bits(radius),c=AnalyticCertified::Bits(coverage),t=AnalyticCertified::Bits(target[axis]);
    const uint64 expectedSign=AnalyticCertified::Bits(sign)&AnalyticCertified::Sign;
    // Positive finite IEEE magnitudes are monotonically ordered. This signed
    // axial-height comparison needs no dot product/abs/subnormal arithmetic.
    if((c&AnalyticCertified::Magnitude)<(r&AnalyticCertified::Magnitude)||
        (t&AnalyticCertified::Sign)!=expectedSign||
        (t&AnalyticCertified::Magnitude)<(r&AnalyticCertified::Magnitude))
        return refuse(TEXT("Supplied endpoint is inward, opposite-facing, or outside retained radius coverage"));
    // Exact() encloses subnormals under DAZ/FTZ. Preserve original target bits
    // separately; no raw Min/Max classification of subnormal coordinates.
    const auto hull=C::TimeDetail::Hull(C::Detail::Point(initial->GetCenter()),C::Detail::Point(target));
    C::FIntervalBezierCenterPath path;path.ControlPoints.Add(hull);C::FReferenceCandidateSetPtr selected;
    if(!C::FReferenceCandidateSet::Select(geometry->GetSelectedUnion(),path,coverage,budget,selected)||
        !selected||selected->GetScene()!=geometry->GetSelectedUnion())
        return refuse(TEXT("Whole actual endpoint segment partition unavailable"));
    C::FReferenceCandidateSet::FClosedPlaneOriginWitness witness;
    if(!selected->ProveCoordinatePlaneOrigin(axis,budget,witness))
        return refuse(TEXT("Every candidate must be coplanar, including after the first closed origin witness"));
    TSharedPtr<FReferencePlaneEndpointSeparation,ESPMode::ThreadSafe> result(new FReferencePlaneEndpointSeparation);
    result->Initial=MoveTemp(initial);result->Partition=MoveTemp(selected);result->Witness=witness;result->Endpoint=target;
    // Candidate distance is at least axial height>=R. Every other provider is
    // excluded on the complete hull at coverage>=R. Only endpoint separation
    // follows; initial skin overlap and segment interior need not be clear.
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);out=result;return true;
}
}
