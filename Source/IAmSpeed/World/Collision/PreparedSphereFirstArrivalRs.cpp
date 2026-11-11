#include "PreparedSphereFirstArrivalRs.h"
#include "PreparedSweepEvidence.h"
namespace Speed::Analytic::Prepared
{
bool FReferenceSphereFirstArrivalRs::TryEvaluate(FReferenceSphereInitialTubeStatePtr initial,float horizon,
    const C::FReferenceSphereOptions& options,C::FDistanceBudget& budget,
    FReferenceSphereFirstArrivalRsPtr& out,FString& reason,C::FReferenceSphereEvaluation* partial)
{
    out.Reset();reason.Reset();if(partial)*partial={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    const auto refuse=[&](const TCHAR* why){reason=why;return false;};
    if(!initial||initial->GetKind()!=EReferenceSphereRsState::SeparatedRs||!budget.Consume())
        return refuse(TEXT("FirstArrivalRs requires a separated retained point, not support/overlap"));
    const auto closest=initial->GetClosestAtHeight();const auto geometry=closest?closest->GetGeometry():nullptr;
    const auto& values=initial->GetInput();int32 axis=INDEX_NONE;double sign=0;
    if(!closest||!geometry||!geometry->HasValidAliases()||
        !OriginPlaneDetail::SignedAxis(closest->GetGeometricNormal(),axis,sign)||
        AnalyticCertified::Bits(closest->GetCoverageRadius())!=AnalyticCertified::Bits(double(values.QueryRadius)))
        return refuse(TEXT("FirstArrivalRs geometry/plane/retained Rc identity differs"));
    for(int32 d=0;d<3;++d)
        if(d!=axis&&((AnalyticCertified::Bits(values.State.Velocity[d])|AnalyticCertified::Bits(values.State.Acceleration[d]))&AnalyticCertified::Magnitude))
            return refuse(TEXT("First axial arrival route does not cover lateral motion"));
    C::FZeroOffsetKinematicCenterReference model;
    if(!C::BuildZeroOffsetKinematicCenterReference(values.State.Location,values.State.Velocity,values.State.Acceleration,
        horizon,FVector3d::ZeroVector,FVector3d::ZeroVector,model))return refuse(TEXT("Actual supplied trajectory/horizon unsupported"));
    // Complete union, no post-contact leaf deletion and no material/Source
    // claim: all geometry participates in this mathematical computation.
    auto reference=C::EvaluateReferenceSphereValues(geometry->GetSelectedUnion(),values.State.Location,values.State.Velocity,
        values.State.Acceleration,horizon,values.SupportRadius,double(values.PhysicalRadius),options,budget,[](int32){return true;});
    if(partial)*partial=reference; // Pure partial diagnostics never become a retained arrival object.
    if((reference.Status!=C::EReferenceSphereStatus::ReferenceNoContact&&reference.Status!=C::EReferenceSphereStatus::ReferenceContactBounded)||
        !reference.ConsumedPrefixClear||!reference.Candidates||reference.Candidates->GetScene()!=geometry->GetSelectedUnion())
        return refuse(*FString::Printf(TEXT("FirstArrivalRs reference/strict consumed prefix unavailable; stage=%u contact_status=%u budget_remaining=%d; Unknown retained"),uint32(reference.Stage),uint32(reference.Contact.Status),budget.Remaining));
    C::FReferenceCandidateSet::FClosedPlaneOriginWitness continuousWitness;
    if(!reference.Candidates->GetCandidateIndices().IsEmpty()&&!reference.Candidates->ProveCoordinatePlaneOrigin(axis,budget,continuousWitness))
        return refuse(TEXT("FirstArrivalRs continuous candidates are not one exact geometric plane"));
    if(reference.Status==C::EReferenceSphereStatus::ReferenceContactBounded&&
        (reference.Contact.CollisionRadiusBits!=AnalyticCertified::Bits(values.SupportRadius)||reference.Contact.Leaves.IsEmpty()))
        return refuse(TEXT("FirstArrivalRs enclosure radius/leaves differ"));
    const auto cover=PreparedExecution::SelectSelectedWholeConsumedSphereHull(geometry,model,reference.ConsumedSeconds,
        double(values.QueryRadius),budget);
    C::FReferenceCandidateSet::FClosedPlaneOriginWitness roundedWitness;
    if(!cover.UsesGeometry(geometry)||!cover.Partition->ProveCoordinatePlaneOrigin(axis,budget,roundedWitness))
        return refuse(TEXT("Whole rounded Rc hull has an unexcluded nonplane provider"));
    // This is a mathematical evaluation of the SAME native DAG, not an
    // applied state. Reclassify its actual rounded center against Rs; never
    // describe the conservative consumed time as exact physical tangency.
    uint32 consumedBits=0;FMemory::Memcpy(&consumedBits,&reference.ConsumedSeconds,4);
    if((consumedBits&0x80000000u)||consumedBits>=0x7f800000u||(consumedBits&&consumedBits<0x00800000u))
        return refuse(TEXT("Consumed native seconds are not supported positive normal or exact +0"));
    auto endpoint=values;
    if(consumedBits)endpoint.State=values.State.Integrate(reference.ConsumedSeconds); // native +0 is an exact no-op
    FReferenceSphereInitialTubeStatePtr consumed;
    if(!FReferenceSphereInitialTubeState::TryObserve(geometry,endpoint,budget,consumed,reason)||!consumed||
        consumed->GetKind()!=EReferenceSphereRsState::SeparatedRs)
        return refuse(TEXT("Actual consumed point is not freshly proved separated inside Rc"));
    TSharedPtr<FReferenceSphereFirstArrivalRs,ESPMode::ThreadSafe> result(new FReferenceSphereFirstArrivalRs);
    result->Initial=MoveTemp(initial);result->ConsumedPoint=MoveTemp(consumed);result->Reference=MoveTemp(reference);
    result->RcPartition=cover.Partition;result->Horizon=horizon;
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);out=result;return true;
}
}
