#include "PreparedSphereRsTemporalPrefix.h"
#include "PreparedSweepEvidence.h"
namespace Speed::Analytic::Prepared
{
bool FReferenceSphereRsTemporalPrefix::TryEvaluate(FReferenceSphereInitialTubeStatePtr initial,float horizon,
    const C::FReferenceSphereOptions& options,C::FDistanceBudget& budget,
    FReferenceSphereRsTemporalPrefixPtr& out,FRsTemporalPrefixDiagnostic& diagnostic,FString& reason)
{
    out.Reset();diagnostic={};reason.Reset();diagnostic.BudgetBefore=budget.Remaining;
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    const auto fail=[&](const TCHAR* why)
    {
        diagnostic.BudgetAtExit=budget.Remaining;
        reason=FString::Printf(TEXT("Rs temporal prefix Unknown: stage=%u time_status=%u candidates=%d prefix_attempts=%d retreats=%d budget_remaining=%d; %s"),
            uint32(diagnostic.Stage),uint32(diagnostic.TimeStatus),diagnostic.Candidates,diagnostic.PrefixAttempts,diagnostic.Retreats,budget.Remaining,why);
        return false;
    };
    if(!initial||initial->GetKind()!=EReferenceSphereRsState::SeparatedRs||!C::ReferenceSphereDetail::OptionsValid(options)||!budget.Consume())
        return fail(TEXT("A fresh separated point and supported objectives are required"));
    const auto closest=initial->GetClosestAtHeight();const auto geometry=closest?closest->GetGeometry():nullptr;
    const auto& values=initial->GetInput();int32 axis=INDEX_NONE;double sign=0;
    if(!geometry||!geometry->HasValidAliases()||!OriginPlaneDetail::SignedAxis(closest->GetGeometricNormal(),axis,sign)||
        AnalyticCertified::Bits(closest->GetCoverageRadius())!=AnalyticCertified::Bits(double(values.QueryRadius)))
        return fail(TEXT("The exact retained axial plane/Rc identity differs"));
    for(int32 d=0;d<3;++d)
        if(d!=axis&&((AnalyticCertified::Bits(values.State.Velocity[d])|AnalyticCertified::Bits(values.State.Acceleration[d]))&AnalyticCertified::Magnitude))
            return fail(TEXT("This proof does not cover lateral motion"));
    C::FZeroOffsetKinematicCenterReference model;
    if(!C::BuildZeroOffsetKinematicCenterReference(values.State.Location,values.State.Velocity,values.State.Acceleration,
        horizon,FVector3d::ZeroVector,FVector3d::ZeroVector,model))return fail(TEXT("Captured trajectory/horizon unsupported"));
    diagnostic.Stage=ERsTemporalPrefixStage::Candidates;C::FReferenceCandidateSetPtr candidates;
    if(!C::FReferenceCandidateSet::Select(geometry->GetSelectedUnion(),model.GetReferenceCurve(),values.SupportRadius,budget,candidates)||!candidates)
        return fail(TEXT("Complete Rs candidate partition unavailable"));
    diagnostic.Candidates=candidates->GetCandidateIndices().Num();diagnostic.Excluded=candidates->GetExcludedIndices().Num();
    diagnostic.Stage=ERsTemporalPrefixStage::Sweep;
    const auto temporal=candidates->Sweep(budget,options.TimeGap,options.SpatialGap,options.SpatialNodesPerAttempt);
    diagnostic.TimeStatus=temporal.Status;diagnostic.LowerTime=temporal.LowerTime;diagnostic.UpperTime=temporal.UpperTime;
    diagnostic.InitialSeparationProved=temporal.InitialSeparationProved;diagnostic.BudgetAfterSweep=budget.Remaining;
    if((temporal.Status!=C::ETensorSphereTimeStatus::NoIntersectionWithProvidedTensors&&temporal.Status!=C::ETensorSphereTimeStatus::FirstContactBracket)||
        !temporal.InitialSeparationProved)return fail(TEXT("Temporal enclosure unavailable; no contact object was requested or fabricated"));
    diagnostic.Stage=ERsTemporalPrefixStage::Coplanarity;C::FReferenceCandidateSet::FClosedPlaneOriginWitness witness;
    if(!candidates->GetCandidateIndices().IsEmpty()&&!candidates->ProveCoordinatePlaneOrigin(axis,budget,witness))
        return fail(TEXT("A nonplane candidate remains unexcluded"));
    diagnostic.Stage=ERsTemporalPrefixStage::Floor;float consumed=horizon;
    if(temporal.Status==C::ETensorSphereTimeStatus::FirstContactBracket)
    {
        const auto floor=C::FloorReferenceTimeToFloatSeconds(model,temporal.LowerTime);
        if(!floor.Known)return fail(TEXT("A consumed float floor is unavailable"));
        consumed=floor.Seconds;
    }
    diagnostic.InitialFloor=consumed;diagnostic.Stage=ERsTemporalPrefixStage::Prefix;
    for(;;)
    {
        uint32 bits=0;FMemory::Memcpy(&bits,&consumed,4);
        if((bits&0x80000000u)||bits>=0x7f800000u||(bits&&bits<0x00800000u)||consumed>horizon||!budget.Consume())
            return fail(TEXT("Consumed float or shared work allowance unavailable"));
        ++diagnostic.PrefixAttempts;diagnostic.TriedSeconds.Add(consumed);
        const bool clear=C::ReferenceSphereDetail::PrefixClear(geometry->GetSelectedUnion(),model,consumed,values.SupportRadius,options,budget);
        diagnostic.BudgetAfterPrefix=budget.Remaining;
        if(clear)
        {diagnostic.PrefixClear=true;break;}
        if(!bits||budget.Remaining<=budget.ConsumptionFloor)return fail(TEXT("No complete rounded prefix was proved"));
        // Earlier seconds are candidates only. The COMPLETE prefix is checked
        // again; neither endpoint clearance nor the continuous floor suffices.
        // Decide the normal->subnormal transition from the original bits,
        // before arithmetic; FTZ/DAZ cannot choose the next candidate.
        consumed=bits<0x01000000u?0.f:consumed*.5f;++diagnostic.Retreats;
    }
    diagnostic.BudgetAfterPrefix=budget.Remaining;diagnostic.Stage=ERsTemporalPrefixStage::RcCoverage;
    const auto cover=PreparedExecution::SelectSelectedWholeConsumedSphereHull(geometry,model,consumed,double(values.QueryRadius),budget);
    C::FReferenceCandidateSet::FClosedPlaneOriginWitness roundedWitness;
    if(!cover.UsesGeometry(geometry)||!cover.Partition->ProveCoordinatePlaneOrigin(axis,budget,roundedWitness))
        return fail(TEXT("Complete actual Rc hull retains an unexcluded nonplane provider"));
    diagnostic.RcCovered=true;diagnostic.BudgetAfterRc=budget.Remaining;diagnostic.Stage=ERsTemporalPrefixStage::ConsumedPoint;
    auto endpoint=values;if(consumed!=0.f)endpoint.State=values.State.Integrate(consumed);
    if(!C::ReferenceSphereDetail::Contains(C::BoundConsumedCenterAtFloatSeconds(model,consumed),endpoint.State.Location))
        return fail(TEXT("The actual consumed center escaped its rounded DAG enclosure"));
    FReferenceSphereInitialTubeStatePtr point;
    const bool observed=FReferenceSphereInitialTubeState::TryObserve(geometry,endpoint,budget,point,reason);
    if(!observed||!point||point->GetKind()!=EReferenceSphereRsState::SeparatedRs)
    {
        const FString why=FString::Printf(TEXT("The actual rounded consumed center is not freshly proved separated; point_observed=%d; %s"),observed,*reason);
        return fail(*why);
    }
    TSharedPtr<FReferenceSphereRsTemporalPrefix,ESPMode::ThreadSafe> result(new FReferenceSphereRsTemporalPrefix);
    result->Initial=MoveTemp(initial);result->ConsumedPoint=MoveTemp(point);result->Temporal=temporal;
    result->RsCandidates=MoveTemp(candidates);result->RcPartition=cover.Partition;result->Horizon=horizon;result->ConsumedSeconds=consumed;
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    diagnostic.Stage=ERsTemporalPrefixStage::Complete;diagnostic.BudgetAtExit=budget.Remaining;out=MoveTemp(result);return true;
}
}
