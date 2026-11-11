#include "PreparedSphereGeneralRsTemporalPrefix.h"
#include "PreparedSweepEvidence.h"
namespace Speed::Analytic::Prepared
{
bool FReferenceSphereGeneralRsTemporalPrefix::TryEvaluate(FReferenceSphereStrictSeparationPtr initial,float horizon,
    const C::FReferenceSphereOptions& options,C::FDistanceBudget& budget,
    FReferenceSphereGeneralRsTemporalPrefixPtr& out,FRsTemporalPrefixDiagnostic& diagnostic,FString& reason)
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
    if(!initial||C::KinematicCenterDetail::Bits(initial->GetSeconds())!=0||!C::ReferenceSphereDetail::OptionsValid(options)||!budget.Consume())
        return fail(TEXT("A fresh separated point and supported objectives are required"));
    const auto geometry=initial->GetGeometry();const auto& values=initial->GetCapturedInput();
    if(!geometry||!geometry->HasValidAliases()||C::KinematicCenterDetail::Bits(initial->GetHorizon())!=C::KinematicCenterDetail::Bits(horizon))
        return fail(TEXT("Captured general geometry/horizon differs"));
    const auto& model=initial->GetModel();
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
    if(!cover.UsesGeometry(geometry))return fail(TEXT("Complete actual Rc rounded-hull partition unavailable"));
    diagnostic.RcCovered=true;diagnostic.BudgetAfterRc=budget.Remaining;diagnostic.Stage=ERsTemporalPrefixStage::ConsumedPoint;
    FReferenceSphereStrictSeparationPtr point;
    if(!FReferenceSphereStrictSeparation::TryProveAt(geometry,values,initial->GetLocalOffset(),initial->GetLocalCOM(),horizon,consumed,options,budget,point,reason)||!point)
    {
        const FString why=FString::Printf(TEXT("Fresh general rounded endpoint separation unavailable; %s"),*reason);
        return fail(*why);
    }
    TSharedPtr<FReferenceSphereGeneralRsTemporalPrefix,ESPMode::ThreadSafe> result(new FReferenceSphereGeneralRsTemporalPrefix);
    result->Initial=MoveTemp(initial);result->ConsumedPoint=MoveTemp(point);result->Temporal=temporal;
    result->RsCandidates=MoveTemp(candidates);result->RcPartition=cover.Partition;result->Horizon=horizon;result->ConsumedSeconds=consumed;
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    diagnostic.Stage=ERsTemporalPrefixStage::Complete;diagnostic.BudgetAtExit=budget.Remaining;out=MoveTemp(result);return true;
}
}
