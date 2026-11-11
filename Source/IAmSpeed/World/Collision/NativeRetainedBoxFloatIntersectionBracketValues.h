#pragma once
#include "NativeFixedBoxPhysicalFloatRangeClearanceValues.h"
#include "NativeRetainedBoxAtFloatInteriorValues.h"
namespace Speed::PreparedExecution
{
class FNativeRetainedBoxFloatIntersectionBracketValues;
using FNativeRetainedBoxFloatIntersectionBracketValuesPtr=TSharedPtr<const FNativeRetainedBoxFloatIntersectionBracketValues,ESPMode::ThreadSafe>;
// For EACH fixed polynomial realization in the retained CP boxes, bound its
// first admitted FLOAT at which the retained union intersects the convex hull
// of the eight PHYSICAL native-DAG corners. Domain: +0 and positive normal
// floats <= originalH, under the SAME numeric controls. First may vary between
// realizations. No continuity/monotonicity or equality to reference-root time.
// Not SAT/OBB/legacy World-call equivalence, continuous CCD/TOI, point/unitnormal,
// event/response/effect/Frame/Ready. A valid non-singleton bracket is NOT a time
// that a physical consumer may select without its remaining contract.
class FNativeRetainedBoxFloatIntersectionBracketValues final
{
    FNativeFixedBoxPhysicalFloatRangeClearanceValuesPtr Prefix;
    FNativeRetainedBoxAtFloatInteriorValuesPtr Inside;
    uint32 FirstPossible=0,InsideOrdinal=0;
    FNativeRetainedBoxFloatIntersectionBracketValues()=default;
public:
    static bool TryJoin(FNativeFixedBoxPhysicalFloatRangeClearanceValuesPtr prefix,FNativeRetainedBoxAtFloatInteriorValuesPtr inside,
        Analytic::Certified::FDistanceBudget& budget,FNativeRetainedBoxFloatIntersectionBracketValuesPtr& out)
    {
        const auto keepPrior=out;out.Reset();namespace C=Analytic::Certified;
        const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!prefix||!inside||!budget.Consume()||C::KinematicCenterDetail::Bits(prefix->GetFirstFloatSeconds())!=0)return false;
        const auto geometry=prefix->GetGeometryForInspection();const auto motion=prefix->GetMotionForInspection();
        const auto actual=inside->GetActualAtFloatForInspection();if(!geometry||!motion||!actual||actual->GetMotionForInspection()!=motion)return false;
        const auto& captured=motion->GetCapturedForInspection();const float c=prefix->GetLastFloatSeconds(),u=actual->GetFloatSeconds();
        uint32 co=0,uo=0;
        if(!C::FloorNativeSweepDetail::Ordinal(c,co)||!C::FloorNativeSweepDetail::Ordinal(u,uo)||co>=uo||u>motion->GetOriginalHorizon()||
            !prefix->MatchesInputs(geometry,motion,captured,0.f,c)||
            !inside->MatchesInputs(inside->GetFirstReferenceForInspection(),actual,geometry,captured,u)||
            prefix->GetNumericForInspection().ControlFingerprint()!=actual->GetNumericForInspection().ControlFingerprint()||
            numeric.ControlFingerprint()!=actual->GetNumericForInspection().ControlFingerprint())return false;
        TSharedPtr<FNativeRetainedBoxFloatIntersectionBracketValues,ESPMode::ThreadSafe> value(new FNativeRetainedBoxFloatIntersectionBracketValues);
        value->Prefix=MoveTemp(prefix);value->Inside=MoveTemp(inside);value->FirstPossible=co+1;value->InsideOrdinal=uo;
        // SAME fixed model already bounds finite consumed physical corners on
        // all[0,H]. For each fixed coefficient realization the finite set of
        // intersecting admitted floats is nonempty (U), has none <=C, and thus
        // has a least element in(C,U]. No interpolation or intermediate-value
        // argument, and no equality of first times across different CP choices.
        const auto finalNumeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(finalNumeric.ControlFingerprint()!=numeric.ControlFingerprint())return false;
        out=MoveTemp(value);return true;
    }
    bool MatchesInputs(const FPreparedGeometrySelectionPtr& geometry,const FNativeFixedBoxReferenceMotionValuesPtr& motion,
        const FNativeSolidStaticValues& captured) const
    {
        if(!Prefix||!Inside||!motion||Prefix->GetMotionForInspection()!=motion)return false;
        const auto actual=Inside->GetActualAtFloatForInspection();if(!actual||actual->GetMotionForInspection()!=motion)return false;
        uint32 c=0,u=0;
        return CodedEndpoints(c,u)&&FirstPossible==c+1&&InsideOrdinal==u&&
            Prefix->MatchesInputs(geometry,motion,captured,0.f,GetClearPrefixSeconds())&&
            Inside->MatchesInputs(Inside->GetFirstReferenceForInspection(),actual,geometry,captured,GetInsideSeconds())&&
            Prefix->GetNumericForInspection().ControlFingerprint()==actual->GetNumericForInspection().ControlFingerprint();
    }
    // true means VALID bracket only. It may be unchanged on Unknown/quota or
    // total-work exhaustion. Every actual update needs a complete new TryJoin.
    static bool TryRefine(FNativeRetainedBoxFloatIntersectionBracketValuesPtr initial,int32 maximumWorkPerProofAttempt,
        Analytic::Certified::FDistanceBudget& budget,FNativeRetainedBoxFloatIntersectionBracketValuesPtr& out)
    {
        const auto keepPrior=out;out.Reset();namespace C=Analytic::Certified;
        Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!initial||maximumWorkPerProofAttempt<=0||!budget.Consume())return false;
        auto current=MoveTemp(initial);const auto geometry=current->Prefix->GetGeometryForInspection();
        const auto motion=current->Prefix->GetMotionForInspection();if(!motion)return false;
        const auto& captured=motion->GetCapturedForInspection();if(!current->MatchesInputs(geometry,motion,captured))return false;
        while(!current->IsSingletonAdmittedTime())
        {
            if(!budget.Consume())break;uint32 c=0,u=0;if(!current->CodedEndpoints(c,u)||u-c<=1)return false;
            const uint32 midpoint=c+(u-c)/2; // Strict interior compressed ordinal.
            if(midpoint<=c||midpoint>=u)return false;
            const float seconds=C::FloorNativeSweepDetail::Seconds(midpoint);
            FNativeFixedBoxPhysicalFloatRangeClearanceValuesPtr candidatePrefix;
            {
                const int32 available=budget.Remaining-budget.ConsumptionFloor;if(available<=0)break;
                const int32 allowed=FMath::Min(available,maximumWorkPerProofAttempt);
                C::FScopedDistanceBudgetFloor reservation(budget,budget.Remaining-allowed);if(!reservation.IsValid())return false;
                FNativeFixedBoxPhysicalFloatRangeClearanceValues::TryProve(geometry,motion,0.f,seconds,budget,candidatePrefix);
            }
            FNativeRetainedBoxFloatIntersectionBracketValuesPtr candidate;
            if(candidatePrefix)
            {
                if(!TryJoin(candidatePrefix,current->Inside,budget,candidate))break;
                current=MoveTemp(candidate);continue;
            }
            // Failure of a prefix proof is NOT inside evidence. Evaluate the
            // SAME native DAG at the original midpoint seconds independently.
            FNativeRetainedBoxAtFloatInteriorValuesPtr candidateInside;
            {
                const int32 available=budget.Remaining-budget.ConsumptionFloor;if(available<=0)break;
                const int32 allowed=FMath::Min(available,maximumWorkPerProofAttempt);
                C::FScopedDistanceBudgetFloor reservation(budget,budget.Remaining-allowed);if(!reservation.IsValid())return false;
                FNativeFixedBoxAtFloatTimeValuesPtr actual;
                if(FNativeFixedBoxAtFloatTimeValues::TryBuild(motion,seconds,budget,actual)&&actual)
                    FNativeRetainedBoxAtFloatInteriorValues::TryFind(current->Inside->GetFirstReferenceForInspection(),actual,budget,candidateInside);
            }
            if(!candidateInside||!TryJoin(current->Prefix,candidateInside,budget,candidate))break;
            current=MoveTemp(candidate);
        }
        if(!current->MatchesInputs(geometry,motion,captured))return false;
        out=MoveTemp(current);return true;
    }
    float GetClearPrefixSeconds() const{return Prefix->GetLastFloatSeconds();}
    float GetInsideSeconds() const{return Inside->GetActualAtFloatForInspection()->GetFloatSeconds();}
    float GetFirstPossibleSeconds() const{return Analytic::Certified::FloorNativeSweepDetail::Seconds(FirstPossible);}
    bool IsSingletonAdmittedTime() const{return FirstPossible==InsideOrdinal;}
    const auto& GetPrefixForInspection() const{return Prefix;}
    const auto& GetInsideForInspection() const{return Inside;}
private:
    bool CodedEndpoints(uint32& c,uint32& u) const
    {
        return Prefix&&Inside&&Analytic::Certified::KinematicCenterDetail::Bits(Prefix->GetFirstFloatSeconds())==0&&
            Analytic::Certified::FloorNativeSweepDetail::Ordinal(GetClearPrefixSeconds(),c)&&
            Analytic::Certified::FloorNativeSweepDetail::Ordinal(GetInsideSeconds(),u)&&c<u;
    }
};
}
