#pragma once
#include "NativeFixedBoxReferenceMotionValues.h"
#include "IAmSpeed/World/Analytic/ReferenceBoundsStrictSeparation.h"
namespace Speed::PreparedExecution
{
class FNativeFixedBoxPhysicalFloatRangeClearanceValues;
using FNativeFixedBoxPhysicalFloatRangeClearanceValuesPtr=TSharedPtr<const FNativeFixedBoxPhysicalFloatRangeClearanceValues,ESPMode::ThreadSafe>;
// Strong separate type for universal strict PHYSICAL-corner-convex-hull
// clearance over every admitted original float in[first,last]. No rounded
// query footprint/shell, SAT/World-query equivalence, reference-only clearance,
// normal/error allowance, event/response/effect or Ready is inferred. The old
// NativeFixedBoxFloatRangeGeometryValues retains its broader query scope.
class FNativeFixedBoxPhysicalFloatRangeClearanceValues final
{
    FPreparedGeometrySelectionPtr Geometry;FNativeFixedBoxReferenceMotionValuesPtr Motion;
    float First=0,Last=0;Analytic::FAnalyticNumericEvidence Numeric;
    Analytic::Certified::FIntervalPoint Hull;
    Analytic::Certified::FReferenceBoundsStrictSeparationPtr Separation;
    FNativeFixedBoxPhysicalFloatRangeClearanceValues()=default;
public:
    static bool TryProve(FPreparedGeometrySelectionPtr geometry,FNativeFixedBoxReferenceMotionValuesPtr motion,
        float first,float last,Analytic::Certified::FDistanceBudget& budget,FNativeFixedBoxPhysicalFloatRangeClearanceValuesPtr& out)
    {
        const auto keepPrior=out;out.Reset();namespace C=Analytic::Certified;
        const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        uint32 a=0,b=0;
        if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||!motion||
            !C::FloorNativeSweepDetail::Ordinal(first,a)||!C::FloorNativeSweepDetail::Ordinal(last,b)||a>b||
            last>motion->GetOriginalHorizon()||!budget.Consume())return false;
        C::FIntervalPoint hull;
        for(int32 k=0;k<8;++k)
        {
            if(!budget.Consume())return false;
            // SAME immutable fixed model includes native float(s*s), owner
            // COM subtraction/local offset addition and physical rotated corner.
            const auto point=motion->BoundConsumedPhysicalCornerBetweenFloatSeconds(k,first,last);
            if(!C::Detail::ValidPoint(point))return false;
            hull=k?C::TimeDetail::Hull(hull,point):point;
        }
        if(!C::Detail::ValidPoint(hull))return false;
        C::FReferenceBoundsCandidateSetPtr partition;C::FReferenceBoundsStrictSeparationPtr separation;
        if(!C::FReferenceBoundsCandidateSet::Select(geometry->GetSelectedUnion(),hull,budget,partition)||
            !C::FReferenceBoundsStrictSeparation::TryProve(MoveTemp(partition),budget,separation)||!separation||
            !separation->Matches(geometry->GetSelectedUnion(),hull))return false;
        // At EACH float, the convex hull of all eight actual native physical
        // corners lies in this entire-range hull. All providers are excluded.
        const auto finalNumeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(finalNumeric.ControlFingerprint()!=numeric.ControlFingerprint())return false;
        TSharedPtr<FNativeFixedBoxPhysicalFloatRangeClearanceValues,ESPMode::ThreadSafe> value(new FNativeFixedBoxPhysicalFloatRangeClearanceValues);
        value->Geometry=MoveTemp(geometry);value->Motion=MoveTemp(motion);value->First=first;value->Last=last;
        value->Numeric=finalNumeric;value->Hull=hull;value->Separation=MoveTemp(separation);out=MoveTemp(value);return true;
    }
    bool MatchesInputs(const FPreparedGeometrySelectionPtr& geometry,const FNativeFixedBoxReferenceMotionValuesPtr& motion,
        const FNativeSolidStaticValues& captured,float first,float last) const
    {
        const auto numeric=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        return numeric.Status==Analytic::EAnalyticNumericStatus::Complete&&numeric.ControlFingerprint()==Numeric.ControlFingerprint()&&
            Geometry&&Geometry==geometry&&Geometry->HasValidAliases()&&Motion&&Motion==motion&&Motion->MatchesCapturedValues(captured)&&
            Analytic::Certified::KinematicCenterDetail::Bits(first)==Analytic::Certified::KinematicCenterDetail::Bits(First)&&
            Analytic::Certified::KinematicCenterDetail::Bits(last)==Analytic::Certified::KinematicCenterDetail::Bits(Last)&&
            Separation&&Separation->Matches(Geometry->GetSelectedUnion(),Hull);
    }
    const auto& GetGeometryForInspection() const{return Geometry;}
    const auto& GetMotionForInspection() const{return Motion;}
    const auto& GetNumericForInspection() const{return Numeric;}
    const auto& GetHullForInspection() const{return Hull;}
    const auto& GetSeparationForInspection() const{return Separation;}
    float GetFirstFloatSeconds() const{return First;}
    float GetLastFloatSeconds() const{return Last;}
};
}
