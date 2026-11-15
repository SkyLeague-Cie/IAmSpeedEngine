#pragma once
#include "NativeFixedBoxAtFloatTimeValues.h"
namespace Speed::PreparedExecution
{
class FNativeFixedBoxFloatRangeGeometryValues;
using FNativeFixedBoxFloatRangeGeometryValuesPtr=TSharedPtr<const FNativeFixedBoxFloatRangeGeometryValues,ESPMode::ThreadSafe>;
// GEOMETRIC clearance over EVERY admitted original float second in [a,b]
// (+0/positive normal) under the retained numeric controls. Same fixed PVA
// origin, all physical corners, and raw-corner/cached-axis physical + rounded
// shape volumes, with declared query shells. Not the original FULL World-call
// chord, a SAT/release predicate miss, first contact, advance or response grant.
class FNativeFixedBoxFloatRangeGeometryValues final
{
    FPreparedGeometrySelectionPtr Geometry;FNativeFixedBoxReferenceMotionValuesPtr Motion;
    float First=0,Last=0;Analytic::FAnalyticNumericEvidence Numeric;
    Analytic::Certified::FIntervalPoint BodyOrigin,CornerHull,PhysicalFootprint,QueryFootprint,Hull;
    Analytic::Certified::FReferenceBoundsStrictSeparationPtr Separation;
    FNativeFixedBoxFloatRangeGeometryValues()=default;
public:
    static bool TryProve(FPreparedGeometrySelectionPtr geometry,FNativeFixedBoxReferenceMotionValuesPtr motion,
        float first,float last,Analytic::Certified::FDistanceBudget& budget,FNativeFixedBoxFloatRangeGeometryValuesPtr& out)
    {
        const auto keepPrior=out;out.Reset();
        const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        uint32 firstOrdinal=0,lastOrdinal=0;
        if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||!motion||
            !Analytic::Certified::FloorNativeSweepDetail::Ordinal(first,firstOrdinal)||
            !Analytic::Certified::FloorNativeSweepDetail::Ordinal(last,lastOrdinal)||
            firstOrdinal>lastOrdinal||last>motion->GetOriginalHorizon()||!budget.Consume())return false;
        const auto& v=motion->GetCapturedForInspection();const auto& q=v.Query;
        if(q.Shape!=Analytic::EQueryShape::Box||q.Start.ContainsNaN()||q.End.ContainsNaN()||
            !Analytic::Certified::FloorNativeSweepDetail::SameVector(v.Start,q.Start)||
            !Analytic::Certified::FloorNativeSweepDetail::SameVector(v.End,q.End)||
            !Analytic::Certified::FloorNativeSweepDetail::SameVector(q.HalfExtent,v.QueryHalfExtent)||
            !Analytic::Certified::FloorNativeSweepDetail::SameQuat(q.Rotation,v.BodyState.Rotation)||
            !Analytic::Certified::FloorNativeSweepDetail::SameQuat(v.OwnerState.Rotation*v.LocalRotation,v.BodyState.Rotation))return false;
        TSharedPtr<FNativeFixedBoxFloatRangeGeometryValues,ESPMode::ThreadSafe> value(new FNativeFixedBoxFloatRangeGeometryValues);
        value->Geometry=MoveTemp(geometry);value->Motion=MoveTemp(motion);value->First=first;value->Last=last;
        // SAME Fixed body origin already includes owner-COM subtraction and
        // local-offset addition, with float(s*s). Never integrate BodyState.
        value->BodyOrigin=value->Motion->BoundConsumedBodyOriginBetweenFloatSeconds(first,last);
        if(!Analytic::Certified::Detail::ValidPoint(value->BodyOrigin))return false;
        for(int32 k=0;k<8;++k)
        {
            if(!budget.Consume())return false;
            const auto point=value->Motion->BoundConsumedPhysicalCornerBetweenFloatSeconds(k,first,last);
            if(!Analytic::Certified::Detail::ValidPoint(point))return false;
            value->CornerHull=k?Analytic::Certified::TimeDetail::Hull(value->CornerHull,point):point;
        }
        // Derive and RETAIN the actual footprint enclosures in this numeric
        // context. Zero-centered COPIES describe shape geometry only; the
        // original captured World-call Start/End are neither changed nor clipped.
        auto shape=q;shape.Start=shape.End=FVector3d::ZeroVector;
        if(!budget.Consume()||!Analytic::Certified::BuildFixedWorldQueryVolumeBounds(shape,value->QueryFootprint))return false;
        shape.HalfExtent=v.PhysicalHalfExtent;
        if(!budget.Consume()||!Analytic::Certified::BuildFixedWorldQueryVolumeBounds(shape,value->PhysicalFootprint))return false;
        value->Hull=value->CornerHull;
        for(const auto& footprint:{value->PhysicalFootprint,value->QueryFootprint})
        {
            Analytic::Certified::FIntervalPoint moving;
            for(int32 d=0;d<3;++d)moving.Axis[d]=AnalyticCertified::Add(value->BodyOrigin.Axis[d],footprint.Axis[d]);
            if(!Analytic::Certified::Detail::ValidPoint(moving))return false;
            value->Hull=Analytic::Certified::TimeDetail::Hull(value->Hull,moving);
        }
        // Existing input shells enlarge this geometry only. They are NOT a
        // curved Box approximation allowance or a complete native SAT proof.
        for(double shell:{q.DomainTolerance,q.InitialOverlapTolerance})
        {
            const uint64 bits=AnalyticCertified::Bits(shell);
            if(!budget.Consume()||(bits&AnalyticCertified::Magnitude)>=AnalyticCertified::Infinity||
                ((bits&AnalyticCertified::Sign)&&(bits&AnalyticCertified::Magnitude)))return false;
            for(int32 d=0;d<3;++d)value->Hull.Axis[d]=AnalyticCertified::Add(value->Hull.Axis[d],
                AnalyticCertified::Interval{-shell,shell,true});
        }
        if(!Analytic::Certified::Detail::ValidPoint(value->Hull))return false;
        Analytic::Certified::FReferenceBoundsCandidateSetPtr partition;
        if(!Analytic::Certified::FReferenceBoundsCandidateSet::Select(value->Geometry->GetSelectedUnion(),value->Hull,budget,partition)||
            !Analytic::Certified::FReferenceBoundsStrictSeparation::TryProve(MoveTemp(partition),budget,value->Separation)||
            !value->Separation||!value->Separation->Matches(value->Geometry->GetSelectedUnion(),value->Hull))return false;
        const auto finalNumeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(finalNumeric.ControlFingerprint()!=numeric.ControlFingerprint())return false;
        value->Numeric=finalNumeric;out=MoveTemp(value);return true;
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
    float GetFirstFloatSeconds() const{return First;}
    float GetLastFloatSeconds() const{return Last;}
    const auto& GetBodyOriginRangeForInspection() const{return BodyOrigin;}
    const auto& GetPhysicalFootprintForInspection() const{return PhysicalFootprint;}
    const auto& GetQueryFootprintForInspection() const{return QueryFootprint;}
    const auto& GetHullForInspection() const{return Hull;}
    const auto& GetSeparationForInspection() const{return Separation;}
};
}
