#pragma once
#include "FloorNativeSolidSweepBounds.h"
namespace Speed::Analytic::Certified
{
// Actual fixed Box corner trajectories against an explicitly chosen exact
// axial plane. No coordinate permutation, radius, tangent plane or Box margin.
class FFixedAxialBoxMotionModel
{
    FFloorNativeSolidModel Fixed;FVector Point=FVector::ZeroVector,Normal=FVector::ZeroVector;
    bool Known=false;
    friend bool BuildFixedAxialBoxMotionModel(const PreparedExecution::FNativeSolidStaticValues&,
        const FVector&,const FVector&,FFixedAxialBoxMotionModel&);
public:
    bool IsKnown() const{return Known;}
    float GetHorizon() const{return Fixed.GetHorizon();}
    FIntervalPoint BoundSupportHull(bool consumed=true) const{return Known?Fixed.BoundSupportHull(consumed):FIntervalPoint{};}
    Interval BoundPhysicalCornerGap(int32 index,float first,float last,bool consumed) const
    {
        if(!Known)return {};const auto corner=Fixed.BoundPhysicalCorner(index,first,last,consumed);
        if(!Detail::ValidPoint(corner))return {};
        // Bound the COMPLETE native Dot(Vertex-Point,N) DAG, including the
        // zero-axis terms. Inf*0 in a tangential subtraction is not a zero.
        Interval terms[3];
        for(int32 d=0;d<3;++d)terms[d]=AnalyticCertified::Multiply(
            AnalyticCertified::Subtract(corner.Axis[d],Interval::Exact(Point[d])),Interval::Exact(Normal[d]));
        return AnalyticCertified::Add(AnalyticCertified::Add(terms[0],terms[1]),terms[2]);
    }
    Interval BoundGap(float first,float last,bool consumed) const
    {
        Interval gap;for(int32 k=0;k<8;++k)
        {const auto g=BoundPhysicalCornerGap(k,first,last,consumed);gap=k?FloorNativeSweepDetail::Minimum(gap,g):g;}
        return gap;
    }
};
inline bool BuildFixedAxialBoxMotionModel(const PreparedExecution::FNativeSolidStaticValues& v,
    const FVector& point,const FVector& normal,FFixedAxialBoxMotionModel& out)
{
    out={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(v.Shape!=PreparedExecution::ENativeSolidQueryShape::Box||point.ContainsNaN()||normal.ContainsNaN())return false;
    int32 nonzero=0;for(int32 d=0;d<3;++d)
    {
        const auto bits=AnalyticCertified::Bits(normal[d]);
        if(bits&AnalyticCertified::Magnitude)
        {if(bits!=AnalyticCertified::Bits(1.)&&bits!=AnalyticCertified::Bits(-1.))return false;++nonzero;}
    }
    if(nonzero!=1)return false;FFixedAxialBoxMotionModel model;
    // Keep the existing fixed-shape checks exactly; in particular this first
    // extension still conservatively refuses a captured body center at Z=0.
    if(!BuildFloorNativeSolidModel(v,model.Fixed)||!model.Fixed.IsFixedBox())return false;
    model.Point=point;model.Normal=normal;model.Known=true;
    if(!AnalyticCertified::Valid(model.BoundGap(0,v.Horizon,true))||
        !AnalyticCertified::Valid(model.BoundGap(0,v.Horizon,false)))return false;
    out=MoveTemp(model);return true;
}
inline FFloorNativeSweepBounds BoundFixedAxialBoxFirstArrival(const FFixedAxialBoxMotionModel& model,FDistanceBudget& budget)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    FFloorNativeSweepBounds out;if(!model.IsKnown()||!budget.Consume())return out;
    const auto nativeHull=model.BoundSupportHull(),referenceHull=model.BoundSupportHull(false);
    if(!Detail::ValidPoint(nativeHull)||!Detail::ValidPoint(referenceHull))return out;
    const auto clear=[](Interval g){return AnalyticCertified::Valid(g)&&g.Lower>0;};
    const auto inside=[](Interval g){return AnalyticCertified::Valid(g)&&g.Upper<0;};
    if(!clear(model.BoundGap(0,0,true))||!clear(model.BoundGap(0,0,false)))return out;
    uint32 end;if(!FloorNativeSweepDetail::Ordinal(model.GetHorizon(),end)||!end)return out;
    struct FCell{uint32 First,Last;};TArray<FCell,TInlineAllocator<64>> stack;stack.Add({0,end});uint32 prefix=0;
    while(stack.Num())
    {
        if(!budget.Consume())return {};const auto cell=stack.Pop(EAllowShrinking::No);
        const float lo=FloorNativeSweepDetail::Seconds(cell.First),hi=FloorNativeSweepDetail::Seconds(cell.Last);
        const auto native=model.BoundGap(lo,hi,true),reference=model.BoundGap(lo,hi,false);
        if(!AnalyticCertified::Valid(native)||!AnalyticCertified::Valid(reference))return {};
        if(clear(native)&&clear(reference)){prefix=cell.Last;continue;}
        if(cell.Last-cell.First<=1)
        {
            // FIRST indeterminate leaf only. Never skip an earlier uncertain
            // tangent or create a later contact using an older clear anchor.
            const auto lowN=model.BoundGap(lo,lo,true),lowR=model.BoundGap(lo,lo,false);
            if(prefix!=cell.First||!cell.First||!clear(lowN)||!clear(lowR)||
                !inside(model.BoundGap(hi,hi,true))||!inside(model.BoundGap(hi,hi,false)))return {};
            out.Status=EFloorNativeSweepStatus::ArrivalBracket;out.ClearPrefixSeconds=lo;out.InsideSeconds=hi;
            out.ConsumedGap=lowN;out.ReferenceGap=lowR;out.SupportHull=TimeDetail::Hull(nativeHull,referenceHull);return out;
        }
        const uint32 middle=cell.First+(cell.Last-cell.First)/2;
        stack.Add({middle,cell.Last});stack.Add({cell.First,middle});
    }
    out.Status=EFloorNativeSweepStatus::Separated;out.ClearPrefixSeconds=model.GetHorizon();
    out.SupportHull=TimeDetail::Hull(nativeHull,referenceHull);return out;
}
}
