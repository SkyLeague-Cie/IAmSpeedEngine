#pragma once
#include "TensorSphereContactBounds.h"
namespace Speed::Analytic::Certified
{
// Exact normalized reference-domain code only. A subdivision edge is never
// promoted to a physical boundary or a support-release contour.
struct FReferenceDyadicCell
{
    uint32 Depth=0;uint64 Index=0;
    bool IsValid() const{return Depth<=52&&Index<(uint64(1)<<Depth);}
    bool SameCode(const FReferenceDyadicCell& other) const{return Depth==other.Depth&&Index==other.Index;}
    Interval GetRange() const
    {
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!IsValid())return {};
        // The exponent and integer casts are exact for Depth<=52. Every
        // endpoint and intermediate midpoint has at most53 significant bits.
        const double width=AnalyticCertified::FromBits(uint64(1023-Depth)<<52);
        return {double(Index)*width,double(Index+1)*width,true};
    }
    Interval MapLocal(Interval local) const
    {
        const auto range=GetRange();
        if(!AnalyticCertified::Valid(range)||!AnalyticCertified::Valid(local)||local.Lower<0||local.Upper>1)return {};
        const double width=AnalyticCertified::FromBits(uint64(1023-Depth)<<52);
        auto result=AnalyticCertified::Add(Interval::Exact(range.Lower),
            AnalyticCertified::Multiply(Interval::Exact(width),local));
        if(!AnalyticCertified::Valid(result))return {};
        // Intersection with the already-proved closed parent cell is safe.
        // Do not add epsilon or assert strict GLOBAL binary64 endpoints.
        result.Lower=FMath::Max(result.Lower,range.Lower);result.Upper=FMath::Min(result.Upper,range.Upper);
        return AnalyticCertified::Valid(result)?result:Interval{};
    }
};
inline bool RestrictReferencePathDyadic(const FIntervalBezierCenterPath& input,const FReferenceDyadicCell& cell,
    FDistanceBudget& budget,FIntervalBezierCenterPath& output)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    const bool valid=CenterDetail::Valid(input)&&cell.IsValid();
    FIntervalBezierCenterPath path;if(valid)path=input;output={};
    if(!valid||!budget.Consume())return false;
    for(uint32 level=0;level<cell.Depth;++level)
    {
        if(!budget.Consume())return false;FIntervalBezierCenterPath left,right;
        if(!CenterDetail::Split(path,left,right))return false;
        const bool second=(cell.Index>>(cell.Depth-1-level))&uint64(1);
        path=second?MoveTemp(right):MoveTemp(left);
    }
    if(!CenterDetail::Valid(path))return false;output=MoveTemp(path);return true;
}
inline bool RestrictReferenceTensorDyadic(const FIntervalTensorSurface& input,const FReferenceDyadicCell& u,
    const FReferenceDyadicCell& v,FDistanceBudget& budget,FIntervalTensorSurface& output)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    const bool valid=input.DegreeU>=0&&input.DegreeU<=16&&input.DegreeV>=0&&input.DegreeV<=16&&
        input.ControlPoints.Num()==(input.DegreeU+1)*(input.DegreeV+1)&&u.IsValid()&&v.IsValid();
    Detail::FNode node;
    if(valid){node.DegreeU=input.DegreeU;node.DegreeV=input.DegreeV;node.Net=input.ControlPoints;}
    output={};if(!valid||!budget.Consume())return false;
    for(const auto& cp:node.Net)if(!budget.Consume()||!Detail::ValidPoint(cp))return false;
    for(int32 axis=0;axis<2;++axis)
    {
        const auto& cell=axis?v:u;
        for(uint32 level=0;level<cell.Depth;++level)
        {
            if(!budget.Consume())return false;Detail::FNode left,right;
            if(!Detail::Split(node,axis==0,left,right))return false;
            const bool second=(cell.Index>>(cell.Depth-1-level))&uint64(1);
            node=second?MoveTemp(right):MoveTemp(left);
        }
    }
    const auto ur=u.GetRange(),vr=v.GetRange();
    if(!AnalyticCertified::Valid(ur)||!AnalyticCertified::Valid(vr)||
        AnalyticCertified::Bits(node.MinU)!=AnalyticCertified::Bits(ur.Lower)||
        AnalyticCertified::Bits(node.MaxU)!=AnalyticCertified::Bits(ur.Upper)||
        AnalyticCertified::Bits(node.MinV)!=AnalyticCertified::Bits(vr.Lower)||
        AnalyticCertified::Bits(node.MaxV)!=AnalyticCertified::Bits(vr.Upper))return false;
    FIntervalTensorSurface restricted;restricted.DegreeU=node.DegreeU;restricted.DegreeV=node.DegreeV;
    restricted.ControlPoints=MoveTemp(node.Net);output=MoveTemp(restricted);return true;
}
}
