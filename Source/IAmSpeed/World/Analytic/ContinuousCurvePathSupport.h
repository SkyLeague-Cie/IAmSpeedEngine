#pragma once
#include "ContinuousCenterPath.h"
#include "ContactCellGeometryMemo.h"

namespace Speed::Analytic::Certified
{
enum class ECurvePathExclusion:uint8 {Undecided,Clear,Unknown};
// Geometry-only reusable roots. All projections/visitation are query-local.
// This helper proves separation only; it cannot supply a hit or an owner.
class FContinuousCurvePathSupport final
{
    using Memo=ContactDetail::FContactCellGeometryMemo;
    using CellPtr=Memo::FCellPtr;
    using APoint=FIntervalPoint;
    struct FRoot
    {
        Detail::FNode P,Du,Dv;
        CellPtr Cached;
        bool Ready=false;int32 ZeroAxis=INDEX_NONE;
    };
    struct FWorkCell
    {
        Detail::FNode P,Du,Dv;
        CellPtr Cached;
    };
    const TConstArrayView<FIntervalTensorSurface> Surfaces;
    const uint64 NumericControl;
    Memo::FCallScope Scope;
    TArray<FRoot> Roots;
    // Optional workload bound, not a distance/tangency/gameplay tolerance.
    // Reaching it gives no proof and leaves the existing exact solver active.
    static constexpr int32 MaximumOptionalCells=64;

    static bool SameRoot(const Detail::FNode& root,const FIntervalTensorSurface& surface)
    {
        namespace A=::AnalyticCertified;
        if(root.DegreeU!=surface.DegreeU||root.DegreeV!=surface.DegreeV||root.Net.Num()!=surface.ControlPoints.Num())return false;
        for(int32 i=0;i<root.Net.Num();++i)for(int32 d=0;d<3;++d)
        {
            const auto& a=root.Net[i].Axis[d];const auto& b=surface.ControlPoints[i].Axis[d];
            if(a.Known!=b.Known||A::Bits(a.Lower)!=A::Bits(b.Lower)||A::Bits(a.Upper)!=A::Bits(b.Upper))return false;
        }
        return true;
    }
    static int32 FindExactZeroAxis(const FIntervalTensorSurface& surface)
    {
        namespace A=::AnalyticCertified;
        for(int32 d=0;d<3;++d)
        {
            bool exact=true;
            for(const auto& p:surface.ControlPoints)
                exact&=(A::Bits(p.Axis[d].Lower)&A::Magnitude)==0&&(A::Bits(p.Axis[d].Upper)&A::Magnitude)==0;
            if(exact)return d;
        }
        return INDEX_NONE;
    }
    static ECurvePathExclusion Separates(const ContactDetail::FCurveSupportBand& band,int32 zero,
        const FIntervalBezierCenterPath& path,Interval radiusSquared,FDistanceBudget& budget)
    {
        namespace A=::AnalyticCertified;
        if(zero<0||zero>=3||!A::Valid(band.Beta)||!A::Valid(band.NormSquared)||!(band.NormSquared.Upper>0))return ECurvePathExclusion::Unknown;
        const int32 x=(zero+1)%3,y=(zero+2)%3;
        Interval projected;bool have=false;
        for(const auto& cp:path.ControlPoints)
        {
            if(!budget.Consume())return ECurvePathExclusion::Unknown;
            const auto p=A::Add(A::Multiply(Interval::Exact(band.Direction[x]),cp.Axis[x]),
                A::Multiply(Interval::Exact(band.Direction[y]),cp.Axis[y]));
            if(!A::Valid(p))return ECurvePathExclusion::Unknown;
            if(!have){projected=p;have=true;}
            else {projected.Lower=FMath::Min(projected.Lower,p.Lower);projected.Upper=FMath::Max(projected.Upper,p.Upper);}
        }
        if(!have||!A::Valid(projected))return ECurvePathExclusion::Unknown;
        // Distance of two intervals is NONNEGATIVE. Never square an overlap's
        // signed difference and accidentally turn it into a separating gap.
        Interval gap;
        if(projected.Lower>band.Beta.Upper)gap=A::Subtract(Interval::Exact(projected.Lower),Interval::Exact(band.Beta.Upper));
        else if(band.Beta.Lower>projected.Upper)gap=A::Subtract(Interval::Exact(band.Beta.Lower),Interval::Exact(projected.Upper));
        else return ECurvePathExclusion::Undecided;
        if(!A::Valid(gap))return ECurvePathExclusion::Unknown;
        if(!(gap.Lower>0))return ECurvePathExclusion::Undecided;
        if(!budget.Consume())return ECurvePathExclusion::Unknown;
        const auto lhs=A::Square(gap),rhs=A::Multiply(radiusSquared,band.NormSquared);
        if(!A::Valid(lhs)||!A::Valid(rhs))return ECurvePathExclusion::Unknown;
        return lhs.Lower>rhs.Upper?ECurvePathExclusion::Clear:ECurvePathExclusion::Undecided;
    }
public:
    FContinuousCurvePathSupport(TConstArrayView<FIntervalTensorSurface> surfaces,Memo* memo,uint64 numericControl)
        :Surfaces(surfaces),NumericControl(numericControl),Scope(memo,numericControl)
    {Roots.SetNum(surfaces.Num());}
    FContinuousCurvePathSupport(const FContinuousCurvePathSupport&)=delete;
    FContinuousCurvePathSupport& operator=(const FContinuousCurvePathSupport&)=delete;

    ECurvePathExclusion TryExclude(int32 provider,const FIntervalBezierCenterPath& path,
        Interval radiusSquared,FDistanceBudget& budget,int32 maximumCells)
    {
        namespace A=::AnalyticCertified;
        if(RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=NumericControl||
            !Surfaces.IsValidIndex(provider)||!CenterDetail::Valid(path)||!A::Valid(radiusSquared)||radiusSquared.Lower<0)return ECurvePathExclusion::Unknown;
        const auto& surface=Surfaces[provider];
        if(surface.DegreeU!=0)return ECurvePathExclusion::Undecided;
        if(surface.DegreeV<0||surface.DegreeV>16||surface.ControlPoints.Num()!=surface.DegreeV+1)return ECurvePathExclusion::Unknown;
        for(const auto& p:surface.ControlPoints)if(!Detail::ValidPoint(p))return ECurvePathExclusion::Unknown;
        const int32 zero=FindExactZeroAxis(surface);
        if(zero==INDEX_NONE||maximumCells<=0)return ECurvePathExclusion::Undecided;
        auto& root=Roots[provider];
        if(!root.Ready)
        {
            if(!budget.Consume())return ECurvePathExclusion::Unknown;
            root.P.DegreeU=surface.DegreeU;root.P.DegreeV=surface.DegreeV;
            for(const auto& p:surface.ControlPoints)root.P.Net.Add(p);
            if(!budget.Consume()||!ContactDetail::OriginalDerivativeNet(root.P,true,root.Du)||
                !budget.Consume()||!ContactDetail::OriginalDerivativeNet(root.P,false,root.Dv))return ECurvePathExclusion::Unknown;
            root.Cached=Scope.FindRoot(root.P,root.Du,root.Dv);root.ZeroAxis=zero;root.Ready=true;
        }
        else if(!SameRoot(root.P,surface))return ECurvePathExclusion::Unknown;
        FWorkCell first;first.Cached=root.Cached;
        if(!first.Cached){first.P=root.P;first.Du=root.Du;first.Dv=root.Dv;}
        TArray<FWorkCell> stack;stack.Add(MoveTemp(first));
        const int32 cap=FMath::Min(maximumCells,MaximumOptionalCells);int32 visited=0;
        while(!stack.IsEmpty())
        {
            if(visited>=cap)return ECurvePathExclusion::Undecided;
            ++visited;auto cell=stack.Pop(EAllowShrinking::No);
            const auto& p=cell.Cached?cell.Cached->Get(Memo::ENet::Position):cell.P;
            const auto& du=cell.Cached?cell.Cached->Get(Memo::ENet::Du):cell.Du;
            const auto& dv=cell.Cached?cell.Cached->Get(Memo::ENet::Dv):cell.Dv;
            // Cold/warm geometry performs exactly the same original work debits.
            for(int32 unit=0;unit<1+p.Net.Num();++unit)if(!budget.Consume())return ECurvePathExclusion::Unknown;
            ContactDetail::FCurveSupportBands bands;
            if(!Scope.CurveSupportBands(cell.Cached,p,root.ZeroAxis,bands))return ECurvePathExclusion::Unknown;
            bool clear=false;
            for(const auto& band:bands.Values)
            {
                const auto proof=Separates(band,bands.ZeroAxis,path,radiusSquared,budget);
                if(proof==ECurvePathExclusion::Unknown)return proof;
                if(proof==ECurvePathExclusion::Clear){clear=true;break;}
            }
            if(clear)continue;
            if(p.DegreeV==0||p.DepthV>=52||visited>=cap)return ECurvePathExclusion::Undecided;
            if(!budget.Consume())return ECurvePathExclusion::Unknown;
            FWorkCell left,right;
            if(!Scope.SplitChildren(cell.Cached,false,p,du,dv,left.P,left.Du,left.Dv,
                right.P,right.Du,right.Dv,left.Cached,right.Cached))return ECurvePathExclusion::Unknown;
            // Both complete parameter halves remain. A failed or merely local
            // minimizer exclusion can never remove a part of this cover.
            stack.Add(MoveTemp(right));stack.Add(MoveTemp(left));
        }
        return ECurvePathExclusion::Clear;
    }
};
}
