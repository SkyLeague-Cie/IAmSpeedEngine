#pragma once
#include "IAmSpeed/World/Analytic/ReferenceDyadicCellBounds.h"

// UNINSTALLED PROPOSAL. These are CONDITIONAL mathematical helpers, never an
// authentication or a World authority. A closed producer must first prove that
// the SAME fixed canonical master has the stated exact product factorization.
// Equal independent coefficient boxes do not establish that premise.
namespace Speed::Analytic::Certified::AffineReferenceDetail
{
namespace A=::AnalyticCertified;
enum class EOutcome:uint8 {Unknown,NotApplicable,Complete,NeedsRefinement};

struct FProduct
{
    int32 FreeWorldAxis=INDEX_NONE;
    bool FreeAlongU=true;
    Interval Start,End; // Enclose two FIXED endpoints of the exact affine factor.
    FIntervalTensorSurface Curve; // degreeU=0; free world axis identically zero.
    FIntervalPoint ExactCurveDerivativeHull; // Additional fixed-source conclusion; not derived from box equality.
};
namespace FixedProductWitnessControl
{
IAMSPEED_API bool RuntimeEnabled();
IAMSPEED_API void PublishObservation();
}
// Conditional fixed-master evaluation, never an independent-original-box
// family claim. The closed native transport authenticates FProduct.
IAMSPEED_API bool EvaluateFixedProductUV(const FProduct&,FVector2d originalUV,FIntervalPoint&);

struct FProjection
{
    int32 FreeWorldAxis=INDEX_NONE;
    FIntervalBezierCenterPath Path;
    TArray<FIntervalTensorSurface> Curves;
};

// A common projection is selected only for the COMPLETE candidate union.
// NotApplicable is a pre-search choice; UNKNOWN after a real attempt is terminal.
// Conditional source theorem inputs, authenticated by the enclosing native
// transport. A shared group string is NOT inferred from interval-box equality.
inline EOutcome SelectStrictDominantProductCover(TConstArrayView<FProduct> products,
    TConstArrayView<FString> exactCurveGroups,const FIntervalBezierCenterPath& path,
    FDistanceBudget& budget,TArray<int32>& retained,TArray<int32>& representatives)
{
    retained.Reset();representatives.Reset();
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!CenterDetail::Valid(path)||products.Num()!=exactCurveGroups.Num())return EOutcome::Unknown;
    if(products.IsEmpty())return EOutcome::NotApplicable;
    const auto center=CenterDetail::Hull(path);if(!Detail::ValidPoint(center))return EOutcome::Unknown;
    const int32 axis=products[0].FreeWorldAxis;if(axis<0||axis>=3)return EOutcome::Unknown;
    TArray<int32> kept,map;map.Init(INDEX_NONE,products.Num());
    TArray<bool> outside;outside.Init(false,products.Num());
    for(int32 i=0;i<products.Num();++i)
    {
        if(!budget.Consume())return EOutcome::Unknown;
        const auto& p=products[i];
        if(p.FreeWorldAxis!=axis)return EOutcome::NotApplicable;
        if(!A::Valid(p.Start)||!A::Valid(p.End)||p.Curve.DegreeU!=0||p.Curve.DegreeV<0||p.Curve.DegreeV>16||
            p.Curve.ControlPoints.Num()!=p.Curve.DegreeV+1)return EOutcome::Unknown;
        for(const auto& cp:p.Curve.ControlPoints)
            if(!budget.Consume()||!Detail::ValidPoint(cp)||A::Bits(cp.Axis[axis].Lower)!=0||A::Bits(cp.Axis[axis].Upper)!=0)return EOutcome::Unknown;
        const bool inc=p.End.Lower>p.Start.Upper,dec=p.Start.Lower>p.End.Upper;
        if(!inc&&!dec)return EOutcome::Unknown;
        const auto lo=inc?p.Start:p.End,hi=inc?p.End:p.Start;
        if(center.Axis[axis].Lower>lo.Upper&&center.Axis[axis].Upper<hi.Lower)
        {kept.Add(i);map[i]=i;}
        else if(center.Axis[axis].Lower>hi.Upper||center.Axis[axis].Upper<lo.Lower)outside[i]=true;
        else return EOutcome::NotApplicable; // No strict original chart at a seam/crossing.
    }
    if(kept.IsEmpty())return EOutcome::NotApplicable;
    const auto sameBounds=[&](const FProduct& a,const FProduct& b,bool reverse)
    {
        if(a.Curve.DegreeV!=b.Curve.DegreeV)return EOutcome::NotApplicable;
        for(int32 j=0;j<a.Curve.ControlPoints.Num();++j)
        {
            if(!budget.Consume())return EOutcome::Unknown;
            const auto& x=a.Curve.ControlPoints[j];
            const auto& y=b.Curve.ControlPoints[reverse?b.Curve.DegreeV-j:j];
            for(int32 d=0;d<3;++d)
                if(x.Axis[d].Known!=y.Axis[d].Known||A::Bits(x.Axis[d].Lower)!=A::Bits(y.Axis[d].Lower)||
                    A::Bits(x.Axis[d].Upper)!=A::Bits(y.Axis[d].Upper))return EOutcome::NotApplicable;
        }
        return EOutcome::Complete;
    };
    for(int32 i=0;i<products.Num();++i)if(outside[i])
    {
        bool found=false;
        for(int32 k:kept)
        {
            if(!budget.Consume())return EOutcome::Unknown;
            if(exactCurveGroups[i].IsEmpty()||exactCurveGroups[i]!=exactCurveGroups[k])continue;
            // Necessary enclosure readback only; the authenticated rational
            // group theorem supplies actual coefficient correlation.
            auto match=sameBounds(products[i],products[k],false);
            if(match==EOutcome::Unknown)return match;
            if(match!=EOutcome::Complete)match=sameBounds(products[i],products[k],true);
            if(match==EOutcome::Unknown)return match;
            if(match==EOutcome::Complete){map[i]=k;found=true;break;}
        }
        if(!found)return EOutcome::NotApplicable;
    }
    if(map.Contains(INDEX_NONE))return EOutcome::Unknown;
    retained=MoveTemp(kept);representatives=MoveTemp(map);return EOutcome::Complete;
}

inline EOutcome ProjectCompleteCandidateUnion(TConstArrayView<FProduct> products,
    const FIntervalBezierCenterPath& original,FDistanceBudget& budget,FProjection& out)
{
    out={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!CenterDetail::Valid(original))return EOutcome::Unknown;
    if(products.IsEmpty())return EOutcome::NotApplicable;
    const int32 axis=products[0].FreeWorldAxis;
    if(axis<0||axis>=3)return EOutcome::Unknown;
    const auto center=CenterDetail::Hull(original);
    if(!Detail::ValidPoint(center))return EOutcome::Unknown;
    FProjection result;result.FreeWorldAxis=axis;
    for(const auto& product:products)
    {
        if(!budget.Consume())return EOutcome::Unknown;
        if(product.FreeWorldAxis!=axis)return EOutcome::NotApplicable;
        if(!A::Valid(product.Start)||!A::Valid(product.End))return EOutcome::Unknown;
        const bool increasing=product.End.Lower>product.Start.Upper;
        const bool decreasing=product.Start.Lower>product.End.Upper;
        if(!increasing&&!decreasing)return EOutcome::Unknown;
        const auto lower=increasing?product.Start:product.End;
        const auto upper=increasing?product.End:product.Start;
        if(!(center.Axis[axis].Lower>lower.Upper&&center.Axis[axis].Upper<upper.Lower))
            return EOutcome::NotApplicable;
        const auto& curve=product.Curve;
        if(curve.DegreeU!=0||curve.DegreeV<0||curve.DegreeV>16||curve.ControlPoints.Num()!=curve.DegreeV+1)
            return EOutcome::Unknown;
        for(const auto& point:curve.ControlPoints)
        {
            if(!budget.Consume()||!Detail::ValidPoint(point)||
                A::Bits(point.Axis[axis].Lower)!=0||A::Bits(point.Axis[axis].Upper)!=0)
                return EOutcome::Unknown;
        }
        result.Curves.Add(curve);
    }
    result.Path=original;
    for(auto& point:result.Path.ControlPoints)
    {
        if(!budget.Consume())return EOutcome::Unknown;
        point.Axis[axis]=Interval::Exact(0);
    }
    if(!CenterDetail::Valid(result.Path))return EOutcome::Unknown;
    out=MoveTemp(result);return EOutcome::Complete;
}

inline bool FreeParameter(const FProduct& product,const FIntervalPoint& center,Interval& out)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    out={};if(product.FreeWorldAxis<0||product.FreeWorldAxis>=3||!Detail::ValidPoint(center)||
        !A::Valid(product.Start)||!A::Valid(product.End))return false;
    Interval numerator,denominator;
    if(product.End.Lower>product.Start.Upper)
    {
        numerator=A::Subtract(center.Axis[product.FreeWorldAxis],product.Start);
        denominator=A::Subtract(product.End,product.Start);
    }
    else if(product.Start.Lower>product.End.Upper)
    {
        numerator=A::Subtract(product.Start,center.Axis[product.FreeWorldAxis]);
        denominator=A::Subtract(product.Start,product.End);
    }
    else return false;
    if(!ContactDetail::ProjectionDividePositive(numerator,denominator,out))return false;
    // The pre-search strict-interior proof already establishes the exact
    // quotient is in (0,1). Intersection only removes outward overestimation.
    out.Lower=FMath::Max(0.,out.Lower);out.Upper=FMath::Min(1.,out.Upper);
    return A::Valid(out);
}

// No sampling is used to establish coverage. Binary64 multiplication by2^d
// and the dyadic endpoints are exact for this nonnegative normal range,d<=52.
inline EOutcome CoverFreeParameter(const FProduct& product,Interval parameter,
    double maximumCellWorldWidth,FDistanceBudget& budget,TArray<FReferenceDyadicCell>& out)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    out.Reset();if(!A::Valid(parameter)||parameter.Lower<0||parameter.Upper>1||
        !TimeDetail::NonnegativeFinite(maximumCellWorldWidth)||!(maximumCellWorldWidth>0))return EOutcome::Unknown;
    const auto slope=product.End.Lower>product.Start.Upper?
        A::Subtract(product.End,product.Start):A::Subtract(product.Start,product.End);
    if(!A::Valid(slope)||!(slope.Lower>0))return EOutcome::Unknown;
    constexpr uint64 MaximumCoverCells=16; // Bounded work representation, no gameplay tolerance.
    for(uint32 depth=1;depth<=52;++depth)
    {
        if(!budget.Consume())return EOutcome::Unknown;
        const uint64 count=uint64(1)<<depth;
        const double width=A::FromBits(uint64(1023-depth)<<52);
        const auto worldWidth=A::Multiply(slope,Interval::Exact(width));
        if(!A::Valid(worldWidth))return EOutcome::Unknown;
        if(worldWidth.Upper>maximumCellWorldWidth)continue;
        volatile double scaledLower=parameter.Lower*double(count),scaledUpper=parameter.Upper*double(count);
        if(!A::Finite(scaledLower)||!A::Finite(scaledUpper)||scaledLower<0||scaledUpper>double(count))return EOutcome::Unknown;
        const uint64 first=uint64(scaledLower),last=uint64(scaledUpper);
        if(first==0||last>=count-1)continue; // A finer cell can be interior even for a constant path.
        if(last<first)return EOutcome::Unknown;
        if(last-first>=MaximumCoverCells)return EOutcome::NeedsRefinement;
        TArray<FReferenceDyadicCell> cells;
        for(uint64 index=first;index<=last;++index)
        {
            if(!budget.Consume())return EOutcome::Unknown;
            FReferenceDyadicCell cell;cell.Depth=depth;cell.Index=index;
            const auto range=cell.GetRange();
            if(!A::Valid(range)||!(range.Lower>0&&range.Upper<1))return EOutcome::Unknown;
            cells.Add(cell);
        }
        if(cells.IsEmpty()||cells[0].GetRange().Lower>parameter.Lower||
            cells.Last().GetRange().Upper<parameter.Upper)return EOutcome::Unknown;
        out=MoveTemp(cells);return EOutcome::Complete;
    }
    return EOutcome::Unknown;
}

// The reduced record encloses the PROJECTED polynomial family. The lifted
// record below is valid ONLY for the authenticated FIXED PRODUCT MASTER.
// Do not expose it as an independent-original-CP-family theorem.
inline EOutcome LiftFixedMasterContact(TConstArrayView<FProduct> products,
    TConstArrayView<FIntervalTensorSurface> originalCandidates,
    const FIntervalBezierCenterPath& originalPath,const FTensorSphereContactEnclosure& reduced,
    double maximumFreeCellWidth,double maximumGlobalPointWidth,
    FDistanceBudget& budget,FTensorSphereContactEnclosure& out)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    out={};if(products.Num()!=originalCandidates.Num()||products.IsEmpty()||
        reduced.Status!=ETensorSphereContactStatus::FirstContactEnclosed||reduced.Leaves.IsEmpty())return EOutcome::Unknown;
    const auto center=CenterDetail::Evaluate(originalPath,Interval{reduced.LowerTime,reduced.UpperTime,true});
    if(!Detail::ValidPoint(center))return EOutcome::Unknown;
    const int32 axis=products[0].FreeWorldAxis;
    if(axis<0||axis>=3)return EOutcome::Unknown;
    if(!TimeDetail::NonnegativeFinite(maximumGlobalPointWidth)||!(maximumGlobalPointWidth>0))return EOutcome::Unknown;
    const auto axialWidth=A::Subtract(Interval::Exact(center.Axis[axis].Upper),Interval::Exact(center.Axis[axis].Lower));
    if(!A::Valid(axialWidth))return EOutcome::Unknown;
    if(axialWidth.Upper>maximumGlobalPointWidth)return EOutcome::NeedsRefinement;
    auto result=reduced;result.Leaves.Reset();
    result.SurfacePoint.Axis[axis]=center.Axis[axis];result.RadialUnitNormal.Axis[axis]=Interval::Exact(0);
    for(const auto& projected:reduced.Leaves)
    {
        const int32 index=projected.SurfaceIndex;
        if(index<0||index>=products.Num()||!budget.Consume())return EOutcome::Unknown;
        const auto& product=products[index];
        if(product.FreeWorldAxis!=axis)return EOutcome::Unknown;
        Interval parameter;if(!FreeParameter(product,center,parameter))return EOutcome::Unknown;
        TArray<FReferenceDyadicCell> cells;
        const auto status=CoverFreeParameter(product,parameter,maximumFreeCellWidth,budget,cells);
        if(status!=EOutcome::Complete)return status;
        for(const auto& cell:cells)
        {
            if(!budget.Consume())return EOutcome::Unknown;
            const auto freeRange=cell.GetRange();auto leaf=projected;
            if(product.FreeAlongU)
            {leaf.MinU=freeRange.Lower;leaf.MaxU=freeRange.Upper;}
            else
            {leaf.MinU=projected.MinV;leaf.MaxU=projected.MaxV;leaf.MinV=freeRange.Lower;leaf.MaxV=freeRange.Upper;}
            leaf.PointHull.Axis[axis]=center.Axis[axis];leaf.RadialNormalHull.Axis[axis]=Interval::Exact(0);
            leaf.FixedParameter=FVector2d((leaf.MinU+leaf.MaxU)*.5,(leaf.MinV+leaf.MaxV)*.5);
            if(!budget.Consume())return EOutcome::Unknown;
            bool sampled=false;
            if(FixedProductWitnessControl::RuntimeEnabled())
            {
                sampled=EvaluateFixedProductUV(product,leaf.FixedParameter,leaf.FixedParameterPoint);
                // A failed optional evaluation stays paid. The old full-net
                // fallback needs its own unit and runs before publication.
                if(!sampled&&!budget.Consume())return EOutcome::Unknown;
            }
            if(!sampled&&!ProjectedDistanceDetail::EvaluateFixedUV(originalCandidates[index],leaf.FixedParameter,leaf.FixedParameterPoint))
                return EOutcome::Unknown;
            result.Leaves.Add(MoveTemp(leaf));
        }
    }
    if(result.Leaves.IsEmpty())return EOutcome::Unknown;
    out=MoveTemp(result);return EOutcome::Complete;
}

// Optional successor to the expensive full tensor restriction, AFTER the
// closed producer authenticates the exact rational derivative conclusion.
// This proves rank only; the consumed normal remains the radial contact normal.
inline bool ProveFixedProductLeafRank(const FProduct& product,
    const FReferenceDyadicCell& originalU,const FReferenceDyadicCell& originalV,
    FDistanceBudget& budget,FIntervalPoint& rawNormal)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    rawNormal={};if(!originalU.IsValid()||!originalV.IsValid()||
        product.FreeWorldAxis<0||product.FreeWorldAxis>=3||
        !Detail::ValidPoint(product.ExactCurveDerivativeHull))return false;
    const auto ur=originalU.GetRange(),vr=originalV.GetRange();
    if(!A::Valid(ur)||!A::Valid(vr)||!(ur.Lower>0&&ur.Upper<1&&vr.Lower>0&&vr.Upper<1))return false;
    const auto slope=A::Subtract(product.End,product.Start);
    if(!A::Valid(slope)||!(slope.Lower>0||slope.Upper<0))return false;
    const int32 axis=product.FreeWorldAxis;
    if(A::Bits(product.ExactCurveDerivativeHull.Axis[axis].Lower)!=0||
        A::Bits(product.ExactCurveDerivativeHull.Axis[axis].Upper)!=0)return false;
    auto free=Detail::Point(FVector3d::ZeroVector);free.Axis[axis]=slope;
    auto du=product.FreeAlongU?free:product.ExactCurveDerivativeHull;
    auto dv=product.FreeAlongU?product.ExactCurveDerivativeHull:free;
    const double uw=A::FromBits(uint64(1023-originalU.Depth)<<52);
    const double vw=A::FromBits(uint64(1023-originalV.Depth)<<52);
    // Same local-chart derivatives as full restriction, using exact dyadic
    // scale factors. The global curve hull encloses the retained local curve.
    for(int32 d=0;d<3;++d)
    {
        if(!budget.Consume())return false;
        du.Axis[d]=A::Multiply(du.Axis[d],Interval::Exact(uw));
        dv.Axis[d]=A::Multiply(dv.Axis[d],Interval::Exact(vw));
    }
    FIntervalPoint cross;bool nonzero=false;
    for(int32 d=0;d<3;++d)
    {
        if(!budget.Consume())return false;
        cross.Axis[d]=A::Subtract(A::Multiply(du.Axis[(d+1)%3],dv.Axis[(d+2)%3]),
            A::Multiply(du.Axis[(d+2)%3],dv.Axis[(d+1)%3]));
        if(!A::Valid(cross.Axis[d]))return false;
        nonzero|=cross.Axis[d].Lower>0||cross.Axis[d].Upper<0;
    }
    if(!nonzero)return false;rawNormal=MoveTemp(cross);return true;
}
}
