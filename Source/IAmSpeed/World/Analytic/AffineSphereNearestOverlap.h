#pragma once
#include "SignedAffineSphereResidual.h"
#include "KinematicCenterPathBounds.h"

namespace Speed::Analytic::Certified::AffineSphereNearestOverlap
{
namespace A = ::AnalyticCertified;
namespace R = SignedAffineSphereResidual;
using AffineReferenceDetail::FProduct;

// Values-only draft. The caller must first select the complete original
// union, apply its exact query policy, and authenticate the affine products.
enum class EResult : uint8 { NotApplicable, NoOverlap, UniqueOverlap, Indeterminate, Work, Arithmetic };
struct FBounds
{
    int32 OriginalProvider = INDEX_NONE;
    Interval OriginalFreeParameter;
    Interval OriginalCurveV;
    FIntervalPoint ClosestPoint;
    FIntervalPoint RadialNormal;
    Interval DistanceSquared;
    Interval PenetrationDepth;
    float Depth32 = 0.f;
    bool Depth32Unique = false;
};

namespace Private
{
struct FLeaf
{
    int32 ProductIndex = INDEX_NONE;
    FIntervalPoint ProjectedCenter;
    Detail::FNode Node;
};

inline bool Interior(const FProduct& product,const FIntervalPoint& center)
{
    if(!R::Private::Admits(product)||!R::Private::ValidProduct(product))return false;
    const auto axis=center.Axis[product.FreeWorldAxis];
    const bool increasing=product.End.Lower>product.Start.Upper;
    const auto low=increasing?product.Start:product.End;
    const auto high=increasing?product.End:product.Start;
    return axis.Lower>low.Upper&&axis.Upper<high.Lower;
}

inline Interval SquaredDistance(const FIntervalPoint& a,const FIntervalPoint& b)
{
    auto result=Interval::Exact(0);
    for(int32 axis=0;axis<3;++axis)
        result=A::Add(result,A::Square(A::Subtract(a.Axis[axis],b.Axis[axis])));
    return result;
}

// sqrt is only a proposal. Squaring the expanded endpoints must certify the
// enclosure, so the proof does not assume a particular platform libm error.
inline bool PositiveSqrt(Interval squared,Interval& out)
{
    out={};
    if(!A::Valid(squared)||!(squared.Lower>0))return false;
    double lower=FMath::Sqrt(squared.Lower),upper=FMath::Sqrt(squared.Upper);
    for(int32 step=0;step<4;++step)
    {
        if(!A::Outward(lower,false,lower)||!A::Outward(upper,true,upper)||!(lower>0))return false;
        const auto low=A::Square(Interval::Exact(lower));
        const auto high=A::Square(Interval::Exact(upper));
        if(A::Valid(low)&&A::Valid(high)&&low.Upper<=squared.Lower&&high.Lower>=squared.Upper)
        {
            out={lower,upper,true};
            return A::Valid(out);
        }
    }
    return false;
}

inline bool PointOnRestrictedCurve(const Detail::FNode& node,Interval v,FIntervalPoint& out)
{
    out={};
    if(node.DegreeU!=0||node.DegreeV!=5||node.Net.Num()!=6||!A::Valid(v)||v.Lower<0||v.Upper>1)
        return false;
    auto work=node.Net;
    for(int32 level=node.DegreeV;level>0;--level)
        for(int32 i=0;i<level;++i)
            work[i]=ProjectedDistanceDetail::LerpPoint(work[i],work[i+1],v);
    if(!Detail::ValidPoint(work[0]))return false;
    out=work[0];
    return true;
}

inline StrongConvexCurveMinimum::EResult BoundLeaf(const FLeaf& leaf,FDistanceBudget& budget,
    FTensorDistanceBounds& minimum,StrongConvexCurveMinimum::FInspection* inspection)
{
    FIntervalTensorSurface reduced;
    reduced.DegreeV=leaf.Node.DegreeV;
    for(const auto& cp:leaf.Node.Net)reduced.ControlPoints.Add(cp);
    return StrongConvexCurveMinimum::TryBound(reduced,leaf.ProjectedCenter,budget,36,minimum,inspection);
}

}

// Require one surviving convex half-cell across the complete provider union.
// A tie, nonconvex survivor, or unresolved output is refused.
inline EResult Bound(TConstArrayView<FProduct> products,TConstArrayView<int32> originalProviders,
    const FIntervalPoint& center,double radius,FDistanceBudget& budget,FBounds& out)
{
    out={};
    const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(products.IsEmpty()||products.Num()!=originalProviders.Num()||products.Num()>8||
        !ContactDetail::ProjectionSingleton(center)||!A::Finite(radius)||!(radius>0))
        return EResult::NotApplicable;
    const auto radiusSquared=A::Square(Interval::Exact(radius));
    R::FCenterRange shifted;
    if(!A::Valid(radiusSquared)||!R::BuildRoundedRange(center,radius,shifted))return EResult::Arithmetic;

    TArray<Private::FLeaf,TInlineAllocator<8>> roots;
    double bestUpper=radiusSquared.Upper;
    bool haveInsideWitness=false;
    for(int32 index=0;index<products.Num();++index)
    {
        const auto& product=products[index];
        if(originalProviders[index]<0||!Private::Interior(product,center))return EResult::NotApplicable;
        if(!budget.Consume())return EResult::Work;
        auto& root=roots.AddDefaulted_GetRef();
        root.ProductIndex=index;
        root.ProjectedCenter=center;
        root.ProjectedCenter.Axis[product.FreeWorldAxis]=Interval::Exact(0);
        root.Node.DegreeV=5;
        for(const auto& cp:product.Curve.ControlPoints)root.Node.Net.Add(cp);
        if(!Detail::Bounds(root.Node,root.ProjectedCenter))return EResult::Arithmetic;

        double v=0;
        if(!R::ProposeCurveV(product,shifted,v))continue;
        R::FWitness witness;
        const auto decision=R::ProveInside(product,shifted,v,budget,witness);
        if(decision==R::EDecision::Work)return EResult::Work;
        if(decision==R::EDecision::Arithmetic)return EResult::Arithmetic;
        if(decision!=R::EDecision::Inside)continue;
        const auto distance=Private::SquaredDistance(center,witness.OriginalPoint);
        if(!A::Valid(distance))return EResult::Arithmetic;
        bestUpper=FMath::Min(bestUpper,distance.Upper);
        haveInsideWitness=true;
    }

    TArray<Private::FLeaf,TInlineAllocator<2>> survivors;
    for(const auto& root:roots)
    {
        if(root.Node.LowerSquared>bestUpper)continue;
        if(!budget.Consume())return EResult::Work;
        Detail::FNode a,b;
        if(!Detail::Split(root.Node,false,a,b))return EResult::Arithmetic;
        for(auto* node:{&a,&b})
        {
            if(!budget.Consume())return EResult::Work;
            if(!Detail::Bounds(*node,root.ProjectedCenter))return EResult::Arithmetic;
            if(node->LowerSquared<=bestUpper)
                survivors.Add(Private::FLeaf{root.ProductIndex,root.ProjectedCenter,MoveTemp(*node)});
        }
    }
    if(survivors.IsEmpty())return haveInsideWitness?EResult::Arithmetic:EResult::NoOverlap;
    // A coarse directional hull can retain a competing half even when its
    // exact constrained minimum is strictly farther than the inside witness.
    // Only such a strict lower bound may remove a competitor. This loop does
    // not select a winner on a tie, a missing bound, or an exhausted account.
    using EConvex=StrongConvexCurveMinimum::EResult;
    if(haveInsideWitness)
        for(int32 i=survivors.Num()-1;i>=0&&survivors.Num()>1;--i)
        {
            FTensorDistanceBounds bound;
            const auto status=Private::BoundLeaf(survivors[i],budget,bound,nullptr);
            if(status==EConvex::Work)return EResult::Work;
            if(status==EConvex::Arithmetic)return EResult::Arithmetic;
            if(status==EConvex::Bounded&&bound.LowerSquared>bestUpper)
                survivors.RemoveAt(i,1,EAllowShrinking::No);
        }
    if(survivors.Num()!=1)return EResult::Indeterminate;

    const auto& winner=survivors[0];
    StrongConvexCurveMinimum::FInspection inspection;
    FTensorDistanceBounds minimum;
    const auto status=Private::BoundLeaf(winner,budget,minimum,&inspection);
    if(status==StrongConvexCurveMinimum::EResult::Work)return EResult::Work;
    if(status==StrongConvexCurveMinimum::EResult::Arithmetic)return EResult::Arithmetic;
    if(status!=StrongConvexCurveMinimum::EResult::Bounded)return EResult::Indeterminate;

    if(minimum.LowerSquared>radiusSquared.Upper)
        return haveInsideWitness?EResult::Arithmetic:EResult::NoOverlap;
    const Interval squared={minimum.LowerSquared,FMath::Min(minimum.UpperSquared,bestUpper),true};
    if(!A::Valid(squared))return EResult::Arithmetic;
    if(!(squared.Upper<radiusSquared.Lower))return EResult::Indeterminate;
    if(!budget.Consume())return EResult::Work;
    const Interval localV={inspection.ParameterLower,inspection.ParameterUpper,true};
    FBounds bounds;
    if(!Private::PointOnRestrictedCurve(winner.Node,localV,bounds.ClosestPoint))return EResult::Arithmetic;
    const auto& product=products[winner.ProductIndex];
    const auto freeAxis=product.FreeWorldAxis;
    bounds.ClosestPoint.Axis[freeAxis]=center.Axis[freeAxis];
    if(!AffineReferenceDetail::FreeParameter(product,center,bounds.OriginalFreeParameter))
        return EResult::Arithmetic;
    const auto width=A::Subtract(Interval::Exact(winner.Node.MaxV),Interval::Exact(winner.Node.MinV));
    bounds.OriginalCurveV=A::Add(Interval::Exact(winner.Node.MinV),A::Multiply(width,localV));
    if(!A::Valid(bounds.OriginalCurveV))return EResult::Arithmetic;

    Interval distance;
    if(!Private::PositiveSqrt(squared,distance))return EResult::Indeterminate;
    for(int32 axis=0;axis<3;++axis)
    {
        if(axis==freeAxis)
        {
            bounds.RadialNormal.Axis[axis]=Interval::Exact(0);
            continue;
        }
        if(!ContactDetail::ProjectionDividePositive(A::Subtract(center.Axis[axis],bounds.ClosestPoint.Axis[axis]),
            distance,bounds.RadialNormal.Axis[axis]))return EResult::Arithmetic;
    }
    bounds.PenetrationDepth=A::Subtract(Interval::Exact(radius),distance);
    if(!A::Valid(bounds.PenetrationDepth)||!(bounds.PenetrationDepth.Lower>0))return EResult::Indeterminate;
    const float low=float(bounds.PenetrationDepth.Lower),high=float(bounds.PenetrationDepth.Upper);
    bounds.Depth32Unique=A::Finite(double(low))&&low>0&&
        KinematicCenterDetail::Bits(low)==KinematicCenterDetail::Bits(high);
    if(bounds.Depth32Unique)bounds.Depth32=low;
    bounds.DistanceSquared=squared;
    bounds.OriginalProvider=originalProviders[winner.ProductIndex];
    if(RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=
        numeric.ControlFingerprint())return EResult::Arithmetic;
    out=bounds;
    return EResult::UniqueOverlap;
}
}
