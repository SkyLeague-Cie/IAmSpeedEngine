#pragma once
#include "TensorDistanceBounds.h"

namespace Speed::Analytic::Certified::SignedC2FiniteRival
{
namespace A=::AnalyticCertified;
using I=A::Interval;
using P=FIntervalPoint;

// Valid only for the exact authenticated 1940 finite Y extrusion. Within
// this box the original U=0 edge is closest in Y and the whole V objective
// has a half-squared-distance Hessian at least 500. The caller owns the
// signed provider, scene, filters, and all other contender proofs.
inline bool ContainsOriginal(const P& point)
{
    const auto& x=point.Axis[0];
    const auto& y=point.Axis[1];
    const auto& z=point.Axis[2];
    if(!Detail::ValidPoint(point)||x.Lower<4968.25||x.Upper>4974.5||
        y.Lower< -1342.5||y.Upper> -1338.5||z.Lower<93.171875||z.Upper>95.5)return false;
    const auto first=A::Subtract(x,I::Exact(4968.125));
    const auto side=A::Subtract(y,A::Add(I::Exact(-1340.1875),A::Multiply(I::Exact(-9./32.),first)));
    const auto rise=A::Subtract(z,A::Add(I::Exact(93.3046875),A::Multiply(I::Exact(21./256.),first)));
    return A::Valid(side)&&A::Valid(rise)&&side.Lower>=0.&&rise.Lower>=0.;
}

// A second convex signed chart cell. The original 695 U/V chart and its
// Hessian bound remain valid, and the complete other original source roles
// clear the inflated shell except 1940, which still needs its own live proof.
inline bool ContainsExtended(const P& point)
{
    const auto& x=point.Axis[0];
    const auto& y=point.Axis[1];
    const auto& z=point.Axis[2];
    if(!Detail::ValidPoint(point)||x.Lower<4971.25||x.Upper>4978.||
        y.Lower< -1342.5||y.Upper> -1338.5||z.Lower<93.6875||z.Upper>96.25)return false;
    const auto side=A::Subtract(y,A::Add(I::Exact(-1340.1875),A::Multiply(I::Exact(-9./32.),
        A::Subtract(x,I::Exact(4968.125)))));
    const auto rise=A::Subtract(z,A::Add(I::Exact(93.6875),A::Multiply(I::Exact(3./16.),
        A::Subtract(x,I::Exact(4971.25)))));
    return A::Valid(side)&&A::Valid(rise)&&side.Lower>=0.&&rise.Lower>=0.;
}

// Approach to the signed 695/1940 seam on the phase-5 side. The same
// interior chart and the complete inflated-shell roster are certified
// here. The true finite 1940 edge still needs the live rival proof.
inline bool ContainsApproach(const P& point)
{
    const auto& x=point.Axis[0];
    const auto& y=point.Axis[1];
    const auto& z=point.Axis[2];
    if(!Detail::ValidPoint(point)||x.Lower<4976.25||x.Upper>4979.875||
        y.Lower< -1343.25||y.Upper> -1340.75||z.Lower<94.875||z.Upper>97.25)return false;
    const auto side=A::Subtract(y,A::Add(I::Exact(-1342.5625),A::Multiply(I::Exact(-9./32.),
        A::Subtract(x,I::Exact(4976.5625)))));
    const auto rise=A::Subtract(z,A::Add(I::Exact(94.875),A::Multiply(I::Exact(1./4.),
        A::Subtract(x,I::Exact(4976.25)))));
    return A::Valid(side)&&A::Valid(rise)&&side.Lower>=0.&&rise.Lower>=0.;
}

inline bool Contains(const P& point)
{
    return ContainsOriginal(point)||ContainsExtended(point)||ContainsApproach(point);
}

inline bool ContainsPath(const P& first,const P& middle,const P& last)
{
    // A quadratic lies in the convex hull of its controls. Select one
    // certified cell for the whole span before using that property.
    return (ContainsOriginal(first)&&ContainsOriginal(middle)&&ContainsOriginal(last))||
        (ContainsExtended(first)&&ContainsExtended(middle)&&ContainsExtended(last))||
        (ContainsApproach(first)&&ContainsApproach(middle)&&ContainsApproach(last));
}


// Independently signed stationary approach cell, kept separate from dynamic
// first-arrival and native prefix admission. Y remains above the finite 1940 edge.
// The later stationary cell is on the same side of the finite 1940
// edge. The complete shell roster and original 695 chart are retained.
inline bool ContainsLaterSeamProjection(const P& point)
{
    return Detail::ValidPoint(point)&&
        point.Axis[0].Lower>=4981.625&&point.Axis[0].Upper<=4981.75&&
        point.Axis[1].Lower>=-1343.984375&&point.Axis[1].Upper<=-1343.90625&&
        point.Axis[2].Lower>=96.703125&&point.Axis[2].Upper<=96.796875;
}

inline bool ContainsSeamProjection(const P& point)
{
    if(!Detail::ValidPoint(point))return false;
    return (point.Axis[0].Lower>=4979.75&&point.Axis[0].Upper<=4980.375&&
        point.Axis[1].Lower>=-1343.625&&point.Axis[1].Upper<=-1343.125&&
        point.Axis[2].Lower>=96.&&point.Axis[2].Upper<=96.5)||
        // The sibling cell shares the old X boundary. Its narrower Y and Z
        // ranges keep the entire original signed roster and finite-edge proof.
        (point.Axis[0].Lower>=4980.375&&point.Axis[0].Upper<=4980.4375&&
        point.Axis[1].Lower>=-1343.59375&&point.Axis[1].Upper<=-1343.5625&&
        point.Axis[2].Lower>=96.28125&&point.Axis[2].Upper<=96.3125)||
        ContainsLaterSeamProjection(point);
}

inline I Scale(double scale,I value){return A::Multiply(I::Exact(scale),value);}

// Six times the degree-four Bernstein coefficients of a quadratic square.
inline void SquareCoefficients(const I (&q)[3],I (&out)[5])
{
    out[0]=Scale(6.,A::Square(q[0]));
    out[1]=Scale(6.,A::Multiply(q[0],q[1]));
    out[2]=A::Add(Scale(2.,A::Multiply(q[0],q[2])),Scale(4.,A::Square(q[1])));
    out[3]=Scale(6.,A::Multiply(q[1],q[2]));
    out[4]=Scale(6.,A::Square(q[2]));
}

inline I Evaluate(I* work,int32 count,I v)
{
    const auto oneMinus=A::Subtract(I::Exact(1.),v);
    while(count>1)
    {
        for(int32 i=0;i<count-1;++i)
            work[i]=A::Add(A::Multiply(oneMinus,work[i]),A::Multiply(v,work[i+1]));
        --count;
    }
    return work[0];
}

inline bool ProveOnCheckedFiniteEdge(const FIntervalTensorSurface& source,const P (&centers)[3],double proposalV,double radius,
    FDistanceBudget& budget,double& scaledLower,double certifiedHalfHessian=500.)
{
    scaledLower=0.;
    if(!A::Finite(certifiedHalfHessian)||certifiedHalfHessian<=0.||source.DegreeU!=5||source.DegreeV!=5||source.ControlPoints.Num()!=36||
        !A::Finite(proposalV)||proposalV<0.||proposalV>1.||
        !A::Finite(radius)||!(radius>0.)||radius>93.165008544921875)return false;
    const auto v=I::Exact(proposalV);
    P sample,tangent;
    sample.Axis[1]=I::Exact(-1344.);tangent.Axis[1]=I::Exact(0.);
    for(int32 d=0;d<3;d+=2)
    {
        if(!budget.Consume())return false;
        I edge[6],slope[5];
        for(int32 j=0;j<6;++j)
        {
            edge[j]=source.ControlPoints[j].Axis[d];
            if(!A::Valid(edge[j]))return false;
        }
        for(int32 j=0;j<5;++j)slope[j]=Scale(5.,A::Subtract(edge[j+1],edge[j]));
        sample.Axis[d]=Evaluate(edge,6,v);
        tangent.Axis[d]=Evaluate(slope,5,v);
    }
    if(!Detail::ValidPoint(sample)||!Detail::ValidPoint(tangent)||!budget.Consume())return false;
    I gradient[3],norm[5];
    for(auto& value:gradient)value=I::Exact(0.);
    for(auto& value:norm)value=I::Exact(0.);
    for(int32 d=0;d<3;++d)
    {
        I delta[3];
        for(int32 j=0;j<3;++j)
        {
            delta[j]=A::Subtract(sample.Axis[d],centers[j].Axis[d]);
            gradient[j]=A::Add(gradient[j],A::Multiply(delta[j],tangent.Axis[d]));
        }
        I squared[5];
        SquareCoefficients(delta,squared);
        for(int32 j=0;j<5;++j)norm[j]=A::Add(norm[j],squared[j]);
    }
    I gradientSquared[5];
    SquareCoefficients(gradient,gradientSquared);
    const auto r2=Scale(6.,A::Square(I::Exact(radius)));
    double lower=TNumericLimits<double>::Max();
    for(int32 j=0;j<5;++j)
    {
        // The authenticated caller proved half the objective curvature.
        // Its unconstrained quadratic minimum is a valid lower bound
        // even when the proposal is an original endpoint.
        const auto value=A::Subtract(Scale(certifiedHalfHessian,A::Subtract(norm[j],r2)),gradientSquared[j]);
        if(!A::Valid(value)||value.Lower<=0.)return false;
        const auto scaled=Scale(2.,value);
        if(!A::Valid(scaled))return false;
        lower=FMath::Min(lower,scaled.Lower);
    }
    scaledLower=lower;
    return lower>0.;
}

inline bool Prove(const FIntervalTensorSurface& source,const P (&centers)[3],double proposalV,double radius,
    FDistanceBudget& budget,double& scaledLower,bool staticSeam=false)
{
    scaledLower=0.;
    if(staticSeam)
    {
        // This extension covers only the identical singleton from the
        // original stationary projection and uses the same finite U=0 edge.
        if(!ContainsSeamProjection(centers[0]))return false;
        for(int32 d=0;d<3;++d)
        {
            const auto base=centers[0].Axis[d];
            if(A::Bits(base.Lower)!=A::Bits(base.Upper))return false;
            for(int32 k=1;k<3;++k)
                if(!A::Valid(centers[k].Axis[d])||A::Bits(centers[k].Axis[d].Lower)!=A::Bits(base.Lower)||
                    A::Bits(centers[k].Axis[d].Upper)!=A::Bits(base.Upper))return false;
        }
    }
    else if(!ContainsPath(centers[0],centers[1],centers[2]))return false;
    return ProveOnCheckedFiniteEdge(source,centers,proposalV,radius,budget,scaledLower);
}
}
