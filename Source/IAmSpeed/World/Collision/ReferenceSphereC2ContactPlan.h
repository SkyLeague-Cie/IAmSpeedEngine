#pragma once

#include "PreparedSphereStrictSeparation.h"
#include "PreparedSweepEvidence.h"
#if !UE_BUILD_SHIPPING
#include "ReferenceSphereC2VariableTime.h"
#include "ReferenceSphereLaterC2Tube.h"
#include "ReferenceSphereBroadC2Tube.h"
#include "ReferenceSphereC2FixedRoundedPrefix.h"
#include "ReferenceSphereC2VariableRoundedPrefix.h"
#include "ReferenceSpherePostSeamC2.h"
#endif

namespace Speed::Analytic::Prepared
{
// Operation-local, proof-only prefix on an original complete partition.
// Other providers are discharged for the full original path once.
class FReferenceSphereC2Prefix final
{
    friend class FReferenceRepeatedSphereFloorPlan;
    friend class FReferenceRepeatedSphereC2Edge;
    friend class FReferenceSphereGeneralFirstArrivalRs;
    using I=C::Interval;
    using P=C::FIntervalPoint;
    struct FArc
    {
        double XDerivative=0.,YV=0.,EdgeX=0.,XU=0.,YU=0.,HeightFactor=0.;
        double XFirst=0.,XVMax=0.,ChartXU=0.,ChartYU=0.,YTop=0.,YBottom=0.,Z3Min=0.;
        double ZUpper[6]{},ZU[6]{},ZV[5]{};
    };
    const FReferenceSphereStrictSeparation* Initial=nullptr;
    const C::FReferenceCandidateSet* Partition=nullptr;
    int32 LocalEdge=INDEX_NONE;
    FArc Edge;
    uint64 Numeric=0;
    bool Rounded=false;
#if !UE_BUILD_SHIPPING
    const PostSeamC2::FProof* PostSeamFresh=nullptr;
    uint32 PostSeamFreshEndBits=0;
    static bool BindPostSeamFresh(const PostSeamC2::FProof& proof,float end,FReferenceSphereC2Prefix& out)
    {
        if(!proof.TimeWithinCover(end))return false;
        out.PostSeamFresh=&proof;
        out.PostSeamFreshEndBits=C::KinematicCenterDetail::Bits(end);
        return true;
    }

    const PreparedExecution::FSelectedWholeHullPartition* FixedFreshRc=nullptr;
    uint32 FixedFreshEndBits=0;
    const PreparedExecution::FSelectedWholeHullPartition* VariableFreshRc=nullptr;
    uint32 VariableFreshEndBits=0;
    TOptional<FVector2d> VariableFreshLaterUV;
    TOptional<VariableC2Time::FNativeProposals> VariableFreshBroadUV;

    static bool BindVariableFresh(const FReferenceSphereStrictSeparation& initial,const C::FReferenceCandidateSet& rs,
        const PreparedExecution::FSelectedWholeHullPartition& rc,float end,const FVector2d* laterUV,
        const VariableC2Time::FNativeProposals* broadUV,FReferenceSphereC2Prefix& out)
    {
        const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(numeric.Status!=EAnalyticNumericStatus::Complete||!VariableC2RoundedPrefix::Binds(initial,rs,rc,end,laterUV!=nullptr,broadUV))
            return false;
        FReferenceSphereC2Prefix fresh;
        fresh.Initial=&initial;fresh.Partition=&rs;fresh.VariableFreshRc=&rc;
        if(laterUV)fresh.VariableFreshLaterUV=*laterUV;
        if(broadUV)fresh.VariableFreshBroadUV=*broadUV;
        fresh.VariableFreshEndBits=C::KinematicCenterDetail::Bits(end);fresh.Numeric=numeric.ControlFingerprint();
        out=fresh;return true;
    }

    static bool BindFixedFresh(const FReferenceSphereStrictSeparation& initial,const C::FReferenceCandidateSet& rs,
        const PreparedExecution::FSelectedWholeHullPartition& rc,float end,FReferenceSphereC2Prefix& out)
    {
        const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(numeric.Status!=EAnalyticNumericStatus::Complete||!FixedC2RoundedPrefix::MatchesCapture(initial.GetCapturedInput(),end))
            return false;
        FReferenceSphereC2Prefix fresh;
        fresh.Initial=&initial;fresh.Partition=&rs;fresh.FixedFreshRc=&rc;
        fresh.FixedFreshEndBits=C::KinematicCenterDetail::Bits(end);fresh.Numeric=numeric.ControlFingerprint();
        out=fresh;return true;
    }
#endif

    static double AbsMax(I v) {return FMath::Max(FMath::Abs(v.Lower),FMath::Abs(v.Upper));}
    static double Middle(I v) {return .5*v.Lower+.5*v.Upper;}
    static I Divide(I a,I b)
    {
        I result;
        C::ContactDetail::ProjectionDividePositive(a,b,result);
        return result;
    }
    static bool ExactZero(I v)
    {
        return (AnalyticCertified::Bits(v.Lower)&AnalyticCertified::Magnitude)==0&&
            (AnalyticCertified::Bits(v.Upper)&AnalyticCertified::Magnitude)==0;
    }
    static bool SamePoint(const P& a,const P& b)
    {
        if(!C::Detail::ValidPoint(a)||!C::Detail::ValidPoint(b))return false;
        for(int32 d=0;d<3;++d)
            if(AnalyticCertified::Bits(a.Axis[d].Lower)!=AnalyticCertified::Bits(b.Axis[d].Lower)||
                AnalyticCertified::Bits(a.Axis[d].Upper)!=AnalyticCertified::Bits(b.Axis[d].Upper))return false;
        return true;
    }
    static bool SamePath(const C::FIntervalBezierCenterPath& a,const C::FIntervalBezierCenterPath& b)
    {
        if(a.Degree!=b.Degree||a.ControlPoints.Num()!=b.ControlPoints.Num())return false;
        for(int32 i=0;i<a.ControlPoints.Num();++i)
            if(!SamePoint(a.ControlPoints[i],b.ControlPoints[i]))return false;
        return C::CenterDetail::Valid(a)&&C::CenterDetail::Valid(b);
    }
    static bool BuildArc(const C::FIntervalTensorSurface& surface,C::FDistanceBudget& budget,FArc& out)
    {
        namespace A=::AnalyticCertified;
        out={};
        if(surface.DegreeU!=5||surface.DegreeV!=5||surface.ControlPoints.Num()!=36)return false;
        const auto at=[&](int32 u,int32 v)->const P&{return surface.ControlPoints[u*6+v];};
        const auto five=I::Exact(5.);
        I third;
        if(!C::ContactDetail::ProjectionDividePositive(I::Exact(1.),I::Exact(3.),third))return false;
        const auto tenThirds=A::Multiply(I::Exact(10.),third);
        const auto fiveThirds=A::Multiply(five,third);
        const double big=TNumericLimits<double>::Max();
        double xMin=big,edgeMin=big,edgeYMin=big;
        out.XFirst=out.ChartYU=out.YTop=out.Z3Min=big;
        out.YBottom=-big;
        for(int32 u=0;u<6;++u)
        {
            if(!budget.Consume())return false;
            for(int32 v=0;v<6;++v)
            {
                const auto& p=at(u,v);
                if(!C::Detail::ValidPoint(p)||(v<3&&!ExactZero(p.Axis[2])))return false;
                if(v==0)edgeMin=FMath::Min(edgeMin,p.Axis[0].Lower);
                if(u==0)out.YTop=FMath::Min(out.YTop,p.Axis[1].Lower);
                if(u==5)out.YBottom=FMath::Max(out.YBottom,p.Axis[1].Upper);
                if(v==3)out.Z3Min=FMath::Min(out.Z3Min,p.Axis[2].Lower);
                out.ZUpper[v]=FMath::Max(out.ZUpper[v],p.Axis[2].Upper);
                if(v>0)
                {
                    const auto& previous=at(u,v-1);
                    const auto x=A::Multiply(five,A::Subtract(p.Axis[0],previous.Axis[0]));
                    const auto y=A::Multiply(five,A::Subtract(p.Axis[1],previous.Axis[1]));
                    const auto z=A::Multiply(five,A::Subtract(p.Axis[2],previous.Axis[2]));
                    if(!A::Valid(x)||x.Lower<=0.||!A::Valid(y)||!A::Valid(z))return false;
                    xMin=FMath::Min(xMin,x.Lower);out.YV=FMath::Max(out.YV,AbsMax(y));
                    out.XVMax=FMath::Max(out.XVMax,x.Upper);
                    if(v==1)out.XFirst=FMath::Min(out.XFirst,x.Lower);
                    out.ZV[v-1]=FMath::Max(out.ZV[v-1],AbsMax(z));
                }
                if(u>0)
                {
                    const auto& previous=at(u-1,v);
                    const auto x=A::Multiply(five,A::Subtract(p.Axis[0],previous.Axis[0]));
                    const auto y=A::Multiply(five,A::Subtract(previous.Axis[1],p.Axis[1]));
                    const auto z=A::Multiply(five,A::Subtract(p.Axis[2],previous.Axis[2]));
                    if(!A::Valid(x)||!A::Valid(y)||!A::Valid(z))return false;
                    out.ChartXU=FMath::Max(out.ChartXU,AbsMax(x));out.ChartYU=FMath::Min(out.ChartYU,y.Lower);
                    out.ZU[v]=FMath::Max(out.ZU[v],AbsMax(z));
                    if(v==0)
                    {
                        if(y.Lower<=0.)return false;
                        out.XU=FMath::Max(out.XU,AbsMax(x));edgeYMin=FMath::Min(edgeYMin,y.Lower);
                    }
                }
            }
            // Z/v^2 has degree 3 and Bernstein controls
            // [0, (10/3)Z3, (5/3)Z4, Z5].
            const auto c3=A::Multiply(tenThirds,at(u,3).Axis[2]);
            const auto c4=A::Multiply(fiveThirds,at(u,4).Axis[2]);
            if(!A::Valid(c3)||!A::Valid(c4))return false;
            out.HeightFactor=FMath::Max(out.HeightFactor,FMath::Max(FMath::Max(c3.Upper,c4.Upper),at(u,5).Axis[2].Upper));
        }
        out.XDerivative=xMin;out.EdgeX=edgeMin;out.YU=edgeYMin;
        return out.HeightFactor>0.;
    }
    static bool ClearProjection(const C::FIntervalTensorSurface& surface,const P& centers,I r2,C::FDistanceBudget& budget)
    {
        namespace A=::AnalyticCertified;
        // A direction proposal only. The bound uses every original control.
        const auto dot=[](const P& p){return A::Subtract(A::Multiply(I::Exact(3.),p.Axis[0]),A::Multiply(I::Exact(4.),p.Axis[2]));};
        const auto center=dot(centers);
        if(!A::Valid(center)||surface.ControlPoints.IsEmpty())return false;
        double lower=TNumericLimits<double>::Max();
        for(const auto& point:surface.ControlPoints)
        {
            if(!budget.Consume()||!C::Detail::ValidPoint(point))return false;
            const auto projected=dot(point);
            if(!A::Valid(projected))return false;
            lower=FMath::Min(lower,projected.Lower);
        }
        const auto separation=A::Subtract(I::Exact(lower),I::Exact(center.Upper));
        const auto goal=A::Multiply(I::Exact(25.),r2);
        const auto squared=A::Square(separation);
        return A::Valid(separation)&&separation.Lower>0.&&A::Valid(goal)&&A::Valid(squared)&&squared.Lower>goal.Upper;
    }
    static bool ClearTransverse(const C::FIntervalTensorSurface& surface,const P& centers,const FArc& arc,I z,I flatGap2)
    {
        namespace A=::AnalyticCertified;
        P net;
        if(!C::CandidateDetail::SurfaceHull(surface,net))return false;
        const auto forward=A::Subtract(I::Exact(centers.Axis[0].Upper),I::Exact(arc.EdgeX));
        const auto after=A::Subtract(I::Exact(centers.Axis[1].Lower),I::Exact(net.Axis[1].Upper));
        const auto before=A::Subtract(I::Exact(net.Axis[1].Lower),I::Exact(centers.Axis[1].Upper));
        if(!A::Valid(forward)||!A::Valid(after)||!A::Valid(before))return false;
        const double dy=FMath::Max(after.Lower,before.Lower);
        if(dy<=0.)return false;
        const auto b=A::Multiply(A::Multiply(I::Exact(2.),z),I::Exact(arc.HeightFactor));
        const auto denominator=A::Subtract(A::Square(I::Exact(arc.XDerivative)),b);
        const auto numerator=A::Multiply(A::Square(I::Exact(FMath::Max(0.,forward.Upper))),b);
        I penalty;
        if(!C::ContactDetail::ProjectionDividePositive(numerator,denominator,penalty))return false;
        // Min_{v>=0} [max(0,k*v-d)^2 - b*v^2] = -d^2*b/(k^2-b).
        const auto gap=A::Subtract(A::Add(flatGap2,A::Square(I::Exact(dy))),penalty);
        return A::Valid(gap)&&gap.Lower>0.;
    }
    static I EdgeMargin(const C::FIntervalTensorSurface& surface,const P& endpoint,I lip,C::FDistanceBudget& budget)
    {
        namespace A=::AnalyticCertified;
        if(!budget.Consume())return {};
        const double u=C::ProjectedDistanceDetail::ChooseU(surface,endpoint,1);
        P edge;
        if(!C::ProjectedDistanceDetail::EvaluateFixedUV(surface,FVector2d(u,0.),edge))return {};
        const auto dy=A::Subtract(edge.Axis[1],endpoint.Axis[1]);
        const auto dx=A::Subtract(edge.Axis[0],endpoint.Axis[0]);
        if(!A::Valid(dx)||!A::Valid(dy))return {};
        return A::Subtract(dx,A::Multiply(lip,I::Exact(AbsMax(dy))));
    }
    static bool LateralCap(const FArc& arc,I z,double cap,double& lateral)
    {
        namespace A=::AnalyticCertified;
        I work[6];
        for(int32 v=0;v<6;++v)
        {
            if(v>0&&arc.ZUpper[v]<arc.ZUpper[v-1])return false;
            work[v]=I::Exact(arc.ZUpper[v]);
        }
        const auto t=I::Exact(cap),u=A::Subtract(I::Exact(1.),t);
        for(int32 n=5;n>0;--n)
            for(int32 i=0;i<n;++i)work[i]=A::Add(A::Multiply(u,work[i]),A::Multiply(t,work[i+1]));
        const auto bound=A::Multiply(A::Multiply(I::Exact(2.),z),work[0]);
        if(!A::Valid(bound)||bound.Upper<=0.)return false;
        // Sqrt is only a proposal. Its squared lower bound validates it.
        lateral=2.*FMath::Sqrt(bound.Upper);
        const auto checked=A::Square(I::Exact(lateral));
        return A::Valid(checked)&&checked.Lower>bound.Upper;
    }
    static bool ClearGraph(const C::FIntervalTensorSurface& surface,const FArc& arc,const P& centers,
        const P& referenceEndpoint,FVector2d drift,I z,I rs,I vx,I vy,I lip,I k,I beta,double delta,
        C::FDistanceBudget& budget)
    {
        namespace A=::AnalyticCertified;
        if(arc.ChartYU<=0.||arc.Z3Min<=0.)return false;
        const auto fullLip=Divide(I::Exact(arc.ChartXU),I::Exact(arc.ChartYU));
        const auto nearK=[&](double cap)
        {
            const auto complement=A::Subtract(I::Exact(1.),I::Exact(cap));
            return A::Subtract(A::Multiply(I::Exact(arc.XFirst),A::Square(A::Square(complement))),
                A::Multiply(fullLip,I::Exact(arc.YV)));
        };
        double cap=1./1024.;
        const auto outer=A::Subtract(k,Divide(I::Exact(delta),I::Exact(cap)));
        const auto outerSquared=A::Square(outer),need=A::Multiply(beta,A::Add(I::Exact(1.),A::Square(lip)));
        if(!A::Valid(outer)||outer.Lower<=0.||!A::Valid(outerSquared)||!A::Valid(need)||outerSquared.Lower<=need.Upper)
            return false;
        double lateral=0.;
        for(int32 i=0;i<4;++i)
        {
            if(!budget.Consume()||!LateralCap(arc,z,cap,lateral))return false;
            const auto candidate=Divide(A::Add(I::Exact(delta),A::Multiply(A::Add(I::Exact(1.),lip),I::Exact(lateral))),nearK(cap));
            if(!A::Valid(candidate))return false;
            if(candidate.Upper<=0.||candidate.Upper>=cap)break;
            cap=candidate.Upper;
        }
        const auto kNear=nearK(cap);
        const auto covered=A::Multiply(kNear,I::Exact(cap));
        if(!A::Valid(kNear)||kNear.Lower<=0.||!A::Valid(covered)||covered.Lower<=delta||
            !LateralCap(arc,z,cap,lateral))return false;
        const auto yRange=A::Add(centers.Axis[1],I{-lateral,lateral,true});
        if(!A::Valid(yRange)||yRange.Lower<=arc.YBottom||yRange.Upper>=arc.YTop)return false;

        // Z_u/v^3 and Z_v/v^2 have explicit Bernstein factors. Their
        // conservative derivatives stay small even with independent Y lanes.
        const auto t=I::Exact(cap),t2=A::Square(t),t3=A::Multiply(t,t2);
        const auto zUFactor=A::Add(A::Add(A::Multiply(I::Exact(10.),I::Exact(arc.ZU[3])),
            A::Multiply(A::Multiply(I::Exact(5.),I::Exact(arc.ZU[4])),t)),A::Multiply(I::Exact(arc.ZU[5]),t2));
        const auto zU=A::Multiply(t3,zUFactor);
        const auto zV=A::Add(A::Add(A::Multiply(A::Multiply(I::Exact(6.),I::Exact(arc.ZV[2])),t2),
            A::Multiply(A::Multiply(I::Exact(4.),I::Exact(arc.ZV[3])),t3)),A::Multiply(I::Exact(arc.ZV[4]),A::Square(t2)));
        const auto zVMinFactor=A::Multiply(A::Multiply(I::Exact(30.),I::Exact(arc.Z3Min)),A::Square(A::Subtract(I::Exact(1.),t)));
        const auto vyAbs=I::Exact(AbsMax(vy));
        const auto horizontalChange=A::Multiply(fullLip,vyAbs);
        const auto vLo=Divide(A::Subtract(I::Exact(vx.Lower),horizontalChange),A::Add(I::Exact(arc.XVMax),
            A::Multiply(fullLip,I::Exact(arc.YV))));
        const auto vHi=Divide(A::Add(I::Exact(vx.Upper),horizontalChange),kNear);
        const auto uHi=Divide(A::Add(vyAbs,A::Multiply(I::Exact(arc.YV),vHi)),I::Exact(arc.ChartYU));
        const auto rising=A::Subtract(A::Multiply(zVMinFactor,vLo),A::Multiply(A::Multiply(zUFactor,t),uHi));
        if(!A::Valid(vLo)||vLo.Lower<=0.||!A::Valid(rising)||rising.Lower<=0.)return false;
        const auto zUY=Divide(zU,I::Exact(arc.ChartYU));
        const auto lx=Divide(A::Add(zV,A::Multiply(zUY,I::Exact(arc.YV))),kNear);
        const auto ly=A::Add(zUY,A::Multiply(lx,fullLip));
        if(!A::Valid(lx)||!A::Valid(ly))return false;

        const auto x=Middle(referenceEndpoint.Axis[0]),y=Middle(referenceEndpoint.Axis[1]);
        const double y0=Middle(surface.ControlPoints[0].Axis[1]),y1=Middle(surface.ControlPoints[30].Axis[1]);
        const double dydu=y1-y0;
        if(!A::Finite(dydu)||dydu==0.)return false;
        double u=FMath::Clamp((y-y0)/dydu,0.,1.);
        P edge,tangent,point;
        const auto sample=[&](double sampleU,double sampleV,P& output)
        {return budget.Consume()&&C::ProjectedDistanceDetail::EvaluateFixedUV(surface,FVector2d(sampleU,sampleV),output);};
        constexpr double step=1./1024.;
        if(!sample(u,0.,edge)||!sample(u,step,tangent))return false;
        const double slope=(Middle(tangent.Axis[0])-Middle(edge.Axis[0]))/step;
        if(!A::Finite(slope)||slope<=0.)return false;
        double v=FMath::Clamp((x-Middle(edge.Axis[0]))/slope,0.,cap);
        if(!sample(u,v,point))return false;
        u=FMath::Clamp(u+(y-Middle(point.Axis[1]))/dydu,0.,1.);
        v=FMath::Clamp(v+(x-Middle(point.Axis[0]))/slope,0.,cap);
        if(!sample(u,v,point))return false;
        const auto ex=A::Subtract(referenceEndpoint.Axis[0],point.Axis[0]);
        const auto ey=A::Subtract(referenceEndpoint.Axis[1],point.Axis[1]);
        if(!A::Valid(ex)||!A::Valid(ey))return false;
        const auto errV=Divide(A::Add(I::Exact(AbsMax(ex)),A::Multiply(fullLip,I::Exact(AbsMax(ey)))),kNear);
        const auto errU=Divide(A::Add(I::Exact(AbsMax(ey)),A::Multiply(I::Exact(arc.YV),errV)),I::Exact(arc.ChartYU));
        const auto zMotion=A::Add(A::Multiply(lx,I::Exact(drift.X)),A::Multiply(ly,I::Exact(drift.Y)));
        const auto height=A::Add(A::Add(point.Axis[2],A::Add(A::Multiply(zV,errV),A::Multiply(zU,errU))),zMotion);
        const auto gap=A::Subtract(A::Subtract(z,rs),height),sum=A::Subtract(A::Add(z,rs),height);
        const auto loss=A::Multiply(A::Square(rs),A::Add(A::Square(lx),A::Square(ly)));
        const auto signedGap=A::Subtract(A::Multiply(gap,sum),loss);
        // Distance to the clamped original height chart is bounded below
        // by vertical^2/(1+|grad|^2). Use the factored signed residual.
        return A::Valid(gap)&&gap.Lower>0.&&A::Valid(signedGap)&&signedGap.Lower>0.;
    }
    static bool ClearEdgePrefix(const C::FIntervalTensorSurface& surface,const P& centers,const P& endpoint,
        const P& referenceEndpoint,FVector2d drift,const FArc& arc,const FStaticSphereInitialValues& captured,
        I lastSeconds,C::FDistanceBudget& budget)
    {
        namespace A=::AnalyticCertified;
        I lip;
        if(!C::ContactDetail::ProjectionDividePositive(I::Exact(arc.XU),I::Exact(arc.YU),lip))return false;
        const auto k=A::Subtract(I::Exact(arc.XDerivative),A::Multiply(lip,I::Exact(arc.YV)));
        const auto factor=A::Multiply(A::Multiply(I::Exact(2.),I::Exact(captured.State.Location.Z)),I::Exact(arc.HeightFactor));
        const auto need=A::Multiply(factor,A::Add(I::Exact(1.),A::Square(lip)));
        const auto k2=A::Square(k);
        if(!A::Valid(k)||k.Lower<=0.||!A::Valid(need)||!A::Valid(k2)||k2.Lower<=need.Upper)return false;
        // e(y) extends by clamping the original boundary parameter. Y_v may
        // be nonzero; the independently bounded deviation is in k above.
        const I t{0.,lastSeconds.Upper,true};
        const auto vx=A::Add(I::Exact(captured.State.Velocity.X),A::Multiply(I::Exact(captured.State.Acceleration.X),t));
        const auto vy=A::Add(I::Exact(captured.State.Velocity.Y),A::Multiply(I::Exact(captured.State.Acceleration.Y),t));
        if(!A::Valid(vx)||!A::Valid(vy))return false;
        const auto edgeSpeed=A::Multiply(lip,I::Exact(AbsMax(vy)));
        if(!A::Valid(edgeSpeed)||vx.Lower<=edgeSpeed.Upper)return false;
        const auto margin=EdgeMargin(surface,endpoint,lip,budget);
        if(!A::Valid(margin))return false;
        // x' > L|y'| makes the worst reference edge gap occur at B. The
        // consumed endpoint argument separately carries the uniform DAG drift.
        if(margin.Lower>0.)return true;
        return ClearGraph(surface,arc,centers,referenceEndpoint,drift,I::Exact(captured.State.Location.Z),
            I::Exact(captured.SupportRadius),vx,vy,lip,k,factor,-margin.Lower,budget);
    }
    static I Drift(const FStaticSphereInitialValues& v,int32 axis,I t)
    {
        namespace A=::AnalyticCertified;
        // Enclose original SBox float(t*t), binary64 vector products/adds and
        // the permitted contracted forms. The factors count original stages,
        // plus normal allowances for modes that flush subnormal operands.
        const auto t2=A::Square(t);
        const auto p=I::Exact(FMath::Abs(v.State.Location[axis]));
        const auto velocity=I::Exact(FMath::Abs(v.State.Velocity[axis]));
        const auto accel=I::Exact(FMath::Abs(v.State.Acceleration[axis]));
        const auto scale=A::Add(A::Add(p,A::Multiply(velocity,t)),A::Multiply(accel,t2));
        const auto binary64=A::Add(A::Multiply(I::Exact(16.*0x1p-52),scale),I::Exact(16.*0x1p-1022));
        const auto square32=A::Add(A::Multiply(I::Exact(2.*0x1p-23),t2),I::Exact(2.*0x1p-126));
        return A::Add(binary64,A::Multiply(accel,square32));
    }
    static bool Build(const FReferenceSphereStrictSeparation& initial,const C::FReferenceCandidateSet& partition,
        P centers,bool rounded,C::FDistanceBudget& budget,FReferenceSphereC2Prefix& out)
    {
        namespace A=::AnalyticCertified;
        const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        const auto& v=initial.GetCapturedInput();
        const auto z=I::Exact(v.State.Location.Z),rs=I::Exact(v.SupportRadius);
        if(numeric.Status!=EAnalyticNumericStatus::Complete||!C::Detail::ValidPoint(centers)||
            !A::Valid(z)||!A::Valid(rs)||initial.GetHorizon()>1.f||
            !C::CandidateDetail::PositiveRadius(v.SupportRadius)||z.Lower<=rs.Upper||
            !ExactZero(I::Exact(v.State.Velocity.Z))||!ExactZero(I::Exact(v.State.Acceleration.Z)))return false;
        centers.Axis[2]=z;
        const auto flatGap2=A::Multiply(A::Subtract(z,rs),A::Add(z,rs));
        const auto r2=A::Square(rs);
        const auto surfaces=partition.GetSurfacesForFixedSource();
        if(!A::Valid(flatGap2)||flatGap2.Lower<=0.||!A::Valid(r2)||
            surfaces.Num()!=partition.GetCandidateIndices().Num()||surfaces.IsEmpty())return false;
        FReferenceSphereC2Prefix plan;
        for(int32 i=0;i<surfaces.Num();++i)
        {
            const auto& surface=surfaces[i];
            if(!budget.Consume()||surface.ControlPoints.IsEmpty())return false;
            if(ClearProjection(surface,centers,r2,budget))continue;
            bool flat=true;
            for(const auto& cp:surface.ControlPoints)
            {
                if(!budget.Consume()||!C::Detail::ValidPoint(cp))return false;
                if(!ExactZero(cp.Axis[2])){flat=false;break;}
            }
            if(flat)continue;
            FArc arc;
            if(!BuildArc(surface,budget,arc))return false;
            if(ClearTransverse(surface,centers,arc,z,flatGap2))continue;
            if(plan.LocalEdge!=INDEX_NONE)return false;
            plan.LocalEdge=i;plan.Edge=arc;
        }
        const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(plan.LocalEdge==INDEX_NONE||after.Status!=EAnalyticNumericStatus::Complete||
            after.ControlFingerprint()!=numeric.ControlFingerprint())return false;
        plan.Initial=&initial;plan.Partition=&partition;plan.Rounded=rounded;plan.Numeric=numeric.ControlFingerprint();
        out=plan;return true;
    }
    static bool BoundSource(const FReferenceSphereStrictSeparation& initial,const C::FReferenceCandidateSet& partition)
    {
        const auto& geometry=initial.GetGeometry();
        return geometry&&geometry->HasValidAliases()&&geometry->GetSelectedUnion()==partition.GetScene()&&
            initial.GetModel().IsKnown()&&C::KinematicCenterDetail::Bits(initial.GetSeconds())==0;
    }
public:
    static bool BuildReference(const FReferenceSphereStrictSeparation& initial,const C::FReferenceCandidateSet& partition,
        C::FDistanceBudget& budget,FReferenceSphereC2Prefix& out)
    {
        namespace A=::AnalyticCertified;
        constexpr int32 allowance=512;
        const auto& model=initial.GetModel();
        const auto& path=partition.GetPathForFixedSource();
        if(budget.Remaining-budget.ConsumptionFloor<allowance||
            !BoundSource(initial,partition)||!SamePath(path,model.GetReferenceCurve())||
            A::Bits(partition.GetRadiusForFixedSource())!=A::Bits(initial.GetCapturedInput().SupportRadius))return false;
        C::FScopedDistanceBudgetFloor bounded(budget,budget.Remaining-allowance);
        return bounded.IsValid()&&Build(initial,partition,C::CenterDetail::Hull(path),false,budget,out);
    }
    static bool BuildConsumed(const FReferenceSphereStrictSeparation& initial,
        const PreparedExecution::FSelectedWholeHullPartition& rounded,C::FDistanceBudget& budget,FReferenceSphereC2Prefix& out)
    {
        namespace A=::AnalyticCertified;
        constexpr int32 allowance=512;
        if(budget.Remaining-budget.ConsumptionFloor<allowance||
            !rounded.UsesGeometry(initial.GetGeometry())||!BoundSource(initial,*rounded.Partition))return false;
        const auto& v=initial.GetCapturedInput();
        const auto& model=initial.GetModel();
        const auto& partition=*rounded.Partition;
        const auto& hullPath=partition.GetPathForFixedSource();
        if(double(v.QueryRadius)<v.SupportRadius||A::Bits(partition.GetRadiusForFixedSource())!=A::Bits(double(v.QueryRadius))||
            hullPath.Degree!=0||hullPath.ControlPoints.Num()!=1||
            !SamePoint(hullPath.ControlPoints[0],C::BoundConsumedCenterBetweenFloatSeconds(model,0.f,initial.GetHorizon())))return false;
        C::FScopedDistanceBudgetFloor bounded(budget,budget.Remaining-allowance);
        return bounded.IsValid()&&Build(initial,partition,hullPath.ControlPoints[0],true,budget,out);
    }
    int32 GetLocalEdge() const{return LocalEdge;}
    bool IsLive() const
    {
        const auto current=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        return Initial&&Partition&&Partition->GetSurfacesForFixedSource().IsValidIndex(LocalEdge)&&
            current.Status==EAnalyticNumericStatus::Complete&&current.ControlFingerprint()==Numeric;
    }
    bool ProveReference(double end,C::FDistanceBudget& budget) const
    {
        namespace A=::AnalyticCertified;
        if(!IsLive()||Rounded||!A::Finite(end)||end<=0.||end>1.)return false;
        const auto& path=Partition->GetPathForFixedSource();
        const auto endpoint=C::CenterDetail::Evaluate(path,I::Exact(end));
        const auto centers=C::CenterDetail::Evaluate(path,I{0.,end,true});
        const auto seconds=A::Multiply(I::Exact(end),I::Exact(double(Initial->GetHorizon())));
        return C::Detail::ValidPoint(endpoint)&&C::Detail::ValidPoint(centers)&&A::Valid(seconds)&&
            ClearEdgePrefix(Partition->GetSurfacesForFixedSource()[LocalEdge],centers,endpoint,endpoint,FVector2d::ZeroVector,
                Edge,Initial->GetCapturedInput(),seconds,budget)&&IsLive();
    }
    // A separate theorem at the actual compiled SBox point. A prior
    // consumed-prefix result supplies neither the center nor fresh admission.
    bool ProveFreshActualPoint(const FPreparedGeometrySelectionPtr& geometry,const FStaticSphereInitialValues& captured,
        FVector3d localOffset,FVector3d localCOM,float horizon,float end,const FVector3d& actual,const P& originalBound,
        C::FDistanceBudget& budget) const
    {
#if !UE_BUILD_SHIPPING
        if(PostSeamFresh)
            return C::KinematicCenterDetail::Bits(end)==PostSeamFreshEndBits&&
                PostSeamFresh->Fresh(geometry,captured,localOffset,localCOM,horizon,end,actual,originalBound,budget);
        if(VariableFreshRc)
        {
            const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
            return Initial&&Partition&&C::KinematicCenterDetail::Bits(end)==VariableFreshEndBits&&
                numeric.Status==EAnalyticNumericStatus::Complete&&numeric.ControlFingerprint()==Numeric&&
                VariableC2RoundedPrefix::ProveFresh(*Initial,*Partition,*VariableFreshRc,geometry,captured,
                    localOffset,localCOM,horizon,end,actual,originalBound,budget,
                    VariableFreshLaterUV.IsSet()?&VariableFreshLaterUV.GetValue():nullptr,
                    VariableFreshBroadUV.IsSet()?&VariableFreshBroadUV.GetValue():nullptr);
        }
        if(FixedFreshRc)
        {
            const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
            return Initial&&Partition&&C::KinematicCenterDetail::Bits(end)==FixedFreshEndBits&&
                numeric.Status==EAnalyticNumericStatus::Complete&&numeric.ControlFingerprint()==Numeric&&
                FixedC2RoundedPrefix::ProveFresh(*Initial,*Partition,*FixedFreshRc,geometry,captured,
                    localOffset,localCOM,horizon,end,actual,originalBound,budget);
        }
#endif
        namespace A=::AnalyticCertified;
        constexpr int32 allowance=512;
        if(!IsLive()||!Rounded||Initial->GetGeometry()!=geometry||!BoundSource(*Initial,*Partition)||
            C::KinematicCenterDetail::Bits(horizon)!=C::KinematicCenterDetail::Bits(Initial->GetHorizon())||
            !FMath::IsFinite(end)||end<=0.f||end>horizon||budget.Remaining-budget.ConsumptionFloor<allowance)return false;
        const auto& expected=Initial->GetCapturedInput();
        const auto same=[](const FVector3d& x,const FVector3d& y)
        {
            for(int32 d=0;d<3;++d)if(A::Bits(x[d])!=A::Bits(y[d]))return false;
            return true;
        };
        const auto floatBits=C::KinematicCenterDetail::Bits;
        if(!same(localOffset,Initial->GetLocalOffset())||!same(localCOM,Initial->GetLocalCOM())||
            !same(captured.State.Location,expected.State.Location)||!same(captured.State.Velocity,expected.State.Velocity)||
            !same(captured.State.Acceleration,expected.State.Acceleration)||A::Bits(actual.Z)!=A::Bits(expected.State.Location.Z)||
            A::Bits(captured.SupportRadius)!=A::Bits(expected.SupportRadius)||floatBits(captured.QueryRadius)!=floatBits(expected.QueryRadius)||
            floatBits(captured.PhysicalRadius)!=floatBits(expected.PhysicalRadius)||floatBits(captured.StaticContactOffset)!=floatBits(expected.StaticContactOffset)||
            floatBits(captured.ActualMargin)!=floatBits(expected.ActualMargin))return false;
        const auto rounded=C::BoundConsumedCenterAtFloatSeconds(Initial->GetModel(),end);
        const auto& hull=Partition->GetPathForFixedSource();
        if(!SamePoint(originalBound,rounded)||!C::ReferenceSphereDetail::Contains(rounded,actual)||
            hull.Degree!=0||hull.ControlPoints.Num()!=1||!C::ReferenceSphereDetail::Contains(hull.ControlPoints[0],actual))return false;
        C::FScopedDistanceBudgetFloor bounded(budget,budget.Remaining-allowance);
        FReferenceSphereC2Prefix singleton;
        const auto point=C::Detail::Point(actual);
        if(!bounded.IsValid()||!Build(*Initial,*Partition,point,true,budget,singleton)||
            singleton.GetLocalEdge()!=LocalEdge)return false;
        return ClearEdgePrefix(Partition->GetSurfacesForFixedSource()[LocalEdge],point,point,point,FVector2d::ZeroVector,
            singleton.Edge,captured,I::Exact(double(end)),budget)&&singleton.IsLive();
    }
    bool ProveConsumed(float end,C::FDistanceBudget& budget) const
    {
        namespace A=::AnalyticCertified;
        if(!IsLive()||!Rounded||!FMath::IsFinite(end)||end<=0.f||end>Initial->GetHorizon())return false;
        const auto& v=Initial->GetCapturedInput();
        const auto& model=Initial->GetModel();
        const auto seconds=I::Exact(double(end));
        I normalized;
        if(!C::ContactDetail::ProjectionDividePositive(seconds,I::Exact(double(Initial->GetHorizon())),normalized))return false;
        normalized.Upper=FMath::Min(1.,normalized.Upper);
        const auto ref=C::CenterDetail::Evaluate(model.GetReferenceCurve(),normalized);
        if(!C::Detail::ValidPoint(ref))return false;
        auto endpoint=ref;
        FVector2d error=FVector2d::ZeroVector;
        for(int32 d=0;d<2;++d)
        {
            const auto drift=Drift(v,d,seconds);
            if(!A::Valid(drift)||drift.Upper<0.)return false;
            endpoint.Axis[d]=A::Add(endpoint.Axis[d],I{-drift.Upper,drift.Upper,true});
            error[d]=drift.Upper;
        }
        const auto centers=C::BoundConsumedCenterBetweenFloatSeconds(model,0.f,end);
        const auto referenceCenters=C::CenterDetail::Evaluate(model.GetReferenceCurve(),I{0.,normalized.Upper,true});
        if(!C::Detail::ValidPoint(endpoint)||!C::Detail::ValidPoint(centers)||!C::Detail::ValidPoint(referenceCenters))return false;
        const auto both=C::TimeDetail::Hull(centers,referenceCenters);
        return ClearEdgePrefix(Partition->GetSurfacesForFixedSource()[LocalEdge],both,endpoint,ref,error,Edge,v,seconds,budget)&&IsLive();
    }
};

// Values only. The caller must use the complete original Rs partition in the
// retained time search. An inside sample proves existence at B, never that
// the provider owns first contact, that [0,B) is clear, or that native may move.
class FReferenceSphereC2SignedEndpoint final
{
    friend class FReferenceSphereC2ContactPlan;
    friend class FReferenceRepeatedSphereFloorPlan;
    using I=C::Interval;
    using Result=C::TimeDetail::FSignedTimeInterval;
    using Decision=C::TimeDetail::ESignedTimeDecision;

    static double Middle(I a){return .5*a.Lower+.5*a.Upper;}
    static bool InteriorUV(FVector2d uv)
    {
        return AnalyticCertified::Finite(uv.X)&&AnalyticCertified::Finite(uv.Y)&&
            uv.X>0.&&uv.X<1.&&uv.Y>0.&&uv.Y<=.125;
    }

    static Result EvaluateBoundSurface(const C::FIntervalTensorSurface& surface,
        const C::FIntervalBezierCenterPath& path,double height,double radius,int32 localProvider,double time,
        C::FDistanceBudget& budget)
    {
        namespace A=::AnalyticCertified;
        const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(numeric.Status!=EAnalyticNumericStatus::Complete||!A::Finite(time)||time<=0.||time>1.||
            budget.Remaining-budget.ConsumptionFloor<10||!budget.Consume())return {};
        auto center=C::CenterDetail::Evaluate(path,I::Exact(time));
        if(!C::Detail::ValidPoint(center))return {};
        // The sealed zero-offset model has exact constant height, as both
        // captured coefficients are bit-zero. Do not widen it at every lerp.
        center.Axis[2]=I::Exact(height);
        const double x=Middle(center.Axis[0]),y=Middle(center.Axis[1]);
        const auto& edgeFirst=surface.ControlPoints[0];
        const auto& edgeLast=surface.ControlPoints[30];
        const double dydu=Middle(edgeLast.Axis[1])-Middle(edgeFirst.Axis[1]);
        const double dxdu=Middle(edgeLast.Axis[0])-Middle(edgeFirst.Axis[0]);
        if(!A::Finite(dydu)||dydu==0.||!A::Finite(dxdu))return {};

        // The line and local tangent only propose a fixed original UV.
        // No exact Y-affinity, root, monotonicity or separation is inferred.
        FVector2d uv(C::ProjectedDistanceDetail::ChooseU(surface,center,1),0.);
        C::FIntervalPoint edge,tangent;
        constexpr double step=1./1024.;
        if(!budget.Consume()||!C::ProjectedDistanceDetail::EvaluateFixedUV(surface,uv,edge)||
            !budget.Consume()||!C::ProjectedDistanceDetail::EvaluateFixedUV(surface,FVector2d(uv.X,step),tangent))return {};
        const double dxdv=(Middle(tangent.Axis[0])-Middle(edge.Axis[0]))/step;
        if(!A::Finite(dxdv)||dxdv<=0.)return {};
        uv.Y=(x-Middle(edge.Axis[0]))/dxdv;

        const auto baseGap=A::Subtract(I::Exact(height),I::Exact(radius));
        const auto sum=A::Add(I::Exact(height),I::Exact(radius));
        for(int32 attempt=0;attempt<5&&InteriorUV(uv);++attempt)
        {
            C::FIntervalPoint point;
            if(!budget.Consume()||!C::ProjectedDistanceDetail::EvaluateFixedUV(surface,uv,point))return {};
            const auto dx=A::Subtract(center.Axis[0],point.Axis[0]);
            const auto dy=A::Subtract(center.Axis[1],point.Axis[1]);
            // Subtract Rs before height. Forming height-Zsurface first would
            // discard the signed cubic residual at the world height scale.
            const auto vertical=A::Multiply(A::Subtract(baseGap,point.Axis[2]),A::Subtract(sum,point.Axis[2]));
            const auto signedGap=A::Add(A::Add(A::Square(dx),A::Square(dy)),vertical);
            if(A::Valid(signedGap)&&signedGap.Upper<0.)
            {
                const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
                if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=numeric.ControlFingerprint())return {};
                return {Decision::Inside,localProvider,uv,point,signedGap};
            }

            const double du=(y-Middle(point.Axis[1]))/dydu;
            uv.X+=du;
            uv.Y+=(x-Middle(point.Axis[0])-dxdu*du)/dxdv;
        }
        return {};
    }
};

class FReferenceSphereC2ContactPlan final
{
public:
    struct FObservation
    {
        bool Admitted=false,RoundedClear=false;
        uint32 ClearCalls=0,InsideCalls=0;
    };

    struct FVariableTimeObservation
    {
        uint32 Attempts=0,Clear=0;
        bool Later=false,Broad=false,ExtendedEarly=false,HighPrefixOnly=false,SeamRs=false;
        uint32 HighPreClear=0,BroadPreClear=0,LaterPreClear=0;
        uint64 HighPreClearUsed=0,BroadPreClearUsed=0,LaterPreClearUsed=0,SeamPreClearUsed=0;
        uint32 SeamPreClear=0;
        double First=0.,Last=0.,ScaledLower=0.;
#if !UE_BUILD_SHIPPING
        TOptional<VariableC2Time::FNativeProposals> NativeProposals;
        uint32 NativeEndBits=0;
        bool WholeRcPreproved=false;
#endif
    };

#if !UE_BUILD_SHIPPING
    static bool TryReferencePostSeam(const FReferenceSphereStrictSeparation& initial,
        const C::FReferenceCandidateSet& candidates,const C::FReferenceSphereOptions& options,
        C::FDistanceBudget& budget,const PostSeamC2::FProof& proof,C::FTensorSphereContactEnclosure& out,
        C::ContactDetail::FInteriorTensorContraction* spatial,uint32& selectedBits)
    {
        namespace A=::AnalyticCertified;
        using D=C::TimeDetail::ESignedTimeDecision;
        using Result=C::TimeDetail::FSignedTimeInterval;
        if(!proof.IsLive())return false;
        const auto& path=candidates.GetPathForFixedSource();
        const double radius=initial.GetCapturedInput().SupportRadius;
        uint64 optionalUsed=0;
        constexpr uint64 optionalAllowance=1024;
        const auto clear=[&](double a,double b,C::FDistanceBudget& account)
        {
            if(proof.HasSeparateOrder())return proof.NaturalReferenceClear(b)&&account.Consume();
            constexpr int32 allowance=10;
            if(!(a>=0.&&a<=b&&b<=proof.EndLimit())||optionalAllowance-optionalUsed<allowance||
                account.Remaining-account.ConsumptionFloor<allowance)return false;
            const uint64 before=account.Used;
            C::FScopedDistanceBudgetFloor bounded(account,account.Remaining-allowance);
            C::FIntervalPoint centers[3];
            const bool proved=bounded.IsValid()&&VariableC2Time::Restrict(path,a,b,account,centers)&&
                proof.Clear(centers,radius,account);
            optionalUsed+=account.Used-before;
            return proved;
        };
        const auto signedCell=[&](double a,double b,C::FDistanceBudget& account)->Result
        {
            if(clear(a,b,account))return {D::Clear};
            constexpr int32 allowance=12;
            if(!(a>=0.&&a<b&&b<=proof.EndLimit())||optionalAllowance-optionalUsed<allowance||
                account.Remaining-account.ConsumptionFloor<allowance)return {};
            const uint64 before=account.Used;
            C::FScopedDistanceBudgetFloor bounded(account,account.Remaining-allowance);
            const auto center=C::CenterDetail::Evaluate(path,A::Interval::Exact(b));
            FVector2d uv=FVector2d::ZeroVector;
            C::FIntervalPoint point;
            A::Interval gap;
            const bool proved=bounded.IsValid()&&proof.Inside(center,radius,account,uv,point,gap);
            optionalUsed+=account.Used-before;
            if(!proved)return {};
            return {D::Inside,proof.LocalProvider(),uv,point,gap};
        };
        const C::TimeDetail::FSignedTimeQuery::FClearCallback clearBeforeCoarse(clear);
        const C::TimeDetail::FSignedTimeQuery signedQuery(signedCell,&clearBeforeCoarse);
        // The already certified A/B bounds refine the existing heuristic proof
        // target. Response point, normal, spatial goals and consumers are unchanged.
        const double timeGap=proof.HasSeparateOrder()?FMath::Min(options.TimeGap,proof.NaturalTimeGap()):options.TimeGap;
        auto contact=C::BoundTensorSphereFirstContactGeneric(candidates.GetSurfacesForFixedSource(),path,radius,budget,
            timeGap,options.SpatialGap,options.SpatialNodesPerAttempt,options.PointWidth,options.NormalWidth,24,
            nullptr,&C::ContactDetail::ThreadContactCellGeometryMemo(),&signedQuery,spatial);
        if(!proof.IsLive()||contact.Status!=C::ETensorSphereContactStatus::FirstContactEnclosed||
            !(contact.LowerTime>0.&&contact.UpperTime<=proof.EndLimit())||
            (proof.HasSeparateOrder()&&!proof.NaturalReferenceBracket(contact.LowerTime,contact.UpperTime))||contact.Leaves.IsEmpty())
        {out=MoveTemp(contact);return false;}
        for(auto& leaf:contact.Leaves)
        {
            if(!budget.Consume()||leaf.SurfaceIndex!=proof.LocalProvider())return false;
            leaf.SurfaceIndex=candidates.GetCandidateIndices()[leaf.SurfaceIndex];
        }
        if(!budget.Consume())return false;
        const auto selected=C::FloorReferenceTimeToFloatSeconds(initial.GetModel(),contact.LowerTime);
        if(!selected.Known||!proof.TimeWithinCover(selected.Seconds))return false;
        selectedBits=C::KinematicCenterDetail::Bits(selected.Seconds);
        out=MoveTemp(contact);
        return true;
    }

    using FSeamSideProof=TFunctionRef<bool(const FReferenceSphereStrictSeparation&,
        const C::FReferenceCandidateSet&,C::FDistanceBudget&)>;
    using FPreprovedWholeRc=TFunctionRef<bool(const VariableC2Time::FNativeProposals&,C::FDistanceBudget&)>;
    static bool TryReferenceVariableTime(const FReferenceSphereStrictSeparation& initial,
        const C::FReferenceCandidateSet& candidates,const PreparedExecution::FSelectedWholeHullPartition& rounded,
        const C::FReferenceSphereOptions& options,C::FDistanceBudget& budget,C::FTensorSphereContactEnclosure& out,
        FVariableTimeObservation& observed,bool prepareNative,C::ContactDetail::FInteriorTensorContraction* spatial,
        const FSeamSideProof* sideProof,const FPreprovedWholeRc* preproveWhole=nullptr,
        bool allowBroadPreclear=false,bool legacyHighShort=false)
    {
        if(budget.Remaining-budget.ConsumptionFloor<512)return false;
        const bool original=VariableC2Time::Admits(initial,candidates,rounded);
        const bool later=!original&&LaterC2Tube::Admits(initial,candidates,rounded);
        const bool broad=!original&&!later&&BroadC2Tube::Admits(initial,candidates,rounded);
        const bool extendedEarly=!original&&!later&&!broad&&VariableC2Time::AdmitsExtendedEarly(initial,candidates,rounded);
        const bool high=!original&&!later&&!broad&&!extendedEarly&&
            VariableC2Time::AdmitsHighPrefixOnly(initial,candidates,rounded);
        const bool seam=!original&&!later&&!broad&&!extendedEarly&&!high&&sideProof&&
            VariableC2Time::AdmitsSeamRs(initial,candidates,rounded)&&(*sideProof)(initial,candidates,budget);
        if(!original&&!later&&!broad&&!extendedEarly&&!high&&!seam)return false;
        observed.SeamRs=seam;
        observed.Later=later;observed.Broad=broad;observed.ExtendedEarly=extendedEarly;observed.HighPrefixOnly=high;
        const bool endpointProposal=later||broad||extendedEarly||high||seam;
        const auto fingerprint=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
        const auto surfaces=candidates.GetSurfacesForFixedSource();
        VariableC2Time::FChart chart;
        chart.Later=later;chart.Broad=broad||high||seam;chart.SeamRs=seam;
        if(!chart.Build(surfaces[1],budget))return false;
        const auto& path=candidates.GetPathForFixedSource();
        const double radius=candidates.GetRadiusForFixedSource();
        using D=C::TimeDetail::ESignedTimeDecision;
        const auto r2=AnalyticCertified::Square(C::Interval::Exact(radius));
        uint64 optionalUsed=0;
        FVector2d uv;
        C::FIntervalPoint sample;
        double sampledAt=-1.;
        TOptional<FVector2d> firstQuarter,thirdQuarter;
        const auto proveClear=[&](const C::FIntervalPoint (&centers)[3],FVector2d proposal,
            C::FDistanceBudget& account,double& lower)
        {
            if(!VariableC2Time::ProveQuadraticAtUV(chart,centers,proposal,radius,account,lower))return false;
            if(high||seam)
            {
                double rivalLower=0.;
                const bool clear=seam?VariableC2Time::ProveSeamRsRival(candidates,centers,proposal.Y,account,rivalLower):
                    VariableC2Time::ProveHighRival(candidates,centers,proposal.Y,
                        double(initial.GetCapturedInput().QueryRadius),account,rivalLower);
                if(!clear)return false;
                lower=FMath::Min(lower,rivalLower);
            }
            return true;
        };
        const auto proveCell=[&](double a,double b,C::FDistanceBudget& account,bool reuse)->C::TimeDetail::FSignedTimeInterval
        {
            // Same paid 320-unit allowance. A reused UV is a proposal only.
            const uint64 allowance=(high||seam)?448:320;
            const double proposalTime=endpointProposal?b:.5*a+.5*b;
            const int32 need=(reuse?6:40)+((high||seam)?3:0);
            if(allowance-optionalUsed<uint64(need)||account.Remaining-account.ConsumptionFloor<need)
                return {D::NotApplicable};
            ++observed.Attempts;
            const uint64 before=account.Used;
            const auto prove=[&]() -> C::TimeDetail::FSignedTimeInterval
            {
                C::FScopedDistanceBudgetFloor bounded(account,account.Remaining-need);
                if(!bounded.IsValid())return {D::NotApplicable};
                C::FIntervalPoint centers[3];
                if(!VariableC2Time::Restrict(path,a,b,account,centers))return {D::NotApplicable};
                if(seam)
                    for(const auto& center:centers)
                        if(!VariableC2Time::ContainsSeamRsPrefix(center))return {D::NotApplicable};
                if(!reuse)
                {
                    sampledAt=-1.;
                    const auto center=C::CenterDetail::Evaluate(path,C::Interval::Exact(proposalTime));
                    if(!C::Detail::ValidPoint(center)||!VariableC2Time::Propose(chart,center,account,uv)||
                        !VariableC2Time::Sample(*chart.Value,uv,account,sample))return {D::NotApplicable};
                    sampledAt=proposalTime;
                }
                if(!account.Consume())return {D::NotApplicable};
                const auto d=VariableC2Time::Subtract(sample,centers[2]);
                auto gap=C::Interval::Exact(0.);
                for(const auto& x:d.Axis)gap=AnalyticCertified::Add(gap,AnalyticCertified::Square(x));
                gap=AnalyticCertified::Subtract(gap,r2);
                // Strict witness at this node's exact B. It does not claim
                // an earlier clear prefix, a contact owner, or a response.
                if(AnalyticCertified::Valid(gap)&&gap.Upper<0.)
                {
                    double rivalLower=0.;
                    if(seam&&!VariableC2Time::ProveSeamRsRival(candidates,centers,uv.Y,account,rivalLower))
                        return {D::NotApplicable};
                    return {D::Inside,1,uv,sample,gap};
                }
                double lower=0.;
                if(!proveClear(centers,uv,account,lower))return {D::NotApplicable};
                ++observed.Clear;observed.First=a;observed.Last=b;observed.ScaledLower=lower;
                return {D::Clear};
            };
            const auto result=prove();
            optionalUsed+=account.Used-before;
            if(RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=fingerprint)
                return {D::Arithmetic};
            return result;
        };
        const auto signedCell=[&](double a,double b,C::FDistanceBudget& account)->C::TimeDetail::FSignedTimeInterval
        {
            const double width=endpointProposal?1./32.:0x1p-15;
            if(!(a>=0.&&a<b&&b<=1.&&b-a<=width)||
                RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=fingerprint)
                return {D::NotApplicable};
            const double proposalTime=endpointProposal?b:.5*a+.5*b;
            const bool reuse=sampledAt>=0.&&FMath::Abs(proposalTime-sampledAt)<=(endpointProposal?1./64.:0x1p-15);
            const auto first=proveCell(a,b,account,reuse);
            if(endpointProposal&&reuse&&first.Decision==D::NotApplicable&&sampledAt!=proposalTime)
                return proveCell(a,b,account,false);
            return first;
        };
        // Discharge only the whole current cell. Broad and Later admit
        // the full original rival shell over their own convex paths; High
        // proves its finite rival separately in proveClear. The original
        // frontier still owns inside witnesses and bracket selection.
        const auto preclear=[&](double a,double b,C::FDistanceBudget& account)
        {
            constexpr uint64 limit=384;
            const double width=later?1./32.:.25;
            if(!(high||broad||later||seam)||!(a>=0.&&a<b&&b<=1.&&b-a<=width)||
                RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=fingerprint)
                return false;
            const bool reuse=sampledAt>=0.&&FMath::Abs(b-sampledAt)<=1./64.;
            const int32 need=reuse?8:39;
            auto& charged=high?observed.HighPreClearUsed:
                broad?observed.BroadPreClearUsed:later?observed.LaterPreClearUsed:observed.SeamPreClearUsed;
            if(limit-charged<uint64(need)||account.Remaining-account.ConsumptionFloor<need)
                return false;
            ++observed.Attempts;
            const uint64 before=account.Used;
            double lower=0.;
            FVector2d proposal=uv;
            C::FIntervalPoint proposedSample;
            const auto attempt=[&]()
            {
                C::FScopedDistanceBudgetFloor bounded(account,account.Remaining-need);
                if(!bounded.IsValid())return false;
                C::FIntervalPoint centers[3];
                if(!VariableC2Time::Restrict(path,a,b,account,centers))return false;
                if(!reuse&&(!VariableC2Time::Propose(chart,centers[2],account,proposal)||
                    !VariableC2Time::Sample(*chart.Value,proposal,account,proposedSample)))return false;
                if(broad||later||seam)
                    for(const auto& center:centers)
                        if(seam?!VariableC2Time::ContainsSeamRsPrefix(center):
                            broad?!BroadC2Tube::Contains(center):!LaterC2Tube::Contains(center))return false;
                return proveClear(centers,proposal,account,lower);
            };
            const bool clear=attempt();
            charged+=account.Used-before;
            if(!clear||RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).
                ControlFingerprint()!=fingerprint)return false;
            if(!reuse)
            {
                // Reuse only the fixed UV and its exact original sample.
                // The next time interval must prove all of its own signs.
                uv=proposal;sample=proposedSample;sampledAt=b;
                // Only proposals from the already-paid, exact normalized
                // High callback. Whole native proof independently rechecks.
                if(high&&b==.25)firstQuarter=proposal;
                if(high&&b==.75)thirdQuarter=proposal;
            }
            if(high)++observed.HighPreClear;
            else if(broad)++observed.BroadPreClear;
            else if(later)++observed.LaterPreClear;
            else ++observed.SeamPreClear;
            ++observed.Clear;
            observed.First=a;observed.Last=b;observed.ScaledLower=lower;
            return true;
        };
        // A completed no-contact result requires independent original reference
        // quarters and the full native/literal Rc proof on the same account.
        // The failed optional attempt never supplies a signed search witness.
        // Four signed cells, two native UV proposals and the same old
        // full-Rc proof allowance. Only High uses the additional quarter UVs.
        const int32 wholeAllowance=4*39+64+(high?96:64);
        if((high||(broad&&allowBroadPreclear))&&prepareNative&&preproveWhole&&
            budget.Remaining-budget.ConsumptionFloor>=wholeAllowance+512)
        {
            const auto originalObservation=observed;
            const uint64 began=budget.Used;
            bool clear=false;
            {
                C::FScopedDistanceBudgetFloor bounded(budget,budget.Remaining-wholeAllowance);
                if(bounded.IsValid()&&preclear(0.,.25,budget)&&preclear(.25,.5,budget)&&
                    preclear(.5,.75,budget)&&preclear(.75,1.,budget)&&
                    (!high||(firstQuarter.IsSet()&&thirdQuarter.IsSet())))
                {
                    VariableC2Time::FNativeProposals proposals;
                    proposals.HighPrefixOnly=high;
                    if(high)
                    {
                        proposals.FirstQuarter=firstQuarter.GetValue();
                        proposals.ThirdQuarter=thirdQuarter.GetValue();
                        proposals.HasWholeQuarterProposals=true;
                    }
                    const float end=initial.GetHorizon();
                    const auto half=C::BoundConsumedCenterAtFloatSeconds(initial.GetModel(),.5f*end);
                    const auto tail=C::BoundConsumedCenterAtFloatSeconds(initial.GetModel(),end);
                    if(C::Detail::ValidPoint(half)&&C::Detail::ValidPoint(tail)&&
                        VariableC2Time::Propose(chart,half,budget,proposals.FirstHalf)&&
                        VariableC2Time::Propose(chart,tail,budget,proposals.Tail)&&
                        (*preproveWhole)(proposals,budget)&&
                        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).
                            ControlFingerprint()==fingerprint)
                    {
                        out={};
                        out.Status=C::ETensorSphereContactStatus::NoContactWithProvidedTensors;
                        out.CollisionRadiusBits=AnalyticCertified::Bits(radius);
                        out.HasCallAccounting=true;
                        out.CallInitialUsed=began;
                        out.TimeWorkUsed=high?observed.HighPreClearUsed:observed.BroadPreClearUsed;
                        out.WorkSite=high&&legacyHighShort?TEXT("signed_high_short_whole"):TEXT("signed_high_broad_whole");
                        observed.NativeEndBits=C::KinematicCenterDetail::Bits(end);
                        observed.NativeProposals=proposals;
                        observed.WholeRcPreproved=true;
                        clear=true;
                    }
                }
            }
            if(high&&legacyHighShort)
            {
                UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2HighShortPreclear] EndBits=%08x Clear=%d Cells=%u Used=%llu"),
                    C::KinematicCenterDetail::Bits(initial.GetHorizon()),int32(clear),observed.HighPreClear,budget.Used-began);
            }
            else
            {
                UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2HighBroadPreclear] EndBits=%08x High=%d Clear=%d Cells=%u Used=%llu"),
                    C::KinematicCenterDetail::Bits(initial.GetHorizon()),int32(high),int32(clear),
                    high?observed.HighPreClear:observed.BroadPreClear,budget.Used-began);
            }
            if(clear)return true;
            observed=originalObservation;
            sampledAt=-1.;
            firstQuarter.Reset();
            thirdQuarter.Reset();
        }
        using FPreClear=TFunctionRef<bool(double,double,C::FDistanceBudget&)>;
        const FPreClear clearBeforeCoarse(preclear);
        const C::TimeDetail::FSignedTimeQuery signedQuery(signedCell,(high||broad||later||seam)?&clearBeforeCoarse:nullptr);
        out=C::BoundTensorSphereFirstContactGeneric(surfaces,path,radius,budget,options.TimeGap,options.SpatialGap,
            options.SpatialNodesPerAttempt,options.PointWidth,options.NormalWidth,24,nullptr,
            &C::ContactDetail::ThreadContactCellGeometryMemo(),&signedQuery,(later||broad||extendedEarly||high||seam)?spatial:nullptr);
        if(out.Status==C::ETensorSphereContactStatus::FirstContactEnclosed)
        {
            const auto global=candidates.GetCandidateIndices();
            if(out.Leaves.IsEmpty()){out={};return true;}
            for(auto& leaf:out.Leaves)
            {
                if(!budget.Consume()||!global.IsValidIndex(leaf.SurfaceIndex)){out={};return true;}
                leaf.SurfaceIndex=global[leaf.SurfaceIndex];
            }
        }
        // Keep the original frontier and its chosen lower-end float. These
        // proposals are local to this proof and can only fail a later shortcut.
        using S=C::ETensorSphereContactStatus;
        if((broad||extendedEarly||high||seam)&&prepareNative&&budget.Remaining-budget.ConsumptionFloor>=64)
        {
            bool contact=out.Status==S::FirstContactEnclosed&&observed.Clear&&
                !out.Leaves.IsEmpty()&&out.Leaves[0].SurfaceIndex==695;
            if(seam)
                for(const auto& leaf:out.Leaves)
                    if(leaf.SurfaceIndex!=695)contact=false;
            const bool clear=(broad||high)&&out.Status==S::NoContactWithProvidedTensors;
            float end=clear?initial.GetHorizon():0.f;
            C::FScopedDistanceBudgetFloor proposalFloor(budget,budget.Remaining-64);
            if(proposalFloor.IsValid()&&contact&&budget.Consume())
            {
                const auto selected=C::FloorReferenceTimeToFloatSeconds(initial.GetModel(),out.LowerTime);
                if(selected.Known)end=selected.Seconds;
            }
            if(proposalFloor.IsValid()&&end>0.f&&end<=initial.GetHorizon())
            {
                VariableC2Time::FNativeProposals proposals;
                proposals.ExtendedEarly=extendedEarly;proposals.HighPrefixOnly=high;
                if(seam){proposals.SeamSideFor=&candidates;proposals.SeamNumericControl=fingerprint;}
                const auto half=C::BoundConsumedCenterAtFloatSeconds(initial.GetModel(),.5f*end);
                const auto tail=C::BoundConsumedCenterAtFloatSeconds(initial.GetModel(),end);
                if(C::Detail::ValidPoint(half)&&C::Detail::ValidPoint(tail)&&
                    VariableC2Time::Propose(chart,half,budget,proposals.FirstHalf)&&
                    VariableC2Time::Propose(chart,tail,budget,proposals.Tail)&&
                    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()==fingerprint)
                {
                    if(high&&clear&&firstQuarter.IsSet()&&thirdQuarter.IsSet())
                    {
                        proposals.FirstQuarter=firstQuarter.GetValue();
                        proposals.ThirdQuarter=thirdQuarter.GetValue();
                        proposals.HasWholeQuarterProposals=true;
                    }
                    observed.NativeProposals=proposals;
                    observed.NativeEndBits=C::KinematicCenterDetail::Bits(end);
                }
            }
        }
        return true;
    }
#endif

    static bool TryReferenceContact(const FReferenceSphereStrictSeparation& initial,
        const C::FReferenceCandidateSet& candidates,const C::FReferenceSphereOptions& options,
        C::FDistanceBudget& budget,C::FTensorSphereContactEnclosure& out,FObservation& observed)
    {
        FReferenceSphereC2Prefix plan;
        if(!FReferenceSphereC2Prefix::BuildReference(initial,candidates,budget,plan))return false;
        const auto local=plan.GetLocalEdge();
        const auto surfaces=candidates.GetSurfacesForFixedSource();
        const auto& path=candidates.GetPathForFixedSource();
        const double radius=candidates.GetRadiusForFixedSource(),height=initial.GetCapturedInput().State.Location.Z;
        using D=C::TimeDetail::ESignedTimeDecision;
        // Admit only an actual crossing candidate. A near-edge corridor on
        // its own must leave an ordinary no-contact search on its old path.
        const auto fullInside=FReferenceSphereC2SignedEndpoint::EvaluateBoundSurface(surfaces[local],path,height,radius,local,1.,budget);
        if(fullInside.Decision!=D::Inside||!plan.IsLive())return false;
        observed.Admitted=true;
        const auto signedAttempt=[&](double,double end,C::FDistanceBudget& account)->C::TimeDetail::FSignedTimeInterval
        {
            if(!plan.IsLive())return {D::Arithmetic};
            if(!account.Consume())return {D::Work};
            if(plan.ProveReference(end,account))
            {
                ++observed.ClearCalls;
                return {D::Clear};
            }
            auto signedPoint=end==1.?fullInside:
                FReferenceSphereC2SignedEndpoint::EvaluateBoundSurface(surfaces[local],path,height,radius,local,end,account);
            if(signedPoint.Decision==D::Inside)
            {
                ++observed.InsideCalls;
                return signedPoint;
            }
            return {D::Indeterminate};
        };
        const C::TimeDetail::FSignedTimeQuery signedQuery(signedAttempt);
        // Keep the original full tensor KKT pass, accuracy goals, derivative
        // normals and memo. This only changes how original time cells are signed.
        out=C::BoundTensorSphereFirstContactGeneric(surfaces,path,radius,budget,options.TimeGap,options.SpatialGap,
            options.SpatialNodesPerAttempt,options.PointWidth,options.NormalWidth,24,nullptr,
            &C::ContactDetail::ThreadContactCellGeometryMemo(),&signedQuery);
        if(!plan.IsLive()){out={};return true;}
        if(out.Status==C::ETensorSphereContactStatus::FirstContactEnclosed)
        {
            const auto global=candidates.GetCandidateIndices();
            if(out.Leaves.IsEmpty()){out={};return true;}
            for(auto& leaf:out.Leaves)
            {
                if(!budget.Consume()||!global.IsValidIndex(leaf.SurfaceIndex)){out={};return true;}
                leaf.SurfaceIndex=global[leaf.SurfaceIndex];
            }
        }
        return true;
    }
};
}
