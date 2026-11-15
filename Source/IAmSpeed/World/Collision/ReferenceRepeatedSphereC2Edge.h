#pragma once

#include "ReferenceSphereC2ContactPlan.h"

namespace Speed::Analytic::Prepared
{
// A geometry theorem for one retained tensor, not eligibility evidence. The
// caller must independently authenticate the World exclusion and discharge
// all other eligible providers before using either signed result.
// ReferenceSphereC2ContactPlan.h must friend this type in FReferenceSphereC2Prefix.
class FReferenceRepeatedSphereC2Edge final
{
    using Shared=FReferenceSphereC2Prefix;
    using I=C::Interval;
    using P=C::FIntervalPoint;
    using T=C::FIntervalTensorSurface;
    using Decision=C::TimeDetail::ESignedTimeDecision;
    const T* Surface=nullptr;
    C::FZeroOffsetKinematicCenterReference Motion;
    FStaticSphereInitialValues Captured;
    float Horizon=0;
    I Lip,SkinSquared,YBand;
    double UFirst=0.,ULast=0.;
    uint64 Numeric=0;

    static bool EvaluateEdge(const T& surface,double u,P& result)
    {
        P work[6];
        for(int32 i=0;i<6;++i)work[i]=surface.ControlPoints[i*6];
        for(int32 count=5;count>0;--count)
            for(int32 i=0;i<count;++i)work[i]=C::ProjectedDistanceDetail::LerpPoint(work[i],work[i+1],I::Exact(u));
        result=work[0];return C::Detail::ValidPoint(result);
    }
    static bool BoundEdgeParameters(const T& surface,I y,C::FDistanceBudget& budget,double& first,double& last)
    {
        namespace A=::AnalyticCertified;
        if(!A::Valid(y)||y.Lower<=surface.ControlPoints[30].Axis[1].Upper||
            y.Upper>=surface.ControlPoints[0].Axis[1].Lower)return false;
        const auto inverse=[&](double goal,bool lower,double& value)
        {
            double a=0.,b=1.;
            for(int32 i=0;i<16;++i)
            {
                const double t=.5*(a+b);
                P p;
                if(!budget.Consume()||!EvaluateEdge(surface,t,p))return false;
                // Each retained endpoint is individually safe even when the
                // opposite comparison encounters coefficient uncertainty.
                if(lower?p.Axis[1].Lower>goal:p.Axis[1].Upper>=goal)a=t;
                else b=t;
            }
            value=lower?a:b;
            return true;
        };
        return inverse(y.Upper,true,first)&&inverse(y.Lower,false,last)&&first<last;
    }
    static I LocalLip(const T& surface,double first,double last,C::FDistanceBudget& budget)
    {
        namespace A=::AnalyticCertified;
        I dx[5],dy[5];
        for(int32 i=0;i<5;++i)
        {
            const auto& a=surface.ControlPoints[i*6];
            const auto& b=surface.ControlPoints[(i+1)*6];
            dx[i]=A::Multiply(I::Exact(5.),A::Subtract(b.Axis[0],a.Axis[0]));
            dy[i]=A::Multiply(I::Exact(5.),A::Subtract(a.Axis[1],b.Axis[1]));
        }
        double maxX=0.,minY=TNumericLimits<double>::Max();
        for(int32 coefficient=0;coefficient<5;++coefficient)
        {
            if(!budget.Consume())return {};
            I x[5],y[5];
            for(int32 i=0;i<5;++i){x[i]=dx[i];y[i]=dy[i];}
            for(int32 level=0;level<4;++level)
            {
                const auto t=I::Exact(level<4-coefficient?first:last);
                const auto u=A::Subtract(I::Exact(1.),t);
                for(int32 i=0;i<4-level;++i)
                {
                    x[i]=A::Add(A::Multiply(u,x[i]),A::Multiply(t,x[i+1]));
                    y[i]=A::Add(A::Multiply(u,y[i]),A::Multiply(t,y[i+1]));
                }
            }
            if(!A::Valid(x[0])||!A::Valid(y[0])||y[0].Lower<=0.)return {};
            maxX=FMath::Max(maxX,Shared::AbsMax(x[0]));minY=FMath::Min(minY,y[0].Lower);
        }
        return Shared::Divide(I::Exact(maxX),I::Exact(minY));
    }
    static I LateralCap(const Shared::FArc& arc,I z,I skin,double cap)
    {
        namespace A=::AnalyticCertified;
        I height[6];
        for(int32 i=0;i<6;++i)
        {
            if(i>0&&arc.ZUpper[i]<arc.ZUpper[i-1])return {};
            height[i]=I::Exact(arc.ZUpper[i]);
        }
        const auto t=I::Exact(cap),u=A::Subtract(I::Exact(1.),t);
        for(int32 count=5;count>0;--count)
            for(int32 i=0;i<count;++i)height[i]=A::Add(A::Multiply(u,height[i]),A::Multiply(t,height[i+1]));
        const auto bound=A::Add(skin,A::Multiply(A::Multiply(I::Exact(2.),z),height[0]));
        if(!A::Valid(bound)||bound.Upper<=0.)return {};
        double upper=0.;
        if(!A::Outward(FMath::Sqrt(bound.Upper),true,upper))return {};
        auto checked=A::Square(I::Exact(upper));
        if(!A::Valid(checked)||checked.Lower<=bound.Upper)
        {
            if(!A::Outward(upper,true,upper))return {};
            checked=A::Square(I::Exact(upper));
        }
        return A::Valid(checked)&&checked.Lower>bound.Upper?I::Exact(upper):I{};
    }
    bool Propose(const P& center,C::FDistanceBudget& budget,double& u,P& point) const
    {
        if(!C::Detail::ValidPoint(center)||!budget.Consume())return false;
        u=FMath::Clamp(C::ProjectedDistanceDetail::ChooseU(*Surface,center,1),UFirst,ULast);
        return EvaluateEdge(*Surface,u,point);
    }
    bool ClearAtCentre(const P& center,C::FDistanceBudget& budget) const
    {
        namespace A=::AnalyticCertified;
        double u=0.;P point;
        if(!C::Detail::ValidPoint(center)||center.Axis[1].Lower<YBand.Lower||
            center.Axis[1].Upper>YBand.Upper||!Propose(center,budget,u,point))return false;
        const auto dx=A::Subtract(point.Axis[0],center.Axis[0]);
        const auto dy=A::Subtract(point.Axis[1],center.Axis[1]);
        if(!A::Valid(dx)||!A::Valid(dy))return false;
        const auto gap=A::Subtract(dx,A::Multiply(Lip,I::Exact(Shared::AbsMax(dy))));
        if(!A::Valid(gap)||gap.Lower<=0.)return false;
        const auto sign=A::Subtract(A::Square(gap),A::Multiply(SkinSquared,A::Add(I::Exact(1.),A::Square(Lip))));
        return A::Valid(sign)&&sign.Lower>0.;
    }
    bool IsLive() const
    {
        const auto current=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        return Surface&&Motion.IsKnown()&&current.Status==EAnalyticNumericStatus::Complete&&current.ControlFingerprint()==Numeric;
    }
public:
    static bool Build(const T& surface,const FStaticSphereInitialValues& captured,float horizon,
        C::FDistanceBudget& budget,FReferenceRepeatedSphereC2Edge& output)
    {
        namespace A=::AnalyticCertified;
        output={};
        const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        const auto& state=captured.State;
        if(numeric.Status!=EAnalyticNumericStatus::Complete||!FMath::IsFinite(horizon)||horizon<=0.f||horizon>1.f||
            !C::CandidateDetail::PositiveRadius(captured.SupportRadius)||
            !(double(captured.PhysicalRadius)<state.Location.Z&&state.Location.Z<captured.SupportRadius)||
            (A::Bits(state.Velocity.Z)&A::Magnitude)!=0||(A::Bits(state.Acceleration.Z)&A::Magnitude)!=0)return false;
        FReferenceRepeatedSphereC2Edge plan;
        if(!C::BuildZeroOffsetKinematicCenterReference(state.Location,state.Velocity,state.Acceleration,horizon,
            FVector3d::ZeroVector,FVector3d::ZeroVector,plan.Motion))return false;
        Shared::FArc arc;
        if(!Shared::BuildArc(surface,budget,arc))return false;
        const auto z=I::Exact(state.Location.Z),rs=I::Exact(captured.SupportRadius);
        plan.SkinSquared=A::Multiply(A::Subtract(rs,z),A::Add(rs,z));
        const auto global=Shared::Divide(I::Exact(arc.XU),I::Exact(arc.YU));
        const auto k=A::Subtract(I::Exact(arc.XDerivative),A::Multiply(global,I::Exact(arc.YV)));
        const auto beta=A::Multiply(A::Multiply(I::Exact(2.),z),I::Exact(arc.HeightFactor));
        const auto residual=A::Subtract(Shared::Divide(A::Square(k),A::Add(I::Exact(1.),A::Square(global))),beta);
        constexpr double cap=1./8192.;
        const auto outer=A::Multiply(residual,A::Square(I::Exact(cap)));
        if(!A::Valid(k)||k.Lower<=0.||!A::Valid(outer)||!A::Valid(plan.SkinSquared)||
            plan.SkinSquared.Lower<=0.||outer.Lower<=plan.SkinSquared.Upper)return false;
        const auto lateral=LateralCap(arc,z,plan.SkinSquared,cap);
        const auto ref=C::CenterDetail::Hull(plan.Motion.GetReferenceCurve());
        const auto consumed=C::BoundConsumedCenterBetweenFloatSeconds(plan.Motion,0.f,horizon);
        if(!A::Valid(lateral)||!C::Detail::ValidPoint(ref)||!C::Detail::ValidPoint(consumed))return false;
        const auto centers=C::TimeDetail::Hull(ref,consumed);
        plan.YBand=A::Add(centers.Axis[1],I{-lateral.Upper,lateral.Upper,true});
        if(!BoundEdgeParameters(surface,plan.YBand,budget,plan.UFirst,plan.ULast))return false;
        plan.Lip=LocalLip(surface,plan.UFirst,plan.ULast,budget);
        const auto need=A::Multiply(beta,A::Add(I::Exact(1.),A::Square(plan.Lip)));
        const auto dx2=A::Square(k);
        const auto t=I{0.,double(horizon),true};
        const auto vx=A::Add(I::Exact(state.Velocity.X),A::Multiply(I::Exact(state.Acceleration.X),t));
        const auto vy=A::Add(I::Exact(state.Velocity.Y),A::Multiply(I::Exact(state.Acceleration.Y),t));
        if(!A::Valid(vx)||!A::Valid(vy)||!A::Valid(need)||!A::Valid(dx2)||dx2.Lower<=need.Upper)return false;
        const auto move=A::Multiply(plan.Lip,I::Exact(Shared::AbsMax(vy)));
        if(!A::Valid(move)||vx.Lower<=move.Upper)return false;
        plan.Surface=&surface;plan.Horizon=horizon;plan.Captured=captured;plan.Numeric=numeric.ControlFingerprint();
        if(!plan.IsLive())return false;
        output=MoveTemp(plan);return true;
    }
    bool MatchesReference(const C::FIntervalBezierCenterPath& path) const
    {return IsLive()&&Shared::SamePath(Motion.GetReferenceCurve(),path);}
    bool ClearReference(double end,C::FDistanceBudget& budget) const
    {
        return IsLive()&&AnalyticCertified::Finite(end)&&end>0.&&end<=1.&&
            ClearAtCentre(C::CenterDetail::Evaluate(Motion.GetReferenceCurve(),I::Exact(end)),budget)&&IsLive();
    }
    bool ClearConsumed(float end,C::FDistanceBudget& budget) const
    {
        namespace A=::AnalyticCertified;
        const auto bits=C::KinematicCenterDetail::Bits(end);
        if(!IsLive()||!FMath::IsFinite(end)||(bits&C::KinematicCenterDetail::FloatSign)||end>Horizon)return false;
        if(bits==0)return ClearFreshPoint(Captured.State.Location,budget);
        if(bits<C::KinematicCenterDetail::FloatMinNormal)return false;
        const auto seconds=I::Exact(double(end));
        auto normalized=Shared::Divide(seconds,I::Exact(double(Horizon)));
        if(!A::Valid(normalized))return false;
        normalized.Upper=FMath::Min(1.,normalized.Upper);
        auto center=C::CenterDetail::Evaluate(Motion.GetReferenceCurve(),normalized);
        if(!C::Detail::ValidPoint(center))return false;
        for(int32 i=0;i<2;++i)
        {
            const auto drift=Shared::Drift(Captured,i,seconds);
            if(!A::Valid(drift)||drift.Upper<0.)return false;
            center.Axis[i]=A::Add(center.Axis[i],I{-drift.Upper,drift.Upper,true});
        }
        return ClearAtCentre(center,budget)&&IsLive();
    }
    bool ClearFreshPoint(FVector3d actual,C::FDistanceBudget& budget) const
    {
        return IsLive()&&AnalyticCertified::Bits(actual.Z)==AnalyticCertified::Bits(Captured.State.Location.Z)&&
            ClearAtCentre(C::Detail::Point(actual),budget)&&IsLive();
    }
    C::TimeDetail::FSignedTimeInterval InsideReferenceEdge(int32 provider,double end,C::FDistanceBudget& budget) const
    {
        namespace A=::AnalyticCertified;
        if(!IsLive()||provider<0||!A::Finite(end)||end<=0.||end>1.)return {};
        const auto center=C::CenterDetail::Evaluate(Motion.GetReferenceCurve(),I::Exact(end));
        double u=0.;P point;
        if(!Propose(center,budget,u,point))return {};
        const auto dx=A::Subtract(point.Axis[0],center.Axis[0]),dy=A::Subtract(point.Axis[1],center.Axis[1]);
        const auto sign=A::Subtract(A::Add(A::Square(dx),A::Square(dy)),SkinSquared);
        if(!A::Valid(sign)||sign.Upper>=0.||!IsLive())return {};
        // Exact zero height is structural for V=0. Keep the original
        // directed X/Y evaluation and unmodified original UV for KKT.
        point.Axis[2]=I::Exact(0.);
        return {Decision::Inside,provider,FVector2d(u,0.),point,sign};
    }
};
}
