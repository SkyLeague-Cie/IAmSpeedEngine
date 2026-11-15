#pragma once

// Include after ReferenceSphereC2VariableTime.h. The broad signed atlas
// certifies all relevant 695 minima in its chart and every original rival
// at Rc throughout this correlated center polytope.
namespace Speed::Analytic::Prepared::BroadC2Tube
{
namespace A=::AnalyticCertified;
using I=A::Interval;
using P=C::FIntervalPoint;

inline bool Contains(const P& center)
{
    const auto& x=center.Axis[0];
    const auto& y=center.Axis[1];
    const auto& z=center.Axis[2];
    if(!C::Detail::ValidPoint(center)||x.Lower<4967.75||x.Upper>4974.5||
        y.Lower< -1342.5||y.Upper> -1338.5||z.Lower<93.171875||z.Upper>95.5)return false;
    const auto first=A::Subtract(x,I::Exact(4968.125));
    const auto side=A::Subtract(y,A::Add(I::Exact(-1340.1875),A::Multiply(I::Exact(-9./32.),first)));
    const auto rise=A::Subtract(z,A::Add(I::Exact(93.3046875),A::Multiply(I::Exact(21./256.),first)));
    const auto steep=A::Subtract(z,A::Add(I::Exact(93.515625),
        A::Multiply(I::Exact(13./32.),A::Subtract(x,I::Exact(4970.5)))));
    return A::Valid(side)&&A::Valid(rise)&&A::Valid(steep)&&
        side.Lower>=0.&&rise.Lower>=0.&&steep.Lower>=0.;
}

inline bool Admits(const FReferenceSphereStrictSeparation& initial,const C::FReferenceCandidateSet& rs,
    const PreparedExecution::FSelectedWholeHullPartition& rc)
{
    P hull;
    if(!VariableC2Time::AdmitsGeometry(initial,rs,rc,hull,true))return false;
    for(const auto& center:rs.GetPathForFixedSource().ControlPoints)
        if(!Contains(center))return false;
    return true;
}
}
