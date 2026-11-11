#pragma once

// Include after ReferenceSphereC2VariableTime.h. The signed packet fixes the
// three competitor nets and 695 chart. See the offline polytope certificate.
namespace Speed::Analytic::Prepared::LaterC2Tube
{
namespace A=::AnalyticCertified;
using I=A::Interval;
using P=C::FIntervalPoint;

inline bool Contains(const P& point)
{
    if(!C::Detail::ValidPoint(point)||
        point.Axis[0].Lower<4965.75||point.Axis[0].Upper>4968.5||
        point.Axis[1].Lower< -1340.25||point.Axis[1].Upper> -1339.5||
        point.Axis[2].Lower<93.203125||point.Axis[2].Upper>93.3125)return false;
    const auto rise=A::Multiply(I::Exact(1./32.),A::Subtract(point.Axis[0],I::Exact(4966.)));
    const auto height=A::Subtract(point.Axis[2],A::Add(I::Exact(93.1953125),rise));
    return A::Valid(height)&&height.Lower>=0.;
}

inline bool Admits(const FReferenceSphereStrictSeparation& initial,const C::FReferenceCandidateSet& rs,
    const PreparedExecution::FSelectedWholeHullPartition& rc)
{
    P hull;
    if(!VariableC2Time::AdmitsGeometry(initial,rs,rc,hull))return false;
    // Use the actual Bernstein controls, not the independent X/Y/Z extrema.
    // Their convex hull stays in the authenticated Rc-safe polytope.
    for(const auto& center:rs.GetPathForFixedSource().ControlPoints)
        if(!Contains(center))return false;
    return true;
}
}
