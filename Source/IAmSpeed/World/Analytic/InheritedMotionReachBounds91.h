#pragma once
#include "FixedShapeQueryBounds.h"
#include "IAmSpeed/Base/InheritedMotionPolicyValues91.h"
#include "IAmSpeed/World/Collision/PreparedNativeSolidRequest.h"

namespace Speed::Analytic::Certified
{
// Policy is captured configuration, never a universal hard speed theorem.
// Only containment of the EXISTING full volume permits a separation attempt.
enum class EInheritedReach91:uint8 {Invalid,Outside,Near,Cleared,Unknown};
enum class EInheritedReachRoute91:uint8 {SolidSphere,SolidBox,WheelSuspension,WheelSupport,Other,Count};
struct FInheritedReachInput91
{
    FInheritedMotionPolicyValues91 Policy;
    FKinematicState Owner,Body; // Owner.Location is the actual COM.
    bool Available=false;
};
// One IAS-DLL observer, also callable by the SkyLeague concrete service.
// No observation field participates in eligibility or a physical result.
class IAMSPEED_API FInheritedReachObservation91 final
{
    EInheritedReachRoute91 Route;
    FDistanceBudget& Budget;
    uint64 Before=0,Start=0;int Uncaught=0;bool Active=false;
public:
    EInheritedReach91 Result=EInheritedReach91::Invalid;
    FInheritedReachObservation91(EInheritedReachRoute91,FDistanceBudget&);
    ~FInheritedReachObservation91() noexcept;
};
namespace InheritedReachDetail91
{
namespace A=::AnalyticCertified;
inline bool Nonnegative(double x)
{return A::Finite(x)&&(!(A::Bits(x)&A::Sign)||!(A::Bits(x)&A::Magnitude));}
inline bool Finite(const FKinematicState& s)
{
    return !s.Location.ContainsNaN()&&!s.Rotation.ContainsNaN()&&s.Rotation.IsNormalized()&&
        !s.Velocity.ContainsNaN()&&!s.Acceleration.ContainsNaN()&&
        !s.AngularVelocity.ContainsNaN()&&!s.AngularAcceleration.ContainsNaN();
}
inline Interval Abs(Interval x)
{
    if(!A::Valid(x))return {};
    return Interval{0.,FMath::Max(FMath::Abs(x.Lower),FMath::Abs(x.Upper)),true};
}
inline bool Zero(const FVector& x)
{return x.X==0.&&x.Y==0.&&x.Z==0.;}
inline Interval L1Motion(const FVector& v,const FVector& a,double h)
{
    if(Zero(v)&&Zero(a))return Interval::Exact(0);
    auto sum=Interval::Exact(0);
    for(int32 d=0;d<3;++d)
        sum=A::Add(sum,Abs(A::Add(Interval::Exact(v[d]),A::Multiply(Interval::Exact(a[d]),Interval{0,h,true}))));
    return sum;
}
inline bool Contains(const FIntervalPoint& outer,const FIntervalPoint& inner)
{
    if(!Detail::ValidPoint(outer)||!Detail::ValidPoint(inner))return false;
    for(int32 d=0;d<3;++d)
        if(outer.Axis[d].Lower>inner.Axis[d].Lower||outer.Axis[d].Upper<inner.Axis[d].Upper)return false;
    return true;
}
}
// The charge is a fixed 32 bounded proof groups, prepaid BEFORE builders or
// interval arithmetic: 2 fixed-pose builders, 6 PVA axes, 6 COM/support axes,
// 3 envelope axes, 3 containment axes and 12 scalar/identity/group overhead.
// A group is NOT an interval-instruction or elapsed-time unit. The two existing
// shape builders remain bounded (Sphere or 8-vertex Box), no tensor evaluation.
// Partial precharge failure is terminal Unknown; no credit/floor reset.
inline EInheritedReach91 BuildInheritedMotionReach91(const FInheritedReachInput91& input,
    const FWorldQuery& query,const FWorldQuery& physicalPose,double horizon,double extraShell,
    const FIntervalPoint& existingWhole,FDistanceBudget& budget,FIntervalPoint& proposed)
{
    using namespace InheritedReachDetail91;
    proposed={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    // Known unsupported/uncaptured state is classified BEFORE an attempt.
    if(!input.Available||!input.Policy.Captured||!Nonnegative(input.Policy.ParentMaxV)||
        !Nonnegative(input.Policy.ParentMaxOmega)||!Nonnegative(horizon)||!Nonnegative(extraShell)||
        !Nonnegative(query.DomainTolerance)||!Nonnegative(query.InitialOverlapTolerance)||
        !Finite(input.Owner)||!Finite(input.Body)||
        (query.Shape!=EQueryShape::Sphere&&query.Shape!=EQueryShape::Box)||
        query.Shape!=physicalPose.Shape)return EInheritedReach91::Invalid;
    for(int32 group=0;group<32;++group)if(!budget.Consume())return EInheritedReach91::Unknown;
    const auto linear=L1Motion(input.Owner.Velocity,input.Owner.Acceleration,horizon);
    const auto angular=L1Motion(input.Owner.AngularVelocity,input.Owner.AngularAcceleration,horizon);
    if(!A::Valid(linear)||!A::Valid(angular))return EInheritedReach91::Unknown;
    // L1 is a conservative sufficient admissibility test. Outside does NOT
    // mean a Euclidean policy violation; it only retains the original path.
    if(linear.Upper>double(input.Policy.ParentMaxV)||angular.Upper>double(input.Policy.ParentMaxOmega))
        return EInheritedReach91::Outside;
    auto queryPose=query;queryPose.Start=input.Body.Location;queryPose.End=queryPose.Start;
    queryPose.Rotation=input.Body.Rotation;
    auto physical=physicalPose;physical.Start=input.Body.Location;physical.End=physical.Start;
    physical.Rotation=input.Body.Rotation;
    FIntervalPoint qPose,pPose;
    if(!BuildFixedWorldQueryVolumeBounds(queryPose,qPose)||!BuildFixedWorldQueryVolumeBounds(physical,pPose))
        return EInheritedReach91::Unknown;
    auto pose=TimeDetail::Hull(qPose,pPose);
    auto arm=Interval::Exact(0),support=Interval::Exact(0);
    for(int32 d=0;d<3;++d)
    {
        arm=A::Add(arm,Abs(A::Subtract(Interval::Exact(input.Body.Location[d]),Interval::Exact(input.Owner.Location[d]))));
        support=A::Add(support,Abs(A::Subtract(pose.Axis[d],Interval::Exact(input.Body.Location[d]))));
    }
    if(!A::Valid(arm)||!A::Valid(support))return EInheritedReach91::Unknown;
    Interval expansion=Interval::Exact(0);
    if(horizon>0&&(input.Policy.ParentMaxV>0||input.Policy.ParentMaxOmega>0))
    {
        const auto pointCap=A::Add(Interval::Exact(double(input.Policy.ParentMaxV)),
            A::Multiply(Interval::Exact(double(input.Policy.ParentMaxOmega)),A::Add(arm,support)));
        expansion=A::Multiply(pointCap,Interval::Exact(horizon));
    }
    if(!A::Valid(expansion)||expansion.Upper<0)return EInheritedReach91::Unknown;
    auto shell=A::Add(Interval::Exact(query.DomainTolerance),Interval::Exact(query.InitialOverlapTolerance));
    shell=A::Add(shell,Interval::Exact(extraShell));
    if(!A::Valid(shell)||shell.Upper<0)return EInheritedReach91::Unknown;
    const auto total=A::Add(expansion,Interval::Exact(shell.Upper));
    if(!A::Valid(total)||total.Upper<0)return EInheritedReach91::Unknown;
    for(int32 d=0;d<3;++d)proposed.Axis[d]=A::Add(pose.Axis[d],Interval{-total.Upper,total.Upper,true});
    if(!Detail::ValidPoint(proposed)||!Detail::ValidPoint(existingWhole))return EInheritedReach91::Unknown;
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    // This checks actual rounded/reference motion, q chord, footprint and
    // shells as supplied by the original full-volume producer. No v*dt
    // truncation of a suspension probe, and no new clamp/integration.
    return Contains(proposed,existingWhole)?EInheritedReach91::Near:EInheritedReach91::Outside;
}
}
