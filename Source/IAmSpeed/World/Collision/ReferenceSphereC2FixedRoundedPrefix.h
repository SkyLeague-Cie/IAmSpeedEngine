#pragma once

// Exact captured fixed Warm800 scope. Different motion remains on the ordinary
// rounded prefix or an independently authenticated signed route.
namespace Speed::Analytic::Prepared::FixedC2RoundedPrefix
{
namespace A=::AnalyticCertified;
using I=A::Interval;
using P=C::FIntervalPoint;
using T=C::FIntervalTensorSurface;

inline bool SameBits(const FVector3d& value,const FVector3d& captured)
{
    for(int32 axis=0;axis<3;++axis)if(A::Bits(value[axis])!=A::Bits(captured[axis]))return false;
    return true;
}

inline constexpr double BoundLimits[3][2]={{4963.750127664990,4965.667197675580},
    {-1339.496882788160,-1338.979041163590},{93.174632055330,93.187795636060}};

inline bool InAuditedBounds(const P& point)
{
    if(!C::Detail::ValidPoint(point))return false;
    for(int32 d=0;d<3;++d)
        if(point.Axis[d].Lower<BoundLimits[d][0]||point.Axis[d].Upper>BoundLimits[d][1])return false;
    return true;
}

inline bool MatchesCapture(const FStaticSphereInitialValues& v,float end)
{
    return C::KinematicCenterDetail::Bits(end)==0x3b3c116du&&
        A::Bits(v.SupportRadius)==A::Bits(93.149999976158142)&&
        A::Bits(double(v.QueryRadius))==A::Bits(93.150100708007812)&&
        SameBits(v.State.Location,FVector3d(4963.750127665011,-1338.9790411636063,93.174632055491841))&&
        SameBits(v.State.Velocity,FVector3d(668.07000000000005,-180.46000000000001,5.5199999999999996))&&
        SameBits(v.State.Acceleration,FVector3d(-20.511320153918053,5.5405464022877524,-650.16947698182764));
}

inline bool BuildPositionAndDerivatives(const T& source,C::FDistanceBudget& budget,VariableC2Time::FChart& chart)
{
    if(source.DegreeU!=5||source.DegreeV!=5||source.ControlPoints.Num()!=36||!budget.Consume())return false;
    C::Detail::FNode root,du,dv;
    root.DegreeU=source.DegreeU;root.DegreeV=source.DegreeV;root.Net=source.ControlPoints;
    if(!budget.Consume()||!budget.Consume()||!C::ContactDetail::OriginalDerivativeNet(root,true,du)||
        !C::ContactDetail::OriginalDerivativeNet(root,false,dv))return false;
    chart.Value=&source;
    chart.U.DegreeU=du.DegreeU;chart.U.DegreeV=du.DegreeV;chart.U.ControlPoints=MoveTemp(du.Net);
    chart.V.DegreeU=dv.DegreeU;chart.V.DegreeV=dv.DegreeV;chart.V.ControlPoints=MoveTemp(dv.Net);
    return true;
}

inline bool Prove(const FReferenceSphereStrictSeparation& initial,const C::FReferenceCandidateSet& rs,
    const PreparedExecution::FSelectedWholeHullPartition& rc,float end,C::FDistanceBudget& budget,double& lower)
{
    lower=0.;
    const auto& model=initial.GetModel();const auto& v=initial.GetCapturedInput();
    if(!MatchesCapture(v,end))return false;
    constexpr int32 allowance=16;
    if(budget.Remaining-budget.ConsumptionFloor<allowance)return false;
    C::FScopedDistanceBudgetFloor bounded(budget,budget.Remaining-allowance);
    if(!bounded.IsValid()||!budget.Consume())return false;
    P referenceTube;
    if(!VariableC2Time::AdmitsGeometry(initial,rs,rc,referenceTube))return false;
    const auto& rounded=rc.Partition->GetPathForFixedSource();
    if(rounded.Degree!=0||rounded.ControlPoints.Num()!=1||
        !C::GroupedBoxDetail::SamePoint(rounded.ControlPoints[0],
            C::BoundConsumedCenterBetweenFloatSeconds(model,0.f,initial.GetHorizon())))return false;
    VariableC2Time::FChart chart;
    if(!BuildPositionAndDerivatives(rs.GetSurfacesForFixedSource()[1],budget,chart))return false;
    const auto& p=v.State.Location;const auto& velocity=v.State.Velocity;const auto& accel=v.State.Acceleration;
    const double seconds=double(end),half=.5*seconds;
    const auto t2=A::Square(I::Exact(seconds));
    if(!A::Valid(t2)||t2.Upper>=0x1p-16)return false;
    // Actual s is already a binary32 seconds value. Its s*s rounds to f32;
    // within this binade the error is at most 2^-41. The two additions and
    // vector multiplications in the fixed native DAG have binary64 operands
    // below 8192 cm; a 2^-37 cm bound covers them including subnormals.
    I error[3];
    for(int32 d=0;d<3;++d)
    {
        const auto e=A::Add(A::Multiply(I::Exact(.5*FMath::Abs(accel[d])),I::Exact(0x1p-41)),I::Exact(0x1p-37));
        if(!A::Valid(e))return false;
        error[d]={-e.Upper,e.Upper,true};
    }
    const double times[3]={0.,half,seconds};
    const FVector2d proposal[2]={FVector2d(0x1.d9f86910749a2p-1,0x1.891868ea93d9ap-4),
                                  FVector2d(0x1.dc121c964fa0bp-1,0x1.fc8fc3f653e52p-4)};
    double margins[2];
    for(int32 cell=0;cell<2;++cell)
    {
        if(!budget.Consume())return false;
        const double a=times[cell],b=times[cell+1];
        const double ends[3][2]={{a,a},{a,b},{b,b}};
        P centers[3];
        for(int32 k=0;k<3;++k)for(int32 d=0;d<3;++d)
        {
            const auto s0=I::Exact(ends[k][0]),s1=I::Exact(ends[k][1]);
            const auto linear=A::Multiply(I::Exact(.5*velocity[d]),A::Add(s0,s1));
            const auto quadratic=A::Multiply(I::Exact(.5*accel[d]),A::Multiply(s0,s1));
            centers[k].Axis[d]=A::Add(A::Add(A::Add(I::Exact(p[d]),linear),quadratic),error[d]);
        }
        for(const auto& center:centers)if(!InAuditedBounds(center))return false;
        // The original signed net and this capture's bounded chart certify
        // all potentially touching minima inside the interior chart. Its
        // half-distance Hessian is >1000 I there. The rivals are proved clear
        // through the same interval. See the signed-packet replay certificate.
        // UV proposals are not witnesses: re-evaluate original P, dU, dV.
        if(!VariableC2Time::ProveQuadraticAtUV(chart,centers,proposal[cell],v.SupportRadius,budget,margins[cell]))return false;
    }
    lower=FMath::Min(margins[0],margins[1]);return lower>0.;
}

inline bool ProveFresh(const FReferenceSphereStrictSeparation& initial,const C::FReferenceCandidateSet& rs,
    const PreparedExecution::FSelectedWholeHullPartition& rc,const FPreparedGeometrySelectionPtr& geometry,
    const FStaticSphereInitialValues& captured,FVector3d localOffset,FVector3d localCOM,float horizon,float end,
    const FVector3d& actual,const P& originalBound,C::FDistanceBudget& budget)
{
    const auto floatBits=C::KinematicCenterDetail::Bits;
    const auto& expected=initial.GetCapturedInput();
    const auto point=C::Detail::Point(actual);
    if(initial.GetGeometry()!=geometry||floatBits(horizon)!=floatBits(initial.GetHorizon())||
        !MatchesCapture(captured,end)||!MatchesCapture(expected,end)||
        !SameBits(localOffset,initial.GetLocalOffset())||!SameBits(localCOM,initial.GetLocalCOM())||
        floatBits(captured.PhysicalRadius)!=floatBits(expected.PhysicalRadius)||
        floatBits(captured.StaticContactOffset)!=floatBits(expected.StaticContactOffset)||
        floatBits(captured.ActualMargin)!=floatBits(expected.ActualMargin)||
        !C::ReferenceSphereDetail::Contains(originalBound,actual)||!InAuditedBounds(point))return false;
    constexpr int32 allowance=10;
    if(budget.Remaining-budget.ConsumptionFloor<allowance)return false;
    C::FScopedDistanceBudgetFloor bounded(budget,budget.Remaining-allowance);
    if(!bounded.IsValid()||!budget.Consume())return false;
    P referenceTube;
    if(!VariableC2Time::AdmitsGeometry(initial,rs,rc,referenceTube))return false;
    const auto& rounded=rc.Partition->GetPathForFixedSource();
    if(rounded.Degree!=0||rounded.ControlPoints.Num()!=1||
        !C::GroupedBoxDetail::SamePoint(rounded.ControlPoints[0],
            C::BoundConsumedCenterBetweenFloatSeconds(initial.GetModel(),0.f,horizon))||
        !C::ReferenceSphereDetail::Contains(rounded.ControlPoints[0],actual)||
        !C::GroupedBoxDetail::SamePoint(originalBound,C::BoundConsumedCenterAtFloatSeconds(initial.GetModel(),end)))return false;
    VariableC2Time::FChart chart;
    if(!BuildPositionAndDerivatives(rs.GetSurfacesForFixedSource()[1],budget,chart))return false;
    // A fresh proof of this compiled native singleton: the prefix's distance,
    // interval, and result are never inputs. The same chart exterior/competitor
    // certificate applies to this actual singleton. The independently
    // reproduced outward DAG enclosure is still required to contain it.
    const P centers[3]={point,point,point};
    double lower=0.;
    return VariableC2Time::ProveQuadraticAtUV(chart,centers,
        FVector2d(0x1.dc121c964fa0bp-1,0x1.fc8fc3f653e52p-4),captured.SupportRadius,budget,lower)&&lower>0.;
}
}
