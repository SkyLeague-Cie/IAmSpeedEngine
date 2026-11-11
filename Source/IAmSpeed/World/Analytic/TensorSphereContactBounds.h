#pragma once
#include "HAL/IConsoleManager.h"
#include "TensorSphereTimeBounds.h"
#include "Templates/Function.h"
#include "HAL/PlatformTLS.h"
#include "HAL/PlatformTime.h"
#include <exception>
#include <atomic>
#include <limits>

namespace Speed::Analytic::Prepared { class FReferenceSphereGeneralFirstArrivalRs; }

namespace Speed::Analytic::Certified
{
enum class ETensorSphereContactStatus : uint8 { Unknown, NoContactWithProvidedTensors, InitialInsideTube, FirstContactEnclosed };
// Accepted reference-polynomial leaf, not an exact-root/unique-owner claim.
struct FTensorSphereContactLeaf
{
    int32 SurfaceIndex=INDEX_NONE;
    double MinU=0,MaxU=1,MinV=0,MaxV=1;
    FIntervalPoint PointHull,RadialNormalHull,FixedParameterPoint;
    FVector2d FixedParameter=FVector2d::ZeroVector;
};
enum class ETensorSphereContactFailure:uint8 {None,Input,Time,Arithmetic,Work,SpatialDepth,SpatialSplit,NoCandidate,WidthRefinementLimit,ConsumerRefusal,NumericContext};
struct FTensorSphereContactEnclosure
{
    ETensorSphereContactFailure Failure=ETensorSphereContactFailure::None; // Passive; never an admission.
    const TCHAR* WorkSite=TEXT("not_entered"); // Passive only; never a certificate.
    bool HasCallAccounting=false; // Wrapper return{} leaves accounting unavailable.
    uint64 CallInitialUsed=0,TimeWorkUsed=0,SpatialVisitedCells=0,SpatialSplitsU=0,SpatialSplitsV=0;
    int32 TemporalWidthRefinements=0;
    int32 ConsumerWidthRefinements=0; // Diagnostic subset of the SAME global allowance.
    uint8 TimeFailure=0; // Passive exit site only, never a certified time bound.
    uint8 TimeDyadicSite=0;int32 TimeDyadicDepth=-1,TimeAnchorRefinements=0;
    double TimeDyadicA=0,TimeDyadicB=0;
    // Last union-width rejection, observational only; NONE of these fields
    // authenticate a contact or affect subdivision/admission/width targets.
    bool HasWidthDiagnostic=false;
    int32 WidthRefinement=-1,WidthSurface=INDEX_NONE,WidthDepthU=0,WidthDepthV=0;
    double WidthTimeLower=0,WidthTimeUpper=0,WidthPointTarget=0,WidthNormalTarget=0;
    double WidthMinU=0,WidthMaxU=0,WidthMinV=0,WidthMaxV=0;
    FIntervalPoint WidthPointUnion,WidthNormalUnion,WidthLastDu,WidthLastDv;
    Interval WidthLastGu,WidthLastGv;
    ETensorSphereContactStatus Status=ETensorSphereContactStatus::Unknown;
    double LowerTime=0,UpperTime=1;
    FIntervalPoint SurfacePoint,RadialUnitNormal;
    uint64 CollisionRadiusBits=0; // Actual producer radius; status alone never authenticates another query.
    TArray<FTensorSphereContactLeaf> Leaves; // Nonempty only when FirstContactEnclosed.
    // Boxes are certified ONLY when Status==FirstContactEnclosed.
    // At least one first contact exists, and EVERY first closest surface point
    // and sphere radial unit normal is in these boxes. Small boxes do not assert
    // uniqueness, transverse crossing, an admitted World hit or analytic jet C2.
    // Scope is the declared fixed continuous reference curve and full tensors.
    // Enclosures hold for every fixed polynomial represented by the CP boxes.
    // Changing coefficients between evaluations violates this contract.
};
namespace ContactDetail
{
using MinimumDetail::Subtract;
using MinimumDetail::Dot;
inline Interval SquaredNorm(const FIntervalPoint& a)
{
    auto r=Interval::Exact(0);for(const auto& x:a.Axis)r=AnalyticCertified::Add(r,AnalyticCertified::Square(x));return r;
}
using MinimumDetail::NetHull;
using MinimumDetail::DerivativeHull;
using MinimumDetail::CanContainMinimum;

// An interior stationarity proof can exclude a cell, or restrict the set of
// possible stationary points. It cannot select a leaf, time or normal.
class FInteriorTensorContraction final
{
    friend class ::Speed::Analytic::Prepared::FReferenceSphereGeneralFirstArrivalRs;
    const int32 Provider;
    uint32 Attempts=0,Excluded=0,PrunedChildren=0;
    explicit FInteriorTensorContraction(int32 provider):Provider(provider) {}
public:
    FInteriorTensorContraction(const FInteriorTensorContraction&)=delete;
    FInteriorTensorContraction& operator=(const FInteriorTensorContraction&)=delete;
    bool Matches(int32 local) const {return local==Provider;}
    bool Pay(FDistanceBudget& budget)
    {
        // Three fixed-degree midpoint evaluations, three second derivative
        // hulls, and two bounded matrix/vector interval stages.
        constexpr int32 cost=8;
        if(Attempts>=24||budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor||
            budget.Remaining-budget.ConsumptionFloor<cost||
            budget.Used>MAX_uint64-cost)return false;
        for(int32 i=0;i<cost;++i)if(!budget.Consume())return false;
        ++Attempts;return true;
    }
    void NoteExcluded(){++Excluded;}
    void NotePrunedChild(){++PrunedChildren;}
};

struct FInteriorTensorDomain
{
    Interval U,V;
    bool Known() const {return U.Known&&V.Known;}
    bool ContainsPossiblePoint(const Detail::FNode& node) const
    {
        return !Known()||!(node.MaxU<U.Lower||node.MinU>U.Upper||node.MaxV<V.Lower||node.MinV>V.Upper);
    }
};

enum class EInteriorTensorDomain:uint8 {NotApplicable,Proved,Excluded};
inline EInteriorTensorDomain ContractInteriorTensorDomain(
    const Detail::FNode& node,const Detail::FNode& netDu,const Detail::FNode& netDv,
    const FIntervalPoint& center,const FIntervalPoint& offset,const FIntervalPoint& du,const FIntervalPoint& dv,
    FDistanceBudget& budget,FInteriorTensorContraction& proof,FInteriorTensorDomain& domain)
{
    namespace A=::AnalyticCertified;
    using E=EInteriorTensorDomain;
    const int32 depth=node.DepthU+node.DepthV;
    if(node.DegreeU!=5||node.DegreeV!=5||node.DepthU<0||node.DepthU>32||node.DepthV<0||node.DepthV>32||
        !(node.MinU>0&&node.MaxU<1&&node.MinV>0&&node.MaxV<1)||
        (domain.Known()&&depth%4!=0))return E::NotApplicable;
    const double scaleU=A::FromBits(uint64(1023+node.DepthU)<<52);
    const double scaleV=A::FromBits(uint64(1023+node.DepthV)<<52);
    if((node.MaxU-node.MinU)*scaleU!=1.||(node.MaxV-node.MinV)*scaleV!=1.||!proof.Pay(budget))
        return E::NotApplicable;
    const auto second=[](const Detail::FNode& net,bool alongU,double scale,FIntervalPoint& out)
    {
        if(!MinimumDetail::DerivativeHull(net,alongU,out))return false;
        for(auto& axis:out.Axis)axis=A::Multiply(axis,Interval::Exact(scale));
        return Detail::ValidPoint(out);
    };
    FIntervalPoint duu,duv,dvv;
    if(!second(netDu,true,scaleU,duu)||!second(netDu,false,scaleV,duv)||!second(netDv,false,scaleV,dvv))
        return E::NotApplicable;
    const auto positionMid=Detail::Midpoint(node),duMid=Detail::Midpoint(netDu),dvMid=Detail::Midpoint(netDv);
    if(!Detail::ValidPoint(positionMid)||!Detail::ValidPoint(duMid)||!Detail::ValidPoint(dvMid))return E::NotApplicable;
    const auto delta=Subtract(positionMid,center);
    const Interval gradient[2]={Dot(delta,duMid),Dot(delta,dvMid)};
    const auto h00=A::Add(Dot(du,du),Dot(offset,duu));
    const auto h01=A::Add(Dot(du,dv),Dot(offset,duv));
    const auto h11=A::Add(Dot(dv,dv),Dot(offset,dvv));
    if(!A::Valid(gradient[0])||!A::Valid(gradient[1])||!A::Valid(h00)||!A::Valid(h01)||!A::Valid(h11))
        return E::NotApplicable;
    const Interval hessian[2][2]={{h00,h01},{h01,h11}};
    const auto middle=[](Interval x){return .5*x.Lower+.5*x.Upper;};
    const double a=middle(h00),b=middle(h01),c=middle(h11),det=a*c-b*b;
    if(!A::Finite(det)||!(det>0.))return E::NotApplicable;
    // The approximate inverse is an arbitrary fixed preconditioner, not a
    // claimed bound. For each root x, gradient(x)=0 and the mean-value
    // integral Hessian belongs to hessian, so the following enclosure is
    // necessary even when the approximate inverse is imperfect.
    const double inverse[2][2]={{c/det,-b/det},{-b/det,a/det}};
    for(const auto& row:inverse)for(double x:row)if(!A::Finite(x))return E::NotApplicable;
    const Interval box[2]={{node.MinU,node.MaxU,true},{node.MinV,node.MaxV,true}};
    const Interval old[2]={domain.Known()?domain.U:box[0],domain.Known()?domain.V:box[1]};
    const Interval m[2]={Interval::Exact(middle(box[0])),Interval::Exact(middle(box[1]))};
    Interval retained[2];
    for(int32 i=0;i<2;++i)
    {
        const auto product=[&](Interval x,Interval y)
        {return A::Add(A::Multiply(Interval::Exact(inverse[i][0]),x),A::Multiply(Interval::Exact(inverse[i][1]),y));};
        auto candidate=A::Subtract(m[i],product(gradient[0],gradient[1]));
        for(int32 j=0;j<2;++j)
        {
            const auto remainder=A::Subtract(Interval::Exact(i==j?1.:0.),product(hessian[0][j],hessian[1][j]));
            candidate=A::Add(candidate,A::Multiply(remainder,A::Subtract(box[j],m[j])));
        }
        if(!A::Valid(candidate))return E::NotApplicable;
        const double lo=FMath::Max(box[i].Lower,FMath::Max(old[i].Lower,candidate.Lower));
        const double hi=FMath::Min(box[i].Upper,FMath::Min(old[i].Upper,candidate.Upper));
        if(lo>hi)return E::Excluded;
        retained[i]={lo,hi,true};
    }
    const double wide=FMath::Max(box[0].Upper-box[0].Lower,box[1].Upper-box[1].Lower);
    if(FMath::Max(retained[0].Upper-retained[0].Lower,retained[1].Upper-retained[1].Lower)>=.75*wide)
        return E::NotApplicable;
    domain={retained[0],retained[1]};return E::Proved;
}

inline bool Fits(const FIntervalPoint& p,double maximumWidth)
{
    for(const auto& x:p.Axis)
    {
        const auto width=AnalyticCertified::Subtract(Interval::Exact(x.Upper),Interval::Exact(x.Lower));
        if(!AnalyticCertified::Valid(width)||width.Upper>maximumWidth)return false;
    }
    return true;
}
inline bool RadialNormal(const FIntervalPoint& center,const FIntervalPoint& point,double radius,FIntervalPoint& normal)
{
    const auto delta=Subtract(center,point);
    for(int32 d=0;d<3;++d)
    {
        if(!AnalyticCertified::Valid(delta.Axis[d]))return false;
        // Radius is a positive normal binary64 value. Hardware division is RNE,
        // then the same FTZ/DAZ-safe outward step as the interval kernel.
        volatile double lo=delta.Axis[d].Lower/radius,hi=delta.Axis[d].Upper/radius;
        normal.Axis[d].Known=AnalyticCertified::Outward(lo,false,normal.Axis[d].Lower)&&AnalyticCertified::Outward(hi,true,normal.Axis[d].Upper);
    }
    return Detail::ValidPoint(normal);
}

// Insert inside ContactDetail in TensorSphereContactBounds.h, after
// ProjectionDividePositive. Optional domain proof only: no time/contact,
// geometry replacement, root choice, scalar point, or normal is produced.
enum class ECurveStationaryDomain98:uint8 {NotApplicable,Proved,Excluded,Arithmetic,Work};
inline ECurveStationaryDomain98 CurveStationaryDomain98(
    const Detail::FNode& position,const Detail::FNode& originalDv,
    const FIntervalPoint& center,FDistanceBudget& budget,Interval& retainedLocalV)
{
    namespace A=::AnalyticCertified;
    retainedLocalV={};
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    // The original V endpoints have one-sided KKT conditions. They are NOT
    // stationary roots and are deliberately left to the unchanged algorithm.
    if(position.DegreeU!=0||position.DegreeV<1||position.DegreeV>16||
        !(position.MinV>0&&position.MaxV<1))return ECurveStationaryDomain98::NotApplicable;
    if(!budget.Consume())return ECurveStationaryDomain98::Work;
    if(position.Net.Num()!=position.DegreeV+1||originalDv.DegreeU!=0||
        originalDv.DegreeV!=position.DegreeV-1||originalDv.Net.Num()!=position.DegreeV||
        originalDv.MinV!=position.MinV||originalDv.MaxV!=position.MaxV||
        originalDv.DepthV!=position.DepthV||!Detail::ValidPoint(center)||
        !A::Finite(position.MinV)||!A::Finite(position.MaxV)||position.MinV>=position.MaxV)
        return ECurveStationaryDomain98::Arithmetic;
    for(const auto& p:position.Net)if(!Detail::ValidPoint(p))return ECurveStationaryDomain98::Arithmetic;
    for(const auto& p:originalDv.Net)if(!Detail::ValidPoint(p))return ECurveStationaryDomain98::Arithmetic;
    // originalDv is the EXISTING restriction of the original root derivative.
    // Differentiate that derivative net in this cell's local w coordinate.
    // Do not differentiate widened almost-equal POSITION controls, nor pass
    // restricted metadata to OriginalDerivativeNet (which rejects it).
    if(!budget.Consume())return ECurveStationaryDomain98::Work;
    Detail::FNode derivativeInLocalW;derivativeInLocalW.DegreeU=0;
    derivativeInLocalW.DegreeV=FMath::Max(0,originalDv.DegreeV-1);
    for(int32 i=0;i<=derivativeInLocalW.DegreeV;++i)
    {
        if(!budget.Consume())return ECurveStationaryDomain98::Work;
        auto value=Detail::Point(FVector3d::ZeroVector);
        if(originalDv.DegreeV)
            for(int32 d=0;d<3;++d)value.Axis[d]=A::Multiply(Interval::Exact(double(originalDv.DegreeV)),
                A::Subtract(originalDv.Net[i+1].Axis[d],originalDv.Net[i].Axis[d]));
        if(!Detail::ValidPoint(value))return ECurveStationaryDomain98::Arithmetic;
        derivativeInLocalW.Net.Add(value);
    }
    if(!budget.Consume()||!budget.Consume()||!budget.Consume())return ECurveStationaryDomain98::Work;
    FIntervalPoint p,dv,dvw;
    if(!NetHull(position,p)||!NetHull(originalDv,dv)||!NetHull(derivativeInLocalW,dvw)||
        !Detail::ValidPoint(p)||!Detail::ValidPoint(dv)||!Detail::ValidPoint(dvw))
        return ECurveStationaryDomain98::Arithmetic;
    if(!budget.Consume()||!budget.Consume())return ECurveStationaryDomain98::Work;
    const auto pm=Detail::Midpoint(position),dvm=Detail::Midpoint(originalDv);
    if(!Detail::ValidPoint(pm)||!Detail::ValidPoint(dvm))return ECurveStationaryDomain98::Arithmetic;
    if(!budget.Consume())return ECurveStationaryDomain98::Work;
    const auto h=A::Subtract(Interval::Exact(position.MaxV),Interval::Exact(position.MinV));
    // g(w)=(S(w)-C).D_originalV(w), w in[0,1], h=MaxV-MinV.
    // g'(w)=h*|D_originalV(w)|^2 +(S(w)-C).dD_originalV/dw.
    // These interval operations include EVERY fixed coefficient realization
    // and EVERY fixed center in the supplied time enclosure. Correlations may
    // widen the result, never justify dropping a candidate.
    const auto g=Dot(Subtract(pm,center),dvm);
    auto derivative=A::Add(A::Multiply(h,SquaredNorm(dv)),Dot(Subtract(p,center),dvw));
    if(!A::Valid(h)||!(h.Lower>0)||!A::Valid(g)||!A::Valid(derivative))
        return ECurveStationaryDomain98::Arithmetic;
    if(!(derivative.Lower>0||derivative.Upper<0))return ECurveStationaryDomain98::NotApplicable;
    auto numerator=g;
    if(derivative.Upper<0)
    {
        numerator={-g.Upper,-g.Lower,true};
        derivative={-derivative.Upper,-derivative.Lower,true};
    }
    if(!budget.Consume())return ECurveStationaryDomain98::Work;
    Interval quotient;
    if(!ProjectionDividePositive(numerator,derivative,quotient))return ECurveStationaryDomain98::Arithmetic;
    const auto necessary=A::Subtract(Interval::Exact(.5),quotient);
    if(!A::Valid(necessary))return ECurveStationaryDomain98::Arithmetic;
    if(necessary.Upper<0||necessary.Lower>1)return ECurveStationaryDomain98::Excluded;
    retainedLocalV={FMath::Max(0.,necessary.Lower),FMath::Min(1.,necessary.Upper),true};
    if(!A::Valid(retainedLocalV))return ECurveStationaryDomain98::Arithmetic;
    return ECurveStationaryDomain98::Proved;
}



}
// Optional technical refinement of an already complete contact enclosure.
// It cannot supply a missing contact, choose one owner or override a refusal.
// Integer-only private observation, enabled by an existing caller's timing
// scope. It supplies no authority/cache/budget state and never writes FENV.
namespace ContactKernelProfile72
{
enum EStage:uint32 {TimeSearch,SpatialPass,RootSetup,AffineLift,StageCount};
struct FCounters {uint64 Calls[StageCount]{},Ticks[StageCount]{},Unwinding[StageCount]{};bool Integrity=true;};
inline FCounters& Counters(){static thread_local FCounters value;return value;}
inline bool& Enabled(){static thread_local bool value=false;return value;}
inline void Add(uint64& target,uint64 delta){if(target>MAX_uint64-delta){Counters().Integrity=false;return;}target+=delta;}
class FEnabledScope final
{
    bool Previous;
public:
    explicit FEnabledScope(bool value):Previous(Enabled()){Enabled()=value;}
    ~FEnabledScope(){Enabled()=Previous;}
    FEnabledScope(const FEnabledScope&)=delete;FEnabledScope& operator=(const FEnabledScope&)=delete;
};
class FScope final
{
    EStage Stage;bool Active;uint64 Start=0;int Uncaught=0;
public:
    explicit FScope(EStage stage):Stage(stage),Active(Enabled())
    {if(Active){Add(Counters().Calls[Stage],1);Uncaught=std::uncaught_exceptions();Start=FPlatformTime::Cycles64();}}
    void Stop() noexcept
    {
        if(!Active)return;Active=false;const uint64 end=FPlatformTime::Cycles64();
        if(end<Start)Counters().Integrity=false;else Add(Counters().Ticks[Stage],end-Start);
        if(std::uncaught_exceptions()>Uncaught)Add(Counters().Unwinding[Stage],1);
    }
    ~FScope(){Stop();}
    FScope(const FScope&)=delete;FScope& operator=(const FScope&)=delete;
};
}


// Versioned opt-in exact reduced-curve solver. No query/geometry is cached.
namespace CurveStationaryContact98
{
inline bool& Enabled(){static thread_local bool value=false;return value;}
class FEnabledScope final
{
    bool Previous;
public:
    explicit FEnabledScope(bool enabled):Previous(Enabled()){Enabled()=enabled;}
    ~FEnabledScope(){Enabled()=Previous;}
    FEnabledScope(const FEnabledScope&)=delete;FEnabledScope& operator=(const FEnabledScope&)=delete;
};
struct FObservation
{
    std::atomic<uint64> Attempts{0},Proved{0},Excluded{0},SkippedSiblings{0},WorkFailures{0};
};
inline FObservation& Observation(){static FObservation value;return value;}
}

enum class EContactEnclosureDecision:uint8 {Accept,Refine,Refuse};
using FContactEnclosureFilter=TFunctionRef<EContactEnclosureDecision(const FTensorSphereContactEnclosure&)>;
// Existence follows from the SAME call's initial strict separation and later
// strict inside proof. At its first contact every closest point satisfies box
// KKT on its original full domain. Keeping all such candidates is essential:
// selecting the later inside provider would lose an earlier touching provider.
inline FTensorSphereContactEnclosure BoundTensorSphereFirstContactGeneric(
    TConstArrayView<FIntervalTensorSurface> surfaces,const FIntervalBezierCenterPath& path,double radius,
    FDistanceBudget& budget,double requestedTimeGap,double requestedSpatialSquaredGap,
    int32 spatialNodesPerAttempt,double requestedPointWidth,double requestedNormalWidth,int32 maximumRefinements=24,
    const FContactEnclosureFilter* filter=nullptr,ContactDetail::FContactCellGeometryMemo* geometryMemo=nullptr,
    const TimeDetail::FSignedTimeQuery* signedTimeQuery=nullptr,
    ContactDetail::FInteriorTensorContraction* interiorContraction=nullptr)
{
    const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    ContactDetail::FContactCellGeometryMemo::FCallScope geometryScope(geometryMemo,numeric.ControlFingerprint());
    FTensorSphereContactEnclosure result;result.CallInitialUsed=budget.Used;result.HasCallAccounting=true;
    const auto failed=[&](ETensorSphereContactFailure reason){result.Failure=reason;return result;};
    const auto radiusBits=AnalyticCertified::Bits(radius);
    if((radiusBits&AnalyticCertified::Sign)||(radiusBits&AnalyticCertified::Magnitude)<AnalyticCertified::MinNormal||
        !AnalyticCertified::Finite(radius)||!TimeDetail::NonnegativeFinite(requestedPointWidth)||
        !TimeDetail::NonnegativeFinite(requestedNormalWidth)||maximumRefinements<=0||maximumRefinements>24)return failed(ETensorSphereContactFailure::Input);
    result.CollisionRadiusBits=radiusBits;
    TUniquePtr<FPlanarFamilyTimeShortcutPlan> planeShortcut;
    const bool admittedQuadraticPlanar=signedTimeQuery&&signedTimeQuery->HasAdmittedQuadraticPlanarFamily();
    if(PlanarFamilyTimeShortcutControl::Enabled()||admittedQuadraticPlanar)
    {
        result.WorkSite=TEXT("planar_family_time_setup");
        const auto build=FPlanarFamilyTimeShortcutPlan::Build(surfaces,path,budget,planeShortcut,true,admittedQuadraticPlanar);
        if(build==EPlanarFamilyBuild::Work)return failed(ETensorSphereContactFailure::Work);
        if(build==EPlanarFamilyBuild::Arithmetic||
            (build==EPlanarFamilyBuild::Ready&&!planeShortcut)||
            (build==EPlanarFamilyBuild::NotApplicable&&planeShortcut))
            return failed(ETensorSphereContactFailure::Arithmetic);
    }
    const uint64 initialTimeBefore=budget.Used;result.WorkSite=TEXT("initial_time");
    ContactKernelProfile72::FScope initialTime72(ContactKernelProfile72::TimeSearch);
    TimeDetail::FRetainedSphereTimeSearch timeSearch(surfaces,path,radius,budget,requestedSpatialSquaredGap,spatialNodesPerAttempt,geometryMemo,false,planeShortcut.Get());
    if(!timeSearch.SetSignedTimeQueryBeforeStart(signedTimeQuery))return failed(ETensorSphereContactFailure::Time);
    if(!timeSearch.SetCertifiedPrefixClearBeforeStart(TimeDetail::CertifiedPrefixClearRuntimeEnabled()))
        return failed(ETensorSphereContactFailure::Time);
    if(!timeSearch.SetDirectInitialTimeFrontierBeforeStart(TimeDetail::DirectInitialTimeFrontierRuntimeEnabled()))
        return failed(ETensorSphereContactFailure::Time);
    const bool enableOffsetSpatialSeed=TimeDetail::OffsetSpatialDomainRuntimeEnabled();
    const bool enableLineSpatialSeed=TimeDetail::CoordinateLineSpatialDomainRuntimeEnabled();
    const bool enableDirectSpatialSeed=enableOffsetSpatialSeed||enableLineSpatialSeed;
    if(!timeSearch.SetOffsetSpatialSeedBeforeStart(enableOffsetSpatialSeed))return failed(ETensorSphereContactFailure::Time);
    const auto inspectTime=[&]()
    {
        result.TimeFailure=uint8(timeSearch.GetFailureForInspection());
        result.TimeDyadicSite=timeSearch.GetDyadicSiteForInspection();
        result.TimeDyadicDepth=timeSearch.GetDyadicDepthForInspection();
        result.TimeDyadicA=timeSearch.GetDyadicAForInspection();result.TimeDyadicB=timeSearch.GetDyadicBForInspection();
        result.TimeAnchorRefinements=timeSearch.GetAnchorRefinementsForInspection();
    };
    double timeGap=requestedTimeGap;
    auto time=timeSearch.Advance(timeGap);initialTime72.Stop();result.TimeWorkUsed+=budget.Used-initialTimeBefore;
    inspectTime();
    result.LowerTime=time.LowerTime;result.UpperTime=time.UpperTime;
    if(time.Status==ETensorSphereTimeStatus::NoIntersectionWithProvidedTensors)
    {result.Status=ETensorSphereContactStatus::NoContactWithProvidedTensors;return result;}
    if(time.Status==ETensorSphereTimeStatus::InitialInsideTube)
    {result.Status=ETensorSphereContactStatus::InitialInsideTube;return result;}
    if(time.Status!=ETensorSphereTimeStatus::FirstContactBracket||!time.InitialSeparationProved)return failed(ETensorSphereContactFailure::Time);
    ContactKernelProfile72::FScope rootSetup72(ContactKernelProfile72::RootSetup);
    const auto r2=AnalyticCertified::Square(Interval::Exact(radius));
    double pointWidth=requestedPointWidth,normalWidth=requestedNormalWidth;
    auto pointTarget=AnalyticCertified::Multiply(Interval::Exact(pointWidth),Interval::Exact(.25));
    auto normalTarget=AnalyticCertified::Multiply(Interval::Exact(normalWidth),Interval::Exact(.25));
    if(!AnalyticCertified::Valid(r2)||!AnalyticCertified::Valid(pointTarget)||!AnalyticCertified::Valid(normalTarget))
        return failed(ETensorSphereContactFailure::Arithmetic);
    struct FWork
    {
        int32 SurfaceIndex;Detail::FNode Node,OriginalDu,OriginalDv;
        ContactDetail::FContactCellGeometryMemo::FCellPtr Geometry;
        ContactDetail::FInteriorTensorDomain InteriorDomain;
        const Detail::FNode& Position() const {return Geometry?Geometry->Get(ContactDetail::FContactCellGeometryMemo::ENet::Position):Node;}
        const Detail::FNode& Du() const {return Geometry?Geometry->Get(ContactDetail::FContactCellGeometryMemo::ENet::Du):OriginalDu;}
        const Detail::FNode& Dv() const {return Geometry?Geometry->Get(ContactDetail::FContactCellGeometryMemo::ENet::Dv):OriginalDv;}
    };
    TArray<FWork> stack;
    for(int32 surfaceIndex=0;surfaceIndex<surfaces.Num();++surfaceIndex)
    {
        const auto& surface=surfaces[surfaceIndex];
        result.WorkSite=TEXT("copy_root_surface");
        if(!budget.Consume())return failed(ETensorSphereContactFailure::Work);
        Detail::FNode node;node.DegreeU=surface.DegreeU;node.DegreeV=surface.DegreeV;
        for(const auto& p:surface.ControlPoints)node.Net.Add(p);
        Detail::FNode originalDu,originalDv;
        result.WorkSite=TEXT("original_derivative_nets");
        if(!budget.Consume()||!budget.Consume())return failed(ETensorSphereContactFailure::Work);
        if(!ContactDetail::OriginalDerivativeNet(node,true,originalDu)||!ContactDetail::OriginalDerivativeNet(node,false,originalDv))
            return failed(ETensorSphereContactFailure::Arithmetic);
        auto geometry=geometryScope.FindRoot(node,originalDu,originalDv);
        if(geometry)stack.Add(FWork{surfaceIndex,{},{},{},MoveTemp(geometry)});
        else stack.Add(FWork{surfaceIndex,MoveTemp(node),MoveTemp(originalDu),MoveTemp(originalDv),{}});
    }
    rootSetup72.Stop();
    // Original geometry is moved aside only after admission; no clearance,
    // contact or caller decision is retained in this fallback stack.
    TArray<FWork> originalRoots;
    FIntervalPoint seededCenters;bool spatialSeedActive=false,spatialOnlyRetried=false,spatialOnlyActive=false;
    Interval previousPointTarget,previousNormalTarget;
    // Only an explicit union-width failure is refinable. Numerical, budget,
    // depth and missing-candidate failures remain terminal and diagnostic.
    for(int32 refinement=0;refinement<maximumRefinements;++refinement)
    {
        ContactKernelProfile72::FScope spatialPass72(ContactKernelProfile72::SpatialPass);
        const Interval t{time.LowerTime,time.UpperTime,true};
        const auto center=CenterDetail::Evaluate(path,t);
        if(!AnalyticCertified::Valid(t)||!Detail::ValidPoint(center))return failed(ETensorSphereContactFailure::Arithmetic);
        if(spatialSeedActive)
        {
            bool contained=true;for(int32 d=0;d<3;++d)
                contained&=center.Axis[d].Lower>=seededCenters.Axis[d].Lower&&center.Axis[d].Upper<=seededCenters.Axis[d].Upper;
            if(!contained)
            {
                // An unexpected wider center box cannot borrow an old domain.
                // Restore the COMPLETE original stack and pay the transfer. Prior
                // work stays charged; no refused search is ever restarted.
                if(!budget.Consume())return failed(ETensorSphereContactFailure::Work);
                stack=MoveTemp(originalRoots);spatialSeedActive=false;
            }
        }
        if(refinement==0&&enableDirectSpatialSeed)
        {
            TimeDetail::FDirectCurveSpatialSeed seed;
            auto status=TimeDetail::EDirectCurveSpatialSeed::NotApplicable;
            if(enableLineSpatialSeed)status=TimeDetail::PrepareCoordinateLineSpatialSeed(surfaces,center,budget,seed);
            if(status==TimeDetail::EDirectCurveSpatialSeed::NotApplicable&&enableOffsetSpatialSeed)
                status=timeSearch.PrepareSpatialSeed(center,seed);
            if(status==TimeDetail::EDirectCurveSpatialSeed::Work)return failed(ETensorSphereContactFailure::Work);
            if(status==TimeDetail::EDirectCurveSpatialSeed::Arithmetic)return failed(ETensorSphereContactFailure::Arithmetic);
            if(status==TimeDetail::EDirectCurveSpatialSeed::Proved)
            {
                if(surfaces.Num()!=1||seed.Cells.IsEmpty()||seed.Cells.Num()>2||!Detail::ValidPoint(seed.CoveredCenters))return failed(ETensorSphereContactFailure::Arithmetic);
                if(!budget.Consume())return failed(ETensorSphereContactFailure::Work);
                originalRoots=MoveTemp(stack);
                TArray<FWork> direct;
                // The builder returns increasing V; LIFO visits the left cell
                // first, exactly as the ordinary spatial split traversal.
                for(int32 i=seed.Cells.Num()-1;i>=0;--i)
                {
                    if(!budget.Consume())return failed(ETensorSphereContactFailure::Work);
                    auto& cell=seed.Cells[i];
                    direct.Add(FWork{0,MoveTemp(cell.Position),MoveTemp(cell.OriginalDu),MoveTemp(cell.OriginalDv),{}});
                }
                stack=MoveTemp(direct);seededCenters=seed.CoveredCenters;spatialSeedActive=true;
            }
        }
        result.WorkSite=TEXT("planar_projection_domains");
        ContactDetail::FPlanarProjectionDomains projection;
        const auto planar=ContactDetail::ProjectedPlanarDomains(surfaces,center,budget,projection,
            PlanarFamilyTimeShortcutControl::Enabled()||admittedQuadraticPlanar);
        if(planar==ContactDetail::EPlanarProjectionProof::Work)return failed(ETensorSphereContactFailure::Work);
        if(planar==ContactDetail::EPlanarProjectionProof::Arithmetic)return failed(ETensorSphereContactFailure::Arithmetic);
        const bool projected=planar==ContactDetail::EPlanarProjectionProof::Proved;
        TArray<FPlanarFamilySpatialProvider> familyProjection;
        if(!projected&&planeShortcut)
        {
            const auto familyStatus=planeShortcut->ProjectSpatialFamilies(surfaces,center,budget,familyProjection);
            if(familyStatus==EPlanarFamilySpatialProjection::Work)return failed(ETensorSphereContactFailure::Work);
            if(familyStatus==EPlanarFamilySpatialProjection::Arithmetic||familyProjection.Num()!=surfaces.Num())
                return failed(ETensorSphereContactFailure::Arithmetic);
        }
        TArray<FTensorSphereContactLeaf> accepted;
        TArray<FWork> acceptedNodes;
        result.SurfacePoint={};result.RadialUnitNormal={};
        bool haveCandidate=false,tooWide=false,consumerRefine=false;
        while(!stack.IsEmpty())
        {
            result.WorkSite=TEXT("spatial_cell");
            if(!budget.Consume())return failed(ETensorSphereContactFailure::Work);
            ++result.SpatialVisitedCells;auto work=stack.Pop(EAllowShrinking::No);const auto& node=work.Position();
            const auto* family=!projected&&!familyProjection.IsEmpty()&&familyProjection[work.SurfaceIndex].Proved?
                &familyProjection[work.SurfaceIndex]:nullptr;
            const auto* uv=projected?&projection.Providers[work.SurfaceIndex]:(family?&family->Domain:nullptr);
            if(uv&&(uv->Outside||(uv->Bounded&&
                (node.MaxU<uv->U.Lower||node.MinU>uv->U.Upper||node.MaxV<uv->V.Lower||node.MinV>uv->V.Upper))))continue;
            FIntervalPoint point,du,dv,normal;
            if(!geometryScope.Hull(work.Geometry,ContactDetail::FContactCellGeometryMemo::ENet::Position,node,point))return failed(ETensorSphereContactFailure::Arithmetic);
            if(projected||family)
            {
                const int32 normalAxis=projected?projection.NormalAxis:family->NormalAxis;
                const auto& foot=projected?projection.Foot:family->Foot;
                bool outside=false;
                for(int32 d=0;d<3;++d)if(d!=normalAxis)
                    outside|=point.Axis[d].Upper<foot.Axis[d].Lower||point.Axis[d].Lower>foot.Axis[d].Upper;
                if(outside)continue;
            }
            const auto offset=ContactDetail::Subtract(point,center);const auto distance=ContactDetail::SquaredNorm(offset);
            if(!AnalyticCertified::Valid(distance))return failed(ETensorSphereContactFailure::Arithmetic);
            if(distance.Lower>r2.Upper||distance.Upper<r2.Lower)continue;
            // These are dS/d(original U,V) restricted to the SAME cell. The
            // omitted local coordinate factors are strictly positive, so KKT
            // signs and original-edge tests are unchanged.
            if(!geometryScope.Hull(work.Geometry,ContactDetail::FContactCellGeometryMemo::ENet::Du,work.Du(),du)||!geometryScope.Hull(work.Geometry,ContactDetail::FContactCellGeometryMemo::ENet::Dv,work.Dv(),dv))
                return failed(ETensorSphereContactFailure::Arithmetic);
            const auto gu=ContactDetail::Dot(offset,du),gv=ContactDetail::Dot(offset,dv);
            if(!AnalyticCertified::Valid(gu)||!AnalyticCertified::Valid(gv))return failed(ETensorSphereContactFailure::Arithmetic);
            if(!ContactDetail::CanContainMinimum(gu,node.MinU,node.MaxU)||!ContactDetail::CanContainMinimum(gv,node.MinV,node.MaxV))continue;
            if(!ContactDetail::RadialNormal(center,point,radius,normal))return failed(ETensorSphereContactFailure::Arithmetic);
            if(ContactDetail::Fits(point,FMath::Max(0.,pointTarget.Lower))&&ContactDetail::Fits(normal,FMath::Max(0.,normalTarget.Lower)))
            {
                if(node.DepthU>=std::numeric_limits<double>::digits||node.DepthV>=std::numeric_limits<double>::digits)return failed(ETensorSphereContactFailure::SpatialDepth);
                // For a reduced curve, a strict signed derivative places any
                // closest point in this cell on the corresponding original edge.
                // Test that exact endpoint before adding the whole cell to the
                // contact enclosure. An artificial split edge is never eligible.
                if(node.DegreeU==0&&node.DegreeV>0)
                {
                    const bool atStart=node.MinV==0&&gv.Lower>0;
                    const bool atEnd=node.MaxV==1&&gv.Upper<0;
                    if((atStart||atEnd)&&budget.Consume())
                    {
                        const auto& endpoint=node.Net[atEnd?node.DegreeV:0];
                        const auto edgeDistance=ContactDetail::SquaredNorm(ContactDetail::Subtract(endpoint,center));
                        if(AnalyticCertified::Valid(edgeDistance)&&edgeDistance.Lower>r2.Upper)continue;
                    }
                }
                const auto sample=geometryScope.Midpoint(work.Geometry,node);
                if(!Detail::ValidPoint(sample))return failed(ETensorSphereContactFailure::Arithmetic);
                result.WorkSite=TEXT("accepted_leaf");
                if(!budget.Consume())return failed(ETensorSphereContactFailure::Work);
                FTensorSphereContactLeaf leaf;leaf.SurfaceIndex=work.SurfaceIndex;
                leaf.MinU=node.MinU;leaf.MaxU=node.MaxU;leaf.MinV=node.MinV;leaf.MaxV=node.MaxV;
                leaf.PointHull=point;leaf.RadialNormalHull=normal;leaf.FixedParameterPoint=sample;
                leaf.FixedParameter=FVector2d((node.MinU+node.MaxU)*.5,(node.MinV+node.MaxV)*.5);
                accepted.Add(MoveTemp(leaf));acceptedNodes.Add(MoveTemp(work));
                if(!haveCandidate){result.SurfacePoint=point;result.RadialUnitNormal=normal;haveCandidate=true;}
                else {result.SurfacePoint=TimeDetail::Hull(result.SurfacePoint,point);result.RadialUnitNormal=TimeDetail::Hull(result.RadialUnitNormal,normal);}
                if(!ContactDetail::Fits(result.SurfacePoint,pointWidth)||!ContactDetail::Fits(result.RadialUnitNormal,normalWidth))
                {
                    result.HasWidthDiagnostic=true;result.WidthRefinement=refinement;
                    result.WidthSurface=accepted.Last().SurfaceIndex;result.WidthDepthU=acceptedNodes.Last().Position().DepthU;result.WidthDepthV=acceptedNodes.Last().Position().DepthV;
                    result.WidthTimeLower=time.LowerTime;result.WidthTimeUpper=time.UpperTime;
                    result.WidthPointTarget=pointWidth;result.WidthNormalTarget=normalWidth;
                    result.WidthMinU=acceptedNodes.Last().Position().MinU;result.WidthMaxU=acceptedNodes.Last().Position().MaxU;result.WidthMinV=acceptedNodes.Last().Position().MinV;result.WidthMaxV=acceptedNodes.Last().Position().MaxV;
                    result.WidthPointUnion=result.SurfacePoint;result.WidthNormalUnion=result.RadialUnitNormal;
                    result.WidthLastDu=du;result.WidthLastDv=dv;result.WidthLastGu=gu;result.WidthLastGv=gv;
                    tooWide=true;break;
                }
                continue;
            }
            if(node.DegreeU==0&&node.DegreeV==0)return failed(ETensorSphereContactFailure::NoCandidate);

            // Exact curve-family dispatch, analogous to the historical
            // specialized nearest-profile path. The fixed polynomial is NOT
            // replaced by its chord. All discarded siblings exclude every
            // stationary point by a universal interval mean-value theorem.
            // Original endpoint KKT cells and all other families stay generic.
            if(CurveStationaryContact98::Enabled()&&node.DegreeU==0&&node.DegreeV>0&&node.DepthV<=12&&node.MinV>0&&node.MaxV<1)
            {
                auto& observed=CurveStationaryContact98::Observation();observed.Attempts.fetch_add(1);
                Interval local;
                const auto proof=ContactDetail::CurveStationaryDomain98(node,work.Dv(),center,budget,local);
                if(proof==ContactDetail::ECurveStationaryDomain98::Work)
                {observed.WorkFailures.fetch_add(1);return failed(ETensorSphereContactFailure::Work);}
                if(proof==ContactDetail::ECurveStationaryDomain98::Arithmetic)return failed(ETensorSphereContactFailure::Arithmetic);
                if(proof==ContactDetail::ECurveStationaryDomain98::Excluded)
                {observed.Excluded.fetch_add(1);continue;}
                if(proof==ContactDetail::ECurveStationaryDomain98::Proved)
                {
                    observed.Proved.fetch_add(1);
                    if(!budget.Consume())return failed(ETensorSphereContactFailure::Work);
                    const auto span=AnalyticCertified::Subtract(Interval::Exact(node.MaxV),Interval::Exact(node.MinV));
                    const auto global=AnalyticCertified::Add(Interval::Exact(node.MinV),AnalyticCertified::Multiply(span,local));
                    if(!AnalyticCertified::Valid(global))return failed(ETensorSphereContactFailure::Arithmetic);
                    int32 jumped=0;
                    while(jumped<12)
                    {
                        const auto& current=work.Position();
                        if(current.DepthV>=52)break;
                        const double middle=.5*(current.MinV+current.MaxV);
                        const bool keepLeft=global.Upper<middle,keepRight=global.Lower>middle;
                        if(!keepLeft&&!keepRight)break; // Boundary equality retains BOTH sides.
                        if(!budget.Consume()||!budget.Consume()||!budget.Consume()||!budget.Consume())
                        {observed.WorkFailures.fetch_add(1);return failed(ETensorSphereContactFailure::Work);}
                        Detail::FNode left,right,duLeft,duRight,dvLeft,dvRight;
                        ContactDetail::FContactCellGeometryMemo::FCellPtr leftGeometry,rightGeometry;
                        if(!geometryScope.SplitChildren(work.Geometry,false,current,work.Du(),work.Dv(),
                            left,duLeft,dvLeft,right,duRight,dvRight,leftGeometry,rightGeometry))
                            return failed(ETensorSphereContactFailure::SpatialSplit);
                        ++result.SpatialSplitsV;++jumped;observed.SkippedSiblings.fetch_add(1);
                        if(leftGeometry&&rightGeometry)
                            work=FWork{work.SurfaceIndex,{},{},{},keepLeft?MoveTemp(leftGeometry):MoveTemp(rightGeometry)};
                        else if(keepLeft)work=FWork{work.SurfaceIndex,MoveTemp(left),MoveTemp(duLeft),MoveTemp(dvLeft),{}};
                        else work=FWork{work.SurfaceIndex,MoveTemp(right),MoveTemp(duRight),MoveTemp(dvRight),{}};
                    }
                    if(jumped){stack.Add(MoveTemp(work));continue;}
                }
            }
            if(interiorContraction&&interiorContraction->Matches(work.SurfaceIndex)&&
                ContactDetail::ContractInteriorTensorDomain(node,work.Du(),work.Dv(),center,offset,du,dv,
                    budget,*interiorContraction,work.InteriorDomain)==ContactDetail::EInteriorTensorDomain::Excluded)
            {interiorContraction->NoteExcluded();continue;}
            // Partition schedule ONLY. Every separation/KKT/point/normal
            // predicate stays unchanged; same original derivative nets follow
            // the SAME chosen restriction as positions. No extrusion claim.
            result.WorkSite=TEXT("physical_split_schedule");
            if(!budget.Consume())return failed(ETensorSphereContactFailure::Work);
            const bool alongU=geometryScope.Direction(work.Geometry,node);
            Detail::FNode left,right,duLeft,duRight,dvLeft,dvRight;
            result.WorkSite=TEXT("position_and_derivative_split");
            // Position and each derivative split are paid before computation.
            if(!budget.Consume()||!budget.Consume()||!budget.Consume())return failed(ETensorSphereContactFailure::Work);
            ContactDetail::FContactCellGeometryMemo::FCellPtr leftGeometry,rightGeometry;
            if(!geometryScope.SplitChildren(work.Geometry,alongU,node,work.Du(),work.Dv(),
                left,duLeft,dvLeft,right,duRight,dvRight,leftGeometry,rightGeometry))
                return failed(ETensorSphereContactFailure::SpatialSplit);
            if(alongU)++result.SpatialSplitsU;else ++result.SpatialSplitsV;
            const auto push=[&](FWork child)
            {
                if(interiorContraction&&work.InteriorDomain.Known())
                {
                    if(!work.InteriorDomain.ContainsPossiblePoint(child.Position()))
                    {interiorContraction->NotePrunedChild();return;}
                    // The original refinement accepts only a nested time
                    // bracket, so inherited exclusions remain universal.
                    child.InteriorDomain=work.InteriorDomain;
                }
                stack.Add(MoveTemp(child));
            };
            if(leftGeometry&&rightGeometry)
            {
                push(FWork{work.SurfaceIndex,{},{},{},MoveTemp(rightGeometry)});
                push(FWork{work.SurfaceIndex,{},{},{},MoveTemp(leftGeometry)});
            }
            else
            {
                push(FWork{work.SurfaceIndex,MoveTemp(right),MoveTemp(duRight),MoveTemp(dvRight),{}});
                push(FWork{work.SurfaceIndex,MoveTemp(left),MoveTemp(duLeft),MoveTemp(dvLeft),{}});
            }
        }
        spatialPass72.Stop();
        if(!tooWide)
        {
            if(!haveCandidate)return failed(ETensorSphereContactFailure::NoCandidate);
            // Keep result Unknown with NO leaves until acceptance. A refused
            // continuation must never expose the earlier provisional contact.
            if(!filter)
            {
                result.Leaves=MoveTemp(accepted);result.Status=ETensorSphereContactStatus::FirstContactEnclosed;
                RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);return result;
            }
            auto provisional=result;provisional.Leaves=MoveTemp(accepted);
            provisional.Status=ETensorSphereContactStatus::FirstContactEnclosed;
            const auto callbackControl=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
            const auto decision=(*filter)(provisional);
            if(RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=callbackControl)
                return failed(ETensorSphereContactFailure::NumericContext);
            if(decision==EContactEnclosureDecision::Accept)
            {RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);return provisional;}
            if(decision!=EContactEnclosureDecision::Refine)return failed(ETensorSphereContactFailure::ConsumerRefusal);
            // SAME root, path, candidate union and work account. Tighten only
            // technical targets; never the native normal quantization step.
            const double nextPoint=pointWidth*.25,nextNormal=normalWidth*.25;
            if(!(nextPoint>0&&nextPoint<pointWidth&&nextNormal>0&&nextNormal<normalWidth))
                return failed(ETensorSphereContactFailure::Arithmetic);
            pointWidth=nextPoint;normalWidth=nextNormal;
            pointTarget=AnalyticCertified::Multiply(Interval::Exact(pointWidth),Interval::Exact(.25));
            normalTarget=AnalyticCertified::Multiply(Interval::Exact(normalWidth),Interval::Exact(.25));
            if(!AnalyticCertified::Valid(pointTarget)||!AnalyticCertified::Valid(normalTarget))
                return failed(ETensorSphereContactFailure::Arithmetic);
            consumerRefine=true;

        }
        // Retain every accepted cell and every not-yet-visited cell. All prior
        // exclusions were universal on the OLD interval. The new time interval
        // is proved to be its subset, so those exclusions remain valid.
        // No distance, KKT, normal or witness value is reused for retained cells.
        for(int32 i=acceptedNodes.Num()-1;i>=0;--i)stack.Add(MoveTemp(acceptedNodes[i]));
        if(refinement+1==maximumRefinements)return failed(ETensorSphereContactFailure::WidthRefinementLimit);
        // The optional pass only tightens local cells for the SAME time
        // bracket. Restore the original local targets if that union is wide.
        if(spatialOnlyActive)
        {
            if(tooWide){pointTarget=previousPointTarget;normalTarget=previousNormalTarget;}
            spatialOnlyActive=false;
        }
        // Candidate hull overhang can dominate the exact contact-center
        // motion. Refine the SAME retained cells once before paying for a
        // narrower time bracket; the global point/normal tolerances do not change.
        if(tooWide&&!spatialOnlyRetried&&signedTimeQuery&&signedTimeQuery->HasClearCallback()&&path.Degree==2)
        {
            static const auto* spatialRetry=IConsoleManager::Get().FindTConsoleVariableDataInt(
                TEXT("p.IAmSpeed.AnalyticWorld.SignedSphereSpatialWidthRetry"));
            if(spatialRetry&&spatialRetry->GetValueOnAnyThread()==1)
            {
                const auto smallerPoint=AnalyticCertified::Multiply(pointTarget,Interval::Exact(.25));
                const auto smallerNormal=AnalyticCertified::Multiply(normalTarget,Interval::Exact(.25));
                const auto normalTravel=AnalyticCertified::Multiply(Interval::Exact(radius),smallerNormal);
                if(AnalyticCertified::Valid(smallerPoint)&&smallerPoint.Lower>0&&
                    AnalyticCertified::Valid(normalTravel)&&normalTravel.Lower>0)
                {
                    const auto headroom=AnalyticCertified::Multiply(Interval::Exact(.25),
                        Interval::Exact(FMath::Min(smallerPoint.Lower,normalTravel.Lower)));
                    // RadialNormal encloses (center-point)/radius. No finer
                    // geometry can remove the admitted center uncertainty.
                    if(AnalyticCertified::Valid(headroom)&&headroom.Lower>0&&
                        ContactDetail::Fits(center,headroom.Lower))
                    {
                        previousPointTarget=pointTarget;previousNormalTarget=normalTarget;
                        pointTarget=smallerPoint;normalTarget=smallerNormal;
                        spatialOnlyRetried=true;spatialOnlyActive=true;
                        continue;
                    }
                }
            }
        }
        result.WorkSite=TEXT("time_refinement_gap");
        const double nextGap=timeGap*.25;
        if(!(nextGap>0&&nextGap<timeGap)||!budget.Consume())return failed(ETensorSphereContactFailure::Work);
        const uint64 nextTimeBefore=budget.Used;result.WorkSite=TEXT("refined_time");
        const auto next=[&](){ContactKernelProfile72::FScope nextTime72(ContactKernelProfile72::TimeSearch);return timeSearch.Advance(nextGap);}();result.TimeWorkUsed+=budget.Used-nextTimeBefore;
        inspectTime();
        if(next.Status!=ETensorSphereTimeStatus::FirstContactBracket||!next.InitialSeparationProved||
            next.LowerTime<time.LowerTime||next.UpperTime>time.UpperTime)return failed(ETensorSphereContactFailure::Time);
        time=next;timeGap=nextGap;++result.TemporalWidthRefinements;
        if(consumerRefine)++result.ConsumerWidthRefinements;
        result.LowerTime=time.LowerTime;result.UpperTime=time.UpperTime;
    }
    return failed(ETensorSphereContactFailure::WidthRefinementLimit);
}

// Optional complete first-event certificate. A direct attempt supplies no
// authority until ALL contact widths and original UV samples are certified.
// Refine continues once through the same generic solver with a reduced global
// allowance and the same already-debited account; it never calls this wrapper.
namespace DirectCurveContact
{
IAMSPEED_API bool RuntimeEnabled();
IAMSPEED_API bool TryContact(TConstArrayView<FIntervalTensorSurface> surfaces,
    const FIntervalBezierCenterPath& path,double radius,FDistanceBudget& budget,
    double timeGap,double spatialGap,int32 spatialNodes,double pointWidth,double normalWidth,
    int32 maximumRefinements,const FContactEnclosureFilter* filter,
    ContactDetail::FContactCellGeometryMemo* geometryMemo,FTensorSphereContactEnclosure& result);
}
inline FTensorSphereContactEnclosure BoundTensorSphereFirstContact(
    TConstArrayView<FIntervalTensorSurface> surfaces,const FIntervalBezierCenterPath& path,double radius,
    FDistanceBudget& budget,double requestedTimeGap,double requestedSpatialSquaredGap,
    int32 spatialNodesPerAttempt,double requestedPointWidth,double requestedNormalWidth,int32 maximumRefinements=24,
    const FContactEnclosureFilter* filter=nullptr,ContactDetail::FContactCellGeometryMemo* geometryMemo=nullptr,
    ContactDetail::FInteriorTensorContraction* interiorContraction=nullptr,
    const TimeDetail::FSignedTimeQuery* fixedReferenceClear=nullptr)
{
    if(!DirectCurveContact::RuntimeEnabled())
        return BoundTensorSphereFirstContactGeneric(surfaces,path,radius,budget,requestedTimeGap,requestedSpatialSquaredGap,
            spatialNodesPerAttempt,requestedPointWidth,requestedNormalWidth,maximumRefinements,filter,geometryMemo,
            fixedReferenceClear,interiorContraction);
    const uint64 initialUsed=budget.Used;FTensorSphereContactEnclosure direct;
    if(DirectCurveContact::TryContact(surfaces,path,radius,budget,requestedTimeGap,requestedSpatialSquaredGap,
        spatialNodesPerAttempt,requestedPointWidth,requestedNormalWidth,maximumRefinements,filter,geometryMemo,direct))return direct;
    auto result=BoundTensorSphereFirstContactGeneric(surfaces,path,radius,budget,requestedTimeGap,requestedSpatialSquaredGap,
        spatialNodesPerAttempt,requestedPointWidth,requestedNormalWidth,maximumRefinements,filter,geometryMemo,
        fixedReferenceClear,interiorContraction);
    if(result.HasCallAccounting)
    {result.CallInitialUsed=initialUsed;result.TimeWorkUsed+=direct.TimeWorkUsed;}
    return result;
}

inline FTensorSphereContactEnclosure BoundTensorSphereFirstContact(
    TConstArrayView<FIntervalTensorSurface> surfaces,const FVector3d& start,const FVector3d& end,double radius,
    FDistanceBudget& budget,double requestedTimeGap,double requestedSpatialSquaredGap,
    int32 spatialNodesPerAttempt,double requestedPointWidth,double requestedNormalWidth,int32 maximumRefinements=24)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    FIntervalBezierCenterPath path;if(!CenterDetail::Linear(start,end,path))return {};
    return BoundTensorSphereFirstContact(surfaces,path,radius,budget,requestedTimeGap,requestedSpatialSquaredGap,
        spatialNodesPerAttempt,requestedPointWidth,requestedNormalWidth,maximumRefinements);
}
inline FTensorSphereContactEnclosure BoundTensorSphereFirstContact(
    TConstArrayView<FTensorBezierSurface> surfaces,const FVector3d& start,const FVector3d& end,double radius,
    FDistanceBudget& budget,double requestedTimeGap,double requestedSpatialSquaredGap,
    int32 spatialNodesPerAttempt,double requestedPointWidth,double requestedNormalWidth,int32 maximumRefinements=24)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    // Initial union validation needs at least provider+root work per net.
    // Reject before global copying when the SAME caller budget cannot pay it.
    if(budget.Remaining<=0||surfaces.Num()>budget.Remaining/2)return {};
    TArray<FIntervalTensorSurface> nets;if(!Detail::PromoteTensors(surfaces,nets))return {};
    return BoundTensorSphereFirstContact(MakeArrayView(nets),start,end,radius,budget,requestedTimeGap,
        requestedSpatialSquaredGap,spatialNodesPerAttempt,requestedPointWidth,requestedNormalWidth,maximumRefinements);
}
}
