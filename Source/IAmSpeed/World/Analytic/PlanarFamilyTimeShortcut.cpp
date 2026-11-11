#include "PlanarFamilyTimeShortcut.h"
#include "AnalyticWorldData.h"
#include "DirectionalMotionStrictSeparation.h"
#include "CanonicalAxialPlaneBounds.h"
#include "IAmSpeed/World/Collision/ReferenceAffineContact.h"
#include "GroupedBoxVolumeBounds.h"
#include "TensorSphereTimeBounds.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTime.h"
#include "Misc/ScopedCVar.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "ScalarSphereContactBounds.h"
#include "NativeGroupedBoxVolumeBounds.h"
#endif

namespace
{
TAutoConsoleVariable<int32> CVarPlanarFamilyTimeShortcut(
    TEXT("p.IAmSpeed.AnalyticWorld.PlanarFamilyTimeShortcut"),0,
    TEXT("Default OFF. Exact finite planar-family distance shortcuts before the existing global sphere-time UV minimum. No owner/filter/normal/geometry change."),
    ECVF_Default);
TAutoConsoleVariable<int32> CVarStrongConvexScalarProposal(
    TEXT("p.IAmSpeed.AnalyticWorld.StrongConvexScalarProposal"),0,
    TEXT("Default OFF. Scalar Newton proposes only a fixed original parameter; interval original-net validation and strict fallback remain mandatory."),ECVF_Default);
TAutoConsoleVariable<int32> CVarStrongConvexJointDecision(
    TEXT("p.IAmSpeed.AnalyticWorld.StrongConvexJointDecision"),0,
    TEXT("Default OFF. Joint original de Casteljau value/derivative only for strict scalar-proposal decisions; inconclusive proof falls back to original derivative with all work paid."),ECVF_Default);
Speed::Analytic::Certified::FPlanarFamilyShortcutObservation GPlanarFamilyShortcutObservation;
}

namespace Speed::Analytic::Certified
{
namespace StrongConvexCurveMinimum { void PublishObservation(); }
namespace DirectCurveContact { void PublishObservation(); }
namespace TimeDetail { void PublishCertifiedPrefixClear(); void PublishOffsetSpatialDomain(); }
namespace OffsetCurveTimeHintControl { bool RuntimeEnabled(); void PublishObservation(); }
namespace TimeDetail { static EMonotoneCurveTimeSeed TryOffsetCurveTimeHint(TConstArrayView<FIntervalTensorSurface>,
    const FIntervalBezierCenterPath&,double,FDistanceBudget&,double,double&,double&,FOffsetCurveSpatialProofPtr* capture=nullptr,const FRetainedSphereTimeSearch* owner=nullptr); }
bool PlanarFamilyTimeShortcutControl::RuntimeEnabled()
{
    return CVarPlanarFamilyTimeShortcut.GetValueOnAnyThread()==1;
}

FPlanarFamilyShortcutObservation& PlanarFamilyShortcutObservation()
{
    return GPlanarFamilyShortcutObservation;
}

void PublishPlanarFamilyShortcutObservation()
{
    // Caller must establish the actor drain. These independent atomic loads
    // are only an observational program prefix, never a coherence certificate.
    const auto& observation=PlanarFamilyShortcutObservation();
    const uint64 requests=observation.Requests.load(std::memory_order_relaxed);
    const uint64 clear=observation.Clear.load(std::memory_order_relaxed);
    const uint64 inside=observation.Inside.load(std::memory_order_relaxed);
    const uint64 undecided=observation.Undecided.load(std::memory_order_relaxed);
    const uint64 bounded=observation.Bounded.load(std::memory_order_relaxed);
    const uint64 generic=observation.GenericAttempts.load(std::memory_order_relaxed);
    const uint64 work=observation.Work.load(std::memory_order_relaxed);
    const uint64 arithmetic=observation.Arithmetic.load(std::memory_order_relaxed);
    UE_LOG(LogTemp,Display,TEXT("[PlanarFamilyTimeShortcut] Schema=PlanarFamilyTimeShortcut/v3 Scope=IAmSpeedModuleProgramPrefixAtPreExit SharedGameIAS=1 ObservationSnapshotOnly=1 RequiresExternalDrain=1 NoPASS=1 CountersIndependent=1 RuntimeEnabledNow=%d Requests=%llu Clear=%llu Inside=%llu Undecided=%llu GenericAttempts=%llu Work=%llu Arithmetic=%llu Bounded=%llu"),
        int32(PlanarFamilyTimeShortcutControl::RuntimeEnabled()),requests,clear,inside,undecided,generic,work,arithmetic,bounded);
    StrongConvexCurveMinimum::PublishObservation();
    DirectCurveContact::PublishObservation();
    AffineReferenceDetail::FixedProductWitnessControl::PublishObservation();
    CanonicalPlaneStaticMemo::PublishObservation();
    Speed::Analytic::Prepared::AffineProjectedObservation::PublishObservation();
    GroupedBoxGeometryMemo::PublishObservation();
    ClosedGroupedBoxControl::PublishObservation();
    GroupedBoxProjectedSubdivision::PublishObservation();
    MonotoneCurveTimeControl::PublishObservation();
    CoordinateLineTimeHintControl::PublishObservation();
    ChordEnvelopeTimeHintControl::PublishObservation();
    OffsetCurveTimeHintControl::PublishObservation();
    TimeDetail::PublishCertifiedPrefixClear();
    TimeDetail::PublishOffsetSpatialDomain();
}
}

// DRAFT ONLY. Append to existing PlanarFamilyTimeShortcut.cpp, after its current definitions.
// Definitions live in that existing TU; no new CPP or header include cycle.
namespace Speed::Analytic::Certified
{
namespace StrongConvexCurveMinimum
{
namespace A=::AnalyticCertified;
using FCurve=TArray<FIntervalPoint,TInlineAllocator<16>>;
struct FWork
{
    FDistanceBudget& Budget;uint64 Initial;int32 Limit;
    EResult Failure=EResult::LocalLimit;
    bool Pay()
    {
        if(Limit<=0||Budget.Used<Initial||Budget.Used-Initial>=uint64(Limit))return false;
        if(!Budget.Consume()){Failure=EResult::Work;return false;}
        return true;
    }
};
inline bool DividePositive(Interval a,Interval b,Interval& out)
{
    out={};if(!A::Valid(a)||!A::Valid(b)||!(b.Lower>0))return false;
    bool first=true;double lower=0,upper=0;
    for(double n:{a.Lower,a.Upper})for(double d:{b.Lower,b.Upper})
    {
        volatile double q=n/d;double lo=0,hi=0;
        if(!A::Outward(q,false,lo)||!A::Outward(q,true,hi))return false;
        lower=first?lo:FMath::Min(lower,lo);upper=first?hi:FMath::Max(upper,hi);first=false;
    }
    out={lower,upper,true};return A::Valid(out);
}
inline bool Hull(const FCurve& c,FIntervalPoint& out)
{
    if(c.IsEmpty())return false;out=c[0];
    for(const auto& p:c)
    {
        if(!Detail::ValidPoint(p))return false;
        for(int32 d=0;d<3;++d)
        {out.Axis[d].Lower=FMath::Min(out.Axis[d].Lower,p.Axis[d].Lower);out.Axis[d].Upper=FMath::Max(out.Axis[d].Upper,p.Axis[d].Upper);}
    }
    return Detail::ValidPoint(out);
}
inline bool Derivative(const FCurve& c,FCurve& out)
{
    out.Reset();if(c.Num()<2)return false;
    const auto degree=Interval::Exact(double(c.Num()-1));
    for(int32 i=1;i<c.Num();++i)
    {
        FIntervalPoint p;
        for(int32 d=0;d<3;++d)p.Axis[d]=A::Multiply(degree,A::Subtract(c[i].Axis[d],c[i-1].Axis[d]));
        if(!Detail::ValidPoint(p))return false;out.Add(p);
    }
    return true;
}
inline bool Evaluate(const FCurve& c,double t,FIntervalPoint& out)
{
    if(c.IsEmpty()||!A::Finite(t)||t<0||t>1)return false;
    FCurve work=c;
    for(int32 count=work.Num()-1;count>0;--count)for(int32 i=0;i<count;++i)
        work[i]=ProjectedDistanceDetail::LerpPoint(work[i],work[i+1],Interval::Exact(t));
    out=work[0];return Detail::ValidPoint(out);
}
inline bool Split(const FCurve& c,Interval t,FCurve& left,FCurve& right)
{
    if(c.IsEmpty()||!A::Valid(t)||t.Lower<0||t.Upper>1)return false;
    FCurve work=c;const int32 n=c.Num()-1;left.SetNum(n+1);right.SetNum(n+1);
    left[0]=work[0];right[n]=work[n];
    for(int32 level=1;level<=n;++level)
    {
        for(int32 i=0;i<=n-level;++i)
            work[i]=ProjectedDistanceDetail::LerpPoint(work[i],work[i+1],t);
        left[level]=work[0];right[n-level]=work[n-level];
    }
    for(const auto& p:left)if(!Detail::ValidPoint(p))return false;
    for(const auto& p:right)if(!Detail::ValidPoint(p))return false;
    return true;
}
inline bool Restrict(const FCurve& original,double lo,double hi,FCurve& out)
{
    if(!A::Finite(lo)||!A::Finite(hi)||lo<0||hi>1||lo>hi)return false;
    if(lo==hi)
    {FIntervalPoint p;if(!Evaluate(original,lo,p))return false;out.Init(p,original.Num());return true;}
    FCurve left,right;
    if(lo==0)right=original;
    else if(!Split(original,Interval::Exact(lo),left,right))return false;
    if(hi==1){out=MoveTemp(right);return true;}
    Interval fraction;
    if(!DividePositive(A::Subtract(Interval::Exact(hi),Interval::Exact(lo)),
        A::Subtract(Interval::Exact(1),Interval::Exact(lo)),fraction))return false;
    // The exact ratio is in [0,1] by the already checked original parameters.
    fraction.Lower=FMath::Max(0.,fraction.Lower);fraction.Upper=FMath::Min(1.,fraction.Upper);
    FCurve tail;return Split(right,fraction,out,tail);
}
inline bool Gradient(const FCurve& c,const FCurve& d,const FIntervalPoint& center,double t,Interval& out,FIntervalPoint* evaluatedPoint=nullptr)
{
    FIntervalPoint p,v;if(!Evaluate(c,t,p)||!Evaluate(d,t,v))return false;
    out=Interval::Exact(0);
    for(int32 k=0;k<3;++k)out=A::Add(out,A::Multiply(A::Subtract(p.Axis[k],center.Axis[k]),v.Axis[k]));
    out=A::Multiply(Interval::Exact(2),out);
    if(!A::Valid(out))return false;if(evaluatedPoint)*evaluatedPoint=p;return true;
}
inline bool Curvature(const FCurve& c,const FCurve& d,const FCurve& dd,
    const FIntervalPoint& center,Interval& out)
{
    FIntervalPoint p,v,a;if(!Hull(c,p)||!Hull(d,v)||!Hull(dd,a))return false;
    out=Interval::Exact(0);
    for(int32 k=0;k<3;++k)out=A::Add(out,A::Add(A::Square(v.Axis[k]),
        A::Multiply(A::Subtract(p.Axis[k],center.Axis[k]),a.Axis[k])));
    out=A::Multiply(Interval::Exact(2),out);return A::Valid(out);
}
// Per-thread bounded value cache. These properties depend only on the exact
// six interval coefficients and the admitted arithmetic controls, never on a
// center, time interval, owner identity, snapshot generation or contact result.
// Budget stages remain charged on hits, and original-UV witnesses are unchanged.
struct FChordEnvelope
{
    FVector3d A=FVector3d::ZeroVector,B=FVector3d::ZeroVector;
    double EpsilonUpper=0;
};
using FChordEnvelopes=TArray<FChordEnvelope,TInlineAllocator<8>>;

struct FCurveProperties
{
    mutable FChordEnvelopes Chords;mutable bool ChordsReady=false;
    FCurve D,DD;FIntervalPoint CHull,DHull,DDHull;
    // Untrusted scalar proposal geometry only. None of these values can
    // supply a distance, witness, curvature bound, normal or contact decision.
    mutable FVector3d ScalarCoefficients[6],ScalarAnchor=FVector3d::ZeroVector;
    mutable bool ScalarTried=false,ScalarReady=false;
    // Lazily memoize literal Evaluate results at the three fixed parameters
    // used by the endpoint signs and first Newton/decision probe. These points
    // are immutable geometry, independent of the query center or its bounds.
    mutable FIntervalPoint FixedPoints[3],FixedDerivatives[3];
    mutable uint8 FixedReady=0;
#if WITH_DEV_AUTOMATION_TESTS
    mutable uint32 FixedPointBuildsForTesting=0;
#endif
};
struct FCurvePropertyKey
{
    uint64 Endpoints[36]{};uint64 Numeric=0;
    bool operator==(const FCurvePropertyKey& other)const
    {
        if(Numeric!=other.Numeric)return false;
        for(int32 i=0;i<36;++i)if(Endpoints[i]!=other.Endpoints[i])return false;
        return true;
    }
};
struct FCurvePropertyCache
{
    struct FEntry{FCurvePropertyKey Key;FCurveProperties Value;bool Ready=false;};
    FEntry Entries[64];uint64 Hits=0,Misses=0;
};
static FCurvePropertyCache& CurvePropertyCache()
{
    static thread_local FCurvePropertyCache cache;return cache;
}
static bool BuildCurveProperties(const FCurve& c,FCurveProperties& out)
{
    out={}; // A direct-mapped replacement must not inherit fixed-point memos.
    return Derivative(c,out.D)&&Derivative(out.D,out.DD)&&
        Hull(c,out.CHull)&&Hull(out.D,out.DHull)&&Hull(out.DD,out.DDHull);
}
static uint32 CurvePropertySlot(const FCurvePropertyKey& key)
{
    uint64 hash=1469598103934665603ULL;
    const auto mix=[&](uint64 word){hash^=word;hash*=1099511628211ULL;};
    mix(key.Numeric);for(uint64 word:key.Endpoints)mix(word);
    // Word-wise FNV alone leaves the slot dependent only on the six low
    // coefficient bits. Promoted floats and exact integer nets commonly
    // share all those bits. Avalanche the whole word before the power-of-two
    // reduction; full endpoint equality remains the authority for cache hits.
    hash^=hash>>33;hash*=0xff51afd7ed558ccdULL;
    hash^=hash>>33;hash*=0xc4ceb9fe1a85ec53ULL;
    hash^=hash>>33;
    return uint32(hash&63);
}
static const FCurveProperties* GetCurveProperties(const FCurve& c,uint64 numeric)
{
    // Caller has validated all 18 Known intervals; Known is therefore true
    // for every stored key. Endpoints include signed-zero representations.
    FCurvePropertyKey key;key.Numeric=numeric;
    int32 index=0;for(const auto& p:c)for(const auto& axis:p.Axis)
    {key.Endpoints[index++]=A::Bits(axis.Lower);key.Endpoints[index++]=A::Bits(axis.Upper);}
    if(index!=36)return nullptr;
    auto& cache=CurvePropertyCache();auto& entry=cache.Entries[CurvePropertySlot(key)];
    if(entry.Ready&&entry.Key==key){++cache.Hits;return &entry.Value;}
    ++cache.Misses;entry.Ready=false;
    if(!BuildCurveProperties(c,entry.Value))return nullptr;
    entry.Key=key;entry.Ready=true;return &entry.Value;
}
static bool GradientWithProperties(const FCurve& c,const FCurveProperties& properties,
    const FIntervalPoint& center,double t,Interval& out,FIntervalPoint* evaluatedPoint=nullptr,bool useFixedPointCache=true)
{
    const uint64 bits=A::Bits(t);
    const int32 slot=bits==0?0:bits==0x3fe0000000000000ULL?1:bits==0x3ff0000000000000ULL?2:INDEX_NONE;
    if(!useFixedPointCache||slot==INDEX_NONE)return Gradient(c,properties.D,center,t,out,evaluatedPoint);
    const uint8 ready=uint8(1u<<slot);
    if(!(properties.FixedReady&ready))
    {
        FIntervalPoint point,derivative;
        // Preserve the original evaluation order and failure point. Failed
        // evaluation is never published and no center-specific value is cached.
        if(!Evaluate(c,t,point)||!Evaluate(properties.D,t,derivative))return false;
        properties.FixedPoints[slot]=point;properties.FixedDerivatives[slot]=derivative;
        properties.FixedReady|=ready;
#if WITH_DEV_AUTOMATION_TESTS
        ++properties.FixedPointBuildsForTesting;
#endif
    }
    const auto& point=properties.FixedPoints[slot];const auto& derivative=properties.FixedDerivatives[slot];
    out=Interval::Exact(0);
    for(int32 k=0;k<3;++k)out=A::Add(out,A::Multiply(A::Subtract(point.Axis[k],center.Axis[k]),derivative.Axis[k]));
    out=A::Multiply(Interval::Exact(2),out);
    if(!A::Valid(out))return false;if(evaluatedPoint)*evaluatedPoint=point;return true;
}
static bool FiniteScalarPoint(const FVector3d& point)
{return A::Finite(point.X)&&A::Finite(point.Y)&&A::Finite(point.Z);}
static bool EnsureScalarProposalProperties(const FCurve& c,const FCurveProperties& properties)
{
    if(!properties.ScalarTried)
    {
        FVector3d coefficients[6],differences[6],anchor=FVector3d::ZeroVector;bool valid=c.Num()==6;
        if(valid)
        {
            for(int32 axis=0;axis<3;++axis)anchor[axis]=.5*c[0].Axis[axis].Lower+.5*c[0].Axis[axis].Upper;
            valid=FiniteScalarPoint(anchor);
        }
        for(int32 i=0;i<6&&valid;++i)
        {
            for(int32 axis=0;axis<3;++axis)differences[i][axis]=
                (.5*c[i].Axis[axis].Lower+.5*c[i].Axis[axis].Upper)-anchor[axis];
            valid=FiniteScalarPoint(differences[i]);
        }
        int32 binomial=1;
        for(int32 order=0;order<6&&valid;++order)
        {
            FVector3d work[6];const int32 count=6-order;
            for(int32 i=0;i<count;++i)work[i]=differences[i];
            for(int32 remaining=count-1;remaining>0;--remaining)for(int32 i=0;i<remaining;++i)
                work[i]=.5*work[i]+.5*work[i+1];
            coefficients[order]=double(binomial)*work[0];valid=FiniteScalarPoint(coefficients[order]);
            for(int32 i=0;i<count-1&&valid;++i)
            {differences[i]=differences[i+1]-differences[i];valid=FiniteScalarPoint(differences[i]);}
            if(order<5)binomial=binomial*(5-order)/(order+1);
        }
        if(valid)
        {properties.ScalarAnchor=anchor;for(int32 i=0;i<6;++i)properties.ScalarCoefficients[i]=coefficients[i];properties.ScalarReady=true;}
        properties.ScalarTried=true;
    }
    return properties.ScalarReady;
}
static bool ProposeScalarMinimum(const FCurve& c,const FCurveProperties& properties,
    const FIntervalPoint& center,double& parameter,int32& steps)
{
    steps=0;parameter=.5;
    if(!EnsureScalarProposalProperties(c,properties))return false;
    FVector3d relativeCenter;
    for(int32 axis=0;axis<3;++axis)relativeCenter[axis]=properties.ScalarAnchor[axis]-
        (.5*center.Axis[axis].Lower+.5*center.Axis[axis].Upper);
    if(!FiniteScalarPoint(relativeCenter))return false;
    for(int32 step=0;step<5;++step)
    {
        const double shifted=parameter-.5;
        FVector3d value=properties.ScalarCoefficients[5],derivative=FVector3d::ZeroVector,second=FVector3d::ZeroVector;
        for(int32 i=4;i>=0;--i)
        {second=second*shifted+2.*derivative;derivative=derivative*shifted+value;value=value*shifted+properties.ScalarCoefficients[i];}
        value+=relativeCenter;
        if(!FiniteScalarPoint(value)||!FiniteScalarPoint(derivative)||!FiniteScalarPoint(second))return false;
        double gradient=0,curvature=0;
        for(int32 axis=0;axis<3;++axis)
        {gradient+=value[axis]*derivative[axis];curvature+=derivative[axis]*derivative[axis]+value[axis]*second[axis];}
        if(!A::Finite(gradient)||!A::Finite(curvature)||!(curvature>0))return false;
        const double proposal=parameter-gradient/curvature;
        if(!A::Finite(proposal))return false;
        const double next=FMath::Clamp(proposal,0.,1.);++steps;
        if(next==parameter)break;parameter=next;
    }
    return A::Finite(parameter)&&parameter>=0&&parameter<=1;
}
static bool ScalarProposalRuntimeEnabled(){return CVarStrongConvexScalarProposal.GetValueOnAnyThread()==1;}
static bool WholeCurvature(const FCurveProperties& properties,const FIntervalPoint& center,Interval& out)
{
    // Literal arithmetic order of Curvature, using its already validated hulls.
    const auto& p=properties.CHull;const auto& v=properties.DHull;const auto& a=properties.DDHull;
    out=Interval::Exact(0);
    for(int32 k=0;k<3;++k)out=A::Add(out,A::Add(A::Square(v.Axis[k]),
        A::Multiply(A::Subtract(p.Axis[k],center.Axis[k]),a.Axis[k])));
    out=A::Multiply(Interval::Exact(2),out);return A::Valid(out);
}
namespace Observation { bool Enabled();void RecordDecision(bool stopped);void RecordScalar(bool proposed,bool validated,bool decided,int32 steps);
    void RecordValidationParameter(bool fixed);void RecordJoint(bool valid,bool stopped,uint64 ticks); }
// For every represented fixed curve and center, f'' >= mu > 0 throughout
// [0,1]. Completing the square in the strong-convex Taylor inequality gives
// min f >= f(m) - g(m)^2/(2*mu), including both constrained endpoints.
// The point is evaluated on the ORIGINAL curve at fixed original parameter m.
// No root, local-curvature coverage or retained-UV surrogate is assumed.
static bool AccumulateDecisionBound(const FIntervalPoint& point,const FIntervalPoint& center,
    double m,Interval gradient,double globalCurvatureLower,FTensorDistanceBounds& accumulated)
{
    Interval squared=Interval::Exact(0);
    for(int32 k=0;k<3;++k)squared=A::Add(squared,A::Square(A::Subtract(point.Axis[k],center.Axis[k])));
    const auto gradientSquared=A::Square(gradient);
    const auto twiceMu=A::Multiply(Interval::Exact(2),Interval::Exact(globalCurvatureLower));
    Interval correction;
    if(!A::Valid(squared)||!A::Valid(gradientSquared)||!DividePositive(gradientSquared,twiceMu,correction))return false;
    const auto lower=A::Subtract(squared,correction);
    if(!A::Valid(lower))return false;
    const double candidateLower=FMath::Max(0.,lower.Lower);
    if(candidateLower>squared.Upper)return false;
    FTensorDistanceBounds next=accumulated;
    if(next.Status==EDistanceBoundStatus::Unknown)
    {next.Status=EDistanceBoundStatus::Bounded;next.LowerSquared=candidateLower;next.UpperSquared=squared.Upper;next.SampleUV=FVector2d(0,m);next.SamplePoint=point;}
    else
    {
        next.LowerSquared=FMath::Max(next.LowerSquared,candidateLower);
        if(squared.Upper<next.UpperSquared){next.UpperSquared=squared.Upper;next.SampleUV=FVector2d(0,m);next.SamplePoint=point;}
    }
    if(next.LowerSquared>next.UpperSquared)return false;
    accumulated=MoveTemp(next);return true;
}
static bool DecisionSatisfied(const FTensorDistanceBounds& bound,const Interval* threshold,double requestedGap)
{
    if(bound.Status==EDistanceBoundStatus::Unknown)return false;
    if(threshold)return bound.LowerSquared>threshold->Upper||bound.UpperSquared<threshold->Lower;
    const auto gap=A::Subtract(Interval::Exact(bound.UpperSquared),Interval::Exact(bound.LowerSquared));
    return A::Valid(gap)&&gap.Upper<=requestedGap;
}
// Same original quintic value, plus its derivative from the penultimate
// de Casteljau level: C'(t)=5*(P1(t)-P0(t)). Every interval realization is
// covered; derivative dependency may be wider, so this is decision-only.
static bool EvaluateJointOriginal(const FCurve& c,double t,FIntervalPoint& point,FIntervalPoint& derivative)
{
    point={};derivative={};if(c.Num()!=6||!A::Finite(t)||t<0||t>1)return false;
    for(const auto& cp:c)if(!Detail::ValidPoint(cp))return false;
    FCurve work=c;const auto parameter=Interval::Exact(t);
    for(int32 count=5;count>1;--count)for(int32 i=0;i<count;++i)
        work[i]=ProjectedDistanceDetail::LerpPoint(work[i],work[i+1],parameter);
    point=ProjectedDistanceDetail::LerpPoint(work[0],work[1],parameter);
    if(!Detail::ValidPoint(point))return false;
    for(int32 d=0;d<3;++d)derivative.Axis[d]=A::Multiply(Interval::Exact(5),A::Subtract(work[1].Axis[d],work[0].Axis[d]));
    return Detail::ValidPoint(derivative);
}
// Exact original fallback given the already computed, bit-identical C(t).
// The derivative still comes from the original quartic derivative net.
static bool OriginalGradientAtPoint(const FCurve& derivativeNet,const FIntervalPoint& center,double t,
    const FIntervalPoint& point,Interval& gradient)
{
    gradient={};FIntervalPoint derivative;
    if(!Detail::ValidPoint(center)||!Detail::ValidPoint(point)||!Evaluate(derivativeNet,t,derivative))return false;
    auto result=Interval::Exact(0);
    for(int32 d=0;d<3;++d)result=A::Add(result,A::Multiply(A::Subtract(point.Axis[d],center.Axis[d]),derivative.Axis[d]));
    result=A::Multiply(Interval::Exact(2),result);if(!A::Valid(result))return false;gradient=result;return true;
}
static bool JointOriginalGradient(const FCurve& c,const FIntervalPoint& center,double t,Interval& gradient,FIntervalPoint& point)
{
    gradient={};point={};FIntervalPoint derivative;
    if(!Detail::ValidPoint(center)||!EvaluateJointOriginal(c,t,point,derivative))return false;
    auto result=Interval::Exact(0);
    for(int32 d=0;d<3;++d)result=A::Add(result,A::Multiply(A::Subtract(point.Axis[d],center.Axis[d]),derivative.Axis[d]));
    result=A::Multiply(Interval::Exact(2),result);if(!A::Valid(result))return false;gradient=result;return true;
}
static bool FixedEvaluationParameter(double t)
{const uint64 bits=A::Bits(t);return bits==0||bits==0x3fe0000000000000ULL||bits==0x3ff0000000000000ULL;}

static EResult TryBoundImpl(const FIntervalTensorSurface& surface,const FIntervalPoint& center,
    FDistanceBudget& budget,int32 maximumWork,FTensorDistanceBounds& out,FInspection* inspect,bool usePropertyCache=true,
    const Interval* decisionThresholdSquared=nullptr,double requestedSquaredGap=0,bool useFixedPointCache=true,bool useScalarProposal=false,
    bool useJointDecision=false)
{
    out={};if(inspect)*inspect={};
    const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    // Narrow constant-size admission. No new route for genuine 2D surfaces.
    if(surface.DegreeU!=0||surface.DegreeV!=5||surface.ControlPoints.Num()!=6)return EResult::NotApplicable;
    FWork work{budget,budget.Used,maximumWork};
    if(!work.Pay())return work.Failure;
    if(!Detail::ValidPoint(center))return EResult::Arithmetic;
    FCurve c=surface.ControlPoints;
    for(const auto& p:c)if(!Detail::ValidPoint(p))return EResult::Arithmetic;
    if(!work.Pay())return work.Failure;
    FCurveProperties uncached;
    const FCurveProperties* properties=nullptr;
    if(usePropertyCache)properties=GetCurveProperties(c,numeric.ControlFingerprint());
    else if(BuildCurveProperties(c,uncached))properties=&uncached;
    if(!properties)return EResult::Arithmetic;
    const auto& d=properties->D;const auto& dd=properties->DD;
    Interval wholeCurvature;
    if(!work.Pay())return work.Failure;
    if(!WholeCurvature(*properties,center,wholeCurvature))return EResult::Arithmetic;
    if(!(wholeCurvature.Lower>0))return EResult::NotApplicable;
    const bool adaptive=decisionThresholdSquared||requestedSquaredGap>0;
    if((decisionThresholdSquared&&!A::Valid(*decisionThresholdSquared))||!A::Finite(requestedSquaredGap)||requestedSquaredGap<0)return EResult::Arithmetic;
    const bool observeDecision=adaptive&&Observation::Enabled();
    FTensorDistanceBounds decisionBound;
    Interval g0,g1;
    if(!work.Pay())return work.Failure;
    if(!GradientWithProperties(c,*properties,center,0,g0,nullptr,useFixedPointCache)||
        !GradientWithProperties(c,*properties,center,1,g1,nullptr,useFixedPointCache))return EResult::Arithmetic;
    const bool possibleStart=g0.Upper>=0,possibleEnd=g1.Lower<=0;
    bool seeded=false;double proposalParameter=.5;Interval proposalGradient;
    // Reserve the whole prior algorithm plus the bounded optional attempt.
    // Original maximum:34 units if validation fails,33 with a seed.
    // A joint attempt can add at most2 paid stages before the literal fallback:
    // maximum36 remains within the EXISTING gate36/account reservation.
    // Existing maximumWork/global/floor limits are never raised or refunded.
    const bool proposalReserve=maximumWork>=36&&budget.ConsumptionFloor>=0&&
        int64(budget.Remaining)-int64(budget.ConsumptionFloor)>=32;
    if(useScalarProposal&&proposalReserve)
    {
        if(!work.Pay())return work.Failure;
        int32 scalarSteps=0;const bool proposed=ProposeScalarMinimum(c,*properties,center,proposalParameter,scalarSteps);
        bool validated=false,decided=false;
        if(proposed)
        {
            if(!work.Pay())return work.Failure;
            FIntervalPoint point;
            const bool fixed=FixedEvaluationParameter(proposalParameter);
            if(observeDecision)Observation::RecordValidationParameter(fixed);
            const bool joint=useJointDecision&&adaptive&&!fixed;
            if(joint)
            {
                const uint64 tick=observeDecision?FPlatformTime::Cycles64():0;
                Interval jointGradient;FIntervalPoint jointPoint;
                const bool valid=JointOriginalGradient(c,center,proposalParameter,jointGradient,jointPoint);
                if(valid)
                {
                    if(!work.Pay())return work.Failure;
                    FTensorDistanceBounds jointBound;
                    const bool proved=AccumulateDecisionBound(jointPoint,center,proposalParameter,jointGradient,wholeCurvature.Lower,jointBound);
                    decided=proved&&DecisionSatisfied(jointBound,decisionThresholdSquared,requestedSquaredGap);
                    if(decided){decisionBound=MoveTemp(jointBound);validated=true;}
                }
                if(observeDecision)Observation::RecordJoint(valid,decided,FPlatformTime::Cycles64()-tick);
                // C(t) followed exactly the old fifteen interpolation steps.
                // Its value survives locally; no joint derivative/bound does.
                if(Detail::ValidPoint(jointPoint))point=jointPoint;
                // No joint gradient or bound enters Newton or the old fallback.
                // Its evaluation/decision charges stay in the SAME account.
                if(!decided&&!work.Pay())return work.Failure;
            }
            if(!decided)
            {
                // Only original interval evaluations can become witnesses.
                // The old derivative path remains literal after an inconclusive
                // joint enclosure; scalar values never enter either proof.
                validated=joint&&Detail::ValidPoint(point)?
                    OriginalGradientAtPoint(properties->D,center,proposalParameter,point,proposalGradient):
                    GradientWithProperties(c,*properties,center,proposalParameter,proposalGradient,&point,useFixedPointCache);
                if(validated)
                {
                    seeded=true;
                    if(adaptive)
                    {
                        if(!work.Pay())return work.Failure;
                        const bool proved=AccumulateDecisionBound(point,center,proposalParameter,proposalGradient,wholeCurvature.Lower,decisionBound);
                        decided=proved&&DecisionSatisfied(decisionBound,decisionThresholdSquared,requestedSquaredGap);
                    }
                }
            }
        }
        if(observeDecision)Observation::RecordScalar(proposed,validated,decided,scalarSteps);
        if(decided)
        {
            out=MoveTemp(decisionBound);
            if(inspect)
            {inspect->CurvatureLower=wholeCurvature.Lower;inspect->ParameterLower=0;inspect->ParameterUpper=1;
             inspect->NewtonSteps=0;inspect->PossibleStart=possibleStart;inspect->PossibleEnd=possibleEnd;}
            RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);return EResult::Bounded;
        }
        // Invalid proposal/evaluation falls through without publishing values.
        // A valid fixed v can seed the FIRST exact interval-Newton image;
        // no root is discarded by the scalar solver or by clamping its guess.
    }
    double lo=0,hi=1;bool roots=true;int32 iterations=0;
    for(int32 step=0;step<8&&roots&&lo<hi;++step)
    {
        // Try the inexpensive universal decision before three de Casteljau
        // restrictions. Reuse both the fixed original point and its gradient
        // in the Newton proposal if this radius is still undecided.
        const bool seededStep=seeded&&step==0;
        const double m=seededStep?proposalParameter:.5*lo+.5*hi;
        if(!A::Finite(m)||m<lo||m>hi)return EResult::Arithmetic;
        Interval gm=seededStep?proposalGradient:Interval{},ratio;
        if(adaptive&&!seededStep)
        {
            if(!work.Pay())return work.Failure;
            FIntervalPoint point;
            if(!GradientWithProperties(c,*properties,center,m,gm,&point,useFixedPointCache))return EResult::Arithmetic;
            if(!work.Pay())return work.Failure;
            const bool proved=AccumulateDecisionBound(point,center,m,gm,wholeCurvature.Lower,decisionBound);
            const bool decided=proved&&DecisionSatisfied(decisionBound,decisionThresholdSquared,requestedSquaredGap);
            if(observeDecision)Observation::RecordDecision(decided);
            // A failed optional enclosure adds no bound and refunds no work.
            if(decided)
            {
                out=MoveTemp(decisionBound);
                if(inspect)
                {inspect->CurvatureLower=wholeCurvature.Lower;inspect->ParameterLower=possibleStart?0:lo;inspect->ParameterUpper=possibleEnd?1:hi;
                 inspect->NewtonSteps=iterations;inspect->PossibleStart=possibleStart;inspect->PossibleEnd=possibleEnd;}
                RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
                return EResult::Bounded;
            }
        }
        if(!work.Pay())return work.Failure;
        FCurve rc,rd,rdd;
        if(!Restrict(c,lo,hi,rc)||!Restrict(d,lo,hi,rd)||!Restrict(dd,lo,hi,rdd))return EResult::Arithmetic;
        Interval curvature;
        if(!Curvature(rc,rd,rdd,center,curvature))return EResult::Arithmetic;
        curvature.Lower=FMath::Max(curvature.Lower,wholeCurvature.Lower);
        curvature.Upper=FMath::Min(curvature.Upper,wholeCurvature.Upper);
        if(!A::Valid(curvature))return EResult::Arithmetic;
        // m is a fixed original parameter, not an interval-dependent witness.
        if(!adaptive&&!seededStep)
        {
            if(!work.Pay())return work.Failure;
            if(!GradientWithProperties(c,*properties,center,m,gm,nullptr,useFixedPointCache))return EResult::Arithmetic;
        }
        if(!DividePositive(gm,curvature,ratio))return EResult::Arithmetic;
        const auto next=A::Subtract(Interval::Exact(m),ratio);
        if(!A::Valid(next))return EResult::Arithmetic;
        const double newLo=FMath::Max(lo,next.Lower),newHi=FMath::Min(hi,next.Upper);
        ++iterations;
        if(newLo>newHi){roots=false;break;}
        if(newLo==lo&&newHi==hi)break;
        lo=newLo;hi=newHi;
    }
    // For every represented polynomial and center, the constrained minimum
    // is an interior root OR an original endpoint with the corresponding KKT
    // sign. Never discard endpoints just because an interior Newton interval
    // is empty. Union, then enclose, every possible original endpoint.
    if(!roots)
    {
        if(!possibleStart&&!possibleEnd)return EResult::Arithmetic;
        lo=possibleStart?0:1;hi=possibleEnd?1:0;
    }
    else
    {if(possibleStart)lo=0;if(possibleEnd)hi=1;}
    if(lo>hi||lo<0||hi>1)return EResult::Arithmetic;
    if(!work.Pay())return work.Failure;
    FCurve restricted;FIntervalPoint hull;
    if(!Restrict(c,lo,hi,restricted)||!Hull(restricted,hull))return EResult::Arithmetic;
    Interval lower=Interval::Exact(0);
    for(int32 k=0;k<3;++k)lower=A::Add(lower,A::Square(A::Subtract(hull.Axis[k],center.Axis[k])));
    if(!A::Valid(lower))return EResult::Arithmetic;
    double bestUpper=0,bestV=0;FIntervalPoint bestPoint;bool have=false;
    for(double v:{.5*lo+.5*hi,lo,hi})
    {
        if(!work.Pay())return work.Failure;
        FIntervalPoint point;
        if(!ProjectedDistanceDetail::EvaluateFixedUV(surface,FVector2d(0,v),point))return EResult::Arithmetic;
        Interval squared=Interval::Exact(0);
        for(int32 k=0;k<3;++k)squared=A::Add(squared,A::Square(A::Subtract(point.Axis[k],center.Axis[k])));
        if(!A::Valid(squared))return EResult::Arithmetic;
        if(!have||squared.Upper<bestUpper){bestUpper=squared.Upper;bestV=v;bestPoint=point;have=true;}
    }
    const double bestLower=FMath::Max(0.,lower.Lower);
    if(!have||bestLower>bestUpper)return EResult::Arithmetic;
    out.Status=EDistanceBoundStatus::Bounded;out.LowerSquared=bestLower;out.UpperSquared=bestUpper;
    out.SampleUV=FVector2d(0,bestV);out.SamplePoint=bestPoint;
    if(decisionBound.Status!=EDistanceBoundStatus::Unknown)
    {
        out.LowerSquared=FMath::Max(out.LowerSquared,decisionBound.LowerSquared);
        if(decisionBound.UpperSquared<out.UpperSquared)
        {out.UpperSquared=decisionBound.UpperSquared;out.SampleUV=decisionBound.SampleUV;out.SamplePoint=decisionBound.SamplePoint;}
        if(out.LowerSquared>out.UpperSquared)return EResult::Arithmetic;
    }
    if(inspect)
    {inspect->CurvatureLower=wholeCurvature.Lower;inspect->ParameterLower=lo;inspect->ParameterUpper=hi;
     inspect->NewtonSteps=iterations;inspect->PossibleStart=possibleStart;inspect->PossibleEnd=possibleEnd;}
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    return EResult::Bounded;
}
}

}


// Passive CPP-only measurement. It shares PhaseTiming with the existing contact
// profile and changes neither the helper's result nor its work account.
namespace Speed::Analytic::Certified::StrongConvexCurveMinimum
{
namespace Observation
{
struct FRows
{
    std::atomic<uint64> Calls{0},Finished{0},InFlight{0},Ticks{0},Work{0},Steps{0};
    std::atomic<uint64> Results[5]{},Iterations[9]{},EndpointCases{0},Narrow1e6{0},DecisionProbes{0},DecisionStops{0};
    std::atomic<uint64> ScalarAttempts{0},ScalarProposed{0},ScalarValidated{0},ScalarStops{0},ScalarSteps{0};
    std::atomic<uint64> ValidationFixed{0},ValidationVariable{0},JointAttempts{0},JointValid{0},JointStops{0},JointFallbacks{0},JointTicks{0};
    std::atomic<bool> Integrity{true};
};
FRows Rows;
bool Enabled()
{
    static const IConsoleVariable* cvar=IConsoleManager::Get().FindConsoleVariable(
        TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"));
    return cvar&&cvar->GetInt()!=0;
}
void Add(std::atomic<uint64>& target,uint64 value)
{if(target.fetch_add(value,std::memory_order_relaxed)>MAX_uint64-value)Rows.Integrity.store(false,std::memory_order_relaxed);}
void RecordDecision(bool stopped)
{Add(Rows.DecisionProbes,1);if(stopped)Add(Rows.DecisionStops,1);}
void RecordScalar(bool proposed,bool validated,bool decided,int32 steps)
{Add(Rows.ScalarAttempts,1);if(proposed)Add(Rows.ScalarProposed,1);if(validated)Add(Rows.ScalarValidated,1);if(decided)Add(Rows.ScalarStops,1);if(steps>=0)Add(Rows.ScalarSteps,uint64(steps));else Rows.Integrity.store(false);}
void RecordValidationParameter(bool fixed)
{Add(fixed?Rows.ValidationFixed:Rows.ValidationVariable,1);}
void RecordJoint(bool valid,bool stopped,uint64 ticks)
{Add(Rows.JointAttempts,1);if(valid)Add(Rows.JointValid,1);Add(stopped?Rows.JointStops:Rows.JointFallbacks,1);Add(Rows.JointTicks,ticks);}
struct FScope
{
    FDistanceBudget& Budget;const FInspection& Inspection;
    uint64 Before=0,Start=0;EResult Result=EResult::Arithmetic;
    explicit FScope(FDistanceBudget& budget,const FInspection& inspection)
        :Budget(budget),Inspection(inspection),Before(budget.Used),Start(FPlatformTime::Cycles64())
    {Add(Rows.Calls,1);Add(Rows.InFlight,1);}
    ~FScope()
    {
        const uint64 end=FPlatformTime::Cycles64();
        if(end<Start||Budget.Used<Before)Rows.Integrity.store(false,std::memory_order_relaxed);
        Add(Rows.Ticks,end>=Start?end-Start:0);Add(Rows.Work,Budget.Used>=Before?Budget.Used-Before:0);
        const uint32 outcome=uint32(Result);
        if(outcome<5)Add(Rows.Results[outcome],1);else Rows.Integrity.store(false,std::memory_order_relaxed);
        if(Result==EResult::Bounded)
        {
            if(Inspection.NewtonSteps<0||Inspection.NewtonSteps>8)Rows.Integrity.store(false,std::memory_order_relaxed);
            else {Add(Rows.Steps,uint64(Inspection.NewtonSteps));Add(Rows.Iterations[Inspection.NewtonSteps],1);}
            Add(Rows.EndpointCases,uint64(Inspection.PossibleStart||Inspection.PossibleEnd));
            Add(Rows.Narrow1e6,uint64(Inspection.ParameterUpper-Inspection.ParameterLower<=1e-6));
        }
        Add(Rows.Finished,1);
        if(Rows.InFlight.fetch_sub(1,std::memory_order_relaxed)==0)Rows.Integrity.store(false,std::memory_order_relaxed);
    }
};
}
EResult TryBound(const FIntervalTensorSurface& surface,const FIntervalPoint& center,
    FDistanceBudget& budget,int32 maximumWork,FTensorDistanceBounds& out,FInspection* inspect)
{
    if(!Observation::Enabled())return TryBoundImpl(surface,center,budget,maximumWork,out,inspect);
    FInspection local;FInspection& inspection=inspect?*inspect:local;
    Observation::FScope observation(budget,inspection);
    const auto result=TryBoundImpl(surface,center,budget,maximumWork,out,&inspection);
    observation.Result=result;return result;
}
// Decision-aware route for one-shot and retained callers. Existing TryBound is unchanged.
EResult TryBoundForDecision(const FIntervalTensorSurface& surface,const FIntervalPoint& center,
    FDistanceBudget& budget,int32 maximumWork,FTensorDistanceBounds& out,const Interval* threshold,
    double requestedGap,FInspection* inspect)
{
    const bool scalar=ScalarProposalRuntimeEnabled(),joint=CVarStrongConvexJointDecision.GetValueOnAnyThread()==1;
    if(!Observation::Enabled())return TryBoundImpl(surface,center,budget,maximumWork,out,inspect,true,threshold,requestedGap,true,scalar,joint);
    FInspection local;FInspection& inspection=inspect?*inspect:local;
    Observation::FScope observation(budget,inspection);
    const auto result=TryBoundImpl(surface,center,budget,maximumWork,out,&inspection,true,threshold,requestedGap,true,scalar,joint);
    observation.Result=result;return result;
}
void PublishObservation()
{
    const auto& r=Observation::Rows;
    UE_LOG(LogTemp,Display,TEXT("[StrongConvexJointDecision] DefaultOFF=1 DecisionOnly=1 MaximumHelperWork=36 ValidationFixed=%llu ValidationVariable=%llu Attempts=%llu Valid=%llu Stops=%llu Fallbacks=%llu Ticks=%llu SecondsPerCycle=%.17g"),
        r.ValidationFixed.load(),r.ValidationVariable.load(),r.JointAttempts.load(),r.JointValid.load(),r.JointStops.load(),r.JointFallbacks.load(),r.JointTicks.load(),FPlatformTime::GetSecondsPerCycle64());const uint64 calls=r.Calls.load(),finished=r.Finished.load(),flight=r.InFlight.load();
    UE_LOG(LogTemp,Display,TEXT("[StrongConvexCurveProfile] Scope=ProgramPrefixAtPreExit PerCase=0 RequiresExternalDrain=1 NoPASS=1 Calls=%llu Finished=%llu InFlight=%llu Integrity=%d Ticks=%llu SecondsPerCycle=%.17g Work=%llu NotApplicable=%llu Bounded=%llu LocalLimit=%llu WorkFailure=%llu Arithmetic=%llu NewtonSteps=%llu EndpointCases=%llu Narrow1e6=%llu DecisionProbes=%llu DecisionStops=%llu ScalarAttempts=%llu ScalarProposed=%llu ScalarValidated=%llu ScalarStops=%llu ScalarSteps=%llu Iterations0to8=%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu"),
        calls,finished,flight,int32(r.Integrity.load()),r.Ticks.load(),FPlatformTime::GetSecondsPerCycle64(),r.Work.load(),
        r.Results[0].load(),r.Results[1].load(),r.Results[2].load(),r.Results[3].load(),r.Results[4].load(),
        r.Steps.load(),r.EndpointCases.load(),r.Narrow1e6.load(),r.DecisionProbes.load(),r.DecisionStops.load(),
        r.ScalarAttempts.load(),r.ScalarProposed.load(),r.ScalarValidated.load(),r.ScalarStops.load(),r.ScalarSteps.load(),
        r.Iterations[0].load(),r.Iterations[1].load(),r.Iterations[2].load(),r.Iterations[3].load(),r.Iterations[4].load(),
        r.Iterations[5].load(),r.Iterations[6].load(),r.Iterations[7].load(),r.Iterations[8].load());
}
}

namespace Speed::Analytic::Certified::TimeDetail::JointProjectedSpherePrefix
{
static TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.JointProjectedSpherePrefix"),0,
    TEXT("Bound the exact projected center minus source profile on a quadratic time cell. Default OFF."));
IAMSPEED_API bool RuntimeEnabled(){return Enabled.GetValueOnAnyThread()==1;}
}

namespace Speed::Analytic::Certified
{
FTensorDistanceBounds BoundTensorDistanceSquaredIntervalCenter(const FIntervalTensorSurface& surface,
    const FIntervalPoint& centerBounds,FDistanceBudget& budget,double requestedSquaredGap,
    int32 maximumNodesThisCall,const Interval* decisionThresholdSquared,Detail::FMinimumMidpointMemo* midpointMemo)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    FTensorDistanceBounds result;
    const uint64 usedBefore=budget.Used;
    PlaneProposal95::FWork proposal95{&surface,&budget,usedBefore,maximumNodesThisCall,requestedSquaredGap};
    if (maximumNodesThisCall<=0 || surface.DegreeU<0 || surface.DegreeV<0 || surface.DegreeU>16 || surface.DegreeV>16 ||
        surface.ControlPoints.Num()!=(surface.DegreeU+1)*(surface.DegreeV+1) ||
        !AnalyticCertified::Finite(requestedSquaredGap) ||
        ((AnalyticCertified::Bits(requestedSquaredGap)&AnalyticCertified::Sign)!=0 &&
         (AnalyticCertified::Bits(requestedSquaredGap)&AnalyticCertified::Magnitude)!=0) ||
        (decisionThresholdSquared&&!AnalyticCertified::Valid(*decisionThresholdSquared)) || !budget.Consume()) return result;
    if (!Detail::ValidPoint(centerBounds))return result;
    Detail::FMinimumMidpointMemo::FCallScope midpointScope(midpointMemo,surface,
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint());
    Detail::FNode first; first.DegreeU=surface.DegreeU;first.DegreeV=surface.DegreeV;
    for (const auto& p:surface.ControlPoints)first.Net.Add(p);
    if (!Detail::Bounds(first,centerBounds,decisionThresholdSquared,&midpointScope,&proposal95))return result;
    TArray<Detail::FNode> leaves;leaves.Add(MoveTemp(first));
    Detail::FMinLowerLeafHeap minimum;minimum.PushIndex(0,leaves);
    result.UpperSquared=leaves[0].UpperSquared;
    result.SampleUV=leaves[0].WitnessUV;result.SamplePoint=leaves[0].WitnessPoint;
    double projectedGlobalLower=0;
    bool orthogonalProjection=false;
    const auto rootGap=AnalyticCertified::Subtract(Interval::Exact(result.UpperSquared),Interval::Exact(leaves[0].LowerSquared));
    if(!AnalyticCertified::Valid(rootGap))return {};
    const bool rootThresholdDecided=decisionThresholdSquared&&
        (leaves[0].LowerSquared>decisionThresholdSquared->Upper||result.UpperSquared<decisionThresholdSquared->Lower);
    const bool rootNeedsWork=!rootThresholdDecided&&
        !(rootGap.Upper<=requestedSquaredGap&&!decisionThresholdSquared);
    const auto availableWork=[&]()->int32
    {
        const uint64 spent=budget.Used-usedBefore;
        if(spent>=uint64(maximumNodesThisCall)||budget.ConsumptionFloor<0||budget.Remaining<=budget.ConsumptionFloor)return 0;
        const uint64 local=uint64(maximumNodesThisCall)-spent;
        const uint64 global=uint64(int64(budget.Remaining)-int64(budget.ConsumptionFloor));
        return int32(FMath::Min(local,global)); // Both are at most MAX_int32.
    };
    if(rootNeedsWork&&surface.DegreeU==5&&surface.DegreeV==5&&availableWork()>=3)
    {
        // One bounded predicate/root construction unit, paid even if the
        // optional coefficient-box gate rejects the optimization.
        if(!budget.Consume())return {};
        const int32 dropped=ProjectedDistanceDetail::ExtrusionAxis(surface);
        if(dropped!=INDEX_NONE)
        {
            const auto projected=ProjectedDistanceDetail::ProjectYZ(surface,dropped);
            const auto projectedCenter=ProjectedDistanceDetail::ProjectCenterToYZ(centerBounds,dropped);
            // Same global budget, same ORIGINAL local usedBefore. Reserve one
            // Bounds unit for the true 2D witness; never install/refill a floor.
            const int32 subCap=FMath::Min(256,availableWork()-1);
            if(subCap<=0)return {};
            const auto lowerOnly=BoundTensorDistanceSquaredIntervalCenter(projected,projectedCenter,
                budget,requestedSquaredGap,subCap,decisionThresholdSquared,midpointMemo);
            if(lowerOnly.Status==EDistanceBoundStatus::Unknown)return {};
            projectedGlobalLower=lowerOnly.LowerSquared;
            orthogonalProjection=dropped==1;
            // Discard the projected upper, point and status. Only this exact
            // validated UV on the ORIGINAL 2D family can improve the real upper.
            if(availableWork()<1||!budget.Consume())return {};
            const FVector2d uv(ProjectedDistanceDetail::ChooseU(surface,centerBounds,dropped),lowerOnly.SampleUV.Y);
            FIntervalPoint point;
            if(!ProjectedDistanceDetail::EvaluateFixedUV(surface,uv,point))return {};
            auto upper=Interval::Exact(0);
            for(int32 d=0;d<3;++d)upper=AnalyticCertified::Add(upper,
                AnalyticCertified::Square(AnalyticCertified::Subtract(centerBounds.Axis[d],point.Axis[d])));
            if(!AnalyticCertified::Valid(upper))return {};
            if(upper.Upper<result.UpperSquared)
            {result.UpperSquared=upper.Upper;result.SampleUV=uv;result.SamplePoint=point;}
        }
    }
    if(rootNeedsWork&&surface.DegreeU==0&&surface.DegreeV==5&&availableWork()>=36&&
        PlanarFamilyTimeShortcutControl::RuntimeEnabled())
    {
        // At most 32 constant-size units: four setup, eight triples, four
        // terminal hull/witness stages. Keep a finite 36-unit admission reserve.
        FTensorDistanceBounds convex;
        const auto status=StrongConvexCurveMinimum::TryBoundForDecision(surface,centerBounds,budget,
            availableWork(),convex,decisionThresholdSquared,requestedSquaredGap);
        if(status==StrongConvexCurveMinimum::EResult::Work||
            status==StrongConvexCurveMinimum::EResult::Arithmetic)return {};
        if(status==StrongConvexCurveMinimum::EResult::Bounded)
        {
            projectedGlobalLower=FMath::Max(projectedGlobalLower,convex.LowerSquared);
            if(convex.UpperSquared<result.UpperSquared)
            {
                result.UpperSquared=convex.UpperSquared;
                result.SampleUV=convex.SampleUV;result.SamplePoint=convex.SamplePoint;
            }
        }
    }
    // Keep an inconclusive distance as a valid bracket instead of using
    // the rest of the time search on one nearly tangent endpoint. No
    // unresolved threshold can become clear or inside at this local stop.
    const uint64 localLimit=uint64(orthogonalProjection?FMath::Min(512,maximumNodesThisCall):maximumNodesThisCall);
    for (;;)
    {
        const int32 best=minimum.PeekIndex();
        result.LowerSquared=FMath::Max(projectedGlobalLower,leaves[best].LowerSquared);
        if(result.LowerSquared>result.UpperSquared)return {};
        result.Status=EDistanceBoundStatus::Bounded;
        const auto gap=AnalyticCertified::Subtract(Interval::Exact(result.UpperSquared),Interval::Exact(result.LowerSquared));
        if (!AnalyticCertified::Valid(gap))return FTensorDistanceBounds();
        const bool thresholdDecided=decisionThresholdSquared&&(result.LowerSquared>decisionThresholdSquared->Upper||
            result.UpperSquared<decisionThresholdSquared->Lower);
        // Distance convergence alone cannot answer an unresolved radius threshold.
        // Keep the same nodes/shared budget and refine until a strict decision;
        // an exhausted/invalid attempt still leaves that decision unknown.
        if (gap.Upper<=requestedSquaredGap&&(!decisionThresholdSquared||thresholdDecided))
        {result.Status=EDistanceBoundStatus::Converged;break;}
        // Preserve Converged above when both facts hold; a threshold-only
        // early stop is Bounded, with the same universal lower/upper bounds.
        if(thresholdDecided)break;
        // When the projected lower and original-net upper already meet the
        // requested accuracy, retain that bounded result. If it straddles
        // the radius, the time search must still find a strict witness and
        // close the prefix; it cannot turn this enclosure into clear or hit.
        if(orthogonalProjection&&gap.Upper<=requestedSquaredGap)break;
        // A KKT predicate is new work, paid from the SAME global and local
        // allowance. A local cap stop retains valid Bounded values; an actual
        // Consume denial/invalid predicate remains terminal Unknown.
        if(budget.Used-usedBefore+1>localLimit)break;
        if(!budget.Consume())return FTensorDistanceBounds();
        const auto minimumClass=MinimumDetail::ClassifyMinimum(leaves[best],centerBounds);
        if(minimumClass==MinimumDetail::ECellMinimum::Unknown)return FTensorDistanceBounds();
        if(minimumClass==MinimumDetail::ECellMinimum::Excluded)
        {
            minimum.PopRoot(leaves); // Remove while the old key is intact.
            leaves[best].Net.Reset(); // Local dead slot; witness copies remain owned by result.
            if(minimum.Num()==0)return FTensorDistanceBounds(); // Never infer clear by vacuity.
            continue;
        }
        // Recheck AFTER KKT consumes: the old two-consume guard is stale now.
        if (budget.Remaining<2 || budget.Used-usedBefore+2>localLimit)break;
        const auto& node=leaves[best];
        if (node.DegreeU==0 && node.DegreeV==0)break;
        const bool alongU=Detail::ChooseTensorDistanceSplitU(node);
        Detail::FNode a,b;
        if (!Detail::Split(node,alongU,a,b))break;
        if (!budget.Consume() || !budget.Consume() || !Detail::Bounds(a,centerBounds,decisionThresholdSquared,&midpointScope,&proposal95) || !Detail::Bounds(b,centerBounds,decisionThresholdSquared,&midpointScope,&proposal95))return FTensorDistanceBounds();
        minimum.PopRoot(leaves); // Both new Bounds succeeded; old key is still intact.
        for (const auto* child:{&a,&b})
            if (child->UpperSquared<result.UpperSquared)
            {result.UpperSquared=child->UpperSquared;result.SampleUV=child->WitnessUV;result.SamplePoint=child->WitnessPoint;}
        leaves[best]=MoveTemp(a);leaves.Add(MoveTemp(b));
        minimum.PushIndex(best,leaves);minimum.PushIndex(leaves.Num()-1,leaves);
    }
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    return result;
}
}

#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Analytic::Certified::StrongConvexCurveMinimum
{
IAMSPEED_API uint64 ObservationCallsForTesting()
{return Observation::Rows.Calls.load(std::memory_order_relaxed);}
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "IAmSpeed/World/Simulation/FrameFloatingPointContext.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStrongConvexPropertiesCacheTest,
 "IAmSpeed.AnalyticWorld.TensorDistance.StrongConvexCurveDraft.PropertiesCache",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStrongConvexPropertiesCacheTest::RunTest(const FString&)
{
    using namespace Speed::Analytic::Certified;
    namespace Q=StrongConvexCurveMinimum;namespace B=::AnalyticCertified;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    FIntervalTensorSurface curve;curve.DegreeU=0;curve.DegreeV=5;
    for(int32 i=0;i<6;++i)curve.ControlPoints.Add(Detail::Point(FVector3d(4*i,i*(i-1),0)));
    const auto samePoint=[](const FIntervalPoint& a,const FIntervalPoint& b)
    {for(int32 d=0;d<3;++d)if(a.Axis[d].Known!=b.Axis[d].Known||B::Bits(a.Axis[d].Lower)!=B::Bits(b.Axis[d].Lower)||B::Bits(a.Axis[d].Upper)!=B::Bits(b.Axis[d].Upper))return false;return true;};
    const auto compare=[&](const FIntervalTensorSurface& source,const FIntervalPoint& center,int32 cap,int32 floor)
    {
        FDistanceBudget oldBudget{cap,7,floor},newBudget{cap,7,floor};FTensorDistanceBounds oldBound,newBound;Q::FInspection oldInspection,newInspection;
        const auto oldStatus=Q::TryBoundImpl(source,center,oldBudget,cap,oldBound,&oldInspection,false);
        const uint32 before=VectorGetControlRegister();
        const auto newStatus=Q::TryBoundImpl(source,center,newBudget,cap,newBound,&newInspection,true);
        return oldStatus==newStatus&&oldBudget.Remaining==newBudget.Remaining&&oldBudget.Used==newBudget.Used&&oldBudget.ConsumptionFloor==newBudget.ConsumptionFloor&&
            oldBound.Status==newBound.Status&&B::Bits(oldBound.LowerSquared)==B::Bits(newBound.LowerSquared)&&B::Bits(oldBound.UpperSquared)==B::Bits(newBound.UpperSquared)&&
            B::Bits(oldBound.SampleUV.X)==B::Bits(newBound.SampleUV.X)&&B::Bits(oldBound.SampleUV.Y)==B::Bits(newBound.SampleUV.Y)&&samePoint(oldBound.SamplePoint,newBound.SamplePoint)&&
            B::Bits(oldInspection.CurvatureLower)==B::Bits(newInspection.CurvatureLower)&&B::Bits(oldInspection.ParameterLower)==B::Bits(newInspection.ParameterLower)&&B::Bits(oldInspection.ParameterUpper)==B::Bits(newInspection.ParameterUpper)&&
            oldInspection.NewtonSteps==newInspection.NewtonSteps&&oldInspection.PossibleStart==newInspection.PossibleStart&&oldInspection.PossibleEnd==newInspection.PossibleEnd&&
            (VectorGetControlRegister()&~uint32(0x3f))==(before&~uint32(0x3f));
    };
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        bool ok=true;for(const FVector3d p:{FVector3d(9,6,3),FVector3d(-10,0,3),FVector3d(30,30,3),FVector3d(10,1000,0)})
        for(int32 cap:{0,1,2,3,4,8,24,100})for(int32 floor:{0,1})if(floor<=cap)
            ok&=compare(curve,Detail::Point(p),cap,floor);
        auto family=curve;for(auto& p:family.ControlPoints)p.Axis[2]={-.001,.001,true};
        auto center=Detail::Point(FVector3d(9,6,3));center.Axis[0]={8.99,9.01,true};ok&=compare(family,center,100,0);
        TestTrue(TEXT("cache preserves every output bit, all paid work, endpoint decisions and arithmetic controls"),ok);
        FTensorDistanceBounds result;Q::FInspection inspection;FDistanceBudget first{100};
        Q::TryBoundImpl(curve,center,first,100,result,&inspection,true);const uint64 hits=Q::CurvePropertyCache().Hits;
        FDistanceBudget second{100};Q::TryBoundImpl(curve,center,second,100,result,&inspection,true);
        TestTrue(TEXT("same coefficients and mode hit immutable properties"),Q::CurvePropertyCache().Hits==hits+1);
        auto changed=curve;changed.ControlPoints[2].Axis[1].Upper=B::FromBits(B::Bits(changed.ControlPoints[2].Axis[1].Upper)+1);
        const uint64 misses=Q::CurvePropertyCache().Misses;FDistanceBudget third{100};Q::TryBoundImpl(changed,center,third,100,result,&inspection,true);
        TestTrue(TEXT("one coefficient endpoint ULP invalidates properties"),Q::CurvePropertyCache().Misses==misses+1);
        auto malformed=curve;malformed.ControlPoints[2].Axis[1].Known=false;ok=compare(malformed,center,100,0);
        TestTrue(TEXT("invalid coefficient cannot reuse a valid cache entry"),ok);
        // All fixture endpoint words share the same six low bits. Their
        // geometric differences are carried only by the higher IEEE bits.
        // Exercise the real lookup after interleaving distinct curves, not
        // just the same-curve immediate hit covered above.
        const uint64 numeric=Speed::Analytic::RequireCurrentAnalyticNumericContext(
            Speed::Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
        bool occupied[64]{};int32 distinctSlots=0;TArray<Q::FCurve> interleaved;
        for(int32 j=0;j<32;++j)
        {
            Q::FCurve coefficients=curve.ControlPoints;
            for(int32 i=0;i<6;++i)coefficients[i].Axis[0]=B::Interval::Exact(4*i+64*j);
            Q::FCurvePropertyKey key;key.Numeric=numeric;int32 endpoint=0;
            for(const auto& p:coefficients)for(const auto& axis:p.Axis)
            {key.Endpoints[endpoint++]=B::Bits(axis.Lower);key.Endpoints[endpoint++]=B::Bits(axis.Upper);}
            for(uint64 word:key.Endpoints)TestEqual(TEXT("fixture differs only above the six slot bits"),word&63,uint64(0));
            const uint32 slot=Q::CurvePropertySlot(key);
            if(!occupied[slot]){occupied[slot]=true;++distinctSlots;if(interleaved.Num()<8)interleaved.Add(MoveTemp(coefficients));}
        }
        TestTrue(TEXT("high coefficient bits spread integer curves over multiple cache slots"),distinctSlots>=16);
        TestEqual(TEXT("eight distinct cache-slot curves selected"),interleaved.Num(),8);
        for(const auto& coefficients:interleaved)TestNotNull(TEXT("interleaved properties admitted"),Q::GetCurveProperties(coefficients,numeric));
        const uint64 interleavedHits=Q::CurvePropertyCache().Hits;
        for(const auto& coefficients:interleaved)TestNotNull(TEXT("interleaved properties retained"),Q::GetCurveProperties(coefficients,numeric));
        TestEqual(TEXT("every interleaved curve remains a cache hit"),Q::CurvePropertyCache().Hits,interleavedHits+uint64(interleaved.Num()));
    }
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStrongConvexAdaptiveThresholdTest,
 "IAmSpeed.AnalyticWorld.TensorDistance.StrongConvexCurveDraft.AdaptiveThreshold",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStrongConvexAdaptiveThresholdTest::RunTest(const FString&)
{
    FScopedCVar<int32> scalarOff(TEXT("p.IAmSpeed.AnalyticWorld.StrongConvexScalarProposal"),0,FName(TEXT("FStrongConvexAdaptiveThresholdTestBaseline")));
    using namespace Speed::Analytic::Certified;
    namespace Q=StrongConvexCurveMinimum;namespace B=::AnalyticCertified;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    // Exact Bernstein degree5 elevation of P(t)=(20t,20t^2,0).
    // Center=(9,6,3) has its unique minimum11 at t=1/2.
    FIntervalTensorSurface curve;curve.DegreeU=0;curve.DegreeV=5;
    for(int32 i=0;i<6;++i)curve.ControlPoints.Add(Detail::Point(FVector3d(4*i,i*(i-1),0)));
    const auto center=Detail::Point(FVector3d(9,6,3));
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        FDistanceBudget referenceBudget{100};FTensorDistanceBounds reference;Q::FInspection referenceInspection;
        TestTrue(TEXT("full reference minimum remains bounded"),Q::TryBoundImpl(curve,center,referenceBudget,100,reference,&referenceInspection,false)==Q::EResult::Bounded);
        for(double thresholdValue:{10.5,11.5})
        {
            const auto threshold=B::Interval::Exact(thresholdValue);
            FDistanceBudget budget{100};FTensorDistanceBounds bound;Q::FInspection inspection;
            TestTrue(TEXT("adaptive radius result bounded"),Q::TryBoundForDecision(curve,center,budget,36,bound,&threshold,0,&inspection)==Q::EResult::Bounded);
            TestTrue(TEXT("universal enclosure retains the exact minimum11"),bound.LowerSquared<=11&&bound.UpperSquared>=11);
            TestTrue(TEXT("strict radius decision made"),thresholdValue<11?bound.LowerSquared>threshold.Upper:bound.UpperSquared<threshold.Lower);
            TestEqual(TEXT("decision precedes all Newton restrictions"),inspection.NewtonSteps,0);
            TestEqual(TEXT("four setup and two decision work units"),budget.Used,uint64(6));
            TestTrue(TEXT("adaptive decision spends less than full Newton solve"),budget.Used<referenceBudget.Used);
            TestTrue(TEXT("witness is fixed original UV"),bound.SampleUV.X==0&&bound.SampleUV.Y==.5&&
                bound.SamplePoint.Axis[0].Lower<=10&&bound.SamplePoint.Axis[0].Upper>=10&&
                bound.SamplePoint.Axis[1].Lower<=5&&bound.SamplePoint.Axis[1].Upper>=5&&
                bound.SamplePoint.Axis[2].Lower<=0&&bound.SamplePoint.Axis[2].Upper>=0);
        }
        {
            const auto threshold=B::Interval::Exact(11);
            FDistanceBudget budget{100};FTensorDistanceBounds bound;Q::FInspection inspection;
            TestTrue(TEXT("equality remains bounded without a false strict decision"),Q::TryBoundForDecision(curve,center,budget,36,bound,&threshold,0,&inspection)==Q::EResult::Bounded&&
                bound.LowerSquared<=11&&bound.UpperSquared>=11&&!Q::DecisionSatisfied(bound,&threshold,0));
            TestTrue(TEXT("adaptive finite work bound"),budget.Used<=32);
        }
        {
            FDistanceBudget budget{100};FTensorDistanceBounds bound;Q::FInspection inspection;
            TestTrue(TEXT("requested gap can stop the private route"),Q::TryBoundForDecision(curve,center,budget,36,bound,nullptr,1.e-8,&inspection)==Q::EResult::Bounded&&
                Q::DecisionSatisfied(bound,nullptr,1.e-8)&&inspection.NewtonSteps==0);
        }
        {
            // Every fixed realization is an exact line in XY and stays inside
            // the coefficient Z boxes. Centers range over a whole rectangle;
            // no single center or coefficient realization is substituted.
            auto family=curve;for(int32 i=0;i<6;++i)
            {family.ControlPoints[i]=Detail::Point(FVector3d(4*i,0,0));family.ControlPoints[i].Axis[2]={-.001,.001,true};}
            auto familyCenter=Detail::Point(FVector3d(10,3.5,0));familyCenter.Axis[0]={8,12,true};familyCenter.Axis[1]={3,4,true};
            for(double thresholdValue:{4.,21.})
            {
                const auto threshold=B::Interval::Exact(thresholdValue);FDistanceBudget budget{100};FTensorDistanceBounds bound;
                TestTrue(TEXT("interval coefficient and center families admit an exact decision"),Q::TryBoundForDecision(family,familyCenter,budget,36,bound,&threshold,0)==Q::EResult::Bounded&&
                    Q::DecisionSatisfied(bound,&threshold,0)&&bound.LowerSquared<=9&&bound.UpperSquared>=16);
            }
        }
        {
            const auto threshold=B::Interval::Exact(10.5);FTensorDistanceBounds bound;
            FDistanceBudget local{100};TestTrue(TEXT("local limit is charged without refund"),Q::TryBoundForDecision(curve,center,local,5,bound,&threshold,0)==Q::EResult::LocalLimit&&
                local.Used==5&&local.Remaining==95&&bound.Status==EDistanceBoundStatus::Unknown);
            FDistanceBudget global{6,0,1};TestTrue(TEXT("global floor denies decision proof without refund"),Q::TryBoundForDecision(curve,center,global,36,bound,&threshold,0)==Q::EResult::Work&&
                global.Used==5&&global.Remaining==1&&global.ConsumptionFloor==1&&bound.Status==EDistanceBoundStatus::Unknown);
        }
        {
            FTensorDistanceBounds accumulator;
            const bool accepted=Q::AccumulateDecisionBound(Detail::Point(FVector3d(10,5,0)),center,.5,B::Interval::Exact(1.e308),1.,accumulator);
            TestTrue(TEXT("overflow in optional strong bound publishes no partial result"),!accepted&&accumulator.Status==EDistanceBoundStatus::Unknown);
        }
    }
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStrongConvexFixedPointCacheTest,
 "IAmSpeed.AnalyticWorld.TensorDistance.StrongConvexCurveDraft.FixedPointPropertiesCache",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStrongConvexFixedPointCacheTest::RunTest(const FString&)
{
    using namespace Speed::Analytic::Certified;
    namespace Q=StrongConvexCurveMinimum;namespace B=::AnalyticCertified;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    FIntervalTensorSurface curve;curve.DegreeU=0;curve.DegreeV=5;
    for(int32 i=0;i<6;++i)curve.ControlPoints.Add(Detail::Point(FVector3d(4*i,i*(i-1),0)));
    const auto sameInterval=[](const B::Interval& a,const B::Interval& b)
    {return a.Known==b.Known&&B::Bits(a.Lower)==B::Bits(b.Lower)&&B::Bits(a.Upper)==B::Bits(b.Upper);};
    const auto samePoint=[&](const FIntervalPoint& a,const FIntervalPoint& b)
    {for(int32 d=0;d<3;++d)if(!sameInterval(a.Axis[d],b.Axis[d]))return false;return true;};
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        Q::FCurveProperties properties;TestTrue(TEXT("immutable properties valid"),Q::BuildCurveProperties(curve.ControlPoints,properties));
        TestEqual(TEXT("fixed evaluations remain lazy after property construction"),properties.FixedReady,uint8(0));
        bool equal=true;
        for(int32 centerIndex=0;centerIndex<8;++centerIndex)
        {
            auto center=Detail::Point(FVector3d(9+centerIndex,6-centerIndex,3));
            center.Axis[0].Upper+=.125;
            for(double t:{0.,.5,1.,.25,.75})
            {
                B::Interval direct,cached;FIntervalPoint directPoint,cachedPoint;
                const bool a=Q::Gradient(curve.ControlPoints,properties.D,center,t,direct,&directPoint);
                const bool b=Q::GradientWithProperties(curve.ControlPoints,properties,center,t,cached,&cachedPoint);
                equal&=a==b&&sameInterval(direct,cached)&&samePoint(directPoint,cachedPoint);
            }
        }
        TestTrue(TEXT("fixed and nonfixed gradients preserve every bit for changing interval centers"),equal);
        TestEqual(TEXT("three fixed point pairs built once despite changing centers"),properties.FixedPointBuildsForTesting,uint32(3));
        TestEqual(TEXT("all three fixed memo slots ready"),properties.FixedReady,uint8(7));
        {
            auto replacement=curve;replacement.ControlPoints[2].Axis[0].Upper=B::FromBits(B::Bits(replacement.ControlPoints[2].Axis[0].Upper)+1);
            TestTrue(TEXT("replacement properties valid"),Q::BuildCurveProperties(replacement.ControlPoints,properties));
            TestEqual(TEXT("replacement discards prior geometry points"),properties.FixedReady,uint8(0));
            TestEqual(TEXT("replacement resets point build count"),properties.FixedPointBuildsForTesting,uint32(0));
            B::Interval direct,cached;FIntervalPoint directPoint,cachedPoint;const auto center=Detail::Point(FVector3d(9,6,3));
            TestTrue(TEXT("replacement midpoint uses new exact coefficients"),
                Q::Gradient(replacement.ControlPoints,properties.D,center,.5,direct,&directPoint)&&
                Q::GradientWithProperties(replacement.ControlPoints,properties,center,.5,cached,&cachedPoint)&&
                sameInterval(direct,cached)&&samePoint(directPoint,cachedPoint)&&properties.FixedPointBuildsForTesting==1);
        }
        {
            Q::FCurveProperties failing;
            TestTrue(TEXT("failure fixture starts from valid properties"),Q::BuildCurveProperties(curve.ControlPoints,failing));
            // Fault injection: C evaluates successfully, D fails afterwards.
            // The partially computed pair must never become a reusable memo.
            failing.D[1].Axis[0].Known=false;
            B::Interval gradient;FIntervalPoint point;
            TestTrue(TEXT("failed derivative evaluation publishes no fixed-point pair"),
                !Q::GradientWithProperties(curve.ControlPoints,failing,Detail::Point(FVector3d(9,6,3)),.5,gradient,&point)&&
                failing.FixedReady==0&&failing.FixedPointBuildsForTesting==0);
        }
        equal=true;
        for(double thresholdValue:{10.5,11.,11.5})for(int32 cap:{0,1,2,3,4,5,6,12,24,36,100})
        {
            const auto threshold=B::Interval::Exact(thresholdValue);const auto center=Detail::Point(FVector3d(9,6,3));
            FDistanceBudget directBudget{100,7,3},cachedBudget=directBudget;
            FTensorDistanceBounds direct,cached;Q::FInspection directInspection,cachedInspection;
            const auto a=Q::TryBoundImpl(curve,center,directBudget,cap,direct,&directInspection,true,&threshold,0,false);
            const uint32 controls=VectorGetControlRegister();
            const auto b=Q::TryBoundImpl(curve,center,cachedBudget,cap,cached,&cachedInspection,true,&threshold,0,true);
            equal&=a==b&&directBudget.Remaining==cachedBudget.Remaining&&directBudget.Used==cachedBudget.Used&&directBudget.ConsumptionFloor==cachedBudget.ConsumptionFloor&&
                direct.Status==cached.Status&&B::Bits(direct.LowerSquared)==B::Bits(cached.LowerSquared)&&B::Bits(direct.UpperSquared)==B::Bits(cached.UpperSquared)&&
                B::Bits(direct.SampleUV.X)==B::Bits(cached.SampleUV.X)&&B::Bits(direct.SampleUV.Y)==B::Bits(cached.SampleUV.Y)&&samePoint(direct.SamplePoint,cached.SamplePoint)&&
                B::Bits(directInspection.CurvatureLower)==B::Bits(cachedInspection.CurvatureLower)&&
                B::Bits(directInspection.ParameterLower)==B::Bits(cachedInspection.ParameterLower)&&B::Bits(directInspection.ParameterUpper)==B::Bits(cachedInspection.ParameterUpper)&&
                directInspection.NewtonSteps==cachedInspection.NewtonSteps&&directInspection.PossibleStart==cachedInspection.PossibleStart&&directInspection.PossibleEnd==cachedInspection.PossibleEnd&&
                (VectorGetControlRegister()&~uint32(0x3f))==(controls&~uint32(0x3f));
        }
        TestTrue(TEXT("whole helper preserves result bits, inspection, budget and arithmetic controls"),equal);
    }
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/ScopedCVar.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStrongConvexRetainedDecisionRouteTest,
 "IAmSpeed.AnalyticWorld.TensorDistance.StrongConvexCurveDraft.RetainedDecisionRoute",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStrongConvexRetainedDecisionRouteTest::RunTest(const FString&)
{
    FScopedCVar<int32> scalarOff(TEXT("p.IAmSpeed.AnalyticWorld.StrongConvexScalarProposal"),0,FName(TEXT("FStrongConvexRetainedDecisionRouteTestBaseline")));
    using namespace Speed::Analytic::Certified;namespace Q=StrongConvexCurveMinimum;namespace B=::AnalyticCertified;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.PlanarFamilyTimeShortcut"),1,FName(TEXT("RetainedDecisionRoute")));
    FScopedCVar<int32> profile(TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"),1,FName(TEXT("RetainedDecisionRouteProfile")));
    FIntervalTensorSurface curve;curve.DegreeU=0;curve.DegreeV=5;
    for(int32 i=0;i<6;++i)curve.ControlPoints.Add(Detail::Point(FVector3d(4*i,i*(i-1),0)));
    const auto center=Detail::Point(FVector3d(9,6,3));const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        for(double value:{10.5,11.5})
        {
            const auto threshold=B::Interval::Exact(value);FDistanceBudget budget{2048};
            const uint64 stops=Q::Observation::Rows.DecisionStops.load();
            FRetainedTensorMinimum minimum(curve,center,budget,0,128,&threshold);const auto result=minimum.Advance();
            TestTrue(TEXT("retained uses helper only when the root threshold is unresolved"),Q::Observation::Rows.DecisionStops.load()==stops+(value<11?1:0)&&
                result.Status!=EDistanceBoundStatus::Unknown&&result.LowerSquared<=11&&result.UpperSquared>=11&&
                (value<11?result.LowerSquared>threshold.Upper:result.UpperSquared<threshold.Lower));
            TestTrue(TEXT("retained shared and local budgets count the early helper work"),budget.Used==minimum.GetLocalSpentForInspection()&&budget.Used<20&&
                budget.Used+uint64(budget.Remaining)==2048);
        }
        const auto threshold=B::Interval::Exact(10.5);FDistanceBudget budget{2048};const uint64 calls=Q::Observation::Rows.Calls.load();
        FRetainedTensorMinimum coarse(curve,center,budget,0,1,&threshold);const auto result=coarse.Advance();
        TestTrue(TEXT("retained local1 has no new helper admission"),Q::Observation::Rows.Calls.load()==calls&&budget.Used==1&&
            result.Status!=EDistanceBoundStatus::Unknown&&result.LowerSquared<=11&&result.UpperSquared>=11);
    }
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStrongConvexScalarProposalTest,
 "IAmSpeed.AnalyticWorld.TensorDistance.StrongConvexCurveDraft.ScalarProposal",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStrongConvexScalarProposalTest::RunTest(const FString&)
{
    FScopedCVar<int32> noJoint(TEXT("p.IAmSpeed.AnalyticWorld.StrongConvexJointDecision"),0,FName(TEXT("ScalarProposalHistoricalCost")));
    using namespace Speed::Analytic::Certified;namespace Q=StrongConvexCurveMinimum;namespace B=::AnalyticCertified;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.StrongConvexScalarProposal"),1,FName(TEXT("ScalarProposalTest")));
    FScopedCVar<int32> profile(TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"),1,FName(TEXT("ScalarProposalTestProfile")));
    FIntervalTensorSurface curve;curve.DegreeU=0;curve.DegreeV=5;
    for(int32 i=0;i<6;++i)curve.ControlPoints.Add(Detail::Point(FVector3d(4*i,i*(i-1),0)));
    const auto samePoint=[](const FIntervalPoint& a,const FIntervalPoint& b)
    {for(int32 k=0;k<3;++k)if(a.Axis[k].Known!=b.Axis[k].Known||B::Bits(a.Axis[k].Lower)!=B::Bits(b.Axis[k].Lower)||B::Bits(a.Axis[k].Upper)!=B::Bits(b.Axis[k].Upper))return false;return true;};
    const auto sameBound=[&](const FTensorDistanceBounds& a,const FTensorDistanceBounds& b)
    {return a.Status==b.Status&&B::Bits(a.LowerSquared)==B::Bits(b.LowerSquared)&&B::Bits(a.UpperSquared)==B::Bits(b.UpperSquared)&&
        B::Bits(a.SampleUV.X)==B::Bits(b.SampleUV.X)&&B::Bits(a.SampleUV.Y)==B::Bits(b.SampleUV.Y)&&samePoint(a.SamplePoint,b.SamplePoint);};
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        for(double expectedParameter:{.25,.625})
        {
            // P(t)=(20t,20t^2,0); center offset (-2t,1,3)
            // is perpendicular to P'(t), so min f=4t^2+10.
            const double exactMinimum=4*expectedParameter*expectedParameter+10;
            const auto center=Detail::Point(FVector3d(18*expectedParameter,20*expectedParameter*expectedParameter+1,3));
            const auto numeric=Speed::Analytic::RequireCurrentAnalyticNumericContext(Speed::Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
            const auto properties=Q::GetCurveProperties(curve.ControlPoints,numeric.ControlFingerprint());
            double proposed=0;int32 steps=0;
            TestTrue(TEXT("scalar proposal remains a finite bounded heuristic"),properties&&Q::ProposeScalarMinimum(curve.ControlPoints,*properties,center,proposed,steps)&&
                proposed>=0&&proposed<=1&&steps>0&&steps<=5&&FMath::Abs(proposed-expectedParameter)<1.e-8);
            for(double offset:{-.125,.125})
            {
                const auto threshold=B::Interval::Exact(exactMinimum+offset);FDistanceBudget budget{128};FTensorDistanceBounds bound;Q::FInspection inspection;
                const uint64 stops=Q::Observation::Rows.ScalarStops.load();const uint32 controls=VectorGetControlRegister();
                TestTrue(TEXT("exact validation accepts a strict scalar-proposed decision"),Q::TryBoundForDecision(curve,center,budget,128,bound,&threshold,0,&inspection)==Q::EResult::Bounded&&
                    bound.LowerSquared<=exactMinimum&&bound.UpperSquared>=exactMinimum&&
                    (offset<0?bound.LowerSquared>threshold.Upper:bound.UpperSquared<threshold.Lower)&&
                    inspection.NewtonSteps==0&&budget.Used==7&&Q::Observation::Rows.ScalarStops.load()==stops+1);
                TestTrue(TEXT("proposal never changes arithmetic controls"),(VectorGetControlRegister()&~uint32(0x3f))==(controls&~uint32(0x3f)));
                const double v=bound.SampleUV.Y;
                // The witness enclosure is evaluated from the original net;
                // its original parameter, not the scalar curve value, is kept.
                FIntervalPoint originalPoint;TestTrue(TEXT("original interval-net witness is retained"),ProjectedDistanceDetail::EvaluateFixedUV(curve,FVector2d(0,v),originalPoint)&&
                    bound.SampleUV.X==0&&v>=0&&v<=1&&
                    bound.SamplePoint.Axis[0].Lower<=originalPoint.Axis[0].Upper&&bound.SamplePoint.Axis[0].Upper>=originalPoint.Axis[0].Lower&&
                    bound.SamplePoint.Axis[1].Lower<=originalPoint.Axis[1].Upper&&bound.SamplePoint.Axis[1].Upper>=originalPoint.Axis[1].Lower);
            }
            {
                const auto equality=B::Interval::Exact(exactMinimum);FDistanceBudget budget{128};FTensorDistanceBounds bound;Q::FInspection inspection;
                TestTrue(TEXT("approximate root cannot decide an exact equality"),Q::TryBoundForDecision(curve,center,budget,128,bound,&equality,0,&inspection)==Q::EResult::Bounded&&
                    bound.LowerSquared<=exactMinimum&&bound.UpperSquared>=exactMinimum&&!Q::DecisionSatisfied(bound,&equality,0)&&budget.Used<=34);
            }
        }
        {
            const auto center=Detail::Point(FVector3d(4.5,2.25,3));const auto threshold=B::Interval::Exact(10.);
            const auto numeric=Speed::Analytic::RequireCurrentAnalyticNumericContext(Speed::Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
            const auto properties=Q::GetCurveProperties(curve.ControlPoints,numeric.ControlFingerprint());
            FDistanceBudget baselineBudget{128};FTensorDistanceBounds baseline;Q::FInspection baselineInspection;
            const auto baselineStatus=Q::TryBoundImpl(curve,center,baselineBudget,128,baseline,&baselineInspection,true,&threshold,0,true,false);
            properties->ScalarTried=true;properties->ScalarReady=false; // Fault injection: reject optional proposer only.
            FDistanceBudget fallbackBudget{128};FTensorDistanceBounds fallback;Q::FInspection fallbackInspection;
            const auto fallbackStatus=Q::TryBoundImpl(curve,center,fallbackBudget,128,fallback,&fallbackInspection,true,&threshold,0,true,true);
            TestTrue(TEXT("invalid proposer preserves literal fallback results and charged extra attempt"),baselineStatus==fallbackStatus&&sameBound(baseline,fallback)&&
                fallbackBudget.Used==baselineBudget.Used+1&&fallbackBudget.Remaining+1==baselineBudget.Remaining&&
                fallbackInspection.NewtonSteps==baselineInspection.NewtonSteps&&
                B::Bits(fallbackInspection.ParameterLower)==B::Bits(baselineInspection.ParameterLower)&&B::Bits(fallbackInspection.ParameterUpper)==B::Bits(baselineInspection.ParameterUpper));
            properties->ScalarTried=false;properties->ScalarReady=false;
            FDistanceBudget directBudget{128},publicBudget{128};FTensorDistanceBounds direct,publicBound;Q::FInspection directInspection,publicInspection;
            const auto directStatus=Q::TryBoundImpl(curve,center,directBudget,128,direct,&directInspection,true);
            const auto publicStatus=Q::TryBound(curve,center,publicBudget,128,publicBound,&publicInspection);
            TestTrue(TEXT("old public TryBound contract ignores scalar prototype"),directStatus==publicStatus&&sameBound(direct,publicBound)&&directBudget.Used==publicBudget.Used);
            FDistanceBudget smallA{128},smallB{128};FTensorDistanceBounds a,b;
            const auto sa=Q::TryBoundImpl(curve,center,smallA,24,a,nullptr,true,&threshold,0,true,false);
            const auto sb=Q::TryBoundImpl(curve,center,smallB,24,b,nullptr,true,&threshold,0,true,true);
            TestTrue(TEXT("small local caps do not admit a proposal without its full reserve"),sa==sb&&sameBound(a,b)&&smallA.Used==smallB.Used&&smallA.Remaining==smallB.Remaining);
        }
        {
            auto family=curve;for(auto& point:family.ControlPoints)for(int32 axis=0;axis<2;++axis)
            {point.Axis[axis].Lower-=1.e-6;point.Axis[axis].Upper+=1.e-6;}
            auto center=Detail::Point(FVector3d(4.5,2.25,3));center.Axis[1]={2.249,2.251,true};
            const auto threshold=B::Interval::Exact(10.);FDistanceBudget budget{128};FTensorDistanceBounds bound;
            TestTrue(TEXT("nominal scalar family does not replace interval geometry or centers"),Q::TryBoundForDecision(family,center,budget,128,bound,&threshold,0)==Q::EResult::Bounded&&
                bound.LowerSquared<=10.25&&bound.UpperSquared>=10.25&&bound.LowerSquared>threshold.Upper);
        }
        {
            FIntervalTensorSurface line;line.DegreeU=0;line.DegreeV=5;for(int32 i=0;i<6;++i)line.ControlPoints.Add(Detail::Point(FVector3d(i,0,0)));
            for(double x:{-2.,7.})
            {
                const auto center=Detail::Point(FVector3d(x,0,3));const auto threshold=B::Interval::Exact(12.);
                FDistanceBudget budget{128};FTensorDistanceBounds bound;Q::FInspection inspection;
                TestTrue(TEXT("clamped proposal keeps the constrained endpoint minimum"),Q::TryBoundForDecision(line,center,budget,128,bound,&threshold,0,&inspection)==Q::EResult::Bounded&&
                    bound.LowerSquared<=13&&bound.UpperSquared>=13&&bound.LowerSquared>12&&
                    (x<0?inspection.PossibleStart:inspection.PossibleEnd));
            }
        }
    }
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}
#endif


// Exact optional first-contact seed for one projected quintic and a fixed linear
// path. The original generic time frontier remains the only continuation engine.
namespace Speed::Analytic::Certified
{
namespace MonotoneCurveTimeControl
{
static TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.MonotoneCurveTimeSeed"),0,
    TEXT("Default OFF. Exact strong-convex/monotone-time bracket seed; original path, UV and contact consumers unchanged."),ECVF_Default);
bool RuntimeEnabled(){return Enabled.GetValueOnAnyThread()==1;}
static TAutoConsoleVariable<int32> DyadicEnabled(TEXT("p.IAmSpeed.AnalyticWorld.MonotoneCurveDyadicHint"),0,
    TEXT("Default OFF. Use a certified monotone bracket only to skip bounds before inevitable original dyadic subdivisions; final bounds/UV remain on the old frontier."),ECVF_Default);
bool DyadicHintEnabled(){return DyadicEnabled.GetValueOnAnyThread()==1;}
struct FObservation
{
    std::atomic<uint64> Calls{0},Finished{0},Ticks{0},Work{0},MinimumCalls{0},TimeIterations{0},InsideValidations{0},Results[4]{};
    std::atomic<uint64> DyadicSearches{0},DyadicProved{0},DyadicNodes{0},DyadicSkips{0};
};
static FObservation Observation;
void RecordDyadicHint(uint64 nodes,uint64 skips,bool proved)
{
    if(!StrongConvexCurveMinimum::Observation::Enabled())return;
    ++Observation.DyadicSearches;if(proved)++Observation.DyadicProved;
    Observation.DyadicNodes.fetch_add(nodes);Observation.DyadicSkips.fetch_add(skips);
}
void PublishObservation()
{
    const auto& o=Observation;
    UE_LOG(LogTemp,Display,TEXT("[MonotoneCurveTimeSeed] Scope=ProgramPrefixAtPreExit RequiresExternalDrain=1 NoPASS=1 Calls=%llu Finished=%llu Ticks=%llu SecondsPerCycle=%.17g Work=%llu NotApplicable=%llu Proved=%llu WorkFailure=%llu Arithmetic=%llu MinimumCalls=%llu TimeIterations=%llu InsideValidations=%llu"),
        o.Calls.load(),o.Finished.load(),o.Ticks.load(),FPlatformTime::GetSecondsPerCycle64(),o.Work.load(),o.Results[0].load(),o.Results[1].load(),o.Results[2].load(),o.Results[3].load(),o.MinimumCalls.load(),o.TimeIterations.load(),o.InsideValidations.load());
    UE_LOG(LogTemp,Display,TEXT("[MonotoneCurveDyadicHint] Scope=ClosedSearchesAtProgramPrefix NoPASS=1 Searches=%llu Proved=%llu Nodes=%llu SkippedNodeBounds=%llu"),
        o.DyadicSearches.load(),o.DyadicProved.load(),o.DyadicNodes.load(),o.DyadicSkips.load());
}
}
namespace TimeDetail
{
namespace MonotoneSeedDetail
{
namespace A=::AnalyticCertified;namespace S=StrongConvexCurveMinimum;
using E=EMonotoneCurveTimeSeed;
struct FWork
{
    FDistanceBudget& Budget;uint64 Initial;int32 Maximum;uint64 Numeric;E Failure=E::NotApplicable;
    E Unavailable() const
    {
        if(Budget.ConsumptionFloor<0||Budget.Remaining<Budget.ConsumptionFloor||Budget.Used<Initial)return E::Work;
        const auto current=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        return current.Status==EAnalyticNumericStatus::Complete&&current.ControlFingerprint()==Numeric?E::NotApplicable:E::Arithmetic;
    }
    bool Has(int32 n) const
    {return n>=0&&Budget.Used>=Initial&&Budget.Used-Initial<=uint64(Maximum)&&
        uint64(n)<=uint64(Maximum)-(Budget.Used-Initial)&&Budget.ConsumptionFloor>=0&&
        int64(Budget.Remaining)-Budget.ConsumptionFloor>=n&&Budget.Used<=MAX_uint64-uint64(n);}
    bool Pay()
    {if(!Has(1))return false;if(!Budget.Consume()){Failure=E::Work;return false;}return true;}
    bool Minimum(const FIntervalTensorSurface& surface,const FIntervalPoint& center,
        FTensorDistanceBounds& out,FMonotoneCurveTimeInspection& seen,S::FInspection* inspect=nullptr,
        const Interval* threshold=nullptr,double squaredGap=0,bool adaptive=false)
    {
        const int32 reserve=adaptive?36:24;if(!Has(reserve))return false;
        ++seen.MinimumCalls;
        const auto status=adaptive?S::TryBoundForDecision(surface,center,Budget,reserve,out,threshold,squaredGap,inspect):
            S::TryBound(surface,center,Budget,reserve,out,inspect);
        if(status==S::EResult::Bounded)return true;
        if(status==S::EResult::Work)Failure=E::Work;
        if(status==S::EResult::Arithmetic)Failure=Unavailable();
        return false;
    }
};
inline bool PointAt(const FIntervalBezierCenterPath& path,double t,FIntervalPoint& out)
{
    if(!A::Finite(t)||t<0||t>1)return false;
    out=t==0?path.ControlPoints[0]:t==1?path.ControlPoints[1]:
        ProjectedDistanceDetail::LerpPoint(path.ControlPoints[0],path.ControlPoints[1],Interval::Exact(t));
    return Detail::ValidPoint(out);
}
inline bool TimeDerivative(const S::FCurve& possibleMinimizers,const FIntervalPoint& velocity,
    const FIntervalBezierCenterPath& path,double lo,double hi,Interval& out)
{
    FIntervalPoint endpoints[2];if(!PointAt(path,lo,endpoints[0])||!PointAt(path,hi,endpoints[1]))return false;
    bool have=false;Interval dot;
    // For every FIXED curve/path realization, this linear functional is inside
    // the Bernstein control hull in v and the two endpoint values in time.
    // Independent interval boxes merely widen that same universal bound.
    for(const auto& cp:possibleMinimizers)for(const auto& center:endpoints)
    {
        auto value=Interval::Exact(0);
        for(int32 d=0;d<3;++d)value=A::Add(value,A::Multiply(A::Subtract(cp.Axis[d],center.Axis[d]),velocity.Axis[d]));
        if(!A::Valid(value))return false;
        if(!have){dot=value;have=true;}else{dot.Lower=FMath::Min(dot.Lower,value.Lower);dot.Upper=FMath::Max(dot.Upper,value.Upper);}
    }
    out=A::Multiply(Interval::Exact(-2),dot);return have&&A::Valid(out);
}
static E Run(TConstArrayView<FIntervalTensorSurface> surfaces,const FIntervalBezierCenterPath& path,double radius,
    FDistanceBudget& budget,double gap,int32 maximumWork,FMonotoneCurveTimeSeed& out,FMonotoneCurveTimeInspection& seen)
{
    out={};
    if(surfaces.Num()!=1||surfaces[0].DegreeU!=0||surfaces[0].DegreeV!=5||surfaces[0].ControlPoints.Num()!=6||
        path.Degree!=1||path.ControlPoints.Num()!=2||!NonnegativeFinite(radius)||!(radius>0)||
        !NonnegativeFinite(gap)||!(gap>0)||maximumWork<768)return E::NotApplicable;
    const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=EAnalyticNumericStatus::Complete||!CenterDetail::Valid(path))return E::Arithmetic;
    for(const auto& cp:surfaces[0].ControlPoints)if(!Detail::ValidPoint(cp))return E::Arithmetic;
    // Current wheel paths are literal fixed endpoint binary64s. Do not extend
    // this optional monotonicity theorem to an independently uncertain path.
    for(const auto& p:path.ControlPoints)for(const auto& axis:p.Axis)
        if(!axis.Known||A::Bits(axis.Lower)!=A::Bits(axis.Upper))return E::NotApplicable;
    if(budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor||budget.Used==MAX_uint64)return E::Work;
    FWork work{budget,budget.Used,768,numeric.ControlFingerprint()};if(!work.Has(768))return E::NotApplicable;
    const auto r2=A::Square(Interval::Exact(radius));if(!A::Valid(r2))return work.Unavailable();
    const auto& surface=surfaces[0];FTensorDistanceBounds initial;
    if(!work.Minimum(surface,path.ControlPoints[0],initial,seen,nullptr,&r2,0,true))return work.Failure;
    if(!(initial.LowerSquared>r2.Upper))return E::NotApplicable;
    S::FCurve original;for(const auto& cp:surface.ControlPoints)original.Add(cp);
    FIntervalPoint velocity;
    for(int32 d=0;d<3;++d)velocity.Axis[d]=A::Subtract(path.ControlPoints[1].Axis[d],path.ControlPoints[0].Axis[d]);
    if(!Detail::ValidPoint(velocity))return work.Unavailable();
    double lo=0,hi=1;S::FCurve minimizerCurve;FTensorDistanceBounds inside;bool admitted=false;
    for(double prefix:{1.,.75,.5,.25})
    {
        ++seen.PrefixAttempts;if(!work.Pay())return work.Failure;
        FIntervalPoint end;if(!PointAt(path,prefix,end))return work.Unavailable();
        FTensorDistanceBounds endpoint;
        if(!work.Minimum(surface,end,endpoint,seen,nullptr,&r2,0,true))
        {if(work.Failure==E::NotApplicable)continue;return work.Failure;}
        ++seen.InsideValidations;
        if(!(endpoint.UpperSquared<r2.Lower))continue;
        const auto centerHull=Hull(path.ControlPoints[0],end);
        S::FInspection allMinimizers;FTensorDistanceBounds unused;
        if(!work.Minimum(surface,centerHull,unused,seen,&allMinimizers))
        {if(work.Failure==E::NotApplicable)continue;return work.Failure;}
        if(!(allMinimizers.CurvatureLower>0)||allMinimizers.ParameterLower<0||allMinimizers.ParameterUpper>1||
            allMinimizers.ParameterLower>allMinimizers.ParameterUpper)return E::Arithmetic;
        if(!work.Pay()||!S::Restrict(original,allMinimizers.ParameterLower,allMinimizers.ParameterUpper,minimizerCurve))return work.Failure==E::Work?E::Work:work.Unavailable();
        Interval derivative;if(!TimeDerivative(minimizerCurve,velocity,path,0,prefix,derivative))return work.Unavailable();
        if(!(derivative.Upper<0))continue;
        // Strong convexity is uniform over the entire center hull. The retained
        // V domain includes EVERY constrained minimizer, including both endpoint
        // KKT possibilities. Danskin's derivative, also at a constrained endpoint,
        // lies in this strictly negative time-derivative interval. Thus each
        // represented fixed geometry has exactly one zero before this inside B.
        hi=prefix;inside=endpoint;admitted=true;
        seen.GlobalCurvatureLower=allMinimizers.CurvatureLower;
        seen.ParameterLower=allMinimizers.ParameterLower;seen.ParameterUpper=allMinimizers.ParameterUpper;break;
    }
    if(!admitted)return E::NotApplicable;
    double proposal=.5*hi;
    for(int32 iteration=0;iteration<6;++iteration)
    {
        const auto width=A::Subtract(Interval::Exact(hi),Interval::Exact(lo));if(!A::Valid(width))return work.Unavailable();
        if(width.Upper<=gap)break;
        if(!work.Has(76))break; // retain the valid narrowed seed; no account reset
        const double m=proposal>lo&&proposal<hi?proposal:.5*lo+.5*hi;
        if(!(m>lo&&m<hi))break;
        FIntervalPoint center;if(!work.Pay()||!PointAt(path,m,center))return work.Failure==E::Work?E::Work:work.Unavailable();
        Interval derivative;if(!work.Pay()||!TimeDerivative(minimizerCurve,velocity,path,lo,hi,derivative))return work.Failure==E::Work?E::Work:work.Unavailable();
        if(!(derivative.Upper<0))return work.Unavailable(); // outward reevaluation may be less informative
        // Ask only for the squared-distance accuracy needed by this time target.
        // Every returned enclosure remains universal; this is not a tolerance change.
        const auto valueGap=A::Multiply(A::Multiply(Interval::Exact(gap),Interval::Exact(-derivative.Upper)),Interval::Exact(.0625));
        const double squaredGap=A::Valid(valueGap)?FMath::Max(0.,valueGap.Lower):0.;
        FTensorDistanceBounds distance;if(!work.Minimum(surface,center,distance,seen,nullptr,nullptr,squaredGap,true))return work.Failure;
        const auto value=A::Subtract({distance.LowerSquared,distance.UpperSquared,true},r2);
        Interval quotient;if(!A::Valid(value)||!S::DividePositive(value,A::Negate(derivative),quotient))return work.Unavailable();
        const auto necessary=A::Add(Interval::Exact(m),quotient);if(!A::Valid(necessary))return work.Unavailable();
        const double rootLo=FMath::Max(lo,necessary.Lower),rootHi=FMath::Min(hi,necessary.Upper);
        if(rootLo>rootHi)return E::Arithmetic;
        const double oldLo=lo,oldHi=hi;
        // Keep the seeded prefix strictly clear, including its endpoint A.
        // The Newton enclosure itself is inclusive and may touch the first zero.
        double strictLower=rootLo;
        if(rootLo>0&&!A::Outward(rootLo,false,strictLower))return work.Unavailable();
        lo=FMath::Max(lo,FMath::Max(0.,strictLower));
        // The old B remains a strict inside witness. Newton's upper root bound
        // is NOT an inside witness: it is only a proposal for a separate test.
        if(distance.UpperSquared<r2.Lower){hi=m;inside=distance;}
        const double pad=gap*.125;
        const double upperProposal=FMath::Min(hi,rootHi+pad);
        if(upperProposal>lo&&upperProposal<hi)
        {
            FIntervalPoint upperCenter;if(!work.Pay()||!PointAt(path,upperProposal,upperCenter))return work.Failure==E::Work?E::Work:work.Unavailable();
            FTensorDistanceBounds upper;
            if(!work.Minimum(surface,upperCenter,upper,seen,nullptr,&r2,0,true))return work.Failure;
            ++seen.InsideValidations;
            if(upper.UpperSquared<r2.Lower){hi=upperProposal;inside=upper;}
        }
        if(!(lo<hi))return E::Arithmetic;
        ++seen.TimeIterations;
        // Scalar Newton proposal from an original-UV witness. It grants no
        // bound; the next exact minimum and interval-Newton enclosure do that.
        double slope=0;
        for(int32 d=0;d<3;++d)slope+=-2*(.5*distance.SamplePoint.Axis[d].Lower+.5*distance.SamplePoint.Axis[d].Upper-
            (.5*center.Axis[d].Lower+.5*center.Axis[d].Upper))*(.5*velocity.Axis[d].Lower+.5*velocity.Axis[d].Upper);
        const double scalar=.5*value.Lower+.5*value.Upper;
        proposal=A::Finite(slope)&&slope<0?m-scalar/slope:.5*rootLo+.5*rootHi;
        if(!A::Finite(proposal)||!(proposal>lo&&proposal<hi))proposal=.5*rootLo+.5*FMath::Min(hi,rootHi);
        if(lo==oldLo&&hi==oldHi&&!(proposal>lo&&proposal<hi))break;
    }
    if(!(lo>=0&&lo<hi&&hi<=1)||inside.Status==EDistanceBoundStatus::Unknown||!(inside.UpperSquared<r2.Lower))return E::Arithmetic;
    if(!work.Pay())return work.Failure;
    FMonotoneCurveTimeSeed result;result.Lower=lo;result.Upper=hi;result.InsideWitness=inside;
    result.RestrictedPath.Degree=1;FIntervalPoint a,b;
    if(!PointAt(path,lo,a)||!PointAt(path,hi,b))return work.Unavailable();
    result.RestrictedPath.ControlPoints={a,b};
    if(!CenterDetail::Valid(result.RestrictedPath))return work.Unavailable();
    const auto completed=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(completed.Status!=EAnalyticNumericStatus::Complete||completed.ControlFingerprint()!=numeric.ControlFingerprint())return E::Arithmetic;
    out=MoveTemp(result);return E::Proved;
}
}
EMonotoneCurveTimeSeed TryMonotoneCurveTimeSeed(TConstArrayView<FIntervalTensorSurface> surfaces,
    const FIntervalBezierCenterPath& path,double radius,FDistanceBudget& budget,double gap,int32 maximumWork,
    FMonotoneCurveTimeSeed& out,FMonotoneCurveTimeInspection* inspection)
{
    FMonotoneCurveTimeInspection local;auto& seen=inspection?*inspection:local;seen={};
    const bool observe=StrongConvexCurveMinimum::Observation::Enabled();const uint64 start=observe?FPlatformTime::Cycles64():0,before=budget.Used;
    if(observe)++MonotoneCurveTimeControl::Observation.Calls;
    const auto result=MonotoneSeedDetail::Run(surfaces,path,radius,budget,gap,maximumWork,out,seen);
    if(observe)
    {
        auto& o=MonotoneCurveTimeControl::Observation;o.Ticks.fetch_add(FPlatformTime::Cycles64()-start);o.Work.fetch_add(budget.Used-before);
        o.MinimumCalls.fetch_add(seen.MinimumCalls);o.TimeIterations.fetch_add(seen.TimeIterations);o.InsideValidations.fetch_add(seen.InsideValidations);
        ++o.Results[uint8(result)];++o.Finished;
    }
    return result;
}
}
}


#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Analytic::Certified::MonotoneSeedTests
{
FIntervalTensorSurface Parabola()
{
    FIntervalTensorSurface curve;curve.DegreeU=0;curve.DegreeV=5;
    for(int32 i=0;i<6;++i)curve.ControlPoints.Add(Detail::Point(FVector3d(4*i,i*(i-1),0)));
    return curve;
}
bool SamePoint(const FIntervalPoint& a,const FIntervalPoint& b)
{
    for(int32 d=0;d<3;++d)if(a.Axis[d].Known!=b.Axis[d].Known||
        AnalyticCertified::Bits(a.Axis[d].Lower)!=AnalyticCertified::Bits(b.Axis[d].Lower)||
        AnalyticCertified::Bits(a.Axis[d].Upper)!=AnalyticCertified::Bits(b.Axis[d].Upper))return false;
    return true;
}
}
namespace Speed::Analytic::Certified::TimeDetail
{
struct FMonotoneTimeSeedTestAccess
{
    static bool Split(FRetainedSphereTimeSearch& search,const FTimeNode& node,double middle,
        FIntervalBezierCenterPath& left,FIntervalBezierCenterPath& right)
    {search.SeedHasStrictInside=true;return search.SplitTimeNodeCurve(node,middle,left,right);}
};
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMonotoneCurveTimeSeedGlobalSplitTest,
 "IAmSpeed.AnalyticWorld.TensorTime.MonotoneCurveSeed.NonDyadicGlobalSplit",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FMonotoneCurveTimeSeedGlobalSplitTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;namespace T=TimeDetail;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();TArray<FIntervalTensorSurface> surfaces;surfaces.Add(MonotoneSeedTests::Parabola());
    FIntervalBezierCenterPath path;if(!CenterDetail::Linear(FVector3d(0,0,0),FVector3d(1,2,4),path))return false;
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);FDistanceBudget budget{10000};
        T::FRetainedSphereTimeSearch search(surfaces,path,1.,budget,1.e-7,512,nullptr,false,nullptr,true,true,false,false,false);
        const double a=.123456789,b=.876543211;FIntervalBezierCenterPath restricted;restricted.Degree=1;FIntervalPoint pa,pb;
        if(!T::MonotoneSeedDetail::PointAt(path,a,pa)||!T::MonotoneSeedDetail::PointAt(path,b,pb))return false;
        restricted.ControlPoints={pa,pb};T::FTimeNode node{a,b,0,restricted};
        for(int32 depth=0;depth<40;++depth)
        {
            const double middle=.5*(node.A+node.B);FIntervalBezierCenterPath left,right;
            if(!TestTrue(TEXT("non-dyadic seed children use the original global time"),
                T::FMonotoneTimeSeedTestAccess::Split(search,node,middle,left,right)))return false;
            const auto hull=CenterDetail::Hull(node.Curve),lh=CenterDetail::Hull(left),rh=CenterDetail::Hull(right);
            bool exact=true;
            for(int32 d=0;d<3;++d)
            {
                const double coordinate=middle*double(1<<d);const auto& axis=left.ControlPoints.Last().Axis[d];
                exact&=axis.Lower<=coordinate&&axis.Upper>=coordinate&&lh.Axis[d].Lower>=hull.Axis[d].Lower&&
                    lh.Axis[d].Upper<=hull.Axis[d].Upper&&rh.Axis[d].Lower>=hull.Axis[d].Lower&&rh.Axis[d].Upper<=hull.Axis[d].Upper;
            }
            if(!TestTrue(TEXT("global original midpoint is enclosed, endpoints inherited and child hulls contained"),exact&&
                MonotoneSeedTests::SamePoint(left.ControlPoints[0],node.Curve.ControlPoints[0])&&
                MonotoneSeedTests::SamePoint(right.ControlPoints.Last(),node.Curve.ControlPoints.Last())&&
                MonotoneSeedTests::SamePoint(left.ControlPoints.Last(),right.ControlPoints[0])))return false;
            node=depth%2?T::FTimeNode{node.A,middle,depth+1,MoveTemp(left)}:T::FTimeNode{middle,node.B,depth+1,MoveTemp(right)};
        }
    }
    return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMonotoneCurveTimeSeedFirstContactTest,
 "IAmSpeed.AnalyticWorld.TensorTime.MonotoneCurveSeed.FirstContactAndContinuation",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FMonotoneCurveTimeSeedFirstContactTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    namespace T=TimeDetail;namespace A=::AnalyticCertified;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    TArray<FIntervalTensorSurface> surfaces;surfaces.Add(MonotoneSeedTests::Parabola());
    FIntervalBezierCenterPath path;if(!CenterDetail::Linear(FVector3d(9,6,0),FVector3d(9.5,5.5,0),path))return false;
    const double exactTime=2.-FMath::Sqrt(2.);
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        FDistanceBudget work{1000000};T::FMonotoneCurveTimeSeed seed;T::FMonotoneCurveTimeInspection seen;
        const auto status=T::TryMonotoneCurveTimeSeed(surfaces,path,1.,work,1.e-6,768,seed,&seen);
        if(status!=T::EMonotoneCurveTimeSeed::Proved||!(seed.Lower<exactTime&&seed.Upper>exactTime))
            AddInfo(FString::Printf(TEXT("Monotone local fixture Flush=%u Status=%u A=%.17g B=%.17g Expected=%.17g InsideUpper=%.17g Mu=%.17g V=[%.17g,%.17g] Prefixes=%u Minimums=%u Iterations=%u InsideChecks=%u Used=%llu Remaining=%d"),
                flush,uint32(status),seed.Lower,seed.Upper,exactTime,seed.InsideWitness.UpperSquared,seen.GlobalCurvatureLower,
                seen.ParameterLower,seen.ParameterUpper,seen.PrefixAttempts,seen.MinimumCalls,seen.TimeIterations,seen.InsideValidations,work.Used,work.Remaining));
        if(!TestTrue(TEXT("normal path to parabola has a certified first-contact seed and true inside upper"),
            status==T::EMonotoneCurveTimeSeed::Proved&&seed.Lower<exactTime&&seed.Upper>exactTime&&
            seed.InsideWitness.UpperSquared<A::Square(Interval::Exact(1.)).Lower&&seen.GlobalCurvatureLower>0&&
            seen.MinimumCalls>2&&seen.TimeIterations>0&&work.Used>0&&work.Used<=768&&work.Remaining==1000000-int32(work.Used)))return false;
        FIntervalPoint exactEndpoint,originalWitness;
        if(!T::MonotoneSeedDetail::PointAt(path,seed.Upper,exactEndpoint)||
            !ProjectedDistanceDetail::EvaluateFixedUV(surfaces[0],seed.InsideWitness.SampleUV,originalWitness))return false;
        if(!TestTrue(TEXT("seed end and witness use the original path/curve evaluations exactly"),
            MonotoneSeedTests::SamePoint(seed.RestrictedPath.ControlPoints.Last(),exactEndpoint)&&
            MonotoneSeedTests::SamePoint(seed.InsideWitness.SamplePoint,originalWitness)))return false;
        FDistanceBudget oracleWork{1000000},actualWork{1000000};
        T::FRetainedSphereTimeSearch oracle(surfaces,path,1.,oracleWork,1.e-7,512,nullptr,false,nullptr,true,false,false,false,false);
        T::FRetainedSphereTimeSearch actual(surfaces,path,1.,actualWork,1.e-7,512,nullptr,false,nullptr,true,true,false,false,false);
        const auto old=oracle.Advance(1.e-5),first=actual.Advance(1.e-5),second=actual.Advance(1.e-6);
        if(!TestTrue(TEXT("original frontier continues the exact seed and never reports no-contact"),
            old.Status==ETensorSphereTimeStatus::FirstContactBracket&&first.Status==ETensorSphereTimeStatus::FirstContactBracket&&
            second.Status==ETensorSphereTimeStatus::FirstContactBracket&&first.InitialSeparationProved&&second.InitialSeparationProved&&
            first.LowerTime<=exactTime&&first.UpperTime>=exactTime&&second.LowerTime<=exactTime&&second.UpperTime>=exactTime&&
            second.LowerTime>=first.LowerTime&&second.UpperTime<=first.UpperTime&&
            A::Subtract(Interval::Exact(second.UpperTime),Interval::Exact(second.LowerTime)).Upper<=1.e-6))return false;
    }
    return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMonotoneCurveTimeSeedRefusalAndEndpointTest,
 "IAmSpeed.AnalyticWorld.TensorTime.MonotoneCurveSeed.RefusalsAndEndpointMinimum",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FMonotoneCurveTimeSeedRefusalAndEndpointTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    namespace T=TimeDetail;namespace A=::AnalyticCertified;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    TArray<FIntervalTensorSurface> surfaces;FIntervalTensorSurface line;line.DegreeU=0;line.DegreeV=5;
    for(int32 i=0;i<6;++i)line.ControlPoints.Add(Detail::Point(FVector3d(4*i,0,0)));surfaces.Add(line);
    FIntervalBezierCenterPath path;if(!CenterDetail::Linear(FVector3d(-5,0,0),FVector3d(-1,0,0),path))return false;
    T::FMonotoneCurveTimeSeed seed;T::FMonotoneCurveTimeInspection seen;FDistanceBudget work{100000};
    if(!TestTrue(TEXT("constrained endpoint minimum is retained by the global V theorem"),
        T::TryMonotoneCurveTimeSeed(surfaces,path,2.,work,1.e-6,768,seed,&seen)==T::EMonotoneCurveTimeSeed::Proved&&
        seen.ParameterLower==0&&seen.ParameterUpper==0&&seed.Lower<.75&&seed.Upper>.75&&seed.InsideWitness.SampleUV.Y==0))return false;
    auto family=surfaces;family[0].ControlPoints[0].Axis[0]={-1.e-9,1.e-9,true};FDistanceBudget familyWork{10000};
    if(!TestTrue(TEXT("one seed universally encloses all endpoint-minimum coefficient realizations"),
        T::TryMonotoneCurveTimeSeed(family,path,2.,familyWork,1.e-6,768,seed)==T::EMonotoneCurveTimeSeed::Proved&&
        seed.Lower<.75-2.5e-10&&seed.Upper>.75+2.5e-10&&seed.InsideWitness.UpperSquared<A::Square(Interval::Exact(2.)).Lower))return false;
    // The wider normal path is physically monotone, but its whole rectangular
    // center hull leaves Newton-V at [0,1]. Refusing the OPTIONAL theorem is
    // correct; the original frontier must still find its first contact.
    TArray<FIntervalTensorSurface> wideSurfaces;wideSurfaces.Add(MonotoneSeedTests::Parabola());
    FIntervalBezierCenterPath widePath;if(!CenterDetail::Linear(FVector3d(6,9,0),FVector3d(10,5,0),widePath))return false;
    FDistanceBudget wideHint{1000000};T::FMonotoneCurveTimeInspection wideSeen;
    const auto wideStatus=T::TryMonotoneCurveTimeSeed(wideSurfaces,widePath,2.,wideHint,1.e-6,768,seed,&wideSeen);
    if(wideStatus!=T::EMonotoneCurveTimeSeed::NotApplicable)
        AddInfo(FString::Printf(TEXT("Monotone wide fixture Status=%u Prefixes=%u Minimums=%u Used=%llu"),uint32(wideStatus),wideSeen.PrefixAttempts,wideSeen.MinimumCalls,wideHint.Used));
    if(!TestTrue(TEXT("wide global center hull legitimately refuses optional monotone admission"),
        wideStatus==T::EMonotoneCurveTimeSeed::NotApplicable&&wideHint.Used>0&&seed.InsideWitness.Status==EDistanceBoundStatus::Unknown))return false;
    FDistanceBudget wideWork{1000000};T::FRetainedSphereTimeSearch wideSearch(wideSurfaces,widePath,2.,wideWork,1.e-7,512,nullptr,false,nullptr,true,true,false,false,false);
    const auto wideFirst=wideSearch.Advance(1.e-5);const double wideTime=1.-2./FMath::Sqrt(32.);
    if(!TestTrue(TEXT("wide physical path remains solved by the unchanged complete frontier"),
        wideFirst.Status==ETensorSphereTimeStatus::FirstContactBracket&&wideFirst.InitialSeparationProved&&
        wideFirst.LowerTime<=wideTime&&wideFirst.UpperTime>=wideTime))return false;
    auto two=surfaces;two.Add(MonotoneSeedTests::Parabola());FDistanceBudget unionWork{10000};
    if(!TestTrue(TEXT("one provider cannot hide an earlier contact from another provider"),
        T::TryMonotoneCurveTimeSeed(two,path,2.,unionWork,1.e-6,768,seed)==T::EMonotoneCurveTimeSeed::NotApplicable&&unionWork.Used==0))return false;
    FIntervalBezierCenterPath dip;dip.Degree=2;dip.ControlPoints={Detail::Point(FVector3d(-5,0,0)),Detail::Point(FVector3d(5,0,0)),Detail::Point(FVector3d(-5,0,0))};
    FDistanceBudget dipWork{10000};
    if(!TestTrue(TEXT("enter-and-exit nonmonotone path stays with the original whole-time search"),
        T::TryMonotoneCurveTimeSeed(surfaces,dip,2.,dipWork,1.e-6,768,seed)==T::EMonotoneCurveTimeSeed::NotApplicable&&dipWork.Used==0))return false;
    FDistanceBudget fallbackWork{1000000};T::FRetainedSphereTimeSearch fallback(surfaces,dip,2.,fallbackWork,1.e-7,512,nullptr,false,nullptr,true,true,false,false,false);
    const auto firstDip=fallback.Advance(1.e-5);const double dipTime=.5*(1.-FMath::Sqrt(.4));
    if(!TestTrue(TEXT("rejected nonmonotone hint preserves the first entering contact before the later exit"),
        firstDip.Status==ETensorSphereTimeStatus::FirstContactBracket&&firstDip.InitialSeparationProved&&
        firstDip.LowerTime<=dipTime&&firstDip.UpperTime>=dipTime&&firstDip.UpperTime<.5))return false;
    auto uncertain=path;uncertain.ControlPoints[0].Axis[0].Upper=A::FromBits(A::Bits(-5.)-1);
    FDistanceBudget uncertainWork{10000};
    if(!TestTrue(TEXT("independent path uncertainty cannot borrow fixed-velocity monotonicity"),
        T::TryMonotoneCurveTimeSeed(surfaces,uncertain,2.,uncertainWork,1.e-6,768,seed)==T::EMonotoneCurveTimeSeed::NotApplicable&&uncertainWork.Used==0))return false;
    auto nonlinear=surfaces;for(int32 i=0;i<6;++i)nonlinear[0].ControlPoints[i]=Detail::Point(FVector3d(i%2?10.:-10.,i%2?20.:-20.,0));
    FDistanceBudget nonlinearWork{10000};
    if(!TestTrue(TEXT("unproved global convexity gives no seed and retains its real debit"),
        T::TryMonotoneCurveTimeSeed(nonlinear,path,2.,nonlinearWork,1.e-6,768,seed)==T::EMonotoneCurveTimeSeed::NotApplicable&&nonlinearWork.Used>0&&seed.InsideWitness.Status==EDistanceBoundStatus::Unknown))return false;
    FDistanceBudget noRoom{768,11,1};
    if(!TestTrue(TEXT("optional finite forecast respects the live floor without debit or refund"),
        T::TryMonotoneCurveTimeSeed(surfaces,path,2.,noRoom,1.e-6,768,seed)==T::EMonotoneCurveTimeSeed::NotApplicable&&noRoom.Used==11&&noRoom.Remaining==768&&noRoom.ConsumptionFloor==1))return false;
    FDistanceBudget badAccount{768,11,-1};
    if(!TestTrue(TEXT("invalid account is not an optional fallback"),
        T::TryMonotoneCurveTimeSeed(surfaces,path,2.,badAccount,1.e-6,768,seed)==T::EMonotoneCurveTimeSeed::Work&&badAccount.Used==11))return false;
    auto huge=surfaces;for(int32 i=0;i<6;++i)huge[0].ControlPoints[i]=Detail::Point(FVector3d(double(i)*1.e200,0,0));
    FDistanceBudget hugeWork{10000};
    if(!TestTrue(TEXT("optional derivative overflow falls back with debit and no witness"),
        T::TryMonotoneCurveTimeSeed(huge,path,2.,hugeWork,1.e-6,768,seed)==T::EMonotoneCurveTimeSeed::NotApplicable&&
        hugeWork.Used>0&&seed.InsideWitness.Status==EDistanceBoundStatus::Unknown))return false;
    const uint32 control=VectorGetControlRegister();VectorSetControlRegister((control&~uint32(0x6000))|uint32(0x2000));FDistanceBudget invalidNumeric{10000};
    TestTrue(TEXT("changed numeric controls fail closed without repair"),
        T::TryMonotoneCurveTimeSeed(surfaces,path,2.,invalidNumeric,1.e-6,768,seed)==T::EMonotoneCurveTimeSeed::Arithmetic&&
        (VectorGetControlRegister()&uint32(0x6000))==uint32(0x2000));
    return !HasAnyErrors();
}
#endif


// Pure immutable geometry values only. No volume, active-group mask, clearance,
// partition, scene ownership or work account survives in this memo.
namespace Speed::Analytic::Certified::GroupedBoxGeometryMemo
{
namespace A=::AnalyticCertified;
static TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxGeometryMemo"),0,
    TEXT("Default OFF. Reuse exact restricted tensor splits and exact-axis control projections; original separation and budget remain unchanged."),ECVF_Default);
bool RuntimeEnabled(){return Enabled.GetValueOnAnyThread()==1;}
// A repeated BOX coefficient is not a claim that its real realizations are
// correlated. This factors only extrema of the literal per-CP interval DAG.
static TAutoConsoleVariable<int32> SeparableProjectionEnabled(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxSeparableProjection"),0,
    TEXT("Default OFF. Factor bitwise separable control projections; identical bounds/refusals and debits, with variant-local sticky memo publications."),ECVF_Default);
struct FSeparableCounts
{
    std::atomic<uint64> Checks{0},AdmittedRoots{0},AdmittedDescendants{0},RefusedRoots{0},RefusedDescendants{0},
        ColdOriginal{0},ColdFactored{0},FactoredInvalid{0},OriginalTicks{0},FactoredTicks{0},AdmissionTicks{0},
        OriginalInputPoints{0},FactoredInputPoints{0},NominalMultiplies{0},NominalAdds{0},VariantMisses{0};
};
static FSeparableCounts SeparableCounts;
static void PublishSeparableObservation()
{
    const auto& x=SeparableCounts;
    UE_LOG(LogTemp,Display,TEXT("[GroupedBoxSeparableProjection] Scope=ProgramPrefixAtPreExit NoPASS=1 Checks=%llu AdmittedRoots=%llu AdmittedDescendants=%llu RefusedRoots=%llu RefusedDescendants=%llu ColdOriginal=%llu ColdFactored=%llu FactoredInvalid=%llu OriginalTicks=%llu FactoredTicks=%llu AdmissionTicks=%llu SecondsPerCycle=%.17g OriginalInputPoints=%llu FactoredInputPoints=%llu NominalMultiplies=%llu NominalAdds=%llu VariantMisses=%llu"),
        x.Checks.load(),x.AdmittedRoots.load(),x.AdmittedDescendants.load(),x.RefusedRoots.load(),x.RefusedDescendants.load(),
        x.ColdOriginal.load(),x.ColdFactored.load(),x.FactoredInvalid.load(),x.OriginalTicks.load(),x.FactoredTicks.load(),x.AdmissionTicks.load(),
        FPlatformTime::GetSecondsPerCycle64(),x.OriginalInputPoints.load(),x.FactoredInputPoints.load(),x.NominalMultiplies.load(),x.NominalAdds.load(),x.VariantMisses.load());
}
// 0=not yet inspected, -1=not separable, 1..3=the one U-dependent
// coordinate (others depend on V), 4..6=the one V-dependent coordinate.
// Called only after GeometryOnly has validated every original coefficient.
static int8 FindSeparableLayout(const Detail::FNode& node)
{
    if(node.DegreeU<1||node.DegreeV<1)return -1;
    bool onlyU[3]={true,true,true},onlyV[3]={true,true,true};
    const int32 stride=node.DegreeV+1;
    for(int32 u=0;u<=node.DegreeU;++u)for(int32 v=0;v<=node.DegreeV;++v)
        for(int32 d=0;d<3;++d)
        {
            const auto& value=node.Net[u*stride+v].Axis[d];
            onlyU[d]&=GroupedBoxDetail::SameInterval(value,node.Net[u*stride].Axis[d]);
            onlyV[d]&=GroupedBoxDetail::SameInterval(value,node.Net[v].Axis[d]);
        }
    for(int32 d=0;d<3;++d)
    {
        const int32 a=(d+1)%3,b=(d+2)%3;
        if(onlyU[d]&&onlyV[a]&&onlyV[b])return int8(1+d);
        if(onlyV[d]&&onlyU[a]&&onlyU[b])return int8(4+d);
    }
    return -1;
}
static Interval FactoredProjectHull(const Detail::FNode& node,const FVector3d& axis,int8 layout)
{
    if(layout<1||layout>6)return {};
    const bool independentU=layout<=3;const int32 independentAxis=(int32(layout)-1)%3;
    const int32 independentCount=(independentU?node.DegreeU:node.DegreeV)+1;
    const int32 dependentCount=(independentU?node.DegreeV:node.DegreeU)+1,stride=node.DegreeV+1;
    Interval independentHull;bool haveIndependent=false;
    for(int32 i=0;i<independentCount;++i)
    {
        const auto& point=node.Net[independentU?i*stride:i];
        auto value=A::Multiply(point.Axis[independentAxis],Interval::Exact(axis[independentAxis]));
        // Dot always starts with an outward 0 + product. It cannot be elided.
        if(independentAxis==0)value=A::Add(Interval::Exact(0.),value);
        if(!A::Valid(value))return {};
        if(!haveIndependent){independentHull=value;haveIndependent=true;}
        else {if(value.Lower<independentHull.Lower)independentHull.Lower=value.Lower;
            if(value.Upper>independentHull.Upper)independentHull.Upper=value.Upper;}
    }
    Interval result;bool have=false;
    for(int32 j=0;j<dependentCount;++j)
    {
        const auto& point=node.Net[independentU?j:j*stride];
        auto value=Interval::Exact(0.);
        for(int32 d=0;d<3;++d)
        {
            if(d==independentAxis)
                value=d==0?independentHull:A::Add(value,independentHull);
            else value=A::Add(value,A::Multiply(point.Axis[d],Interval::Exact(axis[d])));
            if(!A::Valid(value))return {};
        }
        if(!have){result=value;have=true;}
        else {if(value.Lower<result.Lower)result.Lower=value.Lower;
            if(value.Upper>result.Upper)result.Upper=value.Upper;}
    }
    return result;
}

struct FCounts
{
    std::atomic<uint64> Cells{0},CellHits{0},ProjectionRequests{0},ProjectionHits{0},ProjectionBuilds{0},
        SplitRequests{0},SplitHits{0},SplitBuilds{0},Replacements{0};
};
static FCounts Counts;
static void PublishAabbObservation();
static void PublishContainedObservation();
void PublishObservation()
{
    PublishAabbObservation();
    PublishContainedObservation();
    PublishSeparableObservation();
    UE_LOG(LogTemp,Display,TEXT("[GroupedBoxGeometryMemo] Scope=ProgramPrefixAtPreExit NoPASS=1 Cells=%llu CellHits=%llu ProjectionRequests=%llu ProjectionHits=%llu ProjectionBuilds=%llu SplitRequests=%llu SplitHits=%llu SplitBuilds=%llu Replacements=%llu"),
        Counts.Cells.load(),Counts.CellHits.load(),Counts.ProjectionRequests.load(),Counts.ProjectionHits.load(),Counts.ProjectionBuilds.load(),
        Counts.SplitRequests.load(),Counts.SplitHits.load(),Counts.SplitBuilds.load(),Counts.Replacements.load());
}
struct FProjection
{
    FVector3d Axis=FVector3d::ZeroVector;Interval Slab;uint32 Sticky=0;bool Ready=false,FactoredMode=false;
};
struct FEntry
{
    Detail::FNode Input,Left,Right;
    FIntervalPoint ControlHull;bool HullReady=false;int8 SeparableLayout=0;
    FProjection Projections[12];
    uint64 Hash=0,Numeric=0;uint32 SplitSticky=0;uint8 NextProjection=0;
    bool Ready=false,SplitReady=false;
};
struct FStore {FEntry Entries[1024];};
static FStore& Store(){static thread_local TUniquePtr<FStore> values; if(!values)values=MakeUnique<FStore>();return *values;}
bool SameAxis(const FVector3d& a,const FVector3d& b)
{for(int32 d=0;d<3;++d)if(A::Bits(a[d])!=A::Bits(b[d]))return false;return true;}
bool GeometryOnly(const Detail::FNode& node)
{
    if(node.DegreeU<0||node.DegreeU>16||node.DegreeV<0||node.DegreeV>16||
        node.Net.Num()!=(node.DegreeU+1)*(node.DegreeV+1)||node.Net.Num()>64||
        node.DepthU<0||node.DepthU>52||node.DepthV<0||node.DepthV>52||
        !A::Finite(node.MinU)||!A::Finite(node.MaxU)||!A::Finite(node.MinV)||!A::Finite(node.MaxV)||
        node.MinU<0||node.MinU>=node.MaxU||node.MaxU>1||node.MinV<0||node.MinV>=node.MaxV||node.MaxV>1||
        A::Bits(node.LowerSquared)!=0||A::Bits(node.UpperSquared)!=0||
        A::Bits(node.WitnessUV.X)!=0||A::Bits(node.WitnessUV.Y)!=0)return false;
    for(const auto& axis:node.WitnessPoint.Axis)
        if(axis.Known||A::Bits(axis.Lower)!=0||A::Bits(axis.Upper)!=0)return false;
    for(const auto& point:node.Net)if(!Detail::ValidPoint(point))return false;
    return true;
}
bool SameNode(const Detail::FNode& a,const Detail::FNode& b)
{
    if(a.DegreeU!=b.DegreeU||a.DegreeV!=b.DegreeV||a.DepthU!=b.DepthU||a.DepthV!=b.DepthV||a.Net.Num()!=b.Net.Num()||
        A::Bits(a.MinU)!=A::Bits(b.MinU)||A::Bits(a.MaxU)!=A::Bits(b.MaxU)||
        A::Bits(a.MinV)!=A::Bits(b.MinV)||A::Bits(a.MaxV)!=A::Bits(b.MaxV))return false;
    for(int32 i=0;i<a.Net.Num();++i)if(!GroupedBoxDetail::SamePoint(a.Net[i],b.Net[i]))return false;
    return true;
}
uint64 Key(const Detail::FNode& node,uint64 numeric)
{
    uint64 hash=1469598103934665603ull;
    const auto add=[&](uint64 value){hash^=value;hash*=1099511628211ull;};
    add(numeric);add(uint64(node.DegreeU));add(uint64(node.DegreeV));add(uint64(node.DepthU));add(uint64(node.DepthV));
    add(A::Bits(node.MinU));add(A::Bits(node.MaxU));add(A::Bits(node.MinV));add(A::Bits(node.MaxV));
    for(const auto& point:node.Net)for(const auto& axis:point.Axis){add(A::Bits(axis.Lower));add(A::Bits(axis.Upper));}
    hash^=hash>>33;hash*=0xff51afd7ed558ccdull;hash^=hash>>33;hash*=0xc4ceb9fe1a85ec53ull;hash^=hash>>33;return hash;
}
FEntry* Acquire(const Detail::FNode& node,uint64 numeric,bool observe)
{
    if(!GeometryOnly(node))return nullptr;
    if(observe)++Counts.Cells;
    const uint64 key=Key(node,numeric);auto& entry=Store().Entries[key&1023u];
    if(entry.Ready&&entry.Hash==key&&entry.Numeric==numeric&&SameNode(entry.Input,node))
    {if(observe)++Counts.CellHits;return &entry;}
    if(observe&&entry.Ready)++Counts.Replacements;
    entry=FEntry{};entry.Input=node;entry.Hash=key;entry.Numeric=numeric;entry.Ready=true;return &entry;
}
bool CompatibleSticky(uint32 required){return (VectorGetControlRegister()&required)==required;}
int32 FixedAxis(const FVector3d& axis)
{
    for(int32 d=0;d<3;++d)
    {
        FVector3d candidate=FVector3d::ZeroVector;candidate[d]=1.;
        if(SameAxis(axis,candidate))return d;
    }
    return INDEX_NONE;
}
Interval Project(const Detail::FNode& node,const FVector3d& axis,FEntry* entry,bool observe)
{
    if(observe)++Counts.ProjectionRequests;
    const bool factoredMode=SeparableProjectionEnabled.GetValueOnAnyThread()==1;
    FProjection* slot=nullptr;
    if(entry)
    {
        const int32 fixed=FixedAxis(axis);
        if(fixed!=INDEX_NONE)slot=&entry->Projections[fixed];
        else
        {
            for(int32 i=3;i<12;++i)if(entry->Projections[i].Ready&&SameAxis(entry->Projections[i].Axis,axis))
            {slot=&entry->Projections[i];break;}
            if(!slot)slot=&entry->Projections[3+(entry->NextProjection++%9)];
        }
        if(slot->Ready&&SameAxis(slot->Axis,axis)&&slot->FactoredMode==factoredMode&&CompatibleSticky(slot->Sticky))
        {if(observe)++Counts.ProjectionHits;return slot->Slab;}
        if(observe&&slot->Ready&&SameAxis(slot->Axis,axis)&&slot->FactoredMode!=factoredMode)++SeparableCounts.VariantMisses;
    }
    if(factoredMode&&entry&&entry->SeparableLayout==0)
    {
        const uint64 start=observe?FPlatformTime::Cycles64():0;
        entry->SeparableLayout=FindSeparableLayout(node);
        if(observe)
        {
            ++SeparableCounts.Checks;SeparableCounts.AdmissionTicks.fetch_add(FPlatformTime::Cycles64()-start);
            const bool root=node.DepthU==0&&node.DepthV==0;
            if(entry->SeparableLayout>0){if(root)++SeparableCounts.AdmittedRoots;else ++SeparableCounts.AdmittedDescendants;}
            else {if(root)++SeparableCounts.RefusedRoots;else ++SeparableCounts.RefusedDescendants;}
        }
    }
    const bool factor=factoredMode&&entry&&entry->SeparableLayout>0;
    const uint64 start=observe?FPlatformTime::Cycles64():0;
    const auto slab=factor?FactoredProjectHull(node,axis,entry->SeparableLayout):GroupedBoxDetail::ProjectHull(node.Net,axis);
    if(observe)
    {
        const uint64 ticks=FPlatformTime::Cycles64()-start;
        if(factor)
        {
            ++SeparableCounts.ColdFactored;SeparableCounts.FactoredTicks.fetch_add(ticks);
            SeparableCounts.FactoredInputPoints.fetch_add(uint64(node.Net.Num()));if(!A::Valid(slab))++SeparableCounts.FactoredInvalid;
            const bool u=entry->SeparableLayout<=3;const int32 independent=(u?node.DegreeU:node.DegreeV)+1,dependent=(u?node.DegreeV:node.DegreeU)+1;
            SeparableCounts.NominalMultiplies.fetch_add(uint64(independent+2*dependent));
            SeparableCounts.NominalAdds.fetch_add(uint64((int32(entry->SeparableLayout)-1)%3==0?independent+2*dependent:3*dependent));
        }
        else {++SeparableCounts.ColdOriginal;SeparableCounts.OriginalTicks.fetch_add(ticks);SeparableCounts.OriginalInputPoints.fetch_add(uint64(node.Net.Num()));}
    }
    if(slot&&A::Valid(slab))
    {slot->Axis=axis;slot->Slab=slab;slot->Sticky=VectorGetControlRegister()&0x3fu;slot->FactoredMode=factoredMode;slot->Ready=true;if(observe)++Counts.ProjectionBuilds;}
    return slab;
}
bool ProjectCell(const Detail::FNode& node,TConstArrayView<FVector3d> axes,FDistanceBudget& budget,
    TArray<Interval,TInlineAllocator<8>>& projected,FGroupedBoxSeparationObservation* observed)
{
    const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=EAnalyticNumericStatus::Complete)return false;
    const bool observe=StrongConvexCurveMinimum::Observation::Enabled();
    auto* entry=Acquire(node,numeric.ControlFingerprint(),observe);
    for(const auto& axis:axes)
    {
        if(observed){observed->Stage=TEXT("cell_projection_or_budget");observed->Axis=projected.Num();}
        if(!budget.Consume())return false;
        const auto slab=Project(node,axis,entry,observe);
        if(!A::Valid(slab))return false;projected.Add(slab);
    }
    return true;
}
// A bounding-box test inside one bounded per-axis projection operation. Every
// original axis and group debit remains at its original caller. An inconclusive
// box computes the literal CP projections before any group mask can change.
static TAutoConsoleVariable<int32> AabbFirstEnabled(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxAabbFirst"),0,
    TEXT("Default OFF. Reject all active box groups with a conservative control AABB before full CP projections; exact fallback and debits retained."),ECVF_Default);
struct FAabbCounts
{
    std::atomic<uint64> Calls{0},Finished{0},Ticks{0},AllClear{0},Fallback{0},Unavailable{0},Slabs{0};
};
static FAabbCounts AabbCounts;
static void PublishAabbObservation()
{
    const auto& x=AabbCounts;
    UE_LOG(LogTemp,Display,TEXT("[GroupedBoxAabbFirst] Scope=ProgramPrefixAtPreExit NoPASS=1 Calls=%llu Finished=%llu Ticks=%llu SecondsPerCycle=%.17g AllClear=%llu Fallback=%llu Unavailable=%llu CoarseSlabs=%llu"),
        x.Calls.load(),x.Finished.load(),x.Ticks.load(),FPlatformTime::GetSecondsPerCycle64(),x.AllClear.load(),x.Fallback.load(),x.Unavailable.load(),x.Slabs.load());
}
struct FAabbScope
{
    bool Active;uint64 Start=0;
    explicit FAabbScope(bool active):Active(active){if(Active){++AabbCounts.Calls;Start=FPlatformTime::Cycles64();}}
    ~FAabbScope(){if(Active){AabbCounts.Ticks.fetch_add(FPlatformTime::Cycles64()-Start);++AabbCounts.Finished;}}
};
static bool ProjectCellForVolumeOriginal(const Detail::FNode& node,const FGroupedBoxVolumeBounds& volume,uint32 activeGroups,
    FDistanceBudget& budget,TArray<Interval,TInlineAllocator<8>>& projected,FGroupedBoxSeparationObservation* observed)
{
    if(AabbFirstEnabled.GetValueOnAnyThread()!=1)return ProjectCell(node,volume.Axes,budget,projected,observed);
    const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=EAnalyticNumericStatus::Complete)return false;
    const bool observe=StrongConvexCurveMinimum::Observation::Enabled();FAabbScope timing(observe);
    auto* entry=Acquire(node,numeric.ControlFingerprint(),observe);
    if(!entry)
    {if(observe)++AabbCounts.Unavailable;return ProjectCell(node,volume.Axes,budget,projected,observed);}
    if(!entry->HullReady)
    {
        FIntervalPoint hull;if(!MinimumDetail::NetHull(node,hull))
        {if(observe)++AabbCounts.Unavailable;return ProjectCell(node,volume.Axes,budget,projected,observed);}
        entry->ControlHull=hull;entry->HullReady=true;
    }
    for(const auto& axis:volume.Axes)
    {
        if(observed){observed->Stage=TEXT("cell_projection_or_budget");observed->Axis=projected.Num();}
        if(!budget.Consume())return false;
        // World coordinate axes already have exact cached slabs. For a new
        // oblique axis, one interval point replaces the entire CP array first.
        auto slab=FixedAxis(axis)!=INDEX_NONE?Project(node,axis,entry,observe):GroupedBoxDetail::Project(entry->ControlHull,axis);
        if(observe)++AabbCounts.Slabs;
        // Overflow/unknown of the optional wider bound cannot replace the
        // original finite projection's result or its per-axis failure point.
        if(!A::Valid(slab))slab=Project(node,axis,entry,observe);
        if(!A::Valid(slab))return false;projected.Add(slab);
    }
    bool allClear=true;
    for(int32 g=0;g<volume.Groups.Num();++g)if(activeGroups&(1u<<g))
    {
        bool clear=false;
        for(int32 a=0;a<volume.Axes.Num();++a)
        {
            const auto body=volume.Slabs[g*volume.Axes.Num()+a],surface=projected[a];
            clear|=body.Upper<surface.Lower||surface.Upper<body.Lower;
        }
        allClear&=clear;
    }
    if(allClear){if(observe)++AabbCounts.AllClear;return true;}
    if(observe)++AabbCounts.Fallback;
    // This remains the SAME bounded axis-projection operation. Its debit was
    // taken above; neither a new budget nor a refunded old debit exists here.
    // The caller's original group loop and dynamic mask run only after all
    // original CP projections have been restored.
    for(int32 a=0;a<volume.Axes.Num();++a)
    {
        const auto slab=Project(node,volume.Axes[a],entry,observe);
        if(!A::Valid(slab))return false;projected[a]=slab;
    }
    return true;
}
// Optional geometric impossibility of the EXISTING slab-separation theorem.
// It never reports a physical contact: false stops this proof attempt only.
static TAutoConsoleVariable<int32> ContainedRejectEnabled(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxContainedReject"),0,
    TEXT("Default OFF. Stop a grouped separation attempt when one complete original patch is strictly contained in one active slab region; original fine fallback remains required."),ECVF_Default);
struct FContainedCounts {std::atomic<uint64> Calls{0},Rejected{0};};
static FContainedCounts ContainedCounts;
struct FProviderCost {std::atomic<uint64> Key{0},Cells{0},Ticks{0},Rejected{0};};
static FProviderCost ProviderCosts[64];static std::atomic<uint64> ProviderCostOverflow{0},ProviderCostOutsideScope{0};
struct FProviderCostScope
{
    FProviderCost* Slot=nullptr;uint64 Start=0;bool Rejected=false;
    FProviderCostScope(const Detail::FNode& node,const FGroupedBoxVolumeBounds& volume,const FGroupedBoxSeparationObservation* observed)
    {
        if(!StrongConvexCurveMinimum::Observation::Enabled()||!observed||observed->Provider<0)return;
        if(volume.Groups.Num()!=8&&volume.Groups.Num()!=14){++ProviderCostOutsideScope;return;}
        const uint64 key=(uint64(uint32(observed->Provider)+1)<<16)|(uint64(volume.Groups.Num())<<10)|(uint64(node.DegreeU)<<5)|uint64(node.DegreeV);
        for(auto& entry:ProviderCosts)
        {uint64 found=entry.Key.load(std::memory_order_relaxed);
            if(found==0){uint64 empty=0;if(entry.Key.compare_exchange_strong(empty,key))found=key;else found=empty;}
            if(found==key){Slot=&entry;Start=FPlatformTime::Cycles64();return;}}
        ++ProviderCostOverflow;
    }
    ~FProviderCostScope(){if(Slot){Slot->Ticks.fetch_add(FPlatformTime::Cycles64()-Start);++Slot->Cells;if(Rejected)++Slot->Rejected;}}
};
bool ProjectCellForVolume(const Detail::FNode& node,const FGroupedBoxVolumeBounds& volume,uint32 activeGroups,
    FDistanceBudget& budget,TArray<Interval,TInlineAllocator<8>>& projected,FGroupedBoxSeparationObservation* observed)
{
    FProviderCostScope providerCost(node,volume,observed);
    if(!ProjectCellForVolumeOriginal(node,volume,activeGroups,budget,projected,observed))return false;
    if(ContainedRejectEnabled.GetValueOnAnyThread()!=1)return true;
    const bool observe=StrongConvexCurveMinimum::Observation::Enabled();if(observe)++ContainedCounts.Calls;
    if(volume.Groups.IsEmpty()||volume.Groups.Num()>16||projected.Num()!=volume.Axes.Num()||
        volume.Slabs.Num()!=volume.Groups.Num()*volume.Axes.Num())return false;
    // All axis projection operations above are already charged, including
    // optional AABB support. This bounded comparison inspects those same
    // values; it spends no returned/refunded work and creates no proof.
    for(int32 g=0;g<volume.Groups.Num();++g)if(activeGroups&(1u<<g))
    {
        bool contained=true;
        for(int32 a=0;a<volume.Axes.Num();++a)
        {
            const auto body=volume.Slabs[g*volume.Axes.Num()+a],surface=projected[a];
            if(!A::Valid(body)||!A::Valid(surface))return false;
            contained&=body.Lower<surface.Lower&&surface.Upper<body.Upper;
        }
        if(contained)
        {
            providerCost.Rejected=true;if(observe)++ContainedCounts.Rejected;
            if(observed){observed->Stage=TEXT("cell_contained_group_region");observed->Group=g;}
            return false;
        }
    }
    return true;
}
static void PublishContainedObservation()
{
    UE_LOG(LogTemp,Display,TEXT("[GroupedBoxContainedReject] Scope=ProgramPrefixAtPreExit NoPhysicalContactClaim=1 Calls=%llu Rejected=%llu"),
        ContainedCounts.Calls.load(),ContainedCounts.Rejected.load());
    for(const auto& entry:ProviderCosts)
    {const uint64 key=entry.Key.load();if(!key)continue;
        UE_LOG(LogTemp,Display,TEXT("[GroupedBoxProviderProjectionCost] Scope=ProgramPrefixAtPreExit DiagnosticIndexOnly=1 RequiresCurrentSceneMap=1 ProjectionStageOnly=1 InclusiveProofTimeNotClaimed=1 Provider=%llu Groups=%llu DegreeU=%llu DegreeV=%llu Cells=%llu ContainedRejects=%llu Ticks=%llu SecondsPerCycle=%.17g"),
            (key>>16)-1,(key>>10)&63,(key>>5)&31,key&31,entry.Cells.load(),entry.Rejected.load(),entry.Ticks.load(),FPlatformTime::GetSecondsPerCycle64());}
    UE_LOG(LogTemp,Display,TEXT("[GroupedBoxProviderProjectionCost] OverflowCalls=%llu OutsidePredictionScope=%llu DetailGroups=8,14 MaxRows=64"),ProviderCostOverflow.load(),ProviderCostOutsideScope.load());
}

bool SplitCell(const Detail::FNode& node,Detail::FNode& left,Detail::FNode& right)
{
    const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=EAnalyticNumericStatus::Complete)return false;
    const bool observe=StrongConvexCurveMinimum::Observation::Enabled();if(observe)++Counts.SplitRequests;
    auto* entry=Acquire(node,numeric.ControlFingerprint(),observe);
    if(entry&&entry->SplitReady&&CompatibleSticky(entry->SplitSticky))
    {left=entry->Left;right=entry->Right;if(observe)++Counts.SplitHits;return true;}
    // Literal original direction and restriction. Success only is published;
    // no failed operation or exhausted account creates a cached clearance.
    if(!Detail::Split(node,Detail::ChooseTensorDistanceSplitU(node),left,right))return false;
    if(entry)
    {entry->Left=left;entry->Right=right;entry->SplitSticky=VectorGetControlRegister()&0x3fu;entry->SplitReady=true;if(observe)++Counts.SplitBuilds;}
    return true;
}
}


#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGroupedBoxGeometryMemoValueTest,
 "IAmSpeed.AnalyticWorld.GeneralBox.GeometryMemoValues",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FGroupedBoxGeometryMemoValueTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    namespace M=GroupedBoxGeometryMemo;namespace A=::AnalyticCertified;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> profile(TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"),1,FName(TEXT("GroupedBoxMemoProfile")));
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        Detail::FNode root;root.DegreeU=root.DegreeV=5;
        for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)
            root.Net.Add(Detail::Point(FVector3d(4*u,4*v,u*u+2*v*v)));
        const uint64 numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
        auto* entry=M::Acquire(root,numeric,false);if(!TestNotNull(TEXT("geometry-only root admitted"),entry))return false;
        TArray<FVector3d,TInlineAllocator<8>> axes={FVector3d(1,0,0),FVector3d(0,1,0),FVector3d(0,0,1),FVector3d(.25,-.75,1.5)};
        TArray<Interval,TInlineAllocator<8>> expected;
        for(const auto& axis:axes)expected.Add(GroupedBoxDetail::ProjectHull(root.Net,axis));
        FDistanceBudget cold{20,7},warm{20,7};TArray<Interval,TInlineAllocator<8>> coldValues,warmValues;
        if(!M::ProjectCell(root,axes,cold,coldValues,nullptr))return false;
        const uint64 before=M::Counts.ProjectionHits.load();
        if(!M::ProjectCell(root,axes,warm,warmValues,nullptr))return false;
        bool same=coldValues.Num()==expected.Num()&&warmValues.Num()==expected.Num();
        for(int32 i=0;same&&i<expected.Num();++i)same=GroupedBoxDetail::SameInterval(expected[i],coldValues[i])&&GroupedBoxDetail::SameInterval(expected[i],warmValues[i]);
        if(!TestTrue(TEXT("warm exact-axis projections preserve every endpoint and debit"),same&&cold.Used==warm.Used&&cold.Remaining==warm.Remaining&&
            cold.Used==11&&M::Counts.ProjectionHits.load()==before+uint64(axes.Num())))return false;
        Detail::FNode expectedA,expectedB,coldA,coldB,warmA,warmB;
        if(!Detail::Split(root,Detail::ChooseTensorDistanceSplitU(root),expectedA,expectedB)||!M::SplitCell(root,coldA,coldB))return false;
        const uint64 splitBefore=M::Counts.SplitHits.load();if(!M::SplitCell(root,warmA,warmB))return false;
        if(!TestTrue(TEXT("literal original split and chosen direction reused bit for bit"),
            M::SameNode(expectedA,coldA)&&M::SameNode(expectedB,coldB)&&M::SameNode(expectedA,warmA)&&M::SameNode(expectedB,warmB)&&
            M::Counts.SplitHits.load()==splitBefore+1))return false;
        // A cache hit must not silently omit masked exception flags that the
        // original arithmetic would have raised. Clear only sticky bits, never
        // repair or change any rounding, mask, FTZ or DAZ control.
        auto stickyRoot=root;stickyRoot.Net[0].Axis[0]=Interval::Exact(.1);
        const FVector3d stickyAxis(.1,.3,.7);const uint32 clear=VectorGetControlRegister()&~uint32(0x3f);
        VectorSetControlRegister(clear);const auto stickyExpected=GroupedBoxDetail::ProjectHull(stickyRoot.Net,stickyAxis);
        const uint32 projectedSticky=VectorGetControlRegister()&0x3fu;
        auto* stickyEntry=M::Acquire(stickyRoot,numeric,false);
        (void)M::Project(stickyRoot,stickyAxis,stickyEntry,false); // publication with the raised flags
        VectorSetControlRegister(clear);const uint64 stickyBefore=M::Counts.ProjectionHits.load();
        const auto stickyActual=M::Project(stickyRoot,stickyAxis,stickyEntry,true);
        if(!TestTrue(TEXT("cleared sticky forces original projection arithmetic and reproduces its flags"),
            projectedSticky!=0&&M::Counts.ProjectionHits.load()==stickyBefore&&
            (VectorGetControlRegister()&0x3fu)==projectedSticky&&GroupedBoxDetail::SameInterval(stickyExpected,stickyActual)))return false;
        Detail::FNode stickyOldA,stickyOldB,stickyA,stickyB;
        VectorSetControlRegister(clear);
        if(!Detail::Split(stickyRoot,Detail::ChooseTensorDistanceSplitU(stickyRoot),stickyOldA,stickyOldB))return false;
        const uint32 splitSticky=VectorGetControlRegister()&0x3fu;
        if(!M::SplitCell(stickyRoot,stickyA,stickyB))return false;
        VectorSetControlRegister(clear);const uint64 stickySplitBefore=M::Counts.SplitHits.load();
        if(!M::SplitCell(stickyRoot,stickyA,stickyB))return false;
        if(!TestTrue(TEXT("split reuse also preserves the original masked exception observations"),
            (splitSticky==0||M::Counts.SplitHits.load()==stickySplitBefore)&&
            (VectorGetControlRegister()&0x3fu)==splitSticky&&M::SameNode(stickyOldA,stickyA)&&M::SameNode(stickyOldB,stickyB)))return false;
        // Input and axis changes have independent exact keys. Neither is a fit.
        auto changed=root;changed.Net[17].Axis[1].Upper=A::FromBits(A::Bits(changed.Net[17].Axis[1].Upper)+1);
        auto changedAxes=axes;changedAxes[0].X=A::FromBits(A::Bits(1.)+1);
        FDistanceBudget changedWork{20};TArray<Interval,TInlineAllocator<8>> changedValues;
        if(!M::ProjectCell(changed,changedAxes,changedWork,changedValues,nullptr))return false;
        for(int32 i=0;i<changedAxes.Num();++i)
            if(!TestTrue(TEXT("one-ULP source/axis changes are evaluated from their own operands"),
                GroupedBoxDetail::SameInterval(changedValues[i],GroupedBoxDetail::ProjectHull(changed.Net,changedAxes[i]))))return false;
        // Exact query-dependent metadata is never admitted into this geometry cache.
        auto foreign=root;foreign.LowerSquared=1;
        if(!TestNull(TEXT("query-dependent node cannot enter geometry memo"),M::Acquire(foreign,numeric,false)))return false;
        auto malformed=root;malformed.Net[0].Axis[0].Known=false;
        if(!TestNull(TEXT("invalid coefficient cannot enter geometry memo"),M::Acquire(malformed,numeric,false)))return false;
        // Exhaustion midway through a cached projection sequence preserves the
        // same prefix, account floor and sticky state; no final proof is minted.
        FDistanceBudget denied{4,23,2};TArray<Interval,TInlineAllocator<8>> partial;FGroupedBoxSeparationObservation observed;
        if(!TestTrue(TEXT("cache hits do not bypass per-axis budget debits"),!M::ProjectCell(root,axes,denied,partial,&observed)&&
            denied.Remaining==2&&denied.Used==25&&denied.ConsumptionFloor==2&&partial.Num()==2&&observed.Axis==2))return false;
    }
    return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGroupedBoxGeometryMemoProofParityTest,
 "IAmSpeed.AnalyticWorld.GeneralBox.GeometryMemoProofParity",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FGroupedBoxGeometryMemoProofParityTest::RunTest(const FString&)
{
    FScopedCVar<int32> containedOff(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxContainedReject"),0,FName(TEXT("GroupedGeometryOracle")));
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        FBoxVolumeGroup group;for(double x:{-1.,1.})for(double y:{-1.,1.})for(double z:{-1.,1.})
            group.Corners.Add(Detail::Point(FVector3d(x,y,z)));
        TArray<FBoxVolumeGroup> groups;groups.Add(group);
        TArray<FVector3d> axes={FVector3d(1,0,0),FVector3d(0,1,0),FVector3d(0,0,1),FVector3d(1,2,3)};
        FIntervalPoint hull=Detail::Point(FVector3d(-1));hull=TimeDetail::Hull(hull,Detail::Point(FVector3d(1)));
        FDistanceBudget closeWork{1000};FGroupedBoxVolumeBounds volume;
        if(!CloseGroupedBoxVolumeBounds(hull,groups,axes,closeWork,volume))return false;
        for(bool crosses:{false,true})
        {
            FIntervalTensorSurface surface;surface.DegreeU=0;surface.DegreeV=2;
            // Clear arch requires actual subdivision: its root AABB intersects
            // the box, while each half is separated on X or Y. The other curve
            // enters the box and must remain denied despite all cached values.
            surface.ControlPoints={Detail::Point(FVector3d(2,0,0)),
                Detail::Point(crosses?FVector3d(0,0,0):FVector3d(2,2,0)),Detail::Point(FVector3d(0,2,0))};
            FDistanceBudget sceneWork{8192};FIndexedIntervalUnionPtr scene;
            if(!FIndexedIntervalUnion::Build(MakeArrayView(&surface,1),sceneWork,scene))return false;
            FReferenceBoundsCandidateSetPtr partition;
            if(!FReferenceBoundsCandidateSet::Select(scene,volume.BroadHull,sceneWork,partition))return false;
            for(int32 allowance=0;allowance<=160;++allowance)
            {
                FDistanceBudget oldBudget{allowance+2,17,2},coldBudget{allowance+2,17,2},warmBudget{allowance+2,17,2};
                FReferenceBoxVolumeStrictSeparationPtr oldProof,coldProof,warmProof;
                FGroupedBoxSeparationObservation oldSeen,coldSeen,warmSeen;
                const bool old=FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,oldBudget,oldProof,&oldSeen,false);
                const bool cold=FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,coldBudget,coldProof,&coldSeen,true);
                const bool warm=FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,warmBudget,warmProof,&warmSeen,true);
                if(!TestTrue(TEXT("OFF/cold/warm keep identical complete proof, refusal, debit and traversal"),
                    old==cold&&old==warm&&oldBudget.Used==coldBudget.Used&&oldBudget.Used==warmBudget.Used&&
                    oldBudget.Remaining==coldBudget.Remaining&&oldBudget.Remaining==warmBudget.Remaining&&
                    coldBudget.ConsumptionFloor==2&&warmBudget.ConsumptionFloor==2&&
                    oldSeen.VisitedCells==coldSeen.VisitedCells&&oldSeen.VisitedCells==warmSeen.VisitedCells&&
                    oldSeen.ActiveGroupMask==coldSeen.ActiveGroupMask&&oldSeen.ActiveGroupMask==warmSeen.ActiveGroupMask&&
                    oldSeen.TestedGroups==coldSeen.TestedGroups&&oldSeen.TestedGroups==warmSeen.TestedGroups&&
                    oldSeen.InheritedGroupSkips==coldSeen.InheritedGroupSkips&&oldSeen.InheritedGroupSkips==warmSeen.InheritedGroupSkips&&
                    FCString::Strcmp(oldSeen.Stage,coldSeen.Stage)==0&&FCString::Strcmp(oldSeen.Stage,warmSeen.Stage)==0&&
                    (!old||(oldProof->Matches(scene,volume)&&coldProof->Matches(scene,volume)&&warmProof->Matches(scene,volume)))&&
                    (old||(!oldProof&&!coldProof&&!warmProof))))return false;
                if(allowance==160&&!TestTrue(TEXT("clear arch splits and closes; crossing curve never clears"),
                    old!=crosses&&oldSeen.VisitedCells>1))return false;
            }
        }
    }
    return !HasAnyErrors();
}
#endif


// Static polygon theorem only. Dynamic local hull, edge containment, query
// policy, owner, candidate partition and native kernel remain fresh.
namespace Speed::Analytic::Certified::CanonicalPlaneStaticMemo
{
namespace A=::AnalyticCertified;namespace P=CanonicalAxialPlaneDetail;
static TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.CanonicalPlaneStaticMemo"),0,
    TEXT("Default OFF. Cache exact per-edge native polygon convexity facts; dynamic containment and every work debit stay unchanged."),ECVF_Default);
bool RuntimeEnabled(){return Enabled.GetValueOnAnyThread()==1;}
struct FCounts
{
    std::atomic<uint64> Calls{0},Finished{0},Ticks{0},PolygonHits{0},EdgeHits{0},EdgeBuilds{0},
        TurnsReplayed{0},TurnsEvaluated{0},OrientEvaluated{0},BudgetStops{0},True{0},False{0};
};
static FCounts Counts;
void PublishObservation()
{
    UE_LOG(LogTemp,Display,TEXT("[CanonicalPlaneStaticMemo] Scope=ProgramPrefixAtPreExit NoPASS=1 Calls=%llu Finished=%llu Ticks=%llu SecondsPerCycle=%.17g PolygonHits=%llu EdgeHits=%llu EdgeBuilds=%llu TurnsReplayed=%llu TurnsEvaluated=%llu OrientEvaluated=%llu BudgetStops=%llu True=%llu False=%llu"),
        Counts.Calls.load(),Counts.Finished.load(),Counts.Ticks.load(),FPlatformTime::GetSecondsPerCycle64(),Counts.PolygonHits.load(),
        Counts.EdgeHits.load(),Counts.EdgeBuilds.load(),Counts.TurnsReplayed.load(),Counts.TurnsEvaluated.load(),Counts.OrientEvaluated.load(),
        Counts.BudgetStops.load(),Counts.True.load(),Counts.False.load());
}
struct FEdge
{
    int32 Before=0,After=0;uint32 Sticky=0;bool Ready=false;
};
struct FPolygon
{
    TArray<FVector2d> Vertices;TArray<FEdge> Edges;uint64 Hash=0,Numeric=0;bool Ready=false;
};
struct FStore {FPolygon Entries[64];};
static FStore& Store(){static thread_local TUniquePtr<FStore> value;if(!value)value=MakeUnique<FStore>();return *value;}
FPolygon* Acquire(TConstArrayView<FVector2d> vertices,uint64 numeric,bool observe)
{
    if(vertices.Num()<3||vertices.Num()>256)return nullptr;
    uint64 hash=1469598103934665603ull;
    const auto add=[&](uint64 value){hash^=value;hash*=1099511628211ull;};add(numeric);add(uint64(vertices.Num()));
    for(const auto& p:vertices)
    {if(!A::Finite(p.X)||!A::Finite(p.Y))return nullptr;add(A::Bits(p.X));add(A::Bits(p.Y));}
    hash^=hash>>33;hash*=0xff51afd7ed558ccdull;hash^=hash>>33;hash*=0xc4ceb9fe1a85ec53ull;hash^=hash>>33;
    auto& entry=Store().Entries[hash&63u];bool same=entry.Ready&&entry.Hash==hash&&entry.Numeric==numeric&&entry.Vertices.Num()==vertices.Num();
    for(int32 i=0;same&&i<vertices.Num();++i)
        same=A::Bits(entry.Vertices[i].X)==A::Bits(vertices[i].X)&&A::Bits(entry.Vertices[i].Y)==A::Bits(vertices[i].Y);
    if(same){if(observe)++Counts.PolygonHits;return &entry;}
    entry=FPolygon{};entry.Vertices.Append(vertices.GetData(),vertices.Num());entry.Edges.SetNum(vertices.Num());
    entry.Hash=hash;entry.Numeric=numeric;entry.Ready=true;return &entry;
}
struct FLocalCounts {uint64 Hits=0,Builds=0,Replayed=0,Evaluated=0,Orients=0,Stops=0;};
static bool Run(const FCanonicalAxialPlaneBounds& value,const FIntervalPoint& hull,FDistanceBudget& budget,
    bool observe,FLocalCounts& counts)
{
    const auto& p=value.Plane;const auto n=p.DomainVertices.Num();
    // Literal original dynamic prefix. No polygon result is consulted before
    // local transformation, finite extents and its strict AABB checks finish.
    if(n<3||!Detail::ValidPoint(hull)||value.UAxis<0||value.VAxis<0)return false;
    FIntervalPoint local;local.Axis[2]=Interval::Exact(0);
    local.Axis[0]=A::Multiply(A::Subtract(hull.Axis[value.UAxis],Interval::Exact(p.Origin[value.UAxis])),Interval::Exact(p.AxisU[value.UAxis]));
    local.Axis[1]=A::Multiply(A::Subtract(hull.Axis[value.VAxis],Interval::Exact(p.Origin[value.VAxis])),Interval::Exact(p.AxisV[value.VAxis]));
    if(!Detail::ValidPoint(local)||local.Axis[0].Lower<=-p.HalfExtents.X||local.Axis[0].Upper>=p.HalfExtents.X||
        local.Axis[1].Lower<=-p.HalfExtents.Y||local.Axis[1].Upper>=p.HalfExtents.Y)return false;
    const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=EAnalyticNumericStatus::Complete)return false;
    auto* polygon=Acquire(p.DomainVertices,numeric.ControlFingerprint(),observe);
    const auto vertex=[&](int32 i){return Detail::Point(FVector3d(p.DomainVertices[i].X,p.DomainVertices[i].Y,0));};
    int32 orientation=0;
    for(int32 k=0;k<n;++k)
    {
        const int32 next=(k+1)%n;const auto a=vertex(k),b=vertex(next);
        if(P::SameFiniteNativeCoordinate(p.DomainVertices[k].X,p.DomainVertices[next].X)&&
            P::SameFiniteNativeCoordinate(p.DomainVertices[k].Y,p.DomainVertices[next].Y))return false;
        auto* edge=polygon?&polygon->Edges[k]:nullptr;
        if(edge&&edge->Ready&&edge->Before==orientation&&(VectorGetControlRegister()&edge->Sticky)==edge->Sticky)
        {
            // Replay the ORIGINAL prefix one debit at a time. A partial account
            // still stops at the same turn; no skipped edge can manufacture work.
            for(int32 j=0;j<n;++j)if(j!=k&&j!=next)
            {if(!budget.Consume()){++counts.Stops;return false;}++counts.Replayed;}
            orientation=edge->After;++counts.Hits;
        }
        else
        {
            const int32 before=orientation;
            for(int32 j=0;j<n;++j)if(j!=k&&j!=next)
            {
                if(!budget.Consume()){++counts.Stops;return false;}++counts.Evaluated;
                if(P::ProveFixedAxisCollinearity(p.DomainVertices[k],p.DomainVertices[next],p.DomainVertices[j]))continue;
                ++counts.Orients;const auto turn=P::Orient(a,b,vertex(j),0,1);
                const int32 sign=P::Positive(turn,1)?1:P::Positive(turn,-1)?-1:0;
                if(!sign||(orientation&&sign!=orientation))return false;orientation=sign;
            }
            // Only the completed static edge theorem is retained. A later hull
            // refusal or a later malformed edge cannot alter the caller's path.
            if(edge)
            {edge->Before=before;edge->After=orientation;edge->Sticky=VectorGetControlRegister()&0x3fu;edge->Ready=true;++counts.Builds;}
        }
        if(!budget.Consume()){++counts.Stops;return false;}
        if(!orientation||!P::Positive(P::Orient(a,b,local,0,1),orientation))return false;
    }
    return true;
}
bool NativeInterior(const FCanonicalAxialPlaneBounds& value,const FIntervalPoint& hull,FDistanceBudget& budget)
{
    const bool observe=StrongConvexCurveMinimum::Observation::Enabled();const uint64 started=observe?FPlatformTime::Cycles64():0;
    if(observe)++Counts.Calls;FLocalCounts counts;const bool result=Run(value,hull,budget,observe,counts);
    if(observe)
    {
        Counts.Ticks.fetch_add(FPlatformTime::Cycles64()-started);++Counts.Finished;
        Counts.EdgeHits.fetch_add(counts.Hits);Counts.EdgeBuilds.fetch_add(counts.Builds);Counts.TurnsReplayed.fetch_add(counts.Replayed);
        Counts.TurnsEvaluated.fetch_add(counts.Evaluated);Counts.OrientEvaluated.fetch_add(counts.Orients);Counts.BudgetStops.fetch_add(counts.Stops);
        if(result)++Counts.True;else++Counts.False;
    }
    return result;
}
}


#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCanonicalPlaneStaticMemoParityTest,
 "IAmSpeed.AnalyticWorld.CanonicalPlane.StaticMemoParity",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCanonicalPlaneStaticMemoParityTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    namespace P=CanonicalAxialPlaneDetail;namespace M=CanonicalPlaneStaticMemo;namespace A=::AnalyticCertified;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> profile(TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"),1,FName(TEXT("PlaneStaticMemoProfile")));
    TArray<TArray<FVector2d>> polygons;
    polygons.Add(TArray<FVector2d>{FVector2d(-2,-2),FVector2d(0,-2),FVector2d(2,-2),FVector2d(2,0),FVector2d(2,2),FVector2d(0,2),FVector2d(-2,2),FVector2d(-2,0)});
    auto reverse=polygons[0];for(int32 i=0;i<reverse.Num()/2;++i)Swap(reverse[i],reverse[reverse.Num()-1-i]);polygons.Add(reverse);
    polygons.Add(TArray<FVector2d>{FVector2d(-2,-2),FVector2d(2,-2),FVector2d(.2,.1),FVector2d(2,2),FVector2d(-2,2)}); // concave
    polygons.Add(TArray<FVector2d>{FVector2d(-2,-2),FVector2d(2,-2),FVector2d(2,2),FVector2d(-2,2),FVector2d(-2,2)}); // late zero edge
    polygons.Add(TArray<FVector2d>{FVector2d(0,0),FVector2d(1,1),FVector2d(2,2),FVector2d(0,2)}); // unproved diagonal collinearity
    polygons.Add(TArray<FVector2d>{FVector2d(-2.1,-1.9),FVector2d(2.3,-1.7),FVector2d(2.2,2.1),FVector2d(-1.8,2.4)});
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        FCanonicalAxialPlaneBounds value;value.UAxis=0;value.VAxis=1;value.NormalAxis=2;
        value.Plane.Origin=FVector3d::ZeroVector;value.Plane.AxisU=FVector3d(1,0,0);value.Plane.AxisV=FVector3d(0,1,0);
        value.Plane.HalfExtents=FVector2d(4,4);
        for(const auto& polygon:polygons)
        {
            value.Plane.DomainVertices=polygon;
            for(FVector3d center:{FVector3d(0,0,0),FVector3d(0,-3,0),FVector3d(3,0,0),FVector3d(0,3,0),FVector3d(5,0,0)})
            {
                auto hull=Detail::Point(center);hull.Axis[0]=A::Add(hull.Axis[0],Interval{-.03125,.03125,true});
                for(int32 allowance=0;allowance<=polygon.Num()*(polygon.Num()-1)+1;++allowance)
                {
                    const uint32 clear=VectorGetControlRegister()&~uint32(0x3f);
                    FDistanceBudget oldBudget{allowance+3,19,3},coldBudget{allowance+3,19,3},warmBudget{allowance+3,19,3};
                    VectorSetControlRegister(clear);const bool old=P::NativeInterior(value,hull,oldBudget,false);const uint32 oldFlags=VectorGetControlRegister();
                    VectorSetControlRegister(clear);const bool cold=P::NativeInterior(value,hull,coldBudget,true);const uint32 coldFlags=VectorGetControlRegister();
                    VectorSetControlRegister(clear);const bool warm=P::NativeInterior(value,hull,warmBudget,true);const uint32 warmFlags=VectorGetControlRegister();
                    if(!TestTrue(TEXT("per-edge hoist preserves results, exact refusal prefix, floor, debit and masked exception flags"),
                        old==cold&&old==warm&&oldBudget.Used==coldBudget.Used&&oldBudget.Used==warmBudget.Used&&
                        oldBudget.Remaining==coldBudget.Remaining&&oldBudget.Remaining==warmBudget.Remaining&&
                        coldBudget.ConsumptionFloor==3&&warmBudget.ConsumptionFloor==3&&oldFlags==coldFlags&&oldFlags==warmFlags))return false;
                }
            }
        }
        // A later invalid edge is never examined early. Hull rejection on the
        // first edge must still debit exactly its three static turns + one test.
        value.Plane.DomainVertices=polygons[3];FDistanceBudget warmBad{1000};
        (void)P::NativeInterior(value,Detail::Point(FVector3d::ZeroVector),warmBad,true);
        FDistanceBudget earlyOld{1000},earlyNew{1000};
        if(!TestTrue(TEXT("future malformed edge cannot precede current dynamic rejection"),
            !P::NativeInterior(value,Detail::Point(FVector3d(0,-3,0)),earlyOld,false)&&
            !P::NativeInterior(value,Detail::Point(FVector3d(0,-3,0)),earlyNew,true)&&earlyOld.Used==4&&earlyNew.Used==4))return false;
        // Same values copied to another plane remain pure geometric data. Origin,
        // local axes and the dynamic hull are always recomputed, not memoized.
        value.Plane.DomainVertices=polygons[0];FDistanceBudget first{1000};
        if(!P::NativeInterior(value,Detail::Point(FVector3d::ZeroVector),first,true))return false;
        value.Plane.DomainVertices[2].X=A::FromBits(A::Bits(2.)+1);
        value.Plane.Origin=FVector3d(10,-7,3);value.Plane.AxisU=FVector3d(-1,0,0);
        FDistanceBudget changedOld{1000},changedNew{1000};const auto changedHull=Detail::Point(FVector3d(10,-7,3));
        const bool a=P::NativeInterior(value,changedHull,changedOld,false),b=P::NativeInterior(value,changedHull,changedNew,true);
        if(!TestTrue(TEXT("one-ULP polygon key and changed dynamic local frame stay exact"),a==b&&changedOld.Used==changedNew.Used&&changedOld.Remaining==changedNew.Remaining))return false;
    }
    return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCanonicalPlaneStaticMemoLargePolygonTest,
 "IAmSpeed.AnalyticWorld.CanonicalPlane.StaticMemoLargePolygon",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCanonicalPlaneStaticMemoLargePolygonTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    namespace P=CanonicalAxialPlaneDetail;namespace M=CanonicalPlaneStaticMemo;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> profile(TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"),1,FName(TEXT("PlaneStaticLargeProfile")));
    FCanonicalAxialPlaneBounds value;value.UAxis=0;value.VAxis=1;value.NormalAxis=2;
    value.Plane.Origin=FVector3d::ZeroVector;value.Plane.AxisU=FVector3d(1,0,0);value.Plane.AxisV=FVector3d(0,1,0);value.Plane.HalfExtents=FVector2d(20,20);
    for(int32 k=0;k<13;++k)value.Plane.DomainVertices.Add(FVector2d(-13+2*k,-13));
    for(int32 k=0;k<13;++k)value.Plane.DomainVertices.Add(FVector2d(13,-13+2*k));
    for(int32 k=0;k<13;++k)value.Plane.DomainVertices.Add(FVector2d(13-2*k,13));
    for(int32 k=0;k<13;++k)value.Plane.DomainVertices.Add(FVector2d(-13,13-2*k));
    const auto hull=Detail::Point(FVector3d::ZeroVector);const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        FDistanceBudget warm{10000};if(!P::NativeInterior(value,hull,warm,true))return false;
        const uint64 turns=M::Counts.TurnsReplayed.load(),hits=M::Counts.EdgeHits.load();FDistanceBudget actual{10000};
        if(!TestTrue(TEXT("52 immutable edges avoid all2600 repeated static turn calculations"),P::NativeInterior(value,hull,actual,true)&&
            actual.Used==2652&&M::Counts.TurnsReplayed.load()==turns+2600&&M::Counts.EdgeHits.load()==hits+52))return false;
        for(int32 allowance:{0,1,17,64,2599,2600,2651,2652,2653})
        {
            FDistanceBudget oldBudget{allowance+2,7,2},newBudget{allowance+2,7,2};
            const bool old=P::NativeInterior(value,hull,oldBudget,false),now=P::NativeInterior(value,hull,newBudget,true);
            if(!TestTrue(TEXT("large cold and warm proofs consume the exact same bounded account"),old==now&&oldBudget.Used==newBudget.Used&&oldBudget.Remaining==newBudget.Remaining&&newBudget.ConsumptionFloor==2))return false;
        }
    }
    const uint32 valid=VectorGetControlRegister();VectorSetControlRegister((valid&~uint32(0x6000))|uint32(0x2000));
    FDistanceBudget invalid{10000};TestTrue(TEXT("incompatible numeric mode cannot consume a cached theorem or repair controls"),
        !P::NativeInterior(value,hull,invalid,true)&&invalid.Used==0&&(VectorGetControlRegister()&uint32(0x6000))==uint32(0x2000));
    return !HasAnyErrors();
}
#endif


namespace Speed::Analytic::Prepared::AffineProjectedObservation
{
namespace C=Certified;namespace A=::AnalyticCertified;
struct FKey {TArray<uint64,TInlineAllocator<96>> Words;uint64 Hash=0;};
struct FEntry {FKey Key;TArray<uint64,TInlineAllocator<32>> OriginalPath;bool Ready=false;};
struct FSet {FEntry Ways[4];uint8 Next=0;};
struct FStore {FSet Sets[64];};
struct FCounts
{
    std::atomic<uint64> Calls{0},Finished{0},Skipped{0},Misses{0},Same3DPathHits{0},Different3DPathHits{0},Replacements{0},
        KeyTicks{0},LookupTicks{0},TotalTicks{0},KeyWords{0},OriginalPathWords{0};
};
static FCounts Counts;
static FStore& Store(){static thread_local TUniquePtr<FStore> store;if(!store)store=MakeUnique<FStore>();return *store;}
static bool AppendPoint(const C::FIntervalPoint& point,TArray<uint64,TInlineAllocator<96>>& words)
{
    for(const auto& axis:point.Axis)
    {if(!axis.Known)return false;words.Add(A::Bits(axis.Lower));words.Add(A::Bits(axis.Upper));}
    return true;
}
static bool MakeKey(TConstArrayView<C::FIntervalTensorSurface> curves,const C::FIntervalBezierCenterPath& path,
    double radius,double timeGap,double spatialGap,int32 nodes,double pointWidth,double normalWidth,
    int32 refinements,uint64 numeric,FKey& out)
{
    out={};if(curves.IsEmpty()||curves.Num()>8||path.ControlPoints.IsEmpty()||path.ControlPoints.Num()>8)return false;
    auto& words=out.Words;words.Add(numeric);words.Add(uint64(curves.Num()));
    int32 total=0;
    for(const auto& curve:curves)
    {
        if(curve.ControlPoints.IsEmpty()||curve.ControlPoints.Num()>64||total>128-curve.ControlPoints.Num())return false;
        total+=curve.ControlPoints.Num();words.Add(uint64(curve.DegreeU));words.Add(uint64(curve.DegreeV));words.Add(uint64(curve.ControlPoints.Num()));
        for(const auto& cp:curve.ControlPoints)if(!AppendPoint(cp,words))return false;
    }
    words.Add(uint64(path.Degree));words.Add(uint64(path.ControlPoints.Num()));
    for(const auto& cp:path.ControlPoints)if(!AppendPoint(cp,words))return false;
    for(double value:{radius,timeGap,spatialGap,pointWidth,normalWidth})words.Add(A::Bits(value));
    words.Add(uint64(nodes));words.Add(uint64(refinements));
    uint64 hash=1469598103934665603ull;for(uint64 word:words){hash^=word;hash*=1099511628211ull;}
    hash^=hash>>33;hash*=0xff51afd7ed558ccdull;hash^=hash>>33;hash*=0xc4ceb9fe1a85ec53ull;hash^=hash>>33;out.Hash=hash;return true;
}
static bool MakeOriginalKey(const C::FIntervalBezierCenterPath& path,TArray<uint64,TInlineAllocator<32>>& out)
{
    out.Reset();if(path.ControlPoints.IsEmpty()||path.ControlPoints.Num()>8)return false;
    out.Add(uint64(path.Degree));out.Add(uint64(path.ControlPoints.Num()));
    for(const auto& cp:path.ControlPoints)for(const auto& axis:cp.Axis)
    {if(!axis.Known)return false;out.Add(A::Bits(axis.Lower));out.Add(A::Bits(axis.Upper));}
    return true;
}
enum class EMatch:uint8 {Miss,Same3DPath,Different3DPath};
static EMatch Observe(FStore& store,const FKey& key,const TArray<uint64,TInlineAllocator<32>>& original,bool& replaced)
{
    replaced=false;auto& set=store.Sets[key.Hash&63u];
    for(auto& entry:set.Ways)
    {
        if(!entry.Ready||entry.Key.Hash!=key.Hash||entry.Key.Words!=key.Words)continue;
        const bool same=entry.OriginalPath==original;
        // Classify relative to the most recent matching projected problem.
        // Retain values only; no contact/normal/UV/account can be retrieved.
        entry.OriginalPath=original;return same?EMatch::Same3DPath:EMatch::Different3DPath;
    }
    FEntry* chosen=nullptr;for(auto& entry:set.Ways)if(!entry.Ready){chosen=&entry;break;}
    if(!chosen){chosen=&set.Ways[set.Next++%4];replaced=true;}
    chosen->Ready=false;chosen->Key=key;chosen->OriginalPath=original;chosen->Ready=true;return EMatch::Miss;
}
void Record(TConstArrayView<C::FIntervalTensorSurface> curves,const C::FIntervalBezierCenterPath& projected,
    const C::FIntervalBezierCenterPath& original,double radius,double timeGap,double spatialGap,int32 nodes,
    double pointWidth,double normalWidth,int32 refinements,uint64 numeric)
{
    if(!C::StrongConvexCurveMinimum::Observation::Enabled())return;
    const uint64 start=FPlatformTime::Cycles64();++Counts.Calls;FKey key;TArray<uint64,TInlineAllocator<32>> originalKey;
    const bool known=MakeKey(curves,projected,radius,timeGap,spatialGap,nodes,pointWidth,normalWidth,refinements,numeric,key)&&MakeOriginalKey(original,originalKey);
    const uint64 keyed=FPlatformTime::Cycles64();Counts.KeyTicks.fetch_add(keyed-start);
    if(!known)++Counts.Skipped;
    else
    {
        Counts.KeyWords.fetch_add(key.Words.Num());Counts.OriginalPathWords.fetch_add(originalKey.Num());
        bool replaced=false;const auto match=Observe(Store(),key,originalKey,replaced);Counts.Replacements.fetch_add(uint64(replaced));
        if(match==EMatch::Miss)++Counts.Misses;
        else if(match==EMatch::Same3DPath)++Counts.Same3DPathHits;
        else ++Counts.Different3DPathHits;
    }
    const uint64 end=FPlatformTime::Cycles64();Counts.LookupTicks.fetch_add(end-keyed);Counts.TotalTicks.fetch_add(end-start);++Counts.Finished;
}
void PublishObservation()
{
    UE_LOG(LogTemp,Display,TEXT("[AffineProjectedProblemObservation] Scope=ProgramPrefixAtPreExit PassiveOnly=1 NoResultCache=1 NoGainClaim=1 TableEntriesPerThread=256 Ways=4 MaxCurves=8 MaxTotalCP=128 MaxPathCP=8 Full3DMeaning=OriginalPathBitsOnly Calls=%llu Finished=%llu Skipped=%llu Misses=%llu Same3DPathHits=%llu Different3DPathHits=%llu Replacements=%llu KeyTicks=%llu LookupTicks=%llu TotalTicks=%llu SecondsPerCycle=%.17g KeyWords=%llu OriginalPathWords=%llu"),
        Counts.Calls.load(),Counts.Finished.load(),Counts.Skipped.load(),Counts.Misses.load(),Counts.Same3DPathHits.load(),Counts.Different3DPathHits.load(),
        Counts.Replacements.load(),Counts.KeyTicks.load(),Counts.LookupTicks.load(),Counts.TotalTicks.load(),FPlatformTime::GetSecondsPerCycle64(),Counts.KeyWords.load(),Counts.OriginalPathWords.load());
}
}


#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAffineProjectedProblemObservationTest,
 "IAmSpeed.AnalyticWorld.Affine.ProjectedProblemObservationKeys",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAffineProjectedProblemObservationTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    namespace O=Speed::Analytic::Prepared::AffineProjectedObservation;namespace A=::AnalyticCertified;
    const auto originalControl=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(originalControl);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),originalControl.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    TArray<FIntervalTensorSurface> curves;FIntervalTensorSurface curve;curve.DegreeU=0;curve.DegreeV=1;
    curve.ControlPoints={Detail::Point(FVector3d(0,0,0)),Detail::Point(FVector3d(0,1,1))};curves.Add(curve);
    FIntervalBezierCenterPath original;original.Degree=1;original.ControlPoints={Detail::Point(FVector3d(-5,2,3)),Detail::Point(FVector3d(-5,4,5))};
    auto projected=original;for(auto& cp:projected.ControlPoints)cp.Axis[0]=Interval::Exact(0);
    const auto make=[&](O::FKey& key,TConstArrayView<FIntervalTensorSurface> nets,const FIntervalBezierCenterPath& path,
        double radius=2.,double timeGap=.001,double spatialGap=.0001,int32 nodes=512,double pointWidth=.01,double normalWidth=.02,int32 refinements=24,uint64 numeric=17)
    {return O::MakeKey(nets,path,radius,timeGap,spatialGap,nodes,pointWidth,normalWidth,refinements,numeric,key);};
    O::FKey base;if(!make(base,curves,projected))return false;
    TArray<uint64,TInlineAllocator<32>> pathKey;if(!O::MakeOriginalKey(original,pathKey))return false;
    auto store=MakeUnique<O::FStore>();bool replaced=false;
    if(!TestTrue(TEXT("first projected key misses, then same original3D path hits"),
        O::Observe(*store,base,pathKey,replaced)==O::EMatch::Miss&&!replaced&&
        O::Observe(*store,base,pathKey,replaced)==O::EMatch::Same3DPath&&!replaced))return false;
    auto otherOriginal=original;for(auto& cp:otherOriginal.ControlPoints)cp.Axis[0]=Interval::Exact(5);
    TArray<uint64,TInlineAllocator<32>> otherPath;if(!O::MakeOriginalKey(otherOriginal,otherPath))return false;
    if(!TestTrue(TEXT("opposite extrusion coordinate is observed as quotient reuse only"),
        O::Observe(*store,base,otherPath,replaced)==O::EMatch::Different3DPath&&
        O::Observe(*store,base,otherPath,replaced)==O::EMatch::Same3DPath))return false;
    auto changedCurve=curves;changedCurve[0].ControlPoints[1].Axis[2].Upper=A::FromBits(A::Bits(1.)+1);
    auto changedPath=projected;changedPath.ControlPoints[1].Axis[2].Upper=A::FromBits(A::Bits(5.)+1);
    O::FKey changed;
    if(!TestTrue(TEXT("one-ULP coefficient and projected-path changes remain different exact problems"),
        make(changed,changedCurve,projected)&&changed.Words!=base.Words&&make(changed,curves,changedPath)&&changed.Words!=base.Words))return false;
    for(int32 field=0;field<8;++field)
    {
        if(!make(changed,curves,projected,field==0?3.:2.,field==1?.002:.001,field==2?.0002:.0001,
            field==3?513:512,field==4?.02:.01,field==5?.03:.02,field==6?25:24,field==7?18:17))return false;
        if(!TestTrue(TEXT("every radius/threshold/local-limit/numeric field participates in the exact key"),changed.Words!=base.Words))return false;
    }
    changed=base;changed.Words.Last()^=1;changed.Hash=base.Hash;
    if(!TestTrue(TEXT("equal hash cannot replace full problem equality"),O::Observe(*store,changed,otherPath,replaced)==O::EMatch::Miss))return false;
    for(uint64 i=0;i<1024;++i)
    {changed=base;changed.Words.Last()=1000+i;changed.Hash=base.Hash;O::Observe(*store,changed,otherPath,replaced);}
    int32 entries=0;for(const auto& set:store->Sets)for(const auto& entry:set.Ways)entries+=entry.Ready?1:0;
    if(!TestTrue(TEXT("collision saturation stays bounded to the four ways of one set"),entries==4&&replaced))return false;
    changedCurve[0].ControlPoints[0].Axis[0].Known=false;
    if(!TestFalse(TEXT("unknown geometry is skipped by the passive census"),make(changed,changedCurve,projected)))return false;
    TArray<FIntervalTensorSurface> tooMany;for(int32 i=0;i<9;++i)tooMany.Add(curve);
    TestFalse(TEXT("bounded cardinality never grows with an oversized problem"),make(changed,tooMany,projected));
    return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGroupedBoxAabbFirstParityTest,
 "IAmSpeed.AnalyticWorld.GeneralBox.AabbFirstParity",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FGroupedBoxAabbFirstParityTest::RunTest(const FString&)
{
    FScopedCVar<int32> containedOff(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxContainedReject"),0,FName(TEXT("GroupedGeometryOracle")));
    using namespace Speed;using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    namespace M=GroupedBoxGeometryMemo;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxAabbFirst"),1,FName(TEXT("GroupedAabbTest")));
    FScopedCVar<int32> profile(TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"),1,FName(TEXT("GroupedAabbObserve")));
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        FBoxVolumeGroup group;
        for(double x:{-1.,1.})for(double y:{-1.,1.})for(double z:{-1.,1.})group.Corners.Add(Detail::Point(FVector3d(x,y,z)));
        TArray<FBoxVolumeGroup> groups;groups.Add(group);
        TArray<FVector3d> axes={FVector3d(1,0,0),FVector3d(0,1,0),FVector3d(0,0,1),FVector3d(.6,.8,0),FVector3d(0,.6,.8),FVector3d(.8,0,.6)};
        const auto hull=TimeDetail::Hull(Detail::Point(FVector3d(-1)),Detail::Point(FVector3d(1)));
        FDistanceBudget closeWork{1000};FGroupedBoxVolumeBounds volume;
        if(!CloseGroupedBoxVolumeBounds(hull,groups,axes,closeWork,volume))return false;
        const uint64 clearsBefore=M::AabbCounts.AllClear.load(),fallbackBefore=M::AabbCounts.Fallback.load();
        for(int32 scenario=0;scenario<3;++scenario)
        {
            FIntervalTensorSurface surface;surface.DegreeU=0;surface.DegreeV=2;
            if(scenario==0)surface.ControlPoints={Detail::Point(FVector3d(3,0,0)),Detail::Point(FVector3d(4,2,1)),Detail::Point(FVector3d(3,-2,-1))};
            else surface.ControlPoints={Detail::Point(FVector3d(2,0,0)),Detail::Point(scenario==1?FVector3d(2,2,0):FVector3d(0,0,0)),Detail::Point(FVector3d(0,2,0))};
            FDistanceBudget setup{1000};FIndexedIntervalUnionPtr scene;FReferenceBoundsCandidateSetPtr partition;
            if(!FIndexedIntervalUnion::Build(MakeArrayView(&surface,1),setup,scene)||!FReferenceBoundsCandidateSet::Select(scene,hull,setup,partition))return false;
            // The outside root may be rejected by the scene BVH. Exercise its
            // actual per-cell projection stage directly as well.
            Detail::FNode root;root.DegreeU=surface.DegreeU;root.DegreeV=surface.DegreeV;root.Net=surface.ControlPoints;
            FDistanceBudget projection{100};TArray<Interval,TInlineAllocator<8>> slabs;
            if(!M::ProjectCellForVolume(root,volume,1,projection,slabs,nullptr))return false;
            bool allClear=true;
            for(int32 g=0;g<volume.Groups.Num();++g)
            {
                bool clear=false;for(int32 a=0;a<axes.Num();++a)
                {const auto body=volume.Slabs[g*axes.Num()+a],s=slabs[a];clear|=body.Upper<s.Lower||s.Upper<body.Lower;}
                allClear&=clear;
            }
            if(!TestTrue(TEXT("only a complete all-group separation bypasses CP projection"),allClear==(scenario==0)&&projection.Used==uint64(axes.Num())))return false;
            for(int32 a=0;a<axes.Num();++a)
            {
                const auto exact=GroupedBoxDetail::ProjectHull(root.Net,axes[a]);
                if(!TestTrue(TEXT("successful cheap slabs enclose exact slabs; inconclusive route restores exact endpoints"),
                    scenario==0?(slabs[a].Lower<=exact.Lower&&slabs[a].Upper>=exact.Upper):GroupedBoxDetail::SameInterval(slabs[a],exact)))return false;
            }
            for(int32 allowance=0;allowance<=200;++allowance)
            {
                FDistanceBudget oldBudget{allowance+3,11,3},newBudget{allowance+3,11,3};
                FReferenceBoxVolumeStrictSeparationPtr oldProof,newProof;FGroupedBoxSeparationObservation oldSeen,newSeen;
                const bool old=FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,oldBudget,oldProof,&oldSeen,false);
                const bool now=FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,newBudget,newProof,&newSeen,true);
                if(!TestTrue(TEXT("all budgets retain original result, mask, traversal, failure prefix and exact charges"),
                    old==now&&oldBudget.Used==newBudget.Used&&oldBudget.Remaining==newBudget.Remaining&&newBudget.ConsumptionFloor==3&&
                    oldSeen.VisitedCells==newSeen.VisitedCells&&oldSeen.ActiveGroupMask==newSeen.ActiveGroupMask&&oldSeen.TestedGroups==newSeen.TestedGroups&&
                    oldSeen.InheritedGroupSkips==newSeen.InheritedGroupSkips&&FCString::Strcmp(oldSeen.Stage,newSeen.Stage)==0&&
                    (!old||(oldProof->Matches(scene,volume)&&newProof->Matches(scene,volume)))))return false;
                if(allowance==200&&!TestTrue(TEXT("far and arch clear, intersecting curve never clears"),old==(scenario!=2)))return false;
            }
        }
        // Two independent obligations: the first box is far from the tensor,
        // the second overlaps it. ANY-separated is not a complete proof.
        auto shifted=group;for(auto& p:shifted.Corners)p.Axis[0]=::AnalyticCertified::Add(p.Axis[0],Interval::Exact(3.));
        TArray<FBoxVolumeGroup> twoGroups=groups;twoGroups.Add(shifted);auto twoHull=group.Corners[0];
        for(const auto& g:twoGroups)for(const auto& p:g.Corners)twoHull=TimeDetail::Hull(twoHull,p);
        FGroupedBoxVolumeBounds two;FDistanceBudget closeTwo{1000};
        if(!CloseGroupedBoxVolumeBounds(twoHull,twoGroups,axes,closeTwo,two))return false;
        Detail::FNode twoRoot;twoRoot.DegreeU=0;twoRoot.DegreeV=2;
        twoRoot.Net={Detail::Point(FVector3d(3,0,0)),Detail::Point(FVector3d(4,2,1)),Detail::Point(FVector3d(3,-2,-1))};
        for(uint32 mask:{3u,2u,1u})
        {
            const uint64 before=M::AabbCounts.AllClear.load();FDistanceBudget account{100};TArray<Interval,TInlineAllocator<8>> result;
            if(!M::ProjectCellForVolume(twoRoot,two,mask,account,result,nullptr))return false;
            if(!TestTrue(TEXT("all active groups required; already excluded groups are not revived"),
                M::AabbCounts.AllClear.load()==before+(mask==1?1u:0u)&&account.Used==uint64(axes.Num())))return false;
            if(mask!=1)for(int32 a=0;a<axes.Num();++a)
                if(!TestTrue(TEXT("one separated obligation cannot suppress exact projections for another"),
                    GroupedBoxDetail::SameInterval(result[a],GroupedBoxDetail::ProjectHull(twoRoot.Net,axes[a]))))return false;
        }
        if(!TestTrue(TEXT("both coarse success and literal fallback really executed"),M::AabbCounts.AllClear.load()>clearsBefore&&M::AabbCounts.Fallback.load()>fallbackBefore))return false;
    }
    return !HasAnyErrors();
}
#endif
#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Analytic::Certified::TimeDetail
{
struct FMonotoneDyadicHintTestAccess
{
    static void Hint(FRetainedSphereTimeSearch& s,double a,double b)
    {s.DyadicHintProved=true;s.DyadicHintLower=a;s.DyadicHintUpper=b;}
    static bool Skip(const FRetainedSphereTimeSearch& s,const FTimeNode& n,bool anchored,double gap,double leaf)
    {return s.CanSkipDyadicNodeBounds(n,anchored,gap,leaf);}
    static void Continued(FRetainedSphereTimeSearch& s,bool value){s.HadBracket=value;}
};
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMonotoneDyadicHintIdentityTest,
 "IAmSpeed.AnalyticWorld.TensorTime.MonotoneDyadicHint.OriginalFrontierAndWitnessBits",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FMonotoneDyadicHintIdentityTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;namespace T=TimeDetail;namespace A=::AnalyticCertified;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const auto same=[](const FTensorSphereTimeBounds& a,const FTensorSphereTimeBounds& b)
    {
        return a.Status==b.Status&&a.InitialSeparationProved==b.InitialSeparationProved&&
            A::Bits(a.LowerTime)==A::Bits(b.LowerTime)&&A::Bits(a.UpperTime)==A::Bits(b.UpperTime)&&
            a.InsideWitnessSurfaceIndex==b.InsideWitnessSurfaceIndex&&
            A::Bits(a.InsideSampleUV.X)==A::Bits(b.InsideSampleUV.X)&&A::Bits(a.InsideSampleUV.Y)==A::Bits(b.InsideSampleUV.Y)&&
            MonotoneSeedTests::SamePoint(a.InsideSurfacePoint,b.InsideSurfacePoint);
    };
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        uint64 totalSkips=0;
        for(int32 fixture=0;fixture<7;++fixture)
        {
            VectorSetControlRegister((saved&~uint32(0x8040))|flush);
            TArray<FIntervalTensorSurface> surfaces;FIntervalBezierCenterPath path;double radius=1.;
            if(fixture<=1||fixture==3)
            {
                surfaces.Add(MonotoneSeedTests::Parabola());
                FVector3d start(9,6,0),end(9.5,5.5,0);
                if(fixture==1)
                {
                    const FVector3d shift(3,-7,2);start+=shift;end+=shift;
                    for(int32 i=0;i<6;++i)surfaces[0].ControlPoints[i]=Detail::Point(FVector3d(4*i,i*(i-1),0)+shift);
                }
                if(fixture==3){start=FVector3d(6,9,0);end=FVector3d(10,5,0);radius=2.;}
                if(!CenterDetail::Linear(start,end,path))return false;
            }
            else
            {
                FIntervalTensorSurface line;line.DegreeU=0;line.DegreeV=5;
                for(int32 i=0;i<6;++i)line.ControlPoints.Add(Detail::Point(FVector3d(4*i,0,0)));surfaces.Add(line);radius=2.;
                if(fixture==4)
                {path.Degree=2;path.ControlPoints={Detail::Point(FVector3d(-5,0,0)),Detail::Point(FVector3d(5,0,0)),Detail::Point(FVector3d(-5,0,0))};}
                else if(!CenterDetail::Linear(FVector3d(-5,0,0),FVector3d(fixture==5?-3.:fixture==6?-1.999:-1.,0,0),path))return false;
            }
            FDistanceBudget oldWork{1000000},hintWork{1000000};
            T::FRetainedSphereTimeSearch old(surfaces,path,radius,oldWork,1.e-7,512,nullptr,false,nullptr,true,false,false,false,false);
            T::FRetainedSphereTimeSearch hint(surfaces,path,radius,hintWork,1.e-7,512,nullptr,false,nullptr,true,false,true,false,false);
            auto expected=old.Advance(1.e-4);auto actual=hint.Advance(1.e-4);
            const bool matching=same(expected,actual)&&old.GetFailureForInspection()==hint.GetFailureForInspection()&&
                old.GetTimeNodeVisitsForInspection()==hint.GetTimeNodeVisitsForInspection();
            if(!matching)AddInfo(FString::Printf(TEXT("Dyadic hint mismatch FTZ=%u fixture=%d status=%d/%d time=[%.17g,%.17g]/[%.17g,%.17g] nodes=%llu/%llu skipped=%llu proved=%d work=%llu/%llu"),
                flush,fixture,int32(expected.Status),int32(actual.Status),expected.LowerTime,expected.UpperTime,actual.LowerTime,actual.UpperTime,
                old.GetTimeNodeVisitsForInspection(),hint.GetTimeNodeVisitsForInspection(),hint.GetDyadicHintSkipsForInspection(),hint.HasDyadicHintForInspection()?1:0,oldWork.Used,hintWork.Used));
            if(!TestTrue(TEXT("original dyadic bracket, provider, UV and original surface-point endpoints are bit-identical"),matching&&expected.Status!=ETensorSphereTimeStatus::Unknown))return false;
            totalSkips+=hint.GetDyadicHintSkipsForInspection();
            const uint64 firstSkips=hint.GetDyadicHintSkipsForInspection();
            if(expected.Status==ETensorSphereTimeStatus::FirstContactBracket)
            {
                expected=old.Advance(1.e-5);actual=hint.Advance(1.e-5);
                if(!TestTrue(TEXT("continuation uses every old retained frontier operation and imports no hint witness"),same(expected,actual)&&
                    old.GetTimeNodeVisitsForInspection()==hint.GetTimeNodeVisitsForInspection()&&hint.GetDyadicHintSkipsForInspection()==firstSkips))return false;
            }
            if(fixture==3||fixture==4||fixture==5)
                if(!TestTrue(TEXT("inapplicable global or nonmonotone hints leave the old route"),!hint.HasDyadicHintForInspection()&&hint.GetDyadicHintSkipsForInspection()==0))return false;
            if(!TestTrue(TEXT("same cumulative accounts remain conserved without refund"),oldWork.Used+uint64(oldWork.Remaining)==1000000&&hintWork.Used+uint64(hintWork.Remaining)==1000000))return false;
        }
        if(!TestTrue(TEXT("the prototype actually skips intermediate bounds in each FTZ/DAZ mode"),totalSkips>0))return false;
    }
    return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMonotoneDyadicHintGuardTest,
 "IAmSpeed.AnalyticWorld.TensorTime.MonotoneDyadicHint.NoAnchorOrWitnessReplacement",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FMonotoneDyadicHintGuardTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;namespace T=TimeDetail;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    TArray<FIntervalTensorSurface> surfaces;surfaces.Add(MonotoneSeedTests::Parabola());
    FIntervalBezierCenterPath path;if(!CenterDetail::Linear(FVector3d(9,6,0),FVector3d(9.5,5.5,0),path))return false;
    FDistanceBudget budget{10000};T::FRetainedSphereTimeSearch search(surfaces,path,1.,budget,1.e-7,512,nullptr,false,nullptr,true,false,true,false,false);
    T::FMonotoneDyadicHintTestAccess::Hint(search,.58,.59);T::FTimeNode node{0.,1.,0,path};
    const auto skip=[&](bool anchored=false,double gap=.01,double leaf=.0025)
    {return T::FMonotoneDyadicHintTestAccess::Skip(search,node,anchored,gap,leaf);};
    if(!TestTrue(TEXT("strict-inside-containing wide original node may skip bounds"),skip()))return false;
    if(!TestFalse(TEXT("an established earlier unresolved anchor is never bypassed"),skip(true)))return false;
    if(!TestFalse(TEXT("candidate already returnable is evaluated normally"),skip(false,2.,.1)))return false;
    if(!TestFalse(TEXT("leaf-width threshold is independently preserved"),skip(false,.01,2.)))return false;
    node.A=.585;if(!TestFalse(TEXT("only containing the upper witness does not authorize the whole hint"),skip()))return false;node.A=0.;
    node.B=.585;if(!TestFalse(TEXT("the complete certified bracket must belong to the node"),skip()))return false;node.B=1.;
    node.Depth=52;if(!TestFalse(TEXT("maximum original dyadic depth is unchanged"),skip()))return false;node.Depth=0;
    node.ForceSubdivision=true;if(!TestFalse(TEXT("forced anchor split retains its own original route"),skip()))return false;node.ForceSubdivision=false;
    node.RetainSpatial=true;if(!TestFalse(TEXT("retained spatial state cannot be skipped"),skip()))return false;node.RetainSpatial=false;
    node.Spatial=MakeUnique<T::FUnionSpatialState>();if(!TestFalse(TEXT("owned spatial solver cannot be skipped"),skip()))return false;node.Spatial.Reset();
    node.SpatialSeed=MakeShared<T::FUnionSpatialState>();if(!TestFalse(TEXT("shared spatial solver cannot be skipped"),skip()))return false;node.SpatialSeed.Reset();
    node.Endpoint=T::FTimeEndpointProof{};if(!TestFalse(TEXT("an existing original endpoint proof stays on its original route"),skip()))return false;node.Endpoint.Reset();
    T::FMonotoneDyadicHintTestAccess::Continued(search,true);if(!TestFalse(TEXT("returned frontier continuation never uses the hint shortcut"),skip()))return false;
    return !HasAnyErrors();
}
#endif

namespace Speed::Analytic::Certified
{
namespace CoordinateLineTimeHintControl
{
static TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.CoordinateLineTimeHint"),0,
    TEXT("Default OFF. Exact coordinate-line root enclosure independently enables old-grid hints. MonotoneCurveDyadicHint separately allows the general curve proof on a line miss; no output UV or geometry substitution."),ECVF_Default);
struct FCounts{std::atomic<uint64> Calls{0},Shape{0},Proved{0},NotApplicable{0},Work{0},Arithmetic{0},Used{0},Ticks{0};};
static FCounts Counts;
bool RuntimeEnabled(){return Enabled.GetValueOnAnyThread()==1;}
void PublishObservation()
{
    UE_LOG(LogTemp,Display,TEXT("[CoordinateLineTimeHint] Scope=ProgramPrefixAtPreExit DefaultOFF=1 OldDyadicGridOnly=1 NoOutputWitness=1 Calls=%llu Shape=%llu Proved=%llu NotApplicable=%llu Work=%llu Arithmetic=%llu Used=%llu Ticks=%llu SecondsPerCycle=%.17g"),
        Counts.Calls.load(),Counts.Shape.load(),Counts.Proved.load(),Counts.NotApplicable.load(),Counts.Work.load(),Counts.Arithmetic.load(),Counts.Used.load(),Counts.Ticks.load(),FPlatformTime::GetSecondsPerCycle64());
}
}
namespace CoordinateLineTimeHintDetail
{
struct FRootCertificate
{
    Interval Time;
    int32 NormalAxis=INDEX_NONE,TangentAxis=INDEX_NONE,Side=0;
    double ClearPrefix=0,InsideTime=1;
};
using E=TimeDetail::EMonotoneCurveTimeSeed;
struct FWork
{
    FDistanceBudget& Budget;uint64 Initial;
    bool Pay(){return Budget.Used>=Initial&&Budget.Used-Initial<64&&Budget.Consume();}
};
static E Try(TConstArrayView<FIntervalTensorSurface> surfaces,const FIntervalBezierCenterPath& path,
    double radius,FDistanceBudget& budget,double requestedGap,double& lower,double& upper,bool observe,
    FRootCertificate* certificate=nullptr,bool rootLocalCoverage=false,int32* refusalReason=nullptr)
{
    namespace A=::AnalyticCertified;lower=0;upper=1;if(certificate)*certificate={};
    if(refusalReason)*refusalReason=1;
    const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=EAnalyticNumericStatus::Complete)return E::Arithmetic;
    if(budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor)return E::Work;
    if(surfaces.Num()!=1||surfaces[0].DegreeU!=0||surfaces[0].DegreeV!=5||surfaces[0].ControlPoints.Num()!=6||
        path.Degree!=1||path.ControlPoints.Num()!=2)return E::NotApplicable;
    if(observe)++CoordinateLineTimeHintControl::Counts.Shape;
    // Optional bounded work only. The ordinary solver remains available when
    // the account cannot reserve this small proof; nothing is credited back.
    if(uint64(int64(budget.Remaining)-budget.ConsumptionFloor)<64||budget.Used>MAX_uint64-64)return E::NotApplicable;
    FWork work{budget,budget.Used};const auto& curve=surfaces[0];
    for(const auto& point:curve.ControlPoints){if(!work.Pay())return E::Work;if(!Detail::ValidPoint(point))return E::Arithmetic;}
    for(const auto& point:path.ControlPoints){if(!work.Pay())return E::Work;if(!Detail::ValidPoint(point))return E::Arithmetic;}
    if(!work.Pay())return E::Work;
    const uint64 radiusBits=A::Bits(radius);
    if(!TimeDetail::NonnegativeFinite(radius)||!TimeDetail::NonnegativeFinite(requestedGap))return E::Arithmetic;
    if((radiusBits&A::Magnitude)<A::MinNormal||!(requestedGap>0))return E::NotApplicable;
    if(refusalReason)*refusalReason=2;
    int32 tangent=INDEX_NONE,constants[2]{},constantCount=0;
    for(int32 d=0;d<3;++d)
    {
        if(!work.Pay())return E::Work;const auto first=curve.ControlPoints[0].Axis[d];
        bool constant=A::Bits(first.Lower)==A::Bits(first.Upper);
        for(const auto& point:curve.ControlPoints)constant&=A::Bits(point.Axis[d].Lower)==A::Bits(first.Lower)&&A::Bits(point.Axis[d].Upper)==A::Bits(first.Upper);
        if(constant){if(constantCount<2)constants[constantCount]=d;++constantCount;}else tangent=d;
    }
    if(constantCount!=2||tangent==INDEX_NONE)return E::NotApplicable;
    if(refusalReason)*refusalReason=3;
    bool increasing=true,decreasing=true;
    for(int32 i=0;i<5;++i)
    {
        if(!work.Pay())return E::Work;
        const auto slope=A::Multiply(Interval::Exact(5),A::Subtract(curve.ControlPoints[i+1].Axis[tangent],curve.ControlPoints[i].Axis[tangent]));
        if(!A::Valid(slope))return E::NotApplicable;
        increasing&=slope.Lower>0;decreasing&=slope.Upper<0;
    }
    if(!increasing&&!decreasing)return E::NotApplicable;
    if(!work.Pay())return E::Work;
    const auto low=curve.ControlPoints[increasing?0:5].Axis[tangent],high=curve.ControlPoints[increasing?5:0].Axis[tangent];
    const auto hull=CenterDetail::Hull(path);
    if(!Detail::ValidPoint(hull))return E::Arithmetic;
    if(refusalReason)*refusalReason=4;
    if(!rootLocalCoverage&&!(hull.Axis[tangent].Lower>low.Upper&&hull.Axis[tangent].Upper<high.Lower))return E::NotApplicable;
    if(refusalReason)*refusalReason=5;
    int32 normal=INDEX_NONE;
    for(int32 candidate=0;candidate<2;++candidate)
    {
        if(!work.Pay())return E::Work;
        const int32 fixed=constants[candidate];const auto origin=curve.ControlPoints[0].Axis[fixed];bool same=true;
        for(const auto& point:path.ControlPoints)same&=A::Bits(point.Axis[fixed].Lower)==A::Bits(origin.Lower)&&A::Bits(point.Axis[fixed].Upper)==A::Bits(origin.Upper);
        if(same){normal=constants[1-candidate];break;}
    }
    if(normal==INDEX_NONE)return E::NotApplicable;
    if(refusalReason)*refusalReason=6;
    const auto n0=path.ControlPoints[0].Axis[normal],n1=path.ControlPoints[1].Axis[normal],height=curve.ControlPoints[0].Axis[normal];
    if(A::Bits(n0.Lower)!=A::Bits(n0.Upper)||A::Bits(n1.Lower)!=A::Bits(n1.Upper))return E::NotApplicable;
    if(!work.Pay())return E::Work;
    const auto delta0=A::Subtract(n0,height),step=A::Subtract(n1,n0),r2=A::Square(Interval::Exact(radius));
    if(!A::Valid(delta0)||!A::Valid(step)||!A::Valid(r2))return E::NotApplicable;
    int32 side=0;if(delta0.Lower>radius&&step.Upper<0)side=1;else if(delta0.Upper< -radius&&step.Lower>0)side=-1;
    if(!side)return E::NotApplicable;
    if(!work.Pay())return E::Work;
    auto numerator=A::Subtract(A::Add(height,A::Multiply(Interval::Exact(double(side)),Interval::Exact(radius))),n0);
    auto denominator=step;if(side>0){numerator=A::Multiply(numerator,Interval::Exact(-1));denominator=A::Multiply(denominator,Interval::Exact(-1));}
    if(refusalReason)*refusalReason=7;
    Interval root;if(!ContactDetail::ProjectionDividePositive(numerator,denominator,root))return E::NotApplicable;
    if(!(root.Lower>0&&root.Upper<1))return E::NotApplicable;
    if(rootLocalCoverage)
    {
        if(refusalReason)*refusalReason=8;
        if(!work.Pay())return E::Work;
        const auto rootCenter=CenterDetail::Evaluate(path,root);
        if(!Detail::ValidPoint(rootCenter)||!(rootCenter.Axis[tangent].Lower>low.Upper&&
            rootCenter.Axis[tangent].Upper<high.Lower))return E::NotApplicable;
        // Every point of this finite line has normal coordinate height.
        // Its distance is ALWAYS >= abs(n(t)-height), even outside its
        // tangent span. Thus the signed quotient already excludes every
        // finite cap before RootLower. Coverage is only needed AT the root.
    }
    // The ordinary hint keeps whole-path tangent coverage. The optional
    // union proof instead has coverage at EVERY first root, and checks the
    // original strict-inside witness below. Both contracts retain the same
    // outward quotient and original finite geometry. No cap is substituted.
    const double a=A::FromBits(A::Bits(root.Lower)-1);double increment=.25*requestedGap;
    if(!A::Finite(increment)||!(increment>0))return E::NotApplicable;
    for(int32 attempt=0;attempt<8;++attempt)
    {
        if(!work.Pay()||!work.Pay())return E::Work;
        volatile double proposed=root.Upper+increment;double b=proposed;
        if(!(b>root.Upper))b=A::FromBits(A::Bits(root.Upper)+1);
        if(b>1)b=1;
        if(!A::Finite(b)||!(b>root.Upper&&b<=1))return E::NotApplicable;
        const auto center=CenterDetail::Evaluate(path,Interval::Exact(b));
        if(!Detail::ValidPoint(center))return E::NotApplicable;
        const auto distance=A::Square(A::Subtract(center.Axis[normal],height));
        if(A::Valid(distance)&&distance.Upper<r2.Lower)
        {
            if(refusalReason)*refusalReason=9;
            if(rootLocalCoverage&&!(center.Axis[tangent].Lower>low.Upper&&center.Axis[tangent].Upper<high.Lower))return E::NotApplicable;
            const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
            if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=numeric.ControlFingerprint())return E::Arithmetic;
            // Optional pure certificate of EVERY first contact. The padded
            // clear/inside bracket is retained separately from the quotient.
            if(certificate)
            {certificate->Time=root;certificate->NormalAxis=normal;certificate->TangentAxis=tangent;
             certificate->Side=side;certificate->ClearPrefix=a;certificate->InsideTime=b;}
            if(refusalReason)*refusalReason=0;
            lower=a;upper=b;return E::Proved;
        }
        increment*=2;if(!A::Finite(increment))return E::NotApplicable;
    }
    return E::NotApplicable;
}
}
namespace TimeDetail
{
EMonotoneCurveTimeSeed TryCoordinateLineTimeHint(TConstArrayView<FIntervalTensorSurface> surfaces,
    const FIntervalBezierCenterPath& path,double radius,FDistanceBudget& budget,double gap,double& lower,double& upper)
{
    const bool observe=StrongConvexCurveMinimum::Observation::Enabled();
    auto& counts=CoordinateLineTimeHintControl::Counts;const uint64 start=observe?FPlatformTime::Cycles64():0,before=budget.Used;
    if(observe)++counts.Calls;
    const auto result=CoordinateLineTimeHintDetail::Try(surfaces,path,radius,budget,gap,lower,upper,observe);
    if(observe)
    {
        counts.Ticks.fetch_add(FPlatformTime::Cycles64()-start);if(budget.Used>=before)counts.Used.fetch_add(budget.Used-before);
        if(result==EMonotoneCurveTimeSeed::Proved)++counts.Proved;else if(result==EMonotoneCurveTimeSeed::NotApplicable)++counts.NotApplicable;
        else if(result==EMonotoneCurveTimeSeed::Work)++counts.Work;else ++counts.Arithmetic;
    }
    return result;
}
}
}

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCoordinateLineTimeHintTest,"IAmSpeed.AnalyticWorld.CoordinateLineTimeHint.ExactRootAndOldGrid",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCoordinateLineTimeHintTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;namespace A=::AnalyticCertified;using E=TimeDetail::EMonotoneCurveTimeSeed;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("valid RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> flag(TEXT("p.IAmSpeed.AnalyticWorld.CoordinateLineTimeHint"),1,FName(TEXT("CoordinateLineTimeHintTest")));
    FScopedCVar<int32> planar(TEXT("p.IAmSpeed.AnalyticWorld.PlanarFamilyTimeShortcut"),1,FName(TEXT("CoordinateLineTimeHintPlanar")));
    FIntervalTensorSurface curve;curve.DegreeU=0;curve.DegreeV=5;
    // Same strictly monotone but non-affine parameterization as the four real walls.
    const double heights[6]{256,307.2,358.4,1331.2,1433.6,1536};
    for(double z:heights)curve.ControlPoints.Add(Detail::Point(FVector3d(0,0,z)));
    TArray<FIntervalTensorSurface> surfaces;surfaces.Add(curve);
    FIntervalBezierCenterPath path;path.Degree=1;path.ControlPoints={Detail::Point(FVector3d(0,-4,600)),Detail::Point(FVector3d(0,4,600.5))};
    for(uint32 mode=0;mode<4;++mode)
    {
        _mm_setcsr((_mm_getcsr()&~(0x8040u|0x3fu))|((mode&1)?0x40u:0)|((mode&2)?0x8000u:0));
        for(int32 direction:{-1,1})
        {
            auto directed=path;for(auto& p:directed.ControlPoints)p.Axis[1]=Interval::Exact(direction*p.Axis[1].Lower);
            FDistanceBudget budget{1048576};double lower=0,upper=1;
            const auto status=TimeDetail::TryCoordinateLineTimeHint(surfaces,directed,1.,budget,1.e-5,lower,upper);
            if(!TestTrue(TEXT("both approach directions contain exact root3/8 with strict-inside upper"),status==E::Proved&&lower<.375&&upper>.375&&upper-lower<1.e-5&&budget.Used<=64))return false;
            const auto center=CenterDetail::Evaluate(directed,Interval::Exact(upper));
            const auto squared=A::Square(center.Axis[1]);if(!TestTrue(TEXT("returned upper is strictly inside original normal line"),A::Valid(squared)&&squared.Upper<1))return false;
            FDistanceBudget offBudget{1048576},onBudget{1048576};TUniquePtr<FPlanarFamilyTimeShortcutPlan> offPlan,onPlan;
            if(FPlanarFamilyTimeShortcutPlan::Build(surfaces,directed,offBudget,offPlan,true)!=EPlanarFamilyBuild::Ready||
                FPlanarFamilyTimeShortcutPlan::Build(surfaces,directed,onBudget,onPlan,true)!=EPlanarFamilyBuild::Ready)return false;
            TimeDetail::FRetainedSphereTimeSearch off(surfaces,directed,1.,offBudget,0.,8192,nullptr,false,offPlan.Get(),true,false,false,false,false);
            TimeDetail::FRetainedSphereTimeSearch on(surfaces,directed,1.,onBudget,0.,8192,nullptr,false,onPlan.Get(),true,false,false,true,false);
            const auto compare=[&](const FTensorSphereTimeBounds& x,const FTensorSphereTimeBounds& y)
            {
                if(x.Status!=y.Status||x.InitialSeparationProved!=y.InitialSeparationProved||x.InsideWitnessSurfaceIndex!=y.InsideWitnessSurfaceIndex||
                    A::Bits(x.LowerTime)!=A::Bits(y.LowerTime)||A::Bits(x.UpperTime)!=A::Bits(y.UpperTime)||
                    A::Bits(x.InsideSampleUV.X)!=A::Bits(y.InsideSampleUV.X)||A::Bits(x.InsideSampleUV.Y)!=A::Bits(y.InsideSampleUV.Y))return false;
                for(int32 d=0;d<3;++d)if(x.InsideSurfacePoint.Axis[d].Known!=y.InsideSurfacePoint.Axis[d].Known||
                    A::Bits(x.InsideSurfacePoint.Axis[d].Lower)!=A::Bits(y.InsideSurfacePoint.Axis[d].Lower)||
                    A::Bits(x.InsideSurfacePoint.Axis[d].Upper)!=A::Bits(y.InsideSurfacePoint.Axis[d].Upper))return false;
                return true;
            };
            const auto a=off.Advance(1.e-5),b=on.Advance(1.e-5);
            if(!TestTrue(TEXT("line hint keeps old dyadic result and original UV"),a.Status==ETensorSphereTimeStatus::FirstContactBracket&&compare(a,b)&&on.GetDyadicHintSkipsForInspection()>0&&onBudget.Used<offBudget.Used))return false;
            const auto a2=off.Advance(2.5e-6),b2=on.Advance(2.5e-6);
            if(!TestTrue(TEXT("continuation keeps old result and UV"),a2.Status==ETensorSphereTimeStatus::FirstContactBracket&&compare(a2,b2)))return false;
        }
        double lower=0,upper=1;FDistanceBudget small{63,0,0};
        if(!TestTrue(TEXT("optional proof does not spend an unavailable64 reserve"),TimeDetail::TryCoordinateLineTimeHint(surfaces,path,1.,small,1.e-5,lower,upper)==E::NotApplicable&&small.Used==0))return false;
        FDistanceBudget edgeBudget{1048576};
        if(!TestTrue(TEXT("zero radius and zero gap retain the original generic route"),
            TimeDetail::TryCoordinateLineTimeHint(surfaces,path,0.,edgeBudget,1.e-5,lower,upper)==E::NotApplicable&&
            TimeDetail::TryCoordinateLineTimeHint(surfaces,path,1.,edgeBudget,0.,lower,upper)==E::NotApplicable))return false;
        auto outside=path;outside.ControlPoints[0].Axis[2]=Interval::Exact(200);FDistanceBudget budget{1048576};
        if(!TestTrue(TEXT("finite tangent endpoints remain mandatory"),TimeDetail::TryCoordinateLineTimeHint(surfaces,outside,1.,budget,1.e-5,lower,upper)==E::NotApplicable))return false;
        auto offAxis=path;offAxis.ControlPoints[0].Axis[0]=Interval::Exact(.1);budget={1048576};
        if(!TestTrue(TEXT("nonconstant orthogonal path remains generic"),TimeDetail::TryCoordinateLineTimeHint(surfaces,offAxis,1.,budget,1.e-5,lower,upper)==E::NotApplicable))return false;
        auto family=surfaces;family[0].ControlPoints[2].Axis[2].Lower=A::FromBits(A::Bits(358.4)-1);family[0].ControlPoints[2].Axis[2].Upper=A::FromBits(A::Bits(358.4)+1);budget={1048576};
        if(!TestTrue(TEXT("independent tangent coefficient intervals retain the same exact line image"),TimeDetail::TryCoordinateLineTimeHint(family,path,1.,budget,1.e-5,lower,upper)==E::Proved))return false;
        family=surfaces;family[0].ControlPoints[2].Axis[1].Upper=A::FromBits(1);budget={1048576};
        if(!TestTrue(TEXT("a forged subnormal interval endpoint is rejected before geometric admission"),TimeDetail::TryCoordinateLineTimeHint(family,path,1.,budget,1.e-5,lower,upper)==E::Arithmetic))return false;
        family=surfaces;family[0].ControlPoints[2].Axis[1].Upper=A::FromBits(A::MinNormal);budget={1048576};
        if(!TestTrue(TEXT("a valid nonconstant normal coefficient may not be treated as a plane"),TimeDetail::TryCoordinateLineTimeHint(family,path,1.,budget,1.e-5,lower,upper)==E::NotApplicable))return false;
    }
    return !HasAnyErrors();
}
#endif


namespace Speed::Analytic::Certified::StrongConvexCurveMinimum
{
enum class EChordEnvelopeBuild:uint8 {Ready,Unavailable,Work,Arithmetic};
static bool ChordFixedPoint(const FIntervalPoint& point,FVector3d& out)
{
    if(!Detail::ValidPoint(point))return false;
    for(int32 d=0;d<3;++d)
    {
        // This is only a proposed fixed binary64 chord vertex. The interval
        // proof below includes the complete original endpoint boxes.
        out[d]=.5*point.Axis[d].Lower+.5*point.Axis[d].Upper;
        const uint64 magnitude=A::Bits(out[d])&A::Magnitude;
        if(magnitude>=A::Infinity||(magnitude!=0&&magnitude<A::MinNormal))return false;
    }
    return true;
}
static bool BuildChordEnvelope(const FCurve& c,FChordEnvelope& out)
{
    out={};if(c.Num()!=6||!ChordFixedPoint(c[0],out.A)||!ChordFixedPoint(c[5],out.B))return false;
    const FVector3d direction=out.B-out.A;const double norm=direction.SquaredLength();
    if(!A::Finite(norm)||norm<0)return false;
    double squaredUpper=0;
    const auto first=Detail::Point(out.A),last=Detail::Point(out.B);
    for(int32 i=0;i<6;++i)
    {
        if(!Detail::ValidPoint(c[i]))return false;
        FIntervalPoint witness;
        if(i==0)witness=first; // Required endpoint-to-A guard, not just distance to the segment.
        else if(i==5)witness=last; // Required endpoint-to-B guard.
        else
        {
            FVector3d middle;if(!ChordFixedPoint(c[i],middle))return false;
            const double proposed=norm>0?FVector3d::DotProduct(middle-out.A,direction)/norm:0;
            if(!A::Finite(proposed))return false;
            const double t=FMath::Clamp(proposed,0.,1.);
            witness=ProjectedDistanceDetail::LerpPoint(first,last,Interval::Exact(t));
        }
        if(!Detail::ValidPoint(witness))return false;
        auto squared=Interval::Exact(0);
        for(int32 d=0;d<3;++d)squared=A::Add(squared,A::Square(A::Subtract(c[i].Axis[d],witness.Axis[d])));
        if(!A::Valid(squared)||squared.Upper<0)return false;
        squaredUpper=FMath::Max(squaredUpper,squared.Upper);
    }
    // sqrt is a scalar proposal only. The lower bound of its interval square
    // must dominate the already certified upper error; otherwise no cache entry.
    double epsilon=FMath::Sqrt(squaredUpper);
    if(!A::Finite(epsilon)||!(epsilon>0))return false;
    for(int32 attempt=0;attempt<8;++attempt)
    {
        const auto squared=A::Square(Interval::Exact(epsilon));
        if(A::Valid(squared)&&squared.Lower>=squaredUpper)
        {out.EpsilonUpper=epsilon;return true;}
        if(!A::Outward(epsilon,true,epsilon))return false;
    }
    return false;
}
static EChordEnvelopeBuild GetCertifiedChordEnvelopes(const FCurve& c,FDistanceBudget& budget,FChordEnvelopes& out)
{
    out.Reset();const auto before=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(before.Status!=EAnalyticNumericStatus::Complete)return EChordEnvelopeBuild::Arithmetic;
    if(budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor)return EChordEnvelopeBuild::Work;
    if(c.Num()!=6)return EChordEnvelopeBuild::Unavailable;
    for(const auto& point:c)if(!Detail::ValidPoint(point))return EChordEnvelopeBuild::Arithmetic;
    // Fixed bounded geometry stage: same 80 charges on a cache hit and miss.
    // A valid but insufficient account takes the original solver untouched.
    if(int64(budget.Remaining)-budget.ConsumptionFloor<80||budget.Used>MAX_uint64-80)return EChordEnvelopeBuild::Unavailable;
    for(int32 i=0;i<80;++i)if(!budget.Consume())return EChordEnvelopeBuild::Work;
    const auto* properties=GetCurveProperties(c,before.ControlFingerprint());
    if(!properties)return EChordEnvelopeBuild::Unavailable;
    if(!properties->ChordsReady)
    {
        TArray<FCurve,TInlineAllocator<8>> pieces;pieces.Add(c);
        for(int32 depth=0;depth<3;++depth)
        {
            TArray<FCurve,TInlineAllocator<8>> next;
            for(const auto& piece:pieces)
            {
                FCurve left,right;if(!Split(piece,Interval::Exact(.5),left,right))return EChordEnvelopeBuild::Unavailable;
                next.Add(MoveTemp(left));next.Add(MoveTemp(right));
            }
            pieces=MoveTemp(next);
        }
        FChordEnvelopes complete;
        for(const auto& piece:pieces)
        {FChordEnvelope chord;if(!BuildChordEnvelope(piece,chord))return EChordEnvelopeBuild::Unavailable;complete.Add(chord);}
        if(complete.Num()!=8)return EChordEnvelopeBuild::Unavailable;
        const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=before.ControlFingerprint())return EChordEnvelopeBuild::Arithmetic;
        // No partial publication; the key is the existing 36 coefficient
        // endpoint bits + numeric controls, never a contact/query identity.
        properties->Chords=MoveTemp(complete);properties->ChordsReady=true;
    }
    const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=before.ControlFingerprint())return EChordEnvelopeBuild::Arithmetic;
    out=properties->Chords;return EChordEnvelopeBuild::Ready;
}
}



namespace Speed::Analytic::Certified::TimeDetail
{
namespace CapsuleExteriorPrefixDetail
{
namespace A=::AnalyticCertified;using E=EMonotoneCurveTimeSeed;
struct FWork{FDistanceBudget& Budget;uint64 Initial;bool Pay(uint64 n){if(Budget.Used<Initial||Budget.Used-Initial>128||n>128-(Budget.Used-Initial))return false;for(uint64 i=0;i<n;++i)if(!Budget.Consume())return false;return true;}};
static bool FiniteVector(const FVector3d& p){return A::Finite(p.X)&&A::Finite(p.Y)&&A::Finite(p.Z);}
static double Dot(const FVector3d& a,const FVector3d& b){return a.X*b.X+a.Y*b.Y+a.Z*b.Z;}
static bool Midpoint(const FIntervalPoint& p,FVector3d& out)
{
    if(!Detail::ValidPoint(p))return false;
    for(int32 d=0;d<3;++d)out[d]=.5*p.Axis[d].Lower+.5*p.Axis[d].Upper;return FiniteVector(out);
}
// Scalar roots are ONLY proposals. Every accepted prefix is proved again with
// original path intervals and a separating-axis inequality below.
static bool FirstRoot(double a,double b,double c,double& root)
{
    if(!A::Finite(a)||!A::Finite(b)||!A::Finite(c)||!(a>0))return false;
    const double discriminant=b*b-4*a*c;
    if(!A::Finite(discriminant)||discriminant<0)return false;
    const double square=FMath::Sqrt(discriminant);
    const double q=-.5*(b+(b>=0?square:-square));
    double first=0,second=0;
    if(q==0){first=second=-b/(2*a);}else{first=q/a;second=c/q;}
    if(!A::Finite(first)||!A::Finite(second))return false;
    if(first>second)Swap(first,second);
    if(first>=0&&first<=1){root=first;return true;}
    if(second>=0&&second<=1){root=second;return true;}
    return false;
}
static bool CapsuleEntryProposal(const FVector3d& start,const FVector3d& end,const FVector3d& a,const FVector3d& b,double radius,double& entry)
{
    entry=1;const FVector3d velocity=end-start,chord=b-a,m=start-a;
    const double vv=Dot(velocity,velocity),dd=Dot(chord,chord),rr=radius*radius;
    if(!A::Finite(vv)||!A::Finite(dd)||!A::Finite(rr)||vv<0||dd<0)return false;
    double s=dd>0?FMath::Clamp(Dot(m,chord)/dd,0.,1.):0;
    if(!A::Finite(s))return false;const auto offset=m-chord*s;const double startDistance=Dot(offset,offset);
    if(!A::Finite(startDistance))return false;if(startDistance<=rr){entry=0;return true;}
    if(!(vv>0))return true;
    double root=0;
    for(const FVector3d& endpoint:{a,b})
    {
        const auto delta=start-endpoint;const double linear=2*Dot(delta,velocity),constant=Dot(delta,delta)-rr;
        if(FirstRoot(vv,linear,constant,root))entry=FMath::Min(entry,root);
    }
    if(dd>0)
    {
        const double vd=Dot(velocity,chord),md=Dot(m,chord);
        const double qa=vv-vd*vd/dd,qb=2*(Dot(m,velocity)-md*vd/dd),qc=Dot(m,m)-md*md/dd-rr;
        if(FirstRoot(qa,qb,qc,root))
        {
            const double projected=(md+root*vd)/dd;
            if(A::Finite(projected)&&projected>=0&&projected<=1)entry=FMath::Min(entry,root);
        }
    }
    return A::Finite(entry)&&entry>=0&&entry<=1;
}
// Closest points choose a useful direction only. Degeneracy, cancellation or a
// poor scalar answer can lose an optimization, never establish separation.
static bool AxisProposal(const FVector3d& p0,const FVector3d& p1,const FVector3d& q0,const FVector3d& q1,FVector3d& axis)
{
    const auto u=p1-p0,v=q1-q0,r=p0-q0;
    const double aa=Dot(u,u),ee=Dot(v,v),bb=Dot(u,v),cc=Dot(u,r),ff=Dot(v,r);
    if(!A::Finite(aa)||!A::Finite(ee)||!A::Finite(bb)||!A::Finite(cc)||!A::Finite(ff)||aa<0||ee<0)return false;
    double s=0,t=0;
    if(!(aa>0)){if(ee>0)t=FMath::Clamp(ff/ee,0.,1.);}
    else if(!(ee>0))s=FMath::Clamp(-cc/aa,0.,1.);
    else
    {
        const double denominator=aa*ee-bb*bb;
        if(!A::Finite(denominator))return false;
        if(denominator>0)s=FMath::Clamp((bb*ff-cc*ee)/denominator,0.,1.);
        t=(bb*s+ff)/ee;
        if(t<0){t=0;s=FMath::Clamp(-cc/aa,0.,1.);}
        else if(t>1){t=1;s=FMath::Clamp((bb-cc)/aa,0.,1.);}
    }
    if(!A::Finite(s)||!A::Finite(t))return false;
    axis=(p0+u*s)-(q0+v*t);return FiniteVector(axis);
}
static bool StrictlySeparated(const FIntervalBezierCenterPath& path,double endTime,
    const FVector3d& chordA,const FVector3d& chordB,double radius,const FVector3d& scalarStart,const FVector3d& scalarEnd)
{
    const auto end=CenterDetail::Evaluate(path,Interval::Exact(endTime));
    if(!Detail::ValidPoint(end))return false;
    FVector3d proposedEnd;for(int32 d=0;d<3;++d)proposedEnd[d]=scalarStart[d]+endTime*(scalarEnd[d]-scalarStart[d]);
    FVector3d proposal;if(!AxisProposal(scalarStart,proposedEnd,chordA,chordB,proposal))return false;
    const auto axis=Detail::Point(proposal);
    if(!Detail::ValidPoint(axis))return false;
    const auto dot=[](const FIntervalPoint& x,const FIntervalPoint& y)
    {auto sum=A::Multiply(x.Axis[0],y.Axis[0]);for(int32 d=1;d<3;++d)sum=A::Add(sum,A::Multiply(x.Axis[d],y.Axis[d]));return sum;};
    const auto p0=dot(axis,path.ControlPoints[0]),p1=dot(axis,end),a=dot(axis,Detail::Point(chordA)),b=dot(axis,Detail::Point(chordB));
    auto norm=A::Square(axis.Axis[0]);for(int32 d=1;d<3;++d)norm=A::Add(norm,A::Square(axis.Axis[d]));
    if(!A::Valid(p0)||!A::Valid(p1)||!A::Valid(a)||!A::Valid(b)||!A::Valid(norm)||!(norm.Lower>0))return false;
    const Interval p{FMath::Min(p0.Lower,p1.Lower),FMath::Max(p0.Upper,p1.Upper),true};
    const Interval q{FMath::Min(a.Lower,b.Lower),FMath::Max(a.Upper,b.Upper),true};
    Interval gap;
    if(p.Lower>q.Upper)gap=A::Subtract(Interval::Exact(p.Lower),Interval::Exact(q.Upper));
    else if(q.Lower>p.Upper)gap=A::Subtract(Interval::Exact(q.Lower),Interval::Exact(p.Upper));
    else return false;
    const auto lhs=A::Square(gap),rhs=A::Multiply(A::Square(Interval::Exact(radius)),norm);
    return A::Valid(gap)&&gap.Lower>0&&A::Valid(lhs)&&A::Valid(rhs)&&lhs.Lower>rhs.Upper;
}
}
// Proves only that the COMPLETE original trajectory prefix is outside one
// exterior capsule. No contact, earliest root, normal, UV or authority output.
EMonotoneCurveTimeSeed TryCapsuleExteriorClearPrefix(const FIntervalBezierCenterPath& path,
    const FVector3d& chordA,const FVector3d& chordB,double outerRadius,FDistanceBudget& budget,double timeGuard,double& clearPrefix)
{
    namespace D=CapsuleExteriorPrefixDetail;namespace A=::AnalyticCertified;using E=EMonotoneCurveTimeSeed;
    clearPrefix=0;const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=EAnalyticNumericStatus::Complete)return E::Arithmetic;
    if(budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor)return E::Work;
    if(path.Degree!=1||path.ControlPoints.Num()!=2)return E::NotApplicable;
    if(!CenterDetail::Valid(path)||!D::FiniteVector(chordA)||!D::FiniteVector(chordB)||
        !NonnegativeFinite(outerRadius)||!NonnegativeFinite(timeGuard))return E::Arithmetic;
    if(!(outerRadius>0&&timeGuard>0))return E::NotApplicable;
    if(uint64(int64(budget.Remaining)-budget.ConsumptionFloor)<128||budget.Used>MAX_uint64-128)return E::NotApplicable;
    D::FWork work{budget,budget.Used};if(!work.Pay(4))return E::Work;
    FVector3d start,end;if(!D::Midpoint(path.ControlPoints[0],start)||!D::Midpoint(path.ControlPoints[1],end))return E::NotApplicable;
    const auto complete=[&](double candidate)
    {
        const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=numeric.ControlFingerprint())return E::Arithmetic;
        clearPrefix=candidate;return E::Proved;
    };
    // Most sub-chords are far from this wheel trajectory. One direct full-
    // prefix theorem avoids even scalar quadratic work for those chords.
    if(!work.Pay(24))return E::Work;
    if(D::StrictlySeparated(path,1.,chordA,chordB,outerRadius,start,end))return complete(1.);
    if(!work.Pay(16))return E::Work;
    double entry=0;if(!D::CapsuleEntryProposal(start,end,chordA,chordB,outerRadius,entry)||!(entry>0))return E::NotApplicable;
    // A scalar no-hit may propose the full prefix. A false proposal fails the
    // same interval theorem; there is no scalar acceptance path.
    double retreat=.25*timeGuard;
    for(int32 attempt=0;attempt<3;++attempt)
    {
        if(!work.Pay(24))return E::Work;
        double candidate=entry-retreat;
        if(!(candidate<entry))candidate=A::FromBits(A::Bits(entry)-1);
        if(!A::Finite(candidate)||!(candidate>0&&candidate<=1))return E::NotApplicable;
        if(D::StrictlySeparated(path,candidate,chordA,chordB,outerRadius,start,end))
            return complete(candidate);
        retreat*=4;if(!A::Finite(retreat)||!(retreat>0))return E::NotApplicable;
    }
    return E::NotApplicable;
}
}



namespace Speed::Analytic::Certified
{
namespace ChordEnvelopeTimeHintControl
{
static TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.ChordEnvelopeTimeHint"),0,
    TEXT("Default OFF. Certified static chord envelopes propose a strictly clear prefix and an original-curve inside point for the unchanged dyadic frontier."),ECVF_Default);
bool RuntimeEnabled(){return Enabled.GetValueOnAnyThread()==1||OffsetCurveTimeHintControl::RuntimeEnabled();}
struct FCounts{std::atomic<uint64> Calls{0},Proved{0},NotApplicable{0},Work{0},Arithmetic{0},Ticks{0},Used{0},Capsules{0},FullClear{0},InsideAttempts{0};};
static FCounts Counts;
void PublishObservation()
{
    UE_LOG(LogTemp,Display,TEXT("[ChordEnvelopeTimeHint] Scope=ProgramPrefixAtPreExit DefaultOFF=1 NoApproximateContact=1 Calls=%llu Proved=%llu NotApplicable=%llu Work=%llu Arithmetic=%llu Used=%llu Capsules=%llu FullClear=%llu InsideAttempts=%llu Ticks=%llu SecondsPerCycle=%.17g"),
        Counts.Calls.load(),Counts.Proved.load(),Counts.NotApplicable.load(),Counts.Work.load(),Counts.Arithmetic.load(),Counts.Used.load(),Counts.Capsules.load(),Counts.FullClear.load(),Counts.InsideAttempts.load(),Counts.Ticks.load(),FPlatformTime::GetSecondsPerCycle64());
}
}
namespace TimeDetail
{
namespace ChordEnvelopeTimeHintDetail
{
using E=EMonotoneCurveTimeSeed;namespace S=StrongConvexCurveMinimum;namespace A=::AnalyticCertified;
static E Try(TConstArrayView<FIntervalTensorSurface> surfaces,const FIntervalBezierCenterPath& path,double radius,
    FDistanceBudget& budget,double gap,double& lower,double& upper,bool observe)
{
    lower=0;upper=1;
    const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=EAnalyticNumericStatus::Complete)return E::Arithmetic;
    if(budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor)return E::Work;
    if(surfaces.Num()!=1||surfaces[0].DegreeU!=0||surfaces[0].DegreeV!=5||surfaces[0].ControlPoints.Num()!=6||
        path.Degree!=1||path.ControlPoints.Num()!=2)return E::NotApplicable;
    if(!CenterDetail::Valid(path)||!NonnegativeFinite(radius)||!NonnegativeFinite(gap))return E::Arithmetic;
    for(const auto& cp:surfaces[0].ControlPoints)if(!Detail::ValidPoint(cp))return E::Arithmetic;
    if(!(radius>0&&gap>0))return E::NotApplicable;
    if(int64(budget.Remaining)-budget.ConsumptionFloor<768||budget.Used>MAX_uint64-768)return E::NotApplicable;
    // Protect the original solver's complete remaining reserve. Optional
    // failures keep every consumed group; no partial proof is published.
    FScopedDistanceBudgetFloor localLimit(budget,budget.Remaining-768);if(!localLimit.IsValid())return E::Work;
    const auto unavailable=[&]()
    {
        if(budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor)return E::Work;
        const auto current=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        return current.Status==EAnalyticNumericStatus::Complete&&current.ControlFingerprint()==numeric.ControlFingerprint()?E::NotApplicable:E::Arithmetic;
    };
    const auto has=[&](int32 n){return n>=0&&int64(budget.Remaining)-budget.ConsumptionFloor>=n&&budget.Used<=MAX_uint64-uint64(n);};
    const auto pay=[&](int32 n){if(!has(n))return false;for(int32 k=0;k<n;++k)if(!budget.Consume())return false;return true;};
    S::FCurve curve;for(const auto& cp:surfaces[0].ControlPoints)curve.Add(cp);
    S::FChordEnvelopes chords;const auto geometry=S::GetCertifiedChordEnvelopes(curve,budget,chords);
    if(geometry==S::EChordEnvelopeBuild::Arithmetic)return E::Arithmetic;
    if(geometry==S::EChordEnvelopeBuild::Work)return E::Work;
    if(geometry!=S::EChordEnvelopeBuild::Ready)return unavailable();
    double clear=1;double prefixes[8]{};
    if(chords.Num()!=8)return E::Arithmetic;
    for(int32 i=0;i<8;++i)
    {
        const auto outside=A::Add(Interval::Exact(radius),Interval::Exact(chords[i].EpsilonUpper));
        if(!A::Valid(outside)||!(outside.Upper>0))return unavailable();
        const auto status=TryCapsuleExteriorClearPrefix(path,chords[i].A,chords[i].B,outside.Upper,budget,gap,prefixes[i]);
        if(observe){++ChordEnvelopeTimeHintControl::Counts.Capsules;if(status==E::Proved&&prefixes[i]==1)++ChordEnvelopeTimeHintControl::Counts.FullClear;}
        if(status!=E::Proved)return status==E::NotApplicable?unavailable():status;
        if(!(prefixes[i]>0&&prefixes[i]<=1))return E::Arithmetic;
        clear=FMath::Min(clear,prefixes[i]);
    }
    if(!(clear>0&&clear<1))return unavailable();
    FVector3d start,end;if(!CapsuleExteriorPrefixDetail::Midpoint(path.ControlPoints[0],start)||
        !CapsuleExteriorPrefixDetail::Midpoint(path.ControlPoints[1],end))return unavailable();
    TArray<double,TInlineAllocator<8>> proposals;
    for(int32 i=0;i<8;++i)
    {
        if(prefixes[i]==1)continue; // this entire exterior capsule is already clear
        if(!pay(16))return unavailable();
        const auto inside=A::Subtract(Interval::Exact(radius),Interval::Exact(chords[i].EpsilonUpper));
        if(!A::Valid(inside)||!(inside.Lower>0))continue;
        double entry=1;
        if(CapsuleExteriorPrefixDetail::CapsuleEntryProposal(start,end,chords[i].A,chords[i].B,inside.Lower,entry)&&
            A::Finite(entry)&&entry>=clear&&entry<1)proposals.Add(entry);
    }
    proposals.Sort();
    const auto squaredRadius=A::Square(Interval::Exact(radius));if(!A::Valid(squaredRadius))return unavailable();
    // The inner capsules provide proposals only. Every accepted B is checked
    // on the complete original interval polynomial with an original-UV witness.
    int32 attempts=0;double last=-1;
    for(double proposal:proposals)
    {
        if(A::Bits(proposal)==A::Bits(last))continue;last=proposal;
        for(double scale:{.25,1.,4.})
        {
            if(attempts++>=4||!has(37))return unavailable();
            double b=FMath::Min(1.,proposal+scale*gap);
            if(!(b>proposal))b=A::FromBits(A::Bits(proposal)+1);
            if(!(b>clear&&b<=1)||!A::Finite(b))continue;
            if(!pay(1))return unavailable();
            const auto center=CenterDetail::Evaluate(path,Interval::Exact(b));if(!Detail::ValidPoint(center))return unavailable();
            FTensorDistanceBounds distance;
            const auto status=S::TryBoundForDecision(surfaces[0],center,budget,36,distance,&squaredRadius,0);
            if(observe)++ChordEnvelopeTimeHintControl::Counts.InsideAttempts;
            if(status==S::EResult::Work)return E::Work;
            if(status==S::EResult::Arithmetic)return unavailable();
            if(status==S::EResult::Bounded&&distance.Status!=EDistanceBoundStatus::Unknown&&distance.UpperSquared<squaredRadius.Lower)
            {
                const auto completed=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
                if(completed.Status!=EAnalyticNumericStatus::Complete||completed.ControlFingerprint()!=numeric.ControlFingerprint())return E::Arithmetic;
                lower=clear;upper=b;return E::Proved;
            }
        }
    }
    return unavailable();
}
}
static EMonotoneCurveTimeSeed TryChordEnvelopeTimeHintOnly(TConstArrayView<FIntervalTensorSurface> surfaces,const FIntervalBezierCenterPath& path,
    double radius,FDistanceBudget& budget,double gap,double& lower,double& upper)
{
    const bool observe=StrongConvexCurveMinimum::Observation::Enabled();const uint64 start=observe?FPlatformTime::Cycles64():0,before=budget.Used;
    auto& counts=ChordEnvelopeTimeHintControl::Counts;if(observe)++counts.Calls;
    const auto result=ChordEnvelopeTimeHintDetail::Try(surfaces,path,radius,budget,gap,lower,upper,observe);
    if(observe)
    {
        counts.Ticks.fetch_add(FPlatformTime::Cycles64()-start);if(budget.Used>=before)counts.Used.fetch_add(budget.Used-before);
        if(result==EMonotoneCurveTimeSeed::Proved)++counts.Proved;else if(result==EMonotoneCurveTimeSeed::NotApplicable)++counts.NotApplicable;
        else if(result==EMonotoneCurveTimeSeed::Work)++counts.Work;else ++counts.Arithmetic;
    }
    return result;
}
EMonotoneCurveTimeSeed TryChordEnvelopeTimeHint(TConstArrayView<FIntervalTensorSurface> surfaces,const FIntervalBezierCenterPath& path,
    double radius,FDistanceBudget& budget,double gap,double& lower,double& upper)
{
    if(OffsetCurveTimeHintControl::RuntimeEnabled())
    {
        const auto offset=TryOffsetCurveTimeHint(surfaces,path,radius,budget,gap,lower,upper);
        if(offset!=EMonotoneCurveTimeSeed::NotApplicable||ChordEnvelopeTimeHintControl::Enabled.GetValueOnAnyThread()!=1)return offset;
    }
    return TryChordEnvelopeTimeHintOnly(surfaces,path,radius,budget,gap,lower,upper);
}
EMonotoneCurveTimeSeed TryChordEnvelopeTimeHintWithSpatialProof(TConstArrayView<FIntervalTensorSurface> surfaces,const FIntervalBezierCenterPath& path,
    double radius,FDistanceBudget& budget,double gap,double& lower,double& upper,FOffsetCurveSpatialProofPtr& proof,const FRetainedSphereTimeSearch* owner)
{
    proof.Reset();
    if(OffsetCurveTimeHintControl::RuntimeEnabled())
    {
        const auto offset=TryOffsetCurveTimeHint(surfaces,path,radius,budget,gap,lower,upper,&proof,owner);
        if(offset!=EMonotoneCurveTimeSeed::NotApplicable||ChordEnvelopeTimeHintControl::Enabled.GetValueOnAnyThread()!=1)return offset;
    }
    return TryChordEnvelopeTimeHintOnly(surfaces,path,radius,budget,gap,lower,upper);
}
}
}



#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCertifiedChordEnvelopeTest,"IAmSpeed.AnalyticWorld.ChordEnvelope.StaticHausdorffAndBinding",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCertifiedChordEnvelopeTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace S=StrongConvexCurveMinimum;namespace A=::AnalyticCertified;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const auto same=[](const S::FChordEnvelopes& x,const S::FChordEnvelopes& y)
    {
        if(x.Num()!=y.Num())return false;
        for(int32 i=0;i<x.Num();++i)
        {
            if(A::Bits(x[i].EpsilonUpper)!=A::Bits(y[i].EpsilonUpper))return false;
            for(int32 d=0;d<3;++d)if(A::Bits(x[i].A[d])!=A::Bits(y[i].A[d])||A::Bits(x[i].B[d])!=A::Bits(y[i].B[d]))return false;
        }
        return true;
    };
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        S::FCurve straight;for(double z:{256.,307.2,358.4,1331.2,1433.6,1536.})straight.Add(Detail::Point(FVector3d(0,0,z)));
        FDistanceBudget first{1000},second{1000};S::FChordEnvelopes a,b;
        if(!TestTrue(TEXT("inhomogeneous line parameterization admits eight certified small-error capsules"),
            S::GetCertifiedChordEnvelopes(straight,first,a)==S::EChordEnvelopeBuild::Ready&&a.Num()==8&&first.Used==80))return false;
        for(const auto& chord:a)if(!TestTrue(TEXT("straight image is not penalized by nonlinear parameterization"),chord.EpsilonUpper<1.e-8))return false;
        const uint64 hits=S::CurvePropertyCache().Hits;
        if(!TestTrue(TEXT("immutable exact key hit keeps complete geometry bits and debit"),
            S::GetCertifiedChordEnvelopes(straight,second,b)==S::EChordEnvelopeBuild::Ready&&same(a,b)&&second.Used==first.Used&&S::CurvePropertyCache().Hits==hits+1))return false;
        S::FCurve uncertain=straight;uncertain[0].Axis[1]={-.125,.125,true};FDistanceBudget changed{1000};
        if(!TestTrue(TEXT("original endpoint uncertainty is included, not replaced by nominal endpoint"),
            S::GetCertifiedChordEnvelopes(uncertain,changed,b)==S::EChordEnvelopeBuild::Ready&&b[0].EpsilonUpper>=.125&&!same(a,b)))return false;
        auto oneUlp=straight;oneUlp[2].Axis[2].Upper=A::FromBits(A::Bits(oneUlp[2].Axis[2].Upper)+1);
        const uint64 misses=S::CurvePropertyCache().Misses;FDistanceBudget ulp{1000};
        if(!TestTrue(TEXT("one coefficient endpoint bit changes exact cache key"),
            S::GetCertifiedChordEnvelopes(oneUlp,ulp,b)==S::EChordEnvelopeBuild::Ready&&S::CurvePropertyCache().Misses==misses+1))return false;
        auto invalid=straight;invalid[2].Axis[0].Known=false;FDistanceBudget bad{1000};
        if(!TestTrue(TEXT("unknown coefficients never receive a capsule proof"),S::GetCertifiedChordEnvelopes(invalid,bad,b)==S::EChordEnvelopeBuild::Arithmetic&&b.IsEmpty()))return false;
        FDistanceBudget small{79};if(!TestTrue(TEXT("optional unavailable reserve leaves original account untouched"),
            S::GetCertifiedChordEnvelopes(straight,small,b)==S::EChordEnvelopeBuild::Unavailable&&small.Used==0&&b.IsEmpty()))return false;
        FDistanceBudget protectedBudget{159,0,80};if(!TestTrue(TEXT("protected fine reserve is not borrowed"),
            S::GetCertifiedChordEnvelopes(straight,protectedBudget,b)==S::EChordEnvelopeBuild::Unavailable&&protectedBudget.Used==0))return false;
        S::FCurve constant;constant.Init(Detail::Point(FVector3d(4,5,6)),6);FDistanceBudget pointBudget{1000};
        if(!TestTrue(TEXT("zero-length chords retain a valid conservative point capsule"),
            S::GetCertifiedChordEnvelopes(constant,pointBudget,b)==S::EChordEnvelopeBuild::Ready&&b.Num()==8))return false;
        // The actual largest-error upper arc from the current authenticated packet.
        const FVector3d controls[6]{{0,-3779.9339173709263,2009.0263206457787},{0,-3742.7825170235415,2024.414934530818},
            {0,-3704.120343040839,2036.1562182157973},{0,-3664.424771931899,2048},{0,-3624.2123859659496,2048},{0,-3584,2048}};
        S::FCurve arc;for(const auto& p:controls)arc.Add(Detail::Point(p));FDistanceBudget arcBudget{1000};
        if(!TestTrue(TEXT("actual high-curvature arc has sub-centimeter capsule cover after eight static pieces"),
            S::GetCertifiedChordEnvelopes(arc,arcBudget,b)==S::EChordEnvelopeBuild::Ready))return false;
        for(const auto& chord:b)if(!TestTrue(TEXT("eight-segment bound stays under 0.26 cm"),chord.EpsilonUpper<.26))return false;
    }
    TestTrue(TEXT("numeric controls restored"),restore.Restore());return !HasAnyErrors();
}
#endif



#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCapsuleExteriorPrefixTest,"IAmSpeed.AnalyticWorld.CapsuleExteriorPrefix.ExactClearOnly",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCapsuleExteriorPrefixTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;namespace A=::AnalyticCertified;
    namespace D=TimeDetail::CapsuleExteriorPrefixDetail;using E=TimeDetail::EMonotoneCurveTimeSeed;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("valid RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const auto path=[](FVector3d start,FVector3d end){FIntervalBezierCenterPath p;p.Degree=1;p.ControlPoints={Detail::Point(start),Detail::Point(end)};return p;};
    for(uint32 mode=0;mode<4;++mode)
    {
        _mm_setcsr((_mm_getcsr()&~(0x8040u|0x3fu))|((mode&1)?0x40u:0)|((mode&2)?0x8000u:0));
        for(int32 endpoint=0;endpoint<2;++endpoint)
        {
            const FVector3d a=endpoint?FVector3d(0,0,0):FVector3d(-2,0,0),b(2,0,0);
            const FVector3d p0=endpoint?FVector3d(-4,0,0):FVector3d(0,4,0),p1=FVector3d::ZeroVector;
            auto p=path(p0,p1);FDistanceBudget budget{128};double prefix=0;
            const auto status=TimeDetail::TryCapsuleExteriorClearPrefix(p,a,b,1.,budget,1.e-5,prefix);
            if(!TestTrue(TEXT("cylinder interior and finite endpoint roots retain a certified prefix before3/4"),
                status==E::Proved&&prefix>.74&&prefix<.75&&budget.Used<=116&&budget.Used>0))return false;
            if(!TestTrue(TEXT("entire returned original prefix separated"),D::StrictlySeparated(p,prefix,a,b,1.,p0,p1)))return false;
            if(!TestFalse(TEXT("a prefix containing true contact cannot be cleared"),D::StrictlySeparated(p,.75,a,b,1.,p0,p1)))return false;
            if(!TestFalse(TEXT("a prefix through the capsule cannot be cleared"),D::StrictlySeparated(p,.9,a,b,1.,p0,p1)))return false;
        }
        auto tangent=path(FVector3d(-2,1,0),FVector3d(2,1,0));FDistanceBudget budget{128};double prefix=0;
        const auto tangentStatus=TimeDetail::TryCapsuleExteriorClearPrefix(tangent,FVector3d::ZeroVector,FVector3d::ZeroVector,1.,budget,1.e-4,prefix);
        if(!TestTrue(TEXT("tangency is either unresolved or only strictly earlier prefix"),tangentStatus==E::NotApplicable||(tangentStatus==E::Proved&&prefix<.5)))return false;
        if(!TestFalse(TEXT("exact tangency is never strict clearance"),D::StrictlySeparated(tangent,.5,FVector3d::ZeroVector,FVector3d::ZeroVector,1.,FVector3d(-2,1,0),FVector3d(2,1,0))))return false;
        for(int32 fixture=0;fixture<3;++fixture)
        {
            FVector3d p0,p1,a,b;
            if(fixture==0){p0=FVector3d(-4,2,0);p1=FVector3d(4,2,0);a=FVector3d(-2,0,0);b=FVector3d(2,0,0);}
            else if(fixture==1){p0=FVector3d(-4,.5,0);p1=FVector3d(-3,.5,0);a=FVector3d(0,0,0);b=FVector3d(2,0,0);}
            else{p0=p1=FVector3d(0,4,0);a=b=FVector3d::ZeroVector;}
            auto p=path(p0,p1);budget={128};prefix=0;
            if(!TestTrue(TEXT("far parallel, finite-end extension, and degenerate point cases prove whole prefix"),
                TimeDetail::TryCapsuleExteriorClearPrefix(p,a,b,1.,budget,1.e-5,prefix)==E::Proved&&prefix==1&&budget.Used==28))return false;
        }
        auto inside=path(FVector3d(0,.5,0),FVector3d(0,4,0));budget={128};prefix=0;
        if(!TestTrue(TEXT("initially inside capsule gives no positive clear prefix"),
            TimeDetail::TryCapsuleExteriorClearPrefix(inside,FVector3d::ZeroVector,FVector3d::ZeroVector,1.,budget,1.e-5,prefix)==E::NotApplicable&&prefix==0))return false;
        auto family=path(FVector3d(0,2,0),FVector3d(0,2,0));for(auto& cp:family.ControlPoints)cp.Axis[1]=Interval{.5,3.5,true};budget={128};prefix=0;
        if(!TestTrue(TEXT("interval path family containing overlap cannot inherit scalar-midpoint clearance"),
            TimeDetail::TryCapsuleExteriorClearPrefix(family,FVector3d::ZeroVector,FVector3d::ZeroVector,1.,budget,1.e-5,prefix)==E::NotApplicable&&prefix==0))return false;
        auto p=path(FVector3d(0,4,0),FVector3d(0,0,0));budget={127};prefix=0;
        if(!TestTrue(TEXT("optional128 reserve is never partly spent when unavailable"),
            TimeDetail::TryCapsuleExteriorClearPrefix(p,FVector3d(-2,0,0),FVector3d(2,0,0),1.,budget,1.e-5,prefix)==E::NotApplicable&&budget.Used==0))return false;
        budget={128,0,-1};if(!TestTrue(TEXT("invalid account remains work failure"),TimeDetail::TryCapsuleExteriorClearPrefix(p,FVector3d(-2,0,0),FVector3d(2,0,0),1.,budget,1.e-5,prefix)==E::Work&&budget.Used==0))return false;
        p.ControlPoints[0].Axis[1].Known=false;budget={128};if(!TestTrue(TEXT("invalid original path remains arithmetic failure"),TimeDetail::TryCapsuleExteriorClearPrefix(p,FVector3d(-2,0,0),FVector3d(2,0,0),1.,budget,1.e-5,prefix)==E::Arithmetic))return false;
        auto huge=path(FVector3d(1.e308,1.e308,0),FVector3d(-1.e308,-1.e308,0));budget={128};prefix=0;
        if(!TestTrue(TEXT("overflow in optional scalar proposals cannot publish clearance"),TimeDetail::TryCapsuleExteriorClearPrefix(huge,FVector3d::ZeroVector,FVector3d::ZeroVector,1.,budget,1.e-5,prefix)==E::NotApplicable&&prefix==0))return false;
    }
    return !HasAnyErrors();
}
#endif



#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FChordEnvelopeTimeHintTest,"IAmSpeed.AnalyticWorld.ChordEnvelope.FirstContactPrefixAndOriginalInside",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FChordEnvelopeTimeHintTest::RunTest(const FString&)
{
    FScopedCVar<int32> offsetOff(TEXT("p.IAmSpeed.AnalyticWorld.OffsetCurveTimeHint"),0,FName(TEXT("ChordEnvelopeOracle")));
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace T=TimeDetail;using E=T::EMonotoneCurveTimeSeed;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FIntervalTensorSurface curve;curve.DegreeU=0;curve.DegreeV=5;
    for(int32 i=0;i<6;++i)curve.ControlPoints.Add(Detail::Point(FVector3d(4*i,i*(i-1),0)));
    TArray<FIntervalTensorSurface> surfaces;surfaces.Add(curve);
    FIntervalBezierCenterPath path;if(!CenterDetail::Linear(FVector3d(6,9,0),FVector3d(10,5,0),path))return false;
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);FDistanceBudget budget{10000};double lower=0,upper=1;
        const auto status=T::TryChordEnvelopeTimeHint(surfaces,path,2.,budget,1.e-5,lower,upper);
        const double exact=1.-2./FMath::Sqrt(32.);
        if(!TestTrue(TEXT("wide path obtains certified prefix and original-curve inside without global time monotonicity"),
            status==E::Proved&&lower>0&&lower<exact&&upper>exact&&upper<1&&upper-lower<.1&&budget.Used<=768&&budget.ConsumptionFloor==0))
        {AddInfo(FString::Printf(TEXT("status=%u lower=%.17g upper=%.17g exact=%.17g Used=%llu floor=%d"),uint32(status),lower,upper,exact,budget.Used,budget.ConsumptionFloor));return false;}
        FTensorDistanceBounds inside;FDistanceBudget check{1000};const auto center=CenterDetail::Evaluate(path,Interval::Exact(upper));
        const auto r2=::AnalyticCertified::Square(Interval::Exact(2.));
        if(!TestTrue(TEXT("upper point freshly validates on the original polynomial"),
            StrongConvexCurveMinimum::TryBoundForDecision(curve,center,check,36,inside,&r2,0)==StrongConvexCurveMinimum::EResult::Bounded&&inside.UpperSquared<r2.Lower))return false;
        FIntervalPoint witness;if(!TestTrue(TEXT("strict inside retains original UV and original curve point"),
            ProjectedDistanceDetail::EvaluateFixedUV(curve,inside.SampleUV,witness)&&MonotoneSeedTests::SamePoint(witness,inside.SamplePoint)))return false;
        FDistanceBudget oldWork{1000000},hintWork{1000000};
        T::FRetainedSphereTimeSearch old(surfaces,path,2.,oldWork,1.e-7,512,nullptr,false,nullptr,true,false,false,false,false);
        T::FRetainedSphereTimeSearch hint(surfaces,path,2.,hintWork,1.e-7,512,nullptr,false,nullptr,true,false,false,false,true);
        const auto same=[](const FTensorSphereTimeBounds& x,const FTensorSphereTimeBounds& y)
        {
            namespace A=::AnalyticCertified;
            return x.Status==y.Status&&x.InitialSeparationProved==y.InitialSeparationProved&&x.InsideWitnessSurfaceIndex==y.InsideWitnessSurfaceIndex&&
                A::Bits(x.LowerTime)==A::Bits(y.LowerTime)&&A::Bits(x.UpperTime)==A::Bits(y.UpperTime)&&
                A::Bits(x.InsideSampleUV.X)==A::Bits(y.InsideSampleUV.X)&&A::Bits(x.InsideSampleUV.Y)==A::Bits(y.InsideSampleUV.Y)&&
                MonotoneSeedTests::SamePoint(x.InsideSurfacePoint,y.InsideSurfacePoint);
        };
        const auto expected=old.Advance(1.e-5),actual=hint.Advance(1.e-5);
        if(!TestTrue(TEXT("certified chord hint retains old dyadic result and original witness bits"),
            expected.Status==ETensorSphereTimeStatus::FirstContactBracket&&same(expected,actual)&&hint.GetDyadicHintSkipsForInspection()>0))return false;
        const auto expectedRefined=old.Advance(2.5e-6),actualRefined=hint.Advance(2.5e-6);
        if(!TestTrue(TEXT("continued frontier retains original contact and witness bits"),same(expectedRefined,actualRefined)))return false;
        auto two=surfaces;two.Add(curve);FDistanceBudget unionBudget{10000};
        if(!TestTrue(TEXT("one provider cannot establish an unchecked union prefix"),T::TryChordEnvelopeTimeHint(two,path,2.,unionBudget,1.e-5,lower,upper)==E::NotApplicable&&unionBudget.Used==0))return false;
        auto quadratic=path;quadratic.Degree=2;quadratic.ControlPoints.Insert(Detail::Point(FVector3d(10,5,0)),1);FDistanceBudget nonLinear{10000};
        if(!TestTrue(TEXT("nonlinear enter-and-exit trajectory keeps ordinary solver"),T::TryChordEnvelopeTimeHint(surfaces,quadratic,2.,nonLinear,1.e-5,lower,upper)==E::NotApplicable&&nonLinear.Used==0))return false;
        FDistanceBudget small{767};if(!TestTrue(TEXT("optional768 reserve does not borrow or partly consume"),T::TryChordEnvelopeTimeHint(surfaces,path,2.,small,1.e-5,lower,upper)==E::NotApplicable&&small.Used==0))return false;
        FDistanceBudget protectedBudget{1000,0,233};if(!TestTrue(TEXT("protected remainder is preserved"),T::TryChordEnvelopeTimeHint(surfaces,path,2.,protectedBudget,1.e-5,lower,upper)==E::NotApplicable&&protectedBudget.Used==0&&protectedBudget.ConsumptionFloor==233))return false;
        FDistanceBudget bad{1000,0,-1};if(!TestTrue(TEXT("invalid account stays Work"),T::TryChordEnvelopeTimeHint(surfaces,path,2.,bad,1.e-5,lower,upper)==E::Work))return false;
    }
    TestTrue(TEXT("numeric controls restored"),restore.Restore());return !HasAnyErrors();
}
#endif



// Append after StrongConvexCurveMinimum definitions, before Offset helper.
namespace Speed::Analytic::Certified::TimeDetail
{
namespace OffsetSpatialDomainDetail
{
namespace A=::AnalyticCertified;namespace S=StrongConvexCurveMinimum;
static TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.OffsetSpatialDomain"),0,
    TEXT("Default OFF. Expand an operation-local exact offset contact domain to every minimum in the current center box; preserve original spatial geometry and witnesses."),ECVF_Default);
static bool SamePoint(const FIntervalPoint& a,const FIntervalPoint& b)
{
    for(int32 d=0;d<3;++d)if(a.Axis[d].Known!=b.Axis[d].Known||A::Bits(a.Axis[d].Lower)!=A::Bits(b.Axis[d].Lower)||A::Bits(a.Axis[d].Upper)!=A::Bits(b.Axis[d].Upper))return false;
    return true;
}
static bool SameSurface(const FIntervalTensorSurface& a,const FIntervalTensorSurface& b)
{
    if(a.DegreeU!=b.DegreeU||a.DegreeV!=b.DegreeV||a.ControlPoints.Num()!=b.ControlPoints.Num())return false;
    for(int32 i=0;i<a.ControlPoints.Num();++i)if(!SamePoint(a.ControlPoints[i],b.ControlPoints[i]))return false;return true;
}
static bool SamePath(const FIntervalBezierCenterPath& a,const FIntervalBezierCenterPath& b)
{
    if(a.Degree!=b.Degree||a.ControlPoints.Num()!=b.ControlPoints.Num())return false;
    for(int32 i=0;i<a.ControlPoints.Num();++i)if(!SamePoint(a.ControlPoints[i],b.ControlPoints[i]))return false;return true;
}
struct FDraft {Interval Parameter,RootTime;double Lower=0,Upper=1;bool Ready=false;};
}
// Opaque outside this CPP. No TLS/global cache or serialized authority.
class FOffsetCurveSpatialProof final
{
    friend struct FOffsetCurveSpatialProofAccess;
    const FRetainedSphereTimeSearch* OwnerAddress=nullptr;
    uint32 OwnerThread=0;
    const FIntervalTensorSurface* SurfaceAddress=nullptr;
    const FIntervalBezierCenterPath* PathAddress=nullptr;
    FIntervalTensorSurface Surface;
    FIntervalBezierCenterPath Path;
    FIntervalPoint CertifiedContactCenters;
    Interval Parameter;
    uint64 RadiusBits=0,NumericControl=0,UsedAtSeal=0;
    int32 RemainingAtSeal=0,MinimumFloor=0;
    FDistanceBudget* Account=nullptr;
    FOffsetCurveSpatialProof()=default;
public:
    ~FOffsetCurveSpatialProof()=default;
};
struct FOffsetCurveSpatialProofAccess
{
    static FOffsetCurveSpatialProofPtr Seal(TConstArrayView<FIntervalTensorSurface> surfaces,
        const FIntervalBezierCenterPath& path,double radius,FDistanceBudget& budget,
        const OffsetSpatialDomainDetail::FDraft& draft,const FRetainedSphereTimeSearch* owner)
    {
        namespace A=::AnalyticCertified;
        // Called only AFTER the offset helper's temporary floor has unwound.
        const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!owner||!draft.Ready||surfaces.Num()!=1||!A::Valid(draft.Parameter)||draft.Parameter.Lower<0||draft.Parameter.Upper>1||
            !(draft.Lower>=0&&draft.Lower<draft.Upper&&draft.Upper<=1)||
            numeric.Status!=EAnalyticNumericStatus::Complete||budget.ConsumptionFloor<0||
            int64(budget.Remaining)-budget.ConsumptionFloor<4||budget.Used>MAX_uint64-4)return {};
        for(int32 i=0;i<4;++i)if(!budget.Consume())return {};
        const auto centers=CenterDetail::Evaluate(path,Interval{draft.Lower,draft.Upper,true});
        if(!Detail::ValidPoint(centers))return {};
        TSharedPtr<FOffsetCurveSpatialProof,ESPMode::ThreadSafe> out(new FOffsetCurveSpatialProof);
        out->OwnerAddress=owner;out->OwnerThread=FPlatformTLS::GetCurrentThreadId();
        out->SurfaceAddress=surfaces.GetData();out->PathAddress=&path;
        out->Surface=surfaces[0];out->Path=path;out->CertifiedContactCenters=centers;out->Parameter=draft.Parameter;
        out->RadiusBits=A::Bits(radius);out->NumericControl=numeric.ControlFingerprint();out->Account=&budget;
        out->UsedAtSeal=budget.Used;out->RemainingAtSeal=budget.Remaining;out->MinimumFloor=budget.ConsumptionFloor;
        return out;
    }
    static EDirectCurveSpatialSeed Expand(const FOffsetCurveSpatialProof& proof,const FRetainedSphereTimeSearch* owner,
        TConstArrayView<FIntervalTensorSurface> surfaces,const FIntervalBezierCenterPath& path,
        double radius,FDistanceBudget& budget,const FIntervalPoint& center,Interval& out,FIntervalPoint& covered)
    {
        namespace A=::AnalyticCertified;namespace S=StrongConvexCurveMinimum;using E=EDirectCurveSpatialSeed;
        out={};covered={};
        if(proof.Account!=&budget||budget.ConsumptionFloor<proof.MinimumFloor||budget.Remaining<budget.ConsumptionFloor||
            budget.Used<proof.UsedAtSeal||budget.Remaining>proof.RemainingAtSeal||
            budget.Used-proof.UsedAtSeal!=uint64(int64(proof.RemainingAtSeal)-budget.Remaining))return E::Work;
        const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!owner||owner!=proof.OwnerAddress||FPlatformTLS::GetCurrentThreadId()!=proof.OwnerThread||
            numeric.Status!=EAnalyticNumericStatus::Complete||numeric.ControlFingerprint()!=proof.NumericControl||
            surfaces.Num()!=1||surfaces.GetData()!=proof.SurfaceAddress||&path!=proof.PathAddress||A::Bits(radius)!=proof.RadiusBits||
            !OffsetSpatialDomainDetail::SameSurface(surfaces[0],proof.Surface)||!OffsetSpatialDomainDetail::SamePath(path,proof.Path)||
            !Detail::ValidPoint(center))return E::Arithmetic;
        if(int64(budget.Remaining)-budget.ConsumptionFloor<32||budget.Used>MAX_uint64-32)return E::NotApplicable;
        FScopedDistanceBudgetFloor limit(budget,budget.Remaining-32);if(!limit.IsValid())return E::Work;
        const auto pay=[&](int32 n){for(int32 i=0;i<n;++i)if(!budget.Consume())return false;return true;};
        // B includes the certified contact box even if the old dyadic bracket
        // extends beyond the offset hint. It therefore covers cstar for EVERY
        // realization, plus every independent center of the current box.
        if(!pay(4))return E::Work;
        const auto box=TimeDetail::Hull(center,proof.CertifiedContactCenters);
        if(!Detail::ValidPoint(box))return E::Arithmetic;
        S::FCurve curve;for(const auto& cp:proof.Surface.ControlPoints)curve.Add(cp);
        const auto* properties=S::GetCurveProperties(curve,proof.NumericControl);
        Interval curvature;if(!properties||!S::WholeCurvature(*properties,box,curvature)||!(curvature.Lower>0))return E::NotApplicable;
        Interval displacement=Interval::Exact(0);
        for(int32 d=0;d<3;++d)
        {
            if(!pay(2))return E::Work;
            const auto width=A::Subtract(Interval::Exact(box.Axis[d].Upper),Interval::Exact(box.Axis[d].Lower));
            const auto& derivative=properties->DHull.Axis[d];
            const double magnitude=FMath::Max(FMath::Abs(derivative.Lower),FMath::Abs(derivative.Upper));
            if(!A::Valid(width)||width.Upper<0||!A::Finite(magnitude))return E::NotApplicable;
            displacement=A::Add(displacement,A::Multiply(Interval::Exact(width.Upper),Interval::Exact(magnitude)));
        }
        if(!pay(2))return E::Work;
        const auto numerator=A::Multiply(Interval::Exact(2),displacement);Interval expansion;
        if(!S::DividePositive(numerator,Interval::Exact(curvature.Lower),expansion)||!A::Valid(expansion)||expansion.Upper<0)return E::NotApplicable;
        const auto low=A::Subtract(Interval::Exact(proof.Parameter.Lower),Interval::Exact(expansion.Upper));
        const auto high=A::Add(Interval::Exact(proof.Parameter.Upper),Interval::Exact(expansion.Upper));
        if(!A::Valid(low)||!A::Valid(high))return E::NotApplicable;
        const Interval candidate{FMath::Max(0.,low.Lower),FMath::Min(1.,high.Upper),true};
        if(!A::Valid(candidate))return E::Arithmetic;
        const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=proof.NumericControl)return E::Arithmetic;
        out=candidate;covered=box;return E::Proved;
    }
};
bool OffsetSpatialDomainRuntimeEnabled(){return OffsetSpatialDomainDetail::Enabled.GetValueOnAnyThread()==1;}
EDirectCurveSpatialSeed BuildDirectCurveSpatialCells(const FIntervalTensorSurface& surface,
    Interval necessaryV,FDistanceBudget& budget,TArray<FDirectCurveSpatialCell,TInlineAllocator<2>>& cells)
{
    namespace A=::AnalyticCertified;namespace S=StrongConvexCurveMinimum;using E=EDirectCurveSpatialSeed;
    cells.Reset();
    const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=EAnalyticNumericStatus::Complete)return E::Arithmetic;
    if(surface.DegreeU!=0||surface.DegreeV!=5||surface.ControlPoints.Num()!=6)return E::NotApplicable;
    if(!A::Valid(necessaryV)||necessaryV.Lower<0||necessaryV.Upper>1)return E::Arithmetic;
    for(const auto& p:surface.ControlPoints)if(!Detail::ValidPoint(p))return E::Arithmetic;
    if(budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor)return E::Work;
    if(int64(budget.Remaining)-budget.ConsumptionFloor<64||budget.Used>MAX_uint64-64)return E::NotApplicable;
    FScopedDistanceBudgetFloor limit(budget,budget.Remaining-64);if(!limit.IsValid())return E::Work;
    const auto pay=[&](int32 n){for(int32 i=0;i<n;++i)if(!budget.Consume())return false;return true;};
    if(!pay(1))return E::Work;
    const auto width=A::Subtract(Interval::Exact(necessaryV.Upper),Interval::Exact(necessaryV.Lower));
    if(!A::Valid(width)||width.Upper<0)return E::NotApplicable;
    int32 depth=0;double step=1.;
    while(depth<48&&width.Upper<=step*.5){step*=.5;++depth;}
    if(depth<2)return E::NotApplicable;
    const uint64 count=uint64(1)<<depth;const double scale=double(count);
    // Scaling by a power of two is exact here. Positive conversion truncates
    // downward; closed-cell coverage includes grid-boundary and endpoint cases.
    const uint64 first=FMath::Min(uint64(necessaryV.Lower*scale),count-1);
    const uint64 last=FMath::Min(uint64(necessaryV.Upper*scale),count-1);
    if(last<first||last-first>1)return E::Arithmetic;
    if(!pay(3))return E::Work;
    Detail::FNode root;root.DegreeU=0;root.DegreeV=5;for(const auto& p:surface.ControlPoints)root.Net.Add(p);
    Detail::FNode du,dv;
    if(!ContactDetail::OriginalDerivativeNet(root,true,du)||!ContactDetail::OriginalDerivativeNet(root,false,dv))return E::NotApplicable;
    TArray<FDirectCurveSpatialCell,TInlineAllocator<2>> draft;
    for(uint64 index=first;index<=last;++index)
    {
        FDirectCurveSpatialCell cell;const double lo=double(index)/scale,hi=double(index+1)/scale;
        const auto restrictNet=[&](const Detail::FNode& original,Detail::FNode& output)->bool
        {
            S::FCurve c,restricted;for(const auto& p:original.Net)c.Add(p);
            if(!S::Restrict(c,lo,hi,restricted))return false;
            output.DegreeU=original.DegreeU;output.DegreeV=original.DegreeV;
            output.MinU=0;output.MaxU=1;output.DepthU=0;output.MinV=lo;output.MaxV=hi;output.DepthV=depth;
            for(const auto& p:restricted)output.Net.Add(p);return true;
        };
        // Each restriction is two certified de Casteljau splits at most;
        // derivatives come from the ORIGINAL root, never from rounded positions.
        if(!pay(6))return E::Work;
        if(!restrictNet(root,cell.Position)||!restrictNet(du,cell.OriginalDu)||!restrictNet(dv,cell.OriginalDv))return E::NotApplicable;
        draft.Add(MoveTemp(cell));
    }
    const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=numeric.ControlFingerprint())return E::Arithmetic;
    cells=MoveTemp(draft);return E::Proved;
}
static EDirectCurveSpatialSeed PrepareOffsetCurveSpatialSeedImpl(const FOffsetCurveSpatialProofPtr& proof,const FRetainedSphereTimeSearch* owner,
    TConstArrayView<FIntervalTensorSurface> surfaces,const FIntervalBezierCenterPath& path,double radius,
    FDistanceBudget& budget,const FIntervalPoint& center,FDirectCurveSpatialSeed& seed)
{
    using E=EDirectCurveSpatialSeed;seed={};if(!proof)return E::NotApplicable;
    if(budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor)return E::Work;
    if(int64(budget.Remaining)-budget.ConsumptionFloor<128||budget.Used>MAX_uint64-128)return E::NotApplicable;
    FScopedDistanceBudgetFloor limit(budget,budget.Remaining-128);if(!limit.IsValid())return E::Work;
    FDirectCurveSpatialSeed draft;
    const auto proofStatus=FOffsetCurveSpatialProofAccess::Expand(*proof,owner,surfaces,path,radius,budget,center,draft.NecessaryV,draft.CoveredCenters);
    if(proofStatus!=E::Proved)return proofStatus;
    const auto build=BuildDirectCurveSpatialCells(surfaces[0],draft.NecessaryV,budget,draft.Cells);
    if(build!=E::Proved)return build;
    seed=MoveTemp(draft);return E::Proved;
}
namespace OffsetSpatialDomainObservation
{
static std::atomic<uint64> Calls{0},Proved{0},NotApplicable{0},Work{0},Arithmetic{0},Cells{0},Ticks{0};
}
EDirectCurveSpatialSeed PrepareOffsetCurveSpatialSeed(const FOffsetCurveSpatialProofPtr& proof,const FRetainedSphereTimeSearch* owner,
    TConstArrayView<FIntervalTensorSurface> surfaces,const FIntervalBezierCenterPath& path,double radius,
    FDistanceBudget& budget,const FIntervalPoint& center,FDirectCurveSpatialSeed& seed)
{
    const bool observe=StrongConvexCurveMinimum::Observation::Enabled();const uint64 start=observe?FPlatformTime::Cycles64():0;
    const auto status=PrepareOffsetCurveSpatialSeedImpl(proof,owner,surfaces,path,radius,budget,center,seed);
    if(observe)
    {
        namespace O=OffsetSpatialDomainObservation;O::Calls.fetch_add(1);O::Ticks.fetch_add(FPlatformTime::Cycles64()-start);
        switch(status){case EDirectCurveSpatialSeed::Proved:O::Proved.fetch_add(1);O::Cells.fetch_add(seed.Cells.Num());break;
            case EDirectCurveSpatialSeed::NotApplicable:O::NotApplicable.fetch_add(1);break;
            case EDirectCurveSpatialSeed::Work:O::Work.fetch_add(1);break;
            case EDirectCurveSpatialSeed::Arithmetic:O::Arithmetic.fetch_add(1);break;}
    }
    return status;
}
void PublishOffsetSpatialDomain()
{
    namespace O=OffsetSpatialDomainObservation;
    UE_LOG(LogTemp,Display,TEXT("[OffsetSpatialDomain] Scope=ProgramPrefixAtPreExit DefaultOFF=1 WholeCenterBoxProof=1 Calls=%llu Proved=%llu NotApplicable=%llu Work=%llu Arithmetic=%llu DirectCells=%llu Ticks=%llu SecondsPerCycle=%.17g"),
        O::Calls.load(),O::Proved.load(),O::NotApplicable.load(),O::Work.load(),O::Arithmetic.load(),O::Cells.load(),O::Ticks.load(),FPlatformTime::GetSecondsPerCycle64());
}

}

namespace Speed::Analytic::Certified
{
namespace OffsetCurveTimeHintControl
{
static TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.OffsetCurveTimeHint"),0,
    TEXT("Default OFF. Direct exact normal-offset root bounds for a transverse regular quintic; only hints the original dyadic frontier, never supplies a contact witness."),ECVF_Default);
bool RuntimeEnabled(){return Enabled.GetValueOnAnyThread()==1;}
struct FCounts{std::atomic<uint64> Calls{0},Proved{0},NotApplicable{0},Work{0},Arithmetic{0},Used{0},Ticks{0},ShapeRefused{0},PathNonSingleton{0},NoFixedPlane{0},Transverse{0},Regular{0},OriginalInside{0},CapsClear{0};};
static FCounts Counts;
void PublishObservation()
{
    UE_LOG(LogTemp,Display,TEXT("[OffsetCurveTimeHint] Scope=ProgramPrefixAtPreExit DefaultOFF=1 NoApproximateContact=1 Calls=%llu Proved=%llu NotApplicable=%llu Work=%llu Arithmetic=%llu Used=%llu ShapeRefused=%llu PathNonSingleton=%llu NoFixedPlane=%llu Transverse=%llu Regular=%llu OriginalInside=%llu CapsClear=%llu Ticks=%llu SecondsPerCycle=%.17g"),
        Counts.Calls.load(),Counts.Proved.load(),Counts.NotApplicable.load(),Counts.Work.load(),Counts.Arithmetic.load(),Counts.Used.load(),Counts.ShapeRefused.load(),Counts.PathNonSingleton.load(),Counts.NoFixedPlane.load(),Counts.Transverse.load(),Counts.Regular.load(),Counts.OriginalInside.load(),Counts.CapsClear.load(),Counts.Ticks.load(),FPlatformTime::GetSecondsPerCycle64());
}
}
namespace TimeDetail
{
namespace OffsetCurveTimeHintDetail
{
namespace A=::AnalyticCertified;namespace S=StrongConvexCurveMinimum;using E=EMonotoneCurveTimeSeed;
static Interval Dot(const FIntervalPoint& a,const FIntervalPoint& b,int32 x,int32 y)
{return A::Add(A::Multiply(a.Axis[x],b.Axis[x]),A::Multiply(a.Axis[y],b.Axis[y]));}
static Interval Cross(const FIntervalPoint& a,const FIntervalPoint& b,int32 x,int32 y)
{return A::Subtract(A::Multiply(a.Axis[x],b.Axis[y]),A::Multiply(a.Axis[y],b.Axis[x]));}
static Interval NormSquared(const FIntervalPoint& a,int32 x,int32 y)
{return A::Add(A::Square(a.Axis[x]),A::Square(a.Axis[y]));}
static bool SqrtEnclosure(Interval squared,Interval& out)
{
    out={};if(!A::Valid(squared)||!(squared.Lower>0))return false;
    double lower=FMath::Sqrt(squared.Lower),upper=FMath::Sqrt(squared.Upper);bool low=false,high=false;
    for(int32 k=0;k<8;++k)
    {
        const auto l=A::Square(Interval::Exact(lower)),u=A::Square(Interval::Exact(upper));
        low=A::Valid(l)&&l.Upper<=squared.Lower;high=A::Valid(u)&&u.Lower>=squared.Upper;
        if(low&&high){out={lower,upper,true};return A::Valid(out)&&out.Lower>0;}
        if(!low&&!A::Outward(lower,false,lower))return false;
        if(!high&&!A::Outward(upper,true,upper))return false;
    }
    return false;
}
static bool EvaluateRange(const S::FCurve& c,Interval v,FIntervalPoint& out)
{
    if(c.IsEmpty()||!A::Valid(v)||v.Lower<0||v.Upper>1)return false;
    S::FCurve work=c;
    for(int32 count=work.Num()-1;count>0;--count)for(int32 i=0;i<count;++i)
        work[i]=ProjectedDistanceDetail::LerpPoint(work[i],work[i+1],v);
    out=work[0];return Detail::ValidPoint(out);
}
static bool OffsetH(const FIntervalPoint& p,const FIntervalPoint& d,const FIntervalPoint& start,
    const FIntervalPoint& velocity,int32 x,int32 y,double signedRadius,Interval& out)
{
    FIntervalPoint delta;for(int32 k=0;k<3;++k)delta.Axis[k]=A::Subtract(p.Axis[k],start.Axis[k]);
    Interval speed,ratio;if(!SqrtEnclosure(NormSquared(d,x,y),speed)||!S::DividePositive(Dot(d,velocity,x,y),speed,ratio))return false;
    out=A::Subtract(Cross(delta,velocity,x,y),A::Multiply(Interval::Exact(signedRadius),ratio));return A::Valid(out);
}
static bool OffsetPoint(const FIntervalPoint& p,const FIntervalPoint& d,int32 x,int32 y,double signedRadius,FIntervalPoint& out)
{
    Interval speed,nx,ny;if(!SqrtEnclosure(NormSquared(d,x,y),speed)||
        !S::DividePositive(d.Axis[x],speed,nx)||!S::DividePositive(d.Axis[y],speed,ny))return false;
    out=p;out.Axis[x]=A::Subtract(p.Axis[x],A::Multiply(Interval::Exact(signedRadius),ny));
    out.Axis[y]=A::Add(p.Axis[y],A::Multiply(Interval::Exact(signedRadius),nx));return Detail::ValidPoint(out);
}
static bool Propose(const S::FCurve& c,const S::FCurveProperties& properties,const FVector3d& start,const FVector3d& velocity,
    int32 x,int32 y,double signedRadius,double& parameter)
{
    parameter=.5;if(!S::EnsureScalarProposalProperties(c,properties))return false;
    double lo=0,hi=1;
    for(int32 step=0;step<8;++step)
    {
        const double w=parameter-.5;FVector3d value=properties.ScalarCoefficients[5],d=FVector3d::ZeroVector,dd=FVector3d::ZeroVector;
        for(int32 i=4;i>=0;--i){dd=dd*w+2.*d;d=d*w+value;value=value*w+properties.ScalarCoefficients[i];}
        value+=properties.ScalarAnchor;
        const double norm2=d[x]*d[x]+d[y]*d[y],cross=d[x]*velocity[y]-d[y]*velocity[x];
        if(!S::FiniteScalarPoint(value)||!S::FiniteScalarPoint(d)||!S::FiniteScalarPoint(dd)||!A::Finite(norm2)||!(norm2>0)||!A::Finite(cross)||cross==0)return false;
        const double speed=FMath::Sqrt(norm2),kappa=(d[x]*dd[y]-d[y]*dd[x])/(norm2*speed);
        const double h=(value[x]-start[x])*velocity[y]-(value[y]-start[y])*velocity[x]-signedRadius*(d[x]*velocity[x]+d[y]*velocity[y])/speed;
        const double derivative=cross*(1.-signedRadius*kappa);
        if(!A::Finite(h)||!A::Finite(derivative)||derivative==0)return false;
        const double candidate=parameter-h/derivative;
        if(h==0||candidate==parameter)return true;
        if((h>0)==(derivative>0))hi=parameter;else lo=parameter;
        const double next=A::Finite(candidate)&&candidate>lo&&candidate<hi?candidate:.5*lo+.5*hi;
        if(next==parameter)break;parameter=next;
    }
    return A::Finite(parameter)&&parameter>0&&parameter<1;
}
// This is the true endpoint interval box, not its nominal midpoint. A fixed
// proposed axis only helps prove separation of the entire original prefix.
static bool EndpointPrefixClear(const FIntervalBezierCenterPath& path,const FIntervalPoint& atB,
    const FIntervalPoint& endpoint,int32 x,int32 y,Interval radiusSquared)
{
    FVector3d start,end,target;
    if(!CapsuleExteriorPrefixDetail::Midpoint(path.ControlPoints[0],start)||!CapsuleExteriorPrefixDetail::Midpoint(atB,end)||
        !CapsuleExteriorPrefixDetail::Midpoint(endpoint,target))return false;
    const auto d=end-start;const double n=d[x]*d[x]+d[y]*d[y];
    if(!A::Finite(n)||n<0)return false;
    const double proposed=n>0?FMath::Clamp(((target[x]-start[x])*d[x]+(target[y]-start[y])*d[y])/n,0.,1.):0;
    if(!A::Finite(proposed))return false;
    auto axis=Detail::Point(start+proposed*d-target);
    const auto p0=Dot(axis,path.ControlPoints[0],x,y),p1=Dot(axis,atB,x,y),q=Dot(axis,endpoint,x,y),norm=NormSquared(axis,x,y);
    if(!A::Valid(p0)||!A::Valid(p1)||!A::Valid(q)||!A::Valid(norm)||!(norm.Lower>0))return false;
    const Interval p{FMath::Min(p0.Lower,p1.Lower),FMath::Max(p0.Upper,p1.Upper),true};Interval gap;
    if(p.Lower>q.Upper)gap=A::Subtract(Interval::Exact(p.Lower),Interval::Exact(q.Upper));
    else if(q.Lower>p.Upper)gap=A::Subtract(Interval::Exact(q.Lower),Interval::Exact(p.Upper));else return false;
    const auto lhs=A::Square(gap),rhs=A::Multiply(radiusSquared,norm);
    return A::Valid(gap)&&gap.Lower>0&&A::Valid(lhs)&&A::Valid(rhs)&&lhs.Lower>rhs.Upper;
}
static E Try(TConstArrayView<FIntervalTensorSurface> surfaces,const FIntervalBezierCenterPath& path,double radius,
    FDistanceBudget& budget,double gap,double& lower,double& upper,bool observe,OffsetSpatialDomainDetail::FDraft* captured,
    FIntervalPoint* capturedRootPoint=nullptr)
{
    if(captured)*captured={};if(capturedRootPoint)*capturedRootPoint={};
    lower=0;upper=1;const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=EAnalyticNumericStatus::Complete)return E::Arithmetic;
    if(budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor)return E::Work;
    if(surfaces.Num()!=1||surfaces[0].DegreeU!=0||surfaces[0].DegreeV!=5||surfaces[0].ControlPoints.Num()!=6||path.Degree!=1||path.ControlPoints.Num()!=2){if(observe)++OffsetCurveTimeHintControl::Counts.ShapeRefused;return E::NotApplicable;}
    if(!CenterDetail::Valid(path)||!NonnegativeFinite(radius)||!NonnegativeFinite(gap))return E::Arithmetic;
    for(const auto& cp:surfaces[0].ControlPoints)if(!Detail::ValidPoint(cp))return E::Arithmetic;
    if(!(radius>0&&gap>0))return E::NotApplicable;
    for(const auto& cp:path.ControlPoints)for(const auto& axis:cp.Axis)if(A::Bits(axis.Lower)!=A::Bits(axis.Upper)){if(observe)++OffsetCurveTimeHintControl::Counts.PathNonSingleton;return E::NotApplicable;}
    int32 fixed=INDEX_NONE;
    for(int32 axis=0;axis<3;++axis)
    {
        const uint64 bits=A::Bits(surfaces[0].ControlPoints[0].Axis[axis].Lower);bool same=true;
        for(const auto& cp:surfaces[0].ControlPoints)same&=A::Bits(cp.Axis[axis].Lower)==bits&&A::Bits(cp.Axis[axis].Upper)==bits;
        for(const auto& cp:path.ControlPoints)same&=A::Bits(cp.Axis[axis].Lower)==bits&&A::Bits(cp.Axis[axis].Upper)==bits;
        if(same){fixed=axis;break;}
    }
    if(fixed==INDEX_NONE){if(observe)++OffsetCurveTimeHintControl::Counts.NoFixedPlane;return E::NotApplicable;}
    const int32 x=(fixed+1)%3,y=(fixed+2)%3;
    if(int64(budget.Remaining)-budget.ConsumptionFloor<256||budget.Used>MAX_uint64-256)return E::NotApplicable;
    FScopedDistanceBudgetFloor localLimit(budget,budget.Remaining-256);if(!localLimit.IsValid())return E::Work;
    const auto unavailable=[&]()
    {if(budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor)return E::Work;
        const auto now=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        return now.Status==EAnalyticNumericStatus::Complete&&now.ControlFingerprint()==numeric.ControlFingerprint()?E::NotApplicable:E::Arithmetic;};
    const auto pay=[&](int32 n){if(n<0||int64(budget.Remaining)-budget.ConsumptionFloor<n||budget.Used>MAX_uint64-uint64(n))return false;
        for(int32 i=0;i<n;++i)if(!budget.Consume())return false;return true;};
    if(!pay(16))return unavailable();S::FCurve curve;for(const auto& cp:surfaces[0].ControlPoints)curve.Add(cp);
    const auto* properties=S::GetCurveProperties(curve,numeric.ControlFingerprint());if(!properties)return unavailable();
    FIntervalPoint velocity;for(int32 k=0;k<3;++k)velocity.Axis[k]=A::Subtract(path.ControlPoints[1].Axis[k],path.ControlPoints[0].Axis[k]);
    const auto transverse=Cross(properties->DHull,velocity,x,y),speed2=NormSquared(properties->DHull,x,y),velocity2=NormSquared(velocity,x,y);
    if(!A::Valid(transverse)||!A::Valid(speed2)||!A::Valid(velocity2)||!(velocity2.Lower>0)||!(transverse.Lower>0||transverse.Upper<0))return unavailable();
    if(observe)++OffsetCurveTimeHintControl::Counts.Transverse;
    const double epsilon=transverse.Lower>0?-1.:1.,signedRadius=epsilon*radius;
    Interval speed,kappa;if(!SqrtEnclosure(speed2,speed)||!S::DividePositive(Cross(properties->DHull,properties->DDHull,x,y),A::Multiply(speed2,speed),kappa))return unavailable();
    const auto scaledKappa=A::Multiply(Interval::Exact(radius),kappa);
    if(!A::Valid(scaledKappa)||!(scaledKappa.Lower> -1&&scaledKappa.Upper<1))return unavailable();
    const auto derivative=A::Multiply(transverse,A::Subtract(Interval::Exact(1),A::Multiply(Interval::Exact(signedRadius),kappa)));
    if(!A::Valid(derivative)||!(derivative.Lower>0||derivative.Upper<0))return unavailable();
    if(observe)++OffsetCurveTimeHintControl::Counts.Regular;
    const auto radiusSquared=A::Square(Interval::Exact(radius));if(!A::Valid(radiusSquared))return unavailable();
    if(int64(budget.Remaining)-budget.ConsumptionFloor<36)return unavailable();FTensorDistanceBounds initial;
    const auto initialStatus=S::TryBoundForDecision(surfaces[0],path.ControlPoints[0],budget,36,initial,&radiusSquared,0);
    if(initialStatus==S::EResult::Work)return E::Work;
    if(initialStatus!=S::EResult::Bounded||initial.Status==EDistanceBoundStatus::Unknown||!(initial.LowerSquared>radiusSquared.Upper))return unavailable();
    FVector3d start,end;if(!CapsuleExteriorPrefixDetail::Midpoint(path.ControlPoints[0],start)||!CapsuleExteriorPrefixDetail::Midpoint(path.ControlPoints[1],end))return unavailable();
    double parameter=0;if(!pay(16)||!Propose(curve,*properties,start,end-start,x,y,signedRadius,parameter))return unavailable();
    FIntervalPoint point,tangent;if(!pay(4)||!S::Evaluate(curve,parameter,point)||!S::Evaluate(properties->D,parameter,tangent))return unavailable();
    Interval h;if(!OffsetH(point,tangent,path.ControlPoints[0],velocity,x,y,signedRadius,h))return unavailable();
    Interval denominator=derivative,numerator=h;if(derivative.Upper<0){denominator=A::Negate(derivative);numerator=A::Negate(h);}
    Interval quotient;if(!S::DividePositive(numerator,denominator,quotient))return unavailable();
    const auto necessary=A::Subtract(Interval::Exact(parameter),quotient);if(!A::Valid(necessary))return unavailable();
    const Interval domain{FMath::Max(0.,necessary.Lower),FMath::Min(1.,necessary.Upper),true};if(!A::Valid(domain))return unavailable();
    FIntervalPoint rootPoint,rootTangent,offset;
    if(!pay(4)||!EvaluateRange(curve,domain,rootPoint)||!EvaluateRange(properties->D,domain,rootTangent)||
        !OffsetPoint(rootPoint,rootTangent,x,y,signedRadius,offset))return unavailable();
    FIntervalPoint delta;for(int32 k=0;k<3;++k)delta.Axis[k]=A::Subtract(offset.Axis[k],path.ControlPoints[0].Axis[k]);
    Interval time;if(!S::DividePositive(Dot(delta,velocity,x,y),velocity2,time)||!(time.Lower>0&&time.Upper<1))return unavailable();
    double a=0;if(!A::Outward(time.Lower,false,a)||!(a>0&&a<1))return unavailable();
    for(double scale:{.25,1.,4.})
    {
        if(!pay(12))return unavailable();double b=FMath::Min(1.,time.Upper+scale*gap);
        if(!(b>time.Upper))b=A::FromBits(A::Bits(time.Upper)+1);
        if(!A::Finite(b)||!(b>a&&b<=1))continue;
        const auto center=CenterDetail::Evaluate(path,Interval::Exact(b));if(!Detail::ValidPoint(center))return unavailable();
        FIntervalPoint radial;for(int32 k=0;k<3;++k)radial.Axis[k]=A::Subtract(point.Axis[k],center.Axis[k]);
        const auto distance=NormSquared(radial,x,y);
        if(!A::Valid(distance)||!(distance.Upper<radiusSquared.Lower))continue;
        if(observe)++OffsetCurveTimeHintControl::Counts.OriginalInside;
        if(!EndpointPrefixClear(path,center,curve[0],x,y,radiusSquared)||!EndpointPrefixClear(path,center,curve.Last(),x,y,radiusSquared))continue;
        if(observe)++OffsetCurveTimeHintControl::Counts.CapsClear;
        const auto completed=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(completed.Status!=EAnalyticNumericStatus::Complete||completed.ControlFingerprint()!=numeric.ControlFingerprint())return E::Arithmetic;
        // Initial separation + original strict InsideB establish an entry.
        // The two true endpoint caps are excluded on[0,B]. Every interior
        // entry is a root of the entering normal-offset h, whose derivative
        // is globally strict. Its necessary V and T boxes above therefore
        // contain the FIRST entry for every represented fixed polynomial.
        if(captured){captured->Parameter=domain;captured->RootTime=time;captured->Lower=a;captured->Upper=b;captured->Ready=true;}
        // Private immediate value transport only; domain is the published RootV.
        if(capturedRootPoint)*capturedRootPoint=rootPoint;
        lower=a;upper=b;return E::Proved;
    }
    return unavailable();
}
}
static EMonotoneCurveTimeSeed TryOffsetCurveTimeHint(TConstArrayView<FIntervalTensorSurface> surfaces,
    const FIntervalBezierCenterPath& path,double radius,FDistanceBudget& budget,double gap,double& lower,double& upper,FOffsetCurveSpatialProofPtr* capture,const FRetainedSphereTimeSearch* owner)
{
    if(capture)capture->Reset();
    const bool observe=StrongConvexCurveMinimum::Observation::Enabled();const uint64 start=observe?FPlatformTime::Cycles64():0,before=budget.Used;
    auto& o=OffsetCurveTimeHintControl::Counts;if(observe)++o.Calls;
    OffsetSpatialDomainDetail::FDraft captured;
    const auto result=OffsetCurveTimeHintDetail::Try(surfaces,path,radius,budget,gap,lower,upper,observe,capture?&captured:nullptr);
    if(capture&&result==EMonotoneCurveTimeSeed::Proved)
        *capture=FOffsetCurveSpatialProofAccess::Seal(surfaces,path,radius,budget,captured,owner);
    if(observe){o.Ticks.fetch_add(FPlatformTime::Cycles64()-start);if(budget.Used>=before)o.Used.fetch_add(budget.Used-before);
        if(result==EMonotoneCurveTimeSeed::Proved)++o.Proved;else if(result==EMonotoneCurveTimeSeed::NotApplicable)++o.NotApplicable;
        else if(result==EMonotoneCurveTimeSeed::Work)++o.Work;else ++o.Arithmetic;}
    return result;
}
}
}


#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FOffsetCurveTimeHintTest,"IAmSpeed.AnalyticWorld.OffsetCurveTimeHint.FirstEntryAndOriginalGrid",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FOffsetCurveTimeHintTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace T=TimeDetail;namespace A=::AnalyticCertified;using E=T::EMonotoneCurveTimeSeed;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> offsetOn(TEXT("p.IAmSpeed.AnalyticWorld.OffsetCurveTimeHint"),1,FName(TEXT("OffsetCurveTest")));
    FScopedCVar<int32> chordOff(TEXT("p.IAmSpeed.AnalyticWorld.ChordEnvelopeTimeHint"),0,FName(TEXT("OffsetCurveTestNoChord")));
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(int32 reversed:{0,1})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        FIntervalTensorSurface curve;curve.DegreeU=0;curve.DegreeV=5;
        for(int32 j=0;j<6;++j){const int32 i=reversed?5-j:j;curve.ControlPoints.Add(Detail::Point(FVector3d(4*i,i*(i-1),0)));}
        TArray<FIntervalTensorSurface> surfaces;surfaces.Add(curve);
        FIntervalBezierCenterPath path;if(!CenterDetail::Linear(FVector3d(6,9,0),FVector3d(10,5,0),path))return false;
        FDistanceBudget budget{10000};double lower=0,upper=1;const double exact=1.-2./FMath::Sqrt(32.);
        const auto status=T::TryOffsetCurveTimeHint(surfaces,path,2.,budget,1.e-5,lower,upper);
        if(!TestTrue(TEXT("both curve orientations certify the unique entering offset and original inside"),
            status==E::Proved&&lower<exact&&upper>exact&&lower>0&&upper<1&&upper-lower<1.e-4&&budget.Used<=256&&budget.ConsumptionFloor==0))
        {AddInfo(FString::Printf(TEXT("status=%u reversed=%d lower=%.17g upper=%.17g exact=%.17g Used=%llu"),uint32(status),reversed,lower,upper,exact,budget.Used));return false;}
        FDistanceBudget oracleWork{1000000},hintWork{1000000};
        T::FRetainedSphereTimeSearch oracle(surfaces,path,2.,oracleWork,1.e-7,512,nullptr,false,nullptr,true,false,false,false,false);
        T::FRetainedSphereTimeSearch hint(surfaces,path,2.,hintWork,1.e-7,512,nullptr,false,nullptr,true,false,false,false,true);
        const auto same=[](const FTensorSphereTimeBounds& a,const FTensorSphereTimeBounds& b)
        {return a.Status==b.Status&&a.InitialSeparationProved==b.InitialSeparationProved&&a.InsideWitnessSurfaceIndex==b.InsideWitnessSurfaceIndex&&
            A::Bits(a.LowerTime)==A::Bits(b.LowerTime)&&A::Bits(a.UpperTime)==A::Bits(b.UpperTime)&&
            A::Bits(a.InsideSampleUV.X)==A::Bits(b.InsideSampleUV.X)&&A::Bits(a.InsideSampleUV.Y)==A::Bits(b.InsideSampleUV.Y)&&MonotoneSeedTests::SamePoint(a.InsideSurfacePoint,b.InsideSurfacePoint);};
        const auto old=oracle.Advance(1.e-5),now=hint.Advance(1.e-5);
        if(!TestTrue(TEXT("offset shortcut changes only work before the same original dyadic split"),
            old.Status==ETensorSphereTimeStatus::FirstContactBracket&&same(old,now)&&hint.GetDyadicHintSkipsForInspection()>0))return false;
        if(!TestTrue(TEXT("later refinement retains original time/UV/point bits"),same(oracle.Advance(2.5e-6),hint.Advance(2.5e-6))))return false;
        auto uncertain=surfaces;uncertain[0].ControlPoints[2].Axis[1]={curve.ControlPoints[2].Axis[1].Lower-1.e-10,curve.ControlPoints[2].Axis[1].Upper+1.e-10,true};FDistanceBudget familyWork{10000};
        if(!TestTrue(TEXT("true coefficient boxes retain the nominal first entry"),T::TryOffsetCurveTimeHint(uncertain,path,2.,familyWork,1.e-5,lower,upper)==E::Proved&&lower<exact&&upper>exact))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FOffsetCurveRefusalTest,"IAmSpeed.AnalyticWorld.OffsetCurveTimeHint.CapsAndConservativeRefusals",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FOffsetCurveRefusalTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace T=TimeDetail;namespace D=T::OffsetCurveTimeHintDetail;using E=T::EMonotoneCurveTimeSeed;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FIntervalTensorSurface curve;curve.DegreeU=0;curve.DegreeV=5;for(int32 i=0;i<6;++i)curve.ControlPoints.Add(Detail::Point(FVector3d(4*i,i*(i-1),0)));
    TArray<FIntervalTensorSurface> surfaces;surfaces.Add(curve);FIntervalBezierCenterPath path;
    if(!CenterDetail::Linear(FVector3d(6,9,0),FVector3d(10,5,0),path))return false;
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);double lower=0,upper=1;FDistanceBudget work{10000};
        if(!TestTrue(TEXT("normal-offset cusp bound refuses instead of selecting a root"),T::TryOffsetCurveTimeHint(surfaces,path,100.,work,1.e-5,lower,upper)==E::NotApplicable&&lower==0&&upper==1))return false;
        FIntervalBezierCenterPath parallel;CenterDetail::Linear(FVector3d(1,1,0),FVector3d(2,2,0),parallel);work={10000};
        if(!TestTrue(TEXT("nontransverse direction retains original time solver"),T::TryOffsetCurveTimeHint(surfaces,parallel,2.,work,1.e-5,lower,upper)==E::NotApplicable))return false;
        FIntervalBezierCenterPath inside;CenterDetail::Linear(FVector3d(10,5,0),FVector3d(12,3,0),inside);work={10000};
        if(!TestTrue(TEXT("initial overlap cannot be turned into positive first-entry prefix"),T::TryOffsetCurveTimeHint(surfaces,inside,2.,work,1.e-5,lower,upper)==E::NotApplicable))return false;
        FIntervalBezierCenterPath endpointPath;CenterDetail::Linear(FVector3d(-1,4,0),FVector3d(-1,0,0),endpointPath);work={10000};
        if(!TestTrue(TEXT("endpoint-cap first contact remains ordinary solver's responsibility"),T::TryOffsetCurveTimeHint(surfaces,endpointPath,1.5,work,1.e-5,lower,upper)==E::NotApplicable))return false;
        const auto r2=::AnalyticCertified::Square(Interval::Exact(1.5));
        const auto endpointCenter=CenterDetail::Evaluate(endpointPath,Interval::Exact(.9));
        if(!TestFalse(TEXT("full prefix through a true endpoint cap may not be excluded"),D::EndpointPrefixClear(endpointPath,endpointCenter,curve.ControlPoints[0],0,1,r2)))return false;
        auto uncertainEndpoint=Detail::Point(FVector3d(0,0,0));uncertainEndpoint.Axis[0]={-2.,2.,true};
        if(!TestFalse(TEXT("endpoint uncertainty cannot be replaced by nominal center"),D::EndpointPrefixClear(endpointPath,endpointCenter,uncertainEndpoint,0,1,r2)))return false;
        auto invalid=surfaces;invalid[0].ControlPoints[1].Axis[0].Known=false;work={10000};
        if(!TestTrue(TEXT("invalid coefficient remains arithmetic failure"),T::TryOffsetCurveTimeHint(invalid,path,2.,work,1.e-5,lower,upper)==E::Arithmetic))return false;
        FDistanceBudget shortWork{255};if(!TestTrue(TEXT("unavailable256 allowance consumes nothing"),T::TryOffsetCurveTimeHint(surfaces,path,2.,shortWork,1.e-5,lower,upper)==E::NotApplicable&&shortWork.Used==0))return false;
        FDistanceBudget floorWork{1000,0,745};if(!TestTrue(TEXT("protected allocation remains untouched"),T::TryOffsetCurveTimeHint(surfaces,path,2.,floorWork,1.e-5,lower,upper)==E::NotApplicable&&floorWork.Used==0&&floorWork.ConsumptionFloor==745))return false;
        FDistanceBudget bad{1000,0,-1};if(!TestTrue(TEXT("invalid account stays work failure"),T::TryOffsetCurveTimeHint(surfaces,path,2.,bad,1.e-5,lower,upper)==E::Work))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
#endif


#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGroupedBoxContainedRejectTest,"IAmSpeed.AnalyticWorld.GeneralBox.ContainedRegionStopsProofOnly",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FGroupedBoxContainedRejectTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace A=::AnalyticCertified;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> memo(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxGeometryMemo"),1,FName(TEXT("ContainedRejectMemo")));
    const auto box=[](double x)
    {FBoxVolumeGroup group;for(int32 i=0;i<8;++i)group.Corners.Add(Detail::Point(FVector3d(x+((i&1)?1.:-1.),(i&2)?1.:-1.,(i&4)?1.:-1.)));return group;};
    const auto close=[](const TArray<FBoxVolumeGroup>& groups,FGroupedBoxVolumeBounds& volume)
    {FIntervalPoint hull;for(const auto& g:groups)for(const auto& p:g.Corners)hull=Detail::ValidPoint(hull)?TimeDetail::Hull(hull,p):p;
        TArray<FVector3d> axes{FVector3d::ForwardVector,FVector3d::RightVector,FVector3d::UpVector};FDistanceBudget work{4096};
        return CloseGroupedBoxVolumeBounds(hull,groups,axes,work,volume);};
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(int32 aabb:{0,1})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        TArray<FBoxVolumeGroup> coarseGroups{box(0)},fineGroups{box(-3),box(3)};FGroupedBoxVolumeBounds coarse,fine;
        if(!close(coarseGroups,coarse)||!close(fineGroups,fine))return false;
        FIntervalTensorSurface patch;patch.DegreeU=patch.DegreeV=1;
        for(double x:{-.25,.25})for(double y:{-.25,.25})patch.ControlPoints.Add(Detail::Point(FVector3d(x,y,0)));
        FIndexedIntervalUnionPtr scene;FDistanceBudget build{4096};if(!FIndexedIntervalUnion::Build(MakeArrayView(&patch,1),build,scene))return false;
        FReferenceBoundsCandidateSetPtr coarsePartition,finePartition;FDistanceBudget select{4096};
        if(!FReferenceBoundsCandidateSet::Select(scene,coarse.BroadHull,select,coarsePartition)||!FReferenceBoundsCandidateSet::Select(scene,fine.BroadHull,select,finePartition))return false;

        FScopedCVar<int32> aabbFlag(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxAabbFirst"),aabb,FName(TEXT("ContainedRejectAabb")));
        FDistanceBudget oldWork{1024},newWork{1024};FReferenceBoxVolumeStrictSeparationPtr oldProof,newProof;FGroupedBoxSeparationObservation observed;
        {FScopedCVar<int32> flag(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxContainedReject"),0,FName(TEXT("ContainedRejectOracle")));
            if(!TestFalse(TEXT("old coarse cannot prove a region containing the original patch"),FReferenceBoxVolumeStrictSeparation::TryProve(coarsePartition,coarse,oldWork,oldProof)))return false;}
        {FScopedCVar<int32> flag(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxContainedReject"),1,FName(TEXT("ContainedRejectOn")));
            if(!TestFalse(TEXT("new coarse refuses globally without claiming a contact"),FReferenceBoxVolumeStrictSeparation::TryProve(coarsePartition,coarse,newWork,newProof,&observed)))return false;}
        if(!TestTrue(TEXT("proved impossibility stops futile subdivisions, output remains absent"),!oldProof&&!newProof&&newWork.Used<oldWork.Used&&
            observed.VisitedCells==1&&FCString::Strcmp(observed.Stage,TEXT("cell_contained_group_region"))==0))return false;
        FDistanceBudget oldFine{1024},newFine{1024};
        {FScopedCVar<int32> flag(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxContainedReject"),0,FName(TEXT("ContainedFineOracle")));
            if(!FReferenceBoxVolumeStrictSeparation::TryProve(finePartition,fine,oldFine,oldProof))return false;}
        {FScopedCVar<int32> flag(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxContainedReject"),1,FName(TEXT("ContainedFineOn")));
            if(!FReferenceBoxVolumeStrictSeparation::TryProve(finePartition,fine,newFine,newProof))return false;}
        if(!TestTrue(TEXT("same original fine theorem remains clear with identical work and volume"),oldProof&&newProof&&oldProof->Matches(scene,fine)&&newProof->Matches(scene,fine)&&oldFine.Used==newFine.Used&&oldFine.Remaining==newFine.Remaining))return false;
        FScopedCVar<int32> flag(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxContainedReject"),1,FName(TEXT("ContainedFloor")));
        FDistanceBudget guarded{1000,7,900};FReferenceBoxVolumeStrictSeparationPtr absent;
        {FScopedDistanceBudgetFloor reserve(guarded,900);if(!reserve.IsValid())return false;
            TestFalse(TEXT("protected coarse proof still refuses"),FReferenceBoxVolumeStrictSeparation::TryProve(coarsePartition,coarse,guarded,absent));}
        if(!TestTrue(TEXT("prior debit and floor retained without reset or refund"),!absent&&guarded.Remaining>=900&&guarded.Used==uint64(1007-guarded.Remaining)&&guarded.ConsumptionFloor==900))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
#endif

namespace Speed::Analytic::Certified::TimeDetail
{
namespace CertifiedPrefixClearDetail
{
static TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.CertifiedPrefixClear"),0,
    TEXT("Default OFF. Reuse exact original-path clear-prefix proofs before repeated time-cell bounds; no approximate contact, threshold or geometry change."),ECVF_Default);
static std::atomic<uint64> Calls{0},Nodes{0};
}
bool CertifiedPrefixClearRuntimeEnabled(){return CertifiedPrefixClearDetail::Enabled.GetValueOnAnyThread()==1;}
void RecordCertifiedPrefixClear(uint64 nodes)
{
    if(!StrongConvexCurveMinimum::Observation::Enabled())return;
    CertifiedPrefixClearDetail::Calls.fetch_add(1);CertifiedPrefixClearDetail::Nodes.fetch_add(nodes);
}
void PublishCertifiedPrefixClear()
{
    UE_LOG(LogTemp,Display,TEXT("[CertifiedPrefixClear] Scope=ProgramPrefixAtPreExit DefaultOFF=1 NoApproximateContact=1 Calls=%llu ClearedTimeCells=%llu"),
        CertifiedPrefixClearDetail::Calls.load(),CertifiedPrefixClearDetail::Nodes.load());
}
}


#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCertifiedPrefixClearTest,"IAmSpeed.AnalyticWorld.CertifiedPrefixClear.OriginalFirstContactAndContinuation",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCertifiedPrefixClearTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace T=TimeDetail;namespace A=::AnalyticCertified;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        FIntervalTensorSurface curve;curve.DegreeU=0;curve.DegreeV=5;
        for(int32 i=0;i<6;++i)curve.ControlPoints.Add(Detail::Point(FVector3d(4*i,0,0)));
        TArray<FIntervalTensorSurface> surfaces;surfaces.Add(curve);FIntervalBezierCenterPath path;
        if(!CenterDetail::Linear(FVector3d(10,2,0),FVector3d(10,0,0),path))return false;
        FDistanceBudget oldWork{65536},newWork{65536};
        T::FRetainedSphereTimeSearch oracle(surfaces,path,1.,oldWork,1.e-7,512,nullptr,false,nullptr,true,false,false,true,false);
        T::FRetainedSphereTimeSearch actual(surfaces,path,1.,newWork,1.e-7,512,nullptr,false,nullptr,true,false,false,true,false);
        if(!TestTrue(TEXT("only a not-started operation can enable prefix discharge"),actual.SetCertifiedPrefixClearBeforeStart(true)))return false;
        const auto old=oracle.Advance(1.e-5),now=actual.Advance(1.e-5);
        const auto valid=[](const FTensorSphereTimeBounds& value)
        {return value.Status==ETensorSphereTimeStatus::FirstContactBracket&&value.InitialSeparationProved&&value.LowerTime<=.5&&value.UpperTime>=.5;};
        if(!TestTrue(TEXT("both modes enclose the real first contact at one half"),valid(old)&&valid(now)))return false;
        if(!TestTrue(TEXT("strict original-path prefix avoids earlier cell proofs"),actual.GetCertifiedPrefixSkipsForInspection()>0&&oracle.GetCertifiedPrefixSkipsForInspection()==0))return false;
        if(!TestFalse(TEXT("mode cannot change after proof search has started"),actual.SetCertifiedPrefixClearBeforeStart(false)))return false;
        const auto next=actual.Advance(2.5e-6);
        if(!TestTrue(TEXT("continued result remains nested around the same first contact"),valid(next)&&next.LowerTime>=now.LowerTime&&next.UpperTime<=now.UpperTime))return false;
        if(!TestTrue(TEXT("only an original curve sample is returned"),next.InsideSurfacePoint.Axis[0].Lower<=10&&next.InsideSurfacePoint.Axis[0].Upper>=10&&next.InsideSurfacePoint.Axis[1].Lower<=0&&next.InsideSurfacePoint.Axis[1].Upper>=0))return false;
        // Consume the remaining allocation honestly; no reset/refund or old
        // successful output may escape an exhausted continuation.
        while(newWork.Remaining>newWork.ConsumptionFloor)if(!newWork.Consume())return false;
        const uint64 exhausted=newWork.Used;const auto refused=actual.Advance(6.25e-7);
        if(!TestTrue(TEXT("an exhausted continuation remains Unknown/Work without refund"),refused.Status==ETensorSphereTimeStatus::Unknown&&actual.GetFailureForInspection()==T::ETimeSearchFailure::Work&&newWork.Used==exhausted&&newWork.Remaining==newWork.ConsumptionFloor))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCertifiedPrefixFallbackTest,"IAmSpeed.AnalyticWorld.CertifiedPrefixClear.NoProofAndContinuousPathFallback",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCertifiedPrefixFallbackTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace T=TimeDetail;namespace A=::AnalyticCertified;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FIntervalTensorSurface curve;curve.DegreeU=0;curve.DegreeV=5;
    for(int32 i=0;i<6;++i)curve.ControlPoints.Add(Detail::Point(FVector3d(4*i,0,0)));
    TArray<FIntervalTensorSurface> surfaces;surfaces.Add(curve);FIntervalBezierCenterPath linear;
    if(!CenterDetail::Linear(FVector3d(10,2,0),FVector3d(10,0,0),linear))return false;
    FDistanceBudget a{1000000},b{1000000};
    T::FRetainedSphereTimeSearch old(surfaces,linear,1.,a,1.e-7,512,nullptr,false,nullptr,true,false,false,false,false);
    T::FRetainedSphereTimeSearch noProof(surfaces,linear,1.,b,1.e-7,512,nullptr,false,nullptr,true,false,false,false,false);
    noProof.SetCertifiedPrefixClearBeforeStart(true);const auto x=old.Advance(1.e-5),y=noProof.Advance(1.e-5);
    if(!TestTrue(TEXT("without a proof the original path and debit are unchanged"),x.Status==ETensorSphereTimeStatus::FirstContactBracket&&y.Status==x.Status&&A::Bits(x.LowerTime)==A::Bits(y.LowerTime)&&A::Bits(x.UpperTime)==A::Bits(y.UpperTime)&&a.Used==b.Used&&a.Remaining==b.Remaining&&noProof.GetCertifiedPrefixSkipsForInspection()==0))return false;
    // Endpoints are both separated; the unchanged quadratic path enters and
    // exits. A linear chord substitute would miss this true first contact.
    FIntervalBezierCenterPath curved;curved.Degree=2;curved.ControlPoints={Detail::Point(FVector3d(10,2,0)),Detail::Point(FVector3d(10,-2,0)),Detail::Point(FVector3d(10,2,0))};
    FDistanceBudget c{1000000};T::FRetainedSphereTimeSearch nonlinear(surfaces,curved,1.,c,1.e-7,512,nullptr,false,nullptr,true,false,false,true,false);
    nonlinear.SetCertifiedPrefixClearBeforeStart(true);const auto result=nonlinear.Advance(1.e-5);const double root=.5-FMath::Sqrt(2.)*.25;
    if(!TestTrue(TEXT("a non-admitted continuous path keeps its interior first entry"),result.Status==ETensorSphereTimeStatus::FirstContactBracket&&result.LowerTime<=root&&result.UpperTime>=root&&nonlinear.GetCertifiedPrefixSkipsForInspection()==0))return false;
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
#endif



namespace Speed::Analytic::Certified::ContactDetail::ContactCellRootLruControl
{
static TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.ContactCellRootLru"),0,
    TEXT("Default OFF. At outer query boundaries only, retire least-recent roots until the rejected publication fits. Same root/cell/control-point caps; no geometry or work result retained."),ECVF_Default);
struct FCounters
{
    std::atomic<uint64> Boundaries{0},RootsRemoved{0},RootPressure{0},MemoryPressure{0},
        CellsReleased{0},PointsReleased{0},StillPinnedOrFull{0};
};
static FCounters Counters;
bool RuntimeEnabled(){return Enabled.GetValueOnAnyThread()==1;}
void RecordBoundaryEviction(uint32 roots,uint32 causes,uint64 beforeCells,uint64 afterCells,
    uint64 beforePoints,uint64 afterPoints,bool publicationFits)
{
    static const IConsoleVariable* phase=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"));
    if(!phase||phase->GetInt()==0)return;
    ++Counters.Boundaries;Counters.RootsRemoved.fetch_add(roots);
    if(causes&1u)++Counters.RootPressure;if(causes&2u)++Counters.MemoryPressure;
    if(beforeCells>=afterCells)Counters.CellsReleased.fetch_add(beforeCells-afterCells);
    if(beforePoints>=afterPoints)Counters.PointsReleased.fetch_add(beforePoints-afterPoints);
    if(!publicationFits)++Counters.StillPinnedOrFull;
}
static void Publish()
{
    UE_LOG(LogTemp,Display,TEXT("[ContactCellRootLru] Scope=ProgramPrefixAtPreExit ObservationOnly=1 MaximumRoots=64 MaximumLiveCells=65536 MaximumLivePoints=1048576 BoundaryOnly=1 Boundaries=%llu RootsRemoved=%llu RootPressure=%llu MemoryPressure=%llu CellsReleased=%llu PointsReleased=%llu StillPinnedOrFull=%llu"),
        Counters.Boundaries.load(),Counters.RootsRemoved.load(),Counters.RootPressure.load(),Counters.MemoryPressure.load(),
        Counters.CellsReleased.load(),Counters.PointsReleased.load(),Counters.StillPinnedOrFull.load());
}
struct FRegister {FRegister(){FCoreDelegates::OnPreExit.AddStatic(&Publish);}};
static FRegister Register;
}


// Append after StrongConvexCurveMinimum and the shared direct-cell builder.
namespace Speed::Analytic::Certified::TimeDetail
{
namespace CoordinateLineSpatialDetail
{
namespace A=::AnalyticCertified;namespace S=StrongConvexCurveMinimum;
using E=EDirectCurveSpatialSeed;
static TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.CoordinateLineSpatialDomain"),0,
    TEXT("Default OFF. Bound all original coordinate-line minimizing parameters for the entire current center box; restrict original nets without substituting segment UV or geometry."),ECVF_Default);
struct FCounters
{
    std::atomic<uint64> Calls{0},Proved{0},NotApplicable{0},Work{0},Arithmetic{0},Used{0},SeedCells{0},Ticks{0};
};
static FCounters Counts;
struct FWork
{
    FDistanceBudget& Budget;uint64 Initial;
    bool Pay(){return Budget.Used>=Initial&&Budget.Used-Initial<48&&Budget.Consume();}
};
static bool Propose(const S::FCurve& curve,const S::FCurveProperties& properties,int32 tangent,
    Interval target,FWork& work,double& parameter)
{
    parameter=.5;
    if(!work.Pay())return false;
    if(!S::EnsureScalarProposalProperties(curve,properties))return true;
    const double relativeTarget=(.5*target.Lower+.5*target.Upper)-properties.ScalarAnchor[tangent];
    const double first=(.5*curve[0].Axis[tangent].Lower+.5*curve[0].Axis[tangent].Upper)-properties.ScalarAnchor[tangent];
    const double last=(.5*curve[5].Axis[tangent].Lower+.5*curve[5].Axis[tangent].Upper)-properties.ScalarAnchor[tangent];
    if(!A::Finite(relativeTarget)||!A::Finite(first)||!A::Finite(last)||first==last)return true;
    double v=(relativeTarget-first)/(last-first);
    if(!A::Finite(v))return true;
    v=FMath::Clamp(v,0.,1.);double lo=0,hi=1;const bool increasing=last>first;
    for(int32 iteration=0;iteration<8;++iteration)
    {
        if(!work.Pay())return false;
        const double shifted=v-.5;
        double value=properties.ScalarCoefficients[5][tangent],derivative=0;
        for(int32 i=4;i>=0;--i)
        {derivative=derivative*shifted+value;value=value*shifted+properties.ScalarCoefficients[i][tangent];}
        const double residual=value-relativeTarget;
        if(!A::Finite(value)||!A::Finite(derivative)||!A::Finite(residual)||
            (increasing?!(derivative>0):!(derivative<0)))break;
        if(residual==0)break;
        if(increasing?(residual>0):(residual<0))hi=v;else lo=v;
        double next=v-residual/derivative;
        if(!A::Finite(next)||next<lo||next>hi)next=.5*lo+.5*hi;
        next=FMath::Clamp(next,0.,1.);if(next==v)break;v=next;
    }
    // Only this fixed, finite parameter survives the scalar proposal. All
    // coverage is proved below with the ORIGINAL interval polynomial.
    if(A::Finite(v)&&v>=0&&v<=1)parameter=v;
    return true;
}
static E Domain(TConstArrayView<FIntervalTensorSurface> surfaces,const FIntervalPoint& center,
    FDistanceBudget& budget,Interval& necessary)
{
    necessary={};
    const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=EAnalyticNumericStatus::Complete)return E::Arithmetic;
    if(budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor)return E::Work;
    if(surfaces.Num()!=1||surfaces[0].DegreeU!=0||surfaces[0].DegreeV!=5||surfaces[0].ControlPoints.Num()!=6)return E::NotApplicable;
    if(!Detail::ValidPoint(center))return E::Arithmetic;
    if(int64(budget.Remaining)-budget.ConsumptionFloor<48||budget.Used>MAX_uint64-48)return E::NotApplicable;
    FScopedDistanceBudgetFloor limit(budget,budget.Remaining-48);if(!limit.IsValid())return E::Work;
    FWork work{budget,budget.Used};const auto& surface=surfaces[0];
    S::FCurve curve;
    for(const auto& point:surface.ControlPoints)
    {if(!work.Pay())return E::Work;if(!Detail::ValidPoint(point))return E::Arithmetic;curve.Add(point);}
    int32 tangent=INDEX_NONE,constantCount=0;
    for(int32 d=0;d<3;++d)
    {
        if(!work.Pay())return E::Work;
        const auto value=curve[0].Axis[d];bool constant=A::Bits(value.Lower)==A::Bits(value.Upper);
        for(const auto& point:curve)constant&=A::Bits(point.Axis[d].Lower)==A::Bits(value.Lower)&&A::Bits(point.Axis[d].Upper)==A::Bits(value.Upper);
        if(constant)++constantCount;else tangent=d;
    }
    if(constantCount!=2||tangent==INDEX_NONE)return E::NotApplicable;
    if(!work.Pay())return E::Work;
    const auto* properties=S::GetCurveProperties(curve,numeric.ControlFingerprint());
    if(!properties)return E::NotApplicable;
    const auto derivative=properties->DHull.Axis[tangent];
    if(!A::Valid(derivative)||!(derivative.Lower>0||derivative.Upper<0))return E::NotApplicable;
    double parameter=.5;
    if(!Propose(curve,*properties,tangent,center.Axis[tangent],work,parameter))return E::Work;
    if(!work.Pay())return E::Work;
    FIntervalPoint point;
    if(!S::Evaluate(curve,parameter,point))return E::NotApplicable;
    if(!work.Pay())return E::Work;
    auto residual=A::Subtract(center.Axis[tangent],point.Axis[tangent]);auto divisor=derivative;
    if(derivative.Upper<0)
    {residual=A::Multiply(residual,Interval::Exact(-1));divisor=A::Multiply(divisor,Interval::Exact(-1));}
    Interval correction;
    if(!ContactDetail::ProjectionDividePositive(residual,divisor,correction))return E::NotApplicable;
    if(!work.Pay())return E::Work;
    const auto raw=A::Add(Interval::Exact(parameter),correction);
    if(!A::Valid(raw))return E::NotApplicable;
    // For an interior minimum, MVT places its original parameter in raw.
    // For an endpoint minimum, the same secant to that endpoint gives a
    // value beyond the endpoint. CLAMP therefore contains it; intersection
    // would incorrectly discard center boxes entirely beyond the finite end.
    const Interval domain{FMath::Clamp(raw.Lower,0.,1.),FMath::Clamp(raw.Upper,0.,1.),true};
    const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=numeric.ControlFingerprint())return E::Arithmetic;
    if(!A::Valid(domain))return E::Arithmetic;
    necessary=domain;return E::Proved;
}
static E Prepare(TConstArrayView<FIntervalTensorSurface> surfaces,const FIntervalPoint& center,
    FDistanceBudget& budget,FDirectCurveSpatialSeed& seed)
{
    seed={};
    if(budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor)return E::Work;
    if(int64(budget.Remaining)-budget.ConsumptionFloor<128||budget.Used>MAX_uint64-128)return E::NotApplicable;
    FScopedDistanceBudgetFloor limit(budget,budget.Remaining-128);if(!limit.IsValid())return E::Work;
    FDirectCurveSpatialSeed draft;
    const auto status=Domain(surfaces,center,budget,draft.NecessaryV);if(status!=E::Proved)return status;
    const auto build=BuildDirectCurveSpatialCells(surfaces[0],draft.NecessaryV,budget,draft.Cells);
    if(build!=E::Proved)return build;
    draft.CoveredCenters=center;seed=MoveTemp(draft);return E::Proved;
}
static void Publish()
{
    UE_LOG(LogTemp,Display,TEXT("[CoordinateLineSpatialDomain] Scope=ProgramPrefixAtPreExit AllCenterBox=1 OriginalCurveAndDerivatives=1 DefaultOFF=1 Calls=%llu Proved=%llu NotApplicable=%llu Work=%llu Arithmetic=%llu Used=%llu SeedCells=%llu Ticks=%llu SecondsPerCycle=%.17g"),
        Counts.Calls.load(),Counts.Proved.load(),Counts.NotApplicable.load(),Counts.Work.load(),Counts.Arithmetic.load(),Counts.Used.load(),Counts.SeedCells.load(),Counts.Ticks.load(),FPlatformTime::GetSecondsPerCycle64());
}
struct FRegister{FRegister(){FCoreDelegates::OnPreExit.AddStatic(&Publish);}};static FRegister Register;
}
bool CoordinateLineSpatialDomainRuntimeEnabled(){return CoordinateLineSpatialDetail::Enabled.GetValueOnAnyThread()==1;}
EDirectCurveSpatialSeed PrepareCoordinateLineSpatialSeed(TConstArrayView<FIntervalTensorSurface> surfaces,
    const FIntervalPoint& center,FDistanceBudget& budget,FDirectCurveSpatialSeed& seed)
{
    namespace Q=CoordinateLineSpatialDetail;using E=EDirectCurveSpatialSeed;
    if(!StrongConvexCurveMinimum::Observation::Enabled())return Q::Prepare(surfaces,center,budget,seed);
    ++Q::Counts.Calls;const uint64 start=FPlatformTime::Cycles64(),before=budget.Used;
    const auto result=Q::Prepare(surfaces,center,budget,seed);
    Q::Counts.Ticks.fetch_add(FPlatformTime::Cycles64()-start);
    if(budget.Used>=before)Q::Counts.Used.fetch_add(budget.Used-before);
    if(result==E::Proved){++Q::Counts.Proved;Q::Counts.SeedCells.fetch_add(seed.Cells.Num());}
    else if(result==E::NotApplicable)++Q::Counts.NotApplicable;
    else if(result==E::Work)++Q::Counts.Work;else ++Q::Counts.Arithmetic;
    return result;
}
}


#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCoordinateLineSpatialDomainTest,
    "IAmSpeed.AnalyticWorld.CoordinateLineSpatialDomain.AllCentersOriginalParameters",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCoordinateLineSpatialDomainTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;
    namespace T=TimeDetail;namespace Q=T::CoordinateLineSpatialDetail;namespace S=StrongConvexCurveMinimum;namespace A=::AnalyticCertified;
    using E=T::EDirectCurveSpatialSeed;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    const auto make=[](int32 tangent,int32 sign,bool nonlinear)
    {
        FIntervalTensorSurface surface;surface.DegreeU=0;surface.DegreeV=5;
        const double q[6]={0,.8,1.6,4.,9.6,20.};
        for(int32 i=0;i<6;++i)
        {FVector3d point(3,-7,11);point[tangent]=double(sign)*(nonlinear?q[i]:4.*i);surface.ControlPoints.Add(Detail::Point(point));}
        return surface;
    };
    const auto contains=[](Interval v,double expected){return A::Valid(v)&&v.Lower<=expected&&v.Upper>=expected;};
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(int32 tangent=0;tangent<3;++tangent)for(int32 sign:{-1,1})for(bool nonlinear:{false,true})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        auto surface=make(tangent,sign,nonlinear);TArray<FIntervalTensorSurface> surfaces{surface};
        S::FCurve curve;for(const auto& cp:surface.ControlPoints)curve.Add(cp);
        for(double expected:{0.,.125,.375,.875,1.})
        {
            FIntervalPoint center;if(!S::Evaluate(curve,expected,center))return false;
            // Offset along a constant coordinate does not alter the minimizer.
            center.Axis[(tangent+1)%3]=A::Add(center.Axis[(tangent+1)%3],Interval::Exact(5));
            FDistanceBudget work{4096,11,17};T::FDirectCurveSpatialSeed seed;
            const auto status=T::PrepareCoordinateLineSpatialSeed(surfaces,center,work,seed);
            if(!TestTrue(TEXT("line/nonuniform quintic root has a proved original-parameter seed"),status==E::Proved&&contains(seed.NecessaryV,expected)&&seed.Cells.Num()>=1&&seed.Cells.Num()<=2))return false;
            bool covered=false;
            for(const auto& cell:seed.Cells)
            {
                covered|=cell.Position.MinV<=expected&&cell.Position.MaxV>=expected;
                if(!TestTrue(TEXT("all three original nets use the same parameter domain"),
                    cell.Position.MinV==cell.OriginalDu.MinV&&cell.Position.MaxV==cell.OriginalDu.MaxV&&
                    cell.Position.MinV==cell.OriginalDv.MinV&&cell.Position.MaxV==cell.OriginalDv.MaxV&&
                    cell.Position.DegreeV==5&&cell.OriginalDv.DegreeV==4&&cell.OriginalDu.DegreeU==0))return false;
            }
            if(!TestTrue(TEXT("closed dyadic cells cover endpoint and interior minimizer"),covered&&work.Remaining>=17&&work.Used==uint64(4107-work.Remaining)))return false;
            for(int32 d=0;d<3;++d)if(!TestTrue(TEXT("whole center box is bound to the seed"),
                A::Bits(seed.CoveredCenters.Axis[d].Lower)==A::Bits(center.Axis[d].Lower)&&A::Bits(seed.CoveredCenters.Axis[d].Upper)==A::Bits(center.Axis[d].Upper)))return false;
        }
        // Entirely outside finite ends: intersection(raw,[0,1]) would be empty,
        // whereas a CLAMP must preserve the constrained endpoint minimum.
        for(int32 end:{0,1})
        {
            auto center=surface.ControlPoints[end?5:0];center.Axis[tangent]=Interval::Exact(double(sign)*(end?40.:-20.));
            FDistanceBudget work{4096};T::FDirectCurveSpatialSeed seed;
            if(!TestTrue(TEXT("finite endpoint clamp remains applicable outside the segment"),
                T::PrepareCoordinateLineSpatialSeed(surfaces,center,work,seed)==E::Proved&&
                seed.NecessaryV.Lower==double(end)&&seed.NecessaryV.Upper==double(end)))return false;
        }
        // A center box crossing the entire segment must retain ALL parameters;
        // the cell builder declines a useless broad seed rather than truncate.
        auto wide=surface.ControlPoints[0];wide.Axis[tangent]=Interval{-40,40,true};
        FDistanceBudget broadWork{4096};Interval broad;
        if(!TestTrue(TEXT("all-center coverage includes both endpoint minima"),Q::Domain(surfaces,wide,broadWork,broad)==E::Proved&&broad.Lower==0&&broad.Upper==1))return false;
        T::FDirectCurveSpatialSeed unavailable;FDistanceBudget broadSeedWork{4096};
        if(!TestTrue(TEXT("broad domain retains ordinary spatial fallback"),T::PrepareCoordinateLineSpatialSeed(surfaces,wide,broadSeedWork,unavailable)==E::NotApplicable&&unavailable.Cells.IsEmpty()))return false;
        // Independent coefficient boxes are not collapsed to a nominal line
        // parameterization. Several represented fixed curves must all fit.
        auto family=surface;const double epsilon=1./1048576.;
        for(auto& cp:family.ControlPoints){cp.Axis[tangent].Lower-=epsilon;cp.Axis[tangent].Upper+=epsilon;}
        TArray<FIntervalTensorSurface> familyArray{family};
        for(int32 pattern=0;pattern<4;++pattern)
        {
            S::FCurve realization;
            for(int32 i=0;i<6;++i)
            {auto cp=family.ControlPoints[i];cp.Axis[tangent]=Interval::Exact(((i+pattern)%3)?cp.Axis[tangent].Lower:cp.Axis[tangent].Upper);realization.Add(cp);}
            constexpr double expected=.375;FIntervalPoint center;if(!S::Evaluate(realization,expected,center))return false;
            FDistanceBudget work{4096};Interval domain;
            if(!TestTrue(TEXT("coefficient families cover every represented chosen minimizer"),Q::Domain(familyArray,center,work,domain)==E::Proved&&contains(domain,expected)))return false;
        }
    }
    TArray<FIntervalTensorSurface> surfaces{make(0,1,false)};auto center=Detail::Point(FVector3d(6,2,11));
    FDistanceBudget guarded{1000,7,900};T::FDirectCurveSpatialSeed seed;
    if(!TestTrue(TEXT("unavailable optional reserve preserves all prior work and floor"),
        T::PrepareCoordinateLineSpatialSeed(surfaces,center,guarded,seed)==E::NotApplicable&&guarded.Remaining==1000&&guarded.Used==7&&guarded.ConsumptionFloor==900&&seed.Cells.IsEmpty()))return false;
    FDistanceBudget reserved{1128,7,1000};
    if(!TestTrue(TEXT("available finite reserve cannot spend protected fallback allowance"),
        T::PrepareCoordinateLineSpatialSeed(surfaces,center,reserved,seed)==E::Proved&&reserved.Remaining>=1000&&reserved.ConsumptionFloor==1000&&reserved.Used==uint64(1135-reserved.Remaining)))return false;
    auto changed=surfaces;changed[0].ControlPoints[2].Axis[1]=Interval::Exact(8);
    FDistanceBudget notLine{4096};Interval domain;
    if(!TestTrue(TEXT("nonconstant orthogonal coordinate cannot borrow the line theorem"),Q::Domain(changed,center,notLine,domain)==E::NotApplicable))return false;
    changed=surfaces;changed[0].ControlPoints[2].Axis[0]=Interval::Exact(-10);
    FDistanceBudget notMonotone{4096};
    if(!TestTrue(TEXT("nonmonotone original tangent requires ordinary fallback"),Q::Domain(changed,center,notMonotone,domain)==E::NotApplicable))return false;
    changed=surfaces;changed[0].ControlPoints[2].Axis[0].Known=false;
    FDistanceBudget invalid{4096};
    if(!TestTrue(TEXT("unknown coefficient is never accepted by scalar proposal"),Q::Domain(changed,center,invalid,domain)==E::Arithmetic))return false;
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
#endif


#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDirectCurveSpatialCellsTest,
    "IAmSpeed.AnalyticWorld.DirectCurveSpatialSeed.DyadicCoverageAndOriginalDerivatives",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDirectCurveSpatialCellsTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace T=TimeDetail;namespace A=::AnalyticCertified;using E=T::EDirectCurveSpatialSeed;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FIntervalTensorSurface surface;surface.DegreeU=0;surface.DegreeV=5;
    // Exact Bernstein representation C(v)=(20v,20v^2,0).
    for(int32 i=0;i<6;++i)surface.ControlPoints.Add(Detail::Point(FVector3d(4*i,i*(i-1),0)));
    const auto contains=[](Interval value,double exact){return A::Valid(value)&&value.Lower<=exact&&value.Upper>=exact;};
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        FDistanceBudget work{1024,7,100};TArray<T::FDirectCurveSpatialCell,TInlineAllocator<2>> cells;
        if(!TestTrue(TEXT("short domain crossing a grid boundary has two original cells"),
            T::BuildDirectCurveSpatialCells(surface,Interval{.24,.26,true},work,cells)==E::Proved&&cells.Num()==2&&
            cells[0].Position.MinV==7./32&&cells[0].Position.MaxV==8./32&&
            cells[1].Position.MinV==8./32&&cells[1].Position.MaxV==9./32))return false;
        for(const auto& cell:cells)
        {
            const auto& node=cell.Position;const double v=(node.MinV+node.MaxV)*.5;
            const auto sample=Detail::Midpoint(node);FIntervalPoint du,dv;
            if(!MinimumDetail::NetHull(cell.OriginalDu,du)||!MinimumDetail::NetHull(cell.OriginalDv,dv))return false;
            if(!TestTrue(TEXT("direct restriction contains exact dyadic polynomial and ORIGINAL derivative values"),
                contains(sample.Axis[0],20*v)&&contains(sample.Axis[1],20*v*v)&&contains(sample.Axis[2],0)&&
                contains(dv.Axis[0],20)&&contains(dv.Axis[1],40*v)&&contains(dv.Axis[2],0)&&
                contains(du.Axis[0],0)&&contains(du.Axis[1],0)&&contains(du.Axis[2],0)&&
                node.DepthV==5&&cell.OriginalDv.DepthV==5&&cell.OriginalDu.DepthV==5&&
                node.MinV==cell.OriginalDv.MinV&&node.MaxV==cell.OriginalDv.MaxV))return false;
        }
        if(!TestTrue(TEXT("builder preserves prior cost and floor"),work.ConsumptionFloor==100&&work.Remaining>=100&&work.Used==uint64(1031-work.Remaining)))return false;
        for(double v:{0.,.5,1.})
        {
            FDistanceBudget endpointWork{1024};cells.Reset();
            if(!TestTrue(TEXT("closed original endpoints and exact grid boundaries remain covered"),
                T::BuildDirectCurveSpatialCells(surface,Interval::Exact(v),endpointWork,cells)==E::Proved&&cells.Num()>=1&&cells.Num()<=2))return false;
            bool covered=false;for(const auto& cell:cells)covered|=cell.Position.MinV<=v&&cell.Position.MaxV>=v&&cell.Position.DepthV<=48;
            if(!TestTrue(TEXT("no singleton root disappears at a dyadic edge"),covered))return false;
        }
        FDistanceBudget unavailable{63};cells.Reset();
        if(!TestTrue(TEXT("optional builder cannot spend unavailable reserve"),T::BuildDirectCurveSpatialCells(surface,Interval{.24,.26,true},unavailable,cells)==E::NotApplicable&&unavailable.Used==0&&cells.IsEmpty()))return false;
        FDistanceBudget broad{1024};
        if(!TestTrue(TEXT("full domain remains ordinary root traversal"),T::BuildDirectCurveSpatialCells(surface,Interval{0,1,true},broad,cells)==E::NotApplicable&&cells.IsEmpty()))return false;
        FDistanceBudget invalidAccount{1024,0,-1};
        if(!TestTrue(TEXT("invalid account remains Work"),T::BuildDirectCurveSpatialCells(surface,Interval{.24,.26,true},invalidAccount,cells)==E::Work))return false;
        auto invalid=surface;invalid.ControlPoints[2].Axis[1].Known=false;FDistanceBudget invalidWork{1024};
        if(!TestTrue(TEXT("unknown original coefficient remains Arithmetic"),T::BuildDirectCurveSpatialCells(invalid,Interval{.24,.26,true},invalidWork,cells)==E::Arithmetic))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FOffsetSpatialDomainBindingTest,
    "IAmSpeed.AnalyticWorld.OffsetSpatialDomain.WholeBoxAndBinding",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FOffsetSpatialDomainBindingTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace T=TimeDetail;namespace A=::AnalyticCertified;using E=T::EDirectCurveSpatialSeed;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> offset(TEXT("p.IAmSpeed.AnalyticWorld.OffsetCurveTimeHint"),1,FName(TEXT("SpatialBindingOffset")));
    FScopedCVar<int32> chord(TEXT("p.IAmSpeed.AnalyticWorld.ChordEnvelopeTimeHint"),0,FName(TEXT("SpatialBindingNoChord")));
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        FIntervalTensorSurface surface;surface.DegreeU=0;surface.DegreeV=5;
        for(int32 i=0;i<6;++i)surface.ControlPoints.Add(Detail::Point(FVector3d(4*i,0,0)));
        TArray<FIntervalTensorSurface> surfaces{surface};FIntervalBezierCenterPath path;
        if(!CenterDetail::Linear(FVector3d(10,2,0),FVector3d(10,0,0),path))return false;
        FDistanceBudget work{20000,3,11};
        T::FRetainedSphereTimeSearch owner(surfaces,path,1.,work,1.e-7,512,nullptr,false,nullptr,true,false,false,false,false);
        T::FOffsetCurveSpatialProofPtr proof;double lower=0,upper=1;
        if(!TestTrue(TEXT("operation-local offset proof is sealed after a true original first entry"),
            T::TryChordEnvelopeTimeHintWithSpatialProof(surfaces,path,1.,work,1.e-5,lower,upper,proof,&owner)==T::EMonotoneCurveTimeSeed::Proved&&proof&&lower<.5&&upper>.5))return false;
        auto centers=Detail::Point(FVector3d(10,1,0));centers.Axis[0]={9,11,true};centers.Axis[1]={0,2,true};
        T::FDirectCurveSpatialSeed seed;
        if(!TestTrue(TEXT("all independent centers have all constrained minima covered"),
            T::PrepareOffsetCurveSpatialSeed(proof,&owner,surfaces,path,1.,work,centers,seed)==E::Proved&&
            seed.NecessaryV.Lower<=.45&&seed.NecessaryV.Upper>=.55&&seed.Cells.Num()>=1&&seed.Cells.Num()<=2))return false;
        for(double v:{.45,.5,.55})
        {bool covered=false;for(const auto& cell:seed.Cells)covered|=cell.Position.MinV<=v&&cell.Position.MaxV>=v;if(!TestTrue(TEXT("whole-box minima survive direct cells"),covered))return false;}
        for(int32 d=0;d<3;++d)if(!TestTrue(TEXT("covered box contains both current box and certified contact"),seed.CoveredCenters.Axis[d].Lower<=centers.Axis[d].Lower&&seed.CoveredCenters.Axis[d].Upper>=centers.Axis[d].Upper))return false;
        FDistanceBudget foreign=work;
        if(!TestTrue(TEXT("copied allocation cannot borrow operation proof"),T::PrepareOffsetCurveSpatialSeed(proof,&owner,surfaces,path,1.,foreign,centers,seed)==E::Work))return false;
        T::FRetainedSphereTimeSearch otherOwner(surfaces,path,1.,work,1.e-7,512,nullptr,false,nullptr,true,false,false,false,false);
        if(!TestTrue(TEXT("same data under another search cannot borrow authority"),T::PrepareOffsetCurveSpatialSeed(proof,&otherOwner,surfaces,path,1.,work,centers,seed)==E::Arithmetic))return false;
        const auto cp=surfaces[0].ControlPoints[2].Axis[0];surfaces[0].ControlPoints[2].Axis[0]=Interval::Exact(A::FromBits(A::Bits(cp.Lower)+1));
        if(!TestTrue(TEXT("one original coefficient bit invalidates proof"),T::PrepareOffsetCurveSpatialSeed(proof,&owner,surfaces,path,1.,work,centers,seed)==E::Arithmetic))return false;
        surfaces[0].ControlPoints[2].Axis[0]=cp;
        const auto pathCp=path.ControlPoints[0].Axis[1];path.ControlPoints[0].Axis[1]=Interval::Exact(A::FromBits(A::Bits(pathCp.Lower)+1));
        if(!TestTrue(TEXT("one path coefficient bit invalidates proof"),T::PrepareOffsetCurveSpatialSeed(proof,&owner,surfaces,path,1.,work,centers,seed)==E::Arithmetic))return false;
        path.ControlPoints[0].Axis[1]=pathCp;
        if(!TestTrue(TEXT("radius identity remains exact"),T::PrepareOffsetCurveSpatialSeed(proof,&owner,surfaces,path,A::FromBits(A::Bits(1.)+1),work,centers,seed)==E::Arithmetic))return false;
        VectorSetControlRegister(VectorGetControlRegister()^uint32(0x8000));
        if(!TestTrue(TEXT("numeric controls cannot change under a retained proof"),T::PrepareOffsetCurveSpatialSeed(proof,&owner,surfaces,path,1.,work,centers,seed)==E::Arithmetic))return false;
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        if(!TestTrue(TEXT("restored exact source and account remain reusable within operation"),T::PrepareOffsetCurveSpatialSeed(proof,&owner,surfaces,path,1.,work,centers,seed)==E::Proved))return false;
        if(!TestTrue(TEXT("no debit is lost across successful and refused expansions"),work.ConsumptionFloor==11&&work.Remaining>=11&&work.Used==uint64(20003-work.Remaining)))return false;
        T::FOffsetCurveSpatialProofPtr absent;
        if(!TestTrue(TEXT("no proof leaves ordinary spatial traversal available"),T::PrepareOffsetCurveSpatialSeed(absent,&owner,surfaces,path,1.,work,centers,seed)==E::NotApplicable&&seed.Cells.IsEmpty()))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FOffsetSpatialDomainContactTest,
    "IAmSpeed.AnalyticWorld.OffsetSpatialDomain.ContactContinuation",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FOffsetSpatialDomainContactTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace T=TimeDetail;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    // This fixture exercises the generic retained route and its subdivision costs.
    // The complete direct-contact route has separate integration/continuation tests.
    FScopedCVar<int32> noDirectLine(TEXT("p.IAmSpeed.AnalyticWorld.DirectCoordinateLineContact"),0,FName(TEXT("SpatialSeedRouteIsolationLine")));
    FScopedCVar<int32> noDirectOffset(TEXT("p.IAmSpeed.AnalyticWorld.DirectOffsetCurveContact"),0,FName(TEXT("SpatialSeedRouteIsolationOffset")));
    FScopedCVar<int32> offset(TEXT("p.IAmSpeed.AnalyticWorld.OffsetCurveTimeHint"),1,FName(TEXT("SpatialContactOffset")));
    FScopedCVar<int32> chord(TEXT("p.IAmSpeed.AnalyticWorld.ChordEnvelopeTimeHint"),0,FName(TEXT("SpatialContactNoChord")));
    FScopedCVar<int32> monotone(TEXT("p.IAmSpeed.AnalyticWorld.MonotoneCurveTimeSeed"),0,FName(TEXT("SpatialContactNoMonotoneSeed")));
    FScopedCVar<int32> dyadic(TEXT("p.IAmSpeed.AnalyticWorld.MonotoneCurveDyadicHint"),0,FName(TEXT("SpatialContactNoMonotoneHint")));
    CurveStationaryContact98::FEnabledScope noStationary(false);
    FScopedCVar<int32> lineTime(TEXT("p.IAmSpeed.AnalyticWorld.CoordinateLineTimeHint"),0,FName(TEXT("SpatialContactNoLineHint")));
    FScopedCVar<int32> lineSpatial(TEXT("p.IAmSpeed.AnalyticWorld.CoordinateLineSpatialDomain"),0,FName(TEXT("SpatialContactNoLineSeed")));
    FScopedCVar<int32> prefix(TEXT("p.IAmSpeed.AnalyticWorld.CertifiedPrefixClear"),1,FName(TEXT("SpatialContactPrefix")));
    const uint32 saved=VectorGetControlRegister();const double exactTime=1.-2./FMath::Sqrt(32.);
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        FIntervalTensorSurface curve;curve.DegreeU=0;curve.DegreeV=5;
        for(int32 i=0;i<6;++i)curve.ControlPoints.Add(Detail::Point(FVector3d(4*i,i*(i-1),0)));
        TArray<FIntervalTensorSurface> surfaces{curve};FIntervalBezierCenterPath path;
        if(!CenterDetail::Linear(FVector3d(6,9,0),FVector3d(10,5,0),path))return false;
        uint64 oldVisited=0,newVisited=0;
        for(int32 mode:{0,1})
        {
            FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.OffsetSpatialDomain"),mode,FName(TEXT("SpatialContactMode")));
            FDistanceBudget work{1000000};int32 calls=0;
            const auto callback=[&](const FTensorSphereContactEnclosure& value)
            {++calls;return calls==1?EContactEnclosureDecision::Refine:EContactEnclosureDecision::Accept;};
            const FContactEnclosureFilter filter(callback);
            const auto result=BoundTensorSphereFirstContact(surfaces,path,2.,work,1.e-5,1.e-7,512,.02,.02,24,&filter);
            if(!TestTrue(TEXT("ordinary and seeded continuation cover the true first closest point"),
                result.Status==ETensorSphereContactStatus::FirstContactEnclosed&&result.LowerTime<=exactTime&&result.UpperTime>=exactTime&&
                result.SurfacePoint.Axis[0].Lower<=10&&result.SurfacePoint.Axis[0].Upper>=10&&
                result.SurfacePoint.Axis[1].Lower<=5&&result.SurfacePoint.Axis[1].Upper>=5&&
                calls>=2&&result.ConsumerWidthRefinements>=1&&!result.Leaves.IsEmpty()))
            {AddInfo(FString::Printf(TEXT("mode=%d status=%u failure=%u site=%s visited=%llu calls=%d time=[%.17g,%.17g]"),mode,uint32(result.Status),uint32(result.Failure),result.WorkSite,result.SpatialVisitedCells,calls,result.LowerTime,result.UpperTime));return false;}
            if(mode)newVisited=result.SpatialVisitedCells;else oldVisited=result.SpatialVisitedCells;
        }
        if(!TestTrue(TEXT("proved original domain avoids intermediate spatial subdivisions"),newVisited<oldVisited))return false;
        FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.OffsetSpatialDomain"),1,FName(TEXT("SpatialContactRefuse")));
        FDistanceBudget refusalWork{1000000};int32 refusalCalls=0;
        const auto refuse=[&](const FTensorSphereContactEnclosure&){++refusalCalls;return EContactEnclosureDecision::Refuse;};
        const FContactEnclosureFilter refusal(refuse);
        const auto refused=BoundTensorSphereFirstContact(surfaces,path,2.,refusalWork,1.e-5,1.e-7,512,.02,.02,24,&refusal);
        if(!TestTrue(TEXT("consumer refusal cannot expose a provisional seeded leaf"),refused.Status==ETensorSphereContactStatus::Unknown&&refused.Leaves.IsEmpty()&&refused.Failure==ETensorSphereContactFailure::ConsumerRefusal&&refusalCalls==1))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
#endif


namespace Speed::Analytic::Certified::TimeDetail
{
namespace DirectInitialTimeFrontierDetail
{
static TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.DirectInitialTimeFrontier"),0,
    TEXT("Default OFF. Start the existing retained time search at an already certified original-path first-contact interval; no surface witness is imported."),ECVF_Default);
static std::atomic<uint64> Searches{0},Applied{0},NodesAfterStart{0},BuildCalls{0},BuildReady{0},BuildNotApplicable{0},BuildWork{0},BuildArithmetic{0},BuildTicks{0};
static EDirectTimeFrontier Build(const FIntervalBezierCenterPath& path,double lower,double upper,FDistanceBudget& budget,FTimeNode& output)
{
    namespace A=::AnalyticCertified;using E=EDirectTimeFrontier;output={};
    const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=EAnalyticNumericStatus::Complete)return E::Arithmetic;
    if(budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor)return E::Work;
    if(path.Degree!=1||path.ControlPoints.Num()!=2)return E::NotApplicable;
    if(!CenterDetail::Valid(path)||!A::Finite(lower)||!A::Finite(upper)||!(lower>=0&&lower<upper&&upper<=1))return E::Arithmetic;
    if(int64(budget.Remaining)-budget.ConsumptionFloor<8||budget.Used>MAX_uint64-8)return E::NotApplicable;
    FScopedDistanceBudgetFloor limit(budget,budget.Remaining-8);if(!limit.IsValid())return E::Work;
    if(!budget.Consume())return E::Work;
    const auto width=A::Subtract(Interval::Exact(upper),Interval::Exact(lower));
    if(!A::Valid(width)||!(width.Upper>0))return E::NotApplicable;
    // Absolute original-time depth. This does not grant 52 NEW levels after
    // narrowing: descendants keep the existing global52 ceiling. The positive
    // normal exponent gives floor(-log2(width)); a non-power-of-two needs one
    // shallower level. Ultra-small widths are conservatively capped at48.
    int32 depth=48;
    if(width.Upper>1./281474976710656.)
    {
        const uint64 bits=A::Bits(width.Upper);
        const int32 exponent=int32((bits>>52)&0x7ffu)-1023;
        const bool fractional=(bits&0x000fffffffffffffull)!=0;
        depth=FMath::Clamp(-exponent-(fractional?1:0),0,48);
    }
    if(depth<2)return E::NotApplicable;
    if(!budget.Consume()||!budget.Consume())return E::Work;
    // Exactly the endpoint arithmetic used by the hint's originalInside(B),
    // not interpolation of previously restricted controls or a runtime pose.
    const auto start=CenterDetail::Evaluate(path,Interval::Exact(lower));
    const auto end=CenterDetail::Evaluate(path,Interval::Exact(upper));
    if(!Detail::ValidPoint(start)||!Detail::ValidPoint(end))return E::NotApplicable;
    FTimeNode draft;draft.A=lower;draft.B=upper;draft.Depth=depth;
    draft.Curve.Degree=1;draft.Curve.ControlPoints={start,end};
    if(!budget.Consume())return E::Work;
    const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=numeric.ControlFingerprint())return E::Arithmetic;
    if(!CenterDetail::Valid(draft.Curve))return E::Arithmetic;
    // Pure path geometry only. This helper neither authenticates [0,A] clear
    // nor publishes Inside(B); only the owning search's existing hint does so.
    output=MoveTemp(draft);return E::Ready;
}
static void Publish()
{
    UE_LOG(LogTemp,Display,TEXT("[DirectInitialTimeFrontier] Scope=ProgramPrefixAtPreExit DefaultOFF=1 OriginalGlobalPath=1 NoImportedSurfaceWitness=1 Searches=%llu Applied=%llu NodesAfterStart=%llu BuildCalls=%llu Ready=%llu NotApplicable=%llu Work=%llu Arithmetic=%llu BuildTicks=%llu SecondsPerCycle=%.17g"),
        Searches.load(),Applied.load(),NodesAfterStart.load(),BuildCalls.load(),BuildReady.load(),BuildNotApplicable.load(),BuildWork.load(),BuildArithmetic.load(),BuildTicks.load(),FPlatformTime::GetSecondsPerCycle64());
}
struct FRegister {FRegister(){FCoreDelegates::OnPreExit.AddStatic(&Publish);}};static FRegister Register;
}
bool DirectInitialTimeFrontierRuntimeEnabled(){return DirectInitialTimeFrontierDetail::Enabled.GetValueOnAnyThread()==1;}
EDirectTimeFrontier BuildDirectInitialTimeFrontier(const FIntervalBezierCenterPath& path,double lower,double upper,FDistanceBudget& budget,FTimeNode& output)
{
    namespace D=DirectInitialTimeFrontierDetail;using E=EDirectTimeFrontier;
    const bool observe=StrongConvexCurveMinimum::Observation::Enabled();const uint64 start=observe?FPlatformTime::Cycles64():0;
    const auto result=D::Build(path,lower,upper,budget,output);
    if(observe)
    {
        ++D::BuildCalls;D::BuildTicks.fetch_add(FPlatformTime::Cycles64()-start);
        if(result==E::Ready)++D::BuildReady;else if(result==E::NotApplicable)++D::BuildNotApplicable;
        else if(result==E::Work)++D::BuildWork;else ++D::BuildArithmetic;
    }
    return result;
}
void RecordDirectInitialTimeFrontier(bool applied,uint64 nodes)
{
    if(!StrongConvexCurveMinimum::Observation::Enabled())return;
    namespace D=DirectInitialTimeFrontierDetail;++D::Searches;
    if(applied){++D::Applied;D::NodesAfterStart.fetch_add(nodes);}
}
}


#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDirectInitialTimeFrontierGeometryTest,
    "IAmSpeed.AnalyticWorld.DirectInitialTimeFrontier.OriginalGlobalParametersAndDepth",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDirectInitialTimeFrontierGeometryTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace T=TimeDetail;namespace A=::AnalyticCertified;using E=T::EDirectTimeFrontier;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();FIntervalBezierCenterPath path;
    if(!CenterDetail::Linear(FVector3d(0,0,0),FVector3d(1,2,4),path))return false;
    TArray<FIntervalTensorSurface> surfaces{MonotoneSeedTests::Parabola()};
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        for(const FVector2d domain:{FVector2d(.123456789,.123466789),FVector2d(0,1.e-6),FVector2d(1.-1.e-6,1),FVector2d(.5-1.e-6,.5+1.e-6)})
        {
            FDistanceBudget work{1024,7,100};T::FTimeNode node;
            if(!TestTrue(TEXT("narrow non-dyadic original domains are built without old ancestor levels"),T::BuildDirectInitialTimeFrontier(path,domain.X,domain.Y,work,node)==E::Ready&&node.A==domain.X&&node.B==domain.Y&&node.Depth>=2&&node.Depth<=48))return false;
            const auto first=CenterDetail::Evaluate(path,Interval::Exact(domain.X)),last=CenterDetail::Evaluate(path,Interval::Exact(domain.Y));
            if(!TestTrue(TEXT("both endpoint coefficients use exactly the original global evaluation"),MonotoneSeedTests::SamePoint(node.Curve.ControlPoints[0],first)&&MonotoneSeedTests::SamePoint(node.Curve.ControlPoints.Last(),last)&&
                !node.Endpoint.IsSet()&&!node.Spatial&&!node.SpatialSeed&&!node.RetainSpatial&&!node.ForceSubdivision&&work.Used==11&&work.Remaining==1020&&work.ConsumptionFloor==100))return false;
            double globalCellWidth=1.;for(int32 i=0;i<node.Depth;++i)globalCellWidth*=.5;
            const auto width=A::Subtract(Interval::Exact(node.B),Interval::Exact(node.A));
            if(!TestTrue(TEXT("absolute depth never grants an extra global subdivision allowance"),width.Upper<=globalCellWidth&&(node.Depth==48||width.Upper>globalCellWidth*.5)))return false;
            FDistanceBudget searchWork{10000};T::FRetainedSphereTimeSearch search(surfaces,path,1.,searchWork,1.e-7,512,nullptr,false,nullptr,true,false,false,false,false);
            const int32 initialDepth=node.Depth;
            for(int32 level=0;level<32&&node.Depth<51;++level)
            {
                const double middle=.5*(node.A+node.B);FIntervalBezierCenterPath left,right;
                if(!TestTrue(TEXT("every child split remains bound to the original global time"),T::FMonotoneTimeSeedTestAccess::Split(search,node,middle,left,right)))return false;
                const auto hull=CenterDetail::Hull(node.Curve),lh=CenterDetail::Hull(left),rh=CenterDetail::Hull(right);
                for(int32 d=0;d<3;++d)
                {
                    const double exact=middle*double(1<<d);
                    if(!TestTrue(TEXT("global coordinate and child hulls are certified after repeated non-dyadic splitting"),left.ControlPoints.Last().Axis[d].Lower<=exact&&left.ControlPoints.Last().Axis[d].Upper>=exact&&
                        lh.Axis[d].Lower>=hull.Axis[d].Lower&&lh.Axis[d].Upper<=hull.Axis[d].Upper&&rh.Axis[d].Lower>=hull.Axis[d].Lower&&rh.Axis[d].Upper<=hull.Axis[d].Upper))return false;
                }
                if(!TestTrue(TEXT("shared exact child endpoint and inherited outer endpoints"),MonotoneSeedTests::SamePoint(left.ControlPoints.Last(),right.ControlPoints[0])&&
                    MonotoneSeedTests::SamePoint(left.ControlPoints[0],node.Curve.ControlPoints[0])&&MonotoneSeedTests::SamePoint(right.ControlPoints.Last(),node.Curve.ControlPoints.Last())))return false;
                node=level%2?T::FTimeNode{node.A,middle,node.Depth+1,MoveTemp(left)}:T::FTimeNode{middle,node.B,node.Depth+1,MoveTemp(right)};
            }
            if(!TestTrue(TEXT("global depth increased from the narrowed root rather than restarting at zero"),node.Depth>initialDepth&&node.Depth<=51))return false;
        }
        T::FTimeNode node;FDistanceBudget unavailable{107,3,100};
        if(!TestTrue(TEXT("unavailable8 allowance does not spend protected work"),T::BuildDirectInitialTimeFrontier(path,.1,.10001,unavailable,node)==E::NotApplicable&&unavailable.Used==3&&unavailable.Remaining==107))return false;
        FDistanceBudget wide{1000};if(!TestTrue(TEXT("broad hint keeps the ordinary root"),T::BuildDirectInitialTimeFrontier(path,0,1,wide,node)==E::NotApplicable&&wide.Used==1))return false;
        FDistanceBudget bad{1000,0,-1};if(!TestTrue(TEXT("invalid account is Work"),T::BuildDirectInitialTimeFrontier(path,.1,.10001,bad,node)==E::Work))return false;
        FDistanceBudget reversed{1000};if(!TestTrue(TEXT("reversed global interval is Arithmetic"),T::BuildDirectInitialTimeFrontier(path,.2,.1,reversed,node)==E::Arithmetic))return false;
        auto quadratic=path;quadratic.Degree=2;quadratic.ControlPoints.Insert(Detail::Point(FVector3d(0,3,1)),1);FDistanceBudget nonLinear{1000};
        if(!TestTrue(TEXT("non-admitted original quadratic is never replaced by its chord"),T::BuildDirectInitialTimeFrontier(quadratic,.1,.10001,nonLinear,node)==E::NotApplicable&&nonLinear.Used==0))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDirectInitialTimeFrontierContactTest,
    "IAmSpeed.AnalyticWorld.DirectInitialTimeFrontier.FirstEntryAndRetainedContinuation",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDirectInitialTimeFrontierContactTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace T=TimeDetail;namespace A=::AnalyticCertified;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> offset(TEXT("p.IAmSpeed.AnalyticWorld.OffsetCurveTimeHint"),1,FName(TEXT("DirectTimeOffset")));
    FScopedCVar<int32> chord(TEXT("p.IAmSpeed.AnalyticWorld.ChordEnvelopeTimeHint"),0,FName(TEXT("DirectTimeNoChord")));
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(bool curved:{false,true})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        FIntervalTensorSurface surface;surface.DegreeU=0;surface.DegreeV=5;
        for(int32 i=0;i<6;++i)surface.ControlPoints.Add(Detail::Point(FVector3d(4*i,curved?i*(i-1):0,0)));
        TArray<FIntervalTensorSurface> surfaces{surface};FIntervalBezierCenterPath path;
        const double radius=curved?2.:1.,exact=curved?1.-2./FMath::Sqrt(32.):.5;
        if(!CenterDetail::Linear(curved?FVector3d(6,9,0):FVector3d(10,2,0),curved?FVector3d(10,5,0):FVector3d(10,0,0),path))return false;
        FDistanceBudget oldWork{1000000},newWork{1000000};
        T::FRetainedSphereTimeSearch oracle(surfaces,path,radius,oldWork,1.e-7,512,nullptr,false,nullptr,true,false,false,!curved,curved);
        T::FRetainedSphereTimeSearch actual(surfaces,path,radius,newWork,1.e-7,512,nullptr,false,nullptr,true,false,false,!curved,curved);
        oracle.SetCertifiedPrefixClearBeforeStart(true);actual.SetCertifiedPrefixClearBeforeStart(true);
        if(!TestTrue(TEXT("fresh search admits the explicit direct-frontier prototype"),actual.SetDirectInitialTimeFrontierBeforeStart(true)))return false;
        const auto old=oracle.Advance(1.e-5),first=actual.Advance(1.e-5);
        const auto valid=[&](const FTensorSphereTimeBounds& x){return x.Status==ETensorSphereTimeStatus::FirstContactBracket&&x.InitialSeparationProved&&x.LowerTime<=exact&&x.UpperTime>=exact;};
        if(!TestTrue(TEXT("direct original interval and old frontier enclose the same real first entry"),valid(old)&&valid(first)&&actual.HasDirectInitialTimeFrontierForInspection()))
        {AddInfo(FString::Printf(TEXT("curve=%d oldstatus=%u status=%u failure=%u direct=%d bounds=[%.17g,%.17g] exact=%.17g"),curved,uint32(old.Status),uint32(first.Status),uint32(actual.GetFailureForInspection()),actual.HasDirectInitialTimeFrontierForInspection(),first.LowerTime,first.UpperTime,exact));return false;}
        if(!TestTrue(TEXT("initial original proof eliminates ancestor-node administration"),actual.GetTimeNodeVisitsForInspection()<oracle.GetTimeNodeVisitsForInspection()))return false;
        if(!TestFalse(TEXT("mode is immutable after initialization"),actual.SetDirectInitialTimeFrontierBeforeStart(false)))return false;
        auto prior=first;
        for(double gap:{1.e-6,1.e-7})
        {
            const auto next=actual.Advance(gap);
            if(!TestTrue(TEXT("retained frontier refines the SAME nested first contact"),valid(next)&&next.LowerTime>=prior.LowerTime&&next.UpperTime<=prior.UpperTime&&
                A::Subtract(Interval::Exact(next.UpperTime),Interval::Exact(next.LowerTime)).Upper<=gap))return false;
            FIntervalPoint originalWitness;
            if(!ProjectedDistanceDetail::EvaluateFixedUV(surfaces[0],next.InsideSampleUV,originalWitness))return false;
            const auto center=CenterDetail::Evaluate(path,Interval::Exact(next.UpperTime));
            const auto distance=ContactDetail::SquaredNorm(ContactDetail::Subtract(originalWitness,center));
            if(!TestTrue(TEXT("the published UV is a newly validated ORIGINAL surface witness at actual upper time"),A::Valid(distance)&&distance.Upper<A::Square(Interval::Exact(radius)).Lower))return false;
            prior=next;
        }
        while(newWork.Remaining>newWork.ConsumptionFloor)if(!newWork.Consume())return false;
        const uint64 consumed=newWork.Used;const auto unknown=actual.Advance(1.e-8);
        if(!TestTrue(TEXT("exhausted continuation stays Unknown/Work with no reset or old result"),unknown.Status==ETensorSphereTimeStatus::Unknown&&actual.GetFailureForInspection()==T::ETimeSearchFailure::Work&&newWork.Used==consumed))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDirectInitialTimeFrontierFallbackTest,
    "IAmSpeed.AnalyticWorld.DirectInitialTimeFrontier.FallbackAndConsumerContinuation",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDirectInitialTimeFrontierFallbackTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace T=TimeDetail;namespace A=::AnalyticCertified;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    // This fixture exercises the generic retained route and its subdivision costs.
    // The complete direct-contact route has separate integration/continuation tests.
    FScopedCVar<int32> noDirectLine(TEXT("p.IAmSpeed.AnalyticWorld.DirectCoordinateLineContact"),0,FName(TEXT("TimeFrontierRouteIsolationLine")));
    FScopedCVar<int32> noDirectOffset(TEXT("p.IAmSpeed.AnalyticWorld.DirectOffsetCurveContact"),0,FName(TEXT("TimeFrontierRouteIsolationOffset")));
    FIntervalTensorSurface line;line.DegreeU=0;line.DegreeV=5;for(int32 i=0;i<6;++i)line.ControlPoints.Add(Detail::Point(FVector3d(4*i,0,0)));
    TArray<FIntervalTensorSurface> surfaces{line};FIntervalBezierCenterPath curved;
    curved.Degree=2;curved.ControlPoints={Detail::Point(FVector3d(10,2,0)),Detail::Point(FVector3d(10,-2,0)),Detail::Point(FVector3d(10,2,0))};
    FDistanceBudget a{1000000},b{1000000};
    T::FRetainedSphereTimeSearch oracle(surfaces,curved,1.,a,1.e-7,512,nullptr,false,nullptr,true,false,false,true,false);
    T::FRetainedSphereTimeSearch actual(surfaces,curved,1.,b,1.e-7,512,nullptr,false,nullptr,true,false,false,true,false);
    actual.SetDirectInitialTimeFrontierBeforeStart(true);const auto old=oracle.Advance(1.e-5),unchanged=actual.Advance(1.e-5);
    const double firstEntry=.5-FMath::Sqrt(2.)*.25;
    if(!TestTrue(TEXT("a continuous enter-and-exit with clear endpoints preserves old code and debit without a hint"),
        old.Status==ETensorSphereTimeStatus::FirstContactBracket&&unchanged.Status==old.Status&&unchanged.LowerTime<=firstEntry&&unchanged.UpperTime>=firstEntry&&
        A::Bits(old.LowerTime)==A::Bits(unchanged.LowerTime)&&A::Bits(old.UpperTime)==A::Bits(unchanged.UpperTime)&&a.Used==b.Used&&a.Remaining==b.Remaining&&!actual.HasDirectInitialTimeFrontierForInspection()))return false;
    FIntervalBezierCenterPath endpoint;if(!CenterDetail::Linear(FVector3d(-5,0,0),FVector3d(.5,0,0),endpoint))return false;
    FDistanceBudget lineEndpointProof{1000000};double endpointA=0,endpointB=1;
    if(!TestTrue(TEXT("finite endpoint is outside the coordinate-line tangent coverage"),
        T::TryCoordinateLineTimeHint(surfaces,endpoint,1.,lineEndpointProof,1.e-5,endpointA,endpointB)==T::EMonotoneCurveTimeSeed::NotApplicable))return false;
    // Last constructor argument controls the CHORD/OFFSET wrapper, not the
    // coordinate-line proof. Disable it to isolate this finite-line fallback.
    FDistanceBudget endpointWork{1000000};T::FRetainedSphereTimeSearch caps(surfaces,endpoint,1.,endpointWork,1.e-7,512,nullptr,false,nullptr,true,false,false,true,false);
    caps.SetDirectInitialTimeFrontierBeforeStart(true);const auto cap=caps.Advance(1.e-5);
    AddInfo(FString::Printf(TEXT("FiniteLineFallback Status=%u Failure=%u Direct=%d A=%.17g B=%.17g Used=%llu"),
        uint32(cap.Status),uint32(caps.GetFailureForInspection()),int32(caps.HasDirectInitialTimeFrontierForInspection()),cap.LowerTime,cap.UpperTime,endpointWork.Used));
    if(!TestTrue(TEXT("non-admitted finite endpoint contact remains generic"),cap.Status==ETensorSphereTimeStatus::FirstContactBracket&&cap.LowerTime<=8./11&&cap.UpperTime>=8./11&&!caps.HasDirectInitialTimeFrontierForInspection()))return false;
    FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.DirectInitialTimeFrontier"),1,FName(TEXT("DirectTimeContact")));
    FScopedCVar<int32> offset(TEXT("p.IAmSpeed.AnalyticWorld.OffsetCurveTimeHint"),1,FName(TEXT("DirectTimeContactOffset")));
    FScopedCVar<int32> chord(TEXT("p.IAmSpeed.AnalyticWorld.ChordEnvelopeTimeHint"),0,FName(TEXT("DirectTimeContactNoChord")));
    FScopedCVar<int32> monotone(TEXT("p.IAmSpeed.AnalyticWorld.MonotoneCurveTimeSeed"),0,FName(TEXT("DirectTimeContactNoMonotone")));
    FScopedCVar<int32> dyadic(TEXT("p.IAmSpeed.AnalyticWorld.MonotoneCurveDyadicHint"),0,FName(TEXT("DirectTimeContactNoDyadic")));
    FScopedCVar<int32> spatial(TEXT("p.IAmSpeed.AnalyticWorld.OffsetSpatialDomain"),1,FName(TEXT("DirectTimeSpatialTogether")));
    FScopedCVar<int32> noLine(TEXT("p.IAmSpeed.AnalyticWorld.CoordinateLineTimeHint"),0,FName(TEXT("DirectTimeContactNoLine")));
    FScopedCVar<int32> noLineSpatial(TEXT("p.IAmSpeed.AnalyticWorld.CoordinateLineSpatialDomain"),0,FName(TEXT("DirectTimeContactNoLineSpatial")));
    surfaces[0]=MonotoneSeedTests::Parabola();FIntervalBezierCenterPath path;if(!CenterDetail::Linear(FVector3d(6,9,0),FVector3d(10,5,0),path))return false;
    FDistanceBudget work{1000000};int32 calls=0;
    const auto callback=[&](const FTensorSphereContactEnclosure&){return ++calls<3?EContactEnclosureDecision::Refine:EContactEnclosureDecision::Accept;};const FContactEnclosureFilter filter(callback);
    const auto result=BoundTensorSphereFirstContact(surfaces,path,2.,work,1.e-5,1.e-7,512,.02,.02,24,&filter);const double exact=1.-2./FMath::Sqrt(32.);
    if(!TestTrue(TEXT("direct temporal and spatial seeds retain the same consumer continuation"),result.Status==ETensorSphereContactStatus::FirstContactEnclosed&&
        result.LowerTime<=exact&&result.UpperTime>=exact&&result.SurfacePoint.Axis[0].Lower<=10&&result.SurfacePoint.Axis[0].Upper>=10&&
        result.SurfacePoint.Axis[1].Lower<=5&&result.SurfacePoint.Axis[1].Upper>=5&&calls==3&&result.ConsumerWidthRefinements==2))
    {AddInfo(FString::Printf(TEXT("status=%u failure=%u site=%s calls=%d temporal=%d"),uint32(result.Status),uint32(result.Failure),result.WorkSite,calls,result.TemporalWidthRefinements));return false;}
    FDistanceBudget refusalWork{1000000};int32 refusedCalls=0;
    const auto refuse=[&](const FTensorSphereContactEnclosure&){++refusedCalls;return EContactEnclosureDecision::Refuse;};const FContactEnclosureFilter refusal(refuse);
    const auto refused=BoundTensorSphereFirstContact(surfaces,path,2.,refusalWork,1.e-5,1.e-7,512,.02,.02,24,&refusal);
    if(!TestTrue(TEXT("refusal still exposes no provisional contact"),refused.Status==ETensorSphereContactStatus::Unknown&&refused.Leaves.IsEmpty()&&refused.Failure==ETensorSphereContactFailure::ConsumerRefusal&&refusedCalls==1))return false;
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
#endif

namespace Speed::Analytic::Certified
{
namespace TensorPlanarFamilyControl
{
static TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.TensorPlanarFamily"),0,
    TEXT("Default OFF. Exact finite coverage and original UV bounds for coordinate-monotone planar tensors; ordinary certified bounds before repeated UV minima."),ECVF_Default);
bool RuntimeEnabled(){return Enabled.GetValueOnAnyThread()==1;}
}
namespace ContactDetail
{
namespace TensorPlanarDetail
{
namespace A=::AnalyticCertified;
using E=EPlanarProjectionProof;
struct FCoordinate
{
    Interval CP[6],Derivative;
    double Scalar[6]{},ScalarDerivative[5]{};
    int32 Axis=INDEX_NONE;
};
struct FModel {FCoordinate U,V;bool Applicable=false;};
struct FEntry
{
    FIntervalTensorSurface Surface;FModel Model;
    uint64 Control=0,Hash=0;uint32 RequiredSticky=0;int32 Normal=INDEX_NONE;bool Ready=false;
};
struct FCache {FEntry Entries[64];};
static FCache& Cache(){static thread_local FCache value;return value;}
struct FCounters
{
    std::atomic<uint64> Calls{0},Hits{0},Misses{0},Covered{0},Outside{0},Domains{0},NotApplicable{0},Ticks{0};
};
static FCounters Counts;
static bool Pay(FDistanceBudget& work,int32 count)
{if(count<0||int64(work.Remaining)-work.ConsumptionFloor<count||work.Used>MAX_uint64-uint64(count))return false;
 for(int32 i=0;i<count;++i)if(!work.Consume())return false;return true;}
static Interval Hull(Interval a,Interval b)
{return {FMath::Min(a.Lower,b.Lower),FMath::Max(a.Upper,b.Upper),true};}
static bool Same(const FIntervalTensorSurface& a,const FIntervalTensorSurface& b)
{
    if(a.DegreeU!=b.DegreeU||a.DegreeV!=b.DegreeV||a.ControlPoints.Num()!=b.ControlPoints.Num())return false;
    for(int32 i=0;i<a.ControlPoints.Num();++i)for(int32 d=0;d<3;++d)
    {const auto& x=a.ControlPoints[i].Axis[d];const auto& y=b.ControlPoints[i].Axis[d];
     if(x.Known!=y.Known||A::Bits(x.Lower)!=A::Bits(y.Lower)||A::Bits(x.Upper)!=A::Bits(y.Upper))return false;}
    return true;
}
static uint64 Hash(const FIntervalTensorSurface& surface,uint64 control,int32 normal)
{
    uint64 h=1469598103934665603ull;const auto add=[&](uint64 value){h^=value;h*=1099511628211ull;};
    add(control);add(uint64(normal));
    for(const auto& p:surface.ControlPoints)for(const auto& a:p.Axis){add(A::Bits(a.Lower));add(A::Bits(a.Upper));}
    h^=h>>33;h*=0xff51afd7ed558ccdull;h^=h>>33;h*=0xc4ceb9fe1a85ec53ull;h^=h>>33;return h;
}
static bool Coordinate(const FIntervalTensorSurface& surface,int32 axis,bool alongU,FCoordinate& out)
{
    out={};out.Axis=axis;
    for(int32 i=0;i<6;++i)
    {
        Interval c=surface.ControlPoints[alongU?i*6:i].Axis[axis];
        for(int32 j=1;j<6;++j)c=Hull(c,surface.ControlPoints[alongU?i*6+j:j*6+i].Axis[axis]);
        out.CP[i]=c;out.Scalar[i]=.5*c.Lower+.5*c.Upper;
        if(!A::Valid(c)||!A::Finite(out.Scalar[i]))return false;
    }
    for(int32 i=0;i<5;++i)
    {
        const auto d=A::Multiply(Interval::Exact(5),A::Subtract(out.CP[i+1],out.CP[i]));
        if(!A::Valid(d))return false;
        out.Derivative=i?Hull(out.Derivative,d):d;
        out.ScalarDerivative[i]=5.*(out.Scalar[i+1]-out.Scalar[i]);
    }
    return out.Derivative.Lower>0||out.Derivative.Upper<0;
}
static FModel Build(const FIntervalTensorSurface& surface,int32 normal)
{
    const int32 x=(normal+1)%3,y=(normal+2)%3;FModel result;
    if(Coordinate(surface,x,true,result.U)&&Coordinate(surface,y,false,result.V))result.Applicable=true;
    else if(Coordinate(surface,y,true,result.U)&&Coordinate(surface,x,false,result.V))result.Applicable=true;
    return result;
}
static const FModel& Model(const FIntervalTensorSurface& surface,uint64 control,int32 normal,bool observe)
{
    const uint64 hash=Hash(surface,control,normal);auto& entry=Cache().Entries[hash&63];
    if(entry.Ready&&entry.Hash==hash&&entry.Control==control&&entry.Normal==normal&&(VectorGetControlRegister()&entry.RequiredSticky)==entry.RequiredSticky&&Same(entry.Surface,surface))
    {if(observe)++Counts.Hits;return entry.Model;}
    const auto model=Build(surface,normal);
    // Only complete applicable immutable properties are retained. Requiring
    // all post-computation sticky bits prevents a hit from erasing side effects.
    entry.Ready=false;entry.Surface=surface;entry.Control=control;entry.Normal=normal;entry.Hash=hash;entry.Model=model;entry.RequiredSticky=VectorGetControlRegister()&0x3fu;entry.Ready=model.Applicable;
    if(observe)++Counts.Misses;return entry.Model;
}
static double ScalarValue(const double* cp,int32 degree,double parameter)
{
    double values[6]{};for(int32 i=0;i<=degree;++i)values[i]=cp[i];
    for(int32 n=degree;n>0;--n)for(int32 i=0;i<n;++i)values[i]=(1.-parameter)*values[i]+parameter*values[i+1];
    return values[0];
}
static bool Inverse(const FCoordinate& coordinate,Interval target,FDistanceBudget& work,Interval& necessary)
{
    double parameter=.5;
    const double middle=.5*target.Lower+.5*target.Upper;
    if(A::Finite(middle))
    {
        double lo=0,hi=1;const bool increasing=coordinate.Derivative.Lower>0;
        for(int32 k=0;k<8;++k)
        {
            if(!Pay(work,1))return false;
            const double value=ScalarValue(coordinate.Scalar,5,parameter),d=ScalarValue(coordinate.ScalarDerivative,4,parameter);
            if(!A::Finite(value)||!A::Finite(d)||!(increasing?d>0:d<0))break;
            const double residual=value-middle;if(!A::Finite(residual)||residual==0)break;
            if(increasing?(residual>0):(residual<0))hi=parameter;else lo=parameter;
            double next=parameter-residual/d;
            if(!A::Finite(next)||next<lo||next>hi)next=.5*lo+.5*hi;
            if(next==parameter)break;parameter=FMath::Clamp(next,0.,1.);
        }
    }
    if(!Pay(work,4))return false;
    Interval values[6];for(int32 i=0;i<6;++i)values[i]=coordinate.CP[i];
    const auto t=Interval::Exact(parameter),one=A::Subtract(Interval::Exact(1),t);
    for(int32 n=5;n>0;--n)for(int32 i=0;i<n;++i)
        values[i]=A::Add(A::Multiply(one,values[i]),A::Multiply(t,values[i+1]));
    auto numerator=A::Subtract(target,values[0]),denominator=coordinate.Derivative;
    if(denominator.Upper<0){numerator=A::Negate(numerator);denominator=A::Negate(denominator);}
    Interval correction;if(!ProjectionDividePositive(numerator,denominator,correction))return false;
    necessary=A::Add(t,correction);return A::Valid(necessary);
}
static bool Covers(const FCoordinate& coordinate,Interval target)
{
    if(coordinate.Derivative.Lower>0)return target.Lower>coordinate.CP[0].Upper&&target.Upper<coordinate.CP[5].Lower;
    return target.Lower>coordinate.CP[5].Upper&&target.Upper<coordinate.CP[0].Lower;
}
static E Run(const FIntervalTensorSurface& surface,int32 normal,const FIntervalPoint& foot,
    FDistanceBudget& budget,FProjectedProviderUV& out,bool& covers,bool observe)
{
    out={};covers=false;
    const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=EAnalyticNumericStatus::Complete)return E::Arithmetic;
    if(budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor)return E::Work;
    if(surface.DegreeU!=5||surface.DegreeV!=5||surface.ControlPoints.Num()!=36)return E::NotApplicable;
    if(normal<0||normal>=3||!Detail::ValidPoint(foot))return E::Arithmetic;
    if(int64(budget.Remaining)-budget.ConsumptionFloor<128||budget.Used>MAX_uint64-128)return E::NotApplicable;
    FScopedDistanceBudgetFloor limit(budget,budget.Remaining-128);if(!limit.IsValid())return E::Work;
    const auto height=surface.ControlPoints[0].Axis[normal];
    if(!A::Valid(height)||A::Bits(height.Lower)!=A::Bits(height.Upper))return E::NotApplicable;
    for(const auto& cp:surface.ControlPoints)
    {if(!Pay(budget,1))return E::Work;if(!Detail::ValidPoint(cp))return E::Arithmetic;
     if(A::Bits(cp.Axis[normal].Lower)!=A::Bits(height.Lower)||A::Bits(cp.Axis[normal].Upper)!=A::Bits(height.Lower))return E::NotApplicable;}
    if(A::Bits(foot.Axis[normal].Lower)!=A::Bits(height.Lower)||A::Bits(foot.Axis[normal].Upper)!=A::Bits(height.Lower))return E::Arithmetic;
    // The same immutable-property debit is paid on cache hits and misses.
    if(!Pay(budget,16))return E::Work;
    const auto& model=Model(surface,numeric.ControlFingerprint(),normal,observe);
    if(!model.Applicable)return E::NotApplicable;
    Interval u,v;
    if(!Inverse(model.U,foot.Axis[model.U.Axis],budget,u)||!Inverse(model.V,foot.Axis[model.V.Axis],budget,v))
    {
        const auto current=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(current.Status!=EAnalyticNumericStatus::Complete||current.ControlFingerprint()!=numeric.ControlFingerprint())return E::Arithmetic;
        return budget.Remaining<=budget.ConsumptionFloor?E::Work:E::NotApplicable;
    }
    const auto current=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(current.Status!=EAnalyticNumericStatus::Complete||current.ControlFingerprint()!=numeric.ControlFingerprint())return E::Arithmetic;
    FProjectedProviderUV domain;
    if(u.Upper<0||u.Lower>1||v.Upper<0||v.Lower>1)domain.Outside=true;
    else{domain.U={FMath::Max(0.,u.Lower),FMath::Min(1.,u.Upper),true};domain.V={FMath::Max(0.,v.Lower),FMath::Min(1.,v.Upper),true};domain.Bounded=true;}
    covers=Covers(model.U,foot.Axis[model.U.Axis])&&Covers(model.V,foot.Axis[model.V.Axis]);
    if(covers&&domain.Outside)return E::Arithmetic;
    out=domain;if(observe){if(covers)++Counts.Covered;if(domain.Outside)++Counts.Outside;else ++Counts.Domains;}
    return E::Proved;
}
static void Publish()
{
    UE_LOG(LogTemp,Display,TEXT("[TensorPlanarFamily] Scope=ProgramPrefixAtPreExit OriginalCP=1 DefaultOFF=1 Calls=%llu PropertyHits=%llu PropertyMisses=%llu Covered=%llu Outside=%llu Domains=%llu NotApplicable=%llu Ticks=%llu SecondsPerCycle=%.17g"),
        Counts.Calls.load(),Counts.Hits.load(),Counts.Misses.load(),Counts.Covered.load(),Counts.Outside.load(),Counts.Domains.load(),Counts.NotApplicable.load(),Counts.Ticks.load(),FPlatformTime::GetSecondsPerCycle64());
}
struct FRegister{FRegister(){FCoreDelegates::OnPreExit.AddStatic(&Publish);}};static FRegister Register;
}
EPlanarProjectionProof TensorPlanarProjectedDomain(const FIntervalTensorSurface& surface,int32 normal,const FIntervalPoint& foot,
    FDistanceBudget& budget,FProjectedProviderUV& out,bool& covers)
{
    namespace D=TensorPlanarDetail;const bool observe=StrongConvexCurveMinimum::Observation::Enabled();
    const uint64 start=observe?FPlatformTime::Cycles64():0;if(observe)++D::Counts.Calls;
    const auto result=D::Run(surface,normal,foot,budget,out,covers,observe);
    if(observe){D::Counts.Ticks.fetch_add(FPlatformTime::Cycles64()-start);if(result==EPlanarProjectionProof::NotApplicable)++D::Counts.NotApplicable;}
    return result;
}
}
}


#if WITH_DEV_AUTOMATION_TESTS
namespace TensorPlanarFamilyFixture
{
using namespace Speed::Analytic::Certified;
static FIntervalTensorSurface Patch(int32 normal,int32 sign=1,double offset=0)
{
    FIntervalTensorSurface surface;surface.DegreeU=5;surface.DegreeV=5;
    const double q[6]={0,1,2,5,12,20};const int32 x=(normal+1)%3,y=(normal+2)%3;
    for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)
    {FVector3d p=FVector3d::ZeroVector;p[x]=offset+sign*q[u];p[y]=q[v];surface.ControlPoints.Add(Detail::Point(p));}
    return surface;
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTensorPlanarFamilyDomainsTest,
    "IAmSpeed.AnalyticWorld.TensorPlanarFamily.MirandaAndNecessaryDomains",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FTensorPlanarFamilyDomainsTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace A=::AnalyticCertified;namespace D=ContactDetail;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.TensorPlanarFamily"),1,FName(TEXT("TensorPlanarDomains")));
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        for(int32 normal:{0,1,2})for(int32 sign:{-1,1})
        {
            auto surface=TensorPlanarFamilyFixture::Patch(normal,sign);FIntervalPoint foot;
            if(!ProjectedDistanceDetail::EvaluateFixedUV(surface,FVector2d(.375,.625),foot))return false;
            foot.Axis[normal]=Interval::Exact(0);FDistanceBudget work{10000};D::FProjectedProviderUV domain;bool covers=false;
            if(!TestTrue(TEXT("nonuniform planar tensor proves finite coverage and all original preimages"),
                D::TensorPlanarProjectedDomain(surface,normal,foot,work,domain,covers)==D::EPlanarProjectionProof::Proved&&covers&&domain.Bounded&&!domain.Outside&&
                domain.U.Lower<=.375&&domain.U.Upper>=.375&&domain.V.Lower<=.625&&domain.V.Upper>=.625))return false;
            const uint64 first=work.Used;FDistanceBudget repeated{10000};D::FProjectedProviderUV second;bool secondCovers=false;
            if(!TestTrue(TEXT("pure property reuse preserves bounds and logical debit"),
                D::TensorPlanarProjectedDomain(surface,normal,foot,repeated,second,secondCovers)==D::EPlanarProjectionProof::Proved&&secondCovers&&repeated.Used==first&&
                A::Bits(domain.U.Lower)==A::Bits(second.U.Lower)&&A::Bits(domain.U.Upper)==A::Bits(second.U.Upper)&&
                A::Bits(domain.V.Lower)==A::Bits(second.V.Lower)&&A::Bits(domain.V.Upper)==A::Bits(second.V.Upper)))return false;
            auto family=surface;
            for(auto& point:family.ControlPoints)for(int32 d=0;d<3;++d)if(d!=normal)
                point.Axis[d]={point.Axis[d].Lower-1.e-6,point.Axis[d].Upper+1.e-6,true};
            FDistanceBudget uncertain{10000};D::FProjectedProviderUV uncertainDomain;bool uncertainCovers=false;
            if(!TestTrue(TEXT("independent non-normal coefficients retain every original realization"),
                D::TensorPlanarProjectedDomain(family,normal,foot,uncertain,uncertainDomain,uncertainCovers)==D::EPlanarProjectionProof::Proved&&uncertainCovers&&
                uncertainDomain.U.Lower<=.375&&uncertainDomain.U.Upper>=.375&&uncertainDomain.V.Lower<=.625&&uncertainDomain.V.Upper>=.625))return false;
            const int32 tangent=(normal+1)%3;const auto keyBefore=D::TensorPlanarDetail::Hash(surface,InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint(),normal);
            surface.ControlPoints[14].Axis[tangent]=Interval::Exact(A::FromBits(A::Bits(surface.ControlPoints[14].Axis[tangent].Lower)+1));
            if(!TestTrue(TEXT("one coefficient bit changes immutable cache identity"),D::TensorPlanarDetail::Hash(surface,InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint(),normal)!=keyBefore))return false;
        }
        auto surface=TensorPlanarFamilyFixture::Patch(2);auto foot=Detail::Point(FVector3d(5,5,0));D::FProjectedProviderUV domain;bool covers=false;
        FDistanceBudget low{140,7,13};
        if(!TestTrue(TEXT("optional reservation protects caller floor before any debit"),D::TensorPlanarProjectedDomain(surface,2,foot,low,domain,covers)==D::EPlanarProjectionProof::NotApplicable&&low.Used==7&&low.Remaining==140&&low.ConsumptionFloor==13))return false;
        for(int32 v=0;v<6;++v)surface.ControlPoints[3*6+v].Axis[0]=Interval::Exact(-2);
        FDistanceBudget fold{10000};
        if(!TestTrue(TEXT("coplanarity alone does not admit a folded parameterization"),D::TensorPlanarProjectedDomain(surface,2,foot,fold,domain,covers)==D::EPlanarProjectionProof::NotApplicable))return false;
        surface=TensorPlanarFamilyFixture::Patch(2);
        for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)surface.ControlPoints[u*6+v].Axis[0]=Interval::Exact(surface.ControlPoints[u*6+v].Axis[0].Lower+3.*v);
        TArray<FIntervalTensorSurface> nonrect{surface};D::FPlanarProjectionDomains unsupported;FDistanceBudget edge{10000};
        if(!TestTrue(TEXT("nonrectangular boundary without uniform coverage remains generic"),D::ProjectedPlanarDomains(nonrect,foot,edge,unsupported,true)==D::EPlanarProjectionProof::NotApplicable&&unsupported.Providers.IsEmpty()))return false;
        surface=TensorPlanarFamilyFixture::Patch(2);surface.ControlPoints[9].Axis[2]=Interval::Exact(.01);FDistanceBudget nonplane{10000};
        if(!TestTrue(TEXT("one displaced coefficient rejects the plane shortcut"),D::TensorPlanarProjectedDomain(surface,2,foot,nonplane,domain,covers)==D::EPlanarProjectionProof::NotApplicable))return false;
        surface=TensorPlanarFamilyFixture::Patch(2);surface.ControlPoints[9].Axis[0].Known=false;FDistanceBudget invalid{10000};
        if(!TestTrue(TEXT("unknown original coefficient is not cached or admitted"),D::TensorPlanarProjectedDomain(surface,2,foot,invalid,domain,covers)==D::EPlanarProjectionProof::Arithmetic))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTensorPlanarFamilyBoundsTest,
    "IAmSpeed.AnalyticWorld.TensorPlanarFamily.OriginalWitnessAndMixedFamilies",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FTensorPlanarFamilyBoundsTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace A=::AnalyticCertified;namespace D=ContactDetail;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.TensorPlanarFamily"),1,FName(TEXT("TensorPlanarBounds")));
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        TArray<FIntervalTensorSurface> surfaces{TensorPlanarFamilyFixture::Patch(2)};FIntervalBezierCenterPath path;
        if(!CenterDetail::Linear(FVector3d(5,5,2),FVector3d(5,5,.5),path))return false;
        FDistanceBudget work{100000};TUniquePtr<FPlanarFamilyTimeShortcutPlan> plan;
        if(!TestTrue(TEXT("5x5 original plane is routed before general minima"),FPlanarFamilyTimeShortcutPlan::Build(surfaces,path,work,plan)==EPlanarFamilyBuild::Ready&&plan))return false;
        FTensorDistanceBounds bound;
        if(!TestTrue(TEXT("support plane is a strict clear lower bound"),plan->TryProvider(surfaces,0,Detail::Point(FVector3d(5,5,2)),Interval::Exact(1),work,bound)==EPlanarFamilyShortcut::Clear))return false;
        const auto tangent=plan->TryProvider(surfaces,0,Detail::Point(FVector3d(5,5,1)),Interval::Exact(1),work,bound);
        if(!TestTrue(TEXT("a non-strict threshold returns genuine finite bounds rather than another UV minimum"),tangent==EPlanarFamilyShortcut::Bounded&&bound.Status==EDistanceBoundStatus::Bounded&&bound.LowerSquared<=1&&bound.UpperSquared>=1))return false;
        if(!TestTrue(TEXT("strict inside requires an original evaluated UV point"),plan->TryProvider(surfaces,0,Detail::Point(FVector3d(5,5,.5)),Interval::Exact(1),work,bound)==EPlanarFamilyShortcut::Inside&&bound.UpperSquared<1))return false;
        FIntervalPoint witness;if(!ProjectedDistanceDetail::EvaluateFixedUV(surfaces[0],bound.SampleUV,witness))return false;
        for(int32 d=0;d<3;++d)if(!TestTrue(TEXT("published witness is the original tensor evaluation"),A::Bits(witness.Axis[d].Lower)==A::Bits(bound.SamplePoint.Axis[d].Lower)&&A::Bits(witness.Axis[d].Upper)==A::Bits(bound.SamplePoint.Axis[d].Upper)))return false;
        FIntervalTensorSurface triangle;triangle.DegreeU=1;triangle.DegreeV=1;
        triangle.ControlPoints={Detail::Point(FVector3d(-100,-100,0)),Detail::Point(FVector3d(100,-100,0)),Detail::Point(FVector3d(0,100,0)),Detail::Point(FVector3d(0,100,0))};
        surfaces={triangle,TensorPlanarFamilyFixture::Patch(2,1,50)};
        if(!CenterDetail::Linear(FVector3d(0,5,2),FVector3d(0,5,.5),path))return false;
        FDistanceBudget mixed{100000};TUniquePtr<FPlanarFamilyTimeShortcutPlan> mixedPlan;
        if(!TestTrue(TEXT("native triangle and high degree exact plane share a finite family"),FPlanarFamilyTimeShortcutPlan::Build(surfaces,path,mixed,mixedPlan)==EPlanarFamilyBuild::Ready&&mixedPlan))return false;
        auto center=Detail::Point(FVector3d(0,5,.5));
        if(!TestTrue(TEXT("member outside foot cannot become falsely clear or inside"),mixedPlan->TryProvider(surfaces,1,center,Interval::Exact(1),mixed,bound)==EPlanarFamilyShortcut::Bounded&&bound.LowerSquared<=.25&&bound.UpperSquared>1&&bound.SampleUV==FVector2d(0,0)))return false;
        TArray<FPlanarFamilySpatialProvider> providers;
        if(!TestTrue(TEXT("outside domain is exposed only under whole-family finite coverage"),mixedPlan->ProjectSpatialFamilies(surfaces,center,mixed,providers)==EPlanarFamilySpatialProjection::Complete&&providers.Num()==2&&providers[0].Proved&&providers[1].Proved&&providers[1].Domain.Outside))return false;
        TArray<FIntervalTensorSurface> lone{surfaces[1]};D::FPlanarProjectionDomains noCoverage;FDistanceBudget loneWork{10000};
        if(!TestTrue(TEXT("an outside sole patch preserves its boundary minima"),D::ProjectedPlanarDomains(lone,center,loneWork,noCoverage,true)==D::EPlanarProjectionProof::NotApplicable&&noCoverage.Providers.IsEmpty()))return false;
        FScopedCVar<int32> off(TEXT("p.IAmSpeed.AnalyticWorld.TensorPlanarFamily"),0,FName(TEXT("TensorPlanarLegacy")));
        FDistanceBudget legacy{10000};TUniquePtr<FPlanarFamilyTimeShortcutPlan> legacyPlan;
        if(!TestTrue(TEXT("default OFF preserves original high-degree generic route"),FPlanarFamilyTimeShortcutPlan::Build(lone,path,legacy,legacyPlan)==EPlanarFamilyBuild::NotApplicable&&!legacyPlan))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
#endif


// Append after CoordinateLineTimeHintDetail AND CoordinateLineSpatialDetail.
// This is pure proof output; caller must seal it to its original source/path,
// numeric control, shared account and consumer operation before retaining it.
namespace Speed::Analytic::Certified::TimeDetail
{
static EMonotoneCurveTimeSeed TryCoordinateLineContactCertificate(
    TConstArrayView<FIntervalTensorSurface> surfaces,const FIntervalBezierCenterPath& path,
    double radius,FDistanceBudget& budget,double requestedGap,
    Interval& rootTime,Interval& rootV,int32& normalAxis,int32& side,
    double& clearPrefix,double& insideTime,FIntervalPoint* capturedRootCenter=nullptr,
    bool rootLocalCoverage=false,int32* refusalReason=nullptr)
{
    namespace A=::AnalyticCertified;using E=EMonotoneCurveTimeSeed;
    rootTime={};rootV={};normalAxis=INDEX_NONE;side=0;clearPrefix=0;insideTime=1;
    if(capturedRootCenter)*capturedRootCenter={};if(refusalReason)*refusalReason=1;
    const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=EAnalyticNumericStatus::Complete)return E::Arithmetic;
    if(budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor)return E::Work;
    if(int64(budget.Remaining)-budget.ConsumptionFloor<128||budget.Used>MAX_uint64-128)return E::NotApplicable;
    FScopedDistanceBudgetFloor limit(budget,budget.Remaining-128);if(!limit.IsValid())return E::Work;
    CoordinateLineTimeHintDetail::FRootCertificate time;
    double lower=0,upper=1;
    // Existing proof keeps whole-path coverage by default. The optional
    // union route instead proves finite-image coverage on RootTime/InsideB;
    // the support-line distance lower bound excludes every earlier cap.
    // Both contracts prove the quotient is the first entry for every realization.
    const auto status=CoordinateLineTimeHintDetail::Try(surfaces,path,radius,budget,
        requestedGap,lower,upper,false,&time,rootLocalCoverage,refusalReason);
    if(status!=E::Proved)return status;
    if(!A::Valid(time.Time)||!(time.Time.Lower>0&&time.Time.Upper<1)||
        time.NormalAxis<0||time.NormalAxis>2||time.TangentAxis<0||time.TangentAxis>2||
        time.NormalAxis==time.TangentAxis||(time.Side!=1&&time.Side!=-1))return E::Arithmetic;
    if(!budget.Consume())return E::Work;
    // Evaluate ONLY the certified ROOT interval, not padded clearA/insideB.
    // For every true contact t*, the original center lies in this enclosure.
    const auto centers=CenterDetail::Evaluate(path,time.Time);
    if(!Detail::ValidPoint(centers))return E::NotApplicable;
    if(refusalReason)*refusalReason=10;
    Interval necessary;
    const auto domain=CoordinateLineSpatialDetail::Domain(surfaces,centers,budget,necessary);
    if(domain==EDirectCurveSpatialSeed::Work)return E::Work;
    if(domain==EDirectCurveSpatialSeed::Arithmetic)return E::Arithmetic;
    if(domain!=EDirectCurveSpatialSeed::Proved)return E::NotApplicable;
    if(!budget.Consume())return E::Work;
    const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=numeric.ControlFingerprint())return E::Arithmetic;
    if(!A::Valid(necessary)||necessary.Lower<0||necessary.Upper>1)return E::Arithmetic;
    // Domain uses original interval coefficients and MVT, never a chord UV.
    // Thus ALL first-contact parameters are covered, including each different
    // fixed CP realization represented by independent coefficient boxes.
    rootTime=time.Time;rootV=necessary;normalAxis=time.NormalAxis;side=time.Side;
    clearPrefix=time.ClearPrefix;insideTime=time.InsideTime;
    if(capturedRootCenter)*capturedRootCenter=centers;if(refusalReason)*refusalReason=0;return E::Proved;
}
}

namespace
{
TAutoConsoleVariable<int32> CVarDirectCurveValueTransport(
    TEXT("p.IAmSpeed.AnalyticWorld.DirectCurveValueTransport"),0,
    TEXT("Default OFF. Reuse already evaluated original root values only inside one direct-contact preparation; no callback or inter-query cache, unchanged logical work."),ECVF_Default);
TAutoConsoleVariable<int32> CVarDirectCoordinateLineContact(
    TEXT("p.IAmSpeed.AnalyticWorld.DirectCoordinateLineContact"),0,
    TEXT("Default OFF. Complete exact first-contact certificate for a finite coordinate-line image; original UV/lift/filter remain mandatory."),ECVF_Default);
TAutoConsoleVariable<int32> CVarDirectOffsetCurveContact(
    TEXT("p.IAmSpeed.AnalyticWorld.DirectOffsetCurveContact"),0,
    TEXT("Default OFF. Complete exact first-contact certificate from the original regular offset curve; no chord contact or approximate UV."),ECVF_Default);
}
namespace { TAutoConsoleVariable<int32> CVarDirectCoplanarFamilyContact(
    TEXT("p.IAmSpeed.AnalyticWorld.DirectCoplanarFamilyContact"),0,
    TEXT("Default OFF. Complete exact first-contact quotient for a wholly covered finite coplanar candidate union; requires TensorPlanarFamily for high-degree UV domains."),ECVF_Default); }
namespace { TAutoConsoleVariable<int32> CVarDirectCurveUnionContact(
    TEXT("p.IAmSpeed.AnalyticWorld.DirectCurveUnionContact"),0,
    TEXT("Default OFF. Complete exact two-provider first-contact union; every provider is a certified first root or whole-path miss. Unknown falls back before publication."),ECVF_Default); }
namespace Speed::Analytic::Certified::ImmutablePlanarFaceRank
{
namespace A=::AnalyticCertified;
namespace M=FixedWheelProbeMath;
namespace Properties=ContactDetail::TensorPlanarDetail;
TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.ImmutablePlanarFaceRank"),0,
    TEXT("Default OFF. Original global derivatives or exact bilinear Jacobian certify regular planar charts before position subdivision; pure bounded metadata only."),ECVF_Default);
struct FModel {FIntervalPoint Constant,U,V;uint8 Kind=0;};
struct FEntry {FIntervalTensorSurface Surface;FModel Model;uint64 Hash=0,Control=0;uint32 Sticky=0;bool Ready=false;};
struct FCache {FEntry Entries[64];uint64 Hits=0,Misses=0,StickyMisses=0;};
static FCache& Cache(){static thread_local FCache value;return value;}
struct FCounters {std::atomic<uint64> Calls{0},Hits{0},Misses{0},Bilinear{0},Global{0},Fallback{0},Ticks{0};};
static FCounters Counts;
static bool Pay(FDistanceBudget& budget,int32 count)
{
    if(count<0||budget.ConsumptionFloor<0||int64(budget.Remaining)-budget.ConsumptionFloor<count||budget.Used>MAX_uint64-uint64(count))return false;
    for(int32 i=0;i<count;++i)if(!budget.Consume())return false;return true;
}
static bool Cross(const FIntervalPoint& a,const FIntervalPoint& b,FIntervalPoint& result)
{
    result={};for(int32 d=0;d<3;++d)
    {
        result.Axis[d]=A::Subtract(A::Multiply(a.Axis[(d+1)%3],b.Axis[(d+2)%3]),A::Multiply(a.Axis[(d+2)%3],b.Axis[(d+1)%3]));
        if(!A::Valid(result.Axis[d]))return false;
    }
    return true;
}
static bool Nonzero(const FIntervalPoint& value)
{if(!Detail::ValidPoint(value))return false;for(const auto& axis:value.Axis)if(axis.Lower>0||axis.Upper<0)return true;return false;}
static FModel Build(const FIntervalTensorSurface& surface)
{
    // Only exact coordinate planes are in this optimization's contract.
    bool planar=false;
    for(int32 d=0;d<3;++d)
    {
        const auto& first=surface.ControlPoints[0].Axis[d];bool same=A::Bits(first.Lower)==A::Bits(first.Upper);
        for(const auto& cp:surface.ControlPoints)same&=A::Bits(cp.Axis[d].Lower)==A::Bits(first.Lower)&&A::Bits(cp.Axis[d].Upper)==A::Bits(first.Upper);
        planar|=same;
    }
    FModel model;if(!planar)return model;
    if(surface.DegreeU==1)
    {
        // S=A+uE+vF+uvG. The SAME fixed G satisfies G cross G=0
        // for every independent realization of the four input boxes.
        FIntervalPoint e,f,g;
        for(int32 d=0;d<3;++d)
        {
            const auto& a=surface.ControlPoints[0].Axis[d];
            e.Axis[d]=A::Subtract(surface.ControlPoints[2].Axis[d],a);
            f.Axis[d]=A::Subtract(surface.ControlPoints[1].Axis[d],a);
            g.Axis[d]=A::Add(A::Subtract(A::Subtract(a,surface.ControlPoints[1].Axis[d]),surface.ControlPoints[2].Axis[d]),surface.ControlPoints[3].Axis[d]);
        }
        if(!Detail::ValidPoint(e)||!Detail::ValidPoint(f)||!Detail::ValidPoint(g)||
            !Cross(e,f,model.Constant)||!Cross(e,g,model.U)||!Cross(g,f,model.V))return FModel{};
        model.Kind=1;return model;
    }
    Detail::FNode original;original.DegreeU=surface.DegreeU;original.DegreeV=surface.DegreeV;original.Net=surface.ControlPoints;
    FIntervalPoint du,dv;
    if(!ContactDetail::DerivativeHull(original,true,du)||!ContactDetail::DerivativeHull(original,false,dv)||
        !Cross(du,dv,model.Constant)||!Nonzero(model.Constant))return FModel{};
    model.Kind=5;return model;
}
static FModel Model(const FIntervalTensorSurface& surface,uint64 control,bool observe)
{
    // Reuse the existing exact CP identity/hash implementation. Shape is also
    // checked by Same; the final discriminator prevents cross-family hashing.
    const uint64 hash=Properties::Hash(surface,control,surface.DegreeU);
    auto& cache=Cache();auto& entry=cache.Entries[hash&63];
    const bool same=entry.Ready&&entry.Hash==hash&&entry.Control==control&&Properties::Same(entry.Surface,surface);
    if(same&&(VectorGetControlRegister()&entry.Sticky)==entry.Sticky)
    {++cache.Hits;if(observe)++Counts.Hits;return entry.Model;}
    if(same)++cache.StickyMisses;
    ++cache.Misses;if(observe)++Counts.Misses;
    const auto value=Build(surface);const auto after=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    entry.Ready=false;entry.Model={};
    if(value.Kind&&after.ControlFingerprint()==control)
    {entry.Surface=surface;entry.Model=value;entry.Hash=hash;entry.Control=control;entry.Sticky=after.VectorControl&uint32(0x3f);entry.Ready=true;}
    return value;
}
static bool Try(const FIntervalTensorSurface& surface,const FTensorSphereContactLeaf& leaf,FDistanceBudget& budget,FIntervalPoint& rawNormal)
{
    rawNormal={};const bool observe=StrongConvexCurveMinimum::Observation::Enabled();
    const uint64 start=observe?FPlatformTime::Cycles64():0;if(observe)++Counts.Calls;
    struct FDone {bool Active;uint64 Start;~FDone(){if(Active)Counts.Ticks.fetch_add(FPlatformTime::Cycles64()-Start);}}done{observe,start};
    const auto before=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    const bool shape=(surface.DegreeU==1&&surface.DegreeV==1&&surface.ControlPoints.Num()==4)||
        (surface.DegreeU==5&&surface.DegreeV==5&&surface.ControlPoints.Num()==36);
    FReferenceDyadicCell u,v;
    if(!shape||!M::DecodeLeafDyadic(leaf.MinU,leaf.MaxU,u)||!M::DecodeLeafDyadic(leaf.MinV,leaf.MaxV,v)||
        !(leaf.MinU>0&&leaf.MaxU<1&&leaf.MinV>0&&leaf.MaxV<1)||!Pay(budget,1))return false;
    for(const auto& cp:surface.ControlPoints)if(!Pay(budget,1)||!Detail::ValidPoint(cp))return false;
    // Same logical debit on a property-cache hit or build. The local attempt
    // has at most117 debits; no computation or failure is refunded.
    if(!Pay(budget,surface.DegreeU==1?24:64))return false;
    const FModel model=Model(surface,before.ControlFingerprint(),observe);if(!model.Kind)return false;
    if(!Pay(budget,16))return false;
    FIntervalPoint result=model.Constant;
    if(model.Kind==1)for(int32 d=0;d<3;++d)
        result.Axis[d]=A::Add(A::Add(model.Constant.Axis[d],A::Multiply(u.GetRange(),model.U.Axis[d])),A::Multiply(v.GetRange(),model.V.Axis[d]));
    // Match the old chart's local derivative scale, not just its orientation.
    // Cell widths are exact positive powers of two, depth<=52.
    const auto scale=A::Multiply(Interval::Exact(A::FromBits(uint64(1023-u.Depth)<<52)),
        Interval::Exact(A::FromBits(uint64(1023-v.Depth)<<52)));
    for(int32 d=0;d<3;++d)result.Axis[d]=A::Multiply(result.Axis[d],scale);
    if(!Nonzero(result))return false;
    const auto after=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(after.ControlFingerprint()!=before.ControlFingerprint())return false;
    if(observe){if(model.Kind==1)++Counts.Bilinear;else ++Counts.Global;}
    rawNormal=result;return true;
}
static bool TryProtected(const FIntervalTensorSurface& surface,const FTensorSphereContactLeaf& leaf,
    FDistanceBudget& budget,FIntervalPoint& normal)
{
    normal={};if(Enabled.GetValueOnAnyThread()!=1)return false;
    FReferenceDyadicCell u,v;
    if(!M::DecodeLeafDyadic(leaf.MinU,leaf.MaxU,u)||!M::DecodeLeafDyadic(leaf.MinV,leaf.MaxV,v))return false;
    // Reserve every remaining debit of the unchanged fallback: Restrict's
    // input+CP+splits, both derivative hulls, and three cross components.
    const int64 fallback=1+int64(surface.ControlPoints.Num())+u.Depth+v.Depth+
        int64(surface.DegreeU)*(surface.DegreeV+1)+int64(surface.DegreeU+1)*surface.DegreeV+3;
    if(fallback<0||budget.ConsumptionFloor<0||int64(budget.Remaining)-budget.ConsumptionFloor<128+fallback)return false;
    FScopedDistanceBudgetFloor allowance(budget,budget.Remaining-128);if(!allowance.IsValid())return false;
    return Try(surface,leaf,budget,normal);
}
static void Publish()
{
    UE_LOG(LogTemp,Display,TEXT("[ImmutablePlanarFaceRank] Scope=ProgramPrefixAtPreExit NoPASS=1 Capacity=64 DefaultOFF=1 Calls=%llu Hits=%llu Misses=%llu Bilinear=%llu Global=%llu Fallback=%llu Ticks=%llu SecondsPerCycle=%.17g"),
        Counts.Calls.load(),Counts.Hits.load(),Counts.Misses.load(),Counts.Bilinear.load(),Counts.Global.load(),Counts.Fallback.load(),Counts.Ticks.load(),FPlatformTime::GetSecondsPerCycle64());
}
struct FRegister {FRegister(){FCoreDelegates::OnPreExit.AddStatic(&Publish);}};static FRegister Register;
}
namespace Speed::Analytic::Certified
{
IAMSPEED_API bool ProveRegularFaceLeafWithImmutablePlanarRank(FIndexedIntervalUnionPtr scene,
    const FTensorSphereContactLeaf& input,FDistanceBudget& budget,FIntervalPoint& rawNormal)
{
    namespace Q=ImmutablePlanarFaceRank;namespace M=FixedWheelProbeMath;namespace A=::AnalyticCertified;
    if(Q::Enabled.GetValueOnAnyThread()!=1)return M::ProveRegularFaceLeaf(scene,input,budget,rawNormal);
    // Literal original consumer proof, with one optional attempt AFTER the
    // original surface has been copied and all scene/leaf checks are fresh.
    const auto leaf=input;rawNormal={};
    const auto before=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!scene||!budget.Consume()||leaf.SurfaceIndex<0||leaf.SurfaceIndex>=scene->Num())return false;
    FReferenceDyadicCell u,v;
    if(!M::DecodeLeafDyadic(leaf.MinU,leaf.MaxU,u)||!M::DecodeLeafDyadic(leaf.MinV,leaf.MaxV,v)||
        !(leaf.MinU>0&&leaf.MaxU<1&&leaf.MinV>0&&leaf.MaxV<1))return false;
    FIntervalTensorSurface original,restricted;
    if(!scene->CopySurfaceForQuery(leaf.SurfaceIndex,budget,original))return false;
    if(Q::TryProtected(original,leaf,budget,rawNormal))return true;
    if(StrongConvexCurveMinimum::Observation::Enabled())++Q::Counts.Fallback;
    if(!RestrictReferenceTensorDyadic(original,u,v,budget,restricted)||restricted.DegreeU<1||restricted.DegreeV<1)return false;
    Detail::FNode node;node.DegreeU=restricted.DegreeU;node.DegreeV=restricted.DegreeV;node.Net=MoveTemp(restricted.ControlPoints);
    const int32 work=node.DegreeU*(node.DegreeV+1)+(node.DegreeU+1)*node.DegreeV;
    for(int32 i=0;i<work;++i)if(!budget.Consume())return false;
    FIntervalPoint du,dv,cross;
    if(!ContactDetail::DerivativeHull(node,true,du)||!ContactDetail::DerivativeHull(node,false,dv))return false;
    bool nonzero=false;
    for(int32 d=0;d<3;++d)
    {
        if(!budget.Consume())return false;
        cross.Axis[d]=A::Subtract(A::Multiply(du.Axis[(d+1)%3],dv.Axis[(d+2)%3]),A::Multiply(du.Axis[(d+2)%3],dv.Axis[(d+1)%3]));
        if(!A::Valid(cross.Axis[d]))return false;nonzero|=cross.Axis[d].Lower>0||cross.Axis[d].Upper<0;
    }
    if(!nonzero)return false;
    const auto after=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(before.ControlFingerprint()!=after.ControlFingerprint())return false;
    rawNormal=MoveTemp(cross);return true;
}
}

namespace Speed::Analytic::Certified::DirectCurveContact
{
namespace A=::AnalyticCertified;
namespace T=TimeDetail;
namespace S=StrongConvexCurveMinimum;
using E=T::EMonotoneCurveTimeSeed;
bool RuntimeEnabled()
{return CVarDirectCoordinateLineContact.GetValueOnAnyThread()==1||CVarDirectOffsetCurveContact.GetValueOnAnyThread()==1||CVarDirectCoplanarFamilyContact.GetValueOnAnyThread()==1||CVarDirectCurveUnionContact.GetValueOnAnyThread()==1;}
struct FObservation
{
    std::atomic<uint64> Calls{0},LineProved{0},OffsetProved{0},CoplanarProved{0},Accepted{0},RefineFallback{0},NotApplicable{0},Failures{0},Used{0},Ticks{0};
    std::atomic<uint64> RootCentersReused{0},RootPointsReused{0};
};
static FObservation Counts;
static bool Pay(FDistanceBudget& budget,int32 count=1)
{
    if(count<0||budget.ConsumptionFloor<0||int64(budget.Remaining)-budget.ConsumptionFloor<count||budget.Used>MAX_uint64-uint64(count))return false;
    for(int32 i=0;i<count;++i)if(!budget.Consume())return false;return true;
}
static bool WidthFits(Interval x,double maximum)
{
    if(!A::Valid(x)||!T::NonnegativeFinite(maximum))return false;
    const auto width=A::Subtract(Interval::Exact(x.Upper),Interval::Exact(x.Lower));
    return A::Valid(width)&&width.Upper<=maximum;
}
static bool IntersectPoint(const FIntervalPoint& first,const FIntervalPoint& second,FIntervalPoint& out)
{
    if(!Detail::ValidPoint(first)||!Detail::ValidPoint(second))return false;
    for(int32 d=0;d<3;++d)
    {out.Axis[d]={FMath::Max(first.Axis[d].Lower,second.Axis[d].Lower),FMath::Min(first.Axis[d].Upper,second.Axis[d].Upper),true};
     if(!A::Valid(out.Axis[d]))return false;}
    return true;
}
// Both inputs enclose EVERY first point of the SAME complete provider union.
// A generic leaf may be a false-positive candidate; dropping it is justified
// only by disjointness from the other universal first-point/normal enclosure.
// Every retained leaf keeps its actual source/UV and original fixed sample.
static bool IntersectContact(const FTensorSphereContactEnclosure& generic,
    const FTensorSphereContactEnclosure& direct,FTensorSphereContactEnclosure& out,FDistanceBudget& budget,
    TConstArrayView<uint8> providerDisposition={})
{
    if(generic.Status!=ETensorSphereContactStatus::FirstContactEnclosed||direct.Status!=ETensorSphereContactStatus::FirstContactEnclosed||
        generic.CollisionRadiusBits!=direct.CollisionRadiusBits||generic.Leaves.IsEmpty()||direct.Leaves.IsEmpty())return false;
    if(!Pay(budget))return false;
    const Interval time{FMath::Max(generic.LowerTime,direct.LowerTime),FMath::Min(generic.UpperTime,direct.UpperTime),true};
    if(!A::Valid(time))return false;
    out=generic;out.LowerTime=time.Lower;out.UpperTime=time.Upper;out.Leaves.Reset();bool have=false;
    for(const auto& original:generic.Leaves)
    {
        if(!Pay(budget))return false;
        if(!Detail::ValidPoint(original.PointHull)||!Detail::ValidPoint(original.RadialNormalHull)||
            !Detail::ValidPoint(original.FixedParameterPoint))return false;
        if(providerDisposition.IsEmpty())
        {if(original.SurfaceIndex!=0)return false;}
        else
        {
            if(original.SurfaceIndex<0||original.SurfaceIndex>=providerDisposition.Num()||providerDisposition[original.SurfaceIndex]>1)return false;
            // This complete, operation-local disposition proves absence at EVERY
            // union first event (RootFoot, whole miss, or strictly later root).
            // A mere omission from direct.Leaves would NOT authorize this skip.
            if(providerDisposition[original.SurfaceIndex]==0)continue;
        }
        FIntervalPoint point,normal;
        if(!IntersectPoint(original.PointHull,direct.SurfacePoint,point)||!IntersectPoint(original.RadialNormalHull,direct.RadialUnitNormal,normal))continue;
        auto leaf=original;leaf.PointHull=point;leaf.RadialNormalHull=normal;out.Leaves.Add(MoveTemp(leaf));
        if(!have){out.SurfacePoint=point;out.RadialUnitNormal=normal;have=true;}
        else {out.SurfacePoint=T::Hull(out.SurfacePoint,point);out.RadialUnitNormal=T::Hull(out.RadialUnitNormal,normal);}
    }
    return have&&Detail::ValidPoint(out.SurfacePoint)&&Detail::ValidPoint(out.RadialUnitNormal);
}
// No consumer runs under this optional proof's local floor. On NotApplicable
// all work remains paid and the original generic call is still untouched.
static E Prepare(TConstArrayView<FIntervalTensorSurface> surfaces,const FIntervalBezierCenterPath& path,
    double radius,FDistanceBudget& budget,double timeGap,double pointWidth,double normalWidth,
    bool lineEnabled,bool offsetEnabled,FTensorSphereContactEnclosure& out,bool& usedLine,
    bool reuseRootValues=CVarDirectCurveValueTransport.GetValueOnAnyThread()==1,
    bool rootLocalLine=false,int32* lineRefusalReason=nullptr)
{
    usedLine=false;if(lineRefusalReason)*lineRefusalReason=1;
    if(surfaces.Num()!=1||surfaces[0].DegreeU!=0||surfaces[0].DegreeV!=5||surfaces[0].ControlPoints.Num()!=6||
        path.Degree!=1||path.ControlPoints.Num()!=2)return E::NotApplicable;
    if(budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor)return E::Work;
    if(int64(budget.Remaining)-budget.ConsumptionFloor<448||budget.Used>MAX_uint64-448)return E::NotApplicable;
    FScopedDistanceBudgetFloor local(budget,budget.Remaining-448);if(!local.IsValid())return E::Work;
    const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=EAnalyticNumericStatus::Complete)return E::Arithmetic;
    Interval rootTime,rootV;double clear=0,inside=1;int32 normalAxis=INDEX_NONE,side=0;
    FIntervalPoint provedRootCenter,provedRootPoint;
    E status=E::NotApplicable;const uint64 timeBefore=budget.Used;
    {
        ContactKernelProfile72::FScope timing(ContactKernelProfile72::TimeSearch);
        if(lineEnabled)
        {
            status=T::TryCoordinateLineContactCertificate(surfaces,path,radius,budget,timeGap,rootTime,rootV,normalAxis,side,clear,inside,
                reuseRootValues?&provedRootCenter:nullptr,rootLocalLine,lineRefusalReason);
            usedLine=status==E::Proved;
        }
        if(status==E::NotApplicable&&offsetEnabled)
        {
            T::OffsetSpatialDomainDetail::FDraft proof;
            status=T::OffsetCurveTimeHintDetail::Try(surfaces,path,radius,budget,timeGap,clear,inside,false,&proof,
                reuseRootValues?&provedRootPoint:nullptr);
            if(status==E::Proved)
            {
                if(!proof.Ready)return E::Arithmetic;
                rootTime=proof.RootTime;rootV=proof.Parameter;
            }
        }
    }
    out.TimeWorkUsed=budget.Used-timeBefore;
    if(status!=E::Proved)return status;
    // clear/inside prove existence; RootTime itself encloses the FIRST event.
    // Its upper endpoint is not asserted to be a new strict-inside witness.
    if(!A::Valid(rootTime)||!A::Valid(rootV)||rootTime.Lower<0||rootTime.Upper>1||rootV.Lower<0||rootV.Upper>1||
        !(clear<rootTime.Lower&&inside>rootTime.Upper&&inside<=1))return E::Arithmetic;
    if(!WidthFits(rootTime,timeGap))return E::NotApplicable;
    ContactKernelProfile72::FScope spatial(ContactKernelProfile72::SpatialPass);
    if(!Pay(budget,4))return E::Work;
    FIntervalPoint point,normal;
    // Both values were evaluated from these identical source/path/root bits
    // in the pure proof above. No callback or mutable source operation occurs
    // before this use. Keep Pay(4), all validity checks and the final context
    // check; skipped duplicate arithmetic could only repeat existing sticky flags.
    const auto center=reuseRootValues&&usedLine?provedRootCenter:CenterDetail::Evaluate(path,rootTime);
    if(!Detail::ValidPoint(center))return E::NotApplicable;
    if(reuseRootValues&&usedLine&&S::Observation::Enabled())++Counts.RootCentersReused;
    if(!usedLine)
    {
        if(reuseRootValues)
        {
            point=provedRootPoint;if(!Detail::ValidPoint(point))return E::NotApplicable;
            if(S::Observation::Enabled())++Counts.RootPointsReused;
        }
        else
        {
            S::FCurve curve;for(const auto& cp:surfaces[0].ControlPoints){if(!Detail::ValidPoint(cp))return E::Arithmetic;curve.Add(cp);}
            if(!T::OffsetCurveTimeHintDetail::EvaluateRange(curve,rootV,point))return E::NotApplicable;
        }
        if(!ContactDetail::RadialNormal(center,point,radius,normal))return E::NotApplicable;
    }
    // The line theorem gives these coordinates exactly even for nonlinear
    // parameterization and independent tangential coefficient boxes.
    if(usedLine)
    {
        if(normalAxis<0||normalAxis>2||(side!=1&&side!=-1))return E::Arithmetic;
        for(int32 d=0;d<3;++d)
        {
            normal.Axis[d]=Interval::Exact(d==normalAxis?double(side):0.);
            if(d!=normalAxis)point.Axis[d]=center.Axis[d];
            else point.Axis[d]=surfaces[0].ControlPoints[0].Axis[d];
        }
    }
    const auto pointTarget=A::Multiply(Interval::Exact(pointWidth),Interval::Exact(.25));
    const auto normalTarget=A::Multiply(Interval::Exact(normalWidth),Interval::Exact(.25));
    if(!A::Valid(pointTarget)||!A::Valid(normalTarget)||!ContactDetail::Fits(point,FMath::Max(0.,pointTarget.Lower))||
        !ContactDetail::Fits(normal,FMath::Max(0.,normalTarget.Lower)))return E::NotApplicable;
    const auto width=A::Subtract(Interval::Exact(rootV.Upper),Interval::Exact(rootV.Lower));
    if(!A::Valid(width))return E::Arithmetic;
    int32 depth=0;double step=1.;while(depth<48&&step*.5>=width.Upper){step*=.5;++depth;}
    if(depth<2)return E::NotApplicable;
    const uint64 count=uint64(1)<<depth;
    // Scaling by an exact power of two is exact here: V is in[0,1], count<=2^48.
    const uint64 first=FMath::Min(uint64(rootV.Lower*double(count)),count-1);
    const uint64 last=FMath::Min(uint64(rootV.Upper*double(count)),count-1);
    if(last<first||last-first>1)return E::NotApplicable;
    // A coarse outward inverse near a true finite endpoint must retain the
    // old refinement route; do not provoke a boundary-feature refusal merely
    // because this optional direct cell cover stopped at depth48.
    if(first==0||last==count-1)return E::NotApplicable;
    TArray<FTensorSphereContactLeaf> leaves;
    for(uint64 index=first;index<=last;++index)
    {
        if(!Pay(budget,2))return E::Work;
        FTensorSphereContactLeaf leaf;leaf.SurfaceIndex=0;leaf.MinU=0;leaf.MaxU=1;
        leaf.MinV=double(index)*step;leaf.MaxV=double(index+1)*step;
        leaf.FixedParameter=FVector2d(.5,(leaf.MinV+leaf.MaxV)*.5);
        if(!ProjectedDistanceDetail::EvaluateFixedUV(surfaces[0],leaf.FixedParameter,leaf.FixedParameterPoint))return E::NotApplicable;
        // The common box may include extra points from the other UV leaf.
        // FixedParameterPoint remains an ORIGINAL representative, never a root.
        leaf.PointHull=point;leaf.RadialNormalHull=normal;leaves.Add(MoveTemp(leaf));
    }
    if(!Pay(budget))return E::Work;
    const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=numeric.ControlFingerprint())return E::Arithmetic;
    out.LowerTime=rootTime.Lower;out.UpperTime=rootTime.Upper;out.SurfacePoint=point;out.RadialUnitNormal=normal;
    out.CollisionRadiusBits=A::Bits(radius);out.Leaves=MoveTemp(leaves);out.SpatialVisitedCells=uint64(out.Leaves.Num());
    out.Status=ETensorSphereContactStatus::FirstContactEnclosed;out.WorkSite=usedLine?TEXT("direct_coordinate_line_contact"):TEXT("direct_offset_curve_contact");
    return E::Proved;
}

// Complete two-provider first-event composition. All source/path intervals
// remain in the caller's original common coordinate frame.
namespace UnionDetail
{
constexpr int32 ProviderCount=2,Allowance=1152;
struct FCounts
{
    std::atomic<uint64> Attempts{0},Proved{0},AllMiss{0},MissProviders{0},RootProviders{0},
        RetainedProviders{0},LaterProviders{0},Unavailable{0},Ticks{0};
    std::atomic<uint64> RootLocalLineProved{0},Pending{0},NoRoot{0},PrefixAttempts{0},PrefixMiss{0},PrefixOverlap{0},TargetsRefused{0};
    std::atomic<uint64> LineRefusal[11]{};
};
static FCounts Counts;
static E WholeMiss(const FIntervalTensorSurface& surface,const FIntervalPoint& pathHull,
    Interval radiusSquared,FDistanceBudget& budget,bool& miss)
{
    miss=false;FIntervalPoint hull;
    if(!Pay(budget))return E::Work;
    if(surface.DegreeU<0||surface.DegreeV<0||surface.DegreeU>5||surface.DegreeV>5||
        surface.ControlPoints.Num()!=(surface.DegreeU+1)*(surface.DegreeV+1))return E::NotApplicable;
    for(const auto& point:surface.ControlPoints)
    {
        if(!Pay(budget))return E::Work;
        if(!Detail::ValidPoint(point))return E::Arithmetic;
        hull=Detail::ValidPoint(hull)?T::Hull(hull,point):point;
    }
    if(!Detail::ValidPoint(hull)||!Detail::ValidPoint(pathHull)||!A::Valid(radiusSquared))return E::Arithmetic;
    Interval lower=Interval::Exact(0.);
    for(int32 d=0;d<3;++d)
    {
        if(!Pay(budget))return E::Work;
        Interval gap=Interval::Exact(0.);
        if(hull.Axis[d].Upper<pathHull.Axis[d].Lower)
            gap=A::Subtract(Interval::Exact(pathHull.Axis[d].Lower),Interval::Exact(hull.Axis[d].Upper));
        else if(pathHull.Axis[d].Upper<hull.Axis[d].Lower)
            gap=A::Subtract(Interval::Exact(hull.Axis[d].Lower),Interval::Exact(pathHull.Axis[d].Upper));
        if(!A::Valid(gap))return E::NotApplicable;
        const double minimum=FMath::Max(0.,gap.Lower);
        lower=A::Add(lower,A::Square(Interval::Exact(minimum)));
        if(!A::Valid(lower))return E::NotApplicable;
    }
    miss=lower.Lower>radiusSquared.Upper;return E::Proved;
}
static E PrefixMiss(const FIntervalTensorSurface& surface,const FIntervalBezierCenterPath& path,
    double upper,Interval radiusSquared,FDistanceBudget& budget,bool& miss)
{
    miss=false;if(!Pay(budget,2))return E::Work;
    if(path.Degree!=1||path.ControlPoints.Num()!=2||!CenterDetail::Valid(path)||
        !A::Finite(upper)||!(upper>=0&&upper<=1))return E::Arithmetic;
    const auto end=CenterDetail::Evaluate(path,Interval::Exact(upper));
    if(!Detail::ValidPoint(end))return E::NotApplicable;
    // For EVERY fixed affine path, its prefix is the segment from its own
    // original P0 to its own original P(upper). No midpoint path is used.
    const auto prefix=T::Hull(path.ControlPoints[0],end);
    return WholeMiss(surface,prefix,radiusSquared,budget,miss);
}
}
static E PrepareUnion(TConstArrayView<FIntervalTensorSurface> surfaces,const FIntervalBezierCenterPath& path,
    double radius,FDistanceBudget& budget,double timeGap,double pointWidth,double normalWidth,
    FTensorSphereContactEnclosure& out,TArray<uint8>& providerDisposition)
{
    out.Status=ETensorSphereContactStatus::Unknown;out.Leaves.Reset();providerDisposition.Reset();
    if(surfaces.Num()!=UnionDetail::ProviderCount||path.Degree!=1||path.ControlPoints.Num()!=2)return E::NotApplicable;
    if(budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor)return E::Work;
    if(int64(budget.Remaining)-budget.ConsumptionFloor<UnionDetail::Allowance||
        budget.Used>MAX_uint64-UnionDetail::Allowance)return E::NotApplicable;
    const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=EAnalyticNumericStatus::Complete)return E::Arithmetic;
    const bool observe=S::Observation::Enabled();const uint64 tick=observe?FPlatformTime::Cycles64():0;
    if(observe)++UnionDetail::Counts.Attempts;
    struct FElapsed{bool Active;uint64 Tick;~FElapsed(){if(Active)UnionDetail::Counts.Ticks.fetch_add(FPlatformTime::Cycles64()-Tick);}}elapsed{observe,tick};
    FScopedDistanceBudgetFloor local(budget,budget.Remaining-UnionDetail::Allowance);if(!local.IsValid())return E::Work;
    const auto unavailable=[&]()
    {
        const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=numeric.ControlFingerprint())return E::Arithmetic;
        if(observe)++UnionDetail::Counts.Unavailable;return E::NotApplicable;
    };
    if(!CenterDetail::Valid(path)||!A::Finite(radius)||!(radius>0)||!T::NonnegativeFinite(timeGap)||
        !T::NonnegativeFinite(pointWidth)||!T::NonnegativeFinite(normalWidth))return E::Arithmetic;
    if(!(timeGap>0&&pointWidth>0&&normalWidth>0))return unavailable();
    if(!Pay(budget,2))return unavailable();
    const auto pathHull=CenterDetail::Hull(path);const auto radiusSquared=A::Square(Interval::Exact(radius));
    if(!Detail::ValidPoint(pathHull)||!A::Valid(radiusSquared))return unavailable();
    FTensorSphereContactEnclosure contacts[UnionDetail::ProviderCount];
    bool missed[UnionDetail::ProviderCount]{},rooted[UnionDetail::ProviderCount]{};
    bool anyRoot=false,allMiss=true;double lower=1.,upper=1.;
    for(int32 i=0;i<surfaces.Num();++i)
    {
        const uint64 missBefore=budget.Used;
        const auto status=UnionDetail::WholeMiss(surfaces[i],pathHull,radiusSquared,budget,missed[i]);
        out.TimeWorkUsed+=budget.Used-missBefore;
        if(status==E::Arithmetic||status==E::Work)return status;
        if(status!=E::Proved)return unavailable();
        if(missed[i]){if(observe)++UnionDetail::Counts.MissProviders;continue;}
        allMiss=false;bool line=false;int32 lineReason=0;
        const auto root=Prepare(MakeArrayView(&surfaces[i],1),path,radius,budget,timeGap,pointWidth,normalWidth,true,true,
            contacts[i],line,CVarDirectCurveValueTransport.GetValueOnAnyThread()==1,true,&lineReason);
        out.TimeWorkUsed+=contacts[i].TimeWorkUsed;
        if(observe)
        {
            if(line&&root==E::Proved)++UnionDetail::Counts.RootLocalLineProved;
            else if(lineReason>0&&lineReason<11)++UnionDetail::Counts.LineRefusal[lineReason];
        }
        if(root==E::Arithmetic||root==E::Work)return root;
        if(root!=E::Proved){if(observe)++UnionDetail::Counts.Pending;continue;}
        const auto& c=contacts[i];
        if(c.Status!=ETensorSphereContactStatus::FirstContactEnclosed||c.Leaves.IsEmpty()||
            !A::Valid(Interval{c.LowerTime,c.UpperTime,true})||c.LowerTime<0||c.UpperTime>1||
            c.CollisionRadiusBits!=A::Bits(radius))return E::Arithmetic;
        rooted[i]=true;lower=anyRoot?FMath::Min(lower,c.LowerTime):c.LowerTime;
        upper=anyRoot?FMath::Min(upper,c.UpperTime):c.UpperTime;anyRoot=true;
        if(observe)++UnionDetail::Counts.RootProviders;
    }
    TArray<uint8> disposition;disposition.SetNumZeroed(surfaces.Num());
    if(allMiss)
    {
        if(!Pay(budget))return unavailable();
        const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=numeric.ControlFingerprint())return E::Arithmetic;
        out.Status=ETensorSphereContactStatus::NoContactWithProvidedTensors;out.LowerTime=0.;out.UpperTime=1.;
        out.SurfacePoint={};out.RadialUnitNormal={};out.CollisionRadiusBits=A::Bits(radius);
        out.WorkSite=TEXT("direct_curve_union_whole_miss");providerDisposition=MoveTemp(disposition);
        if(observe)++UnionDetail::Counts.AllMiss;return E::Proved;
    }
    if(!anyRoot){if(observe)++UnionDetail::Counts.NoRoot;return unavailable();}
    // A complete known first root guarantees that the UNION enters by upper.
    // Every unresolved provider must now miss the ENTIRE ORIGINAL [0,upper],
    // inclusively. This proves its absence at every possible first union event.
    for(int32 i=0;i<surfaces.Num();++i)if(!missed[i]&&!rooted[i])
    {
        if(observe)++UnionDetail::Counts.PrefixAttempts;
        const uint64 prefixBefore=budget.Used;bool prefixMiss=false;
        const auto status=UnionDetail::PrefixMiss(surfaces[i],path,upper,radiusSquared,budget,prefixMiss);
        out.TimeWorkUsed+=budget.Used-prefixBefore;
        if(status==E::Arithmetic||status==E::Work)return status;
        if(status!=E::Proved||!prefixMiss){if(observe)++UnionDetail::Counts.PrefixOverlap;return unavailable();}
        missed[i]=true;if(observe)++UnionDetail::Counts.PrefixMiss;
    }
    const Interval first{lower,upper,true};if(!WidthFits(first,timeGap))return unavailable();
    TArray<FTensorSphereContactLeaf> leaves;FIntervalPoint point,normal;bool have=false;
    for(int32 i=0;i<surfaces.Num();++i)
    {
        if(!Pay(budget))return unavailable();if(missed[i])continue;
        const auto& c=contacts[i];if(!rooted[i])return E::Arithmetic;
        if(c.LowerTime>upper){if(observe)++UnionDetail::Counts.LaterProviders;continue;}
        disposition[i]=1;if(observe)++UnionDetail::Counts.RetainedProviders;
        for(const auto& original:c.Leaves)
        {
            if(!Pay(budget))return unavailable();
            if(original.SurfaceIndex!=0||!Detail::ValidPoint(original.PointHull)||
                !Detail::ValidPoint(original.RadialNormalHull)||!Detail::ValidPoint(original.FixedParameterPoint))return E::Arithmetic;
            auto leaf=original;leaf.SurfaceIndex=i;leaves.Add(MoveTemp(leaf));
        }
        if(!have){point=c.SurfacePoint;normal=c.RadialUnitNormal;have=true;}
        else {point=T::Hull(point,c.SurfacePoint);normal=T::Hull(normal,c.RadialUnitNormal);}
    }
    if(!have||leaves.IsEmpty())return E::Arithmetic;
    const auto pt=A::Multiply(Interval::Exact(pointWidth),Interval::Exact(.25));
    const auto nt=A::Multiply(Interval::Exact(normalWidth),Interval::Exact(.25));
    if(!A::Valid(pt)||!A::Valid(nt)||!ContactDetail::Fits(point,FMath::Max(0.,pt.Lower))||
        !ContactDetail::Fits(normal,FMath::Max(0.,nt.Lower)))
    {if(observe)++UnionDetail::Counts.TargetsRefused;return unavailable();}
    if(!Pay(budget))return unavailable();
    const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=numeric.ControlFingerprint())return E::Arithmetic;
    out.LowerTime=lower;out.UpperTime=upper;out.SurfacePoint=point;out.RadialUnitNormal=normal;
    out.CollisionRadiusBits=A::Bits(radius);out.Leaves=MoveTemp(leaves);out.SpatialVisitedCells=uint64(out.Leaves.Num());
    out.Status=ETensorSphereContactStatus::FirstContactEnclosed;out.WorkSite=TEXT("direct_curve_union_contact");
    providerDisposition=MoveTemp(disposition);if(observe)++UnionDetail::Counts.Proved;return E::Proved;
}

// Insert inside DirectCurveContact, after IntersectContact and before TryContact.
// Pure per-original-provider disposition: 0=proved no foot preimage,1=retained.
// A completed array is published only with a completed first-contact enclosure.
// Prepublication admission uses the SAME position-restriction-then-derivative
// chart arithmetic as the downstream native regular-face check. This does not
// replace that fresh check or make a consumer Refuse eligible for fallback.
static bool ProveCoplanarChartBeforePublication(const FIntervalTensorSurface& surface,
    const FTensorSphereContactLeaf& leaf,FDistanceBudget& budget)
{
    FReferenceDyadicCell u,v;
    if(!FixedWheelProbeMath::DecodeLeafDyadic(leaf.MinU,leaf.MaxU,u)||
        !FixedWheelProbeMath::DecodeLeafDyadic(leaf.MinV,leaf.MaxV,v))return false;
    FIntervalPoint immutableNormal;
    if(ImmutablePlanarFaceRank::TryProtected(surface,leaf,budget,immutableNormal))return true;
    FIntervalTensorSurface restricted;
    if(!RestrictReferenceTensorDyadic(surface,u,v,budget,restricted)||
        restricted.DegreeU<1||restricted.DegreeV<1)return false;
    Detail::FNode node;node.DegreeU=restricted.DegreeU;node.DegreeV=restricted.DegreeV;
    node.Net=MoveTemp(restricted.ControlPoints);
    const int32 work=node.DegreeU*(node.DegreeV+1)+(node.DegreeU+1)*node.DegreeV;
    for(int32 i=0;i<work;++i)if(!Pay(budget))return false;
    FIntervalPoint du,dv;
    if(!ContactDetail::DerivativeHull(node,true,du)||!ContactDetail::DerivativeHull(node,false,dv))return false;
    bool nonzero=false;
    for(int32 d=0;d<3;++d)
    {
        if(!Pay(budget))return false;
        const auto cross=A::Subtract(A::Multiply(du.Axis[(d+1)%3],dv.Axis[(d+2)%3]),
            A::Multiply(du.Axis[(d+2)%3],dv.Axis[(d+1)%3]));
        if(!A::Valid(cross))return false;
        nonzero|=cross.Lower>0||cross.Upper<0;
    }
    return nonzero;
}
static E PrepareCoplanar(TConstArrayView<FIntervalTensorSurface> surfaces,
    const FIntervalBezierCenterPath& path,double radius,FDistanceBudget& budget,
    double timeGap,double pointWidth,double normalWidth,FTensorSphereContactEnclosure& out,
    TArray<uint8>& providerDisposition)
{
    providerDisposition.Reset();out.Status=ETensorSphereContactStatus::Unknown;out.Leaves.Reset();
    out.SurfacePoint={};out.RadialUnitNormal={};out.LowerTime=0;out.UpperTime=1;
    if(surfaces.IsEmpty()||surfaces.Num()>64||path.Degree!=1||path.ControlPoints.Num()!=2)return E::NotApplicable;
    const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=EAnalyticNumericStatus::Complete)return E::Arithmetic;
    if(budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor)return E::Work;
    constexpr int32 Allowance=4096; // Optional local debit; no caller cap is enlarged.
    if(int64(budget.Remaining)-budget.ConsumptionFloor<Allowance||budget.Used>MAX_uint64-Allowance)return E::NotApplicable;
    FScopedDistanceBudgetFloor local(budget,budget.Remaining-Allowance);if(!local.IsValid())return E::Work;
    const auto unavailable=[&]()
    {
        const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=numeric.ControlFingerprint())return E::Arithmetic;
        return E::NotApplicable; // Paid optional local exhaustion/arithmetic bound; old route remains.
    };
    if(!CenterDetail::Valid(path)||!A::Finite(radius)||!(radius>0)||!T::NonnegativeFinite(timeGap))return E::Arithmetic;
    if(!(timeGap>0))return E::NotApplicable;
    for(const auto& surface:surfaces)
    {
        if(!Pay(budget))return unavailable();
        const bool triangle=surface.DegreeU==1&&surface.DegreeV==1&&surface.ControlPoints.Num()==4;
        const bool tensor=surface.DegreeU==5&&surface.DegreeV==5&&surface.ControlPoints.Num()==36;
        if(!triangle&&!tensor)return E::NotApplicable;
        for(const auto& cp:surface.ControlPoints)
        {if(!Pay(budget))return unavailable();if(!Detail::ValidPoint(cp))return E::Arithmetic;}
    }
    const uint64 timeBefore=budget.Used;
    struct FTimeSpent{FTensorSphereContactEnclosure& Out;FDistanceBudget& Budget;uint64 Before;bool Active=true;
        void Stop(){if(Active){Out.TimeWorkUsed+=Budget.Used-Before;Active=false;}}
        ~FTimeSpent(){Stop();}}timeSpent{out,budget,timeBefore};
    ContactKernelProfile72::FScope timeProfile(ContactKernelProfile72::TimeSearch);
    // ALL candidates enter this call, in original order. A second plane,
    // unrelated provider, missing finite coverage, or unsupported chart refuses.
    ContactDetail::FPlanarProjectionDomains whole;
    auto proof=ContactDetail::ProjectedPlanarDomains(surfaces,CenterDetail::Hull(path),budget,whole,true);
    if(proof!=ContactDetail::EPlanarProjectionProof::Proved)return unavailable();
    const int32 axis=whole.NormalAxis;
    if(axis<0||axis>2||whole.Providers.Num()!=surfaces.Num())return E::Arithmetic;
    const auto height=whole.Foot.Axis[axis];
    if(!A::Valid(height)||A::Bits(height.Lower)!=A::Bits(height.Upper))return E::Arithmetic;
    if(!Pay(budget,4))return unavailable();
    const auto n0=path.ControlPoints[0].Axis[axis],n1=path.ControlPoints[1].Axis[axis];
    const auto offset=A::Subtract(n0,height),delta=A::Subtract(n1,n0),r2=A::Square(Interval::Exact(radius));
    if(!A::Valid(offset)||!A::Valid(delta)||!A::Valid(r2))return unavailable();
    int32 side=0;
    if(offset.Lower>radius&&delta.Upper<0)side=1;
    else if(offset.Upper< -radius&&delta.Lower>0)side=-1;
    if(!side)return E::NotApplicable;
    auto numerator=A::Subtract(A::Add(height,Interval::Exact(side*radius)),n0),denominator=delta;
    if(side>0){numerator=A::Negate(numerator);denominator=A::Negate(denominator);}
    Interval root;
    if(!ContactDetail::ProjectionDividePositive(numerator,denominator,root)||!(root.Lower>0&&root.Upper<1)||!WidthFits(root,timeGap))return unavailable();
    // Whole finite coverage makes distance to the COMPLETE original union
    // exactly |normal(t)-height| for every time and each fixed CP realization.
    // The common signed linear crossing is therefore the true FIRST contact.
    bool inside=false;double increment=.25*timeGap;
    for(int32 attempt=0;attempt<8;++attempt)
    {
        if(!Pay(budget,2))return unavailable();
        if(!A::Finite(increment)||!(increment>0))return unavailable();
        volatile double proposal=root.Upper+increment;double b=proposal;
        if(!(b>root.Upper))b=A::FromBits(A::Bits(root.Upper)+1);
        b=FMath::Min(b,1.);if(!(b>root.Upper))return unavailable();
        const auto center=CenterDetail::Evaluate(path,Interval::Exact(b));
        if(!Detail::ValidPoint(center))return unavailable();
        const auto distance=A::Square(A::Subtract(center.Axis[axis],height));
        if(A::Valid(distance)&&distance.Upper<r2.Lower){inside=true;break;}
        increment*=2;
    }
    if(!inside)return E::NotApplicable;
    if(!Pay(budget))return unavailable();
    const auto centers=CenterDetail::Evaluate(path,root);
    if(!Detail::ValidPoint(centers))return unavailable();
    ContactDetail::FPlanarProjectionDomains atRoot;
    proof=ContactDetail::ProjectedPlanarDomains(surfaces,centers,budget,atRoot,true);
    if(proof!=ContactDetail::EPlanarProjectionProof::Proved)return unavailable();
    if(atRoot.NormalAxis!=axis||atRoot.Providers.Num()!=surfaces.Num())return E::Arithmetic;
    timeSpent.Stop();timeProfile.Stop();
    ContactKernelProfile72::FScope spatialProfile(ContactKernelProfile72::SpatialPass);
    FIntervalPoint normal;
    for(int32 d=0;d<3;++d)normal.Axis[d]=Interval::Exact(d==axis?double(side):0.);
    const auto pt=A::Multiply(Interval::Exact(pointWidth),Interval::Exact(.25));
    const auto nt=A::Multiply(Interval::Exact(normalWidth),Interval::Exact(.25));
    if(!A::Valid(pt)||!A::Valid(nt)||!ContactDetail::Fits(atRoot.Foot,FMath::Max(0.,pt.Lower))||
        !ContactDetail::Fits(normal,FMath::Max(0.,nt.Lower)))return unavailable();
    struct FCover{uint64 First=0,Last=0;double Step=1;};
    const auto cover=[&](Interval range,FCover& result)
    {
        if(!A::Valid(range)||range.Lower<0||range.Upper>1)return false;
        const auto width=A::Subtract(Interval::Exact(range.Upper),Interval::Exact(range.Lower));
        if(!A::Valid(width))return false;
        // RootUV can be far narrower than a numerically regular position chart.
        // The wider dyadic cell still contains EVERY true first parameter;
        // exact RootT/Foot/normal remain unchanged and authoritative.
        int32 depth=0;double step=1.;while(depth<32&&step*.5>=width.Upper){step*=.5;++depth;}
        if(depth<2)return false;const uint64 count=uint64(1)<<depth;
        const uint64 first=FMath::Min(uint64(range.Lower*double(count)),count-1);
        const uint64 last=FMath::Min(uint64(range.Upper*double(count)),count-1);
        if(last<first||last-first>1||first==0||last==count-1)return false;
        result={first,last,step};return true;
    };
    TArray<uint8> disposition;disposition.SetNumZeroed(surfaces.Num());
    TArray<FTensorSphereContactLeaf> leaves;
    for(int32 index=0;index<surfaces.Num();++index)
    {
        if(!Pay(budget))return unavailable();
        const auto& domain=atRoot.Providers[index];
        if(domain.Outside)continue; // Complete RootFoot domain proves this ID has no first point.
        if(!domain.Bounded)return E::NotApplicable; // Do not omit an unresolved original provider.
        FCover u,v;if(!cover(domain.U,u)||!cover(domain.V,v))return unavailable();
        const auto& surface=surfaces[index];
        const uint64 evaluation=uint64(surface.ControlPoints.Num())+
            uint64(surface.DegreeU+1)*uint64(surface.DegreeV)*uint64(surface.DegreeV+1)/2+
            uint64(surface.DegreeU)*uint64(surface.DegreeU+1)/2;
        for(uint64 ui=u.First;ui<=u.Last;++ui)for(uint64 vi=v.First;vi<=v.Last;++vi)
        {
            if(leaves.Num()>=64||evaluation>MAX_int32||!Pay(budget,int32(evaluation)+1))return unavailable();
            FTensorSphereContactLeaf leaf;leaf.SurfaceIndex=index;
            leaf.MinU=double(ui)*u.Step;leaf.MaxU=double(ui+1)*u.Step;
            leaf.MinV=double(vi)*v.Step;leaf.MaxV=double(vi+1)*v.Step;
            leaf.FixedParameter=FVector2d(.5*(leaf.MinU+leaf.MaxU),.5*(leaf.MinV+leaf.MaxV));
            if(!ProjectedDistanceDetail::EvaluateFixedUV(surface,leaf.FixedParameter,leaf.FixedParameterPoint))return unavailable();
            if(!ProveCoplanarChartBeforePublication(surface,leaf,budget))return unavailable();
            leaf.PointHull=atRoot.Foot;leaf.RadialNormalHull=normal;leaves.Add(MoveTemp(leaf));
        }
        disposition[index]=1;
    }
    if(leaves.IsEmpty())return E::Arithmetic;
    if(!Pay(budget))return unavailable();
    const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=numeric.ControlFingerprint())return E::Arithmetic;
    out.LowerTime=root.Lower;out.UpperTime=root.Upper;out.CollisionRadiusBits=A::Bits(radius);
    out.SurfacePoint=atRoot.Foot;out.RadialUnitNormal=normal;out.Leaves=MoveTemp(leaves);
    out.SpatialVisitedCells=uint64(out.Leaves.Num());out.Status=ETensorSphereContactStatus::FirstContactEnclosed;
    out.WorkSite=TEXT("direct_coplanar_family_contact");providerDisposition=MoveTemp(disposition);return E::Proved;
}

bool TryContact(TConstArrayView<FIntervalTensorSurface> surfaces,const FIntervalBezierCenterPath& path,double radius,
    FDistanceBudget& budget,double timeGap,double spatialGap,int32 spatialNodes,double pointWidth,double normalWidth,
    int32 maximumRefinements,const FContactEnclosureFilter* filter,
    ContactDetail::FContactCellGeometryMemo* geometryMemo,FTensorSphereContactEnclosure& result)
{
    const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    const uint64 radiusBits=A::Bits(radius);
    // Preserve old input refusals; dispatch cannot accept an invalid old input.
    if((radiusBits&A::Sign)||(radiusBits&A::Magnitude)<A::MinNormal||!A::Finite(radius)||
        !T::NonnegativeFinite(timeGap)||!T::NonnegativeFinite(spatialGap)||spatialNodes<=0||
        !T::NonnegativeFinite(pointWidth)||!T::NonnegativeFinite(normalWidth)||maximumRefinements<=0||maximumRefinements>24||
        !(timeGap>0&&pointWidth>0&&normalWidth>0))return false;
    const bool observe=S::Observation::Enabled();const uint64 tick=observe?FPlatformTime::Cycles64():0,before=budget.Used;
    if(observe)++Counts.Calls;
    struct FFinish{bool Enabled;uint64 Tick,Before;FDistanceBudget& Budget;
        ~FFinish(){if(Enabled){Counts.Ticks.fetch_add(FPlatformTime::Cycles64()-Tick);if(Budget.Used>=Before)Counts.Used.fetch_add(Budget.Used-Before);}}}finish{observe,tick,before,budget};
    result={};result.HasCallAccounting=true;result.CallInitialUsed=before;result.CollisionRadiusBits=radiusBits;
    const auto fail=[&](ETensorSphereContactFailure failure)
    {result.Status=ETensorSphereContactStatus::Unknown;result.Leaves.Reset();result.Failure=failure;if(observe)++Counts.Failures;return true;};
    bool usedLine=false,usedCoplanar=false,usedUnion=false;TArray<uint8> providerDisposition;
    E prepared=Prepare(surfaces,path,radius,budget,timeGap,pointWidth,normalWidth,
        CVarDirectCoordinateLineContact.GetValueOnAnyThread()==1,CVarDirectOffsetCurveContact.GetValueOnAnyThread()==1,result,usedLine);
    if(prepared==E::NotApplicable&&CVarDirectCurveUnionContact.GetValueOnAnyThread()==1)
    {
        prepared=PrepareUnion(surfaces,path,radius,budget,timeGap,pointWidth,normalWidth,result,providerDisposition);
        usedUnion=prepared==E::Proved;
    }
    if(prepared==E::NotApplicable&&CVarDirectCoplanarFamilyContact.GetValueOnAnyThread()==1)
    {
        prepared=PrepareCoplanar(surfaces,path,radius,budget,timeGap,pointWidth,normalWidth,result,providerDisposition);
        usedCoplanar=prepared==E::Proved;
    }
    if(prepared==E::NotApplicable){if(observe)++Counts.NotApplicable;return false;}
    if(prepared!=E::Proved)return fail(prepared==E::Work?ETensorSphereContactFailure::Work:ETensorSphereContactFailure::Arithmetic);
    if(usedUnion&&result.Status==ETensorSphereContactStatus::NoContactWithProvidedTensors)return true;
    if(observe&&!usedUnion){if(usedCoplanar)++Counts.CoplanarProved;else if(usedLine)++Counts.LineProved;else ++Counts.OffsetProved;}
    if(!filter){if(observe)++Counts.Accepted;return true;}
    const auto decision=(*filter)(result);
    if(RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=numeric.ControlFingerprint())
        return fail(ETensorSphereContactFailure::NumericContext);
    if(decision==EContactEnclosureDecision::Accept){if(observe)++Counts.Accepted;return true;}
    if(decision!=EContactEnclosureDecision::Refine)return fail(ETensorSphereContactFailure::ConsumerRefusal);
    if(maximumRefinements<=1)return fail(ETensorSphereContactFailure::WidthRefinementLimit);
    const double nextTime=timeGap*.25,nextPoint=pointWidth*.25,nextNormal=normalWidth*.25;
    if(!(nextTime>0&&nextTime<timeGap&&nextPoint>0&&nextPoint<pointWidth&&nextNormal>0&&nextNormal<normalWidth))
        return fail(ETensorSphereContactFailure::Arithmetic);
    if(!Pay(budget))return fail(ETensorSphereContactFailure::Work);
    if(observe)++Counts.RefineFallback;
    // No generic frontier existed yet. Create it ONCE with smaller targets
    // and allowance. Subsequent generic Refine retains that frontier normally.
    const auto direct=result;FTensorSphereContactEnclosure accepted;bool haveAccepted=false;
    bool bridgeFailed=false;ETensorSphereContactFailure bridgeFailure=ETensorSphereContactFailure::Arithmetic;
    FTensorSphereContactEnclosure lastPublished=direct;
    const auto continuedCallback=[&](const FTensorSphereContactEnclosure& generic)
    {
        FTensorSphereContactEnclosure intersection;
        if(!IntersectContact(generic,direct,intersection,budget,MakeArrayView(providerDisposition)))
        {bridgeFailed=true;bridgeFailure=budget.Remaining<=budget.ConsumptionFloor?ETensorSphereContactFailure::Work:ETensorSphereContactFailure::Arithmetic;return EContactEnclosureDecision::Refuse;}
        intersection.CallInitialUsed=before;intersection.HasCallAccounting=true;
        intersection.TimeWorkUsed+=direct.TimeWorkUsed;intersection.SpatialVisitedCells+=direct.SpatialVisitedCells;
        ++intersection.TemporalWidthRefinements;++intersection.ConsumerWidthRefinements;
        if(intersection.LowerTime<lastPublished.LowerTime||intersection.UpperTime>lastPublished.UpperTime)
        {bridgeFailed=true;return EContactEnclosureDecision::Refuse;}
        lastPublished=intersection;
        const auto next=(*filter)(intersection);
        if(next==EContactEnclosureDecision::Accept){accepted=MoveTemp(intersection);haveAccepted=true;}
        return next;
    };
    const FContactEnclosureFilter continuedFilter(continuedCallback);
    auto generic=BoundTensorSphereFirstContactGeneric(surfaces,path,radius,budget,nextTime,spatialGap,spatialNodes,
        nextPoint,nextNormal,maximumRefinements-1,&continuedFilter,geometryMemo);
    if(bridgeFailed)return fail(bridgeFailure);
    if(generic.Status==ETensorSphereContactStatus::FirstContactEnclosed)
    {
        if(!haveAccepted)return fail(ETensorSphereContactFailure::Arithmetic);
        result=MoveTemp(accepted);if(observe)++Counts.Accepted;return true;
    }
    if(generic.Status!=ETensorSphereContactStatus::Unknown)return fail(ETensorSphereContactFailure::Arithmetic);
    generic.CallInitialUsed=before;generic.HasCallAccounting=true;generic.TimeWorkUsed+=direct.TimeWorkUsed;
    generic.SpatialVisitedCells+=direct.SpatialVisitedCells;++generic.TemporalWidthRefinements;++generic.ConsumerWidthRefinements;
    result=MoveTemp(generic);result.Leaves.Reset();if(observe)++Counts.Failures;return true;
}
void PublishObservation()
{
    const auto& u=UnionDetail::Counts;
    if(u.Attempts.load())
    {
        UE_LOG(LogTemp,Display,TEXT("[AnalyticDirectCurveUnion] DefaultOFF=1 Providers=2 Attempts=%llu Proved=%llu AllMiss=%llu MissProviders=%llu RootProviders=%llu RetainedProviders=%llu LaterProviders=%llu Unavailable=%llu Seconds=%.9f"),
            u.Attempts.load(),u.Proved.load(),u.AllMiss.load(),u.MissProviders.load(),u.RootProviders.load(),u.RetainedProviders.load(),u.LaterProviders.load(),u.Unavailable.load(),FPlatformTime::ToSeconds64(u.Ticks.load()));
        UE_LOG(LogTemp,Display,TEXT("[AnalyticDirectCurveUnionAdmission] RootLocalLineProved=%llu Pending=%llu NoRoot=%llu PrefixAttempts=%llu PrefixMiss=%llu PrefixOverlap=%llu TargetsRefused=%llu LineInput=%llu LineShape=%llu LineSlope=%llu LineWholeCoverage=%llu LineFixedPlane=%llu LineNormal=%llu LineRoot=%llu LineRootCoverage=%llu LineInsideCoverage=%llu LineInverse=%llu"),
            u.RootLocalLineProved.load(),u.Pending.load(),u.NoRoot.load(),u.PrefixAttempts.load(),u.PrefixMiss.load(),u.PrefixOverlap.load(),u.TargetsRefused.load(),
            u.LineRefusal[1].load(),u.LineRefusal[2].load(),u.LineRefusal[3].load(),u.LineRefusal[4].load(),u.LineRefusal[5].load(),
            u.LineRefusal[6].load(),u.LineRefusal[7].load(),u.LineRefusal[8].load(),u.LineRefusal[9].load(),u.LineRefusal[10].load());
    }
    UE_LOG(LogTemp,Display,TEXT("[AnalyticDirectCurveContact] Calls=%llu LineProved=%llu OffsetProved=%llu CoplanarProved=%llu Accepted=%llu RefineFallback=%llu NotApplicable=%llu Failures=%llu Used=%llu Seconds=%.9f"),
        Counts.Calls.load(),Counts.LineProved.load(),Counts.OffsetProved.load(),Counts.CoplanarProved.load(),Counts.Accepted.load(),Counts.RefineFallback.load(),Counts.NotApplicable.load(),Counts.Failures.load(),Counts.Used.load(),
        FPlatformTime::ToSeconds64(Counts.Ticks.load()));
    UE_LOG(LogTemp,Display,TEXT("[DirectCurveValueTransport] DefaultOFF=1 OperationLocalOnly=1 LogicalWorkUnchanged=1 RootCentersReused=%llu RootPointsReused=%llu"),
        Counts.RootCentersReused.load(),Counts.RootPointsReused.load());
}
}


#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCoordinateLineContactCertificateTest,
    "IAmSpeed.AnalyticWorld.CoordinateLineContactCertificate.EveryOriginalFirstPoint",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCoordinateLineContactCertificateTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace A=::AnalyticCertified;namespace T=TimeDetail;using E=T::EMonotoneCurveTimeSeed;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(int32 normal:{0,1,2})for(int32 side:{-1,1})for(bool decreasing:{false,true})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        const int32 tangent=(normal+1)%3;const double q[6]={0,1,2,5,12,20};
        FIntervalTensorSurface curve;curve.DegreeU=0;curve.DegreeV=5;
        for(int32 i=0;i<6;++i){FVector3d p=FVector3d::ZeroVector;p[tangent]=q[decreasing?5-i:i];curve.ControlPoints.Add(Detail::Point(p));}
        TArray<FIntervalTensorSurface> surfaces{curve};
        FVector3d start=FVector3d::ZeroVector,end=FVector3d::ZeroVector;
        start[normal]=side*2.;start[tangent]=4.;end[tangent]=5.6875;
        FIntervalBezierCenterPath path;if(!CenterDetail::Linear(start,end,path))return false;
        FDistanceBudget work{10000,7,3};Interval time,parameter;int32 gotNormal=INDEX_NONE,gotSide=0;double a=0,b=1;
        const auto status=T::TryCoordinateLineContactCertificate(surfaces,path,1.,work,1.e-5,time,parameter,gotNormal,gotSide,a,b);
        if(!TestTrue(TEXT("direct quotient and original nonuniform parameter cover the true first point"),status==E::Proved&&
            time.Lower<=.5&&time.Upper>=.5&&parameter.Lower<=.5&&parameter.Upper>=.5&&
            gotNormal==normal&&gotSide==side&&a<time.Lower&&b>time.Upper&&a<.5&&b>.5&&
            work.Used>=7&&work.Used<=7+128&&work.Used+uint64(work.Remaining)==10007&&work.ConsumptionFloor==3))return false;
        const auto width=A::Subtract(Interval::Exact(parameter.Upper),Interval::Exact(parameter.Lower));
        if(!TestTrue(TEXT("original UV inverse is narrow without an arbitrary line parameter"),A::Valid(width)&&width.Upper<1.e-10))return false;
        const auto rootCenter=CenterDetail::Evaluate(path,time);
        if(!TestTrue(TEXT("root center encloses the true point and excludes padded Inside(B) displacement"),rootCenter.Axis[tangent].Lower<=4.84375&&rootCenter.Axis[tangent].Upper>=4.84375&&
            rootCenter.Axis[normal].Lower<=side&&rootCenter.Axis[normal].Upper>=side&&time.Upper-time.Lower<1.e-10))return false;
        FIntervalPoint point;if(!ProjectedDistanceDetail::EvaluateFixedUV(surfaces[0],FVector2d(0,.5),point))return false;
        if(!TestTrue(TEXT("certificate UV belongs to the original bent parameterization"),point.Axis[tangent].Lower<=4.84375&&point.Axis[tangent].Upper>=4.84375))return false;
        // Independent coefficient and tangential path boxes do not authorize a
        // midpoint coefficient realization: both extreme original parameters survive.
        for(auto& cp:surfaces[0].ControlPoints)cp.Axis[tangent]={cp.Axis[tangent].Lower-1.e-7,cp.Axis[tangent].Upper+1.e-7,true};
        for(auto& cp:path.ControlPoints)cp.Axis[tangent]={cp.Axis[tangent].Lower-1.e-7,cp.Axis[tangent].Upper+1.e-7,true};
        FDistanceBudget families{10000};Interval tf,vf;int32 nf,sf;double af,bf;
        if(!TestTrue(TEXT("every fixed uncertain realization retains its real parameter"),T::TryCoordinateLineContactCertificate(surfaces,path,1.,families,1.e-5,tf,vf,nf,sf,af,bf)==E::Proved&&
            vf.Lower<.5&&vf.Upper>.5&&tf.Lower<=.5&&tf.Upper>=.5))return false;
        FDistanceBudget small{130,5,3};
        if(!TestTrue(TEXT("unavailable128 reserve is a paid-nothing fallback"),T::TryCoordinateLineContactCertificate(surfaces,path,1.,small,1.e-5,tf,vf,nf,sf,af,bf)==E::NotApplicable&&small.Used==5&&small.Remaining==130&&!tf.Known&&!vf.Known))return false;
        for(auto& cp:path.ControlPoints)cp.Axis[tangent]=Interval::Exact(-5);
        FDistanceBudget caps{10000};
        if(!TestTrue(TEXT("finite caps outside tangent coverage cannot use the line certificate"),T::TryCoordinateLineContactCertificate(surfaces,path,1.,caps,1.e-5,tf,vf,nf,sf,af,bf)==E::NotApplicable&&!tf.Known&&!vf.Known))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Analytic::Certified::DirectCurveContactTests
{
inline FIntervalTensorSurface Line()
{
    FIntervalTensorSurface curve;curve.DegreeU=0;curve.DegreeV=5;
    const double q[6]={0,1,2,5,12,20};
    for(double x:q)curve.ControlPoints.Add(Detail::Point(FVector3d(x,0,0)));return curve;
}
inline bool SamePoint(const FIntervalPoint& a,const FIntervalPoint& b)
{
    for(int32 d=0;d<3;++d)if(a.Axis[d].Known!=b.Axis[d].Known||AnalyticCertified::Bits(a.Axis[d].Lower)!=AnalyticCertified::Bits(b.Axis[d].Lower)||
        AnalyticCertified::Bits(a.Axis[d].Upper)!=AnalyticCertified::Bits(b.Axis[d].Upper))return false;return true;
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDirectCurveCompleteGeometryTest,
    "IAmSpeed.AnalyticWorld.DirectCurveContact.OriginalFirstEventAndUV",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDirectCurveCompleteGeometryTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace A=::AnalyticCertified;
    namespace D=DirectCurveContact;namespace T=TimeDetail;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(bool line:{true,false})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        TArray<FIntervalTensorSurface> surfaces{line?DirectCurveContactTests::Line():MonotoneSeedTests::Parabola()};
        FIntervalBezierCenterPath path;
        const double radius=line?1.:2.,time=line?.5:1.-2./FMath::Sqrt(32.);
        const FVector3d firstPoint=line?FVector3d(4.84375,0,0):FVector3d(10,5,0);
        if(!CenterDetail::Linear(line?FVector3d(4,2,0):FVector3d(6,9,0),line?FVector3d(5.6875,0,0):FVector3d(10,5,0),path))return false;
        FDistanceBudget work{1000000,11,7};FTensorSphereContactEnclosure contact;contact.CallInitialUsed=work.Used;contact.HasCallAccounting=true;bool gotLine=false;
        const auto status=D::Prepare(surfaces,path,radius,work,1.e-5,1.e-4,1.e-4,line,!line,contact,gotLine);
        if(!TestTrue(TEXT("one original first event is fully certified without generic cells"),status==T::EMonotoneCurveTimeSeed::Proved&&gotLine==line&&
            contact.Status==ETensorSphereContactStatus::FirstContactEnclosed&&contact.LowerTime<=time&&contact.UpperTime>=time&&
            contact.Leaves.Num()>=1&&contact.Leaves.Num()<=2&&contact.SpatialSplitsU==0&&contact.SpatialSplitsV==0&&
            work.Used<=11+448&&work.Used+uint64(work.Remaining)==1000011&&work.ConsumptionFloor==7))return false;
        for(int32 d=0;d<3;++d)if(!TestTrue(TEXT("all true first point coordinates remain enclosed"),
            contact.SurfacePoint.Axis[d].Lower<=firstPoint[d]&&contact.SurfacePoint.Axis[d].Upper>=firstPoint[d]))return false;
        bool coversRoot=false;
        for(const auto& leaf:contact.Leaves)
        {
            coversRoot|=leaf.MinV<=.5&&leaf.MaxV>=.5;
            FIntervalPoint originalSample;
            if(!ProjectedDistanceDetail::EvaluateFixedUV(surfaces[0],leaf.FixedParameter,originalSample))return false;
            if(!TestTrue(TEXT("dyadic leaf retains a freshly evaluated original representative"),leaf.SurfaceIndex==0&&leaf.MinU==0&&leaf.MaxU==1&&
                leaf.MinV<leaf.MaxV&&leaf.FixedParameter.Y>leaf.MinV&&leaf.FixedParameter.Y<leaf.MaxV&&
                DirectCurveContactTests::SamePoint(originalSample,leaf.FixedParameterPoint)))return false;
        }
        if(!TestTrue(TEXT("closed original UV cells cover the exact first parameter"),coversRoot))return false;
        if(line)
        {if(!TestTrue(TEXT("line normal follows exact geometry, not parameter speed"),contact.RadialUnitNormal.Axis[0].Lower==0&&contact.RadialUnitNormal.Axis[0].Upper==0&&
             contact.RadialUnitNormal.Axis[1].Lower==1&&contact.RadialUnitNormal.Axis[1].Upper==1&&contact.RadialUnitNormal.Axis[2].Lower==0&&contact.RadialUnitNormal.Axis[2].Upper==0))return false;}
        else
        {const double n=FMath::Sqrt(.5);if(!TestTrue(TEXT("offset normal contains the original radial unit vector"),
             contact.RadialUnitNormal.Axis[0].Lower<=-n&&contact.RadialUnitNormal.Axis[0].Upper>=-n&&
             contact.RadialUnitNormal.Axis[1].Lower<=n&&contact.RadialUnitNormal.Axis[1].Upper>=n))return false;}
        FDistanceBudget shortWork{450,3,3};FTensorSphereContactEnclosure absent;bool ignored;
        if(!TestTrue(TEXT("unavailable complete448 reserve falls back without spending"),D::Prepare(surfaces,path,radius,shortWork,1.e-5,1.e-4,1.e-4,line,!line,absent,ignored)==T::EMonotoneCurveTimeSeed::NotApplicable&&
            shortWork.Used==3&&shortWork.Remaining==450&&shortWork.ConsumptionFloor==3&&absent.Leaves.IsEmpty()))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDirectCurveCallbackContinuationTest,
    "IAmSpeed.AnalyticWorld.DirectCurveContact.OnePaidGenericContinuation",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDirectCurveCallbackContinuationTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace D=DirectCurveContact;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.DirectCoordinateLineContact"),1,FName(TEXT("DirectContactContinuation")));
    FScopedCVar<int32> noOffset(TEXT("p.IAmSpeed.AnalyticWorld.DirectOffsetCurveContact"),0,FName(TEXT("DirectContactNoOffset")));
    TArray<FIntervalTensorSurface> surfaces{DirectCurveContactTests::Line()};FIntervalBezierCenterPath path;
    if(!CenterDetail::Linear(FVector3d(4,2,0),FVector3d(5.6875,0,0),path))return false;
    FDistanceBudget work{1000000,17,13};int32 calls=0;uint64 priorUsed=17;double lower=0,upper=1;bool valid=true;
    const auto callback=[&](const FTensorSphereContactEnclosure& candidate)
    {
        ++calls;valid&=candidate.Status==ETensorSphereContactStatus::FirstContactEnclosed&&candidate.ConsumerWidthRefinements==calls-1&&
            candidate.LowerTime>=lower&&candidate.UpperTime<=upper&&candidate.LowerTime<=.5&&candidate.UpperTime>=.5&&
            work.Used>priorUsed&&work.ConsumptionFloor==13&&!candidate.Leaves.IsEmpty();
        lower=candidate.LowerTime;upper=candidate.UpperTime;priorUsed=work.Used;
        valid&=work.Consume();return calls<3?EContactEnclosureDecision::Refine:EContactEnclosureDecision::Accept;
    };
    const FContactEnclosureFilter filter(callback);
    auto contact=BoundTensorSphereFirstContact(surfaces,path,1.,work,1.e-5,1.e-8,1024,1.e-3,1.e-3,5,&filter);
    if(!TestTrue(TEXT("first callback is not replayed and all continuations remain in the original account"),valid&&calls==3&&
        contact.Status==ETensorSphereContactStatus::FirstContactEnclosed&&contact.ConsumerWidthRefinements==2&&contact.TemporalWidthRefinements>=2&&
        contact.CallInitialUsed==17&&contact.TimeWorkUsed<=work.Used-17&&work.Used+uint64(work.Remaining)==1000017&&work.ConsumptionFloor==13))return false;
    FDistanceBudget refusedWork{1000000};int32 refusedCalls=0;
    const auto refuseCallback=[&](const FTensorSphereContactEnclosure&){++refusedCalls;return EContactEnclosureDecision::Refuse;};
    const FContactEnclosureFilter refuse(refuseCallback);
    const auto refused=BoundTensorSphereFirstContact(surfaces,path,1.,refusedWork,1.e-5,1.e-8,1024,1.e-3,1.e-3,5,&refuse);
    if(!TestTrue(TEXT("Refuse never exposes an earlier provisional contact"),refusedCalls==1&&refused.Status==ETensorSphereContactStatus::Unknown&&
        refused.Failure==ETensorSphereContactFailure::ConsumerRefusal&&refused.Leaves.IsEmpty()))return false;
    FDistanceBudget laterRefusedWork{1000000};int32 laterRefusedCalls=0;
    const auto laterRefuseCallback=[&](const FTensorSphereContactEnclosure&){return ++laterRefusedCalls==1?EContactEnclosureDecision::Refine:EContactEnclosureDecision::Refuse;};
    const FContactEnclosureFilter laterRefuse(laterRefuseCallback);
    const auto laterRefused=BoundTensorSphereFirstContact(surfaces,path,1.,laterRefusedWork,1.e-5,1.e-8,1024,1.e-3,1.e-3,5,&laterRefuse);
    if(!TestTrue(TEXT("Refuse after a genuine continuation retains no provisional contact"),laterRefusedCalls==2&&laterRefused.Status==ETensorSphereContactStatus::Unknown&&
        laterRefused.Failure==ETensorSphereContactFailure::ConsumerRefusal&&laterRefused.Leaves.IsEmpty()&&laterRefused.ConsumerWidthRefinements==1))return false;
    FDistanceBudget limitWork{1000000};int32 limitCalls=0;
    const auto refineCallback=[&](const FTensorSphereContactEnclosure&){++limitCalls;return EContactEnclosureDecision::Refine;};
    const FContactEnclosureFilter refine(refineCallback);
    const auto limited=BoundTensorSphereFirstContact(surfaces,path,1.,limitWork,1.e-5,1.e-8,1024,1.e-3,1.e-3,1,&refine);
    if(!TestTrue(TEXT("global allowance includes the first direct callback"),limitCalls==1&&limited.Status==ETensorSphereContactStatus::Unknown&&
        limited.Failure==ETensorSphereContactFailure::WidthRefinementLimit&&limited.Leaves.IsEmpty()))return false;
    FDistanceBudget drained{1000000,0,7};int32 drainCalls=0;
    const auto drainCallback=[&](const FTensorSphereContactEnclosure&){++drainCalls;while(drained.Remaining>drained.ConsumptionFloor)if(!drained.Consume())break;return EContactEnclosureDecision::Refine;};
    const FContactEnclosureFilter drain(drainCallback);
    const auto empty=BoundTensorSphereFirstContact(surfaces,path,1.,drained,1.e-5,1.e-8,1024,1.e-3,1.e-3,5,&drain);
    if(!TestTrue(TEXT("an exhausted continuation never resets or refunds"),drainCalls==1&&empty.Status==ETensorSphereContactStatus::Unknown&&
        empty.Failure==ETensorSphereContactFailure::Work&&empty.Leaves.IsEmpty()&&drained.Remaining==7&&drained.Used==999993))return false;
    FDistanceBudget changedWork{1000000};const uint32 control=VectorGetControlRegister();int32 changedCalls=0;
    const auto changeCallback=[&](const FTensorSphereContactEnclosure&){++changedCalls;VectorSetControlRegister(VectorGetControlRegister()^uint32(0x8000));return EContactEnclosureDecision::Accept;};
    const FContactEnclosureFilter changedFilter(changeCallback);
    const auto changed=BoundTensorSphereFirstContact(surfaces,path,1.,changedWork,1.e-5,1.e-8,1024,1.e-3,1.e-3,5,&changedFilter);
    VectorSetControlRegister(control);
    if(!TestTrue(TEXT("consumer numeric-control mutation cannot publish a direct contact"),changedCalls==1&&changed.Status==ETensorSphereContactStatus::Unknown&&
        changed.Failure==ETensorSphereContactFailure::NumericContext&&changed.Leaves.IsEmpty()))return false;
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDirectCurveIntersectionFallbackTest,
    "IAmSpeed.AnalyticWorld.DirectCurveContact.IntersectionAndUncoveredFallback",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDirectCurveIntersectionFallbackTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace D=DirectCurveContact;namespace A=::AnalyticCertified;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.DirectCoordinateLineContact"),1,FName(TEXT("DirectContactFallback")));
    FScopedCVar<int32> noOffset(TEXT("p.IAmSpeed.AnalyticWorld.DirectOffsetCurveContact"),0,FName(TEXT("DirectContactFallbackNoOffset")));
    TArray<FIntervalTensorSurface> surfaces{DirectCurveContactTests::Line()};FIntervalBezierCenterPath path;
    if(!CenterDetail::Linear(FVector3d(4,2,0),FVector3d(5.6875,0,0),path))return false;
    FDistanceBudget work{1000000};FTensorSphereContactEnclosure direct;bool usedLine;
    if(D::Prepare(surfaces,path,1.,work,1.e-5,1.e-3,1.e-3,true,false,direct,usedLine)!=TimeDetail::EMonotoneCurveTimeSeed::Proved)return false;
    auto broad=direct;broad.LowerTime=.4;broad.UpperTime=.6;
    for(auto& leaf:broad.Leaves)for(int32 d=0;d<3;++d)
    {leaf.PointHull.Axis[d]={leaf.PointHull.Axis[d].Lower-.1,leaf.PointHull.Axis[d].Upper+.1,true};
     leaf.RadialNormalHull.Axis[d]={leaf.RadialNormalHull.Axis[d].Lower-.1,leaf.RadialNormalHull.Axis[d].Upper+.1,true};}
    auto falseLeaf=direct.Leaves[0];
    falseLeaf.PointHull=Detail::Point(FVector3d(100,100,100));broad.Leaves.Add(falseLeaf);
    FTensorSphereContactEnclosure cut;const uint64 paid=work.Used;
    if(!TestTrue(TEXT("universal intersection removes only impossible first-point leaves and preserves original samples"),D::IntersectContact(broad,direct,cut,work)&&
        cut.Leaves.Num()==direct.Leaves.Num()&&cut.LowerTime==direct.LowerTime&&cut.UpperTime==direct.UpperTime&&work.Used>paid&&
        DirectCurveContactTests::SamePoint(cut.SurfacePoint,direct.SurfacePoint)&&DirectCurveContactTests::SamePoint(cut.RadialUnitNormal,direct.RadialUnitNormal)&&
        DirectCurveContactTests::SamePoint(cut.Leaves[0].FixedParameterPoint,direct.Leaves[0].FixedParameterPoint)))return false;
    broad.LowerTime=.8;broad.UpperTime=.9;
    if(!TestTrue(TEXT("contradictory first-time proofs fail closed"),!D::IntersectContact(broad,direct,cut,work)))return false;
    FIntervalBezierCenterPath curved;curved.Degree=2;
    curved.ControlPoints={Detail::Point(FVector3d(5,2,0)),Detail::Point(FVector3d(5,-2,0)),Detail::Point(FVector3d(5,2,0))};
    FDistanceBudget baseline{1000000},actual{1000000};
    const auto old=BoundTensorSphereFirstContactGeneric(surfaces,curved,1.,baseline,1.e-5,1.e-8,1024,1.e-3,1.e-3);
    const auto fallback=BoundTensorSphereFirstContact(surfaces,curved,1.,actual,1.e-5,1.e-8,1024,1.e-3,1.e-3);
    if(!TestTrue(TEXT("uncovered continuous nonlinear motion uses the identical generic route and debit"),old.Status==ETensorSphereContactStatus::FirstContactEnclosed&&
        fallback.Status==old.Status&&A::Bits(old.LowerTime)==A::Bits(fallback.LowerTime)&&A::Bits(old.UpperTime)==A::Bits(fallback.UpperTime)&&baseline.Used==actual.Used&&baseline.Remaining==actual.Remaining))return false;
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
#endif

namespace Speed::Analytic::Certified::GroupedBoxStripProof
{
namespace Private
{
static TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxStripProof"),0,
    TEXT("Default OFF. Bounded universal 1D strip clearance before original grouped 2D proof; no contact/normal/witness."),ECVF_Default);
static TAutoConsoleVariable<int32> ClipFreeAxis(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxStripProof.ClipFreeAxis"),0,
    TEXT("Default OFF. Clip the collapsed world coordinate against each fine Box group before projecting the retained Bernstein strip."));
static std::atomic<uint64> Calls{0},Cleared{0},Unresolved{0},NotApplicable{0},Invalid{0},Cells{0},Used{0},Ticks{0},CollapsedU{0},CollapsedV{0};
constexpr int32 DefaultAllowance=128,MaximumAllowance=512;
static TAutoConsoleVariable<int32> WorkAllowance(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxStripProof.Work"),DefaultAllowance,
    TEXT("Private strip probe debit from the EXISTING account, clamped128..512; does not increase a caller cap or refund fallback work."),ECVF_Default);
static int32 CurrentAllowance(){return FMath::Clamp(WorkAllowance.GetValueOnAnyThread(),DefaultAllowance,MaximumAllowance);}
struct FDetail {uint64 Cells=0;int32 Collapsed=0,FreeAxis=INDEX_NONE;};
// Equal coefficient boxes do NOT assert correlation. Regardless of their
// independent realizations, Hull_i(P_ij) contains sum_i B_i(u) P_ij. Thus the
// curve-family of these row hulls contains EVERY S(u,v), for ALL original u.
// Bit-equality of transverse rows is a usefulness heuristic only; the proof
// always uses the full hull, including the independently varying free axis.
enum class EBuild:uint8 {NotApplicable,Ready,Unresolved,Invalid};
static EBuild BuildStrip(const FIntervalTensorSurface& surface,FDistanceBudget& budget,
    Detail::FNode& output,FDetail& observed)
{
    output={};
    if(surface.DegreeU<=0||surface.DegreeV<=0||surface.DegreeU>16||surface.DegreeV>16||
        surface.ControlPoints.Num()!=(surface.DegreeU+1)*(surface.DegreeV+1))return EBuild::NotApplicable;
    TArray<FIntervalPoint,TInlineAllocator<17>> keepV,keepU;
    keepV.SetNum(surface.DegreeV+1);keepU.SetNum(surface.DegreeU+1);
    bool variesU[3]{},variesV[3]{};const int32 vc=surface.DegreeV+1;
    for(int32 u=0;u<=surface.DegreeU;++u)for(int32 v=0;v<=surface.DegreeV;++v)
    {
        if(!budget.Consume())return EBuild::Unresolved;
        const auto& point=surface.ControlPoints[u*vc+v];
        if(!Detail::ValidPoint(point))return EBuild::Invalid;
        keepV[v]=u?TimeDetail::Hull(keepV[v],point):point;
        keepU[u]=v?TimeDetail::Hull(keepU[u],point):point;
        for(int32 d=0;d<3;++d)
        {
            variesU[d]|=!GroupedBoxDetail::SameInterval(point.Axis[d],surface.ControlPoints[v].Axis[d]);
            variesV[d]|=!GroupedBoxDetail::SameInterval(point.Axis[d],surface.ControlPoints[u*vc].Axis[d]);
        }
    }
    const bool alongU=int32(variesU[0])+int32(variesU[1])+int32(variesU[2])<=1;
    const bool alongV=int32(variesV[0])+int32(variesV[1])+int32(variesV[2])<=1;
    if(!alongU&&!alongV)return EBuild::NotApplicable;
    // Prefer collapsing the larger polynomial degree; this is structure only,
    // not a provider ID, position, fitted plane, or numerical acceptance margin.
    const bool collapseU=alongU&&(!alongV||surface.DegreeU>=surface.DegreeV);
    output.DegreeU=0;output.DegreeV=collapseU?surface.DegreeV:surface.DegreeU;
    const auto& points=collapseU?keepV:keepU;output.Net.Append(points.GetData(),points.Num());
    observed.Collapsed=collapseU?1:2;
    const auto& varies=collapseU?variesU:variesV;
    for(int32 d=0;d<3;++d)if(varies[d])observed.FreeAxis=d;
    return EBuild::Ready; // geometry built, NOT a clearance
}
static EResult ProveClipped(const Detail::FNode& root,const FGroupedBoxVolumeBounds& volume,
    FDistanceBudget& budget,FDetail& observed)
{
    namespace A=::AnalyticCertified;
    constexpr int32 GroupCount=14,AxisCount=6,DirectionCount=AxisCount-1,ControlCount=6;
    const int32 free=observed.FreeAxis;
    if(free<0||free>=3||root.DegreeU!=0||root.DegreeV!=5||root.Net.Num()!=ControlCount||
        volume.Groups.Num()!=GroupCount||volume.Axes.Num()!=AxisCount)return EResult::NotApplicable;
    for(int32 d=0;d<3;++d)
    {
        FVector3d axis=FVector3d::ZeroVector;axis[d]=1.;
        if(volume.Axes[d]!=axis)return EResult::NotApplicable;
    }
    int32 directions[DirectionCount];int32 next=0;
    for(int32 d=0;d<AxisCount;++d)if(d!=free)directions[next++]=d;
    const auto intervalHull=[](const Interval& a,const Interval& b)
    {
        if(!A::Valid(a)||!A::Valid(b))return Interval{};
        return Interval{FMath::Min(a.Lower,b.Lower),FMath::Max(a.Upper,b.Upper),true};
    };
    auto freeRange=root.Net[0].Axis[free];
    for(const auto& point:root.Net)freeRange=intervalHull(freeRange,point.Axis[free]);
    if(!A::Valid(freeRange))return EResult::Invalid;

    struct FCell
    {
        Interval Net[DirectionCount][ControlCount];
        uint32 Active=0;
        int32 Depth=0;
    };
    FCell initial;
    Interval targets[GroupCount][DirectionCount];
    // If a patch point lies in a group, its free coordinate lies in
    // both original intervals. Subtract the complete possible free
    // contribution from each sealed slab, independently for that group.
    // The remaining transverse strip still encloses all original u.
    for(int32 g=0;g<GroupCount;++g)
    {
        if(!budget.Consume())return EResult::Unresolved;
        const auto world=volume.Slabs[g*AxisCount+free];
        if(!A::Valid(world))return EResult::Invalid;
        const Interval clipped{FMath::Max(world.Lower,freeRange.Lower),FMath::Min(world.Upper,freeRange.Upper),true};
        if(clipped.Lower>clipped.Upper)continue;
        initial.Active|=1u<<g;
        for(int32 k=0;k<DirectionCount;++k)
        {
            if(!budget.Consume())return EResult::Unresolved;
            const int32 axis=directions[k];
            const auto freeContribution=A::Multiply(Interval::Exact(volume.Axes[axis][free]),clipped);
            targets[g][k]=A::Subtract(volume.Slabs[g*AxisCount+axis],freeContribution);
            if(!A::Valid(targets[g][k]))return EResult::Invalid;
        }
    }
    if(!initial.Active)return EResult::Cleared;
    for(int32 k=0;k<DirectionCount;++k)
    {
        if(!budget.Consume())return EResult::Unresolved;
        const auto& axis=volume.Axes[directions[k]];
        for(int32 v=0;v<ControlCount;++v)
        {
            auto projected=Interval::Exact(0.);
            for(int32 d=0;d<3;++d)if(d!=free)
                projected=A::Add(projected,A::Multiply(Interval::Exact(axis[d]),root.Net[v].Axis[d]));
            if(!A::Valid(projected))return EResult::Invalid;
            initial.Net[k][v]=projected;
        }
    }
    TArray<FCell,TInlineAllocator<16>> stack;stack.Add(MoveTemp(initial));
    while(!stack.IsEmpty())
    {
        auto cell=stack.Pop(EAllowShrinking::No);++observed.Cells;
        Interval projected[DirectionCount];
        for(int32 k=0;k<DirectionCount;++k)
        {
            if(!budget.Consume())return EResult::Unresolved;
            auto hull=cell.Net[k][0];
            for(int32 v=1;v<ControlCount;++v)hull=intervalHull(hull,cell.Net[k][v]);
            if(!A::Valid(hull))return EResult::Invalid;
            projected[k]=hull;
        }
        for(int32 g=0;g<GroupCount;++g)if(cell.Active&(1u<<g))
        {
            if(!budget.Consume())return EResult::Unresolved;
            for(int32 k=0;k<DirectionCount;++k)
                if(projected[k].Upper<targets[g][k].Lower||targets[g][k].Upper<projected[k].Lower)
                {cell.Active&=~(1u<<g);break;}
        }
        if(!cell.Active)continue;
        if(cell.Depth>=48||!budget.Consume()||!budget.Consume())return EResult::Unresolved;
        FCell left,right;
        left.Active=right.Active=cell.Active;left.Depth=right.Depth=cell.Depth+1;
        for(int32 k=0;k<DirectionCount;++k)
        {
            Interval row[ControlCount];
            for(int32 v=0;v<ControlCount;++v)row[v]=cell.Net[k][v];
            left.Net[k][0]=row[0];right.Net[k][ControlCount-1]=row[ControlCount-1];
            for(int32 level=1;level<ControlCount;++level)
            {
                for(int32 v=0;v<ControlCount-level;++v)
                {
                    row[v]=A::HalfSum(row[v],row[v+1]);
                    if(!A::Valid(row[v]))return EResult::Invalid;
                }
                left.Net[k][level]=row[0];right.Net[k][ControlCount-1-level]=row[ControlCount-1-level];
            }
        }
        stack.Add(MoveTemp(right));stack.Add(MoveTemp(left));
    }
    return EResult::Cleared;
}

static EResult Prove(const FIntervalTensorSurface& surface,const FGroupedBoxVolumeBounds& volume,
    FDistanceBudget& budget,bool memo,FDetail& observed,int32 allowance=DefaultAllowance,bool clipFree=false)
{
    namespace A=::AnalyticCertified;
    const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=EAnalyticNumericStatus::Complete||budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor)return EResult::Invalid;
    if(surface.DegreeU<=0||surface.DegreeV<=0||surface.DegreeU>16||surface.DegreeV>16||
        surface.ControlPoints.Num()!=(surface.DegreeU+1)*(surface.DegreeV+1)||surface.ControlPoints.Num()>64)return EResult::NotApplicable;
    if(volume.Groups.IsEmpty()||volume.Groups.Num()>16||volume.Axes.IsEmpty()||volume.Axes.Num()>12||
        volume.Slabs.Num()!=volume.Groups.Num()*volume.Axes.Num())return EResult::Invalid;
    // Preserve the existing first 2D cell's projection/group checks and two
    // possible children above the caller's floor. No budget is ever restored.
    const int64 fallbackMinimum=int64(volume.Axes.Num())+volume.Groups.Num()+2;
    if(allowance<DefaultAllowance||allowance>MaximumAllowance||
        int64(budget.Remaining)-budget.ConsumptionFloor<allowance+fallbackMinimum||
        budget.Used>MAX_uint64-uint64(allowance)-uint64(fallbackMinimum))return EResult::NotApplicable;
    FScopedDistanceBudgetFloor limit(budget,budget.Remaining-allowance);
    if(!limit.IsValid())return EResult::Invalid;
    if(!budget.Consume())return EResult::Unresolved;
    Detail::FNode root;const auto built=BuildStrip(surface,budget,root,observed);
    if(built!=EBuild::Ready)return built==EBuild::NotApplicable?EResult::NotApplicable:built==EBuild::Unresolved?EResult::Unresolved:EResult::Invalid;
    if(clipFree)return ProveClipped(root,volume,budget,observed);
    struct FWork {Detail::FNode Cell;uint32 ActiveGroups;};
    TArray<FWork,TInlineAllocator<16>> stack;stack.Add(FWork{MoveTemp(root),(1u<<volume.Groups.Num())-1u});
    while(!stack.IsEmpty())
    {
        auto work=stack.Pop(EAllowShrinking::No);++observed.Cells;
        TArray<Interval,TInlineAllocator<8>> projected;
        if(memo)
        {
            if(!GroupedBoxGeometryMemo::ProjectCellForVolume(work.Cell,volume,work.ActiveGroups,budget,projected,nullptr))return EResult::Unresolved;
        }
        else for(const auto& axis:volume.Axes)
        {
            if(!budget.Consume())return EResult::Unresolved;
            const auto slab=GroupedBoxDetail::ProjectHull(work.Cell.Net,axis);
            if(!A::Valid(slab))return EResult::Unresolved;projected.Add(slab);
        }
        for(int32 g=0;g<volume.Groups.Num();++g)
        {
            const uint32 bit=1u<<g;if(!(work.ActiveGroups&bit))continue;
            if(!budget.Consume())return EResult::Unresolved;bool separated=false;
            for(int32 axis=0;axis<volume.Axes.Num();++axis)
            {
                const auto body=volume.Slabs[g*volume.Axes.Num()+axis],patch=projected[axis];
                if(!A::Valid(body)||!A::Valid(patch))return EResult::Invalid;
                separated|=body.Upper<patch.Lower||patch.Upper<body.Lower;
            }
            if(separated)work.ActiveGroups&=~bit;
        }
        if(!work.ActiveGroups)continue;
        if(budget.Remaining-budget.ConsumptionFloor<2)return EResult::Unresolved;
        Detail::FNode left,right;
        if(memo?!GroupedBoxGeometryMemo::SplitCell(work.Cell,left,right):!Detail::Split(work.Cell,false,left,right))return EResult::Unresolved;
        if(!budget.Consume()||!budget.Consume())return EResult::Unresolved;
        // Every sibling keeps ALL its unresolved group obligations. A clear
        // sibling cannot discharge the other half of the original strip.
        stack.Add(FWork{MoveTemp(right),work.ActiveGroups});stack.Add(FWork{MoveTemp(left),work.ActiveGroups});
    }
    return EResult::Cleared;
}
static void Publish()
{
    UE_LOG(LogTemp,Display,TEXT("[GroupedBoxStripProof] Scope=ProgramPrefixAtPreExit DefaultOFF=1 ClearOnly=1 AllowanceDefault=128 AllowanceMax=512 ConfiguredAllowance=%d Calls=%llu Cleared=%llu Unresolved=%llu NotApplicable=%llu Invalid=%llu Cells=%llu BudgetUsed=%llu Ticks=%llu CollapsedU=%llu CollapsedV=%llu SecondsPerCycle=%.17g"),
        CurrentAllowance(),Calls.load(),Cleared.load(),Unresolved.load(),NotApplicable.load(),Invalid.load(),Cells.load(),Used.load(),Ticks.load(),CollapsedU.load(),CollapsedV.load(),FPlatformTime::GetSecondsPerCycle64());
}
struct FRegister {FRegister(){FCoreDelegates::OnPreExit.AddStatic(&Publish);}};static FRegister Register;
}
bool RuntimeEnabled(){return Private::Enabled.GetValueOnAnyThread()==1;}
EResult TryClear(const FIntervalTensorSurface& surface,const FGroupedBoxVolumeBounds& volume,FDistanceBudget& budget,bool memo)
{
    const bool observe=StrongConvexCurveMinimum::Observation::Enabled();
    const uint64 start=observe?FPlatformTime::Cycles64():0,before=budget.Used;
    const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    const bool clip=volume.Groups.Num()==14&&Private::ClipFreeAxis.GetValueOnAnyThread()==1;
    Private::FDetail detail;auto result=Private::Prove(surface,volume,budget,memo,detail,Private::CurrentAllowance(),clip);
    const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=EAnalyticNumericStatus::Complete||after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=numeric.ControlFingerprint())result=EResult::Invalid;
    if(observe)
    {
        ++Private::Calls;Private::Used.fetch_add(budget.Used-before);Private::Ticks.fetch_add(FPlatformTime::Cycles64()-start);Private::Cells.fetch_add(detail.Cells);
        if(detail.Collapsed==1)++Private::CollapsedU;else if(detail.Collapsed==2)++Private::CollapsedV;
        if(result==EResult::Cleared)++Private::Cleared;else if(result==EResult::Unresolved)++Private::Unresolved;
        else if(result==EResult::NotApplicable)++Private::NotApplicable;else ++Private::Invalid;
    }
    return result;
}
}

#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Analytic::Certified::GroupedBoxStripTests
{
static FIntervalTensorSurface Extrusion(bool transpose=false,double family=.0)
{
    FIntervalTensorSurface surface;surface.DegreeU=surface.DegreeV=5;
    for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)
    {
        const int32 free=transpose?v:u,curve=transpose?u:v;
        FIntervalPoint p=Detail::Point(FVector3d(-1024.+2048.*free/5.,-2.+4.*curve/5.,3.-4.*curve/5.+curve*(curve-1)/5.));
        for(int32 d:{1,2}){p.Axis[d].Lower-=family;p.Axis[d].Upper+=family;}
        surface.ControlPoints.Add(p);
    }
    return surface;
}
static bool Volume(double height,FDistanceBudget& budget,FGroupedBoxVolumeBounds& volume,bool extraCollisionGroup=false)
{
    TArray<FBoxVolumeGroup> groups;FIntervalPoint hull;
    for(int32 g=0;g<(extraCollisionGroup?2:1);++g)
    {
        FBoxVolumeGroup group;
        for(int32 k=0;k<8;++k)
        {
            const auto p=Detail::Point(FVector3d(k&1?.25:-.25,k&2?.1:-.1,(g?2.:height)+(k&4?.1:-.1)));
            group.Corners.Add(p);hull=Detail::ValidPoint(hull)?TimeDetail::Hull(hull,p):p;
        }
        groups.Add(MoveTemp(group));
    }
    const TArray<FVector3d> axes{FVector3d(1,0,0),FVector3d(0,1,0),FVector3d(0,0,1),
        FVector3d(1,1,0),FVector3d(0,1,1),FVector3d(1,0,1)};
    return CloseGroupedBoxVolumeBounds(hull,groups,axes,budget,volume);
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGroupedBoxStripGeometryTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.StripUniversalEnvelope",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FGroupedBoxStripGeometryTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace S=GroupedBoxStripProof;namespace A=::AnalyticCertified;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    FScopedCVar<int32> containment(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxContainedReject"),0,FName(TEXT("StripNoContained")));
    FScopedCVar<int32> workDefault(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxStripProof.Work"),128,FName(TEXT("StripDefaultWork")));
    for(uint32 flush:{0u,64u,32768u,32832u})for(bool transpose:{false,true})for(bool memo:{false,true})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        auto surface=GroupedBoxStripTests::Extrusion(transpose,1./1024.);FDistanceBudget build{10000};FGroupedBoxVolumeBounds volume;
        if(!GroupedBoxStripTests::Volume(1.75,build,volume))return false;
        Detail::FNode strip;S::Private::FDetail detail;FDistanceBudget geometry{128};
        if(!TestTrue(TEXT("structural one-changing-axis admission, either orientation"),S::Private::BuildStrip(surface,geometry,strip,detail)==S::Private::EBuild::Ready&&
            strip.DegreeU==0&&strip.DegreeV==5&&strip.Net.Num()==6&&detail.Collapsed==(transpose?2:1)&&geometry.Used==36))return false;
        // Every original coefficient box remains inside its strip row hull.
        // Equal transverse boxes still represent independent coefficients.
        for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)
        {
            const auto& originalCP=surface.ControlPoints[u*6+v];const auto& row=strip.Net[transpose?u:v];
            for(int32 d=0;d<3;++d)if(!TestTrue(TEXT("no independent coefficient lost or correlated"),row.Axis[d].Lower<=originalCP.Axis[d].Lower&&row.Axis[d].Upper>=originalCP.Axis[d].Upper))return false;
        }
        FDistanceBudget probe{10000,11,50};S::Private::FDetail observed;
        if(!TestTrue(TEXT("full V subdivision proves strip separated"),S::Private::Prove(surface,volume,probe,memo,observed)==S::EResult::Cleared&&observed.Cells>1&&
            probe.Used>11&&probe.Used<=11+128&&probe.ConsumptionFloor==50))return false;
        FGroupedBoxVolumeBounds collision;if(!GroupedBoxStripTests::Volume(2.,build,collision))return false;
        FDistanceBudget overlap{10000};if(!TestTrue(TEXT("a real original intersection cannot become Clear"),S::TryClear(surface,collision,overlap,memo)!=S::EResult::Cleared&&overlap.Used<=128))return false;
        FGroupedBoxVolumeBounds two;if(!GroupedBoxStripTests::Volume(1.75,build,two,true))return false;
        FDistanceBudget obligations{10000};if(!TestTrue(TEXT("one clear group never discharges another intersecting group"),S::TryClear(surface,two,obligations,memo)!=S::EResult::Cleared))return false;
        auto uncertain=GroupedBoxStripTests::Extrusion(transpose,.5);FDistanceBudget independent{10000};
        if(!TestTrue(TEXT("independent broad coefficient family cannot inherit nominal clearance"),S::TryClear(uncertain,volume,independent,memo)!=S::EResult::Cleared))return false;
        auto coupled=surface;coupled.ControlPoints[transpose?1:6].Axis[1]=Interval::Exact(7.);
        Detail::FNode rejected;S::Private::FDetail rejectedDetail;FDistanceBudget coupledWork{128};
        if(!TestTrue(TEXT("genuinely two-coordinate coupling is not admitted as the original strip orientation"),S::Private::BuildStrip(coupled,coupledWork,rejected,rejectedDetail)==S::Private::EBuild::NotApplicable))return false;
        FDistanceBudget tiny{137,7,4};const auto remaining=tiny.Remaining;const auto used=tiny.Used;
        if(!TestTrue(TEXT("dedicated128 plus fallback minimum unavailable gives untouched NA"),S::TryClear(surface,volume,tiny,memo)==S::EResult::NotApplicable&&tiny.Remaining==remaining&&tiny.Used==used))return false;
    }
    // Private local allowance is never a replacement account. Even invalid
    // settings clamp to128..512; each configured attempt leaves the current
    // floor plus the original first-cell minimum available for fallback.
    for(int32 configured:{MIN_int32,0,127,128,256,512,513,MAX_int32})
    {
        FScopedCVar<int32> setting(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxStripProof.Work"),configured,FName(TEXT("StripWorkBound")));
        const int32 allowance=FMath::Clamp(configured,128,512);
        if(!TestTrue(TEXT("technical setting is bounded without expanding any caller account"),S::Private::CurrentAllowance()==allowance))return false;
        FDistanceBudget build{10000};FGroupedBoxVolumeBounds collision;
        if(!GroupedBoxStripTests::Volume(2.,build,collision))return false;
        const int32 fallback=collision.Axes.Num()+collision.Groups.Num()+2,initial=allowance+fallback+17;
        FDistanceBudget probe{initial,5,17};const auto outcome=S::TryClear(GroupedBoxStripTests::Extrusion(),collision,probe,false);
        if(!TestTrue(TEXT("unresolved configured probe preserves exact debit and protected fallback/floor"),outcome!=S::EResult::Cleared&&
            probe.ConsumptionFloor==17&&probe.Used>5&&probe.Used<=uint64(5+allowance)&&
            probe.Remaining>=17+fallback&&uint64(probe.Remaining)+probe.Used==uint64(initial+5)))return false;
        FDistanceBudget shortWork{initial-1,5,17};
        if(!TestTrue(TEXT("insufficient configured allowance plus reserve is untouched"),
            S::TryClear(GroupedBoxStripTests::Extrusion(),collision,shortWork,false)==S::EResult::NotApplicable&&
            shortWork.Remaining==initial-1&&shortWork.Used==5&&shortWork.ConsumptionFloor==17))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGroupedBoxStripIntegrationTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.StripClearOrPaidFallback",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FGroupedBoxStripIntegrationTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace S=GroupedBoxStripProof;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> containment(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxContainedReject"),0,FName(TEXT("StripIntegrationNoContained")));
    FScopedCVar<int32> workDefault(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxStripProof.Work"),128,FName(TEXT("StripIntegrationDefaultWork")));
    for(bool memo:{false,true})
    {
        FDistanceBudget build{100000};FGroupedBoxVolumeBounds volume;if(!GroupedBoxStripTests::Volume(1.75,build,volume))return false;
        TArray<FIntervalTensorSurface> surfaces{GroupedBoxStripTests::Extrusion()};FIndexedIntervalUnionPtr scene;
        FReferenceBoundsCandidateSetPtr partition;
        if(!FIndexedIntervalUnion::Build(surfaces,build,scene)||!FReferenceBoundsCandidateSet::Select(scene,volume.BroadHull,build,partition))return false;
        FDistanceBudget a{100000},b=a;FReferenceBoxVolumeStrictSeparationPtr oldProof,newProof;FGroupedBoxSeparationObservation oa,ob;
        if(!TestTrue(TEXT("unchanged authoritative volume and scene prove through both routes"),FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,a,oldProof,&oa,memo,false)&&
            FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,b,newProof,&ob,memo,true)&&oldProof->Matches(scene,volume)&&newProof->Matches(scene,volume)))return false;
        if(!TestTrue(TEXT("strip removes two-dimensional descendants without inventing a different volume"),oa.VisitedCells>0&&ob.VisitedCells==0&&b.Used<a.Used))return false;
        TestTrue(TEXT("endpoint consumer still checks the original complete group"),newProof->ContainsCompleteEndpointGroup(volume.Groups[0].Corners));
        // Tight budgets retain every failed probe debit. No previous proof may
        // leak into an Unknown return. At least one insufficient case is real.
        bool sawFalse=false;
        for(int32 cap=1;cap<400;++cap)
        {
            FDistanceBudget narrow{cap,3,1};auto stale=newProof;
            const bool clear=FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,narrow,stale,nullptr,memo,true);
            if(!clear){sawFalse=true;if(!TestTrue(TEXT("failed narrow proof is null with no reset/refund"),!stale&&narrow.Remaining<=cap&&narrow.Used>=3&&narrow.Used<=uint64(cap+2)))return false;}
        }
        if(!TestTrue(TEXT("insufficient-work branch is nonvacuous"),sawFalse))return false;
        auto coupled=GroupedBoxStripTests::Extrusion();
        coupled.ControlPoints[6].Axis[1].Lower+=.01;coupled.ControlPoints[6].Axis[1].Upper+=.01;
        TArray<FIntervalTensorSurface> coupledSurfaces{coupled,coupled};FIndexedIntervalUnionPtr coupledScene;
        FReferenceBoundsCandidateSetPtr coupledPartition;
        if(!FIndexedIntervalUnion::Build(coupledSurfaces,build,coupledScene)||!FReferenceBoundsCandidateSet::Select(coupledScene,volume.BroadHull,build,coupledPartition))return false;
        FDistanceBudget oldFull{100000};FReferenceBoxVolumeStrictSeparationPtr coupledProof;
        if(!TestTrue(TEXT("coupled surface still clears through ordinary 2D proof"),FReferenceBoxVolumeStrictSeparation::TryProve(coupledPartition,volume,oldFull,coupledProof,nullptr,memo,false)))return false;
        FDistanceBudget exactOld{int32(oldFull.Used)},paidProbe=exactOld;FReferenceBoxVolumeStrictSeparationPtr exactProof,paidProof=coupledProof;
        if(!TestTrue(TEXT("old exact allowance is sufficient only for the old route"),FReferenceBoxVolumeStrictSeparation::TryProve(coupledPartition,volume,exactOld,exactProof,nullptr,memo,false)&&
            !FReferenceBoxVolumeStrictSeparation::TryProve(coupledPartition,volume,paidProbe,paidProof,nullptr,memo,true)&&!paidProof&&paidProbe.Used==oldFull.Used))return false;
        FGroupedBoxVolumeBounds collision;if(!GroupedBoxStripTests::Volume(2.,build,collision))return false;
        FReferenceBoundsCandidateSetPtr collisionPartition;
        if(!FReferenceBoundsCandidateSet::Select(scene,collision.BroadHull,build,collisionPartition))return false;
        FDistanceBudget no{4096};auto stale=newProof;
        if(!TestTrue(TEXT("near/intersecting provider cannot publish a clearance through fallback"),!FReferenceBoxVolumeStrictSeparation::TryProve(collisionPartition,collision,no,stale,nullptr,memo,true)&&!stale&&no.Used>128))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
#endif


#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Analytic::Certified::DirectCoplanarTests
{
static FIntervalTensorSurface Patch(int32 axis,double shift=0,double height=0)
{
    FIntervalTensorSurface result;result.DegreeU=result.DegreeV=5;
    const double q[6]={0,1,2,5,12,20};const int32 x=(axis+1)%3,y=(axis+2)%3;
    for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)
    {FVector3d p=FVector3d::ZeroVector;p[axis]=height;p[x]=q[u]+shift;p[y]=q[v];result.ControlPoints.Add(Detail::Point(p));}
    return result;
}
static FIntervalTensorSurface Triangle(int32 axis)
{
    FIntervalTensorSurface result;result.DegreeU=result.DegreeV=1;
    const int32 x=(axis+1)%3,y=(axis+2)%3;
    for(const FVector2d p:{FVector2d(-100,-100),FVector2d(100,-100),FVector2d(0,100),FVector2d(0,100)})
    {FVector3d point=FVector3d::ZeroVector;point[x]=p.X;point[y]=p.Y;result.ControlPoints.Add(Detail::Point(point));}
    return result;
}
static bool Path(int32 axis,int32 side,FIntervalBezierCenterPath& path)
{
    FVector3d a(5,5,5),b=a;a[axis]=2.*side;b[axis]=0;return CenterDetail::Linear(a,b,path);
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDirectCoplanarCompleteContactTest,
    "IAmSpeed.AnalyticWorld.DirectCoplanarContact.CompleteFiniteFamilyAndOriginalUV",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDirectCoplanarCompleteContactTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace D=DirectCurveContact;namespace C=DirectCoplanarTests;namespace A=::AnalyticCertified;using E=TimeDetail::EMonotoneCurveTimeSeed;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> tensor(TEXT("p.IAmSpeed.AnalyticWorld.TensorPlanarFamily"),1,FName(TEXT("DirectCoplanarTensor")));
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(int32 axis:{0,1,2})for(int32 side:{-1,1})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        TArray<FIntervalTensorSurface> surfaces{C::Triangle(axis),C::Patch(axis),C::Patch(axis,50)};FIntervalBezierCenterPath path;
        if(!C::Path(axis,side,path))return false;
        FDistanceBudget work{100000,7,11};FTensorSphereContactEnclosure contact;TArray<uint8> disposition;
        if(!TestTrue(TEXT("whole original family proves the exact first event"),D::PrepareCoplanar(surfaces,path,1.,work,1.e-5,1.e-4,1.e-4,contact,disposition)==E::Proved&&
            contact.Status==ETensorSphereContactStatus::FirstContactEnclosed&&contact.LowerTime<=.5&&contact.UpperTime>=.5&&
            disposition.Num()==3&&disposition[0]==1&&disposition[1]==1&&disposition[2]==0&&work.Used<=7+4096&&work.ConsumptionFloor==11&&
            work.Used+uint64(work.Remaining)==100007))return false;
        bool triangle=false,tensorLeaf=false;
        for(const auto& leaf:contact.Leaves)
        {
            triangle|=leaf.SurfaceIndex==0;tensorLeaf|=leaf.SurfaceIndex==1;
            if(!TestTrue(TEXT("no original tied identity disappears and no excluded identity appears"),leaf.SurfaceIndex==0||leaf.SurfaceIndex==1))return false;
            FIntervalPoint sample;
            if(!ProjectedDistanceDetail::EvaluateFixedUV(surfaces[leaf.SurfaceIndex],leaf.FixedParameter,sample))return false;
            for(int32 d=0;d<3;++d)
            {
                const double actualPoint=d==axis?0.:5.;const double actualNormal=d==axis?double(side):0.;
                if(!TestTrue(TEXT("true foot and radial normal remain enclosed without replacing original sample"),
                    contact.SurfacePoint.Axis[d].Lower<=actualPoint&&contact.SurfacePoint.Axis[d].Upper>=actualPoint&&
                    contact.RadialUnitNormal.Axis[d].Lower==actualNormal&&contact.RadialUnitNormal.Axis[d].Upper==actualNormal&&
                    A::Bits(sample.Axis[d].Lower)==A::Bits(leaf.FixedParameterPoint.Axis[d].Lower)&&
                    A::Bits(sample.Axis[d].Upper)==A::Bits(leaf.FixedParameterPoint.Axis[d].Upper)))return false;
            }
        }
        if(!TestTrue(TEXT("triangle and tensor closest providers are both retained"),triangle&&tensorLeaf))return false;
        for(auto& p:path.ControlPoints)p.Axis[axis]={p.Axis[axis].Lower-1.e-8,p.Axis[axis].Upper+1.e-8,true};
        FDistanceBudget uncertain{100000};FTensorSphereContactEnclosure family;TArray<uint8> fd;
        if(!TestTrue(TEXT("all fixed linear endpoint realizations share a certified signed crossing"),D::PrepareCoplanar(surfaces,path,1.,uncertain,1.e-5,1.e-4,1.e-4,family,fd)==E::Proved&&
            family.LowerTime<.5&&family.UpperTime>.5))return false;
        FDistanceBudget low{4106,3,11};FTensorSphereContactEnclosure absent;TArray<uint8> empty;
        if(!TestTrue(TEXT("optional4096 reserve is not drawn from protected caller floor"),D::PrepareCoplanar(surfaces,path,1.,low,1.e-5,1.e-4,1.e-4,absent,empty)==E::NotApplicable&&low.Used==3&&low.Remaining==4106&&empty.IsEmpty()))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDirectCoplanarRefusalTest,
    "IAmSpeed.AnalyticWorld.DirectCoplanarContact.CoverageAndOtherFamiliesFallback",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDirectCoplanarRefusalTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace D=DirectCurveContact;namespace C=DirectCoplanarTests;using E=TimeDetail::EMonotoneCurveTimeSeed;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> tensor(TEXT("p.IAmSpeed.AnalyticWorld.TensorPlanarFamily"),1,FName(TEXT("DirectCoplanarRefusalTensor")));
    TArray<FIntervalTensorSurface> surfaces{C::Patch(2)};FIntervalBezierCenterPath path;
    if(!CenterDetail::Linear(FVector3d(-5,5,2),FVector3d(25,5,0),path))return false;
    FDistanceBudget work{100000};FTensorSphereContactEnclosure out;TArray<uint8> disposition;
    if(!TestTrue(TEXT("foot inside only at crossing cannot replace whole finite coverage"),D::PrepareCoplanar(surfaces,path,1.,work,1.e-5,1.e-4,1.e-4,out,disposition)==E::NotApplicable&&out.Leaves.IsEmpty()&&disposition.IsEmpty()))return false;
    if(!C::Path(2,1,path))return false;
    surfaces.Add(C::Patch(2,0,.5));
    if(!TestTrue(TEXT("another height can contact earlier and must not be ignored"),D::PrepareCoplanar(surfaces,path,1.,work,1.e-5,1.e-4,1.e-4,out,disposition)==E::NotApplicable&&out.Leaves.IsEmpty()&&disposition.IsEmpty()))return false;
    surfaces={C::Patch(2)};surfaces[0].ControlPoints[8].Axis[2]=Interval::Exact(.1);
    if(!TestTrue(TEXT("one noncoplanar original coefficient refuses exact plane proof"),D::PrepareCoplanar(surfaces,path,1.,work,1.e-5,1.e-4,1.e-4,out,disposition)==E::NotApplicable))return false;
    surfaces={C::Triangle(2),C::Patch(2)};
    for(int32 v=0;v<6;++v)surfaces[1].ControlPoints[3*6+v].Axis[0]=Interval::Exact(-2);
    if(!TestTrue(TEXT("coverage by another member does not authorize omitting an unresolved folded chart"),D::PrepareCoplanar(surfaces,path,1.,work,1.e-5,1.e-4,1.e-4,out,disposition)==E::NotApplicable&&disposition.IsEmpty()))return false;
    surfaces={C::Patch(2)};surfaces[0].ControlPoints[8].Axis[1].Known=false;
    if(!TestTrue(TEXT("unknown input is an arithmetic refusal, not a cached coverage"),D::PrepareCoplanar(surfaces,path,1.,work,1.e-5,1.e-4,1.e-4,out,disposition)==E::Arithmetic))return false;
    TArray<FIntervalTensorSurface> repeated;for(int32 i=0;i<64;++i)repeated.Add(C::Patch(2));
    FDistanceBudget exhausted{10000,19,13};out.Status=ETensorSphereContactStatus::FirstContactEnclosed;out.Leaves.AddDefaulted();
    if(!TestTrue(TEXT("exhausted optional proof preserves its debit and clears stale contact before fallback"),
        D::PrepareCoplanar(repeated,path,1.,exhausted,1.e-5,1.e-4,1.e-4,out,disposition)==E::NotApplicable&&
        exhausted.Used==19+4096&&exhausted.Remaining==10000-4096&&exhausted.ConsumptionFloor==13&&
        out.Status==ETensorSphereContactStatus::Unknown&&out.Leaves.IsEmpty()&&disposition.IsEmpty()))return false;
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDirectCoplanarConsumerTest,
    "IAmSpeed.AnalyticWorld.DirectCoplanarContact.AllProviderDispositionAndContinuation",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDirectCoplanarConsumerTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace D=DirectCurveContact;namespace C=DirectCoplanarTests;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> tensor(TEXT("p.IAmSpeed.AnalyticWorld.TensorPlanarFamily"),1,FName(TEXT("DirectCoplanarConsumerTensor")));
    FScopedCVar<int32> flag(TEXT("p.IAmSpeed.AnalyticWorld.DirectCoplanarFamilyContact"),1,FName(TEXT("DirectCoplanarConsumer")));
    FScopedCVar<int32> planar(TEXT("p.IAmSpeed.AnalyticWorld.PlanarFamilyTimeShortcut"),1,FName(TEXT("DirectCoplanarConsumerPlanar")));
    TArray<FIntervalTensorSurface> surfaces{C::Triangle(2),C::Patch(2),C::Patch(2,50)};FIntervalBezierCenterPath path;if(!C::Path(2,1,path))return false;
    FDistanceBudget work{1000000,17,13};int32 calls=0;uint64 before=17;bool valid=true;double lo=0,hi=1;
    const auto callback=[&](const FTensorSphereContactEnclosure& candidate)
    {
        ++calls;bool zero=false,one=false;
        for(const auto& leaf:candidate.Leaves){zero|=leaf.SurfaceIndex==0;one|=leaf.SurfaceIndex==1;valid&=leaf.SurfaceIndex==0||leaf.SurfaceIndex==1;}
        valid&=zero&&one&&candidate.LowerTime>=lo&&candidate.UpperTime<=hi&&candidate.LowerTime<=.5&&candidate.UpperTime>=.5&&work.Used>before&&work.ConsumptionFloor==13;
        lo=candidate.LowerTime;hi=candidate.UpperTime;before=work.Used;return calls==1?EContactEnclosureDecision::Refine:EContactEnclosureDecision::Accept;
    };
    const FContactEnclosureFilter filter(callback);
    const auto result=BoundTensorSphereFirstContact(surfaces,path,1.,work,1.e-5,1.e-8,1024,1.e-3,1.e-3,5,&filter);
    if(!TestTrue(TEXT("all tied providers survive one paid retained generic continuation"),valid&&calls==2&&result.Status==ETensorSphereContactStatus::FirstContactEnclosed&&
        result.ConsumerWidthRefinements==1&&result.CallInitialUsed==17&&work.Used+uint64(work.Remaining)==1000017&&work.ConsumptionFloor==13))return false;
    FDistanceBudget rejected{1000000};int32 rejectCalls=0;
    const auto refuse=[&](const FTensorSphereContactEnclosure&){++rejectCalls;return EContactEnclosureDecision::Refuse;};const FContactEnclosureFilter refusal(refuse);
    const auto no=BoundTensorSphereFirstContact(surfaces,path,1.,rejected,1.e-5,1.e-8,1024,1.e-3,1.e-3,5,&refusal);
    if(!TestTrue(TEXT("consumer identity/material refusal exposes no provisional plane contact"),rejectCalls==1&&no.Status==ETensorSphereContactStatus::Unknown&&no.Failure==ETensorSphereContactFailure::ConsumerRefusal&&no.Leaves.IsEmpty()))return false;
    FDistanceBudget preparedWork{100000};FTensorSphereContactEnclosure direct;TArray<uint8> disposition;
    if(D::PrepareCoplanar(surfaces,path,1.,preparedWork,1.e-5,1.e-3,1.e-3,direct,disposition)!=TimeDetail::EMonotoneCurveTimeSeed::Proved)return false;
    auto generic=direct;auto forged=direct.Leaves[0];forged.SurfaceIndex=2;generic.Leaves.Add(forged);FTensorSphereContactEnclosure intersected;
    if(!TestTrue(TEXT("a provider outside RootFoot cannot be reintroduced solely by an overlapping generic point box"),D::IntersectContact(generic,direct,intersected,preparedWork,MakeArrayView(disposition))&&intersected.Leaves.Num()==direct.Leaves.Num()))return false;
    disposition[2]=2;
    if(!TestTrue(TEXT("unresolved disposition is not silently treated as an excluded source"),!D::IntersectContact(generic,direct,intersected,preparedWork,MakeArrayView(disposition))))return false;
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDirectCoplanarCapturedChartTest,
    "IAmSpeed.AnalyticWorld.DirectCoplanarContact.CapturedFloor2943RegularChart",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDirectCoplanarCapturedChartTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;
    namespace D=DirectCurveContact;namespace M=FixedWheelProbeMath;namespace A=::AnalyticCertified;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        FIntervalTensorSurface triangle;triangle.DegreeU=triangle.DegreeV=1;
        triangle.ControlPoints={Detail::Point(FVector3d(0,3760,0)),Detail::Point(FVector3d(4910,0,0)),
            Detail::Point(FVector3d(4910,960,0)),Detail::Point(FVector3d(4910,960,0))};
        // Exact provider2943 bits in canonical packet090d4115, no nominal fit.
        triangle.ControlPoints[1].Axis[1]={A::FromBits(0x40a21e902bcf92f5ULL),A::FromBits(0x40a21e902bcf92f6ULL),true};
        TArray<FIntervalTensorSurface> surfaces{triangle};FIndexedIntervalUnionPtr scene;FDistanceBudget build{4096};
        if(!TestTrue(TEXT("captured finite triangle builds"),FIndexedIntervalUnion::Build(surfaces,build,scene)))return false;
        FTensorSphereContactLeaf deep;deep.SurfaceIndex=0;
        const double scale=281474976710656.;
        const uint64 iu=uint64(.0014047998018530275*scale),iv=uint64(.004263103465335595*scale);
        deep.MinU=double(iu)/scale;deep.MaxU=double(iu+1)/scale;deep.MinV=double(iv)/scale;deep.MaxV=double(iv+1)/scale;
        FDistanceBudget oldRank{4096};FIntervalPoint chart;
        if(!TestFalse(TEXT("captured overly deep position chart loses rank by interval cancellation"),M::ProveRegularFaceLeaf(scene,deep,oldRank,chart)))return false;
        FIntervalBezierCenterPath path;
        if(!CenterDetail::Linear(FVector3d(27.800000000000569,3750.0086394172286,22.472011866782204),
            FVector3d(27.800000000000459,3749.8269614126375,-1.5373009949711207),path))return false;
        FDistanceBudget work{100000};FTensorSphereContactEnclosure contact;TArray<uint8> disposition;
        const auto result=D::PrepareCoplanar(surfaces,path,12.510000228881836,work,3.2295819783073235e-6,
            .001250028610229492,.00012500000593718141,contact,disposition);
        if(!TestTrue(TEXT("captured first contact is prepared with finite regular original charts"),
            result==TimeDetail::EMonotoneCurveTimeSeed::Proved&&contact.Status==ETensorSphereContactStatus::FirstContactEnclosed&&
            contact.LowerTime<=.41492281329591013&&contact.UpperTime>=.41492281329591013&&
            disposition.Num()==1&&disposition[0]==1&&!contact.Leaves.IsEmpty()&&work.Used<=4096))return false;
        for(const auto& leaf:contact.Leaves)
        {
            FReferenceDyadicCell u,v;FDistanceBudget rank{4096};
            if(!TestTrue(TEXT("each published leaf passes the actual native face-rank consumer before any physical acceptance"),
                M::DecodeLeafDyadic(leaf.MinU,leaf.MaxU,u)&&M::DecodeLeafDyadic(leaf.MinV,leaf.MaxV,v)&&u.Depth<=32&&v.Depth<=32&&
                M::ProveRegularFaceLeaf(scene,leaf,rank,chart)))return false;
            FDistanceBudget noWork{0};
            if(!TestFalse(TEXT("prepublication rank never survives denied work"),D::ProveCoplanarChartBeforePublication(triangle,leaf,noWork)))return false;
        }
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
#endif


#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Analytic::Certified::ImmutablePlanarRankTests
{
static FIntervalTensorSurface Triangle()
{
    FIntervalTensorSurface s;s.DegreeU=s.DegreeV=1;
    s.ControlPoints={Detail::Point(FVector3d(0,3760,0)),Detail::Point(FVector3d(4910,0,0)),
        Detail::Point(FVector3d(4910,960,0)),Detail::Point(FVector3d(4910,960,0))};
    s.ControlPoints[1].Axis[1]={AnalyticCertified::FromBits(0x40a21e902bcf92f5ULL),AnalyticCertified::FromBits(0x40a21e902bcf92f6ULL),true};return s;
}
static FIntervalTensorSurface Patch(bool folded=false)
{
    FIntervalTensorSurface s;s.DegreeU=s.DegreeV=5;
    for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)
    {
        const double x=folded?double(u*(u-1)-4*u+5):2.*u+.02*v;
        s.ControlPoints.Add(Detail::Point(FVector3d(x,4.*v,3.)));
    }
    return s;
}
static FTensorSphereContactLeaf Leaf(int32 depth,double u,double v)
{
    const double count=double(uint64(1)<<depth);const uint64 i=uint64(u*count),j=uint64(v*count);
    FTensorSphereContactLeaf leaf;leaf.SurfaceIndex=0;leaf.MinU=double(i)/count;leaf.MaxU=double(i+1)/count;
    leaf.MinV=double(j)/count;leaf.MaxV=double(j+1)/count;return leaf;
}
static bool Same(const FIntervalPoint& a,const FIntervalPoint& b)
{for(int32 d=0;d<3;++d)if(a.Axis[d].Known!=b.Axis[d].Known||AnalyticCertified::Bits(a.Axis[d].Lower)!=AnalyticCertified::Bits(b.Axis[d].Lower)||AnalyticCertified::Bits(a.Axis[d].Upper)!=AnalyticCertified::Bits(b.Axis[d].Upper))return false;return true;}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FImmutablePlanarRankProofTest,
    "IAmSpeed.AnalyticWorld.ImmutablePlanarFaceRank.OriginalJacobiansAndFallback",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FImmutablePlanarRankProofTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;
    namespace Q=ImmutablePlanarFaceRank;namespace X=ImmutablePlanarRankTests;namespace M=FixedWheelProbeMath;namespace A=::AnalyticCertified;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.ImmutablePlanarFaceRank"),1,FName(TEXT("ImmutablePlanarRankProof")));
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);Q::Cache()={};
        for(bool bilinear:{true,false})
        {
            const auto surface=bilinear?X::Triangle():X::Patch();TArray<FIntervalTensorSurface> surfaces{surface};
            FIndexedIntervalUnionPtr scene;FDistanceBudget build{4096};if(!FIndexedIntervalUnion::Build(surfaces,build,scene))return false;
            auto leaf=X::Leaf(48,bilinear?.0014047998018530275:.37,bilinear?.004263103465335595:.61);
            FIntervalPoint normal;FDistanceBudget work{10000};
            if(!TestTrue(TEXT("original bilinear/global Jacobian proves the same deep regular chart without position cancellation"),
                ProveRegularFaceLeafWithImmutablePlanarRank(scene,leaf,work,normal)&&normal.Axis[2].Lower>0))return false;
            const auto misses=Q::Cache().Misses,hits=Q::Cache().Hits;FDistanceBudget again{10000};FIntervalPoint repeated;
            if(!TestTrue(TEXT("immutable coefficients reused while leaf and account remain fresh"),
                ProveRegularFaceLeafWithImmutablePlanarRank(scene,leaf,again,repeated)&&X::Same(normal,repeated)&&
                Q::Cache().Misses==misses&&Q::Cache().Hits==hits+1&&work.Used==again.Used))return false;
            // A nearly exhausted caller retains the exact original proof and debit.
            leaf=X::Leaf(8,bilinear?.21:.37,bilinear?.31:.61);FDistanceBudget old{10000};FIntervalPoint oldNormal;
            if(!M::ProveRegularFaceLeaf(scene,leaf,old,oldNormal))return false;
            FDistanceBudget exact{int32(old.Used)+17,3,17};FIntervalPoint fallback;
            if(!TestTrue(TEXT("optional reserve never spends the complete original fallback allowance"),
                ProveRegularFaceLeafWithImmutablePlanarRank(scene,leaf,exact,fallback)&&X::Same(oldNormal,fallback)&&
                exact.Used==old.Used+3&&exact.Remaining==17&&exact.ConsumptionFloor==17))return false;
        }
        auto folded=X::Patch(true);TArray<FIntervalTensorSurface> shapes{folded};FIndexedIntervalUnionPtr foldScene;FDistanceBudget create{4096};
        if(!FIndexedIntervalUnion::Build(shapes,create,foldScene))return false;
        auto local=X::Leaf(4,.19,.31);FIntervalPoint a,b;FDistanceBudget old{10000},fresh{10000};
        if(!TestTrue(TEXT("globally singular chart keeps the old locally regular proof"),M::ProveRegularFaceLeaf(foldScene,local,old,a)&&
            ProveRegularFaceLeafWithImmutablePlanarRank(foldScene,local,fresh,b)&&X::Same(a,b)&&fresh.Used>old.Used))return false;
        const auto singular=X::Leaf(4,.5,.31);FDistanceBudget denied{10000};
        if(!TestFalse(TEXT("true rank collapse cannot borrow a global certificate"),ProveRegularFaceLeafWithImmutablePlanarRank(foldScene,singular,denied,b)))return false;
        auto triangle=X::Triangle();
        // All four coefficient boxes vary independently; no CP2==CP3 identity.
        for(auto& cp:triangle.ControlPoints)for(int32 d=0;d<2;++d)
            cp.Axis[d]={cp.Axis[d].Lower-1.e-6,cp.Axis[d].Upper+1.e-6,true};
        FDistanceBudget familyWork{128};FIntervalPoint familyNormal;
        if(!TestTrue(TEXT("bilinear determinant formula is universal for independent coefficients"),
            Q::Try(triangle,X::Leaf(8,.21,.31),familyWork,familyNormal)&&familyNormal.Axis[2].Lower>0&&familyWork.Used<=128))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FImmutablePlanarRankCacheTest,
    "IAmSpeed.AnalyticWorld.ImmutablePlanarFaceRank.ExactIdentityModesAndFailures",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FImmutablePlanarRankCacheTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;
    namespace Q=ImmutablePlanarFaceRank;namespace X=ImmutablePlanarRankTests;namespace A=::AnalyticCertified;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x807f))|flush);Q::Cache()={};auto triangle=X::Triangle();const auto leaf=X::Leaf(8,.21,.31);
        FIntervalPoint normal;FDistanceBudget first{128};if(!Q::Try(triangle,leaf,first,normal))return false;
        auto changed=triangle;changed.ControlPoints[1].Axis[0].Upper=A::FromBits(A::Bits(changed.ControlPoints[1].Axis[0].Upper)+1);
        const auto misses=Q::Cache().Misses;FDistanceBudget other{128};
        if(!TestTrue(TEXT("one changed true coefficient cannot reuse an old property entry"),Q::Try(changed,leaf,other,normal)&&Q::Cache().Misses==misses+1))return false;
        // Test full input identity using the same helper used by the cache.
        bool key=true;
        for(int32 i=0;i<triangle.ControlPoints.Num();++i)for(int32 d=0;d<3;++d)for(int32 endpoint=0;endpoint<2;++endpoint)
        {
            auto value=triangle;auto& bound=endpoint?value.ControlPoints[i].Axis[d].Upper:value.ControlPoints[i].Axis[d].Lower;
            bound=A::FromBits(A::Bits(bound)^uint64(1));key&=!Q::Properties::Same(triangle,value);
        }
        if(!TestTrue(TEXT("every endpoint bit participates in equality"),key))return false;
        auto bad=triangle;bad.ControlPoints[0].Axis[0].Known=false;FDistanceBudget invalid{128};const auto oldHits=Q::Cache().Hits;
        if(!TestTrue(TEXT("unknown CP cannot become a property-cache hit"),!Q::Try(bad,leaf,invalid,normal)&&Q::Cache().Hits==oldHits&&!Detail::ValidPoint(normal)))return false;
        auto nonplane=triangle;nonplane.ControlPoints[1].Axis[2]=Interval::Exact(1.);FDistanceBudget unsupported{128};
        if(!TestFalse(TEXT("nonplanar models are not admitted by this bounded specialization"),Q::Try(nonplane,leaf,unsupported,normal)))return false;
        FDistanceBudget noWork{0};if(!TestTrue(TEXT("denied account has no rank or debit"),!Q::Try(triangle,leaf,noWork,normal)&&noWork.Used==0&&!Detail::ValidPoint(normal)))return false;
        Q::Cache()={};VectorSetControlRegister((saved&~uint32(0x807f))|flush);FDistanceBudget build{128};
        if(!Q::Try(triangle,leaf,build,normal))return false;
        const uint32 sticky=VectorGetControlRegister()&0x3f;
        if(!TestTrue(TEXT("captured irrational-coefficient fixture sets sticky arithmetic flags"),sticky!=0))return false;
        VectorSetControlRegister(VectorGetControlRegister()&~uint32(0x3f));const auto prior=Q::Cache().Misses;FDistanceBudget replay{128};
        if(!TestTrue(TEXT("cleared sticky flags cause original coefficient recomputation"),Q::Try(triangle,leaf,replay,normal)&&Q::Cache().Misses==prior+1))return false;
        const auto modes=Q::Cache().Misses;VectorSetControlRegister(VectorGetControlRegister()^uint32(0x40));FDistanceBudget mode{128};
        if(!TestTrue(TEXT("a changed FTZ/DAZ mode never borrows prior properties"),Q::Try(triangle,leaf,mode,normal)&&Q::Cache().Misses==modes+1))return false;
        const uint32 good=VectorGetControlRegister();VectorSetControlRegister(good|uint32(0x2000));bool refused=false;
        try{FDistanceBudget rounding{128};Q::Try(triangle,leaf,rounding,normal);}catch(const FAnalyticNumericUnknown&){refused=true;}
        VectorSetControlRegister(good);if(!TestTrue(TEXT("unsupported rounding remains terminal even with warm metadata"),refused))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
#endif


namespace Speed::Analytic::Certified::GroupedBoxInteriorWitness
{
namespace Private
{
namespace A=::AnalyticCertified;
static TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxInteriorWitnessReject"),0,
    TEXT("Default OFF. One original-UV interior certificate stops only an eight-group coarse separation attempt; never supplies contact or clearance."));
constexpr int32 NewtonSteps=4,MaximumAllowance=512;
struct FCounts {std::atomic<uint64> Calls{0},Finished{0},Applicable{0},Evaluations{0},Rejected{0},Invalid{0},NotApplicable{0},Used{0},Ticks{0},Group[8]{};};
static FCounts Counts;
struct FJet {FVector3d P=FVector3d::ZeroVector,U=FVector3d::ZeroVector,V=FVector3d::ZeroVector;};
static FJet Lerp(const FJet& a,const FJet& b,double t,bool alongU)
{
    FJet r;r.P=a.P+(b.P-a.P)*t;r.U=a.U+(b.U-a.U)*t;r.V=a.V+(b.V-a.V)*t;
    if(alongU)r.U+=b.P-a.P;else r.V+=b.P-a.P;return r;
}
static FJet EvaluateScalar(const FVector3d* controls,int32 du,int32 dv,FVector2d uv)
{
    FJet rows[6],work[6];
    for(int32 u=0;u<=du;++u)
    {
        for(int32 v=0;v<=dv;++v){work[v]={};work[v].P=controls[u*(dv+1)+v];}
        for(int32 n=dv;n>0;--n)for(int32 v=0;v<n;++v)work[v]=Lerp(work[v],work[v+1],uv.Y,false);
        rows[u]=work[0];
    }
    for(int32 n=du;n>0;--n)for(int32 u=0;u<n;++u)rows[u]=Lerp(rows[u],rows[u+1],uv.X,true);
    return rows[0];
}
static int32 EvaluationCost(const FIntervalTensorSurface& s)
{return s.ControlPoints.Num()+(s.DegreeU+1)*s.DegreeV*(s.DegreeV+1)/2+s.DegreeU*(s.DegreeU+1)/2;}
static int32 Allowance(const FIntervalTensorSurface& s,const FGroupedBoxVolumeBounds& v)
{
    int32 corners=0;for(const auto& g:v.Groups)corners+=g.Corners.Num();
    return s.ControlPoints.Num()+corners+NewtonSteps+EvaluationCost(s)+v.Axes.Num()+v.Groups.Num();
}
static bool Consume(FDistanceBudget& work,int32 count)
{for(int32 i=0;i<count;++i)if(!work.Consume())return false;return true;}
static bool Propose(const FIntervalTensorSurface& s,const FGroupedBoxVolumeBounds& volume,FDistanceBudget& work,FVector2d& uv,uint32 activeGroups)
{
    FVector3d controls[36],center=FVector3d::ZeroVector;double best=std::numeric_limits<double>::max();bool found=false;
    for(int32 i=0;i<s.ControlPoints.Num();++i)
    {
        if(!work.Consume()||!Detail::ValidPoint(s.ControlPoints[i]))return false;
        for(int32 d=0;d<3;++d)controls[i][d]=.5*s.ControlPoints[i].Axis[d].Lower+.5*s.ControlPoints[i].Axis[d].Upper;
        if(controls[i].ContainsNaN())return false;
    }
    for(int32 g=0;g<volume.Groups.Num();++g)
    {
        if(!(activeGroups&(1u<<g)))continue;const auto& group=volume.Groups[g];
        FIntervalPoint hull;
        for(const auto& p:group.Corners)
        {if(!work.Consume()||!Detail::ValidPoint(p))return false;hull=Detail::ValidPoint(hull)?TimeDetail::Hull(hull,p):p;}
        if(!Detail::ValidPoint(hull))return false;FVector3d target;
        for(int32 d=0;d<3;++d)target[d]=.5*hull.Axis[d].Lower+.5*hull.Axis[d].Upper;
        if(target.ContainsNaN())return false;
        for(int32 i=0;i<s.ControlPoints.Num();++i)
        {
            const double distance=(controls[i]-target).SizeSquared();
            if(A::Finite(distance)&&distance<best)
            {found=true;best=distance;center=target;uv=FVector2d(s.DegreeU?double(i/(s.DegreeV+1))/s.DegreeU:.5,s.DegreeV?double(i%(s.DegreeV+1))/s.DegreeV:.5);}
        }
    }
    if(!found)return false;
    // Scalar approximation proposes ONLY a fixed parameter. No scalar point,
    // derivative, error estimate or distance is admitted by the proof below.
    for(int32 step=0;step<NewtonSteps;++step)
    {
        if(!work.Consume())return false;const auto q=EvaluateScalar(controls,s.DegreeU,s.DegreeV,uv);
        if(q.P.ContainsNaN()||q.U.ContainsNaN()||q.V.ContainsNaN())break;
        const auto residual=q.P-center;const double a=q.U|q.U,b=q.U|q.V,c=q.V|q.V,d=q.U|residual,e=q.V|residual;
        const double determinant=a*c-b*b;double su=0,sv=0;
        if(A::Finite(determinant)&&determinant>0){su=(c*d-b*e)/determinant;sv=(a*e-b*d)/determinant;}
        else if(A::Finite(a)&&a>0&&A::Finite(d))su=d/a;
        else if(A::Finite(c)&&c>0&&A::Finite(e))sv=e/c;
        else break;
        if(!A::Finite(su)||!A::Finite(sv))break;
        uv.X=FMath::Clamp(uv.X-su,0.,1.);uv.Y=FMath::Clamp(uv.Y-sv,0.,1.);
    }
    return A::Finite(uv.X)&&A::Finite(uv.Y)&&uv.X>=0&&uv.X<=1&&uv.Y>=0&&uv.Y<=1;
}
static EResult AtUV(const FIntervalTensorSurface& surface,const FGroupedBoxVolumeBounds& volume,FVector2d uv,
    FDistanceBudget& work,int32& containedGroup,bool observe)
{
    containedGroup=INDEX_NONE;
    // Never clamp a proof parameter. A scalar proposal outside the original
    // closed domain has no proof and must fall back.
    if(!A::Finite(uv.X)||!A::Finite(uv.Y)||uv.X<0||uv.X>1||uv.Y<0||uv.Y>1)return EResult::NotApplicable;
    if(!Consume(work,EvaluationCost(surface)))return EResult::NotApplicable;
    FIntervalPoint point;if(observe)++Counts.Evaluations;
    if(!ProjectedDistanceDetail::EvaluateFixedUV(surface,uv,point))return EResult::NotApplicable;
    TArray<Interval,TInlineAllocator<12>> slabs;
    for(const auto& axis:volume.Axes)
    {
        if(!work.Consume())return EResult::NotApplicable;
        const auto projected=GroupedBoxDetail::Project(point,axis);if(!A::Valid(projected))return EResult::NotApplicable;slabs.Add(projected);
    }
    for(int32 g=0;g<volume.Groups.Num();++g)
    {
        if(!work.Consume())return EResult::NotApplicable;bool inside=true;
        for(int32 a=0;a<volume.Axes.Num();++a)
        {
            const auto body=volume.Slabs[g*volume.Axes.Num()+a];if(!A::Valid(body))return EResult::Invalid;
            inside&=body.Lower<slabs[a].Lower&&slabs[a].Upper<body.Upper;
        }
        if(inside){containedGroup=g;return EResult::Rejected;}
    }
    return EResult::NotApplicable;
}
static EResult Probe(const FIntervalTensorSurface& s,const FGroupedBoxVolumeBounds& volume,FDistanceBudget& work,int32& group,bool observe,uint32 activeGroups)
{
    group=INDEX_NONE;
    if(volume.Groups.Num()!=8||volume.Axes.IsEmpty()||volume.Axes.Num()>12||volume.Slabs.Num()!=volume.Groups.Num()*volume.Axes.Num()||
        s.DegreeU<0||s.DegreeU>5||s.DegreeV<0||s.DegreeV>5||s.ControlPoints.Num()!=(s.DegreeU+1)*(s.DegreeV+1))return EResult::NotApplicable;
    const uint32 full=(1u<<volume.Groups.Num())-1u;if(activeGroups==MAX_uint32)activeGroups=full;
    if(!activeGroups||(activeGroups&~full))return EResult::NotApplicable;
    for(const auto& g:volume.Groups)if(g.Corners.Num()<8||g.Corners.Num()>32)return EResult::NotApplicable;
    const int32 allowance=Allowance(s,volume),fallback=volume.Axes.Num()+volume.Groups.Num()+2;
    if(allowance<=0||allowance>MaximumAllowance||work.ConsumptionFloor<0||work.Remaining<work.ConsumptionFloor||
        int64(work.Remaining)-work.ConsumptionFloor<int64(allowance)+fallback||work.Used>MAX_uint64-uint64(allowance))return EResult::NotApplicable;
    FScopedDistanceBudgetFloor floor(work,work.Remaining-allowance);if(!floor.IsValid())return EResult::NotApplicable;
    if(observe)++Counts.Applicable;FVector2d uv;
    if(!Propose(s,volume,work,uv,activeGroups))return EResult::NotApplicable;
    return AtUV(s,volume,uv,work,group,observe);
}
static void Publish()
{
    const auto& x=Counts;
    UE_LOG(LogTemp,Display,TEXT("[GroupedBoxInteriorWitness] Scope=ProgramPrefixAtPreExit DefaultOFF=1 NoPhysicalContactClaim=1 OneEvaluationPerProvider=1 AfterRootAxesOnly=1 CoarseGroups=8 Calls=%llu Finished=%llu Applicable=%llu Evaluations=%llu Rejected=%llu Invalid=%llu NotApplicable=%llu BudgetUsed=%llu Ticks=%llu SecondsPerCycle=%.17g"),
        x.Calls.load(),x.Finished.load(),x.Applicable.load(),x.Evaluations.load(),x.Rejected.load(),x.Invalid.load(),x.NotApplicable.load(),x.Used.load(),x.Ticks.load(),FPlatformTime::GetSecondsPerCycle64());
    for(int32 i=0;i<8;++i)if(x.Group[i].load())UE_LOG(LogTemp,Display,TEXT("[GroupedBoxInteriorWitnessGroup] Group=%d Rejected=%llu"),i,x.Group[i].load());
}
struct FRegister{FRegister(){FCoreDelegates::OnPreExit.AddStatic(&Publish);}};static FRegister Register;
}
bool RuntimeEnabled(){return Private::Enabled.GetValueOnAnyThread()==1;}
EResult TryReject(const FIntervalTensorSurface& surface,const FGroupedBoxVolumeBounds& volume,FDistanceBudget& budget,uint32 activeGroups)
{
    const bool observe=StrongConvexCurveMinimum::Observation::Enabled();const uint64 start=observe?FPlatformTime::Cycles64():0,before=budget.Used;
    if(observe)++Private::Counts.Calls;
    const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    int32 group=INDEX_NONE;auto result=EResult::Invalid;
    if(numeric.Status==EAnalyticNumericStatus::Complete)result=Private::Probe(surface,volume,budget,group,observe,activeGroups);
    const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=EAnalyticNumericStatus::Complete||after.Status!=EAnalyticNumericStatus::Complete||numeric.ControlFingerprint()!=after.ControlFingerprint())result=EResult::Invalid;
    if(observe)
    {
        ++Private::Counts.Finished;Private::Counts.Used.fetch_add(budget.Used-before);Private::Counts.Ticks.fetch_add(FPlatformTime::Cycles64()-start);
        if(result==EResult::Rejected){++Private::Counts.Rejected;if(group>=0&&group<8)++Private::Counts.Group[group];}
        else if(result==EResult::Invalid)++Private::Counts.Invalid;else ++Private::Counts.NotApplicable;
    }
    return result;
}
}


#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Analytic::Certified::GroupedBoxInteriorWitnessTests
{
static FIntervalTensorSurface Patch(bool quintic=false,double height=0,double uncertainty=0,bool curved=false)
{
    FIntervalTensorSurface s;s.DegreeU=s.DegreeV=quintic?5:1;const int32 degree=s.DegreeU;
    for(int32 u=0;u<=degree;++u)for(int32 v=0;v<=degree;++v)
    {
        const double z=curved?3.-4.*v/5.+v*(v-1)/5.:height;
        auto p=Detail::Point(FVector3d(-1024.+2048.*u/degree,-2.+4.*v/degree,z));
        p.Axis[2].Lower-=uncertainty;p.Axis[2].Upper+=uncertainty;s.ControlPoints.Add(p);
    }
    return s;
}
static bool Volume(double height,FGroupedBoxVolumeBounds& result,int32 groups=8,bool lastOnly=false)
{
    TArray<FBoxVolumeGroup> values;FIntervalPoint hull;
    for(int32 g=0;g<groups;++g)
    {
        FBoxVolumeGroup group;const double y=lastOnly&&g+1<groups?100.+g:0.;
        for(int32 i=0;i<8;++i)
        {
            const auto p=Detail::Point(FVector3d(i&1?.5:-.5,y+(i&2?.25:-.25),height+(i&4?.1:-.1)));
            group.Corners.Add(p);hull=Detail::ValidPoint(hull)?TimeDetail::Hull(hull,p):p;
        }
        values.Add(MoveTemp(group));
    }
    const TArray<FVector3d> axes{FVector3d(1,0,0),FVector3d(0,1,0),FVector3d(0,0,1),FVector3d(1,1,0),FVector3d(0,1,1),FVector3d(1,0,1)};
    FDistanceBudget work{10000};return CloseGroupedBoxVolumeBounds(hull,values,axes,work,result);
}
static bool Partition(const FIntervalTensorSurface& surface,const FGroupedBoxVolumeBounds& volume,FReferenceBoundsCandidateSetPtr& result)
{
    FIndexedIntervalUnionPtr scene;FDistanceBudget work{10000};
    return FIndexedIntervalUnion::Build(MakeArrayView(&surface,1),work,scene)&&FReferenceBoundsCandidateSet::Select(scene,volume.BroadHull,work,result);
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGroupedBoxInteriorWitnessGeometryTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.OriginalInteriorWitnessGeometry",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FGroupedBoxInteriorWitnessGeometryTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;
    namespace X=GroupedBoxInteriorWitnessTests;namespace P=GroupedBoxInteriorWitness;namespace A=::AnalyticCertified;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        for(bool quintic:{false,true})
        {
            const auto patch=X::Patch(quintic,0.,1.e-9);FGroupedBoxVolumeBounds volume;
            if(!X::Volume(0.,volume,8,true))return false;
            FDistanceBudget work{10000,19,11};
            if(!TestTrue(TEXT("one original point inside the eighth group stops coarse attempt"),P::TryReject(patch,volume,work)==P::EResult::Rejected&&work.Used>19&&work.ConsumptionFloor==11))return false;
            FDistanceBudget activeWork{10000};
            if(!TestTrue(TEXT("remaining root group directs the sole proposal"),P::TryReject(patch,volume,activeWork,1u<<7)==P::EResult::Rejected))return false;
            FDistanceBudget noActive{10000,13};
            if(!TestTrue(TEXT("empty or foreign active mask cannot generate a witness"),P::TryReject(patch,volume,noActive,0)==P::EResult::NotApplicable&&P::TryReject(patch,volume,noActive,1u<<8)==P::EResult::NotApplicable&&noActive.Used==13))return false;
            bool entire=false;for(const auto& p:patch.ControlPoints)entire|=!GroupedBoxDetail::Within(volume.BroadHull,p);
            if(!TestTrue(TEXT("whole control net is NOT contained; this is a point witness"),entire))return false;
            int32 group=INDEX_NONE;FDistanceBudget exact{10000};
            if(!TestTrue(TEXT("one group suffices; no split of axes between groups"),P::Private::AtUV(patch,volume,FVector2d(.5,.5),exact,group,false)==P::EResult::Rejected&&group==7))return false;
            for(FVector2d invalid:{FVector2d(-.01,.5),FVector2d(.5,1.01),FVector2d(A::FromBits(A::Infinity|1),.5)})
            {
                FDistanceBudget invalidWork{1000,7,3};
                if(!TestTrue(TEXT("out-of-domain proof UV is refused without clamping/debit"),P::Private::AtUV(patch,volume,invalid,invalidWork,group,false)==P::EResult::NotApplicable&&invalidWork.Used==7&&invalidWork.Remaining==1000))return false;
            }
            auto broadFamily=patch;for(auto& cp:broadFamily.ControlPoints)cp.Axis[2]={-.5,.5,true};FDistanceBudget familyWork{10000};
            if(!TestTrue(TEXT("independent CP intervals cannot collapse to the nominal inside sample"),P::TryReject(broadFamily,volume,familyWork)==P::EResult::NotApplicable))return false;
            FIntervalPoint fixed;if(!ProjectedDistanceDetail::EvaluateFixedUV(patch,FVector2d(.5,.5),fixed))return false;
            auto boundary=volume;const auto z=GroupedBoxDetail::Project(fixed,boundary.Axes[2]);
            for(int32 g=0;g<boundary.Groups.Num();++g)boundary.Slabs[g*boundary.Axes.Num()+2].Upper=z.Upper;
            FDistanceBudget boundaryWork{10000};
            if(!TestTrue(TEXT("strict boundary equality cannot supply an interior witness"),P::Private::AtUV(patch,boundary,FVector2d(.5,.5),boundaryWork,group,false)==P::EResult::NotApplicable))return false;
        }
        const auto curve=X::Patch(true,0.,1.e-9,true);FGroupedBoxVolumeBounds volume;
        if(!X::Volume(2.,volume))return false;FDistanceBudget curveWork{10000};
        if(!TestTrue(TEXT("scalar proposal is checked against the original curved5x5 family"),P::TryReject(curve,volume,curveWork)==P::EResult::Rejected))return false;
        const auto outside=X::Patch(true,4.);FDistanceBudget outsideWork{10000};
        if(!TestTrue(TEXT("outside point is never a rejection certificate"),P::TryReject(outside,volume,outsideWork)==P::EResult::NotApplicable))return false;
    }
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGroupedBoxInteriorWitnessBudgetTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.OriginalInteriorWitnessBudgetAndFallback",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FGroupedBoxInteriorWitnessBudgetTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;
    namespace X=GroupedBoxInteriorWitnessTests;namespace P=GroupedBoxInteriorWitness;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const auto patch=X::Patch(true);FGroupedBoxVolumeBounds volume;if(!X::Volume(0.,volume))return false;
    const int32 allowance=P::Private::Allowance(patch,volume),fallback=volume.Axes.Num()+volume.Groups.Num()+2;
    TestTrue(TEXT("finite allowance stays within fixed512 local ceiling"),allowance>0&&allowance<=P::Private::MaximumAllowance);
    FDistanceBudget shortWork{allowance+fallback+11-1,23,11};const auto before=shortWork;
    if(!TestTrue(TEXT("insufficient optional reservation has zero debit"),P::TryReject(patch,volume,shortWork)==P::EResult::NotApplicable&&shortWork.Remaining==before.Remaining&&shortWork.Used==before.Used&&shortWork.ConsumptionFloor==before.ConsumptionFloor))return false;
    FDistanceBudget exact{allowance+fallback+11,23,11};
    if(!TestTrue(TEXT("same account pays attempt while retaining original floor and fallback minimum"),P::TryReject(patch,volume,exact)==P::EResult::Rejected&&exact.Used>23&&exact.Used<=uint64(23+allowance)&&exact.Remaining>=fallback+11&&exact.ConsumptionFloor==11))return false;
    FDistanceBudget overflow{10000,MAX_uint64-uint64(allowance)+1,0};const auto priorUsed=overflow.Used;
    if(!TestTrue(TEXT("counter overflow cannot create paid or unpaid certificate"),P::TryReject(patch,volume,overflow)==P::EResult::NotApplicable&&overflow.Used==priorUsed))return false;
    FGroupedBoxVolumeBounds fine;if(!X::Volume(0.,fine,14))return false;FDistanceBudget fineWork{10000,31,7};
    if(!TestTrue(TEXT("fourteen-group fine path is not probed or charged"),P::TryReject(patch,fine,fineWork)==P::EResult::NotApplicable&&fineWork.Used==31&&fineWork.Remaining==10000))return false;
    FReferenceBoundsCandidateSetPtr partition;if(!X::Partition(patch,volume,partition)||partition->GetCandidateIndices().Num()!=1)return false;
    FDistanceBudget oldWork{4096},newWork{4096};FReferenceBoxVolumeStrictSeparationPtr oldProof,newProof;FGroupedBoxSeparationObservation oldObserve,newObserve;
    const bool oldKnown=FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,oldWork,oldProof,&oldObserve,false,false,false);
    const bool newKnown=FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,newWork,newProof,&newObserve,false,false,true);
    if(!TestTrue(TEXT("original interior witness stops unresolved coarse DFS early with null proof"),!oldKnown&&!newKnown&&!oldProof&&!newProof&&newWork.Used<oldWork.Used&&newObserve.VisitedCells==1))return false;
    // A candidate intersects the union AABB but each actual group is already
    // separated at the root: the optional witness must cost exactly zero.
    auto rootClearVolume=volume;FIntervalPoint rootHull;
    for(int32 g=0;g<rootClearVolume.Groups.Num();++g)for(auto& cp:rootClearVolume.Groups[g].Corners)
    {
        cp.Axis[2]=::AnalyticCertified::Interval::Exact((g&1)?5.:-5.);
        rootHull=Detail::ValidPoint(rootHull)?TimeDetail::Hull(rootHull,cp):cp;
    }
    FGroupedBoxVolumeBounds rootClosed;FDistanceBudget closeWork{10000};
    if(!CloseGroupedBoxVolumeBounds(rootHull,rootClearVolume.Groups,rootClearVolume.Axes,closeWork,rootClosed))return false;
    FReferenceBoundsCandidateSetPtr rootPartition;if(!X::Partition(patch,rootClosed,rootPartition)||rootPartition->GetCandidateIndices().Num()!=1)return false;
    FDistanceBudget rootOff{10000},rootOn{10000};FReferenceBoxVolumeStrictSeparationPtr rootOffProof,rootOnProof;FGroupedBoxSeparationObservation rootOffObs,rootOnObs;
    if(!TestTrue(TEXT("root axes clear bypasses every optional witness operation and charge"),
        FReferenceBoxVolumeStrictSeparation::TryProve(rootPartition,rootClosed,rootOff,rootOffProof,&rootOffObs,false,false,false)&&
        FReferenceBoxVolumeStrictSeparation::TryProve(rootPartition,rootClosed,rootOn,rootOnProof,&rootOnObs,false,false,true)&&
        rootOff.Used==rootOn.Used&&rootOff.Remaining==rootOn.Remaining&&rootOffObs.VisitedCells==1&&rootOnObs.VisitedCells==1))return false;
    // Keep an actual candidate whose broad hull overlaps while all values
    // stay outside each group: a curved patch above the box is clear after
    // original subdivisions. A failed optional point never creates clearance.
    auto clear=X::Patch(true,0.,0.,true);FGroupedBoxVolumeBounds below;if(!X::Volume(1.75,below))return false;
    FReferenceBoundsCandidateSetPtr clearPartition;if(!X::Partition(clear,below,clearPartition)||clearPartition->GetCandidateIndices().Num()!=1)return false;
    FDistanceBudget oldClear{20000},newClear{20000};FReferenceBoxVolumeStrictSeparationPtr oldClearProof,newClearProof;
    const bool a=FReferenceBoxVolumeStrictSeparation::TryProve(clearPartition,below,oldClear,oldClearProof,nullptr,false,false,false);
    const bool b=FReferenceBoxVolumeStrictSeparation::TryProve(clearPartition,below,newClear,newClearProof,nullptr,false,false,true);
    if(!TestTrue(TEXT("unresolved witness uses original clear proof with cumulative paid work"),a&&b&&oldClearProof&&newClearProof&&newClearProof->Matches(clearPartition->GetScene(),below)&&newClear.Used>oldClear.Used))return false;
    const uint32 good=VectorGetControlRegister();VectorSetControlRegister(good|uint32(0x2000));FDistanceBudget unsupported{10000,17};
    const auto denied=P::TryReject(patch,volume,unsupported);VectorSetControlRegister(good);
    TestTrue(TEXT("unsupported numeric context refuses before any proposal"),denied==P::EResult::Invalid&&unsupported.Used==17);
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Analytic::Certified::DirectCurveUnionTests
{
static FIntervalTensorSurface Line(double height=0.,bool vertical=false)
{
    auto s=DirectCurveContactTests::Line();
    for(auto& p:s.ControlPoints)
    {
        const auto x=p.Axis[0];
        if(vertical){p.Axis[0]=Interval::Exact(height);p.Axis[1]=AnalyticCertified::Subtract(AnalyticCertified::Multiply(Interval::Exact(2.),x),Interval::Exact(20.));}
        else p.Axis[1]=Interval::Exact(height);
    }
    return s;
}
static bool AllIndices(const FTensorSphereContactEnclosure& c,int32 index)
{if(c.Leaves.IsEmpty())return false;for(const auto& l:c.Leaves)if(l.SurfaceIndex!=index)return false;return true;}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDirectCurveUnionGeometryTest,
 "IAmSpeed.AnalyticWorld.DirectCurveUnion.CompleteFirstEventAndTies",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDirectCurveUnionGeometryTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;
    namespace D=DirectCurveContact;namespace X=DirectCurveUnionTests;using E=TimeDetail::EMonotoneCurveTimeSeed;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        FIntervalBezierCenterPath path;if(!CenterDetail::Linear(FVector3d(4,3,0),FVector3d(5.6875,-1,0),path))return false;
        for(bool reverse:{false,true})
        {
            TArray<FIntervalTensorSurface> surfaces{X::Line(reverse?-1.:0.),X::Line(reverse?0.:-1.)};
            FDistanceBudget work{1000000,17,13};FTensorSphereContactEnclosure result;TArray<uint8> disposition;
            const auto status=D::PrepareUnion(surfaces,path,1.,work,1.e-5,1.e-3,1.e-3,result,disposition);
            const int32 winner=reverse?1:0;
            if(!TestTrue(TEXT("minimum of two complete first roots keeps original provider order and drops only strictly later"),
                status==E::Proved&&result.Status==ETensorSphereContactStatus::FirstContactEnclosed&&
                result.LowerTime<=.5&&result.UpperTime>=.5&&disposition.Num()==2&&disposition[winner]==1&&disposition[1-winner]==0&&
                X::AllIndices(result,winner)&&work.Used<=17+D::UnionDetail::Allowance&&work.Used+uint64(work.Remaining)==1000017&&work.ConsumptionFloor==13))return false;
            for(const auto& leaf:result.Leaves)
            {
                FIntervalPoint evaluated;if(!ProjectedDistanceDetail::EvaluateFixedUV(surfaces[leaf.SurfaceIndex],leaf.FixedParameter,evaluated))return false;
                if(!TestTrue(TEXT("all representatives retain their ORIGINAL provider and UV evaluation"),DirectCurveContactTests::SamePoint(evaluated,leaf.FixedParameterPoint)))return false;
            }
        }
        TArray<FIntervalTensorSurface> ties{X::Line(),X::Line()};FDistanceBudget tiedWork{1000000};
        FTensorSphereContactEnclosure tied;TArray<uint8> tiedDisposition;
        if(!TestTrue(TEXT("overlapping equal roots keep both IDs without an arbitrary owner tie-break"),
            D::PrepareUnion(ties,path,1.,tiedWork,1.e-5,1.e-3,1.e-3,tied,tiedDisposition)==E::Proved&&
            tiedDisposition.Num()==2&&tiedDisposition[0]==1&&tiedDisposition[1]==1&&tied.Leaves.Num()>=2))return false;
        bool first=false,second=false;for(const auto& leaf:tied.Leaves){first|=leaf.SurfaceIndex==0;second|=leaf.SurfaceIndex==1;}
        if(!TestTrue(TEXT("all tied first providers remain present"),first&&second))return false;
        TArray<FIntervalTensorSurface> axes{X::Line(),X::Line(6.,true)};FDistanceBudget axisWork{1000000};
        FTensorSphereContactEnclosure axisContact;TArray<uint8> axisDisposition;
        if(!TestTrue(TEXT("individual line normals need not share a global normal axis"),
            D::PrepareUnion(axes,path,1.,axisWork,1.e-5,1.e-3,1.e-3,axisContact,axisDisposition)==E::Proved&&
            axisDisposition[0]==1&&axisDisposition[1]==0&&X::AllIndices(axisContact,0)))return false;
        TArray<FIntervalTensorSurface> mixed{MonotoneSeedTests::Parabola(),X::Line(4.)};
        FIntervalBezierCenterPath diagonal;if(!CenterDetail::Linear(FVector3d(6,9,0),FVector3d(10,5,0),diagonal))return false;
        FDistanceBudget mixedWork{1000000};FTensorSphereContactEnclosure curve;TArray<uint8> mixedDisposition;
        const double exact=1.-2./FMath::Sqrt(32.);
        if(!TestTrue(TEXT("original curved quintic enters before the later line without substituting a chord"),
            D::PrepareUnion(mixed,diagonal,2.,mixedWork,1.e-5,1.e-3,1.e-3,curve,mixedDisposition)==E::Proved&&
            curve.LowerTime<=exact&&curve.UpperTime>=exact&&mixedDisposition[0]==1&&mixedDisposition[1]==0))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDirectCurveUnionMissAndRefusalTest,
 "IAmSpeed.AnalyticWorld.DirectCurveUnion.WholeMissUnknownAndBudget",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDirectCurveUnionMissAndRefusalTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;
    namespace D=DirectCurveContact;namespace X=DirectCurveUnionTests;using E=TimeDetail::EMonotoneCurveTimeSeed;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        FIntervalBezierCenterPath path;if(!CenterDetail::Linear(FVector3d(4,3,0),FVector3d(5.6875,-1,0),path))return false;
        TArray<FIntervalTensorSurface> surfaces{X::Line(),X::Line(100.)};
        FDistanceBudget work{1000000};FTensorSphereContactEnclosure contact;TArray<uint8> disposition;
        if(!TestTrue(TEXT("complete whole-path miss is a valid exclusion next to a certified root"),
            D::PrepareUnion(surfaces,path,1.,work,1.e-5,1.e-3,1.e-3,contact,disposition)==E::Proved&&
            disposition[0]==1&&disposition[1]==0&&X::AllIndices(contact,0)))return false;
        surfaces[0]=X::Line(-100.);FDistanceBudget clearWork{1000000};
        if(!TestTrue(TEXT("all providers must be whole-path clear before union NoContact"),
            D::PrepareUnion(surfaces,path,1.,clearWork,1.e-5,1.e-3,1.e-3,contact,disposition)==E::Proved&&
            contact.Status==ETensorSphereContactStatus::NoContactWithProvidedTensors&&contact.Leaves.IsEmpty()&&disposition[0]==0&&disposition[1]==0))return false;
        surfaces[0]=X::Line();surfaces[1]=X::Line();surfaces[1].ControlPoints[2].Axis[2]=Interval::Exact(.1);
        FDistanceBudget unknown{1000000,11};
        if(!TestTrue(TEXT("an unresolved second provider cannot be omitted after the first proof"),
            D::PrepareUnion(surfaces,path,1.,unknown,1.e-5,1.e-3,1.e-3,contact,disposition)==E::NotApplicable&&
            contact.Status==ETensorSphereContactStatus::Unknown&&contact.Leaves.IsEmpty()&&disposition.IsEmpty()&&unknown.Used>11))return false;
        surfaces[1]=X::Line(-1.);FDistanceBudget shortWork{D::UnionDetail::Allowance+12,17,13};
        if(!TestTrue(TEXT("insufficient optional reservation cannot borrow caller floor or spend"),
            D::PrepareUnion(surfaces,path,1.,shortWork,1.e-5,1.e-3,1.e-3,contact,disposition)==E::NotApplicable&&
            shortWork.Used==17&&shortWork.Remaining==D::UnionDetail::Allowance+12&&shortWork.ConsumptionFloor==13))return false;
        surfaces[1].ControlPoints[0].Axis[0].Known=false;FDistanceBudget invalid{1000000};
        if(!TestTrue(TEXT("unknown original coefficient is never converted to a geometric miss"),
            D::PrepareUnion(surfaces,path,1.,invalid,1.e-5,1.e-3,1.e-3,contact,disposition)==E::Arithmetic&&contact.Leaves.IsEmpty()&&disposition.IsEmpty()))return false;
        TArray<FIntervalTensorSurface> tangent{X::Line(),X::Line(100.)};FIntervalBezierCenterPath parallel;
        if(!CenterDetail::Linear(FVector3d(4,1,0),FVector3d(5,1,0),parallel))return false;FDistanceBudget tangency{1000000};
        if(!TestTrue(TEXT("tangency cannot be reclassified as strict whole-path miss"),
            D::PrepareUnion(tangent,parallel,1.,tangency,1.e-5,1.e-3,1.e-3,contact,disposition)==E::NotApplicable&&contact.Leaves.IsEmpty()))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDirectCurveUnionConsumerTest,
 "IAmSpeed.AnalyticWorld.DirectCurveUnion.OriginalConsumerAndPaidRefinement",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDirectCurveUnionConsumerTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace X=DirectCurveUnionTests;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.DirectCurveUnionContact"),1,FName(TEXT("DirectCurveUnionConsumer")));
    FScopedCVar<int32> noLine(TEXT("p.IAmSpeed.AnalyticWorld.DirectCoordinateLineContact"),0,FName(TEXT("DirectCurveUnionNoSingle")));
    FScopedCVar<int32> noOffset(TEXT("p.IAmSpeed.AnalyticWorld.DirectOffsetCurveContact"),0,FName(TEXT("DirectCurveUnionNoSingleOffset")));
    FScopedCVar<int32> noPlanes(TEXT("p.IAmSpeed.AnalyticWorld.DirectCoplanarFamilyContact"),0,FName(TEXT("DirectCurveUnionNoPlane")));
    TArray<FIntervalTensorSurface> surfaces{X::Line(),X::Line(-1.)};FIntervalBezierCenterPath path;
    if(!CenterDetail::Linear(FVector3d(4,3,0),FVector3d(5.6875,-1,0),path))return false;
    FDistanceBudget work{1000000,17,13};int32 calls=0;uint64 before=17;double lower=0,upper=1;bool valid=true;
    const auto callback=[&](const FTensorSphereContactEnclosure& c)
    {
        ++calls;valid&=c.Status==ETensorSphereContactStatus::FirstContactEnclosed&&c.LowerTime>=lower&&c.UpperTime<=upper&&
            c.LowerTime<=.5&&c.UpperTime>=.5&&X::AllIndices(c,0)&&work.Used>before&&work.ConsumptionFloor==13;
        lower=c.LowerTime;upper=c.UpperTime;before=work.Used;return calls==1?EContactEnclosureDecision::Refine:EContactEnclosureDecision::Accept;
    };
    const FContactEnclosureFilter filter(callback);
    const auto result=BoundTensorSphereFirstContact(surfaces,path,1.,work,1.e-5,1.e-8,1024,1.e-3,1.e-3,5,&filter);
    if(!TestTrue(TEXT("original generic continuation preserves union exclusion and the same paid account"),valid&&calls==2&&
        result.Status==ETensorSphereContactStatus::FirstContactEnclosed&&result.ConsumerWidthRefinements==1&&
        result.CallInitialUsed==17&&work.Used+uint64(work.Remaining)==1000017&&work.ConsumptionFloor==13))return false;
    FDistanceBudget denied{1000000};int32 deniedCalls=0;const auto deny=[&](const FTensorSphereContactEnclosure&){++deniedCalls;return EContactEnclosureDecision::Refuse;};
    const FContactEnclosureFilter refusal(deny);const auto refused=BoundTensorSphereFirstContact(surfaces,path,1.,denied,1.e-5,1.e-8,1024,1.e-3,1.e-3,5,&refusal);
    if(!TestTrue(TEXT("consumer refusal is terminal, never a request for fallback"),deniedCalls==1&&refused.Status==ETensorSphereContactStatus::Unknown&&
        refused.Failure==ETensorSphereContactFailure::ConsumerRefusal&&refused.Leaves.IsEmpty()))return false;
    surfaces[0]=X::Line(100.);surfaces[1]=X::Line(-100.);FDistanceBudget noHit{1000000};int32 clearCallbacks=0;
    const auto impossible=[&](const FTensorSphereContactEnclosure&){++clearCallbacks;return EContactEnclosureDecision::Refuse;};const FContactEnclosureFilter noCallback(impossible);
    const auto clear=BoundTensorSphereFirstContact(surfaces,path,1.,noHit,1.e-5,1.e-8,1024,1.e-3,1.e-3,5,&noCallback);
    if(!TestTrue(TEXT("complete strict miss never fabricates a contact callback"),clear.Status==ETensorSphereContactStatus::NoContactWithProvidedTensors&&clearCallbacks==0&&clear.Leaves.IsEmpty()))return false;
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
#endif


#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Analytic::Certified::DirectCurveValueTransportTests
{
namespace A=::AnalyticCertified;
static bool Same(const FTensorSphereContactEnclosure& a,const FTensorSphereContactEnclosure& b)
{
    const auto point=DirectCurveContactTests::SamePoint;
    if(a.Status!=b.Status||a.Failure!=b.Failure||FCString::Strcmp(a.WorkSite,b.WorkSite)!=0||
        a.HasCallAccounting!=b.HasCallAccounting||a.CallInitialUsed!=b.CallInitialUsed||a.TimeWorkUsed!=b.TimeWorkUsed||
        a.SpatialVisitedCells!=b.SpatialVisitedCells||a.SpatialSplitsU!=b.SpatialSplitsU||a.SpatialSplitsV!=b.SpatialSplitsV||
        a.TemporalWidthRefinements!=b.TemporalWidthRefinements||a.ConsumerWidthRefinements!=b.ConsumerWidthRefinements||
        A::Bits(a.LowerTime)!=A::Bits(b.LowerTime)||A::Bits(a.UpperTime)!=A::Bits(b.UpperTime)||
        a.CollisionRadiusBits!=b.CollisionRadiusBits||!point(a.SurfacePoint,b.SurfacePoint)||!point(a.RadialUnitNormal,b.RadialUnitNormal)||
        a.Leaves.Num()!=b.Leaves.Num())return false;
    for(int32 i=0;i<a.Leaves.Num();++i)
    {
        const auto& x=a.Leaves[i];const auto& y=b.Leaves[i];
        if(x.SurfaceIndex!=y.SurfaceIndex||A::Bits(x.MinU)!=A::Bits(y.MinU)||A::Bits(x.MaxU)!=A::Bits(y.MaxU)||
            A::Bits(x.MinV)!=A::Bits(y.MinV)||A::Bits(x.MaxV)!=A::Bits(y.MaxV)||
            A::Bits(x.FixedParameter.X)!=A::Bits(y.FixedParameter.X)||A::Bits(x.FixedParameter.Y)!=A::Bits(y.FixedParameter.Y)||
            !point(x.PointHull,y.PointHull)||!point(x.RadialNormalHull,y.RadialNormalHull)||!point(x.FixedParameterPoint,y.FixedParameterPoint))return false;
    }
    return true;
}
static FIntervalPoint RotateAxes(const FIntervalPoint& point,int32 axis)
{
    FIntervalPoint result;for(int32 d=0;d<3;++d)result.Axis[(d+axis)%3]=point.Axis[d];return result;
}
static bool Fixture(bool line,int32 axis,bool family,TArray<FIntervalTensorSurface>& surfaces,FIntervalBezierCenterPath& path)
{
    surfaces={line?DirectCurveContactTests::Line():MonotoneSeedTests::Parabola()};
    if(family)for(auto& cp:surfaces[0].ControlPoints)
    {cp.Axis[0].Lower-=1.e-10;cp.Axis[0].Upper+=1.e-10;}
    if(!CenterDetail::Linear(line?FVector3d(4,2,0):FVector3d(6,9,0),line?FVector3d(5.6875,0,0):FVector3d(10,5,0),path))return false;
    for(auto& cp:surfaces[0].ControlPoints)cp=RotateAxes(cp,axis);
    for(auto& cp:path.ControlPoints)cp=RotateAxes(cp,axis);return true;
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDirectCurveValueTransportParityTest,
    "IAmSpeed.AnalyticWorld.DirectCurveContact.RootValueTransportParity",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDirectCurveValueTransportParityTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;
    namespace Q=DirectCurveValueTransportTests;namespace D=DirectCurveContact;namespace T=TimeDetail;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();uint32 cases=0,provedLine=0,provedOffset=0;
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(uint32 sticky:{0u,1u,0x20u,0x3fu})
    for(bool line:{true,false})for(int32 axis:{0,1,2})for(bool family:{false,true})
    {
        TArray<FIntervalTensorSurface> surfaces;FIntervalBezierCenterPath path;
        if(!Q::Fixture(line,axis,family,surfaces,path))return false;
        const uint32 start=(saved&~uint32(0x807f))|flush|sticky;
        FTensorSphereContactEnclosure oldContact,newContact;bool oldLine=false,newLine=false;
        FDistanceBudget oldWork{100000,13,11},newWork=oldWork;
        StrongConvexCurveMinimum::CurvePropertyCache()={};VectorSetControlRegister(start);
        const auto oldStatus=D::Prepare(surfaces,path,line?1.:2.,oldWork,1.e-5,1.e-4,1.e-4,line,!line,oldContact,oldLine,false);
        const uint32 oldFlags=VectorGetControlRegister();
        StrongConvexCurveMinimum::CurvePropertyCache()={};VectorSetControlRegister(start);
        const auto newStatus=D::Prepare(surfaces,path,line?1.:2.,newWork,1.e-5,1.e-4,1.e-4,line,!line,newContact,newLine,true);
        const uint32 newFlags=VectorGetControlRegister();++cases;
        if(!TestTrue(TEXT("transport preserves original first-event/UV/normal bits, complete MXCSR and exact shared-account debit"),
            oldStatus==newStatus&&oldLine==newLine&&Q::Same(oldContact,newContact)&&oldFlags==newFlags&&
            oldWork.Used==newWork.Used&&oldWork.Remaining==newWork.Remaining&&oldWork.ConsumptionFloor==newWork.ConsumptionFloor))return false;
        if(newStatus==T::EMonotoneCurveTimeSeed::Proved){if(line)++provedLine;else ++provedOffset;}
        if(!family&&!TestTrue(TEXT("both literal geometry routes are actually exercised"),newStatus==T::EMonotoneCurveTimeSeed::Proved&&newLine==line))return false;
    }
    if(!TestTrue(TEXT("all modes, sticky sets, axis permutations and independent families were tested"),cases==192&&provedLine>=48&&provedOffset>=48))return false;
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDirectCurveValueTransportLifetimeTest,
    "IAmSpeed.AnalyticWorld.DirectCurveContact.RootValueTransportLifetimeAndBudget",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDirectCurveValueTransportLifetimeTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;
    namespace Q=DirectCurveValueTransportTests;namespace D=DirectCurveContact;namespace T=TimeDetail;namespace A=::AnalyticCertified;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(bool line:{true,false})
    {
        VectorSetControlRegister((saved&~uint32(0x807f))|flush);
        TArray<FIntervalTensorSurface> surfaces;FIntervalBezierCenterPath path;if(!Q::Fixture(line,0,false,surfaces,path))return false;
        FIntervalPoint captured;FDistanceBudget work{10000};
        if(line)
        {
            Interval time,parameter;int32 normal,side;double lo,hi;
            if(!TestTrue(TEXT("line private output is the literal original center at published RootTime"),
                T::TryCoordinateLineContactCertificate(surfaces,path,1.,work,1.e-5,time,parameter,normal,side,lo,hi,&captured)==T::EMonotoneCurveTimeSeed::Proved&&
                DirectCurveContactTests::SamePoint(captured,CenterDetail::Evaluate(path,time))))return false;
            auto changed=surfaces;for(auto& cp:changed[0].ControlPoints)cp.Axis[2]=Interval::Exact(.125);FDistanceBudget miss{10000};
            if(!TestTrue(TEXT("next mismatched source cannot retain the previous private center"),
                T::TryCoordinateLineContactCertificate(changed,path,1.,miss,1.e-5,time,parameter,normal,side,lo,hi,&captured)==T::EMonotoneCurveTimeSeed::NotApplicable&&
                !Detail::ValidPoint(captured)))return false;
        }
        else
        {
            T::OffsetSpatialDomainDetail::FDraft proof;double lo,hi;
            if(!TestTrue(TEXT("offset private output is produced only with its root certificate"),
                T::OffsetCurveTimeHintDetail::Try(surfaces,path,2.,work,1.e-5,lo,hi,false,&proof,&captured)==T::EMonotoneCurveTimeSeed::Proved&&proof.Ready))return false;
            StrongConvexCurveMinimum::FCurve curve;for(const auto& cp:surfaces[0].ControlPoints)curve.Add(cp);FIntervalPoint expected;
            if(!TestTrue(TEXT("offset output equals independent literal evaluation at exactly the published RootV"),
                T::OffsetCurveTimeHintDetail::EvaluateRange(curve,proof.Parameter,expected)&&DirectCurveContactTests::SamePoint(expected,captured)))return false;
            auto changed=path;changed.Degree=2;FDistanceBudget miss{10000};
            if(!TestTrue(TEXT("next unsupported path clears private point and certificate"),
                T::OffsetCurveTimeHintDetail::Try(surfaces,changed,2.,miss,1.e-5,lo,hi,false,&proof,&captured)==T::EMonotoneCurveTimeSeed::NotApplicable&&
                !Detail::ValidPoint(captured)&&!proof.Ready))return false;
        }
        for(int32 available:{0,1,127,128,255,256,447,448,449,512})
        {
            FDistanceBudget oldWork{available+7,19,7},newWork=oldWork;FTensorSphereContactEnclosure oldContact,newContact;bool oldLine,newLine;
            StrongConvexCurveMinimum::CurvePropertyCache()={};VectorSetControlRegister((saved&~uint32(0x807f))|flush);
            const auto oldStatus=D::Prepare(surfaces,path,line?1.:2.,oldWork,1.e-5,1.e-4,1.e-4,line,!line,oldContact,oldLine,false);const uint32 flags=VectorGetControlRegister();
            StrongConvexCurveMinimum::CurvePropertyCache()={};VectorSetControlRegister((saved&~uint32(0x807f))|flush);
            const auto newStatus=D::Prepare(surfaces,path,line?1.:2.,newWork,1.e-5,1.e-4,1.e-4,line,!line,newContact,newLine,true);
            if(!TestTrue(TEXT("reserve boundary and paid fallback remain exact with no reset or refund"),oldStatus==newStatus&&Q::Same(oldContact,newContact)&&
                oldLine==newLine&&oldWork.Used==newWork.Used&&oldWork.Remaining==newWork.Remaining&&oldWork.ConsumptionFloor==newWork.ConsumptionFloor&&flags==VectorGetControlRegister()))return false;
        }
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Analytic::Certified::DirectCurveUnionPrefixTests
{
static FIntervalTensorSurface LatePatch(double x=10.)
{
    FIntervalTensorSurface s;s.DegreeU=1;s.DegreeV=1;
    for(double u:{x,x+2.})for(double y:{-2.,0.})s.ControlPoints.Add(Detail::Point(FVector3d(u,y,0)));return s;
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDirectCurveUnionRootLocalTest,
 "IAmSpeed.AnalyticWorld.DirectCurveUnion.RootLocalFiniteLine",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDirectCurveUnionRootLocalTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;
    namespace T=TimeDetail;namespace L=CoordinateLineTimeHintDetail;using E=T::EMonotoneCurveTimeSeed;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        TArray<FIntervalTensorSurface> curves{DirectCurveContactTests::Line()};FIntervalBezierCenterPath path;
        if(!CenterDetail::Linear(FVector3d(-1,3,0),FVector3d(11,-1,0),path))return false;
        FDistanceBudget oldWork{10000},newWork{10000};double a,b;L::FRootCertificate oldRoot,newRoot;int32 reason=0;
        if(!TestTrue(TEXT("ordinary standalone hint retains its original whole-path contract"),
            L::Try(curves,path,1.,oldWork,1.e-5,a,b,false,&oldRoot,false,&reason)==E::NotApplicable&&reason==4&&!oldRoot.Time.Known))return false;
        if(!TestTrue(TEXT("finite-line distance lower bound excludes caps before the root even with an initially outside tangent"),
            L::Try(curves,path,1.,newWork,1.e-5,a,b,false,&newRoot,true,&reason)==E::Proved&&reason==0&&
            newRoot.Time.Lower<=.5&&newRoot.Time.Upper>=.5&&a<.5&&b>.5&&newWork.Used<=64))return false;
        const auto atRoot=CenterDetail::Evaluate(path,newRoot.Time),inside=CenterDetail::Evaluate(path,Interval::Exact(b));
        if(!TestTrue(TEXT("both true root foot and strict inside witness belong to the original finite image"),
            atRoot.Axis[0].Lower>0&&atRoot.Axis[0].Upper<20&&inside.Axis[0].Lower>0&&inside.Axis[0].Upper<20&&
            AnalyticCertified::Square(inside.Axis[1]).Upper<1.))return false;
        Interval rootT,rootV;int32 normal,side;FIntervalPoint transported;FDistanceBudget certificate{10000};
        if(!TestTrue(TEXT("original finite UV inverse is still mandatory after local coverage"),
            T::TryCoordinateLineContactCertificate(curves,path,1.,certificate,1.e-5,rootT,rootV,normal,side,a,b,&transported,true,&reason)==E::Proved&&
            normal==1&&side==1&&rootV.Lower>0&&rootV.Upper<1&&DirectCurveContactTests::SamePoint(transported,CenterDetail::Evaluate(path,rootT))))return false;
        FIntervalBezierCenterPath outside;if(!CenterDetail::Linear(FVector3d(25,3,0),FVector3d(30,-1,0),outside))return false;
        FDistanceBudget invalidCoverage{10000};
        if(!TestTrue(TEXT("a support-line root outside the finite curve never becomes a contact"),
            L::Try(curves,outside,1.,invalidCoverage,1.e-5,a,b,false,&newRoot,true,&reason)==E::NotApplicable&&reason==8&&!newRoot.Time.Known))return false;
        FIntervalBezierCenterPath nearCap;if(!CenterDetail::Linear(FVector3d(18.-1.e-6,3,0),FVector3d(22.-1.e-6,-1,0),nearCap))return false;
        FDistanceBudget capWork{10000};
        if(!TestTrue(TEXT("root coverage cannot replace finite coverage of the strict-inside witness near a cap"),
            L::Try(curves,nearCap,1.,capWork,1.e-5,a,b,false,&newRoot,true,&reason)==E::NotApplicable&&reason==9&&!newRoot.Time.Known&&capWork.Used<40))return false;
        nearCap.ControlPoints[1].Axis[0].Upper+=2.e-7;FDistanceBudget capFamily{10000};
        if(!TestTrue(TEXT("uncertain finite cap still requires coverage for every realization at InsideB"),
            L::Try(curves,nearCap,1.,capFamily,1.e-5,a,b,false,&newRoot,true,&reason)==E::NotApplicable&&reason==9&&!newRoot.Time.Known&&capFamily.Used<40))return false;
        auto uncertain=path;uncertain.ControlPoints[0].Axis[0].Lower-=1.e-9;uncertain.ControlPoints[0].Axis[0].Upper+=1.e-9;
        uncertain.ControlPoints[1].Axis[0].Lower-=1.e-9;uncertain.ControlPoints[1].Axis[0].Upper+=1.e-9;
        FDistanceBudget family{10000};
        if(!TestTrue(TEXT("coverage holds for every original uncertain tangential path, never its midpoint"),
            L::Try(curves,uncertain,1.,family,1.e-5,a,b,false,&newRoot,true,&reason)==E::Proved&&newRoot.Time.Lower<=.5&&newRoot.Time.Upper>=.5))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDirectCurveUnionPrefixTest,
 "IAmSpeed.AnalyticWorld.DirectCurveUnion.CompletePrefixExclusion",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDirectCurveUnionPrefixTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;
    namespace D=DirectCurveContact;namespace X=DirectCurveUnionPrefixTests;namespace A=::AnalyticCertified;using E=TimeDetail::EMonotoneCurveTimeSeed;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);FIntervalBezierCenterPath path;
        if(!CenterDetail::Linear(FVector3d(-1,3,0),FVector3d(11,-1,0),path))return false;
        const auto late=X::LatePatch();FDistanceBudget missWork{10000};bool whole=false,prefix=false;
        if(!TestTrue(TEXT("fixture neighbor intersects the whole path but is strictly absent through the first root"),
            D::UnionDetail::WholeMiss(late,CenterDetail::Hull(path),A::Square(Interval::Exact(1.)),missWork,whole)==E::Proved&&!whole&&
            D::UnionDetail::PrefixMiss(late,path,.5,A::Square(Interval::Exact(1.)),missWork,prefix)==E::Proved&&prefix))return false;
        for(bool reverse:{false,true})
        {
            TArray<FIntervalTensorSurface> surfaces{reverse?late:DirectCurveContactTests::Line(),reverse?DirectCurveContactTests::Line():late};
            FDistanceBudget work{1000000,17,13};FTensorSphereContactEnclosure contact;TArray<uint8> disposition;
            const int32 winner=reverse?1:0;
            if(!TestTrue(TEXT("pending neighbor is excluded only after a universal closed-prefix proof regardless of provider order"),
                D::PrepareUnion(surfaces,path,1.,work,1.e-5,1.e-3,1.e-3,contact,disposition)==E::Proved&&
                contact.LowerTime<=.5&&contact.UpperTime>=.5&&disposition.Num()==2&&disposition[winner]==1&&disposition[1-winner]==0&&
                DirectCurveUnionTests::AllIndices(contact,winner)&&work.Used<=17+D::UnionDetail::Allowance&&
                work.Used+uint64(work.Remaining)==1000017&&work.ConsumptionFloor==13))return false;
        }
        FIntervalTensorSurface touching;touching.DegreeU=0;touching.DegreeV=0;touching.ControlPoints={Detail::Point(FVector3d(6,1,0))};
        FDistanceBudget boundary{10000};prefix=true;
        if(!TestTrue(TEXT("contact exactly at the prefix upper endpoint is not a strict miss"),
            D::UnionDetail::PrefixMiss(touching,path,.5,A::Square(Interval::Exact(1.)),boundary,prefix)==E::Proved&&!prefix))return false;
        TArray<FIntervalTensorSurface> unresolved{DirectCurveContactTests::Line(),touching};FDistanceBudget unproved{1000000};
        FTensorSphereContactEnclosure none;TArray<uint8> dispositions;
        if(!TestTrue(TEXT("unknown geometry at the root remains unresolved and prevents every publication"),
            D::PrepareUnion(unresolved,path,1.,unproved,1.e-5,1.e-3,1.e-3,none,dispositions)==E::NotApplicable&&
            none.Status==ETensorSphereContactStatus::Unknown&&none.Leaves.IsEmpty()&&dispositions.IsEmpty()&&unproved.Used>0))return false;
        auto family=path;for(auto& p:family.ControlPoints){p.Axis[0].Lower-=1.e-8;p.Axis[0].Upper+=1.e-8;}
        FDistanceBudget familyWork{10000};prefix=false;
        if(!TestTrue(TEXT("prefix comparison includes all independently represented endpoint families"),
            D::UnionDetail::PrefixMiss(late,family,.5,A::Square(Interval::Exact(1.)),familyWork,prefix)==E::Proved&&prefix))return false;
        FDistanceBudget denied{D::UnionDetail::Allowance+12,19,13};
        if(!TestTrue(TEXT("new admissions do not enlarge the optional reserve or borrow the old floor"),
            D::PrepareUnion(unresolved,path,1.,denied,1.e-5,1.e-3,1.e-3,none,dispositions)==E::NotApplicable&&
            denied.Used==19&&denied.Remaining==D::UnionDetail::Allowance+12&&denied.ConsumptionFloor==13))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
#endif


#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Analytic::Certified::StrongConvexJointTests
{
namespace A=::AnalyticCertified;namespace Q=StrongConvexCurveMinimum;
static FIntervalTensorSurface Curve(double translation=0,bool family=false)
{
    FIntervalTensorSurface result;result.DegreeU=0;result.DegreeV=5;
    for(int32 i=0;i<6;++i)
    {
        auto p=Detail::Point(FVector3d(translation+4*i,-translation+i*(i-1),translation));
        if(family)for(int32 d=0;d<2;++d){p.Axis[d].Lower-=.001;p.Axis[d].Upper+=.001;}
        result.ControlPoints.Add(p);
    }
    return result;
}
static bool SameBound(const FTensorDistanceBounds& a,const FTensorDistanceBounds& b)
{return a.Status==b.Status&&A::Bits(a.LowerSquared)==A::Bits(b.LowerSquared)&&A::Bits(a.UpperSquared)==A::Bits(b.UpperSquared)&&
    A::Bits(a.SampleUV.X)==A::Bits(b.SampleUV.X)&&A::Bits(a.SampleUV.Y)==A::Bits(b.SampleUV.Y)&&DirectCurveContactTests::SamePoint(a.SamplePoint,b.SamplePoint);}
static bool Contains(const FIntervalPoint& bound,const FVector3d& value)
{for(int32 d=0;d<3;++d)if(!A::Valid(bound.Axis[d])||bound.Axis[d].Lower>value[d]||bound.Axis[d].Upper<value[d])return false;return true;}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStrongConvexJointEvaluationTest,
 "IAmSpeed.AnalyticWorld.TensorDistance.StrongConvexCurveDraft.JointOriginalEvaluation",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStrongConvexJointEvaluationTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace J=StrongConvexJointTests;namespace Q=StrongConvexCurveMinimum;namespace A=::AnalyticCertified;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();uint32 cases=0;
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(double shift:{0.,1073741824.})for(bool family:{false,true})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        const auto surface=J::Curve(shift,family);Q::FCurve curve=surface.ControlPoints,derivative;
        if(!Q::Derivative(curve,derivative))return false;
        const auto center=Detail::Point(FVector3d(shift+9.,-shift+6.,shift+3.));
        for(double t:{0.,.125,.25,.5,.625,.875,1.})
        {
            FIntervalPoint point,jointD,originalPoint;
            if(!TestTrue(TEXT("joint value follows all fifteen original interpolation steps exactly"),
                Q::EvaluateJointOriginal(curve,t,point,jointD)&&Q::Evaluate(curve,t,originalPoint)&&DirectCurveContactTests::SamePoint(point,originalPoint)))return false;
            if(!TestTrue(TEXT("large translations and independent coefficient boxes retain the exact polynomial/derivative realization"),
                J::Contains(point,FVector3d(shift+20*t,-shift+20*t*t,shift))&&J::Contains(jointD,FVector3d(20,40*t,0))))return false;
            // The initial C evaluation has already happened in BOTH routes.
            // The fallback must retain its value/flags, not imitate a cold call.
            const uint32 afterPoint=VectorGetControlRegister();Interval oldGradient,newGradient;FIntervalPoint oldPoint;
            if(!Q::Gradient(curve,derivative,center,t,oldGradient,&oldPoint))return false;const uint32 oldFlags=VectorGetControlRegister();
            VectorSetControlRegister(afterPoint);
            if(!TestTrue(TEXT("fallback reuses only C while reproducing the original quartic derivative/dot and sticky flags"),
                Q::OriginalGradientAtPoint(derivative,center,t,point,newGradient)&&oldGradient.Known==newGradient.Known&&
                A::Bits(oldGradient.Lower)==A::Bits(newGradient.Lower)&&A::Bits(oldGradient.Upper)==A::Bits(newGradient.Upper)&&
                DirectCurveContactTests::SamePoint(oldPoint,point)&&oldFlags==VectorGetControlRegister()))return false;
            ++cases;
        }
        for(double bad:{-1.,2.,std::numeric_limits<double>::quiet_NaN()})
        {
            FIntervalPoint point=Detail::Point(FVector3d(1,2,3)),d=point;
            if(!TestTrue(TEXT("invalid parameters cannot preserve a prior joint output"),!Q::EvaluateJointOriginal(curve,bad,point,d)&&
                !Detail::ValidPoint(point)&&!Detail::ValidPoint(d)))return false;
        }
    }
    if(!TestTrue(TEXT("112 distinct exact/family/mode/parameter cases were exercised"),cases==112))return false;
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStrongConvexJointDecisionTest,
 "IAmSpeed.AnalyticWorld.TensorDistance.StrongConvexCurveDraft.JointDecisionAndProtectedFallback",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStrongConvexJointDecisionTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace J=StrongConvexJointTests;namespace Q=StrongConvexCurveMinimum;namespace A=::AnalyticCertified;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> profile(TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"),1,FName(TEXT("JointDecisionProfile")));
    FScopedCVar<int32> scalar(TEXT("p.IAmSpeed.AnalyticWorld.StrongConvexScalarProposal"),1,FName(TEXT("JointDecisionScalar")));
    FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.StrongConvexJointDecision"),1,FName(TEXT("JointDecisionEnabled")));
    const uint32 saved=VectorGetControlRegister();const auto surface=J::Curve();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(double parameter:{.25,.625})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        const auto center=Detail::Point(FVector3d(18*parameter,20*parameter*parameter+1,3));
        const double exactMinimum=4*parameter*parameter+10;
        for(double margin:{-.125,0.,.125})
        {
            const auto threshold=Interval::Exact(exactMinimum+margin);Q::FInspection oldInspection,newInspection;
            FDistanceBudget oldWork{43,13,7},newWork=oldWork;FTensorDistanceBounds oldBound,newBound;
            const auto oldStatus=Q::TryBoundImpl(surface,center,oldWork,36,oldBound,&oldInspection,true,&threshold,0,true,true,false);
            const uint64 attempts=Q::Observation::Rows.JointAttempts.load(),stops=Q::Observation::Rows.JointStops.load(),fallbacks=Q::Observation::Rows.JointFallbacks.load();
            const auto newStatus=Q::TryBoundImpl(surface,center,newWork,36,newBound,&newInspection,true,&threshold,0,true,true,true);
            if(!TestTrue(TEXT("exactly36 available units protect the full original helper fallback and nonzero caller floor"),
                oldStatus==Q::EResult::Bounded&&newStatus==oldStatus&&newWork.Used<=49&&newWork.Remaining>=7&&
                newWork.Used+uint64(newWork.Remaining)==56&&newWork.ConsumptionFloor==7&&
                newBound.LowerSquared<=exactMinimum&&newBound.UpperSquared>=exactMinimum&&Q::Observation::Rows.JointAttempts.load()==attempts+1))return false;
            if(margin!=0)
            {
                if(!TestTrue(TEXT("strict joint certificate makes the same side decision with the original witness and seven stages"),
                    (margin<0?newBound.LowerSquared>threshold.Upper:newBound.UpperSquared<threshold.Lower)&&
                    oldWork.Used==20&&newWork.Used==20&&newInspection.NewtonSteps==0&&
                    A::Bits(oldBound.SampleUV.Y)==A::Bits(newBound.SampleUV.Y)&&DirectCurveContactTests::SamePoint(oldBound.SamplePoint,newBound.SamplePoint)&&
                    Q::Observation::Rows.JointStops.load()==stops+1))return false;
            }
            else if(!TestTrue(TEXT("inconclusive joint bound is discarded, original result bits restored, two real stages stay charged"),
                J::SameBound(oldBound,newBound)&&newWork.Used==oldWork.Used+2&&newInspection.NewtonSteps==oldInspection.NewtonSteps&&
                A::Bits(newInspection.ParameterLower)==A::Bits(oldInspection.ParameterLower)&&A::Bits(newInspection.ParameterUpper)==A::Bits(oldInspection.ParameterUpper)&&
                Q::Observation::Rows.JointFallbacks.load()==fallbacks+1))return false;
        }
        const auto equality=Interval::Exact(exactMinimum);
        for(int32 available:{0,1,4,6,16,31,32,34,35,36,37,64})
        {
            FDistanceBudget oldWork{available+7,13,7},newWork=oldWork;FTensorDistanceBounds a,b;
            const auto before=Q::Observation::Rows.JointAttempts.load();
            const auto x=Q::TryBoundImpl(surface,center,oldWork,36,a,nullptr,true,&equality,0,true,true,false);
            const auto y=Q::TryBoundImpl(surface,center,newWork,36,b,nullptr,true,&equality,0,true,true,true);
            if(!TestTrue(TEXT("no original helper success is lost at or below its admission boundary"),
                x==y&&newWork.Remaining>=7&&newWork.ConsumptionFloor==7&&newWork.Used+uint64(newWork.Remaining)==uint64(available+20)&&
                (available<36?(oldWork.Used==newWork.Used&&J::SameBound(a,b)&&Q::Observation::Rows.JointAttempts.load()==before):J::SameBound(a,b))))return false;
        }
        FDistanceBudget dispatch{128};FTensorDistanceBounds result;const auto threshold=Interval::Exact(exactMinimum+.125);
        const uint64 calls=Q::Observation::Rows.JointAttempts.load();
        if(!TestTrue(TEXT("the public decision route actually enables the private joint proof"),
            Q::TryBoundForDecision(surface,center,dispatch,36,result,&threshold,0,nullptr)==Q::EResult::Bounded&&
            Q::Observation::Rows.JointAttempts.load()==calls+1))return false;
    }
    // Do not pay a joint probe where existing fixed-point values/derivatives
    // are already cached. Measure that excluded population independently.
    FIntervalTensorSurface line;line.DegreeU=0;line.DegreeV=5;
    for(int32 i=0;i<6;++i)line.ControlPoints.Add(Detail::Point(FVector3d(4*i,0,0)));
    for(double x:{-1.,10.,21.})
    {
        const auto center=Detail::Point(FVector3d(x,0,3));const auto threshold=Interval::Exact(8.5);
        const auto attempts=Q::Observation::Rows.JointAttempts.load(),fixed=Q::Observation::Rows.ValidationFixed.load();
        FDistanceBudget oldWork{64},newWork{64};FTensorDistanceBounds oldBound,newBound;
        const auto a=Q::TryBoundImpl(line,center,oldWork,36,oldBound,nullptr,true,&threshold,0,true,true,false);
        const auto b=Q::TryBoundImpl(line,center,newWork,36,newBound,nullptr,true,&threshold,0,true,true,true);
        if(!TestTrue(TEXT("exact0 half and1 proposals preserve fixed-cache path and its original debit"),a==Q::EResult::Bounded&&a==b&&
            J::SameBound(oldBound,newBound)&&oldWork.Used==newWork.Used&&Q::Observation::Rows.JointAttempts.load()==attempts&&
            Q::Observation::Rows.ValidationFixed.load()==fixed+2))return false;
    }
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Analytic::Certified::SeparableProjectionTests
{
namespace A=::AnalyticCertified;
static Detail::FNode Node(int32 independentAxis,bool transpose,bool family,int32 kind=0)
{
    Detail::FNode result;result.DegreeU=5;result.DegreeV=5;
    for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)
    {
        const int32 i=transpose?v:u,j=transpose?u:v;FIntervalPoint point;
        for(int32 d=0;d<3;++d)
        {
            const double index=double(d==independentAxis?i:j);
            double value=d==independentAxis?(index-2.)*1.25:(d==((independentAxis+1)%3)?(index-3.)*.3:(index*index-7.)*.7);
            if(kind==1)value*=1.e150;
            if(kind==2)value*=A::FromBits(A::MinNormal);
            if(kind==3)value=(int32(index)&1)?-0.:0.;
            if(kind==4)value=(index==2?1.7e308:-1.6e308);
            point.Axis[d]=::AnalyticCertified::Interval::Exact(value);
            if(family&&A::Valid(point.Axis[d])&&value!=0.)
            {
                double lower,upper;if(A::Outward(point.Axis[d].Lower,false,lower)&&A::Outward(point.Axis[d].Upper,true,upper))
                    point.Axis[d]={lower,upper,true};
            }
        }
        result.Net.Add(point);
    }
    return result;
}
static bool SameResult(Interval a,Interval b)
{return a.Known==b.Known&&(!a.Known||(A::Bits(a.Lower)==A::Bits(b.Lower)&&A::Bits(a.Upper)==A::Bits(b.Upper)));}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGroupedBoxSeparableProjectionBitsTest,
 "IAmSpeed.AnalyticWorld.GeneralBox.SeparableProjectionBitsAndRefusals",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FGroupedBoxSeparableProjectionBitsTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    namespace M=GroupedBoxGeometryMemo;namespace T=SeparableProjectionTests;namespace A=::AnalyticCertified;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();uint64 cases=0,known=0,refused=0,stickyDifferent=0;
    const TArray<FVector3d> axes={FVector3d(1,0,0),FVector3d(0,1,0),FVector3d(0,0,1),FVector3d(0,0,0),
        FVector3d(-0.,-0.,-0.),FVector3d(.1,.3,.7),FVector3d(-.1,.3,-.7),FVector3d(2,-3,7),
        FVector3d(1.e150,-1.e150,1.e150),FVector3d(A::FromBits(1),0,0)};
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(uint32 sticky:{0u,0x10u,0x20u,0x3fu})
    for(int32 d=0;d<3;++d)for(bool transpose:{false,true})for(bool family:{false,true})for(int32 kind=0;kind<5;++kind)
    {
        const uint32 control=(saved&~uint32(0x807f))|flush|sticky;VectorSetControlRegister(control);
        const auto node=T::Node(d,transpose,family,kind);
        if(!TestTrue(TEXT("fixture geometry is original finite interval data"),M::GeometryOnly(node)))return false;
        const int8 layout=M::FindSeparableLayout(node);
        if(!TestTrue(TEXT("all coordinate permutations and transpositions admit exactly repeated boxes"),layout>0))return false;
        for(const auto& axis:axes)
        {
            VectorSetControlRegister(control);const auto expected=GroupedBoxDetail::ProjectHull(node.Net,axis);
            const uint32 expectedControl=VectorGetControlRegister();
            VectorSetControlRegister(control);const auto actual=M::FactoredProjectHull(node,axis,layout);
            const uint32 actualControl=VectorGetControlRegister();++cases;if(actual.Known)++known;else ++refused;
            if(expectedControl!=actualControl)++stickyDifferent;
            if(!TestTrue(TEXT("factorization keeps every extremum bit and arithmetic refusal, never suppresses a material overflow"),T::SameResult(expected,actual)))return false;
            if(!TestTrue(TEXT("no control changes or artificial exception flags"),
                (expectedControl&~uint32(0x3f))==(actualControl&~uint32(0x3f))&&
                (!actual.Known||((expectedControl^actualControl)&~uint32(0x30))==0)&&
                InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).Status==EAnalyticNumericStatus::Complete))return false;
        }
    }
    // Validation/exception policy is explicit: sticky bits are observations,
    // while control masks and rounding determine admissibility.
    VectorSetControlRegister(saved&~uint32(0x3f));
    auto evidence=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    for(uint32 sticky=0;sticky<64;++sticky)
    {auto e=evidence;e.VectorControl=(e.VectorControl&~uint32(0x3f))|sticky;e.SnapshotVectorControl=e.VectorControl;
        if(!TestTrue(TEXT("all masked sticky observations keep the same numerical contract"),ClassifyAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush,e).Status==EAnalyticNumericStatus::Complete&&e.ControlFingerprint()==evidence.ControlFingerprint()))return false;}
    auto unmasked=evidence;unmasked.VectorControl&=~uint32(0x80);unmasked.SnapshotVectorControl=unmasked.VectorControl;
    auto rounded=evidence;rounded.VectorControl|=0x2000;rounded.SnapshotVectorControl=rounded.VectorControl;
    if(!TestTrue(TEXT("unmasked or directed-rounding arithmetic is still rejected"),
        ClassifyAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush,unmasked).Status==EAnalyticNumericStatus::Unknown&&
        ClassifyAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush,rounded).Status==EAnalyticNumericStatus::Unknown))return false;
    auto node=T::Node(0,false,false);const int8 layout=M::FindSeparableLayout(node);
    for(const auto& axis:{FVector3d(A::FromBits(A::Infinity),0,0),FVector3d(0,A::FromBits(A::Infinity|1),0)})
        if(!TestTrue(TEXT("non-finite axis cannot become a factorized success"),!M::FactoredProjectHull(node,axis,layout).Known&&!GroupedBoxDetail::ProjectHull(node.Net,axis).Known))return false;
    for(int32 independentAxis=0;independentAxis<3;++independentAxis)for(bool transpose:{false,true})
    {
        auto overflow=T::Node(independentAxis,transpose,false);
        for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)for(int32 d=0;d<3;++d)
        {
            const int32 index=d==independentAxis?(transpose?v:u):(transpose?u:v);
            overflow.Net[u*6+v].Axis[d]=Interval::Exact((d==2?-1.:1.)*(9.2e307+double(index)*1.e305));
        }
        const int8 arrangement=M::FindSeparableLayout(overflow);
        if(!TestTrue(TEXT("finite final mathematical sum cannot hide an overflowing intermediate native addition"),arrangement>0&&
            !GroupedBoxDetail::ProjectHull(overflow.Net,FVector3d(1)).Known&&
            !M::FactoredProjectHull(overflow,FVector3d(1),arrangement).Known))return false;
    }
    AddInfo(FString::Printf(TEXT("Separable projection comparisons=%llu Known=%llu Refused=%llu StickyDifferent=%llu; sticky PE/UE equality is not the contract"),cases,known,refused,stickyDifferent));
    return TestTrue(TEXT("known and arithmetic-refusal fixtures are nonvacuous"),cases==9600&&known>0&&refused>0)&&!HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGroupedBoxSeparableProjectionMemoTest,
 "IAmSpeed.AnalyticWorld.GeneralBox.SeparableProjectionMemoMutationAndBudget",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FGroupedBoxSeparableProjectionMemoTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    namespace M=GroupedBoxGeometryMemo;namespace T=SeparableProjectionTests;namespace A=::AnalyticCertified;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> profile(TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"),1,FName(TEXT("SeparableProjectionProfile")));
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x807f))|flush);
        const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
        auto node=T::Node(0,false,true);const FVector3d axis(.1,.3,.7);
        auto* entry=M::Acquire(node,numeric,false);if(!entry)return false;
        for(auto& p:entry->Projections)p=M::FProjection{};entry->SeparableLayout=0;
        const uint64 hit0=M::Counts.ProjectionHits.load();
        Interval off,on,warm;uint32 onSticky=0;
        {FScopedCVar<int32> flag(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxSeparableProjection"),0,FName(TEXT("SeparableOracle")));
            off=M::Project(node,axis,entry,true);}
        {FScopedCVar<int32> flag(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxSeparableProjection"),1,FName(TEXT("SeparableCandidate")));
            on=M::Project(node,axis,entry,true);onSticky=VectorGetControlRegister()&0x3f;
            if(!TestTrue(TEXT("ON does not borrow OFF publication"),M::Counts.ProjectionHits.load()==hit0&&entry->SeparableLayout>0))return false;
            warm=M::Project(node,axis,entry,true);
            if(!TestTrue(TEXT("warm exact variant reuses only its own numerical publication"),M::Counts.ProjectionHits.load()==hit0+1&&T::SameResult(off,on)&&T::SameResult(on,warm)))return false;
            // The required sticky union belongs to THIS arithmetic variant.
            // Clear it and ensure an actual recomputation, rather than borrowing
            // a former full-CP publication or manufacturing observations.
            const uint32 control=VectorGetControlRegister()&~uint32(0x3f);VectorSetControlRegister(control);
            const auto expected=M::FactoredProjectHull(node,axis,entry->SeparableLayout);const uint32 expectedFlags=VectorGetControlRegister();
            VectorSetControlRegister(control);const uint64 before=M::Counts.ProjectionHits.load();
            const auto rebuilt=M::Project(node,axis,entry,true);
            if(!TestTrue(TEXT("variant required flags remain fresh after caller clears them"),T::SameResult(expected,rebuilt)&&VectorGetControlRegister()==expectedFlags&&
                (onSticky==0||M::Counts.ProjectionHits.load()==before)))return false;
        }
        const uint64 hit1=M::Counts.ProjectionHits.load();
        {FScopedCVar<int32> flag(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxSeparableProjection"),0,FName(TEXT("SeparableOracleAgain")));
            if(!TestTrue(TEXT("OFF cannot borrow the factored publication"),T::SameResult(off,M::Project(node,axis,entry,true))&&M::Counts.ProjectionHits.load()==hit1))return false;}
        auto changed=node;changed.Net[23].Axis[0].Upper=A::FromBits(A::Bits(changed.Net[23].Axis[0].Upper)+1);
        if(!TestTrue(TEXT("one coefficient ULP invalidates exact separability"),M::FindSeparableLayout(changed)<0))return false;
        FScopedCVar<int32> flag(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxSeparableProjection"),1,FName(TEXT("SeparableMutation")));
        auto* changedEntry=M::Acquire(changed,numeric,false);if(!changedEntry)return false;
        if(!TestTrue(TEXT("changed source uses literal fallback with its own cache key"),T::SameResult(M::Project(changed,axis,changedEntry,false),GroupedBoxDetail::ProjectHull(changed.Net,axis))&&changedEntry->SeparableLayout<0))return false;
        auto invalid=node;invalid.Net[0].Axis[0].Known=false;
        if(!TestTrue(TEXT("invalid original geometry cannot enter factorization or gain a result"),M::Acquire(invalid,numeric,false)==nullptr&&!M::Project(invalid,axis,nullptr,false).Known))return false;
        auto foreign=node;foreign.LowerSquared=1.;
        if(!TestTrue(TEXT("query-dependent metadata still prevents geometry admission"),M::Acquire(foreign,numeric,false)==nullptr))return false;
        // Split-created coefficient widening may destroy separability. Both
        // admitted descendants and literal fallbacks retain exact endpoints.
        Detail::FNode left,right;if(!Detail::Split(node,true,left,right))return false;
        for(const auto* child:{&left,&right})
        {auto* childEntry=M::Acquire(*child,numeric,false);if(!childEntry||!TestTrue(TEXT("descendants never assume their parent's separability"),
            T::SameResult(M::Project(*child,axis,childEntry,false),GroupedBoxDetail::ProjectHull(child->Net,axis))))return false;}
        const TArray<FVector3d,TInlineAllocator<8>> axes={axis,FVector3d(1,0,0),FVector3d(0,1,0)};
        FDistanceBudget budget{4,23,2};TArray<Interval,TInlineAllocator<8>> partial;FGroupedBoxSeparationObservation seen;
        if(!TestTrue(TEXT("factorized hits retain each axis debit and partial failure at floor"),!M::ProjectCell(node,axes,budget,partial,&seen)&&budget.Used==25&&budget.Remaining==2&&budget.ConsumptionFloor==2&&partial.Num()==2&&seen.Axis==2))return false;
    }
    return !HasAnyErrors();
}
#endif


// Fixed-axis linear projection commutes with restriction of the ORIGINAL
// tensor polynomial. Its interval implementations need not commute. These
// cells bound the original family, not arbitrary realizations of a widened
// XYZ child control net. They never enter GroupedBoxGeometryMemo::FEntry.
namespace Speed::Analytic::Certified::GroupedBoxProjectedSubdivision
{
namespace A=::AnalyticCertified;
static TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxProjectedSubdivision"),0,
    TEXT("0 OFF (default); 1 eight-group coarse only; 2 all admissible volumes including fine. Project original controls before dyadic scalar subdivision."),ECVF_Default);
int32 RuntimeMode(){const int32 value=Enabled.GetValueOnAnyThread();return value==1||value==2?value:0;}
struct FCounts
{
    std::atomic<uint64> Begins{0},Admitted{0},Refused{0},Cells{0},Axes{0},CoordinateAxes{0},
        AabbAxes{0},AabbClear{0},ScalarHullAxes{0},RootProjectionAxes{0},RootPointDots{0},
        NominalDotMultiplies{0},ScalarSplits{0},ScalarHalfSums{0},Ticks{0},PeakLiveBytes{0};
};
static FCounts Counts;
static void Peak(uint64 value)
{uint64 current=Counts.PeakLiveBytes.load();while(current<value&&!Counts.PeakLiveBytes.compare_exchange_weak(current,value)){};}
struct FContext
{
    TArray<FIntervalPoint,TInlineAllocator<36>> Original;
    FVector3d Axes[6];uint64 Numeric=0;int32 DegreeU=0,DegreeV=0;
    bool Observe=false;uint64 LiveCells=0;
};
using FContextPtr=TSharedPtr<FContext,ESPMode::NotThreadSafe>;
struct FCell
{
    FContextPtr Context;
    TArray<Interval,TInlineAllocator<36>> Net[3];
    int32 DepthU=0,DepthV=0;double MinU=0,MaxU=1,MinV=0,MaxV=1;
    uint8 ReadyMask=0;
    explicit FCell(FContextPtr context):Context(MoveTemp(context))
    {++Context->LiveCells;if(Context->Observe)Peak(sizeof(FContext)+Context->LiveCells*sizeof(FCell));}
    ~FCell(){--Context->LiveCells;}
};
struct FScope
{
    bool Observe=false;uint64 Start=0;
    explicit FScope(bool observe):Observe(observe){if(Observe)Start=FPlatformTime::Cycles64();}
    ~FScope(){if(Observe)Counts.Ticks.fetch_add(FPlatformTime::Cycles64()-Start);}
};
void PublishObservation()
{
    const auto& x=Counts;
    UE_LOG(LogTemp,Display,TEXT("[GroupedBoxProjectedSubdivision] Scope=ProgramPrefixAtPreExit NoPASS=1 Begins=%llu Admitted=%llu Refused=%llu Cells=%llu Axes=%llu CoordinateAxes=%llu AabbAxes=%llu AabbClear=%llu ScalarHullAxes=%llu RootProjectionAxes=%llu RootPointDots=%llu NominalDotIntervalMultiplies=%llu ScalarSplits=%llu ScalarHalfSums=%llu Ticks=%llu SecondsPerCycle=%.17g PeakLivePayloadBytes=%llu InterQueryCache=0"),
        x.Begins.load(),x.Admitted.load(),x.Refused.load(),x.Cells.load(),x.Axes.load(),x.CoordinateAxes.load(),
        x.AabbAxes.load(),x.AabbClear.load(),x.ScalarHullAxes.load(),x.RootProjectionAxes.load(),x.RootPointDots.load(),
        x.NominalDotMultiplies.load(),x.ScalarSplits.load(),x.ScalarHalfSums.load(),x.Ticks.load(),
        FPlatformTime::GetSecondsPerCycle64(),x.PeakLiveBytes.load());
}
static bool SameAxis(const FVector3d& a,const FVector3d& b)
{for(int32 d=0;d<3;++d)if(A::Bits(a[d])!=A::Bits(b[d]))return false;return true;}
static bool ContextValid(const FContext& context)
{
    const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    return numeric.Status==EAnalyticNumericStatus::Complete&&numeric.ControlFingerprint()==context.Numeric;
}
static bool SameDomain(const FCell& cell,const Detail::FNode& node)
{
    const auto& c=*cell.Context;
    return node.DegreeU==c.DegreeU&&node.DegreeV==c.DegreeV&&node.Net.Num()==c.Original.Num()&&
        node.DepthU==cell.DepthU&&node.DepthV==cell.DepthV&&A::Bits(node.MinU)==A::Bits(cell.MinU)&&
        A::Bits(node.MaxU)==A::Bits(cell.MaxU)&&A::Bits(node.MinV)==A::Bits(cell.MinV)&&A::Bits(node.MaxV)==A::Bits(cell.MaxV);
}
static void SetDomain(FCell& cell,const Detail::FNode& node)
{cell.DepthU=node.DepthU;cell.DepthV=node.DepthV;cell.MinU=node.MinU;cell.MaxU=node.MaxU;cell.MinV=node.MinV;cell.MaxV=node.MaxV;}
FCellPtr Begin(const Detail::FNode& original,TConstArrayView<FVector3d> axes)
{
    const bool observe=StrongConvexCurveMinimum::Observation::Enabled();FScope timing(observe);
    if(observe)++Counts.Begins;
    const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    const auto refuse=[&]()->FCellPtr{if(observe)++Counts.Refused;return {};};
    if(numeric.Status!=EAnalyticNumericStatus::Complete||axes.Num()!=6||original.DegreeU<0||original.DegreeU>5||
        original.DegreeV<0||original.DegreeV>5||original.Net.Num()!=(original.DegreeU+1)*(original.DegreeV+1)||
        original.DepthU!=0||original.DepthV!=0||A::Bits(original.MinU)!=0||A::Bits(original.MinV)!=0||
        original.MaxU!=1.||original.MaxV!=1.)return refuse();
    for(int32 a=0;a<6;++a)
    {
        if(!GroupedBoxDetail::ValidAxis(axes[a]))return refuse();
        if(a<3){FVector3d expected=FVector3d::ZeroVector;expected[a]=1.;if(!SameAxis(expected,axes[a]))return refuse();}
    }
    for(const auto& point:original.Net)if(!Detail::ValidPoint(point))return refuse();
    FContextPtr context=MakeShared<FContext,ESPMode::NotThreadSafe>();
    context->Original.Append(original.Net);context->DegreeU=original.DegreeU;context->DegreeV=original.DegreeV;
    context->Numeric=numeric.ControlFingerprint();context->Observe=observe;
    for(int32 a=0;a<6;++a)context->Axes[a]=axes[a];
    if(observe)++Counts.Admitted;
    return MakeShared<FCell,ESPMode::NotThreadSafe>(MoveTemp(context));
}
static Interval Hull(TConstArrayView<Interval> values)
{
    Interval result;
    for(const auto value:values)
    {if(!A::Valid(value))return {};if(!result.Known)result=value;
        else {result.Lower=FMath::Min(result.Lower,value.Lower);result.Upper=FMath::Max(result.Upper,value.Upper);}}
    return result;
}
static bool BuildRootAxis(FCell& cell,int32 axis)
{
    if(axis<0||axis>2)return false;
    if(cell.ReadyMask&(1u<<axis))return true;
    if(cell.DepthU||cell.DepthV||cell.MinU!=0.||cell.MaxU!=1.||cell.MinV!=0.||cell.MaxV!=1.)return false;
    TArray<Interval,TInlineAllocator<36>> temporary;const auto& context=*cell.Context;
    if(context.Observe)++Counts.RootProjectionAxes;
    for(const auto& point:context.Original)
    {
        if(context.Observe){++Counts.RootPointDots;Counts.NominalDotMultiplies.fetch_add(3);}
        const auto value=GroupedBoxDetail::Project(point,context.Axes[3+axis]);
        if(!A::Valid(value))return false;temporary.Add(value);
    }
    cell.Net[axis]=MoveTemp(temporary);cell.ReadyMask|=uint8(1u<<axis);return true;
}
static bool AllClear(TConstArrayView<Interval> projected,const FGroupedBoxVolumeBounds& volume,uint32 active)
{
    for(int32 g=0;g<volume.Groups.Num();++g)if(active&(1u<<g))
    {
        bool separate=false;
        for(int32 a=0;a<6;++a)
        {const auto body=volume.Slabs[g*6+a],surface=projected[a];separate|=body.Upper<surface.Lower||surface.Upper<body.Lower;}
        if(!separate)return false;
    }
    return true;
}
bool ProjectCell(const FCellPtr& scalar,const Detail::FNode& node,const FGroupedBoxVolumeBounds& volume,uint32 activeGroups,
    FDistanceBudget& budget,TArray<Interval,TInlineAllocator<8>>& projected,FGroupedBoxSeparationObservation* observed)
{
    projected.Reset();
    if(!scalar||!scalar->Context||!SameDomain(*scalar,node)||!ContextValid(*scalar->Context)||
        volume.Axes.Num()!=6||volume.Groups.IsEmpty()||volume.Groups.Num()>16||volume.Slabs.Num()!=volume.Groups.Num()*6||
        !activeGroups||(activeGroups&~((1u<<volume.Groups.Num())-1u)))return false;
    const auto& context=*scalar->Context;FScope timing(context.Observe);if(context.Observe)++Counts.Cells;
    for(int32 a=0;a<6;++a)if(!SameAxis(context.Axes[a],volume.Axes[a]))return false;
    FIntervalPoint coordinateHull;if(!MinimumDetail::NetHull(node,coordinateHull))return false;
    const bool aabbFirst=GroupedBoxGeometryMemo::AabbFirstEnabled.GetValueOnAnyThread()==1;
    // Preserve the six original per-axis charges, including every partial
    // prefix on budget failure. The bounded scalar-net work replaces the old
    // full dot-per-control operation; it is not a refunded or hidden retry.
    for(int32 a=0;a<6;++a)
    {
        if(observed){observed->Stage=TEXT("scalar_root_projection_or_budget");observed->Axis=a;}
        if(!budget.Consume())return false;if(context.Observe)++Counts.Axes;
        Interval bound;
        if(a<3){bound=coordinateHull.Axis[a];if(context.Observe)++Counts.CoordinateAxes;}
        else if(aabbFirst)
        {bound=GroupedBoxDetail::Project(coordinateHull,context.Axes[a]);if(context.Observe){++Counts.AabbAxes;Counts.NominalDotMultiplies.fetch_add(3);}}
        else {if(!BuildRootAxis(*scalar,a-3))return false;bound=Hull(scalar->Net[a-3]);if(context.Observe)++Counts.ScalarHullAxes;}
        // An optional wider AABB overflow falls back to the scalar root
        // polynomial under this same paid projection operation.
        if(!A::Valid(bound)&&a>=3)
        {if(!BuildRootAxis(*scalar,a-3))return false;bound=Hull(scalar->Net[a-3]);if(context.Observe)++Counts.ScalarHullAxes;}
        if(!A::Valid(bound))return false;projected.Add(bound);
    }
    if(aabbFirst&&AllClear(projected,volume,activeGroups))
    {if(context.Observe)++Counts.AabbClear;return true;}
    if(aabbFirst)
    {
        for(int32 a=0;a<3;++a)
        {if(!BuildRootAxis(*scalar,a))return false;const auto bound=Hull(scalar->Net[a]);if(!A::Valid(bound))return false;projected[3+a]=bound;if(context.Observe)++Counts.ScalarHullAxes;}
    }
    // Preserve the optional rejection of this separation theorem. It still
    // proves no physical contact and never widens the remaining allowance.
    if(GroupedBoxGeometryMemo::ContainedRejectEnabled.GetValueOnAnyThread()==1)
        for(int32 g=0;g<volume.Groups.Num();++g)if(activeGroups&(1u<<g))
        {
            bool contained=true;
            for(int32 a=0;a<6;++a)
            {const auto body=volume.Slabs[g*6+a];contained&=body.Lower<projected[a].Lower&&projected[a].Upper<body.Upper;}
            if(contained){if(observed){observed->Stage=TEXT("scalar_cell_contained_group_region");observed->Group=g;}return false;}
        }
    return true;
}
bool SplitCell(const FCellPtr& scalar,const Detail::FNode& node,const Detail::FNode& left,const Detail::FNode& right,FCellPtr& a,FCellPtr& b)
{
    a.Reset();b.Reset();
    if(!scalar||!scalar->Context||scalar->ReadyMask!=7||!SameDomain(*scalar,node)||!ContextValid(*scalar->Context))return false;
    const auto& context=*scalar->Context;FScope timing(context.Observe);
    const bool splitU=left.DepthU==node.DepthU+1&&left.DepthV==node.DepthV;
    if(!splitU&&!(left.DepthU==node.DepthU&&left.DepthV==node.DepthV+1))return false;
    const double midpoint=0.5*((splitU?node.MinU:node.MinV)+(splitU?node.MaxU:node.MaxV));
    if(left.DegreeU!=node.DegreeU||right.DegreeU!=node.DegreeU||left.DegreeV!=node.DegreeV||right.DegreeV!=node.DegreeV||
        left.DepthU!=right.DepthU||left.DepthV!=right.DepthV||left.MinU!=node.MinU||left.MinV!=node.MinV||
        right.MaxU!=node.MaxU||right.MaxV!=node.MaxV||
        (splitU?(left.MaxU!=midpoint||right.MinU!=midpoint||left.MaxV!=node.MaxV||right.MinV!=node.MinV):
            (left.MaxV!=midpoint||right.MinV!=midpoint||left.MaxU!=node.MaxU||right.MinU!=node.MinU)))return false;
    FCellPtr l=MakeShared<FCell,ESPMode::NotThreadSafe>(scalar->Context),r=MakeShared<FCell,ESPMode::NotThreadSafe>(scalar->Context);
    SetDomain(*l,left);SetDomain(*r,right);
    const int32 count=(splitU?context.DegreeU:context.DegreeV)+1,lanes=(splitU?context.DegreeV:context.DegreeU)+1,stride=context.DegreeV+1;
    for(int32 axis=0;axis<3;++axis)
    {
        if(scalar->Net[axis].Num()!=context.Original.Num())return false;
        l->Net[axis].SetNumUninitialized(context.Original.Num());r->Net[axis].SetNumUninitialized(context.Original.Num());
        for(int32 lane=0;lane<lanes;++lane)
        {
            Interval q[6];for(int32 k=0;k<count;++k)q[k]=scalar->Net[axis][splitU?k*stride+lane:lane*stride+k];
            const auto at=[&](int32 k){return splitU?k*stride+lane:lane*stride+k;};
            l->Net[axis][at(0)]=q[0];r->Net[axis][at(count-1)]=q[count-1];
            for(int32 level=1;level<count;++level)
            {
                for(int32 k=0;k<count-level;++k)
                {q[k]=A::HalfSum(q[k],q[k+1]);if(context.Observe)++Counts.ScalarHalfSums;if(!A::Valid(q[k]))return false;}
                l->Net[axis][at(level)]=q[0];r->Net[axis][at(count-level-1)]=q[count-level-1];
            }
        }
    }
    l->ReadyMask=r->ReadyMask=7;a=MoveTemp(l);b=MoveTemp(r);if(context.Observe)++Counts.ScalarSplits;return true;
}
}


#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Analytic::Certified::GroupedProjectedTests
{
namespace P=GroupedBoxProjectedSubdivision;
static Detail::FNode Root()
{
    Detail::FNode root;root.DegreeU=root.DegreeV=5;
    for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)
    {
        auto p=Detail::Point(FVector3d(4*u-2*v-8,v*v-2*u,8-3*v+u*v));
        for(auto& c:p.Axis){c.Lower-=1./16.;c.Upper+=1./16.;}root.Net.Add(p);
    }
    return root;
}
static TArray<FVector3d> Axes()
{return {FVector3d(1,0,0),FVector3d(0,1,0),FVector3d(0,0,1),FVector3d(1,2,-1),FVector3d(2,-1,1),FVector3d(-1,1,3)};}
static bool WideVolume(FGroupedBoxVolumeBounds& out)
{
    FBoxVolumeGroup group;FIntervalPoint hull;
    for(int32 k=0;k<8;++k){const auto p=Detail::Point(FVector3d(k&1?100.:-100.,k&2?100.:-100.,k&4?100.:-100.));
        group.Corners.Add(p);hull=Detail::ValidPoint(hull)?TimeDetail::Hull(hull,p):p;}
    TArray<FBoxVolumeGroup> groups{group};const auto axes=Axes();FDistanceBudget work{10000};
    return CloseGroupedBoxVolumeBounds(hull,groups,axes,work,out);
}
static bool GroupCount(const FGroupedBoxVolumeBounds& source,int32 count,FGroupedBoxVolumeBounds& output)
{
    TArray<FBoxVolumeGroup> groups;for(int32 i=0;i<count;++i)groups.Add(source.Groups[0]);
    FDistanceBudget work{10000};return CloseGroupedBoxVolumeBounds(source.BroadHull,groups,source.Axes,work,output);
}
static int64 Pow(int64 value,int32 n){int64 result=1;for(int32 i=0;i<n;++i)result*=value;return result;}
// Exact binary rational oracle. Coefficients are multiples of 1/16, U/V
// are p/16 and all weighted sums fit in 53 significant integer bits.
static FVector3d ExactPoint(const Detail::FNode& root,int32 u,int32 v,int32 realization)
{
    const int64 choose[]={1,5,10,10,5,1};int64 sum[3]={0,0,0};
    for(int32 i=0;i<6;++i)for(int32 j=0;j<6;++j)
    {
        const int64 weight=choose[i]*Pow(u,i)*Pow(16-u,5-i)*choose[j]*Pow(v,j)*Pow(16-v,5-j);
        for(int32 d=0;d<3;++d)
        {
            const auto c=root.Net[i*6+j].Axis[d];
            const double selected=realization==0?c.Lower:realization==1?c.Upper:((i+j+d)&1?c.Lower:c.Upper);
            sum[d]+=int64(selected*16.)*weight;
        }
    }
    return FVector3d(double(sum[0])/17592186044416.,double(sum[1])/17592186044416.,double(sum[2])/17592186044416.);
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGroupedProjectedRootFamilyTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.ProjectedSubdivisionOriginalFamily",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FGroupedProjectedRootFamilyTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;
    namespace X=GroupedProjectedTests;namespace P=GroupedBoxProjectedSubdivision;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE exact rational fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> contained(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxContainedReject"),0,FName(TEXT("ProjectedNoContained")));
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})for(bool aabb:{false,true})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        FScopedCVar<int32> useAabb(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxAabbFirst"),aabb?1:0,FName(TEXT("ProjectedAabb")));
        const auto originalRoot=X::Root();FGroupedBoxVolumeBounds volume;if(!X::WideVolume(volume))return false;
        struct FItem{Detail::FNode Node;P::FCellPtr Scalar;int32 Level;};
        TArray<FItem> stack;stack.Add({originalRoot,P::Begin(originalRoot,volume.Axes),0});FDistanceBudget work{100000};
        while(!stack.IsEmpty())
        {
            auto item=stack.Pop(EAllowShrinking::No);TArray<Interval,TInlineAllocator<8>> bounds;const auto used=work.Used;
            if(!TestTrue(TEXT("each root/descendant uses exactly the six unchanged axis charges"),
                item.Scalar&&P::ProjectCell(item.Scalar,item.Node,volume,1,work,bounds,nullptr)&&bounds.Num()==6&&work.Used==used+6))return false;
            for(int32 u=int32(item.Node.MinU*16);u<=int32(item.Node.MaxU*16);++u)
                for(int32 v=int32(item.Node.MinV*16);v<=int32(item.Node.MaxV*16);++v)for(int32 realization=0;realization<3;++realization)
                {
                    const auto point=X::ExactPoint(originalRoot,u,v,realization);
                    for(int32 axis=0;axis<6;++axis)
                    {
                        const double exact=point.X*volume.Axes[axis].X+point.Y*volume.Axes[axis].Y+point.Z*volume.Axes[axis].Z;
                        if(!TestTrue(TEXT("scalar domains contain independent ORIGINAL coefficients at exact global UV, including edges"),
                            bounds[axis].Lower<=exact&&exact<=bounds[axis].Upper))return false;
                    }
                }
            if(item.Level<4)
            {
                Detail::FNode left,right;P::FCellPtr a,b;
                if(!TestTrue(TEXT("same dyadic U/V domain is subdivided without reprojecting an independent child family"),
                    Detail::Split(item.Node,(item.Level&1)==0,left,right)&&P::SplitCell(item.Scalar,item.Node,left,right,a,b)))return false;
                stack.Add({MoveTemp(right),MoveTemp(b),item.Level+1});stack.Add({MoveTemp(left),MoveTemp(a),item.Level+1});
            }
        }
    }
    TestTrue(TEXT("restore context"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGroupedProjectedAdmissionAndBudgetTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.ProjectedSubdivisionAdmissionAndBudget",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FGroupedProjectedAdmissionAndBudgetTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace X=GroupedProjectedTests;namespace P=GroupedBoxProjectedSubdivision;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> contained(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxContainedReject"),0,FName(TEXT("ProjectedBudgetNoContained")));
    FScopedCVar<int32> aabb(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxAabbFirst"),0,FName(TEXT("ProjectedBudgetNoAabb")));
    auto root=X::Root();FGroupedBoxVolumeBounds volume;if(!X::WideVolume(volume))return false;
    auto bad=root;bad.Net[7].Axis[2]={};if(!TestFalse(TEXT("unknown original coefficients cannot enter scalar ancestry"),bool(P::Begin(bad,volume.Axes))))return false;
    auto wrongAxes=volume.Axes;wrongAxes[0]=FVector3d(1,1,0);
    if(!TestFalse(TEXT("unsupported world-axis layout keeps the original route"),bool(P::Begin(root,wrongAxes))))return false;
    Detail::FNode left,right;if(!Detail::Split(root,true,left,right))return false;
    if(!TestFalse(TEXT("a widened descendant is never mistaken for an original root"),bool(P::Begin(left,volume.Axes))))return false;
    for(int32 cap=0;cap<12;++cap)
    {
        const auto scalar=P::Begin(root,volume.Axes);FDistanceBudget work{cap,7,2};TArray<Interval,TInlineAllocator<8>> projected;
        const bool ok=P::ProjectCell(scalar,root,volume,1,work,projected,nullptr);
        const int32 paid=FMath::Min(6,FMath::Max(0,cap-2));
        if(!TestTrue(TEXT("every partial axis prefix respects floor, retains debit and has no refund"),
            ok==(cap>=8)&&work.Used==uint64(7+paid)&&work.Remaining==cap-paid&&work.ConsumptionFloor==2))return false;
    }
    auto scalar=P::Begin(root,volume.Axes);FDistanceBudget work{1000};TArray<Interval,TInlineAllocator<8>> projected;
    if(!P::ProjectCell(scalar,root,volume,1,work,projected,nullptr))return false;
    auto foreign=volume;foreign.Axes[5].X=::AnalyticCertified::FromBits(::AnalyticCertified::Bits(foreign.Axes[5].X)+1);
    const auto before=work.Used;
    if(!TestTrue(TEXT("changed fixed axis cannot reuse an original projection"),!P::ProjectCell(scalar,root,foreign,1,work,projected,nullptr)&&work.Used==before))return false;
    auto malformed=right;malformed.MinU=.75;P::FCellPtr a,b;
    if(!TestTrue(TEXT("child-domain substitution cannot manufacture scalar ancestry"),!P::SplitCell(scalar,root,left,malformed,a,b)&&!a&&!b))return false;
    const uint32 saved=VectorGetControlRegister();VectorSetControlRegister((saved&~uint32(0x6000))|0x2000);
    if(!TestTrue(TEXT("live changed numeric controls refuse, without repair or budget reset"),
        !P::ProjectCell(scalar,root,volume,1,work,projected,nullptr)&&work.Used==before&&(VectorGetControlRegister()&0x6000)==0x2000))return false;
    VectorSetControlRegister(saved);TestTrue(TEXT("restored"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGroupedProjectedSeparationTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.ProjectedSubdivisionStrictSeparation",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FGroupedProjectedSeparationTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE integration fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> contained(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxContainedReject"),0,FName(TEXT("ProjectedIntegrationNoContained")));
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})for(bool memo:{false,true})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        FDistanceBudget build{100000};FGroupedBoxVolumeBounds initial,volume;
        if(!GroupedBoxStripTests::Volume(1.75,build,initial)||!GroupedProjectedTests::GroupCount(initial,8,volume))return false;
        const auto surface=GroupedBoxStripTests::Extrusion(false,1./1024.);TArray<FIntervalTensorSurface> surfaces{surface};
        FIndexedIntervalUnionPtr scene;FReferenceBoundsCandidateSetPtr partition;
        if(!FIndexedIntervalUnion::Build(surfaces,build,scene)||!FReferenceBoundsCandidateSet::Select(scene,volume.BroadHull,build,partition))return false;
        FReferenceBoxVolumeStrictSeparationPtr oldProof,newProof;FDistanceBudget oldWork{100000},newWork{100000};
        if(!TestTrue(TEXT("both strict proofs bind the original scene, complete unchanged groups and endpoint contract"),
            FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,oldWork,oldProof,nullptr,memo,false,false,0)&&
            FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,newWork,newProof,nullptr,memo,false,false,1)&&
            oldProof->Matches(scene,volume)&&newProof->Matches(scene,volume)&&newProof->ContainsCompleteEndpointGroup(volume.Groups[0].Corners)))return false;
        // Real intersection at (u,v)=(.5,.5), z=2: no strict separating
        // axis can clear this original point even if its scalar nets differ.
        FGroupedBoxVolumeBounds collisionBase,collision;
        if(!GroupedBoxStripTests::Volume(2.,build,collisionBase)||!GroupedProjectedTests::GroupCount(collisionBase,8,collision))return false;
        FReferenceBoundsCandidateSetPtr collisionPartition;
        if(!FReferenceBoundsCandidateSet::Select(scene,collision.BroadHull,build,collisionPartition))return false;
        FDistanceBudget blocked{4096,5,17};auto stale=newProof;
        if(!TestTrue(TEXT("original penetrating realization never becomes a clearance; failure keeps its debit"),
            !FReferenceBoxVolumeStrictSeparation::TryProve(collisionPartition,collision,blocked,stale,nullptr,memo,false,false,1)&&
            !stale&&blocked.Used>5&&blocked.Remaining>=17&&blocked.Used+uint64(blocked.Remaining)==4101))return false;
        FDistanceBudget capped{32,3,1};stale=newProof;
        if(!TestTrue(TEXT("outer proof budget failure clears stale result and preserves the fine reserve"),
            !FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,capped,stale,nullptr,memo,false,false,1)&&
            !stale&&capped.Used>=3&&capped.Remaining>=1&&capped.Used+uint64(capped.Remaining)==35))return false;
        FGroupedBoxVolumeBounds fine;if(!GroupedProjectedTests::GroupCount(initial,14,fine))return false;
        FReferenceBoundsCandidateSetPtr finePartition;if(!FReferenceBoundsCandidateSet::Select(scene,fine.BroadHull,build,finePartition))return false;
        FDistanceBudget fineOff{100000,7,31},fineOne=fineOff,fineTwo=fineOff;
        FReferenceBoxVolumeStrictSeparationPtr proofOff,proofOne,proofTwo;FGroupedBoxSeparationObservation observationOff,observationOne;
        if(!TestTrue(TEXT("mode1 preserves literal original fine proof, nodes, debits and reserve; mode2 is explicit"),
            FReferenceBoxVolumeStrictSeparation::TryProve(finePartition,fine,fineOff,proofOff,&observationOff,memo,false,false,0)&&
            FReferenceBoxVolumeStrictSeparation::TryProve(finePartition,fine,fineOne,proofOne,&observationOne,memo,false,false,1)&&
            fineOff.Used==fineOne.Used&&fineOff.Remaining==fineOne.Remaining&&fineOne.ConsumptionFloor==31&&
            observationOff.VisitedCells==observationOne.VisitedCells&&
            FReferenceBoxVolumeStrictSeparation::TryProve(finePartition,fine,fineTwo,proofTwo,nullptr,memo,false,false,2)&&
            proofTwo->Matches(scene,fine)&&proofTwo->ContainsCompleteEndpointGroup(fine.Groups[0].Corners)))return false;
    }
    TestTrue(TEXT("restored"),restore.Restore());return !HasAnyErrors();
}
#endif



#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGroupedProjectedCaptured791Test,
 "IAmSpeed.AnalyticWorld.GroupedBox.ProjectedSubdivisionCaptured791Provider",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FGroupedProjectedCaptured791Test::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace P=GroupedBoxProjectedSubdivision;namespace A=::AnalyticCertified;
    // Packet-authenticated provider726, source090d4115..., record1376925..1378944.
    // This tests its exact failing UV domain; it is not a claim that the full
    // historical frame791 scene/volume has been requalified by this fixture.
    static constexpr uint64 controls[36][6]={
    {0x0000000000000000ull,0x0000000000000000ull,0x40aec3ef1535754bull,0x40aec3ef1535754cull,0x40337ca1866b95ceull,0x40337ca1866b95cfull},
    {0x0000000000000000ull,0x0000000000000000ull,0x40aee915d761c9e9ull,0x40aee915d761c9eaull,0x403b2e5f9ff717bfull,0x403b2e5f9ff717c0ull},
    {0x0000000000000000ull,0x0000000000000000ull,0x40af0cb9d77e832full,0x40af0cb9d77e8330ull,0x4042597cb8670af3ull,0x4042597cb8670af4ull},
    {0x0000000000000000ull,0x0000000000000000ull,0x40af2e60d35523d4ull,0x40af2e60d35523d5ull,0x4047f892ffbb3959ull,0x4047f892ffbb395aull},
    {0x0000000000000000ull,0x0000000000000000ull,0x40af4d9aae3cf7ebull,0x40af4d9aae3cf7ecull,0x404e61b85b441c72ull,0x404e61b85b441c73ull},
    {0x0000000000000000ull,0x0000000000000000ull,0x40af6a09e667f3bcull,0x40af6a09e667f3bdull,0x4052bec333018866ull,0x4052bec333018867ull},
    {0x40867fdd7d3ed05eull,0x40867fdd7d3ed05full,0x40aec3ef1535754bull,0x40aec3ef1535754cull,0x40337ca1866b95ceull,0x40337ca1866b95cfull},
    {0x40867fdd7d3ed05eull,0x40867fdd7d3ed05full,0x40aee915d761c9e9ull,0x40aee915d761c9eaull,0x403b2e5f9ff717bfull,0x403b2e5f9ff717c0ull},
    {0x40867fdd7d3ed05eull,0x40867fdd7d3ed05full,0x40af0cb9d77e832full,0x40af0cb9d77e8330ull,0x4042597cb8670af3ull,0x4042597cb8670af4ull},
    {0x40867fdd7d3ed05eull,0x40867fdd7d3ed05full,0x40af2e60d35523d4ull,0x40af2e60d35523d5ull,0x4047f892ffbb3959ull,0x4047f892ffbb395aull},
    {0x40867fdd7d3ed05eull,0x40867fdd7d3ed05full,0x40af4d9aae3cf7ebull,0x40af4d9aae3cf7ecull,0x404e61b85b441c72ull,0x404e61b85b441c73ull},
    {0x40867fdd7d3ed05eull,0x40867fdd7d3ed05full,0x40af6a09e667f3bcull,0x40af6a09e667f3bdull,0x4052bec333018866ull,0x4052bec333018867ull},
    {0x40967fdd7d3ed05eull,0x40967fdd7d3ed05full,0x40aec3ef1535754bull,0x40aec3ef1535754cull,0x40337ca1866b95ceull,0x40337ca1866b95cfull},
    {0x40967fdd7d3ed05eull,0x40967fdd7d3ed05full,0x40aee915d761c9e9ull,0x40aee915d761c9eaull,0x403b2e5f9ff717bfull,0x403b2e5f9ff717c0ull},
    {0x40967fdd7d3ed05eull,0x40967fdd7d3ed05full,0x40af0cb9d77e832full,0x40af0cb9d77e8330ull,0x4042597cb8670af3ull,0x4042597cb8670af4ull},
    {0x40967fdd7d3ed05eull,0x40967fdd7d3ed05full,0x40af2e60d35523d4ull,0x40af2e60d35523d5ull,0x4047f892ffbb3959ull,0x4047f892ffbb395aull},
    {0x40967fdd7d3ed05eull,0x40967fdd7d3ed05full,0x40af4d9aae3cf7ebull,0x40af4d9aae3cf7ecull,0x404e61b85b441c72ull,0x404e61b85b441c73ull},
    {0x40967fdd7d3ed05eull,0x40967fdd7d3ed05full,0x40af6a09e667f3bcull,0x40af6a09e667f3bdull,0x4052bec333018866ull,0x4052bec333018867ull},
    {0x40a0dfe61def1c46ull,0x40a0dfe61def1c47ull,0x40aec3ef1535754bull,0x40aec3ef1535754cull,0x40337ca1866b95ceull,0x40337ca1866b95cfull},
    {0x40a0dfe61def1c46ull,0x40a0dfe61def1c47ull,0x40aee915d761c9e9ull,0x40aee915d761c9eaull,0x403b2e5f9ff717bfull,0x403b2e5f9ff717c0ull},
    {0x40a0dfe61def1c46ull,0x40a0dfe61def1c47ull,0x40af0cb9d77e832full,0x40af0cb9d77e8330ull,0x4042597cb8670af3ull,0x4042597cb8670af4ull},
    {0x40a0dfe61def1c46ull,0x40a0dfe61def1c47ull,0x40af2e60d35523d4ull,0x40af2e60d35523d5ull,0x4047f892ffbb3959ull,0x4047f892ffbb395aull},
    {0x40a0dfe61def1c46ull,0x40a0dfe61def1c47ull,0x40af4d9aae3cf7ebull,0x40af4d9aae3cf7ecull,0x404e61b85b441c72ull,0x404e61b85b441c73ull},
    {0x40a0dfe61def1c46ull,0x40a0dfe61def1c47ull,0x40af6a09e667f3bcull,0x40af6a09e667f3bdull,0x4052bec333018866ull,0x4052bec333018867ull},
    {0x40a67fdd7d3ed05eull,0x40a67fdd7d3ed05full,0x40aec3ef1535754bull,0x40aec3ef1535754cull,0x40337ca1866b95ceull,0x40337ca1866b95cfull},
    {0x40a67fdd7d3ed05eull,0x40a67fdd7d3ed05full,0x40aee915d761c9e9ull,0x40aee915d761c9eaull,0x403b2e5f9ff717bfull,0x403b2e5f9ff717c0ull},
    {0x40a67fdd7d3ed05eull,0x40a67fdd7d3ed05full,0x40af0cb9d77e832full,0x40af0cb9d77e8330ull,0x4042597cb8670af3ull,0x4042597cb8670af4ull},
    {0x40a67fdd7d3ed05eull,0x40a67fdd7d3ed05full,0x40af2e60d35523d4ull,0x40af2e60d35523d5ull,0x4047f892ffbb3959ull,0x4047f892ffbb395aull},
    {0x40a67fdd7d3ed05eull,0x40a67fdd7d3ed05full,0x40af4d9aae3cf7ebull,0x40af4d9aae3cf7ecull,0x404e61b85b441c72ull,0x404e61b85b441c73ull},
    {0x40a67fdd7d3ed05eull,0x40a67fdd7d3ed05full,0x40af6a09e667f3bcull,0x40af6a09e667f3bdull,0x4052bec333018866ull,0x4052bec333018867ull},
    {0x40ac1fd4dc8e8475ull,0x40ac1fd4dc8e8476ull,0x40aec3ef1535754bull,0x40aec3ef1535754cull,0x40337ca1866b95ceull,0x40337ca1866b95cfull},
    {0x40ac1fd4dc8e8475ull,0x40ac1fd4dc8e8476ull,0x40aee915d761c9e9ull,0x40aee915d761c9eaull,0x403b2e5f9ff717bfull,0x403b2e5f9ff717c0ull},
    {0x40ac1fd4dc8e8475ull,0x40ac1fd4dc8e8476ull,0x40af0cb9d77e832full,0x40af0cb9d77e8330ull,0x4042597cb8670af3ull,0x4042597cb8670af4ull},
    {0x40ac1fd4dc8e8475ull,0x40ac1fd4dc8e8476ull,0x40af2e60d35523d4ull,0x40af2e60d35523d5ull,0x4047f892ffbb3959ull,0x4047f892ffbb395aull},
    {0x40ac1fd4dc8e8475ull,0x40ac1fd4dc8e8476ull,0x40af4d9aae3cf7ebull,0x40af4d9aae3cf7ecull,0x404e61b85b441c72ull,0x404e61b85b441c73ull},
    {0x40ac1fd4dc8e8475ull,0x40ac1fd4dc8e8476ull,0x40af6a09e667f3bcull,0x40af6a09e667f3bdull,0x4052bec333018866ull,0x4052bec333018867ull}
    };
    // Exact Python Fraction oracle for lower/upper/checkerboard CP realizations
    // at the nine closed-domain grid points. Each pair encloses an exact real
    // projected value, never a fit or a nominal-float authority.
    static constexpr uint64 oracle[27][6][2]={
    {{0x403274e3b0bd86edull,0x403274e3b0bd86eeull},{0x40aefbae2189654bull,0x40aefbae2189654cull},{0x4040883fa57d8b53ull,0x4040883fa57d8b54ull},{0x40beed1285ef27bbull,0x40beed1285ef27bcull},{0xc0ae6fb994307903ull,0xc0ae6fb994307902ull},{0x40af9d2755e9ccc5ull,0x40af9d2755e9ccc6ull}},
    {{0x403274e3b0bd86edull,0x403274e3b0bd86eeull},{0x40aefc05a899e1feull,0x40aefc05a899e1ffull},{0x4040949c815b7932ull,0x4040949c815b7933ull},{0x40beed515347e892ull,0x40beed515347e893ull},{0xc0ae6fdfa7d17dfeull,0xc0ae6fdfa7d17dfdull},{0x40af9e133748b09eull,0x40af9e133748b09full}},
    {{0x403274e3b0bd86edull,0x403274e3b0bd86eeull},{0x40aefc5d25f304d2ull,0x40aefc5d25f304d3ull},{0x4040a0fda8e31f1eull,0x4040a0fda8e31f1full},{0x40beed900e51fc1bull,0x40beed900e51fc1cull},{0xc0ae7005a08c823bull,0xc0ae7005a08c823aull},{0x40af9eff427c2f3aull,0x40af9eff427c2f3bull}},
    {{0x40329103859a1571ull,0x40329103859a1572ull},{0x40aefbae2189654bull,0x40aefbae2189654cull},{0x4040883fa57d8b53ull,0x4040883fa57d8b54ull},{0x40beed2ea5c4044aull,0x40beed2ea5c4044bull},{0xc0ae6f4914dd06c9ull,0xc0ae6f4914dd06c8ull},{0x40af9cef164013a8ull,0x40af9cef164013a9ull}},
    {{0x40329103859a1571ull,0x40329103859a1572ull},{0x40aefc05a899e1feull,0x40aefc05a899e1ffull},{0x4040949c815b7932ull,0x4040949c815b7933ull},{0x40beed6d731cc521ull,0x40beed6d731cc522ull},{0xc0ae6f6f287e0bc4ull,0xc0ae6f6f287e0bc3ull},{0x40af9ddaf79ef781ull,0x40af9ddaf79ef782ull}},
    {{0x40329103859a1571ull,0x40329103859a1572ull},{0x40aefc5d25f304d2ull,0x40aefc5d25f304d3ull},{0x4040a0fda8e31f1eull,0x4040a0fda8e31f1full},{0x40beedac2e26d8aaull,0x40beedac2e26d8abull},{0xc0ae6f9521391001ull,0xc0ae6f9521391000ull},{0x40af9ec702d2761dull,0x40af9ec702d2761eull}},
    {{0x4032ad235a76a3f6ull,0x4032ad235a76a3f7ull},{0x40aefbae2189654bull,0x40aefbae2189654cull},{0x4040883fa57d8b53ull,0x4040883fa57d8b54ull},{0x40beed4ac598e0d8ull,0x40beed4ac598e0d9ull},{0xc0ae6ed89589948full,0xc0ae6ed89589948eull},{0x40af9cb6d6965a8bull,0x40af9cb6d6965a8cull}},
    {{0x4032ad235a76a3f6ull,0x4032ad235a76a3f7ull},{0x40aefc05a899e1feull,0x40aefc05a899e1ffull},{0x4040949c815b7932ull,0x4040949c815b7933ull},{0x40beed8992f1a1afull,0x40beed8992f1a1b0ull},{0xc0ae6efea92a998aull,0xc0ae6efea92a9989ull},{0x40af9da2b7f53e64ull,0x40af9da2b7f53e65ull}},
    {{0x4032ad235a76a3f6ull,0x4032ad235a76a3f7ull},{0x40aefc5d25f304d2ull,0x40aefc5d25f304d3ull},{0x4040a0fda8e31f1eull,0x4040a0fda8e31f1full},{0x40beedc84dfbb538ull,0x40beedc84dfbb539ull},{0xc0ae6f24a1e59dc7ull,0xc0ae6f24a1e59dc6ull},{0x40af9e8ec328bd00ull,0x40af9e8ec328bd01ull}},
    {{0x403274e3b0bd86edull,0x403274e3b0bd86eeull},{0x40aefbae2189654cull,0x40aefbae2189654dull},{0x4040883fa57d8b53ull,0x4040883fa57d8b54ull},{0x40beed1285ef27bcull,0x40beed1285ef27bdull},{0xc0ae6fb994307904ull,0xc0ae6fb994307903ull},{0x40af9d2755e9ccc6ull,0x40af9d2755e9ccc7ull}},
    {{0x403274e3b0bd86edull,0x403274e3b0bd86eeull},{0x40aefc05a899e1ffull,0x40aefc05a899e200ull},{0x4040949c815b7933ull,0x4040949c815b7934ull},{0x40beed515347e893ull,0x40beed515347e894ull},{0xc0ae6fdfa7d17dffull,0xc0ae6fdfa7d17dfeull},{0x40af9e133748b09full,0x40af9e133748b0a0ull}},
    {{0x403274e3b0bd86edull,0x403274e3b0bd86eeull},{0x40aefc5d25f304d3ull,0x40aefc5d25f304d4ull},{0x4040a0fda8e31f1full,0x4040a0fda8e31f20ull},{0x40beed900e51fc1cull,0x40beed900e51fc1dull},{0xc0ae7005a08c823cull,0xc0ae7005a08c823bull},{0x40af9eff427c2f3bull,0x40af9eff427c2f3cull}},
    {{0x40329103859a1572ull,0x40329103859a1573ull},{0x40aefbae2189654cull,0x40aefbae2189654dull},{0x4040883fa57d8b53ull,0x4040883fa57d8b54ull},{0x40beed2ea5c4044bull,0x40beed2ea5c4044cull},{0xc0ae6f4914dd06caull,0xc0ae6f4914dd06c9ull},{0x40af9cef164013a9ull,0x40af9cef164013aaull}},
    {{0x40329103859a1572ull,0x40329103859a1573ull},{0x40aefc05a899e1ffull,0x40aefc05a899e200ull},{0x4040949c815b7933ull,0x4040949c815b7934ull},{0x40beed6d731cc522ull,0x40beed6d731cc523ull},{0xc0ae6f6f287e0bc5ull,0xc0ae6f6f287e0bc4ull},{0x40af9ddaf79ef782ull,0x40af9ddaf79ef783ull}},
    {{0x40329103859a1572ull,0x40329103859a1573ull},{0x40aefc5d25f304d3ull,0x40aefc5d25f304d4ull},{0x4040a0fda8e31f1full,0x4040a0fda8e31f20ull},{0x40beedac2e26d8abull,0x40beedac2e26d8acull},{0xc0ae6f9521391002ull,0xc0ae6f9521391001ull},{0x40af9ec702d2761eull,0x40af9ec702d2761full}},
    {{0x4032ad235a76a3f6ull,0x4032ad235a76a3f7ull},{0x40aefbae2189654cull,0x40aefbae2189654dull},{0x4040883fa57d8b53ull,0x4040883fa57d8b54ull},{0x40beed4ac598e0d9ull,0x40beed4ac598e0daull},{0xc0ae6ed895899490ull,0xc0ae6ed89589948full},{0x40af9cb6d6965a8cull,0x40af9cb6d6965a8dull}},
    {{0x4032ad235a76a3f6ull,0x4032ad235a76a3f7ull},{0x40aefc05a899e1ffull,0x40aefc05a899e200ull},{0x4040949c815b7933ull,0x4040949c815b7934ull},{0x40beed8992f1a1b0ull,0x40beed8992f1a1b1ull},{0xc0ae6efea92a998bull,0xc0ae6efea92a998aull},{0x40af9da2b7f53e65ull,0x40af9da2b7f53e66ull}},
    {{0x4032ad235a76a3f6ull,0x4032ad235a76a3f7ull},{0x40aefc5d25f304d3ull,0x40aefc5d25f304d4ull},{0x4040a0fda8e31f1full,0x4040a0fda8e31f20ull},{0x40beedc84dfbb539ull,0x40beedc84dfbb53aull},{0xc0ae6f24a1e59dc8ull,0xc0ae6f24a1e59dc7ull},{0x40af9e8ec328bd01ull,0x40af9e8ec328bd02ull}},
    {{0x403274e3b0bd86edull,0x403274e3b0bd86eeull},{0x40aefbae2189654cull,0x40aefbae2189654dull},{0x4040883fa57d8b53ull,0x4040883fa57d8b54ull},{0x40beed1285ef27bcull,0x40beed1285ef27bdull},{0xc0ae6fb994307903ull,0xc0ae6fb994307902ull},{0x40af9d2755e9ccc6ull,0x40af9d2755e9ccc7ull}},
    {{0x403274e3b0bd86edull,0x403274e3b0bd86eeull},{0x40aefc05a899e1feull,0x40aefc05a899e1ffull},{0x4040949c815b7933ull,0x4040949c815b7934ull},{0x40beed515347e893ull,0x40beed515347e894ull},{0xc0ae6fdfa7d17dffull,0xc0ae6fdfa7d17dfeull},{0x40af9e133748b09full,0x40af9e133748b0a0ull}},
    {{0x403274e3b0bd86edull,0x403274e3b0bd86eeull},{0x40aefc5d25f304d3ull,0x40aefc5d25f304d4ull},{0x4040a0fda8e31f1full,0x4040a0fda8e31f20ull},{0x40beed900e51fc1bull,0x40beed900e51fc1cull},{0xc0ae7005a08c823cull,0xc0ae7005a08c823bull},{0x40af9eff427c2f3aull,0x40af9eff427c2f3bull}},
    {{0x40329103859a1571ull,0x40329103859a1572ull},{0x40aefbae2189654cull,0x40aefbae2189654dull},{0x4040883fa57d8b53ull,0x4040883fa57d8b54ull},{0x40beed2ea5c4044aull,0x40beed2ea5c4044bull},{0xc0ae6f4914dd06c9ull,0xc0ae6f4914dd06c8ull},{0x40af9cef164013a9ull,0x40af9cef164013aaull}},
    {{0x40329103859a1571ull,0x40329103859a1572ull},{0x40aefc05a899e1feull,0x40aefc05a899e1ffull},{0x4040949c815b7933ull,0x4040949c815b7934ull},{0x40beed6d731cc521ull,0x40beed6d731cc522ull},{0xc0ae6f6f287e0bc5ull,0xc0ae6f6f287e0bc4ull},{0x40af9ddaf79ef782ull,0x40af9ddaf79ef783ull}},
    {{0x40329103859a1571ull,0x40329103859a1572ull},{0x40aefc5d25f304d3ull,0x40aefc5d25f304d4ull},{0x4040a0fda8e31f1full,0x4040a0fda8e31f20ull},{0x40beedac2e26d8aaull,0x40beedac2e26d8abull},{0xc0ae6f9521391002ull,0xc0ae6f9521391001ull},{0x40af9ec702d2761dull,0x40af9ec702d2761eull}},
    {{0x4032ad235a76a3f6ull,0x4032ad235a76a3f7ull},{0x40aefbae2189654cull,0x40aefbae2189654dull},{0x4040883fa57d8b53ull,0x4040883fa57d8b54ull},{0x40beed4ac598e0d9ull,0x40beed4ac598e0daull},{0xc0ae6ed89589948full,0xc0ae6ed89589948eull},{0x40af9cb6d6965a8bull,0x40af9cb6d6965a8cull}},
    {{0x4032ad235a76a3f6ull,0x4032ad235a76a3f7ull},{0x40aefc05a899e1feull,0x40aefc05a899e1ffull},{0x4040949c815b7933ull,0x4040949c815b7934ull},{0x40beed8992f1a1b0ull,0x40beed8992f1a1b1ull},{0xc0ae6efea92a998bull,0xc0ae6efea92a998aull},{0x40af9da2b7f53e65ull,0x40af9da2b7f53e66ull}},
    {{0x4032ad235a76a3f6ull,0x4032ad235a76a3f7ull},{0x40aefc5d25f304d3ull,0x40aefc5d25f304d4ull},{0x4040a0fda8e31f1full,0x4040a0fda8e31f20ull},{0x40beedc84dfbb539ull,0x40beedc84dfbb53aull},{0xc0ae6f24a1e59dc7ull,0xc0ae6f24a1e59dc6ull},{0x40af9e8ec328bd00ull,0x40af9e8ec328bd01ull}}
    };
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE capture fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> contained(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxContainedReject"),0,FName(TEXT("CaptureScalarNoContained")));
    FScopedCVar<int32> aabb(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxAabbFirst"),0,FName(TEXT("CaptureScalarNoAabb")));
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        Detail::FNode node;node.DegreeU=node.DegreeV=5;
        for(const auto& values:controls)
        {FIntervalPoint point;for(int32 d=0;d<3;++d)point.Axis[d]={A::FromBits(values[2*d]),A::FromBits(values[2*d+1]),true};node.Net.Add(point);}
        FGroupedBoxVolumeBounds volume;if(!GroupedProjectedTests::WideVolume(volume))return false;
        auto scalar=P::Begin(node,volume.Axes);FDistanceBudget work{10000};TArray<Interval,TInlineAllocator<8>> bounds;
        if(!scalar||!P::ProjectCell(scalar,node,volume,1,work,bounds,nullptr))return false;
        for(int32 axis=0;axis<2;++axis)
        {
            const int32 depth=axis==0?14:8,index=axis==0?84:79;
            for(int32 level=0;level<depth;++level)
            {
                Detail::FNode left,right;P::FCellPtr a,b;
                if(!Detail::Split(node,axis==0,left,right)||!P::SplitCell(scalar,node,left,right,a,b))return false;
                if((index>>(depth-level-1))&1){node=MoveTemp(right);scalar=MoveTemp(b);}
                else {node=MoveTemp(left);scalar=MoveTemp(a);}
            }
        }
        if(!TestTrue(TEXT("same original dyadic failing domain retained globally"),node.MinU==84./16384.&&node.MaxU==85./16384.&&
            node.MinV==79./256.&&node.MaxV==80./256.&&P::ProjectCell(scalar,node,volume,1,work,bounds,nullptr)))return false;
        for(const auto& row:oracle)for(int32 axis=0;axis<6;++axis)
            if(!TestTrue(TEXT("captured root coefficients remain enclosed at deep domain endpoints/interior in every mode"),
                bounds[axis].Lower<=A::FromBits(row[axis][0])&&A::FromBits(row[axis][1])<=bounds[axis].Upper))return false;
    }
    TestTrue(TEXT("restored"),restore.Restore());return !HasAnyErrors();
}
#endif

// Clear-only Fourier-Motzkin elimination of one coordinate from COMPLETE
// group slabs. The relaxed 1D family encloses every original CP realization.
namespace Speed::Analytic::Certified::GroupedBoxExtrusionElimination
{
namespace Private
{
namespace A=::AnalyticCertified;
static TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxExtrusionElimination"),0,
    TEXT("Default OFF. Bounded coarse8-only exact extrusion-coordinate elimination; original fine reserve is unchanged."));
constexpr int32 Allowance=256,MaximumDirections=8;
struct FCounts
{std::atomic<uint64> Calls{0},Applicable{0},Clear{0},Unresolved{0},Unavailable{0},Invalid{0},Cells{0},Directions{0},GroupsCleared{0},Work{0},Ticks{0};};
static FCounts Counts;
static bool Same(Interval a,Interval b)
{return a.Known==b.Known&&A::Bits(a.Lower)==A::Bits(b.Lower)&&A::Bits(a.Upper)==A::Bits(b.Upper);}
static Interval Hull(Interval a,Interval b)
{if(!A::Valid(a))return b;if(!A::Valid(b))return {};return {FMath::Min(a.Lower,b.Lower),FMath::Max(a.Upper,b.Upper),true};}
static int32 Sign(double x)
{const auto bits=A::Bits(x),magnitude=bits&A::Magnitude;if(!magnitude)return 0;return (bits&A::Sign)?-1:1;}
static bool NormalOrZero(double x)
{const auto m=A::Bits(x)&A::Magnitude;return m==0||(m>=A::MinNormal&&m<A::Infinity);}
static bool SameAxis(FVector3d a,FVector3d b)
{for(int32 d=0;d<3;++d)if(A::Bits(a[d])!=A::Bits(b[d]))return false;return true;}
struct FOriented {int32 Axis=0,Sign=1;Interval A,B,C;};
struct FDirection {Interval B,C;int32 I=INDEX_NONE,J=INDEX_NONE,OriginalAxis=INDEX_NONE;};
struct FCell {Interval Net[MaximumDirections][6];uint32 Active=0;int32 Depth=0;};
struct FModel
{FOriented Oriented[4];int32 OrientedCount=0;FDirection Direction[MaximumDirections];int32 Directions=0;Interval Slab[8][MaximumDirections];FCell Root;};
static const FIntervalPoint& Point(const FIntervalTensorSurface& s,int32 u,int32 v,bool transpose)
{return s.ControlPoints[transpose?v*6+u:u*6+v];}
static bool FindLayout(const FIntervalTensorSurface& s,int32& eliminated,bool& transpose)
{
    bool fallback=false;int32 fallbackAxis=0;bool fallbackTranspose=false;
    for(int32 e=0;e<3;++e)for(bool t:{false,true})
    {
        bool same=true;
        for(int32 u=0;u<6&&same;++u)for(int32 v=0;v<6&&same;++v)
            for(int32 d=0;d<3;++d)
                same&=Same(Point(s,u,v,t).Axis[d],Point(s,d==e?u:0,d==e?0:v,t).Axis[d]);
        if(same)
        {
            // Prefer removing the genuinely varying coordinate. A constant
            // coordinate may also match after transposition but would leave
            // the long extrusion to the 1D DFS instead of eliminating it.
            bool variable=false;for(int32 u=1;u<6;++u)variable|=!Same(Point(s,u,0,t).Axis[e],Point(s,0,0,t).Axis[e]);
            if(variable){eliminated=e;transpose=t;return true;}
            if(!fallback){fallback=true;fallbackAxis=e;fallbackTranspose=t;}
        }
    }
    if(fallback){eliminated=fallbackAxis;transpose=fallbackTranspose;}return fallback;
}
static EResult Build(const FIntervalTensorSurface& s,const FGroupedBoxVolumeBounds& volume,
    FDistanceBudget& budget,uint32 active,FModel& model,bool observe)
{
    // Validation/recognition is paid once per original control point. Equal
    // boxes are only an admission heuristic, never a correlation assertion.
    Interval xRange;for(const auto& cp:s.ControlPoints)
    {if(!budget.Consume())return EResult::Unresolved;if(!Detail::ValidPoint(cp))return EResult::Invalid;}
    int32 e=0;bool transpose=false;if(!FindLayout(s,e,transpose))return EResult::NotApplicable;
    const int32 y=(e+1)%3,z=(e+2)%3;
    Interval sectionY[6],sectionZ[6];
    for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)
    {
        const auto& cp=Point(s,u,v,transpose);xRange=Hull(xRange,cp.Axis[e]);
        sectionY[v]=Hull(sectionY[v],cp.Axis[y]);sectionZ[v]=Hull(sectionZ[v],cp.Axis[z]);
    }
    // Exact sign classification precedes interval arithmetic: DAZ must never
    // turn a nonzero eliminated coefficient into a zero-axis constraint.
    for(int32 axis=0;axis<6;++axis)
    {
        const auto n=volume.Axes[axis];if(!NormalOrZero(n[e]))return EResult::NotApplicable;
        const int32 sign=Sign(n[e]);
        if(sign)
        {
            if(model.OrientedCount>=4)return EResult::NotApplicable;
            auto& q=model.Oriented[model.OrientedCount++];q.Axis=axis;q.Sign=sign;
            q.A=Interval::Exact(A::FromBits(A::Bits(n[e])&A::Magnitude));
            q.B=Interval::Exact(n[y]);q.C=Interval::Exact(n[z]);
            if(sign<0){q.B=A::Negate(q.B);q.C=A::Negate(q.C);}
        }
        else
        {
            if(model.Directions>=MaximumDirections)return EResult::NotApplicable;
            auto& d=model.Direction[model.Directions++];d.OriginalAxis=axis;
            d.B=Interval::Exact(n[y]);d.C=Interval::Exact(n[z]);
        }
    }
    for(int32 i=0;i<model.OrientedCount;++i)for(int32 j=i+1;j<model.OrientedCount;++j)
    {
        if(model.Directions>=MaximumDirections)return EResult::NotApplicable;
        auto& d=model.Direction[model.Directions++];d.I=i;d.J=j;
    }
    if(observe){++Counts.Applicable;Counts.Directions.fetch_add(model.Directions);}
    model.Root.Active=active;
    // Each fixed direction replaces one bounded full-net projection. Each
    // original coefficient box is valid; all newly computed failures are an
    // inconclusive optional attempt, not an authority-bearing result.
    for(int32 k=0;k<model.Directions;++k)
    {
        if(!budget.Consume())return EResult::Unresolved;auto& d=model.Direction[k];
        if(d.OriginalAxis==INDEX_NONE)
        {
            const auto& i=model.Oriented[d.I];const auto& j=model.Oriented[d.J];
            d.B=A::Subtract(A::Multiply(j.A,i.B),A::Multiply(i.A,j.B));
            d.C=A::Subtract(A::Multiply(j.A,i.C),A::Multiply(i.A,j.C));
        }
        if(!A::Valid(d.B)||!A::Valid(d.C))return EResult::Unresolved;
        for(int32 v=0;v<6;++v)
        {model.Root.Net[k][v]=A::Add(A::Multiply(d.B,sectionY[v]),A::Multiply(d.C,sectionZ[v]));if(!A::Valid(model.Root.Net[k][v]))return EResult::Unresolved;}
    }
    for(int32 g=0;g<8;++g)if(active&(1u<<g))
    {
        if(!budget.Consume())return EResult::Unresolved;
        Interval x=volume.Slabs[g*6+e];x.Lower=FMath::Max(x.Lower,xRange.Lower);x.Upper=FMath::Min(x.Upper,xRange.Upper);
        if(x.Lower>x.Upper){model.Root.Active&=~(1u<<g);if(observe)++Counts.GroupsCleared;continue;}
        Interval oriented[4];
        for(int32 i=0;i<model.OrientedCount;++i)
        {const auto& q=model.Oriented[i];oriented[i]=q.Axis==e?x:volume.Slabs[g*6+q.Axis];if(q.Sign<0)oriented[i]=A::Negate(oriented[i]);}
        for(int32 k=0;k<model.Directions;++k)
        {
            if(!budget.Consume())return EResult::Unresolved;const auto& d=model.Direction[k];
            if(d.OriginalAxis!=INDEX_NONE){model.Slab[g][k]=volume.Slabs[g*6+d.OriginalAxis];continue;}
            const auto& i=model.Oriented[d.I];const auto& j=model.Oriented[d.J];
            const auto low=A::Subtract(A::Multiply(j.A,Interval::Exact(oriented[d.I].Lower)),A::Multiply(i.A,Interval::Exact(oriented[d.J].Upper)));
            const auto high=A::Subtract(A::Multiply(j.A,Interval::Exact(oriented[d.I].Upper)),A::Multiply(i.A,Interval::Exact(oriented[d.J].Lower)));
            if(!A::Valid(low)||!A::Valid(high))return EResult::Unresolved;
            model.Slab[g][k]={low.Lower,high.Upper,true};if(!A::Valid(model.Slab[g][k]))return EResult::Unresolved;
        }
    }
    return EResult::Ready;
}
static EResult Prove(FModel& model,FDistanceBudget& budget,bool observe)
{
    TArray<FCell,TInlineAllocator<8>> stack;stack.Add(MoveTemp(model.Root));
    while(!stack.IsEmpty())
    {
        auto cell=stack.Pop(EAllowShrinking::No);if(!cell.Active)continue;
        if(!budget.Consume())return EResult::Unresolved;if(observe)++Counts.Cells;
        Interval projected[MaximumDirections];
        for(int32 k=0;k<model.Directions;++k)
        {if(!budget.Consume())return EResult::Unresolved;for(int32 v=0;v<6;++v)projected[k]=Hull(projected[k],cell.Net[k][v]);if(!A::Valid(projected[k]))return EResult::Unresolved;}
        for(int32 g=0;g<8;++g)if(cell.Active&(1u<<g))
        {
            if(!budget.Consume())return EResult::Unresolved;bool separated=false;
            for(int32 k=0;k<model.Directions;++k)
            {const auto p=projected[k],b=model.Slab[g][k];separated|=p.Upper<b.Lower||b.Upper<p.Lower;}
            if(separated){cell.Active&=~(1u<<g);if(observe)++Counts.GroupsCleared;}
        }
        if(!cell.Active)continue;if(cell.Depth>=48)return EResult::Unresolved;
        if(!budget.Consume()||!budget.Consume())return EResult::Unresolved;
        FCell left,right;left.Active=right.Active=cell.Active;left.Depth=right.Depth=cell.Depth+1;
        for(int32 k=0;k<model.Directions;++k)
        {
            Interval q[6];for(int32 v=0;v<6;++v)q[v]=cell.Net[k][v];left.Net[k][0]=q[0];right.Net[k][5]=q[5];
            for(int32 level=1;level<6;++level)
            {
                for(int32 v=0;v<6-level;++v){q[v]=A::HalfSum(q[v],q[v+1]);if(!A::Valid(q[v]))return EResult::Unresolved;}
                left.Net[k][level]=q[0];right.Net[k][5-level]=q[5-level];
            }
        }
        stack.Add(MoveTemp(right));stack.Add(MoveTemp(left));
    }
    return EResult::Cleared;
}
static EResult Attempt(const FIntervalTensorSurface& surface,const FGroupedBoxVolumeBounds& volume,FDistanceBudget& budget,uint32 active,bool observe)
{
    if(volume.Groups.Num()!=8||volume.Axes.Num()!=6||volume.Slabs.Num()!=48||surface.DegreeU!=5||surface.DegreeV!=5||surface.ControlPoints.Num()!=36||!active||(active&~255u))return EResult::NotApplicable;
    for(int32 a=0;a<6;++a)
    {if(!GroupedBoxDetail::ValidAxis(volume.Axes[a]))return EResult::Invalid;if(a<3){FVector3d axis=FVector3d::ZeroVector;axis[a]=1.;if(!SameAxis(axis,volume.Axes[a]))return EResult::NotApplicable;}}
    for(const auto s:volume.Slabs)if(!A::Valid(s))return EResult::Invalid;
    if(budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor)return EResult::Invalid;
    // Preserve the caller's complete fine reservation. Within coarse this
    // leaves only the original next-cell minimum, NOT all of old coarse work.
    constexpr int32 MinimumCoarseFallback=6+8+2;
    if(int64(budget.Remaining)-budget.ConsumptionFloor<Allowance+MinimumCoarseFallback||budget.Used>MAX_uint64-Allowance)return EResult::NotApplicable;
    FScopedDistanceBudgetFloor floor(budget,budget.Remaining-Allowance);if(!floor.IsValid())return EResult::Invalid;
    FModel model;const auto built=Build(surface,volume,budget,active,model,observe);return built==EResult::Ready?Prove(model,budget,observe):built;
}
static void Publish()
{
    const auto& x=Counts;
    UE_LOG(LogTemp,Display,TEXT("[GroupedBoxExtrusionElimination] Scope=ProgramPrefixAtPreExit NoPASS=1 DefaultOFF=1 CoarseOnly=1 Allowance=256 Calls=%llu Applicable=%llu Cleared=%llu Unresolved=%llu Unavailable=%llu Invalid=%llu Cells=%llu Directions=%llu GroupClears=%llu Work=%llu Ticks=%llu SecondsPerCycle=%.17g"),
        x.Calls.load(),x.Applicable.load(),x.Clear.load(),x.Unresolved.load(),x.Unavailable.load(),x.Invalid.load(),x.Cells.load(),x.Directions.load(),x.GroupsCleared.load(),x.Work.load(),x.Ticks.load(),FPlatformTime::GetSecondsPerCycle64());
}
struct FRegister{FRegister(){FCoreDelegates::OnPreExit.AddStatic(&Publish);}};static FRegister Register;
}
bool RuntimeEnabled(){return Private::Enabled.GetValueOnAnyThread()==1;}
EResult TryClear(const FIntervalTensorSurface& surface,const FGroupedBoxVolumeBounds& volume,FDistanceBudget& budget,uint32 active)
{
    const bool observe=StrongConvexCurveMinimum::Observation::Enabled();const uint64 began=observe?FPlatformTime::Cycles64():0,before=budget.Used;
    if(observe)++Private::Counts.Calls;
    const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    auto result=numeric.Status==EAnalyticNumericStatus::Complete?Private::Attempt(surface,volume,budget,active,observe):EResult::Invalid;
    const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.Status!=EAnalyticNumericStatus::Complete||after.Status!=EAnalyticNumericStatus::Complete||numeric.ControlFingerprint()!=after.ControlFingerprint())result=EResult::Invalid;
    if(observe)
    {
        Private::Counts.Work.fetch_add(budget.Used-before);Private::Counts.Ticks.fetch_add(FPlatformTime::Cycles64()-began);
        if(result==EResult::Cleared)++Private::Counts.Clear;else if(result==EResult::Unresolved)++Private::Counts.Unresolved;
        else if(result==EResult::Invalid)++Private::Counts.Invalid;else ++Private::Counts.Unavailable;
    }
    return result;
}
}

#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Analytic::Certified::ExtrusionEliminationTests
{
namespace A=::AnalyticCertified;
static FVector3d Permute(FVector3d p,int32 e)
{FVector3d q;q[e]=p.X;q[(e+1)%3]=p.Y;q[(e+2)%3]=p.Z;return q;}
static FIntervalTensorSurface Surface(double sum=2,int32 e=0,bool transpose=false,double width=0,double xScale=1)
{
    FIntervalTensorSurface s;s.DegreeU=s.DegreeV=5;s.ControlPoints.SetNum(36);
    const double x[]={-2,-1,-.5,.5,1,2};
    for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)
    {
        auto p=Detail::Point(Permute(FVector3d(x[u]*xScale,.5*sum,.5*sum),e));
        if(width>0)for(auto& c:p.Axis){c.Lower-=width;c.Upper+=width;}
        s.ControlPoints[transpose?v*6+u:u*6+v]=p;
    }
    return s;
}
static bool Volume(FGroupedBoxVolumeBounds& out,int32 e=0,int32 kind=0,bool lastOnly=false,int32 count=8)
{
    TArray<FBoxVolumeGroup> groups;FIntervalPoint hull;
    for(int32 g=0;g<count;++g)
    {
        FBoxVolumeGroup group;
        for(int32 bits=0;bits<8;++bits)
        {
            const double s1=bits&1?0.:-6.,s2=bits&2?0.:-6.,s3=bits&4?4.:-4.;FVector3d p;
            if(kind==0)p=FVector3d((s1-s2)/3.,(2*s1+s2)/6.+s3/2.,(2*s1+s2)/6.-s3/2.);
            else {const double q1=bits&1?0.:-4.,q2=bits&2?1.:0.;p=FVector3d(q2,(q1-q2+s3)/2.,(q1-q2-s3)/2.);}
            if(lastOnly&&g+1<count)p.Y-=50.;const auto point=Detail::Point(Permute(p,e));group.Corners.Add(point);
            hull=Detail::ValidPoint(hull)?TimeDetail::Hull(hull,point):point;
        }
        groups.Add(MoveTemp(group));
    }
    TArray<FVector3d> axes{FVector3d(1,0,0),FVector3d(0,1,0),FVector3d(0,0,1),
        Permute(FVector3d(1,1,1),e),Permute(kind==0?FVector3d(-2,1,1):FVector3d(1,0,0),e),Permute(FVector3d(0,1,-1),e)};
    FDistanceBudget budget{10000};return CloseGroupedBoxVolumeBounds(hull,groups,axes,budget,out);
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGroupedExtrusionEliminationGeometryTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.ExtrusionEliminationCommonCoordinate",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FGroupedExtrusionEliminationGeometryTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace X=ExtrusionEliminationTests;namespace P=GroupedBoxExtrusionElimination;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE optional proof"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> stripWork(TEXT("p.IAmSpeed.AnalyticWorld.GroupedBoxStripProof.Work"),256,FName(TEXT("CompareExtrusionElimination")));
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})for(int32 e=0;e<3;++e)for(bool transpose:{false,true})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);FGroupedBoxVolumeBounds volume;if(!X::Volume(volume,e))return false;
        auto surface=X::Surface(2,e,transpose,1./1048576.);FDistanceBudget work{4096,5,17};const auto status=P::TryClear(surface,volume,work,255);
        if(!TestTrue(TEXT("same-x pair constraints strictly separate all eight groups for independent coefficient families"),
            status==P::EResult::Cleared&&work.Used>5&&work.Used<=261&&work.Remaining>=17&&work.ConsumptionFloor==17))return false;
        if(e==0&&!transpose)
        {
            FDistanceBudget strip{4096};if(!TestTrue(TEXT("old independent-axis strip cannot recover common-x feasibility"),
                GroupedBoxStripProof::TryClear(surface,volume,strip,false)!=GroupedBoxStripProof::EResult::Cleared))return false;
        }
        for(double sum:{0.,-2.})
        {
            const auto intersects=X::Surface(sum,e,transpose);FDistanceBudget blocked{4096,5,17};
            if(!TestTrue(TEXT("an original touching/interior point never becomes strict clearance"),
                P::TryClear(intersects,volume,blocked,255)!=P::EResult::Cleared&&blocked.Used>5&&blocked.Used<=261&&blocked.Remaining>=17))return false;
        }
        FGroupedBoxVolumeBounds cap;if(!X::Volume(cap,e,1))return false;auto longSurface=X::Surface(2,e,transpose,0,50.);FDistanceBudget capped{4096};
        if(!TestTrue(TEXT("finite group extrusion bounds survive elimination of a much longer surface"),P::TryClear(longSurface,cap,capped,255)==P::EResult::Cleared))return false;
        auto unsupported=surface;unsupported.ControlPoints[7].Axis[(e+1)%3].Upper+=.25;FDistanceBudget missing{4096};
        if(!TestTrue(TEXT("nonseparable endpoint boxes are a paid optional non-admission, never a fitted surface"),
            P::TryClear(unsupported,volume,missing,255)==P::EResult::NotApplicable&&missing.Used==36))return false;
    }
    TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGroupedExtrusionEliminationBudgetTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.ExtrusionEliminationMasksAndBudget",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FGroupedExtrusionEliminationBudgetTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace X=ExtrusionEliminationTests;namespace P=GroupedBoxExtrusionElimination;namespace A=::AnalyticCertified;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE masks and budget"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    auto surface=X::Surface();FGroupedBoxVolumeBounds volume;if(!X::Volume(volume))return false;
    for(int32 available=0;available<=300;++available)
    {
        FDistanceBudget work{available+17,13,17};const auto status=P::TryClear(surface,volume,work,255);
        if(!TestTrue(TEXT("one finite optional allowance, original fine floor and all spent work retained"),
            (available<272?status==P::EResult::NotApplicable&&work.Used==13:status==P::EResult::Cleared&&work.Used>13)&&
            work.Used<=269&&work.Remaining>=17&&work.ConsumptionFloor==17&&work.Used+uint64(work.Remaining)==uint64(available+30)))return false;
    }
    FGroupedBoxVolumeBounds mixed;if(!X::Volume(mixed,0,0,true))return false;const auto touching=X::Surface(0);
    FDistanceBudget seven{4096},all{4096},last{4096};
    if(!TestTrue(TEXT("a discharged group cannot discharge its sibling: all groups and original active mask remain obligations"),
        P::TryClear(touching,mixed,seven,127)==P::EResult::Cleared&&P::TryClear(touching,mixed,all,255)!=P::EResult::Cleared&&
        P::TryClear(touching,mixed,last,128)!=P::EResult::Cleared))return false;
    FGroupedBoxVolumeBounds fine;if(!X::Volume(fine,0,0,false,14))return false;FDistanceBudget unchanged{4096,9,17};
    if(!TestTrue(TEXT("fine14 is never speculatively entered or charged"),P::TryClear(surface,fine,unchanged,255)==P::EResult::NotApplicable&&unchanged.Used==9))return false;
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);auto tiny=volume;tiny.Axes[3].X=A::FromBits(1);
        FDistanceBudget b{4096};if(!TestTrue(TEXT("nonzero subnormal elimination coefficient is never classified as zero under DAZ"),P::TryClear(surface,tiny,b,255)==P::EResult::NotApplicable&&b.Used==36))return false;
    }
    VectorSetControlRegister(saved);FDistanceBudget overflow{4096,MAX_uint64-4,17};
    if(!TestTrue(TEXT("counter-overflow gate refuses the optional scope without consuming"),P::TryClear(surface,volume,overflow,255)==P::EResult::NotApplicable&&overflow.Used==MAX_uint64-4))return false;
    auto invalid=surface;invalid.ControlPoints[0].Axis[0].Known=false;FDistanceBudget invalidWork{4096};
    if(!TestTrue(TEXT("malformed original family is terminal, never a clearance"),P::TryClear(invalid,volume,invalidWork,255)==P::EResult::Invalid))return false;
    TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGroupedExtrusionEliminationIntegrationTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.ExtrusionEliminationCompleteProof",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FGroupedExtrusionEliminationIntegrationTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace X=ExtrusionEliminationTests;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE complete scene proof"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FGroupedBoxVolumeBounds volume;if(!X::Volume(volume))return false;TArray<FIntervalTensorSurface> surfaces{X::Surface()};FDistanceBudget build{10000};
    FIndexedIntervalUnionPtr scene;FReferenceBoundsCandidateSetPtr partition;
    if(!FIndexedIntervalUnion::Build(surfaces,build,scene)||!FReferenceBoundsCandidateSet::Select(scene,volume.BroadHull,build,partition))return false;
    FReferenceBoxVolumeStrictSeparationPtr oldProof,proof;FDistanceBudget oldWork{10000},work{10000,7,31};
    if(!TestTrue(TEXT("only strict clear binds the unchanged complete scene/volume and actual endpoint consumer"),
        FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,oldWork,oldProof,nullptr,false,false,false,0,false)&&
        FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,work,proof,nullptr,false,false,false,0,true)&&
        proof&&proof->Matches(scene,volume)&&proof->ContainsCompleteEndpointGroup(volume.Groups[0].Corners)&&work.ConsumptionFloor==31))return false;
    // The old projections remain finite while pairwise elimination products
    // overflow. This optional failure must retain debit and fall back BEFORE
    // any proof is published; it is not a numeric-mode or source failure.
    auto hugeAxes=volume.Axes;for(int32 a=3;a<6;++a)hugeAxes[a]*=1e200;
    FGroupedBoxVolumeBounds huge;FDistanceBudget closeWork{10000};
    if(!CloseGroupedBoxVolumeBounds(volume.BroadHull,volume.Groups,hugeAxes,closeWork,huge))return false;
    FDistanceBudget optional{4096,5,17};
    if(!TestTrue(TEXT("finite original projections with overflowing optional elimination retain an inconclusive paid result"),
        GroupedBoxExtrusionElimination::TryClear(surfaces[0],huge,optional,255)==GroupedBoxExtrusionElimination::EResult::Unresolved&&
        optional.Used>5&&optional.Used<=261&&optional.ConsumptionFloor==17))return false;
    FDistanceBudget hugeOld{10000},hugeNew{10000};FReferenceBoxVolumeStrictSeparationPtr ho,hn;
    if(!TestTrue(TEXT("optional arithmetic overflow cannot replace a successful original two-dimensional proof"),
        FReferenceBoxVolumeStrictSeparation::TryProve(partition,huge,hugeOld,ho,nullptr,false,false,false,0,false)&&
        FReferenceBoxVolumeStrictSeparation::TryProve(partition,huge,hugeNew,hn,nullptr,false,false,false,0,true)&&
        hn&&hn->Matches(scene,huge)&&hugeNew.Used>hugeOld.Used))return false;
    surfaces[0]=X::Surface(0);FIndexedIntervalUnionPtr blockedScene;FReferenceBoundsCandidateSetPtr blockedPartition;
    if(!FIndexedIntervalUnion::Build(surfaces,build,blockedScene)||!FReferenceBoundsCandidateSet::Select(blockedScene,volume.BroadHull,build,blockedPartition))return false;
    FDistanceBudget blocked{4096,7,31};auto stale=proof;
    if(!TestTrue(TEXT("real root intersection falls back with spent optional work and never leaves stale complete proof"),
        !FReferenceBoxVolumeStrictSeparation::TryProve(blockedPartition,volume,blocked,stale,nullptr,false,false,false,0,true)&&!stale&&
        blocked.Used>263&&blocked.Remaining>=31&&blocked.Used+uint64(blocked.Remaining)==4103))return false;
    TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
#endif

// R-only fragment for the SAME existing PlanarFamilyTimeShortcut.cpp, before
// StaticBodyHierarchy::Prepare. This is not a contact surface or a derivative
// model. The source/UV identity remains owned by the immutable original union.
namespace Speed::Analytic::Certified::StaticBodyHierarchy::DegreeReduction
{
namespace A=::AnalyticCertified;
enum class EResult:uint8 {Unavailable,Ready,Work,Invalid};
constexpr uint64 MaximumWork=394;
struct FPointwiseRoot
{
 int32 DegreeU=0,DegreeV=0;
 TArray<FIntervalPoint,TInlineAllocator<16>> Net;
 FIntervalPoint Residual[6];
};
// For each original realization P, R takes the two endpoint rows from P.
// E_j bounds every coefficient of P-elevate(R) in row j of the retained axis.
// At ANY fixed (u,v), choose the two errors for j equal to
// delta_j(u)=sum_i B_i^5(u)*(P_ij-elevate(R)_ij), which lies in E_j.
// The reduced interval net therefore contains the original POINT at that UV.
// This does NOT assert that a fixed degree1x5 function equals a degree5x5 one.
// Convex restriction/hulls/projections and strict-clear proofs may use it;
// derivatives, rank, inward witnesses, contact positions/normals may NOT.
static EResult TryBuild(const FIntervalTensorSurface& original,bool reduceU,
 FDistanceBudget& budget,FPointwiseRoot& output)
{
 output={};
 const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
 if(numeric.Status!=EAnalyticNumericStatus::Complete)return EResult::Invalid;
 if(budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor||budget.Used>MAX_uint64-MaximumWork)
  return EResult::Work;
 if(original.DegreeU!=5||original.DegreeV!=5||original.ControlPoints.Num()!=36)return EResult::Unavailable;
 if(uint64(int64(budget.Remaining)-budget.ConsumptionFloor)<MaximumWork)return EResult::Unavailable;
 // Every storage object is inline. No heap/atlas allocation is performed by
// this helper, and the entire debit is forecast before the first operation.
 const auto pay=[&](uint64 n){return CandidateDetail::ConsumeRepeated(budget,n);};
 const auto finish=[&](EResult result)
 {
  const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
  return after.Status==EAnalyticNumericStatus::Complete&&after.ControlFingerprint()==numeric.ControlFingerprint()?
   result:EResult::Invalid;
 };
 if(!pay(1))return EResult::Work;
 for(const auto& point:original.ControlPoints)
 {if(!pay(1))return EResult::Work;if(!Detail::ValidPoint(point))return finish(EResult::Invalid);}
 Interval t[6],oneMinusT[6];t[0]=Interval::Exact(0);t[5]=Interval::Exact(1);
 oneMinusT[0]=t[5];oneMinusT[5]=t[0];
 for(int32 i=1;i<5;++i)
 {
  // DividePositive evaluates four directed endpoint divisions, then one
// outward subtraction. Exact i/5 is enclosed, never treated as binary i*.2.
  if(!pay(5))return EResult::Work;
  if(!StrongConvexCurveMinimum::DividePositive(Interval::Exact(double(i)),Interval::Exact(5),t[i]))
   return finish(EResult::Unavailable);
  oneMinusT[i]=A::Subtract(Interval::Exact(1),t[i]);
  if(!A::Valid(oneMinusT[i]))return finish(EResult::Unavailable);
 }
 const auto at=[&](int32 i,int32 j)->const FIntervalPoint&
 {return original.ControlPoints[reduceU?i*6+j:j*6+i];};
 FPointwiseRoot reduced;reduced.DegreeU=reduceU?1:5;reduced.DegreeV=reduceU?5:1;
 for(int32 j=0;j<6;++j)
 {
  for(auto& axis:reduced.Residual[j].Axis)axis=Interval::Exact(0);
  // Endpoint residuals are exactly zero: R uses those SAME endpoint values,
// not an independently selected realization of their interval boxes.
  for(int32 i=1;i<5;++i)for(int32 d=0;d<3;++d)
  {
   if(!pay(4))return EResult::Work;
   const auto elevated=A::Add(A::Multiply(oneMinusT[i],at(0,j).Axis[d]),A::Multiply(t[i],at(5,j).Axis[d]));
   const auto residual=A::Subtract(at(i,j).Axis[d],elevated);
   if(!A::Valid(residual))return finish(EResult::Unavailable);
   auto& hull=reduced.Residual[j].Axis[d];
   hull.Lower=FMath::Min(hull.Lower,residual.Lower);hull.Upper=FMath::Max(hull.Upper,residual.Upper);
  }
 }
 reduced.Net.SetNum(12); // TInlineAllocator<16>: no allocation.
 for(int32 k=0;k<2;++k)for(int32 j=0;j<6;++j)
 {
  FIntervalPoint point;
  for(int32 d=0;d<3;++d)
  {
   if(!pay(1))return EResult::Work;
   point.Axis[d]=A::Add(at(k*5,j).Axis[d],reduced.Residual[j].Axis[d]);
   if(!A::Valid(point.Axis[d]))return finish(EResult::Unavailable);
  }
  if(!pay(1))return EResult::Work;
  reduced.Net[reduceU?k*6+j:j*2+k]=point;
 }
 if(!pay(1))return EResult::Work;
 if(finish(EResult::Ready)!=EResult::Ready)return EResult::Invalid;
 output=MoveTemp(reduced);return EResult::Ready;
}
}

// Published geometry only: no query, filter, hit, body state or authority.
namespace Speed::Analytic::Certified::StaticBodyTubes
{
static bool RuntimeStationaryEnabled();
static StaticBodyHierarchy::EFrontier TryBuildFrontier(const FIndexedIntervalUnion&,int32,
 const FGroupedBoxVolumeBounds&,uint32,FDistanceBudget&,
 TArray<StaticBodyHierarchy::FLeaf,TInlineAllocator<16>>&);
}

namespace Speed::Analytic::Certified::StaticBodyHierarchy
{
namespace Private
{
namespace A=::AnalyticCertified;
static TAutoConsoleVariable<int32> Mode(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy"),0,
 TEXT("Default OFF.1=fine14,2=coarse8,3=both; prepare original-domain hierarchy only at explicit publication."));
static TAutoConsoleVariable<int32> Depth(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.Depth"),3,
 TEXT("Publication-only curvature depth, clamped3..5. No frame-time tree construction."));
static TAutoConsoleVariable<int32> ReduceDegree(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.DegreeReduction"),0,
 TEXT("Default OFF, publication-only. Pointwise outer1x5/5x1 nets for body strict-clear hierarchy only; never a contact or derivative model."));
constexpr int32 MaximumProviders=256,MaximumNodes=16384;
constexpr uint64 MaximumBytes=16ull*1024*1024;
struct FNode
{
 FIntervalPoint Hull;
 double MinU=0,MaxU=1,MinV=0,MaxV=1;
 int32 Left=INDEX_NONE,Right=INDEX_NONE,FirstPoint=INDEX_NONE;
 int32 DepthU=0,DepthV=0;
 uint8 DegreeU=5,DegreeV=5,PointCount=36;
 int32 SupportFrame=INDEX_NONE;Interval NormalSupport,TangentSupport;
};
struct FPlan {int32 Provider=0;bool AlongU=false;int32 Extrusion=INDEX_NONE;};
struct FCounts
{
 std::atomic<uint64> Calls{0},Unavailable{0},NumericMisses{0},StickyDifferences{0},Ready{0},WorkFailure{0},Invalid{0},
 Nodes{0},GroupTests{0},GroupClears{0},LeavesLoaded{0},ProvidersClear{0},Work{0},Ticks{0},InitStickyOr{0},QueryStickyOr{0};
};
static FCounts Counts;
static bool SameAxis(FVector3d a,FVector3d b)
{for(int32 d=0;d<3;++d)if(A::Bits(a[d])!=A::Bits(b[d]))return false;return true;}
static int32 CurrentDepth(){return FMath::Clamp(Depth.GetValueOnAnyThread(),3,5);}
static bool Pay(FDistanceBudget& budget,uint64 count){return CandidateDetail::ConsumeRepeated(budget,count);}
// Private additions inside StaticBodyHierarchy::Private. This bounds the
// SURFACE on the volume's original axes; it never changes the volume region.
static TAutoConsoleVariable<int32> SharedSupportMode(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.SharedSupportSAT"),0,
 TEXT("Default OFF. Depth3 prepared surface OBB supports projected on ORIGINAL volume axes. Strict-clear only; no new contact/normal or endpoint region."));
constexpr int32 SharedDirectionCount=256,SharedDirectionKeys=512;
constexpr uint64 MaximumQuerySupportWork=128;
struct FSharedFrame {FVector3d Normal,Tangent;Interval InverseSquaredNorm;int32 Extrusion=INDEX_NONE;};
struct FSharedCounts
{
 std::atomic<uint64> Queries{0},Tests{0},Clears{0},CoefficientBuilds{0},CoefficientHits{0},NodeProjections{0},BudgetSkips{0},ArithmeticSkips{0};
};
static FSharedCounts SharedCounts;
static int32 ChooseSharedDirection(const Detail::FNode& cell,bool alongU,int32 extrusion)
{
 if(extrusion<0||extrusion>1||cell.Net.IsEmpty())return INDEX_NONE;
 const int32 last=alongU?cell.DegreeU*(cell.DegreeV+1):cell.DegreeV;
 if(!cell.Net.IsValidIndex(last))return INDEX_NONE;
 const int32 d0=extrusion==0?1:0;double delta[2]{};
 for(int32 k=0;k<2;++k)
 {
  const int32 d=k?2:d0;const auto a=cell.Net[0].Axis[d],b=cell.Net[last].Axis[d];
  if(!A::Valid(a)||!A::Valid(b))return INDEX_NONE;
  delta[k]=(b.Lower+.5*(b.Upper-b.Lower))-(a.Lower+.5*(a.Upper-a.Lower));
  if(!FMath::IsFinite(delta[k]))return INDEX_NONE;
 }
 if(delta[0]==0.&&delta[1]==0.)return INDEX_NONE;
 double angle=FMath::Atan2(-delta[0],delta[1]);if(angle<0.)angle+=UE_DOUBLE_PI;
 const int32 bin=FMath::FloorToInt(angle*double(SharedDirectionCount)/UE_DOUBLE_PI+.5)%SharedDirectionCount;
 if(bin==0||bin==SharedDirectionCount/2)return INDEX_NONE;
 return extrusion*SharedDirectionCount+bin;
}
static bool MakeSharedFrame(int32 key,FSharedFrame& out)
{
 out={};if(key<0||key>=SharedDirectionKeys)return false;
 const int32 extrusion=key/SharedDirectionCount,d0=extrusion==0?1:0;
 const double angle=UE_DOUBLE_PI*double(key%SharedDirectionCount)/double(SharedDirectionCount);
 FSharedFrame frame;frame.Extrusion=extrusion;frame.Normal=frame.Tangent=FVector3d::ZeroVector;
 frame.Normal[d0]=FMath::Cos(angle);frame.Normal[2]=FMath::Sin(angle);
 frame.Tangent[d0]=-frame.Normal[2];frame.Tangent[2]=frame.Normal[d0];
 if(!GroupedBoxDetail::ValidAxis(frame.Normal)||!GroupedBoxDetail::ValidAxis(frame.Tangent))return false;
 const auto norm=A::Add(A::Square(Interval::Exact(frame.Normal[d0])),A::Square(Interval::Exact(frame.Normal[2])));
 if(!A::Valid(norm)||norm.Lower<=0)return false;
 if(!StrongConvexCurveMinimum::DividePositive(Interval::Exact(1.),norm,frame.InverseSquaredNorm))return false;
 if(!A::Valid(frame.InverseSquaredNorm))return false;out=frame;return true;
}
struct FSharedQuerySupports
{
 struct FEntry {int32 Frame=INDEX_NONE;uint8 Mask=0;Interval Normal[3],Tangent[3],Extrusion[3];};
 TArray<FEntry,TInlineAllocator<8>> Entries;
 uint64 Used=0;bool Observe=false;
 explicit FSharedQuerySupports(bool observe):Observe(observe){}
 bool PayOptional(FDistanceBudget& budget)
 {
  if(Used>=MaximumQuerySupportWork||budget.ConsumptionFloor<0||budget.Remaining<=budget.ConsumptionFloor||budget.Used==MAX_uint64)
  {if(Observe)++SharedCounts.BudgetSkips;return false;}
  if(!budget.Consume())return false;++Used;return true;
 }
 bool ProjectNode(int32 frameIndex,const FSharedFrame& frame,Interval normal,Interval tangent,
  const FIntervalPoint& hull,const FGroupedBoxVolumeBounds& volume,FDistanceBudget& budget,
  Interval (&projected)[3],uint8& mask)
 {
  mask=0;if(frame.Extrusion<0||frame.Extrusion>1||!A::Valid(normal)||!A::Valid(tangent))return false;
  // Caller has proved that volume axes0..2 are exact XYZ. Every point in any
  // certified slab REGION is inside the hull of its group's XYZ slabs.
  // Clipping only that irrelevant extrusion tail changes no accepted region.
  Interval relevant;
  for(int32 g=0;g<volume.Groups.Num();++g)
  {
   const auto s=volume.Slabs[g*6+frame.Extrusion];if(!A::Valid(s))return false;
   if(!relevant.Known)relevant=s;else{relevant.Lower=FMath::Min(relevant.Lower,s.Lower);relevant.Upper=FMath::Max(relevant.Upper,s.Upper);}
  }
  auto e=hull.Axis[frame.Extrusion];e.Lower=FMath::Max(e.Lower,relevant.Lower);e.Upper=FMath::Min(e.Upper,relevant.Upper);
  if(!A::Valid(e))return false;
  FEntry* row=nullptr;for(auto& item:Entries)if(item.Frame==frameIndex){row=&item;break;}
  if(!row){row=&Entries.AddDefaulted_GetRef();row->Frame=frameIndex;}
  const int32 d0=frame.Extrusion==0?1:0;
  for(int32 k=0;k<3;++k)
  {
   const uint8 bit=uint8(1u<<k);const auto axis=volume.Axes[k+3];
   if(!(row->Mask&bit))
   {
    if(!PayOptional(budget))continue;
    const auto dot=[&](FVector3d direction)
    {return A::Add(A::Multiply(Interval::Exact(axis[d0]),Interval::Exact(direction[d0])),
      A::Multiply(Interval::Exact(axis[2]),Interval::Exact(direction[2])));};
    const auto n=A::Multiply(dot(frame.Normal),frame.InverseSquaredNorm),t=A::Multiply(dot(frame.Tangent),frame.InverseSquaredNorm);
    const auto x=Interval::Exact(axis[frame.Extrusion]);
    if(!A::Valid(n)||!A::Valid(t)||!A::Valid(x)){if(Observe)++SharedCounts.ArithmeticSkips;continue;}
    row->Normal[k]=n;row->Tangent[k]=t;row->Extrusion[k]=x;row->Mask|=bit;if(Observe)++SharedCounts.CoefficientBuilds;
   }
   else if(Observe)++SharedCounts.CoefficientHits;
   if(!PayOptional(budget))continue;
   const auto oriented=A::Add(A::Add(A::Multiply(normal,row->Normal[k]),A::Multiply(tangent,row->Tangent[k])),A::Multiply(e,row->Extrusion[k]));
   const auto aabb=GroupedBoxDetail::Project(hull,axis);
   if(!A::Valid(oriented)||!A::Valid(aabb)){if(Observe)++SharedCounts.ArithmeticSkips;continue;}
   auto bounded=oriented;bounded.Lower=FMath::Max(bounded.Lower,aabb.Lower);bounded.Upper=FMath::Min(bounded.Upper,aabb.Upper);
   if(!A::Valid(bounded)){if(Observe)++SharedCounts.ArithmeticSkips;continue;}
   projected[k]=bounded;mask|=bit;if(Observe)++SharedCounts.NodeProjections;
  }
  return mask!=0;
 }
};
static void PublishShared()
{
 const auto& c=SharedCounts;
 UE_LOG(LogTemp,Display,TEXT("[StaticSharedSupportSAT] DefaultOFF=1 OriginalVolumeAxesOnly=1 Depth=3 DirectionsPerPlane=256 MaxQueryWork=128 Queries=%llu Tests=%llu Clears=%llu CoefficientBuilds=%llu CoefficientHits=%llu NodeProjections=%llu BudgetSkips=%llu ArithmeticSkips=%llu"),
  c.Queries.load(),c.Tests.load(),c.Clears.load(),c.CoefficientBuilds.load(),c.CoefficientHits.load(),c.NodeProjections.load(),c.BudgetSkips.load(),c.ArithmeticSkips.load());
}
struct FSharedRegister{FSharedRegister(){FCoreDelegates::OnPreExit.AddStatic(&PublishShared);}};static FSharedRegister SharedRegister;

static FIntervalPoint Hull(TConstArrayView<FIntervalPoint> points)
{
 FIntervalPoint value;bool first=true;
 for(const auto& p:points){if(!Detail::ValidPoint(p))return {};value=first?p:TimeDetail::Hull(value,p);first=false;}
 return value;
}
static void Publish()
{
 const auto& c=Counts;
 UE_LOG(LogTemp,Display,TEXT("[StaticBodyHierarchy] Scope=ProgramPrefixAtPreExit NoPASS=1 Calls=%llu Unavailable=%llu NumericMisses=%llu StickyDifferences=%llu Ready=%llu WorkFailure=%llu Invalid=%llu Nodes=%llu GroupTests=%llu GroupClears=%llu LeavesLoaded=%llu ProvidersClear=%llu Work=%llu Ticks=%llu SecondsPerCycle=%.17g InitStickyOr=%llu QueryStickyOr=%llu"),
 c.Calls.load(),c.Unavailable.load(),c.NumericMisses.load(),c.StickyDifferences.load(),c.Ready.load(),c.WorkFailure.load(),c.Invalid.load(),c.Nodes.load(),c.GroupTests.load(),c.GroupClears.load(),c.LeavesLoaded.load(),c.ProvidersClear.load(),c.Work.load(),c.Ticks.load(),FPlatformTime::GetSecondsPerCycle64(),c.InitStickyOr.load(),c.QueryStickyOr.load());
}
struct FRegister{FRegister(){FCoreDelegates::OnPreExit.AddStatic(&Publish);}};static FRegister Register;
}
class FAtlas final
{
 friend FAtlasPtr Prepare(TConstArrayView<FIntervalTensorSurface>,FDistanceBudget&);
 friend EDirectionalProof TryClearDirectionalProvider(const FIndexedIntervalUnion&,int32,FDirectionalRegionBuilder&,FDistanceBudget&,FDirectionalMotionObservation*);
 friend EFrontier BuildFrontier(const FIndexedIntervalUnion&,int32,const FGroupedBoxVolumeBounds&,uint32,FDistanceBudget&,TArray<FLeaf,TInlineAllocator<16>>&);
 TArray<int32> Roots; // Exact original provider order, no hashing in queries.
 TArray<Private::FNode> Nodes;
 TArray<FIntervalPoint> Points; // Leaves only; no derivatives or parent CP copies.
 TArray<Private::FSharedFrame> SupportFrames; // Only selected immutable orientations.
 uint64 Numeric=0;uint32 Sticky=0;int32 Providers=0,PreparedDepth=0;
public:
 uint64 AllocatedBytes()const{return uint64(Roots.GetAllocatedSize())+uint64(Nodes.GetAllocatedSize())+uint64(Points.GetAllocatedSize())+uint64(SupportFrames.GetAllocatedSize())+sizeof(FAtlas);}
};
bool RuntimeEnabledFor(int32 groups)
{
 if(groups==1)return StaticBodyTubes::RuntimeStationaryEnabled();
 const int32 mode=Private::Mode.GetValueOnAnyThread();
 return (groups==14&&(mode==1||mode==3))||(groups==8&&(mode==2||mode==3));
}
FAtlasPtr Prepare(TConstArrayView<FIntervalTensorSurface> input,FDistanceBudget& budget)
{
 if(Private::Mode.GetValueOnAnyThread()<1||Private::Mode.GetValueOnAnyThread()>3)return {};
 const uint64 began=FPlatformTime::Cycles64(),initial=budget.Used;
 int32 admitted=0;uint64 splitCount=0,pointVisits=0,retainedBytes=0;
 uint64 sharedFrameCount=0,supportNodes=0,supportPointProjections=0;
 uint64 reductionAttempts=0,reductionReady=0,reductionUnavailable=0,reductionSkipped=0,reductionWork=0;const int32 depth=Private::CurrentDepth();
 const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
 const auto finish=[&](const TCHAR* status,FAtlasPtr result=FAtlasPtr{})
 {
  const uint64 elapsed=FPlatformTime::Cycles64()-began,used=budget.Used>=initial?budget.Used-initial:0;
  UE_LOG(LogTemp,Display,TEXT("[StaticBodyHierarchyBuild] Status=%s PublicationOnly=1 Depth=%d InputProviders=%d Admitted=%d Splits=%llu PointVisits=%llu RetainedBytes=%llu MaximumBytes=%llu Work=%llu Numeric=%llu Ticks=%llu SecondsPerCycle=%.17g ReductionAttempts=%llu ReductionReady=%llu ReductionUnavailable=%llu ReductionReserveSkips=%llu ReductionWork=%llu SharedFrames=%llu SupportNodes=%llu SupportPointProjections=%llu"),
   status,depth,input.Num(),admitted,splitCount,pointVisits,retainedBytes,Private::MaximumBytes,used,numeric.ControlFingerprint(),elapsed,FPlatformTime::GetSecondsPerCycle64(),reductionAttempts,reductionReady,reductionUnavailable,reductionSkipped,reductionWork,sharedFrameCount,supportNodes,supportPointProjections);
  return result;
 };
 if(numeric.Status!=EAnalyticNumericStatus::Complete||budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor)
  return finish(TEXT("numeric_or_budget"));
 TArray<Private::FPlan> plans;
 // The union has already validated these same owned CPs. Recognizing the
 // static schedule is still paid, and does not assert coefficient correlation.
 for(int32 i=0;i<input.Num();++i)
 {
  if(!budget.Consume())return finish(TEXT("recognition_work"));
  const auto& s=input[i];if(s.DegreeU!=5||s.DegreeV!=5||s.ControlPoints.Num()!=36)continue;
  if(!Private::Pay(budget,36))return finish(TEXT("recognition_cp_work"));
  int32 axis=0;bool transpose=false;
  if(GroupedBoxExtrusionElimination::Private::FindLayout(s,axis,transpose))
   plans.Add({i,transpose,axis});
 }
 admitted=plans.Num();if(plans.IsEmpty())return finish(TEXT("no_admitted_providers"));
 const bool sharedSupports=(Private::SharedSupportMode.GetValueOnAnyThread()==1||DirectionalMotionControl::RuntimeEnabled())&&depth==3;
 const uint64 supportNodeWork=sharedSupports?445:0; // proposal/frame13 + two36CP projections, each3Mul+3Add.
 const uint64 leaves=uint64(1)<<depth,nodes=2*leaves-1,pointCount=uint64(admitted)*leaves*36;
 const uint64 forecast=sizeof(FAtlas)+uint64(input.Num())*sizeof(int32)+uint64(admitted)*nodes*sizeof(Private::FNode)+pointCount*sizeof(FIntervalPoint)+(sharedSupports?Private::SharedDirectionKeys*sizeof(Private::FSharedFrame):0);
 if(admitted>Private::MaximumProviders||uint64(admitted)*nodes>Private::MaximumNodes||forecast>Private::MaximumBytes)
  return finish(TEXT("storage_cap_before_allocation"));
 // Finite exact debit forecast: node hull validation + split270 HalfSum +
 // child construction2 + copying36 leaf CPs. No uncharged hidden warmup.
 const uint64 required=uint64(admitted)*(nodes*(37+supportNodeWork)+(leaves-1)*272+leaves*36);
 if(budget.ConsumptionFloor<0||uint64(int64(budget.Remaining)-budget.ConsumptionFloor)<required||budget.Used>MAX_uint64-required)
  return finish(TEXT("construction_work_unavailable"));
 TSharedPtr<FAtlas,ESPMode::ThreadSafe> atlas(new FAtlas);
 atlas->Roots.Init(INDEX_NONE,input.Num());atlas->Nodes.Reserve(int32(uint64(admitted)*nodes));atlas->Points.Reserve(int32(pointCount));
 atlas->Numeric=numeric.ControlFingerprint();atlas->Providers=admitted;atlas->PreparedDepth=depth;
 int32 sharedOrdinals[Private::SharedDirectionKeys];for(auto& v:sharedOrdinals)v=INDEX_NONE;
 if(sharedSupports)atlas->SupportFrames.Reserve(Private::SharedDirectionKeys);
 const auto build=[&](auto&& self,const Detail::FNode& cell,bool alongU,int32 extrusion,int32 remaining)->int32
 {
  const int32 n=cell.Net.Num();
  if(cell.DegreeU<1||cell.DegreeV<1||cell.DegreeU>5||cell.DegreeV>5||
     n!=(cell.DegreeU+1)*(cell.DegreeV+1)||n>36)return INDEX_NONE;
  if(!Private::Pay(budget,uint64(n)+1))return INDEX_NONE;pointVisits+=uint64(n);
  const auto hull=Private::Hull(MakeArrayView(cell.Net));if(!Detail::ValidPoint(hull))return INDEX_NONE;
  Private::FNode saved;saved.Hull=hull;saved.MinU=cell.MinU;saved.MaxU=cell.MaxU;saved.MinV=cell.MinV;saved.MaxV=cell.MaxV;
  saved.DepthU=cell.DepthU;saved.DepthV=cell.DepthV;
  saved.DegreeU=uint8(cell.DegreeU);saved.DegreeV=uint8(cell.DegreeV);saved.PointCount=uint8(n);
  if(sharedSupports)
  {
   if(!Private::Pay(budget,13+12*uint64(n)))return INDEX_NONE;
   const int32 key=Private::ChooseSharedDirection(cell,alongU,extrusion);
   if(key!=INDEX_NONE)
   {
    if(key<0||key>=Private::SharedDirectionKeys)return INDEX_NONE;
    int32 ordinal=sharedOrdinals[key];
    if(ordinal==INDEX_NONE)
    {
     Private::FSharedFrame frame;if(!Private::MakeSharedFrame(key,frame))return INDEX_NONE;
     ordinal=atlas->SupportFrames.Add(frame);sharedOrdinals[key]=ordinal;++sharedFrameCount;
    }
    const auto normal=GroupedBoxDetail::ProjectHull(cell.Net,atlas->SupportFrames[ordinal].Normal);
    const auto tangent=GroupedBoxDetail::ProjectHull(cell.Net,atlas->SupportFrames[ordinal].Tangent);
    supportPointProjections+=uint64(n)*2;
    if(Private::A::Valid(normal)&&Private::A::Valid(tangent)){saved.SupportFrame=ordinal;saved.NormalSupport=normal;saved.TangentSupport=tangent;++supportNodes;}
   }
  }
  const int32 index=atlas->Nodes.Add(saved);
  if(remaining==0)
  {
   if(!Private::Pay(budget,uint64(n)))return INDEX_NONE;
   atlas->Nodes[index].FirstPoint=atlas->Points.Num();atlas->Points.Append(cell.Net.GetData(),cell.Net.Num());return index;
  }
  const uint64 degree=uint64(alongU?cell.DegreeU:cell.DegreeV),lanes=uint64(alongU?cell.DegreeV+1:cell.DegreeU+1);
  const uint64 splitWork=lanes*degree*(degree+1)/2*3+2;
  if(!Private::Pay(budget,splitWork))return INDEX_NONE;
  Detail::FNode left,right;if(!Detail::Split(cell,alongU,left,right))return INDEX_NONE;++splitCount;
  const int32 a=self(self,left,alongU,extrusion,remaining-1),b=a!=INDEX_NONE?self(self,right,alongU,extrusion,remaining-1):INDEX_NONE;
  if(a==INDEX_NONE||b==INDEX_NONE)return INDEX_NONE;
  atlas->Nodes[index].Left=a;atlas->Nodes[index].Right=b;return index;
 };
 int32 completedProviders=0;
 const uint64 originalPerProvider=nodes*(37+supportNodeWork)+(leaves-1)*272+leaves*36;
 const bool reduceDegree=Private::ReduceDegree.GetValueOnAnyThread()==1;
 for(const auto& plan:plans)
 {
  Detail::FNode root;root.DegreeU=root.DegreeV=5;root.Net=input[plan.Provider].ControlPoints;
  if(reduceDegree)
  {
   // Protect ALL remaining original atlas construction, including this root.
   // The optional reduction pays394 extra only when that exact reserve fits.
   const uint64 reserve=uint64(admitted-completedProviders)*originalPerProvider;
   if(uint64(int64(budget.Remaining)-budget.ConsumptionFloor)>=reserve+DegreeReduction::MaximumWork&&
      budget.Used<=MAX_uint64-(reserve+DegreeReduction::MaximumWork))
   {
    ++reductionAttempts;const uint64 before=budget.Used;
    DegreeReduction::FPointwiseRoot reduced;DegreeReduction::EResult result;
    {
     FScopedDistanceBudgetFloor floor(budget,budget.Remaining-int32(DegreeReduction::MaximumWork));
     if(!floor.IsValid())return finish(TEXT("reduction_floor_invalid"));
     result=DegreeReduction::TryBuild(input[plan.Provider],!plan.AlongU,budget,reduced);
    }
    reductionWork+=budget.Used-before;
    if(result==DegreeReduction::EResult::Work||result==DegreeReduction::EResult::Invalid)
     return finish(TEXT("reduction_work_or_numeric_not_published"));
    if(result==DegreeReduction::EResult::Ready)
    {
     // Outer pointwise geometry ONLY: subdivision/hull/projection preserve it.
     // No normal/rank/inside/contact witness may be inferred from this net.
     root.DegreeU=reduced.DegreeU;root.DegreeV=reduced.DegreeV;root.Net=MoveTemp(reduced.Net);++reductionReady;
    }
    else ++reductionUnavailable;
   }
   else ++reductionSkipped;
  }
  const int32 at=build(build,root,plan.AlongU,plan.Extrusion,depth);if(at==INDEX_NONE)return finish(TEXT("incomplete_tree_not_published"));
  atlas->Roots[plan.Provider]=at;++completedProviders;
 }
 retainedBytes=atlas->AllocatedBytes();
 if(retainedBytes>Private::MaximumBytes)return finish(TEXT("allocator_bytes_cap_not_published"));
 const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
 if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=atlas->Numeric)return finish(TEXT("numeric_changed_not_published"));
 // Diagnostic history only. This prepared geometry is NOT a replay of
 // frame arithmetic; initialization status flags are not worker preconditions.
 atlas->Sticky=VectorGetControlRegister()&0x3f;
 return finish(TEXT("complete"),atlas);
}
EFrontier BuildFrontier(const FIndexedIntervalUnion& scene,int32 provider,const FGroupedBoxVolumeBounds& volume,
 uint32 active,FDistanceBudget& budget,TArray<FLeaf,TInlineAllocator<16>>& leaves)
{
 leaves.Reset();
 if(volume.Groups.Num()==1)
 {
  const auto tubes=StaticBodyTubes::TryClear(scene,provider,volume,active,budget);
  if(tubes==StaticBodyTubes::EResult::Clear)return EFrontier::Ready;
  if(tubes==StaticBodyTubes::EResult::Invalid)return EFrontier::Invalid;
  return EFrontier::Unavailable;
 }
 if(volume.Groups.Num()==14)
 {
  const auto tube=StaticBodyTubes::TryBuildFrontier(scene,provider,volume,active,budget,leaves);
  if(tube!=EFrontier::Unavailable)return tube;
 }
 const bool observe=StrongConvexCurveMinimum::Observation::Enabled();
 const uint64 began=observe?FPlatformTime::Cycles64():0,initial=budget.Used;auto& counts=Private::Counts;if(observe)++counts.Calls;
 const auto finish=[&](EFrontier status)
 {
  if(status!=EFrontier::Ready)leaves.Reset();
  if(observe)
  {
   counts.Ticks.fetch_add(FPlatformTime::Cycles64()-began);if(budget.Used>=initial)counts.Work.fetch_add(budget.Used-initial);
   if(status==EFrontier::Ready){++counts.Ready;if(leaves.IsEmpty())++counts.ProvidersClear;}
   else if(status==EFrontier::Unavailable)++counts.Unavailable;else if(status==EFrontier::Work)++counts.WorkFailure;else ++counts.Invalid;
  }
  return status;
 };
 const auto& atlas=scene.GetStaticBodyHierarchy();
 if(!atlas||!atlas->Roots.IsValidIndex(provider)||atlas->Roots[provider]==INDEX_NONE)return finish(EFrontier::Unavailable);
 const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
 if(numeric.Status!=EAnalyticNumericStatus::Complete)return finish(EFrontier::Invalid);
 if(numeric.ControlFingerprint()!=atlas->Numeric){if(observe)++counts.NumericMisses;return finish(EFrontier::Unavailable);}
 // Masked status flags do not select arithmetic (Numeric::ControlFingerprint).
 // Preserve the worker's own flags: neither require nor replay GT history.
 // The full control fingerprint, current Complete status and final check stay.
 if(observe)
 {
  const uint32 sticky=VectorGetControlRegister()&0x3f;
  counts.InitStickyOr.fetch_or(atlas->Sticky);counts.QueryStickyOr.fetch_or(sticky);
  if(sticky!=atlas->Sticky)++counts.StickyDifferences;
 }
 if(volume.Groups.IsEmpty()||volume.Groups.Num()>16||volume.Axes.IsEmpty()||volume.Axes.Num()>12||
  volume.Slabs.Num()!=volume.Groups.Num()*volume.Axes.Num()||
  !active||(active&~((1u<<volume.Groups.Num())-1)))return finish(EFrontier::Invalid);
 // The compact hierarchy specializes the six original axes. The generic
 // proof still checks every added axis on its complete retained domains.
 if(volume.Axes.Num()!=6)return finish(EFrontier::Unavailable);
 for(int32 a=0;a<3;++a){FVector3d axis=FVector3d::ZeroVector;axis[a]=1.;if(!Private::SameAxis(axis,volume.Axes[a]))return finish(EFrontier::Unavailable);}
 for(const auto slab:volume.Slabs)if(!::AnalyticCertified::Valid(slab))return finish(EFrontier::Invalid);
 const bool useShared=Private::SharedSupportMode.GetValueOnAnyThread()==1&&atlas->PreparedDepth==3&&!atlas->SupportFrames.IsEmpty();
 Private::FSharedQuerySupports supportCache(observe);if(observe&&useShared)++Private::SharedCounts.Queries;
 struct FWork{int32 Node;uint32 Active;};
 TArray<FWork,TInlineAllocator<8>> stack;stack.Add({atlas->Roots[provider],active});
 while(!stack.IsEmpty())
 {
  auto work=stack.Pop(EAllowShrinking::No);if(!budget.Consume())return finish(EFrontier::Work);
  if(observe)++counts.Nodes;
  const auto& node=atlas->Nodes[work.Node];
  Interval sharedProjected[3];uint8 sharedMask=0;bool sharedAttempted=false;
  for(int32 g=0;g<volume.Groups.Num();++g)if(work.Active&(1u<<g))
  {
   if(!budget.Consume())return finish(EFrontier::Work);if(observe)++counts.GroupTests;
   bool separate=false;
   for(int32 d=0;d<3;++d){const auto b=volume.Slabs[g*6+d],s=node.Hull.Axis[d];separate|=b.Upper<s.Lower||s.Upper<b.Lower;}
   if(!separate&&useShared&&node.SupportFrame!=INDEX_NONE)
   {
    if(!atlas->SupportFrames.IsValidIndex(node.SupportFrame)||!Private::A::Valid(node.NormalSupport)||!Private::A::Valid(node.TangentSupport))return finish(EFrontier::Invalid);
    if(!sharedAttempted)
    {
     sharedAttempted=true;
     supportCache.ProjectNode(node.SupportFrame,atlas->SupportFrames[node.SupportFrame],node.NormalSupport,node.TangentSupport,node.Hull,volume,budget,sharedProjected,sharedMask);
    }
    for(int32 k=0;k<3;++k)if(sharedMask&(1u<<k))
    {if(observe)++Private::SharedCounts.Tests;const auto b=volume.Slabs[g*6+k+3],s=sharedProjected[k];separate|=b.Upper<s.Lower||s.Upper<b.Lower;}
    if(separate&&observe)++Private::SharedCounts.Clears;
   }
   if(separate){work.Active&=~(1u<<g);if(observe)++counts.GroupClears;}
  }
  if(!work.Active)continue;
  if(node.FirstPoint==INDEX_NONE)
  {stack.Add({node.Right,work.Active});stack.Add({node.Left,work.Active});continue;}
  // Paid owned value copy, not a new source and not a scalar-root memo seed.
  if(!Private::Pay(budget,node.PointCount))return finish(EFrontier::Work);
  FLeaf leaf;leaf.ActiveGroups=work.Active;auto& cell=leaf.Cell;
  cell.DegreeU=node.DegreeU;cell.DegreeV=node.DegreeV;cell.DepthU=node.DepthU;cell.DepthV=node.DepthV;
  cell.MinU=node.MinU;cell.MaxU=node.MaxU;cell.MinV=node.MinV;cell.MaxV=node.MaxV;
  cell.Net.Append(atlas->Points.GetData()+node.FirstPoint,node.PointCount);leaves.Add(MoveTemp(leaf));if(observe)++counts.LeavesLoaded;
 }
 const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
 if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=numeric.ControlFingerprint())return finish(EFrontier::Invalid);
 return finish(EFrontier::Ready);
}
}



namespace Speed::Analytic::Certified
{
void FIndexedIntervalUnion::PrepareStaticBodyHierarchyAtPublication(FDistanceBudget& budget) const
{
 // Callers own the final GT publication boundary. No mandatory publication
 // work follows this optional preparation and no query can initialize it.
 if(!IsInGameThread()||StaticBodyHierarchy::Private::Mode.GetValueOnAnyThread()<1||
    StaticBodyHierarchy::Private::Mode.GetValueOnAnyThread()>3)return;
 uint8 expected=0;
 if(!StaticHierarchyState.compare_exchange_strong(expected,uint8(1),std::memory_order_acq_rel))return;
 struct FSeal
 {
  std::atomic<uint8>& State;
  ~FSeal(){State.store(uint8(2),std::memory_order_release);}
 } seal{StaticHierarchyState};
 try{StaticHierarchy=StaticBodyHierarchy::Prepare(MakeArrayView(Surfaces),budget);}
 catch(...)
 {
  StaticHierarchy.Reset();
  UE_LOG(LogTemp,Display,TEXT("[StaticBodyHierarchyBuild] Status=exception_not_published PublicationOnly=1 SourceUnchanged=1 NoRefund=1"));
 }
}
}


#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Analytic::Certified::StaticBodyHierarchyTests
{
namespace A=::AnalyticCertified;
static FIntervalTensorSurface Surface(bool transpose=false,double width=0,double zShift=0)
{
 FIntervalTensorSurface s;s.DegreeU=s.DegreeV=5;s.ControlPoints.SetNum(36);
 for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)
 {
  const double z=3.-4.*double(v)/5.+double(v*(v-1))/5.+zShift;
  auto p=Detail::Point(FVector3d(-2.+4.*double(u)/5.,0,z));
  if(width>0)for(auto& a:p.Axis){a.Lower-=width;a.Upper+=width;}
  s.ControlPoints[transpose?v*6+u:u*6+v]=p;
 }
 return s;
}
static bool Volume(double z,FGroupedBoxVolumeBounds& out,bool lastOnly=false,int32 count=14)
{
 TArray<FBoxVolumeGroup> groups;FIntervalPoint hull;
 for(int32 g=0;g<count;++g)
 {
  FBoxVolumeGroup group;
  for(int32 bits=0;bits<8;++bits)
  {
   const auto p=Detail::Point(FVector3d(bits&1?.1:-.1,bits&2?.1:-.1,
    z+(bits&4?.1:-.1)-(lastOnly&&g+1<count?10.:0.)));
   group.Corners.Add(p);hull=Detail::ValidPoint(hull)?TimeDetail::Hull(hull,p):p;
  }
  groups.Add(MoveTemp(group));
 }
 TArray<FVector3d> axes{FVector3d(1,0,0),FVector3d(0,1,0),FVector3d(0,0,1),
  FVector3d(1,1,0),FVector3d(0,1,1),FVector3d(1,0,1)};
 FDistanceBudget b{10000};return CloseGroupedBoxVolumeBounds(hull,groups,axes,b,out);
}
static bool Build(TConstArrayView<FIntervalTensorSurface> input,FDistanceBudget& budget,FIndexedIntervalUnionPtr& scene,bool prepare=false)
{
 if(!FIndexedIntervalUnion::Build(input,budget,scene))return false;
 if(prepare)scene->PrepareStaticBodyHierarchyAtPublication(budget);
 return true;
}
static bool SameNode(const Detail::FNode& a,const Detail::FNode& b)
{
 if(a.DegreeU!=b.DegreeU||a.DegreeV!=b.DegreeV||a.DepthU!=b.DepthU||a.DepthV!=b.DepthV||
 a.Net.Num()!=b.Net.Num()||A::Bits(a.MinU)!=A::Bits(b.MinU)||A::Bits(a.MaxU)!=A::Bits(b.MaxU)||
 A::Bits(a.MinV)!=A::Bits(b.MinV)||A::Bits(a.MaxV)!=A::Bits(b.MaxV))return false;
 for(int32 i=0;i<a.Net.Num();++i)if(!GroupedBoxDetail::SamePoint(a.Net[i],b.Net[i]))return false;
 return true;
}
static bool ExpectedLeaves(const FIntervalTensorSurface& surface,bool alongU,TArray<Detail::FNode>& out,int32 maximumDepth)
{
 Detail::FNode root;root.DegreeU=root.DegreeV=5;root.Net=surface.ControlPoints;out={root};
 for(int32 depth=0;depth<maximumDepth;++depth)
 {
  TArray<Detail::FNode> next;
  for(const auto& node:out){Detail::FNode a,b;if(!Detail::Split(node,alongU,a,b))return false;next.Add(MoveTemp(a));next.Add(MoveTemp(b));}
  out=MoveTemp(next);
 }
 return true;
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStaticBodyHierarchyDomainTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.StaticHierarchyOriginalDomains",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStaticBodyHierarchyDomainTest::RunTest(const FString&)
{
 FScopedCVar<int32> directionalOracleOff(TEXT("p.IAmSpeed.AnalyticWorld.NativeDirectionalMotionClearance"),0,FName(TEXT("FStaticBodyHierarchyDomainTestDirectionalOracle")));
 FScopedCVar<int32> sharedOff(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.SharedSupportSAT"),0,FName(TEXT("SharedSupportHistoricalOracle")));
 using namespace Speed;using namespace Analytic;using namespace Certified;namespace H=StaticBodyHierarchy;namespace T=StaticBodyHierarchyTests;
 const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("RNE original-domain preparation"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 FScopedCVar<int32> reductionOff(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.DegreeReduction"),0);
 FScopedCVar<int32> mode(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy"),1),depth(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.Depth"),3);
 const uint32 saved=VectorGetControlRegister();
 for(int32 depthValue:{3,5})for(uint32 flush:{0u,64u,32768u,32832u})for(bool transpose:{false,true})for(double width:{0.,1./1048576.})
 {
  VectorSetControlRegister((saved&~uint32(0x8040))|flush);
  FScopedCVar<int32> depthCase(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.Depth"),depthValue,FName(TEXT("StaticHierarchyDomainDepthCase")));
  const auto source=T::Surface(transpose,width);FDistanceBudget create{100000};
  FIndexedIntervalUnionPtr scene;if(!T::Build(MakeArrayView(&source,1),create,scene,true)||!scene->GetStaticBodyHierarchy())return false;
  const uint64 count=uint64(1)<<depthValue;
  if(!TestTrue(TEXT("depth3/5 initialization is fully charged and memory remains bounded"),
   create.Used==74+(2*count-1)*37+(count-1)*272+count*36&&
   scene->GetStaticBodyHierarchy()->AllocatedBytes()<=StaticBodyHierarchy::Private::MaximumBytes))return false;
  FGroupedBoxVolumeBounds volume;if(!T::Volume(2.25,volume,true))return false;
  TArray<Detail::FNode> expected;if(!T::ExpectedLeaves(source,transpose,expected,depthValue))return false;
  TArray<H::FLeaf,TInlineAllocator<16>> leaves;FDistanceBudget query{100000,9,17};
  if(!TestTrue(TEXT("complete static prefix returns only owned ambiguous leaves"),
   H::BuildFrontier(*scene,0,volume,(1u<<14)-1,query,leaves)==H::EFrontier::Ready&&!leaves.IsEmpty()&&leaves.Num()<(1<<depthValue)&&query.ConsumptionFloor==17))return false;
  int32 prior=-1;
  for(const auto& leaf:leaves)
  {
   int32 match=INDEX_NONE;for(int32 i=0;i<expected.Num();++i)if(T::SameNode(leaf.Cell,expected[i])){match=i;break;}
   if(!TestTrue(TEXT("leaf controls and global dyadic domain match literal original restrictions in deterministic order"),
    match>prior&&match!=INDEX_NONE&&leaf.ActiveGroups==(1u<<13)))return false;prior=match;
  }
  for(int32 k=0;k<=64;++k)
  {
   const double v=double(k)/64.,z=3.-4.*v+4.*v*v;
   if(z<2.15||z>2.35)continue;
   bool covered=false;for(const auto& leaf:leaves)
    covered|=transpose?(leaf.Cell.MinU<=v&&v<=leaf.Cell.MaxU):(leaf.Cell.MinV<=v&&v<=leaf.Cell.MaxV);
   if(!TestTrue(TEXT("every actual surface point inside the last group remains in a returned closed domain"),covered))return false;
  }
  const auto keep=leaves[0].Cell;
  FGroupedBoxVolumeBounds clearVolume;if(!T::Volume(1.75,clearVolume))return false;FDistanceBudget clear{100000};
  TArray<H::FLeaf,TInlineAllocator<16>> none;
  if(!TestTrue(TEXT("true curved arch clearance discharges every branch and every group"),
   H::BuildFrontier(*scene,0,clearVolume,(1u<<14)-1,clear,none)==H::EFrontier::Ready&&none.IsEmpty()&&T::SameNode(keep,leaves[0].Cell)))return false;
 }
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStaticBodyHierarchyLifetimeBudgetTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.StaticHierarchyPublicationAndBudget",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStaticBodyHierarchyLifetimeBudgetTest::RunTest(const FString&)
{
 FScopedCVar<int32> directionalOracleOff(TEXT("p.IAmSpeed.AnalyticWorld.NativeDirectionalMotionClearance"),0,FName(TEXT("FStaticBodyHierarchyLifetimeBudgetTestDirectionalOracle")));
 FScopedCVar<int32> sharedOff(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.SharedSupportSAT"),0,FName(TEXT("SharedSupportHistoricalOracle")));
 using namespace Speed;using namespace Analytic;using namespace Certified;namespace H=StaticBodyHierarchy;namespace T=StaticBodyHierarchyTests;
 const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("RNE lifetime and budget"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 FScopedCVar<int32> reductionOff(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.DegreeReduction"),0);
 FScopedCVar<int32> mode(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy"),1),depth(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.Depth"),3);
 auto source=T::Surface();FIndexedIntervalUnionPtr ordinary,scene;FDistanceBudget cold{100000},create{100000};
 if(!TestTrue(TEXT("ordinary source/readback build never performs hidden hierarchy initialization"),
  T::Build(MakeArrayView(&source,1),cold,ordinary)&&!ordinary->GetStaticBodyHierarchy()&&cold.Used==37&&
  T::Build(MakeArrayView(&source,1),create,scene,true)&&scene->GetStaticBodyHierarchy()&&create.Used==2821))return false;
 const auto once=scene->GetStaticBodyHierarchy();FDistanceBudget second{100000,13,17};scene->PrepareStaticBodyHierarchyAtPublication(second);
 if(!TestTrue(TEXT("successful publication is one-shot without a second charge"),scene->GetStaticBodyHierarchy()==once&&second.Used==13))return false;
 FGroupedBoxVolumeBounds volume;if(!T::Volume(2.25,volume))return false;
 TArray<H::FLeaf,TInlineAllocator<16>> leaves;FDistanceBudget q{100000};
 if(H::BuildFrontier(*scene,0,volume,16383,q,leaves)!=H::EFrontier::Ready||leaves.IsEmpty())return false;
 const auto retained=leaves[0].Cell;
 source.ControlPoints[0].Axis[2].Lower-=1;
 FDistanceBudget again{100000};
 if(!TestTrue(TEXT("sourcecaller mutation cannot alter the published tree or its owned output"),
  H::BuildFrontier(*scene,0,volume,16383,again,leaves)==H::EFrontier::Ready&&T::SameNode(retained,leaves[0].Cell)))return false;
 for(int32 available:{0,1,7,31})
 {
  FDistanceBudget tight{available+17,11,17};leaves.Add({});
  if(!TestTrue(TEXT("partial traversal never publishes an incomplete frontier or refunds work"),
   H::BuildFrontier(*scene,0,volume,16383,tight,leaves)==H::EFrontier::Work&&leaves.IsEmpty()&&tight.Remaining>=17&&
   tight.ConsumptionFloor==17&&tight.Used+uint64(tight.Remaining)==uint64(available+28)))return false;
 }
 const uint32 saved=VectorGetControlRegister();VectorSetControlRegister(saved^uint32(0x8000));FDistanceBudget mismatch{100000,3};
 if(!TestTrue(TEXT("worker control mismatch uses original fallback without retained tree"),
  H::BuildFrontier(*scene,0,volume,16383,mismatch,leaves)==H::EFrontier::Unavailable&&leaves.IsEmpty()&&mismatch.Used==3))return false;
 VectorSetControlRegister(saved);FDistanceBudget missing{100000,3};
 if(!TestTrue(TEXT("unprepared ordinary snapshot remains literal fallback"),H::BuildFrontier(*ordinary,0,volume,16383,missing,leaves)==H::EFrontier::Unavailable&&missing.Used==3))return false;
 TWeakPtr<const FIndexedIntervalUnion,ESPMode::ThreadSafe> weak=scene;auto atlas=scene->GetStaticBodyHierarchy();scene.Reset();
 if(!TestTrue(TEXT("atlas values do not retain the publication source in a reference cycle"),!weak.IsValid()&&bool(atlas)))return false;
 source=T::Surface();FDistanceBudget limited{74};FIndexedIntervalUnionPtr without;
 if(!TestTrue(TEXT("init forecast refuses before allocating an incomplete tree and retains actual recognition debit"),
  T::Build(MakeArrayView(&source,1),limited,without,true)&&!without->GetStaticBodyHierarchy()&&limited.Used==74))return false;
 FDistanceBudget retry{100000,13,17};without->PrepareStaticBodyHierarchyAtPublication(retry);
 if(!TestTrue(TEXT("unavailable publication is sealed with no hidden retry or refund"),!without->GetStaticBodyHierarchy()&&retry.Used==13))return false;
 TArray<FIntervalTensorSurface> tooMany;tooMany.Init(source,257);FDistanceBudget capped{1000000};FIndexedIntervalUnionPtr oversized;
 if(!TestTrue(TEXT("provider storage cap is checked before any forest publication"),
  T::Build(tooMany,capped,oversized,true)&&!oversized->GetStaticBodyHierarchy()))return false;
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStaticBodyHierarchyCompleteProofTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.StaticHierarchyFineProof",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStaticBodyHierarchyCompleteProofTest::RunTest(const FString&)
{
 FScopedCVar<int32> directionalOracleOff(TEXT("p.IAmSpeed.AnalyticWorld.NativeDirectionalMotionClearance"),0,FName(TEXT("FStaticBodyHierarchyCompleteProofTestDirectionalOracle")));
 FScopedCVar<int32> sharedOff(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.SharedSupportSAT"),0,FName(TEXT("SharedSupportHistoricalOracle")));
 using namespace Speed;using namespace Analytic;using namespace Certified;namespace H=StaticBodyHierarchy;namespace T=StaticBodyHierarchyTests;
 const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("RNE full proof"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 FScopedCVar<int32> reductionOff(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.DegreeReduction"),0);
 FScopedCVar<int32> mode(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy"),1),depth(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.Depth"),3);
 const auto source=T::Surface();FDistanceBudget setup{100000};FIndexedIntervalUnionPtr scene;
 if(!T::Build(MakeArrayView(&source,1),setup,scene,true)||!scene->GetStaticBodyHierarchy())return false;
 for(int32 scalarMode:{0,2})for(double z:{1.75,2.0})
 {
  FGroupedBoxVolumeBounds volume;if(!T::Volume(z,volume))return false;FReferenceBoundsCandidateSetPtr partition;
  if(!FReferenceBoundsCandidateSet::Select(scene,volume.BroadHull,setup,partition))return false;
  FReferenceBoxVolumeStrictSeparationPtr oldProof,newProof;FDistanceBudget oldWork{20000,9,17},newWork{20000,9,17};bool oldResult=false;
  FGroupedBoxSeparationObservation oldObservation,newObservation;
  {FScopedCVar<int32> off(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy"),0,FName(TEXT("StaticHierarchyFineOracleOff")));
   oldResult=FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,oldWork,oldProof,&oldObservation,false,false,false,scalarMode,false);}
  if(!TestTrue(TEXT("fine atlas mode remains active after the independent OFF oracle"),H::RuntimeEnabledFor(14)&&!H::RuntimeEnabledFor(8)))return false;
  const bool result=FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,newWork,newProof,&newObservation,false,false,false,scalarMode,false);
  if(z==1.75&&!TestTrue(TEXT("static hierarchy actually discharges the original root before recursive solver cells"),newObservation.VisitedCells==1&&oldObservation.VisitedCells>1))return false;
  if(!TestTrue(TEXT("same full original scene and unchanged endpoint consumer, with no fabricated collision result"),
   result==oldResult&&result==(z==1.75)&&newWork.ConsumptionFloor==17&&newWork.Remaining>=17&&
   (result?(newProof->Matches(scene,volume)&&newProof->ContainsCompleteEndpointGroup(volume.Groups[0].Corners)):!newProof)))return false;
 }
 if(!TestTrue(TEXT("mode1 targets fine14 and never changes the coarse8 attempt"),H::RuntimeEnabledFor(14)&&!H::RuntimeEnabledFor(8)))return false;
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
#endif


#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStaticBodyHierarchyReducedIntegrationTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.StaticHierarchyPointwiseReducedIntegration",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStaticBodyHierarchyReducedIntegrationTest::RunTest(const FString&)
{
 FScopedCVar<int32> directionalOracleOff(TEXT("p.IAmSpeed.AnalyticWorld.NativeDirectionalMotionClearance"),0,FName(TEXT("FStaticBodyHierarchyReducedIntegrationTestDirectionalOracle")));
 FScopedCVar<int32> sharedOff(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.SharedSupportSAT"),0,FName(TEXT("SharedSupportHistoricalOracle")));
 using namespace Speed;using namespace Analytic;using namespace Certified;
 namespace H=StaticBodyHierarchy;namespace D=H::DegreeReduction;namespace T=StaticBodyHierarchyTests;
 const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("RNE reduced atlas integration"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 FScopedCVar<int32> mode(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy"),1,FName(TEXT("ReducedAtlasIntegrationMode"))),
  reduction(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.DegreeReduction"),1,FName(TEXT("ReducedAtlasIntegrationReduction")));
 const uint32 saved=VectorGetControlRegister();int32 cases=0;
 for(int32 depthValue:{3,5})for(uint32 flush:{0u,64u,32768u,32832u})for(bool transpose:{false,true})for(double width:{0.,1./1048576.})
 {
  VectorSetControlRegister((saved&~uint32(0x8040))|flush);
  FScopedCVar<int32> depth(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.Depth"),depthValue,FName(TEXT("ReducedAtlasIntegrationDepth")));
  const auto source=T::Surface(transpose,width);FIndexedIntervalUnionPtr scene;FDistanceBudget setup{100000};
  if(!T::Build(MakeArrayView(&source,1),setup,scene,true)||!scene->GetStaticBodyHierarchy())return false;
  const uint64 count=uint64(1)<<depthValue;
  if(!TestTrue(TEXT("reduction and true smaller forest construction are each charged once"),
   setup.Used==74+D::MaximumWork+(2*count-1)*13+(count-1)*92+count*12&&
   scene->GetStaticBodyHierarchy()->AllocatedBytes()<=H::Private::MaximumBytes))return false;
  FIntervalTensorSurface unchanged;FDistanceBudget readback{1000};
  if(!TestTrue(TEXT("public source still owns all36 original degree5x5 control boxes"),
   scene->CopySurfaceForQuery(0,readback,unchanged)&&unchanged.DegreeU==5&&unchanged.DegreeV==5&&unchanged.ControlPoints.Num()==36))return false;
  for(int32 i=0;i<36;++i)if(!TestTrue(TEXT("original source bits never replaced by reduced outer values"),GroupedBoxDetail::SamePoint(source.ControlPoints[i],unchanged.ControlPoints[i])))return false;
  D::FPointwiseRoot reduced;FDistanceBudget reductionWork{1000};
  if(D::TryBuild(source,!transpose,reductionWork,reduced)!=D::EResult::Ready)return false;
  Detail::FNode root;root.DegreeU=reduced.DegreeU;root.DegreeV=reduced.DegreeV;root.Net=reduced.Net;
  TArray<Detail::FNode> expected{root};
  for(int32 level=0;level<depthValue;++level)
  {
   TArray<Detail::FNode> next;
   for(const auto& cell:expected){Detail::FNode a,b;if(!Detail::Split(cell,transpose,a,b))return false;next.Add(MoveTemp(a));next.Add(MoveTemp(b));}
   expected=MoveTemp(next);
  }
  FGroupedBoxVolumeBounds crossing;if(!T::Volume(2.25,crossing,true))return false;
  TArray<H::FLeaf,TInlineAllocator<16>> leaves;FDistanceBudget query{100000,9,17};
  if(!TestTrue(TEXT("reduced atlas returns the complete ambiguous frontier under the same account"),
   H::BuildFrontier(*scene,0,crossing,16383,query,leaves)==H::EFrontier::Ready&&!leaves.IsEmpty()&&leaves.Num()<(1<<depthValue)&&query.ConsumptionFloor==17))return false;
  int32 prior=-1;
  for(const auto& leaf:leaves)
  {
   int32 match=INDEX_NONE;for(int32 i=0;i<expected.Num();++i)if(T::SameNode(leaf.Cell,expected[i])){match=i;break;}
   if(!TestTrue(TEXT("actual12CP metadata, deterministic domains and inherited final-group mask"),
    leaf.Cell.DegreeU==(transpose?5:1)&&leaf.Cell.DegreeV==(transpose?1:5)&&leaf.Cell.Net.Num()==12&&
    match>prior&&match!=INDEX_NONE&&leaf.ActiveGroups==(1u<<13)))return false;prior=match;
  }
  for(int32 k=0;k<=64;++k)
  {
   const double v=double(k)/64.,z=3.-4.*v+4.*v*v;if(z<2.15||z>2.35)continue;
   bool covered=false;for(const auto& leaf:leaves)covered|=transpose?(leaf.Cell.MinU<=v&&v<=leaf.Cell.MaxU):(leaf.Cell.MinV<=v&&v<=leaf.Cell.MaxV);
   if(!TestTrue(TEXT("same-UV pointwise coverage includes all actual ambiguous samples"),covered))return false;
  }
  if(width==0.)for(int32 scalarMode:{0,2})for(double z:{1.75,2.0})
  {
   FGroupedBoxVolumeBounds volume;if(!T::Volume(z,volume))return false;FReferenceBoundsCandidateSetPtr partition;FDistanceBudget select{10000};
   if(!FReferenceBoundsCandidateSet::Select(scene,volume.BroadHull,select,partition))return false;
   FReferenceBoxVolumeStrictSeparationPtr oracle,proof;FDistanceBudget oldWork{20000,9,17},newWork{20000,9,17};
   FGroupedBoxSeparationObservation oldObserved,newObserved;bool oldResult=false;
   {FScopedCVar<int32> off(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy"),0,FName(TEXT("ReducedAtlasIndependentOracle")));
    oldResult=FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,oldWork,oracle,&oldObserved,false,false,false,scalarMode,false);}
   if(!TestTrue(TEXT("candidate mode survives oracle temporary slot"),H::RuntimeEnabledFor(14)&&!H::RuntimeEnabledFor(8)))return false;
   const bool result=FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,newWork,proof,&newObserved,false,false,false,scalarMode,false);
   if(!TestTrue(TEXT("complete scene clearance and unchanged consumer; no intersection is declared clear"),
    result==oldResult&&result==(z==1.75)&&newWork.Remaining>=17&&newWork.ConsumptionFloor==17&&
    (result?(proof->Matches(scene,volume)&&proof->ContainsCompleteEndpointGroup(volume.Groups[0].Corners)):!proof)))return false;
   if(z==1.75&&!TestTrue(TEXT("real atlas route eliminates recursion with scalar mode0 and2"),newObserved.VisitedCells==1&&oldObserved.VisitedCells>1))return false;
  }
  ++cases;
 }
 if(!TestTrue(TEXT("32 actual reduced publication contexts"),cases==32))return false;
 VectorSetControlRegister(saved);
 FScopedCVar<int32> depth(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.Depth"),3,FName(TEXT("ReducedAtlasBudgetDepth")));
 const auto source=T::Surface();FGroupedBoxVolumeBounds crossing;if(!T::Volume(2.25,crossing))return false;
 // Both remaining-space and Used-overflow forecasts must preserve the ENTIRE
 // original forest, not only the optional394-unit attempt or a minimal root.
 for(bool usedBoundary:{false,true})
 {
  FDistanceBudget setup=usedBoundary?FDistanceBudget{100000,MAX_uint64-2821,17}:FDistanceBudget{2821+17,11,17};
  const uint64 before=setup.Used;FIndexedIntervalUnionPtr scene;
  if(!TestTrue(TEXT("optional reduction reserve-skip leaves original atlas fully constructible"),
   T::Build(MakeArrayView(&source,1),setup,scene,true)&&scene->GetStaticBodyHierarchy()&&setup.Used-before==2821&&
   setup.ConsumptionFloor==17&&setup.Remaining>=17&&(usedBoundary?setup.Used==MAX_uint64:setup.Remaining==17)))return false;
  TArray<H::FLeaf,TInlineAllocator<16>> leaves;FDistanceBudget query{100000};
  if(H::BuildFrontier(*scene,0,crossing,16383,query,leaves)!=H::EFrontier::Ready||leaves.IsEmpty())return false;
  for(const auto& leaf:leaves)if(!TestTrue(TEXT("reserve skip retained original36CP leaf geometry"),leaf.Cell.DegreeU==5&&leaf.Cell.DegreeV==5&&leaf.Cell.Net.Num()==36))return false;
  const auto atlas=scene->GetStaticBodyHierarchy();FDistanceBudget second{100000,13,17};scene->PrepareStaticBodyHierarchyAtPublication(second);
  if(!TestTrue(TEXT("reserve skip is a sealed one-shot publication, not a delayed reduction retry"),scene->GetStaticBodyHierarchy()==atlas&&second.Used==13))return false;
 }
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Analytic::Certified::StaticBodyDegreeReductionTests
{
namespace D=StaticBodyHierarchy::DegreeReduction;
namespace A=::AnalyticCertified;
static FIntervalTensorSurface Surface(bool reduceU,bool nonlinear,double width)
{
 FIntervalTensorSurface s;s.DegreeU=s.DegreeV=5;s.ControlPoints.SetNum(36);
 for(int32 i=0;i<6;++i)for(int32 j=0;j<6;++j)
 {
  auto p=Detail::Point(FVector3d(2*i+5*j,j*j,3+2*j+i*j+(nonlinear?i*i:2*i)));
  for(auto& a:p.Axis){a.Lower-=width;a.Upper+=width;}
  s.ControlPoints[reduceU?i*6+j:j*6+i]=p;
 }
 return s;
}
static uint64 Power(uint64 n,int32 exponent){uint64 r=1;for(int32 k=0;k<exponent;++k)r*=n;return r;}
// Exact rational oracle: inputs are dyadics with denominator1024; parameters
// have denominator8, so the common denominator is2^40. Every integer product
// and accumulated numerator below is <2^53, hence binary64 adds are exact.
static FVector3d OriginalPoint(const FIntervalTensorSurface& s,int32 ku,int32 kv,int32 realization)
{
 static const uint64 choose[6]={1,5,10,10,5,1};FVector3d numerator=FVector3d::ZeroVector;
 for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)
 {
  const uint64 wu=choose[u]*Power(uint64(ku),u)*Power(uint64(8-ku),5-u);
  const uint64 wv=choose[v]*Power(uint64(kv),v)*Power(uint64(8-kv),5-v);
  for(int32 d=0;d<3;++d)
  {
   const auto a=s.ControlPoints[u*6+v].Axis[d];
   const bool upper=realization==1||(realization==2&&((u+2*v+d)%3==0));
   numerator[d]+=(upper?a.Upper:a.Lower)*1024.*double(wu*wv);
  }
 }
 return numerator/1099511627776.;
}
static bool Contains(const FIntervalPoint& bounds,FVector3d p)
{
 if(!Detail::ValidPoint(bounds))return false;
 for(int32 d=0;d<3;++d)if(p[d]<bounds.Axis[d].Lower||p[d]>bounds.Axis[d].Upper)return false;
 return true;
}
// Test-only adapter. Production retains the distinct pointwise-root type and
// never passes this net to a contact/rank/derivative or inward-witness API.
static bool Evaluate(const Detail::FNode& n,FVector2d uv,FIntervalPoint& out)
{
 FIntervalTensorSurface test;
 test.DegreeU=n.DegreeU;test.DegreeV=n.DegreeV;test.ControlPoints=n.Net;
 return ProjectedDistanceDetail::EvaluateFixedUV(test,uv,out);
}
static Detail::FNode Node(const D::FPointwiseRoot& value)
{Detail::FNode n;n.DegreeU=value.DegreeU;n.DegreeV=value.DegreeV;n.Net=value.Net;return n;}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStaticBodyDegreeReductionPointwiseTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.StaticDegreeReductionPointwise",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStaticBodyDegreeReductionPointwiseTest::RunTest(const FString&)
{
 using namespace Speed;using namespace Analytic;using namespace Certified;namespace D=StaticBodyHierarchy::DegreeReduction;namespace T=StaticBodyDegreeReductionTests;
 const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("RNE universal pointwise enclosure"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 const uint32 saved=VectorGetControlRegister();
 for(uint32 flush:{0u,64u,32768u,32832u})for(bool reduceU:{false,true})for(bool nonlinear:{false,true})for(double width:{0.,1./1024.})
 {
  VectorSetControlRegister((saved&~uint32(0x8040))|flush);
  const auto surface=T::Surface(reduceU,nonlinear,width);D::FPointwiseRoot reduced;FDistanceBudget budget{411,13,17};
  if(!TestTrue(TEXT("12 proof-only coefficients with exact unchanged work and floor"),D::TryBuild(surface,reduceU,budget,reduced)==D::EResult::Ready&&
   reduced.DegreeU==(reduceU?1:5)&&reduced.DegreeV==(reduceU?5:1)&&reduced.Net.Num()==12&&
   budget.Used==407&&budget.Remaining==17&&budget.ConsumptionFloor==17))return false;
  const auto root=T::Node(reduced);
  for(int32 ku=0;ku<=8;++ku)for(int32 kv=0;kv<=8;++kv)
  {
   FIntervalPoint value;if(!T::Evaluate(root,FVector2d(double(ku)/8.,double(kv)/8.),value))return false;
   for(int32 r=0;r<3;++r)if(!TestTrue(TEXT("original independent-family exact dyadic samples remain enclosed"),
    T::Contains(value,T::OriginalPoint(surface,ku,kv,r))))return false;
  }
  Detail::FNode left,right;if(!Detail::Split(root,true,left,right))return false;
  TArray<Detail::FNode> quarters;
  for(const auto& half:{left,right})
  {Detail::FNode a,b;if(!Detail::Split(half,false,a,b))return false;quarters.Add(MoveTemp(a));quarters.Add(MoveTemp(b));}
  for(const auto& cell:quarters)for(int32 iu=0;iu<=2;++iu)for(int32 iv=0;iv<=2;++iv)
  {
   const double u=.5*iu,v=.5*iv;
   const int32 ku=int32(8*(cell.MinU+(cell.MaxU-cell.MinU)*u)),kv=int32(8*(cell.MinV+(cell.MaxV-cell.MinV)*v));
   FIntervalPoint value;if(!T::Evaluate(cell,FVector2d(u,v),value))return false;
   for(int32 r=0;r<3;++r)if(!TestTrue(TEXT("dyadic restriction preserves pointwise coverage on the SAME original UV"),
    T::Contains(value,T::OriginalPoint(surface,ku,kv,r))))return false;
  }
 }
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStaticBodyDegreeReductionContractTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.StaticDegreeReductionNotDerivativeModel",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStaticBodyDegreeReductionContractTest::RunTest(const FString&)
{
 using namespace Speed;using namespace Analytic;using namespace Certified;namespace D=StaticBodyHierarchy::DegreeReduction;namespace T=StaticBodyDegreeReductionTests;
 const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("RNE nonlinear residual contract"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 const uint32 saved=VectorGetControlRegister();
 for(uint32 flush:{0u,64u,32768u,32832u})
 {
  VectorSetControlRegister((saved&~uint32(0x8040))|flush);
  auto surface=T::Surface(true,false,0);const double z[6]={0,4,6,6,4,0};
  for(int32 i=0;i<6;++i)for(int32 j=0;j<6;++j)surface.ControlPoints[i*6+j].Axis[2]=Interval::Exact(z[i]);
  D::FPointwiseRoot reduced;FDistanceBudget budget{394};if(D::TryBuild(surface,true,budget,reduced)!=D::EResult::Ready)return false;
  const auto root=T::Node(reduced);FIntervalPoint value;if(!T::Evaluate(root,FVector2d(.5,.5),value))return false;
  const auto reducedDerivative=::AnalyticCertified::Subtract(reduced.Net[6].Axis[2],reduced.Net[0].Axis[2]);
  if(!TestTrue(TEXT("20u(1-u) is covered at midpoint, but reduced derivative does NOT certify original derivative20"),
   value.Axis[2].Lower<=5&&value.Axis[2].Upper>=5&&reducedDerivative.Upper<7&&20>reducedDerivative.Upper))return false;
  // Different signed E_j preserve curvature-axis information. No common
// averaging, least-squares fit or tolerance may erase any of these residuals.
  for(int32 i=0;i<6;++i)for(int32 j=0;j<6;++j)surface.ControlPoints[i*6+j].Axis[2]=Interval::Exact((j&1)?-z[i]:z[i]);
  FDistanceBudget signedBudget{394};if(D::TryBuild(surface,true,signedBudget,reduced)!=D::EResult::Ready)return false;
  if(!TestTrue(TEXT("signed per-row residuals remain distinct and include their real extrema"),
   reduced.Residual[0].Axis[2].Upper>=6&&reduced.Residual[0].Axis[2].Lower>-1&&
   reduced.Residual[1].Axis[2].Lower<=-6&&reduced.Residual[1].Axis[2].Upper<1))return false;
 }
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStaticBodyDegreeReductionBudgetTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.StaticDegreeReductionBudgetAndRefusals",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStaticBodyDegreeReductionBudgetTest::RunTest(const FString&)
{
 using namespace Speed;using namespace Analytic;using namespace Certified;namespace D=StaticBodyHierarchy::DegreeReduction;namespace T=StaticBodyDegreeReductionTests;
 const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("RNE budget and refusal contracts"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 const auto source=T::Surface(true,false,0);D::FPointwiseRoot out;
 for(int32 available=0;available<394;++available)
 {
  out.Net.Add(Detail::Point(FVector3d::ZeroVector));FDistanceBudget budget{available+17,13,17};
  if(!TestTrue(TEXT("full finite forecast fails before work/allocation and clears stale output"),D::TryBuild(source,true,budget,out)==D::EResult::Unavailable&&
   out.Net.IsEmpty()&&budget.Used==13&&budget.Remaining==available+17&&budget.ConsumptionFloor==17))return false;
 }
 auto bad=source;bad.ControlPoints[3].Axis[1].Known=false;FDistanceBudget invalid{1000,9};
 if(!TestTrue(TEXT("invalid original coefficient is terminal and prior validation work remains spent"),
  D::TryBuild(bad,true,invalid,out)==D::EResult::Invalid&&out.Net.IsEmpty()&&invalid.Used>9))return false;
 auto huge=source;for(int32 i=0;i<6;++i)for(int32 j=0;j<6;++j)huge.ControlPoints[i*6+j].Axis[2]=Interval::Exact(i==0||i==5?1.e308:-1.e308);
 FDistanceBudget overflow{1000,7,17};
 if(!TestTrue(TEXT("optional residual overflow is paid unavailable, never an incomplete proof"),
  D::TryBuild(huge,true,overflow,out)==D::EResult::Unavailable&&out.Net.IsEmpty()&&overflow.Used>7&&overflow.Used<=401&&overflow.Remaining>=17))return false;
 FDistanceBudget usedOverflow{1000,MAX_uint64-393,17},badFloor{1000,3,-1};
 if(!TestTrue(TEXT("invalid cumulative account and floor stay terminal without refund"),
  D::TryBuild(source,true,usedOverflow,out)==D::EResult::Work&&usedOverflow.Used==MAX_uint64-393&&
  D::TryBuild(source,true,badFloor,out)==D::EResult::Work&&badFloor.Used==3))return false;
 std::fesetround(FE_UPWARD);FDistanceBudget rounding{1000,3};
 const bool refused=D::TryBuild(source,true,rounding,out)==D::EResult::Invalid&&out.Net.IsEmpty()&&rounding.Used==3;
 std::fesetround(FE_TONEAREST);if(!TestTrue(TEXT("wrong rounding cannot create a retained proof root"),refused))return false;
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStaticBodyDegreeReductionCaptured726Test,
 "IAmSpeed.AnalyticWorld.GroupedBox.StaticDegreeReductionCaptured726",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStaticBodyDegreeReductionCaptured726Test::RunTest(const FString&)
{
 using namespace Speed;using namespace Analytic;using namespace Certified;namespace D=StaticBodyHierarchy::DegreeReduction;namespace T=StaticBodyDegreeReductionTests;namespace A=::AnalyticCertified;
 const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("RNE captured exact source coefficient family"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 // Packet090d4115... provider726, C2_OUTER_HIGH_BODY_V1/XPYP/side/LowerArc1.
 // These are all36 original lower/upper endpoint bits, not fitted samples.
 static const uint64 cp[36][6]={
  {0x0000000000000000ull,0x0000000000000000ull,0x40aec3ef1535754bull,0x40aec3ef1535754cull,0x40337ca1866b95ceull,0x40337ca1866b95cfull},
  {0x0000000000000000ull,0x0000000000000000ull,0x40aee915d761c9e9ull,0x40aee915d761c9eaull,0x403b2e5f9ff717bfull,0x403b2e5f9ff717c0ull},
  {0x0000000000000000ull,0x0000000000000000ull,0x40af0cb9d77e832full,0x40af0cb9d77e8330ull,0x4042597cb8670af3ull,0x4042597cb8670af4ull},
  {0x0000000000000000ull,0x0000000000000000ull,0x40af2e60d35523d4ull,0x40af2e60d35523d5ull,0x4047f892ffbb3959ull,0x4047f892ffbb395aull},
  {0x0000000000000000ull,0x0000000000000000ull,0x40af4d9aae3cf7ebull,0x40af4d9aae3cf7ecull,0x404e61b85b441c72ull,0x404e61b85b441c73ull},
  {0x0000000000000000ull,0x0000000000000000ull,0x40af6a09e667f3bcull,0x40af6a09e667f3bdull,0x4052bec333018866ull,0x4052bec333018867ull},
  {0x40867fdd7d3ed05eull,0x40867fdd7d3ed05full,0x40aec3ef1535754bull,0x40aec3ef1535754cull,0x40337ca1866b95ceull,0x40337ca1866b95cfull},
  {0x40867fdd7d3ed05eull,0x40867fdd7d3ed05full,0x40aee915d761c9e9ull,0x40aee915d761c9eaull,0x403b2e5f9ff717bfull,0x403b2e5f9ff717c0ull},
  {0x40867fdd7d3ed05eull,0x40867fdd7d3ed05full,0x40af0cb9d77e832full,0x40af0cb9d77e8330ull,0x4042597cb8670af3ull,0x4042597cb8670af4ull},
  {0x40867fdd7d3ed05eull,0x40867fdd7d3ed05full,0x40af2e60d35523d4ull,0x40af2e60d35523d5ull,0x4047f892ffbb3959ull,0x4047f892ffbb395aull},
  {0x40867fdd7d3ed05eull,0x40867fdd7d3ed05full,0x40af4d9aae3cf7ebull,0x40af4d9aae3cf7ecull,0x404e61b85b441c72ull,0x404e61b85b441c73ull},
  {0x40867fdd7d3ed05eull,0x40867fdd7d3ed05full,0x40af6a09e667f3bcull,0x40af6a09e667f3bdull,0x4052bec333018866ull,0x4052bec333018867ull},
  {0x40967fdd7d3ed05eull,0x40967fdd7d3ed05full,0x40aec3ef1535754bull,0x40aec3ef1535754cull,0x40337ca1866b95ceull,0x40337ca1866b95cfull},
  {0x40967fdd7d3ed05eull,0x40967fdd7d3ed05full,0x40aee915d761c9e9ull,0x40aee915d761c9eaull,0x403b2e5f9ff717bfull,0x403b2e5f9ff717c0ull},
  {0x40967fdd7d3ed05eull,0x40967fdd7d3ed05full,0x40af0cb9d77e832full,0x40af0cb9d77e8330ull,0x4042597cb8670af3ull,0x4042597cb8670af4ull},
  {0x40967fdd7d3ed05eull,0x40967fdd7d3ed05full,0x40af2e60d35523d4ull,0x40af2e60d35523d5ull,0x4047f892ffbb3959ull,0x4047f892ffbb395aull},
  {0x40967fdd7d3ed05eull,0x40967fdd7d3ed05full,0x40af4d9aae3cf7ebull,0x40af4d9aae3cf7ecull,0x404e61b85b441c72ull,0x404e61b85b441c73ull},
  {0x40967fdd7d3ed05eull,0x40967fdd7d3ed05full,0x40af6a09e667f3bcull,0x40af6a09e667f3bdull,0x4052bec333018866ull,0x4052bec333018867ull},
  {0x40a0dfe61def1c46ull,0x40a0dfe61def1c47ull,0x40aec3ef1535754bull,0x40aec3ef1535754cull,0x40337ca1866b95ceull,0x40337ca1866b95cfull},
  {0x40a0dfe61def1c46ull,0x40a0dfe61def1c47ull,0x40aee915d761c9e9ull,0x40aee915d761c9eaull,0x403b2e5f9ff717bfull,0x403b2e5f9ff717c0ull},
  {0x40a0dfe61def1c46ull,0x40a0dfe61def1c47ull,0x40af0cb9d77e832full,0x40af0cb9d77e8330ull,0x4042597cb8670af3ull,0x4042597cb8670af4ull},
  {0x40a0dfe61def1c46ull,0x40a0dfe61def1c47ull,0x40af2e60d35523d4ull,0x40af2e60d35523d5ull,0x4047f892ffbb3959ull,0x4047f892ffbb395aull},
  {0x40a0dfe61def1c46ull,0x40a0dfe61def1c47ull,0x40af4d9aae3cf7ebull,0x40af4d9aae3cf7ecull,0x404e61b85b441c72ull,0x404e61b85b441c73ull},
  {0x40a0dfe61def1c46ull,0x40a0dfe61def1c47ull,0x40af6a09e667f3bcull,0x40af6a09e667f3bdull,0x4052bec333018866ull,0x4052bec333018867ull},
  {0x40a67fdd7d3ed05eull,0x40a67fdd7d3ed05full,0x40aec3ef1535754bull,0x40aec3ef1535754cull,0x40337ca1866b95ceull,0x40337ca1866b95cfull},
  {0x40a67fdd7d3ed05eull,0x40a67fdd7d3ed05full,0x40aee915d761c9e9ull,0x40aee915d761c9eaull,0x403b2e5f9ff717bfull,0x403b2e5f9ff717c0ull},
  {0x40a67fdd7d3ed05eull,0x40a67fdd7d3ed05full,0x40af0cb9d77e832full,0x40af0cb9d77e8330ull,0x4042597cb8670af3ull,0x4042597cb8670af4ull},
  {0x40a67fdd7d3ed05eull,0x40a67fdd7d3ed05full,0x40af2e60d35523d4ull,0x40af2e60d35523d5ull,0x4047f892ffbb3959ull,0x4047f892ffbb395aull},
  {0x40a67fdd7d3ed05eull,0x40a67fdd7d3ed05full,0x40af4d9aae3cf7ebull,0x40af4d9aae3cf7ecull,0x404e61b85b441c72ull,0x404e61b85b441c73ull},
  {0x40a67fdd7d3ed05eull,0x40a67fdd7d3ed05full,0x40af6a09e667f3bcull,0x40af6a09e667f3bdull,0x4052bec333018866ull,0x4052bec333018867ull},
  {0x40ac1fd4dc8e8475ull,0x40ac1fd4dc8e8476ull,0x40aec3ef1535754bull,0x40aec3ef1535754cull,0x40337ca1866b95ceull,0x40337ca1866b95cfull},
  {0x40ac1fd4dc8e8475ull,0x40ac1fd4dc8e8476ull,0x40aee915d761c9e9ull,0x40aee915d761c9eaull,0x403b2e5f9ff717bfull,0x403b2e5f9ff717c0ull},
  {0x40ac1fd4dc8e8475ull,0x40ac1fd4dc8e8476ull,0x40af0cb9d77e832full,0x40af0cb9d77e8330ull,0x4042597cb8670af3ull,0x4042597cb8670af4ull},
  {0x40ac1fd4dc8e8475ull,0x40ac1fd4dc8e8476ull,0x40af2e60d35523d4ull,0x40af2e60d35523d5ull,0x4047f892ffbb3959ull,0x4047f892ffbb395aull},
  {0x40ac1fd4dc8e8475ull,0x40ac1fd4dc8e8476ull,0x40af4d9aae3cf7ebull,0x40af4d9aae3cf7ecull,0x404e61b85b441c72ull,0x404e61b85b441c73ull},
  {0x40ac1fd4dc8e8475ull,0x40ac1fd4dc8e8476ull,0x40af6a09e667f3bcull,0x40af6a09e667f3bdull,0x4052bec333018866ull,0x4052bec333018867ull}
 };
 // Independently generated exact rational sum of the36 ORIGINAL Bernstein
 // interval coefficients, outward-rounded once at nine fixed dyadic UVs.
 struct FOracle {double U,V;uint64 Axis[6];};
 static const FOracle oracle[]={
  {0.125,0.125, {0x407c1fd4dc8e8475ull,0x407c1fd4dc8e8477ull,0x40aedae88030505eull,0x40aedae880305060ull,0x40389428762d4436ull,0x40389428762d4438ull}},
  {0.125,0.5, {0x407c1fd4dc8e8475ull,0x407c1fd4dc8e8477ull,0x40af1c73b21fddc2ull,0x40af1c73b21fddc4ull,0x40459267e9966e82ull,0x40459267e9966e84ull}},
  {0.125,0.875, {0x407c1fd4dc8e8475ull,0x407c1fd4dc8e8477ull,0x40af57d693281483ull,0x40af57d693281485ull,0x40509441c104c2a4ull,0x40509441c104c2a6ull}},
  {0.5,0.125, {0x409c1fd4dc8e8475ull,0x409c1fd4dc8e8477ull,0x40aedae88030505eull,0x40aedae880305060ull,0x40389428762d4436ull,0x40389428762d4438ull}},
  {0.5,0.5, {0x409c1fd4dc8e8475ull,0x409c1fd4dc8e8477ull,0x40af1c73b21fddc2ull,0x40af1c73b21fddc4ull,0x40459267e9966e82ull,0x40459267e9966e84ull}},
  {0.5,0.875, {0x409c1fd4dc8e8475ull,0x409c1fd4dc8e8477ull,0x40af57d693281483ull,0x40af57d693281485ull,0x40509441c104c2a4ull,0x40509441c104c2a6ull}},
  {0.875,0.125, {0x40a89bda40fcb3e6ull,0x40a89bda40fcb3e8ull,0x40aedae88030505eull,0x40aedae880305060ull,0x40389428762d4436ull,0x40389428762d4438ull}},
  {0.875,0.5, {0x40a89bda40fcb3e6ull,0x40a89bda40fcb3e8ull,0x40af1c73b21fddc2ull,0x40af1c73b21fddc4ull,0x40459267e9966e82ull,0x40459267e9966e84ull}},
  {0.875,0.875, {0x40a89bda40fcb3e6ull,0x40a89bda40fcb3e8ull,0x40af57d693281483ull,0x40af57d693281485ull,0x40509441c104c2a4ull,0x40509441c104c2a6ull}}
 };
 const uint32 saved=VectorGetControlRegister();
 for(uint32 flush:{0u,64u,32768u,32832u})for(bool transpose:{false,true})
 {
  VectorSetControlRegister((saved&~uint32(0x8040))|flush);
  FIntervalTensorSurface surface;surface.DegreeU=surface.DegreeV=5;surface.ControlPoints.SetNum(36);
  for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)for(int32 d=0;d<3;++d)
   surface.ControlPoints[transpose?v*6+u:u*6+v].Axis[d]={A::FromBits(cp[u*6+v][d*2]),A::FromBits(cp[u*6+v][d*2+1]),true};
  D::FPointwiseRoot reduced;FDistanceBudget budget{394};
  if(!TestTrue(TEXT("captured216 endpoint bits produce12 owned proof-only CP within exact debit"),
   D::TryBuild(surface,!transpose,budget,reduced)==D::EResult::Ready&&reduced.Net.Num()==12&&budget.Used==394))return false;
  const auto root=T::Node(reduced);
  for(const auto& expected:oracle)
  {
   FIntervalPoint point;if(!T::Evaluate(root,transpose?FVector2d(expected.V,expected.U):FVector2d(expected.U,expected.V),point))return false;
   for(int32 d=0;d<3;++d)if(!TestTrue(TEXT("reduced evaluation encloses the independently computed original full coefficient-family interval"),
    point.Axis[d].Lower<=A::FromBits(expected.Axis[d*2])&&point.Axis[d].Upper>=A::FromBits(expected.Axis[d*2+1])))return false;
  }
  for(const auto& residual:reduced.Residual)for(const auto& axis:residual.Axis)
   if(!TestTrue(TEXT("captured reduction residual remains finite and microscopic; diagnostic only, not an admission tolerance"),
    A::Valid(axis)&&FMath::Abs(axis.Lower)<1.e-9&&FMath::Abs(axis.Upper)<1.e-9))return false;
 }
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStaticHierarchyPublicationStickyIndependenceTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.StaticHierarchyStickyIndependent",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStaticHierarchyPublicationStickyIndependenceTest::RunTest(const FString&)
{
 FScopedCVar<int32> directionalOracleOff(TEXT("p.IAmSpeed.AnalyticWorld.NativeDirectionalMotionClearance"),0,FName(TEXT("FStaticHierarchyPublicationStickyIndependenceTestDirectionalOracle")));
 FScopedCVar<int32> sharedOff(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.SharedSupportSAT"),0,FName(TEXT("SharedSupportHistoricalOracle")));
 using namespace Speed;using namespace Analytic;using namespace Certified;
 namespace H=StaticBodyHierarchy;namespace T=StaticBodyHierarchyTests;namespace D=H::DegreeReduction;
 const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("RNE prepared geometry transfer"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 FScopedCVar<int32> mode(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy"),1,FName(TEXT("StickyTransferMode"))),
  depth(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.Depth"),3,FName(TEXT("StickyTransferDepth")));
 const uint32 saved=VectorGetControlRegister();int32 publications=0,queries=0;
 for(uint32 flush:{0u,64u,32768u,32832u})for(int32 reduced:{0,1})for(uint32 initHistory:{0u,63u})
 {
  FScopedCVar<int32> reduction(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.DegreeReduction"),reduced,FName(TEXT("StickyTransferReduction")));
  const uint32 control=(saved&~uint32(0x807f))|flush;
  VectorSetControlRegister(control|initHistory);
  const auto source=T::Surface();FIndexedIntervalUnionPtr scene;FDistanceBudget setup{100000};
  if(!T::Build(MakeArrayView(&source,1),setup,scene,true)||!scene->GetStaticBodyHierarchy())return false;
  ++publications;
  // The geometry's own initialization may legitimately add status flags to
  // initHistory0; no flag from either initialization history is a worker precondition.
  Detail::FNode root;root.DegreeU=root.DegreeV=5;root.Net=source.ControlPoints;
  if(reduced)
  {
   D::FPointwiseRoot value;FDistanceBudget reduceWork{1000};
   if(D::TryBuild(source,true,reduceWork,value)!=D::EResult::Ready)return false;
   root.DegreeU=value.DegreeU;root.DegreeV=value.DegreeV;root.Net=value.Net;
  }
  TArray<Detail::FNode> expected{root};
  for(int32 level=0;level<3;++level)
  {
   TArray<Detail::FNode> next;
   for(const auto& cell:expected){Detail::FNode a,b;if(!Detail::Split(cell,false,a,b))return false;next.Add(MoveTemp(a));next.Add(MoveTemp(b));}
   expected=MoveTemp(next);
  }
  FGroupedBoxVolumeBounds volume;if(!T::Volume(2.25,volume,true))return false;
  for(uint32 queryHistory:{0u,1u,2u,4u,8u,16u,32u,63u})
  {
   VectorSetControlRegister(control|queryHistory);
   const uint32 before=VectorGetControlRegister();TArray<H::FLeaf,TInlineAllocator<16>> leaves;FDistanceBudget query{100000,9,17};
   const auto status=H::BuildFrontier(*scene,0,volume,16383,query,leaves);
   const uint32 after=VectorGetControlRegister();
   if(!TestTrue(TEXT("publication and worker sticky histories are independent without any flag mutation"),
    status==H::EFrontier::Ready&&!leaves.IsEmpty()&&before==after&&query.ConsumptionFloor==17))return false;
   int32 prior=-1;
   for(const auto& leaf:leaves)
   {
    int32 match=INDEX_NONE;for(int32 i=0;i<expected.Num();++i)if(T::SameNode(leaf.Cell,expected[i])){match=i;break;}
    if(!TestTrue(TEXT("fresh frontier still equals the literal closed restriction oracle and correct source representation"),
     match!=INDEX_NONE&&match>prior&&leaf.ActiveGroups==(1u<<13)&&leaf.Cell.Net.Num()==(reduced?12:36)))return false;prior=match;
   }
   ++queries;
  }
  // Real controls still select arithmetic. A mismatched FTZ/DAZ fingerprint
  // stays unavailable, and non-RNE remains Invalid before any traversal.
  TArray<H::FLeaf,TInlineAllocator<16>> refused;refused.Add({});FDistanceBudget denied{100000,13,17};
  VectorSetControlRegister((control^uint32(0x8000))|63u);
  const auto mismatch=H::BuildFrontier(*scene,0,volume,16383,denied,refused);
  VectorSetControlRegister(control);
  if(!TestTrue(TEXT("real mode mismatch still clears output and refuses without work"),mismatch==H::EFrontier::Unavailable&&refused.IsEmpty()&&denied.Used==13))return false;
  refused.Add({});VectorSetControlRegister(control|uint32(0x2000));
  const auto invalid=H::BuildFrontier(*scene,0,volume,16383,denied,refused);
  VectorSetControlRegister(control);
  if(!TestTrue(TEXT("non-RNE controls still fail before geometry"),invalid==H::EFrontier::Invalid&&refused.IsEmpty()&&denied.Used==13))return false;
  FGroupedBoxVolumeBounds clearVolume;if(!T::Volume(1.75,clearVolume))return false;FReferenceBoundsCandidateSetPtr partition;FDistanceBudget selection{10000};
  if(!FReferenceBoundsCandidateSet::Select(scene,clearVolume.BroadHull,selection,partition))return false;
  VectorSetControlRegister(control);FReferenceBoxVolumeStrictSeparationPtr proof;FDistanceBudget work{20000};FGroupedBoxSeparationObservation observed;
  if(!TestTrue(TEXT("cleared worker history reaches the actual atlas and original-scene consumer"),
   FReferenceBoxVolumeStrictSeparation::TryProve(partition,clearVolume,work,proof,&observed,false,false,false,2,false)&&
   observed.VisitedCells==1&&proof&&proof->Matches(scene,clearVolume)&&proof->ContainsCompleteEndpointGroup(clearVolume.Groups[0].Corners)))return false;
 }
 if(!TestTrue(TEXT("16 actual publications and128 cross-history frontier queries"),publications==16&&queries==128))return false;
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
#endif


namespace Speed::Analytic::Certified::AffineReferenceDetail
{
namespace FixedProductWitnessControl
{
static TAutoConsoleVariable<int32> CVarEnabled(TEXT("p.IAmSpeed.AnalyticWorld.FixedProductWitness"),0,
    TEXT("Default OFF. Evaluate an authenticated fixed-product master witness through its original curve parameter and affine free factor; no TOI/UV/normal change."),ECVF_Default);
struct FCounts {std::atomic<uint64> Calls{0},Complete{0},Ticks{0};};
static FCounts Counts;
bool RuntimeEnabled(){return CVarEnabled.GetValueOnAnyThread()==1;}
void PublishObservation()
{
    UE_LOG(LogTemp,Display,TEXT("[FixedProductWitness] DefaultOFF=1 FixedSourceOnly=1 Calls=%llu Complete=%llu Seconds=%.9f"),
        Counts.Calls.load(),Counts.Complete.load(),FPlatformTime::ToSeconds64(Counts.Ticks.load()));
}
}
bool EvaluateFixedProductUV(const FProduct& product,FVector2d originalUV,FIntervalPoint& out)
{
    out={};const auto before=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    const bool observe=StrongConvexCurveMinimum::Observation::Enabled();
    const uint64 start=observe?FPlatformTime::Cycles64():0;
    if(observe)++FixedProductWitnessControl::Counts.Calls;
    struct FTime {bool Active;uint64 Start;~FTime(){if(Active)FixedProductWitnessControl::Counts.Ticks.fetch_add(FPlatformTime::Cycles64()-Start);}} timing{observe,start};
    const int32 axis=product.FreeWorldAxis;
    const auto freeParameter=Interval::Exact(product.FreeAlongU?originalUV.X:originalUV.Y);
    const double curveParameter=product.FreeAlongU?originalUV.Y:originalUV.X;
    if(axis<0||axis>=3||!A::Valid(product.Start)||!A::Valid(product.End)||
        !A::Valid(freeParameter)||freeParameter.Lower<0||freeParameter.Upper>1||
        product.Curve.DegreeU!=0||product.Curve.DegreeV<0||product.Curve.DegreeV>16||
        product.Curve.ControlPoints.Num()!=product.Curve.DegreeV+1)return false;
    // These checks validate the conditional product representation. The
    // enclosing source transport, not equal coefficient boxes, authenticates
    // that this product is the SAME fixed original master.
    for(const auto& cp:product.Curve.ControlPoints)
        if(!Detail::ValidPoint(cp)||A::Bits(cp.Axis[axis].Lower)!=0||A::Bits(cp.Axis[axis].Upper)!=0)return false;
    FIntervalPoint value;
    if(!ProjectedDistanceDetail::EvaluateFixedUV(product.Curve,FVector2d(.5,curveParameter),value))return false;
    const auto complement=A::Subtract(Interval::Exact(1),freeParameter);
    value.Axis[axis]=A::Add(A::Multiply(complement,product.Start),A::Multiply(freeParameter,product.End));
    if(!Detail::ValidPoint(value))return false;
    const auto after=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(after.ControlFingerprint()!=before.ControlFingerprint())return false;
    out=MoveTemp(value);if(observe)++FixedProductWitnessControl::Counts.Complete;return true;
}
}

#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Analytic::Certified::FixedProductWitnessTests
{
namespace R=AffineReferenceDetail;namespace A=::AnalyticCertified;
static R::FProduct Product(int32 axis,bool alongU,bool reverse,bool curved,bool uncertain=false)
{
    R::FProduct p;p.FreeWorldAxis=axis;p.FreeAlongU=alongU;
    p.Start=Interval::Exact(reverse?10.:-10.);p.End=Interval::Exact(reverse?-10.:10.);
    p.Curve.DegreeU=0;p.Curve.DegreeV=5;
    const double quadratic[6]={0,0,2,6,12,20};
    for(int32 j=0;j<6;++j)
    {
        FVector3d point=FVector3d::ZeroVector;point[(axis+1)%3]=2.*j;point[(axis+2)%3]=curved?quadratic[j]:0.;
        auto cp=Detail::Point(point);
        if(uncertain)for(int32 d=0;d<3;++d)if(d!=axis)
        {double lo=0,hi=0;A::Outward(point[d],false,lo);A::Outward(point[d],true,hi);cp.Axis[d]={lo,hi,true};}
        p.Curve.ControlPoints.Add(cp);
    }
    p.ExactCurveDerivativeHull=Detail::Point(FVector3d::ZeroVector);
    p.ExactCurveDerivativeHull.Axis[(axis+1)%3]=Interval::Exact(10.);
    p.ExactCurveDerivativeHull.Axis[(axis+2)%3]=curved?Interval{0,40,true}:Interval::Exact(0.);
    return p;
}
static FIntervalTensorSurface Original(const R::FProduct& p)
{
    FIntervalTensorSurface out;out.DegreeU=out.DegreeV=5;
    for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)
    {
        const int32 free=p.FreeAlongU?u:v,curve=p.FreeAlongU?v:u;
        auto cp=p.Curve.ControlPoints[curve];
        cp.Axis[p.FreeWorldAxis]=Interval::Exact(p.Start.Lower+(p.End.Lower-p.Start.Lower)*double(free)/5.);
        out.ControlPoints.Add(cp);
    }
    return out;
}
static bool Contains(const FIntervalPoint& a,FVector3d p)
{if(!Detail::ValidPoint(a))return false;for(int32 d=0;d<3;++d)if(a.Axis[d].Lower>p[d]||a.Axis[d].Upper<p[d])return false;return true;}
static bool SamePoint(const FIntervalPoint& a,const FIntervalPoint& b)
{for(int32 d=0;d<3;++d)if(a.Axis[d].Known!=b.Axis[d].Known||A::Bits(a.Axis[d].Lower)!=A::Bits(b.Axis[d].Lower)||A::Bits(a.Axis[d].Upper)!=A::Bits(b.Axis[d].Upper))return false;return true;}
static FTensorSphereContactEnclosure Reduced(const R::FProduct& p)
{
    FTensorSphereContactEnclosure r;r.Status=ETensorSphereContactStatus::FirstContactEnclosed;
    r.LowerTime=r.UpperTime=.5;r.CollisionRadiusBits=A::Bits(1.);r.SurfacePoint=Detail::Point(FVector3d::ZeroVector);
    r.SurfacePoint.Axis[(p.FreeWorldAxis+1)%3]=Interval::Exact(5.);
    r.RadialUnitNormal=Detail::Point(FVector3d::ZeroVector);r.RadialUnitNormal.Axis[(p.FreeWorldAxis+2)%3]=Interval::Exact(1.);
    FTensorSphereContactLeaf leaf;leaf.SurfaceIndex=0;leaf.MinU=0;leaf.MaxU=1;leaf.MinV=.5;leaf.MaxV=.5+1./1048576.;
    leaf.FixedParameter=FVector2d(.5,(leaf.MinV+leaf.MaxV)*.5);leaf.PointHull=r.SurfacePoint;leaf.RadialNormalHull=r.RadialUnitNormal;
    ProjectedDistanceDetail::EvaluateFixedUV(p.Curve,leaf.FixedParameter,leaf.FixedParameterPoint);r.Leaves.Add(leaf);return r;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFixedProductWitnessExactParameterTest,
    "IAmSpeed.AnalyticWorld.FixedProductWitness.ExactMasterParameter",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FFixedProductWitnessExactParameterTest::RunTest(const FString&)
{
    using namespace Speed::Analytic::Certified;namespace T=FixedProductWitnessTests;namespace R=AffineReferenceDetail;namespace A=::AnalyticCertified;
    const auto saved=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(saved);std::fenv_t held;
    if(!saved.bValid||!restore.IsApplied()||std::feholdexcept(&held)!=0||std::fesetround(FE_TONEAREST)!=0)return false;
    const uint32 controls=VectorGetControlRegister();int32 checks=0;
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(int32 axis=0;axis<3;++axis)for(bool alongU:{false,true})for(bool reverse:{false,true})for(bool uncertain:{false,true})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);const auto p=T::Product(axis,alongU,reverse,true,uncertain);const auto original=T::Original(p);
        for(double free:{0.,.125,.5,.875,1.})for(double curve:{0.,.0625,.5,.9375,1.})
        {
            const FVector2d uv=alongU?FVector2d(free,curve):FVector2d(curve,free);FIntervalPoint reduced,full;
            FVector3d exact=FVector3d::ZeroVector;exact[axis]=reverse?10.-20.*free:-10.+20.*free;
            exact[(axis+1)%3]=10.*curve;exact[(axis+2)%3]=20.*curve*curve;
            if(!TestTrue(TEXT("same exact master point in product and independent original coefficient enclosure"),
                R::EvaluateFixedProductUV(p,uv,reduced)&&ProjectedDistanceDetail::EvaluateFixedUV(original,uv,full)&&T::Contains(reduced,exact)&&T::Contains(full,exact)))return false;
            ++checks;
        }
    }
    TestEqual(TEXT("2400 axes/transposition/slope/family/FTZ/UV cases"),checks,2400);
    auto p=T::Product(0,true,false,true);FIntervalPoint output=Detail::Point(FVector3d(1,2,3));
    p.Curve.ControlPoints[0].Axis[0]=Interval::Exact(1.);
    TestTrue(TEXT("invalid product free coordinate refuses and clears stale output"),!R::EvaluateFixedProductUV(p,FVector2d(.5,.5),output)&&!Detail::ValidPoint(output));
    p=T::Product(0,true,false,true);
    TestFalse(TEXT("out-of-domain original parameter refused"),R::EvaluateFixedProductUV(p,FVector2d(.5,1.01),output));
    TestTrue(TEXT("non-RNE installed"),std::fesetround(FE_UPWARD)==0);bool caught=false;
    try{R::EvaluateFixedProductUV(p,FVector2d(.5,.5),output);}catch(const Speed::Analytic::FAnalyticNumericUnknown&){caught=true;}
    TestTrue(TEXT("current numeric boundary still enforced"),caught);TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFixedProductWitnessLiftAndConsumerTest,
    "IAmSpeed.AnalyticWorld.FixedProductWitness.LiftConsumerAndBudget",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FFixedProductWitnessLiftAndConsumerTest::RunTest(const FString&)
{
    using namespace Speed::Analytic::Certified;namespace T=FixedProductWitnessTests;namespace R=AffineReferenceDetail;namespace M=FixedWheelProbeMath;namespace A=::AnalyticCertified;
    const auto saved=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(saved);std::fenv_t held;
    if(!saved.bValid||!restore.IsApplied()||std::feholdexcept(&held)!=0||std::fesetround(FE_TONEAREST)!=0)return false;
    const uint32 controls=VectorGetControlRegister();int32 checks=0;
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(int32 axis=0;axis<3;++axis)for(bool alongU:{false,true})for(bool reverse:{false,true})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);const auto product=T::Product(axis,alongU,reverse,false);const auto original=T::Original(product);const auto reduced=T::Reduced(product);
        FVector3d start=FVector3d::ZeroVector,end;start[axis]=.25;start[(axis+1)%3]=5.;start[(axis+2)%3]=3.;end=start;end[(axis+2)%3]=-1.;
        FIntervalBezierCenterPath path;if(!CenterDetail::Linear(start,end,path))return false;
        for(int32 cap:{0,1,10,20,30,50,1000})
        {
            FTensorSphereContactEnclosure before,after;FDistanceBudget a{cap+7,13,7},b{cap+7,13,7};R::EOutcome sa,sb;
            {FScopedCVar<int32> flag(TEXT("p.IAmSpeed.AnalyticWorld.FixedProductWitness"),0,FName(TEXT("ProductWitnessOracle")));sa=R::LiftFixedMasterContact(MakeArrayView(&product,1),MakeArrayView(&original,1),path,reduced,.00025,.001,a,before);}
            {FScopedCVar<int32> flag(TEXT("p.IAmSpeed.AnalyticWorld.FixedProductWitness"),1,FName(TEXT("ProductWitnessCandidate")));sb=R::LiftFixedMasterContact(MakeArrayView(&product,1),MakeArrayView(&original,1),path,reduced,.00025,.001,b,after);}
            if(!TestTrue(TEXT("same closed result kind and cumulative budget/floor without refund"),sa==sb&&a.Used==b.Used&&a.Remaining==b.Remaining&&a.ConsumptionFloor==b.ConsumptionFloor))return false;
            if(sb!=R::EOutcome::Complete){if(!TestTrue(TEXT("no partial leaf publication on exhaustion"),after.Leaves.IsEmpty()))return false;continue;}
            if(!TestTrue(TEXT("time, universal point, normal and leaf partition are unchanged"),A::Bits(before.LowerTime)==A::Bits(after.LowerTime)&&A::Bits(before.UpperTime)==A::Bits(after.UpperTime)&&T::SamePoint(before.SurfacePoint,after.SurfacePoint)&&T::SamePoint(before.RadialUnitNormal,after.RadialUnitNormal)&&before.Leaves.Num()==after.Leaves.Num()))return false;
            if(cap!=1000)continue; // Tight caps compare the lift only; consumers retain their own paid work.
            for(int32 i=0;i<after.Leaves.Num();++i)
            {
                const auto& x=before.Leaves[i];const auto& y=after.Leaves[i];
                if(!TestTrue(TEXT("original parameter bits, identity and domain preserved"),x.SurfaceIndex==y.SurfaceIndex&&A::Bits(x.FixedParameter.X)==A::Bits(y.FixedParameter.X)&&A::Bits(x.FixedParameter.Y)==A::Bits(y.FixedParameter.Y)&&x.MinU==y.MinU&&x.MaxU==y.MaxU&&x.MinV==y.MinV&&x.MaxV==y.MaxV))return false;
                const double f=alongU?y.FixedParameter.X:y.FixedParameter.Y,c=alongU?y.FixedParameter.Y:y.FixedParameter.X;
                FVector3d exact=FVector3d::ZeroVector;exact[axis]=reverse?10.-20.*f:-10.+20.*f;exact[(axis+1)%3]=10.*c;
                FReferenceDyadicCell u,v;FIntervalPoint normal;
                if(!TestTrue(TEXT("same original fixed master sample and regular chart consumed"),T::Contains(y.FixedParameterPoint,exact)&&M::DecodeLeafDyadic(y.MinU,y.MaxU,u)&&M::DecodeLeafDyadic(y.MinV,y.MaxV,v)&&R::ProveFixedProductLeafRank(product,u,v,b,normal)))return false;
            }
            M::FBoundedFixedProbeSample sample;FScalarSphereContact scalar;
            FVector3d point;for(int32 d=0;d<3;++d)point[d]=after.Leaves[0].FixedParameterPoint.Axis[d].Lower;
            const FVector3d center=start+.5*(end-start);
            if(!TestTrue(TEXT("actual probe sample and scalar contact conversion remain bounded"),M::TryBoundedFixedProbeSample(after,1./300.,sample)&&ConvertSphereContact(after,0,point,center,1.,.99,scalar)==EScalarSphereContactStatus::Bounded))return false;
            // Deliberately unusable OPTIONAL curve metadata. This is only a
            // fallback-control fixture, not a new product-factorization proof;
            // the old original-net evaluation remains the sample authority.
            auto unavailable=product;unavailable.Curve.ControlPoints[0].Axis[axis]=Interval::Exact(1.);
            FTensorSphereContactEnclosure fallback,denied;FDistanceBudget retry{1007,13,7};
            const int32 oldSpent=int32(a.Used-13);FDistanceBudget tight{oldSpent+7,13,7};
            R::EOutcome retried,blocked;
            {FScopedCVar<int32> flag(TEXT("p.IAmSpeed.AnalyticWorld.FixedProductWitness"),1,FName(TEXT("ProductWitnessFallback")));
             retried=R::LiftFixedMasterContact(MakeArrayView(&unavailable,1),MakeArrayView(&original,1),path,reduced,.00025,.001,retry,fallback);
             blocked=R::LiftFixedMasterContact(MakeArrayView(&unavailable,1),MakeArrayView(&original,1),path,reduced,.00025,.001,tight,denied);}
            if(!TestTrue(TEXT("optional failure is paid and literal original evaluation remains the fallback"),
                retried==R::EOutcome::Complete&&fallback.Leaves.Num()==before.Leaves.Num()&&retry.Used==a.Used+uint64(before.Leaves.Num())&&
                T::SamePoint(fallback.Leaves[0].FixedParameterPoint,before.Leaves[0].FixedParameterPoint)))return false;
            if(!TestTrue(TEXT("old exact cap cannot credit the failed optional attempt or publish partial leaves"),
                blocked==R::EOutcome::Unknown&&denied.Leaves.IsEmpty()&&tight.Used==uint64(oldSpent)+13&&tight.Remaining==7&&tight.ConsumptionFloor==7))return false;
            ++checks;
        }
    }
    TestTrue(TEXT("all48 source orientations reach real conversion"),checks>=48);TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
#endif


namespace Speed::Analytic::Certified::ClosedGroupedBoxControl
{
static TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.ClosedGroupedVolume"),0,
    TEXT("Default OFF. Owned immutable grouped-volume closure; fresh scene proof and original logical work remain mandatory."));
static std::atomic<uint64> Calls{0},Work{0},PointProjections{0};
bool RuntimeEnabled(){return Enabled.GetValueOnAnyThread()==1;}
void RecordSkippedClosure(uint64 work,uint64 points)
{
    static const IConsoleVariable* phase=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"));
    if(!phase||phase->GetInt()==0)return;
    ++Calls;Work.fetch_add(work);PointProjections.fetch_add(points);
}
void PublishObservation()
{
    UE_LOG(LogTemp,Display,TEXT("[ClosedGroupedVolume] DefaultOFF=1 OwnedClosure=1 Calls=%llu LogicalWorkPaid=%llu PointProjectionsSkipped=%llu"),
        Calls.load(),Work.load(),PointProjections.load());
}
}

#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Analytic::Certified::ClosedGroupedVolumeTests
{
static bool Inputs(int32 count,FIntervalPoint& broad,TArray<FBoxVolumeGroup,TInlineAllocator<16>>& groups,
    TArray<FVector3d,TInlineAllocator<8>>& axes)
{
    FWorldQuery query;query.Shape=EQueryShape::Box;query.Start=query.End=FVector3d::ZeroVector;
    query.HalfExtent=FVector3d(1.);query.Rotation=FQuat4d(FVector3d::UpVector,UE_DOUBLE_PI/4.);
    FBoxVolumeGroup group;if(!NativeGroupedBoxDetail::FixedQueryGroup(query,group))return false;
    for(const auto& p:group.Corners)broad=Detail::ValidPoint(broad)?TimeDetail::Hull(broad,p):p;
    for(int32 i=0;i<count;++i)groups.Add(group);
    for(const auto axis:{FVector3d::ForwardVector,FVector3d::RightVector,FVector3d::UpVector})axes.Add(axis);
    for(const auto axis:{FVector3d::ForwardVector,FVector3d::RightVector,FVector3d::UpVector})axes.Add(query.Rotation.RotateVector(axis));
    return true;
}
static bool Scene(bool crossing,FIndexedIntervalUnionPtr& scene)
{
    FIntervalTensorSurface surface;
    if(crossing)
    {surface.DegreeU=surface.DegreeV=1;for(double y:{-5.,5.})for(double z:{-5.,5.})surface.ControlPoints.Add(Detail::Point(FVector3d(.25,y,z)));}
    else surface.ControlPoints.Add(Detail::Point(FVector3d(1.3,1.3,0.)));
    FDistanceBudget budget{8192};return FIndexedIntervalUnion::Build(MakeArrayView(&surface,1),budget,scene);
}
static bool Prove(FReferenceBoundsCandidateSetPtr partition,const FGroupedBoxVolumeBounds& volume,
    const FClosedGroupedBoxVolumePtr& closed,FDistanceBudget& budget,FReferenceBoxVolumeStrictSeparationPtr& proof)
{
    return closed?FReferenceBoxVolumeStrictSeparation::TryProveClosed(partition,closed,budget,proof,nullptr,false,false,false,0,false):
        FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,budget,proof,nullptr,false,false,false,0,false);
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FClosedGroupedVolumeOwnershipTest,
    "IAmSpeed.AnalyticWorld.ClosedGroupedVolume.OwnershipAndProof",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FClosedGroupedVolumeOwnershipTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace T=ClosedGroupedVolumeTests;
    const auto saved=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(saved);std::fenv_t held;
    if(!saved.bValid||!restore.IsApplied()||std::feholdexcept(&held)!=0||std::fesetround(FE_TONEAREST)!=0)return false;
    const uint32 control=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})for(int32 count:{1,8,14})
    {
        const uint32 mode=(control&~uint32(0x8040))|flush;VectorSetControlRegister(mode);
        FIntervalPoint broad;TArray<FBoxVolumeGroup,TInlineAllocator<16>> groups;TArray<FVector3d,TInlineAllocator<8>> axes;
        if(!T::Inputs(count,broad,groups,axes))return false;
        FDistanceBudget a{8192,13,7},b=a;FGroupedBoxVolumeBounds volume;FClosedGroupedBoxVolumePtr closed;
        if(!TestTrue(TEXT("factory executes same complete closure once and owns exact bits"),
            CloseGroupedBoxVolumeBounds(broad,groups,axes,a,volume)&&FClosedGroupedBoxVolume::Build(broad,groups,axes,b,closed)&&
            closed&&GroupedBoxDetail::Same(volume,closed->GetVolumeForInspection())&&a.Used==b.Used&&a.Remaining==b.Remaining))return false;
        groups[0].Corners[0].Axis[0]=Interval::Exact(1000.);groups.Empty();axes[0]=FVector3d(1,2,3);axes.Empty();
        if(!TestTrue(TEXT("mutating then releasing every source container cannot mutate the owner"),GroupedBoxDetail::Same(volume,closed->GetVolumeForInspection())))return false;
        FIndexedIntervalUnionPtr scene;if(!T::Scene(false,scene))return false;FReferenceBoundsCandidateSetPtr partition;FDistanceBudget select{8192};
        if(!FReferenceBoundsCandidateSet::Select(scene,volume.BroadHull,select,partition))return false;
        FDistanceBudget p{8192,13,7},q=p;FReferenceBoxVolumeStrictSeparationPtr x,y;
        if(!TestTrue(TEXT("same real scene proof and same original debit"),T::Prove(partition,volume,{},p,x)&&T::Prove(partition,volume,closed,q,y)&&
            x&&y&&p.Used==q.Used&&p.Remaining==q.Remaining&&x->Matches(scene,volume)&&y->Matches(scene,volume)&&
            x->ContainsCompleteEndpointGroup(volume.Groups[0].Corners)&&y->ContainsCompleteEndpointGroup(volume.Groups[0].Corners)))return false;
        auto wrong=volume;wrong.Slabs[0].Upper+=.25;FDistanceBudget bad{8192};auto stale=x;
        if(!TestTrue(TEXT("mutable API still rejects changed slabs and clears output"),!T::Prove(partition,wrong,{},bad,stale)&&!stale&&
            !y->Matches(scene,wrong)&&GroupedBoxDetail::Same(volume,closed->GetVolumeForInspection())))return false;
        // Alias the owner's own views into a replacement factory call. Build
        // retains the prior owner before clearing output, so no dangling view.
        const auto& own=closed->GetVolumeForInspection();FDistanceBudget again{8192};
        if(!TestTrue(TEXT("factory preserves aliased owner inputs until a new complete publication"),FClosedGroupedBoxVolume::Build(own.BroadHull,own.Groups,own.Axes,again,closed)&&
            GroupedBoxDetail::Same(volume,closed->GetVolumeForInspection())))return false;
        VectorSetControlRegister(mode^uint32(0x40));FDistanceBudget changed{8192,13,7};stale=y;
        const bool mismatch=T::Prove(partition,volume,closed,changed,stale);VectorSetControlRegister(mode);
        if(!TestTrue(TEXT("different valid numeric controls cannot consume an old closed value"),!mismatch&&!stale&&changed.Used==13))return false;
        VectorSetControlRegister(mode&~uint32(63));FDistanceBudget cleared{8192};
        if(!TestTrue(TEXT("unrelated cleared exception history does not invalidate immutable geometry"),T::Prove(partition,volume,closed,cleared,stale)))return false;
        FIndexedIntervalUnionPtr cross;if(!T::Scene(true,cross))return false;FReferenceBoundsCandidateSetPtr cp;FDistanceBudget cs{8192};
        if(!FReferenceBoundsCandidateSet::Select(cross,volume.BroadHull,cs,cp))return false;
        FDistanceBudget ca{4096},cb=ca;FReferenceBoxVolumeStrictSeparationPtr pa,pb;
        if(!TestTrue(TEXT("crossing surface remains Unknown/refused with identical spent work"),!T::Prove(cp,volume,{},ca,pa)&&!T::Prove(cp,volume,closed,cb,pb)&&
            !pa&&!pb&&ca.Used==cb.Used&&ca.Remaining==cb.Remaining))return false;
    }
    TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FClosedGroupedVolumeBudgetTest,
    "IAmSpeed.AnalyticWorld.ClosedGroupedVolume.PartialBudgetAndMutation",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FClosedGroupedVolumeBudgetTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace T=ClosedGroupedVolumeTests;
    const auto saved=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(saved);std::fenv_t held;
    if(!saved.bValid||!restore.IsApplied()||std::feholdexcept(&held)!=0||std::fesetround(FE_TONEAREST)!=0)return false;
    const uint32 control=VectorGetControlRegister();int32 pairs=0;
    for(uint32 flush:{0u,64u,32768u,32832u})for(int32 count:{1,8,14})
    {
        VectorSetControlRegister((control&~uint32(0x8040))|flush);
        FIntervalPoint broad;TArray<FBoxVolumeGroup,TInlineAllocator<16>> groups;TArray<FVector3d,TInlineAllocator<8>> axes;
        if(!T::Inputs(count,broad,groups,axes))return false;
        FClosedGroupedBoxVolumePtr closed;FDistanceBudget build{8192};
        if(!FClosedGroupedBoxVolume::Build(broad,groups,axes,build,closed))return false;const auto& volume=closed->GetVolumeForInspection();
        FIndexedIntervalUnionPtr scene;if(!T::Scene(false,scene))return false;FReferenceBoundsCandidateSetPtr partition;FDistanceBudget select{8192};
        if(!FReferenceBoundsCandidateSet::Select(scene,volume.BroadHull,select,partition))return false;
        FDistanceBudget full{8192};FReferenceBoxVolumeStrictSeparationPtr fullProof;
        if(!T::Prove(partition,volume,{},full,fullProof))return false;
        for(int32 cap=0;cap<=int32(full.Used)+2;++cap)for(int32 floor:{0,7})
        {
            FDistanceBudget a{cap+floor,13,floor},b=a;FReferenceBoxVolumeStrictSeparationPtr x=fullProof,y=fullProof;
            const bool ka=T::Prove(partition,volume,{},a,x),kb=T::Prove(partition,volume,closed,b,y);
            if(!TestTrue(TEXT("every partial accounting prefix agrees; no atomic preflight or refund"),ka==kb&&a.Used==b.Used&&a.Remaining==b.Remaining&&
                bool(x)==bool(y)&&b.Remaining>=floor&&b.ConsumptionFloor==floor))return false;++pairs;
        }
        for(uint64 tail:{0ull,1ull,19ull,20ull,117ull,118ull,201ull,202ull})
        {
            FDistanceBudget a{8192,MAX_uint64-tail,0},b=a;FReferenceBoxVolumeStrictSeparationPtr x,y;
            const bool ka=T::Prove(partition,volume,{},a,x),kb=T::Prove(partition,volume,closed,b,y);
            if(!TestTrue(TEXT("Used overflow preserves exactly the consumed prefix"),ka==kb&&a.Used==b.Used&&a.Remaining==b.Remaining&&bool(x)==bool(y)))return false;
        }
        axes.Add(FVector3d(1,1,1));FDistanceBudget extra{8192};FClosedGroupedBoxVolumePtr augmented;
        if(!TestTrue(TEXT("new axes require a new complete owner"),FClosedGroupedBoxVolume::Build(broad,groups,axes,extra,augmented)&&
            augmented!=closed&&augmented->GetVolumeForInspection().Axes.Num()==7&&volume.Axes.Num()==6&&
            !fullProof->Matches(scene,augmented->GetVolumeForInspection())))return false;
        groups[0].Corners[0].Axis[0]={};FDistanceBudget invalid{8192};augmented=closed;
        if(!TestTrue(TEXT("failed factory cannot publish or retain a previous closure"),!FClosedGroupedBoxVolume::Build(broad,groups,axes,invalid,augmented)&&!augmented))return false;
    }
    TestTrue(TEXT("budget comparisons exercised"),pairs>1000);TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Analytic::Certified::SharedSurfaceObbTests
{
namespace A=::AnalyticCertified;
static FIntervalTensorSurface Surface(bool transpose=false,int32 extrusion=0,double width=0)
{
 FIntervalTensorSurface s;s.DegreeU=s.DegreeV=5;s.ControlPoints.SetNum(36);
 const int32 section=extrusion==0?1:0;
 for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)
 {
  FVector3d q=FVector3d::ZeroVector;q[extrusion]=-1800.+3600.*double(u)/5.;q[section]=-1.+2.*double(v)/5.;
  q[2]=1.2+1.2*double(v)/5.+.8*double(v*(v-1))/20.;auto p=Detail::Point(q);
  if(width>0)for(auto& a:p.Axis){a.Lower-=width;a.Upper+=width;}
  s.ControlPoints[transpose?v*6+u:u*6+v]=p;
 }
 return s;
}
static bool Volume(FGroupedBoxVolumeBounds& out,int32 extrusion=0,bool crossing=false,bool lastOnly=false)
{
 const int32 section=extrusion==0?1:0;TArray<FBoxVolumeGroup> groups;FIntervalPoint hull;
 for(int32 g=0;g<14;++g)
 {
  FBoxVolumeGroup group;
  for(int32 bits=0;bits<8;++bits)
  {
   FVector3d p=FVector3d::ZeroVector;p[extrusion]=(bits&1?.01:-.01);p[section]=.0625+(bits&2?.01:-.01);
   p[2]=2.+.0625+.2*.0625*.0625+(crossing&&(!lastOnly||g==13)?0.:-.05)+(bits&4?.01:-.01);
   auto q=Detail::Point(p);group.Corners.Add(q);hull=Detail::ValidPoint(hull)?TimeDetail::Hull(hull,q):q;
  }
  groups.Add(MoveTemp(group));
 }
 FVector3d n=FVector3d::ZeroVector;n[section]=-1.;n[2]=1.;
 TArray<FVector3d> axes{FVector3d(1,0,0),FVector3d(0,1,0),FVector3d(0,0,1),n,FVector3d(1,1,0),FVector3d(1,0,1)};
 FDistanceBudget b{10000};return CloseGroupedBoxVolumeBounds(hull,groups,axes,b,out);
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStaticSharedSurfaceObbProofTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.StaticSharedSurfaceObbProof",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStaticSharedSurfaceObbProofTest::RunTest(const FString&)
{
 FScopedCVar<int32> directionalOracleOff(TEXT("p.IAmSpeed.AnalyticWorld.NativeDirectionalMotionClearance"),0,FName(TEXT("FStaticSharedSurfaceObbProofTestDirectionalOracle")));
 using namespace Speed;using namespace Analytic;using namespace Certified;namespace H=StaticBodyHierarchy;namespace T=SharedSurfaceObbTests;
 const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("RNE shared immutable surface supports"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 FScopedCVar<int32> mode(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy"),3,FName(TEXT("SharedObbProofMode"))),
  depth(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.Depth"),3,FName(TEXT("SharedObbProofDepth"))),
  enabled(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.SharedSupportSAT"),1,FName(TEXT("SharedObbProofEnabled")));
 const uint32 saved=VectorGetControlRegister();int32 contexts=0;
 for(uint32 flush:{0u,64u,32768u,32832u})for(bool transpose:{false,true})for(int32 reduced:{0,1})for(int32 extrusion:{0,1})
 {
  VectorSetControlRegister((saved&~uint32(0x8040))|flush);
  FScopedCVar<int32> reduction(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.DegreeReduction"),reduced,FName(TEXT("SharedObbProofReduction")));
  const auto source=T::Surface(transpose,extrusion,1.e-9);FIndexedIntervalUnionPtr scene;FDistanceBudget setup{100000};
  if(!StaticBodyHierarchyTests::Build(MakeArrayView(&source,1),setup,scene,true)||!scene->GetStaticBodyHierarchy())return false;
  if(!TestTrue(TEXT("actual support construction is charged within finite forecast and retained cap"),setup.Used>2821&&setup.Used<=10000&&scene->GetStaticBodyHierarchy()->AllocatedBytes()<=H::Private::MaximumBytes))return false;
  for(bool crossing:{false,true})
  {
   FGroupedBoxVolumeBounds volume;if(!T::Volume(volume,extrusion,crossing,crossing))return false;
   TArray<H::FLeaf,TInlineAllocator<16>> plain,oriented;FDistanceBudget a{100000,9,17},b{100000,9,17};
   {FScopedCVar<int32> off(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.SharedSupportSAT"),0,FName(TEXT("SharedObbProofPlainOracle")));
    if(H::BuildFrontier(*scene,0,volume,16383,a,plain)!=H::EFrontier::Ready)return false;}
   if(!TestTrue(TEXT("OBB targets the real ambiguous AABB frontier, never clears a crossing final group"),
    H::BuildFrontier(*scene,0,volume,16383,b,oriented)==H::EFrontier::Ready&&!plain.IsEmpty()&&oriented.IsEmpty()==!crossing&&b.ConsumptionFloor==17&&b.Remaining>=17))return false;
   if(crossing)for(const auto& leaf:oriented)if(!TestTrue(TEXT("each surviving domain retains the independent last group"),leaf.ActiveGroups==(1u<<13)))return false;
   if(!crossing)
   {
    FReferenceBoundsCandidateSetPtr partition;FDistanceBudget select{10000};if(!FReferenceBoundsCandidateSet::Select(scene,volume.BroadHull,select,partition))return false;
    FReferenceBoxVolumeStrictSeparationPtr oracle,proof;FDistanceBudget oldWork{30000,9,17},newWork{30000,9,17};FGroupedBoxSeparationObservation oldObservation,newObservation;
    {FScopedCVar<int32> off(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy"),0,FName(TEXT("SharedObbProofExactOracle")));
     if(!FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,oldWork,oracle,&oldObservation,false,false,false,2,false))return false;}
    if(!TestTrue(TEXT("full original scene proof and unchanged consumer; real early route"),
     FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,newWork,proof,&newObservation,false,false,false,2,false)&&
     proof->Matches(scene,volume)&&proof->ContainsCompleteEndpointGroup(volume.Groups[0].Corners)&&newObservation.VisitedCells==1&&oldObservation.VisitedCells>1))return false;
   }
  }
  ++contexts;
 }
 TestEqual(TEXT("32 genuine publication configurations"),contexts,32);TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStaticSharedSurfaceObbRegionTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.StaticSharedSurfaceObbOriginalRegion",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStaticSharedSurfaceObbRegionTest::RunTest(const FString&)
{
 FScopedCVar<int32> directionalOracleOff(TEXT("p.IAmSpeed.AnalyticWorld.NativeDirectionalMotionClearance"),0,FName(TEXT("FStaticSharedSurfaceObbRegionTestDirectionalOracle")));
 using namespace Speed;using namespace Analytic;using namespace Certified;namespace H=StaticBodyHierarchy;namespace P=H::Private;namespace A=::AnalyticCertified;
 const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("RNE original slab region"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 FScopedCVar<int32> mode(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy"),3,FName(TEXT("SharedObbRegionMode"))),
  depth(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.Depth"),3,FName(TEXT("SharedObbRegionDepth"))),
  enabled(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.SharedSupportSAT"),1,FName(TEXT("SharedObbRegionEnabled"))),
  reduction(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.DegreeReduction"),1,FName(TEXT("SharedObbRegionReduction")));
 FBoxVolumeGroup diamond;FIntervalPoint broad;
 for(int32 x:{-1,1})for(FVector2d yz:{FVector2d(1,0),FVector2d(-1,0),FVector2d(0,1),FVector2d(0,-1)})
 {const auto p=Detail::Point(FVector3d(x,yz.X,yz.Y));diamond.Corners.Add(p);broad=Detail::ValidPoint(broad)?TimeDetail::Hull(broad,p):p;}
 TArray<FBoxVolumeGroup> groups;groups.Init(diamond,14);
 TArray<FVector3d> axes{FVector3d(1,0,0),FVector3d(0,1,0),FVector3d(0,0,1),FVector3d(1,0,0),FVector3d(0,1,0),FVector3d(0,0,1)};
 FGroupedBoxVolumeBounds volume;FDistanceBudget close{10000};if(!CloseGroupedBoxVolumeBounds(broad,groups,axes,close,volume))return false;
 auto source=SharedSurfaceObbTests::Surface();
 for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)
 {source.ControlPoints[u*6+v]=Detail::Point(FVector3d(-2.+4.*double(u)/5.,.6+.4*double(v)/5.,1.-.4*double(v)/5.));}
 const auto witness=Detail::Point(FVector3d(0,.8,.8));
 const auto unsafeCornerSupport=GroupedBoxDetail::ProjectHull(diamond.Corners,FVector3d(0,1,1));
 if(!TestTrue(TEXT("discriminant point is outside corner hull yet inside every certified old slab"),unsafeCornerSupport.Upper<1.6))return false;
 for(int32 k=0;k<6;++k){const auto projected=GroupedBoxDetail::Project(witness,axes[k]);if(!TestTrue(TEXT("old region includes witness"),volume.Slabs[k].Lower<projected.Lower&&projected.Upper<volume.Slabs[k].Upper))return false;}
 const uint32 saved=VectorGetControlRegister();
 for(uint32 flush:{0u,64u,32768u,32832u})
 {
  VectorSetControlRegister((saved&~uint32(0x8040))|flush);FDistanceBudget init{100000};FIndexedIntervalUnionPtr scene;
  if(!StaticBodyHierarchyTests::Build(MakeArrayView(&source,1),init,scene,true)||!scene->GetStaticBodyHierarchy())return false;
  FDistanceBudget query{100000};TArray<H::FLeaf,TInlineAllocator<16>> leaves;
  if(!TestTrue(TEXT("surface through old region is NEVER discharged by the smaller corner hull"),H::BuildFrontier(*scene,0,volume,16383,query,leaves)==H::EFrontier::Ready&&!leaves.IsEmpty()))return false;
  for(int32 extrusion:{0,1})
  {
   P::FSharedFrame frame;frame.Extrusion=extrusion;const int32 d=extrusion==0?1:0;
   frame.Normal=frame.Tangent=FVector3d::ZeroVector;frame.Normal[d]=3;frame.Normal.Z=4;frame.Tangent[d]=-4;frame.Tangent.Z=3;
   if(!TestTrue(TEXT("positive reciprocal uses the existing outward operator"),StrongConvexCurveMinimum::DividePositive(Interval::Exact(1),Interval::Exact(25),frame.InverseSquaredNorm)))return false;
   const auto patch=SharedSurfaceObbTests::Surface(false,extrusion);const auto hull=P::Hull(patch.ControlPoints);
   const auto n=GroupedBoxDetail::ProjectHull(patch.ControlPoints,frame.Normal),t=GroupedBoxDetail::ProjectHull(patch.ControlPoints,frame.Tangent);
   FGroupedBoxVolumeBounds shifted;if(!SharedSurfaceObbTests::Volume(shifted,extrusion,true))return false;
   P::FSharedQuerySupports cache(false);Interval projections[3];uint8 mask=0;FDistanceBudget work{10000};
   if(!TestTrue(TEXT("nonunit3/4 frame uses outward inverse; zero and negative coefficients are valid"),cache.ProjectNode(0,frame,n,t,hull,shifted,work,projections,mask)&&mask==7))return false;
   FVector3d exactPoint=FVector3d::ZeroVector;exactPoint[d]=.0625;exactPoint.Z=2.+.0625+.2*.0625*.0625;
   for(int32 k=0;k<3;++k){const auto p=GroupedBoxDetail::Project(Detail::Point(exactPoint),shifted.Axes[k+3]);if(!TestTrue(TEXT("same point survives long-extrusion clipping and inverse reprojection"),projections[k].Lower<=p.Lower&&p.Upper<=projections[k].Upper))return false;}
  }
 }
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStaticSharedSurfaceObbBudgetTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.StaticSharedSurfaceObbBudgetAndModes",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStaticSharedSurfaceObbBudgetTest::RunTest(const FString&)
{
 FScopedCVar<int32> directionalOracleOff(TEXT("p.IAmSpeed.AnalyticWorld.NativeDirectionalMotionClearance"),0,FName(TEXT("FStaticSharedSurfaceObbBudgetTestDirectionalOracle")));
 using namespace Speed;using namespace Analytic;using namespace Certified;namespace H=StaticBodyHierarchy;namespace T=SharedSurfaceObbTests;
 const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("RNE support budget"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 FScopedCVar<int32> mode(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy"),3,FName(TEXT("SharedObbBudgetMode"))),
  depth(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.Depth"),3,FName(TEXT("SharedObbBudgetDepth"))),
  enabled(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.SharedSupportSAT"),1,FName(TEXT("SharedObbBudgetEnabled")));
 const auto source=T::Surface();FDistanceBudget init{100000};FIndexedIntervalUnionPtr scene;
 if(!StaticBodyHierarchyTests::Build(MakeArrayView(&source,1),init,scene,true)||!scene->GetStaticBodyHierarchy())return false;
 FGroupedBoxVolumeBounds crossing;if(!T::Volume(crossing,0,true))return false;
 for(int32 available=0;available<=256;++available)
 {
  TArray<H::FLeaf,TInlineAllocator<16>> leaves;leaves.Add({});FDistanceBudget work{available+17,13,17};
  const auto result=H::BuildFrontier(*scene,0,crossing,16383,work,leaves);
  if(!TestTrue(TEXT("spent optional work is never refunded; incomplete/crossing frontiers never become Clear"),
   (result==H::EFrontier::Work||result==H::EFrontier::Ready)&&work.Remaining>=17&&work.ConsumptionFloor==17&&work.Used+uint64(work.Remaining)==uint64(available+30)&&
   (result==H::EFrontier::Ready?!leaves.IsEmpty():leaves.IsEmpty())))return false;
 }
 TArray<H::FLeaf,TInlineAllocator<16>> leaves;FDistanceBudget overflow{1000,MAX_uint64,17};
 if(!TestTrue(TEXT("overflow is terminal without output or credit"),H::BuildFrontier(*scene,0,crossing,16383,overflow,leaves)==H::EFrontier::Work&&leaves.IsEmpty()&&overflow.Used==MAX_uint64))return false;
 auto nonXYZ=crossing;Swap(nonXYZ.Axes[0],nonXYZ.Axes[3]);FDistanceBudget unsupported{10000,7,17};
 if(!TestTrue(TEXT("no presumed XYZ position or clipping on arbitrary axes"),H::BuildFrontier(*scene,0,nonXYZ,16383,unsupported,leaves)==H::EFrontier::Unavailable&&unsupported.Used==7))return false;
 const uint32 saved=VectorGetControlRegister();VectorSetControlRegister(saved^uint32(0x8000));FDistanceBudget changed{10000,7,17};
 if(!TestTrue(TEXT("worker control mismatch refuses atlas before any support work"),H::BuildFrontier(*scene,0,crossing,16383,changed,leaves)==H::EFrontier::Unavailable&&changed.Used==7))return false;
 VectorSetControlRegister(saved);VectorSetControlRegister((saved&~uint32(0x6000))|uint32(0x2000));FDistanceBudget rounding{10000,7,17};
 if(!TestTrue(TEXT("non-RNE remains invalid, not a cached admission"),H::BuildFrontier(*scene,0,crossing,16383,rounding,leaves)==H::EFrontier::Invalid&&rounding.Used==7))return false;
 VectorSetControlRegister(saved);H::Private::FSharedQuerySupports cache(false);FDistanceBudget allowance{10000};
 for(int32 i=0;i<200;++i)cache.PayOptional(allowance);
 TestTrue(TEXT("local support work stops at128 inside the same finite account"),cache.Used==128&&allowance.Used==128);
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
#endif

namespace Speed::Analytic::Certified
{
namespace DirectionalMotionControl
{
static TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.NativeDirectionalMotionClearance"),0,
 TEXT("Default OFF. Alternative coarse body proof from complete native directional supports and the published depth3 atlas. Keeps original fine14 reserve and fresh primitive endpoint constraints."));
bool RuntimeEnabled(){return Enabled.GetValueOnAnyThread()==1;}
}
EDirectionalProof FDirectionalRegionBuilder::GetSlab(const FVector3d& axis,FDistanceBudget& budget,Interval& output)
{
 output={};const auto before=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
 if(before.Status!=EAnalyticNumericStatus::Complete||before.ControlFingerprint()!=Numeric||!GroupedBoxDetail::ValidAxis(axis))return EDirectionalProof::Invalid;
 if(!budget.Consume())return EDirectionalProof::Work;
 for(const auto& item:Slabs)
 {
  bool same=true;for(int32 d=0;d<3;++d)same&=::AnalyticCertified::Bits(item.Axis[d])==::AnalyticCertified::Bits(axis[d]);
  if(same){output=item.Bounds;if(Observed)++Observed->SupportHits;return EDirectionalProof::Clear;}
 }
 if(Slabs.Num()>=MaximumDirections)return EDirectionalProof::Unavailable;
 Interval bound;
 if(!Support(axis,budget,bound))return EDirectionalProof::Unavailable;
 const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
 if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=Numeric||!::AnalyticCertified::Valid(bound))return EDirectionalProof::Invalid;
 if(!budget.Consume())return EDirectionalProof::Work;
 Slabs.Add({axis,bound});output=bound;
 if(Observed){++Observed->SupportBuilds;Observed->Directions=Slabs.Num();}
 return EDirectionalProof::Clear;
}
EDirectionalProof FReferenceDirectionalMotionStrictSeparation::TryProve(FReferenceBoundsCandidateSetPtr partition,
 FCompleteMotionSupport support,FDistanceBudget& budget,FReferenceDirectionalMotionStrictSeparationPtr& output,FDirectionalMotionObservation* observed)
{
 output.Reset();if(observed)*observed={};
 const auto before=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
 if(before.Status!=EAnalyticNumericStatus::Complete||!partition||!partition->GetScene()||!Detail::ValidPoint(partition->GetQueryHull())||
  partition->GetCandidateIndices().Num()+partition->GetExcludedIndices().Num()!=partition->GetScene()->Num())return EDirectionalProof::Invalid;
 if(!budget.Consume())return EDirectionalProof::Work;
 FDirectionalRegionBuilder region(partition->GetQueryHull(),support,before.ControlFingerprint(),observed);
 for(int32 index:partition->GetCandidateIndices())
 {
  if(observed)++observed->Providers;
  const auto result=StaticBodyHierarchy::TryClearDirectionalProvider(*partition->GetScene(),index,region,budget,observed);
  if(result!=EDirectionalProof::Clear)return result;
 }
 const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
 if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=before.ControlFingerprint())return EDirectionalProof::Invalid;
 if(!budget.Consume())return EDirectionalProof::Work;
 TSharedPtr<FReferenceDirectionalMotionStrictSeparation,ESPMode::ThreadSafe> result(new FReferenceDirectionalMotionStrictSeparation);
 result->Partition=MoveTemp(partition);result->Slabs=MoveTemp(region.Slabs);result->Numeric=after.ControlFingerprint();
 output=MoveTemp(result);return EDirectionalProof::Clear;
}
bool FReferenceDirectionalMotionStrictSeparation::Matches(const FIndexedIntervalUnionPtr& scene,const FIntervalPoint& broad)const
{
 const auto current=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
 return current.Status==EAnalyticNumericStatus::Complete&&current.ControlFingerprint()==Numeric&&Partition&&Partition->GetScene()==scene&&
  GroupedBoxDetail::SamePoint(Partition->GetQueryHull(),broad);
}
bool FReferenceDirectionalMotionStrictSeparation::ContainsCompleteEndpointGroup(TConstArrayView<FIntervalPoint> points)const
{
 if(!Partition||points.Num()<8||points.Num()>32||!Matches(Partition->GetScene(),Partition->GetQueryHull()))return false;
 for(const auto& point:points)if(!GroupedBoxDetail::Within(Partition->GetQueryHull(),point))return false;
 for(const auto& slab:Slabs)
 {
  const auto actual=GroupedBoxDetail::ProjectHull(points,slab.Axis);
  if(!::AnalyticCertified::Valid(actual)||actual.Lower<slab.Bounds.Lower||actual.Upper>slab.Bounds.Upper)return false;
 }
 const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
 return after.Status==EAnalyticNumericStatus::Complete&&after.ControlFingerprint()==Numeric;
}
}

namespace Speed::Analytic::Certified::StaticBodyTubes
{
static bool TryClearDirectional(const FIndexedIntervalUnion&,int32,FDirectionalRegionBuilder&,
 const FVector3d&,const FVector3d&,uint64,FDistanceBudget&);
}

namespace Speed::Analytic::Certified::StaticBodyHierarchy
{
EDirectionalProof TryClearDirectionalProvider(const FIndexedIntervalUnion& scene,int32 provider,
 FDirectionalRegionBuilder& region,FDistanceBudget& budget,FDirectionalMotionObservation* observed)
{
 const auto& atlas=scene.GetStaticBodyHierarchy();
 if(!atlas||!atlas->Roots.IsValidIndex(provider)||atlas->Roots[provider]==INDEX_NONE||atlas->PreparedDepth!=3||atlas->SupportFrames.IsEmpty())return EDirectionalProof::Unavailable;
 const auto before=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
 if(before.Status!=EAnalyticNumericStatus::Complete)return EDirectionalProof::Invalid;
 if(before.ControlFingerprint()!=atlas->Numeric)return EDirectionalProof::Unavailable;
 TArray<int32,TInlineAllocator<8>> stack;stack.Add(atlas->Roots[provider]);
 while(!stack.IsEmpty())
 {
  if(!budget.Consume())return EDirectionalProof::Work;
  const int32 index=stack.Pop(EAllowShrinking::No);if(!atlas->Nodes.IsValidIndex(index))return EDirectionalProof::Invalid;
  const auto& node=atlas->Nodes[index];if(observed)++observed->Nodes;
  bool separated=false;
  for(int32 d=0;d<3;++d)
  {const auto a=region.GetBroadHull().Axis[d],s=node.Hull.Axis[d];if(!Private::A::Valid(s))return EDirectionalProof::Invalid;separated|=a.Upper<s.Lower||s.Upper<a.Lower;}
  if(separated){if(observed)++observed->AabbClears;continue;}
  if(node.SupportFrame!=INDEX_NONE)
  {
   if(!atlas->SupportFrames.IsValidIndex(node.SupportFrame)||!Private::A::Valid(node.NormalSupport)||!Private::A::Valid(node.TangentSupport))return EDirectionalProof::Invalid;
   const auto& frame=atlas->SupportFrames[node.SupportFrame];
   // Normal first: tangent is paid only if the normal is inconclusive.
   for(int32 which=0;which<2&&!separated;++which)
   {
    Interval body;const auto status=region.GetSlab(which?frame.Tangent:frame.Normal,budget,body);
    if(status!=EDirectionalProof::Clear)return status;
    const auto source=which?node.TangentSupport:node.NormalSupport;
    separated=body.Upper<source.Lower||source.Upper<body.Lower;
   }
   if(separated){if(observed)++observed->DirectionalClears;continue;}
   if(node.FirstPoint!=INDEX_NONE&&StaticBodyTubes::TryClearDirectional(scene,provider,region,
    frame.Normal,frame.Tangent,before.ControlFingerprint(),budget))
       return EDirectionalProof::Clear;
  }
  if(node.FirstPoint!=INDEX_NONE)return EDirectionalProof::Unresolved;
  if(!atlas->Nodes.IsValidIndex(node.Left)||!atlas->Nodes.IsValidIndex(node.Right))return EDirectionalProof::Invalid;
  stack.Add(node.Right);stack.Add(node.Left);
 }
 const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
 if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=before.ControlFingerprint())return EDirectionalProof::Invalid;
 return EDirectionalProof::Clear;
}
}

#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Analytic::Certified::DirectionalRegionTests
{
static FIntervalTensorSurface Surface(double shift)
{
 FIntervalTensorSurface s;s.DegreeU=s.DegreeV=5;
 for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)
 {
  const double y=-1.+2.*double(v)/5.;
  const double z=-1.+shift+.1+1.6*double(v)/5.+.4*double(v*(v-1))/20.;
  s.ControlPoints.Add(Detail::Point(FVector3d(-2.+4.*double(u)/5.,y,z)));
 }
 return s;
}
static FIntervalPoint Broad()
{FIntervalPoint p;for(auto& x:p.Axis)x={-3.,3.,true};return p;}
static FBoxVolumeGroup Box(FVector3d center,double radius)
{
 FBoxVolumeGroup group;
 for(int32 bits=0;bits<8;++bits)
 {FVector3d point;for(int32 d=0;d<3;++d)point[d]=center[d]+((bits&(1<<d))?radius:-radius);group.Corners.Add(Detail::Point(point));}
 return group;
}
static bool Scene(bool crossing,FIndexedIntervalUnionPtr& output)
{
 TArray<FIntervalTensorSurface> all;all.Add(Surface(2.));if(crossing)all.Add(Surface(0.));
 FDistanceBudget budget{100000};if(!FIndexedIntervalUnion::Build(all,budget,output))return false;
 output->PrepareStaticBodyHierarchyAtPublication(budget);return !!output->GetStaticBodyHierarchy();
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDirectionalRegionFullSourceTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.DirectionalRegionFullSourceAndEndpoint",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDirectionalRegionFullSourceTest::RunTest(const FString&)
{
 using namespace Speed;using namespace Analytic;using namespace Certified;namespace T=DirectionalRegionTests;
 const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("RNE directional region"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 FScopedCVar<int32> flag(TEXT("p.IAmSpeed.AnalyticWorld.NativeDirectionalMotionClearance"),1,FName(TEXT("DirectionalRegionOn")));
 FScopedCVar<int32> mode(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy"),3,FName(TEXT("DirectionalRegionAtlas")));
 FScopedCVar<int32> depth(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.Depth"),3,FName(TEXT("DirectionalRegionDepth")));
 const uint32 saved=VectorGetControlRegister();
 for(uint32 flush:{0u,64u,32768u,32832u})for(int32 reduced:{0,1})
 {
  VectorSetControlRegister((saved&~uint32(0x807f))|flush);
  FScopedCVar<int32> reduce(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.DegreeReduction"),reduced,FName(TEXT("DirectionalRegionReduction")));
  FIndexedIntervalUnionPtr scene;if(!T::Scene(false,scene))return false;
  FDistanceBudget selection{10000};FReferenceBoundsCandidateSetPtr partition;
  if(!FReferenceBoundsCandidateSet::Select(scene,T::Broad(),selection,partition)||partition->GetCandidateIndices().Num()!=1)return false;
  auto body=T::Box(FVector3d::ZeroVector,.1);
  const auto support=[&](const FVector3d& axis,FDistanceBudget& budget,Interval& out)
  {out={};if(!budget.Consume())return false;out=GroupedBoxDetail::ProjectHull(body.Corners,axis);return ::AnalyticCertified::Valid(out);};
  FDistanceBudget budget{1000};FDirectionalMotionObservation observation;FReferenceDirectionalMotionStrictSeparationPtr proof;
  if(!TestTrue(TEXT("actual atlas support route clears ALL source domains"),FReferenceDirectionalMotionStrictSeparation::TryProve(partition,support,budget,proof,&observation)==EDirectionalProof::Clear&&
   proof&&proof->Matches(scene,T::Broad())&&proof->GetDirectionCountForInspection()>0&&observation.DirectionalClears>0&&observation.Providers==1))return false;
  if(!TestTrue(TEXT("actual complete body group is admitted"),proof->ContainsCompleteEndpointGroup(body.Corners)))return false;
  // This point is in the old broad region but on the source. A proof for
  // Broad alone would be false; all new constraints MUST survive to endpoint.
  const auto escaped=T::Box(FVector3d(0,0,2),.01);
  for(const auto& point:escaped.Corners)if(!GroupedBoxDetail::Within(T::Broad(),point))return false;
  if(!TestFalse(TEXT("endpoint inside old broad region outside directional region is denied"),proof->ContainsCompleteEndpointGroup(escaped.Corners)))return false;
  // The old exact grouped solver independently agrees on the smaller actual
  // body. This is NOT a transfer of its old broad/slab region into our proof.
  TArray<FBoxVolumeGroup> groups{body};TArray<FVector3d> axes{FVector3d(1,0,0),FVector3d(0,1,0),FVector3d(0,0,1)};
  FGroupedBoxVolumeBounds grouped;FDistanceBudget oldBudget{10000};FReferenceBoxVolumeStrictSeparationPtr old;
  if(!TestTrue(TEXT("independent exact grouped proof on actual body remains clear"),CloseGroupedBoxVolumeBounds(T::Broad(),groups,axes,oldBudget,grouped)&&
   FReferenceBoxVolumeStrictSeparation::TryProve(partition,grouped,oldBudget,old)&&old&&old->ContainsCompleteEndpointGroup(body.Corners)))return false;
  FIndexedIntervalUnionPtr other;if(!T::Scene(false,other))return false;
  if(!TestFalse(TEXT("source identity is retained, not just equivalent coefficients"),proof->Matches(other,T::Broad())))return false;
  FIndexedIntervalUnionPtr crossing;if(!T::Scene(true,crossing))return false;FReferenceBoundsCandidateSetPtr crossed;
  if(!FReferenceBoundsCandidateSet::Select(crossing,T::Broad(),selection,crossed))return false;
  auto stale=proof;FDistanceBudget unresolved{1000};
  if(!TestTrue(TEXT("one intersecting last provider prevents publication for the union"),FReferenceDirectionalMotionStrictSeparation::TryProve(crossed,support,unresolved,stale)!=EDirectionalProof::Clear&&!stale&&unresolved.Used>0))return false;
  // Deep ownership: mutating the callback's source after proof cannot change
  // its region, and the final consumer still rejects the escaped group.
  body=escaped;
  if(!TestFalse(TEXT("query storage mutation cannot enlarge a retained region"),proof->ContainsCompleteEndpointGroup(body.Corners)))return false;
 }
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDirectionalRegionControlsBudgetTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.DirectionalRegionControlsBudgetAndNoRetry",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDirectionalRegionControlsBudgetTest::RunTest(const FString&)
{
 using namespace Speed;using namespace Analytic;using namespace Certified;namespace T=DirectionalRegionTests;
 const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("RNE directional budget"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 FScopedCVar<int32> flag(TEXT("p.IAmSpeed.AnalyticWorld.NativeDirectionalMotionClearance"),1,FName(TEXT("DirectionalBudgetOn")));
 FScopedCVar<int32> mode(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy"),3,FName(TEXT("DirectionalBudgetAtlas")));
 FScopedCVar<int32> depth(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.Depth"),3,FName(TEXT("DirectionalBudgetDepth")));
 const uint32 saved=VectorGetControlRegister();
 for(uint32 flush:{0u,64u,32768u,32832u})
 {
  const uint32 control=(saved&~uint32(0x807f))|flush;VectorSetControlRegister(control);
  FIndexedIntervalUnionPtr scene;if(!T::Scene(false,scene))return false;FDistanceBudget select{1000};FReferenceBoundsCandidateSetPtr partition;
  if(!FReferenceBoundsCandidateSet::Select(scene,T::Broad(),select,partition))return false;
  const auto body=T::Box(FVector3d::ZeroVector,.1);
  const auto support=[&](const FVector3d& axis,FDistanceBudget& budget,Interval& out)
  {out={};if(!budget.Consume())return false;out=GroupedBoxDetail::ProjectHull(body.Corners,axis);return ::AnalyticCertified::Valid(out);};
  FDistanceBudget generous{1000};FReferenceDirectionalMotionStrictSeparationPtr reference;
  if(FReferenceDirectionalMotionStrictSeparation::TryProve(partition,support,generous,reference)!=EDirectionalProof::Clear)return false;
  const int32 required=int32(generous.Used);
  for(int32 cap=0;cap<=required+1;++cap)
  {
   FDistanceBudget budget{7+cap,13,7};auto result=reference;
   const auto status=FReferenceDirectionalMotionStrictSeparation::TryProve(partition,support,budget,result);
   if(!TestTrue(TEXT("same finite account never credits or publishes partial proof"),(status==EDirectionalProof::Clear)==(cap>=required)&&bool(result)==(status==EDirectionalProof::Clear)&&
    budget.ConsumptionFloor==7&&budget.Remaining>=7&&budget.Used>=13&&budget.Used-13+uint64(budget.Remaining)==uint64(7+cap)))return false;
  }
  FDistanceBudget overflow{100,MAX_uint64-1,7};auto refused=reference;
  if(!TestTrue(TEXT("overflow consumes its last unit once and publishes nothing"),FReferenceDirectionalMotionStrictSeparation::TryProve(partition,support,overflow,refused)!=EDirectionalProof::Clear&&!refused&&overflow.Used==MAX_uint64&&overflow.Remaining==99))return false;
  VectorSetControlRegister(control^64u);FDistanceBudget mismatch{100};
  if(!TestTrue(TEXT("workers do not borrow an atlas of another control mode"),FReferenceDirectionalMotionStrictSeparation::TryProve(partition,support,mismatch,refused)==EDirectionalProof::Unavailable&&!refused))return false;
  VectorSetControlRegister(control);FDistanceBudget mutate{100};
  const auto changed=[&](const FVector3d& axis,FDistanceBudget& budget,Interval& out)
  {const bool result=support(axis,budget,out);VectorSetControlRegister(VectorGetControlRegister()^64u);return result;};
  const auto bad=FReferenceDirectionalMotionStrictSeparation::TryProve(partition,changed,mutate,refused);VectorSetControlRegister(control);
  if(!TestTrue(TEXT("a callback cannot change controls then publish"),bad==EDirectionalProof::Invalid&&!refused&&mutate.Used>0))return false;
  const auto invalid=[](const FVector3d&,FDistanceBudget&,Interval& out){out={};return true;};FDistanceBudget invalidWork{100};
  if(!TestTrue(TEXT("unknown bounds cannot become a clear certificate"),FReferenceDirectionalMotionStrictSeparation::TryProve(partition,invalid,invalidWork,refused)==EDirectionalProof::Invalid&&!refused))return false;
  // No lazy init: an otherwise equivalent ordinary source without the
  // publication phase is unavailable and remains unprepared after the query.
  auto source=T::Surface(2.);FIndexedIntervalUnionPtr ordinary;FDistanceBudget build{10000};
  if(!FIndexedIntervalUnion::Build(MakeArrayView(&source,1),build,ordinary))return false;FReferenceBoundsCandidateSetPtr missing;
  if(!FReferenceBoundsCandidateSet::Select(ordinary,T::Broad(),build,missing))return false;FDistanceBudget none{100};
  if(!TestTrue(TEXT("no frame-time atlas build or hidden retry"),FReferenceDirectionalMotionStrictSeparation::TryProve(missing,support,none,refused)==EDirectionalProof::Unavailable&&!refused&&!ordinary->GetStaticBodyHierarchy()))return false;
 }
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
#endif


// Immutable outward refit of proposed Legacy facet topology. A clear-only
// acceleration of ORIGINAL interval families; no Legacy hit, normal or TOI.
namespace Speed::Analytic::Certified::StaticBodyTubes
{
namespace A=::AnalyticCertified;
namespace D=StaticBodyHierarchy::DegreeReduction;
static TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.StaticLegacyBodyTubes"),0,
 TEXT("Default OFF. Publication-only original-family tubes on existing facet topology; grouped14 strict-clear only, exact fallback."));
bool RuntimeEnabled(){return Enabled.GetValueOnAnyThread()==1;}
static TAutoConsoleVariable<int32> RefineNearFacet(
 TEXT("p.IAmSpeed.AnalyticWorld.StaticLegacyBodyTubeRefine"),0,
 TEXT("Default OFF. Subdivide inconclusive certified rectangles within the same proof budget."));
static TAutoConsoleVariable<int32> DirectionalRetry(
 TEXT("p.IAmSpeed.AnalyticWorld.StaticLegacyBodyTubeDirectional"),0,
 TEXT("Default OFF. Try the prepared surface tubes on existing coarse motion slabs before building the fine volume."));
static TAutoConsoleVariable<int32> StationaryRetry(
 TEXT("p.IAmSpeed.AnalyticWorld.StaticLegacyBodyTubeStationary"),0,
 TEXT("Default OFF. Use prepared surface tubes for stationary single-box separation."));
static bool RuntimeStationaryEnabled(){return RuntimeEnabled()&&StationaryRetry.GetValueOnAnyThread()==1;}
constexpr int32 MaxProviders=256,MaxSegments=32768,MaxPerProvider=1024;
constexpr int32 QueryWork=1024;
constexpr uint64 MaxBytes=32ull*1024*1024;
constexpr double ErrorLimit=.001;
static TAutoConsoleVariable<int32> LocalAxisRetry(
 TEXT("p.IAmSpeed.AnalyticWorld.StaticLegacyBodyTubeLocalAxis"),0,
 TEXT("Default OFF. Retry near coarse leaves on a prepared local direction with fresh motion support."));
struct FFacet
{
 FIntervalPoint Corners[4];
 FIntervalPoint Hull;
 FVector3d LocalAxis=FVector3d::ZeroVector;
};

// Arbitrary finite axes are valid witnesses. Quantization only shares the
// expensive dynamic support between adjacent facets. It supplies no normal
// to the simulation and never replaces the projection of the full tube.
static FVector3d PrepareLocalAxis(const FVector3d& along,const FVector3d& extrusion)
{
 const auto normal=FVector3d::CrossProduct(along,extrusion);
 const double scale=normal.GetAbsMax();
 if(!FMath::IsFinite(scale)||scale<=0.)return FVector3d::ZeroVector;
 FVector3d axis;
 for(int32 d=0;d<3;++d)
  axis[d]=double(FMath::RoundToInt(16.*normal[d]/scale))/16.;
 for(int32 d=0;d<3;++d)
 {
  if(axis[d]==0.)continue;
  if(axis[d]<0.)axis=-axis;
  break;
 }
 for(int32 d=0;d<3;++d)if(axis[d]==0.)axis[d]=0.;
 return axis;
}
struct FNode {FIntervalPoint Hull;int32 Left=INDEX_NONE,Right=INDEX_NONE,First=0,Count=0;};
static TAutoConsoleVariable<int32> FrontierEnabled(
 TEXT("p.IAmSpeed.AnalyticWorld.StaticLegacyBodyTubeFrontier"),0,
 TEXT("Default OFF. Continue the exact proof on the original source domains unresolved by the tube BVH."));
struct FOriginalLeaf
{
 FIntervalPoint Controls[12];
 double Min=0.,Max=1.;
 int32 Depth=0;
};
using FTubeLeaves=TArray<StaticBodyHierarchy::FLeaf,TInlineAllocator<16>>;
struct FEntry
{
 TArray<FFacet> Facets;
 TArray<FNode> Nodes;
 TArray<int32> Indices;
 TArray<FOriginalLeaf> OriginalLeaves;
 bool FreeAlongU=true;
};
static bool Pay(FDistanceBudget& b,uint64 n){return CandidateDetail::ConsumeRepeated(b,n);}
static FIntervalPoint Hull(FIntervalPoint a,const FIntervalPoint& b)
{return Detail::ValidPoint(a)?TimeDetail::Hull(a,b):b;}
// Each tube already encloses the entire source leaf in the bilinear
// rectangle's interval image. The two half rectangles enclose that same
// image by outward de Casteljau. Each half must clear every remaining group.
template<typename FSeparate>
static bool RefineFacet(const FFacet& facet,uint32 groups,FDistanceBudget& budget,FSeparate&& separate)
{
 constexpr int32 RefineWork=256,MaxDepth=6;
 const int32 floor=FMath::Max(budget.ConsumptionFloor,budget.Remaining-RefineWork);
 FScopedDistanceBudgetFloor local(budget,floor);
 if(!local.IsValid())return false;
 struct FCell {FIntervalPoint Corners[4];uint32 Groups=0;int32 Depth=0;};
 FCell root;root.Groups=groups;
 for(int32 i=0;i<4;++i)root.Corners[i]=facet.Corners[i];
 TArray<FCell,TInlineAllocator<MaxDepth+1>> stack;stack.Add(root);
 while(!stack.IsEmpty())
 {
  auto cell=stack.Pop(EAllowShrinking::No);
  if(cell.Depth>0)
  {
   if(!budget.Consume()||!separate(cell.Corners,4,facet.Hull,cell.Groups))return false;
   if(!cell.Groups)continue;
  }
  if(cell.Depth==MaxDepth||!Pay(budget,8))return false;
  // The extrusion is usually the long dimension. Narrow it first,
  // then refine both axes when the strict axes remain inconclusive.
  const int32 step=cell.Depth<3||(cell.Depth%2)==0?2:1;
  FCell lower=cell,upper=cell;++lower.Depth;++upper.Depth;
  for(int32 i=0;i<4;++i)
  {
   if(i&step)continue;
   const int32 opposite=i|step;
   FIntervalPoint midpoint;
   for(int32 d=0;d<3;++d)midpoint.Axis[d]=A::HalfSum(cell.Corners[i].Axis[d],cell.Corners[opposite].Axis[d]);
   if(!Detail::ValidPoint(midpoint))return false;
   lower.Corners[opposite]=midpoint;
   upper.Corners[i]=midpoint;
  }
  stack.Add(upper);stack.Add(lower);
 }
 return true;
}

static FAtlasPtr Prepare(TConstArrayView<FIntervalTensorSurface>,TConstArrayView<FInput>,FDistanceBudget&,uint64,uint64&);
static EResult Traverse(const FIndexedIntervalUnion&,int32,const FGroupedBoxVolumeBounds&,uint32,
 FDistanceBudget&,FTubeLeaves*);
class FAtlas final
{
 friend FAtlasPtr Prepare(TConstArrayView<FIntervalTensorSurface>,TConstArrayView<FInput>,FDistanceBudget&,uint64,uint64&);
 friend class ::Speed::Analytic::Certified::FIndexedIntervalUnion;
 friend EResult Traverse(const FIndexedIntervalUnion&,int32,const FGroupedBoxVolumeBounds&,uint32,
  FDistanceBudget&,FTubeLeaves*);
 friend bool TryClearDirectional(const FIndexedIntervalUnion&,int32,FDirectionalRegionBuilder&,
  const FVector3d&,const FVector3d&,uint64,FDistanceBudget&);
 TArray<int32> Roots;TArray<FEntry> Entries;uint64 Numeric=0,Bytes=0;
};
static bool Observe()
{static const IConsoleVariable* p=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"));return p&&p->GetInt()==1;}
struct FCounters {std::atomic<uint64> Calls{0},Ready{0},Cleared{0},Unresolved{0},Unavailable{0},Invalid{0},LeafNear{0},RefineCalls{0},RefineClears{0},WorkStop{0},ArithmeticStop{0},Nodes{0},Facets{0},Work{0},Ticks{0};};
static FCounters Counts;
static std::atomic<uint64> CoarseCalls{0},CoarseReady{0},CoarseClears{0};
static std::atomic<uint64> LocalAxisAttempts{0},LocalAxisLeavesClear{0};
static std::atomic<uint64> FrontierReady{0},FrontierLeaves{0};
static FCounters StationaryCounts;
static void Publish()
{
 if(Observe()&&FrontierReady.load())
  UE_LOG(LogTemp,Display,TEXT("[StaticLegacyBodyTubeFrontier] Scope=ProgramPrefixAtPreExit Ready=%llu Leaves=%llu"),
   FrontierReady.load(),FrontierLeaves.load());
 if(Observe()&&StationaryCounts.Calls.load())
 {
  const auto& s=StationaryCounts;
  UE_LOG(LogTemp,Display,TEXT("[StaticLegacyBodyTubeStationary] Scope=ProgramPrefixAtPreExit Calls=%llu Ready=%llu Clears=%llu Unresolved=%llu Unavailable=%llu Invalid=%llu Work=%llu Ticks=%llu SecondsPerCycle=%.17g"),
   s.Calls.load(),s.Ready.load(),s.Cleared.load(),s.Unresolved.load(),s.Unavailable.load(),s.Invalid.load(),
   s.Work.load(),s.Ticks.load(),FPlatformTime::GetSecondsPerCycle64());
 }
 if(Observe()&&LocalAxisAttempts.load())
  UE_LOG(LogTemp,Display,TEXT("[StaticLegacyBodyTubeLocalAxis] Scope=ProgramPrefixAtPreExit Attempts=%llu LeavesClear=%llu"),
   LocalAxisAttempts.load(),LocalAxisLeavesClear.load());
 if(Observe()&&CoarseCalls.load())UE_LOG(LogTemp,Display,TEXT("[StaticLegacyBodyTubeDirectional] Scope=ProgramPrefixAtPreExit Calls=%llu Ready=%llu Clears=%llu"),
  CoarseCalls.load(),CoarseReady.load(),CoarseClears.load());
 if(Observe())UE_LOG(LogTemp,Display,TEXT("[StaticLegacyBodyTubes] Scope=ProgramPrefixAtPreExit NoPASS=1 Calls=%llu Ready=%llu Cleared=%llu Unresolved=%llu Unavailable=%llu Invalid=%llu LeafNear=%llu RefineCalls=%llu RefineClears=%llu WorkStop=%llu ArithmeticStop=%llu Nodes=%llu Facets=%llu Work=%llu Ticks=%llu SecondsPerCycle=%.17g"),
 Counts.Calls.load(),Counts.Ready.load(),Counts.Cleared.load(),Counts.Unresolved.load(),Counts.Unavailable.load(),Counts.Invalid.load(),Counts.LeafNear.load(),Counts.RefineCalls.load(),Counts.RefineClears.load(),Counts.WorkStop.load(),Counts.ArithmeticStop.load(),Counts.Nodes.load(),Counts.Facets.load(),Counts.Work.load(),Counts.Ticks.load(),FPlatformTime::GetSecondsPerCycle64());}
struct FRegister {FRegister(){FCoreDelegates::OnPreExit.AddStatic(&Publish);}};static FRegister Register;

// 2x6 coefficients suffice: the target rectangle is affine, hence its exact
// degree(1,5) elevation uses j/5. No degree7 approximation enters this proof.
static bool CertifyFacet(const Detail::FNode& original,bool freeAlongU,
 const FExtrudedQuinticPatch& p,int32 segment,bool reverseFree,FDistanceBudget& budget,FFacet& out)
{
 // A finite explicit charge covers six directed quotients (24 scalar
 // quotients), 12 coefficient-parameter operations,12 prepared frame
 // operations,36*5 residual operations,12 expansions and16 copies, plus
 // eight setup/validation units (264 total). Failure never publishes a partial facet.
 if(!Pay(budget,264))return false;
 const auto& f=p.SectionFacetGeometry[segment];
 if(f.Origin.ContainsNaN()||f.AxisU.ContainsNaN()||p.ExtrusionAxis.ContainsNaN()||
  !A::Finite(f.HalfExtents.X)||!A::Finite(f.HalfExtents.Y)||f.HalfExtents.X<=0||f.HalfExtents.Y<=0)return false;
 const auto limit=A::Square(Interval::Exact(ErrorLimit));if(!A::Valid(limit))return false;
 FFacet result;const Interval expansion{-ErrorLimit,ErrorLimit,true};
 FIntervalPoint halfU,offset[2];
 for(int32 d=0;d<3;++d)
 {
  halfU.Axis[d]=A::Multiply(Interval::Exact(f.HalfExtents.X),Interval::Exact(f.AxisU[d]));
  const auto halfV=A::Multiply(Interval::Exact(f.HalfExtents.Y),Interval::Exact(p.ExtrusionAxis[d]));
  offset[0].Axis[d]=A::Subtract(Interval::Exact(f.Origin[d]),halfV);
  offset[1].Axis[d]=A::Add(Interval::Exact(f.Origin[d]),halfV);
 }
 if(!Detail::ValidPoint(halfU)||!Detail::ValidPoint(offset[0])||!Detail::ValidPoint(offset[1]))return false;
 for(int32 j=0;j<6;++j)
 {
  Interval t;if(!StrongConvexCurveMinimum::DividePositive(Interval::Exact(double(j)),Interval::Exact(5.),t))return false;
  const auto along=A::Subtract(A::Multiply(Interval::Exact(2.),t),Interval::Exact(1.));
  for(int32 side=0;side<2;++side)
  {
   const auto& cp=original.Net[freeAlongU?side*6+j:j*2+side];
   auto squared=Interval::Exact(0.);FIntervalPoint rectangle;
   for(int32 d=0;d<3;++d)
   {
    const auto r=A::Add(offset[reverseFree?1-side:side].Axis[d],A::Multiply(along,halfU.Axis[d]));
    rectangle.Axis[d]=r;squared=A::Add(squared,A::Square(A::Subtract(cp.Axis[d],r)));
   }
   if(!Detail::ValidPoint(rectangle)||!A::Valid(squared)||squared.Upper>limit.Lower)return false;
   if(j==0||j==5)
   {
    const int32 corner=side*2+(j==5?1:0);
    for(int32 d=0;d<3;++d)result.Corners[corner].Axis[d]=A::Add(rectangle.Axis[d],expansion);
    if(!Detail::ValidPoint(result.Corners[corner]))return false;
    result.Hull=Hull(result.Hull,result.Corners[corner]);
   }
  }
 }
 if(LocalAxisRetry.GetValueOnAnyThread()==1)
 {
  if(!Pay(budget,20))return false;
  result.LocalAxis=PrepareLocalAxis(f.AxisU,p.ExtrusionAxis);
 }
 out=result;return true;
}

static bool BuildEntry(const FIntervalTensorSurface& source,const FInput& input,
 FDistanceBudget& budget,FEntry& out)
{
 out={};if(!input.Patch||source.DegreeU!=5||source.DegreeV!=5||source.ControlPoints.Num()!=36)return false;
 const auto& p=*input.Patch;const int32 segments=p.SectionParameters.Num()-1;
 if(segments<1||segments>MaxPerProvider||p.SectionFacetGeometry.Num()!=segments||p.SectionSegmentBvhIndices.Num()!=segments||
  p.SectionSegmentBvhNodes.IsEmpty()||p.SectionSegmentBvhNodes.Num()>2*segments-1||
  A::Bits(p.SectionParameters[0])!=A::Bits(0.)||A::Bits(p.SectionParameters.Last())!=A::Bits(1.))return false;
 D::FPointwiseRoot reduced;if(D::TryBuild(source,input.FreeAlongU,budget,reduced)!=D::EResult::Ready)return false;
 Detail::FNode root;root.DegreeU=reduced.DegreeU;root.DegreeV=reduced.DegreeV;root.Net.Append(reduced.Net);
 int32 freeAxis=INDEX_NONE;
 for(int32 d=0;d<3;++d)
 {if(A::Bits(p.ExtrusionAxis[d])==A::Bits(1.)){if(freeAxis!=INDEX_NONE)return false;freeAxis=d;}
  else if(A::Bits(p.ExtrusionAxis[d])&A::Magnitude)return false;}
 if(freeAxis==INDEX_NONE)return false;
 const bool reverseFree=root.Net[0].Axis[freeAxis].Lower>root.Net[input.FreeAlongU?6:1].Axis[freeAxis].Upper;
 struct FWork {Detail::FNode Cell;int32 First,Last;double A,B;int32 Depth;};
 TArray<FWork,TInlineAllocator<25>> stack;stack.Add({MoveTemp(root),0,segments,0.,1.,0});
 FEntry result;result.Facets.SetNum(segments);result.FreeAlongU=input.FreeAlongU;
 if(FrontierEnabled.GetValueOnAnyThread()==1&&segments>1)
  result.OriginalLeaves.SetNum(segments);
 while(!stack.IsEmpty())
 {
  if(!budget.Consume())return false;
  auto work=stack.Pop(EAllowShrinking::No);
  if(work.Depth>24||work.First<0||work.Last>segments||work.First>=work.Last||
   A::Bits(p.SectionParameters[work.First])!=A::Bits(work.A)||A::Bits(p.SectionParameters[work.Last])!=A::Bits(work.B))return false;
  if(work.Last==work.First+1)
  {
   if(!CertifyFacet(work.Cell,input.FreeAlongU,p,work.First,reverseFree,budget,result.Facets[work.First]))return false;
   if(!result.OriginalLeaves.IsEmpty())
   {
    if(work.Cell.Net.Num()!=12||!Pay(budget,13))return false;
    auto& saved=result.OriginalLeaves[work.First];
    saved.Min=work.A;saved.Max=work.B;saved.Depth=work.Depth;
    for(int32 i=0;i<12;++i)saved.Controls[i]=work.Cell.Net[i];
   }
   continue;
  }
  if(work.Depth==24)return false;
  const double middle=.5*(work.A+work.B);int32 lo=work.First+1,hi=work.Last;
  while(lo<hi){if(!budget.Consume())return false;const int32 m=lo+(hi-lo)/2;if(p.SectionParameters[m]<middle)lo=m+1;else hi=m;}
  if(lo>=work.Last||A::Bits(p.SectionParameters[lo])!=A::Bits(middle)||!Pay(budget,92))return false;
  Detail::FNode a,b;if(!Detail::Split(work.Cell,!input.FreeAlongU,a,b))return false;
  stack.Add({MoveTemp(b),lo,work.Last,middle,work.B,work.Depth+1});
  stack.Add({MoveTemp(a),work.First,lo,work.A,middle,work.Depth+1});
 }
 // The supplied old bounds are NEVER read. Only a validated topology/order is
 // retained. Every facet appears exactly once; cycles and shared nodes refuse.
 if(!Pay(budget,uint64(segments)+p.SectionSegmentBvhNodes.Num()))return false;
 result.Nodes.SetNum(p.SectionSegmentBvhNodes.Num());result.Indices=p.SectionSegmentBvhIndices;
 TArray<uint8> seenFacet,seenNode;seenFacet.Init(0,segments);seenNode.Init(0,result.Nodes.Num());
 struct FVisit {int32 Node;bool After;};TArray<FVisit,TInlineAllocator<64>> visit;visit.Add({0,false});
 int32 visited=0,covered=0;
 while(!visit.IsEmpty())
 {
  if(!Pay(budget,10))return false;
  const auto at=visit.Pop(EAllowShrinking::No);if(!result.Nodes.IsValidIndex(at.Node))return false;
  const auto& old=p.SectionSegmentBvhNodes[at.Node];auto& node=result.Nodes[at.Node];
  if(at.After)
  {node.Hull=Hull(result.Nodes[node.Left].Hull,result.Nodes[node.Right].Hull);if(!Detail::ValidPoint(node.Hull))return false;continue;}
  if(seenNode[at.Node]++)return false;++visited;
  if(old.IndexCount>0)
  {
   if(old.IndexCount>8||old.FirstIndex<0||old.FirstIndex>segments-old.IndexCount)return false;
   node.First=old.FirstIndex;node.Count=old.IndexCount;
   for(int32 k=0;k<node.Count;++k)
   {if(!budget.Consume())return false;const int32 f=result.Indices[node.First+k];if(!seenFacet.IsValidIndex(f)||seenFacet[f]++)return false;++covered;node.Hull=Hull(node.Hull,result.Facets[f].Hull);}
  }
  else
  {
   if(old.LeftChild<0||old.RightChild<0||old.LeftChild==old.RightChild)return false;
   node.Left=old.LeftChild;node.Right=old.RightChild;
   visit.Add({at.Node,true});visit.Add({node.Right,false});visit.Add({node.Left,false});
  }
 }
 if(covered!=segments||visited!=result.Nodes.Num())return false;
 out=MoveTemp(result);return true;
}

static uint64 EntryBytes(const FEntry& e)
{return e.Facets.GetAllocatedSize()+e.Nodes.GetAllocatedSize()+e.Indices.GetAllocatedSize()+e.OriginalLeaves.GetAllocatedSize();}
static FAtlasPtr Prepare(TConstArrayView<FIntervalTensorSurface> sources,TConstArrayView<FInput> inputs,
 FDistanceBudget& budget,uint64 maximumBytes,uint64& retained)
{
 retained=0;const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
 if(numeric.Status!=EAnalyticNumericStatus::Complete||inputs.Num()>MaxProviders||maximumBytes>MaxBytes||
  budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor)return {};
 const uint64 started=FPlatformTime::Cycles64(),used=budget.Used;
 // Array capacities, object and bounded shared ownership overhead are included.
 const uint64 base=sizeof(FAtlas)+256+uint64(sources.Num())*sizeof(int32)+uint64(inputs.Num())*sizeof(FEntry);
 if(base>maximumBytes||!Pay(budget,uint64(sources.Num())+inputs.Num()+1))return {};
 TSharedPtr<FAtlas,ESPMode::ThreadSafe> atlas(new FAtlas);atlas->Numeric=numeric.ControlFingerprint();
 atlas->Roots.Init(INDEX_NONE,sources.Num());atlas->Entries.Reserve(inputs.Num());
 atlas->Bytes=sizeof(FAtlas)+256+atlas->Roots.GetAllocatedSize()+atlas->Entries.GetAllocatedSize();
 int32 segments=0,skipped=0;
 for(const auto& input:inputs)
 {
  if(!budget.Consume())break;
  if(!sources.IsValidIndex(input.Provider)||atlas->Roots[input.Provider]!=INDEX_NONE||!input.Patch){++skipped;continue;}
  const auto& p=*input.Patch;const int32 n=p.SectionParameters.Num()-1;
  if(n<1||n>MaxPerProvider||n>MaxSegments-segments||p.SectionSegmentBvhNodes.Num()>2*n-1){++skipped;continue;}
  // Covers owned staging plus final capacities; no array allocation is allowed
  // to borrow storage outside the caller's remaining shared atlas32MiB cap.
  const uint64 leafBytes=FrontierEnabled.GetValueOnAnyThread()==1?sizeof(FOriginalLeaf):0;
  const uint64 forecast=128*1024+2ull*(uint64(n)*(sizeof(FFacet)+sizeof(int32)+1+leafBytes)+uint64(p.SectionSegmentBvhNodes.Num())*(sizeof(FNode)+1));
  if(atlas->Bytes>maximumBytes||forecast>maximumBytes-atlas->Bytes){++skipped;continue;}
  FEntry entry;if(!BuildEntry(sources[input.Provider],input,budget,entry)){++skipped;continue;}
  const uint64 bytes=EntryBytes(entry);if(bytes>forecast||bytes>maximumBytes-atlas->Bytes){++skipped;continue;}
  atlas->Bytes+=bytes;segments+=n;atlas->Roots[input.Provider]=atlas->Entries.Add(MoveTemp(entry));
 }
 const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
 if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=atlas->Numeric)return {};
 UE_LOG(LogTemp,Display,TEXT("[StaticLegacyBodyTubesBuild] PublicationOnly=1 OriginalFamily=1 Input=%d Ready=%d Skipped=%d Segments=%d Bytes=%llu Work=%llu Ticks=%llu SecondsPerCycle=%.17g"),
 inputs.Num(),atlas->Entries.Num(),skipped,segments,atlas->Bytes,budget.Used-used,FPlatformTime::Cycles64()-started,FPlatformTime::GetSecondsPerCycle64());
 if(atlas->Entries.IsEmpty())return {};retained=atlas->Bytes;return atlas;
}

// Clear here means the traversal is complete. When a frontier is
// requested it owns every remaining obligation, including its group mask.
static EResult Traverse(const FIndexedIntervalUnion& scene,int32 provider,const FGroupedBoxVolumeBounds& volume,
 uint32 active,FDistanceBudget& budget,FTubeLeaves* frontier)
{
 if(frontier)frontier->Reset();
 const int32 groupCount=volume.Groups.Num();
 const bool stationary=groupCount==1;
 if(stationary?!RuntimeStationaryEnabled():(!RuntimeEnabled()||groupCount!=14))return EResult::Unavailable;
 const int32 workLimit=stationary?512:QueryWork;
 auto& counts=stationary?StationaryCounts:Counts;
 const bool observe=Observe();const uint64 started=observe?FPlatformTime::Cycles64():0,used=budget.Used;
 if(observe)++counts.Calls;
 const auto finish=[&](EResult r)
 {if(observe){counts.Work+=budget.Used-used;counts.Ticks+=FPlatformTime::Cycles64()-started;
  if(r==EResult::Clear&&(!frontier||frontier->IsEmpty()))++counts.Cleared;else if(r==EResult::Unresolved)++counts.Unresolved;else if(r==EResult::Invalid)++counts.Invalid;else ++counts.Unavailable;}return r;};
 const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
 if(numeric.Status!=EAnalyticNumericStatus::Complete)return finish(EResult::Invalid);
 const auto atlas=scene.GetStaticBodyTubes();
 if(!atlas||atlas->Numeric!=numeric.ControlFingerprint()||!atlas->Roots.IsValidIndex(provider)||atlas->Roots[provider]==INDEX_NONE)return finish(EResult::Unavailable);
 const int64 minimum=int64(volume.Axes.Num())+volume.Groups.Num()+2;
 if(volume.Axes.IsEmpty()||volume.Axes.Num()>12||volume.Slabs.Num()!=volume.Axes.Num()*groupCount||active==0||active>>groupCount||
  budget.ConsumptionFloor<0||int64(budget.Remaining)-budget.ConsumptionFloor<workLimit+minimum||
  budget.Used>MAX_uint64-uint64(workLimit+minimum))return finish(EResult::Unavailable);
 if(observe)++counts.Ready;
 FScopedDistanceBudgetFloor reserve(budget,budget.Remaining-workLimit);if(!reserve.IsValid())return finish(EResult::Unavailable);
 const auto& entry=atlas->Entries[atlas->Roots[provider]];
 if(frontier&&entry.OriginalLeaves.Num()!=entry.Facets.Num())return finish(EResult::Unavailable);
 struct FWork{int32 Node;uint32 Mask;};TArray<FWork,TInlineAllocator<64>> stack;stack.Add({0,active});
 const auto separate=[&](const FIntervalPoint* corners,int32 count,const FIntervalPoint& hull,uint32& mask)->bool
 {
  Interval projected[12];
  for(int32 a=0;a<volume.Axes.Num();++a)
  {
   if(!budget.Consume())return false;
   projected[a]=count?GroupedBoxDetail::ProjectHull(MakeArrayView(corners,count),volume.Axes[a]):GroupedBoxDetail::Project(hull,volume.Axes[a]);
   if(!A::Valid(projected[a])){if(observe)++counts.ArithmeticStop;return false;}
  }
  for(int32 g=0;g<groupCount;++g)if(mask&(1u<<g))
  {
   if(!budget.Consume())return false;bool clear=false;
   for(int32 a=0;a<volume.Axes.Num();++a)
   {const auto body=volume.Slabs[g*volume.Axes.Num()+a];if(!A::Valid(body)){if(observe)++counts.ArithmeticStop;return false;}clear|=body.Upper<projected[a].Lower||projected[a].Upper<body.Lower;}
   if(clear)mask&=~(1u<<g);
  }
  return true;
 };
 bool complete=true;
 while(!stack.IsEmpty())
 {
  if(!budget.Consume()){complete=false;break;}
  auto work=stack.Pop(EAllowShrinking::No);const auto& node=entry.Nodes[work.Node];if(observe)++counts.Nodes;
  if(!separate(nullptr,0,node.Hull,work.Mask)){complete=false;break;}if(!work.Mask)continue;
  if(node.Count)
  {
   for(int32 k=0;k<node.Count;++k)
   {
    if(!budget.Consume()){complete=false;break;}
    const int32 facetIndex=entry.Indices[node.First+k];
    const auto& facet=entry.Facets[facetIndex];uint32 mask=work.Mask;if(observe)++counts.Facets;
    if(!separate(facet.Corners,4,facet.Hull,mask)){complete=false;break;}
    if(mask)
    {
     if(frontier&&frontier->Num()<32&&Pay(budget,13))
     {
      const auto& source=entry.OriginalLeaves[facetIndex];
      auto& leaf=frontier->AddDefaulted_GetRef();leaf.ActiveGroups=mask;
      auto& cell=leaf.Cell;
      cell.DegreeU=entry.FreeAlongU?1:5;cell.DegreeV=entry.FreeAlongU?5:1;
      if(entry.FreeAlongU){cell.MinV=source.Min;cell.MaxV=source.Max;cell.DepthV=source.Depth;}
      else{cell.MinU=source.Min;cell.MaxU=source.Max;cell.DepthU=source.Depth;}
      cell.Net.Append(source.Controls,12);
      continue;
     }
     if(!frontier&&RefineNearFacet.GetValueOnAnyThread()==1)
     {
      if(observe)++counts.RefineCalls;
      if(RefineFacet(facet,mask,budget,separate))
      {
       if(observe)++counts.RefineClears;
       continue;
      }
     }
     if(observe)++counts.LeafNear;
     complete=false;break;
    }
   }
   if(!complete)break;
  }
  else {stack.Add({node.Right,work.Mask});stack.Add({node.Left,work.Mask});}
 }
 const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
 if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=numeric.ControlFingerprint())return finish(EResult::Invalid);
 if(observe&&!complete&&budget.Remaining==budget.ConsumptionFloor)++counts.WorkStop;
 return finish(complete?EResult::Clear:EResult::Unresolved);
}

EResult TryClear(const FIndexedIntervalUnion& scene,int32 provider,const FGroupedBoxVolumeBounds& volume,
 uint32 active,FDistanceBudget& budget)
{
 if(volume.Groups.Num()==14&&FrontierEnabled.GetValueOnAnyThread()==1)return EResult::Unavailable;
 return Traverse(scene,provider,volume,active,budget,nullptr);
}

static StaticBodyHierarchy::EFrontier TryBuildFrontier(const FIndexedIntervalUnion& scene,int32 provider,
 const FGroupedBoxVolumeBounds& volume,uint32 active,FDistanceBudget& budget,FTubeLeaves& out)
{
 using E=StaticBodyHierarchy::EFrontier;
 if(FrontierEnabled.GetValueOnAnyThread()!=1)return E::Unavailable;
 const auto status=Traverse(scene,provider,volume,active,budget,&out);
 if(status==EResult::Clear)
 {
  if(Observe()){++FrontierReady;FrontierLeaves+=out.Num();}
  return E::Ready;
 }
 out.Reset();
 if(status==EResult::Unavailable)
 {
  const auto clear=Traverse(scene,provider,volume,active,budget,nullptr);
  if(clear==EResult::Clear)return E::Ready;
  if(clear==EResult::Invalid)return E::Invalid;
 }
 return status==EResult::Invalid?E::Invalid:E::Unavailable;
}

static bool TryClearDirectional(const FIndexedIntervalUnion& scene,int32 provider,FDirectionalRegionBuilder& region,
 const FVector3d& normal,const FVector3d& tangent,uint64 numeric,FDistanceBudget& budget)
{
 const bool localAxis=LocalAxisRetry.GetValueOnAnyThread()==1;
 if(!RuntimeEnabled()||(!localAxis&&DirectionalRetry.GetValueOnAnyThread()!=1))return false;
 const bool observe=Observe();if(observe)++CoarseCalls;
 const auto atlas=scene.GetStaticBodyTubes();
 if(!atlas||atlas->Numeric!=numeric||!atlas->Roots.IsValidIndex(provider)||atlas->Roots[provider]==INDEX_NONE)
  return false;
 if(observe)++CoarseReady;
 // The original coarse scope still owns its work floor. Reuse the two
 // complete-motion projections already requested for this provider.
 constexpr int32 WorkLimit=512;
 FScopedDistanceBudgetFloor local(budget,FMath::Max(budget.ConsumptionFloor,budget.Remaining-WorkLimit));
 if(!local.IsValid())return false;
 const FVector3d axes[2]={normal,tangent};
 Interval body[2];
 for(int32 a=0;a<2;++a)
  if(region.GetSlab(axes[a],budget,body[a])!=EDirectionalProof::Clear)return false;
 const auto& entry=atlas->Entries[atlas->Roots[provider]];
 const auto separated=[&](const FIntervalPoint& hull,const FIntervalPoint* corners,int32 count)
 {
  for(int32 a=0;a<2;++a)
  {
   const auto range=count?GroupedBoxDetail::ProjectHull(MakeArrayView(corners,count),axes[a]):
    GroupedBoxDetail::Project(hull,axes[a]);
   if(A::Valid(range)&&(body[a].Upper<range.Lower||range.Upper<body[a].Lower))return true;
  }
  return false;
 };
 TArray<int32,TInlineAllocator<32>> stack;
 stack.Add(0);
 while(!stack.IsEmpty())
 {
  if(!Pay(budget,3))return false;
  const auto& node=entry.Nodes[stack.Pop(EAllowShrinking::No)];
  bool boxClear=false;
  for(int32 d=0;d<3;++d)
  {
   const auto box=region.GetBroadHull().Axis[d],surface=node.Hull.Axis[d];
   boxClear|=box.Upper<surface.Lower||surface.Upper<box.Lower;
  }
  if(boxClear||separated(node.Hull,nullptr,0))continue;
  if(!node.Count){stack.Add(node.Right);stack.Add(node.Left);continue;}
  for(int32 k=0;k<node.Count;++k)
  {
   if(!Pay(budget,9))return false;
   const auto& facet=entry.Facets[entry.Indices[node.First+k]];
   if(separated(facet.Hull,facet.Corners,4))continue;
   if(!localAxis||!GroupedBoxDetail::ValidAxis(facet.LocalAxis)||!Pay(budget,5))return false;
   if(observe)++LocalAxisAttempts;
   Interval localBody;
   if(region.GetSlab(facet.LocalAxis,budget,localBody)!=EDirectionalProof::Clear)return false;
   const auto localSource=GroupedBoxDetail::ProjectHull(MakeArrayView(facet.Corners),facet.LocalAxis);
   if(!A::Valid(localSource)||!(localBody.Upper<localSource.Lower||localSource.Upper<localBody.Lower))return false;
   if(observe)++LocalAxisLeavesClear;
  }
 }
 if(observe)++CoarseClears;
 return true;
}
}

namespace Speed::Analytic::Certified
{
uint64 FIndexedIntervalUnion::PrepareStaticBodyTubesAtPublication(TConstArrayView<StaticBodyTubes::FInput> inputs,
 FDistanceBudget& budget,uint64 maximumBytes) const
{
 if(!IsInGameThread()||!StaticBodyTubes::RuntimeEnabled())return 0;
 uint8 expected=0;if(!StaticTubesState.compare_exchange_strong(expected,uint8(1),std::memory_order_acq_rel))return 0;
 struct FSeal {std::atomic<uint8>& State;~FSeal(){State.store(uint8(2),std::memory_order_release);}} seal{StaticTubesState};
 uint64 retained=0;
 try{StaticTubes=StaticBodyTubes::Prepare(MakeArrayView(Surfaces),inputs,budget,maximumBytes,retained);}
 catch(...){StaticTubes.Reset();retained=0;}
 return retained;
}
}


#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Analytic::Certified::StaticBodyTubesTests
{
namespace A=::AnalyticCertified;
static FIntervalTensorSurface Surface(bool transpose=false,bool reverse=false,double width=0.)
{
 FIntervalTensorSurface s;s.DegreeU=s.DegreeV=5;s.ControlPoints.SetNum(36);
 for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)
 {
  auto p=Detail::Point(FVector3d((reverse?1.:-1.)*(2.-4.*double(u)/5.),-1.+2.*double(v)/5.,3.-4.*double(v)/5.+double(v*(v-1))/5.));
  if(width>0.)for(auto& x:p.Axis){x.Lower-=width;x.Upper+=width;}
  s.ControlPoints[transpose?v*6+u:u*6+v]=p;
 }
 return s;
}
static bool Patch(FExtrudedQuinticPatch& out)
{
 out={};out.ExtrusionAxis=FVector3d(1,0,0);out.MinimumExtrusionCoordinate=-2.;out.MaximumExtrusionCoordinate=2.;
 for(int32 v=0;v<6;++v)out.SectionControlPoints[v]=FVector3d(0,-1.+2.*double(v)/5.,3.-4.*double(v)/5.+double(v*(v-1))/5.);
 return out.BuildQueryApproximationBounded(.00099,StaticBodyTubes::MaxPerProvider);
}
static bool Scene(const FIntervalTensorSurface& surface,FIndexedIntervalUnionPtr& out)
{FDistanceBudget b{100000};return FIndexedIntervalUnion::Build(MakeArrayView(&surface,1),b,out);}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStaticLegacyBodyTubesOriginalFamilyTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.StaticLegacyBodyTubesOriginalFamily",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStaticLegacyBodyTubesOriginalFamilyTest::RunTest(const FString&)
{
 using namespace Speed;using namespace Analytic;using namespace Certified;namespace T=StaticBodyTubesTests;namespace H=StaticBodyTubes;namespace A=::AnalyticCertified;
 const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("RNE tube preparation"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.StaticLegacyBodyTubes"),1,FName(TEXT("TubeOriginalEnabled")));
 const uint32 saved=VectorGetControlRegister();int32 cases=0,points=0;
 for(uint32 flush:{0u,64u,32768u,32832u})for(bool transpose:{false,true})for(bool reverse:{false,true})
 {
  VectorSetControlRegister((saved&~uint32(0x807f))|flush);
  const auto source=T::Surface(transpose,reverse,1./1099511627776.);FExtrudedQuinticPatch patch;
  if(!TestTrue(TEXT("actual bounded Legacy builder"),T::Patch(patch)&&patch.SectionParameters.Num()>2))return false;
  H::FInput input{0,&patch,!transpose};H::FEntry entry;FDistanceBudget setup{1000000,11,17};
  if(!TestTrue(TEXT("universal original-family refit, finite account"),H::BuildEntry(source,input,setup,entry)&&
    entry.Facets.Num()==patch.SectionParameters.Num()-1&&!entry.Nodes.IsEmpty()&&setup.Used>11&&setup.ConsumptionFloor==17))return false;
  for(int32 segment=0;segment<entry.Facets.Num();++segment)for(double free:{0.,.5,1.})for(double alpha:{0.,.5,1.})
  {
   const double v=patch.SectionParameters[segment]+alpha*(patch.SectionParameters[segment+1]-patch.SectionParameters[segment]);
   FIntervalPoint point;const FVector2d uv=transpose?FVector2d(v,free):FVector2d(free,v);
   if(!TestTrue(TEXT("sampled entire CP family stays within its certified tube, both UV layouts"),
      ProjectedDistanceDetail::EvaluateFixedUV(source,uv,point)&&GroupedBoxDetail::Within(entry.Facets[segment].Hull,point)))return false;
   ++points;
  }
  auto changed=source;changed.ControlPoints[14].Axis[2]=A::Add(changed.ControlPoints[14].Axis[2],A::Interval::Exact(.1));
  H::FEntry denied;FDistanceBudget changedWork{1000000};
  if(!TestTrue(TEXT("changed original coefficient family cannot borrow prior representation bound"),!H::BuildEntry(changed,input,changedWork,denied)&&denied.Nodes.IsEmpty()))return false;
  auto moved=patch;moved.SectionFacetGeometry[0].Origin.Z+=.01;H::FInput foreign{0,&moved,!transpose};FDistanceBudget movedWork{1000000};
  if(!TestTrue(TEXT("actual stored rectangle mutation refuses independently of nominal chord error"),!H::BuildEntry(source,foreign,movedWork,denied)&&denied.Nodes.IsEmpty()))return false;
  auto bogus=patch;for(auto& bounds:bogus.SectionSegmentBounds)bounds=FBox3d(FVector3d(1000),FVector3d(1001));
  for(auto& node:bogus.SectionSegmentBvhNodes)node.Bounds=FBox3d(FVector3d(1000),FVector3d(1001));
  H::FInput proposed{0,&bogus,!transpose};H::FEntry refitted;FDistanceBudget refitWork{1000000};
  if(!TestTrue(TEXT("old AABB values are ignored; only topology proposes the new immutable refit"),H::BuildEntry(source,proposed,refitWork,refitted)&&
     GroupedBoxDetail::SamePoint(entry.Nodes[0].Hull,refitted.Nodes[0].Hull)))return false;
  bogus.SectionSegmentBvhIndices[1]=bogus.SectionSegmentBvhIndices[0];FDistanceBudget duplicateWork{1000000};
  if(!TestTrue(TEXT("incomplete or duplicated facet coverage never publishes"),!H::BuildEntry(source,proposed,duplicateWork,denied)&&denied.Nodes.IsEmpty()))return false;
  FIndexedIntervalUnionPtr scene;if(!T::Scene(source,scene))return false;
  FDistanceBudget prepare{1000000,13,17};const uint64 retained=scene->PrepareStaticBodyTubesAtPublication(MakeArrayView(&input,1),prepare,4*1024*1024);
  const auto owner=scene->GetStaticBodyTubes();FDistanceBudget again{1000000,29,17};
  if(!TestTrue(TEXT("published derived geometry is owned once, no retry work"),owner&&retained>0&&retained<4*1024*1024&&
     scene->PrepareStaticBodyTubesAtPublication(MakeArrayView(&input,1),again,4*1024*1024)==0&&again.Used==29&&scene->GetStaticBodyTubes()==owner))return false;
  // Destroy every proposed facet/topology array. Query never borrows them.
  patch={};FGroupedBoxVolumeBounds volume;if(!StaticBodyHierarchyTests::Volume(1.75,volume))return false;
  FDistanceBudget query{100000,7,17};if(!TestTrue(TEXT("deep owned refit survives producer scratch destruction"),H::TryClear(*scene,0,volume,16383,query)==H::EResult::Clear))return false;
  ++cases;
 }
 TestTrue(TEXT("sixteen real mode/layout/orientation preparations, nonempty samples"),cases==16&&points>144);
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStaticLegacyBodyTubesFineProofTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.StaticLegacyBodyTubesFineProofAndBudget",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStaticLegacyBodyTubesFineProofTest::RunTest(const FString&)
{
 using namespace Speed;using namespace Analytic;using namespace Certified;namespace T=StaticBodyTubesTests;namespace H=StaticBodyTubes;namespace A=::AnalyticCertified;
 const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("RNE full grouped tube proof"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.StaticLegacyBodyTubes"),1,FName(TEXT("TubeFineEnabled"))),
  atlasOff(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy"),0,FName(TEXT("TubeFineHistoricalAtlasOff")));
 const uint32 saved=VectorGetControlRegister();int32 clears=0,collisions=0;
 for(uint32 flush:{0u,64u,32768u,32832u})
 {
  VectorSetControlRegister((saved&~uint32(0x807f))|flush);
  const auto source=T::Surface();FExtrudedQuinticPatch patch;if(!T::Patch(patch))return false;
  H::FInput input{0,&patch,true};FIndexedIntervalUnionPtr scene;if(!T::Scene(source,scene))return false;
  FDistanceBudget prepare{1000000};if(!scene->PrepareStaticBodyTubesAtPublication(MakeArrayView(&input,1),prepare,4*1024*1024))return false;
  for(double z:{1.75,2.})
  {
   FGroupedBoxVolumeBounds volume;if(!StaticBodyHierarchyTests::Volume(z,volume,true))return false;
   FReferenceBoundsCandidateSetPtr partition;FDistanceBudget selection{10000};
   if(!FReferenceBoundsCandidateSet::Select(scene,volume.BroadHull,selection,partition)||partition->GetCandidateIndices().Num()!=1)return false;
   FDistanceBudget cold{100000,11,17},warm=cold;FReferenceBoxVolumeStrictSeparationPtr oldProof,newProof;
   FGroupedBoxSeparationObservation oldObs,newObs;bool before=false;
   {FScopedCVar<int32> off(TEXT("p.IAmSpeed.AnalyticWorld.StaticLegacyBodyTubes"),0,FName(TEXT("TubeFineOracleOff")));
    before=FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,cold,oldProof,&oldObs,false,false,false,2,false);}
   const bool after=FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,warm,newProof,&newObs,false,false,false,2,false);
   if(!TestTrue(TEXT("real fine14 original-scene proof and original endpoint consumer remain authoritative"),before==after&&after==(z==1.75)&&
      warm.ConsumptionFloor==17&&warm.Remaining>=17&&(after?(newProof->Matches(scene,volume)&&newProof->ContainsCompleteEndpointGroup(volume.Groups.Last().Corners)):!newProof)))return false;
   if(after)
   {if(!TestTrue(TEXT("actual integration clears root before generic subdivisions"),newObs.VisitedCells==1&&oldObs.VisitedCells>1))return false;++clears;}
   else ++collisions;
   FDistanceBudget shortWork{H::QueryWork+21+17,19,17};const auto shortBefore=shortWork;
   if(!TestTrue(TEXT("requires whole local allowance plus original initial continuation"),H::TryClear(*scene,0,volume,16383,shortWork)==H::EResult::Unavailable&&
       shortWork.Used==shortBefore.Used&&shortWork.Remaining==shortBefore.Remaining))return false;
   FDistanceBudget overflow{100000,MAX_uint64-100,17};
   if(!TestTrue(TEXT("Used overflow cannot borrow or refund continuation"),H::TryClear(*scene,0,volume,16383,overflow)==H::EResult::Unavailable&&overflow.Used==MAX_uint64-100))return false;
   FDistanceBudget paid{100000,3,17};const auto result=H::TryClear(*scene,0,volume,16383,paid);
   if(!TestTrue(TEXT("clear versus near differs, every attempt remains paid in the sole account"),result==(z==1.75?H::EResult::Clear:H::EResult::Unresolved)&&paid.Used>3&&paid.Remaining>=17&&paid.ConsumptionFloor==17))return false;
  }
  FGroupedBoxVolumeBounds volume;if(!StaticBodyHierarchyTests::Volume(1.75,volume))return false;
  const auto state=VectorGetControlRegister();VectorSetControlRegister(state^64u);FDistanceBudget modeWork{100000,3,17};
  if(!TestTrue(TEXT("different numerical control cannot use initialization geometry"),H::TryClear(*scene,0,volume,16383,modeWork)==H::EResult::Unavailable&&modeWork.Used==3))return false;
  VectorSetControlRegister((state&~uint32(0x6000))|0x2000);FDistanceBudget invalid{100000,3,17};
  if(!TestTrue(TEXT("unsupported rounding remains invalid, never clearance"),H::TryClear(*scene,0,volume,16383,invalid)==H::EResult::Invalid&&invalid.Used==3))return false;
  VectorSetControlRegister(state);
 }
 TestTrue(TEXT("four real strict clears and four real intersections"),clears==4&&collisions==4);
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStaticLegacyBodyTubeLocalRefineTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.StaticLegacyBodyTubeLocalRefine",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStaticLegacyBodyTubeLocalRefineTest::RunTest(const FString&)
{
 using namespace Speed;using namespace Analytic;using namespace Certified;
 const auto original=CaptureFrameFloatingPointContext();
 FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("RNE tube refinement"),original.bValid&&restore.IsApplied()&&
   std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 StaticBodyTubes::FFacet facet;
 facet.Corners[0]=Detail::Point(FVector3d(-1,-1,0));
 facet.Corners[1]=Detail::Point(FVector3d(-1,-1,.1));
 facet.Corners[2]=Detail::Point(FVector3d(1,1,0));
 facet.Corners[3]=Detail::Point(FVector3d(1,1,.1));
 const auto prove=[&](double y,int32 allowance)
 {
  FDistanceBudget budget{allowance,0,0};
  const auto separate=[&](const FIntervalPoint* corners,int32 count,const FIntervalPoint&,uint32& groups)
  {
   if(!budget.Consume())return false;
   for(int32 d=0;d<2;++d)
   {
    FVector3d axis=FVector3d::ZeroVector;axis[d]=1;
    const auto range=GroupedBoxDetail::ProjectHull(MakeArrayView(corners,count),axis);
    if(!::AnalyticCertified::Valid(range))return false;
    const double center=d?y:-.75;
    if(range.Upper<center-.1||range.Lower>center+.1){groups=0;break;}
   }
   return true;
  };
  uint32 whole=1;
  if(!separate(facet.Corners,4,facet.Hull,whole)||whole!=1)return false;
  return StaticBodyTubes::RefineFacet(facet,whole,budget,separate);
 };
 TestTrue(TEXT("both children separated when whole rectangle slabs overlap"),prove(.75,256));
 TestFalse(TEXT("intersecting child cannot be skipped"),prove(-.75,256));
 TestFalse(TEXT("budget exhaustion cannot publish a clear"),prove(.75,3));
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStaticLegacyBodyTubeDirectionalTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.StaticLegacyBodyTubeDirectional",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStaticLegacyBodyTubeDirectionalTest::RunTest(const FString&)
{
 using namespace Speed;using namespace Analytic;using namespace Certified;
 namespace T=StaticBodyTubesTests;
 const auto original=CaptureFrameFloatingPointContext();
 FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("RNE directional tubes"),original.bValid&&restore.IsApplied()&&
   std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 FScopedCVar<int32> tubes(TEXT("p.IAmSpeed.AnalyticWorld.StaticLegacyBodyTubes"),1,FName(TEXT("TestTubeDirectionalSource")));
 FScopedCVar<int32> direction(TEXT("p.IAmSpeed.AnalyticWorld.StaticLegacyBodyTubeDirectional"),1,FName(TEXT("TestTubeDirectional")));
 const auto surface=T::Surface();FExtrudedQuinticPatch patch;FIndexedIntervalUnionPtr scene;
 if(!T::Patch(patch)||!T::Scene(surface,scene))return false;
 StaticBodyTubes::FInput input{0,&patch,true};
 FDistanceBudget preparation{1000000};
 if(!scene->PrepareStaticBodyTubesAtPublication(MakeArrayView(&input,1),preparation,4*1024*1024))return false;
 const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
 FIntervalPoint broad;for(auto& a:broad.Axis)a={-5.,5.,true};
 const auto prove=[&](double z,int32 allowance)
 {
  FDistanceBudget work{allowance,0,0};
  const auto support=[&](const FVector3d& axis,FDistanceBudget& account,Interval& slab)
  {
   if(!account.Consume())return false;
   const double c=axis.Z!=0?z:0.;slab={c-.1,c+.1,true};return true;
  };
  FDirectionalRegionBuilder region(broad,support,numeric.ControlFingerprint(),nullptr);
  return StaticBodyTubes::TryClearDirectional(*scene,0,region,FVector3d(0,0,1),
    FVector3d(0,1,0),numeric.ControlFingerprint(),work);
 };
 TestTrue(TEXT("full source tubes separate from a region with an overlapping broad box"),prove(1.75,1000));
 TestFalse(TEXT("one near tube blocks a coarse clear"),prove(2.,1000));
 TestFalse(TEXT("insufficient work blocks a coarse clear"),prove(1.75,3));
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStaticLegacyBodyTubeStationaryTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.StaticLegacyBodyTubeStationary",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStaticLegacyBodyTubeStationaryTest::RunTest(const FString&)
{
 using namespace Speed;using namespace Analytic;using namespace Certified;
 namespace T=StaticBodyTubesTests;
 const auto original=CaptureFrameFloatingPointContext();
 FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("RNE stationary tubes"),original.bValid&&restore.IsApplied()&&
   std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 FScopedCVar<int32> tubes(TEXT("p.IAmSpeed.AnalyticWorld.StaticLegacyBodyTubes"),1,FName(TEXT("TestTubeStationarySource")));
 FScopedCVar<int32> station(TEXT("p.IAmSpeed.AnalyticWorld.StaticLegacyBodyTubeStationary"),1,FName(TEXT("TestTubeStationary")));
 const auto surface=T::Surface();FExtrudedQuinticPatch patch;FIndexedIntervalUnionPtr scene;
 if(!T::Patch(patch)||!T::Scene(surface,scene))return false;
 StaticBodyTubes::FInput input{0,&patch,true};
 FDistanceBudget preparation{1000000};
 if(!scene->PrepareStaticBodyTubesAtPublication(MakeArrayView(&input,1),preparation,4*1024*1024))return false;
 for(double z:{1.75,2.})
 {
  FGroupedBoxVolumeBounds volume;if(!StaticBodyHierarchyTests::Volume(z,volume,false,1))return false;
  FDistanceBudget selection{10000};
  FReferenceBoundsCandidateSetPtr partition;
  if(!FReferenceBoundsCandidateSet::Select(scene,volume.BroadHull,selection,partition)||
    partition->GetCandidateIndices().Num()!=1)return false;
  FDistanceBudget off{100000,0,0},on=off;FReferenceBoxVolumeStrictSeparationPtr proof;
  FGroupedBoxSeparationObservation previous,observed;bool oldClear;
  {
   FScopedCVar<int32> disable(TEXT("p.IAmSpeed.AnalyticWorld.StaticLegacyBodyTubeStationary"),0,FName(TEXT("TestTubeStationaryOracle")));
   oldClear=FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,off,proof,&previous,false,false,false,2,false);
  }
  const bool newClear=FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,on,proof,&observed,false,false,false,2,false);
  if(!TestTrue(TEXT("actual grouped stationary proof agrees on separation and possible collision"),
   newClear==oldClear&&newClear==(z==1.75)))return false;
  if(newClear&&!TestTrue(TEXT("stationary tube discharges the original provider at root"),
   proof&&proof->Matches(scene,volume)&&observed.VisitedCells==1&&previous.VisitedCells>1))return false;
 }
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStaticLegacyBodyTubeLocalAxisTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.StaticLegacyBodyTubeLocalAxis",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStaticLegacyBodyTubeLocalAxisTest::RunTest(const FString&)
{
 using namespace Speed;using namespace Analytic;using namespace Certified;
 const auto original=CaptureFrameFloatingPointContext();
 FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("RNE local axis"),original.bValid&&restore.IsApplied()&&
   std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 const auto axis=StaticBodyTubes::PrepareLocalAxis(FVector3d(0,1,1),FVector3d(1,0,0));
 const auto reverse=StaticBodyTubes::PrepareLocalAxis(FVector3d(0,-1,-1),FVector3d(1,0,0));
 TestTrue(TEXT("equivalent opposite facets share a finite witness direction"),
  axis==FVector3d(0,1,-1)&&reverse==axis&&GroupedBoxDetail::ValidAxis(axis));
 const FIntervalPoint surface[4]={
  Detail::Point(FVector3d(-1,-1,2)),Detail::Point(FVector3d(-1,1,4)),
  Detail::Point(FVector3d(1,-1,2)),Detail::Point(FVector3d(1,1,4))};
 const auto projection=GroupedBoxDetail::ProjectHull(MakeArrayView(surface),axis);
 FIntervalPoint box=Detail::Point(FVector3d(0,.75,2.25));
 for(auto& d:box.Axis){d.Lower-=.1;d.Upper+=.1;}
 const auto coordinateClear=[&](const FVector3d& direction)
 {
  const auto a=GroupedBoxDetail::ProjectHull(MakeArrayView(surface),direction);
  const auto b=GroupedBoxDetail::Project(box,direction);
  return a.Upper<b.Lower||b.Upper<a.Lower;
 };
 TestFalse(TEXT("coordinate bounds overlap on Y"),coordinateClear(FVector3d(0,1,0)));
 TestFalse(TEXT("coordinate bounds overlap on Z"),coordinateClear(FVector3d(0,0,1)));
 const auto body=GroupedBoxDetail::Project(box,axis);
 TestTrue(TEXT("the complete facet hull and complete box separate on the new direction"),
  projection.Upper<body.Lower);
 box=Detail::Point(FVector3d(0,.75,3.75));
 for(auto& d:box.Axis){d.Lower-=.1;d.Upper+=.1;}
 TestFalse(TEXT("no separation on an intersecting box"),coordinateClear(axis));
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStaticLegacyBodyTubeFrontierTest,
 "IAmSpeed.AnalyticWorld.GroupedBox.StaticLegacyBodyTubeFrontier",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStaticLegacyBodyTubeFrontierTest::RunTest(const FString&)
{
 using namespace Speed;using namespace Analytic;using namespace Certified;
 namespace T=StaticBodyTubesTests;
 const auto original=CaptureFrameFloatingPointContext();
 FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("RNE original source frontier"),original.bValid&&restore.IsApplied()&&
  std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 FScopedCVar<int32> tubes(TEXT("p.IAmSpeed.AnalyticWorld.StaticLegacyBodyTubes"),1,FName(TEXT("TestTubeFrontierSource")));
 FScopedCVar<int32> frontier(TEXT("p.IAmSpeed.AnalyticWorld.StaticLegacyBodyTubeFrontier"),1,FName(TEXT("TestTubeFrontier")));
 const auto surface=T::Surface();FExtrudedQuinticPatch patch;FIndexedIntervalUnionPtr scene;
 if(!T::Patch(patch)||!T::Scene(surface,scene))return false;
 StaticBodyTubes::FInput input{0,&patch,true};
 FDistanceBudget preparation{1000000};
 if(!scene->PrepareStaticBodyTubesAtPublication(MakeArrayView(&input,1),preparation,4*1024*1024))return false;
 FGroupedBoxVolumeBounds volume;if(!StaticBodyHierarchyTests::Volume(2.,volume,true))return false;
 StaticBodyTubes::FTubeLeaves pending;FDistanceBudget work{100000,0,0};
 if(!TestTrue(TEXT("a true possible contact is retained as original restricted source work"),
  StaticBodyTubes::TryBuildFrontier(*scene,0,volume,16383,work,pending)==StaticBodyHierarchy::EFrontier::Ready
  &&!pending.IsEmpty()&&pending.Num()<patch.SectionFacetGeometry.Num()))return false;
 bool coversContact=false;
 for(const auto& leaf:pending)
 {
  const auto& cell=leaf.Cell;
  if(!TestTrue(TEXT("retains original reduced domains and active group, never a facet hit"),
    cell.Net.Num()==12&&cell.DegreeU==1&&cell.DegreeV==5&&cell.DepthV>0&&leaf.ActiveGroups==8192))return false;
  coversContact|=cell.MinV<=.5&&cell.MaxV>=.5;
 }
 TestTrue(TEXT("candidate domains still contain the exact contact region"),coversContact);
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
#endif
