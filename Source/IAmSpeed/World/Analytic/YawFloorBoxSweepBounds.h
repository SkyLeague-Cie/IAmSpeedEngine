#pragma once
#include "FloorNativeSolidSweepBounds.h"
#include <atomic>
namespace Speed::Analytic::Certified
{
// Whole-interval GEOMETRY only. No yaw arrival, overlap, established support,
// native contact, event, resolve or advance capability is produced here.
struct FYawFloorBoxSweepBounds
{
    FIntervalPoint QueryEnvelope;
    Interval ConsumedGap,ReferenceGap;
    bool WholeFloorSeparated=false;
};
namespace YawFloorBoxDetail
{
using namespace AnalyticCertified;
// All bounds below are for the guarded Win64 binary64 RNE/SSE/AVX DAG,
// including permitted FTZ/DAZ and an unfused overbound of fused operations.
// Error compares each computed node with its exact polynomial in raw leaves.
// DAZ allowances are applied at EACH operand before amplification; a single
// smallest-normal allowance on the final coordinate would be insufficient.
struct FNode {Interval Range;double Error=0;};
struct FPoint {FNode Axis[3];};
struct FQuaternion {FNode Axis[4];};
inline double Tiny(){return FromBits(MinNormal);}
inline double MagnitudeOf(Interval a)
{return Valid(a)?FMath::Max(FMath::Abs(a.Lower),FMath::Abs(a.Upper)):-1.;}
inline bool ValidNode(const FNode& a)
{return Valid(a.Range)&&Valid(Interval::Exact(a.Error))&&!(Bits(a.Error)&Sign);}
inline FNode Leaf(double value){return {Interval::Exact(value),0};}
inline Interval ErrorValue(double value){return Interval::Exact(value);}
inline Interval InputError(const FNode& a)
{return Add(ErrorValue(a.Error),Interval::Exact(Tiny()));}
inline Interval RoundError(Interval range)
{
    if(!Valid(range))return {};
    return Add(Multiply(Interval::Exact(MagnitudeOf(range)),
        Interval::Exact(std::numeric_limits<double>::epsilon())),Interval::Exact(Tiny()));
}
namespace NodeErrorUpper
{
// Only the mode is shared. Changing it does not change the mathematical
// enclosure or failure classification of any FNode operation.
IAMSPEED_API extern std::atomic<bool> Enabled;
inline bool RuntimeEnabled(){return Enabled.load(std::memory_order_relaxed);}
struct FUpper {double Value=0.;bool Known=false;};
inline bool ValidUpper(FUpper value)
{
    const auto bits=Bits(value.Value),magnitude=bits&Magnitude;
    return value.Known&&!(bits&Sign)&&magnitude<Infinity&&
        (magnitude==0||magnitude>=MinNormal);
}
inline FUpper ExactUpper(double value)
{
    if(Bits(value)&Sign)return {};
    const auto exact=Interval::Exact(value);
    return Valid(exact)?FUpper{exact.Upper,true}:FUpper{};
}
inline FUpper AddUpper(FUpper a,FUpper b)
{
    if(!ValidUpper(a)||!ValidUpper(b))return {};
    volatile double value=a.Value+b.Value;
    FUpper result;result.Known=Outward(value,true,result.Value);return result;
}
inline FUpper MultiplyUpper(FUpper a,FUpper b)
{
    if(!ValidUpper(a)||!ValidUpper(b))return {};
    volatile double value=a.Value*b.Value;
    // Match Multiply's conservative refusal of MaxFinite, not merely Inf.
    if((Bits(value)&Magnitude)>=0x7fefffffffffffffULL)return {};
    FUpper result;result.Known=Outward(value,true,result.Value);return result;
}
inline FUpper Input(const FNode& value)
{
    // A positive subnormal Error is accepted by ValidNode. Exact expands its
    // upper endpoint to MinNormal BEFORE DAZ can erase the original value.
    return AddUpper(ExactUpper(value.Error),ExactUpper(Tiny()));
}
inline FUpper Round(Interval range)
{
    if(!Valid(range))return {};
    return AddUpper(MultiplyUpper(ExactUpper(MagnitudeOf(range)),
        ExactUpper(std::numeric_limits<double>::epsilon())),ExactUpper(Tiny()));
}
inline FUpper SumError(const FNode& a,const FNode& b,Interval range)
{return AddUpper(AddUpper(Input(a),Input(b)),Round(range));}
inline FUpper ProductError(const FNode& a,const FNode& b,Interval range)
{
    const auto ea=Input(a),eb=Input(b);
    return AddUpper(AddUpper(AddUpper(
        MultiplyUpper(ExactUpper(MagnitudeOf(a.Range)),eb),
        MultiplyUpper(ExactUpper(MagnitudeOf(b.Range)),ea)),MultiplyUpper(ea,eb)),Round(range));
}
// The error-input intervals are nonnegative. Product rounding can create a
// Lower=-MinNormal, but every propagated error interval satisfies -Upper<=Lower.
// Thus a valid nonnegative Upper also excludes a hidden lower-only overflow.
// Keep every original outward stage and parenthesis. Only .Upper is consumed.
// Omitted lower arithmetic may raise different masked sticky flags; none are
// manufactured, cleared or replayed. Live RNE/FTZ/DAZ guards remain at callers.
}

inline FNode Sum(const FNode& a,const FNode& b)
{
    if(!ValidNode(a)||!ValidNode(b))return {};
    FNode out;out.Range=Add(a.Range,b.Range);
    if(NodeErrorUpper::RuntimeEnabled())
    {
        const auto error=NodeErrorUpper::SumError(a,b,out.Range);
        if(!NodeErrorUpper::ValidUpper(error))return {};out.Error=error.Value;return out;
    }
    const auto error=Add(Add(InputError(a),InputError(b)),RoundError(out.Range));
    if(!Valid(error))return {};out.Error=error.Upper;return out;
}
inline FNode Negative(FNode a)
{if(!ValidNode(a))return {};a.Range=Negate(a.Range);return a;}
inline FNode Difference(const FNode& a,const FNode& b){return Sum(a,Negative(b));}
inline FNode Product(const FNode& a,const FNode& b)
{
    if(!ValidNode(a)||!ValidNode(b))return {};
    FNode out;out.Range=Multiply(a.Range,b.Range);
    if(NodeErrorUpper::RuntimeEnabled())
    {
        const auto error=NodeErrorUpper::ProductError(a,b,out.Range);
        if(!NodeErrorUpper::ValidUpper(error))return {};out.Error=error.Value;return out;
    }
    const auto ea=InputError(a),eb=InputError(b);
    // |a'b'-a*b| <= |a'| eb + |b'| ea + ea eb, where a'/b' include DAZ.
    const auto error=Add(Add(Add(
        Multiply(Interval::Exact(MagnitudeOf(a.Range)),eb),
        Multiply(Interval::Exact(MagnitudeOf(b.Range)),ea)),Multiply(ea,eb)),RoundError(out.Range));
    if(!Valid(error))return {};out.Error=error.Upper;return out;
}
inline FPoint Point(const FVector& p)
{FPoint out;for(int32 d=0;d<3;++d)out.Axis[d]=Leaf(p[d]);return out;}
inline FQuaternion Quaternion(const FQuat& q)
{FQuaternion out;out.Axis[0]=Leaf(q.X);out.Axis[1]=Leaf(q.Y);out.Axis[2]=Leaf(q.Z);out.Axis[3]=Leaf(q.W);return out;}
inline FQuaternion YawFamily()
{
    FQuaternion out;out.Axis[0]=out.Axis[1]=Leaf(0);
    out.Axis[2]=out.Axis[3]={{-1.5,1.5,true},0};return out;
}
inline FQuaternion Hamilton(const FQuaternion& a,const FQuaternion& b)
{
    // VectorQuaternionMultiply2 double: W product followed by X/Y/Z
    // signed multiply-adds. The separate products/additions also enclose FMA.
    FQuaternion out;
    out.Axis[0]=Difference(Sum(Sum(Product(a.Axis[3],b.Axis[0]),Product(a.Axis[0],b.Axis[3])),
        Product(a.Axis[1],b.Axis[2])),Product(a.Axis[2],b.Axis[1]));
    out.Axis[1]=Sum(Sum(Difference(Product(a.Axis[3],b.Axis[1]),Product(a.Axis[0],b.Axis[2])),
        Product(a.Axis[1],b.Axis[3])),Product(a.Axis[2],b.Axis[0]));
    out.Axis[2]=Sum(Difference(Sum(Product(a.Axis[3],b.Axis[2]),Product(a.Axis[0],b.Axis[1])),
        Product(a.Axis[1],b.Axis[0])),Product(a.Axis[2],b.Axis[3]));
    out.Axis[3]=Difference(Difference(Difference(Product(a.Axis[3],b.Axis[3]),Product(a.Axis[0],b.Axis[0])),
        Product(a.Axis[1],b.Axis[1])),Product(a.Axis[2],b.Axis[2]));
    return out;
}
inline FPoint Cross(const FPoint& a,const FPoint& b)
{
    FPoint out;for(int32 d=0;d<3;++d)out.Axis[d]=Difference(
        Product(a.Axis[(d+1)%3],b.Axis[(d+2)%3]),Product(a.Axis[(d+2)%3],b.Axis[(d+1)%3]));return out;
}
inline FPoint Rotate(const FQuaternion& q,const FVector& point)
{
    const auto p=Point(point);FPoint v;for(int32 d=0;d<3;++d)v.Axis[d]=q.Axis[d];
    auto t=Cross(v,p);for(int32 d=0;d<3;++d)t.Axis[d]=Product(Leaf(2),t.Axis[d]);
    const auto c=Cross(v,t);FPoint out;
    for(int32 d=0;d<3;++d)out.Axis[d]=Sum(Sum(p.Axis[d],Product(q.Axis[3],t.Axis[d])),c.Axis[d]);return out;
}
inline bool ValidQuaternion(const FQuaternion& q)
{for(int32 d=0;d<4;++d)if(!ValidNode(q.Axis[d]))return false;return true;}
inline FIntervalPoint Ranges(const FPoint& p)
{FIntervalPoint out;for(int32 d=0;d<3;++d){if(!ValidNode(p.Axis[d]))return {};out.Axis[d]=Add(p.Axis[d].Range,Interval{-p.Axis[d].Error,p.Axis[d].Error,true});}return out;}
inline bool Zero(double x){return !(Bits(x)&Magnitude);}
inline bool FiniteYawIntegrator(const PreparedExecution::FNativeSolidStaticValues& v,FQuaternion* rawOutput=nullptr)
{
    if(rawOutput)*rawOutput={};
#if !PLATFORM_ENABLE_VECTORINTRINSICS
    return false; // No transfer of the double SSE sqrt/divide proof to InvSqrt.
#else
    const auto& q=v.OwnerState.Rotation;
    if(!Zero(q.X)||!Zero(q.Y)||!Zero(v.OwnerState.AngularVelocity.X)||!Zero(v.OwnerState.AngularVelocity.Y)||
        !Zero(v.OwnerState.AngularAcceleration.X)||!Zero(v.OwnerState.AngularAcceleration.Y))return false;
    const auto n=Add(Square(Interval::Exact(q.Z)),Square(Interval::Exact(q.W)));
    if(!Valid(n)||n.Upper>2)return false; // outward proof, not rounded SizeSquared()
    const Interval time{0,double(v.Horizon),true};
    const auto nativeSquaredTime=KinematicCenterDetail::RoundedBinary32(Square(time));
    const auto theta=Add(Multiply(Interval::Exact(v.OwnerState.AngularVelocity.Z),time),
        Multiply(Multiply(Interval::Exact(0.5),Interval::Exact(v.OwnerState.AngularAcceleration.Z)),nativeSquaredTime));
    // Theta.SizeSquared(), including zero X/Y terms, must stay finite. In
    // particular float(t*t) overflow cannot enter 0*Inf on a nominal yaw axis.
    const auto angleSquared=Add(Add(Square(Interval::Exact(0)),Square(Interval::Exact(0))),Square(theta));
    if(!Valid(nativeSquaredTime)||!Valid(theta)||!Valid(angleSquared))return false;
    // Native SBox uses the polynomial below 1.e-8, bounded by 1 there.
    // Above it sqrt(AngleSquared) >= sqrt(1.e-8) to within RNE roundoff;
    // |Sin|<=1 gives |Scale|<10001. This is a mathematical enclosure of
    // existing code, never a collision tolerance or a trig accuracy claim.
    FQuaternion dq;dq.Axis[0]=dq.Axis[1]=Leaf(0);
    dq.Axis[2]={Multiply(theta,Interval{-10001,10001,true}),0};dq.Axis[3]={{-1,1,true},0};
    const auto raw=Hamilton(dq,Quaternion(q));if(!ValidQuaternion(raw))return false;
    if(rawOutput)*rawOutput=raw;
    // Finite DQ*Q0 has exact zero X/Y. UE Normalize uses double Dot4,
    // sqrt/divide, then component multiply. Its normal branch has z²+w²<2
    // with ample RNE/FTZ margin; the small-norm fallback is Identity (n=1),
    // and a finite-input squared-sum overflow yields zero (n=0). Q0 return
    // was bounded above separately. Thus n in [0,2], |z|,|w|<1.5 includes
    // ALL branches without assuming unit norm or exact trig phase.
    return true;
#endif
}
inline FIntervalPoint OwnerYawOffset(const FVector& point)
{
    auto out=Ranges(Rotate(YawFamily(),point));if(!Detail::ValidPoint(out))return {};
    // For finite pure yaw every native Z cross term is signed zero. Z is the
    // raw point.Z or its DAZ zero; Exact already includes both if subnormal.
    out.Axis[2]=Interval::Exact(point.Z);return out;
}
inline FIntervalPoint BodyCorner(const FQuat& local,const FVector& corner,Interval normSquared)
{
    if(!Valid(normSquared)||normSquared.Lower<0||normSquared.Upper>2)return {};
    const auto nodes=Rotate(Hamilton(YawFamily(),Quaternion(local)),corner);
    auto out=Ranges(nodes);if(!Detail::ValidPoint(out))return {};
    // Exact raw-quaternion identity, valid even for a non-unit LocalQ:
    // Rotate(Yaw*LocalQ,u).Z = u.Z + (z²+w²)*(Rotate(LocalQ,u).Z-u.Z).
    // Hamilton/Rotate native roundoff is then added from the actual DAG.
    const auto localPoint=FixedQueryBoundsDetail::Rotate(local,corner);
    const auto exact=Add(Interval::Exact(corner.Z),Multiply(normSquared,
        Subtract(localPoint.Axis[2],Interval::Exact(corner.Z))));
    out.Axis[2]=Add(exact,Interval{-nodes.Axis[2].Error,nodes.Axis[2].Error,true});
    return Detail::ValidPoint(out)?out:FIntervalPoint{};
}
inline FIntervalPoint BodyCorner(const FQuat& local,const FVector& corner)
{return BodyCorner(local,corner,Interval{0,2,true});}
inline FIntervalPoint Center(const PreparedExecution::FNativeSolidStaticValues& v,
    const FZeroOffsetKinematicCenterReference& owner,bool consumed)
{
    FIntervalPoint out;
    if(consumed)out=BoundConsumedCenterBetweenFloatSeconds(owner,0.f,v.Horizon);
    else
    {
        const Interval time{0,double(v.Horizon),true};const auto squared=Square(time);
        for(int32 d=0;d<3;++d)out.Axis[d]=Add(Add(Interval::Exact(v.OwnerState.Location[d]),
            Multiply(Interval::Exact(v.OwnerState.Velocity[d]),time)),
            Multiply(Multiply(Interval::Exact(0.5),Interval::Exact(v.OwnerState.Acceleration[d])),squared));
    }
    const auto com=OwnerYawOffset(v.LocalCOM),offset=OwnerYawOffset(v.LocalOffset);
    for(int32 d=0;d<3;++d)out.Axis[d]=Add(Subtract(out.Axis[d],com.Axis[d]),offset.Axis[d]);
    return Detail::ValidPoint(out)?out:FIntervalPoint{};
}
inline bool Join(FIntervalPoint& hull,const FIntervalPoint& point)
{
    if(!Detail::ValidPoint(point))return false;
    hull=Detail::ValidPoint(hull)?TimeDetail::Hull(hull,point):point;return Detail::ValidPoint(hull);
}
}
inline bool BuildYawFloorBoxSweepBounds(const PreparedExecution::FNativeSolidStaticValues& v,FYawFloorBoxSweepBounds& output)
{
    output={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    using namespace YawFloorBoxDetail;using namespace FloorNativeSweepDetail;
    if(v.Shape!=PreparedExecution::ENativeSolidQueryShape::Box||v.Frozen||v.Prescribed||
        v.Query.Shape!=EQueryShape::Box||v.Query.HalfExtent!=v.QueryHalfExtent||
        v.OwnerState.Location.ContainsNaN()||v.BodyState.Location.ContainsNaN()||
        v.LocalCOM.ContainsNaN()||v.LocalOffset.ContainsNaN()||
        v.OwnerState.Rotation.ContainsNaN()||!v.OwnerState.Rotation.IsNormalized()||
        v.LocalRotation.ContainsNaN()||!v.LocalRotation.IsNormalized()||
        v.BodyState.Rotation.ContainsNaN()||!v.BodyState.Rotation.IsNormalized()||
        !SameQuat(v.BodyState.Rotation,v.OwnerState.Rotation*v.LocalRotation)||
        !SameQuat(v.Query.Rotation,v.BodyState.Rotation)||!SameVector(v.Query.Start,v.BodyState.Location)||
        v.PhysicalHalfExtent.ContainsNaN()||v.PhysicalHalfExtent.GetMin()<=0||
        v.QueryHalfExtent.ContainsNaN()||v.QueryHalfExtent.GetMin()<=0)return false;
    FZeroOffsetKinematicCenterReference owner;
    if(!BuildZeroOffsetKinematicCenterReference(v.OwnerState.Location,v.OwnerState.Velocity,v.OwnerState.Acceleration,
        v.Horizon,FVector::ZeroVector,FVector::ZeroVector,owner)||!FiniteYawIntegrator(v))return false;
    // Body linear PVA generally differs from owner COM PVA for rotating offsets.
    // Only the literal World-call chord uses it; PoseAt uses owner integration.
    const FVector end=v.BodyState.Location+v.BodyState.Velocity*v.Horizon+0.5f*v.BodyState.Acceleration*v.Horizon*v.Horizon;
    const FVector start=(FVector(v.OwnerState.Location)-v.OwnerState.Rotation.RotateVector(v.LocalCOM))+
        v.OwnerState.Rotation.RotateVector(v.LocalOffset);
    if(end.ContainsNaN()||!SameVector(v.Query.End,end)||!SameVector(start,v.BodyState.Location))return false;
    const double z=v.BodyState.Location.Z;if(!Finite(z)||z==0)return false;
    const auto sign=Interval::Exact(z>0?1.:-1.);
    FYawFloorBoxSweepBounds result;
    const auto nativeCenter=Center(v,owner,true),referenceCenter=Center(v,owner,false);
    if(!Detail::ValidPoint(nativeCenter)||!Detail::ValidPoint(referenceCenter))return false;
    for(int32 representation=0;representation<2;++representation)
    {
        const FVector extent=representation?v.QueryHalfExtent:v.PhysicalHalfExtent;
        for(int32 k=0;k<8;++k)
        {
            FVector corner;for(int32 d=0;d<3;++d)corner[d]=(k&(1<<d))?extent[d]:-extent[d];
            const auto rotated=BodyCorner(v.LocalRotation,corner);if(!Detail::ValidPoint(rotated))return false;
            FIntervalPoint native,reference,captured;
            const auto capturedCorner=FixedQueryBoundsDetail::Rotate(v.BodyState.Rotation,corner);
            for(int32 d=0;d<3;++d)
            {
                native.Axis[d]=Add(nativeCenter.Axis[d],rotated.Axis[d]);
                reference.Axis[d]=Add(referenceCenter.Axis[d],rotated.Axis[d]);
                captured.Axis[d]=Add(Interval::Exact(v.BodyState.Location[d]),capturedCorner.Axis[d]);
            }
            if(!Join(result.QueryEnvelope,native)||!Join(result.QueryEnvelope,reference)||!Join(result.QueryEnvelope,captured))return false;
            if(!representation)
            {
                const auto capturedGap=Multiply(sign,captured.Axis[2]);
                const auto ng=FloorNativeSweepDetail::Minimum(Multiply(sign,native.Axis[2]),capturedGap);
                const auto rg=FloorNativeSweepDetail::Minimum(Multiply(sign,reference.Axis[2]),capturedGap);
                result.ConsumedGap=k?FloorNativeSweepDetail::Minimum(result.ConsumedGap,ng):ng;
                result.ReferenceGap=k?FloorNativeSweepDetail::Minimum(result.ReferenceGap,rg):rg;
            }
        }
    }
    FIntervalPoint chord;if(!BuildFixedWorldQueryVolumeBounds(v.Query,chord)||!Join(result.QueryEnvelope,chord))return false;
    auto capturedQuery=v.Query;capturedQuery.Start=capturedQuery.End=v.BodyState.Location;
    capturedQuery.HalfExtent=v.PhysicalHalfExtent;
    FIntervalPoint captured;if(!BuildFixedWorldQueryVolumeBounds(capturedQuery,captured)||!Join(result.QueryEnvelope,captured))return false;
    auto footprintQuery=v.Query;footprintQuery.Start=footprintQuery.End=FVector3d::ZeroVector;
    FIntervalPoint footprint;if(!BuildFixedWorldQueryVolumeBounds(footprintQuery,footprint))return false;
    for(const auto& center:{nativeCenter,referenceCenter})
    {
        FIntervalPoint moving;for(int32 d=0;d<3;++d)moving.Axis[d]=Add(center.Axis[d],footprint.Axis[d]);
        if(!Join(result.QueryEnvelope,moving))return false;
    }
    for(double shell:{v.Query.DomainTolerance,v.Query.InitialOverlapTolerance})
    {
        const auto bits=Bits(shell);
        if((bits&Magnitude)>=Infinity||((bits&Sign)&&(bits&Magnitude)))return false;
        for(int32 d=0;d<3;++d)result.QueryEnvelope.Axis[d]=Add(result.QueryEnvelope.Axis[d],Interval{-shell,shell,true});
    }
    if(!Detail::ValidPoint(result.QueryEnvelope)||!Valid(result.ConsumedGap)||!Valid(result.ReferenceGap))return false;
    result.WholeFloorSeparated=result.ConsumedGap.Lower>0&&result.ReferenceGap.Lower>0;
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);output=MoveTemp(result);return true;
}
}

// Appended to the existing geometric bounds header. No World/transport grant.
namespace Speed::Analytic::Certified
{
namespace GeneralBoxDetail
{
using namespace AnalyticCertified;
using namespace YawFloorBoxDetail;
inline Interval Hull(Interval a,Interval b)
{
    if(!Valid(a)||!Valid(b))return {};
    return {FMath::Min(a.Lower,b.Lower),FMath::Max(a.Upper,b.Upper),true};
}
inline Interval Expanded(const FNode& a)
{
    if(!ValidNode(a))return {};
    return Add(a.Range,Interval{-a.Error,a.Error,true});
}
inline Interval DividePositive(Interval numerator,double denominator)
{
    if(!Valid(numerator)||!Finite(denominator)||denominator<=0)return {};
    volatile double lo=numerator.Lower/denominator,hi=numerator.Upper/denominator;
    Interval result;result.Known=Outward(lo,false,result.Lower)&&Outward(hi,true,result.Upper);return result;
}
inline bool TrigPolynomialBounds(Interval x,Interval& scale,Interval& cosine)
{
    scale={};cosine={};
    // Technical domain of this theorem, never a gameplay tolerance. Do not
    // intersect an unsupported input range with this domain to obtain a pass.
    if(!Valid(x)||x.Lower<0||x.Upper>1)return false;
    const double ds[]={2.,48.,3840.,645120.,185794560.,81749606400.,51011754393600.,42849873690624000.};
    const double dc[]={1.,8.,384.,46080.,10321920.,3715891200.,1961990553600.,1428329123020800.};
    static_assert(uint64(42849873690624000.)==42849873690624000ULL);
    const auto polynomial=[&](const double* denominators)
    {
        Interval p=DividePositive(Interval::Exact(1),denominators[6]);
        for(int32 k=5;k>=0;--k)
        {
            auto coefficient=DividePositive(Interval::Exact(1),denominators[k]);
            if(k&1)coefficient=Negate(coefficient);
            p=Add(Multiply(x,p),coefficient);
        }
        Interval power=Interval::Exact(1);
        for(int32 k=0;k<7;++k)power=Multiply(power,x);
        const auto lower=Subtract(p,DividePositive(power,denominators[7]));
        if(!Valid(p)||!Valid(lower))return Interval{};
        // Alternating-series theorem: P6-x^7/D7 <= f <= P6 on [0,1].
        return Interval{lower.Lower,p.Upper,true};
    };
    scale=polynomial(ds);cosine=polynomial(dc);
    // Preserve the existing yaw theorem's scalar Cos range by CHECKING the
    // same intersection at runtime; normalized quaternion components below
    // are not clipped to unit-component limits.
    if(Valid(cosine))
    {cosine.Lower=FMath::Max(-1.,cosine.Lower);cosine.Upper=FMath::Min(1.,cosine.Upper);}
    // Same point/range evaluator, including the actual native small-angle
    // polynomial. Inclusion monotonicity transfers every checked native
    // result at x to an enclosing xRange; no libm ULP assumption is needed.
    constexpr double nativeScaleThreshold=1.e-8;
    if(x.Lower<nativeScaleThreshold)
    {
        const Interval smallScaleInput{x.Lower,FMath::Min(x.Upper,nativeScaleThreshold),true};
        const auto p2=Add(Subtract(Interval::Exact(.5),DividePositive(smallScaleInput,48.)),DividePositive(Square(smallScaleInput),3840.));
        scale=Hull(scale,p2);
    }
    return Valid(scale)&&Valid(cosine);
}
inline bool Contains(Interval interval,double value)
{
    return Valid(interval)&&Finite(value)&&value>=interval.Lower&&value<=interval.Upper;
}
inline bool CheckedNativeExpResults(double x,double nativeScale,double nativeCosine)
{
    const auto xBits=Bits(x);
    if(!Finite(x)||((xBits&Sign)&&(xBits&Magnitude))||!Finite(nativeScale)||!Finite(nativeCosine))return false;
    // Other existing motion models retain their own domains. No new general
    // enclosure is admitted for them by this check.
    if(x>1)return nativeScale>=-10001.&&nativeScale<=10001.&&nativeCosine>=-1.&&nativeCosine<=1.;
    Interval scale,cosine;
    return TrigPolynomialBounds(Interval::Exact(x),scale,cosine)&&Contains(scale,nativeScale)&&Contains(cosine,nativeCosine);
}
inline bool NormalizedFamily(const FQuaternion& raw,const FQuat& captured,FQuaternion& out)
{
    out={};
#if !defined(_MSC_VER) || !defined(_M_X64) || defined(_M_ARM64EC) || !PLATFORM_ENABLE_VECTORINTRINSICS
    return false;
#else
    FQuaternion actual;
    for(int32 k=0;k<4;++k)
    {
        const auto value=Expanded(raw.Axis[k]);if(!Valid(value))return false;
        actual.Axis[k]={value,0};
    }
    FNode square[4];for(int32 k=0;k<4;++k)square[k]=Product(actual.Axis[k],actual.Axis[k]);
    // Both real double SIMD Dot4 associations, including the unfused
    // overbound of SSE/FMA. Never substitute a scalar SizeSquared DAG.
    const auto avx=Expanded(Sum(Sum(square[0],square[1]),Sum(square[2],square[3])));
    const auto sse=Expanded(Sum(Sum(square[0],square[2]),Sum(square[1],square[3])));
    const auto squared=Hull(avx,sse);
    // This is the EXISTING float macro promoted to double, distinct from
    // the native Scale branch's double literal 1.e-8.
    if(!Valid(squared)||squared.Lower<=double(UE_SMALL_NUMBER))return false;
    const auto root=[](double value)
    {return _mm_cvtsd_f64(_mm_sqrt_sd(_mm_setzero_pd(),_mm_set_sd(value)));};
    double rootLo=0,rootHi=0;
    if(!Outward(root(squared.Lower),false,rootLo)||!Outward(root(squared.Upper),true,rootHi)||rootLo<=0)return false;
    volatile double invLo=1./rootHi,invHi=1./rootLo;
    Interval inverse;inverse.Known=Outward(invLo,false,inverse.Lower)&&Outward(invHi,true,inverse.Upper);
    if(!Valid(inverse))return false;
    const double q0[]={captured.X,captured.Y,captured.Z,captured.W};
    for(int32 k=0;k<4;++k)
    {
        const auto normalized=Expanded(Product(actual.Axis[k],FNode{inverse,0}));
        // Include the actual Theta.IsZero early return, plus the exact raw
        // reference Exp(theta)*Q0 family as well as its normalized family.
        const auto component=Hull(Hull(normalized,actual.Axis[k].Range),Interval::Exact(q0[k]));
        if(!Valid(component))return false;out.Axis[k]={component,0};
    }
    return true;
#endif
}
inline bool RotationFamily(const FKinematicState& state,float horizon,bool consumed,FQuaternion& out)
{
    out={};const Interval time{0.,double(horizon),true};
    const auto timeSquared=consumed?KinematicCenterDetail::RoundedBinary32(Square(time)):Square(time);
    if(!Valid(timeSquared))return false;
    FNode theta[3];
    for(int32 d=0;d<3;++d)
    {
        const auto node=Sum(Product(Leaf(state.AngularVelocity[d]),FNode{time,0}),
            Product(Product(Leaf(.5),Leaf(state.AngularAcceleration[d])),FNode{timeSquared,0}));
        const auto values=consumed?Expanded(node):node.Range;
        if(!Valid(values))return false;theta[d]={values,0};
    }
    FNode squares[3];for(int32 d=0;d<3;++d)squares[d]=Product(theta[d],theta[d]);
    Interval x;
    if(consumed)
    {
        // All nonnegative square-sum associations; outward DAG errors are
        // retained before joining the possible native AngleSquared values.
        for(int32 first=0;first<3;++first)
        {
            const auto total=Expanded(Sum(Sum(squares[first],squares[(first+1)%3]),squares[(first+2)%3]));
            x=Valid(x)?Hull(x,total):total;
        }
        if(!Valid(x))return false;
        x.Lower=FMath::Max(0.,x.Lower); // native sum of finite squares is nonnegative
    }
    else
    {
        x=Add(Add(Square(theta[0].Range),Square(theta[1].Range)),Square(theta[2].Range));
        if(!Valid(x))return false;x.Lower=FMath::Max(0.,x.Lower);
    }
    Interval scale,cosine;if(!TrigPolynomialBounds(x,scale,cosine))return false;
    FQuaternion delta;
    for(int32 d=0;d<3;++d)delta.Axis[d]=Product(theta[d],FNode{scale,0});
    delta.Axis[3]={cosine,0};
    return NormalizedFamily(Hamilton(delta,Quaternion(state.Rotation)),state.Rotation,out);
}
inline FIntervalPoint PointRanges(const FPoint& p)
{
    FIntervalPoint out;for(int32 d=0;d<3;++d){out.Axis[d]=Expanded(p.Axis[d]);if(!Valid(out.Axis[d]))return {};}
    return out;
}
}
struct FGeneralBoxMotionInput
{
    FKinematicState Owner;
    FVector LocalCOM=FVector::ZeroVector,LocalOffset=FVector::ZeroVector;
    FQuat LocalRotation=FQuat::Identity;
    FVector PhysicalHalfExtent=FVector::ZeroVector,QueryHalfExtent=FVector::ZeroVector;
    float Horizon=0;
};
struct FGeneralBoxMotionBounds
{
    FIntervalPoint Hull,NativeCenter,ReferenceCenter;
    TArray<FIntervalPoint,TInlineAllocator<32>> Corners; // Four COMPLETE groups of eight, before AABB join.
};
inline bool BuildGeneralBoxMotionBounds(const FGeneralBoxMotionInput& v,FGeneralBoxMotionBounds& output)
{
    output={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    using namespace GeneralBoxDetail;using namespace YawFloorBoxDetail;
    const auto& owner=v.Owner;
    if(owner.Location.ContainsNaN()||owner.Velocity.ContainsNaN()||owner.Acceleration.ContainsNaN()||
        owner.AngularVelocity.ContainsNaN()||owner.AngularAcceleration.ContainsNaN()||
        owner.Rotation.ContainsNaN()||!owner.Rotation.IsNormalized()||v.LocalRotation.ContainsNaN()||!v.LocalRotation.IsNormalized()||
        v.LocalCOM.ContainsNaN()||v.LocalOffset.ContainsNaN()||v.PhysicalHalfExtent.ContainsNaN()||v.QueryHalfExtent.ContainsNaN()||
        v.PhysicalHalfExtent.GetMin()<=0||v.QueryHalfExtent.GetMin()<=0)return false;
    FZeroOffsetKinematicCenterReference center;
    if(!BuildZeroOffsetKinematicCenterReference(owner.Location,owner.Velocity,owner.Acceleration,v.Horizon,
        FVector::ZeroVector,FVector::ZeroVector,center))return false;
    FGeneralBoxMotionBounds result;
    for(bool consumed:{true,false})
    {
        FQuaternion ownerRotation;if(!RotationFamily(owner,v.Horizon,consumed,ownerRotation))return false;
        FIntervalPoint ownerCenter;
        if(consumed)ownerCenter=BoundConsumedCenterBetweenFloatSeconds(center,0.f,v.Horizon);
        else
        {
            const Interval time{0.,double(v.Horizon),true};const auto squared=Square(time);
            for(int32 d=0;d<3;++d)ownerCenter.Axis[d]=Add(Add(Interval::Exact(owner.Location[d]),
                Multiply(Interval::Exact(owner.Velocity[d]),time)),
                Multiply(Multiply(Interval::Exact(.5),Interval::Exact(owner.Acceleration[d])),squared));
        }
        if(!Detail::ValidPoint(ownerCenter))return false;
        const auto com=PointRanges(Rotate(ownerRotation,v.LocalCOM)),offset=PointRanges(Rotate(ownerRotation,v.LocalOffset));
        if(!Detail::ValidPoint(com)||!Detail::ValidPoint(offset))return false;
        FIntervalPoint bodyCenter;
        for(int32 d=0;d<3;++d)bodyCenter.Axis[d]=Add(Subtract(ownerCenter.Axis[d],com.Axis[d]),offset.Axis[d]);
        if(!Detail::ValidPoint(bodyCenter))return false;
        if(consumed)result.NativeCenter=bodyCenter;else result.ReferenceCenter=bodyCenter;
        const auto bodyRotation=Hamilton(ownerRotation,Quaternion(v.LocalRotation));
        for(const auto& extent:{v.PhysicalHalfExtent,v.QueryHalfExtent})for(int32 corner=0;corner<8;++corner)
        {
            FVector local;for(int32 d=0;d<3;++d)local[d]=(corner&(1<<d))?extent[d]:-extent[d];
            const auto rotated=PointRanges(Rotate(bodyRotation,local));if(!Detail::ValidPoint(rotated))return false;
            FIntervalPoint point;
            for(int32 d=0;d<3;++d)point.Axis[d]=Add(bodyCenter.Axis[d],rotated.Axis[d]);
            if(!YawFloorBoxDetail::Join(result.Hull,point))return false;
            result.Corners.Add(point);
        }
    }
    if(!Detail::ValidPoint(result.Hull))return false;
    output=MoveTemp(result);return true;
}
inline bool BuildGeneralNativeBoxSweepVolume(const PreparedExecution::FNativeSolidStaticValues& v,FIntervalPoint& output)
{
    output={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    using namespace FloorNativeSweepDetail;
    if(v.Shape!=PreparedExecution::ENativeSolidQueryShape::Box||v.Frozen||v.Prescribed||
        v.Query.Shape!=EQueryShape::Box||v.Query.HalfExtent!=v.QueryHalfExtent||
        !SameQuat(v.BodyState.Rotation,v.OwnerState.Rotation*v.LocalRotation)||!SameQuat(v.Query.Rotation,v.BodyState.Rotation)||
        !SameVector(v.Query.Start,v.BodyState.Location))return false;
    const FVector bodyOrigin=(FVector(v.OwnerState.Location)-v.OwnerState.Rotation.RotateVector(v.LocalCOM))+
        v.OwnerState.Rotation.RotateVector(v.LocalOffset);
    const FVector chordEnd=v.BodyState.Location+v.BodyState.Velocity*v.Horizon+0.5f*v.BodyState.Acceleration*v.Horizon*v.Horizon;
    if(!SameVector(bodyOrigin,v.BodyState.Location)||!SameVector(chordEnd,v.Query.End))return false;
    FGeneralBoxMotionInput motion;motion.Owner=v.OwnerState;motion.LocalCOM=v.LocalCOM;motion.LocalOffset=v.LocalOffset;
    motion.LocalRotation=v.LocalRotation;motion.PhysicalHalfExtent=v.PhysicalHalfExtent;motion.QueryHalfExtent=v.QueryHalfExtent;
    motion.Horizon=v.Horizon;FGeneralBoxMotionBounds bounds;
    if(!BuildGeneralBoxMotionBounds(motion,bounds))return false;
    auto hull=bounds.Hull;
    FIntervalPoint chord;if(!BuildFixedWorldQueryVolumeBounds(v.Query,chord)||!YawFloorBoxDetail::Join(hull,chord))return false;
    auto captured=v.Query;captured.Start=captured.End=v.BodyState.Location;captured.HalfExtent=v.PhysicalHalfExtent;
    FIntervalPoint pose;if(!BuildFixedWorldQueryVolumeBounds(captured,pose)||!YawFloorBoxDetail::Join(hull,pose))return false;
    auto footprintQuery=v.Query;footprintQuery.Start=footprintQuery.End=FVector3d::ZeroVector;
    FIntervalPoint footprint;if(!BuildFixedWorldQueryVolumeBounds(footprintQuery,footprint))return false;
    for(const auto& center:{bounds.NativeCenter,bounds.ReferenceCenter})
    {
        FIntervalPoint point;for(int32 d=0;d<3;++d)point.Axis[d]=AnalyticCertified::Add(center.Axis[d],footprint.Axis[d]);
        if(!YawFloorBoxDetail::Join(hull,point))return false;
    }
    for(double shell:{v.Query.DomainTolerance,v.Query.InitialOverlapTolerance})
    {
        const auto bits=AnalyticCertified::Bits(shell);
        if((bits&AnalyticCertified::Magnitude)>=AnalyticCertified::Infinity||
            ((bits&AnalyticCertified::Sign)&&(bits&AnalyticCertified::Magnitude)))return false;
        for(int32 d=0;d<3;++d)hull.Axis[d]=AnalyticCertified::Add(hull.Axis[d],Interval{-shell,shell,true});
    }
    if(!Detail::ValidPoint(hull))return false;output=hull;return true;
}
}
