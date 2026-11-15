#pragma once
#include "ContinuousCenterPath.h"
namespace Speed::Analytic::Certified
{
namespace KinematicCenterDetail
{
static_assert(sizeof(float)==sizeof(uint32)&&std::numeric_limits<float>::is_iec559,"Requires IEEE binary32");
constexpr uint32 FloatSign=0x80000000u,FloatMagnitude=0x7fffffffu,FloatMinNormal=0x00800000u,FloatInfinity=0x7f800000u;
inline uint32 Bits(float value) { uint32 bits;FMemory::Memcpy(&bits,&value,sizeof(bits));return bits; }
inline float FromBits(uint32 bits) { float value;FMemory::Memcpy(&value,&bits,sizeof(value));return value; }
inline bool FloatOutward(float value,bool upward,double& out)
{
    auto bits=Bits(value);const auto magnitude=bits&FloatMagnitude;
    if(magnitude>=FloatInfinity)return false;
    if(magnitude<FloatMinNormal)
    {
        if(magnitude==0)bits=upward?FloatMinNormal:FloatSign|FloatMinNormal;
        else bits=(bits&FloatSign)?(upward?0u:FloatSign|FloatMinNormal):(upward?FloatMinNormal:0u);
    }
    else
    {
        if(((bits&FloatSign)!=0)==upward)--bits;else ++bits;
        if((bits&FloatMagnitude)>=FloatInfinity)return false;
        if((bits&FloatMagnitude)<FloatMinNormal)bits=(bits&FloatSign)?(upward?0u:FloatSign|FloatMinNormal):(upward?FloatMinNormal:0u);
    }
    out=double(FromBits(bits));return AnalyticCertified::Finite(out);
}
inline Interval RoundedBinary32(Interval input)
{
    if(!AnalyticCertified::Valid(input))return {};
    volatile float lo=static_cast<float>(input.Lower),hi=static_cast<float>(input.Upper);Interval out;
    out.Known=FloatOutward(lo,false,out.Lower)&&FloatOutward(hi,true,out.Upper);return AnalyticCertified::Valid(out)?out:Interval{};
}
inline bool Normalized(Interval time)
{
    return AnalyticCertified::Valid(time)&&time.Lower>=0&&time.Upper<=1;
}
}
// An explicit LOCAL-OFFSET-ZERO model, captured once from fixed input bits.
// ReferenceCurve is continuous mathematics; consumed pose bounds enclose the
// DIFFERENT actual DAG float(t*Delta), float(s*s), then binary64 vector ops.
// Both local body offset and local COM must be EXACTLY zero. Caller must also
// bind captured subbody KS to the consumed COM P/V/A and exclude suspension or
// override paths. No class/default property proves those current instance bits.
// No Integrate Quantize grid, rotation/offset model, or gameplay state is added.
struct FReferenceTimeFloor;
class FZeroOffsetKinematicCenterReference
{
    FVector3d Start=FVector3d::ZeroVector,Velocity=FVector3d::ZeroVector,Acceleration=FVector3d::ZeroVector;
    float DeltaSeconds=0;
    FIntervalBezierCenterPath ReferenceCurve;
    bool Known=false;
    friend bool BuildZeroOffsetKinematicCenterReference(const FVector3d&,const FVector3d&,const FVector3d&,float,const FVector3d&,const FVector3d&,FZeroOffsetKinematicCenterReference&);
    friend FIntervalPoint BoundConsumedCenterAtFloatSeconds(const FZeroOffsetKinematicCenterReference&,float);
    friend FIntervalPoint BoundConsumedCenterRange(const FZeroOffsetKinematicCenterReference&,Interval);
    friend FReferenceTimeFloor FloorReferenceTimeToFloatSeconds(const FZeroOffsetKinematicCenterReference&,double);
    friend FIntervalPoint BoundConsumedCenterBetweenFloatSeconds(const FZeroOffsetKinematicCenterReference&,float,float);
public:
    FZeroOffsetKinematicCenterReference()=default;
    bool IsKnown() const { return Known; }
    const FIntervalBezierCenterPath& GetReferenceCurve() const { return ReferenceCurve; }
};
inline bool BuildZeroOffsetKinematicCenterReference(const FVector3d& start,const FVector3d& velocity,
    const FVector3d& acceleration,float deltaSeconds,const FVector3d& localOffset,const FVector3d& localCenterOfMass,
    FZeroOffsetKinematicCenterReference& output)
{
    output={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    const uint32 db=KinematicCenterDetail::Bits(deltaSeconds),dm=db&KinematicCenterDetail::FloatMagnitude;
    if((db&KinematicCenterDetail::FloatSign)||dm<KinematicCenterDetail::FloatMinNormal||dm>=KinematicCenterDetail::FloatInfinity)return false;
    for(int32 d=0;d<3;++d)if((AnalyticCertified::Bits(localOffset[d])&AnalyticCertified::Magnitude)!=0||
        (AnalyticCertified::Bits(localCenterOfMass[d])&AnalyticCertified::Magnitude)!=0)return false;
    const auto x=Detail::Point(start),v=Detail::Point(velocity),a=Detail::Point(acceleration);
    if(!Detail::ValidPoint(x)||!Detail::ValidPoint(v)||!Detail::ValidPoint(a))return false;
    const auto delta=Interval::Exact(double(deltaSeconds)),delta2=AnalyticCertified::Square(delta);
    FIntervalBezierCenterPath reference;reference.Degree=2;reference.ControlPoints.SetNum(3);reference.ControlPoints[0]=x;
    for(int32 d=0;d<3;++d)
    {
        const auto vDelta=AnalyticCertified::Multiply(v.Axis[d],delta);
        reference.ControlPoints[1].Axis[d]=AnalyticCertified::Add(x.Axis[d],AnalyticCertified::Multiply(Interval::Exact(0.5),vDelta));
        const auto halfAcceleration=AnalyticCertified::Multiply(Interval::Exact(0.5),a.Axis[d]);
        reference.ControlPoints[2].Axis[d]=AnalyticCertified::Add(AnalyticCertified::Add(x.Axis[d],vDelta),AnalyticCertified::Multiply(halfAcceleration,delta2));
    }
    if(!CenterDetail::Valid(reference))return false;
    FZeroOffsetKinematicCenterReference result;result.Start=start;result.Velocity=velocity;result.Acceleration=acceleration;result.DeltaSeconds=deltaSeconds;result.ReferenceCurve=MoveTemp(reference);result.Known=true;output=MoveTemp(result);return true;
}
inline FIntervalPoint BoundConsumedCenterAtFloatSeconds(const FZeroOffsetKinematicCenterReference& model,float seconds)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    const uint32 bits=KinematicCenterDetail::Bits(seconds),magnitude=bits&KinematicCenterDetail::FloatMagnitude;
    if(!model.Known||(bits&KinematicCenterDetail::FloatSign)||(magnitude&&magnitude<KinematicCenterDetail::FloatMinNormal)||magnitude>=KinematicCenterDetail::FloatInfinity||double(seconds)>double(model.DeltaSeconds))return {};
    const auto s=Interval::Exact(double(seconds));
    const auto squared=KinematicCenterDetail::RoundedBinary32(AnalyticCertified::Square(s));
    FIntervalPoint out;
    for(int32 d=0;d<3;++d)
    {
        const auto displacement=AnalyticCertified::Multiply(Interval::Exact(model.Velocity[d]),s);
        const auto acceleration=AnalyticCertified::Multiply(AnalyticCertified::Multiply(Interval::Exact(0.5),Interval::Exact(model.Acceleration[d])),squared);
        out.Axis[d]=AnalyticCertified::Add(AnalyticCertified::Add(Interval::Exact(model.Start[d]),displacement),acceleration);
    }
    return Detail::ValidPoint(out)?out:FIntervalPoint{};
}
// Direct bounds for every actual binary32 Integrate time in [first,last].
// Endpoints are the consumed seconds themselves: no seconds/Delta ratio or
// multiplication back to seconds is allowed to change this range. The square
// still follows the actual rounded binary32 s*s before binary64 vector ops.
// This is a universal enclosure, not a continuous-path/contact existence proof.
inline FIntervalPoint BoundConsumedCenterBetweenFloatSeconds(
    const FZeroOffsetKinematicCenterReference& model,float first,float last)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!model.Known)return {};
    for(float value:{first,last})
    {
        const uint32 bits=KinematicCenterDetail::Bits(value),magnitude=bits&KinematicCenterDetail::FloatMagnitude;
        if((bits&KinematicCenterDetail::FloatSign)||(magnitude&&magnitude<KinematicCenterDetail::FloatMinNormal)||
            magnitude>=KinematicCenterDetail::FloatInfinity)return {};
    }
    if(first>last||last>model.DeltaSeconds)return {};
    const Interval seconds={double(first),double(last),true};
    const auto squared=KinematicCenterDetail::RoundedBinary32(AnalyticCertified::Square(seconds));
    FIntervalPoint out;
    for(int32 d=0;d<3;++d)
        out.Axis[d]=AnalyticCertified::Add(
            AnalyticCertified::Add(Interval::Exact(model.Start[d]),AnalyticCertified::Multiply(Interval::Exact(model.Velocity[d]),seconds)),
            AnalyticCertified::Multiply(AnalyticCertified::Multiply(Interval::Exact(0.5),Interval::Exact(model.Acceleration[d])),squared));
    return Detail::ValidPoint(out)?out:FIntervalPoint{};
}
// Encloses ALL consumed centers in the normalized prefix/range, not only its
// endpoint. A clear endpoint is never a swept-prefix exclusion certificate.
inline FIntervalPoint BoundConsumedCenterRange(const FZeroOffsetKinematicCenterReference& model,Interval normalizedTime)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!model.Known||!KinematicCenterDetail::Normalized(normalizedTime))return {};
    const auto s=KinematicCenterDetail::RoundedBinary32(AnalyticCertified::Multiply(normalizedTime,Interval::Exact(double(model.DeltaSeconds))));
    const auto squared=KinematicCenterDetail::RoundedBinary32(AnalyticCertified::Square(s));
    FIntervalPoint out;
    for(int32 d=0;d<3;++d)
        out.Axis[d]=AnalyticCertified::Add(AnalyticCertified::Add(Interval::Exact(model.Start[d]),AnalyticCertified::Multiply(Interval::Exact(model.Velocity[d]),s)),AnalyticCertified::Multiply(AnalyticCertified::Multiply(Interval::Exact(0.5),Interval::Exact(model.Acceleration[d])),squared));
    return Detail::ValidPoint(out)?out:FIntervalPoint{};
}
struct FReferenceTimeFloor
{
    bool Known=false;float Seconds=0;
    // This only proves Seconds<=referenceLowerTime*Delta. It does NOT prove
    // consumed endpoint or swept prefix clear. Global range exclusion is due.
};
inline FReferenceTimeFloor FloorReferenceTimeToFloatSeconds(const FZeroOffsetKinematicCenterReference& model,double referenceLowerTime)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    const auto bits=AnalyticCertified::Bits(referenceLowerTime);
    if(!model.Known||!AnalyticCertified::Finite(referenceLowerTime)||((bits&AnalyticCertified::Sign)&&(bits&AnalyticCertified::Magnitude))||referenceLowerTime>1)return {};
    if((bits&AnalyticCertified::Magnitude)==0)return {true,0.f};
    const auto product=AnalyticCertified::Multiply(Interval::Exact(referenceLowerTime),Interval::Exact(double(model.DeltaSeconds)));
    if(!AnalyticCertified::Valid(product)||product.Lower<0)return {};
    float seconds=static_cast<float>(product.Lower);uint32 fb=KinematicCenterDetail::Bits(seconds);
    if((fb&KinematicCenterDetail::FloatMagnitude)>=KinematicCenterDetail::FloatInfinity)return {};
    if((fb&KinematicCenterDetail::FloatMagnitude)<KinematicCenterDetail::FloatMinNormal)seconds=0;
    else if(double(seconds)>product.Lower)
    {
        --fb;seconds=(fb<KinematicCenterDetail::FloatMinNormal)?0.f:KinematicCenterDetail::FromBits(fb);
    }
    if(double(seconds)>product.Lower)return {};
    return {true,seconds};
}
}
