#pragma once
#include "YawFloorBoxSweepBounds.h"
// Mathematical bounds on an ORIGINAL captured
// motion at global float seconds. No World query, advance, response or grant.
namespace Speed::Analytic::Certified
{
namespace GeneralBoxGlobalTimeDetail
{
using namespace GeneralBoxDetail;using namespace YawFloorBoxDetail;
inline bool ValidFloatSecondsRange(float first,float last)
{
    for(float value:{first,last})
    {
        const uint32 bits=KinematicCenterDetail::Bits(value),magnitude=bits&KinematicCenterDetail::FloatMagnitude;
        if((bits&KinematicCenterDetail::FloatSign)||(magnitude&&magnitude<KinematicCenterDetail::FloatMinNormal)||
            magnitude>=KinematicCenterDetail::FloatInfinity)return false;
    }
    return first<=last;
}
inline bool NormalizedFamilyOnGlobalRange(const FQuaternion& raw,const FQuat& captured,bool includeInitial,FQuaternion& out)
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
        const auto regular=Hull(normalized,actual.Axis[k].Range);
        const auto component=includeInitial?Hull(regular,Interval::Exact(q0[k])):regular;
        if(!Valid(component))return false;out.Axis[k]={component,0};
    }
    return true;
#endif
}
inline bool RotationFamilyOnGlobalFloatSeconds(const FKinematicState& state,float first,float last,bool consumed,FQuaternion& out,
    const Interval* exactConsumedSquare=nullptr)
{
    out={};if(!ValidFloatSecondsRange(first,last))return false;
    const Interval time{double(first),double(last),true};
    const auto timeSquared=consumed?(exactConsumedSquare?*exactConsumedSquare:
        KinematicCenterDetail::RoundedBinary32(Square(time))):Square(time);
    if(!Valid(timeSquared))return false;
    FNode theta[3];bool initialPossible=true;
    for(int32 d=0;d<3;++d)
    {
        const auto node=Sum(Product(Leaf(state.AngularVelocity[d]),FNode{time,0}),
            Product(Product(Leaf(.5),Leaf(state.AngularAcceleration[d])),FNode{timeSquared,0}));
        const auto values=consumed?Expanded(node):node.Range;
        if(!Valid(values))return false;theta[d]={values,0};
        // Each model retains its own time-square operands. The consumed
        // group encloses the real exact-zero native branch with float t*t;
        // the reference group uses real t*t. Expanded preserves roundoff.
        // A positive-time cancellation must continue to retain Q0.
        const auto zeroRange=Expanded(node);
        if(!Valid(zeroRange))return false;
        initialPossible&=Contains(zeroRange,0.);
    }
    FNode squares[3];for(int32 d=0;d<3;++d)squares[d]=Product(theta[d],theta[d]);
    Interval x;
    if(consumed)
    {
        // All nonnegative square-sum associations; outward DAG errors are
        // retained before joining the possible native AngleSquared values.
        for(int32 association=0;association<3;++association)
        {
            const auto total=Expanded(Sum(Sum(squares[association],squares[(association+1)%3]),squares[(association+2)%3]));
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
    return NormalizedFamilyOnGlobalRange(Hamilton(delta,Quaternion(state.Rotation)),state.Rotation,initialPossible,out);
}
}
namespace NativeBoxAntipodal
{
// Observation only, defined by the existing native-value translation unit.
IAMSPEED_API void ObserveRange(uint32 rotations,uint32 reflections,bool known);
struct FObservation
{
 bool Enabled=false,Known=false;uint32 Rotations=0,Reflections=0;
 ~FObservation(){if(Enabled)ObserveRange(Rotations,Reflections,Known);}
};
inline FIntervalPoint OppositeOffset(const FIntervalPoint& original)
{
 FIntervalPoint result;
 for(int32 d=0;d<3;++d)
 {
  auto value=AnalyticCertified::Negate(original.Axis[d]);if(!AnalyticCertified::Valid(value))return {};
  // Every zero endpoint emitted by Outward is+0; raw Negate could emit-0.
  if((AnalyticCertified::Bits(value.Lower)&AnalyticCertified::Magnitude)==0)value.Lower=0.;
  if((AnalyticCertified::Bits(value.Upper)&AnalyticCertified::Magnitude)==0)value.Upper=0.;
  result.Axis[d]=value;
 }
 return result;
}
}
inline bool BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(const FGeneralBoxMotionInput& v,float first,float last,
    FGeneralBoxMotionBounds& output,bool antipodal=false,bool exactConsumedSingleton=false)
{
    NativeBoxAntipodal::FObservation observed{antipodal};
    output={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    using namespace GeneralBoxDetail;using namespace YawFloorBoxDetail;using namespace GeneralBoxGlobalTimeDetail;
    if(!ValidFloatSecondsRange(first,last)||last>v.Horizon)return false;
    Interval exactConsumedSquare;
    if(exactConsumedSingleton)
    {
        if(first!=last)return false;
        // A normal binary32 operand squared is exact in binary64. Require the
        // square to stay in the normal finite binary32 range before rounding.
        // The nearest-even store is the native float(s*s), with no DAZ/FTZ edge.
        const double square=double(first)*double(first);
        if(!(square>=double(std::numeric_limits<float>::min())&&square<=double(std::numeric_limits<float>::max())))return false;
        volatile float rounded=static_cast<float>(square);
        exactConsumedSquare=Interval::Exact(double(rounded));
        if(!Valid(exactConsumedSquare))return false;
    }
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
        FQuaternion ownerRotation;
        if(!RotationFamilyOnGlobalFloatSeconds(owner,first,last,consumed,ownerRotation,
            exactConsumedSingleton?&exactConsumedSquare:nullptr))return false;
        FIntervalPoint ownerCenter;
        if(consumed&&!exactConsumedSingleton)ownerCenter=BoundConsumedCenterBetweenFloatSeconds(center,first,last);
        else
        {
            const Interval time{double(first),double(last),true};
            const auto squared=consumed?exactConsumedSquare:Square(time);
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
        for(const auto& extent:{v.PhysicalHalfExtent,v.QueryHalfExtent})
        {
         FIntervalPoint firstHalf[4];
         for(int32 corner=0;corner<8;++corner)
         {
            FVector local;for(int32 d=0;d<3;++d)local[d]=(corner&(1<<d))?extent[d]:-extent[d];
            FIntervalPoint rotated;
            if(!antipodal||corner<4)
            {
                rotated=PointRanges(Rotate(bodyRotation,local));
                if(antipodal){firstHalf[corner]=rotated;++observed.Rotations;}
            }
            else
            {
                // Opposite LOCAL corner7-i, same quaternion interval family.
                // Never negate the already-translated point or mix corner roles.
                rotated=NativeBoxAntipodal::OppositeOffset(firstHalf[7-corner]);++observed.Reflections;
            }
            if(!Detail::ValidPoint(rotated))return false;
            FIntervalPoint point;
            for(int32 d=0;d<3;++d)point.Axis[d]=Add(bodyCenter.Axis[d],rotated.Axis[d]);
            if(!YawFloorBoxDetail::Join(result.Hull,point))return false;
            result.Corners.Add(point);
         }
        }
    }
    if(!Detail::ValidPoint(result.Hull))return false;
    output=MoveTemp(result);observed.Known=true;return true;
}
}
