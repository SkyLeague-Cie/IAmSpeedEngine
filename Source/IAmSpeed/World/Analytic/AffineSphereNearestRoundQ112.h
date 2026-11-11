#pragma once
#include "AffineSphereNearestOverlap.h"
#include "Math/BigInt.h"

namespace Speed::Analytic::Certified::AffineSphereNearestRoundQ112
{
// Conditional values kernel. Neither an affine-source authenticator nor an
// overlap authorizer. Q112 and the unsigned limb widths are proof constants.
// A chart whose values exceed the bounded arithmetic stays unavailable.
struct FLimbs
{
    uint32 Word[4];
    bool Negative;
};
struct FEncodedRange { FLimbs Lower, Upper; };
struct FControl { FEncodedRange Coordinate[2]; };
struct FRoundedOverlap
{
    FVector3d Point=FVector3d::ZeroVector;
    FVector3d Normal=FVector3d::ZeroVector;
    double Depth=0;
    float Depth32=0;
};

namespace DetailQ
{
namespace A=::AnalyticCertified;
using UInt=TBigInt<256,false>;
constexpr int32 FractionBits=112;
struct FNumber { UInt Magnitude; bool Negative=false; };
struct FRange { FNumber Lower,Upper; bool Known=false; };

inline FNumber Number(UInt magnitude,bool negative=false)
{return {magnitude,negative&&!magnitude.IsZero()};}
inline int32 Compare(const FNumber& a,const FNumber& b)
{
    if(a.Negative!=b.Negative)return a.Negative?-1:1;
    if(a.Magnitude==b.Magnitude)return 0;
    return (a.Magnitude<b.Magnitude)!=a.Negative?-1:1;
}
inline bool Bounded(const FNumber& a)
{return a.Magnitude.GetHighestNonZeroBit()<126;}
inline bool Valid(const FRange& a)
{return a.Known&&Bounded(a.Lower)&&Bounded(a.Upper)&&Compare(a.Lower,a.Upper)<=0;}
inline FRange Range(FNumber lo,FNumber hi)
{
    FRange out{MoveTemp(lo),MoveTemp(hi),true};
    return Valid(out)?out:FRange{};
}
inline FRange Exact(FNumber a){return Range(a,a);}
inline FNumber Negate(const FNumber& a){return Number(a.Magnitude,!a.Negative);}
inline FNumber Add(const FNumber& a,const FNumber& b)
{
    if(a.Negative==b.Negative)return Number(a.Magnitude+b.Magnitude,a.Negative);
    if(a.Magnitude<b.Magnitude)return Number(b.Magnitude-a.Magnitude,b.Negative);
    return Number(a.Magnitude-b.Magnitude,a.Negative);
}
inline FRange Add(const FRange& a,const FRange& b)
{
    if(!Valid(a)||!Valid(b))return {};
    return Range(Add(a.Lower,b.Lower),Add(a.Upper,b.Upper));
}
inline FRange Subtract(const FRange& a,const FRange& b)
{
    if(!Valid(a)||!Valid(b))return {};
    return Range(Add(a.Lower,Negate(b.Upper)),Add(a.Upper,Negate(b.Lower)));
}
inline FNumber Product(const FNumber& a,const FNumber& b,bool up)
{
    // Both input magnitudes are <2^126. The exact unsigned product fits.
    const UInt product=a.Magnitude*b.Magnitude;
    UInt scaled=product>>FractionBits;
    const bool negative=a.Negative!=b.Negative;
    if(up!=negative&&(scaled<<FractionBits)!=product)++scaled;
    return Number(scaled,negative);
}
inline FRange Multiply(const FRange& a,const FRange& b)
{
    if(!Valid(a)||!Valid(b))return {};
    if(Compare(b.Lower,{})>=0)
    {
        const auto& lowerScale=a.Lower.Negative?b.Upper:b.Lower;
        const auto& upperScale=a.Upper.Negative?b.Lower:b.Upper;
        return Range(Product(a.Lower,lowerScale,false),Product(a.Upper,upperScale,true));
    }
    FNumber lo=Product(a.Lower,b.Lower,false),hi=Product(a.Lower,b.Lower,true);
    const FNumber x[]={a.Lower,a.Upper},y[]={b.Lower,b.Upper};
    for(const auto& p:x)for(const auto& q:y)
    {
        const auto lower=Product(p,q,false),upper=Product(p,q,true);
        if(Compare(lower,lo)<0)lo=lower;
        if(Compare(upper,hi)>0)hi=upper;
    }
    return Range(lo,hi);
}
inline FRange Square(const FRange& a)
{
    if(!Valid(a))return {};
    if(a.Lower.Negative!=a.Upper.Negative)
    {
        const auto end=a.Lower.Magnitude>a.Upper.Magnitude?a.Lower:a.Upper;
        return Range({},Product(end,end,true));
    }
    if(a.Lower.Negative)return Range(Product(a.Upper,a.Upper,false),Product(a.Lower,a.Lower,true));
    return Range(Product(a.Lower,a.Lower,false),Product(a.Upper,a.Upper,true));
}
inline FNumber Quotient(const FNumber& a,const FNumber& positive,bool up)
{
    // Bounded numerators shifted by FractionBits remain below 2^238.
    UInt value=a.Magnitude<<FractionBits,remainder;
    value.DivideWithRemainder(positive.Magnitude,remainder);
    if(up!=a.Negative&&!remainder.IsZero())++value;
    return Number(value,a.Negative);
}
inline FRange DividePositive(const FRange& a,const FRange& b)
{
    if(!Valid(a)||!Valid(b)||Compare(b.Lower,{})<=0)return {};
    const auto& lo=a.Lower.Negative?b.Lower:b.Upper;
    const auto& hi=a.Upper.Negative?b.Upper:b.Lower;
    return Range(Quotient(a.Lower,lo,false),Quotient(a.Upper,hi,true));
}
inline FNumber Sqrt(const FNumber& a,bool up)
{
    UInt remainder=a.Magnitude<<FractionBits;
    UInt result,bit=UInt(1)<<254;
    while(bit>remainder)bit>>=2;
    while(!bit.IsZero())
    {
        const UInt trial=result+bit;
        result>>=1;
        if(remainder>=trial){remainder-=trial;result+=bit;}
        bit>>=2;
    }
    if(up&&!remainder.IsZero())++result;
    return Number(result);
}
inline FRange PositiveSqrt(const FRange& a)
{
    if(!Valid(a)||Compare(a.Lower,{})<=0)return {};
    return Range(Sqrt(a.Lower,false),Sqrt(a.Upper,true));
}
inline FNumber FromLimbs(const FLimbs& a)
{
    UInt magnitude;
    for(int32 i=0;i<4;++i)magnitude.GetBits()[i]=a.Word[i];
    return Number(magnitude,a.Negative);
}
inline FRange FromDouble(double value)
{
    const uint64 bits=A::Bits(value);
    const uint64 exponent=(bits>>52)&2047;
    const uint64 fraction=bits&0x000fffffffffffffull;
    if(exponent==0)return fraction?FRange{}:Exact({});
    if(exponent==2047)return {};
    const int32 shift=int32(exponent)-1023-52+FractionBits;
    UInt magnitude(int64(fraction|(1ull<<52)));
    if(shift>=0)
    {
        if(shift+52>=126)return {};
        magnitude<<=shift;
    }
    else
    {
        if(shift < -52)return {};
        const UInt quotient=magnitude>>(-shift);
        if((quotient<<(-shift))!=magnitude)return {};
        magnitude=quotient;
    }
    return Exact(Number(magnitude,(bits>>63)!=0));
}
inline uint64 RoundBits(const FNumber& a,int32 precision,int32 bias)
{
    const int32 highest=a.Magnitude.GetHighestNonZeroBit();
    if(highest<0)return 0;
    int32 exponent=highest-FractionBits;
    const int32 discard=highest-(precision-1);
    UInt significand;
    if(discard<=0)significand=a.Magnitude<<(-discard);
    else
    {
        significand=a.Magnitude>>discard;
        const UInt remainder=a.Magnitude-(significand<<discard);
        const UInt halfway=UInt(1)<<(discard-1);
        if(remainder>halfway||(remainder==halfway&&(significand.GetBits()[0]&1)))++significand;
        if(significand.GetHighestNonZeroBit()>=precision){significand>>=1;++exponent;}
    }
    const uint64 s=uint64(significand.GetBits()[0])|(uint64(significand.GetBits()[1])<<32);
    const int32 width=precision==53?64:32;
    return (uint64(a.Negative)<<(width-1))|(uint64(exponent+bias)<<(precision-1))|
        (s&((uint64(1)<<(precision-1))-1));
}
inline bool Round64(const FRange& a,double& out)
{
    if(!Valid(a))return false;
    const auto lo=RoundBits(a.Lower,53,1023),hi=RoundBits(a.Upper,53,1023);
    if(lo!=hi)return false;
    out=A::FromBits(lo);
    return true;
}
inline bool RoundDepth32(const FRange& a,double depth,float& out)
{
    // Pose depth must be positive and ordinary in both target formats.
    if(!Valid(a)||Compare(a.Lower,{})<=0)return false;
    const auto lo=RoundBits(a.Lower,24,127),hi=RoundBits(a.Upper,24,127);
    const float actual=float(depth);
    if(lo!=hi||KinematicCenterDetail::Bits(actual)!=uint32(lo))return false;
    out=actual;
    return true;
}
struct FCurve
{
    FRange Control[2][6];
    FRange Derivative[2][5];
    FRange SecondDerivative[2][4];
};
inline FRange Evaluate(const FRange* source,int32 count,const FRange& t)
{
    FRange points[6];
    for(int32 i=0;i<count;++i)points[i]=source[i];
    for(int32 level=count-1;level>0;--level)
        for(int32 i=0;i<level;++i)
            points[i]=Add(points[i],Multiply(Subtract(points[i+1],points[i]),t));
    return points[0];
}
inline FRange Stationary(const FCurve& curve,const FRange (&center)[2],const FNumber& t)
{
    FRange result=Exact({});
    const auto v=Exact(t);
    for(int32 i=0;i<2;++i)
    {
        const auto delta=Subtract(Evaluate(curve.Control[i],6,v),center[i]);
        result=Add(result,Multiply(delta,Evaluate(curve.Derivative[i],5,v)));
    }
    return result;
}
inline FRange StationaryDerivative(const FCurve& curve,const FRange (&center)[2],const FRange& v)
{
    FRange result=Exact({});
    for(int32 i=0;i<2;++i)
    {
        const auto delta=Subtract(Evaluate(curve.Control[i],6,v),center[i]);
        const auto tangent=Evaluate(curve.Derivative[i],5,v);
        result=Add(result,Add(Square(tangent),Multiply(delta,Evaluate(curve.SecondDerivative[i],4,v))));
    }
    return result;
}
}

// The caller supplies the existing unique-interior-global-minimum proof for
// this exact chart and original singleton. The narrower exact source controls
// must be authenticated for that very chart, not inferred from double boxes.
inline bool TryRoundNearest(TConstArrayView<FControl> controls,int32 freeAxis,
    const FVector3d& center,double radius,double domainTolerance,
    const AffineSphereNearestOverlap::FBounds& nearest,FDistanceBudget& budget,FRoundedOverlap& out)
{
    using namespace DetailQ;
    out={};
    if(controls.Num()!=6||freeAxis<0||freeAxis>=2||nearest.OriginalProvider==INDEX_NONE||
        !A::Valid(nearest.OriginalCurveV)||!(nearest.OriginalCurveV.Lower>0&&nearest.OriginalCurveV.Upper<1)||
        !A::Finite(radius)||!(radius>0)||!A::Finite(domainTolerance)||domainTolerance<0)return false;
    const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    const auto qRadius=FromDouble(radius),qTolerance=FromDouble(domainTolerance);
    const int32 planeAxis[2]={1-freeAxis,2};
    FRange qCenter[2];FCurve curve;
    for(int32 d=0;d<2;++d)
    {
        qCenter[d]=FromDouble(center[planeAxis[d]]);
        if(!Valid(qCenter[d]))return false;
        for(int32 i=0;i<6;++i)
        {
            if(!budget.Consume())return false;
            const auto& row=controls[i].Coordinate[d];
            curve.Control[d][i]=Range(FromLimbs(row.Lower),FromLimbs(row.Upper));
            if(!Valid(curve.Control[d][i]))return false;
        }
        for(int32 i=0;i<5;++i)
        {
            curve.Derivative[d][i]=Multiply(FromDouble(5),Subtract(curve.Control[d][i+1],curve.Control[d][i]));
            if(!Valid(curve.Derivative[d][i]))return false;
        }
        for(int32 i=0;i<4;++i)
        {
            curve.SecondDerivative[d][i]=Multiply(FromDouble(4),
                Subtract(curve.Derivative[d][i+1],curve.Derivative[d][i]));
            if(!Valid(curve.SecondDerivative[d][i]))return false;
        }
    }
    // The existing binary64 bracket is only a seed. Reprove the signed
    // bracket in the refined chart. Leave exact endpoint/tie cases closed.
    const double seedLower=nearest.OriginalCurveV.Lower,seedUpper=nearest.OriginalCurveV.Upper;
    const double spread=FMath::Max(seedUpper-seedLower,0x1p-40);
    const auto left=FromDouble(seedLower-spread),right=FromDouble(seedUpper+spread);
    if(!Valid(left)||!Valid(right)||Compare(left.Lower,{})<=0||
        Compare(right.Upper,FromDouble(1).Lower)>=0)return false;
    auto lo=left.Lower,hi=right.Upper;
    if(!budget.Consume()||!budget.Consume())return false;
    const auto before=Stationary(curve,qCenter,lo),after=Stationary(curve,qCenter,hi);
    if(!Valid(before)||!Valid(after)||Compare(before.Upper,{})>=0||Compare(after.Lower,{})<=0)return false;
    // First try a bounded interval-Newton contraction. By the mean-value
    // theorem every root in the bracket remains in the intersected range.
    // A strict midpoint sign then provides a bounded bisection if necessary.
    for(int32 step=0;step<96;++step)
    {
        const auto span=hi.Magnitude-lo.Magnitude;
        if(span.GetHighestNonZeroBit()<=FractionBits-100)break;
        if(!budget.Consume())return false;
        const auto middle=Number((lo.Magnitude+hi.Magnitude)>>1);
        const auto sign=Stationary(curve,qCenter,middle);
        if(!Valid(sign))return false;
        if(step<4)
        {
            if(!budget.Consume())return false;
            const auto slope=StationaryDerivative(curve,qCenter,Range(lo,hi));
            const auto candidate=Subtract(Exact(middle),DividePositive(sign,slope));
            if(Valid(candidate))
            {
                const auto newLower=Compare(candidate.Lower,lo)>0?candidate.Lower:lo;
                const auto newUpper=Compare(candidate.Upper,hi)<0?candidate.Upper:hi;
                if(Compare(newLower,newUpper)>0)return false;
                if(newUpper.Magnitude-newLower.Magnitude<(span>>1))
                {
                    lo=newLower;hi=newUpper;
                    continue;
                }
            }
        }
        if(Compare(sign.Upper,{})<0)lo=middle;
        else if(Compare(sign.Lower,{})>0)hi=middle;
        else break;
    }
    if(!budget.Consume())return false;
    const auto v=Range(lo,hi);
    const FRange p[2]={Evaluate(curve.Control[0],6,v),Evaluate(curve.Control[1],6,v)};
    const FRange delta[2]={Subtract(qCenter[0],p[0]),Subtract(qCenter[1],p[1])};
    const auto distance=PositiveSqrt(Add(Square(delta[0]),Square(delta[1])));
    const auto depth=Subtract(qRadius,distance);
    if(!Valid(depth)||!Valid(qTolerance)||Compare(depth.Lower,qTolerance.Upper)<=0)return false;
    FRoundedOverlap result;
    result.Point[freeAxis]=center[freeAxis]; // exact interior free chart
    for(int32 d=0;d<2;++d)
        if(!Round64(p[d],result.Point[planeAxis[d]])||
            !Round64(DividePositive(delta[d],distance),result.Normal[planeAxis[d]]))return false;
    if(!Round64(depth,result.Depth)||!RoundDepth32(depth,result.Depth,result.Depth32))return false;
    if(RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=
        numeric.ControlFingerprint())return false;
    out=result;
    return true;
}
}
