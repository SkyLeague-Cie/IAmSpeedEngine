#pragma once
#include "HAL/IConsoleManager.h"
#include "IAmSpeed/World/Analytic/AffineSphereNearestRoundQ112.h"
#include "IAmSpeed/World/Collision/ReferenceTransport.h"

#if !UE_BUILD_SHIPPING
namespace Speed::Analytic::Certified::Case205ReferenceTiming
{
namespace A=::AnalyticCertified;
namespace Q=AffineSphereNearestRoundQ112::DetailQ;
using Q::FNumber;
using Q::FRange;
using Q::Add;
using Q::Multiply;
using Q::Subtract;
using Q::FromDouble;

constexpr uint32 BulkEndBits=0x2f343200u,ReferenceClearBits=0x2f343419u,ReferenceInsideBits=0x2f34341au;
constexpr uint32 NativeClearBits=0x2f343424u,NativeInsideBits=0x2f343425u;
constexpr int32 Provider=1407,ActiveCorner=1,OppositeCorner=5;
constexpr uint32 Samples=NativeInsideBits-BulkEndBits+1;
static_assert(Samples==550&&ReferenceClearBits-BulkEndBits+1==538);
static TAutoConsoleVariable<int32> Enabled(
    TEXT("p.IAmSpeed.AnalyticWorld.NativeBox205ReferenceTiming"),0,
    TEXT("Private 205 reference-time arrival with separate later native witness. Default OFF."));

struct FPoint {FRange Axis[3];};
struct FQuaternion {FRange Axis[4];};

inline FRange FromInterval(const A::Interval& value)
{
    if(!A::Valid(value))return {};
    const auto lo=FromDouble(value.Lower),hi=FromDouble(value.Upper);
    return Q::Valid(lo)&&Q::Valid(hi)?Q::Range(lo.Lower,hi.Upper):FRange{};
}

inline FPoint Point(const FVector& p)
{return {{FromDouble(p.X),FromDouble(p.Y),FromDouble(p.Z)}};}

inline FPoint Cross(const FPoint& a,const FPoint& b)
{
    FPoint out;
    for(int32 i=0;i<3;++i)
    {
        const int32 j=(i+1)%3,k=(i+2)%3;
        out.Axis[i]=Subtract(Multiply(a.Axis[j],b.Axis[k]),Multiply(a.Axis[k],b.Axis[j]));
    }
    return out;
}

inline FPoint Rotate(const FQuaternion& q,const FPoint& p)
{
    const FPoint imaginary{{q.Axis[0],q.Axis[1],q.Axis[2]}};
    auto t=Cross(imaginary,p);
    for(auto& axis:t.Axis)axis=Multiply(FromDouble(2.),axis);
    const auto c=Cross(imaginary,t);
    FPoint result;
    for(int32 d=0;d<3;++d)
        result.Axis[d]=Add(Add(p.Axis[d],Multiply(q.Axis[3],t.Axis[d])),c.Axis[d]);
    return result;
}

inline FRange Hull(const FRange& a,const FRange& b)
{
    if(!Q::Valid(a)||!Q::Valid(b))return {};
    return Q::Range(Q::Compare(a.Lower,b.Lower)<=0?a.Lower:b.Lower,Q::Compare(a.Upper,b.Upper)>=0?a.Upper:b.Upper);
}

inline bool StrictPositive(const FRange& value)
{return Q::Valid(value)&&Q::Compare(value.Lower,{})>0;}

inline bool StrictNegative(const FRange& value)
{return Q::Valid(value)&&Q::Compare(value.Upper,{})<0;}

// Exact continuous real motion only. The unit and raw branches differ only
// by division by the constant initial quaternion norm. Native rounded motion
// is certified independently from the same input by the outer producer.
inline bool BoundReference(const FGeneralBoxMotionInput& input,float first,float last,int32 corner,
    FPoint& raw,FPoint& normalized)
{
    if(first<0.f||first>last||last>KinematicCenterDetail::FromBits(NativeInsideBits)||
        corner<0||corner>=8||input.LocalRotation!=FQuat4d::Identity)return false;
    const auto time=Hull(FromDouble(double(first)),FromDouble(double(last)));
    const auto square=Q::Square(time);
    const auto half=FromDouble(.5);
    FPoint theta,position,lever;
    auto thetaSquared=FromDouble(0.);
    for(int32 d=0;d<3;++d)
    {
        theta.Axis[d]=Add(Multiply(FromDouble(input.Owner.AngularVelocity[d]),time),
            Multiply(Multiply(half,FromDouble(input.Owner.AngularAcceleration[d])),square));
        thetaSquared=Add(thetaSquared,Q::Square(theta.Axis[d]));
        position.Axis[d]=Add(Add(FromDouble(input.Owner.Location[d]),Multiply(FromDouble(input.Owner.Velocity[d]),time)),
            Multiply(Multiply(half,FromDouble(input.Owner.Acceleration[d])),square));
        lever.Axis[d]=Add(Subtract(FromDouble(input.LocalOffset[d]),FromDouble(input.LocalCOM[d])),
            FromDouble((corner&(1<<d))?input.PhysicalHalfExtent[d]:-input.PhysicalHalfExtent[d]));
    }
    const auto one=FromDouble(1.);
    if(!Q::Valid(thetaSquared)||Q::Compare(thetaSquared.Lower,{})<0||Q::Compare(thetaSquared.Upper,one.Lower)>0)return false;
    const auto thetaFourth=Q::Square(thetaSquared);
    const auto series=[&](FRange limit,double firstDenominator,double secondDenominator)
    {
        const auto lo=Subtract(limit,Q::DividePositive(thetaSquared,FromDouble(firstDenominator)));
        const auto hi=Add(lo,Q::DividePositive(thetaFourth,FromDouble(secondDenominator)));
        return Q::Valid(lo)&&Q::Valid(hi)?Q::Range(lo.Lower,hi.Upper):FRange{};
    };
    const auto scale=series(half,48.,3840.),cosine=series(one,8.,384.);
    const auto& initial=input.Owner.Rotation;
    const FPoint q0{{FromDouble(initial.X),FromDouble(initial.Y),FromDouble(initial.Z)}};
    const auto qw=FromDouble(initial.W);
    FPoint imag;
    for(int32 d=0;d<3;++d)imag.Axis[d]=Multiply(theta.Axis[d],scale);
    const auto cross=Cross(imag,q0);
    FQuaternion qRaw,qUnit;
    qRaw.Axis[3]=Multiply(cosine,qw);
    auto normSquared=Q::Square(qw);
    for(int32 d=0;d<3;++d)
    {
        qRaw.Axis[d]=Add(Add(Multiply(cosine,q0.Axis[d]),Multiply(qw,imag.Axis[d])),cross.Axis[d]);
        qRaw.Axis[3]=Subtract(qRaw.Axis[3],Multiply(imag.Axis[d],q0.Axis[d]));
        normSquared=Add(normSquared,Q::Square(q0.Axis[d]));
    }
    const auto norm=Q::PositiveSqrt(normSquared);
    for(int32 d=0;d<4;++d)qUnit.Axis[d]=Q::DividePositive(qRaw.Axis[d],norm);
    const auto rawOffset=Rotate(qRaw,lever),unitOffset=Rotate(qUnit,lever);
    for(int32 d=0;d<3;++d)
    {
        raw.Axis[d]=Add(position.Axis[d],rawOffset.Axis[d]);
        normalized.Axis[d]=Add(position.Axis[d],unitOffset.Axis[d]);
        if(!Q::Valid(raw.Axis[d])||!Q::Valid(normalized.Axis[d]))return false;
    }
    return true;
}

// One dyadic inverse at the anchor, and one slope bound over the admitted
// Y domain. Exact finite-geometry signs are then constant-time per sample.
// Witnesses are proposal-free; an inconclusive comparison remains a refusal.
struct FChart
{
    Q::FCurve Curve;
    FRange YDomain,AnchorY,AnchorZ,Slope;

    template<class TPay>
    bool BoundParameter(const FRange& y,TPay& pay,FRange& parameter) const
    {
        const FNumber zero{},one=FromDouble(1.).Lower;
        if(!Q::Valid(y)||Q::Compare(y.Lower,Curve.Control[0][0].Upper)<=0||
            Q::Compare(y.Upper,Curve.Control[0][5].Lower)>=0)return false;
        const auto inverse=[&](const FNumber& target,FRange& result)
        {
            auto lo=zero,hi=one;
            for(int32 step=0;step<104;++step)
            {
                if(!pay())return false;
                const auto middle=Q::Number((lo.Magnitude+hi.Magnitude)>>1);
                const auto p=Q::Evaluate(Curve.Control[0],6,Q::Exact(middle));
                if(!Q::Valid(p))return false;
                if(Q::Compare(p.Upper,target)<0)lo=middle;
                else if(Q::Compare(p.Lower,target)>0)hi=middle;
                else break;
            }
            result=Q::Range(lo,hi);
            return Q::Valid(result)&&Q::Compare(lo,zero)>0&&Q::Compare(hi,one)<0;
        };
        FRange lo,hi;
        if(!inverse(y.Lower,lo))return false;
        if(Q::Compare(y.Lower,y.Upper)==0){parameter=lo;return true;}
        if(!inverse(y.Upper,hi))return false;
        parameter=Q::Range(lo.Lower,hi.Upper);
        return Q::Valid(parameter);
    }

    template<class TPay>
    bool SetWindow(const FRange& domain,double anchor,TPay& pay)
    {
        const auto y=FromDouble(anchor);
        if(!Q::Valid(domain)||!Q::Valid(y)||Q::Compare(y.Lower,domain.Lower)<0||
            Q::Compare(y.Upper,domain.Upper)>0)return false;
        FRange vAnchor,vDomain;
        if(!BoundParameter(y,pay,vAnchor)||!BoundParameter(domain,pay,vDomain)||!pay())return false;
        const auto dy=Q::Evaluate(Curve.Derivative[0],5,vDomain);
        const auto dz=Q::Evaluate(Curve.Derivative[1],5,vDomain);
        const auto slope=Q::DividePositive(dz,dy);
        const auto z=Q::Evaluate(Curve.Control[1],6,vAnchor);
        if(!StrictPositive(slope)||!Q::Valid(z))return false;
        YDomain=domain;AnchorY=y;AnchorZ=z;Slope=slope;
        return true;
    }

    FRange Gap(const FRange& y,const FRange& z) const
    {
        if(!Q::Valid(YDomain)||!Q::Valid(y)||!Q::Valid(z)||
            Q::Compare(y.Lower,YDomain.Lower)<0||Q::Compare(y.Upper,YDomain.Upper)>0)return {};
        return Subtract(Subtract(z,AnchorZ),Multiply(Slope,Subtract(y,AnchorY)));
    }
};

// Six controls copied verbatim from exact generated Stadium row 51
// (StadiumAffineSphereQ112Generated.h SHA d1e44fc6...432fdb). They are
// usable only after the same packet, recipe, canonical face and affine
// product authentication as the original stationary 1407 certificate.
inline constexpr AffineSphereNearestRoundQ112::FControl Controls[6]={
    { { { { { 0x5bb0462bu, 0x7cf48e70u, 0x33f9de64u, 0x0fb504f3u }, false }, { { 0x5bb0462cu, 0x7cf48e70u, 0x33f9de64u, 0x0fb504f3u }, false } }, { { { 0xa44fb9d4u, 0x830b718fu, 0xcc06219bu, 0x004afb0cu }, false }, { { 0xa44fb9d5u, 0x830b718fu, 0xcc06219bu, 0x004afb0cu }, false } } } },
    { { { { { 0x19be8129u, 0x04c3564bu, 0x4977c71bu, 0x0fc33c8fu }, false }, { { 0x19be812au, 0x04c3564bu, 0x4977c71bu, 0x0fc33c8fu }, false } }, { { { 0x625df4d1u, 0x0ada396au, 0xe1840a52u, 0x005932a8u }, false }, { { 0x625df4d2u, 0x0ada396au, 0xe1840a52u, 0x005932a8u }, false } } } },
    { { { { { 0x3f1f6287u, 0xa3f7f544u, 0x00898d4cu, 0x0fd00edau }, false }, { { 0x3f1f6288u, 0xa3f7f544u, 0x00898d4cu, 0x0fd00edau }, false } }, { { { 0xb919896fu, 0x7b432a26u, 0x556e158du, 0x0068cf96u }, false }, { { 0xb9198970u, 0x7b432a26u, 0x556e158du, 0x0068cf96u }, false } } } },
    { { { { { 0x73b91246u, 0xffd91c18u, 0x8f31ea18u, 0x0fdb4d06u }, false }, { { 0x73b91247u, 0xffd91c18u, 0x8f31ea18u, 0x0fdb4d06u }, false } }, { { { 0xbda0fccfu, 0x1bb311ceu, 0x40be684cu, 0x0079a314u }, false }, { { 0xbda0fcd0u, 0x1bb311ceu, 0x40be684cu, 0x0079a314u }, false } } } },
    { { { { { 0xda23ae47u, 0xfe64b2feu, 0x6008e840u, 0x0fe4d1a0u }, false }, { { 0xda23ae48u, 0xfe64b2feu, 0x6008e840u, 0x0fe4d1a0u }, false } }, { { { 0xd80db427u, 0x7b9c2c9au, 0x4f1b0b0bu, 0x008b7514u }, false }, { { 0xd80db428u, 0x7b9c2c9au, 0x4f1b0b0bu, 0x008b7514u }, false } } } },
    { { { { { 0x908bfebbu, 0x43210d0au, 0x79946a31u, 0x0fec835eu }, false }, { { 0x908bfebcu, 0x43210d0au, 0x79946a31u, 0x0fec835eu }, false } }, { { { 0x2ad2f02du, 0xb4651ee2u, 0x65455a74u, 0x009e0875u }, false }, { { 0x2ad2f02eu, 0xb4651ee2u, 0x65455a74u, 0x009e0875u }, false } } } },
};

template<class TPay>
bool AuthenticateChart(const Prepared::FReferenceTransport& reference,const AffineReferenceDetail::FProduct& product,
    TPay& pay,FChart& out)
{
    constexpr const TCHAR* packet=TEXT("090d41158d13f1127827b720ff467a51e415668aa205209a67ed8afcb6b10dc0");
    constexpr const TCHAR* recipe=TEXT("46cd45f7af71cf745f541c92e1a10066c9dba9cb7fcc27d1fbe0d0ecdcaf31a8");
    constexpr const TCHAR* correlation=TEXT("719562ab664d391eaf4de243d049078909284b5172c997894ae229f3829d0892");
    constexpr const TCHAR* canonical=TEXT("df952a56fdf5215eb6d3fd663c6335f2d19b90220d07624e5d47bd2f28d7bb0f");
    constexpr const TCHAR* bound=TEXT("51bad3088f1b4cfc0dd5afd70f65fc4e669703ba47fc38cee0ad1779866241c8");
    constexpr const TCHAR* projected=TEXT("87cbf26144dfb64be6cf382621d440d7a3ad5aa1f297d03cb411d7db2ca5a05c");
    const auto pins=reference.GetProofIdentities();
    const auto faces=reference.GetFaces();
    const auto map=reference.GetProviderFaceIndices();
    if(!pay()||reference.GetExternalPacketIdentity()!=packet||reference.GetExactAffineProofIdentity()!=correlation||
        pins.Num()!=10||pins[0]!=recipe||!map.IsValidIndex(Provider)||map[Provider]!=Provider||
        !faces.IsValidIndex(Provider)||faces[Provider].CanonicalRecordIdentity!=canonical||
        faces[Provider].BoundRecordIdentity!=bound||product.FreeWorldAxis!=0||!product.FreeAlongU||
        product.Curve.DegreeU!=0||product.Curve.DegreeV!=5||product.Curve.ControlPoints.Num()!=6)return false;
    const Prepared::FReferenceAffineExtrusionDescription* fact=nullptr;
    for(const auto& row:reference.GetExactAffineExtrusions(Provider))
    {
        if(!pay())return false;
        if(&row.Product!=&product)continue;
        if(fact||row.Provider!=Provider||row.SourceFace!=Provider||row.CorrelationProofIdentity!=correlation||
            row.ExactProjectedCurveIdentity!=projected||row.CanonicalRecordIdentity!=canonical||row.BoundRecordIdentity!=bound)
            return false;
        fact=&row;
    }
    if(!fact)return false;
    for(int32 d=0;d<2;++d)
    {
        for(int32 i=0;i<6;++i)
        {
            if(!pay())return false;
            const auto& fixed=Controls[i].Coordinate[d];
            const auto value=Q::Range(Q::FromLimbs(fixed.Lower),Q::FromLimbs(fixed.Upper));
            const auto coarse=FromInterval(product.Curve.ControlPoints[i].Axis[d+1]);
            if(!Q::Valid(value)||!Q::Valid(coarse)||Q::Compare(value.Lower,coarse.Lower)<0||
                Q::Compare(value.Upper,coarse.Upper)>0)return false;
            out.Curve.Control[d][i]=value;
        }
        for(int32 i=0;i<5;++i)
        {
            if(!pay())return false;
            auto& derivative=out.Curve.Derivative[d][i];
            derivative=Multiply(FromDouble(5.),Subtract(out.Curve.Control[d][i+1],out.Curve.Control[d][i]));
            if(!StrictPositive(derivative))return false;
        }
        for(int32 i=0;i<4;++i)
        {
            if(!pay())return false;
            auto& derivative=out.Curve.SecondDerivative[d][i];
            derivative=Multiply(FromDouble(4.),Subtract(out.Curve.Derivative[d][i+1],out.Curve.Derivative[d][i]));
            if(d==0?!StrictNegative(derivative):!StrictPositive(derivative))return false;
        }
    }
    return true;
}

inline bool InOriginalChart(const FChart& chart,const AffineReferenceDetail::FProduct& product,const FPoint& point)
{
    const auto low=FromInterval(product.Start),high=FromInterval(product.End);
    if(!Q::Valid(low)||!Q::Valid(high)||!Q::Valid(point.Axis[0])||!Q::Valid(point.Axis[1]))return false;
    const bool increasing=Q::Compare(low.Upper,high.Lower)<0;
    const auto& left=increasing?low:high;
    const auto& right=increasing?high:low;
    return Q::Compare(left.Upper,point.Axis[0].Lower)<0&&Q::Compare(point.Axis[0].Upper,right.Lower)<0&&
        Q::Compare(chart.Curve.Control[0][0].Upper,point.Axis[1].Lower)<0&&
        Q::Compare(point.Axis[1].Upper,chart.Curve.Control[0][5].Lower)<0;
}

inline bool ReferenceGapDecreases(const FGeneralBoxMotionInput& input,const FChart& chart)
{
    // The derivative of exp(theta/2) has norm at most |theta'|/2.
    // Hamilton products scale that by the fixed raw norm, while the unit
    // family divides by that same constant. Differentiate Rotate as in
    // BoundReference: the four cross-product terms give 8|q||q'||lever|.
    constexpr double minNorm=1.-0x1p-10,maxNorm=1.+0x1p-10;
    const auto duration=FromDouble(double(KinematicCenterDetail::FromBits(NativeInsideBits)));
    const auto& q=input.Owner.Rotation;
    auto norm2=FromDouble(0.);
    for(double component:{q.X,q.Y,q.Z,q.W})norm2=Add(norm2,Q::Square(FromDouble(component)));
    const auto lo=FromDouble(minNorm*minNorm),hi=FromDouble(maxNorm*maxNorm);
    if(!Q::Valid(norm2)||Q::Compare(norm2.Lower,lo.Lower)<=0||Q::Compare(norm2.Upper,hi.Upper)>=0)return false;
    auto thetaRate=FromDouble(0.),lever=FromDouble(0.);
    for(int32 d=0;d<3;++d)
    {
        thetaRate=Add(thetaRate,Add(FromDouble(FMath::Abs(input.Owner.AngularVelocity[d])),
            Multiply(duration,FromDouble(FMath::Abs(input.Owner.AngularAcceleration[d])))));
        const auto axis=Subtract(FromDouble(input.LocalOffset[d]),FromDouble(input.LocalCOM[d]));
        if(!Q::Valid(axis))return false;
        const auto absLower=axis.Lower.Negative?Q::Negate(axis.Lower):axis.Lower;
        const auto absUpper=axis.Upper.Negative?Q::Negate(axis.Upper):axis.Upper;
        lever=Add(lever,Add(Q::Exact(Q::Compare(absLower,absUpper)>0?absLower:absUpper),
            FromDouble(input.PhysicalHalfExtent[d])));
    }
    const auto rotateRate=Multiply(Multiply(FromDouble(4.*maxNorm*maxNorm),thetaRate),lever);
    const auto ySpeed=Subtract(Subtract(FromDouble(input.Owner.Velocity.Y),
        Multiply(duration,FromDouble(FMath::Abs(input.Owner.Acceleration.Y)))),rotateRate);
    const auto zSpeed=Add(Add(FromDouble(FMath::Abs(input.Owner.Velocity.Z)),
        Multiply(duration,FromDouble(FMath::Abs(input.Owner.Acceleration.Z)))),rotateRate);
    return StrictPositive(ySpeed)&&StrictNegative(Subtract(zSpeed,Multiply(chart.Slope,ySpeed)));
}

struct FNativePrefixAndLaterWitness
{
    uint32 ClearThroughBits=0,LastClearBits=0,InsideBits=0,NativeSamples=0;
    FIntervalPoint ConsumedCenter;
    bool Matches(uint32 clearBits,uint32 insideReferenceBits) const
    {
        return clearBits==ReferenceClearBits&&insideReferenceBits==ReferenceInsideBits&&
            ClearThroughBits==clearBits&&LastClearBits==NativeClearBits&&InsideBits==NativeInsideBits&&
            LastClearBits+1==InsideBits&&insideReferenceBits<LastClearBits&&NativeSamples==Samples&&
            Detail::ValidPoint(ConsumedCenter);
    }
};

// Shared paid traversal and sign callbacks come from the original graph.
// All physical bounds use the original global time rather than restarting
// the integration DAG. No payload can escape before both proofs close.
template<class TPay,class TOtherSign,class TContains>
bool Prove(const FGeneralBoxMotionInput& input,const Prepared::FReferenceTransport& reference,
    const AffineReferenceDetail::FProduct& product,uint64 numeric,TPay& pay,TOtherSign&& signOther,TContains&& contains,
    FNativePrefixAndLaterWitness& out,const TCHAR*& stage)
{
    out={};
    const auto seconds=[](uint32 bits){return KinematicCenterDetail::FromBits(bits);};
    const auto bounds=[&](float first,float last,FGeneralBoxMotionBounds& value)
    {
        return pay()&&BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(input,first,last,value)&&
            value.Corners.Num()==32;
    };
    stage=TEXT("chart");
    FChart chart;
    FGeneralBoxMotionBounds wide,atReferenceClear;
    if(!AuthenticateChart(reference,product,pay,chart)||!bounds(0.f,seconds(NativeInsideBits),wide)||
        !bounds(seconds(ReferenceClearBits),seconds(ReferenceClearBits),atReferenceClear))return false;
    const auto wideY=TimeDetail::Hull(wide.Corners[ActiveCorner],wide.Corners[16+ActiveCorner]).Axis[1];
    const auto nearY=atReferenceClear.Corners[ActiveCorner].Axis[1];
    const double anchor=.5*nearY.Lower+.5*nearY.Upper;
    // This pads only the slope's verified chart domain, never any physical
    // sign or eligibility threshold. Independent interval forms may expand
    // endpoints differently. The chart must authenticate the enlarged span.
    const auto domain=A::Add(wideY,A::Interval{-0x1p-20,0x1p-20,true});
    if(!chart.SetWindow(FromInterval(domain),anchor,pay)||!ReferenceGapDecreases(input,chart))return false;
    stage=TEXT("other_corners");
    for(int32 family=0;family<2;++family)for(int32 k=0;k<8;++k)
    {
        if(k==ActiveCorner)continue;
        const auto& point=wide.Corners[16*family+k];
        if(!contains(point)||!signOther(point))return false;
    }
    stage=TEXT("references");
    for(uint32 word:{ReferenceClearBits,ReferenceInsideBits})
    {
        FPoint raw,normalized;
        if(!pay()||!BoundReference(input,seconds(word),seconds(word),ActiveCorner,raw,normalized))return false;
        for(const auto& point:{raw,normalized})
        {
            if(!InOriginalChart(chart,product,point))return false;
            const auto gap=chart.Gap(point.Axis[1],point.Axis[2]);
            if(word==ReferenceClearBits?!StrictPositive(gap):!StrictNegative(gap))return false;
        }
    }
    FPoint raw,normalized;
    if(!pay()||!BoundReference(input,seconds(ReferenceInsideBits),seconds(ReferenceInsideBits),OppositeCorner,raw,normalized)||
        !InOriginalChart(chart,product,raw)||!InOriginalChart(chart,product,normalized))return false;

    stage=TEXT("native_bulk");
    struct FCell {float First,Last;};
    TArray<FCell,TInlineAllocator<32>> pending;
    pending.Add(FCell{0.f,seconds(BulkEndBits)});
    float covered=0.f;
    for(int32 visited=0;visited<512&&!pending.IsEmpty();++visited)
    {
        const auto cell=pending.Pop(EAllowShrinking::No);
        if(cell.First!=covered||!(cell.First<cell.Last))return false;
        FGeneralBoxMotionBounds value;
        if(!bounds(cell.First,cell.Last,value))return false;
        const auto& point=value.Corners[ActiveCorner];
        if(contains(point)&&StrictPositive(chart.Gap(FromInterval(point.Axis[1]),FromInterval(point.Axis[2]))))
        {
            covered=cell.Last;
            continue;
        }
        const auto middle=float(.5*double(cell.First)+.5*double(cell.Last));
        if(!(cell.First<middle&&middle<cell.Last)||pending.Num()>=30||!pay())return false;
        pending.Add(FCell{middle,cell.Last});pending.Add(FCell{cell.First,middle});
    }
    if(!pending.IsEmpty()||covered!=seconds(BulkEndBits))return false;

    stage=TEXT("native_band");
    FIntervalPoint consumedCenter;
    uint32 count=0;
    for(uint32 word=BulkEndBits;word<=NativeInsideBits;++word)
    {
        if(!pay())return false;
        auto pose=input.Owner.Integrate(seconds(word));
        pose.Location-=pose.Rotation.RotateVector(input.LocalCOM);
        pose.Location+=pose.Rotation.RotateVector(input.LocalOffset);
        pose.Rotation=pose.Rotation*input.LocalRotation;
        if(pose.Location.ContainsNaN()||pose.Rotation.ContainsNaN()||
            RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=numeric)return false;
        if(word==ReferenceClearBits)consumedCenter=Detail::Point(pose.Location);
        const auto corner=[&](int32 k)
        {
            FVector local;
            for(int32 d=0;d<3;++d)local[d]=(k&(1<<d))?input.PhysicalHalfExtent[d]:-input.PhysicalHalfExtent[d];
            return pose.Location+pose.Rotation.RotateVector(local);
        };
        const auto vertex=corner(ActiveCorner);
        if(!contains(Detail::Point(vertex)))return false;
        const auto point=Point(vertex);
        const auto gap=chart.Gap(point.Axis[1],point.Axis[2]);
        if(word<NativeInsideBits?!StrictPositive(gap):!StrictNegative(gap))return false;
        if(word==NativeInsideBits)
        {
            const auto opposite=corner(OppositeCorner);
            if(!pay()||!contains(Detail::Point(opposite))||!InOriginalChart(chart,product,point)||
                !InOriginalChart(chart,product,Point(opposite)))return false;
            // The broad physical sign for every non-active corner has
            // already covered this exact opposite endpoint.
        }
        ++count;
    }
    if(!Detail::ValidPoint(consumedCenter)||count!=Samples)return false;
    out={ReferenceClearBits,NativeClearBits,NativeInsideBits,count,consumedCenter};
    stage=TEXT("complete");
    return true;
}
}
#endif
