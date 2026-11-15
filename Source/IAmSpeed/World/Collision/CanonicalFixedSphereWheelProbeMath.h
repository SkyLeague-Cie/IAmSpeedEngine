#pragma once
#include "IAmSpeed/World/Analytic/TensorSphereContactBounds.h"
#include "IAmSpeed/World/Analytic/AnalyticNumericContext.h"
#include "IAmSpeed/World/Analytic/IndexedIntervalUnion.h"
#include "IAmSpeed/World/Analytic/ReferenceDyadicCellBounds.h"
#include "IAmSpeed/Base/SUtils.h"
#include <limits>
#include <type_traits>

// DRAFT: pure fixed-probe conversion values only. No World query, response,
// physical acceptance or permission to advance follows from these helpers.
namespace Speed::Analytic::Certified::FixedWheelProbeMath
{
namespace A=::AnalyticCertified;
using I=A::Interval;
using P=FIntervalPoint;
inline bool DecodeLeafDyadic(double lower,double upper,FReferenceDyadicCell& out)
{
    out={};const uint64 l=A::Bits(lower),u=A::Bits(upper);
    if((l&A::Sign)||(u&A::Sign)||l>=u||u>A::Bits(1.))return false;
    volatile double difference=upper-lower;
    const uint64 w=A::Bits(double(difference)),exponent=(w>>52)&0x7ffu;
    constexpr uint64 fractionMask=0x000fffffffffffffull;
    if((w&A::Sign)||(w&fractionMask)||exponent<971||exponent>1023)return false;
    const uint32 depth=uint32(1023-exponent);const uint64 count=uint64(1)<<depth;
    volatile double scaled=lower*double(count);const double indexValue=scaled;
    if(!A::Finite(indexValue)||indexValue<0||indexValue>=double(count))return false;
    const uint64 index=static_cast<uint64>(indexValue);if(double(index)!=indexValue)return false;
    FReferenceDyadicCell cell;cell.Depth=depth;cell.Index=index;const auto range=cell.GetRange();
    if(!A::Valid(range)||A::Bits(range.Lower)!=l||A::Bits(range.Upper)!=u)return false;
    out=cell;return true;
}
// Uniform rank-2 certificate on the SAME retained contact leaf. The chart
// cross product is not substituted for the consumed radial contact normal.
inline bool ProveRegularFaceLeaf(FIndexedIntervalUnionPtr scene,
    const FTensorSphereContactLeaf& input,FDistanceBudget& budget,P& rawNormal)
{
    const auto leaf=input;rawNormal={};
    const auto before=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!scene||!budget.Consume()||leaf.SurfaceIndex<0||leaf.SurfaceIndex>=scene->Num())return false;
    FReferenceDyadicCell u,v;
    if(!DecodeLeafDyadic(leaf.MinU,leaf.MaxU,u)||!DecodeLeafDyadic(leaf.MinV,leaf.MaxV,v)||
        !(leaf.MinU>0&&leaf.MaxU<1&&leaf.MinV>0&&leaf.MaxV<1))return false;
    FIntervalTensorSurface original,restricted;
    if(!scene->CopySurfaceForQuery(leaf.SurfaceIndex,budget,original)||
        !RestrictReferenceTensorDyadic(original,u,v,budget,restricted)||restricted.DegreeU<1||restricted.DegreeV<1)return false;
    Detail::FNode node;node.DegreeU=restricted.DegreeU;node.DegreeV=restricted.DegreeV;node.Net=MoveTemp(restricted.ControlPoints);
    const int32 work=node.DegreeU*(node.DegreeV+1)+(node.DegreeU+1)*node.DegreeV;
    for(int32 i=0;i<work;++i)if(!budget.Consume())return false;
    P du,dv,cross;
    if(!ContactDetail::DerivativeHull(node,true,du)||!ContactDetail::DerivativeHull(node,false,dv))return false;
    bool nonzero=false;
    for(int32 d=0;d<3;++d)
    {
        if(!budget.Consume())return false;
        cross.Axis[d]=A::Subtract(A::Multiply(du.Axis[(d+1)%3],dv.Axis[(d+2)%3]),
            A::Multiply(du.Axis[(d+2)%3],dv.Axis[(d+1)%3]));
        if(!A::Valid(cross.Axis[d]))return false;
        nonzero|=cross.Axis[d].Lower>0||cross.Axis[d].Upper<0;
    }
    if(!nonzero)return false;
    const auto after=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(before.ControlFingerprint()!=after.ControlFingerprint())return false;
    rawNormal=MoveTemp(cross);return true;
}
inline uint32 FloatBits(float value)
{uint32 bits;FMemory::Memcpy(&bits,&value,sizeof(bits));return bits;}
inline bool NormalOrZero(double value)
{
    const uint64 m=A::Bits(value)&A::Magnitude;
    return m<A::Infinity&&(!m||m>=A::MinNormal);
}
inline bool SameBits(const FVector& a,const FVector& b)
{
    for(int32 d=0;d<3;++d)if(A::Bits(a[d])!=A::Bits(b[d]))return false;
    return true;
}
inline bool ValidVector(const FVector& value)
{return NormalOrZero(value.X)&&NormalOrZero(value.Y)&&NormalOrZero(value.Z);}
inline bool SupportedSeconds(uint32 bits)
{return !(bits&0x80000000u)&&bits<0x7f800000u&&(!bits||bits>=0x00800000u);}

struct FUniqueFloatSeconds
{
    double NormalizedRepresentative=0;
    I Seconds;
    float NativeSeconds=0;
    uint32 NativeSecondsBits=0;
    uint64 NumericControl=0;
};
// Prove the ACTUAL double(Time*Delta) -> float conversion used by the wheel.
// The retained first-contact bracket is not relabelled as a scalar root.
inline bool TryUniqueFloatSeconds(const FTensorSphereContactEnclosure& contact,
    double delta,FUniqueFloatSeconds& out)
{
    static_assert(sizeof(float)==4&&std::numeric_limits<float>::is_iec559);
    const auto status=contact.Status;const bool leaves=!contact.Leaves.IsEmpty();
    const double lower=contact.LowerTime,upper=contact.UpperTime;out={};
    const auto before=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    const uint64 l=A::Bits(lower),u=A::Bits(upper),d=A::Bits(delta);
    if(status!=ETensorSphereContactStatus::FirstContactEnclosed||!leaves||
        (l&A::Sign)||(u&A::Sign)||(d&A::Sign)||l>=A::Infinity||u>=A::Infinity||d>=A::Infinity||
        l>u||u>A::Bits(1.)||(d&&d<A::MinNormal)||
        (l&&l<A::MinNormal)||(u&&u<A::MinNormal))return false;
    if(!d)
    {
        // Real coupled hitbox support probes use +0 Delta with a nonzero
        // geometric segment. Every root time converts to native seconds +0;
        // retain lower as the spatial representative, never turn it into a pose.
        volatile double actualSeconds=lower*delta;
        const float nativeSeconds=static_cast<float>(actualSeconds);
        const auto after=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(FloatBits(nativeSeconds)!=0||before.ControlFingerprint()!=after.ControlFingerprint())return false;
        out.NormalizedRepresentative=lower;out.Seconds=I::Exact(0.);
        out.NativeSeconds=nativeSeconds;out.NativeSecondsBits=0;out.NumericControl=before.ControlFingerprint();return true;
    }
    I seconds;
    if(!u)seconds=I::Exact(0.);
    else if(l==A::Bits(1.)&&u==l)seconds=I::Exact(delta);
    else
    {
        seconds=A::Multiply(I{lower,upper,true},I::Exact(delta));
        if(!A::Valid(seconds))return false;
        // Known nonnegative RNE product for t in[0,1] lies in[0,delta].
        // Intersect with that proved domain, never clamp a chosen time.
        if(A::Bits(seconds.Lower)&A::Sign)seconds.Lower=0.;
        if(A::Bits(seconds.Upper)>d)seconds.Upper=delta;
    }
    if(!A::Valid(seconds)||(A::Bits(seconds.Lower)&A::Sign)||(A::Bits(seconds.Upper)&A::Sign))return false;
    volatile double lo=seconds.Lower,hi=seconds.Upper;
    const uint32 lowBits=FloatBits(static_cast<float>(lo));
    const uint32 highBits=FloatBits(static_cast<float>(hi));
    if(!SupportedSeconds(lowBits)||lowBits!=highBits)return false;
    volatile double product=lower*delta;const uint64 productBits=A::Bits(double(product));
    if((productBits&A::Sign)||productBits>=A::Infinity||productBits<A::Bits(seconds.Lower)||
        productBits>A::Bits(seconds.Upper))return false;
    const float actual=static_cast<float>(product);
    if(FloatBits(actual)!=lowBits)return false;
    const auto after=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(before.ControlFingerprint()!=after.ControlFingerprint())return false;
    out={lower,seconds,actual,lowBits,before.ControlFingerprint()};return true;
}

// A fixed suspension PROBE sample, never a World arrival time or advance
// capability. The lower double is the same spatial sample chosen by the old
// unique-seconds route. Only its actual float(sample*Delta) is authenticated;
// the other roots may convert to other float seconds. Spatial quality must be
// closed separately against the real captured probe radius slack.
struct FBoundedFixedProbeSample
{
    double NormalizedRepresentative=0;
    I SourceSecondsBounds;
    double SampleProduct=0;
    float NativeSeconds=0;
    uint32 NativeSecondsBits=0;
    uint64 NumericControl=0;
};
inline bool TryBoundedFixedProbeSample(const FTensorSphereContactEnclosure& contact,
    double delta,FBoundedFixedProbeSample& out)
{
    out={};const auto before=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    const double lower=contact.LowerTime,upper=contact.UpperTime;
    const uint64 l=A::Bits(lower),u=A::Bits(upper),d=A::Bits(delta);
    if(contact.Status!=ETensorSphereContactStatus::FirstContactEnclosed||contact.Leaves.IsEmpty()||
        (l&A::Sign)||(u&A::Sign)||(d&A::Sign)||l>=A::Infinity||u>=A::Infinity||d>=A::Infinity||
        l>u||u>A::Bits(1.)||(d&&d<A::MinNormal)||(l&&l<A::MinNormal)||(u&&u<A::MinNormal))return false;
    I seconds;
    if(!d||!u)seconds=I::Exact(0.);
    else if(l==A::Bits(1.)&&u==l)seconds=I::Exact(delta);
    else
    {
        seconds=A::Multiply(I{lower,upper,true},I::Exact(delta));
        if(!A::Valid(seconds))return false;
        // Intersect a proven nonnegative product domain, never clip a sample.
        if(A::Bits(seconds.Lower)&A::Sign)seconds.Lower=0.;
        if(A::Bits(seconds.Upper)>d)seconds.Upper=delta;
    }
    if(!A::Valid(seconds))return false;
    volatile double product=lower*delta;const double sampled=product;
    const uint64 bits=A::Bits(sampled);
    if((bits&A::Sign)||bits>=A::Infinity||(bits&&bits<A::MinNormal)||
        sampled<seconds.Lower||sampled>seconds.Upper)return false;
    const float actual=static_cast<float>(product);const uint32 actualBits=FloatBits(actual);
    const auto after=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!SupportedSeconds(actualBits)||before.ControlFingerprint()!=after.ControlFingerprint())return false;
    out={lower,seconds,sampled,actual,actualBits,before.ControlFingerprint()};return true;
}
inline bool MatchesBoundedFixedProbeSample(const FTensorSphereContactEnclosure& contact,double delta,
    const FBoundedFixedProbeSample& stored)
{
    FBoundedFixedProbeSample actual;
    return TryBoundedFixedProbeSample(contact,delta,actual)&&
        A::Bits(actual.NormalizedRepresentative)==A::Bits(stored.NormalizedRepresentative)&&
        actual.SourceSecondsBounds.Known==stored.SourceSecondsBounds.Known&&
        A::Bits(actual.SourceSecondsBounds.Lower)==A::Bits(stored.SourceSecondsBounds.Lower)&&
        A::Bits(actual.SourceSecondsBounds.Upper)==A::Bits(stored.SourceSecondsBounds.Upper)&&
        A::Bits(actual.SampleProduct)==A::Bits(stored.SampleProduct)&&
        actual.NativeSecondsBits==stored.NativeSecondsBits&&FloatBits(stored.NativeSeconds)==stored.NativeSecondsBits&&
        actual.NumericControl==stored.NumericControl;
}

inline I DividePositive(I numerator,I denominator)
{
    if(!A::Valid(numerator)||!A::Valid(denominator)||!(denominator.Lower>0))return {};
    I result;bool first=true;
    for(double n:{numerator.Lower,numerator.Upper})for(double d:{denominator.Lower,denominator.Upper})
    {
        volatile double rounded=n/d;
        if(!NormalOrZero(rounded))return {};
        double lower,upper;
        if(!A::Outward(rounded,false,lower)||!A::Outward(rounded,true,upper))return {};
        if(first||lower<result.Lower)result.Lower=lower;
        if(first||upper>result.Upper)result.Upper=upper;
        first=false;
    }
    result.Known=true;return A::Valid(result)?result:I{};
}
// Derive initial search accuracy from the existing spatial quality, not the
// debug-only seconds lattice. The final point/normal widths and captured slack
// checks are unchanged. Failed geometry or quality never becomes a sample.
inline bool TryFixedProbeSpatialTimeGap(FVector start,FVector end,double pointWidth,
    double normalWidth,double radius,double& out)
{
    out=0;const auto before=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!ValidVector(start)||!ValidVector(end)||!NormalOrZero(pointWidth)||!(pointWidth>0)||
        !NormalOrZero(normalWidth)||!(normalWidth>0)||!NormalOrZero(radius)||!(radius>0))return false;
    I length=I::Exact(0.);
    for(int32 d=0;d<3;++d)
    {
        const I difference=A::Subtract(I::Exact(end[d]),I::Exact(start[d]));
        if(!A::Valid(difference))return false;
        const double lo=A::FromBits(A::Bits(difference.Lower)&A::Magnitude),hi=A::FromBits(A::Bits(difference.Upper)&A::Magnitude);
        length=A::Add(length,I::Exact(FMath::Max(lo,hi)));
        if(!A::Valid(length))return false;
    }
    if(start==end){out=1.;return true;}
    if(!(length.Lower>0))return false;
    const I point=A::Multiply(I::Exact(pointWidth),I::Exact(.125));
    const I normal=A::Multiply(A::Multiply(I::Exact(normalWidth),I::Exact(radius)),I::Exact(.125));
    const I pointGap=DividePositive(point,length),normalGap=DividePositive(normal,length);
    if(!A::Valid(pointGap)||!A::Valid(normalGap)||!(pointGap.Lower>0&&normalGap.Lower>0))return false;
    const double limit=FMath::Min(1.,FMath::Min(pointGap.Lower,normalGap.Lower));
    volatile double half=limit*.5;
    if(!NormalOrZero(half)||!(half>0&&half<=1.))return false;
    if(before.ControlFingerprint()!=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint())return false;
    out=half;return true;
}

inline I SqrtPositive(I value)
{
    if(!A::Valid(value)||!(value.Lower>0))return {};
    volatile double lower=FMath::Sqrt(value.Lower),upper=FMath::Sqrt(value.Upper);
    if(!NormalOrZero(lower)||!NormalOrZero(upper))return {};
    I result;
    if(!A::Outward(lower,false,result.Lower)||!A::Outward(upper,true,result.Upper))return {};
    result.Known=true;return A::Valid(result)&&result.Lower>0?result:I{};
}
inline bool BoundFirstNormalize(const P& input,P& output)
{
    output={};
    if(!Detail::ValidPoint(input))return false;
    const I squared=A::Add(A::Add(A::Square(input.Axis[0]),A::Square(input.Axis[1])),A::Square(input.Axis[2]));
    if(!A::Valid(squared)||!(squared.Lower>double(UE_SMALL_NUMBER)))return false;
    const I scale=DividePositive(I::Exact(1.),SqrtPositive(squared));
    if(!A::Valid(scale))return false;
    for(int32 d=0;d<3;++d)
    {output.Axis[d]=A::Multiply(input.Axis[d],scale);if(!A::Valid(output.Axis[d]))return false;}
    return true;
}
struct FUniqueQuantizedNormal
{
    FVector Quantized=FVector::ZeroVector;
    FVector ReNormalized=FVector::ZeroVector;
    uint64 NumericControl=0;
};
// A failure classification is passive. Only AmbiguousCell permits the caller
// to request tighter enclosures in its existing retained contact operation.
enum class EQuantizedNormalFailure:uint8
{None,Work,Input,RadialBounds,NormalizeBounds,ScalarNormalize,CellArithmetic,AmbiguousCell,DisjointCell,QuantizedMismatch,ConsumedNormalize,NumericContext};
struct FQuantizedNormalDiagnostic
{
    EQuantizedNormalFailure Failure=EQuantizedNormalFailure::None;
    int32 Axis=INDEX_NONE;
    I Radial,FirstNormalized,Shifted;
    double ActualFirst=0,Integer=0,Next=0;
};
// Preserve the existing strict native quantization cells. A scalar may lie
// outside the continuous normal box yet still share its UNIQUE discrete cell.
inline bool TryUniqueQuantizedNormal(const P& radialUnitNormal,FVector actualStoredNormal,
    FDistanceBudget& budget,FUniqueQuantizedNormal& out,FQuantizedNormalDiagnostic* diagnostic=nullptr)
{
    static_assert(std::is_same_v<decltype(FVector::ZeroVector.X),double>);
    out={};if(diagnostic)*diagnostic={};
    const auto failed=[&](EQuantizedNormalFailure failure)
    {if(diagnostic)diagnostic->Failure=failure;return false;};
    const auto before=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!budget.Consume())return failed(EQuantizedNormalFailure::Work);
    if(!ValidVector(actualStoredNormal))return failed(EQuantizedNormalFailure::Input);
    constexpr float stepFloat=1.e-3f;const double step=double(stepFloat);
    P converted;
    for(int32 d=0;d<3;++d)
    {
        if(diagnostic){diagnostic->Axis=d;diagnostic->Radial=radialUnitNormal.Axis[d];}
        if(!budget.Consume())return failed(EQuantizedNormalFailure::Work);
        if(!A::Valid(radialUnitNormal.Axis[d])||
            !A::Outward(radialUnitNormal.Axis[d].Lower,false,converted.Axis[d].Lower)||
            !A::Outward(radialUnitNormal.Axis[d].Upper,true,converted.Axis[d].Upper))return failed(EQuantizedNormalFailure::RadialBounds);
        converted.Axis[d].Known=true;
    }
    P firstNormalized;
    if(!BoundFirstNormalize(converted,firstNormalized))return failed(EQuantizedNormalFailure::NormalizeBounds);
    FVector actualFirst=actualStoredNormal;
    if(!actualFirst.Normalize()||!ValidVector(actualFirst))return failed(EQuantizedNormalFailure::ScalarNormalize);
    FVector fixedBeforeNormalize=FVector::ZeroVector;
    for(int32 d=0;d<3;++d)
    {
        if(diagnostic){diagnostic->Axis=d;diagnostic->Radial=radialUnitNormal.Axis[d];diagnostic->FirstNormalized=firstNormalized.Axis[d];diagnostic->ActualFirst=actualFirst[d];}
        if(!budget.Consume())return failed(EQuantizedNormalFailure::Work);
        const I shifted=A::Add(DividePositive(firstNormalized.Axis[d],I::Exact(step)),I::Exact(.5));
        if(diagnostic)diagnostic->Shifted=shifted;
        if(!A::Valid(shifted))return failed(EQuantizedNormalFailure::CellArithmetic);
        const double integer=FMath::RoundToFloat(actualFirst[d]/stepFloat),next=integer+1.;
        if(diagnostic){diagnostic->Integer=integer;diagnostic->Next=next;}
        if(!NormalOrZero(integer)||!NormalOrZero(next)||!(next>integer)||FMath::FloorToDouble(integer)!=integer)
            return failed(EQuantizedNormalFailure::CellArithmetic);
        if(!(integer<shifted.Lower&&shifted.Upper<next))
            return failed(shifted.Upper>integer&&shifted.Lower<next?EQuantizedNormalFailure::AmbiguousCell:EQuantizedNormalFailure::DisjointCell);
        fixedBeforeNormalize[d]=integer*stepFloat;
        if(!NormalOrZero(fixedBeforeNormalize[d]))return failed(EQuantizedNormalFailure::CellArithmetic);
    }
    FVector expected=fixedBeforeNormalize;
    if(!expected.Normalize()||!ValidVector(expected))return failed(EQuantizedNormalFailure::ScalarNormalize);
    const FVector actual=Speed::QuantizeUnitNormal(actualStoredNormal,stepFloat);
    if(!ValidVector(actual)||!SameBits(actual,expected))return failed(EQuantizedNormalFailure::QuantizedMismatch);
    const FVector consumed=actual.GetSafeNormal();
    if(!ValidVector(consumed)||consumed.IsZero())return failed(EQuantizedNormalFailure::ConsumedNormalize);
    const auto after=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(before.ControlFingerprint()!=after.ControlFingerprint())return failed(EQuantizedNormalFailure::NumericContext);
    if(diagnostic){diagnostic->Axis=INDEX_NONE;diagnostic->Failure=EQuantizedNormalFailure::None;}
    out={actual,consumed,before.ControlFingerprint()};return true;
}
}
