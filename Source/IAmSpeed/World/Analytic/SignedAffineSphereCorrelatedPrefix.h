#pragma once
#include "SignedAffineSphereResidual.h"
#include "KinematicCenterPathBounds.h"

namespace Speed::Analytic::Certified::SignedAffineSphereCorrelatedPrefix
{
namespace A=::AnalyticCertified;
namespace S=SignedAffineSphereResidual;
using E=S::EDecision;
using I=A::Interval;

// Value kernel for an authenticated affine product and the already bound
// zero-offset SBox::AdvancePosition DAG. This is a separation certificate
// only. It never changes the contact interval or the rounded consumed time.
struct FCenterNet
{
    I Axis[2][3];
    I TravelOrigin;
    I TwiceRadius;
    int32 FreeAxis=INDEX_NONE;
};

inline bool BuildCenterNet(const FVector3d& position,const FVector3d& velocity,const FVector3d& acceleration,
    float last,double radius,int32 freeAxis,FCenterNet& out,const FVector3d* chordError=nullptr)
{
    out={};
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    const uint32 bits=KinematicCenterDetail::Bits(last);
    if((bits&KinematicCenterDetail::FloatSign)||
        (bits&&bits<KinematicCenterDetail::FloatMinNormal)||!(last>=0.f&&last<=1.f)||
        !A::Finite(radius)||!(radius>0)||freeAxis<0||freeAxis>=2)return false;
    const int32 travel=1-freeAxis;
    const auto magnitude=[](double x){return I::Exact(A::FromBits(A::Bits(x)&A::Magnitude));};
    const auto t=I::Exact(double(last)),t2=A::Square(t),half=I::Exact(.5);
    // One full binary32 ULP bounds RNE and one minimum-normal allowance
    // covers flushing a subnormal square. t is actual nonnegative float.
    const auto squareError=A::Add(A::Multiply(t2,I::Exact(0x1p-23)),I::Exact(0x1p-126));
    FCenterNet result;
    result.FreeAxis=freeAxis;
    result.TravelOrigin=I::Exact(position[travel]);
    result.TwiceRadius=A::Multiply(I::Exact(2),I::Exact(radius));
    if(!A::Valid(squareError)||!A::Valid(result.TwiceRadius))return false;
    for(int32 d=0;d<2;++d)
    {
        const int32 axis=d?2:travel;
        const auto base=I::Exact(position[axis]),v=I::Exact(velocity[axis]),a=I::Exact(acceleration[axis]);
        const auto offset=d?I::Exact(radius):result.TravelOrigin;
        const auto p=A::Subtract(base,offset);
        const auto linear=A::Multiply(v,t),ha=A::Multiply(a,half);
        // Error propagation for at most five binary64 multiplies/adds on
        // this DAG, with t<=1. Bound |every exact intermediate| by this
        // nonnegative sum. Eight epsilons cover roundoff and propagation;
        // a 2^-1000 absolute floor also covers FTZ/DAZ intermediates/inputs.
        // Fusing either multiply/add can only reduce that rounding count.
        const auto halfAcceleration=A::Multiply(half,magnitude(acceleration[axis]));
        const auto scale=A::Add(A::Add(magnitude(position[axis]),A::Multiply(magnitude(velocity[axis]),t)),
            A::Multiply(halfAcceleration,A::Add(t2,squareError)));
        auto error=A::Add(A::Multiply(halfAcceleration,squareError),
            A::Add(A::Multiply(I::Exact(0x1p-49),scale),I::Exact(0x1p-1000)));
        if(chordError)
        {
            if(!A::Finite((*chordError)[axis])||(*chordError)[axis]<0)return false;
            error=A::Add(error,I::Exact((*chordError)[axis]));
        }
        if(!A::Valid(error)||!(error.Upper>0))return false;
        const I errorBox{-error.Upper,error.Upper,true};
        result.Axis[d][0]=A::Add(p,errorBox);
        result.Axis[d][1]=A::Add(A::Add(p,A::Multiply(half,linear)),errorBox);
        result.Axis[d][2]=A::Add(A::Add(A::Add(p,linear),A::Multiply(ha,t2)),errorBox);
        for(const auto& control:result.Axis[d])if(!A::Valid(control))return false;
    }
    out=result;return true;
}

inline bool BuildReferenceCenterNet(const FIntervalBezierCenterPath& path,const FVector3d& position,
    const FVector3d& velocity,const FVector3d& acceleration,float horizon,double radius,int32 freeAxis,FCenterNet& out)
{
    out={};
    if(path.Degree!=2||!CenterDetail::Valid(path)||!A::Finite(radius)||!(radius>0)||freeAxis<0||freeAxis>=2)return false;
    FZeroOffsetKinematicCenterReference rebuilt;
    if(!BuildZeroOffsetKinematicCenterReference(position,velocity,acceleration,horizon,
        FVector3d::ZeroVector,FVector3d::ZeroVector,rebuilt))return false;
    const auto& original=rebuilt.GetReferenceCurve();
    for(int32 i=0;i<3;++i)for(int32 d=0;d<3;++d)
    {
        const auto expected=path.ControlPoints[i].Axis[d],actual=original.ControlPoints[i].Axis[d];
        if(A::Bits(expected.Lower)!=A::Bits(actual.Lower)||A::Bits(expected.Upper)!=A::Bits(actual.Upper))return false;
    }
    const int32 travel=1-freeAxis;
    const auto delta=I::Exact(double(horizon)),delta2=A::Square(delta),half=I::Exact(.5);
    FCenterNet result;
    result.FreeAxis=freeAxis;
    result.TravelOrigin=I::Exact(position[travel]);
    result.TwiceRadius=A::Multiply(I::Exact(2),I::Exact(radius));
    if(!A::Valid(result.TwiceRadius)||!A::Valid(delta2))return false;
    for(int32 d=0;d<2;++d)
    {
        const int32 axis=d?2:travel;
        // Same quadratic Bernstein controls as the bit-bound path above.
        // Translate the constant before adding its motion terms, so their
        // uncertainty does not inherit the large absolute world coordinate.
        const auto base=d?A::Subtract(I::Exact(position[axis]),I::Exact(radius)):I::Exact(0);
        const auto linear=A::Multiply(I::Exact(velocity[axis]),delta);
        const auto quadratic=A::Multiply(A::Multiply(half,I::Exact(acceleration[axis])),delta2);
        result.Axis[d][0]=base;
        result.Axis[d][1]=A::Add(base,A::Multiply(half,linear));
        result.Axis[d][2]=A::Add(A::Add(base,linear),quadratic);
        for(const auto& control:result.Axis[d])if(!A::Valid(control))return false;
    }
    out=result;return true;
}

namespace DetailJoint
{
template<int32 N,int32 M>
bool DoubleDegree(const I (&net)[N],const int32 (&choose)[N],const int32 (&chooseDouble)[M],
    FDistanceBudget& budget,I (&elevated)[M],I (&square)[M])
{
    static_assert(M==2*N-1);
    for(int32 k=0;k<M;++k)elevated[k]=square[k]=I::Exact(0);
    for(int32 i=0;i<N;++i)for(int32 j=0;j<N;++j)
    {
        if(!budget.Consume())return false;
        const int32 k=i+j;
        I weight;
        if(!ContactDetail::ProjectionDividePositive(I::Exact(choose[i]*choose[j]),I::Exact(chooseDouble[k]),weight))
            return false;
        const auto weighted=A::Multiply(weight,net[i]);
        elevated[k]=A::Add(elevated[k],weighted);
        square[k]=A::Add(square[k],A::Multiply(weighted,net[j]));
        if(!A::Valid(elevated[k])||!A::Valid(square[k]))return false;
    }
    return true;
}

struct FNode
{
    I Net[5][11];
    int32 TimeCount=5,CurveCount=11,Depth=0;
};
struct FVariation {int32 MinimumEnd=INDEX_NONE;double Score=0;};
inline bool ScanAxis(const FNode& n,bool time,FVariation& out)
{
    out={};
    const int32 count=time?n.TimeCount:n.CurveCount,other=time?n.CurveCount:n.TimeCount;
    if(count<=1)return true;
    bool increasing=true,decreasing=true;
    double largest=0;
    const auto at=[&](int32 i,int32 j){return time?n.Net[i][j]:n.Net[j][i];};
    for(int32 i=1;i<count;++i)for(int32 j=0;j<other;++j)
    {
        const auto before=at(i-1,j),after=at(i,j);
        const auto delta=A::Subtract(after,before);
        if(!A::Valid(delta))return false;
        increasing&=before.Upper<=after.Lower;
        decreasing&=after.Upper<=before.Lower;
        largest=FMath::Max3(largest,FMath::Abs(delta.Lower),FMath::Abs(delta.Upper));
    }
    if(increasing||decreasing)out.MinimumEnd=increasing?0:count-1;
    out.Score=largest*double(count-1); // Ordering hint only.
    return A::Finite(out.Score);
}
inline void RestrictEnd(FNode& n,bool time,int32 end)
{
    if(time)
    {
        for(int32 j=0;j<n.CurveCount;++j)n.Net[0][j]=n.Net[end][j];
        n.TimeCount=1;
    }
    else
    {
        for(int32 i=0;i<n.TimeCount;++i)n.Net[i][0]=n.Net[i][end];
        n.CurveCount=1;
    }
}
inline bool Split(const FNode& source,bool time,FNode& left,FNode& right,const I* parameter=nullptr)
{
    const int32 count=time?source.TimeCount:source.CurveCount;
    const int32 other=time?source.CurveCount:source.TimeCount;
    if(count<2||source.Depth>=52)return false;
    left.TimeCount=right.TimeCount=source.TimeCount;
    left.CurveCount=right.CurveCount=source.CurveCount;
    left.Depth=right.Depth=source.Depth+1;
    for(int32 j=0;j<other;++j)
    {
        const auto get=[&](int32 i){return time?source.Net[i][j]:source.Net[j][i];};
        const auto put=[&](FNode& n,int32 i,I value){if(time)n.Net[i][j]=value;else n.Net[j][i]=value;};
        I work[11];
        for(int32 i=0;i<count;++i)work[i]=get(i);
        put(left,0,work[0]);put(right,count-1,work[count-1]);
        for(int32 level=1;level<count;++level)
        {
            for(int32 i=0;i<count-level;++i)
            {
                work[i]=parameter?A::Add(A::Multiply(A::Subtract(I::Exact(1),*parameter),work[i]),
                    A::Multiply(*parameter,work[i+1])):A::HalfSum(work[i],work[i+1]);
                if(!A::Valid(work[i]))return false;
            }
            put(left,level,work[0]);put(right,count-1-level,work[count-1-level]);
        }
    }
    return true;
}

// Both splits are de Casteljau restrictions of the same fixed residual.
// An outward enclosure of first/last is enough for the second split.
inline bool RestrictTime(const FNode& root,double first,double last,FDistanceBudget& budget,FNode& out)
{
    if(!A::Finite(first)||!A::Finite(last)||!(first>=0&&first<last&&last<=1))return false;
    auto retained=root;
    if(last<1)
    {
        const auto t=I::Exact(last);
        FNode discarded;
        if(!budget.Consume()||!Split(root,true,retained,discarded,&t))return false;
    }
    if(first>0)
    {
        I t;
        if(!ContactDetail::ProjectionDividePositive(I::Exact(first),I::Exact(last),t)||t.Lower<0||t.Upper>1)return false;
        FNode discarded;
        if(!budget.Consume()||!Split(retained,true,discarded,out,&t))return false;
    }
    else out=MoveTemp(retained);
    return true;
}

inline E BuildResidual(const AffineReferenceDetail::FProduct& product,const FCenterNet& center,
    FDistanceBudget& budget,FNode& root)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!S::Private::Admits(product)||product.FreeWorldAxis!=center.FreeAxis)return E::NotApplicable;
    if(!S::Private::ValidProduct(product))return E::Arithmetic;
    const int32 axis=1-center.FreeAxis;
    constexpr int32 choose2[]={1,2,1},choose4[]={1,4,6,4,1};
    constexpr int32 choose5[]={1,5,10,10,5,1},choose10[]={1,10,45,120,210,252,210,120,45,10,1};
    I time[2][5],timeSquared[2][5],curve[2][11],curveSquared[2][11];
    for(int32 d=0;d<2;++d)
    {
        I input[6];
        for(int32 j=0;j<6;++j)
        {
            const auto& point=product.Curve.ControlPoints[j];
            input[j]=d?point.Axis[2]:A::Subtract(point.Axis[axis],center.TravelOrigin);
        }
        if(!DoubleDegree(center.Axis[d],choose2,choose4,budget,time[d],timeSquared[d])||
            !DoubleDegree(input,choose5,choose10,budget,curve[d],curveSquared[d]))
            return budget.Remaining<=budget.ConsumptionFloor?E::Work:E::Arithmetic;
    }
    I timeOnly[5],curveOnly[11];
    for(int32 i=0;i<5;++i)timeOnly[i]=A::Add(A::Add(timeSquared[0][i],timeSquared[1][i]),
        A::Multiply(center.TwiceRadius,time[1][i]));
    for(int32 j=0;j<11;++j)curveOnly[j]=A::Subtract(A::Add(curveSquared[0][j],curveSquared[1][j]),
        A::Multiply(center.TwiceRadius,curve[1][j]));
    // u=center travel, v=curve travel, both translated by the same origin.
    // z=center height-Rs and w=curve height. Then
    // F=(u-v)^2+(z-w)^2+2*Rs*(z-w), without subtracting two large squares.
    // Pure-time and pure-curve terms need only univariate products.
    for(int32 i=0;i<5;++i)for(int32 j=0;j<11;++j)
    {
        if(!budget.Consume())return E::Work;
        const auto cross=A::Add(A::Multiply(time[0][i],curve[0][j]),A::Multiply(time[1][i],curve[1][j]));
        root.Net[i][j]=A::Subtract(A::Add(timeOnly[i],curveOnly[j]),A::Multiply(I::Exact(2),cross));
        if(!A::Valid(root.Net[i][j]))return E::Arithmetic;
    }
    return E::Clear; // Construction succeeded. This is not a separation result.
}

inline bool PositiveCoefficients(const FNode& cell)
{
    for(int32 i=0;i<cell.TimeCount;++i)
        for(int32 j=0;j<cell.CurveCount;++j)
            if(!(cell.Net[i][j].Lower>0))return false;
    return true;
}

inline E ExcludeResidual(FNode&& root,FDistanceBudget& budget,bool stopAtNegativeCorner=false)
{
    TArray<FNode> pending;pending.Reserve(16);pending.Add(MoveTemp(root));
    for(int32 visited=0;!pending.IsEmpty();++visited)
    {
        if(visited>=128)return E::Indeterminate;
        if(!budget.Consume())return E::Work;
        auto cell=pending.Pop(EAllowShrinking::No);
        if(PositiveCoefficients(cell))continue;
        if(stopAtNegativeCorner)
        {
            // A negative corner disproves universal separation. It never
            // publishes an inside witness or a contact from the residual net.
            const int32 t=cell.TimeCount-1,v=cell.CurveCount-1;
            if(cell.Net[0][0].Upper<0||cell.Net[t][0].Upper<0||
                cell.Net[0][v].Upper<0||cell.Net[t][v].Upper<0)return E::Indeterminate;
        }
        FVariation timeDirection,curveDirection;
        if(!ScanAxis(cell,true,timeDirection)||!ScanAxis(cell,false,curveDirection))return E::Arithmetic;
        if(timeDirection.MinimumEnd!=INDEX_NONE)
        {RestrictEnd(cell,true,timeDirection.MinimumEnd);pending.Add(MoveTemp(cell));continue;}
        if(curveDirection.MinimumEnd!=INDEX_NONE)
        {RestrictEnd(cell,false,curveDirection.MinimumEnd);pending.Add(MoveTemp(cell));continue;}
        if(cell.Depth>=52||(cell.TimeCount==1&&cell.CurveCount==1))return E::Indeterminate;
        const bool splitTime=cell.TimeCount>1&&(cell.CurveCount==1||timeDirection.Score>=curveDirection.Score);
        FNode left,right;
        if(!Split(cell,splitTime,left,right))return E::Arithmetic;
        pending.Add(MoveTemp(right));pending.Add(MoveTemp(left));
    }
    return E::Clear;
}
}

inline E Exclude(const AffineReferenceDetail::FProduct& product,const FCenterNet& center,FDistanceBudget& budget)
{
    DetailJoint::FNode root;
    const auto built=DetailJoint::BuildResidual(product,center,budget,root);
    if(built!=E::Clear)return built;
    return DetailJoint::ExcludeResidual(MoveTemp(root),budget);
}
}
