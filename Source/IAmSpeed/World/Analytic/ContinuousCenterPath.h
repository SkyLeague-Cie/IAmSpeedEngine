#pragma once
#include "TensorDistanceBounds.h"
namespace Speed::Analytic::Certified
{
// One FIXED continuous polynomial center curve is enclosed by these coefficients.
// Not an independently rounded Lerp, Integrate pose, rotating offset or time cast.
struct FIntervalBezierCenterPath
{
    int32 Degree=0;
    TArray<FIntervalPoint,TInlineAllocator<3>> ControlPoints;
};
namespace CenterDetail
{
inline bool Valid(const FIntervalBezierCenterPath& path)
{
    if(path.Degree<0||path.Degree>2||path.ControlPoints.Num()!=path.Degree+1)return false;
    for(const auto& p:path.ControlPoints)if(!Detail::ValidPoint(p))return false;return true;
}
inline bool Linear(const FVector3d& start,const FVector3d& end,FIntervalBezierCenterPath& out)
{
    out={};FIntervalBezierCenterPath path;path.Degree=1;path.ControlPoints={Detail::Point(start),Detail::Point(end)};
    if(!Valid(path))return false;out=MoveTemp(path);return true;
}
inline FIntervalPoint Hull(const FIntervalBezierCenterPath& path)
{
    if(!Valid(path))return {};
    auto p=path.ControlPoints[0];
    for(const auto& q:path.ControlPoints)for(int32 d=0;d<3;++d)
    {p.Axis[d].Lower=FMath::Min(p.Axis[d].Lower,q.Axis[d].Lower);p.Axis[d].Upper=FMath::Max(p.Axis[d].Upper,q.Axis[d].Upper);}
    return p;
}
inline bool Split(const FIntervalBezierCenterPath& path,FIntervalBezierCenterPath& left,FIntervalBezierCenterPath& right)
{
    left={};right={};if(!Valid(path))return false;
    left.Degree=right.Degree=path.Degree;left.ControlPoints.SetNum(path.Degree+1);right.ControlPoints.SetNum(path.Degree+1);
    auto work=path.ControlPoints;left.ControlPoints[0]=work[0];right.ControlPoints[path.Degree]=work[path.Degree];
    for(int32 level=1;level<=path.Degree;++level)
    {
        for(int32 i=0;i<=path.Degree-level;++i)work[i]=Detail::Half(work[i],work[i+1]);
        left.ControlPoints[level]=work[0];right.ControlPoints[path.Degree-level]=work[path.Degree-level];
    }
    return Valid(left)&&Valid(right);
}
inline FIntervalPoint Evaluate(const FIntervalBezierCenterPath& path,Interval time)
{
    if(!Valid(path)||!AnalyticCertified::Valid(time)||time.Lower<0||time.Upper>1)return {};
    // The SAME fixed polynomial restricted to [a,b] has Bernstein controls
    // Q_i = blossom(a repeated degree-i, b repeated i). Each parameter here
    // is an exact binary64 endpoint, not a varying interval reused in (1-t)
    // and t. Outward arithmetic encloses each restricted coefficient; their
    // convex hull then encloses every real t in [a,b], including a==b.
    // No division, center translation, rounded Integrate or new curve is used.
    FIntervalBezierCenterPath restricted;restricted.Degree=path.Degree;
    restricted.ControlPoints.SetNum(path.Degree+1);
    const auto a=Interval::Exact(time.Lower),b=Interval::Exact(time.Upper);
    for(int32 coefficient=0;coefficient<=path.Degree;++coefficient)
    {
        auto work=path.ControlPoints;
        for(int32 level=0;level<path.Degree;++level)
        {
            const auto parameter=level<path.Degree-coefficient?a:b;
            const auto other=AnalyticCertified::Subtract(Interval::Exact(1),parameter);
            if(!AnalyticCertified::Valid(other))return {};
            for(int32 i=0;i<path.Degree-level;++i)for(int32 d=0;d<3;++d)
                work[i].Axis[d]=AnalyticCertified::Add(AnalyticCertified::Multiply(other,work[i].Axis[d]),
                    AnalyticCertified::Multiply(parameter,work[i+1].Axis[d]));
        }
        if(!Detail::ValidPoint(work[0]))return {};
        restricted.ControlPoints[coefficient]=work[0];
    }
    return Hull(restricted);
}
}
}
