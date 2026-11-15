#pragma once
#include "YawFloorBoxSweepBounds.h"

namespace Speed::Analytic::Certified
{
// PROPOSAL ONLY, not installed/compiled/admitted. Pure geometric family:
// the Cartesian product of the real COM segment and raw/normalized quaternion
// chord. This is not a claim that a setter executes an integrator or a chord.
// A real setter ticket must bind both operands, actual assignment, phase,
// snapshot, all shapes/configurations and the subsequent real child sync.
struct FBoxPoseCorridorInput
{
    FVector BeforeCOM=FVector::ZeroVector,AfterCOM=FVector::ZeroVector;
    FQuat BeforeRotation=FQuat::Identity,AfterRotation=FQuat::Identity;
    FVector LocalCOM=FVector::ZeroVector,LocalOffset=FVector::ZeroVector;
    FQuat LocalRotation=FQuat::Identity;
    FVector PhysicalHalfExtent=FVector::ZeroVector,QueryHalfExtent=FVector::ZeroVector;
    double DomainShell=0,OverlapShell=0;
};

inline bool BuildBoxPoseCorridorBounds(const FBoxPoseCorridorInput& v,FIntervalPoint& output,
    TArray<FIntervalPoint,TInlineAllocator<16>>* retainedCorners=nullptr)
{
    output={};if(retainedCorners)retainedCorners->Reset();
    TArray<FIntervalPoint,TInlineAllocator<16>> corners;
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    using namespace YawFloorBoxDetail;
    using namespace GeneralBoxDetail;
    if(v.BeforeCOM.ContainsNaN()||v.AfterCOM.ContainsNaN()||
        v.BeforeRotation.ContainsNaN()||!v.BeforeRotation.IsNormalized()||
        v.AfterRotation.ContainsNaN()||!v.AfterRotation.IsNormalized()||
        v.LocalRotation.ContainsNaN()||!v.LocalRotation.IsNormalized()||
        v.LocalCOM.ContainsNaN()||v.LocalOffset.ContainsNaN()||
        v.PhysicalHalfExtent.ContainsNaN()||v.QueryHalfExtent.ContainsNaN()||
        v.PhysicalHalfExtent.GetMin()<=0||v.QueryHalfExtent.GetMin()<=0)return false;
    FQuaternion raw;
    const double before[]={v.BeforeRotation.X,v.BeforeRotation.Y,v.BeforeRotation.Z,v.BeforeRotation.W};
    const double after[]={v.AfterRotation.X,v.AfterRotation.Y,v.AfterRotation.Z,v.AfterRotation.W};
    for(int32 k=0;k<4;++k)
    {
        const auto range=GeneralBoxDetail::Hull(Interval::Exact(before[k]),Interval::Exact(after[k]));
        if(!AnalyticCertified::Valid(range))return false;
        raw.Axis[k]={range,0};
    }
    // Includes raw endpoints; retain the existing normalization-domain refusal.
    // In particular, never flip an actually assigned quaternion sign to hide
    // a chord containing zero or widen/clip the accepted normalization domain.
    FQuaternion ownerRotation;
    if(!NormalizedFamily(raw,v.BeforeRotation,ownerRotation))return false;
    FPoint ownerCenter;
    for(int32 d=0;d<3;++d)
    {
        const auto range=GeneralBoxDetail::Hull(Interval::Exact(v.BeforeCOM[d]),Interval::Exact(v.AfterCOM[d]));
        if(!AnalyticCertified::Valid(range))return false;
        ownerCenter.Axis[d]={range,0};
    }
    // Preserve both native rotates and their arithmetic errors. Replacing
    // them by Rotate(LocalOffset-LocalCOM) would change the consumed DAG.
    const auto rotatedCOM=Rotate(ownerRotation,v.LocalCOM);
    const auto rotatedOffset=Rotate(ownerRotation,v.LocalOffset);
    FPoint center;
    for(int32 d=0;d<3;++d)
        center.Axis[d]=Sum(Difference(ownerCenter.Axis[d],rotatedCOM.Axis[d]),rotatedOffset.Axis[d]);
    const auto bodyRotation=Hamilton(ownerRotation,Quaternion(v.LocalRotation));
    FIntervalPoint result;
    for(const auto& extent:{v.PhysicalHalfExtent,v.QueryHalfExtent})for(int32 corner=0;corner<8;++corner)
    {
        FVector local;
        for(int32 d=0;d<3;++d)local[d]=(corner&(1<<d))?extent[d]:-extent[d];
        const auto rotated=Rotate(bodyRotation,local);FPoint point;
        for(int32 d=0;d<3;++d)point.Axis[d]=Sum(center.Axis[d],rotated.Axis[d]);
        const auto values=PointRanges(point);
        if(!YawFloorBoxDetail::Join(result,values))return false;
        if(retainedCorners)corners.Add(values);
    }
    // Same real query shells, never a modified radius/extent/quality budget.
    for(double shell:{v.DomainShell,v.OverlapShell})
    {
        if(!AnalyticCertified::Valid(Interval::Exact(shell))||shell<0)return false;
        for(int32 d=0;d<3;++d)
        {
            result.Axis[d]=AnalyticCertified::Add(result.Axis[d],Interval{-shell,shell,true});
            if(retainedCorners)for(auto& corner:corners)
            {
                corner.Axis[d]=AnalyticCertified::Add(corner.Axis[d],Interval{-shell,shell,true});
                if(!AnalyticCertified::Valid(corner.Axis[d]))return false;
            }
        }
    }
    if(!Detail::ValidPoint(result))return false;
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    output=result;if(retainedCorners)*retainedCorners=MoveTemp(corners);return true;
}

// Only containment in EACH exact retained primitive envelope already proved
// strictly clear of the complete union AND Landscape can grant a caller's
// geometry subclaim. This function itself produces no such grant or token.
inline bool BoxPoseCorridorWithin(const FIntervalPoint& outer,const FIntervalPoint& corridor)
{
    if(!Detail::ValidPoint(outer)||!Detail::ValidPoint(corridor))return false;
    for(int32 d=0;d<3;++d)
        if(corridor.Axis[d].Lower<outer.Axis[d].Lower||corridor.Axis[d].Upper>outer.Axis[d].Upper)return false;
    return true;
}
}
