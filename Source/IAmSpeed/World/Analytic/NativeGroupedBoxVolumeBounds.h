#pragma once
#include "GroupedBoxVolumeBounds.h"
#include "BoxPoseCorridorBounds.h"
#include "GeneralBoxGlobalTimeBounds.h"

namespace Speed::Analytic::Certified
{
namespace NativeGroupedBoxDetail
{
// Bound each SAME indexed corner on a closed cover of ORIGINAL seconds,
// then hull that corner only. Never rebase the owner or mix vertex roles.
// The returned group remains one complete convex Box family for the whole
// requested interval; this does not declare the old coarser volume clear.
inline bool ContractedGlobalMotionPiece(const FGeneralBoxMotionInput& input,float first,float last,
    FDistanceBudget& budget,FGeneralBoxMotionBounds& output,bool antipodal=false)
{
    output={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!GeneralBoxGlobalTimeDetail::ValidFloatSecondsRange(first,last)||last>input.Horizon)return false;
    struct FCell {float First,Last;int32 Depth;};
    TArray<FCell,TInlineAllocator<4>> stack;stack.Add(FCell{first,last,0});
    FGeneralBoxMotionBounds result;bool have=false;
    while(!stack.IsEmpty())
    {
        // Every node is prepaid, including each actual interval-model build.
        if(!budget.Consume())return false;
        const auto cell=stack.Pop(EAllowShrinking::No);
        const float middle=cell.First+.5f*(cell.Last-cell.First);
        if(cell.Depth<2&&middle>cell.First&&middle<cell.Last&&
            GeneralBoxGlobalTimeDetail::ValidFloatSecondsRange(cell.First,middle)&&
            GeneralBoxGlobalTimeDetail::ValidFloatSecondsRange(middle,cell.Last))
        {
            stack.Add(FCell{middle,cell.Last,cell.Depth+1});
            stack.Add(FCell{cell.First,middle,cell.Depth+1});continue;
        }
        // Adjacent/unsupported midpoint: retain this WHOLE closed cell.
        FGeneralBoxMotionBounds part;
        if(!BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(input,cell.First,cell.Last,part,antipodal)||
            part.Corners.Num()!=32||!Detail::ValidPoint(part.Hull)||
            !Detail::ValidPoint(part.NativeCenter)||!Detail::ValidPoint(part.ReferenceCenter))return false;
        if(!have){result=MoveTemp(part);have=true;continue;}
        if(!budget.Consume())return false;
        result.Hull=TimeDetail::Hull(result.Hull,part.Hull);
        if(!budget.Consume())return false;
        result.NativeCenter=TimeDetail::Hull(result.NativeCenter,part.NativeCenter);
        if(!budget.Consume())return false;
        result.ReferenceCenter=TimeDetail::Hull(result.ReferenceCenter,part.ReferenceCenter);
        for(int32 k=0;k<32;++k)
        {if(!budget.Consume())return false;result.Corners[k]=TimeDetail::Hull(result.Corners[k],part.Corners[k]);}
    }
    if(!have)return false;output=MoveTemp(result);return true;
}
// This factory retains the ORIGINAL input and H in both closed cells.
// It never integrates, rebases the owner or changes a captured World query.
inline bool GlobalMotionPieces(const FGeneralBoxMotionInput& input,FDistanceBudget& budget,
    TArray<FGeneralBoxMotionBounds,TInlineAllocator<2>>& output,bool antipodal=false)
{
    output.Reset();RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    const float middle=.5f*input.Horizon;
    const bool split=middle>0&&middle<input.Horizon&&
        GeneralBoxGlobalTimeDetail::ValidFloatSecondsRange(0.f,middle)&&
        GeneralBoxGlobalTimeDetail::ValidFloatSecondsRange(middle,input.Horizon);
    TArray<FGeneralBoxMotionBounds,TInlineAllocator<2>> result;
    if(split)
    {
        for(const auto& cell:{TPair<float,float>(0.f,middle),TPair<float,float>(middle,input.Horizon)})
        {
            FGeneralBoxMotionBounds part;
            if(!ContractedGlobalMotionPiece(input,cell.Key,cell.Value,budget,part,antipodal))return false;
            result.Add(MoveTemp(part));
        }
    }
    else
    {
        // A horizon with no supported interior float midpoint retains the
        // original whole-time theorem and all of its refusal conditions.
        FGeneralBoxMotionBounds whole;if(!budget.Consume()||!BuildGeneralBoxMotionBounds(input,whole))return false;
        result.Add(MoveTemp(whole));
    }
    output=MoveTemp(result);return true;
}
// Same fixed-query center DAG and both existing Rotate/SAT representations,
// retained per corner BEFORE any whole-Box AABB join.
inline bool FixedQueryGroup(const FWorldQuery& query,FBoxVolumeGroup& output)
{
    output={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(query.Shape!=EQueryShape::Box||query.Start.ContainsNaN()||query.End.ContainsNaN()||
        query.Rotation.ContainsNaN()||!query.Rotation.IsNormalized()||
        query.HalfExtent.ContainsNaN()||query.HalfExtent.GetMin()<=0)return false;
    FIntervalPoint center;
    for(int32 d=0;d<3;++d)center.Axis[d]=AnalyticCertified::Add(Interval::Exact(query.Start[d]),
        AnalyticCertified::Multiply(AnalyticCertified::Subtract(Interval::Exact(query.End[d]),
            Interval::Exact(query.Start[d])),Interval{0,1,true}));
    if(!Detail::ValidPoint(center))return false;
    Private::FBoxSweepContext context;context.Initialize(query);
    FBoxVolumeGroup result;
    for(int32 vertex=0;vertex<8;++vertex)
    {
        FVector3d corner;
        for(int32 d=0;d<3;++d)corner[d]=(vertex&(1<<d))?query.HalfExtent[d]:-query.HalfExtent[d];
        auto rotated=FixedQueryBoundsDetail::Rotate(query.Rotation,corner);
        FIntervalPoint axes=Detail::Point(FVector3d::ZeroVector);
        for(int32 d=0;d<3;++d)for(int32 k=0;k<3;++k)axes.Axis[d]=AnalyticCertified::Add(axes.Axis[d],
            AnalyticCertified::Multiply(Interval::Exact(context.Axis[k][d]),Interval::Exact(corner[k])));
        if(!Detail::ValidPoint(rotated)||!Detail::ValidPoint(axes))return false;
        const auto offset=TimeDetail::Hull(rotated,axes);FIntervalPoint point;
        for(int32 d=0;d<3;++d)point.Axis[d]=AnalyticCertified::Add(center.Axis[d],offset.Axis[d]);
        if(!Detail::ValidPoint(point))return false;result.Corners.Add(point);
    }
    output=MoveTemp(result);return true;
}
inline bool Inflate(TArray<FBoxVolumeGroup,TInlineAllocator<16>>& groups,double domainShell,double overlapShell)
{
    for(double shell:{domainShell,overlapShell})
    {
        const auto value=Interval::Exact(shell);
        if(!AnalyticCertified::Valid(value)||value.Lower<0)return false;
        for(auto& group:groups)for(auto& point:group.Corners)for(int32 d=0;d<3;++d)
        {
            point.Axis[d]=AnalyticCertified::Add(point.Axis[d],Interval{-shell,shell,true});
            if(!AnalyticCertified::Valid(point.Axis[d]))return false;
        }
    }
    return true;
}
inline bool Close(FIntervalPoint hull,TArray<FBoxVolumeGroup,TInlineAllocator<16>>& groups,
    const FQuat4d& rotation,double domainShell,double overlapShell,FDistanceBudget& budget,FGroupedBoxVolumeBounds& output,
    FClosedGroupedBoxVolumePtr* closedOutput=nullptr)
{
    output={};if(closedOutput)closedOutput->Reset();
    if(rotation.ContainsNaN()||!rotation.IsNormalized()||!Inflate(groups,domainShell,overlapShell))return false;
    // Fixed choices only. No unit-vector or certified-normal claim is used.
    TArray<FVector3d,TInlineAllocator<8>> axes;
    for(const FVector3d axis:{FVector3d::ForwardVector,FVector3d::RightVector,FVector3d::UpVector})axes.Add(axis);
    for(const FVector3d axis:{FVector3d::ForwardVector,FVector3d::RightVector,FVector3d::UpVector})axes.Add(rotation.RotateVector(axis));
    if(!closedOutput)return CloseGroupedBoxVolumeBounds(hull,groups,axes,budget,output);
    if(!FClosedGroupedBoxVolume::Build(hull,groups,axes,budget,*closedOutput))return false;
    output=(*closedOutput)->GetVolumeForInspection();return true;
}
inline bool MotionGroups(const FGeneralBoxMotionBounds& bounds,TArray<FBoxVolumeGroup,TInlineAllocator<16>>& groups)
{
    if(bounds.Corners.Num()!=32)return false;
    // consumed/reference x physical/query x all8 corners, in producer order.
    for(int32 begin=0;begin<32;begin+=8)
    {
        FBoxVolumeGroup group;
        for(int32 k=0;k<8;++k)group.Corners.Add(bounds.Corners[begin+k]);
        groups.Add(MoveTemp(group));
    }
    return true;
}
// Insert inside NativeGroupedBoxDetail, after MotionGroups.
// Whole was built once from the same input by the native value factory.
inline bool CoarsePrimitiveFromWhole(const FGeneralBoxMotionBounds& whole,const FQuat4d& rotation,
    double domainShell,double overlapShell,FDistanceBudget& budget,FGroupedBoxVolumeBounds& output)
{
    output={};TArray<FBoxVolumeGroup,TInlineAllocator<16>> groups;
    if(!MotionGroups(whole,groups)||groups.Num()!=4)return false;
    return Close(whole.Hull,groups,rotation,domainShell,overlapShell,budget,output);
}
inline bool RefinedPrimitiveFromWhole(const FGeneralBoxMotionInput& input,const FGeneralBoxMotionBounds& whole,
    const FQuat4d& rotation,double domainShell,double overlapShell,FDistanceBudget& budget,FGroupedBoxVolumeBounds& output)
{
    output={};TArray<FGeneralBoxMotionBounds,TInlineAllocator<2>> pieces;
    if(!GlobalMotionPieces(input,budget,pieces))return false;
    TArray<FBoxVolumeGroup,TInlineAllocator<16>> groups;
    for(const auto& part:pieces)if(!MotionGroups(part,groups))return false;
    if(groups.Num()!=pieces.Num()*4||pieces.IsEmpty()||pieces.Num()>2)return false;
    return Close(whole.Hull,groups,rotation,domainShell,overlapShell,budget,output);
}
}
inline bool BuildGeneralNativeBoxGroupedVolume(const PreparedExecution::FNativeSolidStaticValues& v,
    FDistanceBudget& budget,FGroupedBoxVolumeBounds& output,const TCHAR** observedStage=nullptr,bool antipodal=false,
    FClosedGroupedBoxVolumePtr* closedOutput=nullptr)
{
    const auto stage=[&](const TCHAR* name){if(observedStage)*observedStage=name;};
    stage(TEXT("native_sweep_input"));
    output={};if(closedOutput)closedOutput->Reset();FIntervalPoint originalHull;
    // Retain every original capture/shape/transform/chord/FP validation.
    if(!BuildGeneralNativeBoxSweepVolume(v,originalHull))return false;
    FGeneralBoxMotionInput input;input.Owner=v.OwnerState;input.Horizon=v.Horizon;
    input.LocalCOM=v.LocalCOM;input.LocalOffset=v.LocalOffset;input.LocalRotation=v.LocalRotation;
    input.PhysicalHalfExtent=v.PhysicalHalfExtent;input.QueryHalfExtent=v.QueryHalfExtent;
    stage(TEXT("global_motion_pieces"));TArray<FGeneralBoxMotionBounds,TInlineAllocator<2>> pieces;
    if(!NativeGroupedBoxDetail::GlobalMotionPieces(input,budget,pieces,antipodal))return false;
    TArray<FBoxVolumeGroup,TInlineAllocator<16>> groups;
    stage(TEXT("motion_groups"));for(const auto& part:pieces)
        if(!NativeGroupedBoxDetail::MotionGroups(part,groups))return false;
    stage(TEXT("fixed_query_chord"));FBoxVolumeGroup chord;
    if(!NativeGroupedBoxDetail::FixedQueryGroup(v.Query,chord))return false;
    groups.Add(MoveTemp(chord));
    auto pose=v.Query;pose.Start=pose.End=v.BodyState.Location;pose.HalfExtent=v.PhysicalHalfExtent;
    stage(TEXT("physical_pose"));FBoxVolumeGroup physicalPose;
    if(!NativeGroupedBoxDetail::FixedQueryGroup(pose,physicalPose))return false;
    groups.Add(MoveTemp(physicalPose));
    auto footprintQuery=v.Query;footprintQuery.Start=footprintQuery.End=FVector3d::ZeroVector;
    stage(TEXT("footprint"));FBoxVolumeGroup footprint;
    if(!NativeGroupedBoxDetail::FixedQueryGroup(footprintQuery,footprint))return false;
    stage(TEXT("center_footprint_groups"));for(const auto& part:pieces)
    for(const auto& center:{part.NativeCenter,part.ReferenceCenter})
    {
        FBoxVolumeGroup group;
        for(const auto& corner:footprint.Corners)
        {
            FIntervalPoint point;
            for(int32 d=0;d<3;++d)point.Axis[d]=AnalyticCertified::Add(center.Axis[d],corner.Axis[d]);
            if(!Detail::ValidPoint(point))return false;group.Corners.Add(point);
        }
        groups.Add(MoveTemp(group));
    }
    stage(TEXT("group_count"));if(groups.Num()!=pieces.Num()*6+2||pieces.IsEmpty()||pieces.Num()>2)return false;
    stage(TEXT("close_grouped_volume"));return NativeGroupedBoxDetail::Close(originalHull,groups,v.Query.Rotation,
        v.Query.DomainTolerance,v.Query.InitialOverlapTolerance,budget,output,closedOutput);
}
inline bool BuildGeneralBoxPrimitiveGroupedVolume(const FGeneralBoxMotionInput& input,const FQuat4d& axesRotation,
    double domainShell,double overlapShell,FDistanceBudget& budget,FGroupedBoxVolumeBounds& output)
{
    output={};FGeneralBoxMotionBounds motion;
    if(!BuildGeneralBoxMotionBounds(input,motion))return false;
    return NativeGroupedBoxDetail::RefinedPrimitiveFromWhole(input,motion,axesRotation,domainShell,overlapShell,budget,output);
}
inline bool BuildBoxPoseCorridorGroupedVolume(const FBoxPoseCorridorInput& input,FDistanceBudget& budget,
    FGroupedBoxVolumeBounds& output)
{
    output={};FIntervalPoint hull;
    TArray<FIntervalPoint,TInlineAllocator<16>> corners;
    if(!BuildBoxPoseCorridorBounds(input,hull,&corners)||corners.Num()!=16)return false;
    TArray<FBoxVolumeGroup,TInlineAllocator<16>> groups;
    for(int32 begin=0;begin<16;begin+=8)
    {
        FBoxVolumeGroup group;
        for(int32 k=0;k<8;++k)group.Corners.Add(corners[begin+k]);
        groups.Add(MoveTemp(group));
    }
    // Corridor producer has ALREADY inflated these corner boxes and hull.
    const FQuat axesRotation=input.BeforeRotation*input.LocalRotation;
    return NativeGroupedBoxDetail::Close(hull,groups,axesRotation,0,0,budget,output);
}
inline bool GroupedBoxProofContainsEndpoint(const FReferenceBoxVolumeStrictSeparationPtr& proof,
    FWorldQuery pose,const FVector3d& physicalExtent,const FVector3d& queryExtent)
{
    if(!proof)return false;
    for(const auto& extent:{physicalExtent,queryExtent})
    {
        pose.HalfExtent=extent;FBoxVolumeGroup actual;
        if(!NativeGroupedBoxDetail::FixedQueryGroup(pose,actual)||
            !proof->ContainsCompleteEndpointGroup(actual.Corners))return false;
    }
    return true;
}
}
