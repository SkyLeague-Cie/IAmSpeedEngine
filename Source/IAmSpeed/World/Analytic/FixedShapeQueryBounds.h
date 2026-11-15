#pragma once
#include "IndexedIntervalUnion.h"
#include "AnalyticWorldQuery.h"
#include "AnalyticBoxSweepContext.h"
namespace Speed::Analytic::Certified
{
namespace FixedQueryBoundsDetail
{
inline FIntervalPoint Cross(const FIntervalPoint& a,const FIntervalPoint& b)
{
    FIntervalPoint out;
    for(int32 d=0;d<3;++d)out.Axis[d]=AnalyticCertified::Subtract(
        AnalyticCertified::Multiply(a.Axis[(d+1)%3],b.Axis[(d+2)%3]),
        AnalyticCertified::Multiply(a.Axis[(d+2)%3],b.Axis[(d+1)%3]));
    return out;
}
inline FIntervalPoint Rotate(const FQuat4d& q,const FVector3d& point)
{
    auto p=Detail::Point(point);const auto v=Detail::Point(FVector3d(q.X,q.Y,q.Z));
    auto t=Cross(v,p);
    for(int32 d=0;d<3;++d)t.Axis[d]=AnalyticCertified::Multiply(Interval::Exact(2),t.Axis[d]);
    const auto c=Cross(v,t);
    for(int32 d=0;d<3;++d)p.Axis[d]=AnalyticCertified::Add(
        AnalyticCertified::Add(p.Axis[d],AnalyticCertified::Multiply(Interval::Exact(q.W),t.Axis[d])),c.Axis[d]);
    return p;
}
}
// Literal FIXED-orientation FWorldQuery volume on Start -> End. This is NOT a
// bound on a native accelerated/rotating solid or an Integrate/Quantize DAG.
// Kernel release/domain tolerances and native response margins are not invented
// here: the consumer must separately bind its actual shape and operation.
// In particular DomainTolerance can make a finite-plane kernel accept a face
// outside this geometric volume. This bound ALONE cannot exclude that native
// kernel result or establish coverage for suspension/support/solid execution.
inline bool BuildFixedWorldQueryVolumeBounds(const FWorldQuery& query,FIntervalPoint& out)
{
    out={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(query.Start.ContainsNaN()||query.End.ContainsNaN()||query.Rotation.ContainsNaN()||!query.Rotation.IsNormalized())return false;
    FIntervalPoint center;
    for(int32 d=0;d<3;++d)
    {
        const auto start=Interval::Exact(query.Start[d]),end=Interval::Exact(query.End[d]);
        center.Axis[d]=AnalyticCertified::Add(start,AnalyticCertified::Multiply(
            AnalyticCertified::Subtract(end,start),Interval{0,1,true}));
    }
    if(!Detail::ValidPoint(center))return false;
    FIntervalPoint offset=Detail::Point(FVector3d::ZeroVector);
    switch(query.Shape)
    {
    case EQueryShape::Ray:break;
    case EQueryShape::Sphere:
        if(!CandidateDetail::PositiveRadius(query.Radius))return false;
        for(int32 d=0;d<3;++d)offset.Axis[d]={-query.Radius,query.Radius,true};break;
    case EQueryShape::Box:
    {
        if(query.HalfExtent.ContainsNaN()||query.HalfExtent.GetMin()<=0)return false;
        Private::FBoxSweepContext context;context.Initialize(query);
        bool first=true;
        for(int32 vertex=0;vertex<8;++vertex)
        {
            FVector3d corner;
            for(int32 d=0;d<3;++d)corner[d]=(vertex&(1<<d))?query.HalfExtent[d]:-query.HalfExtent[d];
            auto rotated=FixedQueryBoundsDetail::Rotate(query.Rotation,corner);
            // SAT uses normalized cached axes; other witnesses rotate actual
            // corners. Enclose BOTH retained representations, without assuming
            // their last rounded bits are interchangeable.
            FIntervalPoint axes=Detail::Point(FVector3d::ZeroVector);
            for(int32 d=0;d<3;++d)for(int32 k=0;k<3;++k)axes.Axis[d]=AnalyticCertified::Add(
                axes.Axis[d],AnalyticCertified::Multiply(Interval::Exact(context.Axis[k][d]),Interval::Exact(corner[k])));
            if(!Detail::ValidPoint(rotated)||!Detail::ValidPoint(axes))return false;
            rotated=TimeDetail::Hull(rotated,axes);offset=first?rotated:TimeDetail::Hull(offset,rotated);first=false;
        }
        break;
    }
    default:return false;
    }
    FIntervalPoint result;
    for(int32 d=0;d<3;++d)result.Axis[d]=AnalyticCertified::Add(center.Axis[d],offset.Axis[d]);
    if(!Detail::ValidPoint(result))return false;
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);out=result;return true;
}
class FReferenceBoundsCandidateSet;
using FReferenceBoundsCandidateSetPtr=TSharedPtr<const FReferenceBoundsCandidateSet,ESPMode::ThreadSafe>;
// Complete conservative partition for an already enclosed volume. Strict AABB
// separation keeps tangency. No positive-radius workaround is needed for rays.
// A candidate is UNKNOWN geometry, not a hit; no Source/filter/frame permission
// follows even from a complete empty partition.
class FReferenceBoundsCandidateSet final
{
    FIndexedIntervalUnionPtr Scene;FIntervalPoint Hull;
    TArray<int32> Candidates,Excluded;
    FReferenceBoundsCandidateSet()=default;
public:
    static bool Select(FIndexedIntervalUnionPtr scene,FIntervalPoint hull,FDistanceBudget& budget,FReferenceBoundsCandidateSetPtr& out,FReferenceSelectionWork* work=nullptr)
    {
        const FReferenceBoundsCandidateSetPtr keepPrior=out;out.Reset();if(work)*work={};
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!scene||!Detail::ValidPoint(hull)||budget.Remaining<=0||scene->Num()>budget.Remaining)return false;
        TSharedPtr<FReferenceBoundsCandidateSet,ESPMode::ThreadSafe> result(new FReferenceBoundsCandidateSet);
        result->Scene=scene;result->Hull=hull;
        Collision::FOrderedBoundsScratch scratch;TConstArrayView<int32> ordered;int32 next=0;
        const bool indexed=scene->BoundsIndex && scene->ProviderWorkAvailable(budget);
        if(indexed)
        {
            ordered=scene->BoundsIndex->FindCandidates(FIndexedIntervalUnion::NativeBox(hull),scratch);
            if(work)*work={scratch.NodesVisited,scratch.ProviderBoundsTested,0,true};
        }
        for(int32 i=0;i<scene->Num();++i)
        {
            if(!budget.Consume())return false;bool separate=false;
            // Boxes are private owned immutable values, validated by Build.
            // The query hull was validated above; no census/World cache is used.
            if(indexed){separate=next>=ordered.Num()||ordered[next]!=i;if(!separate)++next;}
            else
            {
                const auto& box=scene->Boxes[i];
                for(int32 d=0;d<3;++d)separate|=hull.Axis[d].Upper<box.Axis[d].Lower||box.Axis[d].Upper<hull.Axis[d].Lower;
            }
            (separate?result->Excluded:result->Candidates).Add(i);
        }
        if(result->Candidates.Num()+result->Excluded.Num()!=scene->Num())return false;
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);out=MoveTemp(result);return true;
    }
    const FIndexedIntervalUnionPtr& GetScene() const {return Scene;}
    const FIntervalPoint& GetQueryHull() const {return Hull;}
    TConstArrayView<int32> GetCandidateIndices() const {return Candidates;}
    TConstArrayView<int32> GetExcludedIndices() const {return Excluded;}
};
}
