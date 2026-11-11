#pragma once
#include "FixedShapeQueryBounds.h"
namespace Speed::Analytic::Certified
{
class FReferenceBoundsStrictSeparation;
using FReferenceBoundsStrictSeparationPtr=TSharedPtr<const FReferenceBoundsStrictSeparation,ESPMode::ThreadSafe>;
// Universal clearance of one COMPLETE volume from the entire provided union.
// This is neither a legacy facet-kernel miss nor a World/advance/response grant.
class FReferenceBoundsStrictSeparation final
{
    FReferenceBoundsCandidateSetPtr Partition;
public:
    struct FRefinedProvider {int32 Index=INDEX_NONE;FTensorDistanceBounds Bounds;};
private:
    TArray<FRefinedProvider> Refined;
    FReferenceBoundsStrictSeparation()=default;
public:
    static bool TryProve(FReferenceBoundsCandidateSetPtr partition,FDistanceBudget& budget,FReferenceBoundsStrictSeparationPtr& out)
    {
        out.Reset();RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!partition||!partition->GetScene()||!Detail::ValidPoint(partition->GetQueryHull())||!budget.Consume()||
            partition->GetCandidateIndices().Num()+partition->GetExcludedIndices().Num()!=partition->GetScene()->Num())return false;
        TSharedPtr<FReferenceBoundsStrictSeparation,ESPMode::ThreadSafe> result(new FReferenceBoundsStrictSeparation);
        result->Partition=MoveTemp(partition);const auto& p=result->Partition;const auto zero=Interval::Exact(0);
        for(int32 index:p->GetCandidateIndices())
        {
            FIntervalTensorSurface surface;
            if(!p->GetScene()->CopySurfaceForQuery(index,budget,surface))return false;
            const auto d=BoundTensorDistanceSquaredIntervalCenter(surface,p->GetQueryHull(),budget,0,budget.Remaining,&zero);
            if(d.Status==EDistanceBoundStatus::Unknown||!AnalyticCertified::Finite(d.LowerSquared)||d.LowerSquared<=0||
                !AnalyticCertified::Finite(d.UpperSquared)||d.UpperSquared<d.LowerSquared)return false;
            result->Refined.Add({index,d});
        }
        if(result->Refined.Num()!=p->GetCandidateIndices().Num())return false;
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);out=MoveTemp(result);return true;
    }
    bool Matches(const FIndexedIntervalUnionPtr& scene,const FIntervalPoint& hull) const
    {
        if(!Partition||Partition->GetScene()!=scene)return false;const auto& retained=Partition->GetQueryHull();
        for(int32 d=0;d<3;++d)if(retained.Axis[d].Known!=hull.Axis[d].Known||
            AnalyticCertified::Bits(retained.Axis[d].Lower)!=AnalyticCertified::Bits(hull.Axis[d].Lower)||
            AnalyticCertified::Bits(retained.Axis[d].Upper)!=AnalyticCertified::Bits(hull.Axis[d].Upper))return false;
        return true;
    }
    const auto& GetPartition() const{return Partition;}
    TConstArrayView<FRefinedProvider> GetRefinedProviders() const{return Refined;}
};
}
