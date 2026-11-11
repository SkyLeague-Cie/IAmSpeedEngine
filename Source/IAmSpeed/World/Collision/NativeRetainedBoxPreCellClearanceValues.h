#pragma once
#include "NativeRetainedBoxCornerReferenceValues.h"
#include "IAmSpeed/World/Analytic/ReferenceBoundsStrictSeparation.h"
namespace Speed::PreparedExecution
{
class FNativeRetainedBoxPreCellClearanceValues;
using FNativeRetainedBoxPreCellClearanceValuesPtr=TSharedPtr<const FNativeRetainedBoxPreCellClearanceValues,ESPMode::ThreadSafe>;
// Strict clearance of the ENTIRE fixed reference Box from EVERY retained
// provider, over the closed prefix ending at the START of one local root's
// time cell. The remainder of that cell before the root is NOT covered.
// No earliest hit, consumed-float/query-SAT miss, native advance or Ready.
class FNativeRetainedBoxPreCellClearanceValues final
{
public:
    struct FSlab
    {
        Analytic::Certified::FReferenceDyadicCell Cell;
        bool InitialPointOnly=false; // Cell is unused for this explicit tau0 case.
        Analytic::Certified::FIntervalPoint Hull;
        Analytic::Certified::FReferenceBoundsStrictSeparationPtr Separation;
    };
private:
    FNativeRetainedBoxCornerReferenceValuesPtr Corner;
    TArray<FSlab> Slabs;double PrefixEnd=0;
    FNativeRetainedBoxPreCellClearanceValues()=default;
public:
    // Pure reference volume helper only; its hull alone proves no clearance,
    // native-query miss, response, advance or physical authority.
    static bool BoundWholeBoxReference(const FNativeFixedBoxReferenceMotionValues& motion,
        const Analytic::Certified::FReferenceDyadicCell& cell,bool initial,
        Analytic::Certified::FDistanceBudget& budget,Analytic::Certified::FIntervalPoint& out)
    {
        out={};bool first=true;
        for(int32 k=0;k<8;++k)
        {
            if(!budget.Consume())return false;
            const auto& original=motion.GetCornerPathsForInspection()[k];
            if(!Analytic::Certified::CenterDetail::Valid(original))return false;
            Analytic::Certified::FIntervalPoint hull;
            if(initial)hull=original.ControlPoints[0];
            else
            {
                Analytic::Certified::FIntervalBezierCenterPath path;
                if(!Analytic::Certified::RestrictReferencePathDyadic(original,cell,budget,path))return false;
                hull=Analytic::Certified::CenterDetail::Hull(path);
            }
            if(!Analytic::Certified::Detail::ValidPoint(hull))return false;
            out=first?hull:Analytic::Certified::TimeDetail::Hull(out,hull);first=false;
        }
        // At each time, the full fixed Box is the convex hull of these SAME
        // eight common-motion vertices. Their Bernstein/AABB hull contains
        // edges, faces and interior too; not just eight independent traces.
        return !first&&Analytic::Certified::Detail::ValidPoint(out);
    }
public:
    static bool TryProve(FNativeRetainedBoxCornerReferenceValuesPtr corner,
        Analytic::Certified::FDistanceBudget& budget,FNativeRetainedBoxPreCellClearanceValuesPtr& out)
    {
        const auto keepPrior=out;out.Reset();
        Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!corner||!budget.Consume())return false;
        const auto geometry=corner->GetGeometryForInspection();const auto root=corner->GetCornerReferenceForInspection();
        if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||!root)return false;
        const auto motion=root->GetMotionForInspection();const auto time=root->GetTimeCellForInspection();
        const auto range=time.GetRange();if(!motion||!AnalyticCertified::Valid(range))return false;
        TSharedPtr<FNativeRetainedBoxPreCellClearanceValues,ESPMode::ThreadSafe> value(new FNativeRetainedBoxPreCellClearanceValues);
        value->Corner=MoveTemp(corner);value->PrefixEnd=range.Lower;double covered=0;
        const auto append=[&](Analytic::Certified::FReferenceDyadicCell cell,bool initial)
        {
            if(!budget.Consume())return false;FSlab slab;slab.Cell=cell;slab.InitialPointOnly=initial;
            if(!initial)
            {
                const auto r=cell.GetRange();
                if(!AnalyticCertified::Valid(r)||AnalyticCertified::Bits(r.Lower)!=AnalyticCertified::Bits(covered))return false;
                covered=r.Upper;
            }
            if(!BoundWholeBoxReference(*motion,cell,initial,budget,slab.Hull))return false;
            Analytic::Certified::FReferenceBoundsCandidateSetPtr partition;
            if(!Analytic::Certified::FReferenceBoundsCandidateSet::Select(geometry->GetSelectedUnion(),slab.Hull,budget,partition)||
                !Analytic::Certified::FReferenceBoundsStrictSeparation::TryProve(MoveTemp(partition),budget,slab.Separation)||
                !slab.Separation||!slab.Separation->Matches(geometry->GetSelectedUnion(),slab.Hull))return false;
            // Stronger than a post-filtered provider set: every owner remains
            // in this complete geometry proof regardless of query policy.
            value->Slabs.Add(MoveTemp(slab));return true;
        };
        if(time.Index==0)
        {
            // A zero-length closed prefix still contains the initial BOX.
            // Never accept an empty list as proof of no initial contact.
            if(!append({},true))return false;
        }
        else for(uint32 level=1;level<=time.Depth;++level)
        {
            const uint64 prefix=time.Index>>(time.Depth-level);
            if(prefix&uint64(1))
            {
                Analytic::Certified::FReferenceDyadicCell left;left.Depth=level;left.Index=prefix-1;
                if(!append(left,false))return false;
            }
        }
        if(value->Slabs.IsEmpty()||AnalyticCertified::Bits(covered)!=AnalyticCertified::Bits(value->PrefixEnd))return false;
        Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        out=MoveTemp(value);return true;
    }
    bool MatchesInputs(const FNativeRetainedBoxCornerReferenceValuesPtr& corner,const FNativeSolidStaticValues& captured) const
    {
        return Corner&&Corner==corner&&Corner->GetGeometryForInspection()->HasValidAliases()&&
            Corner->GetCornerReferenceForInspection()->GetMotionForInspection()->MatchesCapturedValues(captured);
    }
    const auto& GetCornerForInspection() const{return Corner;}
    TConstArrayView<FSlab> GetSlabsForInspection() const{return Slabs;}
    double GetReferencePrefixEndNormalized() const{return PrefixEnd;}
};
}
