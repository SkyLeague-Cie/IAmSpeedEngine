#pragma once
#include "FixedShapeQueryBounds.h"

namespace Speed::Analytic::Certified
{
// FUTURE, uninstalled. Each group encloses ONE COMPLETE convex Box volume
// family. All its vertices belong together. A proof never clears eight points
// independently and never assumes that the arena itself is convex.
struct FBoxVolumeGroup
{
    TArray<FIntervalPoint,TInlineAllocator<16>> Corners;
};
struct FGroupedBoxVolumeBounds
{
    FIntervalPoint BroadHull;
    TArray<FBoxVolumeGroup,TInlineAllocator<16>> Groups;
    TArray<FVector3d,TInlineAllocator<8>> Axes;
    // Group-major; one projection of the ENTIRE group on each fixed axis.
    TArray<Interval> Slabs;
};
namespace StaticBodyTubes
{
enum class EResult:uint8 {Unavailable,Clear,Unresolved,Invalid};
IAMSPEED_API EResult TryClear(const FIndexedIntervalUnion&,int32,const FGroupedBoxVolumeBounds&,uint32,FDistanceBudget&);
}
namespace GroupedBoxDetail
{
inline bool SameInterval(Interval a,Interval b)
{
    return a.Known==b.Known&&AnalyticCertified::Bits(a.Lower)==AnalyticCertified::Bits(b.Lower)&&
        AnalyticCertified::Bits(a.Upper)==AnalyticCertified::Bits(b.Upper);
}
inline bool SamePoint(const FIntervalPoint& a,const FIntervalPoint& b)
{
    for(int32 d=0;d<3;++d)if(!SameInterval(a.Axis[d],b.Axis[d]))return false;
    return true;
}
inline bool Within(const FIntervalPoint& outer,const FIntervalPoint& inner)
{
    if(!Detail::ValidPoint(outer)||!Detail::ValidPoint(inner))return false;
    for(int32 d=0;d<3;++d)if(inner.Axis[d].Lower<outer.Axis[d].Lower||inner.Axis[d].Upper>outer.Axis[d].Upper)return false;
    return true;
}
inline Interval Project(const FIntervalPoint& point,const FVector3d& axis)
{
    if(!Detail::ValidPoint(point))return {};
    auto result=Interval::Exact(0);
    for(int32 d=0;d<3;++d)result=AnalyticCertified::Add(result,
        AnalyticCertified::Multiply(point.Axis[d],Interval::Exact(axis[d])));
    return result;
}
template<typename Points>
inline Interval ProjectHull(const Points& points,const FVector3d& axis)
{
    Interval result;
    for(const auto& point:points)
    {
        const auto p=Project(point,axis);if(!AnalyticCertified::Valid(p))return {};
        if(!result.Known)result=p;
        else {result.Lower=FMath::Min(result.Lower,p.Lower);result.Upper=FMath::Max(result.Upper,p.Upper);}
    }
    return result;
}
inline bool ValidAxis(const FVector3d& axis)
{
    bool nonzero=false;
    for(int32 d=0;d<3;++d)
    {
        const auto value=Interval::Exact(axis[d]);if(!AnalyticCertified::Valid(value))return false;
        nonzero|=(AnalyticCertified::Bits(axis[d])&AnalyticCertified::Magnitude)>=AnalyticCertified::MinNormal;
    }
    return nonzero;
}
inline bool Same(const FGroupedBoxVolumeBounds& a,const FGroupedBoxVolumeBounds& b)
{
    if(!SamePoint(a.BroadHull,b.BroadHull)||a.Groups.Num()!=b.Groups.Num()||a.Axes.Num()!=b.Axes.Num()||a.Slabs.Num()!=b.Slabs.Num())return false;
    for(int32 k=0;k<a.Axes.Num();++k)for(int32 d=0;d<3;++d)
        if(AnalyticCertified::Bits(a.Axes[k][d])!=AnalyticCertified::Bits(b.Axes[k][d]))return false;
    for(int32 k=0;k<a.Slabs.Num();++k)if(!SameInterval(a.Slabs[k],b.Slabs[k]))return false;
    for(int32 k=0;k<a.Groups.Num();++k)
    {
        if(a.Groups[k].Corners.Num()!=b.Groups[k].Corners.Num())return false;
        for(int32 p=0;p<a.Groups[k].Corners.Num();++p)
            if(!SamePoint(a.Groups[k].Corners[p],b.Groups[k].Corners[p]))return false;
    }
    return true;
}
}

// Pure constructor, not a scene/query/advance capability. Shells must ALREADY
// have been applied with the producer's exact original XYZ inflation to each
// corner enclosure. The caller supplies every native/reference/query group.
inline bool CloseGroupedBoxVolumeBounds(FIntervalPoint broadHull,
    TConstArrayView<FBoxVolumeGroup> groups,TConstArrayView<FVector3d> axes,
    FDistanceBudget& budget,FGroupedBoxVolumeBounds& output)
{
    output={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!Detail::ValidPoint(broadHull)||groups.IsEmpty()||groups.Num()>16||axes.IsEmpty()||axes.Num()>12)return false;
    FGroupedBoxVolumeBounds result;result.BroadHull=broadHull;
    for(const auto& axis:axes)
    {if(!budget.Consume()||!GroupedBoxDetail::ValidAxis(axis))return false;result.Axes.Add(axis);}
    for(const auto& group:groups)
    {
        if(group.Corners.Num()<8||group.Corners.Num()>32)return false;
        for(const auto& point:group.Corners)
        {
            if(!budget.Consume()||!Detail::ValidPoint(point))return false;
            result.BroadHull=TimeDetail::Hull(result.BroadHull,point);
        }
        for(const auto& axis:result.Axes)
        {
            if(!budget.Consume())return false;
            const auto slab=GroupedBoxDetail::ProjectHull(group.Corners,axis);
            if(!AnalyticCertified::Valid(slab))return false;result.Slabs.Add(slab);
        }
        result.Groups.Add(group);
    }
    if(!Detail::ValidPoint(result.BroadHull)||result.Slabs.Num()!=result.Groups.Num()*result.Axes.Num())return false;
    output=MoveTemp(result);return true;
}

class FClosedGroupedBoxVolume;
using FClosedGroupedBoxVolumePtr=TSharedPtr<const FClosedGroupedBoxVolume,ESPMode::ThreadSafe>;
namespace ClosedGroupedBoxControl
{
IAMSPEED_API bool RuntimeEnabled();
IAMSPEED_API void RecordSkippedClosure(uint64 work,uint64 points);
IAMSPEED_API void PublishObservation();
}
// Owned mathematical closure, not scene/query/actor authority. No public
// promotion of mutable Slabs, hash or detached "trusted" flag exists.
class FClosedGroupedBoxVolume final
{
    FGroupedBoxVolumeBounds Value;
    uint64 Numeric=0,RepeatedWork=0,RepeatedPointProjections=0;
    FClosedGroupedBoxVolume()=default;
    friend class FReferenceBoxVolumeStrictSeparation;
public:
    static bool Build(FIntervalPoint broad,TConstArrayView<FBoxVolumeGroup> groups,
        TConstArrayView<FVector3d> axes,FDistanceBudget& budget,FClosedGroupedBoxVolumePtr& output)
    {
        const auto prior=output;output.Reset();
        const auto before=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        FGroupedBoxVolumeBounds local;
        if(!CloseGroupedBoxVolumeBounds(broad,groups,axes,budget,local))return false;
        const auto after=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(before.ControlFingerprint()!=after.ControlFingerprint())return false;
        // Close has bounded A<=12, G<=16, C<=32. These integer sums cannot
        // overflow. All TArrays are owned copies, including each group's net.
        TSharedPtr<FClosedGroupedBoxVolume,ESPMode::ThreadSafe> result(new FClosedGroupedBoxVolume);
        result->Numeric=after.ControlFingerprint();result->RepeatedWork=uint64(local.Axes.Num());
        for(const auto& group:local.Groups)
        {result->RepeatedWork+=uint64(group.Corners.Num()+local.Axes.Num());
         result->RepeatedPointProjections+=uint64(group.Corners.Num())*uint64(local.Axes.Num());}
        result->Value=MoveTemp(local);output=MoveTemp(result);return true;
    }
    const FGroupedBoxVolumeBounds& GetVolumeForInspection() const{return Value;}
};

class FReferenceBoxVolumeStrictSeparation;
using FReferenceBoxVolumeStrictSeparationPtr=TSharedPtr<const FReferenceBoxVolumeStrictSeparation,ESPMode::ThreadSafe>;
// DISTINCT from FReferenceBoundsStrictSeparation. BroadHull alone is NOT clear.
// For each group, all of its fixed-axis slabs together enclose a region proved
// clear of every retained surface. An endpoint must fit ONE complete region;
// selecting a different region per vertex would be unsound.
// Passive observation only. It cannot mint a geometry proof and performs no
// query, refinement, budget consumption or retry of a failed operation.
struct FGroupedBoxSeparationObservation
{
    const TCHAR* Stage=TEXT("not_entered");int32 Provider=INDEX_NONE,Group=INDEX_NONE,Axis=INDEX_NONE;
    int32 DegreeU=0,DegreeV=0,StackDepth=0;uint64 VisitedCells=0,TestedGroups=0,InheritedGroupSkips=0;
    uint32 ActiveGroupMask=0; // Passive; no external capability to install a mask.
    double MinU=0,MaxU=0,MinV=0,MaxV=0;
};
namespace GroupedBoxGeometryMemo
{
IAMSPEED_API bool RuntimeEnabled();
IAMSPEED_API void PublishObservation();
IAMSPEED_API bool ProjectCell(const Detail::FNode& node,TConstArrayView<FVector3d> axes,FDistanceBudget& budget,
    TArray<Interval,TInlineAllocator<8>>& projected,FGroupedBoxSeparationObservation* observed);
IAMSPEED_API bool ProjectCellForVolume(const Detail::FNode& node,const FGroupedBoxVolumeBounds& volume,uint32 activeGroups,
    FDistanceBudget& budget,TArray<Interval,TInlineAllocator<8>>& projected,FGroupedBoxSeparationObservation* observed);
IAMSPEED_API bool SplitCell(const Detail::FNode& node,Detail::FNode& left,Detail::FNode& right);
}
namespace GroupedBoxProjectedSubdivision
{
// Operation-local ORIGINAL-root polynomial projections, not a memo of the
// independently widened XYZ child-net family. Only TryProve's own stack may
// pair these values with its corresponding XYZ node/domain.
struct FCell;
using FCellPtr=TSharedPtr<FCell,ESPMode::NotThreadSafe>;
// 0 OFF, 1 eight-group coarse only, 2 all admissible volumes including fine.
IAMSPEED_API int32 RuntimeMode();
IAMSPEED_API void PublishObservation();
IAMSPEED_API FCellPtr Begin(const Detail::FNode& original,TConstArrayView<FVector3d> axes);
IAMSPEED_API bool ProjectCell(const FCellPtr& scalar,const Detail::FNode& node,
    const FGroupedBoxVolumeBounds& volume,uint32 activeGroups,FDistanceBudget& budget,
    TArray<Interval,TInlineAllocator<8>>& projected,FGroupedBoxSeparationObservation* observed);
IAMSPEED_API bool SplitCell(const FCellPtr& scalar,const Detail::FNode& node,
    const Detail::FNode& left,const Detail::FNode& right,FCellPtr& a,FCellPtr& b);
}

namespace GroupedBoxExtrusionElimination
{
enum class EResult:uint8 {NotApplicable,Ready,Cleared,Unresolved,Invalid};
IAMSPEED_API bool RuntimeEnabled();
// Clear-only on the original root family. The256 optional units stay spent;
// the caller's full fine reservation survives, not all former coarse work.
IAMSPEED_API EResult TryClear(const FIntervalTensorSurface&,const FGroupedBoxVolumeBounds&,FDistanceBudget&,uint32 activeGroups);
}

namespace StaticBodyHierarchy
{
enum class EFrontier:uint8 {Unavailable,Ready,Invalid,Work};
struct FLeaf {Detail::FNode Cell;uint32 ActiveGroups=0;};
IAMSPEED_API bool RuntimeEnabledFor(int32 groupCount);
// Ready with zero leaves is a complete strict rejection of the supplied root
// obligations. Nonempty leaves cover every domain still requiring exact work.
IAMSPEED_API EFrontier BuildFrontier(const FIndexedIntervalUnion&,int32 provider,
    const FGroupedBoxVolumeBounds&,uint32 active,
    FDistanceBudget&,TArray<FLeaf,TInlineAllocator<16>>& leaves);
}

namespace GroupedBoxStripProof
{
enum class EResult:uint8 {NotApplicable,Cleared,Unresolved,Invalid};
IAMSPEED_API bool RuntimeEnabled();
// Pure optional ALL-u strip enclosure. Only Cleared can discharge this
// provider; Unresolved keeps the original 2D proof and its already spent work.
IAMSPEED_API EResult TryClear(const FIntervalTensorSurface& surface,const FGroupedBoxVolumeBounds& volume,
    FDistanceBudget& budget,bool enableGeometryMemo);
}
namespace GroupedBoxInteriorWitness
{
enum class EResult:uint8 {NotApplicable,Rejected,Invalid};
IAMSPEED_API bool RuntimeEnabled();
// One bounded ORIGINAL-UV probe per provider, only for eight-group coarse
// volumes. Rejected stops this separation attempt; it is never a contact.
IAMSPEED_API EResult TryReject(const FIntervalTensorSurface& surface,const FGroupedBoxVolumeBounds& volume,
    FDistanceBudget& budget,uint32 activeGroups=MAX_uint32);
}
class FReferenceBoxVolumeStrictSeparation final
{
    FReferenceBoundsCandidateSetPtr Partition;
    FGroupedBoxVolumeBounds Volume;
    FReferenceBoxVolumeStrictSeparation()=default;
    static bool CellSeparated(const Detail::FNode& cell,const FGroupedBoxVolumeBounds& volume,
        FDistanceBudget& budget,uint32& activeGroups,bool& separate,FGroupedBoxSeparationObservation* observed,bool enableGeometryMemo,const GroupedBoxProjectedSubdivision::FCellPtr& scalar)
    {
        separate=false;
        const uint32 full=(1u<<volume.Groups.Num())-1u;
        if(!activeGroups||(activeGroups&~full))return false;
        TArray<Interval,TInlineAllocator<8>> projected;
        if(scalar)
        {
            if(!GroupedBoxProjectedSubdivision::ProjectCell(scalar,cell,volume,activeGroups,budget,projected,observed))return false;
        }
        else if(enableGeometryMemo)
        {
            if(!GroupedBoxGeometryMemo::ProjectCellForVolume(cell,volume,activeGroups,budget,projected,observed))return false;
        }
        else for(const auto& axis:volume.Axes)
        {
            if(observed){observed->Stage=TEXT("cell_projection_or_budget");observed->Axis=projected.Num();}
            if(!budget.Consume())return false;
            const auto slab=GroupedBoxDetail::ProjectHull(cell.Net,axis);
            if(!AnalyticCertified::Valid(slab))return false;projected.Add(slab);
        }
        for(int32 g=0;g<volume.Groups.Num();++g)
        {
            const uint32 bit=1u<<g;
            if(!(activeGroups&bit)){if(observed)++observed->InheritedGroupSkips;continue;}
            if(observed){observed->Stage=TEXT("cell_group_or_budget");observed->Group=g;observed->ActiveGroupMask=activeGroups;}
            if(!budget.Consume())return false;
            if(observed)++observed->TestedGroups;
            bool groupSeparate=false;
            for(int32 a=0;a<volume.Axes.Num();++a)
            {
                const auto body=volume.Slabs[g*volume.Axes.Num()+a],surface=projected[a];
                groupSeparate|=body.Upper<surface.Lower||surface.Upper<body.Lower;
            }
            // A strict separation on this ORIGINAL polynomial domain remains
            // valid on its descendants. Scan ALL other active groups even if
            // one is unresolved; never infer full-volume clearance from one.
            if(groupSeparate)activeGroups&=~bit;
        }
        if(observed)observed->ActiveGroupMask=activeGroups;
        separate=activeGroups==0;return true;
    }
private:
    static bool TryProveImpl(FReferenceBoundsCandidateSetPtr partition,const FGroupedBoxVolumeBounds& volume,
        const FClosedGroupedBoxVolume* closed,FDistanceBudget& budget,FReferenceBoxVolumeStrictSeparationPtr& output,
        FGroupedBoxSeparationObservation* observed,bool enableGeometryMemo,bool enableStrip,bool enableInteriorWitness,
        int32 projectedSubdivisionMode,bool enableExtrusionElimination)
    {
        if(observed){*observed={};observed->Stage=TEXT("partition_or_initial_budget");}
        output.Reset();const auto currentNumeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(closed&&currentNumeric.ControlFingerprint()!=closed->Numeric)return false;
        if(!partition||!partition->GetScene()||!GroupedBoxDetail::SamePoint(partition->GetQueryHull(),volume.BroadHull)||
            volume.Groups.IsEmpty()||volume.Groups.Num()>16||volume.Axes.IsEmpty()||volume.Axes.Num()>12||
            volume.Slabs.Num()!=volume.Groups.Num()*volume.Axes.Num()||!budget.Consume()||
            partition->GetCandidateIndices().Num()+partition->GetExcludedIndices().Num()!=partition->GetScene()->Num())return false;
        // Revalidate every stored projection before it can certify geometry.
        if(observed)observed->Stage=TEXT("reclose_volume");
        if(closed)
        {
            // Same partial-prefix debit as every original Consume, including
            // floor and Used overflow. Immutable geometry replaces arithmetic,
            // never work accounting or current scene/query authority.
            if(!CandidateDetail::ConsumeRepeated(budget,closed->RepeatedWork))return false;
            ClosedGroupedBoxControl::RecordSkippedClosure(closed->RepeatedWork,closed->RepeatedPointProjections);
        }
        else
        {
            FGroupedBoxVolumeBounds checked;
            if(!CloseGroupedBoxVolumeBounds(volume.BroadHull,volume.Groups,volume.Axes,budget,checked)||
                !GroupedBoxDetail::Same(volume,checked))return false;
        }
        for(int32 index:partition->GetCandidateIndices())
        {
            if(observed){observed->Stage=TEXT("copy_provider");observed->Provider=index;}
            FIntervalTensorSurface surface;
            if(!partition->GetScene()->CopySurfaceForQuery(index,budget,surface))return false;
            if(enableStrip)
            {
                const auto strip=GroupedBoxStripProof::TryClear(surface,volume,budget,enableGeometryMemo);
                if(strip==GroupedBoxStripProof::EResult::Invalid)return false;
                if(strip==GroupedBoxStripProof::EResult::Cleared)continue;
            }
            Detail::FNode root;root.DegreeU=surface.DegreeU;root.DegreeV=surface.DegreeV;
            root.Net=surface.ControlPoints;
            // Masks have internal same-provider, same-volume ancestry only.
            // They describe the FIXED original CP family on nested domains,
            // not every independent model of widened child CP intervals.
            struct FWork {Detail::FNode Cell;uint32 ActiveGroups;GroupedBoxProjectedSubdivision::FCellPtr Scalar;};
            const uint32 full=(1u<<volume.Groups.Num())-1u;
            const bool useProjected=projectedSubdivisionMode==2||(projectedSubdivisionMode==1&&volume.Groups.Num()==8);
            auto scalar=useProjected?GroupedBoxProjectedSubdivision::Begin(root,volume.Axes):GroupedBoxProjectedSubdivision::FCellPtr{};
            TArray<FWork,TInlineAllocator<16>> stack;stack.Add(FWork{MoveTemp(root),full,MoveTemp(scalar)});
            while(!stack.IsEmpty())
            {
                auto work=stack.Pop(EAllowShrinking::No);auto& cell=work.Cell;bool separate=false;
                if(observed)
                {
                    ++observed->VisitedCells;observed->StackDepth=stack.Num();observed->ActiveGroupMask=work.ActiveGroups;
                    observed->DegreeU=cell.DegreeU;observed->DegreeV=cell.DegreeV;
                    observed->MinU=cell.MinU;observed->MaxU=cell.MaxU;
                    observed->MinV=cell.MinV;observed->MaxV=cell.MaxV;
                }
                if(!CellSeparated(cell,volume,budget,work.ActiveGroups,separate,observed,enableGeometryMemo,work.Scalar))return false;
                if(separate)continue;
                // Only the original root is probed, after all its cheap axes
                // failed to discharge the remaining groups, before first split.
                if(enableInteriorWitness&&volume.Groups.Num()==8&&cell.DepthU==0&&cell.DepthV==0)
                {
                    const auto witness=GroupedBoxInteriorWitness::TryReject(surface,volume,budget,work.ActiveGroups);
                    if(witness!=GroupedBoxInteriorWitness::EResult::NotApplicable)
                    {
                        if(observed)observed->Stage=witness==GroupedBoxInteriorWitness::EResult::Rejected?
                            TEXT("original_uv_inside_group_region"):TEXT("original_uv_witness_numeric");
                        return false;
                    }
                }
                if(enableExtrusionElimination&&volume.Groups.Num()==8&&cell.DepthU==0&&cell.DepthV==0)
                {
                    const auto eliminated=GroupedBoxExtrusionElimination::TryClear(surface,volume,budget,work.ActiveGroups);
                    if(eliminated==GroupedBoxExtrusionElimination::EResult::Invalid)return false;
                    if(eliminated==GroupedBoxExtrusionElimination::EResult::Cleared)continue;
                    // Optional setup/search remains spent; current coarse2D
                    // continues and the caller's original fine floor survives.
                }
                if(volume.Groups.Num()==14&&cell.DepthU==0&&cell.DepthV==0&&StaticBodyTubes::RuntimeEnabled())
                {
                    const auto tube=StaticBodyTubes::TryClear(*partition->GetScene(),index,volume,work.ActiveGroups,budget);
                    if(tube==StaticBodyTubes::EResult::Invalid)return false;
                    if(tube==StaticBodyTubes::EResult::Clear)continue;
                    // Optional effort stays spent; no new work account or
                    // approximation response. Original atlas/DFS continues.
                }
                if(StaticBodyHierarchy::RuntimeEnabledFor(volume.Groups.Num())&&cell.DepthU==0&&cell.DepthV==0)
                {
                    TArray<StaticBodyHierarchy::FLeaf,TInlineAllocator<16>> frontier;
                    const auto prepared=StaticBodyHierarchy::BuildFrontier(*partition->GetScene(),index,volume,work.ActiveGroups,budget,frontier);
                    if(prepared==StaticBodyHierarchy::EFrontier::Invalid||prepared==StaticBodyHierarchy::EFrontier::Work)return false;
                    if(prepared==StaticBodyHierarchy::EFrontier::Ready)
                    {
                        // Each remaining closed leaf belongs to this original
                        // provider. Do not seed a scalar memo as a fresh root.
                        for(int32 i=frontier.Num()-1;i>=0;--i)
                            stack.Add(FWork{MoveTemp(frontier[i].Cell),frontier[i].ActiveGroups,{}});
                        continue;
                    }
                }
                if(observed)observed->Stage=TEXT("point_or_budget_before_split");
                if((cell.DegreeU==0&&cell.DegreeV==0)||budget.Remaining-budget.ConsumptionFloor<2)return false;
                Detail::FNode a,b;
                if(observed)observed->Stage=TEXT("split");
                if(enableGeometryMemo? !GroupedBoxGeometryMemo::SplitCell(cell,a,b):
                    !Detail::Split(cell,Detail::ChooseTensorDistanceSplitU(cell),a,b))return false;
                GroupedBoxProjectedSubdivision::FCellPtr scalarA,scalarB;
                if(work.Scalar&&!GroupedBoxProjectedSubdivision::SplitCell(work.Scalar,cell,a,b,scalarA,scalarB))return false;
                if(observed)observed->Stage=TEXT("consume_child_a");
                if(!budget.Consume())return false;
                if(observed)observed->Stage=TEXT("consume_child_b");
                if(!budget.Consume())return false;
                // Each sibling receives its OWN copy of the parent's still
                // unresolved obligations; neither can discharge the other.
                stack.Add(FWork{MoveTemp(b),work.ActiveGroups,MoveTemp(scalarB)});stack.Add(FWork{MoveTemp(a),work.ActiveGroups,MoveTemp(scalarA)});
            }
        }
        TSharedPtr<FReferenceBoxVolumeStrictSeparation,ESPMode::ThreadSafe> result(new FReferenceBoxVolumeStrictSeparation);
        result->Partition=MoveTemp(partition);result->Volume=volume;
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(observed)observed->Stage=TEXT("complete");
        output=MoveTemp(result);return true;
    }
public:
    static bool TryProve(FReferenceBoundsCandidateSetPtr partition,const FGroupedBoxVolumeBounds& volume,
        FDistanceBudget& budget,FReferenceBoxVolumeStrictSeparationPtr& output,
        FGroupedBoxSeparationObservation* observed=nullptr,bool enableGeometryMemo=GroupedBoxGeometryMemo::RuntimeEnabled(),
        bool enableStrip=GroupedBoxStripProof::RuntimeEnabled(),
        bool enableInteriorWitness=GroupedBoxInteriorWitness::RuntimeEnabled(),
        int32 projectedSubdivisionMode=GroupedBoxProjectedSubdivision::RuntimeMode(),
        bool enableExtrusionElimination=GroupedBoxExtrusionElimination::RuntimeEnabled())
    {
        return TryProveImpl(MoveTemp(partition),volume,nullptr,budget,output,observed,enableGeometryMemo,
            enableStrip,enableInteriorWitness,projectedSubdivisionMode,enableExtrusionElimination);
    }
    static bool TryProveClosed(FReferenceBoundsCandidateSetPtr partition,FClosedGroupedBoxVolumePtr closed,
        FDistanceBudget& budget,FReferenceBoxVolumeStrictSeparationPtr& output,
        FGroupedBoxSeparationObservation* observed=nullptr,bool enableGeometryMemo=GroupedBoxGeometryMemo::RuntimeEnabled(),
        bool enableStrip=GroupedBoxStripProof::RuntimeEnabled(),bool enableInteriorWitness=GroupedBoxInteriorWitness::RuntimeEnabled(),
        int32 projectedSubdivisionMode=GroupedBoxProjectedSubdivision::RuntimeMode(),
        bool enableExtrusionElimination=GroupedBoxExtrusionElimination::RuntimeEnabled())
    {
        // Keep the owner by value through the call; output may alias a caller's
        // previous proof. This entry never accepts a separate mutable volume.
        if(!closed){output.Reset();return false;}
        return TryProveImpl(MoveTemp(partition),closed->Value,closed.Get(),budget,output,observed,enableGeometryMemo,
            enableStrip,enableInteriorWitness,projectedSubdivisionMode,enableExtrusionElimination);
    }
    bool Matches(const FIndexedIntervalUnionPtr& scene,const FGroupedBoxVolumeBounds& volume) const
    {return Partition&&Partition->GetScene()==scene&&GroupedBoxDetail::Same(Volume,volume);}
    bool ContainsCompleteEndpointGroup(TConstArrayView<FIntervalPoint> corners) const
    {
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!Partition||corners.Num()<8||corners.Num()>32)return false;
        for(const auto& point:corners)if(!GroupedBoxDetail::Within(Volume.BroadHull,point))return false;
        TArray<Interval,TInlineAllocator<8>> slabs;
        for(const auto& axis:Volume.Axes)
        {
            const auto range=GroupedBoxDetail::ProjectHull(corners,axis);
            if(!AnalyticCertified::Valid(range))return false;slabs.Add(range);
        }
        for(int32 g=0;g<Volume.Groups.Num();++g)
        {
            bool within=true;
            for(int32 a=0;a<Volume.Axes.Num();++a)
            {
                const auto bound=Volume.Slabs[g*Volume.Axes.Num()+a],actual=slabs[a];
                within&=actual.Lower>=bound.Lower&&actual.Upper<=bound.Upper;
            }
            if(within)return true;
        }
        return false;
    }
    const FIntervalPoint& GetBroadHullForInspection() const{return Volume.BroadHull;}
};
}
