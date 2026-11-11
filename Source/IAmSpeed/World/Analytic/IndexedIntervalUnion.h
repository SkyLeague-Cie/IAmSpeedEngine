#pragma once
#include <atomic>
#include "TensorSphereContactBounds.h"
#include "KinematicCenterPathBounds.h"
#include "Templates/SharedPointer.h"
#include "../Collision/OrderedBoundsIndex.h"
#include "../Simulation/FrameFloatingPointContext.h"

namespace Speed::Analytic {struct FExtrudedQuinticPatch;}
namespace Speed::Analytic::Certified
{
// Owned immutable snapshot of the COMPLETE provided mathematical union.
// Canonical indices refer to input order; this is no World/filter/admission ticket.
class FIndexedIntervalUnion;
enum class EEmptyBounds91:uint8 {Clear,Near,Unknown};
using FIndexedIntervalUnionPtr=TSharedPtr<const FIndexedIntervalUnion,ESPMode::ThreadSafe>;
namespace StaticBodyHierarchy
{
class FAtlas;
using FAtlasPtr=TSharedPtr<const FAtlas,ESPMode::ThreadSafe>;
// Explicit publication-only opt-in: ordinary source readbacks never prepare it.
IAMSPEED_API FAtlasPtr Prepare(TConstArrayView<FIntervalTensorSurface>,FDistanceBudget&);
}

namespace StaticBodyTubes
{
class FAtlas;
using FAtlasPtr=TSharedPtr<const FAtlas,ESPMode::ThreadSafe>;
// Borrowed only during explicit publication. Geometry is a proposal: the
// complete ORIGINAL coefficient family is independently enclosed before use.
struct FInput {int32 Provider=INDEX_NONE;const FExtrudedQuinticPatch* Patch=nullptr;bool FreeAlongU=true;};
IAMSPEED_API bool RuntimeEnabled();
}

class FReferenceCandidateSet;
class FReferenceBoundsCandidateSet;
// Diagnostics of geometry selection only, never contact/admission evidence.
struct FReferenceSelectionWork {uint64 NodesVisited=0,ProviderBoundsTested=0,RadiusPredicates=0;bool UsedBvh=false;uint64 CandidatesBeforeSegment=0,CandidatesAfterSegment=0,SegmentTests=0,SegmentExcluded=0,
    SegmentUncertain=0,SegmentAxes=0,SegmentPairs=0,SegmentInapplicable=0;};
using FReferenceCandidateSetPtr=TSharedPtr<const FReferenceCandidateSet,ESPMode::ThreadSafe>;
// A borrowed immutable ordered view. Compact exclusions are the COMPLETE
// complement in [0,N), never a reduced scene. No lazy allocation or TLS storage.
// As with TConstArrayView, the candidate-set owner must outlive the view.
class FExcludedProviderView
{
    TConstArrayView<int32> Explicit,Included;
    const FReferenceCandidateSet* Owner=nullptr;
    int32 ProviderCount=0;
    FExcludedProviderView(const FReferenceCandidateSet* owner,int32 count,TConstArrayView<int32> included)
        :Included(included),Owner(owner),ProviderCount(count){}
    friend class FReferenceCandidateSet;
public:
    FExcludedProviderView()=default;
    FExcludedProviderView(TConstArrayView<int32> values):Explicit(values){}
    template<typename Element,typename Size> FExcludedProviderView(TArrayView<Element,Size> values):Explicit(values){}
    template<typename Allocator> FExcludedProviderView(const TArray<int32,Allocator>& values):Explicit(values){}
    bool IsCompact()const{return Owner!=nullptr;}
    int32 Num()const{return IsCompact()?ProviderCount-Included.Num():Explicit.Num();}
    bool IsEmpty()const{return Num()==0;}
    TConstArrayView<int32> GetExplicitView()const{check(!IsCompact());return Explicit;}
    bool IsValidIndex(int32 i)const{return i>=0&&i<Num();}
    const FReferenceCandidateSet* GetCompactOwner()const{return Owner;}
    bool SharesIncludedStorage(TConstArrayView<int32> values)const
    {return IsCompact()&&values.GetData()==Included.GetData()&&values.Num()==Included.Num();}
    bool SameStorage(const FExcludedProviderView& other)const
    {
        if(IsCompact()!=other.IsCompact()||Num()!=other.Num())return false;
        return IsCompact()?Owner==other.Owner&&ProviderCount==other.ProviderCount&&SharesIncludedStorage(other.Included):
            Explicit.GetData()==other.Explicit.GetData();
    }
    int32 operator[](int32 ordinal)const
    {
        check(IsValidIndex(ordinal));if(!IsCompact())return Explicit[ordinal];
        int32 provider=ordinal;
        for(int32 included:Included){if(included>provider)break;++provider;}
        return provider;
    }
    bool Contains(int32 provider)const
    {
        if(!IsCompact())return Explicit.Contains(provider);
        if(provider<0||provider>=ProviderCount)return false;
        for(int32 included:Included){if(included==provider)return false;if(included>provider)break;}
        return true;
    }
    struct FIterator
    {
        const FExcludedProviderView* View=nullptr;int32 Position=0,NextIncluded=0;
        void SkipIncluded()
        {if(View->IsCompact())while(NextIncluded<View->Included.Num()&&View->Included[NextIncluded]==Position){++NextIncluded;++Position;}}
        int32 operator*()const{return View->IsCompact()?Position:View->Explicit[Position];}
        FIterator& operator++(){++Position;SkipIncluded();return *this;}
        bool operator!=(const FIterator& other)const{return Position!=other.Position;}
    };
    FIterator begin()const{FIterator i{this,0,0};i.SkipIncluded();return i;}
    FIterator end()const{return {this,IsCompact()?ProviderCount:Explicit.Num(),Included.Num()};}
};
namespace CandidateDetail
{
// Exact repeated Consume, including partial failure/floors/Used overflow.
// This only coalesces integer accounting; it never skips a geometry predicate.
inline bool ConsumeRepeated(FDistanceBudget& budget,uint64 count)
{
    if(count==0)return true;
    if(budget.ConsumptionFloor<0||budget.Remaining<=budget.ConsumptionFloor||budget.Used==MAX_uint64)return false;
    const uint64 available=FMath::Min(uint64(int64(budget.Remaining)-budget.ConsumptionFloor),MAX_uint64-budget.Used);
    const uint64 paid=FMath::Min(count,available);
    budget.Remaining-=int32(paid);budget.Used+=paid;return paid==count;
}
inline bool SurfaceHull(const FIntervalTensorSurface& surface,FIntervalPoint& hull)
{
    hull={};
    if(surface.DegreeU<0||surface.DegreeV<0||surface.DegreeU>16||surface.DegreeV>16||
        surface.ControlPoints.Num()!=(surface.DegreeU+1)*(surface.DegreeV+1))return false;
    bool first=true;
    for(const auto& point:surface.ControlPoints)
    {
        if(!Detail::ValidPoint(point))return false;
        hull=first?point:TimeDetail::Hull(hull,point);first=false;
    }
    return !first&&Detail::ValidPoint(hull);
}
inline Interval BoxDistanceSquared(const FIntervalPoint& a,const FIntervalPoint& b)
{
    if(!Detail::ValidPoint(a)||!Detail::ValidPoint(b))return {};
    auto sum=Interval::Exact(0);
    for(int32 d=0;d<3;++d)
    {
        auto gap=Interval::Exact(0);
        if(a.Axis[d].Lower>b.Axis[d].Upper)
            gap=AnalyticCertified::Subtract(Interval::Exact(a.Axis[d].Lower),Interval::Exact(b.Axis[d].Upper));
        else if(b.Axis[d].Lower>a.Axis[d].Upper)
            gap=AnalyticCertified::Subtract(Interval::Exact(b.Axis[d].Lower),Interval::Exact(a.Axis[d].Upper));
        sum=AnalyticCertified::Add(sum,AnalyticCertified::Square(gap));
    }
    return AnalyticCertified::Valid(sum)?sum:Interval{};
}
inline bool PositiveRadius(double radius)
{
    const auto bits=AnalyticCertified::Bits(radius);
    return !(bits&AnalyticCertified::Sign)&&(bits&AnalyticCertified::Magnitude)>=AnalyticCertified::MinNormal&&AnalyticCertified::Finite(radius);
}
}
// Source89: arbitrary finite oblique axis; complete interval projections.
enum class EObliqueCapsule89:uint8 {Separated,Near,Unknown};
namespace CandidateDetail
{
inline EObliqueCapsule89 ObliqueFixedSegmentBox89(const FIntervalBezierCenterPath& path,
    const FIntervalPoint& box,Interval radiusSquared,const FIntervalPoint& direction)
{
    namespace A=AnalyticCertified;
    if(path.Degree!=1||path.ControlPoints.Num()!=2||!CenterDetail::Valid(path)||
        !Detail::ValidPoint(box)||!Detail::ValidPoint(direction)||!A::Valid(radiusSquared)||radiusSquared.Lower<0)
        return EObliqueCapsule89::Unknown;
    const auto& start=path.ControlPoints[0];const auto& end=path.ControlPoints[1];FIntervalPoint offset;
    for(int32 k=0;k<3;++k)
    {
        const auto mid=A::Multiply(A::Add(Interval::Exact(box.Axis[k].Lower),Interval::Exact(box.Axis[k].Upper)),Interval::Exact(.5));
        offset.Axis[k]=A::Subtract(mid,start.Axis[k]);
    }
    const auto cross=[](const FIntervalPoint& a,const FIntervalPoint& b)
    {
        FIntervalPoint r;for(int32 k=0;k<3;++k){const int32 j=(k+1)%3,l=(k+2)%3;
            r.Axis[k]=A::Subtract(A::Multiply(a.Axis[j],b.Axis[l]),A::Multiply(a.Axis[l],b.Axis[j]));}return r;
    };
    const auto chosen=cross(cross(direction,offset),direction);
    if(!Detail::ValidPoint(offset)||!Detail::ValidPoint(chosen))return EObliqueCapsule89::Unknown;
    FIntervalPoint axis;
    // A lower endpoint is just an arbitrary binary64 coefficient. No exact
    // perpendicular, nearest point or normalization is assumed by the proof.
    for(int32 k=0;k<3;++k)axis.Axis[k]=Interval::Exact(chosen.Axis[k].Lower);
    const auto dot=[](const FIntervalPoint& a,const FIntervalPoint& b)
    {auto r=A::Multiply(a.Axis[0],b.Axis[0]);for(int32 k=1;k<3;++k)r=A::Add(r,A::Multiply(a.Axis[k],b.Axis[k]));return r;};
    auto norm=A::Square(axis.Axis[0]);for(int32 k=1;k<3;++k)norm=A::Add(norm,A::Square(axis.Axis[k]));
    if(!A::Valid(norm))return EObliqueCapsule89::Unknown;
    if(!(norm.Upper>0))return EObliqueCapsule89::Near;
    const auto a=dot(axis,start),b=dot(axis,end),p=dot(axis,box);
    if(!A::Valid(a)||!A::Valid(b)||!A::Valid(p))return EObliqueCapsule89::Unknown;
    const Interval segment{FMath::Min(a.Lower,b.Lower),FMath::Max(a.Upper,b.Upper),true};auto gap=Interval::Exact(0);
    if(segment.Lower>p.Upper)gap=A::Subtract(Interval::Exact(segment.Lower),Interval::Exact(p.Upper));
    else if(p.Lower>segment.Upper)gap=A::Subtract(Interval::Exact(p.Lower),Interval::Exact(segment.Upper));
    if(!A::Valid(gap))return EObliqueCapsule89::Unknown;
    if(!(gap.Lower>0))return EObliqueCapsule89::Near;
    const auto left=A::Square(gap),right=A::Multiply(radiusSquared,norm);
    if(!A::Valid(left)||!A::Valid(right))return EObliqueCapsule89::Unknown;
    return left.Lower>right.Upper?EObliqueCapsule89::Separated:EObliqueCapsule89::Near;
}
}

class FIndexedIntervalUnion
{
    TArray<FIntervalTensorSurface> Surfaces;
    TArray<FIntervalPoint> Boxes;
    TSharedPtr<const Collision::FOrderedBoundsIndex> BoundsIndex;
    bool RadiusArithmeticSafe=false;
    uint64 SelectionWorstCaseWork=0; // Immutable provider+CP reserve, built once.
    // Optional geometry cache: one final GT preparation, no source mutation.
    mutable std::atomic<uint8> StaticHierarchyState{0};
    mutable StaticBodyHierarchy::FAtlasPtr StaticHierarchy;
    mutable std::atomic<uint8> StaticTubesState{0};
    mutable StaticBodyTubes::FAtlasPtr StaticTubes;
    TArray<uint64> SelectionWorkPrefix; // N+1 immutable exact provider+CP prefix sums.
    static FBox3d NativeBox(const FIntervalPoint& box)
    {return FBox3d(FVector3d(box.Axis[0].Lower,box.Axis[1].Lower,box.Axis[2].Lower),
                  FVector3d(box.Axis[0].Upper,box.Axis[1].Upper,box.Axis[2].Upper));}
    // Bit-only sufficient arithmetic domain; never a contact tolerance.
    // |x|<2^500 bounds outward differences below2^502, squares below2^1004
    // and three-axis sums below2^1006. No skipped old leaf can then refuse.
    static bool SafeRadiusOperands(const FIntervalPoint& box)
    {
        for(const auto& axis:box.Axis)for(double value:{axis.Lower,axis.Upper})
            if((AnalyticCertified::Bits(value)&AnalyticCertified::Magnitude)>=0x5f30000000000000ULL)return false;
        return true;
    }
    bool ProviderWorkAvailable(const FDistanceBudget& budget) const
    {return budget.ConsumptionFloor>=0 && int64(budget.Remaining)-budget.ConsumptionFloor>=Num() &&
        uint64(Num())<=MAX_uint64-budget.Used;}
    static bool RadiusEnvelope(const FIntervalPoint& center,double radius,FIntervalPoint& out)
    {
        out={};
        // If old gap.Lower<=r, RNE_FTZ(gap)<=nextUp(r), hence the real gap
        // is <nextUp(nextUp(r)). This also covers flushed zero/subnormal gaps
        // because r>=MinNormal. Outward Add/Subtract enclose the resulting
        // center expansion. No old candidate is removed, including tangency.
        const double padded=AnalyticCertified::FromBits(AnalyticCertified::Bits(radius)+2);
        for(int32 d=0;d<3;++d)
        {
            const auto lo=AnalyticCertified::Subtract(Interval::Exact(center.Axis[d].Lower),Interval::Exact(padded));
            const auto hi=AnalyticCertified::Add(Interval::Exact(center.Axis[d].Upper),Interval::Exact(padded));
            if(!AnalyticCertified::Valid(lo)||!AnalyticCertified::Valid(hi))return false;
            out.Axis[d]={lo.Lower,hi.Upper,true};
        }
        return Detail::ValidPoint(out);
    }
    // Private entry ONLY for Select: its center/radius/r2 are already valid;
    // every Boxes element was validated by Build and is owned/immutable.
    // Same per-provider unit covers no MORE interval arithmetic than before.
    static bool SeparateValidatedCenterRadiusBox(const FIntervalPoint& center,const FIntervalPoint& box,
        double radius,Interval radiusSquared,bool& separate)
    {
        separate=false;auto sum=Interval::Exact(0);
        for(int32 d=0;d<3;++d)
        {
            auto gap=Interval::Exact(0);
            if(center.Axis[d].Lower>box.Axis[d].Upper)
                gap=AnalyticCertified::Subtract(Interval::Exact(center.Axis[d].Lower),Interval::Exact(box.Axis[d].Upper));
            else if(box.Axis[d].Lower>center.Axis[d].Upper)
                gap=AnalyticCertified::Subtract(Interval::Exact(box.Axis[d].Lower),Interval::Exact(center.Axis[d].Upper));
            if(!AnalyticCertified::Valid(gap))return false; // Attempted failure is terminal.
            // NEW necessary separation theorem, before any square is attempted.
            // Strictness retains tangency; no expanded center/overflow fallback.
            if(gap.Lower>radius){separate=true;return true;}
            sum=AnalyticCertified::Add(sum,AnalyticCertified::Square(gap));
            if(!AnalyticCertified::Valid(sum))return false;
        }
        separate=sum.Lower>radiusSquared.Upper;return true;
    }
    enum class EFixedSegmentProof : uint8 {Unknown,Keep,Separated};
    struct FFixedSegmentWork {uint64 Axes=0,Pairs=0;bool HasUncertainAxis=false;};
    // A necessary condition for any sphere center to reach this CP hull is
    // intersection with its radius-expanded coordinate box. Ratios are NEVER
    // evaluated: compare only outward cross products of positive denominators.
    static EFixedSegmentProof SeparateFixedLinearSegmentRadiusBox(
        const FIntervalBezierCenterPath& path,const FIntervalPoint& box,double radius,
        FDistanceBudget& budget,FFixedSegmentWork& work)
    {
        work={};
        if(path.Degree!=1||path.ControlPoints.Num()!=2||!CenterDetail::Valid(path)||
            !Detail::ValidPoint(box)||!CandidateDetail::PositiveRadius(radius)||
            !SafeRadiusOperands(path.ControlPoints[0])||!SafeRadiusOperands(path.ControlPoints[1])||
            !SafeRadiusOperands(box)||(AnalyticCertified::Bits(radius)&AnalyticCertified::Magnitude)>=0x5f30000000000000ULL)
            return EFixedSegmentProof::Unknown;
        const auto previous=::Speed::CaptureFrameFloatingPointContext();
        if(!previous.bValid)return EFixedSegmentProof::Unknown;
        ::Speed::FScopedFrameFloatingPointContext numeric(previous);
        if(!numeric.IsApplied())return EFixedSegmentProof::Unknown;
        const auto prove=[&]()
        {
            namespace A=::AnalyticCertified;
            struct FRatio {double Numerator=0,Denominator=1;};
            FRatio entry[4]{{0,1}},exit[4]{{1,1}};int32 count=1;
            for(int32 axis=0;axis<3;++axis)
            {
                if(!budget.Consume())return EFixedSegmentProof::Unknown;
                ++work.Axes;
                auto start=path.ControlPoints[0].Axis[axis];
                const auto end=path.ControlPoints[1].Axis[axis];
                auto direction=A::Subtract(end,start);
                const auto low=A::Subtract(Interval::Exact(box.Axis[axis].Lower),Interval::Exact(radius));
                const auto high=A::Add(Interval::Exact(box.Axis[axis].Upper),Interval::Exact(radius));
                if(!A::Valid(direction)||!A::Valid(low)||!A::Valid(high))return EFixedSegmentProof::Unknown;
                Interval expanded{low.Lower,high.Upper,true};
                if(direction.Lower<=0&&direction.Upper>=0)
                {work.HasUncertainAxis=true;continue;}
                if(direction.Upper<0)
                {start=A::Negate(start);direction=A::Negate(direction);expanded=A::Negate(expanded);}
                if(!A::Valid(start)||!A::Valid(direction)||!A::Valid(expanded)||!(direction.Lower>0))
                    return EFixedSegmentProof::Unknown;
                const auto lo=A::Subtract(Interval::Exact(expanded.Lower),start);
                const auto hi=A::Subtract(Interval::Exact(expanded.Upper),start);
                if(!A::Valid(lo)||!A::Valid(hi))return EFixedSegmentProof::Unknown;
                entry[count]={lo.Lower,lo.Lower>=0?direction.Upper:direction.Lower};
                exit[count]={hi.Upper,hi.Upper>=0?direction.Lower:direction.Upper};++count;
            }
            // At most4x4 comparisons, each charged before its arithmetic. Bounds
            // |input|<2^500 put cross terms conservatively below2^1008.
            for(int32 i=0;i<count;++i)for(int32 j=0;j<count;++j)
            {
                if(!budget.Consume())return EFixedSegmentProof::Unknown;
                ++work.Pairs;
                const auto left=A::Multiply(Interval::Exact(entry[i].Numerator),Interval::Exact(exit[j].Denominator));
                const auto right=A::Multiply(Interval::Exact(exit[j].Numerator),Interval::Exact(entry[i].Denominator));
                const auto difference=A::Subtract(left,right);
                if(!A::Valid(left)||!A::Valid(right)||!A::Valid(difference))return EFixedSegmentProof::Unknown;
                if(difference.Lower>0)return EFixedSegmentProof::Separated;
            }
            return EFixedSegmentProof::Keep;
        };
        const auto result=prove();
        // Novel auxiliary arithmetic must not add or erase masked sticky flags
        // in the original selector/consumer. Failure to restore is a refusal.
        if(!numeric.Restore())return EFixedSegmentProof::Unknown;
        return result;
    }

    FIndexedIntervalUnion()=default;
    friend class FReferenceCandidateSet;
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    friend class FReferenceCandidateInventory;
#endif
    friend class FReferenceBoundsCandidateSet;
public:
    static bool Build(TConstArrayView<FIntervalTensorSurface> input,FDistanceBudget& budget,FIndexedIntervalUnionPtr& out)
    {
        out.Reset();RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(input.IsEmpty()||budget.Remaining<=0||input.Num()>budget.Remaining)return false;
        TSharedPtr<FIndexedIntervalUnion,ESPMode::ThreadSafe> result(new FIndexedIntervalUnion);
        result->SelectionWorkPrefix.Add(0);
        for(const auto& surface:input)
        {
            if(!budget.Consume())return false;
            // Validate sizes before reading/copying the bounded net. Charge CPs
            // as well as providers; there is no large hidden unbudgeted copy.
            if(surface.DegreeU<0||surface.DegreeV<0||surface.DegreeU>16||surface.DegreeV>16||
                surface.ControlPoints.Num()!=(surface.DegreeU+1)*(surface.DegreeV+1))return false;
            for(const auto& point:surface.ControlPoints)
                if(!budget.Consume()||!Detail::ValidPoint(point))return false;
            FIntervalPoint box;if(!CandidateDetail::SurfaceHull(surface,box))return false;
            result->Surfaces.Add(surface);result->Boxes.Add(box);
            result->SelectionWorstCaseWork+=1u+uint64(surface.ControlPoints.Num());
            result->SelectionWorkPrefix.Add(result->SelectionWorstCaseWork);
        }
        // ACTUAL validated CP hulls, not nominal/faceted approximation boxes.
        // Balanced construction/query have <=2N nodes. Their bounded lookup
        // is included in provider work; existing provider/CP debit points stay.
        TArray<FBox3d> bounds;bounds.Reserve(result->Num());result->RadiusArithmeticSafe=true;
        for(const auto& box:result->Boxes){bounds.Add(NativeBox(box));result->RadiusArithmeticSafe&=SafeRadiusOperands(box);}
        result->BoundsIndex=Collision::FOrderedBoundsIndex::CreateBvh(bounds);
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        out=result;return true;
    }
    int32 Num() const { return Surfaces.Num(); }
    IAMSPEED_API void PrepareStaticBodyHierarchyAtPublication(FDistanceBudget&) const;
    StaticBodyHierarchy::FAtlasPtr GetStaticBodyHierarchy()const
    {
        // Never read the sharedptr while its sole writer is preparing it.
        if(StaticHierarchyState.load(std::memory_order_acquire)!=2)return {};
        return StaticHierarchy;
    }
    IAMSPEED_API uint64 PrepareStaticBodyTubesAtPublication(TConstArrayView<StaticBodyTubes::FInput>,FDistanceBudget&,uint64 maximumBytes) const;
    StaticBodyTubes::FAtlasPtr GetStaticBodyTubes()const
    {if(StaticTubesState.load(std::memory_order_acquire)!=2)return {};return StaticTubes;}
    // Immutable COMPLETE CP hull inventory. The same N logical provider
    // units as Select are paid before BVH lookup; no partition/filter is
    // dropped, no provider/CP is copied and touching bounds remain Near.
    EEmptyBounds91 EmptyBoundsProof91(const FIntervalPoint& hull,FDistanceBudget& budget) const
    {
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!Detail::ValidPoint(hull))return EEmptyBounds91::Unknown;
        for(int32 i=0;i<Num();++i)if(!budget.Consume())return EEmptyBounds91::Unknown;
        if(!BoundsIndex)return EEmptyBounds91::Unknown;
        Collision::FOrderedBoundsScratch scratch;
        const bool empty=BoundsIndex->FindCandidates(NativeBox(hull),scratch).IsEmpty();
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        return empty?EEmptyBounds91::Clear:EEmptyBounds91::Near;
    }
    // Bounded immutable coefficient access for canonical query implementations.
    // This copies geometry only, never World policy or native query authority.
    bool CopySurfaceForQuery(int32 index,FDistanceBudget& budget,FIntervalTensorSurface& out) const
    {
        out={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!Surfaces.IsValidIndex(index)||int64(Surfaces[index].ControlPoints.Num())+1>budget.Remaining||!budget.Consume())return false;
        FIntervalTensorSurface copy;copy.DegreeU=Surfaces[index].DegreeU;copy.DegreeV=Surfaces[index].DegreeV;
        for(const auto& point:Surfaces[index].ControlPoints)
        {if(!budget.Consume())return false;copy.ControlPoints.Add(point);}
        out=MoveTemp(copy);return true;
    }
    // Exact bounded comparison of owned CP intervals. Packet/digest equality
    // alone never proves that another producer supplied the same geometry.
    bool SameCoefficients(const FIndexedIntervalUnion& other,FDistanceBudget& budget) const
    {
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(Surfaces.Num()!=other.Surfaces.Num())return false;
        for(int32 i=0;i<Surfaces.Num();++i)
        {
            if(!budget.Consume())return false;
            const auto& a=Surfaces[i];const auto& b=other.Surfaces[i];
            if(a.DegreeU!=b.DegreeU||a.DegreeV!=b.DegreeV||a.ControlPoints.Num()!=b.ControlPoints.Num())return false;
            for(int32 k=0;k<a.ControlPoints.Num();++k)
            {
                if(!budget.Consume())return false;
                for(int32 d=0;d<3;++d)
                {
                    const auto& x=a.ControlPoints[k].Axis[d];const auto& y=b.ControlPoints[k].Axis[d];
                    if(x.Known!=y.Known||AnalyticCertified::Bits(x.Lower)!=AnalyticCertified::Bits(y.Lower)||
                        AnalyticCertified::Bits(x.Upper)!=AnalyticCertified::Bits(y.Upper))return false;
                }
            }
        }
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);return true;
    }

};
// Checked constructor is the ONLY publisher. It captures scene, path, radius
// and the complete partition. Default/refused/interrupted scans publish null.
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
// Complete ordered broadphase inventory of the original CP family. It owns
// only indices and a two-point path, never copied surface networks. This is
// not a contact, a miss, a filter, or an exact narrowphase certificate.
class FReferenceCandidateInventory final
{
    FIndexedIntervalUnionPtr Scene;
    FIntervalBezierCenterPath Path;
    double Radius=0;
    uint64 Numeric=0;
    TArray<int32> Indices;
    FReferenceCandidateInventory()=default;
public:
    static bool Select(FIndexedIntervalUnionPtr scene,const FIntervalBezierCenterPath& path,double radius,
        FDistanceBudget& budget,TSharedPtr<const FReferenceCandidateInventory,ESPMode::ThreadSafe>& output,
        FReferenceSelectionWork* work=nullptr)
    {
        output.Reset();if(work)*work={};
        const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(numeric.Status!=EAnalyticNumericStatus::Complete||!scene||path.Degree!=1||path.ControlPoints.Num()!=2||
            !CenterDetail::Valid(path)||!CandidateDetail::PositiveRadius(radius)||
            !scene->BoundsIndex||!scene->RadiusArithmeticSafe||!scene->ProviderWorkAvailable(budget))return false;
        const auto center=CenterDetail::Hull(path);
        if(!FIndexedIntervalUnion::SafeRadiusOperands(center)||
            (AnalyticCertified::Bits(radius)&AnalyticCertified::Magnitude)>=0x5f30000000000000ULL)return false;
        const auto r2=AnalyticCertified::Square(Interval::Exact(radius));FIntervalPoint envelope;
        if(!AnalyticCertified::Valid(r2)||!FIndexedIntervalUnion::RadiusEnvelope(center,radius,envelope))return false;
        // The same N-unit bound as the original indexed selector pays the
        // immutable <=2N-node lookup, before any lookup or result allocation.
        if(!CandidateDetail::ConsumeRepeated(budget,uint64(scene->Num())))return false;
        Collision::FOrderedBoundsScratch scratch;
        const auto ordered=scene->BoundsIndex->FindCandidates(FIndexedIntervalUnion::NativeBox(envelope),scratch);
        if(work)*work={scratch.NodesVisited,scratch.ProviderBoundsTested,0,true};
        TSharedPtr<FReferenceCandidateInventory,ESPMode::ThreadSafe> result(new FReferenceCandidateInventory);
        result->Scene=scene;result->Path=path;result->Radius=radius;result->Numeric=numeric.ControlFingerprint();
        int32 prior=INDEX_NONE;
        for(int32 index:ordered)
        {
            if(index<=prior||!scene->Boxes.IsValidIndex(index)||!budget.Consume())return false;
            prior=index;bool separated=false;if(work)++work->RadiusPredicates;
            if(!FIndexedIntervalUnion::SeparateValidatedCenterRadiusBox(center,scene->Boxes[index],radius,r2,separated))return false;
            // Strict separation only, identical radius predicate. Unknown
            // provider kinds remain in this list for the caller to refuse.
            if(!separated)result->Indices.Add(index);
        }
        const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=result->Numeric)return false;
        output=MoveTemp(result);return true;
    }
    const FIndexedIntervalUnionPtr& GetScene()const{return Scene;}
    const FIntervalBezierCenterPath& GetPathForFixedSource()const{return Path;}
    double GetRadiusForFixedSource()const{return Radius;}
    TConstArrayView<int32> GetCandidateIndices()const{return Indices;}
    bool HasCurrentNumericControls()const
    {
        const auto now=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        return now.Status==EAnalyticNumericStatus::Complete&&now.ControlFingerprint()==Numeric;
    }
    bool AccountOwnedQueryStorageBytes(FDistanceBudget& budget,uint64& output)const
    {
        output=0;if(!budget.Consume())return false;
        uint64 bytes=sizeof(FReferenceCandidateInventory);
        for(uint64 extra:{uint64(Path.ControlPoints.GetAllocatedSize()),uint64(Indices.GetAllocatedSize())})
        {if(extra>MAX_uint64-bytes)return false;bytes+=extra;}
        output=bytes;return true;
    }
};
using FReferenceCandidateInventoryPtr=TSharedPtr<const FReferenceCandidateInventory,ESPMode::ThreadSafe>;
#endif


class FReferenceCandidateSet
{
    FIndexedIntervalUnionPtr Scene;
    FIntervalBezierCenterPath Path;
    double Radius=0;
    TArray<int32> CandidateIndices,ExcludedIndices;
    bool CompactExclusions=false;
    TArray<FIntervalTensorSurface> CandidateSurfaces;
    FReferenceCandidateSet()=default;
public:
    // Only a proved-empty full capsule may publish this ordinary complete
    // candidate partition. RadiusEnvelope is rederived from the ACTUAL path,
    // not trusted from a caller's hull. +1 bounded setup unit, then N units
    // in EmptyBoundsProof91 also cover the N original-order excluded writes.
    static EEmptyBounds91 SelectInheritedEmpty91(FIndexedIntervalUnionPtr scene,
        const FIntervalBezierCenterPath& path,double radius,const FIntervalPoint& reach,
        FDistanceBudget& budget,FReferenceCandidateSetPtr& out)
    {
        out.Reset();
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!scene||!CenterDetail::Valid(path)||!CandidateDetail::PositiveRadius(radius)||
            !Detail::ValidPoint(reach)||!budget.Consume())return EEmptyBounds91::Unknown;
        FIntervalPoint capsule;
        if(!FIndexedIntervalUnion::RadiusEnvelope(CenterDetail::Hull(path),radius,capsule))return EEmptyBounds91::Unknown;
        for(int32 d=0;d<3;++d)
            if(reach.Axis[d].Lower>capsule.Axis[d].Lower||reach.Axis[d].Upper<capsule.Axis[d].Upper)return EEmptyBounds91::Near;
        const auto status=scene->EmptyBoundsProof91(reach,budget);
        if(status!=EEmptyBounds91::Clear)return status;
        TSharedPtr<FReferenceCandidateSet,ESPMode::ThreadSafe> result(new FReferenceCandidateSet);
        result->Scene=scene;result->Path=path;result->Radius=radius;
        result->ExcludedIndices.Reserve(scene->Num());
        for(int32 i=0;i<scene->Num();++i)result->ExcludedIndices.Add(i);
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        out=result;return EEmptyBounds91::Clear;
    }
    static bool Select(FIndexedIntervalUnionPtr scene,const FIntervalBezierCenterPath& path,double radius,
        FDistanceBudget& budget,FReferenceCandidateSetPtr& out,FReferenceSelectionWork* work=nullptr,bool planarEdgeCull=false,
        bool compactPartition=false)
    {
        out.Reset();if(work)*work={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!scene||!CenterDetail::Valid(path)||!CandidateDetail::PositiveRadius(radius)||
            budget.Remaining<=0||scene->Num()>budget.Remaining)return false;
        const auto center=CenterDetail::Hull(path);const auto r2=AnalyticCertified::Square(Interval::Exact(radius));
        if(!Detail::ValidPoint(center)||!AnalyticCertified::Valid(r2))return false;
        TSharedPtr<FReferenceCandidateSet,ESPMode::ThreadSafe> result(new FReferenceCandidateSet);
        result->Scene=scene;result->Path=path;result->Radius=radius;
        Collision::FOrderedBoundsScratch scratch;TConstArrayView<int32> ordered;int32 next=0;
        const bool indexed=scene->BoundsIndex && scene->RadiusArithmeticSafe &&
            FIndexedIntervalUnion::SafeRadiusOperands(center) &&
            (AnalyticCertified::Bits(radius)&AnalyticCertified::Magnitude)<0x5f30000000000000ULL &&
            scene->ProviderWorkAvailable(budget);
        if(indexed)
        {
            FIntervalPoint envelope;
            if(!FIndexedIntervalUnion::RadiusEnvelope(center,radius,envelope))return false;
            ordered=scene->BoundsIndex->FindCandidates(FIndexedIntervalUnion::NativeBox(envelope),scratch);
            if(work)*work={scratch.NodesVisited,scratch.ProviderBoundsTested,0,true};
        }
        if(indexed&&compactPartition)
        {
            result->CompactExclusions=true;int32 firstUnpaid=0;
            // FindCandidates returns increasing original provider indices. A
            // skipped gap is certified outside by this SAME BVH envelope.
            for(int32 i:ordered)
            {
                if(i<firstUnpaid||i>=scene->Num())return false;
                if(!CandidateDetail::ConsumeRepeated(budget,uint64(i-firstUnpaid))||!budget.Consume())return false;
                const uint64 remainingBase=scene->SelectionWorstCaseWork-scene->SelectionWorkPrefix[i]-1;
                bool separate=false;if(work)++work->RadiusPredicates;
                if(!FIndexedIntervalUnion::SeparateValidatedCenterRadiusBox(center,scene->Boxes[i],radius,r2,separate))return false;
                constexpr uint64 edgeWork=512;
                const bool edgeReserve=budget.ConsumptionFloor>=0&&
                    int64(budget.Remaining)-budget.ConsumptionFloor>=0&&
                    uint64(int64(budget.Remaining)-budget.ConsumptionFloor)>=edgeWork+remainingBase&&
                    budget.Used<=MAX_uint64-edgeWork-remainingBase;
                if(!separate&&planarEdgeCull&&edgeReserve&&
                    !Detail::SeparatePlanarEdgeHull(scene->Surfaces[i],center,r2,budget,separate))return false;
                if(!separate)
                {
                    for(int32 cp=0;cp<scene->Surfaces[i].ControlPoints.Num();++cp)if(!budget.Consume())return false;
                    result->CandidateIndices.Add(i);result->CandidateSurfaces.Add(scene->Surfaces[i]);
                }
                firstUnpaid=i+1;
            }
            if(!CandidateDetail::ConsumeRepeated(budget,uint64(scene->Num()-firstUnpaid)))return false;
            RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
            out=result;return true;
        }
        uint64 remainingBase=scene->SelectionWorstCaseWork;
        for(int32 i=0;i<scene->Num();++i)
        {
            if(!budget.Consume())return false;
            --remainingBase;
            bool separate=false;
            const bool outside=indexed && (next>=ordered.Num()||ordered[next]!=i);
            if(outside)separate=true;
            else
            {
                if(indexed)++next;if(work)++work->RadiusPredicates;
                if(!FIndexedIntervalUnion::SeparateValidatedCenterRadiusBox(center,scene->Boxes[i],radius,r2,separate))return false;
            }
            // The optional support proof may only spend surplus work. Keep
            // enough for every remaining original provider and CP, including
            // this candidate. A small budget retains the original partition.
            constexpr uint64 edgeWork=512;
            const bool edgeReserve=budget.ConsumptionFloor>=0&&
                int64(budget.Remaining)-budget.ConsumptionFloor>=0&&
                uint64(int64(budget.Remaining)-budget.ConsumptionFloor)>=edgeWork+remainingBase&&
                budget.Used<=MAX_uint64-edgeWork-remainingBase;
            if(!separate&&planarEdgeCull&&edgeReserve&&
                !Detail::SeparatePlanarEdgeHull(scene->Surfaces[i],center,r2,budget,separate))return false;
            // Complete canonical partition, same provider and CP debit points.
            if(separate)result->ExcludedIndices.Add(i);
            else
            {
                for(int32 cp=0;cp<scene->Surfaces[i].ControlPoints.Num();++cp)if(!budget.Consume())return false;
                result->CandidateIndices.Add(i);result->CandidateSurfaces.Add(scene->Surfaces[i]);
            }
            remainingBase-=uint64(scene->Surfaces[i].ControlPoints.Num());
        }
        if(result->CandidateIndices.Num()+result->ExcludedIndices.Num()!=scene->Num())return false;
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        out=result;return true;
    }

    // Explicit private-caller opt-in only. General Select above is unchanged.
    // Degree1 is required; no curved/consumed/rotating motion is relabelled.
    static bool SelectFixedLinearSphere(FIndexedIntervalUnionPtr scene,const FIntervalBezierCenterPath& path,double radius,
        FDistanceBudget& budget,FReferenceCandidateSetPtr& out,FReferenceSelectionWork* work=nullptr)
    {
        out.Reset();if(work)*work={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!scene||path.Degree!=1||path.ControlPoints.Num()!=2||!CenterDetail::Valid(path)||!CandidateDetail::PositiveRadius(radius)||
            budget.Remaining<=0||scene->Num()>budget.Remaining)return false;
        const auto center=CenterDetail::Hull(path);const auto r2=AnalyticCertified::Square(Interval::Exact(radius));
        if(!Detail::ValidPoint(center)||!AnalyticCertified::Valid(r2))return false;
        TSharedPtr<FReferenceCandidateSet,ESPMode::ThreadSafe> result(new FReferenceCandidateSet);
        result->Scene=scene;result->Path=path;result->Radius=radius;
        Collision::FOrderedBoundsScratch scratch;TConstArrayView<int32> ordered;int32 next=0;
        const bool indexed=scene->BoundsIndex && scene->RadiusArithmeticSafe &&
            FIndexedIntervalUnion::SafeRadiusOperands(center) &&
            (AnalyticCertified::Bits(radius)&AnalyticCertified::Magnitude)<0x5f30000000000000ULL &&
            scene->ProviderWorkAvailable(budget);
        if(indexed)
        {
            FIntervalPoint envelope;
            if(!FIndexedIntervalUnion::RadiusEnvelope(center,radius,envelope))return false;
            ordered=scene->BoundsIndex->FindCandidates(FIndexedIntervalUnion::NativeBox(envelope),scratch);
            if(work)*work={scratch.NodesVisited,scratch.ProviderBoundsTested,0,true};
        }
        for(int32 i=0;i<scene->Num();++i)
        {
            if(!budget.Consume())return false;
            bool separate=false;
            const bool outside=indexed && (next>=ordered.Num()||ordered[next]!=i);
            if(outside)separate=true;
            else
            {
                if(indexed)++next;if(work)++work->RadiusPredicates;
                if(!FIndexedIntervalUnion::SeparateValidatedCenterRadiusBox(center,scene->Boxes[i],radius,r2,separate))return false;
            }
            // Complete canonical partition, same provider and CP debit points.
            if(separate)result->ExcludedIndices.Add(i);
            else
            {
                if(work)++work->CandidatesBeforeSegment;
                if(indexed)
                {
                    if(work)++work->SegmentTests;
                    FIndexedIntervalUnion::FFixedSegmentWork segmentWork;
                    const auto proof=FIndexedIntervalUnion::SeparateFixedLinearSegmentRadiusBox(path,scene->Boxes[i],radius,budget,segmentWork);
                    if(work){work->SegmentAxes+=segmentWork.Axes;work->SegmentPairs+=segmentWork.Pairs;}
                    if(proof==FIndexedIntervalUnion::EFixedSegmentProof::Unknown)return false;
                    if(proof==FIndexedIntervalUnion::EFixedSegmentProof::Separated)
                    {if(work)++work->SegmentExcluded;result->ExcludedIndices.Add(i);continue;}
                    if(work)++work->SegmentUncertain;
                }
                else if(work)++work->SegmentInapplicable;
                if(work)++work->CandidatesAfterSegment;
                for(int32 cp=0;cp<scene->Surfaces[i].ControlPoints.Num();++cp)if(!budget.Consume())return false;
                result->CandidateIndices.Add(i);result->CandidateSurfaces.Add(scene->Surfaces[i]);
            }
        }
        if(result->CandidateIndices.Num()+result->ExcludedIndices.Num()!=scene->Num())return false;
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        out=result;return true;
    }

    struct FClosedPlaneOriginWitness
    {
        int32 ProviderIndex=INDEX_NONE;
        FVector2d UV=FVector2d::ZeroVector;
    };
    // A restricted EXACT theorem on the retained CPs, including closed seams.
    // Every candidate lies in the coordinate plane through zero; at least one
    // bilinear boundary edge has antipodal singleton endpoints, hence contains
    // the origin exactly at its midpoint. No nominal frame/triangle is used.
    // The owner of the distance witness is NOT a unique physical hit owner.
    bool ProveCoordinatePlaneOrigin(int32 axis,FDistanceBudget& budget,FClosedPlaneOriginWitness& out) const
    {
        out={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!Scene||axis<0||axis>2||CandidateIndices.IsEmpty()||
            CandidateIndices.Num()!=CandidateSurfaces.Num()||!budget.Consume())return false;
        auto zero=[](double x){return (AnalyticCertified::Bits(x)&AnalyticCertified::Magnitude)==0;};
        auto singleton=[&](const Interval& q,double& value)
        {
            if(!AnalyticCertified::Valid(q))return false;
            if(zero(q.Lower)&&zero(q.Upper)){value=0;return true;}
            const auto bits=AnalyticCertified::Bits(q.Lower);
            if(bits!=AnalyticCertified::Bits(q.Upper)||(bits&AnalyticCertified::Magnitude)<AnalyticCertified::MinNormal)return false;
            value=q.Lower;return true;
        };
        // CP order is u-major, then v. Both endpoints and all edge parameters
        // are in the CLOSED domain; an interior-per-provider rule is unsound
        // for ordinary seams in a partition of one canonical finite face.
        constexpr int32 edges[4][2]={{0,1},{2,3},{0,2},{1,3}};
        const FVector2d uv[4]={FVector2d(0,.5),FVector2d(1,.5),FVector2d(.5,0),FVector2d(.5,1)};
        FClosedPlaneOriginWitness found;
        for(int32 i=0;i<CandidateSurfaces.Num();++i)
        {
            if(!budget.Consume()||CandidateIndices[i]<0||CandidateIndices[i]>=Scene->Num())return false;
            const auto& surface=CandidateSurfaces[i];
            for(const auto& cp:surface.ControlPoints)
                if(!budget.Consume()||!Detail::ValidPoint(cp)||!zero(cp.Axis[axis].Lower)||!zero(cp.Axis[axis].Upper))return false;
            if(found.ProviderIndex!=INDEX_NONE||surface.DegreeU!=1||surface.DegreeV!=1||surface.ControlPoints.Num()!=4)continue;
            for(int32 e=0;e<4;++e)
            {
                if(!budget.Consume())return false;bool match=true;
                for(int32 d=0;d<3;++d)
                {
                    double a=0,b=0;
                    if(!singleton(surface.ControlPoints[edges[e][0]].Axis[d],a)||!singleton(surface.ControlPoints[edges[e][1]].Axis[d],b)||
                        (!(zero(a)&&zero(b))&&(AnalyticCertified::Bits(a)^AnalyticCertified::Bits(b))!=AnalyticCertified::Sign))
                    {match=false;break;}
                }
                if(match){found={CandidateIndices[i],uv[e]};break;}
            }
            // Do not return here: EVERY remaining candidate must be coplanar.
        }
        if(found.ProviderIndex==INDEX_NONE)return false;
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        out=found;return true;
    }

    TConstArrayView<int32> GetCandidateIndices() const { return MakeArrayView(CandidateIndices); }
    FExcludedProviderView GetExcludedIndices() const
    {return CompactExclusions?FExcludedProviderView(this,Scene->Num(),MakeArrayView(CandidateIndices)):
        FExcludedProviderView(MakeArrayView(ExcludedIndices));}
    bool HasCompactExclusions()const{return CompactExclusions;}
    const FIndexedIntervalUnionPtr& GetScene() const { return Scene; }
    // Read-only original query material, not an authority or a new publisher.
    const FIntervalBezierCenterPath& GetPathForFixedSource() const {return Path;}
    double GetRadiusForFixedSource() const {return Radius;}
    TConstArrayView<FIntervalTensorSurface> GetSurfacesForFixedSource() const {return MakeArrayView(CandidateSurfaces);}
    // Count only this query object's owned storage, not its shared Scene.
    // Capacities include retained original-order indices, path and copied nets.
    // Caller accounts sharing as an attributed upper bound, not exclusive heap.
    bool AccountOwnedQueryStorageBytes(FDistanceBudget& budget,uint64& output) const
    {
        output=0;if(!budget.Consume())return false;uint64 total=sizeof(*this);
        const auto add=[&](uint64 bytes){if(bytes>MAX_uint64-total)return false;total+=bytes;return true;};
        if(!add(Path.ControlPoints.GetAllocatedSize())||!add(CandidateIndices.GetAllocatedSize())||
            !add(ExcludedIndices.GetAllocatedSize())||!add(CandidateSurfaces.GetAllocatedSize()))return false;
        for(const auto& surface:CandidateSurfaces)
            if(!budget.Consume()||!add(surface.ControlPoints.GetAllocatedSize()))return false;
        output=total;return true;
    }

    FTensorSphereTimeBounds Sweep(FDistanceBudget& budget,double timeGap,double spatialGap,int32 spatialNodesPerAttempt) const
    {
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!budget.Consume()||spatialNodesPerAttempt<=0||!TimeDetail::NonnegativeFinite(timeGap)||
            !TimeDetail::NonnegativeFinite(spatialGap))return {};
        if(CandidateIndices.IsEmpty())
        {
            FTensorSphereTimeBounds result;result.Status=ETensorSphereTimeStatus::NoIntersectionWithProvidedTensors;
            result.LowerTime=result.UpperTime=1;result.InitialSeparationProved=true;return result;
        }
        auto result=BoundTensorSphereSweep(MakeArrayView(CandidateSurfaces),Path,Radius,budget,timeGap,spatialGap,spatialNodesPerAttempt);
        if(result.InsideWitnessSurfaceIndex!=INDEX_NONE)
        {
            if(!CandidateIndices.IsValidIndex(result.InsideWitnessSurfaceIndex))return {};
            result.InsideWitnessSurfaceIndex=CandidateIndices[result.InsideWitnessSurfaceIndex];
        }
        return result;
    }
    // Explicit generic-entry opt-in. Partition and provider order stay immutable.
    // Debit:1 entry +4 direction/radius primitives +54 per attempted provider
    // (<=53 interval/value primitives and one checked provider access).
    // A fixed block is reserved before execution; refused blocks debit nothing.
    EObliqueCapsule89 SeparateAllOblique89(FDistanceBudget& budget,uint64& tested) const
    {
        tested=0;RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!budget.Consume())return EObliqueCapsule89::Unknown;
        if(!Scene||Path.Degree!=1||Path.ControlPoints.Num()!=2||!CenterDetail::Valid(Path)||
            !CandidateDetail::PositiveRadius(Radius))return EObliqueCapsule89::Unknown;
        const auto pay=[&](int32 n)
        {
            if(budget.ConsumptionFloor<0||int64(budget.Remaining)-budget.ConsumptionFloor<n||uint64(n)>MAX_uint64-budget.Used)return false;
            for(int32 k=0;k<n;++k)if(!budget.Consume())return false;return true;
        };
        if(!pay(4))return EObliqueCapsule89::Unknown;
        FIntervalPoint direction;for(int32 k=0;k<3;++k)
            direction.Axis[k]=AnalyticCertified::Subtract(Path.ControlPoints[1].Axis[k],Path.ControlPoints[0].Axis[k]);
        const auto radiusSquared=AnalyticCertified::Square(Interval::Exact(Radius));
        if(!Detail::ValidPoint(direction)||!AnalyticCertified::Valid(radiusSquared))return EObliqueCapsule89::Unknown;
        for(int32 index:CandidateIndices)
        {
            if(!pay(54))return EObliqueCapsule89::Unknown;
            ++tested;if(!Scene->Boxes.IsValidIndex(index))return EObliqueCapsule89::Unknown;
            const auto status=CandidateDetail::ObliqueFixedSegmentBox89(Path,Scene->Boxes[index],radiusSquared,direction);
            if(status!=EObliqueCapsule89::Separated)return status;
        }
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        return EObliqueCapsule89::Separated;
    }
    FTensorSphereContactEnclosure Contact(FDistanceBudget& budget,double timeGap,double spatialGap,int32 spatialNodesPerAttempt,
        double pointWidth,double normalWidth,int32 maximumRefinements=24,
        const FContactEnclosureFilter* filter=nullptr,bool* actualKernelEntered89=nullptr,
        ContactDetail::FInteriorTensorContraction* interiorContraction=nullptr,
        const TimeDetail::FSignedTimeQuery* fixedReferenceClear=nullptr) const
    {
        if(actualKernelEntered89)*actualKernelEntered89=false;
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!budget.Consume()||maximumRefinements<=0||maximumRefinements>24||spatialNodesPerAttempt<=0||!TimeDetail::NonnegativeFinite(timeGap)||
            !TimeDetail::NonnegativeFinite(spatialGap)||!TimeDetail::NonnegativeFinite(pointWidth)||!TimeDetail::NonnegativeFinite(normalWidth))return {};
        if(CandidateIndices.IsEmpty())
        {
            FTensorSphereContactEnclosure result;result.Status=ETensorSphereContactStatus::NoContactWithProvidedTensors;
            result.LowerTime=result.UpperTime=1;return result;
        }
        // SAME kernel; later inside-provider index is never assigned as owner
        // of the first contact enclosure. All first closest points stay covered.
        // The kernel owns LOCAL indices. A callback sees a separately paid,
        // complete copy in the immutable scene's GLOBAL provider order.
        const auto mapForFilter=[&](const FTensorSphereContactEnclosure& provisional)
        {
            if(provisional.Leaves.IsEmpty())return EContactEnclosureDecision::Refuse;
            for(const auto& leaf:provisional.Leaves)
            {
                if(!budget.Consume()||!CandidateIndices.IsValidIndex(leaf.SurfaceIndex))return EContactEnclosureDecision::Refuse;
            }
            auto global=provisional;
            for(auto& leaf:global.Leaves)leaf.SurfaceIndex=CandidateIndices[leaf.SurfaceIndex];
            return (*filter)(global);
        };
        const FContactEnclosureFilter mappedFilter(mapForFilter);
        if(actualKernelEntered89)*actualKernelEntered89=true;
        auto result=BoundTensorSphereFirstContact(MakeArrayView(CandidateSurfaces),Path,Radius,budget,timeGap,spatialGap,
            spatialNodesPerAttempt,pointWidth,normalWidth,maximumRefinements,filter?&mappedFilter:nullptr,
            &ContactDetail::ThreadContactCellGeometryMemo(),interiorContraction,fixedReferenceClear);
        if(result.Status==ETensorSphereContactStatus::FirstContactEnclosed)
        {
            if(result.Leaves.IsEmpty())return {};
            for(auto& leaf:result.Leaves)
            {
                if(!budget.Consume()||!CandidateIndices.IsValidIndex(leaf.SurfaceIndex))return {};
                leaf.SurfaceIndex=CandidateIndices[leaf.SurfaceIndex];
            }
        }
        return result;
    }
};
// Consumed rounded path clearance is a SEPARATE theorem: every center in the
// whole requested time range is strictly separated from every provided net.
// No contact existence/IVT is inferred from this possibly discontinuous DAG.
// This helper deliberately offers no permission to integrate, nor an MTD.
inline bool ProveConsumedCenterRangeClear(const FIndexedIntervalUnionPtr& scene,
    const FZeroOffsetKinematicCenterReference& model,Interval normalizedTime,double radius,
    FDistanceBudget& budget,double spatialGap,int32 spatialNodesPerAttempt)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!scene||!model.IsKnown()||!CandidateDetail::PositiveRadius(radius)||
        !TimeDetail::NonnegativeFinite(spatialGap)||spatialNodesPerAttempt<=0)return false;
    const auto center=BoundConsumedCenterRange(model,normalizedTime);
    if(!Detail::ValidPoint(center))return false;
    // Degree-zero interval CP represents EVERY possible fixed center in this
    // range. Using its full hull to exclude providers is universal, not a sample.
    FIntervalBezierCenterPath hull;hull.ControlPoints.Add(center);
    FReferenceCandidateSetPtr proof;if(!FReferenceCandidateSet::Select(scene,hull,radius,budget,proof))return false;
    const auto r=proof->Sweep(budget,0,spatialGap,spatialNodesPerAttempt);
    return r.Status==ETensorSphereTimeStatus::NoIntersectionWithProvidedTensors;
}
// Same complete-union theorem over exact consumed binary32 seconds. A false
// result is UNKNOWN, never a collision, a clear prefix, or an authority miss.
inline bool ProveConsumedCenterFloatSecondsRangeClear(const FIndexedIntervalUnionPtr& scene,
    const FZeroOffsetKinematicCenterReference& model,float first,float last,double radius,
    FDistanceBudget& budget,double spatialGap,int32 spatialNodesPerAttempt)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!scene||!model.IsKnown()||!CandidateDetail::PositiveRadius(radius)||
        !TimeDetail::NonnegativeFinite(spatialGap)||spatialNodesPerAttempt<=0)return false;
    const auto center=BoundConsumedCenterBetweenFloatSeconds(model,first,last);
    if(!Detail::ValidPoint(center))return false;
    FIntervalBezierCenterPath hull;hull.ControlPoints.Add(center);
    FReferenceCandidateSetPtr proof;
    if(!FReferenceCandidateSet::Select(scene,hull,radius,budget,proof))return false;
    const auto r=proof->Sweep(budget,0,spatialGap,spatialNodesPerAttempt);
    return r.Status==ETensorSphereTimeStatus::NoIntersectionWithProvidedTensors;
}
}
