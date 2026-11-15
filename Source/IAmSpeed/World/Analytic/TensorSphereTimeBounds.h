#pragma once
#include "ContinuousCenterPath.h"
#include "ContinuousCurvePathSupport.h"
#include "HAL/PlatformTime.h"

namespace Speed::Analytic::Prepared { class FReferenceAffineContactPlan; class FReferenceSphereC2ContactPlan; class FReferenceRepeatedSphereFloorPlan; class FReferenceSphereGeneralFirstArrivalRs; }
#include "PlanarFamilyTimeShortcut.h"
namespace Speed::Analytic::Certified
{
// Each net encloses one FIXED real polynomial; all exclusion and inside
// bounds are universal over its coefficient boxes. No existential inside choice.
// Scope is exactly the union of the provided full tensor domains. This is not
// a world query: finite planes, filters, atlas domains and authority are separate.
enum class ETensorSphereTimeStatus : uint8 { Unknown, NoIntersectionWithProvidedTensors, InitialInsideTube, FirstContactBracket };
struct FTensorSphereTimeBounds
{
    ETensorSphereTimeStatus Status=ETensorSphereTimeStatus::Unknown;
    double LowerTime=0,UpperTime=1;
    int32 InsideWitnessSurfaceIndex=INDEX_NONE;
    FVector2d InsideSampleUV=FVector2d::ZeroVector;
    FIntervalPoint InsideSurfacePoint;
    bool InitialSeparationProved=false;
    // This index/UV belongs to the later inside witness, NOT necessarily the
    // earliest touching provider. No boundary contact/normal/MTD/hit is claimed.
};

namespace MonotoneCurveTimeControl
{
IAMSPEED_API bool RuntimeEnabled();
IAMSPEED_API bool DyadicHintEnabled();
IAMSPEED_API void RecordDyadicHint(uint64 nodes,uint64 skips,bool proved);
IAMSPEED_API void PublishObservation();
}
namespace CoordinateLineTimeHintControl
{
IAMSPEED_API bool RuntimeEnabled();
IAMSPEED_API void PublishObservation();
}
namespace ChordEnvelopeTimeHintControl
{
IAMSPEED_API bool RuntimeEnabled();
IAMSPEED_API void PublishObservation();
}
namespace TimeDetail
{
enum class EMonotoneCurveTimeSeed:uint8 {NotApplicable,Proved,Work,Arithmetic};
struct FMonotoneCurveTimeSeed
{
    double Lower=0,Upper=1;
    FIntervalBezierCenterPath RestrictedPath;
    FTensorDistanceBounds InsideWitness;
};
struct FMonotoneCurveTimeInspection
{
    uint32 PrefixAttempts=0,MinimumCalls=0,TimeIterations=0,InsideValidations=0;
    double GlobalCurvatureLower=0,ParameterLower=0,ParameterUpper=1;
};
IAMSPEED_API EMonotoneCurveTimeSeed TryMonotoneCurveTimeSeed(
    TConstArrayView<FIntervalTensorSurface> surfaces,const FIntervalBezierCenterPath& path,double radius,
    FDistanceBudget& budget,double requestedTimeGap,int32 maximumWork,FMonotoneCurveTimeSeed& out,
    FMonotoneCurveTimeInspection* inspection=nullptr);
IAMSPEED_API EMonotoneCurveTimeSeed TryCoordinateLineTimeHint(
    TConstArrayView<FIntervalTensorSurface> surfaces,const FIntervalBezierCenterPath& path,double radius,
    FDistanceBudget& budget,double requestedTimeGap,double& lower,double& upper);
IAMSPEED_API EMonotoneCurveTimeSeed TryChordEnvelopeTimeHint(
    TConstArrayView<FIntervalTensorSurface> surfaces,const FIntervalBezierCenterPath& path,double radius,
    FDistanceBudget& budget,double requestedTimeGap,double& lower,double& upper);
// Inside Certified::TimeDetail.
class FRetainedSphereTimeSearch;
class FOffsetCurveSpatialProof;
using FOffsetCurveSpatialProofPtr=TSharedPtr<const FOffsetCurveSpatialProof,ESPMode::ThreadSafe>;
enum class EDirectCurveSpatialSeed:uint8 {NotApplicable,Proved,Work,Arithmetic};
struct FDirectCurveSpatialCell {Detail::FNode Position,OriginalDu,OriginalDv;};
struct FDirectCurveSpatialSeed
{
    TArray<FDirectCurveSpatialCell,TInlineAllocator<2>> Cells;
    FIntervalPoint CoveredCenters;
    Interval NecessaryV;
};
IAMSPEED_API bool OffsetSpatialDomainRuntimeEnabled();
IAMSPEED_API EMonotoneCurveTimeSeed TryChordEnvelopeTimeHintWithSpatialProof(
    TConstArrayView<FIntervalTensorSurface> surfaces,const FIntervalBezierCenterPath& path,
    double radius,FDistanceBudget& budget,double gap,double& lower,double& upper,
    FOffsetCurveSpatialProofPtr& proof,const FRetainedSphereTimeSearch* owner);
IAMSPEED_API EDirectCurveSpatialSeed PrepareOffsetCurveSpatialSeed(
    const FOffsetCurveSpatialProofPtr& proof,const FRetainedSphereTimeSearch* owner,TConstArrayView<FIntervalTensorSurface> surfaces,
    const FIntervalBezierCenterPath& path,double radius,FDistanceBudget& budget,
    const FIntervalPoint& currentCenterBox,FDirectCurveSpatialSeed& seed);
// Inside Certified::TimeDetail, after the shared FDirectCurveSpatialSeed API.
IAMSPEED_API bool CoordinateLineSpatialDomainRuntimeEnabled();
IAMSPEED_API EDirectCurveSpatialSeed PrepareCoordinateLineSpatialSeed(
    TConstArrayView<FIntervalTensorSurface> surfaces,const FIntervalPoint& centers,
    FDistanceBudget& budget,FDirectCurveSpatialSeed& seed);

// Pure original-curve restriction, shared with a separately proved CoordinateLine domain.
IAMSPEED_API EDirectCurveSpatialSeed BuildDirectCurveSpatialCells(
    const FIntervalTensorSurface& surface,Interval necessaryV,FDistanceBudget& budget,
    TArray<FDirectCurveSpatialCell,TInlineAllocator<2>>& cells);
}
namespace TimeDetail
{
inline bool NonnegativeFinite(double x)
{
    const auto b=AnalyticCertified::Bits(x);
    return AnalyticCertified::Finite(x) && !((b&AnalyticCertified::Sign) && (b&AnalyticCertified::Magnitude));
}
inline FIntervalPoint Hull(const FIntervalPoint& a,const FIntervalPoint& b)
{
    FIntervalPoint p;
    for(int32 d=0;d<3;++d)
        p.Axis[d]={FMath::Min(a.Axis[d].Lower,b.Axis[d].Lower),FMath::Max(a.Axis[d].Upper,b.Axis[d].Upper),true};
    return p;
}
// Insert in TimeDetail. This is a separation proof only. No UV or closest
// point from the difference tensor can be a contact witness.
namespace JointProjectedSpherePrefix
{
IAMSPEED_API bool RuntimeEnabled();
enum class EResult:uint8 {Unresolved,Clear,Unknown};

inline EResult Exclude(const FIntervalTensorSurface& surface,const FIntervalBezierCenterPath& path,
    Interval radiusSquared,FDistanceBudget& budget)
{
    constexpr int32 LocalLimit=256;
    if(path.Degree!=2||path.ControlPoints.Num()!=3||budget.ConsumptionFloor<0||
        budget.Remaining-budget.ConsumptionFloor<3*LocalLimit||
        MAX_uint64-budget.Used<uint64(3*LocalLimit))return EResult::Unresolved;
    // Start with the observed Y-extruded quintic family. The projection
    // only discards a nonnegative squared axis term.
    if(ProjectedDistanceDetail::ExtrusionAxis(surface)!=1)return EResult::Unresolved;
    if(!budget.Consume())return EResult::Unknown;
    const auto profile=ProjectedDistanceDetail::ProjectYZ(surface,1);
    FIntervalTensorSurface delta;
    delta.DegreeU=path.Degree;
    delta.DegreeV=profile.DegreeV;
    // Sum_i B_i(t)=Sum_j B_j(v)=1. The tensor with these coefficients is
    // precisely the projection of Center(t)-Surface(v) on this time cell.
    for(const auto& cp:path.ControlPoints)
    {
        const auto c=ProjectedDistanceDetail::ProjectCenterToYZ(cp,1);
        for(const auto& s:profile.ControlPoints)
        {
            if(!budget.Consume())return EResult::Unknown;
            FIntervalPoint value;
            for(int32 d=0;d<3;++d)value.Axis[d]=AnalyticCertified::Subtract(c.Axis[d],s.Axis[d]);
            if(!Detail::ValidPoint(value))return EResult::Unknown;
            delta.ControlPoints.Add(value);
        }
    }
    const auto result=BoundTensorDistanceSquaredIntervalCenter(delta,Detail::Point(FVector3d::ZeroVector),
        budget,0.,LocalLimit,&radiusSquared);
    if(result.Status==EDistanceBoundStatus::Unknown)return EResult::Unknown;
    return result.LowerSquared>radiusSquared.Upper?EResult::Clear:EResult::Unresolved;
}
}

enum class ESignedTimeDecision : uint8 { NotApplicable, Indeterminate, Clear, Inside, Work, Arithmetic };

struct FSignedTimeInterval
{
    ESignedTimeDecision Decision = ESignedTimeDecision::NotApplicable;
    int32 Provider = INDEX_NONE;
    FVector2d ProjectedUV = FVector2d::ZeroVector;
    FIntervalPoint ProjectedPoint;
    Interval SignedInsideGap;
};

// Only the authenticated affine plan may supply fixed-master signs.
class FSignedTimeQuery final
{
    friend class ::Speed::Analytic::Prepared::FReferenceAffineContactPlan;
    friend class ::Speed::Analytic::Prepared::FReferenceSphereC2ContactPlan;
    friend class ::Speed::Analytic::Prepared::FReferenceRepeatedSphereFloorPlan;
    friend class ::Speed::Analytic::Prepared::FReferenceSphereGeneralFirstArrivalRs;
    using FCallback = TFunctionRef<FSignedTimeInterval(double,double,FDistanceBudget&)>;
    using FClearCallback = TFunctionRef<bool(double,double,FDistanceBudget&)>;
#if !UE_BUILD_SHIPPING
    using FDyadicHintCallback = TFunctionRef<bool(double&,double&,FDistanceBudget&)>;
    struct FDyadicHintStats { uint64 PrefixSkips=0, SplitSkips=0; };
    explicit FSignedTimeQuery(FCallback callback,const FClearCallback* clear=nullptr,bool admittedQuadraticPlanar=false,
        const FDyadicHintCallback* hint=nullptr,FDyadicHintStats* stats=nullptr)
        : Callback(callback),ClearCallback(clear),AdmittedQuadraticPlanar(admittedQuadraticPlanar),DyadicHintCallback(hint),DyadicHintStats(stats) {}
#else
    explicit FSignedTimeQuery(FCallback callback,const FClearCallback* clear=nullptr,bool admittedQuadraticPlanar=false)
        : Callback(callback),ClearCallback(clear),AdmittedQuadraticPlanar(admittedQuadraticPlanar) {}
#endif
    const FCallback Callback;
    const FClearCallback* ClearCallback;
    const bool AdmittedQuadraticPlanar;
#if !UE_BUILD_SHIPPING
    const FDyadicHintCallback* DyadicHintCallback;
    FDyadicHintStats* const DyadicHintStats;
#endif
public:
    FSignedTimeQuery(const FSignedTimeQuery&) = delete;
    FSignedTimeQuery& operator=(const FSignedTimeQuery&) = delete;
    FSignedTimeInterval operator()(double first, double last, FDistanceBudget& budget) const
    { return Callback(first,last,budget); }
    bool HasClearCallback() const {return ClearCallback!=nullptr;}
    bool HasAdmittedQuadraticPlanarFamily() const {return AdmittedQuadraticPlanar;}
    bool TryClearBeforeCoarse(double first,double last,FDistanceBudget& budget) const
    {return ClearCallback&&(*ClearCallback)(first,last,budget);}
#if !UE_BUILD_SHIPPING
    bool TryDyadicHint(double& first,double& last,FDistanceBudget& budget) const
    {return DyadicHintCallback&&(*DyadicHintCallback)(first,last,budget);}
    void NoteDyadicHintSkip(bool prefix) const
    {
        if(!DyadicHintStats)return;
        uint64& count=prefix?DyadicHintStats->PrefixSkips:DyadicHintStats->SplitSkips;
        if(count<MAX_uint64)++count;
    }
#endif
};

struct FUnionBound
{
    bool Known=false,AllClear=false,AnyInside=false;
    int32 SurfaceIndex=INDEX_NONE;
    FTensorDistanceBounds Witness;
    // Signed inside never manufactures an absolute squared-distance bound.
    Interval SignedInsideGap;
};
inline FUnionBound Evaluate(TConstArrayView<FIntervalTensorSurface> surfaces,
    const FIntervalPoint& center,FDistanceBudget& budget,Interval radiusSquared,double spatialGap,int32 spatialNodesPerAttempt,bool stopAtFirstInside=false,
    const FPlanarFamilyTimeShortcutPlan* planeShortcut=nullptr,EPlanarFamilyShortcut* shortcutFailure=nullptr)
{
    if(shortcutFailure)*shortcutFailure=EPlanarFamilyShortcut::NotApplicable;
    FUnionBound result; result.AllClear=true;
    for(int32 i=0;i<surfaces.Num();++i)
    {
        // Provider attempt and its spatial work share the same caller budget.
        if(!budget.Consume())return FUnionBound();
        FTensorDistanceBounds r;
        const auto shortcut=planeShortcut?planeShortcut->TryProvider(surfaces,i,center,radiusSquared,budget,r):
            EPlanarFamilyShortcut::NotApplicable;
        if(shortcut==EPlanarFamilyShortcut::Clear)continue;
        if(shortcut==EPlanarFamilyShortcut::Work||shortcut==EPlanarFamilyShortcut::Arithmetic)
        {if(shortcutFailure)*shortcutFailure=shortcut;return {};}
        if(shortcut!=EPlanarFamilyShortcut::Inside&&shortcut!=EPlanarFamilyShortcut::Bounded)
        {
            if(planeShortcut)planeShortcut->ObserveGenericAttempt();
            r=BoundTensorDistanceSquaredIntervalCenter(surfaces[i],center,budget,spatialGap,spatialNodesPerAttempt,
                &radiusSquared,&Detail::ThreadMinimumMidpointMemo());
        }
        if(r.Status==EDistanceBoundStatus::Unknown)return FUnionBound();
        if(!(r.LowerSquared>radiusSquared.Upper))result.AllClear=false;
        if(r.UpperSquared<radiusSquared.Lower && !result.AnyInside)
        {result.AnyInside=true;result.SurfaceIndex=i;result.Witness=r;}
        if(stopAtFirstInside&&result.AnyInside){result.AllClear=false;result.Known=true;return result;}
    }
    result.Known=true;return result;
}
// A live state is owned by one node. A known whole state can be frozen by
// moving its ownership into a shared const snapshot. Children only materialize
// independent metadata/heaps when their whole-hull proof is actually needed.
// The original Surfaces/Path/Budget stay alive and stable for the whole search.
class FRetainedSphereTimeSearch;
// This value belongs to one immutable search only. It is not a world-query
// result, an owner identity, a contact or an authority token.
struct FTimeEndpointProof
{
    const FRetainedSphereTimeSearch* Owner=nullptr;
    const FDistanceBudget* Account=nullptr;
    FIntervalPoint Center;
    FUnionBound Bound;
    Interval RadiusSquared;
    uint64 NumericControl=0,UsedAtPublication=0;
    int32 RemainingAtPublication=0;
    uint32 RequiredSticky=0;
    const FSignedTimeQuery* SignedSource=nullptr;
    uint64 SignedTimeBits=0;
};
enum class ESignedTimeCost : uint8 { Initial,Coarse,Preclear,Signed,Endpoint,Whole,Count };
struct FSignedTimeCostObservation
{
    struct FRow {uint64 Calls=0,Ticks=0,Work=0;};
    static constexpr int32 RowCount=int32(ESignedTimeCost::Count);
    FRow Rows[RowCount];
    uint64 InsidePublished=0,InsideReused=0,WholeHullLookups=0,WholeHullHits=0,WholeHullStores=0;
    const FDistanceBudget* Account=nullptr;
    bool HasSamples() const {return Rows[int32(ESignedTimeCost::Initial)].Calls!=0;}
};
inline thread_local FSignedTimeCostObservation* SignedTimeCosts=nullptr;
inline FSignedTimeCostObservation* ActiveSignedTimeCosts(const FDistanceBudget& account)
{
    return SignedTimeCosts&&SignedTimeCosts->Account==&account?SignedTimeCosts:nullptr;
}
class FScopedSignedTimeCosts
{
    FSignedTimeCostObservation* Previous=SignedTimeCosts;
public:
    explicit FScopedSignedTimeCosts(FSignedTimeCostObservation* observation) {SignedTimeCosts=observation;}
    ~FScopedSignedTimeCosts(){SignedTimeCosts=Previous;}
    FScopedSignedTimeCosts(const FScopedSignedTimeCosts&)=delete;
    FScopedSignedTimeCosts& operator=(const FScopedSignedTimeCosts&)=delete;
};
class FScopedSignedTimeCostRow
{
    const FDistanceBudget& Account;
    FSignedTimeCostObservation::FRow* Row=nullptr;
    uint64 Start=0,Work=0;
public:
    FScopedSignedTimeCostRow(ESignedTimeCost stage,const FDistanceBudget& account):Account(account)
    {
        if(auto* observation=ActiveSignedTimeCosts(account))
        {
            Row=&observation->Rows[int32(stage)];
            Start=FPlatformTime::Cycles64();Work=Account.Used;
        }
    }
    ~FScopedSignedTimeCostRow()
    {
        if(Row)
        {
            const auto end=FPlatformTime::Cycles64();
            ++Row->Calls;
            if(end>=Start)Row->Ticks+=end-Start;
            if(Account.Used>=Work)Row->Work+=Account.Used-Work;
        }
    }
    FScopedSignedTimeCostRow(const FScopedSignedTimeCostRow&)=delete;
    FScopedSignedTimeCostRow& operator=(const FScopedSignedTimeCostRow&)=delete;
};
template<class F>
auto ObserveSignedTimeCost(ESignedTimeCost stage,const FDistanceBudget& account,F&& call)
{
    FScopedSignedTimeCostRow cost(stage,account);
    return call();
}
struct FTimeEndpointCounters
{
    uint64 Requests=0,Evaluations=0,Hits=0,Publications=0,RightInheritances=0;
};
class FUnionSpatialState
{
    friend class FRetainedSphereTimeSearch;
    const FIntervalBezierCenterPath* OriginalPath=nullptr;
    const FIntervalTensorSurface* Surfaces=nullptr;
    int32 ProviderCount=0;
    FIntervalPoint CertifiedCenterHull;
    TArray<TUniquePtr<FRetainedTensorMinimum>> Minimums;
public:
    FUnionSpatialState()=default;
    const FRetainedTensorMinimum* FrozenMinimum(int32 i) const{return Minimums[i].Get();}
};
struct FTimeNode
{
    double A=0,B=1;int32 Depth=0;
    FIntervalBezierCenterPath Curve;
    bool ForceSubdivision=false; // Only an already evaluated unresolved anchor.
    TUniquePtr<FUnionSpatialState> Spatial;
    TSharedPtr<const FUnionSpatialState> SpatialSeed;
    FIntervalPoint SeedCenterHull;
    bool RetainSpatial=false; // Enabled only by a forced unresolved anchor.
    TOptional<FTimeEndpointProof> Endpoint; // Inline, bounded by the existing live time frontier.
};
enum class EDirectTimeFrontier:uint8 {NotApplicable,Ready,Work,Arithmetic};
IAMSPEED_API bool DirectInitialTimeFrontierRuntimeEnabled();
IAMSPEED_API EDirectTimeFrontier BuildDirectInitialTimeFrontier(
    const FIntervalBezierCenterPath& originalPath,double lower,double upper,FDistanceBudget& budget,FTimeNode& output);
IAMSPEED_API void RecordDirectInitialTimeFrontier(bool applied,uint64 nodes);

}
// One operation-local search retains the unresolved frontier and its strict
// inside witness. A smaller gap revisits that SAME frontier; it never repeats
// initialization, replaces the budget, or moves an unresolved anchor forward.
namespace TimeDetail
{
enum class ETimeSearchFailure:uint8 {None,Input,InitialUnknown,InitialThreshold,Work,Arithmetic,AnchorGap,WholeUnknown,EndpointUnknown,Dyadic,NoLaterWitness,NonNested,RepeatedGap,Numeric,RetainedSpatial,PlaneShortcutWork,PlaneShortcutArithmetic};
IAMSPEED_API bool CertifiedPrefixClearRuntimeEnabled();
IAMSPEED_API void RecordCertifiedPrefixClear(uint64 nodes);
class FRetainedSphereTimeSearch final
{
    const TConstArrayView<FIntervalTensorSurface> Surfaces;
    const FIntervalBezierCenterPath& Path;
    const double Radius,SpatialGap;
    const int32 SpatialNodes;
    FDistanceBudget& Budget;
    const FPlanarFamilyTimeShortcutPlan* PlaneShortcut=nullptr;
    EPlanarFamilyShortcut ShortcutFailure=EPlanarFamilyShortcut::NotApplicable;
    // Declared before retained nodes so their borrowed memo outlives them.
    TUniquePtr<FRetainedTensorMinimum::FWholeGeometryHullMemo> WholeGeometryHulls;
    TArray<FTimeNode> Stack,Deferred;
    FTensorSphereTimeBounds Last;
    bool Initialized=false,Terminal=false,HadBracket=false;
    double LastGap=0;
    uint64 NumericControl=0;
    const bool EnableEndpointMemo,EnableMonotoneTimeSeed,EnableMonotoneDyadicHint,EnableCoordinateLineHint,EnableChordEnvelopeHint;
    bool DyadicHintProved=false;double DyadicHintLower=0,DyadicHintUpper=1;
    uint64 TimeNodeVisits=0,DyadicHintSkips=0,CertifiedPrefixSkips=0;
    bool EnableCertifiedPrefixClear=false,EnableOffsetSpatialSeed=false;
#if !UE_BUILD_SHIPPING
    bool SignedDyadicHintProved=false;
#endif
    bool EnableDirectInitialFrontier=false,DirectInitialFrontierApplied=false;
    FOffsetCurveSpatialProofPtr OffsetSpatialProof;
    double InitialClearPrefix=0;
    bool SeedHasStrictInside=false;
    FTimeEndpointCounters EndpointCounters;
#if WITH_DEV_AUTOMATION_TESTS
    friend struct FTimeEndpointMemoTestAccess;
    friend struct FMonotoneTimeSeedTestAccess;
    friend struct FMonotoneDyadicHintTestAccess;
#endif
    const FSignedTimeQuery* SignedTimeQuery=nullptr;
    bool SignedSawInside=false;
    TUniquePtr<FContinuousCurvePathSupport> CurveSupport; // Experimental exclusion only; disabled on production paths.
    uint64 CurveSupportAttempts=0,CurveSupportClearedCells=0; // Passive inspection only.
    ETimeSearchFailure Failure=ETimeSearchFailure::None;
    int32 AnchorRefinements=0;
    // Passive last time-node failure site; never consulted by a proof branch.
    uint8 DyadicSite=0;int32 DyadicDepth=-1;double DyadicA=0,DyadicB=0;
    void NoteDyadic(uint8 site,const FTimeNode& node)
    {DyadicSite=site;DyadicDepth=node.Depth;DyadicA=node.A;DyadicB=node.B;}
    ETimeSearchFailure FailureFor(ETimeSearchFailure original) const
    {
        if(ShortcutFailure==EPlanarFamilyShortcut::Work)return ETimeSearchFailure::PlaneShortcutWork;
        if(ShortcutFailure==EPlanarFamilyShortcut::Arithmetic)return ETimeSearchFailure::PlaneShortcutArithmetic;
        return original;
    }
    FRetainedTensorMinimumCounters SpatialCounters;
    bool AddSpatialCounters(const FRetainedTensorMinimumCounters& before,const FRetainedTensorMinimumCounters& after)
    {
        const auto add=[](uint64& total,uint64 a,uint64 b)
        {if(b<a||total>MAX_uint64-(b-a))return false;total+=b-a;return true;};
        if(!add(SpatialCounters.NewRoots,before.NewRoots,after.NewRoots)||
            !add(SpatialCounters.ReusedLeaves,before.ReusedLeaves,after.ReusedLeaves)||
            !add(SpatialCounters.DirtyBounds,before.DirtyBounds,after.DirtyBounds)||
            !add(SpatialCounters.KKTExcluded,before.KKTExcluded,after.KKTExcluded)||
            !add(SpatialCounters.UVSplits,before.UVSplits,after.UVSplits)||
            !add(SpatialCounters.LocalStops,before.LocalStops,after.LocalStops))return false;
        SpatialCounters.PeakHeapLeaves=FMath::Max(SpatialCounters.PeakHeapLeaves,after.PeakHeapLeaves);return true;
    }
    bool SpatialBinding(const FUnionSpatialState& state) const
    {
        return state.OriginalPath==&Path&&state.Surfaces==Surfaces.GetData()&&
            state.ProviderCount==Surfaces.Num()&&state.Minimums.Num()==Surfaces.Num()&&Detail::ValidPoint(state.CertifiedCenterHull);
    }
    // The child curve was produced only by splitting this SAME original Path.
    // Its outward control hull and the certified parent hull both contain every
    // real descendant center. Their intersection preserves that fact; it does
    // not change a control point, query, clock, witness or polynomial family.
    bool ShareSpatialChildren(FTimeNode& parent,FTimeNode& left,FTimeNode& right,bool forced)
    {
        if(!forced&&!parent.RetainSpatial)return true;
        TSharedPtr<const FUnionSpatialState> seed=parent.SpatialSeed;
        const FIntervalPoint inherited=parent.Spatial?parent.Spatial->CertifiedCenterHull:(seed?parent.SeedCenterHull:FIntervalPoint{});
        if(parent.Spatial)
        {
            if(!SpatialBinding(*parent.Spatial))return false;
            for(const auto& minimum:parent.Spatial->Minimums)
                if(minimum&&!minimum->IsKnownSnapshot())return false;
            // Ownership transfer only: no metadata or heap copy here.
            seed=MakeShareable(parent.Spatial.Release());
        }
        left.RetainSpatial=true;right.RetainSpatial=true;
        if(!seed)return true; // No whole proof has been evaluated in this branch.
        if(!SpatialBinding(*seed))return false;
        // Do not allocate a provider table just to validate a hull. The seed's
        // existing binding is already checked; this narrower hull has genuine
        // internal same-path ancestry and is intersected again below.
        const auto inherit=[&](FTimeNode& child)->bool
        {
            if(!Budget.Consume())return false;
            auto hull=CenterDetail::Hull(child.Curve);
            if(!Detail::ValidPoint(hull)||!Detail::ValidPoint(inherited))return false;
            for(int32 d=0;d<3;++d)
            {
                hull.Axis[d].Lower=FMath::Max(hull.Axis[d].Lower,inherited.Axis[d].Lower);
                hull.Axis[d].Upper=FMath::Min(hull.Axis[d].Upper,inherited.Axis[d].Upper);
            }
            if(!Detail::ValidPoint(hull))return false;
            for(int32 d=0;d<3;++d)
                if(hull.Axis[d].Lower<seed->CertifiedCenterHull.Axis[d].Lower||
                    hull.Axis[d].Upper>seed->CertifiedCenterHull.Axis[d].Upper)return false;
            child.SpatialSeed=seed;child.SeedCenterHull=hull;return true;
        };
        return inherit(left)&&inherit(right);
    }
    FUnionBound EvaluateSpatial(FUnionSpatialState& state,Interval radiusSquared,
        const FUnionSpatialState* seed)
    {
        ShortcutFailure=EPlanarFamilyShortcut::NotApplicable;
        FUnionBound result;result.AllClear=true;
        if(!SpatialBinding(state))return {};
        for(int32 i=0;i<Surfaces.Num();++i)
        {
            if(!Budget.Consume())return {};
            FTensorDistanceBounds bound;
            const auto shortcut=PlaneShortcut?PlaneShortcut->TryProvider(
                Surfaces,i,state.CertifiedCenterHull,radiusSquared,Budget,bound):EPlanarFamilyShortcut::NotApplicable;
            if(shortcut==EPlanarFamilyShortcut::Clear)continue;
            if(shortcut==EPlanarFamilyShortcut::Work||shortcut==EPlanarFamilyShortcut::Arithmetic)
            {ShortcutFailure=shortcut;return {};}
            if(shortcut!=EPlanarFamilyShortcut::Inside&&shortcut!=EPlanarFamilyShortcut::Bounded)
            {
                if(PlaneShortcut)PlaneShortcut->ObserveGenericAttempt();
                if(!state.Minimums[i])
                {
                    if(seed&&seed->FrozenMinimum(i))
                    {
                        const FRetainedTensorMinimum& frozen=*seed->FrozenMinimum(i);
                        ERetainedTensorMinimumFailure why=ERetainedTensorMinimumFailure::None;
                        state.Minimums[i]=frozen.ForkContainedImmutable(state.CertifiedCenterHull,why);
                        if(!state.Minimums[i]||!AddSpatialCounters({},state.Minimums[i]->GetCountersForInspection()))return {};
                    }
                    else
                    {
                        state.Minimums[i]=MakeUnique<FRetainedTensorMinimum>(
                            Surfaces[i],state.CertifiedCenterHull,Budget,SpatialGap,SpatialNodes,&radiusSquared,&Detail::ThreadMinimumMidpointMemo());
                        state.Minimums[i]->BindWholeGeometryHullMemoBeforeStart(WholeGeometryHulls.Get());
                    }
                }
                const auto before=state.Minimums[i]->GetCountersForInspection();
                bound=state.Minimums[i]->Advance();
                if(!AddSpatialCounters(before,state.Minimums[i]->GetCountersForInspection())||bound.Status==EDistanceBoundStatus::Unknown)return {};
            }
            if(bound.Status==EDistanceBoundStatus::Unknown)return {};
            if(!(bound.LowerSquared>radiusSquared.Upper))result.AllClear=false;
            if(bound.UpperSquared<radiusSquared.Lower)
            {
                result.AnyInside=true;result.SurfaceIndex=i;result.Witness=bound;
                result.AllClear=false;result.Known=true;return result;
            }
        }
        result.Known=true;return result;
    }
    FUnionBound EvaluateWholeSpatial(FTimeNode& node,const FIntervalPoint& rawHull,Interval radiusSquared)
    {
        FScopedSignedTimeCostRow cost(ESignedTimeCost::Whole,Budget);
        if(!WholeGeometryHulls&&SignedTimeQuery)
        {
            static const auto* enabled=IConsoleManager::Get().FindTConsoleVariableDataInt(
                TEXT("p.IAmSpeed.AnalyticWorld.SignedSphereWholeGeometryHullMemo"));
            if(enabled&&enabled->GetValueOnAnyThread()==1)
                WholeGeometryHulls=MakeUnique<FRetainedTensorMinimum::FWholeGeometryHullMemo>(NumericControl);
        }
        if(!node.Spatial)
        {
            if(!Budget.Consume()||!Detail::ValidPoint(rawHull))return {};
            for(int32 i=0;i<Surfaces.Num();++i)if(!Budget.Consume())return {};
            auto state=MakeUnique<FUnionSpatialState>();state->OriginalPath=&Path;state->Surfaces=Surfaces.GetData();
            state->ProviderCount=Surfaces.Num();state->CertifiedCenterHull=rawHull;state->Minimums.SetNum(Surfaces.Num());
            if(node.SpatialSeed)
            {
                if(!node.RetainSpatial||!SpatialBinding(*node.SpatialSeed)||!Detail::ValidPoint(node.SeedCenterHull))return {};
                state->CertifiedCenterHull=node.SeedCenterHull;
                for(int32 d=0;d<3;++d)
                    if(state->CertifiedCenterHull.Axis[d].Lower<node.SpatialSeed->CertifiedCenterHull.Axis[d].Lower||
                        state->CertifiedCenterHull.Axis[d].Upper>node.SpatialSeed->CertifiedCenterHull.Axis[d].Upper)return {};
            }
            node.Spatial=MoveTemp(state);
        }
        if(!SpatialBinding(*node.Spatial)||!Detail::ValidPoint(rawHull))return {};
        // Only the internal split helper may install a narrowed center scope.
        for(int32 d=0;d<3;++d)
            if(node.Spatial->CertifiedCenterHull.Axis[d].Lower<rawHull.Axis[d].Lower||
                node.Spatial->CertifiedCenterHull.Axis[d].Upper>rawHull.Axis[d].Upper)return {};
        return EvaluateSpatial(*node.Spatial,radiusSquared,node.SpatialSeed.Get());
    }
    static bool SameEndpointBits(const FIntervalPoint& a,const FIntervalPoint& b)
    {
        for(int32 d=0;d<3;++d)
            if(!a.Axis[d].Known||!b.Axis[d].Known||
                AnalyticCertified::Bits(a.Axis[d].Lower)!=AnalyticCertified::Bits(b.Axis[d].Lower)||
                AnalyticCertified::Bits(a.Axis[d].Upper)!=AnalyticCertified::Bits(b.Axis[d].Upper))return false;
        return true;
    }
    // This hint never supplies a returned bound, endpoint, UV or restricted
    // path. Its strict inside point only proves that this original dyadic node
    // cannot be cleared. Both original width tests require subdivision here.
    bool CanSkipDyadicNodeBounds(const FTimeNode& node,bool anchored,double gap,double leafWidth) const
    {
        if(!DyadicHintProved||SeedHasStrictInside||anchored||HadBracket||node.ForceSubdivision||
            node.RetainSpatial||node.Spatial||node.SpatialSeed||node.Endpoint.IsSet()||node.Depth>=52||
            !(node.A<=DyadicHintLower&&node.B>=DyadicHintUpper))return false;
        const auto width=AnalyticCertified::Subtract(Interval::Exact(node.B),Interval::Exact(node.A));
        return AnalyticCertified::Valid(width)&&width.Upper>gap&&width.Upper>leafWidth;
    }
    bool SplitTimeNodeCurve(const FTimeNode& node,double middle,
        FIntervalBezierCenterPath& left,FIntervalBezierCenterPath& right) const
    {
        if(!SeedHasStrictInside)return CenterDetail::Split(node.Curve,left,right);
        // Seed boundaries need not be dyadic. A half split of the restricted
        // curve could differ from the rounded global middle. Evaluate that
        // GLOBAL time on the ORIGINAL fixed linear path instead.
        if(Path.Degree!=1||Path.ControlPoints.Num()!=2||node.Curve.Degree!=1||node.Curve.ControlPoints.Num()!=2||
            !(middle>node.A&&middle<node.B))return false;
        auto point=ProjectedDistanceDetail::LerpPoint(Path.ControlPoints[0],Path.ControlPoints[1],Interval::Exact(middle));
        const auto parentHull=CenterDetail::Hull(node.Curve);
        if(!Detail::ValidPoint(point)||!Detail::ValidPoint(parentHull))return false;
        for(int32 d=0;d<3;++d)
        {point.Axis[d].Lower=FMath::Max(point.Axis[d].Lower,parentHull.Axis[d].Lower);point.Axis[d].Upper=FMath::Min(point.Axis[d].Upper,parentHull.Axis[d].Upper);}
        if(!Detail::ValidPoint(point))return false;
        left={};right={};left.Degree=right.Degree=1;
        left.ControlPoints={node.Curve.ControlPoints[0],point};right.ControlPoints={point,node.Curve.ControlPoints[1]};
        return CenterDetail::Valid(left)&&CenterDetail::Valid(right);
    }
    bool EndpointBinding(const FTimeEndpointProof& proof,const FIntervalPoint& center,double time) const
    {
        return proof.Owner==this&&proof.Account==&Budget&&proof.Bound.Known&&
            proof.NumericControl==NumericControl&&SameEndpointBits(proof.Center,center)&&
            (!proof.SignedSource||(proof.SignedSource==SignedTimeQuery&&
                proof.SignedTimeBits==AnalyticCertified::Bits(time)&&proof.Bound.AnyInside&&
                AnalyticCertified::Valid(proof.Bound.SignedInsideGap)&&proof.Bound.SignedInsideGap.Upper<0));
    }
    bool RetainSignedEndpoint(FTimeNode& node,Interval radiusSquared,const FSignedTimeInterval& value)
    {
        if(!SignedTimeQuery||node.Curve.ControlPoints.IsEmpty()||!Surfaces.IsValidIndex(value.Provider)||
            !AnalyticCertified::Valid(value.SignedInsideGap)||!(value.SignedInsideGap.Upper<0)||
            !Detail::ValidPoint(value.ProjectedPoint)||
            !NonnegativeFinite(value.ProjectedUV.X)||value.ProjectedUV.X>1||
            !NonnegativeFinite(value.ProjectedUV.Y)||value.ProjectedUV.Y>1||
            RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=NumericControl)
            return false;
        FTimeEndpointProof proof;
        proof.Owner=this;proof.Account=&Budget;proof.Center=node.Curve.ControlPoints.Last();
        proof.RadiusSquared=radiusSquared;proof.NumericControl=NumericControl;
        proof.UsedAtPublication=Budget.Used;proof.RemainingAtPublication=Budget.Remaining;
        proof.RequiredSticky=VectorGetControlRegister()&0x3fu;
        proof.SignedSource=SignedTimeQuery;proof.SignedTimeBits=AnalyticCertified::Bits(node.B);
        proof.Bound.Known=true;proof.Bound.AnyInside=true;proof.Bound.SurfaceIndex=value.Provider;
        proof.Bound.Witness.SampleUV=value.ProjectedUV;proof.Bound.Witness.SamplePoint=value.ProjectedPoint;
        proof.Bound.SignedInsideGap=value.SignedInsideGap;
        node.Endpoint=MoveTemp(proof);
        return true;
    }
    // Only the right child's terminal coefficient is copied exactly by Split.
    // Sharing an endpoint proof never substitutes for a whole-interval proof.
    bool ShareEndpointRight(const FTimeNode& parent,FTimeNode& right)
    {
        if(!parent.Endpoint.IsSet())return true;
        const auto& proof=parent.Endpoint.GetValue();
        if(!EnableEndpointMemo&&!proof.SignedSource)return true;
        if(parent.Curve.ControlPoints.IsEmpty()||right.Curve.ControlPoints.IsEmpty())return false;
        if(!EndpointBinding(proof,parent.Curve.ControlPoints.Last(),parent.B)||
            (proof.SignedSource&&AnalyticCertified::Bits(parent.B)!=AnalyticCertified::Bits(right.B))||
            !SameEndpointBits(proof.Center,right.Curve.ControlPoints.Last()))return true;
        if(!Budget.Consume())return false; // one bounded inline proof copy, no refund
        right.Endpoint=proof;++EndpointCounters.RightInheritances;return true;
    }

    bool HasReusableSignedInsideEndpoint(const FTimeNode& node,Interval radiusSquared) const
    {
        if(!SignedTimeQuery||!node.Endpoint.IsSet()||node.Curve.ControlPoints.IsEmpty())return false;
        const auto& proof=node.Endpoint.GetValue();
        if(proof.SignedSource!=SignedTimeQuery||
            !EndpointBinding(proof,node.Curve.ControlPoints.Last(),node.B)||
            !AnalyticCertified::Valid(radiusSquared)||
            AnalyticCertified::Bits(proof.RadiusSquared.Lower)!=AnalyticCertified::Bits(radiusSquared.Lower)||
            AnalyticCertified::Bits(proof.RadiusSquared.Upper)!=AnalyticCertified::Bits(radiusSquared.Upper)||
            Budget.Used<proof.UsedAtPublication||Budget.Remaining>proof.RemainingAtPublication||
            Budget.Used-proof.UsedAtPublication!=
                uint64(int64(proof.RemainingAtPublication)-int64(Budget.Remaining))||
            (VectorGetControlRegister()&proof.RequiredSticky)!=proof.RequiredSticky)return false;
        const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        return numeric.Status==EAnalyticNumericStatus::Complete&&numeric.ControlFingerprint()==NumericControl;
    }
    FUnionBound EvaluateEndpointSpatial(FTimeNode& node,Interval radiusSquared)
    {
        FScopedSignedTimeCostRow cost(ESignedTimeCost::Endpoint,Budget);
        ShortcutFailure=EPlanarFamilyShortcut::NotApplicable;
        if(!EnableEndpointMemo&&!node.Endpoint.IsSet())
            return Evaluate(Surfaces,node.Curve.ControlPoints.Last(),Budget,radiusSquared,SpatialGap,SpatialNodes,true,PlaneShortcut,&ShortcutFailure);
        ++EndpointCounters.Requests;
        if(node.Curve.ControlPoints.IsEmpty())return {};
        const auto& center=node.Curve.ControlPoints.Last();
        const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(numeric.Status!=EAnalyticNumericStatus::Complete||numeric.ControlFingerprint()!=NumericControl)return {};
        if(node.Endpoint.IsSet())
        {
            const auto& proof=node.Endpoint.GetValue();
            if(EndpointBinding(proof,center,node.B)&&AnalyticCertified::Valid(radiusSquared)&&
                AnalyticCertified::Bits(proof.RadiusSquared.Lower)==AnalyticCertified::Bits(radiusSquared.Lower)&&
                AnalyticCertified::Bits(proof.RadiusSquared.Upper)==AnalyticCertified::Bits(radiusSquared.Upper))
            {
                // A saved proof never legitimizes a reset or a broken account.
                if(Budget.Used<proof.UsedAtPublication||Budget.Remaining>proof.RemainingAtPublication)return {};
                if((VectorGetControlRegister()&proof.RequiredSticky)==proof.RequiredSticky)
                {
                    if(!Budget.Consume())return {}; // validation/materialization, even on a hit
                    ++EndpointCounters.Hits;return proof.Bound;
                }
            }
            node.Endpoint.Reset(); // foreign/mutated endpoint or cleared sticky flags: ordinary evaluation
        }
        ++EndpointCounters.Evaluations;
        const auto result=Evaluate(Surfaces,center,Budget,radiusSquared,SpatialGap,SpatialNodes,true,PlaneShortcut,&ShortcutFailure);
        if(result.Known&&EnableEndpointMemo)
        {
            const auto completed=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
            if(completed.Status!=EAnalyticNumericStatus::Complete||completed.ControlFingerprint()!=NumericControl)return {};
            // The already paid endpoint evaluation produced this bounded value.
            // Publish only its complete result; Unknown/partial work is never retained.
            FTimeEndpointProof proof;proof.Owner=this;proof.Account=&Budget;proof.Center=center;proof.Bound=result;proof.RadiusSquared=radiusSquared;
            proof.NumericControl=NumericControl;proof.UsedAtPublication=Budget.Used;proof.RemainingAtPublication=Budget.Remaining;
            proof.RequiredSticky=VectorGetControlRegister()&0x3fu;
            node.Endpoint=MoveTemp(proof);++EndpointCounters.Publications;
        }
        return result;
    }

public:
    FRetainedSphereTimeSearch(TConstArrayView<FIntervalTensorSurface> surfaces,
        const FIntervalBezierCenterPath& path,double radius,FDistanceBudget& budget,
        double spatialGap,int32 spatialNodes,ContactDetail::FContactCellGeometryMemo* geometryMemo=nullptr,bool enableCurveSupport=false,
        const FPlanarFamilyTimeShortcutPlan* planeShortcut=nullptr,
        bool enableEndpointMemo=PlanarFamilyTimeShortcutControl::RuntimeEnabled(),
        bool enableMonotoneTimeSeed=MonotoneCurveTimeControl::RuntimeEnabled(),
        bool enableMonotoneDyadicHint=MonotoneCurveTimeControl::DyadicHintEnabled(),
        bool enableCoordinateLineHint=CoordinateLineTimeHintControl::RuntimeEnabled(),
        bool enableChordEnvelopeHint=ChordEnvelopeTimeHintControl::RuntimeEnabled())
        :Surfaces(surfaces),Path(path),Radius(radius),SpatialGap(spatialGap),SpatialNodes(spatialNodes),Budget(budget),PlaneShortcut(planeShortcut),
        NumericControl(RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()),
        EnableEndpointMemo(enableEndpointMemo),EnableMonotoneTimeSeed(enableMonotoneTimeSeed),EnableMonotoneDyadicHint(enableMonotoneDyadicHint),EnableCoordinateLineHint(enableCoordinateLineHint),EnableChordEnvelopeHint(enableChordEnvelopeHint)
    {if(enableCurveSupport)CurveSupport=MakeUnique<FContinuousCurvePathSupport>(surfaces,geometryMemo,NumericControl);}
    ~FRetainedSphereTimeSearch()
    {
        if(auto* observation=ActiveSignedTimeCosts(Budget);observation&&WholeGeometryHulls)
        {
            observation->WholeHullLookups+=WholeGeometryHulls->GetLookups();
            observation->WholeHullHits+=WholeGeometryHulls->GetHits();
            observation->WholeHullStores+=WholeGeometryHulls->GetStores();
        }
        if(EnableMonotoneDyadicHint||EnableCoordinateLineHint||EnableChordEnvelopeHint)MonotoneCurveTimeControl::RecordDyadicHint(TimeNodeVisits,DyadicHintSkips,DyadicHintProved);
        if(EnableCertifiedPrefixClear)RecordCertifiedPrefixClear(CertifiedPrefixSkips);
        if(EnableDirectInitialFrontier)RecordDirectInitialTimeFrontier(DirectInitialFrontierApplied,TimeNodeVisits);}
    // Explicit operation-local opt-in before initialization. Existing direct
    // callers and old parity fixtures keep the unchanged mode by default.
    bool SetCertifiedPrefixClearBeforeStart(bool enabled)
    {if(Initialized||Terminal||HadBracket)return false;EnableCertifiedPrefixClear=enabled;return true;}
    uint64 GetCertifiedPrefixSkipsForInspection() const{return CertifiedPrefixSkips;}
    bool SetDirectInitialTimeFrontierBeforeStart(bool enabled)
    {if(Initialized||Terminal||HadBracket)return false;EnableDirectInitialFrontier=enabled;return true;}
    bool HasDirectInitialTimeFrontierForInspection() const{return DirectInitialFrontierApplied;}

    bool SetOffsetSpatialSeedBeforeStart(bool enabled)
    {if(Initialized||Terminal||HadBracket)return false;EnableOffsetSpatialSeed=enabled;return true;}
    EDirectCurveSpatialSeed PrepareSpatialSeed(const FIntervalPoint& center,FDirectCurveSpatialSeed& seed) const
    {
        seed={};
        if(!EnableOffsetSpatialSeed||!OffsetSpatialProof||!HadBracket||Terminal||
            Last.Status!=ETensorSphereTimeStatus::FirstContactBracket||!Last.InitialSeparationProved)
            return EDirectCurveSpatialSeed::NotApplicable;
        return PrepareOffsetCurveSpatialSeed(OffsetSpatialProof,this,Surfaces,Path,Radius,Budget,center,seed);
    }
    FRetainedSphereTimeSearch(const FRetainedSphereTimeSearch&)=delete;
    FRetainedSphereTimeSearch& operator=(const FRetainedSphereTimeSearch&)=delete;
    const FTimeEndpointCounters& GetEndpointCountersForInspection() const{return EndpointCounters;}
    uint64 GetTimeNodeVisitsForInspection() const{return TimeNodeVisits;}
    uint64 GetDyadicHintSkipsForInspection() const{return DyadicHintSkips;}
    bool HasDyadicHintForInspection() const{return DyadicHintProved;}
    uint64 GetCurveSupportAttemptsForInspection() const{return CurveSupportAttempts;}
    uint64 GetCurveSupportClearedCellsForInspection() const{return CurveSupportClearedCells;}
    ETimeSearchFailure GetFailureForInspection() const{return Failure;}
    int32 GetAnchorRefinementsForInspection() const{return AnchorRefinements;}
    uint8 GetDyadicSiteForInspection() const{return DyadicSite;}
    int32 GetDyadicDepthForInspection() const{return DyadicDepth;}
    double GetDyadicAForInspection() const{return DyadicA;}
    double GetDyadicBForInspection() const{return DyadicB;}
    const FRetainedTensorMinimumCounters& GetSpatialCountersForInspection() const{return SpatialCounters;}
    bool SetSignedTimeQueryBeforeStart(const FSignedTimeQuery* query)
    {
        if(Initialized||Terminal||HadBracket)return false;
        SignedTimeQuery=query;
        return true;
    }
    FTensorSphereTimeBounds Advance(double requestedTimeGap)
    {
        FTensorSphereTimeBounds result;
        if(Terminal||RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=NumericControl||
            !NonnegativeFinite(requestedTimeGap)||!NonnegativeFinite(Radius)){Failure=ETimeSearchFailure::Input;return result;}
        const auto r2=AnalyticCertified::Square(Interval::Exact(Radius));
        if(!Initialized)
        {
            Initialized=true;
            if(SpatialNodes<=0||Surfaces.IsEmpty()||!NonnegativeFinite(Radius)||!NonnegativeFinite(SpatialGap)||
                !CenterDetail::Valid(Path)||!AnalyticCertified::Valid(r2)){Failure=ETimeSearchFailure::Input;Terminal=true;return result;}
            const auto initial=ObserveSignedTimeCost(ESignedTimeCost::Initial,Budget,[&]{
                return Evaluate(Surfaces,Path.ControlPoints[0],Budget,r2,SpatialGap,SpatialNodes,false,PlaneShortcut,&ShortcutFailure);
            });
            if(!initial.Known){Failure=FailureFor(ETimeSearchFailure::InitialUnknown);Terminal=true;return result;}
            if(initial.AnyInside)
            {
                result.Status=ETensorSphereTimeStatus::InitialInsideTube;result.UpperTime=0;
                result.InsideWitnessSurfaceIndex=initial.SurfaceIndex;result.InsideSampleUV=initial.Witness.SampleUV;
                result.InsideSurfacePoint=initial.Witness.SamplePoint;Terminal=true;return result;
            }
            if(!initial.AllClear){Failure=ETimeSearchFailure::InitialThreshold;Terminal=true;return result;}
            FTimeNode first{0.,1.,0,Path};
            if(EnableMonotoneTimeSeed)
            {
                FMonotoneCurveTimeSeed seed;
                const auto status=TryMonotoneCurveTimeSeed(Surfaces,Path,Radius,Budget,requestedTimeGap,768,seed);
                if(status==EMonotoneCurveTimeSeed::Work||status==EMonotoneCurveTimeSeed::Arithmetic)
                {Failure=status==EMonotoneCurveTimeSeed::Work?ETimeSearchFailure::Work:ETimeSearchFailure::Arithmetic;Terminal=true;return result;}
                if(status==EMonotoneCurveTimeSeed::Proved)
                {
                    if(!(seed.Lower>=0&&seed.Lower<seed.Upper&&seed.Upper<=1)||
                        !CenterDetail::Valid(seed.RestrictedPath)||seed.InsideWitness.Status==EDistanceBoundStatus::Unknown||
                        !(seed.InsideWitness.UpperSquared<r2.Lower))
                    {Failure=ETimeSearchFailure::Arithmetic;Terminal=true;return result;}
                    first.A=seed.Lower;first.B=seed.Upper;first.Curve=MoveTemp(seed.RestrictedPath);
                    InitialClearPrefix=seed.Lower;SeedHasStrictInside=true;
                    FTimeEndpointProof proof;proof.Owner=this;proof.Account=&Budget;proof.Center=first.Curve.ControlPoints.Last();
                    proof.RadiusSquared=r2;proof.NumericControl=NumericControl;proof.UsedAtPublication=Budget.Used;
                    proof.RemainingAtPublication=Budget.Remaining;proof.RequiredSticky=VectorGetControlRegister()&0x3fu;
                    proof.Bound.Known=true;proof.Bound.AnyInside=true;proof.Bound.AllClear=false;proof.Bound.SurfaceIndex=0;
                    proof.Bound.Witness=seed.InsideWitness;first.Endpoint=MoveTemp(proof);
                }
            }
            // A separate default-OFF prototype keeps the complete old dyadic
            // frontier. The proof is only a control-flow hint for nodes that
            // necessarily split; no result witness or endpoint is imported.
            if(EnableCoordinateLineHint&&!EnableMonotoneTimeSeed)
            {
                double lower=0,upper=1;
                const auto status=TryCoordinateLineTimeHint(Surfaces,Path,Radius,Budget,requestedTimeGap,lower,upper);
                if(status==EMonotoneCurveTimeSeed::Work||status==EMonotoneCurveTimeSeed::Arithmetic)
                {Failure=status==EMonotoneCurveTimeSeed::Work?ETimeSearchFailure::Work:ETimeSearchFailure::Arithmetic;Terminal=true;return result;}
                if(status==EMonotoneCurveTimeSeed::Proved)
                {
                    if(!(lower>=0&&lower<upper&&upper<=1)){Failure=ETimeSearchFailure::Arithmetic;Terminal=true;return result;}
                    // This exact line proof provides existence, not an invented
                    // original UV. Only the old frontier produces the witness.
                    DyadicHintLower=lower;DyadicHintUpper=upper;DyadicHintProved=true;
                }
            }
            if(EnableChordEnvelopeHint&&!EnableMonotoneTimeSeed&&!DyadicHintProved)
            {
                double lower=0,upper=1;
                const auto status=EnableOffsetSpatialSeed?
                    TryChordEnvelopeTimeHintWithSpatialProof(Surfaces,Path,Radius,Budget,requestedTimeGap,lower,upper,OffsetSpatialProof,this):
                    TryChordEnvelopeTimeHint(Surfaces,Path,Radius,Budget,requestedTimeGap,lower,upper);
                if(status==EMonotoneCurveTimeSeed::Work||status==EMonotoneCurveTimeSeed::Arithmetic)
                {Failure=status==EMonotoneCurveTimeSeed::Work?ETimeSearchFailure::Work:ETimeSearchFailure::Arithmetic;Terminal=true;return result;}
                if(status==EMonotoneCurveTimeSeed::Proved)
                {
                    if(!(lower>=0&&lower<upper&&upper<=1)){Failure=ETimeSearchFailure::Arithmetic;Terminal=true;return result;}
                    // A capsule only bounded the clear prefix. Its B was checked
                    // on the original curve, and the old frontier still owns UV.
                    DyadicHintLower=lower;DyadicHintUpper=upper;DyadicHintProved=true;
                }
            }
            if(EnableMonotoneDyadicHint&&!EnableMonotoneTimeSeed&&!DyadicHintProved)
            {
                FMonotoneCurveTimeSeed seed;
                const auto status=TryMonotoneCurveTimeSeed(Surfaces,Path,Radius,Budget,requestedTimeGap,768,seed);
                if(status==EMonotoneCurveTimeSeed::Work||status==EMonotoneCurveTimeSeed::Arithmetic)
                {Failure=status==EMonotoneCurveTimeSeed::Work?ETimeSearchFailure::Work:ETimeSearchFailure::Arithmetic;Terminal=true;return result;}
                if(status==EMonotoneCurveTimeSeed::Proved)
                {
                    if(!(seed.Lower>=0&&seed.Lower<seed.Upper&&seed.Upper<=1)||
                        !CenterDetail::Valid(seed.RestrictedPath)||seed.InsideWitness.Status==EDistanceBoundStatus::Unknown||
                        !(seed.InsideWitness.UpperSquared<r2.Lower))
                    {Failure=ETimeSearchFailure::Arithmetic;Terminal=true;return result;}
                    DyadicHintLower=seed.Lower;DyadicHintUpper=seed.Upper;DyadicHintProved=true;
                }
            }
            if(EnableDirectInitialFrontier&&!EnableMonotoneTimeSeed&&!SeedHasStrictInside&&DyadicHintProved)
            {
                FTimeNode direct;
                const auto status=BuildDirectInitialTimeFrontier(Path,DyadicHintLower,DyadicHintUpper,Budget,direct);
                if(status==EDirectTimeFrontier::Work||status==EDirectTimeFrontier::Arithmetic)
                {Failure=status==EDirectTimeFrontier::Work?ETimeSearchFailure::Work:ETimeSearchFailure::Arithmetic;Terminal=true;return result;}
                if(status==EDirectTimeFrontier::Ready)
                {
                    // Existing current-operation proof: ALL providers clear on
                    // [0,A], and some original point strictly inside at B.
                    // No surface witness/UV is imported: the unchanged endpoint
                    // evaluation below must produce a fresh original witness.
                    first=MoveTemp(direct);InitialClearPrefix=DyadicHintLower;
                    SeedHasStrictInside=true;DirectInitialFrontierApplied=true;
                }
            }
#if !UE_BUILD_SHIPPING
            // A signed hint can only prune the original dyadic frontier.
            // Set it after all old seed/frontier choices, so it cannot replace
            // the path or provide the retained endpoint or its payload.
            if(SignedTimeQuery&&!DyadicHintProved&&!SeedHasStrictInside&&!EnableMonotoneTimeSeed)
            {
                double lower=0.,upper=1.;
                if(SignedTimeQuery->TryDyadicHint(lower,upper,Budget)&&
                    0.<=lower&&lower<upper&&upper<=1.)
                {
                    DyadicHintLower=lower;DyadicHintUpper=upper;
                    DyadicHintProved=true;SignedDyadicHintProved=true;
                }
            }
#endif
            Stack.Add(MoveTemp(first));
        }
        else
        {
            if(!HadBracket||!(requestedTimeGap<LastGap)||Deferred.IsEmpty()){Failure=ETimeSearchFailure::RepeatedGap;Terminal=true;return result;}
            // Deferred is in chronological DFS order and contains every earlier
            // unresolved cell plus the cell carrying the upper inside witness.
            // Already clear gaps stay discharged for this immutable path.
            Stack.Reset();
            for(int32 i=Deferred.Num()-1;i>=0;--i)
            {
                if(Deferred[i].A<Last.LowerTime||Deferred[i].B>Last.UpperTime){Failure=ETimeSearchFailure::NonNested;Terminal=true;return result;}
                Stack.Add(MoveTemp(Deferred[i]));
            }
            Deferred.Reset();
        }
        result.InitialSeparationProved=true;
        const auto leafGap=AnalyticCertified::Multiply(Interval::Exact(requestedTimeGap),Interval::Exact(.25));
        if(!AnalyticCertified::Valid(leafGap)){Failure=ETimeSearchFailure::Arithmetic;Terminal=true;return result;}
        const double targetLeafWidth=FMath::Max(0.,leafGap.Lower);
        bool anchored=false;double anchor=0,clearPrefix=HadBracket?Last.LowerTime:InitialClearPrefix;
        while(!Stack.IsEmpty())
        {
            if(!Budget.Consume()){result.LowerTime=anchored?anchor:clearPrefix;Failure=ETimeSearchFailure::Work;Terminal=true;return result;}
            auto node=Stack.Pop(EAllowShrinking::No);
            if(TimeNodeVisits<MAX_uint64)++TimeNodeVisits;
            // The hint certified separation on the ORIGINAL fixed path for
            // every provider up to DyadicHintLower. The outward CP box of a
            // restricted node need not itself be separated: its true path is
            // already covered by that stronger proof. No point/UV is imported.
            // Preserve any existing unresolved anchor and forced continuation.
#if !UE_BUILD_SHIPPING
            const bool beforeCertifiedPrefix=node.B<DyadicHintLower||
                (SignedDyadicHintProved&&node.B==DyadicHintLower);
            if((EnableCertifiedPrefixClear||SignedDyadicHintProved)&&DyadicHintProved&&!SeedHasStrictInside&&
                !anchored&&!node.ForceSubdivision&&node.A>=0&&beforeCertifiedPrefix)
#else
            if(EnableCertifiedPrefixClear&&DyadicHintProved&&!SeedHasStrictInside&&
                !anchored&&!node.ForceSubdivision&&node.A>=0&&node.B<DyadicHintLower)
#endif
            {
                if(!Budget.Consume()){result.LowerTime=clearPrefix;Failure=ETimeSearchFailure::Work;Terminal=true;return result;}
                if(node.Endpoint.IsSet())
                {
                    const auto& proof=node.Endpoint.GetValue();
                    const bool reusable=EndpointBinding(proof,node.Curve.ControlPoints.Last(),node.B)&&AnalyticCertified::Valid(r2)&&
                        AnalyticCertified::Bits(proof.RadiusSquared.Lower)==AnalyticCertified::Bits(r2.Lower)&&
                        AnalyticCertified::Bits(proof.RadiusSquared.Upper)==AnalyticCertified::Bits(r2.Upper)&&
                        Budget.Used>=proof.UsedAtPublication&&Budget.Remaining<=proof.RemainingAtPublication&&
                        (VectorGetControlRegister()&proof.RequiredSticky)==proof.RequiredSticky;
                    if(reusable&&proof.Bound.AnyInside)
                    {result.LowerTime=clearPrefix;Failure=ETimeSearchFailure::Arithmetic;Terminal=true;return result;}
                }
                clearPrefix=node.B;
                if(CertifiedPrefixSkips<MAX_uint64)++CertifiedPrefixSkips;
#if !UE_BUILD_SHIPPING
                if(SignedDyadicHintProved)SignedTimeQuery->NoteDyadicHintSkip(true);
#endif
                continue;
            }
            if(anchored)
            {
                const auto fromAnchor=AnalyticCertified::Subtract(Interval::Exact(node.A),Interval::Exact(anchor));
                if(!AnalyticCertified::Valid(fromAnchor))
                {result.LowerTime=anchor;Failure=ETimeSearchFailure::Arithmetic;Terminal=true;return result;}
                if(fromAnchor.Lower>requestedTimeGap)
                {
                    // Revisit the EARLIEST unresolved cell instead of moving
                    // its anchor past an unproved gap. Keep this node and ALL
                    // future cells, including the old strict inside witness.
                    if(Deferred.IsEmpty()||Deferred[0].Depth>=52)
                    {NoteDyadic(1,Deferred.IsEmpty()?node:Deferred[0]);result.LowerTime=anchor;Failure=ETimeSearchFailure::Dyadic;Terminal=true;return result;}
                    Deferred[0].ForceSubdivision=true;
                    Stack.Add(MoveTemp(node));
                    for(int32 i=Deferred.Num()-1;i>=0;--i)
                    {
                        if(!Budget.Consume()){result.LowerTime=anchor;Failure=ETimeSearchFailure::Work;Terminal=true;return result;}
                        Stack.Add(MoveTemp(Deferred[i]));
                    }
                    Deferred.Reset();anchored=false;++AnchorRefinements;
                    continue;
                }
            }
            if(node.ForceSubdivision)
            {
                if(node.Depth>=52||!Budget.Consume())
                {result.LowerTime=node.A;Failure=ETimeSearchFailure::Work;Terminal=true;return result;}
                const double middle=.5*(node.A+node.B);
                FIntervalBezierCenterPath left,right;
                if(!(middle>node.A&&middle<node.B)||!SplitTimeNodeCurve(node,middle,left,right))
                {NoteDyadic(2,node);result.LowerTime=node.A;Failure=ETimeSearchFailure::Dyadic;Terminal=true;return result;}
                FTimeNode leftNode{node.A,middle,node.Depth+1,MoveTemp(left)};
                FTimeNode rightNode{middle,node.B,node.Depth+1,MoveTemp(right)};
                if(!ShareEndpointRight(node,rightNode)||!ShareSpatialChildren(node,leftNode,rightNode,true))
                {result.LowerTime=node.A;Failure=ETimeSearchFailure::RetainedSpatial;Terminal=true;return result;}
                Stack.Add(MoveTemp(rightNode));Stack.Add(MoveTemp(leftNode));
                // Only universal whole-center bounds/UV states are inherited; endpoint states are never widened.
                continue;
            }
            if(CanSkipDyadicNodeBounds(node,anchored,requestedTimeGap,targetLeafWidth))
            {
                // Identical normal split and child handling to the original
                // path below. Earlier siblings still execute every old bound.
                if(!Budget.Consume()){result.LowerTime=node.A;Failure=ETimeSearchFailure::Work;Terminal=true;return result;}
                const double middle=.5*(node.A+node.B);
                if(!(middle>node.A&&middle<node.B)){NoteDyadic(3,node);result.LowerTime=node.A;Failure=ETimeSearchFailure::Dyadic;Terminal=true;return result;}
                FIntervalBezierCenterPath left,right;
                if(!SplitTimeNodeCurve(node,middle,left,right)){NoteDyadic(4,node);Failure=ETimeSearchFailure::Dyadic;Terminal=true;return result;}
                FTimeNode leftNode{node.A,middle,node.Depth+1,MoveTemp(left)};
                FTimeNode rightNode{middle,node.B,node.Depth+1,MoveTemp(right)};
                if(!ShareEndpointRight(node,rightNode)||!ShareSpatialChildren(node,leftNode,rightNode,false))
                {result.LowerTime=node.A;Failure=ETimeSearchFailure::RetainedSpatial;Terminal=true;return result;}
                Stack.Add(MoveTemp(rightNode));Stack.Add(MoveTemp(leftNode));
                if(DyadicHintSkips<MAX_uint64)++DyadicHintSkips;
#if !UE_BUILD_SHIPPING
                if(SignedDyadicHintProved)SignedTimeQuery->NoteDyadicHintSkip(false);
#endif
                continue;
            }
            // Only the authenticated product plan can clear a complete time
            // cell here. Keep any already bound inside witness, and let the
            // same frontier own every later subdivision and publication.
            if(SignedTimeQuery&&SignedTimeQuery->HasClearCallback())
            {
                const bool retainedInside=node.Endpoint.IsSet()&&node.Endpoint.GetValue().Bound.AnyInside&&
                    EndpointBinding(node.Endpoint.GetValue(),node.Curve.ControlPoints.Last(),node.B);
                if(!retainedInside&&ObserveSignedTimeCost(ESignedTimeCost::Preclear,Budget,[&]{
                    return SignedTimeQuery->TryClearBeforeCoarse(node.A,node.B,Budget);
                }))
                {
                    if(!anchored)clearPrefix=node.B;
                    continue;
                }
            }
            // Root Bounds can cheaply close a far interval. Any actual
            // arithmetic/budget Unknown remains terminal; it is never retried.
            const auto hull=CenterDetail::Hull(node.Curve);
            const auto coarse=ObserveSignedTimeCost(ESignedTimeCost::Coarse,Budget,[&]{
                return Evaluate(Surfaces,hull,Budget,r2,SpatialGap,1,false,PlaneShortcut,&ShortcutFailure);
            });
            if(!coarse.Known){NoteDyadic(7,node);result.LowerTime=anchored?anchor:node.A;Failure=FailureFor(ETimeSearchFailure::WholeUnknown);Terminal=true;return result;}
            if(coarse.AllClear)
            {
                if((SeedHasStrictInside||SignedTimeQuery)&&node.Endpoint.IsSet()&&
                    EndpointBinding(node.Endpoint.GetValue(),node.Curve.ControlPoints.Last(),node.B)&&
                    node.Endpoint.GetValue().Bound.AnyInside)
                {Failure=ETimeSearchFailure::Arithmetic;Terminal=true;return result;}
                if(!anchored)clearPrefix=node.B;continue;
            }
            // The cheap Cartesian hull remains first. Only unresolved cells
            // try the correlation-preserving bound on the SAME full path.
            // Every provider must be excluded; a local stop proves nothing.
            auto inside=coarse;
            bool signedHandled=false;
            if(SignedTimeQuery&&!inside.AnyInside)
            {
                static const auto* reuse=IConsoleManager::Get().FindTConsoleVariableDataInt(
                    TEXT("p.IAmSpeed.AnalyticWorld.SignedSphereRetainedEndpointReuse"));
                if(reuse&&reuse->GetValueOnAnyThread()==1&&HasReusableSignedInsideEndpoint(node,r2))
                {
                    // This is the same B and the same signed witness retained from
                    // its parent. It proves existence at B, never whole-cell clear.
                    ++EndpointCounters.Requests;
                    if(!Budget.Consume())
                    {
                        result.LowerTime=anchored?anchor:node.A;
                        Failure=ETimeSearchFailure::Work;Terminal=true;return result;
                    }
                    ++EndpointCounters.Hits;
                    if(auto* observation=ActiveSignedTimeCosts(Budget))++observation->InsideReused;
                    inside=node.Endpoint.GetValue().Bound;
                    SignedSawInside=true;
                    signedHandled=true;
                }
            }
            if(SignedTimeQuery&&!inside.AnyInside&&!signedHandled)
            {
                const auto signedProof=ObserveSignedTimeCost(ESignedTimeCost::Signed,Budget,[&]{
                    return (*SignedTimeQuery)(node.A,node.B,Budget);
                });
                signedHandled=signedProof.Decision!=ESignedTimeDecision::NotApplicable;
                if(signedProof.Decision==ESignedTimeDecision::Work||signedProof.Decision==ESignedTimeDecision::Arithmetic)
                {
                    result.LowerTime=anchored?anchor:node.A;
                    Failure=signedProof.Decision==ESignedTimeDecision::Work?ETimeSearchFailure::Work:ETimeSearchFailure::Arithmetic;
                    Terminal=true;return result;
                }
                if(signedProof.Decision==ESignedTimeDecision::Clear)
                {
                    if(node.Endpoint.IsSet()&&
                        EndpointBinding(node.Endpoint.GetValue(),node.Curve.ControlPoints.Last(),node.B)&&
                        node.Endpoint.GetValue().Bound.AnyInside)
                    {Failure=ETimeSearchFailure::Arithmetic;Terminal=true;return result;}
                    if(!anchored)clearPrefix=node.B;
                    continue;
                }
                if(signedProof.Decision==ESignedTimeDecision::Inside)
                {
                    if(auto* observation=ActiveSignedTimeCosts(Budget))++observation->InsidePublished;
                    if(!RetainSignedEndpoint(node,r2,signedProof))
                    {Failure=ETimeSearchFailure::Arithmetic;Terminal=true;return result;}
                    inside=node.Endpoint.GetValue().Bound;
                    SignedSawInside=true;
                }
                else if(node.Endpoint.IsSet()&&node.Endpoint.GetValue().SignedSource&&
                    EndpointBinding(node.Endpoint.GetValue(),node.Curve.ControlPoints.Last(),node.B))
                {
                    inside=EvaluateEndpointSpatial(node,r2);
                    if(!inside.Known)
                    {NoteDyadic(6,node);Failure=FailureFor(ETimeSearchFailure::EndpointUnknown);Terminal=true;return result;}
                }
                // An admitted but indeterminate signed cell is subdivided on
                // the same frontier. It proves neither Clear nor Inside.
            }
            if(!signedHandled&&!inside.AnyInside&&node.Curve.Degree==2&&JointProjectedSpherePrefix::RuntimeEnabled())
            {
                using E=JointProjectedSpherePrefix::EResult;
                bool allClear=true;
                for(const auto& surface:Surfaces)
                {
                    const auto proof=JointProjectedSpherePrefix::Exclude(surface,node.Curve,r2,Budget);
                    if(proof==E::Unknown)
                    {
                        NoteDyadic(10,node);
                        result.LowerTime=anchored?anchor:node.A;
                        Failure=ETimeSearchFailure::WholeUnknown;
                        Terminal=true;
                        return result;
                    }
                    if(proof!=E::Clear){allClear=false;break;}
                }
                if(allClear)
                {
                    if(SeedHasStrictInside&&node.Endpoint.IsSet()&&
                        EndpointBinding(node.Endpoint.GetValue(),node.Curve.ControlPoints.Last(),node.B)&&
                        node.Endpoint.GetValue().Bound.AnyInside)
                    {
                        Failure=ETimeSearchFailure::Arithmetic;
                        Terminal=true;
                        return result;
                    }
                    if(!anchored)clearPrefix=node.B;
                    continue;
                }
            }
            if(!signedHandled&&CurveSupport&&!inside.AnyInside)
            {
                bool allClear=true;
                for(int32 provider=0;provider<Surfaces.Num();++provider)
                {
                    if(CurveSupportAttempts<MAX_uint64)++CurveSupportAttempts;
                    const auto proof=CurveSupport->TryExclude(provider,node.Curve,r2,Budget,8);
                    if(proof==ECurvePathExclusion::Unknown)
                    {NoteDyadic(9,node);result.LowerTime=anchored?anchor:node.A;Failure=FailureFor(ETimeSearchFailure::WholeUnknown);Terminal=true;return result;}
                    if(proof!=ECurvePathExclusion::Clear){allClear=false;break;}
                }
                if(allClear)
                {if(CurveSupportClearedCells<MAX_uint64)++CurveSupportClearedCells;if(!anchored)clearPrefix=node.B;continue;}
            }
            if(!signedHandled&&!inside.AnyInside)
            {
                // Initialization above validated ALL providers. A strict
                // endpoint witness gives existence at B, not a first owner.
                inside=EvaluateEndpointSpatial(node,r2);
                if(!inside.Known){NoteDyadic(5,node);result.LowerTime=anchored?anchor:node.A;Failure=FailureFor(ETimeSearchFailure::EndpointUnknown);Terminal=true;return result;}
            }
            if(!signedHandled&&!inside.AnyInside)
            {
                // Clear endpoints do not exclude an interior enter-and-exit.
                // Only this full-hull proof can discharge the whole cell.
                const auto whole=EvaluateWholeSpatial(node,hull,r2);
                if(!whole.Known){NoteDyadic(8,node);result.LowerTime=anchored?anchor:node.A;Failure=FailureFor(ETimeSearchFailure::WholeUnknown);Terminal=true;return result;}
                if(whole.AllClear){if(!anchored)clearPrefix=node.B;continue;}
                inside=whole;
            }
            // If endpoint B is strictly inside, whole.AllClear is impossible:
            // B belongs to the whole center hull. Do not spend the full spatial
            // attempt trying to prove a contradicted universal separation.
            const double candidateLower=anchored?anchor:node.A;
            const auto width=AnalyticCertified::Subtract(Interval::Exact(node.B),Interval::Exact(candidateLower));
            if(!AnalyticCertified::Valid(width)){Failure=ETimeSearchFailure::Arithmetic;Terminal=true;return result;}
            if(inside.AnyInside&&width.Upper<=requestedTimeGap)
            {
                result.Status=ETensorSphereTimeStatus::FirstContactBracket;
                result.LowerTime=candidateLower;result.UpperTime=node.B;
                result.InsideWitnessSurfaceIndex=inside.SurfaceIndex;result.InsideSampleUV=inside.Witness.SampleUV;
                result.InsideSurfacePoint=inside.Witness.SamplePoint;
                if(HadBracket&&(result.LowerTime<Last.LowerTime||result.UpperTime>Last.UpperTime)){Failure=ETimeSearchFailure::NonNested;Terminal=true;return {};}
                Deferred.Add(MoveTemp(node));Stack.Reset();Last=result;LastGap=requestedTimeGap;HadBracket=true;
                return result;
            }
            const auto localWidth=AnalyticCertified::Subtract(Interval::Exact(node.B),Interval::Exact(node.A));
            if(!AnalyticCertified::Valid(localWidth)){Failure=ETimeSearchFailure::Arithmetic;Terminal=true;return result;}
            if(node.Depth>=52||localWidth.Upper<=targetLeafWidth)
            {
                if(!anchored){anchored=true;anchor=node.A;}
                Deferred.Add(MoveTemp(node));continue;
            }
            if(!Budget.Consume()){result.LowerTime=anchored?anchor:node.A;Failure=ETimeSearchFailure::Work;Terminal=true;return result;}
            const double middle=.5*(node.A+node.B);
            if(!(middle>node.A&&middle<node.B)){NoteDyadic(3,node);result.LowerTime=anchored?anchor:node.A;Failure=ETimeSearchFailure::Dyadic;Terminal=true;return result;}
            FIntervalBezierCenterPath left,right;
            if(!SplitTimeNodeCurve(node,middle,left,right)){NoteDyadic(4,node);Failure=ETimeSearchFailure::Dyadic;Terminal=true;return result;}
            FTimeNode leftNode{node.A,middle,node.Depth+1,MoveTemp(left)};
            FTimeNode rightNode{middle,node.B,node.Depth+1,MoveTemp(right)};
            if(!ShareEndpointRight(node,rightNode)||!ShareSpatialChildren(node,leftNode,rightNode,false))
            {result.LowerTime=node.A;Failure=ETimeSearchFailure::RetainedSpatial;Terminal=true;return result;}
            Stack.Add(MoveTemp(rightNode));Stack.Add(MoveTemp(leftNode));
        }
        result.LowerTime=anchored?anchor:1.;
        // A retained inside witness can never become a new no-contact proof.
        if(!anchored&&!HadBracket&&!SeedHasStrictInside&&!DyadicHintProved&&!SignedSawInside){result.Status=ETensorSphereTimeStatus::NoIntersectionWithProvidedTensors;result.UpperTime=1.;}
        if(anchored||HadBracket||SeedHasStrictInside||DyadicHintProved||SignedSawInside)Failure=ETimeSearchFailure::NoLaterWitness;
        Terminal=true;return result;
    }
};
}
// Fixed CONTINUOUS reference curve, not an independently rounded runtime pose
// or time cast. The one-shot API uses the same retained search operation.
inline FTensorSphereTimeBounds BoundTensorSphereSweep(TConstArrayView<FIntervalTensorSurface> surfaces,
    const FIntervalBezierCenterPath& path,double radius,FDistanceBudget& budget,
    double requestedTimeGap,double requestedSpatialSquaredGap,int32 spatialNodesPerAttempt)
{
    TimeDetail::FRetainedSphereTimeSearch search(surfaces,path,radius,budget,requestedSpatialSquaredGap,spatialNodesPerAttempt);
    return search.Advance(requestedTimeGap);
}

inline FTensorSphereTimeBounds BoundTensorSphereSweep(TConstArrayView<FIntervalTensorSurface> surfaces,
    const FVector3d& start,const FVector3d& end,double radius,FDistanceBudget& budget,
    double requestedTimeGap,double requestedSpatialSquaredGap,int32 spatialNodesPerAttempt)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    FIntervalBezierCenterPath path;if(!CenterDetail::Linear(start,end,path))return {};
    return BoundTensorSphereSweep(surfaces,path,radius,budget,requestedTimeGap,requestedSpatialSquaredGap,spatialNodesPerAttempt);
}
inline FTensorSphereTimeBounds BoundTensorSphereSweep(TConstArrayView<FTensorBezierSurface> surfaces,
    const FVector3d& start,const FVector3d& end,double radius,FDistanceBudget& budget,
    double requestedTimeGap,double requestedSpatialSquaredGap,int32 spatialNodesPerAttempt)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    // Initial union validation needs at least provider+root work per net.
    // Reject before global copying when the SAME caller budget cannot pay it.
    if(budget.Remaining<=0||surfaces.Num()>budget.Remaining/2)return {};
    TArray<FIntervalTensorSurface> nets;if(!Detail::PromoteTensors(surfaces,nets))return {};
    return BoundTensorSphereSweep(MakeArrayView(nets),start,end,radius,budget,requestedTimeGap,requestedSpatialSquaredGap,spatialNodesPerAttempt);
}
}
