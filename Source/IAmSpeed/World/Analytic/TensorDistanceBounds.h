#pragma once
#include "AnalyticWorldData.h"
#include "AnalyticNumericContext.h"
#include "AnalyticIntervalBounds.h"
#include "HAL/PlatformTLS.h"
#include "HAL/PlatformTime.h"
#include <exception>
#include <atomic>
#include "HAL/IConsoleManager.h"

namespace Speed::Analytic::Certified
{
using AnalyticCertified::Interval;
enum class EDistanceBoundStatus : uint8 { Unknown, Bounded, Converged };
struct FDistanceBudget
{
    int32 Remaining = 0;
    uint64 Used = 0;
    // A technical reservation can only deny consumes. Remaining/Used are
    // never credited or reset when a scope releases its reservation.
    int32 ConsumptionFloor = 0;
    bool Consume() { if (ConsumptionFloor < 0 || Remaining <= ConsumptionFloor || Used == MAX_uint64) return false; --Remaining; ++Used; return true; }
};
class FScopedDistanceBudgetFloor final
{
    FDistanceBudget* Budget=nullptr;int32 Previous=0;
public:
    FScopedDistanceBudgetFloor(FDistanceBudget& budget,int32 floor)
    {
        if(floor<0||budget.ConsumptionFloor<0||floor<budget.ConsumptionFloor||floor>budget.Remaining)return;
        Budget=&budget;Previous=budget.ConsumptionFloor;budget.ConsumptionFloor=floor;
    }
    ~FScopedDistanceBudgetFloor(){if(Budget)Budget->ConsumptionFloor=Previous;}
    FScopedDistanceBudgetFloor(const FScopedDistanceBudgetFloor&)=delete;
    FScopedDistanceBudgetFloor& operator=(const FScopedDistanceBudgetFloor&)=delete;
    bool IsValid() const {return Budget!=nullptr;}
};
struct FIntervalPoint { Interval Axis[3]; };
// One fixed real polynomial has its exact coefficients inside this net. The
// boxes may lose correlations, but coefficients never change during a query.
// Proofs below hold for EVERY fixed polynomial represented by these boxes.
struct FIntervalTensorSurface
{
    int32 DegreeU=0,DegreeV=0;
    TArray<FIntervalPoint,TInlineAllocator<16>> ControlPoints;
};
struct FTensorDistanceBounds
{
    EDistanceBoundStatus Status = EDistanceBoundStatus::Unknown;
    double LowerSquared = 0, UpperSquared = 0;
    FVector2d SampleUV = FVector2d::ZeroVector;
    // Contains the exact point of each represented fixed polynomial at SampleUV.
    // This is neither a rounded evaluator point nor a collision/TOI/MTD witness.
    FIntervalPoint SamplePoint;
};

namespace ProjectedDistanceDetail
{inline bool EvaluateFixedUV(const FIntervalTensorSurface&,FVector2d,FIntervalPoint&);}
// An optional witness proposal, never a plane/surface/normal/authority theorem.
// Registration and the only publisher live in the existing IAS .cpp.
namespace PlaneProposal95
{
enum ECounter:uint32 {Calls,Admissions,SkippedInverse,SkippedLocal,Evaluated,Improved,
    ThresholdDecided,GapClosed,WorkFail,NumericFail,WorkUnits,Ticks,Count};
struct FObservation
{
    std::atomic<uint64> Values[Count]{},Active{0},Finished{0};
    std::atomic<bool> Integrity{true},ObservedEnabled{false};
};
inline FObservation& Observation(){static FObservation value;return value;}
inline void Add(ECounter counter,uint64 amount=1)
{auto& o=Observation();if(o.Values[counter].fetch_add(amount)>MAX_uint64-amount)o.Integrity.store(false);}
#if WITH_DEV_AUTOMATION_TESTS
inline thread_local int32 TestEnabled=-1;
struct FScopedTestEnable
{
    int32 Previous;explicit FScopedTestEnable(bool enabled):Previous(TestEnabled){TestEnabled=enabled?1:0;}
    ~FScopedTestEnable(){TestEnabled=Previous;}
};
#endif
inline bool Enabled()
{
#if WITH_DEV_AUTOMATION_TESTS
    if(TestEnabled>=0)return TestEnabled==1;
#endif
    // The central CPP registers once; no registration/export from this header.
    static const IConsoleVariable* flag=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.AnalyticWorld.DirectPlaneUVProposal95"));
    return flag&&flag->GetInt()==1;
}
struct FObservedScope
{
    uint64 Began=FPlatformTime::Cycles64();
    FObservedScope(){auto& o=Observation();o.ObservedEnabled.store(true);if(o.Active.fetch_add(1)==MAX_uint64)o.Integrity.store(false);Add(Calls);}
    ~FObservedScope(){auto& o=Observation();Add(Ticks,FPlatformTime::Cycles64()-Began);if(!o.Active.fetch_sub(1)||o.Finished.fetch_add(1)==MAX_uint64)o.Integrity.store(false);}
};
struct FWork
{
    const FIntervalTensorSurface* Original=nullptr;
    FDistanceBudget* Budget=nullptr;
    uint64 InitialUsed=0;
    int32 Limit=0;
    double RequestedGap=0;
    bool WorkFailure=false;
    uint64 AvailableLocal()const
    {return Budget&&Budget->Used>=InitialUsed&&Limit>=0&&Budget->Used-InitialUsed<=uint64(Limit)?uint64(Limit)-(Budget->Used-InitialUsed):0;}
    bool Pay(uint64 units)
    {
        if(WorkFailure||!Budget||units>AvailableLocal()){WorkFailure=true;Add(WorkFail);return false;}
        for(uint64 i=0;i<units;++i)
        {
            if(!Budget->Consume()){WorkFailure=true;Add(WorkFail);return false;}
            Add(WorkUnits);
        }
        return true;
    }
};
}

namespace Detail
{
inline bool ValidPoint(const FIntervalPoint& p)
{
    for (const auto& a : p.Axis) if (!AnalyticCertified::Valid(a)) return false;
    return true;
}
inline FIntervalPoint Point(const FVector3d& p)
{
    return {{Interval::Exact(p.X), Interval::Exact(p.Y), Interval::Exact(p.Z)}};
}
inline bool PromoteTensor(const FTensorBezierSurface& source,FIntervalTensorSurface& output)
{
    output={};
    if(source.DegreeU<0||source.DegreeV<0||source.DegreeU>16||source.DegreeV>16||
        source.ControlPoints.Num()!=(source.DegreeU+1)*(source.DegreeV+1))return false;
    FIntervalTensorSurface converted;converted.DegreeU=source.DegreeU;converted.DegreeV=source.DegreeV;
    for(const auto& p:source.ControlPoints)
    {
        const auto bounds=Point(p);if(!ValidPoint(bounds))return false;
        converted.ControlPoints.Add(bounds);
    }
    output=MoveTemp(converted);return true;
}
inline bool PromoteTensors(TConstArrayView<FTensorBezierSurface> source,TArray<FIntervalTensorSurface>& output)
{
    output.Reset();
    for(const auto& surface:source)
    {
        FIntervalTensorSurface converted;if(!PromoteTensor(surface,converted)){output.Reset();return false;}
        output.Add(MoveTemp(converted));
    }
    return true;
}
inline FIntervalPoint Half(const FIntervalPoint& a, const FIntervalPoint& b)
{
    FIntervalPoint p;
    for (int32 d=0; d<3; ++d) p.Axis[d]=AnalyticCertified::HalfSum(a.Axis[d],b.Axis[d]);
    return p;
}
struct FNode
{
    int32 DegreeU=0, DegreeV=0, DepthU=0, DepthV=0;
    double MinU=0, MaxU=1, MinV=0, MaxV=1;
    TArray<FIntervalPoint, TInlineAllocator<16>> Net;
    double LowerSquared=0, UpperSquared=0;
    FIntervalPoint WitnessPoint;
    FVector2d WitnessUV=FVector2d::ZeroVector;
};
inline FIntervalPoint Midpoint(const FNode& n)
{
    TArray<FIntervalPoint,TInlineAllocator<16>> u,v;
    const int32 vc=n.DegreeV+1;
    u.SetNum(n.DegreeU+1); v.SetNum(vc);
    for (int32 i=0; i<=n.DegreeU; ++i)
    {
        for (int32 j=0; j<vc; ++j) v[j]=n.Net[i*vc+j];
        for (int32 count=vc-1; count>0; --count)
            for (int32 j=0; j<count; ++j) v[j]=Half(v[j],v[j+1]);
        u[i]=v[0];
    }
    for (int32 count=n.DegreeU; count>0; --count)
        for (int32 i=0; i<count; ++i) u[i]=Half(u[i],u[i+1]);
    return u[0];
}
// INSERT inside Speed::Analytic::Certified::Detail, immediately AFTER the
// existing Detail::Midpoint definition. Add HAL/PlatformTLS.h at the TOP of the
// containing header (outside every namespace). Do not include this fragment
// from ContactCellGeometryMemo.h: no reverse include dependency is required.
//
// This is a geometry-value memo only. The caller retains every original work
// debit, validation and branch. Root and memo outlive their call scope; scopes
// are thread-affine. No FNode query field or source/domain authority is stored.
// Private observation only, enabled by the existing Contact timing scope.
// Parent CallMidpoint INCLUDES RootHash/CellHash/SameRoot/SameCell; do not sum.
// Ticks include timing/instrumentation overhead. No proof, cache or budget input.
namespace MinimumMidpointProfile84
{
enum EStage:uint32 {RootHash,CellHash,SameRoot,SameCell,CallMidpoint,StageCount};
struct FCounters
{
    uint64 Calls[StageCount]{},Finished[StageCount]{},InFlight[StageCount]{},Ticks[StageCount]{},Unwinding[StageCount]{};
    bool Integrity=true;
};
inline FCounters& Counters(){static thread_local FCounters value;return value;}
inline bool& Enabled(){static thread_local bool value=false;return value;}
inline void Add(uint64& target,uint64 delta)
{if(target>MAX_uint64-delta){Counters().Integrity=false;return;}target+=delta;}
class FEnabledScope final
{
    bool Previous;
public:
    explicit FEnabledScope(bool value):Previous(Enabled()){Enabled()=value;}
    ~FEnabledScope(){Enabled()=Previous;}
    FEnabledScope(const FEnabledScope&)=delete;FEnabledScope& operator=(const FEnabledScope&)=delete;
    FEnabledScope(FEnabledScope&&)=delete;FEnabledScope& operator=(FEnabledScope&&)=delete;
};
class FScope final
{
    EStage Stage;bool Active;uint64 Start=0;int Uncaught=0;
public:
    explicit FScope(EStage stage):Stage(stage),Active(Enabled())
    {
        if(!Active)return;
        if(Stage>=StageCount){Counters().Integrity=false;Active=false;return;}
        auto& c=Counters();Add(c.Calls[Stage],1);Add(c.InFlight[Stage],1);
        Uncaught=std::uncaught_exceptions();Start=FPlatformTime::Cycles64();
    }
    void Stop() noexcept
    {
        if(!Active)return;Active=false;const uint64 end=FPlatformTime::Cycles64();auto& c=Counters();
        if(end<Start)c.Integrity=false;else Add(c.Ticks[Stage],end-Start);
        if(std::uncaught_exceptions()>Uncaught)Add(c.Unwinding[Stage],1);
        Add(c.Finished[Stage],1);if(!c.InFlight[Stage])c.Integrity=false;else --c.InFlight[Stage];
    }
    ~FScope(){Stop();}
    FScope(const FScope&)=delete;FScope& operator=(const FScope&)=delete;
    FScope(FScope&&)=delete;FScope& operator=(FScope&&)=delete;
};
}

class FMinimumMidpointMemo final
{
public:
    struct FCounters
    {
        uint64 Requests=0,Hits=0,Builds=0,Evictions=0;
        uint64 RetainedMiddleRequests=0,RetainedMiddleHits=0,RetainedMiddlePublications=0;
        uint64 OrdinaryBoxRecomputedCP=0,OrdinaryBoxAvoidedCP=0;
    };
private:
    // Integer identities preserve every endpoint/Known bit, including signed
    // zero. They neither reconstruct nor canonicalize a coefficient interval.
    struct FAxisIdentity
    {
        uint64 Lower=0,Upper=0;bool Known=false;
    };
    struct FPointIdentity {FAxisIdentity Axis[3];};
    struct FCellIdentity
    {
        int32 DegreeU=0,DegreeV=0,DepthU=0,DepthV=0;
        uint64 MinU=0,MaxU=0,MinV=0,MaxV=0;
        TArray<FPointIdentity> Controls;
    };
    struct FEntry
    {
        FCellIdentity Geometry;
        FIntervalPoint Middle;
        uint32 RequiredSticky=0;
    };
    struct FRoot
    {
        int32 DegreeU=0,DegreeV=0;uint64 Hash=0,Control=0;
        TArray<FPointIdentity> Controls;
        // Hash collision is never equality. Every candidate index is checked
        // against its COMPLETE integer geometry identity before returning it.
        TMap<uint64,TArray<int32>> Cells;
    };
    static constexpr int32 MaximumRoots=64,MaximumEntries=65536;
    static constexpr uint64 MaximumPoints=1048576;
    const uint32 OwnerThread=FPlatformTLS::GetCurrentThreadId();
    TArray<FRoot> Roots;
    TArray<FEntry> Entries;
    // Root controls + cell controls + one retained midpoint per entry. Arrays
    // use heap storage rather than an inline16 payload in every cached cell.
    uint64 StoredPoints=0;
    uint32 ActiveCalls=0;
    bool Saturated=false;
    FCounters Counters;

    static void Count(uint64& value){if(value<MAX_uint64)++value;}
    static uint64 Mix(uint64 hash,uint64 word)
    {
        // Unsigned wrap is intentional. Full identity checks below make a
        // collision a longer lookup only, never a geometry/proof equivalence.
        return (hash^word)*uint64(1099511628211ULL);
    }
    static bool Sticky(uint64 control,uint32 required,uint32* observed=nullptr)
    {
#if defined(_MSC_VER) && defined(_M_X64) && !defined(_M_ARM64EC) && PLATFORM_ENABLE_VECTORINTRINSICS
        const auto evidence=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(evidence.Status!=EAnalyticNumericStatus::Complete||evidence.ControlFingerprint()!=control)return false;
        const uint32 value=evidence.VectorControl;
        if((value&required)!=required)return false;
        if(observed)*observed=value&0x3f;return true;
#else
        return false;
#endif
    }
    static bool Shape(int32 degreeU,int32 degreeV,int32 count)
    {
        return degreeU>=0&&degreeU<=16&&degreeV>=0&&degreeV<=16&&
            count==(degreeU+1)*(degreeV+1);
    }
    static bool ValidIntervalBits(const Interval& value)
    {
        // Same endpoint acceptance as AnalyticCertified::Valid, implemented
        // entirely with integer operations so memo admission adds no FP flags.
        namespace A=::AnalyticCertified;
        if(!value.Known)return false;
        const uint64 lo=A::Bits(value.Lower),hi=A::Bits(value.Upper);
        const uint64 lm=lo&A::Magnitude,hm=hi&A::Magnitude;
        if(lm>=A::Infinity||hm>=A::Infinity||(lm&&lm<A::MinNormal)||(hm&&hm<A::MinNormal))return false;
        if(!lm&&!hm)return true; // -0 and +0 are ordered equal in the old check.
        const uint64 lk=(lo&A::Sign)?~lo:(lo^A::Sign);
        const uint64 hk=(hi&A::Sign)?~hi:(hi^A::Sign);
        return lk<=hk;
    }
    static bool ValidPointBits(const FIntervalPoint& point)
    {
        for(const auto& axis:point.Axis)if(!ValidIntervalBits(axis))return false;
        return true;
    }
    static FPointIdentity Identity(const FIntervalPoint& point)
    {
        FPointIdentity result;
        for(int32 d=0;d<3;++d)
            result.Axis[d]={AnalyticCertified::Bits(point.Axis[d].Lower),
                AnalyticCertified::Bits(point.Axis[d].Upper),point.Axis[d].Known};
        return result;
    }
    static bool SamePoint(const FPointIdentity& stored,const FIntervalPoint& point)
    {
        for(int32 d=0;d<3;++d)
            if(stored.Axis[d].Known!=point.Axis[d].Known||
                stored.Axis[d].Lower!=AnalyticCertified::Bits(point.Axis[d].Lower)||
                stored.Axis[d].Upper!=AnalyticCertified::Bits(point.Axis[d].Upper))return false;
        return true;
    }
    template<class TControls> static uint64 HashControls(uint64 hash,const TControls& controls)
    {
        hash=Mix(hash,uint64(controls.Num()));
        for(const auto& point:controls)for(const auto& axis:point.Axis)
        {
            hash=Mix(hash,AnalyticCertified::Bits(axis.Lower));
            hash=Mix(hash,AnalyticCertified::Bits(axis.Upper));
            hash=Mix(hash,axis.Known?1u:0u);
        }
        return hash;
    }
    static uint64 RootHash(const FIntervalTensorSurface& surface,uint64 control)
    {
        MinimumMidpointProfile84::FScope profile84(MinimumMidpointProfile84::RootHash);
        uint64 hash=Mix(uint64(14695981039346656037ULL),control);
        hash=Mix(hash,uint64(uint32(surface.DegreeU)));
        hash=Mix(hash,uint64(uint32(surface.DegreeV)));
        return HashControls(hash,surface.ControlPoints);
    }
    static uint64 CellHash(const FNode& node)
    {
        MinimumMidpointProfile84::FScope profile84(MinimumMidpointProfile84::CellHash);
        uint64 hash=uint64(14695981039346656037ULL);
        for(int32 value:{node.DegreeU,node.DegreeV,node.DepthU,node.DepthV})hash=Mix(hash,uint64(uint32(value)));
        for(double value:{node.MinU,node.MaxU,node.MinV,node.MaxV})hash=Mix(hash,AnalyticCertified::Bits(value));
        return HashControls(hash,node.Net);
    }
    static bool SameRoot(const FRoot& stored,const FIntervalTensorSurface& surface,uint64 control,uint64 hash)
    {
        MinimumMidpointProfile84::FScope profile84(MinimumMidpointProfile84::SameRoot);
        if(stored.Hash!=hash||stored.Control!=control||stored.DegreeU!=surface.DegreeU||
            stored.DegreeV!=surface.DegreeV||stored.Controls.Num()!=surface.ControlPoints.Num())return false;
        for(int32 i=0;i<stored.Controls.Num();++i)if(!SamePoint(stored.Controls[i],surface.ControlPoints[i]))return false;
        return true;
    }
    static bool SameCell(const FCellIdentity& stored,const FNode& node)
    {
        MinimumMidpointProfile84::FScope profile84(MinimumMidpointProfile84::SameCell);
        if(stored.DegreeU!=node.DegreeU||stored.DegreeV!=node.DegreeV||
            stored.DepthU!=node.DepthU||stored.DepthV!=node.DepthV||
            stored.MinU!=AnalyticCertified::Bits(node.MinU)||stored.MaxU!=AnalyticCertified::Bits(node.MaxU)||
            stored.MinV!=AnalyticCertified::Bits(node.MinV)||stored.MaxV!=AnalyticCertified::Bits(node.MaxV)||
            stored.Controls.Num()!=node.Net.Num())return false;
        for(int32 i=0;i<stored.Controls.Num();++i)if(!SamePoint(stored.Controls[i],node.Net[i]))return false;
        return true;
    }
    bool CanStore(uint64 points)
    {
        const bool fits=StoredPoints<=MaximumPoints&&points<=MaximumPoints-StoredPoints;
        if(!fits)Saturated=true;
        return fits;
    }
    void EvictIfInactive()
    {
        if(ActiveCalls==0&&Saturated)
        {
            Entries.Empty();Roots.Empty();StoredPoints=0;Saturated=false;
            Count(Counters.Evictions);
        }
    }
    int32 FindOrBindRoot(const FIntervalTensorSurface& surface,uint64 control)
    {
        if(!Shape(surface.DegreeU,surface.DegreeV,surface.ControlPoints.Num()))return INDEX_NONE;
        for(const auto& point:surface.ControlPoints)if(!ValidPointBits(point))return INDEX_NONE;
        const uint64 hash=RootHash(surface,control);
        for(int32 i=Roots.Num()-1;i>=0;--i)if(SameRoot(Roots[i],surface,control,hash))return i;
        if(Roots.Num()>=MaximumRoots){Saturated=true;return INDEX_NONE;}
        if(!CanStore(uint64(surface.ControlPoints.Num())))return INDEX_NONE;
        FRoot root;root.DegreeU=surface.DegreeU;root.DegreeV=surface.DegreeV;root.Hash=hash;root.Control=control;
        root.Controls.Reserve(surface.ControlPoints.Num());
        for(const auto& point:surface.ControlPoints)root.Controls.Add(Identity(point));
        StoredPoints+=uint64(root.Controls.Num());return Roots.Add(MoveTemp(root));
    }
    int32 FindEntry(int32 rootIndex,uint64 hash,const FNode& node) const
    {
        const auto* bucket=Roots[rootIndex].Cells.Find(hash);
        if(bucket)for(int32 index:*bucket)
            if(Entries.IsValidIndex(index)&&SameCell(Entries[index].Geometry,node))return index;
        return INDEX_NONE;
    }
    void Publish(int32 rootIndex,int32 existing,uint64 hash,const FNode& node,const FIntervalPoint& middle,uint32 flags)
    {
        if(existing!=INDEX_NONE)
        {
            Entries[existing].Middle=middle;Entries[existing].RequiredSticky=flags;
            Count(Counters.Builds);return;
        }
        if(Entries.Num()>=MaximumEntries){Saturated=true;return;}
        const uint64 charge=uint64(node.Net.Num())+1;
        if(!CanStore(charge))return;
        FEntry entry;auto& geometry=entry.Geometry;
        geometry.DegreeU=node.DegreeU;geometry.DegreeV=node.DegreeV;
        geometry.DepthU=node.DepthU;geometry.DepthV=node.DepthV;
        geometry.MinU=AnalyticCertified::Bits(node.MinU);geometry.MaxU=AnalyticCertified::Bits(node.MaxU);
        geometry.MinV=AnalyticCertified::Bits(node.MinV);geometry.MaxV=AnalyticCertified::Bits(node.MaxV);
        geometry.Controls.Reserve(node.Net.Num());
        for(const auto& point:node.Net)geometry.Controls.Add(Identity(point));
        entry.Middle=middle;entry.RequiredSticky=flags;
        const int32 index=Entries.Add(MoveTemp(entry));
        Roots[rootIndex].Cells.FindOrAdd(hash).Add(index);StoredPoints+=charge;Count(Counters.Builds);
    }
public:
    FMinimumMidpointMemo()=default;
    FMinimumMidpointMemo(const FMinimumMidpointMemo&)=delete;
    FMinimumMidpointMemo& operator=(const FMinimumMidpointMemo&)=delete;
    // Inspection is passive and owner-thread only. Values count actual requests,
    // returned hits and successful publications, never mathematical proof work.
    const FCounters& GetCountersForInspection() const{return Counters;}
    uint64 GetStoredPointsForInspection() const{return StoredPoints;}
    int32 GetEntriesForInspection() const{return Entries.Num();}
    int32 GetRootsForInspection() const{return Roots.Num();}
    uint32 GetActiveCallsForInspection() const{return ActiveCalls;}

    class FCallScope final
    {
        FMinimumMidpointMemo* Owner=nullptr;
        const FIntervalTensorSurface* Root=nullptr;
        uint64 Control=0;
        uint32 ScopeThread=0;
        int32 RootIndex=INDEX_NONE;
        bool RootAttempted=false,ReuseSameCallBox=true;
    public:
        FCallScope(FMinimumMidpointMemo* memo,const FIntervalTensorSurface& surface,uint64 control,bool reuseSameCallBox=true)
            :Root(&surface),Control(control),ReuseSameCallBox(reuseSameCallBox)
        {
            // Null oracle construction does not inspect the FPU or build any
            // root/key/midpoint. A scope never changes a numerical control bit.
            if(!memo)return;
            ScopeThread=FPlatformTLS::GetCurrentThreadId();
            if(memo->OwnerThread!=ScopeThread||!Sticky(control,0)||memo->ActiveCalls==MAX_uint32)return;
            memo->EvictIfInactive();Owner=memo;++Owner->ActiveCalls;
        }
        ~FCallScope()
        {
            // Cross-thread scope destruction violates the borrowing contract;
            // do not mutate the owner directory from that foreign thread.
            if(Owner&&ScopeThread==FPlatformTLS::GetCurrentThreadId())--Owner->ActiveCalls;
        }
        FCallScope(const FCallScope&)=delete;FCallScope& operator=(const FCallScope&)=delete;
        FCallScope(FCallScope&&)=delete;FCallScope& operator=(FCallScope&&)=delete;
        FCallScope(FMinimumMidpointMemo*,FIntervalTensorSurface&&,uint64,bool=true)=delete;
        FCallScope(FMinimumMidpointMemo*,const FIntervalTensorSurface&&,uint64,bool=true)=delete;

        // Policy only, not a cached value or an extra numeric-context probe.
        // False is an explicit ordinary-Min/Max oracle with the same memo path.
        bool MayReuseSameCallBox() const
        {return Owner&&ReuseSameCallBox&&ScopeThread==FPlatformTLS::GetCurrentThreadId();}
        // Count CPs only in completed ordinary validation passes. Partial or
        // quick-decided/refused passes are excluded from BOTH counters.
        void ObserveOrdinaryBoxPass(uint64 points,bool avoided)
        {
            if(!Owner||ScopeThread!=FPlatformTLS::GetCurrentThreadId())return;
            auto& value=avoided?Owner->Counters.OrdinaryBoxAvoidedCP:Owner->Counters.OrdinaryBoxRecomputedCP;
            value=points>MAX_uint64-value?MAX_uint64:value+points;
        }

        // No geometry or query value enters this guard API. The retained
        // operation privately owns the immutable cell/value association.
        bool HasRetainedMiddleContext(uint32 required=0,uint32* observed=nullptr) const
        {
            return Owner&&ScopeThread==FPlatformTLS::GetCurrentThreadId()&&Sticky(Control,required,observed);
        }
        void ObserveRetainedMiddle(bool hit,bool published)
        {
            if(!HasRetainedMiddleContext())return;
            Count(Owner->Counters.RetainedMiddleRequests);
            if(hit)Count(Owner->Counters.RetainedMiddleHits);
            if(published)Count(Owner->Counters.RetainedMiddlePublications);
        }

        FIntervalPoint Midpoint(const FNode& node)
        {
            MinimumMidpointProfile84::FScope profile84(MinimumMidpointProfile84::CallMidpoint);
            if(!Owner||ScopeThread!=FPlatformTLS::GetCurrentThreadId())return Detail::Midpoint(node);
            Count(Owner->Counters.Requests);
            uint32 lookupSticky=0;
            if(!Sticky(Control,0,&lookupSticky)||!Shape(node.DegreeU,node.DegreeV,node.Net.Num()))return Detail::Midpoint(node);
            if(!RootAttempted)
            {
                RootAttempted=true;RootIndex=Owner->FindOrBindRoot(*Root,Control);
            }
            if(RootIndex==INDEX_NONE)return Detail::Midpoint(node);
            const uint64 hash=CellHash(node);
            const int32 existing=Owner->FindEntry(RootIndex,hash,node);
            if(existing!=INDEX_NONE)
            {
                const auto& entry=Owner->Entries[existing];
                // This successful lookup path is integer-only: shape, complete
                // identities, hash, map search and counters. A new root has no
                // entries, so reaching an existing entry did not allocate or
                // compute a geometric value. No callback/control setter occurs.
                // Reuse THIS invocation's fresh evidence only; cold arithmetic
                // below still performs its original publication-time inspection.
                if((lookupSticky&entry.RequiredSticky)==entry.RequiredSticky)
                {Count(Owner->Counters.Hits);return entry.Middle;}
            }
            // EXACT original arithmetic at its old call site. No eager sample,
            // derivative, hull, query witness, debit, threshold or proof branch.
            const auto middle=Detail::Midpoint(node);
            uint32 flags=0;
            if(ValidPointBits(middle)&&Sticky(Control,0,&flags))
                Owner->Publish(RootIndex,existing,hash,node,middle,flags);
            return middle;
        }
    };
};

inline FMinimumMidpointMemo& ThreadMinimumMidpointMemo()
{
    static thread_local TUniquePtr<FMinimumMidpointMemo> memo;
    if(!memo)memo=MakeUnique<FMinimumMidpointMemo>();return *memo;
}

enum class EDirectionalHullLower : uint8 {NotApplicable,Known,Unknown};
inline EDirectionalHullLower BoundDirectionalHullLower(const FNode& node,
    const FIntervalPoint& center,const FVector3d& direction,double& lowerSquared)
{
    lowerSquared=0;
    if(node.Net.IsEmpty()||!ValidPoint(center))return EDirectionalHullLower::Unknown;
    bool nonzero=false;
    for(int32 d=0;d<3;++d)
    {
        if(!AnalyticCertified::Finite(direction[d]))return EDirectionalHullLower::Unknown;
        nonzero|=(AnalyticCertified::Bits(direction[d])&AnalyticCertified::Magnitude)!=0;
    }
    if(!nonzero)return EDirectionalHullLower::NotApplicable;
    auto normSquared=Interval::Exact(0);
    for(int32 d=0;d<3;++d)normSquared=AnalyticCertified::Add(normSquared,
        AnalyticCertified::Square(Interval::Exact(direction[d])));
    if(!AnalyticCertified::Valid(normSquared)||!(normSquared.Upper>0))return EDirectionalHullLower::Unknown;
    double low=0,high=0;bool first=true;
    for(const auto& point:node.Net)
    {
        if(!ValidPoint(point))return EDirectionalHullLower::Unknown;
        auto projection=Interval::Exact(0);
        for(int32 d=0;d<3;++d)projection=AnalyticCertified::Add(projection,AnalyticCertified::Multiply(
            Interval::Exact(direction[d]),AnalyticCertified::Subtract(point.Axis[d],center.Axis[d])));
        if(!AnalyticCertified::Valid(projection))return EDirectionalHullLower::Unknown;
        if(first||projection.Lower<low)low=projection.Lower;
        if(first||projection.Upper>high)high=projection.Upper;
        first=false;
    }
    // Every fixed polynomial lies in the convex hull of ALL its coefficients.
    // Every center belongs to its full box. No witness-only or tangent argument.
    if(low<=0&&high>=0)return EDirectionalHullLower::Known;
    const double separation=low>0?low:high;
    const auto numerator=AnalyticCertified::Square(Interval::Exact(separation));
    if(!AnalyticCertified::Valid(numerator))return EDirectionalHullLower::Unknown;
    // Nonzero stored d proves ||d||^2>0 in real arithmetic. The denominator's
    // certified upper, not an approximate unit length, gives a lower quotient.
    volatile double quotient=FMath::Max(0.,numerator.Lower)/normSquared.Upper;
    double outwardLower=0;
    if(!AnalyticCertified::Outward(quotient,false,outwardLower))return EDirectionalHullLower::Unknown;
    lowerSquared=FMath::Max(0.,outwardLower);
    return AnalyticCertified::Finite(lowerSquared)?EDirectionalHullLower::Known:EDirectionalHullLower::Unknown;
}

// Root-only separating supports. Directions are merely proposals; their
// certified projections contain ALL four independent coefficient boxes.
// An entire center-path hull is used, never just its endpoints as witnesses.
inline bool SeparatePlanarEdgeHull(const FIntervalTensorSurface& surface,
    const FIntervalPoint& center,Interval radiusSquared,FDistanceBudget& budget,bool& separated)
{
    separated=false;
    if(surface.DegreeU!=1||surface.DegreeV!=1||surface.ControlPoints.Num()!=4)return true;
    int32 axis=INDEX_NONE;
    for(int32 d=0;d<3&&axis==INDEX_NONE;++d)
    {
        const auto h=surface.ControlPoints[0].Axis[d];
        if(!AnalyticCertified::Valid(h)||AnalyticCertified::Bits(h.Lower)!=AnalyticCertified::Bits(h.Upper))continue;
        bool same=true;
        for(const auto& p:surface.ControlPoints)
            same&=AnalyticCertified::Bits(p.Axis[d].Lower)==AnalyticCertified::Bits(h.Lower)&&
                AnalyticCertified::Bits(p.Axis[d].Upper)==AnalyticCertified::Bits(h.Upper);
        if(same)axis=d;
    }
    if(axis==INDEX_NONE)return true;
    // Fixed bound for <=4 coefficients,3 directions,3 dimensions. No UV walk.
    constexpr int32 WorkBound=512;
    if(budget.ConsumptionFloor<0||int64(budget.Remaining)-budget.ConsumptionFloor<WorkBound||
        budget.Used>MAX_uint64-WorkBound)return false;
    for(int32 k=0;k<WorkBound;++k)if(!budget.Consume())return false;
    FNode root;root.DegreeU=surface.DegreeU;root.DegreeV=surface.DegreeV;root.Net=surface.ControlPoints;
    const int32 x=(axis+1)%3,y=(axis+2)%3;
    const auto axial=AnalyticCertified::Square(AnalyticCertified::Subtract(surface.ControlPoints[0].Axis[axis],center.Axis[axis]));
    if(!AnalyticCertified::Valid(axial))return false;
    constexpr int32 edges[3][2]={{0,1},{1,2},{2,0}};
    for(const auto& edge:edges)
    {
        const auto& a=surface.ControlPoints[edge[0]];const auto& b=surface.ControlPoints[edge[1]];
        FVector3d direction=FVector3d::ZeroVector;
        direction[x]=(.5*b.Axis[y].Lower+.5*b.Axis[y].Upper)-(.5*a.Axis[y].Lower+.5*a.Axis[y].Upper);
        direction[y]=(.5*a.Axis[x].Lower+.5*a.Axis[x].Upper)-(.5*b.Axis[x].Lower+.5*b.Axis[x].Upper);
        const double scale=FMath::Max(FMath::Abs(direction[x]),FMath::Abs(direction[y]));
        if(!AnalyticCertified::Finite(scale)||!(scale>0))continue;
        direction[x]/=scale;direction[y]/=scale;
        double lateral=0;
        const auto status=BoundDirectionalHullLower(root,center,direction,lateral);
        if(status==EDirectionalHullLower::Unknown)return false;
        if(status!=EDirectionalHullLower::Known)continue;
        const auto total=AnalyticCertified::Add(Interval::Exact(FMath::Max(0.,axial.Lower)),Interval::Exact(lateral));
        if(!AnalyticCertified::Valid(total))return false;
        if(total.Lower>radiusSquared.Upper){separated=true;return true;}
    }
    return true;
}

inline EDirectionalHullLower ProposeDirectionalHullLower(const FNode& node,
    const FIntervalPoint& center,const FIntervalPoint& witness,double& lowerSquared)
{
    lowerSquared=0;FVector3d direction=FVector3d::ZeroVector;double scale=0;
    // This is only a proposal. Each resulting stored component becomes an
    // exact constant of the subsequent proof, irrespective of rounding here.
    for(int32 d=0;d<3;++d)
    {
        const double a=.5*witness.Axis[d].Lower+.5*witness.Axis[d].Upper;
        const double b=.5*center.Axis[d].Lower+.5*center.Axis[d].Upper;
        direction[d]=a-b;
        if(!AnalyticCertified::Finite(direction[d]))return EDirectionalHullLower::NotApplicable;
        scale=FMath::Max(scale,FMath::Abs(direction[d]));
    }
    if(!(scale>0))return EDirectionalHullLower::NotApplicable;
    for(int32 d=0;d<3;++d)
    {
        direction[d]/=scale;
        if(!AnalyticCertified::Finite(direction[d]))return EDirectionalHullLower::NotApplicable;
    }
    return BoundDirectionalHullLower(node,center,direction,lowerSquared);
}


// Root-only direct projection/inverse. Floating-model arithmetic chooses UV;
// the ORIGINAL net at that fixed global UV is the sole upper witness.
inline bool ImprovePlaneProposal95(const FNode& n,const FIntervalPoint& center,
    double& lower,double& upper,FVector2d& witnessUV,FIntervalPoint& witnessPoint,
    const Interval* threshold,PlaneProposal95::FWork* work)
{
    namespace Q=PlaneProposal95;namespace A=::AnalyticCertified;
    if(!work||!work->Original||n.DepthU||n.DepthV||
        n.MinU!=0||n.MaxU!=1||n.MinV!=0||n.MaxV!=1||(n.DegreeU==0&&n.DegreeV==0)||!Q::Enabled())return true;
    if(threshold&&(lower>threshold->Upper||upper<threshold->Lower))return true;
    Q::FObservedScope observed;
    const auto& original=*work->Original;
    const uint64 controls=uint64(original.ControlPoints.Num());
    const uint64 blends=uint64(original.DegreeU+1)*uint64(original.DegreeV)*uint64(original.DegreeV+1)/2+
        uint64(original.DegreeU)*uint64(original.DegreeU+1)/2;
    // Preserve the original cheap one-node/coarse route before any proposal
    // admission. This is the exact worst added work, not a saved-work heuristic.
    const uint64 maximum=1+controls+blends+3+uint64(n.Net.Num())+1;
    if(work->AvailableLocal()<maximum){Q::Add(Q::SkippedLocal);return true;}
    if(!work->Pay(1))return false;Q::Add(Q::Admissions);
    const uint64 numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
    const int32 vc=n.DegreeV+1;
    const FIntervalPoint* corners[]={&n.Net[0],&n.Net[n.DegreeU*vc],&n.Net[n.DegreeV],&n.Net[n.DegreeU*vc+n.DegreeV]};
    FVector3d p[4],target;double scale=0;
    for(int32 d=0;d<3;++d)
    {
        target[d]=.5*center.Axis[d].Lower+.5*center.Axis[d].Upper;
        if(!A::Finite(target[d])){Q::Add(Q::SkippedInverse);return true;}
        scale=FMath::Max(scale,FMath::Abs(target[d]));
        for(int32 k=0;k<4;++k)
        {
            p[k][d]=.5*corners[k]->Axis[d].Lower+.5*corners[k]->Axis[d].Upper;
            if(!A::Finite(p[k][d])){Q::Add(Q::SkippedInverse);return true;}
            scale=FMath::Max(scale,FMath::Abs(p[k][d]));
        }
    }
    if(!(scale>0)||!A::Finite(scale)){Q::Add(Q::SkippedInverse);return true;}
    for(int32 d=0;d<3;++d){target[d]/=scale;for(auto& point:p)point[d]/=scale;}
    const FVector3d du=p[1]-p[0],dv=p[2]-p[0],offset=target-p[0];
    const double a=FVector3d::DotProduct(du,du),b=FVector3d::DotProduct(du,dv),
        c=FVector3d::DotProduct(dv,dv),x=FVector3d::DotProduct(offset,du),y=FVector3d::DotProduct(offset,dv);
    double u=0,v=0;
    if(n.DegreeU==0)
    {if(!(c>0)){Q::Add(Q::SkippedInverse);return true;}v=y/c;}
    else if(n.DegreeV==0)
    {if(!(a>0)){Q::Add(Q::SkippedInverse);return true;}u=x/a;}
    else
    {
        const double determinant=a*c-b*b;
        if(!(determinant>0)||!A::Finite(determinant)){Q::Add(Q::SkippedInverse);return true;}
        u=(x*c-y*b)/determinant;v=(y*a-x*b)/determinant;
        // Collapsed-corner triangle model only proposes its inverse. Equal
        // midpoint bits do NOT establish correlated/equal exact coefficients.
        bool collapsed=true;for(int32 d=0;d<3;++d)collapsed&=A::Bits(p[1][d])==A::Bits(p[3][d]);
        if(collapsed)
        {if(!(1.-u>0)){Q::Add(Q::SkippedInverse);return true;}v/=(1.-u);}
    }
    if(!A::Finite(u)||!A::Finite(v)){Q::Add(Q::SkippedInverse);return true;}
    const FVector2d uv(FMath::Clamp(u,0.,1.),FMath::Clamp(v,0.,1.));
    // Paid full-net copies, de Casteljau point blends, and distance axes.
    if(!work->Pay(controls+blends+3))return false;
    FIntervalPoint point;
    const auto numericFailure=[&](){Q::Add(Q::NumericFail);return false;};
    if(!ProjectedDistanceDetail::EvaluateFixedUV(original,uv,point))return numericFailure();
    auto distance=Interval::Exact(0);
    for(int32 d=0;d<3;++d)distance=A::Add(distance,A::Square(A::Subtract(center.Axis[d],point.Axis[d])));
    if(!A::Valid(distance)||RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=numeric)
        return numericFailure();
    Q::Add(Q::Evaluated);
    // The midpoint/corners and their original winner on ties stay intact.
    if(!(distance.Upper<upper))return true;
    if(!work->Pay(uint64(n.Net.Num())+1))return false;
    double directional=0;
    if(ProposeDirectionalHullLower(n,center,point,directional)==EDirectionalHullLower::Unknown)return numericFailure();
    const double improvedLower=FMath::Max(lower,directional);
    if(improvedLower>distance.Upper)return numericFailure();
    const auto gap=A::Subtract(Interval::Exact(distance.Upper),Interval::Exact(improvedLower));
    if(!A::Valid(gap))return numericFailure();
    lower=improvedLower;upper=distance.Upper;witnessUV=uv;witnessPoint=MoveTemp(point);Q::Add(Q::Improved);
    if(threshold&&(lower>threshold->Upper||upper<threshold->Lower))Q::Add(Q::ThresholdDecided);
    if(!threshold&&gap.Upper<=work->RequestedGap)Q::Add(Q::GapClosed);
    return true;
}

// Threshold-only conservative early decision. The corner is an actual
// fixed parameter of THIS restricted tensor, not a nominal point or a hit.
// An undecided result continues the same ordinary bound/account; arithmetic
// failure is terminal. Every leaf/provider remains in the existing heap/union.
enum class EThresholdQuickResult:uint8 {Decided,Undecided,Unknown};
struct FThresholdQuickWitness
{
    double LowerSquared=0,UpperSquared=0;
    FIntervalPoint Point;FVector2d UV=FVector2d::ZeroVector;
};
// An immediate same-invocation stack result ONLY. It contains no node,
// center, threshold, witness, authority or persistent geometry association.
// Only Undecided publishes the six exact endpoints of the original scan.
struct FSameCallQuickBox
{
    double Low[3]{},High[3]{};bool Ready=false;
    FSameCallQuickBox()=default;
    FSameCallQuickBox(const FSameCallQuickBox&)=delete;
    FSameCallQuickBox& operator=(const FSameCallQuickBox&)=delete;
};
inline EThresholdQuickResult TryThresholdCornerAndBox(const FNode& n,const FIntervalPoint& center,
    const Interval* threshold,FThresholdQuickWitness& out,FSameCallQuickBox* sameCallBox=nullptr)
{
    if(sameCallBox)sameCallBox->Ready=false;
    out={};if(!threshold)return EThresholdQuickResult::Undecided;
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    namespace A=::AnalyticCertified;
    if(!A::Valid(*threshold)||!ValidPoint(center)||n.DegreeU<0||n.DegreeV<0||n.DegreeU>16||n.DegreeV>16||
        n.Net.Num()!=(n.DegreeU+1)*(n.DegreeV+1)||n.Net.IsEmpty()||
        !A::Finite(n.MinU)||!A::Finite(n.MaxU)||!A::Finite(n.MinV)||!A::Finite(n.MaxV)||
        n.MinU<0||n.MaxU>1||n.MinV<0||n.MaxV>1||n.MinU>n.MaxU||n.MinV>n.MaxV)return EThresholdQuickResult::Unknown;
    double low[3],high[3];
    for(int32 d=0;d<3;++d){low[d]=n.Net[0].Axis[d].Lower;high[d]=n.Net[0].Axis[d].Upper;}
    for(const auto& cp:n.Net)
    {
        if(!ValidPoint(cp))return EThresholdQuickResult::Unknown;
        for(int32 d=0;d<3;++d){low[d]=FMath::Min(low[d],cp.Axis[d].Lower);high[d]=FMath::Max(high[d],cp.Axis[d].Upper);}
    }
    auto lower=Interval::Exact(0),upper=Interval::Exact(0);
    for(int32 d=0;d<3;++d)
    {
        auto gap=Interval::Exact(0);
        if(center.Axis[d].Lower>high[d])gap=A::Subtract(Interval::Exact(center.Axis[d].Lower),Interval::Exact(high[d]));
        else if(center.Axis[d].Upper<low[d])gap=A::Subtract(Interval::Exact(low[d]),Interval::Exact(center.Axis[d].Upper));
        lower=A::Add(lower,A::Square(gap));
        upper=A::Add(upper,A::Square(A::Subtract(center.Axis[d],n.Net[0].Axis[d])));
    }
    if(!A::Valid(lower)||!A::Valid(upper))return EThresholdQuickResult::Unknown;
    const double lo=FMath::Max(0.,lower.Lower),hi=upper.Upper;
    if(!A::Finite(lo)||!A::Finite(hi)||lo<0||hi<0||lo>hi)return EThresholdQuickResult::Unknown;
    if(!(lo>threshold->Upper||hi<threshold->Lower))
    {
        if(sameCallBox)
        {
            for(int32 d=0;d<3;++d){sameCallBox->Low[d]=low[d];sameCallBox->High[d]=high[d];}
            sameCallBox->Ready=true;
        }
        return EThresholdQuickResult::Undecided;
    }
    FThresholdQuickWitness result;result.LowerSquared=lo;result.UpperSquared=hi;
    result.Point=n.Net[0];result.UV=FVector2d(n.MinU,n.MinV);out=MoveTemp(result);
    return EThresholdQuickResult::Decided;
}

inline bool Bounds(FNode& n, const FIntervalPoint& center,const Interval* threshold=nullptr,FMinimumMidpointMemo::FCallScope* midpointMemo=nullptr,PlaneProposal95::FWork* proposal95=nullptr)
{
    FThresholdQuickWitness quick;FSameCallQuickBox quickBox;
    const auto decision=TryThresholdCornerAndBox(n,center,threshold,quick,
        midpointMemo&&midpointMemo->MayReuseSameCallBox()?&quickBox:nullptr);
    if(decision==EThresholdQuickResult::Unknown)return false;
    if(decision==EThresholdQuickResult::Decided)
    {
        n.LowerSquared=quick.LowerSquared;n.UpperSquared=quick.UpperSquared;
        n.WitnessPoint=MoveTemp(quick.Point);n.WitnessUV=quick.UV;return true;
    }
    if (n.Net.IsEmpty() || !ValidPoint(center)) return false;
    double low[3],high[3];const bool reuseBox=quickBox.Ready;
    for (int32 d=0; d<3; ++d)
    { low[d]=reuseBox?quickBox.Low[d]:n.Net[0].Axis[d].Lower; high[d]=reuseBox?quickBox.High[d]:n.Net[0].Axis[d].Upper; }
    for (const auto& p : n.Net)
    {
        if (!ValidPoint(p)) return false;
        if(!reuseBox)for (int32 d=0; d<3; ++d)
        { low[d]=FMath::Min(low[d],p.Axis[d].Lower); high[d]=FMath::Max(high[d],p.Axis[d].Upper); }
    }
    if(midpointMemo)midpointMemo->ObserveOrdinaryBoxPass(uint64(n.Net.Num()),reuseBox);
    // Convex control hull is inside this box for the exact restricted polynomial.
    Interval lower=Interval::Exact(0),upper=Interval::Exact(0);
    n.WitnessPoint=midpointMemo?midpointMemo->Midpoint(n):Midpoint(n); if (!ValidPoint(n.WitnessPoint)) return false;
    for (int32 d=0; d<3; ++d)
    {
        Interval delta=Interval::Exact(0);
        if (center.Axis[d].Lower>high[d])
            delta=AnalyticCertified::Subtract(Interval::Exact(center.Axis[d].Lower),Interval::Exact(high[d]));
        else if (center.Axis[d].Upper<low[d])
            delta=AnalyticCertified::Subtract(Interval::Exact(low[d]),Interval::Exact(center.Axis[d].Upper));
        lower=AnalyticCertified::Add(lower,AnalyticCertified::Square(delta));
        const auto difference=AnalyticCertified::Subtract(center.Axis[d],n.WitnessPoint.Axis[d]);
        upper=AnalyticCertified::Add(upper,AnalyticCertified::Square(difference));
    }
    if (!AnalyticCertified::Valid(lower) || !AnalyticCertified::Valid(upper)) return false;
    n.LowerSquared=FMath::Max(0.,lower.Lower); n.UpperSquared=upper.Upper;
    n.WitnessUV=FVector2d(0.5*(n.MinU+n.MaxU),0.5*(n.MinV+n.MaxV));
    // Corners are exact parameters of the restricted polynomial, including
    // collapsed original edges. A midpoint-only witness can waste the budget
    // refining infinitely many equivalent parameters along such an edge.
    // Each candidate upper bound is universal over the SAME coefficient boxes.
    const int32 vc=n.DegreeV+1;
    for(int32 i=0;i<2;++i)for(int32 j=0;j<2;++j)
    {
        const auto& point=n.Net[(i?n.DegreeU:0)*vc+(j?n.DegreeV:0)];
        auto cornerUpper=Interval::Exact(0);
        for(int32 d=0;d<3;++d)
            cornerUpper=AnalyticCertified::Add(cornerUpper,AnalyticCertified::Square(AnalyticCertified::Subtract(center.Axis[d],point.Axis[d])));
        if(!AnalyticCertified::Valid(cornerUpper))return false;
        if(cornerUpper.Upper<n.UpperSquared)
        {
            n.UpperSquared=cornerUpper.Upper;n.WitnessPoint=point;
            n.WitnessUV=FVector2d(i?n.MaxU:n.MinU,j?n.MaxV:n.MinV);
        }
    }
    double directionalLower=0;
    if(ProposeDirectionalHullLower(n,center,n.WitnessPoint,directionalLower)==EDirectionalHullLower::Unknown)return false;
    n.LowerSquared=FMath::Max(n.LowerSquared,directionalLower);
    if(!ImprovePlaneProposal95(n,center,n.LowerSquared,n.UpperSquared,n.WitnessUV,n.WitnessPoint,threshold,proposal95))return false;
    return n.LowerSquared<=n.UpperSquared;
}
inline bool Split(const FNode& n, bool alongU, FNode& a, FNode& b)
{
    // Keep every retained midpoint exactly representable: a dyadic grid of
    // depth <=52 has midpoints of depth <=53 in [0,1]. Stop with valid bounds
    // at this representation limit; never certify a rounded UV as the witness.
    if ((alongU ? n.DepthU : n.DepthV) >= 52) return false;
    a=n; b=n;
    const double middle=alongU ? 0.5*(n.MinU+n.MaxU) : 0.5*(n.MinV+n.MaxV);
    if (alongU)
    { if (!(middle>n.MinU && middle<n.MaxU)) return false; a.MaxU=b.MinU=middle; ++a.DepthU; ++b.DepthU; }
    else
    { if (!(middle>n.MinV && middle<n.MaxV)) return false; a.MaxV=b.MinV=middle; ++a.DepthV; ++b.DepthV; }
    const int32 count=alongU?n.DegreeU+1:n.DegreeV+1;
    const int32 lanes=alongU?n.DegreeV+1:n.DegreeU+1,vc=n.DegreeV+1;
    TArray<FIntervalPoint,TInlineAllocator<16>> work;work.SetNum(count);
    for (int32 lane=0;lane<lanes;++lane)
    {
        const auto index=[=](int32 i){return alongU?i*vc+lane:lane*vc+i;};
        for (int32 i=0;i<count;++i)work[i]=n.Net[index(i)];
        a.Net[index(0)]=work[0];b.Net[index(count-1)]=work[count-1];
        for (int32 level=1;level<count;++level)
        {
            for (int32 i=0;i<count-level;++i)work[i]=Half(work[i],work[i+1]);
            a.Net[index(level)]=work[0];b.Net[index(count-1-level)]=work[count-1-level];
        }
    }
    return true;
}


// Partition schedule only: physical control differences choose a direction.
// These scores are NEVER a separation, exact-extrusion, contact or derivative
// certificate. Independent coefficient boxes remain independent. Every result
// is still proved by the unchanged universal interval Bounds and Split.
inline bool ChooseTensorDistanceSplitU(const FNode& n)
{
    const bool balanced=n.DegreeU>0&&(n.DegreeV==0||n.DepthU<=n.DepthV);
    const bool canU=n.DegreeU>0&&n.DepthU<52,canV=n.DegreeV>0&&n.DepthV<52;
    if(!canU&&!canV)return balanced; // Preserve the representability stop.
    if(!canV)return true;if(!canU)return false;
    double score[2]={0.,0.};bool known=true;
    const auto add=[&](int32 direction,const FIntervalPoint& a,const FIntervalPoint& b,int32 degree)
    {
        for(int32 d=0;d<3;++d)
        {
            const auto delta=AnalyticCertified::Subtract(a.Axis[d],b.Axis[d]);
            if(!AnalyticCertified::Valid(delta)){known=false;return;}
            const double size=FMath::Max(FMath::Abs(delta.Lower),FMath::Abs(delta.Upper));
            const auto scaled=AnalyticCertified::Multiply(Interval::Exact(size),Interval::Exact(double(degree)));
            if(!AnalyticCertified::Valid(scaled)){known=false;return;}
            score[direction]=FMath::Max(score[direction],scaled.Upper);
        }
    };
    const int32 vc=n.DegreeV+1;
    for(int32 i=0;i<=n.DegreeU&&known;++i)for(int32 j=0;j<=n.DegreeV&&known;++j)
    {
        if(i<n.DegreeU)add(0,n.Net[i*vc+j],n.Net[(i+1)*vc+j],n.DegreeU);
        if(j<n.DegreeV&&known)add(1,n.Net[i*vc+j],n.Net[i*vc+j+1],n.DegreeV);
    }
    return !known||score[0]==score[1]?balanced:score[0]>score[1];
}

// The old linear scan selected the smallest (LowerSquared, leaf index).
// Keep indices, never pointers into the reallocating leaves array. Callers
// remove an index before changing its key and then insert it exactly once.
class FMinLowerLeafHeap final
{
    TArray<int32,TInlineAllocator<16>> Indices;
    template<class TLeaf> static bool Before(const TArray<TLeaf>& leaves,int32 a,int32 b)
    {
        return leaves[a].LowerSquared<leaves[b].LowerSquared ||
            (leaves[a].LowerSquared==leaves[b].LowerSquared && a<b);
    }
public:
    int32 Num() const {return Indices.Num();}
    int32 PeekIndex() const {return Indices[0];}
    template<class TLeaf> void PushIndex(int32 index,const TArray<TLeaf>& leaves)
    {
        int32 position=Indices.Add(index);
        while(position>0)
        {
            const int32 parent=(position-1)/2;
            if(!Before(leaves,index,Indices[parent]))break;
            Indices[position]=Indices[parent];position=parent;
        }
        Indices[position]=index;
    }
    template<class TLeaf> void PopRoot(const TArray<TLeaf>& leaves)
    {
        const int32 last=Indices.Pop(EAllowShrinking::No);
        if(Indices.IsEmpty())return;
        int32 position=0;
        // Guard before 2*position+1, including even heap sizes.
        while(position<Indices.Num()/2)
        {
            int32 child=position*2+1;
            if(child+1<Indices.Num()&&Before(leaves,Indices[child+1],Indices[child]))++child;
            if(!Before(leaves,Indices[child],last))break;
            Indices[position]=Indices[child];position=child;
        }
        Indices[position]=last;
    }
};
}
namespace MinimumDetail
{
inline FIntervalPoint Subtract(const FIntervalPoint& a,const FIntervalPoint& b)
{
    FIntervalPoint r;for(int32 d=0;d<3;++d)r.Axis[d]=AnalyticCertified::Subtract(a.Axis[d],b.Axis[d]);return r;
}
inline Interval Dot(const FIntervalPoint& a,const FIntervalPoint& b)
{
    auto r=Interval::Exact(0);for(int32 d=0;d<3;++d)r=AnalyticCertified::Add(r,AnalyticCertified::Multiply(a.Axis[d],b.Axis[d]));return r;
}
inline bool NetHull(const Detail::FNode& n,FIntervalPoint& p)
{
    if(n.Net.IsEmpty())return false;p=n.Net[0];
    for(const auto& q:n.Net)
    {
        if(!Detail::ValidPoint(q))return false;
        for(int32 d=0;d<3;++d)
        {p.Axis[d].Lower=FMath::Min(p.Axis[d].Lower,q.Axis[d].Lower);p.Axis[d].Upper=FMath::Max(p.Axis[d].Upper,q.Axis[d].Upper);}
    }
    return Detail::ValidPoint(p);
}
inline bool DerivativeHull(const Detail::FNode& n,bool alongU,FIntervalPoint& derivative)
{
    // Derivative in the restricted node's local parameter. The omitted original
    // parameter width is positive, so stationary/KKT signs are unchanged.
    const int32 degree=alongU?n.DegreeU:n.DegreeV,vc=n.DegreeV+1;
    for(int32 d=0;d<3;++d)derivative.Axis[d]=Interval::Exact(0);
    if(degree==0)return true;
    bool first=true;
    for(int32 i=0;i<(alongU?n.DegreeU:n.DegreeU+1);++i)
        for(int32 j=0;j<(alongU?n.DegreeV+1:n.DegreeV);++j)
        {
            const int32 a=i*vc+j,b=alongU?(i+1)*vc+j:i*vc+j+1;
            for(int32 d=0;d<3;++d)
            {
                const auto x=AnalyticCertified::Multiply(Interval::Exact(double(degree)),AnalyticCertified::Subtract(n.Net[b].Axis[d],n.Net[a].Axis[d]));
                if(!AnalyticCertified::Valid(x))return false;
                if(first)derivative.Axis[d]=x;
                else {derivative.Axis[d].Lower=FMath::Min(derivative.Axis[d].Lower,x.Lower);derivative.Axis[d].Upper=FMath::Max(derivative.Axis[d].Upper,x.Upper);}
            }
            first=false;
        }
    return !first&&Detail::ValidPoint(derivative);
}
inline bool CanContainMinimum(Interval gradient,double minParameter,double maxParameter)
{
    // Gradient is (S-C).dS. An original lower edge requires gradient>=0,
    // an original upper edge requires gradient<=0, and an interior needs0.
    // Artificial subdivision boundaries NEVER acquire original-edge status.
    if(gradient.Lower>0 && minParameter>0)return false;
    if(gradient.Upper<0 && maxParameter<1)return false;
    return true;
}
enum class ECellMinimum:uint8 {Unknown,Possible,Excluded};
// n is an exact dyadic restriction of the original full finite tensor domain.
// This only excludes cells incapable of containing any GLOBAL minimum, for
// every fixed polynomial/center represented by the independent boxes.
struct FMinimumGeometryHulls
{
    FIntervalPoint Hull,Du,Dv;
};
inline bool ReadMinimumGeometryHulls(const Detail::FNode& n,FMinimumGeometryHulls& bounds)
{
    return NetHull(n,bounds.Hull)&&DerivativeHull(n,true,bounds.Du)&&DerivativeHull(n,false,bounds.Dv);
}
template<class FReadGeometry>
inline ECellMinimum ClassifyMinimumWithGeometry(const Detail::FNode& n,const FIntervalPoint& center,
    FReadGeometry&& read)
{
    if(n.DegreeU<0||n.DegreeU>16||n.DegreeV<0||n.DegreeV>16||n.Net.Num()!=(n.DegreeU+1)*(n.DegreeV+1)||
        !Detail::ValidPoint(center)||!AnalyticCertified::Finite(n.MinU)||!AnalyticCertified::Finite(n.MaxU)||
        !AnalyticCertified::Finite(n.MinV)||!AnalyticCertified::Finite(n.MaxV)||
        n.MinU<0||n.MaxU>1||!(n.MinU<n.MaxU)||n.MinV<0||n.MaxV>1||!(n.MinV<n.MaxV))return ECellMinimum::Unknown;
    FMinimumGeometryHulls geometry;
    if(!read(n,geometry))return ECellMinimum::Unknown;
    const auto delta=Subtract(geometry.Hull,center);if(!Detail::ValidPoint(delta))return ECellMinimum::Unknown;
    const auto gu=Dot(delta,geometry.Du),gv=Dot(delta,geometry.Dv);
    if(!AnalyticCertified::Valid(gu)||!AnalyticCertified::Valid(gv))return ECellMinimum::Unknown;
    return CanContainMinimum(gu,n.MinU,n.MaxU)&&CanContainMinimum(gv,n.MinV,n.MaxV)?ECellMinimum::Possible:ECellMinimum::Excluded;
}
inline ECellMinimum ClassifyMinimum(const Detail::FNode& n,const FIntervalPoint& center)
{
    return ClassifyMinimumWithGeometry(n,center,ReadMinimumGeometryHulls);
}
}


namespace ProjectedDistanceDetail
{
// Optional work allocation only. The family still contains every independent
// coefficient in the original boxes; identical boxes do not correlate reals.
inline bool IsUsefulYZFamily(const FIntervalTensorSurface& surface)
{
    if(surface.DegreeU!=5||surface.DegreeV!=5||surface.ControlPoints.Num()!=36)return false;
    for(int32 v=0;v<6;++v)for(int32 u=1;u<6;++u)for(int32 d=1;d<3;++d)
    {
        const auto& a=surface.ControlPoints[v].Axis[d];
        const auto& b=surface.ControlPoints[u*6+v].Axis[d];
        if(AnalyticCertified::Bits(a.Lower)!=AnalyticCertified::Bits(b.Lower)||
            AnalyticCertified::Bits(a.Upper)!=AnalyticCertified::Bits(b.Upper))return false;
    }
    return true;
}
// The captured geometry also contains curves swept along world Y.
// A repeated interval box still encloses independent real coefficients.
// Only the lower bound is projected; every upper witness uses the full net.
inline int32 ExtrusionAxis(const FIntervalTensorSurface& surface)
{
    if(IsUsefulYZFamily(surface))return 0;
    if(surface.DegreeU!=5||surface.DegreeV!=5||surface.ControlPoints.Num()!=36)return INDEX_NONE;
    // A single changed Y coefficient in an otherwise rejected patch is not
    // a useful Y extrusion. Require the XZ profile to repeat and every Y
    // row to precede the next one strictly, in one direction.
    bool increasing=true,decreasing=true;
    Interval previous;
    for(int32 u=0;u<6;++u)
    {
        Interval y=surface.ControlPoints[u*6].Axis[1];
        if(!AnalyticCertified::Valid(y))return INDEX_NONE;
        for(int32 v=0;v<6;++v)
        {
            const auto& point=surface.ControlPoints[u*6+v];
            const auto& profile=surface.ControlPoints[v];
            if(!AnalyticCertified::Valid(point.Axis[1]))return INDEX_NONE;
            for(int32 d:{0,2})
                if(AnalyticCertified::Bits(point.Axis[d].Lower)!=AnalyticCertified::Bits(profile.Axis[d].Lower)||
                    AnalyticCertified::Bits(point.Axis[d].Upper)!=AnalyticCertified::Bits(profile.Axis[d].Upper))
                    return INDEX_NONE;
            y.Lower=FMath::Min(y.Lower,point.Axis[1].Lower);
            y.Upper=FMath::Max(y.Upper,point.Axis[1].Upper);
        }
        if(u)
        {
            increasing&=previous.Upper<y.Lower;
            decreasing&=y.Upper<previous.Lower;
            if(!increasing&&!decreasing)return INDEX_NONE;
        }
        previous=y;
    }
    return 1;
}
inline FIntervalPoint ProjectCenterToYZ(const FIntervalPoint& center,int32 dropped)
{
    return {{Interval::Exact(0),center.Axis[(dropped+1)%3],center.Axis[(dropped+2)%3]}};
}
inline FIntervalTensorSurface ProjectYZ(const FIntervalTensorSurface& surface,int32 dropped=0)
{
    FIntervalTensorSurface projected;projected.DegreeU=0;projected.DegreeV=surface.DegreeV;
    const int32 vc=surface.DegreeV+1;
    for(int32 v=0;v<vc;++v)
    {
        auto point=ProjectCenterToYZ(surface.ControlPoints[v],dropped);
        for(int32 u=1;u<=surface.DegreeU;++u)for(int32 d=1;d<3;++d)
        {
            const auto& other=surface.ControlPoints[u*vc+v].Axis[(dropped+d)%3];
            point.Axis[d].Lower=FMath::Min(point.Axis[d].Lower,other.Lower);
            point.Axis[d].Upper=FMath::Max(point.Axis[d].Upper,other.Upper);
        }
        projected.ControlPoints.Add(point);
    }
    return projected;
}
inline FIntervalPoint LerpPoint(const FIntervalPoint& a,const FIntervalPoint& b,Interval t)
{
    FIntervalPoint point;const auto complement=AnalyticCertified::Subtract(Interval::Exact(1),t);
    for(int32 d=0;d<3;++d)point.Axis[d]=AnalyticCertified::Add(
        AnalyticCertified::Multiply(complement,a.Axis[d]),AnalyticCertified::Multiply(t,b.Axis[d]));
    return point;
}
inline bool EvaluateFixedUV(const FIntervalTensorSurface& surface,FVector2d uv,FIntervalPoint& output)
{
    output={};const auto tu=Interval::Exact(uv.X),tv=Interval::Exact(uv.Y);
    if(!AnalyticCertified::Valid(tu)||!AnalyticCertified::Valid(tv)||
        tu.Lower<0||tu.Upper>1||tv.Lower<0||tv.Upper>1||
        surface.DegreeU<0||surface.DegreeU>16||surface.DegreeV<0||surface.DegreeV>16||
        surface.ControlPoints.Num()!=(surface.DegreeU+1)*(surface.DegreeV+1))return false;
    TArray<FIntervalPoint,TInlineAllocator<16>> rows,work;
    rows.SetNum(surface.DegreeU+1);work.SetNum(surface.DegreeV+1);
    const int32 vc=surface.DegreeV+1;
    for(int32 u=0;u<=surface.DegreeU;++u)
    {
        for(int32 v=0;v<vc;++v)
        {work[v]=surface.ControlPoints[u*vc+v];if(!Detail::ValidPoint(work[v]))return false;}
        for(int32 count=vc-1;count>0;--count)for(int32 v=0;v<count;++v)
            work[v]=LerpPoint(work[v],work[v+1],tv);
        rows[u]=work[0];
    }
    for(int32 count=surface.DegreeU;count>0;--count)for(int32 u=0;u<count;++u)
        rows[u]=LerpPoint(rows[u],rows[u+1],tu);
    if(!Detail::ValidPoint(rows[0]))return false;output=rows[0];return true;
}
inline double ChooseU(const FIntervalTensorSurface& surface,const FIntervalPoint& center,int32 dropped=0)
{
    // A parameter proposal only, not a proof of affine X or exact extrusion.
    // Invalid arithmetic merely chooses the always-valid dyadic midpoint.
    const double a=surface.ControlPoints[0].Axis[dropped].Lower;
    const double b=surface.ControlPoints[surface.DegreeU*(surface.DegreeV+1)].Axis[dropped].Upper;
    const double x=.5*center.Axis[dropped].Lower+.5*center.Axis[dropped].Upper;
    const double span=b-a;
    if(!AnalyticCertified::Finite(x)||!AnalyticCertified::Finite(span)||span==0)return .5;
    const double u=(x-a)/span;
    if(!AnalyticCertified::Finite(u))return .5;
    const double selected=FMath::Clamp(u,0.,1.);
    const auto bounds=Interval::Exact(selected);
    return AnalyticCertified::Valid(bounds)&&bounds.Lower>=0&&bounds.Upper<=1?selected:.5;
}
}

namespace RetainedMinimumDetail
{
struct FLeafBounds
{
    double LowerSquared=0,UpperSquared=0;
    FIntervalPoint WitnessPoint;
    FVector2d WitnessUV=FVector2d::ZeroVector;
};
// Same arithmetic and order as the existing Detail::Bounds, without mutating
// shared UV geometry. The surface is bound by the owning retained operation.
inline bool Bounds(const FIntervalTensorSurface&,const Detail::FNode& n,
    const FIntervalPoint& center,FLeafBounds& output,const Interval* threshold=nullptr,Detail::FMinimumMidpointMemo::FCallScope* midpointMemo=nullptr,PlaneProposal95::FWork* proposal95=nullptr)
{
    using Detail::ValidPoint;using Detail::Midpoint;

    Detail::FThresholdQuickWitness quick;Detail::FSameCallQuickBox quickBox;
    const auto decision=Detail::TryThresholdCornerAndBox(n,center,threshold,quick,
        midpointMemo&&midpointMemo->MayReuseSameCallBox()?&quickBox:nullptr);
    if(decision==Detail::EThresholdQuickResult::Unknown)return false;
    if(decision==Detail::EThresholdQuickResult::Decided)
    {
        output.LowerSquared=quick.LowerSquared;output.UpperSquared=quick.UpperSquared;
        output.WitnessPoint=MoveTemp(quick.Point);output.WitnessUV=quick.UV;return true;
    }

    if (n.Net.IsEmpty() || !ValidPoint(center)) return false;
    double low[3],high[3];const bool reuseBox=quickBox.Ready;
    for (int32 d=0; d<3; ++d)
    { low[d]=reuseBox?quickBox.Low[d]:n.Net[0].Axis[d].Lower; high[d]=reuseBox?quickBox.High[d]:n.Net[0].Axis[d].Upper; }
    for (const auto& p : n.Net)
    {
        if (!ValidPoint(p)) return false;
        if(!reuseBox)for (int32 d=0; d<3; ++d)
        { low[d]=FMath::Min(low[d],p.Axis[d].Lower); high[d]=FMath::Max(high[d],p.Axis[d].Upper); }
    }
    if(midpointMemo)midpointMemo->ObserveOrdinaryBoxPass(uint64(n.Net.Num()),reuseBox);
    // Convex control hull is inside this box for the exact restricted polynomial.
    Interval lower=Interval::Exact(0),upper=Interval::Exact(0);
    output.WitnessPoint=midpointMemo?midpointMemo->Midpoint(n):Midpoint(n); if (!ValidPoint(output.WitnessPoint)) return false;
    for (int32 d=0; d<3; ++d)
    {
        Interval delta=Interval::Exact(0);
        if (center.Axis[d].Lower>high[d])
            delta=AnalyticCertified::Subtract(Interval::Exact(center.Axis[d].Lower),Interval::Exact(high[d]));
        else if (center.Axis[d].Upper<low[d])
            delta=AnalyticCertified::Subtract(Interval::Exact(low[d]),Interval::Exact(center.Axis[d].Upper));
        lower=AnalyticCertified::Add(lower,AnalyticCertified::Square(delta));
        const auto difference=AnalyticCertified::Subtract(center.Axis[d],output.WitnessPoint.Axis[d]);
        upper=AnalyticCertified::Add(upper,AnalyticCertified::Square(difference));
    }
    if (!AnalyticCertified::Valid(lower) || !AnalyticCertified::Valid(upper)) return false;
    output.LowerSquared=FMath::Max(0.,lower.Lower); output.UpperSquared=upper.Upper;
    output.WitnessUV=FVector2d(0.5*(n.MinU+n.MaxU),0.5*(n.MinV+n.MaxV));
    // Corners are exact parameters of the restricted polynomial, including
    // collapsed original edges. A midpoint-only witness can waste the budget
    // refining infinitely many equivalent parameters along such an edge.
    // Each candidate upper bound is universal over the SAME coefficient boxes.
    const int32 vc=n.DegreeV+1;
    for(int32 i=0;i<2;++i)for(int32 j=0;j<2;++j)
    {
        const auto& point=n.Net[(i?n.DegreeU:0)*vc+(j?n.DegreeV:0)];
        auto cornerUpper=Interval::Exact(0);
        for(int32 d=0;d<3;++d)
            cornerUpper=AnalyticCertified::Add(cornerUpper,AnalyticCertified::Square(AnalyticCertified::Subtract(center.Axis[d],point.Axis[d])));
        if(!AnalyticCertified::Valid(cornerUpper))return false;
        if(cornerUpper.Upper<output.UpperSquared)
        {
            output.UpperSquared=cornerUpper.Upper;output.WitnessPoint=point;
            output.WitnessUV=FVector2d(i?n.MaxU:n.MinU,j?n.MaxV:n.MinV);
        }
    }
    double directionalLower=0;
    if(Detail::ProposeDirectionalHullLower(n,center,output.WitnessPoint,directionalLower)==Detail::EDirectionalHullLower::Unknown)return false;
    output.LowerSquared=FMath::Max(output.LowerSquared,directionalLower);
    if(!Detail::ImprovePlaneProposal95(n,center,output.LowerSquared,output.UpperSquared,output.WitnessUV,output.WitnessPoint,threshold,proposal95))return false;
    return output.LowerSquared<=output.UpperSquared;

}
}

// Shared exported one-shot minimum. Defaults occur once on this declaration.
// Retained states and convenience overloads call this same implementation.
IAMSPEED_API FTensorDistanceBounds BoundTensorDistanceSquaredIntervalCenter(
    const FIntervalTensorSurface& surface,const FIntervalPoint& centerBounds,FDistanceBudget& budget,
    double requestedSquaredGap,int32 maximumNodesThisCall=MAX_int32,
    const Interval* decisionThresholdSquared=nullptr,Detail::FMinimumMidpointMemo* midpointMemo=nullptr);

struct FRetainedTensorMinimumCounters
{
    uint64 NewRoots=0,ReusedLeaves=0,DirtyBounds=0,KKTExcluded=0,UVSplits=0,LocalStops=0;
    int32 PeakHeapLeaves=0;
};
enum class ERetainedTensorMinimumFailure:uint8 {None,Input,Numeric,Work,Arithmetic,Scope};
// DRAFT ONLY. Inside Certified namespace in TensorDistanceBounds.h before retained class.
namespace PlanarFamilyTimeShortcutControl { IAMSPEED_API bool RuntimeEnabled(); }
namespace StrongConvexCurveMinimum
{
enum class EResult:uint8 {NotApplicable,Bounded,LocalLimit,Work,Arithmetic};
struct FInspection
{
    double CurvatureLower=0,ParameterLower=0,ParameterUpper=1;
    int32 NewtonSteps=0;
    bool PossibleStart=false,PossibleEnd=false;
};
IAMSPEED_API EResult TryBound(const FIntervalTensorSurface& surface,const FIntervalPoint& center,
    FDistanceBudget& budget,int32 maximumWork,FTensorDistanceBounds& out,FInspection* inspect=nullptr);
IAMSPEED_API EResult TryBoundForDecision(const FIntervalTensorSurface& surface,const FIntervalPoint& center,
    FDistanceBudget& budget,int32 maximumWork,FTensorDistanceBounds& out,const Interval* threshold,
    double requestedGap,FInspection* inspect=nullptr);
}

class FRetainedTensorMinimum final
{
    struct FGeometricMiddle
    {
        FIntervalPoint Point;
        uint64 Control=0;
        uint32 Thread=0,RequiredSticky=0;
        bool Known=false;
    };
    struct FGeometryCell final
    {
        const Detail::FNode Node;
        const FGeometricMiddle Middle;
        FGeometryCell(Detail::FNode&& node,const FGeometricMiddle& middle)
            :Node(MoveTemp(node)),Middle(middle){}
    };
public:
    class FWholeGeometryHullMemo
    {
        using FHulls=MinimumDetail::FMinimumGeometryHulls;
        struct FEntry
        {
            const FIntervalTensorSurface* Surface=nullptr;
            TSharedPtr<const FGeometryCell> Cell;
            FHulls Bounds;
            uint32 RequiredSticky=0;
        };
        static constexpr uint32 Capacity=64;
        FEntry Entries[Capacity];
        const uint32 Thread=FPlatformTLS::GetCurrentThreadId();
        const uint64 Control;
        uint64 Lookups=0,Hits=0,Stores=0;
        static bool SameDomain(const Detail::FNode& a,const Detail::FNode& b)
        {
            using ::AnalyticCertified::Bits;
            return a.DegreeU==b.DegreeU&&a.DegreeV==b.DegreeV&&a.DepthU==b.DepthU&&a.DepthV==b.DepthV&&
                Bits(a.MinU)==Bits(b.MinU)&&Bits(a.MaxU)==Bits(b.MaxU)&&
                Bits(a.MinV)==Bits(b.MinV)&&Bits(a.MaxV)==Bits(b.MaxV)&&a.Net.Num()==b.Net.Num();
        }
        static bool SameNet(const Detail::FNode& a,const Detail::FNode& b)
        {
            using ::AnalyticCertified::Bits;
            for(int32 i=0;i<a.Net.Num();++i)
                for(int32 d=0;d<3;++d)
                {
                    const auto& x=a.Net[i].Axis[d];const auto& y=b.Net[i].Axis[d];
                    if(x.Known!=y.Known||Bits(x.Lower)!=Bits(y.Lower)||Bits(x.Upper)!=Bits(y.Upper))return false;
                }
            return true;
        }
        bool CurrentSticky(uint32 required,uint32* observed=nullptr) const
        {
#if defined(_MSC_VER) && defined(_M_X64) && !defined(_M_ARM64EC) && PLATFORM_ENABLE_VECTORINTRINSICS
            const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
            if(numeric.Status!=EAnalyticNumericStatus::Complete||numeric.ControlFingerprint()!=Control||
                (numeric.VectorControl&required)!=required)return false;
            if(observed)*observed=numeric.VectorControl&0x3f;
            return true;
#else
            return false;
#endif
        }
    public:
        explicit FWholeGeometryHullMemo(uint64 control):Control(control){}
        uint64 GetLookups() const{return Lookups;}
        uint64 GetHits() const{return Hits;}
        uint64 GetStores() const{return Stores;}
        bool Read(const FIntervalTensorSurface& surface,const TSharedPtr<const FGeometryCell>& cell,
            const Detail::FNode& n,FHulls& bounds)
        {
            const auto original=[&]{return MinimumDetail::ReadMinimumGeometryHulls(n,bounds);};
            if(n.DegreeU!=5||n.DegreeV!=5||n.Net.Num()!=36||!cell||
                Thread!=FPlatformTLS::GetCurrentThreadId())return original();
#if defined(_MSC_VER) && defined(_M_X64) && !defined(_M_ARM64EC) && PLATFORM_ENABLE_VECTORINTRINSICS
            using ::AnalyticCertified::Bits;
            uint32 hash=PointerHash(&surface);
            for(double t:{n.MinU,n.MaxU,n.MinV,n.MaxV})
                hash=HashCombineFast(hash,GetTypeHash(Bits(t)));
            auto& entry=Entries[hash&(Capacity-1)];
            ++Lookups;
            if(entry.Surface==&surface&&entry.Cell&&
                (entry.Cell==cell||(SameDomain(entry.Cell->Node,n)&&SameNet(entry.Cell->Node,n)))&&
                CurrentSticky(entry.RequiredSticky))
            {
                bounds=entry.Bounds;++Hits;
                return true;
            }
            if(!original())return false;
            uint32 sticky=0;
            if(CurrentSticky(0,&sticky))
            {
                entry={&surface,cell,bounds,sticky};++Stores;
            }
            return true;
#else
            return original();
#endif
        }
    };
private:
    struct FLeaf
    {
        TSharedPtr<const FGeometryCell> Geometry;
        double LowerSquared=0;
        bool Dirty=false;
    };
    const FIntervalTensorSurface* Surface=nullptr;
    FDistanceBudget* Budget=nullptr;
    Detail::FMinimumMidpointMemo* MidpointMemo=nullptr;
    FWholeGeometryHullMemo* WholeGeometryHullMemo=nullptr;
    FIntervalPoint Center;
    double RequestedGap=0,ProjectedLower=0;
    bool ConvexAttempted=false;
    Interval Threshold;
    bool HasThreshold=false,Initialized=false,Terminal=false,LocalStopRecorded=false;
    int32 LocalLimit=0;
    uint64 NumericControl=0,LocalSpent=0,LastUsed=0;
    int32 LastRemaining=0;
    TArray<FLeaf> Leaves;
    Detail::FMinLowerLeafHeap Minimum;
    FTensorDistanceBounds Result;
    FRetainedTensorMinimumCounters Counters;
    ERetainedTensorMinimumFailure Failure=ERetainedTensorMinimumFailure::None;

    bool BoundsForCell(const Detail::FNode& n,const FIntervalPoint& center,
        RetainedMinimumDetail::FLeafBounds& output,const Interval* threshold,
        Detail::FMinimumMidpointMemo::FCallScope* midpointMemo,
        const FGeometricMiddle* stored,FGeometricMiddle* captured,PlaneProposal95::FWork* proposal95)
    {
        // Default-null is the actual unchanged arithmetic oracle. Nothing is
        // sampled or inspected, and no snapshot is published, on this route.
        if(!MidpointMemo)return RetainedMinimumDetail::Bounds(*Surface,n,center,output,threshold,midpointMemo,proposal95);
        using Detail::ValidPoint;using Detail::Midpoint;

        Detail::FThresholdQuickWitness quick;Detail::FSameCallQuickBox quickBox;
        const auto decision=Detail::TryThresholdCornerAndBox(n,center,threshold,quick,
            midpointMemo&&midpointMemo->MayReuseSameCallBox()?&quickBox:nullptr);
        if(decision==Detail::EThresholdQuickResult::Unknown)return false;
        if(decision==Detail::EThresholdQuickResult::Decided)
        {
            output.LowerSquared=quick.LowerSquared;output.UpperSquared=quick.UpperSquared;
            output.WitnessPoint=MoveTemp(quick.Point);output.WitnessUV=quick.UV;return true;
        }

        if (n.Net.IsEmpty() || !ValidPoint(center)) return false;
        double low[3],high[3];const bool reuseBox=quickBox.Ready;
        for (int32 d=0; d<3; ++d)
        { low[d]=reuseBox?quickBox.Low[d]:n.Net[0].Axis[d].Lower; high[d]=reuseBox?quickBox.High[d]:n.Net[0].Axis[d].Upper; }
        for (const auto& p : n.Net)
        {
            if (!ValidPoint(p)) return false;
            if(!reuseBox)for (int32 d=0; d<3; ++d)
            { low[d]=FMath::Min(low[d],p.Axis[d].Lower); high[d]=FMath::Max(high[d],p.Axis[d].Upper); }
        }
        if(midpointMemo)midpointMemo->ObserveOrdinaryBoxPass(uint64(n.Net.Num()),reuseBox);
        // Convex control hull is inside this box for the exact restricted polynomial.
        Interval lower=Interval::Exact(0),upper=Interval::Exact(0);
        // Only the old true-geometric Midpoint site can create this value. A
        // threshold/corner witness never reaches here and cannot populate Middle.
        const bool retainedHit=stored&&stored->Known&&stored->Control==NumericControl&&
            stored->Thread==FPlatformTLS::GetCurrentThreadId()&&midpointMemo&&
            midpointMemo->HasRetainedMiddleContext(stored->RequiredSticky);
        output.WitnessPoint=retainedHit?stored->Point:(midpointMemo?midpointMemo->Midpoint(n):Midpoint(n));
        if (!ValidPoint(output.WitnessPoint)) return false;
        uint32 requiredSticky=0;
        const bool publish=!retainedHit&&captured&&midpointMemo&&
            midpointMemo->HasRetainedMiddleContext(0,&requiredSticky);
        if(publish)
        {
            captured->Point=output.WitnessPoint;captured->Control=NumericControl;
            captured->Thread=FPlatformTLS::GetCurrentThreadId();captured->RequiredSticky=requiredSticky;captured->Known=true;
        }
        if(midpointMemo)midpointMemo->ObserveRetainedMiddle(retainedHit,publish);
        for (int32 d=0; d<3; ++d)
        {
            Interval delta=Interval::Exact(0);
            if (center.Axis[d].Lower>high[d])
                delta=AnalyticCertified::Subtract(Interval::Exact(center.Axis[d].Lower),Interval::Exact(high[d]));
            else if (center.Axis[d].Upper<low[d])
                delta=AnalyticCertified::Subtract(Interval::Exact(low[d]),Interval::Exact(center.Axis[d].Upper));
            lower=AnalyticCertified::Add(lower,AnalyticCertified::Square(delta));
            const auto difference=AnalyticCertified::Subtract(center.Axis[d],output.WitnessPoint.Axis[d]);
            upper=AnalyticCertified::Add(upper,AnalyticCertified::Square(difference));
        }
        if (!AnalyticCertified::Valid(lower) || !AnalyticCertified::Valid(upper)) return false;
        output.LowerSquared=FMath::Max(0.,lower.Lower); output.UpperSquared=upper.Upper;
        output.WitnessUV=FVector2d(0.5*(n.MinU+n.MaxU),0.5*(n.MinV+n.MaxV));
        // Corners are exact parameters of the restricted polynomial, including
        // collapsed original edges. A midpoint-only witness can waste the budget
        // refining infinitely many equivalent parameters along such an edge.
        // Each candidate upper bound is universal over the SAME coefficient boxes.
        const int32 vc=n.DegreeV+1;
        for(int32 i=0;i<2;++i)for(int32 j=0;j<2;++j)
        {
            const auto& point=n.Net[(i?n.DegreeU:0)*vc+(j?n.DegreeV:0)];
            auto cornerUpper=Interval::Exact(0);
            for(int32 d=0;d<3;++d)
                cornerUpper=AnalyticCertified::Add(cornerUpper,AnalyticCertified::Square(AnalyticCertified::Subtract(center.Axis[d],point.Axis[d])));
            if(!AnalyticCertified::Valid(cornerUpper))return false;
            if(cornerUpper.Upper<output.UpperSquared)
            {
                output.UpperSquared=cornerUpper.Upper;output.WitnessPoint=point;
                output.WitnessUV=FVector2d(i?n.MaxU:n.MinU,j?n.MaxV:n.MinV);
            }
        }
        double directionalLower=0;
        if(Detail::ProposeDirectionalHullLower(n,center,output.WitnessPoint,directionalLower)==Detail::EDirectionalHullLower::Unknown)return false;
        output.LowerSquared=FMath::Max(output.LowerSquared,directionalLower);
        if(!Detail::ImprovePlaneProposal95(n,center,output.LowerSquared,output.UpperSquared,output.WitnessUV,output.WitnessPoint,threshold,proposal95))return false;
        return output.LowerSquared<=output.UpperSquared;

    }

    bool Context()
    {
        if(Terminal)return false;
        const auto context=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(context.Status!=EAnalyticNumericStatus::Complete||context.ControlFingerprint()!=NumericControl)
        {Refuse(ERetainedTensorMinimumFailure::Numeric);return false;}
        if(!Budget||Budget->Used<LastUsed||Budget->Remaining>LastRemaining||Budget->ConsumptionFloor<0)
        {Refuse(ERetainedTensorMinimumFailure::Work);return false;}
        return true;
    }
    FTensorDistanceBounds Refuse(ERetainedTensorMinimumFailure why)
    {Failure=why;Terminal=true;Result={};return Result;}
    void Improve(const RetainedMinimumDetail::FLeafBounds& bound)
    {
        if(bound.UpperSquared<Result.UpperSquared)
        {Result.UpperSquared=bound.UpperSquared;Result.SampleUV=bound.WitnessUV;Result.SamplePoint=bound.WitnessPoint;}
    }
    FTensorDistanceBounds Finish()
    {return Context()?Result:FTensorDistanceBounds();}
    FRetainedTensorMinimum()=default;
public:
    FRetainedTensorMinimum(const FIntervalTensorSurface& surface,const FIntervalPoint& center,
        FDistanceBudget& budget,double gap,int32 localLimit,const Interval* threshold=nullptr,Detail::FMinimumMidpointMemo* midpointMemo=nullptr)
        :Surface(&surface),Budget(&budget),MidpointMemo(midpointMemo),Center(center),RequestedGap(gap),
        HasThreshold(threshold!=nullptr),LocalLimit(localLimit),LastUsed(budget.Used),LastRemaining(budget.Remaining)
    {
        const auto context=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(context.Status!=EAnalyticNumericStatus::Complete){Refuse(ERetainedTensorMinimumFailure::Numeric);return;}
        NumericControl=context.ControlFingerprint();if(threshold)Threshold=*threshold;
        if(localLimit<=0||surface.DegreeU<0||surface.DegreeU>16||surface.DegreeV<0||surface.DegreeV>16||
            surface.ControlPoints.Num()!=(surface.DegreeU+1)*(surface.DegreeV+1)||!Detail::ValidPoint(center)||
            !AnalyticCertified::Finite(gap)||((AnalyticCertified::Bits(gap)&AnalyticCertified::Sign)&&
            (AnalyticCertified::Bits(gap)&AnalyticCertified::Magnitude))||
            (HasThreshold&&!AnalyticCertified::Valid(Threshold)))Refuse(ERetainedTensorMinimumFailure::Input);
    }
    void BindWholeGeometryHullMemoBeforeStart(FWholeGeometryHullMemo* memo)
    {if(!Initialized&&!Terminal)WholeGeometryHullMemo=memo;}
    // Surface, Budget and optional memos are borrowed for this operation and all descendants.
    // Keep the immutable, nonrelocating surface, live global account and owner-thread
    // memo alive until every child has been destroyed.
    FRetainedTensorMinimum(FIntervalTensorSurface&&,const FIntervalPoint&,
        FDistanceBudget&,double,int32,const Interval* =nullptr,Detail::FMinimumMidpointMemo* =nullptr)=delete;
    FRetainedTensorMinimum(const FRetainedTensorMinimum&)=delete;
    FRetainedTensorMinimum& operator=(const FRetainedTensorMinimum&)=delete;
    const FIntervalPoint& GetCoveredCenter() const{return Center;}
    const FRetainedTensorMinimumCounters& GetCountersForInspection() const{return Counters;}
    ERetainedTensorMinimumFailure GetFailureForInspection() const{return Failure;}
    uint64 GetLocalSpentForInspection() const{return LocalSpent;}
    bool IsTerminalForInspection() const{return Terminal;}

    // Frozen parents are read-only. Their geometry, heap, metadata, result,
    // debit observations and terminal status never change while a child is
    // materialized. The shared live budget is still debited for every copy.
    ERetainedTensorMinimumFailure InspectSnapshotContext() const
    {
        if(Terminal)return Failure==ERetainedTensorMinimumFailure::None?ERetainedTensorMinimumFailure::Input:Failure;
        const auto context=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(context.Status!=EAnalyticNumericStatus::Complete||context.ControlFingerprint()!=NumericControl)
            return ERetainedTensorMinimumFailure::Numeric;
        if(!Budget||Budget->Used<LastUsed||Budget->Remaining>LastRemaining||Budget->ConsumptionFloor<0)
            return ERetainedTensorMinimumFailure::Work;
        return ERetainedTensorMinimumFailure::None;
    }
    bool IsKnownSnapshot() const
    {return InspectSnapshotContext()==ERetainedTensorMinimumFailure::None&&Initialized&&Result.Status!=EDistanceBoundStatus::Unknown;}

    // A child pays metadata/index records before copying. Equal center scopes
    // retain the original local debit. A failed materialization supplies no
    // child and no cached result; its caller must terminate that time search.
    TUniquePtr<FRetainedTensorMinimum> ForkContainedImmutable(const FIntervalPoint& center,ERetainedTensorMinimumFailure& why) const
    {
        why=InspectSnapshotContext();if(why!=ERetainedTensorMinimumFailure::None)return nullptr;
        if(!Initialized||!Detail::ValidPoint(center))
        {why=ERetainedTensorMinimumFailure::Input;return nullptr;}
        bool strict=false;
        for(int32 d=0;d<3;++d)
        {
            if(center.Axis[d].Lower<Center.Axis[d].Lower||center.Axis[d].Upper>Center.Axis[d].Upper)
            {why=ERetainedTensorMinimumFailure::Scope;return nullptr;}
            strict|=center.Axis[d].Lower>Center.Axis[d].Lower||center.Axis[d].Upper<Center.Axis[d].Upper;
        }
        const uint64 initialSpent=strict?0:LocalSpent;
        if(initialSpent>uint64(LocalLimit)||uint64(Leaves.Num())>uint64(LocalLimit)-initialSpent)
        {why=ERetainedTensorMinimumFailure::Work;return nullptr;}
        const uint64 before=Budget->Used;
        for(int32 i=0;i<Leaves.Num();++i)
            if(!Budget->Consume()){why=ERetainedTensorMinimumFailure::Work;return nullptr;}
        const uint64 copied=Budget->Used-before;
        const uint64 spent=strict?copied:LocalSpent+copied;
        if(spent>uint64(LocalLimit)){why=ERetainedTensorMinimumFailure::Work;return nullptr;}
        auto child=TUniquePtr<FRetainedTensorMinimum>(new FRetainedTensorMinimum());
        child->Surface=Surface;child->Budget=Budget;child->MidpointMemo=MidpointMemo;child->Center=center;
        child->WholeGeometryHullMemo=WholeGeometryHullMemo;child->RequestedGap=RequestedGap;
        child->ProjectedLower=ProjectedLower;child->Threshold=Threshold;child->HasThreshold=HasThreshold;
        child->ConvexAttempted=strict?false:ConvexAttempted;
        child->Initialized=true;child->LocalLimit=LocalLimit;child->LocalSpent=spent;
        child->NumericControl=NumericControl;child->LastUsed=Budget->Used;child->LastRemaining=Budget->Remaining;
        child->Leaves=Leaves;child->Minimum=Minimum;child->Result=Result;
        child->Counters.ReusedLeaves=uint64(Minimum.Num());child->Counters.PeakHeapLeaves=Minimum.Num();
        child->LocalStopRecorded=strict?false:LocalStopRecorded;
        if(strict)for(auto& leaf:child->Leaves)if(leaf.Geometry)leaf.Dirty=true;
        // Parent upper/witness and projected lower remain valid for the subset.
        // A paid original-net UV evaluation can only improve the matching upper.
        if(strict&&child->LocalSpent<uint64(LocalLimit))
        {
            if(!Budget->Consume()){why=ERetainedTensorMinimumFailure::Work;return nullptr;}
            ++child->LocalSpent;FIntervalPoint point;
            if(!ProjectedDistanceDetail::EvaluateFixedUV(*Surface,Result.SampleUV,point))
            {why=ERetainedTensorMinimumFailure::Arithmetic;return nullptr;}
            auto upper=Interval::Exact(0);
            for(int32 d=0;d<3;++d)upper=AnalyticCertified::Add(upper,
                AnalyticCertified::Square(AnalyticCertified::Subtract(center.Axis[d],point.Axis[d])));
            if(!AnalyticCertified::Valid(upper)){why=ERetainedTensorMinimumFailure::Arithmetic;return nullptr;}
            if(upper.Upper<child->Result.UpperSquared)
            {child->Result.UpperSquared=upper.Upper;child->Result.SamplePoint=point;}
        }
        child->LastUsed=Budget->Used;child->LastRemaining=Budget->Remaining;
        why=InspectSnapshotContext();
        if(why!=ERetainedTensorMinimumFailure::None)return nullptr;
        if(!child->Context()){why=child->Failure;return nullptr;}
        return child;
    }

    // Mutable API preserves its original terminal-failure semantics for
    // direct callers. The lazy time frontier uses only the const API above.
    TUniquePtr<FRetainedTensorMinimum> ForkContained(const FIntervalPoint& center)
    {
        ERetainedTensorMinimumFailure why=ERetainedTensorMinimumFailure::None;
        auto child=ForkContainedImmutable(center,why);
        if(!child){Refuse(why);return nullptr;}
        LastUsed=Budget->Used;LastRemaining=Budget->Remaining;
        if(!Context())return nullptr;
        return child;
    }

    FTensorDistanceBounds Advance()
    {
        if(!Context())return {};
        Detail::FMinimumMidpointMemo::FCallScope midpointScope(MidpointMemo,*Surface,NumericControl);
        const uint64 before=Budget->Used;
        struct FSpentScope
        {
            FRetainedTensorMinimum& Self;uint64 Before;
            ~FSpentScope()
            {
                Self.LocalSpent+=Self.Budget->Used-Before;
                Self.LastUsed=Self.Budget->Used;Self.LastRemaining=Self.Budget->Remaining;
            }
        } spentScope{*this,before};
        const auto spent=[&](){return LocalSpent+(Budget->Used-before);};
        const auto available=[&]()->int32
        {
            const uint64 used=spent();
            if(used>=uint64(LocalLimit)||Budget->ConsumptionFloor<0||Budget->Remaining<=Budget->ConsumptionFloor)return 0;
            return int32(FMath::Min(uint64(LocalLimit)-used,uint64(int64(Budget->Remaining)-int64(Budget->ConsumptionFloor))));
        };
        PlaneProposal95::FWork proposal95{Surface,Budget,before,int32(uint64(LocalLimit)>LocalSpent?uint64(LocalLimit)-LocalSpent:0),RequestedGap};
        if(!Initialized)
        {
            if(!Budget->Consume())return Refuse(ERetainedTensorMinimumFailure::Work);
            Detail::FNode first;first.DegreeU=Surface->DegreeU;first.DegreeV=Surface->DegreeV;
            for(const auto& point:Surface->ControlPoints)first.Net.Add(point);
            RetainedMinimumDetail::FLeafBounds bound;FGeometricMiddle middle;
            if(!BoundsForCell(first,Center,bound,HasThreshold?&Threshold:nullptr,&midpointScope,nullptr,&middle,&proposal95))return Refuse(proposal95.WorkFailure?ERetainedTensorMinimumFailure::Work:ERetainedTensorMinimumFailure::Arithmetic);
            FLeaf leaf;leaf.Geometry=MakeShared<FGeometryCell>(MoveTemp(first),middle);leaf.LowerSquared=bound.LowerSquared;
            Leaves.Add(MoveTemp(leaf));Minimum.PushIndex(0,Leaves);Initialized=true;
            Result.UpperSquared=bound.UpperSquared;Result.SampleUV=bound.WitnessUV;Result.SamplePoint=bound.WitnessPoint;
            ++Counters.NewRoots;Counters.PeakHeapLeaves=1;
            const auto gap=AnalyticCertified::Subtract(Interval::Exact(Result.UpperSquared),Interval::Exact(bound.LowerSquared));
            if(!AnalyticCertified::Valid(gap))return Refuse(ERetainedTensorMinimumFailure::Arithmetic);
            const bool decided=HasThreshold&&(bound.LowerSquared>Threshold.Upper||Result.UpperSquared<Threshold.Lower);
            const bool needs=!decided&&!(gap.Upper<=RequestedGap&&!HasThreshold);
            if(needs&&Surface->DegreeU==5&&Surface->DegreeV==5&&available()>=3)
            {
                if(!Budget->Consume())return Refuse(ERetainedTensorMinimumFailure::Work);
                const int32 dropped=ProjectedDistanceDetail::ExtrusionAxis(*Surface);
                if(dropped!=INDEX_NONE)
                {
                    const auto projected=ProjectedDistanceDetail::ProjectYZ(*Surface,dropped);
                    const auto center=ProjectedDistanceDetail::ProjectCenterToYZ(Center,dropped);
                    const int32 cap=FMath::Min(256,available()-1);
                    if(cap<=0)return Refuse(ERetainedTensorMinimumFailure::Work);
                    const auto lower=BoundTensorDistanceSquaredIntervalCenter(projected,center,*Budget,RequestedGap,cap,HasThreshold?&Threshold:nullptr,MidpointMemo);
                    if(lower.Status==EDistanceBoundStatus::Unknown)return Refuse(ERetainedTensorMinimumFailure::Arithmetic);
                    ProjectedLower=lower.LowerSquared;
                    if(available()<1||!Budget->Consume())return Refuse(ERetainedTensorMinimumFailure::Work);
                    const FVector2d uv(ProjectedDistanceDetail::ChooseU(*Surface,Center,dropped),lower.SampleUV.Y);FIntervalPoint point;
                    if(!ProjectedDistanceDetail::EvaluateFixedUV(*Surface,uv,point))return Refuse(ERetainedTensorMinimumFailure::Arithmetic);
                    auto upper=Interval::Exact(0);
                    for(int32 d=0;d<3;++d)upper=AnalyticCertified::Add(upper,
                        AnalyticCertified::Square(AnalyticCertified::Subtract(Center.Axis[d],point.Axis[d])));
                    if(!AnalyticCertified::Valid(upper))return Refuse(ERetainedTensorMinimumFailure::Arithmetic);
                    if(upper.Upper<Result.UpperSquared){Result.UpperSquared=upper.Upper;Result.SampleUV=uv;Result.SamplePoint=point;}
                }
            }
        }
        const bool thresholdAlreadyDecided=HasThreshold&&Minimum.Num()>0&&
            (FMath::Max(ProjectedLower,Leaves[Minimum.PeekIndex()].LowerSquared)>Threshold.Upper||
             Result.UpperSquared<Threshold.Lower);
        if(!thresholdAlreadyDecided&&!ConvexAttempted&&Surface->DegreeU==0&&Surface->DegreeV==5&&
            available()>=36&&PlanarFamilyTimeShortcutControl::RuntimeEnabled())
        {
            ConvexAttempted=true;FTensorDistanceBounds convex;
            const auto status=StrongConvexCurveMinimum::TryBoundForDecision(*Surface,Center,*Budget,available(),convex,
                HasThreshold?&Threshold:nullptr,RequestedGap);
            if(status==StrongConvexCurveMinimum::EResult::Work)return Refuse(ERetainedTensorMinimumFailure::Work);
            if(status==StrongConvexCurveMinimum::EResult::Arithmetic)return Refuse(ERetainedTensorMinimumFailure::Arithmetic);
            if(status==StrongConvexCurveMinimum::EResult::Bounded)
            {
                ProjectedLower=FMath::Max(ProjectedLower,convex.LowerSquared);
                if(convex.UpperSquared<Result.UpperSquared)
                {Result.UpperSquared=convex.UpperSquared;Result.SampleUV=convex.SampleUV;Result.SamplePoint=convex.SamplePoint;}
            }
        }
        for(;;)
        {
            if(Minimum.Num()==0)return Refuse(ERetainedTensorMinimumFailure::Arithmetic);
            const int32 best=Minimum.PeekIndex();
            Result.LowerSquared=FMath::Max(ProjectedLower,Leaves[best].LowerSquared);
            if(Result.LowerSquared>Result.UpperSquared)return Refuse(ERetainedTensorMinimumFailure::Arithmetic);
            Result.Status=EDistanceBoundStatus::Bounded;
            const auto gap=AnalyticCertified::Subtract(Interval::Exact(Result.UpperSquared),Interval::Exact(Result.LowerSquared));
            if(!AnalyticCertified::Valid(gap))return Refuse(ERetainedTensorMinimumFailure::Arithmetic);
            const bool decided=HasThreshold&&(Result.LowerSquared>Threshold.Upper||Result.UpperSquared<Threshold.Lower);
            if(gap.Upper<=RequestedGap&&(!HasThreshold||decided)){Result.Status=EDistanceBoundStatus::Converged;return Finish();}
            if(decided)return Finish();
            if(spent()+1>uint64(LocalLimit))
            {if(!LocalStopRecorded){++Counters.LocalStops;LocalStopRecorded=true;}return Finish();}
            if(!Budget->Consume())return Refuse(ERetainedTensorMinimumFailure::Work);
            if(!Leaves[best].Geometry)return Refuse(ERetainedTensorMinimumFailure::Arithmetic);
            if(Leaves[best].Dirty)
            {
                RetainedMinimumDetail::FLeafBounds fresh;
                if(!BoundsForCell(Leaves[best].Geometry->Node,Center,fresh,HasThreshold?&Threshold:nullptr,&midpointScope,&Leaves[best].Geometry->Middle,nullptr,&proposal95))return Refuse(proposal95.WorkFailure?ERetainedTensorMinimumFailure::Work:ERetainedTensorMinimumFailure::Arithmetic);
                Minimum.PopRoot(Leaves);
                Leaves[best].LowerSquared=FMath::Max(Leaves[best].LowerSquared,fresh.LowerSquared);Leaves[best].Dirty=false;
                Improve(fresh);Minimum.PushIndex(best,Leaves);++Counters.DirtyBounds;
                continue;
            }
            const auto& cell=Leaves[best].Geometry;
            const auto classification=WholeGeometryHullMemo?
                MinimumDetail::ClassifyMinimumWithGeometry(cell->Node,Center,[&](const auto& n,auto& hulls){
                    return WholeGeometryHullMemo->Read(*Surface,cell,n,hulls);
                }):MinimumDetail::ClassifyMinimum(cell->Node,Center);
            if(classification==MinimumDetail::ECellMinimum::Unknown)return Refuse(ERetainedTensorMinimumFailure::Arithmetic);
            if(classification==MinimumDetail::ECellMinimum::Excluded)
            {
                Minimum.PopRoot(Leaves);Leaves[best].Geometry.Reset();++Counters.KKTExcluded;
                if(Minimum.Num()==0)return Refuse(ERetainedTensorMinimumFailure::Arithmetic);
                continue;
            }
            if(Budget->Remaining<2||spent()+2>uint64(LocalLimit))
            {if(!LocalStopRecorded){++Counters.LocalStops;LocalStopRecorded=true;}return Finish();}
            const auto& geometry=Leaves[best].Geometry->Node;
            if(geometry.DegreeU==0&&geometry.DegreeV==0)return Finish();
            const bool alongU=Detail::ChooseTensorDistanceSplitU(geometry);Detail::FNode left,right;
            if(!Detail::Split(geometry,alongU,left,right))return Finish();
            RetainedMinimumDetail::FLeafBounds a,b;FGeometricMiddle leftMiddle,rightMiddle;
            if(!Budget->Consume()||!Budget->Consume())return Refuse(ERetainedTensorMinimumFailure::Work);
            if(!BoundsForCell(left,Center,a,HasThreshold?&Threshold:nullptr,&midpointScope,nullptr,&leftMiddle,&proposal95)||
                !BoundsForCell(right,Center,b,HasThreshold?&Threshold:nullptr,&midpointScope,nullptr,&rightMiddle,&proposal95))return Refuse(proposal95.WorkFailure?ERetainedTensorMinimumFailure::Work:ERetainedTensorMinimumFailure::Arithmetic);
            Minimum.PopRoot(Leaves);Improve(a);Improve(b);
            FLeaf first,second;first.Geometry=MakeShared<FGeometryCell>(MoveTemp(left),leftMiddle);first.LowerSquared=a.LowerSquared;
            second.Geometry=MakeShared<FGeometryCell>(MoveTemp(right),rightMiddle);second.LowerSquared=b.LowerSquared;
            Leaves[best]=MoveTemp(first);Leaves.Add(MoveTemp(second));
            Minimum.PushIndex(best,Leaves);Minimum.PushIndex(Leaves.Num()-1,Leaves);++Counters.UVSplits;
            Counters.PeakHeapLeaves=FMath::Max(Counters.PeakHeapLeaves,Minimum.Num());
        }
    }
};

// Diagnostic bounds on every fixed full finite polynomial represented by the net.
// Caller supplies squared spatial gap and a budget shared by all providers.
// No hit, normal, TOI, MTD, eligibility, or proof-ticket conversion is provided.
// Implementation is in the existing PlanarFamilyTimeShortcut.cpp TU.

inline FTensorDistanceBounds BoundTensorDistanceSquaredIntervalCenter(const FTensorBezierSurface& surface,
    const FIntervalPoint& centerBounds,FDistanceBudget& budget,double requestedSquaredGap,
    int32 maximumNodesThisCall=MAX_int32)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(budget.Remaining<=0||maximumNodesThisCall<=0)return {};
    FIntervalTensorSurface net;if(!Detail::PromoteTensor(surface,net))return {};
    return BoundTensorDistanceSquaredIntervalCenter(net,centerBounds,budget,requestedSquaredGap,maximumNodesThisCall);
}
inline FTensorDistanceBounds BoundTensorDistanceSquared(const FIntervalTensorSurface& surface,
    const FVector3d& center,FDistanceBudget& budget,double requestedSquaredGap)
{
    return BoundTensorDistanceSquaredIntervalCenter(surface,Detail::Point(center),budget,requestedSquaredGap);
}
inline FTensorDistanceBounds BoundTensorDistanceSquared(const FTensorBezierSurface& surface,
    const FVector3d& center, FDistanceBudget& budget, double requestedSquaredGap)
{
    // Exact() performs only bit classification before the mandatory core guard.
    return BoundTensorDistanceSquaredIntervalCenter(surface,Detail::Point(center),budget,requestedSquaredGap);
}

}
