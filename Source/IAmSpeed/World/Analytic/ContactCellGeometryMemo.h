#pragma once
#include "TensorDistanceBounds.h"
#include "HAL/PlatformTLS.h"
#include <atomic>

namespace Speed::Analytic::Certified::ContactDetail
{
using MinimumDetail::NetHull;
namespace ContactCellRootLruControl
{
IAMSPEED_API bool RuntimeEnabled();
IAMSPEED_API void RecordBoundaryEviction(uint32 roots,uint32 causes,uint64 beforeCells,uint64 afterCells,
    uint64 beforePoints,uint64 afterPoints,bool publicationFits);
}
// Construct a net of dS/dU or dS/dV in ORIGINAL parameters, before any
// position-net restriction. Its subsequent restrictions never differentiate
// almost equal, independently widened position coefficients.
// Caller pays one budget unit for this bounded whole-net operation.
inline bool OriginalDerivativeNet(const Detail::FNode& root,bool alongU,Detail::FNode& output)
{
    output={};
    if(root.DepthU!=0||root.DepthV!=0||root.MinU!=0||root.MaxU!=1||root.MinV!=0||root.MaxV!=1||
        root.DegreeU<0||root.DegreeU>16||root.DegreeV<0||root.DegreeV>16||
        root.Net.Num()!=(root.DegreeU+1)*(root.DegreeV+1))return false;
    for(const auto& point:root.Net)if(!Detail::ValidPoint(point))return false;
    const int32 degree=alongU?root.DegreeU:root.DegreeV;
    Detail::FNode derivative;derivative.DegreeU=alongU?FMath::Max(0,root.DegreeU-1):root.DegreeU;
    derivative.DegreeV=alongU?root.DegreeV:FMath::Max(0,root.DegreeV-1);
    const int32 vc=root.DegreeV+1;
    for(int32 u=0;u<=derivative.DegreeU;++u)for(int32 v=0;v<=derivative.DegreeV;++v)
    {
        auto point=Detail::Point(FVector3d::ZeroVector);
        if(degree)
        {
            const int32 a=u*vc+v,b=alongU?(u+1)*vc+v:u*vc+v+1;
            for(int32 d=0;d<3;++d)point.Axis[d]=AnalyticCertified::Multiply(Interval::Exact(double(degree)),
                AnalyticCertified::Subtract(root.Net[b].Axis[d],root.Net[a].Axis[d]));
        }
        if(!Detail::ValidPoint(point))return false;
        derivative.Net.Add(point);
    }
    output=MoveTemp(derivative);return true;
}


struct FCurveSupportBand {FVector3d Direction=FVector3d::ZeroVector;Interval Beta,NormSquared;};
struct FCurveSupportBands {FCurveSupportBand Values[2];int32 ZeroAxis=INDEX_NONE;};
// A direction is merely a finite proposal. The band is proved from ALL the
// restricted position controls. No path, distance, radius or hit is cached.
inline bool BuildCurveSupportBands(const Detail::FNode& node,int32 zero,FCurveSupportBands& output)
{
    namespace A=::AnalyticCertified;output={};
    if(node.DegreeU!=0||node.DegreeV<0||node.DegreeV>16||node.Net.Num()!=node.DegreeV+1)return false;
    for(const auto& p:node.Net)if(!Detail::ValidPoint(p))return false;
    // The projection axis belongs to the root geometry. Interval restriction
    // widens even exact-zero controls; no child-zero premise is needed for an
    // orthogonal projection to remain a lower bound on full 3D distance.
    if(zero<0||zero>=3)return false;
    const int32 x=(zero+1)%3,y=(zero+2)%3;
    const auto dx=A::Subtract(node.Net.Last().Axis[x],node.Net[0].Axis[x]);
    const auto dy=A::Subtract(node.Net.Last().Axis[y],node.Net[0].Axis[y]);
    if(!A::Valid(dx)||!A::Valid(dy))return false;
    FCurveSupportBands result;result.ZeroAxis=zero;
    if((A::Bits(dx.Lower)&A::Magnitude)==0&&(A::Bits(dy.Lower)&A::Magnitude)==0)
    {result.Values[0].Direction[x]=1.;result.Values[1].Direction[y]=1.;}
    else
    {
        result.Values[0].Direction[x]=dy.Lower;result.Values[0].Direction[y]=-dx.Lower;
        result.Values[1].Direction[x]=dx.Lower;result.Values[1].Direction[y]=dy.Lower;
    }
    for(auto& band:result.Values)
    {
        band.NormSquared=A::Add(A::Square(Interval::Exact(band.Direction[x])),A::Square(Interval::Exact(band.Direction[y])));
        if(!A::Valid(band.NormSquared)||!(band.NormSquared.Upper>0))return false;
        bool have=false;
        for(const auto& p:node.Net)
        {
            const auto projection=A::Add(A::Multiply(Interval::Exact(band.Direction[x]),p.Axis[x]),
                A::Multiply(Interval::Exact(band.Direction[y]),p.Axis[y]));
            if(!A::Valid(projection))return false;
            if(!have){band.Beta=projection;have=true;}
            else {band.Beta.Lower=FMath::Min(band.Beta.Lower,projection.Lower);band.Beta.Upper=FMath::Max(band.Beta.Upper,projection.Upper);}
        }
        if(!have||!A::Valid(band.Beta))return false;
    }
    output=result;return true;
}

// Contact geometry only. No distance, KKT, center, time, radial normal, width,
// hit, provider policy, work account or authority is retained in this object.
// All publication occurs at the original arithmetic sites after old debits.
class FContactCellGeometryMemo final
{
public:
    struct FCounters
    {
        uint64 Requests=0,Hits=0,Builds=0,RootHits=0,HullHits=0,MiddleHits=0,
            DirectionHits=0,ChildrenHits=0,CellsCreated=0,GeometryEvictions=0;
    };
    enum class ENet:uint8 {Position,Du,Dv};
    class FCell;
    using FCellPtr=TSharedPtr<FCell,ESPMode::ThreadSafe>;
private:
    // Each live cell retains its own lifetime marker. Directory eviction may
    // drop roots while test/reentrant handles remain; those cells still count.
    struct FStorage final
    {
        std::atomic<uint64> LiveCells{0},LivePoints{0};
    };
    using FStoragePtr=TSharedPtr<FStorage,ESPMode::ThreadSafe>;
private:
    struct FPointValue {FIntervalPoint Value;uint32 Sticky=0;bool Ready=false;};
public:
    class FCell final
    {
        friend class FContactCellGeometryMemo;
        const FStoragePtr Storage;
        Detail::FNode Nets[3];uint64 Control=0,PointCharge=0;
        FPointValue Hulls[3],Middle;
        bool Direction=false,DirectionReady=false,ChildrenReady=false,ChildrenAlongU=false;
        uint32 DirectionSticky=0,ChildrenSticky=0;
        FCellPtr Left,Right;
        FCurveSupportBands CurveBands;uint32 CurveBandsSticky=0;bool CurveBandsReady=false;
        void Charge()
        {
            PointCharge=uint64(Nets[0].Net.Num())+uint64(Nets[1].Net.Num())+uint64(Nets[2].Net.Num());
            Storage->LiveCells.fetch_add(1);Storage->LivePoints.fetch_add(PointCharge);
        }
        FCell(FStoragePtr storage,const Detail::FNode& p,const Detail::FNode& du,const Detail::FNode& dv,uint64 control)
            :Storage(MoveTemp(storage)),Nets{p,du,dv},Control(control){Charge();}
        FCell(FStoragePtr storage,Detail::FNode&& p,Detail::FNode&& du,Detail::FNode&& dv,uint64 control)
            :Storage(MoveTemp(storage)),Nets{MoveTemp(p),MoveTemp(du),MoveTemp(dv)},Control(control){Charge();}
        FCell(const FCell&)=delete;FCell& operator=(const FCell&)=delete;
        FCell(FCell&&)=delete;FCell& operator=(FCell&&)=delete;
    public:
        ~FCell(){Storage->LiveCells.fetch_sub(1);Storage->LivePoints.fetch_sub(PointCharge);}
        const Detail::FNode& Get(ENet kind) const {return Nets[uint8(kind)];}
    };
private:
    // Memory policy only. Reaching it disables additional memo publication;
    // the original arithmetic still runs with the SAME collision work account.
    static constexpr uint64 MaximumCells=65536,MaximumPoints=1048576;
    static constexpr int32 MaximumRoots=64;
    const uint32 OwnerThread=FPlatformTLS::GetCurrentThreadId();
    TArray<FCellPtr> Roots;
    const FStoragePtr Storage=MakeShared<FStorage,ESPMode::ThreadSafe>();
    uint32 ActiveCalls=0;bool Saturated=false,RootLimitReached=false;
    uint64 PendingCells=0,PendingPoints=0;
    FCounters Counters;
    static void Count(uint64& value){if(value<MAX_uint64)++value;}
    static bool GeometryOnly(const Detail::FNode& node)
    {
        // Contact creates these fields with their defaults and never computes
        // distance bounds/witnesses inside a FWork node. Decline a foreign
        // non-default node rather than retain any query-dependent metadata.
        if(AnalyticCertified::Bits(node.LowerSquared)!=0||AnalyticCertified::Bits(node.UpperSquared)!=0||
            AnalyticCertified::Bits(node.WitnessUV.X)!=0||AnalyticCertified::Bits(node.WitnessUV.Y)!=0)return false;
        for(const auto& axis:node.WitnessPoint.Axis)
            if(axis.Known||AnalyticCertified::Bits(axis.Lower)!=0||AnalyticCertified::Bits(axis.Upper)!=0)return false;
        return true;
    }
    static bool SamePoint(const FIntervalPoint& a,const FIntervalPoint& b)
    {
        for(int32 d=0;d<3;++d)if(a.Axis[d].Known!=b.Axis[d].Known||
            AnalyticCertified::Bits(a.Axis[d].Lower)!=AnalyticCertified::Bits(b.Axis[d].Lower)||
            AnalyticCertified::Bits(a.Axis[d].Upper)!=AnalyticCertified::Bits(b.Axis[d].Upper))return false;
        return true;
    }
    static bool SameNode(const Detail::FNode& a,const Detail::FNode& b)
    {
        if(a.DegreeU!=b.DegreeU||a.DegreeV!=b.DegreeV||a.DepthU!=b.DepthU||a.DepthV!=b.DepthV||
            a.Net.Num()!=b.Net.Num()||AnalyticCertified::Bits(a.MinU)!=AnalyticCertified::Bits(b.MinU)||
            AnalyticCertified::Bits(a.MaxU)!=AnalyticCertified::Bits(b.MaxU)||
            AnalyticCertified::Bits(a.MinV)!=AnalyticCertified::Bits(b.MinV)||
            AnalyticCertified::Bits(a.MaxV)!=AnalyticCertified::Bits(b.MaxV)||
            AnalyticCertified::Bits(a.LowerSquared)!=AnalyticCertified::Bits(b.LowerSquared)||
            AnalyticCertified::Bits(a.UpperSquared)!=AnalyticCertified::Bits(b.UpperSquared)||
            AnalyticCertified::Bits(a.WitnessUV.X)!=AnalyticCertified::Bits(b.WitnessUV.X)||
            AnalyticCertified::Bits(a.WitnessUV.Y)!=AnalyticCertified::Bits(b.WitnessUV.Y)||
            !SamePoint(a.WitnessPoint,b.WitnessPoint))return false;
        for(int32 i=0;i<a.Net.Num();++i)if(!SamePoint(a.Net[i],b.Net[i]))return false;
        return true;
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
    static uint64 Points(const Detail::FNode& p,const Detail::FNode& du,const Detail::FNode& dv)
    {return uint64(p.Net.Num())+uint64(du.Net.Num())+uint64(dv.Net.Num());}
    bool Fits(uint64 cells,uint64 points) const
    {
        const uint64 liveCells=Storage->LiveCells.load(),livePoints=Storage->LivePoints.load();
        return liveCells<=MaximumCells&&cells<=MaximumCells-liveCells&&
            livePoints<=MaximumPoints&&points<=MaximumPoints-livePoints;
    }
    void RememberPressure(uint64 cells,uint64 points,bool rootLimit=false)
    {
        Saturated=true;RootLimitReached|=rootLimit;
        PendingCells=FMath::Max(PendingCells,cells);PendingPoints=FMath::Max(PendingPoints,points);
    }
    bool CanPublish(uint64 cells,uint64 points)
    {
        const bool fits=Fits(cells,points);if(!fits)RememberPressure(cells,points);return fits;
    }
    void PrepareOuterCall(bool lru)
    {
        if(ActiveCalls!=0||!Saturated)return;
        const uint64 beforeCells=Storage->LiveCells.load(),beforePoints=Storage->LivePoints.load();
        const uint32 causes=(RootLimitReached?1u:0u)|(!Fits(PendingCells,PendingPoints)?2u:0u);
        uint32 removed=0;
        if(!lru)
        {
            removed=uint32(Roots.Num());Roots.Empty();Count(Counters.GeometryEvictions);
        }
        else
        {
            // Directory order is oldest to newest. A dropped root may remain
            // pinned outside the directory: its complete subtree stays charged
            // by FStorage, so no memory cap is discounted or widened.
            while(!Roots.IsEmpty()&&
                ((RootLimitReached&&Roots.Num()>=MaximumRoots)||!Fits(PendingCells,PendingPoints)))
            {
                Roots.RemoveAt(0,1,EAllowShrinking::No);++removed;
            }
            if(removed)Count(Counters.GeometryEvictions);
            ContactCellRootLruControl::RecordBoundaryEviction(removed,causes,beforeCells,Storage->LiveCells.load(),
                beforePoints,Storage->LivePoints.load(),
                (!RootLimitReached||Roots.Num()<MaximumRoots)&&Fits(PendingCells,PendingPoints));
        }
        // A later failed publication records fresh pressure. This never
        // changes live accounting, retained handles, or collision work.
        Saturated=false;RootLimitReached=false;PendingCells=PendingPoints=0;
    }
    FCellPtr Root(uint64 control,const Detail::FNode& p,const Detail::FNode& du,const Detail::FNode& dv,bool lru)
    {
        if(!Sticky(control,0)||!GeometryOnly(p)||!GeometryOnly(du)||!GeometryOnly(dv))return {};
        for(int32 i=Roots.Num()-1;i>=0;--i)
        {
            const auto& cell=Roots[i];
            if(cell->Control==control&&SameNode(cell->Nets[0],p)&&SameNode(cell->Nets[1],du)&&SameNode(cell->Nets[2],dv))
            {
                Count(Counters.RootHits);FCellPtr result=cell;
                if(lru&&i+1<Roots.Num()){Roots.RemoveAt(i,1,EAllowShrinking::No);Roots.Add(result);}
                return result;
            }
        }
        const uint64 points=Points(p,du,dv);
        if(Roots.Num()>=MaximumRoots){RememberPressure(1,points,true);return {};}
        if(!CanPublish(1,points))return {};
        FCellPtr cell(new FCell(Storage,p,du,dv,control));Roots.Add(cell);Count(Counters.CellsCreated);return cell;
    }
public:
    FContactCellGeometryMemo()=default;
    FContactCellGeometryMemo(const FContactCellGeometryMemo&)=delete;
    FContactCellGeometryMemo& operator=(const FContactCellGeometryMemo&)=delete;
    const FCounters& GetCountersForInspection() const{return Counters;}
    uint64 GetLiveCellsForInspection() const{return Storage->LiveCells.load();}
    uint64 GetLivePointsForInspection() const{return Storage->LivePoints.load();}
    int32 GetDirectoryRootsForInspection() const{return Roots.Num();}
    class FCallScope final
    {
        FContactCellGeometryMemo* Owner=nullptr;uint64 Control=0;bool RootLru=false;
        bool Compatible(const FCellPtr& cell) const
        {return Owner&&cell&&cell->Storage==Owner->Storage&&cell->Control==Control;}
    public:
        FCallScope(FContactCellGeometryMemo* owner,uint64 control):Control(control)
        {
            if(!owner||owner->OwnerThread!=FPlatformTLS::GetCurrentThreadId()||!Sticky(control,0))return;
            RootLru=ContactCellRootLruControl::RuntimeEnabled();
            owner->PrepareOuterCall(RootLru);
            if(owner->ActiveCalls==MAX_uint32)return;Owner=owner;++Owner->ActiveCalls;
        }
        ~FCallScope(){if(Owner)--Owner->ActiveCalls;}
        FCallScope(const FCallScope&)=delete;FCallScope& operator=(const FCallScope&)=delete;
        FCellPtr FindRoot(const Detail::FNode& p,const Detail::FNode& du,const Detail::FNode& dv)
        {return Owner?Owner->Root(Control,p,du,dv,RootLru):FCellPtr{};}
        bool Hull(const FCellPtr& cell,ENet kind,const Detail::FNode& node,FIntervalPoint& output)
        {
            const bool compatible=Compatible(cell) && &node==&cell->Get(kind);
            if(Owner)Count(Owner->Counters.Requests);
            if(compatible)
            {
                const auto& saved=cell->Hulls[uint8(kind)];
                if(saved.Ready&&Sticky(Control,saved.Sticky))
                {output=saved.Value;Count(Owner->Counters.Hits);Count(Owner->Counters.HullHits);return true;}
            }
            if(!NetHull(node,output))return false;
            uint32 flags=0;if(compatible&&Sticky(Control,0,&flags))
            {auto& saved=cell->Hulls[uint8(kind)];saved.Value=output;saved.Sticky=flags;saved.Ready=true;Count(Owner->Counters.Builds);}
            return true;
        }
        FIntervalPoint Midpoint(const FCellPtr& cell,const Detail::FNode& node)
        {
            const bool compatible=Compatible(cell) && &node==&cell->Get(ENet::Position);
            if(Owner)Count(Owner->Counters.Requests);
            if(compatible&&cell->Middle.Ready&&Sticky(Control,cell->Middle.Sticky))
            {Count(Owner->Counters.Hits);Count(Owner->Counters.MiddleHits);return cell->Middle.Value;}
            const auto result=Detail::Midpoint(node);uint32 flags=0;
            if(compatible&&Detail::ValidPoint(result)&&Sticky(Control,0,&flags))
            {cell->Middle.Value=result;cell->Middle.Sticky=flags;cell->Middle.Ready=true;Count(Owner->Counters.Builds);}
            return result;
        }
        bool Direction(const FCellPtr& cell,const Detail::FNode& node)
        {
            const bool compatible=Compatible(cell) && &node==&cell->Get(ENet::Position);
            if(Owner)Count(Owner->Counters.Requests);
            if(compatible&&cell->DirectionReady&&Sticky(Control,cell->DirectionSticky))
            {Count(Owner->Counters.Hits);Count(Owner->Counters.DirectionHits);return cell->Direction;}
            const bool result=Detail::ChooseTensorDistanceSplitU(node);uint32 flags=0;
            if(compatible&&Sticky(Control,0,&flags))
            {cell->Direction=result;cell->DirectionSticky=flags;cell->DirectionReady=true;Count(Owner->Counters.Builds);}
            return result;
        }

        // Caller pays the same bounded geometric operation on cold/warm paths.
        bool CurveSupportBands(const FCellPtr& cell,const Detail::FNode& node,int32 zero,FCurveSupportBands& output)
        {
            const bool compatible=Compatible(cell)&& &node==&cell->Get(ENet::Position);
            if(compatible&&cell->CurveBandsReady&&cell->CurveBands.ZeroAxis==zero&&Sticky(Control,cell->CurveBandsSticky))
            {output=cell->CurveBands;return true;}
            if(!BuildCurveSupportBands(node,zero,output))return false;
            uint32 flags=0;if(compatible&&Sticky(Control,0,&flags))
            {cell->CurveBands=output;cell->CurveBandsSticky=flags;cell->CurveBandsReady=true;}
            return true;
        }

        bool Children(const FCellPtr& cell,bool alongU,FCellPtr& left,FCellPtr& right)
        {
            left.Reset();right.Reset();if(Owner)Count(Owner->Counters.Requests);
            if(!Compatible(cell)||!cell->ChildrenReady||cell->ChildrenAlongU!=alongU||!Sticky(Control,cell->ChildrenSticky))return false;
            left=cell->Left;right=cell->Right;Count(Owner->Counters.Hits);Count(Owner->Counters.ChildrenHits);return true;
        }
        // This is the sole child-construction entry point. No caller can
        // inject precomputed nets into a cache used by a physical query.
        bool SplitChildren(const FCellPtr& cell,bool alongU,
            const Detail::FNode& p,const Detail::FNode& du,const Detail::FNode& dv,
            Detail::FNode& lp,Detail::FNode& ld,Detail::FNode& lv,
            Detail::FNode& rp,Detail::FNode& rd,Detail::FNode& rv,FCellPtr& left,FCellPtr& right)
        {
            const bool compatible=Compatible(cell)&& &p==&cell->Get(ENet::Position)&&
                &du==&cell->Get(ENet::Du)&& &dv==&cell->Get(ENet::Dv);
            left.Reset();right.Reset();
            if(compatible&&Children(cell,alongU,left,right))return true;
            if(!Detail::Split(p,alongU,lp,rp)||!Detail::Split(du,alongU,ld,rd)||
                !Detail::Split(dv,alongU,lv,rv))return false;
            if(compatible)PublishChildren(cell,alongU,MoveTemp(lp),MoveTemp(ld),MoveTemp(lv),
                MoveTemp(rp),MoveTemp(rd),MoveTemp(rv),left,right);
            return true;
        }
    private:
        void PublishChildren(const FCellPtr& cell,bool alongU,
            Detail::FNode&& lp,Detail::FNode&& ld,Detail::FNode&& lv,
            Detail::FNode&& rp,Detail::FNode&& rd,Detail::FNode&& rv,FCellPtr& left,FCellPtr& right)
        {
            left.Reset();right.Reset();uint32 flags=0;
            if(!Compatible(cell)||!Sticky(Control,0,&flags))return;
            if(!GeometryOnly(lp)||!GeometryOnly(ld)||!GeometryOnly(lv)||!GeometryOnly(rp)||!GeometryOnly(rd)||!GeometryOnly(rv))return;
            const uint64 points=Points(lp,ld,lv)+Points(rp,rd,rv);if(!Owner->CanPublish(2,points))return;
            FCellPtr l(new FCell(Owner->Storage,MoveTemp(lp),MoveTemp(ld),MoveTemp(lv),Control));
            FCellPtr r(new FCell(Owner->Storage,MoveTemp(rp),MoveTemp(rd),MoveTemp(rv),Control));
            cell->Left=l;cell->Right=r;cell->ChildrenAlongU=alongU;cell->ChildrenSticky=flags;cell->ChildrenReady=true;
            Count(Owner->Counters.CellsCreated);Count(Owner->Counters.CellsCreated);Count(Owner->Counters.Builds);
            left=MoveTemp(l);right=MoveTemp(r);
        }
    };
};
inline FContactCellGeometryMemo& ThreadContactCellGeometryMemo()
{
    static thread_local TUniquePtr<FContactCellGeometryMemo> memo;
    if(!memo)memo=MakeUnique<FContactCellGeometryMemo>();return *memo;
}

}
