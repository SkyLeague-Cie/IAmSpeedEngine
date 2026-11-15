#pragma once
#include "NativeSolidSweepVolume.h"
#include "BoxPoseCorridorBounds.h"
#include "YawFloorBoxFirstArrival.h"
#include "NativeGroupedBoxVolumeBounds.h"
namespace Speed::Analytic::Certified
{
struct FNativeBoxCachedPlaneValues
{
    FVector Point=FVector::ZeroVector,Normal=FVector::ZeroVector;
    uint64 SourceId=0,SurfaceId=0,FeatureId=0,PrimitiveId=0,CanonicalGroupId=0;uint32 MaterialId=0;
};
enum class EBoxFloorSupportMotion:uint8 {Unknown,Fixed,Yaw};
// Whole-H geometric bounds ONLY. Masked corners retain their measured bands;
// this is no tangency, nonpenetration, force, projection or event capability.
struct FNativeBoxFloorSupportBounds
{
    EBoxFloorSupportMotion Motion=EBoxFloorSupportMotion::Unknown;
    FIntervalPoint QueryEnvelope;Interval NativeCornerGap[8],ReferenceCornerGap[8];
    FVector SideNormal=FVector::ZeroVector;FNativeBoxCachedPlaneValues CachedPlane;
    bool Complete=false;
};
inline bool BuildNativeBoxFloorSupportBounds(const PreparedExecution::FNativeSolidStaticValues& v,FNativeBoxFloorSupportBounds& output)
{
    output={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(v.Shape!=PreparedExecution::ENativeSolidQueryShape::Box)return false;
    FNativeBoxFloorSupportBounds result;FFloorNativeSolidModel fixed;FYawFloorBoxMotionModel yaw;
    if(BuildNativeSolidSweepVolume(v,fixed,result.QueryEnvelope)&&fixed.IsFixedBox())result.Motion=EBoxFloorSupportMotion::Fixed;
    else
    {
        FYawFloorBoxSweepBounds wide;
        if(!BuildYawFloorBoxSweepBounds(v,wide)||!BuildYawFloorBoxMotionModel(v,yaw))return false;
        result.QueryEnvelope=wide.QueryEnvelope;result.Motion=EBoxFloorSupportMotion::Yaw;
    }
    result.SideNormal=FVector(0,0,v.BodyState.Location.Z<0?-1.:1.);
    for(int32 k=0;k<8;++k)
    {
        result.NativeCornerGap[k]=result.Motion==EBoxFloorSupportMotion::Fixed?
            fixed.BoundPhysicalCornerGap(k,0,v.Horizon,true):yaw.BoundPhysicalCornerGap(k,0,v.Horizon,true);
        result.ReferenceCornerGap[k]=result.Motion==EBoxFloorSupportMotion::Fixed?
            fixed.BoundPhysicalCornerGap(k,0,v.Horizon,false):yaw.BoundPhysicalCornerGap(k,0,v.Horizon,false);
        if(!AnalyticCertified::Valid(result.NativeCornerGap[k])||!AnalyticCertified::Valid(result.ReferenceCornerGap[k]))return false;
    }
    if(!Detail::ValidPoint(result.QueryEnvelope))return false;result.Complete=true;output=MoveTemp(result);return true;
}
// Separate GEOMETRY descriptor for the actual cached 3D refiner. This is
// neither the Fixed/Yaw support status nor a NoCorner/clear/advance result.
class FGeneralBoxPhysicalFloorClearBounds;
struct FNativeGeneralBoxFloorDescriptorBounds
{
    TSharedPtr<const FGeneralBoxPhysicalFloorClearBounds,ESPMode::ThreadSafe> PhysicalClear;
    FGeneralBoxMotionInput GetMotionInputForInspection()const
    {
        FGeneralBoxMotionInput input;input.Owner=Captured.OwnerState;input.LocalCOM=Captured.LocalCOM;input.LocalOffset=Captured.LocalOffset;
        input.LocalRotation=Captured.LocalRotation;input.PhysicalHalfExtent=Captured.PhysicalHalfExtent;
        input.QueryHalfExtent=Captured.QueryHalfExtent;input.Horizon=Captured.Horizon;return input;
    }
    FIntervalPoint QueryEnvelope;
    FGeneralBoxMotionBounds OriginalMotion,AtZero;
    // Original complete finite partition for the observation only. A single
    // nonflat candidate requires a separate graph proof after native NoCorner.
    FReferenceBoundsCandidateSetPtr ObservationFinitePartition;
    int32 ObservationCurvedProvider=INDEX_NONE;
    FIntervalPoint ObservationInitialEnvelope;
    PreparedExecution::FNativeSolidStaticValues Captured;
    FNativeBoxCachedPlaneValues CachedPlane;
    FVector SideNormal=FVector::ZeroVector;
    uint64 NumericFingerprint=0;bool Complete=false;
    const void* ObservationRequestIdentity=nullptr;
    uint64 ObservationWorkUsed=0;int32 ObservationWorkLimit=0,ObservationWorkRemaining=0,ObservationConsumptionFloor=0;
};
inline bool NativeBoxNeedsGeneralFloorDescriptor(const PreparedExecution::FNativeSolidStaticValues& v)
{
    // Exclusive applicability before any Fixed/Yaw interval arithmetic. Both
    // old builders reject these raw nonzero angular/nonyaw operands. A later
    // failure of the chosen builder remains terminal, never a fallback.
    if(v.Shape!=PreparedExecution::ENativeSolidQueryShape::Box)return false;
    const auto nonzero=[](double x){return (AnalyticCertified::Bits(x)&AnalyticCertified::Magnitude)!=0;};
    bool rotating=false;for(int32 d=0;d<3;++d)
        rotating|=nonzero(v.OwnerState.AngularVelocity[d])||nonzero(v.OwnerState.AngularAcceleration[d]);
    const bool nonyaw=nonzero(v.OwnerState.Rotation.X)||nonzero(v.OwnerState.Rotation.Y)||
        nonzero(v.OwnerState.AngularVelocity.X)||nonzero(v.OwnerState.AngularVelocity.Y)||
        nonzero(v.OwnerState.AngularAcceleration.X)||nonzero(v.OwnerState.AngularAcceleration.Y);
    return rotating&&nonyaw;
}
inline bool BuildNativeGeneralBoxFloorDescriptorBounds(const PreparedExecution::FNativeSolidStaticValues& v,
    FDistanceBudget& budget,FNativeGeneralBoxFloorDescriptorBounds& output)
{
    output={};const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!NativeBoxNeedsGeneralFloorDescriptor(v))return false;
    FNativeGeneralBoxFloorDescriptorBounds result;
    // Covers original World chord/query/SAT/physical footprints and shells.
    // Against FLOOR these are only finite-domain coverage, not separation.
    if(!budget.Consume()||!BuildGeneralNativeBoxSweepVolume(v,result.QueryEnvelope))return false;
    FGeneralBoxMotionInput input;input.Owner=v.OwnerState;input.LocalCOM=v.LocalCOM;input.LocalOffset=v.LocalOffset;
    input.LocalRotation=v.LocalRotation;input.PhysicalHalfExtent=v.PhysicalHalfExtent;
    input.QueryHalfExtent=v.QueryHalfExtent;input.Horizon=v.Horizon;
    if(!NativeGroupedBoxDetail::ContractedGlobalMotionPiece(input,0.f,v.Horizon,budget,result.OriginalMotion)||
        !budget.Consume()||!BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(input,0.f,0.f,result.AtZero))return false;
    if(result.OriginalMotion.Corners.Num()!=32||result.AtZero.Corners.Num()!=32||
        !Detail::ValidPoint(result.OriginalMotion.Hull)||!Detail::ValidPoint(result.AtZero.Hull))return false;
    result.SideNormal=FVector(0,0,v.BodyState.Location.Z<0?-1.:1.);
    result.Captured=v;result.NumericFingerprint=numeric.ControlFingerprint();result.Complete=true;
    if(RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=result.NumericFingerprint)return false;
    output=MoveTemp(result);return true;
}
inline bool UnmaskedBoxCornersStayClear(const FNativeBoxFloorSupportBounds& bounds,uint8 actualMask)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!bounds.Complete||bounds.Motion==EBoxFloorSupportMotion::Unknown||!actualMask)return false;
    for(int32 k=0;k<8;++k)
    {
        if(!AnalyticCertified::Valid(bounds.NativeCornerGap[k])||!AnalyticCertified::Valid(bounds.ReferenceCornerGap[k]))return false;
        if(!(actualMask&(1u<<k))&&(bounds.NativeCornerGap[k].Lower<=0||bounds.ReferenceCornerGap[k].Lower<=0))return false;
    }
    // Native SupportGap=min(0,masked gaps)<=0. Subtracting it cannot create
    // a new zero/inside gap from these strictly positive unmasked raw gaps.
    return true;
}
}

// UNINSTALLED / UNCOMPILED. Pure mathematical candidate only.
// No real refiner result, mask, roundoff policy, support, contact or transport
// capability follows. Non-FLOOR / Landscape and intrinsic call bindings remain
// separate obligations. Include current NativeFloorBoxSupportBounds.h first.
namespace Speed::Analytic::Certified
{
namespace GeneralPhysicalFloorClearDetail
{
using namespace AnalyticCertified;
using namespace GeneralBoxDetail;
struct FCell {uint32 First=0,Last=0;};
inline bool CardinalFloor(const FVector& point,const FVector& normal)
{
    for(int32 d=0;d<3;++d)if(!FMath::IsFinite(point[d])||!FMath::IsFinite(normal[d]))return false;
    return !(Bits(point.Z)&Magnitude)&&!(Bits(normal.X)&Magnitude)&&!(Bits(normal.Y)&Magnitude)&&
        (Bits(normal.Z)&Magnitude)==Bits(1.);
}
inline bool StrictPositive(Interval value)
{
    if(!Valid(value))return false;
    const uint64 bits=Bits(value.Lower),magnitude=bits&Magnitude;
    return !(bits&Sign)&&magnitude>=MinNormal&&magnitude<Infinity;
}
inline bool RawGap(const FIntervalPoint& vertex,const FVector& point,const FVector& normal,Interval& output)
{
    output={};if(!Detail::ValidPoint(vertex))return false;
    FNode terms[3];
    for(int32 d=0;d<3;++d)
        terms[d]=Product(Difference(FNode{vertex.Axis[d],0},Leaf(point[d])),Leaf(normal[d]));
    Interval gap;
    // Keep the finite full XYZ subtraction/dot DAG. A coordinate-axis normal
    // never hides an overflowing subtraction followed by Inf*0.
    for(int32 first=0;first<3;++first)
    {
        const auto sum=Sum(Sum(terms[first],terms[(first+1)%3]),terms[(first+2)%3]);
        // Includes the native final rounded '-0' ONLY as a geometric premise.
        // The actual refiner still has to observe EstablishedMask==0 itself.
        const auto value=Expanded(Difference(sum,Leaf(0.)));
        if(!Valid(value))return false;gap=Valid(gap)?GeneralBoxDetail::Hull(gap,value):value;
    }
    if(!Valid(gap))return false;output=gap;return true;
}
inline bool SameInput(const FGeneralBoxMotionInput& a,const FGeneralBoxMotionInput& b)
{
    using namespace FloorNativeSweepDetail;
    return KinematicCenterDetail::Bits(a.Horizon)==KinematicCenterDetail::Bits(b.Horizon)&&
        SameVector(a.Owner.Location,b.Owner.Location)&&SameVector(a.Owner.Velocity,b.Owner.Velocity)&&
        SameVector(a.Owner.Acceleration,b.Owner.Acceleration)&&SameQuat(a.Owner.Rotation,b.Owner.Rotation)&&
        SameVector(a.Owner.AngularVelocity,b.Owner.AngularVelocity)&&SameVector(a.Owner.AngularAcceleration,b.Owner.AngularAcceleration)&&
        SameVector(a.LocalCOM,b.LocalCOM)&&SameVector(a.LocalOffset,b.LocalOffset)&&SameQuat(a.LocalRotation,b.LocalRotation)&&
        SameVector(a.PhysicalHalfExtent,b.PhysicalHalfExtent)&&SameVector(a.QueryHalfExtent,b.QueryHalfExtent);
}
}
class FGeneralBoxPhysicalFloorClearBounds;
using FGeneralBoxPhysicalFloorClearBoundsPtr=TSharedPtr<const FGeneralBoxPhysicalFloorClearBounds,ESPMode::ThreadSafe>;
namespace CorrelatedAxialBoxFloorProof
{
struct FWorldYawGapDiagnostic;
inline bool BoundWorldYaw(const FGeneralBoxMotionInput&,float,FDistanceBudget&,Interval (&)[8],FWorldYawGapDiagnostic*);
}
inline bool TryProveGeneralBoxPhysicalFloorClear(const FGeneralBoxMotionInput&,const FVector&,const FVector&,
    FDistanceBudget&,FGeneralBoxPhysicalFloorClearBoundsPtr&,bool allowWorldYaw=false);
class FGeneralBoxPhysicalFloorClearBounds final
{
    friend bool TryProveGeneralBoxPhysicalFloorClear(const FGeneralBoxMotionInput&,const FVector&,const FVector&,
        FDistanceBudget&,FGeneralBoxPhysicalFloorClearBoundsPtr&,bool);
    FGeneralBoxMotionInput Captured;
    FVector PlanePoint=FVector::ZeroVector,PlaneNormal=FVector::ZeroVector;
    FIntervalPoint PhysicalEnvelope;
    Interval MinimumGap[16];
    TArray<GeneralPhysicalFloorClearDetail::FCell> ClosedCover;
    uint64 NumericFingerprint=0,VisitedCells=0;
    FGeneralBoxPhysicalFloorClearBounds()=default;
public:
    bool Matches(const FGeneralBoxMotionInput& input,const FVector& point,const FVector& normal)const
    {
        const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        return NumericFingerprint==numeric.ControlFingerprint()&&!ClosedCover.IsEmpty()&&
            GeneralPhysicalFloorClearDetail::SameInput(Captured,input)&&
            FloorNativeSweepDetail::SameVector(PlanePoint,point)&&FloorNativeSweepDetail::SameVector(PlaneNormal,normal);
    }
    const FGeneralBoxMotionInput& GetInputForInspection()const{return Captured;}
    const FIntervalPoint& GetPhysicalEnvelopeForInspection()const{return PhysicalEnvelope;}
    TConstArrayView<GeneralPhysicalFloorClearDetail::FCell> GetClosedCoverForInspection()const{return ClosedCover;}
    const Interval& GetMinimumGapForInspection(int32 index)const{check(index>=0&&index<16);return MinimumGap[index];}
    uint64 GetVisitedCellCountForInspection()const{return VisitedCells;}
};
inline bool TryProveGeneralBoxPhysicalFloorClear(const FGeneralBoxMotionInput& input,const FVector& point,const FVector& normal,
    FDistanceBudget& budget,FGeneralBoxPhysicalFloorClearBoundsPtr& output,bool allowWorldYaw)
{
    const auto keepPrior=output;output.Reset();
    const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    using namespace GeneralPhysicalFloorClearDetail;
    uint32 end=0;
    if(!CardinalFloor(point,normal)||!FloorNativeSweepDetail::Ordinal(input.Horizon,end)||!end)return false;
    // Keep the original input alive even if it aliases the prior output.
    TSharedPtr<FGeneralBoxPhysicalFloorClearBounds,ESPMode::ThreadSafe> result(new FGeneralBoxPhysicalFloorClearBounds);
    result->Captured=input;result->PlanePoint=point;result->PlaneNormal=normal;
    result->NumericFingerprint=numeric.ControlFingerprint();
    struct FPendingCell {FCell Cell;uint16 ClearCorners;};
    TArray<FPendingCell,TInlineAllocator<32>> stack;stack.Add(FPendingCell{{0,end},0});
    constexpr uint16 AllCorners=0xffff;
    bool haveEnvelope=false;
    Interval yawGaps[8];
    bool triedYaw=false,haveYaw=false;
    while(!stack.IsEmpty())
    {
        if(RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=result->NumericFingerprint||
            !budget.Consume())return false;
        const auto pending=stack.Pop(EAllowShrinking::No);++result->VisitedCells;
        const auto cell=pending.Cell;
        const float first=FloorNativeSweepDetail::Seconds(cell.First),last=FloorNativeSweepDetail::Seconds(cell.Last);
        FGeneralBoxMotionBounds bounds;
        // Every interval uses the original owner and global seconds.
        if(!BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(result->Captured,first,last,bounds)||bounds.Corners.Num()!=32)return false;
        Interval gaps[16];
        uint16 clear=pending.ClearCorners;
        for(int32 family=0;family<2;++family)for(int32 k=0;k<8;++k)
        {
            const int32 index=family*8+k,role=family*16+k;
            const uint16 bit=uint16(1u<<index);
            if(clear&bit)continue;
            if(!budget.Consume()||!RawGap(bounds.Corners[role],result->PlanePoint,result->PlaneNormal,gaps[index]))return false;
            if(allowWorldYaw&&result->PlaneNormal.Z==1.&&!StrictPositive(gaps[index]))
            {
                if(!triedYaw)
                {
                    triedYaw=true;
                    haveYaw=CorrelatedAxialBoxFloorProof::BoundWorldYaw(result->Captured,result->Captured.Horizon,budget,yawGaps,nullptr);
                }
                if(haveYaw&&yawGaps[k].Lower<=gaps[index].Upper)
                    gaps[index].Lower=FMath::Max(gaps[index].Lower,yawGaps[k].Lower);
            }
            if(StrictPositive(gaps[index]))clear|=bit;
        }
        uint32 middle=0;
        if(clear!=AllCorners)
        {
            // Adjacent endpoints alone cannot close their continuous interval,
            // including the [0, FLT_MIN] gap in the compressed ordinals.
            if(cell.Last<=cell.First+1)return false;
            const float midpoint=first+.5f*(last-first);
            if(!FloorNativeSweepDetail::Ordinal(midpoint,middle)||middle<=cell.First||middle>=cell.Last)
                middle=cell.First+(cell.Last-cell.First)/2;
            if(middle<=cell.First||middle>=cell.Last)return false;
        }
        // A strictly clear corner on this closed interval is already proved
        // on both children. Publish its certified bound once, at this node.
        const uint16 newCorners=clear&~pending.ClearCorners;
        for(int32 family=0;family<2;++family)for(int32 k=0;k<8;++k)
        {
            const int32 index=family*8+k,role=family*16+k;
            if(!(newCorners&(1u<<index)))continue;
            if(!budget.Consume())return false;
            auto& gap=result->MinimumGap[index];
            gap=Valid(gap)?GeneralBoxDetail::Hull(gap,gaps[index]):gaps[index];
            if(!haveEnvelope){result->PhysicalEnvelope=bounds.Corners[role];haveEnvelope=true;}
            else result->PhysicalEnvelope=TimeDetail::Hull(result->PhysicalEnvelope,bounds.Corners[role]);
        }
        if(clear==AllCorners)
        {
            result->ClosedCover.Add(cell);
            continue;
        }
        stack.Add(FPendingCell{{middle,cell.Last},clear});
        stack.Add(FPendingCell{{cell.First,middle},clear});
        if(stack.Num()>32)return false;
    }
    if(!haveEnvelope||!Detail::ValidPoint(result->PhysicalEnvelope)||result->ClosedCover.IsEmpty()||
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=result->NumericFingerprint)return false;
    // DFS visits the left child first; publish only an exact closed partition.
    if(result->ClosedCover[0].First!=0||result->ClosedCover.Last().Last!=end)return false;
    for(int32 i=1;i<result->ClosedCover.Num();++i)
        if(result->ClosedCover[i-1].Last!=result->ClosedCover[i].First)return false;
    for(const auto& gap:result->MinimumGap)if(!StrictPositive(gap))return false;
    output=MoveTemp(result);return true;
}
}

namespace Speed::Analytic::Certified
{
inline bool MatchesNativeGeneralBoxFloorPhysicalClear(const FNativeGeneralBoxFloorDescriptorBounds& g)
{
    const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    return g.Complete&&g.NumericFingerprint==numeric.ControlFingerprint()&&g.PhysicalClear&&
        g.PhysicalClear->Matches(g.GetMotionInputForInspection(),g.CachedPlane.Point,g.CachedPlane.Normal);
}
}

// UNINSTALLED / UNCOMPILED. One pure original-time search, no native return,
// floor atlas, response, support or advance authority is implied.
namespace Speed::Analytic::Certified
{
enum class EGeneralPhysicalFloorMotion:uint8 {Unknown,WholePhysicalClear,FirstAdjacentArrival,InitialNativeBand};
inline TAutoConsoleVariable<int32> CVarGeneralBoxFloorInitialNativeBand(
    TEXT("p.IAmSpeed.AnalyticWorld.GeneralBoxFloorInitialNativeBand"),0,
    TEXT("Observe the existing native TOI-zero decision for a proved near-floor general Box. Default OFF."));
inline double GeneralBoxFloorInitialNativeRoundoffLower(const PreparedExecution::FNativeSolidStaticValues& v)
{
    using namespace AnalyticCertified;
    const auto& owner=v.OwnerState;
    const FVector bodyCenter=(FVector(owner.Location)-owner.Rotation.RotateVector(v.LocalCOM))+
        owner.Rotation.RotateVector(v.LocalOffset);
    if(!FloorNativeSweepDetail::SameVector(bodyCenter,v.BodyState.Location))return 0.;
    const auto scale=Add(Interval::Exact(bodyCenter.GetAbsMax()),Interval::Exact(v.PhysicalHalfExtent.GetMax()));
    if(!Valid(scale))return 0.;
    const auto roundoff=Multiply(Interval::Exact(64.*std::numeric_limits<double>::epsilon()),
        Interval::Exact(FMath::Max(1.,scale.Lower)));
    return Valid(roundoff)&&roundoff.Lower>0.?roundoff.Lower:0.;
}
class FGeneralPhysicalFloorMotionBounds;
using FGeneralPhysicalFloorMotionBoundsPtr=TSharedPtr<const FGeneralPhysicalFloorMotionBounds,ESPMode::ThreadSafe>;
inline EGeneralPhysicalFloorMotion BoundGeneralPhysicalFloorMotion(const FGeneralBoxMotionInput&,
    const FVector&,const FVector&,FDistanceBudget&,FGeneralPhysicalFloorMotionBoundsPtr&,double nativeInitialBand=0.);
class FGeneralPhysicalFloorMotionBounds final
{
    friend EGeneralPhysicalFloorMotion BoundGeneralPhysicalFloorMotion(const FGeneralBoxMotionInput&,
        const FVector&,const FVector&,FDistanceBudget&,FGeneralPhysicalFloorMotionBoundsPtr&,double);
    FGeneralBoxMotionInput Captured;FVector PlanePoint,PlaneNormal;
    EGeneralPhysicalFloorMotion Status=EGeneralPhysicalFloorMotion::Unknown;
    uint64 NumericFingerprint=0,VisitedClassifications=0;
    uint32 ClearOrdinal=0,InsideOrdinal=0;uint8 CommonInsideMask=0;
    Interval InitialGaps[16],ClearGaps[16],InsideGaps[16];
    TArray<GeneralPhysicalFloorClearDetail::FCell> PrefixCover;
    FGeneralPhysicalFloorMotionBounds()=default;
public:
    EGeneralPhysicalFloorMotion GetStatus()const{return Status;}
    float GetClearSeconds()const{return FloorNativeSweepDetail::Seconds(ClearOrdinal);}
    float GetInsideSeconds()const{return FloorNativeSweepDetail::Seconds(InsideOrdinal);}
    uint8 GetCommonInsideMask()const{return CommonInsideMask;}
    const FGeneralBoxMotionInput& GetInputForInspection()const{return Captured;}
    const Interval& GetInitialGapForInspection(int32 index)const{check(index>=0&&index<16);return InitialGaps[index];}
    TConstArrayView<GeneralPhysicalFloorClearDetail::FCell> GetPrefixForInspection()const{return PrefixCover;}
    uint64 GetClassificationsForInspection()const{return VisitedClassifications;}
    bool Matches(const FGeneralBoxMotionInput& input,const FVector& point,const FVector& normal)const
    {
        const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        return Status!=EGeneralPhysicalFloorMotion::Unknown&&numeric.ControlFingerprint()==NumericFingerprint&&
            GeneralPhysicalFloorClearDetail::SameInput(Captured,input)&&
            FloorNativeSweepDetail::SameVector(point,PlanePoint)&&FloorNativeSweepDetail::SameVector(normal,PlaneNormal);
    }
};
inline EGeneralPhysicalFloorMotion BoundGeneralPhysicalFloorMotion(const FGeneralBoxMotionInput& input,
    const FVector& point,const FVector& normal,FDistanceBudget& budget,FGeneralPhysicalFloorMotionBoundsPtr& output,
    double nativeInitialBand)
{
    const auto keepPrior=output;output.Reset();using namespace GeneralPhysicalFloorClearDetail;
    const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    uint32 end=0;
    if(!CardinalFloor(point,normal)||!FloorNativeSweepDetail::Ordinal(input.Horizon,end)||!end)return EGeneralPhysicalFloorMotion::Unknown;
    TSharedPtr<FGeneralPhysicalFloorMotionBounds,ESPMode::ThreadSafe> result(new FGeneralPhysicalFloorMotionBounds);
    result->Captured=input;result->PlanePoint=point;result->PlaneNormal=normal;result->NumericFingerprint=numeric.ControlFingerprint();
    const auto classify=[&](uint32 first,uint32 last,Interval(&gaps)[16],bool& allClear,uint8& commonInside)
    {
        allClear=false;commonInside=0;
        if(RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=result->NumericFingerprint||
            !budget.Consume())return false;
        ++result->VisitedClassifications;FGeneralBoxMotionBounds bounds;
        if(!BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(result->Captured,FloorNativeSweepDetail::Seconds(first),
            FloorNativeSweepDetail::Seconds(last),bounds)||bounds.Corners.Num()!=32)return false;
        allClear=true;
        for(int32 family=0;family<2;++family)for(int32 k=0;k<8;++k)
        {
            if(!budget.Consume()||!RawGap(bounds.Corners[family*16+k],point,normal,gaps[family*8+k]))return false;
            allClear&=StrictPositive(gaps[family*8+k]);
        }
        if(first==last)for(int32 k=0;k<8;++k)
        {
            const auto strictNegative=[](Interval value)
            {
                if(!AnalyticCertified::Valid(value))return false;
                const uint64 bits=AnalyticCertified::Bits(value.Upper),magnitude=bits&AnalyticCertified::Magnitude;
                return (bits&AnalyticCertified::Sign)&&magnitude>=AnalyticCertified::MinNormal&&magnitude<AnalyticCertified::Infinity;
            };
            if(strictNegative(gaps[k])&&strictNegative(gaps[k+8]))commonInside|=uint8(1u<<k);
        }
        return !(allClear&&commonInside);
    };
    bool clear=false;uint8 inside=0;
    if(!classify(0,0,result->InitialGaps,clear,inside)||!clear)return EGeneralPhysicalFloorMotion::Unknown;
    if(normal.Z==1.&&AnalyticCertified::Finite(nativeInitialBand)&&nativeInitialBand>0.)
    {
        // Selection for the existing native observation only. The original
        // raw/ref gaps are strictly positive, but BOTH minima are inside the
        // already compiled initial-event roundoff band. No physical contact
        // or positive-time prefix is claimed by this pure value.
        double nativeUpper=result->InitialGaps[0].Upper;
        double referenceUpper=result->InitialGaps[8].Upper;
        for(int32 k=1;k<8;++k)
        {
            nativeUpper=FMath::Min(nativeUpper,result->InitialGaps[k].Upper);
            referenceUpper=FMath::Min(referenceUpper,result->InitialGaps[k+8].Upper);
        }
        if(nativeUpper<=nativeInitialBand&&referenceUpper<=nativeInitialBand)
        {
            // Retain a true future encounter on the ORIGINAL motion as a
            // premise for consulting native TOI-zero policy. A near-floor
            // departure can never acquire an initial event from this route.
            if(!classify(end,end,result->InsideGaps,clear,inside))return EGeneralPhysicalFloorMotion::Unknown;
            if(inside)
            {
                result->Status=EGeneralPhysicalFloorMotion::InitialNativeBand;
                result->InsideOrdinal=end;result->CommonInsideMask=inside;
                if(!result->Matches(input,point,normal))return EGeneralPhysicalFloorMotion::Unknown;
                output=MoveTemp(result);return EGeneralPhysicalFloorMotion::InitialNativeBand;
            }
        }
    }
    TArray<FCell,TInlineAllocator<32>> stack;stack.Add(FCell{0,end});
    while(!stack.IsEmpty())
    {
        const auto cell=stack.Pop(EAllowShrinking::No);
        if(cell.First!=result->ClearOrdinal||cell.First>=cell.Last)return EGeneralPhysicalFloorMotion::Unknown;
        Interval gaps[16];
        if(!classify(cell.First,cell.Last,gaps,clear,inside))return EGeneralPhysicalFloorMotion::Unknown;
        if(clear)
        {
            // Retain a continuous CLOSED prefix, not just clear float samples.
            result->PrefixCover.Add(cell);result->ClearOrdinal=cell.Last;continue;
        }
        if(cell.Last==cell.First+1)
        {
            // No second search, no skipping an unresolved earlier leaf. C=0
            // does not authorize a positive-time native event.
            if(!cell.First||!classify(cell.First,cell.First,result->ClearGaps,clear,inside)||!clear||
                !classify(cell.Last,cell.Last,result->InsideGaps,clear,inside)||!inside)return EGeneralPhysicalFloorMotion::Unknown;
            result->InsideOrdinal=cell.Last;result->CommonInsideMask=inside;
            result->Status=EGeneralPhysicalFloorMotion::FirstAdjacentArrival;break;
        }
        const uint32 middle=cell.First+(cell.Last-cell.First)/2;
        if(middle<=cell.First||middle>=cell.Last)return EGeneralPhysicalFloorMotion::Unknown;
        stack.Add(FCell{middle,cell.Last});stack.Add(FCell{cell.First,middle});
        if(stack.Num()>32)return EGeneralPhysicalFloorMotion::Unknown;
    }
    if(result->Status!=EGeneralPhysicalFloorMotion::FirstAdjacentArrival)
    {
        if(result->ClearOrdinal!=end||!stack.IsEmpty())return EGeneralPhysicalFloorMotion::Unknown;
        result->Status=EGeneralPhysicalFloorMotion::WholePhysicalClear;
    }
    if(result->PrefixCover.IsEmpty()||result->PrefixCover[0].First!=0||result->PrefixCover.Last().Last!=result->ClearOrdinal)
        return EGeneralPhysicalFloorMotion::Unknown;
    for(int32 i=1;i<result->PrefixCover.Num();++i)
        if(result->PrefixCover[i-1].Last!=result->PrefixCover[i].First)return EGeneralPhysicalFloorMotion::Unknown;
    if(RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=result->NumericFingerprint)
        return EGeneralPhysicalFloorMotion::Unknown;
    const auto status=result->Status;output=MoveTemp(result);return status;
}
}

namespace Speed::Analytic::Certified
{
// Sealed by the prepared service with its ACTUAL complete FLOOR atlas and
// both finite domains. No contact, response or advance follows from fields.
struct FNativeGeneralBoxFloorMotionDescriptorBounds
{
    PreparedExecution::FNativeSolidStaticValues Captured;
    FIntervalPoint QueryEnvelope;FBoundedPlane NativePlane;FNativeBoxCachedPlaneValues Plane;
    FGeneralPhysicalFloorMotionBoundsPtr Motion;uint64 NumericFingerprint=0;bool Complete=false;
    FGeneralBoxMotionInput GetMotionInputForInspection()const
    {
        FGeneralBoxMotionInput input;input.Owner=Captured.OwnerState;input.LocalCOM=Captured.LocalCOM;input.LocalOffset=Captured.LocalOffset;
        input.LocalRotation=Captured.LocalRotation;input.PhysicalHalfExtent=Captured.PhysicalHalfExtent;
        input.QueryHalfExtent=Captured.QueryHalfExtent;input.Horizon=Captured.Horizon;return input;
    }
    bool HasMatchingPhysicalMotion()const
    {
        const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        return Complete&&NumericFingerprint==numeric.ControlFingerprint()&&Motion&&Detail::ValidPoint(QueryEnvelope)&&
            NativePlane.IsValid()&&Motion->Matches(GetMotionInputForInspection(),Plane.Point,Plane.Normal);
    }
};
}

// Pure mathematical corridor only. This value does not authorize a setter, response,
// support force, child synchronization or actor. All-child coverage is separate.
namespace Speed::Analytic::Certified
{
namespace ClosedPhysicalFloorPoseDetail
{
inline bool NonNegative(AnalyticCertified::Interval value)
{
    if(!AnalyticCertified::Valid(value))return false;
    const uint64 bits=AnalyticCertified::Bits(value.Lower),magnitude=bits&AnalyticCertified::Magnitude;
    return magnitude<AnalyticCertified::Infinity&&(!(bits&AnalyticCertified::Sign)||!magnitude);
}
inline uint64 Ordered(double value)
{
    const uint64 bits=AnalyticCertified::Bits(value);
    return (bits&AnalyticCertified::Sign)?~bits:(bits^AnalyticCertified::Sign);
}
inline bool Within(const FIntervalPoint& outer,const FIntervalPoint& inner)
{
    if(!Detail::ValidPoint(outer)||!Detail::ValidPoint(inner))return false;
    for(int32 d=0;d<3;++d)
        if(Ordered(inner.Axis[d].Lower)<Ordered(outer.Axis[d].Lower)||
            Ordered(inner.Axis[d].Upper)>Ordered(outer.Axis[d].Upper))return false;
    return true;
}
inline bool SamePoint(const FIntervalPoint& a,const FIntervalPoint& b)
{
    if(!Detail::ValidPoint(a)||!Detail::ValidPoint(b))return false;
    for(int32 d=0;d<3;++d)if(AnalyticCertified::Bits(a.Axis[d].Lower)!=AnalyticCertified::Bits(b.Axis[d].Lower)||
        AnalyticCertified::Bits(a.Axis[d].Upper)!=AnalyticCertified::Bits(b.Axis[d].Upper))return false;
    return true;
}
inline bool SameInput(const FBoxPoseCorridorInput& a,const FBoxPoseCorridorInput& b)
{
    using namespace FloorNativeSweepDetail;
    return SameVector(a.BeforeCOM,b.BeforeCOM)&&SameVector(a.AfterCOM,b.AfterCOM)&&
        SameQuat(a.BeforeRotation,b.BeforeRotation)&&SameQuat(a.AfterRotation,b.AfterRotation)&&
        SameVector(a.LocalCOM,b.LocalCOM)&&SameVector(a.LocalOffset,b.LocalOffset)&&SameQuat(a.LocalRotation,b.LocalRotation)&&
        SameVector(a.PhysicalHalfExtent,b.PhysicalHalfExtent)&&SameVector(a.QueryHalfExtent,b.QueryHalfExtent)&&
        AnalyticCertified::Bits(a.DomainShell)==AnalyticCertified::Bits(b.DomainShell)&&
        AnalyticCertified::Bits(a.OverlapShell)==AnalyticCertified::Bits(b.OverlapShell);
}
}
class FClosedPhysicalFloorPoseCorridorBounds;
using FClosedPhysicalFloorPoseCorridorBoundsPtr=TSharedPtr<const FClosedPhysicalFloorPoseCorridorBounds,ESPMode::ThreadSafe>;
inline bool TryProveClosedPhysicalFloorPoseCorridor(const FBoxPoseCorridorInput&,const FIntervalPoint&,
    const FVector&,const FVector&,FDistanceBudget&,FClosedPhysicalFloorPoseCorridorBoundsPtr&);
class FClosedPhysicalFloorPoseCorridorBounds final
{
    friend bool TryProveClosedPhysicalFloorPoseCorridor(const FBoxPoseCorridorInput&,const FIntervalPoint&,
        const FVector&,const FVector&,FDistanceBudget&,FClosedPhysicalFloorPoseCorridorBoundsPtr&);
    FBoxPoseCorridorInput Captured;FIntervalPoint AuthenticatedRawEnvelope,FullShellEnvelope;
    FVector PlanePoint,PlaneNormal;AnalyticCertified::Interval PhysicalGaps[8];uint64 NumericFingerprint=0;
    FClosedPhysicalFloorPoseCorridorBounds()=default;
public:
    const FBoxPoseCorridorInput& GetInputForInspection()const{return Captured;}
    const FIntervalPoint& GetRawEnvelopeForInspection()const{return AuthenticatedRawEnvelope;}
    const FIntervalPoint& GetShellEnvelopeForInspection()const{return FullShellEnvelope;}
    const AnalyticCertified::Interval& GetPhysicalGapForInspection(int32 k)const{check(k>=0&&k<8);return PhysicalGaps[k];}
    bool Matches(const FBoxPoseCorridorInput& input,const FIntervalPoint& raw,const FVector& point,const FVector& normal)const
    {
        const auto context=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        return context.ControlFingerprint()==NumericFingerprint&&ClosedPhysicalFloorPoseDetail::SameInput(Captured,input)&&
            ClosedPhysicalFloorPoseDetail::SamePoint(AuthenticatedRawEnvelope,raw)&&
            FloorNativeSweepDetail::SameVector(PlanePoint,point)&&FloorNativeSweepDetail::SameVector(PlaneNormal,normal);
    }
};
inline bool TryProveClosedPhysicalFloorPoseCorridor(const FBoxPoseCorridorInput& input,const FIntervalPoint& raw,
    const FVector& point,const FVector& normal,FDistanceBudget& budget,FClosedPhysicalFloorPoseCorridorBoundsPtr& output)
{
    const auto keepPrior=output;output.Reset();
    const auto context=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!GeneralPhysicalFloorClearDetail::CardinalFloor(point,normal)||!Detail::ValidPoint(raw))return false;
    TSharedPtr<FClosedPhysicalFloorPoseCorridorBounds,ESPMode::ThreadSafe> result(new FClosedPhysicalFloorPoseCorridorBounds);
    result->Captured=input;result->AuthenticatedRawEnvelope=raw;result->PlanePoint=point;result->PlaneNormal=normal;
    result->NumericFingerprint=context.ControlFingerprint();
    if(!budget.Consume()||!BuildBoxPoseCorridorBounds(result->Captured,result->FullShellEnvelope)||
        !ClosedPhysicalFloorPoseDetail::Within(raw,result->FullShellEnvelope))return false;
    auto physicalInput=result->Captured;physicalInput.DomainShell=physicalInput.OverlapShell=0;
    FIntervalPoint physicalHull;TArray<FIntervalPoint,TInlineAllocator<16>> corners;
    if(!budget.Consume()||!BuildBoxPoseCorridorBounds(physicalInput,physicalHull,&corners)||corners.Num()!=16)return false;
    // Physical0..7 only. Query8..15 and the original inflated physical group
    // are NOT a physical FLOOR nonpenetration theorem. Every physical corner
    // includes the existing raw/normalized quaternion family and native DAG.
    for(int32 k=0;k<8;++k)
    {
        if(RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=result->NumericFingerprint||
            !budget.Consume()||!GeneralPhysicalFloorClearDetail::RawGap(corners[k],point,normal,result->PhysicalGaps[k])||
            !ClosedPhysicalFloorPoseDetail::NonNegative(result->PhysicalGaps[k]))return false;
    }
    if(RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=result->NumericFingerprint)return false;
    output=MoveTemp(result);return true;
}
}

// UNINSTALLED proposal. Actual BODY pose geometry only, no owner advance,
// query, response, setter or synchronization permission. The reference below
// is ideal unit normalization of the supplied BODY quaternion, not an extra
// execution of FQuat::GetNormalized or an owner/local transform theorem.
namespace Speed::Analytic::Certified
{
struct FPhysicalBodyBoxPoseInput
{
    FVector BeforeCenter=FVector::ZeroVector,AfterCenter=FVector::ZeroVector;
    FQuat BeforeRotation=FQuat::Identity,AfterRotation=FQuat::Identity;
    FVector PhysicalHalfExtent=FVector::ZeroVector;
};
struct FPhysicalBodyBoxPoseBounds
{
    FPhysicalBodyBoxPoseInput Captured;
    FIntervalPoint NativeCorners[8],UnitReferenceCorners[8],CompleteHull;
    uint64 NumericFingerprint=0;bool Known=false;
};
namespace PhysicalBodyBoxPoseDetail
{
inline bool JoinPoint(FIntervalPoint& hull,const FIntervalPoint& point)
{return YawFloorBoxDetail::Join(hull,point);}
inline bool SameInput(const FPhysicalBodyBoxPoseInput& a,const FPhysicalBodyBoxPoseInput& b)
{
    return FloorNativeSweepDetail::SameVector(a.BeforeCenter,b.BeforeCenter)&&FloorNativeSweepDetail::SameVector(a.AfterCenter,b.AfterCenter)&&
        FloorNativeSweepDetail::SameVector(a.PhysicalHalfExtent,b.PhysicalHalfExtent)&&
        FloorNativeSweepDetail::SameQuat(a.BeforeRotation,b.BeforeRotation)&&FloorNativeSweepDetail::SameQuat(a.AfterRotation,b.AfterRotation);
}
}
namespace PhysicalBodyBoxPoseDetail
{
inline bool BuildOnRotationHull(const FPhysicalBodyBoxPoseInput& input,const YawFloorBoxDetail::FQuaternion& raw,
    FDistanceBudget& budget,FPhysicalBodyBoxPoseBounds& output)
{
    const auto captured=input;output={};
    const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    using namespace AnalyticCertified;using namespace YawFloorBoxDetail;
    if(!budget.Consume()||captured.BeforeCenter.ContainsNaN()||captured.AfterCenter.ContainsNaN()||
        captured.BeforeRotation.ContainsNaN()||captured.AfterRotation.ContainsNaN()||
        !captured.BeforeRotation.IsNormalized()||!captured.AfterRotation.IsNormalized()||
        captured.PhysicalHalfExtent.ContainsNaN()||captured.PhysicalHalfExtent.GetMin()<=0)return false;
    FPhysicalBodyBoxPoseBounds result;result.Captured=captured;result.NumericFingerprint=numeric.ControlFingerprint();
    Interval square[4];
    for(int32 k=0;k<4;++k)
    {
        if(!ValidNode(raw.Axis[k]))return false;
        square[k]=Square(raw.Axis[k].Range);
    }
    // Exact real q.q lies inside this interval; preserve the existing chord
    // normalization domain and refuse antipodal/zero-containing families.
    const auto squared=Add(Add(square[0],square[1]),Add(square[2],square[3]));
    if(!Valid(squared)||squared.Lower<=double(UE_SMALL_NUMBER))return false;
    volatile double lo=1./squared.Upper,hi=1./squared.Lower;Interval inverse;
    inverse.Known=Outward(lo,false,inverse.Lower)&&Outward(hi,true,inverse.Upper);
    const auto deltaScale=Subtract(inverse,Interval::Exact(1.));if(!Valid(deltaScale))return false;
    FPoint center;
    for(int32 d=0;d<3;++d)center.Axis[d]={GeneralBoxDetail::Hull(Interval::Exact(captured.BeforeCenter[d]),Interval::Exact(captured.AfterCenter[d])),0};
    for(int32 k=0;k<8;++k)
    {
        FVector corner;for(int32 d=0;d<3;++d)corner[d]=(k&(1<<d))?captured.PhysicalHalfExtent[d]:-captured.PhysicalHalfExtent[d];
        const auto rotated=Rotate(raw,corner);FPoint native;
        for(int32 d=0;d<3;++d)
        {
            // Native BODY pose: one actual stored center plus its raw Rotate.
            // No imaginary extra local Hamilton or COM/offset operations.
            native.Axis[d]=Sum(center.Axis[d],rotated.Axis[d]);
            // Quadratic correlation: R(q/sqrt(s),x)=R(q,x)+(1/s-1)*(R(q,x)-x).
            // Range describes ideal real algebra; Error is used only for the
            // actual native raw Rotate/Sum, never as a fake Normalize result.
            const auto correction=Multiply(deltaScale,Subtract(rotated.Axis[d].Range,Interval::Exact(corner[d])));
            result.UnitReferenceCorners[k].Axis[d]=Add(center.Axis[d].Range,Add(rotated.Axis[d].Range,correction));
        }
        result.NativeCorners[k]=Ranges(native);
        if(!Detail::ValidPoint(result.NativeCorners[k])||!Detail::ValidPoint(result.UnitReferenceCorners[k])||
            !PhysicalBodyBoxPoseDetail::JoinPoint(result.CompleteHull,result.NativeCorners[k])||
            !PhysicalBodyBoxPoseDetail::JoinPoint(result.CompleteHull,result.UnitReferenceCorners[k]))return false;
    }
    if(RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=result.NumericFingerprint)return false;
    result.Known=true;output=MoveTemp(result);return true;
}
}
inline bool BuildPhysicalBodyBoxPoseBounds(const FPhysicalBodyBoxPoseInput& input,FDistanceBudget& budget,
    FPhysicalBodyBoxPoseBounds& output)
{
    using AnalyticCertified::Interval;
    const double a[]={input.BeforeRotation.X,input.BeforeRotation.Y,input.BeforeRotation.Z,input.BeforeRotation.W};
    const double b[]={input.AfterRotation.X,input.AfterRotation.Y,input.AfterRotation.Z,input.AfterRotation.W};
    YawFloorBoxDetail::FQuaternion raw;
    for(int32 k=0;k<4;++k)
        raw.Axis[k]={GeneralBoxDetail::Hull(Interval::Exact(a[k]),Interval::Exact(b[k])),0};
    return PhysicalBodyBoxPoseDetail::BuildOnRotationHull(input,raw,budget,output);
}
class FClosedPhysicalBodyBoxFloorBounds;
using FClosedPhysicalBodyBoxFloorBoundsPtr=TSharedPtr<const FClosedPhysicalBodyBoxFloorBounds,ESPMode::ThreadSafe>;
inline bool TryProveClosedPhysicalBodyBoxFloor(const FPhysicalBodyBoxPoseInput&,const FVector&,const FVector&,FDistanceBudget&,FClosedPhysicalBodyBoxFloorBoundsPtr&);
class FClosedPhysicalBodyBoxFloorBounds final
{
    friend bool TryProveClosedPhysicalBodyBoxFloor(const FPhysicalBodyBoxPoseInput&,const FVector&,const FVector&,FDistanceBudget&,FClosedPhysicalBodyBoxFloorBoundsPtr&);
    FPhysicalBodyBoxPoseBounds Body;FVector PlanePoint,PlaneNormal;AnalyticCertified::Interval Gaps[16];
    FClosedPhysicalBodyBoxFloorBounds()=default;
public:
    const FPhysicalBodyBoxPoseBounds& GetBodyBoundsForInspection()const{return Body;}
    const AnalyticCertified::Interval& GetGapForInspection(int32 k)const{check(k>=0&&k<16);return Gaps[k];}
    bool Matches(const FPhysicalBodyBoxPoseInput& input,const FVector& point,const FVector& normal)const
    {
        return Body.Known&&Body.NumericFingerprint==RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()&&
            PhysicalBodyBoxPoseDetail::SameInput(Body.Captured,input)&&FloorNativeSweepDetail::SameVector(PlanePoint,point)&&FloorNativeSweepDetail::SameVector(PlaneNormal,normal);
    }
};
inline bool TryProveClosedPhysicalBodyBoxFloor(const FPhysicalBodyBoxPoseInput& input,const FVector& point,const FVector& normal,
    FDistanceBudget& budget,FClosedPhysicalBodyBoxFloorBoundsPtr& output)
{
    const auto keep=output;output.Reset();
    if(!GeneralPhysicalFloorClearDetail::CardinalFloor(point,normal))return false;
    TSharedPtr<FClosedPhysicalBodyBoxFloorBounds,ESPMode::ThreadSafe> result(new FClosedPhysicalBodyBoxFloorBounds);
    result->PlanePoint=point;result->PlaneNormal=normal;
    if(!BuildPhysicalBodyBoxPoseBounds(input,budget,result->Body))return false;
    for(int32 family=0;family<2;++family)for(int32 k=0;k<8;++k)
    {
        const auto& corner=family?result->Body.UnitReferenceCorners[k]:result->Body.NativeCorners[k];
        if(!budget.Consume()||!GeneralPhysicalFloorClearDetail::RawGap(corner,point,normal,result->Gaps[family*8+k])||
            !ClosedPhysicalFloorPoseDetail::NonNegative(result->Gaps[family*8+k]))return false;
    }
    if(!result->Matches(input,point,normal))return false;output=MoveTemp(result);return true;
}
}

namespace Speed::Analytic::Certified
{
// The two real BASE writes trace distinct legs. Prove the fixed-center
// quaternion chord first, then the fixed-rotation center segment. Each
// quarter encloses the exact shared chord parameter, not rounded waypoints.
inline bool TryBuildRotationFirstPhysicalBoxFloor(const FPhysicalBodyBoxPoseInput& input,
    const FVector& point,const FVector& normal,FDistanceBudget& budget,FIntervalPoint& hull,
    TArray<FBoxVolumeGroup,TInlineAllocator<16>>& groups)
{
    using namespace AnalyticCertified;
    hull={};groups.Reset();
    if(!GeneralPhysicalFloorClearDetail::CardinalFloor(point,normal))return false;
    FIntervalPoint complete;
    TArray<FBoxVolumeGroup,TInlineAllocator<16>> parts;
    const auto append=[&](const FPhysicalBodyBoxPoseBounds& bounds)
    {
        for(int32 family=0;family<2;++family)
        {
            auto& group=parts.AddDefaulted_GetRef();
            for(int32 k=0;k<8;++k)
            {
                const auto& corner=family?bounds.UnitReferenceCorners[k]:bounds.NativeCorners[k];
                Interval gap;
                if(!budget.Consume()||!GeneralPhysicalFloorClearDetail::RawGap(corner,point,normal,gap)||
                    !ClosedPhysicalFloorPoseDetail::NonNegative(gap)||!YawFloorBoxDetail::Join(complete,corner))return false;
                group.Corners.Add(corner);
            }
        }
        return true;
    };
    const double a[]={input.BeforeRotation.X,input.BeforeRotation.Y,input.BeforeRotation.Z,input.BeforeRotation.W};
    const double b[]={input.AfterRotation.X,input.AfterRotation.Y,input.AfterRotation.Z,input.AfterRotation.W};
    auto rotating=input;rotating.AfterCenter=rotating.BeforeCenter;
    constexpr int32 QuarterCount=4;
    for(int32 i=0;i<QuarterCount;++i)
    {
        YawFloorBoxDetail::FQuaternion raw;
        for(int32 k=0;k<4;++k)
        {
            const auto first=Interval::Exact(a[k]),last=Interval::Exact(b[k]);
            const auto at=[&](int32 index)
            {
                if(index==0||Bits(a[k])==Bits(b[k]))return first;
                if(index==QuarterCount)return last;
                return Add(first,Multiply(Subtract(last,first),Interval::Exact(index*.25)));
            };
            raw.Axis[k]={GeneralBoxDetail::Hull(at(i),at(i+1)),0};
        }
        FPhysicalBodyBoxPoseBounds bounds;
        if(!PhysicalBodyBoxPoseDetail::BuildOnRotationHull(rotating,raw,budget,bounds)||!append(bounds))return false;
    }
    auto translating=input;translating.BeforeRotation=translating.AfterRotation;
    FPhysicalBodyBoxPoseBounds bounds;
    if(!BuildPhysicalBodyBoxPoseBounds(translating,budget,bounds)||!append(bounds))return false;
    hull=complete;groups=MoveTemp(parts);return true;
}
}

// PROSPECTIVE PURE GEOMETRY. Not installed and not physical acceptance.
// The original input/H remains intact; end is the actually observed native
// callback TOI. This proves nonpenetration of BOTH physical Box families on
// the complete CLOSED prefix, including an initially departing boundary.
namespace Speed::Analytic::Certified
{
// Proof-only lower bound on the original prefix with an axial floor and an
// identity local rotation. No contact or motion is computed here.
namespace CorrelatedAxialBoxFloorProof
{
using namespace ::AnalyticCertified;
using GeneralBoxDetail::Hull;
using GeneralBoxDetail::DividePositive;

inline FIntervalPoint Sum(FIntervalPoint a,const FIntervalPoint& b)
{
    for(int32 d=0;d<3;++d)a.Axis[d]=Add(a.Axis[d],b.Axis[d]);
    return a;
}
inline FIntervalPoint Scale(FIntervalPoint p,Interval s)
{
    for(auto& x:p.Axis)x=Multiply(x,s);
    return p;
}
inline FIntervalPoint Cross(const FIntervalPoint& a,const FIntervalPoint& b)
{
    FIntervalPoint c;
    for(int32 d=0;d<3;++d)
    {
        const int32 j=(d+1)%3,k=(d+2)%3;
        c.Axis[d]=Subtract(Multiply(a.Axis[j],b.Axis[k]),Multiply(a.Axis[k],b.Axis[j]));
    }
    return c;
}
inline Interval NormSquared(const FIntervalPoint& a)
{
    return Add(Add(Square(a.Axis[0]),Square(a.Axis[1])),Square(a.Axis[2]));
}
inline bool Within(Interval x,double lo,double hi)
{
    return Valid(x)&&x.Lower>=lo&&x.Upper<=hi;
}
inline bool PartsInRange(const FVector& v,double maximumZ)
{
    return Within(Add(Interval::Exact(FMath::Abs(v.X)),Interval::Exact(FMath::Abs(v.Y))),-0x1p-1022,128.)&&
        Within(Interval::Exact(v.Z),-maximumZ,maximumZ);
}

inline bool Bound(const FGeneralBoxMotionInput& v,float last,FDistanceBudget& budget,Interval (&lower)[8])
{
#if !defined(_MSC_VER) || !defined(_M_X64) || defined(_M_ARM64EC) || !PLATFORM_ENABLE_VECTORINTRINSICS
    return false;
#else
    if(!(last>0&&last<=0x1.1p-14f)||Bits(v.LocalRotation.W)!=Bits(1.)||
        ((Bits(v.LocalRotation.X)|Bits(v.LocalRotation.Y)|Bits(v.LocalRotation.Z))&Magnitude))return false;
    const auto& owner=v.Owner;
    const auto& q=owner.Rotation;
    if(!Within(Interval::Exact(owner.Location.Z),-32.,32.)||
        !Within(Interval::Exact(owner.Velocity.Z),-64.,64.)||
        !Within(Interval::Exact(owner.Acceleration.Z),-512.,512.)||
        !Within(Interval::Exact(q.Z),-0x1p-11,0x1p-11)||
        !Within(Interval::Exact(q.W),-0x1p-11,0x1p-11)||
        !PartsInRange(v.LocalCOM,24.)||!PartsInRange(v.LocalOffset,32.)||
        !PartsInRange(v.PhysicalHalfExtent,24.))return false;
    const auto qxyz=Detail::Point(FVector(q.X,q.Y,q.Z));
    const auto s0=Add(NormSquared(qxyz),Square(Interval::Exact(q.W)));
    const auto w=Detail::Point(owner.AngularVelocity);
    const auto alpha=Detail::Point(owner.AngularAcceleration);
    if(!Within(s0,1.-0x1p-40,1.+0x1p-40)||!Within(NormSquared(w),-0x1p-1022,1.)||
        !Within(NormSquared(alpha),-0x1p-1022,100.)||
        budget.Remaining-budget.ConsumptionFloor<32||MAX_uint64-budget.Used<32)return false;
    for(int32 i=0;i<32;++i)if(!budget.Consume())return false;

    const Interval time{0.,double(last),true};
    const auto t2=Square(time);
    const auto b=Scale(alpha,Interval::Exact(.5));
    const auto axis=Sum(w,Scale(b,time));
    const auto u2=NormSquared(axis);
    if(!Within(Multiply(t2,u2),-0x1p-1022,0x1p-24))return false;
    // sin(t*|axis|)/(t*|axis|) = 1 + t^2*S, and
    // (1-cos(t*|axis|))/(t*|axis|)^2 = 1/2 + t^2*C.
    // Taylor remainder has the sign of the first omitted term here.
    const auto correction=[&](double divisor,double remainder)
    {
        const auto coefficient=Subtract(Interval::Exact(0.),DividePositive(u2,divisor));
        const auto upper=Add(coefficient,DividePositive(Multiply(t2,Square(u2)),remainder));
        return Hull(coefficient,upper);
    };
    const auto sinc=correction(6.,120.);
    const auto cosc=correction(24.,720.);
    if(!Valid(sinc)||!Valid(cosc))return false;

    for(int32 k=0;k<8;++k)
    {
        FIntervalPoint x;
        for(int32 d=0;d<3;++d)
            x.Axis[d]=Add(Subtract(Interval::Exact(v.LocalOffset[d]),Interval::Exact(v.LocalCOM[d])),
                Interval::Exact((k&(1<<d))?v.PhysicalHalfExtent[d]:-v.PhysicalHalfExtent[d]));
        if(!Within(NormSquared(x),-0x1p-1022,12544.))return false;
        const auto twiceCross=Scale(Cross(qxyz,x),Interval::Exact(2.));
        const auto rotDelta=Sum(Scale(twiceCross,Interval::Exact(q.W)),Cross(qxyz,twiceCross));
        FIntervalPoint unit=x;
        for(int32 d=0;d<3;++d)
            unit.Axis[d]=Add(x.Axis[d],Hull(DividePositive(rotDelta.Axis[d],s0.Lower),
                DividePositive(rotDelta.Axis[d],s0.Upper)));

        const auto wx=Cross(w,unit),bx=Cross(b,unit);
        const auto c1=wx.Axis[2],c2=bx.Axis[2],d0=Cross(w,wx).Axis[2];
        const auto d1=Sum(Cross(w,bx),Cross(b,wx)).Axis[2],d2=Cross(b,bx).Axis[2];
        const auto g3=Add(Add(Multiply(sinc,Add(c1,Multiply(time,c2))),
                Multiply(Interval::Exact(.5),Add(d1,Multiply(time,d2)))),
            Multiply(Multiply(time,cosc),Add(d0,Multiply(time,Add(d1,Multiply(time,d2))))));
        const auto base1=Interval::Exact(owner.Velocity.Z);
        const auto base2=Multiply(Interval::Exact(.5),Interval::Exact(owner.Acceleration.Z));
        const auto quadratic=Add(c2,Multiply(Interval::Exact(.5),d0));
        const auto eval=[&](Interval z,Interval norm)
        {
            const auto g0=Add(Interval::Exact(owner.Location.Z),z);
            const auto g1=Add(base1,Multiply(norm,c1));
            const auto g2=Add(base2,Multiply(norm,quadratic));
            return Add(g0,Multiply(time,Add(g1,Multiply(time,Add(g2,Multiply(time,Multiply(norm,g3)))))));
        };
        // Include raw and unit rotation. The gated Win64 native path has
        // at most 2.655e-13 cm directional rotation error after removing the
        // radial cosine scale, 3.980e-13 cm norm-squared error, 6.42e-14 cm
        // binary32 t*t translation error, and 2.431e-13 cm Rotate/placement
        // error (eight binary64 roundings per bounded monomial). These sum
        // below 9.72e-13 cm, including zero/flush branches. Subtract a larger
        // exact binary64 allowance from the lower proof, never from a
        // collision threshold or from a physical position.
        const auto range=Hull(eval(unit.Axis[2],Interval::Exact(1.)),eval(Add(x.Axis[2],rotDelta.Axis[2]),s0));
        lower[k]=Subtract(range,Interval::Exact(0x1.4p-40));
        if(!Valid(lower[k]))return false;
    }
    return true;
#endif
}
}

inline TAutoConsoleVariable<int32> CVarCorrelatedAxialBoxFloorPrefix(
    TEXT("p.IAmSpeed.AnalyticWorld.CorrelatedAxialBoxFloorPrefix"),0,
    TEXT("Directed short-prefix bound for an axial floor and identity local rotation. Default OFF."));
class FGeneralBoxPhysicalFloorPrefixBounds;
using FGeneralBoxPhysicalFloorPrefixBoundsPtr=TSharedPtr<const FGeneralBoxPhysicalFloorPrefixBounds,ESPMode::ThreadSafe>;
inline bool TryProveGeneralBoxPhysicalFloorPrefix(const FGeneralBoxMotionInput&,const FVector&,const FVector&,float,
    FDistanceBudget&,FGeneralBoxPhysicalFloorPrefixBoundsPtr&);
inline bool TryProveFixedCardinalBoxPhysicalFloorPrefix(const FGeneralBoxMotionInput&,const FVector&,const FVector&,float,
    FDistanceBudget&,FGeneralBoxPhysicalFloorPrefixBoundsPtr&);
class FGeneralBoxPhysicalFloorPrefixBounds final
{
    friend bool TryProveFixedCardinalBoxPhysicalFloorPrefix(const FGeneralBoxMotionInput&,const FVector&,const FVector&,float,
        FDistanceBudget&,FGeneralBoxPhysicalFloorPrefixBoundsPtr&);
    friend bool TryProveGeneralBoxPhysicalFloorPrefix(const FGeneralBoxMotionInput&,const FVector&,const FVector&,float,
        FDistanceBudget&,FGeneralBoxPhysicalFloorPrefixBoundsPtr&);
    FGeneralBoxMotionInput Captured;
    FVector PlanePoint=FVector::ZeroVector,PlaneNormal=FVector::ZeroVector;
    float MaximumAdvanceSeconds=0;
    FIntervalPoint PhysicalEnvelope;
    Interval MinimumGap[16];
    TArray<GeneralPhysicalFloorClearDetail::FCell> ClosedCover;
    uint64 NumericFingerprint=0,VisitedCells=0;
    bool FixedCardinal=false;
    FGeneralBoxPhysicalFloorPrefixBounds()=default;
public:
    bool Matches(const FGeneralBoxMotionInput& input,const FVector& point,const FVector& normal,float end)const
    {
        const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        return numeric.ControlFingerprint()==NumericFingerprint&&!ClosedCover.IsEmpty()&&
            GeneralPhysicalFloorClearDetail::SameInput(Captured,input)&&
            FloorNativeSweepDetail::SameVector(PlanePoint,point)&&FloorNativeSweepDetail::SameVector(PlaneNormal,normal)&&
            KinematicCenterDetail::Bits(MaximumAdvanceSeconds)==KinematicCenterDetail::Bits(end);
    }
    const FGeneralBoxMotionInput& GetInputForInspection()const{return Captured;}
    const FIntervalPoint& GetPhysicalEnvelopeForInspection()const{return PhysicalEnvelope;}
    bool IsFixedCardinal() const{return FixedCardinal;}
    float GetMaximumAdvanceSeconds()const{return MaximumAdvanceSeconds;}
    TConstArrayView<GeneralPhysicalFloorClearDetail::FCell> GetClosedCoverForInspection()const{return ClosedCover;}
    const Interval& GetMinimumGapForInspection(int32 index)const{check(index>=0&&index<16);return MinimumGap[index];}
    uint64 GetVisitedCellCountForInspection()const{return VisitedCells;}
};
inline bool TryProveGeneralBoxPhysicalFloorPrefix(const FGeneralBoxMotionInput& input,const FVector& point,const FVector& normal,
    float endSeconds,FDistanceBudget& budget,FGeneralBoxPhysicalFloorPrefixBoundsPtr& output)
{
    const auto keepPrior=output;output.Reset();
    const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    using namespace GeneralPhysicalFloorClearDetail;
    uint32 end=0,originalEnd=0;
    if(!CardinalFloor(point,normal)||!FloorNativeSweepDetail::Ordinal(input.Horizon,originalEnd)||!originalEnd||
        !FloorNativeSweepDetail::Ordinal(endSeconds,end)||!end||end>originalEnd)return false;
    TSharedPtr<FGeneralBoxPhysicalFloorPrefixBounds,ESPMode::ThreadSafe> result(new FGeneralBoxPhysicalFloorPrefixBounds);
    result->Captured=input;result->PlanePoint=point;result->PlaneNormal=normal;
    result->MaximumAdvanceSeconds=endSeconds;result->NumericFingerprint=numeric.ControlFingerprint();
    struct FPendingCell {FCell Cell;uint16 ClearCorners;};
    TArray<FPendingCell,TInlineAllocator<32>> stack;stack.Add(FPendingCell{{0,end},0});
    constexpr uint16 AllCorners=0xffff;
    Interval correlated[8];
    const bool tryCorrelated=normal.Z==1.&&endSeconds<=0x1.1p-14f&&
        CVarCorrelatedAxialBoxFloorPrefix.GetValueOnAnyThread()==1;
    bool triedCorrelated=false,haveCorrelated=false;
    bool haveEnvelope=false;
    while(!stack.IsEmpty())
    {
        if(RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=result->NumericFingerprint||
            !budget.Consume())return false;
        const auto pending=stack.Pop(EAllowShrinking::No);++result->VisitedCells;
        const auto cell=pending.Cell;
        const float first=FloorNativeSweepDetail::Seconds(cell.First),last=FloorNativeSweepDetail::Seconds(cell.Last);
        FGeneralBoxMotionBounds bounds;
        // Every interval uses the original owner and global seconds.
        if(!BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(result->Captured,first,last,bounds)||bounds.Corners.Num()!=32)return false;
        Interval gaps[16];
        uint16 clear=pending.ClearCorners;
        for(int32 family=0;family<2;++family)for(int32 k=0;k<8;++k)
        {
            const int32 index=family*8+k,role=family*16+k;
            const uint16 bit=uint16(1u<<index);
            if(clear&bit)continue;
            if(!budget.Consume()||!RawGap(bounds.Corners[role],result->PlanePoint,result->PlaneNormal,gaps[index]))return false;
            if(tryCorrelated&&!ClosedPhysicalFloorPoseDetail::NonNegative(gaps[index]))
            {
                if(!triedCorrelated)
                {
                    triedCorrelated=true;
                    haveCorrelated=CorrelatedAxialBoxFloorProof::Bound(result->Captured,endSeconds,budget,correlated);
                }
                if(haveCorrelated&&correlated[k].Lower<=gaps[index].Upper)
                    gaps[index].Lower=FMath::Max(gaps[index].Lower,correlated[k].Lower);
            }
            if(ClosedPhysicalFloorPoseDetail::NonNegative(gaps[index]))clear|=bit;
        }
        uint32 middle=0;
        if(clear!=AllCorners)
        {
            // Adjacent endpoints alone cannot close their continuous interval,
            // including the [0, FLT_MIN] gap in the compressed ordinals.
            if(cell.Last<=cell.First+1)
            {
                for(int32 index=0;index<16;++index)if(!(clear&(1u<<index)))
                    UE_LOG(LogTemp,Display,TEXT("[NativeBoxFloorPrefixAdjacentRefusal] First=%.17g Last=%.17g CornerRole=%d Gap=(%.17g,%.17g)"),
                        double(first),double(last),index,gaps[index].Lower,gaps[index].Upper);
                return false;
            }
            // Split the ordered native seconds, including the cell at
            // zero, rather than repeatedly halving the real duration there.
            // The same closed ranges and gaps are tested. In at most 31
            // subdivisions the existing adjacent-cell proof must decide.
            middle=cell.First+(cell.Last-cell.First)/2;
            if(middle<=cell.First||middle>=cell.Last)return false;
        }
        // A strictly clear corner on this closed interval is already proved
        // on both children. Publish its certified bound once, at this node.
        const uint16 newCorners=clear&~pending.ClearCorners;
        for(int32 family=0;family<2;++family)for(int32 k=0;k<8;++k)
        {
            const int32 index=family*8+k,role=family*16+k;
            if(!(newCorners&(1u<<index)))continue;
            if(!budget.Consume())return false;
            auto& gap=result->MinimumGap[index];
            gap=Valid(gap)?GeneralBoxDetail::Hull(gap,gaps[index]):gaps[index];
            if(!haveEnvelope){result->PhysicalEnvelope=bounds.Corners[role];haveEnvelope=true;}
            else result->PhysicalEnvelope=TimeDetail::Hull(result->PhysicalEnvelope,bounds.Corners[role]);
        }
        if(clear==AllCorners)
        {
            result->ClosedCover.Add(cell);
            continue;
        }
        stack.Add(FPendingCell{{middle,cell.Last},clear});
        stack.Add(FPendingCell{{cell.First,middle},clear});
        if(stack.Num()>32)return false;
    }
    if(!haveEnvelope||!Detail::ValidPoint(result->PhysicalEnvelope)||result->ClosedCover.IsEmpty()||
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=result->NumericFingerprint)return false;
    if(result->ClosedCover[0].First!=0||result->ClosedCover.Last().Last!=end)return false;
    for(int32 i=1;i<result->ClosedCover.Num();++i)
        if(result->ClosedCover[i-1].Last!=result->ClosedCover[i].First)return false;
    for(const auto& gap:result->MinimumGap)if(!ClosedPhysicalFloorPoseDetail::NonNegative(gap))return false;
    output=MoveTemp(result);return true;
}
// Narrow Fixed provenance for an exactly upside-down cardinal owner on +Z
// FLOOR. The caller authenticates the original Fixed support descriptor,
// complete scene/filters and its one-use remaining work account.
inline bool TryProveFixedCardinalBoxPhysicalFloorPrefix(const FGeneralBoxMotionInput& input,
    const FVector& point,const FVector& normal,float endSeconds,FDistanceBudget& budget,
    FGeneralBoxPhysicalFloorPrefixBoundsPtr& output)
{
    const auto keepPrior=output;output.Reset();
    const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    using AnalyticCertified::Bits;
    constexpr auto magnitude=AnalyticCertified::Magnitude;
    const auto zero=[](double x){return (Bits(x)&magnitude)==0;};
    const auto zeroVector=[&](const FVector& v){return zero(v.X)&&zero(v.Y)&&zero(v.Z);};
    const auto& q=input.Owner.Rotation;
    const auto& lq=input.LocalRotation;
    uint32 end=0,horizon=0;
    if(!GeneralPhysicalFloorClearDetail::CardinalFloor(point,normal)||Bits(normal.Z)!=Bits(1.)||
        !zero(q.X)||(Bits(q.Y)&magnitude)!=Bits(1.)||!zero(q.Z)||!zero(q.W)||
        !zero(lq.X)||!zero(lq.Y)||!zero(lq.Z)||Bits(lq.W)!=Bits(1.)||
        !zeroVector(input.Owner.AngularVelocity)||!zeroVector(input.Owner.AngularAcceleration)||
        !zero(input.Owner.Velocity.Z)||!FloorNativeSweepDetail::Ordinal(input.Horizon,horizon)||
        !FloorNativeSweepDetail::Ordinal(endSeconds,end)||!end||end>horizon||endSeconds>1.f)
        return false;
    const auto boundedPositive=[](double x)
    {
        return AnalyticCertified::Finite(x)&&x>=1.&&x<=0x1p16;
    };
    const double p=input.Owner.Location.Z,com=input.LocalCOM.Z,offset=input.LocalOffset.Z;
    const double half=input.PhysicalHalfExtent.Z,acceleration=input.Owner.Acceleration.Z;
    if(!boundedPositive(p)||!boundedPositive(com)||!boundedPositive(offset)||!boundedPositive(half)||
        !boundedPositive(-acceleration)||!budget.Consume())return false;

    // Keep the unchanged general arithmetic enclosure for the entire physical
    // box, but use the fixed signed normal theorem for the tiny cancellation.
    // This enclosure independently validates finite tangential inputs.
    FGeneralBoxMotionBounds bounds;
    if(!BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(input,0.f,endSeconds,bounds)||bounds.Corners.Num()!=32)return false;
    const auto exactDifference=[](double a,double b,double& result)
    {
        // Sterbenz, with normal result. Multiplication by 2 is exact in this domain.
        if(!(a>0.&&b>0.&&a<=2.*b&&b<=2.*a))return false;
        volatile double difference=a-b;
        const auto absolute=Bits(difference)&magnitude;
        if(absolute<AnalyticCertified::MinNormal)return false;
        result=difference;return true;
    };
    double comMinusHalf=0.,offsetMinusPosition=0.,referenceInitial=0.;
    if(!exactDifference(com,half,comMinusHalf)||!exactDifference(offset,p,offsetMinusPosition)||
        !exactDifference(comMinusHalf,offsetMinusPosition,referenceInitial)||!(referenceInitial>0.))return false;
    const double halfAcceleration=acceleration*.5; // exact exponent shift
    const double t=double(endSeconds),square=t*t; // exact product of two promoted binary32 values
    if(!(square>=double(std::numeric_limits<float>::min())))return false;
    const auto travel=AnalyticCertified::Multiply(Interval::Exact(halfAcceleration),Interval{0.,square,true});
    const auto referenceLow=AnalyticCertified::Add(Interval::Exact(referenceInitial),travel);
    const auto referenceHigh=AnalyticCertified::Add(referenceLow,Interval::Exact(2.*half));

    // The native Z DAG is ordered as compiled by the fixed model. Force each
    // store explicitly. Cardinal +/- unit quaternion rotation has exact Z
    // inversion on the bounded operands, and +Z plane dot adds only zeros.
    // RNE, with either allowed FTZ/DAZ setting, is monotone at each operation;
    // the negative coefficient therefore makes each corner nonincreasing.
    const auto nativeGap=[&](float seconds,double corner)
    {
        volatile float t2=seconds*seconds;
        volatile double displacement=halfAcceleration*double(t2);
        volatile double owner=p+displacement;
        volatile double shifted=owner+com;
        volatile double center=shifted-offset;
        volatile double gap=center+corner;
        return double(gap);
    };
    const Interval nativeLow{nativeGap(endSeconds,-half),nativeGap(0.f,-half),true};
    const Interval nativeHigh{nativeGap(endSeconds,half),nativeGap(0.f,half),true};
    const Interval gaps[2][2]={{nativeHigh,nativeLow},{referenceHigh,referenceLow}};
    for(const auto& family:gaps)for(const auto& gap:family)
        if(!AnalyticCertified::Valid(gap)||!(gap.Lower>0.))return false;

    TSharedPtr<FGeneralBoxPhysicalFloorPrefixBounds,ESPMode::ThreadSafe> result(new FGeneralBoxPhysicalFloorPrefixBounds);
    result->Captured=input;result->PlanePoint=point;result->PlaneNormal=normal;
    result->MaximumAdvanceSeconds=endSeconds;result->NumericFingerprint=numeric.ControlFingerprint();
    for(int32 family=0;family<2;++family)for(int32 corner=0;corner<8;++corner)
    {
        const auto& bound=bounds.Corners[family*16+corner];
        if(!budget.Consume()||!Detail::ValidPoint(bound)||!budget.Consume())return false;
        result->MinimumGap[family*8+corner]=gaps[family][(corner&4)?1:0];
        result->PhysicalEnvelope=family||corner?TimeDetail::Hull(result->PhysicalEnvelope,bound):bound;
    }
    if(!Detail::ValidPoint(result->PhysicalEnvelope)||
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=result->NumericFingerprint)
        return false;
    result->ClosedCover.Add(GeneralPhysicalFloorClearDetail::FCell{0,end});
    result->VisitedCells=1;result->FixedCardinal=true;
    output=MoveTemp(result);return true;
}
}

// Prospective immutable geometry values, not a native-contact capability.
namespace Speed::Analytic::Certified
{
struct FNativeGeneralCachedFloorArrivalDescriptorBounds
{
    PreparedExecution::FNativeSolidStaticValues Captured;
    FIntervalPoint QueryEnvelope;FBoundedPlane NativePlane;FNativeBoxCachedPlaneValues Plane;
    FGeneralBoxPhysicalFloorPrefixBoundsPtr CachedPrefix;
    uint64 NumericFingerprint=0,PreludeWorkUsed=0,TotalWorkUsed=0;
    int32 OriginalWorkLimit=0,RemainingWork=0;bool Complete=false;
    FGeneralBoxMotionInput GetMotionInputForInspection()const
    {
        FGeneralBoxMotionInput input;input.Owner=Captured.OwnerState;input.LocalCOM=Captured.LocalCOM;input.LocalOffset=Captured.LocalOffset;
        input.LocalRotation=Captured.LocalRotation;input.PhysicalHalfExtent=Captured.PhysicalHalfExtent;
        input.QueryHalfExtent=Captured.QueryHalfExtent;input.Horizon=Captured.Horizon;return input;
    }
    bool HasMatchingPhysicalPrefix()const
    {
        const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        return Complete&&CachedPrefix&&NativePlane.IsValid()&&Detail::ValidPoint(QueryEnvelope)&&
            NumericFingerprint==numeric.ControlFingerprint()&&OriginalWorkLimit>0&&RemainingWork>=0&&
            PreludeWorkUsed>0&&TotalWorkUsed>=PreludeWorkUsed&&TotalWorkUsed<=uint64(OriginalWorkLimit)&&
            TotalWorkUsed+uint64(RemainingWork)==uint64(OriginalWorkLimit)&&
            CachedPrefix->Matches(GetMotionInputForInspection(),Plane.Point,Plane.Normal,CachedPrefix->GetMaximumAdvanceSeconds());
    }
};
}

// Source-only review draft. Intended after the CorrelatedAxialBoxFloorProof
// helpers in NativeFloorBoxSupportBounds.h. No consumer, actor or qualification.
namespace Speed::Analytic::Certified::CorrelatedAxialBoxFloorProof
{
struct FWorldYawGapDiagnostic
{
    const TCHAR* Stage=TEXT("input_domain");
    double OffAxisRadiansUpper=0.;
    Interval EffectiveCenterZ;
    double SmallestLower=0.;
};

// Strictly a geometric enclosure of all eight physical corner gaps for both
// consumed native and continuous reference families. No support mask, event,
// witness, motion change or contact threshold.
inline bool BoundWorldYaw(const FGeneralBoxMotionInput& input,float last,
    FDistanceBudget& budget,Interval (&out)[8],FWorldYawGapDiagnostic* diagnostic=nullptr)
{
    FWorldYawGapDiagnostic details;
    const auto finish=[&](const TCHAR* stage,bool success)
    {
        details.Stage=stage;
        if(diagnostic)*diagnostic=details;
        return success;
    };
#if !defined(_MSC_VER) || !defined(_M_X64) || defined(_M_ARM64EC) || !PLATFORM_ENABLE_VECTORINTRINSICS
    return finish(TEXT("numeric_platform"),false);
#else
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!GeneralBoxGlobalTimeDetail::ValidFloatSecondsRange(0.f,last)||
        !(last>0&&last<=0x1p-8f&&last<=input.Horizon)||
        Bits(input.LocalRotation.W)!=Bits(1.)||
        ((Bits(input.LocalRotation.X)|Bits(input.LocalRotation.Y)|Bits(input.LocalRotation.Z))&Magnitude))
        return finish(TEXT("time_or_local_rotation"),false);

    const auto& state=input.Owner;
    const auto& q=state.Rotation;
    const auto p=Interval::Exact(state.Location.Z);
    const auto vz=Interval::Exact(state.Velocity.Z);
    const auto az=Interval::Exact(state.Acceleration.Z);
    if(!Within(p,-32.,32.)||!Within(vz,-64.,64.)||!Within(az,-512.,512.)||
        !Within(Interval::Exact(q.Z),-0x1p-11,0x1p-11)||
        !Within(Interval::Exact(q.W),-0x1p-11,0x1p-11)||
        !PartsInRange(input.LocalCOM,24.)||!PartsInRange(input.LocalOffset,32.)||
        !PartsInRange(input.PhysicalHalfExtent,24.)||input.PhysicalHalfExtent.GetMin()<=0)
        return finish(TEXT("geometry_or_center_domain"),false);

    const auto qxyz=Detail::Point(FVector(q.X,q.Y,q.Z));
    const auto s0=Add(NormSquared(qxyz),Square(Interval::Exact(q.W)));
    const auto w2=NormSquared(Detail::Point(state.AngularVelocity));
    const auto a2=NormSquared(Detail::Point(state.AngularAcceleration));
    if(!Within(s0,1.-0x1p-40,1.+0x1p-40)||!Valid(w2)||w2.Upper>36.||
        !Valid(a2)||a2.Upper>100.)
        return finish(TEXT("rotation_domain"),false);

    const Interval time{0.,double(last),true};
    const auto realSquared=Square(time);
    const auto nativeSquared=KinematicCenterDetail::RoundedBinary32(realSquared);
    if(!Valid(realSquared)||!Valid(nativeSquared))return finish(TEXT("time_square"),false);
    const auto squared=Hull(realSquared,nativeSquared);
    const auto sumXY=[](const FVector& v)
    {
        return Add(Interval::Exact(FromBits(Bits(v.X)&Magnitude)),Interval::Exact(FromBits(Bits(v.Y)&Magnitude)));
    };
    const auto offAxis=Add(Multiply(time,sumXY(state.AngularVelocity)),
        Multiply(Multiply(Interval::Exact(.5),squared),sumXY(state.AngularAcceleration)));
    if(!Valid(offAxis)||offAxis.Upper>0x1p-13)
        return finish(TEXT("off_axis_domain"),false);
    details.OffAxisRadiansUpper=offAxis.Upper;
    const auto center=Add(Add(p,Multiply(vz,time)),
        Multiply(Multiply(Interval::Exact(.5),az),squared));
    if(!Valid(center))return finish(TEXT("center_range"),false);
    details.EffectiveCenterZ=center;

    // Effective |theta|<.025 even with binary32 t*t and all native roundings.
    // For x=AngleSquared<.001, the existing checked trig ranges imply
    // |Scale|<.51, .99<Cosine<=1. Their phase accuracy is irrelevant to
    // D_yaw=(0,0,theta.Z*Scale,Cosine). Every such rotation about WORLD +Z
    // preserves the vertical component, including an inverted input Q0.
    //
    // With u=2^-53, the deviation of the native normalized direction from
    // unit(D_yaw*Q0) is bounded by
    // (0.52*offAxis+gamma(4)*1.03*|Q0|+u*1.03+flush)/.98.
    // This uses positive norms >.98, four Hamilton roundings per term, and
    // cancellation of the common inv-length scale in the last products.
    // The native normalized norm-squared lies in the existing Win64 range
    // of TightYawNormSquared, within 2^-49 of 1. For |x|<=112 the resulting
    // directional error (excluding off-axis) is <1.34e-13 cm, the radial
    // error <3.98e-13 cm. |q_native.Z/W|<2^-10 by yaw preserving the norm
    // of the ZW pair, plus this direction error. On the input guards above,
    // the actual vertical COM/offset/corner DAG, including COM translation,
    // has <=8 roundings per monomial and sum of absolute terms <274 cm,
    // i.e. <2.44e-13 cm. Their sum plus flush is below 2^-40 cm.
    //
    // For the ideal unit and raw families, the exact rotation-exponential
    // difference from the yaw projection is at most |x|*|theta_xy| by
    // Duhamel on skew matrices. The raw scale |Q0|^2 is covered as well.
    // Theta.IsZero returning Q0 is covered by the exact captured raw value.
    // 224*offAxis dominates both native and reference tilt effects.
    const auto error=Add(Interval::Exact(0x1p-40),
        Multiply(Interval::Exact(224.),Interval::Exact(offAxis.Upper)));
    if(!Valid(error))return finish(TEXT("error_range"),false);
    if(budget.Remaining-budget.ConsumptionFloor<32||MAX_uint64-budget.Used<32)
        return finish(TEXT("proof_budget"),false);
    for(int32 i=0;i<32;++i)if(!budget.Consume())return finish(TEXT("proof_budget"),false);

    Interval gaps[8];
    details.SmallestLower=std::numeric_limits<double>::max();
    for(int32 k=0;k<8;++k)
    {
        FIntervalPoint x;
        for(int32 d=0;d<3;++d)
            x.Axis[d]=Add(Subtract(Interval::Exact(input.LocalOffset[d]),Interval::Exact(input.LocalCOM[d])),
                Interval::Exact((k&(1<<d))?input.PhysicalHalfExtent[d]:-input.PhysicalHalfExtent[d]));
        const auto radius2=NormSquared(x);
        if(!Valid(radius2)||radius2.Upper>12544.)return finish(TEXT("physical_radius"),false);

        const auto twiceCross=Scale(Cross(qxyz,x),Interval::Exact(2.));
        const auto deltaZ=Sum(Scale(twiceCross,Interval::Exact(q.W)),Cross(qxyz,twiceCross)).Axis[2];
        const auto normalizedDelta=Hull(DividePositive(deltaZ,s0.Lower),DividePositive(deltaZ,s0.Upper));
        const auto q0Projection=Hull(Add(x.Axis[2],deltaZ),Add(x.Axis[2],normalizedDelta));
        gaps[k]=Add(Add(center,q0Projection),Interval{-error.Upper,error.Upper,true});
        if(!Valid(gaps[k]))return finish(TEXT("gap_range"),false);
        details.SmallestLower=FMath::Min(details.SmallestLower,gaps[k].Lower);
    }
    for(int32 k=0;k<8;++k)out[k]=gaps[k];
    return finish(TEXT("enclosed"),true);
#endif
}
}