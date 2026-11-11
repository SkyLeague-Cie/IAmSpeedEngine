#pragma once
#include "YawFloorBoxSweepBounds.h"
namespace Speed::Analytic::Certified
{
namespace YawFloorBoxDetail
{
inline bool TightYawNormSquared(const PreparedExecution::FNativeSolidStaticValues& v,Interval& output)
{
    output={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    // Exact outward rational enclosures of the CURRENT native Normalize
    // tolerance, not collision slop. A different constant requires a new proof.
    if(KinematicCenterDetail::Bits(float(UE_SMALL_NUMBER))!=0x322bcc77u)return false;
    FQuaternion raw;if(!FiniteYawIntegrator(v,&raw))return false;
    Interval squares[4];for(int32 d=0;d<4;++d)squares[d]=Square(raw.Axis[d].Range);
    // Close both actual double Dot4 association DAGs (FMA included by the
    // unfused overbound). Finite raw input alone does NOT rule out Dot4 Inf.
    const auto avx=Add(Add(squares[0],squares[1]),Add(squares[2],squares[3]));
    const auto sse=Add(Add(squares[0],squares[2]),Add(squares[1],squares[3]));
    if(!Valid(avx)||!Valid(sse))return false;
    // u=2^-53, mu=2^-1022, tau=double(UE_SMALL_NUMBER). Source rational proof:
    // L=(1-u)^4/(1+u)^10*(1-8mu/tau)-16mu
    // U=(1+u)^4/(1-u)^10*(1+8mu/tau)+16mu.
    // Dot4 positive reduction, double sqrt/divide and final multiplication;
    // 16mu also covers final component DAZ/FTZ loss. No naive double 1+u.
    const double lower=FromBits(0x3feffffffffffff2ULL),upper=FromBits(0x3ff0000000000008ULL);
    const auto& q=v.OwnerState.Rotation;
    const auto captured=Add(Square(Interval::Exact(q.Z)),Square(Interval::Exact(q.W)));
    if(!Valid(captured))return false;
    // Return-Q0 and Identity fallback are distinct from the normal branch.
    output={FMath::Min(captured.Lower,lower),FMath::Max(captured.Upper,upper),true};
    return Valid(output)&&output.Lower>0&&output.Upper<=2;
}
}
class FYawFloorBoxMotionModel
{
    FZeroOffsetKinematicCenterReference OwnerCOM;
    FVector P=FVector::ZeroVector,V=FVector::ZeroVector,A=FVector::ZeroVector;
    FIntervalPoint RotatedCOM,RotatedOffset,Corner[8],Captured[8];
    Interval NormSquared;float Horizon=0;int32 Side=0;bool Known=false;
    friend bool BuildYawFloorBoxMotionModel(const PreparedExecution::FNativeSolidStaticValues&,FYawFloorBoxMotionModel&);
public:
    bool IsKnown() const{return Known;}
    float GetHorizon() const{return Horizon;}
    Interval GetYawNormSquared() const{return NormSquared;}
    FIntervalPoint BoundCenter(float first,float last,bool consumed) const
    {
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        uint32 a,b;if(!Known||first>last||last>Horizon||!FloorNativeSweepDetail::Ordinal(first,a)||!FloorNativeSweepDetail::Ordinal(last,b))return {};
        FIntervalPoint center;
        if(consumed)center=BoundConsumedCenterBetweenFloatSeconds(OwnerCOM,first,last);
        else
        {
            const Interval time{double(first),double(last),true};const auto squared=AnalyticCertified::Square(time);
            for(int32 d=0;d<3;++d)center.Axis[d]=AnalyticCertified::Add(
                AnalyticCertified::Add(Interval::Exact(P[d]),AnalyticCertified::Multiply(Interval::Exact(V[d]),time)),
                AnalyticCertified::Multiply(AnalyticCertified::Multiply(Interval::Exact(.5),Interval::Exact(A[d])),squared));
        }
        for(int32 d=0;d<3;++d)center.Axis[d]=AnalyticCertified::Add(
            AnalyticCertified::Subtract(center.Axis[d],RotatedCOM.Axis[d]),RotatedOffset.Axis[d]);
        return Detail::ValidPoint(center)?center:FIntervalPoint{};
    }
    Interval BoundPhysicalCornerGap(int32 index,float first,float last,bool consumed) const
    {
        const auto center=BoundCenter(first,last,consumed);
        if(index<0||index>=8||!Detail::ValidPoint(center))return {};
        auto gap=AnalyticCertified::Multiply(Interval::Exact(double(Side)),
            AnalyticCertified::Add(center.Axis[2],Corner[index].Axis[2]));
        if(first==0)
        {
            const auto captured=AnalyticCertified::Multiply(Interval::Exact(double(Side)),Captured[index].Axis[2]);
            if(!AnalyticCertified::Valid(gap)||!AnalyticCertified::Valid(captured))return {};
            gap={FMath::Min(gap.Lower,captured.Lower),FMath::Max(gap.Upper,captured.Upper),true};
        }
        return gap;
    }
    Interval BoundGap(float first,float last,bool consumed) const
    {
        const auto center=BoundCenter(first,last,consumed);if(!Detail::ValidPoint(center))return {};
        const auto sign=Interval::Exact(double(Side));Interval gap,captured;
        for(int32 k=0;k<8;++k)
        {
            const auto g=AnalyticCertified::Multiply(sign,AnalyticCertified::Add(center.Axis[2],Corner[k].Axis[2]));
            gap=k?FloorNativeSweepDetail::Minimum(gap,g):g;
            const auto c=AnalyticCertified::Multiply(sign,Captured[k].Axis[2]);
            captured=k?FloorNativeSweepDetail::Minimum(captured,c):c;
        }
        if(!AnalyticCertified::Valid(gap)||!AnalyticCertified::Valid(captured))return {};
        // Union with the captured pose ONLY when zero belongs to this range.
        // An earlier captured gap cannot cap the upper gap at a later endpoint.
        if(first==0)gap={FMath::Min(gap.Lower,captured.Lower),FMath::Max(gap.Upper,captured.Upper),true};
        return gap;
    }
    FIntervalPoint BoundSupportHull(bool consumed=true) const
    {
        const auto center=BoundCenter(0,Horizon,consumed);if(!Detail::ValidPoint(center))return {};
        FIntervalPoint hull;
        for(int32 k=0;k<8;++k)
        {
            FIntervalPoint point;for(int32 d=0;d<3;++d)point.Axis[d]=AnalyticCertified::Add(center.Axis[d],Corner[k].Axis[d]);
            if(!YawFloorBoxDetail::Join(hull,point)||!YawFloorBoxDetail::Join(hull,Captured[k]))return {};
        }
        return hull;
    }
};
inline bool BuildYawFloorBoxMotionModel(const PreparedExecution::FNativeSolidStaticValues& v,FYawFloorBoxMotionModel& output)
{
    output={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    using namespace FloorNativeSweepDetail;
    if(v.Shape!=PreparedExecution::ENativeSolidQueryShape::Box||v.Frozen||v.Prescribed||
        v.OwnerState.Location.ContainsNaN()||v.BodyState.Location.ContainsNaN()||
        v.OwnerState.Rotation.ContainsNaN()||!v.OwnerState.Rotation.IsNormalized()||
        v.LocalRotation.ContainsNaN()||!v.LocalRotation.IsNormalized()||
        v.BodyState.Rotation.ContainsNaN()||!v.BodyState.Rotation.IsNormalized()||
        v.LocalCOM.ContainsNaN()||v.LocalOffset.ContainsNaN()||v.PhysicalHalfExtent.ContainsNaN()||v.PhysicalHalfExtent.GetMin()<=0||
        !SameQuat(v.BodyState.Rotation,v.OwnerState.Rotation*v.LocalRotation))return false;
    FYawFloorBoxMotionModel model;model.P=v.OwnerState.Location;model.V=v.OwnerState.Velocity;model.A=v.OwnerState.Acceleration;model.Horizon=v.Horizon;
    if(!BuildZeroOffsetKinematicCenterReference(model.P,model.V,model.A,v.Horizon,FVector::ZeroVector,FVector::ZeroVector,model.OwnerCOM)||
        !YawFloorBoxDetail::TightYawNormSquared(v,model.NormSquared))return false;
    const FVector center=(FVector(v.OwnerState.Location)-v.OwnerState.Rotation.RotateVector(v.LocalCOM))+
        v.OwnerState.Rotation.RotateVector(v.LocalOffset);
    if(center.ContainsNaN()||!SameVector(center,v.BodyState.Location)||!AnalyticCertified::Finite(center.Z)||center.Z==0)return false;
    model.RotatedCOM=YawFloorBoxDetail::OwnerYawOffset(v.LocalCOM);model.RotatedOffset=YawFloorBoxDetail::OwnerYawOffset(v.LocalOffset);
    if(!Detail::ValidPoint(model.RotatedCOM)||!Detail::ValidPoint(model.RotatedOffset))return false;
    for(int32 k=0;k<8;++k)
    {
        FVector corner;for(int32 d=0;d<3;++d)corner[d]=(k&(1<<d))?v.PhysicalHalfExtent[d]:-v.PhysicalHalfExtent[d];
        model.Corner[k]=YawFloorBoxDetail::BodyCorner(v.LocalRotation,corner,model.NormSquared);
        const auto rotated=FixedQueryBoundsDetail::Rotate(v.BodyState.Rotation,corner);
        for(int32 d=0;d<3;++d)model.Captured[k].Axis[d]=AnalyticCertified::Add(Interval::Exact(v.BodyState.Location[d]),rotated.Axis[d]);
        if(!Detail::ValidPoint(model.Corner[k])||!Detail::ValidPoint(model.Captured[k]))return false;
    }
    model.Side=center.Z>0?1:-1;model.Known=true;
    if(!Detail::ValidPoint(model.BoundSupportHull())||!Detail::ValidPoint(model.BoundSupportHull(false)))return false;
    output=MoveTemp(model);return true;
}
inline FFloorNativeSweepBounds BoundYawFloorBoxFirstArrival(const FYawFloorBoxMotionModel& model,FDistanceBudget& budget)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    FFloorNativeSweepBounds out;if(!model.IsKnown()||!budget.Consume())return out;
    const auto nativeHull=model.BoundSupportHull(),referenceHull=model.BoundSupportHull(false);
    if(!Detail::ValidPoint(nativeHull)||!Detail::ValidPoint(referenceHull))return out;
    auto clear=[](Interval gap){return AnalyticCertified::Valid(gap)&&gap.Lower>0;};
    auto inside=[](Interval gap){return AnalyticCertified::Valid(gap)&&gap.Upper<0;};
    if(!clear(model.BoundGap(0,0,true))||!clear(model.BoundGap(0,0,false)))return out;
    uint32 end;if(!FloorNativeSweepDetail::Ordinal(model.GetHorizon(),end)||!end)return out;
    struct FCell{uint32 First,Last;};TArray<FCell,TInlineAllocator<64>> stack;stack.Add({0,end});uint32 prefix=0;
    while(stack.Num())
    {
        if(!budget.Consume())return {};
        const auto cell=stack.Pop(EAllowShrinking::No);
        const float lo=FloorNativeSweepDetail::Seconds(cell.First),hi=FloorNativeSweepDetail::Seconds(cell.Last);
        const auto native=model.BoundGap(lo,hi,true),reference=model.BoundGap(lo,hi,false);
        if(!AnalyticCertified::Valid(native)||!AnalyticCertified::Valid(reference))return {};
        if(clear(native)&&clear(reference)){prefix=cell.Last;continue;}
        if(cell.Last-cell.First<=1)
        {
            // FIRST indeterminate leaf only. Never skip/anchor an uncertain
            // earlier tangent and later relabel another crossing as earliest.
            const auto lowNative=model.BoundGap(lo,lo,true),lowReference=model.BoundGap(lo,lo,false);
            if(prefix!=cell.First||!cell.First||!clear(lowNative)||!clear(lowReference)||
                !inside(model.BoundGap(hi,hi,true))||!inside(model.BoundGap(hi,hi,false)))return {};
            out.Status=EFloorNativeSweepStatus::ArrivalBracket;out.ClearPrefixSeconds=lo;out.InsideSeconds=hi;
            out.ConsumedGap=lowNative;out.ReferenceGap=lowReference;out.SupportHull=TimeDetail::Hull(nativeHull,referenceHull);
            return out; // no Box response margin or pre-created native hit
        }
        const uint32 middle=cell.First+(cell.Last-cell.First)/2;
        stack.Add({middle,cell.Last});stack.Add({cell.First,middle});
    }
    out.Status=EFloorNativeSweepStatus::Separated;out.ClearPrefixSeconds=model.GetHorizon();
    out.SupportHull=TimeDetail::Hull(nativeHull,referenceHull);return out;
}
}
