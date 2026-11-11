#pragma once
#include "KinematicCenterPathBounds.h"
#include "FixedShapeQueryBounds.h"
#include "IAmSpeed/World/Collision/PreparedNativeSolidRequest.h"
namespace Speed::Analytic::Certified
{
// Pure geometry for the coordinate floor z=0. It carries no World/filter,
// finite-domain, Contact, native operation, event, response or advance authority.
// Sphere uses captured Rs, NOT rounded query Rc or physical response Rp.
// Box presently supports fixed rotation only; rotating motion is Unknown.
enum class EFloorNativeSweepStatus:uint8 {Unknown,Separated,ArrivalBracket};
struct FFloorNativeSweepBounds
{
    EFloorNativeSweepStatus Status=EFloorNativeSweepStatus::Unknown;
    float ClearPrefixSeconds=0,InsideSeconds=0;
    Interval ConsumedGap,ReferenceGap;
    FIntervalPoint SupportHull;
    bool CapturedSphereGapSatisfied=false;
};
namespace FloorNativeSweepDetail
{
inline bool SameVector(const FVector& a,const FVector& b)
{
    for(int32 d=0;d<3;++d)if(AnalyticCertified::Bits(a[d])!=AnalyticCertified::Bits(b[d]))return false;
    return true;
}
inline bool ZeroVector(const FVector& a)
{
    for(int32 d=0;d<3;++d)if(AnalyticCertified::Bits(a[d])&AnalyticCertified::Magnitude)return false;
    return true;
}
inline bool SameQuat(const FQuat& a,const FQuat& b)
{
    return AnalyticCertified::Bits(a.X)==AnalyticCertified::Bits(b.X)&&AnalyticCertified::Bits(a.Y)==AnalyticCertified::Bits(b.Y)&&
        AnalyticCertified::Bits(a.Z)==AnalyticCertified::Bits(b.Z)&&AnalyticCertified::Bits(a.W)==AnalyticCertified::Bits(b.W);
}
// Ordinals deliberately omit subnormal times: the numeric contract permits
// flushing them. Zero is +0, and no seconds/Delta round trip occurs here.
inline bool Ordinal(float seconds,uint32& out)
{
    const uint32 bits=KinematicCenterDetail::Bits(seconds);
    if(bits==0){out=0;return true;}
    if(bits<KinematicCenterDetail::FloatMinNormal||bits>=KinematicCenterDetail::FloatInfinity)return false;
    out=bits-KinematicCenterDetail::FloatMinNormal+1;return true;
}
inline float Seconds(uint32 ordinal)
{
    return ordinal?KinematicCenterDetail::FromBits(ordinal+KinematicCenterDetail::FloatMinNormal-1):0.f;
}
inline Interval Minimum(Interval a,Interval b)
{
    if(!AnalyticCertified::Valid(a)||!AnalyticCertified::Valid(b))return {};
    return {FMath::Min(a.Lower,b.Lower),FMath::Min(a.Upper,b.Upper),true};
}
}
class FFloorNativeSolidModel
{
    FZeroOffsetKinematicCenterReference OwnerCOM;
    FVector P=FVector::ZeroVector,V=FVector::ZeroVector,A=FVector::ZeroVector;
    FIntervalPoint RotatedCOM,RotatedOffset,Corner[8];
    float Horizon=0;double Radius=0,GapBudget=0,CeilingZ=0;int32 Side=0;bool Box=false,Known=false;
    friend bool BuildFloorNativeSolidModel(const PreparedExecution::FNativeSolidStaticValues&,FFloorNativeSolidModel&);
    friend bool BuildNativeCeilingSphereModel(const PreparedExecution::FNativeSolidStaticValues&,double,FFloorNativeSolidModel&);
public:
    bool IsKnown() const {return Known;}
    float GetHorizon() const {return Horizon;}
    double GetCapturedGapBudget() const {return GapBudget;}
    bool IsFixedBox() const {return Known&&Box;}
    // Direct OWNER COM, before the Box's -RotatedCOM/+RotatedOffset DAG.
    // Every admitted original float in [first,last] is enclosed. This is the
    // unconstrained model Integrate path, not a live post-advance GetPhysCOM.
    FIntervalPoint BoundConsumedOwnerCOMBetweenFloatSeconds(float first,float last) const
    {
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        uint32 a=0,b=0;
        if(!Known||!FloorNativeSweepDetail::Ordinal(first,a)||!FloorNativeSweepDetail::Ordinal(last,b)||
            a>b||last>Horizon)return {};
        const auto center=BoundConsumedCenterBetweenFloatSeconds(OwnerCOM,first,last);
        return Detail::ValidPoint(center)?center:FIntervalPoint{};
    }
    // Reference math and actual float(s*s) consumed DAG are bounded separately.
    FIntervalPoint BoundCenter(float first,float last,bool consumed) const
    {
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!Known||first>last||last>Horizon)return {};
        uint32 a,b;if(!FloorNativeSweepDetail::Ordinal(first,a)||!FloorNativeSweepDetail::Ordinal(last,b))return {};
        FIntervalPoint center;
        if(consumed)center=BoundConsumedCenterBetweenFloatSeconds(OwnerCOM,first,last);
        else
        {
            const Interval seconds={double(first),double(last),true};
            const auto squared=AnalyticCertified::Square(seconds);
            for(int32 d=0;d<3;++d)center.Axis[d]=AnalyticCertified::Add(
                AnalyticCertified::Add(Interval::Exact(P[d]),AnalyticCertified::Multiply(Interval::Exact(V[d]),seconds)),
                AnalyticCertified::Multiply(AnalyticCertified::Multiply(Interval::Exact(0.5),Interval::Exact(A[d])),squared));
        }
        if(Box)for(int32 d=0;d<3;++d)center.Axis[d]=AnalyticCertified::Add(
            AnalyticCertified::Subtract(center.Axis[d],RotatedCOM.Axis[d]),RotatedOffset.Axis[d]);
        return Detail::ValidPoint(center)?center:FIntervalPoint{};
    }
    // The same physical corner DAG, without choosing a coordinate plane.
    // This carries no finite-domain, Contact, World or response authority.
    FIntervalPoint BoundPhysicalCorner(int32 index,float first,float last,bool consumed) const
    {
        auto center=BoundCenter(first,last,consumed);
        if(!Known||!Box||index<0||index>=8||!Detail::ValidPoint(center))return {};
        for(int32 d=0;d<3;++d)center.Axis[d]=AnalyticCertified::Add(center.Axis[d],Corner[index].Axis[d]);
        return Detail::ValidPoint(center)?center:FIntervalPoint{};
    }
    // Reuse the SAME owner-COM reference polynomial for every fixed corner.
    // Interval CP copies overapproximate correlations, but provenance remains
    // one common COM curve plus the same fixed COM/local/corner offsets.
    bool BuildReferenceCornerPath(int32 index,FIntervalBezierCenterPath& output) const
    {
        output={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!Known||!Box||index<0||index>=8)return false;
        auto path=OwnerCOM.GetReferenceCurve();
        if(!CenterDetail::Valid(path)||path.Degree!=2)return false;
        for(auto& cp:path.ControlPoints)for(int32 d=0;d<3;++d)
            cp.Axis[d]=AnalyticCertified::Add(AnalyticCertified::Add(
                AnalyticCertified::Subtract(cp.Axis[d],RotatedCOM.Axis[d]),RotatedOffset.Axis[d]),Corner[index].Axis[d]);
        if(!CenterDetail::Valid(path))return false;output=MoveTemp(path);return true;
    }
    FIntervalPoint BoundPhysicalCornerOffset(int32 index) const
    {
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        return Known&&Box&&index>=0&&index<8?Corner[index]:FIntervalPoint{};
    }
    const FIntervalBezierCenterPath& GetOwnerReferenceCurveForInspection() const{return OwnerCOM.GetReferenceCurve();}
    Interval BoundPhysicalCornerGap(int32 index,float first,float last,bool consumed) const
    {
        const auto center=BoundCenter(first,last,consumed);
        if(!Known||!Box||index<0||index>=8||!Detail::ValidPoint(center))return {};
        return AnalyticCertified::Multiply(Interval::Exact(double(Side)),
            AnalyticCertified::Add(center.Axis[2],Corner[index].Axis[2]));
    }
    Interval BoundGap(float first,float last,bool consumed) const
    {
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        const auto center=BoundCenter(first,last,consumed);if(!Detail::ValidPoint(center))return {};
        const auto sign=Interval::Exact(double(Side));
        if(!Box)
        {
            // Bound the original consumed and reference world COM first. An
            // origin shift in the motion coefficients changes native rounding.
            const auto height=CeilingZ>0?
                AnalyticCertified::Subtract(center.Axis[2],Interval::Exact(CeilingZ)):center.Axis[2];
            return AnalyticCertified::Subtract(AnalyticCertified::Multiply(sign,height),Interval::Exact(Radius));
        }
        Interval result;
        for(int32 k=0;k<8;++k)
        {
            const auto gap=AnalyticCertified::Multiply(sign,AnalyticCertified::Add(center.Axis[2],Corner[k].Axis[2]));
            result=k?FloorNativeSweepDetail::Minimum(result,gap):gap;
        }
        return result;
    }
    // Physical fixed Box corners / Sphere Rs hull only. Rc and rounded SAT axes
    // need their own query envelope; this is not a finite-world miss proof.
    FIntervalPoint BoundSupportHull(bool consumed=true) const
    {
        auto center=BoundCenter(0.f,Horizon,consumed);if(!Detail::ValidPoint(center))return {};
        if(!Box)
        {
            for(int32 d=0;d<3;++d)center.Axis[d]=AnalyticCertified::Add(center.Axis[d],Interval{-Radius,Radius,true});
            return center;
        }
        FIntervalPoint out;
        for(int32 k=0;k<8;++k)
        {
            FIntervalPoint point;for(int32 d=0;d<3;++d)point.Axis[d]=AnalyticCertified::Add(center.Axis[d],Corner[k].Axis[d]);
            out=k?TimeDetail::Hull(out,point):point;
        }
        return Detail::ValidPoint(out)?out:FIntervalPoint{};
    }
};
inline bool BuildFloorNativeSolidModel(const PreparedExecution::FNativeSolidStaticValues& values,FFloorNativeSolidModel& out)
{
    out={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    using namespace FloorNativeSweepDetail;using PreparedExecution::ENativeSolidQueryShape;
    if(values.Frozen||values.Prescribed||!SameVector(values.OwnerState.Velocity,values.BodyState.Velocity)||
        !SameVector(values.OwnerState.Acceleration,values.BodyState.Acceleration))return false;
    FFloorNativeSolidModel model;model.P=values.OwnerState.Location;model.V=values.OwnerState.Velocity;
    model.A=values.OwnerState.Acceleration;model.Horizon=values.Horizon;
    // This zero-offset helper models the OWNER COM only. Box offsets are then
    // applied in the actual origin subtraction / local-offset addition order.
    if(!BuildZeroOffsetKinematicCenterReference(model.P,model.V,model.A,model.Horizon,
        FVector::ZeroVector,FVector::ZeroVector,model.OwnerCOM))return false;
    if(values.Shape==ENativeSolidQueryShape::Sphere)
    {
        if(!ZeroVector(values.LocalOffset)||!ZeroVector(values.LocalCOM)||!SameVector(values.OwnerState.Location,values.BodyState.Location)||
            !AnalyticCertified::Finite(values.SupportRadius)||values.SupportRadius<=0||!values.SphereMarginCaptured||
            !PreparedExecution::IsSupportedSphereResponseMargin(values.SphereMargin))return false;
        model.Radius=values.SupportRadius;model.GapBudget=double(values.SphereMargin);
    }
    else if(values.Shape==ENativeSolidQueryShape::Box)
    {
        if(!ZeroVector(values.OwnerState.AngularVelocity)||!ZeroVector(values.OwnerState.AngularAcceleration)||
            !ZeroVector(values.BodyState.AngularVelocity)||!ZeroVector(values.BodyState.AngularAcceleration)||
            values.OwnerState.Rotation.ContainsNaN()||!values.OwnerState.Rotation.IsNormalized()||
            values.LocalRotation.ContainsNaN()||!values.LocalRotation.IsNormalized()||
            values.BodyState.Rotation.ContainsNaN()||!values.BodyState.Rotation.IsNormalized()||
            !SameQuat(values.BodyState.Rotation,values.OwnerState.Rotation*values.LocalRotation)||
            values.PhysicalHalfExtent.ContainsNaN()||values.PhysicalHalfExtent.GetMin()<=0)return false;
        const FVector rotatedCOM=values.OwnerState.Rotation.RotateVector(values.LocalCOM);
        const FVector rotatedOffset=values.OwnerState.Rotation.RotateVector(values.LocalOffset);
        if(!SameVector((FVector(values.OwnerState.Location)-rotatedCOM)+rotatedOffset,values.BodyState.Location))return false;
        model.Box=true;model.RotatedCOM=FixedQueryBoundsDetail::Rotate(values.OwnerState.Rotation,values.LocalCOM);
        model.RotatedOffset=FixedQueryBoundsDetail::Rotate(values.OwnerState.Rotation,values.LocalOffset);
        if(!Detail::ValidPoint(model.RotatedCOM)||!Detail::ValidPoint(model.RotatedOffset))return false;
        for(int32 k=0;k<8;++k)
        {
            FVector corner;for(int32 d=0;d<3;++d)corner[d]=(k&(1<<d))?values.PhysicalHalfExtent[d]:-values.PhysicalHalfExtent[d];
            model.Corner[k]=FixedQueryBoundsDetail::Rotate(values.BodyState.Rotation,corner);
            if(!Detail::ValidPoint(model.Corner[k]))return false;
        }
        // There is no Sphere CollisionMargin on Box. No response gap invented.
        model.GapBudget=0;
    }
    else return false;
    const double startZ=values.BodyState.Location.Z;
    if(!AnalyticCertified::Finite(startZ)||startZ==0)return false;
    model.Side=startZ>0?1:-1;model.Known=true;
    if(!Detail::ValidPoint(model.BoundSupportHull()))return false;
    out=MoveTemp(model);return true;
}
inline bool BuildNativeCeilingSphereModel(const PreparedExecution::FNativeSolidStaticValues& values,
    double ceilingZ,FFloorNativeSolidModel& out)
{
    out={};
    if(values.Shape!=PreparedExecution::ENativeSolidQueryShape::Sphere||
        !AnalyticCertified::Finite(ceilingZ)||ceilingZ<=0||!(values.BodyState.Location.Z<ceilingZ)||
        !BuildFloorNativeSolidModel(values,out))return false;
    out.CeilingZ=ceilingZ;out.Side=-1;return true;
}
inline FFloorNativeSweepBounds BoundFloorNativeFirstArrival(const FFloorNativeSolidModel& model,FDistanceBudget& budget)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    FFloorNativeSweepBounds out;if(!model.IsKnown())return out;
    const auto hull=model.BoundSupportHull();if(!Detail::ValidPoint(hull)||!budget.Consume())return out;
    auto clear=[](Interval gap){return AnalyticCertified::Valid(gap)&&gap.Lower>0;};
    auto inside=[](Interval gap){return AnalyticCertified::Valid(gap)&&gap.Upper<0;};
    const auto initialNative=model.BoundGap(0,0,true),initialReference=model.BoundGap(0,0,false);
    // Initial support / MTD has a different operation contract, still required.
    if(!clear(initialNative)||!clear(initialReference))return out;
    struct FCell {uint32 First,Last;};TArray<FCell,TInlineAllocator<64>> stack;
    uint32 end;if(!FloorNativeSweepDetail::Ordinal(model.GetHorizon(),end)||end==0)return out;
    stack.Add({0,end});float prefix=0,anchor=0;bool anchored=false;
    while(stack.Num())
    {
        if(!budget.Consume())return {};
        const auto cell=stack.Pop(EAllowShrinking::No);
        const float lo=FloorNativeSweepDetail::Seconds(cell.First),hi=FloorNativeSweepDetail::Seconds(cell.Last);
        const auto native=model.BoundGap(lo,hi,true),reference=model.BoundGap(lo,hi,false);
        if(!AnalyticCertified::Valid(native)||!AnalyticCertified::Valid(reference))return {};
        if(clear(native)&&clear(reference)){if(!anchored)prefix=hi;continue;}
        const float candidate=anchored?anchor:lo;
        const auto lowNative=model.BoundGap(candidate,candidate,true),lowReference=model.BoundGap(candidate,candidate,false);
        const auto highNative=model.BoundGap(hi,hi,true),highReference=model.BoundGap(hi,hi,false);
        const bool sphereGap=!model.IsFixedBox()&&clear(lowNative)&&clear(lowReference)&&
            lowNative.Upper<=model.GetCapturedGapBudget()&&lowReference.Upper<=model.GetCapturedGapBudget();
        const bool boxGeometricLeaf=model.IsFixedBox()&&cell.Last-cell.First<=1;
        if(clear(lowNative)&&clear(lowReference)&&((sphereGap&&candidate>0.f)||boxGeometricLeaf)&&inside(highNative)&&inside(highReference))
        {
            // Earlier indeterminate leaves remain INSIDE this bracket; a later
            // inside endpoint never erases an earlier tangent/arrival candidate.
            out.Status=EFloorNativeSweepStatus::ArrivalBracket;out.ClearPrefixSeconds=candidate;
            out.InsideSeconds=hi;out.ConsumedGap=lowNative;out.ReferenceGap=lowReference;out.SupportHull=hull;
            // Box has a GEOMETRIC bracket only. Its positive precontact gap is
            // not converted into a native zero-gap contact or response budget.
            out.CapturedSphereGapSatisfied=sphereGap;return out;
        }
        if(cell.Last-cell.First<=1)
        {
            if(!anchored){anchor=prefix;anchored=true;}
            continue;
        }
        const uint32 middle=cell.First+(cell.Last-cell.First)/2;
        stack.Add({middle,cell.Last});stack.Add({cell.First,middle});
    }
    if(anchored)return {};
    out.Status=EFloorNativeSweepStatus::Separated;out.ClearPrefixSeconds=model.GetHorizon();out.SupportHull=hull;return out;
}
}
