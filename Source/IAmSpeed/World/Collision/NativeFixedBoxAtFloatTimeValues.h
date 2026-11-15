#pragma once
#include "NativeRetainedBoxReferenceFirstEntryValues.h"
namespace Speed::PreparedExecution
{
class FNativeFixedBoxAtFloatTimeValues;
using FNativeFixedBoxAtFloatTimeValuesPtr=TSharedPtr<const FNativeFixedBoxAtFloatTimeValues,ESPMode::ThreadSafe>;
// Actual fixed-Box floating DAG at ONE caller-provided float time, plus
// separate reference/consumed enclosures and differences at those SAME seconds.
// This does not SELECT a native collision time or prove a swept prefix clear.
// No native query-SAT/point-other/normal/error allowance/hit/response/Ready.
class FNativeFixedBoxAtFloatTimeValues final
{
    FNativeFixedBoxReferenceMotionValuesPtr Motion;float Seconds=0;
    Analytic::FAnalyticNumericEvidence Numeric;
    FKinematicState OwnerAtSeconds,BodyPoseAtSeconds;
    FVector PhysicalCorners[8];
    Analytic::Certified::FIntervalPoint ReferenceAtSeconds[8],ConsumedAtSeconds[8],NativeMinusReference[8];
    AnalyticCertified::Interval DifferenceSquared[8];
    FNativeFixedBoxAtFloatTimeValues()=default;
    static bool Contains(const Analytic::Certified::FIntervalPoint& bound,const FVector& point)
    {
        // Exact() encloses original subnormal input bits under DAZ/FTZ. Never
        // compare raw subnormal components as though they were literal zero.
        const auto p=Analytic::Certified::Detail::Point(point);
        if(!Analytic::Certified::Detail::ValidPoint(bound)||!Analytic::Certified::Detail::ValidPoint(p))return false;
        for(int32 d=0;d<3;++d)if(p.Axis[d].Lower<bound.Axis[d].Lower||p.Axis[d].Upper>bound.Axis[d].Upper)return false;
        return true;
    }
    static bool Finite(const FKinematicState& state)
    {
        return Analytic::Certified::Detail::ValidPoint(Analytic::Certified::Detail::Point(state.Location))&&
            Analytic::Certified::Detail::ValidPoint(Analytic::Certified::Detail::Point(state.Velocity))&&
            Analytic::Certified::Detail::ValidPoint(Analytic::Certified::Detail::Point(state.Acceleration))&&
            Analytic::Certified::Detail::ValidPoint(Analytic::Certified::Detail::Point(state.AngularVelocity))&&
            Analytic::Certified::Detail::ValidPoint(Analytic::Certified::Detail::Point(state.AngularAcceleration))&&!state.Rotation.ContainsNaN();
    }
public:
    static bool TryBuild(FNativeFixedBoxReferenceMotionValuesPtr motion,float seconds,
        Analytic::Certified::FDistanceBudget& budget,FNativeFixedBoxAtFloatTimeValuesPtr& out)
    {
        const auto keepPrior=out;out.Reset();
        const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        uint32 ordinal=0;
        if(!motion||!Analytic::Certified::FloorNativeSweepDetail::Ordinal(seconds,ordinal)||
            seconds>motion->GetOriginalHorizon()||!budget.Consume())return false;
        TSharedPtr<FNativeFixedBoxAtFloatTimeValues,ESPMode::ThreadSafe> value(new FNativeFixedBoxAtFloatTimeValues);
        value->Motion=MoveTemp(motion);value->Seconds=seconds;const auto& v=value->Motion->GetCapturedForInspection();
        // Refuse unavailable enclosures before executing the actual native DAG.
        for(int32 k=0;k<8;++k)
        {
            if(!budget.Consume())return false;
            value->ReferenceAtSeconds[k]=value->Motion->BoundPhysicalCornerAtFloatSeconds(k,seconds,false);
            value->ConsumedAtSeconds[k]=value->Motion->BoundPhysicalCornerAtFloatSeconds(k,seconds,true);
            if(!Analytic::Certified::Detail::ValidPoint(value->ReferenceAtSeconds[k])||
                !Analytic::Certified::Detail::ValidPoint(value->ConsumedAtSeconds[k]))return false;
        }
        // The REAL Speed::FKinematicState::Integrate / SBox DAG, including at
        // +0. No Quantize and no double(s)*double(s) replacement for float(s*s).
        value->OwnerAtSeconds=v.OwnerState.Integrate(seconds);value->BodyPoseAtSeconds=value->OwnerAtSeconds;
        auto& pose=value->BodyPoseAtSeconds;
        pose.Location-=pose.Rotation.RotateVector(v.LocalCOM);
        pose.Location+=pose.Rotation.RotateVector(v.LocalOffset);
        pose.Rotation=pose.Rotation*v.LocalRotation;
        if(!Finite(value->OwnerAtSeconds)||!Finite(pose)||
            !Analytic::Certified::FloorNativeSweepDetail::SameQuat(value->OwnerAtSeconds.Rotation,v.OwnerState.Rotation)||
            !Analytic::Certified::FloorNativeSweepDetail::SameQuat(pose.Rotation,v.BodyState.Rotation))return false;
        for(int32 k=0;k<8;++k)
        {
            if(!budget.Consume())return false;FVector local;
            for(int32 d=0;d<3;++d)local[d]=(k&(1<<d))?v.PhysicalHalfExtent[d]:-v.PhysicalHalfExtent[d];
            const FVector point=pose.Location+pose.Rotation.RotateVector(local);
            if(!Contains(value->ConsumedAtSeconds[k],point))return false;
            value->PhysicalCorners[k]=point;
            value->NativeMinusReference[k]=Analytic::Certified::ContactDetail::Subtract(
                Analytic::Certified::Detail::Point(point),value->ReferenceAtSeconds[k]);
            if(!Analytic::Certified::Detail::ValidPoint(value->NativeMinusReference[k]))return false;
            auto error=Analytic::Certified::ContactDetail::SquaredNorm(value->NativeMinusReference[k]);
            if(!AnalyticCertified::Valid(error))return false;
            error.Lower=FMath::Max(0.,error.Lower); // true squared norm is nonnegative.
            value->DifferenceSquared[k]=error;
        }
        const auto finalNumeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(finalNumeric.ControlFingerprint()!=numeric.ControlFingerprint())return false;
        value->Numeric=finalNumeric;out=MoveTemp(value);return true;
    }
    bool MatchesMotionAndSeconds(const FNativeFixedBoxReferenceMotionValuesPtr& motion,
        const FNativeSolidStaticValues& captured,float seconds) const
    {
        const auto numeric=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        return numeric.Status==Analytic::EAnalyticNumericStatus::Complete&&numeric.ControlFingerprint()==Numeric.ControlFingerprint()&&
            Motion&&Motion==motion&&Motion->MatchesCapturedValues(captured)&&
            Analytic::Certified::KinematicCenterDetail::Bits(Seconds)==Analytic::Certified::KinematicCenterDetail::Bits(seconds);
    }
    // Exact common Motion only: does NOT imply this float equals first contact.
    bool SharesMotionWithReferenceEntry(const FNativeRetainedBoxReferenceFirstEntryValuesPtr& reference) const
    {return Motion&&reference&&reference->GetPrefixForInspection()->GetCornerForInspection()->
        GetCornerReferenceForInspection()->GetMotionForInspection()==Motion;}
    const auto& GetNumericForInspection() const{return Numeric;}
    const auto& GetMotionForInspection() const{return Motion;}
    float GetFloatSeconds() const{return Seconds;}
    const auto& GetOwnerAtFloatTimeForInspection() const{return OwnerAtSeconds;}
    const auto& GetBodyPoseAtFloatTimeForInspection() const{return BodyPoseAtSeconds;}
    const auto& GetPhysicalCornersForInspection() const{return PhysicalCorners;}
    const auto& GetReferenceAtSameSecondsForInspection() const{return ReferenceAtSeconds;}
    const auto& GetConsumedAtSameSecondsForInspection() const{return ConsumedAtSeconds;}
    const auto& GetNativeMinusReferenceForInspection() const{return NativeMinusReference;}
    const auto& GetDifferenceSquaredForInspection() const{return DifferenceSquared;}
};
}
