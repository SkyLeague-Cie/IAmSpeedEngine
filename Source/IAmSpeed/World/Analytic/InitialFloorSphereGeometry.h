#pragma once
// Captured-pose geometry only. No native/World/response authority.
#include "FloorNativeSolidSweepBounds.h"
namespace Speed::Analytic::Certified
{
enum class EInitialFloorSphereSide:uint8 {Unknown,Separated,ExactTouch,Overlap};
struct FInitialFloorSphereGeometry
{
    EInitialFloorSphereSide SupportSide=EInitialFloorSphereSide::Unknown;
    EInitialFloorSphereSide PhysicalSide=EInitialFloorSphereSide::Unknown;
    Interval SupportGap,PhysicalGap;
    FVector CapturedCenter=FVector::ZeroVector,PlaneWitness=FVector::ZeroVector,SideNormal=FVector::ZeroVector;
    // Positive gap within a captured numerical budget NEVER creates a hit.
    bool PositiveSupportGapWithinCapturedMargin=false;
};
inline FInitialFloorSphereGeometry ClassifyCapturedFloorSphereGeometry(
    const PreparedExecution::FNativeSolidStaticValues& values)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    using namespace FloorNativeSweepDetail;using PreparedExecution::ENativeSolidQueryShape;
    FInitialFloorSphereGeometry out;
    if(values.Shape!=ENativeSolidQueryShape::Sphere||values.Frozen||values.Prescribed||
        !ZeroVector(values.LocalOffset)||!ZeroVector(values.LocalCOM)||
        !SameVector(values.OwnerState.Location,values.BodyState.Location)||
        !SameVector(values.OwnerState.Velocity,values.BodyState.Velocity)||
        !SameVector(values.OwnerState.Acceleration,values.BodyState.Acceleration)||
        values.BodyState.Location.ContainsNaN()||!values.SphereMarginCaptured||
        !PreparedExecution::IsSupportedSphereResponseMargin(values.SphereMargin))return out;
    const auto supportBits=AnalyticCertified::Bits(values.SupportRadius);
    uint32 physicalBits=0;FMemory::Memcpy(&physicalBits,&values.PhysicalRadius,4);
    if((supportBits&AnalyticCertified::Sign)||supportBits<AnalyticCertified::MinNormal||supportBits>=AnalyticCertified::Infinity||
        (physicalBits&KinematicCenterDetail::FloatSign)||physicalBits<KinematicCenterDetail::FloatMinNormal||
        physicalBits>=KinematicCenterDetail::FloatInfinity||
        !PreparedExecution::IsSupportedSphereResponseMargin(values.StaticContactOffset))return out;
    const double rp=double(values.PhysicalRadius);
    // Same actual native addition as the captured support-radius getter. This
    // does not infer either radius or the offset from rounded query Rc.
    volatile double expectedSupport=rp+double(values.StaticContactOffset);
    if(AnalyticCertified::Bits(expectedSupport)!=supportBits||values.SupportRadius<rp)return out;
    const uint64 centerBits=AnalyticCertified::Bits(values.BodyState.Location.Z);
    const uint64 centerMagnitude=centerBits&AnalyticCertified::Magnitude;
    // The exactly-on-plane center has no unique two-sided MTD normal. Its
    // overlap operation must choose/authenticate that branch separately.
    if(!centerMagnitude||centerMagnitude>=AnalyticCertified::Infinity)return out;
    auto classify=[centerMagnitude](double radius,Interval& gap)
    {
        const uint64 r=AnalyticCertified::Bits(radius);
        if(centerMagnitude==r){gap=Interval::Exact(0.);return EInitialFloorSphereSide::ExactTouch;}
        gap=AnalyticCertified::Subtract(Interval::Exact(AnalyticCertified::FromBits(centerMagnitude)),Interval::Exact(radius));
        if(!AnalyticCertified::Valid(gap))return EInitialFloorSphereSide::Unknown;
        // Integer magnitude order proves the sign even if outward bounds cross
        // zero at the smallest normal/subnormal scale; no DAZ comparison used.
        return centerMagnitude>r?EInitialFloorSphereSide::Separated:EInitialFloorSphereSide::Overlap;
    };
    out.SupportSide=classify(values.SupportRadius,out.SupportGap);out.PhysicalSide=classify(rp,out.PhysicalGap);
    if(out.SupportSide==EInitialFloorSphereSide::Unknown||out.PhysicalSide==EInitialFloorSphereSide::Unknown)return {};
    out.CapturedCenter=values.BodyState.Location;
    out.PlaneWitness=FVector(out.CapturedCenter.X,out.CapturedCenter.Y,0.);
    out.SideNormal=FVector(0,0,(centerBits&AnalyticCertified::Sign)?-1.:1.);
    out.PositiveSupportGapWithinCapturedMargin=out.SupportSide==EInitialFloorSphereSide::Separated&&
        out.SupportGap.Upper<=double(values.SphereMargin);
    // No Integrate(0), manufactured time, penetration depth, response arm,
    // identity/material, Contact or capability. Rp skin overlap and physical
    // penetration remain distinct. A consumer must bind the finite canonical
    // floor + all providers/filter/ignore to this SAME live request/serial and
    // authenticate the chosen normal, ordinary resolve/projection/Ball hooks.
    return out;
}
}
