#pragma once
#include "PreparedNativeSolidRequest.h"
#include "IAmSpeed/World/Analytic/InitialFloorSphereGeometry.h"
namespace Speed::PreparedExecution
{
// Actual native branch values, copied by its real game-specific implementation.
// Serialized manifold and transient force-sleep state stay distinct. This is
// neither a query nor an assertion of tangency/nonpenetration/force coverage.
struct FNativeSleepingSupportDecisionValues
{
    const ISpeedComponent* MovementOwner=nullptr;
    bool HasSupport=false,Equilibrium=false,ForceSleeping=false,Decision=false;uint8 StableFrames=0;
    FVector StoredNormal=FVector::ZeroVector,StoredPoint=FVector::ZeroVector,BranchNormal=FVector::ZeroVector;
    FVector COMVelocity=FVector::ZeroVector,TangentialVelocity=FVector::ZeroVector;
    float NormalSpeed=0,NativeWakeSpeed=0;
};
struct FSerializedSphereSupportManifold
{FVector Normal=FVector::ZeroVector,Point=FVector::ZeroVector;};
// Conditional geometry values ONLY. There is deliberately no FWorldHit/TOI,
// native query request, Contact, MTD, force or advance capability here.
struct FSerializedSphereSupportEnvelope
{
    Analytic::Certified::FIntervalPoint Hull;
    Analytic::Certified::FInitialFloorSphereGeometry Initial;
    Analytic::Certified::Interval NativeSupportGap,ReferenceSupportGap;
};
inline bool BuildSerializedSphereSupportEnvelope(const FNativeSolidStaticValues& v,
    const FSerializedSphereSupportManifold& manifold,FSerializedSphereSupportEnvelope& output)
{
    output={};using namespace Analytic::Certified;
    Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(v.Shape!=ENativeSolidQueryShape::Sphere||manifold.Point.ContainsNaN()||manifold.Normal.ContainsNaN())return false;
    // Exact coordinate-plane join, including signed zero. DAZ must not turn a
    // tiny nonzero normal/height into an invented plane or an invented axis.
    if((AnalyticCertified::Bits(manifold.Point.Z)&AnalyticCertified::Magnitude)||
        (AnalyticCertified::Bits(manifold.Normal.X)&AnalyticCertified::Magnitude)||
        (AnalyticCertified::Bits(manifold.Normal.Y)&AnalyticCertified::Magnitude)||
        (AnalyticCertified::Bits(manifold.Normal.Z)&AnalyticCertified::Magnitude)!=AnalyticCertified::Bits(1.))return false;
    const auto initial=ClassifyCapturedFloorSphereGeometry(v);
    if(initial.SupportSide==EInitialFloorSphereSide::Unknown||initial.SideNormal.Z!=manifold.Normal.Z)return false;
    uint32 radiusBits=0;FMemory::Memcpy(&radiusBits,&v.QueryRadius,4);
    if(radiusBits<0x00800000u||radiusBits>=0x7f800000u)return false;
    FFloorNativeSolidModel model;if(!BuildFloorNativeSolidModel(v,model))return false;
    const auto native=model.BoundCenter(0.f,v.Horizon,true),reference=model.BoundCenter(0.f,v.Horizon,false);
    if(!Detail::ValidPoint(native)||!Detail::ValidPoint(reference))return false;
    auto hull=TimeDetail::Hull(TimeDetail::Hull(native,reference),Detail::Point(v.BodyState.Location));
    const double radius=FMath::Max(v.SupportRadius,double(v.QueryRadius));
    for(int32 d=0;d<3;++d)hull.Axis[d]=AnalyticCertified::Add(hull.Axis[d],Interval{-radius,radius,true});
    const auto nativeGap=model.BoundGap(0.f,v.Horizon,true),referenceGap=model.BoundGap(0.f,v.Horizon,false);
    if(!Detail::ValidPoint(hull)||!AnalyticCertified::Valid(nativeGap)||!AnalyticCertified::Valid(referenceGap))return false;
    // Gaps, including positive gaps admitted by a native support slop, remain
    // measured intervals. Presence of a serialized plane NEVER makes them 0.
    output.Hull=hull;output.Initial=initial;output.NativeSupportGap=nativeGap;output.ReferenceSupportGap=referenceGap;return true;
}
struct FSerializedSphereSupportGeometryValues
{
    FSerializedSphereSupportEnvelope Envelope;
    uint64 SourceId=0,SurfaceId=0,FeatureId=0,PrimitiveId=0,CanonicalGroupId=0;uint32 MaterialId=0;
};
}
