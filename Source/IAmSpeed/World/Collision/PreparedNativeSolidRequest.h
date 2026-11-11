#pragma once
#include "PreparedPhysicalExecution.h"
#include "IAmSpeed/Base/InheritedMotionPolicyValues91.h"
#include "CanonicalPreparedFrameClock.h"
#include "PreparedWorldSnapshot.h"
#include "PreparedNativeTraceValues.h"
class ISolidSweeper;class USolidSubBody;struct FCollisionShape;struct FCollisionQueryParams;
namespace Speed::PreparedExecution
{
enum class ENativeSolidQueryShape:uint8 {Unknown,Sphere,Box};
// Common static-response CONFIGURATION only. Pose-dependent World inertia,
// selected/current hits, constraint inventory, AcceptHit ignores/counters and
// derived wheel/Ball state must be joined at their actual ordinary consume.
struct FNativeSolidResponseConfigValues
{
    float BodyMass=0,OwnerPhysMass=0,Restitution=0,ImpactThreshold=0;
    float StaticFriction=0,DynamicFriction=0,HitDamping=0,OwnerMaxSpeed=0,OwnerMaxAngularSpeed=0;
    FMatrix CachedBodyLocalInvInertia=FMatrix::Identity;
    bool MainSubBody=false,ApplyRestForce=false,AerialShotContactModel=false;
    bool SphereProjectionPolicyCaptured=false,SphereWorldStaticProjectionEnabled=false;
};
IAMSPEED_API bool SameNativeSolidResponseConfigValues(const FNativeSolidResponseConfigValues&,
    const FNativeSolidResponseConfigValues&) noexcept;
struct FNativeSolidStaticValues
{
    const ISolidSweeper* Sweeper=nullptr;const USolidSubBody* Body=nullptr;const ISpeedComponent* Owner=nullptr;
    const void* BodyClass=nullptr;uint64 OwnerStableId=0;int32 BodyIndex=INDEX_NONE;
    ENativeSolidQueryShape Shape=ENativeSolidQueryShape::Unknown;FCanonicalPreparedFrameClock FrameClock;
    FKinematicState OwnerState,BodyState;
    FInheritedMotionPolicyValues91 InheritedMotionPolicy91;
    FVector LocalOffset=FVector::ZeroVector,LocalCOM=FVector::ZeroVector;FQuat LocalRotation=FQuat::Identity;
    FVector Start=FVector::ZeroVector,End=FVector::ZeroVector;float Horizon=0;
    bool Frozen=false,Prescribed=false;
    // Query extents contain the actual FCollisionShape rounded values; physical
    // extents retain UBoxSubBody's independent double getter. No equality or
    // Sphere margin is inferred for Box.
    FVector QueryHalfExtent=FVector::ZeroVector,PhysicalHalfExtent=FVector::ZeroVector;
    float QueryRadius=0,PhysicalRadius=0;double SupportRadius=0;float StaticContactOffset=0,StaticImpactFriction=0;
    bool SphereMarginCaptured=false;float SphereMargin=0;uint64 SphereMemberSerial=0;
    Analytic::FWorldQuery Query;FNativeSolidTraceValues Trace;
    FNativeSolidResponseConfigValues ResponseConfig;
};
IAMSPEED_API bool SameNativeSolidStaticValues(const FNativeSolidStaticValues&,const FNativeSolidStaticValues&) noexcept;
IAMSPEED_API bool CaptureNativeSolidTraceValues(const ISolidSweeper&,const FCollisionQueryParams&,FNativeSolidTraceValues&);
class FNativeSolidStaticRequest;
using FNativeSolidStaticRequestPtr=TSharedPtr<const FNativeSolidStaticRequest,ESPMode::ThreadSafe>;
IAMSPEED_API bool CaptureNativeSolidCCD(const UWorld*,const ISolidSweeper*,const USolidSubBody*,FVector,FVector,
    const FCollisionShape&,const FCollisionQueryParams&,float,FNativeSolidStaticRequestPtr&);
IAMSPEED_API bool RevalidateNativeSolidCCD(const FNativeSolidStaticRequest&) noexcept;
// One actual ordinary FullFrame component/body World call. Immutable inputs
// ONLY: no CCD proof, whole Ctoi/cache/dynamic result, Contact, event, response,
// advance or Ready follows. Terminal diagnostic purposes cannot mint it.
class FNativeSolidStaticRequest final
{
    FStepIdentity Step;FNativeSolidStaticValues Values;FPreparedWorldSnapshotPtr Snapshot;
    FNativeSolidStaticRequest()=default;
    friend IAMSPEED_API bool CaptureNativeSolidCCD(const UWorld*,const ISolidSweeper*,const USolidSubBody*,FVector,FVector,
        const FCollisionShape&,const FCollisionQueryParams&,float,FNativeSolidStaticRequestPtr&);
    friend IAMSPEED_API bool RevalidateNativeSolidCCD(const FNativeSolidStaticRequest&) noexcept;
public:
    const FStepIdentity& GetStep() const {return Step;}
    const FNativeSolidStaticValues& GetValues() const {return Values;}
    const FPreparedWorldSnapshotPtr& GetSnapshot() const {return Snapshot;}
};
}
