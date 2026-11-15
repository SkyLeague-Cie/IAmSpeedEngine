#pragma once
#include "PreparedPhysicalExecution.h"
#include "CanonicalPreparedFrameClock.h"
#include "PreparedGeometrySelection.h"
#include "IAmSpeed/Base/Kinematic.h"
#include "IAmSpeed/World/Analytic/KinematicCenterPathBounds.h"
#include "IAmSpeed/World/Analytic/AnalyticWorldQuery.h"
class USolidSubBody;class USphereSubBody;struct FCollisionShape;
namespace Speed::PreparedExecution
{
IAMSPEED_API bool SameKinematicBits(const FKinematicState& A,const FKinematicState& B) noexcept;
struct FSphereValues
{
    const void* Owner=nullptr;const USphereSubBody* Body=nullptr;const void* BodyClass=nullptr;
    uint64 OwnerStableId=0;int32 BodyIndex=INDEX_NONE;
    FCanonicalPreparedFrameClock FrameClock; // retain BOTH actual local and canonical identities
    FKinematicState OwnerState,BodyState;
    FVector LocalOffset=FVector::ZeroVector,LocalCOM=FVector::ZeroVector;FQuat LocalRotation=FQuat::Identity;
    FVector Start=FVector::ZeroVector,End=FVector::ZeroVector;
    float Horizon=0,CollisionRadius=0,PhysicalRadius=0;double RadiusInflation=0;
    // Distinct actual getter values, never inferred from Rc-Rp.
    double StaticSupportRadius=0;float StaticContactOffsetCm=0,StaticImpactFriction=0;
    bool Frozen=false,Prescribed=false;
    // The actual protected virtual margin is copied within the native sphere
    // member scope and bit-rechecked there after ground sweep. Scope serial is
    // monotone; closure revokes raw captures even if stack/PVA are reused.
    // RadiusInflation remains descriptive, never a response budget.
    bool ResponseMarginCaptured=false;float ResponseMargin=0;uint64 ResponseScopeSerial=0;
    Analytic::FWorldQuery Filter;
};
class FNativeSphereRequest;
using FNativeSphereRequestPtr=TSharedPtr<const FNativeSphereRequest,ESPMode::ThreadSafe>;
IAMSPEED_API bool CaptureSphereCCD(const UWorld* World,const USolidSubBody* Solid,FVector Start,FVector End,
    const FCollisionShape& Shape,float Horizon,FNativeSphereRequestPtr& Out);
IAMSPEED_API bool RevalidateSphereCCD(const FNativeSphereRequest& Request) noexcept;
// An immutable capture + mathematical reference only. This does not certify a
// clear prefix, rounded endpoint, MTD, response, projection or a physical Step.
class FNativeSphereRequest final
{
    FStepIdentity Step;FSphereValues Values;
    FPreparedGeometrySelectionPtr Geometry;
    TSharedPtr<const FPreparedWorldSnapshot,ESPMode::ThreadSafe> Snapshot;
    Analytic::Certified::FZeroOffsetKinematicCenterReference Reference;
    TSharedPtr<const Analytic::Prepared::FScene,ESPMode::ThreadSafe> Scene;
    TSharedPtr<const FPreparedPhysicalQueryDomain,ESPMode::ThreadSafe> Filters;
    FNativeSphereRequest()=default;
    friend IAMSPEED_API bool CaptureSphereCCD(const UWorld*,const USolidSubBody*,FVector,FVector,const FCollisionShape&,float,
        TSharedPtr<const FNativeSphereRequest,ESPMode::ThreadSafe>&);
    friend IAMSPEED_API bool RevalidateSphereCCD(const FNativeSphereRequest&) noexcept;
public:
    const FStepIdentity& GetStep() const {return Step;}
    const TSharedPtr<const FPreparedWorldSnapshot,ESPMode::ThreadSafe>& GetSnapshot() const {return Snapshot;}
    const FPreparedGeometrySelectionPtr& GetGeometrySelection() const {return Geometry;}
    const FSphereValues& GetValues() const {return Values;}
    const Analytic::Certified::FZeroOffsetKinematicCenterReference& GetReference() const {return Reference;}
    // Nominal provenance/policy ONLY; geometry comes from GetGeometrySelection.
    const TSharedPtr<const Analytic::Prepared::FScene,ESPMode::ThreadSafe>& GetScene() const {return Scene;}
};
}
