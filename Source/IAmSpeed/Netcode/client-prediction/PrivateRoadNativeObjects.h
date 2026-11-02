#pragma once
#include "CoreMinimal.h"
#include "IAmSpeed/Actors/SpeedCar.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/World/Simulation/SpeedPhysicsSimulation.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include <memory>
#include "PrivateRoadNativeObjects.generated.h"
namespace Speed::Netcode::Prediction { struct FPrivateRoadAccess;class FPrivateRoadPhysicsWorld; }
namespace Speed::CommonState {struct FSleepValues;struct FQuantizationValues;}
namespace Speed::Netcode::Prediction
{
// Seven declared value blocks only. Shape configuration, parent/registry,
// ignored/external lists, solid/wheel/suspension extra state remain separate.
struct FPrivateRoadSubBodyValues
{
    FTransform LocalPose=FTransform::Identity;
    ISpeedComponent* ParentComponent=nullptr;
    USSubBody::ESubBodyType SubBodyType=USSubBody::ESubBodyType::Unknown;
    ECollisionChannel CollisionChannel=ECC_WorldDynamic;
    FCollisionResponseParams ResponseParams{};
    TArray<UPrimitiveComponent*> AlwaysIgnoredComponents{};
    TArray<UPrimitiveComponent*> IgnoredComponents{};
    TArray<USSubBody::ESubBodyType> IgnoredSubBodyTypes{};
    TArray<TWeakObjectPtr<UBoxSubBody>> ExternalBoxSubBodies{};
    TArray<TWeakObjectPtr<USphereSubBody>> ExternalSphereSubBodies{};
    TArray<TWeakObjectPtr<USWheelSubBody>> ExternalWheelSubBodies{};
    TArray<TWeakObjectPtr<UCylinderSubBody>> ExternalCylinderSubBodies{};
    Speed::FKinematicState Kinematics;
    SHitResult FutureHit;
    SHitResult CurrentHit;
    SHitResult GroundHit;
    SHitResult BoxHit;
    SHitResult SphereHit;
    SHitResult WheelHit;
 };
// Declared solid values only. No complete reference/configuration closure.
struct FPrivateRoadBoxDomain
{
    FVector BoxExtent=FVector(32.f,32.f,32.f);
    bool bProductionNonpenetratingSphereContacts=false;
    TArray<FVector> CurrentGroundContactsWS{};
    TArray<FVector> PreviousFrameGroundContactsWS{};
    TArray<FVector> CurrentGroundNormalsWS{};
    TArray<FVector> PrevGroundContactsLS{};
    TArray<FVector> LatchedEdgeContactsLS{};
    bool bEdgeSupportLatched=false;
    int32 EdgeSupportLatchFrame=-1;
    bool bRoofSurfaceTraversalLatched=false;
    int32 LastRoofSurfaceContactFrame=INDEX_NONE;
    TWeakObjectPtr<UPrimitiveComponent> RoofSurfaceComp{};
    FVector CompositeGroundNormal=FVector::ZeroVector;
    TMap<TWeakObjectPtr<UPrimitiveComponent>, uint32> HitCountThisFrame{};
    uint32 MaxHitsPerComponentPerFrame=4;
    TArray<float> PrevLambdaN{};
    bool bGroundContactStable=false;
    int32 LastResolvedGroundHitFrame=INDEX_NONE;
    bool bHasGroundContact=false;
    bool bFreshEdgeRecoverCandidate=false;
    float StableTime=0.f;
    bool bGroundHitFromSweep=false;
    bool bGroundPlaneValid=false;
    FVector GroundPlanePointWS=FVector::ZeroVector;
    FVector GroundPlaneN=FVector::UpVector;
    float GroundPlaneD=0.f;
    TWeakObjectPtr<UPrimitiveComponent> GroundComp{};
    uint64 EstablishedSupportSourceId=0;
    uint64 EstablishedSupportSurfaceId=0;
    int32 FreshSupportClampUntilFrame=INDEX_NONE;
    FVector FreshSupportClampN=FVector::UpVector;
    TWeakObjectPtr<UPrimitiveComponent> FreshSupportClampComp{};
    FVector PrevGroundNormal=FVector::UpVector;
    bool bHadGroundContactPrevFrame=false;
    FVector PrevContactNormal=FVector::UpVector;
    bool bHadContactPrevFrame=false;
    float MinSlopCm=0.05f;
    int32 LastWallOrGutterFrame=INDEX_NONE;
    FVector LastWallOrGutterN=FVector::ZeroVector;
    float LastWallOrGutterUpDot=0.f;
    SHitResult LastResolvedSphereHit{};
    int32 LastResolvedSphereHitFrame=INDEX_NONE;
};
struct FPrivateRoadWheelDomain
{
    ISpeedWheeledComponent* WheelComponent=nullptr;
    UChaosVehicleWheel* ChaosWheel=nullptr;
    Chaos::FSimpleWheelSim* PWheel=nullptr;
    Chaos::FSimpleSuspensionSim* PSuspension=nullptr;
    float WheelMass=USWheelSubBody::DefaultWheelMass;
    float SuspensionSpringRate=2750.0f;
    float SuspensionDampingCompressionRatio=0.3f;
    float SuspensionDampingReboundRatio=0.5f;
    bool bUseSuspensionForceModelOverride=false;
    bool bClampSuspensionForceToPositive=false;
    bool bClampPositiveUntilFirstCompression=false;
    bool bUseEffectiveSuspensionSweepRadius=false;
    bool bUseIncreasingDisplacementAsCompression=false;
    bool bUseCompressionCrossingContactImpulse=false;
    uint8 GroundForceApplicationMode=0;
    uint8 ContactImpulseApplicationMode=0;
    float SuspensionStiffnessForceScale=1.0f;
    float SuspensionStiffnessReferenceDisplacement=0.0f;
    float SuspensionPreRestCompressionDampingScale=-1.0f;
    bool bClampNegativeSuspensionDisplacement=false;
    bool bDisableSuspensionBumpStopForce=false;
    float SuspensionSpringRateCmOverride=0.0f;
    float SuspensionCompressionDampingOverride=0.0f;
    float SuspensionReboundDampingOverride=0.0f;
    float SuspensionMaxRaiseOverride=-1.0f;
    float SuspensionMaxDropOverride=-1.0f;
    float ConfiguredStaticSpringCompression=0.0f;
};
struct FPrivateRoadSphereDomain {float Radius=0.f;};
struct FPrivateRoadSolidValues
{
    float Mass=1.0f;
    bool bIsMainSubBody=false;
    bool bApplyRestForce=true;
    bool EnableFakePhysics=false;
    bool bEnableConstraintOnSubBodyHit=false;
    float Restitution=0.0f;
    float SphereBoxRestitutionOverride=-1.0f;
    float SphereBoxFrictionOverride=-1.0f;
    float SphereBoxManifoldFrictionOverride=-1.0f;
    bool bUseCoupledContactImpulse=false;
    bool bUsePostNormalFrictionImpulse=false;
    bool bUsePersistentBilateralContact=false;
    float CoupledContactMaxLinearSpeed=-1.0f;
    float CoupledContactMaxAngularSpeed=-1.0f;
    float CoupledContactMaxAbsLocalNormalY=-1.0f;
    float StaticFriction=0.0f;
    float DynamicFriction=0.0f;
    float ImpactThreshold=0.0f;
    float HitDamping=1.0f;
    FMatrix InvInertiaLocal=FMatrix::Identity;
    int idx=-1;
};
// Declared runtime values only, not configuration or a complete wheel checkpoint.
struct FPrivateRoadWheelRuntimeValues
{
    float SuspensionForce=0.0f;
    uint8 bIsJumping=0;
    bool bJumpUnilateralSupport=false;
    uint8 ConsecutiveGroundFrames=0;
    float SteeringAngle=0.0f;
    float Omega=0.0f;
    float RollAngle=0.0f;
    bool bWasOnGroundPrevFrame=false;
    bool bCountedGroundFrame=false;
    bool bContactVelocityLocked=false;
    bool bCrossedIntoCompressionThisFrame=false;
    FVector ForwardAxis=FVector::ForwardVector;
    FVector RightAxis=FVector::RightVector;
    FVector UpAxis=FVector::UpVector;
    SWheelRenderData RenderData;
};

}


// Only the private physical factory selects this default subobject subtype.
// Observe the real native dispatch; do not add impulses or optional constraints.
UCLASS(NotBlueprintable,Transient)
class IAMSPEED_API USpeedBranchBoxSubBody final : public UBoxSubBody
{
    GENERATED_BODY()
    friend class Speed::Netcode::Prediction::FPrivateRoadPhysicsWorld;
    std::weak_ptr<Speed::Netcode::Prediction::FPrivateRoadAccess> StorageAccess;
    std::unique_ptr<Speed::Netcode::Prediction::FPrivateRoadSubBodyValues> OwnedBodyValues;
    bool UsesOriginalBodyStorage() const;
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    Speed::Netcode::Prediction::FPrivateRoadSolidValues ReadLegacySolidDiagnosticValues() const;
#endif
    std::unique_ptr<Speed::Netcode::Prediction::FPrivateRoadSolidValues> OwnedSolidValues;
    float& MassSolidStorage() override;
    float const& MassSolidStorage() const override;
    bool& bIsMainSubBodySolidStorage() override;
    bool const& bIsMainSubBodySolidStorage() const override;
    bool& bApplyRestForceSolidStorage() override;
    bool const& bApplyRestForceSolidStorage() const override;
    bool& EnableFakePhysicsSolidStorage() override;
    bool const& EnableFakePhysicsSolidStorage() const override;
    bool& bEnableConstraintOnSubBodyHitSolidStorage() override;
    bool const& bEnableConstraintOnSubBodyHitSolidStorage() const override;
    float& RestitutionSolidStorage() override;
    float const& RestitutionSolidStorage() const override;
    float& SphereBoxRestitutionOverrideSolidStorage() override;
    float const& SphereBoxRestitutionOverrideSolidStorage() const override;
    float& SphereBoxFrictionOverrideSolidStorage() override;
    float const& SphereBoxFrictionOverrideSolidStorage() const override;
    float& SphereBoxManifoldFrictionOverrideSolidStorage() override;
    float const& SphereBoxManifoldFrictionOverrideSolidStorage() const override;
    bool& bUseCoupledContactImpulseSolidStorage() override;
    bool const& bUseCoupledContactImpulseSolidStorage() const override;
    bool& bUsePostNormalFrictionImpulseSolidStorage() override;
    bool const& bUsePostNormalFrictionImpulseSolidStorage() const override;
    bool& bUsePersistentBilateralContactSolidStorage() override;
    bool const& bUsePersistentBilateralContactSolidStorage() const override;
    float& CoupledContactMaxLinearSpeedSolidStorage() override;
    float const& CoupledContactMaxLinearSpeedSolidStorage() const override;
    float& CoupledContactMaxAngularSpeedSolidStorage() override;
    float const& CoupledContactMaxAngularSpeedSolidStorage() const override;
    float& CoupledContactMaxAbsLocalNormalYSolidStorage() override;
    float const& CoupledContactMaxAbsLocalNormalYSolidStorage() const override;
    float& StaticFrictionSolidStorage() override;
    float const& StaticFrictionSolidStorage() const override;
    float& DynamicFrictionSolidStorage() override;
    float const& DynamicFrictionSolidStorage() const override;
    float& ImpactThresholdSolidStorage() override;
    float const& ImpactThresholdSolidStorage() const override;
    float& HitDampingSolidStorage() override;
    float const& HitDampingSolidStorage() const override;
    FMatrix& InvInertiaLocalSolidStorage() override;
    FMatrix const& InvInertiaLocalSolidStorage() const override;
    int& idxSolidStorage() override;
    int const& idxSolidStorage() const override;

    Speed::FKinematicState& KinematicsStorage() override;
    Speed::FKinematicState const& KinematicsStorage() const override;
    SHitResult& FutureHitStorage() override;
    SHitResult const& FutureHitStorage() const override;
    SHitResult& CurrentHitStorage() override;
    SHitResult const& CurrentHitStorage() const override;
    SHitResult& GroundHitStorage() override;
    SHitResult const& GroundHitStorage() const override;
    SHitResult& BoxHitStorage() override;
    SHitResult const& BoxHitStorage() const override;
    SHitResult& SphereHitStorage() override;
    SHitResult const& SphereHitStorage() const override;
    SHitResult& WheelHitStorage() override;
    SHitResult const& WheelHitStorage() const override;    FTransform PhysicalLocalPose() const override;
    void SetPhysicalLocalOffset(const FVector&) override;
    ISpeedComponent*& ParentComponentBodyDomainStorage() override;
    ISpeedComponent* const& ParentComponentBodyDomainStorage() const override;
    ESubBodyType& SubBodyTypeBodyDomainStorage() override;
    ESubBodyType const& SubBodyTypeBodyDomainStorage() const override;
    ECollisionChannel& CollisionChannelBodyDomainStorage() override;
    ECollisionChannel const& CollisionChannelBodyDomainStorage() const override;
    FCollisionResponseParams& ResponseParamsBodyDomainStorage() override;
    FCollisionResponseParams const& ResponseParamsBodyDomainStorage() const override;
    TArray<UPrimitiveComponent*>& AlwaysIgnoredComponentsBodyDomainStorage() override;
    TArray<UPrimitiveComponent*> const& AlwaysIgnoredComponentsBodyDomainStorage() const override;
    TArray<UPrimitiveComponent*>& IgnoredComponentsBodyDomainStorage() override;
    TArray<UPrimitiveComponent*> const& IgnoredComponentsBodyDomainStorage() const override;
    TArray<ESubBodyType>& IgnoredSubBodyTypesBodyDomainStorage() override;
    TArray<ESubBodyType> const& IgnoredSubBodyTypesBodyDomainStorage() const override;
    TArray<TWeakObjectPtr<UBoxSubBody>>& ExternalBoxSubBodiesBodyDomainStorage() override;
    TArray<TWeakObjectPtr<UBoxSubBody>> const& ExternalBoxSubBodiesBodyDomainStorage() const override;
    TArray<TWeakObjectPtr<USphereSubBody>>& ExternalSphereSubBodiesBodyDomainStorage() override;
    TArray<TWeakObjectPtr<USphereSubBody>> const& ExternalSphereSubBodiesBodyDomainStorage() const override;
    TArray<TWeakObjectPtr<USWheelSubBody>>& ExternalWheelSubBodiesBodyDomainStorage() override;
    TArray<TWeakObjectPtr<USWheelSubBody>> const& ExternalWheelSubBodiesBodyDomainStorage() const override;
    TArray<TWeakObjectPtr<UCylinderSubBody>>& ExternalCylinderSubBodiesBodyDomainStorage() override;
    TArray<TWeakObjectPtr<UCylinderSubBody>> const& ExternalCylinderSubBodiesBodyDomainStorage() const override;
    FVector& BoxExtentBoxDomainStorage() override;
    FVector const& BoxExtentBoxDomainStorage() const override;
    bool& bProductionNonpenetratingSphereContactsBoxDomainStorage() override;
    bool const& bProductionNonpenetratingSphereContactsBoxDomainStorage() const override;
    TArray<FVector>& CurrentGroundContactsWSBoxDomainStorage() override;
    TArray<FVector> const& CurrentGroundContactsWSBoxDomainStorage() const override;
    TArray<FVector>& PreviousFrameGroundContactsWSBoxDomainStorage() override;
    TArray<FVector> const& PreviousFrameGroundContactsWSBoxDomainStorage() const override;
    TArray<FVector>& CurrentGroundNormalsWSBoxDomainStorage() override;
    TArray<FVector> const& CurrentGroundNormalsWSBoxDomainStorage() const override;
    TArray<FVector>& PrevGroundContactsLSBoxDomainStorage() override;
    TArray<FVector> const& PrevGroundContactsLSBoxDomainStorage() const override;
    TArray<FVector>& LatchedEdgeContactsLSBoxDomainStorage() override;
    TArray<FVector> const& LatchedEdgeContactsLSBoxDomainStorage() const override;
    bool& bEdgeSupportLatchedBoxDomainStorage() override;
    bool const& bEdgeSupportLatchedBoxDomainStorage() const override;
    int32& EdgeSupportLatchFrameBoxDomainStorage() override;
    int32 const& EdgeSupportLatchFrameBoxDomainStorage() const override;
    bool& bRoofSurfaceTraversalLatchedBoxDomainStorage() override;
    bool const& bRoofSurfaceTraversalLatchedBoxDomainStorage() const override;
    int32& LastRoofSurfaceContactFrameBoxDomainStorage() override;
    int32 const& LastRoofSurfaceContactFrameBoxDomainStorage() const override;
    TWeakObjectPtr<UPrimitiveComponent>& RoofSurfaceCompBoxDomainStorage() override;
    TWeakObjectPtr<UPrimitiveComponent> const& RoofSurfaceCompBoxDomainStorage() const override;
    FVector& CompositeGroundNormalBoxDomainStorage() override;
    FVector const& CompositeGroundNormalBoxDomainStorage() const override;
    TMap<TWeakObjectPtr<UPrimitiveComponent>, uint32>& HitCountThisFrameBoxDomainStorage() override;
    TMap<TWeakObjectPtr<UPrimitiveComponent>, uint32> const& HitCountThisFrameBoxDomainStorage() const override;
    uint32& MaxHitsPerComponentPerFrameBoxDomainStorage() override;
    uint32 const& MaxHitsPerComponentPerFrameBoxDomainStorage() const override;
    TArray<float>& PrevLambdaNBoxDomainStorage() override;
    TArray<float> const& PrevLambdaNBoxDomainStorage() const override;
    bool& bGroundContactStableBoxDomainStorage() override;
    bool const& bGroundContactStableBoxDomainStorage() const override;
    int32& LastResolvedGroundHitFrameBoxDomainStorage() override;
    int32 const& LastResolvedGroundHitFrameBoxDomainStorage() const override;
    bool& bHasGroundContactBoxDomainStorage() override;
    bool const& bHasGroundContactBoxDomainStorage() const override;
    bool& bFreshEdgeRecoverCandidateBoxDomainStorage() override;
    bool const& bFreshEdgeRecoverCandidateBoxDomainStorage() const override;
    float& StableTimeBoxDomainStorage() override;
    float const& StableTimeBoxDomainStorage() const override;
    bool& bGroundHitFromSweepBoxDomainStorage() override;
    bool const& bGroundHitFromSweepBoxDomainStorage() const override;
    bool& bGroundPlaneValidBoxDomainStorage() override;
    bool const& bGroundPlaneValidBoxDomainStorage() const override;
    FVector& GroundPlanePointWSBoxDomainStorage() override;
    FVector const& GroundPlanePointWSBoxDomainStorage() const override;
    FVector& GroundPlaneNBoxDomainStorage() override;
    FVector const& GroundPlaneNBoxDomainStorage() const override;
    float& GroundPlaneDBoxDomainStorage() override;
    float const& GroundPlaneDBoxDomainStorage() const override;
    TWeakObjectPtr<UPrimitiveComponent>& GroundCompBoxDomainStorage() override;
    TWeakObjectPtr<UPrimitiveComponent> const& GroundCompBoxDomainStorage() const override;
    uint64& EstablishedSupportSourceIdBoxDomainStorage() override;
    uint64 const& EstablishedSupportSourceIdBoxDomainStorage() const override;
    uint64& EstablishedSupportSurfaceIdBoxDomainStorage() override;
    uint64 const& EstablishedSupportSurfaceIdBoxDomainStorage() const override;
    int32& FreshSupportClampUntilFrameBoxDomainStorage() override;
    int32 const& FreshSupportClampUntilFrameBoxDomainStorage() const override;
    FVector& FreshSupportClampNBoxDomainStorage() override;
    FVector const& FreshSupportClampNBoxDomainStorage() const override;
    TWeakObjectPtr<UPrimitiveComponent>& FreshSupportClampCompBoxDomainStorage() override;
    TWeakObjectPtr<UPrimitiveComponent> const& FreshSupportClampCompBoxDomainStorage() const override;
    FVector& PrevGroundNormalBoxDomainStorage() override;
    FVector const& PrevGroundNormalBoxDomainStorage() const override;
    bool& bHadGroundContactPrevFrameBoxDomainStorage() override;
    bool const& bHadGroundContactPrevFrameBoxDomainStorage() const override;
    FVector& PrevContactNormalBoxDomainStorage() override;
    FVector const& PrevContactNormalBoxDomainStorage() const override;
    bool& bHadContactPrevFrameBoxDomainStorage() override;
    bool const& bHadContactPrevFrameBoxDomainStorage() const override;
    float& MinSlopCmBoxDomainStorage() override;
    float const& MinSlopCmBoxDomainStorage() const override;
    int32& LastWallOrGutterFrameBoxDomainStorage() override;
    int32 const& LastWallOrGutterFrameBoxDomainStorage() const override;
    FVector& LastWallOrGutterNBoxDomainStorage() override;
    FVector const& LastWallOrGutterNBoxDomainStorage() const override;
    float& LastWallOrGutterUpDotBoxDomainStorage() override;
    float const& LastWallOrGutterUpDotBoxDomainStorage() const override;
    SHitResult& LastResolvedSphereHitBoxDomainStorage() override;
    SHitResult const& LastResolvedSphereHitBoxDomainStorage() const override;
    int32& LastResolvedSphereHitFrameBoxDomainStorage() override;
    int32 const& LastResolvedSphereHitFrameBoxDomainStorage() const override;
    std::unique_ptr<Speed::Netcode::Prediction::FPrivateRoadBoxDomain> OwnedBoxDomain;

public:
    explicit USpeedBranchBoxSubBody(const FObjectInitializer& Initializer=FObjectInitializer::Get());
protected:
    void ResolveCurrentHitPrv(const float&,const float&) override;
};
// Same inherited wheel physics; only the declared core value storage routes.
// Exact-class parallel query admission remains closed until separately checked.
UCLASS(NotBlueprintable,Transient)
class IAMSPEED_API USpeedBranchWheelSubBody final : public USWheelSubBody
{
    GENERATED_BODY()
    friend class Speed::Netcode::Prediction::FPrivateRoadPhysicsWorld;
    std::weak_ptr<Speed::Netcode::Prediction::FPrivateRoadAccess> StorageAccess;
    std::unique_ptr<Speed::Netcode::Prediction::FPrivateRoadSubBodyValues> OwnedBodyValues;
    bool UsesOriginalBodyStorage() const;
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    Speed::Netcode::Prediction::FPrivateRoadSolidValues ReadLegacySolidDiagnosticValues() const;
#endif
    std::unique_ptr<Speed::Netcode::Prediction::FPrivateRoadSolidValues> OwnedSolidValues;
    float& MassSolidStorage() override;
    float const& MassSolidStorage() const override;
    bool& bIsMainSubBodySolidStorage() override;
    bool const& bIsMainSubBodySolidStorage() const override;
    bool& bApplyRestForceSolidStorage() override;
    bool const& bApplyRestForceSolidStorage() const override;
    bool& EnableFakePhysicsSolidStorage() override;
    bool const& EnableFakePhysicsSolidStorage() const override;
    bool& bEnableConstraintOnSubBodyHitSolidStorage() override;
    bool const& bEnableConstraintOnSubBodyHitSolidStorage() const override;
    float& RestitutionSolidStorage() override;
    float const& RestitutionSolidStorage() const override;
    float& SphereBoxRestitutionOverrideSolidStorage() override;
    float const& SphereBoxRestitutionOverrideSolidStorage() const override;
    float& SphereBoxFrictionOverrideSolidStorage() override;
    float const& SphereBoxFrictionOverrideSolidStorage() const override;
    float& SphereBoxManifoldFrictionOverrideSolidStorage() override;
    float const& SphereBoxManifoldFrictionOverrideSolidStorage() const override;
    bool& bUseCoupledContactImpulseSolidStorage() override;
    bool const& bUseCoupledContactImpulseSolidStorage() const override;
    bool& bUsePostNormalFrictionImpulseSolidStorage() override;
    bool const& bUsePostNormalFrictionImpulseSolidStorage() const override;
    bool& bUsePersistentBilateralContactSolidStorage() override;
    bool const& bUsePersistentBilateralContactSolidStorage() const override;
    float& CoupledContactMaxLinearSpeedSolidStorage() override;
    float const& CoupledContactMaxLinearSpeedSolidStorage() const override;
    float& CoupledContactMaxAngularSpeedSolidStorage() override;
    float const& CoupledContactMaxAngularSpeedSolidStorage() const override;
    float& CoupledContactMaxAbsLocalNormalYSolidStorage() override;
    float const& CoupledContactMaxAbsLocalNormalYSolidStorage() const override;
    float& StaticFrictionSolidStorage() override;
    float const& StaticFrictionSolidStorage() const override;
    float& DynamicFrictionSolidStorage() override;
    float const& DynamicFrictionSolidStorage() const override;
    float& ImpactThresholdSolidStorage() override;
    float const& ImpactThresholdSolidStorage() const override;
    float& HitDampingSolidStorage() override;
    float const& HitDampingSolidStorage() const override;
    FMatrix& InvInertiaLocalSolidStorage() override;
    FMatrix const& InvInertiaLocalSolidStorage() const override;
    int& idxSolidStorage() override;
    int const& idxSolidStorage() const override;

    std::unique_ptr<Speed::Netcode::Prediction::FPrivateRoadWheelRuntimeValues> OwnedWheelRuntime;
    float& SuspensionForceRuntimeStorage() override;
    float const& SuspensionForceRuntimeStorage() const override;
    uint8& bIsJumpingRuntimeStorage() override;
    uint8 const& bIsJumpingRuntimeStorage() const override;
    bool& bJumpUnilateralSupportRuntimeStorage() override;
    bool const& bJumpUnilateralSupportRuntimeStorage() const override;
    uint8& ConsecutiveGroundFramesRuntimeStorage() override;
    uint8 const& ConsecutiveGroundFramesRuntimeStorage() const override;
    float& SteeringAngleRuntimeStorage() override;
    float const& SteeringAngleRuntimeStorage() const override;
    float& OmegaRuntimeStorage() override;
    float const& OmegaRuntimeStorage() const override;
    float& RollAngleRuntimeStorage() override;
    float const& RollAngleRuntimeStorage() const override;
    bool& bWasOnGroundPrevFrameRuntimeStorage() override;
    bool const& bWasOnGroundPrevFrameRuntimeStorage() const override;
    bool& bCountedGroundFrameRuntimeStorage() override;
    bool const& bCountedGroundFrameRuntimeStorage() const override;
    bool& bContactVelocityLockedRuntimeStorage() override;
    bool const& bContactVelocityLockedRuntimeStorage() const override;
    bool& bCrossedIntoCompressionThisFrameRuntimeStorage() override;
    bool const& bCrossedIntoCompressionThisFrameRuntimeStorage() const override;
    FVector& ForwardAxisRuntimeStorage() override;
    FVector const& ForwardAxisRuntimeStorage() const override;
    FVector& RightAxisRuntimeStorage() override;
    FVector const& RightAxisRuntimeStorage() const override;
    FVector& UpAxisRuntimeStorage() override;
    FVector const& UpAxisRuntimeStorage() const override;
    SWheelRenderData& RenderDataRuntimeStorage() override;
    SWheelRenderData const& RenderDataRuntimeStorage() const override;

    Speed::FKinematicState& KinematicsStorage() override;
    Speed::FKinematicState const& KinematicsStorage() const override;
    SHitResult& FutureHitStorage() override;
    SHitResult const& FutureHitStorage() const override;
    SHitResult& CurrentHitStorage() override;
    SHitResult const& CurrentHitStorage() const override;
    SHitResult& GroundHitStorage() override;
    SHitResult const& GroundHitStorage() const override;
    SHitResult& BoxHitStorage() override;
    SHitResult const& BoxHitStorage() const override;
    SHitResult& SphereHitStorage() override;
    SHitResult const& SphereHitStorage() const override;
    SHitResult& WheelHitStorage() override;
    SHitResult const& WheelHitStorage() const override;    float PhysicalSuspensionRestLength() const override;
    FTransform PhysicalLocalPose() const override;
    void SetPhysicalLocalOffset(const FVector&) override;
    ISpeedComponent*& ParentComponentBodyDomainStorage() override;
    ISpeedComponent* const& ParentComponentBodyDomainStorage() const override;
    ESubBodyType& SubBodyTypeBodyDomainStorage() override;
    ESubBodyType const& SubBodyTypeBodyDomainStorage() const override;
    ECollisionChannel& CollisionChannelBodyDomainStorage() override;
    ECollisionChannel const& CollisionChannelBodyDomainStorage() const override;
    FCollisionResponseParams& ResponseParamsBodyDomainStorage() override;
    FCollisionResponseParams const& ResponseParamsBodyDomainStorage() const override;
    TArray<UPrimitiveComponent*>& AlwaysIgnoredComponentsBodyDomainStorage() override;
    TArray<UPrimitiveComponent*> const& AlwaysIgnoredComponentsBodyDomainStorage() const override;
    TArray<UPrimitiveComponent*>& IgnoredComponentsBodyDomainStorage() override;
    TArray<UPrimitiveComponent*> const& IgnoredComponentsBodyDomainStorage() const override;
    TArray<ESubBodyType>& IgnoredSubBodyTypesBodyDomainStorage() override;
    TArray<ESubBodyType> const& IgnoredSubBodyTypesBodyDomainStorage() const override;
    TArray<TWeakObjectPtr<UBoxSubBody>>& ExternalBoxSubBodiesBodyDomainStorage() override;
    TArray<TWeakObjectPtr<UBoxSubBody>> const& ExternalBoxSubBodiesBodyDomainStorage() const override;
    TArray<TWeakObjectPtr<USphereSubBody>>& ExternalSphereSubBodiesBodyDomainStorage() override;
    TArray<TWeakObjectPtr<USphereSubBody>> const& ExternalSphereSubBodiesBodyDomainStorage() const override;
    TArray<TWeakObjectPtr<USWheelSubBody>>& ExternalWheelSubBodiesBodyDomainStorage() override;
    TArray<TWeakObjectPtr<USWheelSubBody>> const& ExternalWheelSubBodiesBodyDomainStorage() const override;
    TArray<TWeakObjectPtr<UCylinderSubBody>>& ExternalCylinderSubBodiesBodyDomainStorage() override;
    TArray<TWeakObjectPtr<UCylinderSubBody>> const& ExternalCylinderSubBodiesBodyDomainStorage() const override;
    ISpeedWheeledComponent*& WheelComponentWheelDomainStorage() override;
    ISpeedWheeledComponent* const& WheelComponentWheelDomainStorage() const override;
    UChaosVehicleWheel*& ChaosWheelWheelDomainStorage() override;
    UChaosVehicleWheel* const& ChaosWheelWheelDomainStorage() const override;
    Chaos::FSimpleWheelSim*& PWheelWheelDomainStorage() override;
    Chaos::FSimpleWheelSim* const& PWheelWheelDomainStorage() const override;
    Chaos::FSimpleSuspensionSim*& PSuspensionWheelDomainStorage() override;
    Chaos::FSimpleSuspensionSim* const& PSuspensionWheelDomainStorage() const override;
    float& WheelMassWheelDomainStorage() override;
    float const& WheelMassWheelDomainStorage() const override;
    float& SuspensionSpringRateWheelDomainStorage() override;
    float const& SuspensionSpringRateWheelDomainStorage() const override;
    float& SuspensionDampingCompressionRatioWheelDomainStorage() override;
    float const& SuspensionDampingCompressionRatioWheelDomainStorage() const override;
    float& SuspensionDampingReboundRatioWheelDomainStorage() override;
    float const& SuspensionDampingReboundRatioWheelDomainStorage() const override;
    bool& bUseSuspensionForceModelOverrideWheelDomainStorage() override;
    bool const& bUseSuspensionForceModelOverrideWheelDomainStorage() const override;
    bool& bClampSuspensionForceToPositiveWheelDomainStorage() override;
    bool const& bClampSuspensionForceToPositiveWheelDomainStorage() const override;
    bool& bClampPositiveUntilFirstCompressionWheelDomainStorage() override;
    bool const& bClampPositiveUntilFirstCompressionWheelDomainStorage() const override;
    bool& bUseEffectiveSuspensionSweepRadiusWheelDomainStorage() override;
    bool const& bUseEffectiveSuspensionSweepRadiusWheelDomainStorage() const override;
    bool& bUseIncreasingDisplacementAsCompressionWheelDomainStorage() override;
    bool const& bUseIncreasingDisplacementAsCompressionWheelDomainStorage() const override;
    bool& bUseCompressionCrossingContactImpulseWheelDomainStorage() override;
    bool const& bUseCompressionCrossingContactImpulseWheelDomainStorage() const override;
    uint8& GroundForceApplicationModeWheelDomainStorage() override;
    uint8 const& GroundForceApplicationModeWheelDomainStorage() const override;
    uint8& ContactImpulseApplicationModeWheelDomainStorage() override;
    uint8 const& ContactImpulseApplicationModeWheelDomainStorage() const override;
    float& SuspensionStiffnessForceScaleWheelDomainStorage() override;
    float const& SuspensionStiffnessForceScaleWheelDomainStorage() const override;
    float& SuspensionStiffnessReferenceDisplacementWheelDomainStorage() override;
    float const& SuspensionStiffnessReferenceDisplacementWheelDomainStorage() const override;
    float& SuspensionPreRestCompressionDampingScaleWheelDomainStorage() override;
    float const& SuspensionPreRestCompressionDampingScaleWheelDomainStorage() const override;
    bool& bClampNegativeSuspensionDisplacementWheelDomainStorage() override;
    bool const& bClampNegativeSuspensionDisplacementWheelDomainStorage() const override;
    bool& bDisableSuspensionBumpStopForceWheelDomainStorage() override;
    bool const& bDisableSuspensionBumpStopForceWheelDomainStorage() const override;
    float& SuspensionSpringRateCmOverrideWheelDomainStorage() override;
    float const& SuspensionSpringRateCmOverrideWheelDomainStorage() const override;
    float& SuspensionCompressionDampingOverrideWheelDomainStorage() override;
    float const& SuspensionCompressionDampingOverrideWheelDomainStorage() const override;
    float& SuspensionReboundDampingOverrideWheelDomainStorage() override;
    float const& SuspensionReboundDampingOverrideWheelDomainStorage() const override;
    float& SuspensionMaxRaiseOverrideWheelDomainStorage() override;
    float const& SuspensionMaxRaiseOverrideWheelDomainStorage() const override;
    float& SuspensionMaxDropOverrideWheelDomainStorage() override;
    float const& SuspensionMaxDropOverrideWheelDomainStorage() const override;
    float& ConfiguredStaticSpringCompressionWheelDomainStorage() override;
    float const& ConfiguredStaticSpringCompressionWheelDomainStorage() const override;
    float& RadiusSphereDomainStorage() override;
    float const& RadiusSphereDomainStorage() const override;
    std::unique_ptr<Speed::Netcode::Prediction::FPrivateRoadWheelDomain> OwnedWheelDomain;
    std::unique_ptr<Speed::Netcode::Prediction::FPrivateRoadSphereDomain> OwnedSphereDomain;

public:
    explicit USpeedBranchWheelSubBody(const FObjectInitializer& Initializer=FObjectInitializer::Get());
};
// A real subtype whose lifecycle never creates a Chaos/network physics owner.
// Its manually owned wheel/suspension storage belongs to the private factory.
UCLASS(NotBlueprintable,Transient)
class IAMSPEED_API USpeedBranchWheeledComponent final : public USpeedWheeledComponent
{
    GENERATED_BODY()
    friend class Speed::Netcode::Prediction::FPrivateRoadPhysicsWorld;
    std::weak_ptr<Speed::Netcode::Prediction::FPrivateRoadAccess> Access;
    bool HasIsolatedNativeConstruction() const;

    // Established after original construction, before its first physical step.
    // External calls route to legacy facade storage. Other native domains are
    // still unclosed; never infer complete owner/restore authority from this.
    struct FOwnedComponentValues
    {
        TAtomic<uint64> PublishedStableId{0};
        SBaseGameState BaseGameState;
        FBasePhysicsState BasePhysicsState;
        FWheeledGameState WheeledGameState;
        FWheeledPhysicsState WheeledPhysicsState;
        FWheeledInputState WheeledPhysicalInput;
        TArray<FPhysicalContactConstraint> PhysicalConstraints;
        Speed::FSimulationSleepState SleepState;
        Speed::FIdentityKinematicQuantizationCache KinematicQuantizationCache;
        const Speed::IStaticCollisionWorld* StaticRestingWorld=nullptr;
        float StaticSupportFrameHorizon=0;

    };
    std::unique_ptr<FOwnedComponentValues> OwnedComponentValues;
    bool UsesOriginalComponentStorage() const;
    TAtomic<uint64>& PublishedSimulationStableIdStorage() override;
    TAtomic<uint64> const& PublishedSimulationStableIdStorage() const override;
    TArray<FPhysicalContactConstraint>& PhysicalConstraintsStorage() override;
    TArray<FPhysicalContactConstraint> const& PhysicalConstraintsStorage() const override;
    Speed::FSimulationSleepState& SleepStateStorage() override;
    Speed::FSimulationSleepState const& SleepStateStorage() const override;
    Speed::FIdentityKinematicQuantizationCache& KinematicQuantizationCacheStorage() override;
    Speed::FIdentityKinematicQuantizationCache const& KinematicQuantizationCacheStorage() const override;
    const Speed::IStaticCollisionWorld*& StaticRestingWorldStorage() override;
    const Speed::IStaticCollisionWorld* const& StaticRestingWorldStorage() const override;
    float& StaticSupportFrameHorizonStorage() override;
    float const& StaticSupportFrameHorizonStorage() const override;

    void* SuspendStorageRoutingForExternalObservation() const override;
    void ResumeStorageRoutingAfterExternalObservation(void*) const override;
    void EstablishOriginalComponentStorage();
    SBaseGameState& BaseGameStateStorage() override;
    const SBaseGameState& BaseGameStateStorage() const override;
    FBasePhysicsState& BasePhysicsStateStorage() override;
    const FBasePhysicsState& BasePhysicsStateStorage() const override;
    FWheeledGameState& WheeledGameStateStorage() override;
    const FWheeledGameState& WheeledGameStateStorage() const override;
    FWheeledPhysicsState& WheeledPhysicsStateStorage() override;
    const FWheeledPhysicsState& WheeledPhysicsStateStorage() const override;
    FWheeledInputState& WheeledPhysicalInputStorage() override;
    const FWheeledInputState& WheeledPhysicalInputStorage() const override;

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    // Copies only the declared cache subset during this original native step.
    // No mutable reference, restore authority or complete coverage is returned.
    bool CopyActualCacheDiagnosticValues(Speed::CommonState::FSleepValues&,
        Speed::CommonState::FQuantizationValues&) const;
#endif
    bool IsInitialSynchronizationComplete() const override;
    void UpdateInputs() override;
    friend class USpeedBranchBoxSubBody;
    void ObserveContactPair(const USSubBody&,const USSubBody&,uint8 Kind);
    void RegisterWheelGroundContact(const SWheelGroundContact&) override;
    void OnRegister() override;
    void OnCreatePhysicsState() override;
    void OnDestroyPhysicsState() override;
    void BeginPlay() override;
public:
    explicit USpeedBranchWheeledComponent(const FObjectInitializer& Initializer=FObjectInitializer::Get());
    bool AllowsExternalNetworkStateRestore() const override {return false;}
    bool CanRestoreSimulationSnapshot(TConstArrayView<uint8>) const override {return false;}
#if !UE_BUILD_SHIPPING
    bool AllowsPhysicsPhaseObserverInstallation() const override {return false;}
#endif
    bool InstallCanonicalInput(uint64,std::shared_ptr<const Speed::Input::V2::FOwnerInputSnapshot>) override {return false;}
    bool ValidateCanonicalFrameCommit(uint64 Frame) const override;
    bool CommitCanonicalFrame(uint64 Frame) noexcept override;
    void OnCanonicalFramePublished(uint64) override {}
    void AbortCanonicalFrame(uint64,ECanonicalFrameAbortReason) noexcept override;
};
UCLASS(NotBlueprintable,NotPlaceable,Transient)
class IAMSPEED_API ASpeedBranchCar final : public ASpeedCar
{
    GENERATED_BODY()
public:
    explicit ASpeedBranchCar(const FObjectInitializer& Initializer=FObjectInitializer::Get());
    void BeginPlay() override;
    void Tick(float) override {}
};
UCLASS(NotBlueprintable,NotPlaceable,Transient)
class IAMSPEED_API ASpeedBranchSimulation final : public ASpeedPhysicsSimulation
{
    GENERATED_BODY()
    friend class Speed::Netcode::Prediction::FPrivateRoadPhysicsWorld;
    std::weak_ptr<Speed::Netcode::Prediction::FPrivateRoadAccess> Access;
    bool InitializePrivate(USpeedWorldSubsystem& Bridge);
    bool AdvancePrivate(bool Resimulation);
    uint64 NextPrivate() const {return CanonicalNumFrameDriverStorage();}
    ESimulationWorkerResult DriveSimulation(float,float) override {return ESimulationWorkerResult::Failed;}
    void WaitBetweenFrames(FSimulationWorkerWaitContext&) override {}
    bool AllowsLegacySnapshotRollback() const override {return false;}
    bool AllowsNativeDriverExecution() const override {return HasOriginalDriverExecutionRoute();}
public:
    ASpeedBranchSimulation();
    void BeginPlay() override;
    void Tick(float) override {}
};
