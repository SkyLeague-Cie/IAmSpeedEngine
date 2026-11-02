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
    Speed::FKinematicState Kinematics;
    SHitResult FutureHit;
    SHitResult CurrentHit;
    SHitResult GroundHit;
    SHitResult BoxHit;
    SHitResult SphereHit;
    SHitResult WheelHit;
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
    SHitResult const& WheelHitStorage() const override;
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
    SHitResult const& WheelHitStorage() const override;
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
    uint64 NextPrivate() const {return CanonicalNumFrame;}
    ESimulationWorkerResult DriveSimulation(float,float) override {return ESimulationWorkerResult::Failed;}
    void WaitBetweenFrames(FSimulationWorkerWaitContext&) override {}
    bool AllowsLegacySnapshotRollback() const override {return false;}
public:
    ASpeedBranchSimulation();
    void BeginPlay() override;
    void Tick(float) override {}
};
