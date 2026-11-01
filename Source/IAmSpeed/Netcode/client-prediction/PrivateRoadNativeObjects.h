#pragma once
#include "CoreMinimal.h"
#include "IAmSpeed/Actors/SpeedCar.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/World/Simulation/SpeedPhysicsSimulation.h"
#include <memory>
#include "PrivateRoadNativeObjects.generated.h"
namespace Speed::Netcode::Prediction { struct FPrivateRoadAccess;class FPrivateRoadPhysicsWorld; }

// A real subtype whose lifecycle never creates a Chaos/network physics owner.
// Its manually owned wheel/suspension storage belongs to the private factory.
UCLASS(NotBlueprintable,Transient)
class IAMSPEED_API USpeedBranchWheeledComponent final : public USpeedWheeledComponent
{
    GENERATED_BODY()
    friend class Speed::Netcode::Prediction::FPrivateRoadPhysicsWorld;
    std::weak_ptr<Speed::Netcode::Prediction::FPrivateRoadAccess> Access;
    void UpdateInputs() override;
    void OnRegister() override;
    void OnCreatePhysicsState() override;
    void OnDestroyPhysicsState() override;
    void BeginPlay() override;
public:
    explicit USpeedBranchWheeledComponent(const FObjectInitializer& Initializer=FObjectInitializer::Get());
    bool AllowsExternalNetworkStateRestore() const override {return false;}
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
