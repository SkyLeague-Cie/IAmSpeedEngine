#pragma once

#include "CoreMinimal.h"
#include "GameFramework/SpringArmComponent.h"
#include "SpeedCarCameraPresentation.h"
#include "SpeedArmComponent.generated.h"

class UCameraComponent;
class USphereComponent;
class USpeedWheeledComponent;
class ASpeedCar;
class ASpeedSimulation;
class ISpeedComponent;
struct FSimulationPresentationBinding;

/** Common UObject ownership; the simulation owns the only mutable camera solver. */
UCLASS()
class IAMSPEED_API USpeedArmComponent : public USpringArmComponent
{
	GENERATED_BODY()
public:
	USpeedArmComponent(const FObjectInitializer& ObjectInitializer);
	void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void InitializeForCar(ASpeedCar& Owner, USpeedWheeledComponent& Movement);
	bool UsesGenericOwnerSnapshot() const { return bGenericSnapshotInitialized; }
	bool ReadGenericOwnerSnapshot(FSimulationPoseConsumption& Out) const;
	bool ApplyGenericCameraSnapshot(const FSimulationPoseConsumption& Owner);
	/** Pure GT import seam, also exercised with sealed physical publications. */
	bool UpdateCameraFromSnapshot(const FSimulationPresentationOutput& Output, const FSimulationPoseConsumption& Owner);

	// Reflected names and nested object paths intentionally match the original
	// game component. Derived arms inherit, never create a second Camera/Sphere.
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = Camera, meta = (AllowPrivateAccess = "true"))
	UCameraComponent* Camera;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	USphereComponent* HitSphere;
	UPROPERTY(BlueprintReadWrite, Category = Lag, EditDefaultsOnly)
	float LagSpeedCoeff = 1.0f;

protected:
	using FEndpointPtr = TSharedPtr<ISimulationPresentationProducer, ESPMode::ThreadSafe>;
	/** Called only inside the acknowledged physical binding boundary. The game
	 * override supplies its endpoint; the generic implementation is not also run. */
	virtual FEndpointPtr CreateCameraEndpoint(const FSimulationPresentationBinding& Binding);
	FEndpointPtr BindEndpointAtLifecycle(ASpeedSimulation& Simulation, ISpeedComponent& Owner, ISpeedComponent* Target);
	const FEndpointPtr& GetBoundEndpoint() const { return BoundEndpoint; }
	ASpeedSimulation* GetBoundSimulation() const;
	void UpdateDesiredArmLocation(bool bTrace, bool bLocationLag, bool bRotationLag, float Delta) override;

private:
#if WITH_DEV_AUTOMATION_TESTS
	friend class FGenericCameraPhysicalAdapterTest;
#endif
	void BindGenericAtWorldBeginPlay();
	FEndpointPtr BoundEndpoint;
	TWeakObjectPtr<ASpeedSimulation> BoundSimulation;
	TWeakObjectPtr<USpeedWheeledComponent> GenericMovement;
	FDelegateHandle WorldBeginPlayHandle;
	bool bGenericSnapshotInitialized = false;
	bool bGenericSettingsUnsupported = false;
	FSpeedCarCameraConfiguration InitialConfiguration;
	FSpeedCarCameraArmState InitialState;
	TSharedPtr<const FSpeedCarCameraPose, ESPMode::ThreadSafe> GenericSnapshot;
	uint64 LastPublicationSerial = 0;
	FTransform ExpectedWorld = FTransform::Identity;
};
