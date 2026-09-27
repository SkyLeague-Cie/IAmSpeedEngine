// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "WheeledVehiclePawn.h"
#include "IAmSpeed/Camera/SpeedArmComponent.h"
#include "IAmSpeed/Input/InputStream.h"
#include "IAmSpeed/Input/InputStreamV2.h"
#include <memory>
#include "SpeedCar.generated.h"

class USpeedWheeledComponent;
#if !UE_BUILD_SHIPPING
class ASpeedSimulation;
#endif

/**
 *
 */
UCLASS()
class IAMSPEED_API ASpeedCar : public AWheeledVehiclePawn
{
	GENERATED_BODY()

	/** Cast pointer to the Chaos Vehicle movement component */
	TObjectPtr<USpeedWheeledComponent> SpeedWheeledComponent;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = Camera, meta = (AllowPrivateAccess = "true"))
	USpeedArmComponent* CameraArm;
public:
	ASpeedCar(const FObjectInitializer& ObjectInitializer);
	USpeedArmComponent* GetSpeedCameraArm() const { return CameraArm; }

	void BeginPlay() override;
	void Tick(float Delta) override;
	void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	/** Lets the possessed SpeedController bind common and specialized actions. */
	void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

	// --- Input functions ---
	// Set the throttle input for this frame, value between 0 and 1
	void SetThrottleInput(const float& Throttle);
	// Set the brake input for this frame, value between 0 and 1
	void SetBrakeInput(const float& Brake);
	// Set the steering input for this frame, value between -1 and 1
	void SetSteeringInput(const float& Steering);
	void SetFrameInputStream(std::shared_ptr<Speed::Input::FInputStream> Stream);
	bool SetFrameInputStreamV2(std::shared_ptr<Speed::Input::V2::FInputStream> Stream);
	bool NeutralizeProducedInputAtBoundary();
	bool NeutralizeProducedInputAfterOwnerJoined();
	void SetCameraBackInput(bool bBack);
	void SetCameraYawInput(float Value);
	void SetCameraPitchInput(float Value);
	virtual void ClearCameraInputs();
	// --- End of input functions ---

	void SetPhysSparkleLocation(const FVector& HitLocation);
	UFUNCTION(BlueprintImplementableEvent, Category = "Utils")
	void SetSparkleLoction(const FVector& HitLocation);
	UFUNCTION(BlueprintImplementableEvent, Category = "Utils")
	void OnStartSparkle();
	void StartSparkleTimer();
	UFUNCTION(BlueprintImplementableEvent, Category = "Utils")
	void StopSparkle();

	UFUNCTION(BlueprintImplementableEvent, Category = "Utils")
	void Demo(ASpeedCar* car, bool isOnOpponentCar);
	void DemoedBy(ASpeedCar* car);

	UFUNCTION(BlueprintCallable, Category = "Utils")
	void StartConfrontationInSec(const float& TimeSec);

	virtual bool OnTheSameTeamAs(const ASpeedCar& OtherCar) const;

	bool HasAuthority() const;
	bool IsOwningClient() const;
	bool IsRemoteClient() const;

	// Freezes car movement. It serves when we want to pause the game
	void FreezeMovement();
protected:
	virtual void HandleKinematics();
	virtual void HandleCameraPresentation(float Delta);
	virtual void HandleSparkle();
	virtual void DemoedByPrv(ASpeedCar* car);

private:
	FSimulationPoseConsumption GenericOwnerConsumption;
#if !UE_BUILD_SHIPPING
	/** Resolves the authoritative simulation once and audits snapshot cadence. */
	void AuditPresentationFrameCadence(float GameDeltaSeconds);
#endif

	bool hasSparkleLocation = false;
	FVector SparkleLocation = FVector::ZeroVector;
	FTimerHandle SparkleTimerHandler;
#if !UE_BUILD_SHIPPING
	TWeakObjectPtr<ASpeedSimulation> PresentationSimulation;
	uint64 LastObservedSimulationFrame = MAX_uint64;
#endif
};
