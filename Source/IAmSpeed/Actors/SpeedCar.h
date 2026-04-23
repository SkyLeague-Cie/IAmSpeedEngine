// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "WheeledVehiclePawn.h"
#include "SpeedCar.generated.h"

class USpeedWheeledComponent;

/**
 * 
 */
UCLASS()
class IAMSPEED_API ASpeedCar : public AWheeledVehiclePawn
{
	GENERATED_BODY()

	/** Cast pointer to the Chaos Vehicle movement component */
	TObjectPtr<USpeedWheeledComponent> SpeedWheeledComponent;
public:
	ASpeedCar(const FObjectInitializer& ObjectInitializer);
	
	void BeginPlay() override;
	void Tick(float Delta) override;
	void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	// --- Input functions ---
	// Set the throttle input for this frame, value between 0 and 1
	void SetThrottleInput(const float& Throttle);
	// Set the brake input for this frame, value between 0 and 1
	void SetBrakeInput(const float& Brake);
	// Set the steering input for this frame, value between -1 and 1
	void SetSteeringInput(const float& Steering);
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
	void StartConfrontationInSec(int TimeSec);

	virtual bool OnTheSameTeamAs(const ASpeedCar& OtherCar) const;

	bool HasAuthority() const;
	bool IsOwningClient() const;
	bool IsRemoteClient() const;
protected:
	virtual void HandleKinematics();
	virtual void HandleSparkle();

	void CheckDemo();
private:
	bool hasSparkleLocation = false;
	FVector SparkleLocation = FVector::ZeroVector;
	FTimerHandle SparkleTimerHandler;

	TWeakObjectPtr<ASpeedCar> CarWhichDemoed = nullptr;
};
