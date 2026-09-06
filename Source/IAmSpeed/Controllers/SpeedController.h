#pragma once

#include "CoreMinimal.h"
#include "GameFramework/PlayerController.h"
#include "SpeedController.generated.h"

class ASpeedCar;
class UEnhancedInputComponent;
class UInputAction;
class UInputMappingContext;
struct FInputActionValue;

/**
 * Generic player controller for an IAmSpeed car.
 *
 * The base class owns the common ground-driving inputs. Games can specialize
 * presentation and input filtering through the protected virtual hooks.
 */
UCLASS()
class IAMSPEED_API ASpeedController : public APlayerController
{
	GENERATED_BODY()

public:
	/** Binds the common IAmSpeed driving actions to this controller. */
	virtual void SetupEnhancedInputComponent(UEnhancedInputComponent* EnhancedInputComponent);

	/** Applies forward throttle in the normalized [0, 1] range. */
	void Throttle(const FInputActionValue& Value);
	/** Announces the beginning of braking to presentation specializations. */
	void StartBrake(const FInputActionValue& Value);
	/** Applies braking/reverse input in the normalized [0, 1] range. */
	void Brake(const FInputActionValue& Value);
	/** Clears braking and announces its end to presentation specializations. */
	void StopBrake(const FInputActionValue& Value);
	/** Applies signed steering input in the normalized [-1, 1] range. */
	void Steering(const FInputActionValue& Value);
	/** Handles the bound pause action; games may specialize their pause presentation. */
	virtual void PauseInput(const FInputActionValue& Value);

	/**
	 * Toggles Unreal pause normally. In standalone mode, SetPause also suspends
	 * or resumes the separately-owned IAmSpeed simulation worker.
	 */
	void Pause() override;
	bool SetPause(bool bPause, FCanUnpause CanUnpauseDelegate = FCanUnpause()) override;

protected:
	void SetupInputComponent() override;
	void OnPossess(APawn* InPawn) override;

	/** Lets games apply their user-configured steering response or deadzone. */
	virtual float FilterSteeringInput(float SteeringInput) const;
	/** Lets games mirror brake state into presentation such as brake lights. */
	virtual void OnBrakeInputChanged(bool bBraking);
	/** Presentation hook called after Unreal and the owned simulation agree on pause. */
	virtual void OnPauseStateChanged(bool bPaused) {}
	/** Aligns the owned standalone simulation with Unreal's current pause state. */
	void SynchronizeOwnedSimulationPauseWithWorld();

	/** Input mapping context shared by the generic and game-specific actions. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Input)
	TObjectPtr<UInputMappingContext> InputMappingContext = nullptr;

	/** IAmSpeed car currently controlled by this player controller. */
	UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly, Category = Input)
	TObjectPtr<ASpeedCar> SpeedCar = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Input)
	TObjectPtr<UInputAction> SteeringAction = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Input)
	TObjectPtr<UInputAction> ThrottleAction = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Input)
	TObjectPtr<UInputAction> BrakeAction = nullptr;

	/** Exposed here so derived controllers can bind their own pause UI action. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Input)
	TObjectPtr<UInputAction> PauseAction = nullptr;

private:
	/** Updates the worker owned by the authoritative IAmSpeed game mode. */
	void SetStandaloneSimulationPaused(bool bPaused);
};
