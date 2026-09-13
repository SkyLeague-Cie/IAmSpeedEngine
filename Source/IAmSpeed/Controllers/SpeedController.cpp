#include "SpeedController.h"

#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "IAmSpeed/Actors/SpeedCar.h"
#include "IAmSpeed/World/Simulation/SpeedGameMode.h"
#include "InputActionValue.h"

void ASpeedController::SetupInputComponent()
{
	Super::SetupInputComponent();

	if (UEnhancedInputLocalPlayerSubsystem* Subsystem =
		ULocalPlayer::GetSubsystem<UEnhancedInputLocalPlayerSubsystem>(GetLocalPlayer()))
	{
		Subsystem->AddMappingContext(InputMappingContext, 0);
	}
}

void ASpeedController::OnPossess(APawn* InPawn)
{
	if (SpeedCar) SpeedCar->ClearCameraInputs();
	Super::OnPossess(InPawn);
	SpeedCar = CastChecked<ASpeedCar>(InPawn);
	SpeedCar->ClearCameraInputs();
}

void ASpeedController::OnUnPossess()
{
	if (SpeedCar) SpeedCar->ClearCameraInputs();
	SpeedCar = nullptr;
	Super::OnUnPossess();
}

void ASpeedController::SetupEnhancedInputComponent(
	UEnhancedInputComponent* EnhancedInputComponent)
{
	check(EnhancedInputComponent);

	EnhancedInputComponent->BindAction(
		SteeringAction, ETriggerEvent::Triggered, this, &ASpeedController::Steering);
	EnhancedInputComponent->BindAction(
		SteeringAction, ETriggerEvent::Completed, this, &ASpeedController::Steering);
	EnhancedInputComponent->BindAction(
		ThrottleAction, ETriggerEvent::Triggered, this, &ASpeedController::Throttle);
	EnhancedInputComponent->BindAction(
		ThrottleAction, ETriggerEvent::Completed, this, &ASpeedController::Throttle);
	EnhancedInputComponent->BindAction(
		BrakeAction, ETriggerEvent::Triggered, this, &ASpeedController::Brake);
	EnhancedInputComponent->BindAction(
		BrakeAction, ETriggerEvent::Started, this, &ASpeedController::StartBrake);
	EnhancedInputComponent->BindAction(
		BrakeAction, ETriggerEvent::Completed, this, &ASpeedController::StopBrake);
	EnhancedInputComponent->BindAction(
		PauseAction, ETriggerEvent::Started, this, &ASpeedController::PauseInput);
	EnhancedInputComponent->BindAction(StartBackCameraAction, ETriggerEvent::Started, this, &ASpeedController::StartBackCamera);
	EnhancedInputComponent->BindAction(StartBackCameraAction, ETriggerEvent::Completed, this, &ASpeedController::CompleteBackCamera);
	EnhancedInputComponent->BindAction(CamYawAction, ETriggerEvent::Triggered, this, &ASpeedController::CamYaw);
	EnhancedInputComponent->BindAction(CamYawAction, ETriggerEvent::Completed, this, &ASpeedController::CompleteCamYaw);
	EnhancedInputComponent->BindAction(CamPitchAction, ETriggerEvent::Triggered, this, &ASpeedController::CamPitch);
	EnhancedInputComponent->BindAction(CamPitchAction, ETriggerEvent::Completed, this, &ASpeedController::CompleteCamPitch);
}

void ASpeedController::StartBackCamera(const FInputActionValue&)
{
	if (SpeedCar) SpeedCar->SetCameraBackInput(true);
}
void ASpeedController::CompleteBackCamera(const FInputActionValue&)
{
	if (SpeedCar) SpeedCar->SetCameraBackInput(false);
}
void ASpeedController::CamYaw(const FInputActionValue& Value)
{
	if (SpeedCar) SpeedCar->SetCameraYawInput(Value.Get<float>());
}
void ASpeedController::CamPitch(const FInputActionValue& Value)
{
	if (SpeedCar) SpeedCar->SetCameraPitchInput(Value.Get<float>());
}
void ASpeedController::CompleteCamYaw(const FInputActionValue&)
{
	CamYaw(FInputActionValue(0.0f));
}
void ASpeedController::CompleteCamPitch(const FInputActionValue&)
{
	CamPitch(FInputActionValue(0.0f));
}

void ASpeedController::Throttle(const FInputActionValue& Value)
{
	if (SpeedCar)
	{
		SpeedCar->SetThrottleInput(FMath::Clamp(Value.Get<float>(), 0.0f, 1.0f));
	}
}

void ASpeedController::StartBrake(const FInputActionValue&)
{
	OnBrakeInputChanged(true);
}

void ASpeedController::Brake(const FInputActionValue& Value)
{
	if (SpeedCar)
	{
		SpeedCar->SetBrakeInput(FMath::Clamp(Value.Get<float>(), 0.0f, 1.0f));
	}
}

void ASpeedController::StopBrake(const FInputActionValue&)
{
	OnBrakeInputChanged(false);
	if (SpeedCar)
	{
		SpeedCar->SetBrakeInput(0.0f);
	}
}

void ASpeedController::Steering(const FInputActionValue& Value)
{
	if (SpeedCar)
	{
		const float SteeringInput =
			FMath::Clamp(Value.Get<float>(), -1.0f, 1.0f);
		SpeedCar->SetSteeringInput(FilterSteeringInput(SteeringInput));
	}
}

float ASpeedController::FilterSteeringInput(const float SteeringInput) const
{
	return SteeringInput;
}

void ASpeedController::PauseInput(const FInputActionValue&)
{
	Pause();
}

void ASpeedController::OnBrakeInputChanged(const bool)
{
}

void ASpeedController::Pause()
{
	if (GetNetMode() == NM_Standalone)
	{
		const bool bTargetPaused = !IsPaused();
		if (SetPause(bTargetPaused))
		{
			OnPauseStateChanged(bTargetPaused);
		}
		return;
	}

	Super::Pause();
}

bool ASpeedController::SetPause(
	const bool bPause, FCanUnpause CanUnpauseDelegate)
{
	const bool bStandalone = GetNetMode() == NM_Standalone;
	const bool bStateWillChange = bPause != IsPaused();

	// Stop the worker before Unreal freezes the game thread. If Unreal rejects
	// the pause request, resume it immediately so both clocks remain aligned.
	if (bStandalone && bPause && bStateWillChange)
	{
		SetStandaloneSimulationPaused(true);
	}

	const bool bPauseChanged =
		Super::SetPause(bPause, MoveTemp(CanUnpauseDelegate));
	if (!bStandalone || !bStateWillChange)
	{
		return bPauseChanged;
	}

	if (bPauseChanged)
	{
		if (!bPause)
		{
			SetStandaloneSimulationPaused(false);
		}
	}
	else if (bPause)
	{
		SetStandaloneSimulationPaused(false);
	}

	return bPauseChanged;
}

void ASpeedController::SetStandaloneSimulationPaused(const bool bPaused)
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	if (ASpeedGameMode* SpeedGameMode =
		Cast<ASpeedGameMode>(World->GetAuthGameMode()))
	{
		SpeedGameMode->SetSimulationPaused(bPaused);
	}
}

void ASpeedController::SynchronizeOwnedSimulationPauseWithWorld()
{
	if (GetNetMode() == NM_Standalone)
	{
		SetStandaloneSimulationPaused(IsPaused());
	}
}
