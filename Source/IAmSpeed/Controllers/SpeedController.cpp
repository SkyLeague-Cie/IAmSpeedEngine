#include "SpeedController.h"

#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "IAmSpeed/Actors/SpeedCar.h"
#include "IAmSpeed/World/Simulation/SpeedGameMode.h"
#include "InputActionValue.h"
#include "CoreGlobals.h"

void ASpeedController::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (InputSnapshots)
	{
		const auto Snapshot = InputSnapshots->ReadLatest();
		if (Snapshot) HandleInputs(*Snapshot);
	}
}

bool ASpeedController::BindAction(const FString& Name, Speed::Input::FActionId Action,
	Speed::Input::FInputPresentationBindings::FCallback Callback)
{
	check(IsInGameThread());
	return PresentationBindings.BindAction(TCHAR_TO_UTF8(*Name), Action, MoveTemp(Callback));
}

void ASpeedController::HandleInputs(const Speed::Input::FPublishedInputFrame& Snapshot)
{
	check(IsInGameThread());
	PresentationBindings.HandleInputs(Snapshot);
}

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
	if (SpeedCar && bFrameInputProducerActive) SpeedCar->SetFrameInputStream(nullptr);
	if (SpeedCar) SpeedCar->ClearCameraInputs();
	Super::OnPossess(InPawn);
	SpeedCar = CastChecked<ASpeedCar>(InPawn);
	SpeedCar->ClearCameraInputs();
	auto Device = std::make_shared<Speed::Input::FDeviceInputProducer>(FMath::Max(1, InputProducerId));
	DeviceInputProducer = Device.get();
	InputProducer = MoveTemp(Device);
	InputSnapshots = std::make_shared<Speed::Input::FInputStream>(InputProducer);
	PresentationBindings.ResetObservation();
	bFrameInputProducerActive = bUseFrameInputProducer;
	if (bFrameInputProducerActive) SpeedCar->SetFrameInputStream(InputSnapshots);
}

void ASpeedController::OnUnPossess()
{
	if (SpeedCar && bFrameInputProducerActive) SpeedCar->SetFrameInputStream(nullptr);
	bFrameInputProducerActive = false;
	DeviceInputProducer = nullptr;
	InputProducer.reset();
	InputSnapshots.reset();
	if (SpeedCar) SpeedCar->ClearCameraInputs();
	SpeedCar = nullptr;
	Super::OnUnPossess();
}

bool ASpeedController::PublishDeviceAction(Speed::Input::FActionId Action, int16 Value, bool bEmitEdges)
{
	check(IsInGameThread());
	return DeviceInputProducer && DeviceInputProducer->SetAction(GFrameCounter, Action, Value, bEmitEdges);
}

bool ASpeedController::PublishDeviceAxis(Speed::Input::FActionId Action, float Value, bool bSigned)
{
	const auto Quantized = Speed::Input::QuantizeAxis(Value, bSigned);
	return Quantized && PublishDeviceAction(Action, *Quantized);
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
		const float Input = FMath::Clamp(Value.Get<float>(), 0.0f, 1.0f);
		if (bFrameInputProducerActive)
		{
			ensure(PublishDeviceAxis(Speed::Input::Throttle, Value.Get<float>(), false));
		}
		else SpeedCar->SetThrottleInput(Input); // Migration adapter.
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
		const float Input = FMath::Clamp(Value.Get<float>(), 0.0f, 1.0f);
		if (bFrameInputProducerActive)
		{
			ensure(PublishDeviceAxis(Speed::Input::Brake, Value.Get<float>(), false));
		}
		else SpeedCar->SetBrakeInput(Input);
	}
}

void ASpeedController::StopBrake(const FInputActionValue&)
{
	OnBrakeInputChanged(false);
	if (SpeedCar)
	{
		if (bFrameInputProducerActive) { ensure(PublishDeviceAction(Speed::Input::Brake, 0)); }
		else SpeedCar->SetBrakeInput(0.0f);
	}
}

void ASpeedController::Steering(const FInputActionValue& Value)
{
	if (SpeedCar)
	{
		const float SteeringInput =
			FMath::Clamp(Value.Get<float>(), -1.0f, 1.0f);
		const float Filtered = FilterSteeringInput(SteeringInput);
		if (bFrameInputProducerActive)
		{
			ensure(FMath::IsFinite(Value.Get<float>()) && PublishDeviceAxis(Speed::Input::Steering, Filtered, true));
		}
		else SpeedCar->SetSteeringInput(Filtered);
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
