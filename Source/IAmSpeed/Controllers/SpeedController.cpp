#include "SpeedController.h"

#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "IAmSpeed/Actors/SpeedCar.h"
#include "IAmSpeed/World/Simulation/SpeedGameMode.h"
#include "IAmSpeed/World/Simulation/SpeedSimulation.h"
#include "InputActionValue.h"

void ASpeedController::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (InputSnapshots && !InputSnapshots->IsLifecyclePaused())
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

bool ASpeedController::ConfigureInputProducer(std::shared_ptr<Speed::Input::IInputProducer> Producer)
{
	check(IsInGameThread());
	if (SpeedCar || Speed::Input::FPresentationInputScope::IsActive()) return false;
	InputProducer = MoveTemp(Producer);
	return true;
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
	if (InputSnapshots) InputSnapshots->Deactivate();
	if (SpeedCar && InputSnapshots) SpeedCar->SetFrameInputStream(nullptr);
	if (SpeedCar) SpeedCar->ClearCameraInputs();
	Super::OnPossess(InPawn);
	SpeedCar = CastChecked<ASpeedCar>(InPawn);
	SpeedCar->ClearCameraInputs();
	InputSnapshots = InputProducer ? std::make_shared<Speed::Input::FInputStream>(InputProducer) : nullptr;
	PresentationBindings.ResetObservation();
	if (InputSnapshots)
	{
		// Initial ownership establishes a fresh baseline without polling a device.
		ApplyInputLifecyclePause(IsPaused());
		SpeedCar->SetFrameInputStream(InputSnapshots);
	}
}

void ASpeedController::ReleaseInputLifecycle()
{
	// Mandatory close happens before ownership is released. Retained worker
	// references cannot acquire/publish again; copied history stays immutable.
	const auto Closed = InputSnapshots ? InputSnapshots->Deactivate()
		: (InputProducer ? InputProducer->CancelLifecycle() : Speed::Input::EInputLifecycleResult::UnaffectedByPolicy);
	if (Closed == Speed::Input::EInputLifecycleResult::Rejected) bInputLifecycleFault = true;
	if (IsValid(SpeedCar))
	{
		SpeedCar->SetFrameInputStream(nullptr);
		SpeedCar->ClearCameraInputs();
	}
	InputSnapshots.reset();
	InputProducer.reset();
	SpeedCar = nullptr;
}

void ASpeedController::OnUnPossess()
{
	ReleaseInputLifecycle();
	Super::OnUnPossess();
}

void ASpeedController::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	ReleaseInputLifecycle();
	Super::EndPlay(EndPlayReason);
}

void ASpeedController::SetupEnhancedInputComponent(
	UEnhancedInputComponent* EnhancedInputComponent)
{
	check(EnhancedInputComponent);

	// Legacy device/gameplay dispatch remains only when no independent producer
	// was installed. Enhanced Input is never an acquisition source for that producer.
	if (!InputProducer)
	{
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
	}
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
	if (InputSnapshots) return;
	if (SpeedCar)
	{
		SpeedCar->SetThrottleInput(FMath::Clamp(Value.Get<float>(), 0.0f, 1.0f));
	}
}

void ASpeedController::StartBrake(const FInputActionValue&)
{
	if (InputSnapshots) return;
	OnBrakeInputChanged(true);
}

void ASpeedController::Brake(const FInputActionValue& Value)
{
	if (InputSnapshots) return;
	if (SpeedCar)
	{
		SpeedCar->SetBrakeInput(FMath::Clamp(Value.Get<float>(), 0.0f, 1.0f));
	}
}

void ASpeedController::StopBrake(const FInputActionValue&)
{
	if (InputSnapshots) return;
	OnBrakeInputChanged(false);
	if (SpeedCar)
	{
		SpeedCar->SetBrakeInput(0.0f);
	}
}

void ASpeedController::Steering(const FInputActionValue& Value)
{
	if (InputSnapshots) return;
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

bool ASpeedController::ApplyInputLifecyclePause(const bool bPaused)
{
	if (!InputSnapshots) return true;
	const auto Result = InputSnapshots->SetLifecyclePaused(bPaused);
	bInputLifecycleFault = Result == Speed::Input::EInputLifecycleResult::Rejected;
	return !bInputLifecycleFault;
}

ESimulationQuiescence ASpeedController::QuiesceStandaloneInputOwner()
{
	UWorld* World = GetWorld();
	ASpeedGameMode* GameMode = World ? Cast<ASpeedGameMode>(World->GetAuthGameMode()) : nullptr;
	ASpeedSimulation* Simulation = GameMode ? GameMode->GetSpeedSimulation() : nullptr;
	return Simulation ? Simulation->TryPauseOwnedSimulation() : ESimulationQuiescence::AlreadyStopped;
}

bool ASpeedController::SetPause(const bool bPause, FCanUnpause CanUnpauseDelegate)
{
	// Preserve existing pause hosting until an independent producer is installed.
	if (!InputSnapshots)
	{
		const bool Standalone = GetNetMode() == NM_Standalone;
		const bool Changed = bPause != IsPaused();
		if (Standalone && bPause && Changed) SetStandaloneSimulationPaused(true);
		const bool Accepted = Super::SetPause(bPause, MoveTemp(CanUnpauseDelegate));
		if (Standalone && Changed && ((Accepted && !bPause) || (!Accepted && bPause)))
			SetStandaloneSimulationPaused(false);
		return Accepted;
	}
	if (GetNetMode() != NM_Standalone || bPause == IsPaused())
		return Super::SetPause(bPause, MoveTemp(CanUnpauseDelegate));
	if (bPause)
	{
		const auto Boundary = QuiesceStandaloneInputOwner();
		if (Boundary != ESimulationQuiescence::BoundaryAcknowledged && Boundary != ESimulationQuiescence::AlreadyStopped)
		{
			bInputLifecycleFault = true;
			return false; // Pending pause remains; no source mutation or automatic resume.
		}
		// AlreadyStopped only permits source cancellation. No physical state write.
		if (!ApplyInputLifecyclePause(true)) return false;
	}
	const bool bChanged = Super::SetPause(bPause, MoveTemp(CanUnpauseDelegate));
	if ((bChanged && !bPause) || (!bChanged && bPause))
	{
		// Unreal accepted resume, or rejected pause. Reset to a fresh neutral
		// acquisition generation before allowing another real physical frame.
		if (!ApplyInputLifecyclePause(false)) return false;
		bInputLifecycleFault = false;
		SetStandaloneSimulationPaused(false);
	}
	return bChanged;
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
	if (GetNetMode() != NM_Standalone) return;
	const bool bPaused = IsPaused();
	if (!InputSnapshots)
	{
		if (bPaused || !bInputLifecycleFault) SetStandaloneSimulationPaused(bPaused);
		return;
	}
	if (bPaused)
	{
		const auto Boundary = QuiesceStandaloneInputOwner();
		if (Boundary != ESimulationQuiescence::BoundaryAcknowledged && Boundary != ESimulationQuiescence::AlreadyStopped)
		{
			bInputLifecycleFault = true;
			return;
		}
		ApplyInputLifecyclePause(true);
	}
	else if (!bInputLifecycleFault && ApplyInputLifecyclePause(false))
	{
		SetStandaloneSimulationPaused(false);
	}
}
