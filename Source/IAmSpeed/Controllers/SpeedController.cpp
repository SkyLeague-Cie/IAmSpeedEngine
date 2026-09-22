#include "SpeedController.h"

#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "IAmSpeed/Actors/SpeedCar.h"
#include "IAmSpeed/World/Simulation/SpeedGameMode.h"
#include "IAmSpeed/World/Simulation/SpeedSimulation.h"
#include "InputActionValue.h"

ASpeedController::ASpeedController()
{
	PrimaryActorTick.bTickEvenWhenPaused = true;
	bShouldPerformFullTickWhenPaused = true;
}

void ASpeedController::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (bInputSessionPendingV2 && !bInputLifecycleFault) RefreshInputSessionV2();
	if (InputSessionV2) { ServiceInputSessionV2(); return; }
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
	if (SpeedCar || InputSessionV2 || Speed::Input::FPresentationInputScope::IsActive()) return false;
	InputProducer = MoveTemp(Producer);
	return true;
}

void ASpeedController::SetupInputComponent()
{
	Super::SetupInputComponent();
	// Mapping contexts are configuration data only. OS acquisition owns physical input.
}

void ASpeedController::OnPossess(APawn* InPawn)
{
	ESimulationQuiescence InputBoundary = ESimulationQuiescence::AlreadyStopped;
	if (SpeedCar && InputSessionV2)
	{
		if (!ReleaseInputLifecycle()) return;
	}
	bInputSessionRequiredV2 = !bScenarioOwnsInputAuthority && (InputSessionV2 != nullptr || RequiresInputSessionV2());
	if (!bInputSessionRequiredV2) bInputSessionPendingV2 = false;
	if (InputSessionV2)
	{
		InputBoundary = QuiesceStandaloneInputOwner();
		if (InputBoundary != ESimulationQuiescence::BoundaryAcknowledged && InputBoundary != ESimulationQuiescence::AlreadyStopped)
		{ bInputLifecycleFault = true; return; }
	}
	if (InputSnapshots) InputSnapshots->Deactivate();
	if (SpeedCar && InputSnapshots) SpeedCar->SetFrameInputStream(nullptr);
	if (SpeedCar) SpeedCar->ClearCameraInputs();
	Super::OnPossess(InPawn);
	SpeedCar = CastChecked<ASpeedCar>(InPawn);
	SpeedCar->ClearCameraInputs();
	if (bInputSessionRequiredV2 && !InputSessionV2)
	{
		// Claim neutral authority immediately; a late GameMode/driver cannot
		// open a one-frame fallback to Enhanced Input while the factory awaits it.
		SpeedCar->SetFrameInputStreamV2(nullptr);
		if (InputProducer)
		{
			QuiesceStandaloneInputOwner(); bInputLifecycleFault = true;
			UE_LOG(LogTemp, Error, TEXT("Legacy test producer must be migrated to a V2 session; no device factory fallback"));
			return;
		}
		bInputSessionPendingV2 = true; RefreshInputSessionV2(); return;
	}
	if (InputSessionV2)
	{
		if (InputSessionV2->Descriptor)
		{
			if (!BindInputPresentationV2() || !InputSessionV2->Activate() || !BeginRegistryInputSession()) bInputLifecycleFault = true;
			return;
		}
		if (!InputSessionV2->Activate() || (IsPaused() && !InputSessionV2->PauseAtBoundary())
			|| !SpeedCar->SetFrameInputStreamV2(InputSessionV2->Stream))
		{ InputSessionV2->CloseAtBoundary(); bInputLifecycleFault = true; return; }
		bInputLifecycleFault = false;
		if (!IsPaused()) SetStandaloneSimulationPaused(false);
		return;
	}
	InputSnapshots = InputProducer ? std::make_shared<Speed::Input::FInputStream>(InputProducer) : nullptr;
	PresentationBindings.ResetObservation();
	if (InputSnapshots)
	{
		// Initial ownership establishes a fresh baseline without polling a device.
		ApplyInputLifecyclePause(IsPaused());
		SpeedCar->SetFrameInputStream(InputSnapshots);
	}
}

bool ASpeedController::ReleaseInputLifecycle()
{
	if (InputSessionV2 && !ReleaseInputSessionV2()) return false;
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
	bInputSessionPendingV2 = false;
	return !bInputLifecycleFault;
}

void ASpeedController::OnUnPossess()
{
	if (!ReleaseInputLifecycle())
	{
		UE_LOG(LogTemp, Error, TEXT("Input teardown failed; physical admission remains terminal"));
		if (InputSessionV2) return; // Retryable ownership retention; never silently unpossess a live session.
	}
	Super::OnUnPossess();
}

void ASpeedController::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (!ReleaseInputLifecycle())
	{
		UE_LOG(LogTemp, Error, TEXT("Input end-play teardown failed; physical admission remains terminal"));
		if (InputSessionV2)
			UE_LOG(LogTemp, Fatal, TEXT("Cannot destroy controller while independent input owners remain live"));
	}
	Super::EndPlay(EndPlayReason);
}

void ASpeedController::SetupEnhancedInputComponent(UEnhancedInputComponent*)
{
	// Retained ABI for saved pawns. No UE-frame physical or control dispatch.
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
	if (Result == Speed::Input::EInputLifecycleResult::Rejected)
	{
		bInputLifecycleFault = true;
		return false;
	}
	// A successful source call alone cannot discharge a failed worker acknowledgement.
	return true;
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
	if (InputSessionV2) return SetInputPauseV2(bPause, MoveTemp(CanUnpauseDelegate));
	// Preserve existing pause hosting until an independent producer is installed.
	if (!InputSnapshots)
	{
		const bool Standalone = GetNetMode() == NM_Standalone;
		const bool Changed = bPause != IsPaused();
		if (Standalone && Changed && !bPause && bInputLifecycleFault)
		{
			const auto Boundary = QuiesceStandaloneInputOwner();
			if (Boundary != ESimulationQuiescence::BoundaryAcknowledged && Boundary != ESimulationQuiescence::AlreadyStopped)
				return false;
			bInputLifecycleFault = false;
		}
		if (Standalone && bPause && Changed) SetStandaloneSimulationPaused(true);
		const bool Accepted = Super::SetPause(bPause, MoveTemp(CanUnpauseDelegate));
		if (Standalone && Changed && ((Accepted && !bPause) || (!Accepted && bPause)))
			SetStandaloneSimulationPaused(false);
		return Accepted;
	}
	if (GetNetMode() != NM_Standalone
		|| (bPause == IsPaused() && !(bPause && bInputLifecycleFault)))
		return Super::SetPause(bPause, MoveTemp(CanUnpauseDelegate));
	if (bPause || bInputLifecycleFault)
	{
		const auto Boundary = QuiesceStandaloneInputOwner();
		if (Boundary != ESimulationQuiescence::BoundaryAcknowledged && Boundary != ESimulationQuiescence::AlreadyStopped)
		{
			bInputLifecycleFault = true;
			return false; // Pending pause remains; no source mutation or automatic resume.
		}
		// AlreadyStopped only permits source cancellation. No physical state write.
		if (!ApplyInputLifecyclePause(true)) return false;
		bInputLifecycleFault = false; // Both the owner boundary and source control succeeded.
	}
	const bool bChanged = Super::SetPause(bPause, MoveTemp(CanUnpauseDelegate));
	if ((bChanged && !bPause) || (!bChanged && bPause && !IsPaused()))
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
	if (bScenarioOwnsInputAuthority) return;
	const auto* Mode = GetWorld() ? GetWorld()->GetAuthGameMode<ASpeedGameMode>() : nullptr;
	if (Mode && !Mode->AllowsDeviceInputAuthority()) return; // Scenario controls its worker boundary.

	if (!InputSessionV2 && GetNetMode() != NM_Standalone) return;
	const bool bPaused = IsPaused();
	if (InputSessionV2 && InputSessionV2->Descriptor)
	{
		if (bPaused && InputSessionV2->RegistryBound && (!InputSessionV2->RegistryPaused
			|| (InputSessionV2->PendingCommand && InputSessionV2->PendingOperation == Speed::Input::V2::EBoundaryOperation::Resume)))
			QueueRegistryInputCommand(Speed::Input::V2::EBoundaryOperation::PauseAll);
		if (bPaused) QuiesceStandaloneInputOwner();
		ServiceRegistryInputSession(); return;
	}
	if (InputSessionV2)
	{
		if (bPaused)
		{
			const auto Boundary = QuiesceStandaloneInputOwner();
			bInputLifecycleFault = (Boundary != ESimulationQuiescence::BoundaryAcknowledged
				&& Boundary != ESimulationQuiescence::AlreadyStopped) || !InputSessionV2->PauseAtBoundary();
			if (!bInputLifecycleFault && Boundary == ESimulationQuiescence::BoundaryAcknowledged && IsValid(SpeedCar))
				bInputLifecycleFault = !SpeedCar->NeutralizeProducedInputAtBoundary();
		}
		else if (!bInputLifecycleFault && InputSessionV2->Stream->IsLifecyclePaused())
			bInputLifecycleFault = !InputSessionV2->RequestResumeAtBoundary();
		return;
	}
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
		if (ApplyInputLifecyclePause(true)) bInputLifecycleFault = false;
	}
	else if (!bInputLifecycleFault && ApplyInputLifecyclePause(false))
	{
		SetStandaloneSimulationPaused(false);
	}
}
