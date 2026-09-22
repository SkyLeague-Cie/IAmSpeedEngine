#include "SpeedController.h"
#include "IAmSpeed/Actors/SpeedCar.h"
#include "IAmSpeed/World/Simulation/SpeedSimulation.h"
#include "IAmSpeed/World/Simulation/SpeedGameMode.h"
#include "InputActionValue.h"

bool ASpeedController::RestartInputSessionAtBoundary()
{
	check(IsInGameThread());
	if (!bInputSessionRequiredV2 || bInputLifecycleFault || Speed::Input::FPresentationInputScope::IsActive()) return false;
	if (!ReleaseInputSessionV2() || bInputLifecycleFault) return false;
	bInputSessionPendingV2 = true;
	return true;
}

bool ASpeedController::RefreshInputSessionV2()
{
	check(IsInGameThread());
	if (!bInputSessionRequiredV2 || !IsValid(SpeedCar) || InputSessionV2 || bInputLifecycleFault
		|| Speed::Input::FPresentationInputScope::IsActive()) return false;
	ASpeedGameMode* Mode = GetWorld() ? GetWorld()->GetAuthGameMode<ASpeedGameMode>() : nullptr;
	ASpeedSimulation* Driver = Mode ? Mode->GetSpeedSimulation() : nullptr;
	if (!Driver) return false; // GameMode may spawn its owned driver after possession.
	const auto Boundary = Driver->TryPauseOwnedSimulation();
	uint64 FirstFrame = 0;
	if ((Boundary != ESimulationQuiescence::BoundaryAcknowledged && Boundary != ESimulationQuiescence::AlreadyStopped)
		|| !Driver->ReadInputFirstFrameAtPausedBoundary(FirstFrame))
	{ bInputLifecycleFault = true; return false; }
	auto Session = CreateInputSessionV2(FirstFrame);
	if (!Session || !Session->Stream || !Session->Presentation || Session->Closed
		|| Session->Session <= LastInputSessionV2 || Session->Stream->GetEpoch().Value != Session->Session
		|| !Session->Stream->CanConfigure())
	{
		if (Session) Session->CloseAtBoundary();
		bInputLifecycleFault = true;
		UE_LOG(LogTemp, Error, TEXT("Independent input session creation rejected; physics remains paused"));
		return false;
	}
	InputSessionV2 = std::move(Session); LastInputSessionV2 = InputSessionV2->Session;
	if (!BindInputPresentationV2() || !InputSessionV2->Activate()
		|| !InputSessionV2->PauseAtBoundary() || !SpeedCar->SetFrameInputStreamV2(InputSessionV2->Stream)
		|| (!IsPaused() && !InputSessionV2->RequestResumeAtBoundary()))
	{ bInputLifecycleFault = true; ReleaseInputSessionV2(); return false; }
	bInputSessionPendingV2 = false;
	// Even initial attachment waits for the acquisition owner's fresh baseline.
	// ServiceInputSessionV2 wakes physics only after that acknowledgement.
	return true;
}

bool ASpeedController::ConfigureInputSessionV2(std::shared_ptr<Speed::Input::V2::FInputHostSession> Session)
{
	check(IsInGameThread());
	if (SpeedCar || InputProducer || InputSnapshots || InputSessionV2 || !Session || !Session->Stream
		|| !Session->Presentation || Session->Closed || !Session->Session || Session->Session <= LastInputSessionV2
		|| Session->Stream->GetEpoch().Value != Session->Session || !Session->Stream->CanConfigure()
		|| Speed::Input::FPresentationInputScope::IsActive()) return false;
	LastInputSessionV2 = Session->Session; InputSessionV2 = std::move(Session); return true;
}

void ASpeedController::HandleInputs()
{
	check(IsInGameThread());
	const auto Session = InputSessionV2;
	if (!Session || !Session->Presentation || Session->Stream->IsLifecyclePaused()) return;
	const auto Result = Session->Presentation->HandleInputs();
	if (Result == Speed::Input::V2::EDispatchStatus::ResyncRequired)
	{
		// A gap is observable. Baseline recovery must not replay a delivered prefix.
		UE_LOG(LogTemp, Error, TEXT("Input presentation history lost in session %llu"), Session->Session);
		Session->Presentation->Resynchronize();
	}
}

bool ASpeedController::ServiceInputSessionV2()
{
	check(IsInGameThread());
	const auto Session = InputSessionV2;
	if (!Session || Session->Closed) return false;
	if (Session->Controls)
	{
		const auto Batch = Session->Controls->Read();
		if (Batch.Status == Speed::Input::V2::EControlRead::Invalid)
		{
			Session->Stream->RequestStop(); QuiesceStandaloneInputOwner();
			bInputLifecycleFault = true; return false;
		}
		if (Batch.Status == Speed::Input::V2::EControlRead::Resynchronized)
			UE_LOG(LogTemp, Error, TEXT("Control input history resynchronized in session %llu; missing commands were not replayed"), Session->Session);
		for (const auto& Request : Batch.Requests)
		{
			if (InputSessionV2 != Session || Session->Closed) break;
			if (Request.Session != Session->Session) { bInputLifecycleFault = true; return false; }
			if (Request.State == Speed::Input::V2::EStateAction::Started)
			{
				const auto Result = ExecuteInputControlV2(Request);
				if (!ControlReceiptsV2.Record(Request, Result)) { bInputLifecycleFault = true; return false; }
			}
		}
	}
	if (InputSessionV2 != Session || Session->Closed) return false;
	if (Session->ResumePending && !IsPaused() && !bInputLifecycleFault && Session->CompleteFreshResume())
		SetStandaloneSimulationPaused(false);
	if (Session->Closed) { bInputLifecycleFault = true; return false; }
	HandleInputs(); return !bInputLifecycleFault;
}

Speed::Input::V2::EControlApplication ASpeedController::ExecuteInputControlV2(const Speed::Input::V2::FControlRequest& Request)
{
	using namespace Speed::Input::V2;
	if (Request.Command != EControlCommand::Pause) return EControlApplication::Rejected;
	const bool Before = IsPaused();
	PauseInput(FInputActionValue(true));
	if (bInputLifecycleFault) return EControlApplication::Rejected;
	return Before != IsPaused() ? EControlApplication::Applied : EControlApplication::NoChange;
}

bool ASpeedController::ReleaseInputSessionV2()
{
	check(IsInGameThread());
	if (!InputSessionV2) return true;
	InputSessionV2->Stream->RequestStop(); // Closure admission is always permitted.
	if (Speed::Input::FPresentationInputScope::IsActive())
	{
		// Keep owners for a retry outside the callback. Never write physical
		// state or release a prepared owner's locks from presentation dispatch.
		bInputLifecycleFault = true; return false;
	}
	const auto Boundary = QuiesceStandaloneInputOwner();
	bool Joined = false;
	auto JoinAndFlush = [&]()
	{
		ASpeedGameMode* Mode = GetWorld() ? GetWorld()->GetAuthGameMode<ASpeedGameMode>() : nullptr;
		ASpeedSimulation* Driver = Mode ? Mode->GetSpeedSimulation() : nullptr;
		bInputLifecycleFault = true; // A retired driver must never restart in place.
		if (!Driver || !Driver->JoinOwnedSimulationForInputTeardown()) return false;
		Joined = true;
		return IsValid(SpeedCar) && SpeedCar->NeutralizeProducedInputAfterOwnerJoined();
	};
	if (Boundary == ESimulationQuiescence::BoundaryAcknowledged)
	{
		if (IsValid(SpeedCar) && !SpeedCar->NeutralizeProducedInputAtBoundary())
		{ bInputLifecycleFault = true; return false; }
	}
	else if (!JoinAndFlush()) return false; // Includes AlreadyStopped; no assumed fence.
	if (!InputSessionV2->CloseAtBoundary())
	{
		if (!Joined && !JoinAndFlush()) return false;
		// Terminal receipts succeeded before acquisition/session retirement.
		if (!InputSessionV2->RetireAtJoinedBoundary()) return false;
		UE_LOG(LogTemp, Error, TEXT("Input session %llu retired after joined-owner failure; no restart claimed"), InputSessionV2->Session);
	}
	if (IsValid(SpeedCar) && !SpeedCar->SetFrameInputStreamV2(nullptr))
	{ bInputLifecycleFault = true; return false; }
	InputSessionV2.reset(); InputReceiversV2.clear(); return true;
}

bool ASpeedController::SetInputPauseV2(bool bPause, FCanUnpause CanUnpauseDelegate)
{
	if (!HasAuthority() || Speed::Input::FPresentationInputScope::IsActive()) return false;
	if (bPause == IsPaused() && !bInputLifecycleFault) return true;
	const auto Boundary = QuiesceStandaloneInputOwner();
	if (Boundary != ESimulationQuiescence::BoundaryAcknowledged && Boundary != ESimulationQuiescence::AlreadyStopped)
	{ bInputLifecycleFault = true; return false; }
	if (!InputSessionV2->PauseAtBoundary()) { bInputLifecycleFault = true; return false; }
	if (Boundary == ESimulationQuiescence::BoundaryAcknowledged && IsValid(SpeedCar)
		&& !SpeedCar->NeutralizeProducedInputAtBoundary()) { bInputLifecycleFault = true; return false; }
	bInputLifecycleFault = false;
	const bool Accepted = Super::SetPause(bPause, MoveTemp(CanUnpauseDelegate));
	if (!IsPaused() && !InputSessionV2->RequestResumeAtBoundary())
	{ bInputLifecycleFault = true; return false; }
	// The independent acquisition owner supplies the fresh-resume acknowledgment.
	// Tick may wake physics only after that acknowledgment, even if UE unpaused.
	return Accepted;
}
