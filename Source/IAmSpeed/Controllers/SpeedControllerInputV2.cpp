#include "SpeedController.h"
#include "IAmSpeed/Actors/SpeedCar.h"
#include "IAmSpeed/World/Simulation/SpeedSimulation.h"
#include "InputActionValue.h"

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
	// Stop admission immediately, retaining all owners if the boundary fails.
	InputSessionV2->Stream->RequestStop();
	const auto Boundary = QuiesceStandaloneInputOwner();
	if ((Boundary != ESimulationQuiescence::BoundaryAcknowledged && Boundary != ESimulationQuiescence::AlreadyStopped)
		|| !InputSessionV2->CloseAtBoundary()) { bInputLifecycleFault = true; return false; }
	if (IsValid(SpeedCar) && !SpeedCar->SetFrameInputStreamV2(nullptr))
	{ bInputLifecycleFault = true; return false; }
	InputSessionV2.reset(); InputReceiversV2.clear(); return true;
}

bool ASpeedController::SetInputPauseV2(bool bPause, FCanUnpause CanUnpauseDelegate)
{
	if (GetNetMode() != NM_Standalone || Speed::Input::FPresentationInputScope::IsActive()) return false;
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
