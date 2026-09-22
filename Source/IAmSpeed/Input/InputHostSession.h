#pragma once

#include "ActionDispatch.h"
#include "ControlActionReader.h"
#include "InputAcquisitionWorker.h"
#include "InputSessionRegistry.h"
#include "InputObservationChannel.h"

namespace Speed::Input::V2
{
// Construct before possession. A session is single-use; replacement needs a
// new epoch, producer and acquisition journal, including after world reset.
struct FInputHostSession final
{
	std::shared_ptr<FInputStream> Stream;
	std::shared_ptr<FRawAcquisitionJournal> Journal;
	std::unique_ptr<FInputAcquisitionWorker> Acquisition;
	std::unique_ptr<FControlActionReader> Controls;
	std::unique_ptr<FInputPresentationBindings> Presentation;
	std::optional<FSessionDescriptor> Descriptor;
	std::shared_ptr<FInputObservationChannel> Observation;
	std::uint64_t PendingCommand = 0, ResumeGeneration = 0, RegistryVersion = 0, WorkerGeneration = 0;
	std::vector<std::uint64_t> SupersededCommands;
	EBoundaryOperation PendingOperation = EBoundaryOperation::Bind;
	bool RegistryBound = false, RegistryPaused = true;
	bool IsPaused() const { return Descriptor ? RegistryPaused : Stream && Stream->IsLifecyclePaused(); }
	void RequestStopObservation() { if (Observation) Observation->Deactivate(); else if (Stream) Stream->RequestStop(); }
	std::uint64_t Session = 0;
	bool ResumePending = false;
	bool Closed = false;
	bool TerminalRetired = false;

	static std::shared_ptr<FInputHostSession> Create(std::shared_ptr<IInputProducer> Source,
		FStreamEpoch Epoch, FProducerIdentity Identity, FFrameNumber FirstFrame)
	{
		if (!Source || !Epoch.Value || !Identity.Id || FPresentationInputScope::IsActive()) return {};
		auto Out = std::make_shared<FInputHostSession>();
		Out->Session = Epoch.Value;
		Out->Stream = std::make_shared<FInputStream>(Source, Source->GetContract(), Epoch, Identity, FirstFrame);
		Out->Presentation = std::make_unique<FInputPresentationBindings>(Out->Stream);
		return Out;
	}
	static std::shared_ptr<FInputHostSession> CreateDescriptor(FSessionDescriptor Binding)
	{
		if (!Binding.Id || !Binding.Epoch || !Binding.Contract || FPresentationInputScope::IsActive()) return {};
		auto Out = std::make_shared<FInputHostSession>();
		Out->Session = Binding.Epoch;
		Out->Observation = std::make_shared<FInputObservationChannel>(Binding.Contract, FStreamEpoch{Binding.Epoch});
		Out->Presentation = std::make_unique<FInputPresentationBindings>(Out->Observation);
		Out->Descriptor = std::move(Binding);
		return Out;
	}
	bool Activate()
	{
		if (Descriptor) return !Closed && Observation && Presentation && Presentation->Seal() && Observation->Activate();
		return !Closed && Stream && Presentation && Presentation->Seal() && Stream->Activate();
	}
	// Host proves physical quiescence before calling either lifecycle method.
	bool PauseAtBoundary()
	{
		if (Closed || !Stream) return false;
		if (Stream->IsLifecyclePaused() && !ResumePending) return true;
		// Re-pause cancels an outstanding fresh acknowledgment and its edges.
		if (Stream->SetLifecyclePaused(true) == ELifecycleResult::Rejected) return false;
		ResumePending = false; return true;
	}
	bool RequestResumeAtBoundary()
	{
		if (Closed || !Stream) return false;
		if (!Stream->IsLifecyclePaused()) return true;
		if (ResumePending) return true;
		if (Journal && !Journal->RequestFreshResume()) return false;
		ResumePending = true; return true;
	}
	// False is also the ordinary wait case. The host must not wake physics yet.
	bool CompleteFreshResume()
	{
		if (Closed || !Stream || !ResumePending || (Journal && !Journal->IsResumeReady())) return false;
		if (Stream->SetLifecyclePaused(false) == ELifecycleResult::Rejected) { Closed = true; return false; }
		ResumePending = false; return true;
	}
	bool CloseAtBoundary()
	{
		if (Descriptor)
		{
			if (RegistryBound || PendingCommand) return false;
			RequestStopObservation(); Closed = true; ResumePending = false;
			return !Acquisition || Acquisition->Stop();
		}
		ResumePending = false;
		if (!Stream) return false;
		Stream->RequestStop();
		// Keep every owner on rejection; caller retries after physical drainage.
		if (Stream->CancelSourceAtBoundary() == ELifecycleResult::Rejected) return false;
		Closed = true;
		return !Acquisition || Acquisition->Stop();
	}
	// Emergency teardown only after the physical owner has been joined and its
	// outstanding reservation aborted. Cancellation failure is NOT success: the
	// retired session is permanently terminal and cannot be replaced in place.
	bool RetireAtJoinedBoundary()
	{
		if (!Stream) return false; // Mandatory stop is permitted during observer-triggered destruction.
		Stream->RequestStop();
		if (Acquisition && !Acquisition->Stop()) return false;
		ResumePending = false; Closed = true; TerminalRetired = true;
		return true;
	}
};
}
