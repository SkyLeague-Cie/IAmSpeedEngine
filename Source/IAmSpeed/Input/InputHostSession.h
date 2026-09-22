#pragma once

#include "ActionDispatch.h"
#include "ControlActionReader.h"
#include "InputAcquisitionWorker.h"

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
	std::uint64_t Session = 0;
	bool ResumePending = false;
	bool Closed = false;

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
	bool Activate()
	{
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
		ResumePending = false;
		if (!Stream) return false;
		Stream->RequestStop();
		// Keep every owner on rejection; caller retries after physical drainage.
		if (Stream->CancelSourceAtBoundary() == ELifecycleResult::Rejected) return false;
		Closed = true;
		return !Acquisition || Acquisition->Stop();
	}
};
}
