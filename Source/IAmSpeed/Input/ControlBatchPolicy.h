#pragma once

#include "ControlActionReader.h"

#include <algorithm>

namespace Speed::Input::V2
{
// Pause owns the entire acquired control batch, including gameplay presses
// observed just before the Pause edge. Neither ordering nor an even number of
// Pause edges may let a gameplay command cross the menu transition.
inline bool SuppressGameplayControl(const FControlRequests& Batch,
	const FControlRequest& Request, bool PausedAtBatchStart, bool PausedNow)
{
	if (Request.Command == EControlCommand::Pause) return false;
	return PausedAtBatchStart || PausedNow
		|| std::any_of(Batch.Requests.begin(), Batch.Requests.end(), [](const FControlRequest& Candidate)
		{
			return Candidate.Command == EControlCommand::Pause && Candidate.State == EStateAction::Started;
		});
}
}
