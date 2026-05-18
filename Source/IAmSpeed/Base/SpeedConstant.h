#pragma once

struct SpeedConstants
{
	static constexpr unsigned int NbCCDSubsteps = 16; // number of substeps for CCD sweep (important for performance, but also for stability of the solver)
	static constexpr unsigned short int RecordedHistorySize = 1024; // size of the history buffer for predicted states, used for client-side prediction and reconciliation. Must be a power of 2 for efficient modulo operation when accessing the buffer.
	static constexpr float SuspScale = 100.0f; // suspension displacement is sent with 1 / SuspScale cm precision.
};
