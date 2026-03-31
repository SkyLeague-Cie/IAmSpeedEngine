#pragma once

struct SpeedConstants
{
	static constexpr unsigned int NbCCDSubsteps = 16; // number of substeps for CCD sweep (important for performance, but also for stability of the solver)
	static constexpr unsigned short int PredictedHistorySize = 1024; // size of the history buffer for predicted states, used for client-side prediction and reconciliation. Must be a power of 2 for efficient modulo operation when accessing the buffer.
	static constexpr float SuspScale = 100.0f; // scale factor for suspension displacement to be sent over the network as int16 (to save bandwidth). The actual suspension displacement in cm will be divided by this value before being quantized and multiplied by this value after being dequantized.
};