#pragma once

#include "CoreMinimal.h"

enum class ESimulationExecutionMode : uint8
{
	UnrealAsyncCallback = 0,
	GameThread = 1,
	IAmSpeedThread = 2,
};

namespace Speed::CanonicalFrameDriver
{
	/** Enables the canonical pipeline; its execution host remains the legacy callback by default. */
	IAMSPEED_API bool IsEnabled();

	/** Selects the temporary callback, game-thread milestone or owned worker. */
	IAMSPEED_API ESimulationExecutionMode GetExecutionMode();

	/** Allows the owned worker to pause without losing its canonical frame. */
	IAMSPEED_API bool IsOwnedThreadPaused();

	/** True only when a sealed scenario should run without real-time pacing. */
	IAMSPEED_API bool IsFastSimulationEnabled();

	/** Safety bound for one complete simulation-owned Fast run. */
	IAMSPEED_API uint32 GetFastMaxFramesPerRun();

	/** Engine async callbacks yielded after a scenario is sealed, before Fast starts. */
	IAMSPEED_API uint32 GetFastReadyDelayCallbacks();
}
