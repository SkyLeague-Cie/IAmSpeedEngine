#pragma once

#include "CoreMinimal.h"

namespace Speed::CanonicalFrameDriver
{
	/** Experimental ownership switch. Disabled by default until equivalence gates pass. */
	IAMSPEED_API bool IsEnabled();

	/** True only when a sealed scenario should run without real-time pacing. */
	IAMSPEED_API bool IsFastSimulationEnabled();

	/** Safety bound for one complete simulation-owned Fast run. */
	IAMSPEED_API uint32 GetFastMaxFramesPerRun();

	/** Engine async callbacks yielded after a scenario is sealed, before Fast starts. */
	IAMSPEED_API uint32 GetFastReadyDelayCallbacks();
}
