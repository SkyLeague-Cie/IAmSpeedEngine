#pragma once

#include "CoreMinimal.h"

/** Integer-addressed timing shared by every canonical simulation driver. */
struct IAMSPEED_API FCanonicalFrameContext
{
	static constexpr uint32 PhysicsFramesPerSecond = 300u;
	static constexpr float CanonicalPhysicalDeltaTime = 1.0f / float(PhysicsFramesPerSecond);

	explicit FCanonicalFrameContext(const uint64 InNumFrame)
		: NumFrame(InNumFrame)
		, PhysicalDeltaTime(float(LogicalRateDenominator) / float(LogicalRateNumerator))
		, SimTime(static_cast<float>(InNumFrame - FrameOrigin) * PhysicalDeltaTime)
	{
	}

	uint64 NumFrame = 0;
    // Logical integration clock, distinct from pacing/EngineFPS. Native
    // admission captures these from the actual MakeCanonicalFrameContext.
    uint64 LogicalRateNumerator = PhysicsFramesPerSecond;
    uint64 LogicalRateDenominator = 1;
    uint64 FrameOrigin = 0;
	float PhysicalDeltaTime = CanonicalPhysicalDeltaTime;
	float SimTime = 0.0f;
#if !UE_BUILD_SHIPPING
	bool bResimulationForTesting = false; // observation/admission metadata only
#endif
};
