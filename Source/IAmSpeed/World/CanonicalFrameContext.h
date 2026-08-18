#pragma once

#include "CoreMinimal.h"

/** Integer-addressed timing shared by every canonical simulation driver. */
struct IAMSPEED_API FCanonicalFrameContext
{
	static constexpr uint32 PhysicsFramesPerSecond = 300u;
	static constexpr float CanonicalPhysicalDeltaTime = 1.0f / float(PhysicsFramesPerSecond);

	explicit FCanonicalFrameContext(const uint64 InNumFrame)
		: NumFrame(InNumFrame)
		, PhysicalDeltaTime(CanonicalPhysicalDeltaTime)
		, SimTime(static_cast<float>(InNumFrame) * CanonicalPhysicalDeltaTime)
	{
	}

	uint64 NumFrame = 0;
	float PhysicalDeltaTime = CanonicalPhysicalDeltaTime;
	float SimTime = 0.0f;
};
