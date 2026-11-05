#pragma once

#include "CoreMinimal.h"

// Execution purpose is not physics authority or a restore capability. A native
// owner chooses it only after its actual correction transaction is admitted.
enum class ECanonicalFramePurpose : uint8 { Forward, Resimulation };

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

	ECanonicalFramePurpose Purpose = ECanonicalFramePurpose::Forward;
	uint64 NumFrame = 0;
	float PhysicalDeltaTime = CanonicalPhysicalDeltaTime;
	float SimTime = 0.0f;
#if !UE_BUILD_SHIPPING
	bool bResimulationForTesting = false; // legacy fixture metadata; never grants restore authority
#endif
    bool IsResimulation() const noexcept
    {
#if !UE_BUILD_SHIPPING
        return Purpose==ECanonicalFramePurpose::Resimulation || bResimulationForTesting;
#else
        return Purpose==ECanonicalFramePurpose::Resimulation;
#endif
    }
};
