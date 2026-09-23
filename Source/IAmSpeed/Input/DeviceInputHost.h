#pragma once
#include "CoreMinimal.h"
#include "InputHostSession.h"
#include "DeviceActivityPolicy.h"

namespace Speed::Input::V2
{
// No gameplay defaults are hidden in the platform factory. The game supplies
// its canonical mapping, activity policy and OS acquisition cadence.
struct FDeviceInputHostConfig
{
	std::shared_ptr<const FInputActionContract> Contract;
	FFrameNumber FirstFrame = 0;
	FInputProcessingPolicy Processing;
	FActivityConfig Activity{}; // Zero is invalid, never an implicit product policy.
	std::chrono::microseconds Cadence{0};
	std::chrono::milliseconds StartupTimeout{0};
	std::vector<FControlBinding> Controls;
};
IAMSPEED_API FStreamEpoch AllocateInputStreamEpoch();
// Unsupported targets return null. A future platform adapter implements this
// factory without changing the producer, mapper, simulation or controller.
IAMSPEED_API std::shared_ptr<FInputHostSession> CreateDeviceInputHost(const FDeviceInputHostConfig& Config);
}
