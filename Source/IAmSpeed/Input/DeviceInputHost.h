#pragma once
#include "CoreMinimal.h"
#include "InputHostSession.h"
#include "DeviceActivityPolicy.h"

namespace Speed::Input::V2
{
class FDeviceInputWarmContext;
struct FDeviceInputHostConfig;
IAMSPEED_API std::shared_ptr<FDeviceInputWarmContext> CreateDeviceInputWarmContext();
IAMSPEED_API std::shared_ptr<FInputHostSession> CreateDeviceInputHost(const FDeviceInputHostConfig& Config);
// Controller-owned state outlives a car input session. Its platform catalogue
// is initialized only by the acquisition worker on the first Pump call.
class IAMSPEED_API FDeviceInputWarmContext final
{
public:
	static std::shared_ptr<FDeviceInputWarmContext> Create();
	~FDeviceInputWarmContext();
	bool Close();
	// Opaque state passed only to the platform host. It creates no OS device on
	// the calling thread; first GameInput initialization remains in Pump.
	std::shared_ptr<void> PlatformState() const { return Native; }
private:
	FDeviceInputWarmContext() = default;
	std::shared_ptr<void> Native;
};
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
	std::shared_ptr<FDeviceInputWarmContext> WarmContext;
};
IAMSPEED_API FStreamEpoch AllocateInputStreamEpoch();
// Unsupported targets return null. A future platform adapter implements this
// factory without changing the producer, mapper, simulation or controller.
}
