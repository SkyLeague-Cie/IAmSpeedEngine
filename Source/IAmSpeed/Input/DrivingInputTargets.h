#pragma once
#include "InputFrame.h"

namespace Speed::Input
{
// Shared values-only boundary used by the real wheeled component and native
// producer tests. It does not simulate physics, filter or apply slew twice.
struct FDrivingInputTargets
{
	bool Valid = false;
	std::uint8_t ThrottleValue = 0, BrakeValue = 0;
	std::int8_t SteeringValue = 0;
};
inline FDrivingInputTargets ReadDrivingInputTargets(const std::optional<FInputFrame>& Frame,
	FFrameNumber ExpectedFrame)
{
	if (!Frame || !Frame->IsValid() || Frame->GetConsumptionFrame() != ExpectedFrame) return {};
	return {true, static_cast<std::uint8_t>(Frame->GetActions()[Throttle]),
		static_cast<std::uint8_t>(Frame->GetActions()[Brake]),
		static_cast<std::int8_t>(Frame->GetActions()[Steering])};
}
}
