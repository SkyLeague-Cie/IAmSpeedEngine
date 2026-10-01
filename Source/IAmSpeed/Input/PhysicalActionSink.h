#pragma once

#include "InputFrameV2.h"
#include "DrivingInputTargets.h"

namespace Speed::Input::V2
{
// Pure assembly only. The future physical consumer commits this complete
// result once; this function neither calls setters nor applies slew.
inline FDrivingInputTargets AssembleDrivingTargets(const FInputFrame& Frame,
	const FInputActionContract& Contract, FStreamEpoch ExpectedEpoch, FFrameNumber ExpectedFrame)
{
	if (!ExpectedEpoch.Value || !Frame.IsValidFor(Contract) || Frame.GetData().StreamEpoch.Value != ExpectedEpoch.Value
		|| Frame.GetData().ConsumptionFrame != ExpectedFrame) return {};
	FDrivingInputTargets Result;
	for (const auto& B : Contract.GetDescription().Physical)
	{
		const auto Value = Frame.GetData().Values[B.Action];
		switch (B.Destination)
		{
		case EPhysicalDestination::Throttle: Result.ThrottleValue = static_cast<std::uint8_t>(Value); break;
		case EPhysicalDestination::Brake: Result.BrakeValue = static_cast<std::uint8_t>(Value); break;
		case EPhysicalDestination::Steering: Result.SteeringValue = static_cast<std::int8_t>(Value); break;
		default: return {};
		}
	}
	Result.Valid = true;
	return Result;
}
}
