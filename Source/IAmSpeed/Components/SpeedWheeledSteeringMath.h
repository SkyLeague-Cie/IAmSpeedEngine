#pragma once

#include "CoreMinimal.h"

namespace IAmSpeedSteering
{
	FORCEINLINE float ComputeUnsteeredAligningYawAcceleration(
		const float TargetYawRate,
		const float CurrentYawRate,
		const float TimeConstant,
		const float SlipAuthority,
		const bool bPreserveResidualYawDamping = false)
	{
		if (bPreserveResidualYawDamping)
		{
			// The caller already scales the target by slip authority. Keep the
			// remaining yaw error damped when that target returns to zero.
			return (TargetYawRate - CurrentYawRate) / FMath::Max(TimeConstant, SMALL_NUMBER);
		}
		const float BoundedAuthority = FMath::Clamp(SlipAuthority, 0.0f, 1.0f);
		return ((TargetYawRate - CurrentYawRate)
			/ FMath::Max(TimeConstant, SMALL_NUMBER)) * BoundedAuthority;
	}
}
