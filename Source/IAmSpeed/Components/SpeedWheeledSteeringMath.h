#pragma once

#include "CoreMinimal.h"

namespace IAmSpeedSteering
{
	FORCEINLINE float ComputeUnsteeredAligningYawAcceleration(
		const float TargetYawRate,
		const float CurrentYawRate,
		const float TimeConstant,
		const float SlipAuthority)
	{
		const float BoundedAuthority = FMath::Clamp(SlipAuthority, 0.0f, 1.0f);
		return ((TargetYawRate - CurrentYawRate)
			/ FMath::Max(TimeConstant, SMALL_NUMBER)) * BoundedAuthority;
	}
}
