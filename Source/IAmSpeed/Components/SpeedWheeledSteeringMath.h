#pragma once

#include "CoreMinimal.h"

namespace IAmSpeedSteering
{
	struct FUnsteeredAligningYawResponse
	{
		float SlipAuthority = 0.0f;
		float TargetYawRate = 0.0f;
		float AngularAcceleration = 0.0f;
	};

	/** Pure wheel-frame yaw response. Slip controls the target, never the damping. */
	FORCEINLINE FUnsteeredAligningYawResponse ComputeUnsteeredAligningYawResponse(
		const float SideSpeed,
		const float MinSlipSpeed,
		const float FullSlipSpeed,
		const float AligningMaxRate,
		const float CurrentYawRate,
		const float TimeConstant,
		const float DeltaTime)
	{
		const float SafeFullSlipSpeed = FMath::Max(1.0f, FullSlipSpeed);
		const float SafeMinSlipSpeed = FMath::Clamp(
			MinSlipSpeed, 0.0f, SafeFullSlipSpeed - KINDA_SMALL_NUMBER);
		const float SlipAuthority = FMath::Clamp(
			(FMath::Abs(SideSpeed) - SafeMinSlipSpeed)
				/ FMath::Max(1.0f, SafeFullSlipSpeed - SafeMinSlipSpeed),
			0.0f, 1.0f);
		const float TargetYawRate = -FMath::Sign(SideSpeed)
			* FMath::Max(0.0f, AligningMaxRate) * SlipAuthority;
		// The response stays finite even for direct diagnostic callers that do
		// not provide the positive fixed-step delta enforced by production.
		const float SafeTimeConstant = FMath::Max3(
			FMath::Max(0.0f, DeltaTime), TimeConstant, SMALL_NUMBER);
		return {SlipAuthority, TargetYawRate,
			(TargetYawRate - CurrentYawRate) / SafeTimeConstant};
	}
}
