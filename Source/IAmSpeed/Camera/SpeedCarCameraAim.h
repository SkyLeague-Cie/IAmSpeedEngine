#pragma once

#include "CoreMinimal.h"

/** Copied settings only. No live configuration or target belongs in this policy. */
struct IAMSPEED_API FSpeedCarCameraAimSettings
{
	float Stiffness = .55f;
	float AirMinVelocityXY = 120.0f;
	float AirVerticalPitchStartRatio = 1.0f;
	float AirVerticalPitchFullRatio = 2.2f;
	float AirVerticalVelocityScale = .1f;
};

/** Complete direction history, independent of an arm, UObject or output codec. */
struct IAMSPEED_API FSpeedCarCameraAimHistory
{
	bool bHasCameraGroundState = false;
	bool bWasOnGroundForCamera = false;
	bool bAirCameraUsesReverseVelocity = false;
	// Retained extraction field: reset by the accepted ground policy, not a new latch.
	bool bAirCameraForwardLockedAfterFlip = false;
	FVector CachedAirCameraHorizontalTarget = FVector::ForwardVector;
	FVector CachedGroundCameraHorizontalForward = FVector::ForwardVector;
	FVector CarTarget = FVector::ForwardVector;
	FRotator CarRotator = FRotator::ZeroRotator;
};

/** Standalone admission state; an outer camera owns its own timeline instead. */
struct IAMSPEED_API FSpeedCarCameraAimState : public FSpeedCarCameraAimHistory
{
	uint64 LastFrame = MAX_uint64;
	bool bTimelineInvalid = false;
};

/** Car-only direction policy. This additive extraction is not yet activated.
 * State copies are in-process values, NOT authenticated rollback packets.
 * Host arm/publication and collision policy deliberately remain separate. */
class IAMSPEED_API FSpeedCarCameraAim : private FSpeedCarCameraAimState
{
public:
	FSpeedCarCameraAim(const FSpeedCarCameraAimSettings& InSettings,
		const FSpeedCarCameraAimState& InitialState = FSpeedCarCameraAimState());
	bool Step(uint64 Frame, const FQuat& CarOrientation, const FVector& CarVelocity,
		bool bOnGround, bool bBackView, FRotator& OutBaseRotation);
	const FSpeedCarCameraAimState& GetState() const { return *this; }
	static FRotator RearView(const FRotator& CarRotation);
	/** Pure fixed-step arithmetic on caller-owned history. No frame admission or
	 * additional validation: the outer caller owns those guards and their order.
	 * This seam does not install a producer or retain a second history. */
	static void AdvancePolicy(FSpeedCarCameraAimHistory& History,
		const FSpeedCarCameraAimSettings& PolicySettings, const FVector& CarForward,
		const FVector& CarUp, const FVector& CarVelocity, bool bOnGround);
private:
	const FSpeedCarCameraAimSettings Settings;
	void ComputeCarTarget(const FVector& carForwardVector, const FVector& carUpVector,
		const FVector& carVelocity, bool isOnGround);
	void ComputeCarRotation(const FVector& carForwardVector, const FVector& carUpVector,
		const FVector& carVelocity, bool isOnGround, const float& delta);
};
