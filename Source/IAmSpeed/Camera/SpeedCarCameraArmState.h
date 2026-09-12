#pragma once

#include "SpeedCarCameraAim.h"

/** Single caller-owned arm/filter/timeline state for a physical car camera.
 * No target, world object, output producer or wire layout belongs here.
 * Game extensions inherit this once and serialize explicit fields.
 * Zeroed aim seeds preserve the outer-state initialization contract; a host's
 * configured initial state supplies its meaningful forward seed separately. */
struct IAMSPEED_API FSpeedCarCameraArmState : public FSpeedCarCameraAimHistory
{
    FSpeedCarCameraArmState()
    {
        CachedAirCameraHorizontalTarget = FVector::ZeroVector;
        CachedGroundCameraHorizontalForward = FVector::ZeroVector;
        CarTarget = FVector::ZeroVector;
    }

	float BaseDistance{};
	float ConfiguredCameraHeight{};
	float Stiffness{};
	float SwivelSpeed{};
	float TransitionSpeed{};
	bool bIsSwivelInverted{};
	bool bIsOnBackCam{};
	bool bIsSwitchingCam{};
	float SwitchTransitionRemainingSeconds{};
	bool bHasToDampenOffset{};
	float DesiredRegularArmLength{};
	FRotator RotOffset = FRotator::ZeroRotator;
	FRotator PhysRotator = FRotator::ZeroRotator;
	float TargetArmLength = 350;
	FVector TargetOffset = FVector(0, 0, 150);
	FVector SocketOffset = FVector::ZeroVector;
	float CameraLagSpeed = 32.2f;
	float CameraRotationLagSpeed = 37;
	float LagSpeedCoeff = 1;
	float ProbeSize = 25;
	FRotator PreviousDesiredRot = FRotator::ZeroRotator;
	uint64 LastFrame = MAX_uint64;
	uint64 LastAppliedCommandSerial = 0;
	uint64 LastAppliedCommandFrame = MAX_uint64;
	bool bTimelineInvalid = false;
};
