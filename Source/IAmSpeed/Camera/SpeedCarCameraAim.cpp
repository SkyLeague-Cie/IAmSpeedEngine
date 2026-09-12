#include "SpeedCarCameraAim.h"

namespace SpeedCarCameraAimMath
{
	constexpr float MinAirCameraVelocityXY = 50.0f;
	constexpr float BackwardTakeoffForwardDotThreshold = -0.25f;
	constexpr float AirCameraReturnForwardDotThreshold = 0.35f;
	constexpr float AirCameraReturnUpZThreshold = 0.25f;
}

FSpeedCarCameraAim::FSpeedCarCameraAim(const FSpeedCarCameraAimSettings& InSettings,
	const FSpeedCarCameraAimState& InitialState)
	: FSpeedCarCameraAimState(InitialState), Settings(InSettings)
{
}

bool FSpeedCarCameraAim::Step(const uint64 Frame, const FQuat& CarOrientation,
	const FVector& CarVelocity, const bool bOnGround, const bool bBackView,
	FRotator& OutBaseRotation)
{
	if (bTimelineInvalid || Frame == MAX_uint64 ||
		(LastFrame != MAX_uint64 && Frame != LastFrame + 1) ||
		CarOrientation.ContainsNaN() || !CarOrientation.IsNormalized() || CarVelocity.ContainsNaN() ||
		!FMath::IsFinite(CarVelocity.SizeSquared()) ||
		!FMath::IsFinite(Settings.Stiffness) || Settings.Stiffness < 0 || Settings.Stiffness > 1 ||
		!FMath::IsFinite(Settings.AirMinVelocityXY) || !FMath::IsFinite(Settings.AirVerticalPitchStartRatio) ||
		!FMath::IsFinite(Settings.AirVerticalPitchFullRatio) || !FMath::IsFinite(Settings.AirVerticalVelocityScale) ||
		CachedAirCameraHorizontalTarget.ContainsNaN() || CachedGroundCameraHorizontalForward.ContainsNaN() ||
		CarTarget.ContainsNaN() || CarRotator.ContainsNaN())
	{
		bTimelineInvalid = true;
		return false;
	}
	constexpr float Delta = 1.0f / 300.0f;
	ComputeCarRotation(CarOrientation.GetForwardVector(), CarOrientation.GetUpVector(), CarVelocity, bOnGround, Delta);
	if (CarTarget.ContainsNaN() || CarRotator.ContainsNaN() ||
		CachedAirCameraHorizontalTarget.ContainsNaN() || CachedGroundCameraHorizontalForward.ContainsNaN())
	{
		bTimelineInvalid = true;
		return false;
	}
	LastFrame = Frame;
	OutBaseRotation = bBackView ? RearView(CarRotator) : CarRotator;
	return true;
}

FRotator FSpeedCarCameraAim::RearView(const FRotator& CarRotation)
{
	return FRotator(CarRotation.Pitch, CarRotation.Yaw + 180.0, CarRotation.Roll);
}

void FSpeedCarCameraAim::AdvancePolicy(FSpeedCarCameraAimHistory& History,
	const FSpeedCarCameraAimSettings& PolicySettings, const FVector& CarForward,
	const FVector& CarUp, const FVector& CarVelocity, const bool bOnGround)
{
	// Temporary arithmetic context only. Standalone timeline/guards are neither
	// invoked nor copied back into the caller's authoritative outer state.
	FSpeedCarCameraAimState Values;
	static_cast<FSpeedCarCameraAimHistory&>(Values) = History;
	FSpeedCarCameraAim Policy(PolicySettings, Values);
	constexpr float Delta = 1.0f / 300.0f;
	Policy.ComputeCarRotation(CarForward, CarUp, CarVelocity, bOnGround, Delta);
	History = static_cast<const FSpeedCarCameraAimHistory&>(Policy.GetState());
}

void FSpeedCarCameraAim::ComputeCarTarget(const FVector& carForwardVector, const FVector& carUpVector,
	const FVector& carVelocity, bool isOnGround)
{
	if (!bHasCameraGroundState)
	{
		bHasCameraGroundState = true;
		bWasOnGroundForCamera = isOnGround;
	}

	if (isOnGround)
	{
		bWasOnGroundForCamera = true;
		bAirCameraUsesReverseVelocity = false;
		bAirCameraForwardLockedAfterFlip = false;

		auto UpZ = carUpVector.Z;
		auto ForwardZ = carForwardVector.Z;
		if (UpZ >= 0)
		{
			CarTarget = carForwardVector - ForwardZ * (1 - UpZ) * carUpVector;
		}
		else
		{
			CarTarget = carForwardVector - ForwardZ * (1 + UpZ) * carUpVector;
		}

		CachedAirCameraHorizontalTarget = FVector(CarTarget.X, CarTarget.Y, 0.0f).GetSafeNormal(UE_SMALL_NUMBER, CachedAirCameraHorizontalTarget);
		CachedGroundCameraHorizontalForward = CachedAirCameraHorizontalTarget;
	}
	else
	{
		FVector2D carVelocityXY(carVelocity.X, carVelocity.Y);
		const float CarVelocityXYLength = carVelocityXY.Length();
		const float MinAirVelocityXY = FMath::Max(SpeedCarCameraAimMath::MinAirCameraVelocityXY,
			Settings.AirMinVelocityXY);
		if (CarVelocityXYLength < MinAirVelocityXY)
		{
			CarTarget = CachedAirCameraHorizontalTarget;
		}
		else
		{
			const FVector2D carForwardXY(carForwardVector.X, carForwardVector.Y);
			const float ForwardDotVelocity = carForwardXY.GetSafeNormal().Dot(carVelocityXY.GetSafeNormal());
			if (bWasOnGroundForCamera)
			{
				bAirCameraUsesReverseVelocity = ForwardDotVelocity < SpeedCarCameraAimMath::BackwardTakeoffForwardDotThreshold;
			}
			else if (bAirCameraUsesReverseVelocity &&
				ForwardDotVelocity > SpeedCarCameraAimMath::AirCameraReturnForwardDotThreshold &&
				carUpVector.Z > SpeedCarCameraAimMath::AirCameraReturnUpZThreshold)
			{
				bAirCameraUsesReverseVelocity = false;
			}

			const float AirCameraDirection = bAirCameraUsesReverseVelocity ? -1.0f : 1.0f;
			const FVector HorizontalVelocityTarget(AirCameraDirection * carVelocity.X, AirCameraDirection * carVelocity.Y, 0.0f);
			const FVector HorizontalDirection = HorizontalVelocityTarget.GetSafeNormal(UE_SMALL_NUMBER, CachedAirCameraHorizontalTarget);
			CachedAirCameraHorizontalTarget = HorizontalDirection;

			const float VerticalPitchStartRatio = FMath::Max(0.0f, Settings.AirVerticalPitchStartRatio);
			const float VerticalPitchFullRatio = FMath::Max(VerticalPitchStartRatio + UE_SMALL_NUMBER,
				Settings.AirVerticalPitchFullRatio);
			const float VerticalVelocityScale = FMath::Clamp(Settings.AirVerticalVelocityScale, 0.0f, 1.0f);
			const float VerticalRatio = FMath::Abs(carVelocity.Z) / FMath::Max(CarVelocityXYLength, 1.0f);
			const float VerticalAlpha = FMath::SmoothStep(0.0f, 1.0f,
				FMath::Clamp((VerticalRatio - VerticalPitchStartRatio) / (VerticalPitchFullRatio - VerticalPitchStartRatio), 0.0f, 1.0f));
			CarTarget = HorizontalVelocityTarget + FVector(0.0f, 0.0f, carVelocity.Z * VerticalVelocityScale * VerticalAlpha);
		}

		bWasOnGroundForCamera = false;
	}
}

void FSpeedCarCameraAim::ComputeCarRotation(const FVector& carForwardVector, const FVector& carUpVector,
	const FVector& carVelocity, bool isOnGround, const float& delta)
{
	ComputeCarTarget(carForwardVector, carUpVector, carVelocity, isOnGround);
	FRotator tmpCarRotator(CarTarget.Rotation());
	float camInterpSpeed = isOnGround ? (10 * Settings.Stiffness + 10) : (5 * Settings.Stiffness + 3);
	tmpCarRotator = FMath::RInterpTo(CarRotator,
		tmpCarRotator, delta, camInterpSpeed);
	CarRotator.Roll = 0;
	CarRotator.Pitch = tmpCarRotator.Pitch;
	CarRotator.Yaw = tmpCarRotator.Yaw;
}
