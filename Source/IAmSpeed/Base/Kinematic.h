#pragma once

#include "CoreMinimal.h"

namespace Speed
{
	struct FKinematicState
	{
		FKinematicState() = default;

		// Location in absolute space
		FVector_NetQuantize100 Location = FVector::ZeroVector;
		// Velocity in absolute space
		FVector_NetQuantize100 Velocity = FVector::ZeroVector; // velocity in absolute space
		// Velocity in absolute space
		FVector_NetQuantize Acceleration = FVector::ZeroVector; // acceleration in absolute space

		// Rotation in absolute space
		FQuat Rotation = FQuat::Identity;
		// Angular velocity in rad/s
		FVector_NetQuantize100 AngularVelocity = FVector::ZeroVector; // angular velocity in rad/s
		// Angular acceleration in rad/s^2
		FVector_NetQuantize100 AngularAcceleration = FVector::ZeroVector; // angular acceleration in rad/s^2

		FKinematicState Integrate(const float& delta) const;

		bool Serialize(FArchive& Ar);
		void Quantize(const FQuat& PrevRotation);
		bool NetSerialize(FArchive& Ar, class UPackageMap* Map, bool& bOutSuccess);

		FString ToString() const;
	};

	inline FArchive& operator<<(FArchive& Ar, FKinematicState& KinematicState)
	{
		Ar << KinematicState.Location;
		Ar << KinematicState.Velocity;
		Ar << KinematicState.Acceleration;
		Ar << KinematicState.Rotation;
		Ar << KinematicState.AngularVelocity;
		Ar << KinematicState.AngularAcceleration;
		return Ar;
	}

	FKinematicState Lerp(const FKinematicState& A, const FKinematicState& B, float Alpha);
}

using SKinematic = Speed::FKinematicState;
