#pragma once

#include "Kinematic.h"

namespace Speed
{
	/** Remembers only proven no-op quantizations; derived per-component state, never serialized. */
	class FIdentityKinematicQuantizationCache
	{
	public:
		/** Returns true when an identical previously observed no-op allows quantization to be omitted. */
		bool Quantize(FKinematicState& State, const FQuat& PreviousRotation)
		{
			if (bValid && NoOpInput.Matches(State, PreviousRotation)) return true;
			const FInput Before(State, PreviousRotation);
			State.Quantize(PreviousRotation);
			// Do not assume quantization is idempotent: prove its output equalled its input.
			bValid = Before.Matches(State, PreviousRotation);
			if (bValid) NoOpInput = Before;
			return false;
		}
		/** Revoke eligibility at an explicit restore/reinitialization boundary. */
		void Reset() { bValid = false; }
	private:
		struct FInput
		{
			FVector Location = FVector::ZeroVector;
			FVector Velocity = FVector::ZeroVector;
			FVector AngularVelocity = FVector::ZeroVector;
			FQuat Rotation = FQuat::Identity;
			FQuat PreviousRotation = FQuat::Identity;
			FInput() = default;
			FInput(const FKinematicState& State, const FQuat& Previous)
				: Location(State.Location), Velocity(State.Velocity), AngularVelocity(State.AngularVelocity),
				Rotation(State.Rotation), PreviousRotation(Previous) {}
			bool Matches(const FKinematicState& State, const FQuat& Previous) const
			{
				// Compare fields, not FKinematicState padding. Accelerations are deliberately
				// excluded: the canonical quantizer neither reads nor writes those fields.
				const auto EqualVector = [](const FVector& A, const FVector& B)
				{
					return FMemory::Memcmp(&A, &B, sizeof(FVector)) == 0;
				};
				return EqualVector(Location, State.Location) && EqualVector(Velocity, State.Velocity) &&
					EqualVector(AngularVelocity, State.AngularVelocity) &&
					FMemory::Memcmp(&Rotation, &State.Rotation, sizeof(FQuat)) == 0 &&
					FMemory::Memcmp(&PreviousRotation, &Previous, sizeof(FQuat)) == 0;
			}
		};
		FInput NoOpInput;
		bool bValid = false;
	};
	static_assert(sizeof(FIdentityKinematicQuantizationCache) <= 192, "Keep no-op quantization memory bounded per component");
}
