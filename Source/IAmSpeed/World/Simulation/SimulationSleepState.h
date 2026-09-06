#pragma once

#include "IAmSpeed/Base/Kinematic.h"

namespace Speed
{
	/** Recomputable sleep eligibility; never an independent physical state. */
	class FSimulationSleepState
	{
	public:
#if !UE_BUILD_SHIPPING
		/** Thread-local instrumentation, excluded from physical and Shipping state. */
		static uint64& ThreadIntegrationSkips()
		{
			static thread_local uint64 Count = 0;
			return Count;
		}
#endif
		/** Remember the last completed pose, not a predicted/intermediate pose. */
		void ObserveCompletedPose(const SKinematic& State)
		{
			Location = State.Location;
			Rotation = State.Rotation;
			bHasCompletedPose = true;
		}
		/** Restoration invalidates the optimization until another frame completes. */
		void Reset() { bHasCompletedPose = false; }
		/** Forces, impulses and pose changes revoke sleep immediately, without a timer. */
		bool CanSleep(const SKinematic& State, const bool bStableSupport) const
		{
			return bStableSupport && bHasCompletedPose &&
				State.Location == Location && State.Rotation == Rotation &&
				State.Velocity.IsZero() && State.AngularVelocity.IsZero() &&
				State.Acceleration.IsZero() && State.AngularAcceleration.IsZero();
		}
	private:
		FVector Location = FVector::ZeroVector;
		FQuat Rotation = FQuat::Identity;
		bool bHasCompletedPose = false;
	};
}
