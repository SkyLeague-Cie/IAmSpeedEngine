#pragma once

#include "SUtils.h"

namespace Speed
{
	/** Bounded cache of the pure quaternion/packed-Euler conversion, not of simulation state. */
	class FQuaternionQuantizationCache
	{
	public:
		/** Reuses only identical input bits; the caller's current hemisphere reference is always applied. */
		FQuat Get(const FQuat& Quat, const FQuat& Reference)
		{
			FQuat Out;
			bool bFound = false;
			for (const FEntry& Entry : Entries)
			{
				if (Entry.bValid && FMemory::Memcmp(&Entry.Input, &Quat, sizeof(FQuat)) == 0)
				{
					Out = Entry.Output;
					bFound = true;
					break;
				}
			}
			if (!bFound)
			{
				Out = QuantizeWithoutHemisphere(Quat);
				FEntry& Entry = Entries[NextEntry];
				NextEntry = (NextEntry + 1) % UE_ARRAY_COUNT(Entries);
				Entry.Input = Quat;
				Entry.Output = Out;
				Entry.bValid = true;
			}
			if (QuatDot(Out, Reference) < 0.f)
			{
				Out.X *= -1; Out.Y *= -1; Out.Z *= -1; Out.W *= -1;
			}
			return Out;
		}
	private:
		static FQuat QuantizeWithoutHemisphere(const FQuat& Quat)
		{
			// Keep the original operation order and float intermediates: they define network quantization.
			FRotator R = Quat.Rotator();
			R.Normalize();
			R.Pitch = FRotator::NormalizeAxis(R.Pitch);
			R.Yaw = FRotator::NormalizeAxis(R.Yaw);
			R.Roll = FRotator::NormalizeAxis(R.Roll);
			const uint16 SP = FRotator::CompressAxisToShort(R.Pitch);
			const uint16 SY = FRotator::CompressAxisToShort(R.Yaw);
			const uint16 SR = FRotator::CompressAxisToShort(R.Roll);
			const float Pitch = FRotator::DecompressAxisFromShort(SP);
			const float Yaw = FRotator::DecompressAxisFromShort(SY);
			const float Roll = FRotator::DecompressAxisFromShort(SR);
			FQuat Out = FRotator(Pitch, Yaw, Roll).Quaternion();
			Out.Normalize();
			return Out;
		}
		struct FEntry
		{
			FQuat Input = FQuat::Identity;
			FQuat Output = FQuat::Identity;
			bool bValid = false;
		};
		FEntry Entries[4];
		uint8 NextEntry = 0;
	};
	static_assert(sizeof(FQuaternionQuantizationCache) <= 512, "Keep quaternion memoization below 512 bytes per lane");
}
