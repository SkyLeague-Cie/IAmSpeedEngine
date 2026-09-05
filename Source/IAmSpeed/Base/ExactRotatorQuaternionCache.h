#pragma once

#include "CoreMinimal.h"

namespace Speed
{
	/** Pure Euler-to-quaternion memoization; no component state or Unreal rotation-cache history. */
	class FExactRotatorQuaternionCache
	{
	public:
		/** Recomputes for any changed input bits, including signed zero; storage never grows. */
		FQuat Get(const FRotator& Rotation)
		{
			for (const FEntry& Entry : Entries)
			{
				if (Entry.bValid && FMemory::Memcmp(&Entry.Input, &Rotation, sizeof(FRotator)) == 0)
					return Entry.Output;
			}
			const FQuat Out = Rotation.Quaternion();
			FEntry& Entry = Entries[NextEntry];
			NextEntry = (NextEntry + 1) % UE_ARRAY_COUNT(Entries);
			Entry.Input = Rotation;
			Entry.Output = Out;
			Entry.bValid = true;
			return Out;
		}
	private:
		struct FEntry
		{
			FRotator Input = FRotator::ZeroRotator;
			FQuat Output = FQuat::Identity;
			bool bValid = false;
		};
		FEntry Entries[4];
		uint8 NextEntry = 0;
	};
	static_assert(sizeof(FExactRotatorQuaternionCache) <= 512, "Keep local rotation memoization below 512 bytes per lane");
}
