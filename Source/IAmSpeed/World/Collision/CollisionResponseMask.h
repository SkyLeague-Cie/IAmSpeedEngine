#pragma once

#include "Engine/EngineTypes.h"
#if PLATFORM_CPU_X86_FAMILY
#include <emmintrin.h>
#endif

namespace Speed
{
	// ECC_MAX includes a deprecated, non-stored flag; only the response array is indexable.
	inline constexpr uint8 CollisionResponseChannelCount = UE_ARRAY_COUNT(FCollisionResponseContainer::EnumArray);
	static_assert(CollisionResponseChannelCount <= 64, "Blocking mask must represent every stored collision channel");

	/** Converts only stored Unreal response channels; overlaps do not block a single sweep. */
	inline uint64 GetBlockingResponseMask(const FCollisionResponseContainer& Responses)
	{
#if PLATFORM_CPU_X86_FAMILY
		if constexpr (CollisionResponseChannelCount == 32)
		{
			// SSE2 compares the stored bytes directly, including invalid enum bytes.
			// Two unaligned loads read exactly the 32 valid channels, never ECC_MAX.
			static_assert(sizeof(Responses.EnumArray[0]) == 1, "SIMD requires byte-sized responses");
			const __m128i Blocking = _mm_set1_epi8(static_cast<char>(ECR_Block));
			const __m128i Low = _mm_loadu_si128(reinterpret_cast<const __m128i*>(Responses.EnumArray));
			const __m128i High = _mm_loadu_si128(reinterpret_cast<const __m128i*>(Responses.EnumArray + 16));
			return uint64(_mm_movemask_epi8(_mm_cmpeq_epi8(Low, Blocking))) |
				(uint64(_mm_movemask_epi8(_mm_cmpeq_epi8(High, Blocking))) << 16);
		}
		else
#endif
		{
			uint64 Mask = 0;
			for (uint8 Channel = 0; Channel < CollisionResponseChannelCount; ++Channel)
			{
				if (Responses.GetResponse(static_cast<ECollisionChannel>(Channel)) == ECR_Block)
					Mask |= uint64(1) << Channel;
			}
			return Mask;
		}
	}
}
