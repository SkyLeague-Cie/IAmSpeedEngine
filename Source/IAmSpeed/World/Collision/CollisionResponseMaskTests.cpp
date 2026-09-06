#if WITH_DEV_AUTOMATION_TESTS
#include "CollisionResponseMask.h"
#include "Misc/AutomationTest.h"
#include "Math/RandomStream.h"
#include "HAL/PlatformTime.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedResponseMaskTest,
	"IAmSpeed.ContactDetection.ResponseMask",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedResponseMaskTest::RunTest(const FString& Parameters)
{
	FCollisionResponseContainer Responses(ECR_Ignore);
	TestEqual(TEXT("ignore-only responses have no blockers"), Speed::GetBlockingResponseMask(Responses), uint64(0));
	Responses.SetAllChannels(ECR_Overlap);
	TestEqual(TEXT("overlaps are not blockers"), Speed::GetBlockingResponseMask(Responses), uint64(0));
	// Exercise every stored response, including channel 31 but not the deprecated flag at 32.
	for (uint8 Channel = 0; Channel < Speed::CollisionResponseChannelCount; ++Channel)
	{
		Responses.SetAllChannels(ECR_Ignore);
		Responses.SetResponse(static_cast<ECollisionChannel>(Channel), ECR_Block);
		const uint64 Expected = uint64(1) << Channel;
		TestEqual(TEXT("each collision channel has its exact bit"), Speed::GetBlockingResponseMask(Responses), Expected);
		const FCollisionResponseContainer Copy = Responses;
		TestEqual(TEXT("equal content in another object returns the same mask"), Speed::GetBlockingResponseMask(Copy), Expected);
		Responses.SetResponse(static_cast<ECollisionChannel>(Channel), ECR_Overlap);
		TestEqual(TEXT("mutation replaces a blocking response"), Speed::GetBlockingResponseMask(Responses), uint64(0));
	}
	for (uint32 Pattern = 0; Pattern < 64; ++Pattern)
	{
		uint64 Expected = 0;
		for (uint8 Channel = 0; Channel < Speed::CollisionResponseChannelCount; ++Channel)
		{
			const auto Response = static_cast<ECollisionResponse>((Pattern * 17 + Channel * 7) % 3);
			Responses.SetResponse(static_cast<ECollisionChannel>(Channel), Response);
			if (Response == ECR_Block) Expected |= uint64(1) << Channel;
		}
		TestEqual(TEXT("mixed responses retain the full exact mask"), Speed::GetBlockingResponseMask(Responses), Expected);
	}
	// Exhaust every byte value in every stored position. Even an invalid enum
	// byte must not be mistaken for ECR_Block, and channel 32 is never read.
	Responses.SetAllChannels(ECR_Ignore);
	for (uint8 Channel = 0; Channel < Speed::CollisionResponseChannelCount; ++Channel)
	{
		for (uint32 Value = 0; Value <= MAX_uint8; ++Value)
		{
			Responses.EnumArray[Channel] = static_cast<uint8>(Value);
			TestEqual(TEXT("only the exact blocking byte contributes to the mask"),
				Speed::GetBlockingResponseMask(Responses),
				Value == ECR_Block ? uint64(1) << Channel : uint64(0));
		}
		Responses.EnumArray[Channel] = ECR_Ignore;
	}
	return true;
}

namespace
{
	uint64 OriginalResponseMask(const FCollisionResponseContainer& Responses)
	{
		uint64 Mask = 0;
		for (uint8 Channel = 0; Channel < Speed::CollisionResponseChannelCount; ++Channel)
			if (Responses.GetResponse(static_cast<ECollisionChannel>(Channel)) == ECR_Block)
				Mask |= uint64(1) << Channel;
		return Mask;
	}

	template<bool Optimized>
	FORCENOINLINE double MeasureResponseMask(const TArray<FCollisionResponseContainer>& Responses, uint64& Checksum)
	{
		uint64 Sum = 0;
		const double Start = FPlatformTime::Seconds();
		for (int32 Repeat = 0; Repeat < 64; ++Repeat)
			for (const auto& Response : Responses)
			{
				if constexpr (Optimized) Sum = Sum * 33 + Speed::GetBlockingResponseMask(Response);
				else Sum = Sum * 33 + OriginalResponseMask(Response);
			}
		const double Elapsed = FPlatformTime::Seconds() - Start;
		Checksum = Sum;
		return Elapsed * 1.e9 / (64 * Responses.Num());
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedResponseMaskCostTest,
	"IAmSpeed.ContactDetection.ResponseMaskCost", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedResponseMaskCostTest::RunTest(const FString& Parameters)
{
	FRandomStream Random(150952);
	TArray<FCollisionResponseContainer> Responses;
	for (int32 I = 0; I < 4096; ++I)
	{
		auto& Response = Responses.Emplace_GetRef(ECR_Ignore);
		for (auto& Byte : Response.EnumArray)
			Byte = static_cast<uint8>(Random.GetUnsignedInt() % (I % 2 ? 3 : 256));
		TestEqual(TEXT("simultaneous byte matches retain every bit"), Speed::GetBlockingResponseMask(Response), OriginalResponseMask(Response));
	}
	alignas(FCollisionResponseContainer) uint8 Storage[sizeof(FCollisionResponseContainer) + 16 * alignof(FCollisionResponseContainer)];
	for (int32 Offset = 0; Offset < 16; ++Offset)
	{
		auto* Response = new (Storage + Offset * alignof(FCollisionResponseContainer)) FCollisionResponseContainer(ECR_Block);
		TestEqual(TEXT("non-SIMD-aligned container keeps all channel bits"), Speed::GetBlockingResponseMask(*Response), OriginalResponseMask(*Response));
		Response->~FCollisionResponseContainer();
	}
	// Both the common all-ignore case and irregular masks are measured; timing
	// is diagnostic only, with no CPU-frequency-dependent pass threshold.
	for (const bool AllIgnore : { false, true })
	{
		if (AllIgnore) for (auto& Response : Responses) Response.SetAllChannels(ECR_Ignore);
		uint64 OriginalSum = 0, OptimizedSum = 0;
		MeasureResponseMask<false>(Responses, OriginalSum);
		MeasureResponseMask<true>(Responses, OptimizedSum);
		for (int32 Pair = 0; Pair < 5; ++Pair)
		{
			double Original, Optimized;
			if (Pair % 2 == 0)
			{
				Original = MeasureResponseMask<false>(Responses, OriginalSum);
				Optimized = MeasureResponseMask<true>(Responses, OptimizedSum);
			}
			else
			{
				Optimized = MeasureResponseMask<true>(Responses, OptimizedSum);
				Original = MeasureResponseMask<false>(Responses, OriginalSum);
			}
			TestEqual(TEXT("response microbenchmark checksums"), OriginalSum, OptimizedSum);
			AddInfo(FString::Printf(TEXT("[ResponseMaskMicro] AllIgnore=%d Pair=%d OriginalNs=%.6f OptimizedNs=%.6f"), AllIgnore, Pair, Original, Optimized));
		}
	}
	return true;
}
#endif
