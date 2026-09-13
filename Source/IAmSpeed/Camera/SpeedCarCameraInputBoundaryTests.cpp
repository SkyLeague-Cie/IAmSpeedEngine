#include "IAmSpeed/Components/SpeedWheeledComponent.h"

#if WITH_DEV_AUTOMATION_TESTS
#include "SpeedCarCameraPresentation.h"
#include "Misc/AutomationTest.h"
#include "Serialization/MemoryReader.h"
#include "Serialization/MemoryWriter.h"
#include "UObject/StrongObjectPtr.h"
#include <limits>

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedCameraInputBoundaryTest,
	"IAmSpeed.Camera.NetworkInputBoundary",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedCameraInputBoundaryTest::RunTest(const FString& Parameters)
{
	FSpeedCarCameraPhysicalInput A, B;
	TestTrue(TEXT("quantize A"), FSpeedCarCameraPhysicalInput::Quantize(true, 1, -1, A));
	TestTrue(TEXT("quantize B"), FSpeedCarCameraPhysicalInput::Quantize(false, -1, 1, B));
	TestTrue(TEXT("exact endpoints"), A.IsBack() && A.Yaw == 127 && A.Pitch == -127);
	const auto BeforeNaN = A;
	TestFalse(TEXT("NaN rejected"), FSpeedCarCameraPhysicalInput::Quantize(false,
		std::numeric_limits<float>::quiet_NaN(), 0, A));
	TestTrue(TEXT("NaN leaves destination unchanged"), A.Equals(BeforeNaN));

	TStrongObjectPtr<USpeedWheeledComponent> LiveOwner(NewObject<USpeedWheeledComponent>());
	TStrongObjectPtr<USpeedWheeledComponent> ReplayOwner(NewObject<USpeedWheeledComponent>());
	auto* Live = LiveOwner.Get();
	auto* Replay = ReplayOwner.Get();
	Live->EnableGenericCameraInput(true);
	Replay->EnableGenericCameraInput(true);
	Live->SinceCanMoveFrame = Replay->SinceCanMoveFrame = 100;
	Live->WheeledUserInput.Throttle = 201;
	Live->WheeledUserInput.Brake = 37;
	Live->WheeledUserInput.Steer = -71;
	Live->LastWheeledInputSlewFrame = 93;
	TNetRewindHistory<FNetworkWheeledSpeedInputState, true> History(32, true);
	TArray<FNetworkWheeledSpeedInputState> LastPackets;

	// The actual UE history container and production BuildData/ApplyData methods,
	// not synthetic presentation packets. This does not run a Chaos solver rewind.
	for (int32 Index = 0; Index < 5100; ++Index)
	{
		const auto Expected = Index % 2 ? B : A;
		Live->PublishHeldCameraInput(Expected);
		Live->BaseGameState.NumFrame = 120 + Index;
		const int32 SlewBeforeCapture = Live->LastWheeledInputSlewFrame;
		FNetworkWheeledSpeedInputState Packet;
		Packet.LocalFrame = 9000 + Index;
		Packet.BuildData(Live);
		TestTrue(TEXT("one capture supplies archived camera"), Packet.WheeledInput.Camera.Equals(Expected));
		TestTrue(TEXT("capture leaves wheel command queue empty"), Live->PendingWheeledInputCommands.IsEmpty());
		TestTrue(TEXT("capture never rewrites wheel controls or slew clock"),
			Live->WheeledUserInput.Throttle == 201 && Live->WheeledUserInput.Brake == 37 &&
			Live->WheeledUserInput.Steer == -71 && Live->LastWheeledInputSlewFrame == SlewBeforeCapture);
		TestEqual(TEXT("component mapping independent of Chaos offset"),
			Packet.ResolveActivationFrame(100, true), 121 + Index);
		TestTrue(TEXT("record real history"), History.RecordData(Packet.LocalFrame, &Packet));
		Live->PublishHeldCameraInput(Index % 2 ? A : B); // GT changes after BuildData.
		auto Duplicate = Packet;
		Duplicate.BuildData(Live);
		TestTrue(TEXT("duplicate BuildData same frame does not recapture GT"), Duplicate.WheeledInput.Camera.Equals(Expected));
		Live->BaseGameState.NumFrame = 121 + Index;
		Live->UpdateInputs();
		TestTrue(TEXT("Step applies archived value despite opposite mailbox"), Live->WheeledPhysicalInput.Camera.Equals(Expected));
		const uint64 ApplyCount = Live->CameraInputApplyCount;
		Live->ConsumeQueuedCameraInputsForFrame(121 + Index);
		TestEqual(TEXT("consumed latch applies once"), Live->CameraInputApplyCount, ApplyCount);
		TestTrue(TEXT("no growing camera input journal"), Live->PendingCameraInputCommands.IsEmpty());
		if (Index >= 5084) LastPackets.Add(Packet);
	}
	TestEqual(TEXT("one mailbox capture per normal history frame"), Live->CameraMailboxReadCount, uint64(5100));
	for (const auto& Recorded : LastPackets)
	{
		FNetworkWheeledSpeedInputState Extracted;
		TestTrue(TEXT("extract actual rewind-history entry"), History.ExtractData(Recorded.LocalFrame, true, &Extracted, true));
		Replay->PublishHeldCameraInput(Recorded.WheeledInput.Camera.IsBack() ? B : A);
		Extracted.ApplyData(Replay);
		Replay->BaseGameState.NumFrame = Extracted.ResolveActivationFrame(100, false);
		Replay->UpdateInputs();
		TestTrue(TEXT("historical ApplyData wins over opposite live mailbox"),
			Replay->WheeledPhysicalInput.Camera.Equals(Recorded.WheeledInput.Camera));
		TArray<uint8> Bytes;
		Replay->AppendPresentationSnapshot(Bytes);
		FSpeedCarCameraInputSnapshot Published;
		TestTrue(TEXT("physical applied input is published"), FSpeedCarCameraInputSnapshot::Read(Bytes, Published));
		TestTrue(TEXT("publication matches applied camera, not desired GT"),
			Published.bBackCamera == Recorded.WheeledInput.Camera.IsBack() &&
			Published.CameraYaw == float(Recorded.WheeledInput.Camera.Yaw) / 127.0f &&
			Published.CameraPitch == float(Recorded.WheeledInput.Camera.Pitch) / 127.0f);
	}
	TestEqual(TEXT("historical application reads no live mailbox"), Replay->CameraMailboxReadCount, uint64(0));

	FNetworkWheeledSpeedInputState Packet = LastPackets.Last();
	Packet.ClientFramesSinceCanMove = INDEX_NONE;
	TestEqual(TEXT("local countdown before start still uses next component frame"),
		Packet.ResolveActivationFrame(INDEX_NONE, true), int32(Packet.ClientFrame + 1));
	TestEqual(TEXT("remote without component provenance keeps legacy fallback"),
		Packet.ResolveActivationFrame(INDEX_NONE, false), Packet.LocalFrame);
	Packet.ClientFrame = MAX_uint32;
	TestEqual(TEXT("overflow rejected rather than wrapping timeline"), Packet.ResolveActivationFrame(INDEX_NONE, true), INDEX_NONE);
	Packet = LastPackets.Last();
	Packet.WheeledInput.bCanMove = true;
	TArray<uint8> Encoded;
	FMemoryWriter Writer(Encoded, true);
	bool bSuccess = false;
	TestTrue(TEXT("complete input serializes"), Packet.NetSerialize(Writer, nullptr, bSuccess) && bSuccess);
	const auto EqualPacket = [](const FNetworkWheeledSpeedInputState& L, const FNetworkWheeledSpeedInputState& R)
	{
		return L.LocalFrame == R.LocalFrame && L.ClientFrame == R.ClientFrame &&
			L.ClientFramesSinceCanMove == R.ClientFramesSinceCanMove && L.bIsAutonomousProxy == R.bIsAutonomousProxy &&
			L.WheeledInput.bCanMove == R.WheeledInput.bCanMove && L.WheeledInput.Throttle == R.WheeledInput.Throttle &&
			L.WheeledInput.Brake == R.WheeledInput.Brake && L.WheeledInput.Steer == R.WheeledInput.Steer &&
			L.WheeledInput.Camera.Equals(R.WheeledInput.Camera);
	};
	FNetworkWheeledSpeedInputState Decoded;
	FMemoryReader Reader(Encoded, true);
	TestTrue(TEXT("complete input roundtrip"), Decoded.NetSerialize(Reader, nullptr, bSuccess) && bSuccess && EqualPacket(Decoded, Packet));
	for (int32 Length = 0; Length < Encoded.Num(); ++Length)
	{
		TArray<uint8> Truncated;
		Truncated.Append(Encoded.GetData(), Length);
		FMemoryReader ShortReader(Truncated, true);
		auto Destination = Packet;
		Destination.LocalFrame = 7;
		const auto Before = Destination;
		TestFalse(TEXT("truncated whole input rejected"), Destination.NetSerialize(ShortReader, nullptr, bSuccess));
		TestTrue(TEXT("truncation changes no destination fields"), EqualPacket(Destination, Before));
	}
	for (int32 TailOffset = 0; TailOffset < 4; ++TailOffset)
	{
		auto Corrupt = Encoded;
		Corrupt[Corrupt.Num() - 4 + TailOffset] = TailOffset < 2 ? 255 : 128;
		FMemoryReader BadReader(Corrupt, true);
		auto Destination = Packet;
		TestFalse(TEXT("unknown version flags or forbidden axes rejected"), Destination.NetSerialize(BadReader, nullptr, bSuccess));
		TestTrue(TEXT("invalid tail leaves complete destination unchanged"), EqualPacket(Destination, Packet));
	}
	FNetworkWheeledSpeedInputState Neutral = Packet;
	Neutral.WheeledInput.Camera = FSpeedCarCameraPhysicalInput();
	Neutral.ApplyData(Replay);
	Replay->ConsumeQueuedCameraInputsForFrame(Neutral.ResolveActivationFrame(100, false));
	TestTrue(TEXT("absent extension applies canonical neutral"), Replay->WheeledPhysicalInput.Camera.Equals(Neutral.WheeledInput.Camera));
	Live->SetHeldCameraYaw(0);
	Live->SetHeldCameraPitch(0);
	Live->SetHeldCameraBack(false);
	Live->BaseGameState.NumFrame = 6000;
	Packet.LocalFrame = 20000;
	Packet.BuildData(Live);
	TestTrue(TEXT("Completed endpoints release to exact zero"), !Packet.WheeledInput.Camera.IsBack() &&
		Packet.WheeledInput.Camera.Yaw == 0 && Packet.WheeledInput.Camera.Pitch == 0);
	return true;
}
#endif
