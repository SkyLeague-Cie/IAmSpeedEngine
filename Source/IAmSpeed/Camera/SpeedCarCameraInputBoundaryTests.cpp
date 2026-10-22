#include "IAmSpeed/Components/SpeedWheeledComponent.h"

#if WITH_DEV_AUTOMATION_TESTS
#include "SpeedCarCameraPresentation.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include "Serialization/MemoryReader.h"
#include "Serialization/MemoryWriter.h"
#include "Serialization/BitReader.h"
#include "Serialization/BitWriter.h"
#include "UObject/StrongObjectPtr.h"
#include <limits>

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedInputFrameHeaderBoundaryTest,
 "IAmSpeed.Camera.InputFrameHeaderBoundary",
 EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)
bool FIAmSpeedInputFrameHeaderBoundaryTest::RunTest(const FString&)
{
 // Independent byte construction, not SerializeFrames/SerializeIntPacked.
 const auto Packed=[](uint32 Value)
 {
  TArray<uint8> Bytes;
  do{const uint8 Payload=uint8(Value&127);Value>>=7;Bytes.Add(uint8((Payload<<1)|(Value?1:0)));}while(Value);
  return Bytes;
 };
 const auto Check=[&](int32 Frame,FNetworkWheeledSpeedInputState* Delta)
 {
  FNetworkWheeledSpeedInputState Packet;Packet.bInputProjectionValid=false;
  Packet.ServerFrame=Frame;Packet.SetDeltaSourceData(Delta);
  FBitWriter Written(1024,true),Reference(1024,true);uint8 Prefix=5;
  Written.SerializeBits(&Prefix,3);Reference.SerializeBits(&Prefix,3);
  uint8 Version=1,Presence=0;Reference<<Version<<Presence;
  if(Delta)
  {
   uint8 Increment=Frame==Delta->ServerFrame+1;Reference.SerializeBits(&Increment,1);
   if(!Increment)
   {
    const int32 Difference=Frame-Delta->ServerFrame;uint8 Negative=Difference<0;
    Reference.SerializeBits(&Negative,1);const auto Bytes=Packed(uint32(FMath::Abs(Difference)));
    Reference.SerializeBits(const_cast<uint8*>(Bytes.GetData()),Bytes.Num()*8);
   }
  }
  else
  {
   const auto Bytes=Packed(uint32(Frame+1));Reference.SerializeBits(const_cast<uint8*>(Bytes.GetData()),Bytes.Num()*8);
  }
  bool Success=false;
  TestTrue(TEXT("real frame header saves at nonzero bit offset"),Packet.NetSerialize(Written,nullptr,Success)&&Success&&!Written.IsError());
  TestEqual(TEXT("independent packed header bit count"),Written.GetNumBits(),Reference.GetNumBits());
  FBitReader ActualBits(Written.GetData(),Written.GetNumBits()),ReferenceBits(Reference.GetData(),Reference.GetNumBits());
  bool SameBits=Written.GetNumBits()==Reference.GetNumBits();
  while(SameBits&&ActualBits.GetBitsLeft())
  {uint8 L=0,R=0;ActualBits.SerializeBits(&L,1);ReferenceBits.SerializeBits(&R,1);SameBits=L==R;}
  TestTrue(TEXT("saving preserves independent LSB continuation and delta bits"),SameBits);
  FBitReader Reader(Reference.GetData(),Reference.GetNumBits());uint8 ReadPrefix=0;Reader.SerializeBits(&ReadPrefix,3);
  FNetworkWheeledSpeedInputState Destination;Destination.ServerFrame=Destination.LocalFrame=3111;
  Destination.SetDeltaSourceData(Delta);
  TestTrue(TEXT("independent nonaligned header decodes"),Destination.NetSerialize(Reader,nullptr,Success)&&Success&&!Reader.IsError());
  TestEqual(TEXT("nonaligned prefix preserved"),ReadPrefix,Prefix);
  TestEqual(TEXT("header transports server frame"),int32(Destination.ServerFrame),Frame);
  TestEqual(TEXT("header seeds endpoint local frame"),int32(Destination.LocalFrame),Frame);
  TestEqual(TEXT("header consumes exactly its bits"),Reader.GetBitsLeft(),int64(0));
 };
 const int32 BoundaryFrames[]={INDEX_NONE,0,126,127,128,16382,16383,MAX_int32-1};
 for(int32 Frame:BoundaryFrames)Check(Frame,nullptr);
 FNetworkWheeledSpeedInputState Delta;Delta.ServerFrame=200;
 for(int32 Frame:{201,199,0,40000})Check(Frame,&Delta);

 const auto Reject=[&](const TArray<uint8>& Bytes)
 {
  FMemoryReader Reader(Bytes);FNetworkWheeledSpeedInputState Destination;
  Destination.ServerFrame=Destination.LocalFrame=3111;Destination.ClientFrame=3999;
  const auto Before=Destination;bool Success=true;
  TestFalse(TEXT("malformed packed header rejected"),Destination.NetSerialize(Reader,nullptr,Success));
  TestTrue(TEXT("malformed header propagates archive failure"),Reader.IsError()&&!Success);
  TestTrue(TEXT("malformed header is transactional"),Destination.ServerFrame==Before.ServerFrame
   &&Destination.LocalFrame==Before.LocalFrame&&Destination.ClientFrame==Before.ClientFrame
   &&Destination.bInputProjectionValid==Before.bInputProjectionValid&&Destination.WheeledInput.Camera.Equals(Before.WheeledInput.Camera));
 };
 TArray<uint8> Complete{1,0};Complete.Append(Packed(uint32(MAX_int32)));
 for(int32 Length=0;Length<Complete.Num();++Length)
 {TArray<uint8> Short;Short.Append(Complete.GetData(),Length);Reject(Short);}
 for(int32 Width=0;Width<10;++Width)
 {TArray<uint8> Truncated{1,0};for(int32 Index=0;Index<Width;++Index)Truncated.Add(1);Reject(Truncated);}
 Reject(TArray<uint8>{1,0,1,1,1,1,1}); // Truncated after five continuations.
 Reject(TArray<uint8>{1,0,1,1,1,1,1,1,1,1,1,1}); // Ten continuations have no terminator.
 // Preserve the distinct generic-memory compatibility policy of UE archives.
 for(int32 Width=6;Width<=10;++Width)
 {
  TArray<uint8> Compatible{1,0};for(int32 Index=1;Index<Width;++Index)Compatible.Add(1);Compatible.Add(0);
  FMemoryReader CompatibilityReader(Compatible);FNetworkWheeledSpeedInputState Value;bool Saved=false;
  TestTrue(TEXT("memory archive retains legacy wide packed input"),Value.NetSerialize(CompatibilityReader,nullptr,Saved)&&Saved&&!CompatibilityReader.IsError());
  TestEqual(TEXT("legacy memory modulo uint32 value preserved"),int32(Value.ServerFrame),INDEX_NONE);
 }
 TArray<uint8> WidePayload{1,0,1,1,1,1,0x20};FMemoryReader WideReader(WidePayload);
 FNetworkWheeledSpeedInputState WideValue;bool WideSuccess=false;
 TestTrue(TEXT("memory fifth payload keeps UE modulo semantics"),WideValue.NetSerialize(WideReader,nullptr,WideSuccess)&&WideSuccess&&!WideReader.IsError());
 TestEqual(TEXT("wide memory payload projects modulo uint32"),int32(WideValue.ServerFrame),INDEX_NONE);
 // Nonminimal-but-in-range remains accepted; no new canonical-encoding rule.
 TArray<uint8> Nonminimal{1,0,1,0};FMemoryReader Reader(Nonminimal);
 FNetworkWheeledSpeedInputState Destination;bool Success=false;
 TestTrue(TEXT("nonminimal uint32 encoding is preserved"),Destination.NetSerialize(Reader,nullptr,Success)&&Success&&!Reader.IsError());
 TestEqual(TEXT("nonminimal zero transports INDEX_NONE"),int32(Destination.ServerFrame),INDEX_NONE);
 return !HasAnyErrors();
}

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
	ON_SCOPE_EXIT { Live->RetireInputProcessingOnWorker(); Replay->RetireInputProcessingOnWorker(); };
	// Publish only the timeline origin before admitting the first remote packet.
	TestTrue(TEXT("replay timeline projection prepared"), Replay->ValidateCanonicalFrameCommit(99));
	TestTrue(TEXT("replay timeline projection committed"), Replay->CommitCanonicalFrame(99));
	TNetRewindHistory<FNetworkWheeledSpeedInputState, true> History(32, true);
	TArray<FNetworkWheeledSpeedInputState> LastPackets;

	// The actual UE history container and production BuildData/ApplyData methods,
	// not synthetic presentation packets. This does not run a Chaos solver rewind.
	for (int32 Index = 0; Index < 5100; ++Index)
	{
		const auto Expected = Index % 2 ? B : A;
		Live->PublishHeldCameraInput(Expected);
		Live->BaseGameState.NumFrame = 120 + Index;
		FWheeledInputState Requested;
		Requested.Throttle = 201; Requested.Brake = 37; Requested.Steer = -71; Requested.Camera = Expected;
		TestTrue(TEXT("immutable received packet admitted"), Live->SubmitLegacyWheeledInput(120 + Index, 120 + Index, Requested)
			== Speed::Input::ELegacyRemoteAdmission::Accepted);
		Live->UpdateInputs();
		const auto PhysicalBeforeCapture = Live->WheeledPhysicalInput;
		TestTrue(TEXT("same-frame projection prepared"), Live->ValidateCanonicalFrameCommit(119 + Index));
		TestTrue(TEXT("same-frame projection committed"), Live->CommitCanonicalFrame(119 + Index));
		FNetworkWheeledSpeedInputState Packet;
		Packet.LocalFrame = 9000 + Index;
		Packet.BuildData(Live);
		TestTrue(TEXT("one capture supplies archived camera"), Packet.WheeledInput.Camera.Equals(Expected));
		TestTrue(TEXT("network export keeps Requested and never changes Applied"),
			Packet.WheeledInput.Throttle == 201 && Packet.WheeledInput.Brake == 37 && Packet.WheeledInput.Steer == -71
			&& Live->WheeledPhysicalInput.Throttle == PhysicalBeforeCapture.Throttle
			&& Live->WheeledPhysicalInput.Brake == PhysicalBeforeCapture.Brake
			&& Live->WheeledPhysicalInput.Steer == PhysicalBeforeCapture.Steer);
		TestEqual(TEXT("component mapping independent of Chaos offset"),
			Packet.ResolveActivationFrame(100, true), 121 + Index);
		TestTrue(TEXT("record real history"), History.RecordData(Packet.LocalFrame, &Packet));
		Live->PublishHeldCameraInput(Index % 2 ? A : B); // GT changes after BuildData.
		auto Duplicate = Packet;
		Duplicate.BuildData(Live);
		TestTrue(TEXT("duplicate BuildData same frame does not recapture GT"), Duplicate.WheeledInput.Camera.Equals(Expected));
		// Opposite GT mailbox cannot rewrite the completed physical snapshot.
		TestTrue(TEXT("Step applies archived value despite opposite mailbox"), Live->WheeledPhysicalInput.Camera.Equals(Expected));
		const uint64 ApplyCount = Live->CameraInputApplyCount;
		Live->ConsumeQueuedCameraInputsForFrame(121 + Index);
		TestEqual(TEXT("consumed latch applies once"), Live->CameraInputApplyCount, ApplyCount);
		TestTrue(TEXT("no growing camera input journal"), Live->PendingCameraInputCommands.IsEmpty());
		if (Index >= 5084) LastPackets.Add(Packet);
	}
	TestEqual(TEXT("network BuildData never polls a live mailbox"), Live->CameraMailboxReadCount, uint64(0));
	for (const auto& Recorded : LastPackets)
	{
		FNetworkWheeledSpeedInputState Extracted;
		TestTrue(TEXT("extract actual rewind-history entry"), History.ExtractData(Recorded.LocalFrame, true, &Extracted, true));
		Replay->PublishHeldCameraInput(Recorded.WheeledInput.Camera.IsBack() ? B : A);
		Extracted.ApplyData(Replay);
		Replay->BaseGameState.NumFrame = Extracted.ResolveActivationFrame(100, false);
		Replay->UpdateInputs();
		const uint64 N = uint64(Replay->NumFrame()) - 1;
		TestTrue(TEXT("received frame prepares"), Replay->ValidateCanonicalFrameCommit(N));
		TestTrue(TEXT("received frame commits"), Replay->CommitCanonicalFrame(N));
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

	// Exercise UE's missing-frame routes, not an independently authored lerp.
	// Held axes are selected as one complete command with its source timeline.
	TNetRewindHistory<FNetworkWheeledSpeedInputState, true> SparseHistory(32, true);
	auto Lower = LastPackets[0];
	Lower.LocalFrame = 100; Lower.ServerFrame = 200;
	Lower.ClientFrame = 300; Lower.ClientFramesSinceCanMove = 200;
	Lower.WheeledInput.Camera = A;
	auto Upper = Lower;
	Upper.LocalFrame = 104; Upper.ServerFrame = 204;
	Upper.ClientFrame = 304; Upper.ClientFramesSinceCanMove = 204;
	Upper.WheeledInput.Camera = B;
	TestTrue(TEXT("record sparse lower"), SparseHistory.RecordData(100, &Lower));
	TestTrue(TEXT("record sparse upper"), SparseHistory.RecordData(104, &Upper));
	for (int32 Frame = 101; Frame <= 108; ++Frame)
	{
		FNetworkWheeledSpeedInputState Selected;
		TestTrue(TEXT("UE interpolation/extrapolation extracts held packet"),
			SparseHistory.ExtractData(Frame, true, &Selected));
		const auto& Expected = Frame < 102 ? Lower : Upper;
		TestTrue(TEXT("held axes and Back never become a blended command"), Selected.WheeledInput.Camera.Equals(Expected.WheeledInput.Camera));
		TestTrue(TEXT("selected command keeps component activation provenance"),
			Selected.ClientFrame == Expected.ClientFrame && Selected.ClientFramesSinceCanMove == Expected.ClientFramesSinceCanMove);
		TestEqual(TEXT("UE extraction keeps requested history address"), int32(Selected.LocalFrame), Frame);
		Selected.DecayData(1.0f);
		TestTrue(TEXT("input decay preserves held camera"), Selected.WheeledInput.Camera.Equals(Expected.WheeledInput.Camera));
		const auto BeforeStale = Replay->WheeledPhysicalInput.Camera;
		Selected.ApplyData(Replay);
		TestTrue(TEXT("old/interpolated packet cannot rewrite committed forward-only input"),
			Replay->WheeledPhysicalInput.Camera.Equals(BeforeStale));
	}
	auto Merged = Upper;
	SparseHistory.MergeData(100, &Merged);
	TestTrue(TEXT("UE merge retains newest whole held command"), Merged.WheeledInput.Camera.Equals(B) && Merged.ClientFrame == Upper.ClientFrame);
	auto Different = Upper;
	Different.WheeledInput.Camera = A;
	Different.WheeledInput.bCanMove = false;
	TestFalse(TEXT("camera mismatch requests rewind even during countdown"), Upper.CompareData(Different));
	TestTrue(TEXT("identical camera packet compares equal"), Upper.CompareData(Upper));
	TestEqual(TEXT("all history routes still avoid live mailbox reads"), Replay->CameraMailboxReadCount, uint64(0));

	// Camera queue boundaries are independent of the mechanical command queue.
	Replay->EnableGenericCameraInput(true);
	for (int32 Frame = 0; Frame < 300; ++Frame) Replay->QueueCameraInputForFrame(Frame, Frame % 2 ? B : A);
	TestEqual(TEXT("camera queue has bounded retention"), Replay->PendingCameraInputCommands.Num(), 256);
	TestEqual(TEXT("overflow retains newest activation addresses"), Replay->PendingCameraInputCommands[0].ActivationFrame, 44);
	Replay->QueueCameraInputForFrame(299, A);
	Replay->QueueCameraInputForFrame(298, B);
	TestEqual(TEXT("duplicate address replaces without growing queue"), Replay->PendingCameraInputCommands.Num(), 256);
	Replay->ConsumeQueuedCameraInputsForFrame(298);
	TestTrue(TEXT("future command is not applied early"), Replay->WheeledPhysicalInput.Camera.Equals(B));
	TestEqual(TEXT("future command stays pending"), Replay->PendingCameraInputCommands.Num(), 1);
	Replay->ConsumeQueuedCameraInputsForFrame(299);
	TestTrue(TEXT("replacement at same frame is applied"), Replay->WheeledPhysicalInput.Camera.Equals(A));
	Replay->QueueCameraInputForFrame(400, B);
	Replay->EnableGenericCameraInput(false);
	TestTrue(TEXT("disable removes pending old-owner captures"), Replay->PendingCameraInputCommands.IsEmpty());
	const uint64 ReadsBeforeDisabled = Replay->CameraMailboxReadCount;
	Replay->PublishHeldCameraInput(B);
	TestFalse(TEXT("disabled capture is absent"), Replay->CaptureNetworkCameraInput(400, 400).IsPresent());
	TestEqual(TEXT("disabled capture never reads mailbox"), Replay->CameraMailboxReadCount, ReadsBeforeDisabled);
	Replay->EnableGenericCameraInput(true);
	const auto Reactivated = Replay->CaptureNetworkCameraInput(400, 400);
	TestFalse(TEXT("reactivation starts with a cleared mailbox"), Reactivated.IsPresent());
	Replay->ConsumeQueuedCameraInputsForFrame(400);
	TestTrue(TEXT("neutral reset reaches physical input through normal queue"), Replay->WheeledPhysicalInput.Camera.Equals(Reactivated));

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
		return L.ServerFrame == R.ServerFrame && L.ClientFrame == R.ClientFrame &&
			L.ClientFramesSinceCanMove == R.ClientFramesSinceCanMove && L.bIsAutonomousProxy == R.bIsAutonomousProxy &&
			L.WheeledInput.bCanMove == R.WheeledInput.bCanMove && L.WheeledInput.Throttle == R.WheeledInput.Throttle &&
			L.WheeledInput.Brake == R.WheeledInput.Brake && L.WheeledInput.Steer == R.WheeledInput.Steer &&
			L.WheeledInput.Camera.Equals(R.WheeledInput.Camera);
	};
	FNetworkWheeledSpeedInputState Decoded;
	FMemoryReader Reader(Encoded, true);
	TestTrue(TEXT("complete input roundtrip"), Decoded.NetSerialize(Reader, nullptr, bSuccess) && bSuccess && EqualPacket(Decoded, Packet));
	// SerializeFrames transports ServerFrame. It only seeds the endpoint-local
	// address here; ReceiveNewData applies that endpoint's frame offset later.
	TestTrue(TEXT("raw decode seeds endpoint-local address from transported server frame"),
		Decoded.LocalFrame == Decoded.ServerFrame && Decoded.ServerFrame == Packet.ServerFrame);
	for (int32 Length = 0; Length < Encoded.Num(); ++Length)
	{
		TArray<uint8> Truncated;
		Truncated.Append(Encoded.GetData(), Length);
		FMemoryReader ShortReader(Truncated, true);
		auto Destination = Packet;
		Destination.LocalFrame = 7;
		const auto Before = Destination;
		TestFalse(TEXT("truncated whole input rejected"), Destination.NetSerialize(ShortReader, nullptr, bSuccess));
		TestTrue(TEXT("truncation changes no destination fields"),
			Destination.LocalFrame == Before.LocalFrame && EqualPacket(Destination, Before));
	}
	for (int32 TailOffset = 0; TailOffset < 4; ++TailOffset)
	{
		auto Corrupt = Encoded;
		Corrupt[Corrupt.Num() - 4 + TailOffset] = TailOffset < 2 ? 255 : 128;
		FMemoryReader BadReader(Corrupt, true);
		auto Destination = Packet;
		TestFalse(TEXT("unknown version flags or forbidden axes rejected"), Destination.NetSerialize(BadReader, nullptr, bSuccess));
		TestTrue(TEXT("invalid tail leaves complete destination unchanged"),
			Destination.LocalFrame == Packet.LocalFrame && EqualPacket(Destination, Packet));
	}
	FNetworkWheeledSpeedInputState Neutral = Packet;
	Neutral.WheeledInput.Camera = FSpeedCarCameraPhysicalInput();
	Neutral.ApplyData(Replay);
	Replay->ConsumeQueuedCameraInputsForFrame(Neutral.ResolveActivationFrame(100, false));
	TestTrue(TEXT("absent extension applies canonical neutral"), Replay->WheeledPhysicalInput.Camera.Equals(Neutral.WheeledInput.Camera));
	Live->SetHeldCameraYaw(0);
	Live->SetHeldCameraPitch(0);
	Live->SetHeldCameraBack(false);
	TestFalse(TEXT("skipped release frame cannot publish"), Live->ValidateCanonicalFrameCommit(5999));
	Live->BaseGameState.NumFrame = 5220;
	Packet.LocalFrame = 20000;
	Packet.BuildData(Live);
	TestTrue(TEXT("GT release cannot rewrite the previous completed snapshot"),
		Packet.WheeledInput.Camera.Equals(LastPackets.Last().WheeledInput.Camera));
	FWheeledInputState Released = Live->WheeledPhysicalInput;
	Released.Camera = FSpeedCarCameraPhysicalInput();
	TestTrue(TEXT("immutable release packet admitted"), Live->SubmitLegacyWheeledInput(5220, 5220, Released)
		== Speed::Input::ELegacyRemoteAdmission::Accepted);
	Live->UpdateInputs();
	TestTrue(TEXT("release projection prepared"), Live->ValidateCanonicalFrameCommit(5219));
	TestTrue(TEXT("release projection committed"), Live->CommitCanonicalFrame(5219));
	Packet.BuildData(Live);
	TestEqual(TEXT("release export never polls the live mailbox"), Live->CameraMailboxReadCount, uint64(0));
	TestTrue(TEXT("Completed endpoints release to exact zero"), !Packet.WheeledInput.Camera.IsBack() &&
		Packet.WheeledInput.Camera.Yaw == 0 && Packet.WheeledInput.Camera.Pitch == 0);
	return true;
}
#endif
