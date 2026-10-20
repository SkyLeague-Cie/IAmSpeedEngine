#include "IAmSpeed/World/Simulation/FrameWitnessBlob.h"
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "Misc/AutomationTest.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedWitnessBlobRoundTrip,"IAmSpeed.Physics.FrameWitness.BlobRoundTrip",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedWitnessBlobRoundTrip::RunTest(const FString&)
{
 for(int32 Size:{0,1,65537,434678})
 {
  TArray<uint8> Raw,Packed,Again;Raw.SetNumUninitialized(Size);
  for(int32 I=0;I<Size;++I)Raw[I]=uint8((uint64(I)*97+(I>>3))&255);
  TestTrue(TEXT("encode original byte array"),Speed::FrameWitnessBlob::Encode(Raw,Packed));
  TestTrue(TEXT("decode exact native zlib transport"),Speed::FrameWitnessBlob::Decode(Packed,Again));
  TestTrue(TEXT("every original byte retained"),Raw==Again);
 }
 return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedWitnessBlobReject,"IAmSpeed.Physics.FrameWitness.BlobRejectMalformed",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedWitnessBlobReject::RunTest(const FString&)
{
 using namespace Speed::FrameWitnessBlob;
 TArray<uint8> Raw,Packed,Destination;Raw.Init(0x35,4096);Destination.Add(0xA7);
 TestTrue(TEXT("setup"),Encode(Raw,Packed));
 // Invalid compressed streams are covered by the bounded Python decoder.
 // Here validate preflight/checksum failures without asking Engine zlib to
 // emit an expected error into Unreal's global automation error stream.
 for(int32 Offset:{0,16,24,43})
 {
  auto Bad=Packed;Bad[Offset]^=1;const auto Before=Destination;
  TestFalse(TEXT("corrupt transport rejected"),Decode(Bad,Destination));
  TestTrue(TEXT("failure preserves previous output"),Destination==Before);
 }
 auto Bad=Packed;Put64(Bad.GetData()+8,uint64(MaxRawBytes)+1);
 TestFalse(TEXT("oversize declaration rejected before allocation"),Decode(Bad,Destination));
 Bad=Packed;Bad.Add(0);TestFalse(TEXT("extra bytes rejected"),Decode(Bad,Destination));
 Bad=Packed;Bad.SetNum(HeaderBytes-1);TestFalse(TEXT("truncated header rejected"),Decode(Bad,Destination));
 return true;
}
#endif
