#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "FramePointerWitness.h"
#include "NativeVehicleConfigWitness1534.h"
#include "Misc/AutomationTest.h"
namespace
{
template<class T> void DeferredAdd1456(TArray<uint8>& B,const T& V)
{B.Append(reinterpret_cast<const uint8*>(&V),sizeof(V));}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedDeferredCanonical1456,
 "IAmSpeed.Physics.FrameWitness.DeferredCanonical1456",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedDeferredCanonical1456::RunTest(const FString&)
{
 using namespace Speed;
 FFramePointerCanonicalSnapshot1456 Captured;Captured.Reserve1456(4096,16);
 TArray<uint8> Golden,Direct;TArray<int32> RawBoundaries1534,ExpectedBoundaries1534;
 const uint32 Prefix=0x1234abcdu;const uint16 Suffix=0x59f3u;
 DeferredAdd1456(Golden,Prefix);DeferredAdd1456(Golden,uint32(7));
 DeferredAdd1456(Golden,uint64(42));DeferredAdd1456(Golden,uint64(3));
 DeferredAdd1456(Golden,uint64(8));DeferredAdd1456(Golden,uint64(4));
 DeferredAdd1456(Golden,uint32(2));DeferredAdd1456(Golden,uint64(7));DeferredAdd1456(Golden,uint64(19));DeferredAdd1456(Golden,Suffix);
 {
  uint64 NativeStorage=0;const UPTRINT Address=reinterpret_cast<UPTRINT>(&NativeStorage);
  FFramePointerRegistry Registry;FFramePointerKey K;K.Domain=7;K.Owner=42;K.Generation=3;K.Role=8;K.Element=4;K.Membership={7,19};
  if(!TestTrue(TEXT("semantic registry is sealed"),Registry.Register(Address,K)&&Registry.Seal()))return false;
  TArray<uint8> Raw;FFramePointerCaptureScope Scope(Raw,Registry);
  DeferredAdd1456(Raw,Prefix);FFramePointerCaptureScope::Observe(Raw,Address);DeferredAdd1456(Raw,Address);DeferredAdd1456(Raw,Suffix);
  if(!TestTrue(TEXT("legacy projector matches independently authored exact bytes"),Scope.MakeCanonical(Direct)&&Direct==Golden)
   ||!TestTrue(TEXT("complete values captured before scope/registry lifetime ends"),Scope.CaptureCanonicalSnapshot1456(Captured)))return false;
  RawBoundaries1534={0,int32(sizeof(Prefix)),int32(sizeof(Prefix)+sizeof(Address)),Raw.Num()};
  ExpectedBoundaries1534={0,int32(sizeof(Prefix)),int32(Golden.Num()-sizeof(Suffix)),Golden.Num()};
  TArray<uint8> Projected1534;TArray<int32> Boundaries1534;
  TestTrue(TEXT("field boundaries use the identical validated projector"),Scope.MakeCanonicalWithBoundaries1534(Projected1534,RawBoundaries1534,Boundaries1534)&&Projected1534==Golden&&Boundaries1534==ExpectedBoundaries1534);
  for(const TArray<int32> BadBoundaries1534:{TArray<int32>{5},TArray<int32>{4,0},TArray<int32>{Raw.Num()+1}})
  {
   TArray<uint8> Sentinel1534={0x91};TArray<int32> SentinelBounds1534={73};
   TestFalse(TEXT("interior/nonmonotone/out-of-range boundary refuses exact localization"),Scope.MakeCanonicalWithBoundaries1534(Sentinel1534,BadBoundaries1534,SentinelBounds1534));
   TestTrue(TEXT("boundary refusal is transactional for bytes and offsets"),Sentinel1534==TArray<uint8>{0x91}&&SentinelBounds1534==TArray<int32>{73});
  }
  // Neither later raw drift nor further events may change the owned snapshot.
  Raw[sizeof(Prefix)]^=1;FFramePointerCaptureScope::Observe(Raw,Address);DeferredAdd1456(Raw,Address);
  TArray<uint8> Refused={0x5a};const auto Before=Refused;
  TestFalse(TEXT("legacy live byte drift still refuses"),Scope.MakeCanonical(Refused));TestTrue(TEXT("refusal leaves output unchanged"),Refused==Before);
 }
 TArray<uint8> Deferred;
 if(!TestTrue(TEXT("owned projection survives destroyed scope, registry, raw and native object"),Captured.MakeCanonical(Deferred)&&Deferred==Golden&&Deferred==Direct))return false;
 TestEqual(TEXT("all semantic observations captured"),Captured.NumObservations1456(),1);
 TArray<uint8> Deferred1534;TArray<int32> DeferredBounds1534;
 TestTrue(TEXT("deferred field offsets survive complete native/scope lifetime drain"),Captured.MakeCanonicalWithBoundaries1534(Deferred1534,RawBoundaries1534,DeferredBounds1534)&&Deferred1534==Golden&&DeferredBounds1534==ExpectedBoundaries1534);
 {
  FFramePointerRegistry Registry;Registry.Seal();TArray<uint8> Raw;FFramePointerCaptureScope Scope(Raw,Registry);
  const UPTRINT Unknown=UPTRINT(0x12345678u);FFramePointerCaptureScope::Observe(Raw,Unknown);DeferredAdd1456(Raw,Unknown);
  FFramePointerCanonicalSnapshot1456 Bad;TArray<uint8> Sentinel={0x3c};const auto Before=Sentinel;
  TestFalse(TEXT("unknown identity cannot be captured as complete"),Scope.CaptureCanonicalSnapshot1456(Bad));
  TestFalse(TEXT("uncaptured identity remains invalid"),Bad.MakeCanonical(Sentinel));TestTrue(TEXT("unknown output stays untouched"),Sentinel==Before);
 }
 {
  FFramePointerRegistry Registry;Registry.Seal();TArray<uint8> Raw;FFramePointerCaptureScope Scope(Raw,Registry,1);
  const UPTRINT Null=0;for(int I=0;I<2;++I){FFramePointerCaptureScope::Observe(Raw,Null);DeferredAdd1456(Raw,Null);}
  FFramePointerCanonicalSnapshot1456 Bad;TestFalse(TEXT("overflow cannot be captured as complete"),Scope.CaptureCanonicalSnapshot1456(Bad));
 }
 {
  uint64 NativeStorage=0;const UPTRINT Address=reinterpret_cast<UPTRINT>(&NativeStorage);
  FFramePointerRegistry Registry;FFramePointerKey K;K.Domain=9;Registry.Register(Address,K);Registry.Seal();
  TArray<uint8> Raw;FFramePointerCaptureScope Scope(Raw,Registry);FFramePointerCaptureScope::Observe(Raw,Address);DeferredAdd1456(Raw,Address);Raw[0]^=1;
  FFramePointerCanonicalSnapshot1456 Bad;TArray<uint8> Sentinel={0x6e};const auto Before=Sentinel;
  TestTrue(TEXT("capture success is values-only, never a canonical certificate"),Scope.CaptureCanonicalSnapshot1456(Bad));
  TestFalse(TEXT("deferred projector keeps the original exact byte-drift gate"),Bad.MakeCanonical(Sentinel));TestTrue(TEXT("deferred refusal leaves output untouched"),Sentinel==Before);
 }
 {
  Chaos::FSimpleWheelConfig Wheel;Chaos::FSimpleSuspensionConfig Suspension;TArray<uint8> WheelBefore,WheelAfter,SuspensionBefore,SuspensionAfter;
  TestTrue(TEXT("owned empty graph configuration is boundedly observable"),Testing1534::AppendWheelConfig(WheelBefore,Wheel));
  Wheel.WheelRadius+=1;Testing1534::AppendWheelConfig(WheelAfter,Wheel);
  TestTrue(TEXT("semantic setup pointer cannot hide changed actual wheel configuration"),WheelBefore!=WheelAfter);
  Testing1534::AppendSuspensionConfig(SuspensionBefore,Suspension);Suspension.SetSuspensionMaxDrop(25);Testing1534::AppendSuspensionConfig(SuspensionAfter,Suspension);
  TestTrue(TEXT("actual suspension travel and dependent max length are observed"),SuspensionBefore!=SuspensionAfter);
  Wheel.LateralSlipGraph.Add(Chaos::FVec2(0,1));TArray<uint8> ActivePrivateGraph;
  TestFalse(TEXT("nonempty opaque graph never receives a complete configuration witness"),Testing1534::AppendWheelConfig(ActivePrivateGraph,Wheel));
 }
 return !HasAnyErrors();
}
#endif
