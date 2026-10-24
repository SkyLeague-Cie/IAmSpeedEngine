#include "IAmSpeed/Base/EpisodeShapeFieldsForTesting.h"
#include "IAmSpeed/World/Simulation/NativeSelectedStateBytesForTesting.h"
#include "IAmSpeed/World/Simulation/FrameFloatingPointContext.h"
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "Misc/AutomationTest.h"
#include <stdexcept>

namespace
{
// Independent oracle: memcpy each field, never call the production Add/helper.
template<class T> void ReferenceScalar(TArray<uint8>& Out,const T& Value)
{
 static_assert(std::is_arithmetic_v<T>);
 const int32 Offset=Out.AddUninitialized(sizeof(T));
 FMemory::Memcpy(Out.GetData()+Offset,&Value,sizeof(T));
}
template<class V> void ReferenceXYZ(TArray<uint8>& Out,const V& Value)
{ReferenceScalar(Out,Value.X);ReferenceScalar(Out,Value.Y);ReferenceScalar(Out,Value.Z);}
void ReferenceQuat(TArray<uint8>& Out,const FQuat& Value)
{ReferenceXYZ(Out,Value);ReferenceScalar(Out,Value.W);}
void ReferenceMatrix(TArray<uint8>& Out,const FMatrix& Value)
{for(int32 Row=0;Row<4;++Row)for(int32 Column=0;Column<4;++Column)ReferenceScalar(Out,Value.M[Row][Column]);}
void ReferenceKinematic(TArray<uint8>& Out,const Speed::FKinematicState& Value)
{
 ReferenceXYZ(Out,Value.Location);ReferenceXYZ(Out,Value.Velocity);ReferenceXYZ(Out,Value.Acceleration);
 ReferenceQuat(Out,Value.Rotation);ReferenceXYZ(Out,Value.AngularVelocity);ReferenceXYZ(Out,Value.AngularAcceleration);
}
template<class T> void SetScalarBits(T& Value,uint32 Index)
{
 using S=std::remove_cvref_t<T>;
 static_assert(std::is_same_v<S,float> || std::is_same_v<S,double>);
 // Native scalar bits: both zero signs, subnormals, extrema, infinities and
 // distinct quiet NaN payloads. No arithmetic or vector normalization.
 if constexpr(std::is_same_v<S,float>)
 {
  const uint32 Bits[]={0u,0x80000000u,1u,0x80000001u,0x007fffffu,0x00800000u,
   0x3f800001u,0xc0200001u,0x7f7fffffu,0xff7fffffu,0x7f800000u,0xff800000u,
   0x7fc00001u,0x7fc00002u,0xffc00123u,0x3eaaaaabu};
  FMemory::Memcpy(&Value,&Bits[Index%16],sizeof(Value));
 }
 else
 {
  const uint64 Bits[]={0ull,0x8000000000000000ull,1ull,0x8000000000000001ull,
   0x000fffffffffffffull,0x0010000000000000ull,0x3ff0000000000001ull,0xc004000000000001ull,
   0x7fefffffffffffffull,0xffefffffffffffffull,0x7ff0000000000000ull,0xfff0000000000000ull,
   0x7ff8000000000001ull,0x7ff8000000000002ull,0xfff8000000000123ull,0x3fd5555555555555ull};
  FMemory::Memcpy(&Value,&Bits[Index%16],sizeof(Value));
 }
}
template<class V> void FillXYZ(V& Value,uint32 Index)
{SetScalarBits(Value.X,Index);SetScalarBits(Value.Y,Index+1);SetScalarBits(Value.Z,Index+2);}
void FillKinematic(Speed::FKinematicState& Value,uint32 Index)
{
 FillXYZ(Value.Location,Index);FillXYZ(Value.Velocity,Index+3);FillXYZ(Value.Acceleration,Index+6);
 FillXYZ(Value.Rotation,Index+9);SetScalarBits(Value.Rotation.W,Index+12);
 FillXYZ(Value.AngularVelocity,Index+13);FillXYZ(Value.AngularAcceleration,Index+16);
}
// Nineteen distinct native representations detect any field reordering even
// between fields sixteen positions apart in the existing rotating pattern set.
template<class T> void SetDistinctKinematicBits(T& Value,uint32 Index)
{
 using S=std::remove_cvref_t<T>;
 static_assert(std::is_same_v<S,float> || std::is_same_v<S,double>);
 if constexpr(std::is_same_v<S,float>)
 {
  const uint32 Bits[]={0u,0x80000000u,1u,0x80000001u,0x007fffffu,0x00800000u,
   0x3f800001u,0xc0200001u,0x7f7fffffu,0xff7fffffu,0x7f800000u,0xff800000u,
   0x7fc00001u,0x7fc00002u,0xffc00123u,0x3eaaaaabu,0x3f000001u,0xbf000001u,0x00400001u};
  FMemory::Memcpy(&Value,&Bits[Index%19],sizeof(Value));
 }
 else
 {
  const uint64 Bits[]={0ull,0x8000000000000000ull,1ull,0x8000000000000001ull,
   0x000fffffffffffffull,0x0010000000000000ull,0x3ff0000000000001ull,0xc004000000000001ull,
   0x7fefffffffffffffull,0xffefffffffffffffull,0x7ff0000000000000ull,0xfff0000000000000ull,
   0x7ff8000000000001ull,0x7ff8000000000002ull,0xfff8000000000123ull,0x3fd5555555555555ull,
   0x3fe0000000000001ull,0xbfe0000000000001ull,0x0008000000000001ull};
  FMemory::Memcpy(&Value,&Bits[Index%19],sizeof(Value));
 }
}
void FillDistinctKinematic(Speed::FKinematicState& Value,uint32 Case)
{
 SetDistinctKinematicBits(Value.Location.X,Case);SetDistinctKinematicBits(Value.Location.Y,Case+1);SetDistinctKinematicBits(Value.Location.Z,Case+2);
 SetDistinctKinematicBits(Value.Velocity.X,Case+3);SetDistinctKinematicBits(Value.Velocity.Y,Case+4);SetDistinctKinematicBits(Value.Velocity.Z,Case+5);
 SetDistinctKinematicBits(Value.Acceleration.X,Case+6);SetDistinctKinematicBits(Value.Acceleration.Y,Case+7);SetDistinctKinematicBits(Value.Acceleration.Z,Case+8);
 SetDistinctKinematicBits(Value.Rotation.X,Case+9);SetDistinctKinematicBits(Value.Rotation.Y,Case+10);SetDistinctKinematicBits(Value.Rotation.Z,Case+11);SetDistinctKinematicBits(Value.Rotation.W,Case+12);
 SetDistinctKinematicBits(Value.AngularVelocity.X,Case+13);SetDistinctKinematicBits(Value.AngularVelocity.Y,Case+14);SetDistinctKinematicBits(Value.AngularVelocity.Z,Case+15);
 SetDistinctKinematicBits(Value.AngularAcceleration.X,Case+16);SetDistinctKinematicBits(Value.AngularAcceleration.Y,Case+17);SetDistinctKinematicBits(Value.AngularAcceleration.Z,Case+18);
}
// Independent selected-state oracle: no production Add or bounded writer.
void ReferenceContactBase(TArray<uint8>& Out,const FBasePhysicsState& X)
{
 ReferenceKinematic(Out,X.Kinematic);ReferenceScalar(Out,X.bIsFrozen);
 ReferenceScalar(Out,X.nbFramesbeforeCanMove);ReferenceScalar(Out,X.bStartCountdown);
}
void ReferenceContactWheeled(TArray<uint8>& Out,const FWheeledPhysicsState& X)
{
 ReferenceScalar(Out,X.bStartCountdown);ReferenceScalar(Out,X.nbFramesbeforeCanMove);ReferenceScalar(Out,X.FramesSinceLastImpact);
 ReferenceXYZ(Out,X.AllowedSideVelocity);ReferenceXYZ(Out,X.AllowedAngularVelocity);
 ReferenceScalar(Out,uint64(4));for(const float Value:X.SuspensionLastDisplacement)ReferenceScalar(Out,Value);
 ReferenceScalar(Out,X.PhysicalThrottleInput);ReferenceScalar(Out,X.PhysicalBrakeInput);
 ReferenceScalar(Out,X.PhysicalSteerInput);ReferenceScalar(Out,X.NbFramesSinceGroundContact);
}
// Retain the previous fixture spelling independently of the new writer.
void LegacyContactBase(TArray<uint8>& Out,const FBasePhysicsState& X)
{
 using W=Speed::FEpisodeStateBytesForTesting;
 W::Add(Out,X.Kinematic);W::Add(Out,X.bIsFrozen);W::Add(Out,X.nbFramesbeforeCanMove);W::Add(Out,X.bStartCountdown);
}
void LegacyContactWheeled(TArray<uint8>& Out,const FWheeledPhysicsState& X)
{
 using W=Speed::FEpisodeStateBytesForTesting;
 W::Add(Out,X.bStartCountdown);W::Add(Out,X.nbFramesbeforeCanMove);W::Add(Out,X.FramesSinceLastImpact);
 W::Add(Out,X.AllowedSideVelocity);W::Add(Out,X.AllowedAngularVelocity);W::Add(Out,X.SuspensionLastDisplacement);
 W::Add(Out,X.PhysicalThrottleInput);W::Add(Out,X.PhysicalBrakeInput);W::Add(Out,X.PhysicalSteerInput);W::Add(Out,X.NbFramesSinceGroundContact);
}
void FillContactStates(FBasePhysicsState& Base,FWheeledPhysicsState& Wheel,uint32 Slot)
{
 FillDistinctKinematic(Base.Kinematic,Slot);Base.bIsFrozen=(Slot&1)!=0;
 Base.nbFramesbeforeCanMove=uint16(Slot);Base.bStartCountdown=(Slot&2)!=0;
 Wheel.bStartCountdown=(Slot&4)!=0;Wheel.nbFramesbeforeCanMove=uint16(Slot);Wheel.FramesSinceLastImpact=uint8(Slot);
 FillXYZ(Wheel.AllowedSideVelocity,Slot+3);FillXYZ(Wheel.AllowedAngularVelocity,Slot+6);
 for(uint32 Index=0;Index<4;++Index)SetScalarBits(Wheel.SuspensionLastDisplacement[Index],Slot+Index+9);
 Wheel.PhysicalThrottleInput=uint8(Slot+1);Wheel.PhysicalBrakeInput=uint8(Slot+2);
 Wheel.PhysicalSteerInput=int8(int32(Slot%255)-127);Wheel.NbFramesSinceGroundContact=uint8(Slot+3);
}
void ReferencePointer(TArray<uint8>& Out,int* Pointer)
{
 const UPTRINT Address=reinterpret_cast<UPTRINT>(Pointer);
 Speed::FFramePointerCaptureScope::Observe(Out,Address);ReferenceScalar(Out,Address);
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FEpisodeNumericCodecTests,
 "IAmSpeed.Simulation.FrameWitness.NumericCodecByteConformance",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FEpisodeNumericCodecTests::RunTest(const FString&)
{
 using W=Speed::FEpisodeStateBytesForTesting;
 const auto Numeric=Speed::CaptureFrameFloatingPointContext();
 Speed::FScopedFrameFloatingPointContext Guard(Numeric);
 if(!TestTrue(TEXT("captured numeric context applies"),Guard.IsApplied()))return false;
 const uint32 Prefix=0x9273a51bu;const uint16 Suffix=0xd34fu;
 const auto Check=[&](const TCHAR* Label,const auto& Value,const auto& Reference)
 {
  TArray<uint8> Actual,Expected;
  W::Add(Actual,Prefix);W::Add(Actual,Value);W::Add(Actual,Suffix);
  ReferenceScalar(Expected,Prefix);Reference(Expected,Value);ReferenceScalar(Expected,Suffix);
  TestEqual(FString::Printf(TEXT("%s complete byte count"),Label),Actual.Num(),Expected.Num());
  TestTrue(FString::Printf(TEXT("%s prefix/payload/suffix exact bytes"),Label),Actual==Expected);
 };
 for(uint32 Case=0;Case<16;++Case)
 {
  FVector Vector;FillXYZ(Vector,Case);
  Check(TEXT("FVector"),Vector,[](auto& O,const auto& X){ReferenceXYZ(O,X);});
  FVector_NetQuantize Net;FillXYZ(Net,Case);
  Check(TEXT("FVector_NetQuantize"),Net,[](auto& O,const auto& X){ReferenceXYZ(O,X);});
  FVector_NetQuantize10 Net10;FillXYZ(Net10,Case);
  Check(TEXT("FVector_NetQuantize10"),Net10,[](auto& O,const auto& X){ReferenceXYZ(O,X);});
  FVector_NetQuantize100 Net100;FillXYZ(Net100,Case);
  Check(TEXT("FVector_NetQuantize100"),Net100,[](auto& O,const auto& X){ReferenceXYZ(O,X);});
  FVector_NetQuantizeNormal Normal;FillXYZ(Normal,Case);
  Check(TEXT("FVector_NetQuantizeNormal"),Normal,[](auto& O,const auto& X){ReferenceXYZ(O,X);});
  FQuat Quat;FillXYZ(Quat,Case);SetScalarBits(Quat.W,Case+3);
  Check(TEXT("FQuat"),Quat,ReferenceQuat);
  FMatrix Matrix;
  for(int32 Row=0;Row<4;++Row)for(int32 Column=0;Column<4;++Column)
   SetScalarBits(Matrix.M[Row][Column],Case+Row*4+Column);
  Check(TEXT("FMatrix row-major"),Matrix,ReferenceMatrix);
  Speed::FKinematicState Kinematic;FillKinematic(Kinematic,Case);
  Check(TEXT("FKinematicState explicit fields"),Kinematic,ReferenceKinematic);
 }
 // Every field has a distinct representation; rotate to put each special value
 // in every position. ReferenceKinematic appends each scalar independently.
 for(uint32 Case=0;Case<19;++Case)
 {
  Speed::FKinematicState State;FillDistinctKinematic(State,Case);
  Check(TEXT("FKinematicState 19 distinct scalar representations"),State,ReferenceKinematic);
  TArray<uint8> Payload;W::Add(Payload,State);
  TestEqual(TEXT("kinematic payload has exactly 19 scalars and no count prefix"),
   Payload.Num(),int32(19*sizeof(State.Location.X)));
 }
 // Exercise the existing fixed-array length prefix independently, too.
 const float Fixed[3]={1.f,-0.f,3.f};TArray<uint8> FixedActual,FixedExpected;
 W::Add(FixedActual,Fixed);ReferenceScalar(FixedExpected,uint64(3));
 for(float Value:Fixed)ReferenceScalar(FixedExpected,Value);
 TestTrue(TEXT("fixed scalar array retains uint64 count prefix"),FixedActual==FixedExpected);

 // A complete history, not a reduced sample. Its slot marker distinguishes
 // every slot even when the 16 special floating-point patterns repeat.
 TArray<uint8> HistoryActual,HistoryExpected;
 W::Add(HistoryActual,int32(1024));ReferenceScalar(HistoryExpected,int32(1024));
 for(uint32 Slot=0;Slot<1024;++Slot)
 {
  Speed::FKinematicState State;FillKinematic(State,Slot);
  W::Add(HistoryActual,Slot);W::Add(HistoryActual,State);
  ReferenceScalar(HistoryExpected,Slot);ReferenceKinematic(HistoryExpected,State);
 }
 TestTrue(TEXT("all 1024 marked history slots retain exact bytes"),HistoryActual==HistoryExpected);

 // Full legacy/new/reference agreement, including mixed widths and the fixed
 // suspension array prefix. Distinct slot counters detect omission/reordering.
 for(uint32 Case=0;Case<19;++Case)
 {
  FBasePhysicsState Base;FWheeledPhysicsState Wheel;FillContactStates(Base,Wheel,Case);
  TArray<uint8> New,Legacy,Reference;
  W::Add(New,Prefix);W::Add(Legacy,Prefix);ReferenceScalar(Reference,Prefix);
  Speed::SelectedStateBytesForTesting::AppendBaseState(New,Base);Speed::SelectedStateBytesForTesting::AppendWheeledStatePrefix(New,Wheel);
  LegacyContactBase(Legacy,Base);LegacyContactWheeled(Legacy,Wheel);
  ReferenceContactBase(Reference,Base);ReferenceContactWheeled(Reference,Wheel);
  W::Add(New,Suffix);W::Add(Legacy,Suffix);ReferenceScalar(Reference,Suffix);
  TestTrue(TEXT("1221 bounded states exactly match legacy and independent scalar oracle with special bits"),New==Legacy&&New==Reference);
 }
 {
  TArray<uint8> New,Legacy,Reference;
  for(uint32 Car=0;Car<2;++Car)
  {
   W::Add(New,int32(1024));W::Add(Legacy,int32(1024));ReferenceScalar(Reference,int32(1024));
   for(int32 Slot=0;Slot<1024;++Slot)
   {W::Add(New,Slot);W::Add(Legacy,Slot);ReferenceScalar(Reference,Slot);}
   W::Add(New,int32(1024));W::Add(Legacy,int32(1024));ReferenceScalar(Reference,int32(1024));
   for(uint32 Slot=0;Slot<1024;++Slot)
   {
    FBasePhysicsState Base;FWheeledPhysicsState Wheel;FillContactStates(Base,Wheel,Slot+Car*1024);
    Speed::SelectedStateBytesForTesting::AppendBaseState(New,Base);LegacyContactBase(Legacy,Base);ReferenceContactBase(Reference,Base);
   }
   W::Add(New,int32(1024));W::Add(Legacy,int32(1024));ReferenceScalar(Reference,int32(1024));
   for(uint32 Slot=0;Slot<1024;++Slot)
   {
    FBasePhysicsState Base;FWheeledPhysicsState Wheel;FillContactStates(Base,Wheel,Slot+Car*1024);
    Speed::SelectedStateBytesForTesting::AppendWheeledStatePrefix(New,Wheel);LegacyContactWheeled(Legacy,Wheel);ReferenceContactWheeled(Reference,Wheel);
   }
  }
  TestTrue(TEXT("1221 both cars retain all1024 base/frame/wheeled history slots and count prefixes"),New==Legacy&&New==Reference);
 }

 // Exercise each mixed-width suffix boundary independently. The legacy path
 // retains four scalar Append calls; ReferenceScalar uses its separate memcpy
 // oracle. Nonzero framing detects a write at an incorrect destination offset.
 static_assert(std::is_same_v<decltype(FWheeledPhysicsState{}.CarBumpCooldownFrames),uint16>);
 const uint32 Schemas1564[]={0,0x15640001u,MAX_uint32};
 const uint8 Kinds1564[]={0,1,MAX_uint8};
 const uint64 Ids1564[]={0,1,MAX_uint64};
 const uint16 Cooldowns1564[]={0,1,MAX_uint16};
 uint32 CombinedCase1564=0;
 for(uint32 Schema1564:Schemas1564)for(uint8 Kind1564:Kinds1564)
 for(uint64 Id1564:Ids1564)for(uint16 Cooldown1564:Cooldowns1564)
 {
  TArray<uint8> New1564,Legacy1564,Reference1564;
  W::Add(New1564,Prefix);W::Add(Legacy1564,Prefix);ReferenceScalar(Reference1564,Prefix);
  Speed::SelectedStateBytesForTesting::AppendWheeledBumpSuffix1564(New1564,Schema1564,Kind1564,Id1564,Cooldown1564);
  W::Add(Legacy1564,Schema1564);W::Add(Legacy1564,Kind1564);W::Add(Legacy1564,Id1564);W::Add(Legacy1564,Cooldown1564);
  ReferenceScalar(Reference1564,Schema1564);ReferenceScalar(Reference1564,Kind1564);ReferenceScalar(Reference1564,Id1564);ReferenceScalar(Reference1564,Cooldown1564);
  TestEqual(TEXT("1564 suffix exact 15 byte payload without padding"),New1564.Num()-int32(sizeof(Prefix)),15);
  W::Add(New1564,Suffix);W::Add(Legacy1564,Suffix);ReferenceScalar(Reference1564,Suffix);
  TestTrue(TEXT("1564 suffix boundaries preserve legacy and independent framed scalar bytes"),New1564==Legacy1564&&New1564==Reference1564);

  // Keep both previous80/15-byte tests above; this independently checks their
  // complete95-byte concatenation with rotating special prefix bit patterns.
  FBasePhysicsState Base1564;FWheeledPhysicsState Wheel1564;
  FillContactStates(Base1564,Wheel1564,CombinedCase1564++);
  Wheel1564.CarBumpCooldownFrames=Cooldown1564;
  TArray<uint8> Combined1564,LegacyCombined1564,ReferenceCombined1564;
  W::Add(Combined1564,Prefix);W::Add(LegacyCombined1564,Prefix);ReferenceScalar(ReferenceCombined1564,Prefix);
  Speed::SelectedStateBytesForTesting::AppendWheeledStateWithBump1564(Combined1564,Wheel1564,Schema1564,Kind1564,Id1564);
  LegacyContactWheeled(LegacyCombined1564,Wheel1564);
  W::Add(LegacyCombined1564,Schema1564);W::Add(LegacyCombined1564,Kind1564);W::Add(LegacyCombined1564,Id1564);W::Add(LegacyCombined1564,Cooldown1564);
  ReferenceContactWheeled(ReferenceCombined1564,Wheel1564);
  ReferenceScalar(ReferenceCombined1564,Schema1564);ReferenceScalar(ReferenceCombined1564,Kind1564);ReferenceScalar(ReferenceCombined1564,Id1564);ReferenceScalar(ReferenceCombined1564,Cooldown1564);
  TestEqual(TEXT("1564 combined exact95-byte payload without padding"),Combined1564.Num()-int32(sizeof(Prefix)),95);
  W::Add(Combined1564,Suffix);W::Add(LegacyCombined1564,Suffix);ReferenceScalar(ReferenceCombined1564,Suffix);
  TestTrue(TEXT("1564 combined states preserve both independent complete framed oracles"),Combined1564==LegacyCombined1564&&Combined1564==ReferenceCombined1564);
 }


 // Range serializers retain all framing and scalar representations; the two
 // oracles below append fields independently, never call a range writer.
 for(int32 Count1564:{0,1,2,1024})
 {
  TArray<int32> Frames1564;TArray<FBasePhysicsState> Base1564;TArray<FWheeledPhysicsState> Wheel1564;
  Frames1564.SetNum(Count1564);Base1564.SetNum(Count1564);Wheel1564.SetNum(Count1564);
  for(int32 Index1564=0;Index1564<Count1564;++Index1564)
  {
   Frames1564[Index1564]=Index1564%2 ? MIN_int32+Index1564 : MAX_int32-Index1564;
   FillContactStates(Base1564[Index1564],Wheel1564[Index1564],uint32(Index1564));
   Wheel1564[Index1564].CarBumpCooldownFrames=uint16(Index1564%2 ? MAX_uint16-Index1564 : Index1564);
  }
  const auto CaptureInputs1564=[&]
  {
   TArray<uint8> Bytes;
   for(int32 Index1564=0;Index1564<Count1564;++Index1564)
   {
    ReferenceScalar(Bytes,Frames1564[Index1564]);ReferenceContactBase(Bytes,Base1564[Index1564]);
    ReferenceContactWheeled(Bytes,Wheel1564[Index1564]);ReferenceScalar(Bytes,Wheel1564[Index1564].CarBumpCooldownFrames);
   }
   return Bytes;
  };
  const auto InputBefore1564=CaptureInputs1564();
  TArray<uint8> Batched1564,Legacy1564,Reference1564;
  W::Add(Batched1564,Prefix);W::Add(Legacy1564,Prefix);ReferenceScalar(Reference1564,Prefix);
  for(TArray<uint8>* Bytes:{&Batched1564,&Legacy1564})W::Add(*Bytes,Count1564);
  ReferenceScalar(Reference1564,Count1564);
  TestTrue(TEXT("range frame IDs admitted"),Speed::SelectedStateBytesForTesting::AppendFrameHistory1564(Batched1564,Frames1564));
  for(int32 Frame1564:Frames1564){W::Add(Legacy1564,Frame1564);ReferenceScalar(Reference1564,Frame1564);}
  for(TArray<uint8>* Bytes:{&Batched1564,&Legacy1564})W::Add(*Bytes,Count1564);
  ReferenceScalar(Reference1564,Count1564);
  TestTrue(TEXT("range base states admitted"),Speed::SelectedStateBytesForTesting::AppendBaseHistory1564(Batched1564,Base1564));
  for(const auto& X:Base1564){LegacyContactBase(Legacy1564,X);ReferenceContactBase(Reference1564,X);}
  for(TArray<uint8>* Bytes:{&Batched1564,&Legacy1564})W::Add(*Bytes,Count1564);
  ReferenceScalar(Reference1564,Count1564);
  TestTrue(TEXT("range wheel prefixes admitted"),Speed::SelectedStateBytesForTesting::AppendWheelPrefixHistory1564(Batched1564,Wheel1564));
  for(const auto& X:Wheel1564){LegacyContactWheeled(Legacy1564,X);ReferenceContactWheeled(Reference1564,X);}
  W::Add(Batched1564,Suffix);W::Add(Legacy1564,Suffix);ReferenceScalar(Reference1564,Suffix);
  TestTrue(TEXT("0/1/2/1024 ranges preserve all legacy/reference framing and special bits"),Batched1564==Legacy1564&&Batched1564==Reference1564);
  TestEqual(TEXT("range framed exact byte count"),Batched1564.Num(),int32(sizeof(Prefix)+3*sizeof(int32)+Count1564*(sizeof(int32)+156+80)+sizeof(Suffix)));

  // Success plus first/middle/last refusal. Count framing deliberately remains
  // the full authored count even if only a prefix of wheel records is valid.
  TArray<int32> FailurePositions1564{INDEX_NONE};
  if(Count1564){FailurePositions1564.AddUnique(0);FailurePositions1564.AddUnique(Count1564/2);FailurePositions1564.AddUnique(Count1564-1);}
  for(int32 FailAt1564:FailurePositions1564)
  {
   TArray<uint8> BatchWheel1564,LegacyWheel1564,ReferenceWheel1564;
   W::Add(BatchWheel1564,Prefix);W::Add(LegacyWheel1564,Prefix);ReferenceScalar(ReferenceWheel1564,Prefix);
   W::Add(BatchWheel1564,Count1564);W::Add(LegacyWheel1564,Count1564);ReferenceScalar(ReferenceWheel1564,Count1564);
   SIZE_T RangeCapacity1564=BatchWheel1564.GetAllocatedSize();
   int32 Calls1564=0,FailureIndex1564=123;
   const auto Identity1564=[&](const FWheeledPhysicsState& X,uint8& Kind,uint64& StableId)
   {
    const int32 Index1564=Calls1564++;RangeCapacity1564=BatchWheel1564.GetAllocatedSize();
    TestTrue(TEXT("live range callback receives exact original record in order"),&X==&Wheel1564[Index1564]);
    if(Index1564==FailAt1564)return false;
    Kind=uint8(Index1564%2);StableId=Kind ? MAX_uint64-uint64(Index1564) : 0;return true;
   };
   const bool Accepted1564=Speed::SelectedStateBytesForTesting::AppendWheelBumpHistory1564(BatchWheel1564,Wheel1564,0x15640001u,Identity1564,FailureIndex1564);
   const int32 Successful1564=FailAt1564==INDEX_NONE ? Count1564 : FailAt1564;
   TestEqual(TEXT("range identity result preserved"),Accepted1564,FailAt1564==INDEX_NONE);
   TestEqual(TEXT("range failure index exact"),FailureIndex1564,FailAt1564);
   TestEqual(TEXT("range callback stops at first refusal"),Calls1564,Successful1564+(FailAt1564==INDEX_NONE ? 0 : 1));
   for(int32 Index1564=0;Index1564<Successful1564;++Index1564)
   {
    const auto& X=Wheel1564[Index1564];const uint8 Kind=uint8(Index1564%2);const uint64 Id=Kind ? MAX_uint64-uint64(Index1564) : 0;
    LegacyContactWheeled(LegacyWheel1564,X);W::Add(LegacyWheel1564,uint32(0x15640001u));W::Add(LegacyWheel1564,Kind);W::Add(LegacyWheel1564,Id);W::Add(LegacyWheel1564,X.CarBumpCooldownFrames);
    ReferenceContactWheeled(ReferenceWheel1564,X);ReferenceScalar(ReferenceWheel1564,uint32(0x15640001u));ReferenceScalar(ReferenceWheel1564,Kind);ReferenceScalar(ReferenceWheel1564,Id);ReferenceScalar(ReferenceWheel1564,X.CarBumpCooldownFrames);
   }
   TestEqual(TEXT("range trim retains complete appended backing capacity"),BatchWheel1564.GetAllocatedSize(),RangeCapacity1564);
   TestEqual(TEXT("range refusal retains only fully validated records"),BatchWheel1564.Num(),int32(sizeof(Prefix)+sizeof(int32)+Successful1564*95));
   W::Add(BatchWheel1564,Suffix);W::Add(LegacyWheel1564,Suffix);ReferenceScalar(ReferenceWheel1564,Suffix);
   TestTrue(TEXT("range full/partial bytes preserve independent legacy framing"),BatchWheel1564==LegacyWheel1564&&BatchWheel1564==ReferenceWheel1564);
  }
  TestTrue(TEXT("all source history scalar bytes unchanged"),CaptureInputs1564()==InputBefore1564);
 }
 {
  TArray<FWheeledPhysicsState> Wheel1564;Wheel1564.SetNum(2);
  FBasePhysicsState Base1564;
  for(int32 Index1564=0;Index1564<2;++Index1564)FillContactStates(Base1564,Wheel1564[Index1564],uint32(Index1564));
  TArray<uint8> Partial1564,Expected1564;W::Add(Partial1564,Prefix);ReferenceScalar(Expected1564,Prefix);
  int32 Calls1564=0,Failure1564=123;bool Threw1564=false;
  try
  {
   Speed::SelectedStateBytesForTesting::AppendWheelBumpHistory1564(Partial1564,Wheel1564,0x15640001u,
    [&](const FWheeledPhysicsState&,uint8& Kind,uint64& Id)
    {if(Calls1564++==1)throw std::runtime_error("fixture identity failure");Kind=1;Id=17;return true;},Failure1564);
  }
  catch(const std::runtime_error&){Threw1564=true;}
  ReferenceContactWheeled(Expected1564,Wheel1564[0]);ReferenceScalar(Expected1564,uint32(0x15640001u));
  ReferenceScalar(Expected1564,uint8(1));ReferenceScalar(Expected1564,uint64(17));ReferenceScalar(Expected1564,Wheel1564[0].CarBumpCooldownFrames);
  TestTrue(TEXT("range exception retains prior committed bytes without tail"),Threw1564&&Calls1564==2&&Partial1564==Expected1564);
 }
 TestTrue(TEXT("range extent exact boundary admitted"),Speed::SelectedStateBytesForTesting::Private::HistoryRangeFits1564(MAX_int32-156,1,156));
 TestFalse(TEXT("range overflow rejected before allocation"),Speed::SelectedStateBytesForTesting::Private::HistoryRangeFits1564(MAX_int32-155,1,156));
 TestFalse(TEXT("range negative count rejected"),Speed::SelectedStateBytesForTesting::Private::HistoryRangeFits1564(0,-1,95));
 {
  // Reinterpret only constructs the view; alias rejection must happen before
  // any int32 dereference or destination resize, so no fabricated object read.
  TArray<uint8> Aliased1564;Aliased1564.Init(0x5a,32);const auto Original1564=Aliased1564;
  TConstArrayView<int32> View1564(reinterpret_cast<const int32*>(Aliased1564.GetData()),1);
  TestFalse(TEXT("aliased scalar input rejected before mutation"),Speed::SelectedStateBytesForTesting::AppendFrameHistory1564(Aliased1564,View1564));
  TestTrue(TEXT("alias refusal keeps original bytes"),Aliased1564==Original1564);
 }

 int Object=7,Unknown=9;
 const Speed::FFramePointerKey Key{1,41,2,7,0};Speed::FFramePointerRegistry Registry;
 if(!TestTrue(TEXT("explicit pointer registration"),Registry.Register(&Object,Key))
  ||!TestTrue(TEXT("sealed pointer registry"),Registry.Seal()))return false;
 FVector Vector;FillXYZ(Vector,6);FMatrix Matrix;
 for(int32 Row=0;Row<4;++Row)for(int32 Column=0;Column<4;++Column)
  SetScalarBits(Matrix.M[Row][Column],Row*4+Column);
 TArray<uint8> Actual,Expected,ActualCanonical,ExpectedCanonical;
 TArray<Speed::FFramePointerObservation> ActualEvents,ExpectedEvents;
 {
  Speed::FFramePointerCaptureScope Scope(Actual,Registry);
  W::Add(Actual,Prefix);W::Add(Actual,Vector);W::Add(Actual,&Object);
  W::Add(Actual,Matrix);W::Add(Actual,static_cast<int*>(nullptr));W::Add(Actual,Suffix);
  ActualEvents=Scope.Observations();
  TestTrue(TEXT("numeric codec pointer stream canonicalizes"),Scope.MakeCanonical(ActualCanonical));
 }
 {
  Speed::FFramePointerCaptureScope Scope(Expected,Registry);
  ReferenceScalar(Expected,Prefix);ReferenceXYZ(Expected,Vector);ReferencePointer(Expected,&Object);
  ReferenceMatrix(Expected,Matrix);ReferencePointer(Expected,nullptr);ReferenceScalar(Expected,Suffix);
  ExpectedEvents=Scope.Observations();
  TestTrue(TEXT("independent field stream canonicalizes"),Scope.MakeCanonical(ExpectedCanonical));
 }
 TestTrue(TEXT("interleaved pointer raw bytes identical"),Actual==Expected);
 TestTrue(TEXT("same-phase pointer canonical bytes identical"),ActualCanonical==ExpectedCanonical);
 TestEqual(TEXT("actual pointer event count"),ActualEvents.Num(),2);
 TestEqual(TEXT("reference pointer event count"),ExpectedEvents.Num(),2);
 if(ActualEvents.Num()==2&&ExpectedEvents.Num()==2)
 {
  TestEqual(TEXT("first pointer offset has no inserted length prefix"),ActualEvents[0].Offset,
   int32(sizeof(Prefix)+sizeof(Vector.X)+sizeof(Vector.Y)+sizeof(Vector.Z)));
  TestEqual(TEXT("null offset follows row-major matrix bytes"),ActualEvents[1].Offset,
   int32(ActualEvents[0].Offset+sizeof(UPTRINT)+16*sizeof(Matrix.M[0][0])));
  for(int32 I=0;I<2;++I)
  {
   TestEqual(TEXT("independent pointer offset"),ActualEvents[I].Offset,ExpectedEvents[I].Offset);
   TestEqual(TEXT("independent raw address"),ActualEvents[I].RawAddress,ExpectedEvents[I].RawAddress);
   TestTrue(TEXT("independent semantic identity"),ActualEvents[I].Key==ExpectedEvents[I].Key);
   TestTrue(TEXT("both pointer identities remain known"),ActualEvents[I].bKnown&&ExpectedEvents[I].bKnown);
   TestTrue(TEXT("ordinary pointers retain their event kind"),
    !ActualEvents[I].bObjectId&&!ExpectedEvents[I].bObjectId
    &&!ActualEvents[I].bPairToken&&!ExpectedEvents[I].bPairToken
    &&!ActualEvents[I].bSolverPairToken&&!ExpectedEvents[I].bSolverPairToken);
  }
 }
 {
  TArray<uint8> Raw,Unchanged{99};Speed::FFramePointerCaptureScope Scope(Raw,Registry);
  W::Add(Raw,Vector);W::Add(Raw,&Unknown);
  TestFalse(TEXT("unregistered pointer still refuses canonicalization"),Scope.MakeCanonical(Unchanged));
  TestTrue(TEXT("unknown flag remains visible"),Scope.HasUnknown());
  TestTrue(TEXT("unknown leaves destination untouched"),Unchanged==TArray<uint8>{99});
 }
 {
  TArray<uint8> Raw,Unchanged{99};Speed::FFramePointerCaptureScope Scope(Raw,Registry,1);
  W::Add(Raw,Vector);W::Add(Raw,&Object);W::Add(Raw,Matrix);W::Add(Raw,&Object);
  TestFalse(TEXT("pointer overflow still refuses canonicalization"),Scope.MakeCanonical(Unchanged));
  TestTrue(TEXT("overflow flag remains visible"),Scope.HasOverflow());
  TestTrue(TEXT("overflow leaves destination untouched"),Unchanged==TArray<uint8>{99});
 }
 {
  TArray<uint8> Raw,Unchanged{99};Speed::FFramePointerCaptureScope Scope(Raw,Registry);
  W::Add(Raw,Vector);W::Add(Raw,&Object);
  Raw[Scope.Observations()[0].Offset]^=1;
  TestFalse(TEXT("raw pointer corruption still refuses canonicalization"),Scope.MakeCanonical(Unchanged));
  TestTrue(TEXT("corruption leaves destination untouched"),Unchanged==TArray<uint8>{99});
 }
 TestTrue(TEXT("captured numeric context restores"),Guard.Restore());
 return true;
}
#endif
