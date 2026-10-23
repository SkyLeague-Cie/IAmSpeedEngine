#include "IAmSpeed/Base/EpisodeShapeFieldsForTesting.h"
#include "IAmSpeed/World/Simulation/NativeContactStateBytes1221.h"
#include "IAmSpeed/World/Simulation/FrameFloatingPointContext.h"
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "Misc/AutomationTest.h"

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
  Speed::Testing1221::AppendContactBaseState(New,Base);Speed::Testing1221::AppendContactWheeledState(New,Wheel);
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
    Speed::Testing1221::AppendContactBaseState(New,Base);LegacyContactBase(Legacy,Base);ReferenceContactBase(Reference,Base);
   }
   W::Add(New,int32(1024));W::Add(Legacy,int32(1024));ReferenceScalar(Reference,int32(1024));
   for(uint32 Slot=0;Slot<1024;++Slot)
   {
    FBasePhysicsState Base;FWheeledPhysicsState Wheel;FillContactStates(Base,Wheel,Slot+Car*1024);
    Speed::Testing1221::AppendContactWheeledState(New,Wheel);LegacyContactWheeled(Legacy,Wheel);ReferenceContactWheeled(Reference,Wheel);
   }
  }
  TestTrue(TEXT("1221 both cars retain all1024 base/frame/wheeled history slots and count prefixes"),New==Legacy&&New==Reference);
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
