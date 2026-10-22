#include "IAmSpeed/Base/EpisodeShapeFieldsForTesting.h"
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
