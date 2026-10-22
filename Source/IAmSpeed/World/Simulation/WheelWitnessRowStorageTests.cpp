#include "FrameSolveWitness.h"
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "Misc/AutomationTest.h"
#include "UObject/StrongObjectPtr.h"
#include <type_traits>

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FWheelWitnessRowStorageTests,
 "IAmSpeed.Simulation.FrameWitness.WheelRowStorageIsolation",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FWheelWitnessRowStorageTests::RunTest(const FString&)
{
 using namespace Speed;
 TStrongObjectPtr<USpeedWheeledComponent> Left(NewObject<USpeedWheeledComponent>());
 TStrongObjectPtr<USpeedWheeledComponent> Right(NewObject<USpeedWheeledComponent>());
 // Authored payloads exercise growth, shrink and an empty terminal row.
 TArray<int32> Orders;TArray<float> Scalars;TArray<double> Doubles;TArray<FVector> Vectors;
 for(int32 I=0;I<32;++I){Orders.Add(31-I);Scalars.Add(float(I)*0.25f);Doubles.Add(double(I)*-0.125);}
 for(int32 I=0;I<8;++I)Vectors.Add(FVector(double(I),double(I)+0.5,double(I)-0.25));
 TArray<uint8> LeftRaw,RightRaw,LeftCanonical,RightCanonical;
 const auto Capture=[&](const ISpeedComponent* Owner,TArray<uint8>& Raw,TArray<uint8>& Canonical)
 {
  FFramePointerRegistry Registry;const FFramePointerKey Key{1,41,2,7,0};
  if(!Registry.Register(Owner,Key)||!Registry.Seal())return false;
  TArray<uint8> Main;FFrameSolveWitnessScope Scope(17,Main,Registry);
  if(!Scope.EnableWheelManifoldCapture1368())return false;
  FFrameSolveWitnessScope::RecordWheelManifold1368(0,Owner,0,0,{},{});
  FFrameSolveWitnessScope::RecordWheelManifold1368(1,Owner,1,7,{},Orders,Scalars,Doubles,Vectors);
  // A real intervening main-stream row must not leak into the wheel stream.
  SHitResult Hit;FFrameSolveWitnessScope::Record(17,0,0,41,0,0,Hit,false,0,1);
  FFrameSolveWitnessScope::RecordWheelManifold1368(1,Owner,2,14,{},{},TConstArrayView<float>(Scalars.GetData(),1));
  FFrameSolveWitnessScope::RecordWheelManifold1368(255,Owner,3,21,{},{});
  Raw=Scope.RawWheelManifold1368();
  return Scope.NumRows()==1&&Scope.NumWheelManifoldRows1368()==4&&!Scope.HasOverflow()
   &&Scope.MakeWheelManifoldCanonical1368(Canonical);
 };
 if(!TestTrue(TEXT("left multi-event capture completes"),Capture(Left.Get(),LeftRaw,LeftCanonical))
  ||!TestTrue(TEXT("right multi-event capture completes"),Capture(Right.Get(),RightRaw,RightCanonical)))return false;

 // Independent reader of the documented framed binary wire. It never calls
 // the production field encoder or canonical projector to construct expected
 // rows: every consumed field is checked against the authored inputs above.
 const auto CheckRaw=[&](const TArray<uint8>& Bytes,const ISpeedComponent* Owner)
 {
  int32 Offset=0;int32 PreviousBytes=0;
  const auto Read=[&](auto Expected)
  {
   using T=decltype(Expected);static_assert(std::is_arithmetic_v<T>);
   if(Offset<0||Offset>Bytes.Num()-int32(sizeof(T)))return false;
   T Actual;FMemory::Memcpy(&Actual,Bytes.GetData()+Offset,sizeof(T));Offset+=sizeof(T);
   return FMemory::Memcmp(&Actual,&Expected,sizeof(T))==0;
  };
  for(uint32 Row=0;Row<4;++Row)
  {
   if(!Read(uint32(0x334d5749))||Offset>Bytes.Num()-int32(sizeof(uint32)))return false;
   uint32 RowBytes=0;FMemory::Memcpy(&RowBytes,Bytes.GetData()+Offset,sizeof(RowBytes));Offset+=sizeof(RowBytes);
   if(RowBytes>uint32(Bytes.Num()-Offset))return false;
   const int32 End=Offset+int32(RowBytes);const uint8 Event=Row==0?0:Row==3?255:1;
   if(!Read(uint32(3))||!Read(uint64(17))||!Read(Row)||!Read(int32(Row>=2?1:0))
    ||!Read(Event)||!Read(reinterpret_cast<UPTRINT>(Owner))||!Read(int32(Row))||!Read(int32(Row*7))
    ||!Read(int32(0)))return false; // Explicit zero contact count, not omitted.
   const int32 Count=Row==1?32:0;
   if(!Read(Count))return false;for(int32 I=0;I<Count;++I)if(!Read(Orders[I]))return false;
   const int32 FloatCount=Row==1?32:Row==2?1:0;
   if(!Read(FloatCount))return false;for(int32 I=0;I<FloatCount;++I)if(!Read(Scalars[I]))return false;
   if(!Read(Count))return false;for(int32 I=0;I<Count;++I)if(!Read(Doubles[I]))return false;
   const int32 VectorCount=Row==1?8:0;
   if(!Read(VectorCount))return false;
   for(int32 I=0;I<VectorCount;++I)if(!Read(Vectors[I].X)||!Read(Vectors[I].Y)||!Read(Vectors[I].Z))return false;
   if(Offset!=End)return false; // No stale tail bytes after the large row.
   if(Row==2&&int32(RowBytes)>=PreviousBytes)return false;
   PreviousBytes=int32(RowBytes);
  }
  return Offset==Bytes.Num();
 };
 TestTrue(TEXT("left raw stream independently decodes every exact field and row"),CheckRaw(LeftRaw,Left.Get()));
 TestTrue(TEXT("right raw stream independently decodes every exact field and row"),CheckRaw(RightRaw,Right.Get()));
 TestTrue(TEXT("distinct real owners remain visible in raw pointer bytes"),LeftRaw!=RightRaw);
 TestTrue(TEXT("semantic identity preserves all canonical bytes across owner addresses"),LeftCanonical==RightCanonical);
 // A new scope cannot retain the prior scope's largest row or sequence.
 TArray<uint8> AgainRaw,AgainCanonical;
 TestTrue(TEXT("second scope completes independently"),Capture(Left.Get(),AgainRaw,AgainCanonical));
 TestTrue(TEXT("same authored event sequence produces identical raw bytes"),AgainRaw==LeftRaw);
 TestTrue(TEXT("same authored event sequence produces identical canonical bytes"),AgainCanonical==LeftCanonical);
 return !HasAnyErrors();
}
#endif
