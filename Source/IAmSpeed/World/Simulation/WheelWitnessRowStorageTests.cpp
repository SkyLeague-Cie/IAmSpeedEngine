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
 const auto Capture=[&](const ISpeedComponent* Owner,TArray<uint8>& Raw,TArray<uint8>& Canonical,bool Prepared=false)
 {
  FFramePointerRegistry Registry;const FFramePointerKey Key{1,41,2,7,0};
  if(!Registry.Register(Owner,Key)||!Registry.Seal())return false;
  TArray<uint8> Main;FWheelWitnessSnapshot1368 WheelSnapshot;FFramePointerCanonicalSnapshot1456 MainSnapshot;
  if(Prepared&&!WheelSnapshot.ReserveAndPreTouch1368())return false;
  bool Captured=false;
  {
  FFrameSolveWitnessScope Scope(17,Main,Registry);
  if(Prepared&&!Scope.PrepareFreshWheelCaptureStorage1368(WheelSnapshot))return false;
  if(!Scope.EnableWheelManifoldCapture1368())return false;
  FFrameSolveWitnessScope::RecordWheelManifold1368(0,Owner,0,0,{},{});
  FFrameSolveWitnessScope::RecordWheelManifold1368(1,Owner,1,7,{},Orders,Scalars,Doubles,Vectors);
  // A real intervening main-stream row must not leak into the wheel stream.
  SHitResult Hit;FFrameSolveWitnessScope::Record(17,0,0,41,0,0,Hit,false,0,1);
  FFrameSolveWitnessScope::RecordWheelManifold1368(1,Owner,2,14,{},{},TConstArrayView<float>(Scalars.GetData(),1));
  FFrameSolveWitnessScope::RecordWheelManifold1368(255,Owner,3,21,{},{});
  Raw=Scope.RawWheelManifold1368();
  Captured=Scope.NumRows()==1&&Scope.NumWheelManifoldRows1368()==4&&!Scope.HasOverflow()
   &&Scope.MakeWheelManifoldCanonical1368(Canonical);
  if(Prepared&&(!Captured||!Scope.SealAndConsumeCanonicalSnapshot1456(MainSnapshot)
   ||!Scope.ConsumeWheelWitness1368(WheelSnapshot)))return false;
  }
  if(Prepared)
  {
   TArray<uint8> ReleasedRaw,ReleasedCanonical,RawDebris,CanonicalDebris;
   if(WheelSnapshot.ReleaseAfterClosure(ReleasedRaw,ReleasedCanonical,RawDebris,CanonicalDebris)!=EWheelWitnessRelease1368::TransferredValid
    ||ReleasedRaw!=Raw||ReleasedCanonical!=Canonical||!RawDebris.IsEmpty()||!CanonicalDebris.IsEmpty()
    ||ReleasedRaw.GetAllocatedSize()<128*1024||ReleasedCanonical.GetAllocatedSize()<128*1024)return false;
  }
  return Captured;
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
 // Exercise the same authored rows and independent wire reader with fresh
 // prepared capacity. No event storage, framing or release rule is replaced.
 TArray<uint8> PreparedRaw,PreparedCanonical,OtherPreparedRaw,OtherPreparedCanonical;
 TestTrue(TEXT("prepared left capture completes with both real byte capacities"),Capture(Left.Get(),PreparedRaw,PreparedCanonical,true));
 TestTrue(TEXT("prepared right capture completes independently"),Capture(Right.Get(),OtherPreparedRaw,OtherPreparedCanonical,true));
 TestTrue(TEXT("prepared raw matches original byte oracle"),PreparedRaw==LeftRaw&&OtherPreparedRaw==RightRaw);
 TestTrue(TEXT("prepared raw independently decodes every field"),CheckRaw(PreparedRaw,Left.Get())&&CheckRaw(OtherPreparedRaw,Right.Get()));
 TestTrue(TEXT("prepared canonical equals original independent owner oracle"),PreparedCanonical==LeftCanonical&&OtherPreparedCanonical==RightCanonical);
 FFramePointerRegistry Registry;TestTrue(TEXT("prepared refusal registry seals"),Registry.Register(Left.Get(),FFramePointerKey{1,41,2,7,0})&&Registry.Seal());
 {
  FWheelWitnessSnapshot1368 Prepared,Second;Prepared.ReserveAndPreTouch1368();Second.ReserveAndPreTouch1368();
  TArray<uint8> Main,InnerMain;FFrameSolveWitnessScope Outer(17,Main,Registry);
  {FFrameSolveWitnessScope Inner(17,InnerMain,Registry);
   TestFalse(TEXT("inactive outer TLS scope cannot take prepared storage"),Outer.PrepareFreshWheelCaptureStorage1368(Prepared));}
  TestTrue(TEXT("TLS refusal preserves prepared capacity for active outer"),Outer.PrepareFreshWheelCaptureStorage1368(Prepared));
  TestTrue(TEXT("prepared raw capacity adopted before first record"),Outer.RawWheelManifold1368().IsEmpty()&&Outer.RawWheelManifold1368().GetAllocatedSize()>=128*1024);
  TestFalse(TEXT("scope may adopt fresh wheel bytes only once"),Outer.PrepareFreshWheelCaptureStorage1368(Second));
 }
 {
  FWheelWitnessSnapshot1368 Prepared;Prepared.ReserveAndPreTouch1368();
  {
   TArray<uint8> Main;FFrameSolveWitnessScope Scope(17,Main,Registry);Scope.EnableWheelManifoldCapture1368();
   FFrameSolveWitnessScope::RecordWheelManifold1368(0,Left.Get(),0,0,{},{});
   const TArray<uint8> Before=Scope.RawWheelManifold1368();
   TestFalse(TEXT("enabled nonempty wheel stream refuses adoption"),Scope.PrepareFreshWheelCaptureStorage1368(Prepared));
   TestTrue(TEXT("refusal retains every existing raw byte"),Scope.RawWheelManifold1368()==Before);
   FFrameSolveWitnessScope::RecordWheelManifold1368(255,Left.Get(),0,0,{},{});
  }
  TArray<uint8> Main;FFrameSolveWitnessScope Fresh(17,Main,Registry);
  TestTrue(TEXT("nonempty refusal leaves snapshot pristine and reusable"),Fresh.PrepareFreshWheelCaptureStorage1368(Prepared));
  TestTrue(TEXT("refused snapshot still owns its prepared capacity"),Fresh.RawWheelManifold1368().GetAllocatedSize()>=128*1024);
 }
 {
  FWheelWitnessSnapshot1368 Captured;Captured.ReserveAndPreTouch1368();
  {TArray<uint8> Main;FFrameSolveWitnessScope Scope(17,Main,Registry);Scope.EnableWheelManifoldCapture1368();
   FFrameSolveWitnessScope::RecordWheelManifold1368(0,Left.Get(),0,0,{},{});
   FFrameSolveWitnessScope::RecordWheelManifold1368(255,Left.Get(),0,0,{},{});
   TestTrue(TEXT("nonempty wheel witness captures normally"),Scope.ConsumeWheelWitness1368(Captured));
   TestFalse(TEXT("open captured snapshot cannot prepare capacity"),Captured.ReserveAndPreTouch1368());}
  TArray<uint8> Raw,Canonical,RawDebris,CanonicalDebris,Main;FFrameSolveWitnessScope Scope(17,Main,Registry);
  TestFalse(TEXT("captured snapshot cannot donate capacity to another scope"),Scope.PrepareFreshWheelCaptureStorage1368(Captured));
  TestTrue(TEXT("captured refusal retains valid release"),Captured.ReleaseAfterClosure(Raw,Canonical,RawDebris,CanonicalDebris)==EWheelWitnessRelease1368::TransferredValid);
  TestFalse(TEXT("released snapshot cannot prepare capacity"),Captured.ReserveAndPreTouch1368());
  TestFalse(TEXT("released snapshot cannot donate capacity"),Scope.PrepareFreshWheelCaptureStorage1368(Captured));
 }
 {
  FWheelWitnessSnapshot1368 Prepared;Prepared.ReserveAndPreTouch1368();FFramePointerCanonicalSnapshot1456 MainSnapshot;
  TArray<uint8> Main;FFrameSolveWitnessScope Scope(17,Main,Registry);
  TestTrue(TEXT("empty main stream may be consumed"),Scope.SealAndConsumeCanonicalSnapshot1456(MainSnapshot));
  TestFalse(TEXT("consumed main scope refuses fresh wheel byte adoption"),Scope.PrepareFreshWheelCaptureStorage1368(Prepared));
 }
 {
  FWheelWitnessSnapshot1368 Prepared;Prepared.ReserveAndPreTouch1368();
  {
   TArray<uint8> Main;FFrameSolveWitnessScope Scope(17,Main,Registry);
   TestTrue(TEXT("growable wheel byte preparation succeeds"),Scope.PrepareFreshWheelCaptureStorage1368(Prepared));
   Scope.EnableWheelManifoldCapture1368();FFrameSolveWitnessScope::RecordWheelManifold1368(0,Left.Get(),0,0,{},{});
   for(int32 I=0;I<512;++I)FFrameSolveWitnessScope::RecordWheelManifold1368(1,Left.Get(),I,0,{},Orders,Scalars,Doubles,Vectors);
   FFrameSolveWitnessScope::RecordWheelManifold1368(255,Left.Get(),0,0,{},{});
   TestTrue(TEXT("raw wheel stream grows beyond its hint"),Scope.RawWheelManifold1368().Num()>128*1024);
   TestTrue(TEXT("large wheel stream is not truncated or rejected by hint"),Scope.NumWheelManifoldRows1368()==514&&!Scope.HasOverflow()&&Scope.ConsumeWheelWitness1368(Prepared));
  }
  TArray<uint8> Raw,Canonical,RawDebris,CanonicalDebris;
  TestTrue(TEXT("large captured stream releases normally"),Prepared.ReleaseAfterClosure(Raw,Canonical,RawDebris,CanonicalDebris)==EWheelWitnessRelease1368::TransferredValid);
  TestTrue(TEXT("both real wheel streams grow beyond hint"),Raw.Num()>128*1024&&Canonical.Num()>128*1024&&RawDebris.IsEmpty()&&CanonicalDebris.IsEmpty());
 }
 return !HasAnyErrors();
}
#endif
