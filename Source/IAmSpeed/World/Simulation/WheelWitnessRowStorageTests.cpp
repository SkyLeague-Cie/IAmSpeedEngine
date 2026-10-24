#include "FrameSolveWitness.h"
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "Misc/AutomationTest.h"
#include "UObject/StrongObjectPtr.h"
#include <type_traits>
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "Components/BoxComponent.h"
#include "IAmSpeed/Base/EpisodeStateBytesForTesting.h"
#include "FrameFloatingPointContext.h"

namespace Speed
{
// Private read-only test access: no storage accessor enters the public API.
struct FWheelWitnessPreparedStorageTestAccess1368
{
 struct FState {const void* Data[5]{};SIZE_T Allocated[5]{};int32 Num[5]{};};
 static FState Read(const TArray<uint8>* const Buffers[5])
 {
  FState State;
  for(int32 I=0;I<5;++I){State.Data[I]=Buffers[I]->GetData();State.Allocated[I]=Buffers[I]->GetAllocatedSize();State.Num[I]=Buffers[I]->Num();}
  return State;
 }
 static FState Snapshot(const FWheelWitnessSnapshot1368& S)
 {
  const TArray<uint8>* Buffers[]={&S.Raw,&S.Canonical,&S.PreparedRowRaw1368,&S.PreparedRowCanonical1368,&S.PreparedRowBuild1368};return Read(Buffers);
 }
 static FState Scope(const FFrameSolveWitnessScope& S)
 {
  const TArray<uint8>* Buffers[]={&S.WheelRaw1368,&S.WheelCanonical1368,&S.WheelRowRawScratch1368,&S.WheelRowCanonicalScratch1368,&S.WheelRowCanonicalBuildScratch1368};return Read(Buffers);
 }
 static bool Same(const FState& A,const FState& B)
 {for(int32 I=0;I<5;++I)if(A.Data[I]!=B.Data[I]||A.Allocated[I]!=B.Allocated[I]||A.Num[I]!=B.Num[I])return false;return true;}
 static bool Prepared(const FState& S)
 {for(int32 I=0;I<5;++I)if(S.Num[I]||S.Allocated[I]<(I<2?128*1024:8*1024))return false;return true;}
 static bool Empty(const FState& S)
 {for(int32 I=0;I<5;++I)if(S.Num[I]||S.Allocated[I]||S.Data[I])return false;return true;}
};
}

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
  const auto PreparedBefore=FWheelWitnessPreparedStorageTestAccess1368::Snapshot(WheelSnapshot);
  if(Prepared&&!FWheelWitnessPreparedStorageTestAccess1368::Prepared(PreparedBefore))return false;
  bool Captured=false;
  {
  FFrameSolveWitnessScope Scope(17,Main,Registry);
  if(Prepared&&!Scope.PrepareFreshWheelCaptureStorage1368(WheelSnapshot))return false;
  if(Prepared&&(!FWheelWitnessPreparedStorageTestAccess1368::Same(PreparedBefore,FWheelWitnessPreparedStorageTestAccess1368::Scope(Scope))
   ||!FWheelWitnessPreparedStorageTestAccess1368::Empty(FWheelWitnessPreparedStorageTestAccess1368::Snapshot(WheelSnapshot))))return false;
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
 // RecordWheelManifold1368 serializes the interface address, which need not
 // equal the concrete UObject address under multiple inheritance.
 const ISpeedComponent* const PreparedOwner=Left.Get();
 FFramePointerRegistry Registry;TestTrue(TEXT("prepared refusal registry seals"),Registry.Register(PreparedOwner,FFramePointerKey{1,41,2,7,0})&&Registry.Seal());
 TestNotNull(TEXT("prepared registry declares the exact serialized owner interface"),Registry.Find(reinterpret_cast<UPTRINT>(PreparedOwner)));
 {
  FWheelWitnessSnapshot1368 Prepared,Second;Prepared.ReserveAndPreTouch1368();Second.ReserveAndPreTouch1368();
  TArray<uint8> Main,InnerMain;FFrameSolveWitnessScope Outer(17,Main,Registry);
  const auto Before=FWheelWitnessPreparedStorageTestAccess1368::Snapshot(Prepared);
  const auto OuterBefore=FWheelWitnessPreparedStorageTestAccess1368::Scope(Outer);
  {FFrameSolveWitnessScope Inner(17,InnerMain,Registry);
   TestFalse(TEXT("inactive outer TLS scope cannot take prepared storage"),Outer.PrepareFreshWheelCaptureStorage1368(Prepared));
   TestTrue(TEXT("TLS refusal leaves all five donor and destination buffers untouched"),FWheelWitnessPreparedStorageTestAccess1368::Same(Before,FWheelWitnessPreparedStorageTestAccess1368::Snapshot(Prepared))&&FWheelWitnessPreparedStorageTestAccess1368::Same(OuterBefore,FWheelWitnessPreparedStorageTestAccess1368::Scope(Outer)));}
  TestTrue(TEXT("TLS refusal preserves prepared capacity for active outer"),Outer.PrepareFreshWheelCaptureStorage1368(Prepared));
  TestTrue(TEXT("all five prepared arrays transfer by exact ownership"),FWheelWitnessPreparedStorageTestAccess1368::Same(Before,FWheelWitnessPreparedStorageTestAccess1368::Scope(Outer))&&FWheelWitnessPreparedStorageTestAccess1368::Empty(FWheelWitnessPreparedStorageTestAccess1368::Snapshot(Prepared)));
  TestTrue(TEXT("prepared raw capacity adopted before first record"),Outer.RawWheelManifold1368().IsEmpty()&&Outer.RawWheelManifold1368().GetAllocatedSize()>=128*1024);
  TestFalse(TEXT("scope may adopt fresh wheel bytes only once"),Outer.PrepareFreshWheelCaptureStorage1368(Second));
  TestTrue(TEXT("double-adoption refusal retains every second prepared capacity"),FWheelWitnessPreparedStorageTestAccess1368::Prepared(FWheelWitnessPreparedStorageTestAccess1368::Snapshot(Second)));
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
 {
  FFramePointerKey Key{1,41,2,7,0};for(uint64 I=0;I<1024;++I)Key.Membership.Add(I);
  FFramePointerRegistry LargeRegistry;TestTrue(TEXT("large semantic membership registry seals"),LargeRegistry.Register(PreparedOwner,Key)&&LargeRegistry.Seal());
  FWheelWitnessSnapshot1368 Prepared;Prepared.ReserveAndPreTouch1368();TArray<uint8> OriginalRaw,ActualCanonical;
  {
   TArray<uint8> Main;FFrameSolveWitnessScope Scope(17,Main,LargeRegistry);
   TestTrue(TEXT("large membership fixture adopts prepared storage"),Scope.PrepareFreshWheelCaptureStorage1368(Prepared));
   Scope.EnableWheelManifoldCapture1368();
   FFrameSolveWitnessScope::RecordWheelManifold1368(0,PreparedOwner,0,0,{},{});
   FFrameSolveWitnessScope::RecordWheelManifold1368(1,PreparedOwner,0,0,{},{});
   FFrameSolveWitnessScope::RecordWheelManifold1368(255,PreparedOwner,0,0,{},{});
   OriginalRaw=Scope.RawWheelManifold1368();
   TestTrue(TEXT("large membership canonical rows remain valid"),Scope.MakeWheelManifoldCanonical1368(ActualCanonical));
   const auto Storage=FWheelWitnessPreparedStorageTestAccess1368::Scope(Scope);
   TestTrue(TEXT("both alternating canonical row scratches grow beyond 8KiB hint"),Storage.Allocated[3]>8*1024&&Storage.Allocated[4]>8*1024);
   TestTrue(TEXT("large membership witness consumes without truncation"),Scope.NumWheelManifoldRows1368()==3&&!Scope.HasOverflow()&&Scope.ConsumeWheelWitness1368(Prepared));
  }
  // Independent legacy scalar encoding at the sole owner-pointer field. The
  // surrounding authored raw bytes and row framing retain their exact order.
  TArray<uint8> Expected;int32 At=0;bool Framing=true;
  const auto Scalar=[&](const auto& V){const int32 Offset=Expected.AddUninitialized(sizeof(V));FMemory::Memcpy(Expected.GetData()+Offset,&V,sizeof(V));};
  constexpr int32 OwnerOffset=sizeof(uint32)+sizeof(uint64)+2*sizeof(int32)+sizeof(uint8);
  constexpr int32 PointerBytes=sizeof(UPTRINT);
  for(int32 Row=0;Row<3;++Row)
  {
   if(At>OriginalRaw.Num()-8){Framing=false;break;}
   uint32 Magic,Bytes;FMemory::Memcpy(&Magic,OriginalRaw.GetData()+At,4);FMemory::Memcpy(&Bytes,OriginalRaw.GetData()+At+4,4);At+=8;
   if(Magic!=0x334d5749||Bytes<uint32(OwnerOffset+sizeof(UPTRINT))||Bytes>uint32(OriginalRaw.Num()-At)){Framing=false;break;}
   Scalar(Magic);Scalar(uint32(int32(Bytes)-PointerBytes+40+Key.Membership.Num()*int32(sizeof(uint64))));
   Expected.Append(OriginalRaw.GetData()+At,OwnerOffset);
   Scalar(Key.Domain);Scalar(Key.Owner);Scalar(Key.Generation);Scalar(Key.Role);Scalar(Key.Element);Scalar(uint32(Key.Membership.Num()));for(uint64 Id:Key.Membership)Scalar(Id);
   Expected.Append(OriginalRaw.GetData()+At+OwnerOffset+PointerBytes,int32(Bytes)-OwnerOffset-PointerBytes);At+=int32(Bytes);
  }
  TestTrue(TEXT("large canonical scratch growth preserves independent scalar wire oracle"),Framing&&At==OriginalRaw.Num()&&Expected==ActualCanonical);
  TArray<uint8> Raw,Canonical,RawDebris,CanonicalDebris;
  TestTrue(TEXT("large membership snapshot releases normally"),Prepared.ReleaseAfterClosure(Raw,Canonical,RawDebris,CanonicalDebris)==EWheelWitnessRelease1368::TransferredValid);
  TestTrue(TEXT("large membership release retains all raw and canonical bytes"),Raw==OriginalRaw&&Canonical==Expected&&RawDebris.IsEmpty()&&CanonicalDebris.IsEmpty());
 }
 {
  FWheelWitnessSnapshot1368 Prepared;Prepared.ReserveAndPreTouch1368();TArray<uint8> OriginalRaw;
  {
   TArray<uint8> Main;FFrameSolveWitnessScope Scope(17,Main,Registry);
   TestTrue(TEXT("late-write fixture adopts all fresh storage"),Scope.PrepareFreshWheelCaptureStorage1368(Prepared));
   Scope.EnableWheelManifoldCapture1368();
   FFrameSolveWitnessScope::RecordWheelManifold1368(0,PreparedOwner,0,0,{},{});
   FFrameSolveWitnessScope::RecordWheelManifold1368(255,PreparedOwner,0,0,{},{});
   OriginalRaw=Scope.RawWheelManifold1368();
   TestTrue(TEXT("late-write fixture captures the original framed rows"),Scope.ConsumeWheelWitness1368(Prepared));
   // The object is non-const; simulate an unobserved write through its existing
   // evidence view after transfer. Closure must preserve it as invalid debris.
   const_cast<TArray<uint8>&>(Scope.RawWheelManifold1368()).Add(0x9b);
  }
  TArray<uint8> Raw,Canonical,RawDebris,CanonicalDebris;
  TestTrue(TEXT("fresh row scratch never hides a late unobserved write"),Prepared.ReleaseAfterClosure(Raw,Canonical,RawDebris,CanonicalDebris)==EWheelWitnessRelease1368::TransferredInvalid);
  TestTrue(TEXT("late-write failure preserves original raw and exact debris"),Raw==OriginalRaw&&RawDebris==TArray<uint8>{0x9b}&&!Canonical.IsEmpty()&&CanonicalDebris.IsEmpty());
 }
 // Compare the actual production row capture against the previous complete
 // W::Add chain. Both use the same live pointers in this process; no ASLR hash.
 // The reference retains all per-scalar writes, including the three spans.
 {
  using W=FEpisodeStateBytesForTesting;
  const auto Numeric=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext NumericGuard(Numeric);
  if(!TestTrue(TEXT("contact-tail oracle preserves captured FP context"),NumericGuard.IsApplied()))return false;
  TStrongObjectPtr<UBoxComponent> Surface(NewObject<UBoxComponent>());
  TArray<TStrongObjectPtr<USWheelSubBody>> Wheels;
  for(int32 I=0;I<32;++I)Wheels.Emplace(NewObject<USWheelSubBody>());
  const uint64 DoubleBits[]={0ull,0x8000000000000000ull,1ull,0x8000000000000001ull,
   0x000fffffffffffffull,0x0010000000000000ull,0x3ff0000000000001ull,0xc004000000000001ull,
   0x7fefffffffffffffull,0xffefffffffffffffull,0x7ff0000000000000ull,0xfff0000000000000ull,
   0x7ff8000000000001ull,0x7ff8000000000002ull,0xfff8000000000123ull,0x3fd5555555555555ull,
   0x3fe0000000000001ull,0xbfe0000000000001ull,0x0008000000000001ull};
  const uint32 FloatBits[]={0u,0x80000000u,1u,0x80000001u,0x007fffffu,0x00800000u,
   0x3f800001u,0xc0200001u,0x7f7fffffu,0xff7fffffu,0x7f800000u,0xff800000u,
   0x7fc00001u,0x7fc00002u,0xffc00123u,0x3eaaaaabu,0x3f000001u,0xbf000001u,0x00400001u};
  for(uint32 Case=0;Case<19;++Case)for(const int32 Count:{0,1,4,32})
  {
   TArray<SWheelGroundContact> Contacts;Contacts.SetNum(Count);
   FFramePointerRegistry Identities;FFramePointerKey OwnerKey{1,41,2,7,0};OwnerKey.Membership={19,31};
   bool Declared=Identities.Register(PreparedOwner,OwnerKey)
    &&Identities.Register(Surface.Get(),FFramePointerKey{4,1,2,0,0});
   for(int32 I=0;I<Count;++I)
   {
    auto& Contact=Contacts[I];Contact.Wheel=Wheels[I].Get();
    Contact.SurfaceComponent=(I+Case)%3==0?nullptr:Surface.Get();
    Declared&=Identities.Register(Contact.Wheel,FFramePointerKey{3,41,2,0,uint64(I+1)});
    FVector* VectorFields[]={&Contact.SurfacePoint,&Contact.WorldPos,&Contact.Normal,&Contact.r};
    uint32 Field=0;
    for(auto* Vector:VectorFields)
    {
     double* Components[]={&Vector->X,&Vector->Y,&Vector->Z};
     for(auto* Component:Components){const uint64 Bits=DoubleBits[(Case+I+Field++)%19];FMemory::Memcpy(Component,&Bits,sizeof(Bits));}
    }
    Contact.SurfaceFaceIndex=I%3==0?MIN_int32:I%3==1?MAX_int32:I;
    Contact.SurfaceSourceId=~uint64(0)-uint64(I);Contact.SurfaceId=(uint64(1)<<63)|uint64(Case+I);
    Contact.SurfaceFeatureId=0x123456789abcdef0ull+uint64(I);
    float* FloatFields[]={&Contact.vN,&Contact.InvMassEff,&Contact.SpringDisplacement};
    for(auto* Value:FloatFields){const uint32 Bits=FloatBits[(Case+I+Field++)%19];FMemory::Memcpy(Value,&Bits,sizeof(Bits));}
    Contact.bVelocityLocked=((Case+I)&1)!=0;Contact.bNewContact=((Case+I)&2)!=0;Contact.bAtBumpStop=((Case+I)&4)!=0;
   }
   if(!TestTrue(TEXT("contact-tail declares all exact serialized pointer identities"),Declared&&Identities.Seal()))return false;
   TArray<uint8> ActualRaw,ActualCanonical,LegacyRaw,LegacyCanonical;
   const auto BeforeFP=CaptureFrameFloatingPointContext();
   {
    TArray<uint8> Main;FFrameSolveWitnessScope Scope(17,Main,Identities);
    if(!Scope.EnableWheelManifoldCapture1368())return false;
    FFrameSolveWitnessScope::RecordWheelManifold1368(0,PreparedOwner,0,0,Contacts,Orders,Scalars,Doubles,Vectors);
    FFrameSolveWitnessScope::RecordWheelManifold1368(1,PreparedOwner,1,7,Contacts,Orders,Scalars,Doubles,Vectors);
    FFrameSolveWitnessScope::RecordWheelManifold1368(255,PreparedOwner,2,14,Contacts,Orders,Scalars,Doubles,Vectors);
    ActualRaw=Scope.RawWheelManifold1368();
    if(!TestTrue(TEXT("contact-tail production capture has all3 certified rows"),!Scope.HasOverflow()
     &&Scope.NumWheelManifoldRows1368()==3&&Scope.MakeWheelManifoldCanonical1368(ActualCanonical)))return false;
   }
   const auto AfterFP=CaptureFrameFloatingPointContext();
   TestTrue(TEXT("contact-tail recording preserves full entry FP context"),BeforeFP.bValid&&AfterFP.bValid
    &&BeforeFP.VectorControl==AfterFP.VectorControl
    &&FMemory::Memcmp(&BeforeFP.Environment,&AfterFP.Environment,sizeof(std::fenv_t))==0);
   for(int32 RowIndex=0;RowIndex<3;++RowIndex)
   {
    TArray<uint8> Row,CanonicalRow;
    FFramePointerCaptureScope Pointers(Row,Identities);
    const uint8 Event=RowIndex==0?0:RowIndex==1?1:255;
    W::Add(Row,uint32(3));W::Add(Row,uint64(17));W::Add(Row,RowIndex);W::Add(Row,int32(0));
    W::Add(Row,Event);W::Add(Row,PreparedOwner);W::Add(Row,RowIndex);W::Add(Row,int32(RowIndex*7));W::Add(Row,Contacts.Num());
    for(const auto& Contact:Contacts)
    {
     W::Add(Row,Contact.Wheel);W::Add(Row,Contact.SurfaceComponent.GetEvenIfUnreachable());W::Add(Row,Contact.SurfaceComponent.IsExplicitlyNull());
     W::Add(Row,Contact.SurfacePoint);W::Add(Row,Contact.SurfaceFaceIndex);W::Add(Row,Contact.SurfaceSourceId);W::Add(Row,Contact.SurfaceId);W::Add(Row,Contact.SurfaceFeatureId);
     W::Add(Row,Contact.WorldPos);W::Add(Row,Contact.Normal);W::Add(Row,Contact.r);W::Add(Row,Contact.vN);W::Add(Row,Contact.InvMassEff);
     W::Add(Row,Contact.SpringDisplacement);W::Add(Row,Contact.bVelocityLocked);W::Add(Row,Contact.bNewContact);W::Add(Row,Contact.bAtBumpStop);
    }
    W::Add(Row,Orders.Num());for(const auto V:Orders)W::Add(Row,V);
    W::Add(Row,Scalars.Num());for(const auto V:Scalars)W::Add(Row,V);
    W::Add(Row,Doubles.Num());for(const auto V:Doubles)W::Add(Row,V);
    W::Add(Row,Vectors.Num());for(const auto& V:Vectors)W::Add(Row,V);
    const auto& Observations=Pointers.Observations();
    constexpr int32 OwnerOffset=sizeof(uint32)+sizeof(uint64)+2*sizeof(int32)+sizeof(uint8);
    constexpr int32 FirstContactOffset=OwnerOffset+sizeof(UPTRINT)+3*sizeof(int32);
    constexpr int32 ContactBytes=2*sizeof(UPTRINT)+140;
    bool Offsets=Observations.Num()==1+2*Count&&Observations[0].Offset==OwnerOffset;
    for(int32 I=0;I<Count&&Offsets;++I)Offsets=Observations[1+2*I].Offset==FirstContactOffset+I*ContactBytes
     &&Observations[2+2*I].Offset==FirstContactOffset+I*ContactBytes+int32(sizeof(UPTRINT));
    if(!TestTrue(TEXT("legacy contact oracle retains owner/wheel/surface canonical pointer offsets"),Offsets)
     ||!TestTrue(TEXT("legacy contact oracle canonicalizes every exact pointer"),Pointers.MakeCanonical(CanonicalRow)))return false;
    W::Add(LegacyRaw,uint32(0x334d5749));W::Add(LegacyRaw,uint32(Row.Num()));LegacyRaw.Append(Row);
    W::Add(LegacyCanonical,uint32(0x334d5749));W::Add(LegacyCanonical,uint32(CanonicalRow.Num()));LegacyCanonical.Append(CanonicalRow);
   }
   const FString Label=FString::Printf(TEXT("contact-tail bits%u count%d"),Case,Count);
   TestTrue(Label+TEXT(" complete raw bytes match previous Add chain"),ActualRaw==LegacyRaw);
   TestTrue(Label+TEXT(" complete canonical bytes and row framing match previous Add chain"),ActualCanonical==LegacyCanonical);
  }
 }
 return !HasAnyErrors();
}
#endif
