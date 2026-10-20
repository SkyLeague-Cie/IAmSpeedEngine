#include "FrameSolveWitness.h"
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "Misc/AutomationTest.h"
#include "SimulationWorld.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "UObject/UObjectGlobals.h"
namespace
{
using namespace Speed;
using CW=FFrameSolveWitnessScope;
using CE=EFrameDynamicContactEvent;
using CP=EFrameDynamicContactPass;
using CO=EFrameDynamicContactOutcome;
constexpr auto DynamicContactAutomationFlags1075=EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDynamicContactOrder1075,"IAmSpeed.Simulation.FrameWitness.DynamicContactOrder1075",DynamicContactAutomationFlags1075)
bool FDynamicContactOrder1075::RunTest(const FString&)
{
 FFramePointerRegistry R;R.Seal();SHitResult H;H.TOI=0.25f;
 const auto Capture=[&](bool Reverse)
 {
  TArray<uint8> Raw,Can;CW Scope(9,Raw,R);
  const uint64 P=CW::BeginDynamicPass(9,CP::Persistent,1.0f/300.0f,0,false,false,0);
  const auto Dynamic=[&] {CW::RecordDynamicEvent(P,CE::NormalBeforeA,0,0,nullptr,nullptr,nullptr);};
  const auto Main=[&] {CW::Record(9,0,0,1,0,0,H,false,0,1);};
  if(Reverse){Dynamic();Main();}else{Main();Dynamic();}
  CW::EndDynamicPass(P,0);TestEqual(TEXT("one shared actual row sequence"),Scope.NumRows(),4);
  TestEqual(TEXT("one real main resolve"),Scope.NumMainRows(),1);
  TestEqual(TEXT("three dynamic carrier rows"),Scope.NumDynamicRows(),3);
  TestTrue(TEXT("complete interleaved stream"),Scope.MakeCanonical(Can));return Can;
 };
 TestTrue(TEXT("same unordered events cannot hide changed main/dynamic order"),Capture(false)!=Capture(true));
 return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDynamicContactPassClosure1075,"IAmSpeed.Simulation.FrameWitness.DynamicContactPassClosure1075",DynamicContactAutomationFlags1075)
bool FDynamicContactPassClosure1075::RunTest(const FString&)
{
 FFramePointerRegistry R;R.Seal();
 {
  TArray<uint8> Raw,Can;CW Scope(9,Raw,R);
  const uint64 P=CW::BeginDynamicPass(9,CP::Persistent,0.01f,0,false,false,0);
  TestFalse(TEXT("open pass cannot certify completion"),Scope.MakeCanonical(Can));
  CW::EndDynamicPass(P,0);TestTrue(TEXT("empty pass explicitly closed"),Scope.MakeCanonical(Can));
 }
 {
  TArray<uint8> Raw,Can;CW Scope(9,Raw,R);
  const uint64 A=CW::BeginDynamicPass(9,CP::Persistent,0.01f,0,false,false,0);
  const uint64 B=CW::BeginDynamicPass(9,CP::Projection,0,0,false,false,0);
  CW::EndDynamicPass(A,0);CW::EndDynamicPass(B,0);CW::EndDynamicPass(A,0);
  TestFalse(TEXT("wrong nesting is permanently UNKNOWN"),Scope.MakeCanonical(Can));
 }
 {
  TArray<uint8> Raw,Can;CW Scope(9,Raw,R,1);
  const uint64 P=CW::BeginDynamicPass(9,CP::Persistent,0.01f,0,false,false,0);
  CW::EndDynamicPass(P,0);TestTrue(TEXT("row cap explicit"),Scope.HasOverflow());
  TestFalse(TEXT("dropped end is UNKNOWN"),Scope.MakeCanonical(Can));
 }
 {
  TArray<uint8> Raw,Can;CW Scope(9,Raw,R);
  TestEqual(TEXT("wrong frame rejected"),CW::BeginDynamicPass(10,CP::Persistent,0,0,false,false,0),uint64(0));
  TestFalse(TEXT("wrong frame remains UNKNOWN"),Scope.MakeCanonical(Can));
 }
 return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDynamicPairTokenPacking1075,"IAmSpeed.Simulation.FrameWitness.DynamicPairTokenPacking1075",DynamicContactAutomationFlags1075)
bool FDynamicPairTokenPacking1075::RunTest(const FString&)
{
 auto* A=NewObject<UBoxSubBody>();auto* B=NewObject<UBoxSubBody>();
 auto* C=NewObject<UBoxSubBody>();auto* D=NewObject<UBoxSubBody>();
 FFramePointerRegistry R,S;FFramePointerKey KA{1,41,1,2,0},KB{1,42,1,2,0};
 R.Register(A,KA);R.Register(B,KB);S.Register(C,KA);S.Register(D,KB);R.Seal();S.Seal();
 // Reverse the words of the actual MakePairKey result. Python source checks tie
 // this persistent layout to both real registration/TOI producers, not a copied enum.
 const auto WorldKey=[](const USSubBody* X,const USSubBody* Y)
 {const uint64 K=USSubBody::MakePairKey(X,Y);return (K<<32)|(K>>32);};
 const auto Capture=[&](const FFramePointerRegistry& Registry,const USSubBody* X,const USSubBody* Y,uint64 Token,bool Valid)
 {
  TArray<uint8> Raw,Can;CW Scope(9,Raw,Registry);
  const uint64 P=CW::BeginDynamicPass(9,CP::Persistent,0.01f,0,false,false,1);
  CW::RecordDynamicEvent(P,CE::NormalBeforeA,0,Token,nullptr,X,Y);
  CW::EndDynamicPass(P,1);
  TestEqual(TEXT("key is validated against actual bodies and declarations"),Scope.MakeCanonical(Can),Valid);
  TestFalse(TEXT("original stream retained"),Raw.IsEmpty());return Can;
 };
 TestTrue(TEXT("separate live object IDs map to the same semantic pair"),
  Capture(R,A,B,WorldKey(A,B),true)==Capture(S,C,D,WorldKey(C,D),true));
 Capture(R,A,B,USSubBody::MakePairKey(A,B),false);
 FFramePointerRegistry Missing;Missing.Seal();Capture(Missing,A,B,WorldKey(A,B),false);
 Capture(R,nullptr,B,WorldKey(A,B),false);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDynamicContactPairFields1075,"IAmSpeed.Simulation.FrameWitness.DynamicContactPairFields1075",DynamicContactAutomationFlags1075)
bool FDynamicContactPairFields1075::RunTest(const FString&)
{
 FFramePointerRegistry R;R.Seal();FDynamicContactPair Pair;
 const auto Capture=[&](const FDynamicContactPair& X)
 {
  TArray<uint8> Raw,Can;CW Scope(9,Raw,R);
  const uint64 P=CW::BeginDynamicPass(9,CP::Persistent,0.01f,0,false,false,1);
  CW::RecordDynamicEvent(P,CE::PairEnd,0,0,&X,nullptr,nullptr);
  CW::EndDynamicPass(P,1);TestTrue(TEXT("explicit pair fields canonical"),Scope.MakeCanonical(Can));return Can;
 };
 const auto Base=Capture(Pair);
#define CHECK_PAIR_FIELD(Name,Value) {auto Changed=Pair;Changed.Name=Value;TestTrue(TEXT(#Name " changes exact stream"),Capture(Changed)!=Base);}
 CHECK_PAIR_FIELD(LocalAnchorA,FVector(1,2,3));CHECK_PAIR_FIELD(LocalAnchorB,FVector(1,2,3));
 CHECK_PAIR_FIELD(LocalNormalB,FVector(1,2,3));CHECK_PAIR_FIELD(LastCOMA,FVector(1,2,3));CHECK_PAIR_FIELD(LastCOMB,FVector(1,2,3));
 CHECK_PAIR_FIELD(FirstSeenFrame,1);CHECK_PAIR_FIELD(LastSeenFrame,1);CHECK_PAIR_FIELD(LastSolvedFrame,1);
 CHECK_PAIR_FIELD(SupportedFrameCount,1);CHECK_PAIR_FIELD(ActiveContactFrameCount,1);CHECK_PAIR_FIELD(ReactionFrameCount,1);CHECK_PAIR_FIELD(FeatureTransitionCount,1);
 CHECK_PAIR_FIELD(AcquisitionNormalSpeed,1);CHECK_PAIR_FIELD(AccumulatedNormalImpulse,1);CHECK_PAIR_FIELD(AccumulatedNormalActivity,1);CHECK_PAIR_FIELD(AccumulatedNormalSupportActivity,1);
 CHECK_PAIR_FIELD(PeakNormalActivity,1);CHECK_PAIR_FIELD(PeakNormalSupportActivity,1);CHECK_PAIR_FIELD(MinimumSeparation,1);CHECK_PAIR_FIELD(MaximumSeparation,1);
 CHECK_PAIR_FIELD(bRollingManifoldReady,true);CHECK_PAIR_FIELD(bActiveContactLastSolve,true);
#undef CHECK_PAIR_FIELD
 return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDynamicRollingContext1075,"IAmSpeed.Simulation.FrameWitness.DynamicRollingContext1075",DynamicContactAutomationFlags1075)
bool FDynamicRollingContext1075::RunTest(const FString&)
{
 FFramePointerRegistry R;R.Seal();FFakePhysicsImpactContext Context;SHitResult Hit;
 const auto Capture=[&](const FFakePhysicsImpactContext& X,const SHitResult& H,CE Event,float Dt)
 {
  TArray<uint8> Raw,Can;CW Scope(9,Raw,R);
  const uint64 P=CW::BeginDynamicPass(9,CP::Persistent,Dt,0,false,true,1);
  CW::RecordDynamicEvent(P,Event,0,0,nullptr,nullptr,nullptr,CO::Completed,true,
   FVector(1,2,3),FVector(4,5,6),7,8,&H,&X);
  CW::EndDynamicPass(P,1);TestTrue(TEXT("full selected rolling arguments canonical"),Scope.MakeCanonical(Can));return Can;
 };
 const auto Base=Capture(Context,Hit,CE::FakePhysicsBefore,0.01f);
#define CHECK_CONTEXT(Name,Value) {auto Changed=Context;Changed.Name=Value;TestTrue(TEXT(#Name " retained"),Capture(Changed,Hit,CE::FakePhysicsBefore,0.01f)!=Base);}
 CHECK_CONTEXT(SelfParentKinematics.Location,FVector(1,2,3));CHECK_CONTEXT(OtherParentKinematics.Velocity,FVector(1,2,3));
 CHECK_CONTEXT(FakeImpulseScale,2);CHECK_CONTEXT(bProjectFakeImpulseOntoContactPlane,true);
 CHECK_CONTEXT(FakeRelativeVelocityBias,FVector(1,2,3));CHECK_CONTEXT(FakeTangentialVelocityScale,2);CHECK_CONTEXT(bUseContactPointRelativeVelocity,true);
#undef CHECK_CONTEXT
 auto ChangedHit=Hit;ChangedHit.TOI=0.25f;
 TestTrue(TEXT("selected hit exact"),Capture(Context,ChangedHit,CE::FakePhysicsBefore,0.01f)!=Base);
 TestTrue(TEXT("before/after distinguished"),Capture(Context,Hit,CE::FakePhysicsAfter,0.01f)!=Base);
 TestTrue(TEXT("actual subinterval retained"),Capture(Context,Hit,CE::FakePhysicsBefore,0.02f)!=Base);
 return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDynamicContactNestedScopes1075,"IAmSpeed.Simulation.FrameWitness.DynamicContactNestedScopes1075",DynamicContactAutomationFlags1075)
bool FDynamicContactNestedScopes1075::RunTest(const FString&)
{
 FFramePointerRegistry R;R.Seal();TArray<uint8> OuterRaw,InnerRaw,Out;
 TestFalse(TEXT("default disabled"),CW::IsActive());
 TestEqual(TEXT("inactive begin has no row or allocation"),CW::BeginDynamicPass(9,CP::Persistent,0,0,false,false,0),uint64(0));
 {
  CW Outer(9,OuterRaw,R);const uint64 A=CW::BeginDynamicPass(9,CP::Persistent,0.01f,0,false,false,0);
  {
   CW Inner(10,InnerRaw,R);const uint64 B=CW::BeginDynamicPass(10,CP::Projection,0,0,false,false,0);
   CW::EndDynamicPass(B,0);TestTrue(TEXT("inner complete"),Inner.MakeCanonical(Out));
  }
  TestEqual(TEXT("outer pass restored after nested scope"),CW::CurrentDynamicPass(),A);
  CW::EndDynamicPass(A,0);TestTrue(TEXT("outer complete"),Outer.MakeCanonical(Out));
  TestEqual(TEXT("inner rows excluded from outer stream"),Outer.NumRows(),2);
 }
 TestFalse(TEXT("disabled restored"),CW::IsActive());return true;
}
#endif
