#if WITH_DEV_AUTOMATION_TESTS
#include "JoinedActorKinematics1213.h"
#include "SubBodyKinematicsTasks.h"
#include "FrameParallelExecutor.h"
#include "FrameParallelSettings.h"
#include "IAmSpeed/Actors/SpeedCar.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/SubBodies/SSubBody.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "Math/VectorRegister.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include <cfenv>
#if WITH_EDITOR
#include "AssetCompilingManager.h"
#endif
namespace
{
template<class T> bool NativeFPBits1220(const T& A,const T& B){return FMemory::Memcmp(&A,&B,sizeof(T))==0;}
bool NativeFPKinematics1220(const SKinematic& A,const SKinematic& B)
{
 return NativeFPBits1220(A.Location,B.Location)&&NativeFPBits1220(A.Rotation,B.Rotation)
  &&NativeFPBits1220(A.Velocity,B.Velocity)&&NativeFPBits1220(A.Acceleration,B.Acceleration)
  &&NativeFPBits1220(A.AngularVelocity,B.AngularVelocity)&&NativeFPBits1220(A.AngularAcceleration,B.AngularAcceleration);
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedNativeActorShadowFP1220,
 "IAmSpeed.Simulation.FrameParallel.NativeActorShadowFP1220",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedNativeActorShadowFP1220::RunTest(const FString&)
{
 using namespace Speed;using namespace Speed::FrameParallel;
 const auto Original=CaptureFrameFloatingPointContext();
 if(!TestTrue(TEXT("native exact FP context supported"),Original.bValid))return false;
 FScopedFrameFloatingPointContext RestoreOriginal(Original);
 if(!TestTrue(TEXT("whole fixture numeric restoration guard"),RestoreOriginal.IsApplied()))return false;
 if(!TestTrue(TEXT("owned workers available"),EnsurePrivateFrameWorkers(FFrameParallelPolicy::MaximumLanes-1)>0))return false;
 const auto Options=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(false)
  .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false).SetTransactional(false);
 UWorld* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Options);
 if(!TestNotNull(TEXT("native FP world"),World))return false;
 GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
 ON_SCOPE_EXIT{World->DestroyWorld(false);GEngine->DestroyWorldContext(World);};
 auto* Bridge=World->GetSubsystem<USpeedWorldSubsystem>();
 if(!TestNotNull(TEXT("native adapter registry"),Bridge))return false;
 TArray<USpeedWheeledComponent*> Cars;TArray<ISpeedComponent*> Actors;
 ON_SCOPE_EXIT{for(auto* C:Cars)Bridge->UnregisterSpeedComponent(C);Bridge->ApplyPendingOps();};
 for(int32 A=0;A<2;++A)
 {
  auto* Car=World->SpawnActor<ASpeedCar>();auto* C=Car?Cast<USpeedWheeledComponent>(Car->GetVehicleMovement()):nullptr;
  if(!TestNotNull(TEXT("actual native car for FP oracle"),C))return false;
  C->SetOwner(Car);C->DisableAsyncPhysicsTickForTesting();Cars.Add(C);Actors.Add(C);
  SKinematic K;K.Location=FVector(113.123456789+A,-27.333333333,1000);
  K.Rotation=FRotator(17.1234567,-49.7654321,11.12345).Quaternion();
  K.Velocity=FVector(129.1234567,-733.8751234,5.25123);K.Acceleration=FVector(51.375,-7.1234567,-650.12345);
  K.AngularVelocity=FVector(.12512345,-.3757654,2.2512345);K.AngularAcceleration=FVector(3.12345,1.7512345,-.6251234);
  C->SetKinematicState(K);
  int32 ChildIndex=0;
  for(auto* Child:C->GetSubBodies())
  {
   Child->SetRelativeLocation(FVector(37.123456789+ChildIndex*3.125,-21.333333333,11.142857143));
   Child->SetRelativeRotation(FRotator(7.1234567+ChildIndex,-13.7654321,5.3333333));++ChildIndex;
  }
 }
#if WITH_EDITOR
 // Finish already-requested graphical asset jobs before changing this owner's
 // rounding mode. No asset compilation belongs inside a numeric physics oracle.
 FAssetCompilingManager::Get().FinishAllCompilation();
#endif
 auto* C=Cars[0];bool SawNativeFlagChange=false;
 for(int32 Rounding:{FE_TONEAREST,FE_DOWNWARD,FE_UPWARD,FE_TOWARDZERO})for(bool Dirty:{false,true})for(bool CallbackChangesRounding:{false,true})
 {
  if(!TestEqual(TEXT("owner rounding selectable"),std::fesetround(Rounding),0))return false;
  std::feclearexcept(FE_ALL_EXCEPT);VectorSetControlRegister(VectorGetControlRegister()&~uint32(0x3f));
  if(Dirty)std::feraiseexcept(FE_DIVBYZERO|FE_INEXACT);
  const auto Entry=CaptureFrameFloatingPointContext();
  // The independent oracle is the ORIGINAL native owner method. Its entire
  // fenv + MXCSR exit is observed, not just a rounding mode or final positions.
  C->UpdateSubBodiesKinematics();const auto NativeExit=CaptureFrameFloatingPointContext();
  SawNativeFlagChange|=!SameJoinedActorNumericContext1213(Entry,NativeExit);
  TArray<SKinematic> Expected;for(auto* Child:C->GetSubBodies())Expected.Add(Child->GetKinematicState());
  if(!TestTrue(TEXT("restore identical native entry"),ApplyFrameFloatingPointContext(Entry)))return false;
  ResetJoinedActorKinematicShadowTotals1213();FJoinedActorKinematics1213 Wave;
  if(!TestTrue(TEXT("shadow wave admits two actual actors"),Wave.CaptureAndCompute(Actors,2,1)))return false;
  if(!TestTrue(TEXT("input capture and all worker computation preserve owner FP exactly"),
   SameJoinedActorNumericContext1213(Entry,CaptureFrameFloatingPointContext())))return false;
  FFrameFloatingPointContext ExpectedExit=NativeExit;
  if(CallbackChangesRounding)
  {
   const int32 Changed=Rounding==FE_DOWNWARD?FE_UPWARD:FE_DOWNWARD;
   if(!TestEqual(TEXT("preceding callback changes rounding"),std::fesetround(Changed),0))return false;
   const auto ChangedEntry=CaptureFrameFloatingPointContext();C->UpdateSubBodiesKinematics();ExpectedExit=CaptureFrameFloatingPointContext();
   Expected.Reset();for(auto* Child:C->GetSubBodies())Expected.Add(Child->GetKinematicState());
   if(!ApplyFrameFloatingPointContext(ChangedEntry))return false;
  }
  if(!TestTrue(TEXT("shadow never skips original native owner arithmetic"),Wave.TryInstall(*C)))return false;
  if(!TestTrue(TEXT("complete native FP exit retained including prior flags and callback rounding"),
   SameJoinedActorNumericContext1213(ExpectedExit,CaptureFrameFloatingPointContext())))return false;
  for(int32 I=0;I<C->GetSubBodies().Num();++I)
   if(!TestTrue(TEXT("all original child output bits retained under each FP mode"),NativeFPKinematics1220(Expected[I],C->GetSubBodies()[I]->GetKinematicState())))return false;
  if(CallbackChangesRounding)TestEqual(TEXT("changed callback FP entry is explicit noncomparable shadow"),GetJoinedActorKinematicShadowTotals1213().EntryChanged,uint64(1));
  TestEqual(TEXT("one original owner invocation recorded"),GetJoinedActorKinematicShadowTotals1213().ActorsReplayed,uint64(1));
 }
 TestTrue(TEXT("test actually exercises a native FP flag transition"),SawNativeFlagChange);
 // Deliberately failing shadow work cannot move/replace native execution or
 // leak its worker flags into the owner. The native operation still runs once.
 if(!ApplyFrameFloatingPointContext(Original))return false;
 C->UpdateSubBodiesKinematics();const auto FailureOracleExit=CaptureFrameFloatingPointContext();
 TArray<SKinematic> FailureOracle;for(auto* Child:C->GetSubBodies())FailureOracle.Add(Child->GetKinematicState());
 if(!ApplyFrameFloatingPointContext(Original))return false;
 ResetJoinedActorKinematicShadowTotals1213();FJoinedActorKinematics1213 FailedShadow;FailedShadow.InjectShadowFailureForTesting(0);
 if(!TestTrue(TEXT("failing shadow work still drains its accepted wave"),FailedShadow.CaptureAndCompute(Actors,2,1)))return false;
 TestTrue(TEXT("shadow exception preserves complete owner entry"),SameJoinedActorNumericContext1213(Original,CaptureFrameFloatingPointContext()));
 bool Escaped=false,Replayed=false;
 try{Replayed=FailedShadow.TryInstall(*C);}catch(...){Escaped=true;}
 TestFalse(TEXT("shadow exception never escapes before original native operation"),Escaped);
 TestTrue(TEXT("original operation still executes after failed shadow"),Replayed);
 TestTrue(TEXT("failed shadow retains exact original FP exit"),SameJoinedActorNumericContext1213(FailureOracleExit,CaptureFrameFloatingPointContext()));
 for(int32 I=0;I<C->GetSubBodies().Num();++I)TestTrue(TEXT("failed shadow retains actual native values"),NativeFPKinematics1220(FailureOracle[I],C->GetSubBodies()[I]->GetKinematicState()));
 const auto FailureTotals=GetJoinedActorKinematicShadowTotals1213();
 TestTrue(TEXT("worker failure remains explicit evidence debt"),FailureTotals.WorkerFailures>0&&FailureTotals.ShadowUnavailable==1);
 TestEqual(TEXT("failed shadow does not duplicate owner work"),FailureTotals.ActorsReplayed,uint64(1));

 // Observe that ordinary child calls share a cumulative FP state. A worker
 // implementation cannot independently reset those flags between children and
 // then claim the owner exit or subsequent callback has been preserved.
 std::feclearexcept(FE_ALL_EXCEPT);VectorSetControlRegister(VectorGetControlRegister()&~uint32(0x3f));
 const SKinematic Origin=C->GetOriginKinematicState();uint32 PriorFlags=VectorGetControlRegister()&0x3f;
 const uint32 Controls=VectorGetControlRegister()&~uint32(0x3f);
 for(auto* Child:C->GetSubBodies())
 {
  Child->UpdateKinematicsFromOwner(Origin);const uint32 Current=VectorGetControlRegister();
  TestEqual(TEXT("native children retain entry controls"),Current&~uint32(0x3f),Controls);
  TestTrue(TEXT("native children inherit previous sticky flags in canonical order"),((Current&0x3f)&PriorFlags)==PriorFlags);
  PriorFlags=Current&0x3f;
 }
 // A guard around arithmetic is insufficient if shadow capture warms the
 // canonical helper's TLS quaternion cache. Exercise its cold-miss flags with
 // identical inputs before/after both shadow capture and revalidation.
 auto* Probe=C->GetSubBodies()[0];
 const FRotator Target(13.1234567,-71.23456789,9.3456789);
 for(auto* Car:Cars)for(auto* Body:Car->GetSubBodies())Body->SetRelativeRotation(Target);
 const auto FlushCanonicalCaptureCache=[&]()
 {
  FSubBodyKinematicInput Input;
  for(int32 I=0;I<8;++I)
  {Probe->SetRelativeRotation(FRotator(21.125+I*3.137,-43.25-I*5.273,7.375+I*1.231));CaptureNativeSubBodyKinematicInput(*Probe,Input);}
  Probe->SetRelativeRotation(Target);
 };
 std::fesetround(FE_TONEAREST);
 FlushCanonicalCaptureCache();std::feclearexcept(FE_ALL_EXCEPT);VectorSetControlRegister(VectorGetControlRegister()&~uint32(0x3f));
 const auto CacheEntry=CaptureFrameFloatingPointContext();FSubBodyKinematicInput ColdInput,WarmInput,AfterShadowInput;
 if(!CaptureNativeSubBodyKinematicInput(*Probe,ColdInput))return false;
 const auto ColdExit=CaptureFrameFloatingPointContext();
 if(!ApplyFrameFloatingPointContext(CacheEntry)||!CaptureNativeSubBodyKinematicInput(*Probe,WarmInput))return false;
 const auto WarmExit=CaptureFrameFloatingPointContext();
 TestTrue(TEXT("cache regression actually distinguishes conversion miss from hit flags"),!SameJoinedActorNumericContext1213(ColdExit,WarmExit));
 FlushCanonicalCaptureCache();if(!ApplyFrameFloatingPointContext(CacheEntry))return false;
 FJoinedActorKinematics1213 CacheShadow;
 if(!TestTrue(TEXT("cache shadow admitted"),CacheShadow.CaptureAndCompute(Actors,2,1)))return false;
 if(!TestTrue(TEXT("cache shadow original callback executed"),CacheShadow.TryInstall(*C)))return false;
 if(!ApplyFrameFloatingPointContext(CacheEntry)||!CaptureNativeSubBodyKinematicInput(*Probe,AfterShadowInput))return false;
 TestTrue(TEXT("shadow leaves canonical capture TLS cold-miss and complete flags unchanged"),SameJoinedActorNumericContext1213(ColdExit,CaptureFrameFloatingPointContext()));
 TestTrue(TEXT("private shadow cache retains exact captured quaternion values"),NativeFPBits1220(ColdInput.RelativeRotation,AfterShadowInput.RelativeRotation));
 // The callback must preserve the original canonical choice, including the
 // pre-existing parallel child batch, rather than force the direct operation.
 for(int32 Requested:{2,4,12})
 {
  FlushCanonicalCaptureCache();if(!ApplyFrameFloatingPointContext(CacheEntry))return false;
  if(!TestTrue(TEXT("original native child batch admits fixture"),TryUpdateSubBodyKinematicsInParallel(*C,Requested,1)))return false;
  const auto BatchExit=CaptureFrameFloatingPointContext();TArray<SKinematic> BatchValues;
  for(auto* Child:C->GetSubBodies())BatchValues.Add(Child->GetKinematicState());
  FlushCanonicalCaptureCache();if(!ApplyFrameFloatingPointContext(CacheEntry))return false;
  FJoinedActorKinematics1213 CallbackShadow;
  if(!CallbackShadow.CaptureAndCompute(Actors,2,1))return false;
  int32 Calls=0;bool BatchAdmitted=false;
  if(!CallbackShadow.TryInstall(*C,[&]()
   {++Calls;BatchAdmitted=TryUpdateSubBodyKinematicsInParallel(*C,Requested,1);if(!BatchAdmitted)C->UpdateSubBodiesKinematics();}))return false;
  TestEqual(TEXT("exact canonical operation callback runs once"),Calls,1);
  TestTrue(TEXT("shadow preserves original native child-batch admission"),BatchAdmitted);
  TestTrue(TEXT("shadow preserves original batch complete FP exit"),SameJoinedActorNumericContext1213(BatchExit,CaptureFrameFloatingPointContext()));
  for(int32 I=0;I<BatchValues.Num();++I)TestTrue(TEXT("shadow preserves original batch output bits"),NativeFPKinematics1220(BatchValues[I],C->GetSubBodies()[I]->GetKinematicState()));
 }
 // No diagnostic quaternion/arithmetic is allowed with a live hardware trap.
 // Clear sticky SIMD flags before unmasking invalid; restore before assertions.
 const auto MaskedEntry=CaptureFrameFloatingPointContext();
 auto UnmaskedEntry=MaskedEntry;UnmaskedEntry.VectorControl=(UnmaskedEntry.VectorControl&~uint32(0x3f))&~uint32(0x80);
 if(!ApplyFrameFloatingPointContext(UnmaskedEntry))return false;
 FJoinedActorKinematics1213 TrapShadow;const bool TrapAdmitted=TrapShadow.CaptureAndCompute(Actors,2,1);
 const bool TrapRestored=ApplyFrameFloatingPointContext(MaskedEntry);
 TestTrue(TEXT("unmasked control restored before test arithmetic"),TrapRestored);
 TestFalse(TEXT("shadow refuses unmasked numeric exceptions before any diagnostic calculation"),TrapAdmitted);
 // RestoreOriginal was constructed before world/adapter cleanup, so its
 // destructor restores the original FP state AFTER teardown arithmetic too.
 if(!TestTrue(TEXT("fixture retains full-context restoration guard through teardown"),RestoreOriginal.IsApplied()))return false;
 AddInfo(TEXT("Shadow correctness only. Worker outputs are never installed; no actor preparation speedup is claimed. Full native fenv/MXCSR oracle covers four rounding modes, initially clean/dirty flags and a preceding callback changing rounding."));
 return !HasAnyErrors();
}
#endif
