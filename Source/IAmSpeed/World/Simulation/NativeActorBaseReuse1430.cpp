#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "JoinedActorKinematics1213.h"
#include "FrameParallelExecutor.h"
#include "FrameParallelSettings.h"
#include "FrameSpeculationSafety1213.h"
#include "IAmSpeed/Actors/SpeedCar.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/SubBodies/SSubBody.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "HAL/IConsoleManager.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include <cfenv>

namespace
{
template<class T> bool BaseBits1430(const T& A,const T& B)
{return FMemory::Memcmp(&A,&B,sizeof(T))==0;}
bool BaseState1430(const SKinematic& A,const SKinematic& B)
{
 return BaseBits1430(A.Location,B.Location)&&BaseBits1430(A.Rotation,B.Rotation)
  &&BaseBits1430(A.Velocity,B.Velocity)&&BaseBits1430(A.Acceleration,B.Acceleration)
  &&BaseBits1430(A.AngularVelocity,B.AngularVelocity)&&BaseBits1430(A.AngularAcceleration,B.AngularAcceleration);
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedNativeActorBaseReuse1430,
 "IAmSpeed.Simulation.FrameParallel.NativeActorBaseReuse1430",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedNativeActorBaseReuse1430::RunTest(const FString&)
{
 using namespace Speed;using namespace Speed::FrameParallel;
 const auto Original=CaptureFrameFloatingPointContext();
 if(!TestTrue(TEXT("complete owner numeric context available"),Original.bValid))return false;
 FScopedFrameFloatingPointContext RestoreOriginal(Original);
 if(!TestTrue(TEXT("owner context restore is supported"),RestoreOriginal.IsApplied())
  ||!TestTrue(TEXT("speculative native oracle has masked FP exceptions"),AreCurrentFrameFloatingPointExceptionsMasked1213()))return false;
 auto* Consume=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.Parallel.ActorBaseConsume1430"));
 if(!TestNotNull(TEXT("explicit opt-in exists"),Consume))return false;
 const int32 OldConsume=Consume->GetInt();ON_SCOPE_EXIT{Consume->Set(OldConsume,ECVF_SetByCode);};
 const auto Options=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(false)
  .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false).SetTransactional(false);
 UWorld* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Options);
 if(!TestNotNull(TEXT("actual native multicars world"),World))return false;
 GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
 ON_SCOPE_EXIT{World->DestroyWorld(false);GEngine->DestroyWorldContext(World);};
 auto* Bridge=World->GetSubsystem<USpeedWorldSubsystem>();
 if(!TestNotNull(TEXT("actual registry"),Bridge))return false;
 TArray<USpeedWheeledComponent*> Cars;TArray<ISpeedComponent*> Actors;
 ON_SCOPE_EXIT{for(auto* C:Cars)Bridge->UnregisterSpeedComponent(C);Bridge->ApplyPendingOps();};
 for(int32 A=0;A<12;++A)
 {
  auto* Car=World->SpawnActor<ASpeedCar>();auto* C=Car?Cast<USpeedWheeledComponent>(Car->GetVehicleMovement()):nullptr;
  if(!TestNotNull(TEXT("actual native car"),C))return false;
  C->SetOwner(Car);C->DisableAsyncPhysicsTickForTesting();Cars.Add(C);Actors.Add(C);
  SKinematic K;K.Location=FVector(1500*A,-123.75*A,1000+A);
  K.Rotation=FRotator(A*3.125,A*-13.75,A*.25).Quaternion();K.Velocity=FVector(300+A,71.25-A,31.75);
  K.Acceleration=FVector(2.5*A,-7.25,-650);K.AngularVelocity=FVector(.125,-.375*A,2.25);
  K.AngularAcceleration=FVector(3.125,1.75*A,-.625);C->SetKinematicState(K);
 }
 Bridge->ApplyPendingOps();
 if(!TestTrue(TEXT("private native workers available"),EnsurePrivateFrameWorkers(FFrameParallelPolicy::MaximumLanes-1)>0))return false;
 // Warm the ORIGINAL native cache before the oracle; worker capture has its
 // separate scratch. Controls and all sticky flags remain part of the oracle.
 Consume->Set(0,ECVF_SetByCode);for(auto* C:Cars)C->UpdateSubBodiesKinematics();
 for(int32 Rounding1430:{FE_TONEAREST,FE_DOWNWARD,FE_UPWARD,FE_TOWARDZERO})for(bool Dirty1430:{false,true})for(int32 W:{2,4,8,12})
 {
  if(std::fesetround(Rounding1430)!=0)return false;
  std::feclearexcept(FE_ALL_EXCEPT);
  if(Dirty1430)std::feraiseexcept(FE_DIVBYZERO|FE_INEXACT);
  const auto Entry=CaptureFrameFloatingPointContext();
  TArray<SKinematic> Expected;TArray<FFrameFloatingPointContext> ExpectedExit;
  Consume->Set(0,ECVF_SetByCode);
  for(auto* C:Cars)
  {
   C->UpdateSubBodiesKinematics();ExpectedExit.Add(CaptureFrameFloatingPointContext());
   for(auto* B:C->GetSubBodies())Expected.Add(B->GetKinematicState());
  }
  if(!ApplyFrameFloatingPointContext(Entry))return false;
  Consume->Set(1,ECVF_SetByCode);ResetJoinedActorKinematicShadowTotals1213();
  FJoinedActorKinematics1213 Wave;
  if(!TestTrue(TEXT("real worker wave admitted"),Wave.CaptureAndCompute(Actors,W,1)))return false;
  int32 BodyIndex=0,ActorIndex=0;
  for(auto* C:Cars)
  {
   uint32 Calls=0;
   if(!TestTrue(TEXT("original native virtual operation executes"),Wave.TryInstall(*C,[&]{++Calls;C->UpdateSubBodiesKinematics();})))return false;
   const auto Exit=CaptureFrameFloatingPointContext();
   if(!TestEqual(TEXT("canonical virtual operation executes exactly once"),Calls,uint32(1))
    ||!TestTrue(TEXT("exact owner FP exit including sticky flags"),SameJoinedActorNumericContext1213(Exit,ExpectedExit[ActorIndex++])))return false;
   for(auto* B:C->GetSubBodies())if(!TestTrue(TEXT("every actual base and wheel result equals original native bits"),BaseState1430(Expected[BodyIndex++],B->GetKinematicState())))return false;
   TestFalse(TEXT("record cannot be replayed"),Wave.TryInstall(*C));
  }
  const auto Stats=GetJoinedActorKinematicShadowTotals1213();
  if(Rounding1430==FE_TONEAREST&&(!TestTrue(TEXT("completed arithmetic was actually consumed, not only shadowed"),Stats.ConsumedBaseItems1430>0)
   ||!TestTrue(TEXT("consumed results came from multiple actual native producer threads"),Stats.MaximumUsefulBaseNativeParticipants1430>1)))return false;
  AddInfo(FString::Printf(TEXT("[NativeActorBaseReuse1430] Requested=%d Computed=%llu Consumed=%llu ConsumedNativeMax=%d RejectedNumeric=%llu RejectedInput=%llu Rounding=%d Dirty=%d"),W,Stats.ComputedBaseItems1430,Stats.ConsumedBaseItems1430,Stats.MaximumUsefulBaseNativeParticipants1430,Stats.RejectedBaseNumeric1430,Stats.RejectedBaseInput1430,Rounding1430,Dirty1430?1:0));
 }
 // A preceding canonical callback changes a later actor input. Keep its
 // original computation and exact exit, and consume no captured stale base.
 {
  auto* Stale=Cars.Last();const auto ParentBefore=Stale->GetKinematicState();
  Consume->Set(1,ECVF_SetByCode);FJoinedActorKinematics1213 StaleWave;
  if(!StaleWave.CaptureAndCompute(Actors,4,1))return false;
  auto ChangedParent=ParentBefore;ChangedParent.Velocity.X+=.125;Stale->SetKinematicState(ChangedParent);
  const auto Entry=CaptureFrameFloatingPointContext();Consume->Set(0,ECVF_SetByCode);
  Stale->UpdateSubBodiesKinematics();const auto ExpectedExit=CaptureFrameFloatingPointContext();
  TArray<SKinematic> Expected;for(auto* B:Stale->GetSubBodies())Expected.Add(B->GetKinematicState());
  if(!ApplyFrameFloatingPointContext(Entry))return false;
  Consume->Set(1,ECVF_SetByCode);ResetJoinedActorKinematicShadowTotals1213();
  if(!StaleWave.TryInstall(*Stale))return false;const auto ActualExit=CaptureFrameFloatingPointContext();
  if(!TestTrue(TEXT("changed actor input retains original complete FP exit"),SameJoinedActorNumericContext1213(ExpectedExit,ActualExit)))return false;
  for(int32 J=0;J<Expected.Num();++J)if(!TestTrue(TEXT("changed actor input retains original native body bits"),BaseState1430(Expected[J],Stale->GetSubBodies()[J]->GetKinematicState())))return false;
  TestEqual(TEXT("no stale actor arithmetic is consumed"),GetJoinedActorKinematicShadowTotals1213().ConsumedBaseItems1430,uint64(0));
  TestTrue(TEXT("changed actor input is explicit refusal"),GetJoinedActorKinematicShadowTotals1213().InputsChanged+GetJoinedActorKinematicShadowTotals1213().EntryChanged>0);
  Stale->SetKinematicState(ParentBefore);
 }
 // A preceding callback changes the FP mode after actor-level validation.
 // Every exact-body reuse must refuse the changed numeric entry and preserve
 // the original native values and complete FP exit in the changed mode.
 auto* C=Cars[0];Consume->Set(0,ECVF_SetByCode);
 const int32 ChangedRounding=std::fegetround()==FE_DOWNWARD?FE_UPWARD:FE_DOWNWARD;
 if(std::fesetround(ChangedRounding)!=0)return false;
 const auto Changed=CaptureFrameFloatingPointContext();C->UpdateSubBodiesKinematics();const auto ChangedExit=CaptureFrameFloatingPointContext();
 TArray<SKinematic> ChangedExpected;for(auto* B:C->GetSubBodies())ChangedExpected.Add(B->GetKinematicState());
 if(!ApplyFrameFloatingPointContext(Original))return false;
 Consume->Set(1,ECVF_SetByCode);ResetJoinedActorKinematicShadowTotals1213();FJoinedActorKinematics1213 ChangedWave;
 if(!ChangedWave.CaptureAndCompute(Actors,4,1))return false;
 if(!ChangedWave.TryInstall(*C,[&]{if(!ApplyFrameFloatingPointContext(Changed))return;C->UpdateSubBodiesKinematics();}))return false;
 const auto ActualChangedExit=CaptureFrameFloatingPointContext();
 if(!TestTrue(TEXT("changed callback mode retains original full exit"),SameJoinedActorNumericContext1213(ChangedExit,ActualChangedExit)))return false;
 for(int32 J=0;J<C->GetSubBodies().Num();++J)if(!TestTrue(TEXT("changed FP callback keeps original native results"),BaseState1430(ChangedExpected[J],C->GetSubBodies()[J]->GetKinematicState())))return false;
 TestEqual(TEXT("changed FP context consumes no stale arithmetic"),GetJoinedActorKinematicShadowTotals1213().ConsumedBaseItems1430,uint64(0));
 TestTrue(TEXT("changed callback input or numeric mismatch is explicit"),GetJoinedActorKinematicShadowTotals1213().RejectedBaseNumeric1430+GetJoinedActorKinematicShadowTotals1213().RejectedBaseInput1430>0);
 TestFalse(TEXT("no owner scope leaks after callback"),IsJoinedActorBaseOwnerScope1430());
 // A failed speculative lane joins normally and cannot move its exception
 // into the canonical owner or hide the original virtual operation.
 if(!ApplyFrameFloatingPointContext(Original))return false;
 Consume->Set(0,ECVF_SetByCode);const auto FailureEntry=CaptureFrameFloatingPointContext();
 C->UpdateSubBodiesKinematics();const auto FailureExit=CaptureFrameFloatingPointContext();
 TArray<SKinematic> FailureExpected;for(auto* B:C->GetSubBodies())FailureExpected.Add(B->GetKinematicState());
 if(!ApplyFrameFloatingPointContext(FailureEntry))return false;
 Consume->Set(1,ECVF_SetByCode);ResetJoinedActorKinematicShadowTotals1213();
 FJoinedActorKinematics1213 FailedWave;FailedWave.InjectShadowFailureForTesting(0);
 if(!FailedWave.CaptureAndCompute(Actors,4,1))return false;
 uint32 FailureCalls=0;if(!FailedWave.TryInstall(*C,[&]{++FailureCalls;C->UpdateSubBodiesKinematics();}))return false;
 const auto ActualFailureExit=CaptureFrameFloatingPointContext();
 if(!TestEqual(TEXT("failed speculation retains one native virtual operation"),FailureCalls,uint32(1))
  ||!TestTrue(TEXT("failed speculation preserves complete original FP exit"),SameJoinedActorNumericContext1213(FailureExit,ActualFailureExit)))return false;
 for(int32 J=0;J<FailureExpected.Num();++J)if(!TestTrue(TEXT("failed speculation preserves original native results"),BaseState1430(FailureExpected[J],C->GetSubBodies()[J]->GetKinematicState())))return false;
 TestEqual(TEXT("failed producer output never consumed"),GetJoinedActorKinematicShadowTotals1213().ConsumedBaseItems1430,uint64(0));
 TestTrue(TEXT("actual injected worker failure remains visible"),GetJoinedActorKinematicShadowTotals1213().WorkerFailures>0);
 TestFalse(TEXT("failure leaves no owner scope"),IsJoinedActorBaseOwnerScope1430());
 AddInfo(TEXT("Scope: actual consumed base arithmetic on real native cars at2/4/8/12 producer lanes, original virtual/cache/wheel paths and exact FP exit retained. This is a bounded arithmetic fixture; whole-frame Normal/Fast commands, contacts, causal states and Stadium qualification remain separate."));
 return !HasAnyErrors();
}
#endif
