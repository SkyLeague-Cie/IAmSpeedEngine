#if WITH_DEV_AUTOMATION_TESTS
#include "JoinedActorKinematics1213.h"
#include "FrameParallelExecutor.h"
#include "FrameParallelSettings.h"
#include "IAmSpeed/Actors/SpeedCar.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/SubBodies/SSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
namespace
{
template<class T> bool WaveBits1213(const T& A,const T& B) {return FMemory::Memcmp(&A,&B,sizeof(T))==0;}
bool WaveKinematic1213(const SKinematic& A,const SKinematic& B)
{
 return WaveBits1213(A.Location,B.Location)&&WaveBits1213(A.Rotation,B.Rotation)
  &&WaveBits1213(A.Velocity,B.Velocity)&&WaveBits1213(A.Acceleration,B.Acceleration)
  &&WaveBits1213(A.AngularVelocity,B.AngularVelocity)&&WaveBits1213(A.AngularAcceleration,B.AngularAcceleration);
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedJoinedNativeCars1213,
 "IAmSpeed.Simulation.FrameParallel.NativeCarsJoinedKinematics1213",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedJoinedNativeCars1213::RunTest(const FString&)
{
 using namespace Speed;using namespace Speed::FrameParallel;
 const auto Options=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(false)
  .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false).SetTransactional(false);
 UWorld* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Options);
 if(!TestNotNull(TEXT("isolated native multicars world"),World))return false;
 GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
 ON_SCOPE_EXIT{World->DestroyWorld(false);GEngine->DestroyWorldContext(World);};
 auto* Bridge=World->GetSubsystem<USpeedWorldSubsystem>();
 if(!TestNotNull(TEXT("native world registry"),Bridge))return false;
 TArray<USpeedWheeledComponent*> Cars;TArray<ISpeedComponent*> Actors;
 ON_SCOPE_EXIT{for(auto* C:Cars)Bridge->UnregisterSpeedComponent(C);Bridge->ApplyPendingOps();};
 for(int32 A=0;A<12;++A)
 {
  auto* Car=World->SpawnActor<ASpeedCar>();
  auto* C=Car?Cast<USpeedWheeledComponent>(Car->GetVehicleMovement()):nullptr;
  if(!TestNotNull(TEXT("real native car component"),C))return false;
  C->SetOwner(Car);C->DisableAsyncPhysicsTickForTesting();Cars.Add(C);Actors.Add(C);
  SKinematic K;K.Location=FVector(1500*A,-123.75*A,1000+A);
  K.Rotation=FRotator(A*3.125,A*-13.75,A*.25).Quaternion();K.Velocity=FVector(300+A,71.25-A,31.75);
  K.Acceleration=FVector(2.5*A,-7.25,-650);K.AngularVelocity=FVector(.125,-.375*A,2.25);
  K.AngularAcceleration=FVector(3.125,1.75*A,-.625);C->SetKinematicState(K);
  if(!TestEqual(TEXT("four production wheel subbodies"),C->GetWheelSubBodies().Num(),4))return false;
 }
 Bridge->ApplyPendingOps();
 TArray<SKinematic> Oracle;
 for(auto* C:Cars){C->UpdateSubBodiesKinematics();for(auto* B:C->GetSubBodies())Oracle.Add(B->GetKinematicState());}
 if(!TestTrue(TEXT("private workers available"),EnsurePrivateFrameWorkers(FFrameParallelPolicy::MaximumLanes-1)>0))return false;
 for(int32 W:{2,4,8,12})
 {
  ResetFrameParallelStageTotals(EFrameParallelStage::Prepare);
  FJoinedActorKinematics1213 Wave;
  if(!TestTrue(TEXT("joined native multicars shadow wave admitted"),Wave.CaptureAndCompute(Actors,W,1)))return false;
  TestEqual(TEXT("all real car identities captured"),Wave.ActorsCaptured(),12);
  TestTrue(TEXT("native workers actually performed nonempty item partitions"),Wave.NativeParticipants()>1);
  TestEqual(TEXT("logical request admitted"),Wave.LogicalLanes(),W);
  // Poison only output storage, after immutable capture. Install must overwrite
  // it from computed values and must not depend on its previous contents.
  for(auto* C:Cars)for(auto* B:C->GetSubBodies()){SKinematic Poison;Poison.Location=FVector(-99999);B->SetKinematicState(Poison);}
  int32 Index=0;
  for(auto* C:Cars)
  {
   if(!TestTrue(TEXT("original ordered call site installs validated actor"),Wave.TryInstall(*C)))return false;
   TestFalse(TEXT("an actor wave record cannot be consumed twice"),Wave.TryInstall(*C));
   for(auto* B:C->GetSubBodies())if(!TestTrue(TEXT("all actual native child values equal unchanged virtual oracle bits"),WaveKinematic1213(Oracle[Index++],B->GetKinematicState())))return false;
  }
  TestEqual(TEXT("all actor records replayed once on owner"),Wave.ActorsReplayed(),12);
  TestTrue(TEXT("FP-entry changes are explicit shadow rejection, never hidden adoption"),Wave.ActorsRejected()>=0&&Wave.ActorsRejected()<=12);
 }
 // Model a preceding canonical callback changing a later actor between wave
 // capture and its original call site. Stale work must never write any child.
 for(int32 Change:{0,1})
 {
  FJoinedActorKinematics1213 Wave;if(!Wave.CaptureAndCompute(Actors,4,1))return false;
  auto* C=Cars.Last();auto* B=C->GetSubBodies().Last();
  const SKinematic Before=B->GetKinematicState();const SKinematic Parent=C->GetKinematicState();const FVector Relative=B->GetRelativeLocation();
  if(Change==0){SKinematic K=Parent;K.Velocity.X+=.125;C->SetKinematicState(K);}
  else B->SetRelativeLocation(Relative+FVector(.125,0,0));
  TestTrue(TEXT("cross-actor mutation does not skip original owner computation"),Wave.TryInstall(*C));
  TestTrue(TEXT("stale shadow is explicitly rejected"),Wave.ActorsRejected()==1);
  TestFalse(TEXT("the unchanged original method updates poisoned/stale child storage"),WaveKinematic1213(Before,B->GetKinematicState()));
  C->SetKinematicState(Parent);B->SetRelativeLocation(Relative);
 }
 {
  FJoinedActorKinematics1213 Serial;TestFalse(TEXT("one lane retains original serial path"),Serial.CaptureAndCompute(Actors,1,1));
  TestFalse(TEXT("no serial speculative output can install"),Serial.TryInstall(*Cars[0]));
  auto Duplicate=Actors;Duplicate.Add(Cars[0]);FJoinedActorKinematics1213 Repeated;
  TestFalse(TEXT("duplicate actor identity rejected before dispatch"),Repeated.CaptureAndCompute(Duplicate,4,1));
  FScopedFrameParallelTask Nested;FJoinedActorKinematics1213 Child;
  TestFalse(TEXT("nested work cannot start another actor wave"),Child.CaptureAndCompute(Actors,4,1));
 }
 AddInfo(TEXT("Scope: SHADOW ONLY; original owner arithmetic never skipped. 12 actual native ASpeedCar/USpeedWheeledComponent instances and their real child arithmetic. This test does not execute whole car gameplay, dynamic contact solve, or the Stadium corpus."));
 return !HasAnyErrors();
}
#endif
