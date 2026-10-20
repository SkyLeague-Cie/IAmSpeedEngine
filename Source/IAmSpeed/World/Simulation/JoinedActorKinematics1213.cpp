#include "JoinedActorKinematics1213.h"
#include "JoinedActorBaseWitness1534.h"
#include "FrameParallelExecutor.h"
#include "FrameParallelSettings.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "IAmSpeed/SubBodies/SSubBody.h"
#include "IAmSpeed/Base/ExactRotatorQuaternionCache.h"
#include "IAmSpeed/SubBodies/Solid/SolidSubBody.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SphereSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "IAmSpeed/SubBodies/Sensor/SensorSubBody.h"
#include "IAmSpeed/SubBodies/Sensor/BoxSensor.h"
#include "IAmSpeed/SubBodies/Sensor/SphereSensor.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTLS.h"
#include <exception>
#include <stdexcept>
#include "IAmSpeed/World/Simulation/FrameSpeculationSafety1213.h"
#if defined(_MSC_VER)
#include <float.h>
#endif
namespace
{
// Diagnostic capture MUST NOT call CaptureNativeSubBodyKinematicInput: that
// helper's function-local TLS cache is also used by the pre-existing canonical
// child batch. Warming it here could change a later hit/miss and FP flags.
// This cache is owned by one shadow capture/revalidation only, never by native.
bool CaptureShadowLocal1213(const USSubBody& Body,
 Speed::FrameParallel::FSubBodyKinematicInput& Out,Speed::FExactRotatorQuaternionCache& Scratch)
{
 const UClass* Type=Body.GetClass();
 if(Type!=USSubBody::StaticClass()&&Type!=USolidSubBody::StaticClass()
  &&Type!=UBoxSubBody::StaticClass()&&Type!=USphereSubBody::StaticClass()
  &&Type!=USWheelSubBody::StaticClass()&&Type!=USensorSubBody::StaticClass()
  &&Type!=UBoxSensor::StaticClass()&&Type!=USphereSensor::StaticClass())return false;
 Speed::FrameParallel::FSubBodyKinematicInput Captured;
 Captured.RelativeLocation=Body.GetRelativeLocation();
 Captured.RelativeRotation=Scratch.Get(Body.GetRelativeRotation());
 Captured.bWheel=Type==USWheelSubBody::StaticClass();
 if(Captured.bWheel)Captured.WheelSpringDisplacement=static_cast<const USWheelSubBody&>(Body).SpringDisplacement();
 Out=Captured;return true;
}
thread_local Speed::FrameParallel::FJoinedActorKinematicShadowTotals1213 GActorShadow1213;
thread_local Speed::FrameParallel::FJoinedActorKinematics1213* GJoinedBaseOwner1430=nullptr;
TAutoConsoleVariable<int32> CVarJoinedBaseConsume1430(TEXT("p.IAmSpeed.Parallel.ActorBaseConsume1430"),0,
 TEXT("Opt-in exact guarded reuse of completed native base kinematics. Default0; no callback, native local rotation-cache access, wheel correction or barrier moves."));
TAutoConsoleVariable<int32> CVarJoinedActors1213(TEXT("p.IAmSpeed.Parallel.ActorKinematics1213"),0,
 TEXT("Captured actor wave defaults to correctness shadow. Separate ActorBaseConsume1430 opt-in may reuse exact validated base arithmetic; original virtual operation, cache access and wheel correction remain on owner. Default0; no gain qualification implied."));
template<class T> bool JoinedBits1213(const T& A,const T& B){return FMemory::Memcmp(&A,&B,sizeof(T))==0;}
bool JoinedOrigin1213(const SKinematic& A,const SKinematic& B)
{
 return JoinedBits1213(A.Location,B.Location)&&JoinedBits1213(A.Rotation,B.Rotation)
  &&JoinedBits1213(A.Velocity,B.Velocity)&&JoinedBits1213(A.Acceleration,B.Acceleration)
  &&JoinedBits1213(A.AngularVelocity,B.AngularVelocity)&&JoinedBits1213(A.AngularAcceleration,B.AngularAcceleration);
}
bool JoinedLocal1213(const Speed::FrameParallel::FSubBodyKinematicInput& A,const Speed::FrameParallel::FSubBodyKinematicInput& B)
{
 return JoinedBits1213(A.RelativeLocation,B.RelativeLocation)&&JoinedBits1213(A.RelativeRotation,B.RelativeRotation)
  &&JoinedBits1213(A.WheelSpringDisplacement,B.WheelSpringDisplacement)&&A.bWheel==B.bWheel;
}
}
namespace Speed::FrameParallel
{

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
namespace {thread_local FJoinedActorBaseWitnessScope1534* GBaseWitness1534=nullptr;}
FJoinedActorBaseWitnessScope1534::FJoinedActorBaseWitnessScope1534(TArray<FJoinedActorBaseEvent1534>& Out,int32 MaximumEvents)
 :Events(Out),OwnerThread(FPlatformTLS::GetCurrentThreadId()),Limit(FMath::Clamp(MaximumEvents,1,4096))
{
 Previous=GBaseWitness1534;bUnavailable=Previous!=nullptr;Events.Reset();Events.Reserve(Limit);GBaseWitness1534=this;
}
FJoinedActorBaseWitnessScope1534::~FJoinedActorBaseWitnessScope1534()
{check(GBaseWitness1534==this);GBaseWitness1534=Previous;}
bool FJoinedActorBaseWitnessScope1534::IsActive(){return GBaseWitness1534!=nullptr;}
uint32 FJoinedActorBaseWitnessScope1534::BeginWave()
{
 auto* S=GBaseWitness1534;if(!S)return 0;
 if(S->OwnerThread!=FPlatformTLS::GetCurrentThreadId()||S->NextWave==MAX_uint32){S->bUnavailable=true;return 0;}
 return ++S->NextWave;
}
void FJoinedActorBaseWitnessScope1534::Record(uint32 Wave,int32 Item,const ISpeedComponent* Actor,const USSubBody* Body,uint32 Producer,bool Consumed)
{
 auto* S=GBaseWitness1534;if(!S)return;
 const auto Numeric=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext Guard(Numeric);
 if(!Guard.IsApplied())S->bUnavailable=true;
 else if(!Wave||Item<0||!Actor||!Body||!Producer||S->OwnerThread!=FPlatformTLS::GetCurrentThreadId()||S->Events.Num()>=S->Limit)S->bUnavailable=true;
 else
 {
  FJoinedActorBaseEvent1534 E;E.Wave=Wave;E.Item=Item;E.Actor=Actor;E.Body=Body;E.ProducerThread=Producer;
  E.ConsumerThread=Consumed?S->OwnerThread:0;E.bConsumed=Consumed;S->Events.Add(E);
 }
 if(!Guard.Restore()||!SameJoinedActorNumericContext1213(Numeric,CaptureFrameFloatingPointContext()))throw std::runtime_error("base1534 witness numeric restoration failed");
}
#endif

FJoinedActorKinematicShadowTotals1213 GetJoinedActorKinematicShadowTotals1213(){return GActorShadow1213;}
void ResetJoinedActorKinematicShadowTotals1213(){GActorShadow1213={};}
bool SameJoinedActorNumericContext1213(const FFrameFloatingPointContext& A,const FFrameFloatingPointContext& B)
{
 // Deliberately conservative on hosts whose opaque fenv contains additional
 // metadata: inability to compare is UNKNOWN, never permission to install.
 return A.bValid&&B.bValid&&A.VectorControl==B.VectorControl
  &&FMemory::Memcmp(&A.Environment,&B.Environment,sizeof(A.Environment))==0;
}
bool IsJoinedActorKinematicWaveEnabled1213(){return CVarJoinedActors1213.GetValueOnAnyThread()!=0;}
bool FJoinedActorKinematics1213::CaptureAndCompute(TConstArrayView<ISpeedComponent*> Actors,
 const int32 LaneLimit,const int32 MinParallelActors)
{
 Records.Reset();Items.Reset();Outputs.Reset();BeforeItem.Reset();AfterItem.Reset();Replayed=Rejected=Lanes=Participants=0;
 BaseOutputs1430.Reset();BaseBefore1430.Reset();BaseAfter1430.Reset();BaseProducerThreads1430.Reset();UsefulBaseThreads1430.Reset();BaseConsumed1430.Reset();
 #if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
 WitnessWave1534=0;
 #endif
 ActiveActor1430=nullptr;bHasBase1430=CVarJoinedBaseConsume1430.GetValueOnAnyThread()!=0;
 if(IsInsideFrameParallelTask()||LaneLimit<2||LaneLimit>FFrameParallelPolicy::MaximumLanes||Actors.Num()<2||Actors.Num()>256)return false;
 const auto Numeric=CaptureFrameFloatingPointContext();if(!Speed::AreFrameFloatingPointExceptionsMasked1213(Numeric))return false;
 FScopedFrameFloatingPointContext OwnerGuard(Numeric);
 const auto Return=[&](bool V){if(!OwnerGuard.Restore())throw std::runtime_error("actor shadow owner capture restoration failed");return V;};
 if(!OwnerGuard.IsApplied()){++GActorShadow1213.ShadowUnavailable;return Return(false);}
 Speed::FExactRotatorQuaternionCache CaptureScratch;
 for(ISpeedComponent* Actor:Actors)
 {
  if(!Actor||!Actor->SupportsJoinedActorKinematicCapture1213())continue;
  const auto& Bodies=Actor->GetSubBodies();if(Bodies.IsEmpty()||Bodies.Num()>256||Items.Num()+Bodies.Num()>1024)continue;
  bool Duplicate=false;for(const auto& R:Records)Duplicate|=R.Actor==Actor;
  if(Duplicate){Records.Reset();Items.Reset();return Return(false);}
  if(!ApplyFrameFloatingPointContext(Numeric)){++GActorShadow1213.ShadowUnavailable;Records.Reset();Items.Reset();return Return(false);}
  FActor R;R.Actor=Actor;R.Entry=CaptureFrameFloatingPointContext();R.Origin=Actor->GetOriginKinematicState();
  R.First=Items.Num();R.Count=Bodies.Num();R.Bodies=Bodies;bool Complete=true;
  for(USSubBody* Body:Bodies)
  {
   FItem I;I.Origin=R.Origin;
   if(!Body||!CaptureShadowLocal1213(*Body,I.Local,CaptureScratch)){Complete=false;break;}
   Items.Add(MoveTemp(I));
  }
  if(!Complete){Items.SetNum(R.First);continue;}
  R.AfterCapture=CaptureFrameFloatingPointContext();if(!R.AfterCapture.bValid){++GActorShadow1213.ShadowUnavailable;Records.Reset();Items.Reset();return Return(false);}
  Records.Add(MoveTemp(R));
 }
 Lanes=FFrameParallelPolicy::ResolveDispatchLanes(LaneLimit,Records.Num(),MinParallelActors);
 if(Records.Num()<2||Lanes<2){Records.Reset();Items.Reset();Lanes=0;return Return(false);}
 Outputs.SetNum(Items.Num());BeforeItem.SetNum(Items.Num());AfterItem.SetNum(Items.Num());
 if(bHasBase1430){BaseOutputs1430.SetNum(Items.Num());BaseBefore1430.SetNum(Items.Num());BaseAfter1430.SetNum(Items.Num());BaseProducerThreads1430.Init(0,Items.Num());BaseConsumed1430.Init(0,Items.Num());}
 UsefulBaseThreads1430.Reserve(FFrameParallelPolicy::MaximumLanes);
 const bool ComputeBase1430=bHasBase1430;
 const TArrayView<SKinematic> BaseResults1430(BaseOutputs1430);
 const TArrayView<FFrameFloatingPointContext> BaseEntry1430(BaseBefore1430),BaseExit1430(BaseAfter1430);
 const TArrayView<uint32> ProducerThreads1430(BaseProducerThreads1430);
 struct FRange{int32 First=0,Count=0;FFrameFloatingPointContext Entry;};
 TArray<FRange> Ranges;for(const auto& R:Records)Ranges.Add({R.First,R.Count,R.AfterCapture});
 const TConstArrayView<FItem> WorkItems(Items);const TArrayView<SKinematic> Results(Outputs);
 const TConstArrayView<FRange> WorkRanges(Ranges);const TArrayView<FFrameFloatingPointContext> ItemBefore(BeforeItem),ItemAfter(AfterItem);
 TArray<uint8> CompletedActors;CompletedActors.Init(0,Records.Num());
 const TArrayView<uint8> Completed(CompletedActors);
#if WITH_DEV_AUTOMATION_TESTS
 const int32 InjectedFailure=InjectedShadowFailure;
#else
 const int32 InjectedFailure=INDEX_NONE;
#endif
 const int32 WorkLanes=Lanes;uint32 Threads[FFrameParallelPolicy::MaximumLanes]={};std::exception_ptr Failures[FFrameParallelPolicy::MaximumLanes];
 const auto Work=[WorkItems,Results,WorkRanges,ItemBefore,ItemAfter,Completed,InjectedFailure,WorkLanes,ComputeBase1430,BaseResults1430,BaseEntry1430,BaseExit1430,ProducerThreads1430,&Threads,&Failures](int32 Lane)
 {
  Threads[Lane]=FPlatformTLS::GetCurrentThreadId();
  try
  {
   // Actor partitions preserve each actor's child order and FP dependency. These
   // contexts are observations only: owner execution is never skipped.
   for(int32 A=Lane;A<WorkRanges.Num();A+=WorkLanes)
   {
    const auto& R=WorkRanges[A];FScopedFrameParallelTask Scope(R.Entry);
    if(!Scope.IsNumericContextApplied())throw std::runtime_error("actor shadow worker numeric context unavailable");
    if(A==InjectedFailure)throw std::runtime_error("injected actor shadow failure");
    for(int32 J=0;J<R.Count;++J)
    {
     const int32 I=R.First+J;ItemBefore[I]=CaptureFrameFloatingPointContext();
     if(ComputeBase1430)
     {
      auto BaseLocal=WorkItems[I].Local;BaseLocal.bWheel=false;
      BaseEntry1430[I]=ItemBefore[I];
      BaseResults1430[I]=CalculateSubBodyKinematics(WorkItems[I].Origin,BaseLocal);
      BaseExit1430[I]=CaptureFrameFloatingPointContext();ProducerThreads1430[I]=Threads[Lane];
      // Existing full shadow remains an independent diagnostic with its original
      // FP entry. Only private worker state is restored here.
      if(!ApplyFrameFloatingPointContext(ItemBefore[I]))throw std::runtime_error("base1430 private numeric restore failed");
     }
     Results[I]=CalculateSubBodyKinematics(WorkItems[I].Origin,WorkItems[I].Local);ItemAfter[I]=CaptureFrameFloatingPointContext();
    }
    if(!Scope.RestoreNumericContext())throw std::runtime_error("actor shadow worker numeric restoration failed");
    Completed[A]=1;
   }
  }
  catch(...){Failures[Lane]=std::current_exception();}
 };
 if(!TryDispatchPrivateFrameParallelLanes1075(Lanes,Work))
 {Records.Reset();Items.Reset();Outputs.Reset();BeforeItem.Reset();AfterItem.Reset();Lanes=0;return Return(false);}
 for(int32 I=0;I<Lanes;++I){bool Seen=false;for(int32 J=0;J<I;++J)Seen|=Threads[J]==Threads[I];if(!Seen&&Threads[I])++Participants;}
 ++GActorShadow1213.Waves;GActorShadow1213.Items+=Items.Num();
 GActorShadow1213.MaximumShadowNativeParticipants=FMath::Max(GActorShadow1213.MaximumShadowNativeParticipants,Participants);
 GActorShadow1213.MaximumShadowLogicalLanes=FMath::Max(GActorShadow1213.MaximumShadowLogicalLanes,Lanes);
 // A shadow exception is evidence unavailability, never an earlier native
 // exception. All accepted lanes have joined; original owner work still runs.
 for(int32 I=0;I<Lanes;++I)if(Failures[I])++GActorShadow1213.WorkerFailures;
 for(int32 A=0;A<Records.Num();++A)Records[A].bShadowComplete=CompletedActors[A]!=0;
 if(bHasBase1430)for(int32 A=0;A<Records.Num();++A)if(Records[A].bShadowComplete)GActorShadow1213.ComputedBaseItems1430+=Records[A].Count;
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
 if(bHasBase1430&&FJoinedActorBaseWitnessScope1534::IsActive())
 {
  WitnessWave1534=FJoinedActorBaseWitnessScope1534::BeginWave();
  for(const auto& Actor:Records)if(Actor.bShadowComplete)for(int32 J=0;J<Actor.Count;++J)
  {
   const int32 I=Actor.First+J;
   FJoinedActorBaseWitnessScope1534::Record(WitnessWave1534,I,Actor.Actor,Actor.Bodies[J],BaseProducerThreads1430[I],false);
  }
 }
#endif
 return Return(true);
}
bool IsJoinedActorBaseOwnerScope1430(){return GJoinedBaseOwner1430!=nullptr;}
bool TryConsumeJoinedActorBase1430(const USSubBody& Body,const SKinematic& Parent,
 const FVector& Location,const FQuat& Rotation,SKinematic& Output)
{
 auto* Wave=GJoinedBaseOwner1430;
 if(!Wave||IsInsideFrameParallelTask()||!Wave->ActiveActor1430||!Wave->bHasBase1430)return false;
 const auto& Actor=*Wave->ActiveActor1430;int32 Index=INDEX_NONE;
 for(int32 J=0;J<Actor.Count;++J)if(Actor.Bodies[J]==&Body){Index=Actor.First+J;break;}
 if(Index==INDEX_NONE||Wave->BaseConsumed1430[Index])return false;
 if(Wave->BaseProducerThreads1430[Index]==0)return false;
 const auto& Input=Wave->Items[Index];
 if(!JoinedOrigin1213(Parent,Input.Origin)||!JoinedBits1213(Location,Input.Local.RelativeLocation)
  ||!JoinedBits1213(Rotation,Input.Local.RelativeRotation))
 {++GActorShadow1213.RejectedBaseInput1430;return false;}
 const auto Entry=CaptureFrameFloatingPointContext();
 if(!SameJoinedActorNumericContext1213(Entry,Wave->BaseBefore1430[Index]))
 {++GActorShadow1213.RejectedBaseNumeric1430;return false;}
 if(!ApplyFrameFloatingPointContext(Wave->BaseAfter1430[Index]))
 {
  if(!ApplyFrameFloatingPointContext(Entry))throw std::runtime_error("base1430 owner fallback numeric restore failed");
  ++GActorShadow1213.RejectedBaseNumeric1430;return false;
 }
 const auto& Result=Wave->BaseOutputs1430[Index];
 // Write precisely the original six fields. Native wheel correction still
 // executes after Super returns; no virtual owner operation is skipped.
 Output.Location=Result.Location;Output.Velocity=Result.Velocity;Output.Acceleration=Result.Acceleration;
 Output.Rotation=Result.Rotation;Output.AngularVelocity=Result.AngularVelocity;Output.AngularAcceleration=Result.AngularAcceleration;
 Wave->BaseConsumed1430[Index]=1;++GActorShadow1213.ConsumedBaseItems1430;
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
 if(Wave->WitnessWave1534)FJoinedActorBaseWitnessScope1534::Record(Wave->WitnessWave1534,Index,Actor.Actor,&Body,Wave->BaseProducerThreads1430[Index],true);
#endif
 Wave->UsefulBaseThreads1430.AddUnique(Wave->BaseProducerThreads1430[Index]);
 GActorShadow1213.MaximumUsefulBaseNativeParticipants1430=FMath::Max(GActorShadow1213.MaximumUsefulBaseNativeParticipants1430,Wave->UsefulBaseThreads1430.Num());
 return true;
}
bool FJoinedActorKinematics1213::TryInstall(ISpeedComponent& Actor)
{
 return TryInstall(Actor,[&Actor](){Actor.UpdateSubBodiesKinematics();});
}
bool FJoinedActorKinematics1213::TryInstall(ISpeedComponent& Actor,TFunctionRef<void()> OriginalOwnerOperation)
{
 FActor* Found=nullptr;for(auto& R:Records)if(R.Actor==&Actor){Found=&R;break;}
 if(!Found||Found->bConsumed)return false;Found->bConsumed=true;
 const auto ActualEntry=CaptureFrameFloatingPointContext();
 const bool EntryMatches=SameJoinedActorNumericContext1213(Found->Entry,ActualEntry);
 bool InputsMatch=false;
 if(ActualEntry.bValid)
 {
  Speed::FExactRotatorQuaternionCache ValidationScratch;
  FScopedFrameFloatingPointContext ReadGuard(ActualEntry);
  // A failed diagnostic apply cannot replace the native operation. Restore
  // before original work; restoration failure alone is infrastructure-fatal.
  if(!ReadGuard.IsApplied())InputsMatch=false;
  else
  try
  {
  InputsMatch=Actor.SupportsJoinedActorKinematicCapture1213()&&JoinedOrigin1213(Found->Origin,Actor.GetOriginKinematicState());
  const auto& Bodies=Actor.GetSubBodies();InputsMatch&=Bodies.Num()==Found->Count;
  if(InputsMatch)for(int32 I=0;I<Bodies.Num();++I)
  {
   FSubBodyKinematicInput Current;
   if(Bodies[I]!=Found->Bodies[I]||!Bodies[I]||!CaptureShadowLocal1213(*Bodies[I],Current,ValidationScratch)
    ||!JoinedLocal1213(Current,Items[Found->First+I].Local)){InputsMatch=false;break;}
  }
  }
  catch(...){InputsMatch=false;}
  if(!ReadGuard.Restore())throw std::runtime_error("actor shadow validation numeric restoration failed");
 }
 // The original virtual operation, local native cache and child order remain
 // at their canonical point. Opt-in1430 may publish only a completed pure-base
 // result/FP exit after exact call-site input and complete numeric-entry checks.
 // One synchronous owner scope; workers never inherit or read this TLS.
 // Exact-body checks occur AFTER the native local conversion/cache access.
 const bool AllowBase1430=bHasBase1430&&Found->bShadowComplete&&InputsMatch
  &&CVarJoinedBaseConsume1430.GetValueOnAnyThread()!=0&&!GJoinedBaseOwner1430&&!IsInsideFrameParallelTask();
 if(AllowBase1430)
 {
  ActiveActor1430=Found;GJoinedBaseOwner1430=this;
  try{OriginalOwnerOperation();}
  catch(...){GJoinedBaseOwner1430=nullptr;ActiveActor1430=nullptr;throw;}
  GJoinedBaseOwner1430=nullptr;ActiveActor1430=nullptr;
 }
 else OriginalOwnerOperation();
 const auto ActualExit=CaptureFrameFloatingPointContext();++Replayed;++GActorShadow1213.ActorsReplayed;
 if(!Found->bShadowComplete){++Rejected;++GActorShadow1213.ShadowUnavailable;}
 else if(!EntryMatches){++Rejected;++GActorShadow1213.EntryChanged;}
 else if(!InputsMatch){++Rejected;++GActorShadow1213.InputsChanged;}
 else
 {
  ++GActorShadow1213.ComparableEntries;
  const auto& Bodies=Actor.GetSubBodies();
  for(int32 I=0;I<Bodies.Num();++I)if(!JoinedOrigin1213(Outputs[Found->First+I],Bodies[I]->GetKinematicState()))++GActorShadow1213.StateDifferences;
  if(!SameJoinedActorNumericContext1213(AfterItem[Found->First+Found->Count-1],ActualExit))++GActorShadow1213.ExitDifferences;
 }
 return true; // Original computation already ran, including on a stale shadow.
}
}
