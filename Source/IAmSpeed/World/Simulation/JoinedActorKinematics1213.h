#pragma once
#include "CoreMinimal.h"
#include "SubBodyKinematicsTasks.h"
#include "FrameFloatingPointContext.h"
class ISpeedComponent;class USSubBody;
namespace Speed::FrameParallel
{
struct FJoinedActorKinematicShadowTotals1213
{
 uint64 Waves=0,Items=0,ActorsReplayed=0,ComparableEntries=0,EntryChanged=0,InputsChanged=0,StateDifferences=0,ExitDifferences=0,WorkerFailures=0,ShadowUnavailable=0;
 int32 MaximumShadowNativeParticipants=0,MaximumShadowLogicalLanes=0;
 uint64 ComputedBaseItems1430=0,ConsumedBaseItems1430=0,RejectedBaseInput1430=0,RejectedBaseNumeric1430=0;
 int32 MaximumUsefulBaseNativeParticipants1430=0;
};
IAMSPEED_API FJoinedActorKinematicShadowTotals1213 GetJoinedActorKinematicShadowTotals1213();
IAMSPEED_API void ResetJoinedActorKinematicShadowTotals1213();
/** The native virtual operation and its local rotation-cache access always run.
 * Default behavior remains shadow-only. Opt-in1430 may replace only completed
 * base arithmetic at its exact native call site, after exact value/FP checks.
 * Wheel correction, callbacks, reset and publication remain on their owner. */
IAMSPEED_API bool TryConsumeJoinedActorBase1430(const USSubBody& Body,
 const SKinematic& Parent,const FVector& Location,const FQuat& Rotation,SKinematic& Output);
IAMSPEED_API bool IsJoinedActorBaseOwnerScope1430();
class IAMSPEED_API FJoinedActorKinematics1213 final
{
public:
 bool CaptureAndCompute(TConstArrayView<ISpeedComponent*> Actors,int32 LaneLimit,int32 MinParallelActors);
 /** True only when this method executed Actor.UpdateSubBodiesKinematics at its
  * original point; caller must not execute it again. Base reuse requires separate1430 opt-in. */
    /** Standalone oracle explicitly invokes the direct native owner operation. */
    bool TryInstall(ISpeedComponent& Actor);
    /** Executes the caller's exact pre-existing canonical choice once, then observes. */
    bool TryInstall(ISpeedComponent& Actor,TFunctionRef<void()> OriginalOwnerOperation);
#if WITH_DEV_AUTOMATION_TESTS
 void InjectShadowFailureForTesting(int32 ActorIndex){InjectedShadowFailure=ActorIndex;}
#endif
 int32 ActorsCaptured() const{return Records.Num();}
 int32 ItemsComputed() const{return Items.Num();}
 int32 ActorsReplayed() const{return Replayed;}
 int32 ActorsRejected() const{return Rejected;}
 int32 LogicalLanes() const{return Lanes;}
 int32 NativeParticipants() const{return Participants;}
private:
 friend bool TryConsumeJoinedActorBase1430(const USSubBody&,const SKinematic&,const FVector&,const FQuat&,SKinematic&);
 struct FItem{SKinematic Origin;FSubBodyKinematicInput Local;};
 struct FActor
 {
  ISpeedComponent* Actor=nullptr;TArray<USSubBody*> Bodies;SKinematic Origin;
  FFrameFloatingPointContext Entry,AfterCapture;int32 First=0,Count=0;bool bConsumed=false,bShadowComplete=false;
 };
 TArray<FActor> Records;TArray<FItem> Items;TArray<SKinematic> Outputs;
 TArray<FFrameFloatingPointContext> BeforeItem,AfterItem;
 TArray<SKinematic> BaseOutputs1430;
 TArray<FFrameFloatingPointContext> BaseBefore1430,BaseAfter1430;
 TArray<uint32> BaseProducerThreads1430,UsefulBaseThreads1430;
 TArray<uint8> BaseConsumed1430;
 FActor* ActiveActor1430=nullptr;
 bool bHasBase1430=false;
 int32 Replayed=0,Rejected=0,Lanes=0,Participants=0;
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
 uint32 WitnessWave1534=0;
#endif
#if WITH_DEV_AUTOMATION_TESTS
 int32 InjectedShadowFailure=INDEX_NONE;
#endif
};
IAMSPEED_API bool IsJoinedActorKinematicWaveEnabled1213();
IAMSPEED_API bool SameJoinedActorNumericContext1213(const FFrameFloatingPointContext& A,const FFrameFloatingPointContext& B);
}
