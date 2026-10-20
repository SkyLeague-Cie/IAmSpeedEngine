#pragma once
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "CoreMinimal.h"
class ISpeedComponent;class USSubBody;
namespace Speed::FrameParallel
{
// Values only. Computed rows are emitted on the canonical owner after the
// accepted private group joined; consumed rows follow the actual six writes.
// A request, a shadow calculation or an unconsumed result is never useful work.
struct FJoinedActorBaseEvent1534
{
 uint32 Wave=0,ProducerThread=0,ConsumerThread=0;int32 Item=INDEX_NONE;
 const ISpeedComponent* Actor=nullptr;const USSubBody* Body=nullptr;bool bConsumed=false;
};
class IAMSPEED_API FJoinedActorBaseWitnessScope1534 final
{
public:
 explicit FJoinedActorBaseWitnessScope1534(TArray<FJoinedActorBaseEvent1534>& Out,int32 MaximumEvents=2048);
 ~FJoinedActorBaseWitnessScope1534();
 FJoinedActorBaseWitnessScope1534(const FJoinedActorBaseWitnessScope1534&)=delete;
 FJoinedActorBaseWitnessScope1534& operator=(const FJoinedActorBaseWitnessScope1534&)=delete;
 bool IsComplete()const{return !bUnavailable;}
 static bool IsActive();
 static uint32 BeginWave();
 static void Record(uint32 Wave,int32 Item,const ISpeedComponent* Actor,const USSubBody* Body,uint32 Producer,bool Consumed);
private:
 TArray<FJoinedActorBaseEvent1534>& Events;FJoinedActorBaseWitnessScope1534* Previous=nullptr;
 uint32 OwnerThread=0,NextWave=0;int32 Limit=0;bool bUnavailable=false;
};
}
#endif
