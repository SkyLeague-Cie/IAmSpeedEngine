#pragma once
#include "CoreMinimal.h"
#if !UE_BUILD_SHIPPING
namespace Speed
{
// Actual callbacks, captured in invocation order on the canonical owner.
// Complete() is meaningful only together with a committed native frame.
class IAMSPEED_API FFrameAppliedCommandWitnessScope
{
public:
 FFrameAppliedCommandWitnessScope(uint64 Frame,int32 ExpectedJournalCommands);
 ~FFrameAppliedCommandWitnessScope();
 static bool IsActive();
 static void ExpectAnalogActor(uint64 Actor) noexcept;
 static bool SealExpectedAnalogActors() noexcept;
 static void MarkCoverageUnknown() noexcept;
 static void RecordJournal(uint64 Frame,uint64 Actor,const TArray<uint8>& Payload,bool Accepted) noexcept;
 static void RecordAnalog(uint64 Actor,uint64 LocalFrame,const TArray<uint8>& Payload) noexcept;
 bool Complete()const;
 bool SaveCommittedFrame()const;
private:
 uint64 Frame=0;int32 ExpectedJournal=0,ObservedJournal=0;
 TArray<uint8> Raw;TSet<uint64> ExpectedAnalog,ObservedAnalog;
 bool bUnknown=false,bExpectedAnalogSealed=false;
 FFrameAppliedCommandWitnessScope* Previous=nullptr;
};
}
#endif
