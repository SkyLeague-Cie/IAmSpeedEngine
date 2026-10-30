#pragma once
#include "CoreMinimal.h"
#include "PresentationClientGate.h"
class UWorld;class UNetDriver;class UNetConnection;
namespace Speed::Netcode::PresentationClient
{
// Immutable process opt-in -SpeedExactRemotePresentationClient. All APIs GT,
// except suppression also denies off-thread bootstrap without touching UObject state.
// No opt-in preserves Legacy. Pending/Admitted/Failed ALL suppress local physics.
IAMSPEED_API bool IsSelected();
IAMSPEED_API bool SuppressesLocalPhysics(const UWorld* World);
IAMSPEED_API EState Query(UWorld& World,uint64& Generation,FString& Why);
// Caller authenticates Trusted from the original human connection/control plane.
// This verifies actual UE identities, whole canonical tuple equality and a pristine
// world audit. It creates no device, physical consumer, worker or presentation data.
IAMSPEED_API bool Admit(UWorld& World,uint64 Generation,UNetDriver& OriginalDriver,
    UNetConnection& OriginalConnection,const FTuple& Trusted,const FTuple& Received,FString& Why);
IAMSPEED_API bool Matches(UWorld& World,uint64 Generation,const FTuple& Tuple,FString& Why);
IAMSPEED_API void Fail(UWorld& World,const FString& Why);
// Re-audit after streaming/BP spawn and before presentation publication. Static
// query/render geometry allowed; arbitrary simulating Chaos bodies are refused,
// never silently disabled. Host still owns early non-IAS Blueprint spawn hooks.
IAMSPEED_API bool Audit(UWorld& World,FString& Why);
}
