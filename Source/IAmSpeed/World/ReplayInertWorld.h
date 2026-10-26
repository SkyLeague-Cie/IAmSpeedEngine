#pragma once
#include "CoreMinimal.h"
class UWorld;class UGameInstance;class AActor;class AGameModeBase;
namespace Speed::ReplayIsolation
{
// Native GT authority only. Arm before OpenLevel; the caller already owns and
// admits the source/session and the real destination package. No recorded class
// path is loaded by this service. SourceIdentity is an audit identity, not a loader.
IAMSPEED_API bool ArmInertReplayDestination(UGameInstance& Owner,UWorld& SourceWorld,
    FName DestinationPackage,uint64 RequestSerial,const FString& SourceIdentity,
    const FString& ModeClassPath,FString& Why);
IAMSPEED_API void RetireInertReplayDestination(UGameInstance& Owner,uint64 RequestSerial);
// Early pending recognition uses GI/package/new-world identity, never GameMode
// or URL (Engine assigns those later). Bound/failed worlds remain inert after
// retirement until actual world destruction. This API and all callers are GT.
IAMSPEED_API bool IsInertReplayWorld(const UWorld* World);
IAMSPEED_API bool IsInertBootstrapReady(const UWorld* World);
IAMSPEED_API bool HasInertBootstrapFailed(const UWorld* World,FString& Why);
IAMSPEED_API void FailInertReplayBootstrap(UWorld& World,const FString& Why);
IAMSPEED_API bool ValidateInertReplayBootstrap(UWorld& World,AGameModeBase& Mode,FString& Why);
IAMSPEED_API bool CompleteInertReplayBootstrap(UWorld& World,AGameModeBase& Mode,FString& Why);
}
