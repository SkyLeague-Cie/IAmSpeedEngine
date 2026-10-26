#pragma once

#include "CoreMinimal.h"

class APlayerController;
class ASpeedGameMode;

namespace Speed::Replay
{
// The game module supplies the recorder; the common GameMode owns its public
// entry point. The existing control-command lane admits the player's action.
// Neither calling SaveReplay nor attaching this interface creates input events,
// runs physics, performs disk IO or grants network authority.
class IGameModeReplayRecorder
{
public:
    virtual ~IGameModeReplayRecorder() = default;
    virtual bool SaveReplay(APlayerController& Player) = 0;
    virtual void StopRecording() = 0;
};

DECLARE_MULTICAST_DELEGATE_OneParam(FGameModeRecorderRequested, ASpeedGameMode&);
IAMSPEED_API FGameModeRecorderRequested& OnGameModeRecorderRequested();
DECLARE_MULTICAST_DELEGATE_OneParam(FGameModeReplayUnavailable, APlayerController&);
IAMSPEED_API FGameModeReplayUnavailable& OnGameModeReplayUnavailable();
}
