#pragma once
#include "CoreMinimal.h"
#include "GameFramework/SaveGame.h"
#include "DrivingInput.h"
#include "SpeedDrivingSettings.generated.h"

USTRUCT()
struct IAMSPEED_API FSpeedDrivingBinding
{
    GENERATED_BODY()
    UPROPERTY(SaveGame) int32 Kind=0;
    UPROPERTY(SaveGame) int32 Code=0;
    UPROPERTY(SaveGame) float Direction=1;
};

/** Per-project user preferences. Nothing writes a vehicle Blueprint or game defaults. */
UCLASS()
class IAMSPEED_API USpeedDrivingSettings : public USaveGame
{
    GENERATED_BODY()
public:
    USpeedDrivingSettings();
    UPROPERTY(SaveGame) int32 Schema=1;
    UPROPERTY(SaveGame) TArray<FSpeedDrivingBinding> Keyboard;
    UPROPERTY(SaveGame) TArray<FSpeedDrivingBinding> Gamepad;
    UPROPERTY(SaveGame) int64 MappingRevision=5;
    UPROPERTY(SaveGame) float CameraDistance=650;
    UPROPERTY(SaveGame) float CameraHeight=150;
    UPROPERTY(SaveGame) float CameraFov=110;
    UPROPERTY(SaveGame) float CameraAngle=-2;
    UPROPERTY(SaveGame) float CameraStiffness=.55f;
    UPROPERTY(SaveGame) float CameraSwivelSpeed=2.9f;
    UPROPERTY(SaveGame) bool bInvertCameraPitch=true;
    UPROPERTY(SaveGame) int32 WindowWidth=1920;
    UPROPERTY(SaveGame) int32 WindowHeight=1080;
    UPROPERTY(SaveGame) int32 MaximumFps=120;
    bool ReadBindings(Speed::Driving::FBindings& Out) const;
    bool IsValid() const;
};
