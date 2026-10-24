#pragma once
#include "IAmSpeed/Controllers/SpeedController.h"
#include "DrivingMenuInput.h"
#include "SpeedDrivingSettings.h"
#include "SpeedDrivingController.generated.h"

namespace Speed::Input::V2 { class FDeviceInputWarmContext; }
class AHUD;

/** Optional stock standalone driving shell. Existing game controllers keep their policy. */
UCLASS(Blueprintable)
class IAMSPEED_API ASpeedDrivingController : public ASpeedController
{
    GENERATED_BODY()
public:
    void Tick(float DeltaSeconds) override;
    void EndPlay(EEndPlayReason::Type Reason) override;
    void SetupEnhancedInputComponent(UEnhancedInputComponent*) override {}
    /** Keyboard/gamepad navigation uses the V2 journal; HUD clicks use the same menu policy. */
    void DrawDrivingMenu(AHUD& HUD,const FVector2D& ViewportSize) const;
    /** UI-only event. Never publishes gameplay input or bypasses SetPause's V2 owner fences. */
    bool HandleDrivingMenuHitBoxClick(FName BoxName);
    std::shared_ptr<const Speed::Input::V2::FInputActionContract> GetDrivingContract() const;
protected:
    bool RequiresInputSessionV2() const override {return true;}
    void PrepareInputConfigurationV2() override;
    std::shared_ptr<Speed::Input::V2::FInputHostSession> CreateInputSessionV2(uint64 FirstFrame) override;
    Speed::Input::V2::EControlApplication ExecuteInputControlV2(const Speed::Input::V2::FControlRequest& Request) override;
    void OnPauseStateChanged(bool bPaused) override;
private:
#if WITH_DEV_AUTOMATION_TESTS
    friend class FDrivingMenuResume2836;
#endif
    enum class EPage : uint8 { Pause,Graphism,Camera,Commands };
    void Navigate(Speed::Driving::EMenuAction Action);
    void ChangePage(EPage Page);
    void Adjust(int32 Direction);
    void Capture(const Speed::Driving::FMenuInput::FCapture& Binding);
    bool SavePreferences();
    void ApplyVideoPreferences();
    bool QueueCameraSettings(int32 ChangedRow=-1);
    int32 RowCount() const;
    FName MenuHitBoxName(int32 Index) const;
    UPROPERTY(Transient) TObjectPtr<USpeedDrivingSettings> Preferences;
    std::shared_ptr<Speed::Input::V2::FDeviceInputWarmContext> DrivingWarmContext;
    std::shared_ptr<const Speed::Input::V2::FRawAcquisitionJournal> MenuJournal;
    std::unique_ptr<Speed::Driving::FMenuInput> MenuInput;
    EPage Page=EPage::Pause;
    int32 Row=0,ResolutionIndex=3,FpsIndex=4;
    uint64 MenuContext=1;
    bool bMenuVisible=false,bCapture=false,bGamepadBinding=false,bRemapPending=false,bCameraApplied=false,bResetPending=false;
    bool bOwnsMenuPointer=false,bPreviousMouseCursor=false,bPreviousClickEvents=false;
    FString Message;
};
