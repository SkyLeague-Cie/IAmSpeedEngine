#include "SpeedDrivingController.h"
#include "IAmSpeed/Input/DeviceInputHost.h"
#include "IAmSpeed/Actors/SpeedCar.h"
#include "IAmSpeed/Camera/SpeedArmComponent.h"
#include "GameFramework/GameUserSettings.h"
#include "GameFramework/HUD.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Kismet/GameplayStatics.h"

namespace
{
const FIntPoint Resolutions[]={{1024,576},{1280,720},{1600,900},{1920,1080},{2560,1440},{3840,2160}};
constexpr int32 FrameRates[]={30,60,90,100,120,144,165,240,360,0};
const TCHAR* Commands[]={TEXT("Throttle"),TEXT("Brake"),TEXT("Turn left"),TEXT("Turn right"),TEXT("Reset game"),TEXT("Camera left"),TEXT("Camera right"),TEXT("Camera up"),TEXT("Camera down")};
FString BindingName(const FSpeedDrivingBinding& B)
{
    using namespace Speed::Input::V2;
    if(B.Kind==int32(ERawControlKind::PadAxis))
    {
        const TCHAR* Names[]={TEXT("Left stick X"),TEXT("Left stick Y"),TEXT("Right stick X"),TEXT("Right stick Y"),TEXT("LT"),TEXT("RT")};
        return FString::Printf(TEXT("%s %s"),B.Code>=0&&B.Code<6?Names[B.Code]:TEXT("Axis"),B.Direction<0?TEXT("-"):TEXT("+"));
    }
    if(B.Kind==int32(ERawControlKind::PadButton))
    {
        const TCHAR* Names[]={TEXT("A / Cross"),TEXT("B / Circle"),TEXT("X / Square"),TEXT("Y / Triangle"),TEXT("D-pad up"),TEXT("D-pad down"),TEXT("D-pad left"),TEXT("D-pad right"),TEXT("LB"),TEXT("RB"),TEXT("Left stick"),TEXT("Right stick"),TEXT("Menu"),TEXT("View")};
        return B.Code>=0&&B.Code<14?Names[B.Code]:TEXT("Button");
    }
    switch(B.Code){case 0x52:return TEXT("Up");case 0x51:return TEXT("Down");case 0x50:return TEXT("Left");case 0x4F:return TEXT("Right");case 0x28:return TEXT("Enter");case 0x2C:return TEXT("Space");}
    if(B.Code>=4&&B.Code<=29)return FString::Printf(TEXT("%c (HID)"),TCHAR('A'+B.Code-4));
    return FString::Printf(TEXT("Keyboard HID %02X"),B.Code);
}
const TCHAR* SaveSlot=TEXT("IAmSpeed.DrivingPreferences.v1");
}

void ASpeedDrivingController::PrepareInputConfigurationV2()
{
    if(Preferences)return;
    Preferences=Cast<USpeedDrivingSettings>(UGameplayStatics::LoadGameFromSlot(SaveSlot,0));
    const bool Loaded=Preferences && Preferences->IsValid();
    if(!Preferences || !Preferences->IsValid())Preferences=NewObject<USpeedDrivingSettings>(this);
    if(auto* U=GEngine?GEngine->GetGameUserSettings():nullptr)
    {
        if(!Loaded)
        {
            const auto Size=U->GetScreenResolution();if(Size.X>=256&&Size.Y>=256){Preferences->WindowWidth=Size.X;Preferences->WindowHeight=Size.Y;}
            for(int32 Fps:FrameRates)if(FMath::IsNearlyEqual(U->GetFrameRateLimit(),float(Fps)))Preferences->MaximumFps=Fps;
        }
        for(int32 I=0;I<UE_ARRAY_COUNT(Resolutions);++I)if(FIntPoint(Preferences->WindowWidth,Preferences->WindowHeight)==Resolutions[I])ResolutionIndex=I;
        for(int32 I=0;I<UE_ARRAY_COUNT(FrameRates);++I)if(Preferences->MaximumFps==FrameRates[I])FpsIndex=I;
        if(Loaded)ApplyVideoPreferences();
    }
}
std::shared_ptr<const Speed::Input::V2::FInputActionContract> ASpeedDrivingController::GetDrivingContract() const
{
    Speed::Driving::FBindings B;
    if(Preferences && !Preferences->ReadBindings(B))return {};
    return Speed::Driving::MakeContract(B);
}
std::shared_ptr<Speed::Input::V2::FInputHostSession> ASpeedDrivingController::CreateInputSessionV2(uint64 FirstFrame)
{
    using namespace Speed::Input::V2;
    if(!Preferences)PrepareInputConfigurationV2();
    if(!DrivingWarmContext)DrivingWarmContext=CreateDeviceInputWarmContext();
    if(!DrivingWarmContext)return {};
    FDeviceInputHostConfig C;C.Contract=GetDrivingContract();C.FirstFrame=FirstFrame;
    C.Activity={{.20f,.12f,.04f},{.08f,.03f,.02f},30,8,Speed::Input::EDeviceKind::Gamepad,Speed::Input::EDeviceKind::Gamepad,true};
    C.Cadence=std::chrono::microseconds(1000);C.StartupTimeout=std::chrono::milliseconds(5000);C.WarmContext=DrivingWarmContext;
    C.Controls={{Speed::Driving::PauseAction,EControlCommand::Pause},{Speed::Driving::ResetAction,EControlCommand::ResetWorld}};
    C.ResumeRearmMask=(uint32(1)<<Speed::Driving::PauseAction)|(uint32(1)<<Speed::Driving::ResetAction);
    return CreateDeviceInputHost(C);
}
void ASpeedDrivingController::OnPauseStateChanged(bool bPaused)
{
    if(bPaused && !bOwnsMenuPointer)
    {
        bPreviousMouseCursor=bShowMouseCursor;bPreviousClickEvents=bEnableClickEvents;bOwnsMenuPointer=true;
        bShowMouseCursor=true;bEnableClickEvents=true;
        FInputModeGameAndUI Mode;Mode.SetHideCursorDuringCapture(false);Mode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
        if(GetLocalPlayer())SetInputMode(Mode);
    }
    else if(!bPaused && bOwnsMenuPointer)
    {
        bShowMouseCursor=bPreviousMouseCursor;bEnableClickEvents=bPreviousClickEvents;bOwnsMenuPointer=false;
        if(GetLocalPlayer())SetInputMode(FInputModeGameOnly());
    }
    bMenuVisible=bPaused;Page=EPage::Pause;Row=0;bCapture=false;++MenuContext;
    if(bPaused)Message.Reset();
}
Speed::Input::V2::EControlApplication ASpeedDrivingController::ExecuteInputControlV2(const Speed::Input::V2::FControlRequest& R)
{
    using namespace Speed::Input::V2;
    if(R.Command==EControlCommand::Pause && IsPaused() && (Page!=EPage::Pause || bCapture))
    {Navigate(Speed::Driving::EMenuAction::Back);return EControlApplication::Applied;}
    if(R.Command==EControlCommand::ResetWorld)
    {
        if(GetNetMode()!=NM_Standalone || IsPaused() || IsInputLifecycleFaulted() || bResetPending || R.State!=EStateAction::Started)return EControlApplication::Rejected;
        bResetPending=true;return EControlApplication::Dispatched;
    }
    return Super::ExecuteInputControlV2(R);
}
void ASpeedDrivingController::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    if(!bCameraApplied && SpeedCar)bCameraApplied=QueueCameraSettings();
    const auto J=ReadInputJournalV2();
    if(J!=MenuJournal){MenuJournal=J;MenuInput=J?std::make_unique<Speed::Driving::FMenuInput>(J):nullptr;++MenuContext;}
    if(MenuInput)
    {
        std::optional<Speed::Driving::FMenuInput::FCapture> Captured;
        const auto Events=MenuInput->Read(bMenuVisible?MenuContext:0,bCapture,Captured);
        const uint64 ReadContext=MenuContext;
        for(const auto& E:Events)
        {if(!bMenuVisible || MenuContext!=ReadContext)break;Navigate(E.Action);}
        if(bMenuVisible && bCapture && Captured)Capture(*Captured);
    }
    if(bResetPending)
    {
        bResetPending=false;const auto Level=UGameplayStatics::GetCurrentLevelName(this,true);
        if(!Level.IsEmpty())UGameplayStatics::OpenLevel(this,FName(*Level));
    }
}
void ASpeedDrivingController::EndPlay(EEndPlayReason::Type Reason)
{
    MenuInput.reset();MenuJournal.reset();Super::EndPlay(Reason);
    if(DrivingWarmContext && DrivingWarmContext->Close())DrivingWarmContext.reset();
}
int32 ASpeedDrivingController::RowCount() const
{return Page==EPage::Pause?4:Page==EPage::Graphism?3:Page==EPage::Camera?8:12;}
FName ASpeedDrivingController::MenuHitBoxName(int32 Index) const
{return FName(*FString::Printf(TEXT("IAmSpeedDriving.%llu.%d"),MenuContext,Index));}
bool ASpeedDrivingController::HandleDrivingMenuHitBoxClick(FName BoxName)
{
    if(!bMenuVisible || !IsPaused() || bCapture || bRemapPending)return false;
    for(int32 I=0;I<RowCount();++I)if(BoxName==MenuHitBoxName(I))
    {Row=I;Navigate(Speed::Driving::EMenuAction::Accept);return true;}
    return false;
}
void ASpeedDrivingController::ChangePage(EPage P)
{Page=P;Row=0;bCapture=false;Message.Reset();++MenuContext;}
void ASpeedDrivingController::Navigate(Speed::Driving::EMenuAction A)
{
    using Speed::Driving::EMenuAction;
    if(bRemapPending)return;
    const auto ResumeFromMenu=[this]
    {
        // Match Pause(): presentation changes only after the existing lifecycle
        // accepts the request. SetPause retains all owner/baseline fences.
        if(SetPause(false))OnPauseStateChanged(false);
        else Message=TEXT("Resume was not accepted; the menu remains open.");
    };
    if(A==EMenuAction::Back)
    {
        if(bCapture){bCapture=false;++MenuContext;Message=TEXT("Binding cancelled");return;}
        if(Page==EPage::Pause){ResumeFromMenu();return;}
        SavePreferences();ChangePage(EPage::Pause);return;
    }
    if(bCapture)return;
    if(A==EMenuAction::Up || A==EMenuAction::Down){Row=(Row+RowCount()+(A==EMenuAction::Up?-1:1))%RowCount();return;}
    if(A==EMenuAction::Left || A==EMenuAction::Right){Adjust(A==EMenuAction::Left?-1:1);return;}
    if(A!=EMenuAction::Accept)return;
    if(Page==EPage::Pause)
    {if(Row==0)ResumeFromMenu();else ChangePage(Row==1?EPage::Graphism:Row==2?EPage::Camera:EPage::Commands);return;}
    if(Page==EPage::Commands && Row>=1 && Row<=9)
    {bCapture=true;++MenuContext;Message=TEXT("Press a key / button or move an axis. P / Escape / B cancels.");return;}
    if((Page==EPage::Graphism&&Row==2)||(Page==EPage::Camera&&Row==7)||(Page==EPage::Commands&&Row==11))
    {SavePreferences();ChangePage(EPage::Pause);return;}
    if(Page==EPage::Commands && Row==10)
    {Message=TEXT("Select a command to replace its binding; Escape and navigation remain reserved.");return;}
    Adjust(1);
}
void ASpeedDrivingController::Adjust(int32 D)
{
    if(Page==EPage::Graphism)
    {
        if(Row==0){ResolutionIndex=(ResolutionIndex+UE_ARRAY_COUNT(Resolutions)+D)%UE_ARRAY_COUNT(Resolutions);Preferences->WindowWidth=Resolutions[ResolutionIndex].X;Preferences->WindowHeight=Resolutions[ResolutionIndex].Y;}
        else if(Row==1){FpsIndex=(FpsIndex+UE_ARRAY_COUNT(FrameRates)+D)%UE_ARRAY_COUNT(FrameRates);Preferences->MaximumFps=FrameRates[FpsIndex];}
        else return;
        ApplyVideoPreferences();SavePreferences();return;
    }
    if(Page==EPage::Commands){if(Row==0)bGamepadBinding=!bGamepadBinding;return;}
    if(Page!=EPage::Camera || !Preferences)return;
    float* Value=nullptr;float Old=0;
    switch(Row)
    {
    case 0:Value=&Preferences->CameraDistance;Old=*Value;*Value=FMath::Clamp(*Value+25*D,200.f,1500.f);break;
    case 1:Value=&Preferences->CameraHeight;Old=*Value;*Value=FMath::Clamp(*Value+10*D,0.f,600.f);break;
    case 2:Value=&Preferences->CameraFov;Old=*Value;*Value=FMath::Clamp(*Value+5*D,60.f,140.f);break;
    case 3:Value=&Preferences->CameraAngle;Old=*Value;*Value=FMath::Clamp(*Value+D,-30.f,15.f);break;
    case 4:Value=&Preferences->CameraStiffness;Old=*Value;*Value=FMath::Clamp(*Value+.05f*D,0.f,1.f);break;
    case 5:Value=&Preferences->CameraSwivelSpeed;Old=*Value;*Value=FMath::Clamp(*Value+.2f*D,.2f,8.f);break;
    case 6:Preferences->bInvertCameraPitch=!Preferences->bInvertCameraPitch;break;
    default:return;
    }
    if(!QueueCameraSettings(Row))
    {if(Value)*Value=Old;else Preferences->bInvertCameraPitch=!Preferences->bInvertCameraPitch;Message=TEXT("Camera settings could not be applied");}
    else SavePreferences();
}
bool ASpeedDrivingController::QueueCameraSettings(int32 ChangedRow)
{
    if(!SpeedCar || !Preferences)return false;auto* Arm=SpeedCar->GetSpeedCameraArm();if(!Arm)return false;
    using K=ESpeedCarCameraCommandKind;
    const K Kinds[]={K::Distance,K::Height,K::FieldOfView,K::Angle,K::Stiffness,K::SwivelSpeed,K::InvertSwivel};
    const float Values[]={Preferences->CameraDistance,Preferences->CameraHeight,Preferences->CameraFov,Preferences->CameraAngle,
        Preferences->CameraStiffness,Preferences->CameraSwivelSpeed,Preferences->bInvertCameraPitch?1.f:0.f};
    for(int32 I=0;I<7;++I)if((ChangedRow<0 || ChangedRow==I)&&!Arm->QueueGenericCameraSetting(Kinds[I],Values[I]))return false;
    return true;
}
void ASpeedDrivingController::Capture(const Speed::Driving::FMenuInput::FCapture& B)
{
    using namespace Speed::Input::V2;
    const bool Keyboard=B.Control.Kind==ERawControlKind::KeyboardUsage;
    if(Keyboard==bGamepadBinding){Message=TEXT("Use the selected device type");return;}
    if((Keyboard&&(Speed::Driving::IsReservedMenuKey(B.Control.Code))) || (!Keyboard&&B.Control.Kind==ERawControlKind::PadButton&&B.Control.Code==uint16(EPadButton::Menu)))
    {Message=TEXT("This control is reserved for the menu");return;}
    const int32 Index=Row-1;if(Index<0||Index>=9)return;
    if(Index==4&&B.Control.Kind==ERawControlKind::PadAxis){Message=TEXT("Reset requires a key or button");return;}
    if(!CanRestartInputSessionAtBoundary()){Message=TEXT("Cannot change controls right now");return;}
    auto& Device=bGamepadBinding?Preferences->Gamepad:Preferences->Keyboard;
    const auto Old=Device[Index];const int64 OldRevision=Preferences->MappingRevision;
    // Reject a duplicate direction instead of silently invoking two game actions.
    for(int32 I=0;I<Device.Num();++I)if(I!=Index&&Device[I].Kind==int32(B.Control.Kind)&&Device[I].Code==B.Control.Code&&Device[I].Direction==B.Direction)
    {Message=TEXT("Already assigned to another command");return;}
    Device[Index].Kind=int32(B.Control.Kind);Device[Index].Code=B.Control.Code;Device[Index].Direction=B.Direction;
    ++Preferences->MappingRevision;
    if(!Preferences->IsValid()){Device[Index]=Old;Preferences->MappingRevision=OldRevision;Message=TEXT("Unsupported binding");return;}
    bRemapPending=true;bCapture=false;++MenuContext;Message=TEXT("Applying binding...");
    const bool Pad=bGamepadBinding;
    if(!RestartInputSessionAtBoundaryWithCompletion([this,Pad,Index,Old,OldRevision](bool Success)
    {
        bRemapPending=false;
        if(Success && SavePreferences())Message=TEXT("Binding saved");
        else if(Success)return; // Preserve the save error message.
        else{(Pad?Preferences->Gamepad:Preferences->Keyboard)[Index]=Old;Preferences->MappingRevision=OldRevision;Message=TEXT("Binding rejected; previous preferences retained");}
    }))
    {bRemapPending=false;Device[Index]=Old;Preferences->MappingRevision=OldRevision;Message=TEXT("Binding restart rejected");}
}
bool ASpeedDrivingController::SavePreferences()
{
    if(Preferences && Preferences->IsValid() && UGameplayStatics::SaveGameToSlot(Preferences,SaveSlot,0))return true;
    Message=TEXT("Preferences applied but could not be saved");return false;
}
void ASpeedDrivingController::ApplyVideoPreferences()
{
    if(auto* U=GEngine?GEngine->GetGameUserSettings():nullptr)
    {
        U->SetFullscreenMode(EWindowMode::Windowed);U->SetScreenResolution(FIntPoint(Preferences->WindowWidth,Preferences->WindowHeight));
        U->SetVSyncEnabled(false);U->SetFrameRateLimit(float(Preferences->MaximumFps));U->ApplySettings(false);U->ConfirmVideoMode();U->SaveSettings();
        Message=TEXT("Saved (PIE can constrain the viewport)");
    }
}
void ASpeedDrivingController::DrawDrivingMenu(AHUD& H,const FVector2D& ViewportSize) const
{
    if(!bMenuVisible || !Preferences || ViewportSize.X<=0 || ViewportSize.Y<=0)return;
    TArray<FString> Lines;FString Title;
    if(Page==EPage::Pause){Title=TEXT("PAUSE");Lines={TEXT("Resume"),TEXT("Graphism"),TEXT("Camera"),TEXT("Commands")};}
    if(Page==EPage::Graphism)
    {Title=TEXT("GRAPHISM");Lines={FString::Printf(TEXT("Window resolution: %d x %d"),Preferences->WindowWidth,Preferences->WindowHeight),Preferences->MaximumFps?FString::Printf(TEXT("Maximum FPS: %d"),Preferences->MaximumFps):TEXT("Maximum FPS: Uncapped"),TEXT("Back")};}
    if(Page==EPage::Camera)
    {Title=TEXT("CAMERA");Lines={FString::Printf(TEXT("Distance: %.0f cm"),Preferences->CameraDistance),FString::Printf(TEXT("Height: %.0f cm"),Preferences->CameraHeight),FString::Printf(TEXT("Field of view: %.0f"),Preferences->CameraFov),FString::Printf(TEXT("Pitch angle: %.0f"),Preferences->CameraAngle),FString::Printf(TEXT("Stiffness: %.2f"),Preferences->CameraStiffness),FString::Printf(TEXT("Rotation speed: %.1f"),Preferences->CameraSwivelSpeed),Preferences->bInvertCameraPitch?TEXT("Invert pitch: On"):TEXT("Invert pitch: Off"),TEXT("Save and back")};}
    if(Page==EPage::Commands)
    {Title=TEXT("COMMANDS");Lines.Add(bGamepadBinding?TEXT("Device: Gamepad"):TEXT("Device: Keyboard"));const auto& Device=bGamepadBinding?Preferences->Gamepad:Preferences->Keyboard;for(int32 I=0;I<9;++I)Lines.Add(FString::Printf(TEXT("%s: %s"),Commands[I],*BindingName(Device[I])));Lines.Add(TEXT("Navigation: arrows / D-pad, Enter / A, Escape / B"));Lines.Add(TEXT("Save and back"));}
    const float Scale=FMath::Clamp(float(ViewportSize.Y/900.),.6f,1.4f),Width=740*Scale,Line=31*Scale;
    const float X=(ViewportSize.X-Width)*.5f,Y=FMath::Max(40.f,float((ViewportSize.Y-(Lines.Num()+5)*Line)*.5));
    H.DrawRect(FLinearColor(0.025f,.035f,.05f,.96f),X-22*Scale,Y-20*Scale,Width+44*Scale,(Lines.Num()+5)*Line);
    H.DrawText(Title,FLinearColor(.2f,.85f,1),X,Y,nullptr,1.4f*Scale);
    for(int32 I=0;I<Lines.Num();++I)
    {
        H.AddHitBox(FVector2D(X-10*Scale,Y+(I+1.5f)*Line),FVector2D(Width+20*Scale,Line),MenuHitBoxName(I),true);
        if(I==Row)H.DrawRect(FLinearColor(.08f,.25f,.36f,.95f),X-10*Scale,Y+(I+1.5f)*Line,Width+20*Scale,Line);
        H.DrawText((I==Row?TEXT("> "):TEXT("  "))+Lines[I],FLinearColor::White,X,Y+(I+1.6f)*Line,nullptr,Scale);
    }
    H.DrawText(Message.IsEmpty()?TEXT("Arrows / D-pad: navigate  |  Enter / A: select  |  Escape / B: back"):Message,FLinearColor(.8f,.86f,.9f),X,Y+(Lines.Num()+3)*Line,nullptr,.8f*Scale);
}
