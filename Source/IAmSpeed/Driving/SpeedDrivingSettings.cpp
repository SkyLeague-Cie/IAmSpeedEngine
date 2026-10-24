#include "SpeedDrivingSettings.h"

USpeedDrivingSettings::USpeedDrivingSettings()
{
    const Speed::Driving::FBindings Defaults;
    const auto Copy=[](const auto& From,TArray<FSpeedDrivingBinding>& To)
    {for(const auto& B:From){FSpeedDrivingBinding V;V.Kind=int32(B.Control.Kind);V.Code=B.Control.Code;V.Direction=B.Direction;To.Add(V);}};
    Copy(Defaults.Keyboard,Keyboard);Copy(Defaults.Gamepad,Gamepad);
}
bool USpeedDrivingSettings::ReadBindings(Speed::Driving::FBindings& Out) const
{
    if(Keyboard.Num()!=9 || Gamepad.Num()!=9 || MappingRevision<=0)return false;
    Out.Revision=uint64(MappingRevision);
    const auto Copy=[](const TArray<FSpeedDrivingBinding>& From,auto& To)
    {
        for(int32 I=0;I<9;++I)
        {
            const auto& B=From[I];if(B.Kind<0 || B.Kind>2 || B.Code<0 || B.Code>65535)return false;
            To[I]={{Speed::Input::V2::ERawControlKind(B.Kind),uint16(B.Code)},B.Direction};
        }
        return true;
    };
    return Copy(Keyboard,Out.Keyboard)&&Copy(Gamepad,Out.Gamepad)&&bool(Speed::Driving::MakeContract(Out));
}
bool USpeedDrivingSettings::IsValid() const
{
    Speed::Driving::FBindings B;
    const int32 AllowedFps[]={0,30,60,90,100,120,144,165,240,360};bool FpsValid=false;
    for(int32 Fps:AllowedFps)FpsValid|=MaximumFps==Fps;
    return Schema==1 && WindowWidth>=256&&WindowWidth<=16384&&WindowHeight>=256&&WindowHeight<=16384&&FpsValid&&ReadBindings(B) && FMath::IsFinite(CameraDistance) && CameraDistance>=200 && CameraDistance<=1500 &&
        FMath::IsFinite(CameraHeight)&&CameraHeight>=0&&CameraHeight<=600 && FMath::IsFinite(CameraFov)&&CameraFov>=60&&CameraFov<=140 &&
        FMath::IsFinite(CameraAngle)&&CameraAngle>=-30&&CameraAngle<=15 && FMath::IsFinite(CameraStiffness)&&CameraStiffness>=0&&CameraStiffness<=1 &&
        FMath::IsFinite(CameraSwivelSpeed)&&CameraSwivelSpeed>=.2f&&CameraSwivelSpeed<=8;
}
