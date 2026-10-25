#pragma once
#include "IAmSpeed/Input/InputActionContract.h"

namespace Speed::Driving
{
using namespace Input;
using namespace Input::V2;
// Stable IDs shared by the stock driving controller, recordings and camera.
constexpr FActionId PauseAction=3, ResetAction=4, CameraYaw=5, CameraPitch=6;
// P is available in Play In Editor, where Escape is owned by Stop Simulation.
constexpr uint16_t PauseKeyUsage=0x13;
inline bool IsReservedMenuKey(uint16_t Code)
{ return Code==0x29 || Code==0x28 || Code==PauseKeyUsage; }
enum class ECommand : uint8_t { Throttle, Brake, Left, Right, Reset, CameraLeft, CameraRight, CameraUp, CameraDown, Count };
struct FBinding { FRawControl Control; float Direction=1; };
struct FBindings
{
    std::array<FBinding,9> Keyboard{{
        {{ERawControlKind::KeyboardUsage,0x52},1},{{ERawControlKind::KeyboardUsage,0x51},1},
        {{ERawControlKind::KeyboardUsage,0x50},1},{{ERawControlKind::KeyboardUsage,0x4F},1},
        {{ERawControlKind::KeyboardUsage,0x15},1},{{ERawControlKind::KeyboardUsage,0x0D},1},
        {{ERawControlKind::KeyboardUsage,0x0F},1},{{ERawControlKind::KeyboardUsage,0x0C},1},
        {{ERawControlKind::KeyboardUsage,0x0E},1}}};
    std::array<FBinding,9> Gamepad{{
        {{ERawControlKind::PadAxis,uint16_t(EPadAxis::RightTrigger)},1},
        {{ERawControlKind::PadAxis,uint16_t(EPadAxis::LeftTrigger)},1},
        {{ERawControlKind::PadAxis,uint16_t(EPadAxis::LeftX)},-1},
        {{ERawControlKind::PadAxis,uint16_t(EPadAxis::LeftX)},1},
        {{ERawControlKind::PadButton,uint16_t(EPadButton::View)},1},
        {{ERawControlKind::PadAxis,uint16_t(EPadAxis::RightX)},-1},
        {{ERawControlKind::PadAxis,uint16_t(EPadAxis::RightX)},1},
        {{ERawControlKind::PadAxis,uint16_t(EPadAxis::RightY)},1},
        {{ERawControlKind::PadAxis,uint16_t(EPadAxis::RightY)},-1}}};
    uint64_t Revision=5;
};
inline std::shared_ptr<const FInputActionContract> MakeContract(const FBindings& Bindings)
{
    FInputActionContractDescription D;D.Revision={Bindings.Revision};D.Actions=FInputActionContract::BaseActions();
    D.Actions[Steering].Deadzone=.08f;
    for(auto Id:{PauseAction,ResetAction,CameraYaw,CameraPitch})
    {
        FActionDefinition A;A.Id=Id;A.Owner="IAmSpeed";
        A.Name=Id==PauseAction?"Pause":Id==ResetAction?"ResetWorld":Id==CameraYaw?"CameraYaw":"CameraPitch";
        A.Wiring=EActionWiring::Wired;
        if(Id>=CameraYaw){A.Type=EActionType::Axis1D;A.Signed=true;A.Quantization=127;A.Deadzone=.12f;}
        D.Actions.push_back(A);
    }
    constexpr FActionId Actions[]={Throttle,Brake,Steering,Steering,ResetAction,CameraYaw,CameraYaw,CameraPitch,CameraPitch};
    constexpr float Signs[]={1,1,-1,1,1,-1,1,-1,1};
    for(const auto* Device:{&Bindings.Keyboard,&Bindings.Gamepad})for(size_t I=0;I<Device->size();++I)
    {
        const auto B=(*Device)[I];
        if(!B.Control.IsValid() || (B.Direction!=1 && B.Direction!=-1))return {};
        if(Device==&Bindings.Keyboard && (B.Control.Kind!=ERawControlKind::KeyboardUsage || IsReservedMenuKey(B.Control.Code)))return {};
        if(Device==&Bindings.Gamepad && B.Control.Kind!=ERawControlKind::PadAxis && B.Control.Kind!=ERawControlKind::PadButton)return {};
        if(I==4 && B.Control.Kind==ERawControlKind::PadAxis)return {}; // Reset is a digital command.
        FRawActionBinding M{B.Control,Actions[I],B.Direction,{}};
        if(I!=4)
        {
            // Split signed sticks into two half axes, then apply the destination
            // sign. Binding both directions to one stick never doubles its value.
            M.Modifiers.push_back({EScalarModifier::Clamp,0,1});
            M.Modifiers.push_back({EScalarModifier::Scale,Signs[I],0});
        }
        D.Mapping.push_back(std::move(M));
    }
    D.Mapping.push_back({{ERawControlKind::KeyboardUsage,0x29},PauseAction,1,{}});
    D.Mapping.push_back({{ERawControlKind::KeyboardUsage,PauseKeyUsage},PauseAction,1,{}});
    D.Mapping.push_back({{ERawControlKind::PadButton,uint16_t(EPadButton::Menu)},PauseAction,1,{}});
    D.Physical={{Throttle,EPhysicalDestination::Throttle},{Brake,EPhysicalDestination::Brake},{Steering,EPhysicalDestination::Steering}};
    return FInputActionContract::Create(std::move(D));
}
}
