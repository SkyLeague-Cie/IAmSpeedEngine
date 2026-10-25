#pragma once
#include "IAmSpeed/Input/RawAcquisitionJournal.h"
#include "DrivingInput.h"

namespace Speed::Driving
{
using namespace Input::V2;
enum class EMenuAction : uint8_t { Up,Down,Left,Right,Accept,Back };
struct FMenuEvent { EMenuAction Action; uint64_t Sequence=0; };
// A separate read cursor, NEVER a second acquisition or a physical input writer.
// Context changes, overflow and device baselines suppress held controls until release.
class FMenuInput
{
public:
    struct FCapture { FRawControl Control; float Direction=1; };
    explicit FMenuInput(std::shared_ptr<const FRawAcquisitionJournal> J):Journal(std::move(J)){}
    std::vector<FMenuEvent> Read(uint64_t Context,bool Capture,std::optional<FCapture>& Captured)
    {
        Captured.reset();std::vector<FMenuEvent> Events;
        const auto Base=Journal->ReadControlBaseline();if(!Base)return Events;
        if(!Cursor.Session || Context!=CurrentContext)
        {Cursor=Base->Cursor;Previous=Base->State;CurrentContext=Context;HavePrevious=true;return Events;}
        const auto H=Journal->ReadControlsSince(Cursor);
        if(H.Status==EAcquisitionRead::Gap || H.Status==EAcquisitionRead::InvalidCursor || H.Status==EAcquisitionRead::Closed)
        {Cursor=Base->Cursor;Previous=Base->State;HavePrevious=true;return Events;}
        for(const auto& R:H.Readings)
        {
            if(R.FreshBaseline || !HavePrevious || R.DeviceId!=Previous.DeviceId || R.Generation.Value!=Previous.Generation.Value)
            {Previous=R;HavePrevious=true;continue;}
            if(Context)for(size_t I=0;I<R.State.Count;++I)
            {
                const auto V=R.State.Values[I];float Old=0;
                for(size_t J=0;J<Previous.State.Count;++J)if(Previous.State.Values[J].Control==V.Control){Old=Previous.State.Values[J].Value;break;}
                const bool Axis=V.Control.Kind==ERawControlKind::PadAxis;
                if((Axis?std::abs(V.Value)<.65f:V.Value==0) || (Axis?std::abs(Old)>.25f:Old!=0))continue;
                if(Capture)
                {
                    // P / Escape / pad East cancels capture and stays usable.
                    if(IsBack(V.Control))Events.push_back({EMenuAction::Back,R.Sequence});
                    else if(!Captured && !(V.Control.Kind==ERawControlKind::PadButton && V.Control.Code==uint16_t(EPadButton::Menu)))
                        Captured=FCapture{V.Control,V.Value<0?-1.f:1.f};
                    continue;
                }
                const auto A=Navigation(V);if(A)Events.push_back({*A,R.Sequence});
            }
            Previous=R;
        }
        Cursor=H.Next;return Events;
    }
private:
    static bool IsBack(FRawControl C)
    {return (C.Kind==ERawControlKind::KeyboardUsage && (C.Code==0x29 || C.Code==PauseKeyUsage))||(C.Kind==ERawControlKind::PadButton && C.Code==uint16_t(EPadButton::East));}
    static std::optional<EMenuAction> Navigation(FRawValue V)
    {
        auto C=V.Control;
        if(IsBack(C))return EMenuAction::Back;
        if(C.Kind==ERawControlKind::KeyboardUsage)
        {
            switch(C.Code){case 0x52:return EMenuAction::Up;case 0x51:return EMenuAction::Down;case 0x50:return EMenuAction::Left;case 0x4F:return EMenuAction::Right;case 0x28:return EMenuAction::Accept;default:return {};}
        }
        if(C.Kind==ERawControlKind::PadButton)
        {
            switch(EPadButton(C.Code)){case EPadButton::DPadUp:return EMenuAction::Up;case EPadButton::DPadDown:return EMenuAction::Down;case EPadButton::DPadLeft:return EMenuAction::Left;case EPadButton::DPadRight:return EMenuAction::Right;case EPadButton::South:return EMenuAction::Accept;default:return {};}
        }
        if(C.Kind==ERawControlKind::PadAxis)
        {
            if(C.Code==uint16_t(EPadAxis::LeftX))return V.Value<0?EMenuAction::Left:EMenuAction::Right;
            if(C.Code==uint16_t(EPadAxis::LeftY))return V.Value<0?EMenuAction::Down:EMenuAction::Up;
        }
        return {};
    }
    std::shared_ptr<const FRawAcquisitionJournal> Journal;
    FAcquisitionCursor Cursor;FAcquiredRawState Previous;
    uint64_t CurrentContext=0;bool HavePrevious=false;
};
}
