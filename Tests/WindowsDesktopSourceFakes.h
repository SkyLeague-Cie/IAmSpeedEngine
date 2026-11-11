#pragma once
#include "WindowsDiscoveryFakes.h"
#include "IAmSpeed/Input/Windows/GameInputDesktopSource.h"
#include <map>
#include <functional>
class DesktopReading final:public FakeCom<IGameInputReading>
{
public:
    ComPtr<IGameInputDevice> Device;
    GameInputKind Kind=GameInputKindKeyboard;
    std::uint64_t Stamp=1;
    std::vector<GameInputKeyState> Keys;
    GameInputMouseState Mouse{};
    GameInputGamepadState Pad{};
    bool ValidMouse=true;
    unsigned KeyReads=0,MouseReads=0;
    GameInputKind STDMETHODCALLTYPE GetInputKind()override{return Kind;}
    std::uint64_t STDMETHODCALLTYPE GetTimestamp()override{return Stamp;}
    void STDMETHODCALLTYPE GetDevice(IGameInputDevice** Out)override{*Out=Device.Get();if(*Out)(*Out)->AddRef();}
    std::uint32_t STDMETHODCALLTYPE GetControllerAxisCount()override{return 0;}
    std::uint32_t STDMETHODCALLTYPE GetControllerAxisState(std::uint32_t,float*)override{return 0;}
    std::uint32_t STDMETHODCALLTYPE GetControllerButtonCount()override{return 0;}
    std::uint32_t STDMETHODCALLTYPE GetControllerButtonState(std::uint32_t,bool*)override{return 0;}
    std::uint32_t STDMETHODCALLTYPE GetControllerSwitchCount()override{return 0;}
    std::uint32_t STDMETHODCALLTYPE GetControllerSwitchState(std::uint32_t,GameInputSwitchPosition*)override{return 0;}
    std::uint32_t STDMETHODCALLTYPE GetKeyCount()override{return static_cast<std::uint32_t>(Keys.size());}
    std::uint32_t STDMETHODCALLTYPE GetKeyState(std::uint32_t N,GameInputKeyState* Out)override
    {++KeyReads;const auto Count=(std::min)(N,static_cast<std::uint32_t>(Keys.size()));for(std::uint32_t I=0;I<Count;++I)Out[I]=Keys[I];return Count;}
    bool STDMETHODCALLTYPE GetMouseState(GameInputMouseState* Out)override{++MouseReads;*Out=Mouse;return ValidMouse;}
    bool STDMETHODCALLTYPE GetSensorsState(GameInputSensorsState*)override{return false;}
    bool STDMETHODCALLTYPE GetArcadeStickState(GameInputArcadeStickState*)override{return false;}
    bool STDMETHODCALLTYPE GetFlightStickState(GameInputFlightStickState*)override{return false;}
    bool STDMETHODCALLTYPE GetGamepadState(GameInputGamepadState* Out)override{*Out=Pad;return bool(Kind&GameInputKindGamepad);}
    bool STDMETHODCALLTYPE GetRacingWheelState(GameInputRacingWheelState*)override{return false;}
    bool STDMETHODCALLTYPE GetRawReport(IGameInputRawDeviceReport**)override{return false;}
};
class DesktopApi final:public DiscoveryApi
{
public:
    struct FCallback
    {IGameInputDevice* Device=nullptr;GameInputKind Kind=GameInputKindKeyboard;void* Context=nullptr;GameInputReadingCallback Function=nullptr;};
    std::map<GameInputCallbackToken,FCallback> ReadingCallbacks;
    std::map<IGameInputDevice*,ComPtr<DesktopReading>> Current;
    std::map<IGameInputDevice*,std::vector<ComPtr<DesktopReading>>> History;
    std::mutex ReadGate;std::condition_variable ReadFinished;
    unsigned ReadingRunning=0;GameInputCallbackToken NextReadingToken=1;
    bool FailReadUnregister=false,FailReadRegister=false;
    std::atomic<bool> UnregisterWaiting{false};
    std::function<void()> DuringCurrent,BeforeReading;
    HRESULT STDMETHODCALLTYPE RegisterReadingCallback(IGameInputDevice* D,GameInputKind K,void* C,GameInputReadingCallback F,GameInputCallbackToken* T)override
    {
        if(FailReadRegister)return E_FAIL;
        std::lock_guard<std::mutex> Lock(ReadGate);*T=++NextReadingToken;ReadingCallbacks.emplace(*T,FCallback{D,K,C,F});return S_OK;
    }
    HRESULT STDMETHODCALLTYPE GetCurrentReading(GameInputKind K,IGameInputDevice* D,IGameInputReading** Out)override
    {
        ++CurrentCalls;Check(D!=nullptr,"desktop queries retained device, never implicit any-device");
        if(DuringCurrent){auto F=std::move(DuringCurrent);DuringCurrent={};F();}
        std::lock_guard<std::mutex> Lock(ReadGate);*Out=nullptr;
        const auto I=Current.find(D);if(I==Current.end())return GAMEINPUT_E_READING_NOT_FOUND;
        Check(bool(I->second->Kind&K),"SDK query preserves actual source kind");
        *Out=I->second.Get();(*Out)->AddRef();return S_OK;
    }
    HRESULT STDMETHODCALLTYPE GetNextReading(IGameInputReading* Before,GameInputKind K,IGameInputDevice* D,IGameInputReading** Out)override
    {
        ++ReadCalls;++NextCalls;std::lock_guard<std::mutex> Lock(ReadGate);*Out=nullptr;
        const auto& H=History[D];
        for(std::size_t I=0;I<H.size();++I)if(H[I].Get()==Before)
        {
            if(I+1==H.size())return GAMEINPUT_E_READING_NOT_FOUND;
            Check(bool(H[I+1]->Kind&K),"selected history preserves actual SDK kind");
            *Out=H[I+1].Get();(*Out)->AddRef();return S_OK;
        }
        return GAMEINPUT_E_REFERENCE_READING_TOO_OLD;
    }
    bool STDMETHODCALLTYPE UnregisterCallback(GameInputCallbackToken Token)override
    {
        if(Token==1)return FakeApi::UnregisterCallback(Token);
        std::unique_lock<std::mutex> Lock(ReadGate);
        if(FailReadUnregister)return false;
        if(ReadingRunning)UnregisterWaiting.store(true);
        ReadFinished.wait(Lock,[&](){return ReadingRunning==0;});
        ReadingCallbacks.erase(Token);return true;
    }
    void Push(const ComPtr<DesktopReading>& R)
    {
        std::vector<std::pair<GameInputCallbackToken,FCallback>> Calls;
        {
            std::lock_guard<std::mutex> Lock(ReadGate);Current[R->Device.Get()]=R;History[R->Device.Get()].push_back(R);
            for(const auto& P:ReadingCallbacks)if(P.second.Device==R->Device.Get()&&(P.second.Kind&R->Kind))Calls.push_back(P);
            ReadingRunning+=static_cast<unsigned>(Calls.size());
        }
        for(const auto& P:Calls)
        {
            if(BeforeReading)BeforeReading();
            P.second.Function(P.first,P.second.Context,R.Get());
            std::lock_guard<std::mutex> Lock(ReadGate);--ReadingRunning;ReadFinished.notify_all();
        }
    }
    ComPtr<DesktopReading> Keyboard(FakeDevice* D,std::uint64_t Stamp,bool Held)
    {
        ComPtr<DesktopReading> R;R.Attach(new DesktopReading);R->Device=D;R->Kind=GameInputKindKeyboard;R->Stamp=Stamp;
        if(Held){GameInputKeyState Key{};Key.scanCode=0x11;Key.virtualKey='W';R->Keys.push_back(Key);}
        return R;
    }
    ComPtr<DesktopReading> Gamepad(FakeDevice* D,std::uint64_t Stamp,bool Held)
    {
        ComPtr<DesktopReading> R;R.Attach(new DesktopReading);R->Device=D;R->Kind=GameInputKindGamepad;R->Stamp=Stamp;
        R->Pad.buttons=Held?GameInputGamepadA:GameInputGamepadNone;R->Pad.rightTrigger=Held?1.f:0.f;return R;
    }
    ComPtr<DesktopReading> Mouse(FakeDevice* D,std::uint64_t Stamp,GameInputMouseButtons Buttons)
    {
        ComPtr<DesktopReading> R;R.Attach(new DesktopReading);R->Device=D;R->Kind=GameInputKindMouse;R->Stamp=Stamp;R->Mouse.buttons=Buttons;return R;
    }
};
