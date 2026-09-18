// Standalone read-only hardware probe; never compiled into the Unreal module.
#include "IAmSpeed/Input/Windows/GameInputSelectedSource.h"
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

using namespace GameInput::v3;
using namespace Speed::Input;
using namespace Speed::Input::Windows;
template<class T> using ComPtr = Microsoft::WRL::ComPtr<T>;
static std::mutex Output;
static void Log(const std::string& Text) { std::lock_guard<std::mutex> Lock(Output); std::cout<<Text<<'\n'<<std::flush; }
static std::string IdText(const APP_LOCAL_DEVICE_ID& Id)
{
	std::ostringstream S; S<<std::hex<<std::setfill('0');
	for (auto Byte : reinterpret_cast<const unsigned char(&)[32]>(Id)) S<<std::setw(2)<<unsigned(Byte);
	return S.str();
}
struct FMonitor
{
	std::mutex Mutex;
	std::map<std::string,ComPtr<IGameInputDevice>> Devices;
	static void CALLBACK Changed(GameInputCallbackToken,void* Context,IGameInputDevice* Device,
		std::uint64_t Stamp,GameInputDeviceStatus Current,GameInputDeviceStatus)
	{
		auto& Self=*static_cast<FMonitor*>(Context);
		const GameInputDeviceInfo* Info=nullptr;
		const auto Error=Device->GetDeviceInfo(&Info);
		if(FAILED(Error)||!Info) { Log("{\"type\":\"device_info_error\",\"hr\":"+std::to_string(Error)+"}"); return; }
		const auto Id=IdText(Info->deviceId); const bool Connected=(Current&GameInputDeviceConnected)!=0;
		{ std::lock_guard<std::mutex> Lock(Self.Mutex); if(Connected) Self.Devices[Id]=Device; else Self.Devices.erase(Id); }
		Log("{\"type\":\"device\",\"id\":\""+Id+"\",\"os_us\":"+std::to_string(Stamp)
			+",\"connected\":"+(Connected?"true":"false")+",\"vendor\":"+std::to_string(Info->vendorId)
			+",\"product\":"+std::to_string(Info->productId)+",\"kinds\":"+std::to_string(static_cast<unsigned>(Info->supportedInput))+"}");
	}
};
int main(int Argc,char** Argv)
{
	// Fixed CLI: duration in seconds, optional scheduled pause start/duration.
	const int Seconds=Argc>1?std::atoi(Argv[1]):0;
	const int PauseAt=Argc>2?std::atoi(Argv[2]):-1;
	const int PauseFor=Argc>3?std::atoi(Argv[3]):0;
	if(Seconds<1||Seconds>180||PauseAt>Seconds||PauseFor<0||PauseFor>15) return 2;
	ComPtr<IGameInput> Api;
	const auto Init=GameInputCreate(Api.GetAddressOf());
	Log("{\"type\":\"init\",\"hr\":"+std::to_string(Init)+",\"sdk\":3,\"duration_s\":"+std::to_string(Seconds)+"}");
	if(FAILED(Init)||!Api) return 3;
	for(const wchar_t* Name : {L"GameInput.dll",L"GameInputRedist.dll"})
	{
		const auto Module=GetModuleHandleW(Name); wchar_t Path[32768]{};
		if(Module&&GetModuleFileNameW(Module,Path,32768))
		{
			// Hex UTF-16 path avoids quoting/localization ambiguities in JSON.
			std::ostringstream S; S<<"{\"type\":\"runtime_module\",\"path_utf16_hex\":\""<<std::hex<<std::setfill('0');
			for(const wchar_t* P=Path;*P;++P) S<<std::setw(4)<<static_cast<unsigned>(*P);
			S<<"\"}"; Log(S.str());
		}
	}
	// Harness only: permits testing while instructions remain visible in Codex.
	Api->SetFocusPolicy(GameInputEnableBackgroundInput);
	FMonitor Monitor; GameInputCallbackToken Token=0;
	const auto Kinds=GameInputKindRawDeviceReport|GameInputKindController|GameInputKindKeyboard|GameInputKindMouse
		|GameInputKindSensors|GameInputKindArcadeStick|GameInputKindFlightStick|GameInputKindGamepad|GameInputKindRacingWheel;
	const auto Registration=Api->RegisterDeviceCallback(nullptr,Kinds,GameInputDeviceConnected,
		GameInputBlockingEnumeration,&Monitor,&FMonitor::Changed,&Token);
	if(FAILED(Registration)) { Log("{\"type\":\"registration_error\",\"hr\":"+std::to_string(Registration)+"}"); return 4; }
	std::array<bool,ActionCount> Digital{}; for(std::size_t I=3;I<19;++I) Digital[I]=true;
	const FActivityConfig Config{{.20f,.15f,.10f},{.08f,.05f,.04f},0,32,EDeviceKind::Gamepad};
	auto Source=FGameInputSelectedSource::CreateAutomatic(Api.Get(),701,Digital,
		[](const FDeviceState& S,FActionValues& V) { V[Throttle]=S.VirtualKeys[VK_F9]?255:0; V[31]=1; return true; },
		[](const FDeviceState& S,FActionValues& V)
		{
			const auto T=QuantizeAxis(S.Axes[1],false),B=QuantizeAxis(S.Axes[0],false),X=QuantizeAxis(S.Axes[2],true);
			if(!T||!B||!X) return false;
			V[Throttle]=*T;V[Brake]=*B;V[Steering]=*X;V[31]=2;
			for(std::size_t I=0;I<16;++I) V[3+I]=(S.GamepadButtons&(std::uint32_t(1)<<I))?1:0;
			return true;
		},Config);
	if(!Source) { if(!Api->UnregisterCallback(Token)) std::_Exit(7); return 5; }
	Log("{\"type\":\"ready\",\"hz\":300,\"keyboard_recorded\":\"F9 test value only; no keycodes/text\",\"pause_at_s\":"+std::to_string(PauseAt)+",\"pause_for_s\":"+std::to_string(PauseFor)+"}");
	const auto Start=std::chrono::steady_clock::now(); bool Paused=false; std::uint64_t Frames=0,Missing=0;
	std::map<std::string,std::uint64_t> RawStamps;
	while(std::chrono::steady_clock::now()-Start<std::chrono::seconds(Seconds))
	{
		const auto Elapsed=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-Start).count();
		const bool WantPause=PauseAt>=0&&Elapsed>=std::int64_t(PauseAt)*1000000&&Elapsed<std::int64_t(PauseAt+PauseFor)*1000000;
		if(WantPause!=Paused) { Paused=WantPause; Log("{\"type\":\"pause\",\"us\":"+std::to_string(Elapsed)+",\"value\":"+(Paused?"true":"false")+",\"ok\":"+(Source->SetPaused(Paused)?"true":"false")+"}"); }
		const auto Frame=Source->Produce(Frames);
		if(!Frame) ++Missing;
		else
		{
			std::ostringstream S; S<<"{\"type\":\"frame\",\"n\":"<<Frames<<",\"us\":"<<Elapsed<<",\"reset\":"<<(Frame->RequiresReset()?"true":"false")<<",\"edges\":"<<Frame->GetEdgeCount()<<",\"values\":[";
			for(std::size_t I=0;I<ActionCount;++I) { if(I)S<<',';S<<Frame->GetActions()[I]; }
			S<<"],\"status\":"<<static_cast<int>(Source->GetLastPollStatus())<<",\"hr\":"<<Source->GetLastError()<<"}";Log(S.str());
		}
		// Diagnostic current gamepad state (all six axes), separate from authoritative producer.
		if(Frames%10==0)
		{
			std::map<std::string,ComPtr<IGameInputDevice>> Devices;
			{ std::lock_guard<std::mutex> Lock(Monitor.Mutex); Devices=Monitor.Devices; }
			for(const auto& D:Devices)
			{
				const GameInputDeviceInfo* Info=nullptr; if(FAILED(D.second->GetDeviceInfo(&Info))||!Info||!(Info->supportedInput&GameInputKindGamepad))continue;
				ComPtr<IGameInputReading> Reading;const auto Hr=Api->GetCurrentReading(GameInputKindGamepad,D.second.Get(),Reading.GetAddressOf());
				if(FAILED(Hr)||!Reading)continue;
				GameInputGamepadState Pad{};if(!Reading->GetGamepadState(&Pad)||RawStamps[D.first]==Reading->GetTimestamp())continue;
				RawStamps[D.first]=Reading->GetTimestamp();
				std::ostringstream S;S<<"{\"type\":\"raw_pad\",\"id\":\""<<D.first<<"\",\"us\":"<<Elapsed<<",\"os_us\":"<<Reading->GetTimestamp()<<",\"buttons\":"<<static_cast<unsigned>(Pad.buttons)<<",\"axes\":["<<Pad.leftTrigger<<','<<Pad.rightTrigger<<','<<Pad.leftThumbstickX<<','<<Pad.leftThumbstickY<<','<<Pad.rightThumbstickX<<','<<Pad.rightThumbstickY<<"]}";Log(S.str());
			}
		}
		++Frames; std::this_thread::sleep_until(Start+std::chrono::microseconds(Frames*1000000/300));
	}
	const auto FinalStatus=Source->GetLastPollStatus();const auto FinalError=Source->GetLastError();
	const bool SourceStopped=Source->Shutdown();const bool MonitorStopped=Api->UnregisterCallback(Token);
	Log("{\"type\":\"end\",\"frames\":"+std::to_string(Frames)+",\"missing\":"+std::to_string(Missing)+",\"status\":"+std::to_string(static_cast<int>(FinalStatus))+",\"hr\":"+std::to_string(FinalError)+",\"shutdown\":"+(SourceStopped&&MonitorStopped?"true":"false")+"}");
	if(!SourceStopped||!MonitorStopped) std::_Exit(7); // Never free a possibly live callback context.
	return Missing||FinalStatus==EPollStatus::Failed?6:0;
}
