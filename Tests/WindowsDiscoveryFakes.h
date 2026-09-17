#pragma once
#include "WindowsGameInputFakes.h"
#include "IAmSpeed/Input/Windows/GameInputDiscovery.h"
using Speed::Input::Windows::FGameInputDiscovery;
class DiscoveryApi : public FakeApi
{
public:
	struct Event { ComPtr<FakeDevice> Device; uint64_t Stamp; bool Connected; };
	std::vector<Event> Initial;
	bool AllKinds = false;
	void FireDevice(FakeDevice* Device, uint64_t Stamp, bool Connected)
	{
		GameInputDeviceCallback Function; void* Data;
		{ std::lock_guard<std::mutex> Lock(CallbackGate); if (!Callback) return; ++Running; Function = Callback; Data = Context; }
		Function(1, Data, Device, Stamp, Connected ? GameInputDeviceConnected : static_cast<GameInputDeviceStatus>(0), static_cast<GameInputDeviceStatus>(0));
		{ std::lock_guard<std::mutex> Lock(CallbackGate); --Running; CallbackFinished.notify_all(); }
	}
	HRESULT STDMETHODCALLTYPE RegisterDeviceCallback(IGameInputDevice* Device, GameInputKind Kind, GameInputDeviceStatus Status,
		GameInputEnumerationKind Enumeration, void* Data, GameInputDeviceCallback Function, GameInputCallbackToken* Token) override
	{
		AllKinds = !Device && (Kind & GameInputKindKeyboard) && (Kind & GameInputKindGamepad)
			&& (Kind & GameInputKindMouse) && (Kind & GameInputKindRawDeviceReport)
			&& Status == GameInputDeviceConnected && Enumeration == GameInputBlockingEnumeration;
		Callback = Function; Context = Data; *Token = 1;
		for (const auto& E : Initial) FireDevice(E.Device.Get(), E.Stamp, E.Connected);
		return S_OK;
	}
};
static ComPtr<FakeDevice> MakeDevice(std::uint8_t Id, GameInputKind Kind = GameInputKindGamepad)
{
	ComPtr<FakeDevice> D; D.Attach(new FakeDevice); D->Info.supportedInput = Kind;
	std::memset(&D->Info.deviceId, Id, sizeof(D->Info.deviceId)); return D;
}
static FDeviceId Key(std::uint8_t N) { FDeviceId R{}; R.fill(N); return R; }
