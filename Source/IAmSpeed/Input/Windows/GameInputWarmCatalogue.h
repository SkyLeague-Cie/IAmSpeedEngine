#pragma once

// Source-only integration prototype. Not included by the exact-810 build.
// The lasting GameInput callback owns one device catalogue. Fresh sessions
// subscribe and receive an atomic snapshot plus every later lifecycle event.

#include "GameInputAcquisition.h"
#include "GameInputFailureDiagnostic.h"
#include "../DeviceDiscovery.h"
#include <cstring>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <iterator>
#include <vector>

namespace Speed::Input::Windows
{
class FGameInputWarmCatalogue final
{
    using FApi = GameInput::v3::IGameInput;
    using FDevice = GameInput::v3::IGameInputDevice;
    template<class T> using TComPtr = Microsoft::WRL::ComPtr<T>;

public:
    struct FRecord
    {
        FDeviceId Id{};
        TComPtr<FDevice> Device;
        std::uint64_t Timestamp = 0;
        std::uint8_t Kinds = 0;
        bool Connected = false;
    };
    using FSubscriber = bool (*)(void* Context, const FRecord& Record) noexcept;
    using FFailure = void (*)(void* Context, HRESULT Reason) noexcept;

    static std::unique_ptr<FGameInputWarmCatalogue> Create(FApi* Api, HRESULT* RegistrationResult = nullptr)
    {
        using namespace GameInput::v3;
        if (RegistrationResult) *RegistrationResult = E_POINTER;
        if (!Api) return {};
        auto Result = std::unique_ptr<FGameInputWarmCatalogue>(new FGameInputWarmCatalogue(Api));
        const auto Kinds = GameInputKindRawDeviceReport | GameInputKindController | GameInputKindKeyboard
            | GameInputKindMouse | GameInputKindSensors | GameInputKindArcadeStick
            | GameInputKindFlightStick | GameInputKindGamepad | GameInputKindRacingWheel;
        const HRESULT Registration = Api->RegisterDeviceCallback(nullptr, Kinds, GameInputDeviceConnected,
            // Enumeration and later lifecycle events use the same registered
            // callback. A new view may start empty and stays neutral until an
            // actual device arrives and its fresh input baseline is admitted.
            // Blocking enumeration can fail with ERROR_TIMEOUT before any
            // callback is dispatched (measured startup2787/2789).
            GameInputAsyncEnumeration, Result.get(), &OnDevice, &Result->Token);
        if (RegistrationResult) *RegistrationResult = Registration;
        if (FAILED(Registration)) return {};
        Result->Registered = true;
        return Result;
    }

    ~FGameInputWarmCatalogue() { if (!Shutdown()) std::terminate(); }
    FGameInputWarmCatalogue(const FGameInputWarmCatalogue&) = delete;
    FGameInputWarmCatalogue& operator=(const FGameInputWarmCatalogue&) = delete;

    // Single linearization point: a callback either precedes this seed or is
    // delivered to the newly installed subscriber. No second SDK callback.
    std::uint64_t Subscribe(void* Context, FSubscriber Callback, FFailure Fail)
    {
        if (!Context || !Callback || !Fail) return 0;
        std::lock_guard<std::mutex> Lock(Gate);
        if (Stopping || FAILED(Error) || NextSubscriber == UINT64_MAX) return 0;
        for (const auto& Item : Records)
            if (!Callback(Context, Item.second)) return 0;
        const auto Id = ++NextSubscriber;
        try { Subscribers.emplace(Id, FBinding{Context, Callback, Fail}); }
        catch (...) { FailLocked(E_OUTOFMEMORY, EGameInputFailureSite::WarmSubscribeAllocation); return 0; }
        return Id;
    }

    bool Unsubscribe(std::uint64_t Id)
    {
        std::lock_guard<std::mutex> Lock(Gate);
        return Id && Subscribers.erase(Id) == 1;
    }

    TComPtr<FApi> GetApi() const
    {
        std::lock_guard<std::mutex> Lock(Gate);
        return Stopping || FAILED(Error) ? TComPtr<FApi>{} : Api;
    }

    HRESULT GetError() const
    {
        std::lock_guard<std::mutex> Lock(Gate);
        return Error;
    }

    std::optional<FGameInputFailureDiagnostic> ReadFailureDiagnostic() const
    {
        std::lock_guard<std::mutex> Lock(Gate);
        return FirstFailure;
    }

    std::size_t SubscriberCount() const
    {
        std::lock_guard<std::mutex> Lock(Gate);
        return Subscribers.size();
    }

    bool Shutdown()
    {
        std::lock_guard<std::mutex> ControlLock(Control);
        {
            std::lock_guard<std::mutex> Lock(Gate);
            if (!Subscribers.empty()) return false; // Session workers must join first.
            Stopping = true;
        }
        // GameInput unregister fences callbacks. Never hold Gate while waiting.
        if (Registered && !Api->UnregisterCallback(Token)) return false;
        Registered = false;
        std::lock_guard<std::mutex> Lock(Gate);
        Records.clear();
        return true;
    }

private:
    struct FBinding { void* Context; FSubscriber Callback; FFailure Fail; bool Failed = false; };
    explicit FGameInputWarmCatalogue(FApi* InApi) : Api(InApi) {}
    void FailLocked(HRESULT Reason, EGameInputFailureSite Site = EGameInputFailureSite::WarmUnclassified,
        std::uint64_t Timestamp = 0, std::uint64_t Previous = 0, std::uint64_t Details = 0) noexcept
    {
        if (SUCCEEDED(Error))
        {
            Error = Reason;
            FirstFailure = FGameInputFailureDiagnostic{Site, static_cast<std::int32_t>(Reason), Timestamp, Previous, Details};
        }
        for (const auto& Subscriber : Subscribers)
            Subscriber.second.Fail(Subscriber.second.Context, Error);
    }

    static void CALLBACK OnDevice(GameInput::v3::GameInputCallbackToken, void* Context,
        FDevice* Device, std::uint64_t Timestamp,
        GameInput::v3::GameInputDeviceStatus Current,
        GameInput::v3::GameInputDeviceStatus)
    {
        auto& Self = *static_cast<FGameInputWarmCatalogue*>(Context);
        std::lock_guard<std::mutex> Lock(Self.Gate);
        if (Self.Stopping || FAILED(Self.Error)) return;
        try { Self.UpdateLocked(Device, Timestamp, Current); }
        catch (...) { Self.FailLocked(E_OUTOFMEMORY, EGameInputFailureSite::WarmCallbackException, Timestamp); }
    }

    void UpdateLocked(FDevice* Device, std::uint64_t Timestamp,
        GameInput::v3::GameInputDeviceStatus Current)
    {
        using namespace GameInput::v3;
        if (!Device) { FailLocked(E_POINTER, EGameInputFailureSite::WarmNullDevice, Timestamp); return; }
        const GameInputDeviceInfo* Info = nullptr;
        const HRESULT Status = Device->GetDeviceInfo(&Info);
        if (FAILED(Status) || !Info) { FailLocked(FAILED(Status) ? Status : E_UNEXPECTED, FAILED(Status) ? EGameInputFailureSite::WarmDeviceInfoFailure : EGameInputFailureSite::WarmDeviceInfoNull, Timestamp); return; }
        FRecord Next;
        static_assert(sizeof(Info->deviceId) == sizeof(Next.Id));
        std::memcpy(Next.Id.data(), &Info->deviceId, Next.Id.size());
        Next.Device = Device;
        Next.Timestamp = Timestamp;
        Next.Connected = (Current & GameInputDeviceConnected) != 0;
        Next.Kinds = static_cast<std::uint8_t>(((Info->supportedInput & GameInputKindKeyboard) ? 1 : 0)
            | ((Info->supportedInput & GameInputKindGamepad) ? 2 : 0));
        // GameInput serializes callbacks in chronological order. A tombstone
        // can be evicted only after a strictly later callback has crossed this
        // mailbox; a tied divergent event still finds the tombstone. A
        // violation of the SDK ordering guarantee fails closed.
        if (LastCallbackTimestamp && Timestamp < *LastCallbackTimestamp)
        { FailLocked(E_UNEXPECTED, EGameInputFailureSite::WarmRetrogradeCallback, Timestamp, *LastCallbackTimestamp); return; }
        if (!LastCallbackTimestamp || Timestamp > *LastCallbackTimestamp)
        {
            for (auto Old = Records.begin(); Old != Records.end();)
                Old = !Old->second.Connected && Old->second.Timestamp < Timestamp
                    ? Records.erase(Old) : std::next(Old);
            LastCallbackTimestamp = Timestamp;
        }
        const auto It = Records.find(Next.Id);
        if (It != Records.end())
        {
            const auto& Old = It->second;
            if (Timestamp < Old.Timestamp) return;
            if (!Next.Connected && Old.Device.Get() != Device) return;
            if (Timestamp == Old.Timestamp)
            {
                if (Old.Device.Get() == Device && Old.Connected == Next.Connected && Old.Kinds == Next.Kinds) return;
                const std::uint64_t Details = std::uint64_t(Old.Device.Get() != Device)
                    | (std::uint64_t(Old.Connected) << 1) | (std::uint64_t(Next.Connected) << 2)
                    | (std::uint64_t(Old.Kinds) << 8) | (std::uint64_t(Next.Kinds) << 16);
                FailLocked(E_UNEXPECTED, EGameInputFailureSite::WarmDivergentTie, Timestamp, Old.Timestamp, Details); return;
            }
        }
        else if (Records.size() == FDeviceDiscovery::Capacity)
        { FailLocked(E_OUTOFMEMORY, EGameInputFailureSite::WarmCapacity, Timestamp); return; }
        Records[Next.Id] = Next;
        // One session can exhaust its own 256-ID history while a newer
        // session remains valid. Quarantine that subscriber only; a catalogue
        // failure still fans out to every view through FailLocked.
        for (auto& Subscriber : Subscribers)
        {
            auto& Binding = Subscriber.second;
            if (!Binding.Failed && !Binding.Callback(Binding.Context, Next))
            { Binding.Failed = true; Binding.Fail(Binding.Context, E_UNEXPECTED); }
        }
    }

    TComPtr<FApi> Api;
    mutable std::mutex Gate;
    std::mutex Control;
    std::map<FDeviceId, FRecord> Records;
    std::map<std::uint64_t, FBinding> Subscribers;
    GameInput::v3::GameInputCallbackToken Token = 0;
    std::uint64_t NextSubscriber = 0;
    std::optional<std::uint64_t> LastCallbackTimestamp;
    HRESULT Error = S_OK;
    std::optional<FGameInputFailureDiagnostic> FirstFailure;
    bool Registered = false, Stopping = false;
};
}
