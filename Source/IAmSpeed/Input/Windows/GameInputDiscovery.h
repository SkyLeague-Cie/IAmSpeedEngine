#pragma once

#include "GameInputAcquisition.h"
#include "GameInputWarmCatalogue.h"
#include "../DeviceDiscovery.h"
#include <cstring>
#include <type_traits>

namespace Speed::Input::Windows
{
/** Optional discovery leaf. No runtime creation, UE integration or game policy.
 * Owns a values-only selected session. A future polling host obtains a ticket,
 * fetches a fresh reading for its retained device, then Submit validates it.
 * Do not feed already-produced frames from another producer into this session.
 */
class FGameInputDiscovery final : public IInputProducer
{
	using FApi = GameInput::v3::IGameInput;
	using FDevice = GameInput::v3::IGameInputDevice;
	template<class T> using TComPtr = Microsoft::WRL::ComPtr<T>;
public:
	struct FLease { FDeviceSelection Ticket; TComPtr<FDevice> Device; };
	static std::unique_ptr<FGameInputDiscovery> Create(FApi* Api, std::uint64_t ProducerId,
		const std::array<bool, ActionCount>& Digital)
	{
		using namespace GameInput::v3;
		if (!Api || !ProducerId || FPresentationInputScope::IsActive()) return nullptr;
		auto Result = std::unique_ptr<FGameInputDiscovery>(new FGameInputDiscovery(Api, ProducerId, Digital));
		// All kinds declared by the inspected v3 SDK, including unsupported kinds
		// retained in the catalogue. No implicit aggregate keyboard is created.
		const auto Kinds = GameInputKindRawDeviceReport | GameInputKindController | GameInputKindKeyboard
			| GameInputKindMouse | GameInputKindSensors | GameInputKindArcadeStick
			| GameInputKindFlightStick | GameInputKindGamepad | GameInputKindRacingWheel;
		if (FAILED(Api->RegisterDeviceCallback(nullptr, Kinds, GameInputDeviceConnected,
			GameInputBlockingEnumeration, Result.get(), &OnDevice, &Result->Token))) return nullptr;
		Result->Registered = true;
		return Result; // Enumeration failure is observable via GetLastError().
	}
	static std::unique_ptr<FGameInputDiscovery> Create(
		std::shared_ptr<FGameInputWarmCatalogue> Warm, std::uint64_t ProducerId,
		const std::array<bool, ActionCount>& Digital)
	{
		if (!Warm || !ProducerId || FPresentationInputScope::IsActive()) return nullptr;
		auto Api = Warm->GetApi();
		if (!Api) return nullptr;
		auto Result = std::unique_ptr<FGameInputDiscovery>(new FGameInputDiscovery(Api.Get(), ProducerId, Digital));
		Result->Warm = std::move(Warm);
		Result->WarmSubscription = Result->Warm->Subscribe(Result.get(), &OnWarmDevice, &OnWarmFailure);
		return Result->WarmSubscription ? std::move(Result) : nullptr;
	}
	~FGameInputDiscovery() override { if (!Shutdown()) std::terminate(); }
	bool Shutdown()
	{
		std::lock_guard<std::mutex> ControlLock(Control);
		{
			std::lock_guard<std::mutex> Lock(Mailbox);
			Stopping = true; Catalogue.Fail();
		}
		// Callback only takes Mailbox. Never unregister with Mailbox held.
		if (WarmSubscription && !Warm->Unsubscribe(WarmSubscription)) return false;
		WarmSubscription = 0;
		if (Registered && !Api->UnregisterCallback(Token)) return false;
		Registered = false;
		std::lock_guard<std::mutex> Lock(Mailbox);
		Records.clear();
		return true;
	}
	std::vector<FDiscoveredDevice> Snapshot() const
	{
		std::lock_guard<std::mutex> Lock(Mailbox);
		return Catalogue.Snapshot();
	}
	bool Select(std::optional<std::pair<FDeviceId, EDeviceKind>> Request)
	{
		std::lock_guard<std::mutex> Lock(Mailbox);
		return !Stopping && Catalogue.Select(Request);
	}
	std::optional<FLease> AcquireSelected() const
	{
		std::lock_guard<std::mutex> Lock(Mailbox);
		if (Stopping) return std::nullopt;
		const auto Selection = Catalogue.Selected();
		if (!Selection) return std::nullopt;
		const auto It = Records.find(Selection->Device.Id);
		if (It == Records.end()) return std::nullopt;
		return FLease{*Selection, It->second.Device};
	}
	// Raw activity observation only: generation zero cannot submit to the session.
	std::optional<FLease> AcquireDevice(const FDeviceId& Id, EDeviceKind Kind) const
	{
		std::lock_guard<std::mutex> Lock(Mailbox);
		if (Stopping || FAILED(Error)) return std::nullopt;
		const auto It = Records.find(Id);
		if (It == Records.end() || !It->second.Connected
			|| !(It->second.Kinds & static_cast<std::uint8_t>(Kind))) return std::nullopt;
		return FLease{{{Id, It->second.Timestamp, It->second.Kinds, true}, Kind, 0}, It->second.Device};
	}
	bool Submit(const FLease& Lease, std::uint64_t Sequence, const FActionValues& Values)
	{
		std::lock_guard<std::mutex> Lock(Mailbox);
		if (Stopping) return false;
		const auto It = Records.find(Lease.Ticket.Device.Id);
		return It != Records.end() && It->second.Device.Get() == Lease.Device.Get()
			&& Catalogue.Submit(Lease.Ticket, Sequence, Values);
	}
	// Linearization point for a raw batch. The sink may only install prepared
	// values: no allocation, callbacks into discovery, or gameplay execution.
	// Holding Mailbox prevents hotplug from invalidating the ticket mid-install.
	template<class TCommit> bool CommitRaw(const std::optional<FLease>& Lease, TCommit&& Commit)
	{
		static_assert(std::is_nothrow_invocable_r_v<bool, TCommit>, "raw installation must return acceptance without throwing");
		std::lock_guard<std::mutex> Lock(Mailbox);
		if (Stopping || FAILED(Error) || Catalogue.IsFailed()) return false;
		const auto Selected = Catalogue.Selected();
		if (bool(Selected) != bool(Lease)) return false;
		if (Lease)
		{
			const auto& A = *Selected; const auto& B = Lease->Ticket;
			const auto It = Records.find(B.Device.Id);
			if (A.Generation != B.Generation || A.Device.Id != B.Device.Id
				|| A.Device.Revision != B.Device.Revision || A.Kind != B.Kind
				|| It == Records.end() || It->second.Device.Get() != Lease->Device.Get()) return false;
		}
		return Commit();
	}
	bool SetPaused(bool Paused)
	{
		std::lock_guard<std::mutex> Lock(Mailbox);
		return !Stopping && Catalogue.SetPaused(Paused);
	}
	bool Resynchronize(const FLease& Lease)
	{
		std::lock_guard<std::mutex> Lock(Mailbox);
		return !Stopping && Catalogue.Resynchronize(Lease.Ticket);
	}
	void FailAcquisition(HRESULT Reason)
	{
		std::lock_guard<std::mutex> Lock(Mailbox);
		FailLocked(FAILED(Reason) ? Reason : E_UNEXPECTED);
	}
	std::optional<FInputFrame> Produce(FFrameNumber Frame) override
	{
		std::lock_guard<std::mutex> Lock(Mailbox);
		return Stopping ? std::nullopt : Catalogue.Produce(Frame);
	}
	bool Skip(FFrameNumber Frame) override
	{
		std::lock_guard<std::mutex> Lock(Mailbox);
		return !Stopping && Catalogue.Skip(Frame);
	}
	HRESULT GetLastError() const { std::lock_guard<std::mutex> Lock(Mailbox); return Error; }
    FGameInputFailureDiagnostics ReadFailureDiagnostics() const
    {
        FGameInputFailureDiagnostics Result;
        { std::lock_guard<std::mutex> Lock(Mailbox); Result.Discovery = FirstFailure; }
        // Warm callbacks take Warm Gate then Mailbox. Release Mailbox before
        // reading Warm diagnostics to preserve the existing lock order.
        if (Warm) Result.Warm = Warm->ReadFailureDiagnostic();
        return Result;
    }
private:
	struct FRecord { TComPtr<FDevice> Device; std::uint64_t Timestamp; bool Connected; std::uint8_t Kinds; };
	FGameInputDiscovery(FApi* InApi, std::uint64_t Id, const std::array<bool, ActionCount>& Digital)
		: Api(InApi), Catalogue(Id, Digital) {}
	void FailLocked(HRESULT Reason, EGameInputFailureSite Site = EGameInputFailureSite::DiscoveryExternalAcquisition,
        std::uint64_t Timestamp = 0, std::uint64_t Previous = 0, std::uint64_t Details = 0)
    {
        if (SUCCEEDED(Error))
        {
            Error = Reason;
            FirstFailure = FGameInputFailureDiagnostic{Site, static_cast<std::int32_t>(Reason), Timestamp, Previous, Details};
        }
        Catalogue.Fail();
    }
	static void CALLBACK OnDevice(GameInput::v3::GameInputCallbackToken, void* Context, FDevice* Device,
		std::uint64_t Timestamp, GameInput::v3::GameInputDeviceStatus Current, GameInput::v3::GameInputDeviceStatus)
	{
		auto& Self = *static_cast<FGameInputDiscovery*>(Context);
		std::lock_guard<std::mutex> Lock(Self.Mailbox);
		if (Self.Stopping || FAILED(Self.Error)) return;
		try { Self.UpdateLocked(Device, Timestamp, Current); }
		catch (...) { Self.FailLocked(E_OUTOFMEMORY, EGameInputFailureSite::DiscoveryCallbackException, Timestamp); } // Never throw across SDK ABI.
	}
	static bool OnWarmDevice(void* Context, const FGameInputWarmCatalogue::FRecord& Record) noexcept
	{
		auto& Self = *static_cast<FGameInputDiscovery*>(Context);
		std::lock_guard<std::mutex> Lock(Self.Mailbox);
		if (Self.Stopping) return true;
		if (FAILED(Self.Error)) return false;
		try { Self.UpdateLocked(Record.Device.Get(), Record.Timestamp,
			Record.Connected ? GameInput::v3::GameInputDeviceConnected
				: static_cast<GameInput::v3::GameInputDeviceStatus>(0)); }
		catch (...) { Self.FailLocked(E_OUTOFMEMORY, EGameInputFailureSite::DiscoveryWarmCallbackException, Record.Timestamp); }
		return SUCCEEDED(Self.Error);
	}
	static void OnWarmFailure(void* Context, HRESULT Reason) noexcept
	{
		auto& Self = *static_cast<FGameInputDiscovery*>(Context);
		std::lock_guard<std::mutex> Lock(Self.Mailbox);
		if (!Self.Stopping) Self.FailLocked(Reason, EGameInputFailureSite::DiscoveryWarmPropagation);
	}
	void UpdateLocked(FDevice* Device, std::uint64_t Timestamp, GameInput::v3::GameInputDeviceStatus Current)
	{
		using namespace GameInput::v3;
		if (!Device) { FailLocked(E_POINTER, EGameInputFailureSite::DiscoveryNullDevice, Timestamp); return; }
		const GameInputDeviceInfo* Info = nullptr;
		const auto Status = Device->GetDeviceInfo(&Info);
		if (FAILED(Status) || !Info) { FailLocked(FAILED(Status) ? Status : E_UNEXPECTED, FAILED(Status) ? EGameInputFailureSite::DiscoveryDeviceInfoFailure : EGameInputFailureSite::DiscoveryDeviceInfoNull, Timestamp); return; }
		FDeviceId Id{};
		static_assert(sizeof(Info->deviceId) == sizeof(Id), "App-local ID must fit losslessly");
		std::memcpy(Id.data(), &Info->deviceId, Id.size());
		const bool Connected = (Current & GameInputDeviceConnected) != 0;
		const auto Kinds = static_cast<std::uint8_t>(((Info->supportedInput & GameInputKindKeyboard) ? 1 : 0)
			| ((Info->supportedInput & GameInputKindGamepad) ? 2 : 0));
		const auto It = Records.find(Id);
		if (It != Records.end())
		{
			const auto& Old = It->second;
			if (Timestamp < Old.Timestamp) return;
			// An obsolete object's delayed disconnect cannot remove its replacement.
			if (!Connected && Old.Device.Get() != Device) return;
			if (Timestamp == Old.Timestamp)
			{
				if (Old.Device.Get() == Device && Old.Connected == Connected && Old.Kinds == Kinds) return;
				const std::uint64_t Details = std::uint64_t(Old.Device.Get() != Device)
                    | (std::uint64_t(Old.Connected) << 1) | (std::uint64_t(Connected) << 2)
                    | (std::uint64_t(Old.Kinds) << 8) | (std::uint64_t(Kinds) << 16);
                FailLocked(E_UNEXPECTED, EGameInputFailureSite::DiscoveryDivergentTie, Timestamp, Old.Timestamp, Details); return; // No invented ordering for a timestamp tie.
			}
		}
		else if (Records.size() == FDeviceDiscovery::Capacity) { FailLocked(E_OUTOFMEMORY, EGameInputFailureSite::DiscoveryCapacity, Timestamp); return; }
		const auto Result = Catalogue.Update({Id, Timestamp, Kinds, Connected});
		if (Result == EDiscoveryUpdate::Failed) { FailLocked(E_UNEXPECTED, EGameInputFailureSite::DiscoveryCatalogueUpdate, Timestamp, 0, static_cast<std::uint64_t>(Result)); return; }
		Records[Id] = FRecord{TComPtr<FDevice>(Device), Timestamp, Connected, Kinds};
	}
	TComPtr<FApi> Api;
	std::shared_ptr<FGameInputWarmCatalogue> Warm;
	std::uint64_t WarmSubscription = 0;
	mutable std::mutex Mailbox;
	std::mutex Control;
	FDeviceDiscovery Catalogue;
	std::map<FDeviceId, FRecord> Records;
	GameInput::v3::GameInputCallbackToken Token = 0;
	HRESULT Error = S_OK;
    std::optional<FGameInputFailureDiagnostic> FirstFailure;
	bool Registered = false;
	bool Stopping = false;
};
}
