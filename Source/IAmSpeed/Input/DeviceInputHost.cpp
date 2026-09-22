#include "DeviceInputHost.h"
#include <atomic>

#if PLATFORM_WINDOWS && !UE_SERVER
#include "Windows/AllowWindowsPlatformTypes.h"
#include "Windows/PreWindowsApi.h"
THIRD_PARTY_INCLUDES_START
#include "Windows/GameInputRawAcquisition.h"
THIRD_PARTY_INCLUDES_END
#include "Windows/PostWindowsApi.h"
#include "Windows/HideWindowsPlatformTypes.h"
#endif

namespace Speed::Input::V2
{
#if PLATFORM_WINDOWS && !UE_SERVER
namespace
{
// The host passes inert configuration. Native creation, polling and successful
// destruction all occur on the acquisition worker, independently of UE ticks.
class FNativeGameInputAcquisition final : public IInputAcquisition
{
public:
    FNativeGameInputAcquisition(uint64_t Id, FActivityConfig Config, std::shared_ptr<FRawAcquisitionJournal> J)
        : Producer(Id), Activity(Config), Journal(std::move(J)) {}
    EAcquisitionPumpResult Pump() override
    {
        if (Closed) return EAcquisitionPumpResult::Closed;
        if (!Source)
        {
            Microsoft::WRL::ComPtr<GameInput::v3::IGameInput> Api;
            if (FAILED(GameInput::v3::GameInputCreate(Api.GetAddressOf()))) return EAcquisitionPumpResult::Rejected;
            auto Raw=Windows::FGameInputSelectedSource::CreateRaw(Api.Get(),Producer,Activity);
            if (!Raw) return EAcquisitionPumpResult::Rejected;
            Source=std::make_unique<Windows::FGameInputRawAcquisition>(std::move(Raw),Journal);
        }
        return Source->Pump();
    }
    bool Close() override
    {
        Journal->Close();
        if (Source && !Source->Close()) return false;
        Source.reset(); Closed=true; return true;
    }
private:
    uint64_t Producer;
    FActivityConfig Activity;
    std::shared_ptr<FRawAcquisitionJournal> Journal;
    std::unique_ptr<Windows::FGameInputRawAcquisition> Source;
    bool Closed=false;
};
}
#endif
FStreamEpoch AllocateInputStreamEpoch()
{
	static std::atomic<std::uint64_t> Next{1};
	auto Value = Next.load(std::memory_order_relaxed);
	while (Value && Value != std::numeric_limits<std::uint64_t>::max())
		if (Next.compare_exchange_weak(Value, Value + 1, std::memory_order_relaxed)) return {Value};
	return {}; // Exhaustion must never reuse an earlier identity.
}

std::shared_ptr<FInputHostSession> CreateDeviceInputHost(const FDeviceInputHostConfig& Config)
{
#if PLATFORM_WINDOWS && !UE_SERVER
	if (!Config.Contract || !FActionMapper::SupportsContract(*Config.Contract) || !FDeviceActivityPolicy::ValidConfig(Config.Activity)
		|| Config.Cadence.count() <= 0 || Config.Cadence > std::chrono::seconds(1)
		|| Config.StartupTimeout.count() <= 0 || FPresentationInputScope::IsActive()) return {};
	const auto Epoch = AllocateInputStreamEpoch();
	if (!Epoch.Value) return {};
	const FProducerIdentity Identity{EProducerKind::Device, Epoch.Value};
	auto Journal = std::make_shared<FRawAcquisitionJournal>(Epoch.Value, Config.FirstFrame);
	FSessionDescriptor Binding;
	Binding.Id = Binding.Epoch = Binding.Controller = Binding.Producer = Binding.Journal = Epoch.Value;
	Binding.First = Config.FirstFrame; Binding.Kind = EProducerContract::Device;
	Binding.Contract = Config.Contract; Binding.Processing = Config.Processing;
	// Producer construction is deferred to the physical registry worker.
	auto Host = FInputHostSession::CreateDescriptor(std::move(Binding));
	if (!Host) return {};
	Host->Journal = Journal;
	if (!Config.Controls.empty())
	{
		Host->Controls = FControlActionReader::Create(Config.Contract, Journal, Identity, Epoch.Value, Config.Controls);
		if (!Host->Controls) return {};
	}
	auto Acquisition = std::make_shared<FNativeGameInputAcquisition>(Identity.Id, Config.Activity, Journal);
	Host->Acquisition = std::make_unique<FInputAcquisitionWorker>(std::move(Acquisition));
	if (!Host->Acquisition->Start(Config.Cadence)
		|| !Host->Acquisition->WaitForFirstPublication(Config.StartupTimeout)) return {};
	return Host;
#else
	(void)Config;
	return {};
#endif
}
}
