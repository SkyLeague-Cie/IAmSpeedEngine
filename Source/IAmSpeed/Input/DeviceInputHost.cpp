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
	if (!Config.Contract || !FDeviceActivityPolicy::ValidConfig(Config.Activity)
		|| Config.Cadence.count() <= 0 || Config.Cadence > std::chrono::seconds(1)
		|| Config.StartupTimeout.count() <= 0 || FPresentationInputScope::IsActive()) return {};
	const auto Epoch = AllocateInputStreamEpoch();
	if (!Epoch.Value) return {};
	const FProducerIdentity Identity{EProducerKind::Device, Epoch.Value};
	Microsoft::WRL::ComPtr<GameInput::v3::IGameInput> Api;
	if (FAILED(GameInput::v3::GameInputCreate(Api.GetAddressOf()))) return {};
	auto Source = Windows::FGameInputSelectedSource::CreateRaw(Api.Get(), Identity.Id, Config.Activity);
	if (!Source) return {};
	auto Journal = std::make_shared<FRawAcquisitionJournal>(Epoch.Value, Config.FirstFrame);
	auto Producer = FDeviceInputProducer::Create(Journal, Config.Contract, Epoch, Identity, Config.FirstFrame);
	if (!Producer) return {};
	auto Host = FInputHostSession::Create(std::move(Producer), Epoch, Identity, Config.FirstFrame);
	if (!Host) return {};
	Host->Journal = Journal;
	if (!Config.Controls.empty())
	{
		Host->Controls = FControlActionReader::Create(Config.Contract, Journal, Identity, Epoch.Value, Config.Controls);
		if (!Host->Controls) return {};
	}
	auto Acquisition = std::make_shared<Windows::FGameInputRawAcquisition>(std::move(Source), Journal);
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
