#include "DeviceInputHost.h"
#include <atomic>
#include <exception>
#include "HAL/PlatformTime.h"

#if PLATFORM_WINDOWS && !UE_SERVER
#include "Windows/AllowWindowsPlatformTypes.h"
#include "Windows/PreWindowsApi.h"
THIRD_PARTY_INCLUDES_START
#include "Windows/GameInputRawAcquisition.h"
#include "Windows/GameInputWarmContext.h"
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
    FNativeGameInputAcquisition(uint64_t Id, FActivityConfig Config, std::shared_ptr<FRawAcquisitionJournal> J,
        std::shared_ptr<Windows::FGameInputWarmContext> InWarm)
        : Producer(Id), Activity(Config), Journal(std::move(J)), Warm(std::move(InWarm)) {}
    EAcquisitionPumpResult Pump() override
    {
        if (Closed) return EAcquisitionPumpResult::Closed;
        if (!Source)
        {
#if !UE_BUILD_SHIPPING
            const double StartedAt = FPlatformTime::Seconds();
#endif
            if (Warm && IsInGameThread())
            { UE_LOG(LogTemp, Error, TEXT("Warm GameInput catalogue creation rejected on GameThread")); return EAcquisitionPumpResult::Rejected; }
            Microsoft::WRL::ComPtr<GameInput::v3::IGameInput> Api;
            auto Catalogue = Warm ? Warm->GetOrCreate(nullptr) : nullptr;
            const bool Reused = bool(Catalogue);
            if (!Catalogue)
            {
                const HRESULT Created = GameInput::v3::GameInputCreate(Api.GetAddressOf());
                if (FAILED(Created))
                { UE_LOG(LogTemp, Error, TEXT("Independent input GameInputCreate failed: 0x%08X"), uint32(Created)); return EAcquisitionPumpResult::Rejected; }
            }
#if !UE_BUILD_SHIPPING
            const double CreatedAt = FPlatformTime::Seconds();
#endif
            if (Warm && !Catalogue) Catalogue = Warm->GetOrCreate(Api.Get());
            auto Raw = Warm
                ? Windows::FGameInputSelectedSource::CreateRaw(std::move(Catalogue), Producer, Activity, true)
                : Windows::FGameInputSelectedSource::CreateRaw(Api.Get(), Producer, Activity, true);
            if (!Raw)
            { UE_LOG(LogTemp, Error, TEXT("Independent input GameInput discovery rejected")); return EAcquisitionPumpResult::Rejected; }
            Source=std::make_unique<Windows::FGameInputRawAcquisition>(std::move(Raw),Journal);
#if !UE_BUILD_SHIPPING
            UE_LOG(LogTemp, Display, TEXT("[InputAcquisitionStartup] game_input_create_ms=%.3f discovery_ms=%.3f catalogue_reused=%d"),
                1000.0 * (CreatedAt - StartedAt), 1000.0 * (FPlatformTime::Seconds() - CreatedAt), int32(Reused));
#endif
        }
		const auto Result = Source->Pump();
		if (const auto Diagnostic = Source->TakeNeutralizeDiagnostic())
		{
			UE_LOG(LogTemp, Warning, TEXT("Independent input acquisition neutralized; cause=%u poll_status=%u raw_reject=%u raw_error=0x%08X sink_reject=%u acquisition_tick=%llu session=%llu journal_serial=%llu barrier_before=%llu barrier_after=%llu"),
				unsigned(Diagnostic->Cause), unsigned(Diagnostic->PollStatus),
				unsigned(Diagnostic->RawPollReject), uint32(Diagnostic->RawError), unsigned(Diagnostic->SinkReject), Diagnostic->AcquisitionTick,
				Diagnostic->Barrier.Session, Diagnostic->Barrier.Serial,
				Diagnostic->Barrier.BarrierBefore, Diagnostic->Barrier.BarrierAfter);
		}
		return Result;
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
    std::shared_ptr<Windows::FGameInputWarmContext> Warm;
    std::unique_ptr<Windows::FGameInputRawAcquisition> Source;
    bool Closed=false;
};
}
#endif
FDeviceInputWarmContext::~FDeviceInputWarmContext()
{
    if (!Close()) std::terminate();
}

bool FDeviceInputWarmContext::Close()
{
#if PLATFORM_WINDOWS && !UE_SERVER
    auto Context = std::static_pointer_cast<Windows::FGameInputWarmContext>(Native);
    if (Context && !Context->Close()) return false;
#endif
    Native.reset();
    return true;
}

std::shared_ptr<FDeviceInputWarmContext> FDeviceInputWarmContext::Create()
{
#if PLATFORM_WINDOWS && !UE_SERVER
    auto Result = std::shared_ptr<FDeviceInputWarmContext>(new FDeviceInputWarmContext());
    Result->Native = std::make_shared<Windows::FGameInputWarmContext>();
    return Result;
#else
    return {};
#endif
}
std::shared_ptr<FDeviceInputWarmContext> CreateDeviceInputWarmContext()
{
    return FDeviceInputWarmContext::Create();
}
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
	if (!Config.Contract)
	{ UE_LOG(LogTemp, Error, TEXT("Independent input host rejected: contract missing")); return {}; }
	if (const auto Unsupported = FActionMapper::UnsupportedContract(*Config.Contract))
	{
		const auto* Action = Config.Contract->Find(Unsupported->Action);
		const FString Name = Action ? UTF8_TO_TCHAR(Action->Name.c_str()) : TEXT("<unknown>");
		if (Unsupported->Reason == FActionMapper::EUnsupportedContractReason::MouseButton)
		{
			UE_LOG(LogTemp, Error, TEXT("Independent input host rejected: contract action=%u name=%s unsupported_kind=MouseButton device_host=keyboard_gamepad_only"),
				unsigned(Unsupported->Action), *Name);
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("Independent input host rejected: contract action=%u name=%s unsupported_response_exponent=%.9g"),
				unsigned(Unsupported->Action), *Name, double(Unsupported->Exponent));
		}
		return {};
	}
	if (!FDeviceActivityPolicy::ValidConfig(Config.Activity))
	{ UE_LOG(LogTemp, Error, TEXT("Independent input host rejected: device_policy")); return {}; }
	if (Config.Cadence.count() <= 0 || Config.Cadence > std::chrono::seconds(1))
	{ UE_LOG(LogTemp, Error, TEXT("Independent input host rejected: cadence")); return {}; }
	if (Config.StartupTimeout.count() <= 0)
	{ UE_LOG(LogTemp, Error, TEXT("Independent input host rejected: startup_timeout")); return {}; }
	if (FPresentationInputScope::IsActive())
	{ UE_LOG(LogTemp, Error, TEXT("Independent input host rejected: presentation_scope")); return {}; }
	const auto Epoch = AllocateInputStreamEpoch();
	if (!Epoch.Value)
	{ UE_LOG(LogTemp, Error, TEXT("Independent input host rejected: epoch_exhausted")); return {}; }
	const FProducerIdentity Identity{EProducerKind::Device, Epoch.Value};
	auto Journal = std::make_shared<FRawAcquisitionJournal>(Epoch.Value, Config.FirstFrame);
	FSessionDescriptor Binding;
	Binding.Id = Binding.Epoch = Binding.Controller = Binding.Producer = Binding.Journal = Epoch.Value;
	Binding.First = Config.FirstFrame; Binding.Kind = EProducerContract::Device;
	Binding.Contract = Config.Contract; Binding.Processing = Config.Processing;
	// Producer construction is deferred to the physical registry worker.
	auto Host = FInputHostSession::CreateDescriptor(std::move(Binding));
	if (!Host)
	{ UE_LOG(LogTemp, Error, TEXT("Independent input host rejected: descriptor")); return {}; }
	Host->Journal = Journal;
	if (!Config.Controls.empty())
	{
		Host->Controls = FControlActionReader::Create(Config.Contract, Journal, Identity, Epoch.Value, Config.Controls);
		if (!Host->Controls)
		{ UE_LOG(LogTemp, Error, TEXT("Independent input control reader rejected")); return {}; }
	}
	auto Warm = Config.WarmContext
		? std::static_pointer_cast<Windows::FGameInputWarmContext>(Config.WarmContext->PlatformState()) : nullptr;
	if (Config.WarmContext && !Warm)
	{ UE_LOG(LogTemp, Error, TEXT("Independent input host rejected: warm context already closed")); return {}; }
	auto Acquisition = std::make_shared<FNativeGameInputAcquisition>(Identity.Id, Config.Activity, Journal, std::move(Warm));
	Host->Acquisition = std::make_unique<FInputAcquisitionWorker>(std::move(Acquisition));
	Host->AcquisitionStartupTimeout = Config.StartupTimeout;
	if (!Host->Acquisition->Start(Config.Cadence))
	{ UE_LOG(LogTemp, Error, TEXT("Independent input acquisition worker start rejected")); return {}; }
	return Host;
#else
	(void)Config;
	UE_LOG(LogTemp, Error, TEXT("Independent input host rejected: unsupported_platform"));
	return {};
#endif
}
}
