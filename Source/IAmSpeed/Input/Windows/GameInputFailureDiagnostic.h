#pragma once
// Values only. Writers use the existing source/catalogue locks. No admission,
// callback ordering, device selection, polling or lifecycle behavior changes.
#include <cstdint>
#include <optional>
namespace Speed::Input::Windows
{
enum class EGameInputFailureSite : std::uint8_t
{
    None, WarmUnclassified, WarmSubscribeAllocation, WarmCallbackException,
    WarmNullDevice, WarmDeviceInfoFailure, WarmDeviceInfoNull,
    WarmRetrogradeCallback, WarmDivergentTie, WarmCapacity,
    DiscoveryExternalAcquisition, DiscoveryCallbackException,
    DiscoveryWarmCallbackException, DiscoveryWarmPropagation,
    DiscoveryNullDevice, DiscoveryDeviceInfoFailure, DiscoveryDeviceInfoNull,
    DiscoveryDivergentTie, DiscoveryCapacity, DiscoveryCatalogueUpdate
};
struct FGameInputFailureDiagnostic
{
    EGameInputFailureSite Site = EGameInputFailureSite::None;
    std::int32_t HResult = 0;
    std::uint64_t EventTimestamp = 0, PreviousTimestamp = 0, Details = 0;
};
struct FGameInputFailureDiagnostics
{
    std::optional<FGameInputFailureDiagnostic> Discovery, Warm;
};
}
