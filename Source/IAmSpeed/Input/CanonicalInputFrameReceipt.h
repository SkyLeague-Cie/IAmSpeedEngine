#pragma once
#include "InputSessionRegistry.h"

namespace Speed::Input::V2
{
// Ordinary adapter prepared for future driver integration and source tests. No world,
// native lifetime or other command channel is certified by an INPUT receipt.
inline bool BeginEmptyCanonicalInputFrameReceipt(FInputSessionRegistry& Registry, FFrameNumber N)
{
    return Registry.PrepareEmptyFrameReceipt(N) && Registry.InstallAll() && Registry.BeginAll();
}
inline bool ValidateCanonicalInputFrameReceipt(const FInputSessionRegistry& Registry, FFrameNumber N) noexcept
{
    const auto Installed = Registry.ReadInstalled();
    return Installed && Installed->Frame == N && Registry.ValidateComplete();
}
// Caller invokes ONLY after the real world returns Publication==Completed,
// before NotifyCanonicalFramePublished callbacks. No allocation or polling.
// A null result is terminal at that boundary, never a fictitious joint success.
inline std::shared_ptr<const FRegistryFrame> CompleteCanonicalInputFrameReceipt(
    FInputSessionRegistry& Registry, FFrameNumber N) noexcept
{
    if (!ValidateCanonicalInputFrameReceipt(Registry, N) || !Registry.CompleteAll()) return {};
    const auto Completed = Registry.ReadLatest();
    return Completed && Completed->Frame == N ? Completed : nullptr;
}
}
