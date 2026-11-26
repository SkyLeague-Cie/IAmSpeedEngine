#pragma once

#include <atomic>
#include <cstdint>

namespace Speed
{
// The original worker and GT teardown race on a single admission word.
// Closing before the worker claims publication prevents any finalization.
// Closing after the claim requires joining the worker before storage removal.
class FExactRemoteRoundFence final
{
    static constexpr std::uint8_t Closed = 1;
    static constexpr std::uint8_t Publishing = 2;
    std::atomic<std::uint8_t> State{0};

public:
    bool IsClosed() const noexcept { return (State.load(std::memory_order_acquire) & Closed) != 0; }
    bool HoldsPublication() const noexcept { return (State.load(std::memory_order_acquire) & Publishing) != 0; }
    bool TryBeginPublication() noexcept
    {
        std::uint8_t Expected = 0;
        return State.compare_exchange_strong(Expected, Publishing, std::memory_order_acq_rel);
    }
    void FinishPublication() noexcept { State.fetch_and(static_cast<std::uint8_t>(~Publishing), std::memory_order_release); }
    void Close() noexcept { State.fetch_or(Closed, std::memory_order_acq_rel); }
};
}
