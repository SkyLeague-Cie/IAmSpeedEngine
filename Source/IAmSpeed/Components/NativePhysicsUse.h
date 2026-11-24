#pragma once

#include <atomic>
#include <exception>

namespace Speed
{
// The only reversible state is the private, callback-free family setup.
// Neither a pending seal nor a missing stable ID is teardown evidence.
class FNativePhysicsUse final
{
public:
    bool IsPresentationOnly() const noexcept
    {
        return State.load(std::memory_order_acquire) == EState::Presentation;
    }

    bool TryUseNative() noexcept
    {
        EState Expected = EState::Fresh;
        return State.compare_exchange_strong(Expected, EState::Native, std::memory_order_acq_rel)
            || Expected == EState::Native;
    }

    bool TryBeginPresentationSeal() noexcept
    {
        EState Expected = EState::Fresh;
        return State.compare_exchange_strong(Expected, EState::Sealing, std::memory_order_acq_rel);
    }

    void FinishPresentationSeal(bool bSealed) noexcept
    {
        EState Expected = EState::Sealing;
        if (!State.compare_exchange_strong(Expected, bSealed ? EState::Presentation : EState::Fresh,
            std::memory_order_acq_rel)) std::terminate();
    }

private:
    enum class EState : unsigned char { Fresh, Native, Sealing, Presentation };
    std::atomic<EState> State{EState::Fresh};
};
}
