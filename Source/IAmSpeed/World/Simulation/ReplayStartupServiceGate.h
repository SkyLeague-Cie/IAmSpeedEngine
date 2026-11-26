#pragma once

#include <atomic>
#include <cstdint>

namespace Speed
{
// One incarnation. Identity is assigned on GT before the worker is created.
// Only the gate state changes while the worker is alive.
class FReplayStartupServiceGate final
{
    enum class EState : std::uint8_t { Unarmed, Held, Serviced, Released, FrameZeroCompleted, Cancelled };
    std::atomic<EState> State{EState::Unarmed};
    std::uint64_t Token = 0;
    std::uint64_t Generation = 0;

    EState Read() const noexcept { return State.load(std::memory_order_acquire); }

public:
    bool Arm(std::uint64_t InToken, std::uint64_t InGeneration) noexcept
    {
        if (!InToken || !InGeneration || Read() != EState::Unarmed) return false;
        Token = InToken;
        Generation = InGeneration;
        State.store(EState::Held, std::memory_order_release);
        return true;
    }

    bool MatchesToken(std::uint64_t InToken) const noexcept
    { return Read() != EState::Unarmed && Token == InToken; }

    bool MatchesGeneration(std::uint64_t InGeneration) const noexcept
    { return Read() != EState::Unarmed && Generation == InGeneration; }

    bool Holds(std::uint64_t InToken, std::uint64_t InGeneration) const noexcept
    {
        const auto Current = Read();
        return (Current == EState::Held || Current == EState::Serviced)
            && Token == InToken && Generation == InGeneration;
    }

    bool IsHolding() const noexcept
    {
        const auto Current = Read();
        return Current == EState::Held || Current == EState::Serviced;
    }

    bool IsCancelled() const noexcept { return Read() == EState::Cancelled; }

    bool AllowsWork() const noexcept
    {
        const auto Current = Read();
        return Current == EState::Unarmed || Current == EState::Released || Current == EState::FrameZeroCompleted;
    }

    bool BlocksPreFrameControl() const noexcept
    {
        const auto Current = Read();
        return Current != EState::Unarmed && Current != EState::FrameZeroCompleted;
    }

    void CompleteFrameZero() noexcept
    {
        auto Expected = EState::Released;
        State.compare_exchange_strong(Expected, EState::FrameZeroCompleted, std::memory_order_acq_rel);
    }

    void AcknowledgeService(std::uint64_t InGeneration) noexcept
    {
        if (!MatchesGeneration(InGeneration)) return;
        auto Expected = EState::Held;
        State.compare_exchange_strong(Expected, EState::Serviced, std::memory_order_acq_rel);
    }

    bool HasAcknowledged(std::uint64_t InToken, std::uint64_t InGeneration) const noexcept
    { return Read() == EState::Serviced && Token == InToken && Generation == InGeneration; }

    bool Release(std::uint64_t InToken, std::uint64_t InGeneration) noexcept
    {
        if (!HasAcknowledged(InToken, InGeneration)) return false;
        auto Expected = EState::Serviced;
        return State.compare_exchange_strong(Expected, EState::Released, std::memory_order_acq_rel);
    }

    void Invalidate() noexcept { State.store(EState::Cancelled, std::memory_order_release); }
};
}
