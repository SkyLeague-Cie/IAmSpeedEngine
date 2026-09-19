#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>

namespace Speed::Lifecycle
{
enum class ESessionState : std::uint8_t
{ Configured, Running, StopRequested, Draining, Stopped, ThreadExited, Reaped };
enum class EParticipantState : std::uint8_t
{ Active, RetireRequested, Draining, Detached };
enum class EReservationPhase : std::uint8_t { None, Reserved, Committed, Published };
enum class EOutcome : std::uint8_t { Completed, Aborted };

// A portable lifetime witness, NOT a platform module pin or an unload service.
struct FCodeLease final { std::uint64_t Id = 0; };

class FSessionKey final
{
public:
    FSessionKey() = default;
    std::uint64_t GetEpoch() const noexcept { return Epoch; }
private:
    template<std::size_t> friend class TSessionLifecycle;
    std::uint64_t Epoch = 0;
    std::shared_ptr<const std::uint8_t> Identity;
};

struct FParticipantHandle
{
    FSessionKey Session;
    std::size_t Slot = 0;
    std::uint64_t Generation = 0;
};

class FReservationToken final
{
public:
    FReservationToken() = default;
private:
    template<std::size_t> friend class TSessionLifecycle;
    FSessionKey Session;
    std::uint64_t Serial = 0;
};

struct FResolvedReservation
{
    std::uint64_t Serial = 0;
    EOutcome Outcome = EOutcome::Aborted;
};

// One serialized protocol owner calls every method except RequestStop().
// RequestStop may race that owner: its latch and frame admission share one CAS
// word. There is no worker, callback dispatcher, queue, or Unreal integration.
template<std::size_t Capacity = 8>
class TSessionLifecycle final
{
    static_assert(Capacity > 0 && Capacity <= 64, "participant mask capacity");
    static_assert(std::atomic<std::uint32_t>::is_always_lock_free, "stop latch must be lock-free");
public:
    explicit TSessionLifecycle(std::uint64_t Epoch, std::shared_ptr<const FCodeLease> CodeLease)
        : Lease(std::move(CodeLease))
    {
        if (!Epoch || !Lease || !Lease->Id) throw std::invalid_argument("epoch and code lease required");
        Key.Epoch = Epoch;
        Key.Identity = std::make_shared<const std::uint8_t>(std::uint8_t{0});
    }
    TSessionLifecycle(const TSessionLifecycle&) = delete;
    TSessionLifecycle& operator=(const TSessionLifecycle&) = delete;

    // Early destruction of an activated protocol is an owner bug. Releasing
    // its lease implicitly would falsely authorize unloading live code.
    ~TSessionLifecycle()
    {
        if (GetState() != ESessionState::Configured && Phase != ESessionState::Reaped)
            std::terminate();
    }

    FSessionKey GetKey() const noexcept { return Key; }
    ESessionState GetState() const noexcept
    {
        if ((Phase == ESessionState::Configured || Phase == ESessionState::Running) && StopRequested())
            return ESessionState::StopRequested;
        return Phase;
    }
    bool IsPaused() const noexcept { return Paused; }
    EReservationPhase GetReservationPhase() const noexcept { return ReservationPhase; }
    std::optional<FResolvedReservation> LastOutcome() const noexcept { return Outcome; }
    bool HoldsCodeLease() const noexcept { return bool(Lease); }

    // Configuration is sealed by Start. Fixed slots can be reused only before
    // Start and after an explicit Detached acknowledgment, with a new generation.
    std::optional<FParticipantHandle> RegisterParticipant() noexcept
    {
        if (Phase != ESessionState::Configured || StopRequested()) return {};
        for (std::size_t I = 0; I < Capacity; ++I)
        {
            auto& P = Participants[I];
            if ((!P.Used || P.State == EParticipantState::Detached) &&
                P.Generation != std::numeric_limits<std::uint64_t>::max())
            {
                ++P.Generation;
                P.Used = true;
                P.State = EParticipantState::Active;
                return FParticipantHandle{Key, I, P.Generation};
            }
        }
        return {}; // Capacity exhaustion never consumes a stop/retire slot.
    }

    bool Start() noexcept
    {
        if (Phase != ESessionState::Configured || StopRequested()) return false;
        Phase = ESessionState::Running;
        return true;
    }
    bool SetPaused(bool Value) noexcept
    {
        if (GetState() != ESessionState::Running) return false;
        Paused = Value;
        return true; // A pause request is not a frame/stop acknowledgment.
    }

    // Dedicated monotonic flag: no allocation, queue capacity, wait or callback.
    bool RequestStop() noexcept
    {
        return (Admission.fetch_or(StopBit, std::memory_order_acq_rel) & StopBit) == 0;
    }
    bool BeginStopDrain(const FSessionKey& Session) noexcept
    {
        if (!Matches(Session) || GetState() != ESessionState::StopRequested) return false;
        Phase = ESessionState::Draining;
        for (auto& P : Participants)
            if (P.Used && P.State == EParticipantState::Active)
                P.State = EParticipantState::RetireRequested;
        return true;
    }

    bool RequestRetire(const FParticipantHandle& Handle) noexcept
    {
        auto* P = Find(Handle);
        if (!P || P->State != EParticipantState::Active || IsTerminal()) return false;
        P->State = EParticipantState::RetireRequested;
        return true;
    }
    bool BeginParticipantDrain(const FParticipantHandle& Handle) noexcept
    {
        auto* P = Find(Handle);
        if (!P || P->State != EParticipantState::RetireRequested || IsTerminal()) return false;
        P->State = EParticipantState::Draining;
        return true;
    }
    bool AcknowledgeDetached(const FParticipantHandle& Handle) noexcept
    {
        auto* P = Find(Handle);
        if (!P || P->State != EParticipantState::Draining ||
            (ReservationPhase != EReservationPhase::None && (ReservedMask & Bit(Handle.Slot)))) return false;
        P->State = EParticipantState::Detached;
        return true;
    }
    std::optional<EParticipantState> ParticipantState(const FParticipantHandle& Handle) const noexcept
    {
        if (!Matches(Handle.Session) || Handle.Slot >= Capacity) return {};
        const auto& P = Participants[Handle.Slot];
        if (!P.Used || P.Generation != Handle.Generation) return {};
        return P.State;
    }

    std::optional<FReservationToken> TryAdmit() noexcept
    {
        if (Phase != ESessionState::Running || Paused ||
            ReservationPhase != EReservationPhase::None ||
            Serial == std::numeric_limits<std::uint64_t>::max()) return {};
        std::uint64_t Mask = 0;
        for (std::size_t I = 0; I < Capacity; ++I)
            if (Participants[I].Used && Participants[I].State == EParticipantState::Active) Mask |= Bit(I);
        if (!Mask) return {};
        std::uint32_t Expected = 0;
        if (!Admission.compare_exchange_strong(Expected, ReservedBit, std::memory_order_acq_rel)) return {};
        // The CAS linearizes admission before or after the stop request. A
        // request arriving now preserves this admitted reservation for draining.
        ++Serial;
        ReservedMask = Mask;
        ReservationPhase = EReservationPhase::Reserved;
        FReservationToken Token;
        Token.Session = Key;
        Token.Serial = Serial;
        return Token;
    }
    bool MarkCommitted(const FReservationToken& Token) noexcept
    {
        if (!Owns(Token) || ReservationPhase != EReservationPhase::Reserved) return false;
        ReservationPhase = EReservationPhase::Committed;
        return true;
    }
    bool MarkPublished(const FReservationToken& Token) noexcept
    {
        if (!Owns(Token) || ReservationPhase != EReservationPhase::Committed) return false;
        ReservationPhase = EReservationPhase::Published;
        return true;
    }
    bool Complete(const FReservationToken& Token) noexcept
    {
        if (!Owns(Token) || ReservationPhase != EReservationPhase::Published) return false;
        Resolve(EOutcome::Completed);
        return true;
    }
    bool Abort(const FReservationToken& Token) noexcept
    {
        if (!Owns(Token) || ReservationPhase == EReservationPhase::Published) return false;
        RequestStop(); // Application may already have changed state: never continue after an abort.
        Resolve(EOutcome::Aborted);
        return true;
    }
    bool AcknowledgeStopped(const FSessionKey& Session) noexcept
    {
        if (!Matches(Session) || Phase != ESessionState::Draining ||
            ReservationPhase != EReservationPhase::None) return false;
        for (const auto& P : Participants)
            if (P.Used && P.State != EParticipantState::Detached) return false;
        Phase = ESessionState::Stopped;
        return true;
    }
    bool AcknowledgeThreadExited(const FSessionKey& Session) noexcept
    {
        if (!Matches(Session) || Phase != ESessionState::Stopped) return false;
        Phase = ESessionState::ThreadExited;
        return true;
    }
    bool Reap(const FSessionKey& Session) noexcept
    {
        if (!Matches(Session) || Phase != ESessionState::ThreadExited) return false;
        Phase = ESessionState::Reaped;
        Lease.reset();
        return true;
    }

private:
    struct FParticipant
    {
        std::uint64_t Generation = 0;
        EParticipantState State = EParticipantState::Detached;
        bool Used = false;
    };
    static constexpr std::uint32_t StopBit = 1;
    static constexpr std::uint32_t ReservedBit = 2;
    static constexpr std::uint64_t Bit(std::size_t I) noexcept { return std::uint64_t{1} << I; }
    bool StopRequested() const noexcept { return (Admission.load(std::memory_order_acquire) & StopBit) != 0; }
    bool Matches(const FSessionKey& Session) const noexcept
    { return Session.Epoch == Key.Epoch && Session.Identity && Session.Identity == Key.Identity; }
    bool IsTerminal() const noexcept
    { return Phase == ESessionState::Stopped || Phase == ESessionState::ThreadExited || Phase == ESessionState::Reaped; }
    FParticipant* Find(const FParticipantHandle& Handle) noexcept
    {
        if (!Matches(Handle.Session) || Handle.Slot >= Capacity) return nullptr;
        auto& P = Participants[Handle.Slot];
        return P.Used && P.Generation == Handle.Generation ? &P : nullptr;
    }
    bool Owns(const FReservationToken& Token) const noexcept
    { return Matches(Token.Session) && Token.Serial == Serial && ReservationPhase != EReservationPhase::None; }
    void Resolve(EOutcome Result) noexcept
    {
        Outcome = FResolvedReservation{Serial, Result};
        ReservationPhase = EReservationPhase::None;
        ReservedMask = 0;
        Admission.fetch_and(~ReservedBit, std::memory_order_acq_rel);
    }

    FSessionKey Key;
    std::shared_ptr<const FCodeLease> Lease;
    std::array<FParticipant, Capacity> Participants{};
    std::atomic<std::uint32_t> Admission{0};
    ESessionState Phase = ESessionState::Configured;
    EReservationPhase ReservationPhase = EReservationPhase::None;
    std::uint64_t Serial = 0;
    std::uint64_t ReservedMask = 0;
    std::optional<FResolvedReservation> Outcome;
    bool Paused = false;
};
}
