#pragma once

#include <atomic>
#include <cstdint>
#include <exception>
#include <thread>

namespace Speed
{
// Per-world fence for an exclusive round boundary. A scoped borrower must
// retain its scope until the last use of every reference it obtains. A legacy
// raw access without such a scope disqualifies this world for round reset.
// This does not grant UObject lifetime or serialize ordinary simulation access.
class FWorldBorrowGate final
{
    static constexpr std::uint32_t Closed = std::uint32_t{1} << 31;
    static constexpr std::uint32_t Untracked = std::uint32_t{1} << 30;
    static constexpr std::uint32_t CountMask = Untracked - 1;
    mutable std::atomic<std::uint32_t> State{0};

public:
    class FReservation;
    class FBorrow final
    {
        friend class FWorldBorrowGate;
        const FWorldBorrowGate* Gate = nullptr;
        FBorrow* Previous = nullptr;
        bool Counted = false;
        inline static thread_local FBorrow* Current = nullptr;

    public:
        // Only the exact reservation owner can admit a nested scope for its
        // own canonical work. Other lanes still see the storage as closed.
        explicit FBorrow(const FReservation&) noexcept;
        explicit FBorrow(const FWorldBorrowGate& InGate) noexcept
        {
            if (InGate.HasScopedBorrow())
            {
                Enter(InGate);
                return;
            }
            auto Seen = InGate.State.load(std::memory_order_relaxed);
            while (!(Seen & Closed) && (Seen & CountMask) != CountMask)
            {
                if (InGate.State.compare_exchange_weak(Seen, Seen + 1,
                    std::memory_order_acq_rel, std::memory_order_relaxed))
                {
                    Counted = true;
                    Enter(InGate);
                    break;
                }
            }
        }
        FBorrow(const FBorrow&) = delete;
        FBorrow& operator=(const FBorrow&) = delete;
        ~FBorrow() noexcept
        {
            if (!Gate) return;
            if (Current != this) std::terminate();
            Current = Previous;
            if (Counted) Gate->State.fetch_sub(1, std::memory_order_release);
        }
        explicit operator bool() const noexcept { return Gate != nullptr; }
    private:
        void Enter(const FWorldBorrowGate& InGate) noexcept
        {
            Gate = &InGate;
            Previous = Current;
            Current = this;
        }
    };

    class FReservation final
    {
        friend class FWorldBorrowGate;
        friend class FBorrow;
        const FWorldBorrowGate* Gate = nullptr;
        const std::thread::id Owner = std::this_thread::get_id();
        FReservation* Previous = nullptr;
        inline static thread_local FReservation* Current = nullptr;

    public:
        explicit FReservation(const FWorldBorrowGate& InGate) noexcept
        {
            std::uint32_t Expected = 0;
            if (InGate.State.compare_exchange_strong(Expected, Closed, std::memory_order_acq_rel))
            {
                Gate = &InGate;
                Previous = Current;
                Current = this;
            }
        }
        FReservation(const FReservation&) = delete;
        FReservation& operator=(const FReservation&) = delete;
        ~FReservation() noexcept
        {
            if (!Gate) return;
            if (Current != this || Gate->HasScopedBorrow()) std::terminate();
            Current = Previous;
            Gate->State.store(0, std::memory_order_release);
        }
        bool Holds(const FWorldBorrowGate& InGate) const noexcept
        { return Gate == &InGate && Owner == std::this_thread::get_id(); }
    };

    bool AdmitReference() const noexcept
    {
        if (HasScopedBorrow()) return true;

        auto Seen = State.load(std::memory_order_relaxed);
        for (;;)
        {
            if (Seen & Closed) return false;
            if (Seen & Untracked) return true;
            if (State.compare_exchange_weak(Seen, Seen | Untracked,
                std::memory_order_acq_rel, std::memory_order_relaxed)) return true;
        }
    }

    template<class Lookup>
    auto ReadPointer(Lookup&& Find) const -> decltype(Find())
    {
        if (HasScopedBorrow()) return Find();
        const FBorrow Scope(*this);
        if (!Scope) return nullptr;
        auto* Pointer = Find();
        if (Pointer) State.fetch_or(Untracked, std::memory_order_release);
        return Pointer;
    }

    bool HasUntrackedReference() const noexcept
    { return (State.load(std::memory_order_acquire) & Untracked) != 0; }

    // Values-only native queries may be made by the reservation holder.
    // AdmitReference never grants this exception to a raw pointer/reference.
    bool IsReservationOwner() const noexcept
    {
        for (auto* Reservation = FReservation::Current; Reservation; Reservation = Reservation->Previous)
            if (Reservation->Holds(*this)) return true;
        return false;
    }

private:
    bool HasScopedBorrow() const noexcept
    {
        for (auto* Scope = FBorrow::Current; Scope; Scope = Scope->Previous)
            if (Scope->Gate == this) return true;
        return false;
    }
};

inline FWorldBorrowGate::FBorrow::FBorrow(const FReservation& Reservation) noexcept
{
    if (Reservation.Gate && Reservation.Holds(*Reservation.Gate)) Enter(*Reservation.Gate);
}
}
