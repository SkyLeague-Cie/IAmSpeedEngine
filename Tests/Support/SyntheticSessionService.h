#pragma once

#include "IAmSpeed/World/Simulation/Lifecycle/SessionLifecycle.h"

namespace Speed::Lifecycle::Testing
{
struct FTrace final
{
    unsigned ClientsCreated = 0, ClientsDestroyed = 0;
    unsigned SessionsCreated = 0, SessionsDestroyed = 0;
    unsigned ParticipantsCreated = 0, ParticipantsDestroyed = 0;
    unsigned Applied = 0, Published = 0, Completed = 0, Aborted = 0;
};

struct FSessionAddress
{
    std::size_t Slot = 0;
    std::uint64_t Generation = 0;
    FParticipantHandle Participant;
};

enum class ELaneState : std::uint8_t { NeverStarted, Idle, Reserved, Committed, Published, Exited };
struct FObservation
{
    ESessionState Session = ESessionState::Configured;
    EParticipantState Participant = EParticipantState::Active;
    ELaneState Lane = ELaneState::NeverStarted;
    std::uint64_t Value = 0, PublishedValue = 0, PublicationSerial = 0;
    bool ClientOpen = false, Paused = false, HoldsLease = false;
};

namespace Detail
{
struct FParticipant final
{
    explicit FParticipant(std::shared_ptr<FTrace> InTrace) : Trace(std::move(InTrace))
    { ++Trace->ParticipantsCreated; }
    ~FParticipant() { ++Trace->ParticipantsDestroyed; }
    std::shared_ptr<FTrace> Trace;
    std::uint64_t Value = 0;
};

// Test-only record. No arbitrary callable, external physical state or worker.
struct FSession final
{
    FSession(std::uint64_t Epoch, std::shared_ptr<FTrace> InTrace,
        std::shared_ptr<const FCodeLease> Lease)
        : Trace(std::move(InTrace)), Participant(Trace), Protocol(Epoch, std::move(Lease))
    {
        Handle = *Protocol.RegisterParticipant();
        ++Trace->SessionsCreated;
    }
    ~FSession() { ++Trace->SessionsDestroyed; }
    bool Submit(std::uint64_t Value) noexcept
    {
        if (!ClientOpen || Protocol.GetState() != ESessionState::Running) return false;
        LatestInput = Value;
        return true;
    }
    void CloseClient() noexcept
    {
        ClientOpen = false;
        Protocol.RequestStop();
    }
    FObservation Observe() const noexcept
    {
        return {Protocol.GetState(), *Protocol.ParticipantState(Handle), Lane,
            Participant.Value, PublishedValue, PublicationSerial, ClientOpen,
            Protocol.IsPaused(), Protocol.HoldsCodeLease()};
    }

    std::shared_ptr<FTrace> Trace; // Survives every observed member's destructor.
    FParticipant Participant;
    TSessionLifecycle<1> Protocol;
    FParticipantHandle Handle;
    ELaneState Lane = ELaneState::NeverStarted;
    std::uint64_t LatestInput = 0, AdmittedInput = 0, PublishedValue = 0, PublicationSerial = 0;
    bool ClientOpen = true;
};
}

template<std::size_t> class TSyntheticSessionService;

class FSyntheticClient final
{
public:
    FSyntheticClient(const FSyntheticClient&) = delete;
    FSyntheticClient& operator=(const FSyntheticClient&) = delete;
    FSyntheticClient(FSyntheticClient&& Other) noexcept
        : Session(std::move(Other.Session)), Trace(std::move(Other.Trace)),
          Address(std::move(Other.Address)), OwnsClient(std::exchange(Other.OwnsClient, false)) {}
    FSyntheticClient& operator=(FSyntheticClient&& Other) noexcept
    {
        if (this != &Other)
        {
            Close();
            Session = std::move(Other.Session);
            Trace = std::move(Other.Trace);
            Address = std::move(Other.Address);
            OwnsClient = std::exchange(Other.OwnsClient, false);
        }
        return *this;
    }
    ~FSyntheticClient() { Close(); }
    FSessionAddress GetAddress() const noexcept { return Address; }
    std::weak_ptr<const void> LifetimeWitness() const noexcept { return Session; }
    bool Submit(std::uint64_t Value) noexcept
    {
        if (!OwnsClient) return false;
        const auto S = Session.lock();
        return S && S->Submit(Value);
    }
    bool RequestStop() noexcept
    {
        if (!OwnsClient) return false;
        const auto S = Session.lock();
        return S && S->Protocol.RequestStop();
    }
    std::optional<FObservation> Observe() const noexcept
    {
        if (!OwnsClient) return {};
        const auto S = Session.lock();
        if (!S) return {};
        return S->Observe();
    }
    void Close() noexcept
    {
        if (!OwnsClient) return;
        if (const auto S = Session.lock()) S->CloseClient();
        OwnsClient = false;
        ++Trace->ClientsDestroyed;
        Session.reset();
    }
private:
    template<std::size_t> friend class TSyntheticSessionService;
    FSyntheticClient(const std::shared_ptr<Detail::FSession>& InSession,
        FSessionAddress InAddress, std::shared_ptr<FTrace> InTrace)
        : Session(InSession), Trace(std::move(InTrace)), Address(std::move(InAddress))
    { ++Trace->ClientsCreated; }
    std::weak_ptr<Detail::FSession> Session;
    std::shared_ptr<FTrace> Trace;
    FSessionAddress Address;
    bool OwnsClient = true;
};

// All operations, including fake lane and reaper pulses, are serialized by the
// test. This class owns no thread, module service, wait primitive or callback.
template<std::size_t Capacity = 2>
class TSyntheticSessionService final
{
    static_assert(Capacity > 0, "service capacity");
public:
    TSyntheticSessionService() = default;
    TSyntheticSessionService(const TSyntheticSessionService&) = delete;
    TSyntheticSessionService& operator=(const TSyntheticSessionService&) = delete;
    ~TSyntheticSessionService()
    {
        // No implicit drain, join or intentional leak can hide a broken fixture.
        for (const auto& Slot : Slots) if (Slot.Session) std::terminate();
    }
    std::optional<FSyntheticClient> Create(std::shared_ptr<FTrace> Trace,
        std::shared_ptr<const FCodeLease> Lease, bool StartImmediately = true)
    {
        if (!Trace || !Lease || !Lease->Id || NextEpoch == std::numeric_limits<std::uint64_t>::max()) return {};
        for (std::size_t I = 0; I < Capacity; ++I)
        {
            auto& Slot = Slots[I];
            if (Slot.Session || Slot.Generation == std::numeric_limits<std::uint64_t>::max()) continue;
            auto S = std::make_shared<Detail::FSession>(NextEpoch, Trace, std::move(Lease));
            ++NextEpoch;
            ++Slot.Generation;
            Slot.Session = S; // Retain before activation or exposing the weak endpoint.
            if (StartImmediately)
            {
                if (!S->Protocol.Start()) std::terminate();
                S->Lane = ELaneState::Idle;
            }
            return FSyntheticClient(S, {I, Slot.Generation, S->Handle}, std::move(Trace));
        }
        return {}; // Reject before allocation or activation when all slots are full.
    }
    std::size_t RetainedSessions() const noexcept
    {
        std::size_t Count = 0;
        for (const auto& Slot : Slots) Count += Slot.Session ? 1u : 0u;
        return Count;
    }
    std::optional<FObservation> Observe(const FSessionAddress& Address) const noexcept
    {
        const auto S = Find(Address);
        if (!S) return {};
        return S->Observe();
    }
    bool SetPaused(const FSessionAddress& Address, bool Paused) noexcept
    {
        const auto S = Find(Address);
        return S && S->Protocol.SetPaused(Paused);
    }
    bool RequestStop(const FSessionAddress& Address) noexcept
    {
        const auto S = Find(Address);
        return S && S->Protocol.RequestStop();
    }
    std::optional<FReservationToken> Admit(const FSessionAddress& Address) noexcept
    {
        const auto S = Find(Address);
        if (!S || !S->ClientOpen || S->Lane != ELaneState::Idle ||
            S->PublicationSerial == std::numeric_limits<std::uint64_t>::max()) return {};
        auto Token = S->Protocol.TryAdmit();
        if (Token)
        {
            S->AdmittedInput = S->LatestInput;
            S->Lane = ELaneState::Reserved;
        }
        return Token;
    }
    bool Apply(const FSessionAddress& Address, const FReservationToken& Token) noexcept
    {
        const auto S = Find(Address);
        if (!S || !S->Protocol.MarkCommitted(Token)) return false;
        // One serialized, nonthrowing scalar step; invalid tokens cannot write.
        S->Participant.Value = S->AdmittedInput;
        ++S->Trace->Applied;
        S->Lane = ELaneState::Committed;
        return true;
    }
    bool Publish(const FSessionAddress& Address, const FReservationToken& Token) noexcept
    {
        const auto S = Find(Address);
        if (!S || !S->Protocol.MarkPublished(Token)) return false;
        S->PublishedValue = S->Participant.Value;
        ++S->PublicationSerial;
        ++S->Trace->Published;
        S->Lane = ELaneState::Published;
        return true;
    }
    bool Complete(const FSessionAddress& Address, const FReservationToken& Token) noexcept
    {
        const auto S = Find(Address);
        if (!S || !S->Protocol.Complete(Token)) return false;
        ++S->Trace->Completed;
        S->Lane = ELaneState::Idle;
        return true;
    }
    bool Abort(const FSessionAddress& Address, const FReservationToken& Token) noexcept
    {
        const auto S = Find(Address);
        if (!S || !S->Protocol.Abort(Token)) return false;
        ++S->Trace->Aborted;
        S->AdmittedInput = 0;
        S->Lane = ELaneState::Idle;
        return true;
    }
    bool BeginDrain(const FSessionAddress& Address) noexcept
    {
        const auto S = Find(Address);
        if (!S || !S->Protocol.BeginStopDrain(Address.Participant.Session)) return false;
        return S->Protocol.BeginParticipantDrain(Address.Participant);
    }
    bool Detach(const FSessionAddress& Address) noexcept
    {
        const auto S = Find(Address);
        return S && S->Protocol.AcknowledgeDetached(Address.Participant);
    }
    bool StopAtBoundary(const FSessionAddress& Address) noexcept
    {
        const auto S = Find(Address);
        return S && S->Protocol.AcknowledgeStopped(Address.Participant.Session);
    }
    bool ReturnFakeLane(const FSessionAddress& Address) noexcept
    {
        const auto S = Find(Address);
        if (!S || S->Protocol.GetState() != ESessionState::Stopped ||
            (S->Lane != ELaneState::Idle && S->Lane != ELaneState::NeverStarted)) return false;
        // An explicit return from the fake lane, not an inferred real thread exit.
        // NeverStarted has no admitted work and follows this same empty-lane fence.
        S->Lane = ELaneState::Exited;
        return S->Protocol.AcknowledgeThreadExited(Address.Participant.Session);
    }
    std::size_t ReapReady() noexcept
    {
        std::size_t Reaped = 0;
        for (auto& Slot : Slots)
        {
            if (!Slot.Session || Slot.Session->Lane != ELaneState::Exited ||
                Slot.Session->Protocol.GetState() != ESessionState::ThreadExited) continue;
            if (!Slot.Session->Protocol.Reap(Slot.Session->Handle.Session)) std::terminate();
            Slot.Session.reset();
            ++Reaped;
        }
        return Reaped;
    }
private:
    struct FSlot { std::shared_ptr<Detail::FSession> Session; std::uint64_t Generation = 0; };
    std::shared_ptr<Detail::FSession> Find(const FSessionAddress& Address) const noexcept
    {
        if (Address.Slot >= Capacity) return {};
        const auto& Slot = Slots[Address.Slot];
        if (!Slot.Session || Slot.Generation != Address.Generation ||
            !Slot.Session->Protocol.ParticipantState(Address.Participant)) return {};
        return Slot.Session;
    }
    std::array<FSlot, Capacity> Slots{};
    std::uint64_t NextEpoch = 1;
};
}
