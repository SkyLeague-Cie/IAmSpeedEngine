#pragma once
#include "InputFrame.h"
#include <deque>
#include <mutex>
#include <thread>

namespace Speed::Input
{
// Transitional legacy wire semantics only. This is NOT Network V2 ExactRemote:
// Requested still needs the historical one-step filter on the receiving worker.
enum class ELegacyRemoteAdmission : std::uint8_t { Accepted, WrongSource, Closed, Stale, DuplicateOrOutOfOrder, Full };
struct FLegacyRemoteAddress
{
    std::uint64_t Producer = 0, Epoch = 0;
    FFrameNumber SourceFrame = 0, ActivationFrame = 0;
};
template<class TWirePayload> struct TLegacyRemotePreSlewPacket
{
    FLegacyRemoteAddress Address;
    FActionValues Requested{};
    TWirePayload Wire;
};
template<class TWirePayload> class TLegacyRemotePreSlewOwner;

// Transport ingress only. Copy a complete wire payload; never mutate component
// targets, dedup bits, camera flags or physical state from the receive callback.
template<class TWirePayload> class TLegacyRemotePreSlewIngress final
{
    friend class TLegacyRemotePreSlewOwner<TWirePayload>;
public:
    using FPacket = TLegacyRemotePreSlewPacket<TWirePayload>;
    TLegacyRemotePreSlewIngress(std::uint64_t InProducer, std::uint64_t InEpoch, FFrameNumber FirstFrame,
        std::size_t InCapacity = 256)
        : Producer(InProducer), Epoch(InEpoch), MinimumFrame(FirstFrame), Capacity(InCapacity) {}
    ELegacyRemoteAdmission Submit(const FLegacyRemoteAddress& Address, const FActionValues& Values, const TWirePayload& Payload)
    {
        std::lock_guard<std::mutex> Lock(Gate);
        if (!Producer || !Epoch || Address.Producer != Producer || Address.Epoch != Epoch) return ELegacyRemoteAdmission::WrongSource;
        if (Closed) return ELegacyRemoteAdmission::Closed;
        if (Address.ActivationFrame < MinimumFrame) return ELegacyRemoteAdmission::Stale;
        if (HasAccepted && (Address.SourceFrame <= LastSource || Address.ActivationFrame <= LastActivation))
            return ELegacyRemoteAdmission::DuplicateOrOutOfOrder;
        if (Queue.size() >= Capacity) return ELegacyRemoteAdmission::Full;
        auto Packet = std::make_shared<const FPacket>(FPacket{Address, Values, Payload});
        Queue.push_back(std::move(Packet));
        HasAccepted = true; LastSource = Address.SourceFrame; LastActivation = Address.ActivationFrame;
        return ELegacyRemoteAdmission::Accepted;
    }
    void Close() { std::lock_guard<std::mutex> Lock(Gate); Closed = true; Queue.clear(); }
private:
    bool Freeze(FFrameNumber Frame, std::shared_ptr<const FPacket>& Packet)
    {
        std::lock_guard<std::mutex> Lock(Gate);
        if (Closed || Frame < MinimumFrame || Frame == UINT64_MAX) return false;
        if (!Queue.empty() && Queue.front()->Address.ActivationFrame < Frame) return false;
        if (!Queue.empty() && Queue.front()->Address.ActivationFrame == Frame)
        { Packet = Queue.front(); Queue.pop_front(); }
        MinimumFrame = Frame + 1;
        return true;
    }
    const std::uint64_t Producer, Epoch;
    mutable std::mutex Gate;
    std::deque<std::shared_ptr<const FPacket>> Queue;
    FFrameNumber MinimumFrame = 0, LastSource = 0, LastActivation = 0;
    const std::size_t Capacity;
    bool HasAccepted = false, Closed = false;
};

template<class TWirePayload> struct TLegacyRemotePreSlewSnapshot
{
    FFrameNumber Frame = 0;
    std::shared_ptr<const TLegacyRemotePreSlewPacket<TWirePayload>> Packet;
    FActionValues Requested{}, Applied{};
    bool FreshPacket = false;
};

// Construct, poll and destroy on the physical worker. The only persistent
// mutable input processing state is held here, never in the received snapshot.
template<class TWirePayload> class TLegacyRemotePreSlewOwner final
{
public:
    using FIngress = TLegacyRemotePreSlewIngress<TWirePayload>;
    using FSnapshot = TLegacyRemotePreSlewSnapshot<TWirePayload>;
    TLegacyRemotePreSlewOwner(std::shared_ptr<FIngress> InIngress, FFrameNumber FirstFrame,
        std::array<std::uint16_t, ActionCount> InSteps, FActionValues InitialApplied = {})
        : Ingress(std::move(InIngress)), Worker(std::this_thread::get_id()), NextFrame(FirstFrame),
          Steps(InSteps), Applied(InitialApplied) {}
    TLegacyRemotePreSlewOwner(const TLegacyRemotePreSlewOwner&) = delete;
    ~TLegacyRemotePreSlewOwner() { if (Worker != std::this_thread::get_id()) std::terminate(); if (Ingress) Ingress->Close(); }
    std::shared_ptr<const FSnapshot> Poll(FFrameNumber Frame)
    {
        if (Worker != std::this_thread::get_id() || Terminal || !Ingress) return {};
        if (Pending) return Pending->Frame == Frame ? Pending : nullptr;
        if (Frame != NextFrame) return {};
        try
        {
            auto Snapshot = std::make_shared<FSnapshot>();
            auto Packet = LastPacket;
            if (!Ingress->Freeze(Frame, Packet)) { Terminal = true; return {}; }
            Snapshot->Frame = Frame; Snapshot->Packet = Packet; Snapshot->FreshPacket = Packet != LastPacket;
            Snapshot->Requested = Packet ? Packet->Requested : FActionValues{};
            Snapshot->Applied = Snapshot->Requested;
            for (std::size_t I = 0; I < ActionCount; ++I)
            {
                if (!Steps[I]) continue;
                const int Before = Applied[I], Target = Snapshot->Requested[I], Step = Steps[I];
                Snapshot->Applied[I] = static_cast<std::int16_t>(Before < Target ? std::min(Before + Step, Target) : std::max(Before - Step, Target));
            }
            Pending = std::move(Snapshot);
            return Pending;
        }
        catch (...) { Terminal = true; return {}; }
    }
    bool IsOwnerThread() const noexcept { return Worker == std::this_thread::get_id(); }
    bool CanComplete(FFrameNumber Frame) const noexcept
    { return Worker == std::this_thread::get_id() && !Terminal && Pending && Pending->Frame == Frame; }
    bool Complete(FFrameNumber Frame) noexcept
    {
        if (!CanComplete(Frame)) return false;
        Applied = Pending->Applied; LastPacket = Pending->Packet;
        Latest = Pending; Pending.reset(); NextFrame = Frame + 1;
        return true;
    }
    void Abort() noexcept { if (Worker == std::this_thread::get_id()) { Terminal = true; Pending.reset(); Latest.reset(); } }
    std::shared_ptr<const FSnapshot> ReadPending() const { return Worker == std::this_thread::get_id() && !Terminal ? Pending : nullptr; }
    std::shared_ptr<const FSnapshot> ReadLatest() const { return Worker == std::this_thread::get_id() && !Terminal ? Latest : nullptr; }
private:
    std::shared_ptr<FIngress> Ingress;
    const std::thread::id Worker;
    FFrameNumber NextFrame = 0;
    const std::array<std::uint16_t, ActionCount> Steps;
    FActionValues Applied{};
    std::shared_ptr<const TLegacyRemotePreSlewPacket<TWirePayload>> LastPacket;
    std::shared_ptr<const FSnapshot> Pending, Latest;
    bool Terminal = false;
};
}
