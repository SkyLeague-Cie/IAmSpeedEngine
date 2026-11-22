#pragma once
#include "ExactRemoteInputProducer.h"

namespace Speed::Netcode
{
enum class ERemoteIngressState { Unbound, Bound, Retired };

// Thread-neutral acquisition service. A transport receives only this handle,
// never the physical producer or its poll/lifecycle methods. No producer exists
// until the sole owner calls CreateProducerForCurrentOwner at its boundary.
// The existing producer supplies the sole bounded immutable queue and history;
// there is no second queue, frame rewrite or consume ACK.
class FExactRemoteInputIngress final : public std::enable_shared_from_this<FExactRemoteInputIngress>
{
public:
    static std::shared_ptr<FExactRemoteInputIngress> Create(
        std::shared_ptr<const V2::FInputActionContract> Contract,
        V2::FStreamEpoch Epoch, Input::FProducerIdentity Identity, Input::FFrameNumber First,
        std::uint64_t InitialRoundGeneration = 0)
    {
        if (Input::FPresentationInputScope::IsActive() || !Contract || !Epoch.Value || !Identity.Id
            || Identity.Kind > Input::EProducerKind::Network) return {};
        return std::shared_ptr<FExactRemoteInputIngress>(
            new FExactRemoteInputIngress(std::move(Contract), Epoch, Identity, First, InitialRoundGeneration));
    }
    FExactRemoteInputIngress(const FExactRemoteInputIngress&) = delete;
    FExactRemoteInputIngress& operator=(const FExactRemoteInputIngress&) = delete;

    // Closed before the owner binds and permanently after owner retirement.
    // Accepted means retained acquisition only, never simulation processing.
    ERemoteSubmission SubmitFrame(const V2::FInputFrame& Frame)
    { return SubmitFrame(0, Frame); }
    ERemoteSubmission SubmitFrame(std::uint64_t Round, const V2::FInputFrame& Frame)
    {
        std::lock_guard<std::mutex> Lock(Gate);
        if (Round != RoundGeneration) return ERemoteSubmission::WrongRound;
        return Source ? Source->SubmitFrame(Frame) : ERemoteSubmission::Closed;
    }
    ERemoteIngressState GetState() const
    { std::lock_guard<std::mutex> Lock(Gate); return State; }
    bool MatchesBinding(const V2::FInputActionContract& C, V2::FStreamEpoch E,
        Input::FProducerIdentity I, Input::FFrameNumber N) const
    {
        return Contract->GetFingerprint() == C.GetFingerprint() && Epoch.Value == E.Value
            && Identity.Id == I.Id && Identity.Kind == I.Kind && First == N;
    }
    std::unique_ptr<V2::IInputProducer> CreateProducerForCurrentOwner()
    {
        if (Input::FPresentationInputScope::IsActive()) return {};
        std::lock_guard<std::mutex> Lock(Gate);
        if (State != ERemoteIngressState::Unbound) return {};
        auto Core = FExactRemoteInputProducer::Create(Contract, Epoch, Identity, First);
        if (!Core) return {};
        auto Result = std::make_unique<FOwnerSource>(shared_from_this(), std::move(Core));
        Source = Result->Core.get(); State = ERemoteIngressState::Bound;
        return Result;
    }
private:
    class FRoundResume final : public V2::IPreparedInputResume
    {
    public:
        FRoundResume(FExactRemoteInputIngress& InIngress, FExactRemoteInputProducer& InSource,
            Input::FFrameNumber InFrame, const V2::FRoundInputResume& Round)
            : Ingress(InIngress.shared_from_this()), Source(InSource), Frame(InFrame),
              Previous(Round.PreviousGeneration), Next(Round.Generation),
              Neutral(std::make_shared<const V2::FInputFrame>(*Round.Baseline)),
              Owner(std::this_thread::get_id()) {}
        ~FRoundResume() override { Abort(); }
        const V2::FInputFrame& Baseline() const noexcept override { return *Neutral; }

        bool IsCurrent() const
        {
            return std::this_thread::get_id() == Owner && Ingress->Source == &Source
                && Ingress->RoundGeneration == Previous && Source.CanInstallRoundBaseline(Frame, *Neutral);
        }
        bool PrepareFinalization() override
        {
            if (std::this_thread::get_id() != Owner || Fence.owns_lock() || Released) return false;
            std::unique_lock<std::mutex> Lock(Ingress->Gate);
            if (!IsCurrent()) return false;
            Fence = std::move(Lock);
            return true;
        }
        void Finalize() noexcept override
        {
            if (std::this_thread::get_id() != Owner || !Fence.owns_lock() || Finalized) std::terminate();
            Source.InstallRoundBaseline(Neutral, RetiredPending, RetiredHistory);
            Ingress->RoundGeneration = Next;
            Finalized = true;
        }
        void Release() noexcept override
        {
            if (std::this_thread::get_id() != Owner || !Fence.owns_lock() || !Finalized) std::terminate();
            Abort();
        }
        void Abort() noexcept override
        {
            if (std::this_thread::get_id() != Owner) std::terminate();
            if (Fence.owns_lock()) Fence.unlock();
            Released = true;
        }

    private:
        const std::shared_ptr<FExactRemoteInputIngress> Ingress;
        FExactRemoteInputProducer& Source;
        const Input::FFrameNumber Frame;
        const std::uint64_t Previous, Next;
        const std::shared_ptr<const V2::FInputFrame> Neutral;
        const std::thread::id Owner;
        FExactRemoteInputProducer::FFrameQueue RetiredPending{}, RetiredHistory{};
        std::unique_lock<std::mutex> Fence;
        bool Finalized = false, Released = false;
    };

    // All producer entry points, including the first poll and lifecycle call,
    // are pinned to the thread which constructed this source. Ingress submits
    // serialize with destruction through Gate, so a last transport reference
    // cannot move destruction of the physical source to a foreign thread.
    class FOwnerSource final : public V2::IInputProducer, public V2::IInputProducerPollFence
    {
    public:
        FOwnerSource(std::shared_ptr<FExactRemoteInputIngress> InIngress, std::unique_ptr<FExactRemoteInputProducer> InCore)
            : Ingress(std::move(InIngress)), Core(std::move(InCore)), Owner(std::this_thread::get_id()) {}
        ~FOwnerSource() override
        {
            if (!IsOwner()) std::terminate();
            std::lock_guard<std::mutex> Lock(Ingress->Gate);
            Ingress->Source = nullptr; Ingress->State = ERemoteIngressState::Retired;
            if (Core->CancelLifecycle() != V2::ELifecycleResult::Applied) std::terminate();
        }
        V2::IInputProducerPollFence* PollFence() noexcept override { return this; }
        bool SupportsPollFence() const noexcept override { return true; }
        V2::EProducerContract GetProducerContract() const noexcept override { return V2::EProducerContract::ExactRemote; }
        const std::shared_ptr<const V2::FInputActionContract>& GetContract() const override { return Core->GetContract(); }
        V2::EInputFrameReadiness QueryFrameReadiness(Input::FFrameNumber N) const override
        { return IsOwner() ? Core->QueryFrameReadiness(N) : V2::EInputFrameReadiness::Rejected; }
        std::optional<V2::FInputFrame> InspectBaseline(Input::FFrameNumber N) const override
        { return IsOwner() ? Core->InspectBaseline(N) : std::nullopt; }
        std::optional<V2::FInputPollCutoff> FreezeForOwner(Input::FFrameNumber N) override
        { return IsOwner() ? Core->FreezeForOwner(N) : std::nullopt; }
        std::optional<V2::FInputFrame> Produce(Input::FFrameNumber N) override
        { return IsOwner() ? Core->Produce(N) : std::nullopt; }
        bool CloseFrozenCutoff(const V2::FInputPollCutoff& C) noexcept override
        { return IsOwner() && Core->CloseFrozenCutoff(C); }
        V2::ELifecycleResult SetLifecyclePaused(bool Value) override
        { return IsOwner() ? Core->SetLifecyclePaused(Value) : V2::ELifecycleResult::Rejected; }
        V2::ELifecycleResult CancelLifecycle() override
        { return IsOwner() ? Core->CancelLifecycle() : V2::ELifecycleResult::Rejected; }
        std::unique_ptr<V2::IPreparedInputResume> PrepareRoundResumeAtBoundary(Input::FFrameNumber Frame,
            const V2::FRoundInputResume& Round) override
        {
            if (!IsOwner() || Input::FPresentationInputScope::IsActive() || !Round.Baseline
                || !Round.PreviousGeneration || Round.PreviousGeneration == UINT64_MAX
                || Round.Generation != Round.PreviousGeneration + 1) return {};
            auto Plan = std::make_unique<FRoundResume>(*Ingress, *Core, Frame, Round);
            std::lock_guard<std::mutex> Lock(Ingress->Gate);
            if (!Plan->IsCurrent()) return {};
            return Plan;
        }
        // Core stays private to this owner-facing adapter and its enclosing
        // acquisition service; it is never returned to a transport caller.
        friend class FExactRemoteInputIngress;
    private:
        bool IsOwner() const { return std::this_thread::get_id() == Owner; }
        const std::shared_ptr<FExactRemoteInputIngress> Ingress;
        const std::unique_ptr<FExactRemoteInputProducer> Core;
        const std::thread::id Owner;
    };
    FExactRemoteInputIngress(std::shared_ptr<const V2::FInputActionContract> C,
        V2::FStreamEpoch E, Input::FProducerIdentity I, Input::FFrameNumber N, std::uint64_t Round)
        : Contract(std::move(C)), Epoch(E), Identity(I), First(N), RoundGeneration(Round) {}
    const std::shared_ptr<const V2::FInputActionContract> Contract;
    const V2::FStreamEpoch Epoch;
    const Input::FProducerIdentity Identity;
    const Input::FFrameNumber First;
    mutable std::mutex Gate;
    std::uint64_t RoundGeneration; // protected by Gate, including SubmitFrame and source cutover
    FExactRemoteInputProducer* Source = nullptr; // lifetime protected by Gate
    ERemoteIngressState State = ERemoteIngressState::Unbound;
};
}
