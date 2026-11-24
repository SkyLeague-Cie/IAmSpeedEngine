#pragma once
#include <memory>
#include <thread>
#include <utility>

// Included AFTER ISimulationRestoreBoundaryLease is defined. Only the world
// creates an owner; providers get copyable weak views, never a raw lease/ref.
namespace Speed
{
class FPreparedRestoreLeaseOwner;
class FPreparedNativeKinematicFields;
class FPreparedNativeRoundFields;
struct FCommonComponentValueAccess;
struct FSimulationComponentRestoreRequest;
namespace RestoreLeaseDetail
{
enum class EPhase : unsigned char { Preparing, Ready, Finalizing, Finalized, Aborting, Releasing, Closed };
struct FState final
{
    const FSimulationWorld* const World;
    const std::shared_ptr<const PreparedWorld::FRestoreEnvelope> Envelope;
    const PreparedWorld::FAddress Address;
    const std::thread::id Worker = std::this_thread::get_id();
    std::unique_ptr<ISimulationRestoreBoundaryLease> Lease;
    EPhase Phase = EPhase::Preparing;
    bool Querying = false;
    const FSimulationComponentRestoreRequest* ComponentRequest = nullptr;
    const ISpeedComponent* ComponentTarget = nullptr;
    const PreparedWorld::FActorBinding* ComponentBinding = nullptr;
    FState(const FSimulationWorld& W, std::shared_ptr<const PreparedWorld::FRestoreEnvelope> E,
        std::unique_ptr<ISimulationRestoreBoundaryLease> L)
        : World(&W), Envelope(std::move(E)), Address(Envelope->Address), Lease(std::move(L)) {}
    bool Query(const FSimulationWorld* W, const PreparedWorld::FAddress* A) noexcept
    {
        // Wrong-thread queries return before reading any worker-mutated field.
        if (Worker != std::this_thread::get_id()) return false;
        if (Querying || !Lease || (W && W != World) || (A && !(*A == Address))
            || Phase == EPhase::Aborting || Phase == EPhase::Releasing || Phase == EPhase::Closed) return false;
        Querying = true;
        const bool Result = Lease->ProtectsEscapedMutableViews() && Lease->HoldsExactBoundary(*World, Address);
        Querying = false; return Result;
    }
    ~FState() noexcept
    {
        if (!Lease) return;
        if (Phase == EPhase::Finalizing || Phase == EPhase::Finalized || Phase == EPhase::Releasing) std::terminate();
        Phase = EPhase::Closed; Lease->AbortBeforeFinalize();
    }
};
}
class FPreparedRestoreLeaseView final
{
    friend class FPreparedRestoreLeaseOwner;
    friend class FPreparedNativeKinematicFields;
    friend class FPreparedNativeRoundFields;
    friend struct FCommonComponentValueAccess;
    std::weak_ptr<RestoreLeaseDetail::FState> State;
    explicit FPreparedRestoreLeaseView(const std::shared_ptr<RestoreLeaseDetail::FState>& S) noexcept : State(S) {}
    std::shared_ptr<RestoreLeaseDetail::FState> ResolvePreparingState(const FSimulationComponentRestoreRequest& Request,
        const PreparedWorld::FAddress& Address) const noexcept
    {
        auto S = State.lock();
        if (!S || S->Worker != std::this_thread::get_id()) return {};
        if (S->Phase != RestoreLeaseDetail::EPhase::Preparing || S->ComponentRequest != &Request || !S->ComponentBinding
            || !S->Query(nullptr, &Address)) return {};
        return S;
    }

    std::shared_ptr<RestoreLeaseDetail::FState> ResolvePreparingActor(
        const PreparedWorld::FAddress& Address, const PreparedWorld::FActorBinding& Binding) const noexcept
    {
        auto S = State.lock();
        if (!S || S->Worker != std::this_thread::get_id() || S->Phase != RestoreLeaseDetail::EPhase::Preparing
            || !S->ComponentTarget || !S->ComponentBinding || !(*S->ComponentBinding == Binding)
            || !S->Query(nullptr, &Address)) return {};
        return S;
    }

    // Only IAS fragment factories can resolve the native world, and only
    // while the world is dispatching this exact synchronous request.
    const FSimulationWorld* ResolvePreparingWorld(const FSimulationComponentRestoreRequest& Request,
        const PreparedWorld::FAddress& Address) const noexcept
    {
        const auto S = ResolvePreparingState(Request, Address);
        return S ? S->World : nullptr;
    }

    bool MatchesPreparingTarget(const FSimulationComponentRestoreRequest& Request, const PreparedWorld::FAddress& Address,
        const FSimulationWorld& World, const ISpeedComponent& Target) const noexcept
    {
        const auto S = ResolvePreparingState(Request, Address);
        return S && S->World == &World && S->ComponentTarget == &Target;
    }

    bool MatchesPreparingEndpoint(const FSimulationComponentRestoreRequest& Request,
        const PreparedWorld::FAddress& Address, const PreparedWorld::FEndpointBinding& Endpoint) const noexcept
    {
        const auto S = ResolvePreparingState(Request, Address);
        if (!S || Endpoint.Actor != S->ComponentBinding->StableId
            || Endpoint.ActorIncarnation != S->ComponentBinding->Incarnation) return false;
        const auto& Endpoints = S->Envelope->Endpoints;
        const auto I = std::lower_bound(Endpoints.begin(), Endpoints.end(), Endpoint.StableId,
            [](const PreparedWorld::FEndpointBinding& Row, uint64 Id) { return Row.StableId < Id; });
        return I != Endpoints.end() && *I == Endpoint;
    }

    const ISpeedComponent* ResolvePreparingTarget(const FSimulationComponentRestoreRequest& Request,
        const PreparedWorld::FAddress& Address) const noexcept
    {
        const auto S = ResolvePreparingState(Request, Address);
        return S ? S->ComponentTarget : nullptr;
    }
public:
    FPreparedRestoreLeaseView() = default;
    bool HoldsExactBoundary(const FSimulationWorld& W, const PreparedWorld::FAddress& A) const noexcept
    { auto S = State.lock(); return S && S->Query(&W, &A); }
    bool IsExactAddress(const PreparedWorld::FAddress& A) const noexcept
    { auto S = State.lock(); return S && S->Query(nullptr, &A); }
    bool ProtectsEscapedMutableViews() const noexcept
    { auto S = State.lock(); return S && S->Query(nullptr, nullptr); }
};
class FPreparedRestoreLeaseOwner final
{
    friend class FSimulationWorld;
    std::shared_ptr<RestoreLeaseDetail::FState> State;
    FPreparedRestoreLeaseOwner(const FSimulationWorld& W, std::shared_ptr<const PreparedWorld::FRestoreEnvelope> E,
        std::unique_ptr<ISimulationRestoreBoundaryLease> L)
        : State(std::make_shared<RestoreLeaseDetail::FState>(W, std::move(E), std::move(L))) {}
    template<class Callback>
    auto WithComponent(const FSimulationComponentRestoreRequest* Request, const ISpeedComponent& Target,
        const PreparedWorld::FActorBinding& Binding, Callback&& Prepare)
    {
        Require(RestoreLeaseDetail::EPhase::Preparing);
        if (State->ComponentRequest || State->ComponentTarget || State->ComponentBinding) std::terminate();
        struct FClearRequest
        {
            RestoreLeaseDetail::FState& State;
            ~FClearRequest() noexcept
            {
                State.ComponentRequest = nullptr;
                State.ComponentTarget = nullptr;
                State.ComponentBinding = nullptr;
            }
        } Clear{*State};
        State->ComponentRequest = Request;
        State->ComponentTarget = &Target;
        State->ComponentBinding = &Binding;
        return std::forward<Callback>(Prepare)();
    }
    template<class Callback>
    auto PrepareComponent(const FSimulationComponentRestoreRequest& Request, const ISpeedComponent& Target,
        const PreparedWorld::FActorBinding& Binding, Callback&& Prepare)
    {
        return WithComponent(&Request, Target, Binding, std::forward<Callback>(Prepare));
    }
public:
    FPreparedRestoreLeaseOwner(const FPreparedRestoreLeaseOwner&) = delete;
    FPreparedRestoreLeaseOwner& operator=(const FPreparedRestoreLeaseOwner&) = delete;
    FPreparedRestoreLeaseView View() const noexcept { return FPreparedRestoreLeaseView(State); }
    bool AppendPreparedAccess(std::vector<PreparedAccess::FParticipantAccess>& Out)
    {
        Require(RestoreLeaseDetail::EPhase::Preparing);
        const auto Span = [](const void* P, std::size_t N)
        { return PreparedAccess::FSpan{reinterpret_cast<std::uintptr_t>(P), N}; };
        PreparedAccess::FParticipantAccess P;
        P.Participant = reinterpret_cast<std::uintptr_t>(State.get());
        P.Writes = {Span(&State->Phase, sizeof(State->Phase)), Span(&State->Querying, sizeof(State->Querying)),
            Span(&State->Lease, sizeof(State->Lease)), Span(&State->ComponentRequest, sizeof(State->ComponentRequest)),
            Span(&State->ComponentTarget, sizeof(State->ComponentTarget)),
            Span(&State->ComponentBinding, sizeof(State->ComponentBinding))};
        P.ReadOnly = {Span(&State->World, sizeof(State->World)), Span(&State->Envelope, sizeof(State->Envelope)),
            Span(&State->Address, sizeof(State->Address)), Span(&State->Worker, sizeof(State->Worker))};
        Out.push_back(std::move(P));
        const auto Before = Out.size();
        State->Querying = true;
        try
        {
            const bool Result = State->Lease->AppendPreparedAccess(Out) && Out.size() > Before;
            State->Querying = false; return Result;
        }
        catch (...) { State->Querying = false; throw; }
    }
    void SetReady() noexcept
    { Require(RestoreLeaseDetail::EPhase::Preparing); State->Phase = RestoreLeaseDetail::EPhase::Ready; }
    void BeginFinalize() noexcept
    { Require(RestoreLeaseDetail::EPhase::Ready); State->Phase = RestoreLeaseDetail::EPhase::Finalizing; }
    void FinishFinalize() noexcept
    { Require(RestoreLeaseDetail::EPhase::Finalizing); State->Phase = RestoreLeaseDetail::EPhase::Finalized; }
    void BeginAbort() noexcept
    {
        if (State->Worker != std::this_thread::get_id() || State->Querying
            || (State->Phase != RestoreLeaseDetail::EPhase::Preparing && State->Phase != RestoreLeaseDetail::EPhase::Ready)) std::terminate();
        State->Phase = RestoreLeaseDetail::EPhase::Aborting;
    }
    void FinishAbort() noexcept
    {
        Require(RestoreLeaseDetail::EPhase::Aborting);
        State->Lease->AbortBeforeFinalize(); State->Lease.reset(); State->Phase = RestoreLeaseDetail::EPhase::Closed;
    }
    void BeginRelease() noexcept
    { Require(RestoreLeaseDetail::EPhase::Finalized); State->Phase = RestoreLeaseDetail::EPhase::Releasing; }
    void FinishRelease() noexcept
    {
        Require(RestoreLeaseDetail::EPhase::Releasing);
        State->Lease->ReleaseAfterPublication(); State->Lease.reset(); State->Phase = RestoreLeaseDetail::EPhase::Closed;
    }
private:
    // No virtual call and no raw-lease access. MatchesPreparedRestore uses this
    // to reject callbacks from externally retained weak views while the world
    // itself is between prepared entry points.
    bool IsCallbackActive() const noexcept
    { return State->Worker != std::this_thread::get_id() || State->Querying; }
    void Require(RestoreLeaseDetail::EPhase Phase) const noexcept
    {
        if (State->Worker != std::this_thread::get_id() || State->Querying || State->Phase != Phase || !State->Lease) std::terminate();
    }
};
}
