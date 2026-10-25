public:
    class FCompletedBoundaryReservation final
    {
        friend class FInputSessionRegistry;
        std::shared_ptr<const std::uint8_t> Identity;
        explicit FCompletedBoundaryReservation(std::shared_ptr<const std::uint8_t> I) noexcept : Identity(std::move(I)) {}
    public:
        FCompletedBoundaryReservation() = default;
        FCompletedBoundaryReservation(const FCompletedBoundaryReservation&) = delete;
        FCompletedBoundaryReservation& operator=(const FCompletedBoundaryReservation&) = delete;
        FCompletedBoundaryReservation(FCompletedBoundaryReservation&&) noexcept = default;
        FCompletedBoundaryReservation& operator=(FCompletedBoundaryReservation&&) noexcept = default;
    };
    // Local INPUT provenance, not an owner/branch/world identity. Retaining
    // this observation does NOT retain the command fence. A later reservation
    // of the identical Capture is a different transaction and cannot revive it.
    class FCompletedBoundaryReservationObservation final
    {
        friend class FInputSessionRegistry;
        const std::shared_ptr<const std::uint8_t> Identity;
        const std::shared_ptr<const FCompletedBoundaryCapture> Boundary;
        FCompletedBoundaryReservationObservation(std::shared_ptr<const std::uint8_t> I,
            std::shared_ptr<const FCompletedBoundaryCapture> C) noexcept
            : Identity(std::move(I)), Boundary(std::move(C)) {}
    public:
        FCompletedBoundaryReservationObservation(const FCompletedBoundaryReservationObservation&) noexcept = default;
        const auto& Capture() const noexcept { return Boundary; }
    };
private:
    struct FBoundaryReservationPlan
    {
        const std::shared_ptr<const std::uint8_t> Identity;
        const std::shared_ptr<const FCompletedBoundaryCapture> Capture;
        const std::shared_ptr<FInputSessionCommands> CommandOwner; // outlives the retained mutex lock
        std::unique_lock<std::mutex> CommandFence;
        FBoundaryReservationPlan(std::shared_ptr<const std::uint8_t> I,
            std::shared_ptr<const FCompletedBoundaryCapture> C,
            std::shared_ptr<FInputSessionCommands> O, std::unique_lock<std::mutex> L) noexcept
            : Identity(std::move(I)), Capture(std::move(C)), CommandOwner(std::move(O)), CommandFence(std::move(L)) {}
    };
    bool BoundaryMutationBlocked() const noexcept
    { return BoundaryCaptureCalling || BoundaryReservationCalling || bool(BoundaryReservation); }
    bool MatchesBoundaryReservation(const FCompletedBoundaryReservation& T) const noexcept
    { return BoundaryReservation && T.Identity && T.Identity==BoundaryReservation->Identity; }
public:
    // Only the registry worker may retain this reservation. It prevents queued
    // commands and all registry-owned owner mutations from changing the genuine
    // closed INPUT/current-boundary pair while a future coordinator prepares its
    // other participants. No world step, source fence or native completion is
    // proved by this reservation; actor/lifetime/config/world fences are separate.
    // The registry must remain alive until explicit release/abort. Its destructor
    // enforces this precondition. Direct command-bridge calls on this same worker
    // would recursively lock a non-recursive mutex and are forbidden while held;
    // foreign publisher calls wait and serialize after release, rather than reject.
    std::optional<FCompletedBoundaryReservation> ReserveCompletedBoundary(
        std::shared_ptr<const FCompletedBoundaryCapture> Capture)
    {
        if (!Capture || !ValidateCompletedBoundaryCapture(*Capture)) return {};
        BoundaryReservationCalling=true;
        struct FCallingScope
        {
            bool& Flag;
            ~FCallingScope() noexcept { Flag=false; }
        } Calling{BoundaryReservationCalling};
        try
        {
            std::unique_lock<std::mutex> Fence(Commands->Gate);
            // A publisher may have submitted a command after validation's
            // observation. Do not silently overtake it at lock acquisition.
            if (Commands->StopRequested || !Commands->Queue.empty() || Commands->HighId!=Capture->CommandHighId) return {};
            const auto Identity=std::make_shared<const std::uint8_t>(std::uint8_t{0});
            auto Plan=std::make_unique<FBoundaryReservationPlan>(Identity,std::move(Capture),Commands,std::move(Fence));
            BoundaryReservation=std::move(Plan);
            return FCompletedBoundaryReservation(Identity);
        }
        catch (...) { return {}; } // Local lock/plan releases before the worker gate opens.
    }
    std::shared_ptr<const FCompletedBoundaryCapture> ReadReservedCompletedBoundary(const FCompletedBoundaryReservation& T) const noexcept
    {
        return IsWorker() && !FPresentationInputScope::IsActive() && MatchesBoundaryReservation(T)
            ? BoundaryReservation->Capture : nullptr;
    }
    // Pin BOTH identities before any exporter/provider allocation can reenter.
    // Construction and validation allocate nothing and touch no source/owner.
    std::optional<FCompletedBoundaryReservationObservation> ObserveCompletedBoundaryReservation(
        const FCompletedBoundaryReservation& T) const noexcept
    {
        const auto Capture=ReadReservedCompletedBoundary(T);
        if (!Capture) return {};
        return FCompletedBoundaryReservationObservation(T.Identity,Capture);
    }
    bool ValidateCompletedBoundaryReservationObservation(const FCompletedBoundaryReservation& T,
        const FCompletedBoundaryReservationObservation& O) const noexcept
    {
        const auto Capture=ReadReservedCompletedBoundary(T);
        return Capture && Capture==O.Boundary && T.Identity==O.Identity;
    }
    // Aborting changes no consumed input, owner filter, registry publication or
    // source journal. A token alone never keeps the live registry reserved.
    bool AbortCompletedBoundaryReservation(const FCompletedBoundaryReservation& T) noexcept
    {
        if (!IsWorker() || FPresentationInputScope::IsActive() || !MatchesBoundaryReservation(T)) return false;
        BoundaryReservation.reset(); return true;
    }
    // Call only after the coordinator's common publication, or use Abort above.
    // This no-allocation tail has no callbacks and creates no LOCAL/world receipt.
    void ReleaseCompletedBoundaryReservation(const FCompletedBoundaryReservation& T) noexcept
    {
        if (!IsWorker() || FPresentationInputScope::IsActive() || !MatchesBoundaryReservation(T)) std::terminate();
        BoundaryReservation.reset();
    }
public:
