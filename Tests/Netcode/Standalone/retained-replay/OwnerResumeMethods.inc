    // A resume reservation is not a physical input token or a completed frame.
    class FResumeToken
    {
        friend class FSameFrameInputOwner;
        std::shared_ptr<const std::uint8_t> Identity;
        explicit FResumeToken(std::shared_ptr<const std::uint8_t> I) noexcept : Identity(std::move(I)) {}
    public:
        FResumeToken() = default;
        FResumeToken(const FResumeToken&) = delete;
        FResumeToken& operator=(const FResumeToken&) = delete;
        FResumeToken(FResumeToken&&) noexcept = default;
        FResumeToken& operator=(FResumeToken&&) noexcept = default;
    };
    struct FResumePreparation
    {
        EOwnerInputStatus Status = EOwnerInputStatus::InvalidBinding;
        std::optional<FResumeToken> Token;
    };
    FResumePreparation PrepareResumeAtBoundary(FFrameNumber N, std::uint64_t Generation)
    {
        if (!IsOwner()) return {EOwnerInputStatus::WrongOwner, {}};
        if (FPresentationInputScope::IsActive() || CallingSource || ReplayStart || Pending || ResumePending || Replay || Initial
            || Phase != EOwnerInputPhase::Paused || Exhausted) return {EOwnerInputStatus::Busy, {}};
        if (N != NextFrame) return {EOwnerInputStatus::WrongFrame, {}};
        if (!Generation || Generation <= LastResumeGeneration) return {EOwnerInputStatus::InvalidToken, {}};
        const auto BeforeFault = FaultSerial;
        std::unique_ptr<IPreparedInputResume> Reservation;
        CallingSource = true;
        try
        {
            Reservation = Source->PrepareResumeAtBoundary(N);
            if (!Reservation) { CallingSource = false; return {EOwnerInputStatus::ResyncRequired, {}}; }
            auto Baseline = std::make_shared<const FInputFrame>(Reservation->Baseline());
            if (Phase != EOwnerInputPhase::Paused || FaultSerial != BeforeFault
                || !Baseline->IsValidFor(*Binding.Contract) || !ValidateAddress(*Baseline, N)
                || !Baseline->GetData().Reset || !ValidateContinuity(*Baseline))
            {
                Reservation->Abort(); Reservation.reset(); CallingSource = false;
                return {EOwnerInputStatus::ResyncRequired, {}};
            }
            auto Identity = std::make_shared<const std::uint8_t>(std::uint8_t{0});
            static_assert(std::is_nothrow_move_constructible_v<FPendingResume>);
            ResumePending.emplace(FPendingResume{std::move(Identity), std::move(Reservation), std::move(Baseline),
                N, Generation, BeforeFault, PollSerial, false, false});
            CallingSource = false;
            return {EOwnerInputStatus::Ready, FResumeToken(ResumePending->Identity)};
        }
        catch (...)
        {
            if (Reservation) { Reservation->Abort(); Reservation.reset(); }
            CallingSource = false; return {EOwnerInputStatus::ResyncRequired, {}};
        }
    }
    std::shared_ptr<const FInputFrame> ReadPreparedResumeBaseline(const FResumeToken& T) const noexcept
    { return ResumeAllowed(T) ? ResumePending->Baseline : nullptr; }
    bool PrepareResumeFinalization(const FResumeToken& T)
    {
        if (!ResumeAllowed(T) || ResumePending->Finalized) return false;
        if (ResumePending->FinalizationReady) return true;
        bool Ready = false; CallingSource = true;
        try { Ready = ResumePending->Reservation->PrepareFinalization(); } catch (...) {}
        CallingSource = false;
        if (!ResumeAllowed(T) || !Ready) return false;
        ResumePending->FinalizationReady = true; return true;
    }
    bool IsResumeFinalizationPrepared(const FResumeToken& T) const noexcept
    { return ResumeAllowed(T) && ResumePending->FinalizationReady && !ResumePending->Finalized; }
    // Precondition is pinned by the owner-thread reservation and retained source
    // fence. All other owner mutators refuse ResumePending. Global coordinator
    // must finalize every participant and publish before calling Release.
    void FinalizePreparedResume() noexcept
    {
        if (!IsOwner() || FPresentationInputScope::IsActive() || CallingSource || !ResumePending
            || !ResumePending->FinalizationReady || ResumePending->Finalized || Phase != EOwnerInputPhase::Paused
            || NextFrame != ResumePending->Frame || FaultSerial != ResumePending->Fault || PollSerial != ResumePending->Poll)
            std::terminate(); // invariant failure is never a fictitious undo/retry
        CallingSource = true;
        ResumePending->Reservation->Finalize();
        ResumePending->Finalized = true;
        LastResumeGeneration = ResumePending->Generation; RequireBaseline = true; Phase = EOwnerInputPhase::Idle;
        CallingSource = false;
        // NextFrame, LastInput, History, PollSerial and input bytes are untouched.
        // Poll remains Busy until the coordinator releases after publication.
    }
    void ReleasePreparedResume() noexcept
    {
        if (!IsOwner() || FPresentationInputScope::IsActive() || CallingSource || !ResumePending || !ResumePending->Finalized)
            std::terminate();
        CallingSource = true; ResumePending->Reservation->Release(); ResumePending.reset(); CallingSource = false;
    }
    bool AbortPreparedResume(const FResumeToken& T) noexcept
    {
        if (!IsOwner() || FPresentationInputScope::IsActive() || CallingSource || !MatchesResume(T) || ResumePending->Finalized) return false;
        CallingSource = true; ResumePending->Reservation->Abort(); ResumePending.reset(); CallingSource = false;
        return true;
    }
