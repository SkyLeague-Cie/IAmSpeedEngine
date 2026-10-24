public:
    struct FGroupResumeRequest
    {
        std::uint64_t WorkerGeneration = 0, RegistryVersion = 0, ResumeGeneration = 0;
        FFrameNumber Next = 0;
        std::vector<FSessionOutcome> Sessions; // exact complete sorted set, phase must be Paused
    };
    struct FGroupResumeBaseline
    {
        std::uint64_t Session = 0, Epoch = 0;
        std::shared_ptr<const FInputFrame> Input;
    };
    // Immutable INPUT publication only. A native coordinator must embed this
    // beside a complete prepared world checkpoint in its common publication.
    struct FGroupResumePublication
    {
        FFrameNumber Next = 0;
        std::uint64_t ResumeGeneration = 0;
        std::shared_ptr<const FInputRegistryView> Registry;
        std::vector<FGroupResumeBaseline> Baselines;
    };
    class FGroupResumeToken
    {
        friend class FInputSessionRegistry;
        std::shared_ptr<const std::uint8_t> Identity;
        explicit FGroupResumeToken(std::shared_ptr<const std::uint8_t> I) noexcept : Identity(std::move(I)) {}
    public:
        FGroupResumeToken() = default;
        FGroupResumeToken(const FGroupResumeToken&) = delete;
        FGroupResumeToken& operator=(const FGroupResumeToken&) = delete;
        FGroupResumeToken(FGroupResumeToken&&) noexcept = default;
        FGroupResumeToken& operator=(FGroupResumeToken&&) noexcept = default;
    };
private:
    struct FGroupOwnerResume
    {
        FSameFrameInputOwner* Owner = nullptr;
        FSameFrameInputOwner::FResumeToken Token;
    };
    static_assert(std::is_nothrow_move_constructible_v<FGroupOwnerResume>);
    struct FGroupResumePlan
    {
        std::shared_ptr<const std::uint8_t> Identity;
        std::shared_ptr<const FInputRegistryView> Before, Failed;
        std::shared_ptr<const FGroupResumePublication> Publication;
        std::vector<FGroupOwnerResume> Owners;
        std::optional<std::unique_lock<std::mutex>> CommandFence;
        bool Ready = false, Finalized = false, Published = false, Released = false;
        ~FGroupResumePlan() noexcept
        {
            if (Released) return;
            if (Finalized) std::terminate(); // no undo exists after the no-fail tail begins
            for (auto I = Owners.rbegin(); I != Owners.rend(); ++I)
                if (!I->Owner->AbortPreparedResume(I->Token)) std::terminate();
        }
    };
    bool MatchesGroupResume(const FGroupResumeToken& T) const noexcept
    { return ResumeGroup && T.Identity && T.Identity == ResumeGroup->Identity; }
    bool GroupResumeLane() const noexcept
    { return IsWorker() && !FPresentationInputScope::IsActive() && !BoundaryMutationBlocked(); }
public:
    std::optional<FGroupResumeToken> PrepareGroupResumeAtBoundary(const FGroupResumeRequest& R)
    {
        if (!GroupResumeLane() || ResumeGroup || ReplayGroup || Terminal || Pending || Waiting || Replaying || Sessions.empty()
            || R.WorkerGeneration != WorkerGeneration || R.RegistryVersion != View->Version
            || !R.ResumeGeneration || R.Sessions.size() != Sessions.size()) return {};
        try
        {
            // No queued command/stop can be silently overtaken. New commands
            // arriving during preparation make finalization fail before writes.
            std::lock_guard<std::mutex> Gate(Commands->Gate);
            if (Commands->StopRequested || !Commands->Queue.empty()) return {};
            std::size_t Index = 0;
            for (const auto& P : Sessions)
            {
                const auto& S = P.second;
                const auto& Expected = R.Sessions[Index++];
                if (Expected.Session != P.first || Expected.Epoch != S.Description.Epoch
                    || Expected.Phase != ESessionPhase::Paused || S.Phase != ESessionPhase::Paused
                    || !S.Owner || S.Token || S.Journal || S.Description.Kind == EProducerContract::AI
                    || S.Owner->GetNextFrame() != R.Next || R.ResumeGeneration <= S.ResumeGeneration) return {};
            }
            auto Plan = std::make_unique<FGroupResumePlan>();
            Plan->Identity = std::make_shared<const std::uint8_t>(std::uint8_t{0});
            Plan->Before = ReadRegistry();
            auto Next = std::make_shared<FInputRegistryView>(*Plan->Before);
            for (auto& Phase : Next->Phases) Phase.Phase = ESessionPhase::Active;
            auto Failed = std::make_shared<FInputRegistryView>(*Next); Failed->Terminal = true;
            Plan->Failed = std::move(Failed);
            auto Publication = std::make_shared<FGroupResumePublication>();
            Publication->Next = R.Next; Publication->ResumeGeneration = R.ResumeGeneration; Publication->Registry = std::move(Next);
            Publication->Baselines.reserve(Sessions.size()); Plan->Owners.reserve(Sessions.size());
            for (auto& P : Sessions)
            {
                auto& S = P.second;
                auto Prepared = S.Owner->PrepareResumeAtBoundary(R.Next, R.ResumeGeneration);
                if (!Prepared.Token || Prepared.Status != EOwnerInputStatus::Ready) return {};
                // Capacity was reserved. Moves below are non-throwing. Store
                // the token before any subsequent operation can fail.
                Plan->Owners.push_back({S.Owner.get(), std::move(*Prepared.Token)});
                auto Baseline = S.Owner->ReadPreparedResumeBaseline(Plan->Owners.back().Token);
                if (!Baseline) return {};
                Publication->Baselines.push_back({P.first, S.Description.Epoch, std::move(Baseline)});
            }
            Plan->Publication = std::move(Publication);
            ResumeGroup = std::move(Plan);
            return FGroupResumeToken(ResumeGroup->Identity);
        }
        catch (...) { return {}; } // local Plan aborts every already-reserved owner
    }
    std::shared_ptr<const FGroupResumePublication> ReadPreparedGroupResume(const FGroupResumeToken& T) const noexcept
    { return GroupResumeLane() && MatchesGroupResume(T) ? ResumeGroup->Publication : nullptr; }
    bool PrepareGroupResumeFinalization(const FGroupResumeToken& T)
    {
        if (!GroupResumeLane() || !MatchesGroupResume(T) || ResumeGroup->Finalized) return false;
        if (ResumeGroup->Ready) return true;
        try
        {
            // Retain command fence, then source fences in sorted session order.
            // A failed partial acquisition is aborted as a whole, never retried
            // with some owner locks still held.
            ResumeGroup->CommandFence.emplace(Commands->Gate);
            if (Commands->StopRequested || !Commands->Queue.empty()) { ResumeGroup.reset(); return false; }
            for (auto& O : ResumeGroup->Owners)
                if (!O.Owner->PrepareResumeFinalization(O.Token)) { ResumeGroup.reset(); return false; }
            for (const auto& O : ResumeGroup->Owners)
                if (!O.Owner->IsResumeFinalizationPrepared(O.Token)) { ResumeGroup.reset(); return false; }
            ResumeGroup->Ready = true; return true;
        }
        catch (...) { ResumeGroup.reset(); return false; }
    }
    bool AbortGroupResume(const FGroupResumeToken& T) noexcept
    {
        if (!GroupResumeLane() || !MatchesGroupResume(T) || ResumeGroup->Finalized) return false;
        ResumeGroup.reset(); return true;
    }
    void FinalizePreparedGroupResume(const FGroupResumeToken& T) noexcept
    {
        if (!GroupResumeLane() || !MatchesGroupResume(T) || !ResumeGroup->Ready || ResumeGroup->Finalized) std::terminate();
        for (auto& O : ResumeGroup->Owners) O.Owner->FinalizePreparedResume();
        ResumeGroup->Finalized = true;
        // Sources and owner phases are internal. Registry still publishes the
        // pinned paused view until the explicit publication step below.
    }
    void PublishPreparedGroupResume(const FGroupResumeToken& T) noexcept
    {
        if (!GroupResumeLane() || !MatchesGroupResume(T) || !ResumeGroup->Finalized || ResumeGroup->Published) std::terminate();
        for (auto& P : Sessions)
        { P.second.Phase = ESessionPhase::Active; P.second.ResumeGeneration = ResumeGroup->Publication->ResumeGeneration; }
        TerminalView = ResumeGroup->Failed; View = ResumeGroup->Publication->Registry;
        ResumeGroup->Published = true;
    }
    // The common world/input publication must already have happened. This is
    // deliberately separate from registry publication; it is NOT a world restore.
    void ReleasePreparedGroupResume(const FGroupResumeToken& T) noexcept
    {
        if (!GroupResumeLane() || !MatchesGroupResume(T) || !ResumeGroup->Published) std::terminate();
        for (auto& O : ResumeGroup->Owners) O.Owner->ReleasePreparedResume();
        ResumeGroup->Released = true; ResumeGroup.reset();
    }
public:
