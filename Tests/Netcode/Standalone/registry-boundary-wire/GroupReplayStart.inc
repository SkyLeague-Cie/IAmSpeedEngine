public:
    struct FGroupReplayRequest
    {
        std::uint64_t WorkerGeneration = 0, RegistryVersion = 0;
        FFrameNumber First = 0, Last = 0, LiveNext = 0;
        std::vector<FSessionOutcome> Sessions;
        std::vector<FSessionActor> Actors; // exact sorted input target roster; NOT the complete physical world roster
    };
    struct FGroupReplayPublication
    {
        FFrameNumber First = 0, Last = 0, LiveNext = 0;
        std::shared_ptr<const FInputRegistryView> Registry;
        std::vector<std::shared_ptr<const FRegistryFrame>> Frames;
    };
    class FGroupReplayToken
    {
        friend class FInputSessionRegistry;
        std::shared_ptr<const std::uint8_t> Identity;
        explicit FGroupReplayToken(std::shared_ptr<const std::uint8_t> I) noexcept : Identity(std::move(I)) {}
    public:
        FGroupReplayToken() = default;
        FGroupReplayToken(const FGroupReplayToken&) = delete;
        FGroupReplayToken& operator=(const FGroupReplayToken&) = delete;
        FGroupReplayToken(FGroupReplayToken&&) noexcept = default;
        FGroupReplayToken& operator=(FGroupReplayToken&&) noexcept = default;
    };
private:
    struct FReplayOwner
    {
        FSameFrameInputOwner* Owner = nullptr;
        FSameFrameInputOwner::FPreparedReplayToken Token;
    };
    struct FReplayGroupPlan
    {
        std::shared_ptr<const std::uint8_t> Identity;
        std::shared_ptr<const FInputRegistryView> Before;
        std::shared_ptr<const FGroupReplayPublication> Publication;
        std::vector<std::shared_ptr<const FRegistryFrame>> Frames;
        std::vector<FReplayOwner> Owners;
        std::optional<std::unique_lock<std::mutex>> CommandFence;
        bool Ready = false, Finalized = false, Published = false, Released = false;
        ~FReplayGroupPlan() noexcept
        {
            if (Released) return;
            if (Finalized) std::terminate();
            for (auto I = Owners.rbegin(); I != Owners.rend(); ++I)
                if (!I->Owner->AbortRetainedReplayStart(I->Token)) std::terminate();
        }
    };
    bool MatchesGroupReplay(const FGroupReplayToken& T) const noexcept
    { return ReplayGroup && T.Identity && T.Identity == ReplayGroup->Identity; }
    bool GroupReplayLane() const noexcept { return IsWorker() && !FPresentationInputScope::IsActive() && !BoundaryMutationBlocked(); }
public:
    // INPUT history transaction only. Native world state, lifecycle restoration
    // and causal FastSimulation remain refused until joined by a real coordinator.
    std::optional<FGroupReplayToken> PrepareGroupRetainedReplayStart(const FGroupReplayRequest& R)
    {
        if (!GroupReplayLane() || ReplayGroup || ResumeGroup || Terminal || Pending || Waiting || Replaying
            || Sessions.empty() || R.WorkerGeneration != WorkerGeneration || R.RegistryVersion != View->Version
            || R.Sessions.size() != Sessions.size() || R.First > R.Last || R.Last >= R.LiveNext
            || R.Last - R.First >= History.size()) return {};
        try
        {
            std::lock_guard<std::mutex> Gate(Commands->Gate);
            if (Commands->StopRequested || !Commands->Queue.empty()) return {};
            std::size_t Index = 0; std::vector<FSessionActor> Actors;
            for (const auto& P : Sessions)
            {
                const auto& S = P.second; const auto& E = R.Sessions[Index++];
                if (E.Session != P.first || E.Epoch != S.Description.Epoch || E.Phase != ESessionPhase::Active
                    || S.Phase != ESessionPhase::Active || !S.Owner || S.Token || S.Owner->GetNextFrame() != R.LiveNext) return {};
                Actors.insert(Actors.end(), S.Description.Actors.begin(), S.Description.Actors.end());
            }
            std::sort(Actors.begin(), Actors.end());
            if (Actors != R.Actors) return {};
            for (std::size_t I = 0; I < Actors.size(); ++I)
                if (!Actors[I].Id || !Actors[I].Generation || (I && Actors[I-1].Id >= Actors[I].Id)) return {};
            auto Plan = std::make_unique<FReplayGroupPlan>();
            Plan->Identity = std::make_shared<const std::uint8_t>(std::uint8_t{0}); Plan->Before = ReadRegistry();
            Plan->Frames.reserve(std::size_t(R.Last - R.First + 1)); Plan->Owners.reserve(Sessions.size());
            for (auto N = R.First;; ++N)
            {
                auto F = ReadHistorical(N);
                if (!F || !F->Registry || F->Registry->Version != View->Version || F->Registry->WorkerGeneration != WorkerGeneration
                    || F->Inputs.size() != Sessions.size() || F->Registry->Bindings.size() != Sessions.size()) return {};
                Index = 0;
                for (const auto& P : Sessions)
                {
                    const auto& Input = F->Inputs[Index++]; const auto& S = P.second;
                    if (Input.Session != P.first || Input.Epoch != S.Description.Epoch || Input.Actors != S.Description.Actors
                        || !Input.Snapshot || Input.AlreadyAppliedAtInitialBoundary) return {};
                    auto Receipt = S.Owner->ReadCompleted(N);
                    if (!Receipt || Receipt->Replay || Receipt->Snapshot != Input.Snapshot) return {};
                }
                Plan->Frames.push_back(std::move(F)); if (N == R.Last) break;
            }
            auto Pub = std::make_shared<FGroupReplayPublication>(); Pub->First = R.First; Pub->Last = R.Last;
            Pub->LiveNext = R.LiveNext; Pub->Registry = Plan->Before; Pub->Frames = Plan->Frames;
            for (auto& P : Sessions)
            {
                auto T = P.second.Owner->PrepareRetainedReplayStart(R.First, R.Last); if (!T) return {};
                Plan->Owners.push_back({P.second.Owner.get(), std::move(*T)});
                auto V = P.second.Owner->ReadPreparedReplayStart(Plan->Owners.back().Token);
                if (!V || V->LiveNext != R.LiveNext || V->Frames.size() != Plan->Frames.size()) return {};
            }
            Plan->Publication = std::move(Pub); ReplayGroup = std::move(Plan);
            return FGroupReplayToken(ReplayGroup->Identity);
        }
        catch (...) { return {}; }
    }
    std::shared_ptr<const FGroupReplayPublication> ReadPreparedGroupReplay(const FGroupReplayToken& T) const noexcept
    { return GroupReplayLane() && MatchesGroupReplay(T) ? ReplayGroup->Publication : nullptr; }
    bool PrepareGroupReplayFinalization(const FGroupReplayToken& T)
    {
        if (!GroupReplayLane() || !MatchesGroupReplay(T) || ReplayGroup->Finalized) return false;
        if (ReplayGroup->Ready) return true;
        try
        {
            ReplayGroup->CommandFence.emplace(Commands->Gate);
            if (Commands->StopRequested || !Commands->Queue.empty()) { ReplayGroup.reset(); return false; }
            for (auto& O : ReplayGroup->Owners)
                if (!O.Owner->PrepareRetainedReplayFinalization(O.Token)) { ReplayGroup.reset(); return false; }
            ReplayGroup->Ready = true; return true;
        }
        catch (...) { ReplayGroup.reset(); return false; }
    }
    bool AbortGroupReplay(const FGroupReplayToken& T) noexcept
    {
        if (!GroupReplayLane() || !MatchesGroupReplay(T) || ReplayGroup->Finalized) return false;
        ReplayGroup.reset(); return true;
    }
    void FinalizePreparedGroupReplay(const FGroupReplayToken& T) noexcept
    {
        if (!GroupReplayLane() || !MatchesGroupReplay(T) || !ReplayGroup->Ready || ReplayGroup->Finalized) std::terminate();
        for (auto& O : ReplayGroup->Owners) O.Owner->FinalizeRetainedReplayStart(O.Token);
        ReplayPlan.swap(ReplayGroup->Frames); ReplayIndex = 0; Replaying = true; GroupExecutionReplay = true;
        ReplayGroup->Finalized = true;
    }
    void PublishPreparedGroupReplay(const FGroupReplayToken& T) noexcept
    {
        if (!GroupReplayLane() || !MatchesGroupReplay(T) || !ReplayGroup->Finalized || ReplayGroup->Published) std::terminate();
        GroupReplayPublished = ReplayGroup->Publication; GroupReplayReceipt.reset(); ReplayGroup->Published = true;
    }
    void ReleasePreparedGroupReplay(const FGroupReplayToken& T) noexcept
    {
        if (!GroupReplayLane() || !MatchesGroupReplay(T) || !ReplayGroup->Published) std::terminate();
        for (auto& O : ReplayGroup->Owners) O.Owner->ReleaseRetainedReplayStart(O.Token);
        ReplayGroup->Released = true; ReplayGroup.reset();
    }
    std::shared_ptr<const FGroupReplayPublication> ReadGroupReplayPublication() const noexcept
    { return GroupReplayLane() && !Terminal ? GroupReplayPublished : nullptr; }
    bool PrepareGroupReplayFrame(FFrameNumber N)
    {
        if (!GroupReplayLane() || ReplayGroup || ResumeGroup || !GroupExecutionReplay || !Replaying || Terminal || Pending
            || ReplayIndex >= ReplayPlan.size() || ReplayPlan[ReplayIndex]->Frame != N) return false;
        try
        {
            auto Frame = std::make_shared<FRegistryFrame>(); Frame->Frame = N; Frame->Registry = ReplayPlan[ReplayIndex]->Registry;
            Frame->Inputs.reserve(Sessions.size());
            for (auto& P : Sessions)
            {
                auto& S = P.second; auto V = S.Owner->Poll(N);
                if (!V.Token || V.Status != EOwnerInputStatus::Ready) { AbortFrame(); return false; }
                S.Token = std::move(V.Token);
                Frame->Inputs.push_back({P.first, S.Description.Epoch, {}, S.Description.Actors, false, S.Owner->ReadPreparedFilterBefore()});
            }
            Pending = std::move(Frame); return true;
        }
        catch (...) { AbortFrame(); return false; }
    }
    std::shared_ptr<const FRegistryFrame> ReadGroupReplayReceipt() const noexcept
    { return GroupReplayLane() && GroupExecutionReplay && !Terminal ? GroupReplayReceipt : nullptr; }
    bool EndGroupRetainedReplay() noexcept
    {
        if (!GroupReplayLane() || ReplayGroup || Pending || Terminal || !GroupExecutionReplay || !Replaying || ReplayIndex != ReplayPlan.size()) return false;
        for (auto& P : Sessions) if (!P.second.Owner->EndReplay()) { MakeTerminal(); return false; }
        GroupExecutionReplay = Replaying = false; ReplayPlan.clear(); GroupReplayReceipt.reset(); GroupReplayPublished.reset(); return true;
    }
public:
