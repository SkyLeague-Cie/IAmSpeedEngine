public:
    struct FPreparedReplayView
    {
        FFrameNumber First = 0, Last = 0, LiveNext = 0;
        std::uint64_t ReplayPass = 0;
        FContractFingerprint Binding;
        std::vector<std::shared_ptr<const FOwnerInputReceipt>> Frames;
    };
    class FPreparedReplayToken
    {
        friend class FSameFrameInputOwner;
        std::shared_ptr<const std::uint8_t> Identity;
        explicit FPreparedReplayToken(std::shared_ptr<const std::uint8_t> I) noexcept : Identity(std::move(I)) {}
    public:
        FPreparedReplayToken() = default;
        FPreparedReplayToken(const FPreparedReplayToken&) = delete;
        FPreparedReplayToken& operator=(const FPreparedReplayToken&) = delete;
        FPreparedReplayToken(FPreparedReplayToken&&) noexcept = default;
        FPreparedReplayToken& operator=(FPreparedReplayToken&&) noexcept = default;
    };
private:
    struct FPreparedReplayStart
    {
        std::shared_ptr<const std::uint8_t> Identity;
        std::shared_ptr<const FPreparedReplayView> View;
        FActionValues FilterBefore{};
        std::uint64_t Fault = 0, Poll = 0;
        bool Ready = false, Finalized = false;
    };
    bool MatchesReplayStart(const FPreparedReplayToken& T) const noexcept
    { return ReplayStart && T.Identity && T.Identity == ReplayStart->Identity; }
    bool ReplayStartLane() const noexcept
    { return IsOwner() && !FPresentationInputScope::IsActive() && !CallingSource; }
    bool ReplayStartPinned(const FPreparedReplayToken& T) const noexcept
    {
        return ReplayStartLane() && MatchesReplayStart(T) && !ReplayStart->Finalized && !Replay && !Pending && !ResumePending
            && !Initial && Phase == EOwnerInputPhase::Idle && FaultSerial == ReplayStart->Fault
            && PollSerial == ReplayStart->Poll && NextFrame == ReplayStart->View->LiveNext
            && ReplayPass != UINT64_MAX && ReplayPass + 1 == ReplayStart->View->ReplayPass;
    }
public:
    // Historical INPUT replay only. A native coordinator must first prepare the
    // complete authoritative world at P, then publish its transaction together
    // with all owner/registry replay starts. This never pauses or polls Source.
    std::optional<FPreparedReplayToken> PrepareRetainedReplayStart(FFrameNumber First, FFrameNumber Last)
    {
        if (!ReplayStartLane() || ReplayStart || ResumePending || Pending || Initial || Replay
            || Phase != EOwnerInputPhase::Idle || First > Last || Last - First >= HistoryCapacity
            || ReplayPass == UINT64_MAX) return {};
        try
        {
            auto View = std::make_shared<FPreparedReplayView>();
            View->First = First; View->Last = Last; View->LiveNext = NextFrame; View->ReplayPass = ReplayPass + 1;
            View->Binding = Fingerprint; View->Frames.reserve(static_cast<std::size_t>(Last - First + 1));
            FActionValues FilterBefore{};
            for (auto N = First;; ++N)
            {
                const auto& E = History[N % HistoryCapacity];
                if (!E || !E->Snapshot || !E->Receipt || E->Snapshot->Input.GetData().ConsumptionFrame != N
                    || E->Snapshot->BindingFingerprint != Fingerprint || E->Receipt->Snapshot != E->Snapshot) return {};
                if (Filter && !E->Before) return {};
                if (N == First && Filter) FilterBefore = *E->Before;
                View->Frames.push_back(E->Receipt);
                if (N == Last) break;
            }
            auto Identity = std::make_shared<const std::uint8_t>(std::uint8_t{0});
            ReplayStart.emplace(FPreparedReplayStart{std::move(Identity), std::move(View), FilterBefore, FaultSerial, PollSerial, false, false});
            return FPreparedReplayToken(ReplayStart->Identity);
        }
        catch (...) { return {}; }
    }
    std::shared_ptr<const FPreparedReplayView> ReadPreparedReplayStart(const FPreparedReplayToken& T) const noexcept
    { return ReplayStartLane() && MatchesReplayStart(T) ? ReplayStart->View : nullptr; }
    bool PrepareRetainedReplayFinalization(const FPreparedReplayToken& T) noexcept
    {
        if (!ReplayStartPinned(T)) return false;
        ReplayStart->Ready = true; return true;
    }
    bool AbortRetainedReplayStart(const FPreparedReplayToken& T) noexcept
    {
        if (!ReplayStartLane() || !MatchesReplayStart(T) || ReplayStart->Finalized) return false;
        ReplayStart.reset(); return true;
    }
    void FinalizeRetainedReplayStart(const FPreparedReplayToken& T) noexcept
    {
        if (!ReplayStartPinned(T) || !ReplayStart->Ready) std::terminate();
        if (Filter) Filter->Replay = ReplayStart->FilterBefore;
        Replay = true; ReplayFinished = false; ReplayNext = ReplayStart->View->First; ReplayEnd = ReplayStart->View->Last;
        ReplayPass = ReplayStart->View->ReplayPass; ReplayReceipt.reset(); ReplayStart->Finalized = true;
        // Source cursor/lifecycle, NextFrame, LastInput, normal Latest/History,
        // applied live filter and PollSerial are unchanged. Poll stays Busy.
    }
    // Caller has published the common world+registry+owner replay-start view.
    void ReleaseRetainedReplayStart(const FPreparedReplayToken& T) noexcept
    {
        if (!ReplayStartLane() || !MatchesReplayStart(T) || !ReplayStart->Finalized) std::terminate();
        ReplayStart.reset();
    }
public:
