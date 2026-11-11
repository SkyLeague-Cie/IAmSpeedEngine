public:
    // Historical INPUT evidence only, minted from this registry's actual ring
    // while a genuine completed-boundary reservation is held. This proves no
    // world step, other command channel, snapshot/branch anchor or native Next.
    // This first seam requires ONE exact immutable historical registry view
    // throughout the interval; transitions between versions remain unsupported.
    // The current boundary may have a later version and is never substituted.
    class FEmptyInputPeriodEvidence final
    {
        friend class FInputSessionRegistry;
        const std::shared_ptr<const FCompletedBoundaryCapture> Boundary;
        const std::shared_ptr<const std::uint8_t> ReservationIdentity;
        const FFrameNumber First, Last;
        const std::vector<std::shared_ptr<const FRegistryFrame>> Frames;
        FEmptyInputPeriodEvidence(std::shared_ptr<const FCompletedBoundaryCapture> B,
            std::shared_ptr<const std::uint8_t> R, FFrameNumber F, FFrameNumber L,
            std::vector<std::shared_ptr<const FRegistryFrame>> V) noexcept
            : Boundary(std::move(B)), ReservationIdentity(std::move(R)), First(F), Last(L), Frames(std::move(V)) {}
    public:
        FEmptyInputPeriodEvidence(const FEmptyInputPeriodEvidence&) = delete;
        FFrameNumber FirstFrame() const noexcept { return First; }
        FFrameNumber LastFrame() const noexcept { return Last; }
        const auto& HistoricalFrames() const noexcept { return Frames; }
        const auto& ObservedInputBoundary() const noexcept { return Boundary; }
    };
private:
    bool EmptyPeriodBoundaryValid(const std::shared_ptr<const FCompletedBoundaryCapture>& C,
        FFrameNumber First, FFrameNumber Last) const noexcept
    {
        return C && C->Completed() && C->CurrentBoundary() && !Terminal && Sessions.empty()
            && C->CurrentBoundary()->WorkerGeneration == WorkerGeneration && !C->CurrentBoundary()->Terminal
            && C->CurrentBoundary()->Bindings.empty() && C->CurrentBoundary()->Phases.empty()
            && First <= Last && Last != std::numeric_limits<FFrameNumber>::max()
            && Last - First < History.size() && Last <= C->Completed()->Frame;
    }
    bool EmptyPeriodFrameValid(const std::shared_ptr<const FRegistryFrame>& F, FFrameNumber N,
        const std::shared_ptr<const FInputRegistryView>& ExpectedView, std::uint64_t CurrentVersion) const noexcept
    {
        return F && F->Frame == N && F->Registry && !F->Registry->Terminal
            && F->Registry->WorkerGeneration == WorkerGeneration && F->Inputs.empty()
            && F->Registry->Bindings.empty() && F->Registry->Phases.empty()
            && (!ExpectedView || F->Registry == ExpectedView) && F->Registry->Version <= CurrentVersion;
    }
public:
    bool ValidateReservedEmptyInputPeriod(const FCompletedBoundaryReservation& T,
        const FEmptyInputPeriodEvidence& E) const noexcept
    {
        const auto C = ReadReservedCompletedBoundary(T); // Reject foreign thread before worker-owned fields.
        if (!C || C != E.Boundary || T.Identity != E.ReservationIdentity
            || !EmptyPeriodBoundaryValid(C, E.First, E.Last)
            || E.Frames.size() != static_cast<std::size_t>(E.Last - E.First + 1)) return false;
        std::shared_ptr<const FInputRegistryView> HistoricalView;
        for (std::size_t I = 0; I < E.Frames.size(); ++I)
        {
            const auto N = E.First + static_cast<FFrameNumber>(I);
            const auto& F = E.Frames[I];
            if (ReadHistorical(N) != F || !EmptyPeriodFrameValid(F, N, HistoricalView, C->CurrentBoundary()->Version)) return false;
            HistoricalView = F->Registry;
        }
        return true;
    }
    std::shared_ptr<const FEmptyInputPeriodEvidence> CaptureReservedEmptyInputPeriod(
        const FCompletedBoundaryReservation& T, FFrameNumber First, FFrameNumber Last) const
    {
        const auto C = ReadReservedCompletedBoundary(T);
        if (!C || !EmptyPeriodBoundaryValid(C, First, Last)) return {};
        const auto EntryReservationIdentity = T.Identity; // Pin before any allocator can reenter.
        try
        {
            std::vector<std::shared_ptr<const FRegistryFrame>> Frames;
            Frames.reserve(static_cast<std::size_t>(Last - First + 1));
            std::shared_ptr<const FInputRegistryView> HistoricalView;
            for (auto N = First;; ++N)
            {
                const auto F = ReadHistorical(N);
                if (!EmptyPeriodFrameValid(F, N, HistoricalView, C->CurrentBoundary()->Version)) return {};
                HistoricalView = F->Registry;
                Frames.push_back(F);
                if (N == Last) break;
            }
            auto Evidence = std::shared_ptr<const FEmptyInputPeriodEvidence>(new FEmptyInputPeriodEvidence(
                C, EntryReservationIdentity, First, Last, std::move(Frames)));
            // An allocator can reenter and abort/re-reserve the same boundary.
            // Revalidate the exact reservation nonce as well as the frame refs.
            if (!ValidateReservedEmptyInputPeriod(T, *Evidence)) return {};
            return Evidence;
        }
        catch (...) { return {}; } // No polling, source/history mutation or publication.
    }
    // Reject First>Last; a zero-step restore is a separate checkpoint-anchor
    // contract. Missing/overwritten frames never become a fabricated empty row.
    // After reservation release, retained frames are historical data only.
