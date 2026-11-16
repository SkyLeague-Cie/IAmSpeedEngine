public:
    // Authentic INPUT metadata only: a completed cohort N and the independently
    // current registry view observed at one worker boundary. The current view
    // is not inferred from the actors that survived N and is not a prediction
    // of commands that may arrive before the next step.
    class FCompletedBoundaryCapture final
    {
    public:
        const std::shared_ptr<const FRegistryFrame>& Completed() const noexcept { return Frame; }
        const std::shared_ptr<const FInputRegistryView>& CurrentBoundary() const noexcept { return Current; }
        // Arithmetic in the INPUT history domain only. Worlds can advance with
        // zero input sessions; this is not an observed native driver's Next.
        FFrameNumber FrameFollowingCapturedInput() const noexcept { return Frame->Frame + 1; }
        FCompletedBoundaryCapture(const FCompletedBoundaryCapture&) = delete;
    private:
        friend class FInputSessionRegistry;
        FCompletedBoundaryCapture(std::shared_ptr<const std::uint8_t> O,
            std::shared_ptr<const FRegistryFrame> F,std::shared_ptr<const FInputRegistryView> Root,
            std::shared_ptr<const FInputRegistryView> C,std::uint64_t H)
            : Origin(std::move(O)),Frame(std::move(F)),RootView(std::move(Root)),Current(std::move(C)),CommandHighId(H) {}
        const std::shared_ptr<const std::uint8_t> Origin;
        const std::shared_ptr<const FRegistryFrame> Frame;
        const std::shared_ptr<const FInputRegistryView> RootView,Current;
        const std::uint64_t CommandHighId;
    };
    std::shared_ptr<const FCompletedBoundaryCapture> CaptureCompletedBoundary(FFrameNumber CompletedFrame) const
    {
        if(!CanCaptureCompletedBoundary(CompletedFrame))return {};
        BoundaryCaptureCalling=true;
        struct FCaptureScope
        {
            bool& Calling;
            ~FCaptureScope() noexcept { Calling=false; }
        } Calling{BoundaryCaptureCalling};
        try
        {
            const auto Completed=LastCompletedForCapture;
            const auto CurrentRoot=View;
            std::uint64_t HighId=0;
            {
                std::lock_guard<std::mutex> Lock(Commands->Gate);
                if(Commands->StopRequested || !Commands->Queue.empty())return {};
                HighId=Commands->HighId;
            }
            // No allocation or potentially final shared_ptr destruction under
            // Gate. A reentrant/concurrent publisher remains free to submit;
            // worker mutations/recursive captures are denied by Calling above.
            const auto Current=ReadRegistry(); // Copies actual phases, without source polling.
            if(!Current || !Completed || Completed->Frame!=CompletedFrame || !Completed->Registry)return {};
            auto Candidate=std::shared_ptr<const FCompletedBoundaryCapture>(new FCompletedBoundaryCapture(
                BoundaryCaptureOrigin,Completed,CurrentRoot,Current,HighId));
            // This lock is destroyed BEFORE Candidate/Current on any return.
            std::lock_guard<std::mutex> Lock(Commands->Gate);
            if(Commands->StopRequested || !Commands->Queue.empty() || Commands->HighId!=HighId
                || !CompletedCaptureWorkerStateMatches(*Candidate))return {};
            return Candidate; // Observation only: no command fence survives return.
        }
        catch(...) { return {}; } // No input token, registry publication or gameplay state changed.
    }
    bool ValidateCompletedBoundaryCapture(const FCompletedBoundaryCapture& C) const
    {
        if(!CanCaptureCompletedBoundary(C.Frame->Frame) || !CompletedCaptureWorkerStateMatches(C))return false;
        std::lock_guard<std::mutex> Lock(Commands->Gate);
        return !Commands->StopRequested && Commands->Queue.empty() && Commands->HighId==C.CommandHighId;
    }
private:
    bool CompletedCaptureWorkerStateMatches(const FCompletedBoundaryCapture& C) const noexcept
    {
        if(!CompletedBoundaryWorkerStateAvailable(C.Frame->Frame) || C.Origin!=BoundaryCaptureOrigin
            || LastCompletedForCapture!=C.Frame || View!=C.RootView || !C.Current || C.Current->Terminal
            || C.Current->Version!=View->Version || C.Current->WorkerGeneration!=WorkerGeneration
            || C.Current->Phases.size()!=Sessions.size())return false;
        std::size_t Index=0;
        for(const auto& P:Sessions)
        {
            const auto& Phase=C.Current->Phases[Index++];
            if(Phase.Session!=P.first || Phase.Epoch!=P.second.Description.Epoch || Phase.Phase!=P.second.Phase)return false;
        }
        return true;
    }
    bool CompletedBoundaryWorkerStateAvailable(FFrameNumber N) const noexcept
    {
        return IsWorker() && !FPresentationInputScope::IsActive() && !BoundaryReservationCalling && !BoundaryReservation && !Terminal && !Pending && !Waiting
            && !Replaying && !GroupExecutionReplay && !ResumeGroup && !ReplayGroup
            && LastCompletedForCapture && LastCompletedForCapture->Registry && LastCompletedForCapture->Frame==N
            && N!=std::numeric_limits<FFrameNumber>::max();
    }
    bool CanCaptureCompletedBoundary(FFrameNumber N) const noexcept
    {
        // Reject on origin lane BEFORE reading any worker-owned gate.
        return IsWorker() && !BoundaryMutationBlocked() && CompletedBoundaryWorkerStateAvailable(N);
    }
    // This is a read observation, not a held world/lifetime/config/publication
    // fence. The native coordinator must bind it to the actual successful world
    // publication and retain complete prepared fences until its no-fail tail.
public:
