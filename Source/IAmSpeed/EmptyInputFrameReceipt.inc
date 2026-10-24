public:
    // Explicit INPUT receipt for a genuinely session-free forward step. N must
    // come from the real driver's current frame before its world step. This
    // API does not infer N, fill gaps, step a world or prove its publication.
    // Existing PrepareFrame(empty) remains refused; no Reset or owner is made.
    bool PrepareEmptyFrameReceipt(FFrameNumber N)
    {
        if (!IsWorker() || FPresentationInputScope::IsActive() || BoundaryMutationBlocked()
            || Terminal || Pending || Waiting || Replaying || GroupExecutionReplay || ResumeGroup || ReplayGroup
            || Installed || Begun || !Sessions.empty() || !View || View->Terminal
            || View->WorkerGeneration != WorkerGeneration || !View->Bindings.empty() || !View->Phases.empty()
            || N == std::numeric_limits<FFrameNumber>::max()
            || (LastCompletedForCapture && N <= LastCompletedForCapture->Frame)) return false;

        // Allocation may invoke a reentrant allocator. Gate every registry
        // mutation before allocating; no command mutex is held across a step.
        BoundaryReservationCalling = true;
        struct FCallingScope
        {
            bool& Flag;
            ~FCallingScope() noexcept { Flag = false; }
        } Calling{BoundaryReservationCalling};
        try
        {
            auto Frame = std::make_shared<FRegistryFrame>();
            Frame->Frame = N;
            Frame->Registry = View; // Exact immutable prepare-time view/epoch/version.
            Pending = std::move(Frame);
            PendingEmptyReceipt = true;
            return true;
        }
        catch (...) { return false; } // No source, input history or publication changed.
    }
private:
    bool EmptyReceiptStateValid() const noexcept
    {
        return !PendingEmptyReceipt || (Pending && !Terminal && !Replaying && !GroupExecutionReplay
            && !ResumeGroup && !ReplayGroup && Sessions.empty() && View && !View->Terminal
            && Pending->Registry == View && View->WorkerGeneration == WorkerGeneration
            && View->Bindings.empty() && View->Phases.empty() && Pending->Inputs.empty()
            && Pending->Frame != std::numeric_limits<FFrameNumber>::max()
            && (!LastCompletedForCapture || Pending->Frame > LastCompletedForCapture->Frame));
    }
public:
