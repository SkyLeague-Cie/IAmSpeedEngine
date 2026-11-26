    // A sealed LOCAL test timeline, never a relabeled ExactRemote producer.
    // The registry/owner owns this plan and destroys it before its unique source.
    class FPreparedScenarioResume final : public IPreparedInputResume
    {
    public:
        FPreparedScenarioResume(FTestInputProducer& S, FInputFrame B) : Source(S), Frame(std::move(B)) {}
        ~FPreparedScenarioResume() override { Abort(); }
        const FInputFrame& Baseline() const noexcept override { return Frame; }
        bool PrepareFinalization() override
        {
            if (std::this_thread::get_id() != Source.AuthorThread || FPresentationInputScope::IsActive() || Done) return false;
            if (Fence) return true;
            Fence.emplace(Source.Gate);
            if (!Source.ResumeReserved || Source.FrozenFrame || Frame.GetData().ConsumptionFrame < Source.First
                || Frame.GetData().ConsumptionFrame - Source.First != Source.Next)
            { Fence.reset(); return false; }
            return true;
        }
        void Finalize() noexcept override
        {
            if (std::this_thread::get_id() != Source.AuthorThread || !Fence || Done || Finalized) std::terminate();
            Finalized = true; // no consumption, fabrication, lifecycle or frame mutation
        }
        void Release() noexcept override
        {
            if (Done) return;
            if (std::this_thread::get_id() != Source.AuthorThread || !Fence || !Finalized) std::terminate();
            Source.ResumeReserved = false; Done = true; Fence.reset();
        }
        void Abort() noexcept override
        {
            if (Done) return;
            if (std::this_thread::get_id() != Source.AuthorThread) std::terminate();
            if (Finalized) { Release(); return; }
            if (Fence) { Source.ResumeReserved = false; Done = true; Fence.reset(); }
            else { std::lock_guard<std::mutex> Lock(Source.Gate); Source.ResumeReserved = false; Done = true; }
        }
    private:
        FTestInputProducer& Source;
        const FInputFrame Frame;
        std::optional<std::unique_lock<std::mutex>> Fence;
        bool Finalized = false, Done = false;
    };
public:
    std::unique_ptr<IPreparedInputResume> PrepareResumeAtBoundary(FFrameNumber N) override
    {
        if (std::this_thread::get_id() != AuthorThread || FPresentationInputScope::IsActive()) return {};
        std::lock_guard<std::mutex> Lock(Gate);
        if (ResumeReserved || FrozenFrame || N < First || N - First != Next
            || Next >= RetainedFirst + Frames.size() || !Frames[Next - RetainedFirst].GetData().Reset) return {};
        auto Plan = std::make_unique<FPreparedScenarioResume>(*this, Frames[Next - RetainedFirst]);
        ResumeReserved = true; return Plan;
    }
private:
