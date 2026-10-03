#pragma once
#include "InputPublication.h"
#include "SameFrameInputOwner.h"
#include <mutex>

namespace Speed::Input::V2
{
// Synchronized observer history. It owns no producer, exposes no Consume API,
// and can never authorize a physical write from Controller::HandleInputs.
class FInputObservationChannel final : public IInputPublicationSource
{
public:
    FInputObservationChannel(std::shared_ptr<const FInputActionContract> InContract, FStreamEpoch InEpoch)
        : Contract(std::move(InContract)), Epoch(InEpoch) {}
    const std::shared_ptr<const FInputActionContract>& GetContract() const override { return Contract; }
    FStreamEpoch GetEpoch() const override { return Epoch; }
    bool CanConfigure() const override { std::lock_guard<std::mutex> Lock(Gate); return !Started && !Closed; }
    bool IsActive() const override { std::lock_guard<std::mutex> Lock(Gate); return Started && !Closed; }
    bool Activate()
    {
        std::lock_guard<std::mutex> Lock(Gate);
        if (Started || Closed || !Contract || !Epoch.Value) return false;
        Started = true; return true;
    }
    void Deactivate() override { std::lock_guard<std::mutex> Lock(Gate); Closed = true; }
    void Pause()
    {
        std::lock_guard<std::mutex> Lock(Gate);
        Paused = true; Barrier = Serial; PauseSnapshot = Serial ? History[Serial % HistoryCapacity] : nullptr;
    }
    // Worker only, after the complete world/input transaction was published.
    bool Publish(std::shared_ptr<const FOwnerInputSnapshot> Snapshot)
    {
        std::lock_guard<std::mutex> Lock(Gate);
        if (!Started || Closed || !Snapshot || Serial == UINT64_MAX
            || Snapshot->Input.GetData().StreamEpoch.Value != Epoch.Value
            || !Snapshot->Input.IsValidFor(*Contract)) return false;
        const auto ThisThread = std::this_thread::get_id();
        if (Publisher && *Publisher != ThisThread) return false;
        const auto& Previous = History[Serial % HistoryCapacity];
        if ((!Serial && !Snapshot->Input.GetData().Reset)
            || (Serial && (!Previous || Previous->Input.GetData().ConsumptionFrame == UINT64_MAX
                || Snapshot->Input.GetData().ConsumptionFrame != Previous->Input.GetData().ConsumptionFrame + 1))) return false;
        Publisher = ThisThread;
        History[++Serial % HistoryCapacity] = std::move(Snapshot);
        Paused = false;
        return true;
    }
    std::optional<FPublishedFrame> ReadLatest() const override
    {
        std::lock_guard<std::mutex> Lock(Gate);
        if (!Started || Closed || Paused || !Serial) return {};
        return Observe(*History[Serial % HistoryCapacity], Serial);
    }
    FPublishedBatch ReadPublishedSince(FPublicationCursor Cursor) const override
    {
        std::lock_guard<std::mutex> Lock(Gate);
        if (!Started || Closed || Cursor.Epoch.Value != Epoch.Value) return {};
        const FPublicationCursor Next{Epoch, Serial};
        auto Evidence = [&](FPublishedBatch Out) {
            Out.BarrierSerial = Barrier;
            Out.OldestRetainedSerial = Serial >= HistoryCapacity ? Serial - HistoryCapacity + 1 : (Serial ? 1 : 0);
            return Out;
        };
        if (Cursor.Serial > Serial) return Evidence({EReadStatus::InvalidCursor, Next, {}});
        if (Paused) return Evidence({EReadStatus::NoChange, Cursor, {}});
        const bool Boundary = Cursor.Serial < Barrier;
        const auto Start = Boundary ? Barrier : Cursor.Serial;
        // A deliberate lifecycle cutoff is distinct from loss of retained resume frames.
        if (Serial - Start > HistoryCapacity || (Boundary && !PauseSnapshot))
            return Evidence({EReadStatus::Overflow, Next, {}});
        if (Start == Serial) return Evidence({EReadStatus::NoChange, Cursor, {}});
        FPublishedBatch Out{Boundary ? EReadStatus::LifecycleBoundary : EReadStatus::Batch, Next, {}};
        if (Boundary) Out.LifecycleBaseline = Observe(*PauseSnapshot, Barrier);
        for (auto I=Start; I!=Serial;) {
            ++I;
            const auto& Saved = History[I % HistoryCapacity];
            if (!Saved) return Evidence({EReadStatus::Overflow, Next, {}});
            Out.Frames.push_back(Observe(*Saved,I));
        }
        return Evidence(std::move(Out));
    }

private:
    static FPublishedFrame Observe(const FOwnerInputSnapshot& Snapshot, std::uint64_t Serial)
    {
        auto Data = Snapshot.Input.GetData();
        Data.Values = Snapshot.Applied;
        return {FInputFrame(Snapshot.Input.GetContract(), std::move(Data)), Serial};
    }
    const std::shared_ptr<const FInputActionContract> Contract;
    const FStreamEpoch Epoch;
    mutable std::mutex Gate;
    std::array<std::shared_ptr<const FOwnerInputSnapshot>, HistoryCapacity> History{};
    std::shared_ptr<const FOwnerInputSnapshot> PauseSnapshot;
    std::optional<std::thread::id> Publisher;
    std::uint64_t Serial = 0, Barrier = 0;
    bool Started = false, Closed = false, Paused = true;
};
}
