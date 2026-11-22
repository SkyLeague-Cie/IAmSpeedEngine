#include "ExactRemoteRoundReset.h"
#include <algorithm>

namespace Speed::ExactRemoteRound
{
namespace
{
using FRegistry = Input::V2::FInputSessionRegistry;

uint64 HashBaselines(const FRegistry::FGroupResumePublication& Group) noexcept
{
    uint64 Hash = 14695981039346656037ull;
    const auto Add = [&](uint64 V)
    {
        for (unsigned N = 0; N != 8; ++N)
        {
            Hash ^= (V >> (N * 8)) & 255;
            Hash *= 1099511628211ull;
        }
    };
    Add(Group.Next);
    Add(Group.RoundGeneration);
    Add(Group.Baselines.size());
    for (const auto& Baseline : Group.Baselines)
    {
        const auto& Data = Baseline.Input->GetData();
        Add(Baseline.Session);
        Add(Baseline.Epoch);
        Add(Data.Producer.Id);
        Add(static_cast<uint64>(Data.Producer.Kind));
        Add(Data.SourceSequence);
        Add(Data.DeviceGeneration.Value);
        Add(Data.ActiveMask);
        Add(Data.Reset);
        for (auto Value : Data.Values) Add(static_cast<uint16>(Value));
    }
    return Hash;
}
}

std::shared_ptr<const FExactRemoteRoundCommit> Commit(
    FSimulationWorld& World, FRegistry& Registry, FExactRemoteRoundPlan&& Plan,
    std::shared_ptr<const FExactRemoteRoundCommit>& Publication)
{
    const auto& Previous = Plan.Boundary;
    const auto Now = FExactRemoteRoundPlan::FClock::now();
    const auto IsCancelled = [&]() noexcept
    {
        return Plan.Cancellation && Plan.Cancellation->load(std::memory_order_acquire);
    };
    if (!Previous || !Plan.Envelope || !Plan.Lease || !Plan.Supplement
        || !Previous->World().PublicationSerial || !Previous->ConsumedInputs()
        || Previous->ConsumedInputs() != Registry.ReadLatest() || IsCancelled()
        || Plan.Deadline <= Now || Plan.Deadline == FExactRemoteRoundPlan::FClock::time_point::max()
        || Plan.Inputs.RoundBaselines.empty()) return {};

    const auto Address = Plan.Envelope->Address;
    const auto Round = Plan.Inputs.RoundBaselines.front().Input.Generation;
    const auto Prior = std::atomic_load_explicit(&Publication, std::memory_order_acquire);
    if (!Round || (Prior && (Prior->Inputs->RoundGeneration != Round - 1
            || Prior->Previous->Source() != Previous->Source()))
        || Address.Frame != Previous->World().NumFrame
        || Address.PublicationSerial <= Previous->World().PublicationSerial
        || Address.Next != Plan.Inputs.Next || Address.ResumeGeneration != Plan.Inputs.ResumeGeneration
        || !Previous->Source() || Plan.Inputs.WorkerGeneration != Previous->Source()->WorkerGeneration())
        return {};
    for (const auto& Owner : Plan.Inputs.RoundBaselines)
        for (const auto& Actor : Owner.Actors)
            if (std::none_of(Plan.Envelope->Actors.begin(), Plan.Envelope->Actors.end(), [&](const auto& Body)
                { return Body.StableId == Actor.Id && Body.Incarnation == Actor.Generation; })) return {};

    try
    {
        // The registry reservation owns all exact producer reservations. The
        // world owns its native lease and retains it through the common store.
        auto Input = Registry.PrepareGroupResumeAtBoundary(Plan.Inputs);
        if (!Input) return {};
        const auto AbortInput = [&]() noexcept
        {
            if (!Registry.AbortGroupResume(*Input)) std::terminate();
        };
        auto Restore = World.PrepareSnapshotRestore(Plan.Replacement, std::move(Plan.Envelope),
            std::move(Plan.Lease), std::move(Plan.Supplement));
        if (!Restore) { AbortInput(); return {}; }
        bool WorldPrepared = true;
        const auto Abort = [&]() noexcept
        {
            if (WorldPrepared && !World.AbortPreparedRestore(*Restore)) std::terminate();
            AbortInput();
        };
        std::shared_ptr<const FExactRemoteRoundCommit> Draft;
        try
        {
            auto PreparedWorld = World.ReadPreparedRestore(*Restore);
            auto PreparedInputs = Registry.ReadPreparedGroupResume(*Input);
            if (!PreparedWorld || !PreparedInputs) { Abort(); return {}; }
            const auto Hash = HashBaselines(*PreparedInputs);
            if (!Hash) { Abort(); return {}; }
            Draft = std::make_shared<const FExactRemoteRoundCommit>(
                FExactRemoteRoundCommit{Previous, std::move(PreparedWorld), std::move(PreparedInputs),
                    Hash, Plan.Deadline, Plan.Cancellation});
        }
        catch (...) { Abort(); return {}; }
        if (!World.PrepareRestoreFinalization(*Restore))
        {
            // The world cancels its own staged plan on a failed prepare.
            WorldPrepared = false;
            Abort();
            return {};
        }
        if (!Registry.PrepareGroupResumeFinalization(*Input))
        {
            // A failed registry prepare disposes the complete reservation.
            if (!World.AbortPreparedRestore(*Restore)) std::terminate();
            return {};
        }
        if (IsCancelled() || FExactRemoteRoundPlan::FClock::now() >= Plan.Deadline) { Abort(); return {}; }
        World.FinalizePreparedRestore(*Restore);
        Registry.FinalizePreparedGroupResume(*Input);
        Registry.PublishPreparedGroupResume(*Input);
        std::atomic_store_explicit(&Publication, Draft, std::memory_order_release);
        World.ReleasePreparedRestoreAfterPublication(*Restore, Draft->World);
        Registry.ReleasePreparedGroupResume(*Input);
        return Draft;
    }
    catch (...) { return {}; }
}

bool IsExactNeutralStep(const FExactRemoteRoundCommit& Commit, const FCompletedSimulationPublication& Step) noexcept
{
    if (!Commit.Inputs || !Commit.Previous) return false;
    const auto& Group = *Commit.Inputs;
    const auto& Consumed = Step.ConsumedInputs();
    if (Step.Source() != Commit.Previous->Source() || Step.World().NumFrame != Group.Next
        || Commit.Previous->World().PublicationSerial == UINT64_MAX
        || Step.World().PublicationSerial != Commit.Previous->World().PublicationSerial + 1
        || !Step.World().StateHash || !Step.World().InputJournalHash || !Consumed || Consumed->Frame != Group.Next
        || Step.RegistryAtFrame() != Group.Registry || Consumed->Inputs.size() != Group.Baselines.size())
        return false;
    for (std::size_t Index = 0; Index < Group.Baselines.size(); ++Index)
    {
        const auto& Expected = Group.Baselines[Index];
        const auto& Actual = Consumed->Inputs[Index];
        if (Expected.Session != Actual.Session || Expected.Epoch != Actual.Epoch || !Actual.Snapshot
            || Actual.Snapshot->RoundNeutralGeneration != Group.RoundGeneration || !Actual.Snapshot->RoundNeutralControl)
            return false;
        const auto& A = Actual.Snapshot->Input.GetData();
        const auto& E = Expected.Input->GetData();
        if (A.ConsumptionFrame != E.ConsumptionFrame || A.Producer.Id != E.Producer.Id
            || A.Producer.Kind != E.Producer.Kind || A.StreamEpoch.Value != E.StreamEpoch.Value
            || A.DeviceGeneration.Value != E.DeviceGeneration.Value || A.SourceSequence != E.SourceSequence
            || !A.Reset || A.ActiveMask != E.ActiveMask || !A.Transitions.empty() || A.Values != E.Values
            || Actual.Snapshot->Applied != Input::FActionValues{}) return false;
        const auto& Control = Actual.Snapshot->ControlInput().GetData();
        if (!Control.Reset || Control.ActiveMask || !Control.Transitions.empty() || Control.Values != Input::FActionValues{})
            return false;
    }
    return true;
}
}
