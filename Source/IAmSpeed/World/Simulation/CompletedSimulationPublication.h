#pragma once
#include "SimulationFrameJournal.h"
#include "IAmSpeed/Input/InputSessionRegistry.h"
#include <memory>
#include <utility>

class ASpeedPhysicsSimulation;
class FIAmSpeedCompletedSimulationPublicationTest;

namespace Speed
{
// Local source identity, retained by every publication of one owner generation.
// Compare retained shared handles, never this generation number across peers.
class FCompletedSimulationSource final
{
public:
    FCompletedSimulationSource(const FCompletedSimulationSource&) = delete;
    FCompletedSimulationSource& operator=(const FCompletedSimulationSource&) = delete;
    uint64 WorkerGeneration() const noexcept { return Generation; }
private:
    friend class ::ASpeedPhysicsSimulation;
    friend class ::FIAmSpeedCompletedSimulationPublicationTest;
    explicit FCompletedSimulationSource(uint64 InGeneration) noexcept : Generation(InGeneration) {}
    const uint64 Generation;
};

// One immutable completed world/input pair. A retained read is historical;
// it grants no current readiness, network identity or restore capability.
class FCompletedSimulationPublication final
{
public:
    const FSimulationSnapshot& World() const noexcept { return BodySnapshot; }
    const std::shared_ptr<const FCompletedSimulationSource>& Source() const noexcept { return OwnerSource; }
    const std::shared_ptr<const Input::V2::FRegistryFrame>& ConsumedInputs() const noexcept { return Inputs; }
    const std::shared_ptr<const Input::V2::FInputRegistryView>& RegistryAtFrame() const noexcept { return Registry; }
    // Null inputs means no bindings in this produced-input registry.
    // Legacy/game-owned input channels may still exist.
private:
    friend class ::ASpeedPhysicsSimulation;
    friend class ::FIAmSpeedCompletedSimulationPublicationTest;
    using FFrame = Input::V2::FRegistryFrame;
    using FRegistry = Input::V2::FInputRegistryView;
    static bool CanPrepare(const FSimulationSnapshot& Snapshot,
        const std::shared_ptr<const FCompletedSimulationSource>& Source,
        const std::shared_ptr<const FRegistry>& View,
        const std::shared_ptr<const FFrame>& Installed) noexcept
    {
        if (!Source || !Source->WorkerGeneration() || Snapshot.PublicationSerial || Snapshot.Payload.IsEmpty()
            || (View && (View->Terminal || View->WorkerGeneration != Source->WorkerGeneration()))) return false;
        const bool Bound = View && !View->Bindings.empty();
        return Bound ? Installed && Installed->Frame == Snapshot.NumFrame
            && Installed->Registry == View && !Installed->Inputs.empty() : !Installed;
    }
    FCompletedSimulationPublication(const FSimulationSnapshot& Snapshot,
        std::shared_ptr<const FCompletedSimulationSource> Source,
        std::shared_ptr<const FRegistry> View, std::shared_ptr<const FFrame> Installed)
        : BodySnapshot(Snapshot), OwnerSource(std::move(Source)),
          Inputs(std::move(Installed)), Registry(std::move(View)) {}
    bool MatchesCompleted(const std::shared_ptr<const FFrame>& Completed) const noexcept
    {
        // CompleteAll transfers the exact preallocated Pending frame to Latest.
        // Equal-looking replacement records do not authenticate completion.
        return Completed == Inputs && CanPrepare(BodySnapshot, OwnerSource, Registry, Completed);
    }
    bool BindPublicationSerial(uint64 Serial) noexcept
    {
        if (!Serial || BodySnapshot.PublicationSerial) return false;
        BodySnapshot.PublicationSerial = Serial;
        for (FSimulationPresentationOutput& Output : BodySnapshot.PresentationOutputs)
            Output.PublicationSerial = Serial;
        return true;
    }
    // Storage is prepared before either irreversible commit. Only the owner
    // stamps the actual serial after both commits, then moves its mutable alias.
    FSimulationSnapshot BodySnapshot;
    std::shared_ptr<const FCompletedSimulationSource> OwnerSource;
    std::shared_ptr<const FFrame> Inputs;
    std::shared_ptr<const FRegistry> Registry;
};
}
