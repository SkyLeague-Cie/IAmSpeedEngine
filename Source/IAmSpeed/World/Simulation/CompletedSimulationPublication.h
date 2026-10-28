#pragma once
#include "SimulationFrameJournal.h"
#include "IAmSpeed/Input/InputSessionRegistry.h"
#include <memory>

class ASpeedSimulation;
class FIAmSpeedCompletedSimulationPublicationTest;

namespace Speed
{
// One immutable completed-frame read. This contains the EXISTING world snapshot
// and the exact produced-input endpoint used in that frame, not fullstatev3,
// a current source-device sample, a world branch, or a restore capability.
// Readers must retain ONE returned object; separate legacy reads do not join it.
class FCompletedSimulationPublication final
{
public:
    const FSimulationSnapshot& World() const noexcept { return BodySnapshot; }
    const std::shared_ptr<const Input::V2::FRegistryFrame>& ConsumedInputs() const noexcept { return Inputs; }
    const std::shared_ptr<const Input::V2::FInputRegistryView>& RegistryAtFrame() const noexcept { return Registry; }
    // A null ConsumedInputs means this produced-input registry had no bindings.
    // It never certifies absence of legacy journal/game-owned input channels.
private:
    friend class ::ASpeedSimulation;
    friend class ::FIAmSpeedCompletedSimulationPublicationTest;
    using FFrame = Input::V2::FRegistryFrame;
    using FRegistry = Input::V2::FInputRegistryView;
    static bool CanPrepare(const FSimulationSnapshot& Snapshot,
        const std::shared_ptr<const FRegistry>& View, const std::shared_ptr<const FFrame>& Installed) noexcept
    {
        if (Snapshot.PublicationSerial || Snapshot.Payload.IsEmpty() || (View && View->Terminal)) return false;
        const bool Bound = View && !View->Bindings.empty();
        return Bound ? Installed && Installed->Frame == Snapshot.NumFrame
            && Installed->Registry == View && !Installed->Inputs.empty() : !Installed;
    }
    FCompletedSimulationPublication(const FSimulationSnapshot& Snapshot,
        std::shared_ptr<const FRegistry> View, std::shared_ptr<const FFrame> Installed)
        : BodySnapshot(Snapshot), Inputs(std::move(Installed)), Registry(std::move(View)) {}
    bool MatchesCompleted(const std::shared_ptr<const FFrame>& Completed) const noexcept
    {
        // Registry::CompleteAll transfers the SAME preallocated Pending frame to
        // Latest. Equal-looking replacement records never authenticate this pair.
        return Completed == Inputs && CanPrepare(BodySnapshot, Registry, Completed);
    }
    bool BindPublicationSerial(uint64 Serial) noexcept
    {
        if (!Serial || BodySnapshot.PublicationSerial) return false;
        BodySnapshot.PublicationSerial = Serial;
        for (FSimulationPresentationOutput& Output : BodySnapshot.PresentationOutputs)
            Output.PublicationSerial = Serial;
        return true;
    }
    // Allocated/copied BEFORE either irreversible commit. Only ASpeedSimulation
    // stamps the actual world-buffer serial after successful world/input commits,
    // then moves away the only mutable shared_ptr in one atomic publication.
    FSimulationSnapshot BodySnapshot;
    std::shared_ptr<const FFrame> Inputs;
    std::shared_ptr<const FRegistry> Registry;
};
}
