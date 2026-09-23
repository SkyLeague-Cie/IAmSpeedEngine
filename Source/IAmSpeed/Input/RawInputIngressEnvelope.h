#pragma once

#include "RawInput.h"
#include "../World/Simulation/Lifecycle/SessionLifecycle.h"

#include <cstdint>
#include <optional>

namespace Speed::Input::V2
{
// Owning, process-local raw ingress data only. Structural validity does not
// authorize the destination, admit a frame, or keep a session/module alive.
class FRawInputIngressEnvelope final
{
public:
    static std::optional<FRawInputIngressEnvelope> CreateCopied(
        const Lifecycle::FParticipantHandle& Target,
        FStreamEpoch StreamEpoch, FMappingRevision MappingRevision,
        std::uint64_t ConsumptionFrame, const FRawInputSample& Sample)
    {
        if (!Target.Session.GetEpoch() || !Target.Generation ||
            !StreamEpoch.Value || !MappingRevision.Value || !Sample.IsValid() ||
            Sample.Kind == ERawDeviceKind::Desktop) // Requires the versioned composite provenance envelope.
            return {};

        // Validation precedes either vector copy. Allocation failure propagates;
        // this is not an allocation-free canonical finalization operation.
        return FRawInputIngressEnvelope(Target, StreamEpoch, MappingRevision,
            ConsumptionFrame, Sample);
    }

    const Lifecycle::FParticipantHandle& GetTarget() const { return Target; }
    FStreamEpoch GetStreamEpoch() const { return StreamEpoch; }
    FMappingRevision GetMappingRevision() const { return MappingRevision; }
    std::uint64_t GetConsumptionFrame() const { return ConsumptionFrame; }
    const FRawInputSample& GetSample() const { return Sample; }

private:
    FRawInputIngressEnvelope(const Lifecycle::FParticipantHandle& InTarget,
        FStreamEpoch InStreamEpoch, FMappingRevision InMappingRevision,
        std::uint64_t InConsumptionFrame, const FRawInputSample& InSample)
        : Target(InTarget), StreamEpoch(InStreamEpoch),
          MappingRevision(InMappingRevision), ConsumptionFrame(InConsumptionFrame),
          Sample(InSample) {}

    Lifecycle::FParticipantHandle Target;
    FStreamEpoch StreamEpoch;
    FMappingRevision MappingRevision;
    std::uint64_t ConsumptionFrame;
    FRawInputSample Sample;
};
}
