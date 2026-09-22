#pragma once
#include "InputFrameV2.h"
namespace Speed::Input::V2
{
struct FInputPollCutoff { std::uint64_t Sequence = 0, Generation = 0, LifecycleFence = 0; };
// Freeze retains actual immutable source evidence before Produce, not just a
// counter. Later ordinary arrivals remain for the next poll. Lifecycle changes
// or overflow invalidate the fence. Unadapted production producers are refused.
class IInputProducerPollFence
{
public:
    virtual ~IInputProducerPollFence() = default;
    virtual std::optional<FInputPollCutoff> FreezeForOwner(FFrameNumber N) = 0;
    // Atomic final validation + release under the source gate; false also closes
    // the source lease. A zero cutoff closes an exceptional/failed preparation.
    virtual bool CloseFrozenCutoff(const FInputPollCutoff& Cutoff) noexcept = 0;
};
}
