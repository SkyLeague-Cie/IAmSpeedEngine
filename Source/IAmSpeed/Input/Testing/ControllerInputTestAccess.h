#pragma once

#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Controllers/SpeedController.h"

namespace Speed::Input
{
/** Read-only automation witness. Never exposes a controller to the worker or
 * a mutable stream to test code. All returned frames are immutable copies. */
struct FControllerInputTestAccess final
{
    using FWitness = std::weak_ptr<const FInputStream>;
    static FWitness Observe(const ASpeedController& Controller)
    {
        check(IsInGameThread());
        return Controller.InputSnapshots;
    }
    static std::optional<FPublishedInputFrame> ReadLatest(const FWitness& Witness)
    {
        const auto Stream = Witness.lock();
        return Stream ? Stream->ReadLatest() : std::nullopt;
    }
    static std::optional<FInputFrame> ReadRecorded(const FWitness& Witness, FFrameNumber Frame)
    {
        const auto Stream = Witness.lock();
        return Stream ? Stream->ReadRecorded(Frame) : std::nullopt;
    }
    static bool IsActive(const FWitness& Witness)
    {
        const auto Stream = Witness.lock();
        if (!Stream) return false;
        std::lock_guard<std::mutex> Lock(Stream->Mutex);
        return Stream->Active;
    }
};
}
#endif
