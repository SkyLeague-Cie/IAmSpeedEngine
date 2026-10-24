#pragma once
#include "Input/InputFrameV2.h"
#include <memory>

namespace Speed::Input::V2
{
// Optional source-side boundary protocol, distinct from a frame poll/close.
// Old producers keep the rejecting default. Each implementation must qualify
// these guarantees before native admission; noexcept alone is not that proof.
class IPreparedInputResume
{
public:
    virtual ~IPreparedInputResume() = default;
    virtual const FInputFrame& Baseline() const noexcept = 0;
    // All fallible synchronization/validation before any visible mutation.
    // Success retains source lifetime and quiescent fence on the caller thread.
    virtual bool PrepareFinalization() = 0;
    // Infallible tail, no allocation/callback/poll/consume. KEEP the source fence
    // through global publication. Frame N remains unconsumed and immutable.
    virtual void Finalize() noexcept = 0;
    // Only after global publication. Drop held fences; never undo Finalize.
    virtual void Release() noexcept = 0;
    // Before Finalize: withdraw only. After Finalize: release, never roll back.
    // Destruction must perform this cleanup without callbacks or allocation.
    virtual void Abort() noexcept = 0;
};
}
