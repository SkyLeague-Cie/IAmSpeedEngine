#pragma once

#include "CoreMinimal.h"
#include <cfenv>
#include <type_traits>

namespace Speed
{
/** Same-process/thread-transfer values, never a serialized cross-platform format.
 * fenv carries CRT/libc scalar state (including x87 where applicable). The
 * separate engine vector register also retains FTZ/DAZ and vector rounding.
 * Capture on the canonical owner at the dispatch barrier, never in the worker.
 */
struct FFrameFloatingPointContext
{
    std::fenv_t Environment{};
    uint32 VectorControl = 0;
    bool bValid = false;
};
static_assert(std::is_trivially_copyable_v<FFrameFloatingPointContext>);

IAMSPEED_API FFrameFloatingPointContext CaptureFrameFloatingPointContext() noexcept;
IAMSPEED_API bool ApplyFrameFloatingPointContext(const FFrameFloatingPointContext& Context) noexcept;

/** A worker must check IsApplied BEFORE touching its private simulation result.
 * Restore failure is a native infrastructure failure, not a numeric tolerance.
 * No fixed rounding/denormal policy is invented: the existing owner is copied.
 */
class IAMSPEED_API FScopedFrameFloatingPointContext
{
public:
    explicit FScopedFrameFloatingPointContext(const FFrameFloatingPointContext& Context) noexcept;
    ~FScopedFrameFloatingPointContext() noexcept;
    FScopedFrameFloatingPointContext(const FScopedFrameFloatingPointContext&) = delete;
    FScopedFrameFloatingPointContext& operator=(const FScopedFrameFloatingPointContext&) = delete;
    bool IsApplied() const noexcept { return bApplied; }
    /** Explicit variant used by native tests; destructor still guarantees retry. */
    bool Restore() noexcept;
private:
    FFrameFloatingPointContext Previous;
    bool bApplied = false;
    bool bRestoreNeeded = false;
};
}
