#include "FrameFloatingPointContext.h"
#include "Math/VectorRegister.h"

namespace Speed
{
namespace
{
constexpr bool SupportsExactVectorControl()
{
#if PLATFORM_ENABLE_VECTORINTRINSICS && PLATFORM_SUPPORTS_VECTOR_CONTROL_REGISTERS && !PLATFORM_WINDOWS_ARM64EC
    return true;
#else
    // The generic FPU backend's get=0/set=no-op cannot certify FTZ/DAZ transfer.
    return false;
#endif
}
}

FFrameFloatingPointContext CaptureFrameFloatingPointContext() noexcept
{
    FFrameFloatingPointContext Context;
    if (!SupportsExactVectorControl()) return Context;
    if (std::fegetenv(&Context.Environment) != 0) return Context;
    Context.VectorControl = VectorGetControlRegister();
    Context.bValid = true;
    return Context;
}

bool ApplyFrameFloatingPointContext(const FFrameFloatingPointContext& Context) noexcept
{
    if (!Context.bValid || !SupportsExactVectorControl()) return false;
    // CRT/libc can also touch SIMD control; apply the exact vector word last.
    if (std::fesetenv(&Context.Environment) != 0) return false;
    VectorSetControlRegister(Context.VectorControl);
    return VectorGetControlRegister() == Context.VectorControl;
}

FScopedFrameFloatingPointContext::FScopedFrameFloatingPointContext(
    const FFrameFloatingPointContext& Context) noexcept
    : Previous(CaptureFrameFloatingPointContext())
{
    if (!Previous.bValid || !Context.bValid) return;
    bRestoreNeeded = true;
    bApplied = ApplyFrameFloatingPointContext(Context);
    if (!bApplied) Restore();
}

FScopedFrameFloatingPointContext::~FScopedFrameFloatingPointContext() noexcept
{
    const bool bRestored = Restore();
    // Execute restoration even when checks are compiled out. A failure must stay
    // visible as infrastructure evidence; it must never widen equivalence gates.
    ensureAlwaysMsgf(bRestored, TEXT("IAmSpeed task floating-point environment restoration failed"));
}

bool FScopedFrameFloatingPointContext::Restore() noexcept
{
    if (!bRestoreNeeded) return true;
    const bool bRestored = ApplyFrameFloatingPointContext(Previous);
    if (bRestored) bRestoreNeeded = false;
    return bRestored;
}
}
