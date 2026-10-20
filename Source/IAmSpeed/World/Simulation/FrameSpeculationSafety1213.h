#pragma once

#include "IAmSpeed/World/Simulation/FrameFloatingPointContext.h"
#include "Math/VectorRegister.h"
#if defined(_MSC_VER) && defined(_M_X64) && !defined(_M_ARM64EC)
#include <immintrin.h>
#endif

namespace Speed
{
namespace Private
{
// Integer-only predicate shared by native bounded-mask fixtures. ScalarControl
// has the platform's encoding; VectorControl is the complete live SSE/AVX MXCSR.
inline bool AreSpeculationMaskWordsSafe1213(const uint32 VectorControl,
    const uint32 ScalarControl, const uint32 ScalarMask,
    const uint32 ScalarAmbiguityMask) noexcept
{
    return ScalarMask != 0 && (VectorControl & uint32(0x1f80)) == uint32(0x1f80)
        && (ScalarControl & ScalarMask) == ScalarMask
        && (ScalarControl & ScalarAmbiguityMask) == 0;
}
}

/** Owner preflight only, BEFORE any speculative admission geometry or shadow
 * arithmetic. The context overload requires a freshly captured CURRENT owner
 * environment, never an arbitrary detached/modified fenv_t value. Both scalar
 * and SIMD masks must be known and complete; unknown platforms stay serial.
 * No masks, rounding, sticky state, FP arithmetic, allocation or logging changes.
 */
inline bool AreCurrentFrameFloatingPointExceptionsMasked1213() noexcept
{
    // Read control words only: even fegetenv/fesetenv are unnecessary here.
    const uint32 LiveVector = VectorGetControlRegister();
    if ((LiveVector & uint32(0x1f80)) != uint32(0x1f80)) return false;
#if defined(_MSC_VER) && defined(_M_X64) && !defined(_M_ARM64EC)
    // MSVC x64 _controlfp_s can describe SIMD rather than the independent x87
    // word. Read the hardware FCW from a read-only, aligned FXSAVE64 snapshot.
    // Bytes0..1 are raw x87 FCW; bytes24..27 are MXCSR. No FXRSTOR is executed.
    alignas(16) uint8 Saved[512]{};
    _fxsave64(Saved);
    const uint32 Control = uint32(Saved[0]) | (uint32(Saved[1]) << 8);
    const uint32 SavedVector = uint32(Saved[24]) | (uint32(Saved[25]) << 8)
        | (uint32(Saved[26]) << 16) | (uint32(Saved[27]) << 24);
    if (SavedVector != LiveVector) return false;
    return Private::AreSpeculationMaskWordsSafe1213(LiveVector,
        Control, uint32(0x3f), uint32(0));
#elif (defined(__GNUC__) || defined(__clang__)) && (defined(__x86_64__) || defined(__i386__))
    // Non-waiting x87 read: never execute a floating-point operation as a probe.
    unsigned short Control = 0;
    __asm__ __volatile__("fnstcw %0" : "=m"(Control));
    return Private::AreSpeculationMaskWordsSafe1213(LiveVector,
        uint32(Control), uint32(0x3f), uint32(0));
#else
    return false;
#endif
}

inline bool AreFrameFloatingPointExceptionsMasked1213(
    const FFrameFloatingPointContext& Numeric) noexcept
{
    return Numeric.bValid && Numeric.VectorControl == VectorGetControlRegister()
        && AreCurrentFrameFloatingPointExceptionsMasked1213();
}
}
