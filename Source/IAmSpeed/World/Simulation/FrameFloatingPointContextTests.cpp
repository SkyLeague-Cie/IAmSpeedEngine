#if WITH_DEV_AUTOMATION_TESTS

#include "IAmSpeed/World/Simulation/FrameFloatingPointContext.h"
#include "IAmSpeed/World/Simulation/FrameSpeculationSafety1213.h"
#include "FrameParallelExecutor.h"
#include "FrameParallelPolicy.h"
#include "HAL/PlatformTLS.h"
#include "HAL/Event.h"
#include "HAL/PlatformProcess.h"
#include "Math/VectorRegister.h"
#include "Misc/AutomationTest.h"

namespace
{
uint32 Bits(float Value) { uint32 Result; FMemory::Memcpy(&Result, &Value, sizeof(Result)); return Result; }
float ValueFromBits(uint32 Value) { float Result; FMemory::Memcpy(&Result, &Value, sizeof(Result)); return Result; }

// Volatile loads and a non-inlined hardware operation prevent constant folding
// from turning a control-mode test into a comparison of authored literals.
FORCENOINLINE uint32 AddBits(float InA, float InB)
{
    volatile float A = InA, B = InB;
#if PLATFORM_CPU_X86_FAMILY
    float Result;
    _mm_store_ss(&Result, _mm_add_ss(_mm_set_ss(A), _mm_set_ss(B)));
    return Bits(Result);
#else
    volatile float Result = A + B;
    return Bits(Result);
#endif
}
FORCENOINLINE uint32 MultiplyBits(float InA, float InB)
{
    volatile float A = InA, B = InB;
#if PLATFORM_CPU_X86_FAMILY
    float Result;
    _mm_store_ss(&Result, _mm_mul_ss(_mm_set_ss(A), _mm_set_ss(B)));
    return Bits(Result);
#else
    volatile float Result = A * B;
    return Bits(Result);
#endif
}

Speed::FFrameFloatingPointContext WithFlush(const Speed::FFrameFloatingPointContext& Base, bool bFlush)
{
    auto Context = Base;
    if (bFlush) Context.VectorControl |= VECTOR_DENORMALS_FLUSH_TO_ZERO;
    else Context.VectorControl &= ~uint32(VECTOR_DENORMALS_FLUSH_TO_ZERO);
#if PLATFORM_CPU_X86_FAMILY
    // Engine's FTZ mask is bit15 only on SSE. DAZ is an independent bit6.
    if (bFlush) Context.VectorControl |= 0x0040u;
    else Context.VectorControl &= ~0x0040u;
#endif
    return Context;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedFrameFloatContextRestoreTest,
    "IAmSpeed.Simulation.FrameParallel.FloatContextRestore",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)
bool FIAmSpeedFrameFloatContextRestoreTest::RunTest(const FString&)
{
    using namespace Speed;
    const auto Original = CaptureFrameFloatingPointContext();
    if (!TestTrue(TEXT("platform can capture exact scalar and vector environment"), Original.bValid)) return false;
    FScopedFrameFloatingPointContext RestoreOriginal(Original);
    std::fenv_t Held;
    if (!TestEqual(TEXT("nonstop fixture environment"), std::feholdexcept(&Held), 0)) return false;
    TestEqual(TEXT("nearest fixture mode"), std::fesetround(FE_TONEAREST), 0);
    const auto NearestNoFlush = WithFlush(CaptureFrameFloatingPointContext(), false);
    const auto NearestFlush = WithFlush(NearestNoFlush, true);
    TestTrue(TEXT("nonflushing fixture applies"), ApplyFrameFloatingPointContext(NearestNoFlush));
    const uint32 Before = VectorGetControlRegister();
    uint32 TinyPreserved = 0, TinyFlushed = 0, OperandPreserved = 0, OperandFlushed = 0;
    {
        FScopedFrameFloatingPointContext Scope(NearestNoFlush);
        TestTrue(TEXT("owner nonflush context applies"), Scope.IsApplied());
        TinyPreserved = MultiplyBits(ValueFromBits(0x00800000u), .5f);
        OperandPreserved = MultiplyBits(ValueFromBits(0x00000001u), 1.f);
    }
    {
        FScopedFrameFloatingPointContext Scope(NearestFlush);
        TestTrue(TEXT("owner flush context applies"), Scope.IsApplied());
        TinyFlushed = MultiplyBits(ValueFromBits(0x00800000u), .5f);
        OperandFlushed = MultiplyBits(ValueFromBits(0x00000001u), 1.f);
    }
    const uint32 After = VectorGetControlRegister();
    TestEqual(TEXT("gradual tiny result is retained"), TinyPreserved, 0x00400000u);
    TestEqual(TEXT("FTZ changes tiny result on real arithmetic"), TinyFlushed, 0u);
    TestEqual(TEXT("gradual tiny operand is retained"), OperandPreserved, 1u);
    TestEqual(TEXT("DAZ/FZ changes tiny operand on real arithmetic"), OperandFlushed, 0u);
    TestEqual(TEXT("full prior vector word restored"), After, Before);

    TestEqual(TEXT("upward fixture mode"), std::fesetround(FE_UPWARD), 0);
    const auto Upward = CaptureFrameFloatingPointContext();
    ApplyFrameFloatingPointContext(NearestNoFlush);
    const uint32 NearestTie = AddBits(1.f, ValueFromBits(0x33800000u)); // 2^-24, exactly half an ULP at1.
    uint32 UpwardTie = 0;
    {
        FScopedFrameFloatingPointContext Scope(Upward);
        TestTrue(TEXT("alternate rounding mode applies"), Scope.IsApplied());
        UpwardTie = AddBits(1.f, ValueFromBits(0x33800000u));
    }
    TestEqual(TEXT("nearest tie stays even"), NearestTie, 0x3f800000u);
    TestEqual(TEXT("upward tie uses next float"), UpwardTie, 0x3f800001u);
    TestEqual(TEXT("scalar rounding mode restored"), std::fegetround(), FE_TONEAREST);

    FFrameFloatingPointContext Invalid;
    const uint32 BeforeInvalid = VectorGetControlRegister();
    { FScopedFrameFloatingPointContext Scope(Invalid); TestFalse(TEXT("invalid capture cannot execute qualified work"), Scope.IsApplied()); }
    TestEqual(TEXT("invalid scope changes no vector control"), VectorGetControlRegister(), BeforeInvalid);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedFrameFloatContextWorkerTest,
    "IAmSpeed.Simulation.FrameParallel.FloatContextOwnerWorker",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)
bool FIAmSpeedFrameFloatContextWorkerTest::RunTest(const FString&)
{
    using namespace Speed;
    const uint32 OwnerThread = FPlatformTLS::GetCurrentThreadId();
    const auto Original = CaptureFrameFloatingPointContext();
    if (!TestTrue(TEXT("owner capture supported"), Original.bValid)) return false;
    FScopedFrameFloatingPointContext RestoreOriginal(Original);
    std::fenv_t Held;
    TestEqual(TEXT("nonstop owner fixture"), std::feholdexcept(&Held), 0);
    TestEqual(TEXT("nearest owner fixture"), std::fesetround(FE_TONEAREST), 0);
    const auto OwnerContext = WithFlush(CaptureFrameFloatingPointContext(), false);
    ApplyFrameFloatingPointContext(OwnerContext);
    const uint32 ExpectedTiny = MultiplyBits(ValueFromBits(0x00800000u), .5f);
    uint32 WorkerThread = 0, Before = 0, Inside = 0, After = 0, ActualTiny = 0;
    int32 BeforeRound = 0, InsideRound = 0, AfterRound = 0;
    bool bApplied = false, bRestored = false;
    const auto Work = [&]
    {
        WorkerThread = FPlatformTLS::GetCurrentThreadId();
        const auto WorkerOriginal = CaptureFrameFloatingPointContext();
        FScopedFrameFloatingPointContext RestoreWorkerOriginal(WorkerOriginal);
        std::fenv_t WorkerHeld;
        std::feholdexcept(&WorkerHeld);
        std::fesetround(FE_DOWNWARD);
        const auto Poisoned = WithFlush(CaptureFrameFloatingPointContext(), true);
        ApplyFrameFloatingPointContext(Poisoned);
        Before = VectorGetControlRegister(); BeforeRound = std::fegetround();
        {
            FScopedFrameFloatingPointContext Scope(OwnerContext);
            bApplied = Scope.IsApplied(); Inside = VectorGetControlRegister(); InsideRound = std::fegetround();
            ActualTiny = MultiplyBits(ValueFromBits(0x00800000u), .5f);
            bRestored = Scope.Restore();
        }
        After = VectorGetControlRegister(); AfterRound = std::fegetround();
    };
    const bool bMultiThread = Speed::EnsurePrivateFrameWorkers(Speed::FFrameParallelPolicy::MaximumLanes - 1) > 0;
    if (!TestTrue(TEXT("owned worker available for numeric environment proof"), bMultiThread)) return false;
    const auto Backend = Speed::DispatchPrivateFramePair(Work, [] {});
    TestTrue(TEXT("numeric work executed on the IAmSpeed executor"), Backend == Speed::EFrameExecutorBackend::PrivatePool);
    TestTrue(TEXT("worker accepts explicit owner environment"), bApplied && bRestored);
    TestEqual(TEXT("owner vector control word observed inside worker"), Inside, OwnerContext.VectorControl);
    TestEqual(TEXT("owner scalar rounding observed inside worker"), InsideRound, FE_TONEAREST);
    TestEqual(TEXT("tiny actual arithmetic agrees with owner"), ActualTiny, ExpectedTiny);
    TestEqual(TEXT("prior worker vector word restored"), After, Before);
    TestEqual(TEXT("prior worker scalar rounding restored"), AfterRound, BeforeRound);
    if (bMultiThread) TestTrue(TEXT("real IAmSpeed worker witnessed"), WorkerThread != OwnerThread);
    AddInfo(FString::Printf(TEXT("[FrameFloatContext] MultiThread=%u OwnerThread=%u WorkerThread=%u OwnerVector=%08x WorkerBefore=%08x WorkerInside=%08x WorkerAfter=%08x Tiny=%08x"),
        uint32(bMultiThread), OwnerThread, WorkerThread, OwnerContext.VectorControl, Before, Inside, After, ActualTiny));
    return true;
}


IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedFrameSpeculationMasksTest1213,
    "IAmSpeed.Simulation.FrameParallel.SpeculationExceptionMasks1213",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)
bool FIAmSpeedFrameSpeculationMasksTest1213::RunTest(const FString&)
{
    using Speed::Private::AreSpeculationMaskWordsSafe1213;
    // Pure integer fixtures exercise every independent SIMD/x87 exception bit.
    // They never install an unmasked environment or execute trapping arithmetic.
    constexpr uint32 VectorMask = 0x1f80u;
    constexpr uint32 X87Mask = 0x003fu;
    TestTrue(TEXT("all six independent masks admitted"),
        AreSpeculationMaskWordsSafe1213(VectorMask, X87Mask, X87Mask, 0));
    for (uint32 Bit = 0; Bit < 6; ++Bit)
    {
        TestFalse(TEXT("each SIMD mask is required independently"),
            AreSpeculationMaskWordsSafe1213(VectorMask & ~(uint32(1) << (Bit + 7)), X87Mask, X87Mask, 0));
        TestFalse(TEXT("each x87 mask is required independently of SIMD"),
            AreSpeculationMaskWordsSafe1213(VectorMask, X87Mask & ~(uint32(1) << Bit), X87Mask, 0));
    }
    TestFalse(TEXT("unknown scalar mask policy refuses"),
        AreSpeculationMaskWordsSafe1213(VectorMask, X87Mask, 0, 0));
    TestFalse(TEXT("ambiguous scalar/SIMD control refuses"),
        AreSpeculationMaskWordsSafe1213(VectorMask, X87Mask | 0x80000000u, X87Mask, 0x80000000u));
    // Rounding, DAZ, FTZ and existing sticky flags are not a new numeric policy.
    TestTrue(TEXT("unrelated control and sticky bits are preserved, not normalized"),
        AreSpeculationMaskWordsSafe1213(VectorMask | 0xe07fu, X87Mask | 0x0c00u, X87Mask, 0));
    Speed::FFrameFloatingPointContext Invalid;
    TestFalse(TEXT("invalid owner capture refuses before native inspection"),
        Speed::AreFrameFloatingPointExceptionsMasked1213(Invalid));
    const auto Before = Speed::CaptureFrameFloatingPointContext();
    const bool Allowed = Speed::AreFrameFloatingPointExceptionsMasked1213(Before);
    const auto After = Speed::CaptureFrameFloatingPointContext();
    // No environment is changed by this fixture; all assertions run under the
    // entry mask. The qualification lot10 fixture tests a live SIMD rejection.
    TestEqual(TEXT("mask inspection preserves complete vector control"), After.VectorControl, Before.VectorControl);
    TestEqual(TEXT("mask inspection preserves scalar environment"),
        FMemory::Memcmp(&Before.Environment, &After.Environment, sizeof(std::fenv_t)), 0);
    if (!Before.bValid) TestFalse(TEXT("unsupported owner remains serial"), Allowed);
    return true;
}


#endif
