#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "IAmSpeed/World/Simulation/FrameParallelSettings.h"
#include "Math/VectorRegister.h"
#include <cfenv>

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedFrameParallelScopeContextTest,
    "IAmSpeed.Simulation.FrameParallel.ScopeContextPropagation",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FIAmSpeedFrameParallelScopeContextTest::RunTest(const FString&)
{
    const auto Original = Speed::CaptureFrameFloatingPointContext();
    if (!Original.bValid) { AddError(TEXT("Numeric context capture unavailable; native qualification UNKNOWN")); return false; }
    TestFalse(TEXT("test starts outside task"), Speed::IsInsideFrameParallelTask());
    Speed::FFrameFloatingPointContext Owner;
    {
        Speed::FScopedFrameFloatingPointContext RestoreOriginal(Original);
        if (!RestoreOriginal.IsApplied()) { AddError(TEXT("owner numeric guard unavailable")); return false; }
        if (std::fesetround(FE_DOWNWARD) != 0) { AddError(TEXT("rounding control unavailable")); return false; }
        Owner = Speed::CaptureFrameFloatingPointContext();
        if (!RestoreOriginal.Restore()) { AddError(TEXT("owner numeric restoration failed")); return false; }
    }
    const auto BeforeTask = Speed::CaptureFrameFloatingPointContext();
    {
        Speed::FScopedFrameParallelTask Task(Owner);
        TestTrue(TEXT("task context applied before work"), Task.IsNumericContextApplied());
        TestTrue(TEXT("nested dispatch blocked"), Speed::IsInsideFrameParallelTask());
        TestEqual(TEXT("owner rounding copied"), std::fegetround(), FE_DOWNWARD);
        {
            Speed::FScopedFrameParallelTask Nested(Owner);
            TestTrue(TEXT("nested numeric context applied"), Nested.IsNumericContextApplied());
            TestTrue(TEXT("nested explicit restore"), Nested.RestoreNumericContext());
        }
        TestTrue(TEXT("nested task preserves outer admission guard"), Speed::IsInsideFrameParallelTask());
        TestTrue(TEXT("task explicit restoration succeeds"), Task.RestoreNumericContext());
        TestFalse(TEXT("restored scope is no longer applied"), Task.IsNumericContextApplied());
    }
    TestFalse(TEXT("task nesting restored"), Speed::IsInsideFrameParallelTask());
    TestEqual(TEXT("vector control restored"), Speed::CaptureFrameFloatingPointContext().VectorControl,
        BeforeTask.VectorControl);
    {
        Speed::FScopedFrameParallelTask Invalid(Speed::FFrameFloatingPointContext{});
        TestFalse(TEXT("invalid context cannot qualify work"), Invalid.IsNumericContextApplied());
        TestTrue(TEXT("invalid context restore is safe"), Invalid.RestoreNumericContext());
    }
    TestTrue(TEXT("original numeric environment restored"), Speed::ApplyFrameFloatingPointContext(Original));
    return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedFrameParallelOwnerObservationTest,
    "IAmSpeed.Simulation.FrameParallel.OwnerStageObservations",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FIAmSpeedFrameParallelOwnerObservationTest::RunTest(const FString&)
{
    using namespace Speed;
    for (EFrameParallelStage Stage : {EFrameParallelStage::Camera, EFrameParallelStage::Prepare, EFrameParallelStage::QueryBatch})
    {
        ResetFrameParallelStageTotals(Stage);
        RecordFrameParallelStage(Stage, 32, 4, 2, EFrameParallelFallback::None);
        RecordFrameParallelStage(Stage, 3, 1, 1, EFrameParallelFallback::NumericEnvironmentUnavailable);
        const auto Totals = GetFrameParallelStageTotals(Stage);
        TestEqual(TEXT("all routes counted"), Totals.Invocations, uint64(2));
        TestEqual(TEXT("parallel dispatch count distinct from threads"), Totals.ParallelInvocations, uint64(1));
        TestEqual(TEXT("items include serial fallback"), Totals.Items, uint64(35));
        TestEqual(TEXT("actual native participation retained"), Totals.MaximumNativeThreads, 2);
        TestEqual(TEXT("logical dispatch retained"), Totals.MaximumLogicalLanes, 4);
        TestEqual(TEXT("numeric unavailable remains explicit"), Totals.NumericFallbacks, uint64(1));
        TestEqual(TEXT("fallback reason bit retained"), Totals.FallbackMask,
            uint64(1) << static_cast<uint8>(EFrameParallelFallback::NumericEnvironmentUnavailable));
        ResetFrameParallelStageTotals(Stage);
        TestEqual(TEXT("barrier reset clears observations"), GetFrameParallelStageTotals(Stage).Invocations, uint64(0));
    }
    return !HasAnyErrors();
}
#endif
