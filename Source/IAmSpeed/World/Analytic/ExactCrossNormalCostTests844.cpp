#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "ExactCrossNormalCostProbe844.h"
#include "Misc/AutomationTest.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedCrossCostCapture844Test,
    "IAmSpeed.AnalyticWorld.ExactCrossNormalCostCapture844",
    (EAutomationTestFlags::EditorContext | EAutomationTestFlags::ClientContext |
     EAutomationTestFlags::ServerContext | EAutomationTestFlags::CommandletContext) |
    EAutomationTestFlags::EngineFilter)
bool FIAmSpeedCrossCostCapture844Test::RunTest(const FString& Parameters)
{
    using namespace Speed::Analytic;
    using namespace Speed::Analytic::Private;
    FWorldQuery Query; Query.Shape = EQueryShape::Box;
    FBoxSweepContext Context(Query); Context.bLazyExtrudedSatAxes = true;
    FExactCrossNormalCapture844 Capture;
    FExactCrossNormalScope Observe(Query, Context, true, &Capture);
    // Original result must survive repeated keys, eviction, signed zero and axis groups.
    for (int32 I = 0; I < FExactCrossNormalCapture844::Limit; ++I)
    {
        FVector3d Cross((I % 7 == 0) ? -0.0 : 0.0, double(I % 5 + 1), 4.0);
        const double Squared = Cross.SquaredLength();
        const FVector3d Result = Observe.Normalize(Cross, Squared, I % 3);
        if (!FExactCrossNormalScope::SameBits(Result, Cross / FMath::Sqrt(Squared)))
        { AddError(TEXT("844 capture must return original bits")); return false; }
    }
    if (!Capture.IsComplete() || Capture.Rows.Num() != 512 || Capture.Dropped != 0)
    { AddError(TEXT("844 exact capacity boundary")); return false; }
    FExactCrossNormalScope Replay(Query, Context, false);
    for (const auto& R : Capture.Rows)
        if (!FExactCrossNormalScope::SameBits(FExactCrossCostProbe844::Original(R),
            FExactCrossCostProbe844::Reuse(R, Replay)))
        { AddError(TEXT("844 recorded-key replay bits")); return false; }
    // Overflow preserves the full prefix and invalidates timed workload qualification.
    Observe.Normalize(FVector3d(1,2,3), 14.0, 0);
    if (Capture.IsComplete() || Capture.Rows.Num() != 512 || Capture.Dropped != 1)
    { AddError(TEXT("844 overflow must remain explicit")); return false; }
    FExactCrossNormalCapture844 Empty;
    if (!Empty.IsComplete() || Empty.Rows.Num() != 0)
    { AddError(TEXT("844 empty sequence is not positive-work timing proof")); return false; }
    // Exercise report paths only; these synthetic inputs are never runtime provider evidence.
    FExactCrossCostProbe844::Report(Empty, Query, Context, 0);
    FExactCrossCostProbe844::Report(Capture, Query, Context, 0);
    UE_LOG(LogTemp, Display, TEXT("[ExactCrossCostTests844] CaptureRows=512 OverflowRows=1 ReplayBits=PASS SyntheticOnly=1 RuntimeCost=UNKNOWN"));
    return true;
}
#endif
