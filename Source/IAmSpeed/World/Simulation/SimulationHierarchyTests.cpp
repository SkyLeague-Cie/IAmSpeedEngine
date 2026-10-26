#if WITH_DEV_AUTOMATION_TESTS
#include "SpeedSimulation.h"
#include "SpeedPhysicsSimulation.h"
#include "ReplaySimulation.h"
#include "RealTimeSimulation.h"
#include "FastSimulation.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpeedSimulationHierarchyTest,
    "IAmSpeed.Replay.SimulationHierarchy", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FSpeedSimulationHierarchyTest::RunTest(const FString&)
{
    TestTrue(TEXT("Common simulation is abstract"), ASpeedSimulation::StaticClass()->HasAnyClassFlags(CLASS_Abstract));
    TestTrue(TEXT("Common base directly owns only the actor contract"), ASpeedSimulation::StaticClass()->GetSuperClass() == AActor::StaticClass());
    TestTrue(TEXT("Physical branch derives from common base"), ASpeedPhysicsSimulation::StaticClass()->GetSuperClass() == ASpeedSimulation::StaticClass());
    TestTrue(TEXT("Recorded replay branch derives from common base"), ASpeedReplaySimulation::StaticClass()->GetSuperClass() == ASpeedSimulation::StaticClass());
    TestTrue(TEXT("RealTime retains its identity under the physical branch"), ARealTimeSimulation::StaticClass()->GetSuperClass() == ASpeedPhysicsSimulation::StaticClass());
    TestTrue(TEXT("Fast retains its identity under the physical branch"), AFastSimulation::StaticClass()->GetSuperClass() == ASpeedPhysicsSimulation::StaticClass());
    TestFalse(TEXT("Replay cannot be used as a physical worker owner"), ASpeedReplaySimulation::StaticClass()->IsChildOf(ASpeedPhysicsSimulation::StaticClass()));
    const auto* Replay = GetDefault<ASpeedReplaySimulation>();
    TestFalse(TEXT("Replay actor does not replicate live authority"), Replay->GetIsReplicated());
    FSimulationSnapshot Snapshot;
    TestFalse(TEXT("Closed replay has no fabricated snapshot"), Replay->ReadLatestSimulationSnapshot(Snapshot));
    return true;
}
#endif
