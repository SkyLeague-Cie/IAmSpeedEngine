#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/World/Simulation/CanonicalFrameContext.h"
#include "Misc/AutomationTest.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASResimulationFramePurpose,
    "IAmSpeed.Netcode.Prediction.NativeResimulationPurpose",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASResimulationFramePurpose::RunTest(const FString&)
{
    FCanonicalFrameContext Forward(901),Replay(901);
    TestFalse(TEXT("forward purpose is default"),Forward.IsResimulation());
    Replay.Purpose=ECanonicalFramePurpose::Resimulation;
    TestTrue(TEXT("production resimulation purpose is explicit"),Replay.IsResimulation());
    TestEqual(TEXT("frame is unchanged"),Forward.NumFrame,Replay.NumFrame);
    TestEqual(TEXT("fixed physical delta unchanged"),Forward.PhysicalDeltaTime,Replay.PhysicalDeltaTime);
    TestEqual(TEXT("physical time unchanged"),Forward.SimTime,Replay.SimTime);
    TestEqual(TEXT("300Hz unchanged"),FCanonicalFrameContext::PhysicsFramesPerSecond,uint32(300));
#if !UE_BUILD_SHIPPING
    Forward.bResimulationForTesting=true;
    TestTrue(TEXT("historical fixture tag preserved"),Forward.IsResimulation());
    TestEqual(TEXT("fixture tag does not change production purpose"),Forward.Purpose,ECanonicalFramePurpose::Forward);
#endif
    return !HasAnyErrors();
}
#endif
