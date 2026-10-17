#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/Testing/InputHistoryAvailabilityChecks.h"
#include "UObject/StrongObjectPtr.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedWheeledInputHistorySerializationTest,
    "IAmSpeed.Network.WheeledInputHistorySerialization",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)
bool FIAmSpeedWheeledInputHistorySerializationTest::RunTest(const FString&)
{
    if(!RunInputHistoryAvailabilityChecks<FNetworkWheeledSpeedInputState>(*this,
        [](auto& Input,int32 Frame){Input.WheeledInput.Throttle=uint8(Frame%251);},
        [](const auto& Input,int32 Frame){return Input.WheeledInput.Throttle==uint8(Frame%251);})) return false;
    TStrongObjectPtr<USpeedWheeledComponent> Owner(NewObject<USpeedWheeledComponent>());
    FNetworkWheeledSpeedInputState Absent;
    Absent.LocalFrame=Absent.ServerFrame=123; Absent.ClientFrame=123;
    Absent.WheeledInput.Throttle=201;
    Absent.BuildData(nullptr);
    TestTrue(TEXT("Owner starts without legacy authority"),Owner->AllowsExternalNetworkStateRestore());
    Absent.ApplyData(Owner.Get());
    TestTrue(TEXT("Absent projection cannot claim legacy input authority"),Owner->AllowsExternalNetworkStateRestore());
    return true;
}
#endif
