#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Components/SafeNetworkPhysicsComponent.h"
#include "IAmSpeed/Components/SafeNetworkPhysicsRPCObservation.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "Misc/AutomationTest.h"
#include "UObject/StrongObjectPtr.h"
#include "UObject/StructOnScope.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpeedInputRPCObservationTest,
    "IAmSpeed.Network.ActualRPCParameterHistoryObservation",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FSpeedInputRPCObservationTest::RunTest(const FString&)
{
    UFunction* Function = USafeNetworkPhysicsComponent::StaticClass()->FindFunctionByName(TEXT("ServerReceiveInputData"));
    if (!TestNotNull(TEXT("Genuine Engine RPC function"), Function)) return false;
    const FStructProperty* Property = FindFProperty<FStructProperty>(Function, TEXT("ClientInputs"));
    if (!TestNotNull(TEXT("Genuine reflected ClientInputs parameter"), Property)) return false;
    if (!TestTrue(TEXT("Exact Engine RPC proxy struct"), Property->Struct == FNetworkPhysicsRewindDataInputProxy::StaticStruct())) return false;
    FStructOnScope Parameters(Function);
    auto* Proxy = Property->ContainerPtrToValuePtr<FNetworkPhysicsRewindDataInputProxy>(Parameters.GetStructMemory());
    TStrongObjectPtr<USafeNetworkPhysicsComponent> Owner(NewObject<USafeNetworkPhysicsComponent>());
    TStrongObjectPtr<USafeNetworkPhysicsComponent> Other(NewObject<USafeNetworkPhysicsComponent>());
    Proxy->Owner = Owner.Get();
    for (const int32 Count : {1, 3, 15, 16})
    {
        auto History = MakeUnique<TNetRewindHistory<FNetworkWheeledSpeedInputState, true>>(Count, true);
        for (int32 Index = 0; Index < Count; ++Index)
        {
            FNetworkWheeledSpeedInputState Input;
            Input.LocalFrame = Input.ServerFrame = Input.ClientFrame = 100 + Index;
            Input.BuildData(nullptr);
            TestTrue(TEXT("Record genuine local history fixture"), History->RecordData(100 + Index, &Input));
        }
        Proxy->History = MoveTemp(History);
        const auto* Before = Proxy->History.Get();
        const auto Observation = ObserveSpeedInputRPC(Owner.Get(), Function, Parameters.GetStructMemory());
        TestTrue(TEXT("Actual reflected parameter and owner match"), Observation.bParameterMatched && Observation.bOwnerMatched);
        TestTrue(TEXT("Actual history present and has data"), Observation.bHistoryPresent && Observation.bHistoryHasData);
        TestEqual(TEXT("Actual argument capacity, including sixteen"), Observation.HistoryCount, Count);
        TestEqual(TEXT("Actual earliest frame"), Observation.EarliestFrame, 100);
        TestEqual(TEXT("Actual latest frame"), Observation.LatestFrame, 99 + Count);
        TestTrue(TEXT("Observation preserves history identity"), Proxy->History.Get() == Before);
        TestEqual(TEXT("Observation preserves count"), Proxy->History->GetHistorySize(), Count);
        TestFalse(TEXT("Mismatched owner is reported, not replaced"), ObserveSpeedInputRPC(Other.Get(), Function, Parameters.GetStructMemory()).bOwnerMatched);
        TestTrue(TEXT("Owner remains unchanged"), Proxy->Owner.Get() == Owner.Get());
        // No owning Actor: the real base implementation returns false. The
        // observer override must preserve this behavior and leave data alone.
        TestFalse(TEXT("Forward actual RPC call to base without owner"), Owner->CallRemoteFunction(Function, Parameters.GetStructMemory(), nullptr, nullptr));
        TestTrue(TEXT("Forwarding preserves history identity"), Proxy->History.Get() == Before);
        TestEqual(TEXT("Forwarding preserves history count"), Proxy->History->GetHistorySize(), Count);
    }
    Proxy->History.Reset();
    const auto Missing = ObserveSpeedInputRPC(Owner.Get(), Function, Parameters.GetStructMemory());
    TestTrue(TEXT("Missing history remains an actual matched parameter"), Missing.bParameterMatched);
    TestFalse(TEXT("Missing history explicitly unavailable"), Missing.bHistoryPresent);
    TestEqual(TEXT("Missing history has no invented count"), Missing.HistoryCount, INDEX_NONE);
    TestFalse(TEXT("Null parameters unavailable"), ObserveSpeedInputRPC(Owner.Get(), Function, nullptr).bParameterMatched);
    UFunction* OtherFunction = USafeNetworkPhysicsComponent::StaticClass()->FindFunctionByName(TEXT("ServerReceiveDeltaSourceInputFrame"));
    TestFalse(TEXT("Different RPC is not reinterpreted"), ObserveSpeedInputRPC(Owner.Get(), OtherFunction, Parameters.GetStructMemory()).bParameterMatched);
    return !HasAnyErrors();
}
#endif
