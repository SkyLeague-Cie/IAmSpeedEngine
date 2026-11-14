#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/World/Simulation/CompletedSimulationPublication.h"
#include "IAmSpeed/Concurrency/AtomicSharedPtr.h"
#include "Misc/AutomationTest.h"
#include <type_traits>

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedCompletedSimulationPublicationTest,
    "IAmSpeed.Netcode.CompletedPublication.ValueAndEndpointIdentity",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedCompletedSimulationPublicationTest::RunTest(const FString&)
{
    using FPublication = Speed::FCompletedSimulationPublication;
    using FSource = Speed::FCompletedSimulationSource;
    using namespace Speed::Input::V2;
    static_assert(!std::is_copy_constructible_v<FSource>);
    static_assert(!std::is_constructible_v<FSource, uint64>);
    std::shared_ptr<const FSource> Source(new FSource(7));
    std::shared_ptr<const FSource> OtherSource(new FSource(7));
    std::shared_ptr<const FSource> OldSource(new FSource(6));
    FSimulationSnapshot World; World.NumFrame = 41; World.Payload.Add(7);
    FSimulationPresentationBody Body; Body.StableId = 101;
    World.PresentationBodies.Add(Body);
    FSimulationPresentationOutput Output; World.PresentationOutputs.Add(Output);
    auto View = std::make_shared<FInputRegistryView>();
    View->WorkerGeneration = 7; View->Bindings.emplace_back();
    auto Installed = std::make_shared<FRegistryFrame>();
    Installed->Frame = 41; Installed->Registry = View; Installed->Inputs.emplace_back();
    TestTrue(TEXT("Prepared world and exact installed endpoint admitted"), FPublication::CanPrepare(World, Source, View, Installed));
    TestFalse(TEXT("Local source required"), FPublication::CanPrepare(World, {}, View, Installed));
    TestFalse(TEXT("Old owner generation rejected"), FPublication::CanPrepare(World, OldSource, View, Installed));
    TestFalse(TEXT("Bound input cannot disappear"), FPublication::CanPrepare(World, Source, View, {}));
    auto EqualView = std::make_shared<FInputRegistryView>(*View);
    TestFalse(TEXT("Equal-looking registry is not the captured registry"), FPublication::CanPrepare(World, Source, EqualView, Installed));
    auto EqualEndpoint = std::make_shared<FRegistryFrame>(*Installed);
    auto WrongFrame = std::make_shared<FRegistryFrame>(*Installed); ++WrongFrame->Frame;
    TestFalse(TEXT("Different physical frame rejected"), FPublication::CanPrepare(World, Source, View, WrongFrame));
    auto Draft = std::shared_ptr<FPublication>(new FPublication(World, Source, View, Installed));
    TestTrue(TEXT("Exact original endpoint completes the draft"), Draft->MatchesCompleted(Installed));
    TestFalse(TEXT("Equal-looking endpoint does not authenticate completion"), Draft->MatchesCompleted(EqualEndpoint));
    TestFalse(TEXT("Missing completion rejected"), Draft->MatchesCompleted({}));
    TestTrue(TEXT("Retained source handle belongs to original local owner"), Draft->Source() == Source);
    TestFalse(TEXT("Equal generation numbers do not merge two local sources"), Draft->Source() == OtherSource);
    World.Payload[0] = 99; World.PresentationBodies[0].StableId = 202;
    TestEqual(TEXT("Physical bytes copied before commits"), Draft->World().Payload[0], uint8(7));
    TestEqual(TEXT("Local body locator copied before commits"), Draft->World().PresentationBodies[0].StableId, uint64(101));
    TestFalse(TEXT("Zero physical serial rejected"), Draft->BindPublicationSerial(0));
    TestTrue(TEXT("Actual serial stamps the draft once"), Draft->BindPublicationSerial(18));
    TestFalse(TEXT("Physical serial cannot be rebound"), Draft->BindPublicationSerial(19));
    TestEqual(TEXT("Sidecar serial belongs to the same publication"), Draft->World().PresentationOutputs[0].PublicationSerial, uint64(18));
    Speed::Concurrency::TAtomicSharedPtr<const FPublication> Published;
    std::shared_ptr<const FPublication> Sealed = std::move(Draft);
    TestFalse(TEXT("Mutable draft alias consumed"), bool(Draft));
    Published.store(std::move(Sealed));
    const auto Read = Published.load();
    TestTrue(TEXT("One retained read has the original input endpoint"), Read && Read->ConsumedInputs() == Installed);
    TestTrue(TEXT("One retained read has the captured registry"), Read && Read->RegistryAtFrame() == View);
    TestEqual(TEXT("Read keeps the actual world frame"), Read->World().NumFrame, uint64(41));
    TestEqual(TEXT("Read keeps the actual world serial"), Read->World().PublicationSerial, uint64(18));
    Published.store({});
    TestFalse(TEXT("Invalidation clears future reads"), bool(Published.load()));
    TestEqual(TEXT("Previously retained publication stays historical"), Read->World().Payload[0], uint8(7));
    TestTrue(TEXT("No produced bindings remains explicit"), FPublication::CanPrepare(World, Source, {}, {}));
    TestFalse(TEXT("Unbound registry cannot invent a produced endpoint"), FPublication::CanPrepare(World, Source, {}, Installed));
    // Value/endpoint mechanics only: no real registry CompleteAll, worker Step,
    // gameplay admission, source-device polling or restore capability is tested.
    return true;
}
#endif
