#if WITH_DEV_AUTOMATION_TESTS
#include "CompletedSimulationPublication.h"
#include "IAmSpeed/Concurrency/AtomicSharedPtr.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedCompletedSimulationPublicationTest,
    "IAmSpeed.Netcode.CompletedPublication.ValueAndEndpointIdentity",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedCompletedSimulationPublicationTest::RunTest(const FString&)
{
    using FPublication = Speed::FCompletedSimulationPublication;
    using namespace Speed::Input::V2;
    FSimulationSnapshot World; World.NumFrame = 41; World.Payload.Add(7);
    FSimulationPresentationOutput Output; Output.OwnerStableId = 1; Output.Channel = 3; Output.NumFrame = 41;
    World.PresentationOutputs.Add(Output);
    auto View = std::make_shared<FInputRegistryView>();
    View->Bindings.emplace_back(); View->Bindings.back().Id = 9;
    auto Installed = std::make_shared<FRegistryFrame>();
    Installed->Frame = World.NumFrame; Installed->Registry = View;
    Installed->Inputs.emplace_back(); Installed->Inputs.back().Session = 9;
    TestTrue(TEXT("Actual prepared endpoint admitted"), FPublication::CanPrepare(World, View, Installed));
    TestFalse(TEXT("Bound input cannot disappear"), FPublication::CanPrepare(World, View, {}));
    auto EqualReplacement = std::make_shared<FRegistryFrame>(*Installed);
    auto Draft = std::shared_ptr<FPublication>(new FPublication(World, View, Installed));
    TestTrue(TEXT("Exact completed endpoint retained"), Draft->MatchesCompleted(Installed));
    TestFalse(TEXT("Equal replacement is not completion of this endpoint"), Draft->MatchesCompleted(EqualReplacement));
    TestFalse(TEXT("Null completion rejected"), Draft->MatchesCompleted({}));
    auto OtherView = std::make_shared<FInputRegistryView>(*View);
    TestFalse(TEXT("Equal-looking registry is not the same captured registry"), FPublication::CanPrepare(World, OtherView, Installed));
    auto WrongFrame = std::make_shared<FRegistryFrame>(*Installed); WrongFrame->Frame++;
    TestFalse(TEXT("Different historical frame rejected"), FPublication::CanPrepare(World, View, WrongFrame));
    auto TerminalView = std::make_shared<FInputRegistryView>(*View); TerminalView->Terminal = true;
    TestFalse(TEXT("Terminal registry rejected"), FPublication::CanPrepare(World, TerminalView, Installed));
    FSimulationSnapshot Empty; Empty.NumFrame = World.NumFrame;
    TestFalse(TEXT("Missing physical bytes rejected"), FPublication::CanPrepare(Empty, View, Installed));
    FSimulationSnapshot AlreadyStamped = World; AlreadyStamped.PublicationSerial = 12;
    TestFalse(TEXT("Old world publication cannot masquerade as a new draft"), FPublication::CanPrepare(AlreadyStamped, View, Installed));
    TestTrue(TEXT("No produced registry bindings is explicit, not universal absence"), FPublication::CanPrepare(World, {}, {}));
    TestFalse(TEXT("Unbound registry with an input record is inconsistent"), FPublication::CanPrepare(World, {}, Installed));
    World.Payload[0] = 99;
    TestEqual(TEXT("World copied before the commits"), Draft->World().Payload[0], uint8(7));
    TestFalse(TEXT("Zero physical publication serial rejected"), Draft->BindPublicationSerial(0));
    TestTrue(TEXT("Actual world publication serial stamps once"), Draft->BindPublicationSerial(18));
    TestFalse(TEXT("No second serial or rollback-counter rebind"), Draft->BindPublicationSerial(19));
    TestEqual(TEXT("Presentation output uses the exact same physical serial"), Draft->World().PresentationOutputs[0].PublicationSerial, uint64(18));
    Speed::Concurrency::TAtomicSharedPtr<const FPublication> Published;
    std::shared_ptr<const FPublication> Sealed = std::move(Draft);
    Published.store(std::move(Sealed), std::memory_order_seq_cst);
    const auto Read = Published.load(std::memory_order_seq_cst);
    TestTrue(TEXT("One read retains the exact consumed endpoint"), Read && Read->ConsumedInputs() == Installed);
    TestEqual(TEXT("Same read retains the physical frame"), Read->World().NumFrame, uint64(41));
    TestEqual(TEXT("Same read retains the actual publication serial"), Read->World().PublicationSerial, uint64(18));
    Published.store({}, std::memory_order_seq_cst);
    TestFalse(TEXT("Invalidation clears future reads"), bool(Published.load(std::memory_order_seq_cst)));
    TestEqual(TEXT("Retained historical read remains immutable after invalidation"), Read->World().Payload[0], uint8(7));
    // Values/endpoint/publication mechanism only. This test does not run a UE
    // physical step, CompleteAll, actor effects, fullstate restore or Client1v1.
    return true;
}
#endif
