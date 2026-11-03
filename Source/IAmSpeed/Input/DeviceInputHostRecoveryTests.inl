// Included only by DeviceInputHost.cpp so the fixture can replace its private
// bridge without exposing a production setter or importing platform hardware.
#if WITH_DEV_AUTOMATION_TESTS && PLATFORM_WINDOWS && !UE_SERVER
#include "Misc/AutomationTest.h"

namespace Speed::Input::V2::HostStartupRecovery01621
{
struct FSourceState
{
    FSourceState(std::uint64_t InProducer, std::shared_ptr<FRawAcquisitionJournal> InJournal, bool InUnavailable)
        : Producer(InProducer), Journal(std::move(InJournal)), Unavailable(InUnavailable), InitialTicket(Journal->BeginAcquisition()) {}
    const std::uint64_t Producer;
    const std::shared_ptr<FRawAcquisitionJournal> Journal;
    const bool Unavailable;
    const FAcquisitionTicket InitialTicket;
    std::atomic<unsigned> PumpCalls{0}, CloseCalls{0};
    std::atomic<bool> PumpOffGameThread{false}, CloseOnPumpThread{false};
};

FRawAcquisitionBatch Reading(bool NeutralState, std::uint64_t Sequence = 1)
{
    FRawAcquisitionBatch Batch;
    Batch.Count = 1;
    auto& Sample = Batch.Readings[0];
    Sample.DeviceId = NeutralState ? FDeviceDiscovery::Capacity + 1 : 1;
    Sample.Generation = {1}; Sample.Sequence = Sequence; Sample.TimestampMicroseconds = Sequence;
    Sample.Kind = ERawDeviceKind::Keyboard; Sample.FreshBaseline = true;
    Sample.State.Count = 3;
    Sample.State.Values[0] = {{ERawControlKind::KeyboardUsage, 4}, NeutralState ? 0.f : 1.f};
    Sample.State.Values[1] = {{ERawControlKind::KeyboardUsage, 5}, 0.f};
    Sample.State.Values[2] = {{ERawControlKind::KeyboardUsage, 6}, 0.f};
    return Batch;
}

class FFakeSource final : public IInputAcquisition
{
public:
    explicit FFakeSource(std::shared_ptr<FSourceState> InState) : State(std::move(InState)) {}
    EAcquisitionPumpResult Pump() override
    {
        if (Closed) return EAcquisitionPumpResult::Closed;
        ++State->PumpCalls;
        State->PumpOffGameThread.store(!IsInGameThread());
        PumpThread = std::this_thread::get_id();
        if (Published) return EAcquisitionPumpResult::NoChange;
        auto Admission = State->Journal->ReserveAcquisition();
        const auto Ticket = State->Journal->BeginAcquisition();
        if (State->Unavailable) State->Journal->Invalidate();
        if (!State->Journal->Publish(Ticket, Reading(State->Unavailable))) return EAcquisitionPumpResult::Rejected;
        Published = true;
        // No platform API is called. This source exercises the real host and
        // acquisition worker's explicit unavailable/drain/replace contract.
        return State->Unavailable ? EAcquisitionPumpResult::RetryableUnavailable : EAcquisitionPumpResult::Installed;
    }
    bool Close() override
    {
        if (Closed) return true;
        State->Journal->Close();
        State->CloseOnPumpThread.store(PumpThread == std::this_thread::get_id());
        ++State->CloseCalls; Closed = true;
        return true;
    }
private:
    const std::shared_ptr<FSourceState> State;
    std::thread::id PumpThread;
    bool Published = false, Closed = false;
};

struct FBridgeFixture
{
    unsigned Calls = 0;
    std::vector<std::shared_ptr<FSourceState>> Sources;
};
FBridgeFixture* ActiveFixture = nullptr;

std::shared_ptr<IInputAcquisition> CreateFakeSource(std::uint64_t Producer,
    std::shared_ptr<FRawAcquisitionJournal> Journal, const FDeviceRawAcquisitionConfig& Config,
    FDeviceRawAcquisitionFactory)
{
    check(IsInGameThread());
    if (!ActiveFixture || !Producer || !Journal || Journal->BeginAcquisition().Session != Producer
        || !FDeviceActivityPolicy::ValidConfig(Config.Activity)) return {};
    auto SourceState = std::make_shared<FSourceState>(Producer, std::move(Journal), ++ActiveFixture->Calls == 1);
    ActiveFixture->Sources.push_back(SourceState);
    return std::make_shared<FFakeSource>(std::move(SourceState));
}

class FScopedBridge final
{
public:
    explicit FScopedBridge(FBridgeFixture& Fixture)
        : PreviousFixture(ActiveFixture), PreviousBridge(ProductRawBridge.exchange(&CreateFakeSource))
    { check(IsInGameThread()); ActiveFixture = &Fixture; }
    ~FScopedBridge()
    {
        // Restore the exact product bridge, including an already registered
        // Steam bridge, on normal completion and every early test return.
        ProductRawBridge.store(PreviousBridge); ActiveFixture = PreviousFixture;
    }
    FScopedBridge(const FScopedBridge&) = delete;
    FScopedBridge& operator=(const FScopedBridge&) = delete;
private:
    FBridgeFixture* const PreviousFixture;
    const FDeviceRawAcquisitionBridge PreviousBridge;
};

FDeviceInputHostConfig Config()
{
    FInputActionContractDescription Description;
    Description.Revision = {1}; Description.Actions = FInputActionContract::BaseActions();
    Description.Physical = {{0, EPhysicalDestination::Throttle}, {1, EPhysicalDestination::Brake}, {2, EPhysicalDestination::Steering}};
    Description.Mapping = {{{ERawControlKind::KeyboardUsage, 4}, 0, 1},
        {{ERawControlKind::KeyboardUsage, 5}, 1, 1}, {{ERawControlKind::KeyboardUsage, 6}, 2, 1}};
    FDeviceInputHostConfig HostConfig;
    HostConfig.Contract = FInputActionContract::Create(std::move(Description));
    HostConfig.FirstFrame = 37;
    HostConfig.Activity.Stick = {0.2f, 0.1f, 0.05f};
    HostConfig.Activity.Trigger = {0.2f, 0.1f, 0.05f};
    HostConfig.Activity.MinimumResidenceFrames = 30; HostConfig.Activity.MaximumDevices = 8;
    HostConfig.Activity.HybridDeviceKind = EDeviceKind::Gamepad;
    HostConfig.Activity.StartupPreferredKind = EDeviceKind::Gamepad;
    HostConfig.Activity.RetainSameKindOwner = true; HostConfig.Activity.GamepadPriority = true;
    HostConfig.Cadence = std::chrono::microseconds(500); HostConfig.StartupTimeout = std::chrono::milliseconds(2000);
    return HostConfig;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDeviceInputHostStartupRecovery01621Test,
    "IAmSpeed.Input.Host.StartupRecovery01621",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FDeviceInputHostStartupRecovery01621Test::RunTest(const FString&)
{
    using namespace Speed::Input::V2;
    using namespace Speed::Input::V2::HostStartupRecovery01621;
    const auto OriginalBridge = ProductRawBridge.load();
    const auto OriginalFixture = ActiveFixture;
    FBridgeFixture Fixture;
    {
        FScopedBridge Bridge(Fixture);
        auto HostConfig = Config();
        if (!TestTrue(TEXT("minimal keyboard driving contract is valid"), bool(HostConfig.Contract))) return false;
        auto InvalidConfig = HostConfig; InvalidConfig.Contract.reset();
        AddExpectedError(TEXT("Independent input host rejected: contract missing"), EAutomationExpectedErrorFlags::Contains, 1);
        TestFalse(TEXT("missing contract remains rejected before source construction"), bool(CreateDeviceInputHost(InvalidConfig)));
        auto MouseDescription = HostConfig.Contract->GetDescription();
        MouseDescription.Mapping[0].Control = {ERawControlKind::MouseButton, static_cast<std::uint16_t>(EMouseButton::Left)};
        InvalidConfig.Contract = FInputActionContract::Create(std::move(MouseDescription));
        if (!TestTrue(TEXT("unsupported mouse fixture still has a structurally valid contract"), bool(InvalidConfig.Contract))) return false;
        AddExpectedError(TEXT("unsupported_kind=MouseButton"), EAutomationExpectedErrorFlags::Contains, 1);
        TestFalse(TEXT("unsupported contract is not converted into a retryable source"), bool(CreateDeviceInputHost(InvalidConfig)));
        TestEqual(TEXT("invalid contracts never reach the replacement bridge"), Fixture.Calls, 0u);

        auto FirstHost = CreateDeviceInputHost(HostConfig);
        if (!TestTrue(TEXT("first real factory constructs a descriptor and acquisition owner"),
            FirstHost && FirstHost->Descriptor && FirstHost->Journal && FirstHost->Acquisition)) return false;
        const auto OldJournal = FirstHost->Journal;
        TestFalse(TEXT("explicit unavailable result cannot satisfy startup readiness"),
            FirstHost->Acquisition->WaitForFirstPublication(HostConfig.StartupTimeout));
        TestTrue(TEXT("the real worker exposes an explicit retryable startup failure"),
            FirstHost->Acquisition->StartupState(HostConfig.StartupTimeout) == EAcquisitionStartup::Failed
            && FirstHost->Acquisition->IsRetryableUnavailable());
        TestFalse(TEXT("unavailable is distinct from a startup deadline"), FirstHost->Acquisition->IsStartupTimedOut(HostConfig.StartupTimeout));
        TestEqual(TEXT("first factory reached the fixture once"), Fixture.Calls, 1u);
        if (!TestTrue(TEXT("first source provenance is retained"), Fixture.Sources.size() == 1)) return false;
        const auto FirstSource = Fixture.Sources[0];
        const auto OldTicket = FirstSource->InitialTicket;
        const auto NeutralBaseline = OldJournal->ReadControlBaseline();
        if (!TestTrue(TEXT("failed attempt leaves a neutral baseline readable until host drainage"), bool(NeutralBaseline))) return false;
        for (std::size_t Index = 0; Index < NeutralBaseline->State.State.Count; ++Index)
            TestEqual(TEXT("failed source exposes no held keyboard value"), NeutralBaseline->State.State.Values[Index].Value, 0.f);
        TestFalse(TEXT("journal does not close ahead of host drainage"),
            OldJournal->ReadControlsSince({FirstHost->Session, 0}).Status == EAcquisitionRead::Closed);
        TestEqual(TEXT("unavailable stops further source pumps"), FirstSource->PumpCalls.load(), 1u);
        TestEqual(TEXT("source is not destroyed before explicit host drainage"), FirstSource->CloseCalls.load(), 0u);
        TestTrue(TEXT("factory acquisition happens off the game thread"), FirstSource->PumpOffGameThread.load());
        TestTrue(TEXT("unbound failed host closes at its real boundary"), FirstHost->CloseAtBoundary());
        TestTrue(TEXT("failed acquisition worker has drained"), FirstHost->Acquisition->HasFinished());
        TestEqual(TEXT("failed source closes exactly once"), FirstSource->CloseCalls.load(), 1u);
        TestTrue(TEXT("failed source closes on its acquisition owner"), FirstSource->CloseOnPumpThread.load());
        TestTrue(TEXT("the old journal is closed after drainage"),
            OldJournal->ReadControlsSince({FirstHost->Session, 0}).Status == EAcquisitionRead::Closed);
        TestFalse(TEXT("closed old journal refuses the formerly valid acquisition ticket"), OldJournal->Publish(OldTicket, Reading(false, 2)));

        auto SecondHost = CreateDeviceInputHost(HostConfig);
        if (!TestTrue(TEXT("second real factory constructs a replacement owner"),
            SecondHost && SecondHost->Descriptor && SecondHost->Journal && SecondHost->Acquisition)) return false;
        TestTrue(TEXT("replacement starts only after the old owner drained"), FirstHost->Closed && FirstHost->Acquisition->HasFinished());
        TestTrue(TEXT("replacement allocates a strictly newer epoch"), SecondHost->Session > FirstHost->Session);
        TestTrue(TEXT("replacement producer and journal identities follow its new epoch"),
            SecondHost->Descriptor->Epoch == SecondHost->Session && SecondHost->Descriptor->Producer == SecondHost->Session
            && SecondHost->Descriptor->Journal == SecondHost->Session && SecondHost->Journal != OldJournal);
        TestTrue(TEXT("real worker admits the replacement keyboard publication"),
            SecondHost->Acquisition->WaitForFirstPublication(HostConfig.StartupTimeout));
        TestTrue(TEXT("replacement startup is ready"),
            SecondHost->Acquisition->StartupState(HostConfig.StartupTimeout) == EAcquisitionStartup::Ready);
        TestFalse(TEXT("healthy keyboard baseline is not a retryable failure"), SecondHost->Acquisition->IsRetryableUnavailable());
        TestEqual(TEXT("two real factory calls create exactly two sources"), Fixture.Calls, 2u);
        const auto KeyboardBaseline = SecondHost->Journal->ReadControlBaseline();
        if (!TestTrue(TEXT("replacement journal contains a fresh complete keyboard baseline"),
            KeyboardBaseline && KeyboardBaseline->State.Kind == ERawDeviceKind::Keyboard
            && KeyboardBaseline->State.DeviceId == 1 && KeyboardBaseline->State.FreshBaseline
            && KeyboardBaseline->State.Generation.Value == 1 && KeyboardBaseline->State.Sequence == 1)) return false;
        TestEqual(TEXT("replacement exposes the keyboard command"), KeyboardBaseline->State.State.Values[0].Value, 1.f);
        const auto Admission = SecondHost->Journal->ReadInitialOwnerAdmission();
        TestTrue(TEXT("factory recovery never fabricates a physical consumption"),
            Admission.DeliverySequence == 0 && Admission.NextFrame == HostConfig.FirstFrame && !Admission.HasFrozenCutoff);
        // Deliberately reject an obsolete session ticket, then retire this test
        // owner; rejection must retain its existing strict journal fencing.
        TestFalse(TEXT("old acquisition ticket cannot publish into the new epoch"), SecondHost->Journal->Publish(OldTicket, Reading(false, 2)));
        TestTrue(TEXT("replacement host closes after the stale-ticket witness"), SecondHost->CloseAtBoundary());
        TestTrue(TEXT("replacement close drains its worker"), SecondHost->Acquisition->HasFinished());
        TestTrue(TEXT("repeated closure is idempotent"), FirstHost->CloseAtBoundary() && SecondHost->CloseAtBoundary());
        if (!TestTrue(TEXT("replacement source provenance is retained"), Fixture.Sources.size() == 2)) return false;
        TestEqual(TEXT("replacement source closes exactly once"), Fixture.Sources[1]->CloseCalls.load(), 1u);
        TestTrue(TEXT("replacement closes on its own acquisition thread"), Fixture.Sources[1]->CloseOnPumpThread.load());
    }
    TestTrue(TEXT("the exact previously registered product bridge is restored"), ProductRawBridge.load() == OriginalBridge);
    TestTrue(TEXT("private fixture scope restores its prior context"), ActiveFixture == OriginalFixture);
    return !HasAnyErrors();
}
#endif
