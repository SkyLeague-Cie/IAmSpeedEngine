#include "DeviceInputHost.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDeviceRawAcquisitionFactoryTest,
    "IAmSpeed.Input.Menu.RawAcquisitionFactory",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FDeviceRawAcquisitionFactoryTest::RunTest(const FString&)
{
    using namespace Speed::Input;
    using namespace Speed::Input::V2;
    auto Journal = std::make_shared<FRawAcquisitionJournal>(901);
    FDeviceRawAcquisitionConfig Config;
    // Fixture values only; production supplies its existing configured policy.
    Config.Activity.Stick = {0.2f, 0.1f, 0.05f};
    Config.Activity.Trigger = {0.2f, 0.1f, 0.05f};
    Config.Activity.MinimumResidenceFrames = 0;
    Config.Activity.MaximumDevices = 8;
    Config.Activity.HybridDeviceKind = EDeviceKind::Gamepad;
    TestFalse(TEXT("missing journal cannot create an owner"), bool(CreateDeviceRawAcquisition(901, {}, Config)));
    TestFalse(TEXT("zero producer cannot create an owner"), bool(CreateDeviceRawAcquisition(0, Journal, Config)));
    TestFalse(TEXT("another journal session cannot be relabeled"), bool(CreateDeviceRawAcquisition(902, Journal, Config)));
    FDeviceRawAcquisitionConfig Invalid;
    TestFalse(TEXT("no implicit activity policy"), bool(CreateDeviceRawAcquisition(901, Journal, Invalid)));
#if PLATFORM_WINDOWS && !UE_SERVER
    {
        FPresentationInputScope Scope;
        TestFalse(TEXT("presentation cannot create another acquisition path"),
            bool(CreateDeviceRawAcquisition(901, Journal, Config)));
    }
    auto Warm = CreateDeviceInputWarmContext();
    if (!TestTrue(TEXT("inert warm context constructed"), bool(Warm))) return false;
    TestTrue(TEXT("unused warm context closes without native acquisition"), Warm->Close());
    Config.WarmContext = Warm;
    TestFalse(TEXT("closed catalogue cannot be reused"), bool(CreateDeviceRawAcquisition(901, Journal, Config)));
    Config.WarmContext.reset();
    auto Source = CreateDeviceRawAcquisition(901, Journal, Config);
    if (!TestTrue(TEXT("acquisition-only source can be constructed without physical host"), bool(Source))) return false;
    TestFalse(TEXT("construction does not publish or poll a device"), bool(Journal->ReadControlBaseline()));
    TestEqual(TEXT("construction preserves supplied session"), Journal->BeginAcquisition().Session, uint64(901));
    // Do not Pump: this test deliberately does not touch actual hardware.
    TestTrue(TEXT("unused source closes and invalidates its journal"), Source->Close());
    TestTrue(TEXT("close is idempotent"), Source->Close());
    TestTrue(TEXT("consumer observes close"), Journal->ReadControlsSince({901, 0}).Status == EAcquisitionRead::Closed);
#else
    TestFalse(TEXT("unsupported platform does not fabricate a source"),
        bool(CreateDeviceRawAcquisition(901, Journal, Config)));
#endif
    return !HasAnyErrors();
}
#endif
