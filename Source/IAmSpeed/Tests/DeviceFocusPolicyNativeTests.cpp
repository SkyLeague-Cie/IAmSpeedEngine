#include "IAmSpeed/Input/DeviceFocusPolicy.h"
#include "Misc/AutomationTest.h"
#include <thread>
#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedDeviceFocusPolicy,
 "IAmSpeed.Input.DeviceFocusPolicy.ImmutableProcessBinding",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedDeviceFocusPolicy::RunTest(const FString&)
{
 using Speed::Input::FDeviceProcessFocusPolicyLatch;
 FDeviceProcessFocusPolicyLatch Fore;
 TestFalse(TEXT("inert latch does not acquire policy"),Fore.BoundBackground().has_value());
 TestTrue(TEXT("foreground default can bind"),Fore.TryBind(false));
 TestTrue(TEXT("foreground warm reuse accepted"),Fore.TryBind(false));
 TestFalse(TEXT("background change after foreground binding rejected"),Fore.TryBind(true));
 TestTrue(TEXT("failed change leaves foreground policy"),Fore.BoundBackground()==std::optional<bool>(false));
 FDeviceProcessFocusPolicyLatch Back;
 TestTrue(TEXT("explicit background can bind first"),Back.TryBind(true));
 TestTrue(TEXT("background warm reuse accepted"),Back.TryBind(true));
 TestFalse(TEXT("later implicit foreground cannot overwrite background"),Back.TryBind(false));
 for(int I=0;I<16;++I)
 {
  FDeviceProcessFocusPolicyLatch Race;bool F=false,B=false;
  std::thread A([&]{F=Race.TryBind(false);});std::thread C([&]{B=Race.TryBind(true);});A.join();C.join();
  TestTrue(TEXT("exactly one conflicting first policy wins"),F!=B);
  TestTrue(TEXT("winner remains immutable"),Race.BoundBackground()==std::optional<bool>(B)&&Race.TryBind(B)&&!Race.TryBind(!B));
 }
 return !HasAnyErrors();
}
#endif
