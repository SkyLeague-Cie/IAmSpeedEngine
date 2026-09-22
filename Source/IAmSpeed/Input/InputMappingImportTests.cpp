#if WITH_DEV_AUTOMATION_TESTS
#include "InputMappingImport.h"
#include "InputAction.h"
#include "InputMappingContext.h"
#include "InputModifiers.h"
#include "Misc/AutomationTest.h"
#include "UObject/StrongObjectPtr.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FInputMappingImportTest, "IAmSpeed.Input.MappingImport",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)
bool FInputMappingImportTest::RunTest(const FString&)
{
	using namespace Speed::Input::V2;
	TStrongObjectPtr<UInputMappingContext> Context(NewObject<UInputMappingContext>());
	TMap<const UInputAction*, Speed::Input::FActionId> Actions;
	const FKey Keys[] = {EKeys::Gamepad_LeftTriggerAxis, EKeys::Gamepad_RightTriggerAxis, EKeys::Gamepad_LeftX};
	for (Speed::Input::FActionId Id = 0; Id < 3; ++Id)
	{
		UInputAction* Action = NewObject<UInputAction>(Context.Get());
		Action->ValueType = EInputActionValueType::Axis1D;
		Actions.Add(Action, Id); Context->MapKey(Action, Keys[Id]);
	}
	const auto Seed = []
	{
		FInputActionContractDescription D; D.Revision = {1}; D.Actions = FInputActionContract::BaseActions();
		D.Physical = {{0, EPhysicalDestination::Throttle}, {1, EPhysicalDestination::Brake}, {2, EPhysicalDestination::Steering}};
		return D;
	};
	FString Error; auto Description = Seed();
	TestTrue(TEXT("canonical gamepad import independent of platform keyboard adapter"), ImportInputMappings(Context.Get(), Actions, Description, Error));
	TestTrue(TEXT("UE default accumulation copied"), Description.Actions[0].Accumulation == EActionAccumulation::HighestAbsolute);
	const UInputAction* Steering = Context->GetMappings()[2].Action;
	Context->MapKey(Steering, EKeys::Invalid);
	Description = Seed();
	TestTrue(TEXT("disabled secondary mapping does not disable valid action"), ImportInputMappings(Context.Get(), Actions, Description, Error));
	TestEqual(TEXT("disabled mapping omitted"), int32(Description.Mapping.size()), 3);
	UInputModifierDeadZone* Deadzone = NewObject<UInputModifierDeadZone>(Context.Get());
	Deadzone->Type = EDeadZoneType::UnscaledRadial;
	Context->GetMapping(2).Modifiers.Add(Deadzone);
	Description = Seed();
	TestFalse(TEXT("unrepresented deadzone refused"), ImportInputMappings(Context.Get(), Actions, Description, Error));
	TestTrue(TEXT("failed import leaves destination intact"), Description.Mapping.empty());
	Deadzone->Type = EDeadZoneType::Axial;
	Description = Seed();
	TestTrue(TEXT("represented deadzone imported"), ImportInputMappings(Context.Get(), Actions, Description, Error));
	Context->GetMapping(2).Modifiers.Add(NewObject<UInputModifierSmooth>(Context.Get()));
	Description = Seed();
	TestFalse(TEXT("frame-dependent smoothing refused"), ImportInputMappings(Context.Get(), Actions, Description, Error));
	Context->GetMapping(2).Modifiers.Reset(); Context->GetMapping(2).Key = EKeys::Invalid;
	Description = Seed();
	TestFalse(TEXT("required action without any valid binding refused"), ImportInputMappings(Context.Get(), Actions, Description, Error));
	AddInfo(TEXT("Asset importer unit fixture; actual saved SkyLeague assets and keyboard layouts require integration qualification."));
	return true;
}
#endif
