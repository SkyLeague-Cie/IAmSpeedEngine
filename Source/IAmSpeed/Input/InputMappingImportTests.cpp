#if WITH_DEV_AUTOMATION_TESTS
#include "InputMappingImport.h"
#include "InputAction.h"
#include "InputMappingContext.h"
#include "InputModifiers.h"
#include "InputTriggers.h"
#include "ActionMapping.h"
#include <limits>
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
	const FKey MouseKeys[] = {EKeys::LeftMouseButton, EKeys::RightMouseButton, EKeys::MiddleMouseButton};
	for (int32 I = 0; I < 3; ++I) Context->GetMapping(I).Key = MouseKeys[I];
	Description = Seed();
	TestTrue(TEXT("three mouse buttons import into a distinct canonical namespace"), ImportInputMappings(Context.Get(), Actions, Description, Error));
	if (Description.Mapping.size() == 3)
		for (int32 I = 0; I < 3; ++I)
			TestTrue(TEXT("mouse button identity preserved without keyboard VK translation"),
				Description.Mapping[I].Control.Kind == ERawControlKind::MouseButton && Description.Mapping[I].Control.Code == I);
	for (const FKey& Unsupported : {EKeys::ThumbMouseButton, EKeys::MouseScrollUp, EKeys::MouseScrollDown, EKeys::MouseX, EKeys::MouseWheelAxis})
	{
		Context->GetMapping(0).Key = Unsupported; Description = Seed();
		TestFalse(TEXT("unsupported mouse capability fails whole import"), ImportInputMappings(Context.Get(), Actions, Description, Error));
		TestTrue(TEXT("unsupported key is reported and no mapping prefix escapes"), Error.Contains(Unsupported.ToString()) && Description.Mapping.empty());
	}
	for (int32 I = 0; I < 3; ++I) Context->GetMapping(I).Key = Keys[I];
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FInputMappingExplicitUnboundTest,"IAmSpeed.Input.MappingImportExplicitUnbound01621",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FInputMappingExplicitUnboundTest::RunTest(const FString&)
{
    using namespace Speed::Input::V2;
    struct FFixture
    {
        TStrongObjectPtr<UInputMappingContext> Context;
        TMap<const UInputAction*,Speed::Input::FActionId> Actions;
        FInputActionContractDescription Description;
        FFixture():Context(NewObject<UInputMappingContext>())
        {
            Description.Revision={1};Description.Actions=FInputActionContract::BaseActions();
            Description.Physical={{0,EPhysicalDestination::Throttle},{1,EPhysicalDestination::Brake},{2,EPhysicalDestination::Steering}};
            for(Speed::Input::FActionId Id=0;Id<3;++Id)
            {
                auto* Action=NewObject<UInputAction>(Context.Get());Action->ValueType=EInputActionValueType::Axis1D;
                Actions.Add(Action,Id);Context->MapKey(Action,EKeys::Invalid);
            }
        }
        TSet<Speed::Input::FActionId> EmptyIds()const
        {TSet<Speed::Input::FActionId> Result;for(Speed::Input::FActionId Id=0;Id<3;++Id)Result.Add(Id);return Result;}
    };
    FString Error;
    {
        FFixture F;auto Strict=F.Description;
        TestFalse(TEXT("old import API remains strict for unbound base actions"),ImportInputMappings(F.Context.Get(),F.Actions,Strict,Error));
        TestTrue(TEXT("strict refusal publishes no prefix"),Strict.Mapping.empty());
        if(!TestTrue(TEXT("explicit policy admits real authored unbound base slots"),
            ImportInputMappingsAllowUnbound(F.Context.Get(),F.Actions,F.EmptyIds(),F.Description,Error)))return false;
        TestTrue(TEXT("unbound import creates zero raw bindings"),F.Description.Mapping.empty());
        for(const auto& Action:F.Description.Actions)TestTrue(TEXT("base canonical wiring stays Wired"),Action.Wiring==EActionWiring::Wired);
        TestEqual(TEXT("all three physical destinations remain declared"),F.Description.Physical.size(),size_t(3));
        auto Contract=FInputActionContract::Create(F.Description);
        TestTrue(TEXT("actual native contract and mapper retain admission"),Contract&&FActionMapper::SupportsContract(*Contract));
    }
    {
        FFixture F;TSet<Speed::Input::FActionId> Empty;
        TestFalse(TEXT("new API with no explicit policy does not relax required-action checking"),
            ImportInputMappingsAllowUnbound(F.Context.Get(),F.Actions,Empty,F.Description,Error));
    }
    for(int32 Case=0;Case<9;++Case)
    {
        FFixture F;auto Allowed=F.EmptyIds();auto* Asset=const_cast<UInputAction*>(F.Context->GetMappings()[0].Action.Get());
        if(Case==0)F.Actions.Remove(Asset);
        if(Case==1)F.Context->UnmapAllKeysFromAction(Asset);
        if(Case==2)F.Context->GetMapping(0).Key=FKey(TEXT("UnknownExplicitUnboundKey"));
        if(Case==3)F.Context->GetMapping(0).Key=EKeys::Gamepad_RightTriggerAxis;
        if(Case==4)F.Context->GetMapping(0).Triggers.Add(NewObject<UInputTriggerHold>(F.Context.Get()));
        if(Case==5)F.Context->GetMapping(0).Modifiers.Add(NewObject<UInputModifierSmooth>(F.Context.Get()));
        if(Case==6)Asset->Modifiers.Add(NewObject<UInputModifierSmooth>(F.Context.Get()));
        if(Case==7)Asset->ValueType=EInputActionValueType::Axis3D;
        if(Case==8)
        {
            auto* Modifier=NewObject<UInputModifierScalar>(F.Context.Get());
            Modifier->Scalar.X=std::numeric_limits<float>::quiet_NaN();F.Context->GetMapping(0).Modifiers.Add(Modifier);
        }
        TestFalse(TEXT("missing rows/assets, unknown/assigned keys and invalid metadata cannot masquerade as unbound"),
            ImportInputMappingsAllowUnbound(F.Context.Get(),F.Actions,Allowed,F.Description,Error));
        TestTrue(TEXT("explicit-unbound refusal leaves destination untouched"),F.Description.Mapping.empty());
    }
    {
        FFixture F;auto* Other=NewObject<UInputAction>(F.Context.Get());Other->ValueType=EInputActionValueType::Axis1D;
        F.Actions.Add(Other,0);
        TestFalse(TEXT("ambiguous action identity is never an empty-slot proof"),
            ImportInputMappingsAllowUnbound(F.Context.Get(),F.Actions,F.EmptyIds(),F.Description,Error));
    }
    {
        FFixture F;auto* Foreign=NewObject<UInputAction>(F.Context.Get());F.Context->MapKey(Foreign,EKeys::Invalid);
        TestFalse(TEXT("an unknown authored row is not ignored even with KeyNone"),
            ImportInputMappingsAllowUnbound(F.Context.Get(),F.Actions,F.EmptyIds(),F.Description,Error));
    }
    {
        FFixture F;auto* Modifier=NewObject<UInputModifierNegate>(F.Context.Get());
        F.Context->GetMapping(0).Modifiers.Add(Modifier);
        TestTrue(TEXT("supported metadata on an unbound row is validated without inventing a key"),
            ImportInputMappingsAllowUnbound(F.Context.Get(),F.Actions,F.EmptyIds(),F.Description,Error));
        TestTrue(TEXT("validated unbound modifiers still produce no raw bindings"),F.Description.Mapping.empty());
    }
    return !HasAnyErrors(); // Metadata only; no acquisition, sample or physical frame is created.
}
#endif
