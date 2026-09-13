#include "IAmSpeed/Actors/SpeedCar.h"

#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/Controllers/SpeedController.h"
#include "EnhancedInputComponent.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "InputAction.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include "UObject/StrongObjectPtr.h"
#include "UObject/UnrealType.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGenericCameraInputLifecycleTest,
	"SkyLeague.Camera.GenericInputLifecycle",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FGenericCameraInputLifecycleTest::RunTest(const FString& Parameters)
{
	// A real possession/input-delegate test before BeginPlay, not a claim that
	// a Chaos worker has resimulated or that the physical camera is activated.
	const auto Options = UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(true)
		.RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false).SetTransactional(false);
	UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, NAME_None, nullptr, true, ERHIFeatureLevel::Num, &Options);
	if (!TestNotNull(TEXT("input lifecycle world"), World)) return false;
	GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
	ON_SCOPE_EXIT { World->DestroyWorld(false); GEngine->DestroyWorldContext(World); };
	auto* Controller = World->SpawnActor<ASpeedController>();
	auto* First = World->SpawnActorDeferred<ASpeedCar>(ASpeedCar::StaticClass(), FTransform::Identity);
	auto* Second = World->SpawnActorDeferred<ASpeedCar>(ASpeedCar::StaticClass(), FTransform::Identity);
	if (!TestNotNull(TEXT("controller"), Controller) || !TestNotNull(TEXT("first pawn"), First) ||
		!TestNotNull(TEXT("second pawn"), Second)) return false;
	auto* FirstMovement = CastChecked<USpeedWheeledComponent>(First->GetVehicleMovement());
	auto* SecondMovement = CastChecked<USpeedWheeledComponent>(Second->GetVehicleMovement());
	FirstMovement->EnableGenericCameraInput(true);
	SecondMovement->EnableGenericCameraInput(true);
	TArray<UInputAction*> Actions;
	for (const FName Name : { FName(TEXT("StartBackCameraAction")), FName(TEXT("CamYawAction")), FName(TEXT("CamPitchAction")) })
	{
		auto* Property = FindFProperty<FObjectPropertyBase>(ASpeedController::StaticClass(), Name);
		if (!TestNotNull(TEXT("generic action property"), Property)) return false;
		auto* Action = NewObject<UInputAction>(Controller);
		Action->ValueType = EInputActionValueType::Axis1D;
		Property->SetObjectPropertyValue_InContainer(Controller, Action);
		Actions.Add(Action);
	}
	TStrongObjectPtr<UEnhancedInputComponent> Input(NewObject<UEnhancedInputComponent>(Controller));
	Controller->SetupEnhancedInputComponent(Input.Get());
	struct FAuthoredActionInstance : FInputActionInstance
	{
		FAuthoredActionInstance(const UInputAction* Action, float InValue) : FInputActionInstance(Action)
		{
			Value = FInputActionValue(InValue);
			TriggerEvent = ETriggerEvent::Triggered;
		}
	};
	auto Execute = [&](int32 ActionIndex, ETriggerEvent Event, float Value)
	{
		int32 Count = 0;
		const FAuthoredActionInstance Instance(Actions[ActionIndex], Value);
		for (const auto& Binding : Input->GetActionEventBindings())
			if (Binding->GetAction() == Actions[ActionIndex] && Binding->GetTriggerEvent() == Event)
			{
				Binding->Execute(Instance);
				++Count;
			}
		TestEqual(TEXT("one delegate executed per camera event"), Count, 1);
	};
	int32 HistoryFrame = 0;
	auto Capture = [&](USpeedWheeledComponent* Movement)
	{
		FNetworkWheeledSpeedInputState Packet;
		Packet.LocalFrame = ++HistoryFrame;
		Packet.BuildData(Movement);
		return Packet.WheeledInput.Camera;
	};
	Controller->Possess(First);
	TestTrue(TEXT("first possession acknowledged"), Controller->GetPawn() == First);
	Execute(0, ETriggerEvent::Started, 1);
	Execute(1, ETriggerEvent::Triggered, 1);
	Execute(2, ETriggerEvent::Triggered, -1);
	const auto Held = Capture(FirstMovement);
	TestTrue(TEXT("six-binding press path reaches complete held input"), Held.IsBack() && Held.Yaw == 127 && Held.Pitch == -127);
	TestFalse(TEXT("other owner receives no camera input"), Capture(SecondMovement).IsPresent());
	Execute(0, ETriggerEvent::Completed, 1);
	Execute(1, ETriggerEvent::Completed, 1);
	Execute(2, ETriggerEvent::Completed, -1);
	const auto Released = Capture(FirstMovement);
	TestTrue(TEXT("Completed delegates clear axes even with a nonzero supplied value"),
		!Released.IsBack() && Released.Yaw == 0 && Released.Pitch == 0);
	Execute(0, ETriggerEvent::Started, 1);
	Execute(1, ETriggerEvent::Triggered, 1);
	Second->SetCameraPitchInput(-1); // Simulate input left on a previously controlled pawn.
	Controller->Possess(Second);
	TestTrue(TEXT("replacement possession acknowledged"), Controller->GetPawn() == Second);
	TestFalse(TEXT("old pawn held input cleared by possession"), Capture(FirstMovement).IsPresent());
	TestFalse(TEXT("new pawn old held input cleared by possession"), Capture(SecondMovement).IsPresent());
	Execute(2, ETriggerEvent::Triggered, -1);
	TestEqual(TEXT("delegates route to replacement pawn"), int32(Capture(SecondMovement).Pitch), -127);
	Controller->UnPossess();
	TestNull(TEXT("unpossessed controller"), Controller->GetPawn());
	TestFalse(TEXT("unpossession clears last pawn mailbox"), Capture(SecondMovement).IsPresent());
	Execute(0, ETriggerEvent::Started, 1);
	TestFalse(TEXT("unpossessed callback cannot mutate old pawn"), Capture(SecondMovement).IsPresent());
	return true;
}
#endif
