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
	const auto Options = UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(false)
		.RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false).SetTransactional(false);
	UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, NAME_None, nullptr, true, ERHIFeatureLevel::Num, &Options);
	if (!TestNotNull(TEXT("input lifecycle world"), World)) return false;
	GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
	ON_SCOPE_EXIT { World->DestroyWorld(false); GEngine->DestroyWorldContext(World); };
	if (!TestNull(TEXT("input lifecycle deliberately has no physics scene"), World->GetPhysicsScene())) return false;
	auto* Controller = World->SpawnActor<ASpeedController>();
	auto* First = World->SpawnActorDeferred<ASpeedCar>(ASpeedCar::StaticClass(), FTransform::Identity);
	auto* Second = World->SpawnActorDeferred<ASpeedCar>(ASpeedCar::StaticClass(), FTransform::Identity);
	if (!TestNotNull(TEXT("controller"), Controller) || !TestNotNull(TEXT("first pawn"), First) ||
		!TestNotNull(TEXT("second pawn"), Second)) return false;
	auto* FirstMovement = CastChecked<USpeedWheeledComponent>(First->GetVehicleMovement());
	auto* SecondMovement = CastChecked<USpeedWheeledComponent>(Second->GetVehicleMovement());
	FirstMovement->EnableGenericCameraInput(true);
	SecondMovement->EnableGenericCameraInput(true);
	TStrongObjectPtr<UEnhancedInputComponent> Input(NewObject<UEnhancedInputComponent>(Controller));
	Controller->SetupEnhancedInputComponent(Input.Get());
	TestEqual(TEXT("no native UE physical bindings"), Input->GetActionEventBindings().Num(), 0);
	int32 HistoryFrame = 0;
	auto Capture = [&](USpeedWheeledComponent* Movement)
	{
		const int32 Frame = ++HistoryFrame;
		return Movement->CaptureNetworkCameraInput(Frame, Frame);
	};
	Controller->Possess(First);
	TestTrue(TEXT("first possession acknowledged"), Controller->GetPawn() == First);
	First->SetCameraBackInput(true);
	First->SetCameraYawInput(1);
	First->SetCameraPitchInput(-1);
	const auto Held = Capture(FirstMovement);
	TestTrue(TEXT("current owner retains complete camera axes"), Held.IsBack() && Held.Yaw == 127 && Held.Pitch == -127);
	TestFalse(TEXT("other owner receives no camera input"), Capture(SecondMovement).IsPresent());
	First->SetCameraBackInput(false);
	First->SetCameraYawInput(0);
	First->SetCameraPitchInput(0);
	const auto Released = Capture(FirstMovement);
	TestTrue(TEXT("explicit release clears every axis"),
		!Released.IsBack() && Released.Yaw == 0 && Released.Pitch == 0);
	First->SetCameraBackInput(true);
	First->SetCameraYawInput(1);
	Second->SetCameraPitchInput(-1); // Simulate input left on a previously controlled pawn.
	Controller->Possess(Second);
	TestTrue(TEXT("replacement possession acknowledged"), Controller->GetPawn() == Second);
	TestFalse(TEXT("old pawn held input cleared by possession"), Capture(FirstMovement).IsPresent());
	TestFalse(TEXT("new pawn old held input cleared by possession"), Capture(SecondMovement).IsPresent());
	Second->SetCameraPitchInput(-1);
	TestEqual(TEXT("new owner accepts fresh camera input"), int32(Capture(SecondMovement).Pitch), -127);
	Controller->UnPossess();
	TestNull(TEXT("unpossessed controller"), Controller->GetPawn());
	TestFalse(TEXT("unpossession clears last pawn mailbox"), Capture(SecondMovement).IsPresent());
	return true;
}
#endif
