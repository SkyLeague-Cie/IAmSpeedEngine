#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Actors/SpeedCar.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/Controllers/SpeedController.h"
#include "IAmSpeed/Input/DeviceInputSession.h"
#include "IAmSpeed/World/Simulation/RealTimeSimulation.h"
#include "IAmSpeed/World/Simulation/SpeedGameMode.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "ChaosVehicleWheel.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "Engine/GameInstance.h"
#include "GameFramework/WorldSettings.h"
#include "GameFramework/PlayerState.h"
#include "HAL/PlatformProcess.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include "SimpleVehicle.h"
#include "SuspensionSystem.h"
#include "WheelSystem.h"
#include <atomic>

namespace
{
class FControllerLifecycleSessionProbe final : public Speed::Input::IInputProducer
{
public:
	FControllerLifecycleSessionProbe() : Session(75, {}) { Token = Session.SetConnected(true); }
	std::optional<Speed::Input::FInputFrame> Produce(Speed::Input::FFrameNumber Frame) override
	{
		++Calls;
		return Session.Produce(Frame);
	}
	Speed::Input::EInputLifecycleResult ApplyLifecyclePause(bool Paused) override
	{
		++Controls;
		if (RejectNextControl)
		{
			RejectNextControl = false;
			return Speed::Input::EInputLifecycleResult::Rejected;
		}
		// Transparent counted forwarding to the real session lifecycle method.
		Token = Session.SetPaused(Paused);
		return Token ? Speed::Input::EInputLifecycleResult::Applied : Speed::Input::EInputLifecycleResult::Rejected;
	}
	Speed::Input::EInputLifecycleResult CancelLifecycle() override
	{
		++Cancellations;
		return Session.CancelLifecycle();
	}
	Speed::Input::FDeviceInputSession Session;
	std::optional<Speed::Input::FDeviceInputSession::FGeneration> Token;
	std::atomic<uint32> Calls{0};
	uint32 Controls = 0, Cancellations = 0; // Controller thread only.
	bool RejectNextControl = false;
};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedControllerInputLifecycleTest,
	"IAmSpeed.Simulation.ControllerInputLifecycle",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedControllerInputLifecycleTest::RunTest(const FString& Parameters)
{
	using namespace Speed::Input;
	if (!TestTrue(TEXT("controller fixture on game thread"), IsInGameThread())) return false;
	const auto Options = UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(false)
		.RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false)
		.ShouldSimulatePhysics(false).SetTransactional(false);
	UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, NAME_None, nullptr, true,
		ERHIFeatureLevel::Num, &Options);
	if (!TestNotNull(TEXT("worker fixture world"), World)) return false;
	GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
	ON_SCOPE_EXIT { World->DestroyWorld(false); GEngine->DestroyWorldContext(World); };
	World->SetGameInstance(NewObject<UGameInstance>(GEngine));
	World->GetWorldSettings()->DefaultGameMode = ASpeedGameMode::StaticClass();
	if (!TestTrue(TEXT("real game mode installed"), World->SetGameMode(FURL()))) return false;
	auto* GameMode = Cast<ASpeedGameMode>(World->GetAuthGameMode());
	if (!TestNotNull(TEXT("speed game mode"), GameMode)) return false;
	auto* Controller = World->SpawnActor<ASpeedController>();
	if (!TestNotNull(TEXT("controller"), Controller)) return false;
	if (!Controller->PlayerState) Controller->SetPlayerState(World->SpawnActor<APlayerState>());
	if (!TestNotNull(TEXT("pause owner player state"), Controller->PlayerState.Get())) return false;
	ASpeedCar* Car = World->SpawnActor<ASpeedCar>();
	ARealTimeSimulation* Driver = World->SpawnActor<ARealTimeSimulation>();
	USpeedWorldSubsystem* Bridge = World->GetSubsystem<USpeedWorldSubsystem>();
	if (!TestNotNull(TEXT("car"), Car) || !TestNotNull(TEXT("driver"), Driver)
		|| !TestNotNull(TEXT("bridge"), Bridge)) return false;
	USpeedWheeledComponent* Component = Cast<USpeedWheeledComponent>(Car->GetVehicleMovement());
	if (!TestNotNull(TEXT("production wheeled component"), Component)) return false;
	Driver->SpeedWorldSubsystem = Bridge;
	GameMode->SpeedSimActor = Driver;

	// A real immutable empty collision world isolates frame ordering from contacts.
	// The normal core step still runs; no fake physical consumer or query backend.
	auto Data = MakeShared<Speed::Analytic::FAnalyticWorldData>();
	if (!TestTrue(TEXT("empty analytical world validates"), Data->FinalizeAndValidate())) return false;
	Bridge->AnalyticWorldData = Data;
	Bridge->SimulationWorld.SetStaticCollisionWorld(MakeUnique<Speed::FAnalyticStaticCollisionWorld>(*Data));
	Component->SetOwner(Car);
	Component->SetAsyncPhysicsTickEnabled(false); // This fixture has exactly one lane.
	ON_SCOPE_EXIT { Bridge->UnregisterSpeedComponent(Component); Bridge->ApplyPendingOps(); };
	Bridge->ApplyPendingOps();
	if (!TestEqual(TEXT("four production wheel subbodies"), Component->WheelSubBodies.Num(), 4)) return false;

	// Same owned storage as the production binding-admission fixture, alive until join.
	Chaos::FSimpleWheelConfig WheelConfig;
	Chaos::FSimpleSuspensionConfig SuspensionConfig;
	SuspensionConfig.SetSuspensionMaxRaise(10);
	SuspensionConfig.SetSuspensionMaxDrop(25);
	auto Vehicle = MakeUnique<Chaos::FSimpleWheeledVehicle>();
	for (int32 Index = 0; Index < 4; ++Index)
	{
		Vehicle->Wheels.Emplace(&WheelConfig);
		Vehicle->Suspension.Emplace(&SuspensionConfig);
		Component->Wheels.Add(NewObject<UChaosVehicleWheel>(Component));
	}
	Component->VehicleSimulationPT = MakeUnique<USpeedSimulation>();
	Component->SkySimulation = static_cast<USpeedSimulation*>(Component->VehicleSimulationPT.Get());
	Component->SkySimulation->Init(Vehicle);
	AddExpectedMessagePlain(TEXT("[WheelSuspensionBinding] subbodies="), ELogVerbosity::Warning,
		EAutomationExpectedMessageFlags::Contains, 1);
	Component->BindWheelSimulationPointers();
	ON_SCOPE_EXIT
	{
		Component->ClearWheelSimulationPointers();
		Component->SkySimulation = nullptr;
		Component->VehicleSimulationPT.Reset(); // Destroy wheel users before their configs.
	};
	FString BindingReason;
	if (!TestTrue(TEXT("production bindings admitted"), Component->ValidateSimulationBindings(BindingReason))
		|| !TestFalse(TEXT("fixture retains default immobility without a CanMove command"), Component->CanMove())
		|| !TestTrue(TEXT("driver sees analytical authority"), Driver->EnsureSimulationWorldReady())) return false;
	Driver->InitializeCanonicalFrame(0.0f);
	if (!TestEqual(TEXT("real driver initial canonical origin"), Driver->CanonicalNumFrame, uint64(0))) return false;
	// The component constructor replaces the state struct default with the
	// configured TimeBeforeCanMove * physics FPS. Observe it without mutation.
	const uint16 InitialCountdown = Component->GetMinNbFramesBeforeCanMove();
	if (!TestTrue(TEXT("configured initial countdown is positive"), InitialCountdown > 0)
		|| !TestEqual(TEXT("initial state uses configured countdown"),
			int32(Component->WheeledPhysicsState.nbFramesbeforeCanMove), int32(InitialCountdown))) return false;

	auto Source = std::make_shared<FControllerLifecycleSessionProbe>();
	const auto BeforePossession = Source->Token;
	if (!TestTrue(TEXT("install lifecycle producer"), Controller->ConfigureInputProducer(Source))) return false;
	Controller->Possess(Car);
	if (!TestTrue(TEXT("real possession"), Controller->GetPawn() == Car)) return false;
	auto Stream = Controller->InputSnapshots;
	if (!TestTrue(TEXT("controller owns stream"), bool(Stream))) return false;
	TestTrue(TEXT("possession creates a fresh generation without poll"), Source->Token && BeforePossession
		&& *Source->Token > *BeforePossession && Source->Calls.load() == 0);
	FActionValues Held{}; Held[Throttle] = 200;
	if (!TestTrue(TEXT("fresh baseline accepted"), Source->Token && Source->Session.Submit(*Source->Token, 1, Held))) return false;

	FEvent* FrameDone = FPlatformProcess::GetSynchEventFromPool(true);
	FEvent* BlockEntered = FPlatformProcess::GetSynchEventFromPool(true);
	FEvent* BlockRelease = FPlatformProcess::GetSynchEventFromPool(true);
	ON_SCOPE_EXIT
	{
		FPlatformProcess::ReturnSynchEventToPool(FrameDone);
		FPlatformProcess::ReturnSynchEventToPool(BlockEntered);
		FPlatformProcess::ReturnSynchEventToPool(BlockRelease);
	};
	std::atomic<uint32> Budget{0}, Steps{0};
	std::atomic<bool> Block{false}, Failed{false};
	struct FControllerFrameWitness { int32 Target = 0; std::optional<FInputFrame> Input; } Frames[3];
	Driver->SimulationWorker = MakeUnique<FSimulationWorker>([&]
	{
		if (Block.exchange(false))
		{
			BlockEntered->Trigger();
			if (!BlockRelease->Wait(5000)) { Failed = true; return ESimulationWorkerResult::Failed; }
		}
		const uint32 Index = Steps.load();
		if (Index >= Budget.load()) return ESimulationWorkerResult::Idle;
		if (Index >= 3 || !Driver->RunCanonicalFrames(1)) { Failed = true; return ESimulationWorkerResult::Failed; }
		Frames[Index].Target = int32(Component->WheeledUserInput.Throttle);
		const auto Latest = Stream->ReadLatest();
		if (Latest) Frames[Index].Input = Latest->Frame;
		Steps.store(Index + 1);
		FrameDone->Trigger();
		return ESimulationWorkerResult::Advanced;
	}, [](FSimulationWorkerWaitContext& Wait) { Wait.WaitUntil(FPlatformTime::Seconds() + .001); });
	ON_SCOPE_EXIT { BlockRelease->Trigger(); Driver->StopOwnedWorker(); };
	if (!TestTrue(TEXT("owned worker starts"), Driver->SimulationWorker->Start())) return false;
	auto RunOne = [&]()
	{
		FrameDone->Reset();
		Budget.store(Steps.load() + 1);
		Driver->ResumeOwnedSimulation();
		const bool Done = FrameDone->Wait(5000);
		const auto Boundary = Driver->TryPauseOwnedSimulation(1000);
		return Done && Boundary == ESimulationQuiescence::BoundaryAcknowledged && !Failed.load();
	};
	if (!TestTrue(TEXT("first real frame"), RunOne())) return false;
	TestEqual(TEXT("held baseline reaches real consumer"), Frames[0].Target, 200);
	const auto HistoryBeforePause = Frames[0].Input;
	const uint32 PollsBeforePause = Source->Calls.load();
	const auto PrePauseToken = Source->Token;
	Driver->ResumeOwnedSimulation();
	if (!TestTrue(TEXT("actual controller pause accepted"), Controller->SetPause(true))) return false;
	TestTrue(TEXT("Unreal and owned worker paused"), Controller->IsPaused() && Driver->IsOwnedSimulationPaused());
	TestFalse(TEXT("paused stream cannot poll"), Stream->Consume(1).has_value());
	TestFalse(TEXT("paused stream cannot republish old frame"), Stream->PublishCompleted(0));
	TestTrue(TEXT("old reading rejected without polling"), PrePauseToken
		&& !Source->Session.Submit(*PrePauseToken, 2, Held) && Source->Calls.load() == PollsBeforePause);
	const auto PausedHistory = Stream->ReadRecorded(0);
	TestTrue(TEXT("pause preserves immutable history"), PausedHistory && HistoryBeforePause
		&& PausedHistory->GetActions() == HistoryBeforePause->GetActions());
	if (!TestTrue(TEXT("actual controller resume accepted"), Controller->SetPause(false))) return false;
	if (!TestTrue(TEXT("resume with no fresh reading runs neutral"), RunOne())) return false;
	TestEqual(TEXT("first resumed physical target is neutral"), Frames[1].Target, 0);
	TestTrue(TEXT("resumed frame carries reset without synthetic edges"), Frames[1].Input
		&& Frames[1].Input->RequiresReset() && Frames[1].Input->GetEdgeCount() == 0);
	if (!TestTrue(TEXT("fresh held reading after resume accepted"), Source->Token
		&& Source->Session.Submit(*Source->Token, 1, Held))) return false;
	if (!TestTrue(TEXT("fresh held frame runs"), RunOne())) return false;
	TestEqual(TEXT("fresh held target applies on first eligible frame"), Frames[2].Target, 200);

	// Standalone GameMode ignores bPauseable=false. The actual rejection case
	// is a separate pre-game world with no authoritative GameMode.
	{
		UWorld* NoModeWorld = UWorld::CreateWorld(EWorldType::Game, false, NAME_None, nullptr, true,
			ERHIFeatureLevel::Num, &Options);
		if (!TestNotNull(TEXT("no-game-mode world"), NoModeWorld)) return false;
		GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(NoModeWorld);
		ON_SCOPE_EXIT { NoModeWorld->DestroyWorld(false); GEngine->DestroyWorldContext(NoModeWorld); };
		auto* NoModeController = NoModeWorld->SpawnActor<ASpeedController>();
		auto* NoModeCar = NoModeWorld->SpawnActorDeferred<ASpeedCar>(ASpeedCar::StaticClass(), FTransform::Identity);
		if (!TestNotNull(TEXT("no-mode controller"), NoModeController)
			|| !TestNotNull(TEXT("no-mode pawn"), NoModeCar)) return false;
		auto NoModeSource = std::make_shared<FControllerLifecycleSessionProbe>();
		if (!TestTrue(TEXT("no-mode producer installed"), NoModeController->ConfigureInputProducer(NoModeSource))) return false;
		NoModeController->Possess(NoModeCar);
		auto NoModeStream = NoModeController->InputSnapshots;
		if (!TestTrue(TEXT("no-mode stream attached"), bool(NoModeStream))) return false;
		const uint32 BeforeRejectedControls = NoModeSource->Controls;
		TestFalse(TEXT("Unreal rejects pause without game mode"), NoModeController->SetPause(true));
		TestEqual(TEXT("rejected pause resets pause then resume generation"), NoModeSource->Controls, BeforeRejectedControls + 2);
		TestFalse(TEXT("rejected Unreal pause reopens source stream"), NoModeStream->IsLifecyclePaused());
		TestEqual(TEXT("rejection handling never polls"), NoModeSource->Calls.load(), uint32(0));
		NoModeController->UnPossess();
	}

	// A genuinely in-flight worker blocks its canonical acknowledgement.
	Block.store(true);
	Driver->ResumeOwnedSimulation();
	if (!TestTrue(TEXT("worker entered controlled in-flight hold"), BlockEntered->Wait(1000))) return false;
	const uint32 BeforeTimeoutControls = Source->Controls;
	const uint32 BeforeTimeoutPolls = Source->Calls.load();
	TestFalse(TEXT("controller timeout rejects pause"), Controller->SetPause(true));
	TestEqual(TEXT("timeout does not mutate source lifecycle"), Source->Controls, BeforeTimeoutControls);
	TestTrue(TEXT("timeout keeps pause requested"), Driver->IsOwnedSimulationPaused());
	Controller->SynchronizeOwnedSimulationPauseWithWorld();
	TestTrue(TEXT("world sync cannot auto-resume a timed-out request"), Driver->IsOwnedSimulationPaused());
	TestEqual(TEXT("timeout has not polled"), Source->Calls.load(), BeforeTimeoutPolls);
	BlockRelease->Trigger();
	if (!TestTrue(TEXT("same request can be acknowledged after release"), Controller->SetPause(true))) return false;
	if (!TestTrue(TEXT("acknowledged retry may resume"), Controller->SetPause(false))) return false;
	Driver->StopOwnedWorker();
	TestEqual(TEXT("all controlled physical frames completed"), Steps.load(), uint32(3));
	TestFalse(TEXT("worker did not fail"), Failed.load());

	TestEqual(TEXT("joined owner reports AlreadyStopped"), uint8(Driver->TryPauseOwnedSimulation(1)),
		uint8(ESimulationQuiescence::AlreadyStopped));
	const uint32 StoppedPolls = Source->Calls.load();
	Source->RejectNextControl = true;
	TestFalse(TEXT("rejected lifecycle fails closed"), Controller->SetPause(true));
	TestTrue(TEXT("rejected control keeps stream and owner paused"), Stream->IsLifecyclePaused()
		&& Driver->IsOwnedSimulationPaused() && Controller->bInputLifecycleFault);
	TestFalse(TEXT("rejected control cannot republish old input"), Stream->PublishCompleted(2));
	TestTrue(TEXT("controller cancellation retry works with joined worker"), Controller->SetPause(true));
	TestEqual(TEXT("stopped cancellation never polls"), Source->Calls.load(), StoppedPolls);
	TestTrue(TEXT("stopped resume resets source without starting a frame"), Controller->SetPause(false));
	TestEqual(TEXT("stopped resume never polls"), Source->Calls.load(), StoppedPolls);
	Controller->UnPossess();
	TestTrue(TEXT("unpossession closes real source"), Source->Cancellations > 0);
	TestFalse(TEXT("retained detached stream cannot acquire"), Stream->Consume(3).has_value());
	TestFalse(TEXT("retained detached stream cannot publish"), Stream->PublishCompleted(2));
	TestEqual(TEXT("unpossession and retained attempts never poll"), Source->Calls.load(), StoppedPolls);
	const auto Retained = Stream->ReadRecorded(0);
	TestTrue(TEXT("old snapshot remains immutable after detach"), Retained && HistoryBeforePause
		&& Retained->GetActions() == HistoryBeforePause->GetActions());

	// Exercise the actual controller EndPlay hook with a new owned session.
	auto EndSource = std::make_shared<FControllerLifecycleSessionProbe>();
	TestTrue(TEXT("new lifecycle install after unpossession"), Controller->ConfigureInputProducer(EndSource));
	Controller->Possess(Car);
	auto EndStream = Controller->InputSnapshots;
	Controller->EndPlay(EEndPlayReason::RemovedFromWorld);
	TestTrue(TEXT("EndPlay cancels source before releasing controller ownership"), EndSource->Cancellations > 0
		&& !Controller->InputSnapshots && !Controller->InputProducer);
	TestTrue(TEXT("retained EndPlay stream is closed"), EndStream && EndStream->IsLifecyclePaused());
	TestEqual(TEXT("EndPlay has no acquisition"), EndSource->Calls.load(), uint32(0));
	return true;
}
#endif
