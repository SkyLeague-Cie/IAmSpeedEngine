#if WITH_DEV_AUTOMATION_TESTS

#include "IAmSpeed/Actors/SpeedCar.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/Input/DeviceInputSession.h"
#include "IAmSpeed/World/Simulation/RealTimeSimulation.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "ChaosVehicleWheel.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTLS.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include "SimpleVehicle.h"
#include "SuspensionSystem.h"
#include "WheelSystem.h"

namespace
{
enum class EDeviceLifecycleWitness : uint8 { StepBegin, Produce, BeforePublication, StepEnd };
struct FDeviceLifecycleWitness
{
	EDeviceLifecycleWitness Stage;
	uint64 CanonicalFrame;
	uint32 LocalFrame;
	bool bHasPublishedInput;
	uint64 PublishedInputFrame;
};

// Forwards every request unchanged to the real device session.
class FDeviceLifecycleCountedProducer final : public Speed::Input::IInputProducer
{
public:
	FDeviceLifecycleCountedProducer(std::shared_ptr<Speed::Input::FDeviceInputSession> InSource,
		TFunction<void(uint64)> InObserve)
		: Source(std::move(InSource)), Observe(MoveTemp(InObserve)) {}
	std::optional<Speed::Input::FInputFrame> Produce(Speed::Input::FFrameNumber Frame) override
	{
		++Calls;
		Observe(Frame);
		return Source->Produce(Frame);
	}
	uint32 Calls = 0; // Only the worker writes; inspected after join.
private:
	std::shared_ptr<Speed::Input::FDeviceInputSession> Source;
	TFunction<void(uint64)> Observe;
};

// This observer holds values/stream only: no UObject or gameplay mutation.
class FDeviceLifecyclePublicationObserver final : public ISimulationPresentationProducer
{
public:
	FDeviceLifecyclePublicationObserver(uint64 InOwner, std::shared_ptr<Speed::Input::FInputStream> InStream,
		TArray<FDeviceLifecycleWitness>& InWitnesses)
		: Owner(InOwner), Stream(std::move(InStream)), Witnesses(InWitnesses) {}
	uint64 OwnerStableId() const override { return Owner; }
	uint32 Channel() const override { return 0x5533; }
	void Produce(const FSimulationSnapshot& Snapshot, FSimulationPresentationOutput& Out) override
	{
		const auto Previous = Stream->ReadLatest();
		Witnesses.Add({EDeviceLifecycleWitness::BeforePublication, Snapshot.NumFrame, 0,
			Previous.has_value(), Previous ? Previous->Frame.GetConsumptionFrame() : 0});
		Out.NumFrame = Snapshot.NumFrame;
	}
	void InvalidateTimeline() override {}
private:
	uint64 Owner;
	std::shared_ptr<Speed::Input::FInputStream> Stream;
	TArray<FDeviceLifecycleWitness>& Witnesses;
};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedProducedDeviceLifecycleTest,
	"IAmSpeed.Simulation.ProducedDeviceLifecycle",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedProducedDeviceLifecycleTest::RunTest(const FString& Parameters)
{
	using namespace Speed::Input;
	if (!TestTrue(TEXT("fixture setup runs on game thread"), IsInGameThread())) return false;
	const uint32 FixtureThreadId = FPlatformTLS::GetCurrentThreadId();
	constexpr uint32 FrameCount = 10;
	constexpr FActionId DigitalTestAction = 3; // Fixture-only; no gameplay binding.
	const bool Held[FrameCount] = {true, true, false, true, false, false, false, true, true, true};
	const bool Reset[FrameCount] = {true, false, true, true, true, false, true, false, true, false};
	const uint64 SourceFrames[FrameCount] = {2, 3, 5, 7, 8, 8, 9, 10, 13, 13};
	const auto Options = UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(false)
		.RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false)
		.ShouldSimulatePhysics(false).SetTransactional(false);
	UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, NAME_None, nullptr, true,
		ERHIFeatureLevel::Num, &Options);
	if (!TestNotNull(TEXT("worker fixture world"), World)) return false;
	GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
	ON_SCOPE_EXIT { World->DestroyWorld(false); GEngine->DestroyWorldContext(World); };
	ASpeedCar* Car = World->SpawnActor<ASpeedCar>();
	ARealTimeSimulation* Driver = World->SpawnActor<ARealTimeSimulation>();
	USpeedWorldSubsystem* Bridge = World->GetSubsystem<USpeedWorldSubsystem>();
	if (!TestNotNull(TEXT("car"), Car) || !TestNotNull(TEXT("driver"), Driver)
		|| !TestNotNull(TEXT("bridge"), Bridge)) return false;
	USpeedWheeledComponent* Component = Cast<USpeedWheeledComponent>(Car->GetVehicleMovement());
	if (!TestNotNull(TEXT("production wheeled component"), Component)) return false;
	Driver->SpeedWorldSubsystem = Bridge;

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

	std::array<bool, ActionCount> Digital{};
	Digital[DigitalTestAction] = true;
	auto Session = std::make_shared<FDeviceInputSession>(74, Digital);
	FActionValues HeldValues{};
	HeldValues[Throttle] = 200; HeldValues[Brake] = 80; HeldValues[Steering] = -60;
	HeldValues[DigitalTestAction] = 1;
	auto ReleasedValues = HeldValues;
	ReleasedValues[DigitalTestAction] = 0;
	std::optional<FDeviceInputSession::FGeneration> Connected, Disconnected, Reconnected, Paused, Resumed;
	TArray<FDeviceLifecycleWitness> Witnesses;
	Witnesses.Reserve(FrameCount * 4);
	std::shared_ptr<FInputStream> Stream;
	auto Counted = std::make_shared<FDeviceLifecycleCountedProducer>(Session, [&](uint64 C)
	{
		// Produce runs under the stream mutex: never re-enter that stream here.
		// Publication is observed at StepBegin and the separate snapshot callback.
		Witnesses.Add({EDeviceLifecycleWitness::Produce, C, Component->NumFrame(), false, 0});
	});
	Stream = std::make_shared<FInputStream>(Counted);
	Component->SetFrameInputStream(Stream);
	ON_SCOPE_EXIT { Component->SetFrameInputStream(nullptr); };
	auto Observer = MakeShared<FDeviceLifecyclePublicationObserver, ESPMode::ThreadSafe>(
		Bridge->GetSimulationStableId(*Component), Stream, Witnesses);
	if (!TestTrue(TEXT("pre-publication observer registered"), Driver->RegisterPresentationProducer(Observer))) return false;
	ON_SCOPE_EXIT { Driver->UnregisterPresentationProducer(Observer); };
	TestFalse(TEXT("attachment has no publication"), Stream->ReadLatest().has_value());
	TestFalse(TEXT("legacy test override disabled"), Component->IsTestInputOverrideEnabled());

	struct FCompletedFrame { bool bGlobal = false; uint64 GlobalFrame = 0; uint64 GlobalSerial = 0;
		uint64 InputSerial = 0; int32 Target = 0; int32 BrakeTarget = 0; int32 SteeringTarget = 0; bool LifecycleValid = false; uint32 LocalFrame = 0;
		uint16 CountdownBefore = 0; uint16 CountdownAfter = 0;
		bool bCountdownStartedBefore = false; bool bCountdownStartedAfter = false;
		bool bCanMoveBefore = false; bool bCanMoveAfter = false;
		std::optional<FInputFrame> InputCopy; };
	FCompletedFrame Completed[FrameCount];
	uint32 Steps = 0;
	uint32 WorkerThreadId = 0;
	bool bFailed = false;
	FSimulationWorker Worker([&]
	{
		if (Steps == FrameCount) return ESimulationWorkerResult::Complete;
		WorkerThreadId = FPlatformTLS::GetCurrentThreadId();
		const uint64 C = Driver->CanonicalNumFrame;
		// Lifecycle events are injected only at canonical boundaries by this owner.
		// No GT scheduling or OS callback timing is inferred from this fixture.
		bool LifecycleValid = true;
		switch (C)
		{
		case 0:
			Connected = Session->SetConnected(true);
			LifecycleValid = Connected && Session->Submit(*Connected, 1, HeldValues);
			break;
		case 1:
			LifecycleValid = Connected && Session->Submit(*Connected, 2, ReleasedValues);
			break;
		case 2:
			// Queue a real press, then cancel it before the next physical consume.
			LifecycleValid = Connected && Session->Submit(*Connected, 3, HeldValues);
			Disconnected = Session->SetConnected(false);
			LifecycleValid = LifecycleValid && Disconnected && Connected
				&& *Disconnected > *Connected && !Session->Submit(*Connected, 4, HeldValues)
				&& !Session->Submit(*Disconnected, 1, HeldValues);
			break;
		case 3:
			Reconnected = Session->SetConnected(true);
			LifecycleValid = Reconnected && Disconnected && *Reconnected > *Disconnected
				&& !Session->Submit(*Disconnected, 2, HeldValues)
				&& Session->Submit(*Reconnected, 1, HeldValues);
			break;
		case 4:
			Paused = Session->SetPaused(true);
			LifecycleValid = Paused && Reconnected && *Paused > *Reconnected
				&& !Session->Submit(*Reconnected, 2, HeldValues)
				&& !Session->Submit(*Paused, 1, HeldValues);
			break;
		case 5:
			LifecycleValid = Paused && !Session->Submit(*Paused, 2, HeldValues);
			break;
		case 6:
			Resumed = Session->SetPaused(false);
			LifecycleValid = Resumed && Paused && Reconnected && *Resumed > *Paused
				&& !Session->Submit(*Paused, 3, HeldValues)
				&& !Session->Submit(*Reconnected, 3, HeldValues);
			break; // No fresh reading yet: must remain neutral.
		case 7:
			LifecycleValid = Resumed && Session->Submit(*Resumed, 1, HeldValues);
			break;
		case 8:
			Paused = Session->SetPaused(true);
			Resumed = Session->SetPaused(false);
			LifecycleValid = Paused && Resumed && *Resumed > *Paused
				&& !Session->Submit(*Paused, 1, HeldValues)
				&& Session->Submit(*Resumed, 1, HeldValues);
			break; // Fresh reading in the resume boundary itself.
		default: break;
		}
		const auto Before = Stream->ReadLatest();
		Witnesses.Add({EDeviceLifecycleWitness::StepBegin, C, Component->NumFrame(), Before.has_value(),
			Before ? Before->Frame.GetConsumptionFrame() : 0});
		auto& Result = Completed[Steps];
		Result.LifecycleValid = LifecycleValid;
		Result.CountdownBefore = Component->WheeledPhysicsState.nbFramesbeforeCanMove;
		Result.bCountdownStartedBefore = Component->CountdownHasStarted();
		Result.bCanMoveBefore = Component->CanMove();
		if (!Driver->RunCanonicalFrames(1)) { bFailed = true; return ESimulationWorkerResult::Failed; }
		FSimulationSnapshot Global;
		Result.bGlobal = Driver->ReadLatestSimulationSnapshot(Global);
		Result.GlobalFrame = Global.NumFrame; Result.GlobalSerial = Global.PublicationSerial;
		const auto Published = Stream->ReadLatest();
		Result.InputSerial = Published ? Published->Serial : 0;
		if (Published) Result.InputCopy = Published->Frame;
		Result.Target = int32((Stream->ReadRecorded(Component->NumFrame() - 1) ? Stream->ReadRecorded(Component->NumFrame() - 1)->GetActions()[Throttle] : 0));
		Result.BrakeTarget = int32((Stream->ReadRecorded(Component->NumFrame() - 1) ? Stream->ReadRecorded(Component->NumFrame() - 1)->GetActions()[Brake] : 0));
		Result.SteeringTarget = int32((Stream->ReadRecorded(Component->NumFrame() - 1) ? Stream->ReadRecorded(Component->NumFrame() - 1)->GetActions()[Steering] : 0));
		Result.LocalFrame = Component->NumFrame();
		Result.CountdownAfter = Component->WheeledPhysicsState.nbFramesbeforeCanMove;
		Result.bCountdownStartedAfter = Component->CountdownHasStarted();
		Result.bCanMoveAfter = Component->CanMove();
		Witnesses.Add({EDeviceLifecycleWitness::StepEnd, C, Component->NumFrame(), Published.has_value(),
			Published ? Published->Frame.GetConsumptionFrame() : 0});
		++Steps;
		return ESimulationWorkerResult::Advanced;
	}, [](FSimulationWorkerWaitContext&) {});
	ON_SCOPE_EXIT { Worker.StopAndJoin(); };
	if (!TestTrue(TEXT("real worker starts"), Worker.Start())) return false;
	const double Deadline = FPlatformTime::Seconds() + 5.0;
	while (Worker.IsRunning() && FPlatformTime::Seconds() < Deadline) FPlatformProcess::Sleep(0.001f);
	const bool bFinished = !Worker.IsRunning();
	Worker.StopAndJoin(); // No test assertion or teardown races the worker.
	TestTrue(TEXT("finite run completed within deadline"), bFinished);
	TestFalse(TEXT("no canonical frame rejected"), bFailed);
	TestTrue(TEXT("canonical work ran on a separate worker thread"), WorkerThreadId != 0 && WorkerThreadId != FixtureThreadId);
	TestEqual(TEXT("all real frames completed"), Steps, FrameCount);
	TestEqual(TEXT("one producer request per canonical frame"), Counted->Calls, FrameCount);
	TestEqual(TEXT("four ordered witnesses per frame"), Witnesses.Num(), int32(FrameCount * 4));
	if (Steps != FrameCount || Witnesses.Num() != FrameCount * 4) return false;
	for (uint32 C = 0; C < FrameCount; ++C)
	{
		const FString Label = FString::Printf(TEXT("C%u"), C);
		for (uint32 Stage = 0; Stage < 4; ++Stage)
		{
			const auto& Seen = Witnesses[C * 4 + Stage];
			TestEqual(Label + TEXT(" witness stage"), uint8(Seen.Stage), uint8(Stage));
			TestEqual(Label + TEXT(" witness canonical frame"), Seen.CanonicalFrame, uint64(C));
			const bool bAfterCommit = Stage == 3;
			if (Stage != 1)
			{
				TestEqual(Label + TEXT(" publication availability"), Seen.bHasPublishedInput, bAfterCommit || C != 0);
				if (Seen.bHasPublishedInput)
					TestEqual(Label + TEXT(" publication frame"), Seen.PublishedInputFrame, uint64(bAfterCommit ? C : C - 1));
			}
			if (Stage == 1 || Stage == 3)
				TestEqual(Label + TEXT(" local origin from real prepare"), Seen.LocalFrame, C + 1);
		}
		const auto& Result = Completed[C];
		TestTrue(Label + TEXT(" global snapshot committed"), Result.bGlobal);
		TestEqual(Label + TEXT(" global frame"), Result.GlobalFrame, uint64(C));
		TestEqual(Label + TEXT(" global serial"), Result.GlobalSerial, uint64(C + 1));
		TestEqual(Label + TEXT(" input serial"), Result.InputSerial, uint64(C + 1));
		TestTrue(Label + TEXT(" lifecycle generation and admission checks"), Result.LifecycleValid);
		TestEqual(Label + TEXT(" throttle target via real session"), Result.Target, Held[C] ? 200 : 0);
		TestEqual(Label + TEXT(" brake target via real session"), Result.BrakeTarget, Held[C] ? 80 : 0);
		TestEqual(Label + TEXT(" steering target via real session"), Result.SteeringTarget, Held[C] ? -60 : 0);
		TestEqual(Label + TEXT(" natural countdown before frame"), int32(Result.CountdownBefore), int32(InitialCountdown));
		TestEqual(Label + TEXT(" natural countdown after frame"), int32(Result.CountdownAfter), int32(InitialCountdown));
		TestFalse(Label + TEXT(" countdown was not started"), Result.bCountdownStartedBefore || Result.bCountdownStartedAfter);
		TestFalse(Label + TEXT(" no movement lifecycle transition"), Result.bCanMoveBefore || Result.bCanMoveAfter);
		const auto Recorded = Stream->ReadRecorded(C);
		TestTrue(Label + TEXT(" history retains frame/reset/actions"), Recorded
			&& Recorded->GetConsumptionFrame() == C && Recorded->RequiresReset() == Reset[C]
			&& Recorded->GetSourceFrame() == SourceFrames[C]
			&& Recorded->GetActions()[Throttle] == (Held[C] ? 200 : 0)
			&& Recorded->GetActions()[Brake] == (Held[C] ? 80 : 0)
			&& Recorded->GetActions()[Steering] == (Held[C] ? -60 : 0)
			&& Recorded->GetActions()[DigitalTestAction] == (Held[C] && C != 1 ? 1 : 0));
		TestTrue(Label + TEXT(" later frames preserve published value copy"), Recorded && Result.InputCopy
			&& Recorded->GetActions() == Result.InputCopy->GetActions()
			&& Recorded->GetSourceFrame() == Result.InputCopy->GetSourceFrame()
			&& Recorded->GetProducer().Id == Result.InputCopy->GetProducer().Id
			&& Recorded->RequiresReset() == Result.InputCopy->RequiresReset()
			&& Recorded->GetEdgeCount() == (C == 1 ? 1u : 0u)
			&& Recorded->GetEdgeCount() == Result.InputCopy->GetEdgeCount());
		if (C == 1 && Recorded && Recorded->GetEdgeCount() == 1 && Result.InputCopy)
		{
			const auto& Edge = Recorded->GetEdges()[0];
			const auto& Copy = Result.InputCopy->GetEdges()[0];
			TestTrue(Label + TEXT(" actual release retained, no synthetic lifecycle edges"),
				Edge.Action == DigitalTestAction && Edge.Kind == EEdgeKind::Stop && Edge.SourceFrame == 3
				&& Copy.Action == Edge.Action && Copy.Kind == Edge.Kind && Copy.SourceFrame == Edge.SourceFrame);
		}
	}
	for (uint32 Read = 0; Read < 16; ++Read)
	{
		FPresentationInputScope Scope;
		const auto Latest = Stream->ReadLatest();
		TestTrue(TEXT("presentation only observes completed final input"), Latest
			&& Latest->Serial == FrameCount && Latest->Frame.GetConsumptionFrame() == FrameCount - 1);
	}
	TestEqual(TEXT("presentation reads never repoll"), Counted->Calls, FrameCount);
	TestEqual(TEXT("driver advanced only through real steps"), Driver->CanonicalNumFrame, uint64(FrameCount));
	const auto LatestBeforeDetach = Stream->ReadLatest();
	Component->SetFrameInputStream(nullptr); // Worker already joined; retained stream is inactive.
	TestFalse(TEXT("detached stream cannot acquire a new frame"), Stream->Consume(FrameCount).has_value());
	TestFalse(TEXT("detached stream cannot publish even an old frame"), Stream->PublishCompleted(FrameCount - 1));
	TestEqual(TEXT("stop and detach cause no producer polling"), Counted->Calls, FrameCount);
	const auto LatestAfterDetach = Stream->ReadLatest();
	TestTrue(TEXT("detach leaves the last completed observation unchanged"), LatestBeforeDetach && LatestAfterDetach
		&& LatestBeforeDetach->Serial == LatestAfterDetach->Serial
		&& LatestBeforeDetach->Frame.GetActions() == LatestAfterDetach->Frame.GetActions());
	for (uint32 C = 0; C < FrameCount; ++C)
	{
		const auto Recorded = Stream->ReadRecorded(C);
		const auto& Copy = Completed[C].InputCopy;
		TestTrue(TEXT("detachment preserves every copied historical snapshot"), Recorded && Copy
			&& Recorded->GetConsumptionFrame() == Copy->GetConsumptionFrame()
			&& Recorded->GetSourceFrame() == Copy->GetSourceFrame()
			&& Recorded->GetActions() == Copy->GetActions()
			&& Recorded->RequiresReset() == Copy->RequiresReset()
			&& Recorded->GetEdgeCount() == Copy->GetEdgeCount());
	}
	return true;
}

#endif
