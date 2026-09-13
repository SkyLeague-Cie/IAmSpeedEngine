#if WITH_DEV_AUTOMATION_TESTS
#include "SpeedWheeledComponent.h"
#include "IAmSpeed/Actors/SpeedCar.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "IAmSpeed/World/Simulation/CanonicalFrameContext.h"
#include "IAmSpeed/World/Simulation/RealTimeSimulation.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "ChaosVehicleWheel.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include "HAL/PlatformProcess.h"
#include "SimpleVehicle.h"
#include "SuspensionSystem.h"
#include "WheelSystem.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedWheelSimulationAdmissionTest,
	"IAmSpeed.Vehicle.WheelSimulationAdmission",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedWheelSimulationAdmissionTest::RunTest(const FString& Parameters)
{
	const auto Options = UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(false)
		.RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false)
		.ShouldSimulatePhysics(false).SetTransactional(false);
	UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, NAME_None, nullptr, true,
		ERHIFeatureLevel::Num, &Options);
	if (!TestNotNull(TEXT("admission fixture world"), World)) return false;
	GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
	ON_SCOPE_EXIT { World->DestroyWorld(false); GEngine->DestroyWorldContext(World); };
	ASpeedCar* Car = World->SpawnActor<ASpeedCar>();
	ARealTimeSimulation* Driver = World->SpawnActor<ARealTimeSimulation>();
	if (!TestNotNull(TEXT("car"), Car) || !TestNotNull(TEXT("driver"), Driver)) return false;
	USpeedWheeledComponent* Component = Cast<USpeedWheeledComponent>(Car->GetVehicleMovement());
	USpeedWorldSubsystem* Bridge = World->GetSubsystem<USpeedWorldSubsystem>();
	if (!TestNotNull(TEXT("vehicle component"), Component) || !TestNotNull(TEXT("bridge"), Bridge)) return false;
	Driver->SpeedWorldSubsystem = Bridge;
	Component->SetOwner(Car);
	ON_SCOPE_EXIT { Bridge->UnregisterSpeedComponent(Component); Bridge->ApplyPendingOps(); };
	TestEqual(TEXT("four production subbodies"), Component->WheelSubBodies.Num(), 4);

	const auto ExpectRejected = [&](const TCHAR* Label)
	{
		FString Reason;
		TestFalse(Label, Component->ValidateSimulationBindings(Reason));
		TestTrue(TEXT("reason identifies component and index"),
			Reason.Contains(TEXT("component=")) && Reason.Contains(TEXT("ordinal=")) && Reason.Contains(TEXT("idx=")));
		const uint32 BeforeComponentFrame = Component->NumFrame();
		AddExpectedMessagePlain(TEXT("[SimulationBindingRejected]"), ELogVerbosity::Error);
		// RunCanonicalFrames invokes the real StepCanonicalFrame. A rejected step
		// must leave every counter and publication untouched, even on later wheels.
		TestFalse(TEXT("canonical execution fails before preparation"), Driver->RunCanonicalFrames(1));
		TestEqual(TEXT("component preparation did not run"), Component->NumFrame(), BeforeComponentFrame);
		TestEqual(TEXT("driver did not advance"), Driver->CanonicalNumFrame, uint64(0));
		TestEqual(TEXT("no hashes committed"), Driver->GetFrameHashes().Num(), 0);
		FSimulationSnapshot Snapshot;
		TestFalse(TEXT("no partial snapshot published"), Driver->ReadLatestSimulationSnapshot(Snapshot));
	};
	ExpectRejected(TEXT("uncreated physics state reproduces v16 safely"));
	AddExpectedMessagePlain(TEXT("[SimulationBindingRejected]"), ELogVerbosity::Error);
	TAtomic<uint32> WorkerCalls{0};
	TAtomic<ESimulationWorkerResult> WorkerResult{ESimulationWorkerResult::Idle};
	FSimulationWorker Worker([&]
	{
		WorkerCalls.FetchAdd(1);
		const auto Result = Driver->RunCanonicalFrames(1)
			? ESimulationWorkerResult::Advanced : ESimulationWorkerResult::Failed;
		WorkerResult.Store(Result);
		return Result;
	}, [](FSimulationWorkerWaitContext&) {});
	if (!TestTrue(TEXT("negative fixture worker starts"), Worker.Start())) return false;
	const double Deadline = FPlatformTime::Seconds() + 2.0;
	while (Worker.IsRunning() && FPlatformTime::Seconds() < Deadline) FPlatformProcess::Sleep(0.001f);
	const bool bStoppedOnFailure = !Worker.IsRunning();
	Worker.StopAndJoin();
	TestTrue(TEXT("binding failure stops worker without retrying"), bStoppedOnFailure);
	TestEqual(TEXT("worker reports Failed"), WorkerResult.Load(), ESimulationWorkerResult::Failed);
	TestEqual(TEXT("worker attempted exactly one frame"), WorkerCalls.Load(), uint32(1));
	TestEqual(TEXT("failed worker never advanced canonical time"), Driver->CanonicalNumFrame, uint64(0));

	// Stable test-owned Chaos storage; no mesh/scene/Blueprint dependency. Native
	// full Freeplay startup with the actual asset closure is a separate smoke gate.
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
		EAutomationExpectedMessageFlags::Contains, 18);
	Component->BindWheelSimulationPointers();
	const auto ExpectValid = [&]
	{
		FString Reason;
		TestTrue(TEXT("all four complete storage bindings admitted"), Component->ValidateSimulationBindings(Reason));
	};
	ExpectValid();
	for (int32 Index = 0; Index < 4; ++Index)
	{
		USWheelSubBody* Wheel = Component->WheelSubBodies[Index];
		Wheel->SetIdx(INDEX_NONE); ExpectRejected(TEXT("negative wheel index")); Wheel->SetIdx(Index);
		Wheel->SetIdx(4); ExpectRejected(TEXT("out of snapshot capacity")); Wheel->SetIdx(Index);
		Wheel->SetWheelSim(nullptr); ExpectRejected(TEXT("null wheel simulator")); Component->BindWheelSimulationPointers();
		Wheel->SetSuspensionSim(nullptr); ExpectRejected(TEXT("null suspension simulator")); Component->BindWheelSimulationPointers();
		Wheel->SetSuspensionSim(&Component->SkySimulation->PVehicle->Suspension[(Index + 1) % 4]);
		ExpectRejected(TEXT("non-null crossed suspension simulator")); Component->BindWheelSimulationPointers();
		Wheel->SetWheelSim(&Component->SkySimulation->PVehicle->Wheels[(Index + 1) % 4]);
		ExpectRejected(TEXT("non-null crossed wheel simulator")); Component->BindWheelSimulationPointers();
		Component->WheelSubBodies[Index] = nullptr;
		ExpectRejected(TEXT("null subbody")); Component->WheelSubBodies[Index] = Wheel;
	}
	Component->ClearWheelSimulationPointers();
	ExpectRejected(TEXT("physics teardown invalidates admission"));
	Component->BindWheelSimulationPointers();
	ExpectValid();
	// A late invalid registration cannot bypass admission through the redundant
	// registry flushes in Prepare/Step. It becomes visible only at the next frame.
	FString AdmissionReason;
	TestTrue(TEXT("healthy registry admitted"), Bridge->BeginCanonicalFrame(AdmissionReason));
	USpeedWheeledComponent* Late = NewObject<USpeedWheeledComponent>(Car);
	Bridge->RegisterSpeedComponent(Late);
	Bridge->ApplyPendingOps();
	TestTrue(TEXT("queued registration deferred inside admitted frame"), Bridge->ValidateSimulationBindings(AdmissionReason));
	Bridge->EndCanonicalFrame();
	AddExpectedMessagePlain(TEXT("[SimulationBindingRejected]"), ELogVerbosity::Error);
	TestFalse(TEXT("next frame rejects the late incomplete vehicle"), Driver->RunCanonicalFrames(1));
	Bridge->UnregisterSpeedComponent(Late);
	Bridge->ApplyPendingOps();
	// Exercise the formerly crashing restore -> visual-state path for 600 frames.
	// This is not a substitute for >500 active physical Freeplay frames.
	for (int32 Frame = 0; Frame < 600; ++Frame)
	{
		ExpectValid();
		for (int32 Index = 0; Index < 4; ++Index)
			Component->WheeledPhysicsState.SuspensionLastDisplacement[Index] = float(Frame + Index) / 100.0f;
		Component->RecoverWheelState();
		for (int32 Index = 0; Index < 4; ++Index)
		{
			USWheelSubBody* Wheel = Component->WheelSubBodies[Index];
			Wheel->UpdatePhysicsState(1.0f / 300.0f);
			TestEqual(TEXT("restore reaches exactly the indexed suspension"), Wheel->GetLastDisplacement(),
				Component->WheeledPhysicsState.SuspensionLastDisplacement[Index]);
			TestTrue(TEXT("visual spring state finite"), FMath::IsFinite(Wheel->GetRenderData().SpringOffset));
		}
	}
	Component->ClearWheelSimulationPointers();
	return true;
}
#endif
