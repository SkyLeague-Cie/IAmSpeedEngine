#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Actors/SpeedCar.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/World/Simulation/RealTimeSimulation.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedAdapterRespawnBoundaryTest,
    "IAmSpeed.Simulation.AdapterRespawnBoundary",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedAdapterRespawnBoundaryTest::RunTest(const FString&)
{
    const auto Options = UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(false)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false)
        .ShouldSimulatePhysics(false).SetTransactional(false);
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, NAME_None, nullptr, true,
        ERHIFeatureLevel::Num, &Options);
    if (!TestNotNull(TEXT("respawn fixture world"), World)) return false;
    GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
    ON_SCOPE_EXIT { World->DestroyWorld(false); GEngine->DestroyWorldContext(World); };
    // RouteEndPlay requires initialized actors as well as dispatched BeginPlay.
    World->InitializeActorsForPlay(FURL());
    if (!TestTrue(TEXT("fixture world actors initialized"), World->AreActorsInitialized())) return false;
    auto* Driver = World->SpawnActor<ARealTimeSimulation>();
    auto* Bridge = World->GetSubsystem<USpeedWorldSubsystem>();
    auto* First = World->SpawnActor<ASpeedCar>();
    if (!TestNotNull(TEXT("driver"), Driver) || !TestNotNull(TEXT("bridge"), Bridge)
        || !TestNotNull(TEXT("first car"), First)) return false;
    Driver->SpeedWorldSubsystem = Bridge;
    Driver->ActiveExecutionModeValue.Store(static_cast<uint8>(ESimulationExecutionMode::IAmSpeedThread));
    auto* FirstComponent = Cast<USpeedWheeledComponent>(First->GetVehicleMovement());
    if (!TestNotNull(TEXT("first wheeled adapter"), FirstComponent)) return false;
    if (!TestTrue(TEXT("first actor initialized before BeginPlay"), First->IsActorInitialized())) return false;
    FirstComponent->SetOwner(First);
    First->DispatchBeginPlay();
    if (!TestTrue(TEXT("first actor began play before teardown"), First->HasActorBegunPlay())
        || !TestTrue(TEXT("first adapter registered before teardown"), FirstComponent->IsRegistered())
        || !TestTrue(TEXT("first adapter began play before teardown"), FirstComponent->HasBegunPlay())) return false;
    // The V2 authority flag follows the real controller path even when the
    // registry view is empty after its old Bind has been detached.
    if (!TestTrue(TEXT("first car claims neutral V2 authority"), First->SetFrameInputStreamV2(nullptr))) return false;
    std::atomic_store(&Driver->PublishedInputRegistry,
        std::make_shared<const Speed::Input::V2::FInputRegistryView>());
    TAtomic<int32> BoundaryServices = 0;
    std::atomic<uint64> SurvivorSteps{0};
    Driver->SimulationWorker = MakeUnique<FSimulationWorker>(
        [&]() { ++SurvivorSteps; return ESimulationWorkerResult::Advanced; },
        [](FSimulationWorkerWaitContext& Wait) { Wait.WaitUntil(FPlatformTime::Seconds()+.001); },
        [&](bool)
        {
            ++BoundaryServices;
            return Bridge->ServiceInputRetirementsAtBoundary()
                ? ESimulationBoundaryResult::Ready : ESimulationBoundaryResult::Failed;
        });
    ON_SCOPE_EXIT { Driver->StopOwnedWorker(); };
    if (!TestTrue(TEXT("paused lifecycle worker starts"), Driver->SimulationWorker->Start(true))) return false;
    const uint64 FirstId = Bridge->GetSimulationStableId(*FirstComponent);
    if (!TestTrue(TEXT("first adapter admitted"), FirstId != 0)) return false;
    const int32 Before = BoundaryServices.Load();
    FPlatformProcess::SleepNoStats(.01f);
    TestTrue(TEXT("ordinary paused worker keeps servicing boundary"), BoundaryServices.Load() > Before);

    // EndPlay must detach exactly once and receive removal ACK before any
    // sub-body storage can be released. Base EndPlay must not enqueue again.
    Driver->ResumeOwnedSimulation();
    const double RunningDeadline=FPlatformTime::Seconds()+2.0;
    while (SurvivorSteps.load()==0 && FPlatformTime::Seconds()<RunningDeadline) FPlatformProcess::SleepNoStats(.001f);
    if (!TestTrue(TEXT("shared worker really advances before pawn destruction"), SurvivorSteps.load()>0)) return false;
    if (!TestTrue(TEXT("real pawn destruction succeeds"), First->Destroy())) return false;
    if (!TestFalse(TEXT("real Destroy routed actor EndPlay"), First->HasActorBegunPlay())
        || !TestFalse(TEXT("real Destroy routed adapter EndPlay"), FirstComponent->HasBegunPlay())) return false;
    const uint64 AfterDestruction=SurvivorSteps.load();
    TestFalse(TEXT("produced adapter destruction restores prior running state"), Driver->IsOwnedSimulationPaused());
    const double SurvivorDeadline=FPlatformTime::Seconds()+2.0;
    while (SurvivorSteps.load()<=AfterDestruction && FPlatformTime::Seconds()<SurvivorDeadline) FPlatformProcess::SleepNoStats(.001f);
    TestTrue(TEXT("survivor scheduling continues after real EndPlay"), SurvivorSteps.load()>AfterDestruction);
    TestEqual(TEXT("old adapter removed before teardown"), Bridge->GetSimulationStableId(*FirstComponent), uint64(0));

    auto* Replacement = World->SpawnActor<ASpeedCar>();
    auto* ReplacementComponent = Replacement
        ? Cast<USpeedWheeledComponent>(Replacement->GetVehicleMovement()) : nullptr;
    if (!TestNotNull(TEXT("replacement wheeled adapter"), ReplacementComponent)) return false;
    if (!TestTrue(TEXT("replacement actor initialized"), Replacement->IsActorInitialized())) return false;
    ReplacementComponent->SetOwner(Replacement);
    const uint64 ReplacementId = Bridge->GetSimulationStableId(*ReplacementComponent);
    TestTrue(TEXT("replacement admitted with distinct stable identity"),
        ReplacementId != 0 && ReplacementId != FirstId);
    TestTrue(TEXT("shared worker still alive after replacement"),
        Driver->SimulationWorker && Driver->SimulationWorker->IsRunning());
    Driver->TryPauseOwnedSimulation();
    if (!TestTrue(TEXT("replacement claims produced authority"), Replacement->SetFrameInputStreamV2(nullptr))) return false;
    Replacement->DispatchBeginPlay();
    if (!TestTrue(TEXT("paused replacement destruction succeeds"), Replacement->Destroy())) return false;
    if (!TestFalse(TEXT("paused replacement routes actor EndPlay"), Replacement->HasActorBegunPlay())
        || !TestFalse(TEXT("paused replacement routes adapter EndPlay"), ReplacementComponent->HasBegunPlay())) return false;
    TestEqual(TEXT("paused replacement adapter removed before teardown"), Bridge->GetSimulationStableId(*ReplacementComponent), uint64(0));
    TestTrue(TEXT("preexisting world pause remains paused after destruction"), Driver->IsOwnedSimulationPaused());
    TestTrue(TEXT("paused destruction does not retire shared worker"), Driver->SimulationWorker && Driver->SimulationWorker->IsRunning());
    Driver->StopOwnedWorker();
    return true;
}
#endif
