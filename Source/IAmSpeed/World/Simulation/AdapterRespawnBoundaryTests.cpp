#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Actors/SpeedCar.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/World/Simulation/RealTimeSimulation.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "HAL/PlatformProcess.h"
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
    auto* Driver = World->SpawnActor<ARealTimeSimulation>();
    auto* Bridge = World->GetSubsystem<USpeedWorldSubsystem>();
    auto* First = World->SpawnActor<ASpeedCar>();
    if (!TestNotNull(TEXT("driver"), Driver) || !TestNotNull(TEXT("bridge"), Bridge)
        || !TestNotNull(TEXT("first car"), First)) return false;
    Driver->SpeedWorldSubsystem = Bridge;
    Driver->ActiveExecutionModeValue.Store(static_cast<uint8>(ESimulationExecutionMode::IAmSpeedThread));
    auto* FirstComponent = Cast<USpeedWheeledComponent>(First->GetVehicleMovement());
    if (!TestNotNull(TEXT("first wheeled adapter"), FirstComponent)) return false;
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
    Driver->SimulationWorker = MakeUnique<FSimulationWorker>(
        []() { return ESimulationWorkerResult::Idle; },
        [](FSimulationWorkerWaitContext&) {},
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
    FirstComponent->EndPlay(EEndPlayReason::Destroyed);
    TestEqual(TEXT("old adapter removed before teardown"), Bridge->GetSimulationStableId(*FirstComponent), uint64(0));

    auto* Replacement = World->SpawnActor<ASpeedCar>();
    auto* ReplacementComponent = Replacement
        ? Cast<USpeedWheeledComponent>(Replacement->GetVehicleMovement()) : nullptr;
    if (!TestNotNull(TEXT("replacement wheeled adapter"), ReplacementComponent)) return false;
    ReplacementComponent->SetOwner(Replacement);
    const uint64 ReplacementId = Bridge->GetSimulationStableId(*ReplacementComponent);
    TestTrue(TEXT("replacement admitted with distinct stable identity"),
        ReplacementId != 0 && ReplacementId != FirstId);
    TestTrue(TEXT("paused worker still alive after replacement"),
        Driver->SimulationWorker && Driver->SimulationWorker->IsRunning());
    Driver->StopOwnedWorker();
    return true;
}
#endif
