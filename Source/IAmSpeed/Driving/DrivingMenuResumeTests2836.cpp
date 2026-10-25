#if WITH_DEV_AUTOMATION_TESTS
#include "SpeedDrivingController.h"
#include "IAmSpeed/World/Simulation/SpeedGameMode.h"
#include "Engine/Engine.h"
#include "Engine/GameInstance.h"
#include "Engine/World.h"
#include "GameFramework/PlayerState.h"
#include "GameFramework/WorldSettings.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"

// Real controller/menu and Unreal pause hosting, without a device or worker.
// Existing V2 worker/baseline tests remain required independently.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDrivingMenuResume2836,"IAmSpeed.Driving.MenuResume2836",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FDrivingMenuResume2836::RunTest(const FString&)
{
    const auto Options=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(false)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false)
        .ShouldSimulatePhysics(false).SetTransactional(false);
    UWorld* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Options);
    if(!TestNotNull(TEXT("menu fixture world"),World))return false;
    GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
    ON_SCOPE_EXIT {World->DestroyWorld(false);GEngine->DestroyWorldContext(World);};
    World->SetGameInstance(NewObject<UGameInstance>(GEngine));
    World->GetWorldSettings()->DefaultGameMode=ASpeedGameMode::StaticClass();
    if(!TestTrue(TEXT("real standalone pause host"),World->SetGameMode(FURL())))return false;
    auto* Controller=World->SpawnActor<ASpeedDrivingController>();
    if(!TestNotNull(TEXT("actual driving controller"),Controller))return false;
    Controller->SetPlayerState(World->SpawnActor<APlayerState>());
    if(!TestNotNull(TEXT("real pause owner"),Controller->PlayerState.Get()))return false;
    Controller->Pause();
    if(!TestTrue(TEXT("pause shows menu and pauses Unreal"),Controller->IsPaused()&&Controller->bMenuVisible))return false;
    Controller->Navigate(Speed::Driving::EMenuAction::Accept);
    TestFalse(TEXT("Resume selection unpauses Unreal"),Controller->IsPaused());
    TestFalse(TEXT("Resume selection also closes the pause menu"),Controller->bMenuVisible);
    Controller->Pause();
    Controller->Navigate(Speed::Driving::EMenuAction::Back);
    TestFalse(TEXT("Back from root pause page unpauses Unreal"),Controller->IsPaused());
    TestFalse(TEXT("Back also closes the pause menu"),Controller->bMenuVisible);
    Controller->Pause();
    const FName FirstClick=Controller->MenuHitBoxName(0);
    TestTrue(TEXT("pointer is available while the menu owns it"),Controller->bShowMouseCursor&&Controller->bEnableClickEvents);
    TestTrue(TEXT("mouse hitbox reaches the same accepted Resume policy"),Controller->HandleDrivingMenuHitBoxClick(FirstClick));
    TestFalse(TEXT("mouse Resume unpauses Unreal and closes the menu"),Controller->IsPaused()||Controller->bMenuVisible);
    TestFalse(TEXT("menu pointer ownership is released"),Controller->bOwnsMenuPointer);
    TestFalse(TEXT("closed-menu clicks are rejected"),Controller->HandleDrivingMenuHitBoxClick(FirstClick));
    Controller->Pause();
    TestFalse(TEXT("a hitbox from the old menu context cannot act on the new menu"),Controller->HandleDrivingMenuHitBoxClick(FirstClick));
    Controller->Navigate(Speed::Driving::EMenuAction::Accept);
    Controller->Pause();Controller->bRemapPending=true;
    Controller->Navigate(Speed::Driving::EMenuAction::Accept);
    TestTrue(TEXT("pending remap still blocks Resume"),Controller->IsPaused()&&Controller->bMenuVisible);
    Controller->bRemapPending=false;
    Controller->Navigate(Speed::Driving::EMenuAction::Accept);
    UWorld* NoHost=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Options);
    if(!TestNotNull(TEXT("refusal fixture world"),NoHost))return false;
    GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(NoHost);
    ON_SCOPE_EXIT {NoHost->DestroyWorld(false);GEngine->DestroyWorldContext(NoHost);};
    auto* Rejected=NoHost->SpawnActor<ASpeedDrivingController>();
    if(!TestNotNull(TEXT("controller with no pause host"),Rejected))return false;
    Rejected->OnPauseStateChanged(true);
    Rejected->Navigate(Speed::Driving::EMenuAction::Accept);
    TestTrue(TEXT("a refused Resume does not hide the menu"),Rejected->bMenuVisible);
    TestFalse(TEXT("refusal is visible to the player"),Rejected->Message.IsEmpty());
    return !HasAnyErrors();
}
#endif
