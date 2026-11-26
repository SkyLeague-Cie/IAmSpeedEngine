#if WITH_DEV_AUTOMATION_TESTS
#include "PresentationClientWorld.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "Physics/NetworkPhysicsComponent.h"
#include "UObject/UObjectIterator.h"
#include "GameFramework/Actor.h"
#include "IAmSpeed/Actors/SpeedCar.h"
#include "IAmSpeed/Components/SpeedMovementComponent.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "IAmSpeed/World/Simulation/RealTimeSimulation.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedPresentationClientPendingWorldTest,
 "IAmSpeed.Netcode.PresentationClient.PendingWorldBlocksPhysicalBootstrap",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedPresentationClientPendingWorldTest::RunTest(const FString&)
{
 namespace P=Speed::Netcode::PresentationClient;
 if(!TestTrue(TEXT("explicit presentation process profile required; no default-profile PASS"),P::IsSelected()))return false;
 const auto Options=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(true)
  .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false).SetTransactional(false);
 auto* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Options);
 if(!TestNotNull(TEXT("real pending game world"),World))return false;
 GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
 ON_SCOPE_EXIT{World->DestroyWorld(false);GEngine->DestroyWorldContext(World);};
 uint64 Generation=0;FString Why;
 if(!TestTrue(TEXT("not a fabricated connected client; actual pre-connection world is Pending"),P::Query(*World,Generation,Why)==P::EState::Pending&&Generation!=0))return false;
 auto* Actor=World->SpawnActor<AActor>();auto* Movement=NewObject<USpeedMovementComponent>(Actor);
 Actor->AddInstanceComponent(Movement);Movement->RegisterComponent();
 auto* Car=World->SpawnActor<ASpeedCar>();auto* Wheeled=Car?Cast<USpeedWheeledComponent>(Car->GetVehicleMovement()):nullptr;
 auto* Driver=World->SpawnActor<ARealTimeSimulation>();auto* Bridge=World->GetSubsystem<USpeedWorldSubsystem>();
 if(!TestNotNull(TEXT("placed-style car"),Wheeled)||!TestNotNull(TEXT("driver fixture"),Driver)||!TestNotNull(TEXT("world registry"),Bridge))return false;
 TestFalse(TEXT("movement physics-state excluded before history creation"),Movement->IsPhysicsStateCreated());
 TestFalse(TEXT("wheeled physics-state excluded before Chaos vehicle/history creation"),Wheeled->IsPhysicsStateCreated());
 Wheeled->SetOwner(Car); // Movement registration/late registration tested without private API access.
 Bridge->RegisterSpeedComponent(Movement);Bridge->RegisterSpeedComponent(Wheeled);Bridge->ApplyPendingOps();
 TestFalse(TEXT("no pending or active physical registrations"),Bridge->HasPhysicalRegistrationsForPresentationAdmission());
 TestFalse(TEXT("world readiness cannot admit a physical owner"),Driver->EnsureSimulationWorldReady());
 Driver->StartOwnedWorkerIfReady();TestFalse(TEXT("no physical worker started"),Driver->HasOwnedWorkerForPresentationAdmission());
 for(TObjectIterator<UNetworkPhysicsComponent> It;It;++It)
  if(!It->IsTemplate()&&It->GetWorld()==World)
  {
   AddInfo(FString::Printf(TEXT("[PendingHistoryCensus] %s class=%s registered=%d state=%d input=%d"),*It->GetPathName(),*It->GetClass()->GetPathName(),It->IsRegistered()?1:0,It->GetStateHistory_External()?1:0,It->GetInputHistory_External()?1:0));
   TestFalse(TEXT("every original native network component remains unregistered"),It->IsRegistered());
   TestFalse(TEXT("no original native state history"),bool(It->GetStateHistory_External()));
   TestFalse(TEXT("no original native input history"),bool(It->GetInputHistory_External()));
  }
 World->BeginPlay(); // Real Engine path enforces each WorldSubsystem Super bookkeeping.
 AddInfo(TEXT("[PendingWorldBeginPlay] RealEngineWorldBeginPlayCompleted=1"));
 TestTrue(TEXT("actual pristine world audit, static query scene permitted"),P::Audit(*World,Why));
 // Repeat re-registration after the initial load: the same guards still apply.
 Movement->UnregisterComponent();Movement->RegisterComponent();
 TestFalse(TEXT("late re-registration cannot create physical state"),Movement->IsPhysicsStateCreated());
 TestTrue(TEXT("late audit still pristine"),P::Audit(*World,Why));
 AddInfo(TEXT("Native pending-world guards only; authentic NM_Client/connection, Blueprint and socket tests remain separate."));
 return !HasAnyErrors();
}
#endif
