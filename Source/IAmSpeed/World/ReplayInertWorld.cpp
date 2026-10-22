#include "IAmSpeed/World/ReplayInertWorld.h"
#include "IAmSpeed/World/ReplayWorldIdentity.h"
#include "IAmSpeed/Components/SpeedMovementComponent.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "Engine/World.h"
#include "Engine/GameInstance.h"
#include "Engine/Level.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Pawn.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/ChildActorComponent.h"
#include "UObject/UnrealType.h"
#include "GameFramework/GameModeBase.h"
#include "Components/ActorComponent.h"
#include "Components/PrimitiveComponent.h"
#include "HAL/PlatformProperties.h"
#include "Misc/PackageName.h"
#include "UObject/ObjectKey.h"
namespace Speed::ReplayIsolation
{
namespace
{
struct Scope
{
    TWeakObjectPtr<UGameInstance> Owner;TWeakObjectPtr<UWorld> Source;TWeakObjectPtr<UWorld> World;
    FObjectKey SourceKey,WorldKey;FName Package;uint64 Serial=0;FString SourceIdentity,ModePath,Failure;
    EWorldType::Type ExpectedWorldType=EWorldType::None;
    int32 ExpectedPIEInstance=INDEX_NONE;
    bool Bound=false,Retired=false,Fenced=false,Ready=false;
    TArray<TWeakObjectPtr<AActor>> Originals;
    TSet<FObjectKey> Components;
};
TArray<TSharedPtr<Scope>> Scopes;
bool Installed=false;
Scope* Find(const UWorld* World)
{
    if(!World)return nullptr;
    for(const auto& S:Scopes)
    {
        if(S->Bound){if(S->WorldKey==FObjectKey(World))return S.Get();continue;}
        if(!S->Retired&&S->Owner.IsValid()&&S->Owner.Get()==World->GetGameInstance()&&S->SourceKey!=FObjectKey(World)&&
           CanonicalReplayWorldPackage(*World)==S->Package)return S.Get();
    }
    return nullptr;
}
bool Failure(Scope& S,UWorld& World,const FString& Why)
{
    // Latch the failed world too: retiring the request must never re-enable its
    // gameplay subsystems or their shared-asset Deinitialize paths.
    if(!S.Bound){S.World=&World;S.WorldKey=FObjectKey(&World);S.Bound=true;}
    S.Ready=false;if(S.Failure.IsEmpty())S.Failure=Why.IsEmpty()?TEXT("inert replay bootstrap refused"):Why;
    return false;
}
bool PhysicalBody(const AActor& Actor)
{
    for(UActorComponent* C:Actor.GetComponents())if(IsValid(C)&&(C->IsA<USpeedMovementComponent>()||C->IsA<USpeedWheeledComponent>()))return true;
    return false;
}
bool HasClassInChain(const UActorComponent& C,const TCHAR* Path)
{for(UClass* K=C.GetClass();K;K=K->GetSuperClass())if(K->GetPathName()==Path)return true;return false;}
bool Gather(UWorld& World,TArray<AActor*>& Actors,FString& Why)
{
    if(!World.PersistentLevel){Why=TEXT("inert replay missing persistent level");return false;}
    TArray<ULevel*> Levels=World.GetLevels();Levels.AddUnique(World.PersistentLevel);
    for(auto* Level:Levels)
    {
        if(!IsValid(Level)){Why=TEXT("inert replay invalid loaded level");return false;}
        for(AActor* Actor:Level->Actors)if(IsValid(Actor)){Actors.AddUnique(Actor);if(Actors.Num()>50000){Why=TEXT("inert replay actor bound exceeded");return false;}}
    }
    return true;
}
bool Topology(const Scope& S,UWorld& World,FString& Why)
{
    ReplayNativeWorldIdentity Identity;
    if(!ReadReplayNativeWorldIdentity(World,Identity)||Identity.Type!=S.ExpectedWorldType||
       Identity.PIEInstance!=S.ExpectedPIEInstance||Identity.Package!=S.Package||World.GetNetDriver())
    {Why=TEXT("inert replay requires the exact standalone Game/PIE world context and instance");return false;}
    // Only an Engine-produced PIE duplicate is admitted. A renamed disk load,
    // world-type change or assigned construction flag cannot establish this.
    if(Identity.Type==EWorldType::PIE&&(!World.PersistentLevel||!World.PersistentLevel->bWasDuplicatedForPIE))
    {Why=TEXT("inert replay PIE destination was not genuinely duplicated by Engine");return false;}
    if(!World.PersistentLevel||World.GetWorldPartition()||!World.GetStreamingLevels().IsEmpty())
    {Why=TEXT("inert replay does not admit partitioned or streaming maps");return false;}
    // Match the real Engine InitializeActorsForPlay predicate; do NOT set either
    // construction flag or change package/cook metadata to make this pass.
    const bool Rerun=!(FPlatformProperties::RequiresCookedData()||
        (World.IsGameWorld()&&(World.PersistentLevel->bHasRerunConstructionScripts||World.PersistentLevel->bWasDuplicatedForPIE)));
    if(Rerun){Why=TEXT("inert replay refuses a map that will rerun construction scripts");return false;}
    return true;
}
void BeforeWorldInitialization(UWorld* World,const UWorld::InitializationValues)
{
    check(IsInGameThread());if(!World)return;Scope* S=Find(World);if(!S)return;
    if(S->Bound||S->Retired){Failure(*S,*World,TEXT("inert replay world initialization repeated/retired"));return;}
    S->World=World;S->WorldKey=FObjectKey(World);S->Bound=true;
    FString Why;if(!Topology(*S,*World,Why)){Failure(*S,*World,Why);return;}
    TArray<AActor*> Actors;if(!Gather(*World,Actors,Why)){Failure(*S,*World,Why);return;}
    uint64 Count=0;
    // Validate the ENTIRE loaded component set before the first mutation. No
    // unregister/destroy-physics/stop-solver repair is performed on refusal.
    for(auto* Actor:Actors)
    {
        if(Actor->HasActorBegunPlay()){Failure(*S,*World,TEXT("inert replay actor already began play"));return;}
        const bool IsBody=PhysicalBody(*Actor);
        if(auto* Pawn=Cast<APawn>(Actor))if(Pawn->GetController())
        {Failure(*S,*World,TEXT("inert replay authored pawn already possessed"));return;}
        for(UActorComponent* C:Actor->GetComponents())
        {
            if(IsValid(C)&&(C->IsA<UChildActorComponent>()||HasClassInChain(*C,TEXT("/Script/UMG.WidgetComponent"))||(!IsBody&&C->IsA<USkeletalMeshComponent>())))
            {Failure(*S,*World,TEXT("inert replay map contains unadmitted child-actor/widget/non-body animated component"));return;}
            if(!IsValid(C)||++Count>200000||C->IsRegistered()||C->IsPreRegistered()||C->IsPreRegistering()||C->IsPhysicsStateCreated()||C->IsActive())
            {Failure(*S,*World,TEXT("inert replay component already registered/preregistered/physical or invalid"));return;}
        }
    }
#if WITH_EDITOR
    World->bEnableTraceCollision=false;
#endif
    for(auto* Actor:Actors)
    {
        Actor->SetActorEnableCollision(false);Actor->PrimaryActorTick.bStartWithTickEnabled=false;Actor->SetActorTickEnabled(false);
        const bool Body=PhysicalBody(*Actor);
        Actor->AutoReceiveInput=EAutoReceiveInput::Disabled;
        if(auto* Pawn=Cast<APawn>(Actor)){Pawn->AutoPossessPlayer=EAutoReceiveInput::Disabled;Pawn->AutoPossessAI=EAutoPossessAI::Disabled;}
        for(UActorComponent* C:Actor->GetComponents())
        {
            S->Components.Add(FObjectKey(C));
            C->bAutoActivate=false;C->bWantsInitializeComponent=false;
            C->PrimaryComponentTick.bStartWithTickEnabled=false;C->SetComponentTickEnabled(false);
            // Original physical bodies are replaced by the inert native scene.
            // They need no component registration: this prevents original
            // skeletal InitAnim/PostProcessBP/Cloth and component callbacks.
            if(Body)C->bAutoRegister=false;
            if(auto* Primitive=Cast<UPrimitiveComponent>(C))
            {
                Primitive->bAlwaysCreatePhysicsState=false;
                Primitive->SetCollisionEnabled(ECollisionEnabled::NoCollision);
                Primitive->SetGenerateOverlapEvents(false);Primitive->SetSimulatePhysics(false);
            }
        }
        if(Body)Actor->SetActorHiddenInGame(true);
        S->Originals.Add(Actor);
    }
    S->Fenced=true;
}
bool Audit(Scope& S,UWorld& World,AGameModeBase& Mode,FString& Why)
{
    if(!S.Failure.IsEmpty()){Why=S.Failure;return false;}
    if(!S.Bound||!S.Fenced||S.Retired||S.World.Get()!=&World||S.Owner.Get()!=World.GetGameInstance()||
       CanonicalReplayWorldPackage(World)!=S.Package||World.GetAuthGameMode()!=&Mode||Mode.GetClass()->GetPathName()!=S.ModePath)
    {Why=TEXT("inert replay world/mode/request fence mismatch or retired");return Failure(S,World,Why);}
    if(!Topology(S,World,Why))return Failure(S,World,Why);
#if WITH_EDITOR
    if(World.bEnableTraceCollision){Why=TEXT("inert replay editor trace collision re-enabled");return Failure(S,World,Why);}
#endif
    uint64 SeenComponents=0;
    for(const auto& Weak:S.Originals)
    {
        auto* A=Weak.Get();if(!A||A->HasActorBegunPlay()||A->GetActorEnableCollision()||A->IsActorTickEnabled())
        {Why=TEXT("inert replay original actor lifecycle/isolation changed");return Failure(S,World,Why);}
        for(UActorComponent* C:A->GetComponents())
        {
            ++SeenComponents;
            if(!IsValid(C)||!S.Components.Contains(FObjectKey(C))||C->IsPhysicsStateCreated()||C->IsComponentTickEnabled()||C->IsActive()||C->bAutoActivate||C->bWantsInitializeComponent)
            {Why=TEXT("inert replay original component set/physics/tick changed");return Failure(S,World,Why);}
            if(auto* Primitive=Cast<UPrimitiveComponent>(C))
                if(Primitive->bAlwaysCreatePhysicsState||Primitive->GetCollisionEnabled()!=ECollisionEnabled::NoCollision||Primitive->IsSimulatingPhysics())
                {Why=TEXT("inert replay original primitive collision changed");return Failure(S,World,Why);}
        }
    }
    if(SeenComponents!=uint64(S.Components.Num()))
    {Why=TEXT("inert replay original component removed or duplicated");return Failure(S,World,Why);}
    // Include native actors created after the initial fence (mode/controller/
    // camera/driver) in the no-physics invariant; never repair existing state.
    TArray<AActor*> All;if(!Gather(World,All,Why))return Failure(S,World,Why);
    for(auto* A:All)for(UActorComponent* C:A->GetComponents())if(C&&C->IsPhysicsStateCreated())
    {Why=TEXT("inert replay later component has a physics state");return Failure(S,World,Why);}
    return true;
}
}
bool ArmInertReplayDestination(UGameInstance& Owner,UWorld& SourceWorld,FName Package,uint64 Serial,
    const FString& SourceIdentity,const FString& ModePath,FString& Why)
{
    check(IsInGameThread());Why.Empty();
    if(Serial==0||Package.IsNone()||!FPackageName::IsValidLongPackageName(Package.ToString())||SourceIdentity.IsEmpty()||SourceIdentity.Len()>2048||ModePath.IsEmpty()||
       Owner.GetWorld()!=&SourceWorld||SourceWorld.GetGameInstance()!=&Owner||SourceWorld.GetNetMode()!=NM_Standalone)
    {Why=TEXT("inert replay pending destination authority invalid");return false;}
    ReplayNativeWorldIdentity SourceWorldIdentity;
    if(!ReadReplayNativeWorldIdentity(SourceWorld,SourceWorldIdentity))
    {Why=TEXT("inert replay source world context/PIE instance is not exact");return false;}
    // Keep bound weak markers through subsystem Deinitialize and actual world
    // destruction. Never clear them merely because source/session closes.
    Scopes.RemoveAll([](const TSharedPtr<Scope>& S)
    {
        // Threadsafe stale test means index/serial disappearance, not merely
        // PendingKill/Unreachable. Keep bound keys throughout Deinitialize.
        return S->Bound?S->World.IsStale(false,true):(S->Retired||S->Owner.IsStale(false,true));
    });
    for(const auto& S:Scopes)if(S->Owner.Get()==&Owner&&(!S->Retired||S->Serial==Serial))
    {Why=TEXT("inert replay destination already armed or serial reused");return false;}
    if(Scopes.Num()>=128){Why=TEXT("inert replay retained world-scope bound exceeded");return false;}
    auto S=MakeShared<Scope>();S->Owner=&Owner;S->Source=&SourceWorld;S->SourceKey=FObjectKey(&SourceWorld);S->Package=Package;S->ExpectedWorldType=SourceWorldIdentity.Type;S->ExpectedPIEInstance=SourceWorldIdentity.PIEInstance;S->Serial=Serial;S->SourceIdentity=SourceIdentity;S->ModePath=ModePath;Scopes.Add(S);
    if(!Installed){FWorldDelegates::OnPreWorldInitialization.AddStatic(&BeforeWorldInitialization);Installed=true;}
    return true;
}
void RetireInertReplayDestination(UGameInstance& Owner,uint64 Serial)
{
    check(IsInGameThread());for(const auto& S:Scopes)if(S->Owner.Get()==&Owner&&S->Serial==Serial){S->Retired=true;S->Ready=false;}
}
bool IsInertReplayWorld(const UWorld* World){check(IsInGameThread());return Find(World)!=nullptr;}
bool IsInertBootstrapReady(const UWorld* World)
{
    check(IsInGameThread());Scope* S=Find(World);return S&&S->Bound&&S->Fenced&&S->Ready&&!S->Retired&&S->Failure.IsEmpty();
}
bool HasInertBootstrapFailed(const UWorld* World,FString& Why)
{
    check(IsInGameThread());Why.Empty();Scope* S=Find(World);if(!S||S->Failure.IsEmpty())return false;Why=S->Failure;return true;
}
void FailInertReplayBootstrap(UWorld& World,const FString& Why){check(IsInGameThread());if(auto* S=Find(&World))Failure(*S,World,Why);}
bool ValidateInertReplayBootstrap(UWorld& World,AGameModeBase& Mode,FString& Why)
{
    check(IsInGameThread());Why.Empty();Scope* S=Find(&World);if(!S){Why=TEXT("inert replay destination was not armed");return false;}return Audit(*S,World,Mode,Why);
}
bool CompleteInertReplayBootstrap(UWorld& World,AGameModeBase& Mode,FString& Why)
{
    check(IsInGameThread());if(!ValidateInertReplayBootstrap(World,Mode,Why))return false;
    Scope* S=Find(&World);if(!World.GetBegunPlay()||!Mode.HasActorBegunPlay())
    {Why=TEXT("inert replay native lifecycle not completed");return Failure(*S,World,Why);}
    S->Ready=true;return true;
}
}
