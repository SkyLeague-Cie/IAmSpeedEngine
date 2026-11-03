#include "PrivateRoadPhysicsWorld.h"
#include "IAmSpeed/World/Simulation/Internal/ComponentKinematicRead.h"
#include "IAmSpeed/World/Simulation/Internal/BodyKinematicRead.h"
#include "IAmSpeed/Base/SpeedConstant.h"
#include "PrivateRoadNativeObjects.h"
#include "PrivateRoadPartialStateObservation.h"
#include "OriginalRoadBodyValueCapture.h"
#include "OriginalRoadComponentValueCapture.h"
#include "IAmSpeed/Driving/StockRoadCar.h"
#include "IAmSpeed/Netcode/presentation-client/PresentationClientWorld.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "IAmSpeed/World/Simulation/CanonicalFrameContext.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "IAmSpeed/SubBodies/Configs/SubBodyConfig.h"
#include "Engine/World.h"
#include "Engine/Level.h"
#include "UObject/UnrealType.h"
#include "GameFramework/HUD.h"
#include "GameFramework/GameModeBase.h"
#include "Components/BoxComponent.h"
#include "Physics/NetworkPhysicsComponent.h"
#include "Chaos/Framework/PhysicsSolverBase.h"
#include "ChaosVehicleWheel.h"
#include "PhysicsEngine/PhysicsSettings.h"
#include "SimpleVehicle.h"
#include "SuspensionSystem.h"
#include "WheelSystem.h"
#include "UObject/StrongObjectPtr.h"
#include "UObject/UObjectHash.h"
#include "UObject/UObjectIterator.h"
#include "Misc/ScopeExit.h"
#include "UObject/GarbageCollection.h"
#include "IAmSpeed/World/Simulation/NativeEpochDiagnostics.h"
#include "NativeRetirementStorage.h"
#include "IAmSpeed/World/Simulation/RoadSettings/NativeRoadSettingsRead.h"
#include <algorithm>
#include <mutex>

namespace Speed::Netcode::Prediction
{
// No public handle exposes this staging area. It exists only during one GT
// call and cannot be used to install a Device producer, authority receipt or
// legacy snapshot. Native lifetime/restore certification remains separate.
struct FPrivateRoadAccess : Speed::NativeEpoch::IOriginalStorageRoute
{
    bool Retired=false;
    bool SelectsOriginalStorage() const noexcept override;
    struct FSlot {std::shared_ptr<const FBranchProcessedInput> Input;bool Consumed=false;U64 Actor=0,Incarnation=0;};
    UWorld* World=nullptr;
    TWeakObjectPtr<UWorld> OriginalWorldBirth;
    TWeakObjectPtr<ULevel> OriginalLevelBirth;
    TWeakObjectPtr<ASpeedBranchSimulation> OriginalDriverBirth;
    bool OriginalActorWorldStillMatches(const AActor* Actor) const;
    TWeakObjectPtr<USpeedWorldSubsystem> OriginalBridgeBirth;
    TWeakObjectPtr<UBoxComponent> OriginalFloorBirth;
    U64 OriginalStaticSourceId=0;
    TSharedPtr<const Speed::Analytic::FAnalyticWorldData> OriginalSourceGeometry;
    Speed::FImmutableStaticQueryIdentity1075 OriginalQueryIdentity; // actual backend observation, NOT lifetime authority
    using FSourceBindingsRead=bool(*)(const USpeedWorldSubsystem&,const Speed::Analytic::FAnalyticWorldData*,U64,const TWeakObjectPtr<UBoxComponent>&);
    FSourceBindingsRead MatchesOriginalSourceBindings=nullptr; // fixed native owner thunk, no captured view/delegate

    const TObjectPtr<AGameModeBase>* OriginalGameModeCell=nullptr;
    TWeakObjectPtr<UClass> OriginalWorldClassBirth;
    bool OriginalWorldHasNoGameMode() const;
    struct FActualCarAlias
    {
        TWeakObjectPtr<ASpeedBranchCar> Actor;
        TWeakObjectPtr<USpeedBranchWheeledComponent> Component;
        TWeakObjectPtr<UClass> ActorClass,MovementOwnerClass;
        const TObjectPtr<UChaosVehicleMovementComponent>* MovementCell=nullptr;
    };
    TMap<const USpeedBranchWheeledComponent*,FActualCarAlias> ActualCarAliases;
    USpeedBranchWheeledComponent* FindOriginalComponentAlias(const ISpeedComponent* Parent,const AActor* Owner) const;
    const USpeedWorldSubsystem* OriginalBridgeForFrame(UWorld* World,const Speed::IStaticCollisionWorld* StaticWorld) const;
    bool Active=false,RequiresInitialPublication=true,InitialPublicationReleased=false;
    U64 Frame=0;
    TMap<const USpeedBranchWheeledComponent*,FSlot> Slots;
    std::vector<FPrivateRoadContactDispatchPair> ContactDispatches;
    bool ContactDispatchOverflow=false;
};
namespace
{
// This routing scope can only be installed by this translation unit, under
// the original factory/step/startup gate. It is NOT a complete boundary lease:
// caches, subbodies, configurations and callback/lifetime closure remain work.
thread_local const FPrivateRoadAccess* OriginalComponentStorageOwner=nullptr;
struct FOriginalComponentStorageScope
{
    explicit FOriginalComponentStorageScope(const FPrivateRoadAccess& Owner)
        {OriginalComponentStorageOwner=&Owner;}
    ~FOriginalComponentStorageScope(){OriginalComponentStorageOwner=nullptr;}
    FOriginalComponentStorageScope(const FOriginalComponentStorageScope&)=delete;
    FOriginalComponentStorageScope& operator=(const FOriginalComponentStorageScope&)=delete;
};
bool Refuse(FString& Why,const TCHAR* Message) {Why=Message;return false;}
// This fixed native subtype consumes exactly the stock road semantic slots.
// Generic contract validity alone permits other physical destinations and wider
// extension axes. Refuse those recipes before allocating any native graph.
bool PrivateRoadSupportedContract(const BranchV2::FInputActionContract& Contract)
{
    const auto& D=Contract.GetDescription();
    if(D.Actions.size()!=7||D.Physical.size()!=3)return false;
    const char* Names[]={"Throttle","Brake","Steering","Pause","ResetWorld","CameraYaw","CameraPitch"};
    for(size_t I=0;I<7;++I)
    {
        const auto* A=Contract.Find(Speed::Input::FActionId(I));
        const bool Axis=I<3||I>=5,Signed=I==2||I>=5;
        const int16 Quantization=I<2?255:(I==2||I>=5?127:1);
        if(!A||A->Owner!="IAmSpeed"||A->Name!=Names[I]||A->Version!=1
            ||A->Wiring!=BranchV2::EActionWiring::Wired
            ||A->Type!=(Axis?BranchV2::EActionType::Axis1D:BranchV2::EActionType::Bool)
            ||A->Signed!=Signed||A->Quantization!=Quantization)return false;
    }
    for(const auto& B:D.Physical)
        if(B.Action!=static_cast<Speed::Input::FActionId>(B.Destination))return false;
    // Raw controls, mapping revision and scalar transforms may legitimately
    // differ between clients. They do not change these consumed semantics.
    return true;
}
bool Finite(const SKinematic& K)
{
    return !K.Location.ContainsNaN()&&!K.Velocity.ContainsNaN()&&!K.Acceleration.ContainsNaN()
        &&!K.AngularVelocity.ContainsNaN()&&!K.AngularAcceleration.ContainsNaN()
        &&K.Rotation.IsNormalized();
}
uint32 NativeNetworkComponentCount(UWorld* World)
{
    uint32 Count=0;
    for(TObjectIterator<UNetworkPhysicsComponent> It;It;++It)
        if(!It->IsTemplate()&&It->GetWorld()==World)++Count;
    return Count;
}
bool NoNetworkOwner(UWorld* World)
{return World&&!World->Scene&&!World->GetPhysicsScene()&&!World->bShouldSimulatePhysics
    &&Speed::NativeObjectRead::IsResolvedNull(World->NetDriver)&&NativeNetworkComponentCount(World)==0;}
}
bool IsPrivateRoadOriginalStorageRouteForDiagnostics() noexcept
{
    return IsInGameThread()&&OriginalComponentStorageOwner!=nullptr;
}
bool FPrivateRoadAccess::OriginalActorWorldStillMatches(const AActor* Actor) const
{
    const auto* W=OriginalWorldBirth.Get();const auto* L=OriginalLevelBirth.Get();
    return Actor&&W&&W==World&&L&&!W->bIsTearingDown
        &&!W->HasAnyFlags(RF_BeginDestroyed|RF_FinishDestroyed)
        &&!Actor->HasAnyFlags(RF_ClassDefaultObject|RF_BeginDestroyed|RF_FinishDestroyed)
        &&!L->HasAnyFlags(RF_BeginDestroyed|RF_FinishDestroyed)
        &&Actor->GetOuter()==L&&L->GetOuter()==W
        &&Speed::NativeObjectRead::MatchesOriginalBirth(W->PersistentLevel,OriginalLevelBirth)
        &&Speed::NativeObjectRead::MatchesOriginalBirth(L->OwningWorld,OriginalWorldBirth);
}
bool FPrivateRoadAccess::OriginalWorldHasNoGameMode() const
{
    const auto* W=OriginalWorldBirth.Get();const auto* C=OriginalWorldClassBirth.Get();
    return W&&W==World&&C&&W->GetClass()==C&&!C->HasAnyClassFlags(CLASS_NewerVersionExists)
        &&OriginalGameModeCell&&Speed::NativeObjectRead::IsResolvedNull(*OriginalGameModeCell);
}
USpeedBranchWheeledComponent* FPrivateRoadAccess::FindOriginalComponentAlias(const ISpeedComponent* Parent,const AActor* Owner) const
{
    if(!IsInGameThread()||!SelectsOriginalStorage()||!Parent||!Owner)return nullptr;
    for(const auto& Item:ActualCarAliases)
    {
        const auto& R=Item.Value;auto* A=R.Actor.Get();auto* C=R.Component.Get();
        const auto* AC=R.ActorClass.Get();const auto* OC=R.MovementOwnerClass.Get();
        if(!A||A!=Owner||!C||C!=Item.Key||!AC||!OC||!R.MovementCell)continue;
        if(A->GetClass()!=AC||AC->HasAnyClassFlags(CLASS_NewerVersionExists)||OC->HasAnyClassFlags(CLASS_NewerVersionExists)
            ||C->GetOwner()!=A||!OriginalActorWorldStillMatches(A)
            ||!Speed::NativeObjectRead::MatchesOriginalBirth(*R.MovementCell,TWeakObjectPtr<UChaosVehicleMovementComponent>(R.Component)))continue;
        // Real multiple-inheritance adjustment from a verified actual native
        // component. ISpeedComponent is not a UObject or a guessed raw offset.
        if(static_cast<const ISpeedComponent*>(C)==Parent)return C;
    }
    return nullptr;
}
const USpeedWorldSubsystem* FPrivateRoadAccess::OriginalBridgeForFrame(UWorld* W,const Speed::IStaticCollisionWorld* StaticWorld) const
{
    if(!IsInGameThread()||!SelectsOriginalStorage()||!Active||!W||W!=OriginalWorldBirth.Get()||!StaticWorld)
        throw Speed::NativeObjectRead::FUnresolvedOriginal();
    const auto* B=OriginalBridgeBirth.Get();const auto* F=OriginalFloorBirth.Get();
    if(!B||B->GetOuter()!=W||!F||!OriginalStaticSourceId||B->GetStaticCollisionWorld()!=StaticWorld
        ||!OriginalSourceGeometry.IsValid()||!OriginalQueryIdentity.IsValid()
        ||OriginalQueryIdentity.Data!=OriginalSourceGeometry.Get()
        ||!(StaticWorld->GetImmutableQueryIdentity1075()==OriginalQueryIdentity)
        ||!MatchesOriginalSourceBindings
        ||!MatchesOriginalSourceBindings(*B,OriginalSourceGeometry.Get(),OriginalStaticSourceId,OriginalFloorBirth))
        throw Speed::NativeObjectRead::FUnresolvedOriginal();
    // This is the actual IAS epoch/source participant that installed the
    // parent's frame StaticWorld, NOT a claim about an unchanged UE collection.
    return B;
}
bool FPrivateRoadAccess::SelectsOriginalStorage() const noexcept
{
    // Foreign threads return BEFORE reading GT-only epoch fields. The weak
    // route binding also prevents raw-address reuse after actual retirement.
    if(OriginalComponentStorageOwner!=this||!IsInGameThread())return false;
    return !Retired;
}
struct FPrivateRoadPhysicsWorld::FImpl
{
    struct FCar
    {
        FPrivateRoadCarSeed Seed;
        std::shared_ptr<RetirementDraft::FVehicleConfiguration> NativeConfiguration=std::make_shared<RetirementDraft::FVehicleConfiguration>();
        std::shared_ptr<RetirementDraft::FVehicleConfiguration> ReferenceConfiguration=std::make_shared<RetirementDraft::FVehicleConfiguration>();
TWeakObjectPtr<USpeedBranchWheeledComponent> ComponentWeak;
TWeakObjectPtr<ASpeedBranchCar> ActorWeak;
using FVehicleRoot=RetirementDraft::TVehicleRoot<USpeedWheeledComponent::FOriginalWheeledDomain>;
using FWheelRoot=RetirementDraft::TBodyRoot<FVehicleRoot,
    FPrivateRoadSubBodyValues,FPrivateRoadSolidValues,FPrivateRoadWheelRuntimeValues,
    FPrivateRoadWheelDomain,FPrivateRoadSphereDomain>;
using FBoxRoot=RetirementDraft::TBodyRoot<FVehicleRoot,
    FPrivateRoadSubBodyValues,FPrivateRoadSolidValues,FPrivateRoadBoxDomain>;
std::shared_ptr<FVehicleRoot> NativeVehicleRoot;
std::vector<std::shared_ptr<FWheelRoot>> NativeWheelRoots;
std::shared_ptr<FBoxRoot> NativeBoxRoot;
using FReferenceSimulation=std::remove_reference_t<decltype(
    std::declval<USpeedBranchWheeledComponent&>().VehicleSimulationPTWheeledDomainStorage())>;
FReferenceSimulation RetiringReferenceSimulation;
// The aliasing root wiring is a prerequisite, not a public Root/binder accessor.

        ASpeedBranchCar* Actor=nullptr;
        USpeedBranchWheeledComponent* Component=nullptr;
        // Address of the actual native TObjectPtr field, obtained once through
        // genuine reflected metadata outside all original recipe/math scopes.
        // Never a replacement field, guessed offset, setter or lifetime lease.
        const TObjectPtr<UChaosVehicleMovementComponent>* OriginalMovementCell=nullptr;
        TWeakObjectPtr<UClass> OriginalActorClassBirth,OriginalMovementOwnerClassBirth;
        U64 StableId=0;
        bool SolidInitializationWitness=false;
        TArray<TWeakObjectPtr<UChaosVehicleWheel>> OriginalWheelObjects;
        TWeakObjectPtr<UClass> OriginalWheelClassBirth;
        TWeakObjectPtr<UBoxSubBody> OriginalHitboxBirth;
        TArray<TWeakObjectPtr<USWheelSubBody>> OriginalWheelBodyBirth;
        USpeedSimulation* OriginalNativeSimulation=nullptr;
        Chaos::FSimpleWheeledVehicle* OriginalNativeVehicle=nullptr;
        Chaos::FSimpleWheelSim* OriginalNativeWheelAllocation=nullptr;
        Chaos::FSimpleSuspensionSim* OriginalNativeSuspensionAllocation=nullptr;
        std::unique_ptr<FBranchInputFilter> Filter;
    };
    TStrongObjectPtr<UWorld> World;
    struct FOriginalPin
    {
        TStrongObjectPtr<UObject> Retained;
        TWeakObjectPtr<UObject> Identity;
        explicit FOriginalPin(UObject* Object):Retained(Object),Identity(Object){}
        FOriginalPin(FOriginalPin&&)=default;
        FOriginalPin& operator=(FOriginalPin&&)=default;
        FOriginalPin(const FOriginalPin&)=delete;
        FOriginalPin& operator=(const FOriginalPin&)=delete;
        UObject* Get() const {return Identity.Get();} // original serial, never a weak minted after callback
    };
    TArray<FOriginalPin> Pins;
    USpeedWorldSubsystem* Bridge=nullptr;
    ASpeedBranchSimulation* Driver=nullptr;
    std::vector<std::unique_ptr<FCar>> Cars;
    std::shared_ptr<FPrivateRoadAccess> Access=std::make_shared<FPrivateRoadAccess>();
    std::unique_ptr<Speed::NativeEpoch::FDiagnosticOutbox> Diagnostics;
    mutable std::mutex Gate;
    std::atomic<std::shared_ptr<const FPrivateRoadPublication>> Published;
    std::shared_ptr<const Speed::RoadSettings::FDeclaredRecipe> DeclaredSettings;
    U64 Lifetime=0,VehicleContactFrameCount=0;
RetirementDraft::EPhase RetirementPhase=RetirementDraft::EPhase::Open;
TWeakObjectPtr<UWorld> OriginalWorldWeak;
TWeakObjectPtr<ULevel> OriginalLevelWeak;
TWeakObjectPtr<USpeedWorldSubsystem> OriginalBridgeWeak;
TWeakObjectPtr<ASpeedBranchSimulation> OriginalDriverWeak;
TWeakObjectPtr<AActor> OriginalFloorActorWeak;
TWeakObjectPtr<UBoxComponent> OriginalFloorWeak;
TSharedPtr<const Speed::Analytic::FAnalyticWorldData> OriginalFlatGeometry;
TArray<TWeakObjectPtr<USpeedBranchWheeledComponent>> OriginalComponentsWeak;
TArray<TWeakObjectPtr<USpeedBranchBoxSubBody>> OriginalBoxesWeak;
TArray<TWeakObjectPtr<USpeedBranchWheelSubBody>> OriginalWheelsWeak;
// Copies of actual original shared control blocks/aliasing ROOTS, never a new
// owning shared_ptr(rawUObjectOrNativePointer). No authority follows from these.
std::vector<std::shared_ptr<void>> NativeCppKeepAlive;

    std::vector<std::shared_ptr<FNativeConfigurationLifetimeTrace>> NativeConfigurationLifetime;
    bool Closed=false,Failed=false;
    void Pin(UObject* Object)
    {
        if(!Object)return;
        // This factory already owns the actual strong root and its weak birth
        // identity. Reusing that live original root does not mint a new weak
        // identity or register another strong reference during late PinTree.
        // Weak Get checks the captured original serial; a stale/reused raw
        // address alone cannot make an old root count as the current object.
        for(const auto& Existing:Pins)if(Existing.Get()==Object)return;
        Pins.Emplace(Object); // genuine first acquisition; remains callback-capable
    }
    // Original role identities are captured from the real default graph before
    // SetOwner/Initialize callbacks. This is a pre-math equality check, not a
    // replacement for GC exclusion or a complete restoration lease.
    bool OriginalObjectHandlesStillMatch() const
    {
        for(const auto& N:Cars)
        {
            if(!N->NativeVehicleRoot||!N->NativeVehicleRoot->Domain)return false;
            const auto& D=*N->NativeVehicleRoot->Domain;
            if(N->OriginalWheelBodyBirth.Num()!=4||N->OriginalWheelObjects.Num()!=4
                ||D.WheelSubBodies.Num()!=4||D.Wheels.Num()!=4||D.WheelSetups.Num()!=4
                ||!Speed::NativeObjectRead::MatchesOriginalBirth(D.HitboxSubBody,N->OriginalHitboxBirth))return false;
            for(int32 Index=0;Index<4;++Index)
                if(!Speed::NativeObjectRead::MatchesOriginalBirth(D.WheelSubBodies[Index],N->OriginalWheelBodyBirth[Index])
                    ||!Speed::NativeObjectRead::MatchesOriginalBirth(D.Wheels[Index],N->OriginalWheelObjects[Index])
                    ||!Speed::NativeObjectRead::MatchesOriginalClassBirth(D.WheelSetups[Index].WheelClass,N->OriginalWheelClassBirth)
                    ||N->OriginalWheelObjects[Index].Get()->GetClass()!=N->OriginalWheelClassBirth.Get())return false;
        }
        return true;
    }
    bool CaptureOriginalGameModeAbsence()
    {
        if(OriginalComponentStorageOwner||Speed::RoadSettings::SelectedOriginalRoadSettings())return false;
        auto* W=OriginalWorldWeak.Get();if(!W||W!=World.Get())return false;
        Access->OriginalWorldClassBirth=W->GetClass();
        const TWeakObjectPtr<UClass> ValueClassBirth=AGameModeBase::StaticClass();
        Pin(Access->OriginalWorldClassBirth.Get());Pin(ValueClassBirth.Get());
        auto* WC=Access->OriginalWorldClassBirth.Get();
        if(!WC||OriginalWorldWeak.Get()!=W||W->GetClass()!=WC||WC!=UWorld::StaticClass()
            ||!WC->HasAnyClassFlags(CLASS_Native)||WC->HasAnyClassFlags(CLASS_CompiledFromBlueprint|CLASS_NewerVersionExists))return false;
        const auto* P=FindFProperty<FObjectProperty>(WC,TEXT("AuthorityGameMode"));
        if(!P||P->GetOwnerClass()!=WC||P->ArrayDim!=1||!P->HasAllPropertyFlags(CPF_TObjectPtr)
            ||P->GetElementSize()!=sizeof(TObjectPtr<AGameModeBase>)||!P->IsInContainer(WC)
            ||!Speed::NativeObjectRead::MatchesOriginalBirth(P->PropertyClass,ValueClassBirth))return false;
        const auto* Cell=P->ContainerPtrToValuePtr<TObjectPtr<AGameModeBase>>(static_cast<const UObject*>(W));
        if(!Cell||reinterpret_cast<UPTRINT>(Cell)%alignof(TObjectPtr<AGameModeBase>))return false;
        const auto* ActualMode=W->GetAuthGameMode(); // actual ordinary positive read, outside GG/recipe/route
        FGCScopeGuard FinalModeReadBack;
        if(OriginalWorldWeak.Get()!=W||Access->OriginalWorldClassBirth.Get()!=WC||!ValueClassBirth.IsValid()
            ||W->GetClass()!=WC||WC->HasAnyClassFlags(CLASS_NewerVersionExists)||ActualMode
            ||!Speed::NativeObjectRead::IsResolvedNull(*Cell))return false;
        Access->OriginalGameModeCell=Cell;
        return Access->OriginalWorldHasNoGameMode();
    }
    bool OriginalMovementStillMatches(const FCar& N) const
    {
        auto* A=N.ActorWeak.Get();auto* C=N.ComponentWeak.Get();
        auto* AC=N.OriginalActorClassBirth.Get();auto* OC=N.OriginalMovementOwnerClassBirth.Get();
        return A&&A==N.Actor&&C&&C==N.Component&&AC&&OC&&N.OriginalMovementCell
            &&A->GetClass()==AC&&!AC->HasAnyClassFlags(CLASS_NewerVersionExists)
            &&!OC->HasAnyClassFlags(CLASS_NewerVersionExists)
            &&Speed::NativeObjectRead::MatchesOriginalBirth(*N.OriginalMovementCell,
                TWeakObjectPtr<UChaosVehicleMovementComponent>(N.ComponentWeak));
    }
    bool CaptureOriginalMovementCell(FCar& N)
    {
        if(OriginalComponentStorageOwner||Speed::RoadSettings::SelectedOriginalRoadSettings())return false;
        auto* A=N.ActorWeak.Get();auto* C=N.ComponentWeak.Get();if(!A||!C)return false;
        N.OriginalActorClassBirth=A->GetClass();
        N.OriginalMovementOwnerClassBirth=AWheeledVehiclePawn::StaticClass();
        Pin(N.OriginalActorClassBirth.Get());Pin(N.OriginalMovementOwnerClassBirth.Get());
        const TWeakObjectPtr<UClass> MovementValueClassBirth=UChaosVehicleMovementComponent::StaticClass();
        Pin(MovementValueClassBirth.Get());
        auto* AC=N.OriginalActorClassBirth.Get();auto* OC=N.OriginalMovementOwnerClassBirth.Get();
        if(!AC||!OC||N.ActorWeak.Get()!=A||N.ComponentWeak.Get()!=C||A->GetClass()!=AC
            ||!AC->HasAnyClassFlags(CLASS_Native)||AC->HasAnyClassFlags(CLASS_CompiledFromBlueprint|CLASS_NewerVersionExists)
            ||!OC->HasAnyClassFlags(CLASS_Native)||OC->HasAnyClassFlags(CLASS_NewerVersionExists))return false;
        const auto* Property=FindFProperty<FObjectProperty>(OC,TEXT("VehicleMovementComponent"));
        if(!Property||Property->GetOwnerClass()!=OC||Property->ArrayDim!=1
            ||!Property->HasAllPropertyFlags(CPF_TObjectPtr)
            ||Property->GetElementSize()!=sizeof(TObjectPtr<UChaosVehicleMovementComponent>)
            ||!Property->IsInContainer(AC)
            ||!Speed::NativeObjectRead::MatchesOriginalBirth(Property->PropertyClass,MovementValueClassBirth))return false;
        const auto* Cell=Property->ContainerPtrToValuePtr<TObjectPtr<UChaosVehicleMovementComponent>>(static_cast<const UObject*>(A));
        if(!Cell||reinterpret_cast<UPTRINT>(Cell)%alignof(TObjectPtr<UChaosVehicleMovementComponent>))return false;
        const auto* ActualGetter=A->GetVehicleMovement(); // real ordinary read, BEFORE the recipe/route
        // All actual tracked reads, pins and reflected metadata operations
        // above execute OUTSIDE GC exclusion. A foreign OnRead callback may
        // collect/replace/retire objects. Only their ORIGINAL weak identities
        // and classes may admit the subsequent readonly native field access.
        FGCScopeGuard FinalActualFieldReadBack;
        if(N.ActorWeak.Get()!=A||N.ComponentWeak.Get()!=C||N.OriginalActorClassBirth.Get()!=AC
            ||N.OriginalMovementOwnerClassBirth.Get()!=OC||!MovementValueClassBirth.IsValid()||A->GetClass()!=AC
            ||AC->HasAnyClassFlags(CLASS_NewerVersionExists)||OC->HasAnyClassFlags(CLASS_NewerVersionExists)||ActualGetter!=C
            ||!Speed::NativeObjectRead::MatchesOriginalBirth(*Cell,TWeakObjectPtr<UChaosVehicleMovementComponent>(N.ComponentWeak)))return false;
        N.OriginalMovementCell=Cell;
        return OriginalMovementStillMatches(N);
    }
    bool OriginalActorWorldStillMatches(const AActor* A,const UWorld* W) const
    {
        // Same genuine private runtime topology predicate, also used by the
        // native driver's inherited GetWorld calls while the route is active.
        return W==OriginalWorldWeak.Get()&&Access->OriginalActorWorldStillMatches(A);
    }
    bool AcquiredGraphStillLive() const
    {
        auto* W=OriginalWorldWeak.Get();auto* B=OriginalBridgeWeak.Get();auto* D=OriginalDriverWeak.Get();
        if(!W||W!=World.Get()||W->bIsTearingDown||W->HasAnyFlags(RF_BeginDestroyed|RF_FinishDestroyed)
            ||!B||B!=Bridge||B->HasAnyFlags(RF_BeginDestroyed|RF_FinishDestroyed)
            ||!D||D!=Driver||D->IsActorBeingDestroyed()||D->HasAnyFlags(RF_BeginDestroyed|RF_FinishDestroyed))return false;
        if(!Access->OriginalWorldHasNoGameMode())return false;
        for(const auto& P:Pins)if(!P.Get())return false;
        if(!OriginalFloorActorWeak.IsExplicitlyNull()||!OriginalFloorWeak.IsExplicitlyNull())
        {
            auto* A=OriginalFloorActorWeak.Get();auto* F=OriginalFloorWeak.Get();
            if(!A||!F||A->IsActorBeingDestroyed()||!OriginalActorWorldStillMatches(A,W)||F->GetOwner()!=A
                ||A->HasAnyFlags(RF_BeginDestroyed|RF_FinishDestroyed)
                ||F->HasAnyFlags(RF_BeginDestroyed|RF_FinishDestroyed))return false;
        }
        for(const auto& Weak:OriginalComponentsWeak)if(!Weak.IsValid())return false;
        for(const auto& Weak:OriginalBoxesWeak)if(!Weak.IsValid())return false;
        for(const auto& Weak:OriginalWheelsWeak)if(!Weak.IsValid())return false;
        for(const auto& N:Cars)
        {
            if(!N->Actor&&!N->Component)
            {if(!N->ActorWeak.IsExplicitlyNull()||!N->ComponentWeak.IsExplicitlyNull())return false;continue;}
            auto* A=N->ActorWeak.Get();auto* C=N->ComponentWeak.Get();
            if(!A||A!=N->Actor||!C||C!=N->Component||A->IsActorBeingDestroyed()
                ||A->HasAnyFlags(RF_BeginDestroyed|RF_FinishDestroyed)||C->HasAnyFlags(RF_BeginDestroyed|RF_FinishDestroyed)
                ||!OriginalActorWorldStillMatches(A,W)||C->GetOwner()!=A||!OriginalMovementStillMatches(*N))return false;
        }
        return true;
    }
    void PinTree(UObject* Object)
    {
        Pin(Object);ForEachObjectWithOuter(Object,[this](UObject* Child){Pin(Child);},EGetObjectsFlags::IncludeNestedObjects);
    }
};
FPrivateRoadPhysicsWorld::FPrivateRoadPhysicsWorld():Impl(std::make_unique<FImpl>()){}
FPrivateRoadPhysicsWorld::~FPrivateRoadPhysicsWorld()
{
    // This finite prototype has no worker. Its UObject graph is constructed
    // and destroyed on the GT; a later async owner needs its own real join.
    checkf(IsInGameThread(),TEXT("Private road world must retire on the game thread"));
    const bool Retired=Close();
    checkf(Retired,TEXT("Private road world cannot retire during an active step"));
    if(!Retired)std::terminate(); // retirement must execute in Shipping too
}
std::unique_ptr<FPrivateRoadPhysicsWorld> FPrivateRoadPhysicsWorld::Create(std::vector<FPrivateRoadCarSeed> Seeds,FString& Why)
{
    Why.Empty();
    if(OriginalComponentStorageOwner||!IsInGameThread()||Speed::Input::FPresentationInputScope::IsActive()
        ||Speed::Netcode::PresentationClient::IsSelected()||Seeds.empty()||Seeds.size()>16
        ||Chaos::FPhysicsSolverBase::IsNetworkPhysicsPredictionEnabled())
    {Refuse(Why,TEXT("private native world requires GT, IAS-only physical profile, disabled Chaos prediction and finite roster"));return {};}
    std::sort(Seeds.begin(),Seeds.end(),[](const auto& A,const auto& B){return A.Actor<B.Actor;});
    for(const auto& Seed:Seeds)if(!Seed.Contract||!PrivateRoadSupportedContract(*Seed.Contract))
    {Refuse(Why,TEXT("unsupported private road action contract"));return {};}
    if(Speed::RoadSettings::SelectedOriginalRoadSettings())
    {Refuse(Why,TEXT("nested private birth inside original native settings math"));return {};}
    const auto SuppliedSettings=Seeds.front().DeclaredSettings;
    for(const auto& S:Seeds)
        if(bool(S.DeclaredSettings)!=bool(SuppliedSettings)||(SuppliedSettings
            &&S.DeclaredSettings->CanonicalIdentity()!=SuppliedSettings->CanonicalIdentity()))
        {Refuse(Why,TEXT("mixed explicit original native settings recipes"));return {};}
    // This original profile executes only on the GT. Do not silently let a
    // caller select parallel physical readers outside its immutable scope.
    if(SuppliedSettings&&SuppliedSettings->Read<int32>(Speed::RoadSettings::Catalog[72])!=1)
    {Refuse(Why,TEXT("original road settings require the actual serial native lane"));return {};}
    const auto StartPolicy=Seeds.front().StartPolicy;
    if(StartPolicy>EPrivateRoadStartPolicy::ExplicitUnitRecipe)return {};
    for(const auto& S:Seeds)if(S.StartPolicy!=StartPolicy
        ||(StartPolicy==EPrivateRoadStartPolicy::AwaitInitialPublication&&S.CountdownFrames))
    {Refuse(Why,TEXT("mixed startup policy or countdown substituted for initial publication"));return {};}
    auto Owner=std::unique_ptr<FPrivateRoadPhysicsWorld>(new FPrivateRoadPhysicsWorld);
    auto& I=*Owner->Impl;
    I.DeclaredSettings=SuppliedSettings;
    I.Diagnostics.reset(new Speed::NativeEpoch::FDiagnosticOutbox);
    I.Access->RequiresInitialPublication=StartPolicy==EPrivateRoadStartPolicy::AwaitInitialPublication;
    U64 Last=0;
    for(auto& Seed:Seeds)
    {
        if(!Seed.Actor||!Seed.Incarnation||Seed.Actor==Last||!Finite(Seed.Initial))
        {Refuse(Why,TEXT("invalid native initial roster/kinematics"));return {};}
        Last=Seed.Actor;
        auto Node=std::make_unique<FImpl::FCar>();Node->Seed=std::move(Seed);
        const auto& S=Node->Seed;
        Node->Filter=FBranchInputFilter::Create(S.Role,S.Contract,S.Processing,S.Local,S.LocalEpoch,S.Remote,S.PredictionHorizon,{});
        if(!Node->Filter){Refuse(Why,TEXT("private input filter configuration refused"));return {};}
        I.NativeConfigurationLifetime.push_back(Node->NativeConfiguration->Lifetime);
        I.NativeConfigurationLifetime.push_back(Node->ReferenceConfiguration->Lifetime);
        I.Cars.push_back(std::move(Node));
    }
    I.Access->ContactDispatches.reserve(I.Cars.size()*(I.Cars.size()-1)/2);
    const auto Options=UWorld::InitializationValues().InitializeScenes(false).CreateFXSystem(false).AllowAudioPlayback(false).CreatePhysicsScene(false)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false).SetTransactional(false);
    I.World.Reset(UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Options));
    if(!I.World.IsValid()){Refuse(Why,TEXT("private world creation failed"));return {};}
    I.OriginalWorldWeak=I.World.Get();
    I.OriginalLevelWeak=I.World->PersistentLevel.Get();I.Pin(I.OriginalLevelWeak.Get());
    I.Access->OriginalWorldBirth=I.OriginalWorldWeak;I.Access->OriginalLevelBirth=I.OriginalLevelWeak;
    if(!I.OriginalLevelWeak.IsValid()||!Speed::NativeObjectRead::MatchesOriginalBirth(
        I.OriginalLevelWeak.Get()->OwningWorld,I.OriginalWorldWeak))return {};
    // InitWorld assigns this tick flag only inside its scene-initialization
    // branch. Set it explicitly for the deliberately scene-less private world.
    I.World->bShouldSimulatePhysics=false;
    if(I.World->Scene||I.World->GetPhysicsScene())
    {Refuse(Why,TEXT("renderer or physics scene escaped private construction policy"));return {};}
    // Deliberately no Engine world context, BeginPlay, normal tick or scene.
    I.Access->World=I.World.Get();
    if(!I.CaptureOriginalGameModeAbsence()){Refuse(Why,TEXT("original native World GameMode absence binding refused"));return {};}
    I.Bridge=I.World->GetSubsystem<USpeedWorldSubsystem>();
    if(!I.Bridge){Refuse(Why,TEXT("private native bridge missing"));return {};}
    I.OriginalBridgeWeak=I.Bridge;I.Pin(I.Bridge);I.Access->OriginalBridgeBirth=I.OriginalBridgeWeak;
    I.Bridge->BindOriginalEpochStorage(I.Access);
    I.NativeCppKeepAlive.push_back(I.Bridge->OriginalEpochBridgeStorage);
    auto Data=MakeShared<Speed::Analytic::FAnalyticWorldData>();
    auto& P=Data->Planes.AddDefaulted_GetRef();P.SourceId=1;P.SurfaceId=2;P.FeatureId=3;P.PrimitiveId=4;
    I.Access->OriginalStaticSourceId=P.SourceId;
    P.ObjectType=uint32(ECC_WorldStatic);P.BlockingChannels=MAX_uint64;
    P.Normal=FVector3d::UpVector;P.AxisU=FVector3d::ForwardVector;P.AxisV=FVector3d::RightVector;
    P.HalfExtents=FVector2d(100000,100000);
    if(!Data->FinalizeAndValidate(&Why))return {};
    // Exact authored plane, not an imported asset authority certificate.
    Data->Planes[0].bAuthorityEligible=true;
    I.OriginalFlatGeometry=Data;
    I.Access->OriginalSourceGeometry=I.OriginalFlatGeometry; // same genuine shared control block, not shared_ptr(raw)
    I.Access->MatchesOriginalSourceBindings=+[](const USpeedWorldSubsystem& B,const Speed::Analytic::FAnalyticWorldData* Original,U64 SourceId,const TWeakObjectPtr<UBoxComponent>& FloorBirth)
    {
        // Lexical owner friend context reads the ACTUAL selected epoch map.
        // This private fixed thunk returns only a predicate; no reference,
        // lambda capture, original native view or new authority escapes.
        if(!B.UsesOriginalEpochStorage())return false;
        const auto& Geometry=B.AnalyticWorldDataEpochStorage();
        const auto& Sources=B.AnalyticSourceComponentsEpochStorage();
        const auto* Floor=FloorBirth.Get();const auto* Source=Sources.Find(SourceId);
        return Floor&&Original&&Geometry.IsValid()&&Geometry.Get()==Original&&Geometry->Planes.Num()==1
            &&Geometry->Planes[0].SourceId==SourceId&&Sources.Num()==1&&Source&&Source->Get()==Floor;
    };
    auto* FloorActor=I.World->SpawnActor<AActor>();
    if(!FloorActor){Refuse(Why,TEXT("private floor actor missing"));return {};}
    auto* Floor=NewObject<UBoxComponent>(FloorActor);FloorActor->AddInstanceComponent(Floor);
    Floor->SetMobility(EComponentMobility::Static);Floor->SetCollisionObjectType(ECC_WorldStatic);
    Floor->SetCollisionResponseToAllChannels(ECR_Block);I.PinTree(FloorActor);
    I.OriginalFloorActorWeak=FloorActor;I.OriginalFloorWeak=Floor;I.Access->OriginalFloorBirth=I.OriginalFloorWeak;
    {
        FOriginalComponentStorageScope OriginalWorldAdmission(*I.Access);
        I.Bridge->AnalyticSourceComponentsEpochStorage().Add(1,I.OriginalFloorWeak); // genuine existing birth weak, no raw reconstruction
        I.Bridge->AnalyticWorldDataEpochStorage()=Data;
        auto OriginalCollision=MakeUnique<Speed::FAnalyticStaticCollisionWorld>(*Data);
        I.Access->OriginalQueryIdentity=OriginalCollision->GetImmutableQueryIdentity1075();
        if(!I.Access->OriginalQueryIdentity.IsValid()||I.Access->OriginalQueryIdentity.Data!=&Data.Get())return {};
        I.Bridge->SimulationWorldEpochStorage().SetStaticCollisionWorld(MoveTemp(OriginalCollision));
    }
    I.Driver=I.World->SpawnActor<ASpeedBranchSimulation>();
    if(!I.Driver||ASpeedPhysicsSimulation::GetEngineFPS()!=300)
    {Refuse(Why,TEXT("private driver missing or canonical physical rate differs"));return {};}
    I.OriginalDriverWeak=I.Driver;I.Access->OriginalDriverBirth=I.OriginalDriverWeak;
    I.Driver->Access=I.Access;if(!I.Driver->BindOriginalDriverStorage(I.Access)){Refuse(Why,TEXT("original native driver birth storage refused"));return {};}
    I.NativeCppKeepAlive.push_back(I.Driver->OriginalDriverStorage);
    {FOriginalComponentStorageScope OriginalDriver(*I.Access);
        I.Driver->bPublishPresentationDriverStorage()=true;I.Driver->bPublishCompletedWorldInputDriverStorage()=false;}
    I.PinTree(I.Driver);
    const auto BuildOriginalVehicle=[&](USpeedBranchWheeledComponent& C,std::shared_ptr<RetirementDraft::FVehicleConfiguration> Configuration,const TArray<UChaosVehicleWheel*>& PreparedWheels)
    {
        auto& WheelConfig=Configuration->Wheel;auto& SuspensionConfig=Configuration->Suspension;
        auto Vehicle=MakeUnique<Chaos::FSimpleWheeledVehicle>();
        for(int32 W=0;W<4;++W)
        {
            auto& Wheel=Vehicle->Wheels.Emplace_GetRef(&WheelConfig);
            Wheel.MaxOmega=10000;Wheel.AppliedLinearDriveForce=0;Wheel.AppliedLinearBrakeForce=0;
            Wheel.LongitudinalAdhesiveLimit=0;Wheel.LateralAdhesiveLimit=0;Wheel.SideSlipModifier=WheelConfig.SideSlipModifier;
            Vehicle->Suspension.Emplace_GetRef(&SuspensionConfig).SetLocalRestingPosition(FVector(W<2?70:-70,W%2?45:-45,-15));
            C.WheelsWheeledDomainStorage().Add(PreparedWheels[W]);
        }
        C.VehicleSimulationPTWheeledDomainStorage()=MakeUnique<RetirementDraft::TSimulationWithConfiguration<USpeedSimulation,RetirementDraft::FVehicleConfiguration>>(Configuration);
        C.SkySimulationWheeledDomainStorage()=static_cast<USpeedSimulation*>(C.VehicleSimulationPTWheeledDomainStorage().Get());
        C.SkySimulationWheeledDomainStorage()->Init(Vehicle);
        C.BindWheelSimulationPointers();
    };
    for(auto& Node:I.Cars)
    {
        auto& N=*Node;N.Actor=I.World->SpawnActor<ASpeedBranchCar>();
        N.Component=N.Actor?Cast<USpeedBranchWheeledComponent>(N.Actor->GetVehicleMovement()):nullptr;
        auto* C=N.Component;
        if(!C||C->GetClass()!=USpeedBranchWheeledComponent::StaticClass()||!C->HasIsolatedNativeConstruction())
        {Refuse(Why,TEXT("private native component subtype missing"));return {};}
        C->Access=I.Access;
        if(!C->BindOriginalWheeledDomain(I.Access)){Refuse(Why,TEXT("original native component domain birth refused"));return {};}
        N.ActorWeak=N.Actor;N.ComponentWeak=C;I.OriginalComponentsWeak.Add(C);
        I.PinTree(N.Actor);I.PinTree(C);
        if(!I.CaptureOriginalMovementCell(N))return {};
        I.Access->ActualCarAliases.Add(C,FPrivateRoadAccess::FActualCarAlias{
            N.ActorWeak,N.ComponentWeak,N.OriginalActorClassBirth,N.OriginalMovementOwnerClassBirth,N.OriginalMovementCell});
        {
            FGCScopeGuard CaptureOriginalBirthPins;
            auto* Box=Cast<UBoxSubBody>(C->GetDefaultSubobjectByName(USpeedWheeledComponent::HitboxName));
            if(!IsValid(Box))return {};
            N.OriginalHitboxBirth=Box;
            for(int32 Index=0;Index<4;++Index)
            {
                auto* Wheel=Cast<USWheelSubBody>(C->GetDefaultSubobjectByName(USpeedWheeledComponent::WheelNames[Index]));
                if(!IsValid(Wheel))return {};
                N.OriginalWheelBodyBirth.Add(Wheel);
            }
        }
        if(!I.AcquiredGraphStillLive())return {};
        {
            FGCScopeGuard BindActualBodyIdentities;
            auto* Box=Cast<USpeedBranchBoxSubBody>(N.OriginalHitboxBirth.Get());
            if(!Box||N.OriginalWheelBodyBirth.Num()!=4)return {};
            Box->OriginalBodyIdentity=N.OriginalHitboxBirth; // actual weak copy, no raw reconstruction
            for(int32 Index=0;Index<4;++Index)
            {
                auto* Wheel=Cast<USpeedBranchWheelSubBody>(N.OriginalWheelBodyBirth[Index].Get());
                if(!Wheel)return {};
                Wheel->OriginalBodyIdentity=N.OriginalWheelBodyBirth[Index];
            }
        }
        N.NativeVehicleRoot=std::make_shared<FImpl::FCar::FVehicleRoot>(N.NativeConfiguration,C->OriginalWheeledDomain);
        C->OriginalWheeledDomain=RetirementDraft::AliasExistingPart(N.NativeVehicleRoot,N.NativeVehicleRoot->Domain);
        I.NativeCppKeepAlive.push_back(N.NativeVehicleRoot);
        C->SetAsyncPhysicsTickEnabled(false);
        // Retain the actual base-wheel class before any prepared-wheel birth
        // callback. The original profile authors exactly this class; changed,
        // stale or unresolved setup handles later refuse admission.
        N.OriginalWheelClassBirth=UChaosVehicleWheel::StaticClass();
        I.Pin(N.OriginalWheelClassBirth.Get());
        // Both weak assignment and Pin can dispatch actual read callbacks.
        // Revalidate before constructing any child in the acquired component.
        if(!N.OriginalWheelClassBirth.IsValid()||!I.AcquiredGraphStillLive()||!N.OriginalWheelClassBirth.IsValid())
        {Refuse(Why,TEXT("original graph/class invalidated before prepared wheel creation"));return {};}
        const auto PrepareWheelObjects=[&]()
        {
            TArray<UChaosVehicleWheel*> Result;
            for(int32 W=0;W<4;++W)
            {
                auto* Wheel=NewObject<UChaosVehicleWheel>(C);
                if(!Wheel||!I.AcquiredGraphStillLive())return TArray<UChaosVehicleWheel*>{};
                I.Pin(Wheel); // BEFORE the next genuine NewObject callback can collect this wheel
                Wheel->WheelRadius=30;Wheel->SuspensionMaxRaise=10;Wheel->SuspensionMaxDrop=25;Result.Add(Wheel);
            }
            return Result;
        };
        const auto ReferenceWheels=PrepareWheelObjects(),OriginalWheels=PrepareWheelObjects();
        if(ReferenceWheels.Num()!=4||OriginalWheels.Num()!=4)return {};
        for(auto* Wheel:OriginalWheels)N.OriginalWheelObjects.Add(Wheel);
        if(!I.Diagnostics->Begin()){Refuse(Why,TEXT("native birth diagnostics unavailable"));return {};}
        bool BirthOutboxActive=true;ON_SCOPE_EXIT{if(BirthOutboxActive)I.Diagnostics->End();};
        const bool BirthConfigured=[&]() -> bool
        {
        try {
        // Ordinary reference/facade lifecycle is callback-capable. No GC,
        // original recipe or original storage scope is held here. The same
        // owned diagnostic outbox remains active until after native birth.
        C->SetOwner(N.Actor);
        if(!I.AcquiredGraphStillLive())return false;
        const TArray<USSubBody*> FacadeTopology=C->GetSubBodies(); // owned identity list, never a physical-state import
        const auto FacadeTopologyStillMatches=[&]() -> bool
        {
            if(!I.AcquiredGraphStillLive()||FacadeTopology.Num()!=5||N.OriginalWheelBodyBirth.Num()!=4)return false;
            const auto& Current=C->SubBodiesWheeledDomainStorage();
            const auto& Wheels=C->WheelSubBodiesWheeledDomainStorage();
            if(Current.Num()!=5||Wheels.Num()!=4
                ||FacadeTopology[0]!=N.OriginalHitboxBirth.Get()||Current[0]!=FacadeTopology[0]
                ||!Speed::NativeObjectRead::MatchesOriginalBirth(C->HitboxSubBodyWheeledDomainStorage(),N.OriginalHitboxBirth))return false;
            for(int32 Index=0;Index<4;++Index)
                if(FacadeTopology[Index+1]!=N.OriginalWheelBodyBirth[Index].Get()||Current[Index+1]!=FacadeTopology[Index+1]
                    ||!Speed::NativeObjectRead::MatchesOriginalBirth(Wheels[Index],N.OriginalWheelBodyBirth[Index]))return false;
            return true;
        };
        if(!FacadeTopologyStillMatches())return false;
        for(auto* Body:FacadeTopology)
        {
            if(auto* Box=Cast<USpeedBranchBoxSubBody>(Body))
            {
                Box->StorageAccess=I.Access;I.OriginalBoxesWeak.Add(Box);
                if(!FacadeTopologyStillMatches()||!Box->OwnedBodyValues||!Box->OwnedSolidValues||!Box->OwnedBoxDomain)return false;
                N.NativeBoxRoot=std::make_shared<FImpl::FCar::FBoxRoot>(N.NativeVehicleRoot,std::move(Box->OwnedBodyValues),std::move(Box->OwnedSolidValues),std::move(Box->OwnedBoxDomain));
                Box->OwnedBodyValues=RetirementDraft::AliasBodyPart<0>(N.NativeBoxRoot);
                Box->OwnedSolidValues=RetirementDraft::AliasBodyPart<1>(N.NativeBoxRoot);
                Box->OwnedBoxDomain=RetirementDraft::AliasBodyPart<2>(N.NativeBoxRoot);
                I.NativeCppKeepAlive.push_back(N.NativeBoxRoot);
            }
            else if(auto* Wheel=Cast<USpeedBranchWheelSubBody>(Body))
            {
                Wheel->StorageAccess=I.Access;I.OriginalWheelsWeak.Add(Wheel);
                if(!FacadeTopologyStillMatches()||!Wheel->OwnedBodyValues||!Wheel->OwnedSolidValues||!Wheel->OwnedWheelRuntime||!Wheel->OwnedWheelDomain||!Wheel->OwnedSphereDomain)return false;
                auto Root=std::make_shared<FImpl::FCar::FWheelRoot>(N.NativeVehicleRoot,std::move(Wheel->OwnedBodyValues),std::move(Wheel->OwnedSolidValues),std::move(Wheel->OwnedWheelRuntime),std::move(Wheel->OwnedWheelDomain),std::move(Wheel->OwnedSphereDomain));
                Wheel->OwnedBodyValues=RetirementDraft::AliasBodyPart<0>(Root);
                Wheel->OwnedSolidValues=RetirementDraft::AliasBodyPart<1>(Root);
                Wheel->OwnedWheelRuntime=RetirementDraft::AliasBodyPart<2>(Root);
                Wheel->OwnedWheelDomain=RetirementDraft::AliasBodyPart<3>(Root);
                Wheel->OwnedSphereDomain=RetirementDraft::AliasBodyPart<4>(Root);
                N.NativeWheelRoots.push_back(Root);I.NativeCppKeepAlive.push_back(Root);
            }
            else {Refuse(Why,TEXT("private subbody storage subtype missing"));return {};}
        }
        auto* const FacadeHitbox=C->GetHitboxSubBodyForConfiguration();
        if(!FacadeTopologyStillMatches()||!FacadeHitbox||FacadeHitbox!=N.OriginalHitboxBirth.Get()
            ||FacadeHitbox->GetClass()!=USpeedBranchBoxSubBody::StaticClass())
        {Refuse(Why,TEXT("private native contact dispatch subtype missing"));return {};}
        N.ReferenceConfiguration->Suspension.SetSuspensionMaxRaise(10);N.ReferenceConfiguration->Suspension.SetSuspensionMaxDrop(25);
        N.ReferenceConfiguration->Suspension.SpringRate=5000;N.ReferenceConfiguration->Suspension.RestingForce=250000;
        N.ReferenceConfiguration->Suspension.CompressionDamping=1000;N.ReferenceConfiguration->Suspension.ReboundDamping=1000;
        N.NativeConfiguration->Suspension.SetSuspensionMaxRaise(10);N.NativeConfiguration->Suspension.SetSuspensionMaxDrop(25);
        N.NativeConfiguration->Suspension.SpringRate=5000;N.NativeConfiguration->Suspension.RestingForce=250000;
        N.NativeConfiguration->Suspension.CompressionDamping=1000;N.NativeConfiguration->Suspension.ReboundDamping=1000;
        BuildOriginalVehicle(*C,N.ReferenceConfiguration,ReferenceWheels);
        if(!FacadeTopologyStillMatches()){Refuse(Why,TEXT("private facade wheel topology changed"));return false;}
        for(int32 W=0;W<4;++W)
        {
            auto* const Wheel=N.OriginalWheelBodyBirth[W].Get();if(!Wheel)return false;
            Wheel->SetLocalOffset(FVector(W<2?70:-70,W%2?45:-45,-15));
            if(!FacadeTopologyStillMatches())return false;
            Wheel->SetRadiusForConfiguration(30);
            if(!FacadeTopologyStillMatches())return false;
        }
        if(!C->ValidateSimulationBindings(Why)||!FacadeTopologyStillMatches())return false;
        if(C->ThrottleAccelWheeledDomainStorage()!=750||C->BrakeAccelWheeledDomainStorage()!=1100||C->PhysDampingWheeledDomainStorage()!=0||C->AirForwardThrottleWheeledDomainStorage()!=0||C->AirBackwardThrottleWheeledDomainStorage()!=0)
        {Refuse(Why,TEXT("stock native driving profile differs"));return {};}
        C->EstablishOriginalComponentStorage();
        I.NativeCppKeepAlive.push_back(C->OwnedComponentValues);
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
        // Witness only: actual legacy values from the original setup. These
        // are NEVER imported into physical storage or a restore certificate.
        TArray<uint8> OriginalSolidRecipe;
        for(auto* Body:FacadeTopology)
        {
            if(const auto* Box=Cast<USpeedBranchBoxSubBody>(Body))
            {if(!PartialObservation::AppendSolidValues(OriginalSolidRecipe,Box->ReadLegacySolidDiagnosticValues()))return {};}
            else if(const auto* Wheel=Cast<USpeedBranchWheelSubBody>(Body))
            {if(!PartialObservation::AppendSolidValues(OriginalSolidRecipe,Wheel->ReadLegacySolidDiagnosticValues()))return {};}
            else return {};
        }
#endif
        if(!FacadeTopologyStillMatches())return false;
        // Real actor-facing queries and tick lifecycle stay outside the
        // original physical storage route. These are actual native actor
        // values, not a caller pose, copied facade physical state or permission.
        if(Speed::Netcode::PresentationClient::SuppressesLocalPhysics(C->GetWorld()))
            return Refuse(Why,TEXT("private owner facade admission changed"));
        const FQuat OriginalOwnerRotation=N.Actor->GetActorRotation().Quaternion();
        const FVector OriginalOwnerLocation=N.Actor->GetActorLocation();
        C->SetAsyncPhysicsTickEnabled(false);
        if(!FacadeTopologyStillMatches()||OriginalOwnerRotation.ContainsNaN()
            ||!OriginalOwnerRotation.IsNormalized()||OriginalOwnerLocation.ContainsNaN())
            return Refuse(Why,TEXT("original private actor pose/binding unavailable"));
        FGCScopeGuard BirthMathOnly;
        if(!I.AcquiredGraphStillLive())return false; // actual original readback after the last callback
        Speed::RoadSettings::FOriginalRoadSettingsScope SettingsBirthMath(I.DeclaredSettings);
        FOriginalComponentStorageScope OriginalStorage(*I.Access);
        // The actual private domain is initialized through native operations
        // from its explicit constructor/profile program. No facade values,
        // container headers, vehicle state or mutable config views are copied.
        C->InitChaosVehicle();
        Speed::Driving::ApplyStockRoadCar(*C->OriginalWheeledDomain);
        C->GravityZWheeledDomainStorage()=GetDefault<UPhysicsSettings>()->DefaultGravityZ;
        C->SetEngineFPS(300);
        TArray<USSubBody*> OriginalTopology;
        auto* OriginalBox=Cast<USpeedBranchBoxSubBody>(C->GetDefaultSubobjectByName(USpeedWheeledComponent::HitboxName));
        if(!OriginalBox||OriginalBox->GetClass()!=USpeedBranchBoxSubBody::StaticClass())return {};
        OriginalTopology.Add(OriginalBox);
        for(int32 W=0;W<4;++W)
        {
            auto* Body=Cast<USpeedBranchWheelSubBody>(C->GetDefaultSubobjectByName(USpeedWheeledComponent::WheelNames[W]));
            if(!Body||Body->GetClass()!=USpeedBranchWheelSubBody::StaticClass()||Body->GetOwner()!=N.Actor)return {};
            OriginalTopology.Add(Body);
        }
        C->SetSubBodies(OriginalTopology);
        C->InitializeOriginalOwnerFromCapturedPose(N.Actor,OriginalOwnerRotation,OriginalOwnerLocation);
        I.Bridge->RegisterOriginalComponentForPrivateOwner(C);
        BuildOriginalVehicle(*C,N.NativeConfiguration,OriginalWheels);
        N.OriginalNativeSimulation=C->SkySimulationWheeledDomainStorage();
        N.OriginalNativeVehicle=N.OriginalNativeSimulation?N.OriginalNativeSimulation->PVehicle.Get():nullptr;
        if(!N.OriginalNativeVehicle||N.OriginalNativeVehicle->Wheels.Num()!=4||N.OriginalNativeVehicle->Suspension.Num()!=4)return {};
        N.OriginalNativeWheelAllocation=N.OriginalNativeVehicle->Wheels.GetData();
        N.OriginalNativeSuspensionAllocation=N.OriginalNativeVehicle->Suspension.GetData();
        for(int32 W=0;W<4;++W)
        {
            auto* Body=C->ReadWheelSubBodiesForPhysics()[W].Get();
            Body->SetLocalOffset(FVector(W<2?70:-70,W%2?45:-45,-15));
            Body->SetRadiusForConfiguration(30);
            // Same original authored host travel, stored as original native
            // config instead of rereading the mutable UChaosWheel facade.
            Body->ConfigureSuspensionTravel(10,25);
        }
        if(!C->ValidateSimulationBindings(Why))return {};

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
        const auto CaptureOutsideSolidValues=[&](TArray<uint8>& Bytes)
        {
            for(const auto* Body:C->GetSubBodies())
            {
                // Explicit BASE witness excludes the newly initialized solid
                // fields. It retains original parent/type/transform/channel,
                // core hits and ordered ignored/external lists field by field.
                Body->USSubBody::AppendEpisodeStateForTesting(Bytes);
                if(const auto* Box=Cast<USpeedBranchBoxSubBody>(Body))
                    Speed::FEpisodeStateBytesForTesting::Add(Bytes,Box->GetBoxExtent());
                else if(const auto* Wheel=Cast<USpeedBranchWheelSubBody>(Body))
                {
                    Speed::FEpisodeStateBytesForTesting::Add(Bytes,Wheel->GetRadiusWithMargin());
                    Speed::FEpisodeStateBytesForTesting::Add(Bytes,Wheel->GetChaosWheelConfiguration());
                    Speed::FEpisodeStateBytesForTesting::Add(Bytes,Wheel->GetWheelSim());
                    Speed::FEpisodeStateBytesForTesting::Add(Bytes,Wheel->GetSuspensionSim());
                    if(!Wheel->GetWheelSim()||!Wheel->GetSuspensionSim())return false;
                    PartialObservation::AppendCheckedWheel(Bytes,*Wheel->GetWheelSim());
                    const PartialObservation::FCheckedSuspensionCopy Suspension(*Wheel->GetSuspensionSim());Suspension.Append(Bytes);
                }
                else return false;
            }
            return true;
        };
        TArray<uint8> BeforeOther,AfterOther,ActualSolidRecipe;
        if(!CaptureOutsideSolidValues(BeforeOther))return {};
#endif
        for(auto* Body:C->GetSubBodies())Body->Initialize(C);
        for(int32 W=0;W<C->ReadWheelSubBodiesForPhysics().Num();++W)C->ReadWheelSubBodiesForPhysics()[W]->SetIdx(W);
        // Real validation AFTER every new initializer, under the same scope.
        if(!C->ValidateSimulationBindings(Why))return {};
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
        if(!CaptureOutsideSolidValues(AfterOther)||BeforeOther!=AfterOther)
        {Refuse(Why,TEXT("solid initialization changed non-solid native parent/shape/binding/list/storage witness"));return {};}
        for(const auto* Body:C->GetSubBodies())
        {
            if(const auto* Box=Cast<USpeedBranchBoxSubBody>(Body))
            {if(!Box->OwnedSolidValues||!PartialObservation::AppendSolidValues(ActualSolidRecipe,*Box->OwnedSolidValues))return {};}
            else if(const auto* Wheel=Cast<USpeedBranchWheelSubBody>(Body))
            {if(!Wheel->OwnedSolidValues||!PartialObservation::AppendSolidValues(ActualSolidRecipe,*Wheel->OwnedSolidValues))return {};}
            else return {};
        }
        if(OriginalSolidRecipe.IsEmpty()||OriginalSolidRecipe!=ActualSolidRecipe)
        {Refuse(Why,TEXT("new private solid values differ from actual original native setup recipe witness"));return {};}
        N.SolidInitializationWitness=true;
#endif
        C->SetEngineFPS(300);C->SetKinematicState(N.Seed.Initial);
        // Declared fresh-world initial countdown, never a historical restore.
        if(I.Access->RequiresInitialPublication)
        {
            // Closed state, not a timer: no expiry or decrement can release it.
            C->SetCannotMoveLocal();
        }
        else
        {
            C->WheeledPhysicsStateStorage().nbFramesbeforeCanMove=N.Seed.CountdownFrames;
            C->WheeledPhysicsStateStorage().bStartCountdown=N.Seed.CountdownFrames>0;
            C->SinceCanMoveFrameWheeledDomainStorage()=N.Seed.CountdownFrames?INDEX_NONE:0;
            if(N.Seed.CountdownFrames)C->FreezeMovement();else C->UnFreezeMovement();
        }
        C->bPublishWheelPresentationWheeledDomainStorage()=true;I.PinTree(N.Actor);I.PinTree(C);
        return true;
        } catch(const Speed::NativeObjectRead::FUnresolvedOriginal&)
        {return Refuse(Why,TEXT("unresolved original handle during native birth"));}
        }(); // native route/settings/GC scopes have all ended
        I.Diagnostics->End();BirthOutboxActive=false;
        if(!I.Diagnostics->Flush()||!BirthConfigured)return {};
        if(!I.AcquiredGraphStillLive())
        {Refuse(Why,TEXT("original acquired native graph invalidated by birth diagnostic callback"));return {};}
    }
    if(!I.Diagnostics->Begin())return {};
    bool InitializationOutboxActive=true;ON_SCOPE_EXIT{if(InitializationOutboxActive)I.Diagnostics->End();};
    const bool Initialized=[&]() -> bool
    {
    FGCScopeGuard InitializationMathOnly;
    if(!I.AcquiredGraphStillLive())return Refuse(Why,TEXT("acquired graph unavailable before original initialization"));
    FOriginalComponentStorageScope OriginalInitialization(*I.Access);
    Speed::RoadSettings::FOriginalRoadSettingsScope SettingsInitialization(I.DeclaredSettings);
    I.Bridge->ApplyPendingOps();
    for(auto& N:I.Cars)
    {
        N->StableId=I.Bridge->GetSimulationStableId(*N->Component);
        if(!N->StableId){Refuse(Why,TEXT("private native registration identity missing"));return {};}
    }
    if(!NoNetworkOwner(I.World.Get())||!I.Driver->InitializePrivate(*I.Bridge))
    {Refuse(Why,TEXT("private native lifecycle/driver admission failed"));return {};}
    static U64 NextLifetime=1; // GT-only mint at actual successful birth.
    if(NextLifetime==MAX_uint64){Refuse(Why,TEXT("private native lifetime exhausted"));return {};}
    if(!Owner->HasExactRegisteredRoster(Why))return {};
    I.Lifetime=NextLifetime++;
    return true;
    }();
    I.Diagnostics->End();InitializationOutboxActive=false;
    if(!I.Diagnostics->Flush()||!Initialized||!I.AcquiredGraphStillLive())return {};
    {
        FGCScopeGuard FinalBirthReadOnly;
        FOriginalComponentStorageScope OriginalFinalBirth(*I.Access);
        if(!Owner->HasExactRegisteredRoster(Why)||I.Driver->NextPrivate()!=0)return {};
    }
    return Owner;
}
bool FPrivateRoadPhysicsWorld::HasExactRegisteredRoster(FString& Why) const
{
    const auto& I=*Impl;
    if(!I.Bridge)return Refuse(Why,TEXT("private native registry bridge missing"));
    if(!I.OriginalObjectHandlesStillMatch())
        return Refuse(Why,TEXT("original wheel/hitbox handle unresolved, stale or different from actual birth role"));
    // Inspect the actual pending queue under its original lock. This check
    // detects queued/committed topology drift BEFORE private input preparation
    // or physics. It is not a lease excluding later/global native mutations.
    FScopeLock PendingLock(&I.Bridge->PendingCSEpochStorage());
    if(I.Bridge->bCanonicalFrameActiveEpochStorage()||!I.Bridge->PendingOpsEpochStorage().IsEmpty())
        return Refuse(Why,TEXT("private native registry has an active frame or pending mutation"));
    const auto& Actual=I.Bridge->SimulationWorldEpochStorage().GetAdapters();
    if(Actual.Num()!=int32(I.Cars.size()))return Refuse(Why,TEXT("private native registered roster changed"));
    for(const auto& N:I.Cars)
        if(!N->StableId||!Actual.Contains(N->Component)
            ||I.Bridge->SimulationWorldEpochStorage().FindAdapterByStableId(N->StableId)!=N->Component
            ||N->Component->GetPublishedSimulationStableId()!=N->StableId)
            return Refuse(Why,TEXT("private native registered identity changed"));
    return true;
}
bool FPrivateRoadPhysicsWorld::Step(const std::vector<FPrivateRoadRequest>& Requests,bool Resimulation,FString& Why)
{
    Why.Empty();if(OriginalComponentStorageOwner||!IsInGameThread()||Speed::Input::FPresentationInputScope::IsActive())return Refuse(Why,TEXT("private step requires GT physical caller"));
    auto& I=*Impl;std::unique_lock Lock(I.Gate,std::try_to_lock);
    if(!Lock.owns_lock()||I.Closed||I.Failed||Requests.size()!=I.Cars.size())return Refuse(Why,TEXT("private step owner/roster unavailable"));
    if(!I.AcquiredGraphStillLive())return Refuse(Why,TEXT("original acquired graph unavailable before Step"));
    if(!IsValid(I.Driver)||!I.OriginalActorWorldStillMatches(I.Driver,I.World.Get())||I.Driver->HasActorBegunPlay()
        ||I.Driver->IsActorBeingDestroyed())return Refuse(Why,TEXT("private driver lifecycle changed"));
    for(const auto& N:I.Cars)
        if(!IsValid(N->Actor)||!IsValid(N->Component)||N->Actor->IsActorBeingDestroyed()||N->Actor->HasActorBegunPlay()
            ||!I.OriginalActorWorldStillMatches(N->Actor,I.World.Get())||N->Component->GetOwner()!=N->Actor
            ||!I.OriginalMovementStillMatches(*N)||N->Component->GetClass()!=USpeedBranchWheeledComponent::StaticClass())
            return Refuse(Why,TEXT("private native actor/component lifetime changed"));
    for(const auto& Pin:I.Pins)if(!IsValid(Pin.Get()))return Refuse(Why,TEXT("private native graph object invalidated"));
    // Waiting for actual synchronization is a non-mutating admission refusal,
    // not a failed physical step. Refuse BEFORE opening the native stage, so
    // repeated caller attempts cannot retire or erase the original baseline.
    {
        FOriginalComponentStorageScope OriginalStartupRead(*I.Access);
        if(I.Driver->NextPrivate()&&I.Access->RequiresInitialPublication&&!I.Access->InitialPublicationReleased)
            return Refuse(Why,TEXT("awaiting actual initial publication and common activation; no timer release"));
    }
    if(!I.Diagnostics||!I.Diagnostics->Begin())return Refuse(Why,TEXT("native diagnostic owner unavailable"));
    std::shared_ptr<const FPrivateRoadPublication> Completed;
    bool DiagnosticActive=true;ON_SCOPE_EXIT{if(DiagnosticActive)I.Diagnostics->End();};
    bool Advanced=false;try {Advanced=[&]() -> bool
    {
    Speed::RoadSettings::FOriginalRoadSettingsScope SettingsStep(I.DeclaredSettings);
    FGCScopeGuard NativeGCExclusion;
    FOriginalComponentStorageScope OriginalStorage(*I.Access);
    if(!HasExactRegisteredRoster(Why))return false;
    const U64 Frame=I.Driver->NextPrivate();
    if(Frame&&I.Access->RequiresInitialPublication&&!I.Access->InitialPublicationReleased)
        return Refuse(Why,TEXT("awaiting actual initial publication and common activation; no timer release"));
    if(Frame>=uint64(MAX_int32-1)||Chaos::FPhysicsSolverBase::IsNetworkPhysicsPredictionEnabled()||!NoNetworkOwner(I.World.Get()))return Refuse(Why,TEXT("private step native domain changed"));
    std::vector<FBranchInputFilter::FPrepared> Plans;Plans.reserve(I.Cars.size());
    I.Access->Slots.Empty();I.Access->ContactDispatches.clear();I.Access->ContactDispatchOverflow=false;
    ON_SCOPE_EXIT{I.Access->Active=false;I.Access->Slots.Empty();};
    for(const auto& N:I.Cars)
    {
        if(!N->Component||!N->Component->HasIsolatedNativeConstruction())return Refuse(Why,TEXT("private native construction changed before step"));
        const FPrivateRoadRequest* Found=nullptr;
        for(const auto& R:Requests)if(R.Actor==N->Seed.Actor)
        {if(Found||R.Incarnation!=N->Seed.Incarnation)return Refuse(Why,TEXT("private request duplicate/incarnation mismatch"));Found=&R;}
        if(!Found)return Refuse(Why,TEXT("private actor request missing"));
        auto Plan=std::holds_alternative<FPrediction>(Found->Evidence)?N->Filter->PrepareRemote(std::get<FPrediction>(Found->Evidence))
            :N->Filter->PrepareLocal(std::get<std::shared_ptr<const BranchV2::FInputFrame>>(Found->Evidence));
        if(!Plan||!N->Filter->CanCommit(*Plan)||Plan->ReadProcessed()->Frame!=Frame)return Refuse(Why,TEXT("private frame input preparation refused"));
        const auto Processed=Plan->ReadProcessed();
        // This road kernel does not execute menu/reset commands or unknown
        // game actions. Do not silently consume them as successful physics.
        constexpr uint32 Supported=(1u<<0)|(1u<<1)|(1u<<2)|(1u<<5)|(1u<<6);
        if(Processed->ActiveMask&~Supported)return Refuse(Why,TEXT("private road action domain unsupported"));
        for(size_t A=0;A<ActionCount;++A)
            if(!(Supported&(uint32(1)<<A))&&Processed->Requested[A])return Refuse(Why,TEXT("private road action value unsupported"));
        const auto UnsupportedEdge=[&](uint32 Action){return Action>=32||!(Supported&(uint32(1)<<Action));};
        if(const auto* Local=std::get_if<std::shared_ptr<const BranchV2::FInputFrame>>(&Processed->Evidence))
        {for(const auto& E:(*Local)->GetData().Transitions)if(UnsupportedEdge(E.Action))return Refuse(Why,TEXT("private road local command edge unsupported"));}
        if(const auto* Authority=std::get_if<std::shared_ptr<const FPacket>>(&Processed->Evidence))
        {for(const auto& E:(*Authority)->Input.Edges)if(UnsupportedEdge(E.Action))return Refuse(Why,TEXT("private road authority command edge unsupported"));}
        I.Access->Slots.Add(N->Component,{Processed,false,N->Seed.Actor,N->Seed.Incarnation});Plans.push_back(std::move(*Plan));
    }
    I.Access->Frame=Frame;I.Access->Active=true;
    // No rollback claim: any physical failure retires this private branch.
    const auto Fail=[&](const TCHAR* Reason){I.Failed=true;I.Published.store({});return Refuse(Why,Reason);};
    try
    {
    if(!I.Driver->AdvancePrivate(Resimulation))return Fail(TEXT("private canonical native step failed"));
    for(size_t N=0;N<I.Cars.size();++N)
    {
        const auto* Slot=I.Access->Slots.Find(I.Cars[N]->Component);
        if(!Slot||!Slot->Consumed||!I.Cars[N]->Filter->Commit(Plans[N]))return Fail(TEXT("private consumed-input/filter commit failed"));
    }
    auto Out=std::make_shared<FPrivateRoadPublication>();Out->LocalKernelLifetime=I.Lifetime;Out->Frame=Frame;
    Out->AwaitingInitialPublication=I.Access->RequiresInitialPublication&&!I.Access->InitialPublicationReleased;
    if(I.DeclaredSettings)Out->DeclaredSettingsIdentity=I.DeclaredSettings->CanonicalIdentity();
    Out->ContactDispatchObservationComplete=!I.Access->ContactDispatchOverflow;
    std::sort(I.Access->ContactDispatches.begin(),I.Access->ContactDispatches.end(),[](const auto& A,const auto& B)
        {return A.ActorA==B.ActorA?A.ActorB<B.ActorB:A.ActorA<B.ActorA;});
    Out->ContactDispatchPairs=I.Access->ContactDispatches;
    if(!Out->ContactDispatchPairs.empty())
    {
        if(I.VehicleContactFrameCount==MAX_uint64)return Fail(TEXT("native contact diagnostic exhausted"));
        ++I.VehicleContactFrameCount;
    }
    Out->VehicleContactFrameCount=I.VehicleContactFrameCount;
    Out->NativeNetworkComponents=NativeNetworkComponentCount(I.World.Get());
    Out->RenderScenePresent=I.World->Scene!=nullptr;Out->PhysicsScenePresent=I.World->GetPhysicsScene()!=nullptr;
    for(const auto& N:I.Cars)if(AHUD::OnShowDebugInfo.IsBoundToObject(N->Component))++Out->NativeDebugDelegateBindings;
    if(Out->NativeNetworkComponents||Out->NativeDebugDelegateBindings||Out->RenderScenePresent||Out->PhysicsScenePresent)return Fail(TEXT("external network/debug binding appeared in private native graph"));
    if(!I.Driver->ReadLatestSimulationSnapshot(Out->Physics)||Out->Physics.NumFrame!=Frame
        ||Out->Physics.PresentationBodies.Num()!=int32(I.Cars.size()))return Fail(TEXT("private native publication mismatch"));
    // Build from actual ORIGINAL roots, not facade addresses captured by an
    // external caller. Unknown/stale references refuse the observation, and
    // never become a zero value or a complete-state/lifetime certificate.
    OriginalBodyCapture::FReferences BodyReferences;
    bool BodyReferencesComplete=true;
    using BodyRole=OriginalBodyCapture::EReferenceRole;
    const auto BindBodyReference=[&](const void* P,const FImpl::FCar& N,BodyRole Role,uint16 Index=0)
    {return BodyReferences.Add(P,{N.Seed.Actor,N.Seed.Incarnation,uint16(Role),Index});};
    // The sole static source is the actual retained flat-plane recipe and its
    // original weak/pinned component, not a numeric alias assigned to a car.
    const auto& Sources=I.Bridge->AnalyticSourceComponentsEpochStorage();
    const auto* FloorSource=Sources.Find(1);auto* Floor=I.OriginalFloorWeak.Get();
    const auto& Geometry=I.Bridge->AnalyticWorldDataEpochStorage();
    if(!I.AcquiredGraphStillLive()||!Floor||Sources.Num()!=1||!FloorSource||FloorSource->Get()!=Floor
        ||!Geometry.IsValid()||Geometry!=I.OriginalFlatGeometry||Geometry->Planes.Num()!=1
        ||Geometry->Planes[0].SourceId!=1||Geometry->Planes[0].SurfaceId!=2
        ||Geometry->Planes[0].FeatureId!=3||Geometry->Planes[0].PrimitiveId!=4)
        BodyReferencesComplete=false;
    if(BodyReferencesComplete&&!BodyReferences.Add(Floor,{0,0,uint16(BodyRole::StaticSource),0,Geometry->Planes[0].SourceId}))
        BodyReferencesComplete=false;
    for(const auto& N:I.Cars)
    {
        auto* C=N->ComponentWeak.Get();
        if(!BodyReferencesComplete||!C||!N->NativeVehicleRoot||!N->NativeBoxRoot||N->NativeWheelRoots.size()!=4||N->OriginalWheelObjects.Num()!=4)
        {BodyReferencesComplete=false;break;}
        auto* Box=Cast<USpeedBranchBoxSubBody>(C->GetHitboxSubBodyForConfiguration());
        if(!Box||N->NativeBoxRoot->Vehicle!=N->NativeVehicleRoot
            ||C->OriginalWheeledDomain.get()!=N->NativeVehicleRoot->Domain.get()
            ||Box->StorageAccess.lock()!=I.Access
            ||Box->OwnedBodyValues.get()!=std::get<0>(N->NativeBoxRoot->Parts).get()
            ||Box->OwnedSolidValues.get()!=std::get<1>(N->NativeBoxRoot->Parts).get()
            ||Box->OwnedBoxDomain.get()!=std::get<2>(N->NativeBoxRoot->Parts).get()
            ||!BindBodyReference(static_cast<ISpeedComponent*>(C),*N,BodyRole::Component)
            ||!BindBodyReference(static_cast<ISpeedWheeledComponent*>(C),*N,BodyRole::Component)
            ||!BindBodyReference(C,*N,BodyRole::Component)||!BindBodyReference(Box,*N,BodyRole::Box))
        {BodyReferencesComplete=false;break;}
        const auto& Wheels=C->ReadWheelSubBodiesForPhysics();
        const auto& NativeDomain=N->NativeVehicleRoot->Domain;
        if(!NativeDomain||!NativeDomain->VehicleSimulationPT)
        {BodyReferencesComplete=false;break;}
        auto* NativeSimulation=static_cast<USpeedSimulation*>(NativeDomain->VehicleSimulationPT.Get());
        if(NativeSimulation!=N->OriginalNativeSimulation||NativeDomain->SkySimulation!=NativeSimulation)
        {BodyReferencesComplete=false;break;}
        auto* NativeVehicle=NativeSimulation->PVehicle.Get();
        if(Wheels.Num()!=4||!NativeVehicle||NativeVehicle!=N->OriginalNativeVehicle
            ||NativeVehicle->Wheels.Num()!=4||NativeVehicle->Suspension.Num()!=4
            ||NativeVehicle->Wheels.GetData()!=N->OriginalNativeWheelAllocation
            ||NativeVehicle->Suspension.GetData()!=N->OriginalNativeSuspensionAllocation)
        {BodyReferencesComplete=false;break;}
        for(uint16 W=0;W<4;++W)
        {
            const auto& Root=N->NativeWheelRoots[W];
            if(!Root){BodyReferencesComplete=false;break;}
            const auto& Domain=std::get<3>(Root->Parts);
            auto* OriginalWheelObject=N->OriginalWheelObjects[W].Get();
            auto* Body=Cast<USpeedBranchWheelSubBody>(Wheels[W].Get());
            if(Root->Vehicle!=N->NativeVehicleRoot||!Body||Body->OwnedBodyValues.get()!=std::get<0>(Root->Parts).get()
                ||Body->StorageAccess.lock()!=I.Access
                ||Body->OwnedSolidValues.get()!=std::get<1>(Root->Parts).get()
                ||Body->OwnedWheelRuntime.get()!=std::get<2>(Root->Parts).get()
                ||Body->OwnedWheelDomain.get()!=Domain.get()
                ||Body->OwnedSphereDomain.get()!=std::get<4>(Root->Parts).get()
                ||!Domain||!OriginalWheelObject||Domain->ChaosWheel!=OriginalWheelObject
                ||Domain->PWheel!=&NativeVehicle->Wheels[W]||Domain->PSuspension!=&NativeVehicle->Suspension[W]
                ||NativeVehicle->Wheels[W].SetupPtr!=&N->NativeConfiguration->Wheel
                ||NativeVehicle->Suspension[W].SetupPtr!=&N->NativeConfiguration->Suspension
                ||!BindBodyReference(Wheels[W].Get(),*N,BodyRole::Wheel,W)
                ||!BindBodyReference(Domain->ChaosWheel,*N,BodyRole::WheelObject,W)
                ||!BindBodyReference(Domain->PWheel,*N,BodyRole::NativeWheel,W)
                ||!BindBodyReference(Domain->PSuspension,*N,BodyRole::NativeSuspension,W))
            {BodyReferencesComplete=false;break;}
        }
        if(!BodyReferencesComplete)break;
    }
    BodyReferencesComplete=BodyReferencesComplete&&BodyReferences.Seal();
    for(const auto& N:I.Cars)
    {
        auto State=N->Filter->Capture();if(!State||State->Next!=Frame+1)return Fail(TEXT("private filter publication mismatch"));
        Out->Actors.push_back({N->Seed.Actor,N->Seed.Incarnation,N->StableId,*State,I.Access->Slots.FindChecked(N->Component).Input,
            N->Component->CanMove(),N->Component->IsFrozen(),N->Component->WheeledPhysicsStateStorage().nbFramesbeforeCanMove});
        {
            // This code runs inside the actual original storage scope above.
            // Compare to the actual retained fields, not an assumed copy or
            // a caller-supplied certificate. Only diagnostic counts escape.
            const auto* C=N->Component;
            const ISpeedComponent& Interface=*C;
            const unsigned int LocalFrame=static_cast<unsigned int>(Frame);
            const int32 Idx=LocalFrame % SpeedConstants::RecordedHistorySize;
            const SKinematic* FacadeCurrent=&C->BasePhysicsState.Kinematic;
            const SKinematic* FacadeFrame=C->RecordedBaseFrames[Idx]==LocalFrame
                ? &C->RecordedBaseStates[Idx].Kinematic : FacadeCurrent;
            const SKinematic* OriginalCurrent=&C->BasePhysicsStateStorage().Kinematic;
            const SKinematic* OriginalFrame=C->RecordedBaseFramesWheeledDomainStorage()[Idx]==LocalFrame
                ? &C->RecordedBaseStatesWheeledDomainStorage()[Idx].Kinematic : OriginalCurrent;
            const SKinematic* Views[]={&C->GetKinematicState(),&Interface.GetKinematicState(),
                &C->USpeedWheeledComponent::GetKinematicState(),&C->GetKinematicStateForFrame(LocalFrame),
                &Interface.GetKinematicStateForFrame(LocalFrame),&C->USpeedWheeledComponent::GetKinematicStateForFrame(LocalFrame)};
            auto& Observation=Out->Actors.back();
            const void* PhysViews[8][3]={
                {&C->GetPhysRotation(),&Interface.GetPhysRotation(),&C->ISpeedComponent::GetPhysRotation()},
                {&C->GetPhysCOMVelocity(),&Interface.GetPhysCOMVelocity(),&C->ISpeedComponent::GetPhysCOMVelocity()},
                {&C->GetPhysAngularVelocity(),&Interface.GetPhysAngularVelocity(),&C->ISpeedComponent::GetPhysAngularVelocity()},
                {&C->GetPhysAcceleration(),&Interface.GetPhysAcceleration(),&C->ISpeedComponent::GetPhysAcceleration()},
                {&C->GetPhysAngularAcceleration(),&Interface.GetPhysAngularAcceleration(),&C->ISpeedComponent::GetPhysAngularAcceleration()},
                {&C->GetPhysCOM(),&Interface.GetPhysCOM(),&C->USpeedWheeledComponent::GetPhysCOM()},
                {&C->GetPhysCenterOfMassLocal(),&Interface.GetPhysCenterOfMassLocal(),&C->USpeedWheeledComponent::GetPhysCenterOfMassLocal()},
                {&C->GetPhysCOMLocation(),&Interface.GetPhysCOMLocation(),&C->ISpeedComponent::GetPhysCOMLocation()},
            };
            const void* LegacyPhys[]={&FacadeCurrent->Rotation,static_cast<const FVector*>(&FacadeCurrent->Velocity),static_cast<const FVector*>(&FacadeCurrent->AngularVelocity),static_cast<const FVector*>(&FacadeCurrent->Acceleration),static_cast<const FVector*>(&FacadeCurrent->AngularAcceleration),static_cast<const FVector*>(&FacadeCurrent->Location),&C->CenterOfMass,static_cast<const FVector*>(&FacadeCurrent->Location)};
            const void* OriginalPhys[]={&OriginalCurrent->Rotation,static_cast<const FVector*>(&OriginalCurrent->Velocity),static_cast<const FVector*>(&OriginalCurrent->AngularVelocity),static_cast<const FVector*>(&OriginalCurrent->Acceleration),static_cast<const FVector*>(&OriginalCurrent->AngularAcceleration),static_cast<const FVector*>(&OriginalCurrent->Location),&C->CenterOfMassWheeledDomainStorage(),static_cast<const FVector*>(&OriginalCurrent->Location)};
            for(int32 Member=0;Member<8;++Member)for(const void* View:PhysViews[Member])
            {
                ++Observation.PublicPhysReferenceViews;
                if(View==LegacyPhys[Member]&&View!=OriginalPhys[Member])++Observation.PublicPhysReferenceFacadeViews;
                if(View==OriginalPhys[Member])++Observation.PublicPhysReferenceOriginalViews;
            }
            for(int32 Index=0;Index<6;++Index)
            {
                const SKinematic* ExpectedFacade=Index<3?FacadeCurrent:FacadeFrame;
                const SKinematic* ExpectedOriginal=Index<3?OriginalCurrent:OriginalFrame;
                ++Observation.PublicComponentKinematicViews;
                if(Views[Index]==ExpectedFacade && Views[Index]!=ExpectedOriginal)
                    ++Observation.PublicComponentKinematicFacadeViews;
                if(Views[Index]==ExpectedOriginal)++Observation.PublicComponentKinematicOriginalViews;
            }
        }
        if(BodyReferencesComplete)
        {
            try
            {
                OriginalBodyCapture::FWriter W(BodyReferences);
                W.Value(uint32(0x31424749));W.Value(uint16(1));W.Value(N->Seed.Actor);W.Value(N->Seed.Incarnation);
                W.Value(Geometry->SchemaVersion);OriginalBodyCapture::AppendPlane(W,Geometry->Planes[0]);
                const auto AppendPart=[&](const auto& P)
                {if(!P)throw Replay::FFieldError("missing original body root part");OriginalBodyCapture::Append(W,*P);};
                W.Value(uint16(BodyRole::Box));W.Value(uint16(0));
                AppendPart(std::get<0>(N->NativeBoxRoot->Parts));
                AppendPart(std::get<1>(N->NativeBoxRoot->Parts));
                AppendPart(std::get<2>(N->NativeBoxRoot->Parts));
                for(uint16 Index=0;Index<4;++Index)
                {
                    const auto& Root=N->NativeWheelRoots[Index];W.Value(uint16(BodyRole::Wheel));W.Value(Index);
                    AppendPart(std::get<0>(Root->Parts));AppendPart(std::get<1>(Root->Parts));
                    AppendPart(std::get<2>(Root->Parts));AppendPart(std::get<3>(Root->Parts));AppendPart(std::get<4>(Root->Parts));
                }
                Out->Actors.back().OriginalBodyDomainValues=W.Bytes();
                Out->Actors.back().OriginalBodyDomainsAvailable=true;
            }
            catch(const Replay::FFieldError&){} // explicit UNKNOWN, never a full-state admission
        }
        if(BodyReferencesComplete)
        {
            try
            {
                const auto* C=N->ComponentWeak.Get();
                const auto* StaticWorld=I.Bridge->GetStaticCollisionWorld();
                if(!C||!C->OwnedComponentValues||!StaticWorld
                    ||!(StaticWorld->GetImmutableQueryIdentity1075()==I.Access->OriginalQueryIdentity)
                    ||!I.AcquiredGraphStillLive())
                    throw Replay::FFieldError("original component capture graph changed");
                OriginalBodyCapture::FWriter W(BodyReferences);
                W.Value(uint32(0x31434749));W.Value(uint16(1));W.Value(N->Seed.Actor);W.Value(N->Seed.Incarnation);
                OriginalComponentCapture::AppendOwned(W,*C->OwnedComponentValues,N->StableId,
                    StaticWorld,Geometry->Planes[0].SourceId);
                Out->Actors.back().OriginalComponentDomainValues=W.Bytes();
                Out->Actors.back().OriginalComponentDomainAvailable=true;
            }
            catch(const Replay::FFieldError&){} // UNKNOWN, no restore/exclusion authority
        }
        const auto* Hitbox=N->Component->GetHitboxSubBodyForConfiguration();
        if(!Hitbox)return Fail(TEXT("native hitbox diagnostics missing"));
        {
            // Actual public reference identities of the original box and four
            // wheel objects under the real owner scope; no lease is inferred.
            auto ObserveBodyViews=[&](const auto* OriginalBody,const ISweeper& ShapeView)
            {
                if(!OriginalBody)return false;
                const SKinematic* Legacy=&OriginalBody->Kinematics;
                const SKinematic* Original=&Speed::NativeKinematics::FBodyRead::Body(*OriginalBody);
                // Box/sphere and solid sweepers have distinct ISweeper bases.
                // Observe each real subobject through its unambiguous base.
                const ISolidSweeper& SolidView=*OriginalBody;
                const SKinematic* Views[]={&OriginalBody->GetKinematicState(),
                    &OriginalBody->USSubBody::GetKinematicState(),
                    &OriginalBody->USolidSubBody::GetKinematicState(),
                    &SolidView.GetKinematicState(),&ShapeView.GetKinematicState()};
                auto& Observation=Out->Actors.back();
                for(const auto* View:Views)
                {
                    ++Observation.PublicBodyKinematicViews;
                    if(View==Legacy&&View!=Original)++Observation.PublicBodyKinematicFacadeViews;
                    if(View==Original)++Observation.PublicBodyKinematicOriginalViews;
                }
                return true;
            };
            const auto* OriginalBox=Cast<USpeedBranchBoxSubBody>(Hitbox);
            if(!OriginalBox||!ObserveBodyViews(OriginalBox,static_cast<const IBoxSweeper&>(*OriginalBox)))
                return Fail(TEXT("original box public-view diagnostics missing"));
            for(const auto& Entry:N->Component->ReadWheelSubBodiesForPhysics())
            {
                const auto* OriginalWheel=Cast<USpeedBranchWheelSubBody>(Entry.Get());
                if(!OriginalWheel||!ObserveBodyViews(OriginalWheel,static_cast<const ISphereSweeper&>(*OriginalWheel)))
                    return Fail(TEXT("original wheel public-view diagnostics missing"));
            }
        }
        {
            // Real reference identities, even during original routing. Empty
            // hit/constraint values do not certify real contact presence.
            auto& Observation=Out->Actors.back();
            auto ObserveHits=[&](const auto* Body)
            {
                if(!Body)return false;
                const SHitResult* Legacy[]={&Body->CurrentHit,&Body->FutureHit};
                const SHitResult* Native[]={&Speed::NativeKinematics::FBodyRead::CurrentHit(*Body),
                    &Speed::NativeKinematics::FBodyRead::FutureHit(*Body)};
                const SHitResult* Views[]={&Body->GetHit(),&Body->GetFutureHit(),
                    &Body->USSubBody::GetHit(),&Body->USSubBody::GetFutureHit(),
                    &Body->USolidSubBody::GetHit(),&Body->USolidSubBody::GetFutureHit()};
                for(int32 I=0;I<6;++I)
                {
                    ++Observation.PublicContactReferenceViews;
                    if(Views[I]==Legacy[I%2]&&Views[I]!=Native[I%2])++Observation.PublicContactReferenceFacadeViews;
                    if(Views[I]==Native[I%2])++Observation.PublicContactReferenceOriginalViews;
                }
                Observation.PublicContactPredicateChecks+=3;
                if(Body->HasHit()==Legacy[0]->bBlockingHit)++Observation.PublicContactPredicateMatches;
                if(Body->WillHit()==Legacy[1]->bBlockingHit)++Observation.PublicContactPredicateMatches;
                if(Body->GetFutureTOI()==Legacy[1]->TOI)++Observation.PublicContactPredicateMatches;
                return true;
            };
            const auto* Box=Cast<USpeedBranchBoxSubBody>(Hitbox);
            if(!Box||!ObserveHits(Box))return Fail(TEXT("original box contact-view diagnostics missing"));
            for(const auto& Entry:N->Component->ReadWheelSubBodiesForPhysics())
            {
                const auto* Wheel=Cast<USpeedBranchWheelSubBody>(Entry.Get());
                if(!Wheel||!ObserveHits(Wheel))return Fail(TEXT("original wheel contact-view diagnostics missing"));
            }
            const auto* Legacy=&N->Component->PhysicalConstraints;
            const auto* Native=&Speed::NativeKinematics::FComponentRead::Constraints(*N->Component);
            const TArray<FPhysicalContactConstraint>* Views[]={&N->Component->GetPhysicalConstraints(),
                &N->Component->ISpeedComponent::GetPhysicalConstraints()};
            for(const auto* View:Views)
            {
                ++Observation.PublicContactReferenceViews;
                if(View==Legacy&&View!=Native)++Observation.PublicContactReferenceFacadeViews;
                if(View==Native)++Observation.PublicContactReferenceOriginalViews;
            }
        }
        Out->Actors.back().NativeHitboxHalfExtent=Hitbox->GetBoxExtent();
        Out->Actors.back().NativeHitboxCenter=Speed::NativeKinematics::FBodyRead::Body(*Hitbox).Location;
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
        // This is a read-only declared-subset observation, not a complete-state
        // capture through the guarded restore provider. Unknown remains false.
        auto& Observation=Out->Actors.back();
        Observation.NativeSolidInitializationWitness=N->SolidInitializationWitness;
        if(N->Component->CopyActualCacheDiagnosticValues(Observation.NativeSleep,Observation.NativeQuantization))
            PartialObservation::Capture(*N->Component,N->Seed.Actor,N->Seed.Incarnation,
                N->NativeConfiguration->Wheel,N->NativeConfiguration->Suspension,Observation);
        if(Observation.NativePartialValuesAvailable)
        {
            TArray<uint8> RuntimeValues;
            for(const auto& Entry:N->Component->ReadWheelSubBodiesForPhysics())
            {
                const auto* Wheel=Cast<USpeedBranchWheelSubBody>(Entry.Get());
                if(!Wheel||!Wheel->UsesOriginalBodyStorage()||!Wheel->OwnedWheelRuntime
                    ||!PartialObservation::AppendWheelRuntime(RuntimeValues,*Wheel->OwnedWheelRuntime))
                {Observation.NativePartialValuesAvailable=false;break;}
                Observation.NativeWheelRenderRotations.push_back(Wheel->OwnedWheelRuntime->RenderData.RollRotation);
            }
            if(Observation.NativePartialValuesAvailable&&!RuntimeValues.IsEmpty())
                Observation.NativeWheelRuntimeValues.assign(RuntimeValues.GetData(),RuntimeValues.GetData()+RuntimeValues.Num());
            else Observation.NativePartialValuesAvailable=false;
        }
        if(Observation.NativePartialValuesAvailable)
        {
            TArray<uint8> SolidValues;
            for(const auto* Body:N->Component->GetSubBodies())
            {
                const FPrivateRoadSolidValues* Values=nullptr;
                if(const auto* Box=Cast<USpeedBranchBoxSubBody>(Body))Values=Box->UsesOriginalBodyStorage()?Box->OwnedSolidValues.get():nullptr;
                else if(const auto* Wheel=Cast<USpeedBranchWheelSubBody>(Body))Values=Wheel->UsesOriginalBodyStorage()?Wheel->OwnedSolidValues.get():nullptr;
                if(!Values||!PartialObservation::AppendSolidValues(SolidValues,*Values))
                {Observation.NativePartialValuesAvailable=false;break;}
            }
            if(Observation.NativePartialValuesAvailable&&!SolidValues.IsEmpty())
                Observation.NativeSolidValues.assign(SolidValues.GetData(),SolidValues.GetData()+SolidValues.Num());
            else Observation.NativePartialValuesAvailable=false;
        }
#endif
    }
    if(I.Access->RequiresInitialPublication)for(const auto& A:Out->Actors)
    {
        if(Frame==0&&(A.CanMove||!A.Frozen||A.CountdownFrames!=1))return Fail(TEXT("initial publication moved before synchronization"));
        if(Frame>0&&(!A.CanMove||A.Frozen||A.CountdownFrames))return Fail(TEXT("common startup activation not applied"));
    }
    Completed=std::move(Out);return true;
    }
    catch(...) {return Fail(TEXT("private native branch retired after step/publication exception"));}
    }();} catch(...) {Why=TEXT("original native preparation exception");}
    // Original physical stack, routing and GC exclusion have ended.
    I.Diagnostics->End();DiagnosticActive=false;
    bool DiagnosticsComplete=false;
    try {DiagnosticsComplete=I.Diagnostics->Flush();}
    catch(...) {Why=TEXT("original native diagnostic callback exception");}
    const auto Terminal=[&](const TCHAR* Message)
    {I.Failed=true;I.Published.store({});Completed.reset();if(Why.IsEmpty())Why=Message;return false;};
    if(!Advanced||!DiagnosticsComplete||!Completed)return Terminal(TEXT("native stage or diagnostic boundary refused"));
    // No UObject dereference occurs until real strong/weak lifetime validity is
    // rechecked after arbitrary genuine output-device callbacks have returned.
    if(!I.AcquiredGraphStillLive())return Terminal(TEXT("original acquired graph invalidated by actual Step diagnostic callback"));
    for(const auto& Pin:I.Pins)if(!IsValid(Pin.Get()))return Terminal(TEXT("native lifetime invalidated by external diagnostic callback"));
    if(!I.World.IsValid()||I.World->bIsTearingDown||!IsValid(I.Driver)||I.Driver->IsActorBeingDestroyed())
        return Terminal(TEXT("native world/driver retired by external diagnostic callback"));
    for(const auto& N:I.Cars)if(!IsValid(N->Actor)||!IsValid(N->Component)||N->Actor->IsActorBeingDestroyed())
        return Terminal(TEXT("native participant retired by external diagnostic callback"));
    I.Published.store(std::move(Completed));return true;
}
std::shared_ptr<const FPrivateRoadPublication> FPrivateRoadPhysicsWorld::ReadLatest() const noexcept {return Impl->Published.load();}
bool FPrivateRoadPhysicsWorld::ReleaseInitialPublication(const std::shared_ptr<const FPrivateRoadPublication>& Baseline,FString& Why)
{
    Why.Empty();if(OriginalComponentStorageOwner||!IsInGameThread()||Speed::Input::FPresentationInputScope::IsActive())return Refuse(Why,TEXT("startup release requires original GT owner"));
    auto& I=*Impl;std::unique_lock Lock(I.Gate,std::try_to_lock);
    if(!Lock.owns_lock()||I.Closed||I.Failed||I.Access->Active)return Refuse(Why,TEXT("startup original owner unavailable"));
    if(!I.AcquiredGraphStillLive())return Refuse(Why,TEXT("startup original acquired graph unavailable"));
    if(!Lock.owns_lock()||I.Closed||I.Failed||I.Access->Active||!I.Access->RequiresInitialPublication
        ||I.Access->InitialPublicationReleased||!Baseline||Baseline!=I.Published.load()
        ||!Baseline->AwaitingInitialPublication||Baseline->Frame!=0||Baseline->LocalKernelLifetime!=I.Lifetime
        ||!I.World.IsValid()||I.World->bIsTearingDown||!I.Driver||!IsValid(I.Driver)
        ||Baseline->Actors.size()!=I.Cars.size())return Refuse(Why,TEXT("startup original baseline/boundary unavailable"));
    if(!I.Diagnostics||!I.Diagnostics->Begin())return Refuse(Why,TEXT("startup diagnostics unavailable"));
    bool ActivationOutboxActive=true;ON_SCOPE_EXIT{if(ActivationOutboxActive)I.Diagnostics->End();};
    const bool Activated=[&]() -> bool
    {
    FGCScopeGuard ActivationMathOnly;
    FOriginalComponentStorageScope OriginalStorage(*I.Access);
    Speed::RoadSettings::FOriginalRoadSettingsScope SettingsActivation(I.DeclaredSettings);
    if(I.DeclaredSettings&&Baseline->DeclaredSettingsIdentity!=I.DeclaredSettings->CanonicalIdentity())
        return Refuse(Why,TEXT("startup declared settings recipe changed"));
    if(I.Driver->NextPrivate()!=1)return Refuse(Why,TEXT("original startup physical counter changed"));
    if(!HasExactRegisteredRoster(Why))return false;
    for(const auto& Pin:I.Pins)if(!IsValid(Pin.Get()))return Refuse(Why,TEXT("startup native graph invalidated"));
    for(const auto& N:I.Cars)
    {
        const auto* C=N->Component;
        if(!IsValid(N->Actor)||!IsValid(C)||N->Actor->IsActorBeingDestroyed()||N->Actor->HasActorBegunPlay()
            ||!I.OriginalActorWorldStillMatches(N->Actor,I.World.Get())||C->GetOwner()!=N->Actor||!I.OriginalMovementStillMatches(*N)
            ||C->GetClass()!=USpeedBranchWheeledComponent::StaticClass()||!C->HasIsolatedNativeConstruction()||!C->IsFrozen()||C->CanMove()
            ||C->WheeledPhysicsStateStorage().bStartCountdown||C->WheeledPhysicsStateStorage().nbFramesbeforeCanMove!=1)
            return Refuse(Why,TEXT("startup native identity or closed gate changed"));
    }
    // Same established physical countdown-release operations, at a common
    // explicit boundary instead of an arbitrary elapsed frame count.
    for(const auto& N:I.Cars)
    {
        auto* C=N->Component;C->WheeledPhysicsStateStorage().nbFramesbeforeCanMove=0;C->WheeledPhysicsStateStorage().bStartCountdown=false;
        C->SinceCanMoveFrameWheeledDomainStorage()=C->NumFrame()+1;C->bSimTimelineWasCanMoveWheeledDomainStorage()=true;
        C->bSimTimelineHasGroundStateWheeledDomainStorage()=true;C->bSimTimelineWasGroundedWheeledDomainStorage()=C->IsOnTheGround();
        C->ResetWheelTransientStateForCurrentPose(false);C->UnFreezeMovement();
    }
    // Real optional diagnostic edge after physical activation, dispatched by
    // the original outbox outside math/GC/routing and before commit read-back.
    SPEED_NATIVE_LOG(LogTemp,Verbose,TEXT("[IASOriginalActivationCompleted] frame=%llu actors=%u"),I.Driver->NextPrivate(),uint32(I.Cars.size()));
    return true;
    }();
    I.Diagnostics->End();ActivationOutboxActive=false;
    if(!I.Diagnostics->Flush()||!Activated){I.Failed=true;I.Published.store({});return false;}
    const auto StartupFailed=[&](const TCHAR* Reason)
    {I.Failed=true;I.Published.store({});return Refuse(Why,Reason);};
    if(!I.AcquiredGraphStillLive())return StartupFailed(TEXT("startup acquired graph invalidated by actual callback"));
    {
        FGCScopeGuard ActivationReadBackOnly;
        FOriginalComponentStorageScope OriginalActivationReadBack(*I.Access);
        if(!HasExactRegisteredRoster(Why)||I.Driver->NextPrivate()!=1||I.Published.load()!=Baseline)
            return StartupFailed(TEXT("startup original registry/counter/publication changed during callback"));
        for(const auto& N:I.Cars)
        {
            const auto* C=N->ComponentWeak.Get();
            // CanMove must STILL be closed by the synchronization bit here.
            // Check the actual prepared physical state before committing it.
            if(!C||C->CanMove()||C->IsFrozen()||C->WheeledPhysicsStateStorage().bStartCountdown
                ||C->WheeledPhysicsStateStorage().nbFramesbeforeCanMove!=0
                ||C->SinceCanMoveFrameWheeledDomainStorage()!=C->NumFrame()+1)
                return StartupFailed(TEXT("startup actual physical activation failed read-back"));
        }
        I.Access->InitialPublicationReleased=true;
        // Success-only commit, then verify the real combined CanMove result
        // without any callback or physical step between checks.
        for(const auto& N:I.Cars)if(!N->ComponentWeak->CanMove())
        {I.Access->InitialPublicationReleased=false;return StartupFailed(TEXT("startup committed native gate did not open"));}
    }
    return true;
}
std::vector<FNativeConfigurationLifetimeValues> FPrivateRoadPhysicsWorld::ReadNativeConfigurationLifetimeForDiagnostics() const
{
    if(!IsInGameThread())return {};
    auto& I=*Impl;std::unique_lock Lock(I.Gate,std::try_to_lock);if(!Lock.owns_lock())return {};
    std::vector<FNativeConfigurationLifetimeValues> Values;
    for(const auto& Trace:I.NativeConfigurationLifetime)Values.push_back(Trace->Read());
    return Values;
}
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
FPrivateRoadPhysicsWorld::FOriginalWorldReadProbe FPrivateRoadPhysicsWorld::ObserveOriginalWorldReadForDiagnostics()
{
    FOriginalWorldReadProbe Result;
    try
    {
        if(!Impl||OriginalComponentStorageOwner||!IsInGameThread()
            ||Speed::RoadSettings::SelectedOriginalRoadSettings()
            ||Speed::Input::FPresentationInputScope::IsActive())return Result;
        auto& I=*Impl;std::unique_lock Lock(I.Gate,std::try_to_lock);
        if(!Lock.owns_lock()||I.Closed||I.Failed||!I.Access||I.Access->Active
            ||!I.DeclaredSettings||!I.AcquiredGraphStillLive())return Result;
        auto* Expected=I.OriginalWorldWeak.Get();
        if(!Expected)return Result;
        TObjectPtr<UWorld> KnownWorld=Expected;
        {
            FGCScopeGuard KnownWorldReadOnly;
            if(!I.AcquiredGraphStillLive())return Result;
            // Only this real owner can mint its settings scope. This probe never
            // enters FOriginalComponentStorageScope or advances physical state.
            Result.Admitted=true;
            {
                Speed::RoadSettings::FOriginalRoadSettingsScope OwnRecipe(I.DeclaredSettings);
                Result.SameOriginal=KnownWorld.Get()==Expected;
            }
        }
        if(!I.AcquiredGraphStillLive())Result.SameOriginal=false;
        return Result;
    }
    catch(...){return FOriginalWorldReadProbe{};}
}
#endif

bool FPrivateRoadPhysicsWorld::Close()
{
    using RetirementDraft::EPhase;
    if(!IsInGameThread()||OriginalComponentStorageOwner)return false;
    auto& I=*Impl;
    std::unique_lock Lock(I.Gate,std::try_to_lock);
    // This b22 profile has SYNCHRONOUS GT Step, no private worker. This is only
    // exclusion of its real current Step, NOT source/capture/async owner join.
    if(!Lock.owns_lock()||I.Access->Active)return false;
    if(I.RetirementPhase==EPhase::Retired)return true;
    if(I.RetirementPhase!=EPhase::Open)return false; // interrupted is never retired
    I.RetirementPhase=EPhase::AdmissionClosed;
    I.Failed=true; // existing Step and ReleaseInitialPublication now refuse
    I.Published.store({});
    const auto Interrupt=[&]() -> bool {
        I.RetirementPhase=EPhase::Interrupted;
        I.Published.store({});
        // Retain owner, native roots/configs/pins. Caller must not destroy Owner
        // after false. No automatic retry of partly executed callbacks.
        return false;
    };
    // An empty weak identity is only accepted if no corresponding original
    // pointer was acquired. Pointer PRESENCE checks do not dereference a dead
    // UObject and do not convert an interrupted birth into a fake return.
    if((I.OriginalWorldWeak.IsExplicitlyNull()&&I.World.Get()!=nullptr)
        ||(I.OriginalBridgeWeak.IsExplicitlyNull()&&I.Bridge!=nullptr))return Interrupt();
    try {
        // The registry phase still REQUIRES the real callback census. Known
        // diagnostic logs use the actual existing outbox. No stub validation.
        if(!I.Diagnostics||!I.Diagnostics->Begin())return Interrupt();
        bool OutboxActive=true;
        ON_SCOPE_EXIT {if(OutboxActive)I.Diagnostics->End();};
        {
            FGCScopeGuard RegistryOnly; // never extends to Flush/DestroyWorld!
            FOriginalComponentStorageScope OriginalRetirement(*I.Access);
            auto* Bridge=I.OriginalBridgeWeak.Get();
            if(!I.OriginalBridgeWeak.IsExplicitlyNull()&&(!Bridge||Bridge->HasAnyFlags(RF_BeginDestroyed|RF_FinishDestroyed)))return Interrupt();
            for(const auto& Weak:I.OriginalComponentsWeak) {
                auto* C=Weak.Get();
                if(!C||C->HasAnyFlags(RF_BeginDestroyed|RF_FinishDestroyed))return Interrupt();
                if(!Bridge)return Interrupt();
                Bridge->UnregisterSpeedComponent(C);
            }
            if(Bridge)Bridge->ApplyPendingOps(); // actual original call must RETURN
        }
        I.RetirementPhase=EPhase::RegistryReturned;
        I.Access->Active=false;
        I.Access->Slots.Empty();I.Access->World=nullptr;I.Access->Retired=true;
        I.Diagnostics->End();OutboxActive=false;
        // Outbox callbacks may invalidate UObjects or invoke actual GC. C++
        // roots live in FImpl independently. No original route/GC guard held.
        if(!I.Diagnostics->Flush())return Interrupt();
        if(!I.OriginalWorldWeak.IsExplicitlyNull())
        {
            auto* World=I.OriginalWorldWeak.Get();
            if(!World||World->HasAnyFlags(RF_BeginDestroyed|RF_FinishDestroyed))return Interrupt();
            I.RetirementPhase=EPhase::WorldCallbacksEntered;
            World->DestroyWorld(false); // NO FGCScopeGuard around real callbacks
            I.RetirementPhase=EPhase::WorldCallbacksReturned; // actual returned call
        } // null original weak identity means no world was ever constructed
        // Never reuse World/raw I.Driver/raw N.Component after this point.
        // Weak identity/serial + garbage flags resolve each actual survivor.
        {
            FGCScopeGuard DetachOnly;
            for(const auto& Weak:I.OriginalWheelsWeak)if(auto* W=Weak.Get()) {
                if(W->HasAnyFlags(RF_BeginDestroyed|RF_FinishDestroyed))continue;
                W->StorageAccess.reset();
                // These null paths in b22 are assignments/short-circuit only;
                // re-review if their implementations change.
                W->SetWheelSim(nullptr);W->SetSuspensionSim(nullptr);W->SetChaosWheel(nullptr);
                W->OwnedBodyValues.reset();W->OwnedSolidValues.reset();
                W->OwnedWheelRuntime.reset();W->OwnedWheelDomain.reset();W->OwnedSphereDomain.reset();
            }
            for(const auto& Weak:I.OriginalBoxesWeak)if(auto* B=Weak.Get()) {
                if(B->HasAnyFlags(RF_BeginDestroyed|RF_FinishDestroyed))continue;
                B->StorageAccess.reset();B->OwnedBodyValues.reset();
                B->OwnedSolidValues.reset();B->OwnedBoxDomain.reset();
            }
            for(const auto& Weak:I.OriginalComponentsWeak)if(auto* C=Weak.Get()) {
                if(C->HasAnyFlags(RF_BeginDestroyed|RF_FinishDestroyed))continue;
                C->Access.reset();C->OriginalWheeledRoute.reset();
                // NativeCppKeepAlive MUST already hold these original roots.
                C->OriginalWheeledDomain.reset();C->OwnedComponentValues.reset();
                // Reference/facade simulation: preserve original destruction
                // order; its config-pinning wrapper survives through native dtor.
                C->SkySimulationWheeledDomainStorage()=nullptr;
                // Move unique ownership into FCar while the wrapper is alive.
                // Actual native destructor executes later OUTSIDE this GC guard.
                for(auto& N:I.Cars)if(N->ComponentWeak==Weak) {
                    N->RetiringReferenceSimulation=MoveTemp(C->VehicleSimulationPTWheeledDomainStorage());
                    break;
                }
            }
            if(auto* D=I.OriginalDriverWeak.Get()) {
                if(!D->HasAnyFlags(RF_BeginDestroyed|RF_FinishDestroyed)) {
                    D->Access.reset();D->OriginalDriverRoute.reset();D->OriginalDriverStorage.reset();
                }
            }
            if(auto* B=I.OriginalBridgeWeak.Get()) {
                if(!B->HasAnyFlags(RF_BeginDestroyed|RF_FinishDestroyed)) {
                    B->OriginalEpochRoute.reset();B->OriginalEpochBridgeStorage.reset();
                }
            }
        }
        I.RetirementPhase=EPhase::SurvivingFacadesDetached;
        // Concrete typed walk: no UObject read, raw old component, world query,
        // reset helper or synthetic drain boolean participates in this phase.
        for(auto& N:I.Cars) {
            if(!N->NativeVehicleRoot)
            {
                if(N->NativeBoxRoot||!N->NativeWheelRoots.empty())return Interrupt();
                continue; // this actual car had not reached native domain birth
            }
            for(auto& W:N->NativeWheelRoots)
                if(!W||!RetirementDraft::DisconnectWheelRoot(*W,N->NativeVehicleRoot))return Interrupt();
            if(N->NativeBoxRoot&&!RetirementDraft::DisconnectBoxRoot(*N->NativeBoxRoot,N->NativeVehicleRoot))return Interrupt();
        }
        for(auto& N:I.Cars) {
            if(N->NativeVehicleRoot&&!RetirementDraft::ReleaseNativeVehicle(*N->NativeVehicleRoot))return Interrupt();
            N->RetiringReferenceSimulation.Reset(); // no GC exclusion over native dtor
        }
        I.Pins.Empty();I.World.Reset();
        I.Bridge=nullptr;I.Driver=nullptr;I.Cars.clear();
        I.NativeCppKeepAlive.clear(); // actual last-owner native destructors
        I.RetirementPhase=EPhase::NativeReferencesReleased;
        I.Closed=true; // only AFTER real return, detachment, native release
        I.RetirementPhase=EPhase::Retired;
        return true; // finite synchronous profile only, NOT full native/source lease
    } catch(...) {return Interrupt();}
}
}

USpeedBranchBoxSubBody::USpeedBranchBoxSubBody(const FObjectInitializer& Initializer)
    :Super(Initializer),OwnedBodyValues(std::make_shared<Speed::Netcode::Prediction::FPrivateRoadSubBodyValues>()),OwnedSolidValues(std::make_shared<Speed::Netcode::Prediction::FPrivateRoadSolidValues>()),OwnedBoxDomain(std::make_shared<Speed::Netcode::Prediction::FPrivateRoadBoxDomain>()){OwnedBodyValues->SubBodyType=ESubBodyType::Hitbox;}
TWeakObjectPtr<USSubBody> USpeedBranchBoxSubBody::GetPhysicsWeakIdentity() const
{
    if(!UsesOriginalBodyStorage())return Super::GetPhysicsWeakIdentity();
    if(OriginalBodyIdentity.Get()!=this)throw Speed::NativeObjectRead::FUnresolvedOriginal();
    return OriginalBodyIdentity; // real original weak serial, not an index minted during math
}
bool USpeedBranchBoxSubBody::UsesOriginalBodyStorage() const
{
    if(!IsInGameThread()||!Speed::Netcode::Prediction::OriginalComponentStorageOwner)return false;
    const auto A=StorageAccess.lock();
    return OwnedBodyValues&&A&&Speed::Netcode::Prediction::OriginalComponentStorageOwner==A.get();
}
const Speed::FKinematicState& USpeedBranchBoxSubBody::ReadPhysicsSweeperKinematics() const
{
    return Speed::NativeKinematics::FBodyRead::Body(*this);
}

Speed::FKinematicState& USpeedBranchBoxSubBody::KinematicsStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->Kinematics:USSubBody::KinematicsStorage();
}
Speed::FKinematicState const& USpeedBranchBoxSubBody::KinematicsStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->Kinematics:USSubBody::KinematicsStorage();
}
SHitResult& USpeedBranchBoxSubBody::FutureHitStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->FutureHit:USSubBody::FutureHitStorage();
}
SHitResult const& USpeedBranchBoxSubBody::FutureHitStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->FutureHit:USSubBody::FutureHitStorage();
}
SHitResult& USpeedBranchBoxSubBody::CurrentHitStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->CurrentHit:USSubBody::CurrentHitStorage();
}
SHitResult const& USpeedBranchBoxSubBody::CurrentHitStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->CurrentHit:USSubBody::CurrentHitStorage();
}
SHitResult& USpeedBranchBoxSubBody::GroundHitStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->GroundHit:USSubBody::GroundHitStorage();
}
SHitResult const& USpeedBranchBoxSubBody::GroundHitStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->GroundHit:USSubBody::GroundHitStorage();
}
SHitResult& USpeedBranchBoxSubBody::BoxHitStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->BoxHit:USSubBody::BoxHitStorage();
}
SHitResult const& USpeedBranchBoxSubBody::BoxHitStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->BoxHit:USSubBody::BoxHitStorage();
}
SHitResult& USpeedBranchBoxSubBody::SphereHitStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->SphereHit:USSubBody::SphereHitStorage();
}
SHitResult const& USpeedBranchBoxSubBody::SphereHitStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->SphereHit:USSubBody::SphereHitStorage();
}
SHitResult& USpeedBranchBoxSubBody::WheelHitStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->WheelHit:USSubBody::WheelHitStorage();
}
SHitResult const& USpeedBranchBoxSubBody::WheelHitStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->WheelHit:USSubBody::WheelHitStorage();
}
USpeedBranchWheelSubBody::USpeedBranchWheelSubBody(const FObjectInitializer& Initializer)
    :Super(Initializer),
     OwnedBodyValues(std::make_shared<Speed::Netcode::Prediction::FPrivateRoadSubBodyValues>()),
     OwnedSolidValues(std::make_shared<Speed::Netcode::Prediction::FPrivateRoadSolidValues>()),
     OwnedWheelRuntime(std::make_shared<Speed::Netcode::Prediction::FPrivateRoadWheelRuntimeValues>()),
     OwnedWheelDomain(std::make_shared<Speed::Netcode::Prediction::FPrivateRoadWheelDomain>()),
     OwnedSphereDomain(std::make_shared<Speed::Netcode::Prediction::FPrivateRoadSphereDomain>())
{
    OwnedBodyValues->SubBodyType=ESubBodyType::Wheel;
    OwnedBodyValues->IgnoredSubBodyTypes.AddUnique(ESubBodyType::Wheel);
    OwnedSolidValues->bApplyRestForce=false;
    OwnedSolidValues->Mass=USWheelSubBody::DefaultWheelMass;
}
TWeakObjectPtr<USSubBody> USpeedBranchWheelSubBody::GetPhysicsWeakIdentity() const
{
    if(!UsesOriginalBodyStorage())return Super::GetPhysicsWeakIdentity();
    if(OriginalBodyIdentity.Get()!=this)throw Speed::NativeObjectRead::FUnresolvedOriginal();
    return OriginalBodyIdentity; // real original weak serial, not an index minted during math
}
bool USpeedBranchWheelSubBody::UsesOriginalBodyStorage() const
{
    if(!IsInGameThread()||!Speed::Netcode::Prediction::OriginalComponentStorageOwner)return false;
    const auto A=StorageAccess.lock();
    return OwnedBodyValues&&A&&Speed::Netcode::Prediction::OriginalComponentStorageOwner==A.get();
}
const Speed::FKinematicState& USpeedBranchWheelSubBody::ReadPhysicsSweeperKinematics() const
{
    return Speed::NativeKinematics::FBodyRead::Body(*this);
}

Speed::FKinematicState& USpeedBranchWheelSubBody::KinematicsStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->Kinematics:USSubBody::KinematicsStorage();
}
Speed::FKinematicState const& USpeedBranchWheelSubBody::KinematicsStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->Kinematics:USSubBody::KinematicsStorage();
}
SHitResult& USpeedBranchWheelSubBody::FutureHitStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->FutureHit:USSubBody::FutureHitStorage();
}
SHitResult const& USpeedBranchWheelSubBody::FutureHitStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->FutureHit:USSubBody::FutureHitStorage();
}
SHitResult& USpeedBranchWheelSubBody::CurrentHitStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->CurrentHit:USSubBody::CurrentHitStorage();
}
SHitResult const& USpeedBranchWheelSubBody::CurrentHitStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->CurrentHit:USSubBody::CurrentHitStorage();
}
SHitResult& USpeedBranchWheelSubBody::GroundHitStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->GroundHit:USSubBody::GroundHitStorage();
}
SHitResult const& USpeedBranchWheelSubBody::GroundHitStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->GroundHit:USSubBody::GroundHitStorage();
}
SHitResult& USpeedBranchWheelSubBody::BoxHitStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->BoxHit:USSubBody::BoxHitStorage();
}
SHitResult const& USpeedBranchWheelSubBody::BoxHitStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->BoxHit:USSubBody::BoxHitStorage();
}
SHitResult& USpeedBranchWheelSubBody::SphereHitStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->SphereHit:USSubBody::SphereHitStorage();
}
SHitResult const& USpeedBranchWheelSubBody::SphereHitStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->SphereHit:USSubBody::SphereHitStorage();
}
SHitResult& USpeedBranchWheelSubBody::WheelHitStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->WheelHit:USSubBody::WheelHitStorage();
}
SHitResult const& USpeedBranchWheelSubBody::WheelHitStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->WheelHit:USSubBody::WheelHitStorage();
}
float& USpeedBranchWheelSubBody::SuspensionForceRuntimeStorage()
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->SuspensionForce:USWheelSubBody::SuspensionForceRuntimeStorage();
}
float const& USpeedBranchWheelSubBody::SuspensionForceRuntimeStorage() const
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->SuspensionForce:USWheelSubBody::SuspensionForceRuntimeStorage();
}
uint8& USpeedBranchWheelSubBody::bIsJumpingRuntimeStorage()
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->bIsJumping:USWheelSubBody::bIsJumpingRuntimeStorage();
}
uint8 const& USpeedBranchWheelSubBody::bIsJumpingRuntimeStorage() const
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->bIsJumping:USWheelSubBody::bIsJumpingRuntimeStorage();
}
bool& USpeedBranchWheelSubBody::bJumpUnilateralSupportRuntimeStorage()
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->bJumpUnilateralSupport:USWheelSubBody::bJumpUnilateralSupportRuntimeStorage();
}
bool const& USpeedBranchWheelSubBody::bJumpUnilateralSupportRuntimeStorage() const
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->bJumpUnilateralSupport:USWheelSubBody::bJumpUnilateralSupportRuntimeStorage();
}
uint8& USpeedBranchWheelSubBody::ConsecutiveGroundFramesRuntimeStorage()
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->ConsecutiveGroundFrames:USWheelSubBody::ConsecutiveGroundFramesRuntimeStorage();
}
uint8 const& USpeedBranchWheelSubBody::ConsecutiveGroundFramesRuntimeStorage() const
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->ConsecutiveGroundFrames:USWheelSubBody::ConsecutiveGroundFramesRuntimeStorage();
}
float& USpeedBranchWheelSubBody::SteeringAngleRuntimeStorage()
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->SteeringAngle:USWheelSubBody::SteeringAngleRuntimeStorage();
}
float const& USpeedBranchWheelSubBody::SteeringAngleRuntimeStorage() const
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->SteeringAngle:USWheelSubBody::SteeringAngleRuntimeStorage();
}
float& USpeedBranchWheelSubBody::OmegaRuntimeStorage()
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->Omega:USWheelSubBody::OmegaRuntimeStorage();
}
float const& USpeedBranchWheelSubBody::OmegaRuntimeStorage() const
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->Omega:USWheelSubBody::OmegaRuntimeStorage();
}
float& USpeedBranchWheelSubBody::RollAngleRuntimeStorage()
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->RollAngle:USWheelSubBody::RollAngleRuntimeStorage();
}
float const& USpeedBranchWheelSubBody::RollAngleRuntimeStorage() const
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->RollAngle:USWheelSubBody::RollAngleRuntimeStorage();
}
bool& USpeedBranchWheelSubBody::bWasOnGroundPrevFrameRuntimeStorage()
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->bWasOnGroundPrevFrame:USWheelSubBody::bWasOnGroundPrevFrameRuntimeStorage();
}
bool const& USpeedBranchWheelSubBody::bWasOnGroundPrevFrameRuntimeStorage() const
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->bWasOnGroundPrevFrame:USWheelSubBody::bWasOnGroundPrevFrameRuntimeStorage();
}
bool& USpeedBranchWheelSubBody::bCountedGroundFrameRuntimeStorage()
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->bCountedGroundFrame:USWheelSubBody::bCountedGroundFrameRuntimeStorage();
}
bool const& USpeedBranchWheelSubBody::bCountedGroundFrameRuntimeStorage() const
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->bCountedGroundFrame:USWheelSubBody::bCountedGroundFrameRuntimeStorage();
}
bool& USpeedBranchWheelSubBody::bContactVelocityLockedRuntimeStorage()
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->bContactVelocityLocked:USWheelSubBody::bContactVelocityLockedRuntimeStorage();
}
bool const& USpeedBranchWheelSubBody::bContactVelocityLockedRuntimeStorage() const
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->bContactVelocityLocked:USWheelSubBody::bContactVelocityLockedRuntimeStorage();
}
bool& USpeedBranchWheelSubBody::bCrossedIntoCompressionThisFrameRuntimeStorage()
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->bCrossedIntoCompressionThisFrame:USWheelSubBody::bCrossedIntoCompressionThisFrameRuntimeStorage();
}
bool const& USpeedBranchWheelSubBody::bCrossedIntoCompressionThisFrameRuntimeStorage() const
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->bCrossedIntoCompressionThisFrame:USWheelSubBody::bCrossedIntoCompressionThisFrameRuntimeStorage();
}
FVector& USpeedBranchWheelSubBody::ForwardAxisRuntimeStorage()
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->ForwardAxis:USWheelSubBody::ForwardAxisRuntimeStorage();
}
FVector const& USpeedBranchWheelSubBody::ForwardAxisRuntimeStorage() const
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->ForwardAxis:USWheelSubBody::ForwardAxisRuntimeStorage();
}
FVector& USpeedBranchWheelSubBody::RightAxisRuntimeStorage()
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->RightAxis:USWheelSubBody::RightAxisRuntimeStorage();
}
FVector const& USpeedBranchWheelSubBody::RightAxisRuntimeStorage() const
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->RightAxis:USWheelSubBody::RightAxisRuntimeStorage();
}
FVector& USpeedBranchWheelSubBody::UpAxisRuntimeStorage()
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->UpAxis:USWheelSubBody::UpAxisRuntimeStorage();
}
FVector const& USpeedBranchWheelSubBody::UpAxisRuntimeStorage() const
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->UpAxis:USWheelSubBody::UpAxisRuntimeStorage();
}
SWheelRenderData& USpeedBranchWheelSubBody::RenderDataRuntimeStorage()
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->RenderData:USWheelSubBody::RenderDataRuntimeStorage();
}
SWheelRenderData const& USpeedBranchWheelSubBody::RenderDataRuntimeStorage() const
{
    return OwnedWheelRuntime&&UsesOriginalBodyStorage()?OwnedWheelRuntime->RenderData:USWheelSubBody::RenderDataRuntimeStorage();
}
float& USpeedBranchBoxSubBody::MassSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->Mass:USolidSubBody::MassSolidStorage();
}
float const& USpeedBranchBoxSubBody::MassSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->Mass:USolidSubBody::MassSolidStorage();
}
bool& USpeedBranchBoxSubBody::bIsMainSubBodySolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->bIsMainSubBody:USolidSubBody::bIsMainSubBodySolidStorage();
}
bool const& USpeedBranchBoxSubBody::bIsMainSubBodySolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->bIsMainSubBody:USolidSubBody::bIsMainSubBodySolidStorage();
}
bool& USpeedBranchBoxSubBody::bApplyRestForceSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->bApplyRestForce:USolidSubBody::bApplyRestForceSolidStorage();
}
bool const& USpeedBranchBoxSubBody::bApplyRestForceSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->bApplyRestForce:USolidSubBody::bApplyRestForceSolidStorage();
}
bool& USpeedBranchBoxSubBody::EnableFakePhysicsSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->EnableFakePhysics:USolidSubBody::EnableFakePhysicsSolidStorage();
}
bool const& USpeedBranchBoxSubBody::EnableFakePhysicsSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->EnableFakePhysics:USolidSubBody::EnableFakePhysicsSolidStorage();
}
bool& USpeedBranchBoxSubBody::bEnableConstraintOnSubBodyHitSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->bEnableConstraintOnSubBodyHit:USolidSubBody::bEnableConstraintOnSubBodyHitSolidStorage();
}
bool const& USpeedBranchBoxSubBody::bEnableConstraintOnSubBodyHitSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->bEnableConstraintOnSubBodyHit:USolidSubBody::bEnableConstraintOnSubBodyHitSolidStorage();
}
float& USpeedBranchBoxSubBody::RestitutionSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->Restitution:USolidSubBody::RestitutionSolidStorage();
}
float const& USpeedBranchBoxSubBody::RestitutionSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->Restitution:USolidSubBody::RestitutionSolidStorage();
}
float& USpeedBranchBoxSubBody::SphereBoxRestitutionOverrideSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->SphereBoxRestitutionOverride:USolidSubBody::SphereBoxRestitutionOverrideSolidStorage();
}
float const& USpeedBranchBoxSubBody::SphereBoxRestitutionOverrideSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->SphereBoxRestitutionOverride:USolidSubBody::SphereBoxRestitutionOverrideSolidStorage();
}
float& USpeedBranchBoxSubBody::SphereBoxFrictionOverrideSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->SphereBoxFrictionOverride:USolidSubBody::SphereBoxFrictionOverrideSolidStorage();
}
float const& USpeedBranchBoxSubBody::SphereBoxFrictionOverrideSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->SphereBoxFrictionOverride:USolidSubBody::SphereBoxFrictionOverrideSolidStorage();
}
float& USpeedBranchBoxSubBody::SphereBoxManifoldFrictionOverrideSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->SphereBoxManifoldFrictionOverride:USolidSubBody::SphereBoxManifoldFrictionOverrideSolidStorage();
}
float const& USpeedBranchBoxSubBody::SphereBoxManifoldFrictionOverrideSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->SphereBoxManifoldFrictionOverride:USolidSubBody::SphereBoxManifoldFrictionOverrideSolidStorage();
}
bool& USpeedBranchBoxSubBody::bUseCoupledContactImpulseSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->bUseCoupledContactImpulse:USolidSubBody::bUseCoupledContactImpulseSolidStorage();
}
bool const& USpeedBranchBoxSubBody::bUseCoupledContactImpulseSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->bUseCoupledContactImpulse:USolidSubBody::bUseCoupledContactImpulseSolidStorage();
}
bool& USpeedBranchBoxSubBody::bUsePostNormalFrictionImpulseSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->bUsePostNormalFrictionImpulse:USolidSubBody::bUsePostNormalFrictionImpulseSolidStorage();
}
bool const& USpeedBranchBoxSubBody::bUsePostNormalFrictionImpulseSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->bUsePostNormalFrictionImpulse:USolidSubBody::bUsePostNormalFrictionImpulseSolidStorage();
}
bool& USpeedBranchBoxSubBody::bUsePersistentBilateralContactSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->bUsePersistentBilateralContact:USolidSubBody::bUsePersistentBilateralContactSolidStorage();
}
bool const& USpeedBranchBoxSubBody::bUsePersistentBilateralContactSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->bUsePersistentBilateralContact:USolidSubBody::bUsePersistentBilateralContactSolidStorage();
}
float& USpeedBranchBoxSubBody::CoupledContactMaxLinearSpeedSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->CoupledContactMaxLinearSpeed:USolidSubBody::CoupledContactMaxLinearSpeedSolidStorage();
}
float const& USpeedBranchBoxSubBody::CoupledContactMaxLinearSpeedSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->CoupledContactMaxLinearSpeed:USolidSubBody::CoupledContactMaxLinearSpeedSolidStorage();
}
float& USpeedBranchBoxSubBody::CoupledContactMaxAngularSpeedSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->CoupledContactMaxAngularSpeed:USolidSubBody::CoupledContactMaxAngularSpeedSolidStorage();
}
float const& USpeedBranchBoxSubBody::CoupledContactMaxAngularSpeedSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->CoupledContactMaxAngularSpeed:USolidSubBody::CoupledContactMaxAngularSpeedSolidStorage();
}
float& USpeedBranchBoxSubBody::CoupledContactMaxAbsLocalNormalYSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->CoupledContactMaxAbsLocalNormalY:USolidSubBody::CoupledContactMaxAbsLocalNormalYSolidStorage();
}
float const& USpeedBranchBoxSubBody::CoupledContactMaxAbsLocalNormalYSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->CoupledContactMaxAbsLocalNormalY:USolidSubBody::CoupledContactMaxAbsLocalNormalYSolidStorage();
}
float& USpeedBranchBoxSubBody::StaticFrictionSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->StaticFriction:USolidSubBody::StaticFrictionSolidStorage();
}
float const& USpeedBranchBoxSubBody::StaticFrictionSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->StaticFriction:USolidSubBody::StaticFrictionSolidStorage();
}
float& USpeedBranchBoxSubBody::DynamicFrictionSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->DynamicFriction:USolidSubBody::DynamicFrictionSolidStorage();
}
float const& USpeedBranchBoxSubBody::DynamicFrictionSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->DynamicFriction:USolidSubBody::DynamicFrictionSolidStorage();
}
float& USpeedBranchBoxSubBody::ImpactThresholdSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->ImpactThreshold:USolidSubBody::ImpactThresholdSolidStorage();
}
float const& USpeedBranchBoxSubBody::ImpactThresholdSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->ImpactThreshold:USolidSubBody::ImpactThresholdSolidStorage();
}
float& USpeedBranchBoxSubBody::HitDampingSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->HitDamping:USolidSubBody::HitDampingSolidStorage();
}
float const& USpeedBranchBoxSubBody::HitDampingSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->HitDamping:USolidSubBody::HitDampingSolidStorage();
}
FMatrix& USpeedBranchBoxSubBody::InvInertiaLocalSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->InvInertiaLocal:USolidSubBody::InvInertiaLocalSolidStorage();
}
FMatrix const& USpeedBranchBoxSubBody::InvInertiaLocalSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->InvInertiaLocal:USolidSubBody::InvInertiaLocalSolidStorage();
}
int& USpeedBranchBoxSubBody::idxSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->idx:USolidSubBody::idxSolidStorage();
}
int const& USpeedBranchBoxSubBody::idxSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->idx:USolidSubBody::idxSolidStorage();
}
float& USpeedBranchWheelSubBody::MassSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->Mass:USolidSubBody::MassSolidStorage();
}
float const& USpeedBranchWheelSubBody::MassSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->Mass:USolidSubBody::MassSolidStorage();
}
bool& USpeedBranchWheelSubBody::bIsMainSubBodySolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->bIsMainSubBody:USolidSubBody::bIsMainSubBodySolidStorage();
}
bool const& USpeedBranchWheelSubBody::bIsMainSubBodySolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->bIsMainSubBody:USolidSubBody::bIsMainSubBodySolidStorage();
}
bool& USpeedBranchWheelSubBody::bApplyRestForceSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->bApplyRestForce:USolidSubBody::bApplyRestForceSolidStorage();
}
bool const& USpeedBranchWheelSubBody::bApplyRestForceSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->bApplyRestForce:USolidSubBody::bApplyRestForceSolidStorage();
}
bool& USpeedBranchWheelSubBody::EnableFakePhysicsSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->EnableFakePhysics:USolidSubBody::EnableFakePhysicsSolidStorage();
}
bool const& USpeedBranchWheelSubBody::EnableFakePhysicsSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->EnableFakePhysics:USolidSubBody::EnableFakePhysicsSolidStorage();
}
bool& USpeedBranchWheelSubBody::bEnableConstraintOnSubBodyHitSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->bEnableConstraintOnSubBodyHit:USolidSubBody::bEnableConstraintOnSubBodyHitSolidStorage();
}
bool const& USpeedBranchWheelSubBody::bEnableConstraintOnSubBodyHitSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->bEnableConstraintOnSubBodyHit:USolidSubBody::bEnableConstraintOnSubBodyHitSolidStorage();
}
float& USpeedBranchWheelSubBody::RestitutionSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->Restitution:USolidSubBody::RestitutionSolidStorage();
}
float const& USpeedBranchWheelSubBody::RestitutionSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->Restitution:USolidSubBody::RestitutionSolidStorage();
}
float& USpeedBranchWheelSubBody::SphereBoxRestitutionOverrideSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->SphereBoxRestitutionOverride:USolidSubBody::SphereBoxRestitutionOverrideSolidStorage();
}
float const& USpeedBranchWheelSubBody::SphereBoxRestitutionOverrideSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->SphereBoxRestitutionOverride:USolidSubBody::SphereBoxRestitutionOverrideSolidStorage();
}
float& USpeedBranchWheelSubBody::SphereBoxFrictionOverrideSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->SphereBoxFrictionOverride:USolidSubBody::SphereBoxFrictionOverrideSolidStorage();
}
float const& USpeedBranchWheelSubBody::SphereBoxFrictionOverrideSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->SphereBoxFrictionOverride:USolidSubBody::SphereBoxFrictionOverrideSolidStorage();
}
float& USpeedBranchWheelSubBody::SphereBoxManifoldFrictionOverrideSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->SphereBoxManifoldFrictionOverride:USolidSubBody::SphereBoxManifoldFrictionOverrideSolidStorage();
}
float const& USpeedBranchWheelSubBody::SphereBoxManifoldFrictionOverrideSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->SphereBoxManifoldFrictionOverride:USolidSubBody::SphereBoxManifoldFrictionOverrideSolidStorage();
}
bool& USpeedBranchWheelSubBody::bUseCoupledContactImpulseSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->bUseCoupledContactImpulse:USolidSubBody::bUseCoupledContactImpulseSolidStorage();
}
bool const& USpeedBranchWheelSubBody::bUseCoupledContactImpulseSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->bUseCoupledContactImpulse:USolidSubBody::bUseCoupledContactImpulseSolidStorage();
}
bool& USpeedBranchWheelSubBody::bUsePostNormalFrictionImpulseSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->bUsePostNormalFrictionImpulse:USolidSubBody::bUsePostNormalFrictionImpulseSolidStorage();
}
bool const& USpeedBranchWheelSubBody::bUsePostNormalFrictionImpulseSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->bUsePostNormalFrictionImpulse:USolidSubBody::bUsePostNormalFrictionImpulseSolidStorage();
}
bool& USpeedBranchWheelSubBody::bUsePersistentBilateralContactSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->bUsePersistentBilateralContact:USolidSubBody::bUsePersistentBilateralContactSolidStorage();
}
bool const& USpeedBranchWheelSubBody::bUsePersistentBilateralContactSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->bUsePersistentBilateralContact:USolidSubBody::bUsePersistentBilateralContactSolidStorage();
}
float& USpeedBranchWheelSubBody::CoupledContactMaxLinearSpeedSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->CoupledContactMaxLinearSpeed:USolidSubBody::CoupledContactMaxLinearSpeedSolidStorage();
}
float const& USpeedBranchWheelSubBody::CoupledContactMaxLinearSpeedSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->CoupledContactMaxLinearSpeed:USolidSubBody::CoupledContactMaxLinearSpeedSolidStorage();
}
float& USpeedBranchWheelSubBody::CoupledContactMaxAngularSpeedSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->CoupledContactMaxAngularSpeed:USolidSubBody::CoupledContactMaxAngularSpeedSolidStorage();
}
float const& USpeedBranchWheelSubBody::CoupledContactMaxAngularSpeedSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->CoupledContactMaxAngularSpeed:USolidSubBody::CoupledContactMaxAngularSpeedSolidStorage();
}
float& USpeedBranchWheelSubBody::CoupledContactMaxAbsLocalNormalYSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->CoupledContactMaxAbsLocalNormalY:USolidSubBody::CoupledContactMaxAbsLocalNormalYSolidStorage();
}
float const& USpeedBranchWheelSubBody::CoupledContactMaxAbsLocalNormalYSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->CoupledContactMaxAbsLocalNormalY:USolidSubBody::CoupledContactMaxAbsLocalNormalYSolidStorage();
}
float& USpeedBranchWheelSubBody::StaticFrictionSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->StaticFriction:USolidSubBody::StaticFrictionSolidStorage();
}
float const& USpeedBranchWheelSubBody::StaticFrictionSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->StaticFriction:USolidSubBody::StaticFrictionSolidStorage();
}
float& USpeedBranchWheelSubBody::DynamicFrictionSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->DynamicFriction:USolidSubBody::DynamicFrictionSolidStorage();
}
float const& USpeedBranchWheelSubBody::DynamicFrictionSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->DynamicFriction:USolidSubBody::DynamicFrictionSolidStorage();
}
float& USpeedBranchWheelSubBody::ImpactThresholdSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->ImpactThreshold:USolidSubBody::ImpactThresholdSolidStorage();
}
float const& USpeedBranchWheelSubBody::ImpactThresholdSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->ImpactThreshold:USolidSubBody::ImpactThresholdSolidStorage();
}
float& USpeedBranchWheelSubBody::HitDampingSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->HitDamping:USolidSubBody::HitDampingSolidStorage();
}
float const& USpeedBranchWheelSubBody::HitDampingSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->HitDamping:USolidSubBody::HitDampingSolidStorage();
}
FMatrix& USpeedBranchWheelSubBody::InvInertiaLocalSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->InvInertiaLocal:USolidSubBody::InvInertiaLocalSolidStorage();
}
FMatrix const& USpeedBranchWheelSubBody::InvInertiaLocalSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->InvInertiaLocal:USolidSubBody::InvInertiaLocalSolidStorage();
}
int& USpeedBranchWheelSubBody::idxSolidStorage()
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->idx:USolidSubBody::idxSolidStorage();
}
int const& USpeedBranchWheelSubBody::idxSolidStorage() const
{
    return OwnedSolidValues&&UsesOriginalBodyStorage()?OwnedSolidValues->idx:USolidSubBody::idxSolidStorage();
}
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
Speed::Netcode::Prediction::FPrivateRoadSolidValues USpeedBranchBoxSubBody::ReadLegacySolidDiagnosticValues() const
{
    Speed::Netcode::Prediction::FPrivateRoadSolidValues V;
    V.Mass=USolidSubBody::MassSolidStorage();
    V.bIsMainSubBody=USolidSubBody::bIsMainSubBodySolidStorage();
    V.bApplyRestForce=USolidSubBody::bApplyRestForceSolidStorage();
    V.EnableFakePhysics=USolidSubBody::EnableFakePhysicsSolidStorage();
    V.bEnableConstraintOnSubBodyHit=USolidSubBody::bEnableConstraintOnSubBodyHitSolidStorage();
    V.Restitution=USolidSubBody::RestitutionSolidStorage();
    V.SphereBoxRestitutionOverride=USolidSubBody::SphereBoxRestitutionOverrideSolidStorage();
    V.SphereBoxFrictionOverride=USolidSubBody::SphereBoxFrictionOverrideSolidStorage();
    V.SphereBoxManifoldFrictionOverride=USolidSubBody::SphereBoxManifoldFrictionOverrideSolidStorage();
    V.bUseCoupledContactImpulse=USolidSubBody::bUseCoupledContactImpulseSolidStorage();
    V.bUsePostNormalFrictionImpulse=USolidSubBody::bUsePostNormalFrictionImpulseSolidStorage();
    V.bUsePersistentBilateralContact=USolidSubBody::bUsePersistentBilateralContactSolidStorage();
    V.CoupledContactMaxLinearSpeed=USolidSubBody::CoupledContactMaxLinearSpeedSolidStorage();
    V.CoupledContactMaxAngularSpeed=USolidSubBody::CoupledContactMaxAngularSpeedSolidStorage();
    V.CoupledContactMaxAbsLocalNormalY=USolidSubBody::CoupledContactMaxAbsLocalNormalYSolidStorage();
    V.StaticFriction=USolidSubBody::StaticFrictionSolidStorage();
    V.DynamicFriction=USolidSubBody::DynamicFrictionSolidStorage();
    V.ImpactThreshold=USolidSubBody::ImpactThresholdSolidStorage();
    V.HitDamping=USolidSubBody::HitDampingSolidStorage();
    V.InvInertiaLocal=USolidSubBody::InvInertiaLocalSolidStorage();
    V.idx=USolidSubBody::idxSolidStorage();
    return V; // diagnostic copy only, never physical initialization
}
Speed::Netcode::Prediction::FPrivateRoadSolidValues USpeedBranchWheelSubBody::ReadLegacySolidDiagnosticValues() const
{
    Speed::Netcode::Prediction::FPrivateRoadSolidValues V;
    V.Mass=USolidSubBody::MassSolidStorage();
    V.bIsMainSubBody=USolidSubBody::bIsMainSubBodySolidStorage();
    V.bApplyRestForce=USolidSubBody::bApplyRestForceSolidStorage();
    V.EnableFakePhysics=USolidSubBody::EnableFakePhysicsSolidStorage();
    V.bEnableConstraintOnSubBodyHit=USolidSubBody::bEnableConstraintOnSubBodyHitSolidStorage();
    V.Restitution=USolidSubBody::RestitutionSolidStorage();
    V.SphereBoxRestitutionOverride=USolidSubBody::SphereBoxRestitutionOverrideSolidStorage();
    V.SphereBoxFrictionOverride=USolidSubBody::SphereBoxFrictionOverrideSolidStorage();
    V.SphereBoxManifoldFrictionOverride=USolidSubBody::SphereBoxManifoldFrictionOverrideSolidStorage();
    V.bUseCoupledContactImpulse=USolidSubBody::bUseCoupledContactImpulseSolidStorage();
    V.bUsePostNormalFrictionImpulse=USolidSubBody::bUsePostNormalFrictionImpulseSolidStorage();
    V.bUsePersistentBilateralContact=USolidSubBody::bUsePersistentBilateralContactSolidStorage();
    V.CoupledContactMaxLinearSpeed=USolidSubBody::CoupledContactMaxLinearSpeedSolidStorage();
    V.CoupledContactMaxAngularSpeed=USolidSubBody::CoupledContactMaxAngularSpeedSolidStorage();
    V.CoupledContactMaxAbsLocalNormalY=USolidSubBody::CoupledContactMaxAbsLocalNormalYSolidStorage();
    V.StaticFriction=USolidSubBody::StaticFrictionSolidStorage();
    V.DynamicFriction=USolidSubBody::DynamicFrictionSolidStorage();
    V.ImpactThreshold=USolidSubBody::ImpactThresholdSolidStorage();
    V.HitDamping=USolidSubBody::HitDampingSolidStorage();
    V.InvInertiaLocal=USolidSubBody::InvInertiaLocalSolidStorage();
    V.idx=USolidSubBody::idxSolidStorage();
    return V; // diagnostic copy only, never physical initialization
}
#endif
TAtomic<uint64>& USpeedBranchWheeledComponent::PublishedSimulationStableIdStorage()
{
    return UsesOriginalComponentStorage()?OwnedComponentValues->PublishedStableId:ISpeedComponent::PublishedSimulationStableIdStorage();
}
TAtomic<uint64> const& USpeedBranchWheeledComponent::PublishedSimulationStableIdStorage() const
{
    return UsesOriginalComponentStorage()?OwnedComponentValues->PublishedStableId:ISpeedComponent::PublishedSimulationStableIdStorage();
}
USpeedBranchWheeledComponent::USpeedBranchWheeledComponent(const FObjectInitializer& Initializer)
    :Super(Initializer.SetDefaultSubobjectClass<USpeedBranchBoxSubBody>(HitboxName).SetDefaultSubobjectClass<USpeedBranchWheelSubBody>(WheelNames[0]).SetDefaultSubobjectClass<USpeedBranchWheelSubBody>(WheelNames[1]).SetDefaultSubobjectClass<USpeedBranchWheelSubBody>(WheelNames[2]).SetDefaultSubobjectClass<USpeedBranchWheelSubBody>(WheelNames[3]),ESpeedWheeledNetworkConstruction::IAmSpeedOnly)
{
    // Super registers this native object with the global HUD debug multicast.
    // Revoke that unrelated callback before the private factory registers it.
    // Bound-entry removal is not a certificate for every possible escaped view.
    AHUD::OnShowDebugInfo.RemoveAll(this);
    // ASpeedCar deliberately applies its generic road recipe only to the
    // exact generic component class. This specialized native owner explicitly
    // adopts that SAME existing recipe at construction, without retuning it.
    Speed::Driving::ApplyStockRoadCar(*this);
    PrimaryComponentTick.bCanEverTick=false;PrimaryComponentTick.bStartWithTickEnabled=false;
    SetAsyncPhysicsTickEnabled(false);
    // No IAS legacy network DSOs are constructed. The factory also requires
    // Chaos prediction disabled before construction, and censuses zero actual
    // UE NetworkPhysics objects before every private canonical transition.
}
void USpeedBranchWheeledComponent::OnRegister() {UPawnMovementComponent::OnRegister();}
void USpeedBranchWheeledComponent::OnCreatePhysicsState() {UActorComponent::OnCreatePhysicsState();}
void USpeedBranchWheeledComponent::OnDestroyPhysicsState() {UActorComponent::OnDestroyPhysicsState();}
void USpeedBranchWheeledComponent::BeginPlay() {UActorComponent::BeginPlay();}
bool USpeedBranchWheeledComponent::ReadOriginalLegacyGameModePresence() const
{
    const auto A=Access.lock();
    if(!IsInGameThread()||!A||!A->SelectsOriginalStorage()||!A->OriginalWorldHasNoGameMode()
        ||A->FindOriginalComponentAlias(this,GetOwner())!=this)throw Speed::NativeObjectRead::FUnresolvedOriginal();
    return false; // actual resolved absence, same legacy predicate; NOT IAS authority role
}
bool USpeedBranchWheeledComponent::HasIsolatedNativeConstruction() const
{
    // Check actual stored pointers as well as the world's object census: an
    // inherited CDO reference is not an absent native network participant.
    return NetworkConstruction==ESpeedWheeledNetworkConstruction::IAmSpeedOnly
        &&Speed::NativeObjectRead::IsResolvedNull(NetworkPhysicsComponent)
        &&Speed::NativeObjectRead::IsResolvedNull(SNetworkPhysicsComponentWheeledDomainStorage())
        &&Speed::NativeObjectRead::IsResolvedNull(WheeledNetworkPhysicsComponentWheeledDomainStorage())
        &&Speed::NativeObjectRead::IsResolvedNull(SNetworkSettingsWheeledDomainStorage())
        &&Speed::NativeObjectRead::IsResolvedNull(NetDataAssetWheeledDomainStorage())&&!AHUD::OnShowDebugInfo.IsBoundToObject(this)
#if !UE_BUILD_SHIPPING
        &&!ISpeedComponent::IsPhysicsPhaseObservationEnabled() // actual base stored observer, no masked count
#endif
        ;
}
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
bool USpeedBranchWheeledComponent::CopyActualCacheDiagnosticValues(
    Speed::CommonState::FSleepValues& Sleep,Speed::CommonState::FQuantizationValues& Quantization) const
{
    const auto A=Access.lock();
    if(!IsInGameThread()||!A||!A->Active||A->World!=GetWorld()||!A->Slots.Contains(this))return false;
    const auto S=Speed::FCommonCacheValueAccess::Capture(SleepStateStorage());
    const auto Q=Speed::FCommonCacheValueAccess::Capture(KinematicQuantizationCacheStorage());
    if(!Speed::CommonState::ValidCaches(S,Q))return false;
    Sleep=S;Quantization=Q;return true;
}
#endif
bool USpeedBranchWheeledComponent::UsesOriginalComponentStorage() const
{
    if(!IsInGameThread()||!Speed::Netcode::Prediction::OriginalComponentStorageOwner)return false;
    const auto A=Access.lock();
    return OwnedComponentValues&&A&&Speed::Netcode::Prediction::OriginalComponentStorageOwner==A.get();
}
SubBodyConfig USpeedBranchWheeledComponent::GetSubBodyConfig(const USSubBody& SubBody) const
{
    if(!UsesOriginalComponentStorage())return Super::GetSubBodyConfig(SubBody);
    // The exact native base profile returns the same default-invalid config
    // for every subbody, including the hitbox. Its redundant hitbox handle
    // reads do not contribute configuration. Preserve those ordinary facade
    // calls above; original construction uses the unchanged declared value.
    // This result grants no topology, execution, lifetime or restore authority.
    return SubBodyConfig{};
}
void* USpeedBranchWheeledComponent::SuspendStorageRoutingForExternalObservation() const
{
    if(!UsesOriginalComponentStorage())return nullptr;
    auto* Original=const_cast<Speed::Netcode::Prediction::FPrivateRoadAccess*>(
        Speed::Netcode::Prediction::OriginalComponentStorageOwner);
    Speed::Netcode::Prediction::OriginalComponentStorageOwner=nullptr;
    return Original;
}
void USpeedBranchWheeledComponent::ResumeStorageRoutingAfterExternalObservation(void* Token) const
{
    if(!Token)return;
    if(Speed::Netcode::Prediction::OriginalComponentStorageOwner)std::terminate();
    Speed::Netcode::Prediction::OriginalComponentStorageOwner=
        static_cast<const Speed::Netcode::Prediction::FPrivateRoadAccess*>(Token);
}
void USpeedBranchWheeledComponent::EstablishOriginalComponentStorage()
{
    if(OwnedComponentValues)std::terminate(); // once at real birth, never restore
    // Never import a mutable facade value into the physical owner. Default
    // values are native type defaults; the original factory explicitly applies
    // its seed/countdown/freeze, then the real physical steps evolve them.
    OwnedComponentValues=std::make_shared<FOwnedComponentValues>();
}
SBaseGameState& USpeedBranchWheeledComponent::BaseGameStateStorage()
{
    return UsesOriginalComponentStorage()?OwnedComponentValues->BaseGameState:USpeedWheeledComponent::BaseGameStateStorage();
}
const SBaseGameState& USpeedBranchWheeledComponent::BaseGameStateStorage() const
{
    return UsesOriginalComponentStorage()?OwnedComponentValues->BaseGameState:USpeedWheeledComponent::BaseGameStateStorage();
}
const FVector& USpeedBranchWheeledComponent::ReadPhysicsCOM() const
{
    return BasePhysicsStateStorage().Kinematic.Location;
}
const FVector& USpeedBranchWheeledComponent::ReadPhysicsCenterOfMassLocal() const
{
    return CenterOfMassWheeledDomainStorage();
}
const SKinematic& USpeedBranchWheeledComponent::ReadPhysicsKinematicState() const
{
	return BasePhysicsStateStorage().Kinematic;
}

const SKinematic& USpeedBranchWheeledComponent::ReadPhysicsKinematicStateForFrame(const unsigned int& LocalFrame) const
{
	const int32 Idx = LocalFrame % SpeedConstants::RecordedHistorySize;
	if (RecordedBaseFramesWheeledDomainStorage()[Idx] == LocalFrame)
	{
		return RecordedBaseStatesWheeledDomainStorage()[Idx].Kinematic;
	}
	return BasePhysicsStateStorage().Kinematic;
}


FBasePhysicsState& USpeedBranchWheeledComponent::BasePhysicsStateStorage()
{
    return UsesOriginalComponentStorage()?OwnedComponentValues->BasePhysicsState:USpeedWheeledComponent::BasePhysicsStateStorage();
}
const FBasePhysicsState& USpeedBranchWheeledComponent::BasePhysicsStateStorage() const
{
    return UsesOriginalComponentStorage()?OwnedComponentValues->BasePhysicsState:USpeedWheeledComponent::BasePhysicsStateStorage();
}
FWheeledGameState& USpeedBranchWheeledComponent::WheeledGameStateStorage()
{
    return UsesOriginalComponentStorage()?OwnedComponentValues->WheeledGameState:USpeedWheeledComponent::WheeledGameStateStorage();
}
const FWheeledGameState& USpeedBranchWheeledComponent::WheeledGameStateStorage() const
{
    return UsesOriginalComponentStorage()?OwnedComponentValues->WheeledGameState:USpeedWheeledComponent::WheeledGameStateStorage();
}
FWheeledPhysicsState& USpeedBranchWheeledComponent::WheeledPhysicsStateStorage()
{
    return UsesOriginalComponentStorage()?OwnedComponentValues->WheeledPhysicsState:USpeedWheeledComponent::WheeledPhysicsStateStorage();
}
const FWheeledPhysicsState& USpeedBranchWheeledComponent::WheeledPhysicsStateStorage() const
{
    return UsesOriginalComponentStorage()?OwnedComponentValues->WheeledPhysicsState:USpeedWheeledComponent::WheeledPhysicsStateStorage();
}
FWheeledInputState& USpeedBranchWheeledComponent::WheeledPhysicalInputStorage()
{
    return UsesOriginalComponentStorage()?OwnedComponentValues->WheeledPhysicalInput:USpeedWheeledComponent::WheeledPhysicalInputStorage();
}
const FWheeledInputState& USpeedBranchWheeledComponent::WheeledPhysicalInputStorage() const
{
    return UsesOriginalComponentStorage()?OwnedComponentValues->WheeledPhysicalInput:USpeedWheeledComponent::WheeledPhysicalInputStorage();
}
TArray<FPhysicalContactConstraint>& USpeedBranchWheeledComponent::PhysicalConstraintsStorage()
{
    return UsesOriginalComponentStorage()?OwnedComponentValues->PhysicalConstraints:ISpeedComponent::PhysicalConstraintsStorage();
}
TArray<FPhysicalContactConstraint> const& USpeedBranchWheeledComponent::PhysicalConstraintsStorage() const
{
    return UsesOriginalComponentStorage()?OwnedComponentValues->PhysicalConstraints:ISpeedComponent::PhysicalConstraintsStorage();
}
Speed::FSimulationSleepState& USpeedBranchWheeledComponent::SleepStateStorage()
{
    return UsesOriginalComponentStorage()?OwnedComponentValues->SleepState:ISpeedComponent::SleepStateStorage();
}
Speed::FSimulationSleepState const& USpeedBranchWheeledComponent::SleepStateStorage() const
{
    return UsesOriginalComponentStorage()?OwnedComponentValues->SleepState:ISpeedComponent::SleepStateStorage();
}
Speed::FIdentityKinematicQuantizationCache& USpeedBranchWheeledComponent::KinematicQuantizationCacheStorage()
{
    return UsesOriginalComponentStorage()?OwnedComponentValues->KinematicQuantizationCache:ISpeedComponent::KinematicQuantizationCacheStorage();
}
Speed::FIdentityKinematicQuantizationCache const& USpeedBranchWheeledComponent::KinematicQuantizationCacheStorage() const
{
    return UsesOriginalComponentStorage()?OwnedComponentValues->KinematicQuantizationCache:ISpeedComponent::KinematicQuantizationCacheStorage();
}
const Speed::IStaticCollisionWorld*& USpeedBranchWheeledComponent::StaticRestingWorldStorage()
{
    return UsesOriginalComponentStorage()?OwnedComponentValues->StaticRestingWorld:ISpeedComponent::StaticRestingWorldStorage();
}
const Speed::IStaticCollisionWorld* const& USpeedBranchWheeledComponent::StaticRestingWorldStorage() const
{
    return UsesOriginalComponentStorage()?OwnedComponentValues->StaticRestingWorld:ISpeedComponent::StaticRestingWorldStorage();
}
float& USpeedBranchWheeledComponent::StaticSupportFrameHorizonStorage()
{
    return UsesOriginalComponentStorage()?OwnedComponentValues->StaticSupportFrameHorizon:ISpeedComponent::StaticSupportFrameHorizonStorage();
}
float const& USpeedBranchWheeledComponent::StaticSupportFrameHorizonStorage() const
{
    return UsesOriginalComponentStorage()?OwnedComponentValues->StaticSupportFrameHorizon:ISpeedComponent::StaticSupportFrameHorizonStorage();
}
bool USpeedBranchWheeledComponent::IsInitialSynchronizationComplete() const
{
    const auto A=Access.lock();return A&&(!A->RequiresInitialPublication||A->InitialPublicationReleased);
}
void USpeedBranchWheeledComponent::UpdateInputs()
{
    const auto A=Access.lock();bInputFrameRejectedV2WheeledDomainStorage()=true;
    const auto* Slot=A?A->Slots.Find(this):nullptr;
    if(!IsInGameThread()||!A||!A->Active||A->World!=GetWorld()||!Slot||!Slot->Input
        ||Slot->Input->Frame!=A->Frame||uint64(NumFrame())!=A->Frame+1
        ||FrameInputStreamWheeledDomainStorage()||FrameInputStreamV2WheeledDomainStorage()||CanonicalInputSnapshotWheeledDomainStorage()||InputReservationV2WheeledDomainStorage()||ReservedInputFrameV2WheeledDomainStorage()
        ||HasProducedInputAuthority()||HasLegacyRemoteInputAuthority())return;
    const auto& Applied=Slot->Input->Applied;
    bProducedInputFrameOwnedWheeledDomainStorage()=true; // actual frame ownership, no Device authority.
    WheeledPhysicalInputStorage().Throttle=uint8(Applied[0]);WheeledPhysicalInputStorage().Brake=uint8(Applied[1]);WheeledPhysicalInputStorage().Steer=int8(Applied[2]);
    WheeledPhysicalInputStorage().bCanMove=CanMove();
    WheeledPhysicalInputStorage().Camera={};WheeledPhysicalInputStorage().Camera.Flags=1;
    WheeledPhysicalInputStorage().Camera.Yaw=int8(Applied[5]);WheeledPhysicalInputStorage().Camera.Pitch=int8(Applied[6]);
    SyncWheeledPhysicalInputToState();bInputFrameRejectedV2WheeledDomainStorage()=false;
}
const USpeedWorldSubsystem* USpeedBranchBoxSubBody::ResolveEpochBridgeForSweep(UWorld* W,const Speed::IStaticCollisionWorld* StaticWorld) const
{
    if(!UsesOriginalBodyStorage())return Super::ResolveEpochBridgeForSweep(W,StaticWorld);
    const auto A=StorageAccess.lock();
    if(!IsInGameThread()||!A||!A->FindOriginalComponentAlias(GetParentComponent(),GetOwner())
        ||!GetParentComponent()||GetParentComponent()->GetStaticCollisionWorldForFrame()!=StaticWorld)
        throw Speed::NativeObjectRead::FUnresolvedOriginal();
    return A->OriginalBridgeForFrame(W,StaticWorld);
}
void USpeedBranchBoxSubBody::ResolveCurrentHitPrv(const float& Delta,const float& SimTime)
{
    // Capture the genuine dispatch participant before Super can update caches.
    // Weak validity after the synchronous call never invents a lifetime lease.
    const bool Blocking=CurrentHitStorage().bBlockingHit;
    const auto Peer=CurrentHitStorage().Component;
    const bool WasSolid=Cast<USolidSubBody>(Peer.Get())!=nullptr;
    Super::ResolveCurrentHitPrv(Delta,SimTime); // unchanged native solver, once
    if(Blocking&&WasSolid&&Peer.IsValid())
    {
        // ISpeedComponent is native C++, NOT a UObject/UInterface. Resolve
        // through the genuine owning actor, then verify the adjusted native
        // base address rather than performing an invalid Unreal Cast.
        const auto A=StorageAccess.lock();
        auto* Owner=A?A->FindOriginalComponentAlias(GetParentComponent(),GetOwner()):nullptr;
        if(Owner&&GetParentComponent()==static_cast<ISpeedComponent*>(Owner))
            if(auto* SolidPeer=Cast<USolidSubBody>(Peer.Get()))
                Owner->ObserveContactPair(*this,*SolidPeer,Cast<UBoxSubBody>(SolidPeer)?0:1);
    }
}
void USpeedBranchWheeledComponent::RegisterWheelGroundContact(const SWheelGroundContact& Contact)
{
    Super::RegisterWheelGroundContact(Contact); // preserve genuine registration, once
    const auto* Other=Cast<UBoxSubBody>(Contact.SurfaceComponent.Get());
    if(Contact.Wheel&&Other)ObserveContactPair(*Contact.Wheel,*Other,2);
}
void USpeedBranchWheeledComponent::ObserveContactPair(const USSubBody& SubBody,const USSubBody& Other,uint8 Kind)
{
    const auto A=Access.lock();
    if(Kind>2||!IsInGameThread()||!A||!A->Active||A->World!=GetWorld()||SubBody.GetParentComponent()!=this
        ||Other.GetWorld()!=A->World||Other.GetOwner()==GetOwner())return;
    const auto* C=A->FindOriginalComponentAlias(Other.GetParentComponent(),Other.GetOwner());
    if(A->FindOriginalComponentAlias(this,GetOwner())!=this)return;
    if(!C||Other.GetParentComponent()!=static_cast<const ISpeedComponent*>(C))return;
    const auto* Self=A->Slots.Find(this);const auto* Peer=C?A->Slots.Find(C):nullptr;
    if(!Self||!Peer||!Self->Input||!Peer->Input||Self->Input->Frame!=A->Frame||Peer->Input->Frame!=A->Frame
        ||!Self->Actor||!Peer->Actor||Self->Actor==Peer->Actor)return;
    const auto Low=FMath::Min(Self->Actor,Peer->Actor),High=FMath::Max(Self->Actor,Peer->Actor);
    const auto Increment=[&](auto& P)
    {
        uint32& Count=Kind==0?P.Dispatches:(Kind==1?P.BoxWheelDispatches:P.WheelRegistrations);
        if(Count==MAX_uint32)A->ContactDispatchOverflow=true;else ++Count;
    };
    for(auto& P:A->ContactDispatches)if(P.ActorA==Low&&P.ActorB==High){Increment(P);return;}
    // Fixed finite roster has at most120 unordered pairs. Never grow beyond
    // the actual reserved roster capacity inside a canonical physical call.
    if(A->ContactDispatches.size()>=A->ContactDispatches.capacity()){A->ContactDispatchOverflow=true;return;}
    A->ContactDispatches.push_back({Low,High,0,0,0});Increment(A->ContactDispatches.back());
}
bool USpeedBranchWheeledComponent::ValidateCanonicalFrameCommit(uint64 Frame) const
{
    const auto A=Access.lock();const auto* Slot=A?A->Slots.Find(this):nullptr;
    return IsInGameThread()&&A&&A->Active&&A->World==GetWorld()&&A->Frame==Frame&&Slot&&Slot->Input
        &&!Slot->Consumed&&!bInputFrameRejectedV2WheeledDomainStorage()&&PreparedCanonicalInputFrameWheeledDomainStorage().IsSet()&&PreparedCanonicalInputFrameWheeledDomainStorage().GetValue()==Frame;
}
bool USpeedBranchWheeledComponent::CommitCanonicalFrame(uint64 Frame) noexcept
{
    if(!ValidateCanonicalFrameCommit(Frame))return false;Access.lock()->Slots.FindChecked(this).Consumed=true;return true;
}
void USpeedBranchWheeledComponent::AbortCanonicalFrame(uint64,ECanonicalFrameAbortReason) noexcept {bInputFrameRejectedV2WheeledDomainStorage()=true;}
ASpeedBranchCar::ASpeedBranchCar(const FObjectInitializer& Initializer)
    :Super(Initializer.SetDefaultSubobjectClass<USpeedBranchWheeledComponent>(VehicleMovementComponentName))
{PrimaryActorTick.bCanEverTick=false;PrimaryActorTick.bStartWithTickEnabled=false;SetReplicates(false);}
void ASpeedBranchCar::BeginPlay() {AActor::BeginPlay();}
ASpeedBranchSimulation::ASpeedBranchSimulation()
{bAsyncPhysicsTickEnabled=false;PrimaryActorTick.bCanEverTick=false;PrimaryActorTick.bStartWithTickEnabled=false;SetReplicates(false);}
void ASpeedBranchSimulation::BeginPlay() {AActor::BeginPlay();}
UWorld* ASpeedBranchSimulation::ResolveOriginalPhysicalWorld() const
{
    const auto A=Access.lock();
    if(A&&A->SelectsOriginalStorage())
        return A->OriginalDriverBirth.Get()==this&&A->OriginalActorWorldStillMatches(this)?A->World:nullptr;
    return AActor::GetWorld(); // real ordinary Engine facade outside the private route
}
bool ASpeedBranchSimulation::InitializePrivate(USpeedWorldSubsystem& Bridge)
{
    if(!IsInGameThread()||!Access.lock()||GetEngineFPS()!=300)return false;
    SpeedWorldSubsystemDriverStorage()=&Bridge;if(!EnsureSimulationWorldReady())return false;
    SealSimulationInputs();InitializeCanonicalFrame(0);return CanonicalNumFrameDriverStorage()==0;
}
bool ASpeedBranchSimulation::AdvancePrivate(bool Resimulation)
{
    const auto A=Access.lock();
    if(!IsInGameThread()||!A||!A->Active||A->Frame!=CanonicalNumFrameDriverStorage()||A->World!=ResolveOriginalPhysicalWorld())return false;
    FCanonicalFrameContext Context(CanonicalNumFrameDriverStorage());Context.Purpose=Resimulation?ECanonicalFramePurpose::Resimulation:ECanonicalFramePurpose::Forward;
    if(!StepCanonicalFrame(Context))return false;
    ++CanonicalNumFrameDriverStorage();return true; // same success-only counter as RunCanonicalFrames.
}

// All domains below are part of the pending vertical epoch hunk, not a lease.
ISpeedComponent*& USpeedBranchBoxSubBody::ParentComponentBodyDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->ParentComponent:USSubBody::ParentComponentBodyDomainStorage();
}
ISpeedComponent* const& USpeedBranchBoxSubBody::ParentComponentBodyDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->ParentComponent:USSubBody::ParentComponentBodyDomainStorage();
}
USSubBody::ESubBodyType& USpeedBranchBoxSubBody::SubBodyTypeBodyDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->SubBodyType:USSubBody::SubBodyTypeBodyDomainStorage();
}
USSubBody::ESubBodyType const& USpeedBranchBoxSubBody::SubBodyTypeBodyDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->SubBodyType:USSubBody::SubBodyTypeBodyDomainStorage();
}
ECollisionChannel& USpeedBranchBoxSubBody::CollisionChannelBodyDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->CollisionChannel:USSubBody::CollisionChannelBodyDomainStorage();
}
ECollisionChannel const& USpeedBranchBoxSubBody::CollisionChannelBodyDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->CollisionChannel:USSubBody::CollisionChannelBodyDomainStorage();
}
FCollisionResponseParams& USpeedBranchBoxSubBody::ResponseParamsBodyDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->ResponseParams:USSubBody::ResponseParamsBodyDomainStorage();
}
FCollisionResponseParams const& USpeedBranchBoxSubBody::ResponseParamsBodyDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->ResponseParams:USSubBody::ResponseParamsBodyDomainStorage();
}
TArray<UPrimitiveComponent*>& USpeedBranchBoxSubBody::AlwaysIgnoredComponentsBodyDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->AlwaysIgnoredComponents:USSubBody::AlwaysIgnoredComponentsBodyDomainStorage();
}
TArray<UPrimitiveComponent*> const& USpeedBranchBoxSubBody::AlwaysIgnoredComponentsBodyDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->AlwaysIgnoredComponents:USSubBody::AlwaysIgnoredComponentsBodyDomainStorage();
}
TArray<UPrimitiveComponent*>& USpeedBranchBoxSubBody::IgnoredComponentsBodyDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->IgnoredComponents:USSubBody::IgnoredComponentsBodyDomainStorage();
}
TArray<UPrimitiveComponent*> const& USpeedBranchBoxSubBody::IgnoredComponentsBodyDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->IgnoredComponents:USSubBody::IgnoredComponentsBodyDomainStorage();
}
TArray<USSubBody::ESubBodyType>& USpeedBranchBoxSubBody::IgnoredSubBodyTypesBodyDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->IgnoredSubBodyTypes:USSubBody::IgnoredSubBodyTypesBodyDomainStorage();
}
TArray<USSubBody::ESubBodyType> const& USpeedBranchBoxSubBody::IgnoredSubBodyTypesBodyDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->IgnoredSubBodyTypes:USSubBody::IgnoredSubBodyTypesBodyDomainStorage();
}
TArray<TWeakObjectPtr<UBoxSubBody>>& USpeedBranchBoxSubBody::ExternalBoxSubBodiesBodyDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->ExternalBoxSubBodies:USSubBody::ExternalBoxSubBodiesBodyDomainStorage();
}
TArray<TWeakObjectPtr<UBoxSubBody>> const& USpeedBranchBoxSubBody::ExternalBoxSubBodiesBodyDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->ExternalBoxSubBodies:USSubBody::ExternalBoxSubBodiesBodyDomainStorage();
}
TArray<TWeakObjectPtr<USphereSubBody>>& USpeedBranchBoxSubBody::ExternalSphereSubBodiesBodyDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->ExternalSphereSubBodies:USSubBody::ExternalSphereSubBodiesBodyDomainStorage();
}
TArray<TWeakObjectPtr<USphereSubBody>> const& USpeedBranchBoxSubBody::ExternalSphereSubBodiesBodyDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->ExternalSphereSubBodies:USSubBody::ExternalSphereSubBodiesBodyDomainStorage();
}
TArray<TWeakObjectPtr<USWheelSubBody>>& USpeedBranchBoxSubBody::ExternalWheelSubBodiesBodyDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->ExternalWheelSubBodies:USSubBody::ExternalWheelSubBodiesBodyDomainStorage();
}
TArray<TWeakObjectPtr<USWheelSubBody>> const& USpeedBranchBoxSubBody::ExternalWheelSubBodiesBodyDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->ExternalWheelSubBodies:USSubBody::ExternalWheelSubBodiesBodyDomainStorage();
}
TArray<TWeakObjectPtr<UCylinderSubBody>>& USpeedBranchBoxSubBody::ExternalCylinderSubBodiesBodyDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->ExternalCylinderSubBodies:USSubBody::ExternalCylinderSubBodiesBodyDomainStorage();
}
TArray<TWeakObjectPtr<UCylinderSubBody>> const& USpeedBranchBoxSubBody::ExternalCylinderSubBodiesBodyDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->ExternalCylinderSubBodies:USSubBody::ExternalCylinderSubBodiesBodyDomainStorage();
}
FVector& USpeedBranchBoxSubBody::BoxExtentBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->BoxExtent:UBoxSubBody::BoxExtentBoxDomainStorage();
}
FVector const& USpeedBranchBoxSubBody::BoxExtentBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->BoxExtent:UBoxSubBody::BoxExtentBoxDomainStorage();
}
bool& USpeedBranchBoxSubBody::bProductionNonpenetratingSphereContactsBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->bProductionNonpenetratingSphereContacts:UBoxSubBody::bProductionNonpenetratingSphereContactsBoxDomainStorage();
}
bool const& USpeedBranchBoxSubBody::bProductionNonpenetratingSphereContactsBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->bProductionNonpenetratingSphereContacts:UBoxSubBody::bProductionNonpenetratingSphereContactsBoxDomainStorage();
}
TArray<FVector>& USpeedBranchBoxSubBody::CurrentGroundContactsWSBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->CurrentGroundContactsWS:UBoxSubBody::CurrentGroundContactsWSBoxDomainStorage();
}
TArray<FVector> const& USpeedBranchBoxSubBody::CurrentGroundContactsWSBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->CurrentGroundContactsWS:UBoxSubBody::CurrentGroundContactsWSBoxDomainStorage();
}
TArray<FVector>& USpeedBranchBoxSubBody::PreviousFrameGroundContactsWSBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->PreviousFrameGroundContactsWS:UBoxSubBody::PreviousFrameGroundContactsWSBoxDomainStorage();
}
TArray<FVector> const& USpeedBranchBoxSubBody::PreviousFrameGroundContactsWSBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->PreviousFrameGroundContactsWS:UBoxSubBody::PreviousFrameGroundContactsWSBoxDomainStorage();
}
TArray<FVector>& USpeedBranchBoxSubBody::CurrentGroundNormalsWSBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->CurrentGroundNormalsWS:UBoxSubBody::CurrentGroundNormalsWSBoxDomainStorage();
}
TArray<FVector> const& USpeedBranchBoxSubBody::CurrentGroundNormalsWSBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->CurrentGroundNormalsWS:UBoxSubBody::CurrentGroundNormalsWSBoxDomainStorage();
}
TArray<FVector>& USpeedBranchBoxSubBody::PrevGroundContactsLSBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->PrevGroundContactsLS:UBoxSubBody::PrevGroundContactsLSBoxDomainStorage();
}
TArray<FVector> const& USpeedBranchBoxSubBody::PrevGroundContactsLSBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->PrevGroundContactsLS:UBoxSubBody::PrevGroundContactsLSBoxDomainStorage();
}
TArray<FVector>& USpeedBranchBoxSubBody::LatchedEdgeContactsLSBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->LatchedEdgeContactsLS:UBoxSubBody::LatchedEdgeContactsLSBoxDomainStorage();
}
TArray<FVector> const& USpeedBranchBoxSubBody::LatchedEdgeContactsLSBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->LatchedEdgeContactsLS:UBoxSubBody::LatchedEdgeContactsLSBoxDomainStorage();
}
bool& USpeedBranchBoxSubBody::bEdgeSupportLatchedBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->bEdgeSupportLatched:UBoxSubBody::bEdgeSupportLatchedBoxDomainStorage();
}
bool const& USpeedBranchBoxSubBody::bEdgeSupportLatchedBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->bEdgeSupportLatched:UBoxSubBody::bEdgeSupportLatchedBoxDomainStorage();
}
int32& USpeedBranchBoxSubBody::EdgeSupportLatchFrameBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->EdgeSupportLatchFrame:UBoxSubBody::EdgeSupportLatchFrameBoxDomainStorage();
}
int32 const& USpeedBranchBoxSubBody::EdgeSupportLatchFrameBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->EdgeSupportLatchFrame:UBoxSubBody::EdgeSupportLatchFrameBoxDomainStorage();
}
bool& USpeedBranchBoxSubBody::bRoofSurfaceTraversalLatchedBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->bRoofSurfaceTraversalLatched:UBoxSubBody::bRoofSurfaceTraversalLatchedBoxDomainStorage();
}
bool const& USpeedBranchBoxSubBody::bRoofSurfaceTraversalLatchedBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->bRoofSurfaceTraversalLatched:UBoxSubBody::bRoofSurfaceTraversalLatchedBoxDomainStorage();
}
int32& USpeedBranchBoxSubBody::LastRoofSurfaceContactFrameBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->LastRoofSurfaceContactFrame:UBoxSubBody::LastRoofSurfaceContactFrameBoxDomainStorage();
}
int32 const& USpeedBranchBoxSubBody::LastRoofSurfaceContactFrameBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->LastRoofSurfaceContactFrame:UBoxSubBody::LastRoofSurfaceContactFrameBoxDomainStorage();
}
TWeakObjectPtr<UPrimitiveComponent>& USpeedBranchBoxSubBody::RoofSurfaceCompBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->RoofSurfaceComp:UBoxSubBody::RoofSurfaceCompBoxDomainStorage();
}
TWeakObjectPtr<UPrimitiveComponent> const& USpeedBranchBoxSubBody::RoofSurfaceCompBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->RoofSurfaceComp:UBoxSubBody::RoofSurfaceCompBoxDomainStorage();
}
FVector& USpeedBranchBoxSubBody::CompositeGroundNormalBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->CompositeGroundNormal:UBoxSubBody::CompositeGroundNormalBoxDomainStorage();
}
FVector const& USpeedBranchBoxSubBody::CompositeGroundNormalBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->CompositeGroundNormal:UBoxSubBody::CompositeGroundNormalBoxDomainStorage();
}
TMap<TWeakObjectPtr<UPrimitiveComponent>, uint32>& USpeedBranchBoxSubBody::HitCountThisFrameBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->HitCountThisFrame:UBoxSubBody::HitCountThisFrameBoxDomainStorage();
}
TMap<TWeakObjectPtr<UPrimitiveComponent>, uint32> const& USpeedBranchBoxSubBody::HitCountThisFrameBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->HitCountThisFrame:UBoxSubBody::HitCountThisFrameBoxDomainStorage();
}
uint32& USpeedBranchBoxSubBody::MaxHitsPerComponentPerFrameBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->MaxHitsPerComponentPerFrame:UBoxSubBody::MaxHitsPerComponentPerFrameBoxDomainStorage();
}
uint32 const& USpeedBranchBoxSubBody::MaxHitsPerComponentPerFrameBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->MaxHitsPerComponentPerFrame:UBoxSubBody::MaxHitsPerComponentPerFrameBoxDomainStorage();
}
TArray<float>& USpeedBranchBoxSubBody::PrevLambdaNBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->PrevLambdaN:UBoxSubBody::PrevLambdaNBoxDomainStorage();
}
TArray<float> const& USpeedBranchBoxSubBody::PrevLambdaNBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->PrevLambdaN:UBoxSubBody::PrevLambdaNBoxDomainStorage();
}
bool& USpeedBranchBoxSubBody::bGroundContactStableBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->bGroundContactStable:UBoxSubBody::bGroundContactStableBoxDomainStorage();
}
bool const& USpeedBranchBoxSubBody::bGroundContactStableBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->bGroundContactStable:UBoxSubBody::bGroundContactStableBoxDomainStorage();
}
int32& USpeedBranchBoxSubBody::LastResolvedGroundHitFrameBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->LastResolvedGroundHitFrame:UBoxSubBody::LastResolvedGroundHitFrameBoxDomainStorage();
}
int32 const& USpeedBranchBoxSubBody::LastResolvedGroundHitFrameBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->LastResolvedGroundHitFrame:UBoxSubBody::LastResolvedGroundHitFrameBoxDomainStorage();
}
bool& USpeedBranchBoxSubBody::bHasGroundContactBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->bHasGroundContact:UBoxSubBody::bHasGroundContactBoxDomainStorage();
}
bool const& USpeedBranchBoxSubBody::bHasGroundContactBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->bHasGroundContact:UBoxSubBody::bHasGroundContactBoxDomainStorage();
}
bool& USpeedBranchBoxSubBody::bFreshEdgeRecoverCandidateBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->bFreshEdgeRecoverCandidate:UBoxSubBody::bFreshEdgeRecoverCandidateBoxDomainStorage();
}
bool const& USpeedBranchBoxSubBody::bFreshEdgeRecoverCandidateBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->bFreshEdgeRecoverCandidate:UBoxSubBody::bFreshEdgeRecoverCandidateBoxDomainStorage();
}
float& USpeedBranchBoxSubBody::StableTimeBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->StableTime:UBoxSubBody::StableTimeBoxDomainStorage();
}
float const& USpeedBranchBoxSubBody::StableTimeBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->StableTime:UBoxSubBody::StableTimeBoxDomainStorage();
}
bool& USpeedBranchBoxSubBody::bGroundHitFromSweepBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->bGroundHitFromSweep:UBoxSubBody::bGroundHitFromSweepBoxDomainStorage();
}
bool const& USpeedBranchBoxSubBody::bGroundHitFromSweepBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->bGroundHitFromSweep:UBoxSubBody::bGroundHitFromSweepBoxDomainStorage();
}
bool& USpeedBranchBoxSubBody::bGroundPlaneValidBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->bGroundPlaneValid:UBoxSubBody::bGroundPlaneValidBoxDomainStorage();
}
bool const& USpeedBranchBoxSubBody::bGroundPlaneValidBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->bGroundPlaneValid:UBoxSubBody::bGroundPlaneValidBoxDomainStorage();
}
FVector& USpeedBranchBoxSubBody::GroundPlanePointWSBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->GroundPlanePointWS:UBoxSubBody::GroundPlanePointWSBoxDomainStorage();
}
FVector const& USpeedBranchBoxSubBody::GroundPlanePointWSBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->GroundPlanePointWS:UBoxSubBody::GroundPlanePointWSBoxDomainStorage();
}
FVector& USpeedBranchBoxSubBody::GroundPlaneNBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->GroundPlaneN:UBoxSubBody::GroundPlaneNBoxDomainStorage();
}
FVector const& USpeedBranchBoxSubBody::GroundPlaneNBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->GroundPlaneN:UBoxSubBody::GroundPlaneNBoxDomainStorage();
}
float& USpeedBranchBoxSubBody::GroundPlaneDBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->GroundPlaneD:UBoxSubBody::GroundPlaneDBoxDomainStorage();
}
float const& USpeedBranchBoxSubBody::GroundPlaneDBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->GroundPlaneD:UBoxSubBody::GroundPlaneDBoxDomainStorage();
}
TWeakObjectPtr<UPrimitiveComponent>& USpeedBranchBoxSubBody::GroundCompBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->GroundComp:UBoxSubBody::GroundCompBoxDomainStorage();
}
TWeakObjectPtr<UPrimitiveComponent> const& USpeedBranchBoxSubBody::GroundCompBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->GroundComp:UBoxSubBody::GroundCompBoxDomainStorage();
}
uint64& USpeedBranchBoxSubBody::EstablishedSupportSourceIdBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->EstablishedSupportSourceId:UBoxSubBody::EstablishedSupportSourceIdBoxDomainStorage();
}
uint64 const& USpeedBranchBoxSubBody::EstablishedSupportSourceIdBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->EstablishedSupportSourceId:UBoxSubBody::EstablishedSupportSourceIdBoxDomainStorage();
}
uint64& USpeedBranchBoxSubBody::EstablishedSupportSurfaceIdBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->EstablishedSupportSurfaceId:UBoxSubBody::EstablishedSupportSurfaceIdBoxDomainStorage();
}
uint64 const& USpeedBranchBoxSubBody::EstablishedSupportSurfaceIdBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->EstablishedSupportSurfaceId:UBoxSubBody::EstablishedSupportSurfaceIdBoxDomainStorage();
}
int32& USpeedBranchBoxSubBody::FreshSupportClampUntilFrameBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->FreshSupportClampUntilFrame:UBoxSubBody::FreshSupportClampUntilFrameBoxDomainStorage();
}
int32 const& USpeedBranchBoxSubBody::FreshSupportClampUntilFrameBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->FreshSupportClampUntilFrame:UBoxSubBody::FreshSupportClampUntilFrameBoxDomainStorage();
}
FVector& USpeedBranchBoxSubBody::FreshSupportClampNBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->FreshSupportClampN:UBoxSubBody::FreshSupportClampNBoxDomainStorage();
}
FVector const& USpeedBranchBoxSubBody::FreshSupportClampNBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->FreshSupportClampN:UBoxSubBody::FreshSupportClampNBoxDomainStorage();
}
TWeakObjectPtr<UPrimitiveComponent>& USpeedBranchBoxSubBody::FreshSupportClampCompBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->FreshSupportClampComp:UBoxSubBody::FreshSupportClampCompBoxDomainStorage();
}
TWeakObjectPtr<UPrimitiveComponent> const& USpeedBranchBoxSubBody::FreshSupportClampCompBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->FreshSupportClampComp:UBoxSubBody::FreshSupportClampCompBoxDomainStorage();
}
FVector& USpeedBranchBoxSubBody::PrevGroundNormalBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->PrevGroundNormal:UBoxSubBody::PrevGroundNormalBoxDomainStorage();
}
FVector const& USpeedBranchBoxSubBody::PrevGroundNormalBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->PrevGroundNormal:UBoxSubBody::PrevGroundNormalBoxDomainStorage();
}
bool& USpeedBranchBoxSubBody::bHadGroundContactPrevFrameBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->bHadGroundContactPrevFrame:UBoxSubBody::bHadGroundContactPrevFrameBoxDomainStorage();
}
bool const& USpeedBranchBoxSubBody::bHadGroundContactPrevFrameBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->bHadGroundContactPrevFrame:UBoxSubBody::bHadGroundContactPrevFrameBoxDomainStorage();
}
FVector& USpeedBranchBoxSubBody::PrevContactNormalBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->PrevContactNormal:UBoxSubBody::PrevContactNormalBoxDomainStorage();
}
FVector const& USpeedBranchBoxSubBody::PrevContactNormalBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->PrevContactNormal:UBoxSubBody::PrevContactNormalBoxDomainStorage();
}
bool& USpeedBranchBoxSubBody::bHadContactPrevFrameBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->bHadContactPrevFrame:UBoxSubBody::bHadContactPrevFrameBoxDomainStorage();
}
bool const& USpeedBranchBoxSubBody::bHadContactPrevFrameBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->bHadContactPrevFrame:UBoxSubBody::bHadContactPrevFrameBoxDomainStorage();
}
float& USpeedBranchBoxSubBody::MinSlopCmBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->MinSlopCm:UBoxSubBody::MinSlopCmBoxDomainStorage();
}
float const& USpeedBranchBoxSubBody::MinSlopCmBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->MinSlopCm:UBoxSubBody::MinSlopCmBoxDomainStorage();
}
int32& USpeedBranchBoxSubBody::LastWallOrGutterFrameBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->LastWallOrGutterFrame:UBoxSubBody::LastWallOrGutterFrameBoxDomainStorage();
}
int32 const& USpeedBranchBoxSubBody::LastWallOrGutterFrameBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->LastWallOrGutterFrame:UBoxSubBody::LastWallOrGutterFrameBoxDomainStorage();
}
FVector& USpeedBranchBoxSubBody::LastWallOrGutterNBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->LastWallOrGutterN:UBoxSubBody::LastWallOrGutterNBoxDomainStorage();
}
FVector const& USpeedBranchBoxSubBody::LastWallOrGutterNBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->LastWallOrGutterN:UBoxSubBody::LastWallOrGutterNBoxDomainStorage();
}
float& USpeedBranchBoxSubBody::LastWallOrGutterUpDotBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->LastWallOrGutterUpDot:UBoxSubBody::LastWallOrGutterUpDotBoxDomainStorage();
}
float const& USpeedBranchBoxSubBody::LastWallOrGutterUpDotBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->LastWallOrGutterUpDot:UBoxSubBody::LastWallOrGutterUpDotBoxDomainStorage();
}
SHitResult& USpeedBranchBoxSubBody::LastResolvedSphereHitBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->LastResolvedSphereHit:UBoxSubBody::LastResolvedSphereHitBoxDomainStorage();
}
SHitResult const& USpeedBranchBoxSubBody::LastResolvedSphereHitBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->LastResolvedSphereHit:UBoxSubBody::LastResolvedSphereHitBoxDomainStorage();
}
int32& USpeedBranchBoxSubBody::LastResolvedSphereHitFrameBoxDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->LastResolvedSphereHitFrame:UBoxSubBody::LastResolvedSphereHitFrameBoxDomainStorage();
}
int32 const& USpeedBranchBoxSubBody::LastResolvedSphereHitFrameBoxDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBoxDomain->LastResolvedSphereHitFrame:UBoxSubBody::LastResolvedSphereHitFrameBoxDomainStorage();
}
FTransform USpeedBranchBoxSubBody::PhysicalLocalPose() const {return UsesOriginalBodyStorage()?OwnedBodyValues->LocalPose:USSubBody::PhysicalLocalPose();}
void USpeedBranchBoxSubBody::SetPhysicalLocalOffset(const FVector& V) {if(UsesOriginalBodyStorage())OwnedBodyValues->LocalPose.SetLocation(V);else USSubBody::SetPhysicalLocalOffset(V);}
ISpeedComponent*& USpeedBranchWheelSubBody::ParentComponentBodyDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->ParentComponent:USSubBody::ParentComponentBodyDomainStorage();
}
ISpeedComponent* const& USpeedBranchWheelSubBody::ParentComponentBodyDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->ParentComponent:USSubBody::ParentComponentBodyDomainStorage();
}
USSubBody::ESubBodyType& USpeedBranchWheelSubBody::SubBodyTypeBodyDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->SubBodyType:USSubBody::SubBodyTypeBodyDomainStorage();
}
USSubBody::ESubBodyType const& USpeedBranchWheelSubBody::SubBodyTypeBodyDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->SubBodyType:USSubBody::SubBodyTypeBodyDomainStorage();
}
ECollisionChannel& USpeedBranchWheelSubBody::CollisionChannelBodyDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->CollisionChannel:USSubBody::CollisionChannelBodyDomainStorage();
}
ECollisionChannel const& USpeedBranchWheelSubBody::CollisionChannelBodyDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->CollisionChannel:USSubBody::CollisionChannelBodyDomainStorage();
}
FCollisionResponseParams& USpeedBranchWheelSubBody::ResponseParamsBodyDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->ResponseParams:USSubBody::ResponseParamsBodyDomainStorage();
}
FCollisionResponseParams const& USpeedBranchWheelSubBody::ResponseParamsBodyDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->ResponseParams:USSubBody::ResponseParamsBodyDomainStorage();
}
TArray<UPrimitiveComponent*>& USpeedBranchWheelSubBody::AlwaysIgnoredComponentsBodyDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->AlwaysIgnoredComponents:USSubBody::AlwaysIgnoredComponentsBodyDomainStorage();
}
TArray<UPrimitiveComponent*> const& USpeedBranchWheelSubBody::AlwaysIgnoredComponentsBodyDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->AlwaysIgnoredComponents:USSubBody::AlwaysIgnoredComponentsBodyDomainStorage();
}
TArray<UPrimitiveComponent*>& USpeedBranchWheelSubBody::IgnoredComponentsBodyDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->IgnoredComponents:USSubBody::IgnoredComponentsBodyDomainStorage();
}
TArray<UPrimitiveComponent*> const& USpeedBranchWheelSubBody::IgnoredComponentsBodyDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->IgnoredComponents:USSubBody::IgnoredComponentsBodyDomainStorage();
}
TArray<USSubBody::ESubBodyType>& USpeedBranchWheelSubBody::IgnoredSubBodyTypesBodyDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->IgnoredSubBodyTypes:USSubBody::IgnoredSubBodyTypesBodyDomainStorage();
}
TArray<USSubBody::ESubBodyType> const& USpeedBranchWheelSubBody::IgnoredSubBodyTypesBodyDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->IgnoredSubBodyTypes:USSubBody::IgnoredSubBodyTypesBodyDomainStorage();
}
TArray<TWeakObjectPtr<UBoxSubBody>>& USpeedBranchWheelSubBody::ExternalBoxSubBodiesBodyDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->ExternalBoxSubBodies:USSubBody::ExternalBoxSubBodiesBodyDomainStorage();
}
TArray<TWeakObjectPtr<UBoxSubBody>> const& USpeedBranchWheelSubBody::ExternalBoxSubBodiesBodyDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->ExternalBoxSubBodies:USSubBody::ExternalBoxSubBodiesBodyDomainStorage();
}
TArray<TWeakObjectPtr<USphereSubBody>>& USpeedBranchWheelSubBody::ExternalSphereSubBodiesBodyDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->ExternalSphereSubBodies:USSubBody::ExternalSphereSubBodiesBodyDomainStorage();
}
TArray<TWeakObjectPtr<USphereSubBody>> const& USpeedBranchWheelSubBody::ExternalSphereSubBodiesBodyDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->ExternalSphereSubBodies:USSubBody::ExternalSphereSubBodiesBodyDomainStorage();
}
TArray<TWeakObjectPtr<USWheelSubBody>>& USpeedBranchWheelSubBody::ExternalWheelSubBodiesBodyDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->ExternalWheelSubBodies:USSubBody::ExternalWheelSubBodiesBodyDomainStorage();
}
TArray<TWeakObjectPtr<USWheelSubBody>> const& USpeedBranchWheelSubBody::ExternalWheelSubBodiesBodyDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->ExternalWheelSubBodies:USSubBody::ExternalWheelSubBodiesBodyDomainStorage();
}
TArray<TWeakObjectPtr<UCylinderSubBody>>& USpeedBranchWheelSubBody::ExternalCylinderSubBodiesBodyDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->ExternalCylinderSubBodies:USSubBody::ExternalCylinderSubBodiesBodyDomainStorage();
}
TArray<TWeakObjectPtr<UCylinderSubBody>> const& USpeedBranchWheelSubBody::ExternalCylinderSubBodiesBodyDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedBodyValues->ExternalCylinderSubBodies:USSubBody::ExternalCylinderSubBodiesBodyDomainStorage();
}
ISpeedWheeledComponent*& USpeedBranchWheelSubBody::WheelComponentWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->WheelComponent:USWheelSubBody::WheelComponentWheelDomainStorage();
}
ISpeedWheeledComponent* const& USpeedBranchWheelSubBody::WheelComponentWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->WheelComponent:USWheelSubBody::WheelComponentWheelDomainStorage();
}
UChaosVehicleWheel*& USpeedBranchWheelSubBody::ChaosWheelWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->ChaosWheel:USWheelSubBody::ChaosWheelWheelDomainStorage();
}
UChaosVehicleWheel* const& USpeedBranchWheelSubBody::ChaosWheelWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->ChaosWheel:USWheelSubBody::ChaosWheelWheelDomainStorage();
}
Chaos::FSimpleWheelSim*& USpeedBranchWheelSubBody::PWheelWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->PWheel:USWheelSubBody::PWheelWheelDomainStorage();
}
Chaos::FSimpleWheelSim* const& USpeedBranchWheelSubBody::PWheelWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->PWheel:USWheelSubBody::PWheelWheelDomainStorage();
}
Chaos::FSimpleSuspensionSim*& USpeedBranchWheelSubBody::PSuspensionWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->PSuspension:USWheelSubBody::PSuspensionWheelDomainStorage();
}
Chaos::FSimpleSuspensionSim* const& USpeedBranchWheelSubBody::PSuspensionWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->PSuspension:USWheelSubBody::PSuspensionWheelDomainStorage();
}
float& USpeedBranchWheelSubBody::WheelMassWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->WheelMass:USWheelSubBody::WheelMassWheelDomainStorage();
}
float const& USpeedBranchWheelSubBody::WheelMassWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->WheelMass:USWheelSubBody::WheelMassWheelDomainStorage();
}
float& USpeedBranchWheelSubBody::SuspensionSpringRateWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->SuspensionSpringRate:USWheelSubBody::SuspensionSpringRateWheelDomainStorage();
}
float const& USpeedBranchWheelSubBody::SuspensionSpringRateWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->SuspensionSpringRate:USWheelSubBody::SuspensionSpringRateWheelDomainStorage();
}
float& USpeedBranchWheelSubBody::SuspensionDampingCompressionRatioWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->SuspensionDampingCompressionRatio:USWheelSubBody::SuspensionDampingCompressionRatioWheelDomainStorage();
}
float const& USpeedBranchWheelSubBody::SuspensionDampingCompressionRatioWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->SuspensionDampingCompressionRatio:USWheelSubBody::SuspensionDampingCompressionRatioWheelDomainStorage();
}
float& USpeedBranchWheelSubBody::SuspensionDampingReboundRatioWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->SuspensionDampingReboundRatio:USWheelSubBody::SuspensionDampingReboundRatioWheelDomainStorage();
}
float const& USpeedBranchWheelSubBody::SuspensionDampingReboundRatioWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->SuspensionDampingReboundRatio:USWheelSubBody::SuspensionDampingReboundRatioWheelDomainStorage();
}
bool& USpeedBranchWheelSubBody::bUseSuspensionForceModelOverrideWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->bUseSuspensionForceModelOverride:USWheelSubBody::bUseSuspensionForceModelOverrideWheelDomainStorage();
}
bool const& USpeedBranchWheelSubBody::bUseSuspensionForceModelOverrideWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->bUseSuspensionForceModelOverride:USWheelSubBody::bUseSuspensionForceModelOverrideWheelDomainStorage();
}
bool& USpeedBranchWheelSubBody::bClampSuspensionForceToPositiveWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->bClampSuspensionForceToPositive:USWheelSubBody::bClampSuspensionForceToPositiveWheelDomainStorage();
}
bool const& USpeedBranchWheelSubBody::bClampSuspensionForceToPositiveWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->bClampSuspensionForceToPositive:USWheelSubBody::bClampSuspensionForceToPositiveWheelDomainStorage();
}
bool& USpeedBranchWheelSubBody::bClampPositiveUntilFirstCompressionWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->bClampPositiveUntilFirstCompression:USWheelSubBody::bClampPositiveUntilFirstCompressionWheelDomainStorage();
}
bool const& USpeedBranchWheelSubBody::bClampPositiveUntilFirstCompressionWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->bClampPositiveUntilFirstCompression:USWheelSubBody::bClampPositiveUntilFirstCompressionWheelDomainStorage();
}
bool& USpeedBranchWheelSubBody::bUseEffectiveSuspensionSweepRadiusWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->bUseEffectiveSuspensionSweepRadius:USWheelSubBody::bUseEffectiveSuspensionSweepRadiusWheelDomainStorage();
}
bool const& USpeedBranchWheelSubBody::bUseEffectiveSuspensionSweepRadiusWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->bUseEffectiveSuspensionSweepRadius:USWheelSubBody::bUseEffectiveSuspensionSweepRadiusWheelDomainStorage();
}
bool& USpeedBranchWheelSubBody::bUseIncreasingDisplacementAsCompressionWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->bUseIncreasingDisplacementAsCompression:USWheelSubBody::bUseIncreasingDisplacementAsCompressionWheelDomainStorage();
}
bool const& USpeedBranchWheelSubBody::bUseIncreasingDisplacementAsCompressionWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->bUseIncreasingDisplacementAsCompression:USWheelSubBody::bUseIncreasingDisplacementAsCompressionWheelDomainStorage();
}
bool& USpeedBranchWheelSubBody::bUseCompressionCrossingContactImpulseWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->bUseCompressionCrossingContactImpulse:USWheelSubBody::bUseCompressionCrossingContactImpulseWheelDomainStorage();
}
bool const& USpeedBranchWheelSubBody::bUseCompressionCrossingContactImpulseWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->bUseCompressionCrossingContactImpulse:USWheelSubBody::bUseCompressionCrossingContactImpulseWheelDomainStorage();
}
uint8& USpeedBranchWheelSubBody::GroundForceApplicationModeWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->GroundForceApplicationMode:USWheelSubBody::GroundForceApplicationModeWheelDomainStorage();
}
uint8 const& USpeedBranchWheelSubBody::GroundForceApplicationModeWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->GroundForceApplicationMode:USWheelSubBody::GroundForceApplicationModeWheelDomainStorage();
}
uint8& USpeedBranchWheelSubBody::ContactImpulseApplicationModeWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->ContactImpulseApplicationMode:USWheelSubBody::ContactImpulseApplicationModeWheelDomainStorage();
}
uint8 const& USpeedBranchWheelSubBody::ContactImpulseApplicationModeWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->ContactImpulseApplicationMode:USWheelSubBody::ContactImpulseApplicationModeWheelDomainStorage();
}
float& USpeedBranchWheelSubBody::SuspensionStiffnessForceScaleWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->SuspensionStiffnessForceScale:USWheelSubBody::SuspensionStiffnessForceScaleWheelDomainStorage();
}
float const& USpeedBranchWheelSubBody::SuspensionStiffnessForceScaleWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->SuspensionStiffnessForceScale:USWheelSubBody::SuspensionStiffnessForceScaleWheelDomainStorage();
}
float& USpeedBranchWheelSubBody::SuspensionStiffnessReferenceDisplacementWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->SuspensionStiffnessReferenceDisplacement:USWheelSubBody::SuspensionStiffnessReferenceDisplacementWheelDomainStorage();
}
float const& USpeedBranchWheelSubBody::SuspensionStiffnessReferenceDisplacementWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->SuspensionStiffnessReferenceDisplacement:USWheelSubBody::SuspensionStiffnessReferenceDisplacementWheelDomainStorage();
}
float& USpeedBranchWheelSubBody::SuspensionPreRestCompressionDampingScaleWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->SuspensionPreRestCompressionDampingScale:USWheelSubBody::SuspensionPreRestCompressionDampingScaleWheelDomainStorage();
}
float const& USpeedBranchWheelSubBody::SuspensionPreRestCompressionDampingScaleWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->SuspensionPreRestCompressionDampingScale:USWheelSubBody::SuspensionPreRestCompressionDampingScaleWheelDomainStorage();
}
bool& USpeedBranchWheelSubBody::bClampNegativeSuspensionDisplacementWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->bClampNegativeSuspensionDisplacement:USWheelSubBody::bClampNegativeSuspensionDisplacementWheelDomainStorage();
}
bool const& USpeedBranchWheelSubBody::bClampNegativeSuspensionDisplacementWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->bClampNegativeSuspensionDisplacement:USWheelSubBody::bClampNegativeSuspensionDisplacementWheelDomainStorage();
}
bool& USpeedBranchWheelSubBody::bDisableSuspensionBumpStopForceWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->bDisableSuspensionBumpStopForce:USWheelSubBody::bDisableSuspensionBumpStopForceWheelDomainStorage();
}
bool const& USpeedBranchWheelSubBody::bDisableSuspensionBumpStopForceWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->bDisableSuspensionBumpStopForce:USWheelSubBody::bDisableSuspensionBumpStopForceWheelDomainStorage();
}
float& USpeedBranchWheelSubBody::SuspensionSpringRateCmOverrideWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->SuspensionSpringRateCmOverride:USWheelSubBody::SuspensionSpringRateCmOverrideWheelDomainStorage();
}
float const& USpeedBranchWheelSubBody::SuspensionSpringRateCmOverrideWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->SuspensionSpringRateCmOverride:USWheelSubBody::SuspensionSpringRateCmOverrideWheelDomainStorage();
}
float& USpeedBranchWheelSubBody::SuspensionCompressionDampingOverrideWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->SuspensionCompressionDampingOverride:USWheelSubBody::SuspensionCompressionDampingOverrideWheelDomainStorage();
}
float const& USpeedBranchWheelSubBody::SuspensionCompressionDampingOverrideWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->SuspensionCompressionDampingOverride:USWheelSubBody::SuspensionCompressionDampingOverrideWheelDomainStorage();
}
float& USpeedBranchWheelSubBody::SuspensionReboundDampingOverrideWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->SuspensionReboundDampingOverride:USWheelSubBody::SuspensionReboundDampingOverrideWheelDomainStorage();
}
float const& USpeedBranchWheelSubBody::SuspensionReboundDampingOverrideWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->SuspensionReboundDampingOverride:USWheelSubBody::SuspensionReboundDampingOverrideWheelDomainStorage();
}
float& USpeedBranchWheelSubBody::SuspensionMaxRaiseOverrideWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->SuspensionMaxRaiseOverride:USWheelSubBody::SuspensionMaxRaiseOverrideWheelDomainStorage();
}
float const& USpeedBranchWheelSubBody::SuspensionMaxRaiseOverrideWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->SuspensionMaxRaiseOverride:USWheelSubBody::SuspensionMaxRaiseOverrideWheelDomainStorage();
}
float& USpeedBranchWheelSubBody::SuspensionMaxDropOverrideWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->SuspensionMaxDropOverride:USWheelSubBody::SuspensionMaxDropOverrideWheelDomainStorage();
}
float const& USpeedBranchWheelSubBody::SuspensionMaxDropOverrideWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->SuspensionMaxDropOverride:USWheelSubBody::SuspensionMaxDropOverrideWheelDomainStorage();
}
float& USpeedBranchWheelSubBody::ConfiguredStaticSpringCompressionWheelDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->ConfiguredStaticSpringCompression:USWheelSubBody::ConfiguredStaticSpringCompressionWheelDomainStorage();
}
float const& USpeedBranchWheelSubBody::ConfiguredStaticSpringCompressionWheelDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->ConfiguredStaticSpringCompression:USWheelSubBody::ConfiguredStaticSpringCompressionWheelDomainStorage();
}
float& USpeedBranchWheelSubBody::RadiusSphereDomainStorage()
{
    return UsesOriginalBodyStorage()?OwnedSphereDomain->Radius:USphereSubBody::RadiusSphereDomainStorage();
}
float const& USpeedBranchWheelSubBody::RadiusSphereDomainStorage() const
{
    return UsesOriginalBodyStorage()?OwnedSphereDomain->Radius:USphereSubBody::RadiusSphereDomainStorage();
}
FTransform USpeedBranchWheelSubBody::PhysicalLocalPose() const {return UsesOriginalBodyStorage()?OwnedBodyValues->LocalPose:USSubBody::PhysicalLocalPose();}
void USpeedBranchWheelSubBody::SetPhysicalLocalOffset(const FVector& V) {if(UsesOriginalBodyStorage())OwnedBodyValues->LocalPose.SetLocation(V);else USSubBody::SetPhysicalLocalOffset(V);}

float USpeedBranchWheelSubBody::PhysicalSuspensionRestLength() const
{
    return UsesOriginalBodyStorage()?OwnedWheelDomain->SuspensionMaxRaiseOverride:USWheelSubBody::PhysicalSuspensionRestLength();
}

FVector USpeedBranchBoxSubBody::PhysicalLocalOffset() const {return UsesOriginalBodyStorage()?OwnedBodyValues->LocalPose.GetLocation():USSubBody::PhysicalLocalOffset();}
FQuat USpeedBranchBoxSubBody::PhysicalLocalRotation() const {return UsesOriginalBodyStorage()?OwnedBodyValues->LocalPose.GetRotation():USSubBody::PhysicalLocalRotation();}
FRotator USpeedBranchBoxSubBody::PhysicalLocalEulerRotation() const
{return UsesOriginalBodyStorage()?OwnedBodyValues->LocalEulerRotation:USSubBody::PhysicalLocalEulerRotation();}
void USpeedBranchBoxSubBody::SetPhysicalLocalRotation(const FQuat& V)
{
    if(UsesOriginalBodyStorage())
    {
        OwnedBodyValues->LocalEulerRotation=V.Rotator();
        OwnedBodyValues->LocalPose.SetRotation(V);
    }
    else USSubBody::SetPhysicalLocalRotation(V);
}

FVector USpeedBranchWheelSubBody::PhysicalLocalOffset() const {return UsesOriginalBodyStorage()?OwnedBodyValues->LocalPose.GetLocation():USSubBody::PhysicalLocalOffset();}
FQuat USpeedBranchWheelSubBody::PhysicalLocalRotation() const {return UsesOriginalBodyStorage()?OwnedBodyValues->LocalPose.GetRotation():USSubBody::PhysicalLocalRotation();}
FRotator USpeedBranchWheelSubBody::PhysicalLocalEulerRotation() const
{return UsesOriginalBodyStorage()?OwnedBodyValues->LocalEulerRotation:USSubBody::PhysicalLocalEulerRotation();}
void USpeedBranchWheelSubBody::SetPhysicalLocalRotation(const FQuat& V)
{
    if(UsesOriginalBodyStorage())
    {
        OwnedBodyValues->LocalEulerRotation=V.Rotator();
        OwnedBodyValues->LocalPose.SetRotation(V);
    }
    else USSubBody::SetPhysicalLocalRotation(V);
}
