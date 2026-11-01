#include "PrivateRoadPhysicsWorld.h"
#include "PrivateRoadNativeObjects.h"
#include "IAmSpeed/Driving/StockRoadCar.h"
#include "IAmSpeed/Netcode/presentation-client/PresentationClientWorld.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "IAmSpeed/World/Simulation/CanonicalFrameContext.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "Engine/World.h"
#include "Components/BoxComponent.h"
#include "Physics/NetworkPhysicsComponent.h"
#include "Chaos/Framework/PhysicsSolverBase.h"
#include "ChaosVehicleWheel.h"
#include "SimpleVehicle.h"
#include "SuspensionSystem.h"
#include "WheelSystem.h"
#include "UObject/StrongObjectPtr.h"
#include "UObject/UObjectHash.h"
#include "UObject/UObjectIterator.h"
#include "Misc/ScopeExit.h"
#include <algorithm>
#include <mutex>

namespace Speed::Netcode::Prediction
{
// No public handle exposes this staging area. It exists only during one GT
// call and cannot be used to install a Device producer, authority receipt or
// legacy snapshot. Native lifetime/restore certification remains separate.
struct FPrivateRoadAccess
{
    struct FSlot {std::shared_ptr<const FBranchProcessedInput> Input;bool Consumed=false;};
    UWorld* World=nullptr;
    bool Active=false,RequiresInitialPublication=true,InitialPublicationReleased=false;
    U64 Frame=0;
    TMap<const USpeedBranchWheeledComponent*,FSlot> Slots;
};
namespace
{
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
bool NoNetworkOwner(UWorld* World) {return World&&!World->GetNetDriver()&&NativeNetworkComponentCount(World)==0;}
}
struct FPrivateRoadPhysicsWorld::FImpl
{
    struct FCar
    {
        FPrivateRoadCarSeed Seed;
        Chaos::FSimpleWheelConfig WheelConfig;
        Chaos::FSimpleSuspensionConfig SuspensionConfig;
        ASpeedBranchCar* Actor=nullptr;
        USpeedBranchWheeledComponent* Component=nullptr;
        U64 StableId=0;
        std::unique_ptr<FBranchInputFilter> Filter;
    };
    TStrongObjectPtr<UWorld> World;
    TArray<TStrongObjectPtr<UObject>> Pins;
    USpeedWorldSubsystem* Bridge=nullptr;
    ASpeedBranchSimulation* Driver=nullptr;
    std::vector<std::unique_ptr<FCar>> Cars;
    std::shared_ptr<FPrivateRoadAccess> Access=std::make_shared<FPrivateRoadAccess>();
    mutable std::mutex Gate;
    std::atomic<std::shared_ptr<const FPrivateRoadPublication>> Published;
    U64 Lifetime=0;
    bool Closed=false,Failed=false;
    void Pin(UObject* Object) {if(Object)Pins.Emplace(Object);}
    void PinTree(UObject* Object)
    {
        Pin(Object);ForEachObjectWithOuter(Object,[this](UObject* Child){Pin(Child);},true);
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
    if(!IsInGameThread()||Speed::Input::FPresentationInputScope::IsActive()
        ||Speed::Netcode::PresentationClient::IsSelected()||Seeds.empty()||Seeds.size()>16
        ||Chaos::FPhysicsSolverBase::IsNetworkPhysicsPredictionEnabled())
    {Refuse(Why,TEXT("private native world requires GT, IAS-only physical profile, disabled Chaos prediction and finite roster"));return {};}
    std::sort(Seeds.begin(),Seeds.end(),[](const auto& A,const auto& B){return A.Actor<B.Actor;});
    for(const auto& Seed:Seeds)if(!Seed.Contract||!PrivateRoadSupportedContract(*Seed.Contract))
    {Refuse(Why,TEXT("unsupported private road action contract"));return {};}
    const auto StartPolicy=Seeds.front().StartPolicy;
    if(StartPolicy>EPrivateRoadStartPolicy::ExplicitUnitRecipe)return {};
    for(const auto& S:Seeds)if(S.StartPolicy!=StartPolicy
        ||(StartPolicy==EPrivateRoadStartPolicy::AwaitInitialPublication&&S.CountdownFrames))
    {Refuse(Why,TEXT("mixed startup policy or countdown substituted for initial publication"));return {};}
    auto Owner=std::unique_ptr<FPrivateRoadPhysicsWorld>(new FPrivateRoadPhysicsWorld);
    auto& I=*Owner->Impl;
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
        I.Cars.push_back(std::move(Node));
    }
    const auto Options=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(false)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false).SetTransactional(false);
    I.World.Reset(UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Options));
    if(!I.World.IsValid()){Refuse(Why,TEXT("private world creation failed"));return {};}
    // Deliberately no Engine world context, BeginPlay, normal tick or scene.
    I.Access->World=I.World.Get();I.Bridge=I.World->GetSubsystem<USpeedWorldSubsystem>();
    if(!I.Bridge){Refuse(Why,TEXT("private native bridge missing"));return {};}
    I.Pin(I.Bridge);
    auto Data=MakeShared<Speed::Analytic::FAnalyticWorldData>();
    auto& P=Data->Planes.AddDefaulted_GetRef();P.SourceId=1;P.SurfaceId=2;P.FeatureId=3;P.PrimitiveId=4;
    P.ObjectType=uint32(ECC_WorldStatic);P.BlockingChannels=MAX_uint64;
    P.Normal=FVector3d::UpVector;P.AxisU=FVector3d::ForwardVector;P.AxisV=FVector3d::RightVector;
    P.HalfExtents=FVector2d(100000,100000);
    if(!Data->FinalizeAndValidate(&Why))return {};
    // Exact authored plane, not an imported asset authority certificate.
    Data->Planes[0].bAuthorityEligible=true;
    auto* FloorActor=I.World->SpawnActor<AActor>();
    if(!FloorActor){Refuse(Why,TEXT("private floor actor missing"));return {};}
    auto* Floor=NewObject<UBoxComponent>(FloorActor);FloorActor->AddInstanceComponent(Floor);
    Floor->SetMobility(EComponentMobility::Static);Floor->SetCollisionObjectType(ECC_WorldStatic);
    Floor->SetCollisionResponseToAllChannels(ECR_Block);I.PinTree(FloorActor);
    I.Bridge->AnalyticSourceComponents.Add(1,Floor);I.Bridge->AnalyticWorldData=Data;
    I.Bridge->SimulationWorld.SetStaticCollisionWorld(MakeUnique<Speed::FAnalyticStaticCollisionWorld>(*Data));
    I.Driver=I.World->SpawnActor<ASpeedBranchSimulation>();
    if(!I.Driver||ASpeedPhysicsSimulation::GetEngineFPS()!=300)
    {Refuse(Why,TEXT("private driver missing or canonical physical rate differs"));return {};}
    I.Driver->Access=I.Access;I.Driver->bPublishPresentation=true;I.Driver->bPublishCompletedWorldInput=false;
    I.PinTree(I.Driver);
    for(auto& Node:I.Cars)
    {
        auto& N=*Node;N.Actor=I.World->SpawnActor<ASpeedBranchCar>();
        N.Component=N.Actor?Cast<USpeedBranchWheeledComponent>(N.Actor->GetVehicleMovement()):nullptr;
        auto* C=N.Component;
        if(!C||C->GetClass()!=USpeedBranchWheeledComponent::StaticClass())
        {Refuse(Why,TEXT("private native component subtype missing"));return {};}
        C->Access=I.Access;C->SetOwner(N.Actor);C->SetAsyncPhysicsTickEnabled(false);
        N.SuspensionConfig.SetSuspensionMaxRaise(10);N.SuspensionConfig.SetSuspensionMaxDrop(25);
        N.SuspensionConfig.SpringRate=5000;N.SuspensionConfig.RestingForce=250000;
        N.SuspensionConfig.CompressionDamping=1000;N.SuspensionConfig.ReboundDamping=1000;
        auto Vehicle=MakeUnique<Chaos::FSimpleWheeledVehicle>();
        for(int32 W=0;W<4;++W)
        {
            auto& Wheel=Vehicle->Wheels.Emplace_GetRef(&N.WheelConfig);
            Wheel.MaxOmega=10000;Wheel.AppliedLinearDriveForce=0;Wheel.AppliedLinearBrakeForce=0;
            Wheel.LongitudinalAdhesiveLimit=0;Wheel.LateralAdhesiveLimit=0;Wheel.SideSlipModifier=N.WheelConfig.SideSlipModifier;
            Vehicle->Suspension.Emplace_GetRef(&N.SuspensionConfig).SetLocalRestingPosition(FVector(W<2?70:-70,W%2?45:-45,-15));
            auto* HostWheel=NewObject<UChaosVehicleWheel>(C);HostWheel->WheelRadius=30;
            HostWheel->SuspensionMaxRaise=10;HostWheel->SuspensionMaxDrop=25;C->Wheels.Add(HostWheel);
        }
        C->VehicleSimulationPT=MakeUnique<USpeedSimulation>();
        C->SkySimulation=static_cast<USpeedSimulation*>(C->VehicleSimulationPT.Get());C->SkySimulation->Init(Vehicle);
        C->BindWheelSimulationPointers();
        if(C->GetWheelSubBodies().Num()!=4){Refuse(Why,TEXT("private native wheel topology differs"));return {};}
        for(int32 W=0;W<4;++W)
        {C->GetWheelSubBodies()[W]->SetLocalOffset(FVector(W<2?70:-70,W%2?45:-45,-15));C->GetWheelSubBodies()[W]->SetRadiusForConfiguration(30);}
        if(!C->ValidateSimulationBindings(Why))return {};
        if(C->ThrottleAccel!=750||C->BrakeAccel!=1100||C->PhysDamping!=0||C->AirForwardThrottle!=0||C->AirBackwardThrottle!=0)
        {Refuse(Why,TEXT("stock native driving profile differs"));return {};}
        C->SetEngineFPS(300);C->SetKinematicState(N.Seed.Initial);
        // Declared fresh-world initial countdown, never a historical restore.
        if(I.Access->RequiresInitialPublication)
        {
            // Closed state, not a timer: no expiry or decrement can release it.
            C->SetCannotMoveLocal();
        }
        else
        {
            C->WheeledPhysicsState.nbFramesbeforeCanMove=N.Seed.CountdownFrames;
            C->WheeledPhysicsState.bStartCountdown=N.Seed.CountdownFrames>0;
            C->SinceCanMoveFrame=N.Seed.CountdownFrames?INDEX_NONE:0;
            if(N.Seed.CountdownFrames)C->FreezeMovement();else C->UnFreezeMovement();
        }
        C->bPublishWheelPresentation=true;I.PinTree(N.Actor);I.PinTree(C);
    }
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
    I.Lifetime=NextLifetime++;
    return Owner;
}
bool FPrivateRoadPhysicsWorld::Step(const std::vector<FPrivateRoadRequest>& Requests,bool Resimulation,FString& Why)
{
    Why.Empty();if(!IsInGameThread()||Speed::Input::FPresentationInputScope::IsActive())return Refuse(Why,TEXT("private step requires GT physical caller"));
    auto& I=*Impl;std::unique_lock Lock(I.Gate,std::try_to_lock);
    if(!Lock.owns_lock()||I.Closed||I.Failed||!I.World.IsValid()||I.World->bIsTearingDown
        ||!I.Driver||Requests.size()!=I.Cars.size())return Refuse(Why,TEXT("private step owner/roster unavailable"));
    if(!IsValid(I.Driver)||I.Driver->GetWorld()!=I.World.Get()||I.Driver->HasActorBegunPlay()
        ||I.Driver->IsActorBeingDestroyed())return Refuse(Why,TEXT("private driver lifecycle changed"));
    for(const auto& N:I.Cars)
        if(!IsValid(N->Actor)||!IsValid(N->Component)||N->Actor->IsActorBeingDestroyed()||N->Actor->HasActorBegunPlay()
            ||N->Actor->GetWorld()!=I.World.Get()||N->Component->GetOwner()!=N->Actor
            ||N->Actor->GetVehicleMovement()!=N->Component||N->Component->GetClass()!=USpeedBranchWheeledComponent::StaticClass())
            return Refuse(Why,TEXT("private native actor/component lifetime changed"));
    for(const auto& Pin:I.Pins)if(!IsValid(Pin.Get()))return Refuse(Why,TEXT("private native graph object invalidated"));
    const U64 Frame=I.Driver->NextPrivate();
    if(Frame&&I.Access->RequiresInitialPublication&&!I.Access->InitialPublicationReleased)
        return Refuse(Why,TEXT("awaiting actual initial publication and common activation; no timer release"));
    if(Frame>=uint64(MAX_int32-1)||Chaos::FPhysicsSolverBase::IsNetworkPhysicsPredictionEnabled()||!NoNetworkOwner(I.World.Get()))return Refuse(Why,TEXT("private step native domain changed"));
    std::vector<FBranchInputFilter::FPrepared> Plans;Plans.reserve(I.Cars.size());
    I.Access->Slots.Empty();
    ON_SCOPE_EXIT{I.Access->Active=false;I.Access->Slots.Empty();};
    for(const auto& N:I.Cars)
    {
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
        I.Access->Slots.Add(N->Component,{Processed,false});Plans.push_back(std::move(*Plan));
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
    Out->NativeNetworkComponents=NativeNetworkComponentCount(I.World.Get());
    if(Out->NativeNetworkComponents)return Fail(TEXT("UE NetworkPhysics object appeared in IAS-only native graph"));
    if(!I.Driver->ReadLatestSimulationSnapshot(Out->Physics)||Out->Physics.NumFrame!=Frame
        ||Out->Physics.PresentationBodies.Num()!=int32(I.Cars.size()))return Fail(TEXT("private native publication mismatch"));
    for(const auto& N:I.Cars)
    {
        auto State=N->Filter->Capture();if(!State||State->Next!=Frame+1)return Fail(TEXT("private filter publication mismatch"));
        Out->Actors.push_back({N->Seed.Actor,N->Seed.Incarnation,N->StableId,*State,I.Access->Slots.FindChecked(N->Component).Input,
            N->Component->CanMove(),N->Component->IsFrozen(),N->Component->WheeledPhysicsState.nbFramesbeforeCanMove});
    }
    if(I.Access->RequiresInitialPublication)for(const auto& A:Out->Actors)
    {
        if(Frame==0&&(A.CanMove||!A.Frozen||A.CountdownFrames!=1))return Fail(TEXT("initial publication moved before synchronization"));
        if(Frame>0&&(!A.CanMove||A.Frozen||A.CountdownFrames))return Fail(TEXT("common startup activation not applied"));
    }
    I.Published.store(std::move(Out));return true;
    }
    catch(...) {return Fail(TEXT("private native branch retired after step/publication exception"));}
}
std::shared_ptr<const FPrivateRoadPublication> FPrivateRoadPhysicsWorld::ReadLatest() const noexcept {return Impl->Published.load();}
bool FPrivateRoadPhysicsWorld::ReleaseInitialPublication(const std::shared_ptr<const FPrivateRoadPublication>& Baseline,FString& Why)
{
    Why.Empty();if(!IsInGameThread()||Speed::Input::FPresentationInputScope::IsActive())return Refuse(Why,TEXT("startup release requires original GT owner"));
    auto& I=*Impl;std::unique_lock Lock(I.Gate,std::try_to_lock);
    if(!Lock.owns_lock()||I.Closed||I.Failed||I.Access->Active||!I.Access->RequiresInitialPublication
        ||I.Access->InitialPublicationReleased||!Baseline||Baseline!=I.Published.load()
        ||!Baseline->AwaitingInitialPublication||Baseline->Frame!=0||Baseline->LocalKernelLifetime!=I.Lifetime
        ||!I.World.IsValid()||I.World->bIsTearingDown||!I.Driver||!IsValid(I.Driver)
        ||I.Driver->NextPrivate()!=1||Baseline->Actors.size()!=I.Cars.size())return Refuse(Why,TEXT("startup original baseline/boundary unavailable"));
    for(const auto& Pin:I.Pins)if(!IsValid(Pin.Get()))return Refuse(Why,TEXT("startup native graph invalidated"));
    for(const auto& N:I.Cars)
    {
        const auto* C=N->Component;
        if(!IsValid(N->Actor)||!IsValid(C)||N->Actor->IsActorBeingDestroyed()||N->Actor->HasActorBegunPlay()
            ||N->Actor->GetWorld()!=I.World.Get()||C->GetOwner()!=N->Actor||N->Actor->GetVehicleMovement()!=C
            ||C->GetClass()!=USpeedBranchWheeledComponent::StaticClass()||!C->IsFrozen()||C->CanMove()
            ||C->WheeledPhysicsState.bStartCountdown||C->WheeledPhysicsState.nbFramesbeforeCanMove!=1)
            return Refuse(Why,TEXT("startup native identity or closed gate changed"));
    }
    // Same established physical countdown-release operations, at a common
    // explicit boundary instead of an arbitrary elapsed frame count.
    for(const auto& N:I.Cars)
    {
        auto* C=N->Component;C->WheeledPhysicsState.nbFramesbeforeCanMove=0;C->WheeledPhysicsState.bStartCountdown=false;
        C->SinceCanMoveFrame=C->NumFrame()+1;C->bSimTimelineWasCanMove=true;
        C->bSimTimelineHasGroundState=true;C->bSimTimelineWasGrounded=C->IsOnTheGround();
        C->ResetWheelTransientStateForCurrentPose(false);C->UnFreezeMovement();
    }
    I.Access->InitialPublicationReleased=true;return true;
}
bool FPrivateRoadPhysicsWorld::Close()
{
    if(!IsInGameThread())return false;
    auto& I=*Impl;std::unique_lock Lock(I.Gate,std::try_to_lock);if(!Lock.owns_lock()||I.Access->Active)return false;
    if(I.Closed)return true;I.Closed=true;I.Published.store({});I.Access->Slots.Empty();I.Access->World=nullptr;
    if(I.Bridge)
    {
        for(const auto& N:I.Cars)if(N->Component)I.Bridge->UnregisterSpeedComponent(N->Component);
        I.Bridge->ApplyPendingOps();
    }
    for(const auto& N:I.Cars)if(auto* C=N->Component)
    {C->Access.reset();C->ClearWheelSimulationPointers();C->SkySimulation=nullptr;C->VehicleSimulationPT.Reset();}
    if(I.Driver)I.Driver->Access.reset();
    if(I.World.IsValid())I.World->DestroyWorld(false);
    I.Pins.Empty();I.World.Reset();I.Cars.clear();I.Bridge=nullptr;I.Driver=nullptr;return true;
}
}

USpeedBranchWheeledComponent::USpeedBranchWheeledComponent(const FObjectInitializer& Initializer)
    :Super(Initializer,ESpeedWheeledNetworkConstruction::IAmSpeedOnly)
{
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
bool USpeedBranchWheeledComponent::IsInitialSynchronizationComplete() const
{
    const auto A=Access.lock();return A&&(!A->RequiresInitialPublication||A->InitialPublicationReleased);
}
void USpeedBranchWheeledComponent::UpdateInputs()
{
    const auto A=Access.lock();bInputFrameRejectedV2=true;
    const auto* Slot=A?A->Slots.Find(this):nullptr;
    if(!IsInGameThread()||!A||!A->Active||A->World!=GetWorld()||!Slot||!Slot->Input
        ||Slot->Input->Frame!=A->Frame||uint64(NumFrame())!=A->Frame+1
        ||FrameInputStream||FrameInputStreamV2||CanonicalInputSnapshot||InputReservationV2||ReservedInputFrameV2
        ||HasProducedInputAuthority()||HasLegacyRemoteInputAuthority())return;
    const auto& Applied=Slot->Input->Applied;
    bProducedInputFrameOwned=true; // actual frame ownership, no Device authority.
    WheeledPhysicalInput.Throttle=uint8(Applied[0]);WheeledPhysicalInput.Brake=uint8(Applied[1]);WheeledPhysicalInput.Steer=int8(Applied[2]);
    WheeledPhysicalInput.bCanMove=CanMove();
    WheeledPhysicalInput.Camera={};WheeledPhysicalInput.Camera.Flags=1;
    WheeledPhysicalInput.Camera.Yaw=int8(Applied[5]);WheeledPhysicalInput.Camera.Pitch=int8(Applied[6]);
    SyncWheeledPhysicalInputToState();bInputFrameRejectedV2=false;
}
bool USpeedBranchWheeledComponent::ValidateCanonicalFrameCommit(uint64 Frame) const
{
    const auto A=Access.lock();const auto* Slot=A?A->Slots.Find(this):nullptr;
    return IsInGameThread()&&A&&A->Active&&A->World==GetWorld()&&A->Frame==Frame&&Slot&&Slot->Input
        &&!Slot->Consumed&&!bInputFrameRejectedV2&&PreparedCanonicalInputFrame.IsSet()&&PreparedCanonicalInputFrame.GetValue()==Frame;
}
bool USpeedBranchWheeledComponent::CommitCanonicalFrame(uint64 Frame) noexcept
{
    if(!ValidateCanonicalFrameCommit(Frame))return false;Access.lock()->Slots.FindChecked(this).Consumed=true;return true;
}
void USpeedBranchWheeledComponent::AbortCanonicalFrame(uint64,ECanonicalFrameAbortReason) noexcept {bInputFrameRejectedV2=true;}
ASpeedBranchCar::ASpeedBranchCar(const FObjectInitializer& Initializer)
    :Super(Initializer.SetDefaultSubobjectClass<USpeedBranchWheeledComponent>(VehicleMovementComponentName))
{PrimaryActorTick.bCanEverTick=false;PrimaryActorTick.bStartWithTickEnabled=false;SetReplicates(false);}
void ASpeedBranchCar::BeginPlay() {AActor::BeginPlay();}
ASpeedBranchSimulation::ASpeedBranchSimulation()
{bAsyncPhysicsTickEnabled=false;PrimaryActorTick.bCanEverTick=false;PrimaryActorTick.bStartWithTickEnabled=false;SetReplicates(false);}
void ASpeedBranchSimulation::BeginPlay() {AActor::BeginPlay();}
bool ASpeedBranchSimulation::InitializePrivate(USpeedWorldSubsystem& Bridge)
{
    if(!IsInGameThread()||!Access.lock()||GetEngineFPS()!=300)return false;
    SpeedWorldSubsystem=&Bridge;if(!EnsureSimulationWorldReady())return false;
    SealSimulationInputs();InitializeCanonicalFrame(0);return CanonicalNumFrame==0;
}
bool ASpeedBranchSimulation::AdvancePrivate(bool Resimulation)
{
    const auto A=Access.lock();
    if(!IsInGameThread()||!A||!A->Active||A->Frame!=CanonicalNumFrame||A->World!=GetWorld())return false;
    FCanonicalFrameContext Context(CanonicalNumFrame);Context.Purpose=Resimulation?ECanonicalFramePurpose::Resimulation:ECanonicalFramePurpose::Forward;
    if(!StepCanonicalFrame(Context))return false;
    ++CanonicalNumFrame;return true; // same success-only counter as RunCanonicalFrames.
}
