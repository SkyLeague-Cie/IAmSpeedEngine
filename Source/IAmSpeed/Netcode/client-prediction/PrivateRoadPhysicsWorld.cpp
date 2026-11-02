#include "PrivateRoadPhysicsWorld.h"
#include "PrivateRoadNativeObjects.h"
#include "PrivateRoadPartialStateObservation.h"
#include "IAmSpeed/Driving/StockRoadCar.h"
#include "IAmSpeed/Netcode/presentation-client/PresentationClientWorld.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "IAmSpeed/World/Simulation/CanonicalFrameContext.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "Engine/World.h"
#include "GameFramework/HUD.h"
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
    &&!World->GetNetDriver()&&NativeNetworkComponentCount(World)==0;}
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
        Chaos::FSimpleWheelConfig WheelConfig;
        Chaos::FSimpleSuspensionConfig SuspensionConfig;
        Chaos::FSimpleWheelConfig ReferenceWheelConfig;
        Chaos::FSimpleSuspensionConfig ReferenceSuspensionConfig;
        ASpeedBranchCar* Actor=nullptr;
        USpeedBranchWheeledComponent* Component=nullptr;
        U64 StableId=0;
        bool SolidInitializationWitness=false;
        std::unique_ptr<FBranchInputFilter> Filter;
    };
    TStrongObjectPtr<UWorld> World;
    TArray<TStrongObjectPtr<UObject>> Pins;
    USpeedWorldSubsystem* Bridge=nullptr;
    ASpeedBranchSimulation* Driver=nullptr;
    std::vector<std::unique_ptr<FCar>> Cars;
    std::shared_ptr<FPrivateRoadAccess> Access=std::make_shared<FPrivateRoadAccess>();
    std::unique_ptr<Speed::NativeEpoch::FDiagnosticOutbox> Diagnostics;
    mutable std::mutex Gate;
    std::atomic<std::shared_ptr<const FPrivateRoadPublication>> Published;
    U64 Lifetime=0,VehicleContactFrameCount=0;
    bool Closed=false,Failed=false;
    void Pin(UObject* Object) {if(Object)Pins.Emplace(Object);}
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
    const auto StartPolicy=Seeds.front().StartPolicy;
    if(StartPolicy>EPrivateRoadStartPolicy::ExplicitUnitRecipe)return {};
    for(const auto& S:Seeds)if(S.StartPolicy!=StartPolicy
        ||(StartPolicy==EPrivateRoadStartPolicy::AwaitInitialPublication&&S.CountdownFrames))
    {Refuse(Why,TEXT("mixed startup policy or countdown substituted for initial publication"));return {};}
    auto Owner=std::unique_ptr<FPrivateRoadPhysicsWorld>(new FPrivateRoadPhysicsWorld);
    auto& I=*Owner->Impl;
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
        I.Cars.push_back(std::move(Node));
    }
    I.Access->ContactDispatches.reserve(I.Cars.size()*(I.Cars.size()-1)/2);
    const auto Options=UWorld::InitializationValues().InitializeScenes(false).CreateFXSystem(false).AllowAudioPlayback(false).CreatePhysicsScene(false)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false).SetTransactional(false);
    I.World.Reset(UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Options));
    if(!I.World.IsValid()){Refuse(Why,TEXT("private world creation failed"));return {};}
    // InitWorld assigns this tick flag only inside its scene-initialization
    // branch. Set it explicitly for the deliberately scene-less private world.
    I.World->bShouldSimulatePhysics=false;
    if(I.World->Scene||I.World->GetPhysicsScene())
    {Refuse(Why,TEXT("renderer or physics scene escaped private construction policy"));return {};}
    // Deliberately no Engine world context, BeginPlay, normal tick or scene.
    I.Access->World=I.World.Get();I.Bridge=I.World->GetSubsystem<USpeedWorldSubsystem>();
    if(!I.Bridge){Refuse(Why,TEXT("private native bridge missing"));return {};}
    I.Pin(I.Bridge);
    I.Bridge->BindOriginalEpochStorage(I.Access);
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
    {
        FOriginalComponentStorageScope OriginalWorldAdmission(*I.Access);
        I.Bridge->AnalyticSourceComponentsEpochStorage().Add(1,Floor);
        I.Bridge->AnalyticWorldDataEpochStorage()=Data;
        I.Bridge->SimulationWorldEpochStorage().SetStaticCollisionWorld(MakeUnique<Speed::FAnalyticStaticCollisionWorld>(*Data));
    }
    I.Driver=I.World->SpawnActor<ASpeedBranchSimulation>();
    if(!I.Driver||ASpeedPhysicsSimulation::GetEngineFPS()!=300)
    {Refuse(Why,TEXT("private driver missing or canonical physical rate differs"));return {};}
    I.Driver->Access=I.Access;if(!I.Driver->BindOriginalDriverStorage(I.Access)){Refuse(Why,TEXT("original native driver birth storage refused"));return {};}
    {FOriginalComponentStorageScope OriginalDriver(*I.Access);
        I.Driver->bPublishPresentationDriverStorage()=true;I.Driver->bPublishCompletedWorldInputDriverStorage()=false;}
    I.PinTree(I.Driver);
    const auto BuildOriginalVehicle=[&](USpeedBranchWheeledComponent& C,Chaos::FSimpleWheelConfig& WheelConfig,Chaos::FSimpleSuspensionConfig& SuspensionConfig)
    {
        auto Vehicle=MakeUnique<Chaos::FSimpleWheeledVehicle>();
        for(int32 W=0;W<4;++W)
        {
            auto& Wheel=Vehicle->Wheels.Emplace_GetRef(&WheelConfig);
            Wheel.MaxOmega=10000;Wheel.AppliedLinearDriveForce=0;Wheel.AppliedLinearBrakeForce=0;
            Wheel.LongitudinalAdhesiveLimit=0;Wheel.LateralAdhesiveLimit=0;Wheel.SideSlipModifier=WheelConfig.SideSlipModifier;
            Vehicle->Suspension.Emplace_GetRef(&SuspensionConfig).SetLocalRestingPosition(FVector(W<2?70:-70,W%2?45:-45,-15));
            auto* HostWheel=NewObject<UChaosVehicleWheel>(&C);HostWheel->WheelRadius=30;
            HostWheel->SuspensionMaxRaise=10;HostWheel->SuspensionMaxDrop=25;C.WheelsWheeledDomainStorage().Add(HostWheel);
        }
        C.VehicleSimulationPTWheeledDomainStorage()=MakeUnique<USpeedSimulation>();
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
        C->SetOwner(N.Actor);C->SetAsyncPhysicsTickEnabled(false);
        // SetOwner's normal legacy registration remains a facade operation.
        for(auto* Body:C->GetSubBodies())
        {
            if(auto* Box=Cast<USpeedBranchBoxSubBody>(Body))Box->StorageAccess=I.Access;
            else if(auto* Wheel=Cast<USpeedBranchWheelSubBody>(Body))Wheel->StorageAccess=I.Access;
            else {Refuse(Why,TEXT("private subbody storage subtype missing"));return {};}
        }
        if(!C->GetHitboxSubBodyForConfiguration()||C->GetHitboxSubBodyForConfiguration()->GetClass()!=USpeedBranchBoxSubBody::StaticClass())
        {Refuse(Why,TEXT("private native contact dispatch subtype missing"));return {};}
        N.ReferenceSuspensionConfig.SetSuspensionMaxRaise(10);N.ReferenceSuspensionConfig.SetSuspensionMaxDrop(25);
        N.ReferenceSuspensionConfig.SpringRate=5000;N.ReferenceSuspensionConfig.RestingForce=250000;
        N.ReferenceSuspensionConfig.CompressionDamping=1000;N.ReferenceSuspensionConfig.ReboundDamping=1000;
        N.SuspensionConfig.SetSuspensionMaxRaise(10);N.SuspensionConfig.SetSuspensionMaxDrop(25);
        N.SuspensionConfig.SpringRate=5000;N.SuspensionConfig.RestingForce=250000;
        N.SuspensionConfig.CompressionDamping=1000;N.SuspensionConfig.ReboundDamping=1000;
        BuildOriginalVehicle(*C,N.ReferenceWheelConfig,N.ReferenceSuspensionConfig);
        if(C->GetWheelSubBodies().Num()!=4){Refuse(Why,TEXT("private native wheel topology differs"));return {};}
        for(int32 W=0;W<4;++W)
        {C->GetWheelSubBodies()[W]->SetLocalOffset(FVector(W<2?70:-70,W%2?45:-45,-15));C->GetWheelSubBodies()[W]->SetRadiusForConfiguration(30);}
        if(!C->ValidateSimulationBindings(Why))return {};
        if(C->ThrottleAccelWheeledDomainStorage()!=750||C->BrakeAccelWheeledDomainStorage()!=1100||C->PhysDampingWheeledDomainStorage()!=0||C->AirForwardThrottleWheeledDomainStorage()!=0||C->AirBackwardThrottleWheeledDomainStorage()!=0)
        {Refuse(Why,TEXT("stock native driving profile differs"));return {};}
        C->EstablishOriginalComponentStorage();
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
        // Witness only: actual legacy values from the original setup. These
        // are NEVER imported into physical storage or a restore certificate.
        TArray<uint8> OriginalSolidRecipe;
        for(auto* Body:C->GetSubBodies())
        {
            if(const auto* Box=Cast<USpeedBranchBoxSubBody>(Body))
            {if(!PartialObservation::AppendSolidValues(OriginalSolidRecipe,Box->ReadLegacySolidDiagnosticValues()))return {};}
            else if(const auto* Wheel=Cast<USpeedBranchWheelSubBody>(Body))
            {if(!PartialObservation::AppendSolidValues(OriginalSolidRecipe,Wheel->ReadLegacySolidDiagnosticValues()))return {};}
            else return {};
        }
#endif
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
        C->SetSubBodies(OriginalTopology);C->SetOwner(N.Actor);
        BuildOriginalVehicle(*C,N.WheelConfig,N.SuspensionConfig);
        for(int32 W=0;W<4;++W)
        {
            auto* Body=C->GetWheelSubBodies()[W].Get();
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
        for(int32 W=0;W<C->GetWheelSubBodies().Num();++W)C->GetWheelSubBodies()[W]->SetIdx(W);
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
    }
    FOriginalComponentStorageScope OriginalInitialization(*I.Access);
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
    return Owner;
}
bool FPrivateRoadPhysicsWorld::HasExactRegisteredRoster(FString& Why) const
{
    const auto& I=*Impl;
    if(!I.Bridge)return Refuse(Why,TEXT("private native registry bridge missing"));
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
    if(!I.Diagnostics||!I.Diagnostics->Begin())return Refuse(Why,TEXT("native diagnostic owner unavailable"));
    std::shared_ptr<const FPrivateRoadPublication> Completed;
    bool DiagnosticActive=true;ON_SCOPE_EXIT{if(DiagnosticActive)I.Diagnostics->End();};
    bool Advanced=false;try {Advanced=[&]() -> bool
    {
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
    for(const auto& N:I.Cars)
    {
        auto State=N->Filter->Capture();if(!State||State->Next!=Frame+1)return Fail(TEXT("private filter publication mismatch"));
        Out->Actors.push_back({N->Seed.Actor,N->Seed.Incarnation,N->StableId,*State,I.Access->Slots.FindChecked(N->Component).Input,
            N->Component->CanMove(),N->Component->IsFrozen(),N->Component->WheeledPhysicsStateStorage().nbFramesbeforeCanMove});
        const auto* Hitbox=N->Component->GetHitboxSubBodyForConfiguration();
        if(!Hitbox)return Fail(TEXT("native hitbox diagnostics missing"));
        Out->Actors.back().NativeHitboxHalfExtent=Hitbox->GetBoxExtent();
        Out->Actors.back().NativeHitboxCenter=Hitbox->GetKinematicState().Location;
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
        // This is a read-only declared-subset observation, not a complete-state
        // capture through the guarded restore provider. Unknown remains false.
        auto& Observation=Out->Actors.back();
        Observation.NativeSolidInitializationWitness=N->SolidInitializationWitness;
        if(N->Component->CopyActualCacheDiagnosticValues(Observation.NativeSleep,Observation.NativeQuantization))
            PartialObservation::Capture(*N->Component,N->Seed.Actor,N->Seed.Incarnation,
                N->WheelConfig,N->SuspensionConfig,Observation);
        if(Observation.NativePartialValuesAvailable)
        {
            TArray<uint8> RuntimeValues;
            for(const auto& Entry:N->Component->GetWheelSubBodies())
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
    if(!Lock.owns_lock()||I.Closed||I.Failed||I.Access->Active||!I.Access->RequiresInitialPublication
        ||I.Access->InitialPublicationReleased||!Baseline||Baseline!=I.Published.load()
        ||!Baseline->AwaitingInitialPublication||Baseline->Frame!=0||Baseline->LocalKernelLifetime!=I.Lifetime
        ||!I.World.IsValid()||I.World->bIsTearingDown||!I.Driver||!IsValid(I.Driver)
        ||Baseline->Actors.size()!=I.Cars.size())return Refuse(Why,TEXT("startup original baseline/boundary unavailable"));
    FOriginalComponentStorageScope OriginalStorage(*I.Access);
    if(I.Driver->NextPrivate()!=1)return Refuse(Why,TEXT("original startup physical counter changed"));
    if(!HasExactRegisteredRoster(Why))return false;
    for(const auto& Pin:I.Pins)if(!IsValid(Pin.Get()))return Refuse(Why,TEXT("startup native graph invalidated"));
    for(const auto& N:I.Cars)
    {
        const auto* C=N->Component;
        if(!IsValid(N->Actor)||!IsValid(C)||N->Actor->IsActorBeingDestroyed()||N->Actor->HasActorBegunPlay()
            ||N->Actor->GetWorld()!=I.World.Get()||C->GetOwner()!=N->Actor||N->Actor->GetVehicleMovement()!=C
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
    I.Access->InitialPublicationReleased=true;return true;
}
bool FPrivateRoadPhysicsWorld::Close()
{
    if(!IsInGameThread())return false;
    auto& I=*Impl;std::unique_lock Lock(I.Gate,std::try_to_lock);if(!Lock.owns_lock()||I.Access->Active)return false;
    if(I.Closed)return true;I.Closed=true;I.Published.store({});I.Access->Slots.Empty();I.Access->World=nullptr;
    if(I.Bridge)
    {
        FOriginalComponentStorageScope OriginalRetirement(*I.Access);
        for(const auto& N:I.Cars)if(N->Component)I.Bridge->UnregisterSpeedComponent(N->Component);
        I.Bridge->ApplyPendingOps();
    }
    I.Access->Retired=true; // actual native registry retired, before callbacks
    for(const auto& N:I.Cars)if(auto* C=N->Component)C->Access.reset();
    if(I.Driver)I.Driver->Access.reset();
    // The actual synchronous WorldCleanup/PostWorldCleanup callbacks can still
    // query their retained native components. Keep original wheel/suspension
    // storage and its configuration owners alive until those calls RETURN.
    // This is not exclusion of future escaped views or a complete lease.
    if(I.World.IsValid())I.World->DestroyWorld(false);
    for(const auto& N:I.Cars)if(auto* C=N->Component)
    {
        C->ClearWheelSimulationPointers();
        if(C->OriginalWheeledDomain)
        {
            for(const auto& E:C->OriginalWheeledDomain->WheelSubBodies)
                if(auto* B=Cast<USpeedBranchWheelSubBody>(E.Get()))
                {B->OwnedWheelDomain->PWheel=nullptr;B->OwnedWheelDomain->PSuspension=nullptr;B->OwnedWheelDomain->ChaosWheel=nullptr;}
            C->OriginalWheeledDomain->SkySimulation=nullptr;C->OriginalWheeledDomain->VehicleSimulationPT.Reset();
        }
        C->SkySimulationWheeledDomainStorage()=nullptr;C->VehicleSimulationPTWheeledDomainStorage().Reset();
    }
    I.Pins.Empty();I.World.Reset();I.Cars.clear();I.Bridge=nullptr;I.Driver=nullptr;return true;
}
}

USpeedBranchBoxSubBody::USpeedBranchBoxSubBody(const FObjectInitializer& Initializer)
    :Super(Initializer),OwnedBodyValues(std::make_unique<Speed::Netcode::Prediction::FPrivateRoadSubBodyValues>()),OwnedSolidValues(std::make_unique<Speed::Netcode::Prediction::FPrivateRoadSolidValues>()),OwnedBoxDomain(std::make_unique<Speed::Netcode::Prediction::FPrivateRoadBoxDomain>()){OwnedBodyValues->SubBodyType=ESubBodyType::Hitbox;}
bool USpeedBranchBoxSubBody::UsesOriginalBodyStorage() const
{
    if(!IsInGameThread()||!Speed::Netcode::Prediction::OriginalComponentStorageOwner)return false;
    const auto A=StorageAccess.lock();
    return OwnedBodyValues&&A&&Speed::Netcode::Prediction::OriginalComponentStorageOwner==A.get();
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
    :Super(Initializer),OwnedBodyValues(std::make_unique<Speed::Netcode::Prediction::FPrivateRoadSubBodyValues>()),OwnedSolidValues(std::make_unique<Speed::Netcode::Prediction::FPrivateRoadSolidValues>()),OwnedWheelRuntime(std::make_unique<Speed::Netcode::Prediction::FPrivateRoadWheelRuntimeValues>()){OwnedSolidValues->bApplyRestForce=false;OwnedSolidValues->Mass=USWheelSubBody::DefaultWheelMass;}
bool USpeedBranchWheelSubBody::UsesOriginalBodyStorage() const,OwnedWheelDomain(std::make_unique<Speed::Netcode::Prediction::FPrivateRoadWheelDomain>()),OwnedSphereDomain(std::make_unique<Speed::Netcode::Prediction::FPrivateRoadSphereDomain>())
{
    OwnedBodyValues->SubBodyType=ESubBodyType::Wheel;OwnedBodyValues->IgnoredSubBodyTypes.AddUnique(ESubBodyType::Wheel);
    if(!IsInGameThread()||!Speed::Netcode::Prediction::OriginalComponentStorageOwner)return false;
    const auto A=StorageAccess.lock();
    return OwnedBodyValues&&A&&Speed::Netcode::Prediction::OriginalComponentStorageOwner==A.get();
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
bool USpeedBranchWheeledComponent::HasIsolatedNativeConstruction() const
{
    // Check actual stored pointers as well as the world's object census: an
    // inherited CDO reference is not an absent native network participant.
    return NetworkConstruction==ESpeedWheeledNetworkConstruction::IAmSpeedOnly
        &&!NetworkPhysicsComponent&&!SNetworkPhysicsComponentWheeledDomainStorage()&&!WheeledNetworkPhysicsComponentWheeledDomainStorage()
        &&!SNetworkSettingsWheeledDomainStorage()&&!NetDataAssetWheeledDomainStorage()&&!AHUD::OnShowDebugInfo.IsBoundToObject(this)
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
    OwnedComponentValues=std::make_unique<FOwnedComponentValues>();
}
SBaseGameState& USpeedBranchWheeledComponent::BaseGameStateStorage()
{
    return UsesOriginalComponentStorage()?OwnedComponentValues->BaseGameState:USpeedWheeledComponent::BaseGameStateStorage();
}
const SBaseGameState& USpeedBranchWheeledComponent::BaseGameStateStorage() const
{
    return UsesOriginalComponentStorage()?OwnedComponentValues->BaseGameState:USpeedWheeledComponent::BaseGameStateStorage();
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
void USpeedBranchBoxSubBody::ResolveCurrentHitPrv(const float& Delta,const float& SimTime)
{
    // Capture the genuine dispatch participant before Super can update caches.
    // Weak validity after the synchronous call never invents a lifetime lease.
    const bool Blocking=CurrentHitStorage().bBlockingHit;
    const TWeakObjectPtr<USolidSubBody> Peer=Cast<USolidSubBody>(CurrentHitStorage().Component.Get());
    Super::ResolveCurrentHitPrv(Delta,SimTime); // unchanged native solver, once
    if(Blocking&&Peer.IsValid())
    {
        // ISpeedComponent is native C++, NOT a UObject/UInterface. Resolve
        // through the genuine owning actor, then verify the adjusted native
        // base address rather than performing an invalid Unreal Cast.
        const auto* Car=Cast<ASpeedBranchCar>(GetOwner());
        auto* Owner=Car?Cast<USpeedBranchWheeledComponent>(Car->GetVehicleMovement()):nullptr;
        if(Owner&&GetParentComponent()==static_cast<ISpeedComponent*>(Owner))
            Owner->ObserveContactPair(*this,*Peer.Get(),Cast<UBoxSubBody>(Peer.Get())?0:1);
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
    const auto* OtherCar=Cast<ASpeedBranchCar>(Other.GetOwner());
    const auto* C=OtherCar?Cast<USpeedBranchWheeledComponent>(OtherCar->GetVehicleMovement()):nullptr;
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
bool ASpeedBranchSimulation::InitializePrivate(USpeedWorldSubsystem& Bridge)
{
    if(!IsInGameThread()||!Access.lock()||GetEngineFPS()!=300)return false;
    SpeedWorldSubsystemDriverStorage()=&Bridge;if(!EnsureSimulationWorldReady())return false;
    SealSimulationInputs();InitializeCanonicalFrame(0);return CanonicalNumFrameDriverStorage()==0;
}
bool ASpeedBranchSimulation::AdvancePrivate(bool Resimulation)
{
    const auto A=Access.lock();
    if(!IsInGameThread()||!A||!A->Active||A->Frame!=CanonicalNumFrameDriverStorage()||A->World!=GetWorld())return false;
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
