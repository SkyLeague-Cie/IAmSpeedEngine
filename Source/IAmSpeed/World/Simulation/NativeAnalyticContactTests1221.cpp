#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "IAmSpeed/World/Simulation/NativeExactInputFixture1304.h"
#include "IAmSpeed/Actors/SpeedCar.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/Camera/SpeedCarCameraPresentation.h"
#include "IAmSpeed/Input/Testing/WheeledTestProfile.h"
#include "IAmSpeed/World/Simulation/RealTimeSimulation.h"
#include "IAmSpeed/World/Simulation/FastSimulation.h"
#include "IAmSpeed/World/Simulation/CanonicalFrameContext.h"
#include "IAmSpeed/World/Analytic/JoinedTensorCellWave1213.h"
#include "Components/BoxComponent.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/SubBodies/Configs/SubBodyConfig.h"
#include "Math/QuatRotationTranslationMatrix.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/FileManager.h"
#include "IAmSpeed/World/Simulation/FramePresentationTasks.h"
#include "IAmSpeed/World/Simulation/FrameSolveWitness.h"
#include "IAmSpeed/World/Simulation/FrameParallelSettings.h"
#include "IAmSpeed/World/Simulation/JoinedActorKinematics1213.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "IAmSpeed/Base/EpisodeShapeFieldsForTesting.h"
#include "IAmSpeed/SubBodies/SSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "ChaosVehicleWheel.h"
#include "SimpleVehicle.h"
#include "SuspensionSystem.h"
#include "WheelSystem.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTLS.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include <atomic>
namespace
{
// Explicit fixture-owned controller only. It seals the finite run and has no
// game inputs, gameplay, callbacks or effects. It also owns one real native box
// to exercise canonical CCD/solve independently of suspension support. Car physics is
// performed solely by the original native components and canonical world.
class FContactRunControl1221 final:public ISpeedComponent
{
public:
 uint64 Published=0;uint32 Frame=0;uint32 Limit=90;SKinematic State;FVector Local=FVector::ZeroVector;TArray<USSubBody*> Bodies;
 SubBodyConfig Config;UBoxSubBody* Box=nullptr;FQuat Previous=FQuat::Identity;bool Frozen=false;
 TArray<uint64> PreparedFrames;TArray<float> PreparedDeltas;
 void Initialize(AActor& Actor)
 {
  PreparedFrames.Reserve(Limit);PreparedDeltas.Reserve(Limit);
  Config.bValid=true;Config.bIsMainSubBody=true;Config.BoxExtent=FVector(45,30,10);Config.Mass=10;
  Config.StaticFriction=0;Config.DynamicFriction=0;Config.Restitution=0;Config.ImpactThreshold=5;
  Box=NewObject<UBoxSubBody>(&Actor);Bodies.Add(Box);Box->Initialize(this);Actor.AddInstanceComponent(Box);
  State.Location=FVector(0,0,14);State.Velocity=FVector(0,0,-200);
 }
 void AppendNativeWitness(TArray<uint8>& O)const
 {
  using W=Speed::FEpisodeStateBytesForTesting;
  W::Add(O,Published);W::Add(O,Frame);W::Add(O,Limit);W::Array(O,PreparedFrames);W::Array(O,PreparedDeltas);W::Add(O,State);W::Add(O,Local);W::Add(O,Previous);W::Add(O,Frozen);
  W::Add(O,Config.bValid);W::Add(O,Config.bIsMainSubBody);W::Add(O,Config.Mass);W::Add(O,Config.BoxExtent);
  W::Add(O,Config.LocalOffset);W::Add(O,Config.LocalRotation);W::Add(O,Config.StaticFriction);W::Add(O,Config.DynamicFriction);W::Add(O,Config.Restitution);W::Add(O,Config.ImpactThreshold);
  W::Array(O,PhysicalConstraints);SleepState.AppendEpisodeFieldsForTesting<W>(O);KinematicQuantizationCache.AppendEpisodeFieldsForTesting<W>(O);
  W::Add(O,GetStaticCollisionWorldForFrame());W::Add(O,GetStaticSupportFrameHorizon());Box->AppendEpisodeStateForTesting(O);
 }
 bool IsCanonicalRunController()const override{return true;}
 bool IsCanonicalRunReady()const override{return true;}
 bool IsCanonicalRunComplete()const override{return Published>=Limit;}
 void PrepareCanonicalFrame(const FCanonicalFrameContext& C)override{Frame=uint32(C.NumFrame)+1;PreparedFrames.Add(C.NumFrame);PreparedDeltas.Add(C.PhysicalDeltaTime);State.Acceleration=GetNominalGravityAcceleration();State.AngularAcceleration=FVector::ZeroVector;}
 void OnCanonicalFramePublished(uint64 N)override{Published=N+1;}
 void AppendSimulationSnapshot(TArray<uint8>& O)const override
 {Speed::FEpisodeStateBytesForTesting::Add(O,Published);Speed::FEpisodeStateBytesForTesting::Add(O,Frame);}
 unsigned int NumFrame()const override{return Frame;}
 float GetPhysMass()const override{return Config.Mass;}
 const FVector& GetPhysCOM()const override{return State.Location;}
 const FVector& GetPhysCenterOfMassLocal()const override{return Local;}
 const TArray<USSubBody*>& GetSubBodies()const override{return Bodies;}
 TArray<USSubBody*> CreateSubBodies()override{return Bodies;}
 SubBodyConfig GetSubBodyConfig(const USSubBody&)const override{return Config;}
 bool IsFrozen()const override{return Frozen;}
 void SetIsFrozen(bool B)override{Frozen=B;}
 SKinematic GetKinematicsOfSubBody(const USSubBody&,const unsigned int& N)const override{return Box->GetKinematicsFromOwnerKS(GetOriginKinematicStateForFrame(N));}
 FMatrix ComputeWorldInvInertiaTensor()const override
 {
  const FVector E=Config.BoxExtent;FMatrix I=FMatrix::Identity;
  I.M[0][0]=3.0/(Config.Mass*(E.Y*E.Y+E.Z*E.Z));I.M[1][1]=3.0/(Config.Mass*(E.X*E.X+E.Z*E.Z));I.M[2][2]=3.0/(Config.Mass*(E.X*E.X+E.Y*E.Y));
  const FMatrix Rotation=FQuatRotationMatrix(State.Rotation);return Rotation.GetTransposed()*I*Rotation;
 }
 FMatrix ComputeWorldInvInertiaTensorOfSubBody(const USSubBody&)const override{return ComputeWorldInvInertiaTensor();}
 const SKinematic& GetKinematicState()const override{return State;}
 const SKinematic& GetKinematicStateForFrame(const unsigned int&)const override{return State;}
 void SetKinematicState(const SKinematic& S)override{State=S;}
 float GetPhysMaxSpeed()const override{return 10000;}
 float GetPhysMaxAngularSpeed()const override{return 100;}
 FVector GetNominalGravityAcceleration()const override{return FVector(0,0,-650);}
 void SetIsUpsideDown(bool)override{}
 bool IsUpsideDown()const override{return false;}
 bool IsInAutoRecover()const override{return false;}
 bool IsSubBodyInAutoRecoverMode()const override{return false;}
 void RcvImpactOnSubBody(const USSubBody&,const FVector&)override{}
 void PostPhysicsUpdatePrv(const float&)override{QuantizeKinematicState(State,Previous);Previous=State.Rotation;}
};
// Pure fixture-owned read-only receiver. PreForce is called from the REAL
// native PreparePhysicsFrame with its actual DeltaTime, after canonical N latches.
class FPrepareDeltaWitness1221 final:public ISpeedPhysicsPhaseObserver
{
public:
 TArray<uint64> Frames;TArray<float> Deltas;
 void Observe(const ISpeedComponent& Component,ESpeedPhysicsObservationPhase Phase,float Delta)override
 {
  if(Phase!=ESpeedPhysicsObservationPhase::PreForce)return;
  const uint32 Local=Component.NumFrame();Frames.Add(Local?uint64(Local)-1:MAX_uint64);Deltas.Add(Delta);
 }
};
class FContactCVar1221
{
 IConsoleVariable* V=nullptr;int32 Old=0;
public:
 FContactCVar1221(const TCHAR* N,int32 Value):V(IConsoleManager::Get().FindConsoleVariable(N)),Old(V?V->GetInt():0){if(V)V->Set(Value,ECVF_SetByCode);}
 ~FContactCVar1221(){if(V)V->Set(Old,ECVF_SetByCode);}
 bool Is(int32 Value)const{return V&&V->GetInt()==Value;}
};
class FNativeContactProfile1221 final:public Speed::Input::IInputProducer
{
public:
 explicit FNativeContactProfile1221(std::unique_ptr<Speed::Input::FTestInputProducer> P):Source(std::move(P)){}
 std::optional<Speed::Input::FInputFrame> Produce(Speed::Input::FFrameNumber N) override
 {++Calls;return Source->Produce(N);}
 uint32 Calls=0;
private:std::unique_ptr<Speed::Input::FTestInputProducer> Source;
};
struct FNativeContactRow1221
{
 TArray<uint8> Snapshot,ObservedStates,RawObservedStates,Inputs,OrderedSolve,RawOrderedSolve,CameraPose,CameraCheckpoint,PrepareDeltas;
 float ActualDelta=0;
 uint64 Frame=0,Publication=0;int32 CameraNative=0,PrepareNative=0,ShadowNative=0,MainContacts=0;
 uint64 TensorComputed=0,TensorReused=0,TensorWasted=0;
 int32 TensorUsefulNative=0,TensorReusedNative=0,TensorReusedLanes=0;
 bool bComplete=false;
};
void ContactBaseState1221(TArray<uint8>& O,const FBasePhysicsState& X)
{
 using W=Speed::FEpisodeStateBytesForTesting;
 W::Add(O,X.Kinematic);W::Add(O,X.bIsFrozen);W::Add(O,X.nbFramesbeforeCanMove);W::Add(O,X.bStartCountdown);
}
void ContactWheeledState1221(TArray<uint8>& O,const FWheeledPhysicsState& X)
{
 using W=Speed::FEpisodeStateBytesForTesting;
 W::Add(O,X.bStartCountdown);W::Add(O,X.nbFramesbeforeCanMove);W::Add(O,X.FramesSinceLastImpact);
 W::Add(O,X.AllowedSideVelocity);W::Add(O,X.AllowedAngularVelocity);W::Add(O,X.SuspensionLastDisplacement);
 W::Add(O,X.PhysicalThrottleInput);W::Add(O,X.PhysicalBrakeInput);W::Add(O,X.PhysicalSteerInput);W::Add(O,X.NbFramesSinceGroundContact);
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedNativeAnalyticContacts1221,
 "IAmSpeed.Simulation.FrameParallel.NativeAnalyticContacts1221",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedNativeAnalyticContacts1221::RunTest(const FString&)
{
 using namespace Speed;using namespace Speed::Input;using namespace Speed::Input::Testing;
 using W=FEpisodeStateBytesForTesting;
 constexpr uint32 Frames=90;constexpr int32 CarCount=2;
 if(!TestTrue(TEXT("native fixture starts on GT"),IsInGameThread()))return false;
 auto* LaneVar=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.Parallel.Lanes"));
 auto* MinVar=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.Parallel.MinPrepareItems"));
 auto* WaveVar=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.Parallel.ActorKinematics1213"));
 auto* Backend=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.StaticCollision.Backend"));
 if(!TestNotNull(TEXT("lane policy"),LaneVar)||!TestNotNull(TEXT("prepare grain"),MinVar)
  ||!TestNotNull(TEXT("native actor wave opt-in"),WaveVar)||!TestNotNull(TEXT("Analytic selector"),Backend))return false;
 const int32 OldLane=LaneVar->GetInt(),OldMin=MinVar->GetInt(),OldWave=WaveVar->GetInt(),OldBackend=Backend->GetInt();
 ON_SCOPE_EXIT{LaneVar->Set(OldLane,ECVF_SetByCode);MinVar->Set(OldMin,ECVF_SetByCode);WaveVar->Set(OldWave,ECVF_SetByCode);Backend->Set(OldBackend,ECVF_SetByCode);};
 if(!TestTrue(TEXT("native private workers available"),EnsurePrivateFrameWorkers(FFrameParallelPolicy::MaximumLanes-1)>0))return false;
 AddExpectedMessagePlain(TEXT("[WheelSuspensionBinding] subbodies="),ELogVerbosity::Warning,EAutomationExpectedMessageFlags::Contains,CarCount*10);
 const uint32 OldFPS=ASpeedSimulation::EngineFPS;ASpeedSimulation::EngineFPS=300;
 ON_SCOPE_EXIT{ASpeedSimulation::EngineFPS=OldFPS;};
 FContactCVar1221 Canonical(TEXT("p.IAmSpeed.CanonicalFrame.Enabled"),1),Host(TEXT("p.IAmSpeed.Simulation.ExecutionMode"),2);
 FContactCVar1221 Tensor(TEXT("p.IAmSpeed.Parallel.TensorCellWave1213"),1),Observe(TEXT("p.IAmSpeed.Parallel.TensorCellObserve1213"),1);
 FContactCVar1221 Minimum(TEXT("p.IAmSpeed.Parallel.TensorCellMinItems1213"),2),Grain(TEXT("p.IAmSpeed.Parallel.TensorCellMinItemsPerLane1213"),1),Chunk(TEXT("p.IAmSpeed.Parallel.TensorCellChunkItems1213"),1);
 FContactCVar1221 ProviderOff(TEXT("p.IAmSpeed.Parallel.PiecewiseProviderWave1075"),0);
 FContactCVar1221 Assignment(TEXT("p.IAmSpeed.Parallel.TensorCellDynamic1353"),1);
 if(!Canonical.Is(1)||!Host.Is(2)||!Tensor.Is(1)||!Observe.Is(1)||!Minimum.Is(2)||!Grain.Is(1)||!Chunk.Is(1)||!ProviderOff.Is(0)||!Assignment.Is(1))return false;
 const FString EvidenceRoot=FPaths::Combine(FPaths::ProjectSavedDir(),TEXT("IAmSpeed/Qualification1221"),FGuid::NewGuid().ToString(EGuidFormats::Digits));
 TArray<FNativeContactRow1221> Oracle;
 for(bool Fast:{false,true})for(int32 Lanes:{1,2,4,8,12})
 {
  FContactCVar1221 FastSelector(TEXT("p.IAmSpeed.FastSimulation.Enabled"),Fast?1:0);
  if(!FastSelector.Is(Fast?1:0))return false;
  LaneVar->Set(Lanes,ECVF_SetByCode);MinVar->Set(1,ECVF_SetByCode);WaveVar->Set(0,ECVF_SetByCode);Backend->Set(2,ECVF_SetByCode);
  const auto Options=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(false)
   .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false).SetTransactional(false);
  UWorld* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Options);
  if(!TestNotNull(TEXT("isolated native canonical multicars world"),World))return false;
  GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
  ON_SCOPE_EXIT{World->DestroyWorld(false);GEngine->DestroyWorldContext(World);};
  ASpeedSimulation* Driver=Fast?static_cast<ASpeedSimulation*>(World->SpawnActor<AFastSimulation>()):static_cast<ASpeedSimulation*>(World->SpawnActor<ARealTimeSimulation>());auto* Bridge=World->GetSubsystem<USpeedWorldSubsystem>();
  if(!TestNotNull(TEXT("canonical driver"),Driver)||!TestNotNull(TEXT("native registry"),Bridge))return false;
  Driver->SpeedWorldSubsystem=Bridge;
  Driver->ActiveExecutionModeValue.Store(static_cast<uint8>(ESimulationExecutionMode::IAmSpeedThread));
  FContactRunControl1221 Control;Control.Limit=Frames;AActor* BoxActor=World->SpawnActor<AActor>();Control.Initialize(*BoxActor);Bridge->RegisterSpeedComponent(&Control);
  ON_SCOPE_EXIT{Bridge->UnregisterSpeedComponent(&Control);Bridge->ApplyPendingOps();};
  auto Data=MakeShared<Analytic::FAnalyticWorldData>();
  Analytic::FPiecewiseTensorBezierPatch Patch;
  Patch.SourceId=1;Patch.SurfaceId=11;Patch.PrimitiveId=21;Patch.CanonicalGroupId=31;
  Patch.ObjectType=uint32(ECC_WorldStatic);Patch.BlockingChannels=MAX_uint64;Patch.bQueryCollisionEnabled=true;
  for(int32 Cell=0;Cell<64;++Cell)
  {
   auto& C=Patch.Cells.AddDefaulted_GetRef();C.FeatureId=1000+Cell;C.PrimitiveId=2000+Cell;
   C.MinimumU=double(Cell)/64;C.MaximumU=double(Cell+1)/64;C.Surface.DegreeU=1;C.Surface.DegreeV=1;
   const double X0=-80+2.5*Cell,X1=X0+2.5;
   C.Surface.ControlPoints={FVector3d(X0,-200,0),FVector3d(X0,200,0),FVector3d(X1,-200,0),FVector3d(X1,200,0)};
  }
  Data->PiecewiseTensorBezierPatches.Add(MoveTemp(Patch));FString GeometryReason;
  if(!TestTrue(TEXT("finite exact flat Tensor fixture validates"),Data->FinalizeAndValidate(&GeometryReason)))return false;
  Data->PiecewiseTensorBezierPatches[0].bAuthorityEligible=true; // authored fixture only, never a production certification.
  AActor* FloorActor=World->SpawnActor<AActor>();auto* Floor=NewObject<UBoxComponent>(FloorActor);
  FloorActor->AddInstanceComponent(Floor);Floor->SetMobility(EComponentMobility::Static);
  Floor->SetCollisionObjectType(ECC_WorldStatic);Floor->SetCollisionResponseToAllChannels(ECR_Block);
  Bridge->AnalyticSourceComponents.Add(1,Floor);
  Bridge->AnalyticWorldData=Data;Bridge->SimulationWorld.SetStaticCollisionWorld(MakeUnique<FAnalyticStaticCollisionWorld>(*Data,Lanes,1,1));
  Chaos::FSimpleWheelConfig WheelConfig;Chaos::FSimpleSuspensionConfig SuspensionConfig;
  SuspensionConfig.SetSuspensionMaxRaise(10);SuspensionConfig.SetSuspensionMaxDrop(25);
  TArray<USpeedWheeledComponent*> Cars;TArray<TUniquePtr<FPrepareDeltaWitness1221>> PrepareWitnesses;
  TArray<std::shared_ptr<FInputStream>> Streams;TArray<std::shared_ptr<FNativeContactProfile1221>> Profiles;
  ON_SCOPE_EXIT
  {
   for(auto* C:Cars)
   {
    C->SetFrameInputStream(nullptr);C->ClearWheelSimulationPointers();C->SkySimulation=nullptr;C->VehicleSimulationPT.Reset();
    Bridge->UnregisterSpeedComponent(C);
   }
   Bridge->ApplyPendingOps();
  };
  for(int32 A=0;A<CarCount;++A)
  {
   auto* Car=World->SpawnActor<ASpeedCar>();auto* C=Car?Cast<USpeedWheeledComponent>(Car->GetVehicleMovement()):nullptr;
   if(!TestNotNull(TEXT("actual native car"),C))return false;
   C->SetOwner(Car);C->DisableAsyncPhysicsTickForTesting();Cars.Add(C);
   auto DeltaWitness=MakeUnique<FPrepareDeltaWitness1221>();DeltaWitness->Frames.Reserve(Frames);DeltaWitness->Deltas.Reserve(Frames);PrepareWitnesses.Add(MoveTemp(DeltaWitness));
   auto Vehicle=MakeUnique<Chaos::FSimpleWheeledVehicle>();
   for(int32 I=0;I<4;++I){Vehicle->Wheels.Emplace(&WheelConfig);Vehicle->Suspension.Emplace(&SuspensionConfig);C->Wheels.Add(NewObject<UChaosVehicleWheel>(C));}
   C->VehicleSimulationPT=MakeUnique<USpeedSimulation>();C->SkySimulation=static_cast<USpeedSimulation*>(C->VehicleSimulationPT.Get());C->SkySimulation->Init(Vehicle);
   C->BindWheelSimulationPointers();
   FString Reason;if(!TestTrue(TEXT("all production wheel storage admitted"),C->ValidateSimulationBindings(Reason)))return false;
   C->SetEngineFPS(300);C->SetImmediateMovementForTesting();C->EnableGenericCameraInput(true);
   C->SetHeldCameraYaw(.125f);C->SetHeldCameraPitch(-.0625f);
   SKinematic K;K.Location=FVector(2500*A,125*A,10000+100*A);K.Velocity=FVector(300+A*7,25-A,15);
   K.Rotation=FQuat::Identity;K.AngularVelocity=FVector::ZeroVector;C->SetKinematicState(K);
   FWheeledTestProfile P;P.Identity={EProducerKind::Device,uint64(121700+A)};P.FrameCount=Frames;
   const float S=(A%2?-.5f:.5f);P.Keys={{0,.5f,0,S},{30,1.f,0,-S},{60,0,.25f,0}};
   auto Compiled=CompileWheeledTestProfile(P);
   if(!TestTrue(TEXT("distinct finite sealed profile compiles"),Compiled.Error==EWheeledProfileError::None&&bool(Compiled.Producer)))return false;
   auto Counted=std::make_shared<FNativeContactProfile1221>(std::move(Compiled.Producer));auto Stream=std::make_shared<FInputStream>(Counted);
   C->SetFrameInputStream(Stream);Profiles.Add(Counted);Streams.Add(Stream);
  }
  Bridge->ApplyPendingOps();
  if(!TestTrue(TEXT("driver admits immutable world and real cars"),Driver->EnsureSimulationWorldReady()))return false;
  Driver->InitializeCanonicalFrame(0);
  const uint64 MainId=Bridge->GetSimulationStableId(*Cars[0]);
  FSpeedCarCameraConfiguration Config;
  auto SerialCamera=MakeShared<FSpeedCarCameraPresentation,ESPMode::ThreadSafe>(MainId,0,1,Config,FSpeedCarCameraArm::DefaultState());
  auto WorkerCamera=MakeShared<FSpeedCarCameraPresentation,ESPMode::ThreadSafe>(MainId,0,1,Config,FSpeedCarCameraArm::DefaultState());
  // Exercise the real pause-acknowledged binding transaction. It opts into
  // presentation-body capture; bare producer registration intentionally does not.
  auto BoundCamera=Driver->BindPresentationAtFrameBoundary(*Cars[0],static_cast<ISpeedComponent*>(nullptr),
   [&](const FSimulationPresentationBinding& Binding)->TSharedPtr<ISimulationPresentationProducer,ESPMode::ThreadSafe>
   {
    if(Binding.OwnerStableId!=MainId||Binding.TargetStableId!=0||Binding.FirstFrame!=0)return nullptr;
    return SerialCamera;
   });
  // Release20's real AFastSimulation intentionally has no camera presentation.
  // Retain the production rejection; never force a camera onto the Fast driver.
  if(Fast)
  {
   if(!TestTrue(TEXT("Fast driver rejects camera binding and presentation capture"),
    !BoundCamera.IsValid()&&!Driver->bPublishPresentation&&!Driver->AllowsCameraPresentation()))return false;
  }
  else
  {
  if(!TestTrue(TEXT("real driver camera binding includes presentation capture"),
   BoundCamera.IsValid()&&BoundCamera.Get()==&SerialCamera.Get()&&Driver->bPublishPresentation))return false;
  }
  ON_SCOPE_EXIT{Driver->UnregisterPresentationProducer(SerialCamera);};
  const FSpeedCarCameraCommand Cmd{25,1,ESpeedCarCameraCommandKind::Distance,320};
  if(!TestTrue(TEXT("same exact-frame camera command queued"),SerialCamera->QueueAt(Cmd)&&WorkerCamera->QueueAt(Cmd)))return false;
  // Canonical N maps to the existing component-local physical frame N+1.
  // Distinct immutable commands are authored on GT before starting the owner.
  // Camera workers and physical frames never poll that live GT mailbox.
  TArray<FSpeedCarCameraPhysicalInput> AuthoredCameraInputs;AuthoredCameraInputs.SetNum(Frames);
  for(uint32 N=0;N<Frames;++N)
   if(!TestTrue(TEXT("finite distinct camera command quantizes"),FSpeedCarCameraPhysicalInput::Quantize(false,
    float(int32(N%31)-15)/127.f,float(int32(N%17)-8)/127.f,AuthoredCameraInputs[N])))return false;
  for(auto* C:Cars)
  {
   C->EnableGenericCameraInput(true);
   if(!TestTrue(TEXT("camera window fits the real bounded queue"),int32(Frames)<=C->MaxPendingCameraInputs)
    ||!TestEqual(TEXT("initialization did not consume camera commands"),C->CameraInputApplyCount,uint64(0)))return false;
   for(uint32 N=0;N<Frames;++N)C->QueueCameraInputForFrame(int32(N+1),AuthoredCameraInputs[N]);
   if(!TestEqual(TEXT("all finite camera commands admitted before native start"),C->PendingCameraInputCommands.Num(),int32(Frames)))return false;
  }
  // Finite values are authored before start; actual V2 owners are constructed,
  // polled, validated and retired on the same native canonical worker as production.
  Speed::Input::V2::FInputActionContractDescription Description;
  Description.Revision={1};Description.Actions=Speed::Input::V2::FInputActionContract::BaseActions();
  Description.Physical={{Speed::Input::Throttle,Speed::Input::V2::EPhysicalDestination::Throttle},
   {Speed::Input::Brake,Speed::Input::V2::EPhysicalDestination::Brake},{Speed::Input::Steering,Speed::Input::V2::EPhysicalDestination::Steering}};
  auto Contract=Speed::Input::V2::FInputActionContract::Create(Description);
  if(!TestTrue(TEXT("native car input contract"),bool(Contract)))return false;
  TArray<Speed::Input::V2::FSessionDescriptor> ExactDescriptors;
  for(int32 A=0;A<Cars.Num();++A)
  {
   Speed::Input::V2::FSessionDescriptor Binding;Binding.Epoch=1;Binding.First=0;
   Binding.Contract=Contract;Binding.Kind=Speed::Input::V2::EProducerContract::ExactScenario;
   const uint64 ActorId=Bridge->GetSimulationStableId(*Cars[A]);Binding.Actors={{ActorId,ActorId}};
   for(uint32 N=0;N<Frames;++N)
   {
    const auto Legacy=Profiles[A]->Produce(N);if(!TestTrue(TEXT("one authored immutable command"),Legacy.has_value()))return false;
    Speed::Input::V2::FInputFrameData D;D.SourceSequence=N+1;D.ConsumptionFrame=N;
    D.Producer=Legacy->GetProducer();D.StreamEpoch={1};D.DeviceGeneration={1};D.Reset=N==0;D.Values=Legacy->GetActions();
    for(std::size_t K=0;K<D.Values.size();++K)if(D.Values[K])D.ActiveMask|=uint32(1)<<K;
    // ExactScenario carries a complete, ordered activity history, including analog starts/releases.
    if(N)
    {
     const uint32 PreviousMask=Binding.Scenario.back().GetData().ActiveMask;uint32 WithinSequence=0;
     for(std::size_t K=0;K<D.Values.size();++K)if(((PreviousMask^D.ActiveMask)&(uint32(1)<<K))!=0)
      D.Transitions.push_back({static_cast<Speed::Input::FActionId>(K),
       (D.ActiveMask&(uint32(1)<<K))?Speed::Input::V2::ETransition::Started:Speed::Input::V2::ETransition::Completed,
       D.Values[K],{D.SourceSequence,WithinSequence++}});
    }
    Binding.Scenario.emplace_back(Contract,D);
    if(!TestTrue(TEXT("complete authored V2 command accepted by exact contract"),Binding.Scenario.back().IsValidFor(*Contract)))return false;
   }
   Binding.Id=Binding.Controller=Binding.Producer=Binding.Scenario.front().GetData().Producer.Id;
   Cars[A]->SetFrameInputStream(nullptr);
   if(!TestTrue(TEXT("claim actual canonical input authority"),Cars[A]->SetFrameInputStreamV2(nullptr)))return false;
   Speed::Input::V2::FBoundaryCommandDescriptor Command;Command.Id=Driver->AllocateInputCommandId();
   Command.WorkerGeneration=Driver->GetInputWorkerGeneration();Command.RegistryVersion=uint64(A);Command.Binding=Binding;
   if(!TestTrue(TEXT("inert exact timeline queued for actual worker"),Driver->SubmitInputSessionCommand(Command)==Speed::Input::V2::ECommandAdmission::Enqueued))return false;
   ExactDescriptors.Add(MoveTemp(Binding));
  }
  Driver->ResumeOwnedSimulation();
  TArray<TSharedRef<ISimulationPresentationProducer,ESPMode::ThreadSafe>> WorkerList{WorkerCamera};
  TArray<FNativeContactRow1221> Seen;Seen.SetNum(Frames);uint32 Steps=0,OwnerThread=0;int32 FailureLine=0,CameraStatus=-1,CameraReason=-1;FString PointerFailure;bool Failed=false;
  FSimulationWorker Owner([&]
  {
   if(Steps==Frames)return ESimulationWorkerResult::Complete;
   OwnerThread=FPlatformTLS::GetCurrentThreadId();const uint64 N=Driver->CanonicalNumFrame;auto& Row=Seen[Steps];
   FrameParallel::ResetJoinedActorKinematicShadowTotals1213();
   ResetFrameParallelStageTotals(EFrameParallelStage::Prepare);
   ResetFrameParallelStageTotals(EFrameParallelStage::Camera);
   Analytic::ResetTensorCellWaveStats1213();
   if(FFrameSolveWitnessScope::IsActive()){Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
   FFramePointerRegistry Pointers;
   const auto Declare=[&](const auto* P,uint32 Domain,uint64 OwnerId,uint64 Role,uint64 Element)
   {if(!P)return true;FFramePointerKey Key;Key.Domain=Domain;Key.Owner=OwnerId;Key.Generation=1;Key.Role=Role;Key.Element=Element;return Pointers.Register(P,Key);};
   bool Identities=Declare(&Control,1,Bridge->GetSimulationStableId(Control),0,0)
    &&Declare(Control.Box,3,Bridge->GetSimulationStableId(Control),0,0)
    &&Declare(BoxActor,2,Bridge->GetSimulationStableId(Control),1,0)
    &&Declare(Floor,4,1,0,0)&&Declare(FloorActor,5,1,0,0)
    &&Declare(Bridge->SimulationWorld.GetStaticCollisionWorld(),6,1,0,0);
   for(auto* C:Cars)
   {
    const uint64 Id=Bridge->GetSimulationStableId(*C);
    Identities&=Declare(static_cast<ISpeedComponent*>(C),1,Id,0,0);
    if(reinterpret_cast<const void*>(C)!=reinterpret_cast<const void*>(static_cast<ISpeedComponent*>(C)))Identities&=Declare(C,2,Id,0,0);
    Identities&=Declare(C->GetOwner(),2,Id,1,0);
    const auto* Root=C->GetOwner()->GetRootComponent();bool RootIsBody=false;
    int32 Child=0;for(auto* B:C->GetSubBodies()){RootIsBody|=Root==B;Identities&=Declare(B,3,Id,0,uint64(Child++));}
    if(!RootIsBody)Identities&=Declare(Root,2,Id,2,0);
    int32 WheelIndex=0;
    for(const auto& Body:C->GetWheelSubBodies())
    {
     const auto* Wheel=Body.Get();
     if(!Wheel||!Wheel->GetChaosWheelConfiguration()||!Wheel->GetWheelSim()||!Wheel->GetSuspensionSim()){Identities=false;break;}
     Identities&=Declare(Wheel->GetChaosWheelConfiguration(),7,Id,0,uint64(WheelIndex));
     Identities&=Declare(Wheel->GetWheelSim(),8,Id,0,uint64(WheelIndex));
     Identities&=Declare(Wheel->GetSuspensionSim(),9,Id,0,uint64(WheelIndex));++WheelIndex;
    }
   }
   if(!Identities||!Pointers.Seal()){Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
   FFrameSolveWitnessScope Ordered(N,Row.RawOrderedSolve,Pointers);
   // Install/remove on this same canonical owner, outside the physical call.
   // Never replace an existing observer, including on an Idle or failure pulse.
   int32 InstalledPrepareObservers=0;
   ON_SCOPE_EXIT
   {
    for(int32 A=0;A<InstalledPrepareObservers;++A)
     if(!Cars[A]->RemovePhysicsPhaseObserver(*PrepareWitnesses[A])){Failed=true;FailureLine=__LINE__;}
   };
   for(int32 A=0;A<Cars.Num();++A)
   {
    if(!Cars[A]->TryInstallPhysicsPhaseObserver(*PrepareWitnesses[A])){Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
    ++InstalledPrepareObservers;
   }
   const auto Pulse=Driver->DriveOwnedWorkerPulse(1.f/300,float(N)/300);
   if(Driver->CanonicalNumFrame==N&&Pulse==ESimulationWorkerResult::Idle)return ESimulationWorkerResult::Idle;
   if((Pulse!=ESimulationWorkerResult::Advanced&&Pulse!=ESimulationWorkerResult::Complete)||Driver->CanonicalNumFrame!=N+1)
   {Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
   const auto NumericAfterPhysics=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext ObservationGuard(NumericAfterPhysics);
   if(!ObservationGuard.IsApplied()){Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
   const float RequiredDelta=1.f/300;
   if(Control.PreparedFrames.Num()!=int32(N+1)||Control.PreparedDeltas.Num()!=int32(N+1)||Control.PreparedFrames.Last()!=N
    ||FMemory::Memcmp(&Control.PreparedDeltas.Last(),&RequiredDelta,sizeof(float))!=0)
   {Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
   Row.ActualDelta=Control.PreparedDeltas.Last();W::Add(Row.PrepareDeltas,uint32(1));W::Add(Row.PrepareDeltas,uint32(CarCount+1));
   W::Add(Row.PrepareDeltas,Bridge->GetSimulationStableId(Control));W::Add(Row.PrepareDeltas,N);W::Add(Row.PrepareDeltas,Row.ActualDelta);
   for(int32 A=0;A<Cars.Num();++A)
   {
    const auto& PW=*PrepareWitnesses[A];
    if(PW.Frames.Num()!=int32(N+1)||PW.Deltas.Num()!=int32(N+1)||PW.Frames.Last()!=N
     ||FMemory::Memcmp(&PW.Deltas.Last(),&RequiredDelta,sizeof(float))!=0)
    {Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
    W::Add(Row.PrepareDeltas,Bridge->GetSimulationStableId(*Cars[A]));W::Add(Row.PrepareDeltas,N);W::Add(Row.PrepareDeltas,PW.Deltas.Last());
   }
   FSimulationSnapshot Snapshot;
   if(!Driver->ReadLatestSimulationSnapshot(Snapshot)||Snapshot.NumFrame!=N||Snapshot.PresentationOutputs.Num()!=(Fast?0:1))
   {Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
   Row.Frame=N;Row.Publication=Snapshot.PublicationSerial;Row.Snapshot=Snapshot.Payload;
   Row.MainContacts=Ordered.NumMainRows();TArray<uint8> Canonical;
   if(!Ordered.MakeCanonical(Canonical))
   {
    PointerFailure=FString::Printf(TEXT("Solve N=%llu Overflow=%d Unknown=%d OpenPasses=%d\nOffset\tRaw\tKnown\tObjectId\tPair\tSolverPair\tProvider\tFeatureSalt\n"),N,Ordered.HasOverflow()?1:0,Ordered.HasUnknownPointer()?1:0,Ordered.OpenDynamicPassCount());
    for(const auto& E:Ordered.PointerObservations())PointerFailure+=FString::Printf(TEXT("%d\t%llu\t%d\t%d\t%d\t%d\t%llu\t%llu\n"),E.Offset,uint64(E.RawAddress),E.bKnown?1:0,E.bObjectId?1:0,E.bPairToken?1:0,E.bSolverPairToken?1:0,E.PairProviderId,E.PairFeatureSalt);
    Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;
   }Row.OrderedSolve=MoveTemp(Canonical);
   TArray<uint8>& RawStates=Row.RawObservedStates;FFramePointerCaptureScope StatePointers(RawStates,Pointers);
   Control.AppendNativeWitness(RawStates);
   for(int32 A=0;A<Cars.Num();++A)
   {
    const auto CompletedInput=Driver->ReadCompletedInputFrame();
    if(!CompletedInput||CompletedInput->Frame!=N||CompletedInput->Inputs.size()!=std::size_t(CarCount)||!Driver->InputSessionRegistry)
    {Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
    const auto& ExpectedInput=ExactDescriptors[A];const Speed::Input::V2::FInstalledSessionInput* ActualInput=nullptr;
    for(const auto& Entry:CompletedInput->Inputs)if(Entry.Session==ExpectedInput.Id){if(ActualInput){Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}ActualInput=&Entry;}
    if(!ActualInput||!ActualInput->Snapshot||ActualInput->Epoch!=ExpectedInput.Epoch||ActualInput->Actors!=ExpectedInput.Actors
     ||!Speed::Testing1304::SameExactInput(ActualInput->Snapshot->Input,ExpectedInput.Scenario[N])
     ||Driver->InputSessionRegistry->PollCount(ExpectedInput.Id)!=N+1
     ||Cars[A]->CameraInputApplyCount!=N+1||Cars[A]->CameraMailboxReadCount!=0||!Cars[A]->WheeledPhysicalInput.Camera.Equals(AuthoredCameraInputs[N]))
    {Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
    const auto& ActualApplied=ActualInput->Snapshot->Applied;
    if(Cars[A]->WheeledPhysicalInput.Throttle!=ActualApplied[Speed::Input::Throttle]
     ||Cars[A]->WheeledPhysicalInput.Brake!=ActualApplied[Speed::Input::Brake]||Cars[A]->WheeledPhysicalInput.Steer!=ActualApplied[Speed::Input::Steering])
    {Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
    W::Add(Row.Inputs,ExpectedInput.Id);W::Add(Row.Inputs,ActualInput->Epoch);W::Add(Row.Inputs,N);
    W::Add(Row.Inputs,Driver->InputSessionRegistry->PollCount(ExpectedInput.Id));
    Speed::Testing1304::AddExactInput(Row.Inputs,ActualInput->Snapshot->Input,ActualApplied);
    W::Add(Row.Inputs,Cars[A]->CameraInputApplyCount);W::Add(Row.Inputs,Cars[A]->CameraMailboxReadCount);
    const auto& CameraApplied1304=Cars[A]->WheeledPhysicalInput.Camera;const auto& CameraAuthored1304=AuthoredCameraInputs[N];
    W::Add(Row.Inputs,CameraApplied1304.Version);W::Add(Row.Inputs,CameraApplied1304.Flags);W::Add(Row.Inputs,CameraApplied1304.Yaw);W::Add(Row.Inputs,CameraApplied1304.Pitch);
    W::Add(Row.Inputs,CameraAuthored1304.Version);W::Add(Row.Inputs,CameraAuthored1304.Flags);W::Add(Row.Inputs,CameraAuthored1304.Yaw);W::Add(Row.Inputs,CameraAuthored1304.Pitch);
    const auto& Applied=Cars[A]->WheeledPhysicalInput;
    W::Add(Row.Inputs,Applied.bCanMove);W::Add(Row.Inputs,Applied.Throttle);W::Add(Row.Inputs,Applied.Brake);W::Add(Row.Inputs,Applied.Steer);
    W::Add(Row.Inputs,Applied.Camera.Version);W::Add(Row.Inputs,Applied.Camera.Flags);W::Add(Row.Inputs,Applied.Camera.Yaw);W::Add(Row.Inputs,Applied.Camera.Pitch);
    auto* C=Cars[A];ContactBaseState1221(RawStates,C->BasePhysicsState);ContactWheeledState1221(RawStates,C->WheeledPhysicsState);
    W::Add(RawStates,C->BaseGameState.NumFrame);W::Add(RawStates,C->BaseGameState.TestVelocity);
    W::Add(RawStates,C->BaseGameState.TestAngularVelocity);W::Add(RawStates,C->BaseGameState.bHasTestVelocity);
    W::Add(RawStates,C->WheeledGameState.bGroundState);W::Add(RawStates,C->WheeledGameState.bIsUpsideDown);
    W::Add(RawStates,C->WheeledGameState.WheelsOmega);W::Add(RawStates,C->WheeledGameState.WheelsAngularPosition);
    W::Add(RawStates,C->WheeledGameState.NbWheelsOnGround);W::Add(RawStates,C->WheeledGameState.NbFrameSinceInAir);
    W::Add(RawStates,C->CarLocalInvI);W::Add(RawStates,C->RecordedBaseFrames.Num());
    for(int32 F:C->RecordedBaseFrames)W::Add(RawStates,F);
    W::Add(RawStates,C->RecordedBaseStates.Num());for(const auto& X:C->RecordedBaseStates)ContactBaseState1221(RawStates,X);
    W::Add(RawStates,C->RecordedWheeledStates.Num());for(const auto& X:C->RecordedWheeledStates)ContactWheeledState1221(RawStates,X);
    W::Array(RawStates,C->PhysicalConstraints);
    C->SleepState.AppendEpisodeFieldsForTesting<W>(RawStates);C->KinematicQuantizationCache.AppendEpisodeFieldsForTesting<W>(RawStates);
    W::Add(RawStates,C->GetStaticCollisionWorldForFrame());W::Add(RawStates,C->GetStaticSupportFrameHorizon());
    for(auto* B:C->GetSubBodies())B->AppendEpisodeStateForTesting(RawStates);
   }
   if(!StatePointers.MakeCanonical(Row.ObservedStates))
   {
    PointerFailure=FString::Printf(TEXT("N=%llu Overflow=%d Unknown=%d\nOffset\tAddress\tKnown\tObjectId\tPairToken\n"),N,StatePointers.HasOverflow()?1:0,StatePointers.HasUnknown()?1:0);
    for(const auto& E:StatePointers.Observations())PointerFailure+=FString::Printf(TEXT("%d\t%llu\t%d\t%d\t%d\n"),E.Offset,uint64(E.RawAddress),E.bKnown?1:0,E.bObjectId?1:0,E.bPairToken?1:0);
    Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;
   }
   // Save actual production output and participation before running the
   // independent comparison, which must not inflate production worker evidence.
   if(!Fast)
   {
   Row.CameraPose=Snapshot.PresentationOutputs[0].Payload;
   Row.CameraCheckpoint=Snapshot.PresentationOutputs[0].StatePayload;
   Row.CameraNative=GetFrameParallelStageTotals(EFrameParallelStage::Camera).MaximumNativeThreads;
   const auto Expected=Snapshot.PresentationOutputs[0];Snapshot.PresentationOutputs.Reset();
   const auto Validate=[&]
   {
    for(int32 A=0;A<Cars.Num();++A)
    {if(!Driver->InputSessionRegistry||Driver->InputSessionRegistry->PollCount(ExactDescriptors[A].Id)!=N+1)return false;}
    return true;
   };
   if(!FrameParallel::ProduceFramePresentationWithOwnerValidation(Snapshot,WorkerList,Lanes,true,Validate)
    ||Snapshot.PresentationOutputs.Num()!=1||Expected.Payload!=Snapshot.PresentationOutputs[0].Payload
    ||Expected.StatePayload!=Snapshot.PresentationOutputs[0].StatePayload)
   {Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
   FSpeedCarCameraPose Pose;
   const bool PoseRead=FSpeedCarCameraPose::Read(Snapshot.PresentationOutputs[0].Payload,Pose);
   if(PoseRead){CameraStatus=int32(Pose.Result.Status);CameraReason=int32(Pose.Result.Reason);}
   if(!PoseRead||Pose.Frame!=N||Pose.Result.Status!=ESpeedCarCameraStatus::Valid)
   {Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}

   }
   else
   {
    Row.CameraNative=GetFrameParallelStageTotals(EFrameParallelStage::Camera).MaximumNativeThreads;
    if(Snapshot.PresentationOutputs.Num()!=0||Row.CameraNative!=0)
    {Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
   }

   Row.PrepareNative=GetFrameParallelStageTotals(EFrameParallelStage::Prepare).MaximumNativeThreads;
   Row.ShadowNative=FrameParallel::GetJoinedActorKinematicShadowTotals1213().MaximumShadowNativeParticipants;
   const auto TensorStats=Analytic::SnapshotTensorCellWaveStats1213();
   Row.TensorComputed=TensorStats.Computed;Row.TensorReused=TensorStats.Reused;Row.TensorWasted=TensorStats.Wasted;
   Row.TensorUsefulNative=TensorStats.UsefulNativeMax;Row.TensorReusedNative=TensorStats.ReusedNativeMax1213;Row.TensorReusedLanes=TensorStats.ReusedLaneMax1213;
   if(!ObservationGuard.Restore()){Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
   Row.bComplete=true;++Steps;return Steps==Frames?ESimulationWorkerResult::Complete:ESimulationWorkerResult::Advanced;
  },[&](FSimulationWorkerWaitContext& Wait){Driver->WaitBetweenFrames(Wait);},
   [&](bool Paused){return Driver->ServiceInputSessionBoundary(Paused);},[&]{Driver->CloseInputSessionsOnWorker();});
  ON_SCOPE_EXIT{Owner.StopAndJoin();};
  const uint32 GT=FPlatformTLS::GetCurrentThreadId();if(!TestTrue(TEXT("actual native canonical owner starts"),Owner.Start()))return false;
  const double Deadline=FPlatformTime::Seconds()+60;
  while(Owner.IsRunning()&&FPlatformTime::Seconds()<Deadline)FPlatformProcess::SleepNoStats(.001f);
  const bool Finished=!Owner.IsRunning();Owner.StopAndJoin();
  if(Failed)AddInfo(FString::Printf(TEXT("[NativeAnalyticContactsFirstFailure1297] Fast=%d Lanes=%d Steps=%u Line=%d CameraStatus=%d CameraReason=%d"),Fast?1:0,Lanes,Steps,FailureLine,CameraStatus,CameraReason));
  // Save ALL raw/canonical observations only after joining, so file I/O is
  // excluded from physical frames and an UNKNOWN keeps its original bytes.
  const FString CaseDir=FPaths::Combine(EvidenceRoot,FString::Printf(TEXT("Fast%d-W%d"),Fast?1:0,Lanes));
  if(IFileManager::Get().DirectoryExists(*CaseDir))
  {AddError(FString::Printf(TEXT("[NativeAnalyticContacts1221ExportRefused] Existing directory %s; no journal overwritten"),*CaseDir));return false;}
  bool EvidenceSaved=IFileManager::Get().MakeDirectory(*CaseDir,true);
  const auto SavePack=[&](const TCHAR* Name,auto Member)
  {
   TArray<uint8> Packed;W::Add(Packed,uint32(0x31434149));W::Add(Packed,uint32(Frames));
   for(uint32 N=0;N<Frames;++N){const auto& Bytes=Seen[N].*Member;W::Add(Packed,uint64(N));W::Add(Packed,uint32(Bytes.Num()));Packed.Append(Bytes);}
   const FString Path=FPaths::Combine(CaseDir,Name);
   if(IFileManager::Get().FileExists(*Path)){AddError(FString::Printf(TEXT("[NativeAnalyticContacts1221ExportRefused] Existing file %s"),*Path));return false;}
   return FFileHelper::SaveArrayToFile(Packed,*Path,&IFileManager::Get(),FILEWRITE_NoReplaceExisting);
  };
  if(!PointerFailure.IsEmpty())EvidenceSaved&=FFileHelper::SaveStringToFile(PointerFailure,*FPaths::Combine(CaseDir,TEXT("pointers.first-failure.tsv")),FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM,&IFileManager::Get(),FILEWRITE_NoReplaceExisting);
  EvidenceSaved&=SavePack(TEXT("contacts.raw.pack"),&FNativeContactRow1221::RawOrderedSolve);
  EvidenceSaved&=SavePack(TEXT("contacts.canonical.pack"),&FNativeContactRow1221::OrderedSolve);
  EvidenceSaved&=SavePack(TEXT("states.raw.pack"),&FNativeContactRow1221::RawObservedStates);
  EvidenceSaved&=SavePack(TEXT("states.canonical.pack"),&FNativeContactRow1221::ObservedStates);
  EvidenceSaved&=SavePack(TEXT("commands.consumed.pack"),&FNativeContactRow1221::Inputs);
  EvidenceSaved&=SavePack(TEXT("snapshots.pack"),&FNativeContactRow1221::Snapshot);
  EvidenceSaved&=SavePack(TEXT("camera.pose.pack"),&FNativeContactRow1221::CameraPose);
  EvidenceSaved&=SavePack(TEXT("camera.checkpoint.pack"),&FNativeContactRow1221::CameraCheckpoint);
  EvidenceSaved&=SavePack(TEXT("prepare.deltas.pack"),&FNativeContactRow1221::PrepareDeltas);
  // Preserve the ACTUAL complete argument histories even if a dt/count gate
  // rejected a frame before its canonical per-row witness could be emitted.
  TArray<uint8> PrepareHistories;W::Add(PrepareHistories,uint32(1));W::Add(PrepareHistories,uint32(CarCount+1));
  const auto AppendHistory=[&](uint64 Id,const TArray<uint64>& Ns,const TArray<float>& Dts)
  {W::Add(PrepareHistories,Id);W::Array(PrepareHistories,Ns);W::Array(PrepareHistories,Dts);};
  AppendHistory(Bridge->GetSimulationStableId(Control),Control.PreparedFrames,Control.PreparedDeltas);
  for(int32 A=0;A<Cars.Num();++A)AppendHistory(Bridge->GetSimulationStableId(*Cars[A]),PrepareWitnesses[A]->Frames,PrepareWitnesses[A]->Deltas);
  const FString HistoryPath=FPaths::Combine(CaseDir,TEXT("prepare.histories.raw"));
  if(IFileManager::Get().FileExists(*HistoryPath)){AddError(FString::Printf(TEXT("[NativeAnalyticContacts1221ExportRefused] Existing file %s"),*HistoryPath));EvidenceSaved=false;}
  else EvidenceSaved&=FFileHelper::SaveArrayToFile(PrepareHistories,*HistoryPath,&IFileManager::Get(),FILEWRITE_NoReplaceExisting);
  FString Rows=FString::Printf(TEXT("Fast=%d Requested=%d Host=2 Hz=300 Driver=%s Finished=%d Failed=%d FailureLine=%d Steps=%u Scope=IsolatedNativeFixtureFullStadiumUNKNOWN\nN\tFrame\tPublication\tComplete\tMainContacts\tComputed\tReused\tWasted\tUsefulNative\tReusedNative\tReusedLanes\tActualPrepareDelta\n"),Fast?1:0,Lanes,*Driver->GetClass()->GetName(),Finished?1:0,Failed?1:0,FailureLine,Steps);
  for(uint32 N=0;N<Frames;++N){const auto& Row=Seen[N];Rows+=FString::Printf(TEXT("%u\t%llu\t%llu\t%d\t%d\t%llu\t%llu\t%llu\t%d\t%d\t%d\t%.17g\n"),N,Row.Frame,Row.Publication,Row.bComplete?1:0,Row.MainContacts,Row.TensorComputed,Row.TensorReused,Row.TensorWasted,Row.TensorUsefulNative,Row.TensorReusedNative,Row.TensorReusedLanes,double(Row.ActualDelta));}
  const FString RowPath=FPaths::Combine(CaseDir,TEXT("rows.tsv"));
  if(IFileManager::Get().FileExists(*RowPath)){AddError(FString::Printf(TEXT("[NativeAnalyticContacts1221ExportRefused] Existing file %s"),*RowPath));EvidenceSaved=false;}
  else EvidenceSaved&=FFileHelper::SaveStringToFile(Rows,*RowPath,FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM,&IFileManager::Get(),FILEWRITE_NoReplaceExisting);
  AddInfo(FString::Printf(TEXT("[NativeAnalyticContacts1221Evidence] Path=%s Saved=%d"),*CaseDir,EvidenceSaved?1:0));
  if(!TestTrue(TEXT("all raw and canonical evidence persisted after join"),EvidenceSaved))return false;
  if(!TestTrue(TEXT("finite native canonical run completes"),Finished&&!Failed&&Steps==Frames)
   ||!TestTrue(TEXT("physics owner is independent of GT"),OwnerThread!=0&&OwnerThread!=GT))return false;
  for(int32 A=0;A<CarCount;++A)TestEqual(TEXT("one immutable input authored per frame per car before start"),Profiles[A]->Calls,Frames);
  uint64 TotalContacts=0,Computed=0,Reused=0,Wasted=0;int32 UsefulNative=0,ReusedNative=0,ReusedLanes=0;
  for(uint32 N=0;N<Frames;++N)
  {
   const auto& Row=Seen[N];
   if(!TestTrue(TEXT("exact N and publication cover every300Hz frame"),Row.bComplete&&Row.Frame==N&&Row.Publication==N+1))return false;
   if(!Fast&&Lanes>1&&!TestTrue(TEXT("real camera calculation ran on worker plus owner validation"),Row.CameraNative>1))return false;
   TotalContacts+=Row.MainContacts;Computed+=Row.TensorComputed;Reused+=Row.TensorReused;Wasted+=Row.TensorWasted;
   UsefulNative=FMath::Max(UsefulNative,Row.TensorUsefulNative);ReusedNative=FMath::Max(ReusedNative,Row.TensorReusedNative);ReusedLanes=FMath::Max(ReusedLanes,Row.TensorReusedLanes);
   if(Fast||Lanes>1)
   {
    const auto& Ref=Oracle[N];
    if(!TestTrue(FString::Printf(TEXT("first divergence at N%u fast%d lanes%d: snapshot/input/listed states/ordered contacts/camera"),N,Fast?1:0,Lanes),
     Row.Snapshot==Ref.Snapshot&&Row.Inputs==Ref.Inputs&&Row.ObservedStates==Ref.ObservedStates
     &&Row.OrderedSolve==Ref.OrderedSolve&&Row.PrepareDeltas==Ref.PrepareDeltas&&(!Fast||(Row.CameraPose==Ref.CameraPose&&Row.CameraCheckpoint==Ref.CameraCheckpoint))))return false;
   }
  }
  TestTrue(TEXT("fixture exercised actual selected physical contacts"),TotalContacts>0);
  TestEqual(TEXT("every completed speculative kernel accounted as reused or discarded"),Computed,Reused+Wasted);
  if(Lanes>1)TestTrue(TEXT("actual consumed Tensor work used multiple native threads and logical lanes"),Computed>0&&Reused>0&&UsefulNative>1&&ReusedNative>1&&ReusedLanes>1);
  AddInfo(FString::Printf(TEXT("[NativeAnalyticContacts1221] Fast=%d Requested=%d Frames=%u Hz=300 SelectedContacts=%llu Computed=%llu Reused=%llu Wasted=%llu UsefulNativeMax=%d ReusedNativeMax=%d ReusedLaneMax=%d"),Fast?1:0,Lanes,Frames,TotalContacts,Computed,Reused,Wasted,UsefulNative,ReusedNative,ReusedLanes));
  if(!Fast&&Lanes==1)Oracle=MoveTemp(Seen);
 }
 AddInfo(TEXT("Release20 camera contract: Normal camera outputs/checkpoints remain exactly equal across lanes and the independent oracle; Fast binding is rejected and every production camera output/native participant must remain empty/zero. Full physical/input/contact/wheel/state byte comparisons against NormalW1 remain mandatory. Cross-mode camera equality is intentionally not claimed."));
 AddInfo(TEXT("Scope: isolated plugin fixture, two actual native cars and actual camera, plus an explicitly configured native box/controller using production integration/CCD/solve; one authored64cell flat Tensor provider; finite90frame300Hz controller; real Normal/Fast owned pulses with selector0/1,host2;actual full-delta arguments from box Prepare and native car PreForce checked bitwise1/300 on exactly90 consecutive frames; actual consumed car commands, ordered real contacts and declared native body/contact/cache/history fields compared exactly against Normal W1. Actor shadow disabled. Dynamic complete-cell queue1353 explicitly enabled. Actual Tensor parallel reuse is required, never inferred from requests. Vehicle transients and external lifecycle closure beyond the listed fixture fields are NOT certified; full physics/Stadium/matrix equivalence and acceleration remain UNKNOWN. This test is not a performance measurement."));
 return !HasAnyErrors();
}
#endif
