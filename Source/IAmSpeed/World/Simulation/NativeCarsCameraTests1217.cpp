#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "IAmSpeed/World/Simulation/NativeExactInputFixture1304.h"
#include "IAmSpeed/Actors/SpeedCar.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/Camera/SpeedCarCameraPresentation.h"
#include "IAmSpeed/Input/Testing/WheeledTestProfile.h"
#include "IAmSpeed/World/Simulation/RealTimeSimulation.h"
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
class FNativeProfile1217 final:public Speed::Input::IInputProducer
{
public:
 explicit FNativeProfile1217(std::unique_ptr<Speed::Input::FTestInputProducer> P):Source(std::move(P)){}
 std::optional<Speed::Input::FInputFrame> Produce(Speed::Input::FFrameNumber N) override
 {++Calls;return Source->Produce(N);}
 uint32 Calls=0;
private:std::unique_ptr<Speed::Input::FTestInputProducer> Source;
};
struct FCapturedNativeCarState1425
{
 FBasePhysicsState Base;FWheeledPhysicsState Wheeled;SBaseGameState Game;FWheeledGameState WheeledGame;
 FMatrix InvI=FMatrix::Identity;
 TArray<int32> RecordedFrames;TArray<FBasePhysicsState> RecordedBase;TArray<FWheeledPhysicsState> RecordedWheeled;
 TArray<SKinematic> Bodies;
};
struct FNativeRow1217
{
 TArray<uint8> Snapshot,ObservedStates,Inputs,OrderedSolve,CameraPose,CameraCheckpoint;
 TArray<FCapturedNativeCarState1425> CapturedCars;
 ::FSimulationSnapshot CameraOracleSnapshot1434;
 Speed::FFrameFloatingPointContext CameraOracleNumeric1434;
 Speed::FFramePointerCanonicalSnapshot1456 DeferredContacts1456;
 TArray<uint64> CameraOraclePolls1434;
 uint64 BaseConsumed1430=0;int32 BaseUsefulNative1430=0;
 uint64 SnapshotTicks1450=0,CanonicalTicks1450=0,InputTicks1450=0,TypedStateTicks1450=0,PresentationTicks1450=0;
 uint64 Frame=0,Publication=0,PulseCycles1420=0,WitnessCycles1420=0;int32 CameraNative=0,PrepareNative=0,ShadowNative=0,MainContacts=0;
 bool bComplete=false;
};
void BaseState1217(TArray<uint8>& O,const FBasePhysicsState& X)
{
 using W=Speed::FEpisodeStateBytesForTesting;
 W::Add(O,X.Kinematic);W::Add(O,X.bIsFrozen);W::Add(O,X.nbFramesbeforeCanMove);W::Add(O,X.bStartCountdown);
}
void WheeledState1217(TArray<uint8>& O,const FWheeledPhysicsState& X)
{
 using W=Speed::FEpisodeStateBytesForTesting;
 W::Add(O,X.bStartCountdown);W::Add(O,X.nbFramesbeforeCanMove);W::Add(O,X.FramesSinceLastImpact);
 W::Add(O,X.AllowedSideVelocity);W::Add(O,X.AllowedAngularVelocity);W::Add(O,X.SuspensionLastDisplacement);
 W::Add(O,X.PhysicalThrottleInput);W::Add(O,X.PhysicalBrakeInput);W::Add(O,X.PhysicalSteerInput);W::Add(O,X.NbFramesSinceGroundContact);
}
void SerializeCapturedCar1425(TArray<uint8>& O,const FCapturedNativeCarState1425& X)
{
 using W=Speed::FEpisodeStateBytesForTesting;
BaseState1217(O,X.Base);WheeledState1217(O,X.Wheeled);
    W::Add(O,X.Game.NumFrame);W::Add(O,X.Game.TestVelocity);
    W::Add(O,X.Game.TestAngularVelocity);W::Add(O,X.Game.bHasTestVelocity);
    W::Add(O,X.WheeledGame.bGroundState);W::Add(O,X.WheeledGame.bIsUpsideDown);
    W::Add(O,X.WheeledGame.WheelsOmega);W::Add(O,X.WheeledGame.WheelsAngularPosition);
    W::Add(O,X.WheeledGame.NbWheelsOnGround);W::Add(O,X.WheeledGame.NbFrameSinceInAir);
    W::Add(O,X.InvI);W::Add(O,X.RecordedFrames.Num());
    for(int32 F:X.RecordedFrames)W::Add(O,F);
    W::Add(O,X.RecordedBase.Num());for(const auto& State:X.RecordedBase)BaseState1217(O,State);
    W::Add(O,X.RecordedWheeled.Num());for(const auto& State:X.RecordedWheeled)WheeledState1217(O,State);
    for(const auto& B:X.Bodies)W::Add(O,B);
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedNativeCarsCamera1217,
 "IAmSpeed.Simulation.FrameParallel.NativeCarsCanonicalCamera1217",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedNativeCarsCamera1217::RunTest(const FString&)
{
 using namespace Speed;using namespace Speed::Input;using namespace Speed::Input::Testing;
 using W=FEpisodeStateBytesForTesting;
 constexpr uint32 Frames=90;constexpr int32 CarCount=12;
 if(!TestTrue(TEXT("native fixture starts on GT"),IsInGameThread()))return false;
 auto* LaneVar=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.Parallel.Lanes"));
 auto* MinVar=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.Parallel.MinPrepareItems"));
 auto* WaveVar=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.Parallel.ActorKinematics1213"));
 auto* BaseConsumeVar1430=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.Parallel.ActorBaseConsume1430"));
 if(!TestNotNull(TEXT("exact base reuse opt-in"),BaseConsumeVar1430))return false;
 const int32 OldBaseConsume1430=BaseConsumeVar1430->GetInt();
 ON_SCOPE_EXIT{BaseConsumeVar1430->Set(OldBaseConsume1430,ECVF_SetByCode);};
 BaseConsumeVar1430->Set(1,ECVF_SetByCode);
 auto* Backend=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.StaticCollision.Backend"));
 if(!TestNotNull(TEXT("lane policy"),LaneVar)||!TestNotNull(TEXT("prepare grain"),MinVar)
  ||!TestNotNull(TEXT("native actor wave opt-in"),WaveVar)||!TestNotNull(TEXT("Analytic selector"),Backend))return false;
 const int32 OldLane=LaneVar->GetInt(),OldMin=MinVar->GetInt(),OldWave=WaveVar->GetInt(),OldBackend=Backend->GetInt();
 ON_SCOPE_EXIT{LaneVar->Set(OldLane,ECVF_SetByCode);MinVar->Set(OldMin,ECVF_SetByCode);WaveVar->Set(OldWave,ECVF_SetByCode);Backend->Set(OldBackend,ECVF_SetByCode);};
 if(!TestTrue(TEXT("native private workers available"),EnsurePrivateFrameWorkers(FFrameParallelPolicy::MaximumLanes-1)>0))return false;
 AddExpectedMessagePlain(TEXT("[WheelSuspensionBinding] subbodies="),ELogVerbosity::Warning,EAutomationExpectedMessageFlags::Contains,CarCount*5);
 TArray<FNativeRow1217> Oracle;
 for(int32 Lanes:{1,2,4,8,12})
 {
  LaneVar->Set(Lanes,ECVF_SetByCode);MinVar->Set(1,ECVF_SetByCode);WaveVar->Set(1,ECVF_SetByCode);Backend->Set(2,ECVF_SetByCode);
  const auto Options=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(false)
   .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false).SetTransactional(false);
  UWorld* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Options);
  if(!TestNotNull(TEXT("isolated native canonical multicars world"),World))return false;
  GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
  ON_SCOPE_EXIT{World->DestroyWorld(false);GEngine->DestroyWorldContext(World);};
  auto* Driver=World->SpawnActor<ARealTimeSimulation>();auto* Bridge=World->GetSubsystem<USpeedWorldSubsystem>();
  if(!TestNotNull(TEXT("canonical driver"),Driver)||!TestNotNull(TEXT("native registry"),Bridge))return false;
  Driver->SpeedWorldSubsystem=Bridge;
  Driver->ActiveExecutionModeValue.Store(static_cast<uint8>(ESimulationExecutionMode::IAmSpeedThread));
  auto Data=MakeShared<Analytic::FAnalyticWorldData>();
  if(!TestTrue(TEXT("explicit empty Analytic authority"),Data->FinalizeAndValidate()))return false;
  Bridge->AnalyticWorldData=Data;Bridge->SimulationWorld.SetStaticCollisionWorld(MakeUnique<FAnalyticStaticCollisionWorld>(*Data,Lanes,1,1));
  Chaos::FSimpleWheelConfig WheelConfig;Chaos::FSimpleSuspensionConfig SuspensionConfig;
  SuspensionConfig.SetSuspensionMaxRaise(10);SuspensionConfig.SetSuspensionMaxDrop(25);
  TArray<USpeedWheeledComponent*> Cars;
  TArray<std::shared_ptr<FInputStream>> Streams;TArray<std::shared_ptr<FNativeProfile1217>> Profiles;
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
   auto Vehicle=MakeUnique<Chaos::FSimpleWheeledVehicle>();
   for(int32 I=0;I<4;++I){Vehicle->Wheels.Emplace(&WheelConfig);Vehicle->Suspension.Emplace(&SuspensionConfig);C->Wheels.Add(NewObject<UChaosVehicleWheel>(C));}
   C->VehicleSimulationPT=MakeUnique<USpeedSimulation>();C->SkySimulation=static_cast<USpeedSimulation*>(C->VehicleSimulationPT.Get());C->SkySimulation->Init(Vehicle);
   C->BindWheelSimulationPointers();
   FString Reason;if(!TestTrue(TEXT("all production wheel storage admitted"),C->ValidateSimulationBindings(Reason)))return false;
   C->SetEngineFPS(300);C->SetImmediateMovementForTesting();C->EnableGenericCameraInput(true);
   C->SetHeldCameraYaw(.125f);C->SetHeldCameraPitch(-.0625f);
   SKinematic K;K.Location=FVector(2500*A,125*A,10000+100*A);K.Velocity=FVector(300+A*7,25-A,15);
   K.Rotation=FRotator(3*A,11*A,0).Quaternion();K.AngularVelocity=FVector(.125*A,-.25,.375);C->SetKinematicState(K);
   FWheeledTestProfile P;P.Identity={EProducerKind::Device,uint64(121700+A)};P.FrameCount=Frames;
   const float S=(A%2?-.5f:.5f);P.Keys={{0,.5f,0,S},{30,1.f,0,-S},{60,0,.25f,0}};
   auto Compiled=CompileWheeledTestProfile(P);
   if(!TestTrue(TEXT("distinct finite sealed profile compiles"),Compiled.Error==EWheeledProfileError::None&&bool(Compiled.Producer)))return false;
   auto Counted=std::make_shared<FNativeProfile1217>(std::move(Compiled.Producer));auto Stream=std::make_shared<FInputStream>(Counted);
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
  if(!TestTrue(TEXT("real driver camera binding includes presentation capture"),
   BoundCamera.IsValid()&&BoundCamera.Get()==&SerialCamera.Get()&&Driver->bPublishPresentation))return false;
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
  // Freeze this fixture-owned, read-only identity map before paced execution.
  // No actor/body/wheel is added, removed or rebound during these90 frames.
  // The per-frame witness still captures every actual ordered solve.
  FFramePointerRegistry Pointers;
  const auto Declare=[&](const auto* Pointer,uint32 Domain,uint64 OwnerId,uint64 Role,uint64 Element)
  {
   if(!Pointer)return true;
   FFramePointerKey Key;Key.Domain=Domain;Key.Owner=OwnerId;Key.Generation=1;Key.Role=Role;Key.Element=Element;
   return Pointers.Register(Pointer,Key);
  };
  bool Identities=Declare(Bridge->SimulationWorld.GetStaticCollisionWorld(),6,1,0,0);
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
  if(!TestTrue(TEXT("fixed fixture pointer identities seal before the paced owner starts"),Identities&&Pointers.Seal()))return false;
  TArray<FNativeRow1217> Seen;Seen.SetNum(Frames);
  // Reserve observer storage before the paced owner starts; all fields and full histories remain captured.
  for(auto& R:Seen)
  {
   R.Snapshot.Reserve(1024*1024);R.ObservedStates.Reserve(1024*1024);R.Inputs.Reserve(32768);R.CameraPose.Reserve(512);R.CameraCheckpoint.Reserve(4096);
   // Observed contact streams exceed 1.5 MiB. Prepare/touch growable fixture
   // storage on GT; never cap contacts or move their actual owner capture.
   R.OrderedSolve.SetNumZeroed(2*1024*1024);R.OrderedSolve.Reset();
   R.CapturedCars.SetNum(CarCount);R.CameraOraclePolls1434.SetNum(CarCount);
   if(!TestTrue(TEXT("empty contact snapshot storage prepared before owner"),
    R.DeferredContacts1456.ReserveAndPreTouch1456(2*1024*1024,2048)))return false;
   for(int32 A=0;A<CarCount;++A)
   {
    auto& Captured=R.CapturedCars[A];const auto* Car=Cars[A];
    // These are complete fixed-size ring histories (currently1024 slots),
    // not a growing array bounded by this fixture's90-frame run. Allocate
    // and populate real typed storage before starting the paced owner.
    // Every frame below still overwrites all slots with its actual values.
    Captured.RecordedFrames=Car->RecordedBaseFrames;
    Captured.RecordedBase=Car->RecordedBaseStates;
    Captured.RecordedWheeled=Car->RecordedWheeledStates;
    Captured.Bodies.Reserve(Car->GetSubBodies().Num());
   }
  }
  uint32 Steps=0,OwnerThread=0;int32 FailureLine=0,CameraStatus=-1,CameraReason=-1;bool Failed=false;
  FSimulationWorker Owner([&]
  {
   if(Steps==Frames)return ESimulationWorkerResult::Complete;
   OwnerThread=FPlatformTLS::GetCurrentThreadId();const uint64 N=Driver->CanonicalNumFrame;auto& Row=Seen[Steps];
   FrameParallel::ResetJoinedActorKinematicShadowTotals1213();
   ResetFrameParallelStageTotals(EFrameParallelStage::Prepare);
   ResetFrameParallelStageTotals(EFrameParallelStage::Camera);
   FFrameSolveWitnessScope Ordered(N,Row.OrderedSolve,Pointers);
   const uint64 PulseBegin1420=FPlatformTime::Cycles64();
   const auto Pulse=static_cast<ASpeedSimulation*>(Driver)->DriveOwnedWorkerPulse(1.f/300,float(N)/300);
   Row.PulseCycles1420=FPlatformTime::Cycles64()-PulseBegin1420;
   const uint64 WitnessBegin1420=FPlatformTime::Cycles64();
   if(Driver->CanonicalNumFrame==N&&Pulse==ESimulationWorkerResult::Idle)return ESimulationWorkerResult::Idle;
   if((Pulse!=ESimulationWorkerResult::Advanced&&Pulse!=ESimulationWorkerResult::Complete)||Driver->CanonicalNumFrame!=N+1)
   {Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
   FSimulationSnapshot Snapshot;
   if(!Driver->ReadLatestSimulationSnapshot(Snapshot)||Snapshot.NumFrame!=N||Snapshot.PresentationOutputs.Num()!=1)
   {Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
   Row.Frame=N;Row.Publication=Snapshot.PublicationSerial;Row.Snapshot=Snapshot.Payload;
   Row.SnapshotTicks1450=FPlatformTime::Cycles64()-WitnessBegin1420;
   const uint64 CanonicalBegin1450=FPlatformTime::Cycles64();
   Row.MainContacts=Ordered.NumMainRows();
   // DriveOwnedWorkerPulse returned after all physical children joined; the
   // remaining observer reads/copies only. No solve/contact writer follows.
   if(!Ordered.SealAndConsumeCanonicalSnapshot1456(Row.DeferredContacts1456)){Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
   Row.CanonicalTicks1450=FPlatformTime::Cycles64()-CanonicalBegin1450;
   for(int32 A=0;A<Cars.Num();++A)
   {
    const uint64 InputBegin1450=FPlatformTime::Cycles64();
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
    Row.CameraOraclePolls1434[A]=Driver->InputSessionRegistry->PollCount(ExpectedInput.Id);
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
    const uint64 StateBegin1450=FPlatformTime::Cycles64();
    Row.InputTicks1450+=StateBegin1450-InputBegin1450;
    // The paced owner copies exact values and complete histories only. Binary
    // projection runs on GT after natural drain while every actor is still alive.
    auto* C=Cars[A];auto& Saved=Row.CapturedCars[A];
    Saved.Base=C->BasePhysicsState;Saved.Wheeled=C->WheeledPhysicsState;
    Saved.Game=C->BaseGameState;Saved.WheeledGame=C->WheeledGameState;Saved.InvI=C->CarLocalInvI;
    Saved.RecordedFrames=C->RecordedBaseFrames;Saved.RecordedBase=C->RecordedBaseStates;
    Saved.RecordedWheeled=C->RecordedWheeledStates;Saved.Bodies.Reset();
    for(auto* B:C->GetSubBodies())Saved.Bodies.Add(B->GetKinematicState());
    Row.TypedStateTicks1450+=FPlatformTime::Cycles64()-StateBegin1450;
   }
   const uint64 PresentationBegin1450=FPlatformTime::Cycles64();
   // Save actual production output and participation before running the
   // independent comparison, which must not inflate production worker evidence.
   Row.CameraPose=Snapshot.PresentationOutputs[0].Payload;
   Row.CameraCheckpoint=Snapshot.PresentationOutputs[0].StatePayload;
   Row.CameraNative=GetFrameParallelStageTotals(EFrameParallelStage::Camera).MaximumNativeThreads;
   // The production camera has ALREADY run, joined and published in this
   // actual physical frame. Retain its values-only snapshot and exact numeric
   // entry for the separate comparison oracle; it has no live registry access.
   Snapshot.PresentationOutputs.Reset();
   Row.CameraOracleNumeric1434=CaptureFrameFloatingPointContext();
   Row.CameraOracleSnapshot1434=MoveTemp(Snapshot);

   Row.PrepareNative=GetFrameParallelStageTotals(EFrameParallelStage::Prepare).MaximumNativeThreads;
   Row.ShadowNative=FrameParallel::GetJoinedActorKinematicShadowTotals1213().MaximumShadowNativeParticipants;
   {const auto BaseStats1430=FrameParallel::GetJoinedActorKinematicShadowTotals1213();Row.BaseConsumed1430=BaseStats1430.ConsumedBaseItems1430;Row.BaseUsefulNative1430=BaseStats1430.MaximumUsefulBaseNativeParticipants1430;}
   Row.PresentationTicks1450=FPlatformTime::Cycles64()-PresentationBegin1450;
   Row.WitnessCycles1420=FPlatformTime::Cycles64()-WitnessBegin1420;
   Row.bComplete=true;++Steps;return ESimulationWorkerResult::Advanced;
  },[&](FSimulationWorkerWaitContext& Wait){static_cast<ASpeedSimulation*>(Driver)->WaitBetweenFrames(Wait);},
   [&](bool Paused){return Driver->ServiceInputSessionBoundary(Paused);},[&]{Driver->CloseInputSessionsOnWorker();});
  ON_SCOPE_EXIT{Owner.StopAndJoin();};
  const uint32 GT=FPlatformTLS::GetCurrentThreadId();if(!TestTrue(TEXT("actual native canonical owner starts"),Owner.Start()))return false;
  const double Deadline=FPlatformTime::Seconds()+60;
  while(Owner.IsRunning()&&FPlatformTime::Seconds()<Deadline)FPlatformProcess::SleepNoStats(.001f);
  const bool Finished=!Owner.IsRunning();Owner.StopAndJoin();
  if(Failed)AddInfo(FString::Printf(TEXT("[NativeCarsCameraFirstFailure1273] Lanes=%d Steps=%u Line=%d CameraStatus=%d CameraReason=%d"),Lanes,Steps,FailureLine,CameraStatus,CameraReason));
  if(!TestTrue(TEXT("finite native canonical run completes"),Finished&&!Failed&&Steps==Frames)
   ||!TestTrue(TEXT("physics owner is independent of GT"),OwnerThread!=0&&OwnerThread!=GT))return false;
  // No field, history element, comparison or cadence check is removed. Only
  // immutable captured values are read after the physical owner has joined.
  // Replay only the independent oracle over immutable completed-frame
  // snapshots. Real production camera/input validation stayed in every native
  // pulse before publication; every actual poll count was captured at N.
  const uint64 OracleBegin1434=FPlatformTime::Cycles64();
  for(uint32 N=0;N<Frames;++N)
  {
   auto& R=Seen[N];FScopedFrameFloatingPointContext OracleNumeric1434(R.CameraOracleNumeric1434);
   if(!OracleNumeric1434.IsApplied()){AddError(TEXT("Completed camera oracle numeric entry unavailable"));return false;}
   const auto ValidateCompleted1434=[&]()
   {
    if(!R.bComplete||R.Frame!=N||R.Publication!=N+1||R.CameraOraclePolls1434.Num()!=CarCount)return false;
    for(const uint64 Polls:R.CameraOraclePolls1434)if(Polls!=N+1)return false;
    return true;
   };
   const bool Produced1434=FrameParallel::ProduceFramePresentationWithOwnerValidation(
    R.CameraOracleSnapshot1434,WorkerList,Lanes,true,ValidateCompleted1434);
   const auto& Outputs1434=R.CameraOracleSnapshot1434.PresentationOutputs;
   FSpeedCarCameraPose Pose1434;
   const bool Equal1434=Produced1434&&Outputs1434.Num()==1
    &&R.CameraPose==Outputs1434[0].Payload&&R.CameraCheckpoint==Outputs1434[0].StatePayload
    &&FSpeedCarCameraPose::Read(Outputs1434[0].Payload,Pose1434)
    &&Pose1434.Frame==N&&Pose1434.Result.Status==ESpeedCarCameraStatus::Valid;
   const bool Restored1434=OracleNumeric1434.Restore();
   if(!TestTrue(FString::Printf(TEXT("completed-frame camera oracle matches actual native publication at N%u lanes%d"),N,Lanes),Equal1434)
    ||!TestTrue(TEXT("comparison oracle restores complete GT numeric context"),Restored1434))return false;
  }
  const uint64 OracleCycles1434=FPlatformTime::Cycles64()-OracleBegin1434;
  const uint64 ProjectionBegin1425=FPlatformTime::Cycles64();
  for(auto& R:Seen)for(const auto& C:R.CapturedCars)SerializeCapturedCar1425(R.ObservedStates,C);
  const uint64 ProjectionCycles1425=FPlatformTime::Cycles64()-ProjectionBegin1425;
  const double CounterHz1420=1.0/FPlatformTime::GetSecondsPerCycle64();
   // Every raw contact byte, pointer observation and semantic key was copied
   // at N. The same pure projector validates and formats those owned values
   // after natural drain; no physics contact/solve/input/barrier moves.
   const uint64 ContactProjectionBegin1456=FPlatformTime::Cycles64();
   for(uint32 N=0;N<Frames;++N)
   {
    if(!TestTrue(FString::Printf(TEXT("all deferred ordered contact tokens remain valid at frame %u"),N),Seen[N].DeferredContacts1456.MakeCanonical(Seen[N].OrderedSolve)))return false;
   }
   const uint64 ContactProjectionTicks1456=FPlatformTime::Cycles64()-ContactProjectionBegin1456;
   AddInfo(FString::Printf(TEXT("[NativeCarsDeferredContacts1456] Requested=%d Frames=%u CounterHz=%.0f ProjectionTicks=%llu Scope=PureOwnedContactProjectionAfterNaturalOwnerDrain"),Lanes,Frames,CounterHz1420,ContactProjectionTicks1456));
  AddInfo(FString::Printf(TEXT("[NativeCarsDeferredOracle1434] Requested=%d Frames=%u CounterHz=%.0f OracleTicks=%llu Scope=IndependentComparisonAfterNaturalOwnerDrain"),Lanes,Frames,CounterHz1420,OracleCycles1434));
  AddInfo(FString::Printf(TEXT("[NativeCarsDeferredProjection1425] Requested=%d Frames=%u CounterHz=%.0f ProjectionTicks=%llu Scope=GTAfterNaturalOwnerDrain"),Lanes,Frames,CounterHz1420,ProjectionCycles1425));
  for(uint32 N=0;N<Frames;++N)AddInfo(FString::Printf(TEXT("[NativeCarsFrameCost1420] Requested=%d N=%u CounterHz=%.0f PulseTicks=%llu ObserverTicks=%llu Scope=NonOverlappingPulseAndPostPulseWitness"),Lanes,N,CounterHz1420,Seen[N].PulseCycles1420,Seen[N].WitnessCycles1420));
  for(uint32 N=0;N<Frames;++N)
  {
   const auto& R1450=Seen[N];
   const uint64 Classified1450=R1450.SnapshotTicks1450+R1450.CanonicalTicks1450+R1450.InputTicks1450+R1450.TypedStateTicks1450+R1450.PresentationTicks1450;
   if(!TestTrue(TEXT("non-overlapping observer elapsed scopes fit the complete observer interval"),Classified1450<=R1450.WitnessCycles1420))return false;
   AddInfo(FString::Printf(TEXT("[NativeCarsPostCost1456] Requested=%d N=%u CounterHz=%.0f SnapshotTicks=%llu ContactCaptureTicks=%llu InputTicks=%llu TypedStateTicks=%llu PresentationTicks=%llu OtherTicks=%llu TotalTicks=%llu Scope=DisjointOwnerPostPulseWitness"),Lanes,N,CounterHz1420,R1450.SnapshotTicks1450,R1450.CanonicalTicks1450,R1450.InputTicks1450,R1450.TypedStateTicks1450,R1450.PresentationTicks1450,R1450.WitnessCycles1420-Classified1450,R1450.WitnessCycles1420));
  }
  for(uint32 N=0;N<Frames;++N)AddInfo(FString::Printf(TEXT("[NativeCarsUsefulBase1430] Requested=%d N=%u Consumed=%llu ConsumedNativeMax=%d"),Lanes,N,Seen[N].BaseConsumed1430,Seen[N].BaseUsefulNative1430));
  for(int32 A=0;A<CarCount;++A)TestEqual(TEXT("one immutable input authored per frame per car before start"),Profiles[A]->Calls,Frames);
  uint64 ConsumedBaseTotal1430=0;int32 UsefulBaseMax1430=0;
  for(uint32 N=0;N<Frames;++N)
  {
   const auto& Row=Seen[N];
   if(!TestTrue(TEXT("exact N and publication cover every300Hz frame"),Row.bComplete&&Row.Frame==N&&Row.Publication==N+1))return false;
   if(Lanes>1&&(!TestTrue(TEXT("shadow actor arithmetic used distinct private threads; never counted useful"),Row.ShadowNative>1)
    ||!TestTrue(TEXT("original native dispatch remains bounded; shadow is separate"),Row.PrepareNative>=1&&Row.PrepareNative<=Lanes)
    ||!TestTrue(TEXT("real camera calculation ran on worker plus owner validation"),Row.CameraNative>1)))return false;
   ConsumedBaseTotal1430+=Row.BaseConsumed1430;UsefulBaseMax1430=FMath::Max(UsefulBaseMax1430,Row.BaseUsefulNative1430);
   if(!TestEqual(TEXT("isolated separated cars have no selected contacts"),Row.MainContacts,0))return false;
   if(Lanes>1)
   {
    const auto& Ref=Oracle[N];
    if(!TestTrue(FString::Printf(TEXT("first divergence at N%u lanes%d: snapshot/input/state/ordered solve/camera"),N,Lanes),
     Row.Snapshot==Ref.Snapshot&&Row.Inputs==Ref.Inputs&&Row.ObservedStates==Ref.ObservedStates
     &&Row.OrderedSolve==Ref.OrderedSolve&&Row.CameraPose==Ref.CameraPose&&Row.CameraCheckpoint==Ref.CameraCheckpoint))return false;
   }
  }
  if(Lanes>1&&!TestTrue(TEXT("same observed native run consumed real multi-producer base arithmetic"),ConsumedBaseTotal1430>0&&UsefulBaseMax1430>1))return false;
  if(Lanes==1)Oracle=MoveTemp(Seen);
 }
 AddInfo(TEXT("Scope: 12 actual native cars;90 consecutive300Hz frames;sealed per-car producer commands and actual applied analog values;real canonical driver;ordered actual solve observer;real camera pose and full camera checkpoint equal at1/2/4/8/12 requested lanes. Camera uses at most2participants. Empty Analytic world deliberately isolates preparation/camera; zero main contacts is checked. Actor base arithmetic is reused only after exact input/FP validation at the native call site; original virtual/cache/wheel/callback paths remain on owner. Useful base producer counters are separate from shadow and requested lanes. This does not parallelize the complete actor preparation stage. The captured car-state subset is NOT all causally relevant transients: full physics/Stadium/derived Sky/contact equivalence remains UNKNOWN. No speedup or worker policy is qualified by this test."));
 return !HasAnyErrors();
}
#endif
