#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "NativeSelectedStateBytesForTesting.h"
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
#include "Misc/ScopeLock.h"
#include "NativeVehicleStateWitness1368.h"
#include "JoinedActorBaseWitness1534.h"
#include "NativeVehicleConfigWitness1534.h"
#include "FrameParallelExecutor.h"
namespace
{
// Explicit fixture-owned controller only. It seals the finite run and has no
// game inputs, gameplay, callbacks or effects. It also owns one real native box
// to exercise canonical CCD/solve independently of suspension support. Car physics is
// performed solely by the original native components and canonical world.
class FEightCarRunControl1534 final:public ISpeedComponent
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
class FEightCarPrepareDeltaWitness1534 final:public ISpeedPhysicsPhaseObserver
{
public:
 TArray<uint64> Frames;TArray<float> Deltas;
 void Observe(const ISpeedComponent& Component,ESpeedPhysicsObservationPhase Phase,float Delta)override
 {
  if(Phase!=ESpeedPhysicsObservationPhase::PreForce)return;
  const uint32 Local=Component.NumFrame();Frames.Add(Local?uint64(Local)-1:MAX_uint64);Deltas.Add(Delta);
 }
};
class FEightCarCVar1534
{
 IConsoleVariable* V=nullptr;int32 Old=0;
public:
 FEightCarCVar1534(const TCHAR* N,int32 Value):V(IConsoleManager::Get().FindConsoleVariable(N)),Old(V?V->GetInt():0){if(V)V->Set(Value,ECVF_SetByCode);}
 ~FEightCarCVar1534(){if(V)V->Set(Old,ECVF_SetByCode);}
 bool Is(int32 Value)const{return V&&V->GetInt()==Value;}
};
class FEightCarInputProfile1534 final:public Speed::Input::IInputProducer
{
public:
 explicit FEightCarInputProfile1534(std::unique_ptr<Speed::Input::FTestInputProducer> P):Source(std::move(P)){}
 std::optional<Speed::Input::FInputFrame> Produce(Speed::Input::FFrameNumber N) override
 {++Calls;return Source->Produce(N);}
 uint32 Calls=0;
private:std::unique_ptr<Speed::Input::FTestInputProducer> Source;
};
struct FEightCarField1534
{int32 Car=-1;const TCHAR* Name=TEXT("Unknown");int32 RawBegin=0,RawEnd=0,Begin=0,End=0;};
struct FEightCarCapturedRow1534
{
 TArray<uint8> Snapshot,ObservedStates,RawObservedStates,Inputs,OrderedSolve,RawOrderedSolve,CameraPose,CameraCheckpoint,PrepareDeltas,RawWheelManifold,WheelManifold;
 TArray<uint8> NumericContext1534;
 // Only populated on illegal post-seal writes; normal evidence pack bytes stay unchanged.
 TArray<uint8> RawContactAfterSealDebris1534,RawStateAfterSealDebris1534,RawWheelAfterSealDebris1368,CanonicalWheelAfterSealDebris1368,RawWheelDestinationDebris1368,CanonicalWheelDestinationDebris1368;
 Speed::FWheelWitnessSnapshot1368 DeferredWheel1368;
 Speed::FFramePointerCanonicalSnapshot1456 DeferredStates1534,DeferredContacts1534;TArray<int32> RawStateBounds1534,RawContactBounds1534;
 FSimulationSnapshot CameraOracleSnapshot1534;Speed::FFrameFloatingPointContext CameraOracleNumeric1534;TArray<uint64> CameraOraclePolls1534;
 TArray<FEightCarField1534> InputFields1534,StateFields1534;TArray<int32> ContactBounds1534;
 TArray<Speed::FrameParallel::FJoinedActorBaseEvent1534> BaseEvents1534;bool BaseWitnessComplete1534=false;uint64 BaseComputed1534=0;int32 BaseAdmittedLogical1552=0;
 float ActualDelta=0;
 uint64 Frame=0,Publication=0;int32 CameraNative=0,PrepareNative=0,ShadowNative=0,MainContacts=0;
 uint64 TensorComputed=0,TensorReused=0,TensorWasted=0;
 uint64 BaseConsumed1430=0;int32 BaseUsefulNative1430=0;
 Speed::Analytic::FTensorCellWaveStats1213 TensorDiagnostics;
 int32 TensorUsefulNative=0,TensorReusedNative=0,TensorReusedLanes=0;
 int32 GroundedWheels[8]={};bool ActiveSuspension[8]={};int32 WheelManifoldRows=0;
 bool bComplete=false;
};

// Post-drain diagnostic helpers only. These never run inside the physical frame.
int32 EightCarsFirstByte1534(const TArray<uint8>& Actual,const TArray<uint8>& Expected)
{
 const int32 Common=FMath::Min(Actual.Num(),Expected.Num());
 for(int32 I=0;I<Common;++I)if(Actual[I]!=Expected[I])return I;
 return Actual.Num()==Expected.Num()?int32(INDEX_NONE):Common;
}
FString EightCarsFieldAt1534(const TArray<FEightCarField1534>& Fields,int32 Offset)
{
 for(const auto& F:Fields)if(Offset>=F.Begin&&Offset<F.End)
  return FString::Printf(TEXT("Car=%d Field=%s Begin=%d End=%d RelativeByte=%d"),F.Car,F.Name,F.Begin,F.End,Offset-F.Begin);
 return FString::Printf(TEXT("Field=UNKNOWN_OR_EOF Byte=%d"),Offset);
}
FString EightCarsContactAt1534(const TArray<int32>& Bounds,const TArray<uint8>& Bytes,int32 Offset)
{
 for(int32 J=0;J+1<Bounds.Num();++J)if(Offset>=Bounds[J]&&Offset<Bounds[J+1])
 {
  uint32 Schema=0;if(Bounds[J]>=0&&Bounds[J]<=Bytes.Num()-int32(sizeof(Schema)))FMemory::Memcpy(&Schema,Bytes.GetData()+Bounds[J],sizeof(Schema));
  return FString::Printf(TEXT("ContactRow=%d Schema=%u RelativeByte=%d"),J,Schema,Offset-Bounds[J]);
 }
 return FString::Printf(TEXT("ContactRow=UNKNOWN_OR_EOF Byte=%d Bytes=%d"),Offset,Bytes.Num());
}
FString EightCarsWheelAt1534(const TArray<uint8>& Bytes,int32 Offset)
{
 // The separately frozen strict wheel reader resolves variable canonical rows
 // from both raw and canonical streams. Never guess a row from a byte offset.
 return FString::Printf(TEXT("WheelCanonicalByte=%d Bytes=%d WheelRow=UNKNOWN_PENDING_STRICT_RAW_CANONICAL_READER"),Offset,Bytes.Num());
}

void EightCarBaseState1534(TArray<uint8>& O,const FBasePhysicsState& X)
{
 Speed::SelectedStateBytesForTesting::AppendBaseState(O,X);
}
// This is an observer-only identity table, frozen before the simulation owner
// starts. StableId is the canonical registry ID already carried by the exact
// consumed input descriptor for this same car, never an allocation address.
struct FEightCarBumpVictimIdentity1564
{
 const USpeedWheeledComponent* Component=nullptr;
 uint64 StableId=0;
};
constexpr uint32 EightCarBumpOracleSchema1564=0x15640001;
bool EightCarBumpVictimIdentity1564(const FWheeledPhysicsState& X,
 const TArray<FEightCarBumpVictimIdentity1564>& Registry,uint8& Kind,uint64& StableId)
{
 Kind=0;StableId=0;
 if(X.LastCarBumpVictim.IsExplicitlyNull())return true;
 const auto* Victim=X.LastCarBumpVictim.Get();
 if(!Victim)return false; // stale/unobservable is UNKNOWN, never canonical null.
 int32 Matches=0;
 for(const auto& E:Registry)if(E.Component==Victim){++Matches;StableId=E.StableId;}
 if(Matches!=1||StableId==0)return false;
 // The frozen table is valid only while this same canonical registration lives.
 // Pointer/address reuse alone can never revive the previous registry identity.
 if(Victim->GetPublishedSimulationStableId()!=StableId)return false;
 for(const auto& E:Registry)if(E.StableId==StableId&&E.Component!=Victim)return false;
 Kind=1;return true;
}
bool EightCarWheeledState1534(TArray<uint8>& O,const FWheeledPhysicsState& X,
 const TArray<FEightCarBumpVictimIdentity1564>& BumpIdentities)
{
 uint8 BumpVictimKind=0;uint64 BumpVictimStableId=0;
 if(!EightCarBumpVictimIdentity1564(X,BumpIdentities,BumpVictimKind,BumpVictimStableId))return false;
 // Typed suffix: schema u32, explicit-null/stable kind u8, canonical ID u64,
 // actual cooldown u16. Do not normalize cooldown, including an inconsistent
 // null/nonzero pair: a state observer must retain the original causal values.
 Speed::SelectedStateBytesForTesting::AppendWheeledStateWithBump1564(O,X,EightCarBumpOracleSchema1564,BumpVictimKind,BumpVictimStableId);return true;
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedNativeAnalyticEightCars1534,
 "IAmSpeed.Simulation.FrameParallel.NativeAnalyticEightCars1552",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedNativeAnalyticEightCars1534::RunTest(const FString&)
{
 using namespace Speed;using namespace Speed::Input;using namespace Speed::Input::Testing;
 using W=FEpisodeStateBytesForTesting;
 constexpr uint32 Frames=90;constexpr int32 CarCount=8;
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
 const int32 AvailablePrivate1534=EnsurePrivateFrameWorkers(7);
 if(AvailablePrivate1534<7){AddError(FString::Printf(TEXT("[NativeEightCars1534TopologyUnavailable] RequiredTotal=8 AvailablePrivate=%d Status=UNKNOWN_TOPOLOGY_NOT_EXECUTION"),AvailablePrivate1534));return false;}
 AddExpectedMessagePlain(TEXT("[WheelSuspensionBinding] subbodies="),ELogVerbosity::Warning,EAutomationExpectedMessageFlags::Contains,CarCount*4);
 const uint32 OldFPS=ASpeedSimulation::EngineFPS;ASpeedSimulation::EngineFPS=300;
 ON_SCOPE_EXIT{ASpeedSimulation::EngineFPS=OldFPS;};
 FEightCarCVar1534 Canonical(TEXT("p.IAmSpeed.CanonicalFrame.Enabled"),1),Host(TEXT("p.IAmSpeed.Simulation.ExecutionMode"),2);
 FEightCarCVar1534 Tensor(TEXT("p.IAmSpeed.Parallel.TensorCellWave1213"),1),Observe(TEXT("p.IAmSpeed.Parallel.TensorCellObserve1213"),1);
 // Each scene gets its own Normal W1 oracle and complete evidence family.
 constexpr int32 IdleSpin1403=0,CellCount1403=16;
 {
 FEightCarCVar1534 IdlePolicy1403(TEXT("p.IAmSpeed.Parallel.PrivateIdleSpinMicros1403"),IdleSpin1403);
 if(!IdlePolicy1403.Is(IdleSpin1403))return false;
 // Admission uses the eligible subset, not the scene's total cell count.
 // The16-cell scene retains the existing actual Tensor reuse gates.
 const int32 Minimum1403=2;
 const int32 Grain1403=1;
 FEightCarCVar1534 Minimum(TEXT("p.IAmSpeed.Parallel.TensorCellMinItems1213"),Minimum1403),Grain(TEXT("p.IAmSpeed.Parallel.TensorCellMinItemsPerLane1213"),Grain1403),Chunk(TEXT("p.IAmSpeed.Parallel.TensorCellChunkItems1213"),1);
 FEightCarCVar1534 ProviderOff(TEXT("p.IAmSpeed.Parallel.PiecewiseProviderWave1075"),0);
 FEightCarCVar1534 Assignment(TEXT("p.IAmSpeed.Parallel.TensorCellDynamic1353"),1);
 if(!Canonical.Is(1)||!Host.Is(2)||!Tensor.Is(1)||!Observe.Is(1)||!Minimum.Is(Minimum1403)||!Grain.Is(Grain1403)||!Chunk.Is(1)||!ProviderOff.Is(0)||!Assignment.Is(1))return false;
 const FString EvidenceFamily1403=TEXT("IAmSpeed/QualificationEightCars1534");
 const FString EvidenceRoot=FPaths::Combine(FPaths::ProjectSavedDir(),EvidenceFamily1403,FGuid::NewGuid().ToString(EGuidFormats::Digits));
 TArray<FEightCarCapturedRow1534> Oracle;
 AddInfo(FString::Printf(TEXT("[NativeAnalyticEightCars1534Variant] Cells=%d Minimum=%d PerLane=%d ActualIdleSpinMicros=%d Frames=90 Hz=300 Cars=8 Scope=SourceFixtureNotPerformance"),CellCount1403,Minimum1403,Grain1403,IdleSpin1403));
 for(bool Fast:{false,true})for(int32 Lanes:{1,8})
 {
  FEightCarCVar1534 FastSelector(TEXT("p.IAmSpeed.FastSimulation.Enabled"),Fast?1:0);
  if(!FastSelector.Is(Fast?1:0))return false;
  LaneVar->Set(Lanes,ECVF_SetByCode);MinVar->Set(1,ECVF_SetByCode);WaveVar->Set(1,ECVF_SetByCode);Backend->Set(2,ECVF_SetByCode);
  const auto Options=UWorld::InitializationValues().AllowAudioPlayback(true).CreatePhysicsScene(false)
   .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false).SetTransactional(false);
  UWorld* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Options);
  if(!TestNotNull(TEXT("isolated native canonical multicars world"),World))return false;
  GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
  ON_SCOPE_EXIT{World->DestroyWorld(false);GEngine->DestroyWorldContext(World);};
  ASpeedSimulation* Driver=Fast?static_cast<ASpeedSimulation*>(World->SpawnActor<AFastSimulation>()):static_cast<ASpeedSimulation*>(World->SpawnActor<ARealTimeSimulation>());auto* Bridge=World->GetSubsystem<USpeedWorldSubsystem>();
  if(!TestNotNull(TEXT("canonical driver"),Driver)||!TestNotNull(TEXT("native registry"),Bridge))return false;
  Driver->SpeedWorldSubsystem=Bridge;
  Driver->ActiveExecutionModeValue.Store(static_cast<uint8>(ESimulationExecutionMode::IAmSpeedThread));
  FEightCarRunControl1534 Control;Control.Limit=Frames;AActor* BoxActor=World->SpawnActor<AActor>();Control.Initialize(*BoxActor);Bridge->RegisterSpeedComponent(&Control);
  ON_SCOPE_EXIT{Bridge->UnregisterSpeedComponent(&Control);Bridge->ApplyPendingOps();};
  auto Data=MakeShared<Analytic::FAnalyticWorldData>();
  Analytic::FPiecewiseTensorBezierPatch Patch;
  Patch.SourceId=1;Patch.SurfaceId=11;Patch.PrimitiveId=21;Patch.CanonicalGroupId=31;
  Patch.ObjectType=uint32(ECC_WorldStatic);Patch.BlockingChannels=MAX_uint64;Patch.bQueryCollisionEnabled=true;
  for(int32 Cell=0;Cell<CellCount1403;++Cell)
  {
   auto& C=Patch.Cells.AddDefaulted_GetRef();C.FeatureId=1000+Cell;C.PrimitiveId=2000+Cell;
   C.MinimumU=double(Cell)/CellCount1403;C.MaximumU=double(Cell+1)/CellCount1403;C.Surface.DegreeU=1;C.Surface.DegreeV=1;
   // Preserve the wide wheel-support footprint. The physical CCD box spans
   // [-45,45], so its central region must contain independent cells, rather
   // than two100cm cells which cannot exercise the requested Tensor wave.
   const double CentralHalf1403=80.0;
   const double CentralStep=CellCount1403==64?160.0/62.0:160.0/double(CellCount1403-2);
   const double X0=Cell==0?-1200.0:-CentralHalf1403+CentralStep*(Cell-1);
   const double X1=Cell==CellCount1403-1?5200.0:-CentralHalf1403+CentralStep*Cell;
   C.Surface.ControlPoints={FVector3d(X0,-1200,0),FVector3d(X0,1200,0),FVector3d(X1,-1200,0),FVector3d(X1,1200,0)};
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
  // Authored finite plugin fixture configuration, identical in every condition.
  // No production force law or default is changed.
  SuspensionConfig.SpringRate=5000;SuspensionConfig.RestingForce=250000;
  SuspensionConfig.CompressionDamping=1000;SuspensionConfig.ReboundDamping=1000;
  TArray<USpeedWheeledComponent*> Cars;TArray<TUniquePtr<FEightCarPrepareDeltaWitness1534>> PrepareWitnesses;
  TArray<std::shared_ptr<FInputStream>> Streams;TArray<std::shared_ptr<FEightCarInputProfile1534>> Profiles;
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
   // This fixture authors a finite frame-addressed camera queue independently
   // of its driving-only V2 contract. Select that retained adapter explicitly;
   // production stock cars keep their canonical V2 camera slots.
   C->CanonicalCameraYawAction=-1;C->CanonicalCameraPitchAction=-1;
   auto DeltaWitness=MakeUnique<FEightCarPrepareDeltaWitness1534>();DeltaWitness->Frames.Reserve(Frames);DeltaWitness->Deltas.Reserve(Frames);PrepareWitnesses.Add(MoveTemp(DeltaWitness));
   auto Vehicle=MakeUnique<Chaos::FSimpleWheeledVehicle>();
   for(int32 I=0;I<4;++I)
   {
    auto& Wheel=Vehicle->Wheels.Emplace_GetRef(&WheelConfig);
    // These public values are not initialized by this pinned Engine constructor.
    // Author them before any owner starts; never read an indeterminate value.
    Wheel.MaxOmega=10000;Wheel.AppliedLinearDriveForce=0;Wheel.AppliedLinearBrakeForce=0;
    Wheel.LongitudinalAdhesiveLimit=0;Wheel.LateralAdhesiveLimit=0;
    Wheel.SideSlipModifier=WheelConfig.SideSlipModifier;
    auto& AuthoredSuspension=Vehicle->Suspension.Emplace_GetRef(&SuspensionConfig);
    AuthoredSuspension.SetLocalRestingPosition(FVector(I<2?70:-70,I%2?45:-45,-15));
    auto* HostWheel=NewObject<UChaosVehicleWheel>(C);HostWheel->WheelRadius=30;
    HostWheel->SuspensionMaxRaise=10;HostWheel->SuspensionMaxDrop=25;C->Wheels.Add(HostWheel);
   }
   C->VehicleSimulationPT=MakeUnique<USpeedSimulation>();C->SkySimulation=static_cast<USpeedSimulation*>(C->VehicleSimulationPT.Get());C->SkySimulation->Init(Vehicle);
   C->BindWheelSimulationPointers();
   // The generic component deliberately supplies no preset geometry. This new
   // fixture authors four nonzero spheres; the historic airborne test is intact.
   if(!TestEqual(TEXT("four authored native wheel bodies"),C->GetWheelSubBodies().Num(),4))return false;
   for(int32 I=0;I<4;++I)
   {
    auto* Body=C->GetWheelSubBodies()[I].Get();
    const FVector Offset(I<2?70:-70,I%2?45:-45,-15);
    Body->SetLocalOffset(Offset);Body->SetRadiusForConfiguration(30);
   }
   FString Reason;if(!TestTrue(TEXT("all production wheel storage admitted"),C->ValidateSimulationBindings(Reason)))return false;
   C->SetEngineFPS(300);C->SetImmediateMovementForTesting();C->EnableGenericCameraInput(true);
   C->SetHeldCameraYaw(.125f);C->SetHeldCameraPitch(-.0625f);
   SKinematic K;K.Location=FVector(-500+3000*(A%2),-900+600*(A/2),45);K.Velocity=FVector(300+A*7,25-A,-40);
   K.Rotation=FQuat::Identity;K.AngularVelocity=FVector::ZeroVector;C->SetKinematicState(K);
   FWheeledTestProfile P;P.Identity={EProducerKind::Device,uint64(121700+A)};P.FrameCount=Frames;
   const float S=(A%2?-.25f:.25f)+float(A)/64.f;P.Keys={{0,.5f+float(A)/64.f,0,S},{30,.875f-float(A)/128.f,0,-S},{60,0,.25f+float(A)/128.f,0}};
   auto Compiled=CompileWheeledTestProfile(P);
   if(!TestTrue(TEXT("distinct finite sealed profile compiles"),Compiled.Error==EWheeledProfileError::None&&bool(Compiled.Producer)))return false;
   auto Counted=std::make_shared<FEightCarInputProfile1534>(std::move(Compiled.Producer));auto Stream=std::make_shared<FInputStream>(Counted);
   C->SetFrameInputStream(Stream);Profiles.Add(Counted);Streams.Add(Stream);
  }
  Bridge->ApplyPendingOps();
  if(!TestTrue(TEXT("driver admits immutable world and real cars"),Driver->EnsureSimulationWorldReady()))return false;
  Driver->InitializeCanonicalFrame(0);
  const TArray<FEightCarBumpVictimIdentity1564> BumpIdentities1564=[&]
  {
   TArray<FEightCarBumpVictimIdentity1564> R;
   for(auto* C:Cars)R.Add({C,Bridge->GetSimulationStableId(*C)});
   return R;
  }();
  // Observer contract probes use local state copies only. They neither invoke
  // bump mechanics nor change any live current/history state or authored input.
  FWheeledPhysicsState BumpProbe1564;uint8 BumpKind1564=255;uint64 BumpId1564=0;
  if(!TestTrue(TEXT("explicit null bump victim has a stable null identity"),
   EightCarBumpVictimIdentity1564(BumpProbe1564,BumpIdentities1564,BumpKind1564,BumpId1564)
   &&BumpKind1564==0&&BumpId1564==0))return false;
  for(int32 A=0;A<CarCount;++A)
  {
   if(!TestTrue(TEXT("UNKNOWN bump registry identity is nonzero and injective"),BumpIdentities1564[A].StableId!=0))return false;
   for(int32 B=0;B<A;++B)if(!TestTrue(TEXT("UNKNOWN bump registry identity is nonzero and injective"),
    BumpIdentities1564[A].Component!=BumpIdentities1564[B].Component
    &&BumpIdentities1564[A].StableId!=BumpIdentities1564[B].StableId))return false;
   BumpProbe1564.LastCarBumpVictim=Cars[A];BumpProbe1564.CarBumpCooldownFrames=uint16(75+A);
   auto Reordered=BumpIdentities1564;Swap(Reordered[0],Reordered[CarCount-1]);
   if(!TestTrue(TEXT("bump identity follows canonical actor ID despite observer table order"),
    EightCarBumpVictimIdentity1564(BumpProbe1564,Reordered,BumpKind1564,BumpId1564)
    &&BumpKind1564==1&&BumpId1564==BumpIdentities1564[A].StableId))return false;
  }
  BumpProbe1564.LastCarBumpVictim=Cars[0];
  auto Missing=BumpIdentities1564;Missing.RemoveAt(0);
  if(!TestFalse(TEXT("foreign/unmapped bump victim stays UNKNOWN"),
   EightCarBumpVictimIdentity1564(BumpProbe1564,Missing,BumpKind1564,BumpId1564)))return false;
  auto Ambiguous=BumpIdentities1564;Ambiguous.Add(BumpIdentities1564[0]);
  if(!TestFalse(TEXT("ambiguous bump victim stays UNKNOWN"),
   EightCarBumpVictimIdentity1564(BumpProbe1564,Ambiguous,BumpKind1564,BumpId1564)))return false;
  auto ChangedRegistration=BumpIdentities1564;ChangedRegistration[0].StableId+=uint64(MAX_uint32);
  if(!TestFalse(TEXT("changed canonical bump registration stays UNKNOWN"),
   EightCarBumpVictimIdentity1564(BumpProbe1564,ChangedRegistration,BumpKind1564,BumpId1564)))return false;
  auto Aliased=BumpIdentities1564;Aliased[1].StableId=Aliased[0].StableId;
  if(!TestFalse(TEXT("aliased canonical bump identity stays UNKNOWN"),
   EightCarBumpVictimIdentity1564(BumpProbe1564,Aliased,BumpKind1564,BumpId1564)))return false;
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
  TArray<FEightCarCapturedRow1534> Seen;Seen.SetNum(Frames);
  // Exact fixed-history contribution, including the typed bump suffix in
  // EVERY slot. Extra 64KiB/car is initial capacity for other captured fields,
  // not a limit or permission to drop data; all arrays remain growable.
  TArray<uint8> HistorySizeProbe1534;
  EightCarBaseState1534(HistorySizeProbe1534,FBasePhysicsState{});
  const int32 BaseHistoryBytes1534=HistorySizeProbe1534.Num();HistorySizeProbe1534.Reset();
  if(!EightCarWheeledState1534(HistorySizeProbe1534,FWheeledPhysicsState{},BumpIdentities1564))return false;
  const int32 WheeledHistoryBytes1534=HistorySizeProbe1534.Num();
  int32 HistoryBytes1534=0;
  for(const auto* C:Cars)HistoryBytes1534+=3*sizeof(int32)+C->RecordedBaseFrames.Num()*sizeof(int32)
   +C->RecordedBaseStates.Num()*BaseHistoryBytes1534+C->RecordedWheeledStates.Num()*WheeledHistoryBytes1534;
  const int32 StateCapacity1534=HistoryBytes1534+CarCount*65536;
  for(auto& R:Seen)
  {
   R.BaseEvents1534.Reserve(2048);R.StateFields1534.Reserve(128);R.InputFields1534.Reserve(64);
   R.CameraOraclePolls1534.SetNum(CarCount);R.RawStateBounds1534.Reserve(129);
   R.RawObservedStates.SetNumZeroed(StateCapacity1534);R.RawObservedStates.Reset();
   if(!TestTrue(TEXT("empty eight-car state snapshot prepared before owner"),
    R.DeferredStates1534.ReserveAndPreTouch1456(StateCapacity1534,4096)))return false;
   // Native06 raw contacts reached 769,453 bytes, beyond the old 64KiB hint.
   if(!TestTrue(TEXT("empty eight-car contact snapshot prepared before owner"),
    R.DeferredContacts1534.ReserveAndPreTouch1456(1024*1024,2048)))return false;
   if(!TestTrue(TEXT("empty eight-car wheel byte snapshot prepared before owner"),
    R.DeferredWheel1368.ReserveAndPreTouch1368()))return false;
   R.RawContactBounds1534.Reserve(2049);
   R.Snapshot.Reserve(65536);R.Inputs.Reserve(CarCount*4096);R.NumericContext1534.Reserve(256);
   R.PrepareDeltas.Reserve(2*sizeof(uint32)+(CarCount+1)*(2*sizeof(uint64)+sizeof(float)));
   R.RawOrderedSolve.SetNumZeroed(1024*1024);R.RawOrderedSolve.Reset();
   // Native06 raw wheel maximum was 405,756 bytes. No capture is truncated.
   R.RawWheelManifold.SetNumZeroed(512*1024);R.RawWheelManifold.Reset();
   if(!Fast){R.CameraPose.Reserve(512);R.CameraCheckpoint.Reserve(4096);}
  }
  // Only these fixture buffers are pre-touched. Local capture/projector and
  // wheel observer internals may still allocate; no allocation-free claim.
  uint32 Steps=0,OwnerThread=0;int32 FailureLine=0,CameraStatus=-1,CameraReason=-1;FString PointerFailure;bool Failed=false;
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
     Identities&=Declare(Wheel->GetSuspensionSim(),9,Id,0,uint64(WheelIndex));
     // The authored configs are shared by all cars/wheels in this one condition.
     Identities&=Declare(Wheel->GetWheelSim()->SetupPtr,10,1,0,0);
     Identities&=Declare(Wheel->GetSuspensionSim()->SetupPtr,11,1,0,0);++WheelIndex;
    }
   }
   if(!Identities||!Pointers.Seal()){Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
   FFrameSolveWitnessScope Ordered(N,Row.RawOrderedSolve,Pointers);
   if(!Ordered.PrepareFreshCaptureStorage1534(Row.DeferredContacts1534,Row.RawContactBounds1534)||FrameParallel::FJoinedActorBaseWitnessScope1534::IsActive()){Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
   FrameParallel::FJoinedActorBaseWitnessScope1534 BaseWitness1534(Row.BaseEvents1534);
   if(!Ordered.PrepareFreshWheelCaptureStorage1368(Row.DeferredWheel1368)){Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
   if(!Ordered.EnableWheelManifoldCapture1368(true)){Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
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
   // Same-process opaque CRT context only; never a portable wire format.
   W::Add(Row.NumericContext1534,uint32(sizeof(NumericAfterPhysics.Environment)));
   Row.NumericContext1534.Append(reinterpret_cast<const uint8*>(&NumericAfterPhysics.Environment),sizeof(NumericAfterPhysics.Environment));
   W::Add(Row.NumericContext1534,NumericAfterPhysics.VectorControl);W::Add(Row.NumericContext1534,NumericAfterPhysics.bValid);
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
   Row.MainContacts=Ordered.NumMainRows();Row.WheelManifoldRows=Ordered.NumWheelManifoldRows1368();
   if(!Ordered.SealAndConsumePreparedStorage1534(Row.DeferredContacts1534,Row.RawContactBounds1534))
   {
    Row.RawWheelManifold=Ordered.RawWheelManifold1368();
    Ordered.MakeWheelManifoldCanonical1368(Row.WheelManifold);
    PointerFailure=FString::Printf(TEXT("Solve N=%llu Overflow=%d Unknown=%d OpenPasses=%d\nOffset\tRaw\tKnown\tObjectId\tPair\tSolverPair\tProvider\tFeatureSalt\n"),N,Ordered.HasOverflow()?1:0,Ordered.HasUnknownPointer()?1:0,Ordered.OpenDynamicPassCount());
    for(const auto& E:Ordered.PointerObservations())PointerFailure+=FString::Printf(TEXT("%d\t%llu\t%d\t%d\t%d\t%d\t%llu\t%llu\n"),E.Offset,uint64(E.RawAddress),E.bKnown?1:0,E.bObjectId?1:0,E.bPairToken?1:0,E.bSolverPairToken?1:0,E.PairProviderId,E.PairFeatureSalt);
    Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;
   }
   if(!Ordered.ConsumeWheelWitness1368(Row.DeferredWheel1368))
   {
    // Failed capture must still retain original partial raw wheel evidence.
    Row.RawWheelManifold=Ordered.RawWheelManifold1368();
    Ordered.MakeWheelManifoldCanonical1368(Row.WheelManifold);
    Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;
   }
   TArray<uint8>& RawStates=Row.RawObservedStates;FFramePointerCaptureScope StatePointers(RawStates,Pointers);
   if(!StatePointers.PrepareFreshEvents1456(Row.DeferredStates1534))
   {Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
   const auto MarkState1534=[&](int32 Car,const TCHAR* Name)
   {if(!Row.StateFields1534.IsEmpty())Row.StateFields1534.Last().RawEnd=RawStates.Num();FEightCarField1534 F;F.Car=Car;F.Name=Name;F.RawBegin=RawStates.Num();Row.StateFields1534.Add(F);};
   const auto MarkInput1534=[&](int32 Car,const TCHAR* Name)
   {if(!Row.InputFields1534.IsEmpty())Row.InputFields1534.Last().End=Row.Inputs.Num();FEightCarField1534 F;F.Car=Car;F.Name=Name;F.Begin=Row.Inputs.Num();Row.InputFields1534.Add(F);};
   MarkState1534(-1,TEXT("FixtureController"));Control.AppendNativeWitness(RawStates);
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
    MarkInput1534(A,TEXT("SessionEpochFramePoll"));W::Add(Row.Inputs,ExpectedInput.Id);W::Add(Row.Inputs,ActualInput->Epoch);W::Add(Row.Inputs,N);
    W::Add(Row.Inputs,Driver->InputSessionRegistry->PollCount(ExpectedInput.Id));
    MarkInput1534(A,TEXT("ExactConsumedCommandAndAppliedActions"));Speed::Testing1304::AddExactInput(Row.Inputs,ActualInput->Snapshot->Input,ActualApplied);
    MarkInput1534(A,TEXT("CameraConsumptionCounters"));W::Add(Row.Inputs,Cars[A]->CameraInputApplyCount);W::Add(Row.Inputs,Cars[A]->CameraMailboxReadCount);
    const auto& CameraApplied1304=Cars[A]->WheeledPhysicalInput.Camera;const auto& CameraAuthored1304=AuthoredCameraInputs[N];
    MarkInput1534(A,TEXT("AppliedPhysicalCameraCommand"));W::Add(Row.Inputs,CameraApplied1304.Version);W::Add(Row.Inputs,CameraApplied1304.Flags);W::Add(Row.Inputs,CameraApplied1304.Yaw);W::Add(Row.Inputs,CameraApplied1304.Pitch);
    MarkInput1534(A,TEXT("AuthoredPhysicalCameraCommand"));W::Add(Row.Inputs,CameraAuthored1304.Version);W::Add(Row.Inputs,CameraAuthored1304.Flags);W::Add(Row.Inputs,CameraAuthored1304.Yaw);W::Add(Row.Inputs,CameraAuthored1304.Pitch);
    const auto& Applied=Cars[A]->WheeledPhysicalInput;
    MarkInput1534(A,TEXT("ActualAppliedWheeledInput"));W::Add(Row.Inputs,Applied.bCanMove);W::Add(Row.Inputs,Applied.Throttle);W::Add(Row.Inputs,Applied.Brake);W::Add(Row.Inputs,Applied.Steer);
    W::Add(Row.Inputs,Applied.Camera.Version);W::Add(Row.Inputs,Applied.Camera.Flags);W::Add(Row.Inputs,Applied.Camera.Yaw);W::Add(Row.Inputs,Applied.Camera.Pitch);
    auto* C=Cars[A];MarkState1534(A,TEXT("BasePhysicsState"));EightCarBaseState1534(RawStates,C->BasePhysicsState);
    MarkState1534(A,TEXT("WheeledPhysicsState"));
    if(!EightCarWheeledState1534(RawStates,C->WheeledPhysicsState,BumpIdentities1564))
    {
     PointerFailure=FString::Printf(TEXT("UNKNOWN_WHEELED_BUMP_IDENTITY1564\tN=%llu\tCar=%d\tField=WheeledPhysicsState\tHistoryIndex=-1\n"),N,A);
     Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;
    }
    MarkState1534(A,TEXT("GameAndWheelPresentation"));
    W::Add(RawStates,C->BaseGameState.NumFrame);W::Add(RawStates,C->BaseGameState.TestVelocity);
    W::Add(RawStates,C->BaseGameState.TestAngularVelocity);W::Add(RawStates,C->BaseGameState.bHasTestVelocity);
    W::Add(RawStates,C->WheeledGameState.bGroundState);W::Add(RawStates,C->WheeledGameState.bIsUpsideDown);
    W::Add(RawStates,C->WheeledGameState.WheelsOmega);W::Add(RawStates,C->WheeledGameState.WheelsAngularPosition);
    W::Add(RawStates,C->WheeledGameState.NbWheelsOnGround);W::Add(RawStates,C->WheeledGameState.NbFrameSinceInAir);
    MarkState1534(A,TEXT("InertiaAndCompleteRecordedHistory"));
    W::Add(RawStates,C->CarLocalInvI);W::Add(RawStates,C->RecordedBaseFrames.Num());
    if(!SelectedStateBytesForTesting::AppendFrameHistory1564(RawStates,C->RecordedBaseFrames))
    {Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
    W::Add(RawStates,C->RecordedBaseStates.Num());
    if(!SelectedStateBytesForTesting::AppendBaseHistory1564(RawStates,C->RecordedBaseStates))
    {Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
    W::Add(RawStates,C->RecordedWheeledStates.Num());
    int32 HistoryIndex1564=INDEX_NONE;
    if(!SelectedStateBytesForTesting::AppendWheelBumpHistory1564(RawStates,C->RecordedWheeledStates,EightCarBumpOracleSchema1564,
     [&](const FWheeledPhysicsState& X,uint8& Kind,uint64& StableId){return EightCarBumpVictimIdentity1564(X,BumpIdentities1564,Kind,StableId);},HistoryIndex1564))
    {
      PointerFailure=FString::Printf(TEXT("UNKNOWN_WHEELED_BUMP_IDENTITY1564\tN=%llu\tCar=%d\tField=InertiaAndCompleteRecordedHistory\tHistoryIndex=%d\n"),N,A,HistoryIndex1564);
      Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;
    }
    MarkState1534(A,TEXT("ConstraintsSleepQuantizationSupport"));
    W::Array(RawStates,C->PhysicalConstraints);
    C->SleepState.AppendEpisodeFieldsForTesting<W>(RawStates);C->KinematicQuantizationCache.AppendEpisodeFieldsForTesting<W>(RawStates);
    W::Add(RawStates,C->GetStaticCollisionWorldForFrame());W::Add(RawStates,C->GetStaticSupportFrameHorizon());
    MarkState1534(A,TEXT("NativeSubBodiesAndContactCaches"));
    for(auto* B:C->GetSubBodies())B->AppendEpisodeStateForTesting(RawStates);
    W::Add(RawStates,uint32(0x13680001));
    // Only read on the actual canonical owner, after all physical child tasks
    // have joined. This is an observer, not a restore or alternative physics path.
    MarkState1534(A,TEXT("CanonicalSupportAndHandbrakeTransients"));
    W::Add(RawStates,C->EngineFPS);W::Add(RawStates,C->MinNbFramesBeforeCanMove);
    W::Array(RawStates,C->SubBodies);W::Array(RawStates,C->ExtSubBodies);
    W::Add(RawStates,C->WheelFrameHandbrakeValue);W::Add(RawStates,C->WheelFrameHandbrakeActiveTime);
    W::Add(RawStates,C->LastWheelFrameHandbrakeUpdateFrame);W::Add(RawStates,C->TurnStartFrame);W::Add(RawStates,C->LastTurnSummaryFrame);
    W::Add(RawStates,C->SinceCanMoveFrame);W::Add(RawStates,C->bCanonicalSupportStartupInitializationRequested);
    W::Add(RawStates,C->bCanonicalSupportStartupPoseConfirmed);W::Add(RawStates,C->bSimTimelineWasCanMove);
    W::Add(RawStates,C->bSimTimelineWasGrounded);W::Add(RawStates,C->bSimTimelineHasGroundState);
    W::Add(RawStates,C->bDeferWheelGroundStateUpdate);W::Add(RawStates,C->LastCertifiedCoupledPoseCOM);
    W::Add(RawStates,C->LastCertifiedCoupledPoseRotation);W::Add(RawStates,C->LastCertifiedCoupledPoseFrame);
    MarkState1534(A,TEXT("PendingWheelContacts"));
    W::Add(RawStates,C->PendingWheelContacts.Num());
    for(const auto& Contact:C->PendingWheelContacts)Testing1368::AppendWheelContact(RawStates,Contact);
    MarkState1534(A,TEXT("InputAuthorityReservations"));
    W::Add(RawStates,C->bInputFrameRejectedV2);W::Add(RawStates,C->bResetProducedWheeledInputs);
    W::Add(RawStates,C->bProducedInputFrameOwned);W::Add(RawStates,C->bProducedInputAuthority.load(std::memory_order_acquire));
    W::Add(RawStates,C->bLegacyRemoteInputAuthority.load(std::memory_order_acquire));
    W::Add(RawStates,C->bCanonicalInputPending);W::Add(RawStates,bool(C->CanonicalInputSnapshot));
    W::Add(RawStates,C->PreparedCanonicalInputFrame.IsSet());
    if(C->PreparedCanonicalInputFrame.IsSet())W::Add(RawStates,C->PreparedCanonicalInputFrame.GetValue());
    W::Add(RawStates,C->InputReservationV2.has_value());W::Add(RawStates,C->ReservedInputFrameV2.has_value());
    if(C->ReservedInputFrameV2)W::Add(RawStates,*C->ReservedInputFrameV2);
    // A committed fixture may not hide an opaque reservation or legacy producer.
    if(C->InputReservationV2||C->ReservedInputFrameV2||C->LegacyWheeledIngress||C->LegacyWheeledOwner
     ||C->bNetCorrHasTarget||C->NetCorrAccum.bActive)
    {Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
    const auto AppendNetworkInput=[&](const auto& Value)
    {
     W::Add(RawStates,bool(Value));if(!Value)return;
     Testing1368::AppendInput(RawStates,Value->Requested);W::Add(RawStates,Value->LocalFrame);W::Add(RawStates,Value->SinceCanMove);
    };
    MarkState1534(A,TEXT("PublishedPreparedNetworkAndCameraQueue"));
    AppendNetworkInput(std::atomic_load_explicit(&C->PreparedWheeledNetworkInput,std::memory_order_acquire));
    AppendNetworkInput(C->PublishedWheeledNetworkInput.load(std::memory_order_acquire));
    W::Add(RawStates,C->bGenericCameraInputEnabled.load(std::memory_order_acquire));
    W::Add(RawStates,C->PendingCameraInput.load(std::memory_order_acquire));
    {
     FScopeLock CameraQueueLock(&C->PendingCameraInputMutex);
     W::Add(RawStates,C->PendingCameraInputCommands.Num());
     for(const auto& Pending:C->PendingCameraInputCommands)
     {W::Add(RawStates,Pending.ActivationFrame);Testing1368::AppendCameraInput(RawStates,Pending.Input);}
     W::Add(RawStates,C->LastCameraCaptureHistoryFrame);W::Add(RawStates,C->LastCameraCaptureActivationFrame);
     Testing1368::AppendCameraInput(RawStates,C->LastCapturedCameraInput);
    }
    MarkState1534(A,TEXT("NetworkCorrection"));
    W::Add(RawStates,C->NetCorr_LastServerFrame);W::Add(RawStates,C->NetCorr_LastLocalFrame);
    W::Add(RawStates,C->NetCorr_BaseNumPredictedFrames);W::Add(RawStates,C->bNetCorrHasTarget);W::Add(RawStates,C->NetCorrTickCount);
    W::Add(RawStates,C->NetCorrAccum.RemPos);W::Add(RawStates,C->NetCorrAccum.RemRot);W::Add(RawStates,C->NetCorrAccum.RemVel);
    W::Add(RawStates,C->NetCorrAccum.RemAngVel);W::Add(RawStates,C->NetCorrAccum.bActive);
    W::Add(RawStates,C->NetCorrAccum.SourceServerFrame);W::Add(RawStates,C->NetCorrAccum.SourceLocalFrame);
    W::Add(RawStates,C->NetCorr_LastContactN);W::Add(RawStates,C->NetCorr_StableNFrames);
    MarkState1534(A,TEXT("NativeVehicleWheelsAndCompleteSuspensionValues"));
    if(!C->SkySimulation||!C->SkySimulation->PVehicle
     ||!Testing1368::AppendVehicle(RawStates,*C->SkySimulation->PVehicle))
    {Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
    MarkState1534(A,TEXT("ActualWheelAndSuspensionSetupValues"));
    W::Add(RawStates,C->GetWheelSubBodies().Num());
    for(const auto& Wheel:C->GetWheelSubBodies())
    {
     const auto* NativeWheel=Wheel->GetWheelSim();const auto* NativeSuspension=Wheel->GetSuspensionSim();const auto* HostWheel=Wheel->GetChaosWheelConfiguration();
     if(!NativeWheel||!NativeWheel->SetupPtr||!NativeSuspension||!NativeSuspension->SetupPtr||!HostWheel
      ||!Testing1534::AppendWheelConfig(RawStates,NativeWheel->Setup()))
     {Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
     Testing1534::AppendSuspensionConfig(RawStates,NativeSuspension->Setup());
     W::Add(RawStates,HostWheel->WheelRadius);W::Add(RawStates,HostWheel->SuspensionMaxRaise);W::Add(RawStates,HostWheel->SuspensionMaxDrop);
    }
    MarkState1534(A,TEXT("GroundAndActiveSuspensionCoverage"));
    Row.GroundedWheels[A]=C->NumWheelsOnGround();
    for(const auto& Wheel:C->GetWheelSubBodies())
     Row.ActiveSuspension[A]|=Wheel->GetSuspensionSim()->GetLastDisplacement()!=0.f;
    W::Add(RawStates,Row.GroundedWheels[A]);W::Add(RawStates,Row.ActiveSuspension[A]);
   }
   Row.StateFields1534.Last().RawEnd=RawStates.Num();Row.InputFields1534.Last().End=Row.Inputs.Num();
   for(const auto& F:Row.StateFields1534)Row.RawStateBounds1534.Add(F.RawBegin);Row.RawStateBounds1534.Add(RawStates.Num());
   if(!StatePointers.ConsumeCanonicalSnapshot1456(Row.DeferredStates1534))
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
   }
   Row.CameraNative=GetFrameParallelStageTotals(EFrameParallelStage::Camera).MaximumNativeThreads;
   // Freeze actual completed-N values before leaving the canonical owner.
   // Only the independent comparison is replayed after natural drain, as in1217.
   Snapshot.PresentationOutputs.Reset();Row.CameraOracleSnapshot1534=MoveTemp(Snapshot);
   Row.CameraOracleNumeric1534=CaptureFrameFloatingPointContext();
   for(int32 A=0;A<Cars.Num();++A)Row.CameraOraclePolls1534[A]=Driver->InputSessionRegistry?Driver->InputSessionRegistry->PollCount(ExactDescriptors[A].Id):0;

   Row.PrepareNative=GetFrameParallelStageTotals(EFrameParallelStage::Prepare).MaximumNativeThreads;
   Row.ShadowNative=FrameParallel::GetJoinedActorKinematicShadowTotals1213().MaximumShadowNativeParticipants;
   {const auto BaseStats1430=FrameParallel::GetJoinedActorKinematicShadowTotals1213();Row.BaseConsumed1430=BaseStats1430.ConsumedBaseItems1430;Row.BaseComputed1534=BaseStats1430.ComputedBaseItems1430;Row.BaseAdmittedLogical1552=BaseStats1430.MaximumShadowLogicalLanes;Row.BaseWitnessComplete1534=BaseWitness1534.IsComplete();Row.BaseUsefulNative1430=BaseStats1430.MaximumUsefulBaseNativeParticipants1430;}
   const auto TensorStats=Analytic::SnapshotTensorCellWaveStats1213();
   Row.TensorDiagnostics=TensorStats;
   Row.TensorComputed=TensorStats.Computed;Row.TensorReused=TensorStats.Reused;Row.TensorWasted=TensorStats.Wasted;
   Row.TensorUsefulNative=TensorStats.UsefulNativeMax;Row.TensorReusedNative=TensorStats.ReusedNativeMax1213;Row.TensorReusedLanes=TensorStats.ReusedLaneMax1213;
   if(!ObservationGuard.Restore()||!FrameParallel::SameJoinedActorNumericContext1213(NumericAfterPhysics,CaptureFrameFloatingPointContext())){Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
   Row.bComplete=true;++Steps;return Steps==Frames?ESimulationWorkerResult::Complete:ESimulationWorkerResult::Advanced;
  },[&](FSimulationWorkerWaitContext& Wait){Driver->WaitBetweenFrames(Wait);},
   [&](bool Paused){return Driver->ServiceInputSessionBoundary(Paused);},[&]{Driver->CloseInputSessionsOnWorker();});
  ON_SCOPE_EXIT{Owner.StopAndJoin();};
  const uint32 GT=FPlatformTLS::GetCurrentThreadId();if(!TestTrue(TEXT("actual native canonical owner starts"),Owner.Start()))return false;
  const double Deadline=FPlatformTime::Seconds()+60;
  while(Owner.IsRunning()&&FPlatformTime::Seconds()<Deadline)FPlatformProcess::SleepNoStats(.001f);
  const bool Finished=!Owner.IsRunning();Owner.StopAndJoin();
  // Recover every captured raw stream after join, including successful
  // earlier frames of a failed/incomplete run. Canonical validity is separate:
  // UNKNOWN snapshots retain evidence, and non-consumed sources stay intact.
  for(uint32 N=0;N<Frames;++N)
  {
   auto& R=Seen[N];
   const auto RestoreRaw1534=[&](const FFramePointerCanonicalSnapshot1456& Captured,TArray<uint8>& Raw,TArray<uint8>& Debris)
   {
    auto Status=Captured.CopyRawForEvidenceAfterClosure1456(Raw);
    if(Status==EFramePointerRawEvidenceCopy1456::DestinationNotEmpty)
    {
     // Keep both the exact captured bytes and illegal source writes. Never
     // erase the latter or turn this misuse into a successful certificate.
     Swap(Raw,Debris);Failed=true;R.bComplete=false;
     PointerFailure+=FString::Printf(TEXT("Post-seal raw writes retained separately at N=%u\n"),N);
     Status=Captured.CopyRawForEvidenceAfterClosure1456(Raw);
    }
    if(Status!=EFramePointerRawEvidenceCopy1456::Copied&&Status!=EFramePointerRawEvidenceCopy1456::NotCaptured)
    {Failed=true;R.bComplete=false;PointerFailure+=FString::Printf(TEXT("Raw recovery unavailable after join at N=%u Status=%d\n"),N,int32(Status));}
   };
   RestoreRaw1534(R.DeferredContacts1534,R.RawOrderedSolve,R.RawContactAfterSealDebris1534);
   RestoreRaw1534(R.DeferredStates1534,R.RawObservedStates,R.RawStateAfterSealDebris1534);
   auto WheelStatus1368=R.DeferredWheel1368.ReleaseAfterClosure(R.RawWheelManifold,R.WheelManifold,
    R.RawWheelAfterSealDebris1368,R.CanonicalWheelAfterSealDebris1368);
   if(WheelStatus1368==EWheelWitnessRelease1368::DestinationNotEmpty)
   {
    Swap(R.RawWheelManifold,R.RawWheelDestinationDebris1368);Swap(R.WheelManifold,R.CanonicalWheelDestinationDebris1368);
    Failed=true;R.bComplete=false;PointerFailure+=FString::Printf(TEXT("Wheel destination writes retained separately at N=%u\n"),N);
    WheelStatus1368=R.DeferredWheel1368.ReleaseAfterClosure(R.RawWheelManifold,R.WheelManifold,R.RawWheelAfterSealDebris1368,R.CanonicalWheelAfterSealDebris1368);
   }
   if(WheelStatus1368!=EWheelWitnessRelease1368::NotCaptured&&WheelStatus1368!=EWheelWitnessRelease1368::TransferredValid)
   {Failed=true;R.bComplete=false;PointerFailure=FString::Printf(TEXT("Wheel evidence unavailable or invalid after join at N=%u Status=%d\n"),N,int32(WheelStatus1368));}
  }
  // All raw bytes, semantic keys, field boundaries, camera inputs and poll
  // counts were owned captures at N. No live physics getter runs in projection.
  if(Finished&&!Failed&&Steps==Frames)for(uint32 N=0;N<Frames;++N)
  {
   auto& R=Seen[N];TArray<int32> CanonicalBounds1534;
   if(!R.DeferredContacts1534.MakeCanonicalWithBoundaries1534(R.OrderedSolve,R.RawContactBounds1534,R.ContactBounds1534)
    ||!R.DeferredStates1534.MakeCanonicalWithBoundaries1534(R.ObservedStates,R.RawStateBounds1534,CanonicalBounds1534)
    ||CanonicalBounds1534.Num()!=R.StateFields1534.Num()+1)
   {Failed=true;FailureLine=__LINE__;R.bComplete=false;PointerFailure=FString::Printf(TEXT("Deferred exact projection refused at N=%u; raw bytes and boundaries retained\n"),N);break;}
   for(int32 I=0;I<R.StateFields1534.Num();++I){R.StateFields1534[I].Begin=CanonicalBounds1534[I];R.StateFields1534[I].End=CanonicalBounds1534[I+1];}
   if(!Fast)
   {
   FScopedFrameFloatingPointContext OracleNumeric1534(R.CameraOracleNumeric1534);
   if(!OracleNumeric1534.IsApplied()){Failed=true;FailureLine=__LINE__;R.bComplete=false;break;}
   const auto ValidateCompleted1534=[&]()
   {
    if(!R.bComplete||R.Frame!=N||R.Publication!=N+1||R.CameraOraclePolls1534.Num()!=CarCount)return false;
    for(const uint64 Polls:R.CameraOraclePolls1534)if(Polls!=N+1)return false;return true;
   };
   const bool Produced1534=FrameParallel::ProduceFramePresentationWithOwnerValidation(R.CameraOracleSnapshot1534,WorkerList,Lanes,true,ValidateCompleted1534);
   const auto& Outputs1534=R.CameraOracleSnapshot1534.PresentationOutputs;FSpeedCarCameraPose Pose1534;
   const bool PoseRead1534=Produced1534&&Outputs1534.Num()==1&&FSpeedCarCameraPose::Read(Outputs1534[0].Payload,Pose1534);
   if(PoseRead1534){CameraStatus=int32(Pose1534.Result.Status);CameraReason=int32(Pose1534.Result.Reason);}
   const bool Equal1534=PoseRead1534&&Pose1534.Frame==N&&Pose1534.Result.Status==ESpeedCarCameraStatus::Valid&&R.CameraPose==Outputs1534[0].Payload&&R.CameraCheckpoint==Outputs1534[0].StatePayload;
   if(!OracleNumeric1534.Restore()||!Equal1534){Failed=true;FailureLine=__LINE__;R.bComplete=false;break;}
   }
   else if(R.CameraOracleSnapshot1534.PresentationOutputs.Num()!=0||!R.CameraPose.IsEmpty()||!R.CameraCheckpoint.IsEmpty()||R.CameraNative!=0)
   {Failed=true;FailureLine=__LINE__;R.bComplete=false;break;}
  }
  if(Failed)AddInfo(FString::Printf(TEXT("[NativeAnalyticContactsFirstFailure1297] Fast=%d Lanes=%d Steps=%u Line=%d CameraStatus=%d CameraReason=%d"),Fast?1:0,Lanes,Steps,FailureLine,CameraStatus,CameraReason));
  // Save ALL raw/canonical observations only after joining, so file I/O is
  // excluded from physical frames and an UNKNOWN keeps its original bytes.
  const FString CaseDir=FPaths::Combine(EvidenceRoot,FString::Printf(TEXT("Fast%d-W%d"),Fast?1:0,Lanes));
  if(IFileManager::Get().DirectoryExists(*CaseDir))
  {AddError(FString::Printf(TEXT("[NativeAnalyticContacts1368ExportRefused] Existing directory %s; no journal overwritten"),*CaseDir));return false;}
  bool EvidenceSaved=IFileManager::Get().MakeDirectory(*CaseDir,true);
  const auto SavePack=[&](const TCHAR* Name,auto Member)
  {
   TArray<uint8> Packed;W::Add(Packed,uint32(0x31434149));W::Add(Packed,uint32(Frames));
   for(uint32 N=0;N<Frames;++N){const auto& Bytes=Seen[N].*Member;W::Add(Packed,uint64(N));W::Add(Packed,uint32(Bytes.Num()));Packed.Append(Bytes);}
   const FString Path=FPaths::Combine(CaseDir,Name);
   if(IFileManager::Get().FileExists(*Path)){AddError(FString::Printf(TEXT("[NativeAnalyticContacts1368ExportRefused] Existing file %s"),*Path));return false;}
   return FFileHelper::SaveArrayToFile(Packed,*Path,&IFileManager::Get(),FILEWRITE_NoReplaceExisting);
  };
  if(!PointerFailure.IsEmpty())EvidenceSaved&=FFileHelper::SaveStringToFile(PointerFailure,*FPaths::Combine(CaseDir,TEXT("pointers.first-failure.tsv")),FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM,&IFileManager::Get(),FILEWRITE_NoReplaceExisting);
  EvidenceSaved&=SavePack(TEXT("contacts.raw.pack"),&FEightCarCapturedRow1534::RawOrderedSolve);
  // Extra forensic streams exist only on detected misuse; normal pack set,
  // framing, row count and payload bytes are unchanged.
  bool WheelRawDestinationDebris1368=false,WheelCanonicalDestinationDebris1368=false;
  for(const auto& R:Seen){WheelRawDestinationDebris1368|=!R.RawWheelDestinationDebris1368.IsEmpty();WheelCanonicalDestinationDebris1368|=!R.CanonicalWheelDestinationDebris1368.IsEmpty();}
  if(WheelRawDestinationDebris1368)EvidenceSaved&=SavePack(TEXT("wheel-manifold.destination-debris.raw.pack"),&FEightCarCapturedRow1534::RawWheelDestinationDebris1368);
  if(WheelCanonicalDestinationDebris1368)EvidenceSaved&=SavePack(TEXT("wheel-manifold.destination-debris.canonical.pack"),&FEightCarCapturedRow1534::CanonicalWheelDestinationDebris1368);
  bool WheelRawDebris1368=false,WheelCanonicalDebris1368=false;
  for(const auto& R:Seen){WheelRawDebris1368|=!R.RawWheelAfterSealDebris1368.IsEmpty();WheelCanonicalDebris1368|=!R.CanonicalWheelAfterSealDebris1368.IsEmpty();}
  if(WheelRawDebris1368)EvidenceSaved&=SavePack(TEXT("wheel-manifold.after-seal-debris.raw.pack"),&FEightCarCapturedRow1534::RawWheelAfterSealDebris1368);
  if(WheelCanonicalDebris1368)EvidenceSaved&=SavePack(TEXT("wheel-manifold.after-seal-debris.canonical.pack"),&FEightCarCapturedRow1534::CanonicalWheelAfterSealDebris1368);
  bool ContactDebris1534=false,StateDebris1534=false;
  for(const auto& R:Seen){ContactDebris1534|=!R.RawContactAfterSealDebris1534.IsEmpty();StateDebris1534|=!R.RawStateAfterSealDebris1534.IsEmpty();}
  if(ContactDebris1534)EvidenceSaved&=SavePack(TEXT("contacts.after-seal-debris.raw.pack"),&FEightCarCapturedRow1534::RawContactAfterSealDebris1534);
  if(StateDebris1534)EvidenceSaved&=SavePack(TEXT("states.after-seal-debris.raw.pack"),&FEightCarCapturedRow1534::RawStateAfterSealDebris1534);
  EvidenceSaved&=SavePack(TEXT("wheel-manifold.raw.pack"),&FEightCarCapturedRow1534::RawWheelManifold);
  EvidenceSaved&=SavePack(TEXT("wheel-manifold.canonical.pack"),&FEightCarCapturedRow1534::WheelManifold);
  EvidenceSaved&=SavePack(TEXT("contacts.canonical.pack"),&FEightCarCapturedRow1534::OrderedSolve);
  EvidenceSaved&=SavePack(TEXT("states.raw.pack"),&FEightCarCapturedRow1534::RawObservedStates);
  EvidenceSaved&=SavePack(TEXT("states.canonical.pack"),&FEightCarCapturedRow1534::ObservedStates);
  EvidenceSaved&=SavePack(TEXT("commands.consumed.pack"),&FEightCarCapturedRow1534::Inputs);
  EvidenceSaved&=SavePack(TEXT("snapshots.pack"),&FEightCarCapturedRow1534::Snapshot);
  EvidenceSaved&=SavePack(TEXT("camera.pose.pack"),&FEightCarCapturedRow1534::CameraPose);
  EvidenceSaved&=SavePack(TEXT("camera.checkpoint.pack"),&FEightCarCapturedRow1534::CameraCheckpoint);
  EvidenceSaved&=SavePack(TEXT("prepare.deltas.pack"),&FEightCarCapturedRow1534::PrepareDeltas);
  EvidenceSaved&=SavePack(TEXT("numeric.context1534.pack"),&FEightCarCapturedRow1534::NumericContext1534);
  // Preserve the ACTUAL complete argument histories even if a dt/count gate
  // rejected a frame before its canonical per-row witness could be emitted.
  TArray<uint8> PrepareHistories;W::Add(PrepareHistories,uint32(1));W::Add(PrepareHistories,uint32(CarCount+1));
  const auto AppendHistory=[&](uint64 Id,const TArray<uint64>& Ns,const TArray<float>& Dts)
  {W::Add(PrepareHistories,Id);W::Array(PrepareHistories,Ns);W::Array(PrepareHistories,Dts);};
  AppendHistory(Bridge->GetSimulationStableId(Control),Control.PreparedFrames,Control.PreparedDeltas);
  for(int32 A=0;A<Cars.Num();++A)AppendHistory(Bridge->GetSimulationStableId(*Cars[A]),PrepareWitnesses[A]->Frames,PrepareWitnesses[A]->Deltas);
  const FString HistoryPath=FPaths::Combine(CaseDir,TEXT("prepare.histories.raw"));
  if(IFileManager::Get().FileExists(*HistoryPath)){AddError(FString::Printf(TEXT("[NativeAnalyticContacts1368ExportRefused] Existing file %s"),*HistoryPath));EvidenceSaved=false;}
  else EvidenceSaved&=FFileHelper::SaveArrayToFile(PrepareHistories,*HistoryPath,&IFileManager::Get(),FILEWRITE_NoReplaceExisting);
  FString Rows=FString::Printf(TEXT("Fast=%d Requested=%d Host=2 Hz=300 Driver=%s Finished=%d Failed=%d FailureLine=%d Steps=%u Scope=IsolatedNativeFixtureFullStadiumUNKNOWN\nN\tFrame\tPublication\tComplete\tMainContacts\tComputed\tReused\tWasted\tUsefulNative\tReusedNative\tReusedLanes\tActualPrepareDelta\n"),Fast?1:0,Lanes,*Driver->GetClass()->GetName(),Finished?1:0,Failed?1:0,FailureLine,Steps);
  for(uint32 N=0;N<Frames;++N){const auto& Row=Seen[N];Rows+=FString::Printf(TEXT("%u\t%llu\t%llu\t%d\t%d\t%llu\t%llu\t%llu\t%d\t%d\t%d\t%.17g\n"),N,Row.Frame,Row.Publication,Row.bComplete?1:0,Row.MainContacts,Row.TensorComputed,Row.TensorReused,Row.TensorWasted,Row.TensorUsefulNative,Row.TensorReusedNative,Row.TensorReusedLanes,double(Row.ActualDelta));}
  const FString RowPath=FPaths::Combine(CaseDir,TEXT("rows.tsv"));
  if(IFileManager::Get().FileExists(*RowPath)){AddError(FString::Printf(TEXT("[NativeAnalyticContacts1368ExportRefused] Existing file %s"),*RowPath));EvidenceSaved=false;}
  else EvidenceSaved&=FFileHelper::SaveStringToFile(Rows,*RowPath,FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM,&IFileManager::Get(),FILEWRITE_NoReplaceExisting);
  FString AdmissionRows=TEXT("N\tAttempts\tWaves\tCandidates\tEligible\tSkipped\tComputed\tReused\tWasted\tRejectBound\tRejectQuery\tRejectContext\tRejectNumeric\tRequestedMax\tLogicalMax\tUsefulNativeMax\tReusedNativeMax1213\tFallbackSerialPolicy\tFallbackTooFew\tFallbackNested\tFallbackDiagnostics\tFallbackCapacity\tFallbackInvalidBound\tFallbackNumeric\tFallbackExecutorAdmission\n");
  for(uint32 N=0;N<Frames;++N)
  {
   const auto& S=Seen[N].TensorDiagnostics;
   AdmissionRows+=FString::Printf(TEXT("%u\t%llu\t%llu\t%llu\t%llu\t%llu\t%llu\t%llu\t%llu\t%llu\t%llu\t%llu\t%llu\t%d\t%d\t%d\t%d\t%llu\t%llu\t%llu\t%llu\t%llu\t%llu\t%llu\t%llu\n"),N,S.Attempts,S.Waves,S.Candidates,S.Eligible,S.Skipped,S.Computed,S.Reused,S.Wasted,S.RejectBound,S.RejectQuery,S.RejectContext,S.RejectNumeric,S.RequestedMax,S.LogicalMax,S.UsefulNativeMax,S.ReusedNativeMax1213,S.Fallbacks[int32(Analytic::ETensorCellWaveFallback1213::SerialPolicy)],S.Fallbacks[int32(Analytic::ETensorCellWaveFallback1213::TooFew)],S.Fallbacks[int32(Analytic::ETensorCellWaveFallback1213::Nested)],S.Fallbacks[int32(Analytic::ETensorCellWaveFallback1213::Diagnostics)],S.Fallbacks[int32(Analytic::ETensorCellWaveFallback1213::Capacity)],S.Fallbacks[int32(Analytic::ETensorCellWaveFallback1213::InvalidBound)],S.Fallbacks[int32(Analytic::ETensorCellWaveFallback1213::Numeric)],S.Fallbacks[int32(Analytic::ETensorCellWaveFallback1213::ExecutorAdmission)]);
  }
  const FString AdmissionPath=FPaths::Combine(CaseDir,TEXT("tensor.admission.tsv"));
  if(IFileManager::Get().FileExists(*AdmissionPath)){AddError(TEXT("Existing Tensor admission evidence; never overwrite"));EvidenceSaved=false;}
  else EvidenceSaved&=FFileHelper::SaveStringToFile(AdmissionRows,*AdmissionPath,FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM,&IFileManager::Get(),FILEWRITE_NoReplaceExisting);
  FString BaseRows1430=TEXT("N\tFrame\tComplete\tConsumedBaseItems\tConsumedBaseNativeMax\n");
  for(uint32 N=0;N<Frames;++N){const auto& R=Seen[N];BaseRows1430+=FString::Printf(TEXT("%u\t%llu\t%d\t%llu\t%d\n"),N,R.Frame,R.bComplete?1:0,R.BaseConsumed1430,R.BaseUsefulNative1430);}
  const FString BasePath1430=FPaths::Combine(CaseDir,TEXT("base.consumed1430.tsv"));
  if(IFileManager::Get().FileExists(*BasePath1430)){AddError(TEXT("Existing native base consumption evidence; never overwrite"));EvidenceSaved=false;}
  else EvidenceSaved&=FFileHelper::SaveStringToFile(BaseRows1430,*BasePath1430,FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM,&IFileManager::Get(),FILEWRITE_NoReplaceExisting);
  const auto CarOf1534=[&](const FrameParallel::FJoinedActorBaseEvent1534& E) -> int32
  {for(int32 A=0;A<Cars.Num();++A)if(static_cast<const ISpeedComponent*>(Cars[A])==E.Actor)return A;return INDEX_NONE;};
  const auto BodyOf1534=[&](int32 Car,const USSubBody* Body) -> int32
  {if(Car<0||Car>=Cars.Num())return INDEX_NONE;const auto& Bodies=Cars[Car]->GetSubBodies();for(int32 B=0;B<Bodies.Num();++B)if(Bodies[B]==Body)return B;return INDEX_NONE;};
  const auto SaveText1534=[&](const TCHAR* Name,const FString& Value)
  {const FString Path1534=FPaths::Combine(CaseDir,Name);return !IFileManager::Get().FileExists(*Path1534)&&FFileHelper::SaveStringToFile(Value,*Path1534,FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM,&IFileManager::Get(),FILEWRITE_NoReplaceExisting);};
  FString BaseEvents1534=TEXT("N\tWave\tItem\tStage\tCar\tBody\tProducerThread\tConsumerThread\n");
  FString BaseSummary1534=TEXT("N\tComplete\tWitnessComplete\tComputed\tConsumed\tUsefulNativeMax\tOwnerThread\tCameraNative\n");
  FString BaseAdmission1552=TEXT("N\tRequestedTotal\tAdmittedLogical\tComputed\tConsumed\tUsefulNative\tOwnerThread\n");
  FString FieldRows1534=TEXT("N\tStream\tCar\tField\tBegin\tEnd\tRawBegin\tRawEnd\n");
  FString ContactRows1534=TEXT("N\tRow\tBegin\tEnd\n");
  FString CoverageRows1534=TEXT("N\tCar\tGroundedWheels\tActiveSuspension\n");
  for(uint32 N=0;N<Frames;++N)
  {
   const auto& R=Seen[N];BaseSummary1534+=FString::Printf(TEXT("%u\t%d\t%d\t%llu\t%llu\t%d\t%u\t%d\n"),N,R.bComplete?1:0,R.BaseWitnessComplete1534?1:0,R.BaseComputed1534,R.BaseConsumed1430,R.BaseUsefulNative1430,OwnerThread,R.CameraNative);
   for(int32 Car=0;Car<CarCount;++Car)CoverageRows1534+=FString::Printf(TEXT("%u\t%d\t%d\t%d\n"),N,Car,R.GroundedWheels[Car],R.ActiveSuspension[Car]?1:0);
   BaseAdmission1552+=FString::Printf(TEXT("%u\t%d\t%d\t%llu\t%llu\t%d\t%u\n"),N,Lanes,R.BaseAdmittedLogical1552,R.BaseComputed1534,R.BaseConsumed1430,R.BaseUsefulNative1430,OwnerThread);
   for(const auto& E:R.BaseEvents1534){const int32 Car=CarOf1534(E);BaseEvents1534+=FString::Printf(TEXT("%u\t%u\t%d\t%s\t%d\t%d\t%u\t%u\n"),N,E.Wave,E.Item,E.bConsumed?TEXT("Consumed"):TEXT("Computed"),Car,BodyOf1534(Car,E.Body),E.ProducerThread,E.ConsumerThread);}
   for(const auto& F:R.InputFields1534)FieldRows1534+=FString::Printf(TEXT("%u\tcommands.consumed.pack\t%d\t%s\t%d\t%d\t%d\t%d\n"),N,F.Car,F.Name,F.Begin,F.End,F.Begin,F.End);
   for(const auto& F:R.StateFields1534)FieldRows1534+=FString::Printf(TEXT("%u\tstates.canonical.pack\t%d\t%s\t%d\t%d\t%d\t%d\n"),N,F.Car,F.Name,F.Begin,F.End,F.RawBegin,F.RawEnd);
   for(int32 J=0;J+1<R.ContactBounds1534.Num();++J)ContactRows1534+=FString::Printf(TEXT("%u\t%d\t%d\t%d\n"),N,J,R.ContactBounds1534[J],R.ContactBounds1534[J+1]);
  }
  EvidenceSaved&=SaveText1534(TEXT("base.events1534.tsv"),BaseEvents1534);
  EvidenceSaved&=SaveText1534(TEXT("base.summary1534.tsv"),BaseSummary1534);
  EvidenceSaved&=SaveText1534(TEXT("base.admission1552.tsv"),BaseAdmission1552);
  EvidenceSaved&=SaveText1534(TEXT("field.spans1534.tsv"),FieldRows1534);
  EvidenceSaved&=SaveText1534(TEXT("contact.rows1534.tsv"),ContactRows1534);
  EvidenceSaved&=SaveText1534(TEXT("coverage1534.tsv"),CoverageRows1534);
  EvidenceSaved&=SaveText1534(TEXT("scope1534.json"),FString::Printf(TEXT("{\"schema\":1,\"cars\":8,\"frames\":90,\"hz\":300,\"requested_total_threads\":%d,\"available_private_workers\":%d,\"fast\":%d,\"audio_playback_allowed\":true,\"full_world_effects\":\"UNKNOWN\",\"private_callback_invocation_lists\":\"UNKNOWN\",\"audio_equivalence\":\"UNKNOWN\",\"full_physics\":false,\"performance_qualified\":false}\n"),Lanes,AvailablePrivate1534,Fast?1:0));
  AddInfo(FString::Printf(TEXT("[NativeAnalyticContacts1368Evidence] Path=%s Saved=%d"),*CaseDir,EvidenceSaved?1:0));
  if(!TestTrue(TEXT("all raw and canonical evidence persisted after join"),EvidenceSaved))return false;
  if(!TestTrue(TEXT("finite native canonical run completes"),Finished&&!Failed&&Steps==Frames)
   ||!TestTrue(TEXT("physics owner is independent of GT"),OwnerThread!=0&&OwnerThread!=GT))return false;
  for(int32 A=0;A<CarCount;++A)TestEqual(TEXT("one immutable input authored per frame per car before start"),Profiles[A]->Calls,Frames);
  uint64 ConsumedBaseTotal1430=0;int32 UsefulBaseMax1430=0;
  uint64 TotalContacts=0,Computed=0,Reused=0,Wasted=0;int32 UsefulNative=0,ReusedNative=0,ReusedLanes=0;
  uint32 GroundedFrames[CarCount]={},SuspensionFrames[CarCount]={};uint64 WheelManifoldRows=0;
  for(uint32 N=0;N<Frames;++N)
  {
   const auto& Row=Seen[N];
   if(!TestTrue(TEXT("exact N and publication cover every300Hz frame"),Row.bComplete&&Row.Frame==N&&Row.Publication==N+1))return false;
   if(!TestTrue(FString::Printf(TEXT("N%u exact base observer is complete after joined production work"),N),Row.BaseWitnessComplete1534))return false;
   TMap<uint64,FrameParallel::FJoinedActorBaseEvent1534> ComputedItems1534;TSet<uint64> ConsumedItems1534;TSet<uint32> ComputedProducers1534,UsefulProducers1534;int32 PerCarConsumed1534[CarCount]={};
   for(const auto& E:Row.BaseEvents1534)
   {
    const uint64 Key=(uint64(E.Wave)<<32)|uint32(E.Item);const int32 Car=CarOf1534(E);
    if(!TestTrue(FString::Printf(TEXT("N%u actual base event has a declared car/body/producer"),N),E.Wave>0&&E.Item>=0&&Car>=0&&BodyOf1534(Car,E.Body)>=0&&E.ProducerThread!=0))return false;
    if(!E.bConsumed)
    {
     if(!TestTrue(FString::Printf(TEXT("N%u computed base item is unique and already joined"),N),!ComputedItems1534.Contains(Key)&&E.ConsumerThread==0))return false;
     ComputedItems1534.Add(Key,E);ComputedProducers1534.Add(E.ProducerThread);
    }
    else
    {
     const auto* Computed1534=ComputedItems1534.Find(Key);
     if(!TestTrue(FString::Printf(TEXT("N%u car%d consumed item joins its exact completed producer once"),N,Car),Computed1534&&!ConsumedItems1534.Contains(Key)&&Computed1534->Actor==E.Actor&&Computed1534->Body==E.Body&&Computed1534->ProducerThread==E.ProducerThread&&E.ConsumerThread==OwnerThread))return false;
     ConsumedItems1534.Add(Key);UsefulProducers1534.Add(E.ProducerThread);++PerCarConsumed1534[Car];
    }
   }
   if(!TestTrue(FString::Printf(TEXT("N%u event counts equal actual computed/consumed native totals"),N),uint64(ComputedItems1534.Num())==Row.BaseComputed1534&&uint64(ConsumedItems1534.Num())==Row.BaseConsumed1430&&ComputedProducers1534.Num()<=Lanes))return false;
   if(Lanes==8)
   {
    if(!TestTrue(FString::Printf(TEXT("N%u eight admitted logical lanes; exact consumed work from actual multiple producers including owner"),N),Row.BaseAdmittedLogical1552==8&&UsefulProducers1534.Num()>1&&UsefulProducers1534.Num()<=Row.BaseAdmittedLogical1552&&UsefulProducers1534.Contains(OwnerThread)&&Row.BaseUsefulNative1430==UsefulProducers1534.Num()&&ComputedProducers1534.Num()==UsefulProducers1534.Num()&&ComputedItems1534.Num()==CarCount*5&&ConsumedItems1534.Num()==ComputedItems1534.Num()))return false;
    for(int32 Car=0;Car<CarCount;++Car)if(!TestTrue(FString::Printf(TEXT("N%u car%d actually consumed prepared arithmetic"),N,Car),PerCarConsumed1534[Car]==5))return false;
   }
   else if(!TestTrue(TEXT("serial control has zero claimed private base events"),Row.BaseEvents1534.IsEmpty()&&Row.BaseComputed1534==0&&Row.BaseConsumed1430==0&&Row.BaseUsefulNative1430==0))return false;
   if(!Fast&&Lanes>1&&!TestTrue(TEXT("real camera calculation ran on worker plus owner validation"),Row.CameraNative>1))return false;
   ConsumedBaseTotal1430+=Row.BaseConsumed1430;UsefulBaseMax1430=FMath::Max(UsefulBaseMax1430,Row.BaseUsefulNative1430);
   WheelManifoldRows+=Row.WheelManifoldRows;TotalContacts+=Row.MainContacts;Computed+=Row.TensorComputed;Reused+=Row.TensorReused;Wasted+=Row.TensorWasted;
   for(int32 A=0;A<CarCount;++A){GroundedFrames[A]+=Row.GroundedWheels[A]>0;SuspensionFrames[A]+=Row.ActiveSuspension[A];}
   UsefulNative=FMath::Max(UsefulNative,Row.TensorUsefulNative);ReusedNative=FMath::Max(ReusedNative,Row.TensorReusedNative);ReusedLanes=FMath::Max(ReusedLanes,Row.TensorReusedLanes);
   if(Fast||Lanes>1)
   {
    const auto& Ref=Oracle[N];
    const auto Compare1534=[&](const TCHAR* Kind,const TArray<uint8>& Actual,const TArray<uint8>& Expected,const TArray<FEightCarField1534>* ActualFields=nullptr,const TArray<FEightCarField1534>* ExpectedFields=nullptr)
    {
     const int32 Offset=EightCarsFirstByte1534(Actual,Expected);if(Offset==INDEX_NONE)return true;
     FString ActualLocation=ActualFields?EightCarsFieldAt1534(*ActualFields,Offset):TEXT("Field=Unframed");
     FString ExpectedLocation=ExpectedFields?EightCarsFieldAt1534(*ExpectedFields,Offset):TEXT("Field=Unframed");
     if(FCString::Strcmp(Kind,TEXT("contacts.canonical.pack"))==0){ActualLocation=EightCarsContactAt1534(Row.ContactBounds1534,Actual,Offset);ExpectedLocation=EightCarsContactAt1534(Ref.ContactBounds1534,Expected,Offset);}
     if(FCString::Strcmp(Kind,TEXT("wheel-manifold.canonical.pack"))==0){ActualLocation=EightCarsWheelAt1534(Actual,Offset);ExpectedLocation=EightCarsWheelAt1534(Expected,Offset);}
     const FString Difference=FString::Printf(TEXT("N=%u Fast=%d Lanes=%d Pack=%s Offset=%d ActualBytes=%d ExpectedBytes=%d Actual={%s} Expected={%s}"),N,Fast?1:0,Lanes,Kind,Offset,Actual.Num(),Expected.Num(),*ActualLocation,*ExpectedLocation);
     if(!SaveText1534(TEXT("comparison.first-failure.txt"),Difference+TEXT("\n")))AddError(TEXT("first divergence evidence could not be persisted without replacing an existing file"));
     return TestTrue(Difference,false);
    };
    if(!Compare1534(TEXT("commands.consumed.pack"),Row.Inputs,Ref.Inputs,&Row.InputFields1534,&Ref.InputFields1534)
     ||!Compare1534(TEXT("states.canonical.pack"),Row.ObservedStates,Ref.ObservedStates,&Row.StateFields1534,&Ref.StateFields1534)
     ||!Compare1534(TEXT("contacts.canonical.pack"),Row.OrderedSolve,Ref.OrderedSolve)
     ||!Compare1534(TEXT("wheel-manifold.canonical.pack"),Row.WheelManifold,Ref.WheelManifold)
     ||!Compare1534(TEXT("prepare.deltas.pack"),Row.PrepareDeltas,Ref.PrepareDeltas)
     ||(!Fast&&(!Compare1534(TEXT("camera.pose.pack"),Row.CameraPose,Ref.CameraPose)
     ||!Compare1534(TEXT("camera.checkpoint.pack"),Row.CameraCheckpoint,Ref.CameraCheckpoint)))
     ||!Compare1534(TEXT("numeric.context1534.pack"),Row.NumericContext1534,Ref.NumericContext1534)
     ||!Compare1534(TEXT("snapshots.pack"),Row.Snapshot,Ref.Snapshot))return false;
   }
  }
  if(Lanes>1&&!TestTrue(TEXT("same paired grounded run consumed real multi-producer base arithmetic"),ConsumedBaseTotal1430>0&&UsefulBaseMax1430>1))return false;
  TestTrue(TEXT("fixture exercised actual selected physical contacts"),TotalContacts>0);
  TestTrue(TEXT("actual opt-in wheel sidecar events were produced"),WheelManifoldRows>0);
  for(int32 A=0;A<CarCount;++A)
  {
   TestTrue(FString::Printf(TEXT("car%d actually grounded in this condition"),A),GroundedFrames[A]>0);
   TestTrue(FString::Printf(TEXT("car%d suspension state actually changed"),A),SuspensionFrames[A]>0);
   AddInfo(FString::Printf(TEXT("[NativeAnalyticGrounded1368Coverage] Fast=%d Lanes=%d Car=%d Frames=%u GroundedFrames=%u SuspensionFrames=%u"),Fast?1:0,Lanes,A,Frames,GroundedFrames[A],SuspensionFrames[A]));
  }
  TestEqual(TEXT("every completed speculative kernel accounted as reused or discarded"),Computed,Reused+Wasted);
  if(Lanes>1)TestTrue(TEXT("actual consumed Tensor work used multiple native threads and logical lanes"),Computed>0&&Reused>0&&UsefulNative>1&&ReusedNative>1&&ReusedLanes>1);
  AddInfo(FString::Printf(TEXT("[NativeAnalyticContacts1368] Fast=%d Requested=%d Frames=%u Hz=300 SelectedContacts=%llu Computed=%llu Reused=%llu Wasted=%llu UsefulNativeMax=%d ReusedNativeMax=%d ReusedLaneMax=%d"),Fast?1:0,Lanes,Frames,TotalContacts,Computed,Reused,Wasted,UsefulNative,ReusedNative,ReusedLanes));
  if(!Fast&&Lanes==1)
  {
   Oracle=MoveTemp(Seen);

  }
 }
 } // One bounded scene; all four condition journals survive failures.
 AddInfo(TEXT("Release20 camera contract: Normal camera outputs/checkpoints remain exactly equal across lanes and the independent oracle; Fast binding is rejected and every production camera output/native participant must remain empty/zero. Full physical/input/contact/wheel/state byte comparisons against NormalW1 remain mandatory. Cross-mode camera equality is intentionally not claimed."));
 AddInfo(TEXT("Scope: eight independent actual native plugin cars;90 consecutive300Hz frames; Normal/Fast selectors0/1,host2 at1/8total lanes; authored16-cell flat plane with separated support footprints plus one native CCD box. Exact consumed commands, all previously listed native states/history/transients, wheels/suspensions, ordered selected/dynamic contacts and real pure-camera pose/checkpoint are paired with Normal W1. Audio playback is allowed; this fixture has no production audio asset or private delegate-list proof. W8 admits eight logical lanes only when available; actual useful native producer identities INCLUDING the canonical owner are counted separately by the unchanged admission/consumption gates, which accept 2..8 useful producers on each admitted parallel frame and join their actual consumed base items. Eight requested or admitted lanes do not claim eight useful native producers. Current and every recorded wheeled state include the new bump cooldown and a null-or-canonical-stable victim ID; stale, foreign or ambiguous identities remain UNKNOWN. Original virtual/cache/wheel/callback operations remain on their owner. It is bounded arithmetic/capture evidence, not complete actor preparation parallelism, topology portability, whole-world/lifecycle/effects/private-callback/audio/fullStadium/matrix qualification or performance measurement; these remain UNKNOWN."));
 return !HasAnyErrors();
}
#endif
