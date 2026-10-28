// Test-only physical oracle, included in the existing friend fixture.
// Authored finite wheel geometry/config replaces absent Blueprint/Chaos bootstrap.
// Stock drive/steering profile is never retuned. This is not Demo asset, hardware,
// real-time pacing, server/client, restore or fullstate qualification.
using namespace Speed::Input::V2;
constexpr uint32 FrameCount=121,FirstHeld=31,FirstReleased=91;
if(!TestEqual(TEXT("fixed native physical rate remains300Hz"),ASpeedPhysicsSimulation::EngineFPS,uint32(300)))return false;
if(!TestTrue(TEXT("stock physical fixture on GT"),IsInGameThread()))return false;
const auto Contract=Speed::Driving::MakeContract(Speed::Driving::FBindings{});
if(!TestTrue(TEXT("unchanged stock seven action contract"),Contract&&Contract->GetDescription().Actions.size()==7))return false;
struct FStimulus { int32 Action; int16 Value; };
const FStimulus Stimuli[]={{-1,0},{0,255},{1,255},{2,-127},{2,127}};
struct FPhysicalRow { SKinematic K;uint32 Grounded=0;uint64 Frame=0,Serial=0;int16 Applied=0; };
const FString EvidenceRoot=FPaths::Combine(FPaths::ProjectSavedDir(),TEXT("IAmSpeed/StockAtomicTrajectories"),FGuid::NewGuid().ToString(EGuidFormats::Digits));
if(!TestTrue(TEXT("new private evidence directory"),!IFileManager::Get().DirectoryExists(*EvidenceRoot)))return false;
AddExpectedMessagePlain(TEXT("[WheelSuspensionBinding] subbodies="),ELogVerbosity::Warning,EAutomationExpectedMessageFlags::Contains,15);
for(int32 Surface=0;Surface<3;++Surface)
{
 TArray<FPhysicalRow> Neutral;double LateralNegative=0,LateralPositive=0;
 const double Slope=Surface==2?.2:0;
 const FVector Forward=FVector(1,0,Slope).GetSafeNormal(),Normal=FVector(-Slope,0,1).GetSafeNormal();
 const FQuat Rotation=FQuat::FindBetweenNormals(FVector::UpVector,Normal);
 for(int32 Case=0;Case<5;++Case)
 {
  const auto Options=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(false)
   .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false).SetTransactional(false);
  auto* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Options);
  if(!TestNotNull(TEXT("isolated stock physical world"),World))return false;
  GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
  ON_SCOPE_EXIT{World->DestroyWorld(false);GEngine->DestroyWorldContext(World);};
  auto* Car=World->SpawnActor<ASpeedCar>();auto* Driver=World->SpawnActor<ARealTimeSimulation>();auto* Bridge=World->GetSubsystem<USpeedWorldSubsystem>();
  auto* C=Car?Cast<USpeedWheeledComponent>(Car->GetVehicleMovement()):nullptr;
  if(!TestNotNull(TEXT("stock car"),C)||!TestNotNull(TEXT("physical owner"),Driver)||!TestNotNull(TEXT("world bridge"),Bridge))return false;
  Driver->SpeedWorldSubsystem=Bridge;
  auto Data=MakeShared<Speed::Analytic::FAnalyticWorldData>();
  if(Surface)
  {
   auto& P=Data->Planes.AddDefaulted_GetRef();P.SourceId=1;P.SurfaceId=2;P.FeatureId=3;P.PrimitiveId=4;
   P.ObjectType=uint32(ECC_WorldStatic);P.BlockingChannels=MAX_uint64;P.Normal=FVector3d(Normal);P.AxisU=FVector3d(Forward);P.AxisV=FVector3d::RightVector;P.HalfExtents=FVector2d(10000,10000);
  }
  FString Reason;if(!TestTrue(TEXT("finite authored plane/empty air world validates"),Data->FinalizeAndValidate(&Reason)))return false;
  if(Surface)
  {
   Data->Planes[0].bAuthorityEligible=true; // authored fixture, not imported asset certification
   auto* FloorActor=World->SpawnActor<AActor>();auto* Floor=NewObject<UBoxComponent>(FloorActor);FloorActor->AddInstanceComponent(Floor);
   Floor->SetMobility(EComponentMobility::Static);Floor->SetCollisionObjectType(ECC_WorldStatic);Floor->SetCollisionResponseToAllChannels(ECR_Block);Bridge->AnalyticSourceComponents.Add(1,Floor);
  }
  Bridge->AnalyticWorldData=Data;Bridge->SimulationWorld.SetStaticCollisionWorld(MakeUnique<Speed::FAnalyticStaticCollisionWorld>(*Data));
  C->SetOwner(Car);C->SetAsyncPhysicsTickEnabled(false);Bridge->ApplyPendingOps();
  ON_SCOPE_EXIT{Bridge->UnregisterSpeedComponent(C);Bridge->ApplyPendingOps();};
  Chaos::FSimpleWheelConfig WheelConfig;Chaos::FSimpleSuspensionConfig SuspensionConfig;
  SuspensionConfig.SetSuspensionMaxRaise(10);SuspensionConfig.SetSuspensionMaxDrop(25);
  SuspensionConfig.SpringRate=5000;SuspensionConfig.RestingForce=250000;SuspensionConfig.CompressionDamping=1000;SuspensionConfig.ReboundDamping=1000;
  auto Vehicle=MakeUnique<Chaos::FSimpleWheeledVehicle>();
  for(int32 I=0;I<4;++I)
  {
   auto& Wheel=Vehicle->Wheels.Emplace_GetRef(&WheelConfig);Wheel.MaxOmega=10000;Wheel.AppliedLinearDriveForce=0;Wheel.AppliedLinearBrakeForce=0;
   Wheel.LongitudinalAdhesiveLimit=0;Wheel.LateralAdhesiveLimit=0;Wheel.SideSlipModifier=WheelConfig.SideSlipModifier;
   Vehicle->Suspension.Emplace_GetRef(&SuspensionConfig).SetLocalRestingPosition(FVector(I<2?70:-70,I%2?45:-45,-15));
   auto* HostWheel=NewObject<UChaosVehicleWheel>(C);HostWheel->WheelRadius=30;HostWheel->SuspensionMaxRaise=10;HostWheel->SuspensionMaxDrop=25;C->Wheels.Add(HostWheel);
  }
  C->VehicleSimulationPT=MakeUnique<USpeedSimulation>();C->SkySimulation=static_cast<USpeedSimulation*>(C->VehicleSimulationPT.Get());C->SkySimulation->Init(Vehicle);C->BindWheelSimulationPointers();
  ON_SCOPE_EXIT{C->ClearWheelSimulationPointers();C->SkySimulation=nullptr;C->VehicleSimulationPT.Reset();};
  if(!TestEqual(TEXT("four real wheel bodies"),C->GetWheelSubBodies().Num(),4))return false;
  for(int32 I=0;I<4;++I){C->GetWheelSubBodies()[I]->SetLocalOffset(FVector(I<2?70:-70,I%2?45:-45,-15));C->GetWheelSubBodies()[I]->SetRadiusForConfiguration(30);}
  if(!TestTrue(TEXT("native mechanics bindings admitted"),C->ValidateSimulationBindings(Reason)))return false;
  C->SetEngineFPS(300);C->SetImmediateMovementForTesting();
  SKinematic Initial;Initial.Location=Surface?Normal*45:FVector(0,0,1000);Initial.Velocity=Forward*500;Initial.Rotation=Rotation;Initial.AngularVelocity=FVector::ZeroVector;C->SetKinematicState(Initial);
  if(!TestTrue(TEXT("stock coefficients and air controls unchanged"),C->ThrottleAccel==750&&C->BrakeAccel==1100&&C->PhysDamping==0&&C->AirForwardThrottle==0&&C->AirBackwardThrottle==0&&C->CanonicalCameraYawAction==5&&C->CanonicalCameraPitchAction==6))return false;
  if(!TestTrue(TEXT("real driver admits physical world"),Driver->EnsureSimulationWorldReady()))return false;
  Driver->TransitionExecutionMode(ESimulationExecutionMode::IAmSpeedThread);Driver->bPublishCompletedWorldInput=true;Driver->InitializeCanonicalFrame(0);
  FSessionDescriptor Binding;Binding.Id=Binding.Controller=Binding.Producer=9101;Binding.Epoch=1;Binding.First=0;Binding.Contract=Contract;Binding.Kind=EProducerContract::ExactScenario;
  const uint64 Actor=Bridge->GetSimulationStableId(*C);Binding.Actors={{Actor,Actor}};
  for(uint32 N=0;N<FrameCount;++N)
  {
   FInputFrameData D;D.SourceSequence=N+1;D.ConsumptionFrame=N;D.Producer={Speed::Input::EProducerKind::Device,9101};D.StreamEpoch={1};D.DeviceGeneration={1};D.Reset=N==0;
   const auto& Stim=Stimuli[Case];const bool Held=N>=FirstHeld&&N<FirstReleased;
   if(Stim.Action>=0&&Held){D.Values[Stim.Action]=Stim.Value;D.ActiveMask=uint32(1)<<Stim.Action;}
   if(Stim.Action>=0&&(N==FirstHeld||N==FirstReleased))D.Transitions.push_back({Speed::Input::FActionId(Stim.Action),N==FirstHeld?ETransition::Started:ETransition::Completed,int16(N==FirstHeld?Stim.Value:0),{N+1,0}});
   Binding.Scenario.emplace_back(Contract,D);if(!TestTrue(TEXT("complete exact stock action frame valid"),Binding.Scenario.back().IsValidFor(*Contract)))return false;
  }
  if(!TestTrue(TEXT("claim produced canonical authority"),C->SetFrameInputStreamV2(nullptr)))return false;
  FBoundaryCommandDescriptor Command;Command.Id=Driver->AllocateInputCommandId();Command.WorkerGeneration=Driver->GetInputWorkerGeneration();Command.RegistryVersion=0;Command.Binding=Binding;
  if(!TestTrue(TEXT("actual exact timeline queued before owner"),Driver->SubmitInputSessionCommand(Command)==ECommandAdmission::Enqueued))return false;
  Driver->ResumeOwnedSimulation();TArray<FPhysicalRow> Rows;Rows.SetNum(FrameCount);uint32 Steps=0,OwnerThread=0;int32 FailureLine=0;bool Failed=false;
  Driver->SimulationWorker=MakeUnique<FSimulationWorker>([&]
  {
   OwnerThread=FPlatformTLS::GetCurrentThreadId();if(Steps==FrameCount)return ESimulationWorkerResult::Complete;
   const uint32 N=Steps;if(!Driver->RunCanonicalFrames(1)){Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
   const auto Pair=Driver->ReadCompletedWorldAndInputFrame();const auto Input=Driver->ReadCompletedInputFrame();
   if(!Pair||!Input||Pair->ConsumedInputs()!=Input||Input->Frame!=N||Input->Inputs.size()!=1||!Input->Inputs[0].Snapshot||Pair->World().NumFrame!=N||Pair->RegistryAtFrame()!=Input->Registry)
   {Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
   const auto& Snapshot=*Input->Inputs[0].Snapshot;const auto& Applied=Snapshot.Applied;
   if(C->GetWheeledPhysicalInputForWitness().Throttle!=Applied[0]||C->GetWheeledPhysicalInputForWitness().Brake!=Applied[1]||C->GetWheeledPhysicalInputForWitness().Steer!=Applied[2])
   {Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
   auto& R=Rows[N];R.K=C->GetKinematicState();R.Grounded=C->NumWheelsOnGround();R.Frame=N;R.Serial=Pair->World().PublicationSerial;R.Applied=Case?Applied[Stimuli[Case].Action]:0;
   if(R.K.Location.ContainsNaN()||R.K.Velocity.ContainsNaN()||R.K.AngularVelocity.ContainsNaN()||!R.K.Rotation.IsNormalized())
   {Failed=true;FailureLine=__LINE__;return ESimulationWorkerResult::Failed;}
   ++Steps;return ESimulationWorkerResult::Advanced;
  },[](FSimulationWorkerWaitContext&){},[&](bool Paused){return Driver->ServiceInputSessionBoundary(Paused);},[&]{Driver->CloseInputSessionsOnWorker();});
  if(!TestTrue(TEXT("physical worker starts"),Driver->SimulationWorker->Start()))return false;
  ON_SCOPE_EXIT{Driver->StopOwnedWorker();};
  const double Deadline=FPlatformTime::Seconds()+30;
  while(Driver->SimulationWorker->IsRunning()&&FPlatformTime::Seconds()<Deadline)FPlatformProcess::SleepNoStats(.001f);
  const bool Completed=!Driver->SimulationWorker->IsRunning();Driver->StopOwnedWorker();
  if(!TestTrue(FString::Printf(TEXT("all actual physical frames complete surface%d case%d failureline%d"),Surface,Case,FailureLine),Completed&&!Failed&&Steps==FrameCount&&OwnerThread!=GGameThreadId))return false;
  uint32 SupportedHeld=0;FString Csv=TEXT("N,serial,grounded,applied,x,y,z,vx,vy,vz\n");
  for(uint32 N=0;N<FrameCount;++N)
  {
   const auto& R=Rows[N];if(!TestTrue(TEXT("real frame and serial retained"),R.Frame==N&&R.Serial==N+1))return false;
   if(N>=FirstHeld&&N<FirstReleased)SupportedHeld+=R.Grounded>0;
   if(!Case)continue;
   const auto& Ref=Neutral[N];
   if(Surface==0||N<FirstHeld)
    if(!TestTrue(TEXT("independent neutral physical trajectory matches when no traction or action absent"),R.K.Location==Ref.K.Location&&R.K.Velocity==Ref.K.Velocity&&R.K.Rotation==Ref.K.Rotation))return false;
   if(N>=FirstReleased&&!TestEqual(TEXT("release consumed physically without held action"),R.Applied,int16(0)))return false;
  }
  for(const auto& R:Rows)Csv+=FString::Printf(TEXT("%llu,%llu,%u,%d,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\n"),R.Frame,R.Serial,R.Grounded,int(R.Applied),R.K.Location.X,R.K.Location.Y,R.K.Location.Z,R.K.Velocity.X,R.K.Velocity.Y,R.K.Velocity.Z);
  IFileManager::Get().MakeDirectory(*EvidenceRoot,true);const FString File=FPaths::Combine(EvidenceRoot,FString::Printf(TEXT("surface%d-case%d.csv"),Surface,Case));
  if(!TestTrue(TEXT("fresh authored physical evidence saved"),!IFileManager::Get().FileExists(*File)&&FFileHelper::SaveStringToFile(Csv,*File)))return false;
  if(!Surface){if(!TestTrue(TEXT("air fixture actually falls and remains unsupported"),SupportedHeld==0&&Rows.Last().K.Location.Z<Initial.Location.Z&&Rows.Last().K.Velocity.Z<0))return false;}
  else
  {
   if(!TestTrue(TEXT("ground/ramp case actually uses wheel support during stimulus"),SupportedHeld>0))return false;
   const auto& End=Rows[FirstReleased-1];
   if(Case==1&&!TestTrue(TEXT("throttle increases physical forward velocity versus separate neutral control"),FVector::DotProduct(End.K.Velocity-Neutral[FirstReleased-1].K.Velocity,Forward)>1))return false;
   if(Case==2&&!TestTrue(TEXT("brake reduces physical forward velocity versus separate neutral control"),FVector::DotProduct(End.K.Velocity-Neutral[FirstReleased-1].K.Velocity,Forward)<-1))return false;
   if(Case==3)LateralNegative=End.K.Location.Y-Neutral[FirstReleased-1].K.Location.Y;
   if(Case==4)LateralPositive=End.K.Location.Y-Neutral[FirstReleased-1].K.Location.Y;
  }
  AddInfo(FString::Printf(TEXT("[StockCanonicalTrajectory] Surface=%d Case=%d Frames=%u SupportedHeld=%u EndX=%.9g EndY=%.9g EndZ=%.9g PhysicalOwner=%u Evidence=%s Hardware=NONE Pacing=NOT_QUALIFIED ClientMatch=NOT_RUN"),Surface,Case,Steps,SupportedHeld,Rows.Last().K.Location.X,Rows.Last().K.Location.Y,Rows.Last().K.Location.Z,OwnerThread,*File));
  if(!Case)Neutral=Rows;
 }
 if(Surface&&!TestTrue(TEXT("opposite steering produces nonzero opposite physical lateral displacement"),LateralNegative*LateralPositive<0&&FMath::Abs(LateralNegative)>.01&&FMath::Abs(LateralPositive)>.01))return false;
}
return true;
