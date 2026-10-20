#include "AuxiliaryFrameWitnessForTesting.h"
#if !UE_BUILD_SHIPPING
#include "IAmSpeed/Components/SpeedMovementComponent.h"
#include "NativeFrameWitnessRecorder.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "IAmSpeed/Base/EpisodeShapeFieldsForTesting.h"
#include "IAmSpeed/SubBodies/Sensor/SensorSubBody.h"
#include "IAmSpeed/SubBodies/Sensor/BoxSensor.h"
#include "IAmSpeed/SubBodies/Sensor/SphereSensor.h"
namespace AuxiliaryFrameWitnessPrivate
{
using B=Speed::FEpisodeStateBytesForTesting;
void Base(TArray<uint8>& Out,const FBasePhysicsState& X)
{B::Add(Out,X.Kinematic);B::Add(Out,X.bIsFrozen);B::Add(Out,X.nbFramesbeforeCanMove);B::Add(Out,X.bStartCountdown);}
template<class T> void Settings(TArray<uint8>& Out,const T& X)
{
 B::Add(Out,X.bEnablePositionResimulation);B::Add(Out,X.bEnableRotationResimulation);B::Add(Out,X.bEnableVelocityResimulation);B::Add(Out,X.bEnableAngularVelocityResimulation);
 B::Add(Out,X.PositionResimulationThreshold);B::Add(Out,X.RotationResimulationThreshold);B::Add(Out,X.VelocityResimulationThreshold);B::Add(Out,X.AngularVelocityResimulationThreshold);
}
}
bool USensorSubBody::AppendOverlapStateForFrameWitnessForTesting(TArray<uint8>& Out,const Speed::FFramePointerRegistry& R) const
{
 using B=Speed::FEpisodeStateBytesForTesting;
 // Ordered overlaps affect callbacks: retain their actual order unchanged.
 bool Known=true;
 const auto Valid=[&](const auto& W){return !W.IsStale(true,true) && (!W.Get() || R.Find(reinterpret_cast<UPTRINT>(W.Get())));};
 for(const auto& W:OverlapsOrdered)Known&=Valid(W);
 B::Add(Out,OverlapsOrdered.Num());for(const auto& W:OverlapsOrdered)if(!W.IsStale(true,true))B::Add(Out,W);else B::Add(Out,uint64(0));
 const auto Set=[&](const auto& Values)
 {
  auto Copy=Values.Array();bool CanSort=true;for(const auto& W:Copy)CanSort&=Valid(W) && W.Get()!=nullptr;
  Known&=CanSort;
  if(CanSort)Copy.Sort([&](const auto& A,const auto& C){return Speed::FrameWitnessKeyLess(*R.Find(reinterpret_cast<UPTRINT>(A.Get())),*R.Find(reinterpret_cast<UPTRINT>(C.Get())));});
  B::Add(Out,Copy.Num());for(const auto& W:Copy)if(!W.IsStale(true,true))B::Add(Out,W);else B::Add(Out,uint64(0));
 };
 Set(OverlapsSet);Set(TouchedThisFrame);
 if(const auto* Sphere=Cast<USphereSensor>(this)){B::Add(Out,uint8(1));B::Add(Out,Sphere->GetRadius());B::Add(Out,Sphere->GetRadiusWithMargin());}
 else if(const auto* Box=Cast<UBoxSensor>(this)){B::Add(Out,uint8(2));B::Add(Out,Box->GetBoxExtent());}
 else {B::Add(Out,uint8(0));Known=false;}
 // Handles have process-local serials; capture their semantic validity, never raw IDs.
 B::Add(Out,ContinuousOverlapHandle.IsValid());B::Add(Out,BeginOverlapHandle.IsValid());B::Add(Out,EndOverlapHandle.IsValid());
 B::Add(Out,OnBeginOverlap.IsBound());B::Add(Out,OnEndOverlap.IsBound());B::Add(Out,OnContinuousOverlap.IsBound());
 Known&=!ContinuousOverlapHandle.IsValid() && !BeginOverlapHandle.IsValid() && !EndOverlapHandle.IsValid();
 return Known;
}
void USpeedMovementComponent::RecordAuxiliaryFrameWitnessForTesting(uint64 Frame,
 TFunctionRef<bool(Speed::FFramePointerRegistry&)> Declare,
 TFunctionRef<bool(TArray<uint8>&,const Speed::FFramePointerRegistry&)> Append) const
{
 auto* Recorder=Speed::FNativeFrameWitnessRecorder::Current();if(!Recorder)return;
 using namespace AuxiliaryFrameWitnessPrivate;
 const uint64 Actor=GetPublishedSimulationStableId();
 // Every actor contributes to the same sealed registry before its capture.
 // In particular the scenario author and the car share one action contract;
 // per-actor clones must not manufacture two semantic identities for it.
 const bool Declared=Recorder->ExtendPointerRegistry([&](Speed::FFramePointerRegistry& Identities)
 {
  const auto Pointer=[&](const void* P,uint64 Role){return Speed::DeclareAuxiliaryFramePointer(Identities,P,Actor,Role);};
  bool OK=Pointer(GetOwner(),2);
  OK&=Pointer(SNetworkSettings.Get(),10);OK&=Pointer(SNetworkPhysicsComponent.Get(),11);OK&=Pointer(NetDataAsset.Get(),12);
  OK&=Pointer(NetCorrTarget.ImplementationComponent.Get(),13);
  const bool Additional=Declare(Identities);return OK && Additional;
 });
 const auto& Identities=Recorder->PointerRegistry();
 TArray<uint8> Raw,Canonical,Ledger,Effects;TArray<Speed::FFramePointerObservation> Sidecar;bool Fields=true,Known=false;
 {
  Speed::FFramePointerCaptureScope Pointers(Raw,Identities);
  B::Add(Raw,uint32(0x41555832));B::Add(Raw,Frame);B::Add(Raw,Actor);
  const UWorld* WitnessWorld=GetWorld();
  // A standalone dedicated process reports NM_DedicatedServer. Absence of both
  // live and replay drivers defines the admitted no-network observation scope.
  const bool StandaloneScope=WitnessWorld && WitnessWorld->WorldType==EWorldType::Game &&
   !WitnessWorld->GetNetDriver() && !WitnessWorld->GetDemoNetDriver() && GetOwner() && GetOwner()->HasAuthority();
  B::Add(Raw,StandaloneScope);Fields&=StandaloneScope;
  B::Add(Raw,BaseGameState.NumFrame);B::Add(Raw,BaseGameState.TestVelocity);B::Add(Raw,BaseGameState.TestAngularVelocity);B::Add(Raw,BaseGameState.bHasTestVelocity);
  Base(Raw,BasePhysicsState);B::Array(Raw,RecordedBaseFrames);B::Add(Raw,RecordedBaseStates.Num());for(const auto& X:RecordedBaseStates)Base(Raw,X);
  B::Add(Raw,EngineFPS);B::Array(Raw,SubBodies);B::Array(Raw,SolidSubBodies);
  B::Add(Raw,NetCorr_LastServerFrame);B::Add(Raw,NetCorr_LastLocalFrame);B::Add(Raw,NetCorr_BaseNumPredictedFrames);B::Add(Raw,bNetCorrHasTarget);B::Add(Raw,NetCorrTickCount);
  Base(Raw,NetCorrTarget.BaseState);B::Add(Raw,NetCorrTarget.SourceLocalFrame);B::Add(Raw,NetCorrTarget.SourceFramesSinceCanMove);B::Add(Raw,NetCorrTarget.bIsAutonomousProxy);
  Settings(Raw,NetCorrTarget.ClientNetSettings);Settings(Raw,NetCorrTarget.SimProxyNetSettings);
  B::Add(Raw,NetCorrTarget.ServerFrame);B::Add(Raw,NetCorrTarget.LocalFrame);B::Add(Raw,NetCorrTarget.bDataAltered);B::Add(Raw,NetCorrTarget.bReceivedData);B::Add(Raw,NetCorrTarget.bImportant);
#if WITH_EDITORONLY_DATA
  PRAGMA_DISABLE_DEPRECATION_WARNINGS
  B::Add(Raw,NetCorrTarget.InputFrame_DEPRECATED);
  PRAGMA_ENABLE_DEPRECATION_WARNINGS
#endif
  B::Add(Raw,NetCorrTarget.ImplementationComponent.Get());B::Add(Raw,NetCorrTarget.DeltaSourceData);B::Add(Raw,NetCorrTarget.bIsUsingDeltaSerialization);Fields&=NetCorrTarget.DeltaSourceData==nullptr;
  B::Add(Raw,MinNbFramesBeforeCanMove);B::Add(Raw,SinceCanMoveFrame);B::Add(Raw,bEnableSimulation);B::Add(Raw,PhysMass);B::Add(Raw,bEnableGravity);B::Add(Raw,GravityZ);B::Add(Raw,PhysMaxSpeed);B::Add(Raw,PhysMaxAngularSpeed);B::Add(Raw,PhysDamping);B::Add(Raw,CenterOfMass);
  B::Add(Raw,PosStabilityMultiplier);B::Add(Raw,VelStabilityMultiplier);B::Add(Raw,RotStabilityMultiplier);B::Add(Raw,AngVelStabilityMultiplier);
  B::Add(Raw,MaxPosCorrectionPerSecond);B::Add(Raw,MaxVelCorrectionPerSecond);B::Add(Raw,MaxAngVelCorrectionPerSecond);B::Add(Raw,MaxRotCorrectionPerSecond);
  B::Add(Raw,PosCorrDeadzone);B::Add(Raw,VelCorrDeadzone);B::Add(Raw,AngVelCorrDeadzone);B::Add(Raw,RotCorrDeadzone);
  B::Add(Raw,SNetworkSettings);B::Add(Raw,SNetworkPhysicsComponent);B::Add(Raw,NetDataAsset);
  B::Array(Raw,PhysicalConstraints);SleepState.AppendEpisodeFieldsForTesting<B>(Raw);KinematicQuantizationCache.AppendEpisodeFieldsForTesting<B>(Raw);
  B::Add(Raw,GetStaticCollisionWorldForFrame());B::Add(Raw,GetStaticSupportFrameHorizon());
  for(const auto* Body:SubBodies)
  {
   if(!Body || Body->GetParentComponent()!=this){Fields=false;continue;}
   Body->AppendEpisodeStateForTesting(Raw);
   if(const auto* Sensor=Cast<USensorSubBody>(Body))Fields&=Sensor->AppendOverlapStateForFrameWitnessForTesting(Raw,Identities);
  }
  Fields&=Append(Raw,Identities);Known=Declared && Pointers.MakeCanonical(Canonical);Sidecar=Pointers.Observations();
 }
 // These exact auxiliary adapters declare no consumed analog input and own no
 // input-effect ledger. Their gameplay state is in Raw, never replaced by empties.
 B::Add(Ledger,uint32(1));B::Add(Ledger,uint64(0));B::Add(Effects,uint32(1));B::Add(Effects,uint64(0));
 const bool Boundary=Actor && Recorder->CanonicalFrame()==Frame && (!bEnableSimulation || uint64(NumFrame())==Frame+1);
 Recorder->RecordActor(Actor,NumFrame(),Raw,Canonical,Ledger,Effects,Sidecar,0,Boundary,Fields,Known,true,true);
}
#endif
