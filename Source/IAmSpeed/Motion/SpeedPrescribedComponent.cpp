#include "SpeedPrescribedComponent.h"
#include "IAmSpeed/SubBodies/Solid/SolidSubBody.h"
#include "IAmSpeed/World/Simulation/CanonicalFrameContext.h"
#include "IAmSpeed/World/Simulation/CanonicalFrameDriver.h"
#include "Engine/World.h"
#include "Serialization/MemoryReader.h"
#include "Serialization/MemoryWriter.h"
#include <stdexcept>
#include <limits>
namespace
{
struct FPrescribedSavedState
{
    uint8 Version=2,Flags=0;
    uint64 Ticks=0,LastFrame=0,FrameStartTicks=0,Episode=0;
    uint32 CurrentFrame=0;
    double Elapsed=0.,Delta=1./300.;
    int32 Stage=INDEX_NONE;
    void Serialize(FArchive& A){A<<Version<<Flags<<Ticks<<LastFrame<<FrameStartTicks<<Episode<<CurrentFrame<<Elapsed<<Delta<<Stage;}
};
bool ReadPrescribedState(TConstArrayView<uint8> Payload,int32 Stage,FPrescribedSavedState& S)
{
    if(Payload.Num()!=58)return false;
    TArray<uint8> Bytes;Bytes.Append(Payload.GetData(),Payload.Num());FMemoryReader R(Bytes);S.Serialize(R);
    if(R.IsError()||R.Tell()!=Bytes.Num()||S.Version!=2||(S.Flags&~7)||S.Stage!=Stage||!FMath::IsFinite(S.Elapsed)||!FMath::IsFinite(S.Delta)||FMath::Abs(S.Delta-1./300.)>1.e-8||S.Elapsed<0.||S.Elapsed>S.Delta+1.e-8)return false;
    if(S.FrameStartTicks>S.Ticks||S.Ticks-S.FrameStartTicks>1)return false;
    if((S.Flags&4)&&S.Ticks-S.FrameStartTicks!=1)return false;
    return true;
}
}
USpeedPrescribedComponent::USpeedPrescribedComponent(const FObjectInitializer& I):Super(I)
{
    PhysMass=0.f;bEnableGravity=false;CenterOfMass=FVector::ZeroVector;
    PrimaryComponentTick.bCanEverTick=false;
}
bool USpeedPrescribedComponent::Configure(USolidSubBody* Body,const SubBodyConfig& Config,
    const Speed::Motion::FPrescribedMotionTrack& Motion,const FTransform& Anchor,int32 Stage,
    std::shared_ptr<const Speed::Motion::FActivation> Activation)
{
    if(bAdmitted||!Body||Body->GetOwner()!=GetOwner()||!Config.bValid||Config.Mass!=0.f||Config.bEnableFakePhysics||!Motion.IsConfigured()||!Activation||Stage<0||Stage>=128||Anchor.ContainsNaN()||!Anchor.GetScale3D().Equals(FVector::OneVector,1.e-12))return false;
    // One canonical tick may not cross a velocity key. Reference key times
    // must be measured and explicitly quantized by the course author.
    if(!Motion.HasCanonicalKeyTimes())return false;
    ShapeConfig=Config;Track=Motion;AnchorPose=Anchor;StageIndex=Stage;Gate=MoveTemp(Activation);SetSubBodies({Body});bConfigured=true;return true;
}
void USpeedPrescribedComponent::BeginPlay()
{
    if(!bConfigured)throw std::runtime_error("Unconfigured prescribed solid");
    if(!GetWorld()||GetWorld()->GetNetMode()!=NM_Standalone||!Speed::CanonicalFrameDriver::IsEnabled())throw std::runtime_error("Prescribed obstacles require standalone canonical simulation");
    Super::BeginPlay();SetAsyncPhysicsTickEnabled(false);bAdmitted=true;InstallPose(0.,false);UpdateSubBodiesKinematics();
}
void USpeedPrescribedComponent::OnCreatePhysicsState(){UMovementComponent::OnCreatePhysicsState();}
bool USpeedPrescribedComponent::ValidateSimulationBindings(FString& Reason) const
{
    if(!bConfigured||!bAdmitted||!Gate||!GetWorld()||GetWorld()->GetNetMode()!=NM_Standalone||GetSubBodies().Num()!=1||!GetSubBodies()[0]||GetSubBodies()[0]->GetParentComponent()!=this||GetSubBodies()[0]->IsSimulatingPhysics()||PhysMass!=0.f||bEnableGravity||!CenterOfMass.IsZero())
    {Reason=TEXT("Prescribed solid requires an admitted standalone canonical body, immutable track, zero COM/mass and no Chaos/gravity.");return false;}
    return true;
}
void USpeedPrescribedComponent::SetKinematicState(const SKinematic& State)
{
    // Generic forces, impulses, corrections and quantizers cannot overwrite a
    // prescribed trajectory. Only InstallPose writes after canonical admission.
    if(!bAdmitted)Super::SetKinematicState(State);
}
void USpeedPrescribedComponent::InstallPose(double ActiveTicks,bool bMoving)
{
    Speed::Motion::FPrescribedMotionSample S;if(!Track.EvaluateCanonicalTicks(ActiveTicks,S))throw std::runtime_error("Invalid prescribed trajectory sample");
    SKinematic K;K.Location=AnchorPose.TransformPosition(S.Translation);K.Rotation=AnchorPose.GetRotation()*S.Rotation;
    if(bMoving)
    {
        // Positions are addressed by integer300Hz ticks, while the solver
        // integrates its actual float delta. Keep the published derivative
        // identical to that trajectory, including very fast return segments.
        const double Rate=(1./300.)/FrameDelta;
        K.Velocity=AnchorPose.TransformVectorNoScale(S.Velocity)*Rate;
        K.AngularVelocity=AnchorPose.TransformVectorNoScale(S.AngularVelocity)*Rate;
    }
    USpeedMovementComponent::SetKinematicState(K);
}
void USpeedPrescribedComponent::PrepareCanonicalFrame(const FCanonicalFrameContext& Context)
{
    if(Context.NumFrame>=std::numeric_limits<uint32>::max()||FMath::Abs(double(Context.PhysicalDeltaTime)-1./300.)>1.e-8)throw std::runtime_error("Prescribed course motion requires canonical300Hz address");
    if(Gate&&Gate->Episode!=LastEpisode)
    {
        if(Gate->Episode<LastEpisode)throw std::runtime_error("Prescribed episode rewound without snapshot restore");
        LastEpisode=Gate->Episode;Clock.ActiveTicks=0;
    }
    bMovingThisFrame=Gate&&Gate->Stage==StageIndex&&Gate->bInside&&!IsFrozen();
    if(!Clock.Advance(Context.NumFrame,Gate&&Gate->Stage==StageIndex&&Gate->bInside,IsFrozen()))throw std::runtime_error("Duplicate or discontinuous prescribed canonical frame");
    CurrentFrame=uint32(Context.NumFrame+1);FrameStartTicks=Clock.ActiveTicks-(bMovingThisFrame?1:0);FrameElapsed=0.;FrameDelta=Context.PhysicalDeltaTime;
    InstallPose(double(FrameStartTicks),bMovingThisFrame);UpdateSubBodiesKinematics();
}
void USpeedPrescribedComponent::PostIntegrateKinematics(const float& Delta)
{
    FrameElapsed+=double(Delta);
    if(FrameElapsed>FrameDelta+1.e-7)throw std::runtime_error("Prescribed trajectory integrated beyond canonical frame");
    const double Fraction=bMovingThisFrame?FMath::Clamp(FrameElapsed/FrameDelta,0.,1.):0.;
    InstallPose(double(FrameStartTicks)+Fraction,bMovingThisFrame);UpdateSubBodiesKinematics();
}
void USpeedPrescribedComponent::AppendSimulationSnapshot(TArray<uint8>& Payload) const
{
    FPrescribedSavedState S;S.Flags=(Clock.bHasFrame?1:0)|(Clock.bInside?2:0)|(bMovingThisFrame?4:0);S.Ticks=Clock.ActiveTicks;S.LastFrame=Clock.LastFrame;S.FrameStartTicks=FrameStartTicks;S.Episode=LastEpisode;S.CurrentFrame=CurrentFrame;S.Elapsed=FrameElapsed;S.Delta=FrameDelta;S.Stage=StageIndex;
    FMemoryWriter W(Payload);W.Seek(Payload.Num());S.Serialize(W);
}
bool USpeedPrescribedComponent::CanRestoreSimulationSnapshot(TConstArrayView<uint8> Payload) const
{FPrescribedSavedState S;return ReadPrescribedState(Payload,StageIndex,S);}
void USpeedPrescribedComponent::RestoreSimulationSnapshotPayload(TConstArrayView<uint8> Payload)
{
    FPrescribedSavedState S;if(!ReadPrescribedState(Payload,StageIndex,S))throw std::runtime_error("Invalid prescribed snapshot");
    Clock.ActiveTicks=S.Ticks;Clock.LastFrame=S.LastFrame;Clock.bHasFrame=(S.Flags&1)!=0;Clock.bInside=(S.Flags&2)!=0;
    FrameStartTicks=S.FrameStartTicks;LastEpisode=S.Episode;CurrentFrame=S.CurrentFrame;FrameElapsed=S.Elapsed;FrameDelta=S.Delta;bMovingThisFrame=(S.Flags&4)!=0;
    const double Fraction=bMovingThisFrame?FMath::Clamp(FrameElapsed/FrameDelta,0.,1.):0.;InstallPose(double(FrameStartTicks)+Fraction,bMovingThisFrame);UpdateSubBodiesKinematics();
}
