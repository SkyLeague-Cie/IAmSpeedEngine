#include "SensorSubBody.h"
#include "BoxSensor.h"
#include "SphereSensor.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "IAmSpeed/World/Simulation/SensorReplayState.h"
namespace
{
bool StableReference(const TWeakObjectPtr<UPrimitiveComponent>& Weak,Speed::Replay::FSensorReplayReference& Out)
{
    const auto* Body=Cast<USSubBody>(Weak.Get());const auto* Parent=Body?Body->GetParentComponent():nullptr;
    if(!Parent)return false;const auto Id=Parent->GetPublishedSimulationStableId();const auto Index=Parent->GetSubBodies().IndexOfByKey(Body);
    if(!Id || Index<0 || Index>MAX_uint16)return false;Out={Id,uint16(Index)};return true;
}
}
bool USensorSubBody::CaptureReplaySensorState(Speed::Replay::FSensorReplayState& Out)const
{
    using namespace Speed::Replay;FSensorReplayState S;
    if(const auto* Box=Cast<UBoxSensor>(this))
    {S.Shape=1;const auto D=Box->GetBoxExtent();S.Dimensions={D.X,D.Y,D.Z};}
    else if(const auto* Sphere=Cast<USphereSensor>(this)){S.Shape=2;S.Dimensions={Sphere->GetRadius(),0,0};}
    else return false;
    const auto& K=GetKinematicState();S.Kinematic={K.Location.X,K.Location.Y,K.Location.Z,K.Velocity.X,K.Velocity.Y,K.Velocity.Z,
        K.Acceleration.X,K.Acceleration.Y,K.Acceleration.Z,K.Rotation.X,K.Rotation.Y,K.Rotation.Z,K.Rotation.W,
        K.AngularVelocity.X,K.AngularVelocity.Y,K.AngularVelocity.Z,K.AngularAcceleration.X,K.AngularAcceleration.Y,K.AngularAcceleration.Z};
    const auto T=GetRelativeTransform();const auto P=T.GetLocation(),Scale=T.GetScale3D();const auto Q=T.GetRotation();
    S.LocalPosition={P.X,P.Y,P.Z};S.LocalRotation={Q.X,Q.Y,Q.Z,Q.W};S.LocalScale={Scale.X,Scale.Y,Scale.Z};
    S.CollisionChannel=uint8(GetCollisionChannel());
    static_assert(int32(ECC_OverlapAll_Deprecated)==64, "Stored sensor schema must cover every real native response channel");
    for(int32 I=0;I<int32(S.Responses.size());++I)S.Responses[I]=uint8(GetResponseParams().CollisionResponse.GetResponse(ECollisionChannel(I)));
    if(OverlapsOrdered.Num()>256 || OverlapsSet.Num()>256 || TouchedThisFrame.Num()>256)return false;
    for(const auto& Weak:OverlapsOrdered){FSensorReplayReference R;if(!StableReference(Weak,R))return false;S.Ordered.push_back(R);}
    for(const auto& Weak:OverlapsSet){FSensorReplayReference R;if(!StableReference(Weak,R))return false;S.Present.push_back(R);}
    for(const auto& Weak:TouchedThisFrame){FSensorReplayReference R;if(!StableReference(Weak,R))return false;S.Touched.push_back(R);}
    std::sort(S.Present.begin(),S.Present.end());std::sort(S.Touched.begin(),S.Touched.end());
    try{ValidateSensorState(S);}catch(const FFieldError&){return false;}
    Out=std::move(S);return true;
}
