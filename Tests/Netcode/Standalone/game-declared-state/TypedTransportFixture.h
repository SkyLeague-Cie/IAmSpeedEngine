#pragma once
#include "GameDeclaredState.h"
#include "../transport/CanonicalActorState.h"

namespace Speed::Netcode::DeclaredGameFixture
{
namespace D=DeclaredGame;
inline bool ValidateFor(const FWorldState& World,const FActorState& Actor)
{
    if(Actor.Lifecycle!=ELifecycle::Alive || Actor.SchemaVersion!=1)return false;
    const auto Outer=DecodeCanonicalActor(Actor.Payload);if(!Outer)return false;
    const D::FAddress Address{World.Frame,World.RegistryGeneration,Actor.Actor,Actor.Incarnation};
    if(Actor.Type==1)
        return Outer->MechanicalSchema==0x101 && Outer->GameplaySchema==0x102
            && bool(D::Decode<D::FCarPhysical>(Address,Outer->Mechanical))
            && bool(D::Decode<D::FCarGame>(Address,Outer->Gameplay));
    if(Actor.Type==2)
        return Outer->MechanicalSchema==0x103 && Outer->GameplaySchema==0x104
            && bool(D::Decode<D::FBallPhysical>(Address,Outer->Mechanical))
            && bool(D::Decode<D::FBallGame>(Address,Outer->Gameplay));
    return false;
}
inline bool InstallTypedFixture(FReplicatedRoster& Roster,const FWorldState Candidate)
{
    // All typed frame-addressed subpayloads are validated on a private candidate
    // before the portable immutable roster pointer can be published.
    for(const auto& Actor:Candidate.Actors)if(!ValidateFor(Candidate,Actor))return false;
    return Roster.Install(Candidate,[](const FActorState& Actor)
    {return Actor.Lifecycle==ELifecycle::Alive && bool(DecodeCanonicalActor(Actor.Payload));});
}
// All values and the world/body state are authored SOURCE FIXTURES. This is a
// transport adapter for declared subpayloads, never a complete native provider.
inline FWorldState Snapshot(U64 Sequence)
{
    FWorldState S;S.Sequence=Sequence;S.Frame=37+Sequence;S.RegistryGeneration=9;
    S.WorldPayload.assign(1001,0x5a); // Synthetic, deliberately spans >1 fragment.
    D::FCarPhysical Car;Car.boostQty=std::uint16_t(100+Sequence);Car.WheelsOmega[0]=-0.0f;
    Car.WheelsOmega[1]=37.5f;Car.JumpBitField=255;Car.flipSteeringForward=-128;
    Car.WaveDashClosureAssistState=2;Car.FirstJumpWheelWeights={-0.0f,0.2f,0.3f,0.5f};
    D::FCarGame CarGame;CarGame.CancelSpeed=-0.0f;CarGame.flipTorqueVector={123.25,-0.0,-9.5};
    CarGame.TotalAccel=float(Sequence);CarGame.bHasInfiniteBoost=true;
    D::FBallPhysical Ball;Ball.SurfaceState=2;Ball.bHasFlatFloorSupport=true;
    Ball.CachedRollingAxis={-0.0,0,1};Ball.FlatFloorSupportStableFrames=255;
    Ball.FlatFloorSupportNormal={0,0,1};Ball.FlatFloorSupportPoint={321.125,-0.0,16.5};
    D::FBallGame BallGame;BallGame.NumFrame=std::uint32_t(S.Frame);
    BallGame.TestInitialVelocity={0,-12345.25,-0.0};
    for(U64 Id:{100,200})
    {
        const D::FAddress At{S.Frame,S.RegistryGeneration,Id,1};
        FCanonicalActorState A;A.Kinematic[12]=1;A.Kinematic[0]=double(Id)+double(Sequence);
        if(Id==100)
        { A.MechanicalSchema=0x101;A.GameplaySchema=0x102;A.Mechanical=*D::Encode(At,Car);A.Gameplay=*D::Encode(At,CarGame); }
        else
        { A.MechanicalSchema=0x103;A.GameplaySchema=0x104;A.Mechanical=*D::Encode(At,Ball);A.Gameplay=*D::Encode(At,BallGame); }
        S.Actors.push_back({Id,1,std::uint32_t(Id==100?1:2),1,ELifecycle::Alive,*EncodeCanonicalActor(A)});
    }
    return S;
}
}
