#pragma once
// Read-only canonical values of the actual private FOwnedComponentValues.
// This is NOT an exclusion/lifetime capability or a complete world checkpoint.
#include "OriginalRoadBodyValueCapture.h"
#include "IAmSpeed/Netcode/Common/NativeCommonCacheAccess.h"
#include "IAmSpeed/Netcode/Common/CommonComponentState.h"
#include "IAmSpeed/Base/PhysicalContactConstraint.h"

namespace Speed::Netcode::Prediction::OriginalComponentCapture
{
using OriginalBodyCapture::FWriter;

inline void Append(FWriter& A,const SBaseGameState& V)
{A.Value(V.NumFrame);A.Value(V.TestVelocity);A.Value(V.TestAngularVelocity);A.Value(V.bHasTestVelocity);}
inline void Append(FWriter& A,const FBasePhysicsState& V)
{A.Value(V.Kinematic);A.Value(V.bIsFrozen);A.Value(V.nbFramesbeforeCanMove);A.Value(V.bStartCountdown);}
inline void Append(FWriter& A,const FWheeledGameState& V)
{
    A.Value(V.bGroundState);A.Value(V.bIsUpsideDown);
    for(float X:V.WheelsOmega)A.Value(X);
    for(float X:V.WheelsAngularPosition)A.Value(X);
    A.Value(V.NbWheelsOnGround);A.Value(V.NbFrameSinceInAir);
}
inline void Append(FWriter& A,const FWheeledPhysicsState& V)
{
    A.Value(V.bStartCountdown);A.Value(V.nbFramesbeforeCanMove);A.Value(V.FramesSinceLastImpact);
    A.Value(V.LastCarBumpVictim);A.Value(V.CarBumpCooldownFrames);
    A.Value(static_cast<const FVector&>(V.AllowedSideVelocity));A.Value(V.AllowedAngularVelocity);
    for(float X:V.SuspensionLastDisplacement)A.Value(X);
    A.Value(V.PhysicalThrottleInput);A.Value(V.PhysicalBrakeInput);A.Value(V.PhysicalSteerInput);A.Value(V.NbFramesSinceGroundContact);
}
inline void Append(FWriter& A,const FWheeledInputState& V)
{
    A.Value(V.bCanMove);A.Value(V.Throttle);A.Value(V.Brake);A.Value(V.Steer);
    A.Value(V.Camera.Version);A.Value(V.Camera.Flags);A.Value(V.Camera.Yaw);A.Value(V.Camera.Pitch);
}
inline void AppendConstraint(FWriter& A,const FPhysicalContactConstraint& V)
{
    // Distinguish explicit null from stale, and preserve the original order.
    const auto* Source=V.SourceSubBody.Get();const auto* Other=V.OtherComponent.Get();
    if((!Source&&!V.SourceSubBody.IsExplicitlyNull())||(!Other&&!V.OtherComponent.IsExplicitlyNull()))
        throw Replay::FFieldError("stale original component constraint reference");
    const uint32 SourceId=Source?Source->GetUniqueID():0,OtherId=Other?Other->GetUniqueID():0;
    if(V.PairKey!=Speed::CommonState::ConstraintPairKey(SourceId,OtherId))
        throw Replay::FFieldError("original component constraint key is not the declared native pair");
    A.Value(V.Normal);A.Value(V.ContactPoint);A.Value(V.OtherComponent);A.Value(V.SourceSubBody);
    A.Value(V.PenetrationDepth);A.Value(V.TOI);
    // PairKey is not dropped or zeroed. Its exact native derivation was checked
    // above; canonical source/other keys encode it without sending UObject IDs.
    A.Value(uint8(1)); // native HIGH32|LOW32 pair-key derivation schema
    A.Value(V.FramesSinceSeen);A.Value(V.bPersistent);
}
template<class Values>void AppendOwned(FWriter& A,const Values& V,
    uint64 ActualPublishedStableId,const Speed::IStaticCollisionWorld* OriginalStaticWorld,
    uint64 OriginalStaticSourceId)
{
    // The owning world's real graph validation supplies these original values.
    // Equality here is data/identity validation, never a boundary admission.
    const uint64 Id=V.PublishedStableId.Load();
    if(!Id||Id!=ActualPublishedStableId)throw Replay::FFieldError("original component published binding changed");
    A.Value(Id);Append(A,V.BaseGameState);Append(A,V.BasePhysicsState);
    Append(A,V.WheeledGameState);Append(A,V.WheeledPhysicsState);Append(A,V.WheeledPhysicalInput);
    if(V.PhysicalConstraints.Num()>65536)throw Replay::FFieldError("original component constraint count bound");
    A.Value(uint32(V.PhysicalConstraints.Num()));
    for(const auto& C:V.PhysicalConstraints)AppendConstraint(A,C);
    const auto S=Speed::FCommonCacheValueAccess::Capture(V.SleepState);
    const auto Q=Speed::FCommonCacheValueAccess::Capture(V.KinematicQuantizationCache);
    if(!Speed::CommonState::ValidCaches(S,Q))throw Replay::FFieldError("nonfinite original component cache");
    for(double X:S.Location)A.Value(X);for(double X:S.Rotation)A.Value(X);A.Value(S.HasCompletedPose);
    for(double X:Q.NoOpInput)A.Value(X);A.Value(Q.Valid);
    const bool HasAuthority=V.StaticRestingWorld!=nullptr;
    if(HasAuthority&&(!OriginalStaticSourceId||V.StaticRestingWorld!=OriginalStaticWorld))
        throw Replay::FFieldError("foreign original static resting authority");
    A.Value(HasAuthority);
    if(HasAuthority)A.Value(OriginalStaticSourceId);
    A.Value(V.StaticSupportFrameHorizon); // includes actual signed zero; never normalize
}
}
