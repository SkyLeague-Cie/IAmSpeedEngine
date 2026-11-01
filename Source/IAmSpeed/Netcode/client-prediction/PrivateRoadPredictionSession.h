#pragma once
#include "PrivateRoadPhysicsWorld.h"

namespace Speed::Netcode::Prediction
{
enum class EPrivateRoadCorrection : uint8 { Idle, Replaying, Adopted, Failed };
struct FPrivateRoadCorrectionWitness
{
    U64 FirstInvalidated=0,ReplayedFrames=0,ThroughFrame=0,Corrections=0;
    double MaximumPositionChange=0,MaximumVelocityChange=0;
};
struct FPrivateRoadObservedPose
{
    U64 Actor=0,Incarnation=0;
    SKinematic COMState; // completed canonical COM coordinates, not visual origin
};
struct FPrivateRoadObservedFrame
{
    U64 Frame=0;
    std::vector<FPrivateRoadObservedPose> Actors; // complete sorted semantic roster
};
enum class EPrivateRoadPoseComparison : uint8 { Equal, Different, Future, MissingHistory, RosterMismatch, Invalid };
struct FPrivateRoadPoseError
{
    U64 Actor=0,Incarnation=0;
    double Position=0,Velocity=0,Acceleration=0,RotationRadians=0,AngularVelocity=0,AngularAcceleration=0;
};
struct FPrivateRoadPoseComparison
{
    EPrivateRoadPoseComparison Status=EPrivateRoadPoseComparison::Invalid;
    U64 Frame=0;
    std::vector<FPrivateRoadPoseError> Actors;
};
// Experimental actual native prediction owner, finite retained history from
// original birth. Reconciliation constructs a NEW graph from its original
// seeds, replays genuine retained local frames and corrected remote requests,
// catches up the live tail, then adopts one immutable values publication.
// No arbitrary snapshot restore, complete lease or network authentication is
// implied. Max4096 frames: exhaustion is explicit, never silent eviction.
class IAMSPEED_API FPrivateRoadPredictionSession final
{
    struct FImpl;
    std::unique_ptr<FImpl> Impl;
    FPrivateRoadPredictionSession();
public:
    ~FPrivateRoadPredictionSession();
    FPrivateRoadPredictionSession(const FPrivateRoadPredictionSession&)=delete;
    FPrivateRoadPredictionSession& operator=(const FPrivateRoadPredictionSession&)=delete;
    static std::unique_ptr<FPrivateRoadPredictionSession> Create(std::vector<FPrivateRoadCarSeed> Seeds,U64 MaximumFrames,FString& Why);
    // Caller supplies the actual authenticated original ingress peer. Numeric
    // equality here is binding validation, NOT an authentication mechanism.
    FConfirmation ConfirmRemote(U64 OriginalAuthenticatedPeer,const FPacket& Packet);
    bool AdvanceLocal(std::shared_ptr<const BranchV2::FInputFrame> OriginalFrame,FString& Why);
    EPrivateRoadCorrection AdvanceCorrection(U64 MaximumFramesThisPulse,FString& Why);
    std::shared_ptr<const FPrivateRoadPublication> ReadLatest() const noexcept;
    std::optional<FPrivateRoadCorrectionWitness> ReadLastCorrection() const;
    // Read-only exact historical comparison against ACTUAL retained native
    // publications. Ingress must authenticate observation/roster/coordinates;
    // this API grants no source admission, fullstate coverage or pose restore.
    FPrivateRoadPoseComparison CompareObservedKinematics(const FPrivateRoadObservedFrame& Observed) const;
    bool Close(); // GT only; explicit actual native graph retirement
};
}
