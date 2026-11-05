#pragma once
#include "PrivateRoadPhysicsWorld.h"
#include <optional>
#include <vector>

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
// Diagnostic value copies from actual completed publications only. Complete
// below means the finite observed INPUT prefix, never complete causal state.
enum class EPrivateRoadInputPrefixCopy : uint8
{ Unavailable,CompleteObservedPrefix,Missing,Rejected,Overflow };
struct FPrivateRoadObservedActorInput
{
    U64 Actor=0,Incarnation=0;
    Speed::Input::FActionValues Requested{},Before{},Applied{};
    uint32 ActiveMask=0;
    EBranchRequestOrigin Origin=EBranchRequestOrigin::SpeculativeHoldRequest;
    bool OriginEvidenceKnown=false,OriginEvidenceConsistent=false;
    bool ResetEvidenceKnown=false,ActualReset=false;
    bool RoleKnown=false;
    EBranchFilterRole ExpectedRole=EBranchFilterRole::RemoteRequests;
    FBranchFilterState Filter;
    bool CanMove=false,Frozen=false;
    uint16 CountdownFrames=0;
    bool operator==(const FPrivateRoadObservedActorInput&) const = default;
};
inline bool PrivateRoadObservedRoleOriginConsistent(const FPrivateRoadObservedActorInput& A)
{
    if(!A.RoleKnown||!A.OriginEvidenceKnown||!A.OriginEvidenceConsistent)return false;
    if(A.ExpectedRole==EBranchFilterRole::LocalDevice)return A.Origin==EBranchRequestOrigin::RetainedLocalDevice;
    return A.ExpectedRole==EBranchFilterRole::RemoteRequests
        &&(A.Origin==EBranchRequestOrigin::ReceivedAuthorityRequest||A.Origin==EBranchRequestOrigin::SpeculativeHoldRequest);
}
struct FPrivateRoadObservedInputFrame
{
    U64 Frame=0,LocalKernelLifetime=0;
    bool AwaitingInitialPublication=false;
    std::vector<uint8> DeclaredSettingsIdentity; // declared slots only
    std::vector<FPrivateRoadObservedActorInput> Actors;
    bool operator==(const FPrivateRoadObservedInputFrame&) const = default;
};
struct FPrivateRoadObservedInputPrefix
{
    EPrivateRoadInputPrefixCopy Status=EPrivateRoadInputPrefixCopy::Unavailable;
    U64 CompletedGraphAdoptions=0;
    // This is the SAME already-public immutable value publication selected
    // under Gate as Prefix. It exposes no native object/address or operation.
    std::shared_ptr<const FPrivateRoadPublication> Publication;
    std::vector<FPrivateRoadObservedInputFrame> Prefix;
};
IAMSPEED_API std::optional<FPrivateRoadObservedInputFrame>
    CopyPrivateRoadConsumedInputFrameForDiagnostics(const FPrivateRoadPublication& Publication);

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
    // Session validates the actual original-server startup values before calling.
    // Complete native hidden-state/restore admission remains a separate gate.
    bool ReleaseInitialPublication(FString& Why);
    bool AdvanceLocal(std::shared_ptr<const BranchV2::FInputFrame> OriginalFrame,FString& Why);
    EPrivateRoadCorrection AdvanceCorrection(U64 MaximumFramesThisPulse,FString& Why);
    // Bounded values-only input observation. UNKNOWN never grants equality,
    // restoration, source/lifetime authority or a raw Device receipt.
    FPrivateRoadObservedInputPrefix CaptureConsumedInputPrefixForDiagnostics(U64 MaximumFramesToCopy) const;
    std::shared_ptr<const FPrivateRoadPublication> ReadLatest() const noexcept;
    std::optional<FPrivateRoadCorrectionWitness> ReadLastCorrection() const;
    // Read-only exact historical comparison against ACTUAL retained native
    // publications. Ingress must authenticate observation/roster/coordinates;
    // this API grants no source admission, fullstate coverage or pose restore.
    FPrivateRoadPoseComparison CompareObservedKinematics(const FPrivateRoadObservedFrame& Observed) const;
    bool Close(); // GT only; explicit actual native graph retirement
};
}
