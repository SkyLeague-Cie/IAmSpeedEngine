#pragma once

#include "SpeedCarCameraArmState.h"
#include "IAmSpeed/World/Simulation/SimulationFrameJournal.h"

/** Immutable values captured by a host, not live configuration or UObject state. */
struct IAMSPEED_API FSpeedCarCameraArmParameters
{
    FVector ArmLocalPosition = FVector::ZeroVector;
    FVector LocalCameraPosition = FVector::ZeroVector;
    FRotator LocalCameraRotation = FRotator(-2, 0, 0);
    bool bRotationLag = true;
    float SupersonicSpeed = 2200.0f;
    FSpeedCarCameraAimSettings Aim;
};

struct IAMSPEED_API FSpeedCarCameraFrameInput
{
    bool bValid = true;
    bool bOnGround = false;
    bool bBackCamera = false;
    float CameraYaw = 0;
    float CameraPitch = 0;
};

enum class ESpeedCarCameraStatus : uint8
{
    Valid, MissingBodies, FrameDiscontinuity, InvalidInput, CollisionUnavailable,
    UnsupportedSettings, DegradedAttached
};

enum class ESpeedCarCameraReason : uint8
{
    None, TimelineRejected, InvalidInput, VisualGeometryRejected,
    MissingCollisionAuthority, ContinuousQueryRejected, FinalEllipsoidRejected,
    InvalidRelativePose
};

struct IAMSPEED_API FSpeedCarCameraFrameResult
{
    ESpeedCarCameraStatus Status = ESpeedCarCameraStatus::MissingBodies;
    ESpeedCarCameraReason Reason = ESpeedCarCameraReason::None;
    FTransform RelativePose = FTransform::Identity;
    bool bTargetMode = false;
};

/** Host specialization inside one outer frame, never another arm/aim update.
 * The caller retains its complete state. Hooks may extend that state, but must
 * not retain a second shared arm or call Step recursively. No object references
 * from the game thread may cross into the physical producer through this seam. */
class IAMSPEED_API ISpeedCarCameraFramePolicy
{
public:
    virtual ~ISpeedCarCameraFramePolicy() = default;
    virtual void BeginFrame() {}
    virtual void EndFrame() {}
    virtual bool ValidateInput() const { return true; }
    virtual bool ValidateSettings() const { return true; }
    virtual void PrepareFrame() {}
    virtual bool IsTargetMode() const { return false; }
    virtual float PreviousTargetDistanceOffset() const { return 0; }
    virtual void AdvanceTarget(const FVector& Base, const FVector& Forward, const FVector& Up, float Delta) {}
    virtual FRotator TargetRotation() const { return FRotator::ZeroRotator; }
    /** Default owner-only camera is explicitly unconstrained. A host requiring
     * geometry supplies immutable authority here and reports failure, never a GT
     * trace fallback. Constraint history belongs to the host's complete state. */
    virtual ESpeedCarCameraReason ResolveConstraint(uint64 Frame, const FVector& ArmOrigin,
        const FVector& RequestedEye, const FQuat& FinalRotation, FVector& FinalLocation)
    {
        FinalLocation = RequestedEye;
        return ESpeedCarCameraReason::None;
    }
    virtual void CommitConstraint(uint64 Frame) {}
};

/** One fixed physical-frame writer over caller-owned state, shared by all hosts. */
class IAMSPEED_API FSpeedCarCameraArm
{
public:
    static FSpeedCarCameraArmState DefaultState();
    static FSpeedCarCameraFrameResult Step(FSpeedCarCameraArmState& State,
        const FSpeedCarCameraArmParameters& Parameters, uint64 Frame,
        const FSimulationPresentationBody& Car, const FSpeedCarCameraFrameInput& Input,
        ISpeedCarCameraFramePolicy& Policy);
};
