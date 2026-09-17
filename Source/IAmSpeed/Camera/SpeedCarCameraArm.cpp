#include "SpeedCarCameraArm.h"
#include "Misc/ScopeExit.h"

FSpeedCarCameraArmState FSpeedCarCameraArm::DefaultState()
{
    FSpeedCarCameraArmState S;
    S.BaseDistance = 350; S.ConfiguredCameraHeight = 150; S.Stiffness = .55f;
    S.SwivelSpeed = 2.9f; S.TransitionSpeed = 1.3f;
    S.DesiredRegularArmLength = 350;
    S.CachedAirCameraHorizontalTarget = FVector::ForwardVector;
    S.CachedGroundCameraHorizontalForward = FVector::ForwardVector;
    S.CarTarget = FVector::ForwardVector;
    S.PhysRotator = FRotator(-2, 0, 0);
    return S;
}

FSpeedCarCameraFrameResult FSpeedCarCameraArm::Step(FSpeedCarCameraArmState& S,
    const FSpeedCarCameraArmParameters& P, uint64 Frame, const FSimulationPresentationBody& Car,
    const FSpeedCarCameraFrameInput& Input, ISpeedCarCameraFramePolicy& Policy)
{
    Policy.BeginFrame();
    ON_SCOPE_EXIT { Policy.EndFrame(); };
    FSpeedCarCameraFrameResult Out;
    Out.bTargetMode = Policy.IsTargetMode();
    auto Reject = [&](ESpeedCarCameraStatus Status, ESpeedCarCameraReason Reason)
    {
        Out.Status = Status; Out.Reason = Reason;
        return Out;
    };
    if (S.bTimelineInvalid || (S.LastFrame != MAX_uint64 && Frame != S.LastFrame + 1))
    {
        S.bTimelineInvalid = true;
        return Reject(ESpeedCarCameraStatus::FrameDiscontinuity, ESpeedCarCameraReason::TimelineRejected);
    }
    if (!Car.StableId || !Input.bValid || Car.COMState.Location.ContainsNaN() ||
        Car.COMState.Velocity.ContainsNaN() || !Car.COMState.Rotation.IsNormalized() ||
        Car.CenterOfMassLocal.ContainsNaN() || !FMath::IsFinite(Input.CameraYaw) ||
        !FMath::IsFinite(Input.CameraPitch) || !Policy.ValidateInput())
    {
        S.bTimelineInvalid = true;
        return Reject(ESpeedCarCameraStatus::InvalidInput, ESpeedCarCameraReason::InvalidInput);
    }
    S.LastFrame = Frame;
    if (!Policy.ValidateSettings())
    {
        S.bTimelineInvalid = true;
        return Reject(ESpeedCarCameraStatus::UnsupportedSettings, ESpeedCarCameraReason::VisualGeometryRejected);
    }
    constexpr float Delta = 1.0f / 300.0f;
    const FVector Forward = Car.COMState.Rotation.GetForwardVector();
    const FVector Up = Car.COMState.Rotation.GetUpVector();
    const FVector Base = Car.OriginLocation() + Car.COMState.Rotation.RotateVector(P.ArmLocalPosition);
    Policy.PrepareFrame();
    S.bIsOnBackCam = Input.bBackCamera;
    S.bHasToDampenOffset = Input.CameraYaw == 0 && Input.CameraPitch == 0;
    S.RotOffset.Yaw = FMath::Clamp(S.RotOffset.Yaw + Input.CameraYaw * S.SwivelSpeed * 48.0f * Delta, -120.0, 120.0);
    S.RotOffset.Pitch = FMath::Clamp(S.RotOffset.Pitch + Input.CameraPitch * S.SwivelSpeed * 48.0f * Delta *
        (S.bIsSwivelInverted ? -1.0f : 1.0f), -40.0, 40.0);
    const float CarSpeed = Car.COMState.Velocity.Length();
    const float RegularLag = 44.0 * S.Stiffness + 8.0;
    S.CameraLagSpeed = FMath::Abs(CarSpeed) >= P.SupersonicSpeed
        ? RegularLag - (7 * S.Stiffness + 3) : RegularLag;
    const float PreviousOffset = Policy.PreviousTargetDistanceOffset();
    S.TargetArmLength = FMath::Max(0.0f, S.TargetArmLength - PreviousOffset);
    float DistanceToAdd = (S.LagSpeedCoeff / S.CameraLagSpeed) * Car.COMState.Velocity.Length();
    if (!Policy.IsTargetMode() && Input.bOnGround)
    {
        auto ForwardZ = Forward.Z;
        DistanceToAdd += 0.7 * FMath::Abs(ForwardZ) * S.BaseDistance;
    }
    S.DesiredRegularArmLength = S.BaseDistance + DistanceToAdd;
    S.TargetArmLength = FMath::FInterpTo(S.TargetArmLength, S.DesiredRegularArmLength, Delta, 10.0);
    S.TargetArmLength += PreviousOffset;
    if (S.bHasToDampenOffset) S.RotOffset = FMath::RInterpTo(S.RotOffset, FRotator::ZeroRotator, Delta, 5.0f);
    auto Aim = P.Aim;
    Aim.Stiffness = S.Stiffness;
    FSpeedCarCameraAim::AdvancePolicy(S, Aim, Forward, Up, Car.COMState.Velocity, Input.bOnGround);
    Policy.AdvanceTarget(Base, Forward, Up, Delta);
    const FRotator Back(S.CarRotator.Pitch, S.CarRotator.Yaw + 180.0, S.CarRotator.Roll);
    const FRotator Basic = Policy.IsTargetMode() && !S.bIsOnBackCam
        ? Policy.TargetRotation() : (S.bIsOnBackCam ? Back : S.CarRotator);
    S.PhysRotator = FMath::RInterpTo(S.PhysRotator, Basic + S.RotOffset, Delta,
        S.bIsSwitchingCam ? 10.0f * S.TransitionSpeed : 0.0f);
    if (S.bIsSwitchingCam)
    {
        S.SwitchTransitionRemainingSeconds = FMath::Max(0.0f, S.SwitchTransitionRemainingSeconds - Delta);
        if (S.SwitchTransitionRemainingSeconds <= 0) S.bIsSwitchingCam = false;
    }
    const FRotator FinalArm = P.bRotationLag
        ? FRotator(FMath::QInterpTo(FQuat(S.PreviousDesiredRot), FQuat(S.PhysRotator), Delta, S.CameraRotationLagSpeed))
        : S.PhysRotator;
    S.PreviousDesiredRot = FinalArm;
    const FVector ArmOrigin = Base + S.TargetOffset;
    const FVector End = ArmOrigin - FinalArm.Vector() * S.TargetArmLength + FinalArm.RotateVector(S.SocketOffset);
    const FQuat FinalRotation = FinalArm.Quaternion() * P.LocalCameraRotation.Quaternion();
    const FVector RequestedEye = End + FinalArm.RotateVector(P.LocalCameraPosition);
    FVector FinalLocation;
    const auto Reason = Policy.ResolveConstraint(Frame, ArmOrigin, RequestedEye, FinalRotation, FinalLocation);
    if (Reason != ESpeedCarCameraReason::None)
        return Reject(ESpeedCarCameraStatus::CollisionUnavailable, Reason);
    const FTransform PublishedPose(FinalRotation, FinalLocation);
    Policy.CommitConstraint(Frame);
    const FTransform CarPose(Car.COMState.Rotation, Car.OriginLocation());
    Out.RelativePose = PublishedPose.GetRelativeTransform(CarPose);
    Out.Status = Out.RelativePose.IsValid() ? ESpeedCarCameraStatus::Valid : ESpeedCarCameraStatus::InvalidInput;
    if (Out.Status != ESpeedCarCameraStatus::Valid) Out.Reason = ESpeedCarCameraReason::InvalidRelativePose;
    return Out;
}
