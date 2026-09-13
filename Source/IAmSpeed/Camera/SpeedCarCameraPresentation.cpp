#include "SpeedCarCameraPresentation.h"
#include "Serialization/MemoryReader.h"
#include "Serialization/MemoryWriter.h"

namespace SpeedCarCameraCodec
{
void Bool(FArchive& Ar, bool& Value)
{
    uint8 Encoded = Value ? 1 : 0;
    Ar << Encoded;
    if (Encoded > 1) Ar.SetError();
    Value = Encoded != 0;
}
void StateValues(FArchive& Ar, FSpeedCarCameraConfiguration& C, FSpeedCarCameraArmState& S)
{
    auto& P = C.Arm;
    Ar << P.ArmLocalPosition << P.LocalCameraPosition << P.LocalCameraRotation;
    Bool(Ar, P.bRotationLag);
    Ar << P.SupersonicSpeed << P.Aim.Stiffness << P.Aim.AirMinVelocityXY
        << P.Aim.AirVerticalPitchStartRatio << P.Aim.AirVerticalPitchFullRatio << P.Aim.AirVerticalVelocityScale;
    Ar << C.SocketOffset << C.FieldOfView << C.ProbeSize << C.RotationLagSpeed << C.LagSpeedCoeff << C.Version;
    Ar << S.BaseDistance;
    Ar << S.ConfiguredCameraHeight;
    Ar << S.Stiffness;
    Ar << S.SwivelSpeed;
    Ar << S.TransitionSpeed;
    Bool(Ar, S.bIsSwivelInverted);
    Bool(Ar, S.bIsOnBackCam);
    Bool(Ar, S.bIsSwitchingCam);
    Ar << S.SwitchTransitionRemainingSeconds;
    Bool(Ar, S.bHasToDampenOffset);
    Bool(Ar, S.bHasCameraGroundState);
    Bool(Ar, S.bWasOnGroundForCamera);
    Bool(Ar, S.bAirCameraUsesReverseVelocity);
    Bool(Ar, S.bAirCameraForwardLockedAfterFlip);
    Ar << S.CachedAirCameraHorizontalTarget;
    Ar << S.CachedGroundCameraHorizontalForward;
    Ar << S.CarTarget;
    Ar << S.CarRotator;
    Ar << S.DesiredRegularArmLength;
    Ar << S.RotOffset;
    Ar << S.PhysRotator;
    Ar << S.TargetArmLength;
    Ar << S.TargetOffset;
    Ar << S.SocketOffset;
    Ar << S.CameraLagSpeed;
    Ar << S.CameraRotationLagSpeed;
    Ar << S.LagSpeedCoeff;
    Ar << S.ProbeSize;
    Ar << S.PreviousDesiredRot;
    Ar << S.LastFrame;
    Ar << S.LastAppliedCommandSerial;
    Ar << S.LastAppliedCommandFrame;
    Bool(Ar, S.bTimelineInvalid);
}
bool Valid(const FSpeedCarCameraConfiguration& C, const FSpeedCarCameraArmState& S)
{
    const auto& P = C.Arm;
    return C.Version && FMath::IsFinite(C.FieldOfView) && C.FieldOfView > 0 && C.FieldOfView < 180 &&
        FMath::IsFinite(C.ProbeSize) && C.ProbeSize >= 0 && FMath::IsFinite(C.RotationLagSpeed) &&
        FMath::IsFinite(C.LagSpeedCoeff) && !C.SocketOffset.ContainsNaN() &&
        !P.ArmLocalPosition.ContainsNaN() && !P.LocalCameraPosition.ContainsNaN() &&
        !P.LocalCameraRotation.ContainsNaN() && FMath::IsFinite(P.SupersonicSpeed) &&
        FMath::IsFinite(P.Aim.Stiffness) && FMath::IsFinite(P.Aim.AirMinVelocityXY) &&
        FMath::IsFinite(P.Aim.AirVerticalPitchStartRatio) && FMath::IsFinite(P.Aim.AirVerticalPitchFullRatio) &&
        FMath::IsFinite(P.Aim.AirVerticalVelocityScale) &&
        S.BaseDistance > 0 && S.Stiffness >= 0 && S.Stiffness <= 1 &&
        FMath::IsFinite(S.BaseDistance) &&
        FMath::IsFinite(S.ConfiguredCameraHeight) &&
        FMath::IsFinite(S.Stiffness) &&
        FMath::IsFinite(S.SwivelSpeed) &&
        FMath::IsFinite(S.TransitionSpeed) &&
        FMath::IsFinite(S.SwitchTransitionRemainingSeconds) &&
        FMath::IsFinite(S.DesiredRegularArmLength) &&
        FMath::IsFinite(S.TargetArmLength) &&
        FMath::IsFinite(S.CameraLagSpeed) &&
        FMath::IsFinite(S.CameraRotationLagSpeed) &&
        FMath::IsFinite(S.LagSpeedCoeff) &&
        FMath::IsFinite(S.ProbeSize) &&
        !S.CachedAirCameraHorizontalTarget.ContainsNaN() &&
        !S.CachedGroundCameraHorizontalForward.ContainsNaN() &&
        !S.CarTarget.ContainsNaN() &&
        !S.CarRotator.ContainsNaN() &&
        !S.RotOffset.ContainsNaN() &&
        !S.PhysRotator.ContainsNaN() &&
        !S.TargetOffset.ContainsNaN() &&
        !S.SocketOffset.ContainsNaN() &&
        !S.PreviousDesiredRot.ContainsNaN();
}
TArray<uint8> ConfigurationIdentity(FSpeedCarCameraConfiguration C)
{
    // UI fields roll back, lifecycle parameters do not.
    C.Version = 1; C.FieldOfView = 110; C.RotationLagSpeed = 37;
    C.Arm.LocalCameraRotation = FRotator::ZeroRotator;
    auto S = FSpeedCarCameraArm::DefaultState();
    TArray<uint8> Bytes; FMemoryWriter Ar(Bytes);
    StateValues(Ar, C, S);
    return Bytes;
}
void PoseValues(FArchive& Ar, FSpeedCarCameraPose& P)
{
    uint32 Magic = 0x53504350; uint8 Version = 1;
    uint8 Status = uint8(P.Result.Status), Reason = uint8(P.Result.Reason);
    FVector Position = P.Result.RelativePose.GetLocation();
    FQuat Rotation = P.Result.RelativePose.GetRotation();
    Ar << Magic << Version << P.Owner << P.Frame << P.FirstFrame << P.Generation
        << P.Epoch << P.SettingsVersion << P.CommandSerial << Status << Reason;
    Bool(Ar, P.bBackCamera);
    Ar << P.FieldOfView << Position << Rotation;
    if (Magic != 0x53504350 || Version != 1 || !P.Owner || !P.Generation || !P.SettingsVersion ||
        P.Frame == MAX_uint64 || P.FirstFrame == MAX_uint64 ||
        P.Frame < P.FirstFrame || Status > uint8(ESpeedCarCameraStatus::DegradedAttached) ||
        Reason > uint8(ESpeedCarCameraReason::InvalidRelativePose) || Position.ContainsNaN() ||
        !Rotation.IsNormalized() || !FMath::IsFinite(P.FieldOfView) || P.FieldOfView <= 0 || P.FieldOfView >= 180)
        Ar.SetError();
    P.Result.Status = ESpeedCarCameraStatus(Status); P.Result.Reason = ESpeedCarCameraReason(Reason);
    P.Result.RelativePose = FTransform(Rotation, Position);
}
}

void FSpeedCarCameraInputSnapshot::Write(TArray<uint8>& Out) const
{
    Out.Reset(); FMemoryWriter Ar(Out);
    uint32 Magic = 0x53504349; uint8 Version = 1;
    auto Copy = *this;
    Ar << Magic << Version;
    SpeedCarCameraCodec::Bool(Ar, Copy.bOnGround);
    SpeedCarCameraCodec::Bool(Ar, Copy.bBackCamera);
    Ar << Copy.CameraYaw << Copy.CameraPitch;
}
bool FSpeedCarCameraInputSnapshot::Read(const TArray<uint8>& Bytes, FSpeedCarCameraInputSnapshot& Out)
{
    if (Bytes.Num() != 15) return false;
    FMemoryReader Ar(Bytes); uint32 Magic = 0; uint8 Version = 0;
    FSpeedCarCameraInputSnapshot Copy;
    Ar << Magic << Version;
    SpeedCarCameraCodec::Bool(Ar, Copy.bOnGround);
    SpeedCarCameraCodec::Bool(Ar, Copy.bBackCamera);
    Ar << Copy.CameraYaw << Copy.CameraPitch;
    if (Ar.IsError() || !Ar.AtEnd() || Magic != 0x53504349 || Version != 1 ||
        !FMath::IsFinite(Copy.CameraYaw) || !FMath::IsFinite(Copy.CameraPitch)) return false;
    Out = Copy;
    return true;
}
void FSpeedCarCameraPose::Write(TArray<uint8>& Out) const
{
    Out.Reset(); auto Copy = *this; FMemoryWriter Ar(Out);
    SpeedCarCameraCodec::PoseValues(Ar, Copy);
}
bool FSpeedCarCameraPose::Read(const TArray<uint8>& Bytes, FSpeedCarCameraPose& Out)
{
    if (Bytes.Num() != 124) return false;
    FMemoryReader Ar(Bytes); FSpeedCarCameraPose Copy;
    SpeedCarCameraCodec::PoseValues(Ar, Copy);
    if (Ar.IsError() || !Ar.AtEnd()) return false;
    Out = Copy; return true;
}
bool FSpeedCarCameraPose::Compose(const FSimulationPresentationOutput& Packet,
    const FSimulationPoseConsumption& Car, FTransform& OutWorld) const
{
    OutWorld = FTransform::Identity;
    TArray<uint8> CanonicalPayload;
    Write(CanonicalPayload);
    if (CanonicalPayload != Packet.Payload) return false;
    if (Result.Status != ESpeedCarCameraStatus::Valid || !Car.IsValid() ||
        Packet.Channel != FSpeedCarCameraPresentation::CameraChannel || Packet.OwnerStableId != Owner ||
        Car.Body.StableId != Owner || Packet.NumFrame != Frame || Car.NumFrame != Frame ||
        !Packet.PublicationSerial || Packet.PublicationSerial != Car.PublicationSerial ||
        !Result.RelativePose.IsValid() || !Car.Body.COMState.Rotation.IsNormalized() ||
        Car.Body.OriginLocation().ContainsNaN()) return false;
    const FTransform World = Result.RelativePose * FTransform(Car.Body.COMState.Rotation, Car.Body.OriginLocation());
    if (!World.IsValid()) return false;
    OutWorld = World; return true;
}

FSpeedCarCameraPresentation::FSpeedCarCameraPresentation(uint64 InOwner, uint64 InFirstFrame, uint64 InGeneration,
    const FSpeedCarCameraConfiguration& InConfiguration, const FSpeedCarCameraArmState& Initial)
    : Owner(InOwner), FirstFrame(InFirstFrame), Generation(InGeneration),
      Configuration(InConfiguration), State(Initial), Journal(InFirstFrame)
{
    State.ProbeSize = Configuration.ProbeSize; State.SocketOffset = Configuration.SocketOffset;
    State.CameraRotationLagSpeed = Configuration.RotationLagSpeed; State.LagSpeedCoeff = Configuration.LagSpeedCoeff;
    // The owned simulation negotiates frame zero before its first publication.
    // Zero is an address, not a missing identity; MAX remains the sentinel.
    if (!Owner || FirstFrame == MAX_uint64 || !Generation ||
        !SpeedCarCameraCodec::Valid(Configuration, State)) State.bTimelineInvalid = true;
}
bool FSpeedCarCameraPresentation::QueueSetting(ESpeedCarCameraCommandKind Kind, float Value)
{
    if (Kind == ESpeedCarCameraCommandKind::Toggle || !Journal.QueueNext(Kind, Value))
    { bCommandsInvalid.Store(true); return false; }
    return true;
}
bool FSpeedCarCameraPresentation::ApplyCommand(const FSpeedCarCameraCommand& C)
{
    if (Configuration.Version == MAX_uint64 || C.Serial != State.LastAppliedCommandSerial + 1 || C.Frame == MAX_uint64 || !FMath::IsFinite(C.Value) ||
        (State.LastFrame != MAX_uint64 && C.Frame != State.LastFrame + 1)) return false;
    auto Snap = [](float Value, float Min, float Max, float Step)
    {
        const float Clamped = FMath::Clamp(Value, Min, Max);
        return FMath::Clamp(FMath::RoundToFloat((Clamped - Min) / Step) * Step + Min, Min, Max);
    };
    switch (C.Kind)
    {
    case ESpeedCarCameraCommandKind::Height: State.ConfiguredCameraHeight = Snap(C.Value, 40, 200, 10); State.TargetOffset.Z = State.ConfiguredCameraHeight; break;
    case ESpeedCarCameraCommandKind::Distance: State.BaseDistance = Snap(C.Value, 100, 400, 10); break;
    case ESpeedCarCameraCommandKind::FieldOfView: Configuration.FieldOfView = Snap(C.Value, 60, 110, 1); break;
    case ESpeedCarCameraCommandKind::Angle: Configuration.Arm.LocalCameraRotation = FRotator(Snap(C.Value, -15, 0, 1), 0, 0); break;
    case ESpeedCarCameraCommandKind::Stiffness:
        State.Stiffness = Snap(C.Value, 0, 1, .05f);
        State.CameraRotationLagSpeed = 40.0 * State.Stiffness + 15.0;
        Configuration.RotationLagSpeed = State.CameraRotationLagSpeed;
        State.CameraLagSpeed = 44.0 * State.Stiffness + 8.0; break;
    case ESpeedCarCameraCommandKind::SwivelSpeed: State.SwivelSpeed = Snap(C.Value, 1, 10, .1f); break;
    case ESpeedCarCameraCommandKind::TransitionSpeed: State.TransitionSpeed = Snap(C.Value, 1, 2, .1f); break;
    case ESpeedCarCameraCommandKind::InvertSwivel:
        if (C.Value != 0 && C.Value != 1) return false;
        State.bIsSwivelInverted = C.Value != 0; break;
    default: return false;
    }
    ++Configuration.Version;
    State.LastAppliedCommandSerial = C.Serial; State.LastAppliedCommandFrame = C.Frame;
    return true;
}
void FSpeedCarCameraPresentation::ProduceCameraFrame(const FSimulationSnapshot& Bodies, FSimulationPresentationOutput& Out)
{
    Out = FSimulationPresentationOutput();
    Out.OwnerStableId = Owner; Out.Channel = CameraChannel; Out.NumFrame = Bodies.NumFrame;
    const FSimulationPresentationBody* Car = nullptr;
    bool bDuplicate = false;
    for (const auto& Body : Bodies.PresentationBodies)
        if (Body.StableId == Owner) { bDuplicate |= Car != nullptr; Car = &Body; }
    // Admit the complete immutable input before mutating journal, state or settings.
    // Rejected packets do not advance the timeline and cannot be restored.
    FSpeedCarCameraInputSnapshot Captured;
    FSpeedCarCameraPose Pose;
    Pose.Owner = Owner; Pose.Frame = Bodies.NumFrame; Pose.FirstFrame = FirstFrame;
    Pose.Generation = Generation; Pose.Epoch = Epoch;
    const bool bFrameValid = Bodies.NumFrame != MAX_uint64 &&
        Bodies.NumFrame == (State.LastFrame == MAX_uint64 ? FirstFrame : State.LastFrame + 1);
    if (!bFrameValid || !Car || bDuplicate ||
        !FSpeedCarCameraInputSnapshot::Read(Car->Extension, Captured) ||
        Car->COMState.Location.ContainsNaN() || Car->COMState.Velocity.ContainsNaN() ||
        !Car->COMState.Rotation.IsNormalized() || Car->CenterOfMassLocal.ContainsNaN())
    {
        Pose.Result.Status = bFrameValid ? ESpeedCarCameraStatus::InvalidInput : ESpeedCarCameraStatus::FrameDiscontinuity;
        Pose.Result.Reason = bFrameValid ? ESpeedCarCameraReason::InvalidInput : ESpeedCarCameraReason::TimelineRejected;
    }
    else
    {
        TArray<FSpeedCarCameraCommand> Commands;
        if (!Journal.Consume(Bodies.NumFrame, Commands)) State.bTimelineInvalid = true;
        for (const auto& Command : Commands) if (!ApplyCommand(Command)) State.bTimelineInvalid = true;
        if (bCommandsInvalid.Load()) State.bTimelineInvalid = true;
        FSpeedCarCameraFrameInput Input;
        Input.bOnGround = Captured.bOnGround; Input.bBackCamera = Captured.bBackCamera;
        Input.CameraYaw = Captured.CameraYaw; Input.CameraPitch = Captured.CameraPitch;
        ISpeedCarCameraFramePolicy OwnerOnly;
        Pose.Result = FSpeedCarCameraArm::Step(State, Configuration.Arm, Bodies.NumFrame, *Car, Input, OwnerOnly);
    }
    Pose.SettingsVersion = Configuration.Version;
    Pose.CommandSerial = State.LastAppliedCommandSerial; Pose.FieldOfView = Configuration.FieldOfView;
    Pose.bBackCamera = State.bIsOnBackCam;
    Pose.Write(Out.Payload); CaptureState(Out);
}
void FSpeedCarCameraPresentation::CaptureState(FSimulationPresentationOutput& Packet) const
{
    auto& Bytes = Packet.StatePayload;
    Bytes.Reset(); FMemoryWriter Ar(Bytes);
    uint32 Magic = 0x53504353; uint8 Version = 1;
    uint64 SavedOwner = Owner, SavedFirst = FirstFrame, SavedGeneration = Generation, SavedEpoch = Epoch;
    uint64 Frame = Packet.NumFrame, JournalFrame = 0, PrefixHash = 0; uint32 Count = 0;
    Journal.GetCheckpoint(JournalFrame, Count, PrefixHash);
    uint64 PoseHash = Speed::SimulationBoundary::HashBytes(Packet.Payload.GetData(), Packet.Payload.Num());
    Ar << Magic << Version << SavedOwner << SavedFirst << SavedGeneration << SavedEpoch << Frame
        << JournalFrame << Count << PrefixHash << PoseHash;
    auto C = Configuration; auto S = State;
    SpeedCarCameraCodec::StateValues(Ar, C, S);
    uint64 Hash = Speed::SimulationBoundary::HashBytes(Bytes.GetData(), Bytes.Num());
    Ar << Hash;
}
bool FSpeedCarCameraPresentation::ReadPublishedState(const FSimulationPresentationOutput& Packet,
    FSpeedCarCameraConfiguration& OutConfiguration, FSpeedCarCameraArmState& OutState,
    FSpeedCarCameraPose& OutPose, uint64& OutJournalFrame,
    uint32& OutCount, uint64& OutPrefixHash)
{
    // Reject every short/oversized packet before any FMemoryReader operation.
    if (Packet.Channel != CameraChannel || Packet.StatePayload.Num() != StateBytes) return false;
    FSpeedCarCameraPose Pose;
    if (!FSpeedCarCameraPose::Read(Packet.Payload, Pose)) return false;
    FMemoryReader Ar(Packet.StatePayload);
    uint32 Magic = 0, Count = 0; uint8 Version = 0;
    uint64 SavedOwner = 0, SavedFirst = 0, SavedGeneration = 0, SavedEpoch = 0, Frame = 0;
    uint64 JournalFrame = 0, PrefixHash = 0, PoseHash = 0, Hash = 0;
    Ar << Magic << Version << SavedOwner << SavedFirst << SavedGeneration << SavedEpoch << Frame
        << JournalFrame << Count << PrefixHash << PoseHash;
    FSpeedCarCameraConfiguration C; FSpeedCarCameraArmState S;
    SpeedCarCameraCodec::StateValues(Ar, C, S); Ar << Hash;
    if (Ar.IsError() || !Ar.AtEnd() || Magic != 0x53504353 || Version != 1 ||
        SavedOwner != Packet.OwnerStableId || SavedOwner != Pose.Owner || Frame != Packet.NumFrame ||
        Frame != Pose.Frame || SavedFirst != Pose.FirstFrame || SavedGeneration != Pose.Generation ||
        SavedEpoch != Pose.Epoch || C.Version != Pose.SettingsVersion || C.FieldOfView != Pose.FieldOfView ||
        Count > 4096 || S.LastAppliedCommandSerial != Count || Count != Pose.CommandSerial || Pose.bBackCamera != S.bIsOnBackCam ||
        (!Count && S.LastAppliedCommandFrame != MAX_uint64) ||
        (Count && (S.LastAppliedCommandFrame < SavedFirst || S.LastAppliedCommandFrame > Frame)) ||
        JournalFrame != Frame || S.LastFrame != Frame ||
        Hash != Speed::SimulationBoundary::HashBytes(Packet.StatePayload.GetData(), Packet.StatePayload.Num() - sizeof(uint64)) ||
        PoseHash != Speed::SimulationBoundary::HashBytes(Packet.Payload.GetData(), Packet.Payload.Num()) ||
        !SpeedCarCameraCodec::Valid(C, S)) return false;
    OutConfiguration = C; OutState = S; OutPose = Pose;
    OutJournalFrame = JournalFrame; OutCount = Count; OutPrefixHash = PrefixHash;
    return true;
}
bool FSpeedCarCameraPresentation::Accepts(const FSimulationPresentationOutput& Packet) const
{
    FSpeedCarCameraConfiguration C; FSpeedCarCameraArmState S; FSpeedCarCameraPose Pose;
    uint64 Frame = 0, Hash = 0; uint32 Count = 0;
    return ReadPublishedState(Packet, C, S, Pose, Frame, Count, Hash) &&
        Pose.Owner == Owner && Pose.FirstFrame == FirstFrame && Pose.Generation == Generation;
}
bool FSpeedCarCameraPresentation::CanRestore(const FSimulationPresentationOutput& Packet, uint64 ReplayThrough) const
{
    if (bCommandsInvalid.Load() || ReplayThrough < Packet.NumFrame || ReplayThrough == MAX_uint64) return false;
    FSpeedCarCameraConfiguration C; FSpeedCarCameraArmState S; FSpeedCarCameraPose Pose;
    uint64 Frame = 0, Hash = 0; uint32 Count = 0;
    return ReadPublishedState(Packet, C, S, Pose, Frame, Count, Hash) &&
        Pose.Owner == Owner && Pose.FirstFrame == FirstFrame && Pose.Generation == Generation &&
        Pose.Result.Status == ESpeedCarCameraStatus::Valid && !S.bTimelineInvalid &&
        Pose.Epoch <= Epoch && Epoch != MAX_uint64 &&
        SpeedCarCameraCodec::ConfigurationIdentity(C) == SpeedCarCameraCodec::ConfigurationIdentity(Configuration) &&
        Journal.CanRestore(Frame, Count, Hash);
}
void FSpeedCarCameraPresentation::RestoreValidated(const FSimulationPresentationOutput& Packet, uint64 ReplayThrough,
    FSimulationPresentationOutput& Restored)
{
    if (!CanRestore(Packet, ReplayThrough)) { InvalidateTimeline(); Restored = FSimulationPresentationOutput(); return; }
    FSpeedCarCameraConfiguration C; FSpeedCarCameraArmState S; FSpeedCarCameraPose Pose;
    uint64 Frame = 0, Hash = 0; uint32 Count = 0;
    if (!ReadPublishedState(Packet, C, S, Pose, Frame, Count, Hash))
    { InvalidateTimeline(); Restored = FSimulationPresentationOutput(); return; }
    Configuration = C; State = S; ++Epoch;
    Journal.RestoreValidated(Frame, Count, ReplayThrough);
    Restored = Packet; Pose.Epoch = Epoch; Pose.Write(Restored.Payload); CaptureState(Restored);
}
