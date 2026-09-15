#pragma once
#include "SpeedCarCameraArm.h"
#include "SpeedCarCameraCommandJournal.h"

/** Generic physical extension; intentionally not the game's legacy Kind1 bytes. */
struct IAMSPEED_API FSpeedCarCameraInputSnapshot
{
    bool bOnGround = false;
    bool bBackCamera = false;
    float CameraYaw = 0, CameraPitch = 0;
    void Write(TArray<uint8>& Out) const;
    static bool Read(const TArray<uint8>& Bytes, FSpeedCarCameraInputSnapshot& Out);
};

struct IAMSPEED_API FSpeedCarCameraConfiguration
{
    FSpeedCarCameraArmParameters Arm;
    FVector SocketOffset = FVector::ZeroVector;
    float FieldOfView = 110;
    float ProbeSize = 25;
    float RotationLagSpeed = 37;
    float LagSpeedCoeff = 1;
    uint64 Version = 1;
};

struct IAMSPEED_API FSpeedCarCameraPose
{
    uint64 Owner = 0, Frame = 0, FirstFrame = 1, Generation = 1, Epoch = 0;
    uint64 SettingsVersion = 1, CommandSerial = 0;
    FSpeedCarCameraFrameResult Result;
    float FieldOfView = 110;
    bool bBackCamera = false;
    void Write(TArray<uint8>& Out) const;
    static bool Read(const TArray<uint8>& Bytes, FSpeedCarCameraPose& Out);
    bool Compose(const FSimulationPresentationOutput& Packet, const FSimulationPoseConsumption& Car,
        FTransform& OutWorld) const;
};

/** One polymorphic endpoint is selected at lifecycle binding. The common
 * simulation invokes exactly one dispatch; no base producer runs in addition
 * to a game's specialized endpoint. Persistent solver state belongs to the
 * concrete endpoint, never to this dispatch base as a duplicate. */
class IAMSPEED_API FSpeedCarCameraEndpoint : public ISimulationPresentationProducer
{
public:
    void Produce(const FSimulationSnapshot& Bodies, FSimulationPresentationOutput& Out) final
    {
        ProduceCameraFrame(Bodies, Out);
    }
protected:
    virtual void ProduceCameraFrame(const FSimulationSnapshot& Bodies, FSimulationPresentationOutput& Out) = 0;
};

/** Owner-only implementation. No fabricated target, SL decoder or live world. */
class IAMSPEED_API FSpeedCarCameraPresentation final : public FSpeedCarCameraEndpoint
{
public:
    static constexpr uint32 CameraChannel = 0x53504341;
    // Fixed-width v1: header 73 + configuration/state 442 + checksum 8.
    static constexpr int32 StateBytes = 523;
    FSpeedCarCameraPresentation(uint64 Owner, uint64 FirstFrame, uint64 Generation,
        const FSpeedCarCameraConfiguration& Configuration, const FSpeedCarCameraArmState& Initial);
    uint64 OwnerStableId() const override { return Owner; }
    uint32 Channel() const override { return CameraChannel; }
    void InvalidateTimeline() override { State.bTimelineInvalid = true; }
    bool CanRestore(const FSimulationPresentationOutput& Packet, uint64 ReplayThrough) const override;
    void RestoreValidated(const FSimulationPresentationOutput& Packet, uint64 ReplayThrough,
        FSimulationPresentationOutput& Restored) override;
    bool QueueSetting(ESpeedCarCameraCommandKind Kind, float Value);
    bool QueueAt(const FSpeedCarCameraCommand& Command) { return Journal.QueueAt(Command); }
    bool Accepts(const FSimulationPresentationOutput& Packet) const;
    static bool ReadPublishedState(const FSimulationPresentationOutput& Packet,
        FSpeedCarCameraConfiguration& Configuration, FSpeedCarCameraArmState& State,
        FSpeedCarCameraPose& Pose, uint64& JournalFrame,
        uint32& Count, uint64& PrefixHash);
protected:
    void ProduceCameraFrame(const FSimulationSnapshot& Bodies, FSimulationPresentationOutput& Out) override;
private:
    const uint64 Owner, FirstFrame, Generation;
    uint64 Epoch = 0;
    FSpeedCarCameraConfiguration Configuration;
    FSpeedCarCameraArmState State;
    TAtomic<bool> bCommandsInvalid = false;
    FSpeedCarCameraCommandJournal Journal;
    bool ApplyCommand(const FSpeedCarCameraCommand& Command);
    void CaptureState(FSimulationPresentationOutput& Packet) const;
};
