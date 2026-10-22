#pragma once

#include "CoreMinimal.h"
#include "SpeedSimulation.h"
#include "SimulationFrameJournal.h"
#include "ReplayPlaybackClock.h"
#include "ReplaySimulation.generated.h"

// A game module can attach its decoded, immutable display/HUD values without
// making IAmSpeed depend on that module, its archive format or its UObject types.
struct IAMSPEED_API IReplayFrameValues
{
    virtual ~IReplayFrameValues() = default;
    virtual uint64 TypeTag() const { return 0; }
};

struct IAMSPEED_API FRecordedReplayFrame
{
    uint64 Ordinal = 0;
    FSimulationReplayFrame Physical;
    TArray<FSimulationInputCommand> CosmeticInputs;
    TSharedPtr<const IReplayFrameValues, ESPMode::ThreadSafe> Values;
};

enum class EReplayFrameRead : uint8 { Ready, Buffering, Failed };
struct IAMSPEED_API FReplayFrameReadResult
{
    EReplayFrameRead Status = EReplayFrameRead::Failed;
    uint64 Generation = 0;
    TSharedPtr<const FRecordedReplayFrame, ESPMode::ThreadSafe> Frame;
};

// Sole GT owner calls Poll; it must only consume prepared bounded values and
// submit nonblocking IO demands. It may not open/read/decompress a file here.
// Close requests cancellation. The independent IO owner retains and joins jobs.
class IAMSPEED_API IReplayFrameSource
{
public:
    virtual ~IReplayFrameSource() = default;
    virtual uint64 NumFrames() const = 0;
    virtual uint64 FirstPhysicalFrame() const = 0;
    virtual uint32 Frequency() const = 0;
    virtual FReplayFrameReadResult Poll(uint64 Ordinal, uint64 Generation) = 0;
    virtual void Close() = 0;
};

// Prepare validates the ENTIRE selected frame (including presence/archetypes).
// Commit must publish all prepared transforms/state/HUD together on the GT.
// It must never issue gameplay inputs, collision/solver steps, scores or rewards.
enum class EReplayFramePreparation : uint8 { Ready, Pending, Failed };

// Uncompiled source candidate: the paired consumer may retain these immutable
// references only while this session witness and explicit control serial live.
// No additional file owner or IO opening is introduced by this context.
struct FReplayConsumerContext
{
    TSharedPtr<IReplayFrameSource, ESPMode::ThreadSafe> Source;
    TSharedPtr<const FRecordedReplayFrame, ESPMode::ThreadSafe> Previous;
    TSharedPtr<const TAtomic<bool>, ESPMode::ThreadSafe> Session;
    uint64 Generation = 0, ControlSerial = 0;
    bool bDiscontinuity = false, bEmitForwardEffects = false, bPaused = true;
    double Rate = 1;
};

class IAMSPEED_API IReplayFrameConsumer
{
public:
    virtual ~IReplayFrameConsumer() = default;
    // Called on every accepted explicit play/pause/rate/seek, including a
    // repeated pause whose clock epoch does not advance. Must retire staging;
    // this callback cannot emit delayed one-shots or run gameplay.
    virtual void InvalidatePreparation(uint64 NewControlSerial) {}
    // Explicit controls and automatic EOF pauses can change continuous
    // presentation without selecting a new pose. Never emit one-shots here.
    virtual bool TransportChanged(const FReplayConsumerContext& Context) { return true; }
    virtual EReplayFramePreparation PrepareWithContext(const FRecordedReplayFrame& Frame,
        const FReplayConsumerContext& Context)
    { return Prepare(Frame, Context.bDiscontinuity); }
    virtual EReplayFramePreparation Prepare(const FRecordedReplayFrame& Frame, bool Discontinuity) = 0;
    virtual bool Commit(const FRecordedReplayFrame& Frame, uint64 PublicationSerial,
        bool Discontinuity, bool EmitForwardEffects) = 0;
    virtual void Close() = 0;
};

enum class ERecordedReplayState : uint8 { Closed, Buffering, Paused, Playing, Failed };

// Separate common-base replay branch: no canonical worker, physical
// Step, input session, AI, gameplay callbacks or negative-dt integration.
UCLASS(NotBlueprintable)
class IAMSPEED_API ASpeedReplaySimulation : public ASpeedSimulation
{
    GENERATED_BODY()
    Speed::Replay::FPlaybackClock Clock;
    TSharedPtr<IReplayFrameSource, ESPMode::ThreadSafe> Source;
    TSharedPtr<IReplayFrameConsumer, ESPMode::ThreadSafe> Consumer;
    TSharedPtr<const FRecordedReplayFrame, ESPMode::ThreadSafe> Selected;
    // Source candidate: keep the exact ready target while the consumer polls
    // intermediate frames or waits for asset admission. Do not fight those
    // demands by re-polling the target on the next nonblocking Select().
    TSharedPtr<const FRecordedReplayFrame, ESPMode::ThreadSafe> PreparingTarget;
    uint64 PreparingTargetGeneration = 0;
    ERecordedReplayState StateValue = ERecordedReplayState::Closed;
    uint64 PublicationSerial = 0, PublishedGeneration = 0;
    uint64 ControlSerial = 0;
    bool InvalidateExplicitControl(bool RetireContinuous=false);
    bool NotifyTransportState(bool RetireContinuous=false);
    TSharedRef<TAtomic<bool>, ESPMode::ThreadSafe> PublicationRetired = MakeShared<TAtomic<bool>, ESPMode::ThreadSafe>(false);
    bool bBuffering = false;
    // Retained across nonblocking preparation of one ordinary forward target.
    // Explicit transport changes discard it; an automatic EOF pause does not.
    bool bForwardAdvance = false;
    bool Select();
    void Fail();
public:
    ASpeedReplaySimulation();
    bool ReadLatestSimulationSnapshot(FSimulationSnapshot& OutSnapshot) const override;
    bool ReadPresentationPose(uint64 StableId, FSimulationPoseConsumption& Out) override;
    bool ReadPresentationOutput(uint64 StableId, uint32 Channel, FSimulationPresentationOutput& Out) override;
    TSharedPtr<const TAtomic<bool>, ESPMode::ThreadSafe> GetPresentationRetirementWitness() const override
    { return PublicationRetired; }
    void Tick(float DeltaSeconds) override;
    void EndPlay(const EEndPlayReason::Type Reason) override;
    bool Open(TSharedRef<IReplayFrameSource, ESPMode::ThreadSafe> InSource,
        TSharedRef<IReplayFrameConsumer, ESPMode::ThreadSafe> InConsumer);
    void Close();
    bool PlayReplay();
    void PauseReplay();
    bool SetReplayRate(double Rate);
    bool SeekReplay(int64 Ordinal);
    ERecordedReplayState State() const { return StateValue; }
    uint64 SelectedOrdinal() const { return Clock.Index(); }
    uint64 Generation() const { return Clock.Epoch(); }
    uint64 Serial() const { return PublicationSerial; }
    uint64 ExplicitControlSerial() const { return ControlSerial; }
    // Borrowed immutable values are scoped to this Replay actor/session.
    TSharedPtr<const FRecordedReplayFrame, ESPMode::ThreadSafe> Current() const { return Selected; }
};
