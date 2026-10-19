#include "ReplaySimulation.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "SpeedPhysicsSimulation.h"

ASpeedReplaySimulation::ASpeedReplaySimulation()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = false;
    PrimaryActorTick.TickGroup = TG_PrePhysics;
    bReplicates = false;
    SetActorEnableCollision(false);
}

bool ASpeedReplaySimulation::Open(TSharedRef<IReplayFrameSource, ESPMode::ThreadSafe> InSource,
    TSharedRef<IReplayFrameConsumer, ESPMode::ThreadSafe> InConsumer)
{
    check(IsInGameThread());
    if (StateValue != ERecordedReplayState::Closed || !GetWorld() || GetNetMode() != NM_Standalone) return false;
    // An existing live driver is an isolation failure, never something to pause
    // or take over. Other replay drivers in this world are also refused.
    for (TActorIterator<ASpeedPhysicsSimulation> It(GetWorld()); It; ++It) return false;
    for (TActorIterator<ASpeedReplaySimulation> It(GetWorld()); It; ++It) if (*It != this) return false;
    const uint64 Count = InSource->NumFrames(), First = InSource->FirstPhysicalFrame();
    if (!Count || First > MAX_uint64 - (Count - 1) || !Clock.Open(Count, InSource->Frequency())) return false;
    PublicationRetired = MakeShared<TAtomic<bool>, ESPMode::ThreadSafe>(false); // New session witness; old retired witnesses stay true.
    Source = InSource; Consumer = InConsumer; StateValue = ERecordedReplayState::Buffering;
    bBuffering = true; bForwardAdvance = false; SetActorTickEnabled(true);
    return Select(); // Buffering is an accepted open, not an IO retry.
}

void ASpeedReplaySimulation::Fail()
{
    Close(); StateValue = ERecordedReplayState::Failed;
}

void ASpeedReplaySimulation::Close()
{
    check(IsInGameThread()); SetActorTickEnabled(false);
    if (Source) Source->Close();
    if (Consumer) Consumer->Close();
    Source.Reset(); Consumer.Reset(); Selected.Reset(); Clock.Close();
    PublishedGeneration = 0; bBuffering = false; bForwardAdvance = false; StateValue = ERecordedReplayState::Closed;
    PublicationRetired->Store(true);
    // No OS join is inferred here. The module's independent retained IO owner
    // still owns the actual cancel/drain/join across map/actor destruction.
}

bool ASpeedReplaySimulation::Select()
{
    if (!Source || !Consumer) return false;
    const uint64 Ordinal = Clock.Index(), Generation = Clock.Epoch();
    if (Selected && Selected->Ordinal == Ordinal && PublishedGeneration == Generation)
    {
        bBuffering = false; bForwardAdvance = false;
        StateValue = Clock.IsPaused() ? ERecordedReplayState::Paused : ERecordedReplayState::Playing;
        return true;
    }
    auto Result = Source->Poll(Ordinal, Generation);
    // A stale async reply cannot replace the currently selected frame. Poll is
    // responsible for coalescing the desired demand rather than duplicating IO.
    if (Result.Generation != Generation || Result.Status == EReplayFrameRead::Buffering)
    { bBuffering = true; StateValue = ERecordedReplayState::Buffering; return true; }
    if (Result.Status != EReplayFrameRead::Ready || !Result.Frame ||
        Result.Frame->Ordinal != Ordinal ||
        Result.Frame->Physical.Snapshot.NumFrame != Source->FirstPhysicalFrame() + Ordinal ||
        PublicationSerial == MAX_uint64)
    { Fail(); return false; }
    // Ordinary 300Hz playback can cross several physical frames per rendered
    // frame. That is a forward interval, not a seek. Consumers reconstruct
    // continuous effects from the recorded observer and must use recorded event
    // ranges for one-shots; they must not invoke live gameplay callbacks.
    const bool Discontinuity = !Selected || PublishedGeneration != Generation ||
        Ordinal < Selected->Ordinal;
    // Advancing directly to EOF may pause the clock before this target commits.
    // That interval still contains recorded events; an explicit pause/seek/rate
    // change clears the pending permission instead of replaying a delayed burst.
    const bool Effects = !Discontinuity && bForwardAdvance && Clock.Speed() > 0;
    const auto Prepared = Consumer->Prepare(*Result.Frame, Discontinuity);
    if (Prepared == EReplayFramePreparation::Failed) { Fail(); return false; }
    if (Prepared == EReplayFramePreparation::Pending)
    { bBuffering = true; StateValue = ERecordedReplayState::Buffering; return true; }
    const uint64 NextSerial = PublicationSerial + 1;
    if (!Consumer->Commit(*Result.Frame, NextSerial, Discontinuity, Effects)) { Fail(); return false; }
    PublicationSerial = NextSerial;
    Selected = MoveTemp(Result.Frame); PublishedGeneration = Generation; bBuffering = false;
    bForwardAdvance = false;
    StateValue = Clock.IsPaused() ? ERecordedReplayState::Paused : ERecordedReplayState::Playing;
    return true;
}

void ASpeedReplaySimulation::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds); check(IsInGameThread());
    if (!Source || !Consumer) return;
    // Disk wait never accrues catch-up time. Once ready, resume from that exact
    // target; do not skip an extra frame using the buffering interval's delta.
    if (!bBuffering)
    {
        const bool Forward = !Clock.IsPaused() && Clock.Speed() > 0;
        const uint64 Before = Clock.Index();
        if (!Clock.Advance(double(DeltaSeconds))) { Fail(); return; }
        bForwardAdvance = Forward && Clock.Index() > Before;
    }
    Select();
}

bool ASpeedReplaySimulation::PlayReplay()
{
    check(IsInGameThread()); if (!Source || !Clock.Play()) return false;
    return Select();
}
void ASpeedReplaySimulation::PauseReplay()
{
    check(IsInGameThread()); bForwardAdvance = false; Clock.Pause(); if (Source) Select();
}
bool ASpeedReplaySimulation::SetReplayRate(double Rate)
{
    check(IsInGameThread());
    if (!Source || !Clock.SetRate(Rate)) return false;
    bForwardAdvance = false; return true;
}
bool ASpeedReplaySimulation::SeekReplay(int64 Ordinal)
{
    check(IsInGameThread()); if (!Source || !Clock.Seek(Ordinal)) return false;
    bForwardAdvance = false;
    return Select();
}
void ASpeedReplaySimulation::EndPlay(const EEndPlayReason::Type Reason)
{
    Close(); Super::EndPlay(Reason);
}


bool ASpeedReplaySimulation::ReadLatestSimulationSnapshot(FSimulationSnapshot& OutSnapshot) const
{
    check(IsInGameThread()); OutSnapshot = {};
    if (!Selected || !PublicationSerial) return false;
    OutSnapshot = Selected->Physical.Snapshot;
    OutSnapshot.PublicationSerial = PublicationSerial;
    for (auto& Output : OutSnapshot.PresentationOutputs) Output.PublicationSerial = PublicationSerial;
    return true;
}
bool ASpeedReplaySimulation::ReadPresentationPose(uint64 StableId, FSimulationPoseConsumption& Out)
{
    check(IsInGameThread()); Out = {};
    if (!Selected || !PublicationSerial || !StableId) return false;
    const auto& Snapshot = Selected->Physical.Snapshot;
    for (const auto& Body : Snapshot.PresentationBodies) if (Body.StableId == StableId)
    {
        Out.NumFrame = Snapshot.NumFrame; Out.PublicationSerial = PublicationSerial;
        Out.StateHash = Snapshot.StateHash; Out.InputJournalHash = Snapshot.InputJournalHash;
        Out.Body = Body; return Out.IsValid();
    }
    return false;
}
bool ASpeedReplaySimulation::ReadPresentationOutput(uint64 StableId, uint32 Channel, FSimulationPresentationOutput& Out)
{
    check(IsInGameThread()); Out = {};
    if (!Selected || !PublicationSerial || !StableId || !Channel) return false;
    const auto& Snapshot = Selected->Physical.Snapshot;
    for (const auto& Output : Snapshot.PresentationOutputs)
        if (Output.OwnerStableId == StableId && Output.Channel == Channel && Output.NumFrame == Snapshot.NumFrame)
        { Out = Output; Out.PublicationSerial = PublicationSerial; return true; }
    return false;
}
