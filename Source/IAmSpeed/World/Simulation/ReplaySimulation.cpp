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
    ControlSerial = 1;
    Source = InSource; Consumer = InConsumer; StateValue = ERecordedReplayState::Buffering;
    bBuffering = true; bForwardAdvance = false;
    if (!NotifyTransportState()) return false;
    SetActorTickEnabled(true);
    return Select(); // Buffering is an accepted open, not an IO retry.
}

void ASpeedReplaySimulation::Fail()
{
    Close(); StateValue = ERecordedReplayState::Failed;
}

void ASpeedReplaySimulation::Close()
{
    check(IsInGameThread()); SetActorTickEnabled(false);
    PublicationRetired->Store(true); // Retire BEFORE source/consumer callbacks.
    if (Source) Source->Close();
    if (Consumer) Consumer->Close();
    Source.Reset(); Consumer.Reset(); Selected.Reset(); Clock.Close();
    PreparingTarget.Reset(); PreparingTargetGeneration = 0; ControlSerial = 0;
    PublishedGeneration = 0; bBuffering = false; bForwardAdvance = false; StateValue = ERecordedReplayState::Closed;
    PublicationRetired->Store(true);
    // No OS join is inferred here. The module's independent retained IO owner
    // still owns the actual cancel/drain/join across map/actor destruction.
}

bool ASpeedReplaySimulation::Select()
{
    if (!Source || !Consumer) return false;
    // Callbacks may synchronously close or control this driver. Keep their
    // objects alive and retire this invocation before acting on an old reply.
    const auto RetainedSource = Source;
    const auto RetainedConsumer = Consumer;
    const auto Session = GetPresentationRetirementWitness();
    const auto Previous = Selected;
    const uint64 Ordinal = Clock.Index(), Generation = Clock.Epoch();
    const uint64 Control = ControlSerial, StartingSerial = PublicationSerial;
    const auto IsCurrent = [&]()
    {
        return Source == RetainedSource && Consumer == RetainedConsumer &&
            GetPresentationRetirementWitness().Get() == Session.Get() && Session && !Session->Load() &&
            ControlSerial == Control && Clock.Epoch() == Generation && Clock.Index() == Ordinal &&
            PublicationSerial == StartingSerial && Selected == Previous;
    };
    if (!IsCurrent()) return false;
    if (Previous && Previous->Ordinal == Ordinal && PublishedGeneration == Generation)
    {
        PreparingTarget.Reset(); PreparingTargetGeneration = 0;
        bBuffering = false; bForwardAdvance = false;
        StateValue = Clock.IsPaused() ? ERecordedReplayState::Paused : ERecordedReplayState::Playing;
        return true;
    }
    if (!PreparingTarget || PreparingTarget->Ordinal != Ordinal || PreparingTargetGeneration != Generation)
    {
        PreparingTarget.Reset(); PreparingTargetGeneration = 0;
        auto Result = RetainedSource->Poll(Ordinal, Generation);
        // Superseded failure/buffering replies do not alter the newer control.
        if (!IsCurrent()) return false;
        // A stale async reply cannot replace the currently selected frame. Poll is
        // responsible for coalescing the desired demand rather than duplicating IO.
        if (Result.Generation != Generation || Result.Status == EReplayFrameRead::Buffering)
        { bBuffering = true; StateValue = ERecordedReplayState::Buffering; return true; }
        const uint64 First = RetainedSource->FirstPhysicalFrame();
        if (!IsCurrent()) return false;
        if (Result.Status != EReplayFrameRead::Ready || !Result.Frame ||
            Result.Frame->Ordinal != Ordinal || First > MAX_uint64 - Ordinal ||
            Result.Frame->Physical.Snapshot.NumFrame != First + Ordinal ||
            PublicationSerial == MAX_uint64)
        { Fail(); return false; }
        PreparingTarget = MoveTemp(Result.Frame); PreparingTargetGeneration = Generation;
    }
    // Strong ownership also keeps the Frame reference valid when a callback
    // retires PreparingTarget. Never read a replacement target after callback.
    const auto Target = PreparingTarget;
    const auto IsTargetCurrent = [&]()
    {
        return IsCurrent() && PreparingTarget == Target && PreparingTargetGeneration == Generation;
    };
    const uint64 CurrentCount = RetainedSource->NumFrames();
    if (!IsTargetCurrent()) return false;
    const uint64 CurrentFirst = RetainedSource->FirstPhysicalFrame();
    if (!IsTargetCurrent()) return false;
    if (!Target || !CurrentCount || Ordinal >= CurrentCount || CurrentFirst > MAX_uint64 - Ordinal ||
        Target->Physical.Snapshot.NumFrame != CurrentFirst + Ordinal || PublicationSerial == MAX_uint64)
    { Fail(); return false; }
    // Ordinary 300Hz playback can cross several physical frames per rendered
    // frame. That is a forward interval, not a seek. Consumers reconstruct
    // continuous effects from the recorded observer and must use recorded event
    // ranges for one-shots; they must not invoke live gameplay callbacks.
    const bool Discontinuity = !Previous || PublishedGeneration != Generation ||
        Ordinal < Previous->Ordinal;
    // Advancing directly to EOF may pause the clock before this target commits.
    // That interval still contains recorded events; an explicit pause/seek/rate
    // change clears the pending permission instead of replaying a delayed burst.
    const bool Effects = !Discontinuity && bForwardAdvance && Clock.Speed() > 0;
    FReplayConsumerContext Context;
    Context.Source = RetainedSource; Context.Previous = Previous; Context.Session = Session;
    Context.Generation = Generation; Context.ControlSerial = Control;
    Context.bDiscontinuity = Discontinuity; Context.bEmitForwardEffects = Effects;
    Context.bPaused = Clock.IsPaused(); Context.Rate = Clock.Speed();
    const auto Prepared = RetainedConsumer->PrepareWithContext(*Target, Context);
    if (!IsTargetCurrent()) return false;
    if (Prepared == EReplayFramePreparation::Failed) { Fail(); return false; }
    if (Prepared == EReplayFramePreparation::Pending)
    { bBuffering = true; StateValue = ERecordedReplayState::Buffering; return true; }
    const uint64 NextSerial = StartingSerial + 1;
    const bool Committed = RetainedConsumer->Commit(*Target, NextSerial, Discontinuity, Effects);
    if (!IsTargetCurrent()) return false;
    if (!Committed) { Fail(); return false; }
    PublicationSerial = NextSerial;
    Selected = Target; PreparingTarget.Reset(); PreparingTargetGeneration = 0;
    PublishedGeneration = Generation; bBuffering = false;
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
        const bool WasPaused = Clock.IsPaused();
        const bool Forward = !WasPaused && Clock.Speed() > 0;
        const uint64 Before = Clock.Index();
        if (!Clock.Advance(double(DeltaSeconds))) { Fail(); return; }
        bForwardAdvance = Forward && Clock.Index() > Before;
        if (WasPaused != Clock.IsPaused() && !NotifyTransportState()) return;
    }
    Select();
}

bool ASpeedReplaySimulation::NotifyTransportState(bool RetireContinuous)
{
    if (!Source || !Consumer) return false;
    const auto RetainedSource = Source;
    const auto RetainedConsumer = Consumer;
    const auto Session = GetPresentationRetirementWitness();
    if (!Session || Session->Load()) return false;
    const uint64 Control = ControlSerial, Generation = Clock.Epoch();
    const uint64 Ordinal = Clock.Index(), StartingSerial = PublicationSerial;
    FReplayConsumerContext Context;
    Context.Source = RetainedSource; Context.Previous = Selected; Context.Session = Session;
    Context.Generation = Clock.Epoch(); Context.ControlSerial = ControlSerial;
    Context.bPaused = Clock.IsPaused(); Context.Rate = Clock.Speed();
    // Seek retires active continuous presentation before any source IO
    // can buffer. Pause/rate/automatic EOF keep their separate policy.
    Context.bDiscontinuity = RetireContinuous;
    // No pose or event publication occurs in this callback. A forward EOF
    // interval retains bForwardAdvance until its ordinary Select/Commit.
    const bool Accepted = RetainedConsumer->TransportChanged(Context);
    if (Source != RetainedSource || Consumer != RetainedConsumer ||
        GetPresentationRetirementWitness().Get() != Session.Get() || Session->Load() ||
        ControlSerial != Control || Clock.Epoch() != Generation || Clock.Index() != Ordinal ||
        PublicationSerial != StartingSerial) return false;
    if (!Accepted) { Fail(); return false; }
    return true;
}

bool ASpeedReplaySimulation::InvalidateExplicitControl(bool RetireContinuous)
{
    if (!Source || !Consumer) return false;
    if (ControlSerial == MAX_uint64) { Fail(); return false; }
    ++ControlSerial;
    const auto RetainedSource = Source;
    const auto RetainedConsumer = Consumer;
    const auto Session = GetPresentationRetirementWitness();
    const uint64 Control = ControlSerial, Generation = Clock.Epoch();
    const uint64 Ordinal = Clock.Index(), StartingSerial = PublicationSerial;
    RetainedConsumer->InvalidatePreparation(Control);
    if (Source != RetainedSource || Consumer != RetainedConsumer || !Session || Session->Load() ||
        GetPresentationRetirementWitness().Get() != Session.Get() || ControlSerial != Control ||
        Clock.Epoch() != Generation || Clock.Index() != Ordinal || PublicationSerial != StartingSerial) return false;
    PreparingTarget.Reset(); PreparingTargetGeneration = 0;
    bForwardAdvance = false;
    return NotifyTransportState(RetireContinuous);
}

bool ASpeedReplaySimulation::PlayReplay()
{
    check(IsInGameThread()); if (!Source || !Clock.Play() || !InvalidateExplicitControl()) return false;
    return Select();
}
void ASpeedReplaySimulation::PauseReplay()
{
    check(IsInGameThread());
    if (!Source) return;
    Clock.Pause();
    if (InvalidateExplicitControl()) Select();
}
bool ASpeedReplaySimulation::SetReplayRate(double Rate)
{
    check(IsInGameThread());
    if (!Source || !Clock.SetRate(Rate)) return false;
    return InvalidateExplicitControl();
}
bool ASpeedReplaySimulation::SeekReplay(int64 Ordinal)
{
    check(IsInGameThread()); if (!Source || !Clock.Seek(Ordinal) || !InvalidateExplicitControl(true)) return false;
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
