#include "ReplaySimulation.h"
#include "ReplayPlaybackDiagnostics32.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "SpeedPhysicsSimulation.h"

ASpeedReplaySimulation::ASpeedReplaySimulation()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = false;
    PrimaryActorTick.bTickEvenWhenPaused = true;
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
    PlaybackWallLast32=PlaybackUnpausedWall32=FPlatformTime::Seconds();
    PlaybackWorldWasPaused32=GetWorld()->IsPaused();
    if (!Elapsed.Reset(PlaybackUnpausedWall32)) { Clock.Close(); return false; }
    PublicationRetired = MakeShared<TAtomic<bool>, ESPMode::ThreadSafe>(false); // New session witness; old retired witnesses stay true.
    Diagnostics32=Speed::Replay::PlaybackDiagnostics32Enabled();
    Diagnostics32Started=Diagnostics32EOF=Diagnostics32WriteFailed=false;
    DiagnosticTicks32=DiagnosticPolls32=DiagnosticSourcePending32=DiagnosticPreparePending32=0;
    DiagnosticPublications32=DiagnosticBufferingTicks32=0;DiagnosticRenderDelta32=0;
    if(Diagnostics32){DiagnosticWallStart32=FPlatformTime::Seconds();DiagnosticAudioStart32=GetWorld()->GetAudioTimeSeconds();}
    ControlSerial = 1;
    Source = InSource; Consumer = InConsumer; StateValue = ERecordedReplayState::Buffering;
    bBuffering = true; bForwardAdvance = false;
    if (!NotifyTransportState()) return false;
    SetActorTickEnabled(true);
    const bool Accepted=Select();
    if(Accepted&&Diagnostics32)EmitPlaybackDiagnostics32(TEXT("open"));
    return Accepted; // Buffering is an accepted open, not an IO retry.
}

void ASpeedReplaySimulation::Fail()
{
    Close(); StateValue = ERecordedReplayState::Failed;
}

void ASpeedReplaySimulation::Close()
{
    check(IsInGameThread()); SetActorTickEnabled(false);
    if(Diagnostics32&&Source)EmitPlaybackDiagnostics32(TEXT("close"));
    Diagnostics32=false;
    PublicationRetired->Store(true); // Retire BEFORE source/consumer callbacks.
    if (Source) Source->Close();
    if (Consumer) Consumer->Close();
    Source.Reset(); Consumer.Reset(); Selected.Reset(); Clock.Close(); Elapsed.Close();
    PlaybackWallLast32=PlaybackUnpausedWall32=0;PlaybackWorldWasPaused32=false;
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
        if(Diagnostics32)++DiagnosticPolls32;
        auto Result = RetainedSource->Poll(Ordinal, Generation);
        // Superseded failure/buffering replies do not alter the newer control.
        if (!IsCurrent()) return false;
        // A stale async reply cannot replace the currently selected frame. Poll is
        // responsible for coalescing the desired demand rather than duplicating IO.
        if (Result.Generation != Generation || Result.Status == EReplayFrameRead::Buffering)
        { if(Diagnostics32)++DiagnosticSourcePending32; bBuffering = true; StateValue = ERecordedReplayState::Buffering; return true; }
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
    { if(Diagnostics32)++DiagnosticPreparePending32; bBuffering = true; StateValue = ERecordedReplayState::Buffering; return true; }
    const uint64 NextSerial = StartingSerial + 1;
    const bool Committed = RetainedConsumer->Commit(*Target, NextSerial, Discontinuity, Effects);
    if (!IsTargetCurrent()) return false;
    if (!Committed) { Fail(); return false; }
    PublicationSerial = NextSerial;
    if(Diagnostics32)++DiagnosticPublications32;
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
    if(Diagnostics32){++DiagnosticTicks32;DiagnosticRenderDelta32+=double(DeltaSeconds);if(bBuffering)++DiagnosticBufferingTicks32;}
    // FApp/world deltas can be synthetic under fixed timestep/frame rate.
    // Real monotonic wall time alone drives playback. This actor still ticks
    // at ordinary Unreal cadence; it never runs a physical solver here.
    const double WallNow=FPlatformTime::Seconds();auto* World=GetWorld();
    if(!World||!std::isfinite(WallNow)||WallNow<PlaybackWallLast32){Fail();return;}
    const bool WorldPaused=World->IsPaused();
    // Tick even while paused keeps the wall anchor fresh. Exclude the whole
    // observed pause transition interval, once, rather than replaying it on
    // resume. Keep debt earned BEFORE pause and its exact pending target.
    if(!WorldPaused&&!PlaybackWorldWasPaused32)PlaybackUnpausedWall32+=WallNow-PlaybackWallLast32;
    PlaybackWallLast32=WallNow;PlaybackWorldWasPaused32=WorldPaused;
    if(!Elapsed.Accumulate(PlaybackUnpausedWall32,!Clock.IsPaused())){Fail();return;}
    if(WorldPaused)return; // No source/consumer pose publication during world pause.
    if (!bBuffering)
    {
        const bool WasPaused = Clock.IsPaused();
        const bool Forward = !WasPaused && Clock.Speed() > 0;
        const uint64 Before = Clock.Index();
        if (!Clock.Advance(Elapsed.Take())) { Fail(); return; }
        bForwardAdvance = Forward && Clock.Index() > Before;
        if (WasPaused != Clock.IsPaused() && !NotifyTransportState()) return;
    }
    Select();
    if(Diagnostics32&&Diagnostics32Started&&!Diagnostics32EOF&&Source&&Selected&&!bBuffering&&Clock.IsPaused()&&
       Selected->Ordinal==Clock.Index()&&Clock.Fraction()>=1.&&((Clock.Speed()>0&&Clock.Index()+1==Clock.NumFrames())||(Clock.Speed()<0&&Clock.Index()==0)))
    {Diagnostics32EOF=true;EmitPlaybackDiagnostics32(TEXT("eof_committed"));}
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
    if (!GetWorld()) { Fail(); return false; }
    PlaybackWallLast32=PlaybackUnpausedWall32=FPlatformTime::Seconds();
    PlaybackWorldWasPaused32=GetWorld()->IsPaused();
    if (!Elapsed.Reset(PlaybackUnpausedWall32)) { Fail(); return false; }
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
    const bool Accepted=Select();
    if(Accepted&&Diagnostics32&&!Diagnostics32Started){Diagnostics32Started=true;EmitPlaybackDiagnostics32(TEXT("start"));}
    return Accepted;
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

void ASpeedReplaySimulation::EmitPlaybackDiagnostics32(const TCHAR* Phase)
{
    if(!Diagnostics32)return;
    const double Wall=FPlatformTime::Seconds(),Audio=GetWorld()?GetWorld()->GetAudioTimeSeconds():DiagnosticAudioStart32;
    const FString Line=FString::Printf(TEXT("{\"domain\":\"driver\",\"phase\":\"%s\",\"wall\":%.9f,\"wallElapsed\":%.9f,\"audio\":%.9f,\"audioElapsed\":%.9f,\"renderDeltaSum\":%.9f,\"pendingSeconds\":%.9f,\"ticks\":%llu,\"bufferingTicks\":%llu,\"targetPolls\":%llu,\"sourcePending\":%llu,\"preparePending\":%llu,\"publications\":%llu,\"sourceOrdinal\":%llu,\"committedOrdinal\":%llu,\"generation\":%llu,\"control\":%llu,\"rate\":%.6f,\"state\":%d,\"writeFailedBefore\":%s}"),
        Phase,Wall,Wall-DiagnosticWallStart32,Audio,Audio-DiagnosticAudioStart32,DiagnosticRenderDelta32,Elapsed.PendingSeconds(),
        static_cast<unsigned long long>(DiagnosticTicks32),static_cast<unsigned long long>(DiagnosticBufferingTicks32),
        static_cast<unsigned long long>(DiagnosticPolls32),static_cast<unsigned long long>(DiagnosticSourcePending32),
        static_cast<unsigned long long>(DiagnosticPreparePending32),static_cast<unsigned long long>(DiagnosticPublications32),
        static_cast<unsigned long long>(Clock.Index()),static_cast<unsigned long long>(Selected?Selected->Ordinal:MAX_uint64),
        static_cast<unsigned long long>(Clock.Epoch()),static_cast<unsigned long long>(ControlSerial),Clock.Speed(),int32(StateValue),
        Diagnostics32WriteFailed?TEXT("true"):TEXT("false"));
    if(!Speed::Replay::AppendPlaybackDiagnostics32(Line))Diagnostics32WriteFailed=true;
}
FString ASpeedReplaySimulation::PlaybackDiagnosticStatus32()const
{
    if(!Diagnostics32)return FString();
    return FString::Printf(TEXT("ticks=%llu polls=%llu sourcePending=%llu preparePending=%llu publications=%llu pending=%.6f writeFailed=%d"),
        static_cast<unsigned long long>(DiagnosticTicks32),static_cast<unsigned long long>(DiagnosticPolls32),
        static_cast<unsigned long long>(DiagnosticSourcePending32),static_cast<unsigned long long>(DiagnosticPreparePending32),
        static_cast<unsigned long long>(DiagnosticPublications32),Elapsed.PendingSeconds(),Diagnostics32WriteFailed);
}
