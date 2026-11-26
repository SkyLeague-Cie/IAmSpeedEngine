#include "SpeedPhysicsSimulation.h"
#include "ExactRemoteRoundReset.h"
#include "OnePausedCanonicalWork.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "Misc/ScopeExit.h"
#include "Misc/ScopeTryLock.h"

namespace Speed
{
class FExactRoundHostHold final
{
public:
    FExactRoundHostHold(std::shared_ptr<const IExactRemoteRoundHostProtection> InProtection,
        const FExactRoundNativeAddress& InNative)
        : Protection(std::move(InProtection)), Views(Protection->DirectStorageAccess()), Exclusion(Views), Native(InNative) {}

    bool ExcludesViews() const noexcept { return Exclusion.Holds(Views); }
    bool Excludes(const FWorldBorrowGate& Gate) const noexcept { return &Gate == &Views && Exclusion.Holds(Gate); }
    bool HoldsAddress(const FExactRoundNativeAddress& Address) const noexcept { return Native == Address; }
    bool HadUntrackedView() const noexcept { return Views.HasUntrackedReference(); }
    bool Protects() const noexcept
    {
        return ExcludesViews() && &Protection->DirectStorageAccess() == &Views
            && Protection->HoldsRetainedStorage(Native);
    }
    bool AppendReadOnly(std::vector<PreparedAccess::FSpan>& Out) const
    {
        const auto Before = Out.size();
        if (!Protects() || !Protection->AppendRetainedStorageReads(Out) || Out.size() <= Before) return false;
        const auto Span = [](const auto& Value)
        { return PreparedAccess::FSpan{reinterpret_cast<std::uintptr_t>(&Value), sizeof(Value)}; };
        Out.push_back(Span(Protection));
        Out.push_back(Span(Exclusion));
        Out.push_back(Span(Native));
        return true;
    }

private:
    const std::shared_ptr<const IExactRemoteRoundHostProtection> Protection;
    const FWorldBorrowGate& Views;
    const FWorldBorrowGate::FExclusion Exclusion;
    const FExactRoundNativeAddress Native;
};
}

namespace Speed::ExactNeutralDispatchDetail
{
thread_local const FFrameScope* FFrameScope::Current = nullptr;
thread_local const FSelectionScope* FSelectionScope::Current = nullptr;
thread_local FSubBodyPostScope* FSubBodyPostScope::Current = nullptr;
thread_local const FCallbackScope* FCallbackScope::Current = nullptr;
}

namespace
{
class FOriginalRoundRestoreLease final : public Speed::ISimulationRestoreBoundaryLease
{
public:
    FOriginalRoundRestoreLease(Speed::FSimulationWorld& InWorld, Speed::PreparedWorld::FAddress InAddress,
        std::shared_ptr<const Speed::FExactRoundHostHold> InProtection, TFunction<bool()> InCurrent)
        : World(InWorld), Address(InAddress), Protection(std::move(InProtection)), Current(std::move(InCurrent)) {}
    ~FOriginalRoundRestoreLease() override { if (!Closed) AbortBeforeFinalize(); }

    bool HoldsExactBoundary(const Speed::FSimulationWorld& InWorld,
        const Speed::PreparedWorld::FAddress& At) const noexcept override
    {
        return Owner == std::this_thread::get_id() && !Closed && &InWorld == &World && At == Address
            && Current() && Protection->Protects();
    }
    bool ProtectsEscapedMutableViews() const noexcept override
    { return HoldsExactBoundary(World, Address); }
    bool AppendPreparedAccess(std::vector<Speed::PreparedAccess::FParticipantAccess>& Out) const override
    {
        if (!ProtectsEscapedMutableViews()) return false;
        const auto Span = [](const auto& Value)
        { return Speed::PreparedAccess::FSpan{reinterpret_cast<std::uintptr_t>(&Value), sizeof(Value)}; };
        Speed::PreparedAccess::FParticipantAccess Access;
        Access.Participant = reinterpret_cast<std::uintptr_t>(this);
        Access.Writes = {Span(Closed)};
        Access.ReadOnly = {Span(Address), Span(Protection), Span(Current), Span(Owner)};
        if (!Protection->AppendReadOnly(Access.ReadOnly)) return false;
        Out.push_back(std::move(Access));
        return true;
    }
    void AbortBeforeFinalize() noexcept override
    {
        if (Closed) return;
        Closed = true;
    }
    void ReleaseAfterPublication() noexcept override
    {
        if (!HoldsExactBoundary(World, Address)) std::terminate();
        Closed = true;
    }

private:
    Speed::FSimulationWorld& World;
    const Speed::PreparedWorld::FAddress Address;
    const std::shared_ptr<const Speed::FExactRoundHostHold> Protection;
    const TFunction<bool()> Current;
    const std::thread::id Owner = std::this_thread::get_id();
    bool Closed = false;
};
}

bool ASpeedPhysicsSimulation::ConfigureExactRemoteRoundReset(std::shared_ptr<Speed::IExactRemoteRoundProvider> Provider)
{
    check(IsInGameThread());
    if (!Provider || ExactRemoteRoundProvider || !RequiredExactRemoteInputSessions
        || GetNetMode() != NM_DedicatedServer || SimulationWorker || InputSessionRegistry
        || (bExecutionModeInitialized && !IsOwnedWorkerExecutionMode())
        || bInputOwnerRetired.Load() || bOwnedWorkerTerminal.Load()) return false;
    auto* Subsystem = GetSpeedWorldSubsystem(GetWorld());
    if (!Subsystem || !Subsystem->BindExactRemoteRoundStorageOwner(*this, InputWorkerGeneration)) return false;
    ExactRemoteRoundProvider = std::move(Provider);
    return true;
}

bool ASpeedPhysicsSimulation::JoinExactRemoteRoundBeforeStorageTeardown(uint64 ExpectedWorkerGeneration)
{
    if (!IsInGameThread() || !ExactRemoteRoundProvider || !ExpectedWorkerGeneration
        || ExpectedWorkerGeneration != InputWorkerGeneration) return false;
    if (!InputOwnersRetiredAfterJoin(ExpectedWorkerGeneration)
        && (IsGarbageCollecting() || !JoinOwnedSimulationForInputTeardown())) return false;
    auto* Subsystem = GetSpeedWorldSubsystem(GetWorld());
    return !Subsystem || Subsystem->RecordExactRemoteRoundStorageJoin(*this, ExpectedWorkerGeneration);
}

std::shared_ptr<const Speed::FExactRemoteRoundCommit> ASpeedPhysicsSimulation::ReadExactRemoteRoundCommit() const
{
    return std::atomic_load_explicit(&PublishedExactRemoteRound, std::memory_order_acquire);
}

std::shared_ptr<const Speed::FCompletedSimulationPublication> ASpeedPhysicsSimulation::ReadExactRemoteRoundNeutralStep() const
{
    const auto Round = ReadExactRemoteRoundCommit();
    const auto Step = std::atomic_load_explicit(&PublishedRoundNeutralStep, std::memory_order_acquire);
    return Round && Step && Speed::ExactRemoteRound::IsExactNeutralStep(*Round, *Step)
        && Round == ReadExactRemoteRoundCommit() ? Step : nullptr;
}

std::optional<ESimulationBoundaryResult> ASpeedPhysicsSimulation::ServiceExactRemoteRoundBoundary(bool bPauseRequested)
{
    using EResult = ESimulationBoundaryResult;
    if (ExactRemoteRoundFence.IsClosed()) return EResult::Failed;
    if (!SimulationWorker || !SimulationWorker->OwnsCurrentInputBoundary()
        || !InputSessionRegistry || !SpeedWorldSubsystem || !bPublishCompletedWorldInput)
        return ActiveExactRemoteRound ? std::optional<EResult>{EResult::Failed} : std::nullopt;
    if (bPauseRequested || bOwnedSimulationPaused.Load() || bInputOwnerRetired.Load())
        return ActiveExactRemoteRound ? std::optional<EResult>{EResult::Failed} : std::nullopt;

    if (ActiveExactRemoteRound)
    {
        const auto& Round = *ActiveExactRemoteRound;
        if ((Round.Cancellation && Round.Cancellation->load(std::memory_order_acquire))
            || Speed::FExactRemoteRoundPlan::FClock::now() >= Round.Deadline) return EResult::Failed;
        if (CanonicalNumFrame == Round.Inputs->Next)
            return HasActiveExactRoundHostProtection()
                && InputSessionRegistry->QueryFrameReadiness(CanonicalNumFrame, RequiredExactRemoteInputSessions)
                == Speed::Input::V2::EInputFrameReadiness::Ready ? EResult::Ready : EResult::Failed;
        if (CanonicalNumFrame != Round.Inputs->Next + 1) return EResult::Failed;
        const auto Neutral = ReadExactRemoteRoundNeutralStep();
        if (!Neutral || !Speed::ExactRemoteRound::IsExactNeutralStep(Round, *Neutral)) return EResult::Failed;
        if (!ExactRemoteRoundProvider->ReleaseGameplay(Round, *Neutral)) return EResult::Waiting;
        ActiveExactRemoteRound.reset();
        return {};
    }

    const auto Current = ReadCompletedWorldAndInputFrame();
    if (!Current || !Current->Source() || Current->Source()->WorkerGeneration() != InputWorkerGeneration
        || Current->World().NumFrame + 1 != CanonicalNumFrame || !ExactRemoteRoundProvider->WantsRoundBuild(*Current)) return {};

    using ERefusal = Speed::IExactRemoteRoundProvider::EBoundaryRefusal;
    std::optional<ERefusal> Refusal;
    ON_SCOPE_EXIT { if (Refusal) ExactRemoteRoundProvider->OnBoundaryRefused(*Refusal); };
    try
    {
        auto& World = SpeedWorldSubsystem->SimulationWorld;
        auto StorageReservation = std::make_unique<Speed::FWorldBorrowGate::FReservation>(World.StorageAccess);
        if (!StorageReservation->Holds(World.StorageAccess))
        {
            Refusal = World.StorageAccess.HasUntrackedReference()
                ? ERefusal::UntrackedWorldReference : ERefusal::ActiveWorldBorrow;
            return {};
        }
        FScopeTryLock RegistryLock(&SpeedWorldSubsystem->PendingCS);
        if (!RegistryLock.IsLocked() || !SpeedWorldSubsystem->PendingOps.IsEmpty()
            || SpeedWorldSubsystem->bCanonicalFrameActive || World.bOrderDirty || World.IsRestoreBoundaryReserved())
        {
            Refusal = ERefusal::RegistryOrFrameBusy;
            return {};
        }
        std::optional<Speed::FExactRoundNativeAddress> Native;
        std::shared_ptr<const Speed::FExactRoundHostHold> HostHold;
        std::optional<Speed::FExactRemoteRoundBuild::FRoster> OriginalRoster;
        std::unique_ptr<Speed::FExactRemoteRoundPlan> Plan;
        bool bCarBumpSourceValid = false;
        const auto& ContactSettings = Current->ExactRoundContactSettings();
        if (!ContactSettings) return {};
        const Speed::FScopedExactRoundContactSettings ContactScope(&*ContactSettings);
        {
            World.RoundBuildBoundary = Current.get();
            ON_SCOPE_EXIT { World.RoundBuildBoundary = nullptr; };
            Native = Speed::FExactRemoteRoundBuild::CaptureNativeAddress(World, *Current);
            if (!Native || Native->StateVersion != SnapshotBuffer.PublishedSerial()
                || (Current->StaticRoute && !Speed::FExactRemoteRoundBuild::CaptureStaticCollisionRoute(World, *Current)))
            {
                Refusal = ERefusal::NativeAddressChanged;
                return {};
            }
            // Retention sees the address values. Native provider reads only
            // resume once the retained host views have been excluded.
            World.RoundBuildBoundary = nullptr;
            auto Protection = ExactRemoteRoundProvider->RetainHostStorage(*Native, *Current);
            if (!Protection)
            {
                Refusal = ERefusal::HostStorageUnavailable;
                return {};
            }
            HostHold = std::make_shared<const Speed::FExactRoundHostHold>(std::move(Protection), *Native);
            if (!HostHold->ExcludesViews())
            {
                Refusal = HostHold->HadUntrackedView() ? ERefusal::UntrackedHostReference : ERefusal::ActiveHostBorrow;
                return {};
            }
            if (!HostHold->Protects())
            {
                Refusal = ERefusal::HostStorageUnavailable;
                return {};
            }
            World.RoundBuildBoundary = Current.get();
            OriginalRoster = Speed::FExactRemoteRoundBuild::CaptureRoster(World, *Current);
            if (OriginalRoster) Plan = ExactRemoteRoundProvider->Build(World, Current);
            bCarBumpSourceValid = Plan && Plan->Envelope &&
                Speed::FExactRemoteRoundBuild::ValidateColdCarBumpSource(World, *Current, *Plan->Envelope);
        }
        if (!Plan) return {};
        if (bCarBumpSourceValid && Native->Matches(Plan->Envelope->Address)
            && Plan->Boundary == Current && Plan->Inputs.Next == CanonicalNumFrame
            && Plan->Inputs.RoundBaselines.size() == RequiredExactRemoteInputSessions
            && ReadCompletedWorldAndInputFrame() == Current)
        {
            auto Lease = std::make_unique<FOriginalRoundRestoreLease>(World, Plan->Envelope->Address,
                HostHold, [this, Current, Native = *Native, &World, &StorageReservation, &RegistryLock]()
                {
                    return StorageReservation->Holds(World.StorageAccess) && RegistryLock.IsLocked()
                        && SpeedWorldSubsystem->PendingOps.IsEmpty() && !SpeedWorldSubsystem->bCanonicalFrameActive
                        && !World.bOrderDirty && Native.Origin == World.ReadExactRoundOrigin()
                        && Native.StateVersion == SnapshotBuffer.PublishedSerial()
                        && SimulationWorker && SimulationWorker->OwnsCurrentInputBoundary()
                        && Current->Source()->WorkerGeneration() == InputWorkerGeneration
                        && Current->World().NumFrame + 1 == CanonicalNumFrame;
                });
            const auto RetainedRoster = Speed::FPreparedNativeKinematicFields::CaptureRoster(
                World, Plan->Envelope->Address, *Lease);
            if (RetainedRoster && *RetainedRoster == *OriginalRoster
                && Speed::FExactRemoteRoundBuild::MatchesBindings(*RetainedRoster, *Plan->Envelope))
                ActiveExactRemoteRound = Speed::ExactRemoteRound::Commit(
                    World, *InputSessionRegistry, ExactRemoteRoundFence, std::move(*Plan),
                    std::move(Lease), PublishedExactRemoteRound);
            if (ActiveExactRemoteRound)
            {
                ExactRemoteRoundStorage = std::move(StorageReservation);
                ExactRemoteRoundHostHold = std::move(HostHold);
                PublishedCompletedWorldInput.store({}, std::memory_order_seq_cst);
                PublishedInputFrame.store({}, std::memory_order_seq_cst);
                PublishedInputRegistry.store(ActiveExactRemoteRound->Inputs->Registry, std::memory_order_seq_cst);
                return EResult::Ready;
            }
        }
    }
    catch (...) {} // fallible preparation has no committed write
    ExactRemoteRoundProvider->OnRejected();
    return {};
}

bool ASpeedPhysicsSimulation::HasActiveExactRoundHostProtection() const noexcept
{
    return ExactRemoteRoundHostHold && ExactRemoteRoundHostHold->Protects();
}

const Speed::FExactRoundHostHold* ASpeedPhysicsSimulation::ReadExactNeutralDispatchHold(
    uint64 Frame, bool bResimulation) const noexcept
{
    if (!ExactRemoteRoundStorage || bResimulation || IsInGameThread() || Speed::OnePausedWork::IsActive() || !SimulationWorker
        || !SimulationWorker->OwnsCurrentCanonicalWork() || !ExactRemoteRoundFence.HoldsPublication()
        || Frame != CanonicalNumFrame || !SpeedWorldSubsystem || !SpeedWorldSubsystem->bCanonicalFrameActive
        || !ActiveExactRemoteRound || !ExactRemoteRoundHostHold) return nullptr;

    const auto& World = SpeedWorldSubsystem->SimulationWorld;
    const auto& Round = *ActiveExactRemoteRound;
    if (!ExactRemoteRoundStorage->Holds(World.StorageAccess) || !Round.Previous || !Round.Previous->Source()
        || Round.Previous->Source() != CompletedPublicationSource
        || Round.Previous->Source()->WorkerGeneration() != InputWorkerGeneration
        || !Round.Inputs || !Round.Inputs->Registry || Round.Inputs->Registry->WorkerGeneration != InputWorkerGeneration
        || !Round.Inputs->RoundGeneration || Round.Inputs->Next != Frame
        || !Round.World || !Round.World->Envelope) return nullptr;

    const auto& Before = Round.Previous->World();
    const auto& Address = Round.World->Envelope->Address;
    const auto Native = Speed::FExactRoundNativeAddress::FromCompleted(Round.Previous->ExactRoundOrigin(),
        Before.NumFrame, Before.InputJournalHash, Before.PublicationSerial);
    if (!Native || !Address.Valid() || !Native->Matches(Address) || Native->Next != Frame
        || !(Native->Origin == World.ReadExactRoundOrigin()) || Native->StateVersion != SnapshotBuffer.PublishedSerial()
        || Address.ResumeGeneration != Round.Inputs->ResumeGeneration
        || !ExactRemoteRoundHostHold->HoldsAddress(*Native) || !HasActiveExactRoundHostProtection()) return nullptr;
    return ExactRemoteRoundHostHold.get();
}

bool ASpeedPhysicsSimulation::IsCurrentExactNeutralDispatch(const Speed::FWorldBorrowGate& HostGate,
    uint64 WorkerGeneration, uint64 Frame, Speed::EExactNeutralDispatch Kind,
    const ISpeedComponent* Actor, const USSubBody* Sensor) const noexcept
{
    using namespace Speed::ExactNeutralDispatchDetail;
    const auto* Callback = FCallbackScope::Current;
    if (!Callback || !Callback->bActive || Callback->Selection != FSelectionScope::Current
        || Callback->Selection->Frame != FFrameScope::Current || IsInGameThread()) return false;

    const bool bSensor = Kind == Speed::EExactNeutralDispatch::SensorBeginOverlap
        || Kind == Speed::EExactNeutralDispatch::SensorContinuousOverlap;
    if ((bSensor && (!Sensor || Actor)) || (!bSensor && (!Actor || Sensor))) return false;
    const void* Producer = bSensor ? static_cast<const void*>(Sensor) : static_cast<const void*>(Actor);
    const auto& Selection = *Callback->Selection;
    const auto& Current = *Selection.Frame;
    if (&Current.Driver != this || Selection.Kind != Kind || Selection.Producer != Producer
        || Current.Frame != Frame || Current.Generation != WorkerGeneration) return false;
    return SimulationWorker && SimulationWorker->OwnsCurrentCanonicalWork()
        && InputWorkerGeneration == WorkerGeneration && CanonicalNumFrame == Frame
        && ExactRemoteRoundFence.HoldsPublication() && ExactRemoteRoundHostHold.get() == Current.Hold
        && Current.Hold->Excludes(HostGate);
}

bool ASpeedPhysicsSimulation::RejectCurrentExactNeutralPreGameplay(const Speed::FWorldBorrowGate& HostGate,
    uint64 WorkerGeneration, uint64 Frame, const ISpeedComponent* Actor) noexcept
{
    return RejectCurrentExactNeutralActorDispatch(HostGate, WorkerGeneration, Frame,
        Speed::EExactNeutralDispatch::ActorPreGameplay, Actor);
}

bool ASpeedPhysicsSimulation::RejectCurrentExactNeutralPostPhysics(const Speed::FWorldBorrowGate& HostGate,
    uint64 WorkerGeneration, uint64 Frame, const ISpeedComponent* Actor) noexcept
{
    return RejectCurrentExactNeutralActorDispatch(HostGate, WorkerGeneration, Frame,
        Speed::EExactNeutralDispatch::ActorPostPhysics, Actor);
}

bool ASpeedPhysicsSimulation::RejectCurrentExactNeutralActorDispatch(const Speed::FWorldBorrowGate& HostGate,
    uint64 WorkerGeneration, uint64 Frame, Speed::EExactNeutralDispatch Kind, const ISpeedComponent* Actor) noexcept
{
    if (!IsCurrentExactNeutralDispatch(HostGate, WorkerGeneration, Frame, Kind, Actor)) return false;
    if (!RejectedExactNeutralFrame) RejectedExactNeutralFrame = FExactNeutralRejection{Frame, Kind};
    return true;
}

ESimulationWorkerResult ASpeedPhysicsSimulation::FinishExactRemoteRoundWork(ESimulationWorkerResult Result)
{
    if (RejectedExactNeutralFrame)
    {
        const auto& Rejection = *RejectedExactNeutralFrame;
        const TCHAR* Trace = Rejection.Kind == Speed::EExactNeutralDispatch::ActorPreGameplay
            ? TEXT("ExactNeutralPreGameplayRejected") : TEXT("ExactNeutralPostPhysicsRejected");
        UE_LOG(LogTemp, Error, TEXT("[%s] Frame=%llu WorkerGeneration=%llu GlobalPublished=0 MutationPossible=1 Terminal=1"),
            Trace, Rejection.Frame, InputWorkerGeneration);
        ExactRemoteRoundProvider->OnRejected();
        // Preserve the original worker's terminal close and its host pins
        // through StopAndJoin, even if the enclosing pulse reports Advanced.
        return ESimulationWorkerResult::Failed;
    }
    if (!ExactRemoteRoundStorage || Result != ESimulationWorkerResult::Advanced) return Result;
    auto Neutral = std::move(PendingRoundNeutralStep);
    if (!ActiveExactRemoteRound || !Neutral || !HasActiveExactRoundHostProtection()
        || CanonicalNumFrame != ActiveExactRemoteRound->Inputs->Next + 1
        || !Speed::ExactRemoteRound::IsExactNeutralStep(*ActiveExactRemoteRound, *Neutral))
        return ESimulationWorkerResult::Failed;
    std::atomic_store_explicit(&PublishedRoundNeutralStep, std::move(Neutral), std::memory_order_release);
    ExactRemoteRoundHostHold.reset();
    ExactRemoteRoundStorage.reset();
    ExactRemoteRoundFence.FinishPublication();
    if (auto* Audit = ReadOwnedRollbackAudit())
    {
        const auto& Inputs = *ActiveExactRemoteRound->Inputs;
        Audit->RecordRound(true, Inputs.Next, Inputs.RoundGeneration);
    }
    return Result;
}
