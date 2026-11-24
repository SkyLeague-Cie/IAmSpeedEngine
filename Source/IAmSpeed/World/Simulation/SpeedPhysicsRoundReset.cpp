#include "SpeedPhysicsSimulation.h"
#include "ExactRemoteRoundReset.h"
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

ESimulationWorkerResult ASpeedPhysicsSimulation::FinishExactRemoteRoundWork(ESimulationWorkerResult Result)
{
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
    return Result;
}
