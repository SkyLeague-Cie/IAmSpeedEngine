#include "SpeedPhysicsSimulation.h"
#include "ExactRemoteRoundReset.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "Misc/ScopeExit.h"
#include "Misc/ScopeTryLock.h"

namespace
{
class FOriginalRoundRestoreLease final : public Speed::ISimulationRestoreBoundaryLease
{
public:
    FOriginalRoundRestoreLease(Speed::FSimulationWorld& InWorld, Speed::PreparedWorld::FAddress InAddress,
        std::unique_ptr<Speed::ISimulationRestoreBoundaryLease> InProtection, TFunction<bool()> InCurrent)
        : World(InWorld), Address(InAddress), Protection(std::move(InProtection)), Current(std::move(InCurrent)) {}
    ~FOriginalRoundRestoreLease() override { if (!Closed) AbortBeforeFinalize(); }

    bool HoldsExactBoundary(const Speed::FSimulationWorld& InWorld,
        const Speed::PreparedWorld::FAddress& At) const noexcept override
    {
        return !Closed && &InWorld == &World && At == Address && Current()
            && Protection->HoldsExactBoundary(InWorld, At);
    }
    bool ProtectsEscapedMutableViews() const noexcept override
    { return HoldsExactBoundary(World, Address) && Protection->ProtectsEscapedMutableViews(); }
    bool AppendPreparedAccess(std::vector<Speed::PreparedAccess::FParticipantAccess>& Out) const override
    {
        const auto Before = Out.size();
        if (!ProtectsEscapedMutableViews() || !Protection->AppendPreparedAccess(Out) || Out.size() <= Before) return false;
        const auto Span = [](const auto& Value)
        { return Speed::PreparedAccess::FSpan{reinterpret_cast<std::uintptr_t>(&Value), sizeof(Value)}; };
        Speed::PreparedAccess::FParticipantAccess Access;
        Access.Participant = reinterpret_cast<std::uintptr_t>(this);
        Access.Writes = {Span(Closed)};
        Access.ReadOnly = {Span(Address), Span(Protection), Span(Current)};
        Out.push_back(std::move(Access));
        return true;
    }
    void AbortBeforeFinalize() noexcept override
    {
        if (Closed) return;
        Protection->AbortBeforeFinalize();
        Closed = true;
    }
    void ReleaseAfterPublication() noexcept override
    {
        if (!HoldsExactBoundary(World, Address)) std::terminate();
        Protection->ReleaseAfterPublication();
        Closed = true;
    }

private:
    Speed::FSimulationWorld& World;
    const Speed::PreparedWorld::FAddress Address;
    const std::unique_ptr<Speed::ISimulationRestoreBoundaryLease> Protection;
    const TFunction<bool()> Current;
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
            return InputSessionRegistry->QueryFrameReadiness(CanonicalNumFrame, RequiredExactRemoteInputSessions)
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
        std::unique_ptr<Speed::FExactRemoteRoundPlan> Plan;
        std::optional<Speed::FExactRemoteRoundBuild::FRoster> OriginalRoster;
        {
            World.RoundBuildBoundary = Current.get();
            ON_SCOPE_EXIT { World.RoundBuildBoundary = nullptr; };
            OriginalRoster = Speed::FExactRemoteRoundBuild::CaptureRoster(World, *Current);
            if (OriginalRoster) Plan = ExactRemoteRoundProvider->Build(World, Current);
        }
        if (!Plan) return {};
        if (Plan->Envelope && Plan->Lease && Plan->Boundary == Current && Plan->Inputs.Next == CanonicalNumFrame
            && Plan->Inputs.RoundBaselines.size() == RequiredExactRemoteInputSessions
            && ReadCompletedWorldAndInputFrame() == Current)
        {
            Plan->Lease = std::make_unique<FOriginalRoundRestoreLease>(World, Plan->Envelope->Address,
                std::move(Plan->Lease), [this, Current, &World, &StorageReservation, &RegistryLock]()
                {
                    return StorageReservation->Holds(World.StorageAccess) && RegistryLock.IsLocked()
                        && SpeedWorldSubsystem->PendingOps.IsEmpty() && !SpeedWorldSubsystem->bCanonicalFrameActive
                        && !World.bOrderDirty
                        && SimulationWorker && SimulationWorker->OwnsCurrentInputBoundary()
                        && Current->Source()->WorkerGeneration() == InputWorkerGeneration
                        && Current->World().NumFrame + 1 == CanonicalNumFrame;
                });
            const auto RetainedRoster = Speed::FPreparedNativeKinematicFields::CaptureRoster(
                World, Plan->Envelope->Address, *Plan->Lease);
            if (RetainedRoster && *RetainedRoster == *OriginalRoster
                && Speed::FExactRemoteRoundBuild::MatchesBindings(*RetainedRoster, *Plan->Envelope))
                ActiveExactRemoteRound = Speed::ExactRemoteRound::Commit(
                    World, *InputSessionRegistry, ExactRemoteRoundFence, std::move(*Plan), PublishedExactRemoteRound);
            if (ActiveExactRemoteRound)
            {
                ExactRemoteRoundStorage = std::move(StorageReservation);
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

ESimulationWorkerResult ASpeedPhysicsSimulation::FinishExactRemoteRoundWork(ESimulationWorkerResult Result)
{
    if (!ExactRemoteRoundStorage || Result != ESimulationWorkerResult::Advanced) return Result;
    auto Neutral = std::move(PendingRoundNeutralStep);
    if (!ActiveExactRemoteRound || !Neutral || CanonicalNumFrame != ActiveExactRemoteRound->Inputs->Next + 1
        || !Speed::ExactRemoteRound::IsExactNeutralStep(*ActiveExactRemoteRound, *Neutral))
        return ESimulationWorkerResult::Failed;
    std::atomic_store_explicit(&PublishedRoundNeutralStep, std::move(Neutral), std::memory_order_release);
    ExactRemoteRoundStorage.reset();
    ExactRemoteRoundFence.FinishPublication();
    return Result;
}
