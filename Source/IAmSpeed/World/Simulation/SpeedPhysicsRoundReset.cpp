#include "SpeedPhysicsSimulation.h"
#include "ExactRemoteRoundReset.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"

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
        if (!ProtectsEscapedMutableViews() || !Protection->AppendPreparedAccess(Out)) return false;
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
        || bInputOwnerRetired.Load() || bOwnedWorkerTerminal.Load()) return false;
    ExactRemoteRoundProvider = std::move(Provider);
    return true;
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
        const auto Neutral = ReadCompletedWorldAndInputFrame();
        if (!Neutral || !Speed::ExactRemoteRound::IsExactNeutralStep(Round, *Neutral)) return EResult::Failed;
        std::atomic_store_explicit(&PublishedRoundNeutralStep, Neutral, std::memory_order_release);
        if (!ExactRemoteRoundProvider->ReleaseGameplay(Round, *Neutral)) return EResult::Waiting;
        ActiveExactRemoteRound.reset();
        return {};
    }

    const auto Current = ReadCompletedWorldAndInputFrame();
    if (!Current || !Current->Source() || Current->Source()->WorkerGeneration() != InputWorkerGeneration
        || Current->World().NumFrame + 1 != CanonicalNumFrame) return {};
    try
    {
        auto& World = SpeedWorldSubsystem->SimulationWorld;
        auto Plan = ExactRemoteRoundProvider->Build(World, Current);
        if (!Plan) return {};
        if (Plan->Envelope && Plan->Lease && Plan->Boundary == Current && Plan->Inputs.Next == CanonicalNumFrame
            && Plan->Inputs.RoundBaselines.size() == RequiredExactRemoteInputSessions
            && ReadCompletedWorldAndInputFrame() == Current)
        {
            Plan->Lease = std::make_unique<FOriginalRoundRestoreLease>(World, Plan->Envelope->Address,
                std::move(Plan->Lease), [this, Current]()
                {
                    return SimulationWorker && SimulationWorker->OwnsCurrentInputBoundary()
                        && Current->Source()->WorkerGeneration() == InputWorkerGeneration
                        && Current->World().NumFrame + 1 == CanonicalNumFrame;
                });
            ActiveExactRemoteRound = Speed::ExactRemoteRound::Commit(
                World, *InputSessionRegistry, std::move(*Plan), PublishedExactRemoteRound);
            if (ActiveExactRemoteRound)
            {
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
