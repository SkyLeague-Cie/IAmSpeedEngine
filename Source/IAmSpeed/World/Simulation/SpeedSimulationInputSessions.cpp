#include "SpeedSimulation.h"
#include "Misc/ScopeLock.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"

Speed::Input::V2::ECommandAdmission ASpeedSimulation::SubmitInputSessionCommand(
    const Speed::Input::V2::FBoundaryCommandDescriptor& Command,
    std::shared_ptr<Speed::Input::V2::FRawAcquisitionJournal> Journal,
    std::shared_ptr<Speed::Input::V2::FInputObservationChannel> Observation,
    std::shared_ptr<Speed::Input::V2::FAIInputCommands> AI)
{
    check(IsInGameThread());
    using namespace Speed::Input::V2;
    if (GetActiveExecutionMode() != ESimulationExecutionMode::IAmSpeedThread
        || bInputOwnerRetired.Load() || bOwnedWorkerTerminal.Load()
        || Command.WorkerGeneration != InputWorkerGeneration) return ECommandAdmission::Rejected;
    FScopeLock Lock(&InputSessionAdmissionMutex);
    if (Command.Operation == EBoundaryOperation::PauseAll || Command.Operation == EBoundaryOperation::Detach)
    {
        bOwnedSimulationPaused.Store(true);
        if (SimulationWorker) SimulationWorker->RequestPause();
    }
    if (Journal || AI)
    {
        if (Command.Operation != EBoundaryOperation::Bind || (Journal && AI)
            || (Journal && Command.Binding.Kind != EProducerContract::Device)
            || (AI && Command.Binding.Kind != EProducerContract::AI)
            || !Command.Binding.Journal || PendingInputJournals.size() >= 16) return ECommandAdmission::Rejected;
        try { PendingInputJournals.push_back({Command.Binding.Journal, Journal, AI}); }
        catch (...) { return ECommandAdmission::Rejected; }
    }
    if (Observation)
    {
        if (Command.Operation != EBoundaryOperation::Bind || Observation->GetEpoch().Value != Command.Binding.Epoch
            || !Observation->GetContract() || !Command.Binding.Contract
            || Observation->GetContract()->GetFingerprint() != Command.Binding.Contract->GetFingerprint()
            || PendingInputObservations.size() >= 16)
        { if (Journal || AI) PendingInputJournals.pop_back(); return ECommandAdmission::Rejected; }
        try { PendingInputObservations.emplace_back(Command.Binding.Id, Observation); }
        catch (...) { if (Journal || AI) PendingInputJournals.pop_back(); return ECommandAdmission::Rejected; }
    }
    const auto Result = InputSessionCommands->Submit(Command);
    if (Observation && Result != ECommandAdmission::Enqueued) PendingInputObservations.pop_back();
    if ((Journal || AI) && Result != ECommandAdmission::Enqueued) PendingInputJournals.pop_back();
    if (Result == ECommandAdmission::Enqueued) bInputSessionAdmissionRequested.Store(true);
    return Result;
}

std::optional<Speed::Input::V2::FBoundaryReceipt> ASpeedSimulation::ReadInputSessionReceipt(uint64 Id) const
{ return InputSessionCommands->Read(Id); }

std::shared_ptr<const Speed::Input::V2::FInputRegistryView> ASpeedSimulation::ReadInputRegistryView() const
{ return std::atomic_load(&PublishedInputRegistry); }

std::shared_ptr<const Speed::Input::V2::FRegistryFrame> ASpeedSimulation::ReadCompletedInputFrame() const
{ return std::atomic_load(&PublishedInputFrame); }

ESimulationBoundaryResult ASpeedSimulation::ServiceInputSessionBoundary(bool bPauseRequested)
{
    if (SpeedWorldSubsystem && !SpeedWorldSubsystem->ServiceInputRetirementsAtBoundary())
        return ESimulationBoundaryResult::Failed;
    using namespace Speed::Input::V2;
    if (!bInputSessionAdmissionRequested.Load()) return ESimulationBoundaryResult::Ready;
    try
    {
        if (!InputSessionRegistry)
            InputSessionRegistry = std::make_unique<FInputSessionRegistry>(InputSessionCommands, InputWorkerGeneration);
        bool Ready = false;
        {
            FScopeLock Lock(&InputSessionAdmissionMutex);
            bPauseRequested = bPauseRequested || bOwnedSimulationPaused.Load();
            const auto Before = InputSessionRegistry->ReadRegistry();
            // Detach must not erase the actor address before its last held values
            // and pending input effects have been neutralized on this lane.
            if (bPauseRequested && (!bInputNeutralizedDuringPause || NeutralizedInputRegistryVersion != Before->Version))
            {
                if (!SpeedWorldSubsystem || !SpeedWorldSubsystem->NeutralizeCanonicalInputs(*Before))
                    throw std::runtime_error("input boundary neutralization rejected");
                bInputNeutralizedDuringPause = true; NeutralizedInputRegistryVersion = Before->Version;
            }

            for (const auto& Journal : PendingInputJournals)
                if (!InputSessionRegistry->RegisterJournalAtBoundary(Journal)) throw std::runtime_error("input journal rejected");
            PendingInputJournals.clear();
            for (const auto& Observation : PendingInputObservations)
            {
                const auto Old = InputObservations.find(Observation.first);
                if (Old != InputObservations.end() && Old->second != Observation.second) throw std::runtime_error("input observation alias");
                InputObservations[Observation.first] = Observation.second;
            }
            PendingInputObservations.clear();
            Ready = InputSessionRegistry->ServiceBoundary();
        }
        const auto View = InputSessionRegistry->ReadRegistry();
        std::atomic_store(&PublishedInputRegistry, View);
        if (bPauseRequested && (!bInputNeutralizedDuringPause || NeutralizedInputRegistryVersion != View->Version))
        {
            if (!SpeedWorldSubsystem || !SpeedWorldSubsystem->NeutralizeCanonicalInputs(*View))
                throw std::runtime_error("input boundary neutralization rejected");
            bInputNeutralizedDuringPause = true; NeutralizedInputRegistryVersion = View->Version;
        }
        if (!bPauseRequested) bInputNeutralizedDuringPause = false;
        if (InputSessionRegistry->IsTerminal()) throw std::runtime_error("input registry terminal");
        for (auto I = InputObservations.begin(); I != InputObservations.end();)
        {
            const bool Bound = std::any_of(View->Bindings.begin(), View->Bindings.end(), [&](const auto& B)
                { return B.Id == I->first && B.Epoch == I->second->GetEpoch().Value; });
            if (!Bound) { I->second->Deactivate(); I = InputObservations.erase(I); }
            else { if (bPauseRequested) I->second->Pause(); ++I; }
        }
        if (bPauseRequested) std::atomic_store(&PublishedInputFrame, std::shared_ptr<const FRegistryFrame>{});
        return Ready ? ESimulationBoundaryResult::Ready : ESimulationBoundaryResult::Waiting;
    }
    catch (...)
    {
        bOwnedWorkerTerminal.Store(true);
        bCanonicalPublicationTerminal.Store(true);
        return ESimulationBoundaryResult::Failed;
    }
}

void ASpeedSimulation::CloseInputSessionsOnWorker()
{
    if (SpeedWorldSubsystem && !SpeedWorldSubsystem->RetireInputProcessingOnWorker())
        bCanonicalPublicationTerminal.Store(true);
    using namespace Speed::Input::V2;
    std::atomic_store(&PublishedInputFrame, std::shared_ptr<const FRegistryFrame>{});
    if (InputSessionRegistry)
    {
        const auto View = InputSessionRegistry->ReadRegistry();
        if (View && SpeedWorldSubsystem && !SpeedWorldSubsystem->NeutralizeCanonicalInputs(*View))
            bCanonicalPublicationTerminal.Store(true);
        const bool Retired = InputSessionRegistry->CloseWorkerLedger();
        std::atomic_store(&PublishedInputRegistry, InputSessionRegistry->ReadRegistry());
        InputSessionRegistry.reset();
        for (const auto& Observation : InputObservations) Observation.second->Deactivate();
        InputObservations.clear();
        bInputOwnersClosedOnWorker.Store(Retired);
        if (Retired)
        {
            FScopeLock Lock(&InputSessionAdmissionMutex);
            for (const auto& Observation : PendingInputObservations) Observation.second->Deactivate();
            PendingInputObservations.clear(); PendingInputJournals.clear();
        }
    }
}
