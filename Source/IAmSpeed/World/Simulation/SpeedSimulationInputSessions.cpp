#include "SpeedPhysicsSimulation.h"
#include "Misc/ScopeLock.h"
#include "IAmSpeed/Netcode/remote-input/ExactRemoteDedicatedHost.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"

Speed::Input::V2::ECommandAdmission ASpeedPhysicsSimulation::SubmitInputSessionCommand(
    const Speed::Input::V2::FBoundaryCommandDescriptor& Command,
    std::shared_ptr<Speed::Input::V2::FRawAcquisitionJournal> Journal,
    std::shared_ptr<Speed::Input::V2::FInputObservationChannel> Observation,
    std::shared_ptr<Speed::Input::V2::FAIInputCommands> AI,
    std::shared_ptr<Speed::Netcode::FExactRemoteInputIngress> Remote)
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
    if (Command.Operation == EBoundaryOperation::Bind && Command.Binding.Kind == EProducerContract::ExactRemote
        && (!RequiredExactRemoteInputSessions || !Remote)) return ECommandAdmission::Rejected;
    if (Journal || AI || Remote)
    {
        if (Command.Operation != EBoundaryOperation::Bind || unsigned(bool(Journal)) + unsigned(bool(AI)) + unsigned(bool(Remote)) != 1
            || (Journal && Command.Binding.Kind != EProducerContract::Device)
            || (AI && Command.Binding.Kind != EProducerContract::AI)
            || (Remote && (!RequiredExactRemoteInputSessions || Command.Binding.Kind != EProducerContract::ExactRemote))
            || !Command.Binding.Journal || PendingInputJournals.size() >= 16) return ECommandAdmission::Rejected;
        try { PendingInputJournals.push_back({Command.Binding.Journal, Journal, AI, Remote}); }
        catch (...) { return ECommandAdmission::Rejected; }
    }
    if (Observation)
    {
        if (Command.Operation != EBoundaryOperation::Bind || Observation->GetEpoch().Value != Command.Binding.Epoch
            || !Observation->GetContract() || !Command.Binding.Contract
            || Observation->GetContract()->GetFingerprint() != Command.Binding.Contract->GetFingerprint()
            || PendingInputObservations.size() >= 16)
        { if (Journal || AI || Remote) PendingInputJournals.pop_back(); return ECommandAdmission::Rejected; }
        try { PendingInputObservations.emplace_back(Command.Binding.Id, Observation); }
        catch (...) { if (Journal || AI || Remote) PendingInputJournals.pop_back(); return ECommandAdmission::Rejected; }
    }
    const auto Result = InputSessionCommands->Submit(Command);
    if (Observation && Result != ECommandAdmission::Enqueued) PendingInputObservations.pop_back();
    if ((Journal || AI || Remote) && Result != ECommandAdmission::Enqueued) PendingInputJournals.pop_back();
    if (Result == ECommandAdmission::Enqueued) { bInputCommandBoundaryDirty = true; bInputSessionAdmissionRequested.Store(true); }
    return Result;
}

bool ASpeedPhysicsSimulation::ConfigureExactRemoteInputHost(uint32 RequiredSessions)
{
    check(IsInGameThread());
    if (Speed::Netcode::QueryExactRemoteHostPolicy(GetWorld()) != Speed::Netcode::EExactRemoteHostPolicy::IsolatedDedicated
        || GetNetMode() != NM_DedicatedServer || !RequiredSessions || RequiredSessions > 16
        || RequiredExactRemoteInputSessions || SimulationWorker || InputSessionRegistry
        || bInputSessionAdmissionRequested.Load() || bInputOwnerRetired.Load() || bOwnedWorkerTerminal.Load()) return false;
    RequiredExactRemoteInputSessions = RequiredSessions;
    return true;
}

std::optional<Speed::Input::V2::FBoundaryReceipt> ASpeedPhysicsSimulation::ReadInputSessionReceipt(uint64 Id) const
{ return InputSessionCommands->Read(Id); }

std::shared_ptr<const Speed::Input::V2::FInputRegistryView> ASpeedPhysicsSimulation::ReadInputRegistryView() const
{ return PublishedInputRegistry.load(std::memory_order_seq_cst); }

std::shared_ptr<const Speed::Input::V2::FRegistryFrame> ASpeedPhysicsSimulation::ReadCompletedInputFrame() const
{ return PublishedInputFrame.load(std::memory_order_seq_cst); }

std::shared_ptr<const Speed::FCompletedSimulationPublication> ASpeedPhysicsSimulation::ReadCompletedWorldAndInputFrame() const
{
    if (!bPublishCompletedWorldInput || bCanonicalPublicationTerminal.Load() || bOwnedWorkerTerminal.Load()
        || bOwnedSimulationPaused.Load() || bInputOwnerRetired.Load()) return {};
    const auto Frame = PublishedCompletedWorldInput.load(std::memory_order_seq_cst);
    // A pause request may race an already-running frame. Never return a freshly
    // republished pointer while pause/retirement/terminal is already visible.
    if (bOwnedSimulationPaused.Load() || bInputOwnerRetired.Load()
        || bCanonicalPublicationTerminal.Load() || bOwnedWorkerTerminal.Load()) return {};
    return Frame;
}

ESimulationBoundaryResult ASpeedPhysicsSimulation::ServiceInputSessionBoundary(bool bPauseRequested)
{
    if (SpeedWorldSubsystem && !SpeedWorldSubsystem->ServiceInputRetirementsAtBoundary())
        return ESimulationBoundaryResult::Failed;
    using namespace Speed::Input::V2;
    if (bPauseRequested) PublishedCompletedWorldInput.store({}, std::memory_order_seq_cst);
    if (!bInputSessionAdmissionRequested.Load() && !RequiredExactRemoteInputSessions) return ESimulationBoundaryResult::Ready;
    // Any registry/lifecycle change invalidates a current common endpoint.
    if (!RequiredExactRemoteInputSessions) PublishedCompletedWorldInput.store({}, std::memory_order_seq_cst);
    try
    {
        if (!InputSessionRegistry)
            InputSessionRegistry = std::make_unique<FInputSessionRegistry>(InputSessionCommands, InputWorkerGeneration,
                std::vector<FInputSessionRegistry::FJournalService>{}, RequiredExactRemoteInputSessions
                    ? EExactRemoteAdmission::OriginalDeviceExact : EExactRemoteAdmission::Closed);
        bool Ready = false;
        {
            FScopeLock Lock(&InputSessionAdmissionMutex);
            bPauseRequested = bPauseRequested || bOwnedSimulationPaused.Load();
            // Retain the last immutable completed packet while only waiting for
            // future remote input. Actual command/registry changes invalidate it
            // under the same lock which excludes concurrent GT submissions.
            if (bInputCommandBoundaryDirty || bPauseRequested)
                PublishedCompletedWorldInput.store({}, std::memory_order_seq_cst);
            bInputCommandBoundaryDirty = false;
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
            Ready = InputSessionRegistry->ServiceBoundary(CanonicalNumFrame);
        }
        const auto View = InputSessionRegistry->ReadRegistry();
        PublishedInputRegistry.store(View, std::memory_order_seq_cst);
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
        if (bPauseRequested) PublishedInputFrame.store(std::shared_ptr<const FRegistryFrame>{}, std::memory_order_seq_cst);
        if (Ready && RequiredExactRemoteInputSessions && !bPauseRequested)
        {
            const auto Admission = InputSessionRegistry->QueryFrameReadiness(CanonicalNumFrame, RequiredExactRemoteInputSessions);
            if (Admission == EInputFrameReadiness::Rejected) throw std::runtime_error("exact remote cohort rejected");
            if (Admission == EInputFrameReadiness::Waiting)
            { bExactRemoteInputWaiting = true; return ESimulationBoundaryResult::Waiting; }
            if (bExactRemoteInputWaiting) { OnOwnedInputWaitEnded(); bExactRemoteInputWaiting = false; }
        }
        return Ready ? ESimulationBoundaryResult::Ready : ESimulationBoundaryResult::Waiting;
    }
    catch (...)
    {
        bOwnedWorkerTerminal.Store(true);
        bCanonicalPublicationTerminal.Store(true);
        return ESimulationBoundaryResult::Failed;
    }
}

void ASpeedPhysicsSimulation::CloseInputSessionsOnWorker()
{
    PublishedCompletedWorldInput.store({}, std::memory_order_seq_cst);
    if (SpeedWorldSubsystem && !SpeedWorldSubsystem->RetireInputProcessingOnWorker())
        bCanonicalPublicationTerminal.Store(true);
    using namespace Speed::Input::V2;
    PublishedInputFrame.store(std::shared_ptr<const FRegistryFrame>{}, std::memory_order_seq_cst);
    if (InputSessionRegistry)
    {
        const auto View = InputSessionRegistry->ReadRegistry();
        if (View && SpeedWorldSubsystem && !SpeedWorldSubsystem->NeutralizeCanonicalInputs(*View))
            bCanonicalPublicationTerminal.Store(true);
        const bool Retired = InputSessionRegistry->CloseWorkerLedger();
        PublishedInputRegistry.store(InputSessionRegistry->ReadRegistry(), std::memory_order_seq_cst);
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
