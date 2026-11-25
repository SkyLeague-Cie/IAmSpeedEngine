#include "SpeedPhysicsSimulation.h"
#include "CanonicalFrameContext.h"
#include "Misc/ScopeExit.h"
#include "Misc/ScopeLock.h"
#include "IAmSpeed/Netcode/remote-input/ExactRemoteDedicatedHost.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"

namespace
{
bool NeutralizeLegacyBindings(USpeedWorldSubsystem* World,const Speed::Input::V2::FInputRegistryView& View)
{
    auto Legacy=View;
    Legacy.Bindings.erase(std::remove_if(Legacy.Bindings.begin(),Legacy.Bindings.end(),
        [](const auto& B){return B.Network&&B.Network->Version==2;}),Legacy.Bindings.end());
    return World&&World->NeutralizeCanonicalInputs(Legacy);
}
}

Speed::Input::V2::ECommandAdmission ASpeedPhysicsSimulation::SubmitInputSessionCommand(
    const Speed::Input::V2::FBoundaryCommandDescriptor& Command,
    std::shared_ptr<Speed::Input::V2::FRawAcquisitionJournal> Journal,
    std::shared_ptr<Speed::Input::V2::FInputObservationChannel> Observation,
    std::shared_ptr<Speed::Input::V2::FAIInputCommands> AI,
    std::shared_ptr<Speed::Netcode::FExactRemoteInputIngress> Remote)
{
    check(IsInGameThread());
    using namespace Speed::Input::V2;
    if(Command.Binding.Network||Command.NativeV2Control)return ECommandAdmission::Rejected;
    if (ReplayStartupService.IsHolding() && Command.Operation == EBoundaryOperation::Bind
        && Command.Binding.Kind != EProducerContract::ExactRemote) return ECommandAdmission::Rejected;
    if (GetActiveExecutionMode() != ESimulationExecutionMode::IAmSpeedThread
        || bInputOwnerRetired.Load() || bOwnedWorkerTerminal.Load()
        || Command.WorkerGeneration != InputWorkerGeneration) return ECommandAdmission::Rejected;
    FScopeLock Lock(&InputSessionAdmissionMutex);
    if(Command.Operation==EBoundaryOperation::Detach)
    {
        // Reject before the legacy pause request, including a V2 Bind still
        // queued or awaiting its native claim. A stale generic control must
        // never pause an independent requested stream.
        std::lock_guard<std::mutex> Records(InputSessionCommands->Gate);
        for(const auto& P:InputSessionCommands->Records)
        {
            const auto& B=P.second.Command;
            if(B.Operation==EBoundaryOperation::Bind&&B.Binding.Id==Command.Session
                &&B.Binding.Network&&B.Binding.Network->Version==2)return ECommandAdmission::Rejected;
        }
    }
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
    return ConfigureOriginalDeviceRemoteInputHost(RequiredSessions, Speed::Input::V2::EExactRemoteAdmission::OriginalDeviceExact);
}

bool ASpeedPhysicsSimulation::ConfigureOriginalDeviceRemoteInputHost(uint32 RequiredSessions, Speed::Input::V2::EExactRemoteAdmission Admission)
{
    check(IsInGameThread());
    if (!Speed::Input::V2::AdmitsOriginalDeviceRemote(Admission)) return false;
    if (Speed::Netcode::QueryExactRemoteHostPolicy(GetWorld()) != Speed::Netcode::EExactRemoteHostPolicy::IsolatedDedicated
        || GetNetMode() != NM_DedicatedServer || !RequiredSessions || RequiredSessions > 16
        || RequiredExactRemoteInputSessions || SimulationWorker || InputSessionRegistry
        || bInputSessionAdmissionRequested.Load() || bInputOwnerRetired.Load() || bOwnedWorkerTerminal.Load()) return false;
    ConfiguredRemoteInputAdmission = Admission;
    RequiredExactRemoteInputSessions = RequiredSessions;
    MaybeArmReplayStartupServiceOnGT();
    return true;
}

Speed::Input::V2::ECommandAdmission ASpeedPhysicsSimulation::SubmitRequestedNetworkInputSession(
    const Speed::Input::V2::FBoundaryCommandDescriptor& Command,
    std::shared_ptr<Speed::Netcode::FNetworkRequestedInputService> Service)
{
    check(IsInGameThread());
    using namespace Speed::Input::V2;
    if(ReplayStartupService.IsHolding()||!Service||!Command.Binding.Network||Command.Binding.Network->Version!=1||Command.Operation!=EBoundaryOperation::Bind
        ||GetActiveExecutionMode()!=ESimulationExecutionMode::IAmSpeedThread
        ||!bPublishCompletedWorldInput||!CompletedPublicationSource||!NativeInputAdmissionOwner
        ||bInputOwnerRetired.Load()||bOwnedWorkerTerminal.Load()||bCanonicalPublicationTerminal.Load()
        ||Command.WorkerGeneration!=InputWorkerGeneration)return ECommandAdmission::Rejected;
    FScopeLock Lock(&InputSessionAdmissionMutex);
    if(PendingRequestedNetworkSessions.size()>=16)return ECommandAdmission::Rejected;
    try{PendingRequestedNetworkSessions.push_back({Command,std::move(Service)});}
    catch(...){return ECommandAdmission::Rejected;}
    const auto Result=InputSessionCommands->Submit(Command);
    if(Result!=ECommandAdmission::Enqueued)PendingRequestedNetworkSessions.pop_back();
    else bInputSessionAdmissionRequested.Store(true);
    return Result;
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

std::shared_ptr<const Speed::FNativeRequestedInputObservation> ASpeedPhysicsSimulation::ReadNativeRequestedInputObservation() const
{
    if(bInputOwnerRetired.Load()||bOwnedWorkerTerminal.Load()||bCanonicalPublicationTerminal.Load())return {};
    return PublishedRequestedInputObservation.load(std::memory_order_seq_cst);
}

ESimulationBoundaryResult ASpeedPhysicsSimulation::ServiceInputSessionBoundary(bool bPauseRequested)
{
    if (ReplayStartupService.IsCancelled() || !ValidateAnalyticNumericOwner()) return ESimulationBoundaryResult::Failed;
    const bool bReplayStartupService = !ReplayStartupService.AllowsWork();
    if (!bReplayStartupService && ExactRemoteRoundProvider)
        if (const auto Round = ServiceExactRemoteRoundBoundary(bPauseRequested)) return *Round;
    std::optional<Speed::FWorldBorrowGate::FBorrow> WorldBorrow;
    if (SpeedWorldSubsystem)
    {
        WorldBorrow.emplace(SpeedWorldSubsystem->SimulationWorld.StorageAccess);
        if (!*WorldBorrow) return ESimulationBoundaryResult::Failed;
    }
    if (!bReplayStartupService && CompleteOwnedWorkerAtBoundary())
    {
        bOwnedWorkerTerminal.Store(true);
        PublishedCompletedWorldInput.store({}, std::memory_order_seq_cst);
        return ESimulationBoundaryResult::Complete;
    }
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
                    ? ConfiguredRemoteInputAdmission : EExactRemoteAdmission::Closed,NativeInputAdmissionOwner);
        // Complete-context comparison is independent of the V1 bind queue.
        // It cannot issue FNativeNetworkAdmission or construct a source.
        uint64 NativeCutoff=0;
        {
            FScopeLock Lock(&InputSessionAdmissionMutex);std::lock_guard<std::mutex> Records(InputSessionCommands->Gate);
            const auto& Queue=InputSessionCommands->Queue;
            const bool Native=InputSessionRegistry->HasNativeV2Sessions()||std::any_of(Queue.begin(),Queue.end(),[&](auto Id)
            {const auto& C=InputSessionCommands->Records.at(Id).Command;return C.NativeV2Control||(C.Binding.Network&&C.Binding.Network->Version==2);});
            NativeCutoff=Native&&Queue.size()>32?InputSessionCommands->Records.at(Queue[31]).EnqueueOrdinal:InputSessionCommands->EnqueueOrdinal;
        }
        const auto NeutralizeV2=[this](const auto& D,const auto& B){return SpeedWorldSubsystem->RetireRequestedNetworkBindingV2(D,B);};
        if (!bReplayStartupService)
        {
            if(!InputSessionRegistry->ServiceNativeV2Retirements(NeutralizeV2))throw std::runtime_error("native V2 retirement failed");
            ServiceRequestedNetworkPreparationV2AtBoundary();
            ServiceRequestedNetworkBindsV2AtBoundary(NativeCutoff);
        }
        bool Ready = false;
        {FScopeLock Lock(&InputSessionAdmissionMutex);bPauseRequested=bPauseRequested||bOwnedSimulationPaused.Load();}
        const auto Before=InputSessionRegistry->ReadRegistry();
        if(bPauseRequested&&(!bInputNeutralizedDuringPause||NeutralizedInputRegistryVersion!=Before->Version))
        {
            if(!NeutralizeLegacyBindings(SpeedWorldSubsystem,*Before))throw std::runtime_error("input boundary neutralization rejected");
            bInputNeutralizedDuringPause=true;NeutralizedInputRegistryVersion=Before->Version;
        }
        {
            FScopeLock Lock(&InputSessionAdmissionMutex);
            bPauseRequested = bPauseRequested || bOwnedSimulationPaused.Load();
            // Retain the last immutable completed packet while only waiting for
            // future remote input. Actual command/registry changes invalidate it
            // under the same lock which excludes concurrent GT submissions.
            if (bInputCommandBoundaryDirty || bPauseRequested)
                PublishedCompletedWorldInput.store({}, std::memory_order_seq_cst);
            bInputCommandBoundaryDirty = false;

            for (const auto& Journal : PendingInputJournals)
                if (!InputSessionRegistry->RegisterJournalAtBoundary(Journal)) throw std::runtime_error("input journal rejected");
            PendingInputJournals.clear();
            if(!bReplayStartupService&&!PendingRequestedNetworkSessions.empty())
            {
                // This capability is issued only in Run->ServiceBoundary of
                // the actual owned worker, after native world/body validation.
                if(!SimulationWorker||!SimulationWorker->OwnsCurrentInputBoundary()
                    ||!NativeInputAdmissionOwner||!SpeedWorldSubsystem||SpeedWorldSubsystem->GetWorld()!=GetWorld())
                    throw std::runtime_error("native requested input boundary absent");
                if(!SpeedWorldSubsystem->PrepareRequestedNetworkAdmissionBoundary())
                    throw std::runtime_error("native requested input registry unavailable");
                std::vector<FPendingRequestedNetworkSession> Later;
                for(const auto& P:PendingRequestedNetworkSessions)
                {
                    {std::lock_guard<std::mutex> Records(InputSessionCommands->Gate);const auto I=InputSessionCommands->Records.find(P.Command.Id);
                        if(I!=InputSessionCommands->Records.end()&&I->second.EnqueueOrdinal>NativeCutoff){Later.push_back(P);continue;}}
                    if(P.Command.WorkerGeneration!=InputWorkerGeneration||!P.Command.Binding.Network
                        ||P.Command.Binding.Network->WorldGeneration!=InputWorkerGeneration
                        ||!SpeedWorldSubsystem->ValidateRequestedNetworkBinding(P.Command.Binding))continue;
                    std::shared_ptr<const FInputSessionRegistry::FNativeNetworkAdmission> Admission(
                        new FInputSessionRegistry::FNativeNetworkAdmission(NativeInputAdmissionOwner,P.Command,P.Service));
                    // Failure leaves no mutable source. The command is rejected
                    // by Bind when its required native admission is absent.
                    InputSessionRegistry->RegisterNetworkAdmissionAtBoundary(std::move(Admission));
                }
                PendingRequestedNetworkSessions=std::move(Later);
            }
            for (const auto& Observation : PendingInputObservations)
            {
                const auto Old = InputObservations.find(Observation.first);
                if (Old != InputObservations.end() && Old->second != Observation.second) throw std::runtime_error("input observation alias");
                InputObservations[Observation.first] = Observation.second;
            }
            PendingInputObservations.clear();
            Ready = InputSessionRegistry->ServiceBoundary(CanonicalNumFrame,NativeCutoff);
        }
        if(!bReplayStartupService&&!InputSessionRegistry->ServiceNativeV2Claims([this](const auto& D,const auto& B){return SpeedWorldSubsystem->ClaimRequestedNetworkBindingV2(D,B);},NeutralizeV2))
            throw std::runtime_error("native V2 claim/retirement failed");
        const auto View = InputSessionRegistry->ReadRegistry();
        PublishedInputRegistry.store(View, std::memory_order_seq_cst);
        if (bPauseRequested && (!bInputNeutralizedDuringPause || NeutralizedInputRegistryVersion != View->Version))
        {
            if (!NeutralizeLegacyBindings(SpeedWorldSubsystem,*View))
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
        if (bReplayStartupService)
            return Ready ? ESimulationBoundaryResult::Ready : ESimulationBoundaryResult::Waiting;
        if (Ready && RequiredExactRemoteInputSessions && !bPauseRequested)
        {
            const auto Admission = InputSessionRegistry->QueryFrameReadiness(CanonicalNumFrame, RequiredExactRemoteInputSessions);
            if (Admission == EInputFrameReadiness::Rejected) throw std::runtime_error("exact remote cohort rejected");
            if (Admission == EInputFrameReadiness::Waiting)
            { bExactRemoteInputWaiting = true; return ESimulationBoundaryResult::Waiting; }
            if (bExactRemoteInputWaiting) { OnOwnedInputWaitEnded(); bExactRemoteInputWaiting = false; }
        }
        // This observation is made only after the actual owned boundary has
        // admitted the service and native mapping. It never polls/chooses input.
        if(Ready&&InputSessionRegistry->CanObserveRequestedBindingsAtBoundary())
        {
            if(!SimulationWorker||!SimulationWorker->OwnsCurrentInputBoundary()
                ||!CompletedPublicationSource||CompletedPublicationSource->WorkerGeneration()!=InputWorkerGeneration
                ||!SpeedWorldSubsystem||!ValidateRequestedBindingsAtBoundary())
                PublishedRequestedInputObservation.store({},std::memory_order_seq_cst);
            else
            {
                // ReadRegistry is a phase projection. Retain the exact immutable
                // internal view that future completed FRegistryFrames will use.
                const auto ExactRegistry=InputSessionRegistry->View;
                const auto Old=PublishedRequestedInputObservation.load(std::memory_order_seq_cst);
                if(!Old||Old->Source()!=CompletedPublicationSource||Old->RegistryAtBoundary()!=ExactRegistry)
                {
                    const auto Context=MakeCanonicalFrameContext(CanonicalNumFrame);
                    std::shared_ptr<const Speed::FNativeRequestedInputObservation> Captured(
                        new Speed::FNativeRequestedInputObservation(CompletedPublicationSource,ExactRegistry,
                            InputSessionRegistry->CaptureRequestedBindingsAtBoundary(),CanonicalNumFrame,Context.PhysicalDeltaTime));
                    PublishedRequestedInputObservation.store(std::move(Captured),std::memory_order_seq_cst);
                }
            }
        }
        else PublishedRequestedInputObservation.store({},std::memory_order_seq_cst);
        ServiceRequestedInputPresentationsAtBoundary();
        return Ready ? ESimulationBoundaryResult::Ready : ESimulationBoundaryResult::Waiting;
    }
    catch (...)
    {
        PublishedRequestedInputObservation.store({},std::memory_order_seq_cst);
        PublishedCompletedWorldInput.store({}, std::memory_order_seq_cst);
        bOwnedWorkerTerminal.Store(true);
        bCanonicalPublicationTerminal.Store(true);
        return ESimulationBoundaryResult::Failed;
    }
}

void ASpeedPhysicsSimulation::CloseInputSessionsOnWorker()
{
    if (ExactRemoteRoundProvider) ExactRemoteRoundFence.Close();
    ON_SCOPE_EXIT { ExactRemoteRoundStorage.reset(); ExactRemoteRoundFence.FinishPublication(); };
    std::optional<Speed::FWorldBorrowGate::FBorrow> ReservedRoundClose;
    if (ExactRemoteRoundStorage)
    {
        ReservedRoundClose.emplace(*ExactRemoteRoundStorage);
        if (!*ReservedRoundClose) std::terminate();
    }
    ActiveExactRemoteRound.reset();
    PendingRoundNeutralStep.reset();
    if(RequestedPreparationV2)RequestedPreparationV2->CloseOnNativeOwner();
    CloseRequestedInputPresentationsOnWorker();
    PublishedRequestedInputObservation.store({},std::memory_order_seq_cst);
    PublishedCompletedWorldInput.store({}, std::memory_order_seq_cst);
    bool NativeRetired=true;
    if(InputSessionRegistry)
    {
        InputSessionRegistry->AbortFrame();
        InputSessionRegistry->MarkNativeV2Retirements(Speed::Input::V2::ERequestedRetirementReasonV2::WorkerClose);
        NativeRetired=InputSessionRegistry->ServiceNativeV2Retirements([this](const auto& D,const auto& B)
            {return SpeedWorldSubsystem&&SpeedWorldSubsystem->RetireRequestedNetworkBindingV2(D,B);});
    }
    if(!NativeRetired)bCanonicalPublicationTerminal.Store(true);
    const bool AdaptersRetired = !SpeedWorldSubsystem || SpeedWorldSubsystem->RetireInputProcessingOnWorker();
    if (!AdaptersRetired) bCanonicalPublicationTerminal.Store(true);
    using namespace Speed::Input::V2;
    PublishedInputFrame.store(std::shared_ptr<const FRegistryFrame>{}, std::memory_order_seq_cst);
    if (InputSessionRegistry)
    {
        const auto View = InputSessionRegistry->ReadRegistry();
        const bool InputsNeutralized = !View || !SpeedWorldSubsystem || SpeedWorldSubsystem->NeutralizeCanonicalInputs(*View);
        if (!InputsNeutralized) bCanonicalPublicationTerminal.Store(true);
        const bool LedgerRetired = InputSessionRegistry->CloseWorkerLedger();
        const bool Retired = NativeRetired && LedgerRetired && AdaptersRetired && InputsNeutralized;
        PublishedInputRegistry.store(InputSessionRegistry->ReadRegistry(), std::memory_order_seq_cst);
        InputSessionRegistry.reset();
        for (const auto& Observation : InputObservations) Observation.second->Deactivate();
        InputObservations.clear();
        bInputOwnersClosedOnWorker.Store(Retired);
        if (Retired)
        {
            FScopeLock Lock(&InputSessionAdmissionMutex);
            for (const auto& Observation : PendingInputObservations) Observation.second->Deactivate();
            PendingInputObservations.clear(); PendingInputJournals.clear(); PendingRequestedNetworkSessions.clear();PendingNativeV2Binds.clear();
        }
    }
}
