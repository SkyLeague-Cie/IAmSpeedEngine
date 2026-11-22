#include "SpeedPhysicsSimulation.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "Misc/ScopeLock.h"

Speed::Input::V2::ECommandAdmission ASpeedPhysicsSimulation::SubmitRequestedNetworkInputSessionV2(Speed::FRequestedNetworkPreparationRequestV2 R)
{
    using namespace Speed::Input::V2;
    if(!NativeRequestedV2RuntimeQualified())return ECommandAdmission::Rejected;
    if(!IsInGameThread()||!SupportsRequestedV2FrameControl()||!RequestedPreparationV2
        ||GetActiveExecutionMode()!=ESimulationExecutionMode::IAmSpeedThread||!bPublishCompletedWorldInput
        ||!CompletedPublicationSource||bInputOwnerRetired.Load()||bOwnedWorkerTerminal.Load()||bCanonicalPublicationTerminal.Load()
        ||!R.Service||!R.Agreement||R.Command.WorkerGeneration!=InputWorkerGeneration||!R.Command.Binding.Network
        ||R.Command.Binding.Network->Version!=2||!R.Command.Binding.Network->IsValid())return ECommandAdmission::Rejected;
    FScopeLock Lock(&InputSessionAdmissionMutex);if(PendingNativeV2Binds.size()>=16)return ECommandAdmission::Rejected;
    try{PendingNativeV2Binds.push_back({R,std::chrono::steady_clock::now()+std::chrono::seconds(5)});}
    catch(...){return ECommandAdmission::Rejected;}
    const auto Result=InputSessionCommands->Submit(R.Command);
    if(Result!=ECommandAdmission::Enqueued)PendingNativeV2Binds.pop_back();else bInputSessionAdmissionRequested.Store(true);
    return Result;
}
Speed::Input::V2::ECommandAdmission ASpeedPhysicsSimulation::SubmitRequestedNetworkDetachV2(
    const Speed::Input::V2::FBoundaryCommandDescriptor& C)
{
    using namespace Speed::Input::V2;
    if(!NativeRequestedV2RuntimeQualified()||!IsInGameThread()||!C.NativeV2Control||!C.NativeV2Control->IsValid()
        ||C.Operation!=EBoundaryOperation::Detach||C.Binding.Network||C.Session!=C.NativeV2Control->Session
        ||C.Epoch!=C.NativeV2Control->Epoch||C.WorkerGeneration!=InputWorkerGeneration
        ||GetActiveExecutionMode()!=ESimulationExecutionMode::IAmSpeedThread||bInputOwnerRetired.Load()
        ||bOwnedWorkerTerminal.Load()||bCanonicalPublicationTerminal.Load())return ECommandAdmission::Rejected;
    FScopeLock Lock(&InputSessionAdmissionMutex);
    const auto Result=InputSessionCommands->Submit(C);
    if(Result==ECommandAdmission::Enqueued)bInputSessionAdmissionRequested.Store(true);
    // In particular, no RequestPause and no legacy neutralization here.
    return Result;
}
void ASpeedPhysicsSimulation::ServiceRequestedNetworkBindsV2AtBoundary(uint64 Cutoff)
{
    using namespace Speed;using namespace Speed::Input::V2;
    std::vector<FPendingNativeV2Bind> Batch;
    {
        FScopeLock Lock(&InputSessionAdmissionMutex);
        // An ordinary input boundary can have no native V2 work at all.
        // Observe emptiness under the same lock used by GT submissions; do not
        // require or fabricate a native V2 owner for that no-op.
        if(PendingNativeV2Binds.empty())return;
        if(!SimulationWorker||!SimulationWorker->OwnsCurrentInputBoundary()||!InputSessionRegistry)throw std::runtime_error("V2 bind outside owner boundary");
        std::lock_guard<std::mutex> Records(InputSessionCommands->Gate);
        auto I=PendingNativeV2Binds.begin();
        while(I!=PendingNativeV2Binds.end())
        {
            const auto R=InputSessionCommands->Records.find(I->Request.Command.Id);
            if(R!=InputSessionCommands->Records.end()&&R->second.EnqueueOrdinal<=Cutoff){Batch.push_back(std::move(*I));I=PendingNativeV2Binds.erase(I);}else ++I;
        }
    }
    for(const auto& P:Batch)
    {
        // Still closed while ANY V2 lifecycle/error path is unqualified.
        // No cached ComparedOnly receipt can bypass this barrier.
        if(!NativeRequestedV2RuntimeQualified())continue;
        try
        {
            ERequestedPreparationReasonV2 Reason=ERequestedPreparationReasonV2::None;
            auto Context=BuildRequestedPreparationV2AtBoundary(P.Request,Cutoff,P.Deadline,Reason);
            if(Reason!=ERequestedPreparationReasonV2::None||!Context)continue;
            const auto* Component=SpeedWorldSubsystem->ResolveRequestedPreparationMappingV2(P.Request.Command.Binding);
            if(!Component||!Component->SupportsCanonicalRequestedInputsV2(*Context))continue;
            const auto& N=*P.Request.Command.Binding.Network;
            std::shared_ptr<const FNativeRequestedAgreementSealV2> Seal(new FNativeRequestedAgreementSealV2(Context,P.Request.Agreement,P.Request.Service->ReadServiceIdentity(),N));
            std::shared_ptr<const FInputSessionRegistry::FNativeNetworkAdmission> Admission(new FInputSessionRegistry::FNativeNetworkAdmission(NativeInputAdmissionOwner,P.Request.Command,P.Request.Service,std::move(Seal)));
            InputSessionRegistry->RegisterNetworkAdmissionAtBoundary(std::move(Admission));
        }
        catch(...)
        {
            // No source exists yet. Only this Bind lacks its private admission
            // and will receive a rejection when the captured lot is drained.
        }
    }
}
