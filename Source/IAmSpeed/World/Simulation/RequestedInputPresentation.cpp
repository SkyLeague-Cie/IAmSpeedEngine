#include "RequestedInputPresentation.h"
#include "SpeedPhysicsSimulation.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "Misc/ScopeLock.h"

namespace Speed
{
class FManagedRequestedPresentation final:public ISimulationPresentationProducer
{
public:
    FManagedRequestedPresentation(uint64 Id,uint32 InChannel,uint32 Cap,std::unique_ptr<ISimulationPresentationProducer> P)
        :Owner(Id),WireChannel(InChannel),MaxBytes(Cap),Inner(std::move(P)),Thread(std::this_thread::get_id()){}
    uint64 OwnerStableId()const override{return Owner;}
    uint32 Channel()const override{return WireChannel;}
    bool SupportsIndependentFrameProduction()const override{return false;}
    bool CanRestore(const FSimulationPresentationOutput&,uint64)const override{return false;}
    void Produce(const FSimulationSnapshot& Bodies,FSimulationPresentationOutput& Out)override
    {
        Out.Payload.Reset();Out.StatePayload.Reset();
        if(!Valid||std::this_thread::get_id()!=Thread)return;
        try
        {
            Inner->Produce(Bodies,Out);
            if(Out.StatePayload.Num()||Out.Payload.Num()<0||uint32(Out.Payload.Num())>MaxBytes)
            {Out.Payload.Reset();Out.StatePayload.Reset();Failure=ERequestedPresentationReason::InvalidPayload;Valid=false;}
        }
        catch(...){Out.Payload.Reset();Out.StatePayload.Reset();Failure=ERequestedPresentationReason::ProducerFailure;Valid=false;}
    }
    void InvalidateTimeline()override
    {
        Valid=false;
        if(std::this_thread::get_id()!=Thread){Failure=ERequestedPresentationReason::ProducerFailure;return;}
        if(Invalidated)return;Invalidated=true;
        try{Inner->InvalidateTimeline();}catch(...){Failure=ERequestedPresentationReason::ProducerFailure;}
    }
    ERequestedPresentationReason GetFailure()const{return Failure;}
private:
    const uint64 Owner;const uint32 WireChannel,MaxBytes;
    std::unique_ptr<ISimulationPresentationProducer> Inner;
    const std::thread::id Thread;
    bool Valid=true,Invalidated=false;
    ERequestedPresentationReason Failure=ERequestedPresentationReason::None;
};
}

Speed::ERequestedPresentationAdmission ASpeedPhysicsSimulation::SubmitRequestedInputPresentation(
    Speed::FRequestedPresentationDescriptor D,std::unique_ptr<ISimulationPresentationProducer> P)
{
    using namespace Speed;
    const auto Commands=RequestedPresentationCommands.load(std::memory_order_seq_cst);
    if(!IsInGameThread()||!Commands||GetActiveExecutionMode()!=ESimulationExecutionMode::IAmSpeedThread
        ||bPresentationBindingClosed||!bPublishCompletedWorldInput||bInputOwnerRetired.Load()||bOwnedWorkerTerminal.Load()
        ||bCanonicalPublicationTerminal.Load()||D.WorkerGeneration!=InputWorkerGeneration)return ERequestedPresentationAdmission::Rejected;
    const auto Result=Commands->Submit(std::move(D),std::move(P));
    if(Result==ERequestedPresentationAdmission::Enqueued)bInputSessionAdmissionRequested.Store(true);
    return Result;
}
std::optional<Speed::FRequestedPresentationReceipt> ASpeedPhysicsSimulation::ReadRequestedInputPresentationReceipt(uint64 Id)const
{const auto Commands=RequestedPresentationCommands.load(std::memory_order_seq_cst);return Commands?Commands->Read(Id):std::nullopt;}

Speed::ERequestedPresentationReason ASpeedPhysicsSimulation::ValidateRequestedInputPresentation(
    const Speed::FRequestedPresentationDescriptor& D)const
{
    using namespace Speed;using namespace Speed::Input::V2;
    if(!SimulationWorker||!SimulationWorker->OwnsCurrentInputBoundary()||!InputSessionRegistry
        ||!CompletedPublicationSource||!D.Observation||D.Observation->Source()!=CompletedPublicationSource
        ||D.WorkerGeneration!=InputWorkerGeneration||D.Observation->RegistryAtBoundary()!=InputSessionRegistry->View)
        return ERequestedPresentationReason::NotCurrentObservation;
    if(!InputSessionRegistry->CanObserveRequestedBindingsAtBoundary())return ERequestedPresentationReason::Inactive;
    const auto& V=*InputSessionRegistry->View;
    const auto I=std::find_if(V.Bindings.begin(),V.Bindings.end(),[&](const auto& B){return B.Id==D.Session&&B.Epoch==D.Epoch;});
    if(I==V.Bindings.end()||!I->Network)return ERequestedPresentationReason::MissingBinding;
    if(!SpeedWorldSubsystem)return ERequestedPresentationReason::NativeMapping;
    if(I->Network->Version==2)
    {
        const auto S=InputSessionRegistry->Sessions.find(I->Id);
        if(S==InputSessionRegistry->Sessions.end()||!S->second.NativeV2Binding
            ||!SpeedWorldSubsystem->ValidateRequestedNetworkBindingForFrameV2(*I,S->second.NativeV2Binding))return ERequestedPresentationReason::NativeMapping;
        const auto C=S->second.NativeV2Binding->Seal()->ReadClosure();
        if(C.Host||C.Slot||C.Socket)return ERequestedPresentationReason::Inactive;
    }
    else if(!SpeedWorldSubsystem->ValidateRequestedNetworkBinding(*I))return ERequestedPresentationReason::NativeMapping;
    return ERequestedPresentationReason::None;
}

void ASpeedPhysicsSimulation::RetireRequestedInputPresentation(uint64 Id,Speed::ERequestedPresentationReason Reason)
{
    using namespace Speed;
    auto I=ManagedRequestedPresentations.find(Id);if(I==ManagedRequestedPresentations.end())return;
    auto Wrapper=I->second.Wrapper;
    // No command or presentation-list lock across game code. This boundary
    // follows a fully returned/joined Produce call on the same canonical owner.
    auto* Managed=static_cast<FManagedRequestedPresentation*>(Wrapper.Get());
    Managed->InvalidateTimeline();const auto Failure=Managed->GetFailure();
    {FScopeLock Lock(&PresentationProducerMutex);PresentationProducers.RemoveAll([&](const auto& P){return &P.Get()==Wrapper.Get();});}
    ManagedRequestedPresentations.erase(I);
    if(const auto Commands=RequestedPresentationCommands.load(std::memory_order_seq_cst))Commands->Set(Id,
        Failure==ERequestedPresentationReason::None?ERequestedPresentationStatus::Retired:ERequestedPresentationStatus::Failed,
        Failure==ERequestedPresentationReason::None?Reason:Failure);
}

void ASpeedPhysicsSimulation::ServiceRequestedInputPresentationsAtBoundary()
{
    using namespace Speed;
    const auto Commands=RequestedPresentationCommands.load(std::memory_order_seq_cst);
    if(!Commands)return;
    std::vector<uint64> Retire;
    for(const auto& Pair:ManagedRequestedPresentations)
    {
        auto* Wrapper=static_cast<FManagedRequestedPresentation*>(Pair.second.Wrapper.Get());
        if(ValidateRequestedInputPresentation(Pair.second.Descriptor)!=ERequestedPresentationReason::None
            ||Wrapper->GetFailure()!=ERequestedPresentationReason::None)Retire.push_back(Pair.first);
    }
    for(const auto Id:Retire)RetireRequestedInputPresentation(Id,ERequestedPresentationReason::BindingRetired);
    auto Pending=Commands->Drain();
    for(auto& C:Pending)
    {
        auto& D=C.Descriptor;auto Status=ERequestedPresentationStatus::Rejected;auto Reason=ERequestedPresentationReason::None;
        if(!SimulationWorker||!SimulationWorker->OwnsCurrentInputBoundary())Reason=ERequestedPresentationReason::NotCurrentObservation;
        else if(D.Operation==ERequestedPresentationOperation::Remove)
        {
            if(!ManagedRequestedPresentations.count(D.TargetRegistration))Reason=ERequestedPresentationReason::MissingRegistration;
            else{RetireRequestedInputPresentation(D.TargetRegistration,ERequestedPresentationReason::Removed);Status=ERequestedPresentationStatus::Applied;}
        }
        else if((Reason=ValidateRequestedInputPresentation(D))==ERequestedPresentationReason::None)
        {
            try
            {
                const auto& V=*InputSessionRegistry->View;
                const auto B=std::find_if(V.Bindings.begin(),V.Bindings.end(),[&](const auto& X){return X.Id==D.Session&&X.Epoch==D.Epoch;});
                const uint64 BoundOwner=B->Network->LocalComponent;
                if(!C.Producer||C.Producer->OwnerStableId()!=BoundOwner||C.Producer->Channel()!=D.Channel
                    ||C.Producer->IsCameraPresentation()||C.Producer->SupportsIndependentFrameProduction())Reason=ERequestedPresentationReason::InvalidProducer;
                else
                {
                    auto Wrapper=MakeShared<FManagedRequestedPresentation,ESPMode::ThreadSafe>(BoundOwner,D.Channel,D.MaxPayloadBytes,std::move(C.Producer));
                    bool Added=false;
                    {
                        FScopeLock Lock(&PresentationProducerMutex);
                        const bool Duplicate=PresentationProducers.ContainsByPredicate([&](const auto& P){return P->OwnerStableId()==BoundOwner&&P->Channel()==D.Channel;});
                        if(Duplicate)Reason=ERequestedPresentationReason::DuplicateAddress;
                        else if(bPresentationBindingClosed||PresentationProducers.Num()>=64)Reason=ERequestedPresentationReason::Capacity;
                        else
                        {
                            ManagedRequestedPresentations.emplace(D.Id,FManagedRequestedPresentationBinding{D,Wrapper});
                            try{PresentationProducers.Add(Wrapper);Added=true;}
                            catch(...){ManagedRequestedPresentations.erase(D.Id);throw;}
                        }
                    }
                    if(Added)Status=ERequestedPresentationStatus::Applied;
                }
            }
            catch(...){Reason=ERequestedPresentationReason::ProducerFailure;}
        }
        Commands->Set(D.Id,Status,Reason,InputSessionRegistry?InputSessionRegistry->View->Version:0);
    }
}
void ASpeedPhysicsSimulation::CloseRequestedInputPresentationsOnWorker()
{
    using namespace Speed;
    while(!ManagedRequestedPresentations.empty())RetireRequestedInputPresentation(ManagedRequestedPresentations.begin()->first,ERequestedPresentationReason::Closed);
    if(const auto Commands=RequestedPresentationCommands.load(std::memory_order_seq_cst))Commands->Close();
}
