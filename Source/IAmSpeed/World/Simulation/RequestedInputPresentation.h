#pragma once
#include "NativeRequestedInputObservation.h"
#include <map>
#include <mutex>
#include <thread>

class ASpeedPhysicsSimulation;
namespace Speed
{
enum class ERequestedPresentationOperation : uint8 { Register, Remove };
enum class ERequestedPresentationAdmission : uint8 { Enqueued, Rejected };
enum class ERequestedPresentationStatus : uint8 { Pending, Applied, Rejected, Retired, Failed };
enum class ERequestedPresentationReason : uint8
{
    None, Closed, NotCurrentObservation, Inactive, MissingBinding, NativeMapping,
    InvalidProducer, DuplicateAddress, Capacity, MissingRegistration, Removed,
    BindingRetired, ProducerFailure, InvalidPayload,
};
struct FRequestedPresentationDescriptor
{
    uint64 Id=0,WorkerGeneration=0,Session=0,Epoch=0,TargetRegistration=0;
    ERequestedPresentationOperation Operation=ERequestedPresentationOperation::Register;
    uint32 Channel=0,MaxPayloadBytes=0;
    std::shared_ptr<const FNativeRequestedInputObservation> Observation;
};
// Historical command result, never readiness, host approval or a restore grant.
struct FRequestedPresentationReceipt
{
    uint64 Id=0,WorkerGeneration=0,Session=0,Epoch=0,RegistryVersion=0;
    uint32 Channel=0;
    ERequestedPresentationStatus Status=ERequestedPresentationStatus::Pending;
    ERequestedPresentationReason Reason=ERequestedPresentationReason::None;
};
// Presentation data only: exclusive ownership crosses once to the worker.
// Before admission producers must be thread-neutral values-only objects. After
// admission Produce/Invalidate/destruction belong to the canonical owner.
class FRequestedPresentationCommands final
{
    friend class ::ASpeedPhysicsSimulation;
    struct FCommand { FRequestedPresentationDescriptor Descriptor;std::unique_ptr<ISimulationPresentationProducer> Producer; };
    explicit FRequestedPresentationCommands(uint64 G):Generation(G),Publisher(std::this_thread::get_id()){}
    ERequestedPresentationAdmission Submit(FRequestedPresentationDescriptor D,std::unique_ptr<ISimulationPresentationProducer> P)
    {
        if(std::this_thread::get_id()!=Publisher||!D.Id||D.Id==MAX_uint64||D.WorkerGeneration!=Generation
            ||D.Operation>ERequestedPresentationOperation::Remove)return ERequestedPresentationAdmission::Rejected;
        if(D.Operation==ERequestedPresentationOperation::Register)
        {
            if(!P||!D.Observation||!D.Observation->Source()||D.Observation->Source()->WorkerGeneration()!=Generation
                ||!D.Session||!D.Epoch||!D.Channel||!D.MaxPayloadBytes||D.MaxPayloadBytes>1024*1024||D.TargetRegistration)
                return ERequestedPresentationAdmission::Rejected;
        }
        else if(P||!D.TargetRegistration)return ERequestedPresentationAdmission::Rejected;
        std::lock_guard<std::mutex> Lock(Gate);
        if(Closed||D.Id<=HighId||Queue.size()>=32||Receipts.size()>=256)return ERequestedPresentationAdmission::Rejected;
        const auto Id=D.Id;
        FRequestedPresentationReceipt R{D.Id,D.WorkerGeneration,D.Session,D.Epoch,0,D.Channel};
        Receipts.emplace(Id,R);
        try {Queue.push_back({std::move(D),std::move(P)});}
        catch(...){Receipts.erase(Id);return ERequestedPresentationAdmission::Rejected;}
        HighId=Id;return ERequestedPresentationAdmission::Enqueued;
    }
    std::optional<FRequestedPresentationReceipt> Read(uint64 Id) const
    {std::lock_guard<std::mutex> Lock(Gate);const auto I=Receipts.find(Id);return I==Receipts.end()?std::nullopt:std::optional<FRequestedPresentationReceipt>(I->second);}
    std::vector<FCommand> Drain()
    {std::lock_guard<std::mutex> Lock(Gate);std::vector<FCommand> Q;Q.swap(Queue);return Q;}
    void Set(uint64 Id,ERequestedPresentationStatus Status,ERequestedPresentationReason Reason,uint64 Version=0)
    {std::lock_guard<std::mutex> Lock(Gate);auto I=Receipts.find(Id);if(I!=Receipts.end()){I->second.Status=Status;I->second.Reason=Reason;if(Version)I->second.RegistryVersion=Version;}}
    void Close()
    {
        std::vector<FCommand> Q;
        {std::lock_guard<std::mutex> Lock(Gate);Closed=true;Q.swap(Queue);
            for(auto& Pair:Receipts)if(Pair.second.Status==ERequestedPresentationStatus::Pending)
            {Pair.second.Status=ERequestedPresentationStatus::Rejected;Pair.second.Reason=ERequestedPresentationReason::Closed;}}
        // Q releases only never-admitted thread-neutral values on this owner.
    }
    const uint64 Generation;
    const std::thread::id Publisher;
    mutable std::mutex Gate;
    bool Closed=false;uint64 HighId=0;
    std::vector<FCommand> Queue;
    std::map<uint64,FRequestedPresentationReceipt> Receipts;
};
struct FManagedRequestedPresentationBinding
{
    FRequestedPresentationDescriptor Descriptor;
    TSharedPtr<ISimulationPresentationProducer,ESPMode::ThreadSafe> Wrapper;
};
}
