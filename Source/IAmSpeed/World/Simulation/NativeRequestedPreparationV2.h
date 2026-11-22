#pragma once
#include "IAmSpeed/Input/InputSessionRegistry.h"
#include "IAmSpeed/Netcode/agreement/HostAgreementHost.h"
#include <chrono>
#include <mutex>

class ASpeedPhysicsSimulation;
namespace Speed
{
struct FNativeRequestedPreparationBoundaryV2
{
    std::uint64_t WorkerGeneration=0,RegistryVersion=0,CanonicalFrame=0,QueueCutoff=0;
    std::uint64_t LocalComponent=0,ComponentGeneration=0,BodyGeneration=0;
    std::uint64_t LogicalRateNumerator=0,LogicalRateDenominator=0,FrameOrigin=0;
    float PhysicalDeltaTime=0;
    Netcode::Agreement::FHostClosureObservation Closures;
};
// Immutable historical context, issued ONLY inside the actual native owner
// boundary. It has no mutable source or admission/CanMove/restore permission.
class FNativeRequestedPreparationContextV2 final
{
public:
    FNativeRequestedPreparationContextV2(const FNativeRequestedPreparationContextV2&)=delete;
    FNativeRequestedPreparationContextV2& operator=(const FNativeRequestedPreparationContextV2&)=delete;
    const Input::V2::FBoundaryCommandDescriptor& Command()const noexcept{return LocalCommand;}
    const Netcode::FNetworkInputWorkerConfiguration& EffectiveConfiguration()const noexcept{return Effective;}
    const std::shared_ptr<const Input::V2::FInputActionContract>& EffectiveContract()const noexcept{return Contract;}
    const Netcode::Agreement::FOffer& ExpectedOffer()const noexcept{return HostRecord->Offer();}
    const Netcode::Agreement::FOffer& ReceivedReply()const noexcept{return Reply;}
    const std::shared_ptr<const Netcode::Agreement::FHostAgreementRecord>& AgreementRecord()const noexcept{return HostRecord;}
    const FNativeRequestedPreparationBoundaryV2& Boundary()const noexcept{return CapturedBoundary;}
    std::thread::id PreparationThread()const noexcept{return Thread;}
private:
    friend class ::ASpeedPhysicsSimulation;
    FNativeRequestedPreparationContextV2(Input::V2::FBoundaryCommandDescriptor C,
        Netcode::FNetworkInputWorkerConfiguration E,std::shared_ptr<const Input::V2::FInputActionContract> K,
        std::shared_ptr<const Netcode::Agreement::FHostAgreementRecord> A,Netcode::Agreement::FOffer R,
        FNativeRequestedPreparationBoundaryV2 B)
        :LocalCommand(std::move(C)),Effective(std::move(E)),Contract(std::move(K)),HostRecord(std::move(A)),
        Reply(std::move(R)),CapturedBoundary(B),Thread(std::this_thread::get_id()){}
    const Input::V2::FBoundaryCommandDescriptor LocalCommand;
    const Netcode::FNetworkInputWorkerConfiguration Effective;
    const std::shared_ptr<const Input::V2::FInputActionContract> Contract;
    const std::shared_ptr<const Netcode::Agreement::FHostAgreementRecord> HostRecord;
    const Netcode::Agreement::FOffer Reply;
    const FNativeRequestedPreparationBoundaryV2 CapturedBoundary;
    const std::thread::id Thread;
};
struct FRequestedNetworkPreparationRequestV2
{
    Input::V2::FBoundaryCommandDescriptor Command;
    std::shared_ptr<Netcode::FNetworkRequestedInputService> Service;
    std::shared_ptr<const Netcode::Agreement::FHostAgreementRecord> Agreement;
};
enum class ERequestedPreparationAdmissionV2:std::uint8_t { Enqueued,WrongThread,Closed,Invalid,ReusedId,Capacity,AllocationFailure };
enum class ERequestedPreparationStatusV2:std::uint8_t { Pending,ComparedOnly,Rejected,Closed };
enum class ERequestedPreparationReasonV2:std::uint8_t
{ None,Expired,Closed,NativeBoundary,Generation,Registry,Mapping,HostIdentity,AgreementIdentity,ServiceConfiguration,Clock,Profile,GameRejected,GameException,AllocationFailure };
struct FRequestedPreparationReceiptV2
{
    std::uint64_t WorkerGeneration=0,Id=0,EnqueueOrdinal=0,BoundaryCutoff=0;
    ERequestedPreparationStatusV2 Status=ERequestedPreparationStatusV2::Pending;
    ERequestedPreparationReasonV2 Reason=ERequestedPreparationReasonV2::None;
    std::shared_ptr<const FNativeRequestedPreparationContextV2> Context;
};
// Queueing public values does not prepare a native context. Only the actual
// driver can capture/complete this queue. No receipt is evicted or acknowledged
// away within this generation. Zero source construction on every route here.
class FRequestedNetworkPreparationCommandsV2 final
{
public:
    FRequestedNetworkPreparationCommandsV2(std::uint64_t G,std::shared_ptr<const Netcode::Agreement::FHostIdentity> H)
        :Generation(G),ExpectedHost(std::move(H)),Publisher(std::this_thread::get_id())
    {if(!G||!ExpectedHost)throw std::invalid_argument("preparation generation/host");}
    std::uint64_t WorkerGeneration()const noexcept{return Generation;}
    const std::shared_ptr<const Netcode::Agreement::FHostIdentity>& Host()const noexcept{return ExpectedHost;}
    ERequestedPreparationAdmissionV2 Submit(FRequestedNetworkPreparationRequestV2 R)
    {
        using namespace Input::V2;
        if(std::this_thread::get_id()!=Publisher)return ERequestedPreparationAdmissionV2::WrongThread;
        const auto& C=R.Command;const auto& D=C.Binding;
        if(!C.Id||C.Id==UINT64_MAX||C.WorkerGeneration!=Generation||C.Operation!=EBoundaryOperation::Bind
            ||!D.Id||!D.Epoch||D.First==UINT64_MAX||!D.Network||D.Network->Version!=2||!D.Network->IsValid()
            ||!D.Contract||D.Kind!=EProducerContract::ExactRemote||D.Controller||D.Producer||D.Journal
            ||D.EnableDesktop||D.AllowScenarioAppend||D.ResumeRearmMask||!D.Scenario.empty()||D.Actors.size()!=1
            ||!R.Service||!R.Agreement||R.Agreement->Host()!=ExpectedHost)return ERequestedPreparationAdmissionV2::Invalid;
        std::shared_ptr<const FRequestedNetworkPreparationRequestV2> Copy;
        try{Copy=std::make_shared<const FRequestedNetworkPreparationRequestV2>(std::move(R));}
        catch(...){return ERequestedPreparationAdmissionV2::AllocationFailure;}
        std::lock_guard<std::mutex> Lock(Gate);
        if(Closed)return ERequestedPreparationAdmissionV2::Closed;
        if(Copy->Command.Id<=HighId)return ERequestedPreparationAdmissionV2::ReusedId;
        if(Count==Records.size()||Queued==Queue.size())return ERequestedPreparationAdmissionV2::Capacity;
        const auto I=Count;const auto Ord=Ordinal+1;
        Records[I].Request=std::move(Copy);Records[I].Deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
        Records[I].Receipt.WorkerGeneration=Generation;Records[I].Receipt.Id=Records[I].Request->Command.Id;Records[I].Receipt.EnqueueOrdinal=Ord;
        Queue[Queued++]=I;++Count;Ordinal=Ord;HighId=Records[I].Receipt.Id;
        return ERequestedPreparationAdmissionV2::Enqueued;
    }
    std::optional<FRequestedPreparationReceiptV2> Read(std::uint64_t G,std::uint64_t Id)const
    {
        if(G!=Generation)return {};std::lock_guard<std::mutex> Lock(Gate);
        for(std::size_t I=0;I<Count;++I)if(Records[I].Receipt.Id==Id)return Records[I].Receipt;return {};
    }
private:
    friend class ::ASpeedPhysicsSimulation;
    struct FRecord
    {
        std::shared_ptr<const FRequestedNetworkPreparationRequestV2> Request;
        FRequestedPreparationReceiptV2 Receipt;
        std::chrono::steady_clock::time_point Deadline;
    };
    struct FCaptured
    {
        std::size_t Index=0;
        std::shared_ptr<const FRequestedNetworkPreparationRequestV2> Request;
        std::chrono::steady_clock::time_point Deadline;
    };
    struct FBatch {std::uint64_t Cutoff=0;std::vector<FCaptured> Requests;};
    FBatch CaptureOnNativeOwner()
    {
        std::lock_guard<std::mutex> Lock(Gate);FBatch B;B.Requests.reserve(Queued);B.Cutoff=Ordinal;
        for(std::size_t J=0;J<Queued;++J){const auto I=Queue[J];B.Requests.push_back({I,Records[I].Request,Records[I].Deadline});Records[I].Receipt.BoundaryCutoff=B.Cutoff;}
        Queued=0;return B;
    }
    void CompleteOnNativeOwner(std::size_t I,ERequestedPreparationStatusV2 S,ERequestedPreparationReasonV2 Reason,
        std::shared_ptr<const FNativeRequestedPreparationContextV2> Context={})
    {std::lock_guard<std::mutex> Lock(Gate);auto& R=Records[I].Receipt;if(R.Status!=ERequestedPreparationStatusV2::Pending)throw std::logic_error("preparation completion reuse");R.Status=S;R.Reason=Reason;R.Context=std::move(Context);}
    void CloseOnNativeOwner()
    {
        std::lock_guard<std::mutex> Lock(Gate);Closed=true;Queued=0;
        for(std::size_t I=0;I<Count;++I)if(Records[I].Receipt.Status==ERequestedPreparationStatusV2::Pending)
        {Records[I].Receipt.Status=ERequestedPreparationStatusV2::Closed;Records[I].Receipt.Reason=ERequestedPreparationReasonV2::Closed;}
    }
    // This tranche never constructs a mutable input source. The native driver
    // may also close inert pending requests after it has actually joined (or
    // proved absence of) its worker; not from a timeout/PID observation.
    void CloseUnprocessedAfterNativeOwnerJoined(){CloseOnNativeOwner();}
    bool CanResetAfterJoin()const
    {std::lock_guard<std::mutex> Lock(Gate);return !Queued&&std::all_of(Records.begin(),Records.begin()+Count,[](const auto& R){return R.Receipt.Status!=ERequestedPreparationStatusV2::Pending;});}
    const std::uint64_t Generation;
    const std::shared_ptr<const Netcode::Agreement::FHostIdentity> ExpectedHost;
    const std::thread::id Publisher;
    mutable std::mutex Gate;
    std::array<FRecord,256> Records{};
    std::array<std::size_t,16> Queue{};
    std::size_t Count=0,Queued=0;
    std::uint64_t HighId=0,Ordinal=0;
    bool Closed=false;
};
}
