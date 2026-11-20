#pragma once
#include "TcpAgreementSession.h"
#include "HostAgreementTypes.h"

namespace Speed::Netcode::Agreement
{
class FJoinPermit final
{
public:
    const FIPv4Endpoint& Endpoint()const noexcept{return Target;}
    std::uint64_t SlotId()const noexcept{return Binding->Slot->SlotId();}
    std::shared_ptr<const FHostIdentity> Host()const noexcept{return Binding->Host;}
private:
    friend class FHostAgreementHost;
    FJoinPermit(FIPv4Endpoint E,std::shared_ptr<const FEnrollmentBinding> B,std::shared_ptr<const FOffer> O)
        :Target(E),Binding(std::move(B)),Offer(std::move(O)){}
    const FIPv4Endpoint Target;
    const std::shared_ptr<const FEnrollmentBinding> Binding;
    const std::shared_ptr<const FOffer> Offer;
};
struct FHostClosureObservation {std::uint64_t Host=0,Slot=0,Socket=0;};
// Issued only by the real host from one of its own enrolled TCP sessions.
// This is roster/connection evidence, NOT native physical admission.
class FHostAgreementRecord final
{
public:
    const std::shared_ptr<const FHostIdentity>& Host()const noexcept{return Issuer;}
    std::uint64_t SlotId()const noexcept{return Control->SlotId();}
    const FOffer& Offer()const noexcept{return *Expected;}
    const std::shared_ptr<const FConnectionObservedAgreement>& Received()const noexcept{return Observation;}
    FHostClosureObservation ReadClosure()const noexcept
    {return {Issuer->ReadCloseFence(),Control->ReadCloseFence(),Observation->Connection()->ReadCloseFence()};}
private:
    friend class FHostAgreementHost;
    FHostAgreementRecord(std::shared_ptr<const FHostIdentity> H,std::shared_ptr<const FHostSlotControl> C,
        std::shared_ptr<const FOffer> O,std::shared_ptr<const FConnectionObservedAgreement> V)
        :Issuer(std::move(H)),Control(std::move(C)),Expected(std::move(O)),Observation(std::move(V)){}
    const std::shared_ptr<const FHostIdentity> Issuer;
    const std::shared_ptr<const FHostSlotControl> Control;
    const std::shared_ptr<const FOffer> Expected;
    const std::shared_ptr<const FConnectionObservedAgreement> Observation;
};
enum class EHostAdmission:std::uint8_t
{ Queued,WrongThread,Closed,Invalid,ReusedId,Capacity,AllocationFailure,UnknownSlot,ClosurePostedCapacity };
enum class EHostStatus:std::uint8_t { Pending,ReadyForJoin,AgreementObserved,Applied,Rejected,Retired,Failed };
enum class EHostReason:std::uint8_t
{ None,Closed,Revoked,WrongMatch,ActiveStream,StaleEpoch,ReusedOffer,Capacity,RandomUnavailable,TransportFailure,TransportEnded,AllocationFailure };
struct FHostReceipt
{
    std::uint64_t HostGeneration=0,Id=0,SlotId=0,EnqueueOrdinal=0,BoundaryCutoff=0;
    EHostStatus Status=EHostStatus::Pending;
    EHostReason Reason=EHostReason::None;
    ESessionEnd TransportEnd=ESessionEnd::None;
    std::shared_ptr<const FJoinPermit> Permit;
    std::shared_ptr<const FHostAgreementRecord> Agreement;
};
enum class EHostAuditKind:std::uint8_t { Batch,Rejected,PermitPublished,AgreementPublished,FenceObserved,SlotRetired };
struct FHostAudit
{
    std::uint64_t Ordinal=0,Cutoff=0,Command=0,Slot=0,ClosingRequest=0,SlotFence=0,SocketInstance=0;
    EHostAuditKind Kind=EHostAuditKind::Batch;
};
struct FHostLimits
{
    std::size_t ActiveSlots=16,Receipts=256,Commands=32,PendingAdds=16;
    std::chrono::milliseconds Lifetime{300000};
};
struct FRosterRequest
{
    FOffer Offer;
    FIPv4Endpoint ListenAddress{};
    FSessionLimits Session;
};
class IAMSPEED_AGREEMENT_API FHostAgreementHost final
{
public:
    static std::unique_ptr<FHostAgreementHost> Create(std::uint64_t Match,std::uint64_t MatchEpoch,FHostLimits Limits={});
    ~FHostAgreementHost();
    FHostAgreementHost(const FHostAgreementHost&)=delete;
    FHostAgreementHost& operator=(const FHostAgreementHost&)=delete;
    bool Start();
    EHostAdmission SubmitReserve(std::uint64_t Id,FRosterRequest Request);
    // Closing the actual slot is signalled even when the new command cannot
    // be accepted for capacity. ClosurePostedCapacity is not an accepted ID.
    EHostAdmission SubmitRevoke(std::uint64_t Id,std::uint64_t HostGeneration,std::uint64_t SlotId);
    static std::unique_ptr<FTcpAgreementSession> Join(const FJoinPermit&,FSessionLimits Limits={});
    std::optional<FHostReceipt> ReadReceipt(std::uint64_t Id)const;
    std::optional<FHostAudit> ReadAudit(std::size_t Index)const;
    std::shared_ptr<const FHostIdentity> Identity()const noexcept;
    void RequestStop()noexcept;
    bool TryJoin();
    bool HasStopped()const noexcept;
private:
    struct FImpl;
    explicit FHostAgreementHost(std::unique_ptr<FImpl>);
    void Run()noexcept;
    std::unique_ptr<FImpl> Impl;
};
}
