#include "HostAgreementHost.h"
#include <algorithm>
#include <array>
#include <mutex>
#include <stdexcept>
#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <bcrypt.h>
#else
#include <cerrno>
#include <sys/random.h>
#endif

namespace Speed::Netcode::Agreement
{
namespace
{
using FClock=std::chrono::steady_clock;
bool RandomKey(std::array<std::uint8_t,32>& Key)
{
#if defined(_WIN32)
    if(BCryptGenRandom(nullptr,Key.data(),ULONG(Key.size()),BCRYPT_USE_SYSTEM_PREFERRED_RNG)<0)return false;
#else
    std::size_t At=0;
    for(unsigned Try=0;Try<32&&At<Key.size();++Try)
    {
        const auto N=getrandom(Key.data()+At,Key.size()-At,GRND_NONBLOCK);
        if(N<0){if(errno==EINTR)continue;return false;}
        if(!N)return false;At+=std::size_t(N);
    }
    if(At!=Key.size())return false;
#endif
    return std::any_of(Key.begin(),Key.end(),[](auto B){return B!=0;});
}
std::uint64_t NextHost()
{
    static std::atomic<std::uint64_t> Next{1};auto N=Next.load(std::memory_order_relaxed);
    for(unsigned I=0;I<256;++I){if(N==UINT64_MAX)return 0;if(Next.compare_exchange_strong(N,N+1))return N;}return 0;
}
bool ValidLimits(FHostLimits L)
{return L.ActiveSlots&&L.ActiveSlots<=16&&L.Receipts&&L.Receipts<=256&&L.Commands&&L.Commands<=32&&L.PendingAdds&&L.PendingAdds<=16&&L.Lifetime.count()>0&&L.Lifetime.count()<=3600000;}
}
struct FHostAgreementHost::FImpl
{
    struct FCommand
    {
        bool Reserve=false;std::size_t Receipt=0;std::uint64_t Id=0,Ordinal=0;
        std::shared_ptr<FHostSlotControl> Control;
        std::shared_ptr<const FOffer> Offer;
        std::shared_ptr<const FBytes> Bytes;
        FIPv4Endpoint Address;FSessionLimits Session;
    };
    struct FSlot
    {
        bool Used=false,FenceSeen=false;std::size_t Receipt=0;
        std::shared_ptr<FHostSlotControl> Control;
        std::shared_ptr<const FOffer> Offer;
        std::shared_ptr<const FEnrollmentBinding> Binding;
        std::shared_ptr<const FJoinPermit> Permit;
        std::shared_ptr<const FHostAgreementRecord> Record;
        std::unique_ptr<FTcpAgreementSession> Session;
    };
    struct FStreamMark {std::uint64_t Actor=0,Component=0,Incarnation=0,Epoch=0;};
    const FHostLimits Limits;const std::thread::id Controller=std::this_thread::get_id();
    const std::shared_ptr<FHostIdentity> Host;
    mutable std::mutex Gate;
    std::array<FHostReceipt,256> Receipts{};
    std::array<std::shared_ptr<FHostSlotControl>,256> Directory{};
    std::array<std::optional<FCommand>,32> Queue{};
    std::size_t ReceiptCount=0,QueueCount=0,PendingAdds=0;
    std::uint64_t HighId=0,EnqueueOrdinal=0;
    std::array<FHostAudit,2048> Audit{};std::size_t AuditCount=0;
    std::array<FSlot,16> Slots{};std::array<FStreamMark,256> Marks{};std::size_t MarkCount=0;
    std::uint64_t HighOffer=0,CurrentCutoff=0;
    std::atomic<bool> Stop{false},Started{false},Done{false};std::thread Worker;
    FImpl(FHostLimits L,std::shared_ptr<FHostIdentity> H):Limits(L),Host(std::move(H)){}
    void Event(EHostAuditKind Kind,std::uint64_t Command,std::uint64_t Slot,std::uint64_t Close=0,std::uint64_t Fence=0,std::uint64_t Socket=0)
    {
        std::lock_guard<std::mutex> Lock(Gate);
        if(AuditCount==Audit.size())throw std::runtime_error("host audit capacity");
        Audit[AuditCount]={std::uint64_t(AuditCount+1),CurrentCutoff,Command,Slot,Close,Fence,Socket,Kind};++AuditCount;
    }
    void Result(std::size_t I,EHostStatus S,EHostReason Reason,ESessionEnd End=ESessionEnd::None,
        std::shared_ptr<const FJoinPermit> Permit={},std::shared_ptr<const FHostAgreementRecord> Record={})
    {
        std::lock_guard<std::mutex> Lock(Gate);auto& R=Receipts[I];R.Status=S;R.Reason=Reason;R.TransportEnd=End;R.BoundaryCutoff=CurrentCutoff;
        if(Permit)R.Permit=std::move(Permit);if(Record)R.Agreement=std::move(Record);
    }
};
FHostAgreementHost::FHostAgreementHost(std::unique_ptr<FImpl> P):Impl(std::move(P)){}
FHostAgreementHost::~FHostAgreementHost()
{Impl->Host->Close.store(1,std::memory_order_release);if(Impl->Worker.joinable())std::terminate();}
std::unique_ptr<FHostAgreementHost> FHostAgreementHost::Create(std::uint64_t M,std::uint64_t E,FHostLimits L)
{
    if(!M||!E||!ValidLimits(L))return {};const auto G=NextHost();if(!G)return {};
    std::shared_ptr<FHostIdentity> H(new FHostIdentity(M,E,G));return std::unique_ptr<FHostAgreementHost>(new FHostAgreementHost(std::make_unique<FImpl>(L,std::move(H))));
}
bool FHostAgreementHost::Start()
{
    auto& P=*Impl;if(std::this_thread::get_id()!=P.Controller||P.Started.exchange(true))return false;
    try{P.Worker=std::thread([this]{Run();});return true;}
    catch(...){RequestStop();std::lock_guard<std::mutex> Lock(P.Gate);for(std::size_t I=0;I<P.ReceiptCount;++I){P.Receipts[I].Status=EHostStatus::Failed;P.Receipts[I].Reason=EHostReason::AllocationFailure;}P.Done.store(true,std::memory_order_release);return false;}
}
EHostAdmission FHostAgreementHost::SubmitReserve(std::uint64_t Id,FRosterRequest R)
{
    auto& P=*Impl;if(std::this_thread::get_id()!=P.Controller)return EHostAdmission::WrongThread;
    if(!Id||Id==UINT64_MAX||R.Session.Handshake.count()<=0||R.Session.Handshake.count()>30000||R.Session.Lifetime<R.Session.Handshake||R.Session.Lifetime.count()>3600000)return EHostAdmission::Invalid;
    std::shared_ptr<const FOffer> Offer;std::shared_ptr<const FBytes> Bytes;std::shared_ptr<FHostSlotControl> Control;
    try
    {
        auto Encoded=FWireCodec::EncodeOffer(R.Offer);if(!Encoded)return EHostAdmission::Invalid;
        Offer=std::make_shared<const FOffer>(std::move(R.Offer));Bytes=std::make_shared<const FBytes>(std::move(*Encoded));Control=std::shared_ptr<FHostSlotControl>(new FHostSlotControl(Id));
    }
    catch(...){return EHostAdmission::AllocationFailure;}
    std::lock_guard<std::mutex> Lock(P.Gate);
    if(P.Stop.load(std::memory_order_acquire)||P.Host->ReadCloseFence()||P.Done.load(std::memory_order_acquire))return EHostAdmission::Closed;
    if(Id<=P.HighId)return EHostAdmission::ReusedId;
    if(P.ReceiptCount>=P.Limits.Receipts||P.QueueCount>=P.Limits.Commands||P.PendingAdds>=P.Limits.PendingAdds)return EHostAdmission::Capacity;
    const auto I=P.ReceiptCount;const auto Ord=P.EnqueueOrdinal+1;
    FHostReceipt Receipt;Receipt.HostGeneration=P.Host->Generation();Receipt.Id=Id;Receipt.SlotId=Id;Receipt.EnqueueOrdinal=Ord;
    P.Receipts[I]=std::move(Receipt);P.Directory[I]=Control;
    P.Queue[P.QueueCount++]=FImpl::FCommand{true,I,Id,Ord,std::move(Control),std::move(Offer),std::move(Bytes),R.ListenAddress,R.Session};
    ++P.ReceiptCount;++P.PendingAdds;P.HighId=Id;P.EnqueueOrdinal=Ord;return EHostAdmission::Queued;
}
EHostAdmission FHostAgreementHost::SubmitRevoke(std::uint64_t Id,std::uint64_t G,std::uint64_t Slot)
{
    auto& P=*Impl;if(std::this_thread::get_id()!=P.Controller)return EHostAdmission::WrongThread;
    if(!Id||Id==UINT64_MAX||!Slot||G!=P.Host->Generation())return EHostAdmission::Invalid;
    std::lock_guard<std::mutex> Lock(P.Gate);
    if(P.Stop.load(std::memory_order_acquire)||P.Host->ReadCloseFence()||P.Done.load(std::memory_order_acquire))return EHostAdmission::Closed;
    if(Id<=P.HighId)return EHostAdmission::ReusedId;
    std::shared_ptr<FHostSlotControl> C;
    for(std::size_t I=0;I<P.ReceiptCount;++I)if(P.Receipts[I].Id==Slot){C=P.Directory[I];break;}
    if(!C)return EHostAdmission::UnknownSlot;
    if(!C->Close.load(std::memory_order_acquire)){C->ClosingRequest.store(Id,std::memory_order_relaxed);C->Close.store(1,std::memory_order_release);}
    // A signalled close burns its caller ID even if no command slot remains.
    // This is not an accepted command/ordinal or an unrecordable receipt.
    P.HighId=Id;
    if(P.ReceiptCount>=P.Limits.Receipts||P.QueueCount>=P.Limits.Commands)return EHostAdmission::ClosurePostedCapacity;
    const auto I=P.ReceiptCount;const auto Ord=P.EnqueueOrdinal+1;
    FHostReceipt Receipt;Receipt.HostGeneration=G;Receipt.Id=Id;Receipt.SlotId=Slot;Receipt.EnqueueOrdinal=Ord;
    P.Receipts[I]=std::move(Receipt);P.Queue[P.QueueCount++]=FImpl::FCommand{false,I,Id,Ord,std::move(C),{},{},{},{}};
    ++P.ReceiptCount;P.EnqueueOrdinal=Ord;return EHostAdmission::Queued;
}
std::unique_ptr<FTcpAgreementSession> FHostAgreementHost::Join(const FJoinPermit& Permit,FSessionLimits L)
{return FTcpAgreementSession::ConnectEnrolled(Permit.Target,*Permit.Offer,L,Permit.Binding);}
std::optional<FHostReceipt> FHostAgreementHost::ReadReceipt(std::uint64_t Id)const
{
    const auto& P=*Impl;std::lock_guard<std::mutex> Lock(P.Gate);
    for(std::size_t I=0;I<P.ReceiptCount;++I)if(P.Receipts[I].Id==Id)return P.Receipts[I];return {};
}
std::optional<FHostAudit> FHostAgreementHost::ReadAudit(std::size_t I)const
{const auto& P=*Impl;std::lock_guard<std::mutex> Lock(P.Gate);return I<P.AuditCount?std::optional<FHostAudit>(P.Audit[I]):std::nullopt;}
std::shared_ptr<const FHostIdentity> FHostAgreementHost::Identity()const noexcept{return Impl->Host;}
void FHostAgreementHost::RequestStop()noexcept
{Impl->Host->Close.store(1,std::memory_order_release);Impl->Stop.store(true,std::memory_order_release);}
bool FHostAgreementHost::TryJoin()
{
    auto& P=*Impl;if(std::this_thread::get_id()!=P.Controller)return false;if(!P.Started.load())return true;
    if(!P.Done.load(std::memory_order_acquire))return false;if(P.Worker.joinable())P.Worker.join();return true;
}
bool FHostAgreementHost::HasStopped()const noexcept{return Impl->Done.load(std::memory_order_acquire);}
void FHostAgreementHost::Run()noexcept
{
    auto& P=*Impl;const auto End=FClock::now()+P.Limits.Lifetime;
    try
    {
        while(!P.Stop.load(std::memory_order_acquire)&&FClock::now()<End)
        {
            std::array<std::optional<FImpl::FCommand>,32> Batch;std::size_t Count=0;
            {
                std::lock_guard<std::mutex> Lock(P.Gate);Count=P.QueueCount;P.CurrentCutoff=P.EnqueueOrdinal;
                for(std::size_t I=0;I<Count;++I){Batch[I]=std::move(P.Queue[I]);P.Queue[I].reset();if(Batch[I]->Reserve)--P.PendingAdds;}
                P.QueueCount=0;
            }
            if(Count)P.Event(EHostAuditKind::Batch,0,0);
            // Signals were release-published before their queue entries. They
            // also work when a close could not obtain a new receipt slot.
            for(auto& S:P.Slots)if(S.Used&&S.Control->ReadCloseFence()&&!S.FenceSeen)
            {S.FenceSeen=true;S.Session->RequestStop();P.Event(EHostAuditKind::FenceObserved,0,S.Control->SlotId(),S.Control->ReadClosingRequest(),S.Control->ReadCloseFence());}
            for(std::size_t I=0;I<Count;++I)
            {
                auto& C=*Batch[I];if(!C.Reserve){P.Result(C.Receipt,EHostStatus::Applied,EHostReason::Revoked);continue;}
                EHostReason Reject=EHostReason::None;const auto& O=*C.Offer;
                if(P.Host->ReadCloseFence())Reject=EHostReason::Closed;
                else if(C.Control->ReadCloseFence())Reject=EHostReason::Revoked;
                else if(O.Match!=P.Host->Match()||O.MatchEpoch!=P.Host->MatchEpoch())Reject=EHostReason::WrongMatch;
                else if(O.OfferSequence<=P.HighOffer)Reject=EHostReason::ReusedOffer;
                else
                {
                    P.HighOffer=O.OfferSequence;
                    FImpl::FStreamMark* M=nullptr;
                    for(std::size_t J=0;J<P.MarkCount;++J)if(P.Marks[J].Actor==O.Actor&&P.Marks[J].Component==O.Component){M=&P.Marks[J];break;}
                    if(M&&(O.Incarnation<M->Incarnation||(O.Incarnation==M->Incarnation&&O.StreamEpoch<=M->Epoch)))Reject=EHostReason::StaleEpoch;
                    else if(!M&&P.MarkCount==P.Marks.size())Reject=EHostReason::Capacity;
                    else
                    {
                        if(!M)M=&P.Marks[P.MarkCount++];*M={O.Actor,O.Component,O.Incarnation,O.StreamEpoch};
                        for(const auto& S:P.Slots)if(S.Used&&S.Offer->Actor==O.Actor&&S.Offer->Component==O.Component)Reject=EHostReason::ActiveStream;
                    }
                }
                FImpl::FSlot* Slot=nullptr;
                if(Reject==EHostReason::None)
                {for(std::size_t J=0;J<P.Limits.ActiveSlots;++J)if(!P.Slots[J].Used){Slot=&P.Slots[J];break;}if(!Slot)Reject=EHostReason::Capacity;}
                if(Reject!=EHostReason::None)
                {P.Result(C.Receipt,EHostStatus::Rejected,Reject);P.Event(EHostAuditKind::Rejected,C.Id,C.Control->SlotId(),C.Control->ReadClosingRequest(),C.Control->ReadCloseFence());continue;}
                std::array<std::uint8_t,32> Key{};
                if(!RandomKey(Key)){P.Result(C.Receipt,EHostStatus::Failed,EHostReason::RandomUnavailable);P.Event(EHostAuditKind::Rejected,C.Id,C.Id);continue;}
                std::shared_ptr<const FEnrollmentBinding> Binding(new FEnrollmentBinding(P.Host,C.Control,Key,*C.Bytes));
                auto Session=FTcpAgreementSession::ListenEnrolled(C.Address,O,C.Session,Binding);
                if(!Session||!Session->Start()){P.Result(C.Receipt,EHostStatus::Failed,EHostReason::TransportFailure);P.Event(EHostAuditKind::Rejected,C.Id,C.Id);continue;}
                Slot->Used=true;Slot->Receipt=C.Receipt;Slot->Control=std::move(C.Control);Slot->Offer=std::move(C.Offer);Slot->Binding=std::move(Binding);Slot->Session=std::move(Session);
            }
            for(auto& S:P.Slots)if(S.Used)
            {
                if(S.Control->ReadCloseFence())S.Session->RequestStop();
                if(S.Session->HasStopped())
                {
                    if(!S.Session->TryJoin())throw std::runtime_error("wrong session controller");
                    const auto Reason=S.Control->ReadCloseFence()?EHostReason::Revoked:EHostReason::TransportEnded;
                    P.Result(S.Receipt,EHostStatus::Retired,Reason,S.Session->EndReason());
                    P.Event(EHostAuditKind::SlotRetired,0,S.Control->SlotId(),S.Control->ReadClosingRequest(),S.Control->ReadCloseFence(),S.Record?S.Record->Received()->Connection()->Instance():0);S=FImpl::FSlot{};continue;
                }
                if(!S.Control->ReadCloseFence()&&!S.Permit)
                {
                    if(const auto Endpoint=S.Session->ReadBoundEndpoint())
                    {
                        S.Permit=std::shared_ptr<const FJoinPermit>(new FJoinPermit(*Endpoint,S.Binding,S.Offer));
                        P.Result(S.Receipt,EHostStatus::ReadyForJoin,EHostReason::None,ESessionEnd::None,S.Permit);P.Event(EHostAuditKind::PermitPublished,0,S.Control->SlotId());
                    }
                }
                if(!S.Control->ReadCloseFence()&&!P.Host->ReadCloseFence()&&!S.Record)
                {
                    if(const auto V=S.Session->ReadLatestObserved())
                    {
                        if(V->Stage()!=EObservedStage::AcceptReceivedAtServer||!V->Connection()->MatchesHostEnrollment(S.Binding)||V->Connection()->ReadCloseFence()||V->ReceivedCopy()!=S.Binding->OfferBytes||V->SentOfferCopy()!=S.Binding->OfferBytes)throw std::runtime_error("foreign enrollment observation");
                        S.Record=std::shared_ptr<const FHostAgreementRecord>(new FHostAgreementRecord(P.Host,S.Control,S.Offer,V));
                        P.Result(S.Receipt,EHostStatus::AgreementObserved,EHostReason::None,ESessionEnd::None,{},S.Record);P.Event(EHostAuditKind::AgreementPublished,0,S.Control->SlotId(),0,0,V->Connection()->Instance());
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    catch(...){P.Stop.store(true,std::memory_order_release);}
    {
        // Under the admission mutex, close every terminal path (including
        // natural lifetime). A command accepted just before this lock still
        // receives its terminal receipt in the final drain below.
        std::lock_guard<std::mutex> Lock(P.Gate);
        P.Host->Close.store(1,std::memory_order_release);
        P.Stop.store(true,std::memory_order_release);
    }
    // Never detach or declare stopped while a child still owns a socket.
    // The supervising original Job has a finite deadline for a failed join.
    for(auto& S:P.Slots)if(S.Used)S.Session->RequestStop();
    for(;;)
    {
        bool All=true;
        for(auto& S:P.Slots)if(S.Used)
        {
            if(S.Session->TryJoin()){P.Result(S.Receipt,EHostStatus::Retired,EHostReason::Closed,S.Session->EndReason());S=FImpl::FSlot{};}
            else All=false;
        }
        if(All)break;std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    {
        std::lock_guard<std::mutex> Lock(P.Gate);
        for(std::size_t I=0;I<P.ReceiptCount;++I)if(P.Receipts[I].Status==EHostStatus::Pending)
        {P.Receipts[I].Status=EHostStatus::Rejected;P.Receipts[I].Reason=EHostReason::Closed;}
        for(auto& Q:P.Queue)Q.reset();P.QueueCount=0;P.PendingAdds=0;
    }
    P.Done.store(true,std::memory_order_release);
}
}
