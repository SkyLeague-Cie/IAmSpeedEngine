#pragma once
// Production fixed-capacity state machine. Standard C++ only; no Unreal shim.
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>

namespace Speed::CanonicalObserver20
{
constexpr std::size_t Capacity=8;
constexpr std::uint64_t MaxBoundedFrames=6000;
enum class EReadiness:std::uint8_t {None,Waiting,Ready,Complete,Failed};
enum class ECommitAction:std::uint8_t {Continue,Complete,Failed};
enum class EPhase:std::uint8_t {Configuring,Running,Complete,Failed,Retired};
enum class EFailure:std::uint8_t
{None,WrongLane,WrongGeneration,ConcurrentAccess,ReadinessException,ReadinessRefused,
 CommitException,CommitRefused,CommitOrder,CommitProof,AlreadyTerminal};
namespace Detail
{
// Only Configuring may seal to Running. DriverFailed on another lane
// wins without being overwritten by this registration-to-worker handoff.
inline bool SealConfiguring(std::atomic<EPhase>& Phase)
{
    EPhase Expected=EPhase::Configuring;
    return Phase.compare_exchange_strong(Expected,EPhase::Running,
        std::memory_order_acq_rel,std::memory_order_acquire);
}
// Only Running may become Complete. A concurrent GT DriverFailed or a retired
// generation must never be overwritten by a worker completion decision.
inline EReadiness CompleteRunning(std::atomic<EPhase>& Phase)
{
    EPhase Expected=EPhase::Running;
    if(Phase.compare_exchange_strong(Expected,EPhase::Complete,
        std::memory_order_acq_rel,std::memory_order_acquire))return EReadiness::Complete;
    return Expected==EPhase::Complete?EReadiness::Complete:EReadiness::Failed;
}
}
struct FHandle
{
    std::uint64_t Registry=0,Generation=0,OwnerToken=0,Token=0;
    explicit operator bool()const{return Registry&&Generation&&OwnerToken&&Token;}
};
struct FConfig {std::uint64_t Generation=0,OwnerToken=0,MaximumCommittedFrames=0;};
struct FCommitReceipt
{
    std::uint64_t Generation=0,Frame=0,PublicationSerial=0,StateHash=0,InputJournalHash=0;
    bool ParticipantsCommitted=false,InputsCompleted=false,RecorderServiceRan=false;
};
struct FJoinProof
{
    std::uint64_t Registry=0,Generation=0;
    bool WorkerStoppedAndJoined=false,InputOwnersClosed=false;
};
struct FStatus
{
    EPhase Phase=EPhase::Configuring;EFailure Failure=EFailure::None;
    std::uint64_t FailureOwner=0,LastCommittedFrame=0,LastPublicationSerial=0;
    bool HasCommittedFrame=false;
};
template<class Snapshot> struct IObserver
{
    virtual ~IObserver()=default;
    // Owner lane only. Readiness may change through caller-owned atomic values;
    // no UObject, IO, wait, join, setters or input calls are permitted here.
    virtual EReadiness Readiness()const=0;
    // Snapshot and receipt are borrowed only for this call. The observer may
    // update its own preallocated storage, never the simulation or inputs.
    virtual ECommitAction OnCommitted(const FCommitReceipt&,const Snapshot&)=0;
};

template<class Snapshot> class TRegistry
{
    struct FSlot
    {
        FHandle Handle;std::uint64_t MaximumFrames=0;
        std::shared_ptr<IObserver<Snapshot>> Observer;
    };
    const std::uint64_t RegistryId,Generation;
    std::array<FSlot,Capacity> Slots{};
    std::uint64_t NextToken=1,ExpectedFrame=0,PreviousSerial=0;
    std::atomic<EPhase> Phase{EPhase::Configuring};
    std::atomic<EFailure> Failure{EFailure::None};
    std::atomic<std::uint64_t> FailureOwner{0},LastFrame{0},LastSerial{0};
    std::atomic<bool> HasFrame{false},InCallback{false},ReadyAdmitted{false};
    std::atomic_flag Mutation=ATOMIC_FLAG_INIT;
    bool Match(const FHandle& H)const
    {return H&&H.Registry==RegistryId&&H.Generation==Generation;}
    bool Joined(const FJoinProof& P)const
    {return P.Registry==RegistryId&&P.Generation==Generation&&P.WorkerStoppedAndJoined&&P.InputOwnersClosed;}
    EReadiness Fail(EFailure Why,std::uint64_t Owner=0)
    {
        if(Phase.load(std::memory_order_acquire)==EPhase::Retired)return EReadiness::Failed;
        EFailure Empty=EFailure::None;
        if(Failure.compare_exchange_strong(Empty,Why))FailureOwner.store(Owner);
        Phase.store(EPhase::Failed,std::memory_order_release);return EReadiness::Failed;
    }
    bool EnterCall()
    {bool Expected=false;return InCallback.compare_exchange_strong(Expected,true,std::memory_order_acq_rel);}
    void LeaveCall(){InCallback.store(false,std::memory_order_release);}
public:
    TRegistry(std::uint64_t Id,std::uint64_t Gen):RegistryId(Id),Generation(Gen){}
    TRegistry(const TRegistry&)=delete;TRegistry& operator=(const TRegistry&)=delete;
    std::uint64_t Id()const{return RegistryId;}
    std::uint64_t InputGeneration()const{return Generation;}
    bool Contains(const FHandle& H)const
    {
        if(!Match(H)||Phase.load()==EPhase::Retired)return false;
        for(const auto& S:Slots)if(S.Observer&&S.Handle.Token==H.Token&&S.Handle.OwnerToken==H.OwnerToken)return true;
        return false;
    }
    // GT/quiescent registration only. Once a worker asks for its decision, the
    // slot set is immutable until an actual same-generation join has completed.
    bool Add(const FConfig& C,std::shared_ptr<IObserver<Snapshot>> O,FHandle& Out)
    {
        Out={};
        if(!RegistryId||!Generation||!C.OwnerToken||C.Generation!=Generation||!O||
           !C.MaximumCommittedFrames||C.MaximumCommittedFrames>MaxBoundedFrames||
           Phase.load(std::memory_order_acquire)!=EPhase::Configuring||InCallback.load()||Mutation.test_and_set())return false;
        if(Phase.load(std::memory_order_acquire)!=EPhase::Configuring)
        {Mutation.clear(std::memory_order_release);return false;}
        FSlot* Empty=nullptr;
        for(auto& S:Slots)
        {
            if(S.Observer&&S.Handle.OwnerToken==C.OwnerToken){Mutation.clear(std::memory_order_release);return false;}
            if(!S.Observer&&!Empty)Empty=&S;
        }
        if(!Empty||!NextToken||NextToken==std::numeric_limits<std::uint64_t>::max())
        {Mutation.clear(std::memory_order_release);return false;}
        Empty->Handle={RegistryId,Generation,C.OwnerToken,NextToken++};
        Empty->MaximumFrames=C.MaximumCommittedFrames;Empty->Observer=std::move(O);
        const bool Accepted=Phase.load(std::memory_order_acquire)==EPhase::Configuring;
        if(Accepted)Out=Empty->Handle;
        Mutation.clear(std::memory_order_release);return Accepted;
    }
    EReadiness Decision(bool OnOwnerLane,std::uint64_t ActualGeneration)
    {
        if(!OnOwnerLane)return Fail(EFailure::WrongLane);
        if(ActualGeneration!=Generation)return Fail(EFailure::WrongGeneration);
        auto P=Phase.load(std::memory_order_acquire);
        if(P==EPhase::Complete)return EReadiness::Complete;
        if(P==EPhase::Failed||P==EPhase::Retired)return EReadiness::Failed;
        if(P==EPhase::Configuring)
        {
            if(Mutation.test_and_set())return Fail(EFailure::ConcurrentAccess);
            const bool Sealed=Detail::SealConfiguring(Phase);
            Mutation.clear(std::memory_order_release);
            if(!Sealed)return EReadiness::Failed;
        }
        if(!EnterCall())return Fail(EFailure::ConcurrentAccess);
        bool Any=false,Waiting=false,Complete=false,Bad=false;
        for(const auto& S:Slots)if(S.Observer)
        {
            Any=true;EReadiness R=EReadiness::Failed;
            try{R=S.Observer->Readiness();}
            catch(...){Fail(EFailure::ReadinessException,S.Handle.OwnerToken);Bad=true;continue;}
            if(R==EReadiness::Waiting)Waiting=true;
            else if(R==EReadiness::Complete)Complete=true;
            else if(R!=EReadiness::Ready){Fail(EFailure::ReadinessRefused,S.Handle.OwnerToken);Bad=true;}
        }
        LeaveCall();
        if(Bad||Phase.load()==EPhase::Failed)return EReadiness::Failed;
        if(Complete)return Detail::CompleteRunning(Phase);
        ReadyAdmitted.store(Any&&!Waiting,std::memory_order_release);
        return !Any?EReadiness::None:Waiting?EReadiness::Waiting:EReadiness::Ready;
    }
    // Check before EVERY actual step, including frames inside one driver batch.
    bool AllowsStep(bool OnOwnerLane,std::uint64_t ActualGeneration,std::uint64_t Frame)
    {
        if(!OnOwnerLane){Fail(EFailure::WrongLane);return false;}
        if(ActualGeneration!=Generation){Fail(EFailure::WrongGeneration);return false;}
        const auto P=Phase.load();
        if(P==EPhase::Complete||P==EPhase::Failed||P==EPhase::Retired)return false;
        if(P!=EPhase::Running||!ReadyAdmitted.load(std::memory_order_acquire)||Frame!=ExpectedFrame){Fail(EFailure::CommitOrder);return false;}
        return true;
    }
    EReadiness Observe(bool OnOwnerLane,const FCommitReceipt& R,const Snapshot& Values)
    {
        if(!OnOwnerLane)return Fail(EFailure::WrongLane);
        if(R.Generation!=Generation)return Fail(EFailure::WrongGeneration);
        if(Phase.load()!=EPhase::Running)return Fail(EFailure::AlreadyTerminal);
        if(!ReadyAdmitted.load(std::memory_order_acquire))return Fail(EFailure::CommitProof);
        if(!R.ParticipantsCommitted||!R.InputsCompleted||!R.RecorderServiceRan||!R.PublicationSerial)
            return Fail(EFailure::CommitProof);
        if(R.Frame!=ExpectedFrame||R.PublicationSerial<=PreviousSerial)return Fail(EFailure::CommitOrder);
        if(!EnterCall())return Fail(EFailure::ConcurrentAccess);
        // The receipt is from an already published frame. Preserve that fact
        // even if a callback later refuses or throws; there is no rollback.
        LastFrame.store(R.Frame);LastSerial.store(R.PublicationSerial);HasFrame.store(true,std::memory_order_release);
        PreviousSerial=R.PublicationSerial;++ExpectedFrame;
        bool Complete=false,Bad=false;
        for(const auto& S:Slots)if(S.Observer)
        {
            ECommitAction A=ECommitAction::Failed;
            try{A=S.Observer->OnCommitted(R,Values);}
            catch(...){Fail(EFailure::CommitException,S.Handle.OwnerToken);Bad=true;continue;}
            if(A!=ECommitAction::Continue&&A!=ECommitAction::Complete)
            {Fail(EFailure::CommitRefused,S.Handle.OwnerToken);Bad=true;}
            else if(A==ECommitAction::Complete||R.Frame+1>=S.MaximumFrames)Complete=true;
        }
        LeaveCall();
        if(Bad||Phase.load()==EPhase::Failed)return EReadiness::Failed;
        if(Complete)return Detail::CompleteRunning(Phase);
        return EReadiness::Ready;
    }
    void DriverFailed(){Fail(EFailure::CommitRefused);}
    bool HasCommittedAt(std::uint64_t Frame)const
    {return HasFrame.load(std::memory_order_acquire)&&LastFrame.load()==Frame;}
    FStatus Status()const
    {return {Phase.load(),Failure.load(),FailureOwner.load(),LastFrame.load(),LastSerial.load(),HasFrame.load()};}
    // The booleans below are not a public proof oracle: the UE backend obtains
    // them from actual driver join/input-owner witnesses of this generation.
    bool Retire(const FHandle& H,const FJoinProof& P)
    {
        if(!Match(H)||!Joined(P)||InCallback.load()||Mutation.test_and_set())return false;
        FSlot* Found=nullptr;
        for(auto& S:Slots)if(S.Observer&&S.Handle.Token==H.Token&&S.Handle.OwnerToken==H.OwnerToken){Found=&S;break;}
        if(!Found){Mutation.clear(std::memory_order_release);return false;}
        Found->Handle={};Found->MaximumFrames=0; // invalidate before observer destruction/reentrancy
        Found->Observer.reset();
        bool Any=false;for(const auto& S:Slots)Any=Any||bool(S.Observer);
        if(!Any)Phase.store(EPhase::Retired,std::memory_order_release);
        Mutation.clear(std::memory_order_release);return true;
    }
    bool RetireAll(const FJoinProof& P)
    {
        if(!Joined(P)||InCallback.load()||Mutation.test_and_set())return false;
        if(Phase.load()==EPhase::Retired){Mutation.clear(std::memory_order_release);return false;}
        for(auto& S:Slots){S.Handle={};S.MaximumFrames=0;} // all tokens invalid first
        for(auto& S:Slots)S.Observer.reset();
        Phase.store(EPhase::Retired,std::memory_order_release);Mutation.clear(std::memory_order_release);return true;
    }
};
// Existing participant control remains independent. Failed wins; any explicit
// Complete stops; otherwise all controllers must be ready before the next step.
inline EReadiness Combine(EReadiness Participant,EReadiness Observer)
{
    if(Participant==EReadiness::Failed||Observer==EReadiness::Failed)return EReadiness::Failed;
    if(Participant==EReadiness::Complete||Observer==EReadiness::Complete)return EReadiness::Complete;
    if(Participant==EReadiness::Waiting||Observer==EReadiness::Waiting)return EReadiness::Waiting;
    return Participant==EReadiness::None?Observer:Participant;
}
}
