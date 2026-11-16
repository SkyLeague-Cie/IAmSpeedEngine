#pragma once
#include "NetworkFramePolicy.h"
#include "IAmSpeed/Input/InputFrameV2.h"
#include <algorithm>
#include <atomic>
#include <memory>
#include <thread>

namespace Speed::Netcode
{
// Data admission only. The caller authenticates the peer and negotiates this
// opaque wire contract against the complete V2 contract AND processing policy.
// Neither a binding nor a selected value grants CanMove or physical access.
struct FNetworkInputBinding
{
    std::uint64_t Session=0, Epoch=0, Actor=0, Incarnation=0;
    std::array<std::uint8_t,32> ContractId{};
    bool operator==(const FNetworkInputBinding& R) const noexcept
    { return Session==R.Session&&Epoch==R.Epoch&&Actor==R.Actor&&Incarnation==R.Incarnation&&ContractId==R.ContractId; }
};

enum class ENetworkInputConsumer : std::uint8_t { DedicatedServer, Client };
enum class ENetworkInputCause : std::uint8_t { Original, Prediction, Fallback };
struct FNetworkOriginalReference
{
    Input::EProducerKind Kind=Input::EProducerKind::Device;
    std::uint64_t Producer=0, StreamEpoch=0, DeviceGeneration=0, SourceSequence=0;
    bool operator==(const FNetworkOriginalReference& R) const noexcept
    { return Kind==R.Kind&&Producer==R.Producer&&StreamEpoch==R.StreamEpoch&&DeviceGeneration==R.DeviceGeneration&&SourceSequence==R.SourceSequence; }
};

struct FNetworkInputEnvelope
{
    FNetworkInputBinding Binding;
    // Requested quantized values, before the game input filter. Never send an
    // already-applied target through a second physical processing pipeline.
    FNetworkInputState Requested;
    ENetworkInputCause Cause=ENetworkInputCause::Original;
    FNetworkOriginalReference Original;
    std::array<Input::V2::FActionTransition,Input::MaxEdges> Edges{};
    std::size_t EdgeCount=0;
};

struct FNetworkInputWorkerConfiguration
{
    FNetworkInputBinding Binding;
    ENetworkInputConsumer Consumer=ENetworkInputConsumer::Client;
    std::uint64_t AuthenticatedPeer=0, FirstFrame=0;
    // One admitted source generation per instance. Device/stream replacement
    // requires a new quiescent binding, not a silent generation substitution.
    FNetworkOriginalReference Original;
    std::uint64_t ReceivePastFrames=64, ReceiveAheadFrames=64;
    std::uint64_t ReplayFrames=80, PredictionFrames=63;
    FNetworkInputPredictionPolicy Prediction;
    bool AllowLocalFallback=false;
    FNetworkInputState LocalFallback;
};

enum class ENetworkInputAdmission : std::uint8_t
{ Queued, WrongThread, Closed, Full, WrongPeer, InvalidBinding, InvalidValue, InvalidProvenance, OutsideReceiveWindow };
enum class ENetworkInputWorkerStatus : std::uint8_t
{ Available, WrongThread, Closed, WrongFrame, NotPrepared, AlreadyPrepared, MissingBaseline, MissingAuthority, PredictionUnavailable, InvalidPrediction, ConflictingAuthority };
enum class ENetworkInputSelectionOrigin : std::uint8_t { Confirmed, Predicted, LocalFallback };
struct FNetworkInputSelection
{
    ENetworkInputWorkerStatus Status=ENetworkInputWorkerStatus::MissingAuthority;
    ENetworkInputSelectionOrigin Origin=ENetworkInputSelectionOrigin::Confirmed;
    FNetworkInputState Requested;
    // Present only for a received confirmed envelope. Cause still distinguishes
    // original versus server prediction/fallback. Local forecasts have no
    // original reference, source sequence, device generation or button edges.
    std::optional<FNetworkInputEnvelope> Confirmed;
    std::optional<std::uint64_t> EarlierFrame, LaterFrame;
};
struct FNetworkInputDrain
{
    ENetworkInputWorkerStatus Status=ENetworkInputWorkerStatus::Available;
    std::size_t Accepted=0, Duplicates=0, Conflicts=0, OutsideReceiveWindow=0;
};

// SPSC transport mailbox plus one thread-bound value selector. This is the
// source seam for a future physical-owner registration; it does not construct
// an IInputProducer, install a physical input, or pretend a forecast is V2 input.
template<std::size_t InboxCapacity=128,std::size_t HistoryCapacity=256>
class TNetworkInputWorkerSession final
    : public std::enable_shared_from_this<TNetworkInputWorkerSession<InboxCapacity,HistoryCapacity>>
{
    static_assert(InboxCapacity>0&&InboxCapacity<=256&&HistoryCapacity>0&&HistoryCapacity<=1024);
    static_assert(std::atomic<std::size_t>::is_always_lock_free&&std::atomic<std::uint64_t>::is_always_lock_free&&std::atomic<bool>::is_always_lock_free);
    struct FLimit { std::int16_t Min=0, Max=0; };
    struct FRegistration
    {
        FNetworkInputWorkerConfiguration Configuration;
        std::shared_ptr<const Input::V2::FInputActionContract> Contract;
        std::array<FLimit,FNetworkInputState::ActionCount> Limits{};
        std::uint32_t Wired=0, Axes=0;
        std::uint64_t Retention=0;
        bool ValuesValid(const FNetworkInputState& S) const noexcept
        {
            if(S.ActiveMask&~Wired)return false;
            for(std::size_t I=0;I<S.Values.size();++I)
                if(S.Values[I]<Limits[I].Min||S.Values[I]>Limits[I].Max)return false;
            return true;
        }
    };

public:
    class FIngress;
    class FWorker;
    static std::shared_ptr<TNetworkInputWorkerSession> Create(
        FNetworkInputWorkerConfiguration C,std::shared_ptr<const Input::V2::FInputActionContract> Contract)
    {
        const auto& B=C.Binding;
        if(!Contract||!B.Session||!B.Epoch||!B.Actor||!B.Incarnation||!C.AuthenticatedPeer||C.FirstFrame==UINT64_MAX
            ||std::all_of(B.ContractId.begin(),B.ContractId.end(),[](auto V){return V==0;})
            ||C.Consumer>ENetworkInputConsumer::Client||!C.Original.Producer||!C.Original.StreamEpoch||!C.Original.DeviceGeneration
            ||C.Original.Kind>Input::EProducerKind::AI||C.Original.SourceSequence
            ||!C.ReceivePastFrames||!C.ReceiveAheadFrames||!C.ReplayFrames
            ||C.PredictionFrames>=FNetworkInputPredictionPolicy::Window||C.LocalFallback.Reset)return {};
        const auto Retention=std::max({C.ReceivePastFrames,C.ReplayFrames,C.PredictionFrames});
        // At most Retention past slots plus current and Ahead-1 future slots.
        if(Retention>=HistoryCapacity||C.ReceiveAheadFrames>HistoryCapacity-Retention)return {};
        auto R=std::make_shared<FRegistration>();R->Configuration=C;R->Contract=std::move(Contract);R->Retention=Retention;
        for(const auto& A:R->Contract->GetDescription().Actions)
        {
            if(A.Id>=FNetworkInputState::ActionCount)return {};
            if(A.Wiring!=Input::V2::EActionWiring::Wired)continue;
            R->Wired|=std::uint32_t{1}<<A.Id;
            if(A.Type==Input::V2::EActionType::Axis1D)R->Axes|=std::uint32_t{1}<<A.Id;
            R->Limits[A.Id]={std::int16_t(A.Signed?-A.Quantization:0),A.Quantization};
        }
        if(!R->ValuesValid(C.LocalFallback))return {};
        return std::shared_ptr<TNetworkInputWorkerSession>(new TNetworkInputWorkerSession(std::move(R)));
    }
    std::unique_ptr<FIngress> BindIngressOnCurrentThread()
    {
        bool Expected=false;
        if(Closed.load(std::memory_order_acquire)||!IngressBound.compare_exchange_strong(Expected,true))return {};
        try { return std::unique_ptr<FIngress>(new FIngress(this->shared_from_this())); }
        catch(...) { Closed.store(true,std::memory_order_release);return {}; }
    }
    std::unique_ptr<FWorker> BindWorkerOnCurrentThread()
    {
        bool Expected=false;
        if(Closed.load(std::memory_order_acquire)||!WorkerBound.compare_exchange_strong(Expected,true))return {};
        try { return std::unique_ptr<FWorker>(new FWorker(this->shared_from_this())); }
        catch(...) { Closed.store(true,std::memory_order_release);return {}; }
    }
    TNetworkInputWorkerSession(const TNetworkInputWorkerSession&)=delete;
    TNetworkInputWorkerSession& operator=(const TNetworkInputWorkerSession&)=delete;
    bool IsAcquisitionClosed()const noexcept{return Closed.load(std::memory_order_acquire);}
    bool HasQueuedBaseline()const noexcept{return BaselineQueued.load(std::memory_order_acquire);}

    class FIngress final
    {
        friend class TNetworkInputWorkerSession;
        explicit FIngress(std::shared_ptr<TNetworkInputWorkerSession> S):Session(std::move(S)),Owner(std::this_thread::get_id()){}
    public:
        FIngress(const FIngress&)=delete;FIngress& operator=(const FIngress&)=delete;
        ~FIngress(){Session->Closed.store(true,std::memory_order_release);}
        // One-way projection of a real immutable V2 original. No frame-offset
        // or source-sequence override exists; forecast selections have no such
        // conversion. Transport authentication is still the caller's job.
        ENetworkInputAdmission TrySubmitOriginal(std::uint64_t AuthenticatedPeer,const Input::V2::FInputFrame& Original)
        {
            if(Owner!=std::this_thread::get_id())return ENetworkInputAdmission::WrongThread;
            if(Session->Closed.load(std::memory_order_acquire))return ENetworkInputAdmission::Closed;
            const auto& R=*Session->Registration;
            if(!Original.IsValidFor(*R.Contract))return ENetworkInputAdmission::InvalidValue;
            const auto& D=Original.GetData();FNetworkInputEnvelope E;
            E.Binding=R.Configuration.Binding;E.Requested={D.ConsumptionFrame,D.Values,D.ActiveMask,D.Reset};
            E.Original={D.Producer.Kind,D.Producer.Id,D.StreamEpoch.Value,D.DeviceGeneration.Value,D.SourceSequence};
            E.EdgeCount=D.Transitions.size();
            std::copy(D.Transitions.begin(),D.Transitions.end(),E.Edges.begin());
            return TrySubmit(AuthenticatedPeer,E);
        }
        ENetworkInputAdmission TrySubmit(std::uint64_t AuthenticatedPeer,const FNetworkInputEnvelope& E)
        {
            if(Owner!=std::this_thread::get_id())return ENetworkInputAdmission::WrongThread;
            if(Session->Closed.load(std::memory_order_acquire))return ENetworkInputAdmission::Closed;
            const auto& R=*Session->Registration;const auto& C=R.Configuration;
            if(AuthenticatedPeer!=C.AuthenticatedPeer)return ENetworkInputAdmission::WrongPeer;
            if(!(E.Binding==C.Binding))return ENetworkInputAdmission::InvalidBinding;
            if(E.Cause>ENetworkInputCause::Fallback||E.EdgeCount>Input::MaxEdges
                ||(C.Consumer==ENetworkInputConsumer::DedicatedServer&&E.Cause!=ENetworkInputCause::Original))return ENetworkInputAdmission::InvalidProvenance;
            if(E.Requested.Frame<C.FirstFrame||E.Requested.Frame==UINT64_MAX||!R.ValuesValid(E.Requested)
                ||(E.Requested.Reset&&E.Requested.Frame!=C.FirstFrame))return ENetworkInputAdmission::InvalidValue;
            if(!Session->InReceiveWindow(E.Requested.Frame,Session->InputCursor.load(std::memory_order_acquire)))return ENetworkInputAdmission::OutsideReceiveWindow;
            if(E.Cause==ENetworkInputCause::Original)
            {
                auto Expected=C.Original;Expected.SourceSequence=E.Original.SourceSequence;
                if(!E.Original.SourceSequence||!(E.Original==Expected))return ENetworkInputAdmission::InvalidProvenance;
                // Full current V2 validation, including all ordered edges. Any
                // allocation here is on the ingress thread, never the worker.
                Input::V2::FInputFrameData D;
                D.SourceSequence=E.Original.SourceSequence;D.ConsumptionFrame=E.Requested.Frame;
                D.Producer={E.Original.Kind,E.Original.Producer};D.StreamEpoch={E.Original.StreamEpoch};D.DeviceGeneration={E.Original.DeviceGeneration};
                D.Values=E.Requested.Values;D.ActiveMask=E.Requested.ActiveMask;D.Reset=E.Requested.Reset;
                D.Transitions.assign(E.Edges.begin(),E.Edges.begin()+E.EdgeCount);
                if(!Input::V2::FInputFrame(R.Contract,std::move(D)).IsValidFor(*R.Contract))return ENetworkInputAdmission::InvalidValue;
            }
            else if(!(E.Original==FNetworkOriginalReference{})||E.EdgeCount)return ENetworkInputAdmission::InvalidProvenance;
            const auto WriteIndex=Session->Head.load(std::memory_order_relaxed),Next=(WriteIndex+1)%QueueSlots;
            if(Next==Session->Tail.load(std::memory_order_acquire))return ENetworkInputAdmission::Full;
            Session->Inbox[WriteIndex]=E;
            if(Session->Closed.load(std::memory_order_acquire))return ENetworkInputAdmission::Closed;
            Session->Head.store(Next,std::memory_order_release);
            if(E.Requested.Frame==C.FirstFrame)Session->BaselineQueued.store(true,std::memory_order_release);
            return ENetworkInputAdmission::Queued;
        }
        bool Close() noexcept
        {if(Owner!=std::this_thread::get_id())return false;Session->Closed.store(true,std::memory_order_release);return true;}
    private:
        const std::shared_ptr<TNetworkInputWorkerSession> Session;
        const std::thread::id Owner;
    };

    class FWorker final
    {
        friend class TNetworkInputWorkerSession;
        explicit FWorker(std::shared_ptr<TNetworkInputWorkerSession> S):Session(std::move(S)),Owner(std::this_thread::get_id()),NextFrame(Session->Registration->Configuration.FirstFrame){}
    public:
        FWorker(const FWorker&)=delete;FWorker& operator=(const FWorker&)=delete;
        ~FWorker(){Session->Closed.store(true,std::memory_order_release);}
        // Call at the input boundary, before rollback decisions. Captures one
        // queue cutoff; arrivals after it remain for the next BeginFrame.
        FNetworkInputDrain BeginFrame(std::uint64_t Frame,bool FinishAdmittedCutoff=false) noexcept
        {
            FNetworkInputDrain R;
            R.Status=Check(FinishAdmittedCutoff);if(R.Status!=ENetworkInputWorkerStatus::Available)return R;
            if(Prepared){R.Status=ENetworkInputWorkerStatus::AlreadyPrepared;return R;}
            if(Frame!=NextFrame||Frame==UINT64_MAX){R.Status=ENetworkInputWorkerStatus::WrongFrame;return R;}
            FinishCurrentCutoff=FinishAdmittedCutoff;
            const auto Cutoff=Session->Head.load(std::memory_order_acquire);
            auto ReadIndex=Session->Tail.load(std::memory_order_relaxed);
            while(ReadIndex!=Cutoff)
            {
                const auto E=Session->Inbox[ReadIndex];
                ReadIndex=(ReadIndex+1)%QueueSlots;Session->Tail.store(ReadIndex,std::memory_order_release);
                if(!Session->InReceiveWindow(E.Requested.Frame,Frame)){++R.OutsideReceiveWindow;continue;}
                auto& Slot=History[E.Requested.Frame%HistoryCapacity];
                if(Slot&&Slot->Requested.Frame==E.Requested.Frame)
                {
                    if(Same(*Slot,E))++R.Duplicates;
                    else {++R.Conflicts;Conflict=true;}
                    continue;
                }
                const auto Keep=Session->Registration->Retention;
                const auto Floor=Frame>Keep?Frame-Keep:0;
                if(Slot&&Slot->Requested.Frame>=Floor){++R.Conflicts;Conflict=true;continue;}
                Slot=E;++R.Accepted;
            }
            Prepared=true;PreparedPreview.reset();
            if(Conflict)R.Status=ENetworkInputWorkerStatus::ConflictingAuthority;
            return R;
        }
        bool HasPreparedBaseline()const noexcept
        {return Owner==std::this_thread::get_id()&&!OwnerClosed&&Prepared&&Find(Session->Registration->Configuration.FirstFrame);}
        bool ArmPreparedCutoff(std::uint64_t N,bool FinishAdmitted)noexcept
        {if(Owner!=std::this_thread::get_id()||OwnerClosed||!Prepared||NextFrame!=N||(!FinishAdmitted&&Session->Closed.load(std::memory_order_acquire)))return false;FinishCurrentCutoff=FinishAdmitted;return true;}
        // Evaluate only the immutable data candidate. Do not advance the input
        // cursor/last-selected frame or expose it as a completed selection.
        // Retain this exact candidate so a pure policy is not evaluated twice.
        FNetworkInputSelection PreviewPrepared(std::uint64_t Frame) noexcept
        {
            const auto Status=Check();if(Status!=ENetworkInputWorkerStatus::Available)return Failed(Status);
            if(!Prepared)return Failed(ENetworkInputWorkerStatus::NotPrepared);
            if(Frame!=NextFrame)return Failed(ENetworkInputWorkerStatus::WrongFrame);
            if(!HasSelected&&!Find(Session->Registration->Configuration.FirstFrame))return Failed(ENetworkInputWorkerStatus::MissingBaseline);
            if(!PreparedPreview)PreparedPreview=Select(Frame,false);
            return *PreparedPreview;
        }
        FNetworkInputSelection SelectPrepared(std::uint64_t Frame) noexcept
        {
            auto Status=Check();if(Status!=ENetworkInputWorkerStatus::Available)return Failed(Status);
            if(!Prepared)return Failed(ENetworkInputWorkerStatus::NotPrepared);
            if(Frame!=NextFrame)return Failed(ENetworkInputWorkerStatus::WrongFrame);
            Prepared=false;
            const auto First=Session->Registration->Configuration.FirstFrame;
            if(!HasSelected&&!Find(First)){FinishCurrentCutoff=false;return Failed(ENetworkInputWorkerStatus::MissingBaseline);}
            auto Result=PreparedPreview?*PreparedPreview:Select(Frame,false);PreparedPreview.reset();
            if(Result.Status==ENetworkInputWorkerStatus::Available)
            {
                HasSelected=true;LastSelected=Frame;++NextFrame;
                Session->InputCursor.store(NextFrame,std::memory_order_release);
            }
            FinishCurrentCutoff=false;return Result;
        }
        // A replay read uses retained confirmed inputs or an explicitly allowed
        // NEW policy evaluation. It never substitutes a stored old forecast
        // for a confirmed input that was retired or never received.
        FNetworkInputSelection ReadForReplay(std::uint64_t Frame,bool RequireConfirmed=true) const noexcept
        {
            const auto Status=Check();if(Status!=ENetworkInputWorkerStatus::Available)return Failed(Status);
            const auto& C=Session->Registration->Configuration;
            if(!HasSelected||Frame<C.FirstFrame||Frame>LastSelected||LastSelected-Frame>=C.ReplayFrames)return Failed(ENetworkInputWorkerStatus::WrongFrame);
            return Select(Frame,RequireConfirmed);
        }
        bool Close() noexcept
        {if(Owner!=std::this_thread::get_id())return false;OwnerClosed=true;FinishCurrentCutoff=false;Session->Closed.store(true,std::memory_order_release);Prepared=false;PreparedPreview.reset();return true;}
    private:
        static FNetworkInputSelection Failed(ENetworkInputWorkerStatus S) noexcept
        {FNetworkInputSelection R;R.Status=S;return R;}
        ENetworkInputWorkerStatus Check(bool AdmitClosedIngress=false) const noexcept
        {
            if(Owner!=std::this_thread::get_id())return ENetworkInputWorkerStatus::WrongThread;
            if(OwnerClosed||(Session->Closed.load(std::memory_order_acquire)&&!FinishCurrentCutoff&&!AdmitClosedIngress))return ENetworkInputWorkerStatus::Closed;
            return Conflict?ENetworkInputWorkerStatus::ConflictingAuthority:ENetworkInputWorkerStatus::Available;
        }
        const FNetworkInputEnvelope* Find(std::uint64_t Frame) const noexcept
        {
            const auto Keep=Session->Registration->Retention;
            if(NextFrame>Keep&&Frame<NextFrame-Keep)return nullptr;
            const auto& S=History[Frame%HistoryCapacity];return S&&S->Requested.Frame==Frame?&*S:nullptr;
        }
        FNetworkInputSelection Select(std::uint64_t Frame,bool RequireConfirmed) const noexcept
        {
            const auto& R=*Session->Registration;const auto& C=R.Configuration;
            if(const auto* Exact=Find(Frame))
            {
                FNetworkInputSelection Out;Out.Status=ENetworkInputWorkerStatus::Available;
                Out.Requested=Exact->Requested;Out.Confirmed=*Exact;return Out;
            }
            if(RequireConfirmed)return Failed(ENetworkInputWorkerStatus::MissingAuthority);
            const FNetworkInputEnvelope* Earlier=nullptr;
            for(std::uint64_t Offset=1;Offset<=C.PredictionFrames&&Offset<=Frame;++Offset)
                if((Earlier=Find(Frame-Offset)))break;
            if(Earlier)
            {
                const FNetworkInputEnvelope* Later=nullptr;
                for(std::uint64_t Offset=1;Offset<C.ReceiveAheadFrames&&Offset<UINT64_MAX-Frame;++Offset)
                    if((Later=Find(Frame+Offset)))break;
                FNetworkInputSelection Out;Out.Origin=ENetworkInputSelectionOrigin::Predicted;
                Out.EarlierFrame=Earlier->Requested.Frame;if(Later)Out.LaterFrame=Later->Requested.Frame;
                Out.Requested=C.Prediction.Evaluate({Frame,Earlier->Requested,Later?&Later->Requested:nullptr,R.Axes});
                Out.Status=R.ValuesValid(Out.Requested)?ENetworkInputWorkerStatus::Available:ENetworkInputWorkerStatus::InvalidPrediction;
                return Out;
            }
            if(!C.AllowLocalFallback)return Failed(ENetworkInputWorkerStatus::PredictionUnavailable);
            FNetworkInputSelection Out;Out.Status=ENetworkInputWorkerStatus::Available;Out.Origin=ENetworkInputSelectionOrigin::LocalFallback;
            Out.Requested=C.LocalFallback;Out.Requested.Frame=Frame;return Out;
        }
        static bool Same(const FNetworkInputEnvelope& A,const FNetworkInputEnvelope& B) noexcept
        {
            if(!(A.Binding==B.Binding)||A.Cause!=B.Cause||!(A.Original==B.Original)||A.EdgeCount!=B.EdgeCount
                ||A.Requested.Frame!=B.Requested.Frame||A.Requested.Values!=B.Requested.Values||A.Requested.ActiveMask!=B.Requested.ActiveMask||A.Requested.Reset!=B.Requested.Reset)return false;
            for(std::size_t I=0;I<A.EdgeCount;++I)
            {
                const auto& X=A.Edges[I];const auto& Y=B.Edges[I];
                if(X.Action!=Y.Action||X.State!=Y.State||X.ValueAtTransition!=Y.ValueAtTransition||X.Order.Sequence!=Y.Order.Sequence||X.Order.WithinSequence!=Y.Order.WithinSequence)return false;
            }
            return true;
        }
        const std::shared_ptr<TNetworkInputWorkerSession> Session;
        const std::thread::id Owner;
        std::uint64_t NextFrame=0,LastSelected=0;
        bool Prepared=false,HasSelected=false,Conflict=false;
        bool FinishCurrentCutoff=false,OwnerClosed=false;
        std::optional<FNetworkInputSelection> PreparedPreview;
        std::array<std::optional<FNetworkInputEnvelope>,HistoryCapacity> History{};
    };
private:
    explicit TNetworkInputWorkerSession(std::shared_ptr<const FRegistration> R):Registration(std::move(R)),InputCursor(Registration->Configuration.FirstFrame){}
    bool InReceiveWindow(std::uint64_t Frame,std::uint64_t Cursor) const noexcept
    {
        const auto& C=Registration->Configuration;
        return Frame>=C.FirstFrame&&Frame!=UINT64_MAX&&(Frame<Cursor?Cursor-Frame<=C.ReceivePastFrames:Frame-Cursor<C.ReceiveAheadFrames);
    }
    static constexpr std::size_t QueueSlots=InboxCapacity+1;
    const std::shared_ptr<const FRegistration> Registration;
    std::array<FNetworkInputEnvelope,QueueSlots> Inbox{};
    std::atomic<std::size_t> Head{0},Tail{0};
    std::atomic<std::uint64_t> InputCursor;
    std::atomic<bool> Closed{false},IngressBound{false},WorkerBound{false};
    std::atomic<bool> BaselineQueued{false};
};
}
