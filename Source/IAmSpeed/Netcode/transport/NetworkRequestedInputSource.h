#pragma once
#include "NetworkInputWorkerSession.h"
#include "RequestedSourceIdentity.h"
#include "IAmSpeed/Input/RequestedInputOwner.h"

namespace Speed::Netcode
{
// Preconfigured service, referenced by an inert ID by the future native
// registry. Binding its unique mutable source is worker-only; this service
// does not itself prove native admission or own a socket/thread lifetime.
class FNetworkRequestedInputService final
{
    using FSession=TNetworkInputWorkerSession<>;
public:
    struct FSourceBindAttempt
    {
        std::unique_ptr<Input::V2::IRequestedInputSource> Source;
        std::shared_ptr<const FRequestedSourceIdentity> Identity;
        bool WorkerSelectionBound=false,SourceConstructed=false,SourceRetired=false;
        bool BaselineReady=false;
        ERequestedSourceBindFailure Failure=ERequestedSourceBindFailure::None;
    };
    static std::shared_ptr<FNetworkRequestedInputService> Create(FNetworkInputWorkerConfiguration C,
        std::shared_ptr<const Input::V2::FInputActionContract> Contract)
    {
        if(!C.AllowLocalFallback)return {};
        auto Session=FSession::Create(C,Contract);if(!Session)return {};
        return std::shared_ptr<FNetworkRequestedInputService>(new FNetworkRequestedInputService(
            std::move(C),std::move(Contract),std::move(Session)));
    }
    std::unique_ptr<FSession::FIngress> BindIngressOnCurrentThread() {return Session->BindIngressOnCurrentThread();}
    const FNetworkInputWorkerConfiguration& GetConfiguration() const noexcept {return Configuration;}
    const std::shared_ptr<const Input::V2::FInputActionContract>& GetContract() const noexcept {return Contract;}
    std::shared_ptr<const void> ReadServiceIdentity()const noexcept{return ServiceIdentity;}
    ERequestedSourceBindingPhase ReadBindingPhase()const noexcept{return BindingPhase->load(std::memory_order_acquire);}
    bool IsAcquisitionClosed()const noexcept{return Session->IsAcquisitionClosed();}

    static std::optional<Input::V2::FRequestedInputFrame> ConvertSelection(const FNetworkInputSelection& Selection,
        const FNetworkInputBinding& Expected,std::shared_ptr<const Input::V2::FInputActionContract> Contract)
    {
        using namespace Input::V2;
        if(Selection.Status!=ENetworkInputWorkerStatus::Available||Selection.Origin>ENetworkInputSelectionOrigin::LocalFallback
            ||(Selection.Origin==ENetworkInputSelectionOrigin::Confirmed)!=Selection.Confirmed.has_value())return {};
        ERequestedInputOrigin Origin=ERequestedInputOrigin::LocalPrediction;
        std::optional<ERequestedAuthorityCause> Cause;
        std::optional<Input::V2::FInputFrame> Original;
        const auto& State=Selection.Requested;
        if(Selection.Confirmed)
        {
            const auto& E=*Selection.Confirmed;
            if(!(E.Binding==Expected)||E.Cause>ENetworkInputCause::Fallback||E.EdgeCount>Input::MaxEdges
                ||E.Requested.Frame!=State.Frame||E.Requested.Values!=State.Values
                ||E.Requested.ActiveMask!=State.ActiveMask||E.Requested.Reset!=State.Reset)return {};
            Origin=ERequestedInputOrigin::ConfirmedNetwork;
            Cause=static_cast<ERequestedAuthorityCause>(E.Cause);
            if(E.Cause==ENetworkInputCause::Original)
            {
                // Reconstruct only the actual received V2 evidence. Every
                // field is preserved; no forecast or local ordinal is used.
                FInputFrameData D;D.ConsumptionFrame=State.Frame;D.Values=State.Values;
                D.ActiveMask=State.ActiveMask;D.Reset=State.Reset;
                D.Producer={E.Original.Kind,E.Original.Producer};D.StreamEpoch={E.Original.StreamEpoch};
                D.DeviceGeneration={E.Original.DeviceGeneration};D.SourceSequence=E.Original.SourceSequence;
                D.Transitions.assign(E.Edges.begin(),E.Edges.begin()+E.EdgeCount);
                Original.emplace(Contract,std::move(D));
            }
            else if(!(E.Original==FNetworkOriginalReference{})||E.EdgeCount)return {};
        }
        else if(Selection.Origin==ENetworkInputSelectionOrigin::LocalFallback)Origin=ERequestedInputOrigin::LocalFallback;
        return FRequestedInputFrame::FromNetwork(std::move(Contract),{State.Frame,State.Values,State.ActiveMask,State.Reset},Origin,Cause,std::move(Original));
    }
    std::unique_ptr<Input::V2::IRequestedInputSource> BindSourceOnCurrentThread(std::uint64_t SourceGeneration)
    {
        auto Attempt=BindSourceWithIdentityOnCurrentThread(SourceGeneration);return std::move(Attempt.Source);
    }
    FSourceBindAttempt BindSourceWithIdentityOnCurrentThread(std::uint64_t SourceGeneration,bool RequireBaseline=false)
    {
        FSourceBindAttempt A;if(!SourceGeneration){A.Failure=ERequestedSourceBindFailure::InvalidGeneration;return A;}
        if(RequireBaseline&&Session->IsAcquisitionClosed()){A.Failure=ERequestedSourceBindFailure::AcquisitionClosed;return A;}
        if(RequireBaseline&&!Session->HasQueuedBaseline()){A.Failure=ERequestedSourceBindFailure::MissingBaseline;return A;}
        auto Expected=ERequestedSourceBindingPhase::Fresh;
        if(!BindingPhase->compare_exchange_strong(Expected,ERequestedSourceBindingPhase::Binding,std::memory_order_acq_rel)){A.Failure=ERequestedSourceBindFailure::AlreadyBound;return A;}
        try
        {
            const auto Instance=NextSourceInstance();if(!Instance){A.Failure=ERequestedSourceBindFailure::IdentifierExhausted;BindingPhase->store(ERequestedSourceBindingPhase::Closed,std::memory_order_release);return A;}
            auto Worker=Session->BindWorkerOnCurrentThread();if(!Worker){A.Failure=ERequestedSourceBindFailure::WorkerUnavailable;BindingPhase->store(ERequestedSourceBindingPhase::Closed,std::memory_order_release);return A;}
            A.WorkerSelectionBound=true;
            if(RequireBaseline)
            {
                const auto Initial=Worker->BeginFrame(Configuration.FirstFrame);
                if(Initial.Status!=ENetworkInputWorkerStatus::Available||!Worker->HasPreparedBaseline())
                {A.Failure=ERequestedSourceBindFailure::InitialDataInvalid;Worker->Close();BindingPhase->store(ERequestedSourceBindingPhase::Closed,std::memory_order_release);return A;}
                A.BaselineReady=true;
            }
            std::unique_ptr<FSource> Actual(new FSource(Configuration,Contract,std::move(Worker),SourceGeneration,BindingPhase,RequireBaseline));A.SourceConstructed=true;
            A.Identity=std::shared_ptr<const FRequestedSourceIdentity>(new FRequestedSourceIdentity(ServiceIdentity,static_cast<Input::V2::IRequestedInputSource*>(Actual.get()),SourceGeneration,Instance,BindingPhase));
            A.Source=std::move(Actual);BindingPhase->store(ERequestedSourceBindingPhase::Bound,std::memory_order_release);return A;
        }
        catch(...)
        {BindingPhase->store(ERequestedSourceBindingPhase::Closed,std::memory_order_release);A.Source.reset();A.Identity.reset();A.SourceRetired=A.SourceConstructed;A.Failure=ERequestedSourceBindFailure::AllocationFailure;return A;}
    }
private:
    static std::uint64_t NextSourceInstance()noexcept
    {
        static std::atomic<std::uint64_t> Next{1};auto N=Next.load(std::memory_order_relaxed);
        for(unsigned I=0;I<256;++I){if(N==UINT64_MAX)return 0;if(Next.compare_exchange_strong(N,N+1,std::memory_order_relaxed))return N;}return 0;
    }
    class FSource final : public Input::V2::IRequestedInputSource
    {
    public:
        FSource(FNetworkInputWorkerConfiguration C,std::shared_ptr<const Input::V2::FInputActionContract> InContract,
            std::unique_ptr<FSession::FWorker> W,std::uint64_t G,std::shared_ptr<std::atomic<ERequestedSourceBindingPhase>> P,bool InitialPrepared)
            :Configuration(std::move(C)),Contract(std::move(InContract)),Worker(std::move(W)),Generation(G),Owner(std::this_thread::get_id()),BindingPhase(std::move(P)),PreparedInitial(InitialPrepared){}
        ~FSource()override {if(!OnOwner())std::terminate();Cancel();}
        const std::shared_ptr<const Input::V2::FInputActionContract>& GetContract() const noexcept override {return Contract;}
        std::uint64_t GetSourceGeneration() const noexcept override {return Generation;}
        bool HasFiniteRealtimeFallback() const noexcept override {return Configuration.AllowLocalFallback;}
        std::optional<Input::V2::FRequestedInputCutoff> Freeze(Input::FFrameNumber N) override
        {return FreezeImpl(N,false);}
        std::optional<Input::V2::FRequestedInputCutoff> FreezeAtAdmittedNativeCutoff(Input::FFrameNumber N) override
        {return FreezeImpl(N,true);}
        std::optional<Input::V2::FRequestedInputFrame> PreviewAtAdmittedNativeCutoff(Input::FFrameNumber N) override
        {
            if(!OnOwner()||Closed||Frozen)return {};
            if(NativePreviewFrame&&*NativePreviewFrame!=N){Cancel();return {};}
            if(!NativePreviewFrame)
            {
                if(PreparedInitial)
                {
                    if(N!=Configuration.FirstFrame||!Worker->ArmPreparedCutoff(N,true)){Cancel();return {};}
                    PreparedInitial=false;
                }
                else if(Worker->BeginFrame(N,true).Status!=ENetworkInputWorkerStatus::Available){Cancel();return {};}
            }
            auto Candidate=ConvertSelection(Worker->PreviewPrepared(N),Configuration.Binding,Contract);
            if(!Candidate){Cancel();return {};}
            NativePreviewFrame=N;return Candidate;
        }
        std::optional<Input::V2::FRequestedInputCutoff> FreezeImpl(Input::FFrameNumber N,bool Admitted)
        {
            if(!OnOwner()||Closed||Frozen||Ordinal==UINT64_MAX)return {};
            if(NativePreviewFrame)
            {
                if(!Admitted||*NativePreviewFrame!=N||!Worker->ArmPreparedCutoff(N,true)){Cancel();return {};}
            }
            else if(PreparedInitial)
            {
                if(N!=Configuration.FirstFrame||!Worker->ArmPreparedCutoff(N,Admitted)){Cancel();return {};}
                PreparedInitial=false;
            }
            else if(Worker->BeginFrame(N,Admitted).Status!=ENetworkInputWorkerStatus::Available){Cancel();return {};}
            Frozen=Input::V2::FRequestedInputCutoff{{++Ordinal},Generation};Frame=N;Selected=false;return Frozen;
        }
        std::optional<Input::V2::FRequestedInputFrame> Select(const Input::V2::FRequestedInputCutoff& Cutoff,Input::FFrameNumber N) override
        {
            if(!OnOwner()||Closed||!Matches(Cutoff)||N!=Frame||Selected)return {};
            Selected=true;return ConvertSelection(Worker->SelectPrepared(N),Configuration.Binding,Contract);
        }
        bool CloseFrozen(const Input::V2::FRequestedInputCutoff& Cutoff) noexcept override
        {if(!OnOwner()||Closed||!Matches(Cutoff)||!Selected)return false;Frozen.reset();NativePreviewFrame.reset();return true;}
        std::optional<Input::V2::FRequestedInputFrame> ReadConfirmedForReplay(Input::FFrameNumber N) const override
        {return OnOwner()&&!Closed?ConvertSelection(Worker->ReadForReplay(N,true),Configuration.Binding,Contract):std::nullopt;}
        bool Cancel() noexcept override
        {if(!OnOwner())return false;Closed=true;Frozen.reset();NativePreviewFrame.reset();BindingPhase->store(ERequestedSourceBindingPhase::Closed,std::memory_order_release);return Worker->Close();}
    private:
        bool OnOwner() const noexcept {return Owner==std::this_thread::get_id();}
        bool Matches(const Input::V2::FRequestedInputCutoff& C) const noexcept
        {return Frozen&&Frozen->Ordinal.Value==C.Ordinal.Value&&Frozen->SourceGeneration==C.SourceGeneration;}
        const FNetworkInputWorkerConfiguration Configuration;
        const std::shared_ptr<const Input::V2::FInputActionContract> Contract;
        std::unique_ptr<FSession::FWorker> Worker;
        const std::uint64_t Generation;
        const std::thread::id Owner;
        const std::shared_ptr<std::atomic<ERequestedSourceBindingPhase>> BindingPhase;
        std::optional<Input::V2::FRequestedInputCutoff> Frozen;
        Input::FFrameNumber Frame=0;
        std::uint64_t Ordinal=0;
        bool Selected=false,Closed=false;
        bool PreparedInitial=false;
        std::optional<Input::FFrameNumber> NativePreviewFrame;
    };
    FNetworkRequestedInputService(FNetworkInputWorkerConfiguration C,std::shared_ptr<const Input::V2::FInputActionContract> InContract,
        std::shared_ptr<FSession> S):Configuration(std::move(C)),Contract(std::move(InContract)),Session(std::move(S)){}
    const FNetworkInputWorkerConfiguration Configuration;
    const std::shared_ptr<const Input::V2::FInputActionContract> Contract;
    const std::shared_ptr<FSession> Session;
    const std::shared_ptr<const void> ServiceIdentity=std::make_shared<const std::uint8_t>(std::uint8_t{0});
    const std::shared_ptr<std::atomic<ERequestedSourceBindingPhase>> BindingPhase=std::make_shared<std::atomic<ERequestedSourceBindingPhase>>(ERequestedSourceBindingPhase::Fresh);
};
}
