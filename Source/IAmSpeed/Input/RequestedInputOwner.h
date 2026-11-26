#pragma once
#include "RequestedInputFrame.h"
#include "SameFrameInputOwner.h"

namespace Speed::Input::V2
{
struct FRequestedInputCutoff
{
    FLocalInputPollOrdinal Ordinal;
    std::uint64_t SourceGeneration=0;
};
// Construct the mutable implementation at the admitted worker boundary. This
// protocol alone does not admit a native world, participant, or physical write.
class IRequestedInputSource
{
public:
    virtual ~IRequestedInputSource()=default;
    virtual const std::shared_ptr<const FInputActionContract>& GetContract() const noexcept=0;
    virtual std::uint64_t GetSourceGeneration() const noexcept=0;
    virtual bool HasFiniteRealtimeFallback() const noexcept=0;
    virtual std::optional<FRequestedInputCutoff> Freeze(FFrameNumber)=0;
    // Data-only capability: caller must already own the native control cutoff.
    virtual std::optional<FRequestedInputCutoff> FreezeAtAdmittedNativeCutoff(FFrameNumber N){return Freeze(N);}
    // Optional data preflight, never a native admission or consumption receipt.
    virtual std::optional<FRequestedInputFrame> PreviewAtAdmittedNativeCutoff(FFrameNumber){return {};}
    virtual std::optional<FRequestedInputFrame> Select(const FRequestedInputCutoff&,FFrameNumber)=0;
    virtual bool CloseFrozen(const FRequestedInputCutoff&) noexcept=0;
    virtual std::optional<FRequestedInputFrame> ReadConfirmedForReplay(FFrameNumber) const=0;
    virtual bool Cancel() noexcept=0; // Data-source close, never host stop/join.
};
enum class ERequestedDigitalPolicy : std::uint8_t
{
    Unspecified,
    // Original ordered edges are available unchanged. Forecasts carry held
    // state only, never press edges. Explicitly cancel actions becoming inactive.
    HeldStateAndCancelInactive
};
struct FRequestedInputOwnerBinding
{
    std::uint64_t SourceGeneration=0;
    std::shared_ptr<const FInputActionContract> Contract;
    FInputProcessingPolicy Processing;
    ERequestedDigitalPolicy DigitalPolicy=ERequestedDigitalPolicy::Unspecified;
};
struct FRequestedInputConsumption
{
    std::shared_ptr<const FRequestedInputFrame> Request;
    FContractFingerprint BindingFingerprint;
    FActionValues BeforeProcessing{},Applied{};
    FRequestedInputCutoff Cutoff;
    // A game adapter consumes this explicit cancellation together with held
    // state. It must not manufacture Started edges for a forecast.
    std::uint32_t CancelDigitalMask=0;
};
class FRequestedInputOwner;
class FRequestedInputCompletion final
{
    friend class FRequestedInputOwner;
public:
    const std::shared_ptr<const FRequestedInputConsumption>& GetConsumed() const noexcept { return Consumed; }
    FFrameNumber GetFrame() const noexcept { return Consumed->Request->GetState().Frame; }
private:
    explicit FRequestedInputCompletion(std::shared_ptr<const FRequestedInputConsumption> C):Consumed(std::move(C)){}
    std::shared_ptr<const FRequestedInputConsumption> Consumed;
};
enum class ERequestedOwnerPhase : std::uint8_t { Idle,Frozen,Prepared,Installed,Stepping,Quarantined,Closed };

// A distinct requested-input owner route; the strict original producer route
// is untouched. No UObject, native admission, physics step, rollback mutation,
// registry publication, network send, or CanMove grant is performed here.
class FRequestedInputOwner final
{
public:
    class FToken
    {
        friend class FRequestedInputOwner;
        std::shared_ptr<const std::uint8_t> Identity;
        explicit FToken(std::shared_ptr<const std::uint8_t> I):Identity(std::move(I)){}
    public:
        FToken()=default;
        FToken(const FToken&)=delete;FToken& operator=(const FToken&)=delete;
        FToken(FToken&&) noexcept=default;FToken& operator=(FToken&&) noexcept=default;
    };
    static std::unique_ptr<FRequestedInputOwner> Create(std::unique_ptr<IRequestedInputSource> Source,
        FRequestedInputOwnerBinding Binding,FFrameNumber First)
    {
        if(FPresentationInputScope::IsActive())return {};
        try
        {
            if(!Valid(Source.get(),Binding)||First==UINT64_MAX)
            {if(Source)Source->Cancel();return {};}
            return std::unique_ptr<FRequestedInputOwner>(new FRequestedInputOwner(std::move(Source),std::move(Binding),First));
        }
        catch(...){if(Source)Source->Cancel();return {};}
    }
    FRequestedInputOwner(const FRequestedInputOwner&)=delete;
    FRequestedInputOwner& operator=(const FRequestedInputOwner&)=delete;

    // Freeze/drain before the world's rollback decision. The source remains
    // frozen while that decision is made; no input is selected or installed yet.
    bool ValidateSourceAtAdmittedCutoff(FFrameNumber N)
    {
        if(!OnOwner()||FPresentationInputScope::IsActive()||CallingSource||Phase!=ERequestedOwnerPhase::Idle
            ||N!=NextFrame||N==UINT64_MAX||PollOrdinal==UINT64_MAX)return false;
        try
        {
            CallingSource=true;const auto Candidate=Source->PreviewAtAdmittedNativeCutoff(N);CallingSource=false;
            return Phase==ERequestedOwnerPhase::Idle&&Candidate&&Candidate->GetContract()
                &&Candidate->GetContract()->GetFingerprint()==Binding.Contract->GetFingerprint()
                &&Candidate->GetState().Frame==N&&(Latest?!Candidate->GetState().Reset:Candidate->GetState().Reset)
                &&Continuous(*Candidate);
        }
        catch(...){CallingSource=false;return false;}
    }
    std::optional<FToken> FreezeFrame(FFrameNumber N,bool NativeCutoffAdmitted=false)
    {
        if(!OnOwner()||FPresentationInputScope::IsActive()||CallingSource||Phase!=ERequestedOwnerPhase::Idle
            ||N!=NextFrame||N==UINT64_MAX||PollOrdinal==UINT64_MAX)return {};
        try
        {
            auto Identity=std::make_shared<const std::uint8_t>(std::uint8_t{0});
            CallingSource=true;
            const auto Cutoff=NativeCutoffAdmitted?Source->FreezeAtAdmittedNativeCutoff(N):Source->Freeze(N);
            CallingSource=false;
            if(Phase!=ERequestedOwnerPhase::Idle||!Cutoff||Cutoff->SourceGeneration!=Binding.SourceGeneration
                ||Cutoff->Ordinal.Value!=PollOrdinal+1){Quarantine();return {};}
            ++PollOrdinal;Frozen=*Cutoff;TokenIdentity=std::move(Identity);Phase=ERequestedOwnerPhase::Frozen;
            return FToken(TokenIdentity);
        }
        catch(...){CallingSource=false;Quarantine();return {};}
    }
    bool PrepareFrozen(const FToken& Token)
    {
        if(!Allowed(Token,ERequestedOwnerPhase::Frozen))return false;
        try
        {
            CallingSource=true;
            auto Request=Source->Select(*Frozen,NextFrame);
            const bool Closed=Source->CloseFrozen(*Frozen);
            CallingSource=false;
            if(Phase!=ERequestedOwnerPhase::Frozen||!Request||!Closed
                ||Request->GetContract()->GetFingerprint()!=Binding.Contract->GetFingerprint()
                ||Request->GetState().Frame!=NextFrame||(!Latest&&!Request->GetState().Reset)
                ||(Latest&&Request->GetState().Reset)||!Continuous(*Request))return Fail();
            const auto& State=Request->GetState();
            const auto Before=Latest?Latest->GetConsumed()->Applied:FActionValues{};
            auto Applied=State.Values;
            if(!State.Reset)for(std::size_t I=0;I<ActionCount;++I)
            {
                const int Step=Binding.Processing.Step[I];if(!Step)continue;
                Applied[I]=static_cast<std::int16_t>(int(Before[I])+std::clamp(int(Applied[I])-int(Before[I]),-Step,Step));
            }
            const auto Previous=Latest?Latest->GetConsumed()->Request->GetState().ActiveMask:0;
            const auto Cancel=Request->GetOriginal()?0:Previous&~State.ActiveMask&BooleanMask;
            auto ImmutableRequest=std::make_shared<const FRequestedInputFrame>(std::move(*Request));
            Pending=std::make_shared<const FRequestedInputConsumption>(FRequestedInputConsumption{
                std::move(ImmutableRequest),Fingerprint,Before,Applied,*Frozen,Cancel});
            // Reserve the completion before any physical write. It is private
            // until CompleteStep succeeds; selection is not completion.
            PendingCompletion=std::shared_ptr<const FRequestedInputCompletion>(new FRequestedInputCompletion(Pending));
            Phase=ERequestedOwnerPhase::Prepared;return true;
        }
        catch(...){CallingSource=false;return Fail();}
    }
    bool Install(const FToken& Token) noexcept
    {if(!Allowed(Token,ERequestedOwnerPhase::Prepared))return false;Phase=ERequestedOwnerPhase::Installed;return true;}
    std::shared_ptr<const FRequestedInputConsumption> ReadInstalled(const FToken& Token) const noexcept
    {return Allowed(Token,ERequestedOwnerPhase::Installed)||Allowed(Token,ERequestedOwnerPhase::Stepping)?Pending:nullptr;}
    bool BeginStep(const FToken& Token,FFrameNumber N) noexcept
    {if(!Allowed(Token,ERequestedOwnerPhase::Installed)||N!=NextFrame)return false;Phase=ERequestedOwnerPhase::Stepping;return true;}
    bool CanComplete(const FToken& Token,FFrameNumber N) const noexcept
    {return Allowed(Token,ERequestedOwnerPhase::Stepping)&&N==NextFrame;}
    // Called only after actual successful physics and validation of every
    // participant. Allocation-free and callback-free commit, exact endpoint.
    bool CompleteStep(const FToken& Token,FFrameNumber N) noexcept
    {
        if(!CanComplete(Token,N))return false;
        Latest=PendingCompletion;History[N%HistoryCapacity]=Latest;
        if(Pending->Request->GetOriginal())LastOriginal=Pending->Request;
        ++NextFrame;ClearPending();Phase=ERequestedOwnerPhase::Idle;return true;
    }
    bool Abort(const FToken& Token) noexcept
    {if(!OnOwner()||FPresentationInputScope::IsActive()||CallingSource||!Matches(Token))return false;Quarantine();return true;}
    bool RetireAtBoundary() noexcept
    {
        if(!OnOwner()||FPresentationInputScope::IsActive()||CallingSource
            ||(Phase!=ERequestedOwnerPhase::Idle&&Phase!=ERequestedOwnerPhase::Quarantined))return false;
        CallingSource=true;const bool Closed=Source->Cancel();CallingSource=false;
        ClearPending();Latest.reset();LastOriginal.reset();History={};
        Phase=Closed?ERequestedOwnerPhase::Closed:ERequestedOwnerPhase::Quarantined;return Closed;
    }
    std::shared_ptr<const FRequestedInputCompletion> ReadLatest() const noexcept
    {return OnOwner()&&Phase!=ERequestedOwnerPhase::Quarantined&&Phase!=ERequestedOwnerPhase::Closed?Latest:nullptr;}
    std::shared_ptr<const FRequestedInputCompletion> ReadRetained(FFrameNumber N) const noexcept
    {
        if(!OnOwner()||Phase==ERequestedOwnerPhase::Closed||Phase==ERequestedOwnerPhase::Quarantined)return {};
        const auto& R=History[N%HistoryCapacity];return R&&R->GetFrame()==N?R:nullptr;
    }
    // Authority lookup is separate from the immutable record actually consumed
    // by the forward branch. A missing confirmed frame is never filled from
    // that branch's old prediction. This does not restore or execute a replay.
    std::optional<FRequestedInputFrame> ReadConfirmedForReplay(FFrameNumber N)
    {
        if(!OnOwner()||FPresentationInputScope::IsActive()||CallingSource||N>=NextFrame
            ||(Phase!=ERequestedOwnerPhase::Idle&&Phase!=ERequestedOwnerPhase::Frozen))return {};
        CallingSource=true;
        try
        {
            auto R=Source->ReadConfirmedForReplay(N);CallingSource=false;
            if(!R||R->GetState().Frame!=N||R->GetOrigin()!=ERequestedInputOrigin::ConfirmedNetwork
                ||R->GetContract()->GetFingerprint()!=Binding.Contract->GetFingerprint())return {};
            return R;
        }
        catch(...){CallingSource=false;return {};}
    }
    ERequestedOwnerPhase GetPhase() const noexcept {return OnOwner()?Phase:ERequestedOwnerPhase::Quarantined;}
    FFrameNumber GetNextFrame() const noexcept {return OnOwner()?NextFrame:UINT64_MAX;}
    FLocalInputPollOrdinal GetLocalPollOrdinal() const noexcept {return {OnOwner()?PollOrdinal:0};}
    const FContractFingerprint& GetBindingFingerprint() const noexcept {return Fingerprint;}
private:
    static bool Valid(const IRequestedInputSource* Source,const FRequestedInputOwnerBinding& B)
    {
        if(!Source||!B.Contract||!B.SourceGeneration||B.SourceGeneration!=Source->GetSourceGeneration()
            ||!Source->HasFiniteRealtimeFallback()||!Source->GetContract()
            ||Source->GetContract()->GetFingerprint()!=B.Contract->GetFingerprint()
            ||B.DigitalPolicy!=ERequestedDigitalPolicy::HeldStateAndCancelInactive)return false;
        for(std::size_t I=0;I<ActionCount;++I)
        {
            const auto* A=B.Contract->Find(static_cast<FActionId>(I));
            if(B.Processing.Step[I]&&(!A||A->Wiring!=EActionWiring::Wired||A->Type!=EActionType::Axis1D
                ||B.Processing.Step[I]>2*A->Quantization))return false;
        }
        return true;
    }
    FRequestedInputOwner(std::unique_ptr<IRequestedInputSource> S,FRequestedInputOwnerBinding B,FFrameNumber F)
        :Source(std::move(S)),Binding(std::move(B)),Owner(std::this_thread::get_id()),NextFrame(F)
    {
        Fingerprint=Binding.Contract->GetFingerprint();
        const auto Add=[&](std::uint64_t V){for(unsigned I=0;I<8;++I)Fingerprint.push_back(static_cast<std::uint8_t>(V>>(8*I)));};
        Add(0x5251455354494e31ULL);Add(FRequestedInputFrame::Version);Add(Binding.SourceGeneration);Add(std::uint64_t(Binding.DigitalPolicy));
        for(const auto SValue:Binding.Processing.Step)Add(SValue);
        for(const auto& A:Binding.Contract->GetDescription().Actions)
            if(A.Wiring==EActionWiring::Wired&&A.Type==EActionType::Bool)BooleanMask|=std::uint32_t{1}<<A.Id;
    }
    bool Continuous(const FRequestedInputFrame& R) const noexcept
    {
        if(!LastOriginal||!R.GetOriginal())return true;
        const auto& A=LastOriginal->GetOriginal()->GetData();const auto& B=R.GetOriginal()->GetData();
        if(A.Producer.Id!=B.Producer.Id||A.Producer.Kind!=B.Producer.Kind||A.StreamEpoch.Value!=B.StreamEpoch.Value
            ||A.DeviceGeneration.Value!=B.DeviceGeneration.Value||B.SourceSequence<A.SourceSequence)return false;
        if(B.SourceSequence==A.SourceSequence)return B.Values==A.Values&&B.ActiveMask==A.ActiveMask&&B.Transitions.empty();
        for(const auto& Edge:B.Transitions)if(Edge.Order.Sequence<=A.SourceSequence)return false;
        return true;
    }
    bool OnOwner() const noexcept {return Owner==std::this_thread::get_id();}
    bool Matches(const FToken& T) const noexcept {return TokenIdentity&&T.Identity==TokenIdentity;}
    bool Allowed(const FToken& T,ERequestedOwnerPhase P) const noexcept
    {return OnOwner()&&!FPresentationInputScope::IsActive()&&!CallingSource&&Phase==P&&Matches(T);}
    void ClearPending() noexcept {Pending.reset();PendingCompletion.reset();TokenIdentity.reset();Frozen.reset();}
    void Quarantine() noexcept
    {ClearPending();Latest.reset();Phase=ERequestedOwnerPhase::Quarantined;CallingSource=true;Source->Cancel();CallingSource=false;}
    bool Fail() noexcept {Quarantine();return false;}
    std::unique_ptr<IRequestedInputSource> Source;
    const FRequestedInputOwnerBinding Binding;
    const std::thread::id Owner;
    FContractFingerprint Fingerprint;
    FFrameNumber NextFrame=0;
    std::uint64_t PollOrdinal=0;
    std::uint32_t BooleanMask=0;
    bool CallingSource=false;
    ERequestedOwnerPhase Phase=ERequestedOwnerPhase::Idle;
    std::optional<FRequestedInputCutoff> Frozen;
    std::shared_ptr<const std::uint8_t> TokenIdentity;
    std::shared_ptr<const FRequestedInputConsumption> Pending;
    std::shared_ptr<const FRequestedInputCompletion> PendingCompletion,Latest;
    std::shared_ptr<const FRequestedInputFrame> LastOriginal;
    std::array<std::shared_ptr<const FRequestedInputCompletion>,HistoryCapacity> History{};
};
}
