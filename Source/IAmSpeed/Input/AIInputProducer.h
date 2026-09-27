#pragma once
#include "InputProducerV2.h"
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>

namespace Speed::Input::V2
{
// A decision mailbox, not physical state. The timer can publish complete values
// and pulses; only the physical owner turns them into canonical input frames.
class FAIInputCommands final
{
public:
    using FClock=std::chrono::steady_clock;
    struct FDecision { FActionValues Values{}; std::vector<FActionId> Pulses; FClock::time_point Time{}; bool Present=false; };
    explicit FAIInputCommands(std::shared_ptr<const FInputActionContract> C,
        std::chrono::milliseconds MaxAge=std::chrono::milliseconds(500)) : Contract(std::move(C)), Age(MaxAge) {}
    bool Publish(const FActionValues& Values,const std::vector<FActionId>& Pulses={},FClock::time_point Now=FClock::now())
    {
        if (FPresentationInputScope::IsActive() || !Contract || Age.count()<=0) return false;
        std::lock_guard<std::mutex> Lock(Gate);
        if (Closed || PollEvaluator || Pulses.size()+Decision.Pulses.size()>(MaxEdges-ActionCount)/4) return false;
        for (FActionId I=0; I<ActionCount; ++I)
        {
            const auto* A=Contract->Find(I);
            if ((A && !A->Accepts(Values[I])) || (!A && Values[I])) return false;
        }
        for (const auto Id:Pulses)
        { const auto* A=Contract->Find(Id); if (!A || A->Type!=EActionType::Bool || Values[Id]) return false; }
        Decision.Values=Values; Decision.Time=Now; Decision.Present=true;
        Decision.Pulses.insert(Decision.Pulses.end(),Pulses.begin(),Pulses.end()); return true;
    }
    FDecision Take(FClock::time_point Now=FClock::now())
    {
        std::lock_guard<std::mutex> Lock(Gate);
        if (Closed || !Decision.Present || Now<Decision.Time || Now-Decision.Time>Age)
        { Decision={}; return {}; }
        auto Result=Decision; Decision.Pulses.clear(); return Result;
    }
    bool SetPollEvaluator(std::function<FDecision(FFrameNumber)> Evaluate)
    {
        std::lock_guard<std::mutex> Lock(Gate);
        if (Closed || EvaluationStarted || !Evaluate) return false;
        PollEvaluator=std::move(Evaluate); Decision={}; return true;
    }
    bool HasPollEvaluator() const
    {
        std::lock_guard<std::mutex> Lock(Gate);
        return bool(PollEvaluator);
    }
    FDecision EvaluateAtPoll(FFrameNumber N)
    {
        std::function<FDecision(FFrameNumber)> Evaluate;
        {
            std::lock_guard<std::mutex> Lock(Gate);
            if (Closed) return {};
            Evaluate=PollEvaluator;
            if (Evaluate) { EvaluationStarted=true; ++ActiveEvaluations; }
        }
        if (!Evaluate) return Take();
        try
        {
            auto Result=Evaluate(N);
            { std::lock_guard<std::mutex> Lock(Gate); --ActiveEvaluations; Idle.notify_all(); }
            return Result;
        }
        catch (...)
        {
            { std::lock_guard<std::mutex> Lock(Gate); --ActiveEvaluations; Idle.notify_all(); }
            throw;
        }
    }
    void Invalidate() { std::lock_guard<std::mutex> Lock(Gate); Decision={}; }
    void Close() { std::lock_guard<std::mutex> Lock(Gate); Closed=true; Decision={}; }
    bool CloseAndWait(std::chrono::milliseconds Timeout)
    {
        std::unique_lock<std::mutex> Lock(Gate);
        Closed=true; Decision={};
        return Idle.wait_for(Lock,Timeout,[&] { return ActiveEvaluations==0; });
    }
    const std::shared_ptr<const FInputActionContract>& GetContract() const { return Contract; }
private:
    std::shared_ptr<const FInputActionContract> Contract;
    const std::chrono::milliseconds Age;
    mutable std::mutex Gate;
    std::condition_variable Idle;
    FDecision Decision;
    std::function<FDecision(FFrameNumber)> PollEvaluator;
    unsigned ActiveEvaluations=0;
    bool Closed=false;
    bool EvaluationStarted=false;
};

class FAIInputProducer final : public IInputProducer, public IInputProducerPollFence
{
public:
	IInputProducerPollFence* PollFence() noexcept override { return this; }
	bool SupportsPollFence() const noexcept override { return true; }
    static std::unique_ptr<FAIInputProducer> Create(std::shared_ptr<FAIInputCommands> Commands,
        std::shared_ptr<const FInputActionContract> Contract,FStreamEpoch Epoch,FProducerIdentity Identity,FFrameNumber First)
    {
        if (!Commands || !Contract || !Epoch.Value || !Identity.Id || !Commands->GetContract()
            || Commands->GetContract()->GetFingerprint()!=Contract->GetFingerprint()) return {};
        return std::unique_ptr<FAIInputProducer>(new FAIInputProducer(std::move(Commands),std::move(Contract),Epoch,Identity,First));
    }
    EProducerContract GetProducerContract() const noexcept override { return EProducerContract::AI; }
    const std::shared_ptr<const FInputActionContract>& GetContract() const override { return Contract; }
    std::optional<FInputPollCutoff> FreezeForOwner(FFrameNumber N) override
    {
        if (FPresentationInputScope::IsActive() || Closed || Paused || Frozen || N!=Next || Sequence==UINT64_MAX) return {};
        Frozen=(NeutralBaseline && !Commands->HasPollEvaluator())
            ? FAIInputCommands::FDecision{} : Commands->EvaluateAtPoll(N);
        Delivered=false; ++Sequence;
        return FInputPollCutoff{Sequence,1,1};
    }
    std::optional<FInputFrame> Produce(FFrameNumber N) override
    {
        if (!Frozen || Delivered || N!=Next || N==UINT64_MAX) return {};
        FInputFrameData D; D.ConsumptionFrame=N; D.SourceSequence=Sequence;
        D.StreamEpoch=Epoch; D.Producer=Identity; D.DeviceGeneration={1};
        D.Reset=Reset || Frozen->Present!=WasPresent;
        D.Values=Frozen->Values;
        uint32_t Active=PreviousActive;
        if (D.Reset) Active=0;
        uint32_t Ordinal=0;
        const auto Edge=[&](FActionId Id,bool Down,int16_t Value)
        {
            const auto Bit=uint32_t{1}<<Id;
            if (((Active&Bit)!=0)==Down) return;
            if (Down) Active|=Bit; else Active&=~Bit;
            D.Transitions.push_back({Id,Down?ETransition::Started:ETransition::Completed,Value,{Sequence,Ordinal++}});
        };
        for (const auto& A:Contract->GetDescription().Actions)
        {
            if (D.Reset) { if (D.Values[A.Id]) Active|=uint32_t{1}<<A.Id; }
            else Edge(A.Id,D.Values[A.Id]!=0,D.Values[A.Id]);
        }
        if (Frozen->Present)
        {
            if (D.Reset)
            {
                // A live evaluator recomputes at the next poll; deferring this
                // pulse would duplicate a still-eligible jump on that frame.
                Deferred=Commands->HasPollEvaluator() ? std::vector<FActionId>{} : Frozen->Pulses;
            }
            else
            {
                Deferred.insert(Deferred.end(),Frozen->Pulses.begin(),Frozen->Pulses.end());
                for (const auto Id:Deferred) { Edge(Id,false,0); Edge(Id,true,1); Edge(Id,false,0); }
                Deferred.clear();
            }
        }
        else Deferred.clear();
        D.ActiveMask=Active;
        FInputFrame Result(Contract,D);
        if (!Result.IsValidFor(*Contract)) return {};
        PreviousActive=Active; WasPresent=Frozen->Present; Reset=false; NeutralBaseline=false; Delivered=true; ++Next;
        return Result;
    }
    bool CloseFrozenCutoff(const FInputPollCutoff& C) noexcept override
    { const bool Good=Frozen && Delivered && C.Sequence==Sequence && C.Generation==1 && C.LifecycleFence==1; Frozen.reset(); return Good; }
    ELifecycleResult SetLifecyclePaused(bool Value) override
    { if (Frozen || Closed) return ELifecycleResult::Rejected; Paused=Value; Reset=true; NeutralBaseline=!Value; Deferred.clear(); Commands->Invalidate(); return ELifecycleResult::Applied; }
    ELifecycleResult CancelLifecycle() override
    { Closed=true; Frozen.reset(); Deferred.clear(); Commands->Close(); return ELifecycleResult::Applied; }
private:
    FAIInputProducer(std::shared_ptr<FAIInputCommands> C,std::shared_ptr<const FInputActionContract> D,FStreamEpoch E,FProducerIdentity I,FFrameNumber N)
        : Commands(std::move(C)),Contract(std::move(D)),Epoch(E),Identity(I),Next(N) {}
    std::shared_ptr<FAIInputCommands> Commands;
    std::shared_ptr<const FInputActionContract> Contract;
    FStreamEpoch Epoch; FProducerIdentity Identity;
    FFrameNumber Next;
    uint64_t Sequence=0;
    uint32_t PreviousActive=0;
    std::optional<FAIInputCommands::FDecision> Frozen;
    std::vector<FActionId> Deferred;
    bool Reset=true,NeutralBaseline=false,WasPresent=false,Delivered=false,Paused=false,Closed=false;
};
}
