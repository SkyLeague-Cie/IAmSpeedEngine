#include "PrivateRoadPredictionSession.h"
#include <algorithm>
#include <mutex>

namespace Speed::Netcode::Prediction
{
namespace
{
bool SessionRefuse(FString& Why,const TCHAR* Reason) {Why=Reason;return false;}
// Same continuity rules as the original same-frame owner. This validates
// already-retained raw frames without polling/reacquiring or forging receipts.
bool SessionContinues(const BranchV2::FInputFrame& Frame,const std::shared_ptr<const BranchV2::FInputFrame>& Previous)
{
    const auto& D=Frame.GetData();if(!Previous)return D.Reset;
    const auto& Old=Previous->GetData();if(D.SourceSequence<Old.SourceSequence)return false;
    if(D.Reset)return D.DeviceGeneration.Value>=Old.DeviceGeneration.Value&&D.SourceSequence>Old.SourceSequence;
    if(D.DeviceGeneration.Value!=Old.DeviceGeneration.Value)return false;
    if(D.SourceSequence==Old.SourceSequence)return D.Values==Old.Values&&D.ActiveMask==Old.ActiveMask&&D.Transitions.empty();
    auto Active=Old.ActiveMask;
    for(const auto& E:D.Transitions)
    {
        const auto Bit=uint32(1)<<E.Action;const bool Started=E.State==BranchV2::ETransition::Started;
        if(E.Order.Sequence<=Old.SourceSequence||bool(Active&Bit)==Started)return false;
        if(Started)Active|=Bit;else Active&=~Bit;
    }
    return Active==D.ActiveMask;
}
std::array<Consumed::FActionPolicy,ActionCount> SessionActionPolicies(const BranchV2::FInputActionContract& Contract)
{
    std::array<Consumed::FActionPolicy,ActionCount> Out{};
    for(size_t I=0;I<ActionCount;++I)if(const auto* A=Contract.Find(Speed::Input::FActionId(I)))
    {
        if(A->Wiring==BranchV2::EActionWiring::Wired)
            Out[I]={true,A->Type==BranchV2::EActionType::Axis1D,A->Signed,A->Quantization};
    }
    return Out;
}
}
struct FPrivateRoadPredictionSession::FImpl
{
    std::vector<FPrivateRoadCarSeed> Seeds;
    size_t Local=0;
    U64 Maximum=0,AuthorityRevision=0,StagedRevision=0,ReplayNext=0,CorrectionCount=0;
    std::optional<U64> Invalidated;
    std::optional<FPrivateRoadCorrectionWitness> Witness;
    std::vector<std::shared_ptr<const BranchV2::FInputFrame>> LocalFrames;
    std::vector<std::unique_ptr<FRemoteRequestHistory>> Histories,StagedHistories;
    std::unique_ptr<FPrivateRoadPhysicsWorld> Live,Staged;
    std::atomic<std::shared_ptr<const FPrivateRoadPublication>> Published;
    mutable std::mutex Gate;
    bool Closed=false,Failed=false,StagedNeedsRestart=false;
    void Fail() noexcept {Failed=true;Published.store({});}
    bool Requests(U64 Frame,const std::shared_ptr<const BranchV2::FInputFrame>& LocalFrame,
        std::vector<std::unique_ptr<FRemoteRequestHistory>>& Source,std::vector<FPrivateRoadRequest>& Out)
    {
        Out.reserve(Seeds.size());
        for(size_t A=0;A<Seeds.size();++A)
        {
            if(A==Local)Out.push_back({Seeds[A].Actor,Seeds[A].Incarnation,LocalFrame});
            else
            {
                auto Forecast=Source[A]->Predict(Frame);
                if(Forecast.Status!=EPredict::Predicted&&Forecast.Status!=EPredict::Confirmed)return false;
                Out.push_back({Seeds[A].Actor,Seeds[A].Incarnation,std::move(Forecast)});
            }
        }
        return true;
    }
};
FPrivateRoadPredictionSession::FPrivateRoadPredictionSession():Impl(std::make_unique<FImpl>()){}
FPrivateRoadPredictionSession::~FPrivateRoadPredictionSession()
{
    const bool Retired=Close();
    checkf(Retired,TEXT("Native prediction session must retire on its GT outside a step"));
    if(!Retired)std::terminate(); // do not hide lifecycle side effects inside checkf
}
std::unique_ptr<FPrivateRoadPredictionSession> FPrivateRoadPredictionSession::Create(std::vector<FPrivateRoadCarSeed> Seeds,U64 MaximumFrames,FString& Why)
{
    Why.Empty();if(!IsInGameThread()||Seeds.size()<2||Seeds.size()>16||!MaximumFrames||MaximumFrames>4096
        ||Speed::Input::FPresentationInputScope::IsActive()) {SessionRefuse(Why,TEXT("native prediction finite GT domain refused"));return {};}
    std::sort(Seeds.begin(),Seeds.end(),[](const auto& A,const auto& B){return A.Actor<B.Actor;});
    auto Session=std::unique_ptr<FPrivateRoadPredictionSession>(new FPrivateRoadPredictionSession);auto& I=*Session->Impl;
    size_t Locals=0;
    for(size_t A=0;A<Seeds.size();++A)if(Seeds[A].Role==EBranchFilterRole::LocalDevice){++Locals;I.Local=A;}
    if(Locals!=1){SessionRefuse(Why,TEXT("native prediction requires one genuine local input domain"));return {};}
    I.Seeds=std::move(Seeds);I.Maximum=MaximumFrames;I.LocalFrames.reserve(size_t(MaximumFrames));
    I.Histories.resize(I.Seeds.size());
    for(size_t A=0;A<I.Seeds.size();++A)if(A!=I.Local)
    {
        if(!I.Seeds[A].Contract){SessionRefuse(Why,TEXT("native remote contract missing"));return {};}
        I.Histories[A]=std::make_unique<FRemoteRequestHistory>(I.Seeds[A].Remote,SessionActionPolicies(*I.Seeds[A].Contract),I.Seeds[A].PredictionHorizon,size_t(MaximumFrames));
    }
    I.Live=FPrivateRoadPhysicsWorld::Create(I.Seeds,Why);if(!I.Live)return {};
    return Session;
}
FConfirmation FPrivateRoadPredictionSession::ConfirmRemote(U64 Peer,const FPacket& Packet)
{
    FConfirmation Result;if(!IsInGameThread()||Speed::Input::FPresentationInputScope::IsActive())return Result;
    auto& I=*Impl;std::unique_lock Lock(I.Gate,std::try_to_lock);
    if(!Lock.owns_lock()||I.Closed||I.Failed||I.AuthorityRevision==MAX_uint64||Packet.Frame>=I.Maximum)return Result;
    for(size_t A=0;A<I.Seeds.size();++A)if(A!=I.Local&&I.Seeds[A].Actor==Packet.Actor)
    {
        Result=I.Histories[A]->Confirm(Peer,Packet);
        if(Result.Status==EResult::Accepted)
        {
            ++I.AuthorityRevision;
            // Keep original ingress live during replay. A future or equivalent
            // confirmation updates the staged journal without restarting work.
            // Restart only if it changes an ALREADY replayed native transition.
            if(I.Staged)
            {
                try
                {
                    const auto Staged=I.StagedHistories[A]->Confirm(Peer,Packet);
                    if(Staged.Status!=EResult::Accepted&&Staged.Status!=EResult::Duplicate)
                    {I.Fail();Result.Status=EResult::Invalid;return Result;}
                    if(Staged.EarliestInvalidatedFrame&&*Staged.EarliestInvalidatedFrame<I.ReplayNext)I.StagedNeedsRestart=true;
                    if(!I.StagedNeedsRestart)I.StagedRevision=I.AuthorityRevision;
                }
                catch(...) {I.Fail();Result.Status=EResult::Invalid;return Result;}
            }
            if(Result.EarliestInvalidatedFrame&&*Result.EarliestInvalidatedFrame<I.LocalFrames.size())
                I.Invalidated=I.Invalidated?std::min(*I.Invalidated,*Result.EarliestInvalidatedFrame):*Result.EarliestInvalidatedFrame;
        }
        return Result;
    }
    return Result;
}
bool FPrivateRoadPredictionSession::AdvanceLocal(std::shared_ptr<const BranchV2::FInputFrame> Frame,FString& Why)
{
    Why.Empty();if(!IsInGameThread()||Speed::Input::FPresentationInputScope::IsActive())return SessionRefuse(Why,TEXT("native local advance requires GT"));
    auto& I=*Impl;std::unique_lock Lock(I.Gate,std::try_to_lock);
    if(!Lock.owns_lock()||I.Closed||I.Failed||!Frame||I.LocalFrames.size()>=I.Maximum)return SessionRefuse(Why,TEXT("native local owner/history unavailable or exhausted"));
    const auto& S=I.Seeds[I.Local];const auto& D=Frame->GetData();const U64 N=I.LocalFrames.size();
    if(!Frame->IsValidFor(*S.Contract)||D.ConsumptionFrame!=N||D.Producer.Kind!=S.Local.Kind||D.Producer.Id!=S.Local.Id
        ||D.StreamEpoch.Value!=S.LocalEpoch.Value||!SessionContinues(*Frame,I.LocalFrames.empty()?nullptr:I.LocalFrames.back()))
        return SessionRefuse(Why,TEXT("retained local frame identity/continuity refused"));
    try
    {
        std::vector<FPrivateRoadRequest> Requests;
        if(!I.Requests(N,Frame,I.Histories,Requests)||!I.Live->Step(Requests,false,Why))
        {I.Fail();return SessionRefuse(Why,TEXT("native prediction input/physical step failed; branch retired"));}
        I.LocalFrames.push_back(std::move(Frame));I.Published.store(I.Live->ReadLatest());return true;
    }
    catch(...) {I.Fail();return SessionRefuse(Why,TEXT("native prediction advance exception; branch retired"));}
}
EPrivateRoadCorrection FPrivateRoadPredictionSession::AdvanceCorrection(U64 Budget,FString& Why)
{
    Why.Empty();if(!IsInGameThread()||Speed::Input::FPresentationInputScope::IsActive()) {SessionRefuse(Why,TEXT("native correction requires GT"));return EPrivateRoadCorrection::Failed;}
    auto& I=*Impl;std::unique_lock Lock(I.Gate,std::try_to_lock);
    if(!Lock.owns_lock()||I.Closed||I.Failed||!Budget||Budget>256){SessionRefuse(Why,TEXT("native correction owner/budget refused"));return EPrivateRoadCorrection::Failed;}
    if(!I.Invalidated)return EPrivateRoadCorrection::Idle;
    const auto Fail=[&](const TCHAR* Reason){I.Fail();SessionRefuse(Why,Reason);return EPrivateRoadCorrection::Failed;};
    try
    {
        if(I.Staged&&(I.StagedNeedsRestart||I.StagedRevision!=I.AuthorityRevision))
        {if(!I.Staged->Close())return Fail(TEXT("superseded staged native graph did not retire"));I.Staged.reset();I.StagedHistories.clear();}
        if(!I.Staged)
        {
            I.Staged=FPrivateRoadPhysicsWorld::Create(I.Seeds,Why);if(!I.Staged)return Fail(TEXT("fresh native correction graph creation failed"));
            I.StagedHistories.resize(I.Histories.size());
            for(size_t A=0;A<I.Histories.size();++A)if(I.Histories[A])
            {
                I.StagedHistories[A]=std::make_unique<FRemoteRequestHistory>(*I.Histories[A]);
                // The NEW graph's actual birth boundary is before frame0.
                // This is original-seed reconstruction, not a claimed snapshot restore.
                I.StagedHistories[A]->DiscardSpeculationFrom(0);
            }
            I.ReplayNext=0;I.StagedRevision=I.AuthorityRevision;I.StagedNeedsRestart=false;
        }
        U64 Done=0;
        while(I.ReplayNext<I.LocalFrames.size()&&Done<Budget)
        {
            std::vector<FPrivateRoadRequest> Requests;
            if(!I.Requests(I.ReplayNext,I.LocalFrames[size_t(I.ReplayNext)],I.StagedHistories,Requests)
                ||!I.Staged->Step(Requests,true,Why))return Fail(TEXT("native corrected replay failed; no fallback"));
            ++I.ReplayNext;++Done;
        }
        if(I.ReplayNext<I.LocalFrames.size())return EPrivateRoadCorrection::Replaying;
        const auto Old=I.Published.load(),New=I.Staged->ReadLatest();
        if(!Old||!New||Old->Frame!=New->Frame||Old->Actors.size()!=New->Actors.size()
            ||I.CorrectionCount==MAX_uint64||I.StagedRevision!=I.AuthorityRevision)return Fail(TEXT("native corrected publication cut mismatch"));
        FPrivateRoadCorrectionWitness Witness;Witness.FirstInvalidated=*I.Invalidated;Witness.ReplayedFrames=I.ReplayNext;
        Witness.ThroughFrame=New->Frame;Witness.Corrections=I.CorrectionCount+1;
        for(size_t A=0;A<New->Actors.size();++A)
        {
            if(Old->Actors[A].Actor!=New->Actors[A].Actor||Old->Actors[A].Incarnation!=New->Actors[A].Incarnation)return Fail(TEXT("native correction actor identity changed"));
            const auto* X=Old->Physics.PresentationBodies.FindByPredicate([&](const auto& B){return B.StableId==Old->Actors[A].NativeStableId;});
            const auto* Y=New->Physics.PresentationBodies.FindByPredicate([&](const auto& B){return B.StableId==New->Actors[A].NativeStableId;});
            if(!X||!Y)return Fail(TEXT("native correction pose missing"));
            Witness.MaximumPositionChange=FMath::Max(Witness.MaximumPositionChange,FVector::Dist(X->COMState.Location,Y->COMState.Location));
            Witness.MaximumVelocityChange=FMath::Max(Witness.MaximumVelocityChange,FVector::Dist(X->COMState.Velocity,Y->COMState.Velocity));
        }
        // Opaque native graph and its real history change together on their one
        // GT owner. No native pointer or mutable view has been published.
        I.Live.swap(I.Staged);I.Histories.swap(I.StagedHistories);
        I.Witness=Witness;++I.CorrectionCount;I.Invalidated.reset();I.Published.store(New);
        if(!I.Staged->Close())return Fail(TEXT("old native graph retirement failed after adoption"));
        I.Staged.reset();I.StagedHistories.clear();return EPrivateRoadCorrection::Adopted;
    }
    catch(...) {return Fail(TEXT("native correction exception; timeline retired"));}
}
std::shared_ptr<const FPrivateRoadPublication> FPrivateRoadPredictionSession::ReadLatest() const noexcept {return Impl->Published.load();}
std::optional<FPrivateRoadCorrectionWitness> FPrivateRoadPredictionSession::ReadLastCorrection() const
{
    if(!IsInGameThread())return {};auto& I=*Impl;std::unique_lock Lock(I.Gate,std::try_to_lock);
    return Lock.owns_lock()?I.Witness:std::optional<FPrivateRoadCorrectionWitness>{};
}
bool FPrivateRoadPredictionSession::Close()
{
    if(!IsInGameThread())return false;auto& I=*Impl;std::unique_lock Lock(I.Gate,std::try_to_lock);
    if(!Lock.owns_lock())return false;if(I.Closed)return true;
    if(I.Staged&&!I.Staged->Close())return false;if(I.Live&&!I.Live->Close())return false;
    I.Closed=true;I.Published.store({});I.Staged.reset();I.Live.reset();I.StagedHistories.clear();I.Histories.clear();return true;
}
}
