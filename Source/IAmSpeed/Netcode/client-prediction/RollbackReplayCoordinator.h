#pragma once
#include "PredictionDivergence.h"
#include "../state-authority-envelope-v2/StateAuthorityEnvelope.h"
#include <thread>

namespace Speed::Netcode::Prediction
{
// Coordinator only. A real native world adapter is mandatory and is NOT
// supplied here. No legacy restore, live-registry guard or profile is opened.
class IPreparedRollback
{
public:
    virtual ~IPreparedRollback()=default;
    // Prepare has staged every fallible operation, lifetime pin, world/input
    // participant and exact command range under the original owner boundary.
    virtual void CommitRestore() noexcept=0;
    virtual std::optional<FCompletedState> ReadActualRestoredPublication()=0;
    // One complete original canonical physical transition, fixed delta, no
    // pacing wait. Suppress external presentation, never physical gameplay.
    virtual std::optional<FCompletedState> ReplayCanonicalFrame(U64 Frame)=0;
    virtual bool PublishLivePresentationAndRelease()=0;
    // After mutation there is no undo or live fallback. Stop/join the timeline.
    virtual void RetireFailedTimeline() noexcept=0;
};
class IRollbackWorld
{
public:
    virtual ~IRollbackWorld()=default;
    // Failure must leave the original world/input state untouched. The plan
    // must hold all lifetime/configuration/access reservations through release.
    virtual std::unique_ptr<IPreparedRollback> PrepareCompleteRollback(
        std::shared_ptr<const StateAuthorityV2::FValidatedOriginSnapshot> Source,U64 Target)=0;
};
enum class EReplay { Idle, Equal, NeedResync, Busy, Replaying, Complete, Rejected, Failed };
class FReplayCoordinator
{
    std::thread::id Owner=std::this_thread::get_id();
    EReplay Phase=EReplay::Idle;
    U64 Next=0,Target=0,Replayed=0;
    FHistory* History=nullptr;
    std::unique_ptr<IPreparedRollback> Plan;
    FCompletedState Authority;
    EReplay Fail() noexcept
    {if(Plan)Plan->RetireFailedTimeline();Plan.reset();Phase=EReplay::Failed;return Phase;}
public:
    FReplayCoordinator()=default;
    FReplayCoordinator(const FReplayCoordinator&)=delete;
    FReplayCoordinator& operator=(const FReplayCoordinator&)=delete;
    ~FReplayCoordinator() {if(Plan)Plan->RetireFailedTimeline();}
    EReplay Begin(std::shared_ptr<const StateAuthorityV2::FValidatedOriginSnapshot> Source,
        U64 TargetFrame,U64 MaximumReplayFrames,FHistory& Local,IRollbackWorld& World)
    {
        if (std::this_thread::get_id()!=Owner) return EReplay::Rejected;
        if (Plan) return EReplay::Busy;
        if (!Source || !MaximumReplayFrames || MaximumReplayFrames>4096) return EReplay::Rejected;
        const auto& S=Source->Source();const auto& A=S.Address;
        FCompletedState Server;Server.Timeline={A.World,A.AuthorityPeer,A.AuthorityEpoch,A.AuthorityTimelineRevision,A.RegistryGeneration,A.StateSchema};Server.State=S.State;
        const auto Difference=Local.CompareAuthority(Server);
        if (Difference.Result==EComparison::Equal) {Phase=EReplay::Equal;return Phase;}
        if (Difference.Result==EComparison::MissingHistory || Difference.Result==EComparison::FutureAuthority)
        {Phase=EReplay::NeedResync;return Phase;}
        if (Difference.Result!=EComparison::Divergent || !Local.LastFrame() || *Local.LastFrame()!=TargetFrame
            || A.Frame>TargetFrame || TargetFrame-A.Frame>MaximumReplayFrames
            || TargetFrame==std::numeric_limits<U64>::max()) return EReplay::Rejected;
        auto Prepared=World.PrepareCompleteRollback(Source,TargetFrame);
        if (!Prepared) return EReplay::Rejected;
        Plan=std::move(Prepared);History=&Local;Authority=std::move(Server);Target=TargetFrame;Next=A.Frame+1;Replayed=0;
        Plan->CommitRestore();
        const auto Restored=Plan->ReadActualRestoredPublication();
        if (!Restored || Compare(*Restored,Authority).Result!=EComparison::Equal) return Fail();
        History->TruncateFrom(A.Frame);
        if (History->Store(*Restored)!=EStore::Stored) return Fail();
        Phase=EReplay::Replaying;
        return Advance(0);
    }
    EReplay Advance(U64 MaximumFramesThisPulse)
    {
        if (std::this_thread::get_id()!=Owner) return EReplay::Rejected;
        if (!Plan || Phase!=EReplay::Replaying) return Phase;
        U64 Count=0;
        while (Next<=Target && Count<MaximumFramesThisPulse)
        {
            const auto Actual=Plan->ReplayCanonicalFrame(Next);
            if (!Actual || Actual->State.Frame!=Next || !(Actual->Timeline==Authority.Timeline)
                || History->Store(*Actual)!=EStore::Stored) return Fail();
            ++Next;++Count;++Replayed;
        }
        if (Next>Target)
        {
            if (!Plan->PublishLivePresentationAndRelease()) return Fail();
            Plan.reset();Phase=EReplay::Complete;
        }
        return Phase;
    }
    U64 ReplayedFrames() const noexcept {return Replayed;}
    EReplay State() const noexcept {return Phase;}
};
}
