#include "IAmSpeed/Netcode/client-prediction/RollbackReplayCoordinator.h"
#include <cstdlib>
#include <iostream>
using namespace Speed::Netcode;
namespace P=Speed::Netcode::Prediction;
namespace A=Speed::Netcode::StateAuthorityV2;
static unsigned Checks=0;
static void Check(bool B,const char* S){++Checks;if(!B){std::cerr<<"FAIL "<<S<<'\n';std::exit(1);}}
static P::FCompletedState Frame(U64 N,int Position)
{
    P::FCompletedState S;S.Timeline.World={1,2};S.Timeline.AuthorityPeer=3;S.Timeline.AuthorityEpoch=4;S.Timeline.Branch=5;S.Timeline.RegistryGeneration=6;S.Timeline.Schema[0]=7;
    S.State.Sequence=N+1;S.State.Frame=N;S.State.RegistryGeneration=6;S.State.WorldPayload={1};
    FCanonicalActorState C;C.Kinematic[0]=Position;C.Kinematic[12]=1;C.MechanicalSchema=C.GameplaySchema=1;C.Mechanical={1};C.Gameplay={1};
    S.State.Actors.push_back({101,1,1,1,ELifecycle::Alive,*EncodeCanonicalActor(C)});return S;
}
struct FEvidence final:A::IFullSnapshotEvidence
{bool Confirms(const A::FEnvelope&,const A::FTrustedSnapshot&,const std::vector<uint8_t>&)const noexcept override{return true;}};
// Synthetic source evidence ONLY; never a native authority/restore certificate.
static auto Origin()
{
    A::FEnvelope E;E.State=Frame(2,2).State;auto& D=E.Address;D.World={1,2};D.AuthorityPeer=3;D.AuthorityEpoch=4;D.AuthorityTimelineRevision=5;D.BeforeWorldPublication=8;D.PublicationSerial=9;D.WorldStateHash=10;D.Frame=2;D.RegistryGeneration=6;D.StateSchema[0]=7;
    A::FTrustedSnapshot T;T.Address=D;T.Roster={{101,1,1,1,ELifecycle::Alive}};return A::Accept(3,E,T,FEvidence{});
}
struct FWorld final:P::IRollbackWorld
{
    int Prepares=0,Commits=0,Steps=0,Finishes=0,Retires=0;
    bool RejectPrepare=false,BadRestore=false,FailStep=false,FailFinish=false;
    struct FPlan final:P::IPreparedRollback
    {
        FWorld& W;explicit FPlan(FWorld& V):W(V){}
        void CommitRestore()noexcept override{++W.Commits;}
        std::optional<P::FCompletedState> ReadActualRestoredPublication()override{return Frame(2,W.BadRestore?9:2);}
        std::optional<P::FCompletedState> ReplayCanonicalFrame(U64 N)override
        {++W.Steps;if(W.FailStep)return {};return Frame(N,static_cast<int>(N));}
        bool PublishLivePresentationAndRelease()override{++W.Finishes;return !W.FailFinish;}
        void RetireFailedTimeline()noexcept override{++W.Retires;}
    };
    std::unique_ptr<P::IPreparedRollback> PrepareCompleteRollback(std::shared_ptr<const A::FValidatedOriginSnapshot>,U64)override
    {++Prepares;return RejectPrepare?nullptr:std::make_unique<FPlan>(*this);}
}; // Arithmetic world fixture; no UE physics is executed here.
static void Fill(P::FHistory& H,bool Wrong=true)
{for(U64 N=0;N<=5;++N)Check(H.Store(Frame(N,static_cast<int>(N)+(Wrong?100:0)))==P::EStore::Stored,"fixture frame retained");}
int main()
{
    const auto Source=Origin();Check(bool(Source),"synthetic authenticated envelope fixture");
    {
        P::FHistory H(Frame(0,0).Timeline,16,1024*1024);Fill(H);P::FReplayCoordinator C;FWorld W;
        Check(C.Begin(Source,5,10,H,W)==P::EReplay::Replaying,"restore starts correction");
        Check(W.Commits==1 && W.Steps==0 && W.Finishes==0,"restore before replay no early presentation");
        Check(C.Begin(Source,5,10,H,W)==P::EReplay::Busy,"overlapping corrections rejected");
        Check(C.Advance(2)==P::EReplay::Replaying && W.Steps==2 && W.Finishes==0,"bounded replay pulse");
        Check(C.Advance(2)==P::EReplay::Complete && C.ReplayedFrames()==3 && W.Finishes==1 && W.Retires==0,"exact F+1 through target and final publication");
        for(U64 N=2;N<=5;++N)Check(H.CompareAuthority(Frame(N,static_cast<int>(N))).Result==P::EComparison::Equal,"replayed full history replaces old predictions");
    }
    {
        P::FHistory H(Frame(0,0).Timeline,16,1024*1024);Fill(H,false);P::FReplayCoordinator C;FWorld W;
        Check(C.Begin(Source,5,10,H,W)==P::EReplay::Equal && W.Prepares==0,"equal history does not mutate world");
    }
    for(int Mode=0;Mode<4;++Mode)
    {
        P::FHistory H(Frame(0,0).Timeline,16,1024*1024);Fill(H);P::FReplayCoordinator C;FWorld W;
        W.RejectPrepare=Mode==0;W.BadRestore=Mode==1;W.FailStep=Mode==2;W.FailFinish=Mode==3;
        auto Status=C.Begin(Source,5,10,H,W);if(Status==P::EReplay::Replaying)Status=C.Advance(10);
        Check(Status==(Mode==0?P::EReplay::Rejected:P::EReplay::Failed),"native adapter failure never accepted");
        Check(Mode==0?(W.Commits==0 && W.Retires==0 && H.LastFrame()==5):(W.Retires==1),"precommit unchanged or postcommit fail closed");
    }
    {
        P::FHistory H(Frame(0,0).Timeline,16,1024*1024);Fill(H);P::FReplayCoordinator C;FWorld W;
        Check(C.Begin(Source,5,2,H,W)==P::EReplay::Rejected && W.Prepares==0,"catchup budget enforced before mutation");
        Check(C.Begin(Source,4,10,H,W)==P::EReplay::Rejected && W.Prepares==0,"target is actual current historical frame");
    }
    {
        P::FHistory H(Frame(0,0).Timeline,2,1024*1024);Fill(H);P::FReplayCoordinator C;FWorld W;
        Check(C.Begin(Source,5,10,H,W)==P::EReplay::NeedResync && W.Prepares==0,"expired state explicitly requires resync");
    }
    std::cout<<"PASS "<<Checks<<" replay coordinator fixture checks; native adapter absent\n";
}
