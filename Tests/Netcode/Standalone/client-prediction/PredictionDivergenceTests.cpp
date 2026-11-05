#include "IAmSpeed/Netcode/client-prediction/PredictionDivergence.h"
#include "IAmSpeed/Netcode/client-prediction/RemoteRequestPrediction.h"
#include <iostream>
#include <cstdlib>
using namespace Speed::Netcode;
namespace P=Speed::Netcode::Prediction;
static unsigned Checks=0;
static void Check(bool V,const char* Name) {++Checks;if(!V){std::cerr<<"FAIL "<<Name<<'\n';std::exit(1);}}
static P::FCompletedState Sample(U64 Frame)
{
    P::FCompletedState S;S.Timeline.World={1,2};S.Timeline.AuthorityPeer=3;S.Timeline.AuthorityEpoch=4;
    S.Timeline.Branch=5;S.Timeline.RegistryGeneration=6;S.Timeline.Schema[0]=7;
    S.State.Sequence=Frame+1;S.State.Frame=Frame;S.State.RegistryGeneration=6;S.State.WorldPayload={1,2,3};
    for (U64 Id:{101ull,102ull})
    {
        FCanonicalActorState C;C.Kinematic[12]=1;C.MechanicalSchema=1;C.GameplaySchema=1;C.Mechanical={3};C.Gameplay={4};
        S.State.Actors.push_back({Id,1,1,1,ELifecycle::Alive,*EncodeCanonicalActor(C)});
    }
    return S;
}
template<class F>static void Change(P::FCompletedState& S,F Function)
{auto V=*DecodeCanonicalActor(S.State.Actors[0].Payload);Function(V);S.State.Actors[0].Payload=*EncodeCanonicalActor(V);}
int main()
{
    auto A=Sample(20),B=A;
    Check(P::Compare(A,B).Result==P::EComparison::Equal,"identical completed frame");
    B.State.Sequence+=9;Check(P::Compare(A,B).Result==P::EComparison::Equal,"independent local publication serial");
    B=A;Change(B,[](auto& C){C.Kinematic[0]=3;C.Kinematic[1]=4;C.Kinematic[3]=12;C.Kinematic[4]=5;C.Kinematic[13]=8;C.Kinematic[14]=15;});
    auto R=P::Compare(A,B);Check(R.Result==P::EComparison::Divergent,"kinematic mismatch");
    Check(R.Actors.size()==2 && R.Actors[0].Position==5 && R.Actors[0].Velocity==13 && R.Actors[0].AngularVelocity==17,"independent known vector distances");
    B=A;Change(B,[](auto& C){C.Kinematic[11]=1;C.Kinematic[12]=0;});
    Check(std::abs(P::Compare(A,B).Actors[0].RotationRadians-std::acos(-1.0))<1e-12,"half-turn angle");
    B=A;Change(B,[](auto& C){C.Kinematic[12]=-1;});R=P::Compare(A,B);
    Check(R.Actors[0].RotationRadians==0 && R.Result==P::EComparison::Divergent,"quaternion sign geometry equal canonical bytes retained");
    B=A;Change(B,[](auto& C){C.Mechanical[0]=9;});R=P::Compare(A,B);
    Check(R.Result==P::EComparison::Divergent && R.Actors[0].Position==0,"hidden mechanical mismatch cannot be masked by pose");
    B=A;Change(B,[](auto& C){C.Gameplay[0]=9;});Check(P::Compare(A,B).Result==P::EComparison::Divergent,"gameplay mismatch");
    B=A;B.State.WorldPayload[0]=0;R=P::Compare(A,B);Check(R.Result==P::EComparison::Divergent && R.WorldBytesDifferent,"world contact supplement mismatch");
    B=A;B.State.Frame++;Check(P::Compare(A,B).Result==P::EComparison::InvalidState,"different frames never compared");
    B=A;B.Timeline.Branch++;Check(P::Compare(A,B).Result==P::EComparison::WrongTimeline,"wrong branch");
    B=A;B.Timeline.World.Incarnation++;Check(P::Compare(A,B).Result==P::EComparison::WrongTimeline,"world recreation");
    B=A;B.Timeline.AuthorityEpoch++;Check(P::Compare(A,B).Result==P::EComparison::WrongTimeline,"authority epoch");
    B=A;B.State.Actors[1].Incarnation++;Check(P::Compare(A,B).Result==P::EComparison::RosterMismatch,"actor respawn");
    B=A;B.State.Actors[1].Payload={0};Check(P::Compare(A,B).Result==P::EComparison::InvalidState,"malformed complete state");
    B=A;B.State.RegistryGeneration++;Check(P::Compare(A,B).Result==P::EComparison::InvalidState,"registry mismatch");
    P::FHistory H(A.Timeline,3,1024*1024);
    Check(H.Store(A)==P::EStore::Stored,"retain actual frame");
    Check(H.Store(A)==P::EStore::Nonconsecutive && H.Count()==1,"duplicate cannot replace history");
    Check(H.Store(Sample(22))==P::EStore::Nonconsecutive,"gap rejected");
    Check(H.CompareAuthority(Sample(21)).Result==P::EComparison::FutureAuthority,"future authority not compared to current pose");
    Check(H.CompareAuthority(Sample(19)).Result==P::EComparison::MissingHistory,"missing earlier frame");
    Check(H.Store(Sample(21))==P::EStore::Stored && H.Store(Sample(22))==P::EStore::Stored && H.Store(Sample(23))==P::EStore::Stored,"consecutive retention");
    Check(H.Count()==3 && H.CompareAuthority(A).Result==P::EComparison::MissingHistory,"bounded history eviction explicit");
    Check(H.CompareAuthority(Sample(21)).Result==P::EComparison::Equal,"compare correct historical frame");
    auto Wrong=Sample(24);Wrong.Timeline.AuthorityPeer++;Check(H.Store(Wrong)==P::EStore::WrongTimeline && H.Count()==3,"wrong sender timeline no mutation");
    P::FHistory Tiny(A.Timeline,3,1);Check(Tiny.Store(A)==P::EStore::CapacityExceeded && Tiny.Count()==0,"byte budget enforced");
    P::FHistory Bad(A.Timeline,0,1024);Check(Bad.Store(A)==P::EStore::Invalid,"zero retention rejected");
    FBinding Binding;Binding.Session=1;Binding.Epoch=2;Binding.Actor=101;Binding.Incarnation=3;Binding.OwnerPeer=4;Binding.ServerPeer=5;Binding.SourceProducer=6;Binding.SourceKind=0;Binding.Contract[0]=7;
    std::array<Consumed::FActionPolicy,ActionCount> Policies{};
    Policies[0]={true,true,false,255};Policies[1]={true,true,false,255};Policies[2]={true,true,true,127};
    auto Packet=[&](U64 Frame,int16_t Throttle)
    {
        FPacket V;V.Kind=EKind::AuthorityInput;V.Session=1;V.Epoch=2;V.Actor=101;V.Incarnation=3;V.Contract=Binding.Contract;V.Sequence=Frame+1;V.Frame=Frame;
        V.Input.SourceSequence=Frame+1;V.Input.DeviceGeneration=1;V.Input.ProducerId=6;V.Input.ProducerKind=0;V.Input.Values[0]=Throttle;V.Input.ActiveMask=Throttle?1u:0u;return V;
    };
    P::FRemoteRequestHistory Remote(Binding,Policies,10,32);
    Check(Remote.Predict(1).Status==P::EPredict::NoEarlierAuthority,"no implicit neutral opponent");
    auto V=Packet(10,255);
    Check(Remote.Confirm(4,V).Status==EResult::WrongPeer && Remote.Retained()==0,"owner cannot impersonate server");
    Check(Remote.Confirm(5,V).Status==EResult::Accepted,"server command anchor");
    Check(Remote.Confirm(5,V).Status==EResult::Duplicate,"same command idempotent");
    auto Conflict=V;Conflict.Input.Values[0]=254;Check(Remote.Confirm(5,Conflict).Status==EResult::Conflict,"conflicting authority rejected");
    auto Pred=Remote.Predict(11);
    Check(Pred.Status==P::EPredict::Predicted && Pred.Request.Requested[0]==255 && Pred.Request.EvidenceFrame==10,"hold requested axis before filtering");
    Check(Remote.Predict(13).Status==P::EPredict::OutOfOrder,"prediction frame gap rejected");
    Check(Remote.Predict(12).Status==P::EPredict::Predicted,"second speculative frame");
    V=Packet(11,0);auto Receipt=Remote.Confirm(5,V);
    Check(Receipt.Status==EResult::Accepted && Receipt.EarliestInvalidatedFrame==11,"late release invalidates predicted history");
    Remote.DiscardSpeculationFrom(11);
    Check(Remote.Predict(11).Status==P::EPredict::Confirmed,"original authoritative frame preserved for replay");
    Pred=Remote.Predict(12);Check(Pred.Status==P::EPredict::Predicted && Pred.Request.Requested[0]==0 && Pred.Request.ActiveMask==0,"replay future uses newly known request");
    P::FRemoteRequestHistory Horizon(Binding,Policies,1,4);Check(Horizon.Confirm(5,Packet(0,0)).Status==EResult::Accepted,"frame zero anchor allowed");
    Check(Horizon.Predict(1).Status==P::EPredict::Predicted && Horizon.Predict(2).Status==P::EPredict::HorizonExceeded,"bounded prediction lead");
    P::FRemoteRequestHistory Edge(Binding,Policies,10,8);Check(Edge.Confirm(5,Packet(0,0)).Status==EResult::Accepted && Edge.Predict(1).Status==P::EPredict::Predicted,"edge test setup");
    V=Packet(1,0);V.Input.Edges={{0,0,255,2,0},{0,1,0,2,1}};
    Check(Edge.Confirm(5,V).EarliestInvalidatedFrame==1,"within-frame tap invalidates even same final axis");
    P::FRemoteRequestHistory Bound(Binding,Policies,10,1);Check(Bound.Confirm(5,Packet(0,0)).Status==EResult::Accepted && Bound.Predict(1).Status==P::EPredict::CapacityExceeded,"no silent eviction of needed commands");
    P::FRemoteRequestHistory Mixed(Binding,Policies,10,16);
    Check(Mixed.Confirm(5,Packet(0,0)).Status==EResult::Accepted && Mixed.Confirm(5,Packet(2,1)).Status==EResult::Accepted,"mixed exact/speculative setup");
    Check(Mixed.Predict(1).Status==P::EPredict::Predicted && Mixed.Predict(2).Status==P::EPredict::Confirmed && Mixed.Predict(3).Status==P::EPredict::Predicted,"confirmed frame advances prediction cursor");
    Check(Mixed.Confirm(5,Packet(1,0)).Status==EResult::Accepted && Mixed.Retained()==4,"confirmation replaces speculative entry within budget");
    P::FRemoteRequestHistory Cut(Binding,Policies,10,16);
    Check(Cut.Confirm(5,Packet(9,255)).Status==EResult::Accepted && Cut.Confirm(5,Packet(11,0)).Status==EResult::Accepted,"restore cut setup mixes later received confirmation");
    Check(Cut.Predict(10).Status==P::EPredict::Predicted && Cut.Predict(11).Status==P::EPredict::Confirmed && Cut.Predict(12).Status==P::EPredict::Predicted,"consume speculative confirmed speculative sequence");
    Cut.DiscardSpeculationFrom(12);auto AtCut=Cut.Predict(12);
    Check(AtCut.Status==P::EPredict::Predicted && AtCut.Request.EvidenceFrame==11 && AtCut.Request.Requested[0]==0,"restore uses exact first-replay cursor despite earlier surviving speculation");
    Cut.DiscardSpeculationFrom(0);
    Check(Cut.Predict(1).Status==P::EPredict::OutOfOrder,"frame-zero replay cut cannot silently skip first frame");
    Check(Cut.Confirm(5,Packet(0,0)).Status==EResult::Accepted && Cut.Predict(0).Status==P::EPredict::Confirmed,"confirmed frame zero replay has no underflow");
    P::FRemoteRequestHistory Exhaust(Binding,Policies,10,8);
    Check(Exhaust.Confirm(5,Packet(UINT64_MAX-1,0)).Status==EResult::Accepted && Exhaust.Predict(UINT64_MAX).Status==P::EPredict::Predicted,"last representable speculative frame accepted once");
    Check(Exhaust.Predict(UINT64_MAX).Status==P::EPredict::OutOfOrder,"exhausted cursor cannot wrap");
    Exhaust.DiscardSpeculationFrom(UINT64_MAX);
    Check(Exhaust.Predict(UINT64_MAX).Status==P::EPredict::Predicted,"explicit restored cut can replay last frame without overflow");
    auto Invalid=Packet(4,0);Invalid.Input.Values[7]=1;
    Check(Mixed.Confirm(5,Invalid).Status==EResult::Invalid,"unwired axis request rejected");
    Invalid=Packet(4,0);Invalid.Input.ActiveMask=(1u<<7);
    Check(Mixed.Confirm(5,Invalid).Status==EResult::Invalid,"unwired active action rejected");
    Invalid=Packet(4,0);Invalid.Input.ProducerId++;
    Check(Mixed.Confirm(5,Invalid).Status==EResult::Invalid,"different original producer rejected");
    Invalid=Packet(4,0);Invalid.Contract[1]=1;
    Check(Mixed.Confirm(5,Invalid).Status==EResult::Invalid,"different frozen action contract rejected");
    Invalid=Packet(4,0);Invalid.Input.Edges={{7,0,1,5,0}};Invalid.Input.ActiveMask=(1u<<7);
    Check(Mixed.Confirm(5,Invalid).Status==EResult::Invalid,"unwired event rejected");
    std::cout<<"PASS "<<Checks<<" historical divergence checks; no native restore or network qualification\n";
}
