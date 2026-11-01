#include "NativePredictionProcessProbe.h"
#include "RedundantDatagram.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadPredictionSession.h"
#include "IAmSpeed/Driving/DrivingInput.h"
#include <bit>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>

namespace Speed::Netcode
{
namespace ProcessProbeDetail
{
namespace P=Prediction;namespace V=Speed::Input::V2;
constexpr U64 Frames=240,Server=5,PeerA=4,PeerB=7;
enum :std::uint8_t {Hello=1,OwnerInput=2,AuthorityInput=3,Observed=4,RetentionAck=5,ClientDone=6,ObservedError90=8,InitialPublication=9,StartupReady=10,StartActivation=11};
struct FSample {FPacket Packet;std::shared_ptr<const V::FInputFrame> Local;};
bool GameValues(const Speed::Netcode::FFrame& F)
{if(F.Values[0]<0||F.Values[0]>255||F.Values[1]<0||F.Values[1]>255||F.Values[2]<-127||F.Values[2]>127)return false;for(std::size_t A=3;A<ActionCount;++A)if(F.Values[A])return false;return true;}
FBinding Binding(unsigned Actor)
{FBinding B;B.Session=11;B.Epoch=3;B.Actor=101+Actor;B.Incarnation=3;B.OwnerPeer=Actor?PeerB:PeerA;B.ServerPeer=Server;B.SourceProducer=42+Actor;B.SourceKind=0;B.Contract[0]=7;return B;}
std::array<int16,3> Values(unsigned Actor,U64 N)
{return N<30?std::array<int16,3>{int16(Actor?255:128),0,0}:N<60?std::array<int16,3>{0,int16(Actor?255:128),0}:std::array<int16,3>{int16(Actor?128:192),0,int16(Actor?32:-32)};}
std::vector<P::FPrivateRoadCarSeed> Seeds(U64 Self)
{
    auto Contract=Speed::Driving::MakeContract(Speed::Driving::FBindings{});std::vector<P::FPrivateRoadCarSeed> Out;
    for(unsigned A=0;A<2;++A)
    {
        auto B=Binding(A);P::FPrivateRoadCarSeed S;S.Actor=B.Actor;S.Incarnation=B.Incarnation;S.Remote=B;S.Contract=Contract;
        S.Role=Self==B.OwnerPeer?P::EBranchFilterRole::LocalDevice:P::EBranchFilterRole::RemoteRequests;
        S.Local={Speed::Input::EProducerKind::Device,B.SourceProducer};S.LocalEpoch={3}; // default closed initial-publication gate, no warmup countdown
        S.Initial.Location=FVector(0,A?1000:-1000,45);S.Initial.Velocity=FVector(500,0,0);
        S.Processing.Step[0]=17;S.Processing.Step[1]=17;S.Processing.Step[2]=9;Out.push_back(S);
    }
    return Out;
}
FSample Sample(unsigned Actor,U64 N,const std::shared_ptr<const V::FInputActionContract>& Contract)
{
    const auto B=Binding(Actor);const auto Q=Values(Actor,N);FSample S;auto& Pkt=S.Packet;
    Pkt.Kind=EKind::OwnerInput;Pkt.Session=B.Session;Pkt.Epoch=B.Epoch;Pkt.Actor=B.Actor;Pkt.Incarnation=B.Incarnation;
    Pkt.Contract=B.Contract;Pkt.Frame=N;Pkt.Sequence=N+1;Pkt.Input.SourceSequence=N+1;Pkt.Input.DeviceGeneration=1;
    Pkt.Input.ProducerId=B.SourceProducer;Pkt.Input.ProducerKind=B.SourceKind;Pkt.Input.Reset=N==0;
    V::FInputFrameData D;D.ConsumptionFrame=N;D.SourceSequence=N+1;D.DeviceGeneration={1};D.Producer={Speed::Input::EProducerKind::Device,B.SourceProducer};D.StreamEpoch={3};D.Reset=N==0;
    unsigned Ordinal=0;const auto Prev=N?Values(Actor,N-1):std::array<int16,3>{};
    for(unsigned A=0;A<3;++A)
    {
        Pkt.Input.Values[A]=D.Values[A]=Q[A];if(Q[A]){Pkt.Input.ActiveMask|=1u<<A;D.ActiveMask|=1u<<A;}
        if(N&&bool(Prev[A])!=bool(Q[A]))
        {
            Pkt.Input.Edges.push_back({std::uint8_t(A),std::uint8_t(Q[A]?0:1),Q[A],N+1,Ordinal});
            D.Transitions.push_back({Speed::Input::FActionId(A),Q[A]?V::ETransition::Started:V::ETransition::Completed,Q[A],{N+1,Ordinal}});++Ordinal;
        }
    }
    S.Local=std::make_shared<const V::FInputFrame>(Contract,D);return S;
}
std::vector<std::uint8_t> Observe(const P::FPrivateRoadPublication& Actual)
{
    std::vector<std::uint8_t> Bytes;Wire::Put(Bytes,Actual.Frame,8);Wire::Put(Bytes,Actual.Actors.size(),1);
    const auto Number=[&](double X){Wire::Put(Bytes,std::bit_cast<U64>(X),8);};
    for(const auto& A:Actual.Actors)
    {
        const auto* B=Actual.Physics.PresentationBodies.FindByPredicate([&](const auto& V){return V.StableId==A.NativeStableId;});if(!B)return {};
        Wire::Put(Bytes,A.Actor,8);Wire::Put(Bytes,A.Incarnation,8);const auto& K=B->COMState;
        const auto Vector=[&](const auto& V){Number(V.X);Number(V.Y);Number(V.Z);};
        Vector(K.Location);Vector(K.Velocity);Vector(K.Acceleration);Vector(K.AngularVelocity);Vector(K.AngularAcceleration);
        for(double X:{K.Rotation.X,K.Rotation.Y,K.Rotation.Z,K.Rotation.W})Number(X);
    }
    return Bytes;
}
// Exact observable startup values from each REAL completed native graph.
// The original-process channel authenticates the controlled local sender.
// This is deliberately NOT a complete hidden-state or restore certificate.
std::vector<std::uint8_t> StartupPublication(const P::FPrivateRoadPublication& Actual)
{
    if(Actual.Frame!=0||!Actual.AwaitingInitialPublication||Actual.Actors.size()!=2)return {};
    auto Bytes=Observe(Actual);if(Bytes.empty())return {};
    Wire::Put(Bytes,1,8);Wire::Put(Bytes,300,4); // closed road recipe version/rate
    for(const auto& A:Actual.Actors)
    {
        if(A.CanMove||!A.Frozen||A.CountdownFrames!=1||!A.Input||A.Input->Frame!=0||A.Filter.Next!=1)return {};
        Wire::Put(Bytes,A.CanMove,1);Wire::Put(Bytes,A.Frozen,1);Wire::Put(Bytes,A.CountdownFrames,2);
        Wire::Put(Bytes,A.Filter.Next,8);Wire::Put(Bytes,A.Filter.ActiveMask,4);
        for(const auto& Values:{A.Filter.Applied,A.Input->Requested,A.Input->Applied})
            for(const auto Value:Values)Wire::Put(Bytes,U64(Value),2);
    }
    return Bytes;
}
std::vector<std::uint8_t> Activation(const std::vector<std::uint8_t>& Initial)
{std::vector<std::uint8_t> Out;Wire::Put(Out,1,8);Out.insert(Out.end(),Initial.begin(),Initial.end());return Out;}
std::optional<P::FPrivateRoadObservedFrame> ReadObservation(const std::vector<std::uint8_t>& Bytes)
{
    Wire::FReader R{Bytes.data(),Bytes.size()};P::FPrivateRoadObservedFrame Out;Out.Frame=R.Get(8);const auto Count=R.Get(1);if(Count!=2)return {};
    const auto Vector=[&](){const double X=std::bit_cast<double>(R.Get(8)),Y=std::bit_cast<double>(R.Get(8)),Z=std::bit_cast<double>(R.Get(8));return FVector(X,Y,Z);};
    for(unsigned A=0;A<2;++A)
    {
        P::FPrivateRoadObservedPose V;V.Actor=R.Get(8);V.Incarnation=R.Get(8);auto& K=V.COMState;
        K.Location=Vector();K.Velocity=Vector();K.Acceleration=Vector();K.AngularVelocity=Vector();K.AngularAcceleration=Vector();
        const double X=std::bit_cast<double>(R.Get(8)),Y=std::bit_cast<double>(R.Get(8)),Z=std::bit_cast<double>(R.Get(8)),W=std::bit_cast<double>(R.Get(8));K.Rotation=FQuat(X,Y,Z,W);
        if(V.Actor!=101+A||V.Incarnation!=3||K.Location.ContainsNaN()||K.Velocity.ContainsNaN()||K.Acceleration.ContainsNaN()
            ||K.AngularVelocity.ContainsNaN()||K.AngularAcceleration.ContainsNaN()||!K.Rotation.IsNormalized())return {};
        Out.Actors.push_back(V);
    }
    return R.Good&&R.At==Bytes.size()?std::optional<P::FPrivateRoadObservedFrame>(std::move(Out)):std::nullopt;
}
bool WaitRelease(const FNativePredictionProbeOptions& O,FNativeLocalSession& Channel,std::chrono::steady_clock::time_point Deadline)
{
    while(std::chrono::steady_clock::now()<Deadline)
    {
        std::ifstream Release(O.ReleasePath);U64 Session=0,Epoch=0;
        if(Release>>Session>>Epoch)return Session==O.Launcher.Session&&Epoch==O.Launcher.Epoch;
        if(!Channel.OriginalPeersAlive())return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
}
bool WriteResult(const FNativePredictionProbeOptions& O,U64 Through,U64 Corrections,double MaximumError,bool Equal,bool InitialSynchronized)
{
    if(!InitialSynchronized)return false;
    const auto Path=std::filesystem::path(O.ReportPath),Temp=std::filesystem::path(O.ReportPath+".tmp");
    std::ofstream S(Temp,std::ios::trunc);if(!S)return false;
    S<<"{\"scope\":\"three_process_native_road_probe_only\",\"session\":"<<O.Launcher.Session<<",\"peer\":"<<O.Launcher.SelfPeer
        <<",\"original_pid\":"<<O.Launcher.OriginalSelf.Pid<<",\"original_birth\":"<<O.Launcher.OriginalSelf.BirthTicks
        <<",\"through_frame\":"<<Through<<",\"corrections\":"<<Corrections<<",\"maximum_position_error_cm\":"<<MaximumError
        <<",\"final_exact_comparison\":"<<(O.Launcher.SelfPeer==Server?"null":Equal?"true":"false")
        <<",\"clients_reported_final_exact\":"<<(O.Launcher.SelfPeer==Server?"true":"null")
        <<",\"initial_publication_matched\":true,\"initial_publication_frame\":0,\"activation_frame\":1,\"startup_full_hidden_state_certificate\":false,\"hardware\":false,\"real_demo_game_client\":false,\"complete_state_restore\":false,\"localized_rollback\":false}\n";
    S.close();if(!S)return false;std::error_code Error;std::filesystem::rename(Temp,Path,Error);return !Error;
}
}
bool RunNativePredictionProcessProbe(const FNativePredictionProbeOptions& O)
{
    using namespace ProcessProbeDetail;namespace P=Prediction;
    FString Why;U64 TraceNext=0,TracePacket=0;int TraceTransport=-1,TraceInput=-1;const TCHAR* Phase=TEXT("open");
    const auto Fail=[&](int Line)
    {
        UE_LOG(LogTemp,Display,TEXT("[IASNativeProbeFailure] peer=%llu phase=%s line=%d next=%llu packet=%llu transport=%d input=%d reason=%s"),
            O.Launcher.SelfPeer,Phase,Line,TraceNext,TracePacket,TraceTransport,TraceInput,*Why);
        return false;
    };
    if(!IsInGameThread()||O.ReportPath.empty()||O.ReleasePath.empty()||O.Launcher.Peers.size()!=2
        ||(O.Launcher.SelfPeer!=Server&&O.Launcher.SelfPeer!=PeerA&&O.Launcher.SelfPeer!=PeerB))return Fail(__LINE__);
    auto Channel=FNativeLocalSession::Open(O.Launcher);if(!Channel)return Fail(__LINE__);
    const auto Deadline=std::chrono::steady_clock::now()+std::chrono::seconds(90);
    const auto Send=[&](U64 Peer,std::uint8_t Kind,const std::vector<std::uint8_t>& Bytes)
    {const auto S=Channel->Send(Peer,Kind,Bytes);TraceTransport=int(S);return S==ELocalSession::Ready||S==ELocalSession::WouldBlock;};
    const auto SendPackets=[&](U64 Peer,std::uint8_t Kind,const std::vector<FPacket>& Packets)
    {
        if(Packets.empty())return true;FRedundantRecords Records;
        for(const auto& Pkt:Packets){auto Encoded=Encode(Pkt);if(!Encoded)return Fail(__LINE__);Records.push_back(std::move(*Encoded));}
        const auto Groups=PackRedundantDatagrams(ERedundantFamily::Input,Records);if(!Groups)return Fail(__LINE__);
        for(const auto& Group:*Groups)if(!Send(Peer,Kind,Group))return Fail(__LINE__);return true;
    };
    const auto IsPacketGroup=[](const FLocalSessionMessage& M)->std::optional<std::vector<FPacket>>
    {
        const auto G=DecodeRedundantDatagram(M.Bytes.data(),M.Bytes.size());if(!G||G->Family!=ERedundantFamily::Input)return {};
        std::vector<FPacket> Out;for(const auto& B:G->Records){auto Pkt=Decode(B.data(),B.size());if(!Pkt)return {};Out.push_back(std::move(*Pkt));}return Out;
    };
    if(O.Launcher.SelfPeer==Server)
    {
        FExactInbox IA(Binding(0),EKind::OwnerInput,0,GameValues),IB(Binding(1),EKind::OwnerInput,0,GameValues);
        std::array<Consumed::FActionPolicy,ActionCount> Actions{};Actions[0]={true,true,false,255};Actions[1]={true,true,false,255};Actions[2]={true,true,true,127};
        P::FRemoteRequestHistory HA(Binding(0),Actions,512,512),HB(Binding(1),Actions,512,512);
        std::unique_ptr<P::FPrivateRoadPhysicsWorld> World;std::vector<FPacket> History[2];
        bool HelloSeen[2]{},ErrorSeen[2]{},Done[2]{},StartupAck[2]{},Started=false;U64 Next=0;std::vector<std::uint8_t> LastObservation,Initial;
        while(std::chrono::steady_clock::now()<Deadline)
        {
            Phase=TEXT("server-receive");TraceNext=Next;
            for(unsigned Work=0;Work<512;++Work)
            {
                FLocalSessionMessage M;const auto Status=Channel->Receive(M);TraceTransport=int(Status);if(Status==ELocalSession::WouldBlock)break;
                if(Status==ELocalSession::Pending||Status==ELocalSession::Duplicate||Status==ELocalSession::Stale||Status==ELocalSession::UnknownPeer)continue;
                if(Status!=ELocalSession::Ready|| (M.Peer!=PeerA&&M.Peer!=PeerB))return Fail(__LINE__);
                const unsigned A=M.Peer==PeerA?0:1;
                if(M.Kind==Hello||M.Kind==ObservedError90||M.Kind==ClientDone)
                {
                    if(M.Bytes!=std::vector<std::uint8_t>{1})return Fail(__LINE__);
                    if(M.Kind==Hello)HelloSeen[A]=true;
                    else if(M.Kind==ObservedError90)ErrorSeen[A]=true;
                    else Done[A]=true;
                }
                else if(M.Kind==StartupReady)
                {
                    if(Initial.empty()||M.Bytes!=Initial)return Fail(__LINE__);
                    StartupAck[A]=true;
                }
                else if(M.Kind==OwnerInput)
                {
                    auto Packets=IsPacketGroup(M);if(!Packets)return Fail(__LINE__);
                    for(const auto& Pkt:*Packets)
                    {
                        TracePacket=Pkt.Frame;auto& Inbox=A?IB:IA;const auto Result=Inbox.Receive(M.Peer,Pkt);TraceInput=int(Result);
                        if(Result!=EResult::Accepted&&Result!=EResult::Duplicate&&Result!=EResult::Stale&&Result!=EResult::TooFar)return Fail(__LINE__);
                        if(Result==EResult::Accepted||Result==EResult::Duplicate)
                            if(!SendPackets(M.Peer,RetentionAck,{AckFor(Pkt)}))return Fail(__LINE__);
                    }
                }
                else return Fail(__LINE__);
            }
            Phase=TEXT("server-physics");
            if(HelloSeen[0]&&HelloSeen[1]&&!World){World=P::FPrivateRoadPhysicsWorld::Create(Seeds(Server),Why);if(!World)return Fail(__LINE__);}
            if(World&&Next==1&&!Started&&StartupAck[0]&&StartupAck[1])
            {
                Phase=TEXT("server-start-activation");
                if(!World->ReleaseInitialPublication(World->ReadLatest(),Why))return Fail(__LINE__);
                Started=true;
            }
            const bool Barrier=(Next==1&&!Started)||(Next==91&&!(ErrorSeen[0]&&ErrorSeen[1]));
            if(World&&Next<Frames&&!Barrier)
            {
                // Transport cursors may advance independently; their completed
                // immutable history retains a half-pair until its mate arrives.
                // The physical world advances only when BOTH exact frames exist.
                auto A=IA.Consume(Next),B=IB.Consume(Next);
                if(A&&B)
                {
                    A->Packet.Kind=B->Packet.Kind=EKind::AuthorityInput;
                    if(HA.Confirm(Server,A->Packet).Status!=EResult::Accepted||HB.Confirm(Server,B->Packet).Status!=EResult::Accepted)return Fail(__LINE__);
                    if(!World->Step({{101,3,HA.Predict(Next)},{102,3,HB.Predict(Next)}},false,Why))return Fail(__LINE__);
                    History[0].push_back(A->Packet);History[1].push_back(B->Packet);
                    auto Actual=World->ReadLatest();if(!Actual||Actual->Frame!=Next)return Fail(__LINE__);LastObservation=Observe(*Actual);if(LastObservation.empty())return Fail(__LINE__);
                    if(Next==0){Initial=StartupPublication(*Actual);if(Initial.empty())return Fail(__LINE__);}
                    ++Next;
                }
            }
            Phase=TEXT("server-publish");TraceNext=Next;
            if(!Initial.empty())for(U64 Peer:{PeerA,PeerB})
                if(!Send(Peer,Started?StartActivation:InitialPublication,Started?Activation(Initial):Initial))return Fail(__LINE__);
            if(!LastObservation.empty())for(U64 Peer:{PeerA,PeerB})if(!Send(Peer,Observed,LastObservation))return Fail(__LINE__);
            for(unsigned A=0;A<2;++A)
            {
                const auto& H=History[A];if(H.empty())continue;
                const bool Released=ErrorSeen[0]&&ErrorSeen[1];
                const std::size_t Available=Released?H.size():std::min<std::size_t>(H.size(),30);
                const auto First=Available>4?Available-4:0;
                std::vector<FPacket> Packets(H.begin()+First,H.begin()+Available);
                if(Released&&Next<=95&&H.size()>30)Packets.assign(H.begin()+30,H.end());
                if(!SendPackets(A?PeerA:PeerB,AuthorityInput,Packets))return Fail(__LINE__);
            }
            if(Next==Frames&&Done[0]&&Done[1])
            {
                if(!World->Close()||!WriteResult(O,Frames-1,0,0,true,Started))return Fail(__LINE__);
                const bool Released=WaitRelease(O,*Channel,Deadline);Channel->Close();return Released;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return Fail(__LINE__);
    }
    const unsigned Own=O.Launcher.SelfPeer==PeerA?0:1,Other=1-Own;
    auto Contract=Speed::Driving::MakeContract(Speed::Driving::FBindings{});if(!Contract)return Fail(__LINE__);
    FReliableOutbox Outbox(Binding(Own),Server);auto Baseline=Sample(Own,0,Contract);if(Outbox.Queue(Baseline.Packet)!=EResult::Accepted)return Fail(__LINE__);
    std::unique_ptr<P::FPrivateRoadPredictionSession> Client;U64 Next=0,Corrections=0;bool ErrorMeasured=false,Done=false,StartupReadySent=false,Started=false;double MaximumError=0;std::vector<std::uint8_t> Initial,LocalInitial;
    std::optional<P::FPrivateRoadObservedFrame> Final;
    while(std::chrono::steady_clock::now()<Deadline)
    {
        // Check the trusted release BEFORE touching a peer that may already be
        // exiting after reading that same release. Reports alone never release
        // the native job: the launcher must still verify all actual exits/drain.
        if(Done)
        {
            std::ifstream Release(O.ReleasePath);U64 Session=0,Epoch=0;
            if(Release>>Session>>Epoch)
            {
                if(Session!=O.Launcher.Session||Epoch!=O.Launcher.Epoch||!Client->Close())return Fail(__LINE__);
                Channel->Close();return true;
            }
        }
        Phase=TEXT("client-receive");TraceNext=Next;
        if(!Send(Server,Hello,{1}))return Fail(__LINE__);
        for(unsigned Work=0;Work<512;++Work)
        {
            FLocalSessionMessage M;const auto Status=Channel->Receive(M);TraceTransport=int(Status);if(Status==ELocalSession::WouldBlock)break;
            if(Status==ELocalSession::Pending||Status==ELocalSession::Duplicate||Status==ELocalSession::Stale||Status==ELocalSession::UnknownPeer)continue;
            if(Status!=ELocalSession::Ready||M.Peer!=Server)return Fail(__LINE__);
            if(M.Kind==RetentionAck||M.Kind==AuthorityInput)
            {
                auto Packets=IsPacketGroup(M);if(!Packets)return Fail(__LINE__);
                for(const auto& Pkt:*Packets)
                {
                    TracePacket=Pkt.Frame;
                    if(M.Kind==RetentionAck)
                    {
                        if(!Outbox.Acknowledge(Server,Pkt)
                            &&(Pkt.Kind!=EKind::Ack||!Binding(Own).Matches(Pkt)||Pkt.Frame>=std::max<U64>(1,Next)||Pkt.Sequence!=Pkt.Frame+1))return Fail(__LINE__);
                        continue;
                    }
                    if(Pkt.Actor!=101+Other||Pkt.Frame>=Frames)return Fail(__LINE__);
                    if(!Client){if(Pkt.Frame!=0)continue;Client=P::FPrivateRoadPredictionSession::Create(Seeds(O.Launcher.SelfPeer),512,Why);if(!Client)return Fail(__LINE__);}
                    const auto Result=Client->ConfirmRemote(Server,Pkt);TraceInput=int(Result.Status);
                    if(Result.Status!=EResult::Accepted&&Result.Status!=EResult::Duplicate&&Result.Status!=EResult::Stale)return Fail(__LINE__);
                }
            }
            else if(M.Kind==InitialPublication)
            {
                if(M.Bytes.empty()||(!Initial.empty()&&Initial!=M.Bytes))return Fail(__LINE__);
                Initial=M.Bytes;
            }
            else if(M.Kind==StartActivation)
            {
                if(Initial.empty()||M.Bytes!=Activation(Initial))return Fail(__LINE__);
                if(!Started)
                {
                    Phase=TEXT("client-start-activation");
                    if(!Client||Next!=1||!StartupReadySent||LocalInitial!=Initial
                        ||!Client->ReleaseInitialPublication(Why))return Fail(__LINE__);
                    Started=true;
                }
            }
            else if(M.Kind==Observed)
            {
                const auto Obs=ReadObservation(M.Bytes);if(!Obs||Obs->Frame>=Frames)return Fail(__LINE__);
                if(Obs->Frame==Frames-1)Final=*Obs;
                if(Client&&Obs->Frame==90)
                {
                    const auto D=Client->CompareObservedKinematics(*Obs);
                    if(D.Status==P::EPrivateRoadPoseComparison::Different)
                    {
                        for(const auto& A:D.Actors)MaximumError=std::max(MaximumError,A.Position);
                        if(MaximumError>1)ErrorMeasured=true;
                    }
                }
            }
            else return Fail(__LINE__);
        }
        Phase=TEXT("client-physics");
        if(Client&&Next<Frames&&(Next==0||Started)&&Outbox.Size()<64)
        {
            auto S=Next==0?Baseline:Sample(Own,Next,Contract);
            if(!Client->AdvanceLocal(S.Local,Why))return Fail(__LINE__);
            if(Next&&Outbox.Queue(S.Packet)!=EResult::Accepted)return Fail(__LINE__);++Next;
        }
        if(Client&&Next==1&&!Started&&!Initial.empty())
        {
            const auto Actual=Client->ReadLatest();if(!Actual)return Fail(__LINE__);
            LocalInitial=StartupPublication(*Actual);
            if(LocalInitial.empty()||LocalInitial!=Initial)return Fail(__LINE__);
            if(!Send(Server,StartupReady,LocalInitial))return Fail(__LINE__);
            StartupReadySent=true;
        }
        if(Client)
        {
            Phase=TEXT("client-correction");TraceNext=Next;
            const auto State=Client->AdvanceCorrection(256,Why);if(State==P::EPrivateRoadCorrection::Failed)return Fail(__LINE__);
            if(State==P::EPrivateRoadCorrection::Adopted)++Corrections;
            if(Final&&Next==Frames)
            {
                const auto Compare=Client->CompareObservedKinematics(*Final);
                if(Compare.Status==P::EPrivateRoadPoseComparison::Equal&&Corrections&&ErrorMeasured)Done=true;
            }
        }
        Phase=TEXT("client-send");
        std::vector<FPacket> Pending;Outbox.Retry([&](const FPacket& Pkt){Pending.push_back(Pkt);});
        if(!SendPackets(Server,OwnerInput,Pending))return Fail(__LINE__);
        if(ErrorMeasured&&!Send(Server,ObservedError90,{1}))return Fail(__LINE__);
        if(Done)
        {
            if(!Send(Server,ClientDone,{1}))return Fail(__LINE__);
            if(!std::filesystem::exists(O.ReportPath)&&!WriteResult(O,Frames-1,Corrections,MaximumError,true,Started))return Fail(__LINE__);
            std::ifstream Release(O.ReleasePath);U64 Session=0,Epoch=0;
            if(Release>>Session>>Epoch)
            {
                if(Session!=O.Launcher.Session||Epoch!=O.Launcher.Epoch||!Client->Close())return Fail(__LINE__);Channel->Close();return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return Fail(__LINE__);
}
}
