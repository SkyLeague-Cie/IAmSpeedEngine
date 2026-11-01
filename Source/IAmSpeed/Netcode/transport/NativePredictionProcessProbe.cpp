#include "NativePredictionProcessProbe.h"
#include "RedundantDatagram.h"
#include "AuthorityInputJournal.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadPredictionSession.h"
#include "IAmSpeed/Driving/DrivingInput.h"
#include <bit>
#include <coroutine>
#include <exception>
#include <atomic>
#include <utility>
#include "Misc/ScopeExit.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>

namespace Speed::Netcode
{
namespace ProcessProbeDetail
{
namespace P=Prediction;namespace V=Speed::Input::V2;
constexpr U64 Frames=240,Server=5,PeerA=4,PeerB=7;
enum :std::uint8_t {Hello=1,OwnerInput=2,AuthorityInput=3,Observed=4,RetentionAck=5,ClientDone=6,ObservedDivergence=8,InitialPublication=9,StartupReady=10,StartActivation=11,AuthorityRetained=12};
struct FSample {FPacket Packet;std::shared_ptr<const V::FInputFrame> Local;};
bool GameValues(const Speed::Netcode::FFrame& F)
{if(F.Values[0]<0||F.Values[0]>255||F.Values[1]<0||F.Values[1]>255||F.Values[2]<-127||F.Values[2]>127)return false;for(std::size_t A=3;A<ActionCount;++A)if(F.Values[A])return false;return true;}
FBinding Binding(unsigned Actor)
{FBinding B;B.Session=11;B.Epoch=3;B.Actor=101+Actor;B.Incarnation=3;B.OwnerPeer=Actor?PeerB:PeerA;B.ServerPeer=Server;B.SourceProducer=42+Actor;B.SourceKind=0;B.Contract[0]=7;return B;}
U64 ObservationFrame(ENativeProbeScenario Scenario)
{return Scenario==ENativeProbeScenario::InteractingInputFixture?170:90;}
std::array<int16,3> Values(unsigned Actor,U64 N,ENativeProbeScenario Scenario)
{
    if(Scenario==ENativeProbeScenario::InteractingInputFixture)
    {
        if(!N)return {};
        const int16 Throttle=Actor?(N<30||((N-30)/30)%2?255:128):255;
        const int16 Brake=!Actor&&N>=20&&((N-20)/10)%2==0?255:0;
        return {Throttle,Brake,0};
    }
    return N<30?std::array<int16,3>{int16(Actor?255:128),0,0}:N<60?std::array<int16,3>{0,int16(Actor?255:128),0}:std::array<int16,3>{int16(Actor?128:192),0,int16(Actor?32:-32)};}
std::vector<P::FPrivateRoadCarSeed> Seeds(U64 Self,ENativeProbeScenario Scenario)
{
    auto Contract=Speed::Driving::MakeContract(Speed::Driving::FBindings{});std::vector<P::FPrivateRoadCarSeed> Out;
    for(unsigned A=0;A<2;++A)
    {
        auto B=Binding(A);P::FPrivateRoadCarSeed S;S.Actor=B.Actor;S.Incarnation=B.Incarnation;S.Remote=B;S.Contract=Contract;
        S.Role=Self==B.OwnerPeer?P::EBranchFilterRole::LocalDevice:P::EBranchFilterRole::RemoteRequests;
        S.Local={Speed::Input::EProducerKind::Device,B.SourceProducer};S.LocalEpoch={3}; // default closed initial-publication gate, no warmup countdown
        S.Initial.Location=Scenario==ENativeProbeScenario::InteractingInputFixture?FVector(A?70:-70,0,45):FVector(0,A?1000:-1000,45);
        S.Initial.Velocity=FVector::ZeroVector; // default synchronization freeze also zeroes it
        if(Scenario==ENativeProbeScenario::InteractingInputFixture&&A)S.Initial.Rotation=FQuat(FVector::UpVector,PI);
        S.Processing.Step[0]=17;S.Processing.Step[1]=17;S.Processing.Step[2]=9;Out.push_back(S);
    }
    return Out;
}
FSample Sample(unsigned Actor,U64 N,const std::shared_ptr<const V::FInputActionContract>& Contract,ENativeProbeScenario Scenario)
{
    const auto B=Binding(Actor);const auto Q=Values(Actor,N,Scenario);FSample S;auto& Pkt=S.Packet;
    Pkt.Kind=EKind::OwnerInput;Pkt.Session=B.Session;Pkt.Epoch=B.Epoch;Pkt.Actor=B.Actor;Pkt.Incarnation=B.Incarnation;
    Pkt.Contract=B.Contract;Pkt.Frame=N;Pkt.Sequence=N+1;Pkt.Input.SourceSequence=N+1;Pkt.Input.DeviceGeneration=1;
    Pkt.Input.ProducerId=B.SourceProducer;Pkt.Input.ProducerKind=B.SourceKind;Pkt.Input.Reset=N==0;
    V::FInputFrameData D;D.ConsumptionFrame=N;D.SourceSequence=N+1;D.DeviceGeneration={1};D.Producer={Speed::Input::EProducerKind::Device,B.SourceProducer};D.StreamEpoch={3};D.Reset=N==0;
    unsigned Ordinal=0;const auto Prev=N?Values(Actor,N-1,Scenario):std::array<int16,3>{};
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
// Called only AFTER the real native canonical step has committed every input.
// Never publish an inbox retention ACK as proof of physical consumption.
bool ActualConsumedPacket(const P::FPrivateRoadPublication& Actual,const FPacket& Packet)
{
    if(Actual.Frame!=Packet.Frame)return false;
    for(const auto& A:Actual.Actors)if(A.Actor==Packet.Actor&&A.Incarnation==Packet.Incarnation)
    {
        if(!A.Input||A.Input->Frame!=Packet.Frame||A.Input->Origin!=P::EBranchRequestOrigin::ReceivedAuthorityRequest)return false;
        const auto* Original=std::get_if<std::shared_ptr<const FPacket>>(&A.Input->Evidence);
        return Original&&*Original&&Encode(**Original)==Encode(Packet);
    }
    return false;
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
    if(Actual.Frame!=0||!Actual.AwaitingInitialPublication||Actual.Actors.size()!=2||Actual.NativeNetworkComponents||Actual.NativeDebugDelegateBindings||Actual.RenderScenePresent||Actual.PhysicsScenePresent)return {};
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
bool WriteResult(const FNativePredictionProbeOptions& O,U64 Through,U64 Corrections,double MaximumError,bool Equal,bool InitialSynchronized,U64 OwnerAuthorityCount,U64 RemoteAuthorityCount,U64 ActualVehicleContactFrames)
{
    if(!InitialSynchronized||OwnerAuthorityCount!=Frames||RemoteAuthorityCount!=Frames
        ||(O.Scenario==ENativeProbeScenario::InteractingInputFixture&&!ActualVehicleContactFrames))return false;
    const auto Path=std::filesystem::path(O.ReportPath),Temp=std::filesystem::path(O.ReportPath+".tmp");
    std::ofstream S(Temp,std::ios::trunc);if(!S)return false;
    std::ostringstream PositionErrorText;PositionErrorText<<std::setprecision(17)<<MaximumError;
    const auto PositionError=O.Launcher.SelfPeer==Server?std::string("null"):PositionErrorText.str();
    S<<"{\"scope\":\"three_process_native_road_probe_only\",\"session\":"<<O.Launcher.Session<<",\"peer\":"<<O.Launcher.SelfPeer
        <<",\"original_pid\":"<<O.Launcher.OriginalSelf.Pid<<",\"original_birth\":"<<O.Launcher.OriginalSelf.BirthTicks
        <<",\"through_frame\":"<<Through<<",\"corrections\":"<<Corrections<<",\"maximum_position_error_cm\":"<<PositionError
        <<",\"position_error_observation_frame\":"<<(O.Launcher.SelfPeer==Server?"null":std::to_string(ObservationFrame(O.Scenario)))
        <<",\"position_error_scope\":\"actors_at_observation_frame_only\""
        <<",\"unit_contact_fixture\":"<<(O.Scenario==ENativeProbeScenario::InteractingInputFixture?"true":"false")
        <<",\"actual_native_vehicle_contact_frames_in_final_graph\":"<<ActualVehicleContactFrames
        <<",\"final_exact_comparison\":"<<(O.Launcher.SelfPeer==Server?"null":Equal?"true":"false")
        <<",\"clients_reported_final_exact\":"<<(O.Launcher.SelfPeer==Server?"true":"null")
        <<",\"owner_authority_input_count\":"<<(O.Launcher.SelfPeer==Server?"null":std::to_string(OwnerAuthorityCount))
        <<",\"remote_authority_input_count\":"<<(O.Launcher.SelfPeer==Server?"null":std::to_string(RemoteAuthorityCount))
        <<",\"authority_input_prefix_through\":"<<(O.Launcher.SelfPeer==Server?"null":std::to_string(Frames-1))
        <<",\"authority_retention_is_consumption_ack\":false,\"server_completed_input_sources_checked\":"<<(O.Launcher.SelfPeer==Server?"true":"null")
        <<",\"both_clients_retained_both_actor_journals\":"<<(O.Launcher.SelfPeer==Server?"true":"null")
        <<",\"initial_publication_matched\":true,\"initial_publication_frame\":0,\"activation_frame\":1,\"startup_full_hidden_state_certificate\":false,\"hardware\":false,\"real_demo_game_client\":false,\"complete_state_restore\":false,\"localized_rollback\":false}\n";
    S.close();if(!S)return false;std::error_code Error;std::filesystem::rename(Temp,Path,Error);return !Error;
}
}
// The suspended frame contains only owned C++ values and the genuine native
// graph/session owners. Suspension points are placed after native Step/input
// calls return, outside their canonical contexts/plans and native mutex scopes.
// This source layout is not a complete native lifetime/borrow certificate.
struct FNativeProbeTask
{
    struct promise_type
    {
        bool Result=false;std::exception_ptr Failure;
        std::shared_ptr<const Prediction::FPrivateRoadPublication> Latest;
        FNativeProbeTask get_return_object() noexcept;
        std::suspend_always initial_suspend() const noexcept {return {};}
        std::suspend_always final_suspend() const noexcept {return {};}
        void return_value(bool Value) noexcept {Result=Value;}
        void unhandled_exception() noexcept {Failure=std::current_exception();}
        std::suspend_always yield_value(std::shared_ptr<const Prediction::FPrivateRoadPublication> Value) noexcept
        {Latest=std::move(Value);return {};}
    };
    using FHandle=std::coroutine_handle<promise_type>;
    FHandle Handle;
    explicit FNativeProbeTask(FHandle H):Handle(H){}
    FNativeProbeTask(FNativeProbeTask&& Other) noexcept:Handle(std::exchange(Other.Handle,{})){}
    FNativeProbeTask(const FNativeProbeTask&)=delete;
    void Reset() noexcept {if(Handle){const auto Old=std::exchange(Handle,{});Old.destroy();}}
    ~FNativeProbeTask(){Reset();}
};
FNativeProbeTask FNativeProbeTask::promise_type::get_return_object() noexcept
{return FNativeProbeTask(FNativeProbeTask::FHandle::from_promise(*this));}
// Copy options into the coroutine frame: no caller stack/config reference may
// outlive its original call while the actual game thread is back in its host.
FNativeProbeTask MakeNativePredictionProbeTask(FNativePredictionProbeOptions O,std::shared_ptr<FNativeProbePumpDiagnostics> Diagnostics)
{
    using namespace ProcessProbeDetail;namespace P=Prediction;
    FString Why;U64 TraceNext=0,TracePacket=0;int TraceTransport=-1,TraceInput=-1;const TCHAR* Phase=TEXT("open");
    const auto Fail=[&](int Line)
    {
        UE_LOG(LogTemp,Display,TEXT("[IASNativeProbeFailure] peer=%llu phase=%s line=%d next=%llu packet=%llu transport=%d input=%d reason=%s"),
            O.Launcher.SelfPeer,Phase,Line,TraceNext,TracePacket,TraceTransport,TraceInput,*Why);
        return false;
    };
    if(O.Scenario>ENativeProbeScenario::InteractingInputFixture||!IsInGameThread()||!O.MaxReplayFramesPerPulse||O.MaxReplayFramesPerPulse>256||O.ReportPath.empty()||O.ReleasePath.empty()||O.Launcher.Peers.size()!=2
        ||(O.Launcher.SelfPeer!=Server&&O.Launcher.SelfPeer!=PeerA&&O.Launcher.SelfPeer!=PeerB))co_return Fail(__LINE__);
    auto Channel=FNativeLocalSession::Open(O.Launcher);if(!Channel)co_return Fail(__LINE__);
    const auto Deadline=std::chrono::steady_clock::now()+std::chrono::seconds(90);
    const auto Send=[&](U64 Peer,std::uint8_t Kind,const std::vector<std::uint8_t>& Bytes)
    {
        if(Kind==ClientDone)++Diagnostics->ClientDoneSendAttempts;
        const auto S=Channel->Send(Peer,Kind,Bytes);TraceTransport=int(S);
        if(Kind==ClientDone&&S==ELocalSession::Ready)++Diagnostics->ClientDoneDatagrams;
        return S==ELocalSession::Ready||S==ELocalSession::WouldBlock;
    };
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
        // Independent delivery retention for each original client and each actor.
        bool AuthorityAck[2][2][Frames]{};
        bool HelloSeen[2]{},ErrorSeen[2]{},Done[2]{},StartupAck[2]{},Started=false;U64 Next=0;std::vector<std::uint8_t> LastObservation,Initial;
        while(std::chrono::steady_clock::now()<Deadline)
        {
            Phase=TEXT("server-receive");TraceNext=Next;
            for(unsigned Work=0;Work<512;++Work)
            {
                FLocalSessionMessage M;const auto Status=Channel->Receive(M);TraceTransport=int(Status);if(Status==ELocalSession::WouldBlock)break;
                if(Status==ELocalSession::PeerUnavailable||Status==ELocalSession::Pending||Status==ELocalSession::Duplicate||Status==ELocalSession::Stale||Status==ELocalSession::UnknownPeer)continue;
                if(Status!=ELocalSession::Ready|| (M.Peer!=PeerA&&M.Peer!=PeerB))co_return Fail(__LINE__);
                const unsigned A=M.Peer==PeerA?0:1;
                if(M.Kind==Hello||M.Kind==ObservedDivergence||M.Kind==ClientDone)
                {
                    if(M.Bytes!=std::vector<std::uint8_t>{1})co_return Fail(__LINE__);
                    if(M.Kind==Hello)HelloSeen[A]=true;
                    else if(M.Kind==ObservedDivergence)ErrorSeen[A]=true;
                    else Done[A]=true;
                }
                else if(M.Kind==StartupReady)
                {
                    if(Initial.empty()||M.Bytes!=Initial)co_return Fail(__LINE__);
                    StartupAck[A]=true;
                }
                else if(M.Kind==AuthorityRetained)
                {
                    const auto Packets=IsPacketGroup(M);if(!Packets)co_return Fail(__LINE__);
                    for(const auto& Ack:*Packets)
                    {
                        if(Ack.Actor<101||Ack.Actor>102)co_return Fail(__LINE__);
                        const unsigned Actor=unsigned(Ack.Actor-101);
                        if(Ack.Frame>=History[Actor].size()||Encode(Ack)!=Encode(AckFor(History[Actor][size_t(Ack.Frame)])))co_return Fail(__LINE__);
                        AuthorityAck[A][Actor][Ack.Frame]=true; // retention ONLY
                    }
                }
                else if(M.Kind==OwnerInput)
                {
                    auto Packets=IsPacketGroup(M);if(!Packets)co_return Fail(__LINE__);
                    for(const auto& Pkt:*Packets)
                    {
                        TracePacket=Pkt.Frame;auto& Inbox=A?IB:IA;const auto Result=Inbox.Receive(M.Peer,Pkt);TraceInput=int(Result);
                        if(Result!=EResult::Accepted&&Result!=EResult::Duplicate&&Result!=EResult::Stale&&Result!=EResult::TooFar)co_return Fail(__LINE__);
                        if(Result==EResult::Accepted||Result==EResult::Duplicate)
                            if(!SendPackets(M.Peer,RetentionAck,{AckFor(Pkt)}))co_return Fail(__LINE__);
                    }
                }
                else co_return Fail(__LINE__);
            }
            Phase=TEXT("server-physics");
            if(HelloSeen[0]&&HelloSeen[1]&&!World){World=P::FPrivateRoadPhysicsWorld::Create(Seeds(Server,O.Scenario),Why);if(!World)co_return Fail(__LINE__);}
            if(World&&Next==1&&!Started&&StartupAck[0]&&StartupAck[1])
            {
                Phase=TEXT("server-start-activation");
                if(!World->ReleaseInitialPublication(World->ReadLatest(),Why))co_return Fail(__LINE__);
                Started=true;
            }
            const bool Barrier=(Next==1&&!Started)||(Next==ObservationFrame(O.Scenario)+1&&!(ErrorSeen[0]&&ErrorSeen[1]));
            if(World&&Next<Frames&&!Barrier)
            {
                // Transport cursors may advance independently; their completed
                // immutable history retains a half-pair until its mate arrives.
                // The physical world advances only when BOTH exact frames exist.
                auto A=IA.Consume(Next),B=IB.Consume(Next);
                if(A&&B)
                {
                    A->Packet.Kind=B->Packet.Kind=EKind::AuthorityInput;
                    if(HA.Confirm(Server,A->Packet).Status!=EResult::Accepted||HB.Confirm(Server,B->Packet).Status!=EResult::Accepted)co_return Fail(__LINE__);
                    if(!World->Step({{101,3,HA.Predict(Next)},{102,3,HB.Predict(Next)}},false,Why))co_return Fail(__LINE__);
                    auto Actual=World->ReadLatest();
                    if(!Actual||Actual->Frame!=Next||!ActualConsumedPacket(*Actual,A->Packet)||!ActualConsumedPacket(*Actual,B->Packet))co_return Fail(__LINE__);
                    History[0].push_back(A->Packet);History[1].push_back(B->Packet);
                    LastObservation=Observe(*Actual);if(LastObservation.empty())co_return Fail(__LINE__);
                    if(Next==0){Initial=StartupPublication(*Actual);if(Initial.empty())co_return Fail(__LINE__);}
                    ++Next;
                    co_yield Actual; // actual completed native state, no live pointers
                }
            }
            Phase=TEXT("server-publish");TraceNext=Next;
            if(!Initial.empty())for(U64 Peer:{PeerA,PeerB})
                if(!Send(Peer,Started?StartActivation:InitialPublication,Started?Activation(Initial):Initial))co_return Fail(__LINE__);
            if(!LastObservation.empty())for(U64 Peer:{PeerA,PeerB})if(!Send(Peer,Observed,LastObservation))co_return Fail(__LINE__);
            bool BothAuthorityJournalsRetained=Next==Frames;
            for(unsigned Receiver=0;Receiver<2;++Receiver)for(unsigned A=0;A<2;++A)
            {
                const auto& H=History[A];
                const bool Released=ErrorSeen[0]&&ErrorSeen[1];
                // Preserve deliberate REMOTE withholding for the divergence
                // scenario; the owner also receives its actual consumed journal.
                const std::size_t Available=Receiver==A||Released?H.size():std::min<std::size_t>(H.size(),30);
                std::vector<FPacket> Packets;
                for(std::size_t N=0;N<H.size();++N)if(!AuthorityAck[Receiver][A][N])
                {
                    BothAuthorityJournalsRetained=false;
                    if(N<Available&&Packets.size()<16)Packets.push_back(H[N]);
                }
                if(!SendPackets(Receiver?PeerB:PeerA,AuthorityInput,Packets))co_return Fail(__LINE__);
            }
            if(Next==Frames&&Done[0]&&Done[1]&&BothAuthorityJournalsRetained)
            {
                const auto FinalNative=World->ReadLatest();if(!FinalNative)co_return Fail(__LINE__);
                if(!World->Close()||!WriteResult(O,Frames-1,0,0,true,Started,Frames,Frames,FinalNative->VehicleContactFrameCount))co_return Fail(__LINE__);
                while(std::chrono::steady_clock::now()<Deadline)
                {
                    // Destroy file/temporary values before giving the GT back.
                    bool Released=false,ValidRelease=false;
                    {std::ifstream Release(O.ReleasePath);U64 Session=0,Epoch=0;
                    if(Release>>Session>>Epoch){Released=true;ValidRelease=Session==O.Launcher.Session&&Epoch==O.Launcher.Epoch;}}
                    if(Released){Channel->Close();co_return ValidRelease;}
                    if(!Channel->OriginalPeersAlive())co_return false;
                    co_await std::suspend_always{};
                }
                Channel->Close();co_return false;
            }
            co_await std::suspend_always{};
        }
        co_return Fail(__LINE__);
    }
    const unsigned Own=O.Launcher.SelfPeer==PeerA?0:1,Other=1-Own;
    auto Contract=Speed::Driving::MakeContract(Speed::Driving::FBindings{});if(!Contract)co_return Fail(__LINE__);
    FReliableOutbox Outbox(Binding(Own),Server);auto Baseline=Sample(Own,0,Contract,O.Scenario);if(Outbox.Queue(Baseline.Packet)!=EResult::Accepted)co_return Fail(__LINE__);
    std::unique_ptr<P::FPrivateRoadPredictionSession> Client;U64 Next=0,Corrections=0;bool ErrorMeasured=false,Done=false,StartupReadySent=false,Started=false;double MaximumError=0;std::vector<std::uint8_t> Initial,LocalInitial;
    auto AuthorityJournal=FAuthorityInputJournal::Create({Binding(0),Binding(1)},Frames);if(!AuthorityJournal)co_return Fail(__LINE__);
    // Exact original owner frames remain Device evidence. A received server
    // record is checked against them; it is never relabelled as a Device frame.
    std::vector<FPacket> OriginalOwnerPackets{Baseline.Packet};
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
                if(Session!=O.Launcher.Session||Epoch!=O.Launcher.Epoch||!Client->Close())co_return Fail(__LINE__);
                Channel->Close();co_return true;
            }
        }
        Phase=TEXT("client-receive");TraceNext=Next;
        if(!Send(Server,Hello,{1}))co_return Fail(__LINE__);
        for(unsigned Work=0;Work<512;++Work)
        {
            FLocalSessionMessage M;const auto Status=Channel->Receive(M);TraceTransport=int(Status);if(Status==ELocalSession::WouldBlock)break;
            if(Status==ELocalSession::PeerUnavailable||Status==ELocalSession::Pending||Status==ELocalSession::Duplicate||Status==ELocalSession::Stale||Status==ELocalSession::UnknownPeer)continue;
            if(Status!=ELocalSession::Ready||M.Peer!=Server)co_return Fail(__LINE__);
            if(M.Kind==RetentionAck||M.Kind==AuthorityInput)
            {
                auto Packets=IsPacketGroup(M);if(!Packets)co_return Fail(__LINE__);
                for(const auto& Pkt:*Packets)
                {
                    TracePacket=Pkt.Frame;
                    if(M.Kind==RetentionAck)
                    {
                        if(!Outbox.Acknowledge(Server,Pkt)
                            &&(Pkt.Kind!=EKind::Ack||!Binding(Own).Matches(Pkt)||Pkt.Frame>=std::max<U64>(1,Next)||Pkt.Sequence!=Pkt.Frame+1))co_return Fail(__LINE__);
                        continue;
                    }
                    if((Pkt.Actor!=101+Own&&Pkt.Actor!=101+Other)||Pkt.Frame>=Frames||!GameValues(Pkt.Input))co_return Fail(__LINE__);
                    if(Pkt.Actor==101+Own)
                    {
                        if(Pkt.Frame>=OriginalOwnerPackets.size())co_return Fail(__LINE__);
                        auto Original=OriginalOwnerPackets[size_t(Pkt.Frame)];Original.Kind=EKind::AuthorityInput;
                        if(Encode(Original)!=Encode(Pkt))co_return Fail(__LINE__);
                    }
                    else
                    {
                        if(!Client){if(Pkt.Frame!=0)continue;Client=P::FPrivateRoadPredictionSession::Create(Seeds(O.Launcher.SelfPeer,O.Scenario),512,Why);if(!Client)co_return Fail(__LINE__);}
                        const auto Result=Client->ConfirmRemote(Server,Pkt);TraceInput=int(Result.Status);
                        if(Result.Status!=EResult::Accepted&&Result.Status!=EResult::Duplicate&&Result.Status!=EResult::Stale)co_return Fail(__LINE__);
                    }
                    const auto Retained=AuthorityJournal->Retain(Server,Pkt);
                    if(Retained!=EResult::Accepted&&Retained!=EResult::Duplicate)co_return Fail(__LINE__);
                    if(!SendPackets(Server,AuthorityRetained,{AckFor(Pkt)}))co_return Fail(__LINE__);
                }
            }
            else if(M.Kind==InitialPublication)
            {
                if(M.Bytes.empty()||(!Initial.empty()&&Initial!=M.Bytes))co_return Fail(__LINE__);
                Initial=M.Bytes;
            }
            else if(M.Kind==StartActivation)
            {
                if(Initial.empty()||M.Bytes!=Activation(Initial))co_return Fail(__LINE__);
                if(!Started)
                {
                    Phase=TEXT("client-start-activation");
                    if(!Client||Next!=1||!StartupReadySent||LocalInitial!=Initial
                        ||!Client->ReleaseInitialPublication(Why))co_return Fail(__LINE__);
                    Started=true;
                }
            }
            else if(M.Kind==Observed)
            {
                const auto Obs=ReadObservation(M.Bytes);if(!Obs||Obs->Frame>=Frames)co_return Fail(__LINE__);
                if(Obs->Frame==Frames-1)Final=*Obs;
                if(Client&&Obs->Frame==ObservationFrame(O.Scenario))
                {
                    const auto D=Client->CompareObservedKinematics(*Obs);
                    if(D.Status==P::EPrivateRoadPoseComparison::Different)
                    {
                        for(const auto& A:D.Actors)MaximumError=std::max(MaximumError,A.Position);
                        if(MaximumError>1)ErrorMeasured=true;
                    }
                }
            }
            else co_return Fail(__LINE__);
        }
        Phase=TEXT("client-physics");
        if(Client&&Next<Frames&&(Next==0||Started)&&Outbox.Size()<64)
        {
            auto S=Next==0?Baseline:Sample(Own,Next,Contract,O.Scenario);
            if(!Client->AdvanceLocal(S.Local,Why))co_return Fail(__LINE__);
            if(Next)
            {
                if(OriginalOwnerPackets.size()!=Next||Outbox.Queue(S.Packet)!=EResult::Accepted)co_return Fail(__LINE__);
                OriginalOwnerPackets.push_back(S.Packet);
            }
            ++Next;
        }
        if(Client&&Next==1&&!Started&&!Initial.empty())
        {
            const auto Actual=Client->ReadLatest();if(!Actual)co_return Fail(__LINE__);
            LocalInitial=StartupPublication(*Actual);
            if(LocalInitial.empty()||LocalInitial!=Initial)co_return Fail(__LINE__);
            if(!Send(Server,StartupReady,LocalInitial))co_return Fail(__LINE__);
            StartupReadySent=true;
        }
        if(Client)
        {
            Phase=TEXT("client-correction");TraceNext=Next;
            const auto State=Client->AdvanceCorrection(O.MaxReplayFramesPerPulse,Why);if(State==P::EPrivateRoadCorrection::Failed)co_return Fail(__LINE__);
            if(State==P::EPrivateRoadCorrection::Adopted)++Corrections;
            if(Final&&Next==Frames)
            {
                const auto Compare=Client->CompareObservedKinematics(*Final);
                if(Compare.Status==P::EPrivateRoadPoseComparison::Equal&&Corrections&&ErrorMeasured&&AuthorityJournal->CommonContiguousCount()==Frames)Done=true;
            }
        }
        if(Client)co_yield Client->ReadLatest(); // coherent forward/corrected native publication
        Phase=TEXT("client-send");
        std::vector<FPacket> Pending;Outbox.Retry([&](const FPacket& Pkt){Pending.push_back(Pkt);});
        if(!SendPackets(Server,OwnerInput,Pending))co_return Fail(__LINE__);
        if(ErrorMeasured&&!Send(Server,ObservedDivergence,{1}))co_return Fail(__LINE__);
        if(Done)
        {
            const auto FinalNative=Client->ReadLatest();if(!FinalNative)co_return Fail(__LINE__);
            if(!Send(Server,ClientDone,{1}))co_return Fail(__LINE__);
            if(!std::filesystem::exists(O.ReportPath)&&!WriteResult(O,Frames-1,Corrections,MaximumError,true,Started,AuthorityJournal->ContiguousCount(101+Own),AuthorityJournal->ContiguousCount(101+Other),FinalNative->VehicleContactFrameCount))co_return Fail(__LINE__);
            std::ifstream Release(O.ReleasePath);U64 Session=0,Epoch=0;
            if(Release>>Session>>Epoch)
            {
                if(Session!=O.Launcher.Session||Epoch!=O.Launcher.Epoch||!Client->Close())co_return Fail(__LINE__);Channel->Close();co_return true;
            }
        }
        co_await std::suspend_always{};
    }
    co_return Fail(__LINE__);
}

struct FNativePredictionProbePump::FImpl
{
    const std::thread::id Owner=std::this_thread::get_id();
    std::atomic<std::shared_ptr<const Prediction::FPrivateRoadPublication>> Published;
    ENativeProbePumpStep State=ENativeProbePumpStep::Running;
    std::string Error;
    bool Active=false;
    const std::shared_ptr<FNativeProbePumpDiagnostics> Diagnostics=std::make_shared<FNativeProbePumpDiagnostics>();
    // Destroy the coroutine and its real native graph FIRST. World cleanup
    // callbacks may query the still-live pump value publication during unwind.
    FNativeProbeTask Task;
    explicit FImpl(FNativePredictionProbeOptions O):Task(MakeNativePredictionProbeTask(std::move(O),Diagnostics)){}
};
FNativePredictionProbePump::FNativePredictionProbePump(FNativePredictionProbeOptions O):Impl(std::make_unique<FImpl>(std::move(O))){}
FNativePredictionProbePump::~FNativePredictionProbePump()
{
    if(!IsInGameThread()||std::this_thread::get_id()!=Impl->Owner||Impl->Active)std::terminate();
    // A reentrant cleanup callback cannot resume the destroying coroutine.
    Impl->Active=true;Impl->State=ENativeProbePumpStep::Failed;Impl->Published.store({});
    // FImpl destroys the actual suspended coroutine/RAII owners on this GT;
    // its publication/status remain alive until that real unwind completes.
}
std::unique_ptr<FNativePredictionProbePump> FNativePredictionProbePump::Create(FNativePredictionProbeOptions O,std::string& Why)
{
    Why.clear();if(!IsInGameThread()){Why="native probe pump requires the original game thread";return {};}
    return std::unique_ptr<FNativePredictionProbePump>(new FNativePredictionProbePump(std::move(O)));
}
ENativeProbePumpStep FNativePredictionProbePump::Tick()
{
    auto& I=*Impl;
    if(!IsInGameThread()||std::this_thread::get_id()!=I.Owner)return ENativeProbePumpStep::WrongOwner;
    if(I.Active)return ENativeProbePumpStep::Reentrant;
    if(I.State!=ENativeProbePumpStep::Running)return I.State;
    I.Active=true;ON_SCOPE_EXIT{I.Active=false;};
    I.Task.Handle.resume();
    const auto& Promise=I.Task.Handle.promise();
    I.Published.store(Promise.Latest);
    if(I.Task.Handle.done())
    {
        I.State=Promise.Result&&!Promise.Failure?ENativeProbePumpStep::Succeeded:ENativeProbePumpStep::Failed;
        if(Promise.Failure)
        {
            try{std::rethrow_exception(Promise.Failure);}
            catch(const std::exception& E){I.Error=E.what();}
            catch(...){I.Error="native probe exception";}
        }
        if(I.State==ENativeProbePumpStep::Failed){I.Published.store({});if(I.Error.empty())I.Error="native probe reported failure";}
    }
    return I.State;
}
ENativeProbePumpStep FNativePredictionProbePump::Cancel()
{
    auto& I=*Impl;
    if(!IsInGameThread()||std::this_thread::get_id()!=I.Owner)return ENativeProbePumpStep::WrongOwner;
    if(I.Active)return ENativeProbePumpStep::Reentrant;
    if(I.State!=ENativeProbePumpStep::Running)return I.State;
    // Allocate the diagnostic BEFORE the first cancellation mutation. A failed
    // allocation must leave a still-owned running coroutine, not a Cancelled
    // state whose native graph has not actually unwound.
    I.Error="native probe cancelled by its original owner";
    I.Active=true;ON_SCOPE_EXIT{I.Active=false;};
    I.State=ENativeProbePumpStep::Cancelled;I.Published.store({});
    // Detach the handle BEFORE native cleanup can call back into this pump.
    // State/counters/owner remain alive through the actual RAII unwind.
    I.Task.Reset();return I.State;
}
std::optional<FNativeProbePumpDiagnostics> FNativePredictionProbePump::Diagnostics() const
{
    if(!IsInGameThread()||std::this_thread::get_id()!=Impl->Owner)return {};
    return *Impl->Diagnostics;
}
std::shared_ptr<const Prediction::FPrivateRoadPublication> FNativePredictionProbePump::ReadLatest() const noexcept
{return Impl->Published.load();}
std::string FNativePredictionProbePump::FailureReason() const
{return IsInGameThread()&&std::this_thread::get_id()==Impl->Owner?Impl->Error:std::string("wrong native probe owner");}
bool RunNativePredictionProcessProbe(const FNativePredictionProbeOptions& O)
{
    std::string Why;auto Pump=FNativePredictionProbePump::Create(O,Why);if(!Pump)return false;
    U64 Resumes=0;ENativeProbePumpStep Result;
    while((Result=Pump->Tick())==ENativeProbePumpStep::Running){++Resumes;std::this_thread::sleep_for(std::chrono::milliseconds(1));}
    UE_LOG(LogTemp,Display,TEXT("[IASCooperativeNativeProbe] resumes=%llu success=%d scope=finite_unit_authored_probe_no_game_admission"),Resumes+1,Result==ENativeProbePumpStep::Succeeded?1:0);
    return Result==ENativeProbePumpStep::Succeeded;
}
}
