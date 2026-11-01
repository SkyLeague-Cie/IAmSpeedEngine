#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Netcode/transport/NativeDatagramSocket.h"
#include "IAmSpeed/Netcode/transport/RedundantDatagram.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadPhysicsWorld.h"
#include "IAmSpeed/Driving/DrivingInput.h"
#include "Misc/AutomationTest.h"
#include <chrono>
#include <thread>

namespace IASGroupedDatagramCampaign
{
using namespace Speed::Netcode;
namespace P=Speed::Netcode::Prediction;
struct FSample {FPacket Owner,Authority;std::vector<std::uint8_t> State;FRedundantRecords Fragments;};
bool Source(std::vector<FSample>& Samples,FStateBinding& StateBinding,FAutomationTestBase& Test)
{
    auto Contract=Speed::Driving::MakeContract(Speed::Driving::FBindings{});if(!Contract)return false;
    P::FPrivateRoadCarSeed Seed;Seed.Actor=101;Seed.Incarnation=3;Seed.Contract=Contract;
    Seed.Local={Speed::Input::EProducerKind::Device,42};Seed.LocalEpoch={3};
    auto& B=Seed.Remote;B.Session=1;B.Epoch=2;B.Actor=101;B.Incarnation=3;B.OwnerPeer=4;B.ServerPeer=5;
    B.SourceProducer=42;B.SourceKind=0;B.Contract[0]=7;
    Seed.Initial.Location=FVector(0,0,45);Seed.Initial.Velocity=FVector(500,0,0);
    FString Why;auto World=P::FPrivateRoadPhysicsWorld::Create({Seed},Why);
    if(!Test.TestTrue(*FString::Printf(TEXT("actual native publication source: %s"),*Why),bool(World)))return false;
    StateBinding.Session=B.Session;StateBinding.Epoch=B.Epoch;StateBinding.ServerPeer=B.ServerPeer;StateBinding.Schema[0]=9;
    std::array<Consumed::FActionPolicy,ActionCount> Actions{};
    Actions[0]={true,true,false,255};Actions[1]={true,true,false,255};Actions[2]={true,true,true,127};
    P::FRemoteRequestHistory History(B,Actions,256,512);
    for(U64 N=0;N<24;++N)
    {
        FSample S;auto& Q=S.Owner;Q.Kind=EKind::OwnerInput;Q.Session=B.Session;Q.Epoch=B.Epoch;Q.Actor=B.Actor;
        Q.Incarnation=B.Incarnation;Q.Contract=B.Contract;Q.Frame=N;Q.Sequence=N+1;
        Q.Input.SourceSequence=N+1;Q.Input.DeviceGeneration=1;Q.Input.ProducerId=B.SourceProducer;
        Q.Input.ProducerKind=B.SourceKind;Q.Input.Reset=N==0;Q.Input.Values[0]=int16(32+N*7);Q.Input.ActiveMask=1;
        S.Authority=Q;S.Authority.Kind=EKind::AuthorityInput;
        if(History.Confirm(5,S.Authority).Status!=EResult::Accepted||!World->Step({{101,3,History.Predict(N)}},false,Why))return false;
        const auto Actual=World->ReadLatest();if(!Actual||Actual->Frame!=N||Actual->Physics.Payload.IsEmpty())return false;
        // Transfer exact ACTUAL published bytes as an observational opaque subset.
        // Native IDs inside these bytes are NOT a semantic cross-process restore
        // format. Never install this as complete physical state or set coverage.
        FWorldState State;State.Sequence=N+1;State.Frame=N;State.RegistryGeneration=Actual->LocalKernelLifetime;
        State.WorldPayload.assign(Actual->Physics.Payload.GetData(),Actual->Physics.Payload.GetData()+Actual->Physics.Payload.Num());
        const auto Encoded=EncodeState(State);const auto Fragments=FragmentState(State,StateBinding);
        if(!Encoded||!Fragments)return false;S.State=*Encoded;
        for(const auto& Fragment:*Fragments){auto Bytes=EncodeFragment(Fragment);if(!Bytes)return false;S.Fragments.push_back(std::move(*Bytes));}
        Samples.push_back(std::move(S));
    }
    return Test.TestTrue(TEXT("actual native source graph retired before network-only unit"),World->Close());
}
struct FCounts {U64 Offered=0,Dropped=0,Sent=0,Bytes=0,Received=0,Duplicate=0,Future=0;};
struct FSend {FNativeDatagramSocket* Socket;FNativeEndpoint Destination;std::vector<std::uint8_t> Bytes;unsigned Lane;};
bool Measure(unsigned Depth,unsigned Impairment,const std::vector<FSample>& Samples,const FStateBinding& StateBinding,FAutomationTestBase& Test)
{
    FNativeDatagramSocket Owner,Server,Remote;
    if(Owner.Bind({})!=ENativeDatagram::Ready||Server.Bind({})!=ENativeDatagram::Ready||Remote.Bind({})!=ENativeDatagram::Ready)return false;
    const auto OE=Owner.LocalEndpoint(),SE=Server.LocalEndpoint(),RE=Remote.LocalEndpoint();
    if(!OE.Port||!SE.Port||!RE.Port||OE==SE||OE==RE||SE==RE)return false;
    const auto& First=Samples.front().Owner;
    FBinding B;B.Session=First.Session;B.Epoch=First.Epoch;B.Actor=First.Actor;B.Incarnation=First.Incarnation;
    B.Contract=First.Contract;B.OwnerPeer=4;B.ServerPeer=5;B.SourceProducer=First.Input.ProducerId;B.SourceKind=First.Input.ProducerKind;
    const auto Game=[](const FFrame& F){for(std::size_t A=3;A<ActionCount;++A)if(F.Values[A])return false;return true;};
    FExactInbox OwnerInbox(B,EKind::OwnerInput,0,Game),RemoteInbox(B,EKind::AuthorityInput,0,Game);
    FStateReceiver OwnerState(StateBinding),RemoteState(StateBinding);
    U64 OwnerStates=0,RemoteStates=0;FCounts Counts[4];
    const auto RetainInput=[&](FExactInbox& Inbox,const std::vector<std::uint8_t>& Bytes,bool Authority,FCounts& C)
    {
        auto Packet=Decode(Bytes.data(),Bytes.size());if(!Packet||Packet->Kind!=(Authority?EKind::AuthorityInput:EKind::OwnerInput))return false;
        const auto Result=Inbox.Receive(Authority?5:4,*Packet);
        if(Result==EResult::Duplicate||Result==EResult::Stale)++C.Duplicate;
        else if(Result!=EResult::Accepted)return false;
        for(;;)
        {
            auto Next=Inbox.Consume(Inbox.NextFrame());if(!Next)break;
            if(Next->Packet.Frame>=Samples.size()||!Same(Next->Packet,Authority?Samples[size_t(Next->Packet.Frame)].Authority:Samples[size_t(Next->Packet.Frame)].Owner))return false;
        }
        return true;
    };
    const auto RetainState=[&](FStateReceiver& Receiver,const std::vector<std::uint8_t>& Bytes,U64& Complete,FCounts& C)
    {
        const auto Fragment=DecodeFragment(Bytes.data(),Bytes.size());if(!Fragment||Fragment->Ack)return false;
        const auto Result=Receiver.Receive(5,*Fragment);
        if(Result==EResult::Duplicate||Result==EResult::Stale)++C.Duplicate;
        else if(Result==EResult::TooFar)++C.Future;
        else if(Result!=EResult::Accepted)return false;
        if(auto State=Receiver.TakeReady())
        {
            if(State->Sequence!=Complete+1||State->Frame>=Samples.size()||EncodeState(*State)!=std::optional<std::vector<std::uint8_t>>(Samples[size_t(State->Frame)].State))return false;
            ++Complete;
        }
        return true;
    };
    for(unsigned Pulse=0;Pulse<32;++Pulse)
    {
        const auto Last=std::min<std::size_t>(Pulse,Samples.size()-1),FirstIndex=Last+1>Depth?Last+1-Depth:0;
        FRedundantRecords O,A,S;
        for(std::size_t N=FirstIndex;N<=Last;++N)
        {
            auto OW=Encode(Samples[N].Owner),AW=Encode(Samples[N].Authority);if(!OW||!AW)return false;
            O.push_back(std::move(*OW));A.push_back(std::move(*AW));S.insert(S.end(),Samples[N].Fragments.begin(),Samples[N].Fragments.end());
        }
        std::vector<FSend> Queue;
        const auto Enqueue=[&](ERedundantFamily Family,const FRedundantRecords& Records,FNativeDatagramSocket& Socket,FNativeEndpoint To,unsigned Lane)
        {
            auto Groups=PackRedundantDatagrams(Family,Records);if(!Groups)return false;
            for(auto& Group:*Groups)Queue.push_back({&Socket,To,std::move(Group),Lane});return true;
        };
        if(!Enqueue(ERedundantFamily::Input,O,Owner,SE,0)||!Enqueue(ERedundantFamily::Input,A,Server,RE,1)
            ||!Enqueue(ERedundantFamily::StateFragment,S,Server,OE,2)||!Enqueue(ERedundantFamily::StateFragment,S,Server,RE,3))return false;
        if(Impairment==3)std::reverse(Queue.begin(),Queue.end());
        const bool Drop=(Impairment==1&&Pulse%4==1)||(Impairment==2&&Pulse%16>=5&&Pulse%16<=7);
        for(const auto& Item:Queue)
        {
            auto& C=Counts[Item.Lane];++C.Offered;if(Drop){++C.Dropped;continue;}
            const unsigned Copies=Impairment==4&&Pulse%3==0?2:1;
            for(unsigned Copy=0;Copy<Copies;++Copy)
            {if(Item.Socket->Send(Item.Destination,Item.Bytes)!=ENativeDatagram::Ready)return false;++C.Sent;C.Bytes+=Item.Bytes.size();}
        }
        const auto Deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(2);
        do
        {
            for(unsigned SocketIndex=0;SocketIndex<3;++SocketIndex)
            {
                auto& Socket=SocketIndex==0?Server:SocketIndex==1?Owner:Remote;
                for(unsigned Read=0;Read<1024;++Read)
                {
                    FNativeEndpoint ActualSource;std::vector<std::uint8_t> Bytes;
                    const auto Status=Socket.Receive(ActualSource,Bytes);if(Status==ENativeDatagram::WouldBlock)break;
                    if(Status!=ENativeDatagram::Ready)return false;
                    const auto Group=DecodeRedundantDatagram(Bytes.data(),Bytes.size());if(!Group)return false;
                    if(SocketIndex==0)
                    {
                        if(!(ActualSource==OE)||Group->Family!=ERedundantFamily::Input)return false;++Counts[0].Received;
                        for(const auto& Record:Group->Records)if(!RetainInput(OwnerInbox,Record,false,Counts[0]))return false;
                    }
                    else
                    {
                        if(!(ActualSource==SE))return false;
                        if(Group->Family==ERedundantFamily::Input)
                        {
                            if(SocketIndex!=2)return false;++Counts[1].Received;
                            for(const auto& Record:Group->Records)if(!RetainInput(RemoteInbox,Record,true,Counts[1]))return false;
                        }
                        else
                        {
                            auto& C=Counts[SocketIndex==1?2:3];++C.Received;
                            for(const auto& Record:Group->Records)
                                if(!RetainState(SocketIndex==1?OwnerState:RemoteState,Record,SocketIndex==1?OwnerStates:RemoteStates,C))return false;
                        }
                    }
                }
            }
            std::this_thread::yield();
        }while(std::chrono::steady_clock::now()<Deadline);
    }
    const TCHAR* Names[]={TEXT("none"),TEXT("periodic-quarter"),TEXT("burst-three"),TEXT("reverse"),TEXT("duplicates")};
    const auto Complete=U64(Samples.size());
    for(unsigned Lane=0;Lane<4;++Lane)
    {
        const U64 Retained=Lane==0?OwnerInbox.NextFrame():Lane==1?RemoteInbox.NextFrame():Lane==2?OwnerStates:RemoteStates;
        const auto& C=Counts[Lane];
        Test.AddInfo(FString::Printf(TEXT("[IASDatagramLossMeasure] depth=%u impairment=%s lane=%u samples=24 pulses=32 offered=%llu injectedDrop=%llu sent=%llu received=%llu payloadBytes=%llu duplicatesOrStale=%llu futureFragments=%llu contiguous=%llu missing=%llu ueNetcode=0 actualOsSockets=3 separateProcesses=0 completePhysicalRestore=0 clock=logical-pulses"),
            Depth,Names[Impairment],Lane,C.Offered,C.Dropped,C.Sent,C.Received,C.Bytes,C.Duplicate,C.Future,Retained,Complete-Retained));
        const bool ShouldRecover=Impairment==0||Impairment==3||Impairment==4||(Impairment==1&&Depth>=2)||(Impairment==2&&Depth>=4);
        if(ShouldRecover&&!Test.TestEqual(TEXT("bounded original-record redundancy recovers all ordered samples for this impairment"),Retained,Complete))return false;
        if(Depth==1&&(Impairment==1||Impairment==2)&&!Test.TestTrue(TEXT("no historical redundancy positive control retains measurable gaps"),Retained<Complete))return false;
    }
    Owner.Close();Server.Close();Remote.Close();
    FNativeEndpoint From;std::vector<std::uint8_t> Bytes;
    return Test.TestTrue(TEXT("own OS handles actually closed"),!Owner.LocalEndpoint().Port&&!Server.LocalEndpoint().Port&&!Remote.LocalEndpoint().Port&&Owner.Receive(From,Bytes)==ENativeDatagram::Closed);
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASNativeGroupedDatagram,"IAmSpeed.Netcode.Transport.NativeGroupedDatagramLoss",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASNativeGroupedDatagram::RunTest(const FString&)
{
#if PLATFORM_WINDOWS
    using namespace IASGroupedDatagramCampaign;std::vector<FSample> Samples;FStateBinding Binding;
    if(!Source(Samples,Binding,*this))return false;
    const auto Original=Encode(Samples[0].Owner);if(!Original)return false;
    const auto Packed=PackRedundantDatagrams(ERedundantFamily::Input,{*Original});if(!Packed||Packed->size()!=1)return false;
    for(std::size_t Length=0;Length<Packed->front().size();++Length)
        if(!TestFalse(TEXT("every truncated grouped datagram is refused before retaining records"),bool(DecodeRedundantDatagram(Packed->front().data(),Length))))return false;
    auto Trailing=Packed->front();Trailing.push_back(0);
    if(!TestFalse(TEXT("trailing grouped bytes refused"),bool(DecodeRedundantDatagram(Trailing.data(),Trailing.size())))
        ||!TestFalse(TEXT("wrong family cannot wrap input bytes as state"),bool(PackRedundantDatagrams(ERedundantFamily::StateFragment,{*Original}))))return false;
    for(unsigned Depth:{1u,2u,4u,8u})for(unsigned Impairment=0;Impairment<5;++Impairment)
        if(!Measure(Depth,Impairment,Samples,Binding,*this))return false;
    AddInfo(TEXT("Transport unit only: actual OS loopback datagrams and IAS codecs; unit-authored connection bindings, deterministic application-injected loss/reorder/duplicates, logical send pulses. State bytes come from actual native publications but are observational opaque subsets, NOT semantic authenticated/restorable fullstate. No ACK policy, selected production redundancy, WAN latency, three-process predicted match or rollback qualification."));
    return !HasAnyErrors();
#else
    AddError(TEXT("Native grouped datagram Windows implementation not available on this target"));return false;
#endif
}
#endif
