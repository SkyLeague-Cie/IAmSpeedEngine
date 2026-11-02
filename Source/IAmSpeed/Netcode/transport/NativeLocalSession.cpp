#include "NativeLocalSession.h"
#include "NetStateTransport.h"
#include <algorithm>
#include <iterator>
#include <mutex>
#include <thread>

namespace Speed::Netcode
{
namespace LocalSessionDetail
{
constexpr std::size_t Header=60,Chunk=NativeDatagramBudget-Header,MaxMessage=32768,ReplayWindow=64,PendingLimit=32;
// The original startup recipe may occupy more than eight real datagrams.
// Track every permitted part; never accept a truncated/wrapped complete mask.
static_assert((MaxMessage+Chunk-1)/Chunk<64);

struct FClaimRegistry
{std::mutex Gate;std::vector<std::array<std::uint64_t,6>> Tuples;};
bool ClaimOnce(const FLocalSessionConfig& C)
{
    static FClaimRegistry Registry;std::lock_guard Lock(Registry.Gate);
    const std::array<std::uint64_t,6> Tuple{C.Session,C.Epoch,C.SelfPeer,C.SocketNonce,C.OriginalSelf.Pid,C.OriginalSelf.BirthTicks};
    if(Registry.Tuples.size()>=16||std::find(Registry.Tuples.begin(),Registry.Tuples.end(),Tuple)!=Registry.Tuples.end())return false;
    Registry.Tuples.push_back(Tuple);return true; // never reclaimed by idle/close
}
bool Loopback(const FNativeEndpoint& E)
{return E.Port&&E.Address==std::array<std::uint8_t,4>{127,0,0,1};}
struct FAssembly
{
    std::uint64_t Sequence=0;std::uint8_t Kind=0;std::uint64_t Mask=0;std::uint16_t Total=0,Count=0;std::uint32_t Crc=0;
    std::vector<std::uint8_t> Bytes;
};
struct FPeer
{
    FLocalSessionPeer Assigned;std::unique_ptr<FNativeLocalPeerPin> Pin;
    std::uint64_t High=0;std::vector<FAssembly> Partial,Completed;
};
}
struct FNativeLocalSession::FImpl
{
    std::mutex Gate;const std::thread::id Owner=std::this_thread::get_id();
    FLocalSessionConfig Config;FNativeDatagramSocket Socket;
    std::unique_ptr<FNativeLocalPeerPin> Self;std::vector<LocalSessionDetail::FPeer> Peers;
    std::uint64_t Next=1;bool Closed=false;
    bool Alive() const
    {if(Closed||!Self||!Self->OriginalProcessAlive())return false;for(const auto& P:Peers)if(!P.Pin->OriginalProcessAlive())return false;return true;}
    void Retire() noexcept {Socket.Close();Closed=true;}
};
FNativeLocalSession::FNativeLocalSession():Impl(std::make_unique<FImpl>()){}
FNativeLocalSession::~FNativeLocalSession(){Close();}
std::unique_ptr<FNativeLocalSession> FNativeLocalSession::Open(const FLocalSessionConfig& C)
{
    if(!C.Session||!C.Epoch||!C.SelfPeer||!C.SocketNonce||!LocalSessionDetail::Loopback(C.Endpoint)||C.Peers.empty()||C.Peers.size()>2)return {};
    const auto ActualSelf=FNativeLocalPeerPin::InspectCurrent();
    if(!ActualSelf||ActualSelf->Pid!=C.OriginalSelf.Pid||ActualSelf->BirthTicks!=C.OriginalSelf.BirthTicks)return {};
    auto Session=std::unique_ptr<FNativeLocalSession>(new FNativeLocalSession());auto& I=*Session->Impl;
    I.Config=C;I.Self=FNativeLocalPeerPin::PinOriginal(C.OriginalSelf);if(!I.Self)return {};
    for(const auto& P:C.Peers)
    {
        if(!P.Peer||P.Peer==C.SelfPeer||!P.SocketNonce||!LocalSessionDetail::Loopback(P.Endpoint)||P.Endpoint==C.Endpoint||P.OriginalProcess.Pid==C.OriginalSelf.Pid)return {};
        for(const auto& Old:I.Peers)if(Old.Assigned.Peer==P.Peer||Old.Assigned.Endpoint==P.Endpoint||Old.Assigned.OriginalProcess.Pid==P.OriginalProcess.Pid)return {};
        auto Pin=FNativeLocalPeerPin::PinOriginal(P.OriginalProcess);if(!Pin)return {};
        I.Peers.push_back({P,std::move(Pin),0,{},{}});
    }
    if(!LocalSessionDetail::ClaimOnce(C)||I.Socket.Bind(C.Endpoint)!=ENativeDatagram::Ready||!(I.Socket.LocalEndpoint()==C.Endpoint))return {};
    return Session;
}
ELocalSession FNativeLocalSession::Send(std::uint64_t Peer,std::uint8_t Kind,const std::vector<std::uint8_t>& Bytes)
{
    using namespace LocalSessionDetail;auto& I=*Impl;
    if(std::this_thread::get_id()!=I.Owner)return ELocalSession::WrongOwner;
    std::lock_guard Lock(I.Gate);if(I.Closed)return ELocalSession::Closed;
    if(!I.Alive()){I.Retire();return ELocalSession::PeerDead;}
    const auto P=std::find_if(I.Peers.begin(),I.Peers.end(),[&](const auto& V){return V.Assigned.Peer==Peer;});
    if(P==I.Peers.end())return ELocalSession::UnknownPeer;
    if(!Kind||Kind>16||Bytes.empty()||Bytes.size()>MaxMessage||I.Next==0)return ELocalSession::Invalid;
    const auto Sequence=I.Next;I.Next=Sequence==std::numeric_limits<std::uint64_t>::max()?0:Sequence+1;
    const auto Count=(Bytes.size()+Chunk-1)/Chunk;const auto Crc=StateCrc(Bytes);
    for(std::size_t N=0;N<Count;++N)
    {
        const auto First=N*Chunk,Length=std::min(Chunk,Bytes.size()-First);std::vector<std::uint8_t> Out;Out.reserve(Header+Length);
        Wire::Put(Out,0x314c5349,4);Wire::Put(Out,1,1);Wire::Put(Out,Kind,1);Wire::Put(Out,0,2);
        for(auto V:{I.Config.Session,I.Config.Epoch,I.Config.SelfPeer,I.Config.SocketNonce,Sequence})Wire::Put(Out,V,8);
        Wire::Put(Out,N,2);Wire::Put(Out,Count,2);Wire::Put(Out,Bytes.size(),2);Wire::Put(Out,Length,2);Wire::Put(Out,Crc,4);
        Out.insert(Out.end(),Bytes.begin()+First,Bytes.begin()+First+Length);
        const auto Status=I.Socket.Send(P->Assigned.Endpoint,Out);
        if(Status==ENativeDatagram::WouldBlock)return ELocalSession::WouldBlock;
        if(Status!=ENativeDatagram::Ready){I.Retire();return ELocalSession::SocketError;}
    }
    return ELocalSession::Ready;
}
ELocalSession FNativeLocalSession::Receive(FLocalSessionMessage& Message)
{
    using namespace LocalSessionDetail;auto& I=*Impl;Message={};
    if(std::this_thread::get_id()!=I.Owner)return ELocalSession::WrongOwner;
    std::lock_guard Lock(I.Gate);if(I.Closed)return ELocalSession::Closed;
    if(!I.Alive()){I.Retire();return ELocalSession::PeerDead;}
    FNativeEndpoint Source;std::vector<std::uint8_t> Bytes;const auto Status=I.Socket.Receive(Source,Bytes);
    if(Status==ENativeDatagram::WouldBlock)return ELocalSession::WouldBlock;
    // Keep the original process pins and bound socket. A late native module
    // may not have bound its assigned port yet; no packet/ACK is invented.
    if(Status==ENativeDatagram::PeerUnavailable)return ELocalSession::PeerUnavailable;
    if(Status!=ENativeDatagram::Ready)return ELocalSession::SocketError;
    auto P=std::find_if(I.Peers.begin(),I.Peers.end(),[&](const auto& V){return V.Assigned.Endpoint==Source;});
    if(P==I.Peers.end()||!P->Pin->MatchesObservedLoopbackEndpoint(Source))return ELocalSession::UnknownPeer;
    if(Bytes.size()<Header)return ELocalSession::Invalid;
    Wire::FReader R{Bytes.data(),Bytes.size()};
    if(R.Get(4)!=0x314c5349||R.Get(1)!=1)return ELocalSession::Invalid;
    const auto Kind=R.Get(1);if(!Kind||Kind>16||R.Get(2)||R.Get(8)!=I.Config.Session||R.Get(8)!=I.Config.Epoch
        ||R.Get(8)!=P->Assigned.Peer||R.Get(8)!=P->Assigned.SocketNonce)return ELocalSession::Invalid;
    const auto Sequence=R.Get(8),Index=R.Get(2),Count=R.Get(2),Total=R.Get(2),Length=R.Get(2),Crc=R.Get(4);
    if(!R.Good||!Sequence||!Total||Total>MaxMessage||Count!=(Total+Chunk-1)/Chunk||Index>=Count
        ||Length!=std::min<std::uint64_t>(Chunk,Total-Index*Chunk)||R.At!=Header||Length!=Bytes.size()-R.At)return ELocalSession::Invalid;
    if(P->High>=ReplayWindow&&Sequence<=P->High-ReplayWindow)return ELocalSession::Stale;
    // A late-starting peer may have missed many legitimate initial sends.
    // The first complete message establishes this transport replay window.
    if(P->High&&Sequence>P->High&&Sequence-P->High>4096)return ELocalSession::Invalid;
    const auto Compatible=[&](const FAssembly& V)
    {return V.Kind==Kind&&V.Total==Total&&V.Count==Count&&V.Crc==Crc;};
    const auto SamePart=[&](const FAssembly& V)
    {return std::equal(Bytes.begin()+Header,Bytes.end(),V.Bytes.begin()+Index*Chunk);};
    const auto Done=std::find_if(P->Completed.begin(),P->Completed.end(),[&](const auto& V){return V.Sequence==Sequence;});
    if(Done!=P->Completed.end())
    {if(Compatible(*Done)&&SamePart(*Done))return ELocalSession::Duplicate;I.Retire();return ELocalSession::Invalid;}
    auto A=std::find_if(P->Partial.begin(),P->Partial.end(),[&](const auto& V){return V.Sequence==Sequence;});
    if(A==P->Partial.end())
    {
        if(P->Partial.size()>=PendingLimit)return ELocalSession::Full;
        P->Partial.push_back({Sequence,std::uint8_t(Kind),0,std::uint16_t(Total),std::uint16_t(Count),std::uint32_t(Crc),std::vector<std::uint8_t>(Total)});
        A=std::prev(P->Partial.end());
    }
    if(!Compatible(*A)){I.Retire();return ELocalSession::Invalid;}
    const auto Bit=std::uint64_t{1}<<Index;
    if(A->Mask&Bit){if(SamePart(*A))return ELocalSession::Pending;I.Retire();return ELocalSession::Invalid;}
    std::copy(Bytes.begin()+Header,Bytes.end(),A->Bytes.begin()+Index*Chunk);A->Mask|=Bit;
    if(A->Mask!=((std::uint64_t{1}<<Count)-1))return ELocalSession::Pending;
    if(StateCrc(A->Bytes)!=A->Crc){I.Retire();return ELocalSession::Invalid;}
    Message={P->Assigned.Peer,Sequence,std::uint8_t(Kind),A->Bytes};P->High=std::max(P->High,Sequence);
    P->Completed.push_back(std::move(*A));P->Partial.erase(A);
    const auto Old=[&](const auto& V){return P->High>=ReplayWindow&&V.Sequence<=P->High-ReplayWindow;};
    std::erase_if(P->Completed,Old);std::erase_if(P->Partial,Old);
    return ELocalSession::Ready;
}
bool FNativeLocalSession::OriginalPeersAlive() const
{std::lock_guard Lock(Impl->Gate);return Impl->Alive();}
FNativeEndpoint FNativeLocalSession::LocalEndpoint() const
{std::lock_guard Lock(Impl->Gate);return Impl->Socket.LocalEndpoint();}
void FNativeLocalSession::Close() noexcept {std::lock_guard Lock(Impl->Gate);Impl->Retire();}
}
