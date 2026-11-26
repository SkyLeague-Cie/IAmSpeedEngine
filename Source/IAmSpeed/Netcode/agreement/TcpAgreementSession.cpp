#if defined(__linux__) && !defined(_GNU_SOURCE)
#define _GNU_SOURCE 1
#endif
#include "TcpAgreementSession.h"
#include "HostAgreementTypes.h"
#include "IAmSpeed/Concurrency/AtomicSharedPtr.h"
#include <algorithm>
#include <climits>
#include <cstring>
#include <stdexcept>
#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#if !defined(__linux__)
#error This transport currently supports Windows and Linux only.
#endif
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace Speed::Netcode::Agreement
{
namespace
{
using FClock=std::chrono::steady_clock;
#if defined(_WIN32)
using FSocket=SOCKET;
constexpr FSocket Invalid=INVALID_SOCKET;
int LastError(){return WSAGetLastError();}
bool Retry(int E){return E==WSAEWOULDBLOCK||E==WSAEINPROGRESS||E==WSAEALREADY||E==WSAEINTR;}
void Close(FSocket S){closesocket(S);}
struct FSocketPlatform
{
    FSocketPlatform(){WSADATA W{};if(WSAStartup(MAKEWORD(2,2),&W))throw ESessionEnd::SocketFailure;}
    ~FSocketPlatform(){WSACleanup();}
};
#else
using FSocket=int;
constexpr FSocket Invalid=-1;
int LastError(){return errno;}
bool Retry(int E){return E==EWOULDBLOCK||E==EAGAIN||E==EINPROGRESS||E==EALREADY||E==EINTR;}
void Close(FSocket S){::close(S);}
struct FSocketPlatform{};
#endif
struct FOwnedSocket
{
    FSocket S=Invalid;
    ~FOwnedSocket(){if(S!=Invalid)Close(S);}
    FOwnedSocket()=default;
    FOwnedSocket(const FOwnedSocket&)=delete;
    FOwnedSocket& operator=(const FOwnedSocket&)=delete;
};
void Nonblocking(FSocket S)
{
#if defined(_WIN32)
    u_long One=1;if(ioctlsocket(S,FIONBIO,&One)||!SetHandleInformation(reinterpret_cast<HANDLE>(S),HANDLE_FLAG_INHERIT,0))throw ESessionEnd::SocketFailure;
#else
    const int F=fcntl(S,F_GETFL,0);if(F<0||fcntl(S,F_SETFL,F|O_NONBLOCK)<0||fcntl(S,F_SETFD,FD_CLOEXEC)<0)throw ESessionEnd::SocketFailure;
    if(S>=FD_SETSIZE)throw ESessionEnd::SocketFailure;
#endif
}
FSocket OpenSocket()
{
#if defined(_WIN32)
    const auto S=WSASocketW(AF_INET,SOCK_STREAM,IPPROTO_TCP,nullptr,0,WSA_FLAG_NO_HANDLE_INHERIT);
#else
    const auto S=socket(AF_INET,SOCK_STREAM|SOCK_CLOEXEC,IPPROTO_TCP);
#endif
    if(S==Invalid)throw ESessionEnd::SocketFailure;return S;
}
sockaddr_in Address(FIPv4Endpoint E)
{
    sockaddr_in A{};A.sin_family=AF_INET;A.sin_port=htons(E.Port);
    const auto V=(std::uint32_t(E.Address[0])<<24)|(std::uint32_t(E.Address[1])<<16)|(std::uint32_t(E.Address[2])<<8)|E.Address[3];
    A.sin_addr.s_addr=htonl(V);return A;
}
FIPv4Endpoint Endpoint(FSocket S,bool Peer)
{
    sockaddr_in A{};
#if defined(_WIN32)
    int Size=sizeof(A);
#else
    socklen_t Size=sizeof(A);
#endif
    const int Result=Peer?getpeername(S,reinterpret_cast<sockaddr*>(&A),&Size):getsockname(S,reinterpret_cast<sockaddr*>(&A),&Size);
    if(Result||A.sin_family!=AF_INET||std::size_t(Size)!=sizeof(A))throw ESessionEnd::SocketFailure;
    const auto V=ntohl(A.sin_addr.s_addr);
    return {{{std::uint8_t(V>>24),std::uint8_t(V>>16),std::uint8_t(V>>8),std::uint8_t(V)}},ntohs(A.sin_port)};
}
void Check(const std::atomic<bool>& Stop,FClock::time_point Deadline,ESessionEnd Timeout)
{if(Stop.load(std::memory_order_acquire))throw ESessionEnd::LocalStop;if(FClock::now()>=Deadline)throw Timeout;}
bool Ready(FSocket S,bool Write)
{
    fd_set Set;FD_ZERO(&Set);FD_SET(S,&Set);timeval T{0,2000};
#if defined(_WIN32)
    const int N=0;
#else
    const int N=S+1;
#endif
    const int R=select(N,Write?nullptr:&Set,Write?&Set:nullptr,nullptr,&T);
    if(R<0){if(Retry(LastError()))return false;throw ESessionEnd::SocketFailure;}return R>0;
}
void SendBytes(FSocket S,const FBytes& B,const std::atomic<bool>& Stop,FClock::time_point End)
{
    std::size_t At=0;
    while(At<B.size())
    {
        Check(Stop,End,ESessionEnd::HandshakeTimeout);if(!Ready(S,true))continue;
        const auto Count=static_cast<int>(std::min<std::size_t>(B.size()-At,16384));
#if defined(_WIN32)
        const int R=send(S,reinterpret_cast<const char*>(B.data()+At),Count,0);
#else
        const auto R=send(S,B.data()+At,std::size_t(Count),MSG_NOSIGNAL);
#endif
        if(R<0){if(Retry(LastError()))continue;throw ESessionEnd::SocketFailure;}
        if(!R)throw ESessionEnd::PeerClosed;At+=std::size_t(R);
    }
}
void ReceiveBytes(FSocket S,std::uint8_t* D,std::size_t Size,const std::atomic<bool>& Stop,FClock::time_point End)
{
    std::size_t At=0;
    while(At<Size)
    {
        Check(Stop,End,ESessionEnd::HandshakeTimeout);if(!Ready(S,false))continue;
        const auto Count=static_cast<int>(std::min<std::size_t>(Size-At,16384));
#if defined(_WIN32)
        const int R=recv(S,reinterpret_cast<char*>(D+At),Count,0);
#else
        const auto R=recv(S,D+At,std::size_t(Count),0);
#endif
        if(R<0){if(Retry(LastError()))continue;throw ESessionEnd::SocketFailure;}
        if(!R)throw ESessionEnd::PeerClosed;At+=std::size_t(R);
    }
}
FMessage ReceiveFrame(FSocket S,const std::atomic<bool>& Stop,FClock::time_point End)
{
    std::array<std::uint8_t,FWireCodec::FrameHeaderBytes> H{};ReceiveBytes(S,H.data(),H.size(),Stop,End);
    const auto N=FWireCodec::PayloadSize(H.data(),H.size());if(!N)throw ESessionEnd::ProtocolFailure;
    FBytes B(*N);ReceiveBytes(S,B.data(),B.size(),Stop,End);
    if(!FWireCodec::DecodeOffer(B.data(),B.size()))throw ESessionEnd::ProtocolFailure;
    return {static_cast<EMessageKind>(H[9]),std::move(B)};
}
void SendFrame(FSocket S,EMessageKind Kind,const FBytes& Offer,const std::atomic<bool>& Stop,FClock::time_point End)
{
    const auto Frame=FWireCodec::EncodeFrame(Kind,Offer);if(!Frame)throw ESessionEnd::ProtocolFailure;
    SendBytes(S,*Frame,Stop,End);
}
std::uint64_t NextInstance()
{
    static std::atomic<std::uint64_t> Next{1};auto N=Next.load(std::memory_order_relaxed);
    for(unsigned I=0;I<256;++I)
    {if(N==UINT64_MAX)return 0;if(Next.compare_exchange_strong(N,N+1,std::memory_order_relaxed))return N;}
    return 0;
}
bool ValidLimits(FSessionLimits L)
{return L.Handshake.count()>0&&L.Handshake.count()<=30000&&L.Lifetime>=L.Handshake&&L.Lifetime.count()<=3600000;}
}
struct FTcpAgreementSession::FImpl
{
    bool Server;FIPv4Endpoint Address;FBytes Expected;FSessionLimits Limits;
    std::array<std::uint8_t,4> BoundAddress{}; // published once by BoundPort release
    const std::thread::id Controller=std::this_thread::get_id();
    std::atomic<bool> Stop{false},Done{false},Started{false};
    std::atomic<std::uint16_t> BoundPort{0};
    std::atomic<ESessionPhase> Phase{ESessionPhase::Created};
    std::atomic<ESessionEnd> End{ESessionEnd::None};
    const std::shared_ptr<std::atomic<std::uint64_t>> Closed=std::make_shared<std::atomic<std::uint64_t>>(0);
    Speed::Concurrency::TAtomicSharedPtr<const FConnectionObservedAgreement> Latest;
    std::shared_ptr<const FEnrollmentBinding> Enrollment;
    std::thread Worker;
    FImpl(bool S,FIPv4Endpoint A,FBytes E,FSessionLimits L):Server(S),Address(A),Expected(std::move(E)),Limits(L){}
};
FTcpAgreementSession::FTcpAgreementSession(std::unique_ptr<FImpl> P):Impl(std::move(P)){}
FTcpAgreementSession::~FTcpAgreementSession()
{
    // The host must retain the session until its actual thread has stopped and
    // TryJoin succeeds. Never detach a worker holding this object or a socket.
    if(Impl->Worker.joinable())std::terminate();
}
std::unique_ptr<FTcpAgreementSession> FTcpAgreementSession::Listen(FIPv4Endpoint A,const FOffer& O,FSessionLimits L)
{
    if(!ValidLimits(L))return {};auto B=FWireCodec::EncodeOffer(O);if(!B)return {};
    return std::unique_ptr<FTcpAgreementSession>(new FTcpAgreementSession(std::make_unique<FImpl>(true,A,std::move(*B),L)));
}
std::unique_ptr<FTcpAgreementSession> FTcpAgreementSession::Connect(FIPv4Endpoint A,const FOffer& O,FSessionLimits L)
{
    if(!A.Port||!ValidLimits(L))return {};auto B=FWireCodec::EncodeOffer(O);if(!B)return {};
    return std::unique_ptr<FTcpAgreementSession>(new FTcpAgreementSession(std::make_unique<FImpl>(false,A,std::move(*B),L)));
}
std::unique_ptr<FTcpAgreementSession> FTcpAgreementSession::ListenEnrolled(FIPv4Endpoint A,const FOffer& O,FSessionLimits L,std::shared_ptr<const FEnrollmentBinding> E)
{
    auto S=Listen(A,O,L);if(!S||!E||S->Impl->Expected!=E->OfferBytes)return {};
    S->Impl->Enrollment=std::move(E);return S;
}
std::unique_ptr<FTcpAgreementSession> FTcpAgreementSession::ConnectEnrolled(FIPv4Endpoint A,const FOffer& O,FSessionLimits L,std::shared_ptr<const FEnrollmentBinding> E)
{
    auto S=Connect(A,O,L);if(!S||!E||S->Impl->Expected!=E->OfferBytes)return {};
    S->Impl->Enrollment=std::move(E);return S;
}
bool FTcpAgreementSession::Start()
{
    if(std::this_thread::get_id()!=Impl->Controller||Impl->Started.exchange(true))return false;
    Impl->Phase.store(ESessionPhase::Starting,std::memory_order_release);
    try{Impl->Worker=std::thread([this]{Run();});return true;}
    catch(...){Impl->Closed->store(1,std::memory_order_release);Impl->End.store(ESessionEnd::AllocationFailure);Impl->Phase.store(ESessionPhase::Stopped);Impl->Done.store(true,std::memory_order_release);return false;}
}
void FTcpAgreementSession::RequestStop() noexcept {Impl->Stop.store(true,std::memory_order_release);}
bool FTcpAgreementSession::TryJoin()
{
    if(std::this_thread::get_id()!=Impl->Controller)return false;
    if(!Impl->Started.load())return true;
    if(!Impl->Done.load(std::memory_order_acquire))return false;
    if(Impl->Worker.joinable())Impl->Worker.join();return true;
}
bool FTcpAgreementSession::HasStopped() const noexcept{return Impl->Done.load(std::memory_order_acquire);}
ESessionPhase FTcpAgreementSession::Phase() const noexcept{return Impl->Phase.load(std::memory_order_acquire);}
ESessionEnd FTcpAgreementSession::EndReason() const noexcept{return Impl->End.load(std::memory_order_acquire);}
std::optional<FIPv4Endpoint> FTcpAgreementSession::ReadBoundEndpoint() const noexcept
{const auto Port=Impl->BoundPort.load(std::memory_order_acquire);return Port?std::optional<FIPv4Endpoint>({Impl->BoundAddress,Port}):std::nullopt;}
std::shared_ptr<const FConnectionObservedAgreement> FTcpAgreementSession::ReadLatestObserved() const noexcept
{return Impl->Latest.load(std::memory_order_acquire);}
void FTcpAgreementSession::Run() noexcept
{
    auto& P=*Impl;ESessionEnd Reason=ESessionEnd::None;
    {
    try
    {
        FSocketPlatform Platform; (void)Platform;
        FOwnedSocket Listener,Stream;
        struct FCloseOnExit
        {
            FImpl& State;
            ~FCloseOnExit(){State.Closed->store(1,std::memory_order_release);State.Latest.store({},std::memory_order_release);}
        } CloseBeforeSockets{P};
        const auto StartedAt=FClock::now(),Handshake=StartedAt+P.Limits.Handshake,Lifetime=StartedAt+P.Limits.Lifetime;
        Check(P.Stop,Handshake,ESessionEnd::HandshakeTimeout);
        if(P.Server)
        {
            Listener.S=OpenSocket();Nonblocking(Listener.S);const auto A=Address(P.Address);
#if defined(_WIN32)
            const BOOL Exclusive=1;if(setsockopt(Listener.S,SOL_SOCKET,SO_EXCLUSIVEADDRUSE,reinterpret_cast<const char*>(&Exclusive),sizeof(Exclusive)))throw ESessionEnd::SocketFailure;
#endif
            if(bind(Listener.S,reinterpret_cast<const sockaddr*>(&A),sizeof(A))||listen(Listener.S,1))throw ESessionEnd::SocketFailure;
            const auto Local=Endpoint(Listener.S,false);P.BoundAddress=Local.Address;P.BoundPort.store(Local.Port,std::memory_order_release);P.Phase.store(ESessionPhase::Listening,std::memory_order_release);
            for(;;)
            {
                Check(P.Stop,Handshake,ESessionEnd::HandshakeTimeout);if(!Ready(Listener.S,false))continue;
#if defined(_WIN32)
                Stream.S=accept(Listener.S,nullptr,nullptr);
#else
                // Atomic descriptor flags: no fork/exec inheritance window
                // between accept and the subsequent verification below.
                Stream.S=accept4(Listener.S,nullptr,nullptr,SOCK_NONBLOCK|SOCK_CLOEXEC);
#endif
                if(Stream.S!=Invalid)break;if(!Retry(LastError()))throw ESessionEnd::SocketFailure;
            }
            Nonblocking(Stream.S);
        }
        else
        {
            P.Phase.store(ESessionPhase::Connecting,std::memory_order_release);Stream.S=OpenSocket();Nonblocking(Stream.S);const auto A=Address(P.Address);
            if(connect(Stream.S,reinterpret_cast<const sockaddr*>(&A),sizeof(A)))
            {
                if(!Retry(LastError()))throw ESessionEnd::SocketFailure;
                for(;;){Check(P.Stop,Handshake,ESessionEnd::HandshakeTimeout);if(Ready(Stream.S,true))break;}
                int Error=0;
#if defined(_WIN32)
                int Size=sizeof(Error);
#else
                socklen_t Size=sizeof(Error);
#endif
                if(getsockopt(Stream.S,SOL_SOCKET,SO_ERROR,reinterpret_cast<char*>(&Error),&Size)||Error)throw ESessionEnd::SocketFailure;
            }
            const auto Local=Endpoint(Stream.S,false);P.BoundAddress=Local.Address;P.BoundPort.store(Local.Port,std::memory_order_release);
        }
        const auto Local=Endpoint(Stream.S,false),Peer=Endpoint(Stream.S,true);const auto Instance=NextInstance();
        if(!Instance)throw ESessionEnd::SocketFailure;
        P.Phase.store(ESessionPhase::Negotiating,std::memory_order_release);
        if(P.Enrollment)
        {
            std::array<std::uint8_t,40> Prefix{'I','A','S','J',1,0,0,0};
            if(P.Server)
            {
                if(P.Enrollment->Host->ReadCloseFence()||P.Enrollment->Slot->ReadCloseFence())throw ESessionEnd::RosterClosed;
                ReceiveBytes(Stream.S,Prefix.data(),Prefix.size(),P.Stop,Handshake);
                unsigned Difference=0;const std::array<std::uint8_t,8> Header{'I','A','S','J',1,0,0,0};
                for(std::size_t I=0;I<Header.size();++I)Difference|=unsigned(Prefix[I]^Header[I]);
                for(std::size_t I=0;I<32;++I)Difference|=unsigned(Prefix[I+8]^P.Enrollment->Key[I]);
                if(Difference)throw ESessionEnd::EnrollmentRejected;
                if(P.Enrollment->Host->ReadCloseFence()||P.Enrollment->Slot->ReadCloseFence())throw ESessionEnd::RosterClosed;
            }
            else
            {
                for(std::size_t I=0;I<32;++I)Prefix[I+8]=P.Enrollment->Key[I];
                SendBytes(Stream.S,FBytes(Prefix.begin(),Prefix.end()),P.Stop,Handshake);
            }
        }
        // Only the server's actual received credential attaches host enrollment.
        // A client possessing/sending a permit does not prove server identity.
        const std::shared_ptr<const FConnectionObservation> Origin(new FConnectionObservation(Instance,Local,Peer,P.Closed,P.Server?P.Enrollment:nullptr));
        FBytes Received;
        if(P.Server)
        {
            SendFrame(Stream.S,EMessageKind::Offer,P.Expected,P.Stop,Handshake);
            auto Reply=ReceiveFrame(Stream.S,P.Stop,Handshake);
            if(Reply.Kind==EMessageKind::Reject)throw ESessionEnd::PeerRejected;
            if(Reply.Kind!=EMessageKind::Accept||Reply.OfferBytes!=P.Expected)throw ESessionEnd::OfferMismatch;
            Received=std::move(Reply.OfferBytes);SendFrame(Stream.S,EMessageKind::Commit,P.Expected,P.Stop,Handshake);
        }
        else
        {
            auto Offer=ReceiveFrame(Stream.S,P.Stop,Handshake);
            if(Offer.Kind!=EMessageKind::Offer)throw ESessionEnd::ProtocolFailure;
            if(Offer.OfferBytes!=P.Expected)
            {SendFrame(Stream.S,EMessageKind::Reject,Offer.OfferBytes,P.Stop,Handshake);throw ESessionEnd::OfferMismatch;}
            SendFrame(Stream.S,EMessageKind::Accept,Offer.OfferBytes,P.Stop,Handshake);
            auto Commit=ReceiveFrame(Stream.S,P.Stop,Handshake);
            if(Commit.Kind!=EMessageKind::Commit||Commit.OfferBytes!=P.Expected)throw ESessionEnd::OfferMismatch;
            Received=std::move(Commit.OfferBytes);
        }
        if(P.Server&&P.Enrollment&&(P.Enrollment->Host->ReadCloseFence()||P.Enrollment->Slot->ReadCloseFence()))throw ESessionEnd::RosterClosed;
        const std::shared_ptr<const FConnectionObservedAgreement> Observed(new FConnectionObservedAgreement(Origin,P.Expected,std::move(Received),P.Server?EObservedStage::AcceptReceivedAtServer:EObservedStage::CommitReceivedAtClient));
        P.Latest.store(Observed,std::memory_order_release);P.Phase.store(ESessionPhase::Observed,std::memory_order_release);
        for(;;)
        {
            Check(P.Stop,Lifetime,ESessionEnd::LifetimeExpired);if(!Ready(Stream.S,false))continue;
            std::uint8_t Byte=0;
#if defined(_WIN32)
            const int N=recv(Stream.S,reinterpret_cast<char*>(&Byte),1,0);
#else
            const auto N=recv(Stream.S,&Byte,1,0);
#endif
            if(!N)throw ESessionEnd::PeerClosed;
            if(N<0&&Retry(LastError()))continue;
            throw N>0?ESessionEnd::ProtocolFailure:ESessionEnd::SocketFailure;
        }
    }
    catch(ESessionEnd E){Reason=E;}
    catch(...){Reason=ESessionEnd::AllocationFailure;}
    // All native socket/platform destructors have now returned on this thread.
    }
    P.Closed->store(1,std::memory_order_release);
    P.Latest.store({},std::memory_order_release);
    P.End.store(Reason,std::memory_order_release);P.Phase.store(ESessionPhase::Stopped,std::memory_order_release);P.Done.store(true,std::memory_order_release);
}
}
