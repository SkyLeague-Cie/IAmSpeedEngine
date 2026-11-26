#pragma once
#include "AgreementWire.h"
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

namespace Speed::Netcode::Agreement
{
class FEnrollmentBinding;
class FHostAgreementHost;
struct FIPv4Endpoint
{
    std::array<std::uint8_t,4> Address{127,0,0,1};
    std::uint16_t Port=0;
};
enum class EObservedStage:std::uint8_t { AcceptReceivedAtServer,CommitReceivedAtClient };
enum class ESessionPhase:std::uint8_t { Created,Starting,Listening,Connecting,Negotiating,Observed,Stopped };
enum class ESessionEnd:std::uint8_t
{ None,LocalStop,HandshakeTimeout,LifetimeExpired,PeerClosed,PeerRejected,OfferMismatch,ProtocolFailure,SocketFailure,AllocationFailure,EnrollmentRejected,RosterClosed };
class FTcpAgreementSession;

// Actual local socket association, NOT account authentication or a native
// admission token. Constructor is unavailable to codecs and application code.
class FConnectionObservation final
{
public:
    std::uint64_t Instance() const noexcept{return LocalInstance;}
    const FIPv4Endpoint& LocalEndpoint() const noexcept{return Local;}
    const FIPv4Endpoint& PeerEndpoint() const noexcept{return Peer;}
    std::uint64_t ReadCloseFence() const noexcept{return Closed->load(std::memory_order_acquire);}
    bool MatchesHostEnrollment(const std::shared_ptr<const FEnrollmentBinding>& B)const noexcept{return B&&Enrollment==B;}
private:
    friend class FTcpAgreementSession;
    FConnectionObservation(std::uint64_t I,FIPv4Endpoint L,FIPv4Endpoint P,std::shared_ptr<std::atomic<std::uint64_t>> C,std::shared_ptr<const FEnrollmentBinding> E)
        :LocalInstance(I),Local(L),Peer(P),Closed(std::move(C)),Enrollment(std::move(E)){}
    const std::uint64_t LocalInstance;
    const FIPv4Endpoint Local,Peer;
    const std::shared_ptr<std::atomic<std::uint64_t>> Closed;
    const std::shared_ptr<const FEnrollmentBinding> Enrollment;
};
class FConnectionObservedAgreement final
{
public:
    const std::shared_ptr<const FConnectionObservation>& Connection() const noexcept{return Origin;}
    const FBytes& SentOfferCopy() const noexcept{return Sent;}
    const FBytes& ReceivedCopy() const noexcept{return Received;}
    EObservedStage Stage() const noexcept{return ReceivedStage;}
    std::thread::id ReceivingThread() const noexcept{return Thread;}
private:
    friend class FTcpAgreementSession;
    FConnectionObservedAgreement(std::shared_ptr<const FConnectionObservation> C,FBytes S,FBytes R,EObservedStage Stage)
        :Origin(std::move(C)),Sent(std::move(S)),Received(std::move(R)),ReceivedStage(Stage),Thread(std::this_thread::get_id()){}
    const std::shared_ptr<const FConnectionObservation> Origin;
    const FBytes Sent,Received;
    const EObservedStage ReceivedStage;
    const std::thread::id Thread;
};
struct FSessionLimits
{
    std::chrono::milliseconds Handshake{5000},Lifetime{300000};
    // All waits occur on the socket thread, never on a physical-frame caller.
};
// One finite connected exchange per instance. A future multi-peer host owns
// its acceptor/roster separately. Expected offer values are configured locally;
// this class does not establish an authenticated identity or approve a game.
class IAMSPEED_AGREEMENT_API FTcpAgreementSession final
{
public:
    static std::unique_ptr<FTcpAgreementSession> Listen(FIPv4Endpoint Bind,const FOffer& Expected,FSessionLimits Limits={});
    static std::unique_ptr<FTcpAgreementSession> Connect(FIPv4Endpoint Remote,const FOffer& Expected,FSessionLimits Limits={});
    ~FTcpAgreementSession();
    FTcpAgreementSession(const FTcpAgreementSession&)=delete;
    FTcpAgreementSession& operator=(const FTcpAgreementSession&)=delete;
    bool Start(); // original control thread; no socket is opened on this caller
    void RequestStop() noexcept;
    bool TryJoin(); // original control thread; nonblocking unless thread already finished
    bool HasStopped() const noexcept;
    ESessionPhase Phase() const noexcept;
    ESessionEnd EndReason() const noexcept;
    std::optional<FIPv4Endpoint> ReadBoundEndpoint() const noexcept;
    std::shared_ptr<const FConnectionObservedAgreement> ReadLatestObserved() const noexcept;
private:
    friend class FHostAgreementHost;
    static std::unique_ptr<FTcpAgreementSession> ListenEnrolled(FIPv4Endpoint,const FOffer&,FSessionLimits,std::shared_ptr<const FEnrollmentBinding>);
    static std::unique_ptr<FTcpAgreementSession> ConnectEnrolled(FIPv4Endpoint,const FOffer&,FSessionLimits,std::shared_ptr<const FEnrollmentBinding>);
    struct FImpl;
    explicit FTcpAgreementSession(std::unique_ptr<FImpl> P);
    void Run() noexcept;
    std::unique_ptr<FImpl> Impl;
};
}
