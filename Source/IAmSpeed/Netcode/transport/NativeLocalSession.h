#pragma once
#include "NativeLocalPeerPin.h"
#include <memory>
#include <utility>

namespace Speed::Netcode
{
struct FLocalSessionPeer
{
    std::uint64_t Peer=0,SocketNonce=0;
    FNativeProcessDescriptor OriginalProcess;
    FNativeEndpoint Endpoint;
};
struct FLocalSessionConfig
{
    std::uint64_t Session=0,Epoch=0,SelfPeer=0,SocketNonce=0;
    FNativeProcessDescriptor OriginalSelf;
    FNativeEndpoint Endpoint;
    std::vector<FLocalSessionPeer> Peers;
    // Explicit same-host diagnostic fault profile, disabled by default. A
    // selected original IAS UDP fragment is discarded BEFORE the OS send.
    // This is controlled egress loss, never measured external network loss.
    std::uint32_t ControlledLossEvery=0;
    std::uint16_t ControlledLossKinds=0; // bit Kind-1; no input/source authority
};
enum class ELocalSession {Ready,Pending,Duplicate,Stale,WouldBlock,Closed,WrongOwner,Invalid,UnknownPeer,PeerDead,Full,SocketError,PeerUnavailable,InjectedLoss,ConsumerClaimed};
struct FLocalEgressKindCounters
{
    std::uint64_t Attempted=0,DiscardedBeforeOsSend=0,OsSent=0,OsWouldBlock=0;
};
struct FLocalSessionEgressCounters {std::array<FLocalEgressKindCounters,16> Kinds{};};
struct FLocalSessionMessage
{std::uint64_t Peer=0,Sequence=0;std::uint8_t Kind=0;std::vector<std::uint8_t> Bytes;};
// Controlled SAME-HOST launcher session. Config MUST come from the trusted
// launcher with actual child birth/image descriptors and fresh socket nonces.
// Owns its socket privately, binds once, and cannot reconnect. The tuple cannot
// be reused during this module lifetime; the host must pin that module lifetime.
// CRC is NOT cryptographic authentication. This is not remote-host security,
// game-input source admission or a native physical/lifetime restore certificate.
class FNativeLocalIngress;
class FNativeLocalReceivedMessage;
class IAMSPEED_API FNativeLocalSession final
{
    friend class FNativeLocalIngress;
    friend class FNativeLocalReceivedMessage;
    struct FImpl;std::shared_ptr<FImpl> Impl;
    // Called only with the original Gate held and the actual Owner checked.
    static ELocalSession ReceiveOriginal(FImpl& Original,FLocalSessionMessage& Message);
    FNativeLocalSession();
public:
    ~FNativeLocalSession();
    FNativeLocalSession(const FNativeLocalSession&)=delete;
    FNativeLocalSession& operator=(const FNativeLocalSession&)=delete;
    static std::unique_ptr<FNativeLocalSession> Open(const FLocalSessionConfig& LauncherAssigned);
    ELocalSession Send(std::uint64_t Peer,std::uint8_t Kind,const std::vector<std::uint8_t>& OriginalBytes);
    ELocalSession Receive(FLocalSessionMessage& Message);
    // Irreversible exclusive claim of this actual channel's receive path.
    // Failure returns no object. Close/peer death can never be undone.
    std::unique_ptr<FNativeLocalIngress> AcquireIngress(ELocalSession& Status);
    FLocalSessionEgressCounters ReadEgressCounters() const; // immutable actual counters, not an admission
    bool OriginalPeersAlive() const;
    FNativeEndpoint LocalEndpoint() const;
    void Close() noexcept;
};

// Immutable retained bytes from an actual ReceiveOriginal operation. This
// object cannot be authored from a peer number, payload, sequence or CRC.
// Retention after Close is data retention only, never permission to receive.
class IAMSPEED_API FNativeLocalReceivedMessage final
{
    friend class FNativeLocalIngress;
    std::shared_ptr<FNativeLocalSession::FImpl> Source;
    FLocalSessionMessage Original;
    explicit FNativeLocalReceivedMessage(std::shared_ptr<FNativeLocalSession::FImpl> In):Source(std::move(In)){}
public:
    ~FNativeLocalReceivedMessage();
    FNativeLocalReceivedMessage(const FNativeLocalReceivedMessage&)=delete;
    FNativeLocalReceivedMessage& operator=(const FNativeLocalReceivedMessage&)=delete;
    const FLocalSessionMessage& Read() const noexcept {return Original;}
};
// Created only by the real open session; no public constructor or rebinding.
// Uses the original owner thread. Close may be requested from another thread.
// The original mutex linearizes Receive and Close, but Close is NOT a join of
// arbitrary callers' return stacks. Keep the IAS module loaded through every
// operation and through destruction of exports/receipts; no global lease here.
class IAMSPEED_API FNativeLocalIngress final
{
    friend class FNativeLocalSession;
    std::shared_ptr<FNativeLocalSession::FImpl> Original;
    explicit FNativeLocalIngress(std::shared_ptr<FNativeLocalSession::FImpl> In):Original(std::move(In)){}
public:
    ~FNativeLocalIngress();
    FNativeLocalIngress(const FNativeLocalIngress&)=delete;
    FNativeLocalIngress& operator=(const FNativeLocalIngress&)=delete;
    ELocalSession Receive(std::shared_ptr<const FNativeLocalReceivedMessage>& Message);
    void Close() noexcept;
};
}
