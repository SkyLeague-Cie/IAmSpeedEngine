#pragma once
#include "NativeLocalPeerPin.h"

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
};
enum class ELocalSession {Ready,Pending,Duplicate,Stale,WouldBlock,Closed,WrongOwner,Invalid,UnknownPeer,PeerDead,Full,SocketError,PeerUnavailable};
struct FLocalSessionMessage
{std::uint64_t Peer=0,Sequence=0;std::uint8_t Kind=0;std::vector<std::uint8_t> Bytes;};
// Controlled SAME-HOST launcher session. Config MUST come from the trusted
// launcher with actual child birth/image descriptors and fresh socket nonces.
// Owns its socket privately, binds once, and cannot reconnect. The tuple cannot
// be reused during this module lifetime; the host must pin that module lifetime.
// CRC is NOT cryptographic authentication. This is not remote-host security,
// game-input source admission or a native physical/lifetime restore certificate.
class IAMSPEED_API FNativeLocalSession final
{
    struct FImpl;std::unique_ptr<FImpl> Impl;
    FNativeLocalSession();
public:
    ~FNativeLocalSession();
    FNativeLocalSession(const FNativeLocalSession&)=delete;
    FNativeLocalSession& operator=(const FNativeLocalSession&)=delete;
    static std::unique_ptr<FNativeLocalSession> Open(const FLocalSessionConfig& LauncherAssigned);
    ELocalSession Send(std::uint64_t Peer,std::uint8_t Kind,const std::vector<std::uint8_t>& OriginalBytes);
    ELocalSession Receive(FLocalSessionMessage& Message);
    bool OriginalPeersAlive() const;
    FNativeEndpoint LocalEndpoint() const;
    void Close() noexcept;
};
}
