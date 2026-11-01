#pragma once
#include <array>
#include <cstdint>
#include <memory>
#include <vector>

namespace Speed::Netcode
{
// OS datagrams only. No UE socket/net-driver/RPC/replication API. Endpoint
// provenance is NOT session authentication; the IAS session must authenticate it.
constexpr std::size_t NativeDatagramBudget=1232;
struct FNativeEndpoint
{
    std::array<std::uint8_t,4> Address{127,0,0,1};std::uint16_t Port=0;
    bool operator==(const FNativeEndpoint&) const noexcept=default;
};
enum class ENativeDatagram { Ready,WouldBlock,Closed,TooLarge,Invalid,Unsupported,Error,PeerUnavailable };
class IAMSPEED_API FNativeDatagramSocket final
{
    struct FImpl;std::unique_ptr<FImpl> Impl;
public:
    FNativeDatagramSocket();~FNativeDatagramSocket();
    FNativeDatagramSocket(const FNativeDatagramSocket&)=delete;
    FNativeDatagramSocket& operator=(const FNativeDatagramSocket&)=delete;
    // Bind owns an actual exclusive OS endpoint; port zero asks the OS for one.
    ENativeDatagram Bind(const FNativeEndpoint& Requested);
    FNativeEndpoint LocalEndpoint() const;
    ENativeDatagram Send(const FNativeEndpoint& Destination,const std::vector<std::uint8_t>& Bytes);
    // On Windows a previous UDP send to an unbound port can surface as an
    // ICMP Port Unreachable notification here. PeerUnavailable contains no
    // message and does not prove that the original peer process has exited.
    ENativeDatagram Receive(FNativeEndpoint& ActualSource,std::vector<std::uint8_t>& Bytes);
    void Close() noexcept;
};
}
