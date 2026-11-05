#pragma once
#include "NativeDatagramSocket.h"
#include <optional>
#include <string>

namespace Speed::Netcode
{
struct FNativeProcessDescriptor
{
    std::uint32_t Pid=0;std::uint64_t BirthTicks=0;std::wstring ImagePath;
};
// LOCAL transport evidence only. Retains an original OS process HANDLE and
// checks the OS UDP owner table at observation time. Not a socket-generation
// lease, remote-host authentication, image-content hash, game-source admission,
// native physical lifetime certificate, or permission to release any job.
// A real session must additionally bind its nonce/epoch and socket lifetime.
class IAMSPEED_API FNativeLocalPeerPin final
{
    struct FImpl;std::unique_ptr<FImpl> Impl;
    FNativeLocalPeerPin();
public:
    ~FNativeLocalPeerPin();
    FNativeLocalPeerPin(const FNativeLocalPeerPin&)=delete;
    FNativeLocalPeerPin& operator=(const FNativeLocalPeerPin&)=delete;
    static std::optional<FNativeProcessDescriptor> InspectCurrent();
    static std::unique_ptr<FNativeLocalPeerPin> PinOriginal(const FNativeProcessDescriptor& TrustedExpected);
    bool OriginalProcessAlive() const noexcept;
    bool MatchesObservedLoopbackEndpoint(const FNativeEndpoint& ActualSource) const;
};
}
