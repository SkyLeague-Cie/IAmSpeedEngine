#pragma once
#include "AgreementWire.h"
#include <atomic>
#include <memory>

namespace Speed::Netcode::Agreement
{
class FHostAgreementHost;
class FTcpAgreementSession;
class FHostIdentity final
{
public:
    std::uint64_t Match()const noexcept{return MatchId;}
    std::uint64_t MatchEpoch()const noexcept{return Epoch;}
    std::uint64_t Generation()const noexcept{return LocalGeneration;}
    std::uint64_t ReadCloseFence()const noexcept{return Close.load(std::memory_order_acquire);}
private:
    friend class FHostAgreementHost;
    FHostIdentity(std::uint64_t M,std::uint64_t E,std::uint64_t G):MatchId(M),Epoch(E),LocalGeneration(G){}
    const std::uint64_t MatchId,Epoch,LocalGeneration;
    std::atomic<std::uint64_t> Close{0};
};
class FHostSlotControl final
{
public:
    std::uint64_t SlotId()const noexcept{return Id;}
    std::uint64_t ReadCloseFence()const noexcept{return Close.load(std::memory_order_acquire);}
    std::uint64_t ReadClosingRequest()const noexcept{return ClosingRequest.load(std::memory_order_acquire);}
private:
    friend class FHostAgreementHost;
    explicit FHostSlotControl(std::uint64_t I):Id(I){}
    const std::uint64_t Id;
    std::atomic<std::uint64_t> Close{0},ClosingRequest{0};
};
// Issued by the actual host owner. The key is never a wire profile, an
// original input, an account identity or a native physical permission.
class FEnrollmentBinding final
{
private:
    friend class FHostAgreementHost;
    friend class FTcpAgreementSession;
    friend class FJoinPermit;
    FEnrollmentBinding(std::shared_ptr<FHostIdentity> H,std::shared_ptr<FHostSlotControl> S,
        std::array<std::uint8_t,32> K,FBytes B):Host(std::move(H)),Slot(std::move(S)),Key(K),OfferBytes(std::move(B)){}
    const std::shared_ptr<FHostIdentity> Host;
    const std::shared_ptr<FHostSlotControl> Slot;
    const std::array<std::uint8_t,32> Key;
    const FBytes OfferBytes;
};
}
