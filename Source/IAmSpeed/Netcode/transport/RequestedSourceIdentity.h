#pragma once
#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>

namespace Speed::Netcode
{
class FNetworkRequestedInputService;
enum class ERequestedSourceBindingPhase:std::uint8_t { Fresh,Binding,Bound,Closed };
enum class ERequestedSourceBindFailure:std::uint8_t { None,InvalidGeneration,AcquisitionClosed,MissingBaseline,AlreadyBound,IdentifierExhausted,WorkerUnavailable,InitialDataInvalid,AllocationFailure };
// Evidence about an actual DATA source object, never native/world permission.
// A decoder/application cannot synthesize its service key, address or owner.
class FRequestedSourceIdentity final
{
public:
    std::uint64_t Generation()const noexcept{return SourceGeneration;}
    std::uint64_t Instance()const noexcept{return SourceInstance;}
    std::thread::id Owner()const noexcept{return Thread;}
    bool MatchesSource(const void* P)const noexcept{return P&&P==SourceAddress;}
    bool MatchesService(const std::shared_ptr<const void>& K)const noexcept{return K&&K==ServiceKey;}
    bool IsOpen()const noexcept{return Phase->load(std::memory_order_acquire)==ERequestedSourceBindingPhase::Bound;}
    bool IsClosed()const noexcept{return Phase->load(std::memory_order_acquire)==ERequestedSourceBindingPhase::Closed;}
private:
    friend class FNetworkRequestedInputService;
    FRequestedSourceIdentity(std::shared_ptr<const void> K,const void* S,std::uint64_t G,std::uint64_t I,
        std::shared_ptr<std::atomic<ERequestedSourceBindingPhase>> P)
        :ServiceKey(std::move(K)),SourceAddress(S),SourceGeneration(G),SourceInstance(I),Thread(std::this_thread::get_id()),Phase(std::move(P)){}
    const std::shared_ptr<const void> ServiceKey;
    const void* const SourceAddress;
    const std::uint64_t SourceGeneration,SourceInstance;
    const std::thread::id Thread;
    const std::shared_ptr<std::atomic<ERequestedSourceBindingPhase>> Phase;
};
}
