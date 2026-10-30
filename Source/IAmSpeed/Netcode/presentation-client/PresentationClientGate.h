#pragma once
#include "IAmSpeed/Netcode/contract-offer/ActionContractOfferCodec.h"
#include <cstdint>
#include <cmath>
namespace Speed::Netcode::PresentationClient
{
enum class EState : std::uint8_t { Unselected, Pending, Admitted, Failed };
struct FTuple
{
    std::uint64_t Session=0,Epoch=0,Controller=0,Producer=0,Actor=0,Incarnation=0;
    std::vector<std::uint8_t> CanonicalOffer;
    bool Valid() const noexcept
    {
        return Session&&Epoch&&Controller&&Producer&&Actor&&Incarnation
            && ContractOffer::Decode(CanonicalOffer.data(),CanonicalOffer.size()).has_value();
    }
    bool operator==(const FTuple& B) const noexcept
    { return Session==B.Session&&Epoch==B.Epoch&&Controller==B.Controller&&Producer==B.Producer
        &&Actor==B.Actor&&Incarnation==B.Incarnation&&CanonicalOffer==B.CanonicalOffer; }
};
// Ordinary state machine: host authentication is external. Trusted must be the
// host-approved entire tuple, not a fingerprint or a decoded untrusted offer.
class FGate
{
public:
    FGate(bool Enabled,std::uint64_t Generation,double Deadline)
        : State(Enabled?EState::Pending:EState::Unselected),WorldGeneration(Generation),Expires(Deadline)
    { if(Enabled&&(!Generation||!std::isfinite(Deadline)))Fail(); }
    EState GetState() const noexcept { return State; }
    bool SuppressesPhysicalBootstrap() const noexcept { return State!=EState::Unselected; }
    void Fail() noexcept { if(State!=EState::Unselected)State=EState::Failed; }
    EState Observe(double Now,bool IsClient,bool IsServer,std::uintptr_t Driver,
        std::uintptr_t Connection,bool Conflict) noexcept
    {
        if(State==EState::Unselected||State==EState::Failed)return State;
        if(!std::isfinite(Now)||Conflict||IsServer||(State==EState::Pending&&Now>=Expires)
            ||(OriginalDriver&&OriginalDriver!=Driver)||(OriginalConnection&&OriginalConnection!=Connection)
            ||(State==EState::Admitted&&(!IsClient||!Driver||!Connection))) { Fail();return State; }
        if(Driver)OriginalDriver=Driver;
        if(Connection)OriginalConnection=Connection;
        return State;
    }
    bool Admit(double Now,std::uint64_t Generation,std::uintptr_t Driver,std::uintptr_t Connection,
        const FTuple& Trusted,const FTuple& Received,bool IsClient,bool Pristine)
    {
        Observe(Now,IsClient,false,Driver,Connection,false);
        if(State!=EState::Pending||Generation!=WorldGeneration||!IsClient||!Driver||!Connection
            ||!Pristine||!Trusted.Valid()||!Received.Valid()||!(Trusted==Received)) { Fail();return false; }
        Bound=Trusted;State=EState::Admitted;return true;
    }
    bool Matches(const FTuple& Value) const noexcept { return State==EState::Admitted&&Bound==Value; }
private:
    EState State;std::uint64_t WorldGeneration;double Expires;
    std::uintptr_t OriginalDriver=0,OriginalConnection=0;FTuple Bound;
};
}
