#pragma once
#include "RequestedNetworkSessionDescriptor.h"
#include "IAmSpeed/Netcode/transport/RequestedSourceIdentity.h"
#include "IAmSpeed/Netcode/agreement/HostAgreementHost.h"
#include <chrono>

class ASpeedPhysicsSimulation;
namespace Speed {class FNativeRequestedPreparationContextV2;}
namespace Speed::Input::V2
{
class FInputSessionRegistry;
// Qualification barrier. No public setter, command value, ComparedOnly receipt
// or game Supports result can enable the unqualified native runtime route.
inline bool NativeRequestedV2RuntimeQualified()noexcept{return false;}
enum class ERequestedFrameControlV2:std::uint8_t { Ready,NeedsOwnerBoundary,Terminal };
enum class ERequestedRetirementReasonV2:std::uint8_t { None,HostClosed,SlotClosed,SocketClosed,Expired,SourceClosed,MappingChanged,ExplicitDetach,WorkerClose,ConstructionFailed,ClaimFailed,GlobalPause,GlobalRetire,InvalidSelection };
enum class ERequestedConstructionV2:std::uint8_t { NotConstructed,Constructed,ConstructedThenRetired };
struct FRequestedControlTargetV2
{
    std::uint64_t Session=0,Epoch=0,Service=0,SourceGeneration=0,SourceInstance=0;
    bool IsValid()const noexcept{return Session&&Epoch&&Service&&SourceGeneration&&SourceInstance;}
    bool Matches(std::uint64_t S,std::uint64_t E,std::uint64_t V,std::uint64_t G,std::uint64_t I)const noexcept
    {return IsValid()&&Session==S&&Epoch==E&&Service==V&&SourceGeneration==G&&SourceInstance==I;}
};
struct FRequestedLifecycleReceiptV2
{
    ERequestedConstructionV2 Construction=ERequestedConstructionV2::NotConstructed;
    ERequestedRetirementReasonV2 Retirement=ERequestedRetirementReasonV2::None;
    std::uint64_t Service=0,SourceGeneration=0,SourceInstance=0;
    std::uint64_t ControlOrdinal=0,ControlFrame=0,CompletedFrames=0;
    Netcode::Agreement::FHostClosureObservation Closures;
    bool WorkerSelectionBound=false,SourceDestroyed=false;
    Netcode::ERequestedSourceBindFailure BindFailure=Netcode::ERequestedSourceBindFailure::None;
};
// The seal is issued only by the actual native driver AFTER a fresh complete
// context/game check. It is not made from a public preparation receipt.
class FNativeRequestedAgreementSealV2 final
{
public:
    const std::shared_ptr<const Speed::FNativeRequestedPreparationContextV2>& Context()const noexcept{return Prepared;}
    const std::shared_ptr<const Netcode::Agreement::FHostAgreementRecord>& Agreement()const noexcept{return Record;}
    const std::shared_ptr<const void>& ServiceIdentity()const noexcept{return Service;}
    const FRequestedNetworkSessionDescriptor& Mapping()const noexcept{return NativeMapping;}
    Netcode::Agreement::FHostClosureObservation ReadClosure()const noexcept{return Record->ReadClosure();}
    bool Expired(std::chrono::steady_clock::time_point Now)const noexcept{return Now>=Deadline;}
    std::thread::id Owner()const noexcept{return Thread;}
private:
    friend class ::ASpeedPhysicsSimulation;
    FNativeRequestedAgreementSealV2(std::shared_ptr<const Speed::FNativeRequestedPreparationContextV2> C,
        std::shared_ptr<const Netcode::Agreement::FHostAgreementRecord> R,std::shared_ptr<const void> S,
        FRequestedNetworkSessionDescriptor N)
        :Prepared(std::move(C)),Record(std::move(R)),Service(std::move(S)),NativeMapping(std::move(N)),
        Deadline(std::chrono::steady_clock::now()+std::chrono::milliseconds(NativeMapping.Agreement->LeaseMilliseconds)),Thread(std::this_thread::get_id()){}
    const std::shared_ptr<const Speed::FNativeRequestedPreparationContextV2> Prepared;
    const std::shared_ptr<const Netcode::Agreement::FHostAgreementRecord> Record;
    const std::shared_ptr<const void> Service;
    const FRequestedNetworkSessionDescriptor NativeMapping;
    const std::chrono::steady_clock::time_point Deadline;
    const std::thread::id Thread;
};
// Constructed by the registry only AFTER the actual data source exists. The
// source witness closes on destruction, preventing source-address reuse.
class FNativeRequestedBindingV2 final
{
public:
    const std::shared_ptr<const FNativeRequestedAgreementSealV2>& Seal()const noexcept{return AgreementSeal;}
    const std::shared_ptr<const Netcode::FRequestedSourceIdentity>& Source()const noexcept{return BoundSource;}
    const FRequestedNetworkSessionDescriptor& Mapping()const noexcept{return AgreementSeal->Mapping();}
    std::uint64_t Session()const noexcept{return LocalSession;}
    std::uint64_t Epoch()const noexcept{return LocalEpoch;}
    bool OnOwner()const noexcept{return BoundSource->Owner()==std::this_thread::get_id();}
private:
    friend class FInputSessionRegistry;
    FNativeRequestedBindingV2(std::shared_ptr<const FNativeRequestedAgreementSealV2> A,
        std::shared_ptr<const Netcode::FRequestedSourceIdentity> S,std::uint64_t Id,std::uint64_t E)
        :AgreementSeal(std::move(A)),BoundSource(std::move(S)),LocalSession(Id),LocalEpoch(E){}
    const std::shared_ptr<const FNativeRequestedAgreementSealV2> AgreementSeal;
    const std::shared_ptr<const Netcode::FRequestedSourceIdentity> BoundSource;
    const std::uint64_t LocalSession,LocalEpoch;
};
struct FRequestedFrameBindingV2
{
    std::shared_ptr<const FNativeRequestedBindingV2> Binding;
    Netcode::Agreement::FHostClosureObservation Closures;
};
// One immutable control cutoff for N. Consumers must compare the exact token
// with the registry's current frozen token; retained history is not authority.
class FNativeRequestedFrameAdmissionV2 final
{
public:
    std::uint64_t Frame()const noexcept{return N;}
    std::uint64_t Ordinal()const noexcept{return Serial;}
    bool OnOwner()const noexcept{return Thread==std::this_thread::get_id();}
    const std::vector<FRequestedFrameBindingV2>& Bindings()const noexcept{return Captured;}
    std::shared_ptr<const FNativeRequestedBindingV2> Find(std::uint64_t Id,std::uint64_t E)const noexcept
    {for(const auto& R:Captured)if(R.Binding->Session()==Id&&R.Binding->Epoch()==E)return R.Binding;return {};}
private:
    friend class FInputSessionRegistry;
    FNativeRequestedFrameAdmissionV2(std::uint64_t Frame,std::uint64_t O,std::vector<FRequestedFrameBindingV2> B)
        :N(Frame),Serial(O),Thread(std::this_thread::get_id()),Captured(std::move(B)){}
    const std::uint64_t N,Serial;
    const std::thread::id Thread;
    const std::vector<FRequestedFrameBindingV2> Captured;
};
}
