#pragma once
#include "RegistryBoundaryMetadata.h"
#include "IAmSpeed/Input/InputSessionRegistry.h"
#include <memory>

namespace Speed::Netcode::BoundaryMetadata
{
namespace V=Speed::Input::V2;
class FPreparedRegistryMetadata;
std::shared_ptr<const FPreparedRegistryMetadata> PrepareRegistryMetadata(V::FInputSessionRegistry&,const V::FInputSessionRegistry::FCompletedBoundaryReservation&);
class FPreparedRegistryMetadata final
{
public:
    const FMetadata& Metadata()const noexcept{return Facts;}
    const std::vector<std::uint8_t>& CanonicalBytes()const noexcept{return Bytes;}
    const std::shared_ptr<const V::FInputSessionRegistry::FCompletedBoundaryCapture>& OriginCapture()const noexcept{return Capture;}
    FPreparedRegistryMetadata(const FPreparedRegistryMetadata&)=delete;
private:
    friend std::shared_ptr<const FPreparedRegistryMetadata> PrepareRegistryMetadata(V::FInputSessionRegistry&,const V::FInputSessionRegistry::FCompletedBoundaryReservation&);
    FPreparedRegistryMetadata(std::shared_ptr<const V::FInputSessionRegistry::FCompletedBoundaryCapture> C,FMetadata M,std::vector<std::uint8_t> B)noexcept
        :Capture(std::move(C)),Facts(std::move(M)),Bytes(std::move(B)){}
    const std::shared_ptr<const V::FInputSessionRegistry::FCompletedBoundaryCapture> Capture;
    const FMetadata Facts;
    const std::vector<std::uint8_t> Bytes;
};
inline std::shared_ptr<const FPreparedRegistryMetadata> PrepareRegistryMetadata(V::FInputSessionRegistry& Registry,
    const V::FInputSessionRegistry::FCompletedBoundaryReservation& Token)
{
    const auto Capture=Registry.ReadReservedCompletedBoundary(Token);if(!Capture)return {};
    try
    {
        const auto Frame=Capture->Completed();const auto Current=Capture->CurrentBoundary();
        if(!Frame || !Frame->Registry || !Current || Frame->Registry->Terminal || Current->Terminal
            || Current->WorkerGeneration!=Frame->Registry->WorkerGeneration
            || Frame->Inputs.size()!=Frame->Registry->Bindings.size() || Current->Phases.size()!=Current->Bindings.size()
            || Frame->Inputs.size()>MaxSessions || Current->Bindings.size()>MaxSessions)return {};
        FMetadata M;M.CompletedInputFrame=Frame->Frame;M.WorkerGeneration=Current->WorkerGeneration;
        M.ConsumedRegistryVersion=Frame->Registry->Version;M.CurrentRegistryVersion=Current->Version;
        const auto Convert=[&](const V::FSessionDescriptor& D,EPhase Phase)->std::optional<FBinding>
        {
            if(!D.Contract || D.Contract->GetFingerprint().empty() || D.Contract->GetFingerprint().size()>MaxFingerprint
                || D.Actors.empty() || D.Actors.size()>MaxActorsPerView)return {};
            FBinding B;B.Session=D.Id;B.Epoch=D.Epoch;B.Controller=D.Controller;B.Producer=D.Producer;B.Journal=D.Journal;B.First=D.First;
            switch(D.Kind)
            {
                case V::EProducerContract::Device:B.Origin=EOrigin::Device;break;
                case V::EProducerContract::ExactScenario:B.Origin=EOrigin::ExactScenario;break;
                case V::EProducerContract::AI:B.Origin=EOrigin::AI;break;
                default:return {}; // No implicit ExactRemote admission/upgrade.
            }
            B.Phase=Phase;B.AllowScenarioAppend=D.AllowScenarioAppend;B.ResumeRearmMask=D.ResumeRearmMask;
            B.Step=D.Processing.Step;B.ContractFingerprint=D.Contract->GetFingerprint();
            for(const auto& A:D.Actors)B.Actors.push_back({A.Id,A.Generation});
            return B;
        };
        for(std::size_t I=0;I<Frame->Registry->Bindings.size();++I)
        {
            const auto& D=Frame->Registry->Bindings[I];const auto& Closed=Frame->Inputs[I];
            auto B=Convert(D,EPhase::NotCaptured);if(!B || Closed.Session!=D.Id || Closed.Epoch!=D.Epoch || Closed.Actors!=D.Actors || !Closed.Snapshot)return {};
            const auto& S=*Closed.Snapshot;const auto& Input=S.Input.GetData();
            if(!S.Input.IsValidFor(*D.Contract) || Input.ConsumptionFrame!=Frame->Frame || Input.StreamEpoch.Value!=D.Epoch
                || Input.Producer.Id!=D.Producer || Input.Producer.Kind!=(D.Kind==V::EProducerContract::AI?Speed::Input::EProducerKind::AI:Speed::Input::EProducerKind::Device)
                || S.Requested!=Input.Values)return {};
            M.Closed.push_back({D.Id,D.Epoch,Input.SourceSequence,S.CutoffSequence,S.BindingFingerprint});M.Consumed.push_back(std::move(*B));
        }
        for(std::size_t I=0;I<Current->Bindings.size();++I)
        {
            const auto& D=Current->Bindings[I];const auto& P=Current->Phases[I];
            if(P.Session!=D.Id || P.Epoch!=D.Epoch || P.Phase>V::ESessionPhase::Retired)return {};
            auto B=Convert(D,static_cast<EPhase>(P.Phase));if(!B)return {};M.Current.push_back(std::move(*B));
        }
        auto Bytes=Encode(M);if(!Bytes)return {};
        // No virtual providers are invoked. Still refuse a custom allocator's
        // reentrant abort/replacement rather than returning an unreserved result.
        auto Prepared=std::shared_ptr<const FPreparedRegistryMetadata>(new FPreparedRegistryMetadata(Capture,std::move(M),std::move(*Bytes)));
        if(Registry.ReadReservedCompletedBoundary(Token)!=Capture)return {};
        return Prepared;
    }
    catch(...){return {};}
}
// This INPUT metadata block omits scenario authoring, owner mutable history,
// full requested/applied command payloads and native actor/world state. A new
// full-state protocol tuple must join those and authenticate their common
// native publication. This producer is neither such a tuple nor a restore.
}
