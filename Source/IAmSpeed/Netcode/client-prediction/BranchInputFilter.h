#pragma once
#include "RemoteRequestPrediction.h"
#include "IAmSpeed/Input/InputProcessingPolicy.h"
#include <thread>
#include <variant>

namespace Speed::Netcode::Prediction
{
namespace BranchV2=Speed::Input::V2;
enum class EBranchFilterRole : uint8_t { LocalDevice, RemoteRequests };
enum class EBranchRequestOrigin : uint8_t { RetainedLocalDevice, ReceivedAuthorityRequest, SpeculativeHoldRequest };
// Speculation has NO raw Device identity, generation, sequence, reset or edges.
// The actual immutable local frame / received packet is retained when present;
// no FInputFrame, registry receipt or outgoing ACK is manufactured for forecasts.
using FBranchRequestEvidence=std::variant<std::shared_ptr<const BranchV2::FInputFrame>,
    std::shared_ptr<const FPacket>,FSpeculativeRequest>;
struct FBranchProcessedInput
{
    U64 Frame=0;
    EBranchRequestOrigin Origin=EBranchRequestOrigin::SpeculativeHoldRequest;
    Speed::Input::FActionValues Requested{},Before{},Applied{};
    uint32_t ActiveMask=0;
    FBranchRequestEvidence Evidence;
};
struct FBranchFilterState
{
    U64 Next=0;
    Speed::Input::FActionValues Applied{};
    uint32_t ActiveMask=0;
    bool operator==(const FBranchFilterState& R) const noexcept
    {return Next==R.Next&&Applied==R.Applied&&ActiveMask==R.ActiveMask;}
};
// Replayable processing-state participant only. Not a registry/input producer,
// native world lease, authority authenticator or completed-world publisher.
// The native parent must compose this state with the physical world and bind
// semantic frame/actor/branch identities before allowing physical consumption.
class FBranchInputFilter final
{
    const EBranchFilterRole Role;
    const std::shared_ptr<const BranchV2::FInputActionContract> Contract;
    const BranchV2::FInputProcessingPolicy Policy;
    const Speed::Input::FProducerIdentity Local;
    const BranchV2::FStreamEpoch Epoch;
    const FBinding Remote;
    const U64 Horizon;
    const std::thread::id Owner=std::this_thread::get_id();
    const std::shared_ptr<const uint8_t> Lifetime=std::make_shared<const uint8_t>(uint8_t(0));
    FBranchFilterState State;
    U64 Revision=1;
    FBranchInputFilter(EBranchFilterRole RRole,std::shared_ptr<const BranchV2::FInputActionContract> C,BranchV2::FInputProcessingPolicy P,
        Speed::Input::FProducerIdentity L,BranchV2::FStreamEpoch E,FBinding R,U64 H,FBranchFilterState S)
        :Role(RRole),Contract(std::move(C)),Policy(P),Local(L),Epoch(E),Remote(R),Horizon(H),State(S){}
    bool IsOwner() const noexcept
    {return std::this_thread::get_id()==Owner&&!Speed::Input::FPresentationInputScope::IsActive();}
    bool ValidValues(const Speed::Input::FActionValues& Values,uint32_t Active) const noexcept
    {
        for(size_t I=0;I<ActionCount;++I)
        {
            const auto* A=Contract->Find(static_cast<Speed::Input::FActionId>(I));
            if(!A||A->Wiring==BranchV2::EActionWiring::Unwired)
            {if(Values[I]||(Active&(uint32_t(1)<<I)))return false;}
            else if(!A->Accepts(Values[I]))return false;
        }
        return true;
    }
public:
    class FPrepared
    {
        friend class FBranchInputFilter;
        std::shared_ptr<const uint8_t> Identity;
        U64 Revision=0;
        FBranchFilterState Before,After;
        std::shared_ptr<const FBranchProcessedInput> Processed;
    public:
        std::shared_ptr<const FBranchProcessedInput> ReadProcessed() const noexcept {return Processed;}
    };
    static std::unique_ptr<FBranchInputFilter> Create(EBranchFilterRole Role,std::shared_ptr<const BranchV2::FInputActionContract> C,
        BranchV2::FInputProcessingPolicy P,Speed::Input::FProducerIdentity Local,BranchV2::FStreamEpoch Epoch,
        FBinding Remote,U64 MaximumPredictionFrames,FBranchFilterState Baseline)
    {
        if(Speed::Input::FPresentationInputScope::IsActive()||!C||!Local.Id||Local.Kind!=Speed::Input::EProducerKind::Device
            ||!Epoch.Value||!Remote.Valid()||Remote.SourceKind!=uint8_t(Speed::Input::EProducerKind::Device)
            ||Role>EBranchFilterRole::RemoteRequests||!MaximumPredictionFrames||MaximumPredictionFrames>4096||Baseline.Next==UINT64_MAX)return {};
        // Validate the shared quantized processing recipe directly. This does
        // NOT label a speculative request as a Device producer or admit one.
        for(size_t I=0;I<ActionCount;++I)if(P.Step[I])
        {
            const auto* A=C->Find(static_cast<Speed::Input::FActionId>(I));
            if(!A||A->Wiring!=BranchV2::EActionWiring::Wired||A->Type!=BranchV2::EActionType::Axis1D
                ||P.Step[I]>static_cast<uint16_t>(2*A->Quantization))return {};
        }
        auto Result=std::unique_ptr<FBranchInputFilter>(new FBranchInputFilter(Role,std::move(C),P,Local,Epoch,Remote,MaximumPredictionFrames,Baseline));
        if(!Result->ValidValues(Baseline.Applied,Baseline.ActiveMask))return {};return Result;
    }
    FBranchInputFilter(const FBranchInputFilter&)=delete;
    FBranchInputFilter& operator=(const FBranchInputFilter&)=delete;
    std::optional<FBranchFilterState> Capture() const noexcept
    {return IsOwner()?std::optional<FBranchFilterState>(State):std::nullopt;}
private:
    std::optional<FPrepared> Prepare(U64 Frame,const Speed::Input::FActionValues& Requested,uint32_t Active,
        bool ActualReset,EBranchRequestOrigin Origin,FBranchRequestEvidence Evidence) const
    {
        if(!IsOwner()||Frame!=State.Next||Frame==UINT64_MAX||Revision==UINT64_MAX||!ValidValues(Requested,Active))return {};
        // Same pure function as the real authoritative V2 input owner. The
        // previous APPLIED value is state, the held REQUEST is the target.
        const auto Applied=BranchV2::ProcessAuthoritativeInputValues(Policy,Requested,State.Applied,ActualReset);
        if(!ValidValues(Applied,0))return {};
        auto Values=std::make_shared<const FBranchProcessedInput>(FBranchProcessedInput{Frame,Origin,Requested,State.Applied,Applied,Active,std::move(Evidence)});
        FPrepared P;P.Identity=Lifetime;P.Revision=Revision;P.Before=State;P.After={Frame+1,Applied,Active};P.Processed=std::move(Values);return P;
    }
public:
    std::optional<FPrepared> PrepareLocal(std::shared_ptr<const BranchV2::FInputFrame> Retained) const
    {
        if(!IsOwner()||Role!=EBranchFilterRole::LocalDevice||!Retained||!Retained->IsValidFor(*Contract))return {};
        const auto& D=Retained->GetData();
        if(D.Producer.Kind!=Local.Kind||D.Producer.Id!=Local.Id||D.StreamEpoch.Value!=Epoch.Value)return {};
        return Prepare(D.ConsumptionFrame,D.Values,D.ActiveMask,D.Reset,EBranchRequestOrigin::RetainedLocalDevice,std::move(Retained));
    }
    std::optional<FPrepared> PrepareRemote(const FPrediction& Forecast) const
    {
        if(!IsOwner()||Role!=EBranchFilterRole::RemoteRequests)return {};
        if(Forecast.Status==EPredict::Predicted)
        {
            const auto& R=Forecast.Request;
            if(R.EvidenceFrame>=R.Frame||R.Frame-R.EvidenceFrame>Horizon)return {};
            return Prepare(R.Frame,R.Requested,R.ActiveMask,false,EBranchRequestOrigin::SpeculativeHoldRequest,R);
        }
        if(Forecast.Status!=EPredict::Confirmed||!Forecast.ConfirmedPacket)return {};
        const auto& P=*Forecast.ConfirmedPacket;
        // Authentication is retained by the original ingress/history owner;
        // structural equality here cannot turn an untrusted packet into proof.
        if(!Valid(P)||P.Kind!=EKind::AuthorityInput||!Remote.Matches(P)
            ||P.Input.ProducerId!=Remote.SourceProducer||P.Input.ProducerKind!=Remote.SourceKind)return {};
        for(const auto& E:P.Input.Edges)
        {
            const auto* A=Contract->Find(E.Action);
            if(!A||A->Wiring!=BranchV2::EActionWiring::Wired||!A->Accepts(E.Value))return {};
        }
        return Prepare(P.Frame,P.Input.Values,P.Input.ActiveMask,P.Input.Reset,EBranchRequestOrigin::ReceivedAuthorityRequest,std::make_shared<const FPacket>(P));
    }
    bool CanCommit(const FPrepared& P) const noexcept
    {return IsOwner()&&P.Identity==Lifetime&&P.Revision==Revision&&P.Before==State;}
    bool Commit(const FPrepared& P) noexcept
    {if(!CanCommit(P))return false;State=P.After;++Revision;return true;}
    // Processing state rewind ONLY. Native world/branch and authentication are
    // the parent's transaction; no existing registry or native restore guard
    // is bypassed here. Caller retains acquisitions/transport high-waters.
    std::optional<FPrepared> PrepareRestore(const FBranchFilterState& Baseline) const
    {
        if(!IsOwner()||Revision==UINT64_MAX||Baseline.Next==UINT64_MAX||!ValidValues(Baseline.Applied,Baseline.ActiveMask))return {};
        FPrepared P;P.Identity=Lifetime;P.Revision=Revision;P.Before=State;P.After=Baseline;return P;
    }
};
}
