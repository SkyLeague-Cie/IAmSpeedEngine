#pragma once
#include "DesktopPublicationValidation.h"
#include <map>

namespace Speed::Input::V2
{
struct FDesktopPhysicalBoundary
{
    FStreamEpoch Epoch;
    FFrameNumber Frame=0;
    FProducerIdentity Producer;
    bool IsValid()const{return Epoch.Value&&Producer.Id&&Producer.Kind==EProducerKind::Device;}
};
enum class EDesktopTerminalClock:std::uint8_t{Ingress,PhysicalBoundary};
struct FDesktopActionTerminal
{
    FDesktopControlIdentity Origin;
    FActionId Action=0;
    std::uint64_t RawActivation=0,ActionActivation=0;
    std::optional<FInputEdgeIdentity> StartedEdge;
    EIngressCause Cause=EIngressCause::Disconnect;
    EDesktopTerminalClock Clock=EDesktopTerminalClock::Ingress;
    std::uint64_t IngressTerminalSequence=0;
    std::optional<FDesktopPhysicalBoundary> PhysicalBoundary;
    bool IsValid()const
    {
        if(!Origin.Source.DeviceId||!Origin.Source.Generation||!Origin.Control.IsValid()
            ||Action>=ActionCount||!RawActivation||Cause>EIngressCause::Resync
            ||(StartedEdge&&(!StartedEdge->IsValid()||StartedEdge->Action!=Action||!ActionActivation)))return false;
        return Clock==EDesktopTerminalClock::Ingress
            ?IngressTerminalSequence>RawActivation&&!PhysicalBoundary
            :Clock==EDesktopTerminalClock::PhysicalBoundary&&!IngressTerminalSequence&&PhysicalBoundary&&PhysicalBoundary->IsValid();
    }
};
// Sole mapper-lane participant. Preparation copies the ledger; failed mapping
// cannot retire a contribution, replace its Started identity, or advance clocks.
class FDesktopMappingParticipant
{
    struct FContribution
    {
        FDesktopControlIdentity Origin;FActionId Action=0;
        std::uint64_t RawActivation=0,ActionActivation=0;
        std::optional<FInputEdgeIdentity> Started;
    };
    struct FState
    {
        std::vector<FContribution> Contributions;
        std::vector<FDesktopContribution> Raw;
        std::vector<FDesktopSubsource> Members;
        std::map<FDesktopSubsource,std::uint64_t> Local;
        std::array<std::optional<FInputEdgeIdentity>,ActionCount> Started{};
        std::array<std::uint64_t,ActionCount> ActionActivation{};
        std::uint64_t Producer=0,Epoch=0,Revision=0,Accepted=0,LastGroup=0,Activation=0;
        bool Active=false;
    };
public:
    static constexpr std::size_t MaxActionTerminals=4096,MaxPreparedChanges=4096;
    class FPrepared
    {
        friend class FDesktopMappingParticipant;
        FState Next;
        const FDesktopMappingParticipant* Owner=nullptr;
        std::uint64_t BaseSerial=0;
        std::vector<FDesktopIngressChange> Changes;
        std::size_t Cursor=0;
        const FInputActionContract* Contract=nullptr;
        bool Baseline=false,Valid=false,Finished=false;
        bool End(std::size_t I,EIngressCause Cause,std::uint64_t RawEnd,
            std::optional<FDesktopPhysicalBoundary> Boundary={})
        {
            if(Terminals.size()==MaxActionTerminals)return false;
            const auto C=Next.Contributions[I];
            FDesktopActionTerminal T;T.Origin=C.Origin;T.Action=C.Action;T.RawActivation=C.RawActivation;
            T.ActionActivation=C.ActionActivation;T.StartedEdge=C.Started;T.Cause=Cause;
            T.Clock=Boundary?EDesktopTerminalClock::PhysicalBoundary:EDesktopTerminalClock::Ingress;
            T.IngressTerminalSequence=RawEnd;T.PhysicalBoundary=Boundary;
            if(!T.IsValid())return false;
            Terminals.push_back(std::move(T));Next.Contributions.erase(Next.Contributions.begin()+I);return true;
        }
        bool Add(FDesktopControlIdentity Origin,std::uint64_t RawActivation)
        {
            for(const auto& B:Contract->GetDescription().Mapping)
            {
                if(!(B.Control==Origin.Control)||B.Scale==0)continue;
                const auto* A=Contract->Find(B.Action);if(!A||A->Wiring!=EActionWiring::Wired)continue;
                bool Existing=false;for(const auto& C:Next.Contributions)
                    if(C.Origin==Origin&&C.Action==B.Action){Existing=true;break;}
                if(Existing)continue;
                if(Next.Contributions.size()==MaxActionTerminals)return false;
                Next.Contributions.push_back({Origin,B.Action,RawActivation,Next.ActionActivation[B.Action],Next.Started[B.Action]});
            }
            return true;
        }
        bool Apply(const FDesktopIngressChange& C)
        {
            for(std::size_t I=Next.Contributions.size();I--;)
                if(Next.Contributions[I].Origin==C.Origin&&!End(I,EIngressCause::Hardware,C.GlobalSequence))return false;
            return C.Value==0||Add(C.Origin,C.GlobalSequence);
        }
    public:
        std::vector<FDesktopActionTerminal> Terminals;
        bool Advance(std::uint64_t ObservationGroup)
        {
            if(!Valid)return false;
            while(Cursor<Changes.size()&&(!ObservationGroup||Changes[Cursor].ObservationGroup<=ObservationGroup))
                if(!Apply(Changes[Cursor++]))return Valid=false;
            return true;
        }
        bool Transition(FActionId Action,ETransition State,const FInputEdgeIdentity& Identity)
        {
            if(!Valid||Action>=ActionCount||!Identity.IsValid()||Identity.Action!=Action)return false;
            if(State==ETransition::Started)
            {
                if(Next.Activation==UINT64_MAX)return Valid=false;
                Next.ActionActivation[Action]=++Next.Activation;Next.Started[Action]=Identity;
            }
            else Next.Started[Action].reset();
            for(auto& C:Next.Contributions)if(C.Action==Action)
            {C.ActionActivation=Next.ActionActivation[Action];C.Started=Next.Started[Action];}
            return true;
        }
        // The producer may suppress resume-held edges before publishing the
        // final physical frame. Rebind only identities authored in THIS prepared
        // frame, so metadata names its final immutable transition ordinals.
        bool RebindFrameEdges(const FInputFrameData& Before,const FInputFrameData& After)
        {
            if(!Valid||Before.StreamEpoch.Value!=After.StreamEpoch.Value||Before.SourceSequence!=After.SourceSequence
                ||Before.ConsumptionFrame!=After.ConsumptionFrame||Before.Producer.Id!=After.Producer.Id
                ||Before.Producer.Kind!=After.Producer.Kind||Before.DeviceGeneration.Value!=After.DeviceGeneration.Value
                ||Before.Reset!=After.Reset)return false;
            const auto Rebind=[&](std::optional<FInputEdgeIdentity>& Id)
            {
                if(!Id||Id->Epoch!=Before.StreamEpoch.Value||Id->Frame!=Before.ConsumptionFrame
                    ||Id->SourceSequence!=Before.SourceSequence||Id->ProducerId!=Before.Producer.Id
                    ||Id->ProducerKind!=Before.Producer.Kind)return true;
                if(Id->Ordinal>=Before.Transitions.size())return false;
                const auto& Original=Before.Transitions[Id->Ordinal];
                if(Original.Action!=Id->Action||Original.State!=ETransition::Started)return false;
                for(std::size_t I=0;I<After.Transitions.size();++I)
                {
                    const auto& E=After.Transitions[I];
                    if(E.Action==Original.Action&&E.State==Original.State&&E.ValueAtTransition==Original.ValueAtTransition
                        &&E.Order.Sequence==Original.Order.Sequence&&E.Order.WithinSequence==Original.Order.WithinSequence)
                    {Id->Ordinal=static_cast<std::uint32_t>(I);return true;}
                }
                Id.reset();return true;
            };
            for(auto& Id:Next.Started)if(!Rebind(Id))return false;
            for(auto& C:Next.Contributions)if(!Rebind(C.Started))return false;
            for(auto& T:Terminals)if(!Rebind(T.StartedEdge))return false;
            return true;
        }
        bool Finish(const FInputFrameData& Frame)
        {
            if(!Advance(0))return false;
            if(Baseline)
                for(FActionId A=0;A<ActionCount;++A)if(Frame.ActiveMask&(std::uint32_t(1)<<A))
                {
                    if(Next.Activation==UINT64_MAX)return Valid=false;
                    Next.ActionActivation[A]=++Next.Activation;
                    for(auto& C:Next.Contributions)if(C.Action==A)C.ActionActivation=Next.ActionActivation[A];
                }
            Finished=Valid;return Valid;
        }
    };
    std::optional<FPrepared> Prepare(const FRawInputSample& Sample,const FInputActionContract& Contract,
        FStreamEpoch Epoch,FFrameNumber Frame,FProducerIdentity Producer)const
    {
        if(Serial==UINT64_MAX||Sample.Kind!=ERawDeviceKind::Desktop||!Sample.DesktopIngress||!Sample.DesktopIngress->Latest
            ||Sample.DesktopIngress->Publications.size()>256)return {};
        const auto& In=*Sample.DesktopIngress;const auto& Latest=*In.Latest;
        if(!ValidateDesktopPublication(Latest)||Latest.Baseline.ProducerId!=Producer.Id
            ||Latest.Baseline.StreamEpoch!=Epoch.Value||!SameValues(Latest.Sample.FinalState,Sample.FinalState))return {};
        FPrepared P;P.Next=State;P.Contract=&Contract;P.Valid=true;P.Owner=this;P.BaseSerial=Serial;P.Baseline=Sample.Status==ERawSampleStatus::Resync;
        const FDesktopPhysicalBoundary Boundary{Epoch,Frame,Producer};if(!Boundary.IsValid())return {};
        if(P.Next.Producer&&(P.Next.Producer!=Latest.Baseline.ProducerId||P.Next.Epoch!=Latest.Baseline.StreamEpoch))return {};
        if(P.Baseline)
        {
            if(In.Publications.empty()||(P.Next.Producer&&Latest.Baseline.BaselineRevision<=P.Next.Revision))return {};
            std::uint64_t Revision=0,Accepted=0;
            for(const auto& Held:In.Publications)
            {
                if(!Held||!ValidateDesktopPublication(*Held)||Held->Baseline.ProducerId!=Producer.Id
                    ||Held->Baseline.StreamEpoch!=Epoch.Value||(Revision&&Held->Baseline.BaselineRevision!=Revision+1)
                    ||(Accepted&&Held->Baseline.AcceptedThrough<=Accepted))return {};
                Revision=Held->Baseline.BaselineRevision;Accepted=Held->Baseline.AcceptedThrough;
            }
            if(Revision!=Latest.Baseline.BaselineRevision||Accepted!=Latest.Baseline.AcceptedThrough)return {};
            for(std::size_t I=P.Next.Contributions.size();I--;)
            {
                const auto& C=P.Next.Contributions[I];const FRawContributionTerminal* Raw=nullptr;
                for(const auto& Held:In.Publications)for(const auto& T:Held->RawTerminals)
                    if(T.Origin==C.Origin&&T.ActivationSequence==C.RawActivation)Raw=&T;
                if(Raw){if(!P.End(I,Raw->Cause,Raw->TerminalSequence))return {};}
                else if(!P.End(I,EIngressCause::Resync,0,Boundary))return {};
            }
            P.Next.Started={};P.Next.ActionActivation={};P.Next.Raw=Latest.Baseline.Contributions;
            P.Next.Members=Latest.Baseline.Members;P.Next.Local.clear();
            for(const auto& Held:In.Publications)for(const auto& O:Held->Observations)
            {P.Next.Local[O.Source]=O.LocalSequence;P.Next.LastGroup=std::max(P.Next.LastGroup,O.ObservationGroup);}
            for(const auto& C:P.Next.Raw)if(C.Value!=0&&!P.Add(C.Origin,C.LastGlobalSequence))return {};
        }
        else
        {
            if(!P.Next.Active)return {};
            std::vector<FRawChange> Expected;
            for(const auto& Held:In.Publications)
            {
                if(!Held||!ValidateDesktopPublication(*Held)||Held->RequiresLifecycleEnvelope
                    ||Held->Baseline.ProducerId!=P.Next.Producer||Held->Baseline.StreamEpoch!=P.Next.Epoch
                    ||P.Next.Revision==UINT64_MAX||Held->Baseline.BaselineRevision!=P.Next.Revision+1
                    ||Held->Baseline.AcceptedThrough<=P.Next.Accepted
                    ||!SameMembers(P.Next.Members,Held->Baseline.Members)
                    ||!SameValues(Union(P.Next.Raw),Held->PreviousUnion)||!Held->RawTerminals.empty())return {};
                if(!ValidateOrdinary(*Held,P,Expected))return {};
                P.Next.Revision=Held->Baseline.BaselineRevision;P.Next.Accepted=Held->Baseline.AcceptedThrough;
            }
            if(P.Next.Revision!=Latest.Baseline.BaselineRevision||P.Next.Accepted!=Latest.Baseline.AcceptedThrough
                ||!SameRaw(P.Next.Raw,Latest.Baseline.Contributions)||Expected.size()!=Sample.Changes.size())return {};
            for(std::size_t I=0;I<Expected.size();++I)
                if(!SameValue(Expected[I].State,Sample.Changes[I].State)||!Sample.Changes[I].AtomicGroup
                    ||Expected[I].AtomicGroup!=Sample.Changes[I].AtomicGroup)return {};
        }
        P.Next.Producer=Latest.Baseline.ProducerId;P.Next.Epoch=Latest.Baseline.StreamEpoch;
        P.Next.Revision=Latest.Baseline.BaselineRevision;P.Next.Accepted=Latest.Baseline.AcceptedThrough;P.Next.Active=true;
        return P;
    }
    std::optional<FPrepared> PrepareReplacement(const FInputActionContract& Contract,
        FStreamEpoch Epoch,FFrameNumber Frame,FProducerIdentity Producer)const
    {
        if(Serial==UINT64_MAX)return {};
        FPrepared P;P.Next=State;P.Contract=&Contract;P.Valid=true;P.Owner=this;P.BaseSerial=Serial;
        const FDesktopPhysicalBoundary Boundary{Epoch,Frame,Producer};if(!Boundary.IsValid())return {};
        for(std::size_t I=P.Next.Contributions.size();I--;)
            if(!P.End(I,EIngressCause::Replacement,0,Boundary))return {};
        P.Next.Started={};P.Next.ActionActivation={};P.Next.Active=false;return P;
    }
    bool HasActiveSource()const{return State.Active;}
    bool Commit(FPrepared&& P)
    {
        if(P.Owner!=this||P.BaseSerial!=Serial||!P.Valid||!P.Finished||Serial==UINT64_MAX)return false;
        State=std::move(P.Next);LastTerminals=std::move(P.Terminals);++Serial;P.Valid=false;P.Owner=nullptr;return true;
    }
    const std::vector<FDesktopActionTerminal>& Terminals()const{return LastTerminals;}
private:
    static bool SameValue(const FRawValue& A,const FRawValue& B){return A.Control==B.Control&&A.Value==B.Value;}
    static bool SameValues(const std::vector<FRawValue>& A,const std::vector<FRawValue>& B)
    {if(A.size()!=B.size())return false;for(std::size_t I=0;I<A.size();++I)if(!SameValue(A[I],B[I]))return false;return true;}
    static bool SameMembers(const std::vector<FDesktopSubsource>& A,const std::vector<FDesktopSubsource>& B)
    {return A==B;}
    static bool SameRaw(const std::vector<FDesktopContribution>& A,const std::vector<FDesktopContribution>& B)
    {
        if(A.size()!=B.size())return false;
        for(std::size_t I=0;I<A.size();++I)
            if(!(A[I].Origin==B[I].Origin)||A[I].Value!=B[I].Value||A[I].LastGlobalSequence!=B[I].LastGlobalSequence)return false;
        return true;
    }
    static std::vector<FRawValue> Union(const std::vector<FDesktopContribution>& Raw)
    {
        std::map<FRawControl,float> Values;for(const auto& C:Raw){auto& V=Values[C.Origin.Control];V=std::max(V,C.Value);}
        std::vector<FRawValue> Out;for(const auto& V:Values)Out.push_back({V.first,V.second});return Out;
    }
    static bool ValidateOrdinary(const FDesktopPublication& Pub,FPrepared& P,std::vector<FRawChange>& Expected)
    {
        std::size_t ChangeIndex=0,SampleIndex=0;std::uint64_t Accepted=P.Next.Accepted;
        for(const auto& O:Pub.Observations)
        {
            if(!O.LocalSequence||!O.ObservationGroup||O.ObservationGroup<=P.Next.LastGroup
                ||O.AcceptanceSequence<=Accepted||O.AcceptanceSequence>Pub.Baseline.AcceptedThrough
                ||O.FirstChange!=ChangeIndex||O.ChangeCount>Pub.Changes.size()-ChangeIndex
                ||!std::binary_search(P.Next.Members.begin(),P.Next.Members.end(),O.Source))return false;
            const auto Local=P.Next.Local.find(O.Source);
            if(Local!=P.Next.Local.end()&&(Local->second==UINT64_MAX||O.LocalSequence!=Local->second+1))return false;
            const auto Before=Union(P.Next.Raw);Accepted=O.AcceptanceSequence;
            for(std::size_t J=0;J<O.ChangeCount;++J)
            {
                const auto& C=Pub.Changes[ChangeIndex++];
                if(!(C.Origin.Source==O.Source)||C.LocalSequence!=O.LocalSequence||C.OsTimestampUs!=O.OsTimestampUs
                    ||C.ObservationGroup!=O.ObservationGroup||C.WithinObservation!=J||Accepted==UINT64_MAX
                    ||C.GlobalSequence!=Accepted+1||C.Cause!=EIngressCause::Hardware)return false;
                auto It=std::find_if(P.Next.Raw.begin(),P.Next.Raw.end(),[&](const FDesktopContribution& R){return R.Origin==C.Origin;});
                if(It==P.Next.Raw.end()||It->Value==C.Value||P.Changes.size()==MaxPreparedChanges)return false;
                It->Value=C.Value;It->LastGlobalSequence=C.GlobalSequence;Accepted=C.GlobalSequence;P.Changes.push_back(C);
            }
            const auto After=Union(P.Next.Raw);std::uint32_t Within=0;
            if(Before.size()!=After.size())return false;
            for(std::size_t J=0;J<After.size();++J)if(!SameValue(Before[J],After[J]))
            {
                if(SampleIndex==Pub.Sample.Changes.size())return false;
                const auto& C=Pub.Sample.Changes[SampleIndex++];
                if(!SameValue(C.State,After[J])||C.AtomicGroup!=O.ObservationGroup||C.Order.Sequence!=Accepted||C.Order.WithinSequence!=Within++)return false;
                Expected.push_back(C);
            }
            P.Next.Local[O.Source]=O.LocalSequence;P.Next.LastGroup=O.ObservationGroup;
        }
        if(ChangeIndex!=Pub.Changes.size()||SampleIndex!=Pub.Sample.Changes.size()
            ||(Pub.Observations.empty()?Pub.Baseline.AcceptedThrough!=P.Next.Accepted+1:Accepted!=Pub.Baseline.AcceptedThrough)
            ||!SameRaw(P.Next.Raw,Pub.Baseline.Contributions))return false;
        return true;
    }
    std::uint64_t Serial=0;
    FState State;
    std::vector<FDesktopActionTerminal> LastTerminals;
};
}
