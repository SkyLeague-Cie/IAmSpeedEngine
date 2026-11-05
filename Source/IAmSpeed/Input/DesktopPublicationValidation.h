#pragma once
#include "DesktopInputSource.h"
#include <map>

namespace Speed::Input::V2
{
// Copied alongside the physical sample. The producer/journal delivery clock
// remains separate from each subsource's OS/global observation clocks.
struct FDesktopFrameIngress
{
    std::vector<std::shared_ptr<const FDesktopPublication>> Publications;
    std::shared_ptr<const FDesktopPublication> Latest;
};
inline bool ValidateDesktopPublication(const FDesktopPublication& P)
{
    const auto& B=P.Baseline;
    if(P.Schema!=FDesktopPublication::SchemaVersion||P.Ordering!=FDesktopPublication::IngressOrderingVersion
        ||P.Fingerprint!=FDesktopPublication::ContractFingerprint||!B.ProducerId||!B.StreamEpoch
        ||!B.CompositeGeneration||!B.BaselineRevision||!B.AcceptedThrough
        ||B.Members.size()>FDesktopInputSource::MaxMembers||B.Contributions.size()>FDesktopInputSource::MaxContributions
        ||P.Changes.size()>FDesktopInputSource::MaxChanges||P.Observations.size()>FDesktopInputSource::MaxObservations
        ||P.RawTerminals.size()>FDesktopInputSource::MaxContributions||P.Sample.Kind!=ERawDeviceKind::Desktop
        ||P.Sample.DeviceId!=B.ProducerId||P.Sample.Generation.Value!=B.CompositeGeneration
        ||P.Sample.Sequence!=B.AcceptedThrough||P.Sample.Status!=(P.RequiresLifecycleEnvelope?ERawSampleStatus::Resync:ERawSampleStatus::Valid))return false;
    for(std::size_t I=0;I<B.Members.size();++I)
    {
        const auto& M=B.Members[I];
        if(M.Kind>EDesktopSubsourceKind::Mouse||!M.DeviceId||!M.Generation||(I&&!(B.Members[I-1]<M)))return false;
    }
    std::map<FRawControl,float> Union;
    for(std::size_t I=0;I<B.Contributions.size();++I)
    {
        const auto& C=B.Contributions[I];
        if(!C.Origin.Control.Accepts(C.Value)||!C.LastGlobalSequence||C.LastGlobalSequence>B.AcceptedThrough
            ||(C.Origin.Control.Kind!=(C.Origin.Source.Kind==EDesktopSubsourceKind::Keyboard
                ?ERawControlKind::KeyboardUsage:ERawControlKind::MouseButton))
            ||!std::binary_search(B.Members.begin(),B.Members.end(),C.Origin.Source))return false;
        if(I)
        {
            const auto& Before=B.Contributions[I-1].Origin;
            if(!(Before.Source<C.Origin.Source)&&(!(Before.Source==C.Origin.Source)||!(Before.Control<C.Origin.Control)))return false;
        }
        auto& Value=Union[C.Origin.Control];Value=std::max(Value,C.Value);
    }
    if(Union.size()!=P.Sample.FinalState.size()||Union.size()>FRawInputSample::MaxControls)return false;
    std::size_t Index=0;
    for(const auto& V:Union)
    {
        const auto& Actual=P.Sample.FinalState[Index++];
        if(!(V.first==Actual.Control)||V.second!=Actual.Value)return false;
    }
    if(!P.Sample.FinalState.empty()&&!P.Sample.IsValid())return false;
    if(P.Sample.FinalState.empty()&&!P.RequiresLifecycleEnvelope)return false;
    std::uint64_t Last=0;
    for(const auto& C:P.Changes)
    {
        if(C.Cause!=EIngressCause::Hardware||!C.GlobalSequence||C.GlobalSequence<=Last||C.GlobalSequence>B.AcceptedThrough
            ||!C.LocalSequence||!C.ObservationGroup||!C.Origin.Control.Accepts(C.Value)
            ||!std::binary_search(B.Members.begin(),B.Members.end(),C.Origin.Source))return false;
        Last=C.GlobalSequence;
    }
    Last=0;
    for(const auto& T:P.RawTerminals)
    {
        if(T.Cause==EIngressCause::Hardware||T.Cause>EIngressCause::Resync||!T.ActivationSequence
            ||T.TerminalSequence<=T.ActivationSequence||T.TerminalSequence<=Last||T.TerminalSequence>B.AcceptedThrough
            ||!T.Origin.Source.DeviceId||!T.Origin.Source.Generation||!T.Origin.Control.IsValid())return false;
        Last=T.TerminalSequence;
    }
    return true;
}
}
