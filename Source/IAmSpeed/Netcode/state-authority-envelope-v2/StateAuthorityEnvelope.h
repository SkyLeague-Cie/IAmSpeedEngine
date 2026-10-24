#pragma once
#include "../consumed-command-v2/ConsumedCommand.h"
#include "../transport/NetStateTransport.h"
#include <memory>

namespace Speed::Netcode::StateAuthorityV2
{
namespace C=Consumed;
inline constexpr bool NativeWorldRestoreQualified=false;
inline constexpr std::size_t HeaderBytes=128,MaxEnvelopeBytes=MaxWorldState;
struct FAddress
{
    C::FWorldIdentity World;
    U64 AuthorityPeer=0,AuthorityEpoch=0,AuthorityTimelineRevision=0;
    U64 BeforeWorldPublication=0,PublicationSerial=0,WorldStateHash=0;
    U64 Frame=0,RegistryGeneration=0;
    std::array<std::uint8_t,32> StateSchema{};
    bool Valid() const noexcept
    {
        bool Schema=false;for(auto B:StateSchema)Schema|=B!=0;
        return World.Valid() && AuthorityPeer && AuthorityEpoch && AuthorityTimelineRevision
            && BeforeWorldPublication && PublicationSerial>BeforeWorldPublication
            && WorldStateHash && RegistryGeneration && Schema;
    }
    bool operator==(const FAddress& R) const noexcept
    {
        return World==R.World && AuthorityPeer==R.AuthorityPeer && AuthorityEpoch==R.AuthorityEpoch
            && AuthorityTimelineRevision==R.AuthorityTimelineRevision && BeforeWorldPublication==R.BeforeWorldPublication
            && PublicationSerial==R.PublicationSerial && WorldStateHash==R.WorldStateHash
            && Frame==R.Frame && RegistryGeneration==R.RegistryGeneration && StateSchema==R.StateSchema;
    }
};
struct FEnvelope
{
    FAddress Address;
    FWorldState State;
    // Complete authoritative INPUT source set, sorted by session. Non-input
    // actors remain mandatory members of the independent full actor roster.
    std::vector<C::FDecision> Decisions;
};
inline bool TargetPresent(const FWorldState& S,const C::FTarget T) noexcept
{
    for(const auto& A:S.Actors)if(A.Actor==T.Actor && A.Incarnation==T.Incarnation)return true;
    return false;
}
inline bool StructurallyValid(const FEnvelope& E)
{
    const auto& A=E.Address;
    if(!A.Valid() || !ValidState(E.State) || A.Frame!=E.State.Frame
        || A.RegistryGeneration!=E.State.RegistryGeneration || E.Decisions.size()>C::MaxTargets)return false;
    U64 Previous=0;
    for(const auto& D:E.Decisions)
    {
        if(!C::StructurallyValid(D) || D.Stage!=C::EStage::Published || D.Original.Session<=Previous
            || D.World!=A.World || D.Original.Frame!=A.Frame || D.RegistryVersion!=A.RegistryGeneration
            || D.AuthorityTimelineRevision!=A.AuthorityTimelineRevision || D.BeforeWorldPublication!=A.BeforeWorldPublication
            || D.Publication.AuthorityPeer!=A.AuthorityPeer || D.Publication.AuthorityEpoch!=A.AuthorityEpoch
            || D.Publication.Serial!=A.PublicationSerial || D.Publication.WorldStateHash!=A.WorldStateHash)return false;
        if(std::find(D.Targets.begin(),D.Targets.end(),C::FTarget{D.Original.Actor,D.Original.Incarnation})==D.Targets.end())return false;
        // A target may have been destroyed during this frame after consuming
        // input. Preserve its explicit tombstone instead of inventing a rule
        // that every consumed target must still be alive at publication.
        for(const auto& Target:D.Targets)if(!TargetPresent(E.State,Target))return false;
        Previous=D.Original.Session;
    }
    return true;
}
inline std::optional<std::vector<std::uint8_t>> Encode(const FEnvelope& E)
{
    if(!StructurallyValid(E))return {};
    const auto StateBytes=EncodeState(E.State);if(!StateBytes)return {};
    std::vector<std::vector<std::uint8_t>> Decisions;Decisions.reserve(E.Decisions.size());
    std::size_t Size=HeaderBytes+StateBytes->size();if(Size>MaxEnvelopeBytes)return {};
    for(const auto& D:E.Decisions)
    {
        auto Bytes=C::EncodeDecision(D);if(!Bytes || Bytes->size()+4>MaxEnvelopeBytes-Size)return {};
        Size+=Bytes->size()+4;Decisions.push_back(std::move(*Bytes));
    }
    std::vector<std::uint8_t> B;B.reserve(Size);const auto& A=E.Address;
    Wire::Put(B,0x32565349,4);Wire::Put(B,2,2);Wire::Put(B,1,2); // Full snapshot, explicit v2.
    for(auto V:{A.World.Id,A.World.Incarnation,A.AuthorityPeer,A.AuthorityEpoch,A.AuthorityTimelineRevision,
                A.BeforeWorldPublication,A.PublicationSerial,A.WorldStateHash,A.Frame,A.RegistryGeneration})Wire::Put(B,V,8);
    B.insert(B.end(),A.StateSchema.begin(),A.StateSchema.end());
    Wire::Put(B,StateBytes->size(),4);Wire::Put(B,Decisions.size(),2);Wire::Put(B,0,2);
    if(B.size()!=HeaderBytes)return {};
    B.insert(B.end(),StateBytes->begin(),StateBytes->end());
    for(const auto& D:Decisions){Wire::Put(B,D.size(),4);B.insert(B.end(),D.begin(),D.end());}
    return B;
}
inline std::optional<FEnvelope> Decode(const std::vector<std::uint8_t>& B)
{
    if(B.size()<HeaderBytes || B.size()>MaxEnvelopeBytes)return {};
    Wire::FReader R{B.data(),B.size()};if(R.Get(4)!=0x32565349 || R.Get(2)!=2 || R.Get(2)!=1)return {};
    FEnvelope E;auto& A=E.Address;
    A.World={R.Get(8),R.Get(8)};A.AuthorityPeer=R.Get(8);A.AuthorityEpoch=R.Get(8);A.AuthorityTimelineRevision=R.Get(8);
    A.BeforeWorldPublication=R.Get(8);A.PublicationSerial=R.Get(8);A.WorldStateHash=R.Get(8);A.Frame=R.Get(8);A.RegistryGeneration=R.Get(8);
    for(auto& V:A.StateSchema)V=std::uint8_t(R.Get(1));
    const auto StateSize=R.Get(4),Count=R.Get(2);if(R.Get(2) || !R.Good || Count>C::MaxTargets || StateSize>B.size()-R.At)return {};
    auto S=DecodeState(B.data()+R.At,std::size_t(StateSize));if(!S)return {};E.State=std::move(*S);R.At+=std::size_t(StateSize);
    E.Decisions.reserve(std::size_t(Count));
    for(U64 I=0;I<Count;++I)
    {
        const auto Length=R.Get(4);if(!R.Good || Length>C::MaxPayload || Length>B.size()-R.At)return {};
        const std::vector<std::uint8_t> Bytes(B.begin()+R.At,B.begin()+R.At+std::size_t(Length));
        auto D=C::DecodeDecision(Bytes);if(!D)return {};E.Decisions.push_back(std::move(*D));R.At+=std::size_t(Length);
    }
    if(!R.Good || R.At!=B.size() || !StructurallyValid(E))return {};
    return E;
}
struct FActorDescriptor
{
    U64 Actor=0,Incarnation=0;
    std::uint32_t Type=0,SchemaVersion=0;
    ELifecycle Lifecycle=ELifecycle::Alive;
    bool Matches(const FActorState& A) const noexcept
    {return Actor==A.Actor && Incarnation==A.Incarnation && Type==A.Type && SchemaVersion==A.SchemaVersion && Lifecycle==A.Lifecycle;}
};
// Supplied by the actual complete authoritative registry/publication ledger,
// including actors with no input session. Never obtain this by copying a
// received packet's declarations or assuming a larger branch is admitted.
struct FTrustedSnapshot
{
    FAddress Address;
    std::vector<FActorDescriptor> Roster;
    std::vector<C::FTrustedContext> Inputs;
};
inline bool MatchesTrusted(const FEnvelope& E,const FTrustedSnapshot& T)
{
    if(!StructurallyValid(E) || !(E.Address==T.Address) || E.State.Actors.size()!=T.Roster.size()
        || E.Decisions.size()!=T.Inputs.size())return false;
    for(std::size_t I=0;I<T.Roster.size();++I)if(!T.Roster[I].Matches(E.State.Actors[I]))return false;
    for(std::size_t I=0;I<T.Inputs.size();++I)
    {
        const auto& Ctx=T.Inputs[I];const auto& D=E.Decisions[I];
        if(Ctx.AuthorityEpoch!=E.Address.AuthorityEpoch || Ctx.Binding.ServerPeer!=E.Address.AuthorityPeer
            || (I && T.Inputs[I-1].Binding.Session>=Ctx.Binding.Session)
            || !C::MatchesTrusted(D,Ctx))return false;
    }
    return true;
}
// This interface must prove actual completed authoritative world publication,
// every per-type payload and complete registry membership on a trusted stream.
// Parsing bytes, an ACK or an input owner completion cannot supply this proof.
class IFullSnapshotEvidence
{
public:
    virtual ~IFullSnapshotEvidence()=default;
    virtual bool Confirms(const FEnvelope&,const FTrustedSnapshot&,const std::vector<std::uint8_t>&) const noexcept {return false;}
};
class FValidatedOriginSnapshot;
inline std::shared_ptr<const FValidatedOriginSnapshot> Accept(U64,const FEnvelope&,const FTrustedSnapshot&,const IFullSnapshotEvidence&);
class FValidatedOriginSnapshot final
{
public:
    const FEnvelope& Source() const noexcept {return *Pinned;}
    const FTrustedSnapshot& OriginContext() const noexcept {return Trusted;}
    const std::vector<std::uint8_t>& CanonicalBytes() const noexcept {return Bytes;}
    FValidatedOriginSnapshot(const FValidatedOriginSnapshot&)=delete;
private:
    friend std::shared_ptr<const FValidatedOriginSnapshot> Accept(U64,const FEnvelope&,const FTrustedSnapshot&,const IFullSnapshotEvidence&);
    FValidatedOriginSnapshot(std::shared_ptr<const FEnvelope> S,FTrustedSnapshot T,std::vector<std::uint8_t> B)
        :Pinned(std::move(S)),Trusted(std::move(T)),Bytes(std::move(B)){}
    const std::shared_ptr<const FEnvelope> Pinned;
    const FTrustedSnapshot Trusted;
    const std::vector<std::uint8_t> Bytes;
};
inline std::shared_ptr<const FValidatedOriginSnapshot> Accept(U64 TrustedSender,const FEnvelope& Source,
    const FTrustedSnapshot& Context,const IFullSnapshotEvidence& Evidence)
{
    const auto Pinned=std::make_shared<const FEnvelope>(Source);const FTrustedSnapshot Trusted=Context;
    if(TrustedSender!=Trusted.Address.AuthorityPeer || !MatchesTrusted(*Pinned,Trusted))return {};
    auto Bytes=Encode(*Pinned);if(!Bytes || !Evidence.Confirms(*Pinned,Trusted,*Bytes))return {};
    return std::shared_ptr<const FValidatedOriginSnapshot>(new FValidatedOriginSnapshot(Pinned,Trusted,std::move(*Bytes)));
}
// Accept retains origin evidence only. It neither restores a native world nor
// issues a LOCAL step/publication/rollback receipt, and opens no native guard.
}
