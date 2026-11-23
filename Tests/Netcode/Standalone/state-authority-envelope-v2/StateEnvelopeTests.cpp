#include "StateAuthorityEnvelope.h"
#include "../game-declared-state/TypedTransportFixture.h"
#include "../pad-state/DeclaredPadState.h"
#include <cstdio>
#include <cstdlib>
#include <new>
#include <stdexcept>
namespace S=Speed::Netcode::StateAuthorityV2;
namespace N=Speed::Netcode;
namespace C=Speed::Netcode::Consumed;
static int Checks=0;
static std::size_t Allocations=0,FailAt=0;
void* operator new(std::size_t Size)
{++Allocations;if(FailAt && Allocations==FailAt)throw std::bad_alloc();if(auto* P=std::malloc(Size?Size:1))return P;throw std::bad_alloc();}
void* operator new[](std::size_t Size){return ::operator new(Size);}
void operator delete(void* P)noexcept{std::free(P);}
void operator delete[](void* P)noexcept{std::free(P);}
void operator delete(void* P,std::size_t)noexcept{std::free(P);}
void operator delete[](void* P,std::size_t)noexcept{std::free(P);}
#define CHECK(X) do{++Checks;if(!(X)){std::printf("FAIL line%d: %s\n",__LINE__,#X);std::exit(1);}}while(false)
struct FLedgerFixture final:S::IFullSnapshotEvidence
{
    std::vector<std::uint8_t> Exact;
    mutable unsigned Calls=0;
    mutable S::FEnvelope* MutateEnvelope=nullptr;
    mutable S::FTrustedSnapshot* MutateTrusted=nullptr;
    bool Confirms(const S::FEnvelope&,const S::FTrustedSnapshot&,const std::vector<std::uint8_t>& B)const noexcept override
    {
        ++Calls;
        if(MutateEnvelope){MutateEnvelope->State.Actors.clear();MutateEnvelope->Decisions[0].Applied[0]=3;MutateEnvelope=nullptr;}
        if(MutateTrusted){MutateTrusted->Roster.clear();MutateTrusted->Inputs.clear();MutateTrusted=nullptr;}
        return B==Exact;
    }
}; // Authored complete-world evidence fixture. NO native origin ledger exists.
struct FFixture
{
    S::FEnvelope Envelope;
    S::FTrustedSnapshot Trusted;
    FLedgerFixture Ledger;
    FFixture()
    {
        auto& A=Envelope.Address;A.World={700,4};A.AuthorityPeer=20;A.AuthorityEpoch=3;
        A.AuthorityTimelineRevision=7;A.BeforeWorldPublication=16;A.PublicationSerial=17;
        A.WorldStateHash=12345;A.StateSchema[0]=23;
        Envelope.State=N::DeclaredGameFixture::Snapshot(1);A.Frame=Envelope.State.Frame;A.RegistryGeneration=Envelope.State.RegistryGeneration;
        N::FCanonicalActorState Pad;Pad.HasBody=false;Pad.MechanicalSchema=N::Pads::DeclaredStateSchema;Pad.GameplaySchema=N::Pads::DeclaredStateSchema;
        Pad.Mechanical=N::Pads::Encode({42,65535});Pad.Gameplay=N::Pads::Encode({42,65535});
        Envelope.State.Actors.push_back({300,1,3,1,N::ELifecycle::Alive,*N::EncodeCanonicalActor(Pad)});
        C::FTrustedContext Context;auto& B=Context.Binding;
        B.Session=11;B.Epoch=7;B.Actor=100;B.Incarnation=1;B.OwnerPeer=10;B.ServerPeer=A.AuthorityPeer;
        B.SourceProducer=99;B.Contract[0]=23;Context.World=A.World;Context.AuthorityEpoch=A.AuthorityEpoch;
        Context.AuthorityTimelineRevision=A.AuthorityTimelineRevision;Context.BeforeWorldPublication=A.BeforeWorldPublication;
        Context.Frame=A.Frame;Context.RegistryVersion=A.RegistryGeneration;Context.Targets={{100,1}};
        Context.Processing.Origin=C::EOrigin::Device;Context.Processing.Before=std::array<std::int16_t,N::ActionCount>{};
        (*Context.Processing.Before)[0]=64;Context.Processing.Step[0]=10;Context.Processing.BindingFingerprint={1,2,3};
        Context.Actions[0]={true,true,false,255};Context.Actions[1]={true,true,false,255};Context.Actions[2]={true,true,true,127};
        N::FPacket Request;Request.Session=B.Session;Request.Epoch=B.Epoch;Request.Actor=B.Actor;Request.Incarnation=B.Incarnation;
        Request.Kind=N::EKind::OwnerInput;Request.Contract=B.Contract;Request.Sequence=2;Request.Frame=A.Frame;
        Request.Input.SourceSequence=2;Request.Input.DeviceGeneration=7;Request.Input.ProducerId=99;Request.Input.Values[0]=255;
        auto D=C::Propose(Request,Context);CHECK(D && D->Applied[0]==74);
        D->Stage=C::EStage::Published;D->Publication={A.AuthorityPeer,A.AuthorityEpoch,A.PublicationSerial,A.WorldStateHash};
        Envelope.Decisions.push_back(*D);Trusted.Address=A;Trusted.Inputs.push_back(Context);
        for(const auto& Actor:Envelope.State.Actors)Trusted.Roster.push_back({Actor.Actor,Actor.Incarnation,Actor.Type,Actor.SchemaVersion,Actor.Lifecycle});
        const auto Bytes=S::Encode(Envelope);CHECK(Bytes);Ledger.Exact=*Bytes;
    }
};
static void RoundTripAndEvidence()
{
    FFixture F;const auto& Bytes=F.Ledger.Exact;
    CHECK(Bytes.size()>S::HeaderBytes);auto Decoded=S::Decode(Bytes);CHECK(Decoded && S::Encode(*Decoded)==Bytes);
    CHECK(S::MatchesTrusted(*Decoded,F.Trusted));
    CHECK(Decoded->Decisions[0].Original.Input.Values[0]==255 && Decoded->Decisions[0].Applied[0]==74);
    CHECK(Decoded->State.Actors.size()==3 && Decoded->State.Sequence!=Decoded->Address.PublicationSerial);
    S::IFullSnapshotEvidence Default;CHECK(!S::Accept(20,*Decoded,F.Trusted,Default));
    CHECK(!S::Accept(10,*Decoded,F.Trusted,F.Ledger) && F.Ledger.Calls==0);
    for(int Role=0;Role<2;++Role)
    {
        const auto Accepted=S::Accept(20,*Decoded,F.Trusted,F.Ledger);CHECK(Accepted);
        CHECK(Accepted->CanonicalBytes()==Bytes && S::Encode(Accepted->Source())==Bytes);
        CHECK(Accepted->OriginContext().Roster.size()==3 && Accepted->OriginContext().Inputs.size()==1);
        const auto Car=N::DecodeCanonicalActor(Accepted->Source().State.Actors[0].Payload);CHECK(Car);
        const auto CarPhysical=N::DeclaredGame::Decode<N::DeclaredGame::FCarPhysical>({F.Envelope.Address.Frame,9,100,1},Car->Mechanical);
        CHECK(CarPhysical && CarPhysical->boostQty==101 && std::signbit(CarPhysical->WheelsOmega[0]));
        const auto Pad=N::DecodeCanonicalActor(Accepted->Source().State.Actors[2].Payload);CHECK(Pad);
        const auto PadPhysical=N::Pads::Decode(Pad->Gameplay);CHECK(PadPhysical && PadPhysical->StateSerial==65535 && PadPhysical->FramesBeforeActive==42);
    }
    for(std::size_t N=0;N<Bytes.size();++N)CHECK(!S::Decode(std::vector<std::uint8_t>(Bytes.begin(),Bytes.begin()+N)));
    auto Bad=Bytes;Bad.push_back(0);CHECK(!S::Decode(Bad));
    Bad=Bytes;Bad[4]=1;CHECK(!S::Decode(Bad)); // No implicit v1 upgrade.
    Bad=Bytes;Bad[6]=0;CHECK(!S::Decode(Bad));
    Bad=Bytes;for(std::size_t I=120;I<124;++I)Bad[I]=255;CHECK(!S::Decode(Bad));
    Bad=Bytes;Bad[124]=65;Bad[125]=0;CHECK(!S::Decode(Bad));
    Bad=Bytes;Bad[126]=1;CHECK(!S::Decode(Bad));
    Bad=Bytes;const auto Inner=N::EncodeState(F.Envelope.State);CHECK(Inner);
    for(std::size_t I=0;I<4;++I)Bad[S::HeaderBytes+Inner->size()+I]=255;CHECK(!S::Decode(Bad));
    static_assert(!S::NativeWorldRestoreQualified && !C::NativeEndpointQualified);
}
static void ContextAndMembership()
{
    FFixture F;
    const auto Refuse=[&](const S::FEnvelope& E,const S::FTrustedSnapshot& T)
    {const auto Before=F.Ledger.Calls;CHECK(!S::Accept(20,E,T,F.Ledger));CHECK(F.Ledger.Calls==Before);};
    N::U64 S::FAddress::* Members[]={&S::FAddress::AuthorityPeer,&S::FAddress::AuthorityEpoch,&S::FAddress::AuthorityTimelineRevision,
        &S::FAddress::BeforeWorldPublication,&S::FAddress::PublicationSerial,&S::FAddress::WorldStateHash,&S::FAddress::Frame,&S::FAddress::RegistryGeneration};
    for(auto Member:Members){auto T=F.Trusted;++(T.Address.*Member);Refuse(F.Envelope,T);}
    auto T=F.Trusted;++T.Address.World.Id;Refuse(F.Envelope,T);T=F.Trusted;++T.Address.World.Incarnation;Refuse(F.Envelope,T);
    T=F.Trusted;T.Address.StateSchema[0]^=1;Refuse(F.Envelope,T);
    for(int Kind=0;Kind<5;++Kind)
    {
        T=F.Trusted;if(Kind==0)++T.Roster[1].Actor;if(Kind==1)++T.Roster[1].Incarnation;
        if(Kind==2)++T.Roster[1].Type;if(Kind==3)++T.Roster[1].SchemaVersion;if(Kind==4)T.Roster[1].Lifecycle=N::ELifecycle::Destroyed;
        Refuse(F.Envelope,T);
    }
    auto E=F.Envelope;E.State.Actors.pop_back();CHECK(S::StructurallyValid(E));Refuse(E,F.Trusted); // Pad has no input session.
    E=F.Envelope;E.Decisions.clear();CHECK(S::StructurallyValid(E));Refuse(E,F.Trusted);
    T=F.Trusted;T.Inputs.clear();Refuse(F.Envelope,T);
    T=F.Trusted;++T.Inputs[0].AuthorityEpoch;Refuse(F.Envelope,T);
    T=F.Trusted;++T.Inputs[0].Processing.BindingFingerprint[0];Refuse(F.Envelope,T);
    E=F.Envelope;E.Decisions[0].Applied[0]=75;Refuse(E,F.Trusted);
    // Same computed Applied does not replace the original requested frame:
    // the exact full-world origin ledger must still reject changed provenance.
    E=F.Envelope;E.Decisions[0].Original.Input.Values[0]=240;CHECK(S::MatchesTrusted(E,F.Trusted));
    const auto Calls=F.Ledger.Calls;CHECK(!S::Accept(20,E,F.Trusted,F.Ledger));CHECK(F.Ledger.Calls==Calls+1);
    // Even a matching diagnostic world hash does not identify a world/branch.
    for(int Branch=0;Branch<3;++Branch)
    {
        E=F.Envelope;if(Branch==0)++E.Address.World.Id;if(Branch==1)++E.Address.World.Incarnation;
        if(Branch==2)++E.Address.AuthorityTimelineRevision;
        for(auto& D:E.Decisions){D.World=E.Address.World;D.AuthorityTimelineRevision=E.Address.AuthorityTimelineRevision;}
        CHECK(S::Encode(E));Refuse(E,F.Trusted);
    }
    E=F.Envelope;E.Decisions.push_back(E.Decisions[0]);CHECK(!S::Encode(E));
    E=F.Envelope;E.Decisions[0].Targets={{999,1}};CHECK(!S::Encode(E));
    E=F.Envelope;E.Decisions[0].Stage=C::EStage::Proposed;E.Decisions[0].Publication={};CHECK(!S::Encode(E));
    E=F.Envelope;E.Address.PublicationSerial=E.Address.BeforeWorldPublication;CHECK(!S::Encode(E));
    E=F.Envelope;E.State.WorldPayload.resize(S::MaxEnvelopeBytes);CHECK(!S::Encode(E));
    E=F.Envelope;E.Decisions[0].Processing.BindingFingerprint.resize(C::MaxFingerprint,42);
    for(N::U64 Session=12;Session<=14;++Session){auto D=E.Decisions[0];D.Original.Session=Session;E.Decisions.push_back(std::move(D));}
    CHECK(S::StructurallyValid(E));CHECK(!S::Encode(E)); // Valid inner records exceed the whole-envelope cap.
}
static void CallbackPinsAndLifetimes()
{
    FFixture F;const auto Expected=F.Ledger.Exact;
    F.Ledger.MutateEnvelope=&F.Envelope;F.Ledger.MutateTrusted=&F.Trusted;
    const auto Accepted=S::Accept(20,F.Envelope,F.Trusted,F.Ledger);CHECK(Accepted);
    CHECK(F.Envelope.State.Actors.empty() && F.Trusted.Roster.empty());
    CHECK(Accepted->CanonicalBytes()==Expected && Accepted->Source().State.Actors.size()==3);
    CHECK(Accepted->Source().Decisions[0].Applied[0]==74 && Accepted->OriginContext().Inputs.size()==1);
}
static void ExplicitTombstoneAndEmptyInputSet()
{
    FFixture F;auto E=F.Envelope;auto T=F.Trusted;
    E.State.Actors[0].Lifecycle=N::ELifecycle::Destroyed;E.State.Actors[0].Payload.clear();
    T.Roster[0].Lifecycle=N::ELifecycle::Destroyed;CHECK(S::MatchesTrusted(E,T));
    F.Ledger.Exact=*S::Encode(E);CHECK(S::Accept(20,E,T,F.Ledger));
    E=F.Envelope;E.Decisions.clear();T=F.Trusted;T.Inputs.clear();
    F.Ledger.Exact=*S::Encode(E);CHECK(S::Accept(20,E,T,F.Ledger));
    CHECK(!S::Accept(20,E,F.Trusted,F.Ledger)); // Empty is allowed only when the trusted registry says so.
}
static void PreparationAllocationFailures()
{
    FFixture F;const auto Expected=F.Ledger.Exact;
    const auto Start=Allocations;auto Complete=S::Accept(20,F.Envelope,F.Trusted,F.Ledger);CHECK(Complete);
    const auto Count=Allocations-Start;CHECK(Count>0);Complete.reset();
    for(std::size_t Budget=1;Budget<=Count;++Budget)
    {
        bool Threw=false;FailAt=Allocations+Budget;
        try{auto P=S::Accept(20,F.Envelope,F.Trusted,F.Ledger);CHECK(!P);}
        catch(const std::bad_alloc&){Threw=true;}
        FailAt=0;CHECK(Threw);CHECK(S::Encode(F.Envelope)==Expected);CHECK(S::MatchesTrusted(F.Envelope,F.Trusted));
    }
    std::printf("ALLOCATION_FAILURE_POSITIONS %zu; no publication or native live state touched\n",Count);
}
#include "FragmentTests.h"
int main()
{
    RoundTripAndEvidence();ContextAndMembership();CallbackPinsAndLifetimes();ExplicitTombstoneAndEmptyInputSet();PreparationAllocationFailures();
    FragmentOwnerRemoteAndContradictoryDuplicates();FragmentNoSequenceWrap();FragmentAllocationRetry();FragmentProgressionAndRetainedOrigin();
    std::printf("PASS %d checks; state authorityv2+input decisions+complete FIXTURE roster; source/world/native ledger SYNTHETIC or default-deny; native NOT_RUN\n",Checks);
    return 0;
}
