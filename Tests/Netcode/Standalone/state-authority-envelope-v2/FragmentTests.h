#pragma once
#include "StateFragmentsV2.h"
static S::FStreamBinding Stream(const FFixture& F)
{
    S::FStreamBinding B;B.Session=19;B.StreamEpoch=4;const auto& A=F.Envelope.Address;
    B.World=A.World;B.AuthorityPeer=A.AuthorityPeer;B.AuthorityEpoch=A.AuthorityEpoch;
    B.AuthorityTimelineRevision=A.AuthorityTimelineRevision;B.StateSchema=A.StateSchema;return B;
}
static S::FFragment PacketCopy(const S::FFragment& F)
{
    const auto B=S::EncodeFragmentV2(F);CHECK(B && B->size()<=S::FragmentHeaderBytes+S::ChunkBytes);
    auto P=S::DecodeFragmentV2(*B);CHECK(P);return *P;
}
static void FragmentOwnerRemoteAndContradictoryDuplicates()
{
    FFixture F;const auto Binding=Stream(F);const auto Parts=S::Fragment(F.Envelope,Binding);CHECK(Parts && Parts->size()==3);
    S::FReceiver Owner(Binding,1),Remote(Binding,1);
    CHECK(Owner.Receive(999,Parts->front())==N::EResult::WrongPeer);
    auto Bad=Parts->front();++Bad.Address.World.Incarnation;CHECK(Owner.Receive(20,Bad)==N::EResult::WrongStream);
    Bad=Parts->front();++Bad.Address.AuthorityTimelineRevision;CHECK(Owner.Receive(20,Bad)==N::EResult::WrongStream);
    Bad=Parts->front();++Bad.StreamEpoch;CHECK(Owner.Receive(20,Bad)==N::EResult::WrongStream);
    Bad=Parts->front();++Bad.Sequence;CHECK(Owner.Receive(20,Bad)==N::EResult::TooFar);
    for(auto* R:{&Owner,&Remote})
    {
        CHECK(R->Receive(20,PacketCopy((*Parts)[2]))==N::EResult::Accepted);
        Bad=(*Parts)[0];++Bad.Address.Frame;CHECK(R->Receive(20,Bad)==N::EResult::Conflict);
        CHECK(R->Receive(20,PacketCopy((*Parts)[0]))==N::EResult::Accepted);
        CHECK(R->Receive(20,PacketCopy((*Parts)[0]))==N::EResult::Duplicate);
        CHECK(!R->TakeReady() && !R->Acknowledgement());
        CHECK(R->Receive(20,PacketCopy((*Parts)[1]))==N::EResult::Accepted);
        auto Ready=R->TakeReady();CHECK(Ready && S::Encode(*Ready)==F.Ledger.Exact);
        S::IFullSnapshotEvidence Default;CHECK(!S::Accept(20,*Ready,F.Trusted,Default));
        const auto Accepted=S::Accept(20,*Ready,F.Trusted,F.Ledger);CHECK(Accepted && Accepted->CanonicalBytes()==F.Ledger.Exact);
        const auto Ack=R->Acknowledgement();CHECK(Ack && PacketCopy(*Ack).Ack);
        CHECK(R->Receive(20,PacketCopy((*Parts)[1]))==N::EResult::Duplicate);CHECK(!R->TakeReady());
        Bad=(*Parts)[1];Bad.Bytes[0]^=1;CHECK(R->Receive(20,Bad)==N::EResult::Conflict);CHECK(!R->TakeReady());
        Bad=(*Parts)[1];++Bad.Address.PublicationSerial;CHECK(R->Receive(20,Bad)==N::EResult::Conflict);
        CHECK(R->Receive(20,*Ack)==N::EResult::Invalid);
    }
    const auto Bytes=S::EncodeFragmentV2(Parts->front());CHECK(Bytes);
    for(std::size_t N=0;N<Bytes->size();++N)CHECK(!S::DecodeFragmentV2(std::vector<std::uint8_t>(Bytes->begin(),Bytes->begin()+N)));
    auto Wire=*Bytes;Wire[4]=1;CHECK(!S::DecodeFragmentV2(Wire));
    Wire=*Bytes;Wire[6]=2;CHECK(!S::DecodeFragmentV2(Wire));
    Wire=*Bytes;Wire[7]=1;CHECK(!S::DecodeFragmentV2(Wire));
    Wire=*Bytes;for(std::size_t I=144;I<148;++I)Wire[I]=255;CHECK(!S::DecodeFragmentV2(Wire));
    Wire=*Bytes;Wire[154]=0;Wire[155]=0;CHECK(!S::DecodeFragmentV2(Wire));
    Wire=*Bytes;Wire.push_back(0);CHECK(!S::DecodeFragmentV2(Wire));
    // A corrupt complete attempt is discarded without advancing transport.
    S::FReceiver Corrupt(Binding,1);auto CorruptParts=*Parts;CorruptParts[1].Bytes[0]^=1;
    for(const auto& P:CorruptParts)
    {
        const auto Result=Corrupt.Receive(20,P);
        CHECK(Result==(P.Index==2?N::EResult::Invalid:N::EResult::Accepted));
    }
    CHECK(!Corrupt.TakeReady() && !Corrupt.Acknowledgement());
    for(const auto& P:*Parts)CHECK(Corrupt.Receive(20,P)==N::EResult::Accepted);
    CHECK(Corrupt.TakeReady() && Corrupt.Acknowledgement());
    // A coherent CRC still cannot substitute packet headers for body identity.
    S::FReceiver Header(Binding,1);auto HeaderParts=*Parts;for(auto& P:HeaderParts)++P.Address.Frame;
    for(const auto& P:HeaderParts)
    {const auto Result=Header.Receive(20,P);CHECK(Result==(P.Index==2?N::EResult::Invalid:N::EResult::Accepted));}
    CHECK(!Header.TakeReady() && !Header.Acknowledgement());
}
static void FragmentNoSequenceWrap()
{
    FFixture F;const auto Binding=Stream(F);F.Envelope.State.Sequence=std::numeric_limits<N::U64>::max();
    const auto Parts=S::Fragment(F.Envelope,Binding);CHECK(Parts);
    S::FReceiver R(Binding,std::numeric_limits<N::U64>::max());
    for(const auto& P:*Parts)CHECK(R.Receive(20,P)==N::EResult::Accepted);
    CHECK(R.TakeReady() && R.Acknowledgement());CHECK(R.Receive(20,Parts->front())==N::EResult::Duplicate);
    auto P=Parts->front();P.Sequence=1;CHECK(R.Receive(20,P)==N::EResult::Exhausted);
}
static void FragmentAllocationRetry()
{
    FFixture F;const auto Binding=Stream(F);const auto Parts=S::Fragment(F.Envelope,Binding);CHECK(Parts);
    for(bool FinalChunk:{false,true})
    {
        S::FReceiver Probe(Binding,1);
        if(FinalChunk){CHECK(Probe.Receive(20,(*Parts)[0])==N::EResult::Accepted);CHECK(Probe.Receive(20,(*Parts)[1])==N::EResult::Accepted);}
        const auto Index=FinalChunk?2U:0U;const auto Start=Allocations;CHECK(Probe.Receive(20,(*Parts)[Index])==N::EResult::Accepted);
        const auto Count=Allocations-Start;CHECK(Count>0);
        for(std::size_t Budget=1;Budget<=Count;++Budget)
        {
            S::FReceiver R(Binding,1);
            if(FinalChunk){CHECK(R.Receive(20,(*Parts)[0])==N::EResult::Accepted);CHECK(R.Receive(20,(*Parts)[1])==N::EResult::Accepted);}
            bool Threw=false;FailAt=Allocations+Budget;
            try{(void)R.Receive(20,(*Parts)[Index]);}catch(const std::bad_alloc&){Threw=true;}
            FailAt=0;CHECK(Threw);CHECK(!R.TakeReady() && !R.Acknowledgement());
            // Exact retry works even when the failed attempt had already
            // retained the final chunk but had not produced a completed ACK.
            CHECK(R.Receive(20,(*Parts)[Index])==N::EResult::Accepted);
            if(!FinalChunk){CHECK(R.Receive(20,(*Parts)[1])==N::EResult::Accepted);CHECK(R.Receive(20,(*Parts)[2])==N::EResult::Accepted);}
            auto Ready=R.TakeReady();CHECK(Ready && S::Encode(*Ready)==F.Ledger.Exact && R.Acknowledgement());
        }
        std::printf("FRAGMENT_ALLOCATIONS phase%s positions%zu; exact retry preserves no partial publication\n",FinalChunk?"last":"first",Count);
    }
}
static void FragmentProgressionAndRetainedOrigin()
{
    FFixture F;const auto Binding=Stream(F);S::FReceiver Owner(Binding,1),Remote(Binding,1);
    for(N::U64 Sequence=1;Sequence<=3;++Sequence)
    {
        if(Sequence>1)
        {
            auto Pad=F.Envelope.State.Actors.back();
            F.Envelope.State=N::DeclaredGameFixture::Snapshot(Sequence);F.Envelope.State.Actors.push_back(std::move(Pad));
            F.Envelope.Address.Frame=F.Envelope.State.Frame;
            ++F.Envelope.Address.BeforeWorldPublication;++F.Envelope.Address.PublicationSerial;++F.Envelope.Address.WorldStateHash;
            auto& Context=F.Trusted.Inputs[0];Context.Frame=F.Envelope.Address.Frame;
            Context.BeforeWorldPublication=F.Envelope.Address.BeforeWorldPublication;Context.Processing.Before=F.Envelope.Decisions[0].Applied;
            auto Request=F.Envelope.Decisions[0].Original;Request.Frame=Context.Frame;++Request.Sequence;++Request.Input.SourceSequence;
            auto D=C::Propose(Request,Context);CHECK(D);
            D->Stage=C::EStage::Published;const auto& A=F.Envelope.Address;
            D->Publication={A.AuthorityPeer,A.AuthorityEpoch,A.PublicationSerial,A.WorldStateHash};F.Envelope.Decisions[0]=std::move(*D);
            F.Trusted.Address=A;F.Ledger.Exact=*S::Encode(F.Envelope);
            // Component fields are authored source fixtures regenerated for
            // this frame; this is not a native physical continuity proof.
        }
        const auto Parts=S::Fragment(F.Envelope,Binding);CHECK(Parts);
        for(auto* R:{&Owner,&Remote})
        {
            for(const auto& P:*Parts)CHECK(R->Receive(20,PacketCopy(P))==N::EResult::Accepted);
            auto Ready=R->TakeReady();CHECK(Ready);
            const auto Accepted=S::Accept(20,*Ready,F.Trusted,F.Ledger);CHECK(Accepted);
            CHECK(Accepted->Source().Decisions[0].Applied[0]==64+10*Sequence);
        }
    }
    std::printf("FRAGMENT_SOURCE_OWNER_REMOTE snapshots3; inputRequested255 applied74/84/94; world bodies and origin evidence authored SYNTHETIC; no native restore or local completion\n");
}
