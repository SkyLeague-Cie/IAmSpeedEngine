#pragma once
#include "StateAuthorityEnvelope.h"

namespace Speed::Netcode::StateAuthorityV2
{
inline constexpr std::size_t FragmentHeaderBytes=156,ChunkBytes=1000;
struct FStreamBinding
{
    U64 Session=0,StreamEpoch=0;
    C::FWorldIdentity World;
    U64 AuthorityPeer=0,AuthorityEpoch=0,AuthorityTimelineRevision=0;
    std::array<std::uint8_t,32> StateSchema{};
    bool Valid() const noexcept
    {bool Schema=false;for(auto B:StateSchema)Schema|=B!=0;return Session && StreamEpoch && World.Valid() && AuthorityPeer && AuthorityEpoch && AuthorityTimelineRevision && Schema;}
    bool Matches(const FAddress& A) const noexcept
    {return World==A.World && AuthorityPeer==A.AuthorityPeer && AuthorityEpoch==A.AuthorityEpoch && AuthorityTimelineRevision==A.AuthorityTimelineRevision && StateSchema==A.StateSchema;}
};
struct FFragment
{
    FAddress Address;
    U64 Session=0,StreamEpoch=0,Sequence=0; // Transport sequence, independent of PublicationSerial.
    std::uint32_t TotalBytes=0,Crc=0;
    std::uint16_t Index=0,Count=0;
    bool Ack=false;
    std::vector<std::uint8_t> Bytes;
};
inline bool ValidFragment(const FFragment& F) noexcept
{
    if(!F.Address.Valid() || !F.Session || !F.StreamEpoch || !F.Sequence || F.TotalBytes<HeaderBytes
        || F.TotalBytes>MaxEnvelopeBytes || !F.Count || F.Count!=(F.TotalBytes+ChunkBytes-1)/ChunkBytes)return false;
    if(F.Ack)return F.Index==0 && F.Bytes.empty();
    if(F.Index>=F.Count)return false;
    return F.Bytes.size()==std::min<std::size_t>(ChunkBytes,F.TotalBytes-std::size_t(F.Index)*ChunkBytes);
}
inline bool SameTransfer(const FFragment& A,const FFragment& B) noexcept
{
    return A.Address==B.Address && A.Session==B.Session && A.StreamEpoch==B.StreamEpoch && A.Sequence==B.Sequence
        && A.TotalBytes==B.TotalBytes && A.Crc==B.Crc && A.Count==B.Count;
}
inline std::optional<std::vector<std::uint8_t>> EncodeFragmentV2(const FFragment& F)
{
    if(!ValidFragment(F))return {};const auto& A=F.Address;
    std::vector<std::uint8_t> B;B.reserve(FragmentHeaderBytes+F.Bytes.size());
    Wire::Put(B,0x32465349,4);Wire::Put(B,2,2);Wire::Put(B,F.Ack?1:0,1);Wire::Put(B,0,1);
    for(auto V:{A.World.Id,A.World.Incarnation,A.AuthorityPeer,A.AuthorityEpoch,A.AuthorityTimelineRevision,
                A.BeforeWorldPublication,A.PublicationSerial,A.WorldStateHash,A.Frame,A.RegistryGeneration})Wire::Put(B,V,8);
    B.insert(B.end(),A.StateSchema.begin(),A.StateSchema.end());
    for(auto V:{F.Session,F.StreamEpoch,F.Sequence})Wire::Put(B,V,8);
    Wire::Put(B,F.TotalBytes,4);Wire::Put(B,F.Crc,4);Wire::Put(B,F.Index,2);Wire::Put(B,F.Count,2);
    if(B.size()!=FragmentHeaderBytes)return {};B.insert(B.end(),F.Bytes.begin(),F.Bytes.end());return B;
}
inline std::optional<FFragment> DecodeFragmentV2(const std::vector<std::uint8_t>& B)
{
    if(B.size()<FragmentHeaderBytes || B.size()>FragmentHeaderBytes+ChunkBytes)return {};
    Wire::FReader R{B.data(),B.size()};if(R.Get(4)!=0x32465349 || R.Get(2)!=2)return {};
    const auto Ack=R.Get(1);if(Ack>1 || R.Get(1))return {};
    FFragment F;F.Ack=Ack!=0;auto& A=F.Address;
    A.World={R.Get(8),R.Get(8)};A.AuthorityPeer=R.Get(8);A.AuthorityEpoch=R.Get(8);A.AuthorityTimelineRevision=R.Get(8);
    A.BeforeWorldPublication=R.Get(8);A.PublicationSerial=R.Get(8);A.WorldStateHash=R.Get(8);A.Frame=R.Get(8);A.RegistryGeneration=R.Get(8);
    for(auto& V:A.StateSchema)V=std::uint8_t(R.Get(1));
    F.Session=R.Get(8);F.StreamEpoch=R.Get(8);F.Sequence=R.Get(8);
    F.TotalBytes=std::uint32_t(R.Get(4));F.Crc=std::uint32_t(R.Get(4));F.Index=std::uint16_t(R.Get(2));F.Count=std::uint16_t(R.Get(2));
    if(!R.Good)return {};F.Bytes.assign(B.begin()+R.At,B.end());if(!ValidFragment(F))return {};return F;
}
inline std::optional<std::vector<FFragment>> Fragment(const FEnvelope& E,const FStreamBinding& Binding)
{
    if(!Binding.Valid() || !Binding.Matches(E.Address))return {};const auto Bytes=Encode(E);if(!Bytes)return {};
    const auto Count=(Bytes->size()+ChunkBytes-1)/ChunkBytes;const auto Crc=StateCrc(*Bytes);
    std::vector<FFragment> Result;Result.reserve(Count);
    for(std::size_t I=0;I<Count;++I)
    {
        FFragment F;F.Address=E.Address;F.Session=Binding.Session;F.StreamEpoch=Binding.StreamEpoch;F.Sequence=E.State.Sequence;
        F.TotalBytes=std::uint32_t(Bytes->size());F.Crc=Crc;F.Index=std::uint16_t(I);F.Count=std::uint16_t(Count);
        const auto Begin=I*ChunkBytes,End=std::min(Begin+ChunkBytes,Bytes->size());
        F.Bytes.assign(Bytes->begin()+Begin,Bytes->begin()+End);Result.push_back(std::move(F));
    }
    return Result;
}
// Worker-only bounded stop-and-wait framing. A stream is constructed only from
// an explicitly admitted connection/world/authority branch and next sequence.
// A larger epoch/branch/frame is never an implicit reset or admission.
class FReceiver final
{
public:
    FReceiver(FStreamBinding B,U64 FirstSequence):Binding(std::move(B)),Next(FirstSequence?std::optional<U64>(FirstSequence):std::nullopt){}
    EResult Receive(U64 TrustedSender,const FFragment& F)
    {
        if(!Binding.Valid() || !ValidFragment(F) || F.Ack)return EResult::Invalid;
        if(TrustedSender!=Binding.AuthorityPeer)return EResult::WrongPeer;
        if(F.Session!=Binding.Session || F.StreamEpoch!=Binding.StreamEpoch || !Binding.Matches(F.Address))return EResult::WrongStream;
        if(Last && F.Sequence==Last->Sequence)
        {
            if(!SameTransfer(F,*Last))return EResult::Conflict;
            const auto Begin=std::size_t(F.Index)*ChunkBytes;
            return Retained && Begin<=Retained->size() && F.Bytes.size()<=Retained->size()-Begin
                && std::equal(F.Bytes.begin(),F.Bytes.end(),Retained->begin()+Begin)?EResult::Duplicate:EResult::Conflict;
        }
        if(!Next)return EResult::Exhausted;
        if(F.Sequence<*Next)return EResult::Stale;
        if(F.Sequence>*Next || Ready)return EResult::TooFar;
        if(!Pending)
        {
            FFragment NewPending=F;NewPending.Bytes.clear();
            std::vector<std::optional<std::vector<std::uint8_t>>> NewParts(F.Count);
            Pending=std::move(NewPending);Parts=std::move(NewParts);Received=0;
        }
        else if(!SameTransfer(F,*Pending))return EResult::Conflict;
        auto& Part=Parts[F.Index];
        if(Part)
        {
            if(*Part!=F.Bytes)return EResult::Conflict;
            if(Received!=Parts.size())return EResult::Duplicate;
            // A previous allocation failure after the last chunk must remain
            // retryable; duplicated complete input is not a completed receipt.
        }
        else {Part=F.Bytes;++Received;}
        if(Received!=Parts.size())return EResult::Accepted;
        std::vector<std::uint8_t> Bytes;Bytes.reserve(F.TotalBytes);
        for(const auto& P:Parts)Bytes.insert(Bytes.end(),P->begin(),P->end());
        if(Bytes.size()!=F.TotalBytes || StateCrc(Bytes)!=F.Crc){ClearPending();return EResult::Invalid;}
        auto Envelope=Decode(Bytes);
        if(!Envelope || !(Envelope->Address==F.Address) || Envelope->State.Sequence!=F.Sequence)
        {ClearPending();return EResult::Invalid;}
        // Construct ALL fallible retained objects before transport publication.
        auto Storage=std::make_shared<const std::vector<std::uint8_t>>(std::move(Bytes));
        std::optional<FEnvelope> NewReady(std::move(*Envelope));FFragment NewLast=*Pending;NewLast.Bytes.clear();
        static_assert(std::is_nothrow_move_assignable_v<decltype(Ready)> && std::is_nothrow_move_assignable_v<decltype(Last)>);
        Retained=std::move(Storage);Ready=std::move(NewReady);Last=std::move(NewLast);
        Next=F.Sequence==std::numeric_limits<U64>::max()?std::nullopt:std::optional<U64>(F.Sequence+1);
        ClearPending();return EResult::Accepted;
    }
    std::optional<FEnvelope> TakeReady(){auto Result=std::move(Ready);Ready.reset();return Result;}
    std::optional<FFragment> Acknowledgement() const
    {if(!Last)return {};auto Result=*Last;Result.Ack=true;Result.Index=0;Result.Bytes.clear();return Result;}
private:
    void ClearPending() noexcept {Pending.reset();Parts.clear();Received=0;}
    const FStreamBinding Binding;
    std::optional<U64> Next;
    std::optional<FFragment> Pending,Last;
    std::vector<std::optional<std::vector<std::uint8_t>>> Parts;
    std::size_t Received=0;
    std::shared_ptr<const std::vector<std::uint8_t>> Retained;
    std::optional<FEnvelope> Ready;
};
// This ACK proves reassembly only. Before native use, TakeReady still requires
// Accept with actual origin evidence, then the separate local world transaction.
}
