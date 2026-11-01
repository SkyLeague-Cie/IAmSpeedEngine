#pragma once
#include "NetInputTransport.h"
#include <algorithm>
#include <memory>

namespace Speed::Netcode
{
// Finite retention of original-server AuthorityInput records for EVERY actor,
// including the local owner. A transport retention ACK is not a record here.
// The caller must authenticate the original channel and verify that the server
// emitted each record only after its real canonical consumption. This values
// journal does not authenticate a source, certify applied/hidden physical state,
// admit a checkpoint/CanMove transition or grant native lifetime exclusion.
class FAuthorityInputJournal final
{
    struct FActor
    {
        FBinding Binding;
        std::vector<std::shared_ptr<const FPacket>> Frames;
        U64 Contiguous=0;
    };
    std::vector<FActor> Actors;
    U64 Maximum=0,Server=0;
    FAuthorityInputJournal()=default;
public:
    static std::optional<FAuthorityInputJournal> Create(std::vector<FBinding> Roster,U64 MaximumFrames)
    {
        if(Roster.empty()||Roster.size()>16||!MaximumFrames||MaximumFrames>4096)return {};
        std::sort(Roster.begin(),Roster.end(),[](const auto& A,const auto& B){return A.Actor<B.Actor;});
        FAuthorityInputJournal J;J.Maximum=MaximumFrames;J.Server=Roster.front().ServerPeer;
        U64 Previous=0;
        for(const auto& B:Roster)
        {
            if(!B.Valid()||B.Actor==Previous||B.ServerPeer!=J.Server
                ||B.Session!=Roster.front().Session||B.Epoch!=Roster.front().Epoch)return {};
            Previous=B.Actor;J.Actors.push_back({B,std::vector<std::shared_ptr<const FPacket>>(size_t(MaximumFrames)),0});
        }
        return J;
    }
    EResult Retain(U64 OriginalAuthenticatedPeer,const FPacket& Packet)
    {
        if(OriginalAuthenticatedPeer!=Server)return EResult::WrongPeer;
        if(!Valid(Packet)||Packet.Kind!=EKind::AuthorityInput)return EResult::Invalid;
        for(auto& A:Actors)if(A.Binding.Actor==Packet.Actor)
        {
            if(!A.Binding.Matches(Packet)||Packet.Input.ProducerId!=A.Binding.SourceProducer
                ||Packet.Input.ProducerKind!=A.Binding.SourceKind)return EResult::WrongStream;
            if(Packet.Frame>=Maximum)return EResult::TooFar;
            auto& Slot=A.Frames[size_t(Packet.Frame)];
            if(Slot)return Encode(*Slot)==Encode(Packet)?EResult::Duplicate:EResult::Conflict;
            Slot=std::make_shared<const FPacket>(Packet);
            while(A.Contiguous<Maximum&&A.Frames[size_t(A.Contiguous)])++A.Contiguous;
            return EResult::Accepted;
        }
        return EResult::WrongStream;
    }
    U64 CommonContiguousCount() const noexcept
    {
        U64 Count=Maximum;for(const auto& A:Actors)Count=std::min(Count,A.Contiguous);return Count;
    }
    U64 ContiguousCount(U64 Actor) const noexcept
    {for(const auto& A:Actors)if(A.Binding.Actor==Actor)return A.Contiguous;return 0;}
    std::shared_ptr<const FPacket> Read(U64 Actor,U64 Frame) const noexcept
    {
        if(Frame>=Maximum)return {};
        for(const auto& A:Actors)if(A.Binding.Actor==Actor)return A.Frames[size_t(Frame)];
        return {};
    }
};
}
