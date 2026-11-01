#pragma once
#include "../transport/NetInputTransport.h"
#include <map>

namespace Speed::Netcode::Prediction
{
// Explicit speculative request, NOT a Device frame, authority decision or wire
// packet. The canonical input processor must apply the real policy once per
// predicted frame. In particular, do not hold a previous filtered Applied value
// and feed that value through the slew filter again.
struct FSpeculativeRequest
{
    U64 Frame=0, EvidenceFrame=0;
    std::array<std::int16_t,ActionCount> Requested{};
    std::uint32_t ActiveMask=0;
    // No synthetic Started/Completed events are invented by hold-last-request.
};
enum class EPredict { Predicted, Confirmed, NoEarlierAuthority, HorizonExceeded, OutOfOrder, CapacityExceeded };
struct FPrediction
{
    EPredict Status=EPredict::NoEarlierAuthority;
    FSpeculativeRequest Request;
    std::optional<FPacket> ConfirmedPacket;
};
struct FConfirmation
{
    EResult Status=EResult::Invalid;
    std::optional<U64> EarliestInvalidatedFrame;
};
class FRemoteRequestHistory
{
    FBinding Binding;
    U64 Horizon=0;
    std::size_t Capacity=0;
    std::map<U64,FPacket> Confirmed;
    std::map<U64,FSpeculativeRequest> Speculated;
    std::optional<U64> LastPredicted;
    static bool Same(const FPacket& A,const FPacket& B)
    {return Encode(A)==Encode(B);}
    static bool MatchesPrediction(const FSpeculativeRequest& A,const FPacket& B)
    {
        // Hold prediction contains no raw edges/reset. Even an edge whose final
        // held value is unchanged requires exact historical reconciliation.
        return A.Requested==B.Input.Values && A.ActiveMask==B.Input.ActiveMask
            && B.Input.Edges.empty() && !B.Input.Reset;
    }
public:
    FRemoteRequestHistory(FBinding In,U64 MaximumPredictionFrames,std::size_t MaximumRetainedFrames)
        :Binding(In),Horizon(MaximumPredictionFrames),Capacity(MaximumRetainedFrames) {}
    FConfirmation Confirm(U64 AuthenticatedSender,const FPacket& Packet)
    {
        FConfirmation Out;
        if (!Binding.Valid() || !Horizon || !Capacity || Capacity>4096 || !Valid(Packet)
            || Packet.Kind!=EKind::AuthorityInput || !Binding.Matches(Packet)
            || Packet.Input.ProducerId!=Binding.SourceProducer || Packet.Input.ProducerKind!=Binding.SourceKind) return Out;
        if (AuthenticatedSender!=Binding.ServerPeer) {Out.Status=EResult::WrongPeer;return Out;}
        auto Existing=Confirmed.find(Packet.Frame);
        if (Existing!=Confirmed.end())
        {Out.Status=Same(Existing->second,Packet)?EResult::Duplicate:EResult::Conflict;return Out;}
        if (Confirmed.size()+Speculated.size()>=Capacity && !Speculated.count(Packet.Frame))
        {Out.Status=EResult::Full;return Out;}
        // The caller decides when old history is no longer needed. Nothing is
        // silently evicted while an authoritative correction may still need it.
        Confirmed.emplace(Packet.Frame,Packet);Out.Status=EResult::Accepted;
        for (const auto& Entry:Speculated)
        {
            if (Entry.first<Packet.Frame) continue;
            auto Anchor=Confirmed.upper_bound(Entry.first);
            if (Anchor==Confirmed.begin()) continue;
            --Anchor;
            const bool Different=Anchor->first==Entry.first
                ? !MatchesPrediction(Entry.second,Anchor->second)
                : Entry.second.Requested!=Anchor->second.Input.Values || Entry.second.ActiveMask!=Anchor->second.Input.ActiveMask;
            if (Different) {Out.EarliestInvalidatedFrame=Entry.first;break;}
        }
        Speculated.erase(Packet.Frame);
        return Out;
    }
    FPrediction Predict(U64 Frame)
    {
        FPrediction Out;
        if (LastPredicted && (*LastPredicted==std::numeric_limits<U64>::max() || Frame!=*LastPredicted+1))
        {Out.Status=EPredict::OutOfOrder;return Out;}
        auto Exact=Confirmed.find(Frame);
        if (Exact!=Confirmed.end()) {Out.Status=EPredict::Confirmed;Out.ConfirmedPacket=Exact->second;LastPredicted=Frame;return Out;}
        auto Anchor=Confirmed.upper_bound(Frame);
        if (Anchor==Confirmed.begin()) return Out;
        --Anchor;
        if (Frame-Anchor->first>Horizon) {Out.Status=EPredict::HorizonExceeded;return Out;}
        if (Confirmed.size()+Speculated.size()>=Capacity) {Out.Status=EPredict::CapacityExceeded;return Out;}
        Out.Request={Frame,Anchor->first,Anchor->second.Input.Values,Anchor->second.Input.ActiveMask};
        Out.Status=EPredict::Predicted;Speculated.emplace(Frame,Out.Request);LastPredicted=Frame;return Out;
    }
    // Only after the caller observes successful native restore/resimulation may
    // it discard speculative future requests and produce them again. This does
    // not acknowledge native restoration and does not retire confirmed history.
    void DiscardSpeculationFrom(U64 Frame)
    {
        Speculated.erase(Speculated.lower_bound(Frame),Speculated.end());
        LastPredicted=Speculated.empty()?std::optional<U64>{}:std::optional<U64>{Speculated.rbegin()->first};
    }
    std::size_t Retained() const noexcept {return Confirmed.size()+Speculated.size();}
};
}
