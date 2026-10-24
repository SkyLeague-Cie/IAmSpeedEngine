#pragma once
#include "NetInputTransport.h"
#include "IAmSpeed/Input/InputFrameV2.h"

namespace Speed::Netcode
{
// Values-only conversion against pinned source APIs. This deliberately does
// NOT register an ExactRemote owner or bypass current simulation admission.
inline std::optional<Input::V2::FInputFrame> ToV2(const FPacket& P,
    const std::shared_ptr<const Input::V2::FInputActionContract>& Contract)
{
    if (!Contract || !Valid(P) || P.Kind == EKind::Ack) return {};
    Input::V2::FInputFrameData D;
    D.ConsumptionFrame = P.Frame; D.SourceSequence = P.Input.SourceSequence;
    D.StreamEpoch.Value = P.Epoch; D.DeviceGeneration.Value = P.Input.DeviceGeneration;
    D.Producer = {static_cast<Input::EProducerKind>(P.Input.ProducerKind), P.Input.ProducerId};
    D.ActiveMask = P.Input.ActiveMask; D.Values = P.Input.Values; D.Reset = P.Input.Reset;
    for (const auto& E : P.Input.Edges)
        D.Transitions.push_back({E.Action, static_cast<Input::V2::ETransition>(E.State), E.Value,
            {E.SourceSequence, E.Ordinal}});
    Input::V2::FInputFrame Result(Contract, std::move(D));
    if (!Result.IsValidFor(*Contract)) return {};
    return Result;
}
inline std::optional<FPacket> FromV2(const Input::V2::FInputFrame& Frame,
    const FBinding& Binding, EKind Kind, U64 PacketSequence)
{
    const auto& D = Frame.GetData();
    if (!Binding.Valid() || !Frame.GetContract() || !Frame.IsValidFor(*Frame.GetContract())
        || Kind == EKind::Ack || D.StreamEpoch.Value != Binding.Epoch
        || D.Producer.Id != Binding.SourceProducer || std::uint8_t(D.Producer.Kind) != Binding.SourceKind) return {};
    FPacket P; P.Session = Binding.Session; P.Epoch = Binding.Epoch;
    P.Actor = Binding.Actor; P.Incarnation = Binding.Incarnation; P.Contract = Binding.Contract;
    P.Kind = Kind; P.Sequence = PacketSequence; P.Frame = D.ConsumptionFrame;
    P.Input.SourceSequence = D.SourceSequence; P.Input.DeviceGeneration = D.DeviceGeneration.Value;
    P.Input.ProducerId = D.Producer.Id; P.Input.ProducerKind = std::uint8_t(D.Producer.Kind);
    P.Input.Values = D.Values; P.Input.ActiveMask = D.ActiveMask; P.Input.Reset = D.Reset;
    for (const auto& E : D.Transitions)
        P.Input.Edges.push_back({E.Action, std::uint8_t(E.State), E.ValueAtTransition,
            E.Order.Sequence, E.Order.WithinSequence});
    if (!Valid(P)) return {};
    return P;
}
}
