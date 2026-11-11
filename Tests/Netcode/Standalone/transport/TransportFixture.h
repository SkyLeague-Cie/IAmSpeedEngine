#pragma once
#include "NetInputV2Bridge.h"

namespace Speed::Netcode::Fixture
{
inline FBinding Binding()
{
    FBinding B; B.Session = 11; B.Epoch = 7; B.Actor = 23; B.Incarnation = 2;
    B.OwnerPeer = 101; B.ServerPeer = 1; B.SourceProducer = 99; B.Contract[0] = 42;
    return B;
}
inline std::shared_ptr<const Input::V2::FInputActionContract> Contract()
{
    static const auto Value = []
    {
        using namespace Input::V2;
        FInputActionContractDescription D; D.Revision.Value = 1;
        D.Actions = FInputActionContract::BaseActions();
        FActionDefinition Jump; Jump.Id = 3; Jump.Owner = "Fixture"; Jump.Name = "Jump";
        Jump.Wiring = EActionWiring::Wired; D.Actions.push_back(Jump);
        D.Physical = {{0, EPhysicalDestination::Throttle}, {1, EPhysicalDestination::Brake}, {2, EPhysicalDestination::Steering}};
        return FInputActionContract::Create(std::move(D));
    }();
    return Value;
}
inline FPacket Packet(U64 N, EKind Kind = EKind::OwnerInput)
{
    const auto B = Binding(); FPacket P; P.Kind = Kind; P.Session = B.Session; P.Epoch = B.Epoch;
    P.Actor = B.Actor; P.Incarnation = B.Incarnation; P.Contract = B.Contract;
    P.Frame = N; P.Sequence = N + 1; P.Input.SourceSequence = N + 1;
    P.Input.DeviceGeneration = 1; P.Input.ProducerId = B.SourceProducer;
    P.Input.Reset = N == 0; P.Input.ActiveMask = N ? 7 : 0;
    if (N)
    {
        P.Input.Values[0] = std::int16_t((N * 17) % 256);
        P.Input.Values[1] = std::int16_t((N * 3) % 256);
        P.Input.Values[2] = std::int16_t(int(N % 255) - 127);
        if (N == 1) for (std::uint8_t A = 0; A < 3; ++A)
            P.Input.Edges.push_back({A, 0, P.Input.Values[A], N + 1, A});
        if (N % 5 == 0) P.Input.Edges = {{3, 0, 1, N + 1, 0}, {3, 1, 0, N + 1, 1}};
    }
    return P;
}
inline bool ValidateGame(const FFrame& I)
{
    auto P = Packet(0); P.Input = I;
    return bool(ToV2(P, Contract()));
}
}
