#pragma once
#include "NetStateTransport.h"
#include "CanonicalActorState.h"

namespace Speed::Netcode::StateFixture
{
inline FStateBinding Binding()
{ FStateBinding B; B.Session = 11; B.Epoch = 7; B.ServerPeer = 1; B.Schema[0] = 84; return B; }
inline FWorldState Snapshot(U64 Sequence)
{
    FWorldState S; S.Sequence = Sequence; S.Frame = 30 + Sequence; S.RegistryGeneration = Sequence;
    S.WorldPayload = {1, 2, 3, 4};
    // Real field schema, synthetic values/mechanics; not production serializers.
    FCanonicalActorState Car; Car.Kinematic[0] = 123.125 + double(Sequence);
    Car.Kinematic[3] = -8000; Car.Kinematic[12] = 1; Car.Kinematic[13] = -0.25;
    Car.MechanicalSchema = 1; Car.GameplaySchema = 1; Car.Mechanical.assign(1800, std::uint8_t(Sequence));
    Car.Gameplay = {1, 0, 4};
    auto Ball = Car; Ball.Mechanical.clear(); Ball.Gameplay = {0}; Ball.Kinematic[0] = -321.5; Ball.Kinematic[3] = 4200.125;
    FCanonicalActorState Pad; Pad.HasBody = false; Pad.MechanicalSchema = 1; Pad.GameplaySchema = 1; Pad.Gameplay = {1, 0, 42};
    S.Actors = {{10, 1, 1, 1, ELifecycle::Alive, *EncodeCanonicalActor(Car)},
        {20, 1, 2, 1, ELifecycle::Alive, *EncodeCanonicalActor(Ball)},
        {30, 1, 3, 1, Sequence == 1 ? ELifecycle::Alive : ELifecycle::Destroyed,
            Sequence == 1 ? *EncodeCanonicalActor(Pad) : std::vector<std::uint8_t>{}}};
    if (Sequence == 3) { Pad.Gameplay = {0, 1, 99}; S.Actors.push_back({30, 2, 3, 1, ELifecycle::Alive, *EncodeCanonicalActor(Pad)}); }
    return S;
}
inline bool ValidateActor(const FActorState& A)
{ return A.Type >= 1 && A.Type <= 3 && A.SchemaVersion == 1
    && (A.Lifecycle == ELifecycle::Destroyed || bool(DecodeCanonicalActor(A.Payload))); }
}
