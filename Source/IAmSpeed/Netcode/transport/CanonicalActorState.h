#pragma once
#include "NetStateTransport.h"
#include <cmath>
#include <cstring>

namespace Speed::Netcode
{
// Field order follows pinned FKinematicState / SimulationWorld schema2:
// position3, velocity3, acceleration3, quaternion XYZW4, angular velocity3,
// angular acceleration3. Explicit binary64 LE preserves each source bit.
// No new quantization, rounding, interpolation, tolerance or normalization.
struct FCanonicalActorState
{
    bool HasBody = true, Frozen = false;
    std::array<double, 19> Kinematic{};
    std::uint32_t MechanicalSchema = 0, GameplaySchema = 0;
    std::vector<std::uint8_t> Mechanical, Gameplay;
};
inline bool ValidCanonicalActor(const FCanonicalActorState& S)
{
    if (!S.MechanicalSchema || !S.GameplaySchema || S.Mechanical.size() > 12000 || S.Gameplay.size() > 4000) return false;
    if (S.HasBody)
    {
        for (auto V : S.Kinematic) if (!std::isfinite(V)) return false;
        if (S.Kinematic[9] == 0 && S.Kinematic[10] == 0 && S.Kinematic[11] == 0 && S.Kinematic[12] == 0) return false;
    }
    else if (S.Frozen || S.Gameplay.empty()) return false;
    return true;
}
inline std::optional<std::vector<std::uint8_t>> EncodeCanonicalActor(const FCanonicalActorState& S)
{
    static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559);
    if (!ValidCanonicalActor(S)) return {};
    std::vector<std::uint8_t> B;
    Wire::Put(B, 0x31415349, 4); Wire::Put(B, 1, 2);
    Wire::Put(B, (S.HasBody ? 1 : 0) | (S.Frozen ? 2 : 0), 1); Wire::Put(B, 0, 1);
    Wire::Put(B, S.MechanicalSchema, 4); Wire::Put(B, S.GameplaySchema, 4);
    Wire::Put(B, S.Mechanical.size(), 4); Wire::Put(B, S.Gameplay.size(), 4);
    if (S.HasBody) for (double V : S.Kinematic)
    { U64 Bits = 0; std::memcpy(&Bits, &V, sizeof(Bits)); Wire::Put(B, Bits, 8); }
    B.insert(B.end(), S.Mechanical.begin(), S.Mechanical.end()); B.insert(B.end(), S.Gameplay.begin(), S.Gameplay.end());
    return B;
}
inline std::optional<FCanonicalActorState> DecodeCanonicalActor(const std::vector<std::uint8_t>& B)
{
    if (B.size() < 24 || B.size() > MaxActorState) return {};
    Wire::FReader R{B.data(), B.size()};
    if (R.Get(4) != 0x31415349 || R.Get(2) != 1) return {};
    const auto Flags = R.Get(1); if (Flags > 3 || R.Get(1)) return {};
    FCanonicalActorState S; S.HasBody = (Flags & 1) != 0; S.Frozen = (Flags & 2) != 0;
    S.MechanicalSchema = std::uint32_t(R.Get(4)); S.GameplaySchema = std::uint32_t(R.Get(4));
    const auto MechanicalSize = R.Get(4), GameplaySize = R.Get(4);
    const auto Expected = 24 + (S.HasBody ? 152 : 0) + MechanicalSize + GameplaySize;
    if (MechanicalSize > 12000 || GameplaySize > 4000 || Expected != B.size()) return {};
    if (S.HasBody) for (auto& V : S.Kinematic)
    { const auto Bits = R.Get(8); std::memcpy(&V, &Bits, sizeof(V)); }
    if (!R.Good) return {};
    S.Mechanical.assign(B.begin() + R.At, B.begin() + R.At + std::size_t(MechanicalSize)); R.At += std::size_t(MechanicalSize);
    S.Gameplay.assign(B.begin() + R.At, B.end());
    if (!ValidCanonicalActor(S)) return {};
    return S;
}
}
