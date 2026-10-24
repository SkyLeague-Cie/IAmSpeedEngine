#pragma once
#include "../transport/CanonicalActorState.h"
#include <type_traits>

namespace Speed::Netcode::Pads
{
// Exactly the two declared FPadState scalars, not a complete pad checkpoint.
// Static sensor/transform/config, winner/dependencies and lifecycle remain host
// responsibilities. This codec does not authorize any native network correction.
struct FDeclaredState
{
    std::uint16_t FramesBeforeActive = 0, StateSerial = 0;
    bool IsActive() const { return FramesBeforeActive == 0; }
};
constexpr std::uint32_t DeclaredStateSchema = 0x504401;
constexpr bool CompletePadRestoreQualified = false;
inline std::vector<std::uint8_t> Encode(const FDeclaredState& S)
{
    std::vector<std::uint8_t> B;
    Wire::Put(B, 0x3150444E, 4); Wire::Put(B, 1, 2); Wire::Put(B, 0, 2);
    Wire::Put(B, S.FramesBeforeActive, 2); Wire::Put(B, S.StateSerial, 2);
    return B;
}
inline std::optional<FDeclaredState> Decode(const std::vector<std::uint8_t>& B)
{
    if (B.size() != 12) return {};
    Wire::FReader R{B.data(), B.size()};
    if (R.Get(4) != 0x3150444E || R.Get(2) != 1 || R.Get(2)) return {};
    FDeclaredState S;
    S.FramesBeforeActive = std::uint16_t(R.Get(2)); S.StateSerial = std::uint16_t(R.Get(2));
    return R.Good && R.At == B.size() ? std::optional<FDeclaredState>(S) : std::nullopt;
}
template<class State> inline FDeclaredState Capture(const State& S)
{
    using Counter = std::remove_cv_t<std::remove_reference_t<decltype(S.nbFramesBeforeActive)>>;
    using Serial = std::remove_cv_t<std::remove_reference_t<decltype(S.StateSerial)>>;
    static_assert(std::is_unsigned_v<Counter> && sizeof(Counter) == 2);
    static_assert(std::is_unsigned_v<Serial> && sizeof(Serial) == 2);
    return {S.nbFramesBeforeActive, S.StateSerial};
}
// StateSerial is a uint16 semantic transition counter and may wrap. It is not a
// network freshness/epoch key. Only the enclosing authoritative frame/lifecycle
// transaction can choose a newer checkpoint. No interpolation/countdown tick here.
}
