#pragma once
#include "CommonComponentState.h"
#include "IAmSpeed/World/Simulation/SimulationSleepState.h"
#include "IAmSpeed/Base/KinematicQuantizationCache.h"
#include <type_traits>

namespace Speed
{
// Real derived frozen classes, with friend access added and algorithms unchanged.
// No live getter that mutates state, no Reset, canonicalization or quantizer call.
struct FCommonCacheValueAccess
{
    struct FPreparedCaches
    {
        FSimulationSleepState Sleep;
        FIdentityKinematicQuantizationCache Quantization;
        void CopyTo(FSimulationSleepState& ToSleep, FIdentityKinematicQuantizationCache& ToQuantization) const noexcept
        {
            static_assert(std::is_nothrow_copy_assignable_v<FSimulationSleepState>);
            static_assert(std::is_nothrow_copy_assignable_v<FIdentityKinematicQuantizationCache>);
            ToSleep = Sleep; ToQuantization = Quantization;
        }
    };
    static CommonState::FSleepValues Capture(const FSimulationSleepState& S) noexcept
    {
        return {{S.Location.X, S.Location.Y, S.Location.Z},
            {S.Rotation.X, S.Rotation.Y, S.Rotation.Z, S.Rotation.W}, S.bHasCompletedPose};
    }
    static CommonState::FQuantizationValues Capture(const FIdentityKinematicQuantizationCache& Q) noexcept
    {
        const auto& I = Q.NoOpInput;
        return {{I.Location.X, I.Location.Y, I.Location.Z, I.Velocity.X, I.Velocity.Y, I.Velocity.Z,
            I.AngularVelocity.X, I.AngularVelocity.Y, I.AngularVelocity.Z,
            I.Rotation.X, I.Rotation.Y, I.Rotation.Z, I.Rotation.W,
            I.PreviousRotation.X, I.PreviousRotation.Y, I.PreviousRotation.Z, I.PreviousRotation.W}, Q.bValid};
    }
    static std::optional<FPreparedCaches> Prepare(const CommonState::FSleepValues& S,
        const CommonState::FQuantizationValues& Q)
    {
        if (!CommonState::ValidCaches(S, Q)) return {};
        FPreparedCaches P;
        P.Sleep.Location = FVector(S.Location[0], S.Location[1], S.Location[2]);
        P.Sleep.Rotation = FQuat(S.Rotation[0], S.Rotation[1], S.Rotation[2], S.Rotation[3]);
        P.Sleep.bHasCompletedPose = S.HasCompletedPose;
        auto& I = P.Quantization.NoOpInput; const auto& V = Q.NoOpInput;
        I.Location = FVector(V[0], V[1], V[2]); I.Velocity = FVector(V[3], V[4], V[5]);
        I.AngularVelocity = FVector(V[6], V[7], V[8]); I.Rotation = FQuat(V[9], V[10], V[11], V[12]);
        I.PreviousRotation = FQuat(V[13], V[14], V[15], V[16]); P.Quantization.bValid = Q.Valid;
        return P;
    }
};
}
