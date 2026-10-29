#pragma once
#include "InputProducerV2.h"
#include <algorithm>

namespace Speed::Input::V2
{
struct FInputProcessingPolicy
{
    // Zero means direct. Nonzero means maximum quantized change in this step.
    std::array<std::uint16_t, ActionCount> Step{};
};
// Shared admission validation; filtering belongs only to the authoritative owner.
inline bool ValidInputProcessingPolicy(EProducerContract Kind, const FInputProcessingPolicy& P,
    const FInputActionContract& Contract, EExactRemoteAdmission Admission) noexcept
{
    for (std::size_t I = 0; I < ActionCount; ++I)
    {
        if (!P.Step[I]) continue;
        const auto* A = Contract.Find(static_cast<FActionId>(I));
        if (!UsesAuthoritativeDeviceProcessing(Kind, Admission) || !A || A->Wiring != EActionWiring::Wired
            || A->Type != EActionType::Axis1D || P.Step[I] > static_cast<std::uint16_t>(2 * A->Quantization)) return false;
    }
    return true;
}
// Pure calculation shared by the authoritative owner and immutable witness checks.
// Callers must validate the complete policy against the bound contract first.
// This function acquires no input, advances no history and publishes no credit.
inline FActionValues ProcessAuthoritativeInputValues(const FInputProcessingPolicy& Policy,
    const FActionValues& Requested, const FActionValues& Before, bool Reset) noexcept
{
    auto Result = Requested;
    if (Reset) return Result;
    for (std::size_t I = 0; I < ActionCount; ++I)
    {
        const int Step = Policy.Step[I];
        if (Step) Result[I] = static_cast<std::int16_t>(int(Before[I]) + std::clamp(int(Requested[I]) - int(Before[I]), -Step, Step));
    }
    return Result;
}
}
