#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <type_traits>
#include <utility>

namespace Speed::Netcode
{
// A value observed at one frame. Packet authentication and edge delivery
// belong to the caller. A predicted value carries no packet or device identity.
struct FNetworkInputState
{
    static constexpr std::size_t ActionCount = 32;
    std::uint64_t Frame = 0;
    std::array<std::int16_t, ActionCount> Values{};
    std::uint32_t ActiveMask = 0;
    bool Reset = false;
};

struct FNetworkInputPredictionContext
{
    std::uint64_t Frame;
    const FNetworkInputState& Earlier;
    const FNetworkInputState* Later = nullptr;
    std::uint32_t ContinuousAxes = 0;
};

struct FNetworkInputPredictionPolicy
{
    static constexpr std::size_t Window = 64;
    using FFunction = FNetworkInputState (*)(const FNetworkInputPredictionContext&) noexcept;
    FFunction Predict = nullptr;

    static FNetworkInputState Hold(const FNetworkInputPredictionContext& Context) noexcept
    {
        return {Context.Frame, Context.Earlier.Values, Context.Earlier.ActiveMask};
    }

    // A bracket is optional. Live prediction can always proceed with Hold.
    // No digital transition or interpolation over a reset.
    static FNetworkInputState InterpolateAxes(const FNetworkInputPredictionContext& Context) noexcept
    {
        auto Result = Hold(Context);
        const auto* Later = Context.Later;
        if (!Later || Context.Earlier.Reset || Later->Reset
            || Context.Frame <= Context.Earlier.Frame || Context.Frame >= Later->Frame
            || Later->Frame - Context.Earlier.Frame >= Window)
            return Result;
        const auto Span = static_cast<std::int32_t>(Later->Frame - Context.Earlier.Frame);
        const auto Offset = static_cast<std::int32_t>(Context.Frame - Context.Earlier.Frame);
        for (std::size_t Index = 0; Index < FNetworkInputState::ActionCount; ++Index)
        {
            const auto Bit = std::uint32_t{1} << Index;
            if (!(Context.ContinuousAxes & Bit)) continue;
            const auto Start = std::int32_t(Context.Earlier.Values[Index]);
            const auto End = std::int32_t(Later->Values[Index]);
            const auto Weighted = Start * (Span - Offset) + End * Offset;
            Result.Values[Index] = static_cast<std::int16_t>(
                (Weighted + (Weighted < 0 ? -Span / 2 : Span / 2)) / Span);
            if (Result.Values[Index] || ((Context.Earlier.ActiveMask & Later->ActiveMask) & Bit))
                Result.ActiveMask |= Bit;
            else Result.ActiveMask &= ~Bit;
        }
        return Result;
    }

    FNetworkInputState Evaluate(const FNetworkInputPredictionContext& Context) const noexcept
    {
        auto Result = Predict ? Predict(Context) : Hold(Context);
        Result.Frame = Context.Frame;
        Result.Reset = false;
        return Result;
    }
};

// A game supplies the value type and the comparison. The frame and actor
// identity check is shared and runs before any game callback.
enum class EPhysicsPredictionMatch : std::uint8_t { Unavailable, WithinLimits, Divergent };

template<class TPhysicsState>
struct TNetworkPhysicsState
{
    std::uint64_t Frame = 0;
    std::uint64_t Actor = 0;
    std::uint64_t Incarnation = 0;
    TPhysicsState State;
};

template<class TPhysicsState>
struct TNetworkPhysicsPredictionPolicy
{
    using FSample = TNetworkPhysicsState<TPhysicsState>;
    using FExtrapolate = std::optional<TPhysicsState> (*)(const TPhysicsState&, std::uint64_t Frames) noexcept;
    using FCompare = EPhysicsPredictionMatch (*)(const TPhysicsState&, const TPhysicsState&) noexcept;
    FCompare Compare = nullptr;
    FExtrapolate Extrapolate = nullptr;

    std::optional<FSample> Predict(const FSample& Earlier, std::uint64_t TargetFrame, std::uint64_t MaxFrames) const noexcept
    {
        if (!Extrapolate || !Earlier.Frame || !Earlier.Actor || !Earlier.Incarnation
            || TargetFrame <= Earlier.Frame || TargetFrame - Earlier.Frame > MaxFrames)
            return {};
        auto State = Extrapolate(Earlier.State, TargetFrame - Earlier.Frame);
        if (!State) return {};
        return FSample{TargetFrame, Earlier.Actor, Earlier.Incarnation, std::move(*State)};
    }

    // A game may pass a callback capturing its immutable per-world limits.
    // The policy calls it synchronously and retains neither callback nor context.
    template<class FComparison>
    EPhysicsPredictionMatch CompareSameFrame(const FSample& Authority, const FSample& Predicted,
        FComparison&& CompareState) const noexcept
    {
        static_assert(std::is_nothrow_invocable_r_v<EPhysicsPredictionMatch, FComparison,
            const TPhysicsState&, const TPhysicsState&>,
            "A physics comparison must return a match and be noexcept");
        if (!Authority.Frame || !Authority.Actor || !Authority.Incarnation
            || Authority.Frame != Predicted.Frame || Authority.Actor != Predicted.Actor
            || Authority.Incarnation != Predicted.Incarnation)
            return EPhysicsPredictionMatch::Unavailable;
        return std::forward<FComparison>(CompareState)(Authority.State, Predicted.State);
    }

    EPhysicsPredictionMatch CompareSameFrame(const FSample& Authority, const FSample& Predicted) const noexcept
    {
        if (!Compare)
            return EPhysicsPredictionMatch::Unavailable;
        return CompareSameFrame(Authority, Predicted, Compare);
    }
};
}
