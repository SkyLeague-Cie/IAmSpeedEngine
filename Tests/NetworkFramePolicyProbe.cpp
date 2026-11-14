#include "IAmSpeed/Netcode/transport/NetworkFramePolicy.h"
#include <cmath>
#include <cstdio>

using namespace Speed::Netcode;

static unsigned Checks = 0;
static unsigned Failures = 0;

static void Check(bool Passed, const char* Name)
{
    ++Checks;
    if (!Passed)
    {
        ++Failures;
        std::printf("FAIL %s\n", Name);
    }
}

static EPhysicsPredictionMatch CompareValue(const double& Authority, const double& Predicted) noexcept
{
    return std::abs(Authority - Predicted) <= 1
        ? EPhysicsPredictionMatch::WithinLimits : EPhysicsPredictionMatch::Divergent;
}

static std::optional<double> Extrapolate(const double& Value, std::uint64_t Frames) noexcept
{
    return Value + static_cast<double>(Frames);
}

int main()
{
    constexpr FNetworkInputPredictionPolicy Input{FNetworkInputPredictionPolicy::InterpolateAxes};
    const FNetworkInputState Before{10, {200, 30, -40, 1}, 15};
    const FNetworkInputState After{12, {100, 10, 20, 0}, 7};
    const auto Held = Input.Evaluate({11, Before});
    Check(Held.Frame == 11 && Held.Values == Before.Values && Held.ActiveMask == 15,
        "missing later input retains values");
    const auto Gap = Input.Evaluate({11, Before, &After, 7});
    Check(Gap.Values[0] == 150 && Gap.Values[1] == 20 && Gap.Values[2] == -10,
        "only selected axes interpolate");
    Check(Gap.Values[3] == 1 && (Gap.ActiveMask & 8), "digital state stays held");
    Check(!Gap.Reset, "prediction does not synthesize reset");
    auto ResetBefore = Before;
    ResetBefore.Reset = true;
    auto ResetAfter = After;
    ResetAfter.Reset = true;
    Check(Input.Evaluate({11, ResetBefore, &After, 7}).Values == Before.Values,
        "earlier reset blocks interpolation");
    Check(Input.Evaluate({11, Before, &ResetAfter, 7}).Values == Before.Values,
        "later reset blocks interpolation");
    auto Far = After;
    Far.Frame = Before.Frame + FNetworkInputPredictionPolicy::Window;
    Check(Input.Evaluate({11, Before, &Far, 7}).Values == Before.Values,
        "window boundary holds");
    Check(Input.Evaluate({10, Before, &After, 7}).Values == Before.Values,
        "caller owns earlier exact frame");
    Check(Input.Evaluate({12, Before, &After, 7}).Values == Before.Values,
        "caller owns later exact frame");
    auto HighBefore = Before;
    auto HighAfter = After;
    HighBefore.Values[31] = -100;
    HighAfter.Values[31] = 100;
    const auto High = Input.Evaluate({11, HighBefore, &HighAfter, std::uint32_t{1} << 31});
    Check(High.Values[31] == 0 && High.Values[0] == Before.Values[0], "axis 31 mask is bounded");
    Check(FNetworkInputState::ActionCount == 32, "fixed input representation");

    using FSample = TNetworkPhysicsState<double>;
    const FSample Authority{11, 1, 2, 0};
    const FSample Predicted{11, 1, 2, 2};
    const TNetworkPhysicsPredictionPolicy<double> Physics{CompareValue, Extrapolate};
    Check(Physics.CompareSameFrame(Authority, Predicted) == EPhysicsPredictionMatch::Divergent,
        "stored callback remains supported");
    unsigned Calls = 0;
    const double LooseLimit = 3;
    const auto Loose = [&Calls, LooseLimit](const double& A, const double& B) noexcept
    {
        ++Calls;
        return std::abs(A - B) <= LooseLimit
            ? EPhysicsPredictionMatch::WithinLimits : EPhysicsPredictionMatch::Divergent;
    };
    const double TightLimit = 1;
    const auto Tight = [TightLimit](const double& A, const double& B) noexcept
    {
        return std::abs(A - B) <= TightLimit
            ? EPhysicsPredictionMatch::WithinLimits : EPhysicsPredictionMatch::Divergent;
    };
    Check(Physics.CompareSameFrame(Authority, Predicted, Loose) == EPhysicsPredictionMatch::WithinLimits,
        "captured world limits are applied");
    Check(Physics.CompareSameFrame(Authority, Predicted, Tight) == EPhysicsPredictionMatch::Divergent,
        "another world has independent limits");
    Check(Calls == 1, "callback executes once");
    const FSample Invalid[] = {{0, 1, 2, 0}, {11, 0, 2, 0}, {11, 1, 0, 0},
        {12, 1, 2, 0}, {11, 2, 2, 0}, {11, 1, 3, 0}};
    for (const auto& Sample : Invalid)
    {
        Check(Physics.CompareSameFrame(Authority, Sample, Loose) == EPhysicsPredictionMatch::Unavailable,
            "identity mismatch blocks callback");
        Check(Physics.CompareSameFrame(Sample, Predicted, Loose) == EPhysicsPredictionMatch::Unavailable,
            "invalid authority blocks callback");
    }
    Check(Calls == 1, "invalid samples do not reach game callback");
    const TNetworkPhysicsPredictionPolicy<double> Empty;
    Check(Empty.CompareSameFrame(Authority, Predicted) == EPhysicsPredictionMatch::Unavailable,
        "missing stored callback is unavailable");
    Check(Empty.CompareSameFrame(Authority, Predicted, Tight) == EPhysicsPredictionMatch::Divergent,
        "call-scoped callback needs no global policy state");
    const auto Future = Physics.Predict(Authority, 13, 2);
    Check(Future && Future->Frame == 13 && Future->State == 2 && Future->Actor == 1
        && Future->Incarnation == 2, "bounded extrapolation preserves identity");
    Check(!Physics.Predict(Authority, 14, 2), "extrapolation horizon enforced");
    Check(!Physics.Predict(Authority, 11, 2), "extrapolation requires later frame");
    Check(!Empty.Predict(Authority, 12, 2), "missing extrapolator is unavailable");
    std::printf("checks=%u failures=%u\n", Checks, Failures);
    return Failures ? 1 : 0;
}
