#pragma once

#include "ExactRoundConfiguration.h"
#include <optional>

class ASpeedPhysicsSimulation;

namespace Speed
{
// Native response and projection policy consumed by the solver.
// Subbody-specific values still belong to the native round roster.
struct FExactRoundContactSettings final
{
    static constexpr std::uint64_t HashSchema = 2;

    std::optional<float> SphereBoxRestitution;
    std::optional<float> SphereBoxFriction;
    std::optional<float> SphereBoxManifoldFriction;
    bool AerialShotModel = false;
    bool ForceCoupledImpulse = false;
    float CoupledMinNormalSpeed = 0.f;
    std::int32_t CoupledIterations = 1;
    bool SphereBoxProjectionEnabled = true;
    bool SphereWorldStaticProjectionEnabled = true;

    bool operator==(const FExactRoundContactSettings&) const = default;

    std::uint64_t Hash() const noexcept
    {
        FExactRoundConfigurationWriter Writer;
        const auto WriteOverride = [&Writer](const std::optional<float>& Value)
        {
            Writer.Boolean(Value.has_value());
            if (Value) Writer.Float(*Value);
        };
        Writer.Unsigned(0x4941534354530000ull | HashSchema);
        WriteOverride(SphereBoxRestitution);
        WriteOverride(SphereBoxFriction);
        WriteOverride(SphereBoxManifoldFriction);
        Writer.Boolean(AerialShotModel);
        Writer.Boolean(ForceCoupledImpulse);
        Writer.Float(CoupledMinNormalSpeed);
        Writer.Signed(CoupledIterations);
        Writer.Boolean(SphereBoxProjectionEnabled);
        Writer.Boolean(SphereWorldStaticProjectionEnabled);
        return Writer.Hash();
    }

private:
    friend class ::ASpeedPhysicsSimulation;
    static std::optional<FExactRoundContactSettings> Capture() noexcept;
    bool CaptureSolidSettings() noexcept;
    void CaptureSphereProjectionSettings() noexcept;
};

// Response and projection are synchronous on the canonical owner. Keep
// their reads scoped to that call and exact preparation on the same owner.
class FScopedExactRoundContactSettings final
{
public:
    explicit FScopedExactRoundContactSettings(const FExactRoundContactSettings* Settings) noexcept
        : Previous(Active) { Active = Settings; }
    ~FScopedExactRoundContactSettings() { Active = Previous; }
    FScopedExactRoundContactSettings(const FScopedExactRoundContactSettings&) = delete;
    FScopedExactRoundContactSettings& operator=(const FScopedExactRoundContactSettings&) = delete;

    static const FExactRoundContactSettings* Current() noexcept { return Active; }

private:
    inline static thread_local const FExactRoundContactSettings* Active = nullptr;
    const FExactRoundContactSettings* const Previous;
};
}
