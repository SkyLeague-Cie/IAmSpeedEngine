#pragma once

#include <cstdint>

namespace Speed
{
enum class EStaticCollisionBackend : std::uint8_t
{
    UnrealLegacy = 0,
    AnalyticHybrid,
    SurfaceAnalytic,
};

struct FExactRoundStaticCollisionSettings final
{
    EStaticCollisionBackend RequestedBackend = EStaticCollisionBackend::SurfaceAnalytic;
    bool CompactAuthority = false;

    EStaticCollisionBackend SelectedBackend() const noexcept
    {
        return RequestedBackend == EStaticCollisionBackend::UnrealLegacy && CompactAuthority
            ? EStaticCollisionBackend::AnalyticHybrid : RequestedBackend;
    }

    bool operator==(const FExactRoundStaticCollisionSettings&) const = default;
};

// Owner-local evidence limited to strict route selection and query generation.
struct FExactRoundStaticRouteWitness final
{
    EStaticCollisionBackend Backend;
    std::uint64_t ProviderGeneration;

    bool operator==(const FExactRoundStaticRouteWitness&) const = default;
};
}
