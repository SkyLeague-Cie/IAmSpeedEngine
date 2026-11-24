#pragma once

#include <cstdint>
#include <optional>

namespace Speed
{
namespace Analytic { enum class EAnalyticNumericContract : std::uint8_t; }

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

// Values identifying the tensor query representation attached to a strict route.
struct FExactRoundTensorGeometryWitness final
{
    static constexpr std::uint32_t Schema = 1;

    std::uint32_t DigestSchema = Schema;
    std::uint32_t GeometrySchema = 0;
    std::uint64_t Digest = 0;
    Analytic::EAnalyticNumericContract DeclaredNumericContract{};

    bool operator==(const FExactRoundTensorGeometryWitness&) const = default;
};

// Owner-local evidence of strict route, query generation and available tensor data.
struct FExactRoundStaticRouteWitness final
{
    EStaticCollisionBackend Backend;
    std::uint64_t ProviderGeneration;
    // Empty when the provider cannot attest the bounded tensor representation.
    std::optional<FExactRoundTensorGeometryWitness> TensorGeometry;

    bool operator==(const FExactRoundStaticRouteWitness&) const = default;
};
}
