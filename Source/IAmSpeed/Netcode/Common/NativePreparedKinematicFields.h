#pragma once

#include "IAmSpeed/World/Simulation/PreparedSimulationRestore.h"

class USpeedMovementComponent;
class USpeedWheeledComponent;
class USolidSubBody;
class USensorSubBody;

namespace Speed
{
// Partial native fragment for a COM pose/frozen bit, or one solid endpoint's
// kinematic cache. The complete actor retains the exact prepared-world lease
// and composes gameplay, histories, hit state, configuration and lifecycle.
class IAMSPEED_API FPreparedNativeKinematicFields final
{
public:
    struct FRoster
    {
        struct FSubBody
        {
            uint64 StableId = 0, ActorStableId = 0;
            uint32 NativeObjectId = 0;
            bool IsSolidEndpoint = false;
        };
        std::vector<uint64> Actors;
        std::vector<FSubBody> SubBodies;
        bool HasStaticGeometryAuthority = false;
    };

    // Values-only census of the actual registry. Capture on the original
    // owner before staging; incarnation and schema come from the game binding.
    static std::optional<FRoster> CaptureRoster(const FSimulationWorld&, const PreparedWorld::FAddress&,
        const ISimulationRestoreBoundaryLease&);

    // Use the exact request by reference inside PrepareCompleteSimulationRestore.
    // The factory resolves the world without exposing a registry or world view.
    static std::unique_ptr<FPreparedNativeKinematicFields> Prepare(USpeedMovementComponent& Target,
        const FSimulationComponentRestoreRequest&);
    static std::unique_ptr<FPreparedNativeKinematicFields> Prepare(USpeedWheeledComponent& Target,
        const FSimulationComponentRestoreRequest&);
    static std::unique_ptr<FPreparedNativeKinematicFields> PrepareSolid(USolidSubBody& Target,
        const FSimulationComponentRestoreRequest&, const PreparedWorld::FEndpointBinding&, const SKinematic& EndpointState);

    static std::unique_ptr<FPreparedNativeKinematicFields> Prepare(USpeedMovementComponent& Target,
        const FSimulationWorld&, const FSimulationComponentRestoreRequest&);
    static std::unique_ptr<FPreparedNativeKinematicFields> Prepare(USpeedWheeledComponent& Target,
        const FSimulationWorld&, const FSimulationComponentRestoreRequest&);
    static std::unique_ptr<FPreparedNativeKinematicFields> PrepareSolid(USolidSubBody& Target,
        const FSimulationWorld&, const FSimulationComponentRestoreRequest&, const PreparedWorld::FEndpointBinding&,
        const SKinematic& EndpointState);

    FPreparedNativeKinematicFields(const FPreparedNativeKinematicFields&) = delete;
    FPreparedNativeKinematicFields& operator=(const FPreparedNativeKinematicFields&) = delete;
    ~FPreparedNativeKinematicFields() noexcept;
    bool AppendPreparedAccess(std::vector<PreparedAccess::FParticipantAccess>&) const;
    bool PrepareFinalization() noexcept;
    void Finalize() noexcept;
    void AbortBeforeFinalize() noexcept;
    void ReleaseAfterPublication() noexcept;

private:
    friend struct FCommonComponentValueAccess;
    friend class FPreparedNativeRoundFields;
    enum class EPhase : uint8 { Prepared, Ready, Finalized, Released, Aborted };
    static bool MatchesActor(const FSimulationWorld&, const FSimulationComponentRestoreRequest&, const ISpeedComponent&);
    static bool MatchesSolid(const FSimulationWorld&, const FSimulationComponentRestoreRequest&,
        const USolidSubBody&, const PreparedWorld::FEndpointBinding&);
    static bool MatchesSensor(const FSimulationWorld&, const FSimulationComponentRestoreRequest&,
        const USensorSubBody&, const FRoster::FSubBody&);
    FPreparedNativeKinematicFields(const FSimulationWorld&, const FSimulationComponentRestoreRequest&,
        SKinematic& Live, bool* Frozen, const SKinematic& State);

    const FSimulationWorld& World;
    const PreparedWorld::FAddress Address;
    const FPreparedRestoreLeaseView Lease;
    SKinematic& LiveState;
    bool* const LiveFrozen;
    const SKinematic State;
    const bool Frozen;
    EPhase Phase = EPhase::Prepared;
};
}
