#pragma once

#include "IAmSpeed/World/Simulation/PreparedSimulationRestore.h"

class USpeedMovementComponent;
class USpeedWheeledComponent;
class USolidSubBody;
class USensorSubBody;
class USSubBody;

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
        struct FActorConfiguration
        {
            uint64 StableId = 0, ConfigurationHash = 0;
            bool operator==(const FActorConfiguration& B) const noexcept
            { return StableId == B.StableId && ConfigurationHash == B.ConfigurationHash; }
        };
        struct FSubBody
        {
            uint64 StableId = 0, ActorStableId = 0;
            uint32 NativeObjectId = 0;
            bool IsSolidEndpoint = false;
            uint64 Incarnation = 0, ConfigurationHash = 0;
            bool operator==(const FSubBody& B) const noexcept
            {
                return StableId == B.StableId && ActorStableId == B.ActorStableId && NativeObjectId == B.NativeObjectId
                    && IsSolidEndpoint == B.IsSolidEndpoint && Incarnation == B.Incarnation
                    && ConfigurationHash == B.ConfigurationHash;
            }
        };
        std::vector<uint64> Actors;
        // The native actor registration itself is the monotonic StableId.
        // The game still joins its own actor incarnation and schemas to it.
        std::vector<FActorConfiguration> ActorConfigurations;
        std::vector<FSubBody> SubBodies;
        uint64 WorkerGeneration = 0;
        bool HasStaticGeometryAuthority = false;
        bool operator==(const FRoster& B) const noexcept
        {
            return Actors == B.Actors && ActorConfigurations == B.ActorConfigurations && SubBodies == B.SubBodies
                && WorkerGeneration == B.WorkerGeneration && HasStaticGeometryAuthority == B.HasStaticGeometryAuthority;
        }
    };

    // Values-only census of the actual registry. Capture on the original
    // owner before staging. The exact route includes the registered endpoint
    // incarnation and live configuration hashes. Actor schemas and the wire
    // actor incarnation still come from the game binding.
    static std::optional<FRoster> CaptureRoster(const FSimulationWorld&, const PreparedWorld::FAddress&,
        const ISimulationRestoreBoundaryLease&);

    // Values of just the exact dispatch target. The request, component call
    // scope and sealed envelope must still be current under the retained lease.
    static std::optional<std::vector<FRoster::FSubBody>> CaptureBoundSubBodies(const FSimulationComponentRestoreRequest&);

    // Use the exact request by reference inside PrepareCompleteSimulationRestore.
    // The factory resolves the world without exposing a registry or world view.
    static std::unique_ptr<FPreparedNativeKinematicFields> Prepare(USpeedMovementComponent& Target,
        const FSimulationComponentRestoreRequest&);
    static std::unique_ptr<FPreparedNativeKinematicFields> Prepare(USpeedWheeledComponent& Target,
        const FSimulationComponentRestoreRequest&);
    static std::unique_ptr<FPreparedNativeKinematicFields> PrepareSolid(USolidSubBody& Target,
        const FSimulationComponentRestoreRequest&, const PreparedWorld::FEndpointBinding&, const SKinematic& EndpointState);
    static std::unique_ptr<FPreparedNativeKinematicFields> PrepareSolid(const FSimulationComponentRestoreRequest&,
        const PreparedWorld::FEndpointBinding&, const SKinematic& EndpointState);

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
    friend class FExactRemoteRoundBuild;
    enum class EPhase : uint8 { Prepared, Ready, Finalized, Released, Aborted };
    static std::optional<FRoster> CaptureRosterContents(const FSimulationWorld&);
    static uint64 ReadActorConfiguration(const ISpeedComponent&);
    static std::optional<FRoster::FSubBody> ReadSubBodyRow(
        const FSimulationWorld&, const ISpeedComponent&, uint64 ActorId, uint16 Index, const USSubBody&);
    static bool MatchesActor(const FSimulationWorld&, const FSimulationComponentRestoreRequest&, const ISpeedComponent&);
    static bool MatchesSolid(const FSimulationWorld&, const FSimulationComponentRestoreRequest&,
        const USolidSubBody&, const PreparedWorld::FEndpointBinding&);
    static bool MatchesSensor(const FSimulationWorld&, const FSimulationComponentRestoreRequest&,
        const USensorSubBody&, const FRoster::FSubBody&);
    static USolidSubBody* ResolveBoundSolid(const FSimulationWorld&, const FSimulationComponentRestoreRequest&,
        const PreparedWorld::FEndpointBinding&);
    static USensorSubBody* ResolveBoundSensor(const FSimulationWorld&, const FSimulationComponentRestoreRequest&,
        const FRoster::FSubBody&);
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
