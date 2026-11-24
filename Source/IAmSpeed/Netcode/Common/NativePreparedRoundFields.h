#pragma once

#include "NativePreparedKinematicFields.h"
#include "IAmSpeed/Components/Netcode/SpeedPhysicsComponent.h"
#include "IAmSpeed/Components/Netcode/SpeedWheeledPhysicsComponent.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include <array>

class ASpeedPhysicsSimulation;
class USpeedMovementComponent;

namespace Speed
{
// Named native fragments for a new round. The complete actor plan combines
// these with COM/frozen, solid poses, common caches and the game's own state.
class IAMSPEED_API FPreparedNativeRoundFields final
{
public:
    enum class EHistory { Empty, SeedRestoredFrame };

    struct FActorTimeline
    {
        int32 SinceCanMoveFrame = INDEX_NONE;
        uint16 BaseFramesBeforeCanMove = 1;
        bool bBaseCountdownStarted = false;
        EHistory History = EHistory::Empty;
    };

    struct FVehicleState
    {
        FActorTimeline Timeline;
        FWheeledGameState Game;
        FWheeledPhysicsState Physics;
        FWheeledInputState PhysicalInput;
        bool bSupportStartupRequested = false;
        bool bSupportStartupConfirmed = false;
        bool bTimelineWasCanMove = false;
        bool bTimelineHasGroundState = false;
        bool bTimelineWasGrounded = false;
    };

    struct FWheelState
    {
        float SuspensionLengthCm = 0.0f;
        float LastDisplacementCm = 0.0f;
        float AngularVelocity = 0.0f;
        float RollAngle = 0.0f;
        float SteeringAngle = 0.0f;
        uint8 JumpFrames = 0;
        bool bJumpUnilateralSupport = false;
        SWheelRenderData Render;
    };

    struct FCapturedKickoffWheel
    {
        FPreparedNativeKinematicFields::FRoster::FSubBody Binding;
        SKinematic Pose;
        FWheelState State;
        FVector LocalOffset;
        FQuat LocalRotation;
        float MaxSuspensionLengthCm = 0.0f;
        float SuspensionRestRaiseCm = 0.0f;
    };

    struct FCapturedKickoffVehicle
    {
        uint64 WorkerGeneration = 0;
        uint64 FirstFrame = 0;
        uint64 ActorStableId = 0;
        uint32 ActorNativeObjectId = 0;
        SKinematic CenterOfMass;
        FVector LocalCenterOfMass;
        bool bFrozen = false;
        FVehicleState State;
        std::array<FCapturedKickoffWheel, 4> Wheels;
    };

    struct FContactFreeKickoff
    {
        struct FWheel { FPreparedNativeKinematicFields::FRoster::FSubBody Binding; SKinematic Pose; FWheelState State; };
        FVehicleState Vehicle;
        std::array<FWheel, 4> Wheels;
    };

    // Copies the four actual indexed wheels while the original worker is
    // paused and its boundary service is already acknowledged as parked.
    // The caller owns that park. This read neither extends it nor grants a lease.
    static std::optional<FCapturedKickoffVehicle> CaptureKickoffVehicleGT(
        const ASpeedPhysicsSimulation&, uint64 ExpectedGeneration, uint64 ExpectedFirstFrame,
        const USpeedWheeledComponent&);

    // Reads the movement countdown floor from the original registered
    // component during the caller's acknowledged startup park. Zero is valid.
    static std::optional<uint16> ReadMovementMinimumFramesAtParkedStartupGT(
        const ASpeedPhysicsSimulation&, uint64 ExpectedGeneration, uint64 ExpectedFirstFrame,
        uint64 ExpectedActorStableId, const USpeedMovementComponent&);

    // Projects a stationary target with cold wheel contacts. The optional
    // support request sweeps the installed pose on the first movable frame.
    // The result carries no support witness for the target at the reset frame.
    static std::optional<FContactFreeKickoff> ProjectContactFreeKickoff(
        const FCapturedKickoffVehicle&, const SKinematic& TargetCenterOfMass, bool bRequestStartupSupport);

    // Call these with the exact request received by PrepareCompleteSimulationRestore.
    // The local frame is derived from the canonical address. History is sized
    // for the original recorder. SeedRestoredFrame installs only that frame.
    static std::unique_ptr<FPreparedNativeRoundFields> Prepare(USpeedMovementComponent&,
        const FSimulationComponentRestoreRequest&, const FActorTimeline&);
    static std::unique_ptr<FPreparedNativeRoundFields> Prepare(USpeedWheeledComponent&,
        const FSimulationComponentRestoreRequest&, const FVehicleState&);
    // Clears derived common contact, sleep and quantization caches. The
    // existing static geometry remains pinned by the world lease.
    static std::unique_ptr<FPreparedNativeRoundFields> PrepareColdCommon(ISpeedComponent&,
        const FSimulationComponentRestoreRequest&);
    // For a contact-free reset. The first new physical step reacquires hits.
    static std::unique_ptr<FPreparedNativeRoundFields> PrepareClearSolidContacts(USolidSubBody&,
        const FSimulationComponentRestoreRequest&, const PreparedWorld::FEndpointBinding&);
    static std::unique_ptr<FPreparedNativeRoundFields> PrepareClearSolidContacts(
        const FSimulationComponentRestoreRequest&, const PreparedWorld::FEndpointBinding&);
    // Use the native sensor's row from the reauthenticated roster. The reset
    // clears transient overlap/hit state without broadcasting overlap events.
    static std::unique_ptr<FPreparedNativeRoundFields> PrepareClearSensorOverlaps(USensorSubBody&,
        const FSimulationComponentRestoreRequest&, const FPreparedNativeKinematicFields::FRoster::FSubBody&);
    static std::unique_ptr<FPreparedNativeRoundFields> PrepareClearSensorOverlaps(
        const FSimulationComponentRestoreRequest&, const FPreparedNativeKinematicFields::FRoster::FSubBody&);
    // Keep an original sensor's ordered overlaps and membership through this
    // restore. The optional copy contains only sealed, local solid bindings.
    // Slot is the low 16 bits of StableId. Normal frame reset and events are
    // untouched. Compose this fragment with the other actor domains.
    static std::unique_ptr<FPreparedNativeRoundFields> PrepareKeepSensorOverlaps(
        const FSimulationComponentRestoreRequest&, const FPreparedNativeKinematicFields::FRoster::FSubBody&,
        std::vector<PreparedWorld::FEndpointBinding>* RetainedOrdered = nullptr);
    // Includes cleared solid contacts. Combine with the separate solid pose,
    // and supply the matching suspension displacement in FVehicleState.
    static std::unique_ptr<FPreparedNativeRoundFields> PrepareWheel(USWheelSubBody&,
        const FSimulationComponentRestoreRequest&, const PreparedWorld::FEndpointBinding&, const FWheelState&);
    static std::unique_ptr<FPreparedNativeRoundFields> PrepareWheel(
        const FSimulationComponentRestoreRequest&, const PreparedWorld::FEndpointBinding&, const FWheelState&);

    FPreparedNativeRoundFields(const FPreparedNativeRoundFields&) = delete;
    FPreparedNativeRoundFields& operator=(const FPreparedNativeRoundFields&) = delete;
    ~FPreparedNativeRoundFields() noexcept;

    bool AppendPreparedAccess(std::vector<PreparedAccess::FParticipantAccess>&) const;
    bool PrepareFinalization() noexcept;
    void Finalize() noexcept;
    void AbortBeforeFinalize() noexcept;
    void ReleaseAfterPublication() noexcept;

private:
    static bool MatchesParkedKickoffDriverGT(
        const ASpeedPhysicsSimulation&, uint64 ExpectedGeneration, uint64 ExpectedFirstFrame);
    struct FPlan;
    FPreparedNativeRoundFields(const FSimulationWorld&, const FSimulationComponentRestoreRequest&);
    void StageClearContacts(USSubBody&);
    std::unique_ptr<FPlan> Plan;
};
}
