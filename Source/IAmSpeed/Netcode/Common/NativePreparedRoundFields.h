#pragma once

#include "IAmSpeed/Components/Netcode/SpeedPhysicsComponent.h"
#include "IAmSpeed/Components/Netcode/SpeedWheeledPhysicsComponent.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "IAmSpeed/World/Simulation/PreparedSimulationRestore.h"

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

    // Call these with the exact request received by PrepareCompleteSimulationRestore.
    // The local frame is derived from the canonical address. History is sized
    // for the original recorder. SeedRestoredFrame installs only that frame.
    static std::unique_ptr<FPreparedNativeRoundFields> Prepare(USpeedMovementComponent&,
        const FSimulationComponentRestoreRequest&, const FActorTimeline&);
    static std::unique_ptr<FPreparedNativeRoundFields> Prepare(USpeedWheeledComponent&,
        const FSimulationComponentRestoreRequest&, const FVehicleState&);
    // For a contact-free reset. The first new physical step reacquires hits.
    static std::unique_ptr<FPreparedNativeRoundFields> PrepareClearSolidContacts(USolidSubBody&,
        const FSimulationComponentRestoreRequest&, const PreparedWorld::FEndpointBinding&);
    // Includes cleared solid contacts. Combine with the separate solid pose,
    // and supply the matching suspension displacement in FVehicleState.
    static std::unique_ptr<FPreparedNativeRoundFields> PrepareWheel(USWheelSubBody&,
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
    struct FPlan;
    FPreparedNativeRoundFields(const FSimulationWorld&, const FSimulationComponentRestoreRequest&);
    void StageClearContacts(USolidSubBody&);
    std::unique_ptr<FPlan> Plan;
};
}
