#pragma once

#include "CompletedSimulationPublication.h"
#include "ExactRemoteRoundFence.h"
#include "IAmSpeed/Netcode/Common/NativePreparedKinematicFields.h"
#include "PreparedWorldReplacement.h"
#include "SimulationWorld.h"
#include <atomic>
#include <chrono>

namespace Speed
{
// All preparation is entered by the original owned authority worker. A game
// supplies the complete typed snapshot, the world supplement and the native
// lease protecting its adapters and escaped mutable views.
struct FExactRemoteRoundPlan
{
    using FClock = std::chrono::steady_clock;
    std::shared_ptr<const FCompletedSimulationPublication> Boundary;
    FSimulationSnapshot Replacement;
    std::shared_ptr<const PreparedWorld::FRestoreEnvelope> Envelope;
    std::unique_ptr<ISimulationRestoreBoundaryLease> Lease;
    std::unique_ptr<IPreparedSimulationWorldSupplement> Supplement;
    Input::V2::FInputSessionRegistry::FGroupResumeRequest Inputs;
    FClock::time_point Deadline{};
    // A request observed here before finalization aborts both participants.
    // After commit, host cancellation closes gameplay rather than rewinding.
    std::shared_ptr<const std::atomic<bool>> Cancellation;
};

struct FExactRemoteRoundCommit
{
    std::shared_ptr<const FCompletedSimulationPublication> Previous;
    std::shared_ptr<const FSimulationWorld::FPreparedRestorePublication> World;
    std::shared_ptr<const Input::V2::FInputSessionRegistry::FGroupResumePublication> Inputs;
    uint64 NeutralBaselineHash = 0;
    FExactRemoteRoundPlan::FClock::time_point Deadline{};
    std::shared_ptr<const std::atomic<bool>> Cancellation;
};

// Values-only preparation on the original worker, inside the provider's
// Build call for this exact completed publication. No native object pin or
// permission to stage/commit is granted here.
class IAMSPEED_API FExactRemoteRoundBuild final
{
public:
    using FRoster = FPreparedNativeKinematicFields::FRoster;

    struct FActorReplacement { uint64 StableId = 0; SKinematic Kinematic; bool bFrozen = false; };
    struct FReplacement
    {
        FSimulationSnapshot Snapshot;
        std::shared_ptr<const PreparedWorld::FRestoreEnvelope> Envelope;
    };

    static std::optional<FRoster> CaptureRoster(const FSimulationWorld&, const FCompletedSimulationPublication&);

    // Sets Frame, Next, InputHash, StateHash and SupplementHash. The host
    // supplies the other address fields and full bindings/supplements.
    // Use the returned immutable envelope to acquire the real native lease.
    static std::optional<FReplacement> BuildReplacement(const FSimulationWorld&, const FCompletedSimulationPublication&,
        PreparedWorld::FRestoreEnvelope, const std::vector<FActorReplacement>&,
        const std::vector<PreparedWorld::FContactRemoval>&);
};

// Coordinator/provider installed before worker creation. Build runs on the
// same worker which will commit and step, with the native storage reserved.
// Build consumes the completed publication and lease-bound native values.
// Implementations must be bounded and must not wait on the game thread. The
// host must retain the original registered storage and gate direct views
// until the neutral step publishes or the original worker has joined.
class IAMSPEED_API IExactRemoteRoundProvider
{
public:
    virtual ~IExactRemoteRoundProvider() = default;
    // Called with an immutable completed publication, before the world is
    // reserved. Return true only if this exact boundary is needed.
    virtual bool WantsRoundBuild(const FCompletedSimulationPublication&) const noexcept = 0;
    virtual std::unique_ptr<FExactRemoteRoundPlan> Build(
        const FSimulationWorld&, const std::shared_ptr<const FCompletedSimulationPublication>&) = 0;
    enum class EBoundaryRefusal : uint8 { ActiveWorldBorrow, UntrackedWorldReference, RegistryOrFrameBusy };
    virtual void OnBoundaryRefused(EBoundaryRefusal) noexcept { OnRejected(); }
    virtual void OnRejected() noexcept = 0;
    // Called after the real step and immutable publication of the neutral F.
    // The host may authorize F+1 after the presentation ACKs are satisfied.
    virtual bool ReleaseGameplay(const FExactRemoteRoundCommit&,
        const FCompletedSimulationPublication& Neutral) noexcept = 0;
};

namespace ExactRemoteRound
{
// Same-worker composition. The caller preallocates an immutable publication
// slot and prevents other APIs from observing the participants until commit.
// A null result has aborted both prepared participants without a write.
std::shared_ptr<const FExactRemoteRoundCommit> Commit(
    FSimulationWorld&, Input::V2::FInputSessionRegistry&, FExactRemoteRoundFence&, FExactRemoteRoundPlan&&,
    std::shared_ptr<const FExactRemoteRoundCommit>& Publication);

bool IsExactNeutralStep(const FExactRemoteRoundCommit&, const FCompletedSimulationPublication&) noexcept;
}
}
