#pragma once

#include "CompletedSimulationPublication.h"
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

// Coordinator/provider installed before worker creation. Build runs on the
// same worker which will commit and step. Return null if no operation is due.
// Implementations are bounded and non-blocking on the game thread. They obtain
// real leases and read immutable request data only.
class IAMSPEED_API IExactRemoteRoundProvider
{
public:
    virtual ~IExactRemoteRoundProvider() = default;
    virtual std::unique_ptr<FExactRemoteRoundPlan> Build(
        const FSimulationWorld&, const std::shared_ptr<const FCompletedSimulationPublication>&) = 0;
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
    FSimulationWorld&, Input::V2::FInputSessionRegistry&, FExactRemoteRoundPlan&&,
    std::shared_ptr<const FExactRemoteRoundCommit>& Publication);

bool IsExactNeutralStep(const FExactRemoteRoundCommit&, const FCompletedSimulationPublication&) noexcept;
}
}
