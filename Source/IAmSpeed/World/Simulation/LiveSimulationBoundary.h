#pragma once
#include "CoreMinimal.h"
#include "SimulationFrameJournal.h"
#include <atomic>
#include <memory>
#include <utility>

class ASpeedPhysicsSimulation;
class FSimulationWorker;
class UWorld;

namespace Speed::SimulationBoundary
{
enum class ELiveBoundaryResult : uint8 { Waiting, Completed, Cancelled, Rejected, Retired };
namespace Detail
{
// Shared ownership survives handle cancellation until the worker proves disposal.
// These phases carry no physical/input state or user pause intent.
enum class ELiveBoundaryPhase : uint8 { Requested, Acknowledged, Executing, ReleaseRequested, Disposed, Retired };
struct FLiveBoundaryControl final
{
    std::atomic<ELiveBoundaryPhase> Phase{ELiveBoundaryPhase::Requested};
    std::atomic<bool> CancelRequested{false}, WasAcknowledged{false};
    uint64 RequestSerial = 0;
    TWeakObjectPtr<ASpeedPhysicsSimulation> Driver;
    TWeakObjectPtr<UWorld> World;
    uint64 Generation = 0, NextFrame = 0;
    FSimulationSnapshot Snapshot;
    bool CallbackRan = false, CallbackAccepted = false, ScopeActive = false, RetirementRequested = false;
    FString Failure;
};
IAMSPEED_API void RequestLiveBoundaryCancellation(const std::shared_ptr<FLiveBoundaryControl>& Control) noexcept;
}

// GT-owned opaque request. Destruction requests cancellation without waiting;
// the driver and worker retain the request until disposal or an actual join.
class IAMSPEED_API FLiveBoundaryRequest final
{
public:
    ~FLiveBoundaryRequest();
    FLiveBoundaryRequest(const FLiveBoundaryRequest&) = delete;
    FLiveBoundaryRequest& operator=(const FLiveBoundaryRequest&) = delete;
    FLiveBoundaryRequest(FLiveBoundaryRequest&&) = delete;
    FLiveBoundaryRequest& operator=(FLiveBoundaryRequest&&) = delete;
private:
    friend class ::ASpeedPhysicsSimulation;
    explicit FLiveBoundaryRequest(std::shared_ptr<Detail::FLiveBoundaryControl> InControl) : Control(std::move(InControl)) {}
    const std::shared_ptr<Detail::FLiveBoundaryControl> Control;
};

// Ephemeral capability, minted only inside an acknowledged live GT callback.
// It cannot be made from an archive/token or copied beyond the callback scope.
class IAMSPEED_API FLiveBoundaryLease final
{
public:
    FLiveBoundaryLease(const FLiveBoundaryLease&) = delete;
    FLiveBoundaryLease& operator=(const FLiveBoundaryLease&) = delete;
    FLiveBoundaryLease(FLiveBoundaryLease&&) = delete;
    FLiveBoundaryLease& operator=(FLiveBoundaryLease&&) = delete;
    ASpeedPhysicsSimulation* Driver() const;
    UWorld* World() const;
    uint64 WorkerGeneration() const;
    uint64 CompletedFrame() const;
    uint64 NextFrame() const;
    uint64 PublicationSerial() const;
    const FSimulationSnapshot& Snapshot() const;
    bool IsCurrent(const ASpeedPhysicsSimulation& Host) const;
private:
    friend class ::ASpeedPhysicsSimulation;
    explicit FLiveBoundaryLease(std::shared_ptr<Detail::FLiveBoundaryControl> InControl) : Control(std::move(InControl)) {}
    const std::shared_ptr<Detail::FLiveBoundaryControl> Control;
};
}
