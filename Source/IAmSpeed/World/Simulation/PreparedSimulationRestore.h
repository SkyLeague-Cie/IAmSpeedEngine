#pragma once
#include "CoreMinimal.h"
#include "IAmSpeed/Base/Kinematic.h"
#include "PreparedWorldCore.h"
#include "PreparedAccessRanges.h"
#include <memory>

class ISpeedComponent;
namespace Speed
{
class FSimulationWorld;
// No implementation is supplied or admitted in this tranche. The worker owner
// must acquire this BEFORE reading any world/component/UObject address. It must
// pin world, components, sub-bodies, configurations and static geometry, exclude
// step/GT lifecycle/registry mutation AND consumers of escaped mutable views,
// and own an exact monotonic topology/state version. A caller boolean, a raw
// pointer, same-thread assumption or registry input fence is not such a lease.
class ISimulationRestoreBoundaryLease
{
public:
    // Implementations must abort idempotently from destruction before finalize;
    // after finalize they may only release, never restore an earlier state.
    virtual ~ISimulationRestoreBoundaryLease() = default;
    virtual bool HoldsExactBoundary(const FSimulationWorld&, const PreparedWorld::FAddress&) const noexcept { return false; }
    virtual bool ProtectsEscapedMutableViews() const noexcept { return false; }
    virtual bool AppendPreparedAccess(std::vector<PreparedAccess::FParticipantAccess>&) const { return false; }
    virtual void AbortBeforeFinalize() noexcept = 0;
    virtual void ReleaseAfterPublication() noexcept = 0;
};
}
#include "PreparedLeaseView.h"
namespace Speed
{
struct FSimulationComponentRestoreRequest
{
    const PreparedWorld::FAddress& Address;
    const PreparedWorld::FActorSupplement& Supplement;
    const SKinematic& Kinematic;
    bool bFrozen = false;
    TConstArrayView<uint8> LegacyMechanicPayload;
    FPreparedRestoreLeaseView OwnerLease; // safe weak capability, never a raw lease/ref
};
// Preparation must stage ALL listed domains, including common fields, hidden
// filters/sleep/quantization/sub-bodies, gameplay and configuration/lifecycle.
// Never delegate its finalization to RestoreSimulationSnapshot: that old hook
// resets caches and calls unchecked virtual setters/reconstruction routines.
// Plans own copies and lifetime/config pins; no request references may escape.
class IPreparedSimulationComponentRestore
{
public:
    // Idempotent aborting RAII before finalize is REQUIRED, including rejection
    // before this object is stored in the world's plan. No reentrant owner call.
    virtual ~IPreparedSimulationComponentRestore() = default;
    virtual const PreparedWorld::FAddress& Address() const noexcept = 0;
    virtual const PreparedWorld::FActorBinding& Binding() const noexcept = 0;
    virtual uint64 CoveredDomains() const noexcept = 0;
    // Closed by default. Composite providers append COMMON + wheel/subbody +
    // all other retained child-plan ranges, never a guessed actor-wide boolean.
    virtual bool AppendPreparedAccess(std::vector<PreparedAccess::FParticipantAccess>&) const { return false; }
    // Fallible acquisition/validation must happen here. Retain every acquired
    // fence through finalization and common publication. Failure stays read-only.
    virtual bool PrepareFinalization() = 0;
    virtual void Finalize() noexcept = 0; // no allocation, callbacks, queries, resets or reconstruction
    virtual void AbortBeforeFinalize() noexcept = 0;
    virtual void ReleaseAfterPublication() noexcept = 0;
};
// Global gameplay, immutable geometry authority, complete actor registry and
// lifecycle are not encoded by frozen schema2. They are mandatory supplemental
// participants. There is no default "empty means complete" implementation.
class IPreparedSimulationWorldSupplement
{
public:
    virtual bool BindOwnerLease(FPreparedRestoreLeaseView) { return false; }
    // Same aborting RAII and no-reentrancy requirement as component plans.
    virtual ~IPreparedSimulationWorldSupplement() = default;
    virtual const PreparedWorld::FAddress& Address() const noexcept = 0;
    virtual uint64 Schema() const noexcept = 0;
    virtual uint64 CoveredDomains() const noexcept = 0;
    // Closed by default. Composite providers append COMMON + wheel/subbody +
    // all other retained child-plan ranges, never a guessed actor-wide boolean.
    virtual bool AppendPreparedAccess(std::vector<PreparedAccess::FParticipantAccess>&) const { return false; }
    virtual bool PrepareFinalization() = 0;
    virtual void Finalize() noexcept = 0;
    virtual void AbortBeforeFinalize() noexcept = 0;
    virtual void ReleaseAfterPublication() noexcept = 0;
};
}
