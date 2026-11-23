#include "NativePreparedKinematicFields.h"

#include "IAmSpeed/Components/SpeedMovementComponent.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/World/Simulation/SimulationWorld.h"
#include <algorithm>
#include <type_traits>

namespace Speed
{
namespace
{
bool ValidKinematics(const SKinematic& State)
{
    return !State.Location.ContainsNaN() && !State.Velocity.ContainsNaN()
        && !State.Acceleration.ContainsNaN() && State.Rotation.IsNormalized()
        && !State.AngularVelocity.ContainsNaN() && !State.AngularAcceleration.ContainsNaN();
}

PreparedAccess::FSpan KinematicFieldSpan(const void* Address, std::size_t Size)
{
    return {reinterpret_cast<std::uintptr_t>(Address), Size};
}
}

bool FPreparedNativeKinematicFields::MatchesActor(const FSimulationWorld& World,
    const FSimulationComponentRestoreRequest& Request, const ISpeedComponent& Target)
{
    const auto& Binding = Request.Supplement.Binding;
    return Request.OwnerLease.HoldsExactBoundary(World, Request.Address) && Binding.Valid()
        && World.AdaptersByStableId.FindRef(Binding.StableId) == &Target
        && Target.GetPublishedSimulationStableId() == Binding.StableId;
}

std::optional<FPreparedNativeKinematicFields::FRoster> FPreparedNativeKinematicFields::CaptureRoster(
    const FSimulationWorld& World, const PreparedWorld::FAddress& Address, const ISimulationRestoreBoundaryLease& Lease)
{
    if (!Address.Valid() || !Lease.HoldsExactBoundary(World, Address) || !Lease.ProtectsEscapedMutableViews()
        || World.bOrderDirty || World.IsRestoreBoundaryReserved()) return {};

    FRoster Result;
    Result.HasStaticGeometryAuthority = World.StaticCollisionWorld.Get() != nullptr;
    Result.Actors.reserve(World.Bodies.Num());
    Result.SubBodies.reserve(World.StableSubBodyIds.Num());
    for (const auto& Record : World.Bodies)
    {
        if (!Record.StableId || !Record.Adapter || World.AdaptersByStableId.FindRef(Record.StableId) != Record.Adapter
            || Record.Adapter->GetPublishedSimulationStableId() != Record.StableId) return {};
        Result.Actors.push_back(Record.StableId);
    }
    int32 Solids = 0;
    for (const auto& Entry : World.StableSubBodyIds)
    {
        if (!Entry.Key || !Entry.Value || !Entry.Key->GetParentComponent()) return {};
        auto* Parent = Entry.Key->GetParentComponent();
        const uint64 Actor = World.StableIds.FindRef(Parent);
        if (!Actor || World.AdaptersByStableId.FindRef(Actor) != Parent) return {};
        const auto* Solid = World.SolidSubBodiesByStableId.FindRef(Entry.Value);
        if (Solid && Solid != Entry.Key) return {};
        if (Solid) ++Solids;
        Result.SubBodies.push_back({Entry.Value, Actor, Entry.Key->GetUniqueID(), Solid != nullptr});
    }
    if (Solids != World.SolidSubBodiesByStableId.Num()) return {};
    std::sort(Result.Actors.begin(), Result.Actors.end());
    std::sort(Result.SubBodies.begin(), Result.SubBodies.end(),
        [](const auto& A, const auto& B) { return A.StableId < B.StableId; });
    if (!Lease.HoldsExactBoundary(World, Address) || !Lease.ProtectsEscapedMutableViews()) return {};
    return Result;
}

std::unique_ptr<FPreparedNativeKinematicFields> FPreparedNativeKinematicFields::Prepare(
    USpeedMovementComponent& Target, const FSimulationComponentRestoreRequest& Request)
{
    const auto* World = Request.OwnerLease.ResolvePreparingWorld(Request, Request.Address);
    return World ? Prepare(Target, *World, Request) : nullptr;
}

std::unique_ptr<FPreparedNativeKinematicFields> FPreparedNativeKinematicFields::Prepare(
    USpeedWheeledComponent& Target, const FSimulationComponentRestoreRequest& Request)
{
    const auto* World = Request.OwnerLease.ResolvePreparingWorld(Request, Request.Address);
    return World ? Prepare(Target, *World, Request) : nullptr;
}

std::unique_ptr<FPreparedNativeKinematicFields> FPreparedNativeKinematicFields::PrepareSolid(
    USolidSubBody& Target, const FSimulationComponentRestoreRequest& Request,
    const PreparedWorld::FEndpointBinding& Endpoint, const SKinematic& EndpointState)
{
    const auto* World = Request.OwnerLease.ResolvePreparingWorld(Request, Request.Address);
    return World ? PrepareSolid(Target, *World, Request, Endpoint, EndpointState) : nullptr;
}

std::unique_ptr<FPreparedNativeKinematicFields> FPreparedNativeKinematicFields::Prepare(
    USpeedMovementComponent& Target, const FSimulationWorld& World, const FSimulationComponentRestoreRequest& Request)
{
    if (!MatchesActor(World, Request, Target) || !ValidKinematics(Request.Kinematic)) return {};
    auto& Base = Target.BasePhysicsState;
    return std::unique_ptr<FPreparedNativeKinematicFields>(new FPreparedNativeKinematicFields(
        World, Request, Base.Kinematic, &Base.bIsFrozen, Request.Kinematic));
}

std::unique_ptr<FPreparedNativeKinematicFields> FPreparedNativeKinematicFields::Prepare(
    USpeedWheeledComponent& Target, const FSimulationWorld& World, const FSimulationComponentRestoreRequest& Request)
{
    if (!MatchesActor(World, Request, Target) || !ValidKinematics(Request.Kinematic)) return {};
    auto& Base = Target.BasePhysicsState;
    return std::unique_ptr<FPreparedNativeKinematicFields>(new FPreparedNativeKinematicFields(
        World, Request, Base.Kinematic, &Base.bIsFrozen, Request.Kinematic));
}

std::unique_ptr<FPreparedNativeKinematicFields> FPreparedNativeKinematicFields::PrepareSolid(
    USolidSubBody& Target, const FSimulationWorld& World, const FSimulationComponentRestoreRequest& Request,
    const PreparedWorld::FEndpointBinding& Endpoint, const SKinematic& EndpointState)
{
    if (!Request.OwnerLease.HoldsExactBoundary(World, Request.Address) || !Endpoint.StableId
        || !Endpoint.Incarnation || !Endpoint.NativeObjectId || Endpoint.Actor != Request.Supplement.Binding.StableId
        || Endpoint.ActorIncarnation != Request.Supplement.Binding.Incarnation
        || World.SolidSubBodiesByStableId.FindRef(Endpoint.StableId) != &Target
        || World.FindStableSubBodyId(&Target) != Endpoint.StableId
        || Target.GetUniqueID() != Endpoint.NativeObjectId || !Target.GetParentComponent()
        || !MatchesActor(World, Request, *Target.GetParentComponent()) || !ValidKinematics(EndpointState)) return {};
    return std::unique_ptr<FPreparedNativeKinematicFields>(new FPreparedNativeKinematicFields(
        World, Request, Target.Kinematics, nullptr, EndpointState));
}

FPreparedNativeKinematicFields::FPreparedNativeKinematicFields(const FSimulationWorld& InWorld,
    const FSimulationComponentRestoreRequest& Request, SKinematic& Live, bool* InFrozen, const SKinematic& InState)
    : World(InWorld), Address(Request.Address), Lease(Request.OwnerLease), LiveState(Live),
      LiveFrozen(InFrozen), State(InState), Frozen(Request.bFrozen) {}

FPreparedNativeKinematicFields::~FPreparedNativeKinematicFields() noexcept
{
    if (Phase == EPhase::Finalized) std::terminate();
}

bool FPreparedNativeKinematicFields::AppendPreparedAccess(std::vector<PreparedAccess::FParticipantAccess>& Out) const
{
    if ((Phase != EPhase::Prepared && Phase != EPhase::Ready) || !Lease.HoldsExactBoundary(World, Address))
        return false;
    PreparedAccess::FParticipantAccess Part;
    Part.Participant = reinterpret_cast<std::uintptr_t>(this);
    Part.Writes = {KinematicFieldSpan(&LiveState, sizeof(LiveState)), KinematicFieldSpan(&Phase, sizeof(Phase))};
    if (LiveFrozen) Part.Writes.push_back(KinematicFieldSpan(LiveFrozen, sizeof(bool)));
    Part.ReadOnly = {KinematicFieldSpan(&State, sizeof(State)), KinematicFieldSpan(&Frozen, sizeof(Frozen)),
        KinematicFieldSpan(&LiveFrozen, sizeof(LiveFrozen))};
    Out.push_back(std::move(Part));
    return true;
}

bool FPreparedNativeKinematicFields::PrepareFinalization() noexcept
{
    if ((Phase != EPhase::Prepared && Phase != EPhase::Ready) || !Lease.HoldsExactBoundary(World, Address))
        return false;
    Phase = EPhase::Ready;
    return true;
}

void FPreparedNativeKinematicFields::Finalize() noexcept
{
    static_assert(std::is_nothrow_copy_assignable_v<SKinematic>);
    if (Phase != EPhase::Ready) std::terminate();
    LiveState = State;
    if (LiveFrozen) *LiveFrozen = Frozen;
    Phase = EPhase::Finalized;
}

void FPreparedNativeKinematicFields::AbortBeforeFinalize() noexcept
{
    if (Phase == EPhase::Aborted) return;
    if (Phase != EPhase::Prepared && Phase != EPhase::Ready) std::terminate();
    Phase = EPhase::Aborted;
}

void FPreparedNativeKinematicFields::ReleaseAfterPublication() noexcept
{
    if (Phase != EPhase::Finalized) std::terminate();
    Phase = EPhase::Released;
}
}
