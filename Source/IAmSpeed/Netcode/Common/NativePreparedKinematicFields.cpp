#include "NativePreparedKinematicFields.h"

#include "IAmSpeed/Components/SpeedMovementComponent.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/SubBodies/Sensor/SensorSubBody.h"
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
    return Request.OwnerLease.MatchesPreparingTarget(Request, Request.Address, World, Target) && Binding.Valid()
        && World.AdaptersByStableId.FindRef(Binding.StableId) == &Target
        && Target.GetPublishedSimulationStableId() == Binding.StableId;
}

std::optional<FPreparedNativeKinematicFields::FRoster> FPreparedNativeKinematicFields::CaptureRoster(
    const FSimulationWorld& World, const PreparedWorld::FAddress& Address, const ISimulationRestoreBoundaryLease& Lease)
{
    if (!Address.Valid() || !Lease.HoldsExactBoundary(World, Address) || !Lease.ProtectsEscapedMutableViews()
        || World.bOrderDirty || World.IsRestoreBoundaryReserved()) return {};

    const PreparedWorld::FLimits Limits;
    if (static_cast<std::size_t>(World.Bodies.Num()) > Limits.MaxBodies
        || static_cast<std::size_t>(World.StableSubBodyIds.Num()) > Limits.MaxEndpoints
        || World.Bodies.Num() != World.AdaptersByStableId.Num()
        || World.StableSubBodyIds.Num() != World.SubBodiesByStableId.Num()
        || World.StableSubBodyIds.Num() != World.SubBodyNativeObjectIds.Num()) return {};

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
        if (!Entry.Key || !Entry.Value || World.SubBodiesByStableId.FindRef(Entry.Value) != Entry.Key
            || World.SubBodyNativeObjectIds.FindRef(Entry.Value) != Entry.Key->GetUniqueID()
            || !Entry.Key->GetParentComponent()) return {};
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

std::optional<std::vector<FPreparedNativeKinematicFields::FRoster::FSubBody>>
FPreparedNativeKinematicFields::CaptureBoundSubBodies(const FSimulationComponentRestoreRequest& Request)
{
    const auto* World = Request.OwnerLease.ResolvePreparingWorld(Request, Request.Address);
    const auto* Actor = Request.OwnerLease.ResolvePreparingTarget(Request, Request.Address);
    if (!World || !Actor || !MatchesActor(*World, Request, *Actor)) return {};

    const PreparedWorld::FLimits Limits;
    const uint64 ActorId = Request.Supplement.Binding.StableId;
    if (ActorId > (UINT64_MAX >> 16) || static_cast<std::size_t>(World->StableSubBodyIds.Num()) > Limits.MaxEndpoints
        || World->StableSubBodyIds.Num() != World->SubBodiesByStableId.Num()) return {};

    const auto& Children = Actor->GetSubBodies();
    if (static_cast<std::size_t>(Children.Num()) > Limits.MaxEndpoints) return {};
    std::vector<FRoster::FSubBody> Rows;
    Rows.reserve(Children.Num());
    for (int32 Index = 0; Index < Children.Num(); ++Index)
    {
        const uint64 Id = (ActorId << 16) | static_cast<uint16>(Index);
        const auto* Child = Children[Index];
        const auto NativeId = World->SubBodyNativeObjectIds.FindRef(Id);
        const auto* Solid = World->SolidSubBodiesByStableId.FindRef(Id);
        if (!Child || !NativeId || World->FindStableSubBodyId(Child) != Id || World->SubBodiesByStableId.FindRef(Id) != Child
            || Child->GetUniqueID() != NativeId || Child->GetParentComponent() != Actor
            || (Solid && Solid != Child) || (!Solid && !Child->IsA<USensorSubBody>())) return {};
        Rows.push_back({Id, ActorId, NativeId, Solid != nullptr});
    }
    std::size_t Registered = 0;
    for (const auto& Entry : World->StableSubBodyIds)
        if ((Entry.Value >> 16) == ActorId) ++Registered;
    if (Registered != Rows.size()
        || !Request.OwnerLease.MatchesPreparingTarget(Request, Request.Address, *World, *Actor)) return {};
    return Rows;
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

std::unique_ptr<FPreparedNativeKinematicFields> FPreparedNativeKinematicFields::PrepareSolid(
    const FSimulationComponentRestoreRequest& Request, const PreparedWorld::FEndpointBinding& Endpoint,
    const SKinematic& EndpointState)
{
    const auto* World = Request.OwnerLease.ResolvePreparingWorld(Request, Request.Address);
    auto* Target = World ? ResolveBoundSolid(*World, Request, Endpoint) : nullptr;
    return Target ? PrepareSolid(*Target, *World, Request, Endpoint, EndpointState) : nullptr;
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
    if (!MatchesSolid(World, Request, Target, Endpoint) || !ValidKinematics(EndpointState)) return {};
    return std::unique_ptr<FPreparedNativeKinematicFields>(new FPreparedNativeKinematicFields(
        World, Request, Target.Kinematics, nullptr, EndpointState));
}

bool FPreparedNativeKinematicFields::MatchesSolid(const FSimulationWorld& World,
    const FSimulationComponentRestoreRequest& Request, const USolidSubBody& Target, const PreparedWorld::FEndpointBinding& Endpoint)
{
    return Request.OwnerLease.ResolvePreparingWorld(Request, Request.Address) == &World
        && Request.OwnerLease.MatchesPreparingEndpoint(Request, Request.Address, Endpoint) && Endpoint.StableId
        && Endpoint.Incarnation && Endpoint.NativeObjectId && Endpoint.Actor == Request.Supplement.Binding.StableId
        && Endpoint.ActorIncarnation == Request.Supplement.Binding.Incarnation
        && World.SolidSubBodiesByStableId.FindRef(Endpoint.StableId) == &Target
        && World.SubBodiesByStableId.FindRef(Endpoint.StableId) == &Target
        && World.FindStableSubBodyId(&Target) == Endpoint.StableId && Target.GetUniqueID() == Endpoint.NativeObjectId
        && Target.GetParentComponent() && MatchesActor(World, Request, *Target.GetParentComponent());
}

bool FPreparedNativeKinematicFields::MatchesSensor(const FSimulationWorld& World,
    const FSimulationComponentRestoreRequest& Request, const USensorSubBody& Target, const FRoster::FSubBody& Sensor)
{
    const auto& Binding = Request.Supplement.Binding;
    if (Request.OwnerLease.ResolvePreparingWorld(Request, Request.Address) != &World || !Binding.Valid()
        || !Sensor.StableId || !Sensor.NativeObjectId || Sensor.IsSolidEndpoint
        || Sensor.ActorStableId != Binding.StableId || (Sensor.StableId >> 16) != Binding.StableId
        || World.SolidSubBodiesByStableId.Contains(Sensor.StableId)
        || World.SubBodiesByStableId.FindRef(Sensor.StableId) != &Target
        || World.FindStableSubBodyId(&Target) != Sensor.StableId
        || World.SubBodyNativeObjectIds.FindRef(Sensor.StableId) != Sensor.NativeObjectId
        || Target.GetUniqueID() != Sensor.NativeObjectId || !Target.IsSensor()) return false;

    const auto* Parent = Target.GetParentComponent();
    return Parent && MatchesActor(World, Request, *Parent);
}

USolidSubBody* FPreparedNativeKinematicFields::ResolveBoundSolid(const FSimulationWorld& World,
    const FSimulationComponentRestoreRequest& Request, const PreparedWorld::FEndpointBinding& Endpoint)
{
    auto* Solid = World.SolidSubBodiesByStableId.FindRef(Endpoint.StableId);
    return Solid && MatchesSolid(World, Request, *Solid, Endpoint) ? Solid : nullptr;
}

USensorSubBody* FPreparedNativeKinematicFields::ResolveBoundSensor(const FSimulationWorld& World,
    const FSimulationComponentRestoreRequest& Request, const FRoster::FSubBody& Row)
{
    if (Row.IsSolidEndpoint || Row.ActorStableId != Request.Supplement.Binding.StableId) return nullptr;
    auto* Sensor = Cast<USensorSubBody>(World.SubBodiesByStableId.FindRef(Row.StableId));
    return Sensor && MatchesSensor(World, Request, *Sensor, Row) ? Sensor : nullptr;
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
