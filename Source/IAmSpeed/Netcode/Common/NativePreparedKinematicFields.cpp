#include "NativePreparedKinematicFields.h"

#include "IAmSpeed/Components/SpeedMovementComponent.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/SubBodies/Sensor/SensorSubBody.h"
#include "IAmSpeed/World/Simulation/ExactRoundConfiguration.h"
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

bool WriteVector(FExactRoundConfigurationWriter& Out, const FVector& V)
{
    return Out.Double(V.X) && Out.Double(V.Y) && Out.Double(V.Z);
}

uint64 MixConfiguration(uint64 Hash, uint64 Value)
{
    for (unsigned I = 0; I < 8; ++I)
    {
        Hash = (Hash ^ static_cast<uint8>(Value)) * 1099511628211ull;
        Value >>= 8;
    }
    return Hash;
}

uint64 ReadActorConfiguration(const ISpeedComponent& Actor)
{
    FExactRoundConfigurationWriter Extension;
    if (!Actor.AppendExactRoundActorConfiguration(Extension) || !Extension.Hash()) return 0;
    FExactRoundConfigurationWriter Native;
    if (!Native.Unsigned(1) || !Native.Float(Actor.GetPhysMass())
        || !WriteVector(Native, Actor.GetPhysCenterOfMassLocal())
        || !Native.Float(Actor.GetPhysMaxSpeed()) || !Native.Float(Actor.GetPhysMaxAngularSpeed())
        || !Native.Boolean(Actor.IsPrescribedMotion()) || !Native.Boolean(Actor.ShouldPreserveNativeVelocitiesForIntegration())
        || !Native.Unsigned(Extension.Hash())) return 0;
    return Native.Hash();
}

uint64 ReadEndpointConfiguration(const ISpeedComponent& Actor, uint16 Index, const USSubBody& Body)
{
    FExactRoundConfigurationWriter Extension;
    if (!Actor.AppendExactRoundSubBodyConfiguration(Index, Extension) || !Extension.Hash()) return 0;
    FExactRoundConfigurationWriter Native;
    const auto Shape = Body.GetCollisionShape();
    const auto Rotation = Body.GetLocalRotation();
    if (!Rotation.IsNormalized()
        || !Native.Unsigned(1) || !Native.Unsigned(static_cast<uint8>(Body.GetSubBodyType()))
        || !Native.Boolean(Body.IsSensor()) || !Native.Unsigned(Body.GetCollisionChannel())
        || !WriteVector(Native, Body.GetLocalOffset())
        || !Native.Double(Rotation.X) || !Native.Double(Rotation.Y)
        || !Native.Double(Rotation.Z) || !Native.Double(Rotation.W)) return 0;
    for (int32 Channel = 0; Channel < int32(ECC_MAX); ++Channel)
        if (!Native.Unsigned(Body.GetResponseParams().CollisionResponse.GetResponse(ECollisionChannel(Channel)))) return 0;
    if (Shape.IsSphere())
    {
        if (!Native.Unsigned(1) || !Native.Float(Shape.GetSphereRadius())) return 0;
    }
    else if (Shape.IsBox())
    {
        if (!Native.Unsigned(2) || !WriteVector(Native, Shape.GetBox())) return 0;
    }
    else if (Shape.IsCapsule())
    {
        if (!Native.Unsigned(3) || !Native.Float(Shape.GetCapsuleRadius())
            || !Native.Float(Shape.GetCapsuleHalfHeight())) return 0;
    }
    else if (!Shape.IsLine() || !Native.Unsigned(4)) return 0;
    return Native.Unsigned(Extension.Hash()) ? Native.Hash() : 0;
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

    auto Result = CaptureRosterContents(World);
    if (!Lease.HoldsExactBoundary(World, Address) || !Lease.ProtectsEscapedMutableViews()) return {};
    return Result;
}

std::optional<FPreparedNativeKinematicFields::FRoster::FSubBody> FPreparedNativeKinematicFields::ReadSubBodyRow(
    const FSimulationWorld& World, const ISpeedComponent& Actor, uint64 ActorId, uint16 Index, const USSubBody& Child)
{
    const uint64 Id = (ActorId << 16) | Index;
    const uint32 NativeId = World.SubBodyNativeObjectIds.FindRef(Id);
    const auto* Solid = World.SolidSubBodiesByStableId.FindRef(Id);
    if (!NativeId || World.StableSubBodyIds.FindRef(&Child) != Id || World.SubBodiesByStableId.FindRef(Id) != &Child
        || Child.GetUniqueID() != NativeId || Child.GetParentComponent() != &Actor
        || (Solid && Solid != &Child) || (!Solid && !Child.IsA<USensorSubBody>())) return {};

    FRoster::FSubBody Row{Id, ActorId, NativeId, Solid != nullptr};
    if (World.ExactRoundWorkerGeneration)
    {
        Row.Incarnation = World.ReadExactRoundSubBodyIncarnation(Id, Child);
        Row.ConfigurationHash = ReadEndpointConfiguration(Actor, Index, Child);
        if (!Row.Incarnation || !Row.ConfigurationHash) return {};
    }
    return Row;
}

std::optional<FPreparedNativeKinematicFields::FRoster> FPreparedNativeKinematicFields::CaptureRosterContents(
    const FSimulationWorld& World)
{
    const PreparedWorld::FLimits Limits;
    if (static_cast<std::size_t>(World.Bodies.Num()) > Limits.MaxBodies
        || static_cast<std::size_t>(World.StableSubBodyIds.Num()) > Limits.MaxEndpoints
        || World.Bodies.Num() != World.AdaptersByStableId.Num()
        || World.StableSubBodyIds.Num() != World.SubBodiesByStableId.Num()
        || World.StableSubBodyIds.Num() != World.SubBodyNativeObjectIds.Num()
        || (World.ExactRoundWorkerGeneration && World.StableSubBodyIds.Num() != World.ExactRoundSubBodySlots.Num())) return {};

    FRoster Result;
    Result.WorkerGeneration = World.ExactRoundWorkerGeneration;
    Result.HasStaticGeometryAuthority = World.StaticCollisionWorld.Get() != nullptr;
    Result.Actors.reserve(World.Bodies.Num());
    if (Result.WorkerGeneration) Result.ActorConfigurations.reserve(World.Bodies.Num());
    Result.SubBodies.reserve(World.StableSubBodyIds.Num());
    int32 Solids = 0;
    for (const auto& Record : World.Bodies)
    {
        const uint64 ActorId = Record.StableId;
        if (!ActorId || ActorId > (UINT64_MAX >> 16) || !Record.Adapter
            || World.AdaptersByStableId.FindRef(ActorId) != Record.Adapter
            || Record.Adapter->GetPublishedSimulationStableId() != ActorId) return {};
        const auto& Actor = *Record.Adapter;
        const auto& Children = Actor.GetSubBodies();
        if (static_cast<std::size_t>(Children.Num()) > Limits.MaxEndpoints - Result.SubBodies.size()
            || Children.Num() > MAX_uint16 + 1) return {};
        uint64 Configuration = 0;
        if (Result.WorkerGeneration)
        {
            Configuration = ReadActorConfiguration(Actor);
            if (!Configuration) return {};
            Configuration = MixConfiguration(Configuration, Children.Num());
        }
        for (int32 Index = 0; Index < Children.Num(); ++Index)
        {
            const auto* Child = Children[Index];
            const auto Row = Child ? ReadSubBodyRow(World, Actor, ActorId, static_cast<uint16>(Index), *Child) : std::nullopt;
            if (!Row) return {};
            if (Row->IsSolidEndpoint) ++Solids;
            if (Result.WorkerGeneration) Configuration = MixConfiguration(Configuration, Row->ConfigurationHash);
            Result.SubBodies.push_back(*Row);
        }
        if (Result.WorkerGeneration)
        {
            if (!Configuration) return {};
            Result.ActorConfigurations.push_back({ActorId, Configuration});
        }
        Result.Actors.push_back(ActorId);
    }
    if (Solids != World.SolidSubBodiesByStableId.Num()
        || Result.SubBodies.size() != static_cast<std::size_t>(World.StableSubBodyIds.Num())) return {};
    std::sort(Result.Actors.begin(), Result.Actors.end());
    std::sort(Result.ActorConfigurations.begin(), Result.ActorConfigurations.end(),
        [](const auto& A, const auto& B) { return A.StableId < B.StableId; });
    std::sort(Result.SubBodies.begin(), Result.SubBodies.end(),
        [](const auto& A, const auto& B) { return A.StableId < B.StableId; });
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
        const auto* Child = Children[Index];
        const auto Row = Child ? ReadSubBodyRow(*World, *Actor, ActorId, static_cast<uint16>(Index), *Child) : std::nullopt;
        if (!Row) return {};
        Rows.push_back(*Row);
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
        && (!World.ExactRoundWorkerGeneration
            || World.ReadExactRoundSubBodyIncarnation(Endpoint.StableId, Target) == Endpoint.Incarnation)
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
    if (!Parent || !MatchesActor(World, Request, *Parent)) return false;
    if (!World.ExactRoundWorkerGeneration) return true;
    const auto Row = ReadSubBodyRow(World, *Parent, Binding.StableId,
        static_cast<uint16>(Sensor.StableId & MAX_uint16), Target);
    return Row && *Row == Sensor;
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
