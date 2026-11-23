#include "ExactRemoteRoundReset.h"

namespace Speed
{
std::optional<FExactRemoteRoundBuild::FRoster> FExactRemoteRoundBuild::CaptureRoster(
    const FSimulationWorld& World, const FCompletedSimulationPublication& Current)
{
    if (!World.StorageAccess.IsReservationOwner() || World.RoundBuildBoundary != &Current
        || World.bOrderDirty || World.IsRestoreBoundaryReserved() || !Current.World().PublicationSerial) return {};

    const PreparedWorld::FLimits Limits;
    if (World.Bodies.IsEmpty() || static_cast<std::size_t>(World.Bodies.Num()) > Limits.MaxBodies
        || static_cast<std::size_t>(World.StableSubBodyIds.Num()) > Limits.MaxEndpoints
        || World.Bodies.Num() != World.AdaptersByStableId.Num()
        || World.StableSubBodyIds.Num() != World.SubBodyNativeObjectIds.Num()) return {};

    FRoster Result;
    Result.HasStaticGeometryAuthority = World.StaticCollisionWorld.Get() != nullptr;
    Result.Actors.reserve(World.Bodies.Num());
    Result.SubBodies.reserve(World.StableSubBodyIds.Num());
    for (const auto& Body : World.Bodies)
    {
        if (!Body.StableId || !Body.Adapter || World.AdaptersByStableId.FindRef(Body.StableId) != Body.Adapter) return {};
        Result.Actors.push_back(Body.StableId);
    }

    int32 Solids = 0;
    for (const auto& Entry : World.StableSubBodyIds)
    {
        const uint64 Id = Entry.Value;
        const uint64 Actor = Id >> 16;
        const uint32 NativeId = World.SubBodyNativeObjectIds.FindRef(Id);
        const auto* Solid = World.SolidSubBodiesByStableId.FindRef(Id);
        if (!Entry.Key || !Actor || !NativeId || !World.AdaptersByStableId.FindRef(Actor)
            || (Solid && Solid != Entry.Key)) return {};
        if (Solid) ++Solids;
        Result.SubBodies.push_back({Id, Actor, NativeId, Solid != nullptr});
    }
    if (Solids != World.SolidSubBodiesByStableId.Num()) return {};
    std::sort(Result.Actors.begin(), Result.Actors.end());
    std::sort(Result.SubBodies.begin(), Result.SubBodies.end(),
        [](const auto& A, const auto& B) { return A.StableId < B.StableId; });
    return Result;
}

std::optional<FExactRemoteRoundBuild::FReplacement> FExactRemoteRoundBuild::BuildReplacement(
    const FSimulationWorld& World, const FCompletedSimulationPublication& Current,
    PreparedWorld::FRestoreEnvelope Envelope, const std::vector<FActorReplacement>& Actors,
    const std::vector<PreparedWorld::FContactRemoval>& ContactsToRemove)
{
    const auto Roster = CaptureRoster(World, Current);
    if (!Roster || Envelope.Actors.size() != Roster->Actors.size() || Actors.size() > Roster->Actors.size()) return {};
    for (std::size_t I = 0; I < Roster->Actors.size(); ++I)
        if (Envelope.Actors[I].StableId != Roster->Actors[I]) return {};

    std::size_t NextEndpoint = 0;
    for (const auto& Row : Roster->SubBodies)
    {
        if (!Row.IsSolidEndpoint) continue;
        if (NextEndpoint == Envelope.Endpoints.size()) return {};
        const auto& Binding = Envelope.Endpoints[NextEndpoint++];
        if (Binding.StableId != Row.StableId || Binding.Actor != Row.ActorStableId
            || Binding.NativeObjectId != Row.NativeObjectId) return {};
    }
    if (NextEndpoint != Envelope.Endpoints.size()) return {};

    std::vector<PreparedWorld::FBodyReplacement> Replacements;
    Replacements.reserve(Actors.size());
    for (const auto& Actor : Actors)
    {
        const auto& K = Actor.Kinematic;
        if (!K.Rotation.IsNormalized()) return {};
        Replacements.push_back({Actor.StableId, {
            K.Location.X, K.Location.Y, K.Location.Z, K.Velocity.X, K.Velocity.Y, K.Velocity.Z,
            K.Acceleration.X, K.Acceleration.Y, K.Acceleration.Z, K.Rotation.X, K.Rotation.Y, K.Rotation.Z, K.Rotation.W,
            K.AngularVelocity.X, K.AngularVelocity.Y, K.AngularVelocity.Z,
            K.AngularAcceleration.X, K.AngularAcceleration.Y, K.AngularAcceleration.Z}, Actor.bFrozen});
    }

    const auto& Before = Current.World();
    const PreparedWorld::FSnapshotView Source{Before.Payload.GetData(), static_cast<std::size_t>(Before.Payload.Num()),
        Before.NumFrame, Before.InputJournalHash, Before.StateHash, Before.PublicationSerial};
    auto Built = PreparedWorld::BuildReplacement(Source, std::move(Envelope), Replacements, ContactsToRemove);
    if (!Built) return {};

    FReplacement Result;
    Result.Snapshot.NumFrame = Built->Envelope.Address.Frame;
    Result.Snapshot.InputJournalHash = Built->Envelope.Address.InputHash;
    Result.Snapshot.StateHash = Built->Envelope.Address.StateHash;
    Result.Snapshot.Payload.Append(Built->Payload.data(), static_cast<int32>(Built->Payload.size()));
    Result.Envelope = std::make_shared<const PreparedWorld::FRestoreEnvelope>(std::move(Built->Envelope));
    return Result;
}
}
