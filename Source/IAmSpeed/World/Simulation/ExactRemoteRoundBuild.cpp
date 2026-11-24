#include "ExactRemoteRoundReset.h"

namespace Speed
{
std::optional<FExactRemoteRoundBuild::FRoster> FExactRemoteRoundBuild::CaptureRoster(
    const FSimulationWorld& World, const FCompletedSimulationPublication& Current)
{
    if (!World.StorageAccess.IsReservationOwner() || World.RoundBuildBoundary != &Current
        || World.bOrderDirty || World.IsRestoreBoundaryReserved() || !Current.World().PublicationSerial
        || World.Bodies.IsEmpty() || !Current.Source() || !World.ExactRoundWorkerGeneration
        || Current.Source()->WorkerGeneration() != World.ExactRoundWorkerGeneration) return {};
    return FPreparedNativeKinematicFields::CaptureRosterContents(World);
}

bool FExactRemoteRoundBuild::MatchesBindings(const FRoster& Roster,
    const PreparedWorld::FRestoreEnvelope& Envelope) noexcept
{
    if (!Roster.WorkerGeneration || Envelope.Actors.size() != Roster.Actors.size()
        || Roster.ActorConfigurations.size() != Roster.Actors.size()) return false;
    for (std::size_t I = 0; I < Roster.Actors.size(); ++I)
    {
        const auto& Binding = Envelope.Actors[I];
        const auto& Row = Roster.ActorConfigurations[I];
        if (!Binding.Valid() || Binding.StableId != Roster.Actors[I] || Binding.StableId != Row.StableId
            || Binding.ConfigurationHash != Row.ConfigurationHash) return false;
    }

    std::size_t NextEndpoint = 0;
    for (const auto& Row : Roster.SubBodies)
    {
        if (!Row.IsSolidEndpoint) continue;
        if (NextEndpoint == Envelope.Endpoints.size()) return false;
        const auto& Binding = Envelope.Endpoints[NextEndpoint++];
        const auto Actor = std::lower_bound(Envelope.Actors.begin(), Envelope.Actors.end(), Row.ActorStableId,
            [](const auto& Entry, uint64 ActorId) { return Entry.StableId < ActorId; });
        if (Binding.StableId != Row.StableId || Binding.Actor != Row.ActorStableId
            || Binding.NativeObjectId != Row.NativeObjectId || Binding.Incarnation != Row.Incarnation
            || Actor == Envelope.Actors.end() || Actor->StableId != Binding.Actor
            || Binding.ActorIncarnation != Actor->Incarnation) return false;
    }
    return NextEndpoint == Envelope.Endpoints.size();
}

std::optional<FExactRemoteRoundBuild::FReplacement> FExactRemoteRoundBuild::BuildReplacement(
    const FSimulationWorld& World, const FCompletedSimulationPublication& Current,
    PreparedWorld::FRestoreEnvelope Envelope, const std::vector<FActorReplacement>& Actors,
    const std::vector<PreparedWorld::FContactRemoval>& ContactsToRemove)
{
    const auto Roster = CaptureRoster(World, Current);
    if (!Roster || Actors.size() > Roster->Actors.size() || !MatchesBindings(*Roster, Envelope)) return {};

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

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedExactRoundBindingValuesTest,
    "IAmSpeed.Netcode.RoundReset.BindingValues",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedExactRoundBindingValuesTest::RunTest(const FString&)
{
    using FBuild = Speed::FExactRemoteRoundBuild;
    FBuild::FRoster Roster;
    Roster.WorkerGeneration = 3;
    Roster.Actors = {4};
    Roster.ActorConfigurations = {{4, 17}};
    const uint64 Slot = uint64{4} << 16;
    Roster.SubBodies = {{Slot, 4, 25, true, 28, 31}, {Slot + 1, 4, 26, false, 29, 32}};

    Speed::PreparedWorld::FRestoreEnvelope Envelope;
    Envelope.Actors = {{4, 100, 17, 1, 1}};
    Envelope.Endpoints = {{Slot, 28, 4, 100, 25}};
    TestTrue(TEXT("Registered local endpoints and wire actor join"), FBuild::MatchesBindings(Roster, Envelope));
    ++Roster.SubBodies[0].Incarnation;
    TestFalse(TEXT("Reused slot and UObject index reject an earlier incarnation"), FBuild::MatchesBindings(Roster, Envelope));
    Envelope.Endpoints[0].Incarnation = Roster.SubBodies[0].Incarnation;
    TestTrue(TEXT("Rebound exact incarnation joins"), FBuild::MatchesBindings(Roster, Envelope));
    ++Envelope.Endpoints[0].ActorIncarnation;
    TestFalse(TEXT("Foreign actor incarnation refused"), FBuild::MatchesBindings(Roster, Envelope));
    --Envelope.Endpoints[0].ActorIncarnation;
    ++Envelope.Actors[0].ConfigurationHash;
    TestFalse(TEXT("Stale actor configuration refused"), FBuild::MatchesBindings(Roster, Envelope));
    return true;
}
#endif
