#include "ExactRemoteRoundReset.h"

namespace Speed
{
std::optional<FExactRoundNativeAddress> FExactRemoteRoundBuild::CaptureNativeAddress(
    const FSimulationWorld& World, const FCompletedSimulationPublication& Current) noexcept
{
    if (!World.StorageAccess.IsReservationOwner() || World.RoundBuildBoundary != &Current
        || World.bOrderDirty || World.IsRestoreBoundaryReserved() || !Current.Source()
        || Current.Source()->WorkerGeneration() != World.ExactRoundWorkerGeneration
        || !(Current.ExactRoundOrigin() == World.ReadExactRoundOrigin())) return {};
    const auto& Snapshot = Current.World();
    return FExactRoundNativeAddress::FromCompleted(Current.ExactRoundOrigin(), Snapshot.NumFrame,
        Snapshot.InputJournalHash, Snapshot.PublicationSerial);
}

std::optional<FExactRoundStaticRouteWitness> FExactRemoteRoundBuild::CaptureStaticCollisionRoute(
    const FSimulationWorld& World, const FCompletedSimulationPublication& Current)
{
    if (!CaptureNativeAddress(World, Current) || !Current.ContactSettings || !Current.StaticRoute)
        return {};
    const auto Actual = World.CaptureExactRoundStaticRoute(Current.ContactSettings->StaticCollision);
    return Actual == Current.StaticRoute ? Actual : std::nullopt;
}

std::optional<FExactRemoteRoundBuild::FRoster> FExactRemoteRoundBuild::CaptureRoster(
    const FSimulationWorld& World, const FCompletedSimulationPublication& Current)
{
    if (!CaptureNativeAddress(World, Current) || World.Bodies.IsEmpty()) return {};
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
    const auto Native = CaptureNativeAddress(World, Current);
    const auto Roster = CaptureRoster(World, Current);
    if (!Native || !Roster || !Envelope.Address.ResumeGeneration
        || !Envelope.ColdCarBumps.empty() || Actors.size() > Roster->Actors.size()
        || !MatchesBindings(*Roster, Envelope)) return {};
    Native->Stamp(Envelope.Address);

    std::vector<PreparedWorld::FBodyReplacement> Replacements;
    Replacements.reserve(Actors.size());
    for (const auto& Actor : Actors)
    {
        const auto& K = Actor.Kinematic;
        if (!K.Rotation.IsNormalized()) return {};
        if (Actor.bColdCarBump)
        {
            const auto Source = FPreparedNativeRoundFields::ReadCarBumpSource(World, Actor.StableId);
            if (!Source) return {};
            Envelope.ColdCarBumps.push_back(*Source);
        }
        Replacements.push_back({Actor.StableId, {
            K.Location.X, K.Location.Y, K.Location.Z, K.Velocity.X, K.Velocity.Y, K.Velocity.Z,
            K.Acceleration.X, K.Acceleration.Y, K.Acceleration.Z, K.Rotation.X, K.Rotation.Y, K.Rotation.Z, K.Rotation.W,
            K.AngularVelocity.X, K.AngularVelocity.Y, K.AngularVelocity.Z,
            K.AngularAcceleration.X, K.AngularAcceleration.Y, K.AngularAcceleration.Z}, Actor.bFrozen});
    }
    std::sort(Envelope.ColdCarBumps.begin(), Envelope.ColdCarBumps.end(),
        [](const auto& A, const auto& B) { return A.Actor < B.Actor; });

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

bool FExactRemoteRoundBuild::ValidateColdCarBumpSource(const FSimulationWorld& World,
    const FCompletedSimulationPublication& Current, const PreparedWorld::FRestoreEnvelope& Envelope)
{
    if (Envelope.ColdCarBumps.empty()) return true;
    const auto Native = CaptureNativeAddress(World, Current);
    if (!Native || !Native->Matches(Envelope.Address) || !PreparedWorld::ValidEnvelope(Envelope)) return false;
    for (const auto& Bump : Envelope.ColdCarBumps)
    {
        const auto Actual = FPreparedNativeRoundFields::ReadCarBumpSource(World, Bump.Actor);
        if (!Actual || !(*Actual == Bump)) return false;
    }
    const auto& Before = Current.World();
    const auto Parsed = PreparedWorld::ParseSnapshot(Before.Payload.GetData(), static_cast<std::size_t>(Before.Payload.Num()),
        Before.NumFrame, Before.InputJournalHash, Before.StateHash, Envelope.Actors);
    return Parsed && PreparedWorld::MatchesCarBumpPayloads(*Parsed, Envelope.ColdCarBumps, false);
}

bool FExactRemoteRoundBuild::MatchesColdCarBumpPublication(const FSimulationSnapshot& Snapshot,
    const PreparedWorld::FRestoreEnvelope& Envelope)
{
    if (Envelope.ColdCarBumps.empty()) return true;
    if (Snapshot.NumFrame != Envelope.Address.Next) return false;
    const auto Parsed = PreparedWorld::ParseSnapshot(Snapshot.Payload.GetData(), static_cast<std::size_t>(Snapshot.Payload.Num()),
        Snapshot.NumFrame, Snapshot.InputJournalHash, Snapshot.StateHash, Envelope.Actors);
    return Parsed && PreparedWorld::MatchesCarBumpPayloads(*Parsed, Envelope.ColdCarBumps, true);
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
