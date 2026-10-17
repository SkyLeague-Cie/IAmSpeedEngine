#if WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "Engine/NetConnection.h"
#include "Engine/PackageMapClient.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "GameMapsSettings.h"
#include "Misc/AutomationTest.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/ScopeExit.h"
#include "Serialization/MemoryReader.h"
#include "Serialization/MemoryWriter.h"
#include "Tests/NetTestHelpers.h"
#include "UObject/CoreNet.h"
#include "UObject/StrongObjectPtr.h"

// Actual connected Engine PackageMaps, with controlled GUID resolution. The
// state archive is copied locally: this is not a wire-delivery or rollback test.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCarBumpMappedReferenceTest,
    "IAmSpeed.Collision.CarBump.MappedReferenceRetryContract",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FCarBumpMappedReferenceTest::RunTest(const FString& Parameters)
{
    FString Multihome;
    if (!TestTrue(TEXT("Automated fixture requires explicit loopback socket binding"),
        FParse::Value(FCommandLine::Get(), TEXT("MULTIHOME="), Multihome) && Multihome == TEXT("127.0.0.1")) ||
        !TestNotNull(TEXT("Engine exists"), GEngine)) return false;

    const FString OldMap = UGameMapsSettings::GetGameDefaultMap();
    const FString OldMode = UGameMapsSettings::GetGlobalDefaultGameMode();
    const auto OldViewportClass = GEngine->GameViewportClientClass;
    FNetDriverDefinition* GameDriver = GEngine->NetDriverDefinitions.FindByPredicate(
        [](const FNetDriverDefinition& Def) { return Def.DefName == NAME_GameNetDriver; });
    if (!TestNotNull(TEXT("Existing game net driver definition"), GameDriver)) return false;
    const FName OldDriverClass = GameDriver->DriverClassName;
    const FName OldFallbackClass = GameDriver->DriverClassNameFallback;
    // URL construction reads this public default. Port zero lets the OS choose
    // a free port; clients use the actual bound port returned by FTestWorlds.
    TGuardValue<int32> PortScope(FURL::UrlConfig.DefaultPort, 0);
    UGameMapsSettings::SetGameDefaultMap(TEXT("/Engine/Maps/Entry"));
    UGameMapsSettings::SetGlobalDefaultGameMode(TEXT("/Script/Engine.GameModeBase"));
    GEngine->GameViewportClientClass = UGameViewportClient::StaticClass();
    GameDriver->DriverClassName = TEXT("/Script/OnlineSubsystemUtils.IpNetDriver");
    GameDriver->DriverClassNameFallback = GameDriver->DriverClassName;
    ON_SCOPE_EXIT
    {
        // FTestWorlds is constructed below and destroyed before these overrides.
        UGameMapsSettings::SetGameDefaultMap(OldMap);
        UGameMapsSettings::SetGlobalDefaultGameMode(OldMode);
        GEngine->GameViewportClientClass = OldViewportClass;
        if (FNetDriverDefinition* Def = GEngine->NetDriverDefinitions.FindByPredicate(
            [](const FNetDriverDefinition& Item) { return Item.DefName == NAME_GameNetDriver; }))
        {
            Def->DriverClassName = OldDriverClass;
            Def->DriverClassNameFallback = OldFallbackClass;
        }
    };

    UE::Net::FTestWorlds Worlds(TEXT("/Engine/Maps/Entry"), TEXT("/Script/Engine.GameModeBase"),
        EReplicationSystem::Generic);
    Worlds.SetDescription(TEXT("Bump mapped-reference codec contract"));
    if (!TestTrue(TEXT("Server loaded"), Worlds.Server.IsLoaded()) ||
        !TestTrue(TEXT("Server acquired an ephemeral port"), Worlds.Server.GetPort() > 0) ||
        !TestTrue(TEXT("Actual client handshake completed"), Worlds.CreateAndConnectClient())) return false;
    UNetDriver* ServerDriver = Worlds.Server.GetNetDriver();
    UNetDriver* ClientDriver = Worlds.Clients[0].GetNetDriver();
    if (!TestNotNull(TEXT("Server driver"), ServerDriver) ||
        !TestNotNull(TEXT("Client driver"), ClientDriver) ||
        !TestEqual(TEXT("One actual server connection"), ServerDriver->ClientConnections.Num(), 1) ||
        !TestNotNull(TEXT("Client server connection"), ClientDriver->ServerConnection.Get())) return false;
    UNetConnection* ServerConnection = ServerDriver->ClientConnections[0];
    if (!TestNotNull(TEXT("Server peer connection"), ServerConnection)) return false;
    auto* ServerMap = Cast<UPackageMapClient>(ServerConnection->PackageMap.Get());
    auto* ClientMap = Cast<UPackageMapClient>(ClientDriver->ServerConnection->PackageMap.Get());
    const auto ServerCache = ServerDriver->GetNetGuidCache();
    const auto ClientCache = ClientDriver->GetNetGuidCache();
    if (!TestNotNull(TEXT("Actual server package map"), ServerMap) ||
        !TestNotNull(TEXT("Actual client package map"), ClientMap) ||
        !TestTrue(TEXT("Both real GUID caches exist"), ServerCache.IsValid() && ClientCache.IsValid()) ||
        !TestTrue(TEXT("Server owns GUID assignment"), ServerCache->IsNetGUIDAuthority()) ||
        !TestFalse(TEXT("Client does not own GUID assignment"), ClientCache->IsNetGUIDAuthority())) return false;

    AActor* ServerOwner = Worlds.Server.GetWorld()->SpawnActor<AActor>();
    AActor* ClientOwner = Worlds.Clients[0].GetWorld()->SpawnActor<AActor>();
    if (!TestNotNull(TEXT("Server victim owner"), ServerOwner) ||
        !TestNotNull(TEXT("Client victim owner"), ClientOwner)) return false;
    ServerOwner->SetReplicates(true);
    TStrongObjectPtr<USpeedWheeledComponent> ServerVictim(NewObject<USpeedWheeledComponent>(ServerOwner));
    TStrongObjectPtr<USpeedWheeledComponent> ClientVictim(NewObject<USpeedWheeledComponent>(ClientOwner));
    ServerVictim->SetIsReplicated(true);
    ClientVictim->SetIsReplicated(true);
    // No registration/ticks: physical vehicle and wheel lifecycle is outside
    // this codec test. The real maps still check object networking support.
    const FNetworkGUID Guid = ServerCache->GetOrAssignNetGUID(ServerVictim.Get());
    if (!TestTrue(TEXT("Real authority assigned a dynamic non-default GUID"),
        Guid.IsValid() && Guid.IsDynamic() && !Guid.IsDefault()) ||
        !TestNull(TEXT("Victim has not been resolved on client"), ClientCache->GetObjectFromNetGUID(Guid, true))) return false;

    FNetworkWheeledSpeedState Saved;
    Saved.SourceLocalFrame = 421;
    Saved.WheeledState.LastCarBumpVictim = ServerVictim.Get();
    Saved.WheeledState.CarBumpCooldownFrames = 75;
    Saved.WheeledState.PhysicalSteerInput = -37;
    bool bSuccess = false;
    FNetBitWriter Writer(ServerMap, 32768);
    if (!TestTrue(TEXT("Active state writes through the authority package map"),
        Saved.NetSerialize(Writer, ServerMap, bSuccess) && bSuccess && !Writer.IsError()) ||
        !TestTrue(TEXT("Actual encoded bits are nonempty"), Writer.GetNumBits() > 0)) return false;

    FNetworkWheeledSpeedState Loaded;
    TStrongObjectPtr<USpeedWheeledComponent> Receiver(NewObject<USpeedWheeledComponent>(ClientOwner));
    FNetworkWheeledSpeedState Prior;
    Prior.SourceLocalFrame = 17;
    Prior.WheeledState.LastCarBumpVictim = ClientVictim.Get();
    Prior.WheeledState.CarBumpCooldownFrames = 37;
    Prior.WheeledState.PhysicalSteerInput = 21;
    AddExpectedMessagePlain(TEXT("[WheeledSpeed] ApplyData (RESIMULATION?) Triggered for frame = 0"), ELogVerbosity::Warning, EAutomationExpectedMessageFlags::Contains, 2);
    Prior.ApplyData(Receiver.Get());
    if (!TestTrue(TEXT("Seed receiver holds an actual valid cooldown"),
        Receiver->IsCarBumpInCooldown(*ClientVictim.Get()))) return false;
    // A stale pointer must be cleared rather than used while the GUID is absent.
    Loaded.WheeledState.LastCarBumpVictim = ServerVictim.Get();
    FNetBitReader Pending(ClientMap, Writer.GetData(), Writer.GetNumBits());
    bSuccess = false;
    TestFalse(TEXT("Unresolved reference requests retry"), Loaded.NetSerialize(Pending, ClientMap, bSuccess));
    TestTrue(TEXT("Unresolved is a successful decode, not malformed data"), bSuccess);
    TestFalse(TEXT("Unresolved reference does not poison archive"), Pending.IsError());
    TestEqual(TEXT("Unresolved decode consumes full state"), Pending.GetPosBits(), Writer.GetNumBits());
    TestFalse(TEXT("Unresolved victim clears stale pointer"), Loaded.WheeledState.LastCarBumpVictim.IsValid());
    TestEqual(TEXT("Pending cooldown survives decoding"), Loaded.WheeledState.CarBumpCooldownFrames, uint16(75));
    TestEqual(TEXT("Fields after reference are read while pending"), int32(Loaded.WheeledState.PhysicalSteerInput), int32(-37));

    TestFalse(TEXT("Pending GUID cannot be applied as a valid physical state"), Loaded.bStateProjectionValid);
    Loaded.ApplyData(Receiver.Get());
    TestTrue(TEXT("Pending decode preserves receiver's prior victim and cooldown"),
        Receiver->IsCarBumpInCooldown(*ClientVictim.Get()));
    TestFalse(TEXT("Pending projection cannot compare equal to itself"), Loaded.CompareData(Loaded));
    FNetworkWheeledSpeedState PendingInterpolation;
    PendingInterpolation.LocalFrame = Loaded.LocalFrame;
    PendingInterpolation.InterpolateData(Loaded, Loaded);
    TestFalse(TEXT("Interpolation preserves pending admission"), PendingInterpolation.bStateProjectionValid);

    TArray<uint8> PendingBytes;
    FMemoryWriter PendingWriter(PendingBytes);
    bool bPendingWritten = true;
    TestFalse(TEXT("Pending state cannot be forwarded as an invented neutral state"),
        Loaded.NetSerialize(PendingWriter, nullptr, bPendingWritten));
    TestFalse(TEXT("Pending save reports failure"), bPendingWritten);
    TestTrue(TEXT("Pending save marks archive error"), PendingWriter.IsError());
    TestEqual(TEXT("Pending save authors no archive bytes"), PendingBytes.Num(), 0);

    // Deliberate controlled resolution using the real client's cache, as actor
    // channel spawning would do. No local raw pointer is put on the wire.
    ClientCache->RegisterNetGUID_Client(Guid, ClientVictim.Get());
    if (!TestTrue(TEXT("GUID now resolves to the distinct client instance"),
        ClientCache->GetObjectFromNetGUID(Guid, true) == ClientVictim.Get() && ClientVictim.Get() != ServerVictim.Get())) return false;
    FNetBitReader Retry(ClientMap, Writer.GetData(), Writer.GetNumBits());
    bSuccess = false;
    TestTrue(TEXT("Same actual encoded state decodes after resolution"),
        Loaded.NetSerialize(Retry, ClientMap, bSuccess) && bSuccess && !Retry.IsError());
    TestEqual(TEXT("Resolved decode consumes full state"), Retry.GetPosBits(), Writer.GetNumBits());
    TestTrue(TEXT("Decoded victim is client object, not authority object"),
        Loaded.WheeledState.LastCarBumpVictim.Get() == ClientVictim.Get());
    TestEqual(TEXT("Resolved cooldown remains exactly 75 ticks"), Loaded.WheeledState.CarBumpCooldownFrames, uint16(75));
    TestEqual(TEXT("Source frame survives retry"), Loaded.SourceLocalFrame, 421);

    TestTrue(TEXT("Resolved retry becomes eligible for physical application"), Loaded.bStateProjectionValid);
    Loaded.ApplyData(Receiver.Get());
    TestTrue(TEXT("Mapped retry applies the client victim"), Receiver->IsCarBumpInCooldown(*ClientVictim.Get()));

    // Exercise the production decoder with an actually truncated byte archive.
    // This archive check is separate from real PackageMap bit framing above.
    auto ArchiveSource = Saved;
    ArchiveSource.WheeledState.LastCarBumpVictim.Reset();
    ArchiveSource.WheeledState.CarBumpCooldownFrames = 0;
    TArray<uint8> ArchiveBytes;
    FMemoryWriter ByteWriter(ArchiveBytes);
    bool bWritten = false;
    ArchiveSource.NetSerialize(ByteWriter, nullptr, bWritten);
    if (!TestTrue(TEXT("Production state writes a complete byte archive"), bWritten && !ByteWriter.IsError())) return false;
    for (const int32 Cut : {0, 1, ArchiveBytes.Num() / 2, ArchiveBytes.Num() - 1})
    {
        auto Destination = Prior;
        TArray<uint8> ShortBytes;
        ShortBytes.Append(ArchiveBytes.GetData(), Cut);
        FMemoryReader Truncated(ShortBytes);
        bool bDecoded = true;
        Destination.NetSerialize(Truncated, nullptr, bDecoded);
        TestFalse(TEXT("Truncated state fails archive decode"), bDecoded);
        TestTrue(TEXT("Truncated state marks archive error"), Truncated.IsError());
        TestEqual(TEXT("Rejected archive preserves source frame"), Destination.SourceLocalFrame, Prior.SourceLocalFrame);
        TestEqual(TEXT("Rejected archive preserves cooldown"), Destination.WheeledState.CarBumpCooldownFrames, Prior.WheeledState.CarBumpCooldownFrames);
        TestEqual(TEXT("Rejected archive preserves later physical input"), int32(Destination.WheeledState.PhysicalSteerInput), int32(Prior.WheeledState.PhysicalSteerInput));
        TestTrue(TEXT("Rejected archive preserves victim identity"), Destination.WheeledState.LastCarBumpVictim == Prior.WheeledState.LastCarBumpVictim);
        TestTrue(TEXT("Rejected archive preserves prior admission"), Destination.bStateProjectionValid == Prior.bStateProjectionValid);
    }

    Saved.WheeledState.LastCarBumpVictim.Reset();
    Saved.WheeledState.CarBumpCooldownFrames = 0;
    FNetBitWriter NeutralWriter(ServerMap, 32768);
    bSuccess = false;
    TestTrue(TEXT("Neutral mapped state writes"), Saved.NetSerialize(NeutralWriter, ServerMap, bSuccess) && bSuccess);
    FNetBitReader NeutralReader(ClientMap, NeutralWriter.GetData(), NeutralWriter.GetNumBits());
    bSuccess = false;
    TestTrue(TEXT("Neutral state replaces resolved active state"),
        Loaded.NetSerialize(NeutralReader, ClientMap, bSuccess) && bSuccess && !NeutralReader.IsError());
    TestTrue(TEXT("Neutral complete decode is eligible"), Loaded.bStateProjectionValid);
    TestFalse(TEXT("Neutral state clears resolved victim"), Loaded.WheeledState.LastCarBumpVictim.IsValid());
    TestEqual(TEXT("Neutral state clears timer"), Loaded.WheeledState.CarBumpCooldownFrames, uint16(0));

    AddInfo(FString::Printf(TEXT("[BumpMappedReference] port=%d guid=%s archiveBits=%lld scope=connected_maps_local_archive_controlled_resolution; see individual assertions"),
        Worlds.Server.GetPort(), *Guid.ToString(), static_cast<long long>(Writer.GetNumBits())));
    return true;
}
#endif
