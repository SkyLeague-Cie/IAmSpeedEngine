#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/Base/CarBumpSnapshot.h"
#include "Misc/AutomationTest.h"
#include "Serialization/MemoryReader.h"
#include "Serialization/MemoryWriter.h"
#include "UObject/StrongObjectPtr.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCarBumpStateTest,
    "IAmSpeed.Collision.CarBump.StateAndNetEnvelope",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FCarBumpStateTest::RunTest(const FString& Parameters)
{
    TStrongObjectPtr<USpeedWheeledComponent> A(NewObject<USpeedWheeledComponent>());
    TStrongObjectPtr<USpeedWheeledComponent> B(NewObject<USpeedWheeledComponent>());
    TStrongObjectPtr<USpeedWheeledComponent> C(NewObject<USpeedWheeledComponent>());
    TestTrue(TEXT("First target admitted"),A->TryBeginCarBump(*B.Get()));
    TestFalse(TEXT("Same target held in cooldown"),A->TryBeginCarBump(*B.Get()));
    TestFalse(TEXT("Cooldown belongs to attacker"),B->IsCarBumpInCooldown(*A.Get()));
    TestTrue(TEXT("Another target admitted"),A->TryBeginCarBump(*C.Get()));
    TestTrue(TEXT("Last target only, rather than per-pair map"),A->TryBeginCarBump(*B.Get()));
    TestFalse(TEXT("Self target rejected"),A->TryBeginCarBump(*A.Get()));
    A->ResetCarBumpCooldown();
    TestFalse(TEXT("Explicit reset clears same target"),A->IsCarBumpInCooldown(*B.Get()));
    TArray<uint8> Neutral;
    A->AppendSimulationSnapshot(Neutral);
    TestEqual(TEXT("Values-only envelope size"),Neutral.Num(),int32(Speed::Collision::CarBumpSnapshotSize));
    TestTrue(TEXT("Neutral canonical state validates without a target"),A->CanRestoreSimulationSnapshot(Neutral));
    TArray<uint8> Empty;
    TestFalse(TEXT("Old/missing mechanic envelope rejects"),A->CanRestoreSimulationSnapshot(Empty));
    auto Missing=Speed::Collision::EncodeCarBumpSnapshot(75,999);
    TestFalse(TEXT("Unregistered target id rejects before restore"),
        A->CanRestoreSimulationSnapshot(MakeArrayView(Missing.data(),int32(Missing.size()))));

    FNetworkWheeledSpeedState Saved,Loaded;
    Saved.WheeledState.LastCarBumpVictim.Reset();Saved.WheeledState.CarBumpCooldownFrames=0;
    TArray<uint8> Bytes;bool bSuccess=false;
    FMemoryWriter Writer(Bytes);
    TestTrue(TEXT("Neutral net state serializes"),Saved.NetSerialize(Writer,nullptr,bSuccess)&&bSuccess&&!Writer.IsError());
    FMemoryReader Reader(Bytes);bSuccess=false;
    TestTrue(TEXT("Neutral net state reads"),Loaded.NetSerialize(Reader,nullptr,bSuccess)&&bSuccess&&!Reader.IsError());
    TestEqual(TEXT("Neutral timer retained"),Loaded.WheeledState.CarBumpCooldownFrames,uint16(0));
    TestFalse(TEXT("Neutral target retained"),Loaded.WheeledState.LastCarBumpVictim.IsValid());
    Saved.WheeledState.LastCarBumpVictim=B.Get();Saved.WheeledState.CarBumpCooldownFrames=75;
    TArray<uint8> UnmappedBytes;FMemoryWriter Unmapped(UnmappedBytes);bSuccess=true;
    TestFalse(TEXT("Active net state requires object package mapping"),Saved.NetSerialize(Unmapped,nullptr,bSuccess));
    TestFalse(TEXT("Missing map never qualifies net state"),bSuccess);
    TestTrue(TEXT("Missing map emits archive error"),Unmapped.IsError());
    return true;
}

// Real world registry and canonical snapshot path, with active mechanic bytes.
// This deliberately does not claim a physical-frame or transport replay.
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "IAmSpeed/World/Simulation/SimulationWorld.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Misc/ScopeExit.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCarBumpCanonicalActiveRestoreTest,
    "IAmSpeed.Collision.CarBump.CanonicalActiveTargetRestore",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FCarBumpCanonicalActiveRestoreTest::RunTest(const FString& Parameters)
{
    const auto Options=UWorld::InitializationValues().AllowAudioPlayback(false)
        .CreatePhysicsScene(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false);
    UWorld* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,
        nullptr,true,ERHIFeatureLevel::Num,&Options);
    if(!TestNotNull(TEXT("Canonical bump world"),World)) return false;
    GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
    auto* Registry=World->GetSubsystem<USpeedWorldSubsystem>();
    AActor* OwnerA=World->SpawnActor<AActor>();
    AActor* OwnerB=World->SpawnActor<AActor>();
    USpeedWheeledComponent* A=OwnerA?NewObject<USpeedWheeledComponent>(OwnerA):nullptr;
    USpeedWheeledComponent* B=OwnerB?NewObject<USpeedWheeledComponent>(OwnerB):nullptr;
    ON_SCOPE_EXIT
    {
        if(Registry)
        {
            if(A) Registry->UnregisterSpeedComponent(A);
            if(B) Registry->UnregisterSpeedComponent(B);
            Registry->ApplyPendingOps();
        }
        if(OwnerA) OwnerA->Destroy();
        if(OwnerB) OwnerB->Destroy();
        World->DestroyWorld(false);
        GEngine->DestroyWorldContext(World);
    };
    if(!TestNotNull(TEXT("Canonical registry"),Registry) ||
       !TestNotNull(TEXT("Attacker component"),A) || !TestNotNull(TEXT("Victim component"),B)) return false;
    A->SetIsFrozen(false);B->SetIsFrozen(false);
    Registry->RegisterSpeedComponent(A);Registry->RegisterSpeedComponent(B);Registry->ApplyPendingOps();
    const uint64 AttackerId=Registry->GetSimulationStableId(*A);
    const uint64 VictimId=Registry->GetSimulationStableId(*B);
    if(!TestTrue(TEXT("Both real registry identities exist and differ"),
        AttackerId!=0 && VictimId!=0 && AttackerId!=VictimId)) return false;
    constexpr uint64 JournalHash=0xBCCD20261016ull;
    const FSimulationSnapshot Neutral=Registry->CaptureSimulationSnapshot(70,JournalHash);
    TestTrue(TEXT("Active target admitted before capture"),A->TryBeginCarBump(*B));
    TArray<uint8> Active;
    A->AppendSimulationSnapshot(Active);
    uint16 Frames=0;uint64 SerializedVictim=0;
    TestTrue(TEXT("Actual active component payload decodes"),Speed::Collision::DecodeCarBumpSnapshot(
        Active.GetData(),Active.Num(),Frames,SerializedVictim));
    TestTrue(TEXT("Active timer is positive"),Frames>0);
    TestEqual(TEXT("Payload carries actual registry victim identity"),SerializedVictim,VictimId);
    TestTrue(TEXT("Actual active target resolves before restore"),A->CanRestoreSimulationSnapshot(Active));
    const auto Self=Speed::Collision::EncodeCarBumpSnapshot(Frames,AttackerId);
    TestFalse(TEXT("A registered self target still rejects"),A->CanRestoreSimulationSnapshot(
        MakeArrayView(Self.data(),int32(Self.size()))));
    const FSimulationSnapshot Saved=Registry->CaptureSimulationSnapshot(71,JournalHash);
    A->ResetCarBumpCooldown();
    TestFalse(TEXT("Mutation really clears captured cooldown"),A->IsCarBumpInCooldown(*B));
    TestFalse(TEXT("Wrong input journal rejects entire canonical restore"),
        Registry->RestoreSimulationSnapshot(Saved,JournalHash^1));
    TestFalse(TEXT("Rejected restore leaves mechanic unchanged"),A->IsCarBumpInCooldown(*B));
    TestTrue(TEXT("Canonical world restores active target and timer"),
        Registry->RestoreSimulationSnapshot(Saved,JournalHash));
    TestTrue(TEXT("Restored target remains in cooldown"),A->IsCarBumpInCooldown(*B));
    TestFalse(TEXT("Restore does not transfer attacker cooldown to victim"),B->IsCarBumpInCooldown(*A));
    TArray<uint8> Restored;
    A->AppendSimulationSnapshot(Restored);
    TestTrue(TEXT("Restored mechanic bytes exactly match capture"),Restored==Active);
    TestTrue(TEXT("Older neutral world snapshot also restores"),
        Registry->RestoreSimulationSnapshot(Neutral,JournalHash));
    TestFalse(TEXT("Neutral rollback clears target cooldown"),A->IsCarBumpInCooldown(*B));
    AddInfo(FString::Printf(TEXT("[BumpCanonicalRestore] attacker=%llu victim=%llu timerFrames=%u payloadBytes=%d"),
        static_cast<unsigned long long>(AttackerId),static_cast<unsigned long long>(VictimId),Frames,Active.Num()));
    return true;
}

#endif
