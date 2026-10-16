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
#endif
