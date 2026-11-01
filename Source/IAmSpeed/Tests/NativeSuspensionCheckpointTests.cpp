#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Netcode/client-prediction/RestorableSuspension.h"
#include "Misc/AutomationTest.h"
#include <new>

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASNativeSuspensionCheckpoint,
    "IAmSpeed.Netcode.Prediction.NativeSuspensionCheckpoint",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASNativeSuspensionCheckpoint::RunTest(const FString&)
{
    using Speed::Netcode::Prediction::FRestorableSuspension;
    Chaos::FSimpleSuspensionConfig Config;Config.SetSuspensionMaxRaise(10);Config.SetSuspensionMaxDrop(25);
    Config.SpringRate=5000;Config.CompressionDamping=1000;Config.ReboundDamping=1200;Config.SuspensionSmoothing=6;
    Chaos::FSimpleSuspensionSim Native(&Config);
    Native.SetLocalRestingPosition(FVector(70,-45,-15));Native.SetSpringIndex(2);
    for(int32 I=0;I<17;++I)
    {
        Native.SetSuspensionLength(45.f+float(I%7),30.f);Native.SetLocalVelocity(FVector(I,-I,0));
        Native.Simulate(1.f/300.f);Native.GetSpringLength();
    }
    FRestorableSuspension Original(Native),Restored(Native);
    const auto Snapshot=Original.CaptureCheckpoint();
    if(!TestTrue(TEXT("complete native suspension checkpoint captured"),bool(Snapshot)))return false;
    TestTrue(TEXT("initial native copy is exact"),Restored.CaptureCheckpoint()==Snapshot);
    const auto BeforeRead=Original.CaptureCheckpoint();Original.CaptureCheckpoint();
    TestTrue(TEXT("capture never calls mutating GetSpringLength"),Original.CaptureCheckpoint()==BeforeRead);
    Restored.SetSuspensionLength(60,30);Restored.SetLocalVelocity(FVector(9,8,7));Restored.Simulate(1.f/300.f);Restored.GetSpringLength();
    TestTrue(TEXT("native state really changed"),Restored.CaptureCheckpoint()!=Snapshot);
    const auto Plan=Restored.PrepareRestore(*Snapshot);
    if(!TestTrue(TEXT("same config complete restore prepared"),bool(Plan)))return false;
    TestTrue(TEXT("preparation leaves old native state unchanged"),Restored.CaptureCheckpoint()!=Snapshot);
    TestTrue(TEXT("prepared restore commits"),Restored.CommitPrepared(*Plan));
    TestTrue(TEXT("all protected/native fields restored byte exactly"),Restored.CaptureCheckpoint()==Snapshot);
    TestFalse(TEXT("restore token cannot be reused"),Restored.CommitPrepared(*Plan));
    for(int32 I=0;I<240;++I)
    {
        const float Length=40.f+float((I*11)%23);
        Native.SetSuspensionLength(Length,30);Original.SetSuspensionLength(Length,30);Restored.SetSuspensionLength(Length,30);
        Native.Simulate(1.f/300.f);Original.Simulate(1.f/300.f);Restored.Simulate(1.f/300.f);
        TestEqual(TEXT("original Engine force identical"),Original.GetSuspensionForce(),Native.GetSuspensionForce());
        TestEqual(TEXT("restored Engine force identical"),Restored.GetSuspensionForce(),Native.GetSuspensionForce());
        const float ExpectedLength=Native.GetSpringLength();
        TestEqual(TEXT("original Engine smoothing identical"),Original.GetSpringLength(),ExpectedLength);
        TestEqual(TEXT("restored Engine smoothing identical"),Restored.GetSpringLength(),ExpectedLength);
        TestTrue(TEXT("all hidden states remain identical over future steps"),Original.CaptureCheckpoint()==Restored.CaptureCheckpoint());
    }
    const auto Latest=Restored.CaptureCheckpoint();if(!Latest)return false;
    for(size_t N=0;N<Latest->size();++N)
        TestFalse(TEXT("every truncated checkpoint rejected"),bool(Restored.PrepareRestore(Speed::Replay::FBytes(Latest->begin(),Latest->begin()+N))));
    auto Trailing=*Latest;Trailing.push_back(0);TestFalse(TEXT("trailing bytes rejected"),bool(Restored.PrepareRestore(Trailing)));
    auto Bad=*Latest;Bad[4]=2;TestFalse(TEXT("unsupported schema rejected"),bool(Restored.PrepareRestore(Bad)));
    auto OtherConfig=Config;OtherConfig.SpringRate+=1;FRestorableSuspension Other(OtherConfig);
    TestFalse(TEXT("different configuration rejected"),bool(Other.PrepareRestore(*Latest)));
    const auto Stale=Restored.PrepareRestore(*Latest);if(!Stale)return false;
    Restored.SetLastDisplacement(-0.f);
    TestFalse(TEXT("native state mutation invalidates prepared state"),Restored.CommitPrepared(*Stale));
    TestFalse(TEXT("token from another native object refused"),Original.CommitPrepared(*Stale));
    auto UnsafeConfig=Config;UnsafeConfig.SuspensionSmoothing=NUM_SUS_AVERAGING+1;
    FRestorableSuspension Unsafe(UnsafeConfig);TestFalse(TEXT("out-of-bounds Engine smoothing config cannot be checkpointed"),bool(Unsafe.CaptureCheckpoint()));
    UnsafeConfig=Config;UnsafeConfig.SpringRate=std::numeric_limits<float>::quiet_NaN();
    FRestorableSuspension Nonfinite(UnsafeConfig);TestFalse(TEXT("nonfinite configuration rejected"),bool(Nonfinite.CaptureCheckpoint()));
    FRestorableSuspension Declared(Config);const auto DeclaredBytes=Declared.CaptureCheckpoint();if(!DeclaredBytes)return false;
    const auto DeclaredPlan=Declared.PrepareRestore(*DeclaredBytes);if(!DeclaredPlan)return false;
    std::vector<Speed::PreparedAccess::FParticipantAccess> Access;
    TestTrue(TEXT("actual complete suspension field ranges declared"),Declared.AppendPreparedAccess(*DeclaredPlan,17,Access));
    TestTrue(TEXT("all real mutable ranges disjoint from retained config"),Speed::PreparedAccess::ValidateGlobalAccess(Access));
    auto Alias=Access.front();Alias.Participant=18;Alias.Writes={Access.front().ReadOnly.front()};Alias.ReadOnly.clear();Access.push_back(Alias);
    TestFalse(TEXT("writer into immutable configuration rejected globally"),Speed::PreparedAccess::ValidateGlobalAccess(Access));
    alignas(FRestorableSuspension) uint8 Storage[sizeof(FRestorableSuspension)];
    auto* First=new(Storage)FRestorableSuspension(Config);const auto FirstBytes=First->CaptureCheckpoint();if(!FirstBytes)return false;
    const auto OldPlan=First->PrepareRestore(*FirstBytes);if(!OldPlan)return false;
    First->~FRestorableSuspension();auto* Replacement=new(Storage)FRestorableSuspension(Config);
    TestFalse(TEXT("same-address native lifetime ABA cannot accept old token"),Replacement->CommitPrepared(*OldPlan));
    Replacement->~FRestorableSuspension();
    AddInfo(TEXT("Native suspension subplan only: original Engine algorithm, fixed delta, full hidden state and smoothing. No client physics world, world restore lease, input transaction or network match qualification."));
    return !HasAnyErrors();
}
#endif
