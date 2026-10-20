#if WITH_DEV_AUTOMATION_TESTS

#include "IAmSpeed/World/Simulation/FramePresentationTasks.h"
#include "IAmSpeed/Camera/SpeedCarCameraPresentation.h"
#include "HAL/Event.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTLS.h"
#include "Misc/AutomationTest.h"
#include <atomic>
#include <stdexcept>
#include <cfenv>

namespace
{
using FProducerRef = TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe>;

class FPresentationTaskProbe final : public ISimulationPresentationProducer
{
public:
    bool bIndependent = true;
    TFunction<void()> Work;
    uint32 CalledThread = 0;
    bool SupportsIndependentFrameProduction() const override { return bIndependent; }
    uint64 OwnerStableId() const override { return 7; }
    uint32 Channel() const override { return 3; }
    void InvalidateTimeline() override {}
    void Produce(const FSimulationSnapshot&, FSimulationPresentationOutput& Output) override
    {
        CalledThread = FPlatformTLS::GetCurrentThreadId();
        if (Work) Work();
        Output.OwnerStableId = 999; Output.Channel = 999;
        Output.NumFrame = 999; Output.PublicationSerial = 999;
        Output.Payload = {1, 7, 9}; Output.StatePayload = {8, 4};
    }
};

bool SameOutput(const FSimulationPresentationOutput& A, const FSimulationPresentationOutput& B)
{
    return A.OwnerStableId == B.OwnerStableId && A.Channel == B.Channel
        && A.NumFrame == B.NumFrame && A.PublicationSerial == B.PublicationSerial
        && A.Payload == B.Payload && A.StatePayload == B.StatePayload;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedPresentationTaskOwnershipTest,
    "IAmSpeed.Simulation.FrameParallel.PresentationOwnership",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)
bool FIAmSpeedPresentationTaskOwnershipTest::RunTest(const FString&)
{
    const uint32 OwnerThread = FPlatformTLS::GetCurrentThreadId();
    auto Producer = MakeShared<FPresentationTaskProbe, ESPMode::ThreadSafe>();
    TArray<FProducerRef> Producers{Producer};
    FSimulationSnapshot Snapshot; Snapshot.NumFrame = 42; Snapshot.StateHash = 113;
    Snapshot.Payload = {2, 3, 5};
    const auto OriginalPhysicalBytes = Snapshot.Payload;

    Producer->bIndependent = false;
    bool bSerialProducerBeforeValidation = false;
    TestTrue(TEXT("unqualified endpoint stays serial"), Speed::FrameParallel::ProduceFramePresentationWithOwnerValidation(
        Snapshot, Producers, 4, true, [&]()
        {
            bSerialProducerBeforeValidation = Producer->CalledThread == OwnerThread;
            return true;
        }));
    TestTrue(TEXT("legacy dispatch remains on owner before validation"), bSerialProducerBeforeValidation);
    TestEqual(TEXT("one canonical output"), Snapshot.PresentationOutputs.Num(), 1);
    const auto& Output = Snapshot.PresentationOutputs[0];
    TestTrue(TEXT("publisher sanitizes worker envelope"), Output.OwnerStableId == 7 && Output.Channel == 3
        && Output.NumFrame == 42 && Output.PublicationSerial == 0);
    TestTrue(TEXT("physical bytes and hash never change"), Snapshot.Payload == OriginalPhysicalBytes && Snapshot.StateHash == 113);

    Producer->bIndependent = true;
    Snapshot.PresentationOutputs.Reset(); Producer->CalledThread = 0;
    TestTrue(TEXT("no independent owner work avoids pointless dispatch"),
        Speed::FrameParallel::ProduceFramePresentationWithOwnerValidation(Snapshot, Producers, 4, false, [] { return true; }));
    TestEqual(TEXT("empty validation work stays on owner"), Producer->CalledThread, OwnerThread);

    if (!TestTrue(TEXT("owned worker must be available; no Unreal scheduler prerequisite"),
        Speed::EnsurePrivateFrameWorkers(Speed::FFrameParallelPolicy::MaximumLanes - 1) > 0)) return false;
    FEvent* Entered = FPlatformProcess::GetSynchEventFromPool(true);
    FEvent* Release = FPlatformProcess::GetSynchEventFromPool(true);
    std::atomic<bool> Finished{false};
    bool bEnteredWhileValidating = false, bFinishedBeforeRelease = true;
    uint32 ValidationThread = 0;
    Producer->Work = [&]
    {
        Entered->Trigger();
        if (!Release->Wait(2000)) throw std::runtime_error("validation did not release presentation task");
        Finished.store(true, std::memory_order_release);
    };
    Snapshot.PresentationOutputs.Reset();
    const bool bResult = Speed::FrameParallel::ProduceFramePresentationWithOwnerValidation(
        Snapshot, Producers, 2, true, [&]
        {
            ValidationThread = FPlatformTLS::GetCurrentThreadId();
            bEnteredWhileValidating = Entered->Wait(2000);
            bFinishedBeforeRelease = Finished.load(std::memory_order_acquire);
            Release->Trigger();
            return false; // A validation failure still joins the producer.
        });
    FPlatformProcess::ReturnSynchEventToPool(Release);
    FPlatformProcess::ReturnSynchEventToPool(Entered);
    TestFalse(TEXT("validation failure is preserved"), bResult);
    TestTrue(TEXT("camera and owner validation really overlap"), bEnteredWhileValidating && !bFinishedBeforeRelease);
    TestTrue(TEXT("producer is joined before return"), Finished.load(std::memory_order_acquire));
    TestTrue(TEXT("only endpoint moves to worker"), Producer->CalledThread != OwnerThread);
    TestEqual(TEXT("input validation retains its owner thread"), ValidationThread, OwnerThread);
    TestTrue(TEXT("physical bytes remain immutable after actual overlap"), Snapshot.Payload == OriginalPhysicalBytes && Snapshot.StateHash == 113);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedPresentationTaskExceptionTest,
    "IAmSpeed.Simulation.FrameParallel.PresentationExceptionJoin",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)
bool FIAmSpeedPresentationTaskExceptionTest::RunTest(const FString&)
{
    const bool bExpectedWorker = Speed::EnsurePrivateFrameWorkers(Speed::FFrameParallelPolicy::MaximumLanes - 1) > 0;
    if (!TestTrue(TEXT("owned worker available for joined exception proof"), bExpectedWorker)) return false;
    auto Producer = MakeShared<FPresentationTaskProbe, ESPMode::ThreadSafe>();
    TArray<FProducerRef> Producers{Producer};
    std::atomic<bool> Finished{false};
    Producer->Work = [&] { Finished.store(true, std::memory_order_release); throw std::runtime_error("producer first"); };
    FSimulationSnapshot Snapshot; Snapshot.NumFrame = 6;
    bool bOwnerValidationCalled = false, bProducerFailureFirst = false;
    try
    {
        Speed::FrameParallel::ProduceFramePresentationWithOwnerValidation(Snapshot, Producers, 2, true, [&]() -> bool
        {
            bOwnerValidationCalled = true;
            throw std::runtime_error("validation second");
        });
    }
    catch (const std::runtime_error& Error) { bProducerFailureFirst = FCStringAnsi::Strcmp(Error.what(), "producer first") == 0; }
    TestEqual(TEXT("producer failure keeps serial behavior or parallel owner validation"), bOwnerValidationCalled, bExpectedWorker);
    TestTrue(TEXT("producer exception retains original precedence"), bProducerFailureFirst);
    TestTrue(TEXT("exception path joins all work"), Finished.load(std::memory_order_acquire));
    TestTrue(TEXT("failed producer cannot publish partial output"), Snapshot.PresentationOutputs.IsEmpty());

    Producer->Work = [&] { Finished.store(true, std::memory_order_release); };
    Finished.store(false, std::memory_order_release);
    bool bValidationFailure = false;
    try
    {
        Speed::FrameParallel::ProduceFramePresentationWithOwnerValidation(Snapshot, Producers, 2, true,
            []() -> bool { throw std::runtime_error("validation only"); });
    }
    catch (const std::runtime_error& Error) { bValidationFailure = FCStringAnsi::Strcmp(Error.what(), "validation only") == 0; }
    TestTrue(TEXT("owner exception is preserved after join"), bValidationFailure && Finished.load(std::memory_order_acquire));
    TestEqual(TEXT("completed producer assembles before validation exception as before"), Snapshot.PresentationOutputs.Num(), 1);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedPresentationTaskCameraEquivalenceTest,
    "IAmSpeed.Simulation.FrameParallel.CameraExactSequence",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)
bool FIAmSpeedPresentationTaskCameraEquivalenceTest::RunTest(const FString&)
{
    const auto Original=Speed::CaptureFrameFloatingPointContext();
    if(!TestTrue(TEXT("original full camera numeric context available"),Original.bValid))return false;
    Speed::FScopedFrameFloatingPointContext RestoreOriginal(Original);
    if(!TestTrue(TEXT("owned worker required for numeric oracle"),
        Speed::EnsurePrivateFrameWorkers(Speed::FFrameParallelPolicy::MaximumLanes-1)>0))return false;
    bool SawOriginalCameraFlagChange=false;
    for(const int Rounding:{FE_TONEAREST,FE_DOWNWARD,FE_UPWARD,FE_TOWARDZERO})for(const bool Dirty:{false,true})
    {
        if(!Speed::ApplyFrameFloatingPointContext(Original))return false;
        FSpeedCarCameraConfiguration Configuration;
        const auto Initial=FSpeedCarCameraArm::DefaultState();
        auto Serial=MakeShared<FSpeedCarCameraPresentation,ESPMode::ThreadSafe>(42,0,1,Configuration,Initial);
        auto Parallel=MakeShared<FSpeedCarCameraPresentation,ESPMode::ThreadSafe>(42,0,1,Configuration,Initial);
        TArray<FProducerRef> SerialList{Serial},ParallelList{Parallel};
        if(!TestTrue(TEXT("same exact configuration command on both timelines"),
            Serial->QueueAt({2,1,ESpeedCarCameraCommandKind::Distance,320})
            &&Parallel->QueueAt({2,1,ESpeedCarCameraCommandKind::Distance,320})))return false;
        for(uint64 N=0;N<16;++N)
        {
            FSimulationSnapshot A;A.NumFrame=N;A.StateHash=1234+N;A.InputJournalHash=5678;
            A.Payload={2,4,6};
            auto& Body=A.PresentationBodies.AddDefaulted_GetRef();Body.StableId=42;
            Body.COMState.Location=FVector(double(N),double(N*2),100);
            Body.COMState.Velocity=FVector(300,double(N*7),N<8?0:20);
            Body.COMState.Rotation=FRotator(0,double(N*3),0).Quaternion();
            FSpeedCarCameraInputSnapshot Input;Input.bOnGround=N<8;Input.bBackCamera=N>=12;
            Input.CameraYaw=N%3?.25f:0;Input.CameraPitch=N%4?-.125f:0;Input.Write(Body.Extension);
            FSimulationSnapshot B=A;
            A.PresentationOutputs.Reserve(1);B.PresentationOutputs.Reserve(1);
            // Prepare memory/input before the entry checkpoint, to compare the
            // real producer's arithmetic and complete x87/SIMD exit environment.
            if(std::fesetround(Rounding)!=0||std::feclearexcept(FE_ALL_EXCEPT)!=0)return false;
            if(Dirty&&std::feraiseexcept(FE_INEXACT|FE_UNDERFLOW)!=0)return false;
            const auto Entry=Speed::CaptureFrameFloatingPointContext();
            const bool SerialOK=Speed::FrameParallel::ProduceFramePresentationWithOwnerValidation(A,SerialList,1,true,[]{return true;});
            const auto SerialExit=Speed::CaptureFrameFloatingPointContext();
            if(!Speed::ApplyFrameFloatingPointContext(Entry))return false;
            Speed::ResetFrameParallelStageTotals(Speed::EFrameParallelStage::Camera);
            const bool ParallelOK=Speed::FrameParallel::ProduceFramePresentationWithOwnerValidation(B,ParallelList,2,true,[]{return true;});
            const auto ParallelExit=Speed::CaptureFrameFloatingPointContext();
            const bool ExactExit=Speed::FrameParallel::SamePresentationNumericContext1327(SerialExit,ParallelExit);
            SawOriginalCameraFlagChange|=!Speed::FrameParallel::SamePresentationNumericContext1327(Entry,SerialExit);
            if(!Speed::ApplyFrameFloatingPointContext(Original))return false;
            if(!TestTrue(TEXT("real serial/worker camera both complete"),SerialOK&&ParallelOK)
                ||!TestTrue(TEXT("complete native camera FP exit matches serial including sticky flags and rounding"),ExactExit)
                ||!TestTrue(TEXT("real camera executed on separate native worker"),Speed::GetFrameParallelStageTotals(Speed::EFrameParallelStage::Camera).MaximumNativeThreads==2)
                ||!TestTrue(TEXT("real output and entire solver checkpoint are byte exact"),A.PresentationOutputs.Num()==1&&B.PresentationOutputs.Num()==1&&SameOutput(A.PresentationOutputs[0],B.PresentationOutputs[0])))return false;
            FSpeedCarCameraPose Pose;
            if(!TestTrue(TEXT("real camera valid at the same N"),FSpeedCarCameraPose::Read(B.PresentationOutputs[0].Payload,Pose)&&Pose.Frame==N&&Pose.Result.Status==ESpeedCarCameraStatus::Valid))return false;
            TestTrue(TEXT("physical bytes and hash unchanged"),A.Payload==B.Payload&&A.StateHash==B.StateHash);
        }
    }
    TestTrue(TEXT("oracle actually exercised original camera FP flag changes"),SawOriginalCameraFlagChange);
    // A callback that changes rounding violates the independent-validation
    // contract; retain the joined output privately and fail instead of silently
    // publishing an FP environment that no serial execution produced.
    auto Probe=MakeShared<FPresentationTaskProbe,ESPMode::ThreadSafe>();TArray<FProducerRef> List{Probe};
    FSimulationSnapshot Snapshot;bool Refused=false;
    try
    {
        Speed::FrameParallel::ProduceFramePresentationWithOwnerValidation(Snapshot,List,2,true,[&]
        {std::fesetround(std::fegetround()==FE_UPWARD?FE_DOWNWARD:FE_UPWARD);return true;});
    }
    catch(const std::runtime_error& E){Refused=FCStringAnsi::Strcmp(E.what(),"IAmSpeed camera owner validation changed numeric environment")==0;}
    if(!Speed::ApplyFrameFloatingPointContext(Original))return false;
    TestTrue(TEXT("numeric-mutating owner validation fails closed after join"),Refused&&Snapshot.PresentationOutputs.IsEmpty());
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedPresentationOwnerObservationTest,
    "IAmSpeed.Simulation.FrameParallel.PresentationOwnerObservation",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)
bool FIAmSpeedPresentationOwnerObservationTest::RunTest(const FString&)
{
    const uint32 Owner = FPlatformTLS::GetCurrentThreadId();
    if (!TestTrue(TEXT("private worker available"),
        Speed::EnsurePrivateFrameWorkers(Speed::FFrameParallelPolicy::MaximumLanes - 1) > 0)) return false;
    for (const int32 Lanes : {1, 2}) for (const int32 Failure : {0, 1, 2})
    {
        auto Producer = MakeShared<FPresentationTaskProbe, ESPMode::ThreadSafe>();
        TArray<FProducerRef> Producers{Producer};
        FSimulationSnapshot Snapshot; Snapshot.NumFrame = 19;
        Producer->Work = [Failure] { if (Failure == 1) throw std::runtime_error("producer"); };
        int32 Begins = 0, Ends = 0; bool Returned = false, HooksOnOwner = true, Threw = false;
        try
        {
            Speed::FrameParallel::ProduceFramePresentationWithOwnerValidationAndObservation(
                Snapshot, Producers, Lanes, true,
                [Failure]() -> bool { if (Failure == 2) throw std::runtime_error("validation"); return true; },
                [&](const ISimulationPresentationProducer& Observed)
                { ++Begins; HooksOnOwner &= FPlatformTLS::GetCurrentThreadId() == Owner && &Observed == &Producer.Get(); },
                [&](bool DidReturn)
                { ++Ends; Returned = DidReturn; HooksOnOwner &= FPlatformTLS::GetCurrentThreadId() == Owner; });
        }
        catch (const std::runtime_error&) { Threw = true; }
        TestEqual(TEXT("one owner-local dispatch observation"), Begins, 1);
        TestEqual(TEXT("one owner-local completed scope even on exception"), Ends, 1);
        TestTrue(TEXT("no owner observer migrates to worker"), HooksOnOwner);
        TestEqual(TEXT("producer result distinct from validation exception"), Returned, Failure != 1);
        TestEqual(TEXT("exception outcome preserved"), Threw, Failure != 0);
        TestEqual(TEXT("failed producer never publishes partial output"), Snapshot.PresentationOutputs.Num(), Failure == 1 ? 0 : 1);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedPresentationOwnerNumericEntryTest,
    "IAmSpeed.Simulation.FrameParallel.PresentationOwnerNumericEntry",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)
bool FIAmSpeedPresentationOwnerNumericEntryTest::RunTest(const FString&)
{
    const auto Original = Speed::CaptureFrameFloatingPointContext();
    if (!TestTrue(TEXT("original numeric context"), Original.bValid)) return false;
    Speed::FScopedFrameFloatingPointContext Restore(Original);
    if (!TestTrue(TEXT("private worker"), Speed::EnsurePrivateFrameWorkers(Speed::FFrameParallelPolicy::MaximumLanes - 1) > 0)) return false;
    Speed::FFrameFloatingPointContext SerialExit;
    for (const int32 Lanes : {1, 2})
    {
        if (!Speed::ApplyFrameFloatingPointContext(Original)) return false;
        auto Producer = MakeShared<FPresentationTaskProbe, ESPMode::ThreadSafe>();
        TArray<FProducerRef> Producers{Producer}; FSimulationSnapshot Snapshot;
        bool EntrySeen = false, EndSeen = false;
        Producer->Work = [&]
        {
            EntrySeen = std::fegetround() == FE_DOWNWARD && (std::fetestexcept(FE_INEXACT) != 0);
            std::fesetround(FE_TOWARDZERO); std::feraiseexcept(FE_UNDERFLOW);
        };
        const bool Completed = Speed::FrameParallel::ProduceFramePresentationWithOwnerValidationAndObservation(
            Snapshot, Producers, Lanes, true, [] { return true; },
            [](const ISimulationPresentationProducer&)
            { std::fesetround(FE_DOWNWARD); std::feraiseexcept(FE_INEXACT); },
            [&](bool Returned)
            { EndSeen = Returned && std::fegetround() == FE_TOWARDZERO && (std::fetestexcept(FE_UNDERFLOW) != 0); });
        const auto Exit = Speed::CaptureFrameFloatingPointContext();
        const bool SameExit = Lanes == 1 || Speed::FrameParallel::SamePresentationNumericContext1327(SerialExit, Exit);
        if (Lanes == 1) SerialExit = Exit;
        if (!Speed::ApplyFrameFloatingPointContext(Original)) return false;
        TestTrue(TEXT("owner callback FP entry reaches original producer"), Completed && EntrySeen);
        TestTrue(TEXT("owner completion observes original producer FP exit"), EndSeen && SameExit);
        TestTrue(TEXT("parallel route really uses a private worker"), Lanes == 1 || Producer->CalledThread != FPlatformTLS::GetCurrentThreadId());
    }
    return true;
}

#endif
