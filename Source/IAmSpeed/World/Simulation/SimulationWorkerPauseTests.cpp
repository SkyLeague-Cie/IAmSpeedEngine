#include "SimulationWorker.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTLS.h"
#include "HAL/PlatformTime.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpeedWorkerBoundedPauseTest,
	"IAmSpeed.Simulation.WorkerBoundedPause", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)
bool FSpeedWorkerBoundedPauseTest::RunTest(const FString&)
{
	FEvent* Entered = FPlatformProcess::GetSynchEventFromPool(true);
	FEvent* Release = FPlatformProcess::GetSynchEventFromPool(true);
	TAtomic<int32> Count = 0;
	FSimulationWorker Worker([&]()
	{
		if (++Count == 1) { Entered->Trigger(); Release->Wait(5000); }
		return ESimulationWorkerResult::Advanced;
	}, [](FSimulationWorkerWaitContext& Wait) { Wait.WaitUntil(FPlatformTime::Seconds() + .001); });
	TestFalse(TEXT("unstarted worker cannot acknowledge"), Worker.TryPause(1));
	TestTrue(TEXT("worker starts"), Worker.Start());
	TestTrue(TEXT("test holds one in-flight frame"), Entered->Wait(1000));
	TestFalse(TEXT("in-flight timeout is not a boundary"), Worker.TryPause(10));
	TestTrue(TEXT("timeout retains pause request"), Worker.IsPaused());
	Release->Trigger();
	TestTrue(TEXT("same request later receives acknowledgment"), Worker.TryPause(1000));
	TestEqual(TEXT("timeout never resumed another frame"), Count.Load(), 1);
	for (int32 N = 0; N < 100; ++N)
	{
		TestTrue(TEXT("repeated pause is idempotent"), Worker.TryPause(1000));
		Worker.Resume();
		TestTrue(TEXT("new pause requires current request ack"), Worker.TryPause(1000));
		const int32 PausedCount = Count.Load();
		FPlatformProcess::Sleep(.001f);
		TestEqual(TEXT("acknowledged frame cannot run on"), Count.Load(), PausedCount);
	}
	Worker.StopAndJoin();
	TestFalse(TEXT("stopped worker cannot authorize mutation"), Worker.TryPause(10));
	FPlatformProcess::ReturnSynchEventToPool(Entered);
	FPlatformProcess::ReturnSynchEventToPool(Release);
	return true;
}


IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpeedWorkerLifecycleBoundaryTest,
    "IAmSpeed.Simulation.WorkerLifecycleBoundary", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)
bool FSpeedWorkerLifecycleBoundaryTest::RunTest(const FString&)
{
    FEvent* ServicedPaused = FPlatformProcess::GetSynchEventFromPool(true);
    FEvent* ServicedAgain = FPlatformProcess::GetSynchEventFromPool(true);
    TAtomic<bool> AllowBoundary = false;
    TAtomic<bool> ObserveAgain = false;
    TAtomic<bool> Closed = false;
    TAtomic<bool> SameOwner = false;
    TAtomic<int32> Frames = 0;
    uint32 OwnerThread = 0;
    FSimulationWorker Worker([&]()
    {
        ++Frames;
        return ESimulationWorkerResult::Advanced;
    }, [](FSimulationWorkerWaitContext&) {}, [&](bool bPaused)
    {
        if (!OwnerThread) OwnerThread = FPlatformTLS::GetCurrentThreadId();
        if (bPaused)
        {
            ServicedPaused->Trigger();
            if (ObserveAgain.Load()) ServicedAgain->Trigger();
        }
        return AllowBoundary.Load() ? ESimulationBoundaryResult::Ready : ESimulationBoundaryResult::Waiting;
    }, [&]()
    {
        SameOwner.Store(OwnerThread == FPlatformTLS::GetCurrentThreadId());
        Closed.Store(true);
    });
    TestTrue(TEXT("worker starts while lifecycle is waiting"), Worker.Start());
    TestFalse(TEXT("pending lifecycle cannot acknowledge pause"), Worker.TryPause(20));
    TestTrue(TEXT("paused worker services lifecycle"), ServicedPaused->Wait(1000));
    TestEqual(TEXT("waiting boundary forbids physics"), Frames.Load(), 0);
    AllowBoundary.Store(true);
    TestTrue(TEXT("completed lifecycle allows acknowledgment"), Worker.TryPause(1000));
    ObserveAgain.Store(true);
    TestTrue(TEXT("commands continue to be serviced after acknowledgment"), ServicedAgain->Wait(1000));
    TestEqual(TEXT("paused lifecycle never advances physics"), Frames.Load(), 0);
    Worker.StopAndJoin();
    TestTrue(TEXT("join includes owner cleanup"), Closed.Load());
    TestTrue(TEXT("cleanup runs on construction/service lane"), SameOwner.Load());
    TestFalse(TEXT("joined owner cannot acknowledge a new pause"), Worker.TryPause(1));
    FPlatformProcess::ReturnSynchEventToPool(ServicedPaused);
    FPlatformProcess::ReturnSynchEventToPool(ServicedAgain);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpeedWorkerFailedLifecycleTest,
    "IAmSpeed.Simulation.WorkerFailedLifecycle", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)
bool FSpeedWorkerFailedLifecycleTest::RunTest(const FString&)
{
    FEvent* Closed = FPlatformProcess::GetSynchEventFromPool(true);
    TAtomic<int32> Frames = 0;
    FSimulationWorker Worker([&]()
    {
        ++Frames;
        return ESimulationWorkerResult::Advanced;
    }, [](FSimulationWorkerWaitContext&) {}, [](bool)
    {
        return ESimulationBoundaryResult::Failed;
    }, [&]() { Closed->Trigger(); });
    TestTrue(TEXT("worker starts"), Worker.Start());
    TestTrue(TEXT("failed boundary still closes on worker"), Closed->Wait(1000));
    Worker.StopAndJoin();
    TestEqual(TEXT("failed boundary stops before any physics"), Frames.Load(), 0);
    TestFalse(TEXT("failure is never a pause acknowledgment"), Worker.TryPause(1));
    FPlatformProcess::ReturnSynchEventToPool(Closed);
    return true;
}
#endif
