#include "SimulationWorker.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "HAL/PlatformProcess.h"

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
#endif
