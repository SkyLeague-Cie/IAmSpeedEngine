#include "SimulationFrameJournal.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "Async/Async.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpeedPresentationOutputTest,
	"IAmSpeed.Simulation.PresentationOutputAtomicJoin",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)
bool FSpeedPresentationOutputTest::RunTest(const FString&)
{
	using namespace Speed::SimulationBoundary;
	FSnapshotBuffer Buffer;
	FSimulationSnapshot S;
	S.NumFrame = 10; S.StateHash = 42; S.InputJournalHash = 57;
	S.Payload = {1, 2, 3};
	S.PresentationBodies.SetNum(2);
	S.PresentationBodies[0].StableId = 1; S.PresentationBodies[1].StableId = 2;
	FSimulationPresentationOutput O;
	O.NumFrame = 10; O.OwnerStableId = 1; O.Channel = 3; O.Payload = {4, 5};
	S.PresentationOutputs.Add(O);
	TestTrue(TEXT("common packet commits"), Buffer.Publish(S));
	FPresentationFrameLatch Latch;
	FSimulationPoseConsumption Car, Ball;
	FSimulationPresentationOutput Camera;
	TestTrue(TEXT("car latches"), Latch.ReadBody(100, Buffer, 1, Car));
	S.NumFrame = 11; S.PresentationOutputs[0].NumFrame = 11;
	S.PresentationOutputs[0].Payload = {8};
	TestTrue(TEXT("producer may advance between GT consumers"), Buffer.Publish(S));
	TestTrue(TEXT("ball still reads common latched frame"), Latch.ReadBody(100, Buffer, 2, Ball));
	TestTrue(TEXT("camera still reads common latched frame"), Latch.ReadOutput(100, Buffer, 1, 3, Camera));
	TestEqual(TEXT("camera and car serial identical"), Camera.PublicationSerial, Car.PublicationSerial);
	TestEqual(TEXT("camera and ball serial identical"), Camera.PublicationSerial, Ball.PublicationSerial);
	TestEqual(TEXT("old immutable payload retained"), Camera.Payload.Num(), 2);
	TestTrue(TEXT("next GT frame advances all together"), Latch.ReadOutput(101, Buffer, 1, 3, Camera));
	TestEqual(TEXT("new exact frame"), Camera.NumFrame, uint64(11));
	FSimulationSnapshot Copy;
	Buffer.ReadLatest(Copy);
	TestTrue(TEXT("physical payload unaffected"), Copy.Payload == S.Payload);
	TestEqual(TEXT("physical hash unaffected"), Copy.StateHash, S.StateHash);
	auto Bad = S; Bad.PresentationOutputs[0].NumFrame = 9;
	TestFalse(TEXT("mixed output frame cannot publish"), Buffer.Publish(Bad));
	Bad = S;
	const FSimulationPresentationOutput Duplicate = Bad.PresentationOutputs[0];
	Bad.PresentationOutputs.Add(Duplicate);
	TestFalse(TEXT("duplicate address cannot publish"), Buffer.Publish(Bad));
	Bad = S; Bad.PresentationOutputs[0].Payload.SetNum(65536);
	TestFalse(TEXT("output obeys total byte bound"), Buffer.Publish(Bad));
	auto Writer = Async(EAsyncExecution::Thread, [&Buffer, S]() mutable
	{
		for (uint64 F = 12; F < 1012; ++F)
		{
			S.NumFrame = F; S.PresentationOutputs[0].NumFrame = F;
			Buffer.Publish(S);
		}
	});
	for (int32 N = 0; N < 1000; ++N)
	{
		FSimulationSnapshot Read;
		TestTrue(TEXT("concurrent read"), Buffer.ReadLatest(Read));
		TestEqual(TEXT("whole packet frame is atomic"), Read.PresentationOutputs[0].NumFrame, Read.NumFrame);
		TestEqual(TEXT("whole packet serial is atomic"), Read.PresentationOutputs[0].PublicationSerial, Read.PublicationSerial);
	}
	Writer.Get();
	return true;
}
#endif
