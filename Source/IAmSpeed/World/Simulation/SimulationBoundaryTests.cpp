#include "SimulationFrameJournal.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "SimulationWorld.h"
#include "SimulationWorker.h"
#include "RealTimeSimulation.h"
#include "IAmSpeed/Components/SpeedMovementComponent.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "Async/Async.h"
#include "UObject/StrongObjectPtr.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SphereSubBody.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FIAmSpeedWheeledInputQueueTest,
	"IAmSpeed.Simulation.WheeledInputQueue",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedWheeledInputQueueTest::RunTest(const FString& Parameters)
{
	TStrongObjectPtr<USpeedWheeledComponent> Owner(NewObject<USpeedWheeledComponent>());
	USpeedWheeledComponent* Component = Owner.Get();
	FWheeledInputState Input;
	Component->QueueWheeledInputForFrame(7, Input);
	Component->QueueTestWheeledPhysicalInputForFrame(7, Input);
	TestEqual(TEXT("same activation frame replaces a complete command"), Component->PendingWheeledInputCommands.Num(), 1);
	TestTrue(TEXT("test replacement carries bypass policy atomically"), Component->PendingWheeledInputCommands[0].bBypassSlew);
	Component->ConsumeQueuedWheeledInputsForFrame(7);

	// Reproduce network and canonical producers racing with the single consumer.
	auto Network = Async(EAsyncExecution::Thread, [Component, Input]()
	{
		for (int32 Frame = 0; Frame < 2000; ++Frame)
			Component->QueueWheeledInputForFrame(Frame * 2, Input);
	});
	auto Canonical = Async(EAsyncExecution::Thread, [Component, Input]()
	{
		for (int32 Frame = 0; Frame < 2000; ++Frame)
			Component->QueueTestWheeledPhysicalInputForFrame(Frame * 2 + 1, Input);
	});
	for (int32 Frame = 0; Frame < 4000; ++Frame)
		Component->ConsumeQueuedWheeledInputsForFrame(Frame);
	Network.Wait();
	Canonical.Wait();
	const auto& Commands = Component->PendingWheeledInputCommands;
	TestTrue(TEXT("concurrent producers preserve the capacity bound"), Commands.Num() <= Component->MaxPendingWheeledInputs);
	for (int32 Index = 1; Index < Commands.Num(); ++Index)
		TestTrue(TEXT("pending frames remain sorted and unique"), Commands[Index - 1].ActivationFrame < Commands[Index].ActivationFrame);
	Component->ConsumeQueuedWheeledInputsForFrame(MAX_int32);
	TestTrue(TEXT("consumer drains all complete commands"), Commands.IsEmpty());

	FNetworkWheeledSpeedInputState Packet;
	Packet.LocalFrame = 7;
	Packet.WheeledInput.Throttle = 0;
	Packet.ApplyData(Component);
	TestEqual(TEXT("ordinary network input still queues"), Commands.Num(), 1);
	Component->SetTestInputOverrideEnabled(true);
	TestTrue(TEXT("starting a sealed script discards stale queued inputs"), Commands.IsEmpty());
	Input.Throttle = 255;
	Component->QueueTestWheeledPhysicalInputForFrame(7, Input);
	Packet.ApplyData(Component);
	Component->QueueNetworkWheeledInputForFrame(6, Packet.WheeledInput);
	TestEqual(TEXT("network cannot replace scripted commands or insert stale commands"), Commands.Num(), 1);
	TestTrue(TEXT("scripted command keeps its complete payload and slew policy"),
		Commands[0].Input.Throttle == 255 && Commands[0].bBypassSlew);
	Component->ConsumeQueuedWheeledInputsForFrame(7);
	Component->QueueWheeledInputForFrame(8, Input);
	TestEqual(TEXT("script commands using ordinary slew remain accepted"), Commands.Num(), 1);
	Component->ConsumeQueuedWheeledInputsForFrame(8);
	Component->SetTestInputOverrideEnabled(false);
	Packet.ApplyData(Component);
	TestEqual(TEXT("leaving script mode restores network input"), Commands.Num(), 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FIAmSpeedSleepEligibilityTest,
	"IAmSpeed.Simulation.SleepEligibility",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedSleepEligibilityTest::RunTest(const FString& Parameters)
{
	Speed::FSimulationSleepState Sleep;
	SKinematic Rest;
	TestFalse(TEXT("new or restored bodies cannot reuse a previous pose"), Sleep.CanSleep(Rest, true));
	Sleep.ObserveCompletedPose(Rest);
	TestFalse(TEXT("zero speed in mid-air is not sleep"), Sleep.CanSleep(Rest, false));
	TestTrue(TEXT("unchanged supported equilibrium can sleep"), Sleep.CanSleep(Rest, true));
	SKinematic Changed = Rest;
	Changed.Velocity.X = 0.000001;
	TestFalse(TEXT("any linear impulse wakes immediately"), Sleep.CanSleep(Changed, true));
	Changed = Rest;
	Changed.AngularVelocity.Z = 0.000001;
	TestFalse(TEXT("even normal-axis spin remains awake"), Sleep.CanSleep(Changed, true));
	Changed = Rest;
	Changed.Acceleration.X = 1;
	TestFalse(TEXT("a force wakes before translation begins"), Sleep.CanSleep(Changed, true));
	Changed = Rest;
	Changed.AngularAcceleration.Z = 1;
	TestFalse(TEXT("a torque wakes before rotation begins"), Sleep.CanSleep(Changed, true));
	Changed = Rest;
	Changed.Location.Z = 1;
	TestFalse(TEXT("teleport invalidates the old support pose"), Sleep.CanSleep(Changed, true));
	Changed = Rest;
	Changed.Rotation = FQuat(FVector::UpVector, 0.1);
	TestFalse(TEXT("orientation changes invalidate the pose"), Sleep.CanSleep(Changed, true));
	Sleep.Reset();
	TestFalse(TEXT("restoration invalidates sleep even at the same pose"), Sleep.CanSleep(Rest, true));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FIAmSpeedSnapshotHashBytesTest,
	"IAmSpeed.Simulation.SnapshotHashBytes",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedSnapshotHashBytesTest::RunTest(const FString& Parameters)
{
	const auto OriginalByteHash = [](const uint8* Bytes, const SIZE_T Count)
	{
		uint64 Hash = 1469598103934665603ull;
		for (SIZE_T Index = 0; Index < Count; ++Index)
		{
			Hash ^= Bytes[Index];
			Hash *= 1099511628211ull;
		}
		return Hash == 0 ? uint64(1) : Hash;
	};
	TestEqual(TEXT("empty null payload keeps the original offset"),
		Speed::SimulationBoundary::HashBytes(nullptr, 0), OriginalByteHash(nullptr, 0));
	TArray<uint8> Storage;
	Storage.SetNumUninitialized(65536 + 16);
	FRandomStream Random(72319);
	for (int32 Pattern = 0; Pattern < 4; ++Pattern)
	{
		for (int32 Index = 0; Index < Storage.Num(); ++Index)
		{
			Storage[Index] = Pattern == 0 ? 0 : Pattern == 1 ? 255 :
				Pattern == 2 ? uint8(Random.RandRange(0, 255)) :
				(Index % 67 == 0 ? uint8(Random.RandRange(1, 255)) : 0);
		}
		for (int32 Alignment = 0; Alignment < 16; ++Alignment)
		{
			const uint8* Bytes = Storage.GetData() + Alignment;
			for (SIZE_T Count = 0; Count <= 256; ++Count)
			{
				if (!TestEqual(TEXT("all short lengths, tails and alignments preserve byte hash"),
					Speed::SimulationBoundary::HashBytes(Bytes, Count), OriginalByteHash(Bytes, Count)))
					return false;
			}
			for (const SIZE_T Count : { SIZE_T(8191), SIZE_T(8192), SIZE_T(65535), SIZE_T(65536) })
			{
				if (!TestEqual(TEXT("large dense/sparse/zero payload keeps exact original hash"),
					Speed::SimulationBoundary::HashBytes(Bytes, Count), OriginalByteHash(Bytes, Count)))
					return false;
			}
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FIAmSpeedInputJournalTest,
	"IAmSpeed.Simulation.InputJournal",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedInputJournalTest::RunTest(const FString& Parameters)
{
	const TArray<uint8> FirstPayload = { 1, 2, 3 };
	const TArray<uint8> SecondPayload = { 9, 8 };
	Speed::SimulationBoundary::FInputJournal A(3);
	Speed::SimulationBoundary::FInputJournal B(3);
	TestTrue(TEXT("first append"), A.Append(10, 1, FirstPayload));
	TestTrue(TEXT("out-of-order append"), A.Append(2, 1, SecondPayload));
	TestTrue(TEXT("same frame may address another body"), A.Append(2, 2, FirstPayload));
	TestFalse(TEXT("frame and target pair is unique"), A.Append(2, 2, SecondPayload));
	TestFalse(TEXT("capacity is bounded"), A.Append(11, 1, FirstPayload));
	TestTrue(TEXT("reverse first append"), B.Append(2, 1, SecondPayload));
	TestTrue(TEXT("reverse second append"), B.Append(2, 2, FirstPayload));
	TestTrue(TEXT("reverse third append"), B.Append(10, 1, FirstPayload));
	A.Seal();
	B.Seal();
	TestEqual(TEXT("sealed hash is independent of append order"), A.StableHash(), B.StableHash());
	TestEqual(TEXT("commands are ordered by frame"), A.GetCommands()[0].ActivationFrame, uint64(2));
	TestEqual(TEXT("frame index returns only addressed commands"), A.GetCommandsForFrame(2).Num(), 2);
	TestEqual(TEXT("missing frame has an empty range"), A.GetCommandsForFrame(3).Num(), 0);
	TestEqual(TEXT("indexed command lookup preserves payload"), A.Find(10, 1)->Payload[0], uint8(1));
	TestFalse(TEXT("sealed journal rejects writes"), A.Append(12, 1, FirstPayload));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FIAmSpeedSnapshotBufferTest,
	"IAmSpeed.Simulation.SnapshotBuffer",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedSnapshotBufferTest::RunTest(const FString& Parameters)
{
	Speed::SimulationBoundary::FSnapshotBuffer Buffer(4);
	FSimulationSnapshot Source;
	Source.NumFrame = 7;
	Source.Payload = { 4, 3, 2, 1 };
	Source.StateHash = Speed::SimulationBoundary::HashBytes(Source.Payload.GetData(), Source.Payload.Num());
	TestTrue(TEXT("bounded snapshot publishes"), Buffer.Publish(Source));
	Source.Payload[0] = 99;
	FSimulationSnapshot Published;
	TestTrue(TEXT("published snapshot is readable"), Buffer.ReadLatest(Published));
	TestEqual(TEXT("publication owns an immutable copy"), Published.Payload[0], uint8(4));
	TestEqual(TEXT("published frame"), Buffer.PublishedFrame(), uint64(7));
	Source.Payload.Add(0);
	TestFalse(TEXT("oversized payload is rejected"), Buffer.Publish(Source));
	TestEqual(TEXT("failed publication preserves previous frame"), Buffer.PublishedFrame(), uint64(7));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FIAmSpeedFrameHashJournalTest,
	"IAmSpeed.Simulation.FrameHashJournal",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedFrameHashJournalTest::RunTest(const FString& Parameters)
{
	Speed::SimulationBoundary::FFrameHashJournal Expected(3);
	Speed::SimulationBoundary::FFrameHashJournal Actual(3);
	TestTrue(TEXT("first expected hash"), Expected.Append(4, 100));
	TestTrue(TEXT("second expected hash"), Expected.Append(5, 200));
	TestTrue(TEXT("first actual hash"), Actual.Append(4, 100));
	TestTrue(TEXT("second actual hash"), Actual.Append(5, 201));
	TestFalse(TEXT("frames must increase"), Actual.Append(5, 202));
	const TOptional<FSimulationHashDivergence> Divergence = Expected.FirstDivergence(Actual);
	TestTrue(TEXT("divergence is reported"), Divergence.IsSet());
	TestEqual(TEXT("first divergent frame"), Divergence->NumFrame, uint64(5));
	Expected.RemoveFrom(5);
	TestEqual(TEXT("rollback removes restored and later hashes"), Expected.Num(), 1);
	TestTrue(TEXT("restored frame hash can be replaced"), Expected.Append(5, 201));
	TestFalse(TEXT("replacement eliminates divergence"), Expected.FirstDivergence(Actual).IsSet());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FIAmSpeedSnapshotKinematicLayoutTest,
	"IAmSpeed.Simulation.SnapshotKinematicLayout",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedSnapshotKinematicLayoutTest::RunTest(const FString& Parameters)
{
	TStrongObjectPtr<USpeedMovementComponent> First(NewObject<USpeedMovementComponent>());
	TStrongObjectPtr<USpeedMovementComponent> Second(NewObject<USpeedMovementComponent>());
	Speed::FSimulationWorld World;
	World.AddAdapter(*First);
	World.AddAdapter(*Second);
	World.RebuildOrderedAdapters();
	FRandomStream Random(19385);
	for (int32 Sample = 0; Sample < 1024; ++Sample)
	{
		SKinematic State;
		double* Fields[] = { &State.Location.X, &State.Location.Y, &State.Location.Z,
			&State.Velocity.X, &State.Velocity.Y, &State.Velocity.Z,
			&State.Acceleration.X, &State.Acceleration.Y, &State.Acceleration.Z,
			&State.Rotation.X, &State.Rotation.Y, &State.Rotation.Z, &State.Rotation.W,
			&State.AngularVelocity.X, &State.AngularVelocity.Y, &State.AngularVelocity.Z,
			&State.AngularAcceleration.X, &State.AngularAcceleration.Y, &State.AngularAcceleration.Z };
		for (double* Field : Fields)
		{
			// Arbitrary finite double bits, including subnormals and signed zero;
			// no numerical conversion or quaternion normalization belongs here.
			uint64 Bits = (uint64(Random.GetUnsignedInt()) << 32) | Random.GetUnsignedInt();
			if ((Bits & 0x7ff0000000000000ull) == 0x7ff0000000000000ull)
				Bits ^= 0x0010000000000000ull;
			if (Sample % 4 == 0) Bits = 0;
			if (Sample % 4 == 1) Bits = 0x8000000000000000ull;
			FMemory::Memcpy(Field, &Bits, sizeof(Bits));
		}
		First->SetKinematicState(State);
		Second->SetKinematicState(State);
		First->SetIsFrozen(Sample % 2 != 0);
		Second->SetIsFrozen(Sample % 2 == 0);
		const uint64 Frame = uint64(Sample);
		constexpr uint64 InputHash = 19385723;
		TArray<uint8> Expected;
		const auto OriginalAppend = [&Expected](const auto& Value)
		{
			Expected.Append(reinterpret_cast<const uint8*>(&Value), sizeof(Value));
		};
		OriginalAppend(uint32(2)); // Existing schema, not the implementation's writer.
		OriginalAppend(Frame);
		OriginalAppend(InputHash);
		OriginalAppend(uint32(2));
		for (USpeedMovementComponent* Component : { First.Get(), Second.Get() })
		{
			OriginalAppend(World.FindStableId(*Component));
			for (const double* Field : Fields) OriginalAppend(*Field);
			OriginalAppend(uint8(Component->IsFrozen()));
			OriginalAppend(uint32(0)); // No mechanic payload in this base adapter.
		}
		OriginalAppend(uint32(0)); // Active pairs.
		OriginalAppend(uint32(0)); // Pending pairs.
		const FSimulationSnapshot Snapshot = World.CaptureSnapshot(Frame, InputHash);
		if (!TestTrue(TEXT("batched fields keep the original bytes, including the unaligned second body"),
			Snapshot.Payload == Expected)) return false;
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FIAmSpeedSnapshotRestoreTest,
	"IAmSpeed.Simulation.SnapshotRestore",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedSnapshotRestoreTest::RunTest(const FString& Parameters)
{
	USpeedMovementComponent* Component = NewObject<USpeedMovementComponent>();
	Speed::FSimulationWorld World;
	TestTrue(TEXT("component registers"), World.AddAdapter(*Component));
	World.RebuildOrderedAdapters();

	SKinematic ExpectedState;
	ExpectedState.Location = FVector(100.25, -20.5, 77.75);
	ExpectedState.Velocity = FVector(2.0, 3.0, 4.0);
	ExpectedState.Acceleration = FVector(-5.0, 6.0, 7.0);
	ExpectedState.Rotation = FQuat(FVector::UpVector, 0.25);
	ExpectedState.AngularVelocity = FVector(0.1, 0.2, 0.3);
	ExpectedState.AngularAcceleration = FVector(0.4, 0.5, 0.6);
	Component->SetKinematicState(ExpectedState);
	Component->SetIsFrozen(true);
	const uint64 InputHash = 123456789u;
	const FSimulationSnapshot Snapshot = World.CaptureSnapshot(42, InputHash);

	SKinematic MutatedState;
	MutatedState.Location = FVector(-999.0, 0.0, 0.0);
	Component->SetKinematicState(MutatedState);
	Component->SetIsFrozen(false);
	TestTrue(TEXT("valid snapshot restores"), World.RestoreSnapshot(Snapshot, InputHash));
	TestEqual(TEXT("location is restored"),
		FVector(Component->GetKinematicState().Location), FVector(ExpectedState.Location));
	TestEqual(TEXT("velocity is restored"),
		FVector(Component->GetKinematicState().Velocity), FVector(ExpectedState.Velocity));
	TestEqual(TEXT("rotation is restored"),
		Component->GetKinematicState().Rotation, ExpectedState.Rotation);
	TestTrue(TEXT("frozen state is restored"), Component->IsFrozen());
	const FSimulationSnapshot RoundTrip = World.CaptureSnapshot(42, InputHash);
	TestEqual(TEXT("round-trip snapshot hash is exact"), RoundTrip.StateHash, Snapshot.StateHash);

	FSimulationSnapshot Corrupted = Snapshot;
	Corrupted.Payload.Last() ^= 1;
	TestFalse(TEXT("corrupted payload is rejected"), World.RestoreSnapshot(Corrupted, InputHash));
	TestEqual(TEXT("rejected restore leaves live state unchanged"),
		World.CaptureSnapshot(42, InputHash).StateHash, Snapshot.StateHash);
	TestFalse(TEXT("wrong input history is rejected"), World.RestoreSnapshot(Snapshot, InputHash + 1));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FIAmSpeedPersistentPairIndexTest,
	"IAmSpeed.Simulation.PersistentPairIndex",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedPersistentPairIndexTest::RunTest(const FString& Parameters)
{
	USphereSubBody* BodyA = NewObject<USphereSubBody>();
	UBoxSubBody* BodyB = NewObject<UBoxSubBody>();
	const uint32 LowId = FMath::Min(BodyA->GetUniqueID(), BodyB->GetUniqueID());
	const uint32 HighId = FMath::Max(BodyA->GetUniqueID(), BodyB->GetUniqueID());
	const uint64 PairKey = (static_cast<uint64>(LowId) << 32) | HighId;

	Speed::FSimulationWorld World;
	World.AddPendingRollingContactPair(PairKey, *BodyA, *BodyB);
	TestNotNull(TEXT("pending pair is indexed"), World.FindPendingRollingContactPair(PairKey));
	TestTrue(TEXT("pending pair is promoted"), World.ActivatePendingRollingContactPair(PairKey));
	TestNull(TEXT("promotion removes pending index"), World.FindPendingRollingContactPair(PairKey));
	TestNotNull(TEXT("promotion adds active index"), World.FindDynamicContactPair(PairKey));
	const TSet<uint64>* BodyPairs = World.FindDynamicContactPairKeys(BodyA->GetUniqueID());
	TestTrue(TEXT("body adjacency index contains pair"), BodyPairs && BodyPairs->Contains(PairKey));

	World.RemoveDynamicContactPairAtSwap(World.FindDynamicContactPairIndex(PairKey));
	TestNull(TEXT("removal clears active index"), World.FindDynamicContactPair(PairKey));
	TestNull(TEXT("removal clears body adjacency index"),
		World.FindDynamicContactPairKeys(BodyA->GetUniqueID()));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FIAmSpeedSimulationWorkerLifecycleTest,
	"IAmSpeed.Simulation.WorkerLifecycle",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedSimulationWorkerLifecycleTest::RunTest(const FString& Parameters)
{
	TAtomic<int32> WorkCount = 0;
	TAtomic<int32> WaitCount = 0;
	FSimulationWorker Worker(
		[&WorkCount]()
		{
			const int32 Count = ++WorkCount;
			return Count >= 3
				? ESimulationWorkerResult::Complete
				: ESimulationWorkerResult::Advanced;
		},
		[&WaitCount](FSimulationWorkerWaitContext&)
		{
			++WaitCount;
		});
	TestTrue(TEXT("worker starts"), Worker.Start());
	const double Deadline = FPlatformTime::Seconds() + 1.0;
	while (Worker.IsRunning() && FPlatformTime::Seconds() < Deadline)
	{
		FPlatformProcess::SleepNoStats(0.001f);
	}
	Worker.StopAndJoin();
	TestEqual(TEXT("worker stops on terminal result"), WorkCount.Load(), 3);
	TestEqual(TEXT("pacing policy runs only between committed frames"), WaitCount.Load(), 2);
	TestFalse(TEXT("joined worker is no longer running"), Worker.IsRunning());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FIAmSpeedRealTimeFramePacerTest,
	"IAmSpeed.Simulation.RealTimeFramePacer",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedRealTimeFramePacerTest::RunTest(const FString& Parameters)
{
	constexpr double FrameIntervalSeconds = 1.0 / 300.0;
	FRealTimeFramePacer Pacer;
	const FRealTimeFramePacer::FFrameStartSample First =
		Pacer.BeginFrame(10.0, FrameIntervalSeconds);
	TestFalse(TEXT("first frame has no preceding interval"), First.bHasPreviousStart);
	TestEqual(TEXT("first deadline is based on its actual start"),
		First.NextDeadlineSeconds, 10.0 + FrameIntervalSeconds);

	constexpr double RecoverableLatenessSeconds = 0.0006;
	const double LateStartSeconds =
		10.0 + FrameIntervalSeconds + RecoverableLatenessSeconds;
	const FRealTimeFramePacer::FFrameStartSample Late =
		Pacer.BeginFrame(LateStartSeconds, FrameIntervalSeconds);
	TestTrue(TEXT("late frame reports its preceding interval"), Late.bHasPreviousStart);
	TestEqual(TEXT("late interval is measured from actual starts"),
		Late.IntervalSeconds, FrameIntervalSeconds + RecoverableLatenessSeconds);
	const double ExpectedRecoveryPerInterval = RecoverableLatenessSeconds / 3.0;
	TestTrue(TEXT("ordinary lateness is spread over three intervals"),
		FMath::IsNearlyEqual(
			Late.NextDeadlineSeconds,
			LateStartSeconds + FrameIntervalSeconds - ExpectedRecoveryPerInterval,
			1.e-12));
	TestFalse(TEXT("ordinary lateness fits within the bounded recovery policy"),
		Late.bRecoveryLimited);

	const FRealTimeFramePacer::FFrameStartSample Recovery2 =
		Pacer.BeginFrame(Late.NextDeadlineSeconds, FrameIntervalSeconds);
	const FRealTimeFramePacer::FFrameStartSample Recovery3 =
		Pacer.BeginFrame(Recovery2.NextDeadlineSeconds, FrameIntervalSeconds);
	TestTrue(TEXT("three smoothed intervals recover the original phase"),
		FMath::IsNearlyEqual(
			Recovery3.NextDeadlineSeconds,
			10.0 + FrameIntervalSeconds * 4.0,
			1.e-12));

	Pacer.Reset();
	Pacer.BeginFrame(15.0, FrameIntervalSeconds);
	const FRealTimeFramePacer::FFrameStartSample LargeHitch = Pacer.BeginFrame(
		15.0 + FrameIntervalSeconds * 4.0, FrameIntervalSeconds);
	TestTrue(TEXT("large hitch reports bounded recovery"), LargeHitch.bRecoveryLimited);
	TestTrue(TEXT("large hitch cannot shorten the next interval by more than 25 percent"),
		LargeHitch.NextDeadlineSeconds >=
			15.0 + FrameIntervalSeconds * 4.75 - UE_DOUBLE_SMALL_NUMBER);
	const FRealTimeFramePacer::FFrameCountExpectation TenMillisecondTick =
		FRealTimeFramePacer::BuildFrameCountExpectation(0.010, FrameIntervalSeconds);
	TestEqual(TEXT("10 ms tick expects at least three physical frames"),
		TenMillisecondTick.MinimumFrames, uint64(3));
	TestEqual(TEXT("clock phase allows a fourth physical frame"),
		TenMillisecondTick.MaximumFrames, uint64(4));

	Pacer.Reset();
	TestFalse(TEXT("reset starts a new pacing sequence"),
		Pacer.BeginFrame(20.0, FrameIntervalSeconds).bHasPreviousStart);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FIAmSpeedSimulationWorkerRealTimeSpacingTest,
	"IAmSpeed.Simulation.WorkerRealTimeSpacing",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedSimulationWorkerRealTimeSpacingTest::RunTest(const FString& Parameters)
{
	constexpr int32 TargetFrameCount = 12;
	constexpr double FrameIntervalSeconds = 1.0 / 300.0;
	FRealTimeFramePacer Pacer;
	TArray<double> FrameStarts;
	TArray<double> ScheduledStarts;
	FrameStarts.Reserve(TargetFrameCount);
	ScheduledStarts.Reserve(TargetFrameCount);
	FSimulationWorker Worker(
		[&Pacer, &FrameStarts, &ScheduledStarts]()
		{
			const double StartSeconds = FPlatformTime::Seconds();
			FrameStarts.Add(StartSeconds);
			ScheduledStarts.Add(Pacer.GetNextDeadlineSeconds());
			Pacer.BeginFrame(StartSeconds, FrameIntervalSeconds);
			return FrameStarts.Num() >= TargetFrameCount
				? ESimulationWorkerResult::Complete
				: ESimulationWorkerResult::Advanced;
		},
		[&Pacer](FSimulationWorkerWaitContext& WaitContext)
		{
			WaitContext.WaitUntil(Pacer.GetNextDeadlineSeconds(), 0.00025);
		});
	TestTrue(TEXT("paced worker starts"), Worker.Start());
	const double CompletionDeadline = FPlatformTime::Seconds() + 2.0;
	while (Worker.IsRunning() && FPlatformTime::Seconds() < CompletionDeadline)
	{
		FPlatformProcess::SleepNoStats(0.001f);
	}
	Worker.StopAndJoin();
	TestEqual(TEXT("paced worker completes every requested frame"),
		FrameStarts.Num(), TargetFrameCount);
	for (int32 Index = 1; Index < FrameStarts.Num(); ++Index)
	{
		TestTrue(FString::Printf(TEXT("frame start %d respects its smoothed deadline"), Index),
			FrameStarts[Index] >= ScheduledStarts[Index] - 0.000001);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FIAmSpeedSimulationWorkerPauseBoundaryTest,
	"IAmSpeed.Simulation.WorkerPauseBoundary",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedSimulationWorkerPauseBoundaryTest::RunTest(const FString& Parameters)
{
	TAtomic<int32> WorkCount = 0;
	FSimulationWorker Worker(
		[&WorkCount]()
		{
			++WorkCount;
			return ESimulationWorkerResult::Advanced;
		},
		[](FSimulationWorkerWaitContext& WaitContext)
		{
			WaitContext.WaitUntil(FPlatformTime::Seconds() + 0.001);
		});
	TestTrue(TEXT("worker starts"), Worker.Start());
	const double StartDeadline = FPlatformTime::Seconds() + 1.0;
	while (WorkCount.Load() == 0 && FPlatformTime::Seconds() < StartDeadline)
	{
		FPlatformProcess::SleepNoStats(0.001f);
	}

	Worker.Pause();
	const int32 PausedWorkCount = WorkCount.Load();
	FPlatformProcess::SleepNoStats(0.02f);
	TestEqual(TEXT("pause returns at a stable frame boundary"),
		WorkCount.Load(), PausedWorkCount);

	Worker.Resume();
	const double ResumeDeadline = FPlatformTime::Seconds() + 1.0;
	while (WorkCount.Load() == PausedWorkCount &&
		FPlatformTime::Seconds() < ResumeDeadline)
	{
		FPlatformProcess::SleepNoStats(0.001f);
	}
	TestTrue(TEXT("resume advances the next canonical frame"),
		WorkCount.Load() > PausedWorkCount);
	Worker.StopAndJoin();
	return true;
}

#endif
