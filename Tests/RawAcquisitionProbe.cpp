#include "IAmSpeed/Input/RawAcquisitionJournal.h"
#include "IAmSpeed/Input/InputStreamV2.h"
#include "IAmSpeed/Input/ControlActionReader.h"
#include "IAmSpeed/Input/InputAcquisitionWorker.h"
#include <atomic>
#include <cstdlib>
#include <iostream>

using namespace Speed::Input::V2;
static unsigned Checks = 0;
static void Check(bool Value, const char* Message)
{ ++Checks; if (!Value) { std::cerr << "FAIL " << Message << '\n'; std::exit(1); } }
static FAcquiredRawState Reading(std::uint64_t Sequence, bool Held, bool Fresh = false, std::uint64_t Generation = 1)
{
	FAcquiredRawState R; R.DeviceId = 1; R.Sequence = Sequence; R.TimestampMicroseconds = Sequence;
	R.Generation = {Generation}; R.FreshBaseline = Fresh; R.State.Count = 1;
	R.State.Values[0] = {{ERawControlKind::KeyboardUsage, 4}, Held ? 1.0f : 0.0f}; return R;
}
static FRawAcquisitionBatch Batch(std::initializer_list<FAcquiredRawState> Readings)
{
	FRawAcquisitionBatch B; for (const auto& R : Readings) B.Readings[B.Count++] = R; return B;
}
static auto Contract()
{
	FInputActionContractDescription D; D.Revision = {1}; D.Actions = FInputActionContract::BaseActions();
	FActionDefinition A; A.Id = 3; A.Owner = "Fixture"; A.Name = "Button"; A.Wiring = EActionWiring::Wired;
	D.Actions.push_back(A); D.Mapping = {{{ERawControlKind::KeyboardUsage, 4}, 3, 1}};
	D.Physical = {{0, EPhysicalDestination::Throttle}, {1, EPhysicalDestination::Brake}, {2, EPhysicalDestination::Steering}};
	return FInputActionContract::Create(D);
}
int main()
{
	{
		auto D = Contract()->GetDescription(); D.Mapping.push_back({{ERawControlKind::KeyboardUsage, 5}, 3, 1});
		auto C = FInputActionContract::Create(D); Check(bool(C), "OR binding fixture");
		FActionMapper Mapper(C, {1}, {Speed::Input::EProducerKind::Device, 7});
		FRawAcquisitionJournal Hub(13);
		auto A = Reading(1, true, true); A.State.Count = 2; A.State.Values[1] = {{ERawControlKind::KeyboardUsage, 5}, 0};
		Check(Hub.Publish(Hub.BeginAcquisition(), Batch({A})), "atomic OR baseline");
		Check(bool(Mapper.Map(*Hub.Poll(0), 0).Frame), "OR baseline mapped");
		auto B = A; B.Sequence = B.TimestampMicroseconds = 2; B.FreshBaseline = false;
		B.State.Values[0].Value = 0; B.State.Values[1].Value = 1;
		Check(Hub.Publish(Hub.BeginAcquisition(), Batch({B})), "simultaneous OR handover acquisition");
		const auto M = Mapper.Map(*Hub.Poll(1), 1);
		Check(M.Frame && M.Frame->GetData().Values[3] == 1 && M.Frame->GetData().Transitions.empty(), "atomic reading creates no false completed started on OR handover");
	}
	{
		auto Hub = std::make_shared<FRawAcquisitionJournal>(12);
		auto Reader = FControlActionReader::Create(Contract(), Hub, {Speed::Input::EProducerKind::Device, 7}, 12, {{3, EControlCommand::Pause}});
		Check(bool(Reader), "control actions use same action contract");
		Check(Hub->Publish(Hub->BeginAcquisition(), Batch({Reading(1, false, true), Reading(2, true), Reading(3, false)})), "control startup batch");
		const auto First = Reader->Read();
		Check(First.Status == EControlRead::Batch && First.Requests.size() == 2
			&& First.Requests[0].State == EStateAction::Started && First.Requests[1].State == EStateAction::Completed
			&& First.Requests[0].AcquisitionSequence == 2 && First.Requests[0].Session == 12, "first observer retains short press with acquisition identity");
		Check(Reader->Read().Status == EControlRead::NoChange, "control cursor no duplicate requests");
		Hub->Invalidate();
		Check(!Hub->ReadControlBaseline() && Reader->Read().Status == EControlRead::WaitingForBaseline, "gap cannot resync from stale cache");
		Check(Hub->Publish(Hub->BeginAcquisition(), Batch({Reading(4, true, true)})), "control recovery fresh held");
		Check(Reader->Read().Status == EControlRead::Resynchronized && Reader->Read().Requests.empty(), "atomic state cursor resync emits no synthetic start");
		Check(Hub->Publish(Hub->BeginAcquisition(), Batch({Reading(5, false), Reading(6, true)})), "post recovery transitions");
		const auto Recovered = Reader->Read();
		Check(Recovered.Requests.size() == 2 && Recovered.Requests[0].State == EStateAction::Completed
			&& Recovered.Requests[1].State == EStateAction::Started, "recovery continues from exact held baseline");
	}
	{
		struct FBlockingSource final : IInputAcquisition
		{
			std::mutex Mutex; std::condition_variable Changed;
			unsigned Calls = 0; bool Release = false; std::atomic<bool> Closed{false};
			FInputAcquisitionWorker* Worker = nullptr; bool SelfStop = true;
			EAcquisitionPumpResult Pump() override
			{
				std::unique_lock<std::mutex> Lock(Mutex); ++Calls; Changed.notify_all();
				if (Calls == 2) { Changed.wait(Lock, [&] { return Release; }); SelfStop = Worker->Stop(); }
				return EAcquisitionPumpResult::Installed;
			}
			bool Close() override { Closed = true; return true; }
		};
		auto Source = std::make_shared<FBlockingSource>(); FInputAcquisitionWorker Worker(Source);
		Source->Worker = &Worker;
		Check(!Worker.Start(std::chrono::microseconds(0)) && Worker.Start(std::chrono::microseconds(1000)), "explicit valid acquisition cadence");
		Check(Worker.WaitForFirstPublication(std::chrono::milliseconds(2000)), "acquisition startup published independently");
		{
			std::unique_lock<std::mutex> Lock(Source->Mutex);
			Check(Source->Changed.wait_for(Lock, std::chrono::seconds(2), [&] { return Source->Calls >= 2; }), "acquisition advances without UE or physics ticks");
		}
		std::atomic<bool> Stopped{false}; bool StopResult = false;
		std::thread Stopper([&] { StopResult = Worker.Stop(); Stopped = true; });
		const auto StopDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
		while (!Worker.IsStopRequested() && std::chrono::steady_clock::now() < StopDeadline) std::this_thread::yield();
		Check(Worker.IsStopRequested(), "external stop owns join fence");
		Check(!Source->Closed && !Stopped, "inflight acquisition not destroyed early");
		{ std::lock_guard<std::mutex> Lock(Source->Mutex); Source->Release = true; Source->Changed.notify_all(); }
		Stopper.join();
		Check(StopResult && Source->Closed && Stopped && !Source->SelfStop && Worker.Stop(), "self-stop refuses before external join mutex and external stop closes idempotently");
	}
	{
		struct FThrowingSource final : IInputAcquisition
		{
			std::shared_ptr<FRawAcquisitionJournal> Hub = std::make_shared<FRawAcquisitionJournal>(14);
			unsigned Calls = 0; std::mutex Mutex; std::condition_variable Changed; bool Closed = false;
			EAcquisitionPumpResult Pump() override
			{
				if (++Calls > 1) throw 1;
				return Hub->Publish(Hub->BeginAcquisition(), Batch({Reading(1, true, true)}))
					? EAcquisitionPumpResult::Installed : EAcquisitionPumpResult::Rejected;
			}
			bool Close() override
			{
				Hub->Close(); std::lock_guard<std::mutex> Lock(Mutex); Closed = true; Changed.notify_all(); return true;
			}
		};
		auto Source = std::make_shared<FThrowingSource>(); FInputAcquisitionWorker Worker(Source);
		Check(Worker.Start(std::chrono::microseconds(1000)), "failing owner starts");
		{
			std::unique_lock<std::mutex> Lock(Source->Mutex);
			Check(Source->Changed.wait_for(Lock, std::chrono::seconds(2), [&] { return Source->Closed; }), "terminal pump exception closes source before host stop");
		}
		Check(!Source->Hub->Poll(0) && Source->Hub->ReadControlsSince({14, 0}).Status == EAcquisitionRead::Closed,
			"held value cannot survive terminal acquisition failure");
		Check(Worker.Stop(), "failed owner teardown joined");
	}
	{
		auto Hub = std::make_shared<FRawAcquisitionJournal>(1);
		Check(Hub->Publish(Hub->BeginAcquisition(), Batch({Reading(1, false, true), Reading(2, true), Reading(3, false)})), "atomic baseline press release batch");
		auto C = Contract(); Check(bool(C), "action contract");
		auto Device = FDeviceInputProducer::Create(Hub, C, {1}, {Speed::Input::EProducerKind::Device, 7});
		std::shared_ptr<IInputProducer> Producer = std::move(Device);
		FInputStream Stream(Producer, C, {1}, {Speed::Input::EProducerKind::Device, 7});
		Check(Stream.Activate(), "shared physical stream activated");
		auto First = Stream.Consume(0);
		Check(First.Reservation && First.Frame->GetData().Reset && First.Frame->GetData().Transitions.empty(), "baseline contains no synthetic edge");
		Check(Stream.SetLifecyclePaused(true) == ELifecycleResult::Rejected, "pending reservation prevents lifecycle mutation");
		Check(Stream.ConfirmPhysicalCommit(*First.Reservation, true) && Stream.PreparePublication(*First.Reservation), "prepare accepted frame");
		Check(Stream.SetLifecyclePaused(true) == ELifecycleResult::Rejected, "prepared publication still prevents pause");
		Check(Stream.FinalizePublication(*First.Reservation), "finalize reserved frame");
		auto Next = Stream.Consume(1);
		Check(Next.Reservation && Next.Frame->GetData().Transitions.size() == 2
			&& Next.Frame->GetData().Transitions[0].State == ETransition::Started
			&& Next.Frame->GetData().Transitions[1].State == ETransition::Completed
			&& Next.Frame->GetData().Values[3] == 0, "real short press survives neutral final value and baseline split");
		Check(Stream.ConfirmPhysicalCommit(*Next.Reservation, true) && Stream.PublishCompleted(*Next.Reservation), "publish full short press once");
		Check(Stream.Consume(1).Status == EConsumeStatus::PublishedReplay && !Hub->Poll(1), "replay has no second raw consumption");
		auto Controls = Hub->ReadControlsSince({1, 0});
		Check(Controls.Status == EAcquisitionRead::Batch && Controls.Readings.size() == 3, "control history independent of physics reads");
		Check(Stream.SetLifecyclePaused(true) == ELifecycleResult::Applied, "pause after owner commit");
		Check(Hub->Publish(Hub->BeginAcquisition(), Batch({Reading(4, true), Reading(5, false)})), "acquisition continues during pause");
		Check(Stream.Consume(2).Status == EConsumeStatus::Paused, "physical frame unchanged while paused");
		Controls = Hub->ReadControlsSince(Controls.Next);
		Check(Controls.Readings.size() == 2 && Controls.Readings[0].State.Values[0].Value == 1, "paused short control press retained");
		const auto OldTicket = Hub->BeginAcquisition();
		Check(Hub->RequestFreshResume() && !Hub->IsResumeReady(), "resume asks independent acquisition fence");
		Check(Hub->Publish(OldTicket, Batch({Reading(6, true, true)})) && !Hub->IsResumeReady(), "inflight old acquisition may update control but cannot acknowledge resume");
		Check(Hub->Publish(Hub->BeginAcquisition(), Batch({Reading(7, true, true)})) && Hub->IsResumeReady(), "new current reading acknowledges resume");
		Check(Hub->Publish(Hub->BeginAcquisition(), Batch({Reading(8, false), Reading(9, true)})), "post acknowledged resume edges before worker wake");
		Check(Stream.SetLifecyclePaused(false) == ELifecycleResult::Applied, "resume only after fresh acquisition");
		auto Resumed = Stream.Consume(2);
		Check(Resumed.Reservation && Resumed.Frame->GetData().Reset && Resumed.Frame->GetData().Values[3] == 1
			&& Resumed.Frame->GetData().Transitions.empty(), "fresh held resume at unchanged frame without paused edges");
		Check(Stream.ConfirmPhysicalCommit(*Resumed.Reservation, true) && Stream.PublishCompleted(*Resumed.Reservation), "resume baseline publication");
		auto PostResume = Stream.Consume(3);
		Check(PostResume.Reservation && PostResume.Frame->GetData().Transitions.size() == 2, "delayed worker wake cannot discard post resume requests");
		Check(Stream.Abort(*PostResume.Reservation, EAbortReason::Cancelled) && Stream.CancelSourceAtBoundary() == ELifecycleResult::Applied, "abort drains before cancellation");
		Check(Hub->Publish(Hub->BeginAcquisition(), Batch({Reading(10, false)})) && !Hub->Poll(4), "physical cancellation leaves independent controls alive");
		Check(Hub->ReadControlsSince(Controls.Next).Readings.size() == 5, "controls do not replay physical edges");
		Hub->Close(); Check(Hub->ReadControlsSince({1, 0}).Status == EAcquisitionRead::Closed, "acquisition lifetime closure explicit");
	}
	for (unsigned Count : {255u, 256u, 257u})
	{
		FRawAcquisitionJournal Hub(2);
		Check(Hub.Publish(Hub.BeginAcquisition(), Batch({Reading(1, false, true)})) && bool(Hub.Poll(0)), "capacity baseline");
		bool Accepted = true;
		for (unsigned I = 0; I < Count; ++I)
			if (!Hub.Publish(Hub.BeginAcquisition(), Batch({Reading(I + 2, I % 2 == 0)}))) { Accepted = false; break; }
		Check(Accepted == (Count <= 256), "raw pending changes 255 256 257 bound");
		if (Accepted)
		{
			const auto Raw = Hub.Poll(1); Check(Raw && Raw->Changes.size() == Count && Raw->IsValid(), "all pending changes delivered in order");
		}
		else
		{
			Hub.Invalidate();
			Check(!Hub.Poll(1), "overflow caller invalidates entire delivery before physical read");
			Check(Hub.ReadControlsSince({2, 257}).Status == EAcquisitionRead::Gap, "failure is explicit even at latest control cursor");
			Check(Hub.Publish(Hub.BeginAcquisition(), Batch({Reading(258, true, true)})), "fresh overflow recovery");
			const auto Raw = Hub.Poll(1); Check(Raw && Raw->Status == ERawSampleStatus::Resync && Raw->Changes.empty(), "no overflow replay");
		}
		Check(Hub.ReadControlsSince({2, 0}).Status == (Count == 255 ? EAcquisitionRead::Batch : EAcquisitionRead::Gap), "control ring exact capacity versus overflow");
	}
	{
		FRawAcquisitionJournal Hub(3);
		Check(Hub.Publish(Hub.BeginAcquisition(), Batch({Reading(1, false, true)})), "rollback fixture baseline");
		auto Bad = Batch({Reading(2, true), Reading(3, false)}); Bad.Readings[1].State.Values[0].Value = 0.5f;
		Check(!Hub.Publish(Hub.BeginAcquisition(), Bad), "invalid later reading rejects whole batch");
		Check(Hub.ReadControlsSince({3, 1}).Status == EAcquisitionRead::NoChange, "no control prefix on rejected batch");
		Check(Hub.Poll(0)->FinalState[0].Value == 0 && Hub.Poll(1)->Changes.empty(), "no physical prefix on rejected batch");
		Check(!Hub.Publish({4, 1}, Batch({Reading(2, true, true)})), "replacement session refuses stale acquisition ticket");
	}
	std::cout << "PASS RawAcquisitionProbe checks=" << Checks << " hardware=none gameplay=none\n";
}
