#include "IAmSpeed/Input/RawAcquisitionJournal.h"
#include "IAmSpeed/Input/InputStreamV2.h"
#include "IAmSpeed/Input/ControlActionReader.h"
#include "IAmSpeed/Input/ControlBatchPolicy.h"
#include "IAmSpeed/Input/ControlApplicationJournal.h"
#include "IAmSpeed/Input/InputAcquisitionWorker.h"
#include "IAmSpeed/Input/SameFrameInputOwner.h"
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <future>
#include <iostream>
#include <stdexcept>

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
		const auto Request = [](EControlCommand Command, std::uint32_t Ordinal)
		{
			FControlRequest R; R.Session = 50; R.Command = Command;
			R.Ordinal = Ordinal; R.State = EStateAction::Started; return R;
		};
		const auto Up = Request(EControlCommand::AutoControl, 0);
		const auto Pause = Request(EControlCommand::Pause, 1);
		const auto Reset = Request(EControlCommand::ResetWorld, 2);
		const auto Verify = [&](std::initializer_list<FControlRequest> Requests,
			bool PausedAtStart, bool PausedNow, bool ExpectSuppressed, const char* Message)
		{
			FControlRequests B; B.Status = EControlRead::Batch;
			B.Requests.assign(Requests);
			FControlApplicationJournal Receipts;
			bool Valid = true;
			for (const auto& R : B.Requests)
			{
				const bool Suppressed = SuppressGameplayControl(B, R, PausedAtStart, PausedNow);
				Valid &= R.Command == EControlCommand::Pause || Suppressed == ExpectSuppressed;
				Valid &= Receipts.Record(R, Suppressed ? EControlApplication::Rejected : EControlApplication::Applied);
			}
			const auto Read = Receipts.Read(0);
			Valid &= Read.Receipts.size() == B.Requests.size();
			for (std::size_t I = 0; I < B.Requests.size(); ++I)
				Valid &= Read.Receipts[I].Result == (B.Requests[I].Command != EControlCommand::Pause && ExpectSuppressed
					? EControlApplication::Rejected : EControlApplication::Applied);
			Check(Valid, Message);
		};
		Verify({Up, Pause, Reset}, false, false, true, "Up before Pause cannot dispatch gameplay; refusals retained");
		Verify({Pause, Up, Reset}, false, false, true, "Pause before Up cannot dispatch gameplay; refusals retained");
		Verify({Pause, Up, Pause}, false, false, true, "two Pause starts do not reopen same-batch gameplay");
		Verify({Up, Reset}, true, false, true, "paused batch remains suppressed after resume request");
		Verify({Up, Reset}, false, false, false, "fresh unpaused batch resumes gameplay");
		FControlRequests Resync; Resync.Status = EControlRead::Resynchronized;
		Check(Resync.Requests.empty() && !SuppressGameplayControl(Resync, Up, false, false),
			"resync contains no invented pause edge or gameplay receipt");
	}
	{
		FRawAcquisitionJournal Hub(33);
		Check(!Hub.FreezeForOwner(0) && !Hub.CloseFrozenCutoff({}),
			"zero cutoff after failed preparation releases no lock");
		Check(Hub.Publish(Hub.BeginAcquisition(), Batch({Reading(1, false, true)})),
			"failed freeze does not obstruct later baseline");
		const auto Cutoff = Hub.FreezeForOwner(0);
		Check(bool(Cutoff), "owner freezes after failed preparation");
		auto WrongThread = std::async(std::launch::async, [&] { return Hub.CloseFrozenCutoff(*Cutoff); });
		Check(WrongThread.wait_for(std::chrono::seconds(2)) == std::future_status::ready && !WrongThread.get(),
			"foreign close rejected without unlocking owner lease");
		Check(Hub.Poll(0) && Hub.CloseFrozenCutoff(*Cutoff), "owner closes after foreign misuse");
		Check(!Hub.CloseFrozenCutoff(*Cutoff), "double close cannot reuse released lease");
	}
	{
		FRawAcquisitionJournal Hub(34);
		Check(Hub.Publish(Hub.BeginAcquisition(), Batch({Reading(1, true, true)})), "shutdown fixture baseline");
		const auto Cutoff = Hub.FreezeForOwner(0);
		Check(Cutoff && Hub.Poll(0), "shutdown fixture frozen cutoff");
		std::atomic<bool> Entered{false};
		auto Shutdown = std::async(std::launch::async, [&]
		{
			Entered = true;
			const auto Cancel = Hub.CancelLifecycle();
			Hub.Close();
			return Cancel == ELifecycleResult::Applied;
		});
		while (!Entered) std::this_thread::yield();
		Check(Shutdown.wait_for(std::chrono::milliseconds(10)) == std::future_status::timeout,
			"concurrent shutdown waits for owner cutoff");
		Check(Hub.CloseFrozenCutoff(*Cutoff), "owner releases before lifecycle shutdown");
		Check(Shutdown.wait_for(std::chrono::seconds(2)) == std::future_status::ready && Shutdown.get(),
			"concurrent cancel and close complete after owner release");
	}
	{
		struct FThrowingRaw final : IRawInputSource
		{
			std::shared_ptr<FRawAcquisitionJournal> Journal;
			explicit FThrowingRaw(std::shared_ptr<FRawAcquisitionJournal> In) : Journal(std::move(In)) {}
			IInputProducerPollFence* PollFence() noexcept override { return Journal.get(); }
			std::optional<FRawInputSample> Poll(Speed::Input::FFrameNumber N) override
			{ (void)Journal->Poll(N); throw std::runtime_error("mapping fixture"); }
		};
		auto Hub = std::make_shared<FRawAcquisitionJournal>(35);
		Check(Hub->Publish(Hub->BeginAcquisition(), Batch({Reading(1, true, true)})),
			"exception fixture baseline");
		auto C = Contract();
		auto Producer = FDeviceInputProducer::Create(std::make_shared<FThrowingRaw>(Hub), C,
			{35}, {Speed::Input::EProducerKind::Device, 35}, 0);
		FOwnerInputBinding Binding{35, {Speed::Input::EProducerKind::Device, 35}, {35}, C, {}};
		auto Owner = FSameFrameInputOwner::Create(std::move(Producer), Binding, 0);
		Check(bool(Owner), "exception fixture owner constructed");
		const auto Rejected = Owner->Poll(0);
		Check(Rejected.Status == EOwnerInputStatus::ResyncRequired
			&& Owner->GetLastPollFailure() == EOwnerPollFailure::Exception,
			"producer exception rejects the frame without publication");
		auto Acquisition = std::async(std::launch::async, [&]
		{ auto Admission = Hub->ReserveAcquisition(); return Admission.owns_lock(); });
		Check(Acquisition.wait_for(std::chrono::seconds(2)) == std::future_status::ready && Acquisition.get(),
			"producer exception closed the journal cutoff lease");
	}
	{
		FRawAcquisitionJournal Hub(31);
		Check(Hub.Publish(Hub.BeginAcquisition(), Batch({Reading(1, true, true)})), "cutoff fixture baseline");
		const auto Cutoff = Hub.FreezeForOwner(0);
		Check(Cutoff && Hub.Poll(0), "physical cutoff freezes first state");
		std::atomic<bool> Entered{false};
		auto Recovery = std::async(std::launch::async, [&]
		{
			Entered = true;
			auto Admission = Hub.ReserveAcquisition();
			const auto Ticket = Hub.BeginAcquisition();
			Hub.Invalidate();
			return Hub.Publish(Ticket, Batch({Reading(2, false, true, 2)}));
		});
		while (!Entered) std::this_thread::yield();
		Check(Recovery.wait_for(std::chrono::milliseconds(10)) == std::future_status::timeout,
			"acquisition recovery waits until frozen physical cutoff closes");
		Check(Hub.CloseFrozenCutoff(*Cutoff), "pre-recovery physical cutoff closes unchanged");
		Check(Recovery.wait_for(std::chrono::seconds(2)) == std::future_status::ready && Recovery.get(),
			"invalidation and fresh baseline publish after cutoff");
		const auto Next = Hub.FreezeForOwner(1);
		const auto Neutral = Hub.Poll(1);
		Check(Next && Neutral && Neutral->Status == ERawSampleStatus::Resync && Neutral->Changes.empty()
			&& Hub.CloseFrozenCutoff(*Next), "next cutoff is one fresh neutral baseline");
	}
	{
		FRawAcquisitionJournal Hub(32);
		Check(Hub.Publish(Hub.BeginAcquisition(), Batch({Reading(1, true, true)})), "pre-recovery fixture baseline");
		auto Admission = Hub.ReserveAcquisition();
		const auto Ticket = Hub.BeginAcquisition();
		Hub.Invalidate();
		auto Physical = std::async(std::launch::async, [&]
		{
			const auto Cutoff = Hub.FreezeForOwner(0);
			const auto Sample = Hub.Poll(0);
			return Cutoff && Sample && Sample->Status == ERawSampleStatus::Resync
				&& Sample->Changes.empty() && Hub.CloseFrozenCutoff(*Cutoff);
		});
		Check(Physical.wait_for(std::chrono::milliseconds(10)) == std::future_status::timeout,
			"physical freeze cannot split invalidation from neutral publish");
		Check(Hub.Publish(Ticket, Batch({Reading(2, false, true, 2)})), "recovery publishes within acquisition transaction");
		Admission.unlock();
		Check(Physical.wait_for(std::chrono::seconds(2)) == std::future_status::ready && Physical.get(),
			"physical poll resumes only after complete recovery");
	}
	{
		FRawAcquisitionJournal Hub(21);
		auto R = Reading(1, true, true); R.Kind = ERawDeviceKind::Desktop;
		Check(R.State.IsValid(R.Kind), "desktop state structurally representable");
		Check(!Hub.Publish(Hub.BeginAcquisition(), Batch({R})) && !Hub.ReadControlBaseline(),
			"legacy journal rejects desktop without versioned provenance lifecycle envelope");
		R.Kind = ERawDeviceKind::Keyboard;
		Check(Hub.Publish(Hub.BeginAcquisition(), Batch({R})), "rejected desktop batch did not advance journal sequence");
	}
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
		const auto Barrier = Hub->Invalidate();
		Check(Barrier.Session == 12 && Barrier.Serial == 3 && Barrier.BarrierBefore == 0
			&& Barrier.BarrierAfter == 4, "invalidation receipt captures barrier change under lock");
		Check(!Hub->ReadControlBaseline() && Reader->Read().Status == EControlRead::WaitingForBaseline, "gap cannot resync from stale cache");
		Check(Hub->Publish(Hub->BeginAcquisition(), Batch({Reading(4, true, true)})), "control recovery fresh held");
		const auto Resync = Reader->Read();
		Check(Resync.Status == EControlRead::Resynchronized && Resync.Requests.empty()
			&& Resync.HistoryStatus == EAcquisitionRead::Gap && Resync.NeedsBaselineBeforeRead
			&& Resync.PreviousSerial == 3 && Resync.LatestSerial == 4 && Resync.BaselineSerial == 4
			&& Resync.ControlBarrier == 4 && Resync.InvalidationBarrier && !Resync.RingOverflow,
			"barrier recovery reports exact cause without synthetic control start");
		Check(Hub->Publish(Hub->BeginAcquisition(), Batch({Reading(5, false), Reading(6, true)})), "post recovery transitions");
		const auto Recovered = Reader->Read();
		Check(Recovered.Requests.size() == 2 && Recovered.Requests[0].State == EStateAction::Completed
			&& Recovered.Requests[1].State == EStateAction::Started, "recovery continues from exact held baseline");
	}
	{
		auto Hub = std::make_shared<FRawAcquisitionJournal>(14);
		auto Reader = FControlActionReader::Create(Contract(), Hub,
			{Speed::Input::EProducerKind::Device, 7}, 14, {{3, EControlCommand::Pause}});
		Check(bool(Reader), "capacity diagnostic reader");
		bool Published = true;
		for (std::uint64_t I = 1; I <= FRawAcquisitionJournal::Capacity + 1; ++I)
			Published &= Hub->Publish(Hub->BeginAcquisition(), Batch({Reading(I, false, true)}));
		Check(Published, "fresh baselines fill control history without physical change overflow");
		const auto Resync = Reader->Read();
		Check(Resync.Status == EControlRead::Resynchronized && Resync.Requests.empty()
			&& Resync.HistoryStatus == EAcquisitionRead::Gap && !Resync.NeedsBaselineBeforeRead
			&& Resync.PreviousSerial == 0 && Resync.LatestSerial == FRawAcquisitionJournal::Capacity + 1
			&& Resync.BaselineSerial == Resync.LatestSerial && Resync.ControlBarrier == 0
			&& Resync.RingOverflow && !Resync.InvalidationBarrier,
			"ring overflow reports exact capacity cause without replay");
	}
	{
		struct FDeferredSource final : IInputAcquisition
		{
			std::mutex Mutex; std::condition_variable Changed;
			bool Entered = false, Release = false;
			EAcquisitionPumpResult Pump() override
			{
				std::unique_lock<std::mutex> Lock(Mutex);
				Entered = true; Changed.notify_all();
				Changed.wait(Lock, [&] { return Release; });
				return EAcquisitionPumpResult::Installed;
			}
			bool Close() override { return true; }
		};
		auto Source = std::make_shared<FDeferredSource>();
		FInputAcquisitionWorker Worker(Source);
		Check(Worker.Start(std::chrono::microseconds(1000)), "deferred acquisition starts without publication");
		{
			std::unique_lock<std::mutex> Lock(Source->Mutex);
			Check(Source->Changed.wait_for(Lock, std::chrono::seconds(2), [&] { return Source->Entered; }),
				"deferred acquisition enters OS pump independently");
		}
		Check(Worker.StartupState(std::chrono::seconds(2)) == EAcquisitionStartup::Waiting,
			"host can service an unpublished acquisition without blocking");
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		Check(Worker.StartupState(std::chrono::milliseconds(1)) == EAcquisitionStartup::Failed,
			"unpublished acquisition fails after configured deadline");
		{ std::lock_guard<std::mutex> Lock(Source->Mutex); Source->Release = true; Source->Changed.notify_all(); }
		Check(Worker.WaitForFirstPublication(std::chrono::seconds(2)), "deferred acquisition eventually publishes");
		Check(Worker.StartupState(std::chrono::seconds(2)) == EAcquisitionStartup::Ready,
			"fresh publication resumes within deadline");
		Check(Worker.StartupState(std::chrono::milliseconds(1)) == EAcquisitionStartup::Failed,
			"late publication cannot retroactively satisfy an expired deadline");
		Check(Worker.Stop() && Worker.StartupState(std::chrono::seconds(2)) == EAcquisitionStartup::Failed,
			"closed acquisition cannot resume gameplay");
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
		Check(Worker.StartupState(std::chrono::seconds(2)) == EAcquisitionStartup::Failed,
			"terminated worker cannot satisfy a previously successful startup");
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
