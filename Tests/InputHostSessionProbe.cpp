#include "IAmSpeed/Input/InputHostSession.h"
#include "IAmSpeed/Input/ControlApplicationJournal.h"
#include "IAmSpeed/Input/Testing/TestInputProducerV2.h"
#include "Car/Input/SkyProducedPlanV2.h"
#include <cstdlib>
#include <iostream>

using namespace Speed::Input::V2;
static unsigned Checks = 0;
static void Check(bool Value, const char* Message)
{ ++Checks; if (!Value) { std::cerr << "FAIL " << Message << '\n'; std::exit(1); } }
static auto Contract()
{
	auto D = Sky::Input::V2::DescribeGameActions({1}, {{{ERawControlKind::KeyboardUsage, 4}, Sky::Input::V2::Jump, 1}});
	return FInputActionContract::Create(D);
}
static FRawAcquisitionBatch Batch(std::uint64_t Sequence, bool Held, bool Fresh = false)
{
	FRawAcquisitionBatch B; B.Count = 1; auto& R = B.Readings[0];
	R.DeviceId = 1; R.Sequence = Sequence; R.TimestampMicroseconds = Sequence; R.Generation = {1};
	R.FreshBaseline = Fresh; R.State.Count = 1;
	R.State.Values[0] = {{ERawControlKind::KeyboardUsage, 4}, Held ? 1.0f : 0.0f}; return B;
}
static FConsumedInput Consume(FInputHostSession& Host, std::uint64_t Frame)
{
	auto Input = Host.Stream->Consume(Frame);
	Check(Input.Status == EConsumeStatus::Ready && Input.Reservation.has_value(), "one physical reservation");
	Check(Host.Stream->ConfirmPhysicalCommit(*Input.Reservation, true), "grouped target confirmation");
	Check(Host.Stream->PreparePublication(*Input.Reservation), "prepare before world publication");
	Check(Host.Stream->FinalizePublication(*Input.Reservation), "canonical commit"); return Input;
}
int main()
{
	const auto C = Contract(); Check(bool(C) && Sky::Input::V2::IsSkyContract(*C), "named Sky and base catalogue");
	{
		auto Hub = std::make_shared<FRawAcquisitionJournal>(88);
		auto Source = FDeviceInputProducer::Create(Hub, C, {88}, {Speed::Input::EProducerKind::Device, 9});
		auto Host = FInputHostSession::Create(std::move(Source), {88}, {Speed::Input::EProducerKind::Device, 9}, 0);
		Check(Host->Activate(), "terminal retirement fixture");
		Check(Host->RetireAtJoinedBoundary() && Host->Closed && Host->TerminalRetired, "explicit terminal retirement outcome");
		Check(!Host->Activate() && !Host->RequestResumeAtBoundary() && !Host->Stream->Consume(0).Reservation,
			"retired session never admits or resumes input");
	}
	Check(C->Find(Sky::Input::V2::SwitchCam)->Type == EActionType::Bool, "SwitchCam explicitly Boolean");
	for (const auto A : {Sky::Input::V2::Jump, Sky::Input::V2::Powerslide, Sky::Input::V2::SwitchCam,
		Sky::Input::V2::AirRoll, Sky::Input::V2::BackCam, Sky::Input::V2::Pause, Sky::Input::V2::ResetWorld, Sky::Input::V2::AutoControl})
		Check(C->Find(A)->Type == EActionType::Bool, "every stored Boolean classified");
	{
		using namespace Sky::Input::V2;
		const auto* Axis = C->Find(AirRollAxis);
		Check(Axis && Axis->Type == EActionType::Axis1D && Axis->Signed && Axis->Quantization == 127,
			"independent roll uses existing signed physical/wire quantization");
		Speed::Input::FActionValues Values{};
		Values[AirRollAxis]=127; Values[AirPitch]=127;
		Check(ResolveAirRollAxis(Values)==127 && Values[AirYaw]==0 && Values[AirPitch]==127,
			"case149 representation keeps roll127 and forward flip yaw0 independent");
		Values[AirRollAxis]=0; Values[AirYaw]=63;
		Check(ResolveAirRollAxis(Values)==0 && Values[AirYaw]==63,"yaw alone cannot generate roll");
		Values[AirRoll]=1;
		Check(ResolveAirRollAxis(Values)==63,"existing digital modifier retains stick behavior");
		for (const auto Roll : {-127,-1,1,127})
		{
			Values[AirRollAxis]=std::int16_t(Roll);
			Check(ResolveAirRollAxis(Values)==Roll && ResolveAirRollAxis(Values,false)==Roll && Values[AirYaw]==63,
				"independent signed roll survives modifier release without changing yaw");
			FInputFrameData Data; Data.Reset=true; Data.StreamEpoch={1}; Data.SourceSequence=1; Data.DeviceGeneration={1};
			Data.Producer={Speed::Input::EProducerKind::Device,1}; Data.Values=Values;
			for (Speed::Input::FActionId Id=0; Id<Speed::Input::ActionCount; ++Id)
				if (Data.Values[Id]) Data.ActiveMask|=std::uint32_t(1)<<Id;
			FInputFrame Frame(C,Data);
			Check(Frame.IsValidFor(*C),"signed roll limits valid in immutable exact frame");
			Data.Values[AirRollAxis]=std::int16_t(Roll<0?-128:128);
			Check(!FInputFrame(C,Data).IsValidFor(*C),"out-of-range analog roll rejected, not silently clamped");
		}
	}
	for (bool RePauseAfterAck : {false, true})
	{
		auto Hub = std::make_shared<FRawAcquisitionJournal>(77);
		auto Source = FDeviceInputProducer::Create(Hub, C, {77}, {Speed::Input::EProducerKind::Device, 9});
		auto Host = FInputHostSession::Create(std::move(Source), {77}, {Speed::Input::EProducerKind::Device, 9}, 0);
		Host->Journal = Hub;
		Check(Host->Activate(), "bind seal activate");
		Check(Hub->Publish(Hub->BeginAcquisition(), Batch(1, false, true)), "initial acquisition");
		Consume(*Host, 0);
		Check(Host->PauseAtBoundary() && Host->PauseAtBoundary(), "idempotent physical pause");
		Check(Host->RequestResumeAtBoundary() && Host->RequestResumeAtBoundary(), "request fresh once");
		Check(!Host->CompleteFreshResume() && !Host->Closed, "waiting distinct from terminal closure");
		const auto OldTicket = Hub->BeginAcquisition();
		if (RePauseAfterAck)
		{
			Check(Hub->Publish(OldTicket, Batch(2, true, true)), "first fresh acknowledgment");
			Check(Hub->IsResumeReady(), "ack ready before second pause");
		}
		Check(Host->PauseAtBoundary(), "re-pause cancels pending resume");
		Check(!Hub->IsResumeReady() && !Host->ResumePending, "re-pause refences acknowledged and pending readings");
		const std::uint64_t Seq = RePauseAfterAck ? 3 : 2;
		Check(Hub->Publish(OldTicket, Batch(Seq, false, true)), "late old acquisition may feed controls");
		Check(!Hub->IsResumeReady(), "old ticket cannot unlock physics");
		Check(Host->RequestResumeAtBoundary(), "second resume request");
		Check(Hub->Publish(Hub->BeginAcquisition(), Batch(Seq + 1, true, true)), "new fresh held reading");
		Check(Host->CompleteFreshResume(), "fresh held resume accepted");
		const auto Resumed = Consume(*Host, 1);
		Check(Resumed.Frame->GetData().Reset && Resumed.Frame->GetData().Values[Sky::Input::V2::Jump] == 1
			&& Resumed.Frame->GetData().Transitions.empty(), "held fresh baseline no synthetic jump");
		Check(Host->CloseAtBoundary() && Host->CloseAtBoundary(), "close idempotent and nonreactivatable");
		Check(!Host->Activate() && !Host->RequestResumeAtBoundary(), "closed session cannot replay");
	}
	{
		FInputFrameData D; D.Producer = {Speed::Input::EProducerKind::Device, 9}; D.StreamEpoch = {2};
		D.DeviceGeneration = {1}; D.SourceSequence = 1; D.Reset = true;
		std::vector<FInputFrame> Frames{FInputFrame(C, D)};
		D.Reset = false; D.ConsumptionFrame = 1; D.SourceSequence = 2;
		for (const auto A : {Sky::Input::V2::Jump, Sky::Input::V2::Powerslide, Sky::Input::V2::SwitchCam})
			for (unsigned Press = 0; Press < 2; ++Press)
			{
				D.Transitions.push_back({A, ETransition::Started, 1, {2, static_cast<std::uint32_t>(D.Transitions.size())}});
				D.Transitions.push_back({A, ETransition::Completed, 0, {2, static_cast<std::uint32_t>(D.Transitions.size())}});
			}
		Frames.emplace_back(C, D);
		auto Test = FTestInputProducer::Create(C, {2}, D.Producer, 0, Frames);
		Check(bool(Test), "sealed same-frame multiple presses accepted");
		auto Host = FInputHostSession::Create(std::move(Test), {2}, D.Producer, 0);
		Check(Host->Activate(), "test uses same host and stream"); Consume(*Host, 0);
		const auto Produced = Consume(*Host, 1);
		const auto Plan = Sky::Input::V2::BuildPlan(*Produced.Frame, 1);
		Check(Plan && Plan->Count == 12, "no coalescing of jump powerslide camera edges");
		for (std::size_t I = 0; I < Plan->Count; ++I)
			Check(Plan->Commands[I].Identity.Ordinal == I && Plan->Commands[I].Identity.Frame == 1, "ordered individual command identity");
		Check(!Sky::Input::V2::BuildPlan(*Produced.Frame, 2), "wrong physical frame rejected");
		Sky::Input::V2::FEffectLedger Ledger; Ledger.Begin(1);
		for (std::size_t I = 0; I < Plan->Count; ++I) Ledger.Append(Plan->Commands[I].Identity, Sky::Input::V2::EEffect::RejectedEligibility);
		Check(Ledger.Read(0).Receipts.empty(), "effect receipts hidden before canonical commit");
		Check(Ledger.CanCommit(1) && Ledger.Commit(1), "effect receipt commit");
		const auto Receipts = Ledger.Read(0);
		Check(Receipts.Receipts.size() == 12 && Ledger.Read(Receipts.Next).Receipts.empty(), "individual outcomes no replay");
		Ledger.Begin(2); Ledger.Append(Plan->Commands[0].Identity, Sky::Input::V2::EEffect::FirstJumpImpulse); Ledger.Abort();
		Check(Ledger.Read(Receipts.Next).Receipts.empty(), "failed world publication exposes no effect receipt");
		Ledger.Begin(3); Ledger.Append(Plan->Commands[0].Identity, Sky::Input::V2::EEffect::Cancelled);
		Check(Ledger.PrepareCommit(3) && Ledger.PrepareCommit(3), "effect commit synchronization prepared once");
		Ledger.FinalizePreparedCommit();
		Check(Ledger.Read(Receipts.Next).Receipts.size() == 1, "prepared effect finalization publishes exactly once");
		Ledger.Begin(3); Ledger.Append(Plan->Commands[0].Identity, Sky::Input::V2::EEffect::Cancelled);
		Check(Ledger.PrepareCommit(3), "prepare abort effect fixture"); Ledger.Abort();
		Check(Ledger.Read(Receipts.Next).Receipts.size() == 1, "aborted prepared effect not published and lock released");
		{
			Sky::Input::V2::FEffectLedger Terminal;
			Check(Terminal.IsIdleAtBoundary(), "new ledger has no owner transaction");
			std::array<std::optional<FInputEdgeIdentity>, 4> Cancelled;
			for (std::size_t I = 0; I < Cancelled.size(); ++I) Cancelled[I] = Plan->Commands[I].Identity;
			auto Invalid = Cancelled; Invalid[3]->Epoch = 0;
			Check(!Terminal.PublishTerminalCancellations(Invalid) && Terminal.Read(0).Receipts.empty(),
				"invalid final identity rejects whole terminal batch, no valid prefix");
			Terminal.Begin(17);
			Check(!Terminal.IsIdleAtBoundary(), "pending ledger forbids cross-thread teardown reset");
			Check(!Terminal.PublishTerminalCancellations(Cancelled), "pending frame prevents terminal boundary publication");
			Terminal.Append(*Cancelled[0], Sky::Input::V2::EEffect::ArmedFirstJump);
			Check(Terminal.PrepareCommit(17), "prepare frame before lifecycle attempt");
			Check(!Terminal.IsIdleAtBoundary(), "prepared ledger forbids cross-thread unlock");
			Check(!Terminal.PublishTerminalCancellations(Cancelled), "prepared frame rejects terminal batch without acquiring held mutex");
			Terminal.Abort();
			Check(Terminal.IsIdleAtBoundary(), "owner abort closes transaction before joined teardown");
			Check(Terminal.Read(0).Receipts.empty(), "failed terminal attempts and aborted frame remain unpublished");
			Check(Terminal.PublishTerminalCancellations(Cancelled), "four cancellations publish without any next physics frame");
			const auto Closed = Terminal.Read(0);
			Check(Closed.Receipts.size() == Cancelled.size(), "one receipt per cancelled effect");
			for (std::size_t I = 0; I < Closed.Receipts.size(); ++I)
				Check(Closed.Receipts[I].Identity == *Cancelled[I]
					&& Closed.Receipts[I].Effect == Sky::Input::V2::EEffect::Cancelled
					&& Closed.Receipts[I].Boundary == Sky::Input::V2::FEffectReceipt::EBoundary::TerminalBoundary
					&& Closed.Receipts[I].AppliedFrame == 0, "terminal outcome retains origin and does not invent an applied frame");
			// The component clears its retained batch only after the ledger accepts it.
			for (auto& Id : Cancelled) Id.reset();
			Check(Terminal.PublishTerminalCancellations(Cancelled) && Terminal.Read(Closed.Next).Receipts.empty(),
				"retry after successful owner acknowledgment adds no duplicate terminal receipt");
		}
		Ledger.Begin(3);
		for (std::size_t I = 0; I <= Sky::Input::V2::FEffectLedger::FrameCapacity; ++I)
			Ledger.Append(Plan->Commands[0].Identity, Sky::Input::V2::EEffect::FirstJumpImpulse);
		Check(!Ledger.CanCommit(3) && !Ledger.Commit(3), "effect capacity overflow rejects whole publication");
		for (std::uint64_t F = 4; F < 4101; ++F)
		{ Ledger.Begin(F); Ledger.Append(Plan->Commands[0].Identity, Sky::Input::V2::EEffect::Cancelled); Check(Ledger.Commit(F), "bounded ledger commit"); }
		Check(Ledger.Read(Receipts.Next).Status == Sky::Input::V2::EReceiptRead::Gap
			&& Ledger.Read(Receipts.Next).Receipts.empty(), "overflow never returns partial suffix");
	}
	{
		FControlApplicationJournal Ledger; FControlRequest R; R.Session = 3;
		Check(Ledger.Record(R, EControlApplication::Rejected), "control refusal retained");
		Check(Ledger.Record(R, EControlApplication::Dispatched), "dispatch is not effect acknowledgment");
		Check(Ledger.Read(0).Receipts[0].Result == EControlApplication::Rejected
			&& Ledger.Read(0).Receipts[1].Result == EControlApplication::Dispatched, "distinct control outcomes");
		for (unsigned I = 0; I < 256; ++I) Check(Ledger.Record(R, EControlApplication::NoChange), "control history append");
		Check(Ledger.Read(0).Gap && Ledger.Read(0).Receipts.empty(), "control receipt overflow explicit");
	}
	std::cout << "PASS " << Checks << " checks; portable host/plan only, no UE gameplay qualification\n";
}
