#include "IAmSpeed/Input/ActionDispatch.h"
#include "IAmSpeed/Input/Testing/TestInputProducerV2.h"
#include <cstdlib>
#include <iostream>
#include <stdexcept>

using namespace Speed::Input::V2;
using Speed::Input::FFrameNumber;
using Speed::Input::EProducerKind;
static unsigned Checks = 0;
static void Check(bool V, const char* Name) { ++Checks; if (!V) { std::cerr << "FAIL " << Name << '\n'; std::exit(1); } }
static std::shared_ptr<const FInputActionContract> Contract()
{
	FInputActionContractDescription D; D.Revision = {1}; D.Actions = FInputActionContract::BaseActions();
	FActionDefinition Fire; Fire.Id = 3; Fire.Owner = "Example"; Fire.Name = "Fire"; Fire.Wiring = EActionWiring::Wired;
	D.Actions.push_back(Fire);
	D.Mapping = {{{ERawControlKind::KeyboardUsage, 4}, 0, 1}, {{ERawControlKind::KeyboardUsage, 4}, 3, 1}};
	D.Physical = {{0, EPhysicalDestination::Throttle}, {1, EPhysicalDestination::Brake}, {2, EPhysicalDestination::Steering}};
	return FInputActionContract::Create(D);
}
static std::vector<FInputFrame> Scenario(const std::shared_ptr<const FInputActionContract>& C, std::size_t Count = 96)
{
	std::vector<FInputFrame> Frames;
	for (std::size_t I = 0; I < Count; ++I)
	{
		FInputFrameData D; D.SourceSequence = I < 4 ? I + 1 : I - 3;
		D.ConsumptionFrame = I; D.Producer = {EProducerKind::Device, 7}; D.StreamEpoch = {1}; D.DeviceGeneration = {I < 4 ? 1ull : 2ull};
		D.Reset = I == 0 || I == 4;
		const bool Held = I >= 1 && I <= 4;
		D.Values[0] = Held ? 255 : 0; D.Values[3] = Held ? 1 : 0; D.ActiveMask = Held ? 9 : 0;
		if (I == 1) D.Transitions = {{0, ETransition::Started, 255, {2, 0}}, {3, ETransition::Started, 1, {2, 1}}};
		if (I == 2) D.Transitions = {{0, ETransition::Completed, 0, {3, 0}}, {3, ETransition::Completed, 0, {3, 1}},
			{0, ETransition::Started, 255, {3, 2}}, {3, ETransition::Started, 1, {3, 3}}};
		if (I == 5) D.Transitions = {{0, ETransition::Completed, 0, {2, 0}}, {3, ETransition::Completed, 0, {2, 1}}};
		Frames.emplace_back(C, D);
	}
	return Frames;
}
class FRawScenario final : public IRawInputSource
{
public:
	unsigned Polls = 0;
	std::optional<FRawInputSample> Poll(FFrameNumber Frame) override
	{
		++Polls;
		FRawInputSample S; S.DeviceId = 20; S.Generation = {Frame < 4 ? 1ull : 2ull};
		S.Sequence = Frame < 4 ? Frame + 1 : Frame - 3;
		S.Status = Frame == 0 || Frame == 4 ? ERawSampleStatus::Resync : ERawSampleStatus::Valid;
		S.FinalState = {{{ERawControlKind::KeyboardUsage, 4}, Frame >= 1 && Frame <= 4 ? 1.0f : 0.0f}};
		if (Frame == 1 || Frame == 5) S.Changes = {{S.FinalState[0], {S.Sequence, 0}}};
		if (Frame == 2) S.Changes = {{{{ERawControlKind::KeyboardUsage, 4}, 0}, {3, 0}}, {S.FinalState[0], {3, 1}}};
		return S;
	}
};
class FInvalidSource final : public IInputProducer
{
public:
	FInvalidSource(std::shared_ptr<const FInputActionContract> C, std::optional<FInputFrame> F)
		: Config(std::move(C)), Frame(std::move(F)) {}
	std::optional<FInputFrame> Produce(FFrameNumber) override { return Frame; }
	const std::shared_ptr<const FInputActionContract>& GetContract() const override { return Config; }
private:
	std::shared_ptr<const FInputActionContract> Config;
	std::optional<FInputFrame> Frame;
};
static std::shared_ptr<FInputStream> TestStream(const std::shared_ptr<const FInputActionContract>& C, std::size_t Count = 96)
{
	std::shared_ptr<IInputProducer> P = FTestInputProducer::Create(C, {1}, {EProducerKind::Device, 7}, 0, Scenario(C, Count));
	Check(bool(P), "sealed source factory");
	return std::make_shared<FInputStream>(P, C, FStreamEpoch{1}, Speed::Input::FProducerIdentity{EProducerKind::Device, 7});
}
// Both PLAYER and TEST invoke this exact consumer with an IInputProducer stream.
// The native target assignment models the grouped commit, not a UE simulation.
static Speed::Input::FDrivingInputTargets ConsumeAndCommit(FInputStream& Stream, FFrameNumber N)
{
	const auto Before = Stream.ReadLatest();
	const auto Input = Stream.Consume(N);
	Check(Input.Status == EConsumeStatus::Ready && Input.Frame && Input.Targets.Valid, "common consume and sink");
	Check(!Stream.PublishCompleted(N), "uncommitted physical input cannot publish");
	const auto Recorded = Stream.ReadRecorded(N);
	Check(Recorded && Recorded->GetData().ConsumptionFrame == N, "consumed history records addressed frame");
	const auto Still = Stream.ReadLatest();
	Check((!Before && !Still) || (Before && Still && Before->Serial == Still->Serial), "latch not visible as completed");
	const auto PhysicalTargets = Input.Targets; // Grouped assignment at native consumer boundary.
	Check(Stream.ConfirmPhysicalCommit(N, true) && Stream.PublishCompleted(N), "publish only after successful commit");
	const auto Serial = Stream.ReadLatest()->Serial;
	Check(Stream.PublishCompleted(N) && Stream.ReadLatest()->Serial == Serial, "publication idempotent");
	return PhysicalTargets;
}
struct FReceiver
{
	std::vector<std::string> Events;
	std::shared_ptr<FInputStream> Stream;
	FInputPresentationBindings* Bindings = nullptr;
	bool DetachOnStart = false, ThrowOnStart = false;
	void Action(const FActionEvent& E)
	{
		Events.push_back(std::to_string(E.Frame.GetData().ConsumptionFrame) + ":" + std::to_string(E.Action)
			+ ":" + std::to_string(static_cast<unsigned>(E.State)) + ":" + std::to_string(E.Value));
		if (Bindings)
		{
			Check(Stream->ReadLatest().has_value(), "callback reads stream after publication lock is released");
			Check(!Bindings->Unbind("start") && !Bindings->BindSnapshotAction("late", 0, [](const FInputFrame&, Speed::Input::FActionId) {}), "binding mutation rejected during dispatch");
			Check(Bindings->HandleInputs() == EDispatchStatus::Reentrant, "recursive dispatch rejected");
			Check(Stream->Consume(E.Frame.GetData().ConsumptionFrame + 1).Status == EConsumeStatus::PresentationForbidden
				&& !Stream->ConfirmPhysicalCommit(E.Frame.GetData().ConsumptionFrame, true)
				&& !Stream->PublishCompleted(E.Frame.GetData().ConsumptionFrame), "callbacks cannot mutate canonical stream");
		}
		if (E.State == EStateAction::Started && DetachOnStart) Stream->Deactivate();
		if (E.State == EStateAction::Started && ThrowOnStart) throw std::runtime_error("intentional receiver failure");
	}
	void Reset(const FInputFrame& F) { Events.push_back("reset:" + std::to_string(F.GetData().ConsumptionFrame)); }
};
int main()
{
	const auto C = Contract(); Check(bool(C), "contract");
	const auto Raw = std::make_shared<FRawScenario>();
	std::shared_ptr<IInputProducer> Device = FDeviceInputProducer::Create(Raw, C, {1}, {EProducerKind::Device, 7});
	auto Player = std::make_shared<FInputStream>(Device, C, FStreamEpoch{1}, Speed::Input::FProducerIdentity{EProducerKind::Device, 7});
	auto Test = TestStream(C);
	FInputPresentationBindings Bindings(Player); auto Receiver = std::make_shared<FReceiver>(); Receiver->Stream = Player; Receiver->Bindings = &Bindings;
	const std::weak_ptr<FReceiver> Weak = Receiver;
	Check(Bindings.BindReset("reset", Weak, &FReceiver::Reset), "reset binding");
	Check(Bindings.BindAction("start", 0, EStateAction::Started, Weak, &FReceiver::Action), "started binding");
	Check(Bindings.BindAction("complete", 0, EStateAction::Completed, Weak, &FReceiver::Action), "completed binding");
	Check(Bindings.BindAction("triggered", 0, EStateAction::Triggered, Weak, &FReceiver::Action), "triggered binding");
	Check(Bindings.BindAction("second-start", 0, EStateAction::Started, Weak, &FReceiver::Action), "multiple handlers stable registration order");
	std::vector<FFrameNumber> Snapshots;
	Check(Bindings.BindAction("compat", 0, [&](const FInputFrame& F, Speed::Input::FActionId) { Snapshots.push_back(F.GetData().ConsumptionFrame); }), "snapshot compatibility alias");
	Check(Bindings.Seal() && Player->Activate() && Test->Activate(), "seal before activation");
	Check(!Bindings.Unbind("start") && !Bindings.Seal(), "registration permanently sealed");
	for (FFrameNumber N = 0; N < 96; ++N)
	{
		const auto A = ConsumeAndCommit(*Player, N); const auto B = ConsumeAndCommit(*Test, N);
		Check(A.ThrottleValue == B.ThrottleValue && A.BrakeValue == B.BrakeValue && A.SteeringValue == B.SteeringValue, "same physical targets for raw and sealed input");
		const auto PF = Player->ReadRecorded(N); const auto TF = Test->ReadRecorded(N);
		Check(PF->GetData().Values == TF->GetData().Values && PF->GetData().ActiveMask == TF->GetData().ActiveMask
			&& PF->GetData().Transitions.size() == TF->GetData().Transitions.size(), "same canonical snapshot shape");
		Check(PF->GetData().Reset == TF->GetData().Reset && PF->GetData().SourceSequence == TF->GetData().SourceSequence
			&& PF->GetData().DeviceGeneration.Value == TF->GetData().DeviceGeneration.Value, "same reset/source lifecycle");
		for (std::size_t I = 0; I < PF->GetData().Transitions.size(); ++I)
		{
			const auto& X = PF->GetData().Transitions[I]; const auto& Y = TF->GetData().Transitions[I];
			Check(X.Action == Y.Action && X.State == Y.State && X.ValueAtTransition == Y.ValueAtTransition
				&& X.Order.Sequence == Y.Order.Sequence && X.Order.WithinSequence == Y.Order.WithinSequence, "same ordered canonical transitions");
		}
		if (N == 0 || N == 3 || N == 4 || N == 63 || N == 95)
		{
			Check(Bindings.HandleInputs() == EDispatchStatus::Dispatched, "published batch dispatch");
			const auto Count = Receiver->Events.size();
			Check(Bindings.HandleInputs() == EDispatchStatus::NoChange && Receiver->Events.size() == Count, "repeat presentation tick deduplicated");
		}
	}
	Check(Raw->Polls == 96, "one device poll per forward frame");
	const std::vector<std::string> Expected{"reset:0", "1:0:0:255", "1:0:0:255", "2:0:2:0", "2:0:0:255", "2:0:0:255", "3:0:1:255",
		"reset:4", "4:0:1:255", "5:0:2:0"};
	Check(Receiver->Events == Expected, "drained edges ordered before latest-only triggered, reset never synthesizes completion");
	Check(Snapshots == std::vector<FFrameNumber>({0, 3, 4, 63, 95}), "neutral latest snapshot observations preserved");
	Check(Player->Consume(0).Frame.has_value() && Raw->Polls == 96 && Player->PublishCompleted(0)
		&& Player->ReadLatest()->Serial == 96, "retained replay does not poll or republish older frame");
	const auto Batch = Player->ReadPublishedSince({{1}, 0});
	Check(Batch.Status == EReadStatus::Batch && Batch.Frames.size() == 96 && Batch.Next.Serial == 96, "coherent complete published batch");
	Check(Player->ReadPublishedSince({{1}, 97}).Status == EReadStatus::InvalidCursor
		&& Player->ReadPublishedSince({{2}, 0}).Status == EReadStatus::Detached, "future cursor and stale epoch rejected");

	// Separate snapshot compatibility schedule: exactly the original 0/63/95.
	auto Compatibility = TestStream(C); FInputPresentationBindings Compat(Compatibility); std::vector<FFrameNumber> CompatFrames;
	Check(Compat.BindSnapshotAction("old", 0, [&](const FInputFrame& F, Speed::Input::FActionId) { CompatFrames.push_back(F.GetData().ConsumptionFrame); })
		&& Compat.Seal() && Compatibility->Activate(), "snapshot compatibility setup");
	for (FFrameNumber N = 0; N < 96; ++N) { ConsumeAndCommit(*Compatibility, N); if (N == 0 || N == 63 || N == 95) Compat.HandleInputs(); }
	Check(CompatFrames == std::vector<FFrameNumber>({0, 63, 95}), "compatibility schedule unchanged");

	auto Overflow = TestStream(C, 301); FInputPresentationBindings Slow(Overflow); unsigned Delivered = 0;
	Check(Slow.BindSnapshotAction("slow", 0, [&](const FInputFrame&, Speed::Input::FActionId) { ++Delivered; }) && Slow.Seal() && Overflow->Activate(), "overflow setup");
	for (FFrameNumber N = 0; N < 300; ++N) ConsumeAndCommit(*Overflow, N);
	const auto Lost = Overflow->ReadPublishedSince({{1}, 0});
	Check(Lost.Status == EReadStatus::Overflow && Lost.Frames.empty(), "overflow no partial batch");
	Check(Slow.HandleInputs() == EDispatchStatus::ResyncRequired && Delivered == 0, "no callbacks on overflow");
	Check(Slow.Resynchronize() == EDispatchStatus::Resynchronized && Delivered == 0, "explicit baseline without invented events");
	ConsumeAndCommit(*Overflow, 300); Check(Slow.HandleInputs() == EDispatchStatus::Dispatched && Delivered == 1, "resume after explicit resync");

	for (unsigned Mode = 0; Mode < 3; ++Mode)
	{
		auto S = TestStream(C); FInputPresentationBindings B(S); auto R = std::make_shared<FReceiver>(); R->Stream = S;
		R->DetachOnStart = Mode == 0; R->ThrowOnStart = Mode == 1;
		Check(B.BindAction("first", 0, EStateAction::Started, std::weak_ptr<FReceiver>(R), &FReceiver::Action), "weak binding setup");
		unsigned Later = 0; Check(B.BindSnapshotAction("later", 0, [&](const FInputFrame&, Speed::Input::FActionId) { ++Later; }) && B.Seal() && S->Activate(), "later binding setup");
		ConsumeAndCommit(*S, 0); ConsumeAndCommit(*S, 1);
		if (Mode == 2) R.reset();
		const auto Status = B.HandleInputs();
		Check(Mode == 2 ? (Status == EDispatchStatus::Dispatched && Later == 1) : (Status == EDispatchStatus::Detached && Later == 0 && !S->IsActive()),
			"expired receiver skipped; detach/exception stops remaining dispatch");
	}
	auto Failed = TestStream(C); Check(Failed->Activate() && Failed->Consume(0).Targets.Valid, "failure commit setup");
	Check(!Failed->ConfirmPhysicalCommit(0, false) && !Failed->PublishCompleted(0) && !Failed->ReadLatest()
		&& !Failed->Consume(1).Targets.Valid && !Failed->Activate(), "failed commit neutralizes and permanently detaches");
	auto OtherDescription = C->GetDescription(); OtherDescription.Revision = {2};
	const auto OtherContract = FInputActionContract::Create(OtherDescription);
	for (unsigned Mode = 0; Mode < 6; ++Mode)
	{
		auto D = Scenario(C, 1)[0].GetData();
		if (Mode == 0) D.Reset = false;
		if (Mode == 1) D.ConsumptionFrame = 1;
		if (Mode == 2) D.StreamEpoch = {2};
		if (Mode == 3) D.Producer.Id = 100;
		std::optional<FInputFrame> BadFrame = FInputFrame(Mode == 4 ? OtherContract : C, D);
		if (Mode == 5) BadFrame.reset();
		auto Source = std::make_shared<FInvalidSource>(C, BadFrame);
		FInputStream Invalid(Source, C, {1}, {EProducerKind::Device, 7});
		Check(Invalid.Activate(), "invalid frame fixture activates matching producer contract");
		const auto Rejected = Invalid.Consume(0);
		Check(Rejected.Status == EConsumeStatus::InvalidInput && !Rejected.Frame && !Rejected.Targets.Valid
			&& Rejected.Targets.ThrottleValue == 0 && !Invalid.PublishCompleted(0) && !Invalid.ReadLatest(), "invalid source fails neutral without publication");
	}
	auto WrongContractSource = std::make_shared<FInvalidSource>(OtherContract, Scenario(C, 1)[0]);
	FInputStream WrongContract(WrongContractSource, C, {1}, {EProducerKind::Device, 7});
	Check(!WrongContract.Activate(), "producer contract mismatch fails before activation");
	std::cout << "PASS ActionStreamProbe checks=" << Checks << '\n';
}
