#include "IAmSpeed/Input/ActionDispatch.h"
#include "IAmSpeed/Input/Testing/TestInputProducerV2.h"
#include <cstdlib>
#include <iostream>

using namespace Speed::Input::V2;
using Speed::Input::EProducerKind;
using Speed::Input::FProducerIdentity;
using Speed::Input::FFrameNumber;
using Speed::Input::FActionId;
static unsigned Checks = 0;
static void Check(bool V, const char* Message)
{ ++Checks; if (!V) { std::cerr << "FAIL " << Message << '\n'; std::exit(1); } }

// Authored fixture families, not SkyLeague mechanic implementations or defaults.
static std::shared_ptr<const FInputActionContract> Contract()
{
	FInputActionContractDescription D; D.Revision = {7}; D.Actions = FInputActionContract::BaseActions();
	for (FActionId Id = 3; Id < 6; ++Id)
	{
		FActionDefinition A; A.Id = Id; A.Owner = "Fixture";
		A.Name = Id == 3 ? "Pulse" : Id == 4 ? "Hold" : "ToggleRequest";
		A.Wiring = EActionWiring::Wired; D.Actions.push_back(A);
	}
	D.Mapping = {{{ERawControlKind::PadAxis, 4}, 0, 1}, {{ERawControlKind::PadAxis, 5}, 1, 1},
		{{ERawControlKind::PadAxis, 0}, 2, 1}, {{ERawControlKind::PadButton, 0}, 3, 1},
		{{ERawControlKind::PadButton, 1}, 4, 1}, {{ERawControlKind::PadButton, 2}, 5, 1}};
	D.Physical = {{0, EPhysicalDestination::Throttle}, {1, EPhysicalDestination::Brake}, {2, EPhysicalDestination::Steering}};
	return FInputActionContract::Create(D);
}

// Independent expected canonical states. Frames 4/5 model neutral cancellation
// and fresh held resumption. Host pause/hotplug wiring is NOT exercised here.
static std::vector<FInputFrame> Oracle(const std::shared_ptr<const FInputActionContract>& C, FProducerIdentity Id)
{
	std::vector<FInputFrame> Frames;
	for (FFrameNumber N = 0; N < 8; ++N)
	{
		FInputFrameData D; D.ConsumptionFrame = N; D.Producer = Id; D.StreamEpoch = {9};
		D.DeviceGeneration = {N < 4 ? 1ull : N == 4 ? 2ull : 3ull};
		D.SourceSequence = N < 4 ? N + 1 : N == 4 ? 1 : N - 4;
		D.Reset = N == 0 || N == 4 || N == 5;
		const bool Held = N == 1 || N == 2 || N == 3 || N == 5 || N == 7;
		if (Held) { D.Values[0] = 255; D.Values[1] = 255; D.Values[2] = -127; D.Values[3] = D.Values[4] = D.Values[5] = 1; D.ActiveMask = 63; }
		const auto Edge = [&](bool Start)
		{
			for (const FActionId A : {FActionId{3}, FActionId{4}, FActionId{5}, FActionId{2}, FActionId{0}, FActionId{1}})
				D.Transitions.push_back({A, Start ? ETransition::Started : ETransition::Completed,
					static_cast<std::int16_t>(Start ? (A == 2 ? -127 : A < 2 ? 255 : 1) : 0),
					{D.SourceSequence, static_cast<std::uint32_t>(D.Transitions.size())}});
		};
		if (N == 2) { Edge(false); Edge(true); }
		if (N == 1 || N == 7) Edge(true);
		if (N == 6) Edge(false);
		Frames.emplace_back(C, D);
	}
	return Frames;
}

class FRawFixture final : public IRawInputSource
{
public:
	unsigned Polls = 0;
	std::optional<FRawInputSample> Poll(FFrameNumber N) override
	{
		++Polls; if (N >= 8) return {};
		FRawInputSample S; S.Kind = ERawDeviceKind::Gamepad; S.DeviceId = 17;
		S.Generation = {N < 4 ? 1ull : N == 4 ? 2ull : 3ull};
		S.Sequence = N < 4 ? N + 1 : N == 4 ? 1 : N - 4;
		S.Status = N == 0 || N == 4 || N == 5 ? ERawSampleStatus::Resync : ERawSampleStatus::Valid;
		const float V = N == 0 || N == 4 || N == 6 ? 0.0f : 1.0f;
		S.FinalState = {{{ERawControlKind::PadButton, 0}, V}, {{ERawControlKind::PadButton, 1}, V},
			{{ERawControlKind::PadButton, 2}, V}, {{ERawControlKind::PadAxis, 0}, -V},
			{{ERawControlKind::PadAxis, 4}, V}, {{ERawControlKind::PadAxis, 5}, V}};
		if (N == 2) for (const auto& Value : S.FinalState)
			S.Changes.push_back({{Value.Control, 0}, {S.Sequence, static_cast<std::uint32_t>(S.Changes.size())}});
		if (N == 1 || N == 2 || N == 6 || N == 7) for (const auto& Value : S.FinalState)
			S.Changes.push_back({Value, {S.Sequence, static_cast<std::uint32_t>(S.Changes.size())}});
		return S;
	}
};

// Injected canonical AI/network doubles only: no planner, socket or packet policy.
class FInjectedSource final : public IInputProducer
{
public:
	FInjectedSource(std::shared_ptr<const FInputActionContract> C, std::vector<FInputFrame> F)
		: Config(std::move(C)), Frames(std::move(F)) {}
	std::optional<FInputFrame> Produce(FFrameNumber N) override
	{ ++Calls; return N < Frames.size() ? std::optional<FInputFrame>(Frames[static_cast<std::size_t>(N)]) : std::nullopt; }
	const std::shared_ptr<const FInputActionContract>& GetContract() const override { return Config; }
	unsigned Calls = 0;
private:
	std::shared_ptr<const FInputActionContract> Config;
	std::vector<FInputFrame> Frames;
};

static void Equal(const FInputFrame& A, const FInputFrame& B)
{
	const auto& X = A.GetData(); const auto& Y = B.GetData();
	Check(A.IsValidFor(*B.GetContract()), "contract-valid publication");
	Check(X.ConsumptionFrame == Y.ConsumptionFrame && X.Producer.Kind == Y.Producer.Kind && X.Producer.Id == Y.Producer.Id
		&& X.StreamEpoch.Value == Y.StreamEpoch.Value && X.DeviceGeneration.Value == Y.DeviceGeneration.Value
		&& X.SourceSequence == Y.SourceSequence && X.Reset == Y.Reset, "identity and lifecycle oracle");
	Check(X.Values == Y.Values && X.ActiveMask == Y.ActiveMask && X.Transitions.size() == Y.Transitions.size(), "full action payload oracle");
	for (std::size_t I = 0; I < X.Transitions.size(); ++I)
	{
		const auto& E = X.Transitions[I]; const auto& F = Y.Transitions[I];
		Check(E.Action == F.Action && E.State == F.State && E.ValueAtTransition == F.ValueAtTransition
			&& E.Order.Sequence == F.Order.Sequence && E.Order.WithinSequence == F.Order.WithinSequence, "ordered transition oracle");
	}
}

struct FReceiver
{
	std::shared_ptr<FInputStream> Stream;
	std::vector<std::string> Edges;
	unsigned Triggered = 0, Resets = 0;
	void Action(const FActionEvent& E)
	{
		if (E.State == EStateAction::Triggered) ++Triggered;
		else Edges.push_back(std::to_string(E.Frame.GetData().ConsumptionFrame) + ":" + std::to_string(E.Action)
			+ ":" + std::to_string(static_cast<unsigned>(E.State)) + ":" + std::to_string(E.Value));
		Check(Stream->Consume(E.Frame.GetData().ConsumptionFrame + 1).Status == EConsumeStatus::PresentationForbidden,
			"every action family rejects presentation reinjection");
	}
	void Reset(const FInputFrame&) { ++Resets; }
};

int main()
{
	const auto C = Contract(); Check(bool(C), "fixture contract");
	const std::vector<std::string> ExpectedEdges{
		"1:3:0:1", "1:4:0:1", "1:5:0:1", "1:2:0:-127", "1:0:0:255", "1:1:0:255",
		"2:3:2:0", "2:4:2:0", "2:5:2:0", "2:2:2:0", "2:0:2:0", "2:1:2:0",
		"2:3:0:1", "2:4:0:1", "2:5:0:1", "2:2:0:-127", "2:0:0:255", "2:1:0:255",
		"6:3:2:0", "6:4:2:0", "6:5:2:0", "6:2:2:0", "6:0:2:0", "6:1:2:0",
		"7:3:0:1", "7:4:0:1", "7:5:0:1", "7:2:0:-127", "7:0:0:255", "7:1:0:255"};
	for (unsigned Mode = 0; Mode < 4; ++Mode) for (unsigned Schedule = 0; Schedule < 3; ++Schedule)
	{
		const FProducerIdentity Id{Mode < 2 ? EProducerKind::Device : Mode == 2 ? EProducerKind::AI : EProducerKind::Network, 41};
		const auto Expected = Oracle(C, Id);
		const auto Raw = std::make_shared<FRawFixture>();
		std::shared_ptr<FInjectedSource> Injected;
		std::shared_ptr<IInputProducer> Source;
		if (Mode == 0) Source = FDeviceInputProducer::Create(Raw, C, {9}, Id);
		else if (Mode == 1) Source = FTestInputProducer::Create(C, {9}, Id, 0, Expected);
		else { Injected = std::make_shared<FInjectedSource>(C, Expected); Source = Injected; }
		Check(bool(Source), "common producer setup");
		auto Stream = std::make_shared<FInputStream>(Source, C, FStreamEpoch{9}, Id);
		FInputPresentationBindings Bindings(Stream); auto Receiver = std::make_shared<FReceiver>(); Receiver->Stream = Stream;
		const std::weak_ptr<FReceiver> Weak = Receiver;
		for (FActionId A = 0; A < 6; ++A) for (const auto State : {EStateAction::Started, EStateAction::Triggered, EStateAction::Completed})
			Check(Bindings.BindAction(std::to_string(A) + ":" + std::to_string(static_cast<unsigned>(State)), A, State, Weak, &FReceiver::Action), "bind every family/state");
		Check(Bindings.BindReset("reset", Weak, &FReceiver::Reset) && Bindings.Seal() && Stream->Activate(), "activate shared stream");
		unsigned ExpectedTriggered = 0;
		for (FFrameNumber N = 0; N < 8; ++N)
		{
			const auto Input = Stream->Consume(N);
			Check(Input.Status == EConsumeStatus::Ready && Input.Frame && Input.Targets.Valid, "same consumer for every producer");
			Equal(*Input.Frame, Expected[static_cast<std::size_t>(N)]);
			const bool Held = N == 1 || N == 2 || N == 3 || N == 5 || N == 7;
			Check(Input.Targets.ThrottleValue == (Held ? 255 : 0) && Input.Targets.BrakeValue == (Held ? 255 : 0)
				&& Input.Targets.SteeringValue == (Held ? -127 : 0), "whole physical tuple independent oracle");
			Check(Stream->ConfirmPhysicalCommit(N, true) && Stream->PublishCompleted(N), "common modeled physical commit/publication");
			if (Schedule == 0 || (Schedule == 1 && (N == 0 || N == 3 || N == 7)) || (Schedule == 2 && N == 7))
			{
				if (Held) ExpectedTriggered += 6;
				Check(Bindings.HandleInputs() == EDispatchStatus::Dispatched && Bindings.HandleInputs() == EDispatchStatus::NoChange, "cadence and duplicate observation");
			}
			Equal(*Stream->ReadRecorded(N), Expected[static_cast<std::size_t>(N)]);
		}
		Check(Receiver->Edges == ExpectedEdges && Receiver->Triggered == ExpectedTriggered && Receiver->Resets == 3,
			"all edges survive cadence; triggered is latest-only; resets do not synthesize starts");
		for (FFrameNumber N = 0; N < 8; ++N) Equal(*Stream->ReadRecorded(N), Expected[static_cast<std::size_t>(N)]);
		Check(Mode == 0 ? Raw->Polls == 8 : Raw->Polls == 0, "device acquisition only on physical schedule");
		Check(!Injected || Injected->Calls == 8, "injected source once per physical frame");
		Stream->Deactivate(); Check(Stream->Consume(8).Status == EConsumeStatus::Detached && Bindings.HandleInputs() == EDispatchStatus::Detached, "lifetime cancellation");
	}
	std::cout << "PASS ActionFamiliesProbe checks=" << Checks << '\n';
}
