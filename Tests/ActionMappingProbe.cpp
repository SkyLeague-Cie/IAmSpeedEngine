#include "IAmSpeed/Input/ActionMapping.h"
#include "IAmSpeed/Input/PhysicalActionSink.h"
#include "IAmSpeed/Input/Testing/TestInputProducerV2.h"
#include <cstdlib>
#include <iostream>

using namespace Speed::Input::V2;
using Speed::Input::Throttle;
using Speed::Input::Brake;
using Speed::Input::Steering;
using Speed::Input::FFrameNumber;
using Speed::Input::FActionValues;
using Speed::Input::EProducerKind;
static unsigned Checks = 0;
static void Check(bool Value, const char* Name)
{ ++Checks; if (!Value) { std::cerr << "FAIL " << Name << '\n'; std::exit(1); } }
static FRawControl Key(std::uint16_t N) { return {ERawControlKind::KeyboardUsage, N}; }
static FInputActionContractDescription Definition()
{
	FInputActionContractDescription D;
	D.Revision = {1}; D.Actions = FInputActionContract::BaseActions();
	FActionDefinition Fire; Fire.Id = 3; Fire.Owner = "Example"; Fire.Name = "Fire"; Fire.Wiring = EActionWiring::Wired;
	D.Actions.push_back(Fire);
	D.Mapping = {{Key(4), Throttle, 1}, {Key(5), Brake, 1}, {Key(6), Steering, -1}, {Key(7), Steering, 1},
		{Key(4), 3, 1}, {Key(5), 3, 1}};
	D.Physical = {{Throttle, EPhysicalDestination::Throttle}, {Brake, EPhysicalDestination::Brake}, {Steering, EPhysicalDestination::Steering}};
	return D;
}
static FRawInputSample Keys(std::uint64_t Sequence, ERawSampleStatus Status, float A, float B, float C, float D)
{
	FRawInputSample S; S.DeviceId = 10; S.Generation = {1}; S.Kind = ERawDeviceKind::Keyboard;
	S.Sequence = Sequence; S.Status = Status; S.FinalState = {{Key(4), A}, {Key(5), B}, {Key(6), C}, {Key(7), D}}; return S;
}
static FInputFrame Expected(const std::shared_ptr<const FInputActionContract>& Contract, FFrameNumber Frame,
	std::uint64_t Source, bool Reset, int T, int B, int S, int Fire, std::uint32_t Active, std::vector<FActionTransition> Edges = {})
{
	FInputFrameData D; D.SourceSequence = Source; D.ConsumptionFrame = Frame;
	D.Producer = {EProducerKind::Device, 9}; D.StreamEpoch = {1}; D.DeviceGeneration = {1}; D.Reset = Reset;
	D.Values[0] = static_cast<std::int16_t>(T); D.Values[1] = static_cast<std::int16_t>(B);
	D.Values[2] = static_cast<std::int16_t>(S); D.Values[3] = static_cast<std::int16_t>(Fire);
	D.ActiveMask = Active; D.Transitions = std::move(Edges); return FInputFrame(Contract, D);
}
static void Equal(const FInputFrame& Actual, const FInputFrame& Wanted)
{
	const auto& A = Actual.GetData(); const auto& B = Wanted.GetData();
	Check(A.Values == B.Values && A.ActiveMask == B.ActiveMask && A.Reset == B.Reset, "oracle values activity reset");
	Check(A.SourceSequence == B.SourceSequence && A.ConsumptionFrame == B.ConsumptionFrame
		&& A.StreamEpoch.Value == B.StreamEpoch.Value && A.DeviceGeneration.Value == B.DeviceGeneration.Value, "oracle clocks");
	Check(A.Transitions.size() == B.Transitions.size(), "oracle transition count");
	for (std::size_t I = 0; I < A.Transitions.size(); ++I)
	{
		const auto& X = A.Transitions[I]; const auto& Y = B.Transitions[I];
		Check(X.Action == Y.Action && X.State == Y.State && X.ValueAtTransition == Y.ValueAtTransition
			&& X.Order.Sequence == Y.Order.Sequence && X.Order.WithinSequence == Y.Order.WithinSequence, "oracle ordered transition payload");
	}
}
int main()
{
	const auto Contract = FInputActionContract::Create(Definition()); Check(bool(Contract), "contract");
	std::vector<FRawInputSample> Raw{Keys(1, ERawSampleStatus::Resync, 1, 0, 0, 0),
		Keys(2, ERawSampleStatus::Valid, 0, 0, 0, 0), Keys(4, ERawSampleStatus::Valid, 0, 0, 0, 0),
		Keys(5, ERawSampleStatus::Valid, 0, 1, 0, 0)};
	Raw[1].Changes = {{{Key(4), 0}, {2, UINT32_MAX}}};
	Raw[2].Changes = {{{Key(6), 1}, {3, 0}}, {{Key(7), 1}, {3, 1}},
		{{Key(6), 0}, {4, UINT32_MAX - 1}}, {{Key(7), 0}, {4, UINT32_MAX}}};
	Raw[3].Changes = {{{Key(5), 1}, {5, 10}}};
	// Authored directly, independently of mapper or response helper.
	std::vector<FInputFrame> Oracle{
		Expected(Contract, 0, 1, true, 255, 0, 0, 1, 9),
		Expected(Contract, 1, 2, false, 0, 0, 0, 0, 0,
			{{0, ETransition::Completed, 0, {2, 0}}, {3, ETransition::Completed, 0, {2, 1}}}),
		Expected(Contract, 2, 4, false, 0, 0, 0, 0, 0,
			{{2, ETransition::Started, -127, {3, 0}}, {2, ETransition::Completed, 0, {3, 1}},
			 {2, ETransition::Started, 127, {4, 0}}, {2, ETransition::Completed, 0, {4, 1}}}),
		Expected(Contract, 3, 5, false, 0, 255, 0, 1, 10,
			{{1, ETransition::Started, 255, {5, 0}}, {3, ETransition::Started, 1, {5, 1}}})};
	auto Test = FTestInputProducer::Create(Contract, {1}, {EProducerKind::Device, 9}, 0, Oracle);
	Check(bool(Test), "sealed post-mapper oracle");
	FActionMapper Mapper(Contract, {1}, {EProducerKind::Device, 9});
	FActionMapper Repeat(Contract, {1}, {EProducerKind::Device, 9});
	for (std::size_t I = 0; I < Raw.size(); ++I)
	{
		auto M = Mapper.Map(Raw[I], I); auto R = Repeat.Map(Raw[I], I); auto T = Test->Produce(I);
		Check(M.Frame && R.Frame && T, "all sources produce exact frame");
		Equal(*M.Frame, Oracle[I]); Equal(*R.Frame, *T);
		const auto PlayerTargets = AssembleDrivingTargets(*M.Frame, *Contract, {1}, I);
		const auto TestTargets = AssembleDrivingTargets(*T, *Contract, {1}, I);
		Check(PlayerTargets.Valid && TestTargets.Valid && PlayerTargets.ThrottleValue == TestTargets.ThrottleValue
			&& PlayerTargets.BrakeValue == TestTargets.BrakeValue && PlayerTargets.SteeringValue == TestTargets.SteeringValue,
			"player and test share exactly the same pure sink");
	}
	Equal(*Test->Produce(0), Oracle[0]); Check(!Test->Produce(4), "sealed source no extrapolation");
	auto InvalidTimeline = Oracle;
	auto InvalidData = InvalidTimeline[1].GetData(); InvalidData.Transitions.clear();
	InvalidTimeline[1] = FInputFrame(Contract, InvalidData);
	Check(!FTestInputProducer::Create(Contract, {1}, {EProducerKind::Device, 9}, 0, InvalidTimeline), "sealed source rejects missing completion");
	InvalidTimeline = Oracle; InvalidData = InvalidTimeline[0].GetData(); InvalidData.Reset = false;
	InvalidTimeline[0] = FInputFrame(Contract, InvalidData);
	Check(!FTestInputProducer::Create(Contract, {1}, {EProducerKind::Device, 9}, 0, InvalidTimeline), "sealed source requires explicit baseline");
	Check(!AssembleDrivingTargets(Oracle[0], *Contract, {2}, 0).Valid, "sink wrong epoch");
	Check(!AssembleDrivingTargets(Oracle[0], *Contract, {1}, 1).Valid, "sink wrong frame");
	auto OtherD = Definition(); OtherD.Revision = {2}; auto Other = FInputActionContract::Create(OtherD);
	Check(!AssembleDrivingTargets(Oracle[0], *Other, {1}, 0).Valid, "sink wrong contract");

	auto ExpectFailure = [&](FRawInputSample Bad, EMappingStatus Wanted)
	{
		FActionMapper M(Contract, {1}, {EProducerKind::Device, 9});
		Check(bool(M.Map(Raw[0], 0).Frame), "failure fixture baseline");
		auto Fail = M.Map(Bad, 1); Check(!Fail.Frame && Fail.Status == Wanted, "explicit failure status with no prefix");
		Check(M.Map(Raw[1], 1).Status == EMappingStatus::ResyncRequired, "failure requires resync");
		auto Recovery = Keys(1, ERawSampleStatus::Resync, 1, 0, 0, 0); Recovery.Generation = {2};
		auto R = M.Map(Recovery, 1);
		Check(R.Frame && R.Frame->GetData().Reset && R.Frame->GetData().Transitions.empty()
			&& R.Frame->GetData().Values[0] == 255 && R.Frame->GetData().ActiveMask == 9, "fresh held recovery atomic");
	};
	ExpectFailure(Keys(2, ERawSampleStatus::Valid, 0, 0, 0, 0), EMappingStatus::UnexplainedState);
	ExpectFailure(Keys(3, ERawSampleStatus::Valid, 1, 0, 0, 0), EMappingStatus::SequenceGap);
	ExpectFailure(Keys(1, ERawSampleStatus::Valid, 1, 0, 0, 0), EMappingStatus::SequenceGap);
	auto Bad = Raw[1]; Bad.DeviceId = 11; ExpectFailure(Bad, EMappingStatus::DeviceChanged);
	Bad = Raw[1]; Bad.Generation = {2}; ExpectFailure(Bad, EMappingStatus::DeviceChanged);
	Bad = Raw[1]; Bad.FinalState.pop_back(); ExpectFailure(Bad, EMappingStatus::DeviceChanged);
	Bad = Raw[1]; Bad.Status = ERawSampleStatus::Overflow; ExpectFailure(Bad, EMappingStatus::Overflow);
	Bad = Raw[1]; Bad.Status = ERawSampleStatus::Unsupported; ExpectFailure(Bad, EMappingStatus::InvalidRaw);
	Bad = Raw[1]; Bad.FinalState[1].Value = std::numeric_limits<float>::infinity(); ExpectFailure(Bad, EMappingStatus::InvalidRaw);
	Bad = Keys(2, ERawSampleStatus::Valid, 0, 0, 0, 0);
	for (std::uint32_t I = 0; I < 33; ++I) Bad.Changes.push_back({{Key(4), I % 2 ? 1.0f : 0.0f}, {2, I}});
	ExpectFailure(Bad, EMappingStatus::Overflow); // Shared key produces 66 edges.
	FActionMapper Missing(Contract, {1}, {EProducerKind::Device, 9});
	Check(Missing.Map(Raw[1], 0).Status == EMappingStatus::ResyncRequired, "first sample requires explicit baseline");
	FActionMapper Order(Contract, {1}, {EProducerKind::Device, 9});
	Check(Order.Map(Raw[0], 1).Status == EMappingStatus::WrongFrame, "wrong physical address");
	Check(bool(Order.Map(Raw[0], 0).Frame), "recover first physical address");
	Check(Order.Map(Raw[0], 1).Status == EMappingStatus::StaleBaseline, "old resync cannot replay source");

	// Explicit response parameters, two axis contributions: sum first, transform once.
	auto AnalogD = Definition(); AnalogD.Actions.resize(3);
	AnalogD.Mapping = {{{ERawControlKind::PadAxis, 0}, Steering, 0.5f}, {{ERawControlKind::PadAxis, 1}, Steering, 0.5f}};
	AnalogD.Actions[2].Deadzone = 0.25f; AnalogD.Actions[2].Exponent = 2; AnalogD.Actions[2].Sensitivity = 0.5f;
	AnalogD.Actions[2].ActivateAbove = 0.20f; AnalogD.Actions[2].DeactivateAtOrBelow = 0.10f;
	auto Analog = FInputActionContract::Create(AnalogD); Check(bool(Analog), "analog contract");
	FRawInputSample A; A.DeviceId = 12; A.Generation = {1}; A.Kind = ERawDeviceKind::Gamepad;
	A.Sequence = 1; A.Status = ERawSampleStatus::Resync;
	A.FinalState = {{{ERawControlKind::PadAxis, 0}, 1}, {{ERawControlKind::PadAxis, 1}, 1}};
	FActionMapper AM(Analog, {1}, {EProducerKind::Device, 9}); auto AR = AM.Map(A, 0);
	Check(AR.Frame && AR.Frame->GetData().Values[2] == 64 && AR.Frame->GetData().ActiveMask == 4, "response once: ((1-.25)/.75)^2*.5 quantizes64");
	A.Sequence = 2; A.Status = ERawSampleStatus::Valid; A.FinalState[0].Value = 0.625f; A.FinalState[1].Value = 0.625f;
	A.Changes = {{A.FinalState[0], {2, 0}}, {A.FinalState[1], {2, 1}}}; AR = AM.Map(A, 1);
	Check(AR.Frame && AR.Frame->GetData().Values[2] == 16 && AR.Frame->GetData().ActiveMask == 4
		&& AR.Frame->GetData().Transitions.empty(), "hysteresis stays active at response .125");
	A.Sequence = 3; A.FinalState[0].Value = 0.25f; A.FinalState[1].Value = 0.25f;
	A.Changes = {{A.FinalState[0], {3, 0}}, {A.FinalState[1], {3, 1}}}; AR = AM.Map(A, 2);
	Check(AR.Frame && AR.Frame->GetData().Values[2] == 0 && AR.Frame->GetData().ActiveMask == 0
		&& AR.Frame->GetData().Transitions.size() == 1 && AR.Frame->GetData().Transitions[0].ValueAtTransition == 4,
		"hysteresis completes on intermediate response before final neutral");

	// Bool OR: release one of two held keys without completing the shared action.
	FActionMapper Or(Contract, {1}, {EProducerKind::Device, 9});
	Check(bool(Or.Map(Keys(1, ERawSampleStatus::Resync, 1, 1, 0, 0), 0).Frame), "OR baseline");
	auto One = Keys(2, ERawSampleStatus::Valid, 0, 1, 0, 0); One.Changes = {{{Key(4), 0}, {2, 0}}};
	const auto OrFrame = Or.Map(One, 1);
	Check(OrFrame.Frame && OrFrame.Frame->GetData().Values[3] == 1 && OrFrame.Frame->GetData().Transitions.size() == 1,
		"OR preserves other held contribution");
	auto TinyD = AnalogD; TinyD.Actions = FInputActionContract::BaseActions();
	TinyD.Mapping = {{{ERawControlKind::PadAxis, 0}, Steering, 1}};
	const auto TinyContract = FInputActionContract::Create(TinyD);
	Check(bool(TinyContract), "tiny axis contract");
	FActionMapper Tiny(TinyContract, {1}, {EProducerKind::Device, 9});
	A.Sequence = 1; A.Status = ERawSampleStatus::Resync; A.Changes.clear(); A.FinalState[0].Value = 0.001f;
	auto TinyFrame = Tiny.Map(A, 0);
	Check(TinyFrame.Frame && TinyFrame.Frame->GetData().Values[2] == 0 && TinyFrame.Frame->GetData().ActiveMask == 4,
		"activation precedes quantization to zero");
	{
		Speed::Input::FPresentationInputScope Scope;
		Check(!Test->Produce(0) && Or.Map(One, 2).Status == EMappingStatus::PresentationForbidden, "presentation cannot advance source");
	}
	std::cout << "PASS ActionMappingProbe checks=" << Checks << '\n';
}
