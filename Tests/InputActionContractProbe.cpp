#include "IAmSpeed/Input/InputFrameV2.h"
#include "IAmSpeed/Input/Testing/TestInputProducer.h"
#include <cstdlib>
#include <iostream>
#include <type_traits>

namespace V2 = Speed::Input::V2;
using namespace Speed::Input;
static unsigned Checks = 0;
static void Check(bool Value, const char* Name)
{ ++Checks; if (!Value) { std::cerr << "FAIL " << Name << '\n'; std::exit(1); } }
static V2::FInputActionContractDescription Description()
{
	V2::FInputActionContractDescription D;
	D.Revision = {1}; D.Actions = V2::FInputActionContract::BaseActions();
	D.Physical = {{Throttle, V2::EPhysicalDestination::Throttle}, {Brake, V2::EPhysicalDestination::Brake},
		{Steering, V2::EPhysicalDestination::Steering}};
	D.Mapping = {{{V2::ERawControlKind::PadAxis, 4}, Throttle, 1},
		{{V2::ERawControlKind::PadAxis, 5}, Brake, 1}, {{V2::ERawControlKind::PadAxis, 0}, Steering, 1}};
	return D;
}
int main()
{
	static_assert(!std::is_convertible<V2::FInputFrame, FInputFrame>::value, "No silent v2 downgrade");
	static_assert(!std::is_convertible<FInputFrame, V2::FInputFrame>::value, "No invented active mask");
	static_assert(!std::is_same<V2::FStreamEpoch, V2::FDeviceGeneration>::value, "Separate clocks");
	auto D = Description(); const auto Contract = V2::FInputActionContract::Create(D);
	Check(bool(Contract), "valid base contract");
	const auto Copy = V2::FInputActionContract::Create(D);
	Check(Copy && Contract->GetFingerprint() == Copy->GetFingerprint(), "content identity independent of allocation");
	D.Mapping[0].Scale = -1;
	const auto Inverted = V2::FInputActionContract::Create(D);
	Check(Inverted && Inverted->GetFingerprint() != Contract->GetFingerprint(), "mapping included in identity");
	D = Description(); D.Revision.Value = 2;
	const auto Revision = V2::FInputActionContract::Create(D);
	Check(Revision && Revision->GetFingerprint() != Contract->GetFingerprint(), "revision included");
	D = Description(); D.Physical[1].Destination = V2::EPhysicalDestination::Throttle;
	Check(!V2::FInputActionContract::Create(D), "duplicate sink destination");
	D = Description(); D.Physical.pop_back(); Check(!V2::FInputActionContract::Create(D), "incomplete sink");
	D = Description(); D.Actions[2].Quantization = 255;
	Check(!V2::FInputActionContract::Create(D), "base steering scale fixed");
	D = Description(); D.Actions[0].Aliases = {"Brake"};
	Check(!V2::FInputActionContract::Create(D), "alias collision");
	D = Description(); D.Actions[0].ActivateAbove = std::numeric_limits<float>::quiet_NaN();
	Check(!V2::FInputActionContract::Create(D), "nonfinite response");
	D = Description(); D.Actions[0].DeactivateAtOrBelow = 0.5f;
	Check(!V2::FInputActionContract::Create(D), "inverted hysteresis");
	D = Description(); D.Actions[0].Type = static_cast<V2::EActionType>(2);
	Check(!V2::FInputActionContract::Create(D), "unsupported action type");
	D = Description(); V2::FActionDefinition Extension;
	Extension.Id = 3; Extension.Owner = "Example"; Extension.Name = "Fire";
	D.Actions.push_back(Extension);
	const auto Unwired = V2::FInputActionContract::Create(D);
	Check(bool(Unwired), "game owns declarative extension");
	D.Mapping.push_back({{V2::ERawControlKind::PadButton, 0}, 3, 1});
	Check(!V2::FInputActionContract::Create(D), "unwired cannot acquire mapping");
	D.Actions.back().Wiring = V2::EActionWiring::Wired;
	const auto Wired = V2::FInputActionContract::Create(D);
	Check(bool(Wired), "wired bool extension");

	V2::FInputFrameData F;
	F.Producer = {EProducerKind::Device, 8}; F.StreamEpoch = {1}; F.DeviceGeneration = {1}; F.SourceSequence = 9;
	F.Reset = true; F.ActiveMask = 1; F.Values[Throttle] = 0;
	Check(V2::FInputFrame(Contract, F).IsValidFor(*Contract), "active independent from rounded zero");
	F.Values[Throttle] = 255; F.ActiveMask = 0;
	Check(V2::FInputFrame(Contract, F).IsValidFor(*Contract), "validation does not invent activity from values");
	Check(!V2::FInputFrame(Contract, F).IsValidFor(*Revision), "wrong contract rejected");
	Check(!V2::FInputFrame({}, F).IsValidFor(*Contract), "missing contract rejected");
	F.Values[Throttle] = 256; Check(!V2::FInputFrame(Contract, F).IsValidFor(*Contract), "range checked");
	F.Values[Throttle] = 0;
	F.Transitions = {{Throttle, V2::ETransition::Started, 255, {8, 0}},
		{Throttle, V2::ETransition::Completed, 0, {8, 1}}};
	Check(!V2::FInputFrame(Contract, F).IsStructurallyValid(), "reset cannot include transitions");
	F.Reset = false; Check(V2::FInputFrame(Contract, F).IsValidFor(*Contract), "press release within same sample");
	F.Transitions[1].Order.WithinSequence = 0;
	Check(!V2::FInputFrame(Contract, F).IsStructurallyValid(), "duplicate source order rejected");
	F.Transitions[1].Order = {10, 0};
	Check(!V2::FInputFrame(Contract, F).IsStructurallyValid(), "future edge rejected");
	F.Transitions[1].Order = {8, 1}; F.ActiveMask = 1;
	Check(!V2::FInputFrame(Contract, F).IsValidFor(*Contract), "final activity matches last transition");
	F.ActiveMask = 0; F.Transitions[1].State = V2::ETransition::Started;
	Check(!V2::FInputFrame(Contract, F).IsValidFor(*Contract), "duplicate started rejected");
	F.Transitions.clear(); F.ActiveMask = 8;
	Check(!V2::FInputFrame(Unwired, F).IsValidFor(*Unwired), "unwired activity rejected");
	F.ActiveMask = 0; F.Version = 3;
	Check(!V2::FInputFrame(Contract, F).IsStructurallyValid(), "unknown frame version rejected");
	F.Version = 2; F.Transitions.resize(MaxEdges + 1);
	Check(!V2::FInputFrame(Contract, F).IsStructurallyValid(), "overflow rejected whole");

	V2::FRawInputSample S;
	S.DeviceId = 7; S.Generation = {1}; S.Sequence = 2; S.Kind = V2::ERawDeviceKind::Gamepad;
	S.Status = V2::ERawSampleStatus::Valid;
	S.FinalState = {{{V2::ERawControlKind::PadButton, 0}, 0}, {{V2::ERawControlKind::PadAxis, 2}, -1}};
	S.Changes = {{{{V2::ERawControlKind::PadButton, 0}, 1}, {1, 0}},
		{{{V2::ERawControlKind::PadButton, 0}, 0}, {2, 0}}};
	Check(S.IsValid(), "generic right stick and button cycle");
	S.FinalState[0].Value = 1; Check(!S.IsValid(), "journal final state mismatch"); S.FinalState[0].Value = 0;
	S.Status = V2::ERawSampleStatus::Resync; Check(!S.IsValid(), "resync discards edges");
	S.Changes.clear(); Check(S.IsValid(), "held resync baseline");
	S.Status = V2::ERawSampleStatus::Overflow; Check(!S.IsValid(), "raw overflow fail closed");
	S.Status = V2::ERawSampleStatus::Valid; S.FinalState[1].Control.Code = 4;
	Check(!S.IsValid(), "trigger cannot be signed");
	S.FinalState = {{{V2::ERawControlKind::KeyboardUsage, 0x42}, 1}}; S.Kind = V2::ERawDeviceKind::Keyboard;
	Check(S.IsValid(), "explicit HID F9 usage"); S.FinalState[0].Control.Code = 0;
	Check(!S.IsValid(), "unknown physical usage");

	// Existing v1 producer remains source-compatible and independently usable.
	FActionValues LegacyValues{}; LegacyValues[Throttle] = 255;
	const FInputFrame Legacy(1, 0, {EProducerKind::Device, 1}, LegacyValues, {}, 0, true);
	auto Producer = FTestInputProducer::Create({EProducerKind::Device, 1}, 0, {Legacy});
	Check(Producer && Producer->Produce(0)->GetActions()[Throttle] == 255, "legacy sealed fixture retained");
	std::cout << "PASS " << Checks << " checks\n";
}
