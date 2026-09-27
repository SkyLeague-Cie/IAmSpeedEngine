#include "IAmSpeed/Input/Testing/WheeledTestProfile.h"
#include "IAmSpeed/Input/InputStream.h"
#include "IAmSpeed/Input/DrivingInputTargets.h"
#include <cstdlib>
#include <iostream>
#include <limits>

using namespace Speed::Input;
using namespace Speed::Input::Testing;
static unsigned Checks = 0;
static void Check(bool Value, const char* Message)
{
	++Checks;
	if (!Value) { std::cerr << "FAIL " << Message << '\n'; std::exit(1); }
}

// Instrumentation delegates to the actual sealed producer. It never supplies
// values itself and is not a substitute for a physical consumer.
class FCountedProducer final : public IInputProducer
{
public:
	explicit FCountedProducer(std::unique_ptr<FTestInputProducer> In) : Source(std::move(In)) {}
	std::optional<FInputFrame> Produce(FFrameNumber Frame) override
	{ ++Calls; return Source->Produce(Frame); }
	unsigned Calls = 0;
private:
	std::unique_ptr<FTestInputProducer> Source;
};

static FWheeledTestProfile Base(FFrameNumber Count = 8)
{
	FWheeledTestProfile P;
	P.FrameCount = Count;
	P.Identity = {EProducerKind::Device, 71};
	return P;
}

static void Reject(const FWheeledTestProfile& P, EWheeledProfileError Error)
{
	auto R = CompileWheeledTestProfile(P);
	Check(!R.Producer && R.Error == Error, "reject with explicit error and no partial producer");
}

static std::vector<std::array<int, 3>> RunTimeline(unsigned ObservationStride)
{
	auto P = Base();
	// Intentionally unsorted. Expected values below are independent literals.
	P.Keys = {{4, -2.f, 2.f, .5f}, {0, 2.f, -2.f, -2.f},
		{6, .5f / 255.f, 0, 0}, {2, .5f, .5f, -.5f}};
	auto R = CompileWheeledTestProfile(P);
	Check(R.Error == EWheeledProfileError::None && bool(R.Producer), "compile valid profile");
	Check(P.Keys[0].ScenarioFrame == 4, "compilation leaves caller order unchanged");
	// Mutation after compilation cannot change the sealed source.
	P.Keys.clear(); P.Identity.Id = 999;
	auto Counted = std::make_shared<FCountedProducer>(std::move(R.Producer));
	FInputStream Stream(Counted);
	const std::array<std::array<int, 3>, 8> Expected = {{
		{{0, 0, 0}}, {{255, 0, -127}}, {{255, 0, -127}}, {{128, 128, -63}},
		{{128, 128, -63}}, {{0, 255, 64}}, {{0, 255, 64}}, {{1, 0, 0}}
	}};
	std::vector<std::array<int, 3>> Observed;
	Check(!Stream.Consume(1), "cannot skip initial reset C0");
	Check(!Stream.PublishCompleted(0), "unconsumed frame cannot publish");
	const auto Before = Counted->Calls;
	for (FFrameNumber C = 0; C < Expected.size(); ++C)
	{
		const auto Frame = Stream.Consume(C);
		Check(Frame.has_value(), "consume exact compiled frame");
		Check(Frame->GetSourceFrame() == C && Frame->GetConsumptionFrame() == C,
			"source and consumption clocks are explicit canonical frames");
		Check(Frame->GetProducer().Kind == EProducerKind::Device && Frame->GetProducer().Id == 71,
			"producer identity is immutable");
		Check(Frame->RequiresReset() == (C == 0) && Frame->GetEdgeCount() == 0,
			"neutral reset only at C0; no manufactured edges");
		const auto Target = ReadDrivingInputTargets(Frame, C);
		Check(Target.Valid, "real shared driving target reader accepts exact frame");
		Observed.push_back({Target.ThrottleValue, Target.BrakeValue, Target.SteeringValue});
		Check(Observed.back() == Expected[C], "independent timing/hold/quantization oracle");
		Check(!ReadDrivingInputTargets(Frame, C + 1).Valid, "reader rejects wrong frame");
		const auto Calls = Counted->Calls;
		const auto Replay = Stream.Consume(C);
		Check(Replay && Replay->GetActions() == Frame->GetActions() && Counted->Calls == Calls,
			"same-frame replay uses history without producing twice");
		for (std::size_t A = 3; A < ActionCount; ++A)
			Check(Frame->GetActions()[A] == 0, "no game-owned action sneaks into wheeled profile");
		// Publication is a stream test only, not simulated successful UE physics.
		Check(Stream.PublishCompleted(C), "publish recorded stream frame");
		if (C % ObservationStride == 0)
		{
			const auto Latest = Stream.ReadLatest();
			Check(Latest && Latest->Frame.GetConsumptionFrame() == C, "independent observation cadence");
		}
	}
	Check(Counted->Calls - Before == Expected.size(), "exactly one successful production per frame");
	Check(!Stream.Consume(8), "finite horizon has no held fallback");
	Check(!Stream.Skip(8), "test producer cannot silently skip inputs");
	return Observed;
}

int main()
{
	const auto First = RunTimeline(1);
	Check(First == RunTimeline(3), "identical history with sparse presentation");
	Check(First == RunTimeline(7), "repeatability with another observation cadence");
	using E = EWheeledProfileError;
	auto P = Base(0); Reject(P, E::InvalidHorizon);
	P = Base(FWheeledTestProfile::MaxFrameCount + 1); Reject(P, E::InvalidHorizon);
	P = Base(std::numeric_limits<FFrameNumber>::max()); Reject(P, E::InvalidHorizon);
	P = Base(); P.Identity.Id = 0; Reject(P, E::InvalidIdentity);
	P = Base(); P.Identity.Kind = static_cast<EProducerKind>(3); Reject(P, E::InvalidIdentity);
	for (float FWheeledTestProfile::* Axis : {&FWheeledTestProfile::InitialThrottle,
		&FWheeledTestProfile::InitialBrake, &FWheeledTestProfile::InitialSteering})
	{
		for (float Value : {1.f, -1.f, std::numeric_limits<float>::quiet_NaN(),
			std::numeric_limits<float>::infinity()})
		{ P = Base(); P.*Axis = Value; Reject(P, E::InitialInputNotNeutral); }
	}
	for (bool FWheeledTestProfile::* Capability : {&FWheeledTestProfile::BypassWheeledSlew,
		&FWheeledTestProfile::BypassSkySlew, &FWheeledTestProfile::ControlsCanMove,
		&FWheeledTestProfile::HasSkyInput, &FWheeledTestProfile::HasDiscreteActions,
		&FWheeledTestProfile::HasReactiveTriggers, &FWheeledTestProfile::HasCameraInput})
	{ P = Base(); P.*Capability = true; Reject(P, E::UnsupportedCapability); }
	P = Base(); P.Keys = {{0, 0, 0, 0}, {0, 1, 1, 1}}; Reject(P, E::DuplicateKeyFrame);
	P = Base(); P.Keys = {{7, 0, 0, 0}}; Reject(P, E::InvalidKeyFrame);
	P.Keys[0].ScenarioFrame = std::numeric_limits<FFrameNumber>::max(); Reject(P, E::InvalidKeyFrame);
	P = Base(1); P.Keys = {{0, 0, 0, 0}}; Reject(P, E::InvalidKeyFrame);
	for (float FWheeledScenarioKey::* Axis : {&FWheeledScenarioKey::ThrottleValue,
		&FWheeledScenarioKey::BrakeValue, &FWheeledScenarioKey::SteeringValue})
	{
		for (float Value : {std::numeric_limits<float>::quiet_NaN(),
			std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()})
		{ P = Base(); P.Keys = {{0, 0, 0, 0}}; P.Keys[0].*Axis = Value; Reject(P, E::NonFiniteValue); }
	}
	// Independent signed-half and saturation table. No call to QuantizeAxis
	// computes the expected integers; the compiler alone uses that function.
	struct FCase { float T, B, S; int QT, QB, QS; };
	const FCase Cases[] = {{0, 0, 0, 0, 0, 0}, {1, 1, 1, 255, 255, 127},
		{-1, 2, -1, 0, 255, -127}, {.5f, .5f, -.5f, 128, 128, -63},
		{.5f / 255.f, 1.5f / 255.f, -.5f / 127.f, 1, 2, 0},
		{0, 0, -1.5f / 127.f, 0, 0, -1}, {0, 0, .5f / 127.f, 0, 0, 1}};
	for (const auto& C : Cases)
	{
		P = Base(2); P.Keys = {{0, C.T, C.B, C.S}};
		auto R = CompileWheeledTestProfile(P);
		Check(bool(R.Producer), "quantization case compiles");
		FInputStream Stream(std::shared_ptr<IInputProducer>(std::move(R.Producer)));
		Check(Stream.Consume(0).has_value(), "consume neutral baseline before key");
		const auto T = ReadDrivingInputTargets(Stream.Consume(1), 1);
		Check(T.Valid && T.ThrottleValue == C.QT && T.BrakeValue == C.QB && T.SteeringValue == C.QS,
			"independent clamp and ties toward positive infinity");
	}
	P = Base(1);
	auto Neutral = CompileWheeledTestProfile(P);
	Check(Neutral.Producer && Neutral.Producer->Produce(0)->RequiresReset()
		&& !Neutral.Producer->Produce(1), "one-frame neutral-only horizon");
	P = Base(300); P.Keys = {{0, 1, 0, 0}};
	auto Long = CompileWheeledTestProfile(P);
	Check(bool(Long.Producer), "profile longer than history capacity");
	for (FFrameNumber C = 0; C < 300; ++C)
		Check(Long.Producer->Produce(C).has_value(), "long profile sequential consumption");
	Check(!Long.Producer->Produce(0), "expired history replay rejects");
	Check(Long.Producer->Produce(299).has_value(), "recent replay remains available");
	{
		FPresentationInputScope Presentation;
		Reject(Base(), E::ProducerRejected);
	}
	std::cout << "PASS WheeledTestProfileProbe checks=" << Checks
		<< " scope=SOURCE_NATIVE_PORTABLE_ONLY physical_consumer=not_run raw_mapping=not_run\n";
}
