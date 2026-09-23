#pragma once

#include "TestInputProducer.h"
#include <algorithm>

namespace Speed::Input::Testing
{
// Scenario N is observed AFTER physics: its key first affects canonical N+1.
// These are normalized action targets, not OS samples or physical state.
struct FWheeledScenarioKey
{
	FFrameNumber ScenarioFrame = 0;
	float ThrottleValue = 0, BrakeValue = 0, SteeringValue = 0;
};

struct FWheeledTestProfile
{
	// Includes neutral reset C0 and the last observed frame. Bound memory before
	// allocation; long/adaptive scenarios require a separately reviewed producer.
	static constexpr FFrameNumber MaxFrameCount = 16384;
	FFrameNumber FrameCount = 0;
	FProducerIdentity Identity;
	float InitialThrottle = 0, InitialBrake = 0, InitialSteering = 0;
	std::vector<FWheeledScenarioKey> Keys;
	// The caller must describe the WHOLE stimulus, not discard unsupported keys.
	bool BypassWheeledSlew = false;
	bool BypassSkySlew = false;
	bool ControlsCanMove = false;
	bool HasSkyInput = false;
	bool HasDiscreteActions = false;
	bool HasReactiveTriggers = false;
	bool HasCameraInput = false;
};

enum class EWheeledProfileError
{
	None, InvalidHorizon, InvalidIdentity, InitialInputNotNeutral,
	UnsupportedCapability, InvalidKeyFrame, DuplicateKeyFrame,
	NonFiniteValue, ProducerRejected
};

struct FCompiledWheeledProfile
{
	std::unique_ptr<FTestInputProducer> Producer;
	EWheeledProfileError Error = EWheeledProfileError::None;
};

// Portable preparation only. Does not attach a stream, run physics, apply slew,
// inject legacy queues, map raw devices or activate a controller.
inline FCompiledWheeledProfile CompileWheeledTestProfile(const FWheeledTestProfile& Profile)
{
	using E = EWheeledProfileError;
	if (!Profile.FrameCount || Profile.FrameCount > FWheeledTestProfile::MaxFrameCount)
		return {nullptr, E::InvalidHorizon};
	if (!Profile.Identity.Id || Profile.Identity.Kind > EProducerKind::Network)
		return {nullptr, E::InvalidIdentity};
	// Initial live setters have pre-simulation timing. Even a clamped-to-zero
	// nonzero value is NOT reinterpreted as a post-physics key here. NaN rejects.
	if (Profile.InitialThrottle != 0 || Profile.InitialBrake != 0 || Profile.InitialSteering != 0)
		return {nullptr, E::InitialInputNotNeutral};
	if (Profile.BypassWheeledSlew || Profile.BypassSkySlew || Profile.ControlsCanMove
		|| Profile.HasSkyInput || Profile.HasDiscreteActions || Profile.HasReactiveTriggers
		|| Profile.HasCameraInput)
		return {nullptr, E::UnsupportedCapability};
	if (Profile.Keys.size() > Profile.FrameCount - 1)
		return {nullptr, E::InvalidKeyFrame};
	auto Keys = Profile.Keys;
	std::sort(Keys.begin(), Keys.end(), [](const auto& A, const auto& B)
	{ return A.ScenarioFrame < B.ScenarioFrame; });
	std::vector<FActionValues> Values;
	Values.reserve(Keys.size());
	for (std::size_t I = 0; I < Keys.size(); ++I)
	{
		const auto& Key = Keys[I];
		if (Key.ScenarioFrame >= Profile.FrameCount - 1)
			return {nullptr, E::InvalidKeyFrame};
		if (I && Keys[I - 1].ScenarioFrame == Key.ScenarioFrame)
			return {nullptr, E::DuplicateKeyFrame};
		// Exactly one conversion per key/axis: finite, clamp then floor(x*s+.5).
		const auto T = QuantizeAxis(Key.ThrottleValue, false);
		const auto B = QuantizeAxis(Key.BrakeValue, false);
		const auto S = QuantizeAxis(Key.SteeringValue, true);
		if (!T || !B || !S) return {nullptr, E::NonFiniteValue};
		FActionValues V{};
		V[Throttle] = *T; V[Brake] = *B; V[Steering] = *S;
		Values.push_back(V);
	}
	std::vector<FInputFrame> Frames;
	Frames.reserve(static_cast<std::size_t>(Profile.FrameCount));
	FActionValues Held{};
	std::size_t Next = 0;
	for (FFrameNumber C = 0; C < Profile.FrameCount; ++C)
	{
		if (Next < Keys.size() && C > 0 && Keys[Next].ScenarioFrame == C - 1)
			Held = Values[Next++];
		// One synthetic sample per physical frame, including holds. No edges.
		Frames.emplace_back(C, C, Profile.Identity, Held,
			std::array<FActionEdge, MaxEdges>{}, 0, C == 0);
	}
	auto Producer = FTestInputProducer::Create(Profile.Identity, 0, Frames);
	if (!Producer) return {nullptr, E::ProducerRejected};
	return {std::move(Producer), E::None};
}
}
