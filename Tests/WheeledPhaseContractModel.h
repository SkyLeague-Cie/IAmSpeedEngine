#pragma once

// SPECIFICATION MODEL ONLY. Never included by Source/ or used as a producer.
// Describes a proposed closed timeline; it does not execute Unreal physics.
#include "IAmSpeed/Input/InputFrame.h"
#include <algorithm>
#include <limits>
#include <vector>

namespace WheeledPhaseContract
{
using Frame = Speed::Input::FFrameNumber;
struct Axes
{
	int T = 0, B = 0, S = 0;
	bool operator==(const Axes& R) const { return T == R.T && B == R.B && S == R.S; }
};
struct Payload { Axes Values; bool CanMove = false; };
struct Key
{
	Frame Scenario = 0;
	float T = 0, B = 0, S = 0;
	bool CanMove = false, Bypass = false;
};
enum class SeedPolicy { BeforeFirstFrame, NeutralDuringWarmup };
struct Description
{
	// Explicit externally-qualified local frame of the first PostPhysics sample.
	// A fixture value is not proof of the real scenario's runtime anchor.
	Frame FirstLocal = 0, FrameCount = 0, Warmup = 0;
	SeedPolicy Policy = SeedPolicy::BeforeFirstFrame;
	float InitialT = 0, InitialB = 0, InitialS = 0;
	bool OverridesMobilityCountdown = false, HasUnmodelledActions = false;
	std::vector<Key> Keys;
};
struct Command { Frame Canonical; Payload Input; bool Bypass; };
struct Plan
{
	Frame FirstCanonical = 0, FrameCount = 0;
	Axes Seed;
	std::vector<Command> Commands;
};
enum class Error { None, Clock, Policy, Unsupported, Value, Duplicate };
struct Compilation { std::optional<Plan> Value; Error Failure; };

inline Compilation Compile(const Description& D)
{
	if (!D.FirstLocal || !D.FrameCount || D.FrameCount > 16384
		|| D.FirstLocal > Frame(std::numeric_limits<std::int32_t>::max()) - D.FrameCount)
		return {{}, Error::Clock};
	if ((D.Warmup > 0) != (D.Policy == SeedPolicy::NeutralDuringWarmup)
		|| (D.Policy != SeedPolicy::BeforeFirstFrame && D.Policy != SeedPolicy::NeutralDuringWarmup))
		return {{}, Error::Policy};
	if (D.OverridesMobilityCountdown || D.HasUnmodelledActions) return {{}, Error::Unsupported};
	auto Quantize = [](float T, float B, float S) -> std::optional<Axes>
	{
		const auto QT = Speed::Input::QuantizeAxis(T, false);
		const auto QB = Speed::Input::QuantizeAxis(B, false);
		const auto QS = Speed::Input::QuantizeAxis(S, true);
		if (!QT || !QB || !QS) return {};
		return Axes{*QT, *QB, *QS};
	};
	const auto Initial = Quantize(D.InitialT, D.InitialB, D.InitialS);
	if (!Initial) return {{}, Error::Value};
	Plan P{D.FirstLocal - 1, D.FrameCount, D.Warmup ? Axes{} : *Initial, {}};
	if (D.Warmup) P.Commands.push_back({D.FirstLocal, {{}, true}, true});
	for (const auto& K : D.Keys)
	{
		// local activation = FirstLocal + scenario + warmup + 1;
		// canonical activation = local - 1, with no sentinel/overflow alias.
		if (K.Scenario >= D.FrameCount || D.Warmup >= D.FrameCount
			|| K.Scenario >= D.FrameCount - D.Warmup - 1)
			return {{}, Error::Clock};
		const auto Values = Quantize(K.T, K.B, K.S);
		if (!Values) return {{}, Error::Value};
		P.Commands.push_back({D.FirstLocal + K.Scenario + D.Warmup, {*Values, K.CanMove}, K.Bypass});
	}
	if (D.Warmup && (D.Warmup >= D.FrameCount || D.FrameCount < 2)) return {{}, Error::Clock};
	std::sort(P.Commands.begin(), P.Commands.end(), [](const auto& A, const auto& B)
	{ return A.Canonical < B.Canonical; });
	for (std::size_t I = 1; I < P.Commands.size(); ++I)
		if (P.Commands[I - 1].Canonical == P.Commands[I].Canonical) return {{}, Error::Duplicate};
	return {std::move(P), Error::None};
}

struct State
{
	Payload User, Physical, BeforeSlew;
	Axes SyncedAxes;
	std::optional<Frame> LastSlew;
	bool EngineAllowsMovement = false; // Distinct from payload CanMove.
};

class Model
{
public:
	bool Acquire(std::uint64_t Owner, std::uint64_t Generation, const Description& D)
	{
		if (Active || !Owner || !Generation || Generation <= LastGeneration) return false;
		auto Compiled = Compile(D);
		if (!Compiled.Value) return false;
		// Reset before staging the pre-simulation seed. No old owner's held state.
		Current = {}; Timeline = std::move(*Compiled.Value);
		OwnerId = Owner; LastGeneration = Generation; Active = true;
		Next = Timeline.FirstCanonical; Applied.reset(); Recorded.reset(); Observed.reset(); Held.reset(); KeyIndex = 0;
		return true;
	}
	bool Apply(std::uint64_t Owner, std::uint64_t Generation, Frame C)
	{
		if (!Owns(Owner, Generation)) return false;
		if (Applied && C == *Applied) return true; // Immutable compiled command replay.
		if (C != Next || C - Timeline.FirstCanonical >= Timeline.FrameCount
			|| (Applied && Recorded != Applied)) return false;
		if (C == Timeline.FirstCanonical) Current.User.Values = Timeline.Seed;
		if (KeyIndex < Timeline.Commands.size() && Timeline.Commands[KeyIndex].Canonical == C)
			Held = Timeline.Commands[KeyIndex++];
		// The harness requeues ActiveWheeledInput every frame after the first key.
		// A held bypass therefore repeats its complete immediate effect, even
		// when no new profile key occurs on this frame.
		if (Held)
		{
			const auto& Cmd = *Held;
			Current.User = Cmd.Input;
			if (Cmd.Bypass)
			{
				Current.Physical = Current.User;
				Current.BeforeSlew = Current.Physical;
				Current.LastSlew = C;
				Current.SyncedAxes = Current.Physical.Values;
			}
		}
		if (!Current.LastSlew || *Current.LastSlew != C)
		{
			auto Toward = [](int From, int To) { return From + std::clamp(To - From, -16, 16); };
			Current.Physical.Values = {Toward(Current.Physical.Values.T, Current.User.Values.T),
				Toward(Current.Physical.Values.B, Current.User.Values.B),
				Toward(Current.Physical.Values.S, Current.User.Values.S)};
			Current.LastSlew = C;
		}
		Applied = C;
		return true;
	}
	bool ObserveGameplayMobility(std::uint64_t Owner, std::uint64_t Generation, Frame C, bool Allowed)
	{
		if (!Owns(Owner, Generation) || Applied != C || Recorded == C) return false;
		if (Observed == C) return Current.EngineAllowsMovement == Allowed;
		// External engine observation, never an input command overriding countdown.
		Current.EngineAllowsMovement = Allowed;
		Current.User.CanMove = Allowed;
		Observed = C;
		return true;
	}
	bool Record(std::uint64_t Owner, std::uint64_t Generation, Frame C)
	{
		if (!Owns(Owner, Generation) || Applied != C) return false;
		if (Recorded == C) return true;
		Current.SyncedAxes = Current.Physical.Values;
		Recorded = C; Next = C + 1;
		return true;
	}
	bool Release(std::uint64_t Owner, std::uint64_t Generation)
	{
		if (!Owns(Owner, Generation)) return false;
		Active = false; Current = {}; Timeline = {}; Applied.reset(); Recorded.reset(); Observed.reset(); Held.reset();
		return true;
	}
	const State& Read() const { return Current; }
private:
	bool Owns(std::uint64_t Owner, std::uint64_t Generation) const
	{ return Active && Owner == OwnerId && Generation == LastGeneration; }
	State Current;
	Plan Timeline;
	Frame Next = 0;
	std::optional<Frame> Applied, Recorded, Observed;
	std::optional<Command> Held;
	std::size_t KeyIndex = 0;
	std::uint64_t OwnerId = 0, LastGeneration = 0;
	bool Active = false;
};
}
