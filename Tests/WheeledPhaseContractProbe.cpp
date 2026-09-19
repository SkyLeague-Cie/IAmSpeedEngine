#include "WheeledPhaseContractModel.h"
#include "IAmSpeed/Input/Testing/WheeledTestProfile.h"
#include <cstdlib>
#include <iostream>

using namespace WheeledPhaseContract;
static unsigned Checks = 0;
static void Check(bool Value, const char* What)
{
	++Checks;
	if (!Value) { std::cerr << "FAIL " << What << '\n'; std::exit(1); }
}
#define CHECK(...) Check((__VA_ARGS__), #__VA_ARGS__)

int main()
{
	// Case 29 source fixture. Local anchor 1 is EXPLICITLY synthetic, not a
	// measurement of ScenarioStartFrame in a running Unreal scenario.
	Description Case29;
	Case29.FirstLocal = 1; Case29.FrameCount = 70; Case29.Warmup = 5;
	Case29.Policy = SeedPolicy::NeutralDuringWarmup; Case29.InitialT = .25f;
	Case29.Keys = {{59, .25f, 0.f, 0.f, true, true}};
	auto P = Compile(Case29);
	CHECK(P.Value && P.Failure == Error::None);
	CHECK(P.Value->Seed == Axes{});
	CHECK(P.Value->Commands.size() == 2);
	CHECK(P.Value->Commands[0].Canonical == 1 && P.Value->Commands[0].Input.Values == Axes{});
	CHECK(P.Value->Commands[0].Input.CanMove && P.Value->Commands[0].Bypass);
	CHECK(P.Value->Commands[1].Canonical == 65);
	CHECK(P.Value->Commands[1].Input.Values == Axes{64, 0, 0});

	Model M;
	CHECK(!M.Acquire(0, 1, Case29));
	CHECK(M.Acquire(7, 1, Case29));
	CHECK(!M.Acquire(8, 2, Case29));
	Case29.Keys[0].T = 1.f; // Model owns a compiled copy.
	CHECK(!M.Apply(8, 1, 0));
	CHECK(!M.Apply(7, 2, 0));
	CHECK(!M.Apply(7, 1, 1));
	for (Frame C = 0; C < 70; ++C)
	{
		CHECK(M.Apply(7, 1, C));
		const Axes Expected = C < 65 ? Axes{} : Axes{64, 0, 0};
		CHECK(M.Read().User.Values == Expected);
		CHECK(M.Read().Physical.Values == Expected);
		CHECK(M.Read().LastSlew == C);
		if (C > 0)
		{
			CHECK(M.Read().BeforeSlew.Values == Expected);
			CHECK(M.Read().SyncedAxes == Expected); // Bypass synchronizes immediately.
			CHECK(M.Read().User.CanMove && M.Read().Physical.CanMove);
			CHECK(!M.Read().EngineAllowsMovement); // Payload never unlocks countdown.
		}
		CHECK(!M.Apply(7, 1, C + 1)); // Must record current frame first.
		CHECK(M.Apply(7, 1, C));
		CHECK(M.Read().Physical.Values == Expected);
		CHECK(M.ObserveGameplayMobility(7, 1, C, false));
		CHECK(!M.ObserveGameplayMobility(7, 1, C, true)); // Conflicting same-phase observation.
		CHECK(!M.Read().User.CanMove);
		CHECK(M.Record(7, 1, C));
		CHECK(M.Record(7, 1, C));
		CHECK(M.Read().SyncedAxes == Expected);
	}
	CHECK(!M.Apply(7, 1, 70));
	CHECK(!M.Release(8, 1));
	CHECK(M.Release(7, 1));
	CHECK(M.Read().Physical.Values == Axes{} && !M.Read().LastSlew);
	CHECK(!M.Apply(7, 1, 69));
	CHECK(!M.Acquire(7, 1, Case29)); // Released generation cannot resurrect.

	// Separate synthetic seed contract: not the warmup-suppressed case 29.
	Description Seed;
	Seed.FirstLocal = 10; Seed.FrameCount = 3;
	Seed.InitialT = .5f; Seed.InitialB = 1.f; Seed.InitialS = -.5f;
	CHECK(M.Acquire(8, 2, Seed));
	CHECK(M.Read().User.Values == Axes{} && M.Read().Physical.Values == Axes{});
	CHECK(M.Apply(8, 2, 9));
	CHECK(M.Read().User.Values == Axes{128, 255, -63});
	CHECK(M.Read().Physical.Values == Axes{16, 16, -16});
	CHECK(M.Read().BeforeSlew.Values == Axes{} && M.Read().SyncedAxes == Axes{});
	CHECK(M.Apply(8, 2, 9));
	CHECK(M.Read().Physical.Values == Axes{16, 16, -16});
	CHECK(M.Record(8, 2, 9));
	CHECK(M.Read().SyncedAxes == Axes{16, 16, -16});
	CHECK(M.Apply(8, 2, 10));
	CHECK(M.Read().Physical.Values == Axes{32, 32, -32});
	CHECK(M.Release(8, 2));

	Description Ordinary;
	Ordinary.FirstLocal = 1; Ordinary.FrameCount = 3;
	Ordinary.Keys = {{0, 1.f, 0.f, 1.f, true, false}};
	CHECK(M.Acquire(9, 3, Ordinary));
	CHECK(M.Apply(9, 3, 0) && M.Record(9, 3, 0));
	CHECK(M.Apply(9, 3, 1));
	CHECK(M.Read().User.CanMove && !M.Read().Physical.CanMove && !M.Read().EngineAllowsMovement);
	CHECK(M.Read().Physical.Values == Axes{16, 0, 16});
	CHECK(M.Read().BeforeSlew.Values == Axes{} && M.Read().SyncedAxes == Axes{});
	CHECK(M.Record(9, 3, 1));
	CHECK(M.Apply(9, 3, 2));
	CHECK(M.Read().User.CanMove); // Ordinary held payload is also requeued.
	CHECK(M.Read().Physical.Values == Axes{32, 0, 32});
	CHECK(M.Read().SyncedAxes == Axes{16, 0, 16}); // No early ordinary sync.
	CHECK(M.Release(9, 3));

	auto Bad = Seed; Bad.FirstLocal = 0; CHECK(Compile(Bad).Failure == Error::Clock);
	Bad = Seed; Bad.FirstLocal = std::numeric_limits<Frame>::max(); CHECK(Compile(Bad).Failure == Error::Clock);
	Bad = Seed; Bad.FirstLocal = std::numeric_limits<std::int32_t>::max(); CHECK(Compile(Bad).Failure == Error::Clock);
	Bad = Seed; Bad.FrameCount = 0; CHECK(Compile(Bad).Failure == Error::Clock);
	Bad = Seed; Bad.Warmup = 5; CHECK(Compile(Bad).Failure == Error::Policy);
	Bad = Seed; Bad.Policy = SeedPolicy::NeutralDuringWarmup; CHECK(Compile(Bad).Failure == Error::Policy);
	Bad = Seed; Bad.OverridesMobilityCountdown = true; CHECK(Compile(Bad).Failure == Error::Unsupported);
	Bad = Seed; Bad.HasUnmodelledActions = true; CHECK(Compile(Bad).Failure == Error::Unsupported);
	Bad = Seed; Bad.InitialT = std::numeric_limits<float>::quiet_NaN(); CHECK(Compile(Bad).Failure == Error::Value);
	Bad = Seed; Bad.Keys = {{0, 1.f, 0.f, 0.f, true, true}, {0, 0.f, 0.f, 0.f, false, false}};
	CHECK(Compile(Bad).Failure == Error::Duplicate);
	Bad.Keys = {{2, 1.f, 0.f, 0.f, true, true}}; CHECK(Compile(Bad).Failure == Error::Clock);
	Bad.Keys = {{0, std::numeric_limits<float>::infinity(), 0.f, 0.f, false, false}};
	CHECK(Compile(Bad).Failure == Error::Value);

	// W5a must NOT widen the accepted producer contract.
	Speed::Input::Testing::FWheeledTestProfile W1;
	W1.Identity = {Speed::Input::EProducerKind::Device, 1}; W1.FrameCount = 2;
	W1.InitialThrottle = .25f;
	CHECK(!Speed::Input::Testing::CompileWheeledTestProfile(W1).Producer);
	W1.InitialThrottle = 0; W1.ControlsCanMove = true;
	CHECK(!Speed::Input::Testing::CompileWheeledTestProfile(W1).Producer);
	W1.ControlsCanMove = false; W1.BypassWheeledSlew = true;
	CHECK(!Speed::Input::Testing::CompileWheeledTestProfile(W1).Producer);
	std::cout << "PASS WheeledPhaseContractProbe checks=" << Checks
		<< " scope=SPECIFICATION_MODEL_ONLY UE=not_run migrated=0\n";
}
