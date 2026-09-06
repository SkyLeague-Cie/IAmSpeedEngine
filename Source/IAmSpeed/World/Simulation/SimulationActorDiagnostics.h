#pragma once

#include "CoreMinimal.h"

#if !UE_BUILD_SHIPPING
class ISpeedComponent;
namespace Speed { class FSimulationWorld; }

namespace Speed::ActorDiagnostics
{
	enum class EPhase : uint8 { Prepare, Reset, Sweep, Integrate, Projection, Post, Count };
	enum class EFramePhase : uint8 { Initialize, Prepare, Core, Snapshot, Publish, Journal, Finalize,
		SnapshotBodies, SnapshotPairs, SnapshotHash, Count };
	struct FSample { double Milliseconds = 0; uint64 Queries = 0; uint64 Calls = 0; };
	extern thread_local bool bEnabled;
	/** Start an opt-in Fast-run diagnostic, with fixed storage for IDs 1..63. */
	void BeginRun();
	/** Emit aggregate per-actor phases once; overflow is explicitly reported. */
	void EndRun();
	/** Inclusive canonical-frame phases; SnapshotBodies/Pairs/Hash are children of Snapshot. */
	class FFrameScope
	{
	public:
		explicit FFrameScope(EFramePhase Phase) { if (bEnabled) Begin(Phase); }
		~FFrameScope() { if (Sample) Finish(); }
		void Next(EFramePhase Phase) { if (Sample) { Finish(); Begin(Phase); } }
	private:
		void Begin(EFramePhase Phase);
		void Finish();
		FSample* Sample = nullptr;
		double StartSeconds = 0;
	};
	class FScope
	{
	public:
		FScope(const FSimulationWorld& World, const ISpeedComponent& Component, EPhase Phase)
		{
			if (bEnabled) Begin(World, Component, Phase);
		}
		~FScope() { if (Sample) Finish(); }
	private:
		void Begin(const FSimulationWorld& World, const ISpeedComponent& Component, EPhase Phase);
		void Finish();
		FSample* Sample = nullptr;
		double StartSeconds = 0;
		uint32 StartQueries = 0;
	};
}
#define IAMSPEED_ACTOR_SCOPE(World, Component, Phase) \
	Speed::ActorDiagnostics::FScope ActorDiagnosticScope(World, Component, Speed::ActorDiagnostics::EPhase::Phase)
#define IAMSPEED_FRAME_SCOPE(Phase) \
	Speed::ActorDiagnostics::FFrameScope FrameDiagnosticScope(Speed::ActorDiagnostics::EFramePhase::Phase)
#define IAMSPEED_FRAME_PHASE(Phase) FrameDiagnosticScope.Next(Speed::ActorDiagnostics::EFramePhase::Phase)
#else
#define IAMSPEED_ACTOR_SCOPE(World, Component, Phase)
#define IAMSPEED_FRAME_SCOPE(Phase)
#define IAMSPEED_FRAME_PHASE(Phase)
#endif
