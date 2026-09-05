#include "SimulationActorDiagnostics.h"

#if !UE_BUILD_SHIPPING
#include "SimulationWorld.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "IAmSpeed/World/Analytic/StaticWorldQueryAudit.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTime.h"

namespace Speed::ActorDiagnostics
{
	static TAutoConsoleVariable<int32> CVarActorProfile(
		TEXT("p.IAmSpeed.Simulation.ActorProfile"), 0,
		TEXT("Opt-in Fast-run per-actor phase/query attribution; distorts timings, not physics."));
	thread_local bool bEnabled = false;
	struct FActor { FSample Phases[static_cast<int32>(EPhase::Count)]; int32 SubBodies = 0; };
	static thread_local FActor Actors[64];
	static thread_local FSample FramePhases[static_cast<int32>(EFramePhase::Count)];
	static thread_local uint64 OverflowScopes = 0;
	static_assert(sizeof(Actors) + sizeof(FramePhases) <= 16 * 1024, "Bound diagnostic memory independently of run length");

	void BeginRun()
	{
		bEnabled = CVarActorProfile.GetValueOnAnyThread() != 0;
		if (!bEnabled) return;
		for (FActor& Actor : Actors) Actor = FActor();
		for (FSample& Phase : FramePhases) Phase = FSample();
		OverflowScopes = 0;
	}
	void FFrameScope::Begin(EFramePhase Phase)
	{
		Sample = &FramePhases[static_cast<int32>(Phase)];
		StartSeconds = FPlatformTime::Seconds();
	}
	void FFrameScope::Finish()
	{
		Sample->Milliseconds += (FPlatformTime::Seconds() - StartSeconds) * 1000;
		++Sample->Calls;
	}
	void FScope::Begin(const FSimulationWorld& World, const ISpeedComponent& Component, EPhase Phase)
	{
		const uint64 Id = World.FindStableId(Component);
		if (Id == 0 || Id >= UE_ARRAY_COUNT(Actors)) { ++OverflowScopes; return; }
		FActor& Actor = Actors[Id];
		Actor.SubBodies = Component.GetSubBodies().Num();
		Sample = &Actor.Phases[static_cast<int32>(Phase)];
		StartQueries = Analytic::FStaticWorldQueryAudit::GetCurrentFrameCounters().QueryCount;
		StartSeconds = FPlatformTime::Seconds();
	}
	void FScope::Finish()
	{
		Sample->Milliseconds += (FPlatformTime::Seconds() - StartSeconds) * 1000;
		Sample->Queries += Analytic::FStaticWorldQueryAudit::GetCurrentFrameCounters().QueryCount - StartQueries;
		++Sample->Calls;
	}
	void EndRun()
	{
		if (!bEnabled) return;
		static const TCHAR* Names[] = { TEXT("Prepare"), TEXT("Reset"), TEXT("Sweep"), TEXT("Integrate"), TEXT("Projection"), TEXT("Post") };
		for (uint32 Id = 1; Id < UE_ARRAY_COUNT(Actors); ++Id)
		{
			const FActor& Actor = Actors[Id];
			for (uint32 Phase = 0; Phase < UE_ARRAY_COUNT(Names); ++Phase)
			{
				const FSample& S = Actor.Phases[Phase];
				if (!S.Calls) continue;
				UE_LOG(LogTemp, Display, TEXT("[SimulationActorProfile] Id=%u SubBodies=%d Phase=%s Calls=%llu TotalMs=%.9f Queries=%llu"),
					Id, Actor.SubBodies, Names[Phase], S.Calls, S.Milliseconds, S.Queries);
			}
		}
		UE_LOG(LogTemp, Display, TEXT("[SimulationActorProfileSummary] OverflowScopes=%llu"), OverflowScopes);
		static const TCHAR* FrameNames[] = { TEXT("Initialize"), TEXT("Prepare"), TEXT("Core"), TEXT("Snapshot"),
			TEXT("Publish"), TEXT("Journal"), TEXT("Finalize"), TEXT("SnapshotBodies"), TEXT("SnapshotPairs"), TEXT("SnapshotHash") };
		static_assert(UE_ARRAY_COUNT(FrameNames) == UE_ARRAY_COUNT(FramePhases));
		for (uint32 Phase = 0; Phase < UE_ARRAY_COUNT(FrameNames); ++Phase)
		{
			const FSample& Sample = FramePhases[Phase];
			if (!Sample.Calls) continue;
			UE_LOG(LogTemp, Display, TEXT("[SimulationFrameProfile] Phase=%s Calls=%llu TotalMs=%.9f"),
				FrameNames[Phase], Sample.Calls, Sample.Milliseconds);
		}
		bEnabled = false;
	}
}
#endif
