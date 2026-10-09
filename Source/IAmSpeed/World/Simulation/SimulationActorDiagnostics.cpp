#include "SimulationActorDiagnostics.h"

#if !UE_BUILD_SHIPPING
#include "SimulationWorld.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "IAmSpeed/World/Analytic/StaticWorldQueryAudit.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTime.h"
#include "HAL/PlatformTLS.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "IAmSpeed/Components/SpeedMovementComponent.h"
#endif

namespace Speed::ActorDiagnostics
{
	static TAutoConsoleVariable<int32> CVarActorProfile(
		TEXT("p.IAmSpeed.Simulation.ActorProfile"), 0,
		TEXT("Opt-in Fast-run per-actor phase/query attribution; distorts timings, not physics."));
	thread_local bool bEnabled = false;
	struct FActor { FSample Phases[static_cast<int32>(EPhase::Count)]; int32 SubBodies = 0; };
	struct FDiagnosticContext
	{
		FActor Actors[64];
		FSample FramePhases[static_cast<int32>(EFramePhase::Count)];
		uint64 OverflowScopes = 0;
		bool bCollect = false;
	};
	static thread_local FDiagnosticContext DefaultContext;
	static thread_local FDiagnosticContext* CurrentContext = &DefaultContext;
	static_assert(sizeof(FDiagnosticContext) <= 16 * 1024, "Bound each diagnostic context independently of run length");

	FScopedCapture::FScopedCapture(bool bCollect)
		: Captured(MakeUnique<FDiagnosticContext>()), Previous(CurrentContext),
		  bPreviousEnabled(bEnabled), ThreadId(FPlatformTLS::GetCurrentThreadId())
	{
		Captured->bCollect = bCollect;
		CurrentContext = Captured.Get();
		bEnabled = bCollect;
	}
	FScopedCapture::~FScopedCapture()
	{
		check(ThreadId == FPlatformTLS::GetCurrentThreadId() && CurrentContext == Captured.Get());
		CurrentContext = Previous;
		bEnabled = bPreviousEnabled;
	}
	bool FScopedCapture::ReadFrameSample(EFramePhase Phase, FSample& Out) const
	{
		check(ThreadId == FPlatformTLS::GetCurrentThreadId());
		const uint32 Index = static_cast<uint32>(Phase);
		if (Index >= static_cast<uint32>(EFramePhase::Count)) return false;
		Out = Captured->FramePhases[Index];
		return true;
	}
	bool FScopedCapture::ReadActorSample(uint64 Id, EPhase Phase, FSample& Out, int32& OutSubBodies) const
	{
		check(ThreadId == FPlatformTLS::GetCurrentThreadId());
		const uint32 Index = static_cast<uint32>(Phase);
		if (Id == 0 || Id >= UE_ARRAY_COUNT(Captured->Actors) || Index >= static_cast<uint32>(EPhase::Count)) return false;
		Out = Captured->Actors[Id].Phases[Index];
		OutSubBodies = Captured->Actors[Id].SubBodies;
		return true;
	}
	uint64 FScopedCapture::GetOverflowScopes() const
	{
		check(ThreadId == FPlatformTLS::GetCurrentThreadId());
		return Captured->OverflowScopes;
	}

	void BeginRun()
	{
		bEnabled = CurrentContext == &DefaultContext ? CVarActorProfile.GetValueOnAnyThread() != 0 : CurrentContext->bCollect;
		if (!bEnabled) return;
		for (FActor& Actor : CurrentContext->Actors) Actor = FActor();
		for (FSample& Phase : CurrentContext->FramePhases) Phase = FSample();
		CurrentContext->OverflowScopes = 0;
	}
	void FFrameScope::Begin(EFramePhase Phase)
	{
		Sample = &CurrentContext->FramePhases[static_cast<int32>(Phase)];
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
		if (Id == 0 || Id >= UE_ARRAY_COUNT(CurrentContext->Actors)) { ++CurrentContext->OverflowScopes; return; }
		FActor& Actor = CurrentContext->Actors[Id];
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
		for (uint32 Id = 1; Id < UE_ARRAY_COUNT(CurrentContext->Actors); ++Id)
		{
			const FActor& Actor = CurrentContext->Actors[Id];
			for (uint32 Phase = 0; Phase < UE_ARRAY_COUNT(Names); ++Phase)
			{
				const FSample& S = Actor.Phases[Phase];
				if (!S.Calls) continue;
				UE_LOG(LogTemp, Display, TEXT("[SimulationActorProfile] Id=%u SubBodies=%d Phase=%s Calls=%llu TotalMs=%.9f Queries=%llu"),
					Id, Actor.SubBodies, Names[Phase], S.Calls, S.Milliseconds, S.Queries);
			}
		}
		UE_LOG(LogTemp, Display, TEXT("[SimulationActorProfileSummary] OverflowScopes=%llu"), CurrentContext->OverflowScopes);
		static const TCHAR* FrameNames[] = { TEXT("Initialize"), TEXT("Prepare"), TEXT("Core"), TEXT("Snapshot"),
			TEXT("Publish"), TEXT("Journal"), TEXT("Finalize"), TEXT("SnapshotBodies"), TEXT("SnapshotPairs"), TEXT("SnapshotHash") };
		static_assert(UE_ARRAY_COUNT(FrameNames) == UE_ARRAY_COUNT(CurrentContext->FramePhases));
		for (uint32 Phase = 0; Phase < UE_ARRAY_COUNT(FrameNames); ++Phase)
		{
			const FSample& Sample = CurrentContext->FramePhases[Phase];
			if (!Sample.Calls) continue;
			UE_LOG(LogTemp, Display, TEXT("[SimulationFrameProfile] Phase=%s Calls=%llu TotalMs=%.9f"),
				FrameNames[Phase], Sample.Calls, Sample.Milliseconds);
		}
		bEnabled = false;
	}
#if WITH_DEV_AUTOMATION_TESTS
	IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScopedActorCaptureTest,
		"IAmSpeed.Simulation.ActorDiagnostics.ScopedCapture",
		EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
	bool FScopedActorCaptureTest::RunTest(const FString&)
	{
		// Preserve the real caller even when this test runs in an existing diagnostic.
		FScopedCapture TestContext;
		FSimulationWorld World;
		USpeedMovementComponent* Component = NewObject<USpeedMovementComponent>();
		if (!TestNotNull(TEXT("diagnostic adapter"), Component) ||
			!TestTrue(TEXT("adapter registered"), World.AddAdapter(*Component))) return false;
		TestEqual(TEXT("adapter stable ID"), World.FindStableId(*Component), uint64(1));
		FDiagnosticContext* const Caller = CurrentContext;
		FSample* const ActorSample = &Caller->Actors[1].Phases[0];
		ActorSample->Calls = 7;
		ActorSample->Queries = 11;
		ActorSample->Milliseconds = 13;
		Caller->Actors[1].SubBodies = Component->GetSubBodies().Num();
		Caller->OverflowScopes = 17;
		{
			FFrameScope CallerFrame(EFramePhase::Snapshot);
			FScope CallerActor(World, *Component, EPhase::Post);
			{
				FScopedCapture Private;
				TestTrue(TEXT("private context has a distinct address"), CurrentContext != Caller);
				FSample Sample;
				int32 Bodies = -1;
				TestTrue(TEXT("fresh actor readable"), Private.ReadActorSample(1, EPhase::Prepare, Sample, Bodies));
				TestEqual(TEXT("private actor starts empty"), Sample.Calls, uint64(0));
				TestEqual(TEXT("private bodies start empty"), Bodies, 0);
				TestEqual(TEXT("private overflow starts empty"), Private.GetOverflowScopes(), uint64(0));
				CurrentContext->Actors[1].Phases[0].Calls = 99;
				CurrentContext->OverflowScopes = 99;
				BeginRun(); // scoped opt-in is independent of the caller's CVar
				TestTrue(TEXT("BeginRun retains explicit scoped opt-in"), bEnabled);
				Private.ReadActorSample(1, EPhase::Prepare, Sample, Bodies);
				TestEqual(TEXT("BeginRun clears only private actor bucket"), Sample.Calls, uint64(0));
				TestEqual(TEXT("BeginRun clears only private overflow"), Private.GetOverflowScopes(), uint64(0));
				{
					FFrameScope PrivateFrame(EFramePhase::Snapshot);
					FScope PrivateActor(World, *Component, EPhase::Post);
					FDiagnosticContext* const PrivateAddress = CurrentContext;
					{
						FScopedCapture Nested;
						{ FFrameScope NestedFrame(EFramePhase::Snapshot); FScope NestedActor(World, *Component, EPhase::Post); }
						TestTrue(TEXT("nested result readable"), Nested.ReadFrameSample(EFramePhase::Snapshot, Sample));
						TestEqual(TEXT("nested frame counted once"), Sample.Calls, uint64(1));
						Nested.ReadActorSample(1, EPhase::Post, Sample, Bodies);
						TestEqual(TEXT("nested actor counted once"), Sample.Calls, uint64(1));
						bEnabled = false;
					}
					TestTrue(TEXT("private address restored"), CurrentContext == PrivateAddress);
					TestTrue(TEXT("private enabled restored"), bEnabled);
				}
				TestTrue(TEXT("private frame result readable"), Private.ReadFrameSample(EFramePhase::Snapshot, Sample));
				TestEqual(TEXT("nested sample excluded from private bucket"), Sample.Calls, uint64(1));
				Private.ReadActorSample(1, EPhase::Post, Sample, Bodies);
				TestEqual(TEXT("nested actor excluded from private bucket"), Sample.Calls, uint64(1));
				const FSample Before = Sample;
				Bodies = 31;
				TestFalse(TEXT("invalid frame rejected"), Private.ReadFrameSample(EFramePhase::Count, Sample));
				TestFalse(TEXT("zero ID rejected"), Private.ReadActorSample(0, EPhase::Prepare, Sample, Bodies));
				TestFalse(TEXT("overflow ID rejected"), Private.ReadActorSample(64, EPhase::Prepare, Sample, Bodies));
				TestFalse(TEXT("invalid actor phase rejected"), Private.ReadActorSample(1, EPhase::Count, Sample, Bodies));
				TestEqual(TEXT("failed read preserves calls"), Sample.Calls, Before.Calls);
				TestEqual(TEXT("failed read preserves queries"), Sample.Queries, Before.Queries);
				TestEqual(TEXT("failed read preserves milliseconds"), Sample.Milliseconds, Before.Milliseconds);
				TestEqual(TEXT("failed read preserves bodies"), Bodies, 31);
				EndRun();
				TestFalse(TEXT("private EndRun disables private capture"), bEnabled);
			}
			TestTrue(TEXT("caller address restored before its scope finishes"), CurrentContext == Caller);
			TestTrue(TEXT("caller enabled restored after private EndRun"), bEnabled);
		}
		TestEqual(TEXT("live caller scope survived nesting"), Caller->FramePhases[static_cast<int32>(EFramePhase::Snapshot)].Calls, uint64(1));
		TestEqual(TEXT("live caller actor scope survived nesting"), Caller->Actors[1].Phases[static_cast<int32>(EPhase::Post)].Calls, uint64(1));
		const auto CheckCaller = [&]()
		{
			TestTrue(TEXT("caller context restored"), CurrentContext == Caller);
			TestEqual(TEXT("caller frame calls preserved"), Caller->FramePhases[static_cast<int32>(EFramePhase::Snapshot)].Calls, uint64(1));
			TestEqual(TEXT("caller post calls preserved"), Caller->Actors[1].Phases[static_cast<int32>(EPhase::Post)].Calls, uint64(1));
			TestTrue(TEXT("caller actor pointer unchanged"), ActorSample == &CurrentContext->Actors[1].Phases[0]);
			TestEqual(TEXT("caller actor calls preserved"), ActorSample->Calls, uint64(7));
			TestEqual(TEXT("caller actor queries preserved"), ActorSample->Queries, uint64(11));
			TestEqual(TEXT("caller actor time preserved"), ActorSample->Milliseconds, 13.0);
			TestEqual(TEXT("caller actor subbodies preserved"), Caller->Actors[1].SubBodies, Component->GetSubBodies().Num());
			TestEqual(TEXT("caller overflow preserved"), Caller->OverflowScopes, uint64(17));
			TestTrue(TEXT("caller enabled preserved"), bEnabled);
		};
		CheckCaller();
		const auto EarlyFailure = [&]()
		{
			FScopedCapture Failed;
			CurrentContext->Actors[1].Phases[0].Calls = 99;
			CurrentContext->OverflowScopes = 99;
			bEnabled = false;
			return false;
		};
		TestFalse(TEXT("failure return exercised"), EarlyFailure());
		CheckCaller();
#if defined(__cpp_exceptions) || defined(_CPPUNWIND)
		bool bCaught = false;
		try
		{
			FScopedCapture Failed;
			CurrentContext->OverflowScopes = 99;
			bEnabled = false;
			throw 17;
		}
		catch (int Value) { bCaught = Value == 17; CheckCaller(); }
		TestTrue(TEXT("exception unwind exercised"), bCaught);
#endif
		{
			FScopedCapture Disabled(false);
			BeginRun();
			TestFalse(TEXT("BeginRun retains scoped disabled state"), bEnabled);
			{ FFrameScope DisabledFrame(EFramePhase::Snapshot); }
			FSample Sample;
			TestTrue(TEXT("disabled result readable"), Disabled.ReadFrameSample(EFramePhase::Snapshot, Sample));
			TestEqual(TEXT("disabled frame not counted"), Sample.Calls, uint64(0));
		}
		CheckCaller();
		return true;
	}
#endif

}
#endif
