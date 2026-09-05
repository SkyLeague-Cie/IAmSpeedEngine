// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "IAmSpeed/SubBodies/SSubBody.h"
#include "IAmSpeed/World/Analytic/AnalyticWorldData.h"
#include "IAmSpeed/World/Collision/StaticCollisionWorld.h"
#include "IAmSpeed/World/Simulation/SimulationWorld.h"
#include "SpeedWorldSubsystem.generated.h"

class ISpeedComponent;
class USolidSubBody;
struct FCanonicalFrameContext;

enum class ECanonicalRunControlState : uint8
{
	Uncontrolled,
	WaitingForScenario,
	Ready,
	Complete,
};

struct FSpeedStepDiagnostics
{
	uint64 Serial = 0;
	unsigned int Frame = 0;
	int32 IterationCount = 0;
	int32 ComponentSweepCount = 0;
	int32 ResolvedEventCount = 0;
	uint32 StaticQueryCount = 0;
	uint32 LegacySweepCount = 0;
	uint32 AuthorityAttemptCount = 0;
	double StaticAuthorityMilliseconds = 0.0;
	double MaximumStaticAuthorityMilliseconds = 0.0;
	int32 MaximumIterationCount = 0;
	double PhysicalDeltaTimeMilliseconds = 0.0;
	double TotalMilliseconds = 0.0;
	double ResetMilliseconds = 0.0;
	double SweepMilliseconds = 0.0;
	double IntegrateMilliseconds = 0.0;
	double ResolveMilliseconds = 0.0;
	double ProjectionMilliseconds = 0.0;
	double PostMilliseconds = 0.0;
	bool bIterationLimitReached = false;
};

struct FPendingOp
{
    bool bAdd = false;
    ISpeedComponent* Comp = nullptr;
};

/**
 * USpeedWorldSubsystem : Subsystem responsible for managing and updating all speed components in the world.
 */
UCLASS()
class IAMSPEED_API USpeedWorldSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()
public:
	virtual void OnWorldBeginPlay(UWorld& InWorld) override;
	static bool AreUnilateralRollingPairsEnabled();
	static bool ShouldAcquireUnilateralRollingPair(float RelativeNormalSpeed);
    void RegisterSpeedComponent(ISpeedComponent* Comp);
    void UnregisterSpeedComponent(ISpeedComponent* Comp);
    void ApplyPendingOps();
	void RegisterDynamicContactPair(
		USolidSubBody& BodyA,
		USolidSubBody& BodyB,
		const FVector& ContactPoint,
		const FVector& NormalBToA,
		float ImpactRelativeNormalSpeed);
	void ActivatePendingRollingContactPairAtTOI(
		const USolidSubBody& BodyA,
		const USolidSubBody& BodyB,
		float RemainingDt);
	bool IsDynamicContactPairOwnedByRollingManifold(
		const USolidSubBody& BodyA,
		const USolidSubBody& BodyB) const;
	ERollingManifoldContactState GetRollingManifoldContactState(
		const USolidSubBody& Body) const;

	void PrepareCanonicalFrame(const FCanonicalFrameContext& Context);
	ECanonicalRunControlState GetCanonicalRunControlState();
	const Speed::Analytic::FAnalyticWorldData* GetAnalyticWorldData() const
	{
		return AnalyticWorldData.Get();
	}
	const Speed::IStaticCollisionWorld* GetStaticCollisionWorld() const
	{
		return SimulationWorld.GetStaticCollisionWorld();
	}
	const FSpeedStepDiagnostics& GetLastStepDiagnostics() const
	{
		return LastStepDiagnostics;
	}
	FSimulationSnapshot CaptureSimulationSnapshot(uint64 NumFrame, uint64 InputJournalHash);
	/** Restores one validated canonical snapshot without advancing simulation time. */
	bool RestoreSimulationSnapshot(
		const FSimulationSnapshot& Snapshot,
		uint64 ExpectedInputJournalHash);
	/** Resolves the address assigned to a registered simulation component. */
	uint64 GetSimulationStableId(const ISpeedComponent& Component);
	/** Applies the sealed input commands for a frame before component preparation. */
	bool ApplySimulationInputs(uint64 NumFrame, const Speed::SimulationBoundary::FInputJournal& Journal);
	TWeakObjectPtr<UPrimitiveComponent> FindAnalyticSourceComponent(
		uint64 SourceId) const;
    void Step(const float& Dt, const float& SimTime, const unsigned int& Frame);
private:
	Speed::FSimulationWorld SimulationWorld;
	uint64 StepSerial = 0;
	FSpeedStepDiagnostics LastStepDiagnostics;
	mutable bool bLoggedRollingOwnershipState = false;
	bool bLoggedRollingSolveState = false;
	// Runtime data is immutable after validation. Baked assets already return a
	// finalized shared payload, so retaining it avoids a deep copy and a second
	// rebuild of every compact-provider acceleration structure at world start.
	TSharedPtr<const Speed::Analytic::FAnalyticWorldData> AnalyticWorldData;
	// Runtime-only Unreal bridge. The analytical payload retains stable ids;
	// legacy consumers receive their source component without coupling the
	// generic query backend to UObjects.
	TMap<uint64, TWeakObjectPtr<UPrimitiveComponent>> AnalyticSourceComponents;
	uint8 AnalyticWorldBuildAttempt = 0;
	void BuildAnalyticWorldFromLoadedSources();

    void RebuildSortedIfNeeded();
	void AddComponent(ISpeedComponent& Comp);
	void RemoveComponent(ISpeedComponent& Comp);
	void SolveDynamicContactPairs(
		float Dt,
		uint64 PairKeyFilter = 0,
		bool bFilterByPairKey = false,
		bool bSolveFirstSeenFrame = false);
	void ProjectDynamicContactPairs();

    FCriticalSection PendingCS;
    TArray<FPendingOp> PendingOps;
};
